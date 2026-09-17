// SPDX-License-Identifier: GPL-2.0
/*
 * cpu_input_boost.c - Boost CPU frequency briefly on touch/key input
 *                      and on screen wake, to reduce perceived UI latency.
 *
 * Adapted for KawaKernel-A217X (Exynos 850 / Exynos 3830, Galaxy A21s).
 *
 * Original implementation:
 * Copyright (C) 2018-2019 Sultan Alsawaf <sultan@kerneltoast.com>.
 *
 * Adaptation notes (why this differs from upstream cpu_input_boost.c):
 *  - Upstream uses msm_drm_notify (Qualcomm/MSM display notifier), which
 *    does not exist on this Exynos tree. Screen on/off is instead sourced
 *    from state_notifier (drivers/soc/samsung/state_notifier.c), the same
 *    mechanism this kernel's own Dynamic Fsync driver (fs/dyn_sync_cntrl.c)
 *    already relies on to know screen state. That chain is fed by real
 *    hardware events from the DECON display driver
 *    (drivers/video/fbdev/exynos/dpu30/decon_core.c) via
 *    set_power_suspend_state_panel_hook().
 *  - Upstream splits boost frequency by cpu_lp_mask/cpu_perf_mask
 *    (Qualcomm big.LITTLE cluster masks), which don't exist here. Instead
 *    we boost policy->min to policy->max for whichever cpufreq policy is
 *    being adjusted, which applies correctly per-cluster without needing
 *    to hardcode either cluster's frequency table.
 *  - Upstream uses kthread_run_perf_critical(), a helper not present in
 *    this kernel; replaced with the standard kthread_run().
 */

#define pr_fmt(fmt) "cpu_input_boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <uapi/linux/sched/types.h>
#include <linux/slab.h>
#include <linux/state_notifier.h>
#include <linux/battery_saver.h>

/*
 * Khai báo tối thiểu cho API khoá tần số GPU (gpex_clock), thay vì include
 * trực tiếp gpex_clock.h -- header đó kéo theo mali_kbase.h và một chuỗi
 * include nội bộ chỉ được set qua ccflags riêng của
 * drivers/gpu/arm/exynos/Kbuild, không có sẵn khi biên dịch từ
 * drivers/cpufreq/. Enum và chữ ký hàm dưới đây copy khớp 1:1 với
 * drivers/gpu/arm/exynos/include/gpex_clock.h (đã đối chiếu trực tiếp
 * với source, không đoán). Cả 2 hàm đều non-static, build-in (=y, không
 * phải module), nên extern link bình thường là đủ, không cần
 * EXPORT_SYMBOL.
 */
typedef enum {
	GPU_CLOCK_MAX_LOCK = 0,
	GPU_CLOCK_MIN_LOCK,
	GPU_CLOCK_MAX_UNLOCK,
	GPU_CLOCK_MIN_UNLOCK,
} gpex_clock_lock_cmd_t;

typedef enum {
	TMU_LOCK = 0,
	SYSFS_LOCK,
	PMQOS_LOCK,
	CLBOOST_LOCK,
	NUMBER_LOCK
} gpex_clock_lock_type_t;

extern int gpex_clock_lock_clock(gpex_clock_lock_cmd_t lock_command,
				  gpex_clock_lock_type_t lock_type, int clock);
extern int gpex_clock_get_max_clock(void);

/* Available bits for boost state */
#define SCREEN_OFF	BIT(0)
#define INPUT_BOOST	BIT(1)
#define MAX_BOOST	BIT(2)

struct boost_drv {
	struct delayed_work input_unboost;
	struct delayed_work max_unboost;
	struct notifier_block cpu_notif;
	struct notifier_block state_notif;
	wait_queue_head_t boost_waitq;
	atomic64_t max_boost_expires;
	atomic_t state;
};

static struct boost_drv *boost_drv_g __read_mostly;

static u32 get_boost_state(struct boost_drv *b)
{
	return atomic_read(&b->state);
}

static void set_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_or(state, &b->state);
}

static void clear_boost_bit(struct boost_drv *b, u32 state)
{
	atomic_andnot(state, &b->state);
}

static void update_online_cpu_policy(void)
{
	unsigned int cpu;

	/*
	 * Unlike upstream, we don't know the cluster layout via
	 * cpu_lp_mask/cpu_perf_mask, so refresh every online CPU's policy.
	 * cpufreq_update_policy() is a no-op for CPUs that share a policy
	 * already updated, so this is safe (if slightly redundant) on a
	 * 2-cluster layout like exynos3830's domain0/domain1.
	 */
	get_online_cpus();
	for_each_online_cpu(cpu)
		cpufreq_update_policy(cpu);
	put_online_cpus();
}

/*
 * GPU dùng lock_type CLBOOST_LOCK riêng (không đụng TMU_LOCK của thermal
 * hay SYSFS_LOCK người dùng có thể tự set) -- gpex_clock_lock_clock() tự
 * kết hợp an toàn giữa các lock_type bằng MIN/MAX nội bộ (đã đọc source
 * xác nhận). Chỉ engage/disengage đúng 1 lần dựa trên trạng thái tổng
 * hợp INPUT_BOOST|MAX_BOOST, giống hệt cách cpu_notifier_cb xử lý CPU,
 * để tránh 1 trong 2 nguồn boost tắt sớm làm rớt boost của nguồn kia
 * (cả input và wake-boost dùng chung 1 lock_type).
 */
static void gpu_boost_engage(void)
{
	int max_clk = gpex_clock_get_max_clock();

	if (max_clk > 0)
		gpex_clock_lock_clock(GPU_CLOCK_MIN_LOCK, CLBOOST_LOCK, max_clk);
}

static void gpu_boost_disengage(struct boost_drv *b)
{
	if (!(get_boost_state(b) & (INPUT_BOOST | MAX_BOOST)))
		gpex_clock_lock_clock(GPU_CLOCK_MIN_UNLOCK, CLBOOST_LOCK, 0);
}

static void __cpu_input_boost_kick(struct boost_drv *b)
{
	if (get_boost_state(b) & SCREEN_OFF)
		return;

	if (is_battery_saver_on())
		return;

	set_boost_bit(b, INPUT_BOOST);
	gpu_boost_engage();
	wake_up(&b->boost_waitq);
	mod_delayed_work(system_unbound_wq, &b->input_unboost,
			  msecs_to_jiffies(CONFIG_INPUT_BOOST_DURATION_MS));
}

void cpu_input_boost_kick(void)
{
	struct boost_drv *b = boost_drv_g;

	if (!b)
		return;

	__cpu_input_boost_kick(b);
}

static void __cpu_input_boost_kick_max(struct boost_drv *b,
					unsigned int duration_ms)
{
	unsigned long boost_jiffies = msecs_to_jiffies(duration_ms);
	unsigned long curr_expires, new_expires;

	if (get_boost_state(b) & SCREEN_OFF)
		return;

	if (is_battery_saver_on())
		return;

	do {
		curr_expires = atomic64_read(&b->max_boost_expires);
		new_expires = jiffies + boost_jiffies;

		/* Skip this boost if there's a longer boost in effect */
		if (time_after(curr_expires, new_expires))
			return;
	} while (atomic64_cmpxchg(&b->max_boost_expires, curr_expires,
				   new_expires) != curr_expires);

	set_boost_bit(b, MAX_BOOST);
	gpu_boost_engage();
	wake_up(&b->boost_waitq);
	mod_delayed_work(system_unbound_wq, &b->max_unboost, boost_jiffies);
}

void cpu_input_boost_kick_max(unsigned int duration_ms)
{
	struct boost_drv *b = boost_drv_g;

	if (!b)
		return;

	__cpu_input_boost_kick_max(b, duration_ms);
}

static void input_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					    typeof(*b), input_unboost);

	clear_boost_bit(b, INPUT_BOOST);
	gpu_boost_disengage(b);
	wake_up(&b->boost_waitq);
}

static void max_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					    typeof(*b), max_unboost);

	clear_boost_bit(b, MAX_BOOST);
	gpu_boost_disengage(b);
	wake_up(&b->boost_waitq);
}

static int cpu_boost_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	struct boost_drv *b = data;
	u32 old_state = 0;

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);

	while (!kthread_should_stop()) {
		u32 curr_state;

		wait_event_interruptible(b->boost_waitq,
			(curr_state = get_boost_state(b)) != old_state ||
			kthread_should_stop());

		old_state = curr_state;
		update_online_cpu_policy();
	}

	return 0;
}

static int cpu_notifier_cb(struct notifier_block *nb,
			    unsigned long action, void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), cpu_notif);
	struct cpufreq_policy *policy = data;
	u32 state;

	if (action != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	state = get_boost_state(b);

	/* Unboost when the screen is off */
	if (state & SCREEN_OFF) {
		policy->min = policy->cpuinfo.min_freq;
		return NOTIFY_OK;
	}

	/* Boost this policy's cluster to its own max for MAX_BOOST or
	 * INPUT_BOOST alike -- see the file header for why we don't split
	 * by a per-cluster target frequency the way upstream does. */
	if (state & (MAX_BOOST | INPUT_BOOST))
		policy->min = policy->max;
	else
		policy->min = policy->cpuinfo.min_freq;

	return NOTIFY_OK;
}

static int state_notifier_cb(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), state_notif);

	switch (action) {
	case STATE_NOTIFIER_ACTIVE:
		/* Screen just turned on: boost briefly to smooth the
		 * wake-up + unlock animation, same idea as upstream's
		 * wake boost on MSM_DRM_BLANK_UNBLANK. */
		clear_boost_bit(b, SCREEN_OFF);
		__cpu_input_boost_kick_max(b, CONFIG_WAKE_BOOST_DURATION_MS);
		break;
	case STATE_NOTIFIER_SUSPEND:
		set_boost_bit(b, SCREEN_OFF);
		wake_up(&b->boost_waitq);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static void cpu_input_boost_input_event(struct input_handle *handle,
					 unsigned int type, unsigned int code,
					 int value)
{
	struct boost_drv *b = handle->handler->private;

	__cpu_input_boost_kick(b);
}

static int cpu_input_boost_input_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_input_boost_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto unregister_handle;

	return 0;

unregister_handle:
	input_unregister_handle(handle);
free_handle:
	kfree(handle);
	return ret;
}

static void cpu_input_boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_input_boost_ids[] = {
	/* Multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) }
	},
	/* Keypad / hardware buttons (power, volume, etc) */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) }
	},
	{ }
};

static struct input_handler cpu_input_boost_input_handler = {
	.event		= cpu_input_boost_input_event,
	.connect	= cpu_input_boost_input_connect,
	.disconnect	= cpu_input_boost_input_disconnect,
	.name		= "cpu_input_boost_handler",
	.id_table	= cpu_input_boost_ids
};

static int __init cpu_input_boost_init(void)
{
	struct task_struct *boost_thread;
	struct boost_drv *b;
	int ret;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	INIT_DELAYED_WORK(&b->input_unboost, input_unboost_worker);
	INIT_DELAYED_WORK(&b->max_unboost, max_unboost_worker);
	init_waitqueue_head(&b->boost_waitq);
	atomic64_set(&b->max_boost_expires, 0);
	atomic_set(&b->state, 0);

	b->cpu_notif.notifier_call = cpu_notifier_cb;
	ret = cpufreq_register_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		pr_err("Failed to register cpufreq notifier, err: %d\n", ret);
		goto free_b;
	}

	cpu_input_boost_input_handler.private = b;
	ret = input_register_handler(&cpu_input_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto unregister_cpu_notif;
	}

	b->state_notif.notifier_call = state_notifier_cb;
	ret = state_register_client(&b->state_notif);
	if (ret) {
		pr_err("Failed to register state notifier, err: %d\n", ret);
		goto unregister_handler;
	}

	boost_thread = kthread_run(cpu_boost_thread, b, "cpu_boostd");
	if (IS_ERR(boost_thread)) {
		ret = PTR_ERR(boost_thread);
		pr_err("Failed to start CPU boost thread, err: %d\n", ret);
		goto unregister_state_notif;
	}

	boost_drv_g = b;
	return 0;

unregister_state_notif:
	state_unregister_client(&b->state_notif);
unregister_handler:
	input_unregister_handler(&cpu_input_boost_input_handler);
unregister_cpu_notif:
	cpufreq_unregister_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
free_b:
	kfree(b);
	return ret;
}
late_initcall(cpu_input_boost_init);
