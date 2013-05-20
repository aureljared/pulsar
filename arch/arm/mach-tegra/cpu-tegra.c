/*
 * arch/arm/mach-tegra/cpu-tegra.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Based on arch/arm/plat-omap/cpu-omap.c, (C) 2005 Nokia Corporation
 *
 * Copyright (C) 2010-2012 NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/pm_qos_params.h>
#include <linux/earlysuspend.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/cpu_debug.h>

#include <asm/system.h>

#include <mach/clk.h>
#include <mach/edp.h>
#include "board.h"
#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"
#include "pm.h"
#include "tegra_pmqos.h"

/* Symbol to store resume resume */
extern unsigned long long wake_reason_resume;
static spinlock_t user_cap_lock;
struct work_struct htc_suspend_resume_work;

/* tegra throttling and edp governors require frequencies in the table
   to be in ascending order */
static struct cpufreq_frequency_table *freq_table;

static struct clk *cpu_clk;
static struct clk *cpu_g_clk;
static struct clk *emc_clk;

static unsigned long policy_max_speed[CONFIG_NR_CPUS];
static unsigned long target_cpu_speed[CONFIG_NR_CPUS];
static DEFINE_MUTEX(tegra_cpu_lock);
static bool is_suspended;
static int suspend_index;

#ifdef CONFIG_TEGRA3_VARIANT_CPU_OVERCLOCK
int enable_oc = 0;
#endif

/* maximum cpu freq */
unsigned int tegra_cpu_freq_max(unsigned int cpu)
{
#ifdef CONFIG_TEGRA3_VARIANT_CPU_OVERCLOCK
	if (enable_oc)
		return T3_CPU_FREQ_MAX_OC;
#endif	
	if (cpu==0)
		return T3_CPU_FREQ_MAX_0;
	return T3_CPU_FREQ_MAX;
}

// maxwen: see tegra_cpu_init
// values can be changed in sysfs interface of cpufreq
// for scaling_max_freq_limit
static inline unsigned int get_cpu_freq_limit(unsigned int cpu)
{
	BUG_ON(cpu > 3);
	if(tegra_pmqos_cpu_freq_limits[cpu]!=0){
		return tegra_pmqos_cpu_freq_limits[cpu];
	}
	return tegra_cpu_freq_max(cpu);
}

static inline unsigned int get_cpu_freq_limit_min(unsigned int cpu)
{
	BUG_ON(cpu > 3);
	if(tegra_pmqos_cpu_freq_limits_min[cpu]!=0){
		return tegra_pmqos_cpu_freq_limits_min[cpu];
	}
	return T3_CPU_MIN_FREQ;
}

unsigned int tegra_get_suspend_boost_freq(void)
{
	return min((unsigned int)T3_CPU_FREQ_BOOST, get_cpu_freq_limit(0));
}

static bool force_policy_max;

static int force_policy_max_set(const char *arg, const struct kernel_param *kp)
{
	int ret;
	bool old_policy = force_policy_max;

	mutex_lock(&tegra_cpu_lock);

	ret = param_set_bool(arg, kp);
	if ((ret == 0) && (old_policy != force_policy_max))
		tegra_cpu_set_speed_cap(NULL);

	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int force_policy_max_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops policy_ops = {
	.set = force_policy_max_set,
	.get = force_policy_max_get,
};
module_param_cb(force_policy_max, &policy_ops, &force_policy_max, 0644);

static int suspend_cap_freq_set(const char *arg, const struct kernel_param *kp)
{
	int tmp;
	
	if (1 != sscanf(arg, "%d", &tmp))
		return -EINVAL;

    suspend_cap_freq = tmp;
    pr_info("suspend_cap_freq %d\n", suspend_cap_freq);
	return 0;
}

static int suspend_cap_freq_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_uint(buffer, kp);
}

static struct kernel_param_ops suspend_cap_freq_ops = {
	.set = suspend_cap_freq_set,
	.get = suspend_cap_freq_get,
};
module_param_cb(suspend_cap_freq, &suspend_cap_freq_ops, &suspend_cap_freq, 0644);

static int suspend_cap_cpu_num_set(const char *arg, const struct kernel_param *kp)
{
	int tmp;
	
	if (1 != sscanf(arg, "%d", &tmp))
		return -EINVAL;

	if (tmp == PM_QOS_MAX_ONLINE_CPUS_DEFAULT_VALUE)
		tmp = CONFIG_NR_CPUS;

	if (tmp < 1 || tmp > CONFIG_NR_CPUS)
		return -EINVAL;
			
    suspend_cap_cpu_num = tmp;
    pr_info("suspend_cap_cpu_num %d\n", suspend_cap_cpu_num);
	return 0;
}

static int suspend_cap_cpu_num_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_uint(buffer, kp);
}

static struct kernel_param_ops suspend_cap_cpu_num_ops = {
	.set = suspend_cap_cpu_num_set,
	.get = suspend_cap_cpu_num_get,
};
module_param_cb(suspend_cap_cpu_num, &suspend_cap_cpu_num_ops, &suspend_cap_cpu_num, 0644);

static unsigned int cpu_user_cap = 0;

static inline void _cpu_user_cap_set_locked(void)
{
#ifndef CONFIG_TEGRA_CPU_CAP_EXACT_FREQ
	if (cpu_user_cap != 0) {
		int i;
		for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
			if (freq_table[i].frequency > cpu_user_cap)
				break;
		}
		i = (i == 0) ? 0 : i - 1;
		cpu_user_cap = freq_table[i].frequency;
	}
#endif
	tegra_cpu_set_speed_cap(NULL);
}

void tegra_cpu_user_cap_set(unsigned int speed_khz)
{
	mutex_lock(&tegra_cpu_lock);

	cpu_user_cap = speed_khz;
	_cpu_user_cap_set_locked();

	mutex_unlock(&tegra_cpu_lock);
}

static int cpu_user_cap_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);

	ret = param_set_uint(arg, kp);
	if (ret == 0)
		_cpu_user_cap_set_locked();

	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int cpu_user_cap_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_uint(buffer, kp);
}

static struct kernel_param_ops cpu_user_cap_ops = {
	.set = cpu_user_cap_set,
	.get = cpu_user_cap_get,
};
module_param_cb(cpu_user_cap, &cpu_user_cap_ops, &cpu_user_cap, 0644);

static unsigned int user_cap_speed(unsigned int requested_speed)
{
	if ((cpu_user_cap) && (requested_speed > cpu_user_cap))
		return cpu_user_cap;
	return requested_speed;
}

=======
#define powersave_speed(requested_speed) (requested_speed)

/* disable edp limitations */
static unsigned int no_edp_limit = 0;
module_param(no_edp_limit, uint, 0644);

/* disable thermal throttling limitations */
unsigned int no_thermal_throttle_limit = 0;
module_param(no_thermal_throttle_limit, uint, 0644);

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE

static ssize_t show_throttle(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", tegra_is_throttling());
}

cpufreq_freq_attr_ro(throttle);
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */

#ifdef CONFIG_TEGRA_EDP_LIMITS

static const struct tegra_edp_limits *cpu_edp_limits;
static int cpu_edp_limits_size;

static const unsigned int *system_edp_limits;
static bool system_edp_alarm;

static int edp_thermal_index;
static cpumask_t edp_cpumask;
static unsigned int edp_limit;

unsigned int tegra_get_edp_limit(void)
{
	return edp_limit;
}

static unsigned int edp_predict_limit(unsigned int cpus)
{
	unsigned int limit = 0;

	BUG_ON(cpus == 0);
	if (cpu_edp_limits) {
		BUG_ON(edp_thermal_index >= cpu_edp_limits_size);
#ifdef CONFIG_TEGRA3_VARIANT_CPU_OVERCLOCK
        if(enable_oc)
		    limit = cpu_edp_limits[edp_thermal_index].freq_limits_oc[cpus - 1];
        else
		    limit = cpu_edp_limits[edp_thermal_index].freq_limits[cpus - 1];
#else
		limit = cpu_edp_limits[edp_thermal_index].freq_limits[cpus - 1];
#endif
	}
	if (system_edp_limits && system_edp_alarm)
		limit = min(limit, system_edp_limits[cpus - 1]);

	return limit;
}

static void edp_update_limit(void)
{
	unsigned int limit = edp_predict_limit(cpumask_weight(&edp_cpumask));

#ifdef CONFIG_TEGRA_EDP_EXACT_FREQ
	edp_limit = limit;
#else
	unsigned int i;
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (freq_table[i].frequency > limit) {
			break;
		}
	}
	BUG_ON(i == 0);	/* min freq above the limit or table empty */
	edp_limit = freq_table[i-1].frequency;
#endif
}

static unsigned int edp_governor_speed(unsigned int requested_speed)
{
    /* ignore EDP (regulator max output) limitation */
    if (unlikely(no_edp_limit))
        return requested_speed;

	if ((!edp_limit) || (requested_speed <= edp_limit))
		return requested_speed;
	else
		return edp_limit;
}

int tegra_edp_update_thermal_zone(int temperature)
{
	int i;
	int ret = 0;
	int nlimits = cpu_edp_limits_size;
	int index;

	if (!cpu_edp_limits)
		return -EINVAL;

	index = nlimits - 1;

	if (temperature < cpu_edp_limits[0].temperature) {
		index = 0;
	} else {
		for (i = 0; i < (nlimits - 1); i++) {
			if (temperature >= cpu_edp_limits[i].temperature &&
			   temperature < cpu_edp_limits[i + 1].temperature) {
				index = i + 1;
				break;
			}
		}
	}

	mutex_lock(&tegra_cpu_lock);
	edp_thermal_index = index;

	/* Update cpu rate if cpufreq (at least on cpu0) is already started;
	   alter cpu dvfs table for this thermal zone if necessary */
	tegra_cpu_dvfs_alter(edp_thermal_index, &edp_cpumask, true, 0);
	if (target_cpu_speed[0]) {
		edp_update_limit();
		tegra_cpu_set_speed_cap(NULL);
	}
	tegra_cpu_dvfs_alter(edp_thermal_index, &edp_cpumask, false, 0);
	mutex_unlock(&tegra_cpu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tegra_edp_update_thermal_zone);

int tegra_system_edp_alarm(bool alarm)
{
	int ret = -ENODEV;

	mutex_lock(&tegra_cpu_lock);
	system_edp_alarm = alarm;

	/* Update cpu rate if cpufreq (at least on cpu0) is already started
	   and cancel emergency throttling after either edp limit is applied
	   or alarm is canceled */
	if (target_cpu_speed[0]) {
		edp_update_limit();
		ret = tegra_cpu_set_speed_cap(NULL);
	}
	if (!ret || !alarm)
		tegra_edp_throttle_cpu_now(0);

	mutex_unlock(&tegra_cpu_lock);

	return ret;
}

bool tegra_cpu_edp_favor_up(unsigned int n, int mp_overhead)
{
	unsigned int current_limit, next_limit;

	if (n == 0)
		return true;

	if (n >= ARRAY_SIZE(cpu_edp_limits->freq_limits))
		return false;

	current_limit = edp_predict_limit(n);
	next_limit = edp_predict_limit(n + 1);

	return ((next_limit * (n + 1)) >=
		(current_limit * n * (100 + mp_overhead) / 100));
}

bool tegra_cpu_edp_favor_down(unsigned int n, int mp_overhead)
{
	unsigned int current_limit, next_limit;

	if (n <= 1)
		return false;

	if (n > ARRAY_SIZE(cpu_edp_limits->freq_limits))
		return true;

	current_limit = edp_predict_limit(n);
	next_limit = edp_predict_limit(n - 1);

	return ((next_limit * (n - 1) * (100 + mp_overhead) / 100)) >
		(current_limit * n);
}

static int tegra_cpu_edp_notify(
	struct notifier_block *nb, unsigned long event, void *hcpu)
{
	int ret = 0;
	unsigned int cpu_speed, new_speed;
	int cpu = (long)hcpu;

	switch (event) {
	case CPU_UP_PREPARE:
		mutex_lock(&tegra_cpu_lock);
		cpu_set(cpu, edp_cpumask);
		edp_update_limit();

		cpu_speed = tegra_getspeed(0);
		new_speed = edp_governor_speed(cpu_speed);
		if (new_speed < cpu_speed) {
			ret = tegra_cpu_set_speed_cap(NULL);
			pr_info("tegra_cpu_edp_notify:%s cpu:%d force EDP limit %u kHz"
				"\n", ret ? " failed to " : " ", cpu, new_speed);
		}
		if (!ret)
			ret = tegra_cpu_dvfs_alter(
				edp_thermal_index, &edp_cpumask, false, event);
		if (ret) {
			cpu_clear(cpu, edp_cpumask);
			edp_update_limit();
		}
		mutex_unlock(&tegra_cpu_lock);
		break;
	case CPU_DEAD:
		mutex_lock(&tegra_cpu_lock);
		cpu_clear(cpu, edp_cpumask);
		tegra_cpu_dvfs_alter(
			edp_thermal_index, &edp_cpumask, true, event);
		edp_update_limit();
		tegra_cpu_set_speed_cap(NULL);
		mutex_unlock(&tegra_cpu_lock);
		break;
	}
	return notifier_from_errno(ret);
}

static struct notifier_block tegra_cpu_edp_notifier = {
	.notifier_call = tegra_cpu_edp_notify,
};

static void tegra_cpu_edp_init(bool resume)
{
	tegra_get_system_edp_limits(&system_edp_limits);
	tegra_get_cpu_edp_limits(&cpu_edp_limits, &cpu_edp_limits_size);

	if (!(cpu_edp_limits || system_edp_limits)) {
		if (!resume)
			pr_info("tegra_cpu_edp_init: no EDP table is provided\n");
		return;
	}

	/* FIXME: use the highest temperature limits if sensor is not on-line?
	 * If thermal zone is not set yet by the sensor, edp_thermal_index = 0.
	 * Boot frequency allowed SoC to get here, should work till sensor is
	 * initialized.
	 */
	edp_cpumask = *cpu_online_mask;
	edp_update_limit();

	if (!resume) {
		register_hotcpu_notifier(&tegra_cpu_edp_notifier);
		pr_info("tegra_cpu_edp_init: init EDP limit: %u MHz\n", edp_limit/1000);
	}
}

static void tegra_cpu_edp_exit(void)
{
	if (!(cpu_edp_limits || system_edp_limits))
		return;

	unregister_hotcpu_notifier(&tegra_cpu_edp_notifier);
}

#ifdef CONFIG_DEBUG_FS

static int system_edp_alarm_get(void *data, u64 *val)
{
	*val = (u64)system_edp_alarm;
	return 0;
}
static int system_edp_alarm_set(void *data, u64 val)
{
	if (val > 1) {	/* emulate emergency throttling */
		tegra_edp_throttle_cpu_now(val);
		return 0;
	}
	return tegra_system_edp_alarm((bool)val);
}
DEFINE_SIMPLE_ATTRIBUTE(system_edp_alarm_fops,
			system_edp_alarm_get, system_edp_alarm_set, "%llu\n");

static int __init tegra_edp_debug_init(struct dentry *cpu_tegra_debugfs_root)
{
	if (!debugfs_create_file("edp_alarm", 0644, cpu_tegra_debugfs_root,
				 NULL, &system_edp_alarm_fops))
		return -ENOMEM;

	return 0;
}
#endif

#else	/* CONFIG_TEGRA_EDP_LIMITS */
#define edp_governor_speed(requested_speed) (requested_speed)
#define tegra_cpu_edp_init(resume)
#define tegra_cpu_edp_exit()
#define tegra_edp_debug_init(cpu_tegra_debugfs_root) (0)
#endif	/* CONFIG_TEGRA_EDP_LIMITS */

#ifdef CONFIG_DEBUG_FS

static struct dentry *cpu_tegra_debugfs_root;

static int __init tegra_cpu_debug_init(void)
{
	cpu_tegra_debugfs_root = debugfs_create_dir("cpu-tegra", 0);

	if (!cpu_tegra_debugfs_root)
		return -ENOMEM;

	if (tegra_edp_debug_init(cpu_tegra_debugfs_root))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(cpu_tegra_debugfs_root);
	return -ENOMEM;
}

static void __exit tegra_cpu_debug_exit(void)
{
	debugfs_remove_recursive(cpu_tegra_debugfs_root);
}

late_initcall(tegra_cpu_debug_init);
module_exit(tegra_cpu_debug_exit);
#endif /* CONFIG_DEBUG_FS */

int tegra_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

unsigned int tegra_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= CONFIG_NR_CPUS || !ACCESS_ONCE(cpu_clk))
		return 0;

	rate = clk_get_rate(cpu_clk) / 1000;
	return rate;
}

int tegra_update_cpu_speed(unsigned long rate)
{
	int ret = 0;
	struct cpufreq_freqs freqs;
	unsigned long rate_save = rate;
	int status = 1;
	
	// dont allow changes while in early suspend boost mode
	if (in_earlysuspend)
		return ret;
		
	freqs.old = tegra_getspeed(0);
	freqs.new = rate;

	rate = clk_round_rate(cpu_clk, rate * 1000);
	if (!IS_ERR_VALUE(rate))
		freqs.new = rate / 1000;

	if (rate_save > T3_LP_MAX_FREQ) {
		if (is_lp_cluster()) {
			/* set rate to max of LP mode */
			ret = clk_set_rate(cpu_clk, T3_LP_MAX_FREQ * 1000);
#ifndef CONFIG_TEGRA_MPDECISION
			/* change to g mode */
			clk_set_parent(cpu_clk, cpu_g_clk);
#else
            /*
             * the above variant is now no longer preferred since
             * mpdecision would not know about this. Notify mpdecision
             * instead to switch to G mode
             */
             status = mpdecision_gmode_notifier();
             if (status == 0)
             	pr_err("tegra_update_cpu_speed: %s: couldn't switch to gmode (freq)", __func__ );
#endif
			/* restore the target frequency, and
			 * let the rest of the function handle
			 * the frequency scale up
			 */
			freqs.new = rate_save;
		}
	}

	if (freqs.old == freqs.new){
		return ret;
	}
	
	/*
	 * Vote on memory bus frequency based on cpu frequency
	 * This sets the minimum frequency, display or avp may request higher
	 */
	if (freqs.old < freqs.new) {
		ret = tegra_update_mselect_rate(freqs.new);
		if (ret) {
			pr_err("tegra_update_cpu_speed: Failed to scale mselect for cpu"
			       " frequency %u kHz\n", freqs.new);
			return ret;
		}
		ret = clk_set_rate(emc_clk, tegra_emc_to_cpu_ratio(freqs.new));
		if (ret) {
			pr_err("tegra_update_cpu_speed: Failed to scale emc for cpu"
			       " frequency %u kHz\n", freqs.new);
			return ret;
		}
	}

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);


	ret = clk_set_rate(cpu_clk, freqs.new * 1000);
	if (ret) {
		pr_err("tegra_update_cpu_speed: Failed to set cpu frequency to %d kHz\n",
			freqs.new);
		return ret;
	} 

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	if (freqs.old > freqs.new) {
		clk_set_rate(emc_clk, tegra_emc_to_cpu_ratio(freqs.new));
		tegra_update_mselect_rate(freqs.new);
	}

	return 0;
}

unsigned int tegra_count_slow_cpus(unsigned long speed_limit)
{
	unsigned int cnt = 0;
	int i;

	for_each_online_cpu(i)
		if (target_cpu_speed[i] <= speed_limit)
			cnt++;
	return cnt;
}

unsigned int tegra_get_slowest_cpu_n(void) {
	unsigned int cpu = nr_cpu_ids;
	unsigned long rate = ULONG_MAX;
	int i;

	for_each_online_cpu(i)
		if ((i > 0) && (rate > target_cpu_speed[i])) {
			cpu = i;
			rate = target_cpu_speed[i];
		}
	return cpu;
}

unsigned long tegra_cpu_lowest_speed(void) {
	unsigned long rate = ULONG_MAX;
	int i;

	for_each_online_cpu(i)
		rate = min(rate, target_cpu_speed[i]);
	return rate;
}

unsigned long tegra_cpu_highest_speed(void) {
	unsigned long policy_max = ULONG_MAX;
	unsigned long rate = 0;
	int i;

	for_each_online_cpu(i) {
		if (force_policy_max)
			policy_max = min(policy_max, policy_max_speed[i]);
		rate = max(rate, target_cpu_speed[i]);
	}
	rate = min(rate, policy_max);
	return rate;
}

// maxwen: apply all limits to a frequency
static unsigned int get_scaled_freq (unsigned int target_freq)
{
    /* chip-dependent, such as thermal throttle, edp, and user-defined freq. cap */
	target_freq = pmqos_cap_speed (target_freq);
    target_freq = tegra_throttle_governor_speed (target_freq);
	unsigned int edp_freq = edp_governor_speed (target_freq);
	if(edp_freq >= 1000)
	{
		target_freq = edp_governor_speed (target_freq);
	}
	target_freq = user_cap_speed (target_freq);
    return target_freq;
}

unsigned int best_core_to_turn_up (void) {
    /* mitigate high temperature, 0 -> 3 -> 2 -> 1 */
    if (!cpu_online (3))
        return 3;

    if (!cpu_online (2))
        return 2;

    if (!cpu_online (1))
        return 1;

    /* NOT found, return >= nr_cpu_id */
    return nr_cpu_ids;
}
EXPORT_SYMBOL(best_core_to_turn_up);

int tegra_cpu_set_speed_cap(unsigned int *speed_cap)
{
	int ret = 0;
    unsigned int new_speed = tegra_cpu_highest_speed();
    
    unsigned int curr_speed = tegra_getspeed(0);
    
	// dont allow changes while in early suspend boost mode
	if (in_earlysuspend)
		return ret;

	if (is_suspended)
		return -EBUSY;

	new_speed = get_scaled_freq(new_speed);
	if (speed_cap)
		*speed_cap = new_speed;

	if (curr_speed == new_speed)
		return 0;

	ret = tegra_update_cpu_speed(new_speed);
	tegra_auto_hotplug_governor(new_speed, false);
	return ret;
}

int tegra_suspended_target(unsigned int target_freq)
{
	unsigned int new_speed = target_freq;

	if (!is_suspended)
		return -EBUSY;

	/* apply only "hard" caps */
	new_speed = tegra_throttle_governor_speed(new_speed);
	new_speed = edp_governor_speed(new_speed);

	return tegra_update_cpu_speed(new_speed);
}

int tegra_input_boost (int cpu, unsigned int target_freq)
{
    int ret = 0;
    unsigned int curfreq = 0, scaling_max_limit = 0;

    curfreq = tegra_getspeed(0);

    /* get global caped limit */
    target_freq = get_scaled_freq(target_freq);

    /* get any per cpu defined limit cause input_boost
     might not be validated against policy->max */
    scaling_max_limit = get_cpu_freq_limit(cpu);

    /* apply any scaling max limits */
    if (scaling_max_limit < target_freq)
        target_freq = scaling_max_limit;

    /* dont need to boost cpu at this moment */
    if (!curfreq || curfreq >= target_freq) {
        return -EINVAL;
    }

    mutex_lock(&tegra_cpu_lock);

    target_cpu_speed[cpu] = target_freq;

    /* will auto. round-rate */
    ret = tegra_update_cpu_speed(target_freq);

    mutex_unlock(&tegra_cpu_lock);

    return ret;
}
EXPORT_SYMBOL (tegra_input_boost);

static int tegra_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int idx;
	unsigned int freq;
	int ret = 0;

	ret = cpufreq_frequency_table_target(policy, freq_table, target_freq,
		relation, &idx);
	if (ret)
		return ret;

	freq = freq_table[idx].frequency;

	mutex_lock(&tegra_cpu_lock);

	target_cpu_speed[policy->cpu] = freq;
	ret = tegra_cpu_set_speed_cap(NULL);

	mutex_unlock(&tegra_cpu_lock);

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend tegra_cpufreq_early_suspender;
struct early_suspend tegra_cpufreq_performance_early_suspender;
static struct pm_qos_request_list cap_cpu_freq_req;
static struct pm_qos_request_list cap_cpu_num_req;
static struct pm_qos_request_list boost_cpu_freq_req;
#endif

static int tegra_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	mutex_lock(&tegra_cpu_lock);
	if (event == PM_SUSPEND_PREPARE) {
		unsigned int freq;
		is_suspended = true;
		freq=freq_table[suspend_index].frequency;
		tegra_update_cpu_speed(freq);
		tegra_auto_hotplug_governor(freq, true);
	} else if (event == PM_POST_SUSPEND) {
		unsigned int freq;
		is_suspended = false;
		tegra_cpu_edp_init(true);
		
		tegra_cpu_set_speed_cap(&freq);
	}
	mutex_unlock(&tegra_cpu_lock);
	return NOTIFY_OK;
}

static struct notifier_block tegra_cpu_pm_notifier = {
	.notifier_call = tegra_pm_notify,
};

static int tegra_cpu_init(struct cpufreq_policy *policy)
{
	if (policy->cpu >= CONFIG_NR_CPUS)
		return -EINVAL;

	cpu_clk = clk_get_sys(NULL, "cpu");
	cpu_g_clk = clk_get_sys(NULL, "cpu_g");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	emc_clk = clk_get_sys("cpu", "emc");
	if (IS_ERR(emc_clk)) {
		clk_put(cpu_clk);
		return PTR_ERR(emc_clk);
	}

	clk_enable(emc_clk);
	clk_enable(cpu_clk);

	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);
	policy->cur = tegra_getspeed(policy->cpu);
	target_cpu_speed[policy->cpu] = policy->cur;

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 50 * 1000;

	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	if (policy->cpu == 0) {
		policy->max = get_cpu_freq_limit(policy->cpu);
		policy->min = get_cpu_freq_limit_min(policy->cpu);
		register_pm_notifier(&tegra_cpu_pm_notifier);
	}

    /* restore saved cpu frequency */
    if (policy->cpu > 0) {
		policy->max = get_cpu_freq_limit(policy->cpu);
		policy->min = get_cpu_freq_limit_min(policy->cpu);
		tegra_update_cpu_speed(policy->max);
	}

	return 0;
}

static int tegra_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	clk_disable(emc_clk);
	clk_put(emc_clk);
	clk_put(cpu_clk);
	return 0;
}

static int tegra_cpufreq_policy_notifier(
	struct notifier_block *nb, unsigned long event, void *data)
{
	int i, ret;
	struct cpufreq_policy *policy = data;

	if (event == CPUFREQ_NOTIFY) {
		ret = cpufreq_frequency_table_target(policy, freq_table,
			policy->max, CPUFREQ_RELATION_H, &i);
		policy_max_speed[policy->cpu] =
			ret ? policy->max : freq_table[i].frequency;
	}
	return NOTIFY_OK;
}

static struct notifier_block tegra_cpufreq_policy_nb = {
	.notifier_call = tegra_cpufreq_policy_notifier,
};

static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	&throttle,
#endif
	NULL,
};

static struct cpufreq_driver tegra_cpufreq_driver = {
	.verify		= tegra_verify_speed,
	.target		= tegra_target,
	.get		= tegra_getspeed,
	.init		= tegra_cpu_init,
	.exit		= tegra_cpu_exit,
	.name		= "tegra",
	.attr		= tegra_cpufreq_attr,
};

#ifdef CONFIG_HAS_EARLYSUSPEND

static void tegra_cpufreq_early_suspend(struct early_suspend *h)
{
	// this is the last suspend handler
	pr_info("tegra_cpufreq_early_suspend: clean cpu freq boost\n");
	in_earlysuspend = false;
	pm_qos_update_request(&boost_cpu_freq_req, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
	
	pr_info("tegra_cpufreq_early_suspend: cap cpu freq to %d\n", suspend_cap_freq);
	pm_qos_update_request(&cap_cpu_freq_req, (s32)suspend_cap_freq);
	
	if (suspend_cap_freq > T3_LP_MAX_FREQ) {
		pr_info("tegra_cpufreq_early_suspend: cap max cpu to %d\n", suspend_cap_cpu_num);
		pm_qos_update_request(&cap_cpu_num_req, (s32)suspend_cap_cpu_num);
	}
#if 0
	unsigned int scaling_min_freq;
	// TODO: if scaling_min_freq != 51000 is set this will 
	// disable deep sleep now
	// how can we overrule cpufreq?
	scaling_min_freq = get_cpu_freq_limit_min(0);
	if (scaling_min_freq != T3_CPU_MIN_FREQ) {
		pr_info("tegra_delayed_suspend_work: scaling_min_freq %d\n", scaling_min_freq);
	}
#endif
}

static void tegra_cpufreq_late_resume(struct early_suspend *h)
{
	pr_info("tegra_cpufreq_late_resume: clean cpu freq cap\n");
	pm_qos_update_request(&cap_cpu_freq_req, (s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);

	if (suspend_cap_freq > T3_LP_MAX_FREQ) {
		pr_info("tegra_cpufreq_late_resume: clean max cpu cap\n");
		pm_qos_update_request(&cap_cpu_num_req, (s32)PM_QOS_MAX_ONLINE_CPUS_DEFAULT_VALUE);
	}

	// boost at the beginning of the resume
	pr_info("tegra_cpufreq_late_resume: boost cpu freq\n");
	tegra_update_cpu_speed(T3_CPU_FREQ_BOOST);
	// now disable all speed changes until finished
	in_earlysuspend = true;
	pm_qos_update_request(&boost_cpu_freq_req, (s32)T3_CPU_FREQ_BOOST);
}

static void tegra_cpufreq_performance_early_suspend(struct early_suspend *h)
{
	// this is the first suspend handler
	pr_info("tegra_cpufreq_performance_early_suspend: boost cpu freq\n");
	tegra_update_cpu_speed(T3_CPU_FREQ_BOOST);
	// now disable all speed changes until finished
	in_earlysuspend = true;
	pm_qos_update_request(&boost_cpu_freq_req, (s32)T3_CPU_FREQ_BOOST);	
}

static void tegra_cpufreq_performance_late_resume(struct early_suspend *h)
{
	// this is the last resume handler
	pr_info("tegra_cpufreq_performance_late_resume: clean cpu freq boost\n");
	in_earlysuspend = false;
	pm_qos_update_request(&boost_cpu_freq_req, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
}

#endif
static void ril_suspend_resume_worker(struct work_struct *w)
{
	pr_info("ril_suspend_resume_worker: clean cpu cap by RIL\n");
	pm_qos_update_request(&cap_cpu_freq_req, (s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);

	pr_info("ril_suspend_resume_worker: boost cpu freq by RIL\n");
	pm_qos_update_request(&boost_cpu_freq_req, (s32)tegra_get_suspend_boost_freq());
	tegra_update_cpu_speed(tegra_get_suspend_boost_freq());
}

static int __init tegra_cpufreq_init(void)
{
	int ret = 0;

	struct tegra_cpufreq_table_data *table_data =
		tegra_cpufreq_table_get();
	if (IS_ERR_OR_NULL(table_data))
		return -EINVAL;

	suspend_index = table_data->suspend_index;

	ret = tegra_throttle_init(&tegra_cpu_lock);
	if (ret)
		return ret;

	ret = tegra_auto_hotplug_init(&tegra_cpu_lock);
	if (ret)
		return ret;

	freq_table = table_data->freq_table;
	tegra_cpu_edp_init(false);

	INIT_WORK(&ril_suspend_resume_work, ril_suspend_resume_worker);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	pm_qos_add_request(&cap_cpu_freq_req, PM_QOS_CPU_FREQ_MAX, (s32)PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
	pm_qos_add_request(&cap_cpu_num_req, PM_QOS_MAX_ONLINE_CPUS, (s32)PM_QOS_MAX_ONLINE_CPUS_DEFAULT_VALUE);
	pm_qos_add_request(&boost_cpu_freq_req, PM_QOS_CPU_FREQ_MIN, (s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);		
	
	// will cap frequency on screen off
	tegra_cpufreq_early_suspender.suspend = tegra_cpufreq_early_suspend;
	tegra_cpufreq_early_suspender.resume = tegra_cpufreq_late_resume;
	tegra_cpufreq_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 100;
	register_early_suspend(&tegra_cpufreq_early_suspender);

	// to get max boost for the complete suspend and resume time
	tegra_cpufreq_performance_early_suspender.suspend = tegra_cpufreq_performance_early_suspend;
	tegra_cpufreq_performance_early_suspender.resume = tegra_cpufreq_performance_late_resume;
	tegra_cpufreq_performance_early_suspender.level = 0;
	register_early_suspend(&tegra_cpufreq_performance_early_suspender);
#endif


	ret = cpufreq_register_notifier(
		&tegra_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;

	return cpufreq_register_driver(&tegra_cpufreq_driver);
}

static void __exit tegra_cpufreq_exit(void)
{
	tegra_throttle_exit();
	tegra_cpu_edp_exit();
	tegra_auto_hotplug_exit();
#ifdef CONFIG_HAS_EARLYSUSPEND
	pm_qos_remove_request(&cap_cpu_freq_req);
	pm_qos_remove_request(&cap_cpu_num_req);
	pm_qos_remove_request(&boost_cpu_freq_req);
			
	unregister_early_suspend(&tegra_cpufreq_performance_early_suspender);
	unregister_early_suspend(&tegra_cpufreq_early_suspender);
#endif
	cpufreq_unregister_driver(&tegra_cpufreq_driver);
	cpufreq_unregister_notifier(
		&tegra_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
}


MODULE_AUTHOR("Colin Cross <ccross@android.com>");
MODULE_DESCRIPTION("cpufreq driver for Nvidia Tegra2");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
