/*
 * drivers/misc/tegra-profiler/power_clk.c
 *
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/version.h>

#include <linux/tegra_profiler.h>

#include "power_clk.h"
#include "quadd.h"
#include "hrt.h"
#include "comm.h"
#include "debug.h"

#define PCLK_MAX_VALUES	32

struct power_clk_data {
	unsigned int value;
	unsigned int prev;
};

#define PCLK_NB_GPU	0
#define PCLK_NB_EMC	0

enum {
	PCLK_NB_CPU_FREQ,
	PCLK_NB_CPU_HOTPLUG,
	PCLK_NB_CPU_MAX,
};

#define PCLK_NB_MAX	PCLK_NB_CPU_MAX

struct power_clk_source {
	int type;

	struct clk *clkp;
	struct notifier_block nb[PCLK_NB_MAX];

	int cpu;
	int nr;
	struct power_clk_data data[PCLK_MAX_VALUES];

	atomic_t active;
	struct mutex lock;
};

struct power_clk_context_s {
	struct power_clk_source cpu;
	struct power_clk_source gpu;
	struct power_clk_source emc;

	struct timer_list timer;
	unsigned int period;

	unsigned int is_cpufreq : 1;

	struct quadd_ctx *quadd_ctx;
};

static struct power_clk_context_s power_ctx;

static void make_sample(struct power_clk_source *s)
{
	int i;
	u32 values[PCLK_MAX_VALUES];
	struct quadd_iovec vec;

	struct quadd_record_data record;
	struct quadd_power_rate_data *p = &record.power_rate;

	record.record_type = QUADD_RECORD_TYPE_POWER_RATE;

	p->type = (u8)s->type;
	p->time = quadd_get_time();
	p->cpu_id = (u32)s->cpu;
	p->flags = 0;

	if (s->type == QUADD_POWER_CLK_CPU) {
		p->nr_values = 1;
		values[0] = s->data[s->cpu].value;
	} else {
		p->nr_values = (u16)s->nr;
		for (i = 0; i < s->nr; i++)
			values[i] = s->data[i].value;
	}

	vec.base = values;
	vec.len = p->nr_values * sizeof(values[0]);

	quadd_put_sample(&record, &vec, 1);
}

static void
make_sample_hotplug(int cpu, int is_online)
{
	struct quadd_record_data record;
	struct quadd_hotplug_data *s = &record.hotplug;

	record.record_type = QUADD_RECORD_TYPE_HOTPLUG;

	s->cpu = cpu;
	s->is_online = is_online ? 1 : 0;
	s->time = quadd_get_time();
	s->reserved = 0;

	quadd_put_sample(&record, NULL, 0);
}

static inline int
is_data_changed(struct power_clk_source *s)
{
	int i, cpu;

	if (s->type == QUADD_POWER_CLK_CPU) {
		cpu = s->cpu;
		return (s->data[cpu].value != s->data[cpu].prev);
	}

	for (i = 0; i < s->nr; i++) {
		if (s->data[i].value != s->data[i].prev)
			return 1;
	}

	return 0;
}

static inline void
update_data(struct power_clk_source *s)
{
	int i, cpu;

	if (s->type == QUADD_POWER_CLK_CPU) {
		cpu = s->cpu;
		s->data[cpu].prev = s->data[cpu].value;
		return;
	}

	for (i = 0; i < s->nr; i++)
		s->data[i].prev = s->data[i].value;
}

static void check_source(struct power_clk_source *s)
{
	if (is_data_changed(s)) {
		update_data(s);
		make_sample(s);
	}
}

static void
read_source(struct power_clk_source *s, int cpu)
{
	unsigned int value;

	switch (s->type) {
	case QUADD_POWER_CLK_CPU:
		/* update cpu frequency */
		if (cpu < 0 || cpu >= max_t(int, s->nr, nr_cpu_ids)) {
			pr_err_once("error: cpu id: %d\n", cpu);
			break;
		}

		value = cpufreq_get(cpu);

		if (!mutex_trylock(&s->lock))
			break;

		s->cpu = cpu;
		s->data[cpu].value = value;
		pr_debug("PCLK_CPU(%d), value: %u\n", cpu, s->data[cpu].value);
		check_source(s);

		mutex_unlock(&s->lock);
		break;

	case QUADD_POWER_CLK_GPU:
		/* update gpu frequency */
		if (!mutex_trylock(&s->lock))
			break;

		if (s->clkp)
			s->data[0].value =
				(unsigned int)(clk_get_rate(s->clkp) / 1000);
		pr_debug("PCLK_GPU, value: %u\n", s->data[0].value);
		s->cpu = cpu;
		check_source(s);

		mutex_unlock(&s->lock);
		break;

	case QUADD_POWER_CLK_EMC:
		/* update emc frequency */
		if (!mutex_trylock(&s->lock))
			break;

		if (s->clkp)
			s->data[0].value =
				(unsigned int)(clk_get_rate(s->clkp) / 1000);
		pr_debug("PCLK_EMC, value: %u\n", s->data[0].value);
		s->cpu = cpu;
		check_source(s);

		mutex_unlock(&s->lock);
		break;

	default:
		pr_err_once("error: invalid power_clk type\n");
		break;
	}
}

static int
gpu_notifier_call(struct notifier_block *nb,
		  unsigned long action, void *data)
{
	read_source(&power_ctx.gpu, -1);
	return NOTIFY_DONE;
}

static int
emc_notifier_call(struct notifier_block *nb,
		  unsigned long action, void *data)
{
	read_source(&power_ctx.emc, -1);
	return NOTIFY_DONE;
}

static void
read_cpufreq(struct power_clk_source *s, struct cpufreq_freqs *freq)
{
	int cpu, cpufreq;

	if (!mutex_trylock(&s->lock))
		return;

	if (!atomic_read(&s->active))
		goto out_unlock;

	cpu = freq->cpu;
	cpufreq = freq->new;

	pr_debug("cpu: %d, cpufreq: %d\n", cpu, cpufreq);

	if (cpu >= s->nr) {
		pr_err_once("error: cpu id: %d\n", cpu);
		goto out_unlock;
	}

	s->cpu = cpu;
	s->data[cpu].value = cpufreq;

	pr_debug("[%d] cpufreq: %u --> %u\n",
		 cpu, freq->old, cpufreq);

	check_source(s);

out_unlock:
	mutex_unlock(&s->lock);
}

static int
cpufreq_notifier_call(struct notifier_block *nb,
		      unsigned long action, void *hcpu)
{
	struct cpufreq_freqs *freq;
	struct power_clk_source *s = &power_ctx.cpu;

	if (!atomic_read(&s->active))
		return 0;

	pr_debug("action: %lu\n", action);

	if (action == CPUFREQ_POSTCHANGE) {
		freq = hcpu;
		read_cpufreq(s, freq);
	}

	return 0;
}

static int
cpu_hotplug_notifier_call(struct notifier_block *nb,
			  unsigned long action, void *hcpu)
{
	int cpu;
	struct power_clk_source *s = &power_ctx.cpu;

	if (!atomic_read(&s->active))
		return NOTIFY_DONE;

	cpu = (long)hcpu;

	pr_debug("cpu: %d, action: %lu\n", cpu, action);

	if (cpu >= s->nr) {
		pr_err_once("error: cpu id: %d\n", cpu);
		return NOTIFY_DONE;
	}

	switch (action) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		make_sample_hotplug(cpu, 1);
		break;

	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		mutex_lock(&s->lock);
		if (atomic_read(&s->active))
			s->data[cpu].value = 0;
		mutex_unlock(&s->lock);

		make_sample_hotplug(cpu, 0);
		break;

	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static void reset_data(struct power_clk_source *s)
{
	int i;

	for (i = 0; i < s->nr; i++) {
		s->data[i].value = 0;
		s->data[i].prev = 0;
	}
}

static void init_source(struct power_clk_source *s,
			int nr_values,
			unsigned int type)
{
	s->clkp = NULL;
	s->type = type;
	s->nr = min_t(int, nr_values, PCLK_MAX_VALUES);
	atomic_set(&s->active, 0);
	mutex_init(&s->lock);

	reset_data(s);
}

static void
power_clk_work_func(struct work_struct *work)
{
	read_source(&power_ctx.gpu, -1);
	read_source(&power_ctx.emc, -1);
}

static DECLARE_WORK(power_clk_work, power_clk_work_func);

static void power_clk_timer(unsigned long data)
{
	struct timer_list *timer = &power_ctx.timer;

	schedule_work(&power_clk_work);
	timer->expires = jiffies + msecs_to_jiffies(power_ctx.period);
	add_timer(timer);
}

static void
read_all_sources_work_func(struct work_struct *work)
{
	int cpu_id;
	struct power_clk_source *s = &power_ctx.cpu;

	if (power_ctx.is_cpufreq) {
		for_each_possible_cpu(cpu_id)
			read_source(s, cpu_id);
	}

	read_source(&power_ctx.gpu, -1);
	read_source(&power_ctx.emc, -1);
}

static DECLARE_WORK(read_all_sources_work, read_all_sources_work_func);

static int
enable_clock(struct power_clk_source *s, struct notifier_block *nb,
	     const char *dev_id, const char *con_id)
{
	int ret;

	mutex_lock(&s->lock);

	s->clkp = clk_get_sys(dev_id, con_id);
	if (IS_ERR_OR_NULL(s->clkp)) {
		pr_warn("warning: could not setup clock: \"%s:%s\"\n",
			dev_id ? dev_id : "", con_id ? con_id : "");
		ret = -ENOENT;
		goto errout;
	}

	ret = clk_prepare_enable(s->clkp);
	if (ret) {
		pr_warn("warning: could not enable gpu clock\n");
		goto errout_free_clk;
	}

#ifdef CONFIG_COMMON_CLK
	ret = clk_notifier_register(s->clkp, nb);
	if (ret) {
		pr_warn("warning: could not register clock: \"%s:%s\"\n",
			dev_id ? dev_id : "", con_id ? con_id : "");
		goto errout_disable_clk;
	}
#endif

	reset_data(s);
	atomic_set(&s->active, 1);

	mutex_unlock(&s->lock);

	return 0;

#ifdef CONFIG_COMMON_CLK
errout_disable_clk:
	clk_disable_unprepare(s->clkp);
#endif

errout_free_clk:
	clk_put(s->clkp);

errout:
	s->clkp = NULL;
	atomic_set(&s->active, 0);

	mutex_unlock(&s->lock);

	return ret;
}

static void
disable_clock(struct power_clk_source *s, struct notifier_block *nb)
{
	mutex_lock(&s->lock);

	if (atomic_cmpxchg(&s->active, 1, 0)) {
		if (s->clkp) {
#ifdef CONFIG_COMMON_CLK
			clk_notifier_unregister(s->clkp, nb);
#endif
			clk_disable_unprepare(s->clkp);
			clk_put(s->clkp);

			s->clkp = NULL;
		}
	}

	mutex_unlock(&s->lock);
}

int quadd_power_clk_start(void)
{
	struct power_clk_source *s;
	struct timer_list *timer = &power_ctx.timer;
	struct quadd_parameters *param = &power_ctx.quadd_ctx->param;

	if (param->power_rate_freq == 0) {
		pr_info("power_clk is not started\n");
		return 0;
	}

#ifdef CONFIG_COMMON_CLK
	power_ctx.period = 0;
#else
	power_ctx.period = MSEC_PER_SEC / param->power_rate_freq;
	pr_info("pclk: use timer, freq: %u\n", param->power_rate_freq);
#endif

	pr_info("pclk: start, cpufreq: %s\n",
		power_ctx.is_cpufreq ? "yes" : "no");

	/* setup gpu frequency */
	s = &power_ctx.gpu;
	enable_clock(s, &s->nb[PCLK_NB_GPU], "3d", NULL);

	/* setup emc frequency */
	s = &power_ctx.emc;
	enable_clock(s, &s->nb[PCLK_NB_EMC], "cpu", "emc");

	/* setup cpu frequency notifier */
	if (power_ctx.is_cpufreq) {
		s = &power_ctx.cpu;
		mutex_lock(&s->lock);
		reset_data(s);
		atomic_set(&s->active, 1);
		mutex_unlock(&s->lock);
	}

	if (power_ctx.period > 0) {
		init_timer(timer);
		timer->function = power_clk_timer;
		timer->expires = jiffies + msecs_to_jiffies(power_ctx.period);
		timer->data = 0;
		add_timer(timer);
	}

	schedule_work(&read_all_sources_work);

	return 0;
}

void quadd_power_clk_stop(void)
{
	struct power_clk_source *s;
	struct quadd_parameters *param = &power_ctx.quadd_ctx->param;

	if (param->power_rate_freq == 0)
		return;

	if (power_ctx.period > 0)
		del_timer_sync(&power_ctx.timer);

	s = &power_ctx.gpu;
	disable_clock(s, &s->nb[PCLK_NB_GPU]);

	s = &power_ctx.emc;
	disable_clock(s, &s->nb[PCLK_NB_EMC]);

	if (power_ctx.is_cpufreq) {
		s = &power_ctx.cpu;
		mutex_lock(&s->lock);
		atomic_set(&s->active, 0);
		s->clkp = NULL;
		mutex_unlock(&s->lock);
	}

	pr_info("pclk: stop\n");
}

int quadd_power_clk_init(struct quadd_ctx *quadd_ctx)
{
	int __maybe_unused ret;
	struct power_clk_source *s;

	s = &power_ctx.gpu;
	s->nb[PCLK_NB_GPU].notifier_call = gpu_notifier_call;
	init_source(s, 1, QUADD_POWER_CLK_GPU);

	s = &power_ctx.emc;
	s->nb[PCLK_NB_EMC].notifier_call = emc_notifier_call;
	init_source(s, 1, QUADD_POWER_CLK_EMC);

	s = &power_ctx.cpu;
	s->nb[PCLK_NB_CPU_FREQ].notifier_call = cpufreq_notifier_call;
	s->nb[PCLK_NB_CPU_HOTPLUG].notifier_call = cpu_hotplug_notifier_call;
	init_source(s, nr_cpu_ids, QUADD_POWER_CLK_CPU);

	power_ctx.quadd_ctx = quadd_ctx;

#ifdef CONFIG_CPU_FREQ
	ret = cpufreq_register_notifier(&s->nb[PCLK_NB_CPU_FREQ],
					CPUFREQ_TRANSITION_NOTIFIER);
	if (ret < 0) {
		pr_warn("CPU freq registration failed: %d\n", ret);
		power_ctx.is_cpufreq = 0;
	} else {
		power_ctx.is_cpufreq = 1;
	}
#else
	power_ctx.is_cpufreq = 0;
#endif
	quadd_ctx->pclk_cpufreq = power_ctx.is_cpufreq;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0)
	register_cpu_notifier(&s->nb[PCLK_NB_CPU_HOTPLUG]);
#endif

	return 0;
}

void quadd_power_clk_deinit(void)
{
	struct power_clk_source *s = &power_ctx.cpu;

	quadd_power_clk_stop();

#ifdef CONFIG_CPU_FREQ
	if (power_ctx.is_cpufreq)
		cpufreq_unregister_notifier(&s->nb[PCLK_NB_CPU_FREQ],
					    CPUFREQ_TRANSITION_NOTIFIER);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0)
	unregister_cpu_notifier(&s->nb[PCLK_NB_CPU_HOTPLUG]);
#endif
}
