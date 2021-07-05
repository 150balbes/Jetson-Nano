/*
 * gk20a clock scaling profile
 *
 * Copyright (c) 2013-2020, NVIDIA Corporation. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/devfreq.h>
#include <linux/export.h>
#include <soc/tegra/chip-id.h>
#include <linux/pm_qos.h>

#include <governor.h>

#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/clk_arb.h>

#include "platform_gk20a.h"
#include "scale.h"
#include "os_linux.h"

/*
 * gk20a_scale_qos_notify()
 *
 * This function is called when the minimum QoS requirement for the device
 * has changed. The function calls postscaling callback if it is defined.
 */

#if defined(CONFIG_GK20A_PM_QOS) && defined(CONFIG_COMMON_CLK)
int gk20a_scale_qos_notify(struct notifier_block *nb,
			  unsigned long n, void *p)
{
	struct gk20a_scale_profile *profile =
			container_of(nb, struct gk20a_scale_profile,
			qos_notify_block);
	struct gk20a *g = get_gk20a(profile->dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct devfreq *devfreq = l->devfreq;

	if (!devfreq)
		return NOTIFY_OK;

	mutex_lock(&devfreq->lock);
	/* check for pm_qos min and max frequency requirement */
	profile->qos_min_freq =
	  (unsigned long)pm_qos_read_min_bound(PM_QOS_GPU_FREQ_BOUNDS) * 1000UL;
	profile->qos_max_freq =
	  (unsigned long)pm_qos_read_max_bound(PM_QOS_GPU_FREQ_BOUNDS) * 1000UL;

	if (profile->qos_min_freq > profile->qos_max_freq) {
		nvgpu_err(g,
			"QoS: setting invalid limit, min_freq=%lu max_freq=%lu",
			profile->qos_min_freq, profile->qos_max_freq);
		profile->qos_min_freq = profile->qos_max_freq;
	}

	update_devfreq(devfreq);
	mutex_unlock(&devfreq->lock);

	return NOTIFY_OK;
}
#elif defined(CONFIG_GK20A_PM_QOS)
int gk20a_scale_qos_notify(struct notifier_block *nb,
			  unsigned long n, void *p)
{
	struct gk20a_scale_profile *profile =
		container_of(nb, struct gk20a_scale_profile,
			     qos_notify_block);
	struct gk20a_platform *platform = dev_get_drvdata(profile->dev);
	struct gk20a *g = get_gk20a(profile->dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	unsigned long freq;

	if (!platform->postscale)
		return NOTIFY_OK;

	/* get the frequency requirement. if devfreq is enabled, check if it
	 * has higher demand than qos */
	freq = platform->clk_round_rate(profile->dev,
			(u32)pm_qos_read_min_bound(PM_QOS_GPU_FREQ_BOUNDS));
	if (l->devfreq)
		freq = max(l->devfreq->previous_freq, freq);

	/* Update gpu load because we may scale the emc target
	 * if the gpu load changed. */
	nvgpu_pmu_load_update(g);
	platform->postscale(profile->dev, freq);

	return NOTIFY_OK;
}
#else
int gk20a_scale_qos_notify(struct notifier_block *nb,
			  unsigned long n, void *p)
{
	return 0;
}
#endif

/*
 * gk20a_scale_make_freq_table(profile)
 *
 * This function initialises the frequency table for the given device profile
 */

static int gk20a_scale_make_freq_table(struct gk20a_scale_profile *profile)
{
	struct gk20a_platform *platform = dev_get_drvdata(profile->dev);
	int num_freqs, err;
	unsigned long *freqs;

	if (platform->get_clk_freqs) {
		/* get gpu frequency table */
		err = platform->get_clk_freqs(profile->dev, &freqs,
					&num_freqs);

		if (err)
			return -ENOSYS;
	} else
		return -ENOSYS;

	profile->devfreq_profile.freq_table = (unsigned long *)freqs;
	profile->devfreq_profile.max_state = num_freqs;

	return 0;
}

/*
 * gk20a_scale_target(dev, *freq, flags)
 *
 * This function scales the clock
 */

static int gk20a_scale_target(struct device *dev, unsigned long *freq,
			      u32 flags)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = platform->g;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct gk20a_scale_profile *profile = g->scale_profile;
	struct devfreq *devfreq = l->devfreq;
	unsigned long local_freq = *freq;
	unsigned long rounded_rate;
	unsigned long min_freq = 0, max_freq = 0;

	if (nvgpu_clk_arb_has_active_req(g))
		return 0;
	/*
	 * Calculate floor and cap frequency values
	 *
	 * Policy :
	 * We have two APIs to clip the frequency
	 *  1. devfreq
	 *  2. pm_qos
	 *
	 * To calculate floor (min) freq, we select MAX of floor frequencies
	 * requested from both APIs
	 * To get cap (max) freq, we select MIN of max frequencies
	 *
	 * In case we have conflict (min_freq > max_freq) after above
	 * steps, we ensure that max_freq wins over min_freq
	 */
	min_freq = max_t(u32, devfreq->min_freq, profile->qos_min_freq);
	max_freq = min_t(u32, devfreq->max_freq, profile->qos_max_freq);

	if (min_freq > max_freq)
		min_freq = max_freq;

	/* Clip requested frequency */
	if (local_freq < min_freq)
		local_freq = min_freq;

	if (local_freq > max_freq)
		local_freq = max_freq;

	/* set the final frequency */
	rounded_rate = platform->clk_round_rate(dev, local_freq);

	/* Check for duplicate request */
	if (rounded_rate == g->last_freq)
		return 0;

	if (g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_GPCCLK) == rounded_rate)
		*freq = rounded_rate;
	else {
		g->ops.clk.set_rate(g, CTRL_CLK_DOMAIN_GPCCLK, rounded_rate);
		*freq = g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_GPCCLK);
	}

	g->last_freq = *freq;

	/* postscale will only scale emc (dram clock) if evaluating
	 * gk20a_tegra_get_emc_rate() produces a new or different emc
	 * target because the load or_and gpufreq has changed */
	if (platform->postscale)
		platform->postscale(dev, rounded_rate);

	return 0;
}

/*
 * update_load_estimate_busy_cycles(dev)
 *
 * Update load estimate using pmu idle counters. Result is normalised
 * based on the time it was asked last time.
 */

static void update_load_estimate_busy_cycles(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_scale_profile *profile = g->scale_profile;
	unsigned long dt;
	u32 busy_cycles_norm;
	ktime_t t;

	t = ktime_get();
	dt = ktime_us_delta(t, profile->last_event_time);

	profile->dev_stat.total_time = dt;
	profile->last_event_time = t;
	nvgpu_pmu_busy_cycles_norm(g, &busy_cycles_norm);
	profile->dev_stat.busy_time =
		(busy_cycles_norm * dt) / PMU_BUSY_CYCLES_NORM_MAX;
}

/*
 * gk20a_scale_suspend(dev)
 *
 * This function informs devfreq of suspend
 */

void gk20a_scale_suspend(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct devfreq *devfreq = l->devfreq;

	if (!devfreq)
		return;

	devfreq_suspend_device(devfreq);
}

/*
 * gk20a_scale_resume(dev)
 *
 * This functions informs devfreq of resume
 */

void gk20a_scale_resume(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct devfreq *devfreq = l->devfreq;

	if (!devfreq)
		return;

	g->last_freq = 0;
	devfreq_resume_device(devfreq);
}

/*
 * gk20a_scale_get_dev_status(dev, *stat)
 *
 * This function queries the current device status.
 */

static int gk20a_scale_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_scale_profile *profile = g->scale_profile;
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	/* inform edp about new constraint */
	if (platform->prescale)
		platform->prescale(dev);

	/* Make sure there are correct values for the current frequency */
	profile->dev_stat.current_frequency =
				g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_GPCCLK);

	/* Update load estimate */
	update_load_estimate_busy_cycles(dev);

	/* Copy the contents of the current device status */
	*stat = profile->dev_stat;

	/* Finally, clear out the local values */
	profile->dev_stat.total_time = 0;
	profile->dev_stat.busy_time = 0;

	return 0;
}

/*
 * get_cur_freq(struct device *dev, unsigned long *freq)
 *
 * This function gets the current GPU clock rate.
 */

static int get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct gk20a *g = get_gk20a(dev);
	*freq = g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_GPCCLK);
	return 0;
}


/*
 * gk20a_scale_init(dev)
 */

void gk20a_scale_init(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = platform->g;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct gk20a_scale_profile *profile;
	int err;

	if (g->scale_profile)
		return;

	if (!platform->devfreq_governor && !platform->qos_notify)
		return;

	profile = nvgpu_kzalloc(g, sizeof(*profile));
	if (!profile)
		return;

	profile->dev = dev;
	profile->dev_stat.busy = false;

	/* Create frequency table */
	err = gk20a_scale_make_freq_table(profile);
	if (err || !profile->devfreq_profile.max_state)
		goto err_get_freqs;

	profile->qos_min_freq = 0;
	profile->qos_max_freq = UINT_MAX;

	/* Store device profile so we can access it if devfreq governor
	 * init needs that */
	g->scale_profile = profile;

	if (platform->devfreq_governor) {
		struct devfreq *devfreq;

		profile->devfreq_profile.initial_freq =
			profile->devfreq_profile.freq_table[0];
		profile->devfreq_profile.target = gk20a_scale_target;
		profile->devfreq_profile.get_dev_status =
			gk20a_scale_get_dev_status;
		profile->devfreq_profile.get_cur_freq = get_cur_freq;
		profile->devfreq_profile.polling_ms = 25;

		devfreq = devm_devfreq_add_device(dev,
					&profile->devfreq_profile,
					platform->devfreq_governor, NULL);

		if (IS_ERR_OR_NULL(devfreq))
			devfreq = NULL;

		l->devfreq = devfreq;
	}

#ifdef CONFIG_GK20A_PM_QOS
	/* Should we register QoS callback for this device? */
	if (platform->qos_notify) {
		profile->qos_notify_block.notifier_call =
					platform->qos_notify;

		pm_qos_add_min_notifier(PM_QOS_GPU_FREQ_BOUNDS,
					&profile->qos_notify_block);
		pm_qos_add_max_notifier(PM_QOS_GPU_FREQ_BOUNDS,
					&profile->qos_notify_block);
	}
#endif

	return;

err_get_freqs:
	nvgpu_kfree(g, profile);
}

void gk20a_scale_exit(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = platform->g;

#ifdef CONFIG_GK20A_PM_QOS
	if (platform->qos_notify) {
		pm_qos_remove_min_notifier(PM_QOS_GPU_FREQ_BOUNDS,
				&g->scale_profile->qos_notify_block);
		pm_qos_remove_max_notifier(PM_QOS_GPU_FREQ_BOUNDS,
				&g->scale_profile->qos_notify_block);
	}
#endif

	nvgpu_kfree(g, g->scale_profile);
	g->scale_profile = NULL;
}

/*
 * gk20a_scale_hw_init(dev)
 *
 * Initialize hardware portion of the device
 */

void gk20a_scale_hw_init(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	/* make sure that scaling has bee initialised */
	if (!profile)
		return;

	profile->dev_stat.total_time = 0;
	profile->last_event_time = ktime_get();
}
