/*
 * Copyright (c) 2012-2020, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * Power-on-demand clock scaling for nvhost devices
 *
 * devfreq calls nvhost_pod_estimate_freq() for estimating the new
 * frequency for the device. The clocking is done using the load of the device
 * is estimated using the busy times from the device profile. This information
 * indicates if the device frequency should be altered.
 *
 */

#include <linux/devfreq.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/clk/tegra.h>
#include <linux/module.h>

#define CREATE_TRACE_POINTS
#include <trace/events/nvhost_podgov.h>

#include "governor.h"

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define GET_TARGET_FREQ_DONTSCALE	1

#ifdef CONFIG_DEVFREQ_GOV_POD_SCALING_HISTORY_BUFFER_SIZE_MAX
#define MAX_HISTORY_BUF_SIZE		\
	CONFIG_DEVFREQ_GOV_POD_SCALING_HISTORY_BUFFER_SIZE_MAX
#else
#define MAX_HISTORY_BUF_SIZE	0
#endif

static void podgov_enable(struct devfreq *df, int enable);
static void podgov_set_user_ctl(struct devfreq *df, int enable);

static struct devfreq_governor nvhost_podgov;

/*******************************************************************************
 * podgov_info_rec - gr3d scaling governor specific parameters
 ******************************************************************************/

struct podgov_info_rec {
	int			enable;
	int                     suspended;
	int			init;

	ktime_t			last_scale;

	unsigned int		p_block_window;
	unsigned int		p_smooth;
	int			p_damp;
	int			p_load_max;
	int			p_load_target;
	int			p_bias;
	unsigned int		p_user;
	unsigned int		p_freq_request;

	unsigned long		cycles_norm;
	unsigned long		cycles_avg;

	unsigned long		*cycles_history_buf;
	int			p_history_buf_size;
	int			history_next;
	int			history_count;
	unsigned long		recent_high;

	unsigned long		rt_load;

	int			adjustment_type;
	unsigned long		adjustment_frequency;

	struct devfreq		*power_manager;
	struct dentry		*debugdir;

	unsigned long		*freqlist;
	int			freq_count;

	int			freq_avg;

	struct kobj_attribute	enable_3d_scaling_attr;
	struct kobj_attribute	user_attr;
	struct kobj_attribute	freq_request_attr;

	struct mutex		lock;
};

/*******************************************************************************
 * Adjustment type is used to tell the source that requested frequency re-
 * estimation. Type ADJUSTMENT_LOCAL indicates that the re-estimation was
 * initiated by the governor itself. This happens when one of the worker
 * threads want to adjust the frequency.
 *
 * ADJUSTMENT_DEVICE_REQ (default value) indicates that the adjustment was
 * initiated by a device event.
 ******************************************************************************/

enum podgov_adjustment_type {
	ADJUSTMENT_LOCAL = 0,
	ADJUSTMENT_DEVICE_REQ = 1
};


/*******************************************************************************
 * scaling_limit(df, freq)
 *
 * Limit the given frequency
 ******************************************************************************/

static void scaling_limit(struct devfreq *df, unsigned long *freq)
{
	if (*freq < df->min_freq)
		*freq = df->min_freq;
	else if (*freq > df->max_freq)
		*freq = df->max_freq;
}

/*******************************************************************************
 * podgov_enable(dev, enable)
 *
 * This function enables (enable=1) or disables (enable=0) the automatic scaling
 * of the device. If the device is disabled, the device's clock is set to its
 * maximum.
 ******************************************************************************/

static void podgov_enable(struct devfreq *df, int enable)
{
	struct device *dev = df->dev.parent;
	struct podgov_info_rec *podgov = df->data;
	bool polling;

	/* make sure the device is alive before doing any scaling */
	pm_runtime_get_noresume(dev);

	mutex_lock(&podgov->lock);
	mutex_lock(&df->lock);

	trace_podgov_enabled(df->dev.parent, enable);

	/* bad configuration. quit. */
	if (df->min_freq == df->max_freq) {
		mutex_unlock(&df->lock);
		mutex_unlock(&podgov->lock);
		pm_runtime_put(dev);
		return;
	}

	/* store the enable information */
	podgov->enable = enable;

	/* skip local adjustment if we are enabling or the device is
	 * suspended */
	if (!enable && pm_runtime_active(dev)) {
		/* full speed */
		podgov->adjustment_frequency = df->max_freq;
		podgov->adjustment_type = ADJUSTMENT_LOCAL;
		update_devfreq(df);
	}

	polling = podgov->enable && !podgov->p_user;

	/* Need to unlock to call devfreq_monitor_suspend/resume()
	 * still holding podgov->lock to guarantee atomicity
	 */
	mutex_unlock(&df->lock);

	if (polling)
		devfreq_monitor_resume(df);
	else
		devfreq_monitor_suspend(df);

	mutex_unlock(&podgov->lock);
	pm_runtime_put(dev);
}

/*****************************************************************************
 * podgov_set_user_ctl(dev, user)
 *
 * This function enables or disables user control of the gpu. If user control
 * is enabled, setting the freq_request controls the gpu frequency, and other
 * gpu scaling mechanisms are disabled.
 ******************************************************************************/

static void podgov_set_user_ctl(struct devfreq *df, int user)
{
	struct device *dev = df->dev.parent;
	struct podgov_info_rec *podgov = df->data;
	int old_user;
	bool polling;

	/* make sure the device is alive before doing any scaling */
	pm_runtime_get_noresume(dev);

	mutex_lock(&podgov->lock);
	mutex_lock(&df->lock);

	trace_podgov_set_user_ctl(df->dev.parent, user);

	/* store the new user value */
	old_user = podgov->p_user;
	podgov->p_user = user;

	/* skip scaling, if scaling (or the whole device) is turned off
	 * - or the scaling already was in user mode */
	if (pm_runtime_active(dev) && podgov->enable && user && !old_user) {
		/* write request */
		podgov->adjustment_frequency = podgov->p_freq_request;
		podgov->adjustment_type = ADJUSTMENT_LOCAL;
		update_devfreq(df);
	}

	polling = podgov->enable && !podgov->p_user;

	/* Need to unlock to call devfreq_monitor_suspend/resume()
	 * still holding podgov->lock to guarantee atomicity
	 */
	mutex_unlock(&df->lock);

	if (polling)
		devfreq_monitor_resume(df);
	else
		devfreq_monitor_suspend(df);

	mutex_unlock(&podgov->lock);
	pm_runtime_put(dev);
}

/*****************************************************************************
 * podgov_set_freq_request(dev, user)
 *
 * Set the current freq request. If scaling is enabled, and podgov user space
 * control is enabled, this will set the gpu frequency.
 ******************************************************************************/

static void podgov_set_freq_request(struct devfreq *df, int freq_request)
{
	struct device *dev = df->dev.parent;
	struct podgov_info_rec *podgov;

	/* make sure the device is alive before doing any scaling */
	pm_runtime_get_noresume(dev);

	mutex_lock(&df->lock);

	podgov = df->data;

	trace_podgov_set_freq_request(df->dev.parent, freq_request);

	podgov->p_freq_request = freq_request;

	/* update the request only if podgov is enabled, device is turned on
	 * and the scaling is in user mode */
	if (podgov->enable && podgov->p_user &&
	    pm_runtime_active(dev)) {
		podgov->adjustment_frequency = freq_request;
		podgov->adjustment_type = ADJUSTMENT_LOCAL;
		update_devfreq(df);
	}

	mutex_unlock(&df->lock);
	pm_runtime_put(dev);
}


/*******************************************************************************
 * freq = scaling_state_check(df, time)
 *
 * This handler is called to adjust the frequency of the device. The function
 * returns the desired frequency for the clock. If there is no need to tune the
 * clock immediately, 0 is returned.
 ******************************************************************************/

static unsigned long scaling_state_check(struct devfreq *df, ktime_t time)
{
	struct podgov_info_rec *pg = df->data;
	struct devfreq_dev_status *ds = &df->last_status;
	unsigned long dt, busyness, rt_load = pg->rt_load;
	long max_boost, damp, freq, boost, res;

	dt = (unsigned long) ktime_us_delta(time, pg->last_scale);
	if (dt < pg->p_block_window || df->previous_freq == 0)
		return 0;

	/* convert to mhz to avoid overflow */
	freq = df->previous_freq / 1000000;
	max_boost = (df->max_freq/3) / 1000000;

	/* calculate and trace load */
	busyness = 1000ULL * pg->cycles_avg / ds->current_frequency;

	/* consider recent high load if required */
	if (pg->p_history_buf_size && pg->history_count)
		busyness = 1000ULL * pg->recent_high / ds->current_frequency;

	trace_podgov_load(df->dev.parent, rt_load);
	trace_podgov_busy(df->dev.parent, busyness);

	damp = pg->p_damp;

	if (rt_load > pg->p_load_max) {
		/* if too busy, scale up max/3, do not damp */
		boost = max_boost;
		damp = 10;
	} else {
		/* boost = bias * freq * (busyness - target)/target */
		boost = busyness - pg->p_load_target;
		boost *= (pg->p_bias * freq);
		boost /= (100 * pg->p_load_target);

		/* clamp to max boost */
		boost = (boost < max_boost) ? boost : max_boost;
	}

	/* calculate new request */
	res = freq + boost;

	/* Maintain average request */
	pg->freq_avg = (pg->freq_avg * pg->p_smooth) + res;
	pg->freq_avg /= (pg->p_smooth+1);

	/* Applying damping to frequencies */
	res = ((damp * res) + ((10 - damp)*pg->freq_avg)) / 10;

	/* Convert to hz, limit, and apply */
	res = res * 1000000;
	scaling_limit(df, &res);
	trace_podgov_scaling_state_check(df->dev.parent,
					 df->previous_freq, res);
	return res;
}

/*******************************************************************************
 * freqlist_up(podgov, target, steps)
 *
 * This function determines the frequency that is "steps" frequency steps
 * higher compared to the target frequency.
 ******************************************************************************/

static int freqlist_up(struct podgov_info_rec *podgov, unsigned long target,
			int steps)
{
	int i, pos;

	for (i = 0; i < podgov->freq_count; i++)
		if (podgov->freqlist[i] >= target)
			break;

	pos = min(podgov->freq_count - 1, i + steps);
	return podgov->freqlist[pos];
}

/*******************************************************************************
 * debugfs interface for controlling 3d clock scaling on the fly
 ******************************************************************************/

#ifdef CONFIG_DEBUG_FS

static void nvhost_scale_emc_debug_init(struct devfreq *df)
{
	struct podgov_info_rec *podgov = df->data;
	struct dentry *f;
	char dirname[128];

	snprintf(dirname, sizeof(dirname), "%s_scaling",
		to_platform_device(df->dev.parent)->name);

	if (!podgov)
		return;

	podgov->debugdir = debugfs_create_dir(dirname, NULL);
	if (!podgov->debugdir) {
		pr_err("podgov: can\'t create debugfs directory\n");
		f = debugfs_lookup(dirname, NULL);
		if (f)
			pr_err("%s debugfs already created\n", dirname);
		panic("nvhost_scale_emc_debug_init");
		return;
	}

#define CREATE_PODGOV_FILE(fname) \
	do {\
		f = debugfs_create_u32(#fname, S_IRUGO | S_IWUSR, \
			podgov->debugdir, &podgov->p_##fname); \
		if (NULL == f) { \
			pr_err("podgov: can\'t create file " #fname "\n"); \
			return; \
		} \
	} while (0)

	CREATE_PODGOV_FILE(block_window);
	CREATE_PODGOV_FILE(load_max);
	CREATE_PODGOV_FILE(load_target);
	CREATE_PODGOV_FILE(bias);
	CREATE_PODGOV_FILE(damp);
	CREATE_PODGOV_FILE(smooth);
#undef CREATE_PODGOV_FILE
}

static void nvhost_scale_emc_debug_deinit(struct devfreq *df)
{
	struct podgov_info_rec *podgov = df->data;

	debugfs_remove_recursive(podgov->debugdir);
}

#else
static void nvhost_scale_emc_debug_init(struct devfreq *df)
{
	(void)df;
}

static void nvhost_scale_emc_debug_deinit(struct devfreq *df)
{
	(void)df;
}
#endif

/*******************************************************************************
 * sysfs interface for enabling/disabling 3d scaling
 ******************************************************************************/

static ssize_t enable_3d_scaling_show(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      char *buf)
{
	struct podgov_info_rec *podgov = container_of(attr,
						      struct podgov_info_rec,
						      enable_3d_scaling_attr);
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", podgov->enable);

	return res;
}

static ssize_t enable_3d_scaling_store(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       const char *buf, size_t count)
{
	struct podgov_info_rec *podgov = container_of(attr,
						      struct podgov_info_rec,
						      enable_3d_scaling_attr);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	podgov_enable(podgov->power_manager, val);

	return count;
}

/*******************************************************************************
 * sysfs interface for user space control
 * user = [0,1] disables / enabled user space control
 * freq_request is the sysfs node user space writes frequency requests to
 ******************************************************************************/

static ssize_t user_show(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 char *buf)
{
	struct podgov_info_rec *podgov =
		container_of(attr, struct podgov_info_rec, user_attr);
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", podgov->p_user);

	return res;
}

static ssize_t user_store(struct kobject *kobj,
			  struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	struct podgov_info_rec *podgov =
		container_of(attr, struct podgov_info_rec, user_attr);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	podgov_set_user_ctl(podgov->power_manager, val);

	return count;
}

static ssize_t freq_request_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	struct podgov_info_rec *podgov =
		container_of(attr, struct podgov_info_rec, freq_request_attr);
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", podgov->p_freq_request);

	return res;
}

static ssize_t freq_request_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	struct podgov_info_rec *podgov =
		container_of(attr, struct podgov_info_rec, freq_request_attr);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	podgov_set_freq_request(podgov->power_manager, val);

	return count;
}

/*******************************************************************************
 * nvhost_pod_estimate_freq(df, freq)
 *
 * This function is called for re-estimating the frequency. The function is
 * called in three conditions:
 *
 *  (1) Internal request to change the frequency. In this case a new clock
 *      target is immediately set for the device.
 *  (2) Call from the client (something has happened and re-estimation
 *      is required).
 *  (3) Some other reason (i.e. periodic call)
 *
 ******************************************************************************/

static int nvhost_pod_estimate_freq(struct devfreq *df,
				    unsigned long *freq)
{
	struct podgov_info_rec *pg = df->data;
	struct devfreq_dev_status *ds;
	int err, i;
	int buf_size = pg->p_history_buf_size;
	int buf_next = pg->history_next;
	int buf_count = pg->history_count;
	unsigned long *cycles_buffer = pg->cycles_history_buf;
	ktime_t now;
	unsigned long long norm_load;

	/* If the device is suspended, clear the history and set frequency to
	 * min freq.
	 */
	if (pg->suspended) {
		*freq = df->min_freq;
		pg->last_scale = ktime_get();
		i = 0;
		for (; i < MAX_HISTORY_BUF_SIZE; i++)
			pg->cycles_history_buf[i] = 0;
		pg->history_count = 0;
		pg->history_next = 0;
		pg->recent_high = 0;
		pg->freq_avg = 0;
		return 0;
	}

	/* Ensure maximal clock when scaling is disabled */
	if (!pg->enable) {
		*freq = df->max_freq;
		if (*freq == df->previous_freq)
			return GET_TARGET_FREQ_DONTSCALE;
		else
			return 0;
	}

	if (pg->p_user) {
		*freq = pg->p_freq_request;
		return 0;
	}

	err = devfreq_update_stats(df);
	if (err)
		return err;

	ds = &df->last_status;

	if (ds->total_time == 0) {
		*freq = ds->current_frequency;
		return 0;
	}

	now = ktime_get();

	/* Local adjustments (i.e. requests from kernel threads) are
	 * handled here */

	if (pg->adjustment_type == ADJUSTMENT_LOCAL) {

		pg->adjustment_type = ADJUSTMENT_DEVICE_REQ;

		/* Do not do unnecessary scaling */
		scaling_limit(df, &pg->adjustment_frequency);

		trace_podgov_estimate_freq(df->dev.parent,
					   df->previous_freq,
					   pg->adjustment_frequency);

		*freq = pg->adjustment_frequency;
		return 0;
	}

	/* Sustain local variables */
	norm_load = (u64)ds->current_frequency * ds->busy_time / ds->total_time;
	pg->cycles_norm = norm_load;
	pg->cycles_avg = ((u64)pg->cycles_avg * pg->p_smooth + norm_load) /
		(pg->p_smooth + 1);
	pg->rt_load = 1000ULL * ds->busy_time / ds->total_time;

	/* Update history of normalized cycle counts and recent highest count */
	if (buf_size) {
		if (buf_count == buf_size) {
			pg->recent_high = 0;
			i = (buf_next + 1) % buf_size;
			for (; i != buf_next; i = (i + 1) % buf_size) {
				if (cycles_buffer[i] > pg->recent_high)
					pg->recent_high = cycles_buffer[i];
			}
		}
		cycles_buffer[buf_next] = norm_load;
		pg->history_next = (buf_next + 1) % buf_size;
		if (buf_count < buf_size)
			pg->history_count += 1;
		if (norm_load > pg->recent_high)
			pg->recent_high = norm_load;
	}

	*freq = scaling_state_check(df, now);

	if (!(*freq)) {
		*freq = ds->current_frequency;
		return 0;
	}

	if ((*freq = freqlist_up(pg, *freq, 0)) == ds->current_frequency)
		return 0;

	pg->last_scale = now;

	trace_podgov_estimate_freq(df->dev.parent, df->previous_freq, *freq);


	return 0;
}

/*******************************************************************************
 * nvhost_pod_init(struct devfreq *df)
 *
 * Governor initialisation.
 ******************************************************************************/

static int nvhost_pod_init(struct devfreq *df)
{
	struct podgov_info_rec *podgov;
	struct platform_device *d = to_platform_device(df->dev.parent);
	ktime_t now = ktime_get();

	struct kobj_attribute *attr = NULL;

	podgov = kzalloc(sizeof(struct podgov_info_rec), GFP_KERNEL);
	if (!podgov)
		goto err_alloc_podgov;

	podgov->cycles_history_buf =
		kzalloc(sizeof(unsigned long) * MAX_HISTORY_BUF_SIZE,
			GFP_KERNEL);
	if (!podgov->cycles_history_buf)
		goto err_alloc_history_buffer;

	podgov->p_history_buf_size =
		MAX_HISTORY_BUF_SIZE < 100 ? MAX_HISTORY_BUF_SIZE : 100;
	podgov->history_count = 0;
	podgov->history_next = 0;
	podgov->recent_high = 0;

	df->data = (void *)podgov;

	/* Set scaling parameter defaults */
	podgov->enable = 1;
	podgov->suspended = 0;

	podgov->p_load_max = 900;
	podgov->p_load_target = 700;
	podgov->p_bias = 80;
	podgov->p_smooth = 10;
	podgov->p_damp = 7;
	podgov->p_block_window = 50000;

	podgov->adjustment_type = ADJUSTMENT_DEVICE_REQ;
	podgov->p_user = 0;

	/* Reset clock counters */
	podgov->last_scale = now;

	podgov->power_manager = df;

	mutex_init(&podgov->lock);

	attr = &podgov->enable_3d_scaling_attr;
	attr->attr.name = "enable_3d_scaling";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = enable_3d_scaling_show;
	attr->store = enable_3d_scaling_store;
	sysfs_attr_init(&attr->attr);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_enable_sysfs_entry;

	attr = &podgov->freq_request_attr;
	attr->attr.name = "freq_request";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = freq_request_show;
	attr->store = freq_request_store;
	sysfs_attr_init(&attr->attr);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_request_sysfs_entry;

	attr = &podgov->user_attr;
	attr->attr.name = "user";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = user_show;
	attr->store = user_store;
	sysfs_attr_init(&attr->attr);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_user_sysfs_entry;

	podgov->freq_count = df->profile->max_state;
	podgov->freqlist = df->profile->freq_table;
	if (!podgov->freq_count || !podgov->freqlist)
		goto err_get_freqs;

	/* store the limits */
	df->min_freq = podgov->freqlist[0];
	df->max_freq = podgov->freqlist[podgov->freq_count - 1];
	podgov->p_freq_request = df->max_freq;

	podgov->freq_avg = 0;

	nvhost_scale_emc_debug_init(df);

	devfreq_monitor_start(df);

	return 0;

err_get_freqs:
	sysfs_remove_file(&df->dev.parent->kobj, &podgov->user_attr.attr);
err_create_user_sysfs_entry:
	sysfs_remove_file(&df->dev.parent->kobj,
			  &podgov->freq_request_attr.attr);
err_create_request_sysfs_entry:
	sysfs_remove_file(&df->dev.parent->kobj,
			  &podgov->enable_3d_scaling_attr.attr);
err_create_enable_sysfs_entry:
	dev_err(&d->dev, "failed to create sysfs attributes");
	kfree(podgov->cycles_history_buf);
err_alloc_history_buffer:
	kfree(podgov);
err_alloc_podgov:
	return -ENOMEM;
}

/*******************************************************************************
 * nvhost_pod_exit(struct devfreq *df)
 *
 * Clean up governor data structures
 ******************************************************************************/

static void nvhost_pod_exit(struct devfreq *df)
{
	struct podgov_info_rec *podgov = df->data;

	devfreq_monitor_stop(df);

	sysfs_remove_file(&df->dev.parent->kobj, &podgov->user_attr.attr);
	sysfs_remove_file(&df->dev.parent->kobj,
			  &podgov->freq_request_attr.attr);
	sysfs_remove_file(&df->dev.parent->kobj,
			  &podgov->enable_3d_scaling_attr.attr);

	nvhost_scale_emc_debug_deinit(df);
	kfree(podgov->cycles_history_buf);
	kfree(podgov);
}

/******************************************************************************
 * nvhost_pod_suspend(struct devfreq *df)
 *
 * Suspends the governor.
 *****************************************************************************/

static void nvhost_pod_suspend(struct devfreq *df)
{
	// Record suspension in our own data structure because we'll have to
	// erase and restore devfreq's for this to work.
	struct podgov_info_rec *pg = df->data;

	pg->suspended = 1;

	// Update frequency for the final time before going into suspension.
	mutex_lock(&df->lock);
	df->suspended = false;
	update_devfreq(df);
	df->suspended = true;
	mutex_unlock(&df->lock);
	devfreq_monitor_suspend(df);
}

/******************************************************************************
 * nvhost_pod_resume(struct devfreq *df)
 *
 * Resumes the governor.
 *****************************************************************************/

static void nvhost_pod_resume(struct devfreq *df)
{
	// Update our data structure's suspension field
	struct podgov_info_rec *pg = df->data;

	pg->suspended = 0;

	// Resume
	devfreq_monitor_resume(df);
}

static int nvhost_pod_event_handler(struct devfreq *df,
			unsigned int event, void *data)
{
	int ret = 0;

	switch (event) {
	case DEVFREQ_GOV_START:
		ret = nvhost_pod_init(df);
		break;
	case DEVFREQ_GOV_STOP:
		nvhost_pod_exit(df);
		break;
	case DEVFREQ_GOV_INTERVAL:
		devfreq_interval_update(df, (unsigned int *)data);
		break;
	case DEVFREQ_GOV_SUSPEND:
		nvhost_pod_suspend(df);
		break;
	case DEVFREQ_GOV_RESUME:
		nvhost_pod_resume(df);
		break;
	default:
		break;
	}

	return ret;
}

static struct devfreq_governor nvhost_podgov = {
	.name = "nvhost_podgov",
	.get_target_freq = nvhost_pod_estimate_freq,
	.event_handler = nvhost_pod_event_handler,
};


static int __init podgov_init(void)
{
	return devfreq_add_governor(&nvhost_podgov);
}

static void __exit podgov_exit(void)
{
	devfreq_remove_governor(&nvhost_podgov);
	return;
}

/* governor must be registered before initialising client devices */
rootfs_initcall(podgov_init);
module_exit(podgov_exit);
MODULE_LICENSE("GPL");
