/*
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include "governor.h"

struct wmark_gov_param {
	unsigned int		block_window;
	unsigned int		load_target;
	unsigned int		load_max;
	unsigned int		smooth;
	bool			freq_boost_en;
};

struct wmark_gov_info {
	/* probed from the devfreq */
	unsigned long		*freqlist;
	int			freq_count;

	/* algorithm parameters */
	struct wmark_gov_param  param;

	struct kobj_attribute	block_window_attr;
	struct kobj_attribute	load_target_attr;
	struct kobj_attribute	load_max_attr;
	struct kobj_attribute	smooth_attr;
	struct kobj_attribute	freq_boost_en_attr;

	spinlock_t		param_lock;

	/* common data */
	struct devfreq		*df;
	struct platform_device	*pdev;
	struct dentry		*debugdir;

	/* used for ensuring that we do not update frequency too often */
	ktime_t			last_frequency_update;

	/* variable for keeping the average frequency request */
	unsigned long long	average_target_freq;

	/* devfreq notifier_block */
	struct notifier_block nb;
};

static unsigned long freqlist_up(struct wmark_gov_info *wmarkinfo,
			unsigned long curr_freq)
{
	int i, pos;

	for (i = 0; i < wmarkinfo->freq_count; i++)
		if (wmarkinfo->freqlist[i] > curr_freq)
			break;

	pos = min(wmarkinfo->freq_count - 1, i);

	return wmarkinfo->freqlist[pos];
}

static unsigned long freqlist_down(struct wmark_gov_info *wmarkinfo,
			unsigned long curr_freq)
{
	int i, pos;

	for (i = wmarkinfo->freq_count - 1; i >= 0; i--)
		if (wmarkinfo->freqlist[i] < curr_freq)
			break;

	pos = max(0, i);
	return wmarkinfo->freqlist[pos];
}

static unsigned long freqlist_round(struct wmark_gov_info *wmarkinfo,
				    unsigned long freq)
{
	int i, pos;

	for (i = 0; i < wmarkinfo->freq_count; i++)
		if (wmarkinfo->freqlist[i] >= freq)
			break;

	pos = min(wmarkinfo->freq_count - 1, i);
	return wmarkinfo->freqlist[pos];
}


 /*
  * update_watermarks - Re-estimate low and high watermarks
  *     @df: pointer to the devfreq structure
  *     @current_frequency: current frequency of the device
  *     @ideal_frequency: frequency that would change load to target load.
  *
  * This function updates the devfreq high and low watermarks. Target is
  * to ensure that the interrupts are triggered whenever the load changes
  * enough to make a change to the ideal frequency (given the DVFS table).
  */

static void update_watermarks(struct devfreq *df,
			      unsigned long current_frequency,
			      unsigned long ideal_frequency)
{
	struct wmark_gov_info *wmarkinfo = df->data;
	unsigned long long relation = 0, next_freq = 0;
	unsigned long long current_frequency_khz = current_frequency / 1000;
	unsigned long flags;
	struct wmark_gov_param param;

	/* get governor parameters */
	spin_lock_irqsave(&wmarkinfo->param_lock, flags);

	param = wmarkinfo->param;

	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	if (ideal_frequency == wmarkinfo->freqlist[0]) {
		/* disable the low watermark if we are at lowest clock */
		df->profile->set_low_wmark(df->dev.parent, 0);
	} else {
		/* calculate the low threshold; what is the load value
		 * at which we would go into lower frequency given the
		 * that we are running at the new frequency? */
		next_freq = freqlist_down(wmarkinfo, ideal_frequency);
		relation = ((next_freq / current_frequency_khz) *
			param.load_target) / 1000;
		df->profile->set_low_wmark(df->dev.parent, relation);
	}

	if (ideal_frequency ==
	    wmarkinfo->freqlist[wmarkinfo->freq_count - 1]) {
		/* disable the high watermark if we are at highest clock */
		df->profile->set_high_wmark(df->dev.parent, 1000);
	} else {
		/* calculate the high threshold; what is the load value
		 * at which we would go into highest frequency given the
		 * that we are running at the new frequency? */
		next_freq = freqlist_up(wmarkinfo, ideal_frequency);
		relation = ((next_freq / current_frequency_khz) *
			param.load_target) / 1000;
		relation = min_t(unsigned long long, param.load_max,
			       relation);
		df->profile->set_high_wmark(df->dev.parent, relation);
	}
}

static int devfreq_watermark_target_freq(struct devfreq *df,
					 unsigned long *freq)
{
	struct wmark_gov_info *wmarkinfo = df->data;
	struct devfreq_dev_status dev_stat;
	unsigned long long load, relation, ideal_freq;
	ktime_t current_time = ktime_get();
	s64 dt = ktime_us_delta(current_time, wmarkinfo->last_frequency_update);
	int err;
	unsigned long flags;

	struct wmark_gov_param param;

	err = df->profile->get_dev_status(df->dev.parent, &dev_stat);
	if (err < 0)
		return err;

	/* use current frequency by default */
	*freq = dev_stat.current_frequency;

	/* quit now if we are getting calls too often */
	if (!dev_stat.total_time)
		return 0;

	/* get governor parameters */
	spin_lock_irqsave(&wmarkinfo->param_lock, flags);

	param = wmarkinfo->param;

	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	/* calculate first load and relation load/load_target */
	load = (dev_stat.busy_time * 1000) / dev_stat.total_time;

	/* if we cross load max...  */
	if (param.freq_boost_en && load >= param.load_max) {
		/* we go directly to the highest frequency. depending
		 * on frequency table we might never go higher than
		 * the current frequency (i.e. load should be over 100%
		 * to make relation push to the next frequency). */
		ideal_freq = wmarkinfo->freqlist[wmarkinfo->freq_count - 1];
	} else {
		/* otherwise, based on relation between current load and
		 * load target we calculate the "ideal" frequency
		 * where we would be just at the target */
		relation = (load * 1000) / param.load_target;
		ideal_freq = relation * (dev_stat.current_frequency / 1000);

		/* round this frequency */
		ideal_freq = freqlist_round(wmarkinfo, ideal_freq);
	}

	/* update average target frequency */
	wmarkinfo->average_target_freq =
		(param.smooth * wmarkinfo->average_target_freq +
		 ideal_freq) / (param.smooth + 1);

	/* do not scale too often */
	if (dt < param.block_window)
		return 0;

	/* update the frequency */
	*freq = freqlist_round(wmarkinfo, wmarkinfo->average_target_freq);

	/* check if frequency actually got updated */
	if (*freq == dev_stat.current_frequency)
		return 0;

	/* enable hysteresis - frequency is updated */
	wmarkinfo->last_frequency_update = current_time;

	return 0;
}

static ssize_t block_window_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	ssize_t res;
	unsigned int val;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			block_window_attr);

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	val = wmarkinfo->param.block_window;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	res = snprintf(buf, PAGE_SIZE, "%u\n", val);

	return res;
}

static ssize_t block_window_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	unsigned long val = 0;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			block_window_attr);

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	wmarkinfo->param.block_window = val;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	return count;
}

static ssize_t load_target_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	ssize_t res;
	unsigned int val;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			load_target_attr);

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	val = wmarkinfo->param.load_target;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	res = snprintf(buf, PAGE_SIZE, "%u\n", val);

	return res;
}

static ssize_t load_target_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	unsigned long val = 0;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			load_target_attr);

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	wmarkinfo->param.load_target = val;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	return count;
}

static ssize_t load_max_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	ssize_t res;
	unsigned int val;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			load_max_attr);

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	val = wmarkinfo->param.load_max;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	res = snprintf(buf, PAGE_SIZE, "%u\n", val);

	return res;
}

static ssize_t load_max_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	unsigned long val = 0;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			load_max_attr);

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	wmarkinfo->param.load_max = val;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	return count;
}

static ssize_t smooth_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	ssize_t res;
	unsigned int val;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			smooth_attr);

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	val = wmarkinfo->param.smooth;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	res = snprintf(buf, PAGE_SIZE, "%u\n", val);

	return res;
}

static ssize_t smooth_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	unsigned long val = 0;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			smooth_attr);

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	wmarkinfo->param.smooth = val;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	return count;
}

static ssize_t freq_boost_en_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	ssize_t res;
	bool val;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			freq_boost_en_attr);

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	val = wmarkinfo->param.freq_boost_en;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	res = snprintf(buf, PAGE_SIZE, "%d\n", val);

	return res;
}

static ssize_t freq_boost_en_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	struct wmark_gov_info *wmarkinfo = NULL;
	unsigned long val = 0;
	unsigned long flags;

	wmarkinfo = container_of(attr,
			struct wmark_gov_info,
			freq_boost_en_attr);

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	spin_lock_irqsave(&wmarkinfo->param_lock, flags);
	wmarkinfo->param.freq_boost_en = !!val;
	spin_unlock_irqrestore(&wmarkinfo->param_lock, flags);

	return count;
}

#define INIT_SYSFS_ATTR_RW(sysfs_name) \
	do { \
		attr->attr.name = #sysfs_name; \
		attr->attr.mode = 0644; \
		attr->show = sysfs_name##_show; \
		attr->store = sysfs_name##_store; \
		sysfs_attr_init(&attr->attr); \
	} while (0)

static int devfreq_watermark_debug_start(struct devfreq *df)
{
	struct wmark_gov_info *wmarkinfo = df->data;
	struct kobj_attribute *attr = NULL;

	if (!wmarkinfo)
		return 0;

	spin_lock_init(&wmarkinfo->param_lock);

	attr = &wmarkinfo->block_window_attr;
	INIT_SYSFS_ATTR_RW(block_window);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_block_window_sysfs_entry;

	attr = &wmarkinfo->load_target_attr;
	INIT_SYSFS_ATTR_RW(load_target);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_load_target_sysfs_entry;

	attr = &wmarkinfo->load_max_attr;
	INIT_SYSFS_ATTR_RW(load_max);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_load_max_sysfs_entry;

	attr = &wmarkinfo->smooth_attr;
	INIT_SYSFS_ATTR_RW(smooth);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_smooth_sysfs_entry;

	attr = &wmarkinfo->freq_boost_en_attr;
	INIT_SYSFS_ATTR_RW(freq_boost_en);
	if (sysfs_create_file(&df->dev.parent->kobj, &attr->attr))
		goto err_create_freq_boost_en_sysfs_entry;

	return 0;

err_create_freq_boost_en_sysfs_entry:
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->smooth_attr.attr);
err_create_smooth_sysfs_entry:
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->load_max_attr.attr);
err_create_load_max_sysfs_entry:
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->load_target_attr.attr);
err_create_load_target_sysfs_entry:
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->block_window_attr.attr);
err_create_block_window_sysfs_entry:

	return -ENOMEM;
}

static void devfreq_watermark_debug_stop(struct devfreq *df)
{
	struct wmark_gov_info *wmarkinfo = df->data;

	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->freq_boost_en_attr.attr);
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->smooth_attr.attr);
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->load_max_attr.attr);
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->load_target_attr.attr);
	sysfs_remove_file(&df->dev.parent->kobj,
			&wmarkinfo->block_window_attr.attr);
}

static int devfreq_watermark_start(struct devfreq *df)
{
	struct wmark_gov_info *wmarkinfo;
	struct platform_device *pdev = to_platform_device(df->dev.parent);
	int ret;

	if (!df->profile->freq_table) {
		dev_err(&pdev->dev, "Frequency table missing\n");
		return -EINVAL;
	}

	wmarkinfo = kzalloc(sizeof(struct wmark_gov_info), GFP_KERNEL);
	if (!wmarkinfo)
		return -ENOMEM;

	df->data = (void *)wmarkinfo;
	wmarkinfo->freqlist = df->profile->freq_table;
	wmarkinfo->freq_count = df->profile->max_state;
	wmarkinfo->param.load_target = 700;
	wmarkinfo->param.load_max = 900;
	wmarkinfo->param.smooth = 10;
	wmarkinfo->param.block_window = 50000;
	wmarkinfo->param.freq_boost_en = true;
	wmarkinfo->df = df;
	wmarkinfo->pdev = pdev;

	ret = devfreq_watermark_debug_start(df);

	return ret;
}

static int devfreq_watermark_notifier_call(struct notifier_block *nb,
			unsigned long event, void *ptr)
{
	struct wmark_gov_info *data
				= container_of(nb, struct wmark_gov_info, nb);
	struct devfreq *df = (struct devfreq *)data->df;
	unsigned long freq = 0;

	switch (event) {
	case DEVFREQ_PRECHANGE:
		break;
	case DEVFREQ_POSTCHANGE:
		/* get device freq. */
		df->profile->get_cur_freq(df->dev.parent, &freq);

		/* update watermarks by current device freq. */
		if (freq)
			update_watermarks(df, freq, freq);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static int devfreq_watermark_event_handler(struct devfreq *df,
			unsigned int event, void *wmark_type)
{
	struct wmark_gov_info *wmarkinfo;
	int ret = 0;
	struct notifier_block *nb;

	switch (event) {
	case DEVFREQ_GOV_START:
	{
		struct devfreq_dev_status dev_stat;
		ret = df->profile->get_dev_status(df->dev.parent, &dev_stat);
		if (ret < 0)
			break;

		ret = devfreq_watermark_start(df);
		if (ret < 0)
			break;

		/* initialize average target freq */
		wmarkinfo = df->data;
		wmarkinfo->average_target_freq = dev_stat.current_frequency;

		update_watermarks(df, dev_stat.current_frequency,
					dev_stat.current_frequency);

		nb = &wmarkinfo->nb;
		nb->notifier_call = devfreq_watermark_notifier_call;
		ret = devm_devfreq_register_notifier(df->dev.parent,
				df, nb, DEVFREQ_TRANSITION_NOTIFIER);
		break;
	}
	case DEVFREQ_GOV_STOP:
		devfreq_watermark_debug_stop(df);

		wmarkinfo = df->data;
		nb = &wmarkinfo->nb;
		devm_devfreq_unregister_notifier(df->dev.parent,
				df, nb, DEVFREQ_TRANSITION_NOTIFIER);

		/* free wmark_gov_info struct */
		if (df->data != NULL) {
			kfree(df->data);
			df->data = NULL;
		}

		break;
	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(df);
		break;
	case DEVFREQ_GOV_RESUME:
	{
		struct devfreq_dev_status dev_stat;
		ret = df->profile->get_dev_status(df->dev.parent, &dev_stat);
		if (ret < 0)
			break;

		/* reset average target freq to current freq */
		wmarkinfo = df->data;
		wmarkinfo->average_target_freq = dev_stat.current_frequency;

		update_watermarks(df, dev_stat.current_frequency,
					dev_stat.current_frequency);
		devfreq_monitor_resume(df);
		break;
	}
	case DEVFREQ_GOV_WMARK:
		mutex_lock(&df->lock);
		update_devfreq(df);
		mutex_unlock(&df->lock);
		break;
	default:
		break;
	}

	return ret;
}

static struct devfreq_governor devfreq_watermark_active = {
	.name = "wmark_active",
	.get_target_freq = devfreq_watermark_target_freq,
	.event_handler = devfreq_watermark_event_handler,
	.interrupt_driven = true,
};


static int __init devfreq_watermark_init(void)
{
	return devfreq_add_governor(&devfreq_watermark_active);
}

static void __exit devfreq_watermark_exit(void)
{
	devfreq_remove_governor(&devfreq_watermark_active);
}

rootfs_initcall(devfreq_watermark_init);
module_exit(devfreq_watermark_exit);
MODULE_LICENSE("GPL v2");
