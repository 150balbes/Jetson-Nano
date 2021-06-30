/*
 * pwm_fan.c fan driver that is controlled by pwm
 *
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Anshul Jain <anshulj@nvidia.com>
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

#include <linux/therm_est.h>
#include <linux/slab.h>
#include <linux/platform_data/pwm_fan.h>
#include <linux/thermal.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/pwm.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/time.h>
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/version.h>

#include "thermal_core.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/clock.h>
#endif

#define USEC_PER_MIN	(60L * USEC_PER_SEC)

/* Based off of max device tree node name length */
#define MAX_PROFILE_NAME_LENGTH	31

static void fan_update_target_pwm(struct fan_dev_data *fan_data, int val)
{
	if (fan_data) {
		fan_data->next_target_pwm = min(val, fan_data->fan_cap_pwm);

		if (fan_data->next_target_pwm != fan_data->fan_cur_pwm)
			queue_delayed_work(fan_data->workqueue,
					&fan_data->fan_ramp_work,
					msecs_to_jiffies(fan_data->step_time));
	}
}

static ssize_t fan_target_pwm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	ret = sprintf(buf, "%d\n", fan_data->next_target_pwm);
	mutex_unlock(&fan_data->fan_state_lock);

	return ret;
}

static ssize_t fan_target_pwm_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, target_pwm = -1;

	ret = sscanf(buf, "%d", &target_pwm);
	if ((ret <= 0) || (!fan_data) || (target_pwm < 0))
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	if (target_pwm > fan_data->fan_cap_pwm)
		target_pwm = fan_data->fan_cap_pwm;
	fan_update_target_pwm(fan_data, target_pwm);
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static ssize_t fan_temp_control_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	ret = sprintf(buf, "%d\n", fan_data->fan_temp_control_flag);
	mutex_unlock(&fan_data->fan_state_lock);

	return ret;
}

static ssize_t fan_temp_control_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, fan_temp_control_flag = 0;

	ret = sscanf(buf, "%d", &fan_temp_control_flag);
	if ((ret <= 0) || (!fan_data))
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	fan_data->fan_temp_control_flag = (fan_temp_control_flag > 0) ? 1 : 0;
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static ssize_t fan_tach_enabled_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, val;

	if (!fan_data)
		return -EINVAL;
	val = atomic_read(&fan_data->tach_enabled);
	ret = sprintf(buf, "%d\n", val);
	return ret;
}

static ssize_t fan_tach_enabled_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, val, tach_enabled;

	ret = kstrtoint(buf, 10, &val);
	if ((ret < 0) || (!fan_data))
		return -EINVAL;

	if (!gpio_is_valid(fan_data->tach_gpio)) {
		dev_err(dev, "Can't find tach_gpio\n");
		return -EPERM;
	}

	tach_enabled = atomic_read(&fan_data->tach_enabled);
	if (val == 1) {
		if (tach_enabled) {
			dev_err(dev, "tach irq is already enabled\n");
			return -EINVAL;
		}
		if (!fan_data->tach_workqueue) {
			dev_err(dev, "tach not initialized\n");
			return -EAGAIN;
		}
		fan_data->irq_count = 0;
		enable_irq(fan_data->tach_irq);
		atomic_set(&fan_data->tach_enabled, 1);
		queue_delayed_work(fan_data->tach_workqueue,
				&(fan_data->fan_tach_work),
				msecs_to_jiffies(fan_data->tach_period));
	} else if (val == 0) {
		if (!tach_enabled) {
			dev_err(dev, "tach irq is already disabled\n");
			return -EINVAL;
		}
		cancel_delayed_work(&fan_data->fan_tach_work);
		disable_irq(fan_data->tach_irq);
		atomic_set(&fan_data->tach_enabled, 0);
		atomic64_set(&fan_data->rpm_measured, 0);
	} else
		return -EINVAL;

	return count;
}

static ssize_t fan_pwm_cap_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int val, ret, target_pwm;

	ret = sscanf(buf, "%d", &val);

	if ((ret <= 0) || (!fan_data))
		return -EINVAL;
	if (val < 0)
		val = 0;

	mutex_lock(&fan_data->fan_state_lock);
	if (val > fan_data->fan_pwm_max)
		val = fan_data->fan_pwm_max;
	fan_data->fan_cap_pwm = val;
	target_pwm = min(fan_data->fan_cap_pwm, fan_data->next_target_pwm);
	fan_update_target_pwm(fan_data, target_pwm);
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static ssize_t fan_pwm_cap_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	ret = sprintf(buf, "%d\n", fan_data->fan_cap_pwm);
	mutex_unlock(&fan_data->fan_state_lock);

	return ret;
}

static ssize_t fan_step_time_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, step_time = 0;

	ret = sscanf(buf, "%d", &step_time);
	if ((ret <= 0) || (!fan_data))
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	fan_data->step_time = step_time;
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static ssize_t fan_cur_pwm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	ret = sprintf(buf, "%d\n", fan_data->fan_cur_pwm);
	mutex_unlock(&fan_data->fan_state_lock);

	return ret;
}

static ssize_t fan_step_time_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	ret = sprintf(buf, "%d\n", fan_data->step_time);
	mutex_unlock(&fan_data->fan_state_lock);

	return ret;
}

static ssize_t fan_pwm_rpm_table_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int i, bytes_written = 0;

	if (!fan_data)
		return -EINVAL;
	bytes_written = sprintf(buf + bytes_written, "%s\n",
				"(Index, RPM, PWM, RRU, RRD)");
	mutex_lock(&fan_data->fan_state_lock);
	for (i = 0; i < fan_data->active_steps; ++i) {
		bytes_written += sprintf(buf + bytes_written,
					"(%d, %d, %d, %d, %d)\n", i,
					fan_data->fan_rpm[i],
					fan_data->fan_pwm[i],
					fan_data->fan_rru[i],
					fan_data->fan_rrd[i]);
	}
	mutex_unlock(&fan_data->fan_state_lock);

	return bytes_written;
}

/*
 * fan_rpm_measured_show - rpm_measured sysfs fops
 * Caller is responsible to wait at least "tach_period" msec after
 * enable "tach_enabled" to get right RPM value.
 */
static ssize_t fan_rpm_measured_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;
	u64 val;

	if (!fan_data)
		return -EINVAL;
	val = atomic64_read(&fan_data->rpm_measured);
	ret = sprintf(buf, "%lld\n", val);
	return ret;
}

static int pwm_fan_get_cur_state(struct thermal_cooling_device *cdev,
						unsigned long *cur_state)
{
	struct fan_dev_data *fan_data = cdev->devdata;

	if (!fan_data)
		return -EINVAL;

	mutex_lock(&fan_data->fan_state_lock);
	*cur_state = fan_data->next_state;
	mutex_unlock(&fan_data->fan_state_lock);
	return 0;
}

static int pwm_fan_set_cur_state(struct thermal_cooling_device *cdev,
						unsigned long cur_state)
{
	struct fan_dev_data *fan_data = cdev->devdata;
	int target_pwm = 0;

	if (!fan_data)
		return -EINVAL;

	mutex_lock(&fan_data->fan_state_lock);

	if (!fan_data->fan_temp_control_flag) {
		mutex_unlock(&fan_data->fan_state_lock);
		return 0;
	}

	if (fan_data->continuous_gov) {
		/*"continuous_therm_gov" used, "cur_state" indicate target pwm value*/
		target_pwm = min(fan_data->fan_cap_pwm, (int)cur_state);
		fan_data->next_target_pwm = target_pwm;
		fan_update_target_pwm(fan_data, target_pwm);

		mutex_unlock(&fan_data->fan_state_lock);
		return 0;
	}

	if (cur_state >= fan_data->active_steps) {
		mutex_unlock(&fan_data->fan_state_lock);
		return -EINVAL;
	}

	fan_data->next_state = cur_state;

	if (fan_data->next_state <= 0) {
		target_pwm = 0;
		if (fan_data->kickstart_en
			&& delayed_work_pending(&fan_data->fan_hyst_work)) {
			cancel_delayed_work(&fan_data->fan_hyst_work);
			fan_data->fan_kickstart = false;
		}
	} else if (!fan_data->fan_cur_pwm && fan_data->kickstart_en) {
		if (fan_data->fan_kickstart) {
			mutex_unlock(&fan_data->fan_state_lock);
			return 0;
		}
		fan_data->fan_kickstart = true;
		target_pwm = fan_data->fan_startup_pwm;
		queue_delayed_work(fan_data->workqueue, &fan_data->fan_hyst_work,
					msecs_to_jiffies(fan_data->fan_startup_time +
						fan_data->step_time));
	} else
		target_pwm = fan_data->fan_pwm[cur_state];

	target_pwm = min(fan_data->fan_cap_pwm, target_pwm);
	fan_update_target_pwm(fan_data, target_pwm);
	mutex_unlock(&fan_data->fan_state_lock);
	return 0;
}

static int pwm_fan_get_max_state(struct thermal_cooling_device *cdev,
						unsigned long *max_state)
{
	struct fan_dev_data *fan_data = cdev->devdata;

	*max_state = fan_data->active_steps;
	return 0;
}

static struct thermal_cooling_device_ops pwm_fan_cooling_ops = {
	.get_max_state = pwm_fan_get_max_state,
	.get_cur_state = pwm_fan_get_cur_state,
	.set_cur_state = pwm_fan_set_cur_state,
};

static int fan_get_rru(int pwm, struct fan_dev_data *fan_data)
{
	int i;

	for (i = 0; i < fan_data->active_steps - 1 ; i++) {
		if ((pwm >= fan_data->fan_pwm[i]) &&
				(pwm < fan_data->fan_pwm[i + 1])) {
			return fan_data->fan_rru[i];
		}
	}
	return fan_data->fan_rru[fan_data->active_steps - 1];
}

static int fan_get_rrd(int pwm, struct fan_dev_data *fan_data)
{
	int i;

	for (i = 0; i < fan_data->active_steps - 1 ; i++) {
		if ((pwm >= fan_data->fan_pwm[i]) &&
				(pwm < fan_data->fan_pwm[i + 1])) {
			return fan_data->fan_rrd[i];
		}
	}
	return fan_data->fan_rrd[fan_data->active_steps - 1];
}

static void set_pwm_duty_cycle(int pwm, struct fan_dev_data *fan_data)
{
	int duty;

	if (fan_data != NULL && fan_data->pwm_dev != NULL) {
		if (pwm == 0) {
			if (fan_data->fan_pwm_polarity == PWM_POLARITY_INVERSED)
				duty = fan_data->pwm_period;
			else
				duty = 0;
		} else {
			if (fan_data->fan_pwm_polarity == PWM_POLARITY_INVERSED)
				duty = fan_data->fan_pwm_max - pwm;
			else
				duty = pwm;
			duty = duty * fan_data->precision_multiplier / MULTIQP;
		}

		pwm_config(fan_data->pwm_dev,
			duty, fan_data->pwm_period);
		pwm_enable(fan_data->pwm_dev);
	} else {
		pr_err("FAN:PWM device or fan data is null\n");
	}
}

static int get_next_higher_pwm(int pwm, struct fan_dev_data *fan_data)
{
	int i;

	if (fan_data->continuous_gov)
		return fan_data->next_target_pwm;

	for (i = 0; i < fan_data->active_steps; i++)
		if (pwm < fan_data->fan_pwm[i])
			return fan_data->fan_pwm[i];

	return fan_data->fan_pwm[fan_data->active_steps - 1];
}

static int get_next_lower_pwm(int pwm, struct fan_dev_data *fan_data)
{
	int i;

	if (fan_data->continuous_gov)
		return fan_data->next_target_pwm;

	for (i = fan_data->active_steps - 1; i >= 0; i--)
		if (pwm > fan_data->fan_pwm[i])
			return fan_data->fan_pwm[i];

	return fan_data->fan_pwm[0];
}

static void fan_ramping_work_func(struct work_struct *work)
{
	int rru, rrd, err;
	int cur_pwm, next_pwm;
	struct delayed_work *dwork = container_of(work, struct delayed_work,
									work);
	struct fan_dev_data *fan_data = container_of(dwork, struct
						fan_dev_data, fan_ramp_work);

	mutex_lock(&fan_data->fan_state_lock);

	cur_pwm = fan_data->fan_cur_pwm;
	rru = fan_get_rru(cur_pwm, fan_data);
	rrd = fan_get_rrd(cur_pwm, fan_data);
	next_pwm = cur_pwm;

	if (fan_data->next_target_pwm > fan_data->fan_cur_pwm) {
		fan_data->fan_cur_pwm = fan_data->fan_cur_pwm + rru;
		next_pwm = min(
				get_next_higher_pwm(cur_pwm, fan_data),
				fan_data->fan_cur_pwm);
		next_pwm = min(fan_data->next_target_pwm, next_pwm);
		next_pwm = min(fan_data->fan_cap_pwm, next_pwm);
	} else if (fan_data->next_target_pwm < fan_data->fan_cur_pwm) {
		fan_data->fan_cur_pwm = fan_data->fan_cur_pwm - rrd;
		next_pwm = max(get_next_lower_pwm(cur_pwm, fan_data),
							fan_data->fan_cur_pwm);
		next_pwm = max(next_pwm, fan_data->next_target_pwm);
		next_pwm = max(0, next_pwm);
	}

	if ((next_pwm != 0) && !(fan_data->is_fan_reg_enabled)) {
		err = regulator_enable(fan_data->fan_reg);
		if (err < 0)
			dev_err(fan_data->dev,
				" Coudn't enable vdd-fan\n");
		else {
			dev_dbg(fan_data->dev,
				" Enabled vdd-fan\n");
			fan_data->is_fan_reg_enabled = true;
		}
	}
	if ((next_pwm == 0) && (fan_data->is_fan_reg_enabled)) {
		err = regulator_disable(fan_data->fan_reg);
		if (err < 0)
			dev_err(fan_data->dev,
				" Couldn't disable vdd-fan\n");
		else {
			dev_dbg(fan_data->dev,
				" Disabled vdd-fan\n");
			fan_data->is_fan_reg_enabled = false;
		}
	}

	set_pwm_duty_cycle(next_pwm, fan_data);
	fan_data->fan_cur_pwm = next_pwm;
	if (fan_data->next_target_pwm != next_pwm)
		queue_delayed_work(fan_data->workqueue,
				&(fan_data->fan_ramp_work),
				msecs_to_jiffies(fan_data->step_time));
	mutex_unlock(&fan_data->fan_state_lock);
}

static void fan_hyst_work_func(struct work_struct *work)
{
	int target_pwm;
	struct delayed_work *dwork = container_of(work, struct delayed_work,
									work);
	struct fan_dev_data *fan_data = container_of(dwork, struct
						fan_dev_data, fan_hyst_work);

	mutex_lock(&fan_data->fan_state_lock);
	if (fan_data->fan_kickstart) {
		fan_data->fan_kickstart = false;
		target_pwm = fan_data->fan_pwm[fan_data->next_state];
		target_pwm = min(fan_data->fan_cap_pwm, target_pwm);
		fan_update_target_pwm(fan_data, target_pwm);
	}
	mutex_unlock(&fan_data->fan_state_lock);
}

static long long time_diff_us(u64 start_ns, u64 end_ns)
{
	return (end_ns - start_ns) / 1000;
}

static void fan_tach_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan_dev_data *fan_data = container_of(dwork, struct fan_dev_data,
			fan_tach_work);
	int tach_pulse_count, avg;
	long long diff_us = 0;
	unsigned long flags;

	spin_lock_irqsave(&fan_data->irq_lock, flags);
	tach_pulse_count = fan_data->irq_count;
	fan_data->irq_count = 0;
	diff_us = time_diff_us(fan_data->first_irq, fan_data->last_irq);
	spin_unlock_irqrestore(&fan_data->irq_lock, flags);

	/* get time diff */
	if (tach_pulse_count <= 1) {
		atomic64_set(&fan_data->rpm_measured, 0);
	} else if (diff_us < 0) {
		dev_err(fan_data->dev,
			"invalid irq time diff: caught %d, diff_us %lld\n",
			tach_pulse_count, diff_us);
	} else {
		avg = diff_us / (tach_pulse_count - 1);

		/* 2 tach falling irq per round */
		atomic64_set(&fan_data->rpm_measured, USEC_PER_MIN / (avg * 2));
	}

	queue_delayed_work(fan_data->tach_workqueue,
			&(fan_data->fan_tach_work),
			msecs_to_jiffies(fan_data->tach_period));
}

static ssize_t fan_profile_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;

	if (!fan_data)
		return -EINVAL;
	if (fan_data->num_profiles > 0) {
		ret = sprintf(buf, "%s\n",
				fan_data->fan_profile_names[fan_data->current_profile]);
	} else {
		ret = sprintf(buf, "N/A\n");
	}
	return ret;
}

static ssize_t fan_profile_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	size_t buf_len;
	int profile_index = -1;
	int i;

	if (!fan_data)
		return -EINVAL;

	buf_len = min(count, (size_t) MAX_PROFILE_NAME_LENGTH);
	while (buf_len > 0 &&
			(buf[buf_len - 1] == '\n' || buf[buf_len - 1] == ' '))
		buf_len--;

	if (buf_len == 0)
		return -EINVAL;

	for (i = 0; i < fan_data->num_profiles; ++i) {
		if (!strncmp(fan_data->fan_profile_names[i], buf,
				max(buf_len, strlen(fan_data->fan_profile_names[i])))) {
			profile_index = i;
		}
	}

	if (profile_index < 0)
		return -EINVAL;

	mutex_lock(&fan_data->fan_state_lock);

	memcpy(fan_data->fan_pwm, fan_data->fan_profile_pwms[profile_index],
			sizeof(int) * (fan_data->active_steps));
	fan_data->fan_state_cap = fan_data->fan_profile_caps[profile_index];
	fan_data->fan_cap_pwm = fan_data->fan_pwm[fan_data->fan_state_cap];
	fan_data->current_profile = profile_index;

	mutex_unlock(&fan_data->fan_state_lock);

	return count;

}

/*State cap sysfs fops*/
static ssize_t fan_state_cap_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	ret = sprintf(buf, "%d\n", fan_data->fan_state_cap);
	mutex_unlock(&fan_data->fan_state_lock);
	return ret;
}

static ssize_t fan_state_cap_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int val, ret, target_pwm;

	ret = sscanf(buf, "%d", &val);

	if ((ret <= 0) || (!fan_data)) {
		dev_err(dev, "%s, fan_data is null or wrong input\n",
			__func__);
		return -EINVAL;
	}

	if (val < 0)
		val = 0;

	mutex_lock(&fan_data->fan_state_lock);
	if (val >= fan_data->active_steps)
		val = fan_data->active_steps - 1;
	fan_data->fan_state_cap = val;
	fan_data->fan_cap_pwm =
		fan_data->fan_pwm[fan_data->fan_state_cap_lookup[val]];
	target_pwm = min(fan_data->fan_cap_pwm, fan_data->next_target_pwm);
	fan_update_target_pwm(fan_data, target_pwm);
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static ssize_t fan_pwm_state_map_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, index, pwm_val, target_pwm;

	ret = sscanf(buf, "%d %d", &index, &pwm_val);

	if ((ret <= 0) || (!fan_data) || (index < 0) || (pwm_val < 0))
		return -EINVAL;

	dev_dbg(dev, "index=%d, pwm_val=%d", index, pwm_val);

	mutex_lock(&fan_data->fan_state_lock);
	if ((pwm_val > fan_data->fan_cap_pwm) ||
			(index > fan_data->active_steps)) {
		mutex_unlock(&fan_data->fan_state_lock);
		return -EINVAL;
	}
	fan_data->fan_pwm[index] = pwm_val;
	if (index == fan_data->next_state) {
		if (fan_data->next_target_pwm != fan_data->fan_pwm[index]) {
			target_pwm = fan_data->fan_pwm[index];
			target_pwm = min(fan_data->fan_cap_pwm, target_pwm);
			fan_update_target_pwm(fan_data, target_pwm);
		}
	}
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static ssize_t kickstart_params_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int bytes_written = 0;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
	bytes_written += sprintf(buf, "pwm:%d, time:%dms\n",
		fan_data->fan_startup_pwm, fan_data->fan_startup_time);
	mutex_unlock(&fan_data->fan_state_lock);

	return bytes_written;
}

static ssize_t kickstart_params_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, fan_startup_pwm, fan_startup_time;

	ret = sscanf(buf, "%d %d", &fan_startup_pwm, &fan_startup_time);

	if ((ret < 2) || (!fan_data) || (fan_startup_pwm < 0)
		|| (fan_startup_time < 0))
		return -EINVAL;

	dev_dbg(dev, "fan_startup_pwm=%d, fan_startup_time=%d\n",
			fan_startup_pwm, fan_startup_time);

	mutex_lock(&fan_data->fan_state_lock);
	fan_data->fan_startup_pwm = min(fan_startup_pwm, fan_data->fan_cap_pwm);
	fan_data->fan_startup_time = fan_startup_time;
	if (fan_data->fan_startup_pwm && fan_data->fan_startup_time)
		fan_data->kickstart_en = true;
	else
		fan_data->kickstart_en = false;
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static ssize_t fan_kickstart_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int bytes_written = 0;

	if (!fan_data)
		return -EINVAL;
	mutex_lock(&fan_data->fan_state_lock);
		bytes_written += sprintf(buf + bytes_written,
					"%d\n", fan_data->fan_kickstart);
	mutex_unlock(&fan_data->fan_state_lock);

	return bytes_written;
}

static void kickstart_fan(struct fan_dev_data *fan_data)
{
	int target_pwm;

	if (!fan_data->fan_kickstart)
		return;

	target_pwm = min(fan_data->fan_cap_pwm, fan_data->fan_startup_pwm);
	fan_update_target_pwm(fan_data, target_pwm);

	queue_delayed_work(fan_data->workqueue, &fan_data->fan_hyst_work,
			msecs_to_jiffies(fan_data->fan_startup_time));
}

static ssize_t fan_kickstart_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fan_dev_data *fan_data = dev_get_drvdata(dev);
	int ret, fan_kickstart = 0;

	ret = sscanf(buf, "%d", &fan_kickstart);

	if ((ret <= 0) || (!fan_data) || (fan_kickstart < 0)
		|| (fan_data->fan_startup_pwm <= 0)
		|| (fan_data->fan_startup_time <= 0))
		return -EINVAL;

	dev_dbg(dev, "fan_kickstart=%d", fan_kickstart);

	mutex_lock(&fan_data->fan_state_lock);
	fan_data->fan_kickstart = !!fan_kickstart;
	if (fan_kickstart)
		kickstart_fan(fan_data);
	mutex_unlock(&fan_data->fan_state_lock);

	return count;
}

static DEVICE_ATTR(pwm_cap, S_IWUSR | S_IRUGO,
			fan_pwm_cap_show,
			fan_pwm_cap_store);
static DEVICE_ATTR(state_cap, S_IWUSR | S_IRUGO,
			fan_state_cap_show,
			fan_state_cap_store);
static DEVICE_ATTR(pwm_state_map, S_IWUSR | S_IRUGO,
			NULL,
			fan_pwm_state_map_store);
static DEVICE_ATTR(cur_pwm, S_IRUGO,
			fan_cur_pwm_show,
			NULL);
static DEVICE_ATTR(target_pwm, S_IWUSR | S_IRUGO,
			fan_target_pwm_show,
			fan_target_pwm_store);
static DEVICE_ATTR(tach_enable, S_IWUSR | S_IRUGO,
			fan_tach_enabled_show,
			fan_tach_enabled_store);
static DEVICE_ATTR(rpm_measured, S_IRUGO,
			fan_rpm_measured_show,
			NULL);
static DEVICE_ATTR(temp_control, S_IWUSR | S_IRUGO,
			fan_temp_control_show,
			fan_temp_control_store);
static DEVICE_ATTR(step_time, S_IWUSR | S_IRUGO,
			fan_step_time_show,
			fan_step_time_store);
static DEVICE_ATTR(pwm_rpm_table, S_IRUGO,
			fan_pwm_rpm_table_show,
			NULL);
static DEVICE_ATTR(fan_profile, S_IWUSR | S_IRUGO,
			fan_profile_show,
			fan_profile_store);
static DEVICE_ATTR(kickstart_params, S_IWUSR | S_IRUGO,
			kickstart_params_show,
			kickstart_params_store);
static DEVICE_ATTR(fan_kickstart, S_IWUSR | S_IRUGO,
			fan_kickstart_show,
			fan_kickstart_store);

static struct attribute *pwm_fan_attributes[] = {
	&dev_attr_fan_profile.attr,
	&dev_attr_pwm_cap.attr,
	&dev_attr_state_cap.attr,
	&dev_attr_pwm_state_map.attr,
	&dev_attr_cur_pwm.attr,
	&dev_attr_target_pwm.attr,
	&dev_attr_tach_enable.attr,
	&dev_attr_rpm_measured.attr,
	&dev_attr_temp_control.attr,
	&dev_attr_step_time.attr,
	&dev_attr_pwm_rpm_table.attr,
	&dev_attr_kickstart_params.attr,
	&dev_attr_fan_kickstart.attr,
	NULL
};

static const struct attribute_group pwm_fan_group = {
	.attrs = pwm_fan_attributes,
};

static int add_sysfs_entry(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &pwm_fan_group);
}

static void remove_sysfs_entry(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &pwm_fan_group);
}

irqreturn_t fan_tach_isr(int irq, void *data)
{
	struct fan_dev_data *fan_data = data;
	u64 curr_time;
	long long diff_us = 0;
	unsigned long flags;

	if (!fan_data)
		return IRQ_HANDLED;

	spin_lock_irqsave(&fan_data->irq_lock, flags);

	curr_time = sched_clock();
	if (fan_data->irq_count == 0)
		fan_data->first_irq = curr_time;
	else
		fan_data->last_irq = curr_time;

	if (fan_data->old_irq != 0) {
		diff_us = time_diff_us(fan_data->old_irq, curr_time);

		/* WAR: To skip double irq under 20usec and irq when rising */
		if (diff_us > 20 && !gpio_get_value(fan_data->tach_gpio))
			fan_data->irq_count++;
	} else
		fan_data->irq_count++;

	fan_data->old_irq = curr_time;

	spin_unlock_irqrestore(&fan_data->irq_lock, flags);

	return IRQ_HANDLED;
}

static int pwm_fan_probe(struct platform_device *pdev)
{
	int i;
	struct fan_dev_data *fan_data = NULL;
	int *rpm_data;
	int *rru_data;
	int *rrd_data;
	int *lookup_data;
	int *pwm_data;
	int err = 0;
	int of_err = 0;
	struct device_node *node = NULL;
	struct device_node *data_node = NULL;
	struct device_node *base_profile_node = NULL;
	struct device_node *profile_node = NULL;
	const char *default_profile = NULL;
	u32 value;
	int pwm_fan_gpio;
	int tach_gpio;
	int fan_startup_time = 0;
	int fan_startup_pwm = 0;

	if (!pdev)
		return -EINVAL;

	node = pdev->dev.of_node;
	if (!node) {
		pr_err("FAN: dev of_node NULL\n");
		return -EINVAL;
	}

	data_node = of_parse_phandle(node, "shared_data", 0);
	if (!data_node) {
		pr_err("PWM shared data node NULL, parse phandle failed\n");
		return -EINVAL;
	}

	fan_data = devm_kzalloc(&pdev->dev,
				sizeof(struct fan_dev_data), GFP_KERNEL);
	if (!fan_data)
		return -ENOMEM;

	fan_data->dev = &pdev->dev;

	fan_data->fan_reg = regulator_get(fan_data->dev, "vdd-fan");
	if (IS_ERR_OR_NULL(fan_data->fan_reg)) {
		pr_err("FAN: coudln't get the regulator\n");
		devm_kfree(&pdev->dev, (void *)fan_data);
		return -ENODEV;
	}

	of_err |= of_property_read_string(node, "name", &fan_data->name);
	pr_info("FAN dev name: %s\n", fan_data->name);

	pwm_fan_gpio = of_get_named_gpio(data_node, "pwm_gpio", 0);
	if (pwm_fan_gpio < 0)
		of_err |= pwm_fan_gpio;

	err = gpio_request(pwm_fan_gpio, "pwm-fan");
	if (err < 0) {
		pr_err("FAN:gpio request failed\n");
		err = -EINVAL;
		goto gpio_request_fail;
	} else {
		pr_info("FAN:gpio request success.\n");
	}

	of_err |= of_property_read_u32(data_node, "active_steps", &value);
	fan_data->active_steps = (int)value;

	of_err |= of_property_read_u32(data_node, "pwm_period", &value);
	fan_data->pwm_period = (int)value;

	of_err |= of_property_read_u32(data_node, "pwm_id", &value);
	fan_data->pwm_id = (int)value;

	of_err |= of_property_read_u32(data_node, "step_time", &value);
	fan_data->step_time = (int)value;

	of_err |= of_property_read_u32(data_node, "active_pwm_max", &value);
	fan_data->fan_pwm_max = (int)value;

	of_err |= of_property_read_u32(data_node, "state_cap", &value);
	fan_data->fan_state_cap = (int)value;

	if (of_property_read_u32(data_node, "pwm_polarity", &value))
		fan_data->fan_pwm_polarity = PWM_POLARITY_INVERSED;
	else if (value > PWM_POLARITY_INVERSED) {
		dev_warn(&pdev->dev, "invalid polarity, use inversed by default\n");
		fan_data->fan_pwm_polarity = PWM_POLARITY_INVERSED;
	} else
		fan_data->fan_pwm_polarity = value;

	/*
	 * suspend_state value is now expected from DT property but not all
	 * platforms provide this propery via DT and expect suspend_state
	 * to be 1. Hence keeping suspend_state to be 1 by default if
	 * property is not provided by DT and function returns -EINVAL
	 */
	err = of_property_read_u32(data_node, "suspend_state", &value);
	if (err == -EINVAL) {
		value = 1;
		err = 0;
	}
	fan_data->suspend_state = !!value;

	fan_data->pwm_gpio = pwm_fan_gpio;

	if (of_err) {
		err = -ENXIO;
		goto rpm_alloc_fail;
	}

	tach_gpio = of_get_named_gpio(data_node, "tach_gpio", 0);
	if (tach_gpio < 0) {
		fan_data->tach_gpio = -1;
		pr_info("FAN: can't find tach_gpio\n");
	} else {
		fan_data->tach_gpio = tach_gpio;
		value = tach_gpio;
	}

	/* fan pwm profiles */
	fan_data->num_profiles = 0;
	base_profile_node = of_get_child_by_name(node, "profiles");
	if (base_profile_node) {
		fan_data->num_profiles = of_get_available_child_count(base_profile_node);
	}
	if (fan_data->num_profiles > 0) {
		of_err |= of_property_read_string(base_profile_node,
				"default", &default_profile);
		dev_info(&pdev->dev, "Found %d profiles, default profile is %s\n",
				fan_data->num_profiles, default_profile);
		fan_data->fan_profile_names = devm_kzalloc(&pdev->dev,
				sizeof(const char*) * fan_data->num_profiles, GFP_KERNEL);
		if (!fan_data->fan_profile_names) {
			err = -ENOMEM;
			goto profile_alloc_fail;
		}
		fan_data->fan_profile_pwms = devm_kzalloc(&pdev->dev,
				sizeof(int*) * fan_data->num_profiles, GFP_KERNEL);
		if (!fan_data->fan_profile_pwms) {
			err = -ENOMEM;
			goto profile_alloc_fail;
		}
		fan_data->fan_profile_caps = devm_kzalloc(&pdev->dev,
				sizeof(int) * fan_data->num_profiles, GFP_KERNEL);
		if (!fan_data->fan_profile_caps) {
			err = -ENOMEM;
			goto profile_alloc_fail;
		}
		fan_data->current_profile = 0;
		i = 0;
		for_each_available_child_of_node (base_profile_node, profile_node) {
			of_err |= of_property_read_string(profile_node, "name",
					&fan_data->fan_profile_names[i]);
			if (default_profile &&
					!strncmp(default_profile,
					fan_data->fan_profile_names[i], MAX_PROFILE_NAME_LENGTH)) {
				fan_data->current_profile = i;
			}
			value = 0;
			of_err |= of_property_read_u32(profile_node, "state_cap", &value);
			fan_data->fan_profile_caps[i] = (int) value;
			pwm_data = devm_kzalloc(&pdev->dev,
					sizeof(int) * (fan_data->active_steps), GFP_KERNEL);
			if (!pwm_data) {
				err = -ENOMEM;
				goto profile_alloc_fail;
			}
			of_err |= of_property_read_u32_array(profile_node,
					"active_pwm", pwm_data,
					(size_t) fan_data->active_steps);
			fan_data->fan_profile_pwms[i] = pwm_data;
			i++;
		}
	}

	/* rpm array */
	rpm_data = devm_kzalloc(&pdev->dev,
			sizeof(int) * (fan_data->active_steps), GFP_KERNEL);
	if (!rpm_data) {
		err = -ENOMEM;
		goto rpm_alloc_fail;
	}
	of_err |= of_property_read_u32_array(data_node, "active_rpm", rpm_data,
		(size_t) fan_data->active_steps);
	fan_data->fan_rpm = rpm_data;

	/* rru array */
	rru_data = devm_kzalloc(&pdev->dev,
			sizeof(int) * (fan_data->active_steps), GFP_KERNEL);
	if (!rru_data) {
		err = -ENOMEM;
		goto rru_alloc_fail;
	}
	of_err |= of_property_read_u32_array(data_node, "active_rru", rru_data,
		(size_t) fan_data->active_steps);
	fan_data->fan_rru = rru_data;

	/* rrd array */
	rrd_data = devm_kzalloc(&pdev->dev,
			sizeof(int) * (fan_data->active_steps), GFP_KERNEL);
	if (!rrd_data) {
		err = -ENOMEM;
		goto rrd_alloc_fail;
	}
	of_err |= of_property_read_u32_array(data_node, "active_rrd", rrd_data,
		(size_t) fan_data->active_steps);
	fan_data->fan_rrd = rrd_data;

	/* state_cap_lookup array */
	lookup_data = devm_kzalloc(&pdev->dev,
			sizeof(int) * (fan_data->active_steps), GFP_KERNEL);
	if (!lookup_data) {
		err = -ENOMEM;
		goto lookup_alloc_fail;
	}
	of_err |= of_property_read_u32_array(data_node, "state_cap_lookup",
		lookup_data, (size_t) fan_data->active_steps);
	fan_data->fan_state_cap_lookup = lookup_data;

	/* pwm array */
	/* If profiles are in use, use current profile pwm */
	pwm_data = devm_kzalloc(&pdev->dev,
			sizeof(int) * (fan_data->active_steps), GFP_KERNEL);
	if (!pwm_data) {
		err = -ENOMEM;
		goto pwm_alloc_fail;
	}
	if (fan_data->num_profiles > 0) {
		memcpy(pwm_data,
				fan_data->fan_profile_pwms[fan_data->current_profile],
				sizeof(int) * (fan_data->active_steps));
	} else {
		of_err |= of_property_read_u32_array(node, "active_pwm", pwm_data,
			(size_t) fan_data->active_steps);
	}
	fan_data->fan_pwm = pwm_data;

	if (of_err) {
		err = -ENXIO;
		goto workqueue_alloc_fail;
	}

	mutex_init(&fan_data->fan_state_lock);
	fan_data->workqueue = alloc_workqueue(dev_name(&pdev->dev),
				WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!fan_data->workqueue) {
		err = -ENOMEM;
		goto workqueue_alloc_fail;
	}

	/* Use profile cap if profiles are in use*/
	if (fan_data->num_profiles > 0) {
		fan_data->fan_state_cap =
				fan_data->fan_profile_caps[fan_data->current_profile];
	}
	fan_data->fan_cap_pwm = fan_data->fan_pwm[fan_data->fan_state_cap];

	fan_data->precision_multiplier =
			(fan_data->pwm_period * MULTIQP) / fan_data->fan_pwm_max;
	dev_info(&pdev->dev, "cap state:%d, cap pwm:%d\n",
			fan_data->fan_state_cap, fan_data->fan_cap_pwm);

	if (of_find_property(node, "fan_startup_pwm", NULL)
		|| of_find_property(node, "fan_startup_pwm", NULL)) {
		of_err = of_property_read_u32(node, "fan_startup_pwm", &fan_startup_pwm);
		fan_data->fan_startup_pwm = min(fan_startup_pwm, fan_data->fan_cap_pwm);

		of_err |=  of_property_read_u32(node, "fan_startup_time", &fan_startup_time);
		fan_data->fan_startup_time = (int)fan_startup_time;

		if (of_err || !fan_data->fan_startup_pwm || !fan_data->fan_startup_time) {
			fan_data->kickstart_en = false;
			dev_err(&pdev->dev, "fan_startup init fail, fail to enable kick start feature\n");
			err = -ENOMEM;
			goto workqueue_alloc_fail;
		} else {
			fan_data->kickstart_en = true;
		}
	}
	fan_data->fan_kickstart = false;

	fan_data->continuous_gov = of_property_read_bool(node, "continuous_gov_boot_on");

	INIT_DELAYED_WORK(&(fan_data->fan_ramp_work), fan_ramping_work_func);
	INIT_DELAYED_WORK(&(fan_data->fan_hyst_work), fan_hyst_work_func);

	fan_data->cdev =
		thermal_cooling_device_register("pwm-fan",
					fan_data, &pwm_fan_cooling_ops);

	if (IS_ERR_OR_NULL(fan_data->cdev)) {
		dev_err(&pdev->dev, "Failed to register cooling device\n");
		err = -EINVAL;
		goto cdev_register_fail;
	}

	fan_data->pwm_dev = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(fan_data->pwm_dev) &&
		PTR_ERR(fan_data->pwm_dev) != -EPROBE_DEFER) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");
		fan_data->pwm_legacy_api = true;
		fan_data->pwm_dev = pwm_request(fan_data->pwm_id,
					dev_name(&pdev->dev));
	}

	if (IS_ERR(fan_data->pwm_dev)) {
		err = PTR_ERR(fan_data->pwm_dev);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "unable to request PWM for fan\n");
		goto pwm_req_fail;
	} else {
		dev_info(&pdev->dev, "got pwm for fan. polarity is %s\n",
			fan_data->fan_pwm_polarity ? "inversed" : "normal");
	}

	spin_lock_init(&fan_data->irq_lock);
	atomic_set(&fan_data->tach_enabled, 0);
	if (fan_data->tach_gpio != -1) {
		/* init fan tach */
		fan_data->tach_irq = gpio_to_irq(fan_data->tach_gpio);
		err = gpio_request(fan_data->tach_gpio, "pwm-fan-tach");
		if (err < 0) {
			dev_err(&pdev->dev, "fan tach gpio request failed\n");
			goto tach_gpio_request_fail;
		}

		err = gpio_direction_input(fan_data->tach_gpio);
		if (err < 0) {
			dev_err(&pdev->dev,
				"fan tach set gpio direction input failed\n");
			goto tach_request_irq_fail;
		}

		/* TODO: Find a workaround for using the deprecated
		 * gpio_sysfs_set_active_low API which has been deprecated on
		 * the 4.4 kernel version. The calls to change the poliarity of
		 * the tach_gpio has been temporarily disabled as it is not
		 * needed for regular operation of the fan cooling device.
		 */

		err = devm_request_threaded_irq(&pdev->dev,
			fan_data->tach_irq, NULL,
			fan_tach_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"pwm-fan-tach", fan_data);
		if (err < 0) {
			dev_err(&pdev->dev, "fan request irq failed\n");
			goto tach_request_irq_fail;
		}
		dev_info(&pdev->dev, "fan tach request irq success\n");
		disable_irq_nosync(fan_data->tach_irq);
	}

	of_err = of_property_read_u32(data_node, "tach_period", &value);
	if (of_err < 0)
		dev_err(&pdev->dev, "parsing tach_period error: %d\n", of_err);
	else {
		fan_data->tach_period = (int) value;
		dev_info(&pdev->dev, "tach period: %d\n",
			fan_data->tach_period);

		/* init tach work related */
		fan_data->tach_workqueue = alloc_workqueue(fan_data->name,
						WQ_UNBOUND, 1);
		if (!fan_data->tach_workqueue) {
			err = -ENOMEM;
			goto tach_workqueue_alloc_fail;
		}
		INIT_DELAYED_WORK(&(fan_data->fan_tach_work),
				fan_tach_work_func);
	}
	/* init rpm related values */
	fan_data->irq_count = 0;
	atomic64_set(&fan_data->rpm_measured, 0);

	/*turn temp control on*/
	fan_data->fan_temp_control_flag = 1;
	set_pwm_duty_cycle(fan_data->fan_pwm[0], fan_data);
	fan_data->fan_cur_pwm = fan_data->fan_pwm[0];

	gpio_free(fan_data->pwm_gpio);

	platform_set_drvdata(pdev, fan_data);

	if (add_sysfs_entry(&pdev->dev)) {
		dev_err(&pdev->dev, "FAN:Can't create syfs node");
		err = -ENOMEM;
		goto sysfs_fail;
	}

	/* print out initialized info */
	for (i = 0; i < fan_data->active_steps; i++) {
		dev_info(&pdev->dev,
			"index %d: pwm=%d, rpm=%d, rru=%d, rrd=%d, state:%d\n",
			i,
			fan_data->fan_pwm[i],
			fan_data->fan_rpm[i],
			fan_data->fan_rru[i],
			fan_data->fan_rrd[i],
			fan_data->fan_state_cap_lookup[i]);
	}

	return err;

sysfs_fail:
	destroy_workqueue(fan_data->tach_workqueue);
tach_workqueue_alloc_fail:
tach_request_irq_fail:
tach_gpio_request_fail:
	if (fan_data->pwm_legacy_api)
		pwm_free(fan_data->pwm_dev);
pwm_req_fail:
	thermal_cooling_device_unregister(fan_data->cdev);
cdev_register_fail:
	destroy_workqueue(fan_data->workqueue);
workqueue_alloc_fail:
profile_alloc_fail:
pwm_alloc_fail:
lookup_alloc_fail:
rrd_alloc_fail:
rru_alloc_fail:
rpm_alloc_fail:
	gpio_free(pwm_fan_gpio);
gpio_request_fail:
	if (err == -ENXIO)
		pr_err("FAN: of_property_read failed\n");
	else if (err == -ENOMEM)
		pr_err("FAN: memery allocation failed\n");
	return err;
}

static int pwm_fan_remove(struct platform_device *pdev)
{
	struct fan_dev_data *fan_data = platform_get_drvdata(pdev);

	if (!fan_data)
		return -EINVAL;

	thermal_cooling_device_unregister(fan_data->cdev);
	remove_sysfs_entry(&pdev->dev);
	cancel_delayed_work(&fan_data->fan_tach_work);
	destroy_workqueue(fan_data->tach_workqueue);
	disable_irq(fan_data->tach_irq);
	gpio_free(fan_data->tach_gpio);
	if (fan_data->kickstart_en) {
		cancel_delayed_work_sync(&fan_data->fan_hyst_work);
		fan_data->fan_kickstart = false;
	}
	cancel_delayed_work(&fan_data->fan_ramp_work);
	destroy_workqueue(fan_data->workqueue);
	pwm_config(fan_data->pwm_dev, 0, fan_data->pwm_period);
	pwm_disable(fan_data->pwm_dev);
	if (fan_data->pwm_legacy_api)
		pwm_free(fan_data->pwm_dev);

	if (fan_data->fan_reg)
		regulator_put(fan_data->fan_reg);
	return 0;
}

#if CONFIG_PM
static int pwm_fan_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fan_dev_data *fan_data = platform_get_drvdata(pdev);
	int err;

	mutex_lock(&fan_data->fan_state_lock);
	if (fan_data->kickstart_en) {
		cancel_delayed_work(&fan_data->fan_hyst_work);
		fan_data->fan_kickstart = false;
	}
	cancel_delayed_work(&fan_data->fan_ramp_work);

	set_pwm_duty_cycle(0, fan_data);
	pwm_disable(fan_data->pwm_dev);

	err = gpio_request(fan_data->pwm_gpio, "pwm-fan");
	if (err < 0) {
		dev_err(&pdev->dev, "%s:gpio request failed %d\n",
			__func__, fan_data->pwm_gpio);
	}

	gpio_direction_output(fan_data->pwm_gpio, fan_data->suspend_state);

	if (fan_data->is_fan_reg_enabled) {
		err = regulator_disable(fan_data->fan_reg);
		if (err < 0)
			dev_err(&pdev->dev, "Not able to disable Fan regulator\n");
		else {
			dev_dbg(fan_data->dev,
				" Disabled vdd-fan\n");
			fan_data->is_fan_reg_enabled = false;
		}
	}
	/*Stop thermal control*/
	fan_data->fan_temp_control_flag = 0;
	mutex_unlock(&fan_data->fan_state_lock);
	return 0;
}

static int pwm_fan_resume(struct platform_device *pdev)
{
	struct fan_dev_data *fan_data = platform_get_drvdata(pdev);

	mutex_lock(&fan_data->fan_state_lock);

	gpio_free(fan_data->pwm_gpio);

	queue_delayed_work(fan_data->workqueue,
			&fan_data->fan_ramp_work,
			msecs_to_jiffies(fan_data->step_time));
	fan_data->fan_temp_control_flag = 1;
	mutex_unlock(&fan_data->fan_state_lock);
	return 0;
}
#endif


static const struct of_device_id of_pwm_fan_match[] = {
	{ .compatible = "loki-pwm-fan", },
	{ .compatible = "ers-pwm-fan", },
	{ .compatible = "foster-pwm-fan", },
	{ .compatible = "pwm-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_fan_match);

static struct platform_driver pwm_fan_driver = {
	.driver = {
		.name	= "pwm_fan_driver",
		.owner = THIS_MODULE,
		.of_match_table = of_pwm_fan_match,
	},
	.probe = pwm_fan_probe,
	.remove = pwm_fan_remove,
#if CONFIG_PM
	.suspend = pwm_fan_suspend,
	.resume = pwm_fan_resume,
#endif
};

module_platform_driver(pwm_fan_driver);

MODULE_DESCRIPTION("pwm fan driver");
MODULE_AUTHOR("Anshul Jain <anshulj@nvidia.com>");
MODULE_LICENSE("GPL v2");
