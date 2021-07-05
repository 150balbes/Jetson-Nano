/*
 * drivers/misc/therm_fan_est.c
 *
 * Copyright (c) 2013-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/therm_est.h>
#include <linux/thermal.h>
#include <linux/module.h>
#include <linux/hwmon-sysfs.h>
#include <linux/version.h>


/* Based off of max device tree node name length */
#define MAX_PROFILE_NAME_LENGTH	31

static void fan_set_trip_temp_hyst(struct therm_fan_estimator *est, int trip,
							unsigned long hyst_temp,
							unsigned long trip_temp)
{
	est->active_hysteresis[trip] = hyst_temp;
	est->active_trip_temps[trip] = trip_temp;
	if (trip == 1) {
		est->nonsleep_hyst = est->active_hysteresis[1];
	}
}

static void therm_fan_est_work_func(struct work_struct *work)
{
	int i, j, group, index, trip_index = 0;
	int sum[MAX_SUBDEVICE_GROUP] = {0, };
	int sum_max = INT_MIN;
	int sum_min = INT_MAX;
	int temp = 0;
	struct delayed_work *dwork = container_of(work,
					struct delayed_work, work);
	struct therm_fan_estimator *est = container_of(
					dwork,
					struct therm_fan_estimator,
					therm_fan_est_work);
	bool update_flag = false;

	for (i = 0; i < est->ndevs; i++) {
		if (est->devs[i].get_temp(est->devs[i].dev_data, &temp))
			continue;
		est->devs[i].hist[(est->ntemp % HIST_LEN)] = temp;
	}

	for (i = 0; i < est->ndevs; i++) {
		group = est->devs[i].group;
		for (j = 0; j < HIST_LEN; j++) {
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			sum[group] += est->devs[i].hist[index] *
				est->devs[i].coeffs[j];
		}
	}

	if (est->use_tmargin) {
		for (i = 0; i < MAX_SUBDEVICE_GROUP; i++)
			sum[i] = (est->crit_temp[i] * 100) - sum[i];
	}

#if !DEBUG
	for (i = 0; i < MAX_SUBDEVICE_GROUP; i++) {
		sum_min = min(sum_min, sum[i]);
		sum_max = max(sum_max, sum[i]);
	}

	if (est->use_tmargin)
		est->cur_temp = sum_min / 100 + est->toffset;
	else
		est->cur_temp = sum_max / 100 + est->toffset;
#else
	est->cur_temp = est->cur_temp_debug;
#endif

	if (est->is_continuous_gov)
		goto next_work;

	if (est->reset_trip_index) {
		est->reset_trip_index = 0;
		est->current_trip_level = 0;
	}

	if (est->cur_temp != est->pre_temp) {
		if (est->use_tmargin ?
			(est->cur_temp < est->pre_temp) :
			(est->cur_temp > est->pre_temp)) {
			/* temperature is rising */
			read_lock(&est->state_lock);
			for (trip_index = 0;
				trip_index < (MAX_ACTIVE_STATES + 1); trip_index++) {
				if (est->cur_temp < est->active_trip_temps[trip_index])
					break;
			}
			read_unlock(&est->state_lock);

			if (est->use_tmargin) {
				if (est->current_trip_level >= trip_index
					&& est->current_trip_level != (trip_index - 1))
					update_flag = true;
			} else {
				if (est->current_trip_level < trip_index
					&& est->current_trip_level != (trip_index - 1))
					update_flag = true;
			}
		} else {
			/* temperature is cooling */
			read_lock(&est->state_lock);
			for (trip_index = 1;
				trip_index < MAX_ACTIVE_STATES; trip_index++) {
				if (est->cur_temp < (est->active_trip_temps[trip_index]
					- est->active_hysteresis[trip_index]))
					break;
			}
			read_unlock(&est->state_lock);

			if (est->use_tmargin) {
				if (est->current_trip_level < trip_index
					&& est->current_trip_level != (trip_index - 1))
					update_flag = true;
			} else {
				if (est->current_trip_level >= trip_index
					&& est->current_trip_level != (trip_index - 1)
					&& trip_index != (MAX_ACTIVE_STATES + 1))
					update_flag = true;
			}
		}

		if (update_flag) {
			est->current_trip_level = trip_index - 1;
			pr_info("FAN %s trip_level:%d cur_temp:%ld trip_temps[%d]:%d\n",
				(est->cur_temp < est->pre_temp)?"cooling":"rising",
				est->current_trip_level, est->cur_temp,
				trip_index, est->active_trip_temps[trip_index]);

		#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
					thermal_zone_device_update(est->thz,
						THERMAL_EVENT_UNSPECIFIED);
		#else
					thermal_zone_device_update(est->thz);
		#endif
		}

		est->pre_temp = est->cur_temp;
	}
	/*
	 * spec for sleep mode is to attempt to turn off fan once only
	 * more details in bug 2527983
	 */
	if (est->sleep_mode && est->current_trip_level == 0) {
		pr_info("%s, cooling device in sleep.\n", __func__);
		write_lock(&est->state_lock);
		est->sleep_mode = false;
		est->active_hysteresis[1] = est->nonsleep_hyst;
		write_unlock(&est->state_lock);
	}

next_work:
	est->ntemp++;
	queue_delayed_work(est->workqueue, &est->therm_fan_est_work,
				msecs_to_jiffies(est->polling_period));
}

#ifdef CONFIG_THERMAL_GOV_CONTINUOUS
/*function defined in continu_thermal_gov.c*/
void register_fetch_pwm_func(int (*func)(struct thermal_cooling_device *cdev, int trip));

/*fetch pwm table for continuous_therm_gov and then caculate the slope*/
static int fetch_trip_pwm(struct thermal_cooling_device *cdev, int trip)
{
	struct fan_dev_data *fan_data = cdev->devdata;

	return fan_data->fan_pwm[trip];
}
#endif

static int therm_fan_est_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int i;
	struct therm_fan_estimator *est = thz->devdata;
	if (!strcmp(cdev->type, est->cdev_type)) {
		for (i = 0; i < MAX_ACTIVE_STATES; i++)
			thermal_zone_bind_cooling_device(thz, i, cdev, i, i,
					THERMAL_WEIGHT_DEFAULT);
	}

#ifdef CONFIG_THERMAL_GOV_CONTINUOUS
	if (est->is_continuous_gov)
		register_fetch_pwm_func(fetch_trip_pwm);
#endif

	return 0;
}

static int therm_fan_est_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int i;
	struct therm_fan_estimator *est = thz->devdata;
	if (!strcmp(cdev->type, est->cdev_type)) {
		for (i = 0; i < MAX_ACTIVE_STATES; i++)
			thermal_zone_unbind_cooling_device(thz, i, cdev);
	}

	return 0;
}

static int therm_fan_est_get_trip_type(struct thermal_zone_device *thz,
					int trip,
					enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_ACTIVE;
	return 0;
}

static int therm_fan_est_get_trip_temp(struct thermal_zone_device *thz,
					int trip, int *temp)
{
	struct therm_fan_estimator *est = thz->devdata;
	int ret = 0;

	read_lock(&est->state_lock);
	if (trip == 0) {
		*temp = est->active_trip_temps[0] - est->active_hysteresis[0];
		goto out;
	} else if (trip < 0) {
		*temp = est->active_trip_temps[0] - est->active_hysteresis[0];
		ret = -EINVAL;
		goto out;
	}

	/*
	 * PID governor will support hysteresis,
	 * just return the trip temp without hysteresis, if using PID.
	 */
	if (est->is_pid_gov) {
		*temp = est->active_trip_temps[trip];
	} else {
		if (est->current_trip_level == 0)
			*temp = 0;

		if (trip  <= est->current_trip_level) /* tripped then lower */
			*temp = est->active_trip_temps[trip] - est->active_hysteresis[trip];
		else /* not tripped, then upper */
			*temp = est->active_trip_temps[trip];
	}
out:
	read_unlock(&est->state_lock);
	return ret;
}

static int therm_fan_est_set_trip_temp(struct thermal_zone_device *thz,
					int trip, int temp)
{
	struct therm_fan_estimator *est = thz->devdata;

	write_lock(&est->state_lock);

	/*Need trip 0 to remain as it is*/
	if (((temp - est->active_hysteresis[trip]) < 0) || (trip <= 0)) {
		write_unlock(&est->state_lock);
		return -EINVAL;
	}

	fan_set_trip_temp_hyst(est, trip, est->active_hysteresis[trip], temp);

	write_unlock(&est->state_lock);
	return 0;
}

static int therm_fan_est_get_temp(struct thermal_zone_device *thz, int *temp)
{
	struct therm_fan_estimator *est = thz->devdata;

	*temp = est->cur_temp;
	return 0;
}

static int therm_fan_est_set_trip_hyst(struct thermal_zone_device *thz,
				int trip, int hyst_temp)
{
	struct therm_fan_estimator *est = thz->devdata;

	write_lock(&est->state_lock);

	/*Need trip 0 to remain as it is*/
	if ((est->active_trip_temps[trip] - hyst_temp) < 0 || trip <= 0) {
		write_unlock(&est->state_lock);
		return -EINVAL;
	}

	fan_set_trip_temp_hyst(est, trip,
			hyst_temp, est->active_trip_temps[trip]);

	write_unlock(&est->state_lock);
	return 0;
}

static int therm_fan_est_get_trip_hyst(struct thermal_zone_device *thz,
				int trip, int *temp)
{
	struct therm_fan_estimator *est = thz->devdata;
	read_lock(&est->state_lock);
	*temp = est->active_hysteresis[trip];
	read_unlock(&est->state_lock);
	return 0;
}

static struct thermal_zone_device_ops therm_fan_est_ops = {
	.bind = therm_fan_est_bind,
	.unbind = therm_fan_est_unbind,
	.get_trip_type = therm_fan_est_get_trip_type,
	.get_trip_temp = therm_fan_est_get_trip_temp,
	.get_temp = therm_fan_est_get_temp,
	.set_trip_temp = therm_fan_est_set_trip_temp,
	.get_trip_hyst = therm_fan_est_get_trip_hyst,
	.set_trip_hyst = therm_fan_est_set_trip_hyst,
};

static int fan_est_match(struct thermal_zone_device *thz, void *data)
{
	return (strcmp((char *)data, thz->type) == 0);
}

static int fan_est_get_crit_temp(struct therm_fan_estimator *est)
{
	struct thermal_zone_device *thz;
	int crit_temp;
	int i;

	/* Purge the existing values and calculate again */
	memset(est->crit_temp, 0, sizeof(est->crit_temp));

	for (i = 0; i < est->ndevs; i++) {
		thz = thermal_zone_device_find((void *)est->devs[i].dev_data,
			fan_est_match);

		if (!thz) {
			pr_info("Failed to get thermal zone %s, deferring probe\n",
					est->devs[i].dev_data);
			/* Retry if the thermal zone is not found */
			return -EPROBE_DEFER;
		}

		if (thz->ops && thz->ops->get_crit_temp) {
			thz->ops->get_crit_temp(thz, &crit_temp);
			if (crit_temp <= 0) {
				pr_err("THERMAL EST: Failed to get crit temp of %s\n",
					est->devs[i].dev_data);
				return -EINVAL;
			}
		} else {
			pr_err("THERMAL EST: Invalid ops for %s\n",
				est->devs[i].dev_data);
			return -EINVAL;
		}

		if (crit_temp > est->crit_temp[est->devs[i].group])
			est->crit_temp[est->devs[i].group] = crit_temp;
	}

	for (i = 0; i < MAX_SUBDEVICE_GROUP; i++) {
		if (est->crit_temp[i] == 0) {
			pr_err("THERMAL EST: Group %d has no crit temp\n", i);
			return -EINVAL;
		} else {
			pr_info("THERMAL EST: Group %d crit temp - %ld\n", i,
				est->crit_temp[i]);
		}
	}

	return 0;
}

static ssize_t show_coeff(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	ssize_t len, total_len = 0;
	int i, j;

	for (i = 0; i < est->ndevs; i++) {
		len = snprintf(buf + total_len, PAGE_SIZE, "[%d]", i);
		total_len += len;
		for (j = 0; j < HIST_LEN; j++) {
			len = snprintf(buf + total_len, PAGE_SIZE, " %d",
					est->devs[i].coeffs[j]);
			total_len += len;
		}
		len = snprintf(buf + total_len, PAGE_SIZE, "\n");
		total_len += len;
	}
	return strlen(buf);
}

static ssize_t set_coeff(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	int devid, scount;
	s32 coeff[20];

	if (HIST_LEN > 20)
		return -EINVAL;

	scount = sscanf(buf, "[%d] %d %d %d %d %d %d %d %d %d %d " \
			"%d %d %d %d %d %d %d %d %d %d",
			&devid,	&coeff[0], &coeff[1], &coeff[2], &coeff[3],
			&coeff[4], &coeff[5], &coeff[6], &coeff[7], &coeff[8],
			&coeff[9], &coeff[10], &coeff[11], &coeff[12],
			&coeff[13], &coeff[14],	&coeff[15], &coeff[16],
			&coeff[17], &coeff[18],	&coeff[19]);

	if (scount != HIST_LEN + 1)
		return -1;

	if (devid < 0 || devid >= est->ndevs)
		return -EINVAL;

	/* This has obvious locking issues but don't worry about it */
	memcpy(est->devs[devid].coeffs, coeff, sizeof(coeff[0]) * HIST_LEN);

	return count;
}

static ssize_t show_offset(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);

	snprintf(buf, PAGE_SIZE, "%ld\n", est->toffset);
	return strlen(buf);
}

static ssize_t set_offset(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	int offset;

	if (kstrtoint(buf, 0, &offset))
		return -EINVAL;

	est->toffset = offset;

	return count;
}

static ssize_t show_fan_profile(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	int ret;

	if (!est)
		return -EINVAL;
	if (est->num_profiles > 0) {
		ret = sprintf(buf, "%s\n",
				est->fan_profile_names[est->current_profile]);
	} else {
		ret = sprintf(buf, "N/A\n");
	}
	return ret;
}

static ssize_t set_fan_profile(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	size_t buf_len;
	int profile_index = -1;
	int i;

	buf_len = min(count, (size_t) MAX_PROFILE_NAME_LENGTH);
	while (buf_len > 0 &&
			(buf[buf_len - 1] == '\n' || buf[buf_len - 1] == ' '))
		buf_len--;

	if (buf_len == 0)
		return -EINVAL;

	for (i = 0; i < est->num_profiles; ++i) {
		if (!strncmp(est->fan_profile_names[i], buf,
				max(buf_len, strlen(est->fan_profile_names[i])))) {
			profile_index = i;
		}
	}

	if (profile_index < 0)
		return -EINVAL;

	write_lock(&est->state_lock);

	memcpy(est->active_trip_temps, est->fan_profile_trip_temps[profile_index],
			sizeof(s32) * MAX_ACTIVE_STATES);
	memcpy(est->active_hysteresis, est->fan_profile_trip_hysteresis[profile_index],
			sizeof(s32) * MAX_ACTIVE_STATES);

	for (i = 1; i < MAX_ACTIVE_STATES; i++)
		fan_set_trip_temp_hyst(est, i,
			est->active_hysteresis[i],
			est->active_trip_temps[i]);
	est->current_profile = profile_index;

	/* Reset trip because profile has changed trip table */
	est->reset_trip_index = 1;

	write_unlock(&est->state_lock);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
	thermal_zone_device_update(est->thz,
			THERMAL_EVENT_UNSPECIFIED);
#else
	thermal_zone_device_update(est->thz);
#endif

	return count;
}

static ssize_t show_sleep_mode(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	int ret;

	if (!est)
		return -EINVAL;
	ret = sprintf(buf, "%s.\n", est->sleep_mode ? "True" : "False");
	return ret;
}

static ssize_t set_sleep_mode(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	int flag;

	if (kstrtoint(buf, 0, &flag))
		return -EINVAL;

	if (flag != 0 && flag != 1)
		return -EINVAL;

	write_lock(&est->state_lock);
	if (flag) {
		est->sleep_mode = true;
		est->active_hysteresis[1] = 0;
	} else {
		est->sleep_mode = false;
		est->active_hysteresis[1] = est->nonsleep_hyst;
	}
	write_unlock(&est->state_lock);

	return count;
}

static ssize_t show_crit_temps(struct device *dev,
	struct device_attribute *da,
	char *buf)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	ssize_t len, total_len = 0;
	int i;

	for (i = 0; i < MAX_SUBDEVICE_GROUP; i++) {
		len = snprintf(buf + total_len, PAGE_SIZE,
			"group[%d] - %ld", i, est->crit_temp[i]);
		total_len += len;
		len = snprintf(buf + total_len, PAGE_SIZE, "\n");
		total_len += len;
	}
	return strlen(buf);
}

static ssize_t show_zone_map(struct device *dev,
	struct device_attribute *da,
	char *buf)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	ssize_t len, total_len = 0;
	int i;

	for (i = 0; i < est->ndevs; i++) {
	len = snprintf(buf + total_len, PAGE_SIZE,
		"%s - ndev[%d], group[%d]",
		est->devs[i].dev_data, i, est->devs[i].group);
	total_len += len;
	len = snprintf(buf + total_len, PAGE_SIZE, "\n");
	total_len += len;
	}
	return strlen(buf);
}


static ssize_t show_temps(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	ssize_t total_len = 0;
	int i, j;
	int index;

	/* This has obvious locking issues but don't worry about it */
	for (i = 0; i < est->ndevs; i++) {
		total_len += snprintf(buf + total_len, PAGE_SIZE, "[%d]", i);
		for (j = 0; j < HIST_LEN; j++) {
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			total_len += snprintf(buf + total_len,
						PAGE_SIZE,
						" %d",
						est->devs[i].hist[index]);
		}
		total_len += snprintf(buf + total_len, PAGE_SIZE, "\n");
	}
	return strlen(buf);
}

#if DEBUG
static ssize_t set_temps(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_fan_estimator *est = dev_get_drvdata(dev);
	int temp;

	if (kstrtoint(buf, 0, &temp))
		return -EINVAL;

	est->cur_temp_debug = temp;

	return count;
}
#endif

static ssize_t set_update_subdevs_crit_temps(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_fan_estimator *est_data = dev_get_drvdata(dev);
	int err;

	if (est_data->use_tmargin) {
		err = fan_est_get_crit_temp(est_data);
		if (err)
			return -EINVAL;
	}

	return count;
}

static struct sensor_device_attribute therm_fan_est_nodes[] = {
	SENSOR_ATTR(coeff, S_IRUGO | S_IWUSR, show_coeff, set_coeff, 0),
	SENSOR_ATTR(offset, S_IRUGO | S_IWUSR, show_offset, set_offset, 0),
	SENSOR_ATTR(fan_profile, S_IRUGO | S_IWUSR,
				show_fan_profile, set_fan_profile, 0),
	SENSOR_ATTR(sleep_mode, S_IRUGO | S_IWUSR,
				show_sleep_mode, set_sleep_mode, 0),
	SENSOR_ATTR(zone_map, S_IRUGO, show_zone_map, 0, 0),
	SENSOR_ATTR(crit_temps, S_IRUGO, show_crit_temps, 0, 0),
	SENSOR_ATTR(update_subdevs_crit_temps, S_IWUSR, 0,
			set_update_subdevs_crit_temps, 0),
#if DEBUG
	SENSOR_ATTR(temps, S_IRUGO | S_IWUSR, show_temps, set_temps, 0),
#else
	SENSOR_ATTR(temps, S_IRUGO, show_temps, 0, 0),
#endif
};


static int fan_est_get_temp_func(const char *data, int *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find((void *)data, fan_est_match);

	if (!thz || thz->ops->get_temp == NULL || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static int therm_fan_est_probe(struct platform_device *pdev)
{
	int i, j;
	int temp;
	int err = 0;
	int of_err = 0;
	struct therm_fan_estimator *est_data;
	struct therm_fan_est_subdevice *subdevs;
	struct therm_fan_est_subdevice *dev;
	struct thermal_zone_params *tzp;
	struct device_node *node = NULL;
	struct device_node *data_node = NULL;
	struct device_node *base_profile_node = NULL;
	struct device_node *profile_node = NULL;
#if IS_ENABLED(CONFIG_THERMAL_GOV_CONTINUOUS)
	struct device_node *gov_node = NULL;
#endif
	const char *default_profile = NULL;
	int child_count = 0;
	struct device_node *child = NULL;
	const char *gov_name;
	u32 value;

	pr_debug("THERMAL EST start of therm_fan_est_probe.\n");
	if (!pdev)
		return -EINVAL;

	node = pdev->dev.of_node;
	if (!node) {
		pr_err("THERMAL EST: dev of_node NULL\n");
		return -EINVAL;
	}

	data_node = of_parse_phandle(node, "shared_data", 0);
	if (!data_node) {
		pr_err("THERMAL EST shared data node parsing failed\n");
		return -EINVAL;
	}

	child_count = of_get_child_count(data_node);
	of_err |= of_property_read_u32(data_node, "ndevs", &value);
	if (of_err) {
		pr_err("THERMAL EST: missing ndevs\n");
		return -ENXIO;
	}
	if (child_count != (int)value) {
		pr_err("THERMAL EST: ndevs count mismatch\n");
		return -EINVAL;
	}

	est_data = devm_kzalloc(&pdev->dev,
				sizeof(struct therm_fan_estimator), GFP_KERNEL);
	if (IS_ERR_OR_NULL(est_data))
		return -ENOMEM;

	est_data->ndevs = child_count;
	pr_info("THERMAL EST: found %d subdevs\n", est_data->ndevs);

	of_err |= of_property_read_string(node, "name", &est_data->name);
	if (of_err) {
		pr_err("THERMAL EST: name is missing\n");
		err = -ENXIO;
		goto free_est;
	}
	pr_debug("THERMAL EST name: %s.\n", est_data->name);

	of_err |= of_property_read_u32(node, "num_resources", &value);
	if (of_err) {
		pr_err("THERMAL EST: num_resources is missing\n");
		err = -ENXIO;
		goto free_est;
	}
	est_data->num_resources = value;
	pr_info("THERMAL EST num_resources: %d\n", est_data->num_resources);

	est_data->use_tmargin = of_property_read_bool(node, "use_tmargin");

	of_err |= of_property_read_u32(node, "trip_length", &value);
	if (of_err) {
		pr_err("THERMAL EST: missing trip length\n");
		err = -ENXIO;
		goto free_est;
	}

	est_data->trip_length = (int)value;
	subdevs = devm_kzalloc(&pdev->dev,
			child_count * sizeof(struct therm_fan_est_subdevice),
			GFP_KERNEL);
	if (IS_ERR_OR_NULL(subdevs)) {
		err = -ENOMEM;
		goto free_est;
	}

	/* initialize subdevs */
	j = 0;
	for_each_child_of_node(data_node, child) {
		pr_info("[THERMAL EST subdev %d]\n", j);
		of_err |= of_property_read_string(child, "dev_data",
						&subdevs[j].dev_data);
		if (of_err) {
			pr_err("THERMAL EST subdev[%d] dev_data missed\n", j);
			err = -ENXIO;
			goto free_subdevs;
		}
		pr_debug("THERMAL EST subdev name: %s\n",
				(char *)subdevs[j].dev_data);

		subdevs[j].get_temp = &fan_est_get_temp_func;

		if (of_property_read_u32(child, "group", &value)) {
			pr_debug("Set %s to group 0 as default\n",
				(char *)subdevs[j].dev_data);
			subdevs[j].group = 0;
		} else {
			if (value >= MAX_SUBDEVICE_GROUP) {
				pr_err("THERMAL EST: group limit exceed\n");
				err = -ENXIO;
				goto free_subdevs;
			} else
				subdevs[j].group = (int)value;
		}

		of_err |= of_property_read_u32_array(child, "coeffs",
			subdevs[j].coeffs, HIST_LEN);
		for (i = 0; i < HIST_LEN; i++)
			pr_debug("THERMAL EST index %d coeffs %d\n",
				i, subdevs[j].coeffs[i]);
		j++;
	}
	est_data->devs = subdevs;

	of_err |= of_property_read_u32(data_node, "toffset", &value);
	if (of_err) {
		pr_err("THERMAL EST: missing toffset\n");
		err = -ENXIO;
		goto free_subdevs;
	}
	est_data->toffset = (long)value;

	of_err |= of_property_read_u32(data_node, "polling_period", &value);
	if (of_err) {
		pr_err("THERMAL EST: missing polling_period\n");
		err = -ENXIO;
		goto free_subdevs;
	}
	est_data->polling_period = (long)value;

	/* fan trip temp/hyst profiles */
	est_data->num_profiles = 0;
	base_profile_node = of_get_child_by_name(node, "profiles");
	if (base_profile_node) {
		est_data->num_profiles = of_get_available_child_count(base_profile_node);
	}
	if (est_data->num_profiles > 0) {
		of_err |= of_property_read_string(base_profile_node,
				"default", &default_profile);
		if (of_err) {
			pr_err("THERMAL EST: missing default fan profile\n");
			err = -ENXIO;
			goto free_subdevs;
		}
		pr_info("THERMAL EST: Found %d profiles, default profile is %s\n",
				est_data->num_profiles, default_profile);
		est_data->fan_profile_names = devm_kzalloc(&pdev->dev,
				sizeof(const char*) * est_data->num_profiles, GFP_KERNEL);
		if (!est_data->fan_profile_names) {
			err = -ENOMEM;
			goto free_subdevs;
		}
		est_data->fan_profile_trip_temps = devm_kzalloc(&pdev->dev,
				sizeof(s32*) * est_data->num_profiles, GFP_KERNEL);
		if (!est_data->fan_profile_trip_temps) {
			err = -ENOMEM;
			goto free_subdevs;
		}
		est_data->fan_profile_trip_hysteresis = devm_kzalloc(&pdev->dev,
				sizeof(s32*) * est_data->num_profiles, GFP_KERNEL);
		if (!est_data->fan_profile_trip_hysteresis) {
			err = -ENOMEM;
			goto free_subdevs;
		}
		est_data->current_profile = 0;
		i = 0;
		for_each_available_child_of_node (base_profile_node, profile_node) {
			of_err |= of_property_read_string(profile_node, "name",
					&est_data->fan_profile_names[i]);
			if (default_profile &&
					!strncmp(default_profile,
					est_data->fan_profile_names[i], MAX_PROFILE_NAME_LENGTH)) {
				est_data->current_profile = i;
			}
			est_data->fan_profile_trip_temps[i] = devm_kzalloc(&pdev->dev,
					sizeof(s32) * MAX_ACTIVE_STATES, GFP_KERNEL);
			if (!est_data->fan_profile_trip_temps[i]) {
				err = -ENOMEM;
				goto free_subdevs;
			}
			of_err |= of_property_read_u32_array(profile_node,
					"active_trip_temps", est_data->fan_profile_trip_temps[i],
					(size_t) est_data->trip_length);
			if (of_err) {
				pr_err("THERMAL EST: active trip temps failed to parse.\n");
				err = -ENXIO;
				goto free_subdevs;
			}
			est_data->fan_profile_trip_hysteresis[i] = devm_kzalloc(&pdev->dev,
					sizeof(s32) * MAX_ACTIVE_STATES, GFP_KERNEL);
			if (!est_data->fan_profile_trip_hysteresis[i]) {
				err = -ENOMEM;
				goto free_subdevs;
			}
			of_err |= of_property_read_u32_array(profile_node,
					"active_hysteresis", est_data->fan_profile_trip_hysteresis[i],
					(size_t) est_data->trip_length);
			if (of_err) {
				pr_err("THERMAL EST: active hysteresis failed to parse.\n");
				err = -ENXIO;
				goto free_subdevs;
			}
			i++;
		}
		memcpy(est_data->active_trip_temps,
				est_data->fan_profile_trip_temps[est_data->current_profile],
				sizeof(s32) * MAX_ACTIVE_STATES);
		memcpy(est_data->active_hysteresis,
				est_data->fan_profile_trip_hysteresis[est_data->current_profile],
				sizeof(s32) * MAX_ACTIVE_STATES);
	} else {
		of_err |= of_property_read_u32_array(node, "active_trip_temps",
			est_data->active_trip_temps, (size_t) est_data->trip_length);
		if (of_err) {
			pr_err("THERMAL EST: active trip temps failed to parse.\n");
			err = -ENXIO;
			goto free_subdevs;
		}

		of_err |= of_property_read_u32_array(node, "active_hysteresis",
			est_data->active_hysteresis, (size_t) est_data->trip_length);
		if (of_err) {
			pr_err("THERMAL EST: active hysteresis failed to parse.\n");
			err = -ENXIO;
			goto free_subdevs;
		}
	}

	for (i = 0; i < est_data->trip_length; i++)
		pr_debug("THERMAL EST index %d: trip_temp %d, hyst %d\n",
			i, est_data->active_trip_temps[i],
			est_data->active_hysteresis[i]);

	for (i = 1; i < MAX_ACTIVE_STATES; i++) {
		fan_set_trip_temp_hyst(est_data, i, est_data->active_hysteresis[i],
			est_data->active_trip_temps[i]);
		if (((i + 1) < MAX_ACTIVE_STATES)
			&& ((est_data->active_trip_temps[i] - est_data->active_hysteresis[i])
			>= (est_data->active_trip_temps[i + 1] - est_data->active_hysteresis[i + 1]))) {
			pr_err("THERMAL EST: active hysteresis invalid\n");
			err = -EINVAL;
			goto free_subdevs;
		}
	}
	for (i = 0; i < MAX_ACTIVE_STATES; i++)
		pr_debug("THERMAL EST index %d: trip_temps_hyst %d\n",
			i, est_data->active_trip_temps[i] - est_data->active_hysteresis[i]);

	est_data->nonsleep_hyst  = est_data->active_hysteresis[1];
	est_data->sleep_mode = false;

	for (i = 0; i < est_data->ndevs; i++) {
		dev = &est_data->devs[i];
		if (dev->get_temp(dev->dev_data, &temp)) {
			err = -EINVAL;
			goto free_subdevs;
		}
		for (j = 0; j < HIST_LEN; j++)
			dev->hist[j] = temp;
		pr_debug("THERMAL EST init dev[%d] temp hist to %d\n",
			i, temp);
	}

	if (est_data->use_tmargin) {
		err = fan_est_get_crit_temp(est_data);
		if (err) {
			goto free_subdevs;
		}
	}

	of_err |= of_property_read_string(data_node, "cdev_type",
						&est_data->cdev_type);
	if (of_err) {
		pr_err("THERMAL EST: cdev_type is missing\n");
		err = -EINVAL;
		goto free_subdevs;
	}
	pr_debug("THERMAL EST cdev_type: %s.\n", est_data->cdev_type);

	tzp = devm_kzalloc(&pdev->dev, sizeof(struct thermal_zone_params),
				GFP_KERNEL);
	if (IS_ERR_OR_NULL(tzp)) {
		err = -ENOMEM;
		goto free_subdevs;
	}
	memset(tzp, 0, sizeof(struct thermal_zone_params));
	of_err |= of_property_read_string(data_node, "tzp_governor_name",
						&gov_name);
	if (of_err) {
		pr_err("THERMAL EST: governor name is missing\n");
		err = -EINVAL;
		goto free_tzp;
	}
	strcpy(tzp->governor_name, gov_name);
	pr_debug("THERMAL EST governor name: %s\n", tzp->governor_name);
	if (!strncmp(tzp->governor_name,
			THERMAL_GOV_PID, strlen(THERMAL_GOV_PID)))
		est_data->is_pid_gov = true;
	else
		est_data->is_pid_gov = false;

	if (!strncmp(tzp->governor_name,
			THERMAL_CONTINUOUS_GOV,
			strlen(THERMAL_CONTINUOUS_GOV)))
		est_data->is_continuous_gov = true;
	else
		est_data->is_continuous_gov = false;

	rwlock_init(&est_data->state_lock);

	if (est_data->is_continuous_gov) {
		value = est_data->polling_period;
		if (!value) {
			err = -EINVAL;
			goto free_tzp;
		}
#if IS_ENABLED(CONFIG_THERMAL_GOV_CONTINUOUS)
		gov_node = of_get_child_by_name(node, "thermal-zone-params");
		err = continuous_thermal_gov_update_params(tzp, gov_node);
		if (err || !tzp->governor_params) {
			err = -EINVAL;
			pr_err("failed to update governor data");
			goto free_tzp;
		}
#endif
	}
	else
		value = 0;

	est_data->tzp = tzp;
	est_data->thz = thermal_zone_device_register(
					(char *)dev_name(&pdev->dev),
					10, 0x3FF, est_data,
					&therm_fan_est_ops, tzp, 0, value);
	if (IS_ERR_OR_NULL(est_data->thz)) {
		pr_err("THERMAL EST: thz register failed\n");
		err = -EINVAL;
		goto free_tzp;
	}
	pr_info("THERMAL EST: thz register success.\n");

	/* workqueue related */
	est_data->workqueue = alloc_workqueue(dev_name(&pdev->dev),
					WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!est_data->workqueue) {
		err = -ENOMEM;
		goto free_tzp;
	}

	est_data->current_trip_level = 0;
	est_data->reset_trip_index = 0;
	INIT_DELAYED_WORK(&est_data->therm_fan_est_work,
				therm_fan_est_work_func);
	queue_delayed_work(est_data->workqueue,
			&est_data->therm_fan_est_work,
			msecs_to_jiffies(est_data->polling_period));

	for (i = 0; i < ARRAY_SIZE(therm_fan_est_nodes); i++)
		device_create_file(&pdev->dev,
			&therm_fan_est_nodes[i].dev_attr);

	platform_set_drvdata(pdev, est_data);

	pr_info("THERMAL EST: end of probe, return err: %d\n", err);
	return err;

free_tzp:
	devm_kfree(&pdev->dev, (void *)tzp);
free_subdevs:
	devm_kfree(&pdev->dev, (void *)subdevs);
free_est:
	devm_kfree(&pdev->dev, (void *)est_data);
	return err;
}

static int therm_fan_est_remove(struct platform_device *pdev)
{
	struct therm_fan_estimator *est = platform_get_drvdata(pdev);
	int i;

	if (!est)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(therm_fan_est_nodes); i++)
		device_remove_file(&pdev->dev,
					&therm_fan_est_nodes[i].dev_attr);

	cancel_delayed_work(&est->therm_fan_est_work);
	destroy_workqueue(est->workqueue);
	thermal_zone_device_unregister(est->thz);
	devm_kfree(&pdev->dev, (void *)est->tzp);
	devm_kfree(&pdev->dev, (void *)est->devs);
	devm_kfree(&pdev->dev, (void *)est);
	return 0;
}

#if CONFIG_PM
static int therm_fan_est_suspend(struct platform_device *pdev,
							pm_message_t state)
{
	struct therm_fan_estimator *est = platform_get_drvdata(pdev);

	if (!est)
		return -EINVAL;

	pr_debug("therm-fan-est: %s, cur_temp:%ld", __func__, est->cur_temp);
	cancel_delayed_work(&est->therm_fan_est_work);
	est->current_trip_level = 0;

	return 0;
}

static int therm_fan_est_resume(struct platform_device *pdev)
{
	struct therm_fan_estimator *est = platform_get_drvdata(pdev);

	if (!est)
		return -EINVAL;
	pr_debug("therm-fan-est: %s, cur_temp:%ld", __func__, est->cur_temp);

	queue_delayed_work(est->workqueue,
				&est->therm_fan_est_work,
				msecs_to_jiffies(DEFERRED_RESUME_TIME));
	return 0;
}
#endif

static void therm_fan_est_shutdown(struct platform_device *pdev)
{
	struct therm_fan_estimator *est = platform_get_drvdata(pdev);
	pr_info("therm-fan-est: shutting down\n");
	cancel_delayed_work_sync(&est->therm_fan_est_work);
	destroy_workqueue(est->workqueue);
	thermal_zone_device_unregister(est->thz);
	devm_kfree(&pdev->dev, (void *)est->tzp);
	devm_kfree(&pdev->dev, (void *)est->devs);
	devm_kfree(&pdev->dev, (void *)est);
}

static const struct of_device_id of_thermal_est_match[] = {
	{ .compatible = "loki-thermal-est", },
	{ .compatible = "foster-thermal-est", },
	{ .compatible = "thermal-fan-est", },
	{},
};
MODULE_DEVICE_TABLE(of, of_thermal_est_match);

static struct platform_driver therm_fan_est_driver = {
	.driver = {
		.name  = "therm-fan-est-driver",
		.owner = THIS_MODULE,
		.of_match_table = of_thermal_est_match,
	},
	.probe  = therm_fan_est_probe,
	.remove = therm_fan_est_remove,
#if CONFIG_PM
	.suspend = therm_fan_est_suspend,
	.resume = therm_fan_est_resume,
#endif
	.shutdown = therm_fan_est_shutdown,
};

module_platform_driver(therm_fan_est_driver);

MODULE_DESCRIPTION("fan thermal estimator");
MODULE_AUTHOR("Anshul Jain <anshulj@nvidia.com>");
MODULE_LICENSE("GPL v2");
