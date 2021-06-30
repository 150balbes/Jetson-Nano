/* Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* The NVS = NVidia Sensor framework */
/* This NVS kernel driver has a test mechanism for sending specific data up the
 * SW stack by writing the requested data value to the ch sysfs attribute.
 * That data will be sent anytime there would normally be new data from the
 * device.
 * The feature is disabled whenever the device is disabled.  It remains
 * disabled until the ch sysfs attribute is written to again.
 */
/* The NVS HAL will use the scale and offset sysfs attributes to modify the
 * data using the following formula: (data * scale) + offset
 * A scale value of 0 disables scale.
 * A scale value of 1 puts the NVS HAL into calibration mode where the scale
 * and offset are read everytime the data is read to allow realtime calibration
 * of the scale and offset values to be used in the device tree parameters.
 * Keep in mind the data is buffered but the NVS HAL will display the data and
 * scale/offset parameters in the log.
 */
/* This module automatically handles on-change sensors by testing for allowed
 * report rate and whether data has changed.  This allows sensors that are
 * on-change in name only that normally stream data to behave as on-change.
 */
/* This module automatically handles one-shot sensors by disabling the sensor
 * after an event.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/string.h>
#include "nvs_sysfs.h"

#define NVS_SYSFS_DRIVER_VERSION	(3)


static const char * const nvs_snsr_names[] = {
	[0] = "generic_sensor",
	[SENSOR_TYPE_ACCELEROMETER] = "accelerometer",
	[SENSOR_TYPE_MAGNETIC_FIELD] = "magnetic_field",
	[SENSOR_TYPE_ORIENTATION] = "orientation",
	[SENSOR_TYPE_GYROSCOPE] = "gyroscope",
	[SENSOR_TYPE_LIGHT] = "light",
	[SENSOR_TYPE_PRESSURE] = "pressure",
	[SENSOR_TYPE_TEMPERATURE] = "temperature",
	[SENSOR_TYPE_PROXIMITY] = "proximity",
	[SENSOR_TYPE_GRAVITY] = "gravity",
	[SENSOR_TYPE_LINEAR_ACCELERATION] = "linear_acceleration",
	[SENSOR_TYPE_ROTATION_VECTOR] = "rotation_vector",
	[SENSOR_TYPE_RELATIVE_HUMIDITY] = "relative_humidity",
	[SENSOR_TYPE_AMBIENT_TEMPERATURE] = "ambient_temperature",
	[SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED] =
						 "magnetic_field_uncalibrated",
	[SENSOR_TYPE_GAME_ROTATION_VECTOR] = "game_rotation_vector",
	[SENSOR_TYPE_GYROSCOPE_UNCALIBRATED] = "gyroscope_uncalibrated",
	[SENSOR_TYPE_SIGNIFICANT_MOTION] = "significant_motion",
	[SENSOR_TYPE_STEP_DETECTOR] = "step_detector",
	[SENSOR_TYPE_STEP_COUNTER] = "step_counter",
	[SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR] =
						 "geomagnetic_rotation_vector",
	[SENSOR_TYPE_HEART_RATE] = "heart_rate",
	[SENSOR_TYPE_TILT_DETECTOR] = "tilt_detector",
	[SENSOR_TYPE_WAKE_GESTURE] = "wake_gesture",
	[SENSOR_TYPE_GLANCE_GESTURE] = "glance_gesture",
	[SENSOR_TYPE_PICK_UP_GESTURE] = "pick_up_gesture",
	[SENSOR_TYPE_WRIST_TILT_GESTURE] = "wrist_tilt_gesture",
	[SENSOR_TYPE_DEVICE_ORIENTATION] = "device_orientation",
	[SENSOR_TYPE_POSE_6DOF] = "pose_6dof",
	[SENSOR_TYPE_STATIONARY_DETECT] = "stationary_detect",
	[SENSOR_TYPE_MOTION_DETECT] = "motion_detect",
	[SENSOR_TYPE_HEART_BEAT] = "heart_beat",
	[SENSOR_TYPE_DYNAMIC_SENSOR_META] = "dynamic_sensor_meta",
	[SENSOR_TYPE_ADDITIONAL_INFO] = "additional_info",
};

enum NVS_DBG {
	NVS_INFO_DATA = 0,
	NVS_INFO_VER,
	NVS_INFO_ERRS,
	NVS_INFO_RESET,
	NVS_INFO_REGS,
	NVS_INFO_CFG,
	NVS_INFO_DBG,
	NVS_INFO_DBG_DATA,
	NVS_INFO_DBG_BUF,
	NVS_INFO_DBG_IRQ,
	NVS_INFO_LIMIT_MAX,
	NVS_INFO_DBG_KEY = 0x131071D,
};


static inline struct nvs_state *dev_to_nvs_state(struct device *dev)
{
	return dev_get_drvdata(dev);
}

void nvs_mutex_lock(void *handle)
{
	struct nvs_state *st = (struct nvs_state *)handle;

	if (st)
		mutex_lock(&st->mutex);
}

void nvs_mutex_unlock(void *handle)
{
	struct nvs_state *st = (struct nvs_state *)handle;

	if (st)
		mutex_unlock(&st->mutex);
}

static ssize_t nvs_dbg_cfg(struct nvs_state *st, char *buf)
{
	unsigned int i;
	ssize_t t;

	t = snprintf(buf, PAGE_SIZE, "name=%s\n", st->cfg->name);
	t += snprintf(buf + t, PAGE_SIZE - t, "snsr_id=%d\n",
		      st->cfg->snsr_id);
	t += snprintf(buf + t, PAGE_SIZE - t, "timestamp_sz=%d\n",
		      st->cfg->timestamp_sz);
	t += snprintf(buf + t, PAGE_SIZE - t, "snsr_data_n=%d\n",
		      st->cfg->snsr_data_n);
	t += snprintf(buf + t, PAGE_SIZE - t, "kbuf_sz=%d\n",
		      st->cfg->kbuf_sz);
	t += snprintf(buf + t, PAGE_SIZE - t, "ch_n=%u\n", st->cfg->ch_n);
	t += snprintf(buf + t, PAGE_SIZE - t, "ch_sz=%d\n", st->cfg->ch_sz);
	t += snprintf(buf + t, PAGE_SIZE - t, "ch_inf=%p\n", st->cfg->ch_inf);
	t += snprintf(buf + t, PAGE_SIZE - t, "delay_us_min=%u\n",
		      st->cfg->delay_us_min);
	t += snprintf(buf + t, PAGE_SIZE - t, "delay_us_max=%u\n",
		      st->cfg->delay_us_max);
	t += snprintf(buf + t, PAGE_SIZE - t, "uuid: ");
	for (i = 0; i < 16; i++)
		t += snprintf(buf + t, PAGE_SIZE - t, "%02x ",
			      st->cfg->uuid[i]);
	t += snprintf(buf + t, PAGE_SIZE - t, "\nmatrix: ");
	for (i = 0; i < 9; i++)
		t += snprintf(buf + t, PAGE_SIZE - t, "%hhd ",
			      st->cfg->matrix[i]);
	t += snprintf(buf + t, PAGE_SIZE - t, "\nuncal_lo=%d\n",
		      st->cfg->uncal_lo);
	t += snprintf(buf + t, PAGE_SIZE - t, "uncal_hi=%d\n",
		      st->cfg->uncal_hi);
	t += snprintf(buf + t, PAGE_SIZE - t, "cal_lo=%d\n", st->cfg->cal_lo);
	t += snprintf(buf + t, PAGE_SIZE - t, "cal_hi=%d\n", st->cfg->cal_hi);
	t += snprintf(buf + t, PAGE_SIZE - t, "thresh_lo=%d\n",
		      st->cfg->thresh_lo);
	t += snprintf(buf + t, PAGE_SIZE - t, "thresh_hi=%d\n",
		      st->cfg->thresh_hi);
	t += snprintf(buf + t, PAGE_SIZE - t, "report_n=%d\n",
		      st->cfg->report_n);
	t += snprintf(buf + t, PAGE_SIZE - t, "float_significance=%s\n",
		      nvs_float_significances[st->cfg->float_significance]);
	return t;
}

/* dummy enable function to support client's NULL function pointer */
static int nvs_fn_dev_enable(void *client, int snsr_id, int enable)
{
	return 0;
}

static int nvs_disable(struct nvs_state *st)
{
	int ret;

	ret = st->fn_dev->enable(st->client, st->cfg->snsr_id, 0);
	if (!ret) {
		st->enabled = 0;
		st->dbg_data_lock = 0;
	}
	return ret;
}

static ssize_t nvs_ch2buf(struct nvs_state *st, unsigned int ch,
			  char *buf, size_t buf_n, ssize_t t, bool dbg)
{
	unsigned int i;
	unsigned int n;
	s64 val;

	n = st->buf_ch[ch].byte_n;
	if (n) {
		if (n <= sizeof(val)) {
			val = 0;
			memcpy(&val, &st->buf[st->buf_ch[ch].buf_i], n);
			if (st->buf_ch[ch].sign) {
				i = sizeof(val) - n;
				if (i) {
					i *= 8;
					val <<= i;
					val >>= i;
				}
				if (dbg)
					t += snprintf(buf + t, buf_n - t,
						      "%lld (0x", val);
				else
					return snprintf(buf + t, buf_n - t,
							"%lld\n", val);
			} else {
				if (dbg)
					t += snprintf(buf + t, buf_n - t,
						      "%llu (0x", val);
				else
					return snprintf(buf + t, buf_n - t,
							"%llu\n", (u64)val);
			}
		} else {
			if (dbg)
				t += snprintf(buf + t, buf_n - t, "? (0x");
			else
				t += snprintf(buf + t, buf_n - t, "(0x");
		}

		for (i = n - 1; i > 0; i--)
			t += snprintf(buf + t, buf_n - t, "%02X",
				      st->buf[st->buf_ch[ch].buf_i + i]);

		if (dbg)
			t += snprintf(buf + t, buf_n - t, "%02X)  ",
				      st->buf[st->buf_ch[ch].buf_i]);
		else
			t += snprintf(buf + t, buf_n - t, "%02X\n",
				      st->buf[st->buf_ch[ch].buf_i]);
		return t;
	}

	return 0;
}

static ssize_t nvs_dbg_data(struct nvs_state *st, char *buf, size_t buf_n)
{
	ssize_t t;
	unsigned int ch;
	unsigned int n = st->ch_n - 1;

	t = snprintf(buf, PAGE_SIZE, "%s: ", st->cfg->name);
	for (ch = 0; ch < n; ch++) {
		if (!(st->enabled & (1 << ch))) {
			t += snprintf(buf + t, buf_n - t, "disabled ");
			continue;
		}

		t += nvs_ch2buf(st, ch, buf, buf_n, t, true);
	}
	t += snprintf(buf + t, buf_n - t, "ts=%lld  ts_diff=%lld\n",
		      st->ts, st->ts_diff);
	return t;
}

static int nvs_buf_push(struct nvs_state *st, unsigned char *data, s64 ts)
{
	bool push = true;
	char char_buf[128];
	unsigned int i;
	unsigned int n;
	unsigned int ret_n = 0;
	int ret = 0;

	n = st->ch_n - 1; /* - timestamp channel */
	/* It's possible to have events without data (n == 0).
	 * In this case, just the timestamp is sent.
	 */
	if (n && data) {
		if (st->on_change)
			/* on-change needs data change for push */
			push = false;
		for (i = 0; i < n; i++) {
			if (!(st->enabled & (1 << i)))
				continue;

			if (st->on_change) {
				/* wasted cycles when st->first_push
				 * but saved cycles in the long run.
				 */
				ret = memcmp(&st->buf[st->buf_ch[i].buf_i],
					     &data[st->buf_ch[i].buf_i],
					     st->buf_ch[i].byte_n);
				if (ret)
					/* data changed */
					push = true;
			}
			if (!(st->dbg_data_lock & (1 << i)))
				memcpy(&st->buf[st->buf_ch[i].buf_i],
				       &data[st->buf_ch[i].buf_i],
				       st->buf_ch[i].byte_n);
			ret_n += st->buf_ch[i].byte_n;
		}
	}

	if (st->first_push || !data)
		/* first push || pushing just timestamp */
		push = true;
	if (ts) {
		st->ts_diff = ts - st->ts;
		if (st->ts_diff < 0)
			dev_err(st->dev, "%s %s ts_diff=%lld\n",
				__func__, st->cfg->name, st->ts_diff);
		else if (st->on_change && (st->ts_diff <
					   (s64)st->us_period * 1000)) {
			/* data rate faster than requested */
			if (!st->first_push)
				push = false;
		}
	} else {
		st->flush = false;
		if (*st->fn_dev->sts & (NVS_STS_SPEW_MSG | NVS_STS_SPEW_DATA))
			dev_info(st->dev, "%s %s FLUSH\n",
				 __func__, st->cfg->name);
	}
	memcpy(&st->buf[st->buf_ch[n].buf_i], &ts,
	       st->buf_ch[n].byte_n);
	if (push) {
		ret = st->kif_fn->push(st);
		if (!ret) {
			if (ts) {
				st->first_push = false;
				st->ts = ts; /* log ts push */
				if (st->one_shot)
					/* disable one-shot after event */
					nvs_disable(st);
			}
			if (*st->fn_dev->sts & NVS_STS_SPEW_BUF) {
				n = st->buf_ch[n].buf_i + st->buf_ch[n].byte_n;
				for (i = 0; i < n; i++)
					dev_info(st->dev, "%s buf[%u]=%02X\n",
						 st->cfg->name, i, st->buf[i]);
				dev_info(st->dev, "%s ts=%lld  diff=%lld\n",
					 st->cfg->name, ts, st->ts_diff);
			}
		}
	}
	if ((*st->fn_dev->sts & NVS_STS_SPEW_DATA) && ts) {
		nvs_dbg_data(st, char_buf, sizeof(char_buf));
		dev_info(st->dev, "%s", char_buf);
	}
	if (!ret)
		/* return pushed byte count from data if no error.
		 * external entity can use as offset to next data set.
		 */
		ret = ret_n;
	return ret;
}

int nvs_handler(void *handle, void *buffer, s64 ts)
{
	struct nvs_state *st = (struct nvs_state *)handle;
	unsigned char *buf = buffer;
	int ret = 0;

	if (st)
		ret = nvs_buf_push(st, buf, ts);
	return ret;
}

static void nvs_report_mode(struct nvs_state *st)
{
	/* Currently this is called once during initialization. However, there
	 * may be mechanisms where this is allowed to change at runtime, hence
	 * this function above nvs_attr_store for st->cfg->flags.
	 */
	st->on_change = false;
	st->one_shot = false;
	st->special = false;
	switch (st->cfg->flags & REPORTING_MODE_MASK) {
	case SENSOR_FLAG_ON_CHANGE_MODE:
		st->on_change = true;
		break;

	case SENSOR_FLAG_ONE_SHOT_MODE:
		st->one_shot = true;
		break;

	case SENSOR_FLAG_SPECIAL_REPORTING_MODE:
		st->special = true;
		break;
	}
}

static ssize_t nvs_attr_ret(struct nvs_state *st, char *buf,
			    int ival, int fval)
{
	if (st->cfg->float_significance == NVS_FLOAT_NANO) {
		if (fval < 0)
			return snprintf(buf, PAGE_SIZE,
					"-%d.%09u\n", ival, -fval);

		return snprintf(buf, PAGE_SIZE, "%d.%09u\n", ival, fval);
	}

	if (fval < 0)
		return snprintf(buf, PAGE_SIZE, "-%d.%06u\n", ival, -fval);

	return snprintf(buf, PAGE_SIZE, "%d.%06u\n", ival, fval);
}

static int nvs_attr_float(struct nvs_state *st, const char *buf,
			  int *ival, int *fval)
{
	bool neg = false;
	bool i_part = true;
	int i = 0;
	int f = 0;
	int fs;

	if (st->cfg->float_significance == NVS_FLOAT_NANO)
		fs = NVS_FLOAT_SIGNIFICANCE_NANO;
	else
		fs = NVS_FLOAT_SIGNIFICANCE_MICRO;
	if (buf[0] == '-') {
		neg = true;
		buf++;
	} else if (buf[0] == '+') {
		buf++;
	}

	while (*buf) {
		if ('0' <= *buf && *buf <= '9') {
			if (i_part) {
				i = i * 10 + *buf - '0';
			} else {
				f += fs * (*buf - '0');
				fs /= 10;
			}
		} else if (*buf == '\n') {
			if (*(buf + 1) == '\0')
				break;
			else
				return -EINVAL;
		} else if (*buf == '.' && i_part) {
			i_part = false;
		} else {
			return -EINVAL;
		}
		buf++;
	}

	if (neg) {
		if (i)
			i = -i;
		else
			f = -f;
	}

	*ival = i;
	*fval = f;
	return 0;
}

static ssize_t nvs_attr_ch_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	struct nvs_dev_attr *nda = to_nvs_dev_attr(attr);

	return nvs_ch2buf(st, nda->channel, buf, PAGE_SIZE, 0, false);
}

static ssize_t nvs_attr_ch_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	struct nvs_dev_attr *nda = to_nvs_dev_attr(attr);
	unsigned int ch = nda->channel;
	int ret;
	u64 val;

	if (kstrtou64(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
				       (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND))) {
		ret = -EPERM;
	} else {
		/* writing to ch is a debug feature allowing a sticky data
		 * value to be pushed up and the same value pushed until the
		 * device is turned off.
		 */
		memcpy(&st->buf[st->buf_ch[ch].buf_i], &val,
		       st->buf_ch[ch].byte_n);
		st->dbg_data_lock |= (1 << ch);
		ret = nvs_buf_push(st, st->buf, nvs_timestamp());
		if (ret > 0)
			ret = 0;
	}
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_enable_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	ssize_t ret = -EINVAL;

	if (st->fn_dev->enable != nvs_fn_dev_enable) {
		mutex_lock(&st->mutex);
		ret = snprintf(buf, PAGE_SIZE, "%X\n",
			       st->fn_dev->enable(st->client,
						  st->cfg->snsr_id, -1));
		mutex_unlock(&st->mutex);
	}
	return ret;
}

static ssize_t nvs_attr_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	unsigned int enable;
	int ret = 0;

	if (kstrtouint(buf, 0, &enable))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
				       (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND))) {
		ret = -EPERM;
	} else {
		if (enable) {
			st->first_push = true;
			ret = st->fn_dev->enable(st->client, st->cfg->snsr_id,
						 enable);
		} else {
			ret = nvs_disable(st);
		}
	}
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s %X->%X ret=%d",
			 st->cfg->name, __func__, st->enabled, enable, ret);
	if (!ret)
		st->enabled = enable;
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_fmec_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", st->cfg->fifo_max_evnt_cnt);
}

static ssize_t nvs_attr_frec_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", st->cfg->fifo_rsrv_evnt_cnt);
}

static ssize_t nvs_attr_flags_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", st->cfg->flags);
}

static ssize_t nvs_attr_flags_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	unsigned int flags_old;
	unsigned int flags_new;

	if (kstrtoint(buf, 0, &flags_new))
		return -EINVAL;

	flags_old = st->cfg->flags;
	st->cfg->flags &= SENSOR_FLAG_READONLY_MASK;
	flags_new &= ~SENSOR_FLAG_READONLY_MASK;
	st->cfg->flags |= flags_new;
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s %u->%u\n",
			 st->cfg->name, __func__, flags_old, st->cfg->flags);
	return count;
}

static ssize_t nvs_attr_flush_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%x\n", st->flush);
}

static ssize_t nvs_attr_flush_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	int ret = 1;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
				       (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND))) {
		ret = -EPERM;
	} else {
		st->flush = true;
		if (st->fn_dev->flush)
			ret = st->fn_dev->flush(st->client, st->cfg->snsr_id);
	}
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s ret=%d\n",
			 st->cfg->name, __func__, ret);
	if (ret > 0)
		nvs_buf_push(st, NULL, 0);
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_matrix_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	ssize_t t = 0;
	unsigned int i;

	for (i = 0; i < 8; i++)
		t += snprintf(buf + t, PAGE_SIZE - t, "%hhd,",
			      st->cfg->matrix[i]);
	t += snprintf(buf + t, PAGE_SIZE - t, "%hhd\n",
		      st->cfg->matrix[i]);
	return t;
}

static ssize_t nvs_attr_matrix_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	s8 matrix[9];
	char *str;
	unsigned int i;
	int ret;

	for (i = 0; i < sizeof(matrix); i++) {
		str = strsep((char **)&buf, " \0");
		if (str) {
			ret = kstrtos8(str, 10, &matrix[i]);
			if (ret)
				break;

			if (matrix[i] < -1 || matrix[i] > 1) {
				ret = -EINVAL;
				break;
			}
		} else {
			ret = -EINVAL;
			break;
		}
	}
	if (i == sizeof(matrix))
		memcpy(st->cfg->matrix, matrix, sizeof(st->cfg->matrix));
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_milliamp_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%d.%06u\n",
			st->cfg->milliamp.ival, st->cfg->milliamp.fval);
}

static ssize_t nvs_attr_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", st->cfg->name);
}

static const char NVS_DBG_KEY[] = {
	0x54, 0x65, 0x61, 0x67, 0x61, 0x6E, 0x26, 0x52, 0x69, 0x69, 0x73, 0x00
};

static ssize_t nvs_attr_nvs_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	enum NVS_DBG dbg;
	unsigned int i;
	int ret = -EINVAL;

	dbg = st->dbg;
	st->dbg = NVS_INFO_DATA;
	switch (dbg) {
	case NVS_INFO_DATA:
		return nvs_dbg_data(st, buf, PAGE_SIZE);

	case NVS_INFO_VER:
		return snprintf(buf, PAGE_SIZE,
				"sysfs_version=%u driver_version=%u\n",
				NVS_SYSFS_DRIVER_VERSION,
				st->kif_fn->driver_version);

	case NVS_INFO_ERRS:
		i = *st->fn_dev->errs;
		*st->fn_dev->errs = 0;
		return snprintf(buf, PAGE_SIZE, "error count=%u\n", i);

	case NVS_INFO_RESET:
		if (st->fn_dev->reset) {
			mutex_lock(&st->mutex);
			ret = st->fn_dev->reset(st->client, st->cfg->snsr_id);
			mutex_unlock(&st->mutex);
		}
		if (ret)
			return snprintf(buf, PAGE_SIZE, "reset ERR\n");
		else
			return snprintf(buf, PAGE_SIZE, "reset done\n");

	case NVS_INFO_REGS:
		if (st->fn_dev->regs)
			return st->fn_dev->regs(st->client,
						st->cfg->snsr_id, buf);
		break;

	case NVS_INFO_CFG:
		return nvs_dbg_cfg(st, buf);

	case NVS_INFO_DBG_KEY:
		return snprintf(buf, PAGE_SIZE, "%s\n", NVS_DBG_KEY);

	case NVS_INFO_DBG:
		return snprintf(buf, PAGE_SIZE, "DBG spew=%x\n",
				!!(*st->fn_dev->sts & NVS_STS_SPEW_MSG));

	case NVS_INFO_DBG_DATA:
		return snprintf(buf, PAGE_SIZE, "DATA spew=%x\n",
				!!(*st->fn_dev->sts & NVS_STS_SPEW_DATA));

	case NVS_INFO_DBG_BUF:
		return snprintf(buf, PAGE_SIZE, "BUF spew=%x\n",
				!!(*st->fn_dev->sts & NVS_STS_SPEW_BUF));

	case NVS_INFO_DBG_IRQ:
		return snprintf(buf, PAGE_SIZE, "IRQ spew=%x\n",
				!!(*st->fn_dev->sts & NVS_STS_SPEW_IRQ));

	default:
		if (dbg < NVS_INFO_LIMIT_MAX)
			break;

		if (st->fn_dev->nvs_read)
			ret = st->fn_dev->nvs_read(st->client,
						   st->cfg->snsr_id, buf);
		break;
	}

	return ret;
}

static ssize_t nvs_attr_nvs_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	unsigned int dbg;
	int ret;

	ret = kstrtouint(buf, 0, &dbg);
	if (ret)
		return -EINVAL;

	st->dbg = dbg;
	switch (dbg) {
	case NVS_INFO_DATA:
		*st->fn_dev->sts &= ~NVS_STS_SPEW_MSK;
		break;

	case NVS_INFO_DBG:
		*st->fn_dev->sts ^= NVS_STS_SPEW_MSG;
		break;

	case NVS_INFO_DBG_DATA:
		*st->fn_dev->sts ^= NVS_STS_SPEW_DATA;
		break;

	case NVS_INFO_DBG_BUF:
		*st->fn_dev->sts ^= NVS_STS_SPEW_BUF;
		break;

	case NVS_INFO_DBG_IRQ:
		*st->fn_dev->sts ^= NVS_STS_SPEW_IRQ;
		break;

	case NVS_INFO_DBG_KEY:
		break;

	default:
		if (dbg < NVS_INFO_LIMIT_MAX)
			break;

		if (st->fn_dev->nvs_write) {
			st->dbg = NVS_INFO_LIMIT_MAX;
			dbg -= NVS_INFO_LIMIT_MAX;
			ret = st->fn_dev->nvs_write(st->client,
						    st->cfg->snsr_id, dbg);
			if (ret < 0)
				return ret;
		} else if (!st->fn_dev->nvs_read) {
			st->dbg = NVS_INFO_DATA;
			return -EINVAL;
		}

		break;
	}

	return count;
}

static ssize_t nvs_attr_offset_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	struct nvs_dev_attr *nda = to_nvs_dev_attr(attr);
	unsigned int ch = nda->channel;
	int ival;
	int fval;

	if (ch < NVS_CHANNEL_N_MAX) {
		ival = st->cfg->offsets[ch].ival;
		fval = st->cfg->offsets[ch].fval;
	} else {
		ival = st->cfg->offset.ival;
		fval = st->cfg->offset.fval;
	}
	return nvs_attr_ret(st, buf, ival, fval);
}

static ssize_t nvs_attr_offset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	struct nvs_dev_attr *nda = to_nvs_dev_attr(attr);
	unsigned int ch = nda->channel;
	int ival;
	int fval;
	int ret = 1;

	if (nvs_attr_float(st, buf, &ival, &fval))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
				       (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND))) {
		ret = -EPERM;
	} else if (ch < st->cfg->ch_n_max) {
		if (st->fn_dev->offset)
			ret = st->fn_dev->offset(st->client, st->cfg->snsr_id,
						 ch, ival);
		if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
			dev_info(st->dev, "%s %s ch=%u %d:%d->%d:%d ret=%d\n",
				 st->cfg->name, __func__, ch,
				 st->cfg->offsets[ch].ival,
				 st->cfg->offsets[ch].fval, ival, fval, ret);
		if (ret > 0) {
			st->cfg->offsets[ch].ival = ival;
			st->cfg->offsets[ch].fval = fval;
		}
	} else {
		if (st->fn_dev->offset)
			ret = st->fn_dev->offset(st->client, st->cfg->snsr_id,
						 -1, ival);
		if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
			dev_info(st->dev, "%s %s %d:%d->%d:%d ret=%d\n",
				 st->cfg->name, __func__, st->cfg->offset.ival,
				 st->cfg->offset.fval, ival, fval, ret);
		if (ret > 0) {
			st->cfg->offset.ival = ival;
			st->cfg->offset.fval = fval;
		}
	}
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_part_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%s %s\n",
			st->cfg->part, st->cfg->name);
}

static ssize_t nvs_attr_range_max_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	int ival = st->cfg->max_range.ival;
	int fval = st->cfg->max_range.fval;

	return nvs_attr_ret(st, buf, ival, fval);
}

static ssize_t nvs_attr_range_max_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	int ival;
	int fval;
	int ret = 1;

	if (nvs_attr_float(st, buf, &ival, &fval))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
					 (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND)))
		ret = -EPERM;
	else if (st->fn_dev->max_range)
		ret = st->fn_dev->max_range(st->client, st->cfg->snsr_id,
					    ival);
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s %d:%d->%d:%d ret=%d\n",
			 st->cfg->name, __func__, st->cfg->max_range.ival,
			 st->cfg->max_range.fval, ival, fval, ret);
	if (ret > 0) {
		st->cfg->max_range.ival = ival;
		st->cfg->max_range.fval = fval;
	}
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_resolution_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	int ival = st->cfg->resolution.ival;
	int fval = st->cfg->resolution.fval;

	return nvs_attr_ret(st, buf, ival, fval);
}

static ssize_t nvs_attr_resolution_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	int ival;
	int fval;
	int ret = 1;

	if (nvs_attr_float(st, buf, &ival, &fval))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
					 (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND)))
		ret = -EPERM;
	else if (st->fn_dev->resolution)
		ret = st->fn_dev->resolution(st->client, st->cfg->snsr_id,
					     ival);
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s %d:%d->%d:%d ret=%d\n",
			 st->cfg->name, __func__, st->cfg->resolution.ival,
			 st->cfg->resolution.fval, ival, fval, ret);
	if (ret > 0) {
		st->cfg->resolution.ival = ival;
		st->cfg->resolution.fval = fval;
	}
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_scale_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	struct nvs_dev_attr *nda = to_nvs_dev_attr(attr);
	unsigned int ch = nda->channel;
	int ival;
	int fval;

	if (ch < NVS_CHANNEL_N_MAX) {
		ival = st->cfg->scales[ch].ival;
		fval = st->cfg->scales[ch].fval;
	} else {
		ival = st->cfg->scale.ival;
		fval = st->cfg->scale.fval;
	}
	return nvs_attr_ret(st, buf, ival, fval);
}

static ssize_t nvs_attr_scale_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	struct nvs_dev_attr *nda = to_nvs_dev_attr(attr);
	unsigned int ch = nda->channel;
	int ival;
	int fval;
	int ret = 1;

	if (nvs_attr_float(st, buf, &ival, &fval))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
				       (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND))) {
		ret = -EPERM;
	} else if (ch < st->cfg->ch_n_max) {
		if (st->fn_dev->scale)
			ret = st->fn_dev->scale(st->client, st->cfg->snsr_id,
						ch, ival);
		if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
			dev_info(st->dev, "%s %s ch=%u %d:%d->%d:%d ret=%d\n",
				 st->cfg->name, __func__, ch,
				 st->cfg->scales[ch].ival,
				 st->cfg->scales[ch].fval, ival, fval, ret);
		if (ret > 0) {
			st->cfg->scales[ch].ival = ival;
			st->cfg->scales[ch].fval = fval;
		}
	} else {
		if (st->fn_dev->scale)
			ret = st->fn_dev->scale(st->client, st->cfg->snsr_id,
						-1, ival);
		if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
			dev_info(st->dev, "%s %s %d:%d->%d:%d ret=%d\n",
				 st->cfg->name, __func__, st->cfg->scale.ival,
				 st->cfg->scale.fval, ival, fval, ret);
		if (ret > 0) {
			st->cfg->scale.ival = ival;
			st->cfg->scale.fval = fval;
		}
	}
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_self_test_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	ssize_t ret = -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
					 (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND)))
		ret = -EPERM;
	else if (st->fn_dev->self_test)
		ret = st->fn_dev->self_test(st->client, st->cfg->snsr_id, buf);
	mutex_unlock(&st->mutex);
	return ret;
}

static ssize_t nvs_attr_thresh_hi_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", st->cfg->thresh_hi);
}

static ssize_t nvs_attr_thresh_hi_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	int thresh_hi;
	int ret = 1;

	if (kstrtoint(buf, 0, &thresh_hi))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
					 (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND)))
		ret = -EPERM;
	else if (st->fn_dev->thresh_hi)
		ret = st->fn_dev->thresh_hi(st->client, st->cfg->snsr_id,
					    thresh_hi);
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s %d->%d ret=%d\n",
			 st->cfg->name, __func__,
			 st->cfg->thresh_hi, thresh_hi, ret);
	if (ret > 0)
		st->cfg->thresh_hi = thresh_hi;
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_thresh_lo_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", st->cfg->thresh_lo);
}

static ssize_t nvs_attr_thresh_lo_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	int thresh_lo;
	int ret = 1;

	if (kstrtoint(buf, 0, &thresh_lo))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
					 (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND)))
		ret = -EPERM;
	else if (st->fn_dev->thresh_lo)
		ret = st->fn_dev->thresh_lo(st->client, st->cfg->snsr_id,
					    thresh_lo);
	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s %d->%d ret=%d\n",
			 st->cfg->name, __func__,
			 st->cfg->thresh_lo, thresh_lo, ret);
	if (ret > 0)
		st->cfg->thresh_lo = thresh_lo;
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_us_period_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	unsigned int period_us;
	int ret;

	if (st->fn_dev->enable == nvs_fn_dev_enable) {
		ret = st->enabled;
	} else {
		ret = st->fn_dev->enable(st->client,
					 st->cfg->snsr_id, -1);
		if (ret < 0)
			return ret;
	}

	if (ret) {
		if (st->fn_dev->batch_read) {
			ret = st->fn_dev->batch_read(st->client,
						     st->cfg->snsr_id,
						     &period_us, NULL);
			if (ret < 0)
				return ret;
		} else {
			period_us = st->us_period;
		}
	} else {
		period_us = st->cfg->delay_us_min;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", period_us);
}

static ssize_t nvs_attr_us_period_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	unsigned int us_period;
	int ret = 0;

	if (kstrtouint(buf, 0, &us_period))
		return -EINVAL;

	mutex_lock(&st->mutex);
	if (st->shutdown || st->suspend || (*st->fn_dev->sts &
				       (NVS_STS_SHUTDOWN | NVS_STS_SUSPEND))) {
		ret = -EPERM;
	} else {
		if (!st->one_shot) {
			if (us_period < st->cfg->delay_us_min)
				us_period = st->cfg->delay_us_min;
			if (st->cfg->delay_us_max && (us_period >
						      st->cfg->delay_us_max))
				us_period = st->cfg->delay_us_max;
		}
		if (st->fn_dev->batch) {
			ret = st->fn_dev->batch(st->client, st->cfg->snsr_id,
						0, us_period, st->us_timeout);
		} else {
			if (st->us_timeout)
				ret = -EINVAL;
		}
		if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
			dev_info(st->dev, "%s %s %u->%u ret=%d\n",
				 st->cfg->name, __func__,
				 st->us_period, us_period, ret);
		if (!ret)
			st->us_period = us_period;
	}
	mutex_unlock(&st->mutex);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t nvs_attr_us_timeout_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	unsigned int timeout_us;
	int ret;

	if (st->fn_dev->enable == nvs_fn_dev_enable) {
		ret = st->enabled;
	} else {
		ret = st->fn_dev->enable(st->client,
					 st->cfg->snsr_id, -1);
		if (ret < 0)
			return ret;
	}

	if (ret) {
		if (st->fn_dev->batch_read) {
			ret = st->fn_dev->batch_read(st->client,
						     st->cfg->snsr_id,
						     NULL, &timeout_us);
			if (ret < 0)
				return ret;
		} else {
			timeout_us = st->us_timeout;
		}
	} else {
		timeout_us = st->cfg->delay_us_max;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", timeout_us);
}

static ssize_t nvs_attr_us_timeout_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	unsigned int us_timeout;

	if (kstrtouint(buf, 0, &us_timeout))
		return -EINVAL;

	if (*st->fn_dev->sts & NVS_STS_SPEW_MSG)
		dev_info(st->dev, "%s %s %u->%u\n",
			 st->cfg->name, __func__, st->us_timeout, us_timeout);
	st->us_timeout = us_timeout;
	return count;
}

static ssize_t nvs_attr_uuid_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);
	ssize_t t = 0;
	unsigned int i;

	for (i = 0; i < 15; i++)
		t += snprintf(buf + t, PAGE_SIZE - t, "%02X ",
			      st->cfg->uuid[i]);
	t += snprintf(buf + t, PAGE_SIZE - t, "%02X\n", st->cfg->uuid[i]);
	return t;
}

static ssize_t nvs_attr_vendor_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", st->cfg->vendor);
}

static ssize_t nvs_attr_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct nvs_state *st = dev_to_nvs_state(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", st->cfg->version);
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_enable_show, nvs_attr_enable_store);
static DEVICE_ATTR(fifo_max_event_count, S_IRUGO,
		   nvs_attr_fmec_show, NULL);
static DEVICE_ATTR(fifo_reserved_event_count, S_IRUGO,
		   nvs_attr_frec_show, NULL);
static DEVICE_ATTR(flags, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_flags_show, nvs_attr_flags_store);
static DEVICE_ATTR(flush, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_flush_show, nvs_attr_flush_store);
/* matrix permissions are read only - writes are for debug */
static DEVICE_ATTR(matrix, S_IRUGO,
		   nvs_attr_matrix_show, nvs_attr_matrix_store);
static DEVICE_ATTR(milliamp, S_IRUGO,
		   nvs_attr_milliamp_show, NULL);
static DEVICE_ATTR(name, S_IRUGO,
		   nvs_attr_name_show, NULL);
static DEVICE_ATTR(nvs, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_nvs_show, nvs_attr_nvs_store);
static DEVICE_ATTR(part, S_IRUGO,
		   nvs_attr_part_show, NULL);
static DEVICE_ATTR(range_max, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_range_max_show, nvs_attr_range_max_store);
static DEVICE_ATTR(resolution, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_resolution_show, nvs_attr_resolution_store);
static DEVICE_ATTR(self_test, S_IRUGO,
		   nvs_attr_self_test_show, NULL);
static DEVICE_ATTR(thresh_hi, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_thresh_hi_show, nvs_attr_thresh_hi_store);
static DEVICE_ATTR(thresh_lo, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_thresh_lo_show, nvs_attr_thresh_lo_store);
static DEVICE_ATTR(us_period, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_us_period_show, nvs_attr_us_period_store);
static DEVICE_ATTR(us_timeout, S_IRUGO | S_IWUSR | S_IWGRP,
		   nvs_attr_us_timeout_show, nvs_attr_us_timeout_store);
static DEVICE_ATTR(uuid, S_IRUGO,
		   nvs_attr_uuid_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO,
		   nvs_attr_vendor_show, NULL);
static DEVICE_ATTR(version, S_IRUGO,
		   nvs_attr_version_show, NULL);

/* commented attributes are just FYI and dynamically created */
static struct attribute *nvs_attrs[] = {
	/* &dev_attr_ch, */
	&dev_attr_enable.attr,
	&dev_attr_fifo_max_event_count.attr,
	&dev_attr_fifo_reserved_event_count.attr,
	&dev_attr_flags.attr,
	&dev_attr_flush.attr,
	&dev_attr_matrix.attr,
	&dev_attr_milliamp.attr,
	&dev_attr_name.attr,
	&dev_attr_nvs.attr,
	/* &dev_attr_offset.attr, */
	&dev_attr_part.attr,
	&dev_attr_range_max.attr,
	&dev_attr_resolution.attr,
	/* &dev_attr_scale.attr, */
	&dev_attr_self_test.attr,
	&dev_attr_thresh_hi.attr,
	&dev_attr_thresh_lo.attr,
	&dev_attr_us_period.attr,
	&dev_attr_us_timeout.attr,
	&dev_attr_uuid.attr,
	&dev_attr_vendor.attr,
	&dev_attr_version.attr,
	NULL
};

static int nvs_attr_rm(struct nvs_state *st, struct attribute *attr)
{
	unsigned int i;
	unsigned int n;

	n = ARRAY_SIZE(nvs_attrs) - 1;
	for (i = 0; i < n; i++) {
		if (!st->attrs[i])
			return -EINVAL;

		if (st->attrs[i] == attr) {
			for (; i < n; i++)
				st->attrs[i] = st->attrs[i + 1];
			return 0;
		}
	}

	return -EINVAL;
}

static int nvs_init_sysfs(struct nvs_state *st)
{
	bool matrix = false;
	unsigned int i;
	unsigned int j;
	unsigned int k;
	unsigned int n;

	i = st->cfg->snsr_id; /* st->cfg->snsr_id will be >= 0 */
	/* Here we have two ways to identify the sensor:
	 * 1. By name which we try to match to
	 * 2. By sensor ID (st->cfg->snsr_id).  This method is typically used
	 *    by the sensor hub so that name strings don't have to be passed
	 *    and because there are multiple sensors the sensor hub driver has
	 *    to track via the sensor ID.
	 */
	if (st->cfg->name) {
		/* if st->cfg->name exists then we use that */
		for (i = 0; i < ARRAY_SIZE(nvs_snsr_names); i++) {
			if (!strcmp(st->cfg->name, nvs_snsr_names[i]))
				break;
		}
		if (i >= ARRAY_SIZE(nvs_snsr_names))
			i = 0; /* use generic sensor name */
	} else {
		/* no st->cfg->name - use st->cfg->snsr_id to specify device */
		if (i >= ARRAY_SIZE(nvs_snsr_names))
			i = 0; /* use generic sensor name */

		st->cfg->name = nvs_snsr_names[i];
	}

	st->snsr_type = i;
	/* count attributes */
	if (st->cfg->ch_n_max)
		n = (st->cfg->ch_n_max << 1); /* scales and offsets */
	else
		n = 2; /* scale and offset */
	/* allocate memory for scale(s) and offset(s) attributes */
	st->attr_so = kzalloc(n * sizeof(st->attr_so[0]), GFP_KERNEL);
	if (!st->attr_so)
		return -ENOMEM;

	st->attr_so_n = n;
	/* allocate memory for channel attributes */
	st->attr_ch = kzalloc(st->ch_n * sizeof(st->attr_ch[0]), GFP_KERNEL);
	if (!st->attr_so)
		return -ENOMEM;

	n += st->ch_n;
	n += ARRAY_SIZE(nvs_attrs);
	/* subtract the ones that will be removed */
	if (!st->fn_dev->enable)
		n--;
	if ((st->special || st->one_shot) && !st->fn_dev->batch)
		n -= 2;
	if ((st->special || st->one_shot) && !st->fn_dev->flush)
		n--;
	if ((!st->cfg->thresh_lo) && (!st->cfg->thresh_hi) &&
			  (!st->fn_dev->thresh_lo) && (!st->fn_dev->thresh_hi))
		n -= 2;
	if (!st->fn_dev->self_test)
		n--;
	/* test if matrix data */
	for (i = 0; i < ARRAY_SIZE(st->cfg->matrix); i++) {
		if (st->cfg->matrix[i]) {
			matrix = true;
			break;
		}
	}
	if (!matrix)
		n--;
	/* allocate memory */
	if (n < ARRAY_SIZE(nvs_attrs))
		n = ARRAY_SIZE(nvs_attrs);
	st->attrs = kzalloc(n * sizeof(st->attrs[0]), GFP_KERNEL);
	if (!st->attrs)
		return -ENOMEM;

	memcpy(st->attrs, nvs_attrs, n * sizeof(st->attrs[0]));
	/* remove unused attributes */
	if (!st->fn_dev->enable)
		nvs_attr_rm(st, &dev_attr_enable.attr);
	if ((st->special || st->one_shot) && !st->fn_dev->batch) {
		nvs_attr_rm(st, &dev_attr_us_period.attr);
		nvs_attr_rm(st, &dev_attr_us_timeout.attr);
	}
	if ((st->special || st->one_shot) && !st->fn_dev->flush)
		nvs_attr_rm(st, &dev_attr_flush.attr);
	if ((!st->cfg->thresh_lo) && (!st->cfg->thresh_hi) &&
			(!st->fn_dev->thresh_lo) && (!st->fn_dev->thresh_hi)) {
		nvs_attr_rm(st, &dev_attr_thresh_lo.attr);
		nvs_attr_rm(st, &dev_attr_thresh_hi.attr);
	}
	if (!st->fn_dev->self_test)
		nvs_attr_rm(st, &dev_attr_self_test.attr);
	if (!matrix)
		nvs_attr_rm(st, &dev_attr_matrix.attr);
	/* find end of list */
	for (i = 0; i < ARRAY_SIZE(nvs_attrs); i++) {
		if (!st->attrs[i])
			break;
	}

	/* create scale and offset attributes */
	n = st->attr_so_n >> 1;
	for (j = 0; j < n; j++) {
		k = j << 1;
		if (st->cfg->ch_n_max) {
			st->attr_so[k].channel = j;
			st->attr_so[k].dev_attr.attr.name =
					  kasprintf(GFP_KERNEL, "scale%u", j);
		} else {
			st->attr_so[k].channel = -1;
			st->attr_so[k].dev_attr.attr.name =
						kasprintf(GFP_KERNEL, "scale");
		}
		st->attr_so[k].dev_attr.attr.mode =
						   S_IRUGO | S_IWUSR | S_IWGRP;
		st->attr_so[k].dev_attr.show = nvs_attr_scale_show;
		st->attr_so[k].dev_attr.store = nvs_attr_scale_store;
		st->attrs[i] = &st->attr_so[k].dev_attr.attr;
		i++;
	}

	for (j = 0; j < n; j++) {
		k = (j << 1) + 1;
		if (st->cfg->ch_n_max) {
			st->attr_so[k].channel = j;
			st->attr_so[k].dev_attr.attr.name =
					 kasprintf(GFP_KERNEL, "offset%u", j);
		} else {
			st->attr_so[k].channel = -1;
			st->attr_so[k].dev_attr.attr.name =
					       kasprintf(GFP_KERNEL, "offset");
		}
		st->attr_so[k].dev_attr.attr.mode =
						   S_IRUGO | S_IWUSR | S_IWGRP;
		st->attr_so[k].dev_attr.show = nvs_attr_offset_show;
		st->attr_so[k].dev_attr.store = nvs_attr_offset_store;
		st->attrs[i] = &st->attr_so[k].dev_attr.attr;
		i++;
	}

	/* create channel attributes */
	n = st->ch_n - 1; /* - timestamp */
	for (j = 0; j < n; j++) {
		st->attr_ch[j].channel = j;
		if (st->buf_ch[j].sign)
			st->attr_ch[j].dev_attr.attr.name =
					    kasprintf(GFP_KERNEL,
						      "ch%u_%u_s_%u",
						      j, st->buf_ch[j].buf_i,
						      st->buf_ch[j].byte_n);
		else
			st->attr_ch[j].dev_attr.attr.name =
					    kasprintf(GFP_KERNEL,
						      "ch%u_%u_u_%u",
						      j, st->buf_ch[j].buf_i,
						      st->buf_ch[j].byte_n);
		st->attr_ch[j].dev_attr.attr.mode =
						   S_IRUGO | S_IWUSR | S_IWGRP;
		st->attr_ch[j].dev_attr.show = nvs_attr_ch_show;
		st->attr_ch[j].dev_attr.store = nvs_attr_ch_store;
		st->attrs[i] = &st->attr_ch[j].dev_attr.attr;
		i++;
	}

	/* timestamp */
	st->attr_ch[j].channel = n;
	st->attr_ch[j].dev_attr.attr.name = kasprintf(GFP_KERNEL,
						      "ch%u_%u_t_%u", j,
						      st->buf_ch[j].buf_i,
						      st->buf_ch[j].byte_n);
	st->attr_ch[j].dev_attr.attr.mode = S_IRUGO;
	st->attr_ch[j].dev_attr.show = nvs_attr_ch_show;
	st->attr_ch[j].dev_attr.store = NULL;
	st->attrs[i] = &st->attr_ch[j].dev_attr.attr;
	i++;
	for (j = 0; j < i; j++) {
		sysfs_attr_init(st->attrs[j]);
	}
	st->attrs[i] = NULL;
	st->attr_grp.attrs = st->attrs;
	return 0;
}

static int nvs_init_buf(struct nvs_state *st)
{
	unsigned int buf_n;
	unsigned int i;
	unsigned int n;

	n = st->cfg->ch_n;
	buf_n = abs(st->cfg->ch_sz);
	buf_n *= n;
	if (st->cfg->snsr_data_n > buf_n)
		/* extra channel */
		n++;
	n++; /* timestamp */
	st->ch_n = n;
	/* allocate buffer index memory */
	st->buf_ch = kzalloc(sizeof(st->buf_ch[0]) * st->ch_n, GFP_KERNEL);
	if (!st->buf_ch)
		return -ENOMEM;

	/* data channels */
	for (i = 0; i < st->cfg->ch_n; i++) {
		if (st->cfg->ch_sz < 0) {
			st->buf_ch[i].sign = true;
			st->buf_ch[i].byte_n = abs(st->cfg->ch_sz);
		} else {
			st->buf_ch[i].sign = false;
			st->buf_ch[i].byte_n = st->cfg->ch_sz;
		}
	}

	if (st->cfg->snsr_data_n > buf_n) {
		/* extra channel (status) */
		st->buf_ch[i].sign = false;
		st->buf_ch[i].byte_n = st->cfg->snsr_data_n - buf_n;
		buf_n += st->buf_ch[i].byte_n;
		i++;
	}
	/* timestamp */
	st->buf_ch[i].sign = false;
	st->buf_ch[i].byte_n = sizeof(nvs_timestamp());
	buf_n += st->buf_ch[i].byte_n;
	/* allocate buffer memory */
	st->buf = kzalloc(buf_n, GFP_KERNEL);
	if (!st->buf)
		return -ENOMEM;

	/* buffer indexes */
	buf_n = 0;
	for (i = 1; i < st->ch_n; i++) {
		buf_n += st->buf_ch[i - 1].byte_n;
		st->buf_ch[i].buf_i = buf_n;
	}

	return 0;
}

int nvs_suspend(void *handle)
{
	struct nvs_state *st = (struct nvs_state *)handle;
	int ret = 0;

	if (st == NULL)
		return 0;

	mutex_lock(&st->mutex);
	st->suspend = true;
	if (!(st->cfg->flags & SENSOR_FLAG_WAKE_UP)) {
		ret = st->fn_dev->enable(st->client, st->cfg->snsr_id, -1);
		if (ret >= 0)
			st->enabled = ret;
		if (ret > 0)
			ret = st->fn_dev->enable(st->client,
						 st->cfg->snsr_id, 0);
		else
			ret = 0;
	}
	mutex_unlock(&st->mutex);
	return ret;
}

int nvs_resume(void *handle)
{
	struct nvs_state *st = (struct nvs_state *)handle;
	int ret = 0;

	if (st == NULL)
		return 0;

	mutex_lock(&st->mutex);
	if (!(st->cfg->flags & SENSOR_FLAG_WAKE_UP)) {
		if (st->enabled)
			ret = st->fn_dev->enable(st->client,
						st->cfg->snsr_id, st->enabled);
	}
	st->suspend = false;
	mutex_unlock(&st->mutex);
	return ret;
}

static void nvs_remove_sysfs(struct nvs_state *st)
{
	unsigned int i;

	kfree(st->attrs);
	if (st->attr_so) {
		for (i = 0; i < st->attr_so_n; i++)
			kfree((void *)st->attr_so[i].dev_attr.attr.name);
		kfree(st->attr_so);
		st->attr_so_n = 0;
	}

	if (st->attr_ch) {
		for (i = 0; i < st->ch_n; i++)
			kfree((void *)st->attr_ch[i].dev_attr.attr.name);
		kfree(st->attr_ch);
		st->ch_n = 0;
	}
}

static void nvs_remove_buf(struct nvs_state *st)
{
	kfree(st->buf_ch);
	kfree(st->buf);
}

void nvs_shutdown(void *handle)
{
	struct nvs_state *st = (struct nvs_state *)handle;
	int ret;

	if (st == NULL)
		return;

	mutex_lock(&st->mutex);
	st->shutdown = true;
	ret = st->fn_dev->enable(st->client, st->cfg->snsr_id, -1);
	if (ret > 0)
		st->fn_dev->enable(st->client, st->cfg->snsr_id, 0);
	mutex_unlock(&st->mutex);
}

int nvs_remove(void *handle)
{
	struct nvs_state *st = (struct nvs_state *)handle;

	if (st == NULL)
		return 0;

	st->kif_fn->remove(st);
	nvs_remove_sysfs(st);
	nvs_remove_buf(st);
	if (st->cfg->name)
		dev_info(st->dev, "%s %s snsr_id=%d\n",
			 __func__, st->cfg->name, st->cfg->snsr_id);
	else
		dev_info(st->dev, "%s snsr_id=%d\n",
			 __func__, st->cfg->snsr_id);
	kfree(st);
	return 0;
}

static int nvs_init(struct nvs_state *st)
{
	int ret;

	mutex_init(&st->mutex);
	nvs_report_mode(st);
	ret = nvs_init_buf(st);
	if (ret) {
		dev_err(st->dev, "%s nvs_init_buf ERR=%d\n", __func__, ret);
		return ret;
	}

	ret = nvs_init_sysfs(st);
	if (ret) {
		dev_err(st->dev, "%s nvs_init_sysfs ERR=%d\n", __func__, ret);
		return ret;
	}

	ret = st->kif_fn->init(st);
	if (ret) {
		dev_err(st->dev, "%s nvs_init_kif ERR=%d\n", __func__, ret);
		return ret;
	}
	return 0;
}

int nvs_probe(void **handle, void *dev_client, struct device *dev,
	      struct nvs_fn_dev *fn_dev, struct sensor_cfg *snsr_cfg,
	      struct nvs_kif_fn *kif_fn, unsigned int kif_st_n)
{
	struct nvs_state *st;
	unsigned int n;
	int ret;

	if (kif_fn) {
		dev_info(dev, "%s (%s)\n", __func__, kif_fn->name);
	} else {
		dev_err(dev, "%s ERR: kif_fn NULL\n", __func__);
		return -ENODEV;
	}

	if (!snsr_cfg) {
		dev_err(dev, "%s ERR: snsr_cfg NULL\n", __func__);
		return -ENODEV;
	}

	if (snsr_cfg->snsr_id < 0) {
		/* device has been disabled */
		if (snsr_cfg->name)
			dev_info(dev, "%s %s disabled\n",
				 __func__, snsr_cfg->name);
		else
			dev_info(dev, "%s device disabled\n", __func__);
		return -ENODEV;
	}

	n = sizeof(*st);
	if (kif_st_n) {
		n = ALIGN(n, NVS_ALIGN);
		n += kif_st_n;
	}
	n += NVS_ALIGN - 1;
	st = kzalloc(n, GFP_KERNEL);
	if (!st) {
		dev_err(dev, "%s kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	dev_set_drvdata(dev, st);
	st->kif_fn = kif_fn;
	st->client = dev_client;
	st->dev = dev;
	st->fn_dev = fn_dev;
	/* protect ourselves from NULL pointers */
	if (st->fn_dev->enable == NULL)
		/* hook a dummy benign function */
		st->fn_dev->enable = nvs_fn_dev_enable;
	if (st->fn_dev->sts == NULL)
		st->fn_dev->sts = &st->fn_dev_sts;
	if (st->fn_dev->errs == NULL)
		st->fn_dev->errs = &st->fn_dev_errs;
	/* all other pointers are tested for NULL in this code */
	st->cfg = snsr_cfg;
	ret = nvs_init(st);
	if (ret) {
		if (st->cfg->name)
			dev_err(st->dev, "%s %s snsr_id=%d EXIT ERR=%d\n",
				__func__, st->cfg->name,
				st->cfg->snsr_id, ret);
		else
			dev_err(st->dev, "%s snsr_id=%d EXIT ERR=%d\n",
				__func__, st->cfg->snsr_id, ret);
		nvs_remove(st);
	} else {
		*handle = st;
	}
	dev_info(st->dev, "%s (%s) %s done\n",
		 __func__, kif_fn->name, st->cfg->name);
	return ret;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor sysfs module");
MODULE_AUTHOR("NVIDIA Corporation");

