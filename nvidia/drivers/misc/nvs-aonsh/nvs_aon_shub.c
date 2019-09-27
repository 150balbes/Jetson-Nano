/*
 * Sensor Hub driver for NVIDIA's Tegra186 AON Sensor Processing Engine.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * NVS = NVidia Sensor framework
 * See nvs_iio.c and nvs.h for nvs documentation
 * Here is the flow of this driver. Failure in any step causes the driver
 * probe to fail :
 * 1. On boot, the kernel driver fetches the sensor count and all the
 *    sensor chips supported by the sensor hub.
 * 2. It uses the fetched sensor chips from sensor hub to validate the
 *    device tree nodes which represent a sensor chip and respective
 *    attributes.
 * 3. It then fetches the I2C controller info from the DT and validates
 *    the controller id. It creates a setup request comprising the sensor
 *    chip attributes such as the I2C controller it is connected as well
 *    as the I2C address of the chip, GPIO connected to the chip and sends
 *    it to the sensor hub.
 * 4. Upon validating the sensor chip nodes, it uses the I2C controllers
 *    info (controller id + clock rate) and sends an I2C init request to
 *    the sensor hub. Upon successful initialization of the I2C controller,
 *    the driver sends each sensor chip init request. This request also
 *    includes the init request of the slave chip connected over the
 *    auxiliary I2C bus of the chip if it supports one.
 * 5. Upon successful initialization of the I2C controllers and the sensor
 *    chips, the driver fetches the sensor config list supported by each
 *    sensor chip and builds a local copy of it. The driver does not fetch
 *    the entire list present on the sensor hub rather only fetches the
 *    configs supported by the chips passed in the DT node.
 * 6. Upon building the local sensor config table, the driver calls
 *    nvs_probe which sets up all the nvs iio attributes for each sensor
 *    config and creates the iio sysfs nodes for the NVS HAL to communicate.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/of_address.h>
#include <linux/tegra-aon.h>
#include <linux/mailbox_client.h>
#include <linux/time.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>
#include <linux/jiffies.h>
#include <linux/trace_imu.h>

#include <asm/io.h>
#include <asm/arch_timer.h>

#include "aon-shub-messages.h"

#define READJUST_TS_SAMPLES (100)

#define AXIS_N		3
/* block period in ms */
#define TX_BLOCK_PERIOD 200
#define INIT_TOUT_COUNT 15
#define IVC_TIMEOUT	200

#define SENSOR_TYPE_UNKNOWN 0

enum I2C_IDS {
	I2CID_MIN = 1,
	I2C2 = 1,
	I2C8 = 2,
	I2CID_MAX = 2,
};

enum GPIO_CTLR_IDS {
	GPIO_CHIP_ID_MIN,
	AON_GPIO_CHIP_ID = GPIO_CHIP_ID_MIN,
	MAIN_GPIO_CHIP_ID,
	GPIO_CHIP_ID_MAX = MAIN_GPIO_CHIP_ID,
};

struct aon_shub_sensor {
	char name[32];
	char part[32];
	char vendor[32];
	void *nvs_st;
	struct sensor_cfg cfg;
	int type;
	bool genable;	/* global enable */
};

struct shub_chip_node {
	struct device_node *dn;
	u8 chip_id;
};

struct tegra_aon_shub {
	struct device		 *dev;
	struct completion	 *wait_on;
	struct mbox_client	 cl;
	struct mbox_chan	 *mbox;
	struct aon_shub_request	 *shub_req;
	struct aon_shub_response *shub_resp;
	/* Interface to the NVS framework */
	struct nvs_fn_if	 *nvs;
	struct aon_shub_sensor	 **snsrs;
	struct mutex		 shub_mutex;
	struct snsr_chip	 *chips;
	struct shub_chip_node	 *chip_nodes;
	unsigned int		 snsr_cnt;
	u32			 nchips;
	u32			 i2c_clk_rates[I2CID_MAX];
	u32			 chip_id_mask;
	u32			 active_snsr_msk;
	u32			 adjust_ts_counter;
	u64			 ts_res_ns;
	s64			 ts_adjustment;
	bool			 last_tx_done;
};

static const char *const snsr_types[] = {
	[SENSOR_TYPE_UNKNOWN]		= "generic_sensor",
	[SENSOR_TYPE_ACCELEROMETER]	= "accelerometer",
	[SENSOR_TYPE_MAGNETIC_FIELD]	= "magnetic_field",
	[SENSOR_TYPE_ORIENTATION]	= "orientation",
	[SENSOR_TYPE_GYROSCOPE]		= "gyroscope",
};

/* This has to be a multiple of the cache line size */
static inline int ivc_min_frame_size(void)
{
	return cache_line_size();
}

static inline int get_snsr_type(const char *snsr_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(snsr_types); i++) {
		if (!strcmp(snsr_types[i], snsr_name))
			return i;
	}

	return -1;
}

/*
 * Android sensor service expects sensor events to have timestamps in
 * Monotonic time base. However, the sensor events on the sensor hub
 * are timestamped at the TKE timebase. This API provides a conversion
 * from the TKE timebase to the monotonic timebase.
 */
static inline s64 get_ts_adjustment(u64 tsc_res)
{
	s64 tsc = 0;
	struct timespec64 ts;
	s64 delta = 0;
	s64 mono_time = 0;
	unsigned long flags;
	DEFINE_RAW_SPINLOCK(slock);

	raw_spin_lock_irqsave(&slock, flags);
	tsc = (s64)(arch_counter_get_cntvct() * tsc_res);

	ktime_get_ts64(&ts);
	mono_time = timespec_to_ns(&ts);

	delta = mono_time - tsc;
	raw_spin_unlock_irqrestore(&slock, flags);

	return delta;
}

static void tegra_aon_shub_mbox_rcv_msg(struct mbox_client *cl, void *rx_msg)
{
	struct tegra_aon_mbox_msg *msg = rx_msg;
	struct tegra_aon_shub *shub = dev_get_drvdata(cl->dev);
	struct aon_shub_response *shub_resp;
	int snsr_id;
	u32 i;
	s64 ts;
	int cookie;

	shub_resp = (struct aon_shub_response *)msg->data;
	if (shub_resp->resp_type == AON_SHUB_REQUEST_PAYLOAD) {
		i = shub_resp->data.payload.count;
		if (i > ARRAY_SIZE(shub_resp->data.payload.data) || i == 0) {
			dev_err(shub->dev,
				"Invalid payload count\n");
			return;
		}
		if (shub->adjust_ts_counter == READJUST_TS_SAMPLES) {
			shub->ts_adjustment =
				get_ts_adjustment(shub->ts_res_ns);
			shub->adjust_ts_counter = 0;
		}
		shub->adjust_ts_counter++;
		while (i--) {
			snsr_id = shub_resp->data.payload.data[i].snsr_id;
			ts = (s64)shub_resp->data.payload.data[i].ts;
			ts += shub->ts_adjustment;
			shub_resp->data.payload.data[i].ts = (u64)ts;
			cookie = COOKIE(shub->snsrs[snsr_id]->type, ts);
			trace_async_atrace_begin(__func__, TRACE_SENSOR_ID, cookie);
			shub->nvs->handler(shub->snsrs[snsr_id]->nvs_st,
				&shub_resp->data.payload.data[i].x,
				shub_resp->data.payload.data[i].ts);
			trace_async_atrace_end(__func__, TRACE_SENSOR_ID, cookie);
		}
	} else {
		memcpy(shub->shub_resp, msg->data, sizeof(*shub->shub_resp));
		complete(shub->wait_on);
	}
}

static void tegra_aon_shub_mbox_tx_done(struct mbox_client *cl, void *tx_msg,
					int r)
{
	struct tegra_aon_shub *shub = dev_get_drvdata(cl->dev);

	shub->last_tx_done = !r;
}

static int tegra_aon_shub_ivc_msg_send(struct tegra_aon_shub *shub, int len, int timeout)
{
	int status;
	struct tegra_aon_mbox_msg msg;

	shub->last_tx_done = false;
	msg.length = len;
	msg.data = (void *)shub->shub_req;
	status = mbox_send_message(shub->mbox, (void *)&msg);
	if (status < 0) {
		dev_err(shub->dev, "mbox_send_message() failed with %d\n",
			status);
	} else {
		status = wait_for_completion_timeout(shub->wait_on,
						msecs_to_jiffies(timeout));
		if (status == 0) {
			dev_err(shub->dev, "Timeout: failed on IVC %s\n",
				shub->last_tx_done ? "RX" : "TX");
			return -ETIMEDOUT;
		}
		status = shub->shub_resp->status;
	}

	return status;
}

static int tegra_aon_shub_batch_read(void *client, int snsr_id,
				     unsigned int *period_us,
				     unsigned int *timeout_us)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;
	int ret;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_BATCH_RD;
	shub->shub_req->data.batch_rd.snsr_id = snsr_id;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret)
		dev_err(shub->dev, "batch_read ERR: snsr_id: %d!\n", snsr_id);

	if (timeout_us)
		*timeout_us = shub->shub_resp->data.batch_rd.timeout_us;
	if (period_us)
		*period_us = shub->shub_resp->data.batch_rd.period_us;
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_batch(void *client, int snsr_id, int flags,
				unsigned int period, unsigned int timeout)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;
	int ret = 0;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_BATCH;
	shub->shub_req->data.batch.snsr_id = snsr_id;
	shub->shub_req->data.batch.flags = flags;
	if (period < shub->snsrs[snsr_id]->cfg.delay_us_min)
		period = shub->snsrs[snsr_id]->cfg.delay_us_min;
	else if (period > shub->snsrs[snsr_id]->cfg.delay_us_max)
		period = shub->snsrs[snsr_id]->cfg.delay_us_max;
	shub->shub_req->data.batch.period = period;
	shub->shub_req->data.batch.timeout = timeout;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret)
		dev_err(shub->dev,
			"batch ERR: snsr_id: %d period: %u timeout: %u!\n",
			snsr_id, period, timeout);
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_enable(void *client, int snsr_id, int enable)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;
	int ret = 0;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_ENABLE;
	shub->shub_req->data.enable.snsr_id = snsr_id;
	shub->shub_req->data.enable.enable = enable;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret) {
		dev_err(shub->dev,
			"enable ERR: snsr_id: %d enable: %d!\n",
			snsr_id, enable);
	} else {
		ret = shub->shub_resp->data.enable.enable;
	}
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_max_range(void *client, int snsr_id, int max_range)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;
	int ret = 0, i;
	int len;

	if (max_range < 0)
		return -EINVAL;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_RANGE;
	shub->shub_req->data.range.snsr_id = snsr_id;
	shub->shub_req->data.range.setting = max_range;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret) {
		dev_err(shub->dev,
			"range ERR: snsr_id: %d setting: %d!\n",
			snsr_id, max_range);
		goto exit;
	}
	ret = shub->shub_resp->data.range.err;
	switch (ret) {
	case AON_SHUB_NO_ERR:
		memcpy(&shub->snsrs[snsr_id]->cfg.max_range,
				&shub->shub_resp->data.range.max_range,
				sizeof(struct nvs_float));
		memcpy(&shub->snsrs[snsr_id]->cfg.resolution,
				&shub->shub_resp->data.range.resolution,
				sizeof(struct nvs_float));
		/* AXIS sensors need resolution to be put in the scales */
		len = shub->snsrs[snsr_id]->cfg.ch_n_max;
		if (len) {
			for (i = 0; i < len; i++) {
				memcpy(&shub->snsrs[snsr_id]->cfg.scales[i],
					&shub->shub_resp->data.range.resolution,
					sizeof(struct nvs_float));
			}
		}
		break;
	case AON_SHUB_ENODEV:
		/* Invalid snsr_id passed for range setting */
		ret = -ENODEV;
		break;
	case AON_SHUB_EACCES:
		/* can't change the setting on the fly while the device is
		 * active. Disable the device first.
		 */
		ret = -EACCES;
		break;
	case AON_SHUB_EINVAL:
		/* Range setting exceeds max setting */
		ret = -EINVAL;
		break;
	case AON_SHUB_EPERM:
		/* No provision to modify the range for this device */
		ret = -EPERM;
		break;
	default:
		break;
	}

	if (ret)
		dev_err(shub->dev, "range ERR: %d\n", ret);

exit:
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_thresh(struct tegra_aon_shub *shub, int snsr_id,
				 int thresh, bool high)
{
	int ret;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = high ? AON_SHUB_REQUEST_THRESH_HI :
					AON_SHUB_REQUEST_THRESH_LO;
	shub->shub_req->data.thresh.snsr_id = snsr_id;
	shub->shub_req->data.thresh.setting = thresh;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret) {
		dev_err(shub->dev,
			"thresh_%s ERR: snsr_id: %d setting: %d!\n",
			high ? "hi" : "lo", snsr_id, thresh);
		goto exit;
	}
	ret = shub->shub_resp->data.thresh.err;
	switch (ret) {
	case AON_SHUB_NO_ERR:
		if (high)
			shub->snsrs[snsr_id]->cfg.thresh_hi = thresh;
		else
			shub->snsrs[snsr_id]->cfg.thresh_lo = thresh;
		break;
	case AON_SHUB_ENODEV:
		/* Invalid snsr_id passed for threshold setting */
		ret = -ENODEV;
		break;
	case AON_SHUB_EACCES:
		/* can't change the setting on the fly while the device is
		 * active. Disable the device first.
		 */
		ret = -EACCES;
		break;
	case AON_SHUB_EINVAL:
		/* Invalid threshold passed */
		ret = -EINVAL;
		break;
	case AON_SHUB_EPERM:
		/* No provision to modify the thresh for this device */
		ret = -EPERM;
		break;
	default:
		break;
	}

	if (ret)
		dev_err(shub->dev, "thresh_%s ERR: %d\n",
					high ? "hi" : "lo", ret);

exit:
	mutex_unlock(&shub->shub_mutex);
	return ret;
}

static int tegra_aon_shub_thresh_lo(void *client, int snsr_id, int thresh_lo)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;

	return tegra_aon_shub_thresh(shub, snsr_id, thresh_lo, false);
}

static int tegra_aon_shub_thresh_hi(void *client, int snsr_id, int thresh_hi)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;

	return tegra_aon_shub_thresh(shub, snsr_id, thresh_hi, true);
}

static struct nvs_fn_dev aon_shub_nvs_fn = {
	.enable	= tegra_aon_shub_enable,
	.batch	= tegra_aon_shub_batch,
	.batch_read = tegra_aon_shub_batch_read,
	.max_range = tegra_aon_shub_max_range,
	.thresh_lo = tegra_aon_shub_thresh_lo,
	.thresh_hi = tegra_aon_shub_thresh_hi,
};

#ifdef TEGRA_AON_SHUB_DBG_ENABLE
static void tegra_aon_shub_dbg_cfg(struct device *dev, struct sensor_cfg *cfg)
{
	unsigned int i;

	dev_dbg(dev, "name=%s\n", cfg->name);
	dev_dbg(dev, "name=%s\n", cfg->part);
	dev_dbg(dev, "name=%s\n", cfg->vendor);
	dev_dbg(dev, "snsr_id=%d\n", cfg->snsr_id);
	dev_dbg(dev, "timestamp_sz=%d\n", cfg->timestamp_sz);
	dev_dbg(dev, "snsr_data_n=%d\n", cfg->snsr_data_n);
	dev_dbg(dev, "kbuf_sz=%d\n", cfg->kbuf_sz);
	dev_dbg(dev, "ch_n=%u\n", cfg->ch_n);
	dev_dbg(dev, "ch_n_max=%u\n", cfg->ch_n_max);
	dev_dbg(dev, "ch_sz=%d\n", cfg->ch_sz);
	dev_dbg(dev, "delay_us_min=%u\n", cfg->delay_us_min);
	dev_dbg(dev, "delay_us_max=%u\n", cfg->delay_us_max);
	dev_dbg(dev, "matrix: ");
	for (i = 0; i < 9; i++)
		dev_dbg(dev, "%hhd ", cfg->matrix[i]);
	dev_dbg(dev, "\nScales:\n");
	for (i = 0; i < 3; i++) {
		dev_dbg(dev, " %d : ival: %u\n", i, cfg->scales[i].ival);
		dev_dbg(dev, " %d : fval: %u\n", i, cfg->scales[i].fval);
	}
	dev_dbg(dev, "maxrange:\n");
	dev_dbg(dev, " ival: %u ", cfg->max_range.ival);
	dev_dbg(dev, " fval: %u\n", cfg->max_range.fval);
	dev_dbg(dev, "resolution:\n");
	dev_dbg(dev, " ival: %u ", cfg->resolution.ival);
	dev_dbg(dev, " fval: %u\n", cfg->resolution.fval);
	dev_dbg(dev, "milliamp:\n");
	dev_dbg(dev, " ival: %u ", cfg->milliamp.ival);
	dev_dbg(dev, " fval: %u\n", cfg->milliamp.fval);
	dev_dbg(dev, "uncal_lo=%d\n", cfg->uncal_lo);
	dev_dbg(dev, "uncal_hi=%d\n", cfg->uncal_hi);
	dev_dbg(dev, "cal_lo=%d\n", cfg->cal_lo);
	dev_dbg(dev, "cal_hi=%d\n", cfg->cal_hi);
	dev_dbg(dev, "thresh_lo=%d\n", cfg->thresh_lo);
	dev_dbg(dev, "thresh_hi=%d\n", cfg->thresh_hi);
	dev_dbg(dev, "report_n=%d\n", cfg->report_n);
	dev_dbg(dev, "float_significance=%s\n",
			nvs_float_significances[cfg->float_significance]);
}
#endif

static void tegra_aon_shub_copy_cfg(struct tegra_aon_shub *shub,
				    struct aon_shub_sensor *sensor)
{
	int len;
	int i;

	/* TODO: Should we rearrange sensor_cfg in nvs.h so that we
	 * can call memcpy() rather than individual field assignment.
	 */
	len = ARRAY_SIZE(sensor->name);
	strncpy(sensor->name,
		(char *)shub->shub_resp->data.cfg.name,
		len);
	sensor->name[len - 1] = '\0';
	len = ARRAY_SIZE(sensor->part);
	strncpy(sensor->part,
		(char *)shub->shub_resp->data.cfg.part,
		len);
	sensor->part[len - 1] = '\0';
	len = ARRAY_SIZE(sensor->vendor);
	strncpy(sensor->vendor,
		(char *)shub->shub_resp->data.cfg.vendor,
		len);
	sensor->vendor[len - 1] = '\0';
	sensor->type = get_snsr_type(sensor->name);
	sensor->cfg.name = sensor->name;
	sensor->cfg.part = sensor->part;
	sensor->cfg.vendor = sensor->vendor;
	sensor->cfg.version = shub->shub_resp->data.cfg.version;
	sensor->cfg.snsr_id = shub->shub_resp->data.cfg.snsr_id;
	sensor->cfg.kbuf_sz = shub->shub_resp->data.cfg.kbuf_sz;
	sensor->cfg.timestamp_sz =
			shub->shub_resp->data.cfg.timestamp_sz;
	sensor->cfg.snsr_data_n =
			shub->shub_resp->data.cfg.snsr_data_n;
	sensor->cfg.ch_n = shub->shub_resp->data.cfg.ch_n;
	sensor->cfg.ch_n_max = shub->shub_resp->data.cfg.ch_n_max;
	sensor->cfg.ch_sz = shub->shub_resp->data.cfg.ch_sz;
	sensor->cfg.max_range.ival =
			shub->shub_resp->data.cfg.max_range.ival;
	sensor->cfg.max_range.fval =
			shub->shub_resp->data.cfg.max_range.fval;
	sensor->cfg.resolution.ival =
			shub->shub_resp->data.cfg.resolution.ival;
	sensor->cfg.resolution.fval =
			shub->shub_resp->data.cfg.resolution.fval;
	sensor->cfg.milliamp.ival =
			shub->shub_resp->data.cfg.milliamp.ival;
	sensor->cfg.milliamp.fval =
			shub->shub_resp->data.cfg.milliamp.fval;
	sensor->cfg.delay_us_min =
			shub->shub_resp->data.cfg.delay_us_min;
	sensor->cfg.delay_us_max =
			shub->shub_resp->data.cfg.delay_us_max;
	sensor->cfg.fifo_rsrv_evnt_cnt =
			shub->shub_resp->data.cfg.fifo_rsrv_evnt_cnt;
	sensor->cfg.fifo_max_evnt_cnt =
			shub->shub_resp->data.cfg.fifo_max_evnt_cnt;
	sensor->cfg.flags = shub->shub_resp->data.cfg.flags;
	sensor->cfg.uncal_lo = shub->shub_resp->data.cfg.uncal_lo;
	sensor->cfg.uncal_hi = shub->shub_resp->data.cfg.uncal_hi;
	sensor->cfg.cal_lo = shub->shub_resp->data.cfg.cal_lo;
	sensor->cfg.cal_hi = shub->shub_resp->data.cfg.cal_hi;
	sensor->cfg.thresh_lo =
			shub->shub_resp->data.cfg.thresh_lo;
	sensor->cfg.thresh_hi =
			shub->shub_resp->data.cfg.thresh_hi;
	sensor->cfg.float_significance =
			shub->shub_resp->data.cfg.float_significance;
	sensor->cfg.scale.ival =
			shub->shub_resp->data.cfg.scale.ival;
	sensor->cfg.scale.fval =
			shub->shub_resp->data.cfg.scale.fval;
	sensor->cfg.offset.ival =
			shub->shub_resp->data.cfg.offset.ival;
	sensor->cfg.offset.fval =
			shub->shub_resp->data.cfg.offset.fval;
	for (i = 0; i < 3; i++) {
		sensor->cfg.scales[i].ival =
			shub->shub_resp->data.cfg.resolution.ival;
		sensor->cfg.scales[i].fval =
			shub->shub_resp->data.cfg.resolution.fval;
	}
	for (i = 0; i < 9; i++) {
		sensor->cfg.matrix[i] =
				shub->shub_resp->data.cfg.matrix[i];
	}
}

static int tegra_aon_shub_get_cfg(struct tegra_aon_shub *shub, int remote_id)
{
	int ret;
	struct aon_shub_sensor *sensor;

	if (remote_id >= shub->snsr_cnt) {
		dev_err(shub->dev,
			"Invalid sensor config index : %d\n", remote_id);
		return -EINVAL;
	}

	shub->shub_req->req_type = AON_SHUB_REQUEST_SNSR_CFG;
	shub->shub_req->data.cfg.index = remote_id;
	ret = tegra_aon_shub_ivc_msg_send(shub,
				sizeof(struct aon_shub_request),
				IVC_TIMEOUT);
	if (ret) {
		dev_err(shub->dev, " %s No response from AON SHUB..!\n",
			__func__);
		return ret;
	}

	sensor = devm_kzalloc(shub->dev, sizeof(struct aon_shub_sensor),
				GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	shub->snsrs[remote_id] = sensor;
	tegra_aon_shub_copy_cfg(shub, sensor);
#ifdef TEGRA_AON_SHUB_DBG_ENABLE
	tegra_aon_shub_dbg_cfg(shub->dev, &sensor->cfg);
#endif

	return ret;
}

static inline bool is_chipid_valid(struct tegra_aon_shub *shub,
					 u8 chip_id,
					 const char *name)
{
	int i;

	for (i = 0; i < shub->nchips; i++)
		if (shub->chips[i].chip_id == chip_id)
			return !strcmp(name, shub->chips[i].name);

	return false;
}

static inline bool is_i2cid_valid(u8 i2c_id)
{
	return (i2c_id >= I2CID_MIN && i2c_id <= I2CID_MAX);
}

static inline bool is_gpio_ctlr_id_valid(u8 ctlr_id)
{
	return (ctlr_id >= GPIO_CHIP_ID_MIN && ctlr_id <= GPIO_CHIP_ID_MAX);
}

static int tegra_aon_shub_setup(struct tegra_aon_shub *shub,
				struct device_node *np)
{
	int ret;
	struct device_node *cn;
	struct device *dev;
	struct aon_shub_init_setup_request *setup_req;
	u32 chip_id, gpio_ctlr_id;
	s32 rst_gpio;
	u32 i2c_info[3];
	u32 gpios[2];
	int ngpios;
	u32 aux_chip_info[2];
	bool aux_slave = false;
	const char *aux_chip_name;
	int i = 0;

	dev = shub->dev;

	for_each_child_of_node(np, cn) {
		ret = of_property_read_u32(cn, "chip_id", &chip_id);
		if (ret) {
			dev_err(dev, "missing <%s> property\n", "chip_id");
			return ret;
		}

		if (!is_chipid_valid(shub, chip_id, cn->name)) {
			dev_err(dev, "chip_id %d : chip_name %s mismatch\n",
				(int)chip_id, cn->name);
			return -EINVAL;
		}

		ret = of_property_read_u32(cn, "gpio_ctlr_id", &gpio_ctlr_id);
		if (ret) {
			dev_err(dev, "missing <%s> property\n", "gpio_ctlr_id");
			return ret;
		}

		if (!is_gpio_ctlr_id_valid(gpio_ctlr_id)) {
			dev_err(dev, "Invalid gpio_ctlr_id: %d\n",
				(int)gpio_ctlr_id);
			return -EINVAL;
		}

		/*
		 * If reset GPIO is not present, its not an error.
		 */
		ret = of_property_read_u32(cn, "reset_gpio", &rst_gpio);
		if (ret)
			rst_gpio = -1;

		ngpios = of_property_count_u32_elems(cn, "gpios");
		if (ngpios < 0) {
			dev_err(dev, "missing <%s> property\n", "gpios");
			return ngpios;
		}

		if (ngpios > ARRAY_SIZE(gpios)) {
			dev_err(dev, "Invalid element count of <%s> property\n",
				"gpio");
			return -EINVAL;
		}

		ret = of_property_read_u32_array(cn, "gpios", gpios, ngpios);
		if (ret) {
			dev_err(dev, "<%s> property parsing failed : %d\n",
				"gpio", ret);
			return ret;
		}

		ret = of_property_read_u32_array(cn, "i2c_info", i2c_info, 3);
		if (ret) {
			dev_err(dev, "missing <%s> property\n", "i2c_info");
			return ret;
		}

		if (!is_i2cid_valid(i2c_info[0])) {
			dev_err(dev, "Invalid I2C controller id\n");
			return -EINVAL;
		}

		/*
		 * If the aux_chip property is not present, then it is not
		 * an error as it is not necessary to have an aux slave
		 * device. However, if aux_chip property is present and the
		 * aux_chip_name property is missing, then it is an error.
		 */
		ret = of_property_read_u32_array(cn, "aux_chip",
						 aux_chip_info, 2);
		if (ret >= 0) {
			ret = of_property_read_string(cn, "aux_chip_name",
							&aux_chip_name);
			if (ret < 0) {
				dev_err(dev, "missing %s property\n",
							"aux_chip_name");
				return ret;
			}
			if (!is_chipid_valid(shub, aux_chip_info[0],
							aux_chip_name)) {
				dev_err(dev,
					"aux chip_id : chip_name mismatch\n");
				return -EINVAL;
			}
			aux_slave = true;
		} else {
			/*
			 * aux_chip_info[1] holds the I2C address of the slave
			 * chip. If the aux_chip_id is < 0, we don't care about
			 * its I2C address. Only when the chip_id is validated
			 * the I2C addresses are validated on the remote side.
			 */
			aux_chip_info[0] = -1;
		}

		mutex_lock(&shub->shub_mutex);
		shub->shub_req->req_type = AON_SHUB_REQUEST_INIT;
		shub->shub_req->data.init.req = AON_SHUB_INIT_REQUEST_SETUP;
		setup_req = &shub->shub_req->data.init.data.setup;
		for (i = 0; i < ngpios; i++)
			setup_req->gpios[i] = gpios[i];
		setup_req->ngpios = ngpios;
		setup_req->reset_gpio = rst_gpio;
		setup_req->gpio_ctlr_id = gpio_ctlr_id;
		setup_req->chip_id = chip_id;
		setup_req->i2c_id = i2c_info[0];
		setup_req->i2c_addr = i2c_info[2];
		setup_req->slave_chip_id = aux_chip_info[0];
		setup_req->slave_i2c_addr = aux_slave ? aux_chip_info[1] : 0;
		ret = tegra_aon_shub_ivc_msg_send(shub,
					sizeof(struct aon_shub_request),
					IVC_TIMEOUT);
		if (ret) {
			dev_err(shub->dev, "No response from AON SHUB...!\n");
			goto err_exit;
		}
		if (shub->shub_resp->data.init.init_type !=
						AON_SHUB_INIT_REQUEST_SETUP) {
			ret = -EIO;
			dev_err(shub->dev,
				"Invalid response to INIT_SETUP request\n");
			goto err_exit;
		}

		ret = shub->shub_resp->data.init.status;
		if (ret) {
			dev_err(shub->dev, "%s setup failed ERR: %d\n",
				cn->name, ret);
			goto err_exit;
		}

		if (i >= shub->nchips) {
			dev_err(shub->dev, "Invalid num of chip_nodes: %d\n",
				i);
			ret = -EINVAL;
			goto err_exit;
		}
		shub->chip_nodes[i].dn = cn;
		shub->chip_nodes[i].chip_id = chip_id;
		i++;
		if (i >= shub->nchips) {
			dev_err(shub->dev, "Invalid num of chip_nodes: %d\n",
				i);
			ret = -EINVAL;
			goto err_exit;
		}
		if (aux_slave) {
			shub->chip_nodes[i].dn = cn;
			shub->chip_nodes[i].chip_id = aux_chip_info[0];
			i++;
		}

		if (shub->i2c_clk_rates[i2c_info[0] - I2CID_MIN] < i2c_info[1])
			shub->i2c_clk_rates[i2c_info[0] - I2CID_MIN] =
								i2c_info[1];
		shub->chip_id_mask |= BIT(chip_id - 1);
		aux_slave = false;
		mutex_unlock(&shub->shub_mutex);
	}

	return 0;

err_exit:
	mutex_unlock(&shub->shub_mutex);
	return ret;
}

static inline int tegra_aon_shub_count_sensor_chips(struct device_node *dn)
{
	return of_get_child_count(dn);
}

static int tegra_aon_shub_get_snsr_chips(struct tegra_aon_shub *shub)
{
	int ret;
	struct aon_shub_snsr_chips_response *resp;

	shub->shub_req->req_type = AON_SHUB_REQUEST_SNSR_CHIPS;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret) {
		dev_err(shub->dev, "No response from AON SHUB...!\n");
		return ret;
	}

	resp = &shub->shub_resp->data.snsr_chips;
	shub->nchips = resp->nchips;
	if (shub->nchips > AON_SHUB_MAX_CHIPS) {
		dev_err(shub->dev, "Invalid shub->nchips..!\n");
		return -EINVAL;
	}
	shub->chips = devm_kzalloc(shub->dev,
				sizeof(struct snsr_chip) * resp->nchips,
				GFP_KERNEL);
	shub->chip_nodes = devm_kzalloc(shub->dev,
				sizeof(struct shub_chip_node) * resp->nchips,
				GFP_KERNEL);
	if (!shub->chips || !shub->chip_nodes)
		return -ENOMEM;

	memcpy(shub->chips, resp->chips,
			sizeof(struct snsr_chip) * resp->nchips);

	return ret;
}

static int tegra_aon_shub_get_snsr_cnt(struct tegra_aon_shub *shub)
{
	int ret = 0;

	shub->shub_req->req_type = AON_SHUB_REQUEST_SYS;
	shub->shub_req->data.sys.req = AON_SHUB_SYS_REQUEST_SNSR_CNT;
	/*
	 * Work around bug 1986718: The very first HSP interrupt from
	 * SPE to CCPLEX is delayed by several hundred milliseconds for
	 * reasons unknown currently. The following timeout count of
	 * "15" was emperically derived over 120 reboots. The max delay
	 * seen was around 1.5 secs. To account for unexpected conditions,
	 * I am using twice the max currently.
	 */
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  INIT_TOUT_COUNT * IVC_TIMEOUT);
	if (ret) {
		dev_err(shub->dev, "%s No response from AON SHUB...!\n",
			__func__);
		return ret;
	}
	shub->snsr_cnt = shub->shub_resp->data.sys.snsr_cnt;

	return ret;
}

static int tegra_aon_shub_preinit(struct tegra_aon_shub *shub)
{
	int ret = 0;

	mutex_lock(&shub->shub_mutex);
	ret = tegra_aon_shub_get_snsr_cnt(shub);
	if (ret)
		goto err_exit;

	shub->snsrs = devm_kzalloc(shub->dev,
				sizeof(struct aon_shub_sensor *) *
						shub->snsr_cnt,
				GFP_KERNEL);
	if (!shub->snsrs) {
		ret = -ENOMEM;
		goto err_exit;
	}

	ret = tegra_aon_shub_get_snsr_chips(shub);
	if (ret)
		dev_err(shub->dev, "shub_get_snsr_chips failed : %d\n", ret);

err_exit:
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static struct device_node *tegra_aon_shub_find_node_by_chip(
					struct tegra_aon_shub *shub,
					u8 chip_id)
{
	int i;

	for (i = 0; i < shub->nchips; i++)
		if (shub->chip_nodes[i].chip_id == chip_id)
			return shub->chip_nodes[i].dn;

	return NULL;

}

static int tegra_aon_shub_init(struct tegra_aon_shub *shub)
{
	int ret = 0;
	int snsrs = 0;
	int i;
	int count;
	bool i2c_inited = false;
	u32 chip_id_msk;
	u8 chip_id;
	struct aon_shub_init_i2c_request *i2c_req;
	struct aon_shub_init_snsrs_request *snsrs_req;
	struct aon_shub_chip_cfg_ids_response *cfg_ids_resp;
	struct device_node *dn;
	s8 cfg[AON_SHUB_MAX_SNSRS];

	mutex_lock(&shub->shub_mutex);
	for (i = 0; i < ARRAY_SIZE(shub->i2c_clk_rates); i++) {
		if (!shub->i2c_clk_rates[i])
			continue;
		shub->shub_req->req_type = AON_SHUB_REQUEST_INIT;
		shub->shub_req->data.init.req = AON_SHUB_INIT_REQUEST_I2C;
		i2c_req = &shub->shub_req->data.init.data.i2c_init;
		i2c_req->i2c_id = i + 1;
		i2c_req->clk_rate = shub->i2c_clk_rates[i];
		ret = tegra_aon_shub_ivc_msg_send(shub,
					sizeof(struct aon_shub_request),
					IVC_TIMEOUT);
		if (ret) {
			dev_err(shub->dev,
				"%s : No response from AON SHUB...!\n",
				__func__);
			goto err_exit;
		}
		if (shub->shub_resp->data.init.init_type !=
						AON_SHUB_INIT_REQUEST_I2C) {
			ret = -EIO;
			dev_err(shub->dev,
				"Invalid response to I2C init request\n");
			goto err_exit;
		}
		if (shub->shub_resp->data.init.status) {
			dev_err(shub->dev, "I2C init failed\n");
			ret = -1;
			goto err_exit;
		}
		i2c_inited = true;
	}
	if (!i2c_inited) {
		dev_err(shub->dev, "No I2C controllers inited\n");
		ret = -1;
		goto err_exit;
	}
	shub->shub_req->req_type = AON_SHUB_REQUEST_INIT;
	shub->shub_req->data.init.req = AON_SHUB_INIT_REQUEST_SNSR;
	snsrs_req = &shub->shub_req->data.init.data.snsrs_init;
	snsrs_req->chip_id_mask = shub->chip_id_mask;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					sizeof(struct aon_shub_request),
					INIT_TOUT_COUNT * IVC_TIMEOUT);
	if (ret) {
		dev_err(shub->dev, "%s : No response from AON SHUB...!\n",
			__func__);
		goto err_exit;
	}
	if (shub->shub_resp->data.init.init_type !=
					AON_SHUB_INIT_REQUEST_SNSR) {
		ret = -EIO;
		dev_err(shub->dev, "Invalid response to sensor init request\n");
		goto err_exit;
	}
	if (shub->shub_resp->data.init.status) {
		dev_err(shub->dev, "Sensors init failed\n");
		ret = -1;
		goto err_exit;
	}
	chip_id_msk = shub->chip_id_mask;
	while (chip_id_msk) {
		chip_id = __builtin_ctz(chip_id_msk);
		chip_id_msk &= ~BIT(chip_id);
		chip_id++;
		dn = tegra_aon_shub_find_node_by_chip(shub, chip_id);
		if (dn == NULL) {
			dev_err(shub->dev,
				"No device node for chip %d\n", chip_id);
			ret = -EINVAL;
			goto err_exit;
		}
		shub->shub_req->req_type = AON_SHUB_REQUEST_CHIP_CFG_IDS;
		shub->shub_req->data.cfg_ids.chip_id = chip_id;
		ret = tegra_aon_shub_ivc_msg_send(shub,
				sizeof(struct aon_shub_request),
				IVC_TIMEOUT);
		if (ret) {
			dev_err(shub->dev,
				"chip_cfg_ids : No response from AON SHUB!\n");
			goto err_exit;
		}
		cfg_ids_resp = &shub->shub_resp->data.cfg_ids;
		count = cfg_ids_resp->num_snsrs;
		if (count > AON_SHUB_MAX_SNSRS) {
			dev_err(shub->dev,
				"Invalid number of sensors supported..!\n");
			goto err_exit;
		}
		for (i = 0; i < count; i++)
			cfg[i] = cfg_ids_resp->ids[i];
		for (i = 0; i < count; i++) {
			if (cfg[i] == -1)
				continue;
			ret = tegra_aon_shub_get_cfg(shub, cfg[i]);
			if (ret) {
				dev_err(shub->dev,
					"shub_get_cfg() failed for id : %d\n",
					cfg[i]);
				goto err_exit;
			}
			nvs_of_dt(dn, &shub->snsrs[cfg[i]]->cfg, NULL);
			shub->snsrs[cfg[i]]->genable = true;
		}
	}
	mutex_unlock(&shub->shub_mutex);

	shub->nvs = nvs_iio();
	if (shub->nvs == NULL)
		return -ENODEV;

	for (i = 0; i < shub->snsr_cnt; i++) {
		if (shub->snsrs[i] == NULL || !shub->snsrs[i]->genable)
			continue;
		ret = shub->nvs->probe(&shub->snsrs[i]->nvs_st, (void *)shub,
					shub->dev,  &aon_shub_nvs_fn,
					&shub->snsrs[i]->cfg);
		if (!ret) {
			shub->active_snsr_msk |= BIT(i);
			snsrs++;
		}
	}
	if (!snsrs) {
		dev_err(shub->dev, "nvs_probe() failed..!\n");
		return -ENODEV;
	}

	return 0;

err_exit:
	mutex_unlock(&shub->shub_mutex);
	return ret;
}

static int tegra_aon_shub_probe(struct platform_device *pdev)
{
	struct tegra_aon_shub *shub;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int num_sensors;

	dev_dbg(dev, "AON SHUB driver probe()\n");

	if (!np) {
		dev_err(dev, "tegra_aon_shub: DT data required\n");
		return -EINVAL;
	}

	shub = devm_kzalloc(&pdev->dev, sizeof(*shub), GFP_KERNEL);
	if (!shub)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, shub);
	shub->dev = &pdev->dev;
	shub->cl.dev = &pdev->dev;
	shub->cl.tx_block = true;
	shub->cl.tx_tout = TX_BLOCK_PERIOD;
	shub->cl.knows_txdone = false;
	shub->cl.rx_callback = tegra_aon_shub_mbox_rcv_msg;
	shub->cl.tx_done = tegra_aon_shub_mbox_tx_done;
	shub->mbox = mbox_request_channel(&shub->cl, 0);
	if (IS_ERR(shub->mbox)) {
		ret = PTR_ERR(shub->mbox);
		if (ret != -EPROBE_DEFER)
			dev_warn(&pdev->dev, "can't get mailbox chan (%d)\n",
				 (int)PTR_ERR(shub->mbox));
		return ret;
	}
	dev_dbg(dev, "shub->mbox = %p\n", shub->mbox);

	shub->shub_req = devm_kzalloc(&pdev->dev, sizeof(*shub->shub_req),
					GFP_KERNEL);
	if (!shub->shub_req) {
		ret = -ENOMEM;
		goto exit_free_mbox;
	}

	shub->shub_resp = devm_kzalloc(&pdev->dev, sizeof(*shub->shub_resp),
					GFP_KERNEL);
	if (!shub->shub_resp) {
		ret = -ENOMEM;
		goto exit_free_mbox;
	}

	shub->wait_on = devm_kzalloc(&pdev->dev,
				sizeof(struct completion), GFP_KERNEL);
	if (!shub->wait_on) {
		ret = -ENOMEM;
		goto exit_free_mbox;
	}
	init_completion(shub->wait_on);
	mutex_init(&shub->shub_mutex);

	num_sensors = tegra_aon_shub_count_sensor_chips(np);
	if (num_sensors <= 0) {
		dev_err(dev, "No sensors on the shub\n");
		ret = -EINVAL;
		goto exit_free_mbox;
	}

	ret = tegra_aon_shub_preinit(shub);
	if (ret) {
		dev_err(dev, "shub pre-init failed\n");
		goto exit_free_mbox;
	}

	ret = tegra_aon_shub_setup(shub, np);
	if (ret) {
		dev_err(dev, "shub setup failed\n");
		goto exit_free_mbox;
	}

	ret = tegra_aon_shub_init(shub);
	if (ret) {
		dev_err(dev, "shub init failed\n");
		goto exit_free_mbox;
	}

	shub->adjust_ts_counter = 0;
	#define _PICO_SECS (1000000000000ULL)
	shub->ts_res_ns = (_PICO_SECS / (u64)arch_timer_get_cntfrq())/1000;
	#undef _PICO_SECS
	shub->ts_adjustment = get_ts_adjustment(shub->ts_res_ns);

	dev_info(&pdev->dev, "tegra_aon_shub_driver_probe() OK\n");

	return 0;

exit_free_mbox:
	mbox_free_channel(shub->mbox);
	dev_err(&pdev->dev, "tegra_aon_shub_driver_probe() FAILED\n");

	return ret;
}

static int tegra_aon_shub_remove(struct platform_device *pdev)
{
	struct tegra_aon_shub *shub;

	shub  = dev_get_drvdata(&pdev->dev);
	mbox_free_channel(shub->mbox);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_aon_shub_suspend(struct device *dev)
{
	struct tegra_aon_shub *shub = dev_get_drvdata(dev);
	struct aon_shub_pm_request *pm_req;
	int ret = 0;
	u32 en_snsrs, snsr;

	en_snsrs = shub->active_snsr_msk;
	while (en_snsrs) {
		snsr = __builtin_ctz(en_snsrs);
		ret = tegra_aon_shub_enable(shub,
					    shub->snsrs[snsr]->cfg.snsr_id,
					    0);
		if (ret)
			return ret;
		en_snsrs &= ~BIT(snsr);
	}

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_SYS;
	shub->shub_req->data.sys.req = AON_SHUB_SYS_REQUEST_PM;
	pm_req = &shub->shub_req->data.sys.data.pm;
	pm_req->flags = AON_SHUB_PM_REQUEST_SUSPEND;
	pm_req->chip_id_msk = shub->chip_id_mask;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret)
		dev_err(shub->dev, "AON SHUB Suspend ERR: %d\n", ret);
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_resume(struct device *dev)
{
	struct tegra_aon_shub *shub = dev_get_drvdata(dev);
	struct aon_shub_pm_request *pm_req;
	int ret;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_SYS;
	shub->shub_req->data.sys.req = AON_SHUB_SYS_REQUEST_PM;
	pm_req = &shub->shub_req->data.sys.data.pm;
	pm_req->flags = AON_SHUB_PM_REQUEST_RESUME;
	pm_req->chip_id_msk = shub->chip_id_mask;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request),
					  IVC_TIMEOUT);
	if (ret)
		dev_err(shub->dev, "AON SHUB Resume ERR: %d\n", ret);
	mutex_unlock(&shub->shub_mutex);

	return 0;
}

static const struct dev_pm_ops tegra_aon_shub_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_aon_shub_suspend, tegra_aon_shub_resume)
};
#endif

static const struct of_device_id tegra_aon_shub_of_match[] = {
	{
		.compatible = "nvidia,tegra186_aon_shub",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tegra_aon_shub_of_match);

static struct platform_driver tegra_aon_shub_driver = {
	.driver = {
		.name		= "tegra-aon-shub",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(tegra_aon_shub_of_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &tegra_aon_shub_pm_ops,
#endif
	},
	.probe =	tegra_aon_shub_probe,
	.remove =	tegra_aon_shub_remove,
};
module_platform_driver(tegra_aon_shub_driver);

MODULE_DESCRIPTION("NVIDIA Tegra186 AON Sensor Hub Driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
