/*
 * drivers/iio/imu/tsfw_icm20628/tsfw_icm20628.c
 *
 * Copyright (c) 2016, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/errno.h>
#include <linux/nvs.h>
#include <linux/gfp.h>
#include <linux/hid.h>
#include <linux/iio/imu/tsfw_icm20628.h>
#include <linux/module.h>
#include <linux/byteorder/generic.h>

/* #define DEBUG_TSFW_ICM */

#define TSFW_SENSOR_NAME	"tsfw_icm20628"
#define TSFW_SENSOR_VENDOR	"Invensense"
#define TSFW_SENSOR_VERSION	1

#define AXIS_N			3 /* 3 channels of data (X, Y, Z) */
#define SENSOR_REPORT_SYN_MASK	0x80

/* TODO: hacky, declare in .h file of hid */
extern int atvr_ts_sensor_set(struct hid_device *hdev, bool enable);

/* Define sensor */
/* TODO: modify delay_us_min|max, implement batch */
/* TODO: comment delay_us_min|max */
static struct sensor_cfg tsfw_icm20628_cfg_dflt[] = {
	{
		.name			= "accelerometer",
		.snsr_id		= SENSOR_TYPE_ACCELEROMETER,
		.kbuf_sz		= 32,
		.ch_n			= AXIS_N,
		.ch_n_max		= AXIS_N,
		.ch_sz			= -2, /* 2 bytes signed per channel */
		.part			= TSFW_SENSOR_NAME,
		.vendor			= TSFW_SENSOR_VENDOR,
		.version		= TSFW_SENSOR_VERSION,
		.delay_us_min		= 14222,
		.delay_us_max		= 14222,
		.flags			= DYNAMIC_SENSOR_MASK,
		.float_significance	= NVS_FLOAT_NANO,
		.milliamp		= {
			.ival		= 0,
			.fval		= 500000000,
		},
		.max_range		= {
			.ival		= 19,
			.fval		= 613300,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 598550,
		},
		.scales[0]		= {
			.ival		= 0,
			.fval		= 598550,
		},
		.scales[1]		= {
			.ival		= 0,
			.fval		= 598550,
		},
		.scales[2]		= {
			.ival		= 0,
			.fval		= 598550,
		},
		.uuid			= "ts_accel",
		.matrix                 = { 0, -1, 0, 1, 0, 0, 0, 0, 1 },
	},
	{
		.name			= "gyroscope",
		.snsr_id		= SENSOR_TYPE_GYROSCOPE,
		.kbuf_sz		= 32,
		.ch_n			= AXIS_N,
		.ch_n_max		= AXIS_N,
		.ch_sz			= -2, /* 2 bytes signed per channel */
		.part			= TSFW_SENSOR_NAME,
		.vendor			= TSFW_SENSOR_VENDOR,
		.version		= TSFW_SENSOR_VERSION,
		.delay_us_min		= 14222,
		.delay_us_max		= 14222,
		.flags			= DYNAMIC_SENSOR_MASK,
		.float_significance	= NVS_FLOAT_NANO,
		.milliamp		= {
			.ival		= 3,
			.fval		= 700000000,
		},
		.max_range		= {
			.ival		= 34,
			.fval		= 906585040,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 1064225,
		},
		.scales[0]		= {
			.ival		= 0,
			.fval		= 1064225,
		},
		.scales[1]		= {
			.ival		= 0,
			.fval		= 1064225,
		},
		.scales[2]		= {
			.ival		= 0,
			.fval		= 1064225,
		},
		.uuid			= "ts_gyro",
		.matrix                 = { 0, 1, 0, -1, 0, 0, 0, 0, 1 },
	},
	{
		.name			= "magnetic_field",
		.snsr_id		= SENSOR_TYPE_MAGNETIC_FIELD,
		.kbuf_sz		= 32,
		.ch_n			= AXIS_N,
		.ch_n_max		= AXIS_N,
		.ch_sz			= -2, /* 2 bytes signed per channel */
		.part			= TSFW_SENSOR_NAME,
		.vendor			= TSFW_SENSOR_VENDOR,
		.version		= TSFW_SENSOR_VERSION,
		.delay_us_min		= 14222,
		.delay_us_max		= 14222,
		.flags			= DYNAMIC_SENSOR_MASK,
		.float_significance	= NVS_FLOAT_NANO,
		.milliamp		= {
			.ival		= 2,
			.fval		= 800000,
		},
		.max_range		= {
			.ival		= 4912,
			.fval		= 0,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 600000,
		},
		.scales[0]		= {
			.ival		= 0,
			.fval		= 600000,
		},
		.scales[1]		= {
			.ival		= 0,
			.fval		= 600000,
		},
		.scales[2]		= {
			.ival		= 0,
			.fval		= 600000,
		},
		.uuid			= "ts_mag",
		.matrix                 = { -1, 0, 0, 0, -1, 0, 0, 0, 1 },
	}
};

#ifdef DEBUG_TSFW_ICM
#define pr_debugg(...) pr_info(__VA_ARGS__)
/* TODO: remove magic numbers */
static void pr_debugg_buffer(u8 *data, size_t len)
{
	char qwerty[136] = { 0 };
	size_t i, pos = 0;
	if (len > 43)
		len = 43;
	for (i = 0; i < len; i++) {
		if (i != 0 && i % 8 == 0)
			pos += sprintf(qwerty + pos, "\n");
		pos += sprintf(qwerty + pos, "%02x ", data[i]);
	}
	pos += sprintf(qwerty + pos, "\n");
	pr_info("%s", qwerty);
}
#else
#define pr_debugg(...) do {} while (0)
static inline void pr_debugg_buffer(u8 *data, size_t len)
{
}
#endif

/* Different types of fragmentation
 * report_id=5, syn=1, 1 full set (18B)
 * report_id=5, syn=1, 1 full set + 1 partial set (23B)
 * report_id=5, syn=0, 1 partial set (1-17B)
 * report_id=5, syn=0, 1 full set (18B) does this exist?
 * report_id=5, syn=0, 1 partial set + 1 full set (19-25B)
 * report_id=5, syn=0, 1 partial set + 1 full set + 1 partial set (20-25B)
 * report_id=6 (=syn=0), ???
 * report_id=7 (=syn=1), 1 full set (18B)
 * report_id=7 (=syn=1), 2 full sets (36B)
 * report_id=7 (=syn=1), 2 full sets + 1 partial set (41B)
 */

/* only accepts full 18B buffers */
static void pass_to_nvs(struct tsfw_icm20628_state *st, u8 *data)
{
	u16 tmp[SNSR_N * AXIS_N];
	size_t i;
	s64 ts;
	ts = nvs_timestamp(); /* ns */
	pr_debugg("Passing sensor data to nvs ts=%lld\n", ts);
	pr_debugg_buffer(data, SENSOR_DATA_SET_SIZE);

	/* TODO: st->nvs->nvs_mutex_lock(st->nvs_st); */
	memcpy(tmp, data, SENSOR_DATA_SET_SIZE);
	/* swap endianness */
	for (i = 0; i < SNSR_N * AXIS_N; i++)
		tmp[i] = ntohs(tmp[i]);

	/* pass to nvs */
	if (st && st->nvs && st->nvs->handler)
		for (i = 0; i < SNSR_N; i++)
			st->nvs->handler(st->snsr[i].nvs_st, tmp+(AXIS_N*i),
					ts);
	/* TODO: st->nvs->nvs_mutex_unlock(st->nvs_st); */
}

static int handle_sensor_data(struct tsfw_icm20628_state *st,
		u8 report_counter, bool syn, u16 header,
		u8 *data, size_t len)
{
	u8 rem, *tmp;
	pr_debugg("%s #%d ln=%zu syn=%d\n", __func__, report_counter, len, syn);
	pr_debugg_buffer(data, len);

	/* handle initial partial data if any */
	if (syn) {
		tmp = data;
	} else {
		/* TODO: handle first half-packet */
		tmp = data + (SENSOR_DATA_SET_SIZE - st->buffered_cnt);
	}

	/* pass full sets of data to nvs */
	while (tmp + SENSOR_DATA_SET_SIZE <= data + len) {
		pass_to_nvs(st, tmp);
		tmp += SENSOR_DATA_SET_SIZE;
	}

	/* copy remainder to buffer */
	rem = data + len - tmp;
	if (rem) {
		pr_debugg("Copying remaining %d bytes to buffer\n", rem);
		pr_debugg_buffer(tmp, rem);
		memcpy(st->buffered_data, tmp, rem);
	}
	st->buffered_cnt = rem;
	/* TODO: st->prev_report_counter = report_counter; */
	return 0;
}

/* Report #1a
 * data[0]: report id = 0x05
 * data[1]: report counter
 * data[2-18]: HID button data
 * data[18] & 0x80: syn flag (= 0)
 * data[19]: length of sensor data
 * data[20-44]: up to 25 bytes of sensor data
 */
#define RPT1A_DATA_OFFSET	20
#define RPT1A_DATA_MAX_LEN	25

/* Report #1b
 * data[0]: report id = 0x05
 * data[1]: report counter
 * data[2-18]: HID button data
 * data[18] & 0x80: syn flag (= 1)
 * data[19]: length of sensor data
 * data[20-21]: header of which sensors enabled
 * data[22-44]: up to 23 bytes of sensor data
 */
#define RPT1B_DATA_OFFSET	22
#define RPT1B_DATA_MAX_LEN	23

/* Report #2
 * data[0]: report id = 0x06 (syn = 0)
 * data[1]: report counter
 * data[2]: length of sensor data
 * data[3-44]: up to 42 bytes of sensor data
 */
#define RPT2_DATA_OFFSET	3
#define RPT2_DATA_MAX_LEN	42

/* Report #3
 * data[0]: report id = 0x07 (syn = 1)
 * data[1]: report counter
 * data[2]: length of sensor data
 * data[3-4]:  header of which sensors enabled
 * data[5-44]: up to 40 bytes of sensor data
 */
#define RPT3_DATA_OFFSET	5
#define RPT3_DATA_MAX_LEN	40

static int recv(struct tsfw_icm20628_state *st, u8 *data, size_t size)
{
	u8 report_id = data[0];
	u8 report_counter = data[1];
	u16 sensor_header = 0;
	u8 *sensor_data;
	u8 len, len_max;
	bool syn;

	if (!st)
		return -ENODEV;

	pr_debugg("%s received %zu bytes\n", __func__, size);
	pr_debugg_buffer(data, size);

	if (report_id == SENSOR_REPORT_ID_COMBINED) {
		syn = data[18] & SENSOR_REPORT_SYN_MASK;
		len = data[19];
		if (!syn) {
			pr_debugg("Detected sensor packet #1a\n");
			sensor_data = data + RPT1A_DATA_OFFSET;
			len_max = RPT1A_DATA_MAX_LEN;
		} else {
			pr_debugg("Detected sensor packet #1b\n");
			sensor_header = (data[22] << 8) | data[21];
			sensor_data = data + RPT1B_DATA_OFFSET;
			len_max = RPT1B_DATA_MAX_LEN;
		}
	} else if (report_id == SENSOR_REPORT_ID) {
		pr_debugg("Detected sensor packet #2\n");
		syn = false;
		sensor_data = data + RPT2_DATA_OFFSET;
		len = data[2];
		len_max = RPT2_DATA_MAX_LEN;
	} else if (report_id == SENSOR_REPORT_ID_SYN) {
		pr_debugg("Detected sensor packet #3\n");
		syn = true;
		sensor_header = (data[4] << 8) | data[3];
		sensor_data = data + RPT3_DATA_OFFSET;
		len = data[2];
		len_max = RPT3_DATA_MAX_LEN;
	} else {
		pr_err("unexpected sensor report data\n");
		return -EINVAL;
	}

	if (len > len_max) {
		pr_err("invalid len %d sensor report", len);
		return -EINVAL;
	}

	handle_sensor_data(st, report_counter, syn, sensor_header, sensor_data,
			len);
	/* TODO: return error code if fail */
	return 0;
}

static int of_dt(struct tsfw_icm20628_state *st)
{
	size_t i;
	for (i = 0; i < ARRAY_SIZE(tsfw_icm20628_cfg_dflt); i++)
		memcpy(&st->snsr[i].cfg, &tsfw_icm20628_cfg_dflt[i],
				sizeof(st->snsr[i].cfg));

	return 0;

}

static int enable_hw(struct tsfw_icm20628_state *st)
{
	int ret;
	ret = atvr_ts_sensor_set(st->hdev, true);
	if (ret)
		pr_err("%s failed to enable ts sensors\n", __func__);
	return 0;
}

static int disable_hw(struct tsfw_icm20628_state *st)
{
	atvr_ts_sensor_set(st->hdev, false);
	/* TODO: ret check */
	return 0;
}

/* TODO: needed?
static int set_rate_hw(struct tsfw_icm20628_state *st, unsigned int period)
{ return 0; }
 */

static int tsfw_icm_enable(void *client, int snsr_id, int enable)
{
	struct tsfw_icm20628_state *st = (struct tsfw_icm20628_state *)client;
	int ret;

	pr_info("%s snsr_id: %d enable: %d\n", __func__, snsr_id, enable);
	if (enable < 0)
		return st->enabled;

	/* TODO: make use of snsr_id for enabling/disabling specific sensors
	 * snsr_id will be SENSOR_TYPE_* from nvs.h
	 */
	if (enable)
		ret = enable_hw(st);
	else
		ret = disable_hw(st);
	if (!ret)
		st->enabled = enable;
	return ret;
}

static int tsfw_icm_batch(void *client, int snsr_id, int flags,
		unsigned int period, unsigned int timeout)
{
	pr_info("%s\n", __func__);
	/* TODO: remove?
	struct tsfw_icm20628_state *st = (struct tsfw_icm20628_state *)client;
	return set_rate_hw(st, period);
	*/
	return 0;
}

static struct nvs_fn_dev tsfw_icm20628_fn_dev = {
	.enable	= tsfw_icm_enable,
	.batch	= tsfw_icm_batch,
};

static int probe(struct hid_device *hdev, struct tsfw_icm20628_state **pst)
{
	struct tsfw_icm20628_state *st;
	int ret;
	size_t i;

	pr_info("tsfw_icm %s\n", __func__);
	/* TODO: use devm_kzalloc? */
	*pst = kzalloc(sizeof(struct tsfw_icm20628_state), GFP_KERNEL);
	st = *pst;
	if (!st)
		return -ENOMEM;

	ret = of_dt(st);
	if (ret) {
		pr_err("failed to init of_dt\n");
		goto probe_failed;
	}

	st->hdev = hdev;
	tsfw_icm20628_fn_dev.errs = &st->errs;
	tsfw_icm20628_fn_dev.sts = &st->sts;
	st->nvs = nvs_iio();
	if (!st->nvs || !st->nvs->probe) {
		pr_err("failed to get nvs_iio\n");
		ret = -ENODEV;
		goto probe_failed;
	}

	for (i = 0; i < SNSR_N; i++) {
		ret = st->nvs->probe(&st->snsr[i].nvs_st, st, &hdev->dev,
				&tsfw_icm20628_fn_dev, &st->snsr[i].cfg);
		if (ret)
			goto nvs_probe_failed;
	}

	return 0;


nvs_probe_failed:
	if (st && st->nvs && st->nvs->remove)
		for (i = 0; i < SNSR_N; i++)
			st->nvs->remove(st->snsr[i].nvs_st);
probe_failed:
	kfree(st);
	return ret;
}

static int remove(struct tsfw_icm20628_state *st)
{
	size_t i;
	if (st && st->nvs && st->nvs->remove)
		for (i = 0; i < SNSR_N; i++)
			st->nvs->remove(st->snsr[i].nvs_st);

	kfree(st);
	return 0;
}

static struct tsfw_icm20628_fn_dev fns = {
	.probe	= probe,
	.recv	= recv,
	.remove	= remove,
};

struct tsfw_icm20628_fn_dev *tsfw_icm20628_fns()
{
	return &fns;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Thunderstrike FW Accel/Gyro/Mag sensors");
MODULE_AUTHOR("NVIDIA Corporation");
