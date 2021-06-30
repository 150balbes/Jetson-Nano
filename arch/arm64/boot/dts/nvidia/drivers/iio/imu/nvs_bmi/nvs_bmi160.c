/* Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

/* NVS = NVidia Sensor framework */
/* Uses one of the NVS kernel modules: nvs_iio, nvs_input, nvs_relay, nvs_nv */
/* See nvs.h for documentation */

/* This driver operates in one of two modes depending on if an interrupt is
 * defined:
 * 1. Without an interrupt the sensors are all set at the fastest enabled
 *    rate and polled at that rate using the register map.
 * 2. An interrupt defined allows the driver to use the FIFO and its
 *    features, e.g. support for independent ODRs.
 */


#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/nvs.h>
#include <linux/device.h>
#include <linux/version.h>

#define BMI_NAME			"bmi160"
#define BMI_VENDOR			"Bosch"
#define BMI_DRIVER_VERSION		(13)
#define BMI_ACC_VERSION			(1)
#define BMI_GYR_VERSION			(1)
#define BMI_HW_DELAY_POR_MS		(10)
#define BMI_HW_DELAY_DEV_ON_US		(2)
#define BMI_HW_DELAY_DEV_OFF_US		(450)
#define BMI_CMD_DELAY_MS		(2)
#define BMI_I2C_WR_RD_RETRY_N		(5)
/* HW registers */
#define BMI_REG_CHIP_ID			(0x00)
#define BMI_REG_CHIP_ID_POR		(0xD1)
#define BMI_REG_ERR_REG			(0x02)
#define BMI_REG_ERR_REG_drop_cmd_err	(6)
#define BMI_REG_PMU_STATUS		(0x03)
#define BMI_REG_PMU_STATUS_MSK_ACC	(0x30)
#define BMI_REG_PMU_STATUS_MSK_GYR	(0x0C)
#define BMI_REG_PMU_STATUS_MSK_MAG	(0x03)
#define BMI_REG_PMU_STATUS_ACC_SUSP	(0 << 4)
#define BMI_REG_PMU_STATUS_ACC_NORM	(1 << 4)
#define BMI_REG_PMU_STATUS_ACC_LP	(2 << 4)
#define BMI_REG_PMU_STATUS_GYR_SUSP	(0 << 2)
#define BMI_REG_PMU_STATUS_GYR_NORM	(1 << 2)
#define BMI_REG_PMU_STATUS_GYR_FSU	(3 << 2)
#define BMI_REG_PMU_STATUS_MAG_SUSP	(0)
#define BMI_REG_PMU_STATUS_MAG_NORM	(1)
#define BMI_REG_PMU_STATUS_MAG_LP	(2)
#define BMI_REG_DATA_0			(0x04)
#define BMI_REG_DATA_1			(0x05)
#define BMI_REG_DATA_2			(0x06)
#define BMI_REG_DATA_3			(0x07)
#define BMI_REG_DATA_4			(0x08)
#define BMI_REG_DATA_5			(0x09)
#define BMI_REG_DATA_6			(0x0A)
#define BMI_REG_DATA_7			(0x0B)
#define BMI_REG_DATA_8			(0x0C)
#define BMI_REG_DATA_9			(0x0D)
#define BMI_REG_DATA_10			(0x0E)
#define BMI_REG_DATA_11			(0x0F)
#define BMI_REG_DATA_12			(0x10)
#define BMI_REG_DATA_13			(0x11)
#define BMI_REG_DATA_14			(0x12)
#define BMI_REG_DATA_15			(0x13)
#define BMI_REG_DATA_16			(0x14)
#define BMI_REG_DATA_17			(0x15)
#define BMI_REG_DATA_18			(0x16)
#define BMI_REG_DATA_19			(0x17)
#define BMI_REG_SENSORTIME_0		(0x18)
#define BMI_REG_SENSORTIME_1		(0x19)
#define BMI_REG_SENSORTIME_2		(0x1A)
#define BMI_REG_STATUS			(0x1B)
#define BMI_REG_INT_STATUS_0		(0x1C)
#define BMI_REG_INT_STATUS_1		(0x1D)
#define BMI_REG_INT_STATUS_2		(0x1E)
#define BMI_REG_INT_STATUS_3		(0x1F)
#define BMI_REG_TEMPERATURE_0		(0x20)
#define BMI_REG_TEMPERATURE_1		(0x21)
#define BMI_REG_FIFO_LENGTH_0		(0x22)
#define BMI_REG_FIFO_LENGTH_1		(0x23)
#define BMI_REG_FIFO_DATA		(0x24)
#define BMI_REG_ACC_CONF		(0x40)
#define BMI_REG_ACC_RANGE		(0x41)
#define BMI_REG_GYR_CONF		(0x42)
#define BMI_REG_GYR_RANGE		(0x43)
#define BMI_REG_MAG_CONF		(0x44)
#define BMI_REG_FIFO_DOWNS		(0x45)
#define BMI_REG_FIFO_CONFIG_0		(0x46)
#define BMI_REG_FIFO_CONFIG_1		(0x47)
#define BMI_REG_FIFO_CONFIG_1_ENS	(6) // FIXME
#define BMI_REG_MAG_IF_0		(0x4B)
#define BMI_REG_MAG_IF_1		(0x4C)
#define BMI_REG_MAG_IF_2		(0x4D)
#define BMI_REG_MAG_IF_3		(0x4E)
#define BMI_REG_MAG_IF_4		(0x4F)
#define BMI_REG_INT_EN_0		(0x50)
#define BMI_REG_INT_EN_1		(0x51)
#define BMI_REG_INT_EN_2		(0x52)
#define BMI_REG_INT_OUT_CTRL		(0x53)
#define BMI_REG_INT_LATCH		(0x54)
#define BMI_REG_INT_MAP_0		(0x55)
#define BMI_REG_INT_MAP_1		(0x56)
#define BMI_REG_INT_MAP_2		(0x57)
#define BMI_REG_INT_DATA_0		(0x58)
#define BMI_REG_INT_DATA_1		(0x59)
#define BMI_REG_INT_LOWHIGH_0		(0x5A)
#define BMI_REG_INT_LOWHIGH_1		(0x5B)
#define BMI_REG_INT_LOWHIGH_2		(0x5C)
#define BMI_REG_INT_LOWHIGH_3		(0x5D)
#define BMI_REG_INT_LOWHIGH_4		(0x5E)
#define BMI_REG_INT_MOTION_0		(0x5F)
#define BMI_REG_INT_MOTION_1		(0x60)
#define BMI_REG_INT_MOTION_2		(0x61)
#define BMI_REG_INT_MOTION_3		(0x62)
#define BMI_REG_INT_TAP_0		(0x63)
#define BMI_REG_INT_TAP_1		(0x64)
#define BMI_REG_INT_ORIENT_0		(0x65)
#define BMI_REG_INT_ORIENT_1		(0x66)
#define BMI_REG_INT_FLAT_0		(0x67)
#define BMI_REG_INT_FLAT_1		(0x68)
#define BMI_REG_FOC_CONF		(0x69)
#define BMI_REG_CONF			(0x6A)
#define BMI_REG_IF_CONF			(0x6B)
#define BMI_REG_PMU_TRIGGER		(0x6C)
#define BMI_REG_SELF_TEST		(0x6D)
#define BMI_SELFTEST_ACC_POS		(0x0D)
#define BMI_SELFTEST_ACC_NEG		(0x09)
#define BMI_SELFTEST_ACC_DELAY_MS	(50)
#define BMI_SELFTEST_ACC_LIMIT		(8192)
#define BMI_SELFTEST_GYR		(0x10)
#define BMI_SELFTEST_GYR_DELAY_MS	(20)
#define BMI_REG_NV_CONF			(0x70)
#define BMI_REG_OFFSET_0		(0x71)
#define BMI_REG_OFFSET_1		(0x72)
#define BMI_REG_OFFSET_2		(0x73)
#define BMI_REG_OFFSET_3		(0x74)
#define BMI_REG_OFFSET_4		(0x75)
#define BMI_REG_OFFSET_5		(0x76)
#define BMI_REG_OFFSET_6		(0x77)
#define BMI_REG_STEP_CNT_0		(0x78)
#define BMI_REG_STEP_CNT_1		(0x79)
#define BMI_REG_STEP_CONF_0		(0x7A)
#define BMI_REG_STEP_CONF_1		(0x7B)

#define BMI_REG_CMD			(0x7E)
#define BMI_REG_CMD_ACC_SUSP		(0x10)
#define BMI_REG_CMD_ACC_NORM		(0x11)
#define BMI_REG_CMD_ACC_LP		(0x12)
#define BMI_REG_CMD_GYR_SUSP		(0x14)
#define BMI_REG_CMD_GYR_NORM		(0x15)
#define BMI_REG_CMD_GYR_FAST		(0x17)
#define BMI_REG_CMD_MAG_SUSP		(0x18)
#define BMI_REG_CMD_MAG_NORM		(0x19)
#define BMI_REG_CMD_MAG_LP		(0x1A)
#define BMI_REG_CMD_FIFO_FLUSH		(0xB0)
#define BMI_REG_CMD_RST_INT		(0xB1)
#define BMI_REG_CMD_RST_SOFT		(0xB6)

#define AXIS_X				(0)
#define AXIS_Y				(1)
#define AXIS_Z				(2)
#define AXIS_N				(3)
#define BMI_STS_SPEW_FIFO		(1 << NVS_STS_EXT_N)
#define BMI_STS_SPEW_ST			(1 << (NVS_STS_EXT_N + 1))
#define BMI_STS_SPEW_TS			(1 << (NVS_STS_EXT_N + 2))
#define BMI_HW_ACC			(0)
#define BMI_HW_GYR			(1)
#define BMI_HW_N			(2)

enum BMI_INF {
	BMI_INF_VER = 0,
	BMI_INF_DBG,
	BMI_INF_SPEW_FIFO,
	BMI_INF_SPEW_ST,
	BMI_INF_SPEW_TS,
	BMI_INF_REG_WR = 0xC6, /* use 0xD0 on cmd line */
	BMI_INF_REG_RD,
};

/* regulator names in order of powering on */
static char *bmi_vregs[] = {
	"vdd",
	"vdd_IO",
};

static unsigned short bmi_i2c_addrs[] = {
	0x68,
	0x69,
};

static struct sensor_cfg bmi_sensor_cfgs[] = {
	{
		.name			= "accelerometer",
		.snsr_id		= BMI_HW_ACC,
		.kbuf_sz		= 64,
		.ch_n			= AXIS_N,
		.ch_sz			= -2,
		.part			= BMI_NAME,
		.vendor			= BMI_VENDOR,
		.version		= BMI_ACC_VERSION,
		.max_range		= {
			.ival		= 0, /* default = +/-2g */
		},
		/* milliamp is dynamic based on delay */
		.milliamp		= {
			.ival		= 0,
			.fval		= 180000000,
		},
		.delay_us_min		= 625,
		.delay_us_max		= 80000,
		/* default matrix to get the attribute */
		.matrix[0]		= 1,
		.matrix[4]		= 1,
		.matrix[8]		= 1,
		.float_significance	= NVS_FLOAT_NANO,
	},
	{
		.name			= "gyroscope",
		.snsr_id		= BMI_HW_GYR,
		.kbuf_sz		= 64,
		.ch_n			= AXIS_N,
		.ch_sz			= -2,
		.part			= BMI_NAME,
		.vendor			= BMI_VENDOR,
		.version		= BMI_GYR_VERSION,
		.max_range		= {
			.ival		= 0, /* default = +/-2000dps */
		},
		.milliamp		= {
			.ival		= 0,
			.fval		= 800000000,
		},
		.delay_us_min		= 312,
		.delay_us_max		= 40000,
		.float_significance	= NVS_FLOAT_NANO,
	},
};

struct bmi_rr {
	struct nvs_float max_range;
	struct nvs_float resolution;
	struct nvs_float scale;
};

static struct bmi_rr bmi_rr_acc[] = {
	/* all accelerometer values are in g's fval = NVS_FLOAT_NANO */
	{
		.max_range		= {
			.ival		= 19,
			.fval		= 613300000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 598550,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 598550,
		},
	},
	{
		.max_range		= {
			.ival		= 39,
			.fval		= 226600000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 1197101,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 1197101,
		},
	},
	{
		.max_range		= {
			.ival		= 78,
			.fval		= 453200000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 2394202,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 2394202,
		},
	},
	{
		.max_range		= {
			.ival		= 78,
			.fval		= 453200000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 2394202,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 2394202,
		},
	},
};

static struct bmi_rr bmi_rr_gyr[] = {
	/* rad / sec  fval = NVS_FLOAT_NANO */
	{
		.max_range		= {
			.ival		= 34,
			.fval		= 906585040,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 1064225,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 1064225,
		},
	},
	{
		.max_range		= {
			.ival		= 17,
			.fval		= 453292520,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 532113,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 532113,
		},
	},
	{
		.max_range		= {
			.ival		= 8,
			.fval		= 726646260,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 266462,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 266462,
		},
	},
	{
		.max_range		= {
			.ival		= 4,
			.fval		= 363323130,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 133231,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 133231,
		},
	},
	{
		.max_range		= {
			.ival		= 4,
			.fval		= 363323130,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 133231,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 133231,
		},
	},
	{
		.max_range		= {
			.ival		= 2,
			.fval		= 181661565,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 066615,
		},
		.scale			= {
			.ival		= 0,
			.fval		= 066615,
		},
	},
};

struct bmi_cmd {
	u8 cmd;
	unsigned int ms_t;
};

static struct bmi_cmd bmi_cmd_rst_int = {
	.cmd				= BMI_REG_CMD_RST_INT,
	.ms_t				= 20,
};

static struct bmi_cmd bmi_cmd_rst_soft = {
	.cmd				= BMI_REG_CMD_RST_SOFT,
	.ms_t				= 20,
};

static struct bmi_cmd bmi_cmd_fifo_clr = {
	.cmd				= BMI_REG_CMD_FIFO_FLUSH,
	.ms_t				= 0,
};

struct bmi_state;

struct bmi_hw {
	struct bmi_cmd dis;
	struct bmi_cmd en;
	struct bmi_rr *rr;
	unsigned int rr_0n;
	unsigned int buf_i;
	unsigned int fifo_push_n;
	u8 fifo_hdr_mask;
	int (*fn_enable)(struct bmi_state *st);
	int (*fn_batch)(struct bmi_state *st, unsigned int period_us);
	int (*fn_selftest)(struct bmi_state *st);
};

static int bmi_acc_enable(struct bmi_state *st);
static int bmi_acc_batch(struct bmi_state *st, unsigned int period_us);
static int bmi_acc_selftest(struct bmi_state *st);
static int bmi_gyr_enable(struct bmi_state *st);
static int bmi_gyr_batch(struct bmi_state *st, unsigned int period_us);
static int bmi_gyr_selftest(struct bmi_state *st);

static struct bmi_hw bmi_hws[] = {
	{
		.dis			= {
			.cmd		= BMI_REG_CMD_ACC_SUSP,
			.ms_t		= 0,
		},
		.en			= {
			.cmd		= BMI_REG_CMD_ACC_NORM,
			.ms_t		= 5,
		},
		.rr			= bmi_rr_acc,
		.rr_0n			= ARRAY_SIZE(bmi_rr_acc) - 1,
		.buf_i			= (BMI_REG_DATA_14 - BMI_REG_DATA_0),
		.fifo_push_n		= 6,
		.fifo_hdr_mask		= 0x04,
		.fn_enable		= &bmi_acc_enable,
		.fn_batch		= &bmi_acc_batch,
		.fn_selftest		= &bmi_acc_selftest,
	},
	{
		.dis			= {
			.cmd		= BMI_REG_CMD_GYR_SUSP,
			.ms_t		= 0,
		},
		.en			= {
			.cmd		= BMI_REG_CMD_GYR_NORM,
			.ms_t		= 81,
		},
		.rr			= bmi_rr_gyr,
		.rr_0n			= ARRAY_SIZE(bmi_rr_gyr) - 1,
		.buf_i			= (BMI_REG_DATA_8 - BMI_REG_DATA_0),
		.fifo_push_n		= 6,
		.fifo_hdr_mask		= 0x08,
		.fn_enable		= &bmi_gyr_enable,
		.fn_batch		= &bmi_gyr_batch,
		.fn_selftest		= &bmi_gyr_selftest,
	},
};

struct bmi_snsr {
	void *nvs_st;
	struct sensor_cfg cfg;
	struct bmi_hw *hw;
	unsigned int odr_us;
	unsigned int period_us;
	unsigned int timeout_us;
	unsigned int usr_cfg;
	bool flush;
};

struct bmi_state {
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	struct bmi_snsr snsrs[BMI_HW_N];
	struct regulator_bulk_data vreg[ARRAY_SIZE(bmi_vregs)];
	struct workqueue_struct *wq;
	struct work_struct ws;
	unsigned int sts;		/* status flags */
	unsigned int errs;		/* error count */
	unsigned int inf;		/* NVS rd/wr */
	unsigned int enabled;		/* enable status */
	unsigned int period_us;		/* global period */
	unsigned int period_us_max;	/* maximum global period */
	unsigned int timeout_us;	/* global timeout */
	unsigned int snsr_t;		/* HW sensor time */
	unsigned int frame_n;		/* sensor time frame count */
	unsigned int lost_frame_n;	/* frames lost to FIFO overflow */
	unsigned int hw_n;		/* sensor count */
	unsigned int hw2ids[BMI_HW_N];	/* sensor id */
	unsigned int hw_en;		/* for HW access tracking */
	bool pm_en;			/* pm enable status */
	bool irq_dis;			/* interrupt host disable flag */
	bool irq_set_irq_wake;		/* interrupt wake enable status */
	bool no_irq_no_wake_on;		/* user cfg when no IRQ */
	bool irq_setup_done;		/* irq probe status: true=setup complete; */
	atomic64_t ts_irq;		/* interrupt timestamp */
	s64 ts_st_irq;			/* sensor time IRQ timestamp */
	s64 ts;				/* timestamp */
	s64 ts_lo;			/* timestamp threshold low */
	s64 ts_hi;			/* timestamp threshold high */
	s64 ts_odr;			/* timestamp ODR */
	s64 ts_hw;			/* timestamp HW access */
	s64 period_ns;			/* global period in ns */
	u8 int_out_ctrl;		/* user interrupt cfg */
	u8 int_latch;			/* " */
	u8 int_map_0;			/* " */
	u8 int_map_1;			/* " */
	u8 int_map_2;			/* " */
	u8 acc_conf;			/* user cfg */
	u8 gyr_conf;			/* user cfg */
	u16 i2c_addr;			/* I2C address */
	u16 buf_i;			/* buffer index */
	u8 buf[256];			/* data buffer */
};


static void bmi_mutex_lock(struct bmi_state *st)
{
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			st->nvs->nvs_mutex_lock(st->snsrs[i].nvs_st);
	}
}

static void bmi_mutex_unlock(struct bmi_state *st)
{
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			st->nvs->nvs_mutex_unlock(st->snsrs[i].nvs_st);
	}
}

static void bmi_err(struct bmi_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static int bmi_i2c_rd(struct bmi_state *st, u8 reg, u16 len, u8 *val)
{
	struct i2c_msg msg[2];
	s64 ts;
	int ret;

	if (st->i2c_addr) {
		msg[0].addr = st->i2c_addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &reg;
		msg[1].addr = st->i2c_addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = val;
		ts = st->ts_hw;
		if (st->hw_en)
			ts += (BMI_HW_DELAY_DEV_ON_US * 1000);
		else
			ts += (BMI_HW_DELAY_DEV_OFF_US * 1000);
		ts -= nvs_timestamp();
		if (ts > 0) {
			/* HW access timing rules (datasheet 3.2.4) */
			ts /= 1000;
			ts++;
			if (st->sts & NVS_STS_SPEW_MSG)
				dev_info(&st->i2c->dev,
					 "%s HW access delay=%lldus\n",
					 __func__, ts);
			udelay(ts);
		}
		ret = i2c_transfer(st->i2c->adapter, msg, 2);
		st->ts_hw = nvs_timestamp();
		if (ret != 2) {
			bmi_err(st);
			dev_err(&st->i2c->dev, "%s ERR: 0x%02X\n",
				__func__, reg);
			return -EIO;
		}
	}

	return 0;
}

static int bmi_i2c_wr(struct bmi_state *st, u8 reg, u8 val)
{
	struct i2c_msg msg;
	int ret;
	s64 ts;
	u8 buf[2];

	if (st->i2c_addr) {
		buf[0] = reg;
		buf[1] = val;
		msg.addr = st->i2c_addr;
		msg.flags = 0;
		msg.len = 2;
		msg.buf = buf;
		ts = st->ts_hw;
		if (st->hw_en)
			ts += (BMI_HW_DELAY_DEV_ON_US * 1000);
		else
			ts += (BMI_HW_DELAY_DEV_OFF_US * 1000);
		ts -= nvs_timestamp();
		if (ts > 0) {
			/* HW access timing rules (datasheet 3.2.4) */
			ts /= 1000;
			ts++;
			if (st->sts & NVS_STS_SPEW_MSG)
				dev_info(&st->i2c->dev,
					 "%s HW access delay=%lldus\n",
					 __func__, ts);
			udelay(ts);
		}
		ret = i2c_transfer(st->i2c->adapter, &msg, 1);
		st->ts_hw = nvs_timestamp();
		if (ret != 1) {
			bmi_err(st);
			dev_err(&st->i2c->dev, "%s ERR: 0x%02X=>0x%02X\n",
				__func__, val, reg);
			return -EIO;
		}

		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s 0x%02X=>0x%02X\n",
				 __func__, val, reg);
	}

	return 0;
}

static int bmi_i2c_wr_rd(struct bmi_state *st, u8 reg, u8 val)
{
	u8 val_rd;
	unsigned int i;
	int ret;

	if (!st->i2c_addr)
		return 0;

	val_rd = ~val;
	for (i = 0; i < BMI_I2C_WR_RD_RETRY_N; i++) {
		ret = bmi_i2c_wr(st, reg, val);
		if (!ret) {
			udelay(BMI_HW_DELAY_DEV_ON_US);
			ret = bmi_i2c_rd(st, reg, 1, &val_rd);
			if (val == val_rd && !ret)
				break;

			if (st->sts & NVS_STS_SPEW_MSG)
				dev_err(&st->i2c->dev,
					"%s 0x%02X: wr 0x%02X != rd 0x%02X\n",
					 __func__, reg, val, val_rd);
		}
	}

	return ret;
}

static int bmi_cmd_wr(struct bmi_state *st, struct bmi_cmd *cmd)
{
	unsigned int ms;
	int ret;

	ret = bmi_i2c_wr(st, BMI_REG_CMD, cmd->cmd);
	if (!ret) {
		ms = cmd->ms_t;
		if (!ms)
			ms = BMI_CMD_DELAY_MS;
		msleep(ms);
	}
	return ret;
}

struct bmi_odr {
	unsigned int period_us;
	u8 hw;
};

static unsigned int bmi_odr(struct bmi_odr *odrs, unsigned int odrs_n,
			    unsigned int period_us)
{
	unsigned int i;
	unsigned int n;

	n = odrs_n;
	n--;
	for (i = 0; i < n; i++) {
		if (period_us >= odrs[i].period_us)
			return i;
	}

	return i;
}

static struct bmi_odr bmi_odr_acc[] = {
	{ 1280000, 0x01 },
	{ 640000, 0x02 },
	{ 320000, 0x03 },
	{ 160000, 0x04 },
	{ 80000, 0x05 },
	{ 40000, 0x06 },
	{ 20000, 0x07 },
	{ 10000, 0x08 },
	{ 5000,  0x09 },
	{ 2500,  0x0A },
	{ 1250,  0x0B },
	{ 625,   0x0C },
};

static int bmi_acc_batch(struct bmi_state *st, unsigned int period_us)
{
	u8 val;
	unsigned int i;
	unsigned int odr_i;
	int ret;

	i = st->hw2ids[BMI_HW_ACC];
	if (i < BMI_HW_N) {
		odr_i = bmi_odr(bmi_odr_acc, ARRAY_SIZE(bmi_odr_acc),
				period_us);
		val = bmi_odr_acc[odr_i].hw;
		val |= st->acc_conf;
		ret = bmi_i2c_wr(st, BMI_REG_ACC_CONF, val);
		if (!ret)
			st->snsrs[i].odr_us = bmi_odr_acc[odr_i].period_us;
	} else {
		ret = -EPERM;
	}
	return ret;
}

static u8 bmi_acc_range[] = {
	0x03,
	0x05,
	0x08,
	0x0C,
};

static int bmi_acc_enable(struct bmi_state *st)
{
	u8 val;
	unsigned int i;
	int ret;

	i = st->hw2ids[BMI_HW_ACC];
	if (i < BMI_HW_N) {
		val = bmi_acc_range[st->snsrs[i].usr_cfg];
		ret = bmi_i2c_wr(st, BMI_REG_ACC_RANGE, val);
		ret |= bmi_cmd_wr(st, &bmi_hws[BMI_HW_ACC].en);
		if (ret) {
			st->hw_en &= ~(1 << BMI_HW_ACC);
		} else {
			ret = bmi_i2c_rd(st, BMI_REG_PMU_STATUS, 1, &val);
			if (!ret) {
				val &= BMI_REG_PMU_STATUS_MSK_ACC;
				if (val == BMI_REG_PMU_STATUS_ACC_NORM)
					st->hw_en |= (1 << BMI_HW_ACC);
			}
		}
	} else {
		ret = -EPERM;
	}
	return ret;
}

static int bmi_acc_selftest(struct bmi_state *st)
{
	int16_t x_pos;
	int16_t y_pos;
	int16_t z_pos;
	int16_t x_neg;
	int16_t y_neg;
	int16_t z_neg;
	unsigned int i;
	unsigned int usr_cfg;
	int ret;

	i = st->hw2ids[BMI_HW_ACC];
	usr_cfg = st->snsrs[i].usr_cfg;
	st->snsrs[i].usr_cfg = 2; /* 8g */
	ret = bmi_acc_enable(st);
	st->snsrs[i].usr_cfg = usr_cfg;
	ret |= bmi_i2c_wr(st, BMI_REG_ACC_CONF, 0x2C);
	/* Enable accel self-test with positive excitation */
	ret |= bmi_i2c_wr(st, BMI_REG_SELF_TEST, BMI_SELFTEST_ACC_POS);
	mdelay(BMI_SELFTEST_ACC_DELAY_MS);
	/* buffer corrupt from self-test so reset buffer index */
	st->buf_i = 0;
	ret |= bmi_i2c_rd(st, BMI_REG_DATA_14, 6, st->buf);
	x_pos = (st->buf[1] << 8) | (st->buf[0]);
	y_pos = (st->buf[3] << 8) | (st->buf[2]);
	z_pos = (st->buf[5] << 8) | (st->buf[4]);
	/* Enable accel self-test with negative excitation */
	ret |= bmi_i2c_wr(st, BMI_REG_SELF_TEST, BMI_SELFTEST_ACC_NEG);
	mdelay(BMI_SELFTEST_ACC_DELAY_MS);
	ret |= bmi_i2c_rd(st, BMI_REG_DATA_14, 6, st->buf);
	if (ret)
		return ret;

	x_neg = (st->buf[1] << 8) | (st->buf[0]);
	y_neg = (st->buf[3] << 8) | (st->buf[2]);
	z_neg = (st->buf[5] << 8) | (st->buf[4]);
	if (!((abs(x_neg - x_pos) > BMI_SELFTEST_ACC_LIMIT)
		&& (abs(y_neg - y_pos) > BMI_SELFTEST_ACC_LIMIT)
		&& (abs(z_neg - z_pos) > BMI_SELFTEST_ACC_LIMIT))) {
		/* failure */
		ret = 1;
	}
	return ret;
}

static struct bmi_odr bmi_odr_gyr[] = {
	{ 40000, 0x06 },
	{ 20000, 0x07 },
	{ 10000, 0x08 },
	{ 5000,  0x09 },
	{ 2500,  0x0A },
	{ 1250,  0x0B },
	{ 625,   0x0C },
	{ 312,   0x0D },
};

static int bmi_gyr_batch(struct bmi_state *st, unsigned int period_us)
{
	u8 val;
	unsigned int i;
	unsigned int odr_i;
	int ret;

	i = st->hw2ids[BMI_HW_GYR];
	if (i < BMI_HW_N) {
		odr_i = bmi_odr(bmi_odr_gyr, ARRAY_SIZE(bmi_odr_gyr),
				period_us);
		val = bmi_odr_gyr[odr_i].hw;
		val |= st->gyr_conf;
		ret = bmi_i2c_wr(st, BMI_REG_GYR_CONF, val);
		if (!ret)
			st->snsrs[i].odr_us = bmi_odr_gyr[odr_i].period_us;
	} else {
		ret = -EPERM;
	}
	return ret;
}

static int bmi_gyr_enable(struct bmi_state *st)
{
	u8 val;
	unsigned int i;
	int ret;

	i = st->hw2ids[BMI_HW_GYR];
	if (i < BMI_HW_N) {
		val = st->snsrs[i].usr_cfg;
		ret = bmi_i2c_wr(st, BMI_REG_GYR_RANGE, val);
		ret |= bmi_cmd_wr(st, &bmi_hws[BMI_HW_GYR].en);
		if (ret) {
			st->hw_en &= ~(1 << BMI_HW_GYR);
		} else {
			ret = bmi_i2c_rd(st, BMI_REG_PMU_STATUS, 1, &val);
			if (!ret) {
				val &= BMI_REG_PMU_STATUS_MSK_GYR;
				if (val == BMI_REG_PMU_STATUS_GYR_NORM)
					st->hw_en |= (1 << BMI_HW_GYR);
			}
		}
	} else {
		ret = -EPERM;
	}
	return ret;
}

static int bmi_gyr_selftest(struct bmi_state *st)
{
	uint8_t val;
	unsigned int i;
	unsigned int usr_cfg;
	int ret;

	i = st->hw2ids[BMI_HW_ACC];
	usr_cfg = st->snsrs[i].usr_cfg;
	st->snsrs[i].usr_cfg = st->snsrs[i].hw->rr_0n;
	ret = bmi_gyr_enable(st);
	st->snsrs[i].usr_cfg = usr_cfg;
	ret |= bmi_i2c_wr(st, BMI_REG_SELF_TEST, BMI_SELFTEST_GYR);
	mdelay(BMI_SELFTEST_GYR_DELAY_MS);
	ret |= bmi_i2c_rd(st, BMI_REG_STATUS, 1, &val);
	if (!ret) {
		if (!(val & 0x02))
			/* failure */
			ret = 1;
	}
	return ret;
}

static unsigned int bmi_buf2sensortime(u8 *buf)
{
	unsigned int snsr_t = 0;

	snsr_t |= buf[2];
	snsr_t <<= 8;
	snsr_t |= buf[1];
	snsr_t <<= 8;
	snsr_t |= buf[0];
	return snsr_t;
}

static unsigned int bmi_sensortime_rd(struct bmi_state *st,
				      unsigned int *snsr_t)
{
	u8 buf[3];
	int ret;

	ret = bmi_i2c_rd(st, BMI_REG_SENSORTIME_0, sizeof(buf), buf);
	if (ret)
		return ret;

	*snsr_t = bmi_buf2sensortime(buf);
	return 0;
}

static int bmi_ts_init(struct bmi_state *st)
{
	int ret;

	ret = bmi_sensortime_rd(st, &st->snsr_t);
	ret |= bmi_cmd_wr(st, &bmi_cmd_fifo_clr);
	if (!ret)
		st->ts = 0;
	return ret;
}

static int bmi_pm(struct bmi_state *st, bool en)
{
	unsigned int i;
	int ret;

	if (en == st->pm_en)
		return 0;

	if (en) {
		ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
				       ARRAY_SIZE(bmi_vregs));
		if (ret > 0)
			mdelay(BMI_HW_DELAY_POR_MS);
		ret = bmi_cmd_wr(st, &bmi_cmd_rst_soft);
		ret |= bmi_i2c_wr_rd(st, BMI_REG_INT_MAP_0, st->int_map_0);
		ret |= bmi_i2c_wr_rd(st, BMI_REG_INT_MAP_1, st->int_map_1);
		ret |= bmi_i2c_wr_rd(st, BMI_REG_INT_MAP_2, st->int_map_2);
		ret |= bmi_cmd_wr(st, &bmi_cmd_rst_int);
		ret |= bmi_i2c_wr_rd(st, BMI_REG_INT_OUT_CTRL,
				     st->int_out_ctrl);
		ret |= bmi_i2c_wr_rd(st, BMI_REG_INT_LATCH, st->int_latch);
		ret |= bmi_i2c_wr_rd(st, BMI_REG_INT_EN_1, 0x10);
	} else {
		st->ts = 0;
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(bmi_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(bmi_vregs))) {
			/* we're fully powered */
			ret = 0;
			for (i = 0; i < st->hw_n; i++) {
				ret |= bmi_cmd_wr(st, &st->snsrs[i].hw->dis);
				st->hw_en &= ~(1 << i);
			}
		} else if (ret > 0) {
			/* partially powered so go to full before disables */
			ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
					       ARRAY_SIZE(bmi_vregs));
			mdelay(BMI_HW_DELAY_POR_MS);
			for (i = 0; i < st->hw_n; i++) {
				ret |= bmi_cmd_wr(st, &st->snsrs[i].hw->dis);
				st->hw_en &= ~(1 << i);
			}
		}
		/* disables put us in low power sleep state in case no vregs */
		ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(bmi_vregs));
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pm_en=%x  ERR=%d\n",
			__func__, en, ret);
	} else {
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pm_en: %x=>%x\n",
				 __func__, st->pm_en, en);
		st->pm_en = en;
	}
	return ret;
}

static void bmi_pm_exit(struct bmi_state *st)
{
	bmi_pm(st, false);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(bmi_vregs));
}

static int bmi_pm_init(struct bmi_state *st)
{
	int ret;

	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(bmi_vregs), bmi_vregs);
	ret = bmi_pm(st, true);
	return ret;
}

static void bmi_work(struct work_struct *ws)
{
	struct bmi_state *st = container_of((struct work_struct *)ws,
					    struct bmi_state, ws);
	s64 ts1;
	s64 ts2;
	u64 ts_diff;
	unsigned long sleep_us;
	unsigned int buf_i;
	unsigned int i;
	int ret;

	while (st->enabled) {
		ts1 = nvs_timestamp();
		if (st->sts & NVS_STS_SPEW_IRQ)
			dev_info(&st->i2c->dev, "%s ts=%lld\n", __func__, ts1);
		bmi_mutex_lock(st);
		ret = bmi_i2c_rd(st, BMI_REG_DATA_0, sizeof(st->buf), st->buf);
		if (!ret) {
			for (i = 0; i < st->hw_n; i++) {
				if (st->enabled & (1 << i)) {
					buf_i = st->snsrs[i].hw->buf_i;
					st->nvs->handler(st->snsrs[i].nvs_st,
							 &st->buf[buf_i], ts1);
				}
			}
		}
		ts2 = nvs_timestamp();
		ts_diff = (ts2 - ts1) / 1000; /* ns => us */
		if (st->period_us > (unsigned int)ts_diff)
			sleep_us = st->period_us - (unsigned int)ts_diff;
		else
			sleep_us = 0;
		bmi_mutex_unlock(st);
		if (sleep_us > 10000) /* SLEEPING FOR LARGER MSECS ( 10ms+ ) */
			msleep(sleep_us / 1000);
		else if (sleep_us)
			/* SLEEPING FOR ~USECS OR SMALL MSECS ( 10us - 20ms) */
			usleep_range(sleep_us, st->period_us);
	}
}

static void bmi_push_flush(struct bmi_state *st)
{
	unsigned int i;
	int ret;

	for (i = 0; i < st->hw_n; i++) {
		if (st->snsrs[i].flush) {
			ret = st->nvs->handler(st->snsrs[i].nvs_st, NULL, 0LL);
			if (ret >= 0)
				st->snsrs[i].flush = false;
		}
	}
}

static int bmi_push(struct bmi_state *st, s64 ts, unsigned int hw)
{
	unsigned int i;
	unsigned int n;

	i = st->hw2ids[hw];
	if (ts > 0)
		st->nvs->handler(st->snsrs[i].nvs_st, &st->buf[st->buf_i], ts);
	if (st->sts & BMI_STS_SPEW_FIFO) {
		for (n = 0; n < bmi_hws[hw].fifo_push_n; n++) {
			dev_info(&st->i2c->dev,
				 "%s: buf[%u]=0x%02X\n",
				 st->snsrs[i].cfg.name,
				 st->buf_i, st->buf[st->buf_i]);
			st->buf_i++;
		}
	} else {
		n = bmi_hws[hw].fifo_push_n;
		st->buf_i += n;
	}
	return n;
}

static unsigned int fifo_hw_frame_seqs[] = {
	BMI_HW_GYR,
	BMI_HW_ACC,
};

static void bmi_dbg_thr(struct bmi_state *st, s64 ts_irq)
{
	s64 ns;

	if (st->ts_odr)
		ns = st->ts_odr;
	else
		ns = st->period_ns;
	dev_info(&st->i2c->dev,
		 "%s CALC ts=%lld irq=%lld->%lld (%lld) odr=%lld\n",
		 __func__, st->ts, st->ts_st_irq, ts_irq,
		 ts_irq - st->ts_st_irq, ns);
}

static int bmi_frame_regular(struct bmi_state *st, u16 buf_n)
{
	u8 hdr;
	s64 ns;
	s64 ts_irq;
	unsigned int hw;
	unsigned int i;
	unsigned int n;

	st->frame_n++;
	if (st->ts_hi) {
		if (st->ts > st->ts_hi) {
			/* missed this TS sync - calculate new thresholds */
			ts_irq = atomic64_read(&st->ts_irq);
			if (st->ts_odr)
				ns = st->ts_odr;
			else
				ns = st->period_ns;
			ns >>= 1;
			st->ts_lo = ts_irq - ns;
			st->ts_hi = ts_irq + ns;
			if (st->sts & BMI_STS_SPEW_TS)
				bmi_dbg_thr(st, ts_irq);
			st->ts_st_irq = ts_irq;
		} else if (st->ts >= st->ts_lo) {
			/* IRQ TS within range */
			if (st->sts & BMI_STS_SPEW_TS)
				dev_info(&st->i2c->dev,
					 "%s IRQ SYNC ts=%lld=>%lld (%lld)\n",
					 __func__, st->ts, st->ts_st_irq,
					 st->ts - st->ts_st_irq);
			st->ts = st->ts_st_irq;
		}
	} else {
		/* first timestamp and event */
		st->ts_st_irq = st->ts + st->period_ns;
		st->ts_lo = st->period_ns;
		st->ts_lo >>= 1;
		st->ts_lo += st->ts;
		st->ts_hi = st->ts_lo + st->period_ns;
		if (st->sts & BMI_STS_SPEW_TS)
			dev_info(&st->i2c->dev,
				 "%s START ts=%lld lo=%lld hi=%lld odr=%lld\n",
				 __func__, st->ts, st->ts_lo, st->ts_hi,
				 st->period_ns);
	}
	hdr = st->buf[st->buf_i];
	st->buf_i++;
	if (st->ts > st->ts_hw)
		/* st->ts_hw can be used as ts_now since it's the timestamp of
		 * the last time the HW was accessed. It's used here to confirm
		 * that we never go forward in time.
		 */
		st->ts = st->ts_hw;
	for (i = 0; i < BMI_HW_N; i++) {
		hw = fifo_hw_frame_seqs[i];
		if (hdr & bmi_hws[hw].fifo_hdr_mask) {
			n = buf_n - st->buf_i;
			if (n < bmi_hws[hw].fifo_push_n)
				/* not enough data */
				return -1;

			bmi_push(st, st->ts, hw);
		}
	}

	if (st->ts_odr)
		st->ts += st->ts_odr;
	else
		st->ts += st->period_ns;
	return 0;
}

static int bmi_frame_control(struct bmi_state *st, u16 buf_n)
{
	u8 hdr;
	s64 ns;
	unsigned int i;
	unsigned int n;
	unsigned int snsr_t;

	hdr = st->buf[st->buf_i] >> 2;
	hdr &= 0x07;
	n = buf_n - st->buf_i;
	st->buf_i++;
	switch (hdr) {
	case 0:
		if (n < 2)
			break;

		n = st->buf[st->buf_i]; /* skip frame count */
		st->frame_n += n; /* for ODR calculation */
		i = st->lost_frame_n; /* save old */
		st->lost_frame_n += n; /* debug FYI */
		if (st->lost_frame_n < i)
			/* rollover */
			st->lost_frame_n = -1;
		/* update timestamp to include lost frames */
		if (st->ts_odr)
			ns = st->ts_odr;
		else
			ns = st->period_ns;
		ns *= n;
		ns += st->ts;
		if (st->sts & BMI_STS_SPEW_FIFO)
			dev_info(&st->i2c->dev,
				 "SKIP FRAME: buf[%u]=0x%02X\n", st->buf_i, n);
		if (st->sts & BMI_STS_SPEW_TS)
			dev_info(&st->i2c->dev,
				 "SKIP FRAME: n=%u odr=%lld TS: %lld->%lld\n",
				 n, st->ts_odr, st->ts, ns);
		st->ts = ns;
		st->buf_i++;
		return 0;

	case 1:
		if (n < 4)
			break;

		snsr_t = bmi_buf2sensortime(&st->buf[st->buf_i]);
		if (st->ts_odr) {
			if (st->frame_n) {
				if (snsr_t > st->snsr_t) {
					n = snsr_t - st->snsr_t;
				} else {
					/* counter rolled over */
					n = ~0xFFFFFFU; /* 24->32 */
					n |= st->snsr_t;
					n = ~n;
					n++;
					n += snsr_t;
					if (st->sts & BMI_STS_SPEW_TS)
						dev_info(&st->i2c->dev,
							"%s st: %u->%u n=%u\n",
							 __func__, st->snsr_t,
							 snsr_t, n);
				}
				/* n is the number of sensortime ticks since
				 * last time.  Each tick is 39062.5ns.
				 */
				ns = n;
				ns *= 390625;
				n = st->frame_n;
				n *= 10;
				st->ts_odr = ns / n;
			}
		} else {
			/* starting count for sensortime */
			if (st->sts & BMI_STS_SPEW_TS)
				dev_info(&st->i2c->dev,
					 "%s START ts=%lld odr=%lld->%lld\n",
					 __func__, st->ts, st->ts_odr,
					 st->period_ns);

			st->ts_odr = st->period_ns;
		}
		if (st->sts & BMI_STS_SPEW_ST) {
			for (i = 0; i < 3; i++) {
				dev_info(&st->i2c->dev,
					 "SENSORTIME: buf[%u]=0x%02X\n",
					 st->buf_i, st->buf[st->buf_i]);
				st->buf_i++;
			}

			dev_info(&st->i2c->dev,
				 "snsr_t: %u->%u n=%u frame_n=%u odr=%lld\n",
				 st->snsr_t, snsr_t, n, st->frame_n,
				 st->ts_odr);
		}
		st->snsr_t = snsr_t;
		st->frame_n = 0;
		st->buf_i = 0; /* break out of outer loop */
		return -1; /* break out of inner loop */

	case 2:
		if (n < 2)
			break;

		/* ODR has changed - use st->period_ns until next sensortime */
		st->ts_odr = 0;
		if (st->sts & (BMI_STS_SPEW_FIFO | BMI_STS_SPEW_TS))
			dev_info(&st->i2c->dev,
				 "CONFIG: buf[%u]=0x%02X\n",
				 st->buf_i, st->buf[st->buf_i]);
		st->buf_i++;
		return 0;

	default:
		/* should never get here */
		if (st->sts & (BMI_STS_SPEW_FIFO |
			       BMI_STS_SPEW_ST |
			       BMI_STS_SPEW_TS)) {
			for (i = 0; i < n; i++) {
				dev_info(&st->i2c->dev,
					 "ERR: buf[%u]=0x%02X\n",
					 st->buf_i, st->buf[st->buf_i]);
				st->buf_i++;
			}
		}
	}

	return -1;
}

static int bmi_read(struct bmi_state *st)
{
	u8 hdr;
	u16 buf_n;
	u16 fifo_n;
	int ret;

	ret = bmi_i2c_rd(st, BMI_REG_FIFO_LENGTH_0,
			 sizeof(fifo_n), (u8 *)&fifo_n);
	if (ret)
		return ret;

	if (st->sts & BMI_STS_SPEW_FIFO)
		dev_info(&st->i2c->dev, "%s fifo_n=%u\n", __func__, fifo_n);
	fifo_n &= 0x07FF;
	/* to get the sensor time apparently we have to +25... HW bug? */
	fifo_n += 25;
	while (fifo_n) {
		if (fifo_n > sizeof(st->buf))
			buf_n = sizeof(st->buf);
		else
			buf_n = fifo_n;
		ret = bmi_i2c_rd(st, BMI_REG_FIFO_DATA, buf_n, st->buf);
		if (ret)
			return ret;

		st->buf_i = 0;
		while (st->buf_i < buf_n) {
			hdr = st->buf[st->buf_i];
			if (st->sts & BMI_STS_SPEW_FIFO)
				dev_info(&st->i2c->dev,
					 "HDR: buf[%u]=0x%02X  fifo_n=%u\n",
					 st->buf_i, hdr, fifo_n);
			if (hdr == 0x80)
				/* "overreading the FIFO" */
				return 0;

			if (hdr & 0x80) {
				/* regular frame */
				ret = bmi_frame_regular(st, buf_n);
				if (ret < 0)
					/* not enough data to process */
					break;
			} else if (hdr & 0x40) {
				/* control frame */
				ret = bmi_frame_control(st, buf_n);
				if (ret < 0)
					break;
			} else {
				/* error - but possible when disabling */
				st->buf_i = 0; /* exit fifo_n loop */
				break; /* exit (st->buf_i < buf_n) loop */
			}
		}

		if (!st->buf_i)
			/* not enough data to process - exit fifo_n loop */
			break;

		fifo_n -= st->buf_i;
	}

	return 0;
}

static irqreturn_t bmi_irq_thread(int irq, void *dev_id)
{
	struct bmi_state *st = (struct bmi_state *)dev_id;

	bmi_mutex_lock(st);
	bmi_read(st);
	bmi_mutex_unlock(st);
	return IRQ_HANDLED;
}

static irqreturn_t bmi_irq_handler(int irq, void *dev_id)
{
	struct bmi_state *st = (struct bmi_state *)dev_id;
	s64 ts_old;
	s64 ts_diff;
	s64 ts = nvs_timestamp();

	ts_old = atomic64_xchg(&st->ts_irq, ts);
	if (st->sts & NVS_STS_SPEW_IRQ) {
		ts_diff = ts - ts_old;
		dev_info(&st->i2c->dev, "%s ts=%lld  ts_diff=%lld\n",
			 __func__, ts, ts_diff);
	}
	if (!st->ts) {
		/* first timestamp */
		st->ts = ts;
		st->ts_hi = 0;
		st->ts_odr = 0;
		st->frame_n = 0;
	}
	return IRQ_WAKE_THREAD;
}

static int bmi_period(struct bmi_state *st, unsigned int msk_en, int snsr_id)
{
	unsigned int us;
	unsigned int i;
	int ret = 0;

	if (st->i2c->irq > 0) {
		if (msk_en & (1 << snsr_id)) {
			us = st->snsrs[snsr_id].period_us;
			ret = st->snsrs[snsr_id].hw->fn_batch(st, us);
		}
		us = st->period_us_max;
		for (i = 0; i < st->hw_n; i++) {
			if (msk_en & (1 << i)) {
				if (st->snsrs[i].period_us) {
					if (st->snsrs[i].odr_us < us)
						us = st->snsrs[i].odr_us;
				}
			}
		}
	} else {
		us = st->period_us_max;
		for (i = 0; i < st->hw_n; i++) {
			if (msk_en & (1 << i)) {
				if (st->snsrs[i].period_us) {
					if (st->snsrs[i].period_us < us)
						us = st->snsrs[i].period_us;
				}
			}
		}

		for (i = 0; i < st->hw_n; i++) {
			if (msk_en & (1 << i)) {
				ret = st->snsrs[i].hw->fn_batch(st, us);
				if (ret < 0)
					return ret;
			}
		}
	}

	if (st->sts & (NVS_STS_SPEW_MSG & BMI_STS_SPEW_TS))
		dev_info(&st->i2c->dev,
			 "%s msk_en=%X period_us: %u->%u\n",
			 __func__, msk_en, st->period_us, us);
	st->period_us = us;
	st->period_ns = (s64)us * 1000; /* us=> ns */
	return ret;
}

static int bmi_disable(struct bmi_state *st, int snsr_id)
{
	bool disable = true;
	int ret = 0;
	unsigned int msk;
	u8 val;

	if (snsr_id >= 0) {
		msk = ~(1 << snsr_id);
		if (st->enabled & msk) {
			disable = false;
			ret = bmi_cmd_wr(st, &st->snsrs[snsr_id].hw->dis);
			if (!ret) {
				st->hw_en &= msk;
				st->enabled &= msk;
				if (st->i2c->irq > 0) {
					val = (st->enabled <<
					       BMI_REG_FIFO_CONFIG_1_ENS);
					val |= 0x1A;
					ret = bmi_i2c_wr(st,
							 BMI_REG_FIFO_CONFIG_1,
							 val);
				}
				bmi_period(st, st->enabled, snsr_id);
			}
		}
	}
	if (disable) {
		ret = bmi_i2c_wr(st, BMI_REG_FIFO_CONFIG_1, 0);
		ret |= bmi_pm(st, false);
		if (!ret)
			st->enabled = 0;
	}
	return ret;
}

static int bmi_enable(void *client, int snsr_id, int enable)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int ret;
	u8 val;

	if (enable < 0)
		return st->enabled & (1 << snsr_id);

	if (enable) {
		enable = st->enabled | (1 << snsr_id);
		ret = bmi_pm(st, true);
		if (ret < 0)
			return ret;

		ret = bmi_period(st, enable, snsr_id);
		if (st->i2c->irq > 0) {
			if (!st->enabled)
				/* first enabled device */
				ret |= bmi_ts_init(st);
			ret |= st->snsrs[snsr_id].hw->fn_enable(st);
			val = enable << BMI_REG_FIFO_CONFIG_1_ENS;
			val |= 0x1A;
			ret |= bmi_i2c_wr(st, BMI_REG_FIFO_CONFIG_1, val);
			if (!ret)
				st->enabled = enable;
		} else {
			ret |= st->snsrs[snsr_id].hw->fn_enable(st);
			if (!ret) {
				if (st->enabled) {
					/* already enabled */
					st->enabled = enable;
				} else {
					st->enabled = enable;
					cancel_work_sync(&st->ws);
					queue_work(st->wq, &st->ws);
				}
			}
		}
		if (ret)
			bmi_disable(st, snsr_id);
	} else {
		ret = bmi_disable(st, snsr_id);
	}
	return ret;
}

static int bmi_able(struct bmi_state *st, unsigned int msk_en)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < st->hw_n; i++) {
		if (msk_en & (1 << i))
			ret |= bmi_enable(st, i, 1);
	}

	for (i = 0; i < st->hw_n; i++) {
		if (!(msk_en & (1 << i)))
			ret |= bmi_enable(st, i, 0);
	}

	return ret;
}

static int bmi_batch(void *client, int snsr_id, int flags,
		     unsigned int period_us, unsigned int timeout_us)
{
	struct bmi_state *st = (struct bmi_state *)client;

	if (timeout_us && st->i2c->irq <= 0)
		/* timeout not supported (no HW FIFO) */
		return -EINVAL;

	st->snsrs[snsr_id].period_us = period_us;
	st->snsrs[snsr_id].timeout_us = timeout_us;
	return bmi_period(st, st->enabled, snsr_id);
}

static int bmi_batch_read(void *client, int snsr_id,
			  unsigned int *period_us, unsigned int *timeout_us)
{
	struct bmi_state *st = (struct bmi_state *)client;

	if (period_us) {
		if (st->i2c->irq > 0)
			/* IRQ driven */
			*period_us = st->snsrs[snsr_id].odr_us;
		else
			/* poll mode */
			*period_us = st->period_us;
	}
	if (timeout_us)
		*timeout_us = st->timeout_us;
	return 0;
}

static int bmi_flush(void *client, int snsr_id)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int ret = -EINVAL;

	if (st->enabled & (1 << snsr_id)) {
		st->snsrs[snsr_id].flush = true;
		ret = bmi_read(st);
		bmi_push_flush(st);
	}
	return ret;
}

static int bmi_max_range(void *client, int snsr_id, int max_range)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int i = max_range;

	if (st->enabled & (1 << snsr_id))
		/* can't change settings on the fly (disable device first) */
		return -EPERM;

	if (i > st->snsrs[snsr_id].hw->rr_0n)
		/* clamp to highest setting */
		i = st->snsrs[snsr_id].hw->rr_0n;
	st->snsrs[snsr_id].usr_cfg = i;
	st->snsrs[snsr_id].cfg.max_range.ival =
				   st->snsrs[snsr_id].hw->rr[i].max_range.ival;
	st->snsrs[snsr_id].cfg.max_range.fval =
				   st->snsrs[snsr_id].hw->rr[i].max_range.fval;
	st->snsrs[snsr_id].cfg.resolution.ival =
				  st->snsrs[snsr_id].hw->rr[i].resolution.ival;
	st->snsrs[snsr_id].cfg.resolution.fval =
				  st->snsrs[snsr_id].hw->rr[i].resolution.fval;
	st->snsrs[snsr_id].cfg.scale.ival =
				       st->snsrs[snsr_id].hw->rr[i].scale.ival;
	st->snsrs[snsr_id].cfg.scale.fval =
				       st->snsrs[snsr_id].hw->rr[i].scale.fval;
	/* AXIS sensors need resolution put in the scales */
	for (i = 0; i < st->snsrs[snsr_id].cfg.ch_n_max; i++) {
		st->snsrs[snsr_id].cfg.scales[i].ival =
					     st->snsrs[snsr_id].cfg.scale.ival;
		st->snsrs[snsr_id].cfg.scales[i].fval =
					     st->snsrs[snsr_id].cfg.scale.fval;
	}

	return 0;
}

static int bmi_reset(void *client, int snsr_id)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int msk_en = st->enabled;
	int ret;

	/* power up if not already */
	ret = bmi_pm(st, true);
	/* disable all devices which will power us down */
	ret |= bmi_able(st, 0);
	/* power back up which also reinitializes us with the softreset */
	ret |= bmi_pm(st, true);
	/* restore state before all of this */
	ret |= bmi_able(st, msk_en);
	return ret;
}

static int bmi_selftest(void *client, int snsr_id, char *buf)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int msk_en = st->enabled;
	unsigned int i;
	ssize_t t;
	int ret;

	/* power up so we can chat */
	ret = bmi_pm(st, true);
	/* disable all devices which will power us down */
	ret |= bmi_able(st, 0);
	/* power back up which also reinitializes us */
	ret |= bmi_pm(st, true);
	/* do self-test(s) */
	if (snsr_id < 0) {
		for (i = 0; i < st->hw_n; i++) {
			if (st->snsrs[i].hw->fn_selftest) {
				ret |= bmi_cmd_wr(st, &bmi_cmd_rst_soft);
				ret |= st->snsrs[i].hw->fn_selftest(st);
			}
		}
	} else {
		i = snsr_id;
		if (st->snsrs[i].hw->fn_selftest)
			ret |= st->snsrs[i].hw->fn_selftest(st);
	}
	/* restore */
	bmi_able(st, 0);
	bmi_able(st, msk_en);
	if (buf) {
		if (ret < 0) {
			t = snprintf(buf, PAGE_SIZE,
				     "%s=ERR: %d\n", __func__, ret);
		} else {
			if (ret > 0)
				t = snprintf(buf, PAGE_SIZE,
					     "%s=FAIL\n", __func__);
			else
				t = snprintf(buf, PAGE_SIZE,
					     "%s=PASS\n", __func__);
		}
		return t;
	}

	return ret;
}

static int bmi_regs(void *client, int snsr_id, char *buf)
{
	struct bmi_state *st = (struct bmi_state *)client;
	ssize_t t;
	u8 val;
	unsigned int i;
	int ret;

	t = snprintf(buf, PAGE_SIZE, "registers:\n");
	for (i = 0; i < BMI_REG_CMD; i++) {
		if (i == BMI_REG_FIFO_DATA)
			continue;

		ret = bmi_i2c_rd(st, i, 1, &val);
		if (ret)
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "0x%02X=ERR\n", i);
		else if (val)
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "0x%02X=0x%02X\n", i, val);
	}

	return t;
}

static int bmi_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct bmi_state *st = (struct bmi_state *)client;
	s64 ts_irq;

	switch (nvs & 0xFF) {
	case BMI_INF_VER:
	case BMI_INF_DBG:
	case BMI_INF_REG_WR:
	case BMI_INF_REG_RD:
		break;

	case BMI_INF_SPEW_FIFO:
		st->sts ^= BMI_STS_SPEW_FIFO;
		break;

	case BMI_INF_SPEW_ST:
		st->sts ^= BMI_STS_SPEW_ST;
		break;

	case BMI_INF_SPEW_TS:
		ts_irq = atomic64_read(&st->ts_irq);
		dev_info(&st->i2c->dev,
			"ts=%lld irq=%lld sti=%lld odr=%lld lo=%lld hi=%lld\n",
			 st->ts, ts_irq, st->ts_st_irq, st->ts_odr,
			 st->ts_lo, st->ts_hi);
		st->sts ^= BMI_STS_SPEW_TS;
		break;

	default:
		return -EINVAL;
	}

	st->inf = nvs;
	return 0;
}

static int bmi_nvs_read(void *client, int snsr_id, char *buf)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int i;
	unsigned int inf;
	int ret;
	ssize_t t;
	u8 val;

	inf = st->inf;
	switch (inf & 0xFF) {
	case BMI_INF_VER:
		t = snprintf(buf, PAGE_SIZE, "driver v.%u\n",
			     BMI_DRIVER_VERSION);
		t += snprintf(buf + t, PAGE_SIZE - t, "DEVICE TREE:\n");
		t += snprintf(buf + t, PAGE_SIZE - t, "int_out_ctrl=0x%02X\n",
			      st->int_out_ctrl);
		t += snprintf(buf + t, PAGE_SIZE - t, "int_latch=0x%02X\n",
			      st->int_latch);
		t += snprintf(buf + t, PAGE_SIZE - t, "int_map_0=0x%02X\n",
			      st->int_map_0);
		t += snprintf(buf + t, PAGE_SIZE - t, "int_map_1=0x%02X\n",
			      st->int_map_1);
		t += snprintf(buf + t, PAGE_SIZE - t, "int_map_2=0x%02X\n",
			      st->int_map_2);
		t += snprintf(buf + t, PAGE_SIZE - t, "no_irq_no_wake_on=%X\n",
			      st->no_irq_no_wake_on);
		t += snprintf(buf + t, PAGE_SIZE - t, "acc_conf=0x%02X\n",
			      st->acc_conf);
		t += snprintf(buf + t, PAGE_SIZE - t, "gyr_conf=0x%02X\n",
			      st->gyr_conf);
		return t;

	case BMI_INF_DBG:
		st->inf = BMI_INF_VER;
		t = snprintf(buf, PAGE_SIZE, "i2c_addr=0x%X\n", st->i2c_addr);
		t += snprintf(buf + t, PAGE_SIZE - t, "msk_en=%X\n",
			      st->enabled);
		if (st->i2c->irq > 0) {
			t += snprintf(buf + t, PAGE_SIZE - t, "irq=%d\n",
				      st->i2c->irq);
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "irq_set_irq_wake=%X\n",
				      st->irq_set_irq_wake);
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "lost_frame_n=%u\n", st->lost_frame_n);
			st->lost_frame_n = 0;
		}
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "period_us_max=%u\n", st->period_us_max);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "period_us=%u\n", st->period_us);
		for (i = 0; i < st->hw_n; i++) {
			t += snprintf(buf + t, PAGE_SIZE - t, "%s:\n",
				      st->snsrs[i].cfg.name);
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "period_us_odr=%u\n",
				      st->snsrs[i].odr_us);
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "period_us_req=%u\n",
				      st->snsrs[i].period_us);
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "timeout_us=%u\n",
				      st->snsrs[i].timeout_us);
			t += snprintf(buf + t, PAGE_SIZE - t, "usr_cfg=%u\n",
				      st->snsrs[i].usr_cfg);
			t += snprintf(buf + t, PAGE_SIZE - t, "flush=%x\n",
				      st->snsrs[i].flush);
		}

		return t;

	case BMI_INF_SPEW_FIFO:
		return snprintf(buf, PAGE_SIZE, "FIFO spew=%x\n",
				!!(st->sts & BMI_STS_SPEW_FIFO));

	case BMI_INF_SPEW_ST:
		return snprintf(buf, PAGE_SIZE, "sensortime spew=%x\n",
				!!(st->sts & BMI_STS_SPEW_ST));

	case BMI_INF_SPEW_TS:
		return snprintf(buf, PAGE_SIZE, "TS spew=%x\n",
				!!(st->sts & BMI_STS_SPEW_TS));

	case BMI_INF_REG_WR:
		ret = bmi_i2c_wr(st, (u8)((inf >> 8) & 0xFF),
				 (u8)((inf >> 16) & 0xFF));
		if (ret)
			return snprintf(buf, PAGE_SIZE,
					"REG WR (v=>r): ERR=%d\n", ret);

		return snprintf(buf, PAGE_SIZE,
				"REG WR (v=>r): 0x%02X=>0x%02X\n",
				(inf >> 16) & 0xFF, (inf >> 8) & 0xFF);

	case BMI_INF_REG_RD:
		ret = bmi_i2c_rd(st, (u8)((inf >> 8) & 0xFF), 1, &val);
		if (ret)
			return snprintf(buf, PAGE_SIZE,
					"REG RD: ERR=%d\n", ret);

		return snprintf(buf, PAGE_SIZE,
				"REG RD: 0x%02X=0x%02X\n",
				(inf >> 8) & 0xFF, val);

	default:
		break;
	}

	return -EINVAL;
}

static struct nvs_fn_dev bmi_fn_dev = {
	.enable				= bmi_enable,
	.batch				= bmi_batch,
	.batch_read			= bmi_batch_read,
	.flush				= bmi_flush,
	.max_range			= bmi_max_range,
	.reset				= bmi_reset,
	.self_test			= bmi_selftest,
	.regs				= bmi_regs,
	.nvs_write			= bmi_nvs_write,
	.nvs_read			= bmi_nvs_read,
};

static int bmi_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	st->sts |= NVS_STS_SUSPEND;
	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			ret |= st->nvs->suspend(st->snsrs[i].nvs_st);
	}

	/* determine if operational during suspend */
	for (i = 0; i < st->hw_n; i++) {
		if ((st->enabled & (1 << i)) && (st->snsrs[i].cfg.flags &
						 SENSOR_FLAG_WAKE_UP))
			break;
	}
	if (i < st->hw_n) {
		irq_set_irq_wake(st->i2c->irq, 1);
		st->irq_set_irq_wake = true;
	}
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s WAKE_ON=%X\n",
			 __func__, st->irq_set_irq_wake);
	return ret;
}

static int bmi_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	if (st->irq_set_irq_wake) {
		irq_set_irq_wake(st->i2c->irq, 0);
		st->irq_set_irq_wake = false;
	}
	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			ret |= st->nvs->resume(st->snsrs[i].nvs_st);
	}

	st->sts &= ~NVS_STS_SUSPEND;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static SIMPLE_DEV_PM_OPS(bmi_pm_ops, bmi_suspend, bmi_resume);

static void bmi_shutdown(struct i2c_client *client)
{
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;

	st->sts |= NVS_STS_SHUTDOWN;
	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			st->nvs->shutdown(st->snsrs[i].nvs_st);
	}
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int bmi_remove(struct i2c_client *client)
{
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;

	if (st != NULL) {
		bmi_shutdown(client);
		if (st->irq_setup_done && st->i2c->irq > 0)
			free_irq(st->i2c->irq, st);
		if (st->nvs) {
			for (i = 0; i < st->hw_n; i++)
				st->nvs->remove(st->snsrs[i].nvs_st);
		}

		if (st->wq) {
			destroy_workqueue(st->wq);
			st->wq = NULL;
		}
		bmi_pm_exit(st);
	}
	st->irq_setup_done = false; /* clear IRQ setup flag */
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int bmi_id_dev(struct bmi_state *st, const char *name)
{
	u8 val;
	int ret;

	ret = bmi_i2c_rd(st, BMI_REG_CHIP_ID, 1, &val);
	if (!ret) {
		if (val == BMI_REG_CHIP_ID_POR)
			dev_info(&st->i2c->dev, "%s %s found\n",
				 __func__, name);
		else
			dev_info(&st->i2c->dev, "%s %hhx response @ I2C=%x\n",
				 __func__, val, st->i2c->addr);
	}
	return ret;
}

static int bmi_id_i2c(struct bmi_state *st,
		      const struct i2c_device_id *id)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(bmi_i2c_addrs); i++) {
		if (st->i2c->addr == bmi_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(bmi_i2c_addrs)) {
		st->i2c_addr = st->i2c->addr;
		ret = bmi_id_dev(st, id->name);
	} else {
		for (i = 0; i < ARRAY_SIZE(bmi_i2c_addrs); i++) {
			st->i2c_addr = bmi_i2c_addrs[i];
			ret = bmi_id_dev(st, id->name);
			if (!ret)
				break;
		}
	}
	if (ret)
		st->i2c_addr = 0;
	return ret;
}

static int bmi_of_dt(struct bmi_state *st, struct device_node *dn)
{
	u32 tmp;

	/* driver specific defaults */
	if (st->i2c->irq > 0) {
		st->int_out_ctrl = 0x0D;
		st->int_latch = 0x00;
		st->int_map_0 = 0xFF;
		st->int_map_1 = 0xF0;
		st->int_map_2 = 0x00;
	}
	st->no_irq_no_wake_on = true; /* default */
	st->acc_conf = 0x20;
	st->gyr_conf = 0x20;
	if (dn) {
		/* driver specific device tree parameters */
		if (st->i2c->irq > 0) {
			if (!of_property_read_u32(dn, "int_out_ctrl", &tmp))
				st->int_out_ctrl = tmp;
			if (!of_property_read_u32(dn, "int_latch", &tmp))
				st->int_latch = tmp;
			if (!of_property_read_u32(dn, "int_map_0", &tmp))
				st->int_map_0 = tmp;
			if (!of_property_read_u32(dn, "int_map_1", &tmp))
				st->int_map_1 = tmp;
			if (!of_property_read_u32(dn, "int_map_2", &tmp))
				st->int_map_2 = tmp;
		}
		if (!of_property_read_u32(dn, "acc_conf", &tmp))
			st->acc_conf = tmp;
		if (!of_property_read_u32(dn, "gyr_conf", &tmp))
			st->gyr_conf = tmp;
		if (!of_property_read_u32(dn, "no_irq_no_wake_on", &tmp)) {
			if (tmp)
				st->no_irq_no_wake_on = true;
			else
				st->no_irq_no_wake_on = false;
		}
	}
	return 0;
}

static int bmi_init(struct bmi_state *st, const struct i2c_device_id *id)
{
	unsigned long irqflags;
	unsigned int i;
	unsigned int n;
	int lo;
	int hi;
	int ret;

	ret = bmi_of_dt(st, st->i2c->dev.of_node);
	if (ret) {
		dev_err(&st->i2c->dev, "%s _of_dt ERR\n", __func__);
		return ret;
	}

	bmi_pm_init(st);
	ret = bmi_id_i2c(st, id);
	if (ret) {
		dev_err(&st->i2c->dev, "%s _id_i2c ERR\n", __func__);
		return -ENODEV;
	}

	bmi_disable(st, -1); /* disable all devices and power down */
	bmi_fn_dev.errs = &st->errs;
	bmi_fn_dev.sts = &st->sts;
	st->nvs = nvs_auto(NVS_CFG_KIF);
	if (st->nvs == NULL) {
		dev_err(&st->i2c->dev, "%s nvs_ ERR\n", __func__);
		return -ENODEV;
	}

	n = 0;
	for (i = 0; i < BMI_HW_N; i++) {
		st->snsrs[n].hw = &bmi_hws[i];
		memcpy(&st->snsrs[n].cfg, &bmi_sensor_cfgs[i],
		       sizeof(st->snsrs[n].cfg));
		nvs_of_dt(st->i2c->dev.of_node, &st->snsrs[n].cfg, NULL);
		if (st->i2c->irq <= 0 && (st->snsrs[n].cfg.flags &
					  SENSOR_FLAG_WAKE_UP)) {
			/* no interrupt & SENSOR_FLAG_WAKE_UP so... */
			if (st->no_irq_no_wake_on) {
				/* disable WAKE_ON ability */
				st->snsrs[n].cfg.flags &= ~SENSOR_FLAG_WAKE_UP;
			} else {
				/* don't populate WAKE_ON sensor */
				st->hw2ids[i] = -1;
				continue;
			}
		}

		ret = st->nvs->probe(&st->snsrs[n].nvs_st, st, &st->i2c->dev,
				     &bmi_fn_dev, &st->snsrs[n].cfg);
		if (ret) {
			st->hw2ids[i] = -1;
		} else {
			st->hw2ids[i] = n;
			st->snsrs[n].cfg.snsr_id = n;
			bmi_max_range(st, n, st->snsrs[n].cfg.max_range.ival);
			n++;
		}
	}
	if (!n)
		return -ENODEV;

	st->hw_n = n;
	hi = 0;
	if (st->i2c->irq > 0) {
		if (st->int_out_ctrl & 0x22)
			irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		else
			irqflags = IRQF_TRIGGER_RISING;
		ret = request_threaded_irq(st->i2c->irq,
					   bmi_irq_handler, bmi_irq_thread,
					   irqflags, BMI_NAME, st);
		if (ret) {
			dev_err(&st->i2c->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			return -ENODEV;
		}
		st->irq_setup_done = true;

		for (i = 0; i < st->hw_n; i++) {
			if (st->snsrs[i].cfg.delay_us_max > hi)
				hi = st->snsrs[i].cfg.delay_us_max;
		}

		st->period_us_max = hi;
	} else {
		lo = ((int)(~0U >> 1));
		for (i = 0; i < st->hw_n; i++) {
			if (st->snsrs[i].cfg.delay_us_min > hi)
				hi = st->snsrs[i].cfg.delay_us_min;
			if (st->snsrs[i].cfg.delay_us_max < lo)
				lo = st->snsrs[i].cfg.delay_us_max;
		}

		st->period_us_max = lo;
		st->period_us = lo;
		for (i = 0; i < st->hw_n; i++) {
			st->snsrs[i].cfg.delay_us_min = hi;
			st->snsrs[i].cfg.delay_us_max = lo;
			st->snsrs[i].period_us = lo;
		}

		st->wq = create_workqueue(BMI_NAME);
		if (!st->wq) {
			dev_err(&st->i2c->dev, "%s create_workqueue ERR\n",
				__func__);
			return -ENODEV;
		}

		INIT_WORK(&st->ws, bmi_work);
	}

	return 0;
}

static int bmi_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct bmi_state *st;
	int ret;

	/* just test if global disable */
	ret = nvs_of_dt(client->dev.of_node, NULL, NULL);
	if (ret == -ENODEV) {
		dev_info(&client->dev, "%s DT disabled\n", __func__);
		return -ENODEV;
	}

	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = bmi_init(st, id);
	if (ret)
		bmi_remove(client);
	dev_info(&client->dev, "%s done\n", __func__);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	device_resource_registered();
#else
	device_unblock_probing();
#endif
	return ret;
}

static const struct i2c_device_id bmi_i2c_device_id[] = {
	{ BMI_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, bmi_i2c_device_id);

static const struct of_device_id bmi_of_match[] = {
	{ .compatible = "bmi,bmi160", },
	{}
};

MODULE_DEVICE_TABLE(of, bmi_of_match);

static struct i2c_driver bmi_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= bmi_probe,
	.remove				= bmi_remove,
	.shutdown			= bmi_shutdown,
	.driver				= {
		.name			= BMI_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(bmi_of_match),
		.pm			= &bmi_pm_ops,
	},
	.id_table			= bmi_i2c_device_id,
};

module_i2c_driver(bmi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMI160 driver");
MODULE_AUTHOR("NVIDIA Corporation");

