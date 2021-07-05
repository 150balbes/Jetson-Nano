/* Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

/* This driver supports these Texas Instruments temperature sensors:
 * - TMP102
 * - TMP112
 * - TMP75
 * - TMP175
 * - TMP275
 * This driver supports multiple devices at the same time. See device tree
 * instructions below for how.
 */

/* Device tree example:
 * This driver has an auto-detection mechanism that will interrogate the I2C
 * bus using the tmp_i2c_addrs addresses. To enable this mechanism the device
 * tree must have the following:
 * tmp1x2@48 {
 *   compatible = "ti,tmp1x2";
 * ...
 *
 * To disable the auto-detection, specify the actual part:
 * tmp102@48 {
 *   compatible = "ti,tmp102";
 *
 * When specifying the actual part, a list of "register_configuration_0x??"
 * must be defined for each temperature sensor where the ?? is the I2C address
 * and the value what the sensor's configuration register will be programmed
 * with. For example:
 * tmp102@48 {
 *   compatible = "ti,tmp102";
 *   reg = <0x48>;
 *   register_configuration_0x48 = <0x0000>;
 *   register_configuration_0x4A = <0x0002>;
 *   register_configuration_0x4C = <0x0018>;
 *   register_configuration_0x4D = <0x0000>;
 *   register_configuration_0x4E = <0x0000>;
 *   irq_gpio = <&tegra_gpio TEGRA_GPIO(AA, 2) GPIO_ACTIVE_HIGH>;
 *   smbus_ara_enable = <1>;
 *   vcc-supply = <&spmic_sd3>;
 * };
 *
 * If the configurations register TM bit is clear, the sensor will be in
 * Thermostat Mode and will be enabled with thresholds programmed but not
 * monitored by the driver. It is expected that the ASSERT pin will be
 * connected to some HW thermal management.
 * If no configuration register is defined using the auto-detection mechanism,
 * the TM bit defaults to enabled (interrupt mode).
 * If no interrupt is defined, the sensors will be polled instead.
 * When the smbus_ara_enable is enabled, the driver will use the GPIO as an
 * SMBUS ARA signal. The irq_gpio must be defined.
 */


#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/nvs.h>
#include <linux/nvs_on_change.h>

#define TMP_DRIVER_VERSION		(1)
#define TMP_VERSION			(1) /* reported version */
#define TMP_VENDOR			"Texas Instruments"
#define TMP_NAME			"tmp1x2"
#define TMP_NAME_TMP102			"tmp102"
#define TMP_NAME_TMP112			"tmp112"
#define TMP_NAME_TMPx75			"tmpx75"
#define TMP_NAME_TMP75			"tmp75"
#define TMP_NAME_TMP175			"tmp175"
#define TMP_NAME_TMP275			"tmp275"
/* the tmp1x2 has extended features. Any other PART value (> 0) will default
 * to the backward compatible standard features (TMP_REG_CFG = 8 bits).
 */
#define TMP_PART_TMP1x2			(0)
#define TMP_PART_TMPx75			(1)
#define TMP_PART_AUTO			(2)
#define TMP_POR_DELAY_MS		(1)
/* HW registers */
#define TMP_REG_TEMP			(0)
#define TMP_REG_CFG			(1)
#define TMP_REG_CFG_SD			(0)
#define TMP_REG_CFG_SD_MSK		(0x0001)
#define TMP_REG_CFG_TM			(1)
#define TMP_REG_CFG_TM_MSK		(0x0002)
#define TMP_REG_CFG_POL			(2)
#define TMP_REG_CFG_POL_MSK		(0x0004)
#define TMP_REG_CFG_FAULT		(3)
#define TMP_REG_CFG_FAULT_MSK		(0x0018)
#define TMP_REG_CFG_RES			(5)
#define TMP_REG_CFG_RES_MSK		(0x0060)
#define TMP_REG_CFG_OS			(7)
#define TMP_REG_CFG_OS_MSK		(0x0080)
#define TMP_REG_CFG_EM			(12)
#define TMP_REG_CFG_EM_MSK		(0x1000)
#define TMP_REG_CFG_AL			(13)
#define TMP_REG_CFG_AL_MSK		(0x2000)
#define TMP_REG_CFG_CR			(14)
#define TMP_REG_CFG_CR_MSK		(0xC000)
#define TMP_REG_LO			(2)
#define TMP_REG_HI			(3)
#define TMP_REG_N			(4)
/* device capability could use the TMP_I2C_ARA to clear the INT pin */
#define TMP_I2C_ARA			(0x0C) /* I2C Alert Response Address */
/* strings */
#define TMP_STR_REG_CFG			"register_configuration"
#define TMP_STR_SMB_ARA_EN		"smbus_ara_enable"

/* regulator names in order of powering on */
static const char *const tmp_vregs[] = {
	"vcc",
};

static unsigned short tmp_i2c_addrs[] = {
	0x48,
	0x49,
	0x4A,
	0x4B,
	0x4C,
	0x4D,
	0x4E,
	0x4F,
};
#define TMP_HW_N			ARRAY_SIZE(tmp_i2c_addrs)

struct tmp_sensor {
	void *nvs_st;			/* NVS handle */
	struct sensor_cfg cfg;		/* NVS configuration */
	struct nvs_on_change oc;	/* on-change temperature sensor*/
	char part[16];			/* part string */
};

struct tmp_state {
	struct i2c_client *ara;
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	struct regulator_bulk_data vreg[ARRAY_SIZE(tmp_vregs)];
	struct work_struct ws;
	struct workqueue_struct *wq;
	struct tmp_sensor *snsr;
	int irq_gpio;			/* interrupt GPIO */
	unsigned int sts;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int poll_period_ms;	/* global polling period */
	unsigned int poll_en;		/* poll enabled bit mask */
	unsigned int pm_en;		/* pm enabled bit mask */
	unsigned int enabled;		/* sensor enabled bit mask */
	unsigned int snsr_n;		/* sensor count */
	u16 r_cfg[TMP_HW_N];		/* configuration register default */
	u16 i2c_addr[TMP_HW_N];		/* I2C address */
	u8 part[TMP_HW_N];		/* TMP_PART_TMP? */
};

enum TMP_DBG {
	TMP_DBG_STS = 0,
	/* skip sequence to "hide" debug features */
	TMP_DBG_RD = 2,
	TMP_DBG_ARA,
	TMP_DBG_REG,
};

struct tmp_resolution_ms {
	struct nvs_float resolution;
	unsigned int period_ms;
};

struct tmp_resolution_part {
	struct tmp_resolution_ms *resolution_ms;
	unsigned int resolution_0n;
};

struct tmp_max_range_part {
	struct nvs_float *max_range;
	unsigned int max_range_0n;
};

static struct nvs_float tmp_max_range_1x2[] = {
	{
		.ival			= 128,
		.fval			= 0,
	},
	{
		.ival			= 150,
		.fval			= 0,
	},
};

static struct nvs_float tmp_max_range_x75[] = {
	{
		.ival			= 128,
		.fval			= 0,
	},
};

static struct tmp_resolution_ms tmp_resolution_ms_1x2[] = {
	{
		.resolution		= {
			.ival		= 0,
			.fval		= 62500,
		},
		.period_ms		= 35,
	},
};

static struct tmp_resolution_ms tmp_resolution_ms_x75[] = {
	{
		.resolution		= {
			.ival		= 0,
			.fval		= 500000,
		},
		.period_ms		= 38,
	},
	{
		.resolution		= {
			.ival		= 0,
			.fval		= 250000,
		},
		.period_ms		= 75,
	},
	{
		.resolution		= {
			.ival		= 0,
			.fval		= 125000,
		},
		.period_ms		= 150,
	},
	{
		.resolution		= {
			.ival		= 0,
			.fval		= 62500,
		},
		.period_ms		= 300,
	},
};

static struct tmp_max_range_part tmp_max_range_parts[] = {
	{
		.max_range		= tmp_max_range_1x2,
		.max_range_0n		= ARRAY_SIZE(tmp_max_range_1x2) - 1,
	},
	{
		.max_range		= tmp_max_range_x75,
		.max_range_0n		= ARRAY_SIZE(tmp_max_range_x75) - 1,
	},
};

static struct tmp_resolution_part tmp_resolution_parts[] = {
	{
		.resolution_ms		= tmp_resolution_ms_1x2,
		.resolution_0n	       = ARRAY_SIZE(tmp_resolution_ms_1x2) - 1,
	},
	{
		.resolution_ms		= tmp_resolution_ms_x75,
		.resolution_0n	       = ARRAY_SIZE(tmp_resolution_ms_x75) - 1,
	},
};

static const struct i2c_device_id tmp_i2c_device_id[] = {
	{ TMP_NAME_TMP102, TMP_PART_TMP1x2 },
	{ TMP_NAME_TMP112, TMP_PART_TMP1x2 },
	{ TMP_NAME_TMP75, TMP_PART_TMPx75 },
	{ TMP_NAME_TMP175, TMP_PART_TMPx75 },
	{ TMP_NAME_TMP275, TMP_PART_TMPx75 },
	{ TMP_NAME_TMPx75, TMP_PART_AUTO },
	{ TMP_NAME, TMP_PART_AUTO },
	{},
};

static const char *tmp_part_name[] = {
	TMP_NAME,
	TMP_NAME_TMPx75,
};

static struct sensor_cfg tmp_cfg_dflt = {
	.name				= "ambient_temperature",
	.ch_n				= 1,
	.ch_sz				= -2,
	.part				= TMP_NAME,
	.vendor				= TMP_VENDOR,
	.version			= TMP_VERSION,
	.milliamp			= {
		.ival			= 0,
		.fval			= 10000,
	},
	.flags				= SENSOR_FLAG_ON_CHANGE_MODE,
	.thresh_lo			= 2,
	.thresh_hi			= 2,
};


static void tmp_err(struct tmp_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

/* The device may not respond to I2C until the IRQ is ACK'd with the ARA.
 * The problem is that ARA may be needed regardless of whether the IRQ is
 * enabled or not so we test the IRQ GPIO before each I2C transaction as a WAR.
 */
static int tmp_ara_ack(struct tmp_state *st, unsigned int snsr_id, bool force)
{
	int gpio_sts;
	int ret = 0;

	if (st->ara && st->irq_gpio >= 0) {
		if (force)
			gpio_sts = false;
		else
			gpio_sts = gpio_get_value(st->irq_gpio);
		if (!gpio_sts)
			ret = i2c_smbus_read_byte(st->ara);
		if (st->sts & NVS_STS_SPEW_IRQ)
			dev_info(&st->i2c->dev,
				 "%s_0x%02X %s ARA GPIO %d=%d ret=%d\n",
				 tmp_part_name[st->part[snsr_id]],
				 st->i2c_addr[snsr_id], __func__,
				 st->irq_gpio, gpio_sts, ret);
	}
	/* ret < 0: error
	 * ret = 0: no action
	 * ret > 0: ACK
	 */
	return ret;
}

static int tmp_i2c_r(struct tmp_state *st, unsigned int snsr_id,
		     u8 reg, u16 len, u8 *buf)
{
	struct i2c_msg msg[2];
	int ret = -ENODEV;

	if (st->i2c_addr[snsr_id]) {
		tmp_ara_ack(st, snsr_id, false);
		msg[0].addr = st->i2c_addr[snsr_id];
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &reg;
		msg[1].addr = st->i2c_addr[snsr_id];
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = buf;
		ret = i2c_transfer(st->i2c->adapter, msg, 2);
		if (ret != 2 && st->ara) {
			tmp_ara_ack(st, snsr_id, true);
			ret = i2c_transfer(st->i2c->adapter, msg, 2);
		}
		if (ret == 2) {
			ret = 0;
		} else {
			tmp_err(st);
			ret = -EIO;
		}
	}
	return ret;
}

static int tmp_i2c_rd(struct tmp_state *st, unsigned int snsr_id,
		      u8 reg, u16 *val)
{
	u8 buf[2];
	u16 len = sizeof(buf);
	u16 val16;
	int ret = -ENODEV;

	if (snsr_id < TMP_HW_N) {
		if ((reg == TMP_REG_CFG) && st->part[snsr_id])
			len--; /* backward compatible TMP_REG_CFG is 8 bits */
		ret = tmp_i2c_r(st, snsr_id, reg, len, buf);
		if (ret) {
			dev_err(&st->i2c->dev, "%s_0x%02X %s ERR: r0x%02X\n",
				tmp_part_name[st->part[snsr_id]],
				st->i2c_addr[snsr_id], __func__, reg);
		} else {
			if ((reg == TMP_REG_CFG) && st->part[snsr_id]) {
				buf[1] = buf[0];
				buf[0] = 0;
			}
			val16 = buf[0];
			val16 <<= 8;
			val16 |= buf[1];
			*val = val16;
			if (st->sts & NVS_STS_SPEW_MSG)
				dev_info(&st->i2c->dev,
					 "%s_0x%02X %s r0x%02X=>d0x%04X\n",
					 tmp_part_name[st->part[snsr_id]],
					 st->i2c_addr[snsr_id],
					 __func__, reg, val16);
		}
	}
	return ret;
}

static int tmp_i2c_w(struct tmp_state *st, unsigned int snsr_id,
		     u16 len, u8 *buf)
{
	struct i2c_msg msg;
	int ret = -ENODEV;

	if (st->i2c_addr[snsr_id]) {
		tmp_ara_ack(st, snsr_id, false);
		msg.addr = st->i2c_addr[snsr_id];
		msg.flags = 0;
		msg.len = len;
		msg.buf = buf;
		ret = i2c_transfer(st->i2c->adapter, &msg, 1);
		if (ret != 1 && st->ara) {
			tmp_ara_ack(st, snsr_id, true);
			ret = i2c_transfer(st->i2c->adapter, &msg, 1);
		}
		if (ret == 1) {
			ret = 0;
		} else {
			tmp_err(st);
			ret = -EIO;
		}
	}
	return ret;
}

static int tmp_i2c_wr(struct tmp_state *st, unsigned int snsr_id,
		      u8 reg, u16 val)
{
	u8 buf[3];
	u16 len = sizeof(buf);
	int ret = -ENODEV;

	if (snsr_id < TMP_HW_N) {
		buf[0] = reg;
		if ((reg == TMP_REG_CFG) && st->part[snsr_id]) {
			buf[1] = (u8)(val & 0xFF);
			len--;
		} else {
			buf[2] = (u8)(val & 0xFF);
			buf[1] = (u8)(val >> 8);
		}
		ret = tmp_i2c_w(st, snsr_id, len, buf);
		if (ret)
			dev_err(&st->i2c->dev,
				"%s_0x%02X %s ERR: d0x%04X=>r0x%02X\n",
				tmp_part_name[st->part[snsr_id]],
				st->i2c_addr[snsr_id], __func__, val, reg);
		else if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev,
				 "%s_0x%02X %s d0x%04X=>r0x%02X\n",
				 tmp_part_name[st->part[snsr_id]],
				 st->i2c_addr[snsr_id], __func__, val, reg);
	}
	return ret;
}

static int tmp_pm(struct tmp_state *st, int snsr_id, int en)
{
	unsigned int pm_en = st->pm_en;
	unsigned int i;
	unsigned int msk;
	int ret = 0;

	if (en) {
		if (!st->pm_en) {
			ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
					       ARRAY_SIZE(tmp_vregs));
			if (ret > 0)
				mdelay(TMP_POR_DELAY_MS);
		}
		if (ret >= 0) {
			if (snsr_id < 0) {
				for (i = 0; i < st->snsr_n; i++)
					st->pm_en |= (1 << i);
			} else {
				st->pm_en |= (1 << snsr_id);
			}
		}
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(tmp_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(tmp_vregs))) {
			/* we're fully powered */
			if (snsr_id < 0) {
				ret = 0;
				for (i = 0; i < st->snsr_n; i++)
					ret |= tmp_i2c_wr(st, i, TMP_REG_CFG,
							  st->r_cfg[i] |
							  TMP_REG_CFG_SD_MSK);
				st->poll_en = 0;
				st->enabled = 0;
				st->pm_en = 0;
			} else {
				ret = tmp_i2c_wr(st, snsr_id, TMP_REG_CFG,
						 st->r_cfg[snsr_id] |
						 TMP_REG_CFG_SD_MSK);
				msk = ~(1 << snsr_id);
				st->poll_en &= msk;
				st->enabled &= msk;
				st->pm_en &= msk;
			}
		} else if (ret > 0) {
			/* partially powered so go to full before disables */
			ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
					       ARRAY_SIZE(tmp_vregs));
			mdelay(TMP_POR_DELAY_MS);
			if (snsr_id < 0) {
				ret = 0;
				for (i = 0; i < st->snsr_n; i++)
					ret |= tmp_i2c_wr(st, i, TMP_REG_CFG,
							  st->r_cfg[i] |
							  TMP_REG_CFG_SD_MSK);
				st->poll_en = 0;
				st->enabled = 0;
				st->pm_en = 0;
			} else {
				ret = tmp_i2c_wr(st, snsr_id, TMP_REG_CFG,
						 st->r_cfg[snsr_id] |
						 TMP_REG_CFG_SD_MSK);
				msk = ~(1 << snsr_id);
				st->poll_en &= msk;
				st->enabled &= msk;
				st->pm_en &= msk;
			}
		}
		/* disables put us in low power sleep state in case no vregs */
		if (!st->pm_en) /* pull plug if all devices are low power */
			ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
						 ARRAY_SIZE(tmp_vregs));
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		if (snsr_id < 0)
			dev_err(&st->i2c->dev, "%s ALL en=%x  ERR=%d\n",
				__func__, en, ret);
		else
			dev_err(&st->i2c->dev, "%s_0x%02X %s en=%x  ERR=%d\n",
				tmp_part_name[st->part[snsr_id]],
				st->i2c_addr[snsr_id], __func__, en, ret);
	} else if (st->sts & NVS_STS_SPEW_MSG && pm_en != st->pm_en) {
		dev_info(&st->i2c->dev, "%s pm_en: 0x%X=>0x%XS\n",
			 __func__, pm_en, st->pm_en);
	}
	return ret;
}

static int tmp_pm_exit(struct tmp_state *st)
{
	int ret;

	ret = tmp_pm(st, -1, 0);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(tmp_vregs));
	return ret;
}

static int tmp_pm_init(struct tmp_state *st)
{
	int ret;

	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(tmp_vregs), (char **)tmp_vregs);
	ret = tmp_pm(st, -1, 1);
	return ret;
}

static int tmp_poll_period(struct tmp_state *st)
{
	unsigned int i;
	unsigned int poll_period_ms = -1;

	for (i = 0; i < st->snsr_n; i++) {
		if (st->poll_en & (1 << i)) {
			if (st->snsr[i].oc.poll_period_ms < poll_period_ms)
				poll_period_ms = st->snsr[i].oc.poll_period_ms;
		}
	}

	st->poll_period_ms = poll_period_ms;
	return 0;
}

static int tmp_thr_wr(struct tmp_state *st, unsigned int snsr_id,
		      u16 thr_lo, u16 thr_hi)
{
	int ret;

	ret = tmp_i2c_wr(st, snsr_id, TMP_REG_LO, thr_lo);
	ret |= tmp_i2c_wr(st, snsr_id, TMP_REG_HI, thr_hi);
	return ret;
}

static int tmp_cfg_wr(struct tmp_state *st, unsigned int snsr_id, int sts)
{
	u8 r_cfg = st->r_cfg[snsr_id];
	unsigned int poll_en = st->poll_en;
	int ret = 0;

	r_cfg &= ~TMP_REG_CFG_SD_MSK; /* disable shutdown mode */
	if (r_cfg & TMP_REG_CFG_TM_MSK) {
		/* We're in thermostat mode, which is interrupt mode */
		if (sts == RET_HW_UPDATE) {
			if (st->i2c->irq > 0) {
				ret = tmp_thr_wr(st, snsr_id,
					     st->snsr[snsr_id].oc.hw_thresh_lo,
					    st->snsr[snsr_id].oc.hw_thresh_hi);
				if (!ret) {
					st->poll_en &= ~(1 << snsr_id);
					ret = 1; /* flag IRQ enabled */
				}
			} else {
				st->poll_en |= (1 << snsr_id);
			}
		} else if (sts < 0) { /* RET_POLL_NEXT or error */
			ret = tmp_i2c_wr(st, snsr_id, TMP_REG_CFG, r_cfg);
			st->poll_en |= (1 << snsr_id);
		} /* else ret == RET_NO_CHANGE */
		if (st->poll_en && !poll_en) {
			/* start the polling thread */
			tmp_poll_period(st);
			cancel_work_sync(&st->ws);
			queue_work(st->wq, &st->ws);
		} else if (st->poll_en) {
			tmp_poll_period(st);
		}
	} else {
		/* in comparator mode: HW driven ALERT pin (do nothing) */
		ret = tmp_i2c_wr(st, snsr_id, TMP_REG_CFG, r_cfg);
	}
	return ret;
}

static int tmp_rd(struct tmp_state *st, unsigned int snsr_id)
{
	u16 hw;
	s64 ts;
	int ret;

	ts = nvs_timestamp();
	ret = tmp_i2c_rd(st, snsr_id, TMP_REG_TEMP, &hw);
	if (!ret) {
		if (st->sts & NVS_STS_SPEW_DATA)
			dev_info(&st->i2c->dev,
				 "%s %hu %lld   diff=%d %lldns\n",
				 st->snsr[snsr_id].cfg.name, hw, ts,
				 hw - st->snsr[snsr_id].oc.hw,
				 ts - st->snsr[snsr_id].oc.timestamp);
		st->snsr[snsr_id].oc.hw = hw;
		st->snsr[snsr_id].oc.timestamp = ts;
		ret = nvs_on_change_read(&st->snsr[snsr_id].oc);
		ret = tmp_cfg_wr(st, snsr_id, ret);
	}
	return ret;
}

static int tmp_read(struct tmp_state *st, unsigned int snsr_id)
{
	int ret;

	st->nvs->nvs_mutex_lock(st->snsr[snsr_id].nvs_st);
	ret = tmp_rd(st, snsr_id);
	st->nvs->nvs_mutex_unlock(st->snsr[snsr_id].nvs_st);
	return ret;
}

static void tmp_work(struct work_struct *ws)
{
	struct tmp_state *st = container_of((struct work_struct *)ws,
					    struct tmp_state, ws);
	unsigned int i;

	while (st->poll_en) {
		msleep(st->poll_period_ms);
		for (i = 0; i < st->snsr_n; i++) {
			if (st->poll_en & (1 << i))
				tmp_read(st, i);
		}
	}
}

static irqreturn_t tmp_irq_thread(int irq, void *dev_id)
{
	struct tmp_state *st = (struct tmp_state *)dev_id;
	unsigned int i;

	for (i = 0; i < st->snsr_n; i++) {
		if (st->enabled & (1 << i))
			tmp_read(st, i);
	}

	return IRQ_HANDLED;
}

static irqreturn_t tmp_irq_handler(int irq, void *dev_id)
{
	struct tmp_state *st = (struct tmp_state *)dev_id;

	if (st->sts & NVS_STS_SPEW_IRQ)
		dev_info(&st->i2c->dev, "%s\n", __func__);
	return IRQ_WAKE_THREAD;
}

static int tmp_enable(void *client, int snsr_id, int enable)
{
	struct tmp_state *st = (struct tmp_state *)client;
	int ret;

	if (enable < 0)
		return st->enabled & (1 << snsr_id);

	if (enable) {
		enable = st->enabled | (1 << snsr_id);
		ret = tmp_pm(st, snsr_id, 1);
		if (!ret) {
			ret = nvs_on_change_enable(&st->snsr[snsr_id].oc);
			ret |= tmp_cfg_wr(st, snsr_id, RET_POLL_NEXT);
			if (ret < 0)
				tmp_pm(st, snsr_id, 0);
			else
				st->enabled = enable;
		}
	} else {
		ret = tmp_pm(st, snsr_id, 0);
	}
	return ret;
}

static int tmp_batch(void *client, int snsr_id, int flags,
		     unsigned int period_us, unsigned int timeout_us)
{
	struct tmp_state *st = (struct tmp_state *)client;

	if (timeout_us)
		/* timeout not supported (no HW FIFO) */
		return -EINVAL;

	st->snsr[snsr_id].oc.period_us = period_us;
	return 0;
}

static int tmp_max_range(void *client, int snsr_id, int max_range)
{
	struct tmp_state *st = (struct tmp_state *)client;
	unsigned int i = st->part[snsr_id];
	unsigned int j = max_range;

	if (st->enabled & (1 << snsr_id))
		/* can't change settings on the fly (disable device first) */
		return -EBUSY;

	if (j > tmp_max_range_parts[i].max_range_0n)
		/* clamp to highest setting */
		j = tmp_max_range_parts[i].max_range_0n;
	st->snsr[snsr_id].cfg.max_range.ival =
				      tmp_max_range_parts[i].max_range[j].ival;
	st->snsr[snsr_id].cfg.max_range.fval =
				      tmp_max_range_parts[i].max_range[j].fval;
	st->r_cfg[snsr_id] &= ~TMP_REG_CFG_EM_MSK;
	st->r_cfg[snsr_id] |= j << TMP_REG_CFG_EM;
	return 0;
}

static int tmp_resolution(void *client, int snsr_id, int resolution)
{
	struct tmp_state *st = (struct tmp_state *)client;
	unsigned int i = st->part[snsr_id];
	unsigned int j = resolution;

	if (st->enabled & (1 << snsr_id))
		/* can't change settings on the fly (disable device first) */
		return -EBUSY;

	if (j > tmp_resolution_parts[i].resolution_0n)
		/* clamp to highest setting */
		j = tmp_resolution_parts[i].resolution_0n;
	st->snsr[snsr_id].cfg.resolution.ival =
		      tmp_resolution_parts[i].resolution_ms[j].resolution.ival;
	st->snsr[snsr_id].cfg.resolution.fval =
		      tmp_resolution_parts[i].resolution_ms[j].resolution.fval;
	st->snsr[snsr_id].cfg.scale.ival =
		      tmp_resolution_parts[i].resolution_ms[j].resolution.ival;
	st->snsr[snsr_id].cfg.scale.fval =
		      tmp_resolution_parts[i].resolution_ms[j].resolution.fval;
	st->r_cfg[snsr_id] &= ~TMP_REG_CFG_RES_MSK;
	st->r_cfg[snsr_id] |= j << TMP_REG_CFG_RES;
	return 0;
}

static int tmp_thresh_lo(void *client, int snsr_id, int thresh_lo)
{
	struct tmp_state *st = (struct tmp_state *)client;
	int ret;

	ret = nvs_on_change_threshold_lo(&st->snsr[snsr_id].oc, thresh_lo);
	return ret;
}

static int tmp_thresh_hi(void *client, int snsr_id, int thresh_hi)
{
	struct tmp_state *st = (struct tmp_state *)client;
	int ret;

	ret = nvs_on_change_threshold_hi(&st->snsr[snsr_id].oc, thresh_hi);
	return ret;
}

static int tmp_regs(void *client, int snsr_id, char *buf)
{
	struct tmp_state *st = (struct tmp_state *)client;
	ssize_t t;
	u16 val;
	u8 i;
	int ret;

	t = snprintf(buf, PAGE_SIZE, "registers:\n");
	for (i = 0; i < TMP_REG_N; i++) {
		ret = tmp_i2c_rd(st, snsr_id, i, &val);
		if (ret)
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "%#2x=ERR: %d\n", i, ret);
		else
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "%#2x=%#4x\n", i, val);
	}
	return t;
}

static int tmp_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct tmp_state *st = (struct tmp_state *)client;
	u16 val;
	u8 reg;
	int ret;

	switch (nvs & 0xFF) {
	case TMP_DBG_STS:
		return 0;

	case TMP_DBG_RD:
		tmp_read(st, snsr_id);
		dev_info(&st->i2c->dev, "%s tmp_read done\n", __func__);
		return 0;

	case TMP_DBG_ARA:
		if (st->ara) {
			ret = i2c_smbus_read_byte(st->ara);
			dev_info(&st->i2c->dev, "SMBus ARA=%d\n", ret);
		} else {
			dev_info(&st->i2c->dev, "SMBus ARA=N/A\n");
		}
		return 0;

	case TMP_DBG_REG:
		reg = (nvs >> 24) & 0xFF;
		val = (nvs >> 8) & 0xFFFF;
		ret = tmp_i2c_wr(st, snsr_id, reg, val);
		dev_info(&st->i2c->dev, "%s %hx=>%hhx  err=%d\n",
			 __func__, val, reg, ret);
		return ret;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tmp_nvs_read(void *client, int snsr_id, char *buf)
{
	struct tmp_state *st = (struct tmp_state *)client;
	ssize_t t;
	unsigned int i;
	int ret;

	t = snprintf(buf, PAGE_SIZE, "driver v.%u\n", TMP_DRIVER_VERSION);
	t += snprintf(buf + t, PAGE_SIZE - t, "i2c_addr=0x%X\n",
		      st->i2c_addr[snsr_id]);
	t += snprintf(buf + t, PAGE_SIZE - t, "device IRQ=%d\n", st->i2c->irq);
	t += snprintf(buf + t, PAGE_SIZE - t, "DEVICE TREE:\n");
	for (i = 0; i < st->snsr_n; i++)
		t += snprintf(buf + t, PAGE_SIZE - t, "%s_0x%02X = <0x%X>\n",
			      TMP_STR_REG_CFG, st->i2c_addr[i], st->r_cfg[i]);

	if (st->ara) {
		t += snprintf(buf + t, PAGE_SIZE - t, "%s=1\n",
			      TMP_STR_SMB_ARA_EN);
		ret = gpio_get_value(st->irq_gpio);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "irq_gpio %d=%d\n", st->irq_gpio, ret);
	} else {
		t += snprintf(buf + t, PAGE_SIZE - t, "%s=0\n",
			      TMP_STR_SMB_ARA_EN);
	}
	return t;
}

static struct nvs_fn_dev tmp_fn_dev = {
	.enable				= tmp_enable,
	.batch				= tmp_batch,
	.max_range			= tmp_max_range,
	.resolution			= tmp_resolution,
	.thresh_lo			= tmp_thresh_lo,
	.thresh_hi			= tmp_thresh_hi,
	.regs				= tmp_regs,
	.nvs_write			= tmp_nvs_write,
	.nvs_read			= tmp_nvs_read,
};

#ifdef CONFIG_SUSPEND
static int tmp_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	if (st->nvs) {
		for (i = 0; i < st->snsr_n; i++)
			ret |= st->nvs->suspend(st->snsr[i].nvs_st);
	}

	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int tmp_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	if (st->nvs) {
		for (i = 0; i < st->snsr_n; i++)
			ret |= st->nvs->resume(st->snsr[i].nvs_st);
	}

	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(tmp_pm_ops, tmp_suspend, tmp_resume);

static void tmp_shutdown(struct i2c_client *client)
{
	struct tmp_state *st = i2c_get_clientdata(client);
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < st->snsr_n; i++)
			st->nvs->shutdown(st->snsr[i].nvs_st);
	}

	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int tmp_remove(struct i2c_client *client)
{
	struct tmp_state *st = i2c_get_clientdata(client);
	unsigned int i;

	if (st != NULL) {
		tmp_shutdown(client);
		if (st->nvs) {
			for (i = 0; i < st->snsr_n; i++)
				st->nvs->remove(st->snsr[i].nvs_st);
		}

		if (st->wq) {
			destroy_workqueue(st->wq);
			st->wq = NULL;
		}
		tmp_pm_exit(st);
		if (st->ara)
			i2c_unregister_device(st->ara);
		if (st->snsr) {
			devm_kfree(&st->i2c->dev, st->snsr);
			st->snsr = NULL;
		}
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int tmp_id_dev(struct tmp_state *st, unsigned int hw)
{
	u16 val;
	int ret;

	ret = tmp_i2c_rd(st, hw, TMP_REG_CFG, &val);
	if (!ret) {
		if (val == TMP_REG_CFG_RES_MSK)
			st->part[hw] = TMP_PART_TMP1x2;
		else if (val == 0)
			st->part[hw] = TMP_PART_TMPx75;
		else
			ret = -ENODEV;
	}
	return ret;
}

static int tmp_init(struct tmp_state *st, const struct i2c_device_id *id)
{
	u32 val_u32;
	char str[48];
	unsigned long irqflags;
	unsigned int i;
	unsigned int j;
	unsigned int n;
	int ret;

	/* default device specific parameters */
	st->irq_gpio = -1;
	/* device tree parameters */
	st->irq_gpio = of_get_named_gpio(st->i2c->dev.of_node, "irq_gpio", 0);
	if (st->irq_gpio >= 0) {
		if (!gpio_is_valid(st->irq_gpio))
			return -EPROBE_DEFER;

		ret = gpio_request(st->irq_gpio, TMP_NAME);
		if (ret) {
			dev_err(&st->i2c->dev,
				"%s gpio_request(%d %s) ERR:%d\n",
				__func__, st->irq_gpio, TMP_NAME, ret);
			return -EPROBE_DEFER;
		}

		ret = gpio_direction_input(st->irq_gpio);
		if (ret < 0) {
			dev_err(&st->i2c->dev,
				"%s gpio_dir_inp(%d) ERR:%d\n",
				__func__, st->irq_gpio, ret);
			return -ENODEV;
		}

		if (!of_property_read_u32(st->i2c->dev.of_node,
					  TMP_STR_SMB_ARA_EN, &val_u32)) {
			if (val_u32) {
				st->ara = i2c_new_dummy(st->i2c->adapter,
							TMP_I2C_ARA);
				if (!st->ara) {
					/* must have ARA control for IRQ ack */
					dev_err(&st->i2c->dev,
						"%s ERR: i2c_new_dummy\n",
						__func__);
					return -ENODEV;
				}
			}
		}
	}

	tmp_pm_init(st); /* power up */
	n = 0;
	if (id->driver_data >= TMP_PART_AUTO) {
		for (i = 0; i < TMP_HW_N; i++) {
			st->i2c_addr[n] = tmp_i2c_addrs[i];
			ret = tmp_id_dev(st, n);
			if (ret) {
				st->i2c_addr[n] = 0;
			} else {
				dev_info(&st->i2c->dev,
					 "%s found %s part %s\n",
					 __func__, tmp_cfg_dflt.name,
					 tmp_part_name[st->part[n]]);
				n++;
			}
		}

		for (i = 0; i < n; i++) {
			ret = snprintf(str, sizeof(str), "%s_0x%02X",
				       TMP_STR_REG_CFG, st->i2c_addr[i]);
			if (ret <= 0) {
				dev_err(&st->i2c->dev,
					"%s snprintf(%s_0x%02X)\n",
					__func__, TMP_STR_REG_CFG,
					st->i2c_addr[i]);
			} else {
				if (!of_property_read_u32(st->i2c->dev.of_node,
							  str, &val_u32)) {
					st->r_cfg[i] = (u16)val_u32;
					if (val_u32 & TMP_REG_CFG_AL_MSK)
						st->part[i] = TMP_PART_TMP1x2;
					else
						st->part[i] = TMP_PART_TMPx75;
				}
			}
		}
	} else {
		for (i = 0; i < TMP_HW_N; i++) {
			ret = snprintf(str, sizeof(str), "%s_0x%02X",
				       TMP_STR_REG_CFG, tmp_i2c_addrs[i]);
			if (ret <= 0) {
				dev_err(&st->i2c->dev,
					"%s snprintf(reg_cfg_0x%02X)\n",
					__func__, tmp_i2c_addrs[i]);
			} else {
				if (!of_property_read_u32(st->i2c->dev.of_node,
							  str, &val_u32)) {
					st->i2c_addr[n] = tmp_i2c_addrs[i];
					st->r_cfg[n] = (u16)val_u32;
					if (val_u32 & TMP_REG_CFG_AL_MSK)
						st->part[n] = TMP_PART_TMP1x2;
					else
						st->part[n] = TMP_PART_TMPx75;
					n++;
				}
			}
		}
	}
	if (!n)
		return -ENODEV;

	st->snsr_n = n;
	st->snsr = devm_kzalloc(&st->i2c->dev, (size_t)(n * sizeof(*st->snsr)),
				GFP_KERNEL);
	tmp_pm(st, -1, 0); /* lowest sleep state */
	tmp_fn_dev.errs = &st->errs;
	tmp_fn_dev.sts = &st->sts;
	st->nvs = nvs_auto(NVS_CFG_KIF);
	if (st->nvs == NULL) {
		dev_err(&st->i2c->dev, "%s nvs_auto ERR\n", __func__);
		return -ENODEV;
	}

	n = 0;
	for (i = 0; i < st->snsr_n; i++) {
		/* default NVS programmable parameters */
		memcpy(&st->snsr[n].cfg, &tmp_cfg_dflt,
		       sizeof(st->snsr[0].cfg));
		for (j = 0; j < TMP_HW_N; j++) {
			if (st->i2c_addr[n] == tmp_i2c_addrs[j])
				break;
		}
		ret = snprintf(st->snsr[n].part, sizeof(*st->snsr[n].part),
			       "%s_0x%02X",
			       tmp_part_name[st->part[n]], tmp_i2c_addrs[j]);
		if (ret > 0) {
			st->snsr[n].cfg.part = st->snsr[n].part;
		} else {
			st->snsr[n].cfg.part = tmp_part_name[st->part[n]];
			dev_err(&st->i2c->dev, "%s %s snprintf(%s_0x%02X)\n",
				tmp_part_name[st->part[n]], __func__,
				tmp_part_name[st->part[n]], tmp_i2c_addrs[i]);
		}
		ret = snprintf(str, sizeof(str), "%s_0x%02X",
			       tmp_cfg_dflt.name, tmp_i2c_addrs[j]);
		if (ret > 0) {
			nvs_of_dt(st->i2c->dev.of_node, &st->snsr[n].cfg, str);
		} else {
			nvs_of_dt(st->i2c->dev.of_node, &st->snsr[n].cfg, NULL);
			dev_err(&st->i2c->dev, "%s %s snprintf(%s_0x%02X)\n",
				st->snsr[n].part, __func__,
				tmp_cfg_dflt.name, tmp_i2c_addrs[i]);
		}
		ret = st->nvs->probe(&st->snsr[n].nvs_st, st, &st->i2c->dev,
				     &tmp_fn_dev, &st->snsr[n].cfg);
		if (ret) {
			st->snsr_n--;
			for (j = i; j < st->snsr_n; j++) {
				st->r_cfg[j] = st->r_cfg[j + 1];
				st->i2c_addr[j] = st->i2c_addr[j + 1];
				st->part[j] = st->part[j + 1];
			}
		} else {
			st->snsr[n].cfg.snsr_id = n;
			tmp_max_range(st, n, st->snsr[n].cfg.max_range.ival);
			tmp_resolution(st, n, st->snsr[n].cfg.resolution.ival);
			n++;
		}
	}
	if (!n)
		return -ENODEV;

	st->wq = create_workqueue(TMP_NAME);
	if (!st->wq) {
		dev_err(&st->i2c->dev, "%s create_workqueue ERR\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&st->ws, tmp_work);
	if (st->irq_gpio >= 0 && !st->i2c->irq)
		st->i2c->irq = gpio_to_irq(st->irq_gpio);
	if (st->i2c->irq) {
		if (st->snsr_n > 1) {
			irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		} else {
			if (st->r_cfg[0] & TMP_REG_CFG_POL_MSK)
				irqflags = IRQF_TRIGGER_RISING;
			else
				irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		}
		ret = request_threaded_irq(st->i2c->irq,
					   tmp_irq_handler, tmp_irq_thread,
					   irqflags, TMP_NAME, st);
		if (ret) {
			dev_err(&st->i2c->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			return -ENOMEM;
		}
	}

	for (i = 0; i < st->snsr_n; i++) {
		if (!(st->r_cfg[i] & TMP_REG_CFG_TM_MSK)) {
			/* HW driven comparator mode */
			st->snsr[i].oc.threshold = OC_THRESHOLDS_ABSOLUTE;
			tmp_pm(st, i, 1);
			tmp_thr_wr(st, i, st->snsr[i].cfg.thresh_lo,
				   st->snsr[i].cfg.thresh_hi);
			tmp_i2c_wr(st, i, TMP_REG_CFG, st->r_cfg[i]);
		}
	}

	return 0;
}

static int tmp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmp_state *st;
	int ret;

	/* just test if global disable */
	ret = nvs_of_dt(client->dev.of_node, NULL, NULL);
	if (ret == -ENODEV) {
		dev_info(&client->dev, "%s DT disabled\n", __func__);
		return ret;
	}

	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = tmp_init(st, id);
	if (ret)
		tmp_remove(client);
	dev_info(&client->dev, "%s done\n", __func__);
	device_unblock_probing();
	return ret;
}

MODULE_DEVICE_TABLE(i2c, tmp_i2c_device_id);

static const struct of_device_id tmp_of_match[] = {
	{ .compatible = "ti,tmp102", },
	{ .compatible = "ti,tmp112", },
	{ .compatible = "ti,tmp75", },
	{ .compatible = "ti,tmp175", },
	{ .compatible = "ti,tmp275", },
	{ .compatible = "ti,tmpx75", },
	{ .compatible = "ti,tmp1x2", },
	{},
};

MODULE_DEVICE_TABLE(of, tmp_of_match);

static struct i2c_driver tmp_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= tmp_probe,
	.remove				= tmp_remove,
	.shutdown			= tmp_shutdown,
	.driver = {
		.name			= TMP_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(tmp_of_match),
		.pm			= &tmp_pm_ops,
	},
	.id_table			= tmp_i2c_device_id,
};
module_i2c_driver(tmp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TMP1x2 driver");
MODULE_AUTHOR("NVIDIA Corporation");

