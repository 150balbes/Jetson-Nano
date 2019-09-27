/* Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
/* See nvs.h for documentation */
/* See nvs_light.c and nvs_light.h for documentation */


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
#include <linux/nvs_light.h>

/* increment VEML_VERSION_DRIVER each time this driver changes */
#define VEML_VERSION_DRIVER		(2)
/* increment VEML_VERSION_SNSRCFG if data behaves differently */
#define VEML_VERSION_SNSRCFG		(1)
#define VEML_VENDOR			"Vishay"
#define VEML_NAME			"veml6030"
#define VEML_HW_DELAY_MS		(10)
#define VEML_ALS_CFG_DFLT		(0x0800)
#define VEML_ALS_PSM_DFLT		(0x07)
#define VEML_LIGHT_SCALE_IVAL		(0)
#define VEML_LIGHT_SCALE_MICRO		(10000)
#define VEML_LIGHT_THRESHOLD_LO		(10)
#define VEML_LIGHT_THRESHOLD_HI		(10)
#define VEML_POLL_DLY_MS_MAX		(4000)
/* HW registers */
#define VEML_REG_CFG			(0x00)
#define VEML_REG_CFG_ALS_SM		(11)
#define VEML_REG_CFG_ALS_IT		(6)
#define VEML_REG_CFG_ALS_PERS		(4)
#define VEML_REG_CFG_RSRV_ID		(2)
#define VEML_REG_CFG_ALS_INT_EN		(1)
#define VEML_REG_CFG_ALS_SD		(0)
#define VEML_REG_CFG_USER_MSK		(0x1BF0)
#define VEML_REG_WH			(0x01)
#define VEML_REG_WL			(0x02)
#define VEML_REG_PSM			(0x03)
#define VEML_REG_PSM_PSM		(1)
#define VEML_REG_PSM_EN			(0)
#define VEML_REG_PSM_MASK		(0x07)
#define VEML_REG_ALS			(0x04)
#define VEML_REG_WHITE			(0x05)
#define VEML_REG_ALS_INT		(0x06)
#define VEML_REG_ALS_INT_THR_L		(15)
#define VEML_REG_ALS_INT_THR_H		(14)
#define VEML_REG_N			(7)

enum VEML_DBG {
	VEML_DBG_STS = 0,
	/* skip sequence to "hide" debug features */
	VEML_DBG_CFG = 2,
	VEML_DBG_PSM,
	VEML_DBG_REG,
	VEML_DBG_RD,
};


/* regulator names in order of powering on */
static char *veml_vregs[] = {
	"vdd",
};

static unsigned short veml_i2c_addrs[] = {
	0x10,
	0x48,
};

static struct nvs_light_dynamic veml_nlds[] = {
	{ {0, 3600},   {235,  926000}, {0, 35000}, 800, 0x00C0 },
	{ {0, 7200},   {471,  852000}, {0, 35000}, 400, 0x0080 },
	{ {0, 14400},  {943,  704000}, {0, 35000}, 200, 0x0040 },
	{ {0, 28800},  {1887, 408000}, {0, 35000}, 100, 0x0000 },
	{ {0, 57600},  {3774, 816000}, {0, 35000}, 50,  0x0200 },
	{ {0, 115200}, {7549, 632000}, {0, 35000}, 25,  0x0300 }
};

struct veml_psm {
	unsigned int ms;
	struct nvs_float milliamp;
};

static struct veml_psm veml_psm_tbl[] = {
	{ 500,  {0, 21000} },
	{ 1000, {0, 15000} },
	{ 2000, {0, 10000} },
	{ 4000, {0, 6000} }
};

struct veml_state {
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	struct sensor_cfg cfg;
	struct workqueue_struct *wq;
	struct work_struct ws;
	struct regulator_bulk_data vreg[ARRAY_SIZE(veml_vregs)];
	struct nvs_light light;
	struct nvs_light_dynamic nld_tbl[ARRAY_SIZE(veml_nlds)];
	struct nld_thresh nld_thr[ARRAY_SIZE(veml_nlds)];
	unsigned int sts;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enabled;		/* enable status */
	u16 als_cfg;			/* ALS register 0 user settings */
	u16 als_psm;			/* ALS Power Save Mode */
	u16 rc[VEML_REG_N];		/* register cache for reg dump */
};


static void veml_err(struct veml_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
}

static int veml_i2c_rd(struct veml_state *st, u8 reg, u16 *val)
{
	struct i2c_msg msg[2];

	msg[0].addr = st->i2c->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = st->i2c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (__u8 *)val;
	if (i2c_transfer(st->i2c->adapter, msg, 2) != 2) {
		veml_err(st);
		return -EIO;
	}

	return 0;
}

static int veml_i2c_wr(struct veml_state *st, u8 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[3];

	buf[0] = reg;
	buf[1] = val & 0xFF;
	buf[2] = val >> 8;
	msg.addr = st->i2c->addr;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;
	if (i2c_transfer(st->i2c->adapter, &msg, 1) != 1) {
		veml_err(st);
		return -EIO;
	}

	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->i2c->dev,
			 "%s reg=0x%02X: 0x%02X=>0x%02X\n",
			 __func__, reg, st->rc[reg], val);
	st->rc[reg] = val;
	return 0;
}

static int veml_pm(struct veml_state *st, bool enable)
{
	int ret;

	if (enable) {
		ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
				       ARRAY_SIZE(veml_vregs));
		if (ret)
			mdelay(VEML_HW_DELAY_MS);
		veml_i2c_wr(st, VEML_REG_PSM, st->als_psm);
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(veml_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(veml_vregs))) {
			ret = veml_i2c_wr(st,
					  VEML_REG_CFG, st->rc[VEML_REG_CFG] |
					  (1 << VEML_REG_CFG_ALS_SD));
		} else if (ret > 0) {
			nvs_vregs_enable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(veml_vregs));
			mdelay(VEML_HW_DELAY_MS);
			ret = veml_i2c_wr(st,
					  VEML_REG_CFG, st->rc[VEML_REG_CFG] |
					  (1 << VEML_REG_CFG_ALS_SD));
		}
		ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(veml_vregs));
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		dev_err(&st->i2c->dev, "%s pwr=%x ERR=%d\n",
			__func__, enable, ret);
	} else {
		if (st->sts & NVS_STS_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s pwr=%x\n",
				 __func__, enable);
	}
	return ret;
}

static void veml_pm_exit(struct veml_state *st)
{
	veml_pm(st, false);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(veml_vregs));
}

static int veml_pm_init(struct veml_state *st)
{
	int ret;

	st->enabled = 0;
	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(veml_vregs), veml_vregs);
	ret = veml_pm(st, true);
	return ret;
}

static int veml_cmd_wr(struct veml_state *st, bool irq_en)
{
	u16 als_cfg;
	int ret = 0;

	als_cfg = st->als_cfg;
	als_cfg |= st->nld_tbl[st->light.nld_i].driver_data;
	veml_i2c_wr(st, VEML_REG_CFG, als_cfg); /* disable IRQ */
	if (irq_en && st->i2c->irq) {
		ret = veml_i2c_wr(st, VEML_REG_WL, st->light.hw_thresh_lo);
		ret |= veml_i2c_wr(st, VEML_REG_WH, st->light.hw_thresh_hi);
		if (!ret) {
			als_cfg |= (1 << VEML_REG_CFG_ALS_INT_EN);
			ret = veml_i2c_wr(st, VEML_REG_CFG, als_cfg);
			if (!ret)
				ret = RET_HW_UPDATE; /* flag IRQ enabled */
		}
	}
	return ret;
}

static int veml_rd(struct veml_state *st, bool irq)
{
	u16 hw;
	s64 ts;
	int ret;

	if (irq) {
		ret = veml_i2c_rd(st, VEML_REG_ALS_INT, &hw);
		if (ret < 0)
			return ret;

		if (!hw)
			/* not our device - no changes */
			return RET_HW_UPDATE;
	}

	ret = veml_i2c_rd(st, VEML_REG_ALS, &hw);
	if (ret)
		return ret;

	ts = nvs_timestamp();
	if (st->sts & NVS_STS_SPEW_DATA)
		dev_info(&st->i2c->dev,
			 "poll light hw %hu %lld  diff=%d %lldns  index=%u\n",
			 hw, ts, hw - st->light.hw, ts - st->light.timestamp,
			 st->light.nld_i);
	st->light.hw = hw;
	st->light.timestamp = ts;
	ret = nvs_light_read(&st->light);
	switch (ret) {
	case RET_POLL_NEXT:
		if (st->light.nld_i_change)
			ret = veml_cmd_wr(st, false);
		break;

	case RET_NO_CHANGE:
		if (st->i2c->irq)
			ret = RET_HW_UPDATE;
		break;

	case RET_HW_UPDATE:
		ret = veml_cmd_wr(st, true);
		break;

	default:
		break;
	}

	return ret;
}

static int veml_read(struct veml_state *st, bool irq)
{
	int ret;

	st->nvs->nvs_mutex_lock(st->light.nvs_st);
	ret = veml_rd(st, irq);
	st->nvs->nvs_mutex_unlock(st->light.nvs_st);
	return ret;
}

static void veml_work(struct work_struct *ws)
{
	struct veml_state *st = container_of((struct work_struct *)ws,
					     struct veml_state, ws);
	int ret;

	while (st->enabled) {
		msleep(st->light.poll_delay_ms);
		ret = veml_read(st, false);
		if (ret == RET_HW_UPDATE)
			/* switch to IRQ driven */
			break;
	}
}

static irqreturn_t veml_irq_thread(int irq, void *dev_id)
{
	struct veml_state *st = (struct veml_state *)dev_id;
	int ret;

	if (st->sts & NVS_STS_SPEW_IRQ)
		dev_info(&st->i2c->dev, "%s\n", __func__);
	if (st->enabled) {
		ret = veml_read(st, true);
		if (ret < RET_HW_UPDATE) {
			/* switch to polling */
			cancel_work_sync(&st->ws);
			queue_work(st->wq, &st->ws);
		}
	}
	return IRQ_HANDLED;
}

static int veml_disable(struct veml_state *st)
{
	int ret;

	ret = veml_pm(st, false);
	if (!ret)
		st->enabled = 0;
	return ret;
}

static int veml_enable(void *client, int snsr_id, int enable)
{
	struct veml_state *st = (struct veml_state *)client;
	int ret;

	if (enable < 0)
		return st->enabled;

	if (enable) {
		ret = veml_pm(st, true);
		if (!ret) {
			nvs_light_enable(&st->light);
			ret = veml_cmd_wr(st, false);
			if (ret) {
				veml_disable(st);
			} else {
				st->enabled = enable;
				cancel_work_sync(&st->ws);
				queue_work(st->wq, &st->ws);
			}
		}
	} else {
		ret = veml_disable(st);
	}
	return ret;
}

static int veml_batch(void *client, int snsr_id, int flags,
		    unsigned int period, unsigned int timeout)
{
	struct veml_state *st = (struct veml_state *)client;

	if (timeout)
		/* timeout not supported (no HW FIFO) */
		return -EINVAL;

	st->light.delay_us = period;
	return 0;
}

static int veml_resolution(void *client, int snsr_id, int resolution)
{
	struct veml_state *st = (struct veml_state *)client;
	int ret;

	ret = nvs_light_resolution(&st->light, resolution);
	if (st->light.nld_i_change) {
		veml_cmd_wr(st, false);
		cancel_work_sync(&st->ws);
		queue_work(st->wq, &st->ws);
	}
	return ret;
}

static int veml_max_range(void *client, int snsr_id, int max_range)
{
	struct veml_state *st = (struct veml_state *)client;
	int ret;

	ret = nvs_light_max_range(&st->light, max_range);
	if (st->light.nld_i_change) {
		veml_cmd_wr(st, false);
		cancel_work_sync(&st->ws);
		queue_work(st->wq, &st->ws);
	}
	return ret;
}

static int veml_thresh_lo(void *client, int snsr_id, int thresh_lo)
{
	struct veml_state *st = (struct veml_state *)client;

	return nvs_light_threshold_calibrate_lo(&st->light, thresh_lo);
}

static int veml_thresh_hi(void *client, int snsr_id, int thresh_hi)
{
	struct veml_state *st = (struct veml_state *)client;

	return nvs_light_threshold_calibrate_hi(&st->light, thresh_hi);
}

static int veml_regs(void *client, int snsr_id, char *buf)
{
	struct veml_state *st = (struct veml_state *)client;
	ssize_t t;
	u16 val;
	u8 i;
	int ret;

	t = sprintf(buf, "registers:\n");
	for (i = 0; i < VEML_REG_ALS; i++)
		t += sprintf(buf + t, "%#2x=%#4x\n", i, st->rc[i]);

	for (; i < VEML_REG_N; i++) {
		ret = veml_i2c_rd(st, i, &val);
		if (ret)
			t += sprintf(buf + t, "%#2x=ERR: %d\n", i, ret);
		else
			t += sprintf(buf + t, "%#2x=%#4x\n", i, val);
	}

	return t;
}

static int veml_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct veml_state *st = (struct veml_state *)client;
	u16 val;
	u8 reg;
	int ret;

	switch (nvs & 0xFF) {
	case VEML_DBG_STS:
		return 0;

	case VEML_DBG_CFG:
		st->als_cfg = (nvs >> 8) & VEML_REG_CFG_USER_MSK;
		dev_info(&st->i2c->dev, "%s als_cfg=%hx\n",
			 __func__, st->als_cfg);
		return 0;

	case VEML_DBG_PSM:
		st->als_psm = (nvs >> 8) & VEML_REG_PSM_MASK;
		dev_info(&st->i2c->dev,
			 "%s als_psm=%hx (applied when enabled)\n",
			 __func__, st->als_psm);
		return 0;

	case VEML_DBG_REG:
		reg = (nvs >> 24) & 0xFF;
		val = (nvs >> 8) & 0xFFFF;
		st->nvs->nvs_mutex_lock(st->light.nvs_st);
		ret = veml_i2c_wr(st, reg, val);
		st->nvs->nvs_mutex_unlock(st->light.nvs_st);
		dev_info(&st->i2c->dev, "%s %hx=>%hhx  err=%d\n",
			 __func__, val, reg, ret);
		return ret;

	case VEML_DBG_RD:
		st->nvs->nvs_mutex_lock(st->light.nvs_st);
		veml_rd(st, false);
		st->nvs->nvs_mutex_unlock(st->light.nvs_st);
		dev_info(&st->i2c->dev, "%s veml_read done\n", __func__);
		return 0;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int veml_nvs_read(void *client, int snsr_id, char *buf)
{
	struct veml_state *st = (struct veml_state *)client;
	ssize_t t;

	t = sprintf(buf, "driver v.%u\n", VEML_VERSION_DRIVER);
	t += sprintf(buf + t, "irq=%d\n", st->i2c->irq);
	t += sprintf(buf + t, "als_cfg=%hx\n", st->als_cfg);
	t += sprintf(buf + t, "als_psm=%hx\n", st->als_psm);
	return nvs_light_dbg(&st->light, buf + t);
}

static struct nvs_fn_dev veml_fn_dev = {
	.enable				= veml_enable,
	.batch				= veml_batch,
	.resolution			= veml_resolution,
	.max_range			= veml_max_range,
	.thresh_lo			= veml_thresh_lo,
	.thresh_hi			= veml_thresh_hi,
	.regs				= veml_regs,
	.nvs_write			= veml_nvs_write,
	.nvs_read			= veml_nvs_read,
};

#ifdef CONFIG_SUSPEND
static int veml_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct veml_state *st = i2c_get_clientdata(client);
	int ret = 0;

	if (st->nvs && st->light.nvs_st)
		ret = st->nvs->suspend(st->light.nvs_st);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int veml_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct veml_state *st = i2c_get_clientdata(client);
	int ret = 0;

	if (st->nvs && st->light.nvs_st)
		ret = st->nvs->resume(st->light.nvs_st);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(veml_pm_ops, veml_suspend, veml_resume);

static void veml_shutdown(struct i2c_client *client)
{
	struct veml_state *st = i2c_get_clientdata(client);

	if (st->nvs && st->light.nvs_st)
		st->nvs->shutdown(st->light.nvs_st);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int veml_remove(struct i2c_client *client)
{
	struct veml_state *st = i2c_get_clientdata(client);

	if (st != NULL) {
		veml_shutdown(client);
		if (st->nvs && st->light.nvs_st)
			st->nvs->remove(st->light.nvs_st);
		if (st->wq) {
			cancel_work_sync(&st->ws);
			destroy_workqueue(st->wq);
			st->wq = NULL;
		}
		veml_pm_exit(st);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int veml_id_i2c(struct veml_state *st, const char *name)
{
	u16 val;
	int i;

	for (i = 0; i < ARRAY_SIZE(veml_i2c_addrs); i++) {
		if (st->i2c->addr == veml_i2c_addrs[i])
			break;
	}

	if (i < ARRAY_SIZE(veml_i2c_addrs))
		return veml_i2c_rd(st, VEML_REG_ALS_INT, &val);

	return -ENODEV;
}

static struct sensor_cfg veml_cfg_dflt = {
	.name			= NVS_LIGHT_STRING,
	.ch_n			= 1,
	.ch_sz			= 2,
	.part			= VEML_NAME,
	.vendor			= VEML_VENDOR,
	.version		= VEML_VERSION_SNSRCFG,
	.delay_us_max		= VEML_POLL_DLY_MS_MAX * 1000,
	.flags			= SENSOR_FLAG_ON_CHANGE_MODE,
	.scale			= {
		.ival		= VEML_LIGHT_SCALE_IVAL,
		.fval		= VEML_LIGHT_SCALE_MICRO,
	},
	.thresh_lo		= VEML_LIGHT_THRESHOLD_LO,
	.thresh_hi		= VEML_LIGHT_THRESHOLD_HI,
};

static int veml_of_dt(struct veml_state *st, struct device_node *dn)
{
	unsigned int i;
	unsigned int j;
	int ret;

	/* default device specific parameters */
	st->als_cfg = VEML_ALS_CFG_DFLT;
	st->als_psm = VEML_ALS_PSM_DFLT;
	/* default NVS ALS programmable parameters */
	memcpy(&st->cfg, &veml_cfg_dflt, sizeof(st->cfg));
	st->light.cfg = &st->cfg;
	st->light.hw_mask = 0xFFFF;
	st->light.nld_tbl = st->nld_tbl;
	/* device tree parameters */
	if (dn) {
		/* common NVS parameters */
		ret = nvs_of_dt(dn, &st->cfg, NULL);
		if (ret == -ENODEV)
			return -ENODEV;

		/* device specific parameters */
		of_property_read_u16(dn, "als_cfg", &st->als_cfg);
		st->als_cfg &= VEML_REG_CFG_USER_MSK;
		of_property_read_u16(dn, "als_psm", &st->als_psm);
		st->als_psm &= VEML_REG_PSM_MASK;
	}

	memcpy(&st->nld_tbl, &veml_nlds, sizeof(st->nld_tbl));
	st->light.nld_tbl_n = ARRAY_SIZE(veml_nlds);
	if (st->als_psm & (1 << VEML_REG_PSM_EN)) {
		/* fixup dynamic table based on PSM */
		j = st->als_psm >> 1;
		for (i = 0; i < ARRAY_SIZE(veml_nlds); i++) {
			st->nld_tbl[i].delay_min_ms += veml_psm_tbl[j].ms;
			st->nld_tbl[i].milliamp.ival =
						 veml_psm_tbl[j].milliamp.ival;
			st->nld_tbl[i].milliamp.fval =
						 veml_psm_tbl[j].milliamp.fval;
		}
	}

	/* this device supports these programmable parameters */
	st->light.nld_thr = st->nld_thr;
	if (nvs_light_of_dt(&st->light, st->i2c->dev.of_node, st->cfg.part)) {
		st->light.nld_i_lo = 0;
		st->light.nld_i_hi = st->light.nld_tbl_n - 1;
	}
	i = st->light.nld_i_lo;
	st->cfg.resolution.ival = st->nld_tbl[i].resolution.ival;
	st->cfg.resolution.fval = st->nld_tbl[i].resolution.fval;
	i = st->light.nld_i_hi;
	st->cfg.max_range.ival = st->nld_tbl[i].max_range.ival;
	st->cfg.max_range.fval = st->nld_tbl[i].max_range.fval;
	st->cfg.milliamp.ival = st->nld_tbl[i].milliamp.ival;
	st->cfg.milliamp.fval = st->nld_tbl[i].milliamp.fval;
	st->cfg.delay_us_min = st->nld_tbl[i].delay_min_ms * 1000;
	return 0;
}

static int veml_init(struct veml_state *st, const struct i2c_device_id *id)
{
	int ret;

	ret = veml_of_dt(st, st->i2c->dev.of_node);
	if (ret) {
		if (ret == -ENODEV)
			dev_info(&st->i2c->dev, "%s DT disabled\n", __func__);
		else
			dev_err(&st->i2c->dev, "%s _of_dt ERR\n", __func__);
		return -ENODEV;
	}

	veml_pm_init(st);
	ret = veml_id_i2c(st, id->name);
	if (ret) {
		dev_err(&st->i2c->dev, "%s _id_i2c ERR\n", __func__);
		return -ENODEV;
	}

	veml_pm(st, false);
	veml_fn_dev.errs = &st->errs;
	veml_fn_dev.sts = &st->sts;
	st->nvs = nvs_iio();
	if (st->nvs == NULL) {
		dev_err(&st->i2c->dev, "%s nvs_iio ERR\n", __func__);
		return -ENODEV;
	}

	st->light.handler = st->nvs->handler;
	ret = st->nvs->probe(&st->light.nvs_st, st, &st->i2c->dev,
			     &veml_fn_dev, &st->cfg);
	if (ret) {
		dev_err(&st->i2c->dev, "%s nvs_probe ERR\n", __func__);
		return -ENODEV;
	}

	st->wq = create_workqueue(VEML_NAME);
	if (!st->wq) {
		dev_err(&st->i2c->dev, "%s create_workqueue ERR\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&st->ws, veml_work);
	if (st->i2c->irq) {
		ret = request_threaded_irq(st->i2c->irq, NULL, veml_irq_thread,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   VEML_NAME, st);
		if (ret) {
			dev_err(&st->i2c->dev, "%s req_threaded_irq ERR %d\n",
				__func__, ret);
			return -ENOMEM;
		}
	}

	return 0;
}

static int veml_probe(struct i2c_client *client,
		      const struct i2c_device_id *id)
{
	struct veml_state *st;
	int ret;

	dev_info(&client->dev, "%s\n", __func__);
	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		dev_err(&client->dev, "%s devm_kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = veml_init(st, id);
	if (ret) {
		veml_remove(client);
		return ret;
	}

	dev_info(&client->dev, "%s done\n", __func__);
	return 0;
}

static const struct i2c_device_id veml_i2c_device_id[] = {
	{ VEML_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, veml_i2c_device_id);

static const struct of_device_id veml_of_match[] = {
	{ .compatible = "vishay,veml6030", },
	{},
};

MODULE_DEVICE_TABLE(of, veml_of_match);

static struct i2c_driver veml_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= veml_probe,
	.remove		= veml_remove,
	.shutdown	= veml_shutdown,
	.driver = {
		.name		= VEML_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(veml_of_match),
		.pm		= &veml_pm_ops,
	},
	.id_table	= veml_i2c_device_id,
};
module_i2c_driver(veml_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("VEML6030 driver");
MODULE_AUTHOR("NVIDIA Corporation");
