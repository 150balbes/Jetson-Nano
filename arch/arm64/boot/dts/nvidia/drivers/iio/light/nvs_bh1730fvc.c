/* Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
/* See nvs_iio.c and nvs.h for documentation */
/* See nvs_light.c and nvs_light.h for documentation */


#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/nvs.h>
#include <linux/nvs_light.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif

#define BH1730_VENDOR			"ROHM"
#define BH1730_NAME			"bh1730fvc"
#define BH1730_LIGHT_VERSION		(3)
#define BH1730_LIGHT_MAX_RANGE_IVAL	(100000)
#define BH1730_LIGHT_MAX_RANGE_MICRO	(0)
#define BH1730_LIGHT_RESOLUTION_IVAL	(1)
#define BH1730_LIGHT_RESOLUTION_MICRO	(0)
#define BH1730_LIGHT_MILLIAMP_IVAL	(0)
#define BH1730_LIGHT_MILLIAMP_MICRO	(90000)
#define BH1730_LIGHT_SCALE_IVAL		(0)
#define BH1730_LIGHT_SCALE_MICRO	(0)
#define BH1730_LIGHT_OFFSET_IVAL	(0)
#define BH1730_LIGHT_OFFSET_MICRO	(0)
#define BH1730_LIGHT_THRESHOLD_LO	(1)
#define BH1730_LIGHT_THRESHOLD_HI	(1)
#define BH1730_POLL_DLY_US_MIN		(2700)
#define BH1730_POLL_DLY_US_MAX		(688500)

/* HW registers */
#define BH1730_COMMAND			(0x80)

#define BH1730_REG_CONTROL		(0x00)
#define BH1730_REG_TIMING		(0x01)
#define BH1730_REG_INTERRUPT		(0x02)
#define BH1730_REG_THLLOW		(0x03)
#define BH1730_REG_THLHIGH		(0x04)
#define BH1730_REG_THHLOW		(0x05)
#define BH1730_REG_THHHIGH		(0x06)
#define BH1730_REG_GAIN			(0x07)
#define BH1730_REG_ID			(0x12)
#define BH1730_REG_DATA0LOW		(0x14)
#define BH1730_REG_DATA0HIGH		(0x15)
#define BH1730_REG_DATA1LOW		(0x16)
#define BH1730_REG_DATA1HIGH		(0x17)

#define BIT_ADC_INTR	0x20
#define BIT_ADC_VALID	0x10
#define BIT_ONE_TIME	0x08
#define BIT_DATA_SEL	0x04
#define BIT_ADC_EN	0x02
#define BIT_POWER	0x01

#define BH1730_HW_DELAY_MS		(10)

enum BH1730_GAIN {
	BH1730_GAIN_X1 = 0,
	BH1730_GAIN_X2,
	BH1730_GAIN_X64,
	BH1730_GAIN_X128,
};

/* regulator names in order of powering on */
static char *bh1730_vregs[] = {
	"vdd",
	"vid",
};

struct lux_coeff_t {
	u32 d;
	u32 c0;
	u32 c1;
};

struct lux_cal_data_t {
	struct lux_coeff_t *coeff;
	u32 coeff_arr_size;
	u32 res;
	u32 mul;
};

static struct lux_coeff_t def_lux_coeff[] = {
	{  260, 1290, 2733},
	{  550,  795,  859},
	{ 1090,  510,  345},
	{ 2130,  276,  130},
};

static struct lux_cal_data_t def_lux_data = {
	.coeff = def_lux_coeff,
	.coeff_arr_size = ARRAY_SIZE(def_lux_coeff),
	.res = 1000,
	.mul = 100,
};

struct bh1730_state {
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	void *nvs_st;
	struct sensor_cfg cfg;
	struct workqueue_struct *wq;
	struct work_struct ws;
	struct regulator_bulk_data vreg[ARRAY_SIZE(bh1730_vregs)];
	struct nvs_light light;
	struct lux_cal_data_t lux;
	unsigned int sts;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int enabled;		/* enable status */
	bool hw_change;			/* HW changed so drop first sample */
	u32 itime_us;
	u32 gain;
	u16 data0;
	u16 data1;
	u8 cached_reg_addr;
	int always_on;
};


static int bh1730_i2c_wr(struct bh1730_state *st, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(st->i2c, reg | BH1730_COMMAND, val);

	if (ret)
		return ret;

	if (reg == BH1730_REG_GAIN) {
		if (val == BH1730_GAIN_X1)
			st->gain = 1;
		else if (val == BH1730_GAIN_X2)
			st->gain = 2;
		else if (val == BH1730_GAIN_X64)
			st->gain = 64;
		else if (val == BH1730_GAIN_X128)
			st->gain = 128;
	} else if (reg == BH1730_REG_TIMING)
		st->itime_us = (256 - val) * 2700;

	return 0;
}

static int bh1730_i2c_rd(struct bh1730_state *st, u8 reg, u8 *val)
{
	int ret = i2c_smbus_read_byte_data(st->i2c, reg | BH1730_COMMAND);

	if (ret < 0)
		return -EIO;
	*val = (u8)ret;
	return 0;
}

static int bh1730_read_n(struct bh1730_state *st, u8 reg, u8 *val, int n)
{
	int ret = i2c_smbus_read_i2c_block_data(st->i2c, reg | BH1730_COMMAND,
					    n, val);
	if (ret != n)
		return -EIO;
	return 0;
}

static int bh1730_rd_data(struct bh1730_state *st, u16 *val)
{
	u8 data[4];

	if (bh1730_read_n(st, BH1730_REG_DATA0LOW, data, 4)) {
		dev_err(&st->i2c->dev, "%s: bh1730_read_n failure\n", __func__);
		return -EIO;
	}
	val[0] = ((data[1] << 8) | data[0]);
	val[1] = ((data[3] << 8) | data[2]);
	st->data0 = val[0];
	st->data1 = val[1];

	return 0;
}

static int bh1730_pm(struct bh1730_state *st, bool enable)
{
	int ret;

	if (enable) {
		ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
				       ARRAY_SIZE(bh1730_vregs));
		if (ret)
			mdelay(BH1730_HW_DELAY_MS);
		ret = bh1730_i2c_wr(st, BH1730_REG_CONTROL,
				    BIT_ADC_EN | BIT_POWER);
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(bh1730_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(bh1730_vregs))) {
			ret = bh1730_i2c_wr(st, BH1730_REG_CONTROL, 0);
		} else if (ret > 0) {
			nvs_vregs_enable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(bh1730_vregs));
			mdelay(BH1730_HW_DELAY_MS);
			ret = bh1730_i2c_wr(st, BH1730_REG_CONTROL, 0);
		}
		ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
					 ARRAY_SIZE(bh1730_vregs));
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

static void bh1730_pm_exit(struct bh1730_state *st)
{
	bh1730_pm(st, false);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(bh1730_vregs));
}

static int bh1730_pm_init(struct bh1730_state *st)
{
	int ret;

	st->enabled = 0;
	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(bh1730_vregs), bh1730_vregs);
	/* on off for low power mode if regulator still on */
	ret = bh1730_pm(st, true);
	ret |= bh1730_pm(st, false);
	return ret;
}

static int bh1730_init(struct bh1730_state *st)
{
	int ret;

	/* itime 100ms */
	ret = bh1730_i2c_wr(st, BH1730_REG_TIMING, 0xDA);
	/* gain X1 */
	ret |= bh1730_i2c_wr(st, BH1730_REG_GAIN, BH1730_GAIN_X1);
	if (!ret)
		st->hw_change = true;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s itime_us=%d gain=%d err=%d\n",
			 __func__, st->itime_us, st->gain, ret);
	return ret;
}

static u32 bh1730_get_lux(struct bh1730_state *st, u16 *data)
{
	u32 lux;
	int i;
	u32 data0 = data[0];
	u32 data1 = data[1];
	struct lux_coeff_t *coeff;

	lux = 0;
	for (i = 0; i < st->lux.coeff_arr_size; i++) {
		coeff = &st->lux.coeff[i];
		if (st->lux.res * data1 / data0 < coeff->d) {
			lux = coeff->c0 * data0 -
				coeff->c1 * data1;
			break;
		}
	}
	lux = lux * st->lux.mul / (st->gain * st->itime_us);

	return lux;
}

static int bh1730_rd(struct bh1730_state *st)
{
	u16 hw[2];
	s64 ts;
	struct nvs_light *nl = &st->light;
	int ret;

	if (st->hw_change) {
		/* drop first sample after HW change */
		st->hw_change = false;
		return 0;
	}

	ret = bh1730_rd_data(st, hw);
	if (ret)
		return ret;

	ts = nvs_timestamp();
	nl->hw = bh1730_get_lux(st, hw);
	nl->timestamp = ts;
	nvs_light_read(nl);

	if (nl->nld_i_change)
		bh1730_init(st);
	return 0;
}

static int bh1730_read(struct bh1730_state *st)
{
	int ret;

	st->nvs->nvs_mutex_lock(st->nvs_st);
	ret = bh1730_rd(st);
	st->nvs->nvs_mutex_unlock(st->nvs_st);
	return ret;
}

static void bh1730_work(struct work_struct *ws)
{
	struct bh1730_state *st = container_of((struct work_struct *)ws,
					   struct bh1730_state, ws);

	while (st->enabled) {
		msleep(st->light.poll_delay_ms);
		bh1730_read(st);
	}
}

static int bh1730_disable(struct bh1730_state *st)
{
	int ret;

	if (st->always_on)
		return 0;

	ret = bh1730_pm(st, false);
	if (!ret)
		st->enabled = 0;
	return ret;
}

static int bh1730_enable(void *client, int snsr_id, int enable)
{
	struct bh1730_state *st = (struct bh1730_state *)client;
	int ret;

	if (enable < 0)
		return st->enabled;

	if (enable) {
		ret = bh1730_pm(st, true);
		if (!ret) {
			ret = bh1730_init(st);
			st->light.cfg->delay_us_min = st->itime_us;
			st->light.cfg->delay_us_max = st->itime_us;
			nvs_light_enable(&st->light);
			if (ret) {
				bh1730_disable(st);
			} else {
				st->enabled = 1;
				cancel_work_sync(&st->ws);
				queue_work(st->wq, &st->ws);
			}
		}
	} else {
		ret = bh1730_disable(st);
	}
	return ret;
}

static int bh1730_batch(void *client, int snsr_id, int flags,
		    unsigned int period, unsigned int timeout)
{
	struct bh1730_state *st = (struct bh1730_state *)client;

	if (timeout)
		/* timeout not supported (no HW FIFO) */
		return -EINVAL;

	st->light.delay_us = period;
	return 0;
}

static int bh1730_regs(void *client, int snsr_id, char *buf)
{
	struct bh1730_state *st = (struct bh1730_state *)client;
	ssize_t t;
	int ret;
	u8 readval;
	u8 i;
	u8 regs[] = {
		BH1730_REG_CONTROL,
		BH1730_REG_TIMING,
		BH1730_REG_INTERRUPT,
		BH1730_REG_THLLOW,
		BH1730_REG_THLHIGH,
		BH1730_REG_THHLOW,
		BH1730_REG_THHHIGH,
		BH1730_REG_GAIN,
		BH1730_REG_ID,
		BH1730_REG_DATA0LOW,
		BH1730_REG_DATA0HIGH,
		BH1730_REG_DATA1LOW,
		BH1730_REG_DATA1HIGH
	};

	t = snprintf(buf, PAGE_SIZE,  "lux: %u\n", st->light.lux);
	t += snprintf(buf + t, PAGE_SIZE - t,  "registers:\n");
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = bh1730_i2c_rd(st, regs[i], &readval);
		if (ret)
			t += snprintf(buf + t, PAGE_SIZE - t,  "0x%hhx=ERR\n", i);
		else
			t += snprintf(buf + t, PAGE_SIZE - t,  "0x%hhx=0x%hhx\n", i, readval);
	}
	return t;
}

static int bh1730_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct bh1730_state *st = (struct bh1730_state *)client;
	u8 reg;
	u8 val;

	reg = (nvs >> 16) & 0xff;
	val = (nvs >> 8) & 0xff;
	return bh1730_i2c_wr(st, reg, val);
}

static int bh1730_nvs_read(void *client, int snsr_id, char *buf)
{
	struct bh1730_state *st = (struct bh1730_state *)client;

	return nvs_light_dbg(&st->light, buf);
}

static struct nvs_fn_dev bh1730_fn_dev = {
	.enable				= bh1730_enable,
	.batch				= bh1730_batch,
	.regs				= bh1730_regs,
	.nvs_write			= bh1730_nvs_write,
	.nvs_read			= bh1730_nvs_read,
};

#ifdef CONFIG_PM_SLEEP
static int bh1730_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bh1730_state *st = i2c_get_clientdata(client);
	int ret = 0;

	if (st->always_on)
		return 0;

	st->sts |= NVS_STS_SUSPEND;
	if (st->nvs && st->nvs_st)
		ret = st->nvs->suspend(st->nvs_st);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int bh1730_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bh1730_state *st = i2c_get_clientdata(client);
	int ret = 0;

	if (st->always_on)
		return 0;

	if (st->nvs && st->nvs_st)
		ret = st->nvs->resume(st->nvs_st);
	st->sts &= ~NVS_STS_SUSPEND;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static SIMPLE_DEV_PM_OPS(bh1730_pm_ops, bh1730_suspend, bh1730_resume);
#endif

static void bh1730_shutdown(struct i2c_client *client)
{
	struct bh1730_state *st = i2c_get_clientdata(client);

	st->sts |= NVS_STS_SHUTDOWN;
	if (st->nvs && st->nvs_st)
		st->nvs->shutdown(st->nvs_st);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int bh1730_remove(struct i2c_client *client)
{
	struct bh1730_state *st = i2c_get_clientdata(client);

	if (st != NULL) {
		bh1730_shutdown(client);
		if (st->nvs && st->nvs_st)
			st->nvs->remove(st->nvs_st);
		if (st->wq) {
			destroy_workqueue(st->wq);
			st->wq = NULL;
		}
		bh1730_pm_exit(st);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static struct sensor_cfg bh1730_cfg_dflt = {
	.name			= NVS_LIGHT_STRING,
	.ch_n			= 1,
	.ch_sz			= 4,
	.part			= BH1730_NAME,
	.vendor			= BH1730_VENDOR,
	.version		= BH1730_LIGHT_VERSION,
	.max_range		= {
		.ival		= BH1730_LIGHT_MAX_RANGE_IVAL,
		.fval		= BH1730_LIGHT_MAX_RANGE_MICRO,
	},
	.resolution		= {
		.ival		= BH1730_LIGHT_RESOLUTION_IVAL,
		.fval		= BH1730_LIGHT_RESOLUTION_MICRO,
	},
	.milliamp		= {
		.ival		= BH1730_LIGHT_MILLIAMP_IVAL,
		.fval		= BH1730_LIGHT_MILLIAMP_MICRO,
	},
	.delay_us_min		= BH1730_POLL_DLY_US_MIN,
	.delay_us_max		= BH1730_POLL_DLY_US_MAX,
	.flags			= SENSOR_FLAG_ON_CHANGE_MODE,
	.scale			= {
		.ival		= BH1730_LIGHT_SCALE_IVAL,
		.fval		= BH1730_LIGHT_SCALE_MICRO,
	},
	.thresh_lo		= BH1730_LIGHT_THRESHOLD_LO,
	.thresh_hi		= BH1730_LIGHT_THRESHOLD_HI,
};

static int bh1730_of_dt(struct bh1730_state *st, struct device_node *dn)
{
	int ret;
	u32 *lux_coeff;
	int coeff_arr_size = 0, res = 0, mul = 0;

	/* default NVS programmable parameters */
	memcpy(&st->cfg, &bh1730_cfg_dflt, sizeof(st->cfg));
	st->light.cfg = &st->cfg;
	st->light.hw_mask = 0xFFFF;
	memcpy(&st->lux, &def_lux_data, sizeof(def_lux_data));

	/* device tree parameters */
	if (dn) {
		ret = nvs_of_dt(dn, &st->cfg, NULL);
		if (ret == -ENODEV)
			return ret;
		ret = of_property_read_u32(dn,
				"bh1730fvc-lux-resolution", &res);
		ret = of_property_read_u32(dn,
				"bh1730fvc-lux-multiplier", &mul);

		coeff_arr_size = of_property_count_elems_of_size(dn,
					"bh1730fvc-lux-coeff", sizeof(u32));
		if (coeff_arr_size <= 0)
			return coeff_arr_size;

		lux_coeff = (u32 *)devm_kzalloc(&st->i2c->dev,
						sizeof(u32) * coeff_arr_size,
						GFP_KERNEL);
		if (!lux_coeff)
			return -ENOMEM;

		ret = of_property_read_u32_array(dn, "bh1730fvc-lux-coeff",
				lux_coeff, coeff_arr_size);
		if (ret || res == 0 || mul == 0) {
			devm_kfree(&st->i2c->dev, lux_coeff);
			pr_info("bh1730fvc: using default lux coeff\n");
			return 0;
		}

		st->lux.coeff = (struct lux_coeff_t *)lux_coeff;
		st->lux.coeff_arr_size = coeff_arr_size /
				(sizeof(struct lux_coeff_t) / sizeof(u32));
		st->lux.res = res;
		st->lux.mul = mul;
	}

	return 0;
}

static ssize_t bh1730_debugfs_read_lux(struct file *file,
			char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bh1730_state *st = file->private_data;
	char buf[20];
	ssize_t len;

	len = snprintf(buf, sizeof(buf), "lux: %u\n", st->light.hw);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations bh1730_debugfs_lux_fops = {
	.open = simple_open,
	.read = bh1730_debugfs_read_lux,
};

static ssize_t bh1730_debugfs_read_reg(struct file *file,
			char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bh1730_state *st = file->private_data;
	char buf[256];
	u8 val = 0;
	ssize_t len;
	int ret, i;

	u8 regs[] = {
		BH1730_REG_CONTROL,
		BH1730_REG_TIMING,
		BH1730_REG_INTERRUPT,
		BH1730_REG_THLLOW,
		BH1730_REG_THLHIGH,
		BH1730_REG_THHLOW,
		BH1730_REG_THHHIGH,
		BH1730_REG_GAIN,
		BH1730_REG_ID,
		BH1730_REG_DATA0LOW,
		BH1730_REG_DATA0HIGH,
		BH1730_REG_DATA1LOW,
		BH1730_REG_DATA1HIGH
	};

	len = snprintf(buf, sizeof(buf),  "lux: %u\n", st->light.hw);
	len += snprintf(buf + len, sizeof(buf) - len,  "registers:\n");
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = bh1730_i2c_rd(st, regs[i], &val);
		if (ret)
			len += snprintf(buf + len, sizeof(buf) - len,  "0x%02hhx=ERR\n", regs[i]);
		else
			len += snprintf(buf + len, sizeof(buf) - len,  "0x%02hhx=0x%02hhx\n",
					regs[i], val);
	}
	buf[len++] = 0;

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t bh1730_debugfs_write_reg(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bh1730_state *st = file->private_data;
	unsigned reg, val;
	char buf[80];
	int ret;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%10i %10i", &reg, &val);

	switch (ret) {
	case 1:
		st->cached_reg_addr = reg;
		break;
	case 2:
		st->cached_reg_addr = reg;
		ret = bh1730_i2c_wr(st, (u8)reg, (u8)val);
		if (ret) {
			dev_err(&st->i2c->dev, "%s: write failed\n", __func__);
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bh1730_debugfs_reg_fops = {
	.open = simple_open,
	.read = bh1730_debugfs_read_reg,
	.write = bh1730_debugfs_write_reg,
};

static ssize_t bh1730_debugfs_read_alwayson(struct file *file,
			char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bh1730_state *st = file->private_data;
	char buf[16];
	ssize_t len;

	len = snprintf(buf, sizeof(buf), "%d\n", st->always_on);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t bh1730_debugfs_write_alwayson(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bh1730_state *st = file->private_data;
	int val;
	char buf[16];
	size_t buf_size;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(&buf[0], userbuf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	if (!kstrtoint(buf, 0, &val))
		st->always_on = val;

	if (st->always_on && !st->enabled)
		bh1730_enable(st, 0, 1);

	return buf_size;
}

static const struct file_operations bh1730_debugfs_alwayson_fops = {
	.open = simple_open,
	.read = bh1730_debugfs_read_alwayson,
	.write = bh1730_debugfs_write_alwayson,
};

static int bh1730_probe(struct i2c_client *client,
		    const struct i2c_device_id *id)
{
	struct bh1730_state *st;
	struct dentry *debugfs_root;
	int ret;
#ifdef DEBUG
	int i;
#endif

	dev_info(&client->dev, "%s\n", __func__);
	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		dev_err(&client->dev, "%s devm_kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = bh1730_of_dt(st, client->dev.of_node);
	if (ret) {
		if (ret == -ENODEV) {
			dev_info(&client->dev, "%s DT disabled\n", __func__);
		} else {
			dev_err(&client->dev, "%s _of_dt ERR\n", __func__);
			ret = -ENODEV;
		}
		goto bh1730_probe_exit;
	}

	ret = bh1730_pm_init(st);
	if (ret) {
		dev_err(&client->dev, "%s _pm_init ERR\n", __func__);
		ret = -ENODEV;
		goto bh1730_probe_exit;
	}

	bh1730_fn_dev.errs = &st->errs;
	bh1730_fn_dev.sts = &st->sts;
	st->nvs = nvs_iio();
	if (st->nvs == NULL) {
		dev_err(&client->dev, "%s nvs_iio ERR\n", __func__);
		ret = -ENODEV;
		goto bh1730_probe_exit;
	}

	st->light.handler = st->nvs->handler;
	ret = st->nvs->probe(&st->nvs_st, st, &client->dev,
			     &bh1730_fn_dev, &st->cfg);
	if (ret) {
		dev_err(&client->dev, "%s nvs_probe ERR\n", __func__);
		ret = -ENODEV;
		goto bh1730_probe_exit;
	}

	debugfs_root = debugfs_create_dir(BH1730_NAME, NULL);
	if (debugfs_root) {
		debugfs_create_file("lux", 0444, debugfs_root, st,
				    &bh1730_debugfs_lux_fops);
		debugfs_create_file("reg", 0644, debugfs_root, st,
				    &bh1730_debugfs_reg_fops);
		debugfs_create_file("alwayson", 0644, debugfs_root, st,
				    &bh1730_debugfs_alwayson_fops);
	}
#ifdef DEBUG
	for (i = 0; i < st->lux.coeff_arr_size; i++)
		pr_info("lux: if (data1 * %u / data0 < %u)\n"
			"lux:    lux = %u * data0 - %u * data[1]\n",
			st->lux.res, st->lux.coeff[i].d,
			st->lux.coeff[i].c0, st->lux.coeff[i].c1);
	pr_info("lux: lux = lux * %u / (gain * itime_us)\n",
		st->lux.mul);
#endif

	st->light.nvs_st = st->nvs_st;
	st->wq = create_workqueue(BH1730_NAME);
	if (!st->wq) {
		dev_err(&client->dev, "%s create_workqueue ERR\n", __func__);
		ret = -ENOMEM;
		goto bh1730_probe_exit;
	}

	INIT_WORK(&st->ws, bh1730_work);
	dev_info(&client->dev, "%s done\n", __func__);
	return 0;

bh1730_probe_exit:
	bh1730_remove(client);
	return ret;
}

static const struct i2c_device_id bh1730_i2c_device_id[] = {
	{ BH1730_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, bh1730_i2c_device_id);

static const struct of_device_id bh1730_of_match[] = {
	{ .compatible = "rohm,bh1730fvc", },
	{},
};

MODULE_DEVICE_TABLE(of, bh1730_of_match);

static struct i2c_driver bh1730_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= bh1730_probe,
	.remove		= bh1730_remove,
	.shutdown	= bh1730_shutdown,
	.driver = {
		.name		= BH1730_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(bh1730_of_match),
#ifdef CONFIG_PM_SLEEP
		.pm		= &bh1730_pm_ops,
#endif
	},
	.id_table	= bh1730_i2c_device_id,
};
module_i2c_driver(bh1730_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BH1730FVC driver");
MODULE_AUTHOR("NVIDIA Corporation");
