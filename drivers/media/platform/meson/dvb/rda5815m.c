/*
 * Driver for the RDA Microelectronics RDA5815M tuner
 *
 * Copyright (C) 2018 Igor Mokrushin aka McMCC <mcmcc@mail.ru>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/i2c.h>
#include <linux/types.h>
#include "tuner-i2c.h"
#include "rda5815m.h"
#ifdef DUAL_TUNER
#include "tuner_ftm4862.h"
#endif

struct reg_pair_t {
	u8 reg;
	u8 val;
};

const struct reg_pair_t reg_init_table_rda5815m[] =
{
	/* Initial configuration start */
	/* pll setting */
	{ 0x1a, 0x13 },
	{ 0x41, 0x53 },
	{ 0x38, 0x9b },
	{ 0x39, 0x15 },
	{ 0x3a, 0x00 },
	{ 0x3b, 0x00 },
	{ 0x3c, 0x0c },
	{ 0x0c, 0xe2 },
	{ 0x2e, 0x6f },
	{ 0x72, 0x07 },
	{ 0x73, 0x10 },
	{ 0x74, 0x71 },
	{ 0x75, 0x06 },
	{ 0x76, 0x40 },
	{ 0x77, 0x89 },
	{ 0x79, 0x04 },
	{ 0x7a, 0x2a },
	{ 0x7b, 0xaa },
	{ 0x7c, 0xab },
	{ 0x2f, 0x57 },
	{ 0x0d, 0x70 },
	{ 0x18, 0x4b },
	{ 0x30, 0xff },
	{ 0x5c, 0xff },
	{ 0x65, 0x00 },
	{ 0x70, 0x3f },
	{ 0x71, 0x3f },
	{ 0x53, 0xa8 },
	{ 0x46, 0x21 },
	{ 0x47, 0x84 },
	{ 0x48, 0x10 },
	{ 0x49, 0x08 },
	{ 0x60, 0x80 },
	{ 0x61, 0x80 },
	{ 0x6a, 0x08 },
	{ 0x6b, 0x63 },
	{ 0x69, 0xf8 },
	{ 0x57, 0x64 },
	{ 0x05, 0xaa },
	{ 0x06, 0xaa },
	{ 0x15, 0xae },
	{ 0x4a, 0x67 },
	{ 0x4b, 0x77 },

	/* agc setting */
	{ 0x4f, 0x40 },
	{ 0x5b, 0x20 },

	/* for blocker */
	{ 0x16, 0x0c }, /* stage setting */
	{ 0x18, 0x0c },
	{ 0x30, 0x1c },
	{ 0x5c, 0x2c },
	{ 0x6c, 0x3c },
	{ 0x6e, 0x3c },
	{ 0x1b, 0x7c },
	{ 0x1d, 0xbd },
	{ 0x1f, 0xbd },
	{ 0x21, 0xbe },
	{ 0x23, 0xbe },
	{ 0x25, 0xfe },
	{ 0x27, 0xff },
	{ 0x29, 0xff },
	{ 0xb3, 0xff },
	{ 0xb5, 0xff },

	{ 0x17, 0xf0 },
	{ 0x19, 0xf0 },
	{ 0x31, 0xf0 },
	{ 0x5d, 0xf0 },
	{ 0x6d, 0xf0 },
	{ 0x6f, 0xf1 },
	{ 0x1c, 0xf5 },
	{ 0x1e, 0x35 },
	{ 0x20, 0x79 },
	{ 0x22, 0x9d },
	{ 0x24, 0xbe },
	{ 0x26, 0xbe },
	{ 0x28, 0xbe },
	{ 0x2a, 0xcf },
	{ 0xb4, 0xdf },
	{ 0xb6, 0x0f },

	{ 0xb7, 0x15 }, /* start */
	{ 0xb9, 0x6c },
	{ 0xbb, 0x63 },
	{ 0xbd, 0x5a },
	{ 0xbf, 0x5a },
	{ 0xc1, 0x55 },
	{ 0xc3, 0x55 },
	{ 0xc5, 0x47 },
	{ 0xa3, 0x53 },
	{ 0xa5, 0x4f },
	{ 0xa7, 0x4e },
	{ 0xa9, 0x4e },
	{ 0xab, 0x54 },
	{ 0xad, 0x31 },
	{ 0xaf, 0x43 },
	{ 0xb1, 0x9f },

	{ 0xb8, 0x6c }, /* end */
	{ 0xba, 0x92 },
	{ 0xbc, 0x8a },
	{ 0xbe, 0x8a },
	{ 0xc0, 0x82 },
	{ 0xc2, 0x93 },
	{ 0xc4, 0x85 },
	{ 0xc6, 0x77 },
	{ 0xa4, 0x82 },
	{ 0xa6, 0x7e },
	{ 0xa8, 0x7d },
	{ 0xaa, 0x6f },
	{ 0xac, 0x65 },
	{ 0xae, 0x43 },
	{ 0xb0, 0x9f },
	{ 0xb2, 0xf0 },

	{ 0x81, 0x92 }, /* rise */
	{ 0x82, 0xb4 },
	{ 0x83, 0xb3 },
	{ 0x84, 0xac },
	{ 0x85, 0xba },
	{ 0x86, 0xbc },
	{ 0x87, 0xaf },
	{ 0x88, 0xa2 },
	{ 0x89, 0xac },
	{ 0x8a, 0xa9 },
	{ 0x8b, 0x9b },
	{ 0x8c, 0x7d },
	{ 0x8d, 0x74 },
	{ 0x8e, 0x9f },
	{ 0x8f, 0xf0 },

	{ 0x90, 0x15 }, /* fall */
	{ 0x91, 0x39 },
	{ 0x92, 0x30 },
	{ 0x93, 0x27 },
	{ 0x94, 0x29 },
	{ 0x95, 0x0d },
	{ 0x96, 0x10 },
	{ 0x97, 0x1e },
	{ 0x98, 0x1a },
	{ 0x99, 0x19 },
	{ 0x9a, 0x19 },
	{ 0x9b, 0x32 },
	{ 0x9c, 0x1f },
	{ 0x9d, 0x31 },
	{ 0x9e, 0x43 },
};

#ifndef DUAL_TUNER
struct rda5815m_priv {
	struct rda5815m_config *cfg;
	struct i2c_adapter *i2c;
};

static int rda5815m_write_reg(struct rda5815m_priv *priv, u8 reg, u8 val)
#else
int rda5815m_write_reg(struct dual_tuner_priv *priv, u8 reg, u8 val)
#endif
{
	int ret;
	u8 buf[] = { reg, val };
	struct i2c_msg msg = { .addr = priv->cfg->i2c_address, .flags = 0,
			       .buf = buf, .len = 2 };

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "i2c wr failed = %d, reg = %02x ", ret, reg);
		ret = -EREMOTEIO;
	}
	return ret;
}

#ifndef DUAL_TUNER
static int rda5815m_read_reg(struct rda5815m_priv *priv, u8 reg, u8 *val)
#else
int rda5815m_read_reg(struct dual_tuner_priv *priv, u8 reg, u8 *val)
#endif
{
	int ret;
	struct i2c_msg msg[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,
		  .buf = &reg, .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD,
		  .buf = val, .len = 1 },
	};

	ret = i2c_transfer(priv->i2c, msg, 2);
	if (ret == 2) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "i2c rd failed = %d, reg = %02x ", ret, reg);
		ret = -EREMOTEIO;
	}
	return ret;
}

#ifndef DUAL_TUNER
static
#endif
int rda5815m_init(struct dvb_frontend *fe)
{
#ifndef DUAL_TUNER
	struct rda5815m_priv *priv = fe->tuner_priv;
#else
	struct dual_tuner_priv *priv = fe->tuner_priv;
#endif
	unsigned int i;
	int ret = 0;

	dev_dbg(&priv->i2c->dev, "%s() start\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	msleep(1);
	/* Chip register soft reset */
	ret = rda5815m_write_reg(priv, 0x04, 0x04);
	if (ret)
		goto ret_init;
	ret = rda5815m_write_reg(priv, 0x04, 0x05);
	if (ret)
		goto ret_init;

	for (i = 0; i < ARRAY_SIZE(reg_init_table_rda5815m); i++)
	{
		ret = rda5815m_write_reg(priv, reg_init_table_rda5815m[i].reg,
						reg_init_table_rda5815m[i].val);
		if (ret)
			break;
	}
	msleep(10);

ret_init:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	if (ret)
		dev_err(&priv->i2c->dev, "%s() failed\n", __func__);

	return ret;
}

#ifndef DUAL_TUNER
static int rda5815m_release(struct dvb_frontend *fe)
{
	struct rda5815m_priv *priv = fe->tuner_priv;

	fe->tuner_priv = NULL;
	kfree(priv);

	return 0;
}

static
#endif
int rda5815m_sleep(struct dvb_frontend *fe)
{
	int ret = 0;

#ifndef DUAL_TUNER
	struct rda5815m_priv *priv = fe->tuner_priv;
#else
	struct dual_tuner_priv *priv = fe->tuner_priv;
#endif
	u8 val;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	dev_dbg(&priv->i2c->dev, "%s() start\n", __func__);

	ret = rda5815m_read_reg(priv, 0x04, &val);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed read reg = 0x04\n", __func__);
		goto ret_sleep;
	}

	val &= ~(1 << 7); /* set sleep mode */

	ret = rda5815m_write_reg(priv, 0x04, val);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x04\n", __func__);
	}

	msleep(5);

ret_sleep:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return ret;
}

static int Pow(int t, int k) /* t ^ k */
{
	int res = 1;

	while (k)
	{
		if (k & 1)
			res *= t;
		t *= t;
		k >>= 1;
	}

	return res;
}

#ifndef DUAL_TUNER
static
#endif
int rda5815m_set_params(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
#ifndef DUAL_TUNER
	struct rda5815m_priv *priv = fe->tuner_priv;
#else
	struct dual_tuner_priv *priv = fe->tuner_priv;
#endif
	int ret = 0;
	u32 freq = p->frequency / 1000;
	u32 bandwidth = p->symbol_rate / 1000;
	u32 tmp_val = 0;
	u8 val;

	dev_dbg(&priv->i2c->dev,
		"%s: delivery_system = %d frequency = %d symbol_rate = %d\n",
		__func__, p->delivery_system, p->frequency, p->symbol_rate);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = rda5815m_write_reg(priv, 0x04, 0xc1); /* Set RXON = 0, power = ON, AGC enable */
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x04\n", __func__);
		goto ret_set;
	}
	ret = rda5815m_write_reg(priv, 0x2b, 0x95);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x2b\n", __func__);
		goto ret_set;
	}

	/* Set frequency start */
	tmp_val = freq * (Pow(2, 21) / priv->cfg->xtal_freq);

	val = (u8)((tmp_val >> 24) & 0xff);
	ret = rda5815m_write_reg(priv, 0x07, val);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x07\n", __func__);
		goto ret_set;
	}
	val = (u8)((tmp_val >> 16) & 0xff);
	ret = rda5815m_write_reg(priv, 0x08, val);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x08\n", __func__);
		goto ret_set;
	}
	val = (u8)((tmp_val >> 8) & 0xff);
	ret = rda5815m_write_reg(priv, 0x09, val);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x09\n", __func__);
		goto ret_set;
	}
	val = (u8)(tmp_val & 0xff);
	ret = rda5815m_write_reg(priv, 0x0a, val);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x0a\n", __func__);
		goto ret_set;
	}
	/* Set frequency end */

	/* Set filter bandwidth start */
	if (bandwidth < 4000)
		bandwidth = 4000;
	else if (bandwidth > 45000)
		bandwidth = 40000;

	val = (u8)((bandwidth * 135 / 200 + 4000) / 1000);
	val &= 0x3f;
	ret = rda5815m_write_reg(priv, 0x0b, val);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write reg = 0x0b\n", __func__);
		goto ret_set;
	}
	/* Set filter bandwidth end */

	ret = rda5815m_write_reg(priv, 0x04, 0xc3); /* Set RXON = 1, power = ON, AGC enable */
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write _reg = 0x04\n", __func__);
		goto ret_set;
	}
	ret = rda5815m_write_reg(priv, 0x2b, 0x97);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s() failed write _reg = 0x2b\n", __func__);
		goto ret_set;
	}

	msleep(5);
	ret = 0;

ret_set:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return ret;
}

#ifndef DUAL_TUNER
static const struct dvb_tuner_ops rda5815m_tuner_ops = {
	.info = {
		.name		= "RDA Microelectronics RDA5815M",

		.frequency_min = 950000,
		.frequency_max = 2150000,
		.frequency_step = 0,
	},

	.release		= rda5815m_release,
	.init			= rda5815m_init,
	.sleep			= rda5815m_sleep,
	.set_params		= rda5815m_set_params,
};

struct dvb_frontend *rda5815m_attach(struct dvb_frontend *fe,
		struct rda5815m_config *cfg, struct i2c_adapter *i2c)
{
	struct rda5815m_priv *priv = NULL;
	int ret = 0;
	u8 val = 0;

	priv = kzalloc(sizeof(struct rda5815m_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err2;

	priv->cfg = cfg;
	priv->i2c = i2c;

	if (!priv->cfg->xtal_freq)
		priv->cfg->xtal_freq = 27; /* default set 27 MHz */

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = rda5815m_read_reg(priv, 0x00, &val);

	if (ret) {
		dev_err(&i2c->dev, "ERROR: %s() failed read reg = 0x00, exited...\n", __func__);
		goto err1;
	}

	if (val != 0x58) {
		dev_err(&i2c->dev, "ERROR: %s() RDA5815M unable to identify device(id = 0x%02X), exited...\n", __func__, val);
		goto err1;
	}

	dev_info(&i2c->dev, "RDA5815M detected id = 0x%02X\n", val);

	ret = rda5815m_read_reg(priv, 0x01, &val);
	if (!ret)
		dev_info(&i2c->dev, "RDA5815M Chip Rev. = 0x%02X\n", val);
	else
		goto err1;

	ret = rda5815m_read_reg(priv, 0x02, &val);
	if (!ret)
		dev_info(&i2c->dev, "RDA5815M recommended use I2C address = 0x%02X\n", val);
	else
		goto err1;

	memcpy(&fe->ops.tuner_ops, &rda5815m_tuner_ops,
			sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;

	dev_info(&i2c->dev,
		"%s: RDA5815M tuner successfully attached\n",
		KBUILD_MODNAME);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return fe;

err1:
	kfree(priv);
err2:
	dev_err(&i2c->dev, "%s() attach failed\n", __func__);
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return NULL;
}
EXPORT_SYMBOL(rda5815m_attach);

MODULE_DESCRIPTION("RDA Microelectronics RDA5815M tuner driver");
MODULE_AUTHOR("Igor Mokrushin <mcmcc@mail.ru>");
MODULE_LICENSE("GPL");
#endif
