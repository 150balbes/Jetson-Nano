/*
 * Driver for FTM4862 tuners.
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

#include "tuner_ftm4862.h"

static struct mxl608_config mxl608_config = {
	.xtal_freq_hz = MXL608_XTAL_24MHz,
	.if_freq_hz = MXL608_IF_5MHz,
	.agc_type = MXL608_AGC_SELF,
	.i2c_address = 0x60,
	.xtal_cap = 16,
	.gain_level = 11,
	.if_out_gain_level = 11,
	.agc_set_point = 66,
	.agc_invert_pol = 0,
	.invert_if = 1,
	.loop_thru_enable = 0,
	.clk_out_enable = 1,
	.clk_out_div = 0,
	.clk_out_ext = 0,
	.xtal_sharing_mode = 0,
	.single_supply_3_3V = 1,
};

static struct rda5815m_config rda5815m_config = {
	.i2c_address = 0x0c,
};

static int ftm4862_init(struct dvb_frontend *fe)
{
	int ret = 0;

	ret = rda5815m_init(fe);
	ret |= mxl608_init(fe);

	return ret;
}

static int ftm4862_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct dual_tuner_priv *state = fe->tuner_priv;

	*frequency = state->frequency;

	return 0;
}

static int ftm4862_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct dual_tuner_priv *state = fe->tuner_priv;

	*bandwidth = state->bandwidth;

	return 0;
}

static int ftm4862_get_if_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	int ret = 0;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	*frequency = 0;

	switch (c->delivery_system) {
	case SYS_ATSC:
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_C:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBT:
	case SYS_DVBT2:
		ret = mxl608_get_if_frequency(fe, frequency);
		break;
	default:
		break;
	}

	return ret;
}

static int ftm4862_get_status(struct dvb_frontend *fe, u32 *status)
{
	int ret = 0;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	*status = 0;

	switch (c->delivery_system) {
	case SYS_ATSC:
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_C:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBT:
	case SYS_DVBT2:
		ret = mxl608_get_status(fe, status);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
	default:
		*status |= TUNER_STATUS_LOCKED;
		break;
	}

	return ret;
}

static void ftm4862_release(struct dvb_frontend *fe)
{
	struct dual_tuner_priv *state = fe->tuner_priv;

	fe->tuner_priv = NULL;
	kfree(state);
}

static int ftm4862_sleep(struct dvb_frontend *fe)
{
	int ret = 0;

	ret = rda5815m_sleep(fe);
	ret |= mxl608_sleep(fe);

	return ret;
}

static int ftm4862_set_params(struct dvb_frontend *fe)
{
	int ret = -1;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct dual_tuner_priv *state = fe->tuner_priv;

	dev_dbg(&state->i2c->dev,"delivery_system=%d frequency=%d " \
			"symbol_rate=%d bandwidth_hz=%d\n",
			c->delivery_system, c->frequency, c->symbol_rate,
			c->bandwidth_hz);

	switch (c->delivery_system) {
	case SYS_ATSC:
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_C:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBT:
	case SYS_DVBT2:
		ret = mxl608_set_params(fe);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
	default:
		ret = rda5815m_set_params(fe);
		break;
	}
	return ret;
}

static struct dvb_tuner_ops ftm4862_tuner_ops = {
	.info = {
		.name = "FTM4862 dual tuner MxL608 + RDA5815M"
	},

	.init              = ftm4862_init,
	.sleep             = ftm4862_sleep,
	.set_params        = ftm4862_set_params,
	.get_status        = ftm4862_get_status,
	.get_frequency     = ftm4862_get_frequency,
	.get_bandwidth     = ftm4862_get_bandwidth,
	.release           = ftm4862_release,
	.get_if_frequency  = ftm4862_get_if_frequency,
};

struct dvb_frontend *ftm4862_attach(struct dvb_frontend *fe,
	struct ftm4862_config *cfg, struct i2c_adapter *i2c)
{
	struct dual_tuner_priv *state = NULL;
	int ret = 0;
	u8 val = 0;

	state = kzalloc(sizeof(struct dual_tuner_priv), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		dev_err(&i2c->dev, "kzalloc() failed\n");
		goto err1;
	}

	state->config = &mxl608_config;
	state->cfg = &rda5815m_config;
	state->i2c = i2c;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = mxl608_get_chip_id(state);

	/* check return value of mxl608_get_chip_id */
	if (ret)
		goto err2;

	if (!state->cfg->xtal_freq)
		state->cfg->xtal_freq = 27; /* default set 27 MHz */

	ret = rda5815m_read_reg(state, 0x00, &val);
	if (ret) {
		dev_err(&i2c->dev, "ERROR: %s() failed read reg = 0x00, exited...\n", __func__);
		goto err2;
	}

	if (val != 0x58) {
		dev_err(&i2c->dev, "ERROR: %s() RDA5815M unable to identify device(id = 0x%02X), exited...\n", __func__, val);
		goto err2;
	}

	dev_info(&i2c->dev, "RDA5815M detected id = 0x%02X\n", val);

	ret = rda5815m_read_reg(state, 0x01, &val);
	if (!ret)
		dev_info(&i2c->dev, "RDA5815M Chip Rev. = 0x%02X\n", val);
	else
		goto err2;

	ret = rda5815m_read_reg(state, 0x02, &val);
	if (!ret)
		dev_info(&i2c->dev, "RDA5815M recommended use I2C address = 0x%02X\n", val);
	else
		goto err2;

	dev_info(&i2c->dev, "Attaching FTM4862\n");

	fe->tuner_priv = state;

	memcpy(&fe->ops.tuner_ops, &ftm4862_tuner_ops,
		sizeof(struct dvb_tuner_ops));

	return fe;
err2:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	kfree(state);
err1:
	return NULL;

}
EXPORT_SYMBOL(ftm4862_attach);

MODULE_DESCRIPTION("FTM4862 tuner driver");
MODULE_AUTHOR("Igor Mokrushin <mcmcc@mail.ru>");
MODULE_LICENSE("GPL");
