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
#ifndef TUNER_FTM4862_H
#define TUNER_FTM4862_H

#ifdef DUAL_TUNER
#include <linux/kconfig.h>
#include "media/dvb_frontend.h"
#include <linux/i2c.h>
#include <linux/types.h>
#include "tuner-i2c.h"

#include "mxl608.h"
#include "rda5815m.h"

struct dual_tuner_priv {
	struct mxl608_config *config;
	struct rda5815m_config *cfg;

	struct i2c_adapter *i2c;

	u32 frequency;
	u32 bandwidth;
};

struct ftm4862_config {
	int reserved;
};

/* RDA5815M */
int rda5815m_set_params(struct dvb_frontend *fe);
int rda5815m_sleep(struct dvb_frontend *fe);
int rda5815m_init(struct dvb_frontend *fe);
int rda5815m_read_reg(struct dual_tuner_priv *priv, u8 reg, u8 *val);
int rda5815m_write_reg(struct dual_tuner_priv *priv, u8 reg, u8 val);

/* MXL608 */
int mxl608_sleep(struct dvb_frontend *fe);
int mxl608_init(struct dvb_frontend *fe);
int mxl608_set_params(struct dvb_frontend *fe);
int mxl608_get_status(struct dvb_frontend *fe, u32 *status);
int mxl608_get_if_frequency(struct dvb_frontend *fe, u32 *frequency);
int mxl608_read_reg(struct dual_tuner_priv *state, u8 reg, u8 *val);
int mxl608_write_reg(struct dual_tuner_priv *state, u8 reg, u8 val);
int mxl608_get_chip_id(struct dual_tuner_priv *state);

extern struct dvb_frontend *ftm4862_attach(struct dvb_frontend *fe,
					    struct ftm4862_config *cfg,
					    struct i2c_adapter *i2c);
#endif /* DUAL_TUNER */
#endif /* TUNER_FTM4862_H */
