/*
 * ascot3.h
 *
 * Sony Ascot3 DVB-T/T2/C tuner driver
 *
 * Copyright (C) 2015 Sasa Savic <sasa.savic.sr@gmail.com>
 *
 * Based on ascot2e driver
 *
 * Copyright 2012 Sony Corporation
 * Copyright (C) 2014 NetUP Inc.
 * Copyright (C) 2014 Sergey Kozlov <serjk@netup.ru>
 * Copyright (C) 2014 Abylay Ospan <aospan@netup.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
  */

#ifndef __DVB_ASCOT3_H__
#define __DVB_ASCOT3_H__

#include <linux/kconfig.h>
#include <linux/dvb/frontend.h>
#include <linux/i2c.h>

/**
 * struct ascot3_config - the configuration of Ascot2E tuner driver
 * @i2c_address:	I2C address of the tuner
 * @xtal_freq_mhz:	Oscillator frequency, MHz
 */
struct ascot3_config {
	u8	i2c_address;
	u8	xtal_freq_mhz;
};

extern struct dvb_frontend *ascot3_attach(struct dvb_frontend *fe,
					const struct ascot3_config *config,
					struct i2c_adapter *i2c);
#endif
