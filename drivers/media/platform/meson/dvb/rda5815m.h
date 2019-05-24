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
#ifndef RDA5815M_H
#define RDA5815M_H

#include <linux/kconfig.h>
#include "media/dvb_frontend.h"

struct rda5815m_config {
	/* tuner i2c address */
	u8 i2c_address; /* 0x18/0x1e: use as 7-bit 0x0c/0x0f */

	/* crystal freq in MHz */
	u8 xtal_freq; /* default set is 27 */
};

#ifndef DUAL_TUNER
extern struct dvb_frontend *rda5815m_attach(struct dvb_frontend *fe,
			struct rda5815m_config *cfg, struct i2c_adapter *i2c);
#endif

#endif /* RDA5815M_H */
