/*

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/

#ifndef AVL6862_H
#define AVL6862_H

#include <linux/dvb/frontend.h>
#include "media/dvb_frontend.h"

#define MAX_CHANNEL_INFO 256

struct avl6862_priv {
	struct i2c_adapter *i2c;
 	struct avl6862_config *config;
	struct dvb_frontend frontend;
	enum fe_delivery_system delivery_system;

	/* DVB-Tx */
	u16 g_nChannel_ts_total;
};

struct avl6862_config {
	int		i2c_id;        // i2c adapter id
	void		*i2c_adapter;  // i2c adapter
	u8		demod_address; // demodulator i2c address
	u8		tuner_address; // tuner i2c address
	unsigned char 	eDiseqcStatus;
	int             ts_serial;
	int		gpio_lock_led;
};

extern struct dvb_frontend *avl6862_attach(struct avl6862_config *config, struct i2c_adapter *i2c);

#endif /* AVL6862_H */
