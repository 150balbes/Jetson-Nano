/*
 * Driver for the Panasonic MN88436 ATSC demodulator
 *
 * Copyright (C) 2014 Sasa Savic <sasa.savic.sr@gmail.com>
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
 
#ifndef __MN88436_H_
#define __MN88436_H_

#include <linux/types.h>
#include <linux/i2c.h>


#define MN88436_DEMOD_ATSC 		"dvb-fe-mn88436-atsc.fw" 
#define MN88436_DEMOD_PSEQ 		"dvb-fe-mn88436-pseq.fw" 

#define DMD_REG_BANK    		2

#define DMD_MAIN_CPOSET2		0x2
#define DMD_MAIN_GPSET1			0x5
#define DMD_MAIN_RSTSET1		0x10
#define DMD_MAIN_TCBSET			0x15
#define DMD_MAIN_TCBADR			0x17
#define DMD_MAIN_VEQSET2		0x69
#define DMD_MAIN_STSMON1		0xC4
#define DMD_MAIN_PSEQSET		0xF0
#define DMD_MAIN_PSEQPRG		0xF1


extern struct dvb_frontend *mn88436_attach(struct i2c_adapter *i2c,
											u8 device_id);

#endif