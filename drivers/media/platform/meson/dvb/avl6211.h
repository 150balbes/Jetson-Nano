/*
 * Driver for the Availink AVL6211+AV2011 DVB-S/S2 demod+tuner
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
 
#ifndef __AVL6211_H_
#define __AVL6211_H_

#include <linux/types.h>
#include <linux/i2c.h>


#define AVL6211_DEMOD_FW 		"dvb-fe-avl6211.fw" 

#define I2C_MAX_READ	64
#define I2C_MAX_WRITE	64


#define CI_FLAG_IQ_BIT							0x00000000
#define CI_FLAG_IQ_BIT_MASK						0x00000001
#define CI_FLAG_IQ_NO_SWAPPED					0x00000000
#define CI_FLAG_IQ_SWAPPED						0x00000001
#define CI_FLAG_IQ_AUTO_BIT_MASK				0x00000020  


#define CI_FLAG_IQ_AUTO_BIT						0x00000005
#define CI_FLAG_IQ_AUTO_BIT_AUTO				0x00000001

#define CI_FLAG_DVBS2_BIT						0x00000002
#define CI_FLAG_DVBS2_UNDEF						0x00000004
#define CI_FLAG_DVBS2_BIT_MASK					0x0000001c

#define CI_FLAG_MANUAL_LOCK_MODE_BIT			0x00000001
#define CI_FLAG_MANUAL_LOCK_MODE_BIT_MASK		0x00000002
#define CI_FLAG_LOCK_MODE_BIT_MASK				0x00000040


#define ENABLE_FAST_REACQ           0x01
#define DISABLE_FAST_REACQ          0x00
#define ENABLE_CCI                  0x03
#define DISABLE_CCI                 0x02
#define MAX_LOWIF_SR                5000000
#define IF_OFFSET                   500


/* Demod commands */
#define OP_RX_NOOP                  0x00
#define OP_RX_LD_DEFAULT            0x01
#define OP_RX_INIT_GO               0x02
#define OP_RX_RESET_BERPER          0x03
#define OP_RX_HALT                  0x04
#define OP_RX_SLEEP                 0x05
#define OP_RX_WAKE                  0x06
#define OP_RX_BLIND_SCAN            0x08
#define OP_RX_STDOUT_MODE           0x09
	
/* Diseqc status */
#define DISEQC_STATUS_UNINIT				0x00
#define DISEQC_STATUS_INIT					0x01
#define DISEQC_STATUS_CONTINUOUS			0x02
#define DISEQC_STATUS_TONE					0x03
#define DISEQC_STATUS_MOD					0x04

#define I2CM_CMD_LENGTH   0x14
#define I2CM_RSP_LENGTH   0x14

#define OP_I2CM_NOOP      0x00
#define OP_I2CM_INIT	  0x01
#define OP_I2CM_WRITE     0x02
#define OP_I2CM_READ      0x03

	

#define format_addr(X, Y)		\
	do {						\
		Y[0] =(u8)((X) >> 16);	\
		Y[1] =(u8)((X) >> 8);	\
		Y[2] =(u8)(X);			\
	} while (0)


#define format_16(X, Y)			\
	do {						\
		Y[0] =(u8)((X) >> 8);	\
		Y[1] =(u8)((X) & 0xFF);	\
	} while (0)


#define format_32(X, Y)			\
	do {						\
		Y[0] =(u8)((X) >> 24);	\
		Y[1] =(u8)((X) >> 16);	\
		Y[2] =(u8)((X) >> 8);	\
		Y[3] =(u8)((X) & 0xFF);	\
	} while (0)


struct avl6211_pllconf
{
	u16 m_uiClkf; /* Feedback clock divider */
	u16 m_uiClkr; /* Reference clock divider */
	u16 m_uiPllod; /* PLL output divider */
	u16 m_uiPllod2; /* PLL output divider 2 */
	u16 m_uiPllod3; /* PLL output divider 3 */
	u16 ref_freq; /* Reference clock in kHz */
	u16 demod_freq; /* Demod clock in 10kHz */
	u16 fec_freq; /* FEC clock in 10kHz */
	u16 mpeg_freq; /* MPEG clock in 10kHz */
};

struct avl6211_config
{
	u8 tuner_address; /* Tuner i2c address */
	u16 tuner_i2c_clock;	
	u8 demod_address;	/* The demodulator's i2c address  0x0C */ 
				
	u8 mpeg_pol; /* 0 - Falling, 1 - Rising */
	u8 mpeg_mode; /* 0 - Parallel, 1 - Serial */
	u8 mpeg_format; /* 0 - Default TS stream, 1 - TS stream plus parity format */
	
	u8 demod_refclk; /* Reference clock frequency selection */

	/* Serial data is output on pin */
	u8 mpeg_pin; /* 0 -  MPEG_DATA_0, 1 - MPEG_DATA_7 */
	
	u8 tuner_rfagc; /* 0 - Normal pol, 1 - Inverted pol */
	u8 tuner_spectrum; /* 0 - signal spectrum normal, 1 - signal spectrum inverted */
	
	u8 use_lnb_pin59; /* control 13/18V over demod GPIO pin59 */
	u8 use_lnb_pin60; /* control 13/18V over demod GPIO pin60 */

	int (*set_external_vol_gpio)(int *demod_id, int on); /* external 13/18V control */
};



extern struct dvb_frontend* avl6211_attach(struct i2c_adapter* i2c,
											struct avl6211_config* config,
											int id);

#endif