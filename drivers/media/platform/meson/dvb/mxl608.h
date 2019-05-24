/*
 * Driver for the MaxLinear MxL608 tuner
 *
 * Copyright (C) 2014 Sasa Savic <sasa.savic.sr@gmail.com>
 * Copyright (C) 2018 McMCC <mcmcc@mail.ru>
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
#ifndef __MXL608_H__
#define __MXL608_H__

#include <linux/dvb/version.h>
#include "media/dvb_frontend.h"

enum mxl608_if_freq {
	MXL608_IF_3_65MHz,
	MXL608_IF_4MHz,
	MXL608_IF_4_1MHz,
	MXL608_IF_4_15MHz,
	MXL608_IF_4_5MHz,
	MXL608_IF_4_57MHz,
	MXL608_IF_5MHz,
	MXL608_IF_5_38MHz,
	MXL608_IF_6MHz,
	MXL608_IF_6_28MHz,
	MXL608_IF_7_2MHz,
	MXL608_IF_8_25MHz,
	MXL608_IF_35_25MHz,
	MXL608_IF_36MHz,
	MXL608_IF_36_15MHz,
	MXL608_IF_36_65MHz,
	MXL608_IF_44MHz,
};

enum mxl608_xtal_freq {
	MXL608_XTAL_16MHz,
	MXL608_XTAL_24MHz,
};

enum mxl608_agc {
	MXL608_AGC_SELF,
	MXL608_AGC_EXTERNAL,
};

struct mxl608_config {
	enum mxl608_xtal_freq xtal_freq_hz;
	enum mxl608_if_freq if_freq_hz;
	enum mxl608_agc agc_type;

	u8 i2c_address; /* i2c addr = 0x60 */
	u8 xtal_cap;
	u8 gain_level;
	u8 if_out_gain_level;
	u8 agc_set_point;

	u8 agc_invert_pol;
	u8 invert_if;
	u8 loop_thru_enable;
	u8 clk_out_enable;
	u8 clk_out_div;
	u8 clk_out_ext;
	u8 xtal_sharing_mode;
	u8 single_supply_3_3V;
};

#ifndef DUAL_TUNER
extern struct dvb_frontend *mxl608_attach(struct dvb_frontend *fe,
					    struct mxl608_config *cfg,
					    struct i2c_adapter *i2c);
#endif

#endif /* __MXL608_H__ */
