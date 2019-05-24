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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/dvb/frontend.h>
#include "media/dvb_frontend.h"
#include "mn88436.h"

struct mn88436_state { 
	struct dvb_frontend frontend;
	struct i2c_adapter   *i2c;
	enum fe_modulation current_modulation;
	u32 current_frequency;	
	u8 mn88436_bank[DMD_REG_BANK];
	bool boot;
};
static int mn88436_write_reg(struct mn88436_state *state, u8 id, u8 reg, u8 val)
{
	int ret;
	u8 buf[] = { reg, val };
	struct i2c_msg msg = {	.addr = state->mn88436_bank[id], 
							.flags = 0,
							.buf = buf, 
							.len = 2 
						};
	
		
	ret = i2c_transfer(state->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&state->i2c->dev, "i2c wr failed=%d reg=%02x "
				, ret, reg);				
		ret = -EREMOTEIO;
	}
	return ret;
}
static int mn88436_read_reg(struct mn88436_state *state, u8 id, u8 reg, u8 *val)
{	
	int ret;
	u8 buf[] = { reg };
	struct i2c_msg msg[] = {
		{	.addr = state->mn88436_bank[id], 
			.flags = 0,
			.buf = buf, 
			.len = 1 
		},
		{ 	.addr = state->mn88436_bank[id], 
			.flags = I2C_M_RD,
			.buf = val, 
			.len = 1 
		}, 
	};

	ret = i2c_transfer(state->i2c, msg, 2);
	if (ret == 2) {
		ret = 0;
	} else {
		dev_warn(&state->i2c->dev, "i2c rd failed=%d reg=%02x "
				, ret, reg);
		ret = -EREMOTEIO;
	}
	return ret;
}
static int mn88436_write_reg_mask(struct mn88436_state *state, u8 id,
								u8 reg , u8 mask , u8 data)
{
	int ret;
	u8 rd;
	
	ret = mn88436_read_reg(state, id, reg, &rd);
	if (ret)
		goto err;
	
	rd |= mask & data;
	rd &= (mask ^ 0xff) | data;
	
	ret = mn88436_write_reg(state, id, reg, rd);
	if (ret)
		goto err;
	
	return 0;
	
err:
	dev_dbg(&state->i2c->dev, "%s: failed=%d\n", __func__, ret);
	return ret;
}


static int mn88436_read_status(struct dvb_frontend* fe, enum fe_status* status)
{
	struct mn88436_state* state = fe->demodulator_priv;	
	int ret;
	u8 locked;
	*status = 0;
	
	ret = mn88436_read_reg(state, 0, DMD_MAIN_STSMON1, &locked);
	if (ret)
		goto err;

	if (locked & 1) 
		*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER | 
				FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		
	return 0;
err:
	dev_dbg(&state->i2c->dev, "%s: failed=%d\n", __func__, ret);
	return ret;
	
}
static int mn88436_set_frontend(struct dvb_frontend* fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mn88436_state* state = fe->demodulator_priv;
	int cnt = 50, ret;
	u8 locked;
	
	if (!state->boot) {
		ret = -EAGAIN;
		goto err;
	}

	if (fe->ops.tuner_ops.set_params) {
		ret = fe->ops.tuner_ops.set_params(fe);
		if (ret)
			goto err;
	}
	
	ret = mn88436_write_reg(state, 0, DMD_MAIN_RSTSET1, 0x77);
	if (ret)
		goto err;
			
		
	do {
		ret = mn88436_read_reg(state, 0, DMD_MAIN_STSMON1, &locked);

		if (!ret && (locked & 1))
				break;

		msleep(10);   
		
	} while (--cnt);
	
	if (!cnt) {
		ret = -EAGAIN;
		goto err;
	}
	
	dev_dbg(&state->i2c->dev, "Service locked!!!\n");
	
	state->current_frequency = p->frequency;
	state->current_modulation = p->modulation;
	
	return 0;
	
err:
	dev_dbg(&state->i2c->dev, "%s: failed=%d\n", __func__, ret);
	return ret;
}
static int mn88436_get_frontend(struct dvb_frontend *fe, struct dtv_frontend_properties *p)
{
//	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct mn88436_state *state = fe->demodulator_priv;

	p->modulation = state->current_modulation;
	p->frequency = state->current_frequency;
	
	return 0;
}


static int mn88436_init(struct dvb_frontend* fe)
{
	struct mn88436_state* state = fe->demodulator_priv;
	const struct firmware *fw = NULL;	
	int ret, i;
	u8 d;
	
	if (state->boot)
		return 0;
		
	dev_dbg(&state->i2c->dev, "Uploading demod firmware (%s)...\n", MN88436_DEMOD_ATSC);
	
	ret = request_firmware(&fw, MN88436_DEMOD_ATSC, &state->i2c->dev);
	if (ret) {
		dev_dbg(&state->i2c->dev, "Firmware upload failed. Timeout or file not found\n");
		goto err1;
	}    
	
	for (i = 0;;) {
	
		if (fw->data[i] == 0xff) 
			break;
			
		ret = mn88436_write_reg(state, fw->data[i], fw->data[i + 1], fw->data[i + 2]);
		if (ret)
			goto err2;
					
		i = i + 3;		
	}
	
	release_firmware(fw);
	fw = NULL;
	
	dev_dbg(&state->i2c->dev, "Uploading demod pseq (%s)...\n", MN88436_DEMOD_PSEQ);
	ret = request_firmware(&fw, MN88436_DEMOD_PSEQ, &state->i2c->dev);
	if (ret) {
		dev_dbg(&state->i2c->dev, "Pseq upload failed. Timeout or file not found\n");
		goto err1;
	}    
	
	/* Load PSEQ Program */
	ret = mn88436_write_reg(state, 0, DMD_MAIN_PSEQSET , 0x03);
	if (ret)
		goto err2;
	
	for (i = 0; i < fw->size; i++) {
		ret = mn88436_write_reg(state, 0, DMD_MAIN_PSEQPRG , fw->data[i]);
		if (ret)
			goto err2;
	}
	
	release_firmware(fw);
	fw = NULL;
	
	/* Check Parity bit */
	ret = mn88436_read_reg(state, 0, DMD_MAIN_PSEQSET , &d);
	if (ret)
		goto err1;
	
	if (d & 0x20) {
		ret = -EAGAIN;
		goto err1;
	}
	
	ret = mn88436_write_reg(state, 0, DMD_MAIN_PSEQSET , 0x00);
	if (ret)
		goto err1;
		
		
	/* TS parallel (Fixed clock mode) */
	ret = mn88436_write_reg(state, 0, DMD_MAIN_CPOSET2, 0xc1);
	if (ret)
		goto err1;
	ret = mn88436_write_reg(state, 0, DMD_MAIN_GPSET1, 0xff);
	if (ret)
		goto err1;
		

	/* Set TCB Through Mode */
	ret = mn88436_write_reg_mask(state, 0, DMD_MAIN_TCBSET, 0x7f, 0x53);	
	if (ret)
		goto err1;		
	ret = mn88436_write_reg(state, 0, DMD_MAIN_TCBADR, 0x00);
	if (ret)
		goto err1;
	
	
	ret = mn88436_write_reg(state, 0, DMD_MAIN_VEQSET2, 0x80);
	if (ret)
		goto err1;
		
	state->boot = true;
	
	return 0;
	
err2:
	release_firmware(fw);
	fw = NULL;
err1:
	dev_dbg(&state->i2c->dev, "%s: failed=%d\n", __func__, ret);
	return ret;
	
}
static void mn88436_release(struct dvb_frontend* fe)
{
	struct mn88436_state* state = fe->demodulator_priv;
	kfree(state);
}


static struct dvb_frontend_ops mn88436_ops = {
	.delsys = { SYS_ATSC },
	.info = {
		.name = "Panasonic MN88436",		
		.frequency_min_hz = 51 * MHz,
		.frequency_max_hz = 858 * MHz,
		.caps = FE_CAN_8VSB
	},
	.init = mn88436_init,
	.release = mn88436_release,	
	.set_frontend = mn88436_set_frontend,
	.get_frontend = mn88436_get_frontend,	
	.read_status = mn88436_read_status,
	
};

struct dvb_frontend *mn88436_attach(struct i2c_adapter *i2c, 
									u8 device_id)
{
	struct mn88436_state *state = NULL;
	int ret;
	
	
	state = kzalloc(sizeof(struct mn88436_state), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		dev_err(&i2c->dev, "kzalloc() failed\n");
		goto err1;
	}

	state->i2c = i2c;
	
	switch (device_id) {
	case 0:
	default:
		state->mn88436_bank[0] = 0x18;
		state->mn88436_bank[1] = 0x10;
		break;
	case 1:
		state->mn88436_bank[0] = 0x19;
		state->mn88436_bank[1] = 0x11;
		break;
	case 2:
		state->mn88436_bank[0] = 0x1A;
		state->mn88436_bank[1] = 0x12;
		break;
	case 3:
		state->mn88436_bank[0] = 0x1B;
		state->mn88436_bank[1] = 0x13;
		break;		
	}
	/* Try SOFT reset */
	ret = mn88436_write_reg(state, 0, DMD_MAIN_RSTSET1, 0x77);
	if (ret)
		goto err2;
	
	dev_info(&i2c->dev, "MN88436 ATSC successfully attached\n");						
		
	memcpy(&state->frontend.ops, &mn88436_ops,
		       sizeof(struct dvb_frontend_ops));
			   
	state->frontend.demodulator_priv = state;
	
	return &state->frontend;
	
err2:	
	kfree(state);
err1:	
	dev_dbg(&i2c->dev, "%s: failed=%d\n", __func__, ret);	
	return NULL;
}
EXPORT_SYMBOL(mn88436_attach);

MODULE_DESCRIPTION("Panasonic MN88436 ATSC demod driver");
MODULE_AUTHOR("Sasa Savic <sasa.savic.sr@gmail.com>");
MODULE_LICENSE("GPL");
