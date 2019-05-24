/*
 * Availink avl6862 demod driver
 *
 * Copyright (C) 2015 Luis Alves <ljalvs@gmail.com>
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

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/gpio.h>

#include "media/dvb_frontend.h"
#include "avl6862.h"
#include "avl6862_priv.h"

#define dbg_avl(fmt, args...) \
	do {\
		if (debug_avl)\
			dev_info(&priv->i2c->dev, "%s: %s: " fmt "\n", KBUILD_MODNAME, __func__, ##args);\
	} while (0)
MODULE_PARM_DESC(debug_avl, "\n\t\t Enable AVL demodulator debug information");
static int debug_avl;
module_param(debug_avl, int, 0644);

static int avl6862_i2c_rd(struct avl6862_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
			.addr= priv->config->demod_address,
			.flags= I2C_M_RD,
			.len  = len,
			.buf  = buf,
	};
	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "%s: i2c rd failed=%d " \
				"len=%d\n", KBUILD_MODNAME, ret, len);
		ret = -EREMOTEIO;
	}
	return ret;
}


static int avl6862_i2c_wr(struct avl6862_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr= priv->config->demod_address,
		.flags = 0,
		.buf = buf,
		.len = len,
	};
	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "%s: i2c wr failed=%d " \
				"len=%d\n", KBUILD_MODNAME, ret, len);
		ret = -EREMOTEIO;
	}
	return ret;
}


static int avl6862_i2c_wrm(struct avl6862_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr= priv->config->demod_address,
		.flags = 0,  //I2C_M_NOSTART,	/*i2c_transfer will emit a stop flag, so we should send 2 msg together,
						// * and the second msg's flag=I2C_M_NOSTART, to get the right timing*/
		.buf = buf,
		.len = len,
	};
	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "%s: i2c wrm failed=%d " \
				"len=%d\n", KBUILD_MODNAME, ret, len);
		ret = -EREMOTEIO;
	}
	return ret;
}


/* write 32bit words at addr */
#define MAX_WORDS_WR_LEN	((MAX_II2C_WRITE_SIZE-3) / 4)
static int avl6862_i2c_wr_data(struct avl6862_priv *priv,
	u32 addr, u32 *data, int len)
{
	int ret = 0, this_len;
	u8 buf[MAX_II2C_WRITE_SIZE];
	u8 *p;

	while (len > 0) {
		p = buf;
		*(p++) = (u8) (addr >> 16);
		*(p++) = (u8) (addr >> 8);
		*(p++) = (u8) (addr);

		this_len = (len > MAX_WORDS_WR_LEN) ? MAX_WORDS_WR_LEN : len;
		len -= this_len;
		if (len)
			addr += this_len * 4;

		while (this_len--) {
			*(p++) = (u8) ((*data) >> 24);
			*(p++) = (u8) ((*data) >> 16);
			*(p++) = (u8) ((*data) >> 8);
			*(p++) = (u8) (*(data++));
		}

		if (len > 0)
			ret = avl6862_i2c_wrm(priv, buf, (int) (p - buf));
		else
			ret = avl6862_i2c_wr(priv, buf, (int) (p - buf));
		if (ret)
			break;
	}
	return ret;
}

static int avl6862_i2c_wr_reg(struct avl6862_priv *priv,
	u32 addr, u32 data, int reg_size)
{
	u8 buf[3 + 4];
	u8 *p = buf;

	*(p++) = (u8) (addr >> 16);
	*(p++) = (u8) (addr >> 8);
	*(p++) = (u8) (addr);

	switch (reg_size) {
	case 4:
		*(p++) = (u8) (data >> 24);
		*(p++) = (u8) (data >> 16);
	case 2:
		*(p++) = (u8) (data >> 8);
	case 1:
	default:
		*(p++) = (u8) (data);
		break;
	}

	return avl6862_i2c_wr(priv, buf, 3 + reg_size);
}

#define avl6862_WR_REG8(_priv, _addr, _data) \
	avl6862_i2c_wr_reg(_priv, _addr, _data, 1)
#define avl6862_WR_REG16(_priv, _addr, _data) \
	avl6862_i2c_wr_reg(_priv, _addr, _data, 2)
#define avl6862_WR_REG32(_priv, _addr, _data) \
	avl6862_i2c_wr_reg(_priv, _addr, _data, 4)


static int avl6862_i2c_rd_reg(struct avl6862_priv *priv,
	u32 addr, u32 *data, int reg_size)
{
	int ret;
	u8 buf[3 + 4];
	u8 *p = buf;

	*(p++) = (u8) (addr >> 16);
	*(p++) = (u8) (addr >> 8);
	*(p++) = (u8) (addr);
	ret = avl6862_i2c_wr(priv, buf, 3);
	ret |= avl6862_i2c_rd(priv, buf, reg_size);

	*data = 0;
	p = buf;

	switch (reg_size) {
	case 4:
		*data |= (u32) (*(p++)) << 24;
		*data |= (u32) (*(p++)) << 16;
	case 2:
		*data |= (u32) (*(p++)) << 8;
	case 1:
	default:
		*data |= (u32) *(p);
		break;
	}
	return ret;
}

#define avl6862_RD_REG8(_priv, _addr, _data) \
	avl6862_i2c_rd_reg(_priv, _addr, _data, 1)
#define avl6862_RD_REG16(_priv, _addr, _data) \
	avl6862_i2c_rd_reg(_priv, _addr, _data, 2)
#define avl6862_RD_REG32(_priv, _addr, _data) \
	avl6862_i2c_rd_reg(_priv, _addr, _data, 4)


static int avl6862_setup_pll(struct avl6862_priv *priv)
{
	int ret;

	// sys_pll
	ret = avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_divr, 2);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_divf, 99);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_divq, 7);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_range, 1);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_divq2, 11);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_divq3, 13);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_enable2, 0);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_sys_pll_enable3, 0);

	//mpeg_pll
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_divr, 0);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_divf, 35);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_divq, 7);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_range, 3);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_divq2, 11);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_divq3, 13);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_enable2, 0);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_mpeg_pll_enable3, 0);

	//adc_pll
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_divr, 2);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_divf, 99);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_divq, 7);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_range, 1);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_divq2, 11);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_divq3, 13);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_enable2, 1);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_adc_pll_enable3, 1);

	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_reset_register, 0);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_reset_register, 1);
	msleep(20);

	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_dll_out_phase, 96);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_dll_rd_phase, 0);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_deglitch_mode, 1);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_dll_init, 1);
	ret |= avl6862_WR_REG32(priv, hw_E2_AVLEM61_dll_init, 0);
	return ret;
}


#define DEMOD_WAIT_RETRIES	(100)
#define DEMOD_WAIT_MS		(20)
static int avl6862_wait_demod(struct avl6862_priv *priv)
{
	u32 cmd = 0;
	int ret, retry = DEMOD_WAIT_RETRIES;

	do {
		msleep(DEMOD_WAIT_MS);
		ret = avl6862_RD_REG16(priv,0x200 + rc_fw_command_saddr_offset, &cmd);
		if ((ret == 0) && (cmd == 0))
			return ret;
//		else
//			msleep(DEMOD_WAIT_MS);
	} while (--retry);
	ret = -EBUSY;

	return ret;
}

/* TODO remove one of the waits */
static int avl6862_exec_n_wait(struct avl6862_priv *priv, u8 cmd)
{
	int ret;
/*
	ret = avl6862_wait_demod(priv);
	if (ret)
		return ret;
*/
	ret = avl6862_WR_REG16(priv, 0x200 + rc_fw_command_saddr_offset, (u32) cmd);
	if (ret)
		return ret;
	return avl6862_wait_demod(priv);
}


#define DMA_MAX_TRIES	(20)
static int avl6862_patch_demod(struct avl6862_priv *priv, u32 *patch)
{
	int ret = 0;
	u8 unary_op, binary_op, addr_mode_op;
	u32 cmd, num_cmd_words, next_cmd_idx, num_cond_words, num_rvs;
	u32 condition = 0;
	u32 value = 0;
	u32 operation;
	u32 tmp_top_valid, core_rdy_word;
	u32 exp_crc_val, crc_result;
	u32 data = 0;
	u32 type, ref_addr, ref_size;
	u32 data_section_offset;
	u32 args_addr, src_addr, dest_addr, data_offset, length;
	u32 idx, len, i;
	u32 variable_array[PATCH_VAR_ARRAY_SIZE];

	for(i=0; i<PATCH_VAR_ARRAY_SIZE; i++)
		variable_array[i] = 0;

	//total_patch_len = patch[1];
	//standard = patch[2];
	idx = 3;
	args_addr = patch[idx++];
	data_section_offset = patch[idx++];
	/* reserved length */
	len = patch[idx++];
	idx += len;
	/* script length */
	len = patch[idx++];
	len += idx;

	while (idx < len) {
		num_cmd_words = patch[idx++];
		next_cmd_idx = idx + num_cmd_words - 1;
		num_cond_words = patch[idx++];
		if (num_cond_words == 0) {
			condition = 1;
		} else {
			for (i = 0; i < num_cond_words; i++) {
				operation = patch[idx++];
				value = patch[idx++];
				unary_op = (operation >> 8) & 0xff;
				binary_op = operation & 0xff;
				addr_mode_op = ((operation >> 16) & 0x3);

				if ((addr_mode_op == PATCH_OP_ADDR_MODE_VAR_IDX) &&
				    (binary_op != PATCH_OP_BINARY_STORE)) {
					value = variable_array[value]; //grab variable value
				}

				switch(unary_op) {
				case PATCH_OP_UNARY_LOGICAL_NEGATE:
					value = !value;
					break;
				case PATCH_OP_UNARY_BITWISE_NEGATE:
					value = ~value;
					break;
				default:
					break;
				}
				switch(binary_op) {
				case PATCH_OP_BINARY_LOAD:
					condition = value;
					break;
				case PATCH_OP_BINARY_STORE:
					variable_array[value] = condition;
					break;
				case PATCH_OP_BINARY_AND:
					condition = condition && value;
					break;
				case PATCH_OP_BINARY_OR:
					condition = condition || value;
					break;
				case PATCH_OP_BINARY_BITWISE_AND:
					condition = condition & value;
					break;
				case PATCH_OP_BINARY_BITWISE_OR:
					condition = condition | value;
					break;
				case PATCH_OP_BINARY_EQUALS:
					condition = condition == value;
					break;
				case PATCH_OP_BINARY_NOT_EQUALS:
					condition = condition != value;
					break;
				default:
					break;
				}
			}
		}

		avl6862_RD_REG32(priv, 0x29A648, &tmp_top_valid);
		avl6862_RD_REG32(priv, 0x0A0, &core_rdy_word);

		if (condition) {
			cmd = patch[idx++];
			switch(cmd) {
			case PATCH_CMD_PING:
				ret = avl6862_exec_n_wait(priv, AVL_FW_CMD_PING);
				num_rvs = patch[idx++];
				i = patch[idx];
				variable_array[i] = (ret == 0);
				break;
			case PATCH_CMD_VALIDATE_CRC:
				exp_crc_val = patch[idx++];
				src_addr = patch[idx++];
				length = patch[idx++];
				avl6862_WR_REG32(priv,0x200 + rc_fw_command_args_addr_iaddr_offset, args_addr);
				avl6862_WR_REG32(priv,args_addr+0, src_addr);
				avl6862_WR_REG32(priv,args_addr+4, length);
				ret = avl6862_exec_n_wait(priv, AVL_FW_CMD_CALC_CRC);
				avl6862_RD_REG32(priv,args_addr+8, &crc_result);
				num_rvs = patch[idx++];
				i = patch[idx];
				variable_array[i] = (crc_result == exp_crc_val);
				break;
			case PATCH_CMD_LD_TO_DEVICE:
				length = patch[idx++];
				dest_addr = patch[idx++];
				data_offset = patch[idx++];
				data_offset += data_section_offset;
				ret = avl6862_i2c_wr_data(priv, dest_addr, &patch[data_offset], length);
				num_rvs = patch[idx++];
				break;
			case PATCH_CMD_LD_TO_DEVICE_IMM:
				length = patch[idx++];
				dest_addr = patch[idx++];
				data = patch[idx++];
				ret = avl6862_i2c_wr_reg(priv, dest_addr, data, length);
				num_rvs = patch[idx++];
				break;
			case PATCH_CMD_RD_FROM_DEVICE:
				length = patch[idx++];
				src_addr = patch[idx++];
				num_rvs = patch[idx++];
				ret = avl6862_i2c_rd_reg(priv, src_addr, &data, length);
				i = patch[idx];
				variable_array[i] = data;
				break;
			case PATCH_CMD_DMA:
				dest_addr = patch[idx++];
				length = patch[idx++];
				if (length > 0)
					ret = avl6862_i2c_wr_data(priv, dest_addr, &patch[idx], length * 3);
				avl6862_WR_REG32(priv,0x200 + rc_fw_command_args_addr_iaddr_offset, dest_addr);
				ret = avl6862_exec_n_wait(priv,AVL_FW_CMD_DMA);
				idx += length * 3;
				num_rvs = patch[idx++];
				break;
			case PATCH_CMD_DECOMPRESS:
				type = patch[idx++];
				src_addr = patch[idx++];
				dest_addr = patch[idx++];
				if(type == PATCH_CMP_TYPE_ZLIB) {
					ref_addr = patch[idx++];
					ref_size = patch[idx++];
				}
				avl6862_WR_REG32(priv,0x200 + rc_fw_command_args_addr_iaddr_offset, args_addr);
				avl6862_WR_REG32(priv,args_addr+0, type);
				avl6862_WR_REG32(priv,args_addr+4, src_addr);
				avl6862_WR_REG32(priv,args_addr+8, dest_addr);
				if(type == PATCH_CMP_TYPE_ZLIB) {
					avl6862_WR_REG32(priv,args_addr+12, ref_addr);
					avl6862_WR_REG32(priv,args_addr+16, ref_size);
				}
				ret = avl6862_exec_n_wait(priv,AVL_FW_CMD_DECOMPRESS);
				num_rvs = patch[idx++];
				break;
			case PATCH_CMD_ASSERT_CPU_RESET:
				ret |= avl6862_WR_REG32(priv,0x110840, 1);
				num_rvs = patch[idx++];
				break;
			case PATCH_CMD_RELEASE_CPU_RESET:
				avl6862_WR_REG32(priv, 0x110840, 0);
				num_rvs = patch[idx++];
				break;
			case PATCH_CMD_DMA_HW:
				dest_addr = patch[idx++];
				length = patch[idx++];
				if (length > 0)
					ret = avl6862_i2c_wr_data(priv, dest_addr, &patch[idx], length * 3);
				i = 0;
				do {
					if (i++ > DMA_MAX_TRIES)
						return -ENODEV;
					ret |= avl6862_RD_REG32(priv, 0x110048, &data);
				} while (!(0x01 & data));

				if (data)
					ret |= avl6862_WR_REG32(priv, 0x110050, dest_addr);
				idx += length * 3;
				num_rvs = patch[idx++];
				break;
			case PATCH_CMD_SET_COND_IMM:
				data = patch[idx++];
				num_rvs = patch[idx++];
				i = patch[idx];
				variable_array[i] = data;
				break;
			default:
				return -ENODEV;
				break;
			}
			idx += num_rvs;
		} else {
			idx = next_cmd_idx;
			continue;
		}
	}

	return ret;
}

#define DEMOD_WAIT_RETRIES_BOOT	(10)
#define DEMOD_WAIT_MS_BOOT	(20)
static int avl6862_wait_demod_boot(struct avl6862_priv *priv)
{
	int ret, retry = DEMOD_WAIT_RETRIES_BOOT;
	u32 ready_code = 0;
	u32 status = 0;

	do {
		ret = avl6862_RD_REG32(priv, 0x110840, &status); // hw_mcu_reset_base
		ret |= avl6862_RD_REG32(priv, rs_core_ready_word_iaddr_offset, &ready_code);
		dbg_avl("status:%d ready_code:0x%x", status, ready_code);
		if ((ret == 0) && (status == 0) && (ready_code == 0x5aa57ff7))
			return ret;
		else
			msleep(DEMOD_WAIT_MS_BOOT);
	} while (--retry);
	ret = -EBUSY;
	return ret;
}

/* firmware loader */
/* TODO: change to firmware loading from /lib/firmware */
static int avl6862_load_firmware(struct avl6862_priv *priv)
{
	int ret = 0;
	u8 *fw_data;
	u32 *patch, *ptr;
	u32 i, fw_size;

	switch (priv->delivery_system) {
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_B:
		fw_data = AVL_Demod_Patch_DVBCFw;
		fw_size = sizeof(AVL_Demod_Patch_DVBCFw);
		dev_info(&priv->i2c->dev, "Load avl6862 firmware patch for DVB-C size=%d", fw_size);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
		fw_data = AVL_Demod_Patch_DVBSxFw;
		fw_size = sizeof(AVL_Demod_Patch_DVBSxFw);
		dev_info(&priv->i2c->dev, "Load avl6862 firmware patch for DVB-S/S2 size=%d", fw_size);
		break;
	case SYS_DVBT:
	case SYS_DVBT2:
		fw_data = AVL_Demod_Patch_DVBTxFw;
		fw_size = sizeof(AVL_Demod_Patch_DVBTxFw);
		dev_info(&priv->i2c->dev, "Load avl6862 firmware patch for DVB-T/T2 size=%d", fw_size);
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	fw_size &= 0xfffffffc;
	patch = kzalloc(fw_size, GFP_KERNEL);
	if (patch == NULL) {
	  	ret = -ENOMEM;
		goto err;
	}

	ptr = patch;
	for (i = 0; i < fw_size; i += 4) {
	  	*(ptr++) = (fw_data[i]   << 24) |
			   (fw_data[i+1] << 16) |
			   (fw_data[i+2] <<  8) |
			   fw_data[i+3];
	}

	/* check valid FW */
	if ((patch[0] & 0xf0000000) != 0x10000000) {
		ret = -EINVAL;
		goto err1;
	}
	ret |= avl6862_WR_REG32(priv,0x110010, 1);

	// Configure the PLL
	ret |= avl6862_setup_pll(priv);
	if (ret)
		goto err1;

	ret |= avl6862_WR_REG32(priv, 0x0a4 + rs_core_ready_word_iaddr_offset, 0x00000000);
	ret |= avl6862_WR_REG32(priv, 0x110010, 0);

	if ((patch[0] & 0xff) == 1) /* patch version */
		ret |= avl6862_patch_demod(priv, patch);
	else
		ret = -EINVAL;
	if (ret)
		return ret;

	ret = avl6862_wait_demod_boot(priv);

	if (ret)
		return ret;
	
	ret |=  avl6862_RD_REG32(priv,0x0a4 + rs_patch_ver_iaddr_offset, &i);
	
	dev_info(&priv->i2c->dev, "avl6862 patch ver %d.%d build %d", (i>>24) & 0xFF, (i>>16) & 0xFF, i & 0xFFFF);

err1:
	kfree(patch);
err:
	return ret;
}



int  ErrorStatMode_Demod( struct avl6862_priv *priv,AVL_ErrorStatConfig stErrorStatConfig )
{
	int r = AVL_EC_OK;
	u64 time_tick_num = 270000 *  stErrorStatConfig.uiTimeThresholdMs;

	r = avl6862_WR_REG32(priv,0x132050 + esm_mode_offset,(u32) stErrorStatConfig.eErrorStatMode);
	r |= avl6862_WR_REG32(priv,0x132050 + tick_type_offset,(u32) stErrorStatConfig.eAutoErrorStatType);

	r |= avl6862_WR_REG32(priv,0x132050 + time_tick_low_offset, (u32) (time_tick_num));
	r |= avl6862_WR_REG32(priv,0x132050 + time_tick_high_offset, (u32) (time_tick_num >> 32));

	r |= avl6862_WR_REG32(priv,0x132050 + byte_tick_low_offset, stErrorStatConfig.uiTimeThresholdMs);
	r |= avl6862_WR_REG32(priv,0x132050 + byte_tick_high_offset, 0);//high 32-bit is not used

	if(stErrorStatConfig.eErrorStatMode == AVL_ERROR_STAT_AUTO)//auto mode
	{
		//reset auto error stat
		r |= avl6862_WR_REG32(priv,0x132050 + tick_clear_offset,0);
		r |= avl6862_WR_REG32(priv,0x132050 + tick_clear_offset,1);
		r |= avl6862_WR_REG32(priv,0x132050 + tick_clear_offset,0);
	}

	return (r);
}


int  ResetPER_Demod(  struct avl6862_priv *priv)
{
	int r = AVL_EC_OK;
	u32 uiTemp = 0;

	r |= avl6862_RD_REG32(priv,0x132050 + esm_cntrl_offset, &uiTemp);
	uiTemp |= 0x00000001;
	r |= avl6862_WR_REG32(priv,0x132050 + esm_cntrl_offset, uiTemp);

	r |= avl6862_RD_REG32(priv,0x132050 + esm_cntrl_offset, &uiTemp);
	uiTemp |= 0x00000008;
	r |= avl6862_WR_REG32(priv,0x132050 + esm_cntrl_offset, uiTemp);
	uiTemp |= 0x00000001;
	r |= avl6862_WR_REG32(priv,0x132050 + esm_cntrl_offset, uiTemp);
	uiTemp &= 0xFFFFFFFE;
	r |= avl6862_WR_REG32(priv,0x132050 + esm_cntrl_offset, uiTemp);

	return r;
}

static int InitErrorStat_Demod( struct avl6862_priv *priv )
{
	int r = AVL_EC_OK;
	AVL_ErrorStatConfig stErrorStatConfig;

	stErrorStatConfig.eErrorStatMode = AVL_ERROR_STAT_AUTO;
	stErrorStatConfig.eAutoErrorStatType = AVL_ERROR_STAT_TIME;
	stErrorStatConfig.uiTimeThresholdMs = 3000;
	stErrorStatConfig.uiNumberThresholdByte = 0;

	r = ErrorStatMode_Demod(priv,stErrorStatConfig);
	r |= ResetPER_Demod(priv);

	return r;
}

int  DVBSx_Diseqc_Initialize_Demod( struct avl6862_priv *priv,AVL_Diseqc_Para *pDiseqcPara)
{
	int r = AVL_EC_OK;
	u32 i1 = 0;

	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_srst_offset, 1);

	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_samp_frac_n_offset, 2000000); 	  //2M=200*10kHz
	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_samp_frac_d_offset, 166666667);  //uiDDCFrequencyHz  166666667

	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tone_frac_n_offset, ((pDiseqcPara->uiToneFrequencyKHz)<<1));
	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tone_frac_d_offset, (166666667/1000));//uiDDCFrequencyHz  166666667

	// Initialize the tx_control
	r |= avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, &i1);
	i1 &= 0x00000300;
	i1 |= 0x20; 	//reset tx_fifo
	i1 |= ((u32)(pDiseqcPara->eTXGap) << 6);
	i1 |= ((u32)(pDiseqcPara->eTxWaveForm) << 4);
	i1 |= (1<<3);			//enable tx gap.
	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);
	i1 &= ~(0x20);	//release tx_fifo reset
	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);

	// Initialize the rx_control
	i1 = ((u32)(pDiseqcPara->eRxWaveForm) << 2);
	i1 |= (1<<1);	//active the receiver
	i1 |= (1<<3);	//envelop high when tone present
	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_rx_cntrl_offset, i1);
	i1 = (u32)(pDiseqcPara->eRxTimeout);
	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_rx_msg_tim_offset, i1);

	r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_srst_offset, 0);

	if( AVL_EC_OK == r )
	{
		priv->config->eDiseqcStatus = AVL_DOS_Initialized;
	}

	return (r);
}


static int avl6862_init_dvbs(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;
	AVL_Diseqc_Para stDiseqcConfig;

	ret = avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_int_mpeg_clk_MHz_saddr_offset,27000);
	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_int_fec_clk_MHz_saddr_offset,25000);

	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_int_adc_clk_MHz_saddr_offset,12500);// uiADCFrequencyHz  125000000
	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_int_dmd_clk_MHz_saddr_offset,166666667/10000); //uiDDCFrequencyHz  166666667

	ret |= avl6862_WR_REG32(priv, 0xe00 + rc_DVBSx_rfagc_pol_iaddr_offset,AVL_AGC_INVERTED);

	ret |= avl6862_WR_REG32(priv, 0xe00 + rc_DVBSx_format_iaddr_offset, AVL_OFFBIN);//Offbin
	ret |= avl6862_WR_REG32(priv, 0xe00 + rc_DVBSx_input_iaddr_offset, AVL_ADC_IN);//ADC in

	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_IF_Offset_10kHz_saddr_offset,0);

	/* enble agc */
	ret |= avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_AGC_DVBS, 6);

	stDiseqcConfig.eRxTimeout = AVL_DRT_150ms;
	stDiseqcConfig.eRxWaveForm = AVL_DWM_Normal;
	stDiseqcConfig.uiToneFrequencyKHz = 22;
	stDiseqcConfig.eTXGap = AVL_DTXG_15ms;
	stDiseqcConfig.eTxWaveForm = AVL_DWM_Normal;

	ret |= DVBSx_Diseqc_Initialize_Demod(priv, &stDiseqcConfig);
	return ret;
}


static int avl6862_init_dvbc(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;

	ret = avl6862_WR_REG32(priv, 0x600 + rc_DVBC_dmd_clk_Hz_iaddr_offset, 250000000);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_fec_clk_Hz_iaddr_offset, 250000000);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_rfagc_pol_caddr_offset,AVL_AGC_NORMAL);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_if_freq_Hz_iaddr_offset, 5000000);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_adc_sel_caddr_offset, (u8) AVL_IF_Q);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_symbol_rate_Hz_iaddr_offset, 6875000);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_j83b_mode_caddr_offset, AVL_DVBC_J83A);

	//DDC configuration
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_input_format_caddr_offset, AVL_ADC_IN); //ADC in
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_input_select_caddr_offset, AVL_OFFBIN); //RX_OFFBIN
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_tuner_type_caddr_offset, AVL_DVBC_IF); //IF

	//ADC configuration
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_adc_use_pll_clk_caddr_offset, 0);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_sample_rate_Hz_iaddr_offset, 30000000);

	/* enable agc */
    	ret |= avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_AGC_DVBTC, 6);

	return ret;
}

static int avl6862_init_dvbc_b(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;

	ret = avl6862_WR_REG32(priv, 0x600 + rc_DVBC_dmd_clk_Hz_iaddr_offset, 250000000);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_fec_clk_Hz_iaddr_offset, 250000000);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_rfagc_pol_caddr_offset,AVL_AGC_NORMAL);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_if_freq_Hz_iaddr_offset, 5000000);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_adc_sel_caddr_offset, (u8) AVL_IF_Q);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_symbol_rate_Hz_iaddr_offset, 5360000);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_j83b_mode_caddr_offset, AVL_DVBC_J83B);

	//DDC configuration
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_input_format_caddr_offset, AVL_ADC_IN); //ADC in
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_input_select_caddr_offset, AVL_OFFBIN); //RX_OFFBIN
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_tuner_type_caddr_offset, AVL_DVBC_IF); //IF

	//ADC configuration
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_adc_use_pll_clk_caddr_offset, 0);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_sample_rate_Hz_iaddr_offset, 30000000);

	/* enable agc */
    	ret |= avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_AGC_DVBTC, 6);

	return ret;
}

static int avl6862_init_dvbt(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;

	ret = avl6862_WR_REG32(priv, 0xa00 + rc_DVBTx_sample_rate_Hz_iaddr_offset, 30000000);
	ret |= avl6862_WR_REG32(priv, 0xa00 + rc_DVBTx_mpeg_clk_rate_Hz_iaddr_offset, 270000000);

	/* DDC configuration */
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_input_format_caddr_offset, AVL_OFFBIN);
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_input_select_caddr_offset, AVL_ADC_IN);
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_tuner_type_caddr_offset, AVL_DVBTX_REAL_IF);
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_rf_agc_pol_caddr_offset, 0);
	ret |= avl6862_WR_REG32(priv, 0xa00 + rc_DVBTx_nom_carrier_freq_Hz_iaddr_offset, 5000000);

	/* ADC configuration */
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_adc_sel_caddr_offset, (u8)AVL_IF_Q);
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_adc_use_pll_clk_caddr_offset, 0);

	/* enable agc */
    	ret |= avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_AGC_DVBTC, 6);
	return ret;
}

static int avl6862_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;

	dev_dbg(&priv->i2c->dev, "%s: %d\n", __func__, enable);

	if (enable) {
		ret = avl6862_WR_REG32(priv,0x118000 + tuner_i2c_bit_rpt_cntrl_offset, 0x07);
		ret = avl6862_WR_REG32(priv,0x118000 + tuner_i2c_bit_rpt_cntrl_offset, 0x07);
	} else
		ret = avl6862_WR_REG32(priv,0x118000 + tuner_i2c_bit_rpt_cntrl_offset, 0x06);

	return ret;
}


static int avl6862_set_dvbs(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	dbg_avl("Freq:%d Mhz,sym:%d Khz", c->frequency, c->symbol_rate);

	ret = avl6862_WR_REG16(priv, 0xc00 + rs_DVBSx_fec_lock_saddr_offset, 0);
	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_decode_mode_saddr_offset, 0x14);
	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_fec_bypass_coderate_saddr_offset, 0); //DVBS auto lock
	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_iq_mode_saddr_offset, 1); //enable spectrum auto detection
	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_decode_mode_saddr_offset, 0x14);
	ret |= avl6862_WR_REG16(priv, 0xe00 + rc_DVBSx_fec_bypass_coderate_saddr_offset, 0);
	ret |= avl6862_WR_REG32(priv, 0xe00 + rc_DVBSx_int_sym_rate_MHz_iaddr_offset, c->symbol_rate);
	ret |= avl6862_exec_n_wait(priv,AVL_FW_CMD_ACQUIRE);
	return ret;
}


static int avl6862_set_dvbc(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;

	dbg_avl("Freq:%d Mhz,sym:%d", c->frequency, c->symbol_rate);

	ret = avl6862_WR_REG32(priv, 0x600 + rc_DVBC_qam_mode_scan_control_iaddr_offset, 0x0101);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_symbol_rate_Hz_iaddr_offset, c->symbol_rate);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_j83b_mode_caddr_offset, AVL_DVBC_J83A);
	ret |= avl6862_exec_n_wait(priv, AVL_FW_CMD_ACQUIRE);
	return ret;
}

static int avl6862_set_dvbc_b(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;

	dbg_avl("Freq:%d Mhz,sym:%d", c->frequency, c->symbol_rate);

	ret = avl6862_WR_REG32(priv, 0x600 + rc_DVBC_qam_mode_scan_control_iaddr_offset, 0x0101);
	ret |= avl6862_WR_REG32(priv, 0x600 + rc_DVBC_symbol_rate_Hz_iaddr_offset, c->symbol_rate);
	ret |= avl6862_WR_REG8(priv, 0x600 + rc_DVBC_j83b_mode_caddr_offset, AVL_DVBC_J83B);
	ret |= avl6862_exec_n_wait(priv, AVL_FW_CMD_ACQUIRE);
	return ret;
}


static int avl6862_set_dvbt(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u32 bw_fft;
	int ret;

	dbg_avl("Freq:%d bw:%d", c->frequency, c->bandwidth_hz);

	/* set bandwidth */
	if(c->bandwidth_hz <= 1700000) {
		bw_fft = 1845070;
	} else if(c->bandwidth_hz <= 5000000) {
		bw_fft = 5714285;
	} else if(c->bandwidth_hz <= 6000000) {
		bw_fft = 6857143;
	} else if(c->bandwidth_hz <= 7000000) {
		bw_fft = 8000000;
	} else { // if(c->bandwidth_hz <= 8000) {
		bw_fft = 9142857;
	}
    	ret = avl6862_WR_REG32(priv, 0xa00 + rc_DVBTx_fund_rate_Hz_iaddr_offset, bw_fft);
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_l1_proc_only_caddr_offset, 0);

	/* spectrum inversion */
	ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_spectrum_invert_caddr_offset, AVL_SPECTRUM_AUTO);

	switch (c->delivery_system) {
	case SYS_DVBT:
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_acquire_mode_caddr_offset, (u8) AVL_DVBTx_LockMode_T_ONLY);
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_dvbt_layer_select_caddr_offset, 0);
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_data_PLP_ID_caddr_offset, 0);
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_common_PLP_ID_caddr_offset, 0);
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_common_PLP_present_caddr_offset, 0);
		break;
	case SYS_DVBT2:
	default:
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_acquire_mode_caddr_offset, AVL_DVBTx_LockMode_ALL);
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_data_PLP_ID_caddr_offset, c->stream_id);
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_common_PLP_ID_caddr_offset, 0);
		ret |= avl6862_WR_REG8(priv, 0xa00 + rc_DVBTx_common_PLP_present_caddr_offset, 2);
		break;
	}
	ret |= avl6862_exec_n_wait(priv, AVL_FW_CMD_ACQUIRE);
	return ret;
}

#define I2C_RPT_DIV ((0x2A)*(250000)/(240*1000))	//m_CoreFrequency_Hz 250000000
#define uiTSFrequencyHz 270000000


static int avl6862_set_dvbmode(struct dvb_frontend *fe,
		enum fe_delivery_system delsys)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;
	u32 reg;

	/* these modes use the same fw / config */
	if (delsys == SYS_DVBS2)
		delsys = SYS_DVBS;
	else if (delsys == SYS_DVBT2)
		delsys = SYS_DVBT;

	/* already in desired mode */
	if (priv->delivery_system == delsys)
		return 0;

	dbg_avl("initing demod for delsys=%d", delsys);

	priv->delivery_system = delsys;

	ret = avl6862_load_firmware(priv);

	// Load the default configuration
	ret |= avl6862_exec_n_wait(priv, AVL_FW_CMD_LD_DEFAULT);
	ret |= avl6862_exec_n_wait(priv, AVL_FW_CMD_INIT_SDRAM);
	ret |= avl6862_exec_n_wait(priv, AVL_FW_CMD_INIT_ADC);

	switch (priv->delivery_system) {
	case SYS_DVBC_ANNEX_A:
		ret |= avl6862_init_dvbc(fe);
		break;
	case SYS_DVBC_ANNEX_B:
		ret |= avl6862_init_dvbc_b(fe);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
		ret |= avl6862_init_dvbs(fe);
		break;
	case SYS_DVBT:
	case SYS_DVBT2:
	default:
		ret |= avl6862_init_dvbt(fe);
		break;
	}

	/* set gpio / turn off lnb, set 13V */
	ret = avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_LNB_PWR, GPIO_1);
	ret |= avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_LNB_VOLTAGE, GPIO_0);

	/* set TS mode */
	if (priv->config->ts_serial) {
		ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_serial_caddr_offset, AVL_TS_SERIAL);
		dbg_avl("set AVL_TS_SERIAL");
	}
	else {
		ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_serial_caddr_offset, AVL_TS_PARALLEL);
		dbg_avl("set AVL_TS_PARALLEL");
	}

	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_clock_edge_caddr_offset, AVL_MPCM_RISING);
	ret |= avl6862_WR_REG8(priv,0x200 + rc_enable_ts_continuous_caddr_offset, AVL_TS_CONTINUOUS_ENABLE);
	ret |= avl6862_WR_REG32(priv,0x200 + rc_ts_cntns_clk_frac_d_iaddr_offset, uiTSFrequencyHz);
	if (priv->config->ts_serial) {
		switch (priv->delivery_system) {
		case SYS_DVBS:
		case SYS_DVBS2:
			ret |= avl6862_WR_REG32(priv,0x200 + rc_ts_cntns_clk_frac_n_iaddr_offset, uiTSFrequencyHz);
			break;
		case SYS_DVBT:
		case SYS_DVBT2:
		case SYS_DVBC_ANNEX_A:
		case SYS_DVBC_ANNEX_B:
		default:
			ret |= avl6862_WR_REG32(priv,0x200 + rc_ts_cntns_clk_frac_n_iaddr_offset, uiTSFrequencyHz / 2);
			break;
		}
	}
	else
		ret |= avl6862_WR_REG32(priv,0x200 + rc_ts_cntns_clk_frac_n_iaddr_offset, uiTSFrequencyHz / 8);

	/* TS serial pin */
	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_serial_outpin_caddr_offset, AVL_MPSP_DATA0);

	/* TS serial order */
	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_serial_msb_caddr_offset, AVL_MPBO_MSB);

	/* TS serial sync pulse */
	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_sync_pulse_caddr_offset, AVL_TS_SERIAL_SYNC_1_PULSE);
	/* TS error pol */
	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_error_polarity_caddr_offset, AVL_MPEP_Normal);
	/* TS valid pol */
	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_valid_polarity_caddr_offset, AVL_MPVP_Normal);
	/* TS packet len */
	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_packet_len_caddr_offset, AVL_TS_188);
	/* TS parallel order */
	ret |= avl6862_WR_REG8(priv,0x200 + rc_ts_packet_order_caddr_offset, AVL_TS_PARALLEL_ORDER_NORMAL);
	/* TS parallel phase */
	ret |= avl6862_WR_REG8(priv,0x200 + ts_clock_phase_caddr_offset, AVL_TS_PARALLEL_PHASE_0);

	/* TS output enable */
	ret |= avl6862_WR_REG32(priv, REG_TS_OUTPUT, TS_OUTPUT_ENABLE);

	/* init tuner i2c repeater */

	/* hold in reset */
	ret |= avl6862_WR_REG32(priv, 0x118000 + tuner_i2c_srst_offset, 1);
	/* close gate */
	ret |= avl6862_WR_REG32(priv, 0x118000 + tuner_i2c_bit_rpt_cntrl_offset, 0x6);
	ret |= avl6862_RD_REG32(priv, 0x118000 + tuner_i2c_cntrl_offset, &reg);
	reg &= 0xfffffffe;
	ret |= avl6862_WR_REG32(priv, 0x118000 + tuner_i2c_cntrl_offset, reg);
	/* set bit clock */
	ret |= avl6862_WR_REG32(priv, 0x118000 + tuner_i2c_bit_rpt_clk_div_offset, I2C_RPT_DIV);
	/* release from reset */
	ret |= avl6862_WR_REG32(priv, 0x118000 + tuner_i2c_srst_offset, 0);

	ret |= InitErrorStat_Demod(priv);

	if (ret) {
		dev_err(&priv->i2c->dev, "%s: demod init failed",
				KBUILD_MODNAME);
	}

	return ret;
}

static int AVL_Demod_DVBSx_Diseqc_SendModulationData(struct avl6862_priv *priv, AVL_puchar pucBuff, u8 ucSize)
{
	int r = 0;
	u32 i1 = 0;
	u32 i2 = 0;
	u8 Continuousflag = 0;
	u16 uiTempOutTh = 0;

	if (ucSize > 8) {
		r = AVL_EC_WARNING;
	} else {
		if (priv->config->eDiseqcStatus == AVL_DOS_InContinuous) {
			r |= avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, &i1);
			if ((i1>>10) & 0x01) {
				Continuousflag = 1;
				i1 &= 0xfffff3ff;
				r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);
				msleep(Diseqc_delay);		//delay 20ms
			}
		}
		//reset rx_fifo
		r |= avl6862_RD_REG32(priv, 0x16c000 + hw_diseqc_rx_cntrl_offset, &i2);
		r |= avl6862_WR_REG32(priv, 0x16c000 + hw_diseqc_rx_cntrl_offset, (i2|0x01));
		r |= avl6862_WR_REG32(priv, 0x16c000 + hw_diseqc_rx_cntrl_offset, (i2&0xfffffffe));

		r |= avl6862_RD_REG32(priv, 0x16c000 + hw_diseqc_tx_cntrl_offset, &i1);
		i1 &= 0xfffffff8;	//set to modulation mode and put it to FIFO load mode
		r |= avl6862_WR_REG32(priv, 0x16c000 + hw_diseqc_tx_cntrl_offset, i1);

		for (i2=0; i2 < ucSize; i2++) {
			r |= avl6862_WR_REG32(priv, 0x16c000 + hw_tx_fifo_map_offset, pucBuff[i2]);
		}

		i1 |= (1<<2);  //start fifo transmit.
		r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);

		if (AVL_EC_OK == r) {
			priv->config->eDiseqcStatus = AVL_DOS_InModulation;
		}
		do {
			msleep(3);
			if (++uiTempOutTh > 500) {
				r |= AVL_EC_TIMEOUT;
				return(r);
			}
			r = avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_st_offset, &i1);
		} while (1 != ((i1 & 0x00000040) >> 6));

		msleep(Diseqc_delay);		//delay 20ms
		if (Continuousflag == 1) {			//resume to send out wave
			//No data in FIFO
			r |= avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, &i1);
			i1 &= 0xfffffff8;
			i1 |= 0x03; 	//switch to continuous mode
			r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);

			//start to send out wave
			i1 |= (1<<10);
			r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);
			if (AVL_EC_OK == r) {
				priv->config->eDiseqcStatus = AVL_DOS_InContinuous;
			}
		}
	}
	return (r);
}

int  AVL_Demod_DVBSx_Diseqc_GetTxStatus( struct avl6862_priv *priv, AVL_Diseqc_TxStatus * pTxStatus)
{
	int r = 0;
	u32 i1 = 0;

	if( (AVL_DOS_InModulation == priv->config->eDiseqcStatus) || (AVL_DOS_InTone == priv->config->eDiseqcStatus) )
	{
		r |= avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_st_offset, &i1);
		pTxStatus->m_TxDone = (u8)((i1 & 0x00000040)>>6);
		pTxStatus->m_TxFifoCount = (u8)((i1 & 0x0000003c)>>2);
	}
	else
	{
		r |= AVL_EC_GENERAL_FAIL;
	}

	return (r);
}

int AVL_SX_DiseqcSendCmd(struct avl6862_priv *priv, AVL_puchar pCmd, u8 CmdSize)
{
	int r = AVL_EC_OK;
	struct AVL_Diseqc_TxStatus TxStatus;
	dbg_avl(" %*ph", CmdSize, pCmd);

	r = AVL_Demod_DVBSx_Diseqc_SendModulationData(priv, pCmd, CmdSize);
	if(r != AVL_EC_OK) {
		printk("AVL_SX_DiseqcSendCmd failed:%02x\n", r);
	} else {
		do {
			msleep(5);
			r |= AVL_Demod_DVBSx_Diseqc_GetTxStatus(priv,&TxStatus);
		} while(TxStatus.m_TxDone != 1);
		if (r == AVL_EC_OK ) {
		} else {
			printk("AVL_SX_DiseqcSendCmd Err.:%02x\n", r);
		}
	}
	return (r);
}


int  AVL_Demod_DVBSx_Diseqc_SendTone(struct avl6862_priv *priv, u8 ucTone, u8 ucCount)
{
	int r = 0;
	u32 i1 = 0;
	u32 i2 = 0;
	u8 Continuousflag = 0;
	u16 uiTempOutTh = 0;

	if( ucCount>8 )
	{
		r = AVL_EC_WARNING;
	}
	else
	{
			if (priv->config->eDiseqcStatus == AVL_DOS_InContinuous)
			{
				r |= avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, &i1);
				if ((i1>>10) & 0x01)
				{
					Continuousflag = 1;
					i1 &= 0xfffff3ff;
					r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);
					msleep(Diseqc_delay);		//delay 20ms
				}
			}
			//No data in the FIFO.
			r |= avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, &i1);
			i1 &= 0xfffffff8;  //put it into the FIFO load mode.
			if( 0 == ucTone )
			{
				i1 |= 0x01;
			}
			else
			{
				i1 |= 0x02;
			}
			r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);


			for (i2 = 0; i2 < ucCount; i2++) {
				r |= avl6862_WR_REG32(priv, 0x16c000 + hw_tx_fifo_map_offset, 1);
			}

			i1 |= (1<<2);  //start fifo transmit.
			r |= avl6862_WR_REG32(priv, 0x16c000 + hw_diseqc_tx_cntrl_offset, i1);
			if( AVL_EC_OK == r )
			{
				priv->config->eDiseqcStatus = AVL_DOS_InTone;
			}
			do 
			{
				msleep(1);
				if (++uiTempOutTh > 500)
				{
					r |= AVL_EC_TIMEOUT;
					return(r);
				}
				r = avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_st_offset, &i1);
			} while ( 1 != ((i1 & 0x00000040) >> 6) );

			msleep(Diseqc_delay);		//delay 20ms
			if (Continuousflag == 1)			//resume to send out wave
			{
				//No data in FIFO
				r |= avl6862_RD_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, &i1);
				i1 &= 0xfffffff8; 
				i1 |= 0x03; 	//switch to continuous mode
				r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);

				//start to send out wave
				i1 |= (1<<10);	
				r |= avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, i1);

			}
		}
	return (r);
}

static int avl6862_diseqc(struct dvb_frontend *fe,
	struct dvb_diseqc_master_cmd *cmd)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;

	ret = avl6862_set_dvbmode(fe,SYS_DVBS);
	if (ret)
	  return ret;

	return AVL_SX_DiseqcSendCmd(priv,cmd->msg,cmd->msg_len);
}

static int avl6862_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;

	ret = avl6862_set_dvbmode(fe,SYS_DVBS);
	if (ret)
	  return ret;

	ret = AVL_Demod_DVBSx_Diseqc_SendTone(priv,burst == SEC_MINI_A ? 1 : 0, 1);

	return ret;
}

static int avl6862_set_tone(struct dvb_frontend* fe, enum fe_sec_tone_mode tone)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;
	u32 reg;

	ret = avl6862_set_dvbmode(fe,SYS_DVBS);
	if (ret)
	  return ret;

	dbg_avl("tone: %d", tone);
	ret = avl6862_RD_REG32(priv, 0x16c000 + hw_diseqc_tx_cntrl_offset, &reg);
	if (ret)
		return ret;

	switch(tone) {
	case SEC_TONE_ON:
		reg &= 0xfffffff8;
		reg |= 0x3;	// continuous mode
		reg |= (1<<10);	// on
		break;
	case SEC_TONE_OFF:
		reg &= 0xfffff3ff;
		break;
	default:
		return -EINVAL;
	}
	return avl6862_WR_REG32(priv,0x16c000 + hw_diseqc_tx_cntrl_offset, reg);
}

static int avl6862_set_voltage(struct dvb_frontend* fe, enum fe_sec_voltage voltage)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	u32 pwr, vol;
	int ret;

	ret = avl6862_set_dvbmode(fe,SYS_DVBS);
	if (ret)
	  return ret;

	dbg_avl("volt: %d", voltage);

	switch (voltage) {
	case SEC_VOLTAGE_OFF:
		pwr = GPIO_0;
		vol = GPIO_0;
		break;
	case SEC_VOLTAGE_13:
		//power on
		pwr = GPIO_1;
		vol = GPIO_0;
		break;
	case SEC_VOLTAGE_18:
		//power on
		pwr = GPIO_1;
		vol = GPIO_Z;
		break;
	default:
		return -EINVAL;
	}
	ret = avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_LNB_PWR, pwr);
	ret |= avl6862_WR_REG32(priv, REG_GPIO_BASE + GPIO_LNB_VOLTAGE, vol);
	return ret;
}

struct Signal_Level
{
	u16 SignalLevel;
	short SignalDBM;
};
#define Level_High_Stage	36
#define Level_Low_Stage		76

#define Percent_Space_High	10
#define Percent_Space_Mid	30
#define Percent_Space_Low	60
	
static int avl6862_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;
	u32 reg = 0, agc, mul, snr = 0;
	u16 Level;
	int Percent = 0;
	int lock_led = priv->config->gpio_lock_led;

	switch (priv->delivery_system) {
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_B:
		ret |= avl6862_RD_REG32(priv,0x400 + rs_DVBC_mode_status_iaddr_offset, &reg);
		if ((reg & 0xff) == 0x15)
			reg = 1;
		else
		  	reg = 0;
		if (reg) {
			ret |= avl6862_RD_REG16(priv,0x400 + rs_DVBC_snr_dB_x100_saddr_offset, &snr);		  
			if (ret) snr = 0;
		}
		mul = 131;
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
		ret |= avl6862_RD_REG16(priv, 0xc00 + rs_DVBSx_fec_lock_saddr_offset, &reg);
		if (reg) {
			ret |= avl6862_RD_REG32(priv,0xc00 + rs_DVBSx_int_SNR_dB_iaddr_offset, &snr);		  
			if (ret || snr > 10000) snr = 0;
		} else { 
			*status = 0;
			return ret;
		}
		mul = 328;
		break;
	case SYS_DVBT:
	case SYS_DVBT2:
		ret |= avl6862_RD_REG8(priv, 0x800 + rs_DVBTx_fec_lock_caddr_offset, &reg);
		if (reg) {
			ret |= avl6862_RD_REG16(priv,0x800 + rs_DVBTx_snr_dB_x100_saddr_offset, &snr);		  
			if (ret) snr = 0;
		}
		mul = 131;
		break;
	default:
		*status = 0;
		return 1;
	}

	if (ret) {
	  	*status = 0;
		return ret;
	}
	*status = FE_HAS_SIGNAL;
	ret = avl6862_RD_REG16(priv,0x0a4 + rs_rf_agc_saddr_offset, &agc);

	c->strength.len = 2;
	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = - (s32)agc;
	Level = (s32)agc / 1000;
	
	if (Level <= Level_High_Stage)
		Percent = Percent_Space_Low+Percent_Space_Mid + (Level_High_Stage - Level) * Percent_Space_High / Level_High_Stage;
	else if (Level <= Level_Low_Stage)
		Percent = Percent_Space_Low+ (Level_Low_Stage - Level) * Percent_Space_Mid/ (Level_Low_Stage - Level_High_Stage);
	else
		Percent =(90 - Level) * Percent_Space_Low / (90 - Level_Low_Stage);

	c->strength.stat[1].scale = FE_SCALE_RELATIVE;
	c->strength.stat[1].uvalue = (Percent * 65535) / 100; //(100 - agc/1000) * 656;

	if (reg){
		*status |= FE_HAS_CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		c->cnr.len = 2;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		c->cnr.stat[0].svalue = snr * 10;
		c->cnr.stat[1].scale = FE_SCALE_RELATIVE;
		c->cnr.stat[1].uvalue = (snr / 10) * mul;
		if (c->cnr.stat[1].uvalue > 0xffff)
			c->cnr.stat[1].uvalue = 0xffff;
	}
	else
	{
		c->cnr.len = 1;
		c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}
	if (lock_led) {
		gpio_request(lock_led, KBUILD_MODNAME);
		gpio_direction_output(lock_led, snr > 1000 ? 1 : 0);
	}
	dbg_avl("Status:%x level:%d snr:%d", *status, Percent, snr);
	return ret;
}

static int avl6862_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	*strength = 0;
	for (i=0; i < c->strength.len; i++)
		if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
		  *strength = (u16)c->strength.stat[i].uvalue;

	return 0;

}

static int avl6862_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	*snr = 0;
	for (i=0; i < c->cnr.len; i++)
		if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
		  *snr = (u16)c->cnr.stat[i].uvalue;

	return 0;
}

static int avl6862_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	int ret;

	*ber = 10e7;

	switch (priv->delivery_system) {
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_B:
		ret = avl6862_RD_REG32(priv,0x400 + rs_DVBC_post_viterbi_BER_estimate_x10M_iaddr_offset, ber);
		break;
	case SYS_DVBS:
		ret = avl6862_RD_REG32(priv,0xc00 + rs_DVBSx_post_viterbi_BER_estimate_x10M_iaddr_offset, ber);
		break;
	case SYS_DVBS2:
		ret = avl6862_RD_REG32(priv,0xc00 + rs_DVBSx_post_LDPC_BER_estimate_x10M_iaddr_offset, ber);
		break;
	case SYS_DVBT:
		ret = avl6862_RD_REG32(priv,0x800 + rs_DVBTx_post_viterbi_BER_estimate_x10M_iaddr_offset, ber);
		break;
	case SYS_DVBT2:
		ret = avl6862_RD_REG32(priv,0x800 + rs_DVBTx_post_LDPC_BER_estimate_x1B_iaddr_offset, ber);
		if (!ret)
			*ber /= 100;
		break;
	default:
		ret = 1;
	}
	dbg_avl("BER:%d ret:%d", *ber, ret);
	return ret;
}

static int avl6862fe_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}
			

static int avl6862_set_frontend(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u32 demod_mode;
	int ret;
	int lock_led = priv->config->gpio_lock_led;

	if (lock_led) {
		gpio_request(lock_led, KBUILD_MODNAME);
		gpio_direction_output(lock_led, 0);
	}
	/* check that mode is correctly set */
	ret = avl6862_RD_REG32(priv, 0x200 + rs_current_active_mode_iaddr_offset, &demod_mode);
	if (ret)
		return ret;

	/* setup tuner */
	if (fe->ops.tuner_ops.set_params) {
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);
		ret = fe->ops.tuner_ops.set_params(fe);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);

		if (ret)
			return ret;
	}

	switch (c->delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		if (demod_mode != AVL_DVBTX)
			ret = avl6862_set_dvbmode(fe, c->delivery_system);
		ret = avl6862_RD_REG32(priv, 0x200 + rs_current_active_mode_iaddr_offset, &demod_mode);
		if (ret || demod_mode != AVL_DVBTX) {
			dev_err(&priv->i2c->dev, "%s: failed to enter DVBTx mode",
				KBUILD_MODNAME);
			ret = -EAGAIN;
			break;
		}
		ret = avl6862_set_dvbt(fe);
		break;
	case SYS_DVBC_ANNEX_A:
		if (demod_mode != AVL_DVBC)
			ret = avl6862_set_dvbmode(fe, c->delivery_system);
		ret = avl6862_RD_REG32(priv, 0x200 + rs_current_active_mode_iaddr_offset, &demod_mode);
		if (ret || demod_mode != AVL_DVBC) {
			dev_err(&priv->i2c->dev, "%s: failed to enter DVBC mode",
				KBUILD_MODNAME);
			ret = -EAGAIN;
			break;
		}
		ret = avl6862_set_dvbc(fe);
		break;
	case SYS_DVBC_ANNEX_B:
		if (demod_mode != AVL_DVBC)
			ret = avl6862_set_dvbmode(fe, c->delivery_system);
		ret = avl6862_RD_REG32(priv, 0x200 + rs_current_active_mode_iaddr_offset, &demod_mode);
		if (ret || demod_mode != AVL_DVBC) {
			dev_err(&priv->i2c->dev, "%s: failed to enter DVBC annex B mode",
				KBUILD_MODNAME);
			ret = -EAGAIN;
			break;
		}
		ret = avl6862_set_dvbc_b(fe);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
		if (demod_mode != AVL_DVBSX)
			ret = avl6862_set_dvbmode(fe, c->delivery_system);
		ret = avl6862_RD_REG32(priv, 0x200 + rs_current_active_mode_iaddr_offset, &demod_mode);
		if (ret || demod_mode != AVL_DVBSX) {
			dev_err(&priv->i2c->dev, "%s: failed to enter DVBSx mode",
				KBUILD_MODNAME);
			ret = -EAGAIN;
			break;
		}
		ret = avl6862_set_dvbs(fe);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int avl6862_tune(struct dvb_frontend *fe, bool re_tune,
	unsigned int mode_flags, unsigned int *delay, enum fe_status *status)
{
	*delay = HZ / 5;
	if (re_tune) {
		int ret = avl6862_set_frontend(fe);
		if (ret)
			return ret;
	}
	return avl6862_read_status(fe, status);
}
#if 0
static int avl6862_set_property(struct dvb_frontend *fe,
		struct dtv_property *p)
{
	int ret = 0;
	switch (p->cmd) {
	case DTV_DELIVERY_SYSTEM:
		ret = avl6862_set_dvbmode(fe, p->u.data);
		switch (p->u.data) {
		case SYS_DVBC_ANNEX_A:
		case SYS_DVBC_ANNEX_B:
			fe->ops.info.frequency_min = 47000000;
			fe->ops.info.frequency_max = 862000000;
			fe->ops.info.frequency_stepsize = 62500;
			break;
		case SYS_DVBS:
		case SYS_DVBS2:
			fe->ops.info.frequency_min = 950000;
			fe->ops.info.frequency_max = 2150000;
			fe->ops.info.frequency_stepsize = 0;
			break;
		case SYS_DVBT:
		case SYS_DVBT2:
		default:
			fe->ops.info.frequency_min = 174000000;
			fe->ops.info.frequency_max = 862000000;
			fe->ops.info.frequency_stepsize = 250000;
			break;
		}

		break;
	default:
		break;
	}

	return ret;
}
#endif
static int avl6862_init(struct dvb_frontend *fe)
{
	return 0;
}

static int avl6862_sleep(struct dvb_frontend *fe)
{
	return 0;
}

static void avl6862_release(struct dvb_frontend *fe)
{
	struct avl6862_priv *priv = fe->demodulator_priv;
	kfree(priv);
	return;
}

static struct dvb_frontend_ops avl6862_ops = {
	.delsys = {SYS_DVBT, SYS_DVBT2, SYS_DVBC_ANNEX_A, SYS_DVBC_ANNEX_B, SYS_DVBS, SYS_DVBS2},
	.info = {
		.name			= "Availink avl6862",
		.frequency_min_hz	= 175 * MHz,
		.frequency_max_hz	= 2150 * MHz,
		.frequency_stepsize_hz	= 0,
		.frequency_tolerance_hz	= 0,
		.symbol_rate_min	= 1000000,
		.symbol_rate_max	= 45000000,
		.caps = FE_CAN_FEC_1_2                 |
			FE_CAN_FEC_2_3                 |
			FE_CAN_FEC_3_4                 |
			FE_CAN_FEC_4_5                 |
			FE_CAN_FEC_5_6                 |
			FE_CAN_FEC_6_7                 |
			FE_CAN_FEC_7_8                 |
			FE_CAN_FEC_AUTO                |
			FE_CAN_QPSK                    |
			FE_CAN_QAM_16                  |
			FE_CAN_QAM_32                  |
			FE_CAN_QAM_64                  |
			FE_CAN_QAM_128                 |
			FE_CAN_QAM_256                 |
			FE_CAN_QAM_AUTO                |
			FE_CAN_TRANSMISSION_MODE_AUTO  |
			FE_CAN_GUARD_INTERVAL_AUTO     |
			FE_CAN_HIERARCHY_AUTO          |
			FE_CAN_MUTE_TS                 |
			FE_CAN_2G_MODULATION           |
			FE_CAN_MULTISTREAM             |
			FE_CAN_INVERSION_AUTO
	},

	.release			= avl6862_release,
	.init				= avl6862_init,

	.sleep				= avl6862_sleep,
	.i2c_gate_ctrl			= avl6862_i2c_gate_ctrl,

	.read_status			= avl6862_read_status,
	.read_signal_strength		= avl6862_read_signal_strength,
	.read_snr			= avl6862_read_snr,
	.read_ber			= avl6862_read_ber,
	.set_tone			= avl6862_set_tone,
	.set_voltage			= avl6862_set_voltage,
	.diseqc_send_master_cmd 	= avl6862_diseqc,
	.diseqc_send_burst 		= avl6862_burst,
	.get_frontend_algo		= avl6862fe_algo,
	.tune				= avl6862_tune,

// 	.set_property			= NULL, // avl6862_set_property,
	.set_frontend			= avl6862_set_frontend,
};

static struct dvb_frontend_ops avl6762_ops = {
	.delsys = {SYS_DVBT, SYS_DVBT2, SYS_DVBC_ANNEX_A, SYS_DVBC_ANNEX_B},
	.info = {
		.name			= "Availink AVL6762",
		.frequency_min_hz	= 175 * MHz,
		.frequency_max_hz	= 858 * MHz,
		.frequency_stepsize_hz	= 0,
		.frequency_tolerance_hz	= 0,
		.symbol_rate_min	= 1000000,
		.symbol_rate_max	= 45000000,
		.caps = FE_CAN_FEC_1_2                 |
			FE_CAN_FEC_2_3                 |
			FE_CAN_FEC_3_4                 |
			FE_CAN_FEC_4_5                 |
			FE_CAN_FEC_5_6                 |
			FE_CAN_FEC_6_7                 |
			FE_CAN_FEC_7_8                 |
			FE_CAN_FEC_AUTO                |
			FE_CAN_QPSK                    |
			FE_CAN_QAM_16                  |
			FE_CAN_QAM_32                  |
			FE_CAN_QAM_64                  |
			FE_CAN_QAM_128                 |
			FE_CAN_QAM_256                 |
			FE_CAN_QAM_AUTO                |
			FE_CAN_TRANSMISSION_MODE_AUTO  |
			FE_CAN_GUARD_INTERVAL_AUTO     |
			FE_CAN_HIERARCHY_AUTO          |
			FE_CAN_MUTE_TS                 |
			FE_CAN_2G_MODULATION           |
			FE_CAN_MULTISTREAM             |
			FE_CAN_INVERSION_AUTO
	},

	.release			= avl6862_release,
	.init				= avl6862_init,

	.sleep				= avl6862_sleep,
	.i2c_gate_ctrl			= avl6862_i2c_gate_ctrl,

	.read_status			= avl6862_read_status,
	.read_signal_strength		= avl6862_read_signal_strength,
	.read_snr			= avl6862_read_snr,
	.read_ber			= avl6862_read_ber,
	.get_frontend_algo		= avl6862fe_algo,
	.tune				= avl6862_tune,

// 	.set_property			= NULL, // avl6862_set_property,
	.set_frontend			= avl6862_set_frontend,
};


struct dvb_frontend *avl6862_attach(struct avl6862_config *config,
					struct i2c_adapter *i2c)
{
	struct avl6862_priv *priv;
	int ret;
	u32 id, fid;

	priv = kzalloc(sizeof(struct avl6862_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;

        if (config->tuner_address)
		memcpy(&priv->frontend.ops, &avl6862_ops, sizeof(struct dvb_frontend_ops));
        else
		memcpy(&priv->frontend.ops, &avl6762_ops, sizeof(struct dvb_frontend_ops));

	priv->frontend.demodulator_priv = priv;
	priv->config = config;
	priv->i2c = i2c;
	priv->g_nChannel_ts_total = 0,
	priv->delivery_system = -1;

	/* get chip id */
	ret = avl6862_RD_REG32(priv, 0x108000, &id);
	/* get chip family id */
	ret |= avl6862_RD_REG32(priv, 0x40000, &fid);
	if (ret) {
		dev_err(&priv->i2c->dev, "%s: attach failed reading id",
				KBUILD_MODNAME);
		goto err1;
	}

	if (fid != 0x68624955) {
		dev_err(&priv->i2c->dev, "%s: attach failed family id mismatch",
				KBUILD_MODNAME);
		goto err1;
	}
	switch (id) {
		case 0xb: id = 6882;
		case 0xd: id = 6812;
		case 0xe: id = 6762;
		case 0xf: id = 6862;
	}        
	dev_info(&priv->i2c->dev, "%s: found AVL%d " \
				"family_id=0x%x", KBUILD_MODNAME, id, fid);

        if (config->tuner_address) {
		if (!avl6862_set_dvbmode(&priv->frontend, SYS_DVBS))
		    return &priv->frontend;
	} 
	else {
		if (!avl6862_set_dvbmode(&priv->frontend, SYS_DVBT))
		    return &priv->frontend;
	} 
	
err1:
	kfree(priv);
err:
	return NULL;
}
EXPORT_SYMBOL_GPL(avl6862_attach);

MODULE_DESCRIPTION("Availink avl68xx DVB demodulator driver");
MODULE_AUTHOR("Luis Alves (ljalvs@gmail.com)");
MODULE_LICENSE("GPL");


