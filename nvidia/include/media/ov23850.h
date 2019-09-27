/**
 * Copyright (c) 2014-2019, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OV23850_H__
#define __OV23850_H__

#include <media/nvc.h>
#include <uapi/media/nvc_image.h>
#include <uapi/media/ov23850.h>

#define OV23850_EEPROM_ADDRESS		0x50
#define OV23850_EEPROM_SIZE		1024
#define OV23850_EEPROM_STR_SIZE		(OV23850_EEPROM_SIZE * 2)
#define OV23850_EEPROM_BLOCK_SIZE	(1 << 8)
#define OV23850_EEPROM_NUM_BLOCKS \
	 (OV23850_EEPROM_SIZE / OV23850_EEPROM_BLOCK_SIZE)

#define OV23850_OTP_ISP_CTRL_ADDR		0x5000
#define OV23850_OTP_LOAD_CTRL_ADDR		0x3D81
#define OV23850_OTP_MODE_CTRL_ADDR		0x3D84
#define OV23850_OTP_POWER_UP_ADDR		0x3D85
#define OV23850_OTP_START_REG_ADDR_MSB		0x3D88
#define OV23850_OTP_START_REG_ADDR_LSB		0x3D89
#define OV23850_OTP_END_REG_ADDR_MSB		0x3D8A
#define OV23850_OTP_END_REG_ADDR_LSB		0x3D8B
#define OV23850_OTP_START_ADDR	0x6000
#define OV23850_OTP_END_ADDR	0x601F
#define OV23850_OTP_SIZE \
	 (OV23850_OTP_END_ADDR - OV23850_OTP_START_ADDR + 1)

#define OV23850_OTP_STR_SIZE (OV23850_OTP_SIZE * 2)
#define OV23850_OTP_RD_BUSY_MASK		0x80
#define OV23850_OTP_BIST_ERROR_MASK	0x10
#define OV23850_OTP_BIST_DONE_MASK		0x08

#endif  /* __OV23850_H__ */