/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _TEGRA_HV_UFS_H_
#define _TEGRA_HV_UFS_H_

#include <linux/types.h>

#define QUERY_DESC_MAX_SIZE       255
#define QUERY_DESC_MIN_SIZE       2
#define QUERY_DESC_HDR_SIZE       2

/* Attribute idn for Query requests */
enum attr_idn {
	QUERY_ATTR_IDN_BOOTLUN_EN	= 0x0,
	QUERY_ATTR_IDN_PWR_MODE		= 0x02,
	QUERY_ATTR_IDN_ACTIVE_ICC_LVL	= 0x03,
	QUERY_ATTR_IDN_BKOPS_STATUS	= 0x05,
	QUERY_ATTR_IDN_REF_CLK_FREQ	= 0x0A,
	QUERY_ATTR_IDN_CONF_DESC_LCK	= 0x0B,
	QUERY_ATTR_IDN_EE_CONTROL	= 0x0D,
	QUERY_ATTR_IDN_EE_STATUS	= 0x0E,
	QUERY_ATTR_IDN_MAX		= 0x30,
};

/* Query response result code */
enum {
	QUERY_RESULT_SUCCESS                    = 0x00,
	QUERY_RESULT_NOT_READABLE               = 0xF6,
	QUERY_RESULT_NOT_WRITEABLE              = 0xF7,
	QUERY_RESULT_ALREADY_WRITTEN            = 0xF8,
	QUERY_RESULT_INVALID_LENGTH             = 0xF9,
	QUERY_RESULT_INVALID_VALUE              = 0xFA,
	QUERY_RESULT_INVALID_SELECTOR           = 0xFB,
	QUERY_RESULT_INVALID_INDEX              = 0xFC,
	QUERY_RESULT_INVALID_IDN                = 0xFD,
	QUERY_RESULT_INVALID_OPCODE             = 0xFE,
	QUERY_RESULT_GENERAL_FAILURE            = 0xFF,
};



/* UTP QUERY Transaction Specific Fields OpCode */
enum query_opcode {
	UPIU_QUERY_OPCODE_NOP           = 0x0,
	UPIU_QUERY_OPCODE_READ_DESC     = 0x1,
	UPIU_QUERY_OPCODE_WRITE_DESC    = 0x2,
	UPIU_QUERY_OPCODE_READ_ATTR     = 0x3,
	UPIU_QUERY_OPCODE_WRITE_ATTR    = 0x4,
	UPIU_QUERY_OPCODE_READ_FLAG     = 0x5,
	UPIU_QUERY_OPCODE_SET_FLAG      = 0x6,
	UPIU_QUERY_OPCODE_CLEAR_FLAG    = 0x7,
	UPIU_QUERY_OPCODE_TOGGLE_FLAG   = 0x8,
};

/* Descriptor idn for Query requests */
enum desc_idn {
	QUERY_DESC_IDN_DEVICE		= 0x0,
	QUERY_DESC_IDN_CONFIGURATION	= 0x1,
	QUERY_DESC_IDN_UNIT		= 0x2,
	QUERY_DESC_IDN_RFU_0		= 0x3,
	QUERY_DESC_IDN_INTERCONNECT	= 0x4,
	QUERY_DESC_IDN_STRING		= 0x5,
	QUERY_DESC_IDN_RFU_1		= 0x6,
	QUERY_DESC_IDN_GEOMETRY		= 0x7,
	QUERY_DESC_IDN_POWER		= 0x8,
	QUERY_DESC_IDN_DEVICE_HEALTH	= 0x9,
	QUERY_DESC_IDN_MAX,
};

/* Flag idn for Query Requests*/
enum flag_idn {
	QUERY_FLAG_IDN_FDEVICEINIT      = 0x01,
	QUERY_FLAG_IDN_PWR_ON_WPE	= 0x03,
	QUERY_FLAG_IDN_BKOPS_EN         = 0x04,
	QUERY_FLAG_IDN_MAX		= 0x0E,
};

#endif /* _TEGRA_HV_UFS_H_ */

