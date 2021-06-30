/*
 * ov10823.h - ov10823 sensor driver
 *
 * Copyright (c) 2016-2019 NVIDIA Corporation.  All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __OV10823_H__
#define __OV10823_H__

#include <uapi/media/ov10823.h>

#define OV10823_ISP_CTRL_ADDR			0x5002
#define OV10823_OTP_PROGRAME_CTRL_ADDR	0x3D80
#define OV10823_OTP_LOAD_CTRL_ADDR		0x3D81
#define OV10823_OTP_MODE_CTRL_ADDR		0x3D84
#define OV10823_OTP_PROGRAME_START_ADDR		0x3D00
#define OV10823_OTP_PROGRAME_END_ADDR_MSB	0x3D0F
#define OV10823_OTP_SIZE			0x500
#define OV10823_OTP_SRAM_START_ADDR		0x6000
#define OV10823_OTP_STR_SIZE (OV10823_OTP_SIZE * 2)

struct ov10823_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *vif;
};

struct ov10823_platform_data {
	struct ov10823_flash_control flash_cap;
	const char *dev_name;
	unsigned num;
	int reset_gpio;
	int cam1sid_gpio;
	int cam2sid_gpio;
	int cam3sid_gpio;
	unsigned default_i2c_sid_low;
	unsigned default_i2c_sid_high;
	unsigned regw_sid_low;
	unsigned regw_sid_high;
	unsigned cam1i2c_addr;
	unsigned cam2i2c_addr;
	unsigned cam3i2c_addr;
	bool cam_use_26mhz_mclk;
	bool cam_change_i2c_addr;
	bool cam_i2c_recovery;
	bool cam_sids_high_to_low;
	bool cam_skip_sw_reset;
	bool cam_use_osc_for_mclk;
	int (*power_on)(struct ov10823_power_rail *pw);
	int (*power_off)(struct ov10823_power_rail *pw);
	const char *mclk_name;
};

#endif /* __OV10823_H__ */