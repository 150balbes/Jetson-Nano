/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
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

#ifndef LINUX_TEGRA_CAMRTC_I2C_COMMON_H
#define LINUX_TEGRA_CAMRTC_I2C_COMMON_H

#if defined(__KERNEL__)
#include <linux/types.h>
#include <linux/compiler.h>
#else
#include <stdint.h>
#endif

/* I2C_REQUEST_MULTI frame size estimation
 * Assume the sensor has 2 byte register address
 * Assume each setting requires 4 consecutive register write
 * Assume up to four different setting updates
 * Each transfer has 2 byte header (flag + length).
 *
 * Header: 4 bytes
 * Transaction 0: Group Hold. 2 (header) + 3 (data)
 * Transaction 1,2,3,4: Setting update. (2 (header) + 6 (data)) * 4
 * Transaction 5: Group Hold. 2 (header) + 3 (data)
 * Transaction 6: Group Hold. 2 (header) + 3 (data)
 * Total: 51 bytes
 *
 * With RPC header: 75 bytes
 * IVC frame should be multiple of 64, thus 128 is picked.
 */

/* Message IDs */
#define CAMRTC_RPC_REQ_I2C_ADD_SINGLE_DEV	0x01U
#define CAMRTC_RPC_REQ_I2C_DEL_SINGLE_DEV	0x02U
#define CAMRTC_RPC_REQ_I2C_REQUEST_SINGLE	0x03U

#define CAMRTC_RPC_REQ_I2C_ADD_MULTI_DEV	0x11U
#define CAMRTC_RPC_REQ_I2C_DEL_MULTI_DEV	0x12U
#define CAMRTC_RPC_REQ_I2C_ADD_SENSOR		0x13U
#define CAMRTC_RPC_REQ_I2C_DEL_SENSOR		0x14U
#define CAMRTC_RPC_REQ_I2C_REQUEST_MULTI	0x15U

#define CAMRTC_RPC_REQ_I2C_REINIT_DEV		0x21U

#define CAMRTC_RPC_RSP_I2C_RESPONSE		0x31U

/* Message limit */
#define CAMRTC_I2C_REQUEST_MAX_LEN \
	(128U - TEGRA_IVC_RPC_MSG_HEADER_MAX)

/* ADD_SINGLE_DEV */
struct camrtc_rpc_i2c_add_single {
	uint32_t reg_base; /* I2C controller base address */
	uint32_t bus_clk_rate; /* I2C clock rate */
};

/* DEL_SINGLE_DEV */
struct camrtc_rpc_i2c_del_single {
	uint32_t bus_id; /* I2C bus */
};

/* REQUEST_SINGLE
 * Header
 *   +0: Bus ID
 * Followed by repetition of following
 *   +0: Flag
 *   +1: I2C address LSB
 *   +2: I2C address MSB
 *   +3: length of data
 *   +4: data for write
 */

#define CAMRTC_I2C_SINGLE_HEADER_SIZE		1U
#define CAMRTC_I2C_SINGLE_FLAG_OFFSET		0U
#define CAMRTC_I2C_SINGLE_ADDR_OFFSET		1U
#define CAMRTC_I2C_SINGLE_LENGTH_OFFSET		3U
#define CAMRTC_I2C_SINGLE_DATA_OFFSET		4U

#define CAMRTC_I2C_REQUEST_FLAG_READ		0x01U /* read transfer */
#define CAMRTC_I2C_REQUEST_FLAG_TEN		0x40U /* 10 bit address */
#define CAMRTC_I2C_REQUEST_FLAG_NOSTART		0x80U /* continue transfer */

/* ADD_MULTI_DEV */
struct camrtc_rpc_i2c_add_multi {
	uint32_t reg_base; /* I2C controller base address */
};

/* DEL_MULTI_DEV */
struct camrtc_rpc_i2c_del_multi {
	uint32_t bus_id; /* I2C bus */
};

#define CAMRTC_I2C_MP_NONE			0x00U /* no multiplexer */
#define CAMRTC_I2C_MP_TCA9548			0x01U /* TCA9548 */

#define CAMRTC_I2C_SENSOR_FLAG_TEN		0x0001U /* 10 bit address */
#define CAMRTC_I2C_SENSOR_FLAG_FM_PLUS		0x0002U /* fast mode plus */
#define CAMRTC_I2C_SENSOR_FLAG_HS		0x0004U /* high speed */

/* I2C_ADD_SENSOR */
struct camrtc_rpc_i2c_add_sensor {
	uint32_t bus_id; /* I2C bus */
	uint32_t addr; /* slave address of the sensor */
	uint32_t flag; /* CAMRTC_I2C_SENSOR_FLAG_ */
	uint32_t mp_type; /* multiplexer type */
	uint32_t mp_addr; /* slave address of multiplexer */
	uint32_t mp_channel; /* channel in multiplexer */
	uint32_t reserved;
};

/* I2C_DEL_SENSOR */
struct camrtc_rpc_i2c_del_sensor {
	uint16_t sensor_id; /* Sensor identifier */
};

/* I2C_REQUEST_MULTI
 * Header
 *   +0: Sensor ID
 *   +1: Flag
 *   +2: Frame ID LSB
 *   +3: Frame ID MSB
 * Followed by repetition of following
 *   +0: Flag
 *   +1: length of data
 *   +2: data for write
 */

#define CAMRTC_I2C_MULTI_HEADER_SIZE		4U
#define CAMRTC_I2C_MULTI_FLAG_OFFSET		0U
#define CAMRTC_I2C_MULTI_LENGTH_OFFSET		1U
#define CAMRTC_I2C_MULTI_DATA_OFFSET		2U

#define CAMRTC_I2C_REQUEST_MULTI_FLAG_FRAMEID	0x01U /* frame ID is valid */

/* REINIT_DEV */
struct camrtc_rpc_i2c_reinit {
	uint32_t bus_id;
};

/* RESPONSE
 */

#define CAMRTC_I2C_RESPONSE_RESULT_SUCCESS	0
#define CAMRTC_I2C_RESPONSE_RESULT_DROPPED	1
#define CAMRTC_I2C_RESPONSE_RESULT_NO_ACK	2

#define CAMRTC_I2C_RESPONSE_MAX_READ_LEN	64

struct camrtc_rpc_i2c_response {
	uint32_t result;
	uint32_t read_len;
	uint8_t read_data[CAMRTC_I2C_RESPONSE_MAX_READ_LEN];
};

/*
 * Structures of shared memory
 */

struct camrtc_i2c_single_cmd_queue {
	uint8_t flag;
};

#endif /* LINUX_TEGRA_CAMRTC_I2C_COMMON_H */
