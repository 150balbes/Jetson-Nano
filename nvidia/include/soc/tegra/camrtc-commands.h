/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_COMMANDS_H
#define INCLUDE_CAMRTC_COMMANDS_H

#define RTCPU_COMMAND(id, value)	((RTCPU_CMD_ ## id << 24U) | (value))

#define RTCPU_GET_COMMAND_ID(value)	(((value) >> 24U) & 0x7fU)

#define RTCPU_GET_COMMAND_VALUE(value)	((value) & 0xffffffU)

enum {
	RTCPU_CMD_INIT = 0U,
	RTCPU_CMD_FW_VERSION = 1U,
	RTCPU_CMD_IVC_READY = 2U,
	RTCPU_CMD_PING = 3U,
	RTCPU_CMD_PM_SUSPEND = 4U,
	RTCPU_CMD_FW_HASH = 5U,
	RTCPU_CMD_CH_SETUP = 6U,
	RTCPU_CMD_PREFIX = 0x7dU,
	RTCPU_CMD_DOORBELL = 0x7eU,
	RTCPU_CMD_ERROR = 0x7fU,
};

#define RTCPU_FW_DB_VERSION 0U
#define RTCPU_FW_VERSION 1U
#define RTCPU_FW_SM2_VERSION 2U
#define RTCPU_FW_SM3_VERSION 3U
/* SM4 firmware can restore itself after suspend */
#define RTCPU_FW_SM4_VERSION 4U

/* SM5 firmware supports IVC synchronization  */
#define RTCPU_FW_SM5_VERSION 5U
/* SM5 driver supports IVC synchronization  */
#define RTCPU_DRIVER_SM5_VERSION 5U

#define RTCPU_FW_CURRENT_VERSION (RTCPU_FW_SM5_VERSION)

#define RTCPU_IVC_SANS_TRACE 1U
#define RTCPU_IVC_WITH_TRACE 2U

#define RTCPU_FW_HASH_SIZE 20U

#define RTCPU_PM_SUSPEND_SUCCESS (0x100U)
#define RTCPU_PM_SUSPEND_FAILURE (0x001U)

#endif /* INCLUDE_CAMRTC_COMMANDS_H */
