/*
 * Copyright (c) 2013-2019 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __UAPI_TLK_OTE_PROTOCOL_H__
#define __UAPI_TLK_OTE_PROTOCOL_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define TE_IOCTL_MAGIC_NUMBER ('t')
#define TE_IOCTL_OPEN_CLIENT_SESSION \
	_IOWR(TE_IOCTL_MAGIC_NUMBER, 0x10, union te_cmd)
#define TE_IOCTL_CLOSE_CLIENT_SESSION \
	_IOWR(TE_IOCTL_MAGIC_NUMBER, 0x11, union te_cmd)
#define TE_IOCTL_LAUNCH_OPERATION \
	_IOWR(TE_IOCTL_MAGIC_NUMBER, 0x14, union te_cmd)

/* secure storage ioctl */
#define TE_IOCTL_SS_CMD \
	_IOR(TE_IOCTL_MAGIC_NUMBER,  0x30, int)

#define TE_IOCTL_SS_CMD_GET_NEW_REQ	1
#define TE_IOCTL_SS_CMD_REQ_COMPLETE	2

#define TE_IOCTL_MIN_NR _IOC_NR(TE_IOCTL_OPEN_CLIENT_SESSION)
#define TE_IOCTL_MAX_NR _IOC_NR(TE_IOCTL_SS_CMD)

struct te_service_id {
	uint32_t time_low;
	uint16_t time_mid;
	uint16_t time_hi_and_version;
	uint8_t clock_seq_and_node[8];
};

struct te_operation {
	uint32_t	command;
	uint32_t	status;
	uint64_t	list_head;
	uint64_t	list_tail;
	uint32_t	list_count;
	uint32_t	interface_side;
};

/*
 * OpenSession
 */
struct te_opensession {
	struct te_service_id	dest_uuid;
	struct te_operation	operation;
	uint64_t		answer;
};

/*
 * CloseSession
 */
struct te_closesession {
	uint32_t	session_id;
	uint64_t	answer;
};

/*
 * LaunchOperation
 */
struct te_launchop {
	uint32_t		session_id;
	struct te_operation	operation;
	uint64_t		answer;
};

union te_cmd {
	struct te_opensession	opensession;
	struct te_closesession	closesession;
	struct te_launchop	launchop;
};

#endif