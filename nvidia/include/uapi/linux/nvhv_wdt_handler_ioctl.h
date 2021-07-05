/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __UAPI_LINUX_NVHV_WDT_HANDLER_IOCTL_H
#define __UAPI_LINUX_NVHV_WDT_HANDLER_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

/* This matches what is defined in server, if server side
 * changes, this has to be changed
 */
#define MAX_GUESTS_NUM		16
#define HV_GUEST_IOCTL_MAGIC	'X'


/* Get the state of all the guest VMs */
#define TEGRA_HV_WDT_H_GET_STATE	_IOR(HV_GUEST_IOCTL_MAGIC, 0x00, \
					struct hv_wdt_h_state_array)

/* Send command to Monitor about Guest VM action taken */
#define TEGRA_HV_WDT_H_CMD		_IOW(HV_GUEST_IOCTL_MAGIC, 0x01, \
					struct tegra_hv_wdt_h_cmd_array)


enum tegra_hv_wdt_h_cmd_id {
	/* Application has handled the guest vm expiry synchronously */
	MESSAGE_HANDLED_SYNC,
	/* Application has handled  the guest vm expiry asynchronously */
	MESSAGE_HANDLED_ASYNC,
};

enum tegra_hv_wdt_h_state_id {
	/* This is default state or monitor, This state is also reached once
	 *  app updates driver about ASYNC or SYNC recovery triggered for Guest
	 */
	GUEST_STATE_INIT,
	/* This is state, once monitor updates driver about GUEST WDT expiry */
	GUEST_STATE_TIMER_EXPIRED,
	/* Waiting for action from Application */
	GUEST_STATE_WAIT_FOR_ACK,
};

struct hv_wdt_h_state_array {
	/* State of each guest, use enum tegra_hv_wdt_h_state_id */
	__u32 guest_state[MAX_GUESTS_NUM];
};

struct tegra_hv_wdt_h_cmd {
	/* VMID for which command is expected */
	__u32 vmid;
	/* Command for the VMID, use tegra_hv_wdt_h_cmd_id */
	__u32 command;
};

struct tegra_hv_wdt_h_cmd_array {
	/* Number of vmids for which commands is to be issued */
	__u32 num_vmids;
	/* VMID and command to be issued, only first num_vmids will be used from
	 * below array
	 */
	struct tegra_hv_wdt_h_cmd commands[MAX_GUESTS_NUM];
};
#endif
