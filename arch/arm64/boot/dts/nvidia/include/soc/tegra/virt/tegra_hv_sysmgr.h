/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _TEGRA_HV_SYSMGR_H
#define _TEGRA_HV_SYSMGR_H

#include <linux/types.h>

#define SYSMGR_IVCMSG_SIZE_MAX 64

enum hv_sysmgr_msg_type {
	HV_SYSMGR_MSG_TYPE_GUEST_EVENT		= 1,
	HV_SYSMGR_MSG_TYPE_VM_PM_CTL_CMD	= 2,
	HV_SYSMGR_MSG_TYPE_INVALID
};

enum hv_sysmgr_cmd_id {
	HV_SYSMGR_CMD_NORMAL_SHUTDOWN	= 0x0,
	HV_SYSMGR_CMD_NORMAL_REBOOT	= 0x1,
	HV_SYSMGR_CMD_NORMAL_SUSPEND	= 0x2,
	HV_SYSMGR_CMD_NORMAL_RESUME	= 0x3,
	HV_SYSMGR_CMD_INVALID		= 0xFFFFFFFF,
};

enum hv_sysmgr_resp_id {
	HV_SYSMGR_RESP_ACCEPTED		= 0x0,
	HV_SYSMGR_RESP_UNKNOWN_COMMAND	= 0xF,
};

/* This struct comes as payload of hv_pm_ctl_message */
struct hv_sysmgr_command {
	uint32_t cmd_id;
	uint32_t resp_id;
} __packed;

struct hv_sysmgr_message {
	/* msg class */
	uint32_t msg_type;
	/* id of open socket */
	uint32_t socket_id;
	/* client data area. Payload */
	uint8_t client_data[SYSMGR_IVCMSG_SIZE_MAX];
} __packed;


/*
 * QUERY_SYSTEM_STATE COMMAND DATA LAYOUT
 */
struct hyp_sys_state_info {
	/* Indicates System State Transition */
	uint32_t sys_transition_mask;

	/* Indicates which VM shutdown request is pending */
	uint32_t vm_shutdown_mask;

	/* Indicates which VM reboot request is pending */
	uint32_t vm_reboot_mask;

	/* Indicates which VM suspend request is pending */
	uint32_t vm_suspend_phase_1_mask;
	uint32_t vm_suspend_phase_2_mask;

	/* Indicates which VM resume request is pending */
	uint32_t vm_resume_mask;
};

/*
 * Power management calls ID's used by SYSMGR to manage LOCAL/GLOBAL EVENTS
 */
enum system_function_id {
	INVALID_FUNC,
	/*
	 * This is used to get reboot/shutdown masks per VM from hypervisor.
	 * Hypervisor updates state fields on a PSCI event from the VM.
	 */
	QUERY_SYSTEM_STATE,
	GUEST_SHUTDOWN_INIT,
	GUEST_SHUTDOWN_COMPLETE,
	GUEST_REBOOT_INIT,
	GUEST_REBOOT_CONTINUE,
	GUEST_REBOOT_COMPLETE,
	SYSTEM_SHUTDOWN_INIT,
	SYSTEM_SHUTDOWN_COMPLETE,
	SYSTEM_REBOOT_INIT,
	SYSTEM_REBOOT_COMPLETE,
	GUEST_SUSPEND_REQ,
	GUEST_SUSPEND_INIT,
	GUEST_SUSPEND_COMPLETE,
	GUEST_RESUME_INIT,
	GUEST_RESUME_COMPLETE,
	GUEST_PAUSE,
	SYSTEM_SUSPEND_INIT,
	SYSTEM_SUSPEND_COMPLETE,
	MAX_FUNC_ID,
};

typedef enum  {
	VM_STATE_BOOT,
	VM_STATE_HALT,
	VM_STATE_UNHALT,
	VM_STATE_REBOOT,
	VM_STATE_SHUTDOWN,
	VM_STATE_SUSPEND,
	VM_STATE_RESUME,
	VM_STATE_INVALID,
	VM_STATE_MAX
} vm_state;

#define CREATE_CMD(func_id, vmid)	((func_id << 24U) | vmid)

#define QUERY_CMD			CREATE_CMD(QUERY_SYSTEM_STATE, 0)
#define GUEST_SHUTDOWN_INIT_CMD(vmid)	CREATE_CMD(GUEST_SHUTDOWN_INIT, vmid)
#define GUEST_SHUTDOWN_COMPLETE_CMD(vmid) \
				CREATE_CMD(GUEST_SHUTDOWN_COMPLETE, vmid)
#define GUEST_REBOOT_INIT_CMD(vmid)	CREATE_CMD(GUEST_REBOOT_INIT, vmid)
#define GUEST_REBOOT_CONTINUE_CMD(vmid)	CREATE_CMD(GUEST_REBOOT_CONTINUE, vmid)
#define GUEST_REBOOT_COMPLETE_CMD(vmid)	CREATE_CMD(GUEST_REBOOT_COMPLETE, vmid)
#define SYS_SHUTDOWN_INIT_CMD		CREATE_CMD(SYSTEM_SHUTDOWN_INIT, 0)
#define SYS_SHUTDOWN_COMPLETE_CMD	CREATE_CMD(SYSTEM_SHUTDOWN_COMPLETE, 0)
#define SYS_REBOOT_INIT_CMD		CREATE_CMD(SYSTEM_REBOOT_INIT, 0)
#define SYS_REBOOT_COMPLETE_CMD		CREATE_CMD(SYSTEM_REBOOT_COMPLETE, 0)
#define GUEST_SUSPEND_REQ_CMD(vmid)       CREATE_CMD(GUEST_SUSPEND_REQ,vmid)
#define GUEST_SUSPEND_INIT_CMD(vmid)      CREATE_CMD(GUEST_SUSPEND_INIT,vmid)
#define GUEST_SUSPEND_COMPLETE_CMD(vmid) \
		CREATE_CMD(GUEST_SUSPEND_COMPLETE,vmid)
#define GUEST_RESUME_INIT_CMD(vmid)       CREATE_CMD(GUEST_RESUME_INIT,vmid)
#define GUEST_RESUME_COMPLETE_CMD(vmid)   CREATE_CMD(GUEST_RESUME_COMPLETE,vmid)
#define GUEST_PAUSE_CMD(vmid)		CREATE_CMD(GUEST_PAUSE, vmid)
#define SYS_SUSPEND_INIT_CMD		CREATE_CMD(SYSTEM_SUSPEND_INIT, 0)
#define SYS_SUSPEND_COMPLETE_CMD	CREATE_CMD(SYSTEM_SUSPEND_COMPLETE, 0)
#endif /* _TEGRA_HV_SYSMGR_H */
