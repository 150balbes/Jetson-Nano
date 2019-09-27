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

#ifndef __TLK_OTE_PROTOCOL_H__
#define __TLK_OTE_PROTOCOL_H__

#include "ote_types.h"

#include <uapi/misc/ote_protocol.h>

/*
 * shared buffer is 4 pages: 1st are requests, 2nd are params and the
 * remaining 2 are for params physical pages list
 */
#define TE_CMD_DESC_MAX	(PAGE_SIZE / sizeof(struct te_request))
#define TE_PARAM_MAX	(PAGE_SIZE / sizeof(struct te_oper_param))
#define TE_PLIST_MAX	((2 * PAGE_SIZE) / sizeof(uint64_t))

#define MAX_BUFFER_MAP_SIZE	(TE_PLIST_MAX * PAGE_SIZE)
#define MAX_EXT_SMC_ARGS	12

extern struct mutex smc_lock;
extern struct tlk_device tlk_dev;
extern void tlk_fiq_glue_aarch64(void);

uint32_t tlk_send_smc(uint32_t arg0, uintptr_t arg1, uintptr_t arg2);
uint32_t _tlk_generic_smc(uint32_t arg0, uintptr_t arg1, uintptr_t arg2);
void tlk_irq_handler(void);
struct te_oper_param *te_get_free_params(struct tlk_device *dev,
	unsigned int nparams);
void te_put_free_params(struct tlk_device *dev,
	struct te_oper_param *params, uint32_t nparams);
struct te_cmd_req_desc *te_get_free_cmd_desc(struct tlk_device *dev);
void te_put_used_cmd_desc(struct tlk_device *dev,
	struct te_cmd_req_desc *cmd_desc);

/* errors returned by secure world in reponse to SMC calls */
enum {
	TE_ERROR_PREEMPT_BY_IRQ = 0xFFFFFFFD,
	TE_ERROR_PREEMPT_BY_FS = 0xFFFFFFFE,
};

struct tlk_device {
	struct te_request *req_addr;
	dma_addr_t req_addr_phys;
	struct te_oper_param *param_addr;
	dma_addr_t param_addr_phys;
	uint64_t *plist_addr;
	dma_addr_t plist_addr_phys;

	unsigned long *param_bitmap;
	unsigned long *plist_bitmap;

	struct list_head used_cmd_list;
	struct list_head free_cmd_list;
};

struct te_cmd_req_desc {
	struct te_request *req_addr;
	struct list_head list;
};

struct te_shmem_desc {
	struct list_head list;
	uint32_t type;
	uint32_t plist_idx;
	size_t size;
	struct page **pages;
	unsigned int nr_pages;
};

/*
 * Per-session data structure.
 *
 * Both temp (freed upon completion of the associated op) and persist
 * (freed upon session close) memory references are tracked by this
 * structure.
 *
 * Persistent memory references stay on an inactive list until the
 * associated op completes.  If it completes successfully then the
 * references are moved to the active list.
 */
struct te_session {
	struct list_head list;
	uint32_t session_id;
	struct list_head temp_shmem_list;
	struct list_head inactive_persist_shmem_list;
	struct list_head persist_shmem_list;
};

struct tlk_context {
	struct tlk_device *dev;
	struct list_head session_list;
};

enum {
	/* Trusted Application Calls */
	TE_SMC_OPEN_SESSION		= 0x70000001,
	TE_SMC_CLOSE_SESSION		= 0x70000002,
	TE_SMC_LAUNCH_OPERATION		= 0x70000003,
	TE_SMC_TA_EVENT			= 0x70000004,

	/* Trusted OS (64-bit) calls */
	TE_SMC_REGISTER_REQ_BUF		= 0x72000001,
	TE_SMC_INIT_LOGGER		= 0x72000002,
	TE_SMC_RESTART			= 0x72000100,

	/* SIP (SOC specific) calls.  */
	TE_SMC_PROGRAM_VPR		= 0x82000003,
	TE_SMC_REGISTER_FIQ_GLUE	= 0x82000005,
	TE_SMC_VRR_SET_BUF		= 0x82000011,
	TE_SMC_VRR_SEC			= 0x82000012,
};

enum {
	TE_PARAM_TYPE_NONE		= 0x0,
	TE_PARAM_TYPE_INT_RO		= 0x1,
	TE_PARAM_TYPE_INT_RW		= 0x2,
	TE_PARAM_TYPE_MEM_RO		= 0x3,
	TE_PARAM_TYPE_MEM_RW		= 0x4,
	TE_PARAM_TYPE_PERSIST_MEM_RO	= 0x100,
	TE_PARAM_TYPE_PERSIST_MEM_RW	= 0x101,
	TE_PARAM_TYPE_FLAGS_PHYS_LIST	= 0x1000,
	TE_PARAM_TYPE_ALL_FLAGS		= TE_PARAM_TYPE_FLAGS_PHYS_LIST
};

enum {
	TE_MEM_TYPE_NS_USER	= 0x0,
	TE_MEM_TYPE_NS_KERNEL	= 0x1,
};

struct te_oper_param {
	uint32_t index;
	uint32_t type;
	union {
		struct {
			uint32_t val;
		} Int;
		struct {
			uint64_t base;
			uint32_t len;
			uint32_t type;
		} Mem;
	} u;
	uint64_t next_ptr_user;
};

struct te_request {
	uint32_t		type;
	uint32_t		session_id;
	uint32_t		command_id;
	uint64_t		params;
	uint32_t		params_size;
	uint32_t		dest_uuid[4];
	uint32_t		result;
	uint32_t		result_origin;
};

struct te_answer {
	uint32_t	result;
	uint32_t	session_id;
	uint32_t	result_origin;
};

void te_open_session(struct te_opensession *cmd,
	struct te_request *request,
	struct tlk_context *context);

void te_close_session(struct te_closesession *cmd,
	struct te_request *request,
	struct tlk_context *context);

void te_launch_operation(struct te_launchop *cmd,
	struct te_request *request,
	struct tlk_context *context);

enum ta_event_id {
	TA_EVENT_RESTORE_KEYS = 0,

	TA_EVENT_MASK = (1 << TA_EVENT_RESTORE_KEYS),
};

int te_handle_ss_ioctl(struct file *file, unsigned int ioctl_num,
			unsigned long ioctl_param);
void ote_print_logs(void);
int tlk_ss_op(void);
int ote_property_is_disabled(const char *str);
int te_prep_mem_buffers(struct te_request *request,
			struct te_session *session);
void te_release_mem_buffers(struct list_head *buflist);
void te_activate_persist_mem_buffers(struct te_session *session);

#endif