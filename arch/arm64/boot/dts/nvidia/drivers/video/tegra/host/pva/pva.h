/*
 * drivers/video/tegra/host/pva/pva.h
 *
 * Tegra PVA header
 *
 * Copyright (c) 2016-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_PVA_H__
#define __NVHOST_PVA_H__

#include <linux/dma-attrs.h>
#include <linux/mutex.h>
#include <linux/version.h>

#include "nvhost_queue.h"
#include "pva_regs.h"

extern const struct file_operations tegra_pva_ctrl_ops;

enum pva_submit_mode {
	PVA_SUBMIT_MODE_MAILBOX		= 0,
	PVA_SUBMIT_MODE_MMIO_CCQ	= 1,
	PVA_SUBMIT_MODE_CHANNEL_CCQ	= 2
};

struct pva_version_info {
	u32 pva_r5_version;
	u32 pva_compat_version;
	u32 pva_revision;
	u32 pva_built_on;
};

/**
 * Queue count of 8 is maintained per PVA.
 */
#define MAX_PVA_QUEUE_COUNT 8

/**
 * Maximum task count that a queue can support
 */
#define MAX_PVA_TASK_COUNT	16

/**
 * Minium PVA frequency (10MHz)
 */
#define MIN_PVA_FREQUENCY	10000000

/**
 * @brief		struct to hold the segment details
 *
 * addr:		virtual addr of the segment from PRIV2 address base
 * size:		segment size
 * offset:		offset of the addr from priv2 base
 *
 */
struct pva_seg_info {
	void *addr;
	u32 size;
	u32 offset;
};

/**
 * @breif		struct to hold the segment details for debug purpose
 *
 * pva			Pointer to pva struct
 * seg_info		pva_seg_info struct
 *
 */
struct pva_crashdump_debugfs_entry {
	struct pva *pva;
	struct pva_seg_info seg_info;
};

/**
 * @brief		struct to handle dma alloc memory info
 *
 * size			size allocated
 * phys_addr		physical address
 * va			virtual address
 *
 */
struct pva_dma_alloc_info {
	size_t size;
	dma_addr_t pa;
	void *va;
};

/**
 * @brief		struct to handle the PVA firmware information
 *
 * hdr			pointer to the pva_code_hdr struct
 * priv1_buffer		pva_dma_alloc_info for priv1_buffer
 * priv2_buffer		pva_dma_alloc_info for priv2_buffer
 * priv2_reg_offset	priv2 register offset from uCode
 * attrs		dma_attrs struct information
 * trace_buffer_size	buffer size for trace log
 *
 */
struct pva_fw {
	struct pva_ucode_hdr *hdr;

	struct pva_dma_alloc_info priv1_buffer;
	struct pva_dma_alloc_info priv2_buffer;
	u32 priv2_reg_offset;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	struct dma_attrs attrs;
#else
	unsigned long attrs;
#endif

	u32 trace_buffer_size;
};

/*
 * @brief		store trace log segment's address and size
 *
 * addr		Pointer to the pva trace log segment
 * size		Size of pva trace log segment
 * offset		Offset in bytes for trace log segment
 *
 */
struct pva_trace_log {
	void *addr;
	u32 size;
	u32 offset;
};


/*
 * @brief	stores address and other attributes of the vpu function table
 *
 * addr		The pointer to start of the VPU function table
 * size		Table size of the function table
 * handle	The IOVA address of the function table
 * entries	The total number of entries in the function table
 *
 */
struct pva_func_table {
	struct vpu_func *addr;
	uint32_t size;
	dma_addr_t handle;
	uint32_t entries;
};

/**
 * @brief		Driver private data, shared with all applications
 *
 * pdev			Pointer to the PVA device
 * pool			Pointer to Queue table available for the PVA
 * fw_info		firmware information struct
 * irq			IRQ number obtained on registering the module
 * mailbox_mutex	Mutex to avoid concurrent mailbox accesses
 * mailbox_waitq	Mailbox waitqueue for response waiters
 * mailbox_status_regs	Response is stored into this structure temporarily
 * mailbox_status	Status of the mailbox interface
 * debugfs_entry_r5	debugfs segment information for r5
 * debugfs_entry_vpu0	debugfs segment information for vpu0
 * debugfs_entry_vpu1	debugfs segment information for vpu1
 * priv1_dma		struct pva_dma_alloc_info for priv1_dma
 * priv2_dma		struct pva_dma_alloc_info for priv2_dma
 * pva_trace		struct for pva_trace_log
 * submit_mode		Select the task submit mode
 * dbg_vpu_app_id	Set the vpu_app id to debug
 * r5_dbg_wait		Set the r5 debugger to wait
 * timeout_enabled	Set pva timeout enabled based on debug
 * slcg_disable		Second level Clock Gating control variable
 *
 */
struct pva {
	struct platform_device *pdev;
	struct nvhost_queue_pool *pool;
	struct pva_fw fw_info;

	int irq;

	wait_queue_head_t mailbox_waitqueue;
	struct pva_mailbox_status_regs mailbox_status_regs;
	enum pva_mailbox_status mailbox_status;
	struct mutex mailbox_mutex;

	struct mutex ccq_mutex;

	struct pva_crashdump_debugfs_entry debugfs_entry_r5;
	struct pva_crashdump_debugfs_entry debugfs_entry_vpu0;
	struct pva_crashdump_debugfs_entry debugfs_entry_vpu1;

	struct pva_dma_alloc_info priv1_dma;
	struct pva_dma_alloc_info priv2_dma;

	struct pva_trace_log pva_trace;
	u32 submit_mode;

	u32 dbg_vpu_app_id;
	u32 r5_dbg_wait;
	bool timeout_enabled;
	u32 slcg_disable;
	u32 vmem_war_disable;
	bool vpu_perf_counters_enable;

	struct work_struct pva_abort_handler_work;
	bool booted;

	u32 log_level;
};

/**
 * @brief	Copy traces to kernel trace buffer.
 *
 * When mailbox interrupt for copying ucode trace buffer to
 * kernel-ucode shared trace buffer is arrived it copies the kernel-ucode
 * shared trace buffer to kernel ftrace buffer
 *
 * @pva Pointer to pva structure
 *
 */
void pva_trace_copy_to_ftrace(struct pva *pva);

/**
 * @brief	Finalize the PVA Power-on-Sequence.
 *
 * This function called from host subsystem driver after the PVA
 * partition has been brought up, clocks enabled and reset deasserted.
 * In production mode, the function needs to wait until the ready  bit
 * within the PVA aperture has been set. After that enable the PVA IRQ.
 * Register the queue priorities on the PVA.
 *
 * @param pdev	Pointer to PVA device
 * @return:	0 on Success or negative error code
 *
 */
int pva_finalize_poweron(struct platform_device *pdev);

/**
 * @brief	Prepare PVA poweroff.
 *
 * This function called from host subsystem driver before turning off
 * the PVA. The function should turn off the PVA IRQ.
 *
 * @param pdev	Pointer to PVA device
 * @return	0 on Success or negative error code
 *
 */
int pva_prepare_poweroff(struct platform_device *pdev);

/**
 * @brief	Register PVA ISR.
 *
 * This function called from driver to register the
 * PVA ISR with IRQ.
 *
 * @param pdev	Pointer to PVA device
 * @return	0 on Success or negative error code
 *
 */
int pva_register_isr(struct platform_device *dev);

/**
 * @brief	Initiallze pva debug utils
 *
 * @param pdev	Pointer to PVA device
 * @return	none
 *
 */
void pva_debugfs_init(struct platform_device *pdev);

/**
 * @brief	Initiallze PVA abort handler
 *
 * @param pva	Pointer to PVA structure
 * @return	none
 *
 */
void pva_abort_init(struct pva *pva);

/**
 * @brief	Recover PVA back into working state
 *
 * @param pva	Pointer to PVA structure
 * @return	none
 *
 */
void pva_abort(struct pva *pva);

/**
 * @brief	Run the ucode selftests
 *
 * This function is invoked if the ucode is in selftest mode.
 * The function will do the static memory allocation for the
 * ucode self test to run.
 *
 * @param pdev	Pointer to PVA device
 * @return	0 on Success or negative error code
 *
 */
int pva_run_ucode_selftest(struct platform_device *pdev);

/**
 * @brief	Allocate and populate the function table to the memory
 *
 * This function is called when the vpu table needs to be populated.
 * The function also allocates the memory required for the vpu table.
 *
 * @param pva			Pointer to PVA device
 * @param pva_func_table	Pointer to the function table which contains
 *				the address, table size and number of entries
 * @return			0 on Success or negative error code
 *
 */
int pva_alloc_and_populate_function_table(struct pva *pva,
					  struct pva_func_table *fn_table);

/**
 * @brief	Deallocate the memory of the function table
 *
 * This function is called once the allocated memory for vpu table needs to
 * be freed.
 *
 * @param pva			Pointer to PVA device
 * @param pva_func_table	Pointer to the function table which contains
 *				the address, table size and number of entries
 *
 */
void pva_dealloc_vpu_function_table(struct pva *pva,
				    struct pva_func_table *fn_table);

/**
 * @brief	Get PVA version information
 *
 * @param pva	Pointer to a PVA device node
 * @param info	Pointer to an information structure to be filled
 *
 * @return	0 on success, otherwise a negative error code
 */
int pva_get_firmware_version(struct pva *pva,
			     struct pva_version_info *info);

/**
 * @brief	Set trace log level of PVA
 *
 * @param pva	Pointer to a PVA device node
 * @param log_level	32-bit mask for logs that we want to receive
 *
 * @return	0 on success, otherwise a negative error code
 */
int pva_set_log_level(struct pva *pva,
			     u32 log_level);
#endif
