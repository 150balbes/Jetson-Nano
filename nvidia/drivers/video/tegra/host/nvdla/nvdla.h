/*
 * drivers/video/tegra/host/nvdla/nvdla.h
 *
 * Tegra Graphics Host NVDLA
 *
 * Copyright (c) 2016-2019 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_NVDLA_H__
#define __NVHOST_NVDLA_H__

#include <linux/completion.h>
#include <linux/mutex.h>
#include <uapi/linux/nvdev_fence.h>
#include <uapi/linux/nvhost_nvdla_ioctl.h>


#include "nvdla_buffer.h"
#include "dla_os_interface.h"
#include "dla_fw_version.h"

#define ALIGNED_DMA(x) ((x >> 8) & 0xffffffff)

/*
 * Max grid size
 */
#define MAX_GRID_SIZE			SZ_256

/**
 * DLA MCIF (Matrix Convolution Image Filter) outstanding AXI transactions
 * count register offset
 */
#define NVDLA_MCIF_CFG_OUTSTANDING_CNT_0_OFFSET	0x00005014	/* RW-4R */

/**
 * Method ID and Method data THI registers
 */
#define NV_DLA_THI_METHOD_ID	0x00000040      /* RW-4R */
#define NV_DLA_THI_METHOD_DATA	0x00000044      /* RW-4R */

#define NV_DLA_OS_VERSION	0x00001080      /* RW-4R */

#define MAX_NUM_ACTION_LIST	1

/**
 * Maximum number of queue's per engine
 */
#define MAX_NVDLA_QUEUE_COUNT	16

/**
 * Maximum number of tasks per queue
 */
#define MAX_NVDLA_TASK_COUNT	32

/**
 * Maximum number of buffers per pin request
 */
#define MAX_NVDLA_PIN_BUFFERS	32

/**
 * Maximum number of buffers per task
 */
#define MAX_NUM_NVDLA_BUFFERS_PER_TASK	6144

/**
 * Trace Buffer Size
 */
#define TRACE_BUFFER_SIZE		SZ_1M

/**
 * Maximum buffer size for debug dump
 */
#define DEBUG_BUFFER_SIZE		SZ_256

/**
 * Firmware GCOV Buffer Size
 */
#define GCOV_BUFFER_SIZE		SZ_64K

/*
 * CMD submission timeout in msec
 */
#define CMD_TIMEOUT_MSEC	(1000)

/**
 * Max number of fences supported
 */
#define MAX_NUM_NVDLA_PREFENCES		32
#define MAX_NUM_NVDLA_POSTFENCES	32
#define MAX_NUM_NVDLA_EMU_PREFENCES	16
#define MAX_NUM_NVDLA_EMU_POSTFENCES	16
#define MAX_NUM_NVDLA_IN_TASK_STATUS	MAX_NUM_NVDLA_PREFENCES
#define MAX_NUM_NVDLA_OUT_TASK_STATUS	MAX_NUM_NVDLA_POSTFENCES
#define MAX_NUM_NVDLA_OUT_TIMESTAMP  	32
#define NUM_PROFILING_POSTACTION	1

#define MAX_COMMANDS_PER_DEVICE		1

/*
 * keep max cmd size multiple of 256 bytes.
 */
#define MAX_CMD_SIZE			SZ_256
#define NVDLA_CMD_OFFSET(index)		(MAX_CMD_SIZE * index)

/*
 * Tegra/DLA Hardware version specific Quirks
 */
#define NVDLA_QUIRK_T194_A01_WAR	BIT(0)

/**
 * data structure to keep command memory
 *
 * @pa			IOVA pointing to cmd memory with offset
 * @va			VA of cmd memory
 * @index		index pointing command in pool
 *
 */

struct nvdla_cmd_mem_info {
	dma_addr_t pa;
	void *va;
	int index;
};

/**
 * data structure to keep command memory pool
 *
 * @pa			IOVA pointing to cmd memory
 * @va			VA of cmd memory
 * @lock		mutex lock
 * @alloc_table		memory usage bitmap table
 */
struct nvdla_cmd_mem {
	dma_addr_t pa;
	void *va;
	struct mutex lock;
	unsigned long alloc_table;
};

/**
 * data structure to keep command data
 *
 * @method_id		method id with command and other info
 * @method_data		method data for command
 * @wait		If set to true then wait for command completion
 */
struct nvdla_cmd_data {
	uint32_t method_id;
	uint32_t method_data;
	bool wait;
};

enum nvdla_submit_mode {
	NVDLA_SUBMIT_MODE_MMIO		= 0,
	NVDLA_SUBMIT_MODE_CHANNEL	= 1
};

/**
 * data structure to keep per DLA engine device data
 *
 * @pdev		pointer to platform device
 * @pool		pointer to queue table
 * @dbg_mask		debug mask for print level
 * @en_trace		flag to enable kernel tracing
 * @submit_mode		flag to enable task submit mode, default is
 *				NVDLA_SUBMIT_MODE_MMIO
 * @fw_version		saves current firmware version
 * @cmd_mem		structure to hold command memory pool
 * @trace_enable	to enable/disable the DLA firmware trace
 * @events_mask		mask to set/reset the different DLA firmware trace event
 * @is_gos_enabled	flag to check if GoS enabled
 * @debug_dump_pa	physical address of print buffer
 * @debug_dump_va	virtual address of print buffer
 * @trace_dump_pa	physical address of trace buffer
 * @trace_dump_va	virtual address of trace buffer
 * @en_fw_gcov		flag to enable firmware gcov
 * @gcov_dump_pa	physical address of fw gcov buffer
 * @gcov_dump_va	virtual address of fw gcovbuffer
 * @quirks		Tegra/DLA Hardware version specific settings
 */
struct nvdla_device {
	struct platform_device *pdev;
	struct nvdla_queue_pool *pool;
	struct completion cmd_completion;
	struct mutex cmd_lock;
	int cmd_status;
	int waiting;
	u32 dbg_mask;
	u32 en_trace;
	u32 submit_mode;
	u32 fw_version;
	struct nvdla_cmd_mem cmd_mem;
	u32 trace_enable;
	u32 events_mask;
	bool is_gos_enabled;
	bool is_gos_fetched;
	dma_addr_t debug_dump_pa;
	u32 *debug_dump_va;
	dma_addr_t trace_dump_pa;
	u32 *trace_dump_va;
	u32 en_fw_gcov;
	dma_addr_t gcov_dump_pa;
	u32 *gcov_dump_va;
	u32 quirks;
	struct work_struct reset_work;
};

/**
 * struct nvdla_emu_task:	structure for emulator task info
 *
 * @queue		Queue in which task submitted
 * @sp			pointer to syncpt
 * @prefences		pointer to pre fences
 * @postfences		pointer to post fences
 * @num_prefences	Number of prefences in task
 * @num_postfences	Number of postfences in task
 * @fence		Fence tracking for current task
 * @fence_counter	Counter used to track fence value
 *
 */
struct nvdla_emu_task {
	struct nvdla_queue *queue;
	struct nvhost_syncpt *sp;
	struct nvdev_fence prefences[MAX_NUM_NVDLA_EMU_PREFENCES];
	struct nvdev_fence postfences[MAX_NUM_NVDLA_EMU_POSTFENCES];
	u32 num_prefences;
	u32 num_postfences;
	u32 fence;
	u32 fence_counter;
};

/**
 * struct nvdla_task:	structure for task info
 *
 * @queue		Queue in which task submitted
 * @buffers		nvhost buffers for priv/task
 * @sp			pointer to syncpt
 * @prefences		pointer to prefences
 * @postfences		pointer to post fences
 * @fence		fence tracking for current task
 * @ref			Reference count for task
 * @list		List entry
 * @task_desc		DLA task desc VA
 * @task_desc_pa	DLA task desc PA
 * @buf_size		Total size of task dma alloc
 * @timeout		max timeout to wait for task completion
 * @op_handle		pointer to handle list of operation descriptor
 *
 */
struct nvdla_task {
	struct nvdla_queue *queue;
	struct nvdla_buffers *buffers;
	struct nvhost_syncpt *sp;
	struct nvdev_fence prefences[MAX_NUM_NVDLA_PREFENCES];
	struct nvdev_fence postfences[MAX_NUM_NVDLA_POSTFENCES];
	struct nvdla_status_notify in_task_status[MAX_NUM_NVDLA_IN_TASK_STATUS];
	struct nvdla_status_notify sof_task_status[MAX_NUM_NVDLA_OUT_TASK_STATUS];
	struct nvdla_status_notify eof_task_status[MAX_NUM_NVDLA_OUT_TASK_STATUS];
	struct nvdla_mem_handle sof_timestamps[MAX_NUM_NVDLA_OUT_TIMESTAMP];
	struct nvdla_mem_handle eof_timestamps[MAX_NUM_NVDLA_OUT_TIMESTAMP];
	struct nvdla_mem_handle memory_handles[NVDLA_MAX_BUFFERS_PER_TASK];
	u8 num_prefences;
	u8 num_postfences;
	u8 num_in_task_status;
	u8 num_sof_task_status;
	u8 num_eof_task_status;
	u8 num_sof_timestamps;
	u8 num_eof_timestamps;
	u32 num_addresses;
	u32 fence;
	u32 fence_counter;
	struct kref ref;
	struct list_head list;
	struct dla_task_descriptor *task_desc;
	dma_addr_t task_desc_pa;
	size_t buf_size;
	int timeout;
	int pool_index;

	struct dma_buf *memory_dmabuf[NVDLA_MAX_BUFFERS_PER_TASK];
	struct dma_buf *prefences_sem_dmabuf[MAX_NUM_NVDLA_PREFENCES];
	struct dma_buf *in_task_status_dmabuf[MAX_NUM_NVDLA_IN_TASK_STATUS];
	struct dma_buf *postfences_sem_dmabuf[MAX_NUM_NVDLA_POSTFENCES];
	struct dma_buf *sof_task_status_dmabuf[MAX_NUM_NVDLA_OUT_TASK_STATUS];
	struct dma_buf *eof_task_status_dmabuf[MAX_NUM_NVDLA_OUT_TASK_STATUS];
	struct dma_buf *sof_timestamps_dmabuf[MAX_NUM_NVDLA_OUT_TIMESTAMP];
	struct dma_buf *eof_timestamps_dmabuf[MAX_NUM_NVDLA_OUT_TIMESTAMP];
};

struct dla_mem_addr {
	uint64_t val;
};

extern const struct file_operations tegra_nvdla_ctrl_ops;
extern struct nvdla_queue_ops nvdla_queue_ops;

/**
 * nvhost_nvdla_finalize_poweron() finalize power on for DLA
 *
 * @pdev	Pointer for platform device
 *
 * Return	0 on success otherwise negative
 *
 * This function called from nvhost ACM subsystem,
 * to boot falcon and wait until falcon goes idle after initial setup
 */
int nvhost_nvdla_finalize_poweron(struct platform_device *pdev);

/**
 * nvhost_nvdla_prepare_poweron() prepare to poweroff DLA
 *
 * @pdev	Pointer for platform device
 *
 * Return	0 on success otherwise negative
 *
 * This function called from nvhost ACM subsystem,
 * disables falcon interrupts and pass PM core to powergate and clockgate
 */
int nvhost_nvdla_prepare_poweroff(struct platform_device *pdev);

/**
 * nvhost_nvdla_flcn_isr() falcon interrupt handler
 *
 * @pdev	Pointer for platform device
 *
 * Return	0 on success otherwise negative
 *
 * This function called from nvhost falcon subsystem on recieving falcon
 * interrupt, like INT_ON_COMPLETE, INT_ON_ERR, DLA_DEBUG etc.
 */
int nvhost_nvdla_flcn_isr(struct platform_device *pdev);

/**
 * nvdla_send_cmd() send command to DLA
 *
 * @pdev		Pointer for platform device
 * @cmd_data		Pointer command data
 *
 * Return		0 on success otherwise negative
 *
 * This function used to send method to falcon embedding different supporting
 * command. This uses THI registers to send method id and method data
 */
int nvdla_send_cmd(struct platform_device *pdev,
			struct nvdla_cmd_data *cmd_data);

/**
 * nvdla_task_put()	decrease task reference count
 *
 * @task		Pointer to task in operation
 *
 * Return		void
 *
 * This function puts task reference count and zero reference count
 * invokes function to free task.
 */
void nvdla_task_put(struct nvdla_task *task);

/**
 * nvdla_task_get()	increase task reference count
 *
 * @task		Pointer to task in operation
 *
 * Return		void
 *
 * This function gets task reference count
 */
void nvdla_task_get(struct nvdla_task *task);

/**
 * nvdla_task_alloc()	allocate task for a give queue
 *
 * @task		Pointer to nvdla_task
 *
 * Return		allocated task in success, otherwise pointer to err
 *
 * This function allocates task desc and fills up initial task descriptor as
 * task parameter detais
 */
int nvdla_fill_task_desc(struct nvdla_task *task);

/**
 * nvdla_send_postfences()	send back fences to UMD
 *
 * @task		Pointer to nvhost queue
 * @usr_task		Pointer to user task to be updated
 *
 * Return		0 on success otherwise negative
 *
 * This function send post fences back to UMD after task submit
 */
int nvdla_send_postfences(struct nvdla_task *task,
			struct nvdla_ioctl_submit_task *usr_task);

int nvdla_get_cmd_memory(struct platform_device *pdev,
				struct nvdla_cmd_mem_info *cmd_mem_info);
int nvdla_put_cmd_memory(struct platform_device *pdev, int index);
int nvdla_set_queue_state(struct nvdla_queue *queue, int cmd);
int nvdla_get_task_mem(struct nvdla_queue *queue,
				struct nvdla_task **task);
void nvdla_put_task_mem(struct nvdla_task *task);
size_t nvdla_get_max_task_size(void);
int nvdla_alloc_gcov_region(struct platform_device *pdev);
int nvdla_free_gcov_region(struct platform_device *pdev, bool update_region);

int nvdla_emulator_submit(struct nvdla_queue *queue,
				struct nvdla_emu_task *task);
void task_free(struct kref *ref);
int nvdla_get_signal_fences(struct nvdla_queue *queue, void *in_task);
int nvdla_send_gos_region(struct platform_device *pdev);

#endif /* End of __NVHOST_NVDLA_H__ */
