/*
 * NVHOST Queue management header for T194
 *
 * Copyright (c) 2016-2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_NVHOST_QUEUE_H__
#define __NVHOST_NVHOST_QUEUE_H__

#include <linux/kref.h>

struct nvhost_queue_task_pool;

/**
 * @brief	Describe a allocated task mem struct
 *
 * kmem_addr	Address for the task kernel memory
 * dma_addr	Physical address of task memory
 * va		Virtual address of the task memory
 * pool_index	Index to the allocated task memory
 *
 * This is keep track of the memory details of the task
 * struct that is being shared between kernel and firmware.
 */
struct nvhost_queue_task_mem_info {
	void *kmem_addr;
	dma_addr_t dma_addr;
	void *va;
	int pool_index;
};
/**
 * @brief		Information needed in a Queue
 *
 * pool			pointer queue pool
 * kref			struct kref for reference count
 * syncpt_id		Host1x syncpt id
 * id			Queue id
 * list_lock		mutex for tasks lists control
 * tasklist		Head of tasks list
 * sequence		monotonically incrementing task id per queue
 * task_pool		pointer to struct for task memory pool
 * task_dma_size	dma size used in hardware for a task
 * task_kmem_size	kernel memory size for a task
 * attr			queue attribute associated with the host module
 *
 */
struct nvhost_queue {
	struct nvhost_queue_task_pool *task_pool;
	struct nvhost_queue_pool *pool;
	struct kref kref;
	u32 id;

	/* Host1x resources */
	struct nvhost_channel *channel;
	struct platform_device *vm_pdev;
	bool use_channel;
	u32 syncpt_id;

	size_t task_dma_size;
	size_t task_kmem_size;

	u32 sequence;

	struct mutex attr_lock;
	void *attr;

	struct mutex list_lock;
	struct list_head tasklist;
};

/**
 * @brief	hardware specific queue callbacks
 *
 * dump			dump the task information
 * abort		abort all tasks from a queue
 * submit		submit the given list of tasks to hardware
 * get_task_size	get the dma size needed for the task in hw
 *			and the kernel memory size needed for task.
 *
 */
struct nvhost_queue_ops {
	void (*dump)(struct nvhost_queue *queue, struct seq_file *s);
	int (*abort)(struct nvhost_queue *queue);
	int (*submit)(struct nvhost_queue *queue, void *task_arg);
	void (*get_task_size)(size_t *dma_size, size_t *kmem_size);
	int (*set_attribute)(struct nvhost_queue *queue, void *arg);
};

/**
 * @brief	Queue pool data structure to hold queue table
 *
 * pdev			Pointer to the Queue client device
 * ops			Pointer to hardware specific queue ops
 * queues		Queues available for the client
 * queue_lock		Mutex for the bitmap of reserved queues
 * alloc_table		Bitmap of allocated queues
 * max_queue_cnt	Max number queues available for client
 * queue_task_pool	Pointer to the task memory pool for queues.
 *
 */
struct nvhost_queue_pool {
	struct platform_device *pdev;
	struct nvhost_queue_ops *ops;
	struct nvhost_queue *queues;
	struct mutex queue_lock;
	unsigned long alloc_table;
	unsigned int max_queue_cnt;
	void *queue_task_pool;
};

/**
 * @brief	Initialize queue structures
 *
 * This function allocates and initializes queue data structures.
 *
 * @param pdev		Pointer to the Queue client device
 * @param ops		Pointer to device speicific callbacks
 * @param num_queues	Max number queues available for client
 * @return		pointer to queue pool
 *
 */
struct nvhost_queue_pool *nvhost_queue_init(struct platform_device *pdev,
					struct nvhost_queue_ops *ops,
					unsigned int num_queues);

/**
 * @brief	De-initialize queue structures
 *
 * This function free's all queue data structures.
 *
 * @param pool	pointer to queue pool
 * @return	void
 *
 */
void nvhost_queue_deinit(struct nvhost_queue_pool *pool);

/**
 * @brief	Release reference of a queue
 *
 * This function releases reference for a queue.
 *
 * @param queue	Pointer to an allocated queue.
 * @return	void
 *
 */
void nvhost_queue_put(struct nvhost_queue *queue);

/**
 * @brief	Get reference on a queue.
 *
 * This function used to get a reference to an already allocated queue.
 *
 * @param queue	Pointer to an allocated queue.
 * @return	None
 *
 */
void nvhost_queue_get(struct nvhost_queue *queue);

/**
 * @brief	Allocate a queue for client.
 *
 * This function allocates a queue from the pool to client for the user.
 *
 * @param pool		Pointer to a queue pool table
 * @param num_tasks	Max number of tasks per queue
 * @param use_channel	Determines whether the routine allocates a channel for
 *			the queue.
 *
 * @return		Pointer to a queue struct on success
 *			or negative error on failure.
 *
 */
struct nvhost_queue *nvhost_queue_alloc(struct nvhost_queue_pool *pool,
					unsigned int num_tasks,
					bool use_channel);

/**
 * @brief		Abort all active queues
 *
 * @param pool		Pointer to a queue pool table
 */
void nvhost_queue_abort_all(struct nvhost_queue_pool *pool);

/**
 * @brief	Abort tasks within a client queue
 *
 * This function aborts all tasks from the given clinet queue. If there is no
 * active tasks, the function call is no-op.
 * It is expected to be called when an active device fd gets closed.
 *
 * @param queue	Pointer to an allocated queue
 * @return	None
 *
 */
int nvhost_queue_abort(struct nvhost_queue *queue);

/**
 * @brief	submits the given list of tasks to hardware
 *
 * This function submits the given list of tasks to hardware.
 * The submit structure is updated with the fence values as appropriate.
 *
 * @param queue		Pointer to an allocated queue
 * @param submit	Submit the given list of tasks to hardware
 * @return		0 on success or negative error code on failure.
 *
 */
int nvhost_queue_submit(struct nvhost_queue *queue, void *submit);

/**
 * @brief	Get the Task Size needed
 *
 * This function get the needed memory size for the task. This memory is
 * shared memory between kernel and firmware
 *
 * @param queue	Pointer to an allocated queue
 * @return	Size of the task
 *
 */
int nvhost_queue_get_task_size(struct nvhost_queue *queue);

/**
 * @brief	Allocate a memory from task memory pool
 *
 * This function helps to assign a task memory from
 * the preallocated task memory pool. This memory is shared memory between
 * kernel and firmware
 *
 * @queue		Pointer to an allocated queue
 * @task_mem_info	Pointer to nvhost_queue_task_mem_info struct
 *
 * @return	0 on success, otherwise a negative error code is returned
 *
 */
int nvhost_queue_alloc_task_memory(
			struct nvhost_queue *queue,
			struct nvhost_queue_task_mem_info *task_mem_info);

/**
 * @brief	Free the assigned task memory
 *
 * This function helps to unset the assigned task memory
 *
 * @param queue	Pointer to an allocated queue
 * @param index	Index of the assigned task pool memory
 * @return	void
 *
 */
void nvhost_queue_free_task_memory(struct nvhost_queue *queue, int index);

/**
 * @brief	Sets the attribute to the queue
 *
 * This function set the attribute of the queue with the arguments passed
 *
 * @param queue		Pointer to an allocated queue
 * @param arg		The structure which consists of the id and value
 * @return		0 on success or negative error code on failure.
 *
 */
int nvhost_queue_set_attr(struct nvhost_queue *queue, void *arg);

/**
 * @brief		Submit the given command buffer to the device
 *
 * This functions submits the given cmdbuf to a Host1x channel. The
 * submit must perform the given number of syncpoint increments
 * on the engine.
 *
 * @param queue				Pointer to an allocated queue
 * @param cmdbuf			Pointer to a command buffer to submit
 * @param num_cmdbuf_words		Number of words in the command buffer
 * @param num_syncpt_incrs		Number of syncpoint increments the task
 *					issues
 * @param wait_syncpt_ids		Syncpoint ids that should be waited on
 *					Host1x
 * @param wait_syncpoint_thresholds	Syncpoint thresholds for the waits
 * @param task_syncpt_threshold		Pointer to a u32. The variable is
 *					initialized to have the completion
 *					threshold.
 *
 * @return				0 on success or negative error code on
 *					failure.
 *
 */
int nvhost_queue_submit_to_host1x(struct nvhost_queue *queue,
				  u32 *cmdbuf,
				  u32 num_cmdbuf_words,
				  u32 num_syncpt_incrs,
				  u32 *wait_syncpt_ids,
				  u32 *wait_syncpt_thresholds,
				  u32 num_syncpt_waits,
				  u32 *task_syncpt_threshold);

#endif
