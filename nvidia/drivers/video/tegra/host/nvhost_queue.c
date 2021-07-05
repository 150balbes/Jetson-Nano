/*
 * NVHOST queue management for T194
 *
 * Copyright (c) 2016-2020, NVIDIA Corporation.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/dma-attrs.h>

#include <linux/nvhost.h>

#include "nvhost_vm.h"
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_queue.h"
#include "dev.h"

#define CMDBUF_SIZE	4096

/**
 * @brief Describe a task pool struct
 *
 * Array of fixed task memory is allocated during queue_alloc call.
 * The memory will be shared for various task based on availability
 *
 * dma_addr		Physical address of task memory pool
 * va			Virtual address of the task memory pool
 * kmem_addr		Kernel memory for task struct
 * lock			Mutex lock for the array access.
 * alloc_table		Keep track of the index being assigned
 *			and freed for a task
 * max_task_cnt		Maximum task count that can be supported.
 *
 */
struct nvhost_queue_task_pool {
	dma_addr_t dma_addr;
	void *va;
	void *kmem_addr;
	struct mutex lock;

	unsigned long alloc_table;
	unsigned long max_task_cnt;
};

static DEFINE_DMA_ATTRS(task_dma_attrs);

static int nvhost_queue_task_pool_alloc(struct platform_device *pdev,
					struct nvhost_queue *queue,
					unsigned int num_tasks)
{
	int err = 0;
	struct nvhost_queue_task_pool *task_pool;

	task_pool = queue->task_pool;

	/* Allocate the kernel memory needed for the task */
	if (queue->task_kmem_size) {
		task_pool->kmem_addr = kcalloc(num_tasks,
					queue->task_kmem_size, GFP_KERNEL);
		if (!task_pool->kmem_addr) {
			nvhost_err(&pdev->dev,
				   "failed to allocate task_pool->kmem_addr");
			err = -ENOMEM;
			goto err_alloc_task_kmem;
		}
	}

	/* Allocate memory for the task itself */
	task_pool->va = dma_alloc_attrs(&pdev->dev,
				queue->task_dma_size * num_tasks,
				&task_pool->dma_addr, GFP_KERNEL,
				__DMA_ATTR(task_dma_attrs));

	if (task_pool->va == NULL) {
		nvhost_err(&pdev->dev, "failed to allocate task_pool->va");
		err = -ENOMEM;
		goto err_alloc_task_pool;
	}
	task_pool->max_task_cnt = num_tasks;

	mutex_init(&task_pool->lock);

	return err;

err_alloc_task_pool:
	kfree(task_pool->kmem_addr);
err_alloc_task_kmem:
	return err;
}

static void nvhost_queue_task_free_pool(struct platform_device *pdev,
					struct nvhost_queue *queue)
{
	struct nvhost_queue_task_pool *task_pool =
		(struct nvhost_queue_task_pool *)queue->task_pool;

	dma_free_attrs(&queue->vm_pdev->dev,
			queue->task_dma_size * task_pool->max_task_cnt,
			task_pool->va, task_pool->dma_addr,
			__DMA_ATTR(task_dma_attrs));

	kfree(task_pool->kmem_addr);
	task_pool->max_task_cnt = 0;
	task_pool->alloc_table = 0;
}

static int nvhost_queue_dump(struct nvhost_queue_pool *pool,
		struct nvhost_queue *queue,
		struct seq_file *s)
{
	if (pool->ops && pool->ops->dump)
		pool->ops->dump(queue, s);

	return 0;
}

static int queue_dump(struct seq_file *s, void *data)
{
	struct nvhost_queue_pool *pool = s->private;
	unsigned long queue_id;

	mutex_lock(&pool->queue_lock);
	for_each_set_bit(queue_id, &pool->alloc_table,
			pool->max_queue_cnt)
		nvhost_queue_dump(pool, &pool->queues[queue_id], s);
	mutex_unlock(&pool->queue_lock);
	return 0;
}

static int queue_expose_open(struct inode *inode, struct file *file)
{
	return single_open(file, queue_dump, inode->i_private);
}

static const struct file_operations queue_expose_operations = {
	.open = queue_expose_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

struct nvhost_queue_pool *nvhost_queue_init(struct platform_device *pdev,
					struct nvhost_queue_ops *ops,
					unsigned int num_queues)
{
	struct nvhost_device_data *pdata;
	struct nvhost_queue_pool *pool;
	struct nvhost_queue *queues;
	struct nvhost_queue *queue;
	struct nvhost_queue_task_pool *task_pool;
	unsigned int i;
	int err;

	pool = kzalloc(sizeof(struct nvhost_queue_pool), GFP_KERNEL);
	if (pool == NULL) {
		nvhost_err(&pdev->dev, "failed to allocate queue pool");
		err = -ENOMEM;
		goto fail_alloc_pool;
	}

	queues = kcalloc(num_queues, sizeof(struct nvhost_queue), GFP_KERNEL);
	if (queues == NULL) {
		nvhost_err(&pdev->dev, "failed to allocate queues");
		err = -ENOMEM;
		goto fail_alloc_queues;
	}

	task_pool = kcalloc(num_queues,
			sizeof(struct nvhost_queue_task_pool), GFP_KERNEL);
	if (task_pool == NULL) {
		nvhost_err(&pdev->dev, "failed to allocate task_pool");
		err = -ENOMEM;
		goto fail_alloc_task_pool;
	}

	pdata = platform_get_drvdata(pdev);

	/* initialize pool and queues */
	pool->pdev = pdev;
	pool->ops = ops;
	pool->queues = queues;
	pool->alloc_table = 0;
	pool->max_queue_cnt = num_queues;
	pool->queue_task_pool = task_pool;
	mutex_init(&pool->queue_lock);

	debugfs_create_file("queues", S_IRUGO,
			pdata->debugfs, pool,
			&queue_expose_operations);


	for (i = 0; i < num_queues; i++) {
		queue = &queues[i];
		queue->id = i;
		queue->pool = pool;
		queue->task_pool = (void *)&task_pool[i];
		nvhost_queue_get_task_size(queue);
	}

	return pool;

fail_alloc_task_pool:
	kfree(pool->queues);
fail_alloc_queues:
	kfree(pool);
fail_alloc_pool:
	return ERR_PTR(err);
}

void nvhost_queue_deinit(struct nvhost_queue_pool *pool)
{
	if (!pool)
		return;

	kfree(pool->queue_task_pool);
	kfree(pool->queues);
	kfree(pool);
	pool = NULL;
}

void nvhost_queue_abort_all(struct nvhost_queue_pool *pool)
{
	u32 id;

	mutex_lock(&pool->queue_lock);
	for_each_set_bit(id, &pool->alloc_table, pool->max_queue_cnt)
		nvhost_queue_abort(&pool->queues[id]);
	mutex_unlock(&pool->queue_lock);
}

static void nvhost_queue_release(struct kref *ref)
{
	struct nvhost_queue *queue = container_of(ref, struct nvhost_queue,
						kref);
	struct nvhost_queue_pool *pool = queue->pool;

	nvhost_dbg_fn("");

	if (queue->use_channel)
		nvhost_putchannel(queue->channel, 1);

	/* release allocated resources */
	nvhost_syncpt_put_ref_ext(pool->pdev, queue->syncpt_id);

	/* free the task_pool */
	if (queue->task_dma_size)
		nvhost_queue_task_free_pool(pool->pdev, queue);

	/* ..and mark the queue free */
	mutex_lock(&pool->queue_lock);
	clear_bit(queue->id, &pool->alloc_table);
	mutex_unlock(&pool->queue_lock);
}

void nvhost_queue_put(struct nvhost_queue *queue)
{
	nvhost_dbg_fn("");
	kref_put(&queue->kref, nvhost_queue_release);
}

void nvhost_queue_get(struct nvhost_queue *queue)
{
	nvhost_dbg_fn("");
	kref_get(&queue->kref);
}

struct nvhost_queue *nvhost_queue_alloc(struct nvhost_queue_pool *pool,
					unsigned int num_tasks,
					bool use_channel)
{
	struct platform_device *pdev = pool->pdev;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_queue *queues = pool->queues;
	struct nvhost_queue *queue;
	int index = 0;
	int err = 0;

	mutex_lock(&pool->queue_lock);

	index = find_first_zero_bit(&pool->alloc_table,
				    pool->max_queue_cnt);

	/* quit if we found a queue */
	if (index >= pool->max_queue_cnt) {
		dev_err(&pdev->dev, "failed to get free Queue\n");
		err = -ENOMEM;
		goto err_alloc_queue;
	}

	/* reserve the queue */
	queue = &queues[index];
	set_bit(index, &pool->alloc_table);

	/* allocate a syncpt for the queue */
	queue->syncpt_id = nvhost_get_syncpt_host_managed(pdev, index, NULL);
	if (!queue->syncpt_id) {
		dev_err(&pdev->dev, "failed to get syncpt id\n");
		err = -ENOMEM;
		goto err_alloc_syncpt;
	}

	/* initialize queue ref count and sequence*/
	kref_init(&queue->kref);
	queue->use_channel = use_channel;
	queue->sequence = 0;

	/* initialize task list */
	INIT_LIST_HEAD(&queue->tasklist);
	mutex_init(&queue->list_lock);

	/* initialize task list */
	queue->attr = NULL;
	mutex_init(&queue->attr_lock);

	mutex_unlock(&pool->queue_lock);

	/* Check if the queue should allocate a channel */
	if (use_channel) {
		err = nvhost_channel_map(pdata, &queue->channel, queue);
		if (err < 0)
			goto err_alloc_channel;

		queue->vm_pdev = queue->channel->vm->pdev;
	} else {
		queue->vm_pdev = pdev;
	}

	if (queue->task_dma_size) {
		err = nvhost_queue_task_pool_alloc(queue->vm_pdev,
						   queue,
						   num_tasks);
		if (err < 0)
			goto err_alloc_task_pool;
	}

	return queue;

err_alloc_task_pool:
	if (use_channel)
		nvhost_putchannel(queue->channel, 1);
err_alloc_channel:
	mutex_lock(&pool->queue_lock);
	nvhost_syncpt_put_ref_ext(pdev, queue->syncpt_id);
err_alloc_syncpt:
	clear_bit(queue->id, &pool->alloc_table);
err_alloc_queue:
	mutex_unlock(&pool->queue_lock);
	return ERR_PTR(err);
}

int nvhost_queue_abort(struct nvhost_queue *queue)
{
	struct nvhost_queue_pool *pool = queue->pool;

	if (pool->ops && pool->ops->abort)
		return pool->ops->abort(queue);

	return 0;
}

int nvhost_queue_submit(struct nvhost_queue *queue, void *task_arg)
{
	struct nvhost_queue_pool *pool = queue->pool;

	if (pool->ops && pool->ops->submit)
		return pool->ops->submit(queue, task_arg);

	return 0;
}

int nvhost_queue_set_attr(struct nvhost_queue *queue, void *arg)
{
	struct nvhost_queue_pool *pool = queue->pool;

	if (pool->ops && pool->ops->set_attribute)
		return pool->ops->set_attribute(queue, arg);

	return 0;
}

struct nvhost_queue_task {
	struct platform_device *host1x_pdev;

	struct nvhost_queue *queue;

	dma_addr_t dma_addr;
	u32 *cpu_addr;
};

static void queue_task_update(void *priv, int nr_completed)
{
	struct nvhost_queue_task *task = priv;
	struct platform_device *host1x_pdev = task->host1x_pdev;

	dma_free_coherent(&host1x_pdev->dev,
			      CMDBUF_SIZE,
			      task->cpu_addr,
			      task->dma_addr);
	kfree(task);
}

int nvhost_queue_submit_to_host1x(struct nvhost_queue *queue,
				  u32 *cmdbuf,
				  u32 num_cmdbuf_words,
				  u32 num_syncpt_incrs,
				  u32 *wait_syncpt_ids,
				  u32 *wait_syncpt_thresholds,
				  u32 num_syncpt_waits,
				  u32 *task_syncpt_threshold)
{
	struct nvhost_queue_pool *pool = queue->pool;
	struct platform_device *client_pdev = pool->pdev;
	struct platform_device *host1x_pdev =
			to_platform_device(client_pdev->dev.parent);
	struct nvhost_device_data *pdata = platform_get_drvdata(client_pdev);
	struct nvhost_queue_task *task;
	struct nvhost_job *job;
	unsigned int i;
	int err = 0;

	if (queue->use_channel == false)
		return -EINVAL;

	/* Allocate memory for the task and task command buffer */
	task = kzalloc(sizeof(*task), GFP_KERNEL);
	if (task == NULL) {
		nvhost_err(&client_pdev->dev, "failed to allocate task");
		goto err_alloc_task;
	}

	task->cpu_addr = dma_alloc_coherent(&host1x_pdev->dev,
						CMDBUF_SIZE,
						&task->dma_addr,
						GFP_KERNEL);
	if (task->cpu_addr == NULL) {
		nvhost_err(&client_pdev->dev, "failed to allocate task");
		err = -ENOMEM;
		goto err_alloc_cmdbuf;
	}

	/* Copy the command buffer */
	memcpy(task->cpu_addr, cmdbuf, num_cmdbuf_words * 4);

	job = nvhost_job_alloc(queue->channel,
			       1,
			       0,
			       num_syncpt_waits,
			       1);
	if (job == NULL) {
		err = -ENOMEM;
		goto err_alloc_job;
	}

	task->queue = queue;
	task->host1x_pdev = host1x_pdev;

	/* Write waits to the job */
	job->num_waitchk = num_syncpt_waits;
	for (i = 0; i < num_syncpt_waits; i++) {
		job->waitchk[i].syncpt_id = wait_syncpt_ids[i];
		job->waitchk[i].thresh = wait_syncpt_thresholds[i];
		job->waitchk[i].mem = 0;
	}

	/* Initialize syncpoint increments */
	job->sp->id = queue->syncpt_id;
	job->sp->incrs = num_syncpt_incrs;
	job->num_syncpts = 1;

	/* Add the command buffer */
	nvhost_job_add_client_gather_address(job,
					     num_cmdbuf_words,
					     pdata->class,
					     task->dma_addr);

	/* Submit task to hardware */
	err = nvhost_channel_submit(job);
	if (err < 0)
		goto err_submit_job;

	/* Return the number of increments back to the caller */
	*task_syncpt_threshold = job->sp->fence;

	/* Register a callback function for releasing resources */
	err = nvhost_intr_register_notifier(host1x_pdev,
					    queue->syncpt_id,
					    job->sp->fence,
					    queue_task_update, task);
	if (err < 0) {
		nvhost_err(&client_pdev->dev,
			   "failed to register notifier err=%d",
			   err);
		goto err_register_notifier;
	}

	/* nvhost keeps a reference on the job and we don't
	 * need to access it anymore
	 */
	nvhost_job_put(job);

	return 0;

err_register_notifier:
err_submit_job:
	nvhost_job_put(job);
err_alloc_job:
	dma_free_coherent(&host1x_pdev->dev, CMDBUF_SIZE, task->cpu_addr,
			      task->dma_addr);
err_alloc_cmdbuf:
	kfree(task);
err_alloc_task:
	return err;
}

int nvhost_queue_get_task_size(struct nvhost_queue *queue)
{
	struct nvhost_queue_pool *pool = queue->pool;

	if (pool->ops && pool->ops->get_task_size)
		pool->ops->get_task_size(&queue->task_dma_size,
						&queue->task_kmem_size);

	return 0;
}

int nvhost_queue_alloc_task_memory(
			struct nvhost_queue *queue,
			struct nvhost_queue_task_mem_info *task_mem_info)
{
	int err = 0;
	int index, hw_offset, sw_offset;
	struct platform_device *pdev = queue->pool->pdev;
	struct nvhost_queue_task_pool *task_pool =
		(struct nvhost_queue_task_pool *)queue->task_pool;

	mutex_lock(&task_pool->lock);

	index = find_first_zero_bit(&task_pool->alloc_table,
				    task_pool->max_task_cnt);

	/* quit if pre-allocated task array is not free */
	if (index >= task_pool->max_task_cnt) {
		dev_err(&pdev->dev,
				"failed to get Task Pool Memory\n");
		err = -EAGAIN;
		goto err_alloc_task_mem;
	}

	/* assign the task array */
	set_bit(index, &task_pool->alloc_table);
	hw_offset = index * queue->task_dma_size;
	sw_offset = index * queue->task_kmem_size;
	task_mem_info->kmem_addr =
			(void *)((u8 *)task_pool->kmem_addr + sw_offset);
	task_mem_info->va = (void *)((u8 *)task_pool->va + hw_offset);
	task_mem_info->dma_addr = task_pool->dma_addr + hw_offset;
	task_mem_info->pool_index = index;

err_alloc_task_mem:
	mutex_unlock(&task_pool->lock);

	return err;
}

void nvhost_queue_free_task_memory(struct nvhost_queue *queue, int index)
{
	int hw_offset, sw_offset;
	u8 *task_kmem, *task_dma_va;
	struct nvhost_queue_task_pool *task_pool =
			(struct nvhost_queue_task_pool *)queue->task_pool;

	/* clear task kernel and dma virtual memory contents*/
	hw_offset = index * queue->task_dma_size;
	sw_offset = index * queue->task_kmem_size;
	task_kmem = (u8 *)task_pool->kmem_addr + sw_offset;
	task_dma_va = (u8 *)task_pool->va + hw_offset;

	memset(task_kmem, 0, queue->task_kmem_size);
	memset(task_dma_va, 0, queue->task_dma_size);

	mutex_lock(&task_pool->lock);
	clear_bit(index, &task_pool->alloc_table);
	mutex_unlock(&task_pool->lock);
}
