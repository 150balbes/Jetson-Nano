/*
 * PVA Ioctl Handling for T194
 *
 * Copyright (c) 2016-2019, NVIDIA Corporation.  All rights reserved.
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
#include <linux/nospec.h>

#include <asm/ioctls.h>
#include <asm/barrier.h>

#include <uapi/linux/nvdev_fence.h>
#include <uapi/linux/nvhost_pva_ioctl.h>

#include "pva.h"
#include "pva_queue.h"
#include "dev.h"
#include "nvhost_buffer.h"
#include "nvhost_acm.h"

/**
 * @brief pva_private - Per-fd specific data
 *
 * pdev		Pointer the pva device
 * queue	Pointer the struct nvhost_queue
 * buffer	Pointer to the struct nvhost_buffer
 */
struct pva_private {
	struct pva *pva;
	struct nvhost_queue *queue;
	struct nvhost_buffers *buffers;
};

/**
 * @brief	Copy a single task from userspace to kernel space
 *
 * This function copies fields from ioctl_task and performs a deep copy
 * of the task to kernel memory. At the same time, input values shall
 * be validated. This allows using all the fields without manually performing
 * copies of the structure and performing checks later.
 *
 * @param ioctl_task	Pointer to a userspace task that is copied
 *				to kernel memory
 * @param task		Pointer to a task that should be created
 * @return		0 on Success or negative error code
 *
 */
static int pva_copy_task(struct pva_ioctl_submit_task *ioctl_task,
			 struct pva_submit_task *task)
{
	int err = 0;
	int copy_ret = 0;
	int i;

	if (ioctl_task->num_prefences > PVA_MAX_PREFENCES ||
	    ioctl_task->num_input_task_status > PVA_MAX_INPUT_STATUS ||
	    ioctl_task->num_output_task_status > PVA_MAX_OUTPUT_STATUS ||
	    ioctl_task->num_input_surfaces > PVA_MAX_INPUT_SURFACES ||
	    ioctl_task->num_output_surfaces > PVA_MAX_OUTPUT_SURFACES ||
	    ioctl_task->num_pointers > PVA_MAX_POINTERS ||
	    ioctl_task->primary_payload_size > PVA_MAX_PRIMARY_PAYLOAD_SIZE) {
		err = -EINVAL;
		goto err_out;
	}

	/*
	 * These fields are clear-text in the task descriptor. Just
	 * copy them.
	 */
	task->operation			= ioctl_task->operation;
	task->num_prefences		= ioctl_task->num_prefences;
	task->num_input_task_status	= ioctl_task->num_input_task_status;
	task->num_output_task_status	= ioctl_task->num_output_task_status;
	task->num_input_surfaces	= ioctl_task->num_input_surfaces;
	task->num_output_surfaces	= ioctl_task->num_output_surfaces;
	task->num_pointers		= ioctl_task->num_pointers;
	task->primary_payload_size	= ioctl_task->primary_payload_size;
	task->input_scalars		= ioctl_task->input_scalars;
	task->output_scalars		= ioctl_task->output_scalars;
	task->timeout			= ioctl_task->timeout;

	/* Copy the user primary_payload */
	if (task->primary_payload_size) {
		copy_ret = copy_from_user(task->primary_payload,
				(void __user *)(ioctl_task->primary_payload),
				ioctl_task->primary_payload_size);
		if (copy_ret) {
			err = -EFAULT;
			goto err_out;
		}
	}

#define COPY_FIELD(dst, src, num, type)					\
	do {								\
		if ((num) == 0) {					\
			break;						\
		}							\
		copy_ret = copy_from_user((dst),			\
				(void __user *)(src),			\
				(num) * sizeof(type));			\
		if (copy_ret) {						\
			err = -EFAULT;					\
			goto err_out;					\
		}							\
	} while (0)

	/* Copy the fields */
	COPY_FIELD(task->input_surfaces, ioctl_task->input_surfaces,
			task->num_input_surfaces,
			struct pva_surface);
	COPY_FIELD(task->output_surfaces, ioctl_task->output_surfaces,
			task->num_output_surfaces,
			struct pva_surface);
	COPY_FIELD(task->prefences, ioctl_task->prefences, task->num_prefences,
			struct nvdev_fence);
	COPY_FIELD(task->input_task_status, ioctl_task->input_task_status,
			task->num_input_task_status,
			struct pva_status_handle);
	COPY_FIELD(task->output_task_status, ioctl_task->output_task_status,
			task->num_output_task_status,
			struct pva_status_handle);
	COPY_FIELD(task->pointers, ioctl_task->pointers,
			task->num_pointers, struct pva_memory_handle);

	COPY_FIELD(task->pvafences, ioctl_task->pvafences,
			sizeof(task->pvafences), u8);
	COPY_FIELD(task->num_pvafences, ioctl_task->num_pvafences,
		   sizeof(task->num_pvafences), u8);
	COPY_FIELD(task->num_pva_ts_buffers, ioctl_task->num_pva_ts_buffers,
		   sizeof(task->num_pva_ts_buffers), u8);

	for (i = 0; i < PVA_MAX_FENCE_TYPES; i++) {
		if ((task->num_pvafences[i] > PVA_MAX_FENCES_PER_TYPE) ||
			(task->num_pva_ts_buffers[i] >
			 PVA_MAX_FENCES_PER_TYPE)) {
			err = -EINVAL;
			goto err_out;
		}
	}


#undef COPY_FIELD

err_out:
	return err;
}

/**
 * @brief	Submit a task to PVA
 *
 * This function takes the given list of tasks, converts
 * them into kernel internal representation and submits
 * them to the task queue. On success, it populates
 * the post-fence structures in userspace and returns 0.
 *
 * @param priv	PVA Private data
 * @param arg	ioctl data
 * @return	0 on Success or negative error code
 *
 */
static int pva_submit(struct pva_private *priv, void *arg)
{
	struct pva_ioctl_submit_args *ioctl_tasks_header =
		(struct pva_ioctl_submit_args *)arg;
	struct pva_ioctl_submit_task *ioctl_tasks = NULL;
	struct pva_submit_tasks tasks_header;
	struct pva_submit_task *task = NULL;
	int err = 0;
	int i;
	int j;
	int k;
	int threshold;

	memset(&tasks_header, 0, sizeof(tasks_header));

	/* Sanity checks for the task heaader */
	if (ioctl_tasks_header->num_tasks > PVA_MAX_TASKS) {
		err = -EINVAL;
		goto err_check_num_tasks;
	}

	ioctl_tasks_header->num_tasks =
		array_index_nospec(ioctl_tasks_header->num_tasks,
					PVA_MAX_TASKS + 1);

	if (ioctl_tasks_header->version > 0) {
		err = -ENOSYS;
		goto err_check_version;
	}

	/* Allocate memory for the UMD representation of the tasks */
	ioctl_tasks = kcalloc(ioctl_tasks_header->num_tasks,
			sizeof(*ioctl_tasks), GFP_KERNEL);
	if (!ioctl_tasks) {
		err = -ENOMEM;
		goto err_alloc_task_mem;
	}

	/* Copy the tasks from userspace */
	err = copy_from_user(ioctl_tasks,
			(void __user *)ioctl_tasks_header->tasks,
			ioctl_tasks_header->num_tasks * sizeof(*ioctl_tasks));
	if (err < 0) {
		err = -EFAULT;
		goto err_copy_tasks;
	}

	/* Go through the tasks and make a KMD representation of them */
	for (i = 0; i < ioctl_tasks_header->num_tasks; i++) {

		struct nvhost_queue_task_mem_info task_mem_info;

		/* Allocate memory for the task and dma */
		err = nvhost_queue_alloc_task_memory(priv->queue,
							&task_mem_info);
		task = task_mem_info.kmem_addr;
		if ((err < 0) || !task)
			goto err_get_task_buffer;

		err = pva_copy_task(ioctl_tasks + i, task);
		if (err < 0)
			goto err_copy_tasks;

		INIT_LIST_HEAD(&task->node);
		/* Obtain an initial reference */
		kref_init(&task->ref);

		task->pva = priv->pva;
		task->queue = priv->queue;
		task->buffers = priv->buffers;

		task->dma_addr = task_mem_info.dma_addr;
		task->va = task_mem_info.va;
		task->pool_index = task_mem_info.pool_index;

		tasks_header.tasks[i] = task;
		tasks_header.num_tasks += 1;
	}

	/* Populate header structure */
	tasks_header.flags = ioctl_tasks_header->flags;

	/* ..and submit them */
	err = nvhost_queue_submit(priv->queue, &tasks_header);

	if (err < 0) {
		goto err_submit_task;
	}

	/* Copy fences back to userspace */
	for (i = 0; i < ioctl_tasks_header->num_tasks; i++) {
		struct nvpva_fence __user *pvafences =
			(struct nvpva_fence __user *)
			ioctl_tasks[i].pvafences;

		struct platform_device *host1x_pdev =
			to_platform_device(priv->queue->vm_pdev->dev.parent);

		task = tasks_header.tasks[i];

		threshold = tasks_header.task_thresh[i] - task->fence_num + 1;

		/* Return post-fences */
		for (k = 0; k < PVA_MAX_FENCE_TYPES; k++) {
			u32 increment = 0;
			struct nvdev_fence *fence;

			if ((task->num_pvafences[k] == 0) ||
				(k == PVA_FENCE_PRE)) {
				continue;
			}

			switch (k) {
			case PVA_FENCE_SOT_V:
				increment = 1;
				break;
			case PVA_FENCE_SOT_R:
				increment = 1;
				break;
			case PVA_FENCE_POST:
				increment = 1;
				break;
			default:

				break;
			};

			for (j = 0; j < task->num_pvafences[k]; j++) {
				fence = &task->pvafences[k][j].fence;

				switch (fence->type) {
				case NVDEV_FENCE_TYPE_SYNCPT: {
					fence->syncpoint_index =
						priv->queue->syncpt_id;
					fence->syncpoint_value =
						threshold;
					threshold += increment;
					break;
				}
				case NVDEV_FENCE_TYPE_SYNC_FD: {
					struct nvhost_ctrl_sync_fence_info pts;

					pts.id = priv->queue->syncpt_id;
					pts.thresh = threshold;
					threshold += increment;
					err = nvhost_sync_create_fence_fd(
						host1x_pdev,
						&pts, 1,
						"fence_pva",
						&fence->sync_fd);

					break;
				}
				case NVDEV_FENCE_TYPE_SEMAPHORE:
					break;
				default:
					err = -ENOSYS;
					nvhost_warn(&priv->pva->pdev->dev,
						    "Bad fence type");
				}
			}
		}

		err = copy_to_user(pvafences,
				   task->pvafences,
				   sizeof(task->pvafences));
		if (err < 0) {
			nvhost_warn(&priv->pva->pdev->dev,
				    "Failed to copy  pva fences to userspace");
			break;
		}
		/* Drop the reference */
		kref_put(&task->ref, pva_task_free);
	}

	kfree(ioctl_tasks);
	return 0;

err_submit_task:
err_get_task_buffer:
err_copy_tasks:
	for (i = 0; i < tasks_header.num_tasks; i++) {
		task = tasks_header.tasks[i];
		/* Drop the reference */
		kref_put(&task->ref, pva_task_free);
	}
err_alloc_task_mem:
	kfree(ioctl_tasks);
err_check_version:
err_check_num_tasks:
	return err;
}

/**
 * pva_queue_set_attr() - Set attribute to the queue
 *
 * @priv: PVA Private data
 * @arg: ioctl data
 *
 * This function set the attributes of the pv queue.
 */
static int pva_queue_set_attr(struct pva_private *priv, void *arg)
{
	struct pva_ioctl_queue_attr *ioctl_queue_attr =
		(struct pva_ioctl_queue_attr *)arg;
	struct pva_queue_attribute *attrs;
	struct pva_queue_attribute attr;
	struct pva_queue_set_attribute set_attr;
	int id = ioctl_queue_attr->id;
	int val = ioctl_queue_attr->val;
	int err = 0;

	/* Only root is allowed to update queue attributes */
	if (current_uid().val != 0) {
		err = -EINVAL;
		goto end;
	}

	/* Sanity checks for the task heaader */
	if (id >= QUEUE_ATTR_MAX) {
		err = -ENOSYS;
		goto end;
	}

	id = array_index_nospec(id, QUEUE_ATTR_MAX);

	/* Initialize attribute for setting */
	attr.id = id;
	attr.value = val;

	/* Update the attribute cache */
	mutex_lock(&priv->queue->attr_lock);
	attrs =	priv->queue->attr;
	if (!attrs) {
		mutex_unlock(&priv->queue->attr_lock);
		err = -EINVAL;
		goto end;
	}
	attrs[id] = attr;
	mutex_unlock(&priv->queue->attr_lock);

	/* Turn on the hardware */
	err = nvhost_module_busy(priv->pva->pdev);
	if (err)
		goto end;

	/* Set attribute on hardware */
	set_attr.pva = priv->pva;
	set_attr.attr = &attr;
	set_attr.bootup = false;
	err = nvhost_queue_set_attr(priv->queue, &set_attr);

	/* Drop PM runtime reference of PVA */
	nvhost_module_idle(priv->pva->pdev);
end:
	return err;

}

static int pva_pin(struct pva_private *priv, void *arg)
{
	u32 *handles;
	int err = 0;
	int i = 0;
	struct dma_buf *dmabufs[PVA_MAX_PIN_BUFFERS];
	struct pva_pin_unpin_args *buf_list = (struct pva_pin_unpin_args *)arg;
	u32 count = buf_list->num_buffers;

	if (count > PVA_MAX_PIN_BUFFERS)
		return -EINVAL;

	handles = kcalloc(count, sizeof(u32), GFP_KERNEL);
	if (!handles)
		return -ENOMEM;

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
			(count * sizeof(u32)))) {
		err = -EFAULT;
		goto pva_buffer_cpy_err;
	}

	/* get the dmabuf pointer from the fd handle */
	for (i = 0; i < count; i++) {
		dmabufs[i] = dma_buf_get(handles[i]);
		if (IS_ERR_OR_NULL(dmabufs[i])) {
			err = -EFAULT;
			goto pva_buffer_get_err;
		}
	}

	err = nvhost_buffer_pin(priv->buffers, dmabufs, count);

pva_buffer_get_err:
	count = i;
	for (i = 0; i < count; i++)
		dma_buf_put(dmabufs[i]);

pva_buffer_cpy_err:
	kfree(handles);
	return err;
}

static int pva_unpin(struct pva_private *priv, void *arg)
{
	u32 *handles;
	int i = 0;
	int err = 0;
	struct dma_buf *dmabufs[PVA_MAX_PIN_BUFFERS];
	struct pva_pin_unpin_args *buf_list = (struct pva_pin_unpin_args *)arg;
	u32 count = buf_list->num_buffers;

	if (count > PVA_MAX_PIN_BUFFERS)
		return -EINVAL;

	handles = kcalloc(count, sizeof(u32), GFP_KERNEL);
	if (!handles)
		return -ENOMEM;

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
			(count * sizeof(u32)))) {
		err = -EFAULT;
		goto pva_buffer_cpy_err;
	}

	/* get the dmabuf pointer and clean valid ones */
	for (i = 0; i < count; i++) {
		dmabufs[i] = dma_buf_get(handles[i]);
		if (IS_ERR_OR_NULL(dmabufs[i]))
			continue;
	}

	nvhost_buffer_unpin(priv->buffers, dmabufs, count);

	for (i = 0; i < count; i++) {
		if (IS_ERR_OR_NULL(dmabufs[i]))
			continue;

		dma_buf_put(dmabufs[i]);
	}

pva_buffer_cpy_err:
	kfree(handles);
	return err;
}

static int pva_get_characteristics(struct pva_private *priv,
		void *arg)
{
	struct pva_characteristics_req pva_char_req;
	struct pva_characteristics pva_char;
	struct pva_characteristics_req *in_pva_char =
			(struct pva_characteristics_req *)arg;
	u64 in_size = in_pva_char->characteristics_size;
	u64 out_size = sizeof(struct pva_characteristics);
	struct pva_version_info info;
	struct pva *pva = priv->pva;
	int err = 0;

	/* check whether the characteristics has NULL pointer */
	if (!in_pva_char->characteristics) {
		err = -EINVAL;
		goto err_check_characteristics_ptr;
	}

	err = nvhost_module_busy(pva->pdev);
	if (err < 0)
		goto err_poweron;

	err = pva_get_firmware_version(pva, &info);
	if (err < 0)
		goto err_get_firmware_version;

	memset(&pva_char, 0, out_size);
	pva_char.num_vpu = 2;
	pva_char.num_queues = MAX_PVA_QUEUE_COUNT;
	pva_char.submit_mode = pva->submit_mode;
	pva_char.pva_r5_version = info.pva_r5_version;
	pva_char.pva_compat_version = info.pva_compat_version;
	pva_char.pva_revision = info.pva_revision;
	pva_char.pva_built_on = info.pva_built_on;

	/* if input_size more than output_size, copy kernel struct size */
	if (in_size > out_size)
		in_size = out_size;

	/* copy input_size of data to output*/
	pva_char_req.characteristics_filled = in_size;

	err = copy_to_user((void __user *)in_pva_char->characteristics,
			&pva_char,
			in_size);

err_get_firmware_version:
	nvhost_module_idle(pva->pdev);
err_poweron:
err_check_characteristics_ptr:
	return err;
}

static int pva_copy_function_table(struct pva_private *priv,
		void *arg)
{
	struct pva_ioctl_vpu_func_table *ioctl_fn_table = arg;
	struct pva_func_table fn_table;
	struct pva *pva = priv->pva;
	uint32_t table_size;
	int err;

	err = nvhost_module_busy(pva->pdev);
	if (err) {
		nvhost_dbg_info("error in powering up pva\n");
		goto err_poweron;
	}

	err = pva_alloc_and_populate_function_table(pva, &fn_table);
	if (err) {
		nvhost_dbg_info("unable to populate function table\n");
		goto err_vpu_alloc;
	}

	table_size = ioctl_fn_table->size;

	if (fn_table.size > table_size) {
		err = -ENOMEM;
		goto err_table_copy;
	}

	err = copy_to_user((void __user *)ioctl_fn_table->addr,
			fn_table.addr,
			fn_table.size);
	if (err < 0)
		goto err_table_copy;

	ioctl_fn_table->size = fn_table.size;
	ioctl_fn_table->entries = fn_table.entries;

err_table_copy:
	pva_dealloc_vpu_function_table(pva, &fn_table);
err_vpu_alloc:
	nvhost_module_idle(pva->pdev);
err_poweron:
	return err;
}

/**
 * pva_set_rate() - Set PVA minimum frequencies
 *
 * @priv: PVA Private data
 * @arg: ioctl data
 *
 * This function sets PVA minimum frequencies
 */
static int pva_set_rate(struct pva_private *priv, void *arg)
{
	struct pva_ioctl_rate *ioctl_rate = (struct pva_ioctl_rate *)arg;
	unsigned long new_rate = max_t(u64,
				       MIN_PVA_FREQUENCY,
				       ioctl_rate->rate);
	int err;


	/* Set R5 minimum frequency */
	err = nvhost_module_set_rate(priv->pva->pdev,
				     priv,
				     new_rate,
				     0,
				     ioctl_rate->type);
	if (err < 0)
		return err;

	/* Set VPU0 minimum frequency */
	err = nvhost_module_set_rate(priv->pva->pdev,
				     priv,
				     new_rate,
				     1,
				     ioctl_rate->type);
	if (err < 0)
		return err;

	/* Set VPU1 minimum frequency */
	err = nvhost_module_set_rate(priv->pva->pdev,
				     priv,
				     new_rate,
				     2,
				     ioctl_rate->type);
	if (err < 0)
		return err;

	return 0;
}

/**
 * pva_get_rate() - Get current PVA frequency
 *
 * @priv: PVA Private data
 * @arg: ioctl data
 *
 * This function gets the current PVA frequency
 */
static int pva_get_rate(struct pva_private *priv, void *arg)
{
	struct pva_ioctl_rate *ioctl_rate = (struct pva_ioctl_rate *)arg;
	unsigned long rate;
	int err;

	err = nvhost_module_get_rate(priv->pva->pdev,
				     &rate,
				     0);

	ioctl_rate->rate = rate;
	ioctl_rate->type = NVHOST_CLOCK;

	return err;
}

static long pva_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct pva_private *priv = file->private_data;
	u8 buf[NVHOST_PVA_IOCTL_MAX_ARG_SIZE] __aligned(sizeof(u64));
	int err = 0;

	nvhost_dbg_fn("");

	if ((_IOC_TYPE(cmd) != NVHOST_PVA_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_PVA_IOCTL_LAST) ||
		(_IOC_SIZE(cmd) > NVHOST_PVA_IOCTL_MAX_ARG_SIZE))
		return -ENOIOCTLCMD;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {

	case PVA_IOCTL_CHARACTERISTICS:
	{
		err = pva_get_characteristics(priv, buf);
		break;
	}
	case PVA_IOCTL_PIN:
	{
		err = pva_pin(priv, buf);
		break;
	}
	case PVA_IOCTL_UNPIN:
	{
		err = pva_unpin(priv, buf);
		break;
	}
	case PVA_IOCTL_SUBMIT:
	{
		err = pva_submit(priv, buf);
		break;
	}
	case PVA_IOCTL_SET_QUEUE_ATTRIBUTES:
	{
		err = pva_queue_set_attr(priv, buf);
		break;
	}
	case PVA_IOCTL_COPY_VPU_FUNCTION_TABLE:
	{
		err = pva_copy_function_table(priv, buf);
		break;
	}
	case PVA_IOCTL_SET_RATE:
	{
		err = pva_set_rate(priv, buf);
		break;
	}
	case PVA_IOCTL_GET_RATE:
	{
		err = pva_get_rate(priv, buf);
		break;
	}
	default:
		return -ENOIOCTLCMD;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static struct pva_queue_attribute default_queue_attr[QUEUE_ATTR_MAX] = {
	{0, 0},
	{QUEUE_ATTR_PRIORITY, PVA_QUEUE_DEFAULT_PRIORITY},
	{QUEUE_ATTR_VPU, PVA_QUEUE_DEFAULT_VPU_MASK},
	{QUEUE_ATTR_MISR_TO, 0xFFFFFFFF}
};

static void pva_queue_set_default_attr(struct pva_private *priv)
{
	u32 id;
	struct pva_queue_attribute *attr;

	attr = priv->queue->attr;
	for (id = 1; id < QUEUE_ATTR_MAX; id++) {
		attr[id].id = default_queue_attr[id].id;
		attr[id].value = default_queue_attr[id].value;
	}
}

static int pva_open_set_attrs(struct pva_private *priv)
{
	struct pva_queue_set_attribute set_attr;
	unsigned int i;
	int err = 0;

	/* Turn on the hardware */
	err = nvhost_module_busy(priv->pva->pdev);
	if (err < 0)
		goto end;

	/* Set attribute on hardware */
	set_attr.pva = priv->pva;
	set_attr.bootup = false;

	for (i = 1; i < QUEUE_ATTR_MAX; i++) {
		set_attr.attr = &default_queue_attr[i];
		err = nvhost_queue_set_attr(priv->queue, &set_attr);
		if (err < 0)
			break;
	}

	nvhost_module_idle(priv->pva->pdev);

end:
	return err;
}

static int pva_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct pva_queue_attribute *attr;
	struct pva *pva = pdata->private_data;
	struct pva_private *priv;
	int err = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		err = -ENOMEM;
		goto err_alloc_priv;
	}

	attr = kzalloc((sizeof(*attr) * QUEUE_ATTR_MAX), GFP_KERNEL);
	if (!attr) {
		err = -ENOMEM;
		dev_info(&pdev->dev, "unable to allocate memory for attributes\n");
		goto err_alloc_attr;
	}

	file->private_data = priv;
	priv->pva = pva;

	/* add the pva client to nvhost */
	err = nvhost_module_add_client(pdev, priv);
	if (err < 0)
		goto err_add_client;

	priv->queue = nvhost_queue_alloc(pva->pool, MAX_PVA_TASK_COUNT,
		pva->submit_mode == PVA_SUBMIT_MODE_CHANNEL_CCQ);
	if (IS_ERR(priv->queue)) {
		err = PTR_ERR(priv->queue);
		goto err_alloc_queue;
	}

	priv->buffers = nvhost_buffer_init(priv->queue->vm_pdev);
	if (IS_ERR(priv->buffers)) {
		err = PTR_ERR(priv->buffers);
		goto err_alloc_buffer;
	}

	/* Ensure that the stashed attributes are valid */
	mutex_lock(&priv->queue->attr_lock);
	priv->queue->attr = attr;
	pva_queue_set_default_attr(priv);
	mutex_unlock(&priv->queue->attr_lock);

	/* Restore the default attributes in hardware */
	err = pva_open_set_attrs(priv);
	if (err < 0)
		dev_warn(&pdev->dev, "failed to restore queue attributes\n");

	return nonseekable_open(inode, file);

err_alloc_buffer:
	nvhost_queue_put(priv->queue);
err_alloc_queue:
	nvhost_module_remove_client(pdev, priv);
err_add_client:
	kfree(attr);
err_alloc_attr:
	kfree(priv);
err_alloc_priv:
	return err;
}

static int pva_release(struct inode *inode, struct file *file)
{
	struct pva_private *priv = file->private_data;

	/*
	 * Queue attributes are referenced from the queue
	 * structure. Release the attributes before the queue
	 * reference.
	 */
	mutex_lock(&priv->queue->attr_lock);
	kfree(priv->queue->attr);
	priv->queue->attr = NULL;
	mutex_unlock(&priv->queue->attr_lock);

	/*
	 * Release handle to the queue (on-going tasks have their
	 * own references to the queue
	 */
	nvhost_queue_put(priv->queue);

	/* Release handle to nvhost_acm */
	nvhost_module_remove_client(priv->pva->pdev, priv);

	/* Release the handle to buffer structure */
	nvhost_buffer_release(priv->buffers);

	/* Finally, release the private data */
	kfree(priv);

	return 0;
}

const struct file_operations tegra_pva_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = pva_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pva_ioctl,
#endif
	.open = pva_open,
	.release = pva_release,
};
