/*
 * NVDLA IOCTL for T194
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

#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"

#include "flcn/flcn.h"
#include "flcn/hw_flcn.h"

#include "t194/t194.h"


#include "nvdla/nvdla.h"
#include "nvdla/dla_queue.h"
#include "nvdla/nvdla_buffer.h"
#include "nvdla/nvdla_debug.h"

#include <uapi/linux/nvdev_fence.h>
#include <uapi/linux/nvhost_nvdla_ioctl.h>
#include "dla_os_interface.h"

/**
 * struct nvdla_private per unique FD private data
 * @pdev		pointer to platform device
 * @queue		pointer to nvdla_queue
 * @buffers		pointer to nvdla_buffer
 */

struct nvdla_private {
	struct platform_device *pdev;
	struct nvdla_queue *queue;
	struct nvdla_buffers *buffers;
};

static int nvdla_get_fw_ver(struct nvdla_private *priv,
			struct nvdla_get_fw_ver_args *args)
{
	struct platform_device *pdev = priv->pdev;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	int err = 0;

	nvdla_dbg_fn(pdev, "");

	/* update fw_version if engine is not yet powered on */
	err = nvhost_module_busy(pdev);
	if (err)
		return err;

	args->version = nvdla_dev->fw_version;

	nvdla_dbg_fn(pdev, "version returned[%u]", args->version);

	nvhost_module_idle(pdev);

	return 0;
}

static int nvdla_set_queue(struct nvdla_private *priv, void *args)
{
	struct nvdla_queue_status_args *queue_arg =
			(struct nvdla_queue_status_args *)args;
	struct platform_device *pdev = priv->pdev;
	struct nvdla_queue *queue = priv->queue;
	int status = queue_arg->status;
	int err = 0;

	nvdla_dbg_fn(pdev, "");

	if (!queue) {
		nvdla_dbg_err(pdev, "invalid queue\n");
		err = -EINVAL;
		goto inval_queue;
	}

	/* allow to send only one command at a time */
	if (status != (status & -status)) {
		nvdla_dbg_err(pdev, "incorrect queue cmd set[%d]\n", status);
		err = -EINVAL;
		goto inval_input;
	}

	if ((status & NVDLA_QUEUE_FLAGS_SUSPEND) &&
	     (status & NVDLA_QUEUE_FLAGS_RESUME)) {
		nvdla_dbg_err(pdev, "both queue suspend and resume set %d\n",
				status);
		err = -EINVAL;
		goto inval_input;
	}
	if (status & NVDLA_QUEUE_FLAGS_SUSPEND) {
		err = nvdla_set_queue_state(queue, DLA_CMD_QUEUE_SUSPEND);
	} else if (status & NVDLA_QUEUE_FLAGS_RESUME) {
		err = nvdla_set_queue_state(queue, DLA_CMD_QUEUE_RESUME);
	} else {
		nvdla_dbg_err(pdev, "invalid queue cmd %d\n", status);
		err = -EINVAL;
		goto inval_cmd;
	}
	nvdla_dbg_fn(pdev, "done");

inval_queue:
inval_input:
inval_cmd:
	return err;
}

static int nvdla_get_q_status(struct nvdla_private *priv, void *args)
{
	struct nvdla_get_q_status_args *queue_arg =
			(struct nvdla_get_q_status_args *)args;
	struct nvdev_fence __user *usr_fence =
		(struct nvdev_fence __user *)(uintptr_t)queue_arg->fence;
	struct platform_device *pdev = priv->pdev;
	struct nvdla_queue *queue = priv->queue;
	struct nvdev_fence fence = {0};
	int err = 0;

	nvdla_dbg_fn(pdev, "");

	if (!queue) {
		nvdla_dbg_err(pdev, "invalid queue\n");
		err = -EINVAL;
		goto inval_queue;
	}

	fence.syncpoint_index = queue->syncpt_id;
	fence.syncpoint_value = nvhost_syncpt_read_maxval(pdev,
						queue->syncpt_id);
	nvdla_dbg_info(pdev, "syncpt_id[%u] val[%u]\n", fence.syncpoint_index, fence.syncpoint_value);

	if (copy_to_user(usr_fence, &fence, sizeof(struct nvdev_fence))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to send fence");
		goto fail_to_send_fence;
	}
	queue_arg->id = queue->id;

	nvdla_dbg_fn(pdev, "done");

inval_queue:
fail_to_send_fence:
	return err;
}


static int nvdla_pin(struct nvdla_private *priv, void *arg)
{
	u32 handles[MAX_NVDLA_PIN_BUFFERS];
	struct dma_buf *dmabufs[MAX_NVDLA_PIN_BUFFERS];
	int err = 0;
	int i = 0;
	struct nvdla_pin_unpin_args *buf_list =
			(struct nvdla_pin_unpin_args *)arg;
	u32 count;
	struct platform_device *pdev = priv->pdev;

	nvdla_dbg_fn(pdev, "");

	if (!buf_list) {
		nvdla_dbg_err(pdev, "Invalid argument ptr in pin\n");
		err = -EINVAL;
		goto fail_to_get_val_arg;
	}

	count = buf_list->num_buffers;
	if (count == 0 || count > MAX_NVDLA_PIN_BUFFERS ||
	    !buf_list->buffers) {
		nvdla_dbg_err(pdev, "Inval cnt arg for pin\n");
		err = -EINVAL;
		goto fail_to_get_val_cnt;
	}
	nvdla_dbg_info(pdev, "num of buffers [%d]", count);

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
			(count * sizeof(u32)))) {
		err = -EFAULT;
		goto nvdla_buffer_cpy_err;
	}

	/* get the dmabuf pointer from the fd handle */
	for (i = 0; i < count; i++) {
		dmabufs[i] = dma_buf_get(handles[i]);
		if (IS_ERR_OR_NULL(dmabufs[i])) {
			err = -EFAULT;
			goto fail_to_get_dma_buf;
		}
	}

	err = nvdla_buffer_pin(priv->buffers, dmabufs, count);

fail_to_get_dma_buf:
	count = i;
	for (i = 0; i < count; i++)
		dma_buf_put(dmabufs[i]);
nvdla_buffer_cpy_err:
fail_to_get_val_cnt:
fail_to_get_val_arg:
	return err;
}

static int nvdla_unpin(struct nvdla_private *priv, void *arg)
{
	u32 handles[MAX_NVDLA_PIN_BUFFERS];
	struct dma_buf *dmabufs[MAX_NVDLA_PIN_BUFFERS];
	int err = 0;
	int i = 0;
	struct nvdla_pin_unpin_args *buf_list =
			(struct nvdla_pin_unpin_args *)arg;
	u32 count;
	struct platform_device *pdev = priv->pdev;

	nvdla_dbg_fn(pdev, "");

	if (!buf_list) {
		nvdla_dbg_err(pdev, "Invalid argument for pointer\n");
		err = -EINVAL;
		goto fail_to_get_val_arg;
	}

	count = buf_list->num_buffers;
	if (count == 0 || count > MAX_NVDLA_PIN_BUFFERS ||
	    !buf_list->buffers) {
		nvdla_dbg_err(pdev, "Inval count argument for unpin\n");
		err = -EINVAL;
		goto fail_to_get_val_cnt;
	}
	nvdla_dbg_info(pdev, "num of buffers [%d]", count);

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
		(count * sizeof(u32)))) {
		err = -EFAULT;
		goto nvdla_buffer_cpy_err;
	}

	/* get the dmabuf pointer and clean valid ones */
	for (i = 0; i < count; i++) {
		dmabufs[i] = dma_buf_get(handles[i]);
		if (IS_ERR_OR_NULL(dmabufs[i]))
			continue;
	}

	nvdla_buffer_unpin(priv->buffers, dmabufs, count);

	count = i;
	for (i = 0; i < count; i++)
		dma_buf_put(dmabufs[i]);

nvdla_buffer_cpy_err:
fail_to_get_val_cnt:
fail_to_get_val_arg:
	return err;
}

static int nvdla_ping(struct platform_device *pdev,
			   struct nvdla_ping_args *args)
{
	struct nvdla_cmd_mem_info ping_cmd_mem_info;
	struct nvdla_cmd_data cmd_data;
	u32 *ping_va;
	int err = 0;

	if (!args) {
		nvdla_dbg_err(pdev, "Invalid argument for ping\n");
		err = -EINVAL;
		goto fail_to_get_val_arg;
	}

	/* make sure that device is powered on */
	err = nvhost_module_busy(pdev);
	if (err) {
		nvdla_dbg_err(pdev, "failed to power on\n");
		err = -ENODEV;
		goto fail_to_on;
	}

	/* assign ping cmd buffer */
	err = nvdla_get_cmd_memory(pdev, &ping_cmd_mem_info);
	if (err) {
		nvdla_dbg_err(pdev, "dma memory allocation failed for ping");
		goto fail_to_alloc;
	}
	ping_va = ping_cmd_mem_info.va;

	/* pass ping value to falcon */
	*ping_va = args->in_challenge;

	nvdla_dbg_info(pdev, "ping challenge [%d]", *ping_va);

	/* prepare command data */
	cmd_data.method_id = DLA_CMD_PING;
	cmd_data.method_data = ALIGNED_DMA(ping_cmd_mem_info.pa);
	cmd_data.wait = true;

	/* send ping cmd */
	err = nvdla_send_cmd(pdev, &cmd_data);
	if (err) {
		nvdla_dbg_err(pdev, "failed to send ping command");
		goto fail_cmd;
	}

	/* out value should have (in_challenge * 4) */
	args->out_response = *ping_va;

	nvdla_dbg_info(pdev, "ping response [%d]", *ping_va);

	if (args->out_response != args->in_challenge*4) {
		nvdla_dbg_err(pdev, "ping cmd failed. Falcon is not active");
		err = -EINVAL;
	}

fail_cmd:
	nvdla_put_cmd_memory(pdev, ping_cmd_mem_info.index);
fail_to_alloc:
	nvhost_module_idle(pdev);
fail_to_on:
fail_to_get_val_arg:
	return err;
}

/* task management API's */
static int nvdla_get_actions(struct nvdla_ioctl_submit_task *user_task,
			struct nvdla_task *task)
{
	int err = 0;
	struct platform_device *pdev = task->queue->pool->pdev;

	nvdla_dbg_fn(pdev, "copying actions");

	/* get pre fences */
	if (copy_from_user(task->prefences,
		(void __user *)user_task->prefences,
		(task->num_prefences * sizeof(struct nvdev_fence)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy prefences");
		goto fail;
	}

	/* get input task status */
	if (copy_from_user(task->in_task_status,
		(void __user *)user_task->input_task_status,
		(task->num_in_task_status *
			sizeof(struct nvdla_status_notify)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy input task status");
		goto fail;
	}

	/* get post fences */
	if (copy_from_user(task->postfences,
		(void __user *)user_task->postfences,
		(task->num_postfences * sizeof(struct nvdev_fence)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy postfences");
		goto fail;
	}

	/* get sof task status */
	if (copy_from_user(task->sof_task_status,
		(void __user *)user_task->sof_task_status,
		(task->num_sof_task_status *
			sizeof(struct nvdla_status_notify)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy sof task status");
		goto fail;
	}

	/* get eof task status */
	if (copy_from_user(task->eof_task_status,
		(void __user *)user_task->eof_task_status,
		(task->num_eof_task_status *
			sizeof(struct nvdla_status_notify)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy eof task status");
		goto fail;
	}

	/* get sof timestamps */
	if (copy_from_user(task->sof_timestamps,
		(void __user*)user_task->sof_timestamps,
		(task->num_sof_timestamps * sizeof(struct nvdla_mem_handle)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy sof timestamps");
		goto fail;
	}

	/* get eof timestamps */
	if (copy_from_user(task->eof_timestamps,
		(void __user*)user_task->eof_timestamps,
		(task->num_eof_timestamps * sizeof(struct nvdla_mem_handle)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy eof timestamps");
		goto fail;
	}

	nvdla_dbg_info(pdev, "copying actions done");

fail:
	return err;
}

static int nvdla_send_emu_signal_fences(struct nvdla_emu_task *task,
			struct nvdla_ioctl_emu_submit_task *user_task)
{
	int err = 0, i;
	struct platform_device *dla_pdev = task->queue->pool->pdev;
	struct platform_device *host_pdev =
				to_platform_device(dla_pdev->dev.parent);
	struct nvdev_fence __user *prefences =
		(struct nvdev_fence __user *)(uintptr_t)user_task->prefences;
	struct nvdev_fence __user *postfences =
		(struct nvdev_fence __user *)(uintptr_t)user_task->postfences;
	char fence_name[32];

	nvdla_dbg_fn(dla_pdev, "sending signal fences");

	for (i = 0; i < task->num_prefences; i++) {
		if (task->prefences[i].action != NVDEV_FENCE_SIGNAL)
			continue;

		if (task->prefences[i].type == NVDEV_FENCE_TYPE_SYNC_FD) {
			struct nvhost_ctrl_sync_fence_info info;

			info.id = task->prefences[i].syncpoint_index;
			info.thresh = task->prefences[i].syncpoint_value;

			nvdla_dbg_info(dla_pdev,
					"creating pre sync fd [%d]:[%d]\n",
					info.id, info.thresh);

			/* create fence name format example: nvdla0_1_fence */
			snprintf(fence_name, sizeof(fence_name),
				"%s_%d_%d_prefence", dev_name(&dla_pdev->dev),
				task->prefences[i].syncpoint_index, i);

			err = nvhost_sync_create_fence_fd(host_pdev,
				&info, 1, fence_name,
				&task->prefences[i].sync_fd);

			if (err) {
				nvdla_dbg_err(dla_pdev,
					"fail to create prefence syncfd\n");
				goto fail;
			}
		}
	}

	nvdla_dbg_fn(dla_pdev, "copy prefences to user");
	/* send pre fences */
	if (copy_to_user(prefences, task->prefences,
		(task->num_prefences * sizeof(struct nvdev_fence)))) {
		err = -EFAULT;
		nvdla_dbg_err(dla_pdev, "failed to send prefences");
		goto fail;
	}
	nvdla_dbg_info(dla_pdev, "prefences sent");

	for (i = 0; i < task->num_postfences; i++) {
		if (task->postfences[i].action != NVDEV_FENCE_SIGNAL)
			continue;

		if (task->postfences[i].type == NVDEV_FENCE_TYPE_SYNC_FD) {
			struct nvhost_ctrl_sync_fence_info info;

			info.id = task->postfences[i].syncpoint_index;
			info.thresh = task->postfences[i].syncpoint_value;

			nvdla_dbg_info(dla_pdev,
					"creating post sync fd [%d]:[%d]\n",
					info.id, info.thresh);

			/* create fence name format example: nvdla0_1_fence */
			snprintf(fence_name, sizeof(fence_name),
				"%s_%d_%d_postfence", dev_name(&dla_pdev->dev),
				task->postfences[i].syncpoint_index, i);

			err = nvhost_sync_create_fence_fd(host_pdev,
				&info, 1, fence_name,
				&task->postfences[i].sync_fd);

			if (err) {
				nvdla_dbg_err(dla_pdev,
					"fail to create postfence syncfd\n");
				goto fail;
			}
		}
	}

	nvdla_dbg_fn(dla_pdev, "copy postfences to user");
	/* send post fences */
	if (copy_to_user(postfences, task->postfences,
		(task->num_postfences * sizeof(struct nvdev_fence)))) {
		err = -EFAULT;
		nvdla_dbg_err(dla_pdev, "failed to send postfences");
		goto fail;
	}
	nvdla_dbg_info(dla_pdev, "postfences sent");

fail:
	return err;
}

static int nvdla_update_signal_fences(struct nvdla_task *task,
			struct nvdla_ioctl_submit_task *user_task)
{
	int err = 0, i;
	struct platform_device *dla_pdev = task->queue->pool->pdev;
	struct platform_device *host_pdev =
				to_platform_device(dla_pdev->dev.parent);
	struct nvdev_fence __user *prefences =
		(struct nvdev_fence __user *)(uintptr_t)user_task->prefences;
	struct nvdev_fence __user *postfences =
		(struct nvdev_fence __user *)(uintptr_t)user_task->postfences;
	char fence_name[32];

	nvdla_dbg_fn(dla_pdev, "copy fences for user");

	/* update pre fence signals to users */
	for (i = 0; i < task->num_prefences; i++) {
		if (task->prefences[i].action != NVDEV_FENCE_SIGNAL)
			continue;

		if (task->prefences[i].type == NVDEV_FENCE_TYPE_SYNC_FD) {
			struct nvhost_ctrl_sync_fence_info info;

			info.id = task->prefences[i].syncpoint_index;
			info.thresh = task->prefences[i].syncpoint_value;

			nvdla_dbg_info(dla_pdev,
					"creating pre sync fd [%d]:[%d]\n",
					info.id, info.thresh);

			/* create fence name format example: nvdla0_1_fence */
			snprintf(fence_name, sizeof(fence_name),
				"%s_%d_%d_prefence", dev_name(&dla_pdev->dev),
				task->prefences[i].syncpoint_index, i);

			err = nvhost_sync_create_fence_fd(host_pdev,
				&info, 1, fence_name,
				&task->prefences[i].sync_fd);

			if (err) {
				nvdla_dbg_err(dla_pdev,
					"fail to create prefence syncfd\n");
				goto fail;
			}
		}
	}

	nvdla_dbg_fn(dla_pdev, "copy prefences to user");
	/* copy pre fences */
	if (copy_to_user(prefences, task->prefences,
				(task->num_prefences * sizeof(struct nvdev_fence)))) {
		err = -EFAULT;
		nvdla_dbg_err(dla_pdev, "failed to copy prefences");
		goto fail;
	}

	/* update post fence signals to user */
	for (i = 0; i < task->num_postfences; i++) {
		if (task->postfences[i].action != NVDEV_FENCE_SIGNAL)
			continue;

		if (task->postfences[i].type == NVDEV_FENCE_TYPE_SYNC_FD) {
			struct nvhost_ctrl_sync_fence_info info;

			info.id = task->postfences[i].syncpoint_index;
			info.thresh = task->postfences[i].syncpoint_value;

			nvdla_dbg_info(dla_pdev,
					"creating post sync fd [%d]:[%d]\n",
					info.id, info.thresh);

			/* create fence name format example: nvdla0_1_fence */
			snprintf(fence_name, sizeof(fence_name),
				"%s_%d_%d_postfence", dev_name(&dla_pdev->dev),
				task->postfences[i].syncpoint_index, i);

			err = nvhost_sync_create_fence_fd(host_pdev,
				&info, 1, fence_name,
				&task->postfences[i].sync_fd);

			if (err) {
				nvdla_dbg_err(dla_pdev,
					"fail to create postfence syncfd\n");
				goto fail;
			}
		}
	}

	nvdla_dbg_fn(dla_pdev, "copy postfences to user");
	/* copy post fences */
	if (copy_to_user(postfences, task->postfences,
		(task->num_postfences * sizeof(struct nvdev_fence)))) {
		err = -EFAULT;
		nvdla_dbg_err(dla_pdev, "failed to copy postfences");
		goto fail;
	}
	nvdla_dbg_info(dla_pdev, "postfences copied for user");

fail:
	return err;
}

size_t nvdla_get_max_task_size(void)
{
	return sizeof(struct nvdla_task);
}

static int nvdla_val_task_submit_input(struct nvdla_ioctl_submit_task *in_task)
{
	if (in_task->num_prefences > MAX_NUM_NVDLA_PREFENCES) {
		pr_err("num_prefences[%u] crossing expected[%d]\n",
			in_task->num_prefences, MAX_NUM_NVDLA_PREFENCES);
		return -EINVAL;
	}
	if (in_task->num_postfences > MAX_NUM_NVDLA_POSTFENCES) {
		pr_err("num_postfences[%u] crossing expected[%d]\n",
			in_task->num_postfences, MAX_NUM_NVDLA_POSTFENCES);
		return -EINVAL;
	}
	if (in_task->num_input_task_status > MAX_NUM_NVDLA_IN_TASK_STATUS) {
		pr_err("in task status[%u] crossing expected[%d]\n",
			in_task->num_input_task_status,
			MAX_NUM_NVDLA_IN_TASK_STATUS);
		return -EINVAL;
	}
	if (in_task->num_sof_task_status > MAX_NUM_NVDLA_OUT_TASK_STATUS) {
		pr_err("sof task status[%u] crossing expected[%d]\n",
			in_task->num_sof_task_status,
			MAX_NUM_NVDLA_OUT_TASK_STATUS);
		return -EINVAL;
	}
	if (in_task->num_eof_task_status > MAX_NUM_NVDLA_OUT_TASK_STATUS) {
		pr_err("eof task status[%u] crossing expected[%d]\n",
			in_task->num_eof_task_status,
			MAX_NUM_NVDLA_OUT_TASK_STATUS);
		return -EINVAL;
	}
	if (in_task->num_sof_timestamps > MAX_NUM_NVDLA_OUT_TIMESTAMP) {
		pr_err("sof timestamps[%u] crossing expected[%d]\n",
			in_task->num_sof_timestamps,
			MAX_NUM_NVDLA_OUT_TIMESTAMP);
		return -EINVAL;
	}
	if (in_task->num_eof_timestamps > MAX_NUM_NVDLA_OUT_TIMESTAMP) {
		pr_err("eof timestamps[%u] crossing expected[%d]\n",
			in_task->num_eof_timestamps,
			MAX_NUM_NVDLA_OUT_TIMESTAMP);
		return -EINVAL;
	}
	if (in_task->num_addresses < 1) {
		pr_err("num addresses[%u] should be min one\n",
				in_task->num_addresses);
		return -EINVAL;
	}
	if (in_task->num_addresses > NVDLA_MAX_BUFFERS_PER_TASK) {
		pr_err("num addresses[%u] crossing expected[%d]\n",
			in_task->num_addresses,
			NVDLA_MAX_BUFFERS_PER_TASK);
		return -EINVAL;
	}

	return 0;
}

static int nvdla_fill_task(struct nvdla_queue *queue,
				struct nvdla_buffers *buffers,
				struct nvdla_ioctl_submit_task *local_task,
				struct nvdla_task *task)
{
	void *mem;
	int err = 0;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_fn(pdev, "");

	 /* initialize task parameters */
	kref_init(&task->ref);
	task->queue = queue;
	task->buffers = buffers;
	task->sp = &nvhost_get_host(pdev)->syncpt;

	err = nvdla_val_task_submit_input(local_task);
	if (err) {
		nvdla_dbg_err(pdev, "Invalid input arguments");
		goto fail_to_get_val_args;
	}

	task->num_prefences = local_task->num_prefences;
	task->num_postfences = local_task->num_postfences;
	task->num_in_task_status = local_task->num_input_task_status;
	task->num_sof_task_status = local_task->num_sof_task_status;
	task->num_eof_task_status = local_task->num_eof_task_status;
	task->num_sof_timestamps = local_task->num_sof_timestamps;
	task->num_eof_timestamps = local_task->num_eof_timestamps;
	task->num_addresses = local_task->num_addresses;
	task->timeout = local_task->timeout;

	/* assign memory for local task action lists and buf handles */
	mem = task;
	mem += sizeof(struct nvdla_task);

	/* update local fences into task */
	err = nvdla_get_actions(local_task, task);
	if (err) {
		nvdla_dbg_err(pdev, "failed to get actions");
		goto fail_to_get_actions;
	}

	/* get user addresses list */
	if (copy_from_user(task->memory_handles,
		(void __user *)local_task->address_list,
		(task->num_addresses *
			sizeof(struct nvdla_mem_handle)))) {
		err = -EFAULT;
		nvdla_dbg_err(pdev, "failed to copy address list");
		goto fail_to_get_addr_list;
	}

	nvdla_dbg_info(pdev, "local task %p param filled with args", task);

	return 0;

fail_to_get_addr_list:
fail_to_get_actions:
fail_to_get_val_args:
	return err;
}

static void nvdla_dump_task(struct nvdla_task *task)
{
	int i;
	struct nvdla_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_info(pdev, "dumping input task [%p] parameters:", task);
	nvdla_dbg_info(pdev, "num_prefences[%u] num_postfences[%u]",
			task->num_prefences, task->num_postfences);
	nvdla_dbg_info(pdev, "num_in_status[%u] num_sof_task_status[%u] "
			"num_eof_task_status[%u]",
			task->num_in_task_status,
			task->num_sof_task_status,
			task->num_eof_task_status);
	nvdla_dbg_info(pdev, "num_sof_timestamps[%u] num_eof_timestamps[%u]",
			task->num_sof_timestamps,
			task->num_eof_timestamps);
	nvdla_dbg_info(pdev, "num_addresses[%u]", task->num_addresses);

	for (i = 0; i < task->num_prefences; i++) {
		nvdla_dbg_info(pdev, "prefence[%d]: type[%u] syncpt_index[%u]"
				" syncpt_val[%u] sync_fd[%u] semaphore_handle[%u]"
				" semaphore_offset[%u] semaphore_value[%u]",
				i, task->prefences[i].type,
				task->prefences[i].syncpoint_index,
				task->prefences[i].syncpoint_value,
				task->prefences[i].sync_fd,
				task->prefences[i].semaphore_handle,
				task->prefences[i].semaphore_offset,
				task->prefences[i].semaphore_value);
	}

	for (i = 0; i < task->num_postfences; i++) {
		nvdla_dbg_info(pdev, "postfence[%d]: type[%u] syncpt_index[%u]"
				" syncpt_val[%u] sync_fd[%u] semaphore_handle[%u]"
				" semaphore_offset[%u] semaphore_value[%u]",
				i, task->postfences[i].type,
				task->postfences[i].syncpoint_index,
				task->postfences[i].syncpoint_value,
				task->postfences[i].sync_fd,
				task->postfences[i].semaphore_handle,
				task->postfences[i].semaphore_offset,
				task->postfences[i].semaphore_value);
	}

	for (i = 0; i < task->num_in_task_status; i++) {
		nvdla_dbg_info(pdev, "Input task status[%d]:"
				"handle[%u] offset[%u] status[%u]",
				i, task->in_task_status[i].handle,
				task->in_task_status[i].offset,
				task->in_task_status[i].status);
	}

	for (i = 0; i < task->num_sof_task_status; i++) {
		nvdla_dbg_info(pdev, "SOF task status[%d]:"
				"handle[%u] offset[%u] status[%u]",
				i, task->sof_task_status[i].handle,
				task->sof_task_status[i].offset,
				task->sof_task_status[i].status);
	}

	for (i = 0; i < task->num_eof_task_status; i++) {
		nvdla_dbg_info(pdev, "EOF task status[%d]:"
				"handle[%u] offset[%u] status[%u]",
				i, task->eof_task_status[i].handle,
				task->eof_task_status[i].offset,
				task->eof_task_status[i].status);
	}

	for (i = 0; i < task->num_sof_timestamps; i++) {
		nvdla_dbg_info(pdev, "SOF timestamp[%d]:"
				"handle[%u] offset[%u]",
				i, task->sof_timestamps[i].handle,
				task->sof_timestamps[i].offset);
	}

	for (i = 0; i < task->num_eof_timestamps; i++) {
		nvdla_dbg_info(pdev, "EOF timestamp[%d]:"
				"handle[%u] offset[%u]",
				i, task->eof_timestamps[i].handle,
				task->eof_timestamps[i].offset);
	}

	for (i = 0; i < task->num_addresses; i++) {
		nvdla_dbg_info(pdev, "Memory Handles[%d]:"
				"handle[%u] offset[%u]",
				i, task->memory_handles[i].handle,
				task->memory_handles[i].offset);
	}
}

static int nvdla_emu_task_submit(struct nvdla_private *priv, void *arg)
{
	struct nvdla_submit_args *args =
			(struct nvdla_submit_args *)arg;
	struct nvdla_ioctl_emu_submit_task __user *user_tasks;
	struct nvdla_ioctl_emu_submit_task local_tasks[MAX_TASKS_PER_SUBMIT];
	struct platform_device *pdev;
	struct nvdla_queue *queue;
	struct nvdla_emu_task task;
	int err = 0, i = 0;
	u32 num_tasks;

	if (!args || !priv)
		return -EINVAL;

	pdev = priv->pdev;
	queue = priv->queue;
	if (!(queue && pdev))
		return -EINVAL;

	nvdla_dbg_fn(pdev, "inside emulator task submit");

	task.queue = queue;
	task.sp = &nvhost_get_host(pdev)->syncpt;

	user_tasks = (struct nvdla_ioctl_emu_submit_task __user *)
			(uintptr_t)args->tasks;
	if (!user_tasks)
		return -EINVAL;

	num_tasks = args->num_tasks;
	if (num_tasks == 0 || num_tasks > MAX_TASKS_PER_SUBMIT)
		return -EINVAL;

	nvdla_dbg_info(pdev, "num of emulator tasks [%d]", num_tasks);

	/* IOCTL copy descriptors*/
	if (copy_from_user(local_tasks, (void __user *)user_tasks,
			(num_tasks * sizeof(*user_tasks)))) {
		err = -EFAULT;
		goto exit;
	}
	nvdla_dbg_info(pdev, "copy of user tasks done");

	for (i = 0; i < num_tasks; i++) {

		nvdla_dbg_info(pdev, "submit [%d]th task", i + 1);

		task.num_prefences = local_tasks[i].num_prefences;
		task.num_postfences = local_tasks[i].num_postfences;

		/* get pre fences */
		if (copy_from_user(task.prefences,
			(void __user *)local_tasks[i].prefences,
			(task.num_prefences * sizeof(struct nvdev_fence)))) {
			err = -EFAULT;
			nvdla_dbg_err(pdev, "failed to copy prefences");
			goto exit;
		}

		/* get post fences */
		if (copy_from_user(task.postfences,
			(void __user *)local_tasks[i].postfences,
			(task.num_postfences * sizeof(struct nvdev_fence)))) {
			err = -EFAULT;
			nvdla_dbg_err(pdev, "failed to copy postfences");
			goto exit;
		}

		err = nvdla_emulator_submit(queue, &task);
		if (err) {
			nvdla_dbg_err(pdev, "fail to submit task: %d", i + 1);
			goto exit;
		}
		nvdla_dbg_info(pdev, "task[%d] submitted", i + 1);

		/* send signal fences to user */
		err = nvdla_send_emu_signal_fences(&task, local_tasks + i);
		if (err) {
			nvdla_dbg_err(pdev, "fail to send sig fence%d", i + 1);
			goto exit;
		}
		nvdla_dbg_info(pdev, "signal fences of task[%d] sent", i + 1);
	}
	nvdla_dbg_fn(pdev, "Emulator task submitted, done!");

exit:
	return 0;
}

static int nvdla_submit(struct nvdla_private *priv, void *arg)
{
	struct nvdla_submit_args *args =
			(struct nvdla_submit_args *)arg;
	struct nvdla_ioctl_submit_task __user *user_tasks;
	struct nvdla_ioctl_submit_task local_tasks[MAX_TASKS_PER_SUBMIT];
	struct platform_device *pdev;
	struct nvdla_queue *queue;
	struct nvdla_buffers *buffers;
	u32 num_tasks;
	struct nvdla_task *task;
	int err = 0, i = 0;

	if (!args || !priv)
		return -EINVAL;

	pdev = priv->pdev;
	queue = priv->queue;
	buffers = priv->buffers;
	if (!(queue && pdev && buffers))
		return -EINVAL;

	nvdla_dbg_fn(pdev, "inside task submit");

	user_tasks = (struct nvdla_ioctl_submit_task __user *)
			(uintptr_t)args->tasks;
	if (!user_tasks)
		return -EINVAL;

	num_tasks = args->num_tasks;
	if (num_tasks == 0 || num_tasks > MAX_TASKS_PER_SUBMIT)
		return -EINVAL;

	nvdla_dbg_info(pdev, "num of tasks [%d]", num_tasks);

	/* IOCTL copy descriptors*/
	if (copy_from_user(local_tasks, (void __user *)user_tasks,
			(num_tasks * sizeof(*user_tasks)))) {
		err = -EFAULT;
		goto fail_to_copy_task;
	}
	nvdla_dbg_info(pdev, "copy of user tasks done");

	for (i = 0; i < num_tasks; i++) {

		nvdla_dbg_info(pdev, "submit [%d]th task", i + 1);

		err = nvdla_get_task_mem(queue, &task);
		if (err) {
			nvdla_dbg_err(pdev, "failed to get task[%d] mem", i + 1);
			goto fail_to_get_task_mem;
		}
		nvdla_dbg_info(pdev, "task[%d] mem allocate done", i + 1);

		/* fill local task param from user args */
		err = nvdla_fill_task(queue, buffers, local_tasks + i, task);
		if (err) {
			nvdla_dbg_err(pdev, "failed to fill task[%d]", i + 1);
			kref_put(&task->ref, task_free);
			goto fail_to_fill_task;
		}
		nvdla_dbg_info(pdev, "local task[%d] filled", i + 1);

		/* dump task input parameters */
		nvdla_dump_task(task);
		nvdla_dbg_info(pdev, "dump task[%d] done", i + 1);

		/* update task desc fields */
		err = nvdla_fill_task_desc(task);
		if (err) {
			nvdla_dbg_err(pdev, "fail to fill task desc%d", i + 1);
			goto fail_to_fill_task_desc;
		}
		nvdla_dbg_info(pdev, "task[%d] desc filled", i + 1);

		/* get expected signal fences prior to submit */
		err = nvdla_get_signal_fences(queue, task);
		if (err) {
			nvdla_dbg_err(pdev, "fail to get fences%d", i + 1);
			goto fail_to_get_fences;
		}
		nvdla_dbg_info(pdev, "task[%d] got fences", i + 1);

		/* update fences to user */
		err = nvdla_update_signal_fences(task, local_tasks + i);
		if (err) {
			nvdla_dbg_err(pdev, "fail update postfence%d", i + 1);
			goto fail_to_update_postfences;
		}
		nvdla_dbg_info(pdev, "postfences of task[%d] update", i + 1);

		/* send job to engine through queue framework */
		err = nvdla_queue_submit(queue, task);
		if (err) {
			nvdla_dbg_err(pdev, "fail to submit task: %d", i + 1);
			goto fail_to_submit_task;
		}
		nvdla_dbg_info(pdev, "task[%d] submitted", i + 1);
		kref_put(&task->ref, task_free);
	}
	nvdla_dbg_fn(pdev, "Task submitted, done!");

	return 0;

fail_to_submit_task:
fail_to_update_postfences:
fail_to_get_fences:
fail_to_fill_task_desc:
fail_to_fill_task:
	/*TODO: traverse list in reverse and delete jobs */
fail_to_get_task_mem:
fail_to_copy_task:
	return err;
}

static long nvdla_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct nvdla_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;
	u8 buf[NVDLA_IOCTL_MAX_ARG_SIZE] __aligned(sizeof(u64));
	int err = 0;

	/* check for valid IOCTL cmd */
	if ((_IOC_TYPE(cmd) != NVHOST_NVDLA_IOCTL_MAGIC) ||
	    (_IOC_NR(cmd) == _IOC_NR(0)) ||
	    (_IOC_NR(cmd) > NVDLA_IOCTL_LAST) ||
	    (_IOC_SIZE(cmd) > NVDLA_IOCTL_MAX_ARG_SIZE)) {
		return -ENOIOCTLCMD;
	}

	/* copy from user for read commands */
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (!arg)
			return -EINVAL;
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	nvdla_dbg_fn(pdev, "priv:%p cmd:%u", priv, cmd);

	/* handle IOCTL cmd */
	switch (cmd) {
	case NVDLA_IOCTL_PING:
		err = nvdla_ping(pdev, (void *)buf);
		break;
	case NVDLA_IOCTL_PIN:
		err = nvdla_pin(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_UNPIN:
		err = nvdla_unpin(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_SUBMIT:
		err = nvdla_submit(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_SET_QUEUE_STATUS:
		err = nvdla_set_queue(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_GET_FIRMWARE_VERSION:
		err = nvdla_get_fw_ver(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_GET_QUEUE_STATUS:
		err = nvdla_get_q_status(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_EMU_TASK_SUBMIT:
		err = nvdla_emu_task_submit(priv, (void *)buf);
		break;
	default:
		nvdla_dbg_err(pdev, "invalid IOCTL CMD");
		err = -ENOIOCTLCMD;
		break;
	}

	/* copy to user for write commands */
	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static int nvdla_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct nvdla_private *priv;
	int err = 0, index;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL)) {
		err = -ENOMEM;
		goto err_alloc_priv;
	}

	file->private_data = priv;
	priv->pdev = pdev;

	nvdla_dbg_fn(pdev, "priv:%p", priv);

	/* add priv to client list */
	err = nvhost_module_add_client(pdev, priv);
	if (err < 0)
		goto err_add_client;

	/* set rate for EMC to max
         * on device release ACM sets to default rate
         */
	for (index = 0; index < NVHOST_MODULE_MAX_CLOCKS; index++) {
		struct nvhost_clock *clock = &pdata->clocks[index];

		if (clock->moduleid ==
			NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER) {
			err = nvhost_module_set_rate(pdev, priv, UINT_MAX,
				index, clock->bwmgr_request_type);
			if (err < 0)
				goto err_alloc_queue;
			break;
		}
	}

	priv->queue = nvdla_queue_alloc(nvdla_dev->pool,
		MAX_NVDLA_TASK_COUNT,
		nvdla_dev->submit_mode == NVDLA_SUBMIT_MODE_CHANNEL);
	if (IS_ERR(priv->queue)) {
		err = PTR_ERR(priv->queue);
		goto err_alloc_queue;
	}

	priv->buffers = nvdla_buffer_init(priv->queue->vm_pdev);
	if (IS_ERR(priv->buffers)) {
		err = PTR_ERR(priv->buffers);
		goto err_alloc_buffer;
	}

	return nonseekable_open(inode, file);

err_alloc_buffer:
	kfree(priv->buffers);
err_alloc_queue:
	nvhost_module_remove_client(pdev, priv);
err_add_client:
	kfree(priv);
err_alloc_priv:
	return err;
}

static int nvdla_release(struct inode *inode, struct file *file)
{
	struct nvdla_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;

	nvdla_dbg_fn(pdev, "priv:%p", priv);

	nvdla_queue_abort(priv->queue);
	nvdla_queue_put(priv->queue);
	nvdla_buffer_release(priv->buffers);
	nvhost_module_remove_client(pdev, priv);

	kfree(priv);
	return 0;
}

const struct file_operations tegra_nvdla_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvdla_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvdla_ioctl,
#endif
	.open = nvdla_open,
	.release = nvdla_release,
};
