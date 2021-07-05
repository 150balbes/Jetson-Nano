/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/slab.h>   /* kmalloc() */
#include <linux/errno.h> /* error codes */
#include <linux/delay.h> /* For msleep and usleep_range */
#include <linux/version.h>
#include <uapi/scsi/ufs/ioctl.h>
#include "tegra_vblk.h"
#include "tegra_hv_ufs.h"


static int vblk_validate_single_query_io(struct vblk_dev *vblkdev,
					struct ufs_ioc_query_req *query_req,
					size_t *data_len,
					bool *w_flag)
{
	int err = 0;

	switch (query_req->opcode) {
	case UPIU_QUERY_OPCODE_READ_DESC:
		if (query_req->idn >= QUERY_DESC_IDN_MAX) {
			dev_err(vblkdev->device,
					"Desc IDN out of range %d\n",
					query_req->idn);
			err = -EINVAL;
			goto out;
		}

		*data_len = min_t(size_t, QUERY_DESC_MAX_SIZE,
						query_req->buf_size);
		break;

	case UPIU_QUERY_OPCODE_WRITE_DESC:
		if (query_req->idn >= QUERY_DESC_IDN_MAX) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"Desc IDN out of range %d\n",
					query_req->idn);
			goto out;
		}

		*data_len = min_t(size_t, QUERY_DESC_MAX_SIZE,
					query_req->buf_size);
		*w_flag = true;
		break;

	case UPIU_QUERY_OPCODE_READ_ATTR:
		if (query_req->idn >= QUERY_ATTR_IDN_MAX) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"ATTR IDN out of range %d\n",
					query_req->idn);
			goto out;
		}

		if (query_req->buf_size != sizeof(u32)) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"Buf size out of range %d\n",
					query_req->buf_size);
			goto out;
		}
		*data_len = sizeof(u32);
		break;

	case UPIU_QUERY_OPCODE_WRITE_ATTR:
		if (query_req->idn > QUERY_ATTR_IDN_MAX) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"ATTR IDN out of range %d\n",
					query_req->idn);
			goto out;
		}

		if (query_req->buf_size != sizeof(u32)) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"Buf size out of range %d\n",
					query_req->buf_size);
			goto out;
		}
		*data_len = sizeof(u32);
		*w_flag = true;
		break;

	case UPIU_QUERY_OPCODE_READ_FLAG:
		if (query_req->idn > QUERY_FLAG_IDN_MAX) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"Flag IDN out of range %d\n",
					query_req->idn);
			goto out;
		}

		if (query_req->buf_size != sizeof(u8)) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"Buf size out of range %d\n",
					query_req->buf_size);
			goto out;
		}
		*data_len = sizeof(u8);
		break;

	case UPIU_QUERY_OPCODE_SET_FLAG:
	case UPIU_QUERY_OPCODE_CLEAR_FLAG:
	case UPIU_QUERY_OPCODE_TOGGLE_FLAG:
		if (query_req->idn > QUERY_FLAG_IDN_MAX) {
			err = -EINVAL;
			dev_err(vblkdev->device,
					"Flag IDN out of range %d\n",
					query_req->idn);
			goto out;
		}
			/* TODO: Create buffer to be attached */
		*data_len = 0;
		break;
	default:
		err = -EINVAL;
		dev_err(vblkdev->device, "Invalid opcode %d \n",
						query_req->idn);
		break;
	}
out:
	return err;
}

static int vblk_prep_single_query_io(struct vblk_dev *vblkdev,
		struct ufs_ioc_query_req *query_req,
		struct vblk_ioctl_req *ioctl_req)
{
	int err = 0;
	size_t data_len = 0;
	void *ioctl_buf;
	struct vblk_ufs_ioc_query_req *vblk_ioctl_query_req;
	size_t ioctl_len, query_req_buf_offset;
	bool w_flag = false;

	err = vblk_validate_single_query_io(vblkdev, query_req,
				&data_len, &w_flag);
	if (err) {
		dev_err(vblkdev->device, "Validating request failed\n");
		goto out;
	}

	ioctl_len = sizeof(*vblk_ioctl_query_req) + data_len;

	ioctl_buf = kzalloc(ioctl_len, GFP_KERNEL);
	if (ioctl_buf == NULL) {
		err = -ENOMEM;
		dev_err(vblkdev->device, "failed to alloc memory for ioctl buf!\n");
		goto out;
	}

	vblk_ioctl_query_req = ioctl_buf;

	vblk_ioctl_query_req->opcode = query_req->opcode;
	vblk_ioctl_query_req->idn = query_req->idn;
	vblk_ioctl_query_req->index = query_req->index;
	vblk_ioctl_query_req->selector = query_req->selector;
	vblk_ioctl_query_req->buf_size = query_req->buf_size;
	vblk_ioctl_query_req->delay = query_req->delay;
	vblk_ioctl_query_req->error_status = query_req->error_status;

	query_req_buf_offset = sizeof(*vblk_ioctl_query_req);
	vblk_ioctl_query_req->buffer = ((u8 *)vblk_ioctl_query_req) +
						query_req_buf_offset;
	if (data_len && w_flag) {
		err = copy_from_user(vblk_ioctl_query_req->buffer,
				query_req->buffer,
				data_len);
		if (err) {
			kfree(ioctl_buf);
			goto out;
		}

	}

	ioctl_req->ioctl_id = VBLK_UFS_IO_ID;
	ioctl_req->ioctl_buf = ioctl_buf;
	ioctl_req->ioctl_len = ioctl_len;

out:
	return err;
}

static int vblk_complete_single_query_io(struct vblk_dev *vblkdev,
				struct vblk_ufs_ioc_query_req *vblk_req,
				struct ufs_ioc_query_req *req)
{
	u8 *q_req_buf;
	int err;

	q_req_buf = (u8 *)vblk_req + sizeof(*vblk_req);

	err = copy_to_user(req->buffer, q_req_buf, vblk_req->buf_size);
	if (err) {
		dev_err(vblkdev->device, "Failed copy_to_user query_req buffer\n");
		goto out;
	}

	err = copy_to_user(&req->buf_size, &vblk_req->buf_size,
				sizeof(vblk_req->buf_size));
	if (err) {
		dev_err(vblkdev->device, "Failed copy_to_user query_req buf_size\n");
		goto out;
	}

	err = copy_to_user(&req->error_status, &vblk_req->error_status,
			sizeof(vblk_req->error_status));
	if (err) {
		dev_err(vblkdev->device, "Failed copy_to_user query_req status\n");
		goto out;
	}
out:
	return err;
}

int vblk_submit_combo_query_io(struct vblk_dev *vblkdev,
		unsigned int cmd, void __user *user)
{
	struct vblk_ioctl_req *ioctl_req;
	struct ufs_ioc_combo_query_req *combo_query_req, *usr_combo_query_req;
	struct ufs_ioc_query_req *query_req;
	struct vblk_ufs_ioc_query_req *vblk_ioctl_query_req;
	int err, i;
	struct request *rq;
	u32 delay;

	usr_combo_query_req = user;

	ioctl_req = kzalloc(sizeof(*ioctl_req), GFP_KERNEL);
	if (!ioctl_req)
		return -ENOMEM;

	combo_query_req = kzalloc(sizeof(*combo_query_req), GFP_KERNEL);
	if (combo_query_req == NULL) {
		err = -ENOMEM;
		dev_err(vblkdev->device,
				"failed to alloc memory for combo_query!\n");
		goto free_ioctl_req;
	}

	/* Copy the combo query user buffer to kernel buffer */
	err = copy_from_user(combo_query_req, user, sizeof(*combo_query_req));
	if (err) {
		err = -ENOMEM;
		dev_err(vblkdev->device,
				"Copy from user failed for combo_query!\n");
		goto free_combo_query_req;
	}

	/* Validatea combo_query_request */
	if ((combo_query_req->num_cmds == 0) ||
		(combo_query_req->query == NULL) ||
		(combo_query_req->num_cmds > MAX_QUERY_CMD_PER_COMBO)) {
		err = -EINVAL;
		dev_err(vblkdev->device, "Invalid Params for combo_query!\n");
		goto free_combo_query_req;
	}

	query_req = kzalloc(sizeof(struct ufs_ioc_query_req) *
				combo_query_req->num_cmds, GFP_KERNEL);
	if (query_req == NULL) {
		err = -ENOMEM;
		dev_err(vblkdev->device, "Failed to allocate memory for query_req!\n");
		goto free_combo_query_req;
	}

	/* Copy the query user buffer to kernel buffer */
	err = copy_from_user(query_req, combo_query_req->query,
		sizeof(struct ufs_ioc_query_req) * combo_query_req->num_cmds);
	if (err) {
		err = -ENOMEM;
		dev_err(vblkdev->device, "Copy from user failed for query_req!\n");
		goto free_query_req;
	}

	/* Validate each query in combo query */
	for (i = 0; i < combo_query_req->num_cmds; i++) {
		if ((query_req[i].buf_size != 0) &&
				(query_req[i].buffer == NULL)) {
			err = -EINVAL;
			dev_err(vblkdev->device,
				"Invalid Params for query_req %d!\n", i);
			goto free_query_req;
		}
	}

	for (i = 0; i < combo_query_req->num_cmds; i++) {
		err = vblk_prep_single_query_io(vblkdev, &query_req[i],
							ioctl_req);
		query_req[i].error_status = err;
		if (err) {
			err = copy_to_user(
				&usr_combo_query_req->query[i].error_status,
				&query_req[i].error_status,
				sizeof(query_req[i].error_status));
			kfree(ioctl_req->ioctl_buf);
			memset(ioctl_req, 0x0, sizeof(*ioctl_req));
			/*  Return if user wants to terminate on error */
			if (combo_query_req->return_on_error)
				break;
			/* Continue to next request */
			else
				continue;

		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		rq = blk_get_request(vblkdev->queue, REQ_OP_DRV_IN, GFP_KERNEL);
#else
		rq = blk_get_request(vblkdev->queue, READ, GFP_KERNEL);
#endif
		if (IS_ERR_OR_NULL(rq)) {
			dev_err(vblkdev->device,
					"Failed to get handle to a request!\n");
			err = PTR_ERR(rq);
			goto free_query_req;
		}

		rq->special = ioctl_req;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		blk_execute_rq(vblkdev->queue, vblkdev->gd, rq, 0);
		blk_put_request(rq);
#else
		rq->cmd_type = REQ_TYPE_DRV_PRIV;
		err = blk_execute_rq(vblkdev->queue, vblkdev->gd, rq, 0);
		blk_put_request(rq);
#endif
		/* add delay if specified */
		delay = query_req[i].delay;
		if (delay) {
			if (delay > 10000)
				msleep(delay / 1000);
			else
				usleep_range(delay, delay + 10);
		}

		vblk_ioctl_query_req = ioctl_req->ioctl_buf;
		err = vblk_complete_single_query_io(vblkdev,
						vblk_ioctl_query_req,
						&usr_combo_query_req->query[i]);

		kfree(ioctl_req->ioctl_buf);
		memzero_explicit(ioctl_req, sizeof(*ioctl_req));
	}

free_query_req:
	kfree(query_req);
free_combo_query_req:
	kfree(combo_query_req);
free_ioctl_req:
	kfree(ioctl_req);
	return err;
}
