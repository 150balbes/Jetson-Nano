/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h> /* printk() */
#include <linux/slab.h>   /* kmalloc() */
#include <linux/fs.h>   /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/fcntl.h> /* O_ACCMODE */
#include <asm/uaccess.h>
#include <asm-generic/bug.h>
#include <linux/mmc/ioctl.h>
#include <linux/mmc/core.h>
#include "tegra_vblk.h"

#define VBLK_MMC_MAX_IOC_SIZE (256 * 1024)

static uint32_t vblk_get_response_type(uint32_t mmc_resp_type)
{
	uint32_t vblk_resp;
	/* consider only respose types */
	switch (mmc_resp_type & 0x1F) {
	case MMC_RSP_NONE:
		vblk_resp = RESP_TYPE_NO_RESP;
		break;
	case MMC_RSP_R1:
		/*
		 * MMC_RSP_R1, MMC_RSP_R6, MMC_RSP_R6, MMC_RSP_R7 have same
		 * values
		 */
		vblk_resp = RESP_TYPE_R1;
		break;
	case MMC_RSP_R1B:
		vblk_resp = RESP_TYPE_R1B;
		break;
	case MMC_RSP_R2:
		vblk_resp = RESP_TYPE_R2;
		break;
	case MMC_RSP_R3:
		/* MMC_RSP_R3 and MMC_RSP_R4 have same value */
		vblk_resp = RESP_TYPE_R3;
		break;
	default:
		vblk_resp = RESP_TYPE_NUM;
	}
	return vblk_resp;
}

int vblk_prep_mmc_multi_ioc(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user,
		uint32_t cmd)
{
	int err = 0;
	struct combo_info_t *combo_info;
	struct combo_cmd_t *combo_cmd;
	int i = 0;
	uint64_t num_cmd;
	struct mmc_ioc_cmd ic;
	struct mmc_ioc_multi_cmd __user *user_cmd;
	struct mmc_ioc_cmd __user *usr_ptr;
	uint32_t combo_cmd_size;
	uint32_t ioctl_bytes = VBLK_MMC_MAX_IOC_SIZE;
	uint8_t *tmpaddr;
	void *ioctl_buf;

	ioctl_buf = vmalloc(ioctl_bytes);
	if (ioctl_buf == NULL) {
		return -ENOMEM;
	}

	combo_info = (struct combo_info_t *)ioctl_buf;
	combo_cmd_size = sizeof(uint32_t);

	if (cmd == MMC_IOC_MULTI_CMD) {
		user_cmd = (struct mmc_ioc_multi_cmd __user *)user;
		if (copy_from_user(&num_cmd, &user_cmd->num_of_cmds,
				sizeof(num_cmd))) {
			err = -EFAULT;
			goto free_ioc_buf;
		}

		if (num_cmd > MMC_IOC_MAX_CMDS) {
			err = -EINVAL;
			goto free_ioc_buf;
		}

		usr_ptr = (void * __user)&user_cmd->cmds;
	} else {
		num_cmd = 1;
		usr_ptr = (void * __user)user;
	}
	combo_info->count = num_cmd;

	combo_cmd = (struct combo_cmd_t *)(ioctl_buf +
		sizeof(struct combo_info_t));

	combo_cmd_size = sizeof(struct combo_info_t) +
		sizeof(struct combo_cmd_t) * combo_info->count;
	if (combo_cmd_size < sizeof(struct combo_info_t)) {
		dev_err(vblkdev->device,
			"combo_cmd_size is overflowing!\n");
		err = -EINVAL;
		goto free_ioc_buf;
	}

	if (combo_cmd_size > ioctl_bytes) {
		dev_err(vblkdev->device,
			" buffer has no enough space to serve ioctl\n");
		err = -EFAULT;
		goto free_ioc_buf;
	}

	tmpaddr = (uint8_t *)&ic;
	for (i = 0; i < combo_info->count; i++) {
		if (copy_from_user((void *)tmpaddr, usr_ptr, sizeof(ic))) {
			err = -EFAULT;
			goto free_ioc_buf;
		}
		combo_cmd->cmd = ic.opcode;
		combo_cmd->arg = ic.arg;
		combo_cmd->flags = vblk_get_response_type(ic.flags);
		combo_cmd->write_flag = (uint32_t)ic.write_flag;
		combo_cmd->data_len = (uint32_t)(ic.blksz * ic.blocks);
		combo_cmd->buf_offset = combo_cmd_size;
		combo_cmd_size += combo_cmd->data_len;
		if ((combo_cmd_size < combo_cmd->data_len) ||
				(combo_cmd_size > ioctl_bytes)) {
			dev_err(vblkdev->device,
				" buffer has no enough space to serve ioctl\n");
			err = -EFAULT;
			goto free_ioc_buf;
		}

		if (ic.write_flag && combo_cmd->data_len) {
			if (copy_from_user((
				(void *)ioctl_buf +
				combo_cmd->buf_offset),
				(void __user *)(unsigned long)ic.data_ptr,
				(u64)combo_cmd->data_len))
			{
				dev_err(vblkdev->device,
					"copy from user failed for data!\n");
				err = -EFAULT;
				goto free_ioc_buf;
			}
		}
		combo_cmd++;
		usr_ptr++;
	}

	ioctl_req->ioctl_id = VBLK_MMC_MULTI_IOC_ID;
	ioctl_req->ioctl_buf = ioctl_buf;
	ioctl_req->ioctl_len = ioctl_bytes;

free_ioc_buf:
	if (err && ioctl_buf)
		vfree(ioctl_buf);

	return err;
}

int vblk_complete_mmc_multi_ioc(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user,
		uint32_t cmd)
{
	uint64_t num_cmd;
	struct mmc_ioc_cmd ic;
	struct mmc_ioc_cmd *ic_ptr = &ic;
	struct mmc_ioc_multi_cmd __user *user_cmd;
	struct mmc_ioc_cmd __user *usr_ptr;
	struct combo_cmd_t *combo_cmd;
	uint32_t i;
	int err = 0;
	void *ioctl_buf = ioctl_req->ioctl_buf;

	if (cmd == MMC_IOC_MULTI_CMD) {
		user_cmd = (struct mmc_ioc_multi_cmd __user *)user;
		if (copy_from_user(&num_cmd, &user_cmd->num_of_cmds,
				sizeof(num_cmd))) {
			err = -EFAULT;
			goto free_ioc_buf;
		}

		if (num_cmd > MMC_IOC_MAX_CMDS) {
			err = -EINVAL;
			goto free_ioc_buf;
		}

		usr_ptr = (void * __user)&user_cmd->cmds;
	} else {
		usr_ptr = (void * __user)user;
		num_cmd = 1;
	}

	combo_cmd = (struct combo_cmd_t *)(ioctl_buf +
			sizeof(struct combo_info_t));

	for (i = 0; i < num_cmd; i++) {
		if (copy_from_user((void *)ic_ptr, usr_ptr,
			sizeof(struct mmc_ioc_cmd))) {
			err = -EFAULT;
			goto free_ioc_buf;
		}

		if (copy_to_user(&(usr_ptr->response), combo_cmd->response,
			sizeof(combo_cmd->response))) {
			err = -EFAULT;
			goto free_ioc_buf;
		}

		if (!ic.write_flag && combo_cmd->data_len) {
			if (copy_to_user(
				(void __user *)(unsigned long)ic.data_ptr,
				(ioctl_buf + combo_cmd->buf_offset),
				(u64)combo_cmd->data_len))
			{
				dev_err(vblkdev->device,
					"copy to user of ioctl data failed!\n");
				err = -EFAULT;
				goto free_ioc_buf;
			}
		}
		combo_cmd++;
		usr_ptr++;
	}

free_ioc_buf:
	if (ioctl_buf)
		vfree(ioctl_buf);

	return err;
}
