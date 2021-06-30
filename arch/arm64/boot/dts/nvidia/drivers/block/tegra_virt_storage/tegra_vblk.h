/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_VBLK_H_
#define _TEGRA_VBLK_H_

#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/tegra-ivc.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <tegra_virt_storage_spec.h>

#define DRV_NAME "tegra_hv_vblk"

/* Minor number and partition management. */
#define VBLK_MINORS 16

#define IVC_RESET_RETRIES	30

#define VS_LOG_HEADS 4
#define VS_LOG_SECTS 16

#define SECTOR_SIZE 512

#define MAX_VSC_REQS 32

struct vblk_ioctl_req {
	uint32_t ioctl_id;
	void *ioctl_buf;
	uint32_t ioctl_len;
};

struct vsc_request {
	struct vs_request vs_req;
	struct request *req;
	struct req_iterator iter;
	struct vblk_ioctl_req *ioctl_req;
	void *mempool_virt;
	uint32_t mempool_offset;
	uint32_t mempool_len;
	uint32_t id;
	struct vblk_dev* vblkdev;
};

enum vblk_queue_state {
	VBLK_UNKNOWN,
	VBLK_QUEUE_SUSPENDED,
	VBLK_QUEUE_ACTIVE,
};

/*
* The drvdata of virtual device.
*/
struct vblk_dev {
	struct vs_config_info config;
	uint64_t size;                   /* Device size in bytes */
	short users;                     /* How many users */
	short media_change;              /* Flag a media change? */
	spinlock_t lock;                 /* For mutual exclusion */
	struct request_queue *queue;     /* The device request queue */
	struct gendisk *gd;              /* The gendisk structure */
	uint32_t ivc_id;
	uint32_t ivm_id;
	struct tegra_hv_ivc_cookie *ivck;
	struct tegra_hv_ivm_cookie *ivmk;
	uint32_t devnum;
	bool initialized;
	struct work_struct init;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct device *device;
	void *shared_buffer;
	struct mutex ioctl_lock;
	spinlock_t queue_lock;
	struct vsc_request reqs[MAX_VSC_REQS];
	DECLARE_BITMAP(pending_reqs, MAX_VSC_REQS);
	uint32_t inflight_reqs;
	uint32_t max_requests;
	struct mutex req_lock;
	enum vblk_queue_state queue_state;
	struct completion req_queue_empty;
};

int vblk_complete_ioctl_req(struct vblk_dev *vblkdev,
		struct vsc_request *vsc_req);

int vblk_prep_ioctl_req(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		struct vsc_request *vsc_req);

int vblk_prep_sg_io(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user);

int vblk_complete_sg_io(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user);

int vblk_prep_mmc_multi_ioc(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user,
		uint32_t cmd);

int vblk_complete_mmc_multi_ioc(struct vblk_dev *vblkdev,
		struct vblk_ioctl_req *ioctl_req,
		void __user *user,
		uint32_t cmd);

int vblk_submit_ioctl_req(struct block_device *bdev,
		unsigned int cmd, void __user *user);

int vblk_ioctl(struct block_device *bdev, fmode_t mode,
	unsigned int cmd, unsigned long arg);
#endif
