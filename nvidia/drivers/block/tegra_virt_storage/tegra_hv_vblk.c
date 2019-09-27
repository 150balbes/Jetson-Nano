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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h> /* printk() */
#include <linux/pm.h>
#include <linux/slab.h>   /* kmalloc() */
#include <linux/fs.h>   /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/fcntl.h> /* O_ACCMODE */
#include <linux/hdreg.h> /* HDIO_GETGEO */
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <soc/tegra/chip-id.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm-generic/bug.h>
#include <scsi/scsi.h>
#include <scsi/sg.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <linux/version.h>
#include "tegra_vblk.h"

static int vblk_major;

/**
 * vblk_get_req: Get a handle to free vsc request.
 */
static struct vsc_request *vblk_get_req(struct vblk_dev *vblkdev)
{
	struct vsc_request *req = NULL;
	unsigned long bit;

	mutex_lock(&vblkdev->req_lock);

	if (vblkdev->queue_state != VBLK_QUEUE_ACTIVE)
		goto exit;

	bit = find_first_zero_bit(vblkdev->pending_reqs, vblkdev->max_requests);
	if (bit < vblkdev->max_requests) {
		req = &vblkdev->reqs[bit];
		req->vs_req.req_id = bit;
		set_bit(bit, vblkdev->pending_reqs);
		vblkdev->inflight_reqs++;
	}

exit:
	mutex_unlock(&vblkdev->req_lock);
	return req;
}

static struct vsc_request *vblk_get_req_by_sr_num(struct vblk_dev *vblkdev,
		uint32_t num)
{
	struct vsc_request *req;

	if (num >= vblkdev->max_requests)
		return NULL;

	mutex_lock(&vblkdev->req_lock);
	req = &vblkdev->reqs[num];
	if (test_bit(req->id, vblkdev->pending_reqs) == 0) {
		dev_err(vblkdev->device,
			"sr_num: Request index %d is not active!\n",
			req->id);
		req = NULL;
	}
	mutex_unlock(&vblkdev->req_lock);

	/* Assuming serial number is same as index into request array */
	return req;
}

/**
 * vblk_put_req: Free an active vsc request.
 */
static void vblk_put_req(struct vsc_request *req)
{
	struct vblk_dev *vblkdev;

	vblkdev = req->vblkdev;
	if (vblkdev == NULL) {
		pr_err("Request %d does not have valid vblkdev!\n",
				req->id);
		return;
	}

	if (req->id >= vblkdev->max_requests) {
		dev_err(vblkdev->device, "Request Index %d out of range!\n",
				req->id);
		return;
	}

	mutex_lock(&vblkdev->req_lock);
	if (req != &vblkdev->reqs[req->id]) {
		dev_err(vblkdev->device,
			"Request Index %d does not match with the request!\n",
				req->id);
		goto exit;
	}

	if (test_bit(req->id, vblkdev->pending_reqs) == 0) {
		dev_err(vblkdev->device,
			"Request index %d is not active!\n",
			req->id);
	} else {
		clear_bit(req->id, vblkdev->pending_reqs);
		memset(&req->vs_req, 0, sizeof(struct vs_request));
		req->req = NULL;
		memset(&req->iter, 0, sizeof(struct req_iterator));
		vblkdev->inflight_reqs--;

		if ((vblkdev->inflight_reqs == 0) &&
			(vblkdev->queue_state == VBLK_QUEUE_SUSPENDED)) {
			complete(&vblkdev->req_queue_empty);
		}
	}
exit:
	mutex_unlock(&vblkdev->req_lock);
}

static int vblk_send_config_cmd(struct vblk_dev *vblkdev)
{
	struct vs_request *vs_req;
	int i = 0;

	/* This while loop exits as long as the remote endpoint cooperates. */
	if (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0) {
		pr_notice("vblk: send_config wait for ivc channel reset\n");
		while (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0) {
			if (i++ > IVC_RESET_RETRIES) {
				dev_err(vblkdev->device, "ivc reset timeout\n");
				return -EIO;
			}
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(usecs_to_jiffies(1));
		}
	}
	vs_req = (struct vs_request *)
		tegra_hv_ivc_write_get_next_frame(vblkdev->ivck);
	if (IS_ERR_OR_NULL(vs_req)) {
		dev_err(vblkdev->device, "no empty frame for write\n");
		return -EIO;
	}

	vs_req->type = VS_CONFIGINFO_REQ;

	dev_info(vblkdev->device, "send config cmd to ivc #%d\n",
		vblkdev->ivc_id);

	if (tegra_hv_ivc_write_advance(vblkdev->ivck)) {
		dev_err(vblkdev->device, "ivc write failed\n");
		return -EIO;
	}

	return 0;
}

static int vblk_get_configinfo(struct vblk_dev *vblkdev)
{
	struct vs_request *req;
	int32_t status;

	dev_info(vblkdev->device, "get config data from ivc #%d\n",
		vblkdev->ivc_id);

	req = (struct vs_request *)
		tegra_hv_ivc_read_get_next_frame(vblkdev->ivck);
	if (IS_ERR_OR_NULL(req)) {
		dev_err(vblkdev->device, "no empty frame for read\n");
		return -EIO;
	}

	status = req->status;
	vblkdev->config = req->config_info;

	if (tegra_hv_ivc_read_advance(vblkdev->ivck)) {
		dev_err(vblkdev->device, "ivc read failed\n");
		return -EIO;
	}

	if (status != 0)
		return -EINVAL;

	if (vblkdev->config.type != VS_BLK_DEV) {
		dev_err(vblkdev->device, "Non Blk dev config not supported!\n");
		return -EINVAL;
	}

	if (vblkdev->config.blk_config.num_blks == 0) {
		dev_err(vblkdev->device, "controller init failed\n");
		return -EINVAL;
	}

	return 0;
}

static void req_error_handler(struct vblk_dev *vblkdev, struct request *breq)
{
	dev_err(vblkdev->device,
		"Error for request pos %llx type %llx size %x\n",
		(blk_rq_pos(breq) * (uint64_t)SECTOR_SIZE),
		(uint64_t)req_op(breq),
		blk_rq_bytes(breq));

	blk_end_request_all(breq, -EIO);
}

/**
 * complete_bio_req: Complete a bio request after server is
 *		done processing the request.
 */

static bool complete_bio_req(struct vblk_dev *vblkdev)
{
	int status = 0;
	struct bio_vec bvec;
	size_t size;
	size_t total_size = 0;
	struct vsc_request *vsc_req = NULL;
	struct vs_request *vs_req;
	struct vs_request *req_resp;
	struct request *bio_req;
	void *buffer;

	if (!tegra_hv_ivc_can_read(vblkdev->ivck))
		goto no_valid_io;

	req_resp = (struct vs_request *)
		tegra_hv_ivc_read_get_next_frame(vblkdev->ivck);
	if (IS_ERR_OR_NULL(req_resp)) {
		dev_err(vblkdev->device, "ivc read failed\n");
		goto no_valid_io;
	}

	status = req_resp->status;
	if (status != 0) {
		dev_err(vblkdev->device, "IO request error = %d\n",
				status);
	}

	vsc_req = vblk_get_req_by_sr_num(vblkdev, req_resp->req_id);
	if (vsc_req == NULL) {
		dev_err(vblkdev->device, "serial_number mismatch num %d!\n",
				req_resp->req_id);
		goto advance_frame;
	}

	bio_req = vsc_req->req;
	vs_req = &vsc_req->vs_req;

	if ((bio_req != NULL) && (status == 0)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
		if (req_op(bio_req) == REQ_OP_DRV_IN) {
#else
		if (bio_req->cmd_type == REQ_TYPE_DRV_PRIV) {
#endif
			if (req_resp->blkdev_resp.ioctl_resp.status != 0) {
				dev_err(vblkdev->device,
					"IOCTL request failed!\n");
				req_error_handler(vblkdev, bio_req);
				goto put_req;
			}

			if (vblk_complete_ioctl_req(vblkdev, vsc_req)) {
				req_error_handler(vblkdev, bio_req);
			} else {
				if (blk_end_request(bio_req, 0, 0)) {
					dev_err(vblkdev->device,
						"Error completing private request!\n");
				}
			}
		} else {
			if (req_resp->blkdev_resp.blk_resp.status != 0) {
				req_error_handler(vblkdev, bio_req);
				goto put_req;
			}

			if (req_op(bio_req) != REQ_OP_FLUSH) {
				if (vs_req->blkdev_req.blk_req.num_blks !=
						req_resp->blkdev_resp.blk_resp.num_blks) {
					req_error_handler(vblkdev, bio_req);
					goto put_req;
				}
			}

			if (req_op(bio_req) == REQ_OP_READ) {
				rq_for_each_segment(bvec, bio_req,
					vsc_req->iter) {
					size = bvec.bv_len;
					buffer = page_address(bvec.bv_page) +
						bvec.bv_offset;

					if ((total_size + size) >
						(vs_req->blkdev_req.blk_req.num_blks *
						vblkdev->config.blk_config.hardblk_size))
					{
						size =
						(vs_req->blkdev_req.blk_req.num_blks *
						vblkdev->config.blk_config.hardblk_size) -
							total_size;
					}
					memcpy(buffer,
						vsc_req->mempool_virt +
						total_size,
						size);

					total_size += size;
					if (total_size ==
						(vs_req->blkdev_req.blk_req.num_blks *
						vblkdev->config.blk_config.hardblk_size))
						break;
				}
			}

			if (blk_end_request(bio_req, 0,
				vs_req->blkdev_req.blk_req.num_blks *
					vblkdev->config.blk_config.hardblk_size)) {
				dev_err(vblkdev->device,
					"Error completing fs request!\n");
			}
		}
	} else if ((bio_req != NULL) && (status != 0)) {
		req_error_handler(vblkdev, bio_req);
	} else {
		dev_err(vblkdev->device,
			"VSC request %d has null bio request!\n",
			vsc_req->id);
	}

put_req:
	vblk_put_req(vsc_req);

advance_frame:
	if (tegra_hv_ivc_read_advance(vblkdev->ivck)) {
		dev_err(vblkdev->device,
			"Couldn't increment read frame pointer!\n");
	}

	return true;

no_valid_io:
	return false;
}

static bool bio_req_sanity_check(struct vblk_dev *vblkdev,
		struct request *bio_req,
		struct vsc_request *vsc_req)
{
	uint64_t start_offset = (blk_rq_pos(bio_req) * (uint64_t)SECTOR_SIZE);
	uint64_t req_bytes = blk_rq_bytes(bio_req);


	if ((start_offset >= vblkdev->size) || (req_bytes > vblkdev->size) ||
		((start_offset + req_bytes) > vblkdev->size))
	{
		dev_err(vblkdev->device,
			"Invalid I/O limit start 0x%llx size 0x%llx > 0x%llx\n",
			start_offset,
			req_bytes, vblkdev->size);
		return false;
	}

	if ((start_offset % vblkdev->config.blk_config.hardblk_size) != 0) {
		dev_err(vblkdev->device, "Unaligned block offset (%lld %d)\n",
			start_offset, vblkdev->config.blk_config.hardblk_size);
		return false;
	}

	if ((req_bytes % vblkdev->config.blk_config.hardblk_size) != 0) {
		dev_err(vblkdev->device, "Unaligned io length (%lld %d)\n",
			req_bytes, vblkdev->config.blk_config.hardblk_size);
		return false;
	}

	if (req_bytes > (uint64_t)vsc_req->mempool_len) {
		dev_err(vblkdev->device, "Req bytes %llx greater than %x!\n",
			req_bytes, vsc_req->mempool_len);
		return false;
	}

	return true;
}

/**
 * submit_bio_req: Fetch a bio request and submit it to
 * server for processing.
 */
static bool submit_bio_req(struct vblk_dev *vblkdev)
{
	struct vsc_request *vsc_req = NULL;
	struct request *bio_req = NULL;
	struct vs_request *vs_req;
	struct bio_vec bvec;
	size_t size;
	size_t total_size = 0;
	void *buffer;

	if (!tegra_hv_ivc_can_write(vblkdev->ivck))
		goto bio_exit;

	if (vblkdev->queue == NULL)
		goto bio_exit;

	vsc_req = vblk_get_req(vblkdev);
	if (vsc_req == NULL)
		goto bio_exit;

	spin_lock(vblkdev->queue->queue_lock);
	bio_req = blk_fetch_request(vblkdev->queue);
	spin_unlock(vblkdev->queue->queue_lock);

	if (bio_req == NULL)
		goto bio_exit;

	vsc_req->req = bio_req;
	vs_req = &vsc_req->vs_req;

	vs_req->type = VS_DATA_REQ;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
	if (req_op(bio_req) != REQ_OP_DRV_IN) {
#else
	if (bio_req->cmd_type == REQ_TYPE_FS) {
#endif
		if (req_op(bio_req) == REQ_OP_READ) {
			vs_req->blkdev_req.req_op = VS_BLK_READ;
		} else if (req_op(bio_req) == REQ_OP_WRITE) {
			vs_req->blkdev_req.req_op = VS_BLK_WRITE;
		} else if (req_op(bio_req) == REQ_OP_FLUSH) {
			vs_req->blkdev_req.req_op = VS_BLK_FLUSH;
		} else {
			dev_err(vblkdev->device,
				"Request direction is not read/write!\n");
			goto bio_exit;
		}

		vsc_req->iter.bio = NULL;
		if (req_op(bio_req) == REQ_OP_FLUSH) {
			vs_req->blkdev_req.blk_req.blk_offset = 0;
			vs_req->blkdev_req.blk_req.num_blks =
				vblkdev->config.blk_config.num_blks;
		} else {
			if (!bio_req_sanity_check(vblkdev, bio_req, vsc_req)) {
				goto bio_exit;
			}

			vs_req->blkdev_req.blk_req.blk_offset = ((blk_rq_pos(bio_req) *
				(uint64_t)SECTOR_SIZE)
				/ vblkdev->config.blk_config.hardblk_size);
			vs_req->blkdev_req.blk_req.num_blks = ((blk_rq_sectors(bio_req) *
				SECTOR_SIZE) /
				vblkdev->config.blk_config.hardblk_size);

			vs_req->blkdev_req.blk_req.data_offset = vsc_req->mempool_offset;
		}

		if (req_op(bio_req) == REQ_OP_WRITE) {
			rq_for_each_segment(bvec, bio_req, vsc_req->iter) {
				size = bvec.bv_len;
				buffer = page_address(bvec.bv_page) +
						bvec.bv_offset;

				if ((total_size + size) >
					(vs_req->blkdev_req.blk_req.num_blks *
					vblkdev->config.blk_config.hardblk_size))
				{
					size = (vs_req->blkdev_req.blk_req.num_blks *
						vblkdev->config.blk_config.hardblk_size) -
						total_size;
				}

				memcpy(vsc_req->mempool_virt + total_size,
					buffer, size);
				total_size += size;
				if (total_size == (vs_req->blkdev_req.blk_req.num_blks *
					vblkdev->config.blk_config.hardblk_size)) {
					break;
				}
			}
		}
	} else {
		if (vblk_prep_ioctl_req(vblkdev,
			(struct vblk_ioctl_req *)bio_req->special,
			vsc_req)) {
			dev_err(vblkdev->device,
				"Failed to prepare ioctl request!\n");
			goto bio_exit;
		}
	}

	if (!tegra_hv_ivc_write(vblkdev->ivck, vs_req,
				sizeof(struct vs_request))) {
		dev_err(vblkdev->device,
			"Request Id %d IVC write failed!\n",
				vsc_req->id);
		goto bio_exit;
	}

	return true;

bio_exit:
	if (vsc_req != NULL) {
		vblk_put_req(vsc_req);
	}

	if (bio_req != NULL) {
		req_error_handler(vblkdev, bio_req);
		return true;
	}

	return false;
}

static void vblk_request_work(struct work_struct *ws)
{
	struct vblk_dev *vblkdev =
		container_of(ws, struct vblk_dev, work);
	bool req_submitted, req_completed;

	if (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0)
		return;

	req_submitted = true;
	req_completed = true;
	while (req_submitted || req_completed) {
		req_completed = complete_bio_req(vblkdev);

		req_submitted = submit_bio_req(vblkdev);
	}
}

/* The simple form of the request function. */
static void vblk_request(struct request_queue *q)
{
	struct vblk_dev *vblkdev = q->queuedata;

	queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);
}

/* Open and release */
static int vblk_open(struct block_device *device, fmode_t mode)
{
	struct vblk_dev *vblkdev = device->bd_disk->private_data;

	spin_lock(&vblkdev->lock);
	if (!vblkdev->users)
		check_disk_change(device);
	vblkdev->users++;

	spin_unlock(&vblkdev->lock);
	return 0;
}

static void vblk_release(struct gendisk *disk, fmode_t mode)
{
	struct vblk_dev *vblkdev = disk->private_data;

	spin_lock(&vblkdev->lock);

	vblkdev->users--;

	spin_unlock(&vblkdev->lock);
}

int vblk_getgeo(struct block_device *device, struct hd_geometry *geo)
{
	geo->heads = VS_LOG_HEADS;
	geo->sectors = VS_LOG_SECTS;
	geo->cylinders = get_capacity(device->bd_disk) /
		(geo->heads * geo->sectors);

	return 0;
}

/* The device operations structure. */
const struct block_device_operations vblk_ops = {
	.owner           = THIS_MODULE,
	.open            = vblk_open,
	.release         = vblk_release,
	.getgeo          = vblk_getgeo,
	.ioctl           = vblk_ioctl
};

/* Set up virtual device. */
static void setup_device(struct vblk_dev *vblkdev)
{
	uint32_t max_io_bytes;
	uint32_t req_id;
	uint32_t max_requests;
	struct vsc_request *req;

	vblkdev->size =
		vblkdev->config.blk_config.num_blks *
			vblkdev->config.blk_config.hardblk_size;

	spin_lock_init(&vblkdev->lock);
	spin_lock_init(&vblkdev->queue_lock);
	mutex_init(&vblkdev->ioctl_lock);

	vblkdev->queue = blk_init_queue(vblk_request, &vblkdev->queue_lock);
	if (vblkdev->queue == NULL) {
		dev_err(vblkdev->device, "failed to init blk queue\n");
		return;
	}

	vblkdev->queue->queuedata = vblkdev;

	blk_queue_logical_block_size(vblkdev->queue,
		vblkdev->config.blk_config.hardblk_size);
	blk_queue_physical_block_size(vblkdev->queue,
		vblkdev->config.blk_config.hardblk_size);

	if (vblkdev->config.blk_config.req_ops_supported & VS_BLK_FLUSH_OP_F) {
		blk_queue_write_cache(vblkdev->queue, true, false);
	}

	if (vblkdev->config.blk_config.max_read_blks_per_io !=
		vblkdev->config.blk_config.max_write_blks_per_io) {
		dev_err(vblkdev->device,
			"Different read/write blks not supported!\n");
		return;
	}

	/* Set the maximum number of requests possible using
	 * server returned information */
	max_io_bytes = (vblkdev->config.blk_config.hardblk_size *
			vblkdev->config.blk_config.max_read_blks_per_io);
	if (max_io_bytes == 0) {
		dev_err(vblkdev->device, "Maximum io bytes value is 0!\n");
		return;
	}

	max_requests = ((vblkdev->ivmk->size) / max_io_bytes);

	if (max_requests < MAX_VSC_REQS) {
		/* Warn if the virtual storage device supports
		 * normal read write operations */
		if (vblkdev->config.blk_config.req_ops_supported &
				(VS_BLK_READ_OP_F |
				 VS_BLK_WRITE_OP_F)) {
			dev_warn(vblkdev->device,
				"Setting Max requests to %d, consider "
				"increasing mempool size !\n",
				max_requests);
		}
	} else if (max_requests > MAX_VSC_REQS) {
		max_requests = MAX_VSC_REQS;
		dev_warn(vblkdev->device,
			"Reducing the max requests to %d, consider"
			" supporting more requests for the vblkdev!\n",
			MAX_VSC_REQS);
	}

	if (vblkdev->ivck->nframes < max_requests) {
		/* Warn if the virtual storage device supports
		 * normal read write operations */
		if (vblkdev->config.blk_config.req_ops_supported &
				(VS_BLK_READ_OP_F |
				 VS_BLK_WRITE_OP_F)) {
			dev_warn(vblkdev->device,
				"IVC frames %d less than possible max requests %d!\n",
				vblkdev->ivck->nframes, max_requests);
		}
	}

	for (req_id = 0; req_id < max_requests; req_id++){
		req = &vblkdev->reqs[req_id];
		req->mempool_virt = (void *)((uintptr_t)vblkdev->shared_buffer +
			(uintptr_t)(req_id * max_io_bytes));
		req->mempool_offset = (req_id * max_io_bytes);
		req->mempool_len = max_io_bytes;
		req->id = req_id;
		req->vblkdev = vblkdev;
	}

	if (max_requests == 0) {
		dev_err(vblkdev->device,
			"maximum requests set to 0!\n");
		return;
	}
	mutex_init(&vblkdev->req_lock);

	vblkdev->max_requests = max_requests;
	blk_queue_max_hw_sectors(vblkdev->queue, max_io_bytes / SECTOR_SIZE);
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, vblkdev->queue);

	/* And the gendisk structure. */
	vblkdev->gd = alloc_disk(VBLK_MINORS);
	if (!vblkdev->gd) {
		dev_err(vblkdev->device, "alloc_disk failure\n");
		return;
	}
	vblkdev->gd->major = vblk_major;
	vblkdev->gd->first_minor = vblkdev->devnum * VBLK_MINORS;
	vblkdev->gd->fops = &vblk_ops;
	vblkdev->gd->queue = vblkdev->queue;
	vblkdev->gd->private_data = vblkdev;
	vblkdev->gd->flags |= GENHD_FL_EXT_DEVT;

	/* Don't allow scanning of the device when block
	 * requests are not supported */
	if (!(vblkdev->config.blk_config.req_ops_supported &
				VS_BLK_READ_OP_F)) {
		vblkdev->gd->flags |= GENHD_FL_NO_PART_SCAN;
	}

	snprintf(vblkdev->gd->disk_name, 32, "vblkdev%d", vblkdev->devnum);
	set_capacity(vblkdev->gd, (vblkdev->size / SECTOR_SIZE));
	device_add_disk(vblkdev->device, vblkdev->gd);
}

static void vblk_init_device(struct work_struct *ws)
{
	struct vblk_dev *vblkdev = container_of(ws, struct vblk_dev, init);

	/* wait for ivc channel reset to finish */
	if (tegra_hv_ivc_channel_notified(vblkdev->ivck) != 0)
		return;	/* this will be rescheduled by irq handler */

	if (tegra_hv_ivc_can_read(vblkdev->ivck) && !vblkdev->initialized) {
		if (vblk_get_configinfo(vblkdev))
			return;

		vblkdev->initialized = true;
		setup_device(vblkdev);
	}
}

static irqreturn_t ivc_irq_handler(int irq, void *data)
{
	struct vblk_dev *vblkdev = (struct vblk_dev *)data;

	if (vblkdev->initialized)
		queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);
	else
		schedule_work(&vblkdev->init);

	return IRQ_HANDLED;
}

static int tegra_hv_vblk_probe(struct platform_device *pdev)
{
	static struct device_node *vblk_node;
	struct vblk_dev *vblkdev;
	struct device *dev = &pdev->dev;
	int ret;
	struct tegra_hv_ivm_cookie *ivmk;

	if (!is_tegra_hypervisor_mode()) {
		dev_err(dev, "Hypervisor is not present\n");
		return -ENODEV;
	}

	if (vblk_major == 0) {
		dev_err(dev, "major number is invalid\n");
		return -ENODEV;
	}

	vblk_node = dev->of_node;
	if (vblk_node == NULL) {
		dev_err(dev, "No of_node data\n");
		return -ENODEV;
	}

	dev_info(dev, "allocate drvdata buffer\n");
	vblkdev = devm_kzalloc(dev, sizeof(struct vblk_dev), GFP_KERNEL);
	if (vblkdev == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, vblkdev);
	vblkdev->device = dev;

	/* Get properties of instance and ivc channel id */
	if (of_property_read_u32(vblk_node, "instance", &(vblkdev->devnum))) {
		dev_err(dev, "Failed to read instance property\n");
		ret = -ENODEV;
		goto fail;
	} else {
		if (of_property_read_u32_index(vblk_node, "ivc", 1,
			&(vblkdev->ivc_id))) {
			dev_err(dev, "Failed to read ivc property\n");
			ret = -ENODEV;
			goto fail;
		}
		if (of_property_read_u32_index(vblk_node, "mempool", 0,
			&(vblkdev->ivm_id))) {
			dev_err(dev, "Failed to read mempool property\n");
			ret = -ENODEV;
			goto fail;
		}
	}

	vblkdev->ivck = tegra_hv_ivc_reserve(NULL, vblkdev->ivc_id, NULL);
	if (IS_ERR_OR_NULL(vblkdev->ivck)) {
		dev_err(dev, "Failed to reserve IVC channel %d\n",
			vblkdev->ivc_id);
		vblkdev->ivck = NULL;
		ret = -ENODEV;
		goto fail;
	}

	ivmk = tegra_hv_mempool_reserve(vblkdev->ivm_id);
	if (IS_ERR_OR_NULL(ivmk)) {
		dev_err(dev, "Failed to reserve IVM channel %d\n",
			vblkdev->ivm_id);
		ivmk = NULL;
		ret = -ENODEV;
		goto free_ivc;
	}
	vblkdev->ivmk = ivmk;

	vblkdev->shared_buffer = devm_memremap(vblkdev->device,
			ivmk->ipa, ivmk->size, MEMREMAP_WB);
	if (IS_ERR_OR_NULL(vblkdev->shared_buffer)) {
		dev_err(dev, "Failed to map mempool area %d\n",
				vblkdev->ivm_id);
		ret = -ENOMEM;
		goto free_mempool;
	}

	vblkdev->initialized = false;

	vblkdev->wq = alloc_workqueue("vblk_req_wq%d",
		WQ_UNBOUND | WQ_MEM_RECLAIM,
		1, vblkdev->devnum);
	if (vblkdev->wq == NULL) {
		dev_err(dev, "Failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto free_mempool;
	}

	init_completion(&vblkdev->req_queue_empty);
	vblkdev->queue_state = VBLK_QUEUE_ACTIVE;

	INIT_WORK(&vblkdev->init, vblk_init_device);
	INIT_WORK(&vblkdev->work, vblk_request_work);

	if (devm_request_irq(vblkdev->device, vblkdev->ivck->irq,
		ivc_irq_handler, 0, "vblk", vblkdev)) {
		dev_err(dev, "Failed to request irq %d\n", vblkdev->ivck->irq);
		ret = -EINVAL;
		goto free_wq;
	}

	tegra_hv_ivc_channel_reset(vblkdev->ivck);
	if (vblk_send_config_cmd(vblkdev)) {
		dev_err(dev, "Failed to send config cmd\n");
		ret = -EACCES;
		goto free_wq;
	}

	return 0;

free_wq:
	destroy_workqueue(vblkdev->wq);

free_mempool:
	tegra_hv_mempool_unreserve(vblkdev->ivmk);

free_ivc:
	tegra_hv_ivc_unreserve(vblkdev->ivck);

fail:
	return ret;
}

static int tegra_hv_vblk_remove(struct platform_device *pdev)
{
	struct vblk_dev *vblkdev = platform_get_drvdata(pdev);

	if (vblkdev->gd) {
		del_gendisk(vblkdev->gd);
		put_disk(vblkdev->gd);
	}

	if (vblkdev->queue)
		blk_cleanup_queue(vblkdev->queue);

	destroy_workqueue(vblkdev->wq);
	tegra_hv_ivc_unreserve(vblkdev->ivck);
	tegra_hv_mempool_unreserve(vblkdev->ivmk);

	return 0;
}

static int __init vblk_init(void)
{
	vblk_major = 0;
	vblk_major = register_blkdev(vblk_major, "vblk");
	if (vblk_major <= 0) {
		pr_err("vblk: unable to get major number\n");
		return -ENODEV;
	}

	return 0;
}

static void vblk_exit(void)
{
	unregister_blkdev(vblk_major, "vblk");
}

#ifdef CONFIG_PM_SLEEP
static int tegra_hv_vblk_suspend(struct device *dev)
{
	struct vblk_dev *vblkdev = dev_get_drvdata(dev);
	unsigned long flags;

	if (vblkdev->queue) {
		spin_lock_irqsave(vblkdev->queue->queue_lock, flags);
		blk_stop_queue(vblkdev->queue);
		spin_unlock_irqrestore(vblkdev->queue->queue_lock, flags);

		mutex_lock(&vblkdev->req_lock);
		vblkdev->queue_state = VBLK_QUEUE_SUSPENDED;

		/* Mark the queue as empty if inflight requests are 0 */
		if (vblkdev->inflight_reqs == 0)
			complete(&vblkdev->req_queue_empty);
		mutex_unlock(&vblkdev->req_lock);

		wait_for_completion(&vblkdev->req_queue_empty);
		disable_irq(vblkdev->ivck->irq);

		flush_workqueue(vblkdev->wq);

		/* Reset the channel */
		tegra_hv_ivc_channel_reset(vblkdev->ivck);
	}

	return 0;
}

static int tegra_hv_vblk_resume(struct device *dev)
{
	struct vblk_dev *vblkdev = dev_get_drvdata(dev);
	unsigned long flags;

	if (vblkdev->queue) {
		mutex_lock(&vblkdev->req_lock);
		vblkdev->queue_state = VBLK_QUEUE_ACTIVE;
		reinit_completion(&vblkdev->req_queue_empty);
		mutex_unlock(&vblkdev->req_lock);

		enable_irq(vblkdev->ivck->irq);

		spin_lock_irqsave(vblkdev->queue->queue_lock, flags);
		blk_start_queue(vblkdev->queue);
		spin_unlock_irqrestore(vblkdev->queue->queue_lock, flags);

		queue_work_on(WORK_CPU_UNBOUND, vblkdev->wq, &vblkdev->work);
	}

	return 0;
}

static const struct dev_pm_ops tegra_hv_vblk_pm_ops = {
	.suspend = tegra_hv_vblk_suspend,
	.resume = tegra_hv_vblk_resume,
};
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static struct of_device_id tegra_hv_vblk_match[] = {
	{ .compatible = "nvidia,tegra-hv-storage", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_hv_vblk_match);
#endif /* CONFIG_OF */

static struct platform_driver tegra_hv_vblk_driver = {
	.probe	= tegra_hv_vblk_probe,
	.remove	= tegra_hv_vblk_remove,
	.driver	= {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_hv_vblk_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &tegra_hv_vblk_pm_ops,
#endif
	},
};

module_platform_driver(tegra_hv_vblk_driver);

module_init(vblk_init);
module_exit(vblk_exit);

MODULE_AUTHOR("Dilan Lee <dilee@nvidia.com>");
MODULE_DESCRIPTION("Virtual storage device over Tegra Hypervisor IVC channel");
MODULE_LICENSE("GPL");

