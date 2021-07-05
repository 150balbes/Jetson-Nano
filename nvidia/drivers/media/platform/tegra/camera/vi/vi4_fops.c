/*
 * Tegra Video Input 4 device common APIs
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frank@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/nvhost.h>
#include <linux/tegra-powergate.h>
#include <media/capture.h>
#include <media/tegra_camera_platform.h>
#include <media/mc_common.h>
#include <media/vi4_registers.h>
#include <video/vi4.h>
#include <uapi/linux/nvhost_ioctl.h>
#include "nvhost_acm.h"
#include "vi4_formats.h"
#include "vi/vi_notify.h"
#include "vi4_fops.h"
#include <media/sensor_common.h>
#include <trace/events/camera_common.h>
#define BPP_MEM		2
#define MAX_VI_CHANNEL 12
#define NUM_FIELDS_INTERLACED 2
#define NUM_FIELDS_SINGLE 1
#define TOP_FIELD 2
#define BOTTOM_FIELD 1
#define SOF_SYNCPT_IDX	0
#define FE_SYNCPT_IDX	1
/* 256 byte alignment in accordance to NvRmSurface Pitch alignment.
 * It is the worst case scenario considering VIC engine requirements
 */
#define RM_SURFACE_ALIGNMENT 256

static void tegra_channel_error_recovery(struct tegra_channel *chan);
static void tegra_channel_stop_kthreads(struct tegra_channel *chan);
static int tegra_channel_stop_increments(struct tegra_channel *chan);
static void tegra_channel_notify_status_callback(
				struct vi_notify_channel *,
				const struct vi_capture_status *,
				void *);
static void tegra_channel_error_worker(struct work_struct *status_work);
static void tegra_channel_notify_error_callback(void *);

static u32 csimux_config_stream[] = {
	CSIMUX_CONFIG_STREAM_0,
	CSIMUX_CONFIG_STREAM_1,
	CSIMUX_CONFIG_STREAM_2,
	CSIMUX_CONFIG_STREAM_3,
	CSIMUX_CONFIG_STREAM_4,
	CSIMUX_CONFIG_STREAM_5
};

static void vi4_write(struct tegra_channel *chan, unsigned int addr, u32 val)
{
	writel(val, chan->vi->iomem + addr);
}

static u32 vi4_read(struct tegra_channel *chan, unsigned int addr)
{
	return readl(chan->vi->iomem + addr);
}

static void vi4_channel_write(struct tegra_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	writel(val,
		chan->vi->iomem + VI4_CHANNEL_OFFSET * (index + 1) + addr);
}

static void vi4_init_video_formats(struct tegra_channel *chan)
{
	int i;

	chan->num_video_formats = ARRAY_SIZE(vi4_video_formats);
	for (i = 0; i < chan->num_video_formats; i++)
		chan->video_formats[i] = &vi4_video_formats[i];
}

static int tegra_vi4_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegra_channel *chan = container_of(ctrl->handler,
				struct tegra_channel, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_WRITE_ISPFORMAT:
		chan->write_ispformat = ctrl->val;
		break;
	default:
		dev_err(&chan->video->dev, "%s:Not valid ctrl\n", __func__);
		return -EINVAL;
	}

	return err;
}

static const struct v4l2_ctrl_ops vi4_ctrl_ops = {
	.s_ctrl	= tegra_vi4_s_ctrl,
};

static const struct v4l2_ctrl_config vi4_custom_ctrls[] = {
	{
		.ops = &vi4_ctrl_ops,
		.id = TEGRA_CAMERA_CID_WRITE_ISPFORMAT,
		.name = "Write ISP format",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = 1,
		.min = 1,
		.max = 1,
		.step = 1,
	},
};

static int vi4_add_ctrls(struct tegra_channel *chan)
{
	int i;

	/* Add vi4 custom controls */
	for (i = 0; i < ARRAY_SIZE(vi4_custom_ctrls); i++) {
		v4l2_ctrl_new_custom(&chan->ctrl_handler,
			&vi4_custom_ctrls[i], NULL);
		if (chan->ctrl_handler.error) {
			dev_err(chan->vi->dev,
				"Failed to add %s ctrl\n",
				vi4_custom_ctrls[i].name);
			return chan->ctrl_handler.error;
		}
	}

	return 0;
}

static int vi4_channel_setup_queue(struct tegra_channel *chan,
	unsigned int *nbuffers)
{
	/* Make sure minimum number of buffers are passed */
	if (*nbuffers < (QUEUED_BUFFERS - 1))
		*nbuffers = QUEUED_BUFFERS - 1;

	return tegra_channel_alloc_buffer_queue(chan, QUEUED_BUFFERS);
}

static bool vi4_init(struct tegra_channel *chan)
{
	vi4_write(chan, NOTIFY_ERROR, 0x1);
	vi4_write(chan, NOTIFY_TAG_CLASSIFY_0, 0xe39c08e3);
	return true;
}

static bool vi4_check_status(struct tegra_channel *chan)
{
	int status;

	/* check interrupt status error */
	status = vi4_read(chan, CFG_INTERRUPT_STATUS);
	if (status & 0x1)
		dev_err(chan->vi->dev,
			"VI_CFG_INTERRUPT_STATUS_0: MASTER_ERR_STATUS error!\n");

	/* Check VI NOTIFY input FIFO error */
	status = vi4_read(chan, NOTIFY_ERROR);
	if (status & 0x1)
		dev_err(chan->vi->dev,
			"VI_NOTIFY_ERROR_0: NOTIFY_FIFO_OVERFLOW error!\n");

	return true;
}

static bool vi_notify_wait(struct tegra_channel *chan,
		struct tegra_channel_buffer *buf,
		struct timespec *ts)
{
	int i, err;
	u32 thresh[TEGRA_CSI_BLOCKS];
	struct vb2_v4l2_buffer *vb = &buf->buf;
	/*
	 * Increment syncpt for ATOMP_FE
	 *
	 * This is needed in order to keep the syncpt max up to date,
	 * even if we are not waiting for ATOMP_FE here
	 */
	for (i = 0; i < chan->valid_ports; i++)
		buf->thresh[i] = nvhost_syncpt_incr_max_ext(
			chan->vi->ndev,
			chan->syncpt[i][FE_SYNCPT_IDX], 1);

	/*
	 * Increment syncpt for PXL_SOF
	 *
	 * Increment and retrieve PXL_SOF syncpt max value.
	 * This value will be used to wait for next syncpt
	 */
	for (i = 0; i < chan->valid_ports; i++)
		thresh[i] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
					chan->syncpt[i][SOF_SYNCPT_IDX], 1);

	/*
	 * Wait for PXL_SOF syncpt
	 *
	 * Use the syncpt max value we just set as threshold
	 */
	for (i = 0; i < chan->valid_ports; i++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
				chan->syncpt[i][SOF_SYNCPT_IDX], thresh[i],
				chan->timeout, NULL, NULL);
		if (unlikely(err)) {
			dev_err(chan->vi->dev,
				"PXL_SOF syncpt timeout! err = %d\n", err);
			return false;
		} else {
			struct vi_capture_status status;

			err = vi_notify_get_capture_status(chan->vnc[i],
					chan->vnc_id[i],
					thresh[i], &status);
			/* Update the buffer sequence received along with
                         * CSI PXL_EOF Event
                         */
			vb->sequence = status.frame;
			if (unlikely(err))
				dev_err(chan->vi->dev,
					"no capture status! err = %d\n", err);
			else {
				*ts = ns_to_timespec((s64)status.sof_ts);

				dev_dbg(&chan->video->dev,
					"%s: vi4 got SOF syncpt buf[%p]\n",
					__func__, buf);
			}
		}
	}

	return true;
}

static void tegra_channel_surface_setup(
	struct tegra_channel *chan, struct tegra_channel_buffer *buf, int index)
{
	int vnc_id = chan->vnc_id[index];
	unsigned int offset = chan->buffer_offset[index];

	if (chan->embedded_data_height > 0) {
		vi4_channel_write(chan, vnc_id, ATOMP_EMB_SURFACE_OFFSET0,
						  chan->vi->emb_buf);
		vi4_channel_write(chan, vnc_id, ATOMP_EMB_SURFACE_OFFSET0_H,
					  (chan->vi->emb_buf) >> 32 & 0xFF);
	} else {
		vi4_channel_write(chan, vnc_id, ATOMP_EMB_SURFACE_OFFSET0, 0);
		vi4_channel_write(chan, vnc_id, ATOMP_EMB_SURFACE_OFFSET0_H, 0x0);
	}

	vi4_channel_write(chan, vnc_id, ATOMP_EMB_SURFACE_STRIDE0,
					  chan->embedded_data_width * BPP_MEM);
	vi4_channel_write(chan, vnc_id,
		ATOMP_SURFACE_OFFSET0, buf->addr + offset);
	vi4_channel_write(chan, vnc_id,
		ATOMP_SURFACE_STRIDE0,
		chan->format.bytesperline * chan->interlace_bplfactor);
	vi4_channel_write(chan, vnc_id,
		ATOMP_SURFACE_OFFSET0_H, (buf->addr >> 32) & 0xFF);

	if (chan->fmtinfo->fourcc == V4L2_PIX_FMT_NV16) {
		vi4_channel_write(chan, vnc_id,
			ATOMP_SURFACE_OFFSET1, buf->addr + offset +
			chan->format.sizeimage / 2);
		vi4_channel_write(chan, vnc_id,
			ATOMP_SURFACE_OFFSET1_H, ((u64)buf->addr + offset +
			chan->format.sizeimage / 2) >> 32);
		vi4_channel_write(chan, vnc_id,
			ATOMP_SURFACE_STRIDE1,
			chan->format.bytesperline * chan->interlace_bplfactor);

	} else {
		vi4_channel_write(chan, vnc_id, ATOMP_SURFACE_OFFSET1, 0x0);
		vi4_channel_write(chan, vnc_id, ATOMP_SURFACE_OFFSET1_H, 0x0);
		vi4_channel_write(chan, vnc_id, ATOMP_SURFACE_STRIDE1, 0x0);
	}

	vi4_channel_write(chan, vnc_id, ATOMP_SURFACE_OFFSET2, 0x0);
	vi4_channel_write(chan, vnc_id, ATOMP_SURFACE_OFFSET2_H, 0x0);
	vi4_channel_write(chan, vnc_id, ATOMP_SURFACE_STRIDE2, 0x0);
}

static void tegra_channel_handle_error(struct tegra_channel *chan)
{
	struct v4l2_subdev *sd_on_csi = chan->subdev_on_csi;
	static const struct v4l2_event source_ev_fmt = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_ERROR,
	};

	tegra_channel_stop_increments(chan);
	vb2_queue_error(&chan->queue);

	/* Application gets notified after CSI Tx's are reset */
	if (sd_on_csi->devnode)
		v4l2_subdev_notify_event(sd_on_csi, &source_ev_fmt);
}

static void tegra_channel_status_worker(struct work_struct *status_work)
{
	struct tegra_channel *chan;

	chan = container_of(status_work, struct tegra_channel, status_work);

	tegra_channel_handle_error(chan);
}

static void tegra_channel_notify_status_callback(
				struct vi_notify_channel *vnc,
				const struct vi_capture_status *status,
				void *client_data)
{
	struct tegra_channel *chan = (struct tegra_channel *)client_data;
	int i;

	spin_lock(&chan->capture_state_lock);
	if (chan->capture_state == CAPTURE_GOOD)
		chan->capture_state = CAPTURE_ERROR;
	else {
		spin_unlock(&chan->capture_state_lock);
		return;
	}
	spin_unlock(&chan->capture_state_lock);

	for (i = 0; i < chan->valid_ports; i++)
		dev_err(chan->vi->dev, "Status: %2u channel:%02X frame:%04X\n",
			status->status, chan->vnc_id[i], status->frame);
	dev_err(chan->vi->dev, "     timestamp sof %llu eof %llu data 0x%08x\n",
		status->sof_ts, status->eof_ts, status->data);
	dev_err(chan->vi->dev, "     capture_id %u stream %2u vchan %2u\n",
		status->capture_id, status->st, status->vc);

	schedule_work(&chan->status_work);
	trace_tegra_channel_notify_status_callback("");
}

static int tegra_channel_notify_enable(
	struct tegra_channel *chan, unsigned int index)
{
	struct tegra_vi4_syncpts_req req;
	int i, err;

	chan->vnc_id[index] = -1;
	for (i = 0; i < MAX_VI_CHANNEL; i++) {
		chan->vnc[index] = vi_notify_channel_open(i);
		if (!IS_ERR(chan->vnc[index])) {
			chan->vnc_id[index] = i;
			break;
		}
	}
	if (chan->vnc_id[index] < 0) {
		dev_err(chan->vi->dev, "No VI channel available!\n");
		return -EFAULT;
	}

	vi_notify_channel_set_notify_funcs(chan->vnc[index],
			&tegra_channel_notify_status_callback,
			&tegra_channel_notify_error_callback,
			(void *)chan);

	/* get PXL_SOF syncpt id */
	chan->syncpt[index][SOF_SYNCPT_IDX] =
		nvhost_get_syncpt_client_managed(chan->vi->ndev, "tegra-vi4");
	if (chan->syncpt[index][SOF_SYNCPT_IDX] == 0) {
		dev_err(chan->vi->dev, "Failed to get PXL_SOF syncpt!\n");
		return -EFAULT;
	}

	/* get ATOMP_FE syncpt id */
	chan->syncpt[index][FE_SYNCPT_IDX] =
		nvhost_get_syncpt_client_managed(chan->vi->ndev, "tegra-vi4");
	if (chan->syncpt[index][FE_SYNCPT_IDX] == 0) {
		dev_err(chan->vi->dev, "Failed to get ATOMP_FE syncpt!\n");
		nvhost_syncpt_put_ref_ext(
			chan->vi->ndev, chan->syncpt[index][SOF_SYNCPT_IDX]);
		return -EFAULT;
	}

	nvhost_syncpt_set_min_eq_max_ext(
		chan->vi->ndev, chan->syncpt[index][SOF_SYNCPT_IDX]);
	nvhost_syncpt_set_min_eq_max_ext(
		chan->vi->ndev, chan->syncpt[index][FE_SYNCPT_IDX]);

	/* enable VI Notify report */
	req.syncpt_ids[0] = chan->syncpt[index][SOF_SYNCPT_IDX]; /* PXL_SOF */
	req.syncpt_ids[1] = chan->syncpt[index][FE_SYNCPT_IDX]; /* ATOMP_FE */
	req.syncpt_ids[2] = 0xffffffff;
	req.stream = chan->port[index];
	req.vc = chan->virtual_channel;
	req.pad = 0;

	err = vi_notify_channel_enable_reports(
		chan->vnc_id[index], chan->vnc[index], &req);
	if (err < 0)
		dev_err(chan->vi->dev,
			"Failed to enable report for VI Notify, err = %d\n",
			err);

	return err;
}

static int tegra_channel_notify_disable(
	struct tegra_channel *chan, unsigned int index)
{
	int err;
	int ret = 0;
	struct tegra_vi4_syncpts_req req;

	/* clear vi notify callbacks */
	vi_notify_channel_set_notify_funcs(chan->vnc[index],
			NULL, NULL, NULL);

	/* free syncpts */
	nvhost_syncpt_put_ref_ext(
		chan->vi->ndev, chan->syncpt[index][SOF_SYNCPT_IDX]);
	nvhost_syncpt_put_ref_ext(
		chan->vi->ndev, chan->syncpt[index][FE_SYNCPT_IDX]);

	/* close vi-notifier */
	req.syncpt_ids[0] = 0xffffffff;
	req.syncpt_ids[1] = 0xffffffff;
	req.syncpt_ids[2] = 0xffffffff;
	req.stream = chan->port[index];
	req.vc = chan->virtual_channel;
	req.pad = 0;

	err = vi_notify_channel_reset(
		chan->vnc_id[index], chan->vnc[index], &req);
	if (err < 0) {
		dev_err(chan->vi->dev,
			"VI Notify channel reset failed, err = %d\n", err);
		if (!ret)
			ret = err;
	}

	err = vi_notify_channel_close(chan->vnc_id[index], chan->vnc[index]);
	if (err < 0) {
		dev_err(chan->vi->dev,
			"VI Notify channel close failed, err = %d\n", err);
		if (!ret)
			ret = err;
	}

	return ret;
}

static int tegra_channel_capture_setup(struct tegra_channel *chan,
		unsigned int index)
{
	u32 height = chan->format.height;
	u32 width = chan->format.width;
	u32 format = chan->fmtinfo->img_fmt;
	u32 data_type = chan->fmtinfo->img_dt;
	u32 csi_port = chan->port[index];
	u32 stream = 1U << csi_port;
	u32 virtual_ch = 1U << chan->virtual_channel;
	u32 vnc_id;
	int err;

	if (chan->valid_ports > 1) {
		height = chan->gang_height;
		width = chan->gang_width;
	}

	err = tegra_channel_notify_enable(chan, index);
	if (err < 0) {
		dev_err(chan->vi->dev,
			"Failed to setup VI Notifier, err = %d\n", err);
		return err;
	}

	trace_tegra_channel_capture_setup(chan, index);
	vnc_id = chan->vnc_id[index];

	vi4_write(chan, csimux_config_stream[csi_port], 0x1);

	vi4_channel_write(chan, vnc_id, MATCH,
			((stream << STREAM_SHIFT) & STREAM) |
			STREAM_MASK |
			((virtual_ch << VIRTUAL_CHANNEL_SHIFT) &
			VIRTUAL_CHANNEL)  |
			VIRTUAL_CHANNEL_MASK);

	vi4_channel_write(chan, vnc_id, MATCH_DATATYPE,
			((data_type << DATATYPE_SHIFT) & DATATYPE) |
			DATATYPE_MASK);

	vi4_channel_write(chan, vnc_id, DT_OVERRIDE, 0x0);

	vi4_channel_write(chan, vnc_id, MATCH_FRAMEID,
			((0 << FRAMEID_SHIFT) & FRAMEID) | 0);

	vi4_channel_write(chan, vnc_id, FRAME_X, width);
	vi4_channel_write(chan, vnc_id, FRAME_Y, height);
	vi4_channel_write(chan, vnc_id, SKIP_X, 0x0);
	vi4_channel_write(chan, vnc_id, CROP_X, width);
	vi4_channel_write(chan, vnc_id, OUT_X, width);
	vi4_channel_write(chan, vnc_id, SKIP_Y, 0x0);
	vi4_channel_write(chan, vnc_id, CROP_Y, height);
	vi4_channel_write(chan, vnc_id, OUT_Y, height);
	vi4_channel_write(chan, vnc_id, PIXFMT_ENABLE, PIXFMT_EN);
	vi4_channel_write(chan, vnc_id, PIXFMT_WIDE, 0x0);
	vi4_channel_write(chan, vnc_id, PIXFMT_FORMAT, format);
	vi4_channel_write(chan, vnc_id, DPCM_STRIP, 0x0);
	vi4_channel_write(chan, vnc_id, ATOMP_DPCM_CHUNK, 0x0);
	vi4_channel_write(chan, vnc_id, ISPBUFA, 0x0);
	vi4_channel_write(chan, vnc_id, LINE_TIMER, 0x1000000);
	if (chan->embedded_data_height > 0) {
		vi4_channel_write(chan, vnc_id, EMBED_X,
			chan->embedded_data_width * BPP_MEM);
		vi4_channel_write(chan, vnc_id, EMBED_Y,
			chan->embedded_data_height | EXPECT);
	} else {
		vi4_channel_write(chan, vnc_id, EMBED_X, 0);
		vi4_channel_write(chan, vnc_id, EMBED_Y, 0);
	}
	/*
	 * Set ATOMP_RESERVE to 0 so rctpu won't increment syncpt
	 * for captureInfo. This is copied from nvvi driver.
	 *
	 * If we don't set this register to 0, ATOMP_FE syncpt
	 * will be increment by 2 for each frame
	 */
	vi4_channel_write(chan, vnc_id, ATOMP_RESERVE, 0x0);
	dev_dbg(chan->vi->dev,
		"Create Surface with imgW=%d, imgH=%d, memFmt=%d\n",
		width, height, format);
	return 0;
}

static int tegra_channel_capture_frame_single_thread(
			struct tegra_channel *chan,
			struct tegra_channel_buffer *buf)
{
	struct vb2_v4l2_buffer *vb = &buf->buf;
	struct timespec ts;
	int state = VB2_BUF_STATE_DONE;
	unsigned long flags;
	int err = false;
	int run_captures = 0;
	int i;
	int j;

	if (chan->is_interlaced)
		run_captures = NUM_FIELDS_INTERLACED;
	else
		run_captures = NUM_FIELDS_SINGLE;

	for (j = 0 ; j < run_captures; j++) {
		state = VB2_BUF_STATE_DONE;
		spin_lock_irqsave(&chan->capture_state_lock, flags);
		chan->capture_state = CAPTURE_IDLE;
		spin_unlock_irqrestore(&chan->capture_state_lock, flags);

		if (chan->is_interlaced) {
			if (chan->interlace_type == Interleaved) {
				chan->buffer_offset[0] =
					j * chan->format.bytesperline;
				chan->interlace_bplfactor =
						NUM_FIELDS_INTERLACED;
			} else {
			/* Update the offset according to the field received.
			 * Top field associated with buf sequence 1 should
			 * be written first followed by the bottom field.
			 * Unordered fields will be overwritten
			 */
				if ((j == BOTTOM_FIELD) &&
					(vb->sequence != TOP_FIELD))
					j--;
				chan->buffer_offset[0] = j *
				chan->format.bytesperline * chan->format.height;

				chan->interlace_bplfactor = NUM_FIELDS_SINGLE;
			}
		}

		for (i = 0; i < chan->valid_ports; i++)
			tegra_channel_surface_setup(chan, buf, i);

		if (!chan->bfirst_fstart) {
			err = tegra_channel_set_stream(chan, true);
			if (err < 0)
				return err;

			err = tegra_channel_write_blobs(chan);
			if (err < 0)
				return err;
		}

		for (i = 0; i < chan->valid_ports; i++) {
			vi4_channel_write(chan, chan->vnc_id[i],
						CHANNEL_COMMAND, LOAD);
			vi4_channel_write(chan, chan->vnc_id[i],
				CONTROL, SINGLESHOT | MATCH_STATE_EN);
		}

		/* wait for vi notifier events */
		if (!vi_notify_wait(chan, buf, &ts)) {
			tegra_channel_error_recovery(chan);

			state = VB2_BUF_STATE_REQUEUEING;

			spin_lock_irqsave(&chan->capture_state_lock, flags);
			chan->capture_state = CAPTURE_TIMEOUT;
			spin_unlock_irqrestore(&chan->capture_state_lock,
									flags);
		}

		vi4_check_status(chan);

		spin_lock_irqsave(&chan->capture_state_lock, flags);
		if (chan->capture_state == CAPTURE_IDLE)
			chan->capture_state = CAPTURE_GOOD;
		spin_unlock_irqrestore(&chan->capture_state_lock, flags);
	}

	set_timestamp(buf, &ts);
	tegra_channel_ring_buffer(chan, vb, &ts, state);
	trace_tegra_channel_capture_frame("sof", ts);
	return 0;
}

static int tegra_channel_capture_frame_multi_thread(
			struct tegra_channel *chan,
			struct tegra_channel_buffer *buf)
{
	struct timespec ts;
	unsigned long flags;
	bool is_streaming = atomic_read(&chan->is_streaming);
	int restart_version = 0;
	int err = false;
	int i;

	for (i = 0; i < chan->valid_ports; i++)
		tegra_channel_surface_setup(chan, buf, i);

	restart_version = atomic_read(&chan->restart_version);
	if (!is_streaming ||
		restart_version != chan->capture_version) {

		chan->capture_version = restart_version;
		err = tegra_channel_set_stream(chan, true);
		if (err < 0)
			return err;

		err = tegra_channel_write_blobs(chan);
		if (err < 0)
			return err;
	}

	for (i = 0; i < chan->valid_ports; i++) {
		vi4_channel_write(chan, chan->vnc_id[i],
			CHANNEL_COMMAND, LOAD);
		vi4_channel_write(chan, chan->vnc_id[i],
			CONTROL, SINGLESHOT | MATCH_STATE_EN);
	}

	/* wait for vi notifier events */
	vi_notify_wait(chan, buf, &ts);

	vi4_check_status(chan);

	spin_lock_irqsave(&chan->capture_state_lock, flags);
	if (chan->capture_state != CAPTURE_ERROR)
		chan->capture_state = CAPTURE_GOOD;
	spin_unlock_irqrestore(&chan->capture_state_lock, flags);

	set_timestamp(buf, &ts);

	if (chan->capture_state == CAPTURE_GOOD) {
		/*
		 * Set the buffer version to match
		 * current capture version
		 */
		buf->version = chan->capture_version;
		enqueue_inflight(chan, buf);
	} else {
		release_buffer(chan, buf);
		atomic_inc(&chan->restart_version);
	}

	return 0;
}

static int tegra_channel_capture_frame(struct tegra_channel *chan,
				struct tegra_channel_buffer *buf)
{
	int ret = 0;

	if (chan->low_latency)
		ret = tegra_channel_capture_frame_multi_thread(chan, buf);
	else
		ret = tegra_channel_capture_frame_single_thread(chan, buf);

	return ret;
}

static void tegra_channel_release_frame(struct tegra_channel *chan,
				struct tegra_channel_buffer *buf)
{
	int i;
	int err = 0;
	int restart_version = 0;

	buf->state = VB2_BUF_STATE_DONE;

	/*
	 * If the frame capture was started on a different reset version
	 * than our current version than either a reset is imminent or
	 * it has already happened so don't bother waiting for the frame
	 * to complete.
	 */
	restart_version = atomic_read(&chan->restart_version);
	if (buf->version != restart_version) {
		buf->state = VB2_BUF_STATE_REQUEUEING;
		release_buffer(chan, buf);
		return;
	}

	/*
	 * Wait for ATOMP_FE syncpt
	 *
	 * Use the syncpt max value we set in the capture thread
	 */
	for (i = 0; i < chan->valid_ports; i++) {
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
				chan->syncpt[i][FE_SYNCPT_IDX],
				buf->thresh[i],
				chan->timeout, NULL, NULL);
		if (unlikely(err))
			dev_err(chan->vi->dev,
				"ATOMP_FE syncpt timeout! err = %d\n", err);
		else
			dev_dbg(&chan->video->dev,
			"%s: vi4 got EOF syncpt buf[%p]\n", __func__, buf);
	}

	if (err) {
		buf->state = VB2_BUF_STATE_REQUEUEING;
		atomic_inc(&chan->restart_version);
	}

	release_buffer(chan, buf);
}

static int tegra_channel_kthread_release(void *data)
{
	struct tegra_channel *chan = data;
	struct tegra_channel_buffer *buf;

	set_freezable();

	while (1) {

		try_to_freeze();

		wait_event_interruptible(chan->release_wait,
					 !list_empty(&chan->release) ||
					 kthread_should_stop());

		if (kthread_should_stop())
			break;

		buf = dequeue_inflight(chan);
		if (!buf)
			continue;

		tegra_channel_release_frame(chan, buf);
	}

	return 0;
}

static int tegra_channel_stop_increments(struct tegra_channel *chan)
{
	int i;
	struct tegra_vi4_syncpts_req req = {
		.syncpt_ids = {
			0xffffffff,
			0xffffffff,
			0xffffffff,
		},
		.stream = chan->port[0],
		.vc = chan->virtual_channel,
	};

	/* No need to check errors. There's nothing we could do. */
	for (i = 0; i < chan->valid_ports; i++)
		vi_notify_channel_reset(chan->vnc_id[i], chan->vnc[i], &req);

	return 0;
}

static void tegra_channel_capture_done(struct tegra_channel *chan)
{
	struct timespec ts;
	struct tegra_channel_buffer *buf;
	int state = VB2_BUF_STATE_DONE;
	u32 thresh[TEGRA_CSI_BLOCKS];
	int i, err;

	/* dequeue buffer and return if no buffer exists */
	buf = dequeue_buffer(chan, !chan->low_latency);
	if (!buf)
		return;

	/* make sure to read the last frame out before exit */
	for (i = 0; i < chan->valid_ports; i++) {
		tegra_channel_surface_setup(chan, buf, i);
		vi4_channel_write(chan, chan->vnc_id[i], CHANNEL_COMMAND, LOAD);
		vi4_channel_write(chan, chan->vnc_id[i],
			CONTROL, SINGLESHOT | MATCH_STATE_EN);
	}

	for (i = 0; i < chan->valid_ports; i++) {
		/*
		 * Increment syncpt for ATOMP_FE
		 *
		 * Increment and retrieve ATOMP_FE syncpt max value.
		 * This value will be used to wait for next syncpt
		 */
		struct vi_capture_status status;
		thresh[i] = nvhost_syncpt_incr_max_ext(chan->vi->ndev,
					chan->syncpt[i][FE_SYNCPT_IDX], 1);

		/* Wait for ATOMP_FE syncpt
		 *
		 * This is to make sure we don't exit the capture thread
		 * before the last frame is done writing to memory
		 */
		err = nvhost_syncpt_wait_timeout_ext(chan->vi->ndev,
					chan->syncpt[i][FE_SYNCPT_IDX],
					thresh[i],
					chan->timeout, NULL, NULL);
		if (unlikely(err)) {
			dev_err(chan->vi->dev, "ATOMP_FE syncpt timeout!\n");
			break;
		}

		err = vi_notify_get_capture_status(chan->vnc[i],
				chan->vnc_id[i],
				thresh[i], &status);
		if (unlikely(err)) {
			dev_err(chan->vi->dev,
					"no capture status! err = %d\n", err);
			break;
		}
		ts = ns_to_timespec((s64)status.eof_ts);
	}

	if (err) {
		tegra_channel_error_recovery(chan);
		state = VB2_BUF_STATE_REQUEUEING;
	}

	set_timestamp(buf, &ts);
	/* Mark capture state to IDLE as capture is finished */
	chan->capture_state = CAPTURE_IDLE;

	if (chan->low_latency)
		release_buffer(chan, buf);
	else
		tegra_channel_ring_buffer(chan, &buf->buf, &ts, state);

	trace_tegra_channel_capture_done("eof", ts);
}


static struct v4l2_subdev *find_linked_csi_subdev(struct tegra_csi_device *csi,
	struct tegra_channel *chan)
{
	struct tegra_csi_channel *csi_it;
	int i = 0;

	list_for_each_entry(csi_it, &csi->csi_chans, list) {
		for (i = 0; i < chan->num_subdevs; i++)
			if (chan->subdev[i] == &csi_it->subdev)
				return chan->subdev[i];
	}

	return NULL;
}

static void tegra_channel_error_recovery(struct tegra_channel *chan)
{
	int i = 0;

	struct v4l2_subdev *csi_subdev;
	struct tegra_csi_device *csi = tegra_get_mc_csi();

	dev_warn(chan->vi->dev,
		"%s: attempting to reset the capture channel\n", __func__);

	/* Find connected NvCsi stream subdev */
	csi_subdev = find_linked_csi_subdev(csi, chan);
	if (!csi_subdev) {
		dev_err(chan->vi->dev,
			"%s: failed, unable to find linked csi subdev\n",
			__func__);
		return;
	}

	/* Disable VI notify */
	for (i = 0; i < chan->valid_ports; i++)
		tegra_channel_notify_disable(chan, i);

	/* NvCsi reset/recovery */
	v4l2_subdev_call(csi_subdev, core, sync,
		V4L2_SYNC_EVENT_SUBDEV_ERROR_RECOVER);

	/* Clear VI error state */
	vi4_write(chan, CFG_INTERRUPT_STATUS, 0x1);
	vi4_write(chan, NOTIFY_ERROR, 0x1);

	/* Re-initialize VI and capture context */
	for (i = 0; i < chan->valid_ports; i++)
		tegra_channel_capture_setup(chan, i);
}

static int tegra_channel_kthread_capture_start(void *data)
{
	struct tegra_channel *chan = data;
	struct tegra_channel_buffer *buf;
	int err = 0;

	set_freezable();

	while (1) {

		try_to_freeze();

		wait_event_interruptible(chan->start_wait,
					 !list_empty(&chan->capture) ||
					 kthread_should_stop());

		if (kthread_should_stop())
			break;

		/* source is not streaming if error is non-zero */
		/* wait till kthread stop and dont DeQ buffers */
		if (err)
			continue;

		buf = dequeue_buffer(chan, !chan->low_latency);
		if (!buf)
			continue;

		err = tegra_channel_capture_frame(chan, buf);
	}

	return 0;
}

static void tegra_channel_stop_kthreads(struct tegra_channel *chan)
{
	struct tegra_channel_buffer *buf = NULL;

	mutex_lock(&chan->stop_kthread_lock);
	/* Stop the kthread for capture */
	if (chan->kthread_capture_start) {
		kthread_stop(chan->kthread_capture_start);
		chan->kthread_capture_start = NULL;
	}

	if (chan->low_latency) {
		/* Stop the kthread for release frame */
		if (chan->kthread_release) {
			if (!list_empty(&chan->release)) {
				buf = dequeue_inflight(chan);
				if (buf)
					tegra_channel_release_frame(chan, buf);
			}
			kthread_stop(chan->kthread_release);
			chan->kthread_release = NULL;
		}
	}
	mutex_unlock(&chan->stop_kthread_lock);
}

static int vi4_channel_start_streaming(struct vb2_queue *vq, u32 count)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	/* WAR: With newer version pipe init has some race condition */
	/* TODO: resolve this issue to block userspace not to cleanup media */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	struct media_pipeline *pipe = chan->video->entity.pipe;
#endif
	int ret = 0, i;
	unsigned long flags;
	struct v4l2_subdev *sd;
	struct device_node *node;
	struct sensor_mode_properties *sensor_mode;
	struct camera_common_data *s_data;
	unsigned int emb_buf_size = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	ret = media_entity_pipeline_start(&chan->video->entity, pipe);
	if (ret < 0)
		goto error_pipeline_start;
#endif

	if (chan->bypass) {
		ret = tegra_channel_set_stream(chan, true);
		if (ret < 0)
			goto error_set_stream;

		ret = tegra_channel_write_blobs(chan);
		if (ret < 0)
			goto error_set_stream;
		return ret;
	}

	vi4_init(chan);

	spin_lock_irqsave(&chan->capture_state_lock, flags);
	chan->capture_state = CAPTURE_IDLE;
	spin_unlock_irqrestore(&chan->capture_state_lock, flags);

	if (!chan->pg_mode) {
		sd = chan->subdev_on_csi;
		node = sd->dev->of_node;
		s_data = to_camera_common_data(sd->dev);

		/* get sensor properties from DT */
		if (s_data != NULL && node != NULL) {
			int idx = s_data->mode_prop_idx;

			emb_buf_size = 0;
			if (idx < s_data->sensor_props.num_modes) {
				sensor_mode =
					&s_data->sensor_props.sensor_modes[idx];

				chan->embedded_data_width =
					sensor_mode->image_properties.width;
				chan->embedded_data_height =
					sensor_mode->image_properties.\
					embedded_metadata_height;
				/* rounding up to page size */
				emb_buf_size =
					round_up(chan->embedded_data_width *
						chan->embedded_data_height *
						BPP_MEM,
						PAGE_SIZE);
			}
		}

		/* Allocate buffer for Embedded Data if need to*/
		if (emb_buf_size > chan->vi->emb_buf_size) {
			/*
			 * if old buffer is smaller than what we need,
			 * release the old buffer and re-allocate a bigger
			 * one below
			 */
			if (chan->vi->emb_buf_size > 0) {
				dma_free_coherent(chan->vi->dev,
					chan->vi->emb_buf_size,
					chan->vi->emb_buf_addr, chan->vi->emb_buf);
				chan->vi->emb_buf_size = 0;
			}

			chan->vi->emb_buf_addr =
				dma_alloc_coherent(chan->vi->dev,
					emb_buf_size,
					&chan->vi->emb_buf, GFP_KERNEL);
			if (!chan->vi->emb_buf_addr) {
				dev_err(&chan->video->dev,
						"Can't allocate memory for embedded data\n");
				goto error_capture_setup;
			}
			chan->vi->emb_buf_size = emb_buf_size;
		}
	}

	for (i = 0; i < chan->valid_ports; i++) {
		ret = tegra_channel_capture_setup(chan, i);
		if (ret < 0)
			goto error_capture_setup;
	}

	chan->sequence = 0;
	chan->timeout = msecs_to_jiffies(200);
	if (!chan->low_latency)
		tegra_channel_init_ring_buffer(chan);

	INIT_WORK(&chan->error_work, tegra_channel_error_worker);
	INIT_WORK(&chan->status_work, tegra_channel_status_worker);

	/* Start kthread to capture data to buffer */
	chan->kthread_capture_start = kthread_run(
					tegra_channel_kthread_capture_start,
					chan, chan->video->name);
	if (IS_ERR(chan->kthread_capture_start)) {
		dev_err(&chan->video->dev,
			"failed to run kthread for capture start\n");
		ret = PTR_ERR(chan->kthread_capture_start);
		goto error_capture_setup;
	}

	if (chan->low_latency) {
		/* Start thread to release buffers */
		chan->kthread_release = kthread_run(
					tegra_channel_kthread_release,
					chan, chan->video->name);
		if (IS_ERR(chan->kthread_release)) {
			dev_err(&chan->video->dev,
				"failed to run kthread for release\n");
			ret = PTR_ERR(chan->kthread_release);
			goto error_capture_setup;
		}
	}

	return 0;

error_capture_setup:
	if (!chan->pg_mode) {
		tegra_channel_set_stream(chan, false);
		tegra_channel_write_blobs(chan);
	}
error_set_stream:
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	media_entity_pipeline_stop(&chan->video->entity);
error_pipeline_start:
#endif
	vq->start_streaming_called = 0;
	tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_QUEUED,
		chan->low_latency);

	return ret;
}

static int vi4_channel_stop_streaming(struct vb2_queue *vq)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);
	bool is_streaming = atomic_read(&chan->is_streaming);
	int i;
	int err = 0;

	for (i = 0; i < chan->valid_ports; i++) {
		if (chan->vnc_id[i] == -1)
			return 0;
	}

	cancel_work_sync(&chan->status_work);
	cancel_work_sync(&chan->error_work);

	if (!chan->bypass) {
		tegra_channel_stop_kthreads(chan);
		/* wait for last frame memory write ack */
		if (is_streaming && chan->capture_state == CAPTURE_GOOD)
			tegra_channel_capture_done(chan);
		for (i = 0; i < chan->valid_ports; i++) {
			vi4_channel_write(chan, chan->vnc_id[i], CONTROL, 0);
			vi4_channel_write(chan, chan->vnc_id[i],
				CHANNEL_COMMAND, LOAD);
			tegra_channel_notify_disable(chan, i);
		}
		if (!chan->low_latency) {
			/* free all the ring buffers */
			free_ring_buffers(chan, 0);
		}
		/* dequeue buffers back to app which are in capture queue */
		tegra_channel_queued_buf_done(chan, VB2_BUF_STATE_ERROR,
			chan->low_latency);
	}

	tegra_channel_set_stream(chan, false);

	err = tegra_channel_write_blobs(chan);
	if (err < 0)
		return err;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	media_entity_pipeline_stop(&chan->video->entity);
#endif

	return 0;
}

static int vi4_mfi_work(struct tegra_mc_vi *vi, int channel)
{
	struct tegra_channel *it = NULL;
	int ret = 0;

	/* for vi4, the input argument is the hw channel id*/
	/* search the list and match the hw id */
	list_for_each_entry(it, &vi->vi_chans, list) {
		if (channel == it->vnc_id[0]) {
			ret = v4l2_subdev_call(it->subdev_on_csi, core,
					sync, V4L2_SYNC_EVENT_FOCUS_POS);
			if (ret < 0 && ret != -ENOIOCTLCMD) {
				dev_err(vi->dev,
					"%s:channel failed\n", __func__);
				return ret;
			}
		}
	}

	return ret;
}

int tegra_vi4_power_on(struct tegra_mc_vi *vi)
{
	int ret;

	ret = nvhost_module_busy(vi->ndev);
	if (ret) {
		dev_err(vi->dev, "%s:nvhost module is busy\n", __func__);
		return ret;
	}

	ret = tegra_camera_emc_clk_enable();
	if (ret)
		goto err_emc_enable;

	return 0;

err_emc_enable:
	nvhost_module_idle(vi->ndev);

	return ret;
}

void tegra_vi4_power_off(struct tegra_mc_vi *vi)
{
	tegra_channel_ec_close(vi);
	tegra_camera_emc_clk_disable();
	nvhost_module_idle(vi->ndev);
}

static int vi4_power_on(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi;
	struct tegra_csi_device *csi;

	vi = chan->vi;
	csi = vi->csi;

	/* Use chan->video as identifier of vi4 nvhost_module client
	 * since they are unique per channel
	 */
	ret = nvhost_module_add_client(vi->ndev, chan->video);
	if (ret < 0)
		return ret;

	ret = tegra_vi4_power_on(vi);
	if (ret < 0)
		return ret;

	if (atomic_add_return(1, &chan->power_on_refcnt) == 1) {
		ret = tegra_channel_set_power(chan, 1);
		if (ret < 0) {
			dev_err(vi->dev, "Failed to power on subdevices\n");
			return ret;
		}
	}

	return 0;
}

static void vi4_power_off(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi;
	struct tegra_csi_device *csi;

	vi = chan->vi;
	csi = vi->csi;

	if (atomic_dec_and_test(&chan->power_on_refcnt)) {
		ret = tegra_channel_set_power(chan, 0);
		if (ret < 0)
			dev_err(vi->dev, "Failed to power off subdevices\n");
	}

	tegra_vi4_power_off(vi);
	nvhost_module_remove_client(vi->ndev, chan->video);
}

static void tegra_channel_error_worker(struct work_struct *error_work)
{
	struct tegra_channel *chan;

	chan = container_of(error_work, struct tegra_channel, error_work);

	vi4_power_off(chan);
	tegra_channel_handle_error(chan);
}

static void tegra_channel_notify_error_callback(void *client_data)
{
	struct tegra_channel *chan = (struct tegra_channel *)client_data;

	spin_lock(&chan->capture_state_lock);
	if (chan->capture_state == CAPTURE_GOOD)
		chan->capture_state = CAPTURE_ERROR;
	else {
		spin_unlock(&chan->capture_state_lock);
		return;
	}
	spin_unlock(&chan->capture_state_lock);

	schedule_work(&chan->error_work);
}

static void vi4_stride_align(unsigned int *bpl)
{
	*bpl = ((*bpl + (RM_SURFACE_ALIGNMENT) - 1) &
			~((RM_SURFACE_ALIGNMENT) - 1));
}

struct tegra_vi_fops vi4_fops = {
	.vi_power_on = vi4_power_on,
	.vi_power_off = vi4_power_off,
	.vi_start_streaming = vi4_channel_start_streaming,
	.vi_stop_streaming = vi4_channel_stop_streaming,
	.vi_setup_queue = vi4_channel_setup_queue,
	.vi_add_ctrls = vi4_add_ctrls,
	.vi_init_video_formats = vi4_init_video_formats,
	.vi_mfi_work = vi4_mfi_work,
	.vi_stride_align = vi4_stride_align,

};
