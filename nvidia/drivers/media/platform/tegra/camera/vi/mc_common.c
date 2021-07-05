/*
 * Tegra Video Input device common APIs
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <media/tegra_v4l2_camera.h>
#include <media/camera_common.h>
#include <media/v4l2-event.h>
#include <media/tegra_camera_platform.h>
#include <media/vi.h>
#include <media/vi2_registers.h>

#include "dev.h"
#include "host1x/host1x.h"

static struct tegra_mc_vi *tegra_mcvi;

struct tegra_mc_vi *tegra_get_mc_vi(void)
{
	return tegra_mcvi;
}
EXPORT_SYMBOL(tegra_get_mc_vi);

/* In TPG mode, VI only support 2 formats */
static void vi_tpg_fmts_bitmap_init(struct tegra_channel *chan)
{
	int index;

	bitmap_zero(chan->fmts_bitmap, MAX_FORMAT_NUM);

	index = tegra_core_get_idx_by_code(chan,
			MEDIA_BUS_FMT_SRGGB10_1X10, 0);
	bitmap_set(chan->fmts_bitmap, index, 1);

	index = tegra_core_get_idx_by_code(chan,
			MEDIA_BUS_FMT_RGB888_1X32_PADHI, 0);
	bitmap_set(chan->fmts_bitmap, index, 1);
}

/* -----------------------------------------------------------------------------
 * Media Controller and V4L2
 */

static const char *const vi_pattern_strings[] = {
	"Disabled",
	"Black/White Direct Mode",
	"Color Patch Mode",
};

static int vi_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegra_mc_vi *vi = container_of(ctrl->handler, struct tegra_mc_vi,
					   ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		/*
		 * TPG control is only avaiable to TPG driver,
		 * it can't be changed to 0 to disable TPG mode.
		 */
		if (ctrl->val) {
			dev_info(&vi->ndev->dev, "Set TPG mode to %d\n",
				 ctrl->val);
			vi->pg_mode = ctrl->val;
		}
		break;
	default:
		dev_err(vi->dev, "%s:Not valid ctrl\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops vi_ctrl_ops = {
	.s_ctrl	= vi_s_ctrl,
};

void tegra_vi_v4l2_cleanup(struct tegra_mc_vi *vi)
{
	v4l2_ctrl_handler_free(&vi->ctrl_handler);
	v4l2_device_unregister(&vi->v4l2_dev);
	if (!vi->pg_mode)
		media_device_unregister(&vi->media_dev);
}
EXPORT_SYMBOL(tegra_vi_v4l2_cleanup);

static void tegra_vi_notify(struct v4l2_subdev *sd,
					  unsigned int notification, void *arg)
{
	struct tegra_mc_vi *vi = container_of(sd->v4l2_dev,
			struct tegra_mc_vi, v4l2_dev);
	const struct v4l2_event *ev = arg;
	unsigned i;
	struct tegra_channel *chan;

	if (notification != V4L2_DEVICE_NOTIFY_EVENT)
		return;

	list_for_each_entry(chan, &vi->vi_chans, list) {
		for (i = 0; i < chan->num_subdevs; i++)
			if (sd == chan->subdev[i]) {
				v4l2_event_queue(chan->video, arg);
				if (ev->type == V4L2_EVENT_SOURCE_CHANGE &&
						vb2_is_streaming(&chan->queue))
					vb2_queue_error(&chan->queue);
			}
	}
}

int tegra_vi_v4l2_init(struct tegra_mc_vi *vi)
{
	int ret;

	vi->media_dev.dev = vi->dev;
	strlcpy(vi->media_dev.model, "NVIDIA Tegra Video Input Device",
		sizeof(vi->media_dev.model));
	vi->media_dev.hw_revision = 3;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	media_device_init(&vi->media_dev);
#endif

	ret = media_device_register(&vi->media_dev);
	if (ret < 0) {
		dev_err(vi->dev,
			"media device registration failed (%d)\n",
			ret);
		return ret;
	}

	mutex_init(&vi->bw_update_lock);
	vi->v4l2_dev.mdev = &vi->media_dev;
	vi->v4l2_dev.notify = tegra_vi_notify;
	ret = v4l2_device_register(vi->dev, &vi->v4l2_dev);
	if (ret < 0) {
		dev_err(vi->dev, "V4L2 device registration failed (%d)\n",
			ret);
		goto register_error;
	}

	return 0;

register_error:
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	media_device_cleanup(&vi->media_dev);
#endif
	media_device_unregister(&vi->media_dev);
	return ret;
}

static int vi_parse_dt(struct tegra_mc_vi *vi, struct platform_device *dev)
{
	int err = 0;
	int num_channels = 0;
	int i;
	struct tegra_channel *item;
	struct device_node *node = dev->dev.of_node;

	err = of_property_read_u32(node, "num-channels", &num_channels);
	if (err) {
		dev_dbg(&dev->dev,
			"Failed to find num of channels, set to 0\n");
		num_channels = 0;
	}
	vi->num_channels = num_channels;
	for (i = 0; i < num_channels; i++) {
		item = devm_kzalloc(vi->dev, sizeof(*item), GFP_KERNEL);
		if (!item)
			return -ENOMEM;
		item->id = i;
		list_add_tail(&item->list, &vi->vi_chans);
	}

	return 0;
}

static void set_vi_register_base(struct tegra_mc_vi *mc_vi,
			void __iomem *regbase)
{
	mc_vi->iomem = regbase;
}
int tpg_vi_media_controller_init(struct tegra_mc_vi *mc_vi, int pg_mode)
{
	int err = 0, i;
	struct tegra_channel *item;
	const unsigned int num_pre_channels = mc_vi->num_channels;

	/* Allocate TPG channel */
	v4l2_ctrl_handler_init(&mc_vi->ctrl_handler, 1);
	mc_vi->pattern = v4l2_ctrl_new_std_menu_items(&mc_vi->ctrl_handler,
			&vi_ctrl_ops, V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(vi_pattern_strings) - 1,
			0, mc_vi->pg_mode, vi_pattern_strings);

	if (mc_vi->ctrl_handler.error) {
		dev_err(mc_vi->dev, "failed to add controls\n");
		err = mc_vi->ctrl_handler.error;
		goto ctrl_error;
	}

	mc_vi->tpg_start = NULL;
	for (i = 0; i < mc_vi->csi->num_tpg_channels; i++) {
		item = devm_kzalloc(mc_vi->dev, sizeof(*item), GFP_KERNEL);
		if (!item)
			goto channel_init_error;

		item->id = num_pre_channels + i;
		item->pg_mode = pg_mode;
		item->vi = mc_vi;

		err = tegra_channel_init(item);
		if (err) {
			devm_kfree(mc_vi->dev, item);
			goto channel_init_error;
		}

		/* Allocate video_device */
		err = tegra_channel_init_video(item);
		if (err < 0) {
			devm_kfree(mc_vi->dev, item);
			dev_err(&item->video->dev, "failed to allocate video device %s\n",
				item->video->name);
			goto channel_init_error;
		}

		err = video_register_device(item->video, VFL_TYPE_GRABBER, -1);
		if (err < 0) {
			devm_kfree(mc_vi->dev, item);
			video_device_release(item->video);
			dev_err(&item->video->dev, "failed to register %s\n",
				item->video->name);
			goto channel_init_error;
		}

		vi_tpg_fmts_bitmap_init(item);
		/* only inited tpg channels are added */
		list_add_tail(&item->list, &mc_vi->vi_chans);
		if (mc_vi->tpg_start == NULL)
			mc_vi->tpg_start = item;
	}
	mc_vi->num_channels += mc_vi->csi->num_tpg_channels;

	err = tegra_vi_tpg_graph_init(mc_vi);
	if (err)
		goto channel_init_error;

	return err;

channel_init_error:
	dev_err(mc_vi->dev, "%s: channel init failed\n", __func__);
	if (!mc_vi->tpg_start)
		tpg_vi_media_controller_cleanup(mc_vi);
	return err;
ctrl_error:
	v4l2_ctrl_handler_free(&mc_vi->ctrl_handler);
	dev_err(mc_vi->dev, "%s: v2l4_ctl error\n", __func__);
	return err;
}
EXPORT_SYMBOL(tpg_vi_media_controller_init);

void tpg_vi_media_controller_cleanup(struct tegra_mc_vi *mc_vi)
{
	struct tegra_channel *item;
	struct tegra_channel *itemn;

	list_for_each_entry_safe(item, itemn, &mc_vi->vi_chans, list) {
		if (!item->pg_mode)
			continue;
		if (item->video->cdev != NULL)
			video_unregister_device(item->video);
		tegra_channel_cleanup(item);
		list_del(&item->list);
		devm_kfree(mc_vi->dev, item);
		/* decrement media device entity count */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
		mc_vi->media_dev.entity_id--;
#endif
		mc_vi->num_channels--;
	}
	mc_vi->tpg_start = NULL;
	v4l2_ctrl_handler_free(&mc_vi->ctrl_handler);
}
EXPORT_SYMBOL(tpg_vi_media_controller_cleanup);

int tegra_vi_media_controller_init(struct tegra_mc_vi *mc_vi,
				   struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = (struct nvhost_device_data *)
		platform_get_drvdata(pdev);

	if (!pdata)
		return -EINVAL;
	set_vi_register_base(mc_vi, pdata->aperture[0]);

	mc_vi->ndev = pdev;
	mc_vi->dev = &pdev->dev;
	INIT_LIST_HEAD(&mc_vi->vi_chans);
	mutex_init(&mc_vi->mipical_lock);

	err = vi_parse_dt(mc_vi, pdev);
	if (err)
		goto mc_init_fail;

	tegra_mcvi = mc_vi;

	err = tegra_vi_v4l2_init(mc_vi);
	if (err < 0)
		goto mc_init_fail;

	/*
	 * if there is no vi channels listed in DT,
	 * no need to init the channel and graph
	 */
	if (mc_vi->num_channels == 0)
		return 0;

	/* Init Tegra VI channels */
	err = tegra_vi_channels_init(mc_vi);
	if (err < 0) {
		dev_err(&pdev->dev, "Init channel failed\n");
		goto channels_error;
	}

	/* Setup media links between VI and external sensor subdev. */
	err = tegra_vi_graph_init(mc_vi);
	if (err < 0)
		goto graph_error;

	return 0;

graph_error:
	tegra_vi_channels_cleanup(mc_vi);
channels_error:
	tegra_vi_v4l2_cleanup(mc_vi);
mc_init_fail:
	dev_err(&pdev->dev, "%s: failed\n", __func__);
	return err;
}
EXPORT_SYMBOL(tegra_vi_media_controller_init);

void tegra_vi_media_controller_cleanup(struct tegra_mc_vi *mc_vi)
{
	tegra_vi_channels_unregister(mc_vi);
	tegra_vi_graph_cleanup(mc_vi);
	tegra_vi_channels_cleanup(mc_vi);
	tegra_vi_v4l2_cleanup(mc_vi);
	tegra_mcvi = NULL;
}
EXPORT_SYMBOL(tegra_vi_media_controller_cleanup);
