/*
 * NVIDIA Media controller graph management
 *
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra_pm_domains.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <media/v4l2-fwnode.h>
#else
#include <media/v4l2-of.h>
#endif
#include <media/tegra_v4l2_camera.h>
#include <media/mc_common.h>
#include <media/csi.h>

#include "nvcsi/nvcsi.h"

/* -----------------------------------------------------------------------------
 * Graph Management
 */

static struct tegra_vi_graph_entity *
tegra_vi_graph_find_entity(struct tegra_channel *chan,
		       const struct device_node *node)
{
	struct tegra_vi_graph_entity *entity;

	list_for_each_entry(entity, &chan->entities, list) {
		if (entity->node == node)
			return entity;
	}

	return NULL;
}

static int tegra_vi_graph_build_one(struct tegra_channel *chan,
				    struct tegra_vi_graph_entity *entity)
{
	u32 link_flags = MEDIA_LNK_FL_ENABLED;
	struct media_entity *local;
	struct media_entity *remote;
	struct media_pad *local_pad;
	struct media_pad *remote_pad;
	struct tegra_vi_graph_entity *ent;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	struct v4l2_fwnode_link link;
#else
	struct v4l2_of_link link;
#endif
	struct device_node *ep = NULL;
	struct device_node *next;
	int ret = 0;

	if (!entity->subdev) {
		dev_err(chan->vi->dev, "%s:No subdev under entity, skip linking\n",
				__func__);
		return 0;
	}

	local = entity->entity;
	dev_dbg(chan->vi->dev, "creating links for entity %s\n", local->name);

	do {
		/* Get the next endpoint and parse its link. */
		next = of_graph_get_next_endpoint(entity->node, ep);
		if (next == NULL)
			break;

		ep = next;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		dev_dbg(chan->vi->dev, "processing endpoint %pOF\n",
				ep);
		ret = v4l2_fwnode_parse_link(of_fwnode_handle(ep), &link);
		if (ret < 0) {
			dev_err(chan->vi->dev,
			"failed to parse link for %pOF\n", ep);
			continue;
		}
#else
		dev_dbg(chan->vi->dev, "processing endpoint %s\n",
				ep->full_name);
		ret = v4l2_of_parse_link(ep, &link);
		if (ret < 0) {
			dev_err(chan->vi->dev, "failed to parse link for %s\n",
				ep->full_name);
			continue;
		}
#endif

		if (link.local_port >= local->num_pads) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			dev_err(chan->vi->dev,
				"invalid port number %u for %pOF\n",
				link.local_port, to_of_node(link.local_node));
			v4l2_fwnode_put_link(&link);
#else
			dev_err(chan->vi->dev, "invalid port number %u on %s\n",
				link.local_port, link.local_node->full_name);
			v4l2_of_put_link(&link);
#endif
			ret = -EINVAL;
			break;
		}

		local_pad = &local->pads[link.local_port];

		/* Skip sink ports, they will be processed from the other end of
		 * the link.
		 */
		if (local_pad->flags & MEDIA_PAD_FL_SINK) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			dev_dbg(chan->vi->dev, "skipping sink port %pOF:%u\n",
				to_of_node(link.local_node), link.local_port);
			v4l2_fwnode_put_link(&link);
#else
			dev_dbg(chan->vi->dev, "skipping sink port %s:%u\n",
				link.local_node->full_name, link.local_port);
			v4l2_of_put_link(&link);
#endif
			continue;
		}

		/* Skip channel entity , they will be processed separately. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		if (link.remote_node == of_fwnode_handle(chan->vi->dev->of_node)) {
			dev_dbg(chan->vi->dev, "skipping channel port %pOF:%u\n",
				to_of_node(link.local_node), link.local_port);
			v4l2_fwnode_put_link(&link);
			continue;
		}
#else
		if (link.remote_node == chan->vi->dev->of_node) {
			dev_dbg(chan->vi->dev, "skipping channel port %s:%u\n",
				link.local_node->full_name, link.local_port);
			v4l2_of_put_link(&link);
			continue;
		}
#endif


		/* Find the remote entity. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		ent = tegra_vi_graph_find_entity(chan, to_of_node(link.remote_node));
#else
		ent = tegra_vi_graph_find_entity(chan, link.remote_node);
#endif
		if (ent == NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			dev_err(chan->vi->dev, "no entity found for %pOF\n",
				to_of_node(link.remote_node));
			v4l2_fwnode_put_link(&link);
#else
			dev_err(chan->vi->dev, "no entity found for %s\n",
				link.remote_node->full_name);
			v4l2_of_put_link(&link);
#endif
			ret = -EINVAL;
			break;
		}

		remote = ent->entity;

		if (link.remote_port >= remote->num_pads) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			dev_err(chan->vi->dev, "invalid port number %u on %pOF\n",
				link.remote_port, to_of_node(link.remote_node));
			v4l2_fwnode_put_link(&link);
#else
			dev_err(chan->vi->dev, "invalid port number %u on %s\n",
				link.remote_port, link.remote_node->full_name);
			v4l2_of_put_link(&link);
#endif
			ret = -EINVAL;
			break;
		}

		remote_pad = &remote->pads[link.remote_port];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			v4l2_fwnode_put_link(&link);
#else
			v4l2_of_put_link(&link);
#endif

		/* Create the media link. */
		dev_dbg(chan->vi->dev, "creating %s:%u -> %s:%u link\n",
			local->name, local_pad->index,
			remote->name, remote_pad->index);

		ret = tegra_media_create_link(local, local_pad->index, remote,
				remote_pad->index, link_flags);
		if (ret < 0) {
			dev_err(chan->vi->dev,
				"failed to create %s:%u -> %s:%u link\n",
				local->name, local_pad->index,
				remote->name, remote_pad->index);
			break;
		}
	} while (next);

	return ret;
}

static int tegra_vi_graph_build_links(struct tegra_channel *chan)
{
	u32 link_flags = MEDIA_LNK_FL_ENABLED;
	struct media_entity *source;
	struct media_entity *sink;
	struct media_pad *source_pad;
	struct media_pad *sink_pad;
	struct tegra_vi_graph_entity *ent;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	struct v4l2_fwnode_link link;
#else
	struct v4l2_of_link link;
#endif
	struct device_node *ep = NULL;
	int ret = 0;

	dev_dbg(chan->vi->dev, "creating links for channels\n");

	/* Device not registered */
	if (!chan->init_done)
		return -EINVAL;

	ep = chan->endpoint_node;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	dev_dbg(chan->vi->dev, "processing endpoint %pOF\n", ep);
	ret = v4l2_fwnode_parse_link(of_fwnode_handle(ep), &link);
	if (ret < 0) {
		dev_err(chan->vi->dev, "failed to parse link for %pOF\n",
			ep);
		return -EINVAL;
	}
#else
	dev_dbg(chan->vi->dev, "processing endpoint %s\n", ep->full_name);
	ret = v4l2_of_parse_link(ep, &link);
	if (ret < 0) {
		dev_err(chan->vi->dev, "failed to parse link for %s\n",
			ep->full_name);
		return -EINVAL;
	}
#endif

	if (link.local_port >= chan->vi->num_channels) {
		dev_err(chan->vi->dev, "wrong channel number for port %u\n",
			link.local_port);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			v4l2_fwnode_put_link(&link);
#else
			v4l2_of_put_link(&link);
#endif
		return  -EINVAL;
	}

	dev_dbg(chan->vi->dev, "creating link for channel %s\n",
		chan->video->name);

	/* Find the remote entity. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	ent = tegra_vi_graph_find_entity(chan, to_of_node(link.remote_node));
	if (ent == NULL) {
		dev_err(chan->vi->dev, "no entity found for %pOF\n",
			to_of_node(link.remote_node));
			v4l2_fwnode_put_link(&link);
		return -EINVAL;
	}
#else
	ent = tegra_vi_graph_find_entity(chan, link.remote_node);
	if (ent == NULL) {
		dev_err(chan->vi->dev, "no entity found for %s\n",
			link.remote_node->full_name);
			v4l2_of_put_link(&link);
		return -EINVAL;
	}
#endif

	if (ent->entity == NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		dev_err(chan->vi->dev, "entity not bounded %pOF\n",
			to_of_node(link.remote_node));
		v4l2_fwnode_put_link(&link);
#else
		dev_err(chan->vi->dev, "entity not bounded %s\n",
			link.remote_node->full_name);
		v4l2_of_put_link(&link);
#endif
		return -EINVAL;
	}

	source = ent->entity;
	source_pad = &source->pads[link.remote_port];
	sink = &chan->video->entity;
	sink_pad = &chan->pad;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			v4l2_fwnode_put_link(&link);
#else
			v4l2_of_put_link(&link);
#endif

	/* Create the media link. */
	dev_dbg(chan->vi->dev, "creating %s:%u -> %s:%u link\n",
		source->name, source_pad->index,
		sink->name, sink_pad->index);

	ret = tegra_media_create_link(source, source_pad->index,
				       sink, sink_pad->index,
				       link_flags);
	if (ret < 0) {
		dev_err(chan->vi->dev,
			"failed to create %s:%u -> %s:%u link\n",
			source->name, source_pad->index,
			sink->name, sink_pad->index);
		return -EINVAL;
	}

	ret = tegra_channel_init_subdevices(chan);
	if (ret < 0) {
		dev_err(chan->vi->dev, "Failed to initialize sub-devices\n");
		return -EINVAL;
	}

	return 0;
}

static void tegra_vi_graph_remove_links(struct tegra_channel *chan)
{
	struct tegra_vi_graph_entity *entity;

	/* remove entity links and subdev for nvcsi */
	entity = list_first_entry(&chan->entities,
			struct tegra_vi_graph_entity, list);
	if (entity->entity != NULL) {
		media_entity_remove_links(entity->entity);
		video_unregister_device(entity->subdev->devnode);
	}

	/* remove video node for vi */
	tegra_channel_remove_subdevices(chan);
}

static int tegra_vi_graph_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct tegra_channel *chan =
		container_of(notifier, struct tegra_channel, notifier);
	struct tegra_vi_graph_entity *entity;
	int ret;

	dev_dbg(chan->vi->dev, "notify complete, all subdevs registered\n");

	/* Allocate video_device */
	ret = tegra_channel_init_video(chan);
	if (ret < 0) {
		dev_err(chan->vi->dev, "failed to allocate video device %s\n",
			chan->video->name);
		return ret;
	}

	ret = video_register_device(chan->video, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		dev_err(chan->vi->dev, "failed to register %s\n",
			chan->video->name);
		goto register_device_error;
	}

	/* Create links for every entity. */
	list_for_each_entry(entity, &chan->entities, list) {
		if (entity->entity != NULL) {
			ret = tegra_vi_graph_build_one(chan, entity);
			if (ret < 0)
				goto graph_error;
		}
	}

	/* Create links for channels */
	ret = tegra_vi_graph_build_links(chan);
	if (ret < 0)
		goto graph_error;

	ret = v4l2_device_register_subdev_nodes(&chan->vi->v4l2_dev);
	if (ret < 0) {
		dev_err(chan->vi->dev, "failed to register subdev nodes\n");
		goto graph_error;
	}

	chan->link_status++;

	return 0;

graph_error:
	video_unregister_device(chan->video);
register_device_error:
	video_device_release(chan->video);

	return ret;
}

static int tegra_vi_graph_notify_bound(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *subdev,
				   struct v4l2_async_subdev *asd)
{
	struct tegra_channel *chan =
		container_of(notifier, struct tegra_channel, notifier);
	struct tegra_vi_graph_entity *entity;

	/* Locate the entity corresponding to the bound subdev and store the
	 * subdev pointer.
	 */
	list_for_each_entry(entity, &chan->entities, list) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		if (entity->node != to_of_node(subdev->dev->fwnode) &&
			entity->node != to_of_node(subdev->fwnode))
			continue;
#else
		if (entity->node != subdev->dev->of_node &&
			entity->node != subdev->of_node)
			continue;
#endif

		if (entity->subdev) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			dev_err(chan->vi->dev, "duplicate subdev for node %pOF\n",
				entity->node);
#else
			dev_err(chan->vi->dev, "duplicate subdev for node %s\n",
				entity->node->full_name);
#endif
			return -EINVAL;
		}

		dev_info(chan->vi->dev, "subdev %s bound\n", subdev->name);
		entity->entity = &subdev->entity;
		entity->subdev = subdev;
		chan->subdevs_bound++;
		return 0;
	}

	dev_err(chan->vi->dev, "no entity for subdev %s\n", subdev->name);
	return -EINVAL;
}

static void tegra_vi_graph_notify_unbind(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *subdev,
				   struct v4l2_async_subdev *asd)
{
	struct tegra_channel *chan =
		container_of(notifier, struct tegra_channel, notifier);
	struct tegra_vi_graph_entity *entity;

	/* cleanup for complete */
	if (chan->link_status) {
		tegra_vi_graph_remove_links(chan);
		tegra_channel_cleanup_video(chan);
		chan->link_status--;
	}

	/* cleanup for bound */
	list_for_each_entry(entity, &chan->entities, list) {
		if (entity->subdev == subdev) {
			/* remove subdev node */
			chan->subdevs_bound--;
			entity->subdev = NULL;
			entity->entity = NULL;
			dev_info(chan->vi->dev, "subdev %s unbind\n",
				subdev->name);
			break;
		}
	}
}

void tegra_vi_graph_cleanup(struct tegra_mc_vi *vi)
{
	struct tegra_vi_graph_entity *entityp;
	struct tegra_vi_graph_entity *entity;
	struct tegra_channel *chan;

	list_for_each_entry(chan, &vi->vi_chans, list) {
		v4l2_async_notifier_unregister(&chan->notifier);

		list_for_each_entry_safe(entity, entityp,
					&chan->entities, list) {
			of_node_put(entity->node);
			list_del(&entity->list);
		}
	}
}

int tegra_vi_get_port_info(struct tegra_channel *chan,
			struct device_node *node, unsigned int index)
{
	struct device_node *ep = NULL;
	struct device_node *ports;
	struct device_node *port;
	int value = 0xFFFF;
	int ret = 0, i;

	ports = of_get_child_by_name(node, "ports");
	if (ports == NULL)
		ports = node;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;

		ret = of_property_read_u32(port, "reg", &value);
		if (ret < 0)
			continue;

		if (value != index)
			continue;

		for_each_child_of_node(port, ep) {
			if (!ep->name || of_node_cmp(ep->name, "endpoint"))
				continue;

			/* Get virtual channel id */
			ret = of_property_read_u32(ep, "vc-id", &value);

			/* vc-id is optional, default is 0 */
			chan->virtual_channel = (ret < 0) ? 0 : value;

			/* Consider max simultaneous sensor streams to be 16 */
			if (value > 16) {
				dev_err(chan->vi->dev, "vc id >16!\n");
				return -EINVAL;
			}

			/* Get CSI port */
			ret = of_property_read_u32(ep, "port-index", &value);
			if (ret < 0)
				dev_err(chan->vi->dev, "port index error\n");
			chan->port[0] = value;

			if (value > NVCSI_PORT_H) {
				dev_err(chan->vi->dev, "port index >%d!\n",
					NVCSI_PORT_H);
				return -EINVAL;
			}

			/* Get number of data lanes for the endpoint */
			ret = of_property_read_u32(ep, "bus-width", &value);
			if (ret < 0)
				dev_err(chan->vi->dev, "num lanes error\n");
			chan->numlanes = value;

			if (value > 12) {
				dev_err(chan->vi->dev, "num lanes >12!\n");
				return -EINVAL;
			}
			/*
			 * for numlanes greater than 4 multiple CSI bricks
			 * are needed to capture the image, the logic below
			 * checks for numlanes > 4 and add a new CSI brick
			 * as a valid port. Loops around the three CSI
			 * bricks to add as many ports necessary.
			 */
			value -= 4;
			for (i = 1; value > 0; i++, value -= 4) {
				int next_port = chan->port[i-1] + 2;

				next_port = (next_port % (NVCSI_PORT_H + 1));
				chan->port[i] = next_port;
			}
		}
	}

	return ret;
}

static int tegra_vi_graph_parse_one(struct tegra_channel *chan,
				struct device_node *node)
{
	struct device_node *ep = NULL;
	struct device_node *next;
	struct device_node *remote = NULL;
	struct tegra_vi_graph_entity *entity;
	int ret = 0;

	dev_dbg(chan->vi->dev, "parsing node %s\n", node->full_name);
	/* Parse all the remote entities and put them into the list */
	do {
		next = of_graph_get_next_endpoint(node, ep);
		if (next == NULL || !of_device_is_available(next))
			break;
		ep = next;

		dev_dbg(chan->vi->dev, "handling endpoint %s\n", ep->full_name);

		remote = of_graph_get_remote_port_parent(ep);
		if (!remote) {
			ret = -EINVAL;
			break;
		}

		/* skip the vi of_node and duplicated entities */
		if (remote == chan->vi->dev->of_node ||
		    tegra_vi_graph_find_entity(chan, remote) ||
		    !of_device_is_available(remote))
			continue;

		entity = devm_kzalloc(chan->vi->dev, sizeof(*entity),
				GFP_KERNEL);
		if (entity == NULL) {
			ret = -ENOMEM;
			break;
		}

		entity->node = remote;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		entity->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		entity->asd.match.fwnode.fwnode = of_fwnode_handle(remote);
#else
		entity->asd.match_type = V4L2_ASYNC_MATCH_OF;
		entity->asd.match.of.node = remote;
#endif
		list_add_tail(&entity->list, &chan->entities);
		chan->num_subdevs++;

		/* Find remote entities, which are linked to this entity */
		ret = tegra_vi_graph_parse_one(chan, entity->node);
		if (ret < 0)
			break;
	} while (next);

	return ret;
}

int tegra_vi_tpg_graph_init(struct tegra_mc_vi *mc_vi)
{
	int err = 0;
	u32 link_flags = MEDIA_LNK_FL_ENABLED;
	struct tegra_csi_device *csi = mc_vi->csi;
	struct tegra_channel *vi_it;
	struct tegra_csi_channel *csi_it;

	if (!csi) {
		dev_err(mc_vi->dev, "CSI is NULL\n");
		return -EINVAL;
	}
	mc_vi->num_subdevs = mc_vi->num_channels;
	vi_it = mc_vi->tpg_start;
	csi_it = csi->tpg_start;

	list_for_each_entry_from(vi_it, &mc_vi->vi_chans, list) {
		/* Device not registered */
		if (!vi_it->init_done)
			continue;

		list_for_each_entry_from(csi_it, &csi->csi_chans, list) {
			struct media_entity *source = &csi_it->subdev.entity;
			struct media_entity *sink = &vi_it->video->entity;
			struct media_pad *source_pad = csi_it->pads;
			struct media_pad *sink_pad = &vi_it->pad;

			vi_it->bypass = 0;
			err = v4l2_device_register_subdev(&mc_vi->v4l2_dev,
					&csi_it->subdev);
			if (err) {
				dev_err(mc_vi->dev,
					"%s:Fail to register subdev\n",
					__func__);
				goto register_fail;
			}
			dev_dbg(mc_vi->dev, "creating %s:%u -> %s:%u link\n",
				source->name, source_pad->index,
				sink->name, sink_pad->index);

			err = tegra_media_create_link(source, source_pad->index,
					sink, sink_pad->index, link_flags);
			if (err < 0) {
				dev_err(mc_vi->dev,
					"failed to create %s:%u -> %s:%u link\n",
					source->name, source_pad->index,
					sink->name, sink_pad->index);
				goto register_fail;
			}
			err = tegra_channel_init_subdevices(vi_it);
			if (err) {
				dev_err(mc_vi->dev,
					"%s:Init subdevice error\n", __func__);
				goto register_fail;
			}
			csi_it = list_next_entry(csi_it, list);
			break;
		}
	}

	return 0;
register_fail:
	csi_it = csi->tpg_start;
	list_for_each_entry_from(csi_it, &csi->csi_chans, list)
		v4l2_device_unregister_subdev(&csi_it->subdev);
	return err;
}

int tegra_vi_graph_init(struct tegra_mc_vi *vi)
{
	struct tegra_vi_graph_entity *entity;
	struct v4l2_async_subdev **subdevs = NULL;
	unsigned int num_subdevs = 0;
	int ret = 0, i;
	struct device_node *ep = NULL;
	struct device_node *next;
	struct device_node *remote = NULL;
	struct tegra_channel *chan;

	/*
	 * Walk the links to parse the full graph. Each struct tegra_channel
	 * in vi->vi_chans points to each endpoint of the composite node.
	 * Thus parse the remote entity for each endpoint in turn.
	 * Each channel will register a v4l2 async notifier, this makes graph
	 * init independent between vi_chans. There we can skip the current
	 * channel in case of something wrong during graph parsing and try
	 * the next channel. Return error only if memory allocation is failed.
	 */
	chan = list_first_entry(&vi->vi_chans, struct tegra_channel, list);
	do {
		/* Get the next endpoint and parse its entities. */
		next = of_graph_get_next_endpoint(vi->dev->of_node, ep);
		if (next == NULL)
			break;

		ep = next;

		if (!of_device_is_available(ep)) {
			dev_info(vi->dev, "ep of_device is not enabled %s.\n",
					ep->full_name);
			if (list_is_last(&chan->list, &vi->vi_chans))
				break;
			/* Try the next channel */
			chan = list_next_entry(chan, list);
			continue;
		}

		chan->endpoint_node = ep;
		entity = devm_kzalloc(vi->dev, sizeof(*entity), GFP_KERNEL);
		if (entity == NULL) {
			ret = -ENOMEM;
			goto done;
		}

		dev_dbg(vi->dev, "handling endpoint %s\n", ep->full_name);
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote) {
			dev_info(vi->dev, "cannot find remote port parent\n");
			if (list_is_last(&chan->list, &vi->vi_chans))
				break;
			/* Try the next channel */
			chan = list_next_entry(chan, list);
			continue;
		}

		if (!of_device_is_available(remote)) {
			dev_info(vi->dev, "remote of_device is not enabled %s.\n",
					ep->full_name);
			if (list_is_last(&chan->list, &vi->vi_chans))
				break;
			/* Try the next channel */
			chan = list_next_entry(chan, list);
			continue;
		}

		/* Add the remote entity of this endpoint */
		entity->node = remote;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		entity->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		entity->asd.match.fwnode.fwnode = of_fwnode_handle(remote);
#else
		entity->asd.match_type = V4L2_ASYNC_MATCH_OF;
		entity->asd.match.of.node = remote;
#endif
		list_add_tail(&entity->list, &chan->entities);
		chan->num_subdevs++;

		/* Parse and add entities on this enpoint/channel */
		ret = tegra_vi_graph_parse_one(chan, entity->node);
		if (ret < 0) {
			dev_info(vi->dev, "graph parse error: %s.\n",
					entity->node->full_name);
			if (list_is_last(&chan->list, &vi->vi_chans))
				break;
			/* Try the next channel */
			chan = list_next_entry(chan, list);
			continue;
		}

		num_subdevs = chan->num_subdevs;
		subdevs = devm_kzalloc(vi->dev, sizeof(*subdevs) * num_subdevs,
			       GFP_KERNEL);
		if (subdevs == NULL) {
			ret = -ENOMEM;
			goto done;
		}

		i = 0;
		list_for_each_entry(entity, &chan->entities, list)
			subdevs[i++] = &entity->asd;

		chan->notifier.subdevs = subdevs;
		chan->notifier.num_subdevs = num_subdevs;
		chan->notifier.bound = tegra_vi_graph_notify_bound;
		chan->notifier.unbind = tegra_vi_graph_notify_unbind;
		chan->notifier.complete = tegra_vi_graph_notify_complete;
		chan->link_status = 0;
		chan->subdevs_bound = 0;

		/* Register the async notifier for this channel */
		ret = v4l2_async_notifier_register(&vi->v4l2_dev,
					&chan->notifier);
		if (ret < 0) {
			dev_err(vi->dev, "notifier registration failed\n");
			goto done;
		}

		if (list_is_last(&chan->list, &vi->vi_chans))
			break;
		/* One endpoint for each vi channel, go with the next channel */
		chan = list_next_entry(chan, list);
	} while (next);

done:
	if (ret == -ENOMEM) {
		dev_err(vi->dev, "graph init failed\n");
		tegra_vi_graph_cleanup(vi);
		return ret;
	}

	return 0;
}
