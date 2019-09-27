/*
 * Copyright (c) 2015-2018 NVIDIA CORPORATION. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/tegra-camera-rtcpu.h>
#include <linux/bitops.h>
#include "soc/tegra/camrtc-channels.h"
#include "soc/tegra/camrtc-commands.h"

#define NV(p) "nvidia," #p

#define CAMRTC_IVC_CONFIG_SIZE	4096

struct tegra_ivc_region {
	uintptr_t base;
	size_t size;
	dma_addr_t iova;
	size_t config_size;
	size_t ivc_size;
};

struct tegra_ivc_bus {
	struct device dev;
	struct tegra_ivc_channel *chans;
	unsigned num_regions;
	struct tegra_ivc_region regions[];
};

static void tegra_hsp_ring(struct device *dev)
{
	const struct tegra_hsp_ops *ops = tegra_hsp_dev_ops(dev);

	BUG_ON(ops == NULL || ops->ring == NULL);
	ops->ring(dev);
}

static void tegra_ivc_channel_ring(struct ivc *ivc)
{
	struct tegra_ivc_channel *chan =
		container_of(ivc, struct tegra_ivc_channel, ivc);
	struct tegra_ivc_bus *bus =
		container_of(chan->dev.parent, struct tegra_ivc_bus, dev);

	tegra_hsp_ring(&bus->dev);
}

struct device_type tegra_ivc_channel_type = {
	.name = "tegra-ivc-channel",
};
EXPORT_SYMBOL(tegra_ivc_channel_type);

int tegra_ivc_channel_runtime_get(struct tegra_ivc_channel *ch)
{
	BUG_ON(ch == NULL);

	return pm_runtime_get_sync(&ch->dev);
}
EXPORT_SYMBOL(tegra_ivc_channel_runtime_get);

void tegra_ivc_channel_runtime_put(struct tegra_ivc_channel *ch)
{
	BUG_ON(ch == NULL);
	BUG_ON(ch->dev.parent == NULL);
	BUG_ON(ch->dev.parent->parent == NULL);

	pm_runtime_mark_last_busy(ch->dev.parent->parent);
	pm_runtime_mark_last_busy(ch->dev.parent);
	pm_runtime_put(&ch->dev);
}
EXPORT_SYMBOL(tegra_ivc_channel_runtime_put);

static void tegra_ivc_channel_release(struct device *dev)
{
	struct tegra_ivc_channel *chan =
		container_of(dev, struct tegra_ivc_channel, dev);

	of_node_put(dev->of_node);
	kfree(chan);
}

static struct tegra_ivc_channel *tegra_ivc_channel_create(
		struct tegra_ivc_bus *bus, struct device_node *ch_node,
		struct tegra_ivc_region *region)
{
	struct device *peer_device = bus->dev.parent;
	struct camrtc_tlv_ivc_setup *tlv;
	struct {
		u32 rx;
		u32 tx;
	} start, end;
	u32 version, channel_group, nframes, frame_size, queue_size;
	const char *service;
	int ret;

	struct tegra_ivc_channel *chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (unlikely(chan == NULL))
		return ERR_PTR(-ENOMEM);

	chan->dev.parent = &bus->dev;
	chan->dev.type = &tegra_ivc_channel_type;
	chan->dev.bus = &tegra_ivc_bus_type;
	chan->dev.of_node = of_node_get(ch_node);
	chan->dev.release = tegra_ivc_channel_release;
	dev_set_name(&chan->dev, "%s:%s", dev_name(&bus->dev),
			kbasename(ch_node->full_name));
	device_initialize(&chan->dev);
	pm_runtime_no_callbacks(&chan->dev);
	pm_runtime_enable(&chan->dev);

	ret = of_property_read_string(ch_node, NV(service), &service);
	if (ret) {
		dev_err(&chan->dev, "missing <%s> property\n",
			NV(service));
		goto error;
	}

	ret = of_property_read_u32(ch_node, NV(version), &version);
	if (ret)
		version = 0;

	ret = of_property_read_u32(ch_node, NV(group), &channel_group);
	if (ret) {
		dev_err(&chan->dev, "missing <%s> property\n", NV(group));
		goto error;
	}

	ret = of_property_read_u32(ch_node, NV(frame-count), &nframes);
	if (ret || !nframes) {
		dev_err(&chan->dev, "missing <%s> property\n",
			NV(frame-count));
		goto error;
	}
	nframes = 1 << fls(nframes - 1); /* Round up to a power of two */

	ret = of_property_read_u32(ch_node, NV(frame-size), &frame_size);
	if (ret || !frame_size) {
		dev_err(&chan->dev, "missing <%s> property\n", NV(frame-size));
		goto error;
	}

	if (region->config_size + sizeof(*tlv) > CAMRTC_IVC_CONFIG_SIZE) {
		dev_err(&chan->dev, "IVC config size exceeded\n");
		ret = -ENOSPC;
		goto error;
	}

	queue_size = tegra_ivc_total_queue_size(nframes * frame_size);
	if (region->ivc_size + 2 * queue_size > region->size) {
		dev_err(&chan->dev, "buffers exceed IVC region\n");
		ret = -ENOSPC;
		goto error;
	}

	start.rx = region->ivc_size;
	region->ivc_size += queue_size;
	end.rx = region->ivc_size;

	start.tx = end.rx;
	region->ivc_size += queue_size;
	end.tx = region->ivc_size;

	/* Init IVC */
	ret = tegra_ivc_init_with_dma_handle(&chan->ivc,
			region->base + start.rx, region->iova + start.rx,
			region->base + start.tx, region->iova + start.tx,
			nframes, frame_size,
			/* Device used to allocate the shared memory for IVC */
			peer_device,
			tegra_ivc_channel_ring);
	if (ret) {
		dev_err(&chan->dev, "IVC initialization error: %d\n", ret);
		goto error;
	}

	tegra_ivc_channel_reset(&chan->ivc);

	/* Fill channel descriptor */
	tlv = (struct camrtc_tlv_ivc_setup *)
		(region->base + region->config_size);

	tlv->tag = CAMRTC_TAG_IVC_SETUP;
	tlv->len = sizeof(*tlv);
	tlv->rx_iova = region->iova + start.rx;
	tlv->rx_frame_size = frame_size;
	tlv->rx_nframes = nframes;
	tlv->tx_iova = region->iova + start.tx;
	tlv->tx_frame_size = frame_size;
	tlv->tx_nframes = nframes;
	tlv->channel_group = channel_group;
	tlv->ivc_version = version;
	if (strscpy(tlv->ivc_service, service, sizeof(tlv->ivc_service)) < 0)
		dev_warn(&chan->dev, "service name <%s> too long\n", service);

	region->config_size += sizeof(*tlv);
	(++tlv)->tag = 0; /* terminator */

	dev_info(&chan->dev,
		"%s: ver=%u grp=%u RX[%ux%u]=0x%x-0x%x TX[%ux%u]=0x%x-0x%x\n",
		ch_node->name, version, channel_group,
		nframes, frame_size, start.rx, end.rx,
		nframes, frame_size, start.tx, end.tx);

	ret = device_add(&chan->dev);
	if (ret) {
		dev_err(&chan->dev, "channel device error: %d\n", ret);
		goto error;
	}

	return chan;
error:
	put_device(&chan->dev);
	return ERR_PTR(ret);
}

static void tegra_ivc_channel_notify(struct tegra_ivc_channel *chan)
{
	const struct tegra_ivc_channel_ops *ops;

	if (tegra_ivc_channel_notified(&chan->ivc) != 0)
		return;

	if (!chan->is_ready)
		return;

	rcu_read_lock();
	ops = rcu_dereference(chan->ops);

	if (ops != NULL && ops->notify != NULL)
		ops->notify(chan);
	rcu_read_unlock();
}

void tegra_hsp_notify(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);
	struct tegra_ivc_channel *chan;

	for (chan = bus->chans; chan != NULL; chan = chan->next)
		tegra_ivc_channel_notify(chan);
}
EXPORT_SYMBOL(tegra_hsp_notify);

struct device_type tegra_hsp_type = {
	.name = "tegra-hsp",
};
EXPORT_SYMBOL(tegra_hsp_type);

static void tegra_ivc_bus_release(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);
	int i;

	of_node_put(dev->of_node);

	for (i = 0; i < bus->num_regions; i++) {
		if (!bus->regions[i].base)
			continue;

		dma_free_coherent(dev->parent, bus->regions[i].size,
				(void *)bus->regions[i].base,
				bus->regions[i].iova);
	}

	kfree(bus);
}

static int tegra_ivc_bus_match(struct device *dev, struct device_driver *drv)
{
	struct tegra_ivc_driver *ivcdrv = to_tegra_ivc_driver(drv);

	if (dev->type != ivcdrv->dev_type)
		return 0;
	return of_driver_match_device(dev, drv);
}

static void tegra_ivc_bus_stop(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);

	while (bus->chans != NULL) {
		struct tegra_ivc_channel *chan = bus->chans;

		bus->chans = chan->next;
		pm_runtime_disable(&chan->dev);
		device_unregister(&chan->dev);
	}
}

static int tegra_ivc_bus_start(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);
	struct device_node *dn = bus->dev.parent->of_node;
	struct of_phandle_args reg_spec;
	const char *status;
	int i, ret;

	for (i = 0;
		of_parse_phandle_with_fixed_args(dn, NV(ivc-channels), 3,
							i, &reg_spec) == 0;
		i++) {
		struct device_node *ch_node;

		for_each_child_of_node(reg_spec.np, ch_node) {
			struct tegra_ivc_channel *chan;

			ret = of_property_read_string(ch_node,
					"status", &status);

			if (ret == 0) {
				ret = strcmp(status, "disabled");

				if (ret == 0)
					continue;
			}

			chan = tegra_ivc_channel_create(bus, ch_node,
							&bus->regions[i]);
			if (IS_ERR(chan)) {
				ret = PTR_ERR(chan);
				of_node_put(ch_node);
				goto error;
			}

			chan->next = bus->chans;
			bus->chans = chan;
		}
	}

	return 0;
error:
	tegra_ivc_bus_stop(dev);
	return ret;
}

/*
 * This is called during RTCPU boot to synchronize
 * (or re-synchronize in the case of PM resume).
 */
int tegra_ivc_bus_boot_sync(struct tegra_ivc_bus *bus)
{
	int i;

	if (IS_ERR_OR_NULL(bus))
		return 0;

	for (i = 0; i < bus->num_regions; i++) {
		int ret = tegra_camrtc_iovm_setup(bus->dev.parent,
				bus->regions[i].iova);
		if (ret != 0) {
			dev_err(&bus->dev, "IOVM setup error: %d\n", ret);
			return -EIO;
		}
	}

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_bus_boot_sync);

static int tegra_ivc_bus_probe(struct device *dev)
{
	struct tegra_ivc_driver *drv = to_tegra_ivc_driver(dev->driver);
	int ret = -ENXIO;

	if (dev->type == &tegra_ivc_channel_type) {
		struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
		const struct tegra_ivc_channel_ops *ops = drv->ops.channel;

		mutex_init(&chan->ivc_wr_lock);

		BUG_ON(ops == NULL);
		if (ops->probe != NULL) {
			ret = ops->probe(chan);
			if (ret)
				return ret;
		}

		rcu_assign_pointer(chan->ops, ops);
		ret = 0;

	} else if (dev->type == &tegra_hsp_type) {
		const struct tegra_hsp_ops *ops = drv->ops.hsp;

		BUG_ON(ops == NULL || ops->probe == NULL);
		ret = ops->probe(dev);
		if (ret)
			return ret;

		ret = tegra_ivc_bus_start(dev);
		if (ret && ops->remove != NULL)
			ops->remove(dev);
	}

	return ret;
}

static int tegra_ivc_bus_remove(struct device *dev)
{
	struct tegra_ivc_driver *drv = to_tegra_ivc_driver(dev->driver);

	if (dev->type == &tegra_ivc_channel_type) {
		struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
		const struct tegra_ivc_channel_ops *ops = drv->ops.channel;

		WARN_ON(rcu_access_pointer(chan->ops) != ops);
		RCU_INIT_POINTER(chan->ops, NULL);
		synchronize_rcu();

		if (ops->remove != NULL)
			ops->remove(chan);

	} else if (dev->type == &tegra_hsp_type) {
		const struct tegra_hsp_ops *ops = drv->ops.hsp;

		tegra_ivc_bus_stop(dev);

		if (ops->remove != NULL)
			ops->remove(dev);
	}

	return 0;
}

static int tegra_ivc_bus_ready_child(struct device *dev, void *data)
{
	struct tegra_ivc_driver *drv = to_tegra_ivc_driver(dev->driver);
	bool is_ready = (data != NULL) ? *(bool *)data : true;

	if (dev->type == &tegra_ivc_channel_type) {
		struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
		const struct tegra_ivc_channel_ops *ops;

		chan->is_ready = is_ready;
		if (!is_ready)
			atomic_inc(&chan->bus_resets);
		smp_wmb();

		if (drv != NULL) {
			rcu_read_lock();
			ops = rcu_dereference(chan->ops);
			if (ops->ready != NULL)
				ops->ready(chan, is_ready);
			rcu_read_unlock();
		} else {
			dev_warn(dev, "ivc channel driver missing\n");
		}
	}

	return 0;
}

struct bus_type tegra_ivc_bus_type = {
	.name	= "tegra-ivc",
	.match	= tegra_ivc_bus_match,
	.probe	= tegra_ivc_bus_probe,
	.remove	= tegra_ivc_bus_remove,
};
EXPORT_SYMBOL(tegra_ivc_bus_type);

int tegra_ivc_driver_register(struct tegra_ivc_driver *drv)
{
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL(tegra_ivc_driver_register);

void tegra_ivc_driver_unregister(struct tegra_ivc_driver *drv)
{
	return driver_unregister(&drv->driver);
}
EXPORT_SYMBOL(tegra_ivc_driver_unregister);

static int tegra_ivc_bus_parse_regions(struct tegra_ivc_bus *bus,
					struct device_node *dev_node)
{
	struct of_phandle_args reg_spec;
	int i;

	/* Parse out all regions in a node */
	for (i = 0;
		of_parse_phandle_with_fixed_args(dev_node, NV(ivc-channels), 3,
							i, &reg_spec) == 0;
		i++) {
		struct device_node *ch_node;
		struct tegra_ivc_region *region = &bus->regions[i];
		u32 nframes, frame_size, size = CAMRTC_IVC_CONFIG_SIZE;
		int ret = -ENODEV;

		if (reg_spec.args_count < 3) {
			of_node_put(reg_spec.np);
			dev_err(&bus->dev, "invalid region specification\n");
			return -EINVAL;
		}

		for_each_child_of_node(reg_spec.np, ch_node) {
			ret = of_property_read_u32(ch_node, NV(frame-count),
						&nframes);
			if (ret || !nframes) {
				dev_err(&bus->dev, "missing <%s> property\n",
					NV(frame-count));
				break;
			}
			/* Round up to a power of two */
			nframes = 1 << fls(nframes - 1);

			ret = of_property_read_u32(ch_node, NV(frame-size),
						&frame_size);
			if (ret || !frame_size) {
				dev_err(&bus->dev, "missing <%s> property\n",
					NV(frame-size));
				break;
			}

			size += 2 * tegra_ivc_total_queue_size(nframes *
							frame_size);
		}
		of_node_put(reg_spec.np);

		if (ret)
			return ret;

		region->base =
			(uintptr_t)dma_alloc_coherent(bus->dev.parent,
						size, &region->iova,
						GFP_KERNEL | __GFP_ZERO);
		if (!region->base)
			return -ENOMEM;

		region->size = size;
		region->config_size = 0;
		region->ivc_size = CAMRTC_IVC_CONFIG_SIZE;

		dev_info(&bus->dev, "region %u: iova=0x%x-0x%x size=%u\n",
			i, (u32)region->iova, (u32)region->iova + size - 1,
			size);
	}

	return 0;
}

static unsigned tegra_ivc_bus_count_regions(const struct device_node *dev_node)
{
	unsigned i;

	for (i = 0; of_parse_phandle_with_fixed_args(dev_node,
			NV(ivc-channels), 3, i, NULL) == 0; i++)
		;

	return i;
}

struct tegra_ivc_bus *tegra_ivc_bus_create(struct device *dev)
{
	struct tegra_ivc_bus *bus;
	unsigned num;
	int ret;

	num = tegra_ivc_bus_count_regions(dev->of_node);

	bus = kzalloc(sizeof(*bus) + num * sizeof(*bus->regions), GFP_KERNEL);
	if (unlikely(bus == NULL))
		return ERR_PTR(-ENOMEM);

	bus->num_regions = num;
	bus->dev.parent = dev;
	bus->dev.type = &tegra_hsp_type;
	bus->dev.bus = &tegra_ivc_bus_type;
	bus->dev.of_node = of_get_child_by_name(dev->of_node, "hsp");
	bus->dev.release = tegra_ivc_bus_release;
	dev_set_name(&bus->dev, "ivc-%s", dev_name(dev));
	device_initialize(&bus->dev);
	pm_runtime_no_callbacks(&bus->dev);
	pm_runtime_enable(&bus->dev);

	ret = tegra_ivc_bus_parse_regions(bus, dev->of_node);
	if (ret) {
		dev_err(&bus->dev, "IVC regions setup failed: %d\n", ret);
		goto error;
	}

	ret = device_add(&bus->dev);
	if (ret) {
		dev_err(&bus->dev, "IVC instance error: %d\n", ret);
		goto error;
	}

	return bus;

error:
	put_device(&bus->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(tegra_ivc_bus_create);

/*
 * Communicate RTCPU UP/DOWN state to IVC devices.
 */
void tegra_ivc_bus_ready(struct tegra_ivc_bus *bus, bool online)
{
	if (IS_ERR_OR_NULL(bus))
		return;

	device_for_each_child(&bus->dev, &online, tegra_ivc_bus_ready_child);

	if (online)
		tegra_hsp_notify(&bus->dev);
}
EXPORT_SYMBOL(tegra_ivc_bus_ready);

void tegra_ivc_bus_destroy(struct tegra_ivc_bus *bus)
{
	if (IS_ERR_OR_NULL(bus))
		return;

	pm_runtime_disable(&bus->dev);
	device_unregister(&bus->dev);
}
EXPORT_SYMBOL(tegra_ivc_bus_destroy);

static __init int tegra_ivc_bus_init(void)
{
	return bus_register(&tegra_ivc_bus_type);
}

static __exit void tegra_ivc_bus_exit(void)
{
	bus_unregister(&tegra_ivc_bus_type);
}

subsys_initcall(tegra_ivc_bus_init);
module_exit(tegra_ivc_bus_exit);
MODULE_AUTHOR("Remi Denis-Courmont <remid@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra IVC generic bus driver");
MODULE_LICENSE("GPL");
