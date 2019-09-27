// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018, NVIDIA Corporation. All rights reserved. */

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/nvhost.h>
#include <linux/nvhost_t194.h>

#include "nvhost_intr.h"
#include "host1x/host1x.h"

struct nvhost_interrupt_syncpt {
	struct platform_device *host1x_pdev;
	u32 syncpt;
	u32 value;
	void *private_data;
	void (*callback)(void *);

	struct nvhost_waitlist *waiter;
	void *ref;
	struct nvhost_waitlist_external_notifier *notifier;
};

u32 nvhost_interrupt_syncpt_get_syncpt_index(struct nvhost_interrupt_syncpt *is)
{
	return is->syncpt;
}
EXPORT_SYMBOL(nvhost_interrupt_syncpt_get_syncpt_index);

phys_addr_t nvhost_interrupt_syncpt_get_syncpt_addr(
    struct nvhost_interrupt_syncpt *is)
{
    phys_addr_t base;
    size_t size;
    u32 offset;

    nvhost_syncpt_unit_interface_get_aperture(is->host1x_pdev, &base, &size);
    offset = nvhost_syncpt_unit_interface_get_byte_offset(is->syncpt);

    return base + offset;
}
EXPORT_SYMBOL(nvhost_interrupt_syncpt_get_syncpt_addr);

struct nvhost_interrupt_syncpt *nvhost_interrupt_syncpt_get(
	struct device_node *np, void (*callback)(void *), void *private_data)
{
	struct platform_device *host1x_pdev;
	struct nvhost_interrupt_syncpt *is;
	struct device_node *host1x_np;
	int err;

	host1x_np = of_parse_phandle(np, "nvidia,host1x", 0);
	if (!host1x_np)
		return ERR_PTR(-EINVAL);

	host1x_pdev = of_find_device_by_node(host1x_np);
	if (!host1x_pdev)
		return ERR_PTR(-EPROBE_DEFER);

	is = kzalloc(sizeof(*is), GFP_KERNEL);
	if (!is)
		return ERR_PTR(-ENOMEM);

	is->host1x_pdev = host1x_pdev;
	is->callback = callback;
	is->private_data = private_data;

	is->syncpt = nvhost_get_syncpt_client_managed(
		host1x_pdev, "interrupt_syncpt");
	if (!is->syncpt) {
		err = -EBUSY;
		goto free_is;
	}

	is->value = nvhost_syncpt_read_minval(host1x_pdev, is->syncpt);

	return is;

free_is:
	kfree(is);

	return ERR_PTR(err);
}
EXPORT_SYMBOL(nvhost_interrupt_syncpt_get);

void nvhost_interrupt_syncpt_free(struct nvhost_interrupt_syncpt *is)
{
	struct nvhost_master *master = nvhost_get_host(is->host1x_pdev);

	if (is->ref) {
		nvhost_intr_put_ref(&master->intr, is->syncpt, is->ref);
	}

	nvhost_syncpt_put_ref(&master->syncpt, is->syncpt);
	kfree(is);
}
EXPORT_SYMBOL(nvhost_interrupt_syncpt_free);

static void notifier_callback(void *private_data, int count)
{
	struct nvhost_interrupt_syncpt *is = private_data;

	is->callback(is->private_data);
}

int nvhost_interrupt_syncpt_prime(struct nvhost_interrupt_syncpt *is)
{
	struct nvhost_master *master = nvhost_get_host(is->host1x_pdev);
	int err;

	if (is->ref) {
		nvhost_intr_put_ref(&master->intr, is->syncpt, is->ref);
		is->ref = NULL;
	}

	is->waiter = kzalloc(sizeof(*is->waiter), GFP_KERNEL);
	if (!is->waiter) {
		return -ENOMEM;
	}

	is->notifier = kzalloc(sizeof(*is->notifier), GFP_KERNEL);
	if (!is->notifier) {
		err = -ENOMEM;
		goto free_waiter;
	}

	is->notifier->master = master;
	is->notifier->callback = notifier_callback;
	is->notifier->private_data = is;

	err = nvhost_intr_add_action(&master->intr, is->syncpt, ++is->value,
				     NVHOST_INTR_ACTION_FAST_NOTIFY,
				     is->notifier, is->waiter, &is->ref);
	if (err) {
		goto free_notifier;
	}

	return 0;

free_notifier:
	kfree(is->notifier);
free_waiter:
	kfree(is->waiter);

	return err;
}
EXPORT_SYMBOL(nvhost_interrupt_syncpt_prime);
