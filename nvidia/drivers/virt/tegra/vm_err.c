/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#define pr_fmt(fmt) "vm-err: " fmt

#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/vm_err.h>
#include <asm/traps.h>
#include <asm-generic/irq_regs.h>
#include <asm/system_misc.h>
#include <soc/tegra/virt/syscalls.h>
#include <soc/tegra/chip-id.h>

struct tegra_hv_err_ctrl {
	struct device *dev;
	struct err_info_t *err_info;
	unsigned int async_err_arr_items;
	int hv_peer_err_irq_id;
	unsigned int vcpu_cnt;
	struct serr_hook hook;
	struct tegra_hv_vm_err_handlers handlers;
};

static struct tegra_hv_config config;

static unsigned int tegra_hv_virq_intr_info[3]; /* intr_property_size = 3 */

static struct property tegra_hv_virq_intr_prop = {
	.name = "interrupts",
};

static bool check_sync_err(const unsigned int vcpu_id,
	const struct tegra_hv_err_ctrl *const ctrl,
	bool *send_sync_err_ack)
{
	uint64_t rd_idx;
	const struct err_data_t *err_data;

	if (vcpu_id >= ctrl->vcpu_cnt) {
		dev_crit(ctrl->dev, "%s: Invalid vcpu id %u\n", __func__,
			vcpu_id);
		*send_sync_err_ack = false;
		/* Unexpected vcpu id. Enter bad mode. */
		return true;
	}

	/* Shared memory layout is:
	 * |--async-err-metadata--|--async-errors-array-|--sync-errors-array-|
	 * Size of async errors array = Max errors + 1(to avoid same empty
	 * and full conditions of the buffer)
	 * Size of sync errors array = 1 error per VCPU * number of VCPUs in VM
	 */
	rd_idx = ctrl->async_err_arr_items + vcpu_id;
	/* It's already validated at init time that sufficient memory is
	 * allocated to hold async_err_arr_items + sync error per vcpu. Hence,
	 * after validating the vcpu_id above, no need to validate rd_idx here.
	 */
	err_data = &(ctrl->err_info->err_data[rd_idx]);
	if (err_data->err_reason != REASON_SYNC) {
		dev_crit(ctrl->dev, "%s: unexpected reason id %u\n", __func__,
			err_data->err_reason);
		*send_sync_err_ack = true;
		/* Invalid reason. Enter bad mode. */
		return true;
	}

	if (err_data->err_type != SYNC) {
		dev_crit(ctrl->dev, "%s: unexpected error Type %d\n",
			__func__, err_data->err_type);
		*send_sync_err_ack = true;
		/* Unexpected error id. Enter bad mode. */
		return true;
	}

	if (err_data->offending_guest_id != config.guest_id_self) {
		dev_crit(ctrl->dev, "%s: invalid offender id %u\n", __func__,
			err_data->offending_guest_id);
		*send_sync_err_ack = true;
		/* Invalid id of offending guest. Enter bad mode. */
		return true;
	}
	dev_err(ctrl->dev, "Synchronous error on vcpu %u\n", vcpu_id);

	if (ctrl->handlers.fn_self_sync) {
		*send_sync_err_ack = true;
		/* Enter bad_mode (or otherwise) as custom handler dictates */
		return ctrl->handlers.fn_self_sync(err_data);
	}

	/* should never reach here */
	*send_sync_err_ack = true;
	/* Reaching here is unexpected. Enter bad mode. */
	return true;
}

static irqreturn_t async_err_handler(int irq, void *context)
{
	unsigned int num_async_errs_read = 0;
	bool enter_bad_mode = false;
	const struct tegra_hv_err_ctrl *const ctrl = context;
	const unsigned int vcpu_id = hyp_read_vcpu_id();
	uint64_t local_rd_idx, next_rd_idx;
	const struct err_data_t *err_data;
	bool (*fn_self_async)(const struct err_data_t *const err_data);
	bool (*fn_peer)(const struct err_data_t *const err_data);
	bool (*handler)(const struct err_data_t *const err_data);
	struct pt_regs *regs;

	if (vcpu_id != 0) {
		dev_err(ctrl->dev, "Asynchronous error on vcpu %u\n", vcpu_id);
		/* Only VCPU0 is expected to receive async error vIRQ */
		return IRQ_HANDLED;
	}

	fn_self_async = ctrl->handlers.fn_self_async;
	fn_peer = ctrl->handlers.fn_peer;

	if ((fn_self_async == NULL) && (fn_peer == NULL)) {
		dev_err(ctrl->dev, "Asynchronous error handlers absent\n");
		return IRQ_HANDLED;
	}

	local_rd_idx = ctrl->err_info->async_metadata.rd_idx;
	dev_dbg(ctrl->dev, "Local Rd Idx = %llu, shared Wr Idx = %llu\n",
		local_rd_idx, ctrl->err_info->async_metadata.wr_idx);

	/* Check async error. Read until error queue gets empty */
	while (local_rd_idx != ctrl->err_info->async_metadata.wr_idx) {
		next_rd_idx = (local_rd_idx + 1) % ctrl->async_err_arr_items;

		err_data = &(ctrl->err_info->err_data[next_rd_idx]);
		if (err_data->offending_guest_id == config.guest_id_self)
			handler = fn_self_async;
		else
			handler = fn_peer;

		if (handler) {
			enter_bad_mode = handler(err_data);
		}

		local_rd_idx = next_rd_idx;
		num_async_errs_read++;
		dev_dbg(ctrl->dev, "Local Rd Idx = %llu\n", local_rd_idx);
	}

	if (num_async_errs_read) {
		dev_err(ctrl->dev, "%u asynchronous error(s) read\n",
			num_async_errs_read);

		/* Send ack for async error(s) to HV */
		if (hyp_send_async_err_ack(local_rd_idx) != 0) {
			dev_crit(ctrl->dev,
				"%s: Sending ack failed. Setting bad mode\n",
				__func__);
			/* Unexpected */
			enter_bad_mode = true;
		}
	}

	if (enter_bad_mode) {
		regs = get_irq_regs();
		die("Oops - bad mode", regs, 0);
		panic("bad mode");
	}

	return IRQ_HANDLED;
}

static int sync_err_handler(struct pt_regs *regs, int reason,
	uint32_t esr, void *context)
{
	bool enter_bad_mode = false;
	bool send_sync_err_ack = false;
	const struct tegra_hv_err_ctrl *const ctrl = context;
	const unsigned int vcpu_id = hyp_read_vcpu_id();

	/* Check sync error */
	enter_bad_mode = check_sync_err(vcpu_id, ctrl, &send_sync_err_ack);

	/* Send ack for error to HV. */
	if (send_sync_err_ack) {
		if (hyp_send_sync_err_ack() != 0) {
			dev_crit(ctrl->dev,
				"%s: Sending ack failed. Setting bad mode\n",
				__func__);
			/* Unexpected */
			enter_bad_mode = true;
		}
	}

	/* bad_mode() will call die(). Force the latter to panic. */
	if (enter_bad_mode) {
		panic_on_oops = 1;
		wmb();
	}

	/* Caller expects 0 to enter bad mode */
	return (!enter_bad_mode);
}

void tegra_hv_get_config(struct tegra_hv_config *cfg)
{
	cfg->guest_id_self = config.guest_id_self;
	cfg->num_guests = config.num_guests;
}
EXPORT_SYMBOL(tegra_hv_get_config);

static int virq_handler_init(struct platform_device *pdev)
{
	int ret;
	struct irq_data *peer_err_irq_data;
	int lin_peer_err_irq_id;
	struct tegra_hv_err_ctrl *ctrl = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_info(ctrl->dev, "Error notification HV IRQ id: %d\n",
		ctrl->hv_peer_err_irq_id);

	/* Ensure HV returned valid irq */
	if (ctrl->hv_peer_err_irq_id == -1)
		return 0;

	/* Set indicate irq type 0 to indicate Shared Peripheral Irq */
	tegra_hv_virq_intr_info[0] = cpu_to_be32(0);
	/* Id in SPI namespace - subtract number of PPIs
	 * (Private Peripheral Irqs) which is = 32
	 */
	tegra_hv_virq_intr_info[1] = cpu_to_be32(ctrl->hv_peer_err_irq_id - 32);
	/* Trigger irq on low-to-high edge (0x1) */
	tegra_hv_virq_intr_info[2] = cpu_to_be32(IRQF_TRIGGER_RISING);

	tegra_hv_virq_intr_prop.length = sizeof(tegra_hv_virq_intr_info);
	dev_info(ctrl->dev, "tegra_hv_virq_intr_prop.length %u\n",
		tegra_hv_virq_intr_prop.length);

	tegra_hv_virq_intr_prop.value = tegra_hv_virq_intr_info;

	if (of_add_property(dev->of_node, &tegra_hv_virq_intr_prop)) {
		dev_err(ctrl->dev, "%s: failed to add interrupts property\n",
			__func__);
		return -EACCES;
	}

	lin_peer_err_irq_id = of_irq_get(dev->of_node, 0);
	if (lin_peer_err_irq_id < 0) {
		dev_err(ctrl->dev, "%s: Unable to get Linux irq for id %d\n",
			__func__, ctrl->hv_peer_err_irq_id);
		return lin_peer_err_irq_id;
	}

	peer_err_irq_data = irq_get_irq_data(lin_peer_err_irq_id);
	if (peer_err_irq_data == NULL) {
		dev_err(ctrl->dev, "%s: Failed to get data for Linux irq %d\n",
			__func__, lin_peer_err_irq_id);
		return -ENODEV;
	}

	ret = devm_request_irq(dev, lin_peer_err_irq_id, async_err_handler,
			IRQ_NOTHREAD, dev_name(dev), ctrl);
	if (ret < 0) {
		dev_err(ctrl->dev,
			"%s: failed to register IRQ %d, Err %d, %s\n",
			__func__, lin_peer_err_irq_id, ret, dev_name(dev));
		return ret;
	}
	dev_info(ctrl->dev, "Registered Linux IRQ %d for peer notification\n",
		lin_peer_err_irq_id);

	return 0;
}

static int serr_handler_init(struct platform_device *pdev)
{
	struct tegra_hv_err_ctrl *ctrl = platform_get_drvdata(pdev);

	ctrl->hook.fn = sync_err_handler;
	ctrl->hook.priv = platform_get_drvdata(pdev);
	register_serr_hook(&ctrl->hook);

	return 0;
}

static int shared_mem_map(struct platform_device *pdev)
{
	uint64_t ipa, buff_size, required_size;
	int ret;
	struct tegra_hv_err_ctrl *ctrl = platform_get_drvdata(pdev);

	/* Get error info details */
	ret = hyp_read_err_info_get(&ipa, &buff_size,
		&ctrl->async_err_arr_items, &ctrl->hv_peer_err_irq_id,
		&ctrl->vcpu_cnt);
	if (ret != 0) {
		/* It could come here if DTS and defconfig enable execution
		 * of this code, but HV hasn't implemented the hypercall.
		 * Flag error.
		 */
		dev_err(ctrl->dev,
			"%s: failed to get err memory address. Err %d\n",
			__func__, ret);
		return -ENODEV;
	}

	if ((ipa == 0) || (buff_size == 0) ||
		(ctrl->async_err_arr_items == 0)) {
		/* It could come here if DTS and defconfig enable execution
		 * of this code, but PCT hasn't enabled error injection.
		 * A warning should suffice.
		 */
		dev_warn(ctrl->dev, "%s: invalid shared memory parameters.\n",
			__func__);
		dev_warn(ctrl->dev,
			"%s: make sure error injection is enabled in PCT\n",
			__func__);
		return -ENOMEM;
	}

	/* Shared memory layout is:
	 * |--async-err-metadata--|--async-errors-array-|--sync-errors-array-|
	 * Size of async errors array = Max errors + 1 (to avoid same empty and
	 * full conditions of the buffer)
	 * Size of sync errors array = 1 error per VCPU * number of VCPUs on
	 * a VM
	 */
	required_size = sizeof(struct async_metadata_t) +
		(sizeof(struct err_data_t) *
		(ctrl->async_err_arr_items + ctrl->vcpu_cnt));
	if (buff_size < required_size) {
		dev_err(ctrl->dev,
			"%s:invalid params. size %llu. required size %llu\n",
			__func__, buff_size, required_size);
		dev_err(ctrl->dev, "%s: async arr size %u. vcpus %u\n",
			__func__, ctrl->async_err_arr_items, ctrl->vcpu_cnt);
		return -ENOMEM;
	}

	dev_info(ctrl->dev, "%s: Err info IPA for guest %u: 0x%llx\n",
		__func__, config.guest_id_self, ipa);
	dev_info(ctrl->dev, "Err info buf size 0x%llX\n", buff_size);
	dev_info(ctrl->dev, "Async err arr size %u. Number of VCPUs %u\n",
		ctrl->async_err_arr_items, ctrl->vcpu_cnt);

	/* Map shared memory */
	ctrl->err_info = (struct err_info_t *) ioremap_cache(ipa, buff_size);
	if (ctrl->err_info == NULL)
		return -ENOMEM;

	return 0;
}

static int hyp_config_init(struct device *dev)
{
	int ret = hyp_read_gid(&config.guest_id_self);

	if (ret != 0) {
		dev_err(dev, "%s: failed to read guest id. Err %d\n",
			__func__, ret);
		return ret;
	}

	ret = hyp_read_nguests(&config.num_guests);
	if (ret != 0) {
		/* Only privileged guest can query number of guests */
		dev_warn(dev, "%s: can't read number of guests. Err %d\n",
			__func__, ret);
	}

	dev_info(dev, "%s: guest id %u num guests %u\n", __func__,
		config.guest_id_self, config.num_guests);

	return 0;
}

static void shared_structs_check(struct device *dev)
{
	/* Ensure coherency with common header */
	BUILD_BUG_ON(REASON_ENUM_SIZE !=
		(ARRAY_SIZE(tegra_hv_err_reason_desc)));

	/* Dump shared structures sizes. This is needed to manually compare
	 * with HV console dump, if a build bug fires below.
	 */
	dev_info(dev, "async_metadata size 0x%lx\n",
		sizeof(struct async_metadata_t));
	dev_info(dev, "async_bridge_err size 0x%lx\n",
		sizeof(struct async_bridge_err_t));
	dev_info(dev, "async_cbb_err size 0x%lx\n",
		sizeof(struct async_cbb_err_t));
	dev_info(dev, "async_smmu_err size 0x%lx\n",
		sizeof(struct async_smmu_err_t));
	dev_info(dev, "async_mc_err size 0x%lx\n",
		sizeof(struct async_mc_err_t));
	dev_info(dev, "async_mc_err_t19x size 0x%lx\n",
		sizeof(struct async_mc_err_t19x_t));
	dev_info(dev, "sync size 0x%lx\n",
		sizeof(struct sync_t));
	dev_info(dev, "err_data size 0x%lx\n", sizeof(struct err_data_t));

	/* Ensure common structures shared by HV and Linux are in sync */
	BUILD_BUG_ON(sizeof(struct async_metadata_t) != 0x10);
	BUILD_BUG_ON(sizeof(struct async_bridge_err_t) != 0x74);
	BUILD_BUG_ON(sizeof(struct async_cbb_err_t) != 0x259);
	BUILD_BUG_ON(sizeof(struct async_smmu_err_t) != 0x1C);
	BUILD_BUG_ON(sizeof(struct async_mc_err_t) != 0x24);
	BUILD_BUG_ON(sizeof(struct async_mc_err_t19x_t) != 0xBB);
	BUILD_BUG_ON(sizeof(struct sync_t) != 0x149);
	BUILD_BUG_ON(sizeof(struct err_data_t) != 0x265);
}

static int vm_err_handler_init(struct platform_device *pdev)
{
	int ret;
	struct tegra_hv_err_ctrl *ctrl;
	struct device *dev = &pdev->dev;

	if (!is_tegra_hypervisor_mode()) {
		dev_err(dev, "%s: hypervisor is not present\n", __func__);
		return -ENODEV;
	}

	shared_structs_check(dev);

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = dev;
	platform_set_drvdata(pdev, ctrl);

	ret = hyp_config_init(dev);
	if (ret)
		return ret;

	ret = shared_mem_map(pdev);
	if (ret)
		return -ENOMEM;

	ret = serr_handler_init(pdev);
	if (ret)
		return ret;

	ret = virq_handler_init(pdev);
	if (ret)
		return ret;

	return 0;
}

static int vm_err_handler_remove(struct platform_device *pdev)
{
	struct tegra_hv_err_ctrl *ctrl = platform_get_drvdata(pdev);
	struct device_node *node = pdev->dev.of_node;

	if (of_remove_property(node,
		of_find_property(node, "interrupts", NULL))) {
		dev_err(ctrl->dev, "%s: failed to add interrupts property\n",
			__func__);
		return -EACCES;
	}

	unregister_serr_hook(&ctrl->hook);
	iounmap(ctrl->err_info);

	dev_info(ctrl->dev, "%s: cleaned up and unregistered handler\n",
		__func__);

	return 0;
}

static const struct of_device_id tegra_hv_err_match[] = {
	{ .compatible = "nvidia,tegra-hv-err", .data = NULL},
	{},
};

static struct platform_driver tegra_hv_err_pdriver = {
	.driver = {
		.name = "tegra-hv-err-handler",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_hv_err_match),
	},
	.probe = vm_err_handler_init,
	.remove = vm_err_handler_remove,
};

static int tegra_hv_register_hooks_for_device(struct device *dev,
	void *handlers)
{
	struct tegra_hv_err_ctrl *ctrl;
	const struct platform_device *pd = container_of(dev,
		struct platform_device, dev);
	const struct tegra_hv_vm_err_handlers *_handlers =
		(struct tegra_hv_vm_err_handlers *) handlers;

	ctrl = platform_get_drvdata(pd);
	if (!ctrl) {
		dev_err(dev, "%s: no platform data", __func__);
		return 0;
	}

	if (ctrl->handlers.fn_self_async == NULL)
		ctrl->handlers.fn_self_async = _handlers->fn_self_async;

	if (ctrl->handlers.fn_self_sync == NULL)
		ctrl->handlers.fn_self_sync = _handlers->fn_self_sync;

	if (ctrl->handlers.fn_peer == NULL)
		ctrl->handlers.fn_peer = _handlers->fn_peer;

	return 0;
}

int tegra_hv_register_vm_err_hooks(struct tegra_hv_vm_err_handlers *handlers)
{
	int ret;

	if (!handlers) {
		pr_err("%s: invalid error handlers\n", __func__);
		return 1;
	}

	if (!handlers->fn_self_async && !handlers->fn_self_sync
		&& !handlers->fn_peer) {
		platform_driver_unregister(&tegra_hv_err_pdriver);
		return 0;
	}

	if (!tegra_hv_err_pdriver.driver.p) {
		/* Not registered/bound yet */
		ret = platform_driver_register(&tegra_hv_err_pdriver);
		if (ret) {
			pr_err("%s: failed to register driver. Err %d\n",
				__func__, ret);
			return ret;
		}
	}

	ret = driver_for_each_device(&tegra_hv_err_pdriver.driver, NULL,
		handlers, tegra_hv_register_hooks_for_device);
	if (ret) {
		pr_err("%s: failed to attach driver. Err %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_hv_register_vm_err_hooks);
