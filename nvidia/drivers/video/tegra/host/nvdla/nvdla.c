/*
 * NVDLA driver for T194
 *
 * Copyright (c) 2016-2019, NVIDIA Corporation.  All rights reserved.
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-attrs.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <soc/tegra/fuse.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"

#include "flcn/flcn.h"
#include "flcn/hw_flcn.h"
#include "nvhost_syncpt_unit_interface.h"

#include "t194/t194.h"


#include "nvdla/nvdla.h"
#include "nvdla/dla_queue.h"
#include "nvdla/nvdla_buffer.h"
#include "nvdla/nvdla_debug.h"
#include <uapi/linux/nvhost_nvdla_ioctl.h>
#include "dla_fw_version.h"
#include "dla_os_interface.h"

#include "class_ids_t194.h"

static DEFINE_DMA_ATTRS(attrs);

/*
 * Work to handle engine reset for error recovery
 */
static void nvdla_reset_handler(struct work_struct *work)
{
	struct nvdla_device *nvdla_dev = container_of(work,
					struct nvdla_device, reset_work);

	struct platform_device *pdev = nvdla_dev->pdev;

	/* reset engine */
	nvhost_module_reset(pdev, true);

	nvdla_dbg_info(pdev, "Engine reset done\n");
}

static void nvdla_reset_handler_init(struct nvdla_device *nvdla_dev)
{
	INIT_WORK(&nvdla_dev->reset_work, nvdla_reset_handler);
}

int nvhost_nvdla_flcn_isr(struct platform_device *pdev)
{
	uint32_t message;
	uint32_t mailbox0;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	/* dump falcon data if debug enabled */
	mailbox0 = host1x_readl(pdev, flcn_mailbox0_r());

	message = mailbox0 & DLA_RESPONSE_MSG_MASK;

	/* handles engine timeout,
	 * schedule work for reset handler and clears interrupt
	 */
	if (message == DLA_MSG_TASK_TIMEOUT) {
		nvdla_dbg_err(pdev, "engine timeout detected");
		schedule_work(&nvdla_dev->reset_work);
		goto clear_interrupt;
	}
	if (message == DLA_MSG_DEBUG_PRINT)
		nvdla_dbg_fw(pdev, "falcon: %s",
				(char *)nvdla_dev->debug_dump_va);

	if ((message == DLA_MSG_CMD_COMPLETE ||
				message == DLA_MSG_CMD_ERROR) &&
				nvdla_dev->waiting) {
		nvdla_dev->cmd_status =
				(mailbox0 >> DLA_RESPONSE_ERROR_SHIFT) &
						DLA_RESPONSE_ERROR_MASK;
		nvdla_dev->waiting = 0;
		complete(&nvdla_dev->cmd_completion);
	}

clear_interrupt:
	/* logic to clear the interrupt */
	host1x_writel(pdev, flcn_irqmclr_r(), flcn_irqmclr_swgen1_set_f());
	host1x_writel(pdev, flcn_thi_int_stat_r(), flcn_thi_int_stat_clr_f());
	host1x_readl(pdev, flcn_thi_int_stat_r());
	host1x_writel(pdev, flcn_irqsclr_r(), flcn_irqsclr_swgen1_set_f());

	return 0;
}

/* Helper API's */
static int nvdla_alloc_cmd_memory(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	int err = 0;

	/* allocate memory for command */
	nvdla_dev->cmd_mem.va = dma_alloc_attrs(&pdev->dev,
			MAX_CMD_SIZE * MAX_COMMANDS_PER_DEVICE,
			&nvdla_dev->cmd_mem.pa, GFP_KERNEL,
			__DMA_ATTR(attrs));

	if (nvdla_dev->cmd_mem.va == NULL) {
		err = -ENOMEM;
		goto err_alloc_cmd_mem;
	}

	mutex_init(&nvdla_dev->cmd_mem.lock);
	nvdla_dev->cmd_mem.alloc_table = 0;

err_alloc_cmd_mem:
	return err;
}

static int nvdla_free_cmd_memory(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	/* free memory for command */
	dma_free_attrs(&pdev->dev,
			MAX_CMD_SIZE * MAX_COMMANDS_PER_DEVICE,
			nvdla_dev->cmd_mem.va, nvdla_dev->cmd_mem.pa,
			__DMA_ATTR(attrs));

	nvdla_dev->cmd_mem.alloc_table = 0;

	return 0;
}

int nvdla_get_cmd_memory(struct platform_device *pdev,
		struct nvdla_cmd_mem_info *cmd_mem_info)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	int err = 0, index, offset;

	mutex_lock(&nvdla_dev->cmd_mem.lock);

	index = find_first_zero_bit(&nvdla_dev->cmd_mem.alloc_table,
			MAX_COMMANDS_PER_DEVICE);
	if (index >= MAX_COMMANDS_PER_DEVICE) {
		nvdla_dbg_err(pdev, "failed to get cmd mem from pool\n");
		err = -EAGAIN;
		goto err_get_mem;
	}

	/* assign mem */
	set_bit(index, &nvdla_dev->cmd_mem.alloc_table);

	offset = NVDLA_CMD_OFFSET(index);
	cmd_mem_info->va = nvdla_dev->cmd_mem.va + offset;
	cmd_mem_info->pa = nvdla_dev->cmd_mem.pa + offset;
	cmd_mem_info->index = index;

	/* check if IOVA is correctly aligned */
	if (cmd_mem_info->pa & 0xff) {
		err = -EFAULT;
		goto fail_to_aligned_dma;
	}
	memset(cmd_mem_info->va, 0, MAX_CMD_SIZE);

fail_to_aligned_dma:
err_get_mem:
	mutex_unlock(&nvdla_dev->cmd_mem.lock);
	return err;
}

int nvdla_put_cmd_memory(struct platform_device *pdev, int index)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	mutex_lock(&nvdla_dev->cmd_mem.lock);
	clear_bit(index, &nvdla_dev->cmd_mem.alloc_table);
	mutex_unlock(&nvdla_dev->cmd_mem.lock);

	return 0;
}

int nvdla_send_cmd(struct platform_device *pdev,
			struct nvdla_cmd_data *cmd_data)
{
	unsigned long timeout;
	int ret = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	uint32_t method_id = cmd_data->method_id;
	uint32_t method_data = cmd_data->method_data;
	bool wait = cmd_data->wait;

	mutex_lock(&nvdla_dev->cmd_lock);

	/*
	 * enable notification for command completion or error if
	 * wait if required
	 */
	if (wait)
		method_id |= (1 << DLA_INT_ON_COMPLETE_SHIFT) |
					(1 << DLA_INT_ON_ERROR_SHIFT);

	nvdla_dev->waiting = 1;

	nvdla_dbg_reg(pdev, "method_id=[0x%x]", method_id);
	host1x_writel(pdev, NV_DLA_THI_METHOD_ID, method_id);

	nvdla_dbg_reg(pdev, "method_data=[0x%x]", method_data);
	host1x_writel(pdev, NV_DLA_THI_METHOD_DATA, method_data);

	if (!wait) {
		nvdla_dev->waiting = 0;
		mutex_unlock(&nvdla_dev->cmd_lock);
		return 0;
	}

	timeout = msecs_to_jiffies(CMD_TIMEOUT_MSEC);

	if (!wait_for_completion_timeout(&nvdla_dev->cmd_completion, timeout)) {
		nvdla_dev->waiting = 0;
		mutex_unlock(&nvdla_dev->cmd_lock);
		return -ETIMEDOUT;
	}

	if (nvdla_dev->cmd_status != DLA_ERR_NONE) {
		nvdla_dbg_err(pdev, "Command %u failed\n", method_id);
		ret = -EINVAL;
	}

	/* Reset command status after use for next command */
	nvdla_dev->cmd_status = DLA_ERR_NONE;
	nvdla_dev->waiting = 0;

	mutex_unlock(&nvdla_dev->cmd_lock);

	return ret;
}

static int nvdla_set_gcov_region(struct platform_device *pdev, bool unset_region)
{
	int err = 0;
	struct nvdla_cmd_mem_info gcov_cmd_mem_info;
	struct nvdla_cmd_data cmd_data;
	struct dla_region_printf *gcov_region = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	if (!pdata->flcn_isr)
		return 0;

	err = nvhost_module_busy(pdev);
	if (err) {
		nvdla_dbg_err(pdev, "failed to power on\n");
		err = -ENODEV;
		goto fail_to_power_on;
	}

	/* assign memory for gcov command */
	err = nvdla_get_cmd_memory(pdev, &gcov_cmd_mem_info);
	if (err) {
		nvdla_dbg_err(pdev,
			"dma allocation failed for gcov command.");
		goto alloc_gcov_cmd_failed;
	}

	gcov_region = (struct dla_region_printf *)(gcov_cmd_mem_info.va);
	gcov_region->region = DLA_REGION_GCOV;
	if (nvdla_dev->submit_mode == NVDLA_SUBMIT_MODE_CHANNEL
		|| unset_region)
		gcov_region->address = 0;
	else
		gcov_region->address = nvdla_dev->gcov_dump_pa;
	gcov_region->size = GCOV_BUFFER_SIZE;

	cmd_data.method_id = DLA_CMD_SET_REGIONS;
	cmd_data.method_data = ALIGNED_DMA(gcov_cmd_mem_info.pa);
	cmd_data.wait = true;

	err = nvdla_send_cmd(pdev, &cmd_data);

	/* release memory allocated for gcov command */
	nvdla_put_cmd_memory(pdev, gcov_cmd_mem_info.index);

	if (err != 0) {
		nvdla_dbg_err(pdev, "failed to send gcov command");
		goto gcov_send_cmd_failed;
	}

	nvhost_module_idle(pdev);

	return err;

gcov_send_cmd_failed:
alloc_gcov_cmd_failed:
	nvhost_module_idle(pdev);
fail_to_power_on:
	return err;
}

int nvdla_free_gcov_region(struct platform_device *pdev, bool update_region)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	int ret = 0;

	if (update_region) {
		ret = nvdla_set_gcov_region(pdev, true);
		if (ret)
			return ret;
	}

	if (nvdla_dev->gcov_dump_pa) {
		dma_free_attrs(&pdev->dev, GCOV_BUFFER_SIZE,
			       nvdla_dev->gcov_dump_va,
			       nvdla_dev->gcov_dump_pa,
			       __DMA_ATTR(attrs));
		nvdla_dev->gcov_dump_va = NULL;
		nvdla_dev->gcov_dump_pa = 0;
	}

	return 0;
}

int nvdla_alloc_gcov_region(struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	/* Gcov buffer allocation must be done at once only. */
	if (!nvdla_dev->gcov_dump_va) {
		/* allocate gcov region */
		nvdla_dev->gcov_dump_va = dma_alloc_attrs(&pdev->dev,
				   GCOV_BUFFER_SIZE, &nvdla_dev->gcov_dump_pa,
				   GFP_KERNEL, __DMA_ATTR(attrs));

		if (!nvdla_dev->gcov_dump_va) {
			nvdla_dbg_err(pdev,
				"dma gcov memory allocation failed");
			err = -ENOMEM;
			goto fail_alloc_gcov_dma;
		}
	}
	err = nvdla_set_gcov_region(pdev, false);
	if (err)
		nvdla_free_gcov_region(pdev, false);

fail_alloc_gcov_dma:
	return err;
}

static int nvdla_alloc_trace_region(struct platform_device *pdev)
{
	int err = 0;
	struct nvdla_cmd_mem_info trace_cmd_mem_info;
	struct nvdla_cmd_data cmd_data;
	struct dla_region_printf *trace_region = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	if (!pdata->flcn_isr)
		return 0;

	/* Trace buffer allocation must be done at once only. */
	if (!nvdla_dev->trace_dump_va) {
		/* allocate trace region */
		nvdla_dev->trace_dump_va = dma_alloc_attrs(&pdev->dev,
				   TRACE_BUFFER_SIZE, &nvdla_dev->trace_dump_pa,
				   GFP_KERNEL, __DMA_ATTR(attrs));

		if (!nvdla_dev->trace_dump_va) {
			nvdla_dbg_err(pdev,
				"dma trace memory allocation failed");
			err = -ENOMEM;
			goto fail_alloc_trace_dma;
		}
	}

	/* assign memory for trace command */
	err = nvdla_get_cmd_memory(pdev, &trace_cmd_mem_info);
	if (err) {
		nvdla_dbg_err(pdev,
			"dma allocation failed for trace command.");
		goto alloc_trace_cmd_failed;
	}

	trace_region = (struct dla_region_printf *)(trace_cmd_mem_info.va);

	trace_region->region = DLA_REGION_TRACE;
	trace_region->address = nvdla_dev->trace_dump_pa;
	trace_region->size = TRACE_BUFFER_SIZE;
	if (nvdla_dev->submit_mode == NVDLA_SUBMIT_MODE_CHANNEL)
		trace_region->address = 0;

	cmd_data.method_id = DLA_CMD_SET_REGIONS;
	cmd_data.method_data = ALIGNED_DMA(trace_cmd_mem_info.pa);
	cmd_data.wait = true;

	err = nvdla_send_cmd(pdev, &cmd_data);

	/* release memory allocated for trace command */
	nvdla_put_cmd_memory(pdev, trace_cmd_mem_info.index);

	if (err != 0) {
		nvdla_dbg_err(pdev, "failed to send trace command");
		goto trace_send_cmd_failed;
	}

	return err;

trace_send_cmd_failed:
alloc_trace_cmd_failed:
	if (nvdla_dev->trace_dump_pa) {
		dma_free_attrs(&pdev->dev, TRACE_BUFFER_SIZE,
			nvdla_dev->trace_dump_va, nvdla_dev->trace_dump_pa,
			__DMA_ATTR(attrs));
		nvdla_dev->trace_dump_va = NULL;

		nvdla_dev->trace_dump_pa = 0;
	}
fail_alloc_trace_dma:

	return err;
}

static int nvdla_alloc_dump_region(struct platform_device *pdev)
{
	int err = 0;
	struct dla_region_printf *region;
	struct nvdla_cmd_mem_info debug_cmd_mem_info;
	struct nvdla_cmd_data cmd_data;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	if (!pdata->flcn_isr)
		return 0;

	nvdla_dbg_fn(pdev, "");

	/* allocate dump region only once */
	if (!nvdla_dev->debug_dump_va) {
		nvdla_dev->debug_dump_va = dma_alloc_attrs(&pdev->dev,
				   DEBUG_BUFFER_SIZE, &nvdla_dev->debug_dump_pa,
				   GFP_KERNEL, __DMA_ATTR(attrs));
		if (!nvdla_dev->debug_dump_va) {
			nvdla_dbg_err(pdev, "debug dump dma alloc failed");
			err = -ENOMEM;
			goto fail_to_alloc_debug_dump;
		}
	}

	/* assign memory for command */
	err = nvdla_get_cmd_memory(pdev, &debug_cmd_mem_info);
	if (err) {
		nvdla_dbg_err(pdev, "dma alloc for command failed");
		goto set_region_failed;
	}

	region = (struct dla_region_printf *)debug_cmd_mem_info.va;
	region->region = DLA_REGION_PRINTF;
	region->size = DEBUG_BUFFER_SIZE;
	region->address = nvdla_dev->debug_dump_pa;
	if (nvdla_dev->submit_mode == NVDLA_SUBMIT_MODE_CHANNEL)
		region->address = 0;

	/* prepare command data */
	cmd_data.method_id = DLA_CMD_SET_REGIONS;
	cmd_data.method_data = ALIGNED_DMA(debug_cmd_mem_info.pa);
	cmd_data.wait = true;

	/* pass dump region to falcon */
	err = nvdla_send_cmd(pdev, &cmd_data);

	/* release memory allocated for debug print command */
	nvdla_put_cmd_memory(pdev, debug_cmd_mem_info.index);

	if (err != 0) {
		nvdla_dbg_err(pdev, "failed to send printf command");
		goto region_send_cmd_failed;
	}

	return 0;

region_send_cmd_failed:
set_region_failed:
	if (nvdla_dev->debug_dump_pa) {
		dma_free_attrs(&pdev->dev, DEBUG_BUFFER_SIZE,
			nvdla_dev->debug_dump_va, nvdla_dev->debug_dump_pa,
			__DMA_ATTR(attrs));
		nvdla_dev->debug_dump_va = NULL;
		nvdla_dev->debug_dump_pa = 0;
	}
fail_to_alloc_debug_dump:
	return err;
}

int nvdla_send_gos_region(struct platform_device *pdev)
{
	int i;
	int err;
	int num_grids;
	struct flcn *m;
	dma_addr_t *dla_grid;
	struct nvdla_cmd_data cmd_data;
	struct dla_region_gos *gos_region = NULL;
	struct nvdla_cmd_mem_info gos_cmd_mem_info;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	nvdla_dbg_fn(pdev, "");

	m = get_flcn(pdev);
	if (!m) {
		nvdla_dbg_err(pdev, "falcon is not booted!");
		err = -ENXIO;
		goto falcon_not_booted;
	}

	err = nvhost_syncpt_get_cv_dev_address_table(pdev, &num_grids,
				&dla_grid);
	if (err) {
		nvdla_dbg_err(pdev, "failed to get grid[%d]", err);
		nvdla_dev->is_gos_fetched = false;
		goto fail_to_get_grid;
	}

	if (num_grids > MAX_NUM_GRIDS) {
		nvdla_dbg_err(pdev, "num_grid[%d] > than [%d]", num_grids,
				MAX_NUM_GRIDS);
		nvdla_dev->is_gos_fetched = false;
		err = -EINVAL;
		goto fail_to_get_grid;
	}
	nvdla_dev->is_gos_fetched = true;

	/* assign memory for GoS set command */
	err = nvdla_get_cmd_memory(pdev, &gos_cmd_mem_info);
	if (err) {
		nvdla_dbg_err(pdev,
				"dma allocation failed for gos set command.");
		goto alloc_gos_cmd_failed;
	}

	/* set GoS region info */
	gos_region = (struct dla_region_gos *)(gos_cmd_mem_info.va);
	gos_region->region = DLA_REGION_GOS;
	gos_region->num_grids = num_grids;
	gos_region->grid_size = MAX_GRID_SIZE;
	for (i = 0; i < num_grids; i++)
		gos_region->address[i] = dla_grid[i];

	/* set cmd info */
	cmd_data.method_id = DLA_CMD_SET_REGIONS;
	cmd_data.method_data = ALIGNED_DMA(gos_cmd_mem_info.pa);
	cmd_data.wait = true;

	/* send cmd to set gos region */
	err = nvdla_send_cmd(pdev, &cmd_data);

	/* release memory allocated for set GoS command */
	nvdla_put_cmd_memory(pdev, gos_cmd_mem_info.index);

	if (err != 0) {
		nvdla_dbg_err(pdev, "failed to send set gos region command");
		goto gos_set_cmd_failed;
	}

	nvdla_dbg_fn(pdev, "send set gos region done.");

gos_set_cmd_failed:
alloc_gos_cmd_failed:
fail_to_get_grid:
falcon_not_booted:
	return err;
}

/* power management API */
int nvhost_nvdla_finalize_poweron(struct platform_device *pdev)
{
	int ret;
	uint32_t fw_ver_read_bin;
	uint32_t firmware_version;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	nvdla_dbg_fn(pdev, "");

	ret = nvhost_flcn_finalize_poweron(pdev);
	if (ret) {
		nvdla_dbg_err(pdev, "failed to poweron\n");
		goto fail;
	}

	fw_ver_read_bin = host1x_readl(pdev, NV_DLA_OS_VERSION);
	firmware_version = dla_version();

	if ((firmware_version & 0xffff00) != (fw_ver_read_bin & 0xffff00)) {
		nvdla_dbg_err(pdev,
		"Fw version of kernel [%u.%u.%u] doesn't match with actual version[%u.%u.%u]",
		(firmware_version >> 16) & 0xff, (firmware_version >> 8) & 0xff, firmware_version & 0xff,
		(fw_ver_read_bin >> 16 ) & 0xff, (fw_ver_read_bin >> 8) & 0xff, fw_ver_read_bin & 0xff);

		ret = -EINVAL;
		goto fail_to_val_ver;
	}

	nvdla_dbg_info(pdev, "Fw version : [%u.%u.%u]\n",
		(fw_ver_read_bin >> 16) & 0xff,
		(fw_ver_read_bin >> 8) & 0xff,
		fw_ver_read_bin & 0xff);

	nvdla_dev->fw_version = fw_ver_read_bin;

	ret = nvdla_alloc_dump_region(pdev);
	if (ret) {
		nvdla_dbg_err(pdev, "fail alloc dump region\n");
		goto fail_to_alloc_dump_reg;
	}

	ret = nvdla_alloc_trace_region(pdev);
	if (ret) {
		nvdla_dbg_err(pdev, "fail alloc trace region\n");
		goto fail_to_alloc_trace;
	}

	/* Disable GOS until it is fixed in Kernel 4.14 */
	nvdla_dev->is_gos_enabled = false;
	nvdla_dev->is_gos_fetched = true;

	if (nvdla_dev->quirks & NVDLA_QUIRK_T194_A01_WAR) {
		host1x_writel(pdev,
			NVDLA_MCIF_CFG_OUTSTANDING_CNT_0_OFFSET, 0xff);
		nvdla_dbg_info(pdev, "enabled T194-A01 WAR");
	}
	return 0;

fail_to_alloc_trace:
fail_to_alloc_dump_reg:
fail_to_val_ver:
	nvhost_nvdla_prepare_poweroff(pdev);
fail:
	return ret;
}

int nvhost_nvdla_prepare_poweroff(struct platform_device *pdev)
{
	int ret;

	nvdla_dbg_fn(pdev, "");

	ret = nvhost_flcn_prepare_poweroff(pdev);
	if (ret) {
		nvdla_dbg_err(pdev, "failed to poweroff\n");
		goto out;
	}

out:
	return ret;
}

/* driver probe and init */
static struct of_device_id tegra_nvdla_of_match[] = {
	{
		.name = "nvdla0",
		.compatible = "nvidia,tegra194-nvdla",
		.data = (struct nvhost_device_data *)&t19_nvdla0_info },
	{
		.name = "nvdla1",
		.compatible = "nvidia,tegra194-nvdla",
		.data = (struct nvhost_device_data *)&t19_nvdla1_info },
	{ },
};

static int nvdla_probe(struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;
	struct nvdla_device *nvdla_dev = NULL;
	struct device *dev = &pdev->dev;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvdla_of_match, dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else {
		pdata = (struct nvhost_device_data *)pdev->dev.platform_data;
	}

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(dev, "no platform data\n");
		err = -ENODATA;
		goto err_get_pdata;
	}

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA19 &&
		tegra_get_sku_id() == 0x9E) {
		dev_err(dev, "NVDLA IP is disabled in SKU\n");
		err = -ENODEV;
		goto err_no_ip;
	}

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA19 &&
		tegra_get_sku_id() == 0x9F &&
		pdata->class == NV_DLA1_CLASS_ID) {
		dev_err(dev, "NVDLA1 IP is disabled in SKU\n");
		err = -ENODEV;
		goto err_no_ip;
	}

	dma_set_mask(dev, DMA_BIT_MASK(37));

	nvdla_dev = devm_kzalloc(dev, sizeof(*nvdla_dev), GFP_KERNEL);
	if (!nvdla_dev) {
		err = -ENOMEM;
		goto err_alloc_nvdla;
	}

	nvdla_dev->pdev = pdev;
	pdata->pdev = pdev;
	mutex_init(&pdata->lock);
	mutex_init(&nvdla_dev->cmd_lock);
	init_completion(&nvdla_dev->cmd_completion);
	pdata->private_data = nvdla_dev;
	platform_set_drvdata(pdev, pdata);
	nvdla_dev->dbg_mask = debug_err;

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto err_get_resources;

	err = nvhost_module_init(pdev);
	if (err)
		goto err_module_init;

	err = nvhost_client_device_init(pdev);
	if (err)
		goto err_client_device_init;

	/* create debugfs entries */
	nvdla_debug_init(pdev);

	if (pdata->flcn_isr)
		flcn_intr_init(pdev);

	nvdla_dev->pool = nvdla_queue_init(pdev, &nvdla_queue_ops,
				MAX_NVDLA_QUEUE_COUNT);
	if (IS_ERR(nvdla_dev->pool)) {
		err = PTR_ERR(nvdla_dev->pool);
		goto err_queue_init;
	}

	/* init reset handler workqueue */
	nvdla_reset_handler_init(nvdla_dev);

	err = nvhost_syncpt_unit_interface_init(pdev);
	if (err)
		goto err_mss_init;

	err = nvdla_alloc_cmd_memory(pdev);
	if (err)
		goto err_alloc_cmd_mem;

	nvdla_dbg_info(pdev, "pdata:%p initialized\n", pdata);

	return 0;
err_alloc_cmd_mem:
err_mss_init:
	nvdla_queue_deinit(nvdla_dev->pool);
err_queue_init:
	nvhost_client_device_release(pdev);
err_client_device_init:
	nvhost_module_deinit(pdev);
err_module_init:
err_get_resources:
	devm_kfree(dev, nvdla_dev);
err_alloc_nvdla:
err_no_ip:
err_get_pdata:

	return err;
}

static int __exit nvdla_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	nvdla_queue_deinit(nvdla_dev->pool);
	nvhost_client_device_release(pdev);

	nvdla_free_gcov_region(pdev, false);

	if (nvdla_dev->trace_dump_pa) {
		dma_free_attrs(&pdev->dev, TRACE_BUFFER_SIZE,
			       nvdla_dev->trace_dump_va,
			       nvdla_dev->trace_dump_pa,
			       __DMA_ATTR(attrs));
		nvdla_dev->trace_dump_va = NULL;
		nvdla_dev->trace_dump_pa = 0;
	}

	if (nvdla_dev->debug_dump_pa) {
		dma_free_attrs(&pdev->dev, DEBUG_BUFFER_SIZE,
			       nvdla_dev->debug_dump_va,
			       nvdla_dev->debug_dump_pa,
			       __DMA_ATTR(attrs));
		nvdla_dev->debug_dump_va = NULL;
		nvdla_dev->debug_dump_pa = 0;
	}

	/* free command mem in last */
	nvdla_free_cmd_memory(pdev);

	nvdla_dbg_fn(pdev, "");

	return 0;
}

static struct platform_driver nvdla_driver = {
	.probe = nvdla_probe,
	.remove = __exit_p(nvdla_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvdla",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvdla_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

module_platform_driver(nvdla_driver);

MODULE_AUTHOR("Shridhar Rasal <srasal@nvidia.com>");
