/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <nvgpu/nvhost.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/enabled.h>
#include <nvgpu/nvlink.h>
#include <nvgpu/soc.h>
#include <nvgpu/sim.h>
#include <nvgpu/gk20a.h>

#include "nvlink.h"
#include "clk/clk.h"
#include "clk/clk_mclk.h"
#include "module.h"
#include "intr.h"
#include "sysfs.h"
#include "os_linux.h"
#include "platform_gk20a.h"

#include "pci.h"
#include "pci_usermode.h"

#include "driver_common.h"

#define PCI_INTERFACE_NAME "card-%s%%s"

static int nvgpu_pci_tegra_probe(struct device *dev)
{
	return 0;
}

static int nvgpu_pci_tegra_remove(struct device *dev)
{
	return 0;
}

static bool nvgpu_pci_tegra_is_railgated(struct device *pdev)
{
	return false;
}

static long nvgpu_pci_clk_round_rate(struct device *dev, unsigned long rate)
{
	long ret = (long)rate;

	if (rate == UINT_MAX)
		ret = BOOT_GPC2CLK_MHZ * 1000000UL;

	return ret;
}

static struct gk20a_platform nvgpu_pci_device[] = {
	{ /* DEVICE=0x1c35 */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = true,
	.enable_elpg = true,
	.enable_elcg = false,
	.enable_slcg = true,
	.enable_blcg = true,
	.enable_mscg = true,
	.can_slcg    = true,
	.can_blcg    = true,
	.can_elcg    = true,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x86063000,
	.hardcode_sw_threshold = true,
	.ina3221_dcb_index = 0,
	.ina3221_i2c_address = 0x84,
	.ina3221_i2c_port = 0x2,
	},
	{ /* DEVICE=0x1c36 */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = true,
	.enable_elpg = true,
	.enable_elcg = false,
	.enable_slcg = true,
	.enable_blcg = true,
	.enable_mscg = true,
	.can_slcg    = true,
	.can_blcg    = true,
	.can_elcg    = true,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x86062d00,
	.hardcode_sw_threshold = true,
	.ina3221_dcb_index = 0,
	.ina3221_i2c_address = 0x84,
	.ina3221_i2c_port = 0x2,
	},
	{ /* DEVICE=0x1c37 */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = true,
	.enable_elpg = true,
	.enable_elcg = false,
	.enable_slcg = true,
	.enable_blcg = true,
	.enable_mscg = true,
	.can_slcg    = true,
	.can_blcg    = true,
	.can_elcg    = true,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x86063000,
	.hardcode_sw_threshold = true,
	.ina3221_dcb_index = 0,
	.ina3221_i2c_address = 0x84,
	.ina3221_i2c_port = 0x2,
	},
	{ /* DEVICE=0x1c75 */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = true,
	.enable_elpg = true,
	.enable_elcg = false,
	.enable_slcg = true,
	.enable_blcg = true,
	.enable_mscg = true,
	.can_slcg    = true,
	.can_blcg    = true,
	.can_elcg    = true,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x86065300,
	.hardcode_sw_threshold = false,
	.ina3221_dcb_index = 1,
	.ina3221_i2c_address = 0x80,
	.ina3221_i2c_port = 0x1,
	},
	{ /* DEVICE=PG503 SKU 201 */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = false,
	.enable_elpg = false,
	.enable_elcg = false,
	.enable_slcg = false,
	.enable_blcg = false,
	.enable_mscg = false,
	.can_slcg    = false,
	.can_blcg    = false,
	.can_elcg    = false,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x88001e00,
	.hardcode_sw_threshold = false,
	.run_preos = true,
	},
	{ /* DEVICE=PG503 SKU 200 ES */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = false,
	.enable_elpg = false,
	.enable_elcg = false,
	.enable_slcg = false,
	.enable_blcg = false,
	.enable_mscg = false,
	.can_slcg    = false,
	.can_blcg    = false,
	.can_elcg    = false,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x88001e00,
	.hardcode_sw_threshold = false,
	.run_preos = true,
	},
	{
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = false,
	.enable_elpg = false,
	.enable_elcg = false,
	.enable_slcg = false,
	.enable_blcg = false,
	.enable_mscg = false,
	.can_slcg    = false,
	.can_blcg    = false,
	.can_elcg    = false,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x88000126,
	.hardcode_sw_threshold = false,
	.run_preos = true,
	.has_syncpoints = true,
	},
	{ /* SKU250 */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = false,
	.enable_elpg = false,
	.enable_elcg = false,
	.enable_slcg = true,
	.enable_blcg = true,
	.enable_mscg = false,
	.can_slcg    = true,
	.can_blcg    = true,
	.can_elcg    = false,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	.ch_wdt_timeout_ms = 7000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x1,
	.hardcode_sw_threshold = false,
	.run_preos = true,
	.has_syncpoints = true,
	},
	{ /* SKU 0x1e3f */
	/* ptimer src frequency in hz */
	.ptimer_src_freq	= 31250000,

	.probe = nvgpu_pci_tegra_probe,
	.remove = nvgpu_pci_tegra_remove,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= false,
	.can_elpg_init = false,
	.enable_elpg = false,
	.enable_elcg = false,
	.enable_slcg = false,
	.enable_blcg = false,
	.enable_mscg = false,
	.can_slcg    = false,
	.can_blcg    = false,
	.can_elcg    = false,

	.disable_aspm = true,

	/* power management callbacks */
	.is_railgated = nvgpu_pci_tegra_is_railgated,
	.clk_round_rate = nvgpu_pci_clk_round_rate,

	/*
	 * WAR: PCIE X1 is very slow, set to very high value till nvlink is up
	 */
	.ch_wdt_timeout_ms = 30000,

	.honors_aperture = true,
	.dma_mask = DMA_BIT_MASK(40),
	.vbios_min_version = 0x1,
	.hardcode_sw_threshold = false,
	.unified_memory = false,
	},

};

static struct pci_device_id nvgpu_pci_table[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1c35),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 0,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1c36),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 1,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1c37),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 2,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1c75),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 3,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1db1),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 4,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1db0),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 5,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1dbe),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 6,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1df1),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 7,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_NVIDIA, 0x1e3f),
		.class = PCI_BASE_CLASS_DISPLAY << 16,
		.class_mask = 0xff << 16,
		.driver_data = 8,
	},
	{}
};

static irqreturn_t nvgpu_pci_isr(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;
	irqreturn_t ret_stall;
	irqreturn_t ret_nonstall;

	ret_stall = nvgpu_intr_stall(g);
	ret_nonstall = nvgpu_intr_nonstall(g);

#if defined(CONFIG_PCI_MSI)
	/* Send MSI EOI */
	if (g->ops.xve.rearm_msi && g->msi_enabled)
		g->ops.xve.rearm_msi(g);
#endif

	return (ret_stall == IRQ_NONE) ? ret_nonstall : IRQ_WAKE_THREAD;
}

static irqreturn_t nvgpu_pci_intr_thread(int irq, void *dev_id)
{
	struct gk20a *g = dev_id;

	return nvgpu_intr_thread_stall(g);
}

static int nvgpu_pci_init_support(struct pci_dev *pdev)
{
	int err = 0;
	struct gk20a *g = get_gk20a(&pdev->dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct device *dev = &pdev->dev;

	l->regs = nvgpu_devm_ioremap(dev, pci_resource_start(pdev, 0),
				     pci_resource_len(pdev, 0));
	if (IS_ERR(l->regs)) {
		nvgpu_err(g, "failed to remap gk20a registers");
		err = PTR_ERR(l->regs);
		goto fail;
	}

	l->regs_bus_addr = pci_resource_start(pdev, 0);
	if (!l->regs_bus_addr) {
		nvgpu_err(g, "failed to read register bus offset");
		err = -ENODEV;
		goto fail;
	}

	l->bar1 = nvgpu_devm_ioremap(dev, pci_resource_start(pdev, 1),
				     pci_resource_len(pdev, 1));
	if (IS_ERR(l->bar1)) {
		nvgpu_err(g, "failed to remap gk20a bar1");
		err = PTR_ERR(l->bar1);
		goto fail;
	}

	err = nvgpu_init_sim_support_linux_pci(g);
	if (err)
		goto fail;
	err = nvgpu_init_sim_support_pci(g);
	if (err)
		goto fail_sim;

	nvgpu_pci_init_usermode_support(l);

	return 0;

 fail_sim:
	nvgpu_remove_sim_support_linux_pci(g);
 fail:
	if (l->regs)
		l->regs = NULL;

	if (l->bar1)
		l->bar1 = NULL;

	return err;
}

static char *nvgpu_pci_devnode(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = S_IRUGO | S_IWUGO;
	return kasprintf(GFP_KERNEL, "nvgpu-pci/%s", dev_name(dev));
}

static struct class nvgpu_pci_class = {
	.owner = THIS_MODULE,
	.name = "nvidia-pci-gpu",
	.devnode = nvgpu_pci_devnode,
};

#ifdef CONFIG_PM
static int nvgpu_pci_pm_runtime_resume(struct device *dev)
{
	return gk20a_pm_finalize_poweron(dev);
}

static int nvgpu_pci_pm_runtime_suspend(struct device *dev)
{
	return 0;
}

static int nvgpu_pci_pm_resume(struct device *dev)
{
	return gk20a_pm_finalize_poweron(dev);
}

static int nvgpu_pci_pm_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops nvgpu_pci_pm_ops = {
	.runtime_resume = nvgpu_pci_pm_runtime_resume,
	.runtime_suspend = nvgpu_pci_pm_runtime_suspend,
	.resume = nvgpu_pci_pm_resume,
	.suspend = nvgpu_pci_pm_suspend,
};
#endif

static int nvgpu_pci_pm_init(struct device *dev)
{
#ifdef CONFIG_PM
	struct gk20a *g = get_gk20a(dev);

	if (!nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE)) {
		pm_runtime_disable(dev);
	} else {
		if (g->railgate_delay)
			pm_runtime_set_autosuspend_delay(dev,
				g->railgate_delay);

		/*
		 * set gpu dev's use_autosuspend flag to allow
		 * runtime power management of GPU
		 */
		pm_runtime_use_autosuspend(dev);

		/*
		 * runtime PM for PCI devices is forbidden
		 * by default, so unblock RTPM of GPU
		 */
		pm_runtime_put_noidle(dev);
		pm_runtime_allow(dev);
	}
#endif
	return 0;
}

static int nvgpu_pci_pm_deinit(struct device *dev)
{
#ifdef CONFIG_PM
	struct gk20a *g = get_gk20a(dev);

	if (!nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE))
		pm_runtime_enable(dev);
	else
		pm_runtime_forbid(dev);
#endif
	return 0;
}

static int nvgpu_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *pent)
{
	struct gk20a_platform *platform = NULL;
	struct nvgpu_os_linux *l;
	struct gk20a *g;
	int err;
	char nodefmt[64];
	struct device_node *np;

	/* make sure driver_data is a sane index */
	if (pent->driver_data >= sizeof(nvgpu_pci_device) /
				 sizeof(nvgpu_pci_device[0])) {
		return -EINVAL;
	}

	l = kzalloc(sizeof(*l), GFP_KERNEL);
	if (!l) {
		dev_err(&pdev->dev, "couldn't allocate gk20a support");
		return -ENOMEM;
	}

	hash_init(l->ecc_sysfs_stats_htable);

	g = &l->g;

	g->log_mask = NVGPU_DEFAULT_DBG_MASK;

	nvgpu_init_gk20a(g);

	nvgpu_kmem_init(g);

	/* Allocate memory to hold platform data*/
	platform = (struct gk20a_platform *)nvgpu_kzalloc( g,
			sizeof(struct gk20a_platform));
	if (!platform) {
		dev_err(&pdev->dev, "couldn't allocate platform data");
		err = -ENOMEM;
		goto err_free_l;
	}

	/* copy detected device data to allocated platform space*/
	memcpy((void *)platform, (void *)&nvgpu_pci_device[pent->driver_data],
		sizeof(struct gk20a_platform));

	pci_set_drvdata(pdev, platform);

	err = nvgpu_init_enabled_flags(g);
	if (err)
		goto err_free_platform;

	platform->g = g;
	l->dev = &pdev->dev;

	np = nvgpu_get_node(g);
	if (of_dma_is_coherent(np)) {
		__nvgpu_set_enabled(g, NVGPU_USE_COHERENT_SYSMEM, true);
		__nvgpu_set_enabled(g, NVGPU_SUPPORT_IO_COHERENCE, true);
	}

	err = pci_enable_device(pdev);
	if (err)
		goto err_free_platform;
	pci_set_master(pdev);

	g->pci_vendor_id = pdev->vendor;
	g->pci_device_id = pdev->device;
	g->pci_subsystem_vendor_id = pdev->subsystem_vendor;
	g->pci_subsystem_device_id = pdev->subsystem_device;
	g->pci_class = (pdev->class >> 8) & 0xFFFFU; // we only want base/sub
	g->pci_revision = pdev->revision;

	g->ina3221_dcb_index = platform->ina3221_dcb_index;
	g->ina3221_i2c_address = platform->ina3221_i2c_address;
	g->ina3221_i2c_port = platform->ina3221_i2c_port;
	g->hardcode_sw_threshold = platform->hardcode_sw_threshold;

#if defined(CONFIG_PCI_MSI)
	err = pci_enable_msi(pdev);
	if (err) {
		nvgpu_err(g,
			"MSI could not be enabled, falling back to legacy");
		g->msi_enabled = false;
	} else
		g->msi_enabled = true;
#endif

	g->irq_stall = pdev->irq;
	g->irq_nonstall = pdev->irq;
	if (g->irq_stall < 0) {
		err = -ENXIO;
		goto err_disable_msi;
	}

	err = devm_request_threaded_irq(&pdev->dev,
			g->irq_stall,
			nvgpu_pci_isr,
			nvgpu_pci_intr_thread,
#if defined(CONFIG_PCI_MSI)
			g->msi_enabled ? 0 :
#endif
			IRQF_SHARED, "nvgpu", g);
	if (err) {
		nvgpu_err(g,
			"failed to request irq @ %d", g->irq_stall);
		goto err_disable_msi;
	}
	disable_irq(g->irq_stall);

	err = nvgpu_pci_init_support(pdev);
	if (err)
		goto err_free_irq;

	if (strchr(dev_name(&pdev->dev), '%')) {
		nvgpu_err(g, "illegal character in device name");
		err = -EINVAL;
		goto err_free_irq;
	}

	snprintf(nodefmt, sizeof(nodefmt),
		 PCI_INTERFACE_NAME, dev_name(&pdev->dev));

	err = nvgpu_probe(g, "gpu_pci", nodefmt, &nvgpu_pci_class);
	if (err)
		goto err_free_irq;

	err = nvgpu_pci_pm_init(&pdev->dev);
	if (err) {
		nvgpu_err(g, "pm init failed");
		goto err_free_irq;
	}

	err = nvgpu_nvlink_probe(g);
	/*
	 * ENODEV is a legal error which means there is no NVLINK
	 * any other error is fatal
	 */
	if (err) {
		if (err != -ENODEV) {
			nvgpu_err(g, "fatal error probing nvlink, bailing out");
			goto err_free_irq;
		}
		/* Enable Semaphore SHIM on nvlink only for now. */
		__nvgpu_set_enabled(g, NVGPU_SUPPORT_NVLINK, false);
		__nvgpu_set_enabled(g, NVGPU_HAS_SYNCPOINTS, false);
	} else {
		err = nvgpu_nvhost_syncpt_init(g);
		if (err) {
			if (err != -ENOSYS) {
				nvgpu_err(g, "syncpt init failed");
				goto err_free_irq;
			}
		}
	}

	return 0;

err_free_irq:
	nvgpu_free_irq(g);
err_disable_msi:
#if defined(CONFIG_PCI_MSI)
	if (g->msi_enabled)
		pci_disable_msi(pdev);
#endif
err_free_platform:
	nvgpu_kfree(g, platform);
err_free_l:
	kfree(l);
	return err;
}

static void nvgpu_pci_remove(struct pci_dev *pdev)
{
	struct gk20a *g = get_gk20a(&pdev->dev);
	struct device *dev = dev_from_gk20a(g);
	int err;

	/* no support yet for unbind if DGPU is in VGPU mode */
	if (gk20a_gpu_is_virtual(dev))
		return;

	err = nvgpu_nvlink_deinit(g);
	WARN(err, "gpu failed to remove nvlink");

	gk20a_driver_start_unload(g);

	err = nvgpu_quiesce(g);
	/* TODO: handle failure to idle */
	WARN(err, "gpu failed to idle during driver removal");

	nvgpu_free_irq(g);

	nvgpu_remove(dev, &nvgpu_pci_class);

#if defined(CONFIG_PCI_MSI)
	if (g->msi_enabled)
		pci_disable_msi(pdev);
	else {
		/* IRQ does not need to be enabled in MSI as the line is not
		 * shared
		 */
		enable_irq(g->irq_stall);
	}
#endif
	nvgpu_pci_pm_deinit(&pdev->dev);

	/* free allocated platform data space */
	gk20a_get_platform(&pdev->dev)->g = NULL;
	nvgpu_kfree(g, gk20a_get_platform(&pdev->dev));

	gk20a_put(g);
}

static struct pci_driver nvgpu_pci_driver = {
	.name = "nvgpu",
	.id_table = nvgpu_pci_table,
	.probe = nvgpu_pci_probe,
	.remove = nvgpu_pci_remove,
#ifdef CONFIG_PM
	.driver.pm = &nvgpu_pci_pm_ops,
#endif
};

int __init nvgpu_pci_init(void)
{
	int ret;

	ret = class_register(&nvgpu_pci_class);
	if (ret)
		return ret;

	return pci_register_driver(&nvgpu_pci_driver);
}

void __exit nvgpu_pci_exit(void)
{
	pci_unregister_driver(&nvgpu_pci_driver);
	class_unregister(&nvgpu_pci_class);
}
