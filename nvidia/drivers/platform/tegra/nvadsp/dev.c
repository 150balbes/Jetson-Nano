/*
 * dev.c
 *
 * A device driver for ADSP and APE
 *
 * Copyright (C) 2014-2018, NVIDIA Corporation. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/io.h>
#include <linux/tegra_nvadsp.h>
#include <soc/tegra/chip-id.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>
#include <linux/clk/tegra.h>
#include <linux/delay.h>
#include <asm/arch_timer.h>
#include <linux/irqchip/tegra-agic.h>

#include "dev.h"
#include "os.h"
#include "amc.h"
#include "ape_actmon.h"
#include "aram_manager.h"

#include "dev-t21x.h"
#include "dev-t18x.h"

static struct nvadsp_drv_data *nvadsp_drv_data;

#ifdef CONFIG_DEBUG_FS
static int __init adsp_debug_init(struct nvadsp_drv_data *drv_data)
{
	drv_data->adsp_debugfs_root = debugfs_create_dir("tegra_ape", NULL);
	if (!drv_data->adsp_debugfs_root)
		return -ENOMEM;
	return 0;
}
#endif /* CONFIG_DEBUG_FS */


#ifdef CONFIG_PM
static int nvadsp_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int ret = -EINVAL;

	if (drv_data->runtime_resume)
		ret = drv_data->runtime_resume(dev);

	return ret;
}

static int nvadsp_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int ret = -EINVAL;

	if (drv_data->runtime_suspend)
		ret = drv_data->runtime_suspend(dev);

	return ret;
}

static int nvadsp_runtime_idle(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int ret = 0;

	if (drv_data->runtime_idle)
		ret = drv_data->runtime_idle(dev);

	return ret;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM_SLEEP
static int nvadsp_suspend(struct device *dev)
{
	if (pm_runtime_status_suspended(dev))
		return 0;

	return nvadsp_runtime_suspend(dev);
}

static int nvadsp_resume(struct device *dev)
{
	if (pm_runtime_status_suspended(dev))
		return 0;

	return nvadsp_runtime_resume(dev);
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops nvadsp_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(nvadsp_suspend, nvadsp_resume)
	SET_RUNTIME_PM_OPS(nvadsp_runtime_suspend, nvadsp_runtime_resume,
			   nvadsp_runtime_idle)
};

uint64_t nvadsp_get_timestamp_counter(void)
{
	return arch_counter_get_cntvct();
}
EXPORT_SYMBOL(nvadsp_get_timestamp_counter);

static void __init nvadsp_parse_clk_entries(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 val32 = 0;


	/* Optional properties, should come from platform dt files */
	if (of_property_read_u32(dev->of_node, "nvidia,adsp_freq", &val32))
		dev_dbg(dev, "adsp_freq dt not found\n");
	else {
		drv_data->adsp_freq = val32;
		drv_data->adsp_freq_hz = val32 * 1000;
	}

	if (of_property_read_u32(dev->of_node, "nvidia,ape_freq", &val32))
		dev_dbg(dev, "ape_freq dt not found\n");
	else
		drv_data->ape_freq = val32;

	if (of_property_read_u32(dev->of_node, "nvidia,ape_emc_freq", &val32))
		dev_dbg(dev, "ape_emc_freq dt not found\n");
	else
		drv_data->ape_emc_freq = val32;
}

static int __init nvadsp_parse_dt(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 *adsp_reset;
	u32 *adsp_mem;
	int iter;

	adsp_reset = drv_data->unit_fpga_reset;
	adsp_mem = drv_data->adsp_mem;

	for (iter = 0; iter < ADSP_MEM_END; iter++) {
		if (of_property_read_u32_index(dev->of_node, "nvidia,adsp_mem",
			iter, &adsp_mem[iter])) {
			dev_err(dev, "adsp memory dt %d not found\n", iter);
			return -EINVAL;
		}
	}

	for (iter = 0; iter < ADSP_EVP_END; iter++) {
		if (of_property_read_u32_index(dev->of_node,
			"nvidia,adsp-evp-base",
			iter, &drv_data->evp_base[iter])) {
			dev_err(dev, "adsp memory dt %d not found\n", iter);
			return -EINVAL;
		}
	}

	drv_data->adsp_unit_fpga = of_property_read_bool(dev->of_node,
				"nvidia,adsp_unit_fpga");

	drv_data->adsp_os_secload = of_property_read_bool(dev->of_node,
				"nvidia,adsp_os_secload");

	if (drv_data->adsp_unit_fpga) {
		for (iter = 0; iter < ADSP_UNIT_FPGA_RESET_END; iter++) {
			if (of_property_read_u32_index(dev->of_node,
				"nvidia,adsp_unit_fpga_reset", iter,
				&adsp_reset[iter])) {
				dev_err(dev, "adsp reset dt %d not found\n",
					iter);
				return -EINVAL;
			}
		}
	}
	nvadsp_parse_clk_entries(pdev);

	drv_data->state.evp = devm_kzalloc(dev,
			drv_data->evp_base[ADSP_EVP_SIZE], GFP_KERNEL);
	if (!drv_data->state.evp)
		return -ENOMEM;

	return 0;
}

static int __init nvadsp_probe(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev = &pdev->dev;
	struct resource *res = NULL;
	void __iomem *base = NULL;
	uint32_t aram_addr;
	uint32_t aram_size;
	int dram_iter;
	int irq_iter;
	int ret = 0;
	int iter;

	dev_info(dev, "in probe()...\n");

	drv_data = devm_kzalloc(dev, sizeof(*drv_data),
				GFP_KERNEL);
	if (!drv_data) {
		dev_err(&pdev->dev, "Failed to allocate driver data");
		ret = -ENOMEM;
		goto out;
	}

	platform_set_drvdata(pdev, drv_data);
	drv_data->pdev = pdev;
	drv_data->chip_data = of_device_get_match_data(dev);

	ret = nvadsp_parse_dt(pdev);
	if (ret)
		goto out;

#ifdef CONFIG_PM
	ret = nvadsp_pm_init(pdev);
	if (ret) {
		dev_err(dev, "Failed in pm init");
		goto out;
	}
#endif

#ifdef CONFIG_DEBUG_FS
	if (adsp_debug_init(drv_data))
		dev_err(dev,
			"unable to create tegra_ape debug fs directory\n");
#endif

	drv_data->base_regs =
		devm_kzalloc(dev, sizeof(void *) * APE_MAX_REG,
							GFP_KERNEL);
	if (!drv_data->base_regs) {
		dev_err(dev, "Failed to allocate regs");
		ret = -ENOMEM;
		goto out;
	}

	for (iter = 0; iter < APE_MAX_REG; iter++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, iter);
		if (!res) {
			dev_err(dev,
			"Failed to get resource with ID %d\n",
							iter);
			ret = -EINVAL;
			goto out;
		}

		if (!drv_data->adsp_unit_fpga && iter == UNIT_FPGA_RST)
			continue;

		/*
		 * skip if the particular module is not present in a
		 * generation, for which the register start address
		 * is made 0 from dt.
		 */
		if (res->start == 0)
			continue;

		base = devm_ioremap_resource(dev, res);
		if (IS_ERR(base)) {
			dev_err(dev, "Failed to iomap resource reg[%d]\n",
				iter);
			ret = PTR_ERR(base);
			goto out;
		}
		drv_data->base_regs[iter] = base;
		nvadsp_add_load_mappings(res->start, base,
						resource_size(res));
	}

	drv_data->base_regs_saved = drv_data->base_regs;

	for (dram_iter = 0; dram_iter < ADSP_MAX_DRAM_MAP; dram_iter++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, iter++);
		if (!res) {
			dev_err(dev,
			"Failed to get DRAM map with ID %d\n", iter);
			ret = -EINVAL;
			goto out;
		}

		drv_data->dram_region[dram_iter] = res;
	}

	for (irq_iter = 0; irq_iter < NVADSP_VIRQ_MAX; irq_iter++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, irq_iter);
		if (!res) {
			dev_err(dev, "Failed to get irq number for index %d\n",
				irq_iter);
			ret = -EINVAL;
			goto out;
		}
		drv_data->agic_irqs[irq_iter] = res->start;
	}

	nvadsp_drv_data = drv_data;

#ifdef CONFIG_PM
	tegra_pd_add_device(dev);

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto out;
#endif
	ret = nvadsp_hwmbox_init(pdev);
	if (ret)
		goto err;

	ret = nvadsp_mbox_init(pdev);
	if (ret)
		goto err;

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ret = ape_actmon_probe(pdev);
	if (ret)
		goto err;
#endif

	ret = nvadsp_os_probe(pdev);
	if (ret)
		goto err;

	ret = nvadsp_reset_init(pdev);
	if (ret) {
		dev_err(dev, "Failed initialize resets\n");
		goto err;
	}

	ret = nvadsp_app_module_probe(pdev);
	if (ret)
		goto err;

	aram_addr = drv_data->adsp_mem[ARAM_ALIAS_0_ADDR];
	aram_size = drv_data->adsp_mem[ARAM_ALIAS_0_SIZE];
	ret = nvadsp_aram_init(aram_addr, aram_size);
	if (ret)
		dev_err(dev, "Failed to init aram\n");

	drv_data->bwmgr = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_APE_ADSP);
	ret = IS_ERR_OR_NULL(drv_data->bwmgr);
	if (ret)
		dev_err(&pdev->dev, "unable to register bwmgr\n");
err:
#ifdef CONFIG_PM
	ret = pm_runtime_put_sync(dev);
	if (ret < 0)
		dev_err(dev, "pm_runtime_put_sync failed\n");
#endif
out:
	return ret;
}

static int nvadsp_remove(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int err;

	if (drv_data->bwmgr) {
		err = tegra_bwmgr_set_emc(drv_data->bwmgr, 0,
					  TEGRA_BWMGR_SET_EMC_FLOOR);
		if (err) {
			dev_err(&pdev->dev, "failed to set emc freq rate:%d\n",
				err);
		}
		tegra_bwmgr_unregister(drv_data->bwmgr);
	}
	nvadsp_aram_exit();

	pm_runtime_disable(&pdev->dev);

#ifdef CONFIG_PM
	if (!pm_runtime_status_suspended(&pdev->dev))
		nvadsp_runtime_suspend(&pdev->dev);
#endif

	tegra_pd_remove_device(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
static struct nvadsp_chipdata tegra210_adsp_chipdata = {
	.hwmb = {
		.reg_idx = AMISC,
		.hwmbox0_reg = 0x58,
		.hwmbox1_reg = 0X5C,
		.hwmbox2_reg = 0x60,
		.hwmbox3_reg = 0x64,
	},
	.adsp_state_hwmbox = -1,
	.adsp_thread_hwmbox = -1,
	.adsp_irq_hwmbox = -1,
	.reset_init = nvadsp_reset_t21x_init,
	.os_init = nvadsp_os_t21x_init,
#ifdef CONFIG_PM
	.pm_init = nvadsp_pm_t21x_init,
#endif
	.wdt_irq = INT_T210_ADSP_WDT,
	.start_irq = INT_T210_AGIC_START,
	.end_irq = INT_T210_AGIC_END,
};

static struct nvadsp_chipdata tegrat18x_adsp_chipdata = {
	.hwmb = {
		.reg_idx = AHSP,
		.hwmbox0_reg = 0x00000,
		.hwmbox1_reg = 0X08000,
		.hwmbox2_reg = 0X10000,
		.hwmbox3_reg = 0X18000,
		.hwmbox4_reg = 0X20000,
		.hwmbox5_reg = 0X28000,
		.hwmbox6_reg = 0X30000,
		.hwmbox7_reg = 0X38000,
	},
	.adsp_state_hwmbox = 0x30000,
	.adsp_thread_hwmbox = 0x20000,
	.adsp_irq_hwmbox = 0x38000,
	.reset_init = nvadsp_reset_t18x_init,
	.os_init = nvadsp_os_t18x_init,
#ifdef CONFIG_PM
	.pm_init = nvadsp_pm_t18x_init,
#endif
	.wdt_irq = INT_T18x_ATKE_WDT_IRQ,
	.start_irq = INT_T18x_AGIC_START,
	.end_irq = INT_T18x_AGIC_END,
};

static const struct of_device_id nvadsp_of_match[] = {
	{
		.compatible = "nvidia,tegra210-adsp",
		.data = &tegra210_adsp_chipdata,
	}, {
		.compatible = "nvidia,tegra18x-adsp",
		.data = &tegrat18x_adsp_chipdata,
	}, {
		.compatible = "nvidia,tegra18x-adsp-hv",
		.data = &tegrat18x_adsp_chipdata,
	}, {
	},
};
#endif

static struct platform_driver nvadsp_driver __refdata = {
	.driver	= {
		.name	= "nvadsp",
		.owner	= THIS_MODULE,
		.pm	= &nvadsp_pm_ops,
		.of_match_table = of_match_ptr(nvadsp_of_match),
	},
	.probe		= nvadsp_probe,
	.remove		= nvadsp_remove,
};

static int __init nvadsp_init(void)
{
	return platform_driver_register(&nvadsp_driver);
}

static void __exit nvadsp_exit(void)
{
	platform_driver_unregister(&nvadsp_driver);
}

module_init(nvadsp_init);
module_exit(nvadsp_exit);

MODULE_AUTHOR("NVIDIA");
MODULE_DESCRIPTION("Tegra Host ADSP Driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("Dual BSD/GPL");
