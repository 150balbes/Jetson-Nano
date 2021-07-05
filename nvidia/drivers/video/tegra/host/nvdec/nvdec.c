/*
 * Tegra NVDEC Module Support
 *
 * Copyright (c) 2013-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>         /* for kzalloc */
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/nvhost.h>
#include <linux/pm_runtime.h>
#include <linux/clk/tegra.h>
#include <asm/byteorder.h>      /* for parsing ucode image wrt endianness */
#include <linux/delay.h>	/* for udelay */
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <soc/tegra/chip-id.h>
#include <linux/version.h>

#include <linux/tegra_pm_domains.h>
#include <uapi/linux/nvhost_nvdec_ioctl.h>

#include <linux/platform/tegra/mc.h>

#include "dev.h"
#include "nvdec.h"
#include "nvhost_vm.h"
#include "hw_nvdec.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_scale.h"
#include "chip_support.h"
#include "t124/t124.h"
#include "t210/t210.h"
#include "iomap.h"

#if defined(CONFIG_TRUSTED_LITTLE_KERNEL) || defined(CONFIG_TRUSTY)
#include <linux/ote_protocol.h>
#endif

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "t186/t186.h"
#endif
#ifdef CONFIG_TEGRA_T19X_GRHOST
#include "t194/t194.h"
#endif

#define FW_NAME_SIZE			32

static inline struct flcn **get_nvdec(struct platform_device *dev)
{
	return (struct flcn **)nvhost_get_falcon_data(dev);
}
static inline void set_nvdec(struct platform_device *dev, struct flcn **flcn)
{
	nvhost_set_falcon_data(dev, flcn);
}

static int nvhost_nvdec_init_sw(struct platform_device *dev);
static unsigned int tegra_nvdec_enabled_in_bootcmd = -1;
static unsigned int tegra_nvdec_bootloader_enabled;

static int nvdec_get_bl_fw_name(struct platform_device *pdev, char *name)
{
	u8 maj, min;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	u32 debug_mode = host1x_readl(pdev, nvdec_scp_ctl_stat_r()) &
					nvdec_scp_ctl_stat_debug_mode_m();
	bool sim_mode = tegra_platform_is_qt() || tegra_platform_is_vdk();

	nvdec_decode_ver(pdata->version, &maj, &min);
	if (!tegra_nvdec_bootloader_enabled) {
		snprintf(name, FW_NAME_SIZE, "nvhost_nvdec0%d%d_ns.fw",
			maj, min);
		return 0;
	}

	if (sim_mode && debug_mode) {
		snprintf(name, FW_NAME_SIZE, "nvhost_nvdec_bl_no_wpr0%d%d.fw",
			maj, min);
	} else if (sim_mode && !debug_mode) {
		dev_info(&pdev->dev, "Prod + No-WPR not allowed\n");
		return -EINVAL;
	} else if (debug_mode) {
		snprintf(name, FW_NAME_SIZE, "nvhost_nvdec_bl0%d%d.fw",
			maj, min);
	} else {
		snprintf(name, FW_NAME_SIZE, "nvhost_nvdec_bl0%d%d_prod.fw",
			maj, min);
	}

	dev_info(&pdev->dev, "fw name:%s\n", name);
	return 0;
}

static void nvdec_get_fw_name(struct platform_device *pdev, char *name)
{
	u8 maj, min;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	u32 debug_mode = host1x_readl(pdev, nvdec_scp_ctl_stat_r()) &
					nvdec_scp_ctl_stat_debug_mode_m();

	nvdec_decode_ver(pdata->version, &maj, &min);
	if (debug_mode)
		snprintf(name, FW_NAME_SIZE, "nvhost_nvdec0%d%d.fw", maj, min);
	else
		snprintf(name, FW_NAME_SIZE, "nvhost_nvdec0%d%d_prod.fw", maj,
			min);

	dev_info(&pdev->dev, "fw name:%s\n", name);
}

static int nvhost_nvdec_bl_init(struct platform_device *dev)
{
	u32 fb_data_offset = 0;
	struct flcn **m = get_nvdec(dev);
	struct nvdec_bl_shared_data shared_data = {0};
	u32 debug = host1x_readl(dev,
				nvdec_scp_ctl_stat_r()) &
				nvdec_scp_ctl_stat_debug_mode_m();
	bool skip_wpr_settings = debug &&
		(tegra_platform_is_qt() || tegra_platform_is_vdk());

	/*
	 * debuginfo is cleared by the firmware on boot, write a dummy
	 * value here so that a successful boot can be detected.
	 */
	host1x_writel(dev, nvdec_debuginfo_r(), 0xDEADBEEF);
	fb_data_offset = (m[0]->os.bin_data_offset +
				m[0]->os.data_offset) / (sizeof(u32));
	shared_data.ls_fw_start_addr = m[1]->dma_addr >> 8;
	shared_data.ls_fw_size = m[1]->size;
	/* no wpr firmware does not need these */
	if (!skip_wpr_settings) {
		struct mc_carveout_info inf;
		int ret;

		ret = mc_get_carveout_info(&inf, NULL,
					   MC_SECURITY_CARVEOUT1);
		if (ret) {
			dev_err(&dev->dev, "carveout memory allocation failed");
			return -ENOMEM;
		}

		/* Put the 40-bit addr formed by wpr_addr_hi and
		   wpr_addr_lo divided by 256 into 32-bit wpr_addr */
		shared_data.wpr_addr = inf.base >> 8;
		shared_data.wpr_size = inf.size; /* Already in bytes. */
	}

	/* store fw start address for nvdec bl to read */
	memcpy(&(m[0]->mapped[fb_data_offset]), &shared_data,
		sizeof(shared_data));

	return 0;
}

int nvhost_nvdec_finalize_poweron(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int err = 0;
	struct flcn **m;

	dev_dbg(&dev->dev, "flcn_boot: start\n");
	err = nvhost_nvdec_init_sw(dev);
	if (err)
		return err;

	m = get_nvdec(dev);
	err = nvhost_flcn_wait_mem_scrubbing(dev);
	if (err)
		return err;

	/* load transcfg configuration if defined */
	if (pdata->transcfg_addr)
		host1x_writel(dev, pdata->transcfg_addr, pdata->transcfg_val);

	if (tegra_nvdec_bootloader_enabled) {
		err = nvhost_nvdec_bl_init(dev);
		if (err)
			return err;
	}

	err = nvhost_flcn_load_image(dev, m[0]->dma_addr, &m[0]->os, 0);
	if (err)
		return err;

	nvhost_flcn_irq_mask_set(dev);
	nvhost_flcn_irq_dest_set(dev);
	nvhost_flcn_ctxtsw_init(dev);
	nvhost_flcn_start(dev, 0);

	if (tegra_nvdec_bootloader_enabled) {
		u32 debuginfo = host1x_readl(dev, nvdec_debuginfo_r());

		/* Must be zero for successful boot */
		if (debuginfo) {
			dev_err(&dev->dev, "boot failed, debuginfo=%x",
					   debuginfo);
			return -ETIMEDOUT;
		}
	}

	dev_dbg(&dev->dev, "flcn_boot: success\n");

#if defined(CONFIG_TRUSTED_LITTLE_KERNEL)
	tlk_restore_keyslots();
#endif
#if defined(CONFIG_TRUSTY)
	trusty_restore_keyslots();
#endif

	return 0;
}

static int nvdec_read_ucode(struct platform_device *dev,
			    const char *fw_name,
			    struct flcn *m)
{
	const struct firmware *ucode_fw;
	struct ucode_v1_flcn ucode;
	int err;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_READ_ONLY, __DMA_ATTR(attrs));
	m->dma_addr = 0;
	m->mapped = NULL;
	ucode_fw  = nvhost_client_request_firmware(dev, fw_name);
	if (!ucode_fw) {
		dev_err(&dev->dev, "failed to get nvdec firmware %s\n",
				fw_name);
		err = -ENOENT;
		return err;
	}

	m->size = ucode_fw->size;
	m->mapped = dma_alloc_attrs(&dev->dev, m->size, &m->dma_addr,
				    GFP_KERNEL, __DMA_ATTR(attrs));
	if (!m->mapped) {
		dev_err(&dev->dev, "dma memory allocation failed");
		err = -ENOMEM;
		goto clean_up;
	}

	err = flcn_setup_ucode_image(dev, m, ucode_fw, &ucode);
	if (err) {
		dev_err(&dev->dev, "failed to parse firmware image %s\n",
				fw_name);
		goto clean_up;
	}

	m->valid = true;
	release_firmware(ucode_fw);

	return 0;

clean_up:
	if (m->mapped) {
		dma_free_attrs(&dev->dev, m->size, m->mapped, m->dma_addr,
			       __DMA_ATTR(attrs));
		m->mapped = NULL;
		m->dma_addr = 0;
	}
	release_firmware(ucode_fw);
	return err;
}


static int nvhost_nvdec_init_sw(struct platform_device *dev)
{
	int err = 0;
	struct flcn **m = get_nvdec(dev);
	char fw_name[2][FW_NAME_SIZE];
	int i;

	nvhost_dbg_fn("in dev:%p", dev);
	/* check if firmware resources already allocated */
	if (m)
		return 0;

	err = nvdec_get_bl_fw_name(dev, fw_name[0]);
	if (err)
		return -EINVAL;

	if (tegra_nvdec_bootloader_enabled)
		nvdec_get_fw_name(dev, fw_name[1]);

	m = kzalloc(2 * sizeof(struct flcn *), GFP_KERNEL);
	if (!m) {
		dev_err(&dev->dev, "couldn't allocate ucode ptr");
		return -ENOMEM;
	}

	set_nvdec(dev, m);
	nvhost_dbg_fn("primed dev:%p", dev);
	for (i = 0; i < (1 + tegra_nvdec_bootloader_enabled); i++) {
		m[i] = kzalloc(sizeof(struct flcn), GFP_KERNEL);
		if (!m[i]) {
			dev_err(&dev->dev, "couldn't alloc ucode");
			err = -ENOMEM;
			goto err_ucode;
		}

		err = nvdec_read_ucode(dev, fw_name[i], m[i]);
		if (err || !m[i]->valid) {
			dev_err(&dev->dev, "ucode not valid");
			goto err_ucode;
		}
	}

	return 0;

err_ucode:
	kfree(m[0]);
	kfree(m[1]);

	return err;
}

static struct of_device_id tegra_nvdec_of_match[] = {
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-nvdec",
		.data = (struct nvhost_device_data *)&t21_nvdec_info },
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-nvdec",
		.data = (struct nvhost_device_data *)&t18_nvdec_info },
#endif
#ifdef CONFIG_TEGRA_T19X_GRHOST
	{ .compatible = "nvidia,tegra194-nvdec",
		.data = (struct nvhost_device_data *)&t19_nvdec_info,
		.name = "nvdec" },
	{ .compatible = "nvidia,tegra194-nvdec",
		.data = (struct nvhost_device_data *)&t19_nvdec1_info,
		.name = "nvdec1" },
#endif
	{ },
};


static int nvdec_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata;
	struct nvdec_private *priv;

	pdata = container_of(inode->i_cdev,
		struct nvhost_device_data, ctrl_cdev);

	if (WARN_ONCE(pdata == NULL,
			"pdata not found, %s failed\n", __func__))
		return -ENODEV;

	if (WARN_ONCE(pdata->pdev == NULL,
			"device not found, %s failed\n", __func__))
		return -ENODEV;

	priv = kzalloc(sizeof(struct nvdec_private), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdata->pdev->dev,
			"couldn't allocate nvdec private");
		return -ENOMEM;
	}
	priv->pdev = pdata->pdev;
	atomic_set(&priv->refcnt, 0);

	file->private_data = priv;

	return 0;
}

static long nvdec_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct nvdec_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;
	int err;

	if (WARN_ONCE(pdev == NULL, "pdata not found, %s failed\n", __func__))
		return -ENODEV;

	if (_IOC_TYPE(cmd) != NVHOST_NVDEC_IOCTL_MAGIC)
		return -EFAULT;

	switch (cmd) {
	case NVHOST_NVDEC_IOCTL_POWERON:
		err = nvhost_module_busy(pdev);
		if (err)
			return err;

		atomic_inc(&priv->refcnt);
	break;
	case NVHOST_NVDEC_IOCTL_POWEROFF:
		if (atomic_dec_if_positive(&priv->refcnt) >= 0)
			nvhost_module_idle(pdev);
	break;
	default:
		dev_err(&pdev->dev,
			"%s: Unknown nvdec ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int nvdec_release(struct inode *inode, struct file *file)
{
	struct nvdec_private *priv = file->private_data;

	nvhost_module_idle_mult(priv->pdev, atomic_read(&priv->refcnt));
	kfree(priv);

	return 0;
}
static int nvdec_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvdec_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	pdata->pdev = dev;

	if (tegra_platform_is_sim() && tegra_get_chip_id() == TEGRA194) {
		dev_info(&dev->dev, "context isolation disabled on simulator");
		pdata->isolate_contexts = false;
	}

	mutex_init(&pdata->lock);

	platform_set_drvdata(dev, pdata);

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	dev->dev.platform_data = NULL;

	/* get the module clocks to sane state */
	nvhost_module_init(dev);

	err = nvhost_client_device_init(dev);

	return 0;
}

static int __exit nvdec_remove(struct platform_device *dev)
{
	nvhost_client_device_release(dev);
	return 0;
}

static struct platform_driver nvdec_driver = {
	.probe = nvdec_probe,
	.remove = __exit_p(nvdec_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvdec",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvdec_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
		.suppress_bind_attrs = true,
	}
};

const struct file_operations tegra_nvdec_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = nvdec_open,
	.unlocked_ioctl = nvdec_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvdec_ioctl,
#endif
	.release = nvdec_release,
};

static struct of_device_id tegra_nvdec_domain_match[] = {
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-nvdec-pd",
	.data = (struct nvhost_device_data *)&t21_nvdec_info},
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-nvdec-pd",
	.data = (struct nvhost_device_data *)&t18_nvdec_info},
#endif
#ifdef CONFIG_TEGRA_T19X_GRHOST
	{.compatible = "nvidia,tegra194-nvdec1-pd",
	 .data = (struct nvhost_device_data *)&t19_nvdec1_info},
#endif
	{},
};

static int __init tegra_nvdec_bootloader_enabled_arg(char *options)
{
	char *p = options;

	tegra_nvdec_enabled_in_bootcmd = memparse(p, &p);

	return 0;
}
early_param("nvdec_enabled", tegra_nvdec_bootloader_enabled_arg);

static int __init nvdec_init(void)
{
	int ret;

	if (tegra_nvdec_enabled_in_bootcmd != -1) {
		/* If "nvdec_enabled" exists in kernel boot command,
		tegra_nvdec_bootloader_enabled is updated with that value */
		tegra_nvdec_bootloader_enabled = tegra_nvdec_enabled_in_bootcmd;
	} else {
		/* Below kernel config check is for T210 */
		if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER))
			tegra_nvdec_bootloader_enabled = 1;
	}

	ret = nvhost_domain_init(tegra_nvdec_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&nvdec_driver);
}

static void __exit nvdec_exit(void)
{
	platform_driver_unregister(&nvdec_driver);
}

module_init(nvdec_init);
module_exit(nvdec_exit);
