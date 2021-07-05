/*
 * NVCSI driver for T186
 *
 * Copyright (c) 2014-2019, NVIDIA Corporation.  All rights reserved.
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
#include <linux/device.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/fs.h>
#include <asm/ioctls.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <uapi/linux/nvhost_nvcsi_ioctl.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <media/mc_common.h>
#include <media/csi.h>
#include <media/tegra_camera_platform.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t186/t186.h"
#include "nvcsi.h"
#include "camera/csi/csi4_fops.h"

#include "deskew.h"

#define PG_CLK_RATE	102000000
/* width of interface between VI and CSI */
#define CSI_BUS_WIDTH	64
/* number of lanes per brick */
#define NUM_LANES	4

#define CSIA			(1 << 20)
#define CSIF			(1 << 25)

struct nvcsi {
	struct platform_device *pdev;
	struct regulator *regulator;
	struct tegra_csi_device csi;
	struct dentry *dir;
};

static long long input_stats;

static struct tegra_csi_device *mc_csi;
static struct tegra_csi_data t18_nvcsi_data = {
	.info = (struct nvhost_device_data *)&t18_nvcsi_info,
	.csi_fops = &csi4_fops,
};

static const struct of_device_id tegra_nvcsi_of_match[] = {
	{
		.compatible = "nvidia,tegra186-nvcsi",
		.data = &t18_nvcsi_data
	},
	{ },
};

struct nvcsi_private {
	struct platform_device *pdev;
	struct nvcsi_deskew_context deskew_ctx;
};

static int nvcsi_deskew_debugfs_init(struct nvcsi *nvcsi);
static void nvcsi_deskew_debugfs_remove(struct nvcsi *nvcsi);

int nvcsi_finalize_poweron(struct platform_device *pdev)
{
	struct nvcsi *nvcsi = nvhost_get_private_data(pdev);
	int ret;

	if (nvcsi->regulator) {
		ret = regulator_enable(nvcsi->regulator);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable csi regulator failed.");
			return ret;
		}
	}
	return tegra_csi_mipi_calibrate(&nvcsi->csi, true);
}

int nvcsi_prepare_poweroff(struct platform_device *pdev)
{
	struct nvcsi *nvcsi = nvhost_get_private_data(pdev);
	int ret;

	ret = tegra_csi_mipi_calibrate(&nvcsi->csi, false);
	if (ret)
		dev_err(&pdev->dev, "disable mipi calibraiton failed\n");

	if (nvcsi->regulator) {
		ret = regulator_disable(nvcsi->regulator);
		if (ret)
			dev_err(&pdev->dev, "failed to disabled csi regulator failed.");
	}

	return 0;
}

static int nvcsi_probe_regulator(struct nvcsi *nvcsi)
{
	struct device *dev = &nvcsi->pdev->dev;
	struct regulator *regulator;
	const char *regulator_name;
	int err;

	err = of_property_read_string(dev->of_node, "nvidia,csi_regulator",
				      &regulator_name);
	if (err)
		return err;

	regulator = devm_regulator_get(dev, regulator_name);
	if (IS_ERR(regulator))
		return PTR_ERR(regulator);

	nvcsi->regulator = regulator;

	return 0;
}


int nvcsi_cil_sw_reset(int lanes, int enable)
{
	unsigned int phy_num = 0U;
	unsigned int val = enable ? (SW_RESET1_EN | SW_RESET0_EN) : 0U;
	unsigned int addr, i;

	for (i = CSIA; i < CSIF; i = i << 2U) {
		if (lanes & i) {
			addr = CSI4_BASE_ADDRESS + NVCSI_CIL_A_SW_RESET +
						(CSI4_PHY_OFFSET * phy_num);
			host1x_writel(mc_csi->pdev, addr, val);
		}
		if (lanes & (i << 1U)) {
			addr = CSI4_BASE_ADDRESS + NVCSI_CIL_B_SW_RESET +
						(CSI4_PHY_OFFSET * phy_num);
			host1x_writel(mc_csi->pdev, addr, val);
		}
		phy_num++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(nvcsi_cil_sw_reset);

static int nvcsi_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;
	struct nvcsi *nvcsi = NULL;
	struct tegra_csi_data *data = NULL;
	struct tegra_camera_dev_info csi_info;

	memset(&csi_info, 0, sizeof(csi_info));

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvcsi_of_match, &dev->dev);
		if (match) {
			data = (struct tegra_csi_data *) match->data;
			pdata = (struct nvhost_device_data *)data->info;
		}
	} else {
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;
	}

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		err = -ENODATA;
		goto err_get_pdata;
	}

	nvcsi = devm_kzalloc(&dev->dev, sizeof(*nvcsi), GFP_KERNEL);
	if (!nvcsi) {
		err = -ENOMEM;
		goto err_alloc_nvcsi;
	}

	nvcsi->pdev = dev;
	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);
	pdata->private_data = nvcsi;
	mc_csi = &nvcsi->csi;


	err = nvcsi_probe_regulator(nvcsi);
	if (err)
		dev_info(&dev->dev, "failed to get regulator (%d)\n", err);

	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto err_get_resources;

	err = nvhost_module_init(dev);
	if (err)
		goto err_module_init;

	err = nvhost_client_device_init(dev);
	if (err)
		goto err_client_device_init;

	if (data)
		nvcsi->csi.fops = data->csi_fops;

	err = tegra_csi_media_controller_init(&nvcsi->csi, dev);
	nvcsi_deskew_platform_setup(&nvcsi->csi, false);

	if (err < 0)
		goto err_mediacontroller_init;

	nvcsi_deskew_debugfs_init(nvcsi);

	csi_info.pdev = dev;
	csi_info.hw_type = HWTYPE_CSI;
	csi_info.use_max = true;
	csi_info.bus_width = CSI_BUS_WIDTH;
	csi_info.lane_num = NUM_LANES;
	csi_info.pg_clk_rate = PG_CLK_RATE;
	err = tegra_camera_device_register(&csi_info, nvcsi);
	if (err)
		goto err_module_init;

	return 0;

err_mediacontroller_init:
err_client_device_init:
	nvhost_module_deinit(dev);
err_module_init:
err_get_resources:
err_alloc_nvcsi:
err_get_pdata:

	return err;
}

static int __exit nvcsi_remove(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct nvcsi *nvcsi = (struct nvcsi *)pdata->private_data;

	tegra_camera_device_unregister(nvcsi);
	mc_csi = NULL;
	nvcsi_deskew_debugfs_remove(nvcsi);
	tegra_csi_media_controller_remove(&nvcsi->csi);

	return 0;
}

static struct platform_driver nvcsi_driver = {
	.probe = nvcsi_probe,
	.remove = __exit_p(nvcsi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvcsi",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvcsi_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

#ifdef CONFIG_PM_GENERIC_DOMAINS
static struct of_device_id tegra_nvcsi_domain_match[] = {
	{.compatible = "nvidia,tegra186-ve-pd",
	.data = (struct nvhost_device_data *)&t18_nvcsi_info},
	{},
};
#endif
static int dbgfs_deskew_stats(struct seq_file *s, void *data)
{
	deskew_dbgfs_deskew_stats(s);
	return 0;
}

static int dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbgfs_deskew_stats, inode->i_private);
}

static const struct file_operations dbg_show_ops = {
	.open		= dbgfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static int dbgfs_calc_bound(struct seq_file *s, void *data)
{
	deskew_dbgfs_calc_bound(s, input_stats);
	return 0;
}
static int dbg_calc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbgfs_calc_bound, inode->i_private);
}
static const struct file_operations dbg_calc_ops = {
	.open		= dbg_calc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static void nvcsi_deskew_debugfs_remove(struct nvcsi *nvcsi)
{
	debugfs_remove_recursive(nvcsi->dir);
}
static int nvcsi_deskew_debugfs_init(struct nvcsi *nvcsi)
{
	struct dentry *val;

	nvcsi->dir = debugfs_create_dir("deskew", NULL);
	if (!nvcsi->dir)
		return -ENOMEM;

	val = debugfs_create_file("stats", S_IRUGO, nvcsi->dir, mc_csi,
				&dbg_show_ops);
	if (!val)
		goto err_debugfs;
	val = debugfs_create_x64("input_status", S_IRUGO | S_IWUSR,
				nvcsi->dir, &input_stats);
	if (!val)
		goto err_debugfs;
	val = debugfs_create_file("calc_bound", S_IRUGO | S_IWUSR,
				nvcsi->dir, mc_csi, &dbg_calc_ops);
	if (!val)
		goto err_debugfs;
	return 0;
err_debugfs:
	dev_err(mc_csi->dev, "Fail to create debugfs\n");
	debugfs_remove_recursive(nvcsi->dir);
	return -ENOMEM;
}
static long nvcsi_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct nvcsi_private *priv = file->private_data;
	int ret;

	switch (cmd) {
	// sensor must be turned on before calling this ioctl, and streaming
	// should be started shortly after.
	case NVHOST_NVCSI_IOCTL_DESKEW_SETUP: {
		unsigned int active_lanes;

		dev_dbg(mc_csi->dev, "ioctl: deskew_setup\n");
		priv->deskew_ctx.deskew_lanes = get_user(active_lanes,
				(long __user *)arg);
		ret = nvcsi_deskew_setup(&priv->deskew_ctx);
		return ret;
		}
	case NVHOST_NVCSI_IOCTL_DESKEW_APPLY: {
		dev_dbg(mc_csi->dev, "ioctl: deskew_apply\n");
		ret = nvcsi_deskew_apply_check(&priv->deskew_ctx);
		return ret;
		}
	}
	return -ENOIOCTLCMD;
}

static int nvcsi_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct nvcsi_private *priv;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL))
		return -ENOMEM;

	priv->pdev = pdev;

	file->private_data = priv;
	return nonseekable_open(inode, file);
}

static int nvcsi_release(struct inode *inode, struct file *file)
{
	struct nvcsi_private *priv = file->private_data;

	kfree(priv);
	return 0;
}

const struct file_operations tegra_nvcsi_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvcsi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvcsi_ioctl,
#endif
	.open = nvcsi_open,
	.release = nvcsi_release,
};

static int __init nvcsi_init(void)
{

#ifdef CONFIG_PM_GENERIC_DOMAINS
	int ret;

	ret = nvhost_domain_init(tegra_nvcsi_domain_match);
	if (ret)
		return ret;
#endif

	return platform_driver_register(&nvcsi_driver);
}

static void __exit nvcsi_exit(void)
{
	platform_driver_unregister(&nvcsi_driver);
}

late_initcall(nvcsi_init);
module_exit(nvcsi_exit);
