/*
 * NVCSI driver for T194
 *
 * Copyright (c) 2017-2019, NVIDIA Corporation.  All rights reserved.
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

#include "nvcsi-t194.h"
#include <uapi/linux/nvhost_nvcsi_ioctl.h>
#include <linux/tegra-camera-rtcpu.h>

#include <asm/ioctls.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/tegra_prod.h>
#include <linux/kthread.h>

#include <media/mc_common.h>
#include <media/tegra_camera_platform.h>
#include "camera/nvcsi/csi5_fops.h"

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t194/t194.h"
#include "media/csi.h"

#include "deskew.h"

#include "vhost/vhost.h"

/* PG rate based on max ISP throughput */
#define PG_CLK_RATE	102000000
/* width of interface between VI and CSI */
#define CSI_BUS_WIDTH	64
/* number of lanes per brick */
#define NUM_LANES	4

#define PHY_OFFSET			0x10000U
#define CIL_A_SW_RESET			0x11024U
#define CIL_B_SW_RESET			0x110b0U
#define CSIA				(1 << 20)
#define CSIH				(1 << 27)

static long long input_stats;

static struct tegra_csi_device *mc_csi;
struct t194_nvcsi {
	struct platform_device *pdev;
	struct tegra_csi_device csi;
	struct dentry *dir;
	void __iomem *io;
	struct tegra_prod *prod_list;
	atomic_t on;
};

static const struct of_device_id tegra194_nvcsi_of_match[] = {
	{
		.compatible = "nvidia,tegra194-nvcsi",
		.data = &t19_nvcsi_info,
	},
	{ },
};

struct t194_nvcsi_file_private {
	struct platform_device *pdev;
	struct nvcsi_deskew_context deskew_ctx;
};

static int nvcsi_deskew_debugfs_init(struct t194_nvcsi *nvcsi);
static void nvcsi_deskew_debugfs_remove(struct t194_nvcsi *nvcsi);

static int nvhost_nvcsi_prod_apply_virt_WAR(struct t194_nvcsi *nvcsi,
					    unsigned int phy_mode)
{
	dev_info(&nvcsi->pdev->dev,
		 "%s: nvcsi prod setting virt WAR active\n",
		 __func__);
	return vhost_prod_apply(nvcsi->pdev, phy_mode);
}

static int nvhost_nvcsi_cil_sw_reset_virt_WAR(struct platform_device *pdev,
					      unsigned int lanes,
					      unsigned int enable)
{
	dev_info(&pdev->dev,
		 "%s: nvcsi cil sw reset virt WAR active\n",
		 __func__);
	return vhost_cil_sw_reset(pdev, lanes, enable);
}

static long t194_nvcsi_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct t194_nvcsi_file_private *filepriv = file->private_data;
	int ret;

	switch (cmd) {
	// sensor must be turned on before calling this ioctl, and streaming
	// should be started shortly after.
	case NVHOST_NVCSI_IOCTL_DESKEW_SETUP: {
		unsigned int active_lanes;

		dev_dbg(mc_csi->dev, "ioctl: deskew_setup\n");
		ret = copy_from_user(&active_lanes, (const void __user *)arg,
							sizeof(unsigned int));
		if (ret) {
			return -EINVAL;
		} else {
			filepriv->deskew_ctx.deskew_lanes = active_lanes;
			return nvcsi_deskew_setup(&filepriv->deskew_ctx);
		}
	}
	case NVHOST_NVCSI_IOCTL_DESKEW_APPLY: {
		dev_dbg(mc_csi->dev, "ioctl: deskew_apply\n");
		ret = nvcsi_deskew_apply_check(&filepriv->deskew_ctx);
		return ret;
	}
	case NVHOST_NVCSI_IOCTL_PROD_APPLY: {
		unsigned int phy_mode;
		int err;
		struct t194_nvcsi *nvcsi = nvhost_get_private_data(
							filepriv->pdev);

		dev_dbg(mc_csi->dev, "ioctl: prod_apply\n");
		ret = copy_from_user(&phy_mode, (const void __user *)arg,
							sizeof(unsigned int));
		if (ret)
			return -EINVAL;

		if (!nvcsi->io)
			return nvhost_nvcsi_prod_apply_virt_WAR(nvcsi,
								phy_mode);

		err = tegra_prod_set_by_name(&nvcsi->io, "prod",
							nvcsi->prod_list);
		if (err) {
			dev_err(&nvcsi->pdev->dev,
			"%s: prod set fail (err=%d)\n", __func__, err);
			return err;
		}
		if (phy_mode == PHY_CPHY_MODE)
			err = tegra_prod_set_by_name(&nvcsi->io,
							"prod_c_cphy_mode",
							nvcsi->prod_list);
		else
			err = tegra_prod_set_by_name(&nvcsi->io,
							"prod_c_dphy_mode",
							nvcsi->prod_list);
		if (err)
			dev_err(&nvcsi->pdev->dev,
				"%s: phy_prod set fail (err=%d)\n", __func__,
				err);
		return err;
	}

	}

	return -ENOIOCTLCMD;
}

static int t194_nvcsi_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct t194_nvcsi_file_private *filepriv;

	filepriv = kzalloc(sizeof(*filepriv), GFP_KERNEL);
	if (unlikely(filepriv == NULL))
		return -ENOMEM;

	filepriv->pdev = pdev;

	file->private_data = filepriv;

	return nonseekable_open(inode, file);
}

static int t194_nvcsi_release(struct inode *inode, struct file *file)
{
	struct t194_nvcsi_file_private *filepriv = file->private_data;

	kfree(filepriv);

	return 0;
}

const struct file_operations tegra194_nvcsi_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = t194_nvcsi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = t194_nvcsi_ioctl,
#endif
	.open = t194_nvcsi_open,
	.release = t194_nvcsi_release,
};

static void nvcsi_apply_prod(struct platform_device *pdev)
{
	int err = -ENODEV;

	struct t194_nvcsi *nvcsi = nvhost_get_private_data(pdev);
	struct tegra_csi_channel *chan;
	u32 phy_mode;
	bool is_cphy;

	if (!list_empty(&nvcsi->csi.csi_chans)) {

		chan = list_first_entry(&nvcsi->csi.csi_chans,
				struct tegra_csi_channel, list);
		phy_mode = read_phy_mode_from_dt(chan);
		is_cphy = (phy_mode == CSI_PHY_MODE_CPHY);

		err = tegra_prod_set_by_name(&nvcsi->io, "prod",
							nvcsi->prod_list);

		if (err) {
			dev_err(&pdev->dev,
				"%s: prod set fail (err=%d)\n", __func__, err);
		}
		if (is_cphy)
			err = tegra_prod_set_by_name(&nvcsi->io,
							"prod_c_cphy_mode",
							nvcsi->prod_list);
		else
			err = tegra_prod_set_by_name(&nvcsi->io,
							"prod_c_dphy_mode",
							nvcsi->prod_list);
		if (err) {
			dev_err(&pdev->dev,
				"%s: prod set fail (err=%d)\n", __func__, err);
		}
	}

}

static int nvcsi_prod_apply_thread(void *data)
{
	struct platform_device *pdev = data;
	struct t194_nvcsi *nvcsi = nvhost_get_private_data(pdev);

	/*
	 * rtcpu finishes poweron after ~120ms after nvcsi_finalize_poweron
	 * so sleep for around that much before checking rtcpu
	 * power state again
	 */
	while (!tegra_camrtc_is_rtcpu_powered())
		usleep_range(1000*125, 1000*126);

	nvcsi_apply_prod(pdev);
	tegra_csi_mipi_calibrate(&nvcsi->csi, true);
	atomic_set(&nvcsi->on, 1);
	return 0;

}


int tegra194_nvcsi_finalize_poweron(struct platform_device *pdev)
{
	struct t194_nvcsi *nvcsi = nvhost_get_private_data(pdev);

	if (atomic_read(&nvcsi->on) == 1)
		return 0;

	/* rtcpu resets nvcsi registers, so we set prod settings
	 * after rtcpu has finished resetting the registers
	 * which happens during rtcpu's poweron call
	 * TODO: fix this dependency
	 */
	if (!tegra_camrtc_is_rtcpu_powered())
		kthread_run(nvcsi_prod_apply_thread, pdev, "nvcsi-t194-prod");
	else {
		nvcsi_apply_prod(pdev);
		tegra_csi_mipi_calibrate(&nvcsi->csi, true);
		atomic_set(&nvcsi->on, 1);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra194_nvcsi_finalize_poweron);

int tegra194_nvcsi_prepare_poweroff(struct platform_device *pdev)
{
	struct t194_nvcsi *nvcsi = nvhost_get_private_data(pdev);
	int err = 0;

	if (atomic_read(&nvcsi->on) == 0)
		return 0;

	err = tegra_csi_mipi_calibrate(&nvcsi->csi, false);
	if (err) {
		dev_err(&pdev->dev, "calibration failed\n");
		return err;
	}
	atomic_set(&nvcsi->on, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra194_nvcsi_prepare_poweroff);

int tegra194_nvcsi_cil_sw_reset(int lanes, int enable)
{
	unsigned int phy_num = 0U;
	unsigned int val = enable ? (SW_RESET1_EN | SW_RESET0_EN) : 0U;
	unsigned int addr, i;
	struct t194_nvcsi *nvcsi = nvhost_get_private_data(mc_csi->pdev);

	if (!nvcsi->io)
		return nvhost_nvcsi_cil_sw_reset_virt_WAR(mc_csi->pdev,
							  lanes, enable);

	for (i = CSIA; i < CSIH; i = i << 2U) {
		if (lanes & i) {
			addr = CIL_A_SW_RESET + (PHY_OFFSET * phy_num);
			host1x_writel(mc_csi->pdev, addr, val);
		}
		if (lanes & (i << 1U)) {
			addr = CIL_B_SW_RESET + (PHY_OFFSET * phy_num);
			host1x_writel(mc_csi->pdev, addr, val);
		}
		phy_num++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra194_nvcsi_cil_sw_reset);

int t194_nvcsi_early_probe(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata;
	struct t194_nvcsi *nvcsi;

	pdata = (void *)of_device_get_match_data(&pdev->dev);
	if (unlikely(pdata == NULL)) {
		dev_WARN(&pdev->dev, "no platform data\n");
		return -ENODATA;
	}

	nvcsi = devm_kzalloc(&pdev->dev, sizeof(*nvcsi), GFP_KERNEL);
	if (!nvcsi) {
		return -ENOMEM;
	}

	pdata->pdev = pdev;
	nvcsi->pdev = pdev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(pdev, pdata);
	mc_csi = &nvcsi->csi;

	pdata->private_data = nvcsi;

	return 0;
}

int t194_nvcsi_late_probe(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct t194_nvcsi *nvcsi = pdata->private_data;
	struct tegra_camera_dev_info csi_info;
	int err;

	nvcsi->prod_list = devm_tegra_prod_get(&pdev->dev);

	if (IS_ERR(nvcsi->prod_list)) {
		pr_err("%s: Can not find nvcsi prod node\n", __func__);
		return -ENODEV;
	}

	memset(&csi_info, 0, sizeof(csi_info));
	csi_info.pdev = pdev;
	csi_info.hw_type = HWTYPE_CSI;
	csi_info.use_max = true;
	csi_info.bus_width = CSI_BUS_WIDTH;
	csi_info.lane_num = NUM_LANES;
	csi_info.pg_clk_rate = PG_CLK_RATE;
	err = tegra_camera_device_register(&csi_info, nvcsi);
	if (err)
		return err;

	nvcsi->pdev = pdev;
	nvcsi->csi.fops = &csi5_fops;
	atomic_set(&nvcsi->on, 0);
	err = tegra_csi_media_controller_init(&nvcsi->csi, pdev);

	nvcsi_deskew_platform_setup(&nvcsi->csi, true);

	if (err < 0)
		return err;

	nvcsi_deskew_debugfs_init(nvcsi);

	return 0;
}

static int t194_nvcsi_probe(struct platform_device *pdev)
{
	struct resource *mem;
	void __iomem *regs;
	int err;
	struct nvhost_device_data *pdata;
	struct t194_nvcsi *nvcsi;

	err = t194_nvcsi_early_probe(pdev);
	if (err)
		return err;

	pdata = platform_get_drvdata(pdev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		return -EINVAL;
	}
	regs = devm_ioremap_nocache(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	nvcsi = pdata->private_data;
	nvcsi->io = regs;

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		return err;

	err = nvhost_module_init(pdev);
	if (err)
		return err;

	err = nvhost_client_device_init(pdev);
	if (err) {
		nvhost_module_deinit(pdev);
		goto err_client_device_init;
	}

	err = t194_nvcsi_late_probe(pdev);
	if (err)
		goto err_mediacontroller_init;

	return 0;

err_mediacontroller_init:
	nvhost_client_device_release(pdev);
err_client_device_init:
	pdata->private_data = NULL;
	return err;
}

static int __exit t194_nvcsi_remove(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct t194_nvcsi *nvcsi = pdata->private_data;

	tegra_camera_device_unregister(nvcsi);
	mc_csi = NULL;
	nvcsi_deskew_debugfs_remove(nvcsi);
	tegra_csi_media_controller_remove(&nvcsi->csi);

	return 0;
}

static struct platform_driver t194_nvcsi_driver = {
	.probe = t194_nvcsi_probe,
	.remove = __exit_p(t194_nvcsi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "t194-nvcsi",
#ifdef CONFIG_OF
		.of_match_table = tegra194_nvcsi_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

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

static void nvcsi_deskew_debugfs_remove(struct t194_nvcsi *nvcsi)
{
	debugfs_remove_recursive(nvcsi->dir);
}

static int nvcsi_deskew_debugfs_init(struct t194_nvcsi *nvcsi)
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

module_platform_driver(t194_nvcsi_driver);
