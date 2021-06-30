/*
 * Copyright (c) 2013-2019, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/nvhost.h>
#include <uapi/linux/nvhost_vi_ioctl.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/clk.h>

#include <linux/platform/tegra/latency_allowance.h>
#include <media/vi.h>

#include "bus_client.h"
#include "nvhost_acm.h"
#include "chip_support.h"
#include "host1x/host1x.h"
#include "vi/vi_irq.h"
#include "camera/csi/csi2_fops.h"

#define T12_VI_CFG_CG_CTRL	0xb8
#define T12_CG_2ND_LEVEL_EN	1
#define T12_VI_CSI_0_SW_RESET	0x100
#define T12_CSI_CSI_SW_SENSOR_A_RESET 0x858
#define T12_CSI_CSICIL_SW_SENSOR_A_RESET 0x94c
#define T12_VI_CSI_0_CSI_IMAGE_DT 0x120

#define T12_VI_CSI_1_SW_RESET	0x200
#define T12_CSI_CSI_SW_SENSOR_B_RESET 0x88c
#define T12_CSI_CSICIL_SW_SENSOR_B_RESET 0x980
#define T12_VI_CSI_1_CSI_IMAGE_DT 0x220

#define T21_CSI_CILA_PAD_CONFIG0 0x92c
#define T21_CSI1_CILA_PAD_CONFIG0 0x112c
#define T21_CSI2_CILA_PAD_CONFIG0 0x192c

#define VI_MAX_BPP 2

int nvhost_vi_finalize_poweron(struct platform_device *dev)
{
	struct vi *tegra_vi = nvhost_get_private_data(dev);
	int ret = 0;

	if (!tegra_vi)
		return -EINVAL;

	if (tegra_vi->reg) {
		ret = regulator_enable(tegra_vi->reg);
		if (ret) {
			dev_err(&dev->dev,
					"%s: enable csi regulator failed.\n",
					__func__);
			return ret;
		}
	}

#ifdef CONFIG_ARCH_TEGRA_12x_SOC
	/* Only do this for vi.0 not for slave device vi.1 */
	if (dev->id == 0)
		host1x_writel(dev, T12_VI_CFG_CG_CTRL, T12_CG_2ND_LEVEL_EN);
#endif

	ret = vi_enable_irq(tegra_vi);
	if (ret) {
		dev_err(&tegra_vi->ndev->dev, "%s: vi_enable_irq failed\n",
			__func__);
		return ret;
	}

	return tegra_csi_mipi_calibrate(&tegra_vi->csi, true);
}

int nvhost_vi_prepare_poweroff(struct platform_device *dev)
{
	int ret = 0;
	struct vi *tegra_vi = nvhost_get_private_data(dev);

	if (!tegra_vi)
		return -EINVAL;

	ret = tegra_csi_mipi_calibrate(&tegra_vi->csi, false);
	if (ret) {
		dev_err(&tegra_vi->ndev->dev, "%s:disable calibration failed\n",
			__func__);
		return ret;
	}

	ret = vi_disable_irq(tegra_vi);
	if (ret) {
		dev_err(&tegra_vi->ndev->dev, "%s: vi_disable_irq failed\n",
			__func__);
		return ret;
	}

	if (tegra_vi->reg) {
		ret = regulator_disable(tegra_vi->reg);
		if (ret)
			dev_err(&dev->dev,
				"%s: disable csi regulator failed.\n",
				__func__);
	}

	return ret;
}

static long vi_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct vi *tegra_vi = file->private_data;

	if (_IOC_TYPE(cmd) != NVHOST_VI_IOCTL_MAGIC)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(NVHOST_VI_IOCTL_ENABLE_TPG): {
		uint enable;
		int ret = 0;

		if (copy_from_user(&enable,
			(const void __user *)arg, sizeof(uint))) {
			dev_err(&tegra_vi->ndev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}

		if (enable)
			csi_source_from_plld();
		else
			csi_source_from_brick();

		return ret;
	}
	default:
		dev_err(&tegra_vi->ndev->dev,
			"%s: Unknown vi ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int vi_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata;
	struct vi *vi;

	pdata = container_of(inode->i_cdev,
		struct nvhost_device_data, ctrl_cdev);
	if (WARN_ONCE(pdata == NULL, "pdata not found, %s failed\n", __func__))
		return -ENODEV;

	vi = (struct vi *)pdata->private_data;
	if (WARN_ONCE(vi == NULL, "vi not found, %s failed\n", __func__))
		return -ENODEV;

	file->private_data = vi;

	return 0;
}

static int vi_release(struct inode *inode, struct file *file)
{
	return 0;
}

const struct file_operations tegra_vi_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = vi_open,
	.unlocked_ioctl = vi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vi_ioctl,
#endif
	.release = vi_release,
};

/* Reset sensor data if respective clk is ON */
void nvhost_vi_reset_all(struct platform_device *pdev)
{
	void __iomem *reset_reg[4];
	int err;
	bool enabled = false;
	struct nvhost_device_data *pdata = pdev->dev.platform_data;
	struct clk *clk;

	err = nvhost_clk_get(pdev, "cilab", &clk);
	if (!err) {
		reset_reg[0] = pdata->aperture[0] +
			T12_VI_CSI_0_SW_RESET;
		reset_reg[1] = pdata->aperture[0] +
			T12_CSI_CSI_SW_SENSOR_A_RESET;
		reset_reg[2] = pdata->aperture[0] +
			T12_CSI_CSICIL_SW_SENSOR_A_RESET;
		reset_reg[3] = pdata->aperture[0] +
			T12_VI_CSI_0_CSI_IMAGE_DT;

		writel(0, reset_reg[3]);
		writel(0x1, reset_reg[2]);
		writel(0x1, reset_reg[1]);
		writel(0x1f, reset_reg[0]);

		udelay(10);

		writel(0, reset_reg[2]);
		writel(0, reset_reg[1]);
	}

	err = nvhost_clk_get(pdev, "cilcd", &clk);
	if (!err)
		enabled = true;

	err = nvhost_clk_get(pdev, "cile", &clk);
	if (!err)
		enabled = true;

	if (enabled) {
		reset_reg[0] = pdata->aperture[0] +
			T12_VI_CSI_1_SW_RESET;
		reset_reg[1] = pdata->aperture[0] +
			T12_CSI_CSI_SW_SENSOR_B_RESET;
		reset_reg[2] = pdata->aperture[0] +
			T12_CSI_CSICIL_SW_SENSOR_B_RESET;
		reset_reg[3] = pdata->aperture[0] +
			T12_VI_CSI_1_CSI_IMAGE_DT;

		writel(0, reset_reg[3]);
		writel(0x1, reset_reg[2]);
		writel(0x1, reset_reg[1]);
		writel(0x1f, reset_reg[0]);

		udelay(10);

		writel(0, reset_reg[2]);
		writel(0, reset_reg[1]);
	}
}
