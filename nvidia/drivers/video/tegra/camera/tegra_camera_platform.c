/*
 * drivers/video/tegra/camera/tegra_camera_platform.c
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/platform/tegra/bwmgr_mc.h>
#include <linux/platform/tegra/latency_allowance.h>
#include <linux/platform/tegra/isomgr.h>
#include <linux/list.h>

#include <media/vi.h>
#include <media/tegra_camera_dev_mfi.h>
#include <media/tegra_camera_platform.h>

#include "nvhost_acm.h"

#define CAMDEV_NAME "tegra_camera_ctrl"

/* Peak BPP for any of the YUV/Bayer formats */
#define CAMERA_PEAK_BPP 2

#define LANE_SPEED_1_GBPS 1000000000
#define LANE_SPEED_1_5_GBPS 1500000000

#if defined(CONFIG_TEGRA_BWMGR)
#include <linux/platform/tegra/emc_bwmgr.h>
#endif

struct tegra_camera_info {
	char devname[64];
	atomic_t in_use;
	struct device *dev;
#if defined(CONFIG_TEGRA_BWMGR)
	/* bandwidth manager handle */
	struct tegra_bwmgr_client *bwmgr_handle;
#endif
	struct clk *emc;
	struct clk *iso_emc;
#if defined(CONFIG_TEGRA_ISOMGR)
	tegra_isomgr_handle isomgr_handle;
	u64 max_bw;
#endif
	struct mutex update_bw_lock;
	u64 vi_mode_isobw;
	u64 bypass_mode_isobw;
	/* set max bw by default */
	bool en_max_bw;

	u64 phy_pixel_rate;
	u64 active_pixel_rate;
	u64 active_iso_bw;
	u32 max_pixel_depth;
	u32 ppc_divider;
	u32 num_active_streams;
	u32 num_device_lanes;
	u32 total_sensor_lanes;
	u32 sensor_type;
	u32 memory_latency;
	bool pg_mode;
	struct list_head device_list;
	struct mutex device_list_mutex;
};

static const struct of_device_id tegra_camera_of_ids[] = {
	{ .compatible = "nvidia, tegra-camera-platform" },
	{ },
};

static struct miscdevice tegra_camera_misc;

static int tegra_camera_isomgr_register(struct tegra_camera_info *info,
					struct device *dev)
{
#if defined(CONFIG_TEGRA_ISOMGR)
	int ret = 0;
	u32 num_csi_lanes = 0;
	u32 max_lane_speed = 0;
	u32 bits_per_pixel = 0;
	u32 vi_bpp = 0;
	u64 vi_iso_bw = 0;
	u32 vi_margin_pct = 0;
	u32 max_pixel_rate = 0;
	u32 isp_bpp = 0;
	u64 isp_iso_bw = 0;
	u32 isp_margin_pct = 0;
	u32 tpg_max_iso = 0;
	struct device_node *np = dev->of_node;

	dev_dbg(info->dev, "%s++\n", __func__);

	ret |= of_property_read_u32(np, "num_csi_lanes", &num_csi_lanes);
	ret |= of_property_read_u32(np, "max_lane_speed", &max_lane_speed);
	ret |= of_property_read_u32(np, "min_bits_per_pixel", &bits_per_pixel);
	ret |= of_property_read_u32(np, "vi_peak_byte_per_pixel", &vi_bpp);
	ret |= of_property_read_u32(np, "vi_bw_margin_pct", &vi_margin_pct);
	ret |= of_property_read_u32(np, "max_pixel_rate", &max_pixel_rate);
	ret |= of_property_read_u32(np, "isp_peak_byte_per_pixel", &isp_bpp);
	ret |= of_property_read_u32(np, "isp_bw_margin_pct", &isp_margin_pct);

	if (ret)
		dev_info(info->dev, "%s: some fields not in DT.\n", __func__);

	/*
	 * Use per-camera specifics to calculate ISO BW needed,
	 * which is smaller than the per-asic max.
	 *
	 * The formula for VI ISO BW is based on total number
	 * of active csi lanes when all cameras on the camera
	 * board are active.
	 *
	 * The formula for ISP ISO BW is based on max number
	 * of ISP's used in ISO mode given number of camera(s)
	 * on the camera board and the number of ISP's on the ASIC.
	 *
	 * The final ISO BW is based on the max of the two.
	 */
	if (!bits_per_pixel) {
		dev_err(info->dev, "bits_per_pixel is invalid\n");
		return -EINVAL;
	}
	vi_iso_bw = ((num_csi_lanes * max_lane_speed) / bits_per_pixel)
				* vi_bpp * (100 + vi_margin_pct) / 100;
	isp_iso_bw = max_pixel_rate * isp_bpp * (100 + isp_margin_pct) / 100;
	if (vi_iso_bw > isp_iso_bw)
		info->max_bw = vi_iso_bw;
	else
		info->max_bw = isp_iso_bw;

	if (!info->max_bw) {
		dev_err(info->dev, "%s: BW must be non-zero\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "tpg_max_iso", &tpg_max_iso);
	if (ret)
		tpg_max_iso = 0;
	else {
		dev_info(info->dev, "%s tpg_max_iso = %uKBs\n", __func__,
				tpg_max_iso);
		info->max_bw = max_t(u64, info->max_bw, tpg_max_iso);
	}

	dev_info(info->dev, "%s isp_iso_bw=%llu, vi_iso_bw=%llu, max_bw=%llu\n",
				__func__, isp_iso_bw, vi_iso_bw, info->max_bw);

	/* Register with max possible BW for CAMERA usecases.*/
	info->isomgr_handle = tegra_isomgr_register(
					TEGRA_ISO_CLIENT_TEGRA_CAMERA,
					info->max_bw,
					NULL,	/* tegra_isomgr_renegotiate */
					NULL);	/* *priv */

	if (IS_ERR(info->isomgr_handle)) {
		dev_err(info->dev,
			"%s: unable to register to isomgr\n",
				__func__);
		return -ENOMEM;
	}
#endif

	return 0;
}

static int tegra_camera_isomgr_unregister(struct tegra_camera_info *info)
{
#if defined(CONFIG_TEGRA_ISOMGR)
	tegra_isomgr_unregister(info->isomgr_handle);
	info->isomgr_handle = NULL;
#endif

	return 0;
}

static int tegra_camera_isomgr_request(
		struct tegra_camera_info *info, uint iso_bw, uint lt)
{
#if defined(CONFIG_TEGRA_ISOMGR)
	int ret = 0;

	dev_dbg(info->dev,
		"%s++ bw=%u, lt=%u\n", __func__, iso_bw, lt);

	if (!info->isomgr_handle) {
		dev_err(info->dev,
		"%s: isomgr_handle is NULL\n",
		__func__);
		return -EINVAL;
	}

	/* return value of tegra_isomgr_reserve is dvfs latency in usec */
	ret = tegra_isomgr_reserve(info->isomgr_handle,
				iso_bw,	/* KB/sec */
				lt);	/* usec */
	if (!ret) {
		dev_err(info->dev,
		"%s: failed to reserve %u KBps\n", __func__, iso_bw);
		return -ENOMEM;
	}

	/* return value of tegra_isomgr_realize is dvfs latency in usec */
	ret = tegra_isomgr_realize(info->isomgr_handle);
	if (ret)
		dev_dbg(info->dev,
		"%s: tegra_camera isomgr latency is %d usec",
		__func__, ret);
	else {
		dev_err(info->dev,
		"%s: failed to realize %u KBps\n", __func__, iso_bw);
			return -ENOMEM;
	}
#endif

	return 0;
}

int tegra_camera_emc_clk_enable(void)
{
#if defined(CONFIG_TEGRA_BWMGR)
	return 0;
#else
	struct tegra_camera_info *info;
	int ret = 0;

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -EINVAL;
	ret = clk_prepare_enable(info->emc);
	if (ret) {
		dev_err(info->dev, "Cannot enable camera.emc\n");
		return ret;
	}

	ret = clk_prepare_enable(info->iso_emc);
	if (ret) {
		dev_err(info->dev, "Cannot enable camera_iso.emc\n");
		goto err_iso_emc;
	}

	return 0;
err_iso_emc:
	clk_disable_unprepare(info->emc);
	return ret;
#endif
}
EXPORT_SYMBOL(tegra_camera_emc_clk_enable);

int tegra_camera_emc_clk_disable(void)
{
#if defined(CONFIG_TEGRA_BWMGR)
	return 0;
#else
	struct tegra_camera_info *info;

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -EINVAL;
	clk_disable_unprepare(info->emc);
	clk_disable_unprepare(info->iso_emc);
	return 0;
#endif
}
EXPORT_SYMBOL(tegra_camera_emc_clk_disable);

static int tegra_camera_open(struct inode *inode, struct file *file)
{
	struct tegra_camera_info *info;
	struct miscdevice *mdev;

	mdev = file->private_data;
	info = dev_get_drvdata(mdev->parent);
	file->private_data = info;
#if defined(CONFIG_TEGRA_BWMGR)
	/* get bandwidth manager handle if needed */
	info->bwmgr_handle =
		tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_CAMERA_NON_ISO);

	/* set the initial rate */
	if (IS_ERR_OR_NULL(info->bwmgr_handle)) {
		info->bwmgr_handle = NULL;
		return -ENODEV;
	}
	tegra_bwmgr_set_emc(info->bwmgr_handle, 0,
			TEGRA_BWMGR_SET_EMC_SHARED_BW);
	return 0;
#else
	return tegra_camera_emc_clk_enable();
#endif
}

static int tegra_camera_release(struct inode *inode, struct file *file)
{

	struct tegra_camera_info *info;

	info = file->private_data;
#if defined(CONFIG_TEGRA_BWMGR)
	tegra_bwmgr_unregister(info->bwmgr_handle);
#else
	tegra_camera_emc_clk_disable();
#endif
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static u64 vi_mode_d;
static u64 bypass_mode_d;

static int dbgfs_tegra_camera_init(void)
{
	struct dentry *dir;
	struct dentry *val;

	dir = debugfs_create_dir("tegra_camera_platform", NULL);
	if (!dir)
		return -ENOMEM;

	val = debugfs_create_u64("vi", S_IRUGO, dir, &vi_mode_d);
	if (!val)
		return -ENOMEM;
	val = debugfs_create_u64("scf", S_IRUGO, dir, &bypass_mode_d);
	if (!val)
		return -ENOMEM;
	return 0;
}
#endif

/*
 * submits total aggregated iso bw request to isomgr.
 */
int tegra_camera_update_isobw(void)
{
	struct tegra_camera_info *info;
	unsigned long total_khz;
	unsigned long bw;
	int ret = 0;

	if (tegra_camera_misc.parent == NULL) {
		pr_info("driver not enabled, cannot update bw\n");
		return -ENODEV;
	}

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -ENODEV;
	mutex_lock(&info->update_bw_lock);

	bw = info->active_iso_bw;
	if (info->bypass_mode_isobw > info->active_iso_bw)
		bw = info->bypass_mode_isobw;

	/* Bug 200323801 consider iso bw of both vi mode and vi-bypass mode */
	if (bw >= info->max_bw) {
		dev_info(info->dev,
			"%s: Warning, Requested ISO BW %lu has been capped to VI's max BW %llu\n",
			__func__, bw, info->max_bw);
		bw = info->max_bw;
	}

	if (info->pg_mode)
		bw = info->max_bw;
	if (info->num_active_streams == 0)
		bw = 0;

#ifdef CONFIG_TEGRA_MC
	/*
	 * Different chip versions use different APIs to set LA for VI.
	 * If one fails, try another, and fail if both of them don't work.
	 */
	ret = tegra_set_camera_ptsa(TEGRA_LA_VI_W, bw, 1);
	if (ret) {
		ret = tegra_set_latency_allowance(TEGRA_LA_VI_W, bw);
		if (ret) {
			dev_err(info->dev, "%s: set la failed: %d\n",
				__func__, ret);
			return ret;
		}
	}
#endif

	/* Use Khz to prevent overflow */
	total_khz = bwmgr_bw_to_freq(bw);
	total_khz = min(ULONG_MAX / 1000, total_khz);

	dev_dbg(info->dev, "%s:Set iso bw %lu kbyteps at %lu KHz\n",
		__func__, bw, total_khz);
#if !defined(CONFIG_TEGRA_BWMGR)
	ret = clk_set_rate(info->iso_emc, total_khz * 1000);
	if (ret)
		dev_err(info->dev, "%s:Failed to set iso bw\n",
			__func__);
#endif
	/*
	 * Request to ISOMGR.
	 */
	ret = tegra_camera_isomgr_request(info, bw, info->memory_latency);
	if (ret) {
		dev_err(info->dev,
		"%s: failed to reserve %lu KBps with isomgr\n",
		__func__, bw);
		mutex_unlock(&info->update_bw_lock);
		return -ENOMEM;
	}

	info->vi_mode_isobw = bw;
#ifdef CONFIG_DEBUG_FS
	vi_mode_d = bw;
	bypass_mode_d = info->bypass_mode_isobw;
#endif
	mutex_unlock(&info->update_bw_lock);
	return ret;
}
EXPORT_SYMBOL(tegra_camera_update_isobw);

static long tegra_camera_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct tegra_camera_info *info;

	info = file->private_data;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(TEGRA_CAMERA_IOCTL_SET_BW):
	{
		struct bw_info kcopy;
		unsigned long mc_khz = 0;

		memset(&kcopy, 0, sizeof(kcopy));

		if (copy_from_user(&kcopy, (const void __user *)arg,
			sizeof(struct bw_info))) {
			dev_err(info->dev, "%s:Failed to get data from user\n",
				__func__);
			return -EFAULT;
		}
#if defined(CONFIG_TEGRA_ISOMGR)
		if (kcopy.bw > info->max_bw && kcopy.is_iso) {
			dev_info(info->dev,
				"ISO BW req %llu > %lld (max) capping to max",
				kcopy.bw,
				info->max_bw);
			kcopy.bw = info->max_bw;
		}
#endif
		/* Use Khz to prevent overflow */
		mc_khz = bwmgr_bw_to_freq(kcopy.bw);
		mc_khz = min(ULONG_MAX / 1000, mc_khz);

		if (kcopy.is_iso) {
			info->bypass_mode_isobw = kcopy.bw;
			ret = tegra_camera_update_isobw();
		} else {
			dev_dbg(info->dev, "%s:Set bw %llu at %lu KHz\n",
				__func__, kcopy.bw, mc_khz);
#if defined(CONFIG_TEGRA_BWMGR)
			ret = tegra_bwmgr_set_emc(info->bwmgr_handle,
				mc_khz*1000, TEGRA_BWMGR_SET_EMC_SHARED_BW);
#else
			ret = clk_set_rate(info->emc, mc_khz * 1000);
#endif
		}
		break;
	}

	case _IOC_NR(TEGRA_CAMERA_IOCTL_GET_BW):
	{
		unsigned long mc_hz = 0;
		u64 bw = 0;

#if defined(CONFIG_TEGRA_BWMGR)
		ret = tegra_bwmgr_get_client_info(info->bwmgr_handle, &mc_hz,
			TEGRA_BWMGR_SET_EMC_SHARED_BW);
		if (ret)
			return ret;
#else
		mc_hz = clk_get_rate(info->emc);
#endif
		bw = bwmgr_freq_to_bw(mc_hz / 1000);

		if (copy_to_user((void __user *)arg, (const void *)&bw,
			sizeof(bw))) {
			dev_err(info->dev,
				"%s:Failed to copy data to user\n",
				__func__);
			return -EFAULT;
		}

		break;
	}

	case _IOC_NR(TEGRA_CAMERA_IOCTL_GET_CURR_REQ_ISO_BW):
	{
		struct tegra_camera_info *state;
		u64 bw;

		state = dev_get_drvdata(tegra_camera_misc.parent);
		if (!state)
			return -ENODEV;

		mutex_lock(&state->update_bw_lock);
		bw = state->vi_mode_isobw;
		mutex_unlock(&state->update_bw_lock);

		if (copy_to_user((void __user *)arg, (const void *)&bw,
			sizeof(bw))) {
			dev_err(info->dev,
				"%s:Failed to copy data to user\n",
				__func__);
			return -EFAULT;
		}

		break;
	}

	default:
		break;
	}
	return ret;
}

static const struct file_operations tegra_camera_ops = {
	.owner = THIS_MODULE,
	.open = tegra_camera_open,
	.unlocked_ioctl = tegra_camera_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tegra_camera_ioctl,
#endif
	.release = tegra_camera_release,
};



static int tegra_camera_probe(struct platform_device *pdev)
{
	int ret;
	struct tegra_camera_info *info;

	dev_info(&pdev->dev, "%s:camera_platform_driver probe\n", __func__);

	tegra_camera_misc.minor = MISC_DYNAMIC_MINOR;
	tegra_camera_misc.name = CAMDEV_NAME;
	tegra_camera_misc.fops = &tegra_camera_ops;
	tegra_camera_misc.parent = &pdev->dev;

	ret = misc_register(&tegra_camera_misc);
	if (ret) {
		dev_err(tegra_camera_misc.this_device,
			"register failed for %s\n",
			tegra_camera_misc.name);
		return ret;
	}
	info = devm_kzalloc(tegra_camera_misc.this_device,
		sizeof(struct tegra_camera_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	strcpy(info->devname, tegra_camera_misc.name);
	info->dev = tegra_camera_misc.this_device;

#if !defined(CONFIG_TEGRA_BWMGR)
	info->emc = devm_clk_get(info->dev, "emc");
	if (IS_ERR(info->emc)) {
		dev_err(info->dev, "Failed to get camera.emc\n");
		return -EINVAL;
	}
	clk_set_rate(info->emc, 0);
	info->iso_emc = devm_clk_get(info->dev, "iso.emc");
	if (IS_ERR(info->iso_emc)) {
		dev_err(info->dev, "Failed to get camera_iso.emc\n");
		return -EINVAL;
	}
	clk_set_rate(info->iso_emc, 0);
#endif
	mutex_init(&info->update_bw_lock);
	/* Register Camera as isomgr client. */
	ret = tegra_camera_isomgr_register(info, &pdev->dev);
	if (ret) {
		dev_err(info->dev,
		"%s: failed to register CAMERA as isomgr client\n",
		__func__);
		return -ENOMEM;
	}

	info->en_max_bw = of_property_read_bool(pdev->dev.of_node,
		"default-max-bw");
	if (info->en_max_bw == true) {
#if defined(CONFIG_TEGRA_ISOMGR)
		ret = tegra_camera_isomgr_request(info, info->max_bw,
				info->memory_latency);
		if (ret) {
			dev_err(info->dev,
			"%s: failed to request max bw\n", __func__);
			tegra_camera_isomgr_unregister(info);
			return -EFAULT;
		}
#endif
	}

	info->phy_pixel_rate = 0;
	info->active_pixel_rate = 0;
	info->active_iso_bw = 0;
	info->max_pixel_depth = 0;
	info->ppc_divider = 1;
	info->num_active_streams = 0;
	info->num_device_lanes = 0;
	info->total_sensor_lanes = 0;
	info->sensor_type = 0;
	info->memory_latency = 0;
	info->pg_mode = false;
	mutex_init(&info->device_list_mutex);
	INIT_LIST_HEAD(&info->device_list);

	tegra_vi_register_mfi_cb(tegra_camera_dev_mfi_cb, NULL);

	platform_set_drvdata(pdev, info);
#ifdef CONFIG_DEBUG_FS
	ret = dbgfs_tegra_camera_init();
	if (ret)
		dev_err(info->dev, "Fail to create debugfs");
#endif
	return 0;
}

static void update_platform_data(struct tegra_camera_dev_info *cdev,
	struct tegra_camera_info *info, bool dev_registered)
{
	/* TPG: handled differently based on
	 * throughput calculations.
	 */
	if (cdev->sensor_type == SENSORTYPE_VIRTUAL)
		info->pg_mode = dev_registered;

	if (cdev->sensor_type != SENSORTYPE_NONE)
		info->sensor_type = cdev->sensor_type;

	if (cdev->hw_type != HWTYPE_NONE) {
		if (info->num_device_lanes < cdev->lane_num)
			info->num_device_lanes = cdev->lane_num;
	}

	if (dev_registered) {
		/* if bytes per pixel is greater than 2, then num_ppc
		 * is 4 for VI. Set divider for this case.
		 */
		if (cdev->bpp > 2)
			info->ppc_divider = 2;
		if (cdev->sensor_type != SENSORTYPE_NONE)
			info->total_sensor_lanes += cdev->lane_num;
		if (info->total_sensor_lanes <= info->num_device_lanes)
			info->phy_pixel_rate += cdev->pixel_rate;
		if (info->max_pixel_depth < cdev->pixel_bit_depth)
			info->max_pixel_depth = cdev->pixel_bit_depth;
		if (info->memory_latency < cdev->memory_latency)
			info->memory_latency = cdev->memory_latency;
	} else {
		if (info->total_sensor_lanes <= info->num_device_lanes)
			info->phy_pixel_rate -= cdev->pixel_rate;
		info->total_sensor_lanes -= cdev->lane_num;
	}
}

static int add_nvhost_client(struct tegra_camera_dev_info *cdev)
{
	int ret = 0;

	if (cdev->hw_type == HWTYPE_NONE)
		return 0;

	ret = nvhost_module_add_client(cdev->pdev, &cdev->hw_type);
	return ret;
}

static int remove_nvhost_client(struct tegra_camera_dev_info *cdev)
{
	if (cdev->hw_type == HWTYPE_NONE)
		return 0;

	nvhost_module_remove_client(cdev->pdev, &cdev->hw_type);
	return 0;
}

int tegra_camera_device_register(struct tegra_camera_dev_info *cdev_info,
					void *priv)
{
	int err = 0;
	struct tegra_camera_dev_info *cdev;
	struct tegra_camera_info *info;

	/*
	 * If tegra_camera_platform is not enabled, devices
	 * cannot be registered, but the HW engines will still be probed.
	 * So just return without registering.
	 */
	if (tegra_camera_misc.parent == NULL) {
		pr_info("driver not enabled, cannot register any devices\n");
		return 0;
	}

	if (!cdev_info || !priv)
		return -EINVAL;

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -EINVAL;

	cdev = kzalloc(sizeof(struct tegra_camera_dev_info), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	*cdev = *cdev_info;

	INIT_LIST_HEAD(&cdev->device_node);
	cdev->priv = priv;

	mutex_lock(&info->device_list_mutex);
	list_add(&cdev->device_node, &info->device_list);
	err = add_nvhost_client(cdev);
	if (err) {
		mutex_unlock(&info->device_list_mutex);
		dev_err(info->dev, "%s could not add %d to nvhost\n",
				__func__, cdev->hw_type);
		return err;
	}
	update_platform_data(cdev, info, true);
	mutex_unlock(&info->device_list_mutex);

	return err;
}
EXPORT_SYMBOL(tegra_camera_device_register);

int tegra_camera_device_unregister(void *priv)
{
	struct tegra_camera_dev_info *cdev, *tmp;
	int found = 0;
	struct tegra_camera_info *info;

	/*
	 * If tegra_camera_platform is not enabled, devices
	 * were not registered, so return here.
	 */
	if (tegra_camera_misc.parent == NULL) {
		pr_info("driver not enabled, no devices were registered\n");
		return 0;
	}

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -EINVAL;

	mutex_lock(&info->device_list_mutex);
	list_for_each_entry_safe(cdev, tmp, &info->device_list, device_node) {
		if (priv == cdev->priv) {
			list_del(&cdev->device_node);
			found = 1;
			break;
		}
	}

	if (found) {
		remove_nvhost_client(cdev);
		update_platform_data(cdev, info, false);
		kfree(cdev);
	}
	mutex_unlock(&info->device_list_mutex);

	return 0;
}
EXPORT_SYMBOL(tegra_camera_device_unregister);

int tegra_camera_get_device_list_entry(const u32 hw_type, const void *priv,
		struct tegra_camera_dev_info *cdev_info)
{
	struct tegra_camera_dev_info *cdev;
	struct tegra_camera_info *info;
	int found = 0;
	int ret = 0;

	if (tegra_camera_misc.parent == NULL)
		return -EINVAL;

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -EINVAL;

	mutex_lock(&info->device_list_mutex);
	list_for_each_entry(cdev, &info->device_list, device_node) {
		if (hw_type == cdev->hw_type) {
			/*
			 * If priv is NULL yet the hw_type is set to that of a
			 * sensor the first sensor in the device list will be
			 * returned. Otherwise, a NULL priv and a matching
			 * hw_type will return the hw unit (VI, CSI etc.).
			 *
			 * Otherwise, a non NULL priv is used as an additional
			 * constraint to return specific devices who may or
			 * may not share common hw_types.
			 */
			if (!priv) {
				found = 1;
				break;
			} else if (priv == cdev->priv) {
				found = 1;
				break;
			}
		}
	}

	if (found && (cdev_info != NULL))
		*cdev_info = *cdev;

	mutex_unlock(&info->device_list_mutex);

	if (!found)
		return -ENOENT;

	return ret;
}
EXPORT_SYMBOL(tegra_camera_get_device_list_entry);

int tegra_camera_get_device_list_stats(u32 *n_sensors, u32 *n_hwtypes)
{
	struct tegra_camera_dev_info *cdev;
	struct tegra_camera_info *info;
	int ret = 0;

	if (!n_sensors || !n_hwtypes)
		return -EINVAL;

	if (tegra_camera_misc.parent == NULL)
		return -EINVAL;

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -EINVAL;

	*n_sensors = 0;
	*n_hwtypes = 0;

	mutex_lock(&info->device_list_mutex);
	list_for_each_entry(cdev, &info->device_list, device_node) {
		if (cdev->hw_type == HWTYPE_NONE)
			(*n_sensors)++;
		else
			(*n_hwtypes)++;
	}
	mutex_unlock(&info->device_list_mutex);

	return ret;
}
EXPORT_SYMBOL(tegra_camera_get_device_list_stats);

static int calculate_and_set_device_clock(struct tegra_camera_info *info,
		struct tegra_camera_dev_info *cdev)
{
	int ret = 0;
	u64 active_pr = info->active_pixel_rate;
	u64 phy_pr = info->phy_pixel_rate;
	u32 overhead = cdev->overhead + 100;
	u32 max_depth = info->max_pixel_depth;
	u32 bus_width = cdev->bus_width;
	u32 lane_num = cdev->lane_num;
	u64 lane_speed = cdev->lane_speed;
	u32 ppc = (cdev->ppc) ? cdev->ppc : 1;
	u32 ppc_divider = (ppc > 1) ? info->ppc_divider : 1;
	u64 nr = 0;
	u64 dr = 0;
	u64 clk_rate = 0;
	u64 final_pr = (cdev->use_max) ? phy_pr : active_pr;
	bool set_clk = true;

	if (cdev->hw_type == HWTYPE_NONE)
		return 0;

	switch (cdev->hw_type) {
	case HWTYPE_CSI:
		if (info->sensor_type == SENSORTYPE_SLVSEC)
			set_clk = false;
		nr = max_depth * final_pr * overhead;
		dr = bus_width * 100;
		if (dr == 0)
			return -EINVAL;
		break;
	case HWTYPE_VI:
		nr = final_pr * overhead;
		dr = 100 * (ppc / ppc_divider);
		break;
	case HWTYPE_ISPA:
	case HWTYPE_ISPB:
		nr = final_pr * overhead;
		dr = 100 * ppc;
		break;
	case HWTYPE_SLVSEC:
		if (info->sensor_type != SENSORTYPE_SLVSEC)
			set_clk = false;
		nr = lane_speed * lane_num * overhead;
		dr = bus_width * 100;
		if (dr == 0)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	/* avoid rounding errors by adding dr to nr */
	clk_rate = (nr + dr) / dr;

	/* Use special rates based on throughput
	 * for TPG.
	 */
	if (info->pg_mode) {
		clk_rate =  (cdev->pg_clk_rate) ?
			cdev->pg_clk_rate : DEFAULT_PG_CLK_RATE;
	}

	/* no stream active, set to 0 */
	if (info->num_active_streams == 0)
		clk_rate = 0;

	if (clk_rate != cdev->clk_rate)
		cdev->clk_rate = clk_rate;
	else
		set_clk = false;

	if (set_clk) {
		ret = nvhost_module_set_rate(cdev->pdev, &cdev->hw_type,
				cdev->clk_rate, 0, NVHOST_CLOCK);
		if (ret)
			return ret;

		/* save the actual rate set by nvhost */
		ret = nvhost_module_get_rate(cdev->pdev,
				&cdev->actual_clk_rate, 0);
	}

	return ret;
}

int tegra_camera_update_clknbw(void *priv, bool stream_on)
{
	struct tegra_camera_dev_info *cdev;
	struct tegra_camera_info *info;
	int ret = 0;

	info = dev_get_drvdata(tegra_camera_misc.parent);
	if (!info)
		return -EINVAL;

	mutex_lock(&info->device_list_mutex);
	/* Need to traverse the list twice, first to make sure that
	 * stream on is set for the active stream and then to
	 * update clocks and BW.
	 * Needed as devices could have been added in any order in the list.
	 */
	list_for_each_entry(cdev, &info->device_list, device_node) {
		if (priv == cdev->priv) {
			/* set stream on */
			cdev->stream_on = stream_on;
			if (stream_on) {
				info->active_pixel_rate += cdev->pixel_rate;
				info->active_iso_bw += cdev->bw;
				info->num_active_streams++;
			} else {
				info->active_pixel_rate -= cdev->pixel_rate;
				info->active_iso_bw -= cdev->bw;
				info->num_active_streams--;
			}
			break;
		}
	}

	/* update clocks */
	list_for_each_entry(cdev, &info->device_list, device_node) {
		ret = calculate_and_set_device_clock(info, cdev);
		if (ret) {
			mutex_unlock(&info->device_list_mutex);
			return -EINVAL;
		}
	}
	mutex_unlock(&info->device_list_mutex);

	/* set BW */
	tegra_camera_update_isobw();

	return ret;
}
EXPORT_SYMBOL(tegra_camera_update_clknbw);

static int tegra_camera_remove(struct platform_device *pdev)
{
	struct tegra_camera_info *info = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s:camera_platform_driver remove\n", __func__);

	/* deallocate isomgr bw */
	if (info->en_max_bw)
		tegra_camera_isomgr_request(info, 0, info->memory_latency);

	tegra_vi_unregister_mfi_cb();
	tegra_camera_isomgr_unregister(info);
	misc_deregister(&tegra_camera_misc);

	return 0;
}

static struct platform_driver tegra_camera_driver = {
	.probe = tegra_camera_probe,
	.remove = tegra_camera_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra_camera_platform",
		.of_match_table = tegra_camera_of_ids
	}
};
static int __init tegra_camera_init(void)
{
	return platform_driver_register(&tegra_camera_driver);
}
static void __exit tegra_camera_exit(void)
{
	platform_driver_unregister(&tegra_camera_driver);
}

module_init(tegra_camera_init);
module_exit(tegra_camera_exit);

