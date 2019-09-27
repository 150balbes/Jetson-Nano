/*
 * dc_sysfs.c: dc driver sysfs interface.
 *
 * Copyright (c) 2011-2019, NVIDIA CORPORATION, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/atomic.h>

#include "dc.h"
#include "dc_reg.h"
#include "dc_priv.h"
#include "vrr.h"

static ssize_t mode_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_mode *m;
	ssize_t res;

	mutex_lock(&dc->lock);
	m = &dc->mode;
	res = snprintf(buf, PAGE_SIZE,
		"pclk: %d\n"
		"h_ref_to_sync: %d\n"
		"v_ref_to_sync: %d\n"
		"h_sync_width: %d\n"
		"v_sync_width: %d\n"
		"h_back_porch: %d\n"
		"v_back_porch: %d\n"
		"h_active: %d\n"
		"v_active: %d\n"
		"h_front_porch: %d\n"
		"v_front_porch: %d\n"
		"stereo_mode: %d\n",
		m->pclk, m->h_ref_to_sync, m->v_ref_to_sync,
		m->h_sync_width, m->v_sync_width,
		m->h_back_porch, m->v_back_porch,
		m->h_active, m->v_active,
		m->h_front_porch, m->v_front_porch,
		m->stereo_mode);
	mutex_unlock(&dc->lock);

	return res;
}

static DEVICE_ATTR(mode, 0444, mode_show, NULL);

static ssize_t stats_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	bool enabled;

	if (mutex_lock_killable(&dc->lock))
		return -EINTR;
	enabled = tegra_dc_stats_get(dc);
	mutex_unlock(&dc->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t stats_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (mutex_lock_killable(&dc->lock))
		return -EINTR;
	tegra_dc_stats_enable(dc, !!val);
	mutex_unlock(&dc->lock);

	return count;
}

static DEVICE_ATTR(stats_enable, 0644,
	stats_enable_show, stats_enable_store);

static ssize_t enable_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	ssize_t res;

	mutex_lock(&dc->lock);
	res = snprintf(buf, PAGE_SIZE, "%d\n", dc->enabled);
	mutex_unlock(&dc->lock);
	return res;
}

static ssize_t enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val)
		tegra_dc_enable(dc);
	else
		tegra_dc_disable(dc);

	return count;
}

static DEVICE_ATTR(enable, 0600, enable_show, enable_store);

static ssize_t crc_checksum_latched_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	u32 crc;

	if (atomic_read(&dc->crc_ref_cnt.global))
		return -EBUSY;

	if (!dc->enabled) {
		dev_err(&dc->ndev->dev, "%s: DC not enabled.\n", __func__);
		return -EFAULT;
	}

	if (tegra_dc_is_nvdisplay())
		crc = tegra_nvdisp_sysfs_read_rg_crc(dc);
	else
		crc = tegra_dc_sysfs_read_checksum_latched(dc);

	return snprintf(buf, PAGE_SIZE, "%u\n", crc);
}

static ssize_t crc_checksum_latched_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	unsigned long val = 0;

	if (!dc->enabled) {
		dev_err(&dc->ndev->dev, "%s: DC not enabled.\n", __func__);
		return -EFAULT;
	}

	if (atomic_read(&dc->crc_ref_cnt.global))
		return -EBUSY;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val == 1) {
		if (tegra_dc_is_nvdisplay())
			tegra_nvdisp_sysfs_enable_crc(dc);
		else
			tegra_dc_sysfs_enable_crc(dc);

		dev_dbg(&dc->ndev->dev, "crc is enabled.\n");
	} else if (val == 0) {
		if (tegra_dc_is_nvdisplay())
			tegra_nvdisp_sysfs_disable_crc(dc);
		else
			tegra_dc_sysfs_disable_crc(dc);

		dev_dbg(&dc->ndev->dev, "crc is disabled.\n");
	} else
		dev_err(&dc->ndev->dev, "Invalid input.\n");

	return count;
}
static DEVICE_ATTR(crc_checksum_latched, 0644,
		crc_checksum_latched_show, crc_checksum_latched_store);

static ssize_t out_crc_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	u32 crc = 0;

	if (!dc->enabled) {
		dev_err(&dc->ndev->dev, "%s: DC not enabled.\n", __func__);
		return -EFAULT;
	}

	if (dc->out_ops && dc->out_ops->get_crc)
		crc = dc->out_ops->get_crc(dc);

	return snprintf(buf, PAGE_SIZE, "%u\n", crc);
}

static ssize_t out_crc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	unsigned long val = 0;

	if (!dc->enabled) {
		dev_err(&dc->ndev->dev, "%s: DC not enabled.\n", __func__);
		return -EFAULT;
	}

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val > 1) {
		dev_err(&dc->ndev->dev, "Invalid input.\n");
		return count;
	}

	if (dc->out_ops && dc->out_ops->toggle_crc)
			dc->out_ops->toggle_crc(dc, val);

	if (val == 1)
		dev_dbg(&dc->ndev->dev, "crc is enabled.\n");
	else if (val == 0)
		dev_dbg(&dc->ndev->dev, "crc is disabled.\n");

	return count;
}
static DEVICE_ATTR(out_crc, 0644, out_crc_show, out_crc_store);

static ssize_t scanline_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	u32 val;
	unsigned v_blank;
	unsigned v_count;
	unsigned h_blank;
	unsigned h_count;

	if (WARN_ON(!dc) || !dc->enabled ||
		WARN_ON(!tegra_dc_is_powered(dc))) {
		dev_err(&dc->ndev->dev, "%s: DC not enabled.\n", __func__);
		return -ENXIO;
	}

	val = tegra_dc_readl(dc, DC_DISP_DISPLAY_DBG_TIMING);

	v_count = (val & DBG_V_COUNT_MASK) >> DBG_V_COUNT_SHIFT;
	v_blank = !!(val & DBG_V_BLANK);
	h_count = (val & DBG_H_COUNT_MASK) >> DBG_H_COUNT_SHIFT;
	h_blank = !!(val & DBG_H_BLANK);

	return scnprintf(buf, PAGE_SIZE, "%u %u %u %u\n",
		v_count, v_blank, h_count, h_blank);
}
static DEVICE_ATTR(scanline, 0444, scanline_show, NULL);

#define ORIENTATION_PORTRAIT	"portrait"
#define ORIENTATION_LANDSCAPE	"landscape"

static ssize_t orientation_3d_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_out *dc_out = dc->out;
	const char *orientation;

	switch (dc_out->stereo->orientation) {
	case TEGRA_DC_STEREO_LANDSCAPE:
		orientation = ORIENTATION_LANDSCAPE;
		break;
	case TEGRA_DC_STEREO_PORTRAIT:
		orientation = ORIENTATION_PORTRAIT;
		break;
	default:
		pr_err("Invalid value is stored for stereo_orientation.\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", orientation);
}

static ssize_t orientation_3d_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_out *dc_out = dc->out;
	struct tegra_stereo_out *stereo = dc_out->stereo;
	int orientation;

	if (0 == strncmp(buf, ORIENTATION_PORTRAIT,
			min(cnt, ARRAY_SIZE(ORIENTATION_PORTRAIT) - 1))) {
		orientation = TEGRA_DC_STEREO_PORTRAIT;
	} else if (0 == strncmp(buf, ORIENTATION_LANDSCAPE,
			min(cnt, ARRAY_SIZE(ORIENTATION_LANDSCAPE) - 1))) {
		orientation = TEGRA_DC_STEREO_LANDSCAPE;
	} else {
		pr_err("Invalid property value for stereo_orientation.\n");
		return -EINVAL;
	}
	stereo->orientation = orientation;
	stereo->set_orientation(orientation);
	return cnt;
}

static DEVICE_ATTR(stereo_orientation,
	0644, orientation_3d_show, orientation_3d_store);

#define MODE_2D		"2d"
#define MODE_3D		"3d"

static ssize_t mode_3d_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_out *dc_out = dc->out;
	const char *mode;

	switch (dc_out->stereo->mode_2d_3d) {
	case TEGRA_DC_STEREO_MODE_2D:
		mode = MODE_2D;
		break;
	case TEGRA_DC_STEREO_MODE_3D:
		mode = MODE_3D;
		break;
	default:
		pr_err("Invalid value is stored for stereo_mode.\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", mode);
}

static ssize_t mode_3d_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_out *dc_out = dc->out;
	struct tegra_stereo_out *stereo = dc_out->stereo;
	int mode;

	if (0 == strncmp(buf, MODE_2D, min(cnt, ARRAY_SIZE(MODE_2D) - 1))) {
		mode = TEGRA_DC_STEREO_MODE_2D;
	} else if (0 == strncmp(buf, MODE_3D,
			min(cnt, ARRAY_SIZE(MODE_3D) - 1))) {
		mode = TEGRA_DC_STEREO_MODE_3D;
	} else {
		pr_err("Invalid property value for stereo_mode.\n");
		return -EINVAL;
	}
	stereo->mode_2d_3d = mode;
	stereo->set_mode(mode);
	return cnt;
}

static DEVICE_ATTR(stereo_mode,
	0644, mode_3d_show, mode_3d_store);

static ssize_t cmu_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	int e;
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	e = kstrtoint(buf, 10, &val);
	if (e)
		return e;

	tegra_dc_cmu_enable(dc, val);

	return count;
}

static ssize_t cmu_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dc->pdata->cmu_enable);
}

static DEVICE_ATTR(cmu_enable,
		0644, cmu_enable_show, cmu_enable_store);

#ifdef CONFIG_TEGRA_ISOMGR
static ssize_t reserved_bw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	u32 reserved_bw = 0;

	if (tegra_dc_is_nvdisplay()) {
		if (dc->ihub_bw_info)
			reserved_bw = dc->ihub_bw_info->reserved_bw;
	} else {
		reserved_bw = dc->reserved_bw;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", reserved_bw);
}

static DEVICE_ATTR(reserved_bw,
		0444, reserved_bw_show, NULL);
#endif

static ssize_t smart_panel_show(struct device *device,
	struct device_attribute *attr, char  *buf)
{
	return snprintf(buf, PAGE_SIZE, "1\n");
}

static DEVICE_ATTR(smart_panel, 0444, smart_panel_show, NULL);

static ssize_t panel_rotate_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dc->out->rotation);
}

static DEVICE_ATTR(panel_rotation, 0444, panel_rotate_show, NULL);

/* display current window assignment bitmask in
 * hexadecimal for the given dc device->dev */
static ssize_t win_mask_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	ssize_t res;

	if ((!dc) || (!dc->ndev) || (!dc->pdata)) {
		pr_err("%s: dc|device err\n", __func__);
		res = -EINVAL;
		goto exit;
	}
	mutex_lock(&dc->lock);
	res = snprintf(buf, PAGE_SIZE, "0x%lx\n", dc->pdata->win_mask);
	mutex_unlock(&dc->lock);
exit:
	return res;
}

static ssize_t win_mask_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	unsigned long requested_winmask = 0;
	size_t ret;

	if (!tegra_dc_is_nvdisplay())
		return -EINVAL;

	/* try first as decimal, then as hexadecimal */
	if ((kstrtoul(buf, 10, &requested_winmask) < 0) &&
		(kstrtoul(buf, 0, &requested_winmask) < 0))
			return -EINVAL;

	ret = tegra_dc_update_winmask(dc, requested_winmask);
	if (!ret)
		return count;
	return ret;
}

static DEVICE_ATTR(win_mask, 0600, win_mask_show, win_mask_store);


static ssize_t panel_connected_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dc->connected);
}

static DEVICE_ATTR(panel_connected, 0444, panel_connected_show, NULL);

void tegra_dc_remove_sysfs(struct device *dev)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	device_remove_file(dev, &dev_attr_mode);
	device_remove_file(dev, &dev_attr_enable);
	device_remove_file(dev, &dev_attr_stats_enable);
	device_remove_file(dev, &dev_attr_crc_checksum_latched);
	if ((dc->out->type == TEGRA_DC_OUT_HDMI) ||
		(dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP))
		device_remove_file(dev, &dev_attr_out_crc);
	device_remove_file(dev, &dev_attr_win_mask);

#ifdef CONFIG_TEGRA_DC_WIN_H
	device_remove_file(dev, &dev_attr_win_h);
#endif
	device_remove_file(dev, &dev_attr_cmu_enable);
#ifdef CONFIG_TEGRA_ISOMGR
	device_remove_file(dev, &dev_attr_reserved_bw);
#endif
	device_remove_file(dev, &dev_attr_panel_connected);

	if (dc->out->stereo) {
		device_remove_file(dev, &dev_attr_stereo_orientation);
		device_remove_file(dev, &dev_attr_stereo_mode);
	}

	if (dc->fb)
		tegra_fb_remove_sysfs(dev);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		device_remove_file(dev, &dev_attr_smart_panel);

	if (dc->out->type != TEGRA_DC_OUT_HDMI)
		device_remove_file(dev, &dev_attr_panel_rotation);
}

void tegra_dc_create_sysfs(struct device *dev)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_vrr *vrr  = dc->out->vrr;
	int error = 0;

	error |= device_create_file(dev, &dev_attr_mode);
	error |= device_create_file(dev, &dev_attr_enable);
	error |= device_create_file(dev, &dev_attr_stats_enable);
	error |= device_create_file(dev, &dev_attr_crc_checksum_latched);
	if ((dc->out->type == TEGRA_DC_OUT_HDMI) ||
		(dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP))
		error |= device_create_file(dev, &dev_attr_out_crc);
	error |= device_create_file(dev, &dev_attr_win_mask);

#ifdef CONFIG_TEGRA_DC_WIN_H
	error |= device_create_file(dev, &dev_attr_win_h);
#endif
	error |= device_create_file(dev, &dev_attr_cmu_enable);
#ifdef CONFIG_TEGRA_ISOMGR
	error |= device_create_file(dev, &dev_attr_reserved_bw);
#endif
	error |= device_create_file(dev, &dev_attr_scanline);

	if (dc->out->stereo) {
		error |= device_create_file(dev, &dev_attr_stereo_orientation);
		error |= device_create_file(dev, &dev_attr_stereo_mode);
	}

	error |= device_create_file(dev, &dev_attr_panel_connected);

	if (vrr)
#ifdef CONFIG_TEGRA_VRR
		error |= vrr_create_sysfs(dev);
#endif

	if (dc->fb)
		error |= tegra_fb_create_sysfs(dev);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		error |= device_create_file(dev, &dev_attr_smart_panel);
	if (dc->out->type != TEGRA_DC_OUT_HDMI)
		error |= device_create_file(dev, &dev_attr_panel_rotation);

	if (error)
		dev_err(&ndev->dev, "Failed to create sysfs attributes!\n");
}
