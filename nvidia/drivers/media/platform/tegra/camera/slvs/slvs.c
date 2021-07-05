/*
 * NVIDIA Tegra SLVS(-EC) Subdevice for V4L2
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Pekka Pessi <ppessi@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/nospec.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <soc/tegra/chip-id.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/camera_common.h>
#include <media/mc_common.h>
#include <media/slvs.h>

#include <soc/tegra/chip-id.h>

#include "dev.h"
#include "linux/nvhost.h"
#include <linux/version.h>

#define SLVSEC_NUM_LANES 8

#define SYNCGEN_CLOCK  U32_C(408000)
#define XSYNC_WIDTH 4
#define VI_NUM_VGP 6
#define VI_NUM_SYNCGEN 3

struct slvsec_stream_params {
	u8 rate;
	u8 lanes[8];
	u8 num_lanes;
	bool enable_header_crc;
	bool enable_payload_crc;
	u32 watchdog_period;
	u32 symbols;

	struct slvsec_cil_stream_uphy {
		bool aux_idle_detect_force;
		bool cal_skip;
		bool rate_gen_2;
		bool term_other;
		bool term_data;
		bool aux_term_other;
		bool aux_term_data;
		bool aux_term_10kohm;
		bool skip_sleep;
		bool dedicated_calibration;
		u8 aux_idle_mode;
		bool aux_idle_detect;
	} uphy;

	struct slvsec_syncgen {
		u32 number;	/* 0..2 */
		u32 xvs_vgp;
		u32 xhs_vgp;
	} syncgen;
};

struct slvsec_lane_params {
	bool stream1;
	bool half_pad_code;
	bool half_start_code;
	bool switch_polarity;
};

struct slvec_uphy_params {
	u32 blank_timeout;
	u8 sleep_to_iddq;
	u8 iddq_to_sleep;
	u8 dataen_to_sleep;
	u8 sleep_to_dataen;
	u8 dataen_step;
	u8 sleep_step;
	u8 sleep_cali;
	u8 cali_dataen;
	u8 cali_sleep;
	bool always_on_iopll;
	bool standby_off_iopll;
};

struct slvsec_params {
	struct slvsec_lane_params lanes[SLVSEC_NUM_LANES];
	struct slvec_uphy_params uphy;
};

struct slvsec_syncgen_config {
	u32 hclk_div;
	u8 hclk_div_fmt;
	u8 xhs_width;
	u8 xvs_width;
	u8 xvs_to_xhs_delay;
	u16 xvs_interval;
};

struct tegra_mc_slvs {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *slvs_base;
	void __iomem *vi_base;

	u32 syncgen_clock;

	atomic_t power_ref;
	atomic_t sensor_active;

	struct slvsec_params params;

	struct tegra_slvs_stream *streams;
	int num_streams;

	struct dentry *debugdir;
};

struct tegra_slvs_stream {
	struct tegra_mc_slvs *slvs;
	struct v4l2_subdev subdev;
	struct media_pad pads[2];

	u32 id;
	struct slvsec_stream_params params;
	atomic_t is_streaming;
	wait_queue_head_t cil_waitq;

	struct camera_common_framesync framesync;
	struct dentry *debugfs;
	bool syncgen_active;
};

#define to_slvs(_sd) \
	container_of(_sd, struct tegra_slvs_stream, subdev)->slvs

static int tegra_slvs_s_power(struct v4l2_subdev *sd, int enable)
{
	struct tegra_mc_slvs *slvs = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_info(&slvs->pdev->dev, "%s(%s)\n", __func__,
		enable ? "enable" : "disable");

	if (enable) {
		err = nvhost_module_busy(slvs->pdev);
		if (err == 0)
			atomic_inc(&slvs->power_ref);
		else
			dev_err(&slvs->pdev->dev,
				"s_power: failed (%d)\n", err);
	} else {
		nvhost_module_idle(slvs->pdev);
		atomic_dec(&slvs->power_ref);
	}

	return err;
}

#define SLVS_STREAM_OFFSET	0x10000
#define SLVS_STREAM_PAGE_SIZE	0x10000
#define SLVS_CIL_OFFSET		0x30000
#define SLVS_CIL_STREAM_OFFSET	0x30800
#define SLVS_CIL_STREAM_PAGE_SIZE	0x00800
#define SLVS_CIL_LANE_OFFSET	0x31800
#define SLVS_CIL_LANE_SIZE	0x00400

#define SLVS_HOST1X_INTR_STATUS		0x00b0
#define SLVS_STRM_CTRL			0x0000
#define SLVS_STRM_RST_CTRL		0x0004
#define SLVS_STRM_CLK_CTRL		0x0008
#define SLVS_STRM_CH0_CFG		0x000C
#define SLVS_STRM_CH1_CFG		0x0010
#define SLVS_STRM_CH0_EMBD_CFG		0x0014
#define SLVS_STRM_CH1_EMBD_CFG		0x0018
#define SLVS_STRM_TIMEOUT_CTRL		0x001C
#define SLVS_STRM_INTR_MASK		0x0044
#define SLVS_STRM_INTR_MASK_CH0		0x003C
#define SLVS_STRM_INTR_MASK_CH1		0x0040
#define SLVS_STRM_VI_ERR_MASK_CH0	0x0034
#define SLVS_STRM_VI_ERR_MASK_CH1	0x0038
#define SLVS_STRM_INTR_STATUS_CH0	0x0048
#define SLVS_STRM_INTR_STATUS_CH1	0x004C
#define SLVS_STRM_INTR_STATUS		0x0050


#define SLVS_CIL_CTRL			0x00
#define SLVS_CIL_LANE_SWIZZLE		0x04
#define SLVS_CIL_UPHY_CTRL0		0x08
#define SLVS_CIL_UPHY_CTRL1		0x0c
#define SLVS_CIL_UPHY_CTRL2		0x10

#define SLVS_CIL_STRM_CLK_CTRL		0x00
#define SLVS_CIL_STRM_RST_CTRL		0x04
#define SLVS_CIL_STRM_CTRL		0x08
#define SLVS_CIL_STRM_SYM_DEFINE	0x0c
#define SLVS_CIL_STRM_UPHY_MODE		0x10

#define SLVS_CIL_STRM_UPHY_CAL_EN	BIT(3)

#define SLVS_CIL_STRM_INTR_MASK		0x1c
#define SLVS_CIL_STRM_INTR_STATUS	0x20
#define SLVS_CIL_STRM_INTR_CAL_DONE	BIT(3)
#define SLVS_CIL_STRM_CAL_STATUS	0x24
#define SLVS_CIL_STRM_CAL_STATUS_DONE	BIT(0)

#define SLVS_LANE_CLK_CTRL		0x00
#define SLVS_LANE_RST_CTRL		0x04
#define SLVS_LANE_CTRL			0x08
#define SLVS_LANE_STATUS		0x0C

#define TEGRA_IMAGE_DT_RAW16	46

/*
 * SLVS-EC register accessors
 */
static inline void slvs_core_write(struct tegra_mc_slvs *slvs,
		u32 offset, u32 val)
{
	writel(val, slvs->slvs_base + offset);
}

static inline u32 slvs_core_read(struct tegra_mc_slvs *slvs,
		u32 offset)
{
	return readl(slvs->slvs_base + offset);
}

static inline void slvs_stream_write(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset, u32 val)
{
	writel(val, slvs->slvs_base +
		SLVS_STREAM_OFFSET +
		id * SLVS_STREAM_PAGE_SIZE +
		offset);
}

static inline u32 slvs_stream_read(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset)
{
	return readl(slvs->slvs_base +
		SLVS_STREAM_OFFSET +
		id * SLVS_STREAM_PAGE_SIZE +
		offset);
}

static inline void slvs_cil_write(struct tegra_mc_slvs *slvs,
				u32 offset, u32 val)
{
	writel(val, slvs->slvs_base +
		SLVS_CIL_OFFSET + offset);
}

static inline u32 slvs_cil_read(struct tegra_mc_slvs *slvs,
				u32 offset)
{
	return readl(slvs->slvs_base +
		SLVS_CIL_OFFSET + offset);
}

static inline void slvs_cil_stream_write(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset, u32 val)
{
	writel(val, slvs->slvs_base +
		SLVS_CIL_STREAM_OFFSET +
		id * SLVS_CIL_STREAM_PAGE_SIZE +
		offset);
}

static inline u32 slvs_cil_stream_read(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset)
{
	return readl(slvs->slvs_base +
		SLVS_CIL_STREAM_OFFSET +
		id * SLVS_CIL_STREAM_PAGE_SIZE +
		offset);
}

static inline void slvs_cil_lane_write(struct tegra_mc_slvs *slvs,
				u32 lane, u32 offset, u32 val)
{
	writel(val, slvs->slvs_base +
		SLVS_CIL_LANE_OFFSET +
		lane * SLVS_CIL_LANE_SIZE +
		offset);
}

static inline u32 slvs_cil_lane_read(struct tegra_mc_slvs *slvs,
				u32 lane, u32 offset)
{
	return readl(slvs->slvs_base +
		SLVS_CIL_LANE_OFFSET +
		lane * SLVS_CIL_LANE_SIZE +
		offset);
}

/* SLVS parameters */

static int slvs_stream_config(const struct tegra_channel *chan)
{
	u32 data_type = chan->fmtinfo->img_dt;
	u32 width = chan->format.width;
	u32 pixel_width = chan->fmtinfo->width;
	u32 stuffing = 16 - ((width * pixel_width + 7U) / 8U) % 16;
	u32 pd_length = ((width * pixel_width + 7U) / 8U);
	u32 line_length = pd_length + stuffing;

	switch (data_type) {
	case TEGRA_IMAGE_DT_RAW8:
	case TEGRA_IMAGE_DT_RAW10:
	case TEGRA_IMAGE_DT_RAW12:
	case TEGRA_IMAGE_DT_RAW14:
	case TEGRA_IMAGE_DT_RAW16:
		break;
	default:
		WARN(1, "Unsupported data type %u", data_type);
		return -EINVAL;
	}

	if (WARN_ON(pd_length >= 0x20000))
		return -EINVAL;
	if (WARN(line_length % 16 != 0,
			"bad slvs-ec stuffing: width=%u pixel=%ub",
			width, pixel_width))
		return -EINVAL;

	return (data_type << 22) | (stuffing << 18) | (pd_length);
}

static int slvs_cil_stream_uphy_mode(const struct tegra_slvs_stream *stream)
{
	const struct slvsec_cil_stream_uphy *uphy = &stream->params.uphy;

	return
		(uphy->aux_idle_detect_force << 15) |
		(uphy->cal_skip << 14) |
		(uphy->rate_gen_2 << 11) |
		(uphy->term_other << 10) |
		(uphy->term_data << 9) |
		(uphy->aux_term_other << 8) |
		(uphy->aux_term_data << 7) |
		(uphy->aux_term_10kohm << 6) |
		(uphy->skip_sleep << 5) |
		(uphy->dedicated_calibration << 4) |
		(uphy->aux_idle_mode << 1) |
		(uphy->aux_idle_detect << 0);
}

/* ---------------------------------------------------------------------- */
/* SYNCGEN */
/* VI registers and accessors */

#define VI_FW_CFG_IO_VGP       		0x4400

#define VI_FW_CFG_ENABLE_XVS(num)	(BIT(16) | ((2 + 2 * (num)) << 17))
#define VI_FW_CFG_ENABLE_XHS(num)	(BIT(16) | ((3 + 2 * (num)) << 17))

#define VI_FW_SYNCGEN_OFFSET		0x4800
#define VI_FW_SYNCGEN_SIZE		0x0400

#define VI_FW_SYNCGEN_HCLK_DIV		0x00
#define VI_FW_SYNCGEN_HCLK_DIV_FMT	0x04
#define VI_FW_SYNCGEN_XHS		0x08
#define VI_FW_SYNCGEN_XVS		0x0c
#define VI_FW_SYNCGEN_XVS_DELAY		0x10
#define VI_FW_SYNCGEN_INT_STATUS	0x14
#define VI_FW_SYNCGEN_INT_MASK		0x18
#define VI_FW_SYNCGEN_XHS_TIMER		0x1c
#define VI_FW_SYNCGEN_CONTROL		0x20
#define VI_FW_SYNCGEN_COMMAND		0x24
#define VI_FW_SYNCGEN_STATUS		0x28
#define VI_FW_SYNCGEN_SCAN		0x2C
#define VI_FW_SYNCGEN_FORCE_XVS		0x30

static inline void slvs_vi_vgp_config(struct tegra_mc_slvs *slvs,
				u32 vgp_offset, u32 val)
{
	if (vgp_offset > 0)
		writel(val, slvs->vi_base + VI_FW_CFG_IO_VGP +
			(vgp_offset - 1) * 4);
}

static inline void slvs_syncgen_write(struct tegra_mc_slvs *slvs,
				u32 syncgen, u32 offset, u32 val)
{
	writel(val, slvs->vi_base +
		VI_FW_SYNCGEN_OFFSET +
		syncgen * VI_FW_SYNCGEN_SIZE +
		offset);
}

static inline void slvs_vi_syncgen_start(struct tegra_mc_slvs *slvs,
		u32 syncgen, const struct slvsec_syncgen_config *cfg)
{
	if (tegra_platform_is_silicon()) {
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_HCLK_DIV, cfg->hclk_div);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_HCLK_DIV_FMT, cfg->hclk_div_fmt);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_XHS, cfg->xhs_width);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_XVS,
				cfg->xvs_width | (cfg->xvs_interval << 8));
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_XVS_DELAY, cfg->xvs_to_xhs_delay);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_INT_MASK, 0);
	} else {
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_HCLK_DIV, 0x2bc);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_HCLK_DIV_FMT, 0);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_XHS, 0xa);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_XVS, 0xfa00a);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_XVS_DELAY, 0x2);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_INT_MASK, 0xffffffff);
		slvs_syncgen_write(slvs, syncgen,
				VI_FW_SYNCGEN_INT_STATUS, 0xffffffff);
	}
	slvs_syncgen_write(slvs, syncgen,
			VI_FW_SYNCGEN_COMMAND, BIT(0));
	slvs_syncgen_write(slvs, syncgen,
			VI_FW_SYNCGEN_CONTROL, BIT(0));
}

static inline void slvs_vi_syncgen_stop(struct tegra_mc_slvs *slvs,
		u32 syncgen)
{
	slvs_syncgen_write(slvs, syncgen, VI_FW_SYNCGEN_CONTROL, 0);
	slvs_syncgen_write(slvs, syncgen, VI_FW_SYNCGEN_COMMAND, BIT(1));
}

static int slvs_get_syncgen_config(
	struct tegra_mc_slvs *slvs,
	const struct camera_common_framesync *fs,
	struct slvsec_syncgen_config *cfg)
{
	u64 syncgen_clock = slvs->syncgen_clock;
	/* HS per 1000 second. */
	u64 hs_per_1000sec = (u64)fs->xvs * fs->fps;
	u64 xhs;
	u32 frac;
	u32 width;

	if (hs_per_1000sec == 0)
		return -EINVAL;

	xhs = ((10 * 1000 * syncgen_clock) << 32) / hs_per_1000sec * 100;

	for (frac = 0; (xhs >> (frac + 32)) != 0; frac++)
		;

	cfg->hclk_div = xhs >> frac;
	cfg->hclk_div_fmt = 31 - frac;

	/* At least XSYNC_WIDTH iclk cycles */
	width = (XSYNC_WIDTH * syncgen_clock + fs->inck - 1) / fs->inck;
	if (width < XSYNC_WIDTH)
		width = XSYNC_WIDTH;

	cfg->hclk_div_fmt = 0x16;
	cfg->xhs_width = width;
	cfg->xvs_width = width;
	cfg->xvs_to_xhs_delay = 3;
	cfg->xvs_interval = fs->xvs;

	return 0;
}

static int tegra_slvs_syncgen_start(struct tegra_slvs_stream *stream)
{
	struct tegra_mc_slvs *slvs = stream->slvs;
	u32 syncgen = stream->params.syncgen.number;
	u32 xvs_vgp = stream->params.syncgen.xvs_vgp;
	u32 xhs_vgp = stream->params.syncgen.xhs_vgp;
	struct slvsec_syncgen_config config;
	int err;

	err = slvs_get_syncgen_config(slvs,
		&stream->framesync, &config);
	if (err)
		return err;

	slvs_vi_vgp_config(slvs, xvs_vgp, VI_FW_CFG_ENABLE_XVS(syncgen));
	slvs_vi_vgp_config(slvs, xhs_vgp, VI_FW_CFG_ENABLE_XHS(syncgen));
	slvs_vi_syncgen_start(slvs, syncgen, &config);

	return 0;
}

static int tegra_slvs_syncgen_stop(struct tegra_slvs_stream *stream)
{
	struct tegra_mc_slvs *slvs = stream->slvs;

	slvs_vi_syncgen_stop(slvs, stream->params.syncgen.number);
	slvs_vi_vgp_config(slvs, stream->params.syncgen.xvs_vgp, 0);
	slvs_vi_vgp_config(slvs, stream->params.syncgen.xhs_vgp, 0);

	return 0;
}

/*
 * -----------------------------------------------------------------------------
 * Debugfs
 * -----------------------------------------------------------------------------
 */

static void fractional_binary(char buffer[35], u32 value, u32 fraction)
{
	int i, point;

	for (i = 0, point = 0; i < 32; i++) {
		if (i == 32 - fraction) {
			if (i == 0)
				buffer[i + point++] = '0';
			buffer[i + point++] = '.';
		}

		buffer[i + point] = (value & BIT(31 - i)) ? '1' : '0';
	}

	buffer[i + point] = '\0';
}

static int slvs_debugfs_syncgen_config_read(struct seq_file *s, void *data)
{
	struct tegra_slvs_stream *stream = s->private;
	struct tegra_mc_slvs *slvs = stream->slvs;
	u64 syncgen_clock = slvs->syncgen_clock;
	int err;
	struct slvsec_syncgen_config config;
	char buffer[35];
	u64 framerate;

	err = slvs_get_syncgen_config(slvs, &stream->framesync, &config);
	if (err < 0)
		return err;

	if (config.hclk_div == 0 || config.xvs_interval == 0)
		return -EINVAL;

	fractional_binary(buffer, config.hclk_div, config.hclk_div_fmt);

	framerate = (1000 * syncgen_clock) << 32;
	framerate /= config.hclk_div;
	framerate /= config.xvs_interval;
	/* Multiply framerate by 2000 followed by right shift by 32 */
	framerate *= 2000 >> 3;
	if (config.hclk_div_fmt > (32 - 3))
		framerate <<= config.hclk_div_fmt - (32 - 3);
	else
		framerate >>= (32 - 3) - config.hclk_div_fmt;

	seq_printf(s, "clk = %llu\n", 1000ULL * syncgen_clock);
	seq_printf(s, "hclk_div = %u >> %u (%s)\n",
		config.hclk_div, config.hclk_div_fmt, buffer);
	seq_printf(s, "xvs = %u\n", config.xvs_interval);
	seq_printf(s, "fps = %u\n", (unsigned)framerate);
	seq_printf(s, "xhs_width = %u\n", config.xhs_width);
	seq_printf(s, "xvs_width = %u\n", config.xvs_width);
	seq_printf(s, "xvs_to_xhs_delay = %u\n", config.xvs_to_xhs_delay);

	return 0;
}

static int slvs_debugfs_syncgen_config_open(struct inode *inode, struct file *f)
{
	struct tegra_slvs_stream *stream = inode->i_private;

	return single_open(f, slvs_debugfs_syncgen_config_read, stream);
}

static const struct file_operations slvs_debugfs_syncgen_config_ops = {
	.owner = THIS_MODULE,
	.open = slvs_debugfs_syncgen_config_open,
	.release = single_release,
	.read = seq_read,
	.llseek = seq_lseek,
};

static int slvs_debugfs_show_syncgen_active(void *data, u64 *val)
{
	struct tegra_slvs_stream *stream = data;

	*val = (u64)stream->syncgen_active;

	return 0;
}

static int slvs_debugfs_store_syncgen_active(void *data, u64 val)
{
	struct tegra_slvs_stream *stream = data;
	int err;

	if (val > 1U)
		return -EINVAL;

	if ((bool)val == stream->syncgen_active)
		return 0;

	if (val)
		err = tegra_slvs_syncgen_start(stream);
	else
		err = tegra_slvs_syncgen_stop(stream);

	if (err)
		return err;

	stream->syncgen_active = (bool)val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(slvs_debugfs_fops_syncgen_active,
			slvs_debugfs_show_syncgen_active,
			slvs_debugfs_store_syncgen_active,
			"%lld\n");

static void tegra_slvs_debugfs_init_stream(struct tegra_slvs_stream *stream)
{
	struct dentry *dir, *paramdir;

	debugfs_create_atomic_t("is_streaming", S_IRUGO, stream->debugfs,
				&stream->is_streaming);

	dir = debugfs_create_dir("syncgen", stream->debugfs);
	debugfs_create_u32("number", S_IRUGO | S_IWUSR, dir,
			&stream->params.syncgen.number);
	debugfs_create_u32("xvs_vgp", S_IRUGO | S_IWUSR, dir,
			&stream->params.syncgen.xvs_vgp);
	debugfs_create_u32("xhs_vgp", S_IRUGO | S_IWUSR, dir,
			&stream->params.syncgen.xhs_vgp);
	debugfs_create_file("config", S_IRUGO, dir, stream,
			&slvs_debugfs_syncgen_config_ops);
	debugfs_create_file("active", S_IRUGO, dir, stream,
			&slvs_debugfs_fops_syncgen_active);

	paramdir = debugfs_create_dir("params", dir);
	debugfs_create_u32("inck", S_IRUGO | S_IWUSR, paramdir,
			&stream->framesync.inck);
	debugfs_create_u32("xhs", S_IRUGO | S_IWUSR, paramdir,
			&stream->framesync.xhs);
	debugfs_create_u32("xvs", S_IRUGO | S_IWUSR, paramdir,
			&stream->framesync.xvs);
	debugfs_create_u32("fps", S_IRUGO | S_IWUSR, paramdir,
			&stream->framesync.fps);

}

static void tegra_slvs_init_debugfs(struct tegra_mc_slvs *slvs)
{
	struct nvhost_device_data *info = platform_get_drvdata(slvs->pdev);
	int i;
	char name[32];
	struct dentry *dir;

	debugfs_create_u32("syncgen-clock", S_IRUGO | S_IWUSR, info->debugfs,
			&slvs->syncgen_clock);

	for (i = 0; i < slvs->num_streams; i++) {
		snprintf(name, sizeof(name), "mc@%d", i);
		dir = debugfs_create_dir(name, info->debugfs);
		slvs->streams[i].debugfs = dir;
		tegra_slvs_debugfs_init_stream(&slvs->streams[i]);
	}
	speculation_barrier();
}

/*
 * -----------------------------------------------------------------------------
 * SLVS Subdevice Video Operations
 * -----------------------------------------------------------------------------
 */
static inline bool tegra_slvs_is_cal_done(struct tegra_slvs_stream *stream)
{
	u32 val = slvs_cil_stream_read(stream->slvs, stream->id,
				SLVS_CIL_STRM_CAL_STATUS);

	return (val & SLVS_CIL_STRM_CAL_STATUS_DONE) == 0;
}

static int tegra_slvs_start_streaming(struct tegra_slvs_stream *stream)
{
	struct tegra_mc_slvs *slvs = stream->slvs;
	struct tegra_channel *tegra_chan;
	u32 id = stream->id;
	int i, cfg, err;
	u32 val;

	if (WARN_ON(id != 0))
		return -ENXIO;

	dev_info(&slvs->pdev->dev, "%s()\n", __func__);

	tegra_chan = v4l2_get_subdev_hostdata(&stream->subdev);
	cfg = slvs_stream_config(tegra_chan);
	if (cfg < 0)
		return cfg;

	err = camera_common_get_framesync(tegra_chan->subdev_on_csi,
				&stream->framesync);
	if (err < 0)
		return err;

	/* XXX - where we should dig embedded data type ? */

	slvs_core_write(slvs, SLVS_HOST1X_INTR_STATUS, 0x1);
	slvs_stream_write(slvs, id, SLVS_STRM_INTR_STATUS, 0x9);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_INTR_STATUS, 0x1f);
	slvs_stream_write(slvs, id, SLVS_STRM_INTR_STATUS_CH0, 0xf);
	slvs_stream_write(slvs, id, SLVS_STRM_INTR_STATUS_CH1, 0xf);
	slvs_stream_write(slvs, id, SLVS_STRM_INTR_MASK, 0x2);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_INTR_MASK, 0x8);

	/* Reset stream and CIL */
	slvs_stream_write(slvs, id, SLVS_STRM_RST_CTRL, 1);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_RST_CTRL, 1);
	slvs_stream_write(slvs, id, SLVS_STRM_RST_CTRL, 0);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_RST_CTRL, 0);

	/* Reset all lanes */
	for (i = 0; i < SLVSEC_NUM_LANES; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_RST_CTRL, 1);
	for (i = 0; i < SLVSEC_NUM_LANES; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_RST_CTRL, 0);

	/* Calibration */
	for (i = 0; i < SLVSEC_NUM_LANES; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_CTRL, 1);
	val = slvs_cil_stream_read(slvs, id, SLVS_CIL_STRM_UPHY_MODE);
	val |= 1 << 11;
	val |= SLVS_CIL_STRM_UPHY_CAL_EN;
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_UPHY_MODE, val);
	val = (stream->params.num_lanes << 1) | 1;
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, val);

	/* Wait for CAL_DONE */
	usleep_range(100, 110);

	for (i = 0; i < SLVSEC_NUM_LANES; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_CTRL, 0);
	val = slvs_cil_stream_read(slvs, id, SLVS_CIL_STRM_UPHY_MODE);
	val &= ~(1 << 3);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_UPHY_MODE, val);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, 1);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, 0);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, 1);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, 0);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CAL_STATUS,
			SLVS_CIL_STRM_CAL_STATUS_DONE);

	/* Wait for CAL_DONE */
	usleep_range(100, 110);

	/* Program core */
	val = cfg | (stream->params.enable_payload_crc << 28);
	if (tegra_platform_is_silicon()) {
		slvs_stream_write(slvs, id, SLVS_STRM_CH0_CFG, val);
		slvs_stream_write(slvs, id, SLVS_STRM_CH1_CFG, val);
		/* write only if embedded data exists */
		/* TODO: Add proper check */
		slvs_stream_write(slvs, id, SLVS_STRM_CH0_EMBD_CFG, 0x2b);
		slvs_stream_write(slvs, id, SLVS_STRM_CH1_EMBD_CFG, 0x2b);
		/* TODO: Add proper support for this */
		slvs_stream_write(slvs, id, SLVS_STRM_CTRL, 0x5);
		slvs_stream_write(slvs, id, SLVS_STRM_TIMEOUT_CTRL, 0xffffffff);
		slvs_stream_write(slvs, id, SLVS_STRM_CLK_CTRL, 0x0);
	} else {
		val = 0x1b200a68;
		slvs_stream_write(slvs, id, SLVS_STRM_CH0_CFG, val);
		slvs_stream_write(slvs, id, SLVS_STRM_CH1_CFG, val);
		slvs_cil_write(slvs, SLVS_CIL_CTRL, 0x56f1);
		slvs_cil_write(slvs, SLVS_CIL_LANE_SWIZZLE, 0x76543210);
	}

	/* Program CIL */
	if (tegra_platform_is_silicon()) {
		slvs_cil_write(slvs, SLVS_CIL_CTRL, 0x5ff1);
		slvs_cil_write(slvs, SLVS_CIL_LANE_SWIZZLE, 0x01234567);
		slvs_cil_write(slvs, SLVS_CIL_UPHY_CTRL0, 0xf0f800f);
		slvs_cil_write(slvs, SLVS_CIL_UPHY_CTRL1, 0x50580);
		slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_SYM_DEFINE,
				stream->params.symbols);
		val = slvs_cil_stream_uphy_mode(stream);
		slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_UPHY_MODE, val);

		slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CLK_CTRL, 0);
		for (i = 0; i < stream->params.num_lanes; i++)
			slvs_cil_lane_write(slvs, i, SLVS_LANE_CLK_CTRL, 0);
	}

	if (!tegra_platform_is_silicon()) {
		slvs_stream_write(slvs, id, SLVS_STRM_CTRL, 1);
		val = (stream->params.num_lanes << 1) | 1;
		slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, val);
	}

	/* Enable lanes */
	slvs_stream_write(slvs, id, SLVS_STRM_CTRL, 0x1);
	for (i = 0; i < stream->params.num_lanes; i++) {
		/* XXX id=1 => STRM1_LANE_BEG_NUM */
		slvs_cil_lane_write(slvs, i, SLVS_LANE_CTRL, 1);
	}

	val = (stream->params.num_lanes << 1) | 1;
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, val);

	err = tegra_slvs_syncgen_start(stream);
	if (err < 0)
		return err;

	dev_info(&slvs->pdev->dev, "%s() (done)\n", __func__);
	return 0;
}

static void tegra_slvs_cil_notify(struct tegra_slvs_stream *stream,
	u32 events)
{
	(void)events;

	if (tegra_slvs_is_cal_done(stream)) {
		slvs_cil_stream_write(stream->slvs, stream->id,
				SLVS_CIL_STRM_INTR_MASK, 0);
		wake_up(&stream->cil_waitq);
	}
}

static int tegra_slvs_stop_streaming(struct tegra_slvs_stream *stream)
{
	struct tegra_mc_slvs *slvs = stream->slvs;
	u32 id = stream->id;
	int i;

	slvs_stream_write(slvs, id, SLVS_STRM_CTRL, 0);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, 0);
	for (i = 0; i < stream->params.num_lanes; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_CTRL, 0);

	tegra_slvs_syncgen_stop(stream);

	return 0;
}

static int tegra_slvs_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_slvs_stream *stream;
	int ret;

	stream = container_of(subdev, struct tegra_slvs_stream, subdev);

	dev_info(&stream->slvs->pdev->dev, "%s(%s)\n", __func__,
		enable ? "enable" : "disable");

	if (atomic_read(&stream->is_streaming) == enable)
		return 0;

	if (enable)
		ret = tegra_slvs_start_streaming(stream);
	else
		ret = tegra_slvs_stop_streaming(stream);

	atomic_set(&stream->is_streaming, enable);

	return ret;
}

static int tegra_slvs_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tegra_mc_slvs *slvs = v4l2_get_subdevdata(sd);

	/*
	 * Set status to 0 if power is on
	 * Set status to 1 if power is off
	 */
	*status = atomic_read(&slvs->power_ref) == 0;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static const struct v4l2_subdev_video_ops tegra_slvs_video_ops = {
	.s_stream	= tegra_slvs_s_stream,
	.g_input_status = tegra_slvs_g_input_status,
};

static const struct v4l2_subdev_core_ops tegra_slvs_core_ops = {
	.s_power	= tegra_slvs_s_power,
};

static const struct v4l2_subdev_ops tegra_slvs_ops = {
	.core	= &tegra_slvs_core_ops,
	.video  = &tegra_slvs_video_ops,
};

static const struct media_entity_operations tegra_slvs_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * Media controller
 */
static int tegra_slvs_init_stream(struct tegra_mc_slvs *slvs,
				struct tegra_slvs_stream *stream)
{
	struct v4l2_subdev *sd;
	int err;

	atomic_set(&stream->is_streaming, 0);

	/* Initialize V4L2 subdevice and media entity */
	sd = &stream->subdev;
	v4l2_subdev_init(sd, &tegra_slvs_ops);
	sd->dev = slvs->dev;
	v4l2_set_subdevdata(sd, slvs);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &tegra_slvs_media_ops;
	snprintf(sd->name, sizeof(sd->name), "%s-stream-%u",
		dev_name(slvs->dev), stream->id);

	/* Initialize media entity */
	/* XXX - use media_entity_pads_init() directly? */
	err = tegra_media_entity_init(&sd->entity,
				ARRAY_SIZE(stream->pads),
				stream->pads, true, false);
	if (err < 0) {
		dev_err(slvs->dev, "stream@%u: failed to register pads\n",
			stream->id);
		return err;
	}

	err = v4l2_async_register_subdev(sd);
	if (err < 0) {
		dev_err(slvs->dev, "stream@%u: failed to register subdev\n",
			stream->id);
		return err;
	}

	return 0;
}

static int tegra_slvs_init_streams(struct tegra_mc_slvs *slvs)
{
	int i;
	int err;

	for (i = 0; i < slvs->num_streams; i++) {
		err = tegra_slvs_init_stream(slvs, &slvs->streams[i]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int tegra_slvs_count_streams(struct device_node *np)
{
	struct device_node *streams_node, *stream_node;
	int count = 0;

	streams_node = of_get_child_by_name(np, "streams");

	for_each_available_child_of_node(streams_node, stream_node) {
		if (stream_node->name &&
			of_node_cmp(stream_node->name, "stream") == 0)
			count++;
	}

	of_node_put(streams_node);

	return count;
}

static struct device_node *tegra_slvs_get_stream_node(struct device_node *np,
						int num)
{
	struct device_node *streams_node, *stream_node;
	int count = 0;

	streams_node = of_get_child_by_name(np, "streams");

	for_each_available_child_of_node(streams_node, stream_node) {
		if (stream_node->name &&
			of_node_cmp(stream_node->name, "stream") == 0) {
			if (num == count)
				break;
			count++;
		}
	}

	of_node_put(streams_node);

	return stream_node;
}

static const struct camera_common_framesync slvs_default_framesync =
{
	/* Default mode 0 from VI IAS */
	.inck = 74000,
	.xhs = 600,
	.xvs = 4004,
	.fps = 29970
};

static int tegra_slvs_parse_stream_dt(struct tegra_slvs_stream *stream,
				struct device_node *np)
{
	struct slvsec_stream_params *params = &stream->params;
	int i, err;
	bool boolparam;
	u32 numparam;

	err = of_property_read_u32(np, "reg", &numparam);
	if (err < 0)
		stream->id = numparam;

	err = of_property_count_elems_of_size(np, "lanes", sizeof(u32));
	if (err < 0)
		return err;
	/* We can have 1, 2, 4, 6 or 8 lanes */
	if (err <= 0 || err > SLVSEC_NUM_LANES || ((err % 2) == 1 && err > 1))
		return -EINVAL;
	params->num_lanes = err;

	if (!tegra_platform_is_silicon())
		params->num_lanes = 2;

	for (i = 0; i < params->num_lanes; i++) {
		u32 lane;
		err = of_property_read_u32_index(np, "lanes", i, &lane);
		if (err)
			return err;
		if (lane > SLVSEC_NUM_LANES)
			return -EINVAL;
		params->lanes[i] = lane;
	}

	boolparam = of_property_read_bool(np, "nvidia,disable-header-crc");
	params->enable_header_crc = !boolparam;
	boolparam = of_property_read_bool(np, "nvidia,disable-payload-crc");
	params->enable_payload_crc = !boolparam;

	of_property_read_u32(np, "watchdog-period", &params->watchdog_period);

	err = of_property_read_u32(np, "nvidia,symbols", &params->symbols);
	if (err != 0)
		params->symbols = 0x000360aa;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-idle-force");
	params->uphy.aux_idle_detect_force = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-cal-skip");
	params->uphy.cal_skip = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-rate-gen2");
	params->uphy.rate_gen_2 = boolparam;

	boolparam =  of_property_read_bool(np, "nvidia,uphy-term-other");
	params->uphy.term_other = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-term-data");
	params->uphy.term_data = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-term-other");
	params->uphy.aux_term_other = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-term-data");
	params->uphy.aux_term_data = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-term-10kohm");
	params->uphy.aux_term_10kohm = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-skip-sleep");
	params->uphy.skip_sleep = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,"
					"uphy-dedicated-calibration");
	params->uphy.dedicated_calibration = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-idle-detect");
	params->uphy.aux_idle_detect = boolparam;

	err = of_property_read_u32(np, "nvidia,uphy-aux-idle-mode", &numparam);
	if (err == 0)
		params->uphy.aux_idle_mode = numparam & 0x03;

	of_property_read_u32(np, "nvidia,syncgen", &params->syncgen.number);
	if (params->syncgen.number > VI_NUM_SYNCGEN)
		return -EINVAL;

	/* VGP pads used for syncgen */
	of_property_read_u32(np, "nvidia,syncgen-xhs-vgp", &params->syncgen.xhs_vgp);
	if (!tegra_platform_is_silicon())
		params->syncgen.xhs_vgp = 2;
	if (params->syncgen.xhs_vgp > VI_NUM_VGP)
		return -EINVAL;

	of_property_read_u32(np, "nvidia,syncgen-xvs-vgp", &params->syncgen.xvs_vgp);
	if (!tegra_platform_is_silicon())
		params->syncgen.xvs_vgp = 3;
	if (params->syncgen.xvs_vgp > VI_NUM_VGP)
		return -EINVAL;

	stream->framesync = slvs_default_framesync;
	if (!tegra_platform_is_silicon()) {
		/* settings from tegra shell */
		stream->framesync.inck = 72000;
		stream->framesync.xhs = 300;
	}

	return 0;
}

static int tegra_slvs_parse_dt(struct tegra_mc_slvs *slvs)
{
	struct device_node *node = slvs->pdev->dev.of_node;
	int num_streams = tegra_slvs_count_streams(node);
	struct device_node *stream_node;
	struct tegra_slvs_stream *stream;
	int i;

	(void)of_property_read_u32(node, "nvidia,syncgen-clock",
				&slvs->syncgen_clock);

	if (num_streams <= 0) {
		dev_info(slvs->dev, "no streams defined");
		return -ENODEV;
	}

	dev_info(slvs->dev, "slvs has %d available streams", num_streams);

	slvs->num_streams = num_streams;
	slvs->streams = devm_kzalloc(slvs->dev,
				num_streams * sizeof(*stream), GFP_KERNEL);
	if (slvs->streams == NULL)
		return -ENOMEM;

	for (i = 0; i < num_streams; i++) {

		stream_node = tegra_slvs_get_stream_node(node, i);
		if (stream_node == NULL)
			goto error;

		stream = &slvs->streams[i];
		stream->slvs = slvs;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		stream->subdev.fwnode = of_fwnode_handle(stream_node);
#else
		stream->subdev.of_node = stream_node;
#endif
		stream->pads[0].flags = MEDIA_PAD_FL_SINK;
		stream->pads[1].flags = MEDIA_PAD_FL_SOURCE;

		tegra_slvs_parse_stream_dt(stream, stream_node);
	}

	return 0;

error:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	for (i = 0; i < num_streams; i++)
		of_node_put(to_of_node(slvs->streams[i].subdev.fwnode));
#else
	for (i = 0; i < num_streams; i++)
		of_node_put(slvs->streams[i].subdev.of_node);
#endif
	return -ENODEV;
}

void tegra_slvs_media_controller_cil_notify(struct tegra_mc_slvs *slvs,
					u32 id, u32 events)
{
	int i;

	if (IS_ERR_OR_NULL(slvs))
		return;

	for (i = 0; i < slvs->num_streams; i++) {
		if (slvs->streams[i].id == id)
			tegra_slvs_cil_notify(&slvs->streams[i], events);
	}
}
EXPORT_SYMBOL(tegra_slvs_media_controller_cil_notify);

struct tegra_mc_slvs *tegra_slvs_media_controller_init(
	struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct tegra_mc_slvs *slvs;
	int err;

	slvs = devm_kzalloc(&pdev->dev, sizeof(*slvs), GFP_KERNEL);
	slvs->dev = &pdev->dev;
	slvs->pdev = pdev;
	slvs->slvs_base = pdata->aperture[0];
	slvs->vi_base = pdata->aperture[1];

	slvs->syncgen_clock = SYNCGEN_CLOCK;

	if (tegra_platform_is_vdk())
		slvs->syncgen_clock = 72000; /* 72 MHz */

	atomic_set(&slvs->power_ref, 0);

	err = tegra_slvs_parse_dt(slvs);
	if (err < 0)
		return ERR_PTR(err);

	/*
	 * if there is no slvs channels listed in DT,
	 * no need to init the channel and graph
	 */
	if (slvs->num_streams > 0) {
		err = tegra_slvs_init_streams(slvs);
		if (err < 0)
			dev_err(&pdev->dev, "Failed to init slvs streams\n");
		else
			dev_info(&pdev->dev, "Initialized slvs streams for V4L2\n");
	}

	tegra_slvs_init_debugfs(slvs);

	return slvs;
}
EXPORT_SYMBOL(tegra_slvs_media_controller_init);

void tegra_slvs_media_controller_remove(struct tegra_mc_slvs *slvs)
{
	int i;

	if (IS_ERR_OR_NULL(slvs))
		return;

	for (i = 0; i < slvs->num_streams; i++) {
		struct tegra_slvs_stream *stream = &slvs->streams[i];
		v4l2_async_unregister_subdev(&stream->subdev);
		media_entity_cleanup(&stream->subdev.entity);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		of_node_put(to_of_node(stream->subdev.fwnode));
#else
		of_node_put(stream->subdev.of_node);
#endif
	}
}
EXPORT_SYMBOL(tegra_slvs_media_controller_remove);
