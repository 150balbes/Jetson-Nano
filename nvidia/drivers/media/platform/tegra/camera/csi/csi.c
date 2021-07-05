/*
 * NVIDIA Tegra CSI Device
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/nospec.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/camera_common.h>
#include <media/vi.h>
#include <media/csi.h>
#include <trace/events/camera_common.h>

#include "dev.h"
#include "mipical/mipi_cal.h"
#include "linux/nvhost.h"
#include <linux/version.h>

#include <asm/barrier.h>

#include "soc/tegra/camrtc-capture.h"
#include <uapi/linux/nvhost_nvcsi_ioctl.h>
#include "nvcsi/nvcsi.h"
#include "nvcsi/deskew.h"

#define DEFAULT_NUM_TPG_CHANNELS 6

/*
 * deskew should be run when the sensor data rate is >= 1.5 gbps
 * data is sent on both rising/falling edges of clock, so /2
 */
#define CLK_HZ_FOR_DESKEW ((1500*1000*1000)/2)

static struct tegra_csi_device *mc_csi;

struct tegra_csi_device *tegra_get_mc_csi(void)
{
	return mc_csi;
}
EXPORT_SYMBOL(tegra_get_mc_csi);

static int set_csi_properties(struct tegra_csi_device *csi,
			struct platform_device *pdev)
{
	struct camera_common_data *s_data = &csi->s_data[0];

	/*
	* These values are only used for tpg mode
	* With sensor, CSI power and clock info are provided
	* by the sensor sub device
	*/
	s_data->csi_port = 0;
	s_data->numlanes = 12;
	csi->clk_freq = TEGRA_CLOCK_CSI_PORT_MAX;

	return 0;
}

static void update_blank_intervals(struct tegra_csi_channel *chan,
		int portnum, int fmtindex)
{
	struct tegra_csi_port *port = &chan->ports[portnum];
	const struct tpg_frmfmt *tegra_csi_tpg_frmfmt =
						chan->csi->tpg_frmfmt_table;

	port->framerate = tegra_csi_tpg_frmfmt[fmtindex].framerate;
	port->h_blank = tegra_csi_tpg_frmfmt[fmtindex].h_blank;
	port->v_blank = tegra_csi_tpg_frmfmt[fmtindex].v_blank;
}

static struct sensor_mode_properties*
read_mode_from_dt(struct camera_common_data *s_data)
{
	struct sensor_mode_properties *mode = NULL;

	if (s_data) {
		int idx = s_data->mode_prop_idx;

		if (idx < s_data->sensor_props.num_modes)
			mode = &s_data->sensor_props.sensor_modes[idx];
	}

	return mode;
}

u32 read_settle_time_from_dt(struct tegra_csi_channel *chan)
{
	struct camera_common_data *s_data = chan->s_data;
	struct sensor_mode_properties *mode = read_mode_from_dt(s_data);
	struct device *dev = chan->csi->dev;
	unsigned int cil_settletime = 0;

	if (mode) {
		dev_dbg(dev, "settle time reading from props\n");
		cil_settletime = mode->signal_properties.cil_settletime;
	} else if (chan->of_node) {
		int err = 0;
		const char *str;

		dev_dbg(dev, "settle time reading from of_node\n");
		err = of_property_read_string(chan->of_node, "cil_settletime",
			&str);
		if (!err) {
			err = kstrtou32(str, 10, &cil_settletime);
			if (err) {
				dev_dbg(dev,
					"no cil_settletime in of_node");
				cil_settletime = 0;
			}
		}
	}

	return cil_settletime;
}

u32 read_discontinuous_clk_from_dt(struct tegra_csi_channel *chan)
{
	struct camera_common_data *s_data = chan->s_data;
	struct sensor_mode_properties *mode = read_mode_from_dt(s_data);
	struct device *dev = chan->csi->dev;
	unsigned int discontinuous_clk = 1;

	if (mode) {
		discontinuous_clk = mode->signal_properties.discontinuous_clk;
		dev_dbg(dev, "discontinuous_clk = %u reading from props\n", discontinuous_clk);
	} else if (chan->of_node) {
		int err = 0;
		const char *str;

		err = of_property_read_string(chan->of_node, "discontinuous_clk",
			&str);
		if (!err)
			discontinuous_clk = !strncmp(str, "yes", sizeof("yes"));
		else
			dev_dbg(dev,
				"no discontinuous_clk in of_node");
		dev_dbg(dev, "discontinuous_clk = %u from of_node\n", discontinuous_clk);
	}

	return discontinuous_clk;
}

u32 read_phy_mode_from_dt(struct tegra_csi_channel *chan)
{
	struct camera_common_data *s_data = chan->s_data;
	struct sensor_mode_properties *mode = read_mode_from_dt(s_data);
	struct device *dev = chan->csi->dev;
	u32 phy_mode = 0;

	if (mode) {
		dev_dbg(dev, "settle time reading from props\n");
		phy_mode = mode->signal_properties.phy_mode;
	} else {
		dev_dbg(dev, "phy mode unavailable in props, use default\n");
		phy_mode = CSI_PHY_MODE_DPHY;
	}

	return phy_mode;
}

u64 read_pixel_clk_from_dt(struct tegra_csi_channel *chan)
{
	struct sensor_signal_properties *sig_props;
	struct sensor_properties *props;
	u64 pix_clk = 0;
	int mode_idx;

	if (chan && chan->s_data) {
		mode_idx = chan->s_data->mode_prop_idx;
		props =  &chan->s_data->sensor_props;
		sig_props = &props->sensor_modes[mode_idx].signal_properties;
		if (sig_props->serdes_pixel_clock.val != 0ULL)
			pix_clk = sig_props->serdes_pixel_clock.val;
		else
			pix_clk = sig_props->pixel_clock.val;
	}

	return pix_clk;
}

void set_csi_portinfo(struct tegra_csi_device *csi,
	unsigned int port, unsigned int numlanes)
{
	struct camera_common_data *s_data = &csi->s_data[port];

	s_data->csi_port = port;
	s_data->numlanes = numlanes;
	s_data->def_clk_freq = TEGRA_CLOCK_CSI_PORT_MAX;
}
EXPORT_SYMBOL(set_csi_portinfo);

static int tegra_csi_power(struct tegra_csi_device *csi,
			struct tegra_csi_channel *chan, int enable)
{
	int err = 0;

	trace_csi_s_power("enable", enable);
	if (enable) {
		err = csi->fops->csi_power_on(csi);
		if (!err)
			atomic_inc(&csi->power_ref);
	} else {
		err = csi->fops->csi_power_off(csi);
		if (!err)
			atomic_dec(&csi->power_ref);
	}
	return err;
}
EXPORT_SYMBOL(tegra_csi_power);

static int tegra_csi_error_recovery(struct tegra_channel *chan,
	struct tegra_csi_device *csi, struct tegra_csi_channel *csi_chan)
{
	int err = 0;
	int i = 0;
	struct tegra_csi_port *port = &csi_chan->ports[i];
	const char *rec_err_msg =
		"%s: failed to recover pt %u st %u vc %u (pg_mode %d)\n";

	for (i = 0; i < chan->valid_ports; i++) {
		err = csi->fops->csi_error_recover(csi_chan, i);
		if (err) {
			dev_err(csi->dev, rec_err_msg, __func__,
				port->csi_port, port->stream_id,
				port->virtual_channel_id, chan->pg_mode);
			break;
		}
	}

	return err;
}
EXPORT_SYMBOL(tegra_csi_error_recovery);

static int tegra_csi_s_power(struct v4l2_subdev *subdev, int enable)
{
	int err = 0;
	struct tegra_csi_device *csi = to_csi(subdev);
	struct tegra_csi_channel *chan = to_csi_chan(subdev);

	err = tegra_csi_power(csi, chan, enable);

	return err;
}

static int tegra_csi_sync_event(struct v4l2_subdev *subdev,
	unsigned int sync_events)
{
	int err = 0;
	struct tegra_channel *chan = v4l2_get_subdev_hostdata(subdev);
	struct tegra_csi_device *csi = to_csi(subdev);
	struct tegra_csi_channel *csi_chan = to_csi_chan(subdev);

	if (sync_events & V4L2_SYNC_EVENT_SUBDEV_ERROR_RECOVER)
		err = tegra_csi_error_recovery(chan, csi, csi_chan);

	return err;
}

/*
 * -----------------------------------------------------------------------------
 * CSI Subdevice Video Operations
 * -----------------------------------------------------------------------------
 */

int tegra_csi_start_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;

	return csi->fops->csi_start_streaming(chan, port_idx);
}
EXPORT_SYMBOL(tegra_csi_start_streaming);

void tegra_csi_stop_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;

	csi->fops->csi_stop_streaming(chan, port_idx);
}
EXPORT_SYMBOL(tegra_csi_stop_streaming);

static int update_video_source(struct tegra_csi_device *csi, int on, int is_tpg)
{
	mutex_lock(&csi->source_update);
	if (!on) {
		if (is_tpg)
			csi->tpg_active--;
		else
			csi->sensor_active--;
		WARN_ON(csi->tpg_active < 0 || csi->sensor_active < 0);
		goto stream_okay;
	}
	if (is_tpg && csi->tpg_active >= 0 && !csi->sensor_active) {
		csi->tpg_active++;
		goto stream_okay;
	}
	if (!is_tpg && csi->sensor_active >= 0 && !csi->tpg_active) {
		csi->sensor_active++;
		goto stream_okay;
	}
	mutex_unlock(&csi->source_update);
	dev_err(csi->dev, "Request rejected for new %s stream\n",
		is_tpg ? "tpg" : "sensor");
	dev_err(csi->dev, "Active tpg streams %d, active sensor streams %d\n",
			csi->tpg_active, csi->sensor_active);
	return -EINVAL;
stream_okay:
	mutex_unlock(&csi->source_update);
	return 0;
}

static void deskew_setup(struct tegra_csi_channel *chan,
				struct nvcsi_deskew_context *deskew_ctx)
{
	struct sensor_signal_properties *sig_props;
	struct sensor_properties *props;
	int i;
	int mode_idx = -1;
	u64 pix_clk_hz = 0;
	u32 deskew_enable = 0;
	unsigned int csi_lane_start = 0;
	unsigned int csi_port, csi_lanes;

	if (chan->s_data == NULL)
		return;

	mode_idx = chan->s_data->mode_prop_idx;
	props =  &chan->s_data->sensor_props;
	sig_props = &props->sensor_modes[mode_idx].signal_properties;
	if (sig_props->serdes_pixel_clock.val != 0ULL)
		pix_clk_hz = sig_props->serdes_pixel_clock.val;
	else
		pix_clk_hz = sig_props->pixel_clock.val;
	deskew_enable = sig_props->deskew_initial_enable;

	if (pix_clk_hz >= CLK_HZ_FOR_DESKEW && deskew_enable) {
		csi_port = chan->ports[0].csi_port;
		csi_lanes = chan->ports[0].lanes;
		switch (csi_port) {
		case NVCSI_PORT_A:
			csi_lane_start = NVCSI_PHY_0_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_B:
			csi_lane_start = NVCSI_PHY_0_NVCSI_CIL_B_IO0;
			break;
		case NVCSI_PORT_C:
			csi_lane_start = NVCSI_PHY_1_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_D:
			csi_lane_start = NVCSI_PHY_1_NVCSI_CIL_B_IO0;
			break;
		case NVCSI_PORT_E:
			csi_lane_start = NVCSI_PHY_2_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_F:
			csi_lane_start = NVCSI_PHY_2_NVCSI_CIL_B_IO0;
			break;
		case NVCSI_PORT_G:
			csi_lane_start = NVCSI_PHY_3_NVCSI_CIL_A_IO0;
			break;
		case NVCSI_PORT_H:
			csi_lane_start = NVCSI_PHY_3_NVCSI_CIL_B_IO0;
			break;
		default:
			break;
		}
		deskew_ctx->deskew_lanes = 0;
		for (i = 0; i < csi_lanes; ++i)
			deskew_ctx->deskew_lanes |= csi_lane_start << i;
		nvcsi_deskew_setup(deskew_ctx);
	}

}

static int tegra_csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_csi_device *csi;
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct tegra_channel *tegra_chan = v4l2_get_subdev_hostdata(subdev);
	int i, ret = 0;

	if (atomic_read(&chan->is_streaming) == enable)
		return 0;
	trace_csi_s_stream("enable", enable);
	csi = to_csi(subdev);
	if (!csi)
		return -EINVAL;
	ret = update_video_source(csi, enable, chan->pg_mode);
	if (ret)
		return ret;

	/* if it is bypass and real sensor, return here
	 * else let tegra_csi_start_streaming handle it
	 * depending on bypass and pg_mode flags
	 */
	if (tegra_chan->bypass && !tegra_chan->pg_mode) {
		atomic_set(&chan->is_streaming, enable);
		return 0;
	}
	for (i = 0; i < tegra_chan->valid_ports; i++) {
		if (enable) {
				ret = tegra_csi_start_streaming(chan, i);
				if (ret)
					goto start_fail;
				if (!tegra_chan->bypass && !tegra_chan->pg_mode)
					deskew_setup(chan,
						tegra_chan->deskew_ctx);
		} else
			tegra_csi_stop_streaming(chan, i);
	}
	atomic_set(&chan->is_streaming, enable);
	return ret;
start_fail:
	update_video_source(csi, 0, chan->pg_mode);
	/* Reverse sequence to stop streaming on all valid_ports
	 * i is the current failing port, need to stop ports 0 ~ (i-1)
	 */
	for (i = i - 1; i >= 0; i--)
		tegra_csi_stop_streaming(chan, i);
	return ret;
}


/* Used to calculate the settling time based on the mipi and cil clocks */
unsigned int tegra_csi_ths_settling_time(
		struct tegra_csi_device *csi,
		const unsigned int csicil_clk_mhz,
		const unsigned int mipi_clk_mhz)
{
	unsigned int cil_settletime;

	cil_settletime = (115 * csicil_clk_mhz + 8000 * csicil_clk_mhz
		/ (2 * mipi_clk_mhz) - 5500) / 1000;
	return cil_settletime;
}
EXPORT_SYMBOL(tegra_csi_ths_settling_time);

unsigned int tegra_csi_clk_settling_time(
	struct tegra_csi_device *csi,
	const unsigned int csicil_clk_mhz)
{
	unsigned int clk_settletime;

	clk_settletime = ((95 + 300) * csicil_clk_mhz - 13000) / 2000;
	return clk_settletime;
}
EXPORT_SYMBOL(tegra_csi_clk_settling_time);

/*
 * Only use this subdevice media bus ops for test pattern generator,
 * because CSI device is an separated subdevice which has 6 source
 * pads to generate test pattern.
 */
static struct v4l2_mbus_framefmt tegra_csi_tpg_fmts[] = {
	{
		TEGRA_DEF_WIDTH,
		TEGRA_DEF_HEIGHT,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		V4L2_FIELD_NONE,
		V4L2_COLORSPACE_SRGB
	},
	{
		TEGRA_DEF_WIDTH,
		TEGRA_DEF_HEIGHT,
		MEDIA_BUS_FMT_RGB888_1X32_PADHI,
		V4L2_FIELD_NONE,
		V4L2_COLORSPACE_SRGB
	}

};

static struct v4l2_frmsize_discrete tegra_csi_tpg_sizes[] = {
	{1280, 720},
	{1920, 1080},
	{3840, 2160}
};

static int tegra_csi_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *fse)
{
	int i;
	struct tegra_csi_channel *chan = to_csi_chan(sd);

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

	if (fse->index >= ARRAY_SIZE(tegra_csi_tpg_sizes))
		return -EINVAL;
	fse->index = array_index_nospec(fse->index,
					ARRAY_SIZE(tegra_csi_tpg_sizes));

	for (i = 0; i < ARRAY_SIZE(tegra_csi_tpg_fmts); i++) {
		if (tegra_csi_tpg_fmts[i].code == fse->code)
			break;
	}
	if (i == ARRAY_SIZE(tegra_csi_tpg_fmts))
		return -EINVAL;

	fse->min_width = fse->max_width =
			tegra_csi_tpg_sizes[fse->index].width;
	fse->min_height = fse->max_height =
			tegra_csi_tpg_sizes[fse->index].height;
	return 0;
}

static int tegra_csi_get_fmtindex(struct tegra_csi_channel *chan,
				int width, int height, int pixel_format)
{
	int i;
	const struct tpg_frmfmt *tegra_csi_tpg_frmfmt =
						chan->csi->tpg_frmfmt_table;

	for (i = 0; i < chan->csi->tpg_frmfmt_table_size; i++) {
		if (tegra_csi_tpg_frmfmt[i].frmsize.width == width &&
		    tegra_csi_tpg_frmfmt[i].frmsize.height == height &&
		    tegra_csi_tpg_frmfmt[i].pixel_format == pixel_format)
			break;
	}

	if (i == chan->csi->tpg_frmfmt_table_size)
		return -EINVAL;

	return i;
}

static int tegra_csi_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int index;
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	const struct tegra_video_format *format;
	const struct tpg_frmfmt *tegra_csi_tpg_frmfmt =
						chan->csi->tpg_frmfmt_table;
	struct tegra_channel *vi_chan = v4l2_get_subdev_hostdata(sd);

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

	/* One resolution just one framerate */
	if (fie->index > 0)
		return -EINVAL;
	format = tegra_core_get_format_by_code(vi_chan, fie->code, 0);
	if (!format)
		return -EINVAL;
	index = tegra_csi_get_fmtindex(chan, fie->width, fie->height,
					format->fourcc);
	if (index < 0)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = tegra_csi_tpg_frmfmt[index].framerate;

	return 0;
}

static int tegra_csi_try_mbus_fmt(struct v4l2_subdev *sd,
				  struct v4l2_mbus_framefmt *mf)
{
	int i, j;
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	const struct v4l2_frmsize_discrete *sizes;

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

	for (i = 0; i < ARRAY_SIZE(tegra_csi_tpg_fmts); i++) {
		struct v4l2_mbus_framefmt *fmt = &tegra_csi_tpg_fmts[i];

		if (mf->code == fmt->code) {
			for (j = 0; j < ARRAY_SIZE(tegra_csi_tpg_sizes); j++) {
				sizes = &tegra_csi_tpg_sizes[j];
				if (mf->width == sizes->width &&
				    mf->height == sizes->height)
					return 0;
			}
		}
	}

	dev_info(chan->csi->dev, "use TEGRA_DEF_WIDTH x TEGRA_DEF_HEIGHT (1920x1080)\n");
	memcpy(mf, tegra_csi_tpg_fmts, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int tegra_csi_g_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	struct v4l2_mbus_framefmt *format = &chan->ports[0].format;

	if (!chan->pg_mode) {
		dev_err(chan->csi->dev, "CSI is not in TPG mode\n");
		return -EINVAL;
	}

	mutex_lock(&chan->format_lock);
	memcpy(fmt, format, sizeof(struct v4l2_mbus_framefmt));
	mutex_unlock(&chan->format_lock);

	return 0;
}

static int csi_is_power_on(struct tegra_csi_device *csi)
{
	return atomic_read(&csi->power_ref);
}
static int tegra_csi_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tegra_csi_device *csi = to_csi(sd);

	/* Set status to 0 if power is on
	 * Set status to 1 if power is off
	 */
	*status = !csi_is_power_on(csi);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static int tegra_csi_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct v4l2_mbus_framefmt *mbus_fmt = &fmt->format;
	int ret;

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;
	ret = tegra_csi_g_mbus_fmt(subdev, mbus_fmt);
	if (ret)
		return ret;

	return 0;
}

static int tegra_csi_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	int ret;
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct v4l2_mbus_framefmt *format = &fmt->format;
	const struct tegra_video_format *vf;
	struct tegra_channel *vi_chan = v4l2_get_subdev_hostdata(subdev);
	int index, i;

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

	ret = tegra_csi_try_mbus_fmt(subdev, format);
	if (ret)
		return ret;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	vf = tegra_core_get_format_by_code(vi_chan, format->code, 0);
	if (!vf) {
		dev_err(chan->csi->dev, "Fail to find tegra video fmt");
		mutex_unlock(&chan->format_lock);
		return -EINVAL;
	}
	index = tegra_csi_get_fmtindex(chan, format->width,
				format->height, vf->fourcc);
	if (index < 0) {
		dev_err(chan->csi->dev, "Fail to find matching fmt");
		return -EINVAL;
	}

	mutex_lock(&chan->format_lock);
	for (i = 0; i < vi_chan->valid_ports; i++) {
		memcpy(&chan->ports[i].format,
		       &fmt->format, sizeof(struct v4l2_mbus_framefmt));
		chan->ports[i].core_format = vf;
		update_blank_intervals(chan, i, index);
	}
	mutex_unlock(&chan->format_lock);

	return 0;
}

static int tegra_csi_g_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *vfi)
{
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	struct tegra_csi_port *port = &chan->ports[0];

	if (!port->framerate)
		return -EINVAL;

	vfi->interval.numerator = 1;
	vfi->interval.denominator = port->framerate;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_video_ops tegra_csi_video_ops = {
	.s_stream	= tegra_csi_s_stream,
	.g_input_status = tegra_csi_g_input_status,
	.g_frame_interval = tegra_csi_g_frame_interval,
};

static struct v4l2_subdev_pad_ops tegra_csi_pad_ops = {
	.get_fmt	= tegra_csi_get_format,
	.set_fmt	= tegra_csi_set_format,
	.enum_frame_size = tegra_csi_enum_framesizes,
	.enum_frame_interval = tegra_csi_enum_frameintervals,
};

static struct v4l2_subdev_core_ops tegra_csi_core_ops = {
	.s_power	= tegra_csi_s_power,
	.sync		= tegra_csi_sync_event,
};

static struct v4l2_subdev_ops tegra_csi_ops = {
	.core	= &tegra_csi_core_ops,
	.video  = &tegra_csi_video_ops,
	.pad	= &tegra_csi_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations tegra_csi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static int tegra_csi_get_port_info(struct tegra_csi_channel *chan,
				struct device_node *node, unsigned int index)
{
	struct device_node *ep = NULL;
	struct device_node *ports;
	struct device_node *port;
	struct device_node *chan_dt;

	int value = 0xFFFF;
	int ret = 0, i;

	memset(&chan->port[0], INVALID_CSI_PORT, TEGRA_CSI_BLOCKS);
	for_each_child_of_node(node, chan_dt) {
		if (!chan_dt->name || of_node_cmp(chan_dt->name, "channel"))
			continue;
		ret = of_property_read_u32(chan_dt, "reg", &value);
		if (ret < 0)
			return -EINVAL;
		chan->of_node = chan_dt;
		if (value == index)
			break;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	chan->subdev.fwnode = of_fwnode_handle(chan_dt);
#else
	chan->subdev.of_node = chan_dt;
#endif
	ports = of_get_child_by_name(chan_dt, "ports");
	if (ports == NULL)
		return -EINVAL;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;
		ret = of_property_read_u32(port, "reg", &value);
		if (ret < 0)
			continue;
		if (value != 0)
			continue;
		for_each_child_of_node(port, ep) {
			if (!ep->name || of_node_cmp(ep->name, "endpoint"))
				continue;
			ret = of_property_read_u32(ep, "port-index", &value);
			if (ret < 0)
				dev_err(chan->csi->dev, "No port index info\n");
			chan->port[0] = value;

			ret = of_property_read_u32(ep, "bus-width", &value);
			if (ret < 0)
				dev_err(chan->csi->dev, "No bus width info\n");
			chan->numlanes = value;
			if (value > 12) {
				dev_err(chan->csi->dev, "Invalid num lanes\n");
				return -EINVAL;
			}
			/*
			 * for numlanes greater than 4 multiple CSI bricks
			 * are needed to capture the image, the logic below
			 * checks for numlanes > 4 and add a new CSI brick
			 * as a valid port. Loops around the three CSI
			 * bricks to add as many ports necessary.
			 */
			value -= 4;
			for (i = 1; value > 0; i++, value -= 4) {
				int next_port = chan->port[i-1] + 2;

				next_port = (next_port % (NVCSI_PORT_H + 1));
				chan->port[i] = next_port;
			}
		}
	}

	for (i = 0; csi_port_is_valid(chan->port[i]); i++)
		chan->numports++;

	return 0;
}

int tegra_csi_init(struct tegra_csi_device *csi,
		struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	csi->dev = &pdev->dev;
	err = set_csi_properties(csi, pdev);
	if (err)
		return err;

	csi->iomem_base = pdata->aperture[0];
	csi->fops->hw_init(csi);
	return err;
}

static int tegra_csi_channel_init_one(struct tegra_csi_channel *chan)
{
	struct v4l2_subdev *sd;
	int numlanes = 0;
	struct tegra_csi_device *csi = chan->csi;
	int i, ret;
	const struct tegra_video_format *vf;

	mutex_init(&chan->format_lock);

	vf = tegra_core_get_default_format();
	if (vf == NULL) {
		dev_err(csi->dev, "Fail to find tegra video fmt");
		return -EINVAL;
	}

	atomic_set(&chan->is_streaming, 0);
	sd = &chan->subdev;
	/* Initialize V4L2 subdevice and media entity */
	v4l2_subdev_init(sd, &tegra_csi_ops);
	sd->dev = chan->csi->dev;
	v4l2_set_subdevdata(sd, csi);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &tegra_csi_media_ops;
	chan->ports = devm_kzalloc(csi->dev,
			chan->numports * sizeof(struct tegra_csi_port),
			GFP_KERNEL);
	if (!chan->ports)
		return -ENOMEM;

	/* Initialize the default format */
	for (i = 0; i < chan->numports; i++) {
		chan->ports[i].format.code = vf->vf_code;
		chan->ports[i].format.field = V4L2_FIELD_NONE;
		chan->ports[i].format.colorspace = V4L2_COLORSPACE_SRGB;
		chan->ports[i].format.width = TEGRA_DEF_WIDTH;
		chan->ports[i].format.height = TEGRA_DEF_HEIGHT;
		chan->ports[i].core_format = vf;
	}
	if (chan->pg_mode) {
		/* If CSI has 2 existing channels, chan->id will start
		 * from 2 for the first TPG channel, which uses PORT_A(0).
		 * To get the correct PORT number, subtract existing number of
		 * channels from chan->id.
		 * when virtual channel is used, tpg0->pp0/vc0, tpg1->pp1/vc0,
		 * tpg2->pp2/vc0, tpg3->pp3/vc0, tpg4->pp4/vc0, tpg5->pp5/vc0,
		 * tpg6->pp0/vc1, tpg7->pp1/vc1 etc.
		 * pp means pixel parser, correspond to port[0] below.
		 * tpg id correspond to chan->id
		 */
		chan->port[0] = (chan->id - csi->num_channels)
				% NUM_TPG_INSTANCE;
		WARN_ON(chan->port[0] > csi->num_tpg_channels);
		chan->ports[0].stream_id = chan->port[0];
		chan->ports[0].virtual_channel_id
			= (chan->id - csi->num_channels) / NUM_TPG_INSTANCE;
		chan->ports->lanes = 2;
		chan->pads = devm_kzalloc(csi->dev, sizeof(*chan->pads),
				GFP_KERNEL);
		if (!chan->pads)
			return -ENOMEM;
		chan->pads[0].flags = MEDIA_PAD_FL_SOURCE;
	} else {
		chan->pads = devm_kzalloc(csi->dev, 2 * sizeof(*chan->pads),
			GFP_KERNEL);
		if (!chan->pads)
			return -ENOMEM;
		chan->pads[0].flags = MEDIA_PAD_FL_SINK;
		chan->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	}
	snprintf(sd->name, sizeof(sd->name), "%s-%d",
			 chan->pg_mode ? "tpg" :
			 (strlen(csi->devname) == 0 ?
			  dev_name(csi->dev) : csi->devname),
			  (chan->id - csi->num_channels));
	/* Initialize media entity */
	ret = tegra_media_entity_init(&sd->entity, chan->pg_mode ? 1 : 2,
				chan->pads, true, false);
	if (ret < 0)
		return ret;

	for (i = 0; i < chan->numports; i++) {
		numlanes = chan->numlanes - (i * MAX_CSI_BLOCK_LANES);
		WARN_ON(numlanes < 0);
		numlanes = numlanes > MAX_CSI_BLOCK_LANES ?
			MAX_CSI_BLOCK_LANES : numlanes;
		chan->ports[i].lanes = numlanes;

		if (!chan->pg_mode)
			chan->ports[i].csi_port = chan->port[i];
	}

	if (!chan->pg_mode) {
		ret = v4l2_async_register_subdev(sd);
		if (ret < 0) {
			dev_err(csi->dev, "failed to register subdev\n");
			media_entity_cleanup(&sd->entity);
		}
	}
	return ret;
}

static int tegra_csi_channels_init(struct tegra_csi_device *csi)
{
	int ret;
	struct tegra_csi_channel *it;

	list_for_each_entry(it, &csi->csi_chans, list) {
		ret = tegra_csi_channel_init_one(it);
		if (ret)
			return ret;
	}

	return 0;
}

static int csi_parse_dt(struct tegra_csi_device *csi,
			struct platform_device *pdev)
{
	int err = 0, i;
	int num_channels = 0, num_tpg_channels = 0;
	struct device_node *node = pdev->dev.of_node;
	struct tegra_csi_channel *item;

	if (strncmp(node->name, "nvcsi", 5)) {
		node = of_find_node_by_name(node, "nvcsi");
		strncpy(csi->devname, "nvcsi", 6);
	}

	if (node) {
		err = of_property_read_u32(node, "num-channels", &num_channels);
		if (err) {
			dev_dbg(csi->dev, " Failed to find num of channels, set to 0\n");
			num_channels = 0;
		}

		err = of_property_read_u32(node, "num-tpg-channels",
			&num_tpg_channels);
		/* Backward compatibility for T210 and T186.
		 * They both can generate 6 tpg streams, so use 6
		 * as default if DT entry is missing.
		 * For future chips, add this DT entry to
		 * create correct number of tpg video nodes
		 */
		if (err)
			num_tpg_channels = DEFAULT_NUM_TPG_CHANNELS;
	} else {
		num_channels = 0;
		num_tpg_channels = DEFAULT_NUM_TPG_CHANNELS;
	}

	csi->num_tpg_channels = num_tpg_channels;
	csi->num_channels = num_channels;
	for (i = 0; i < num_channels; i++) {
		item = devm_kzalloc(csi->dev, sizeof(*item), GFP_KERNEL);
		if (!item)
			return -ENOMEM;
		list_add_tail(&item->list, &csi->csi_chans);
		item->csi = csi;
		item->id = i;
		err = tegra_csi_get_port_info(item, node, item->id);
		if (err)
			return err;
	}

	return 0;
}

int tpg_csi_media_controller_init(struct tegra_csi_device *csi, int pg_mode)
{
	int i, err;
	struct tegra_csi_channel *item;

	if (!csi)
		return -EINVAL;
	for (i = 0; i < csi->num_tpg_channels; i++) {
		item = devm_kzalloc(csi->dev, sizeof(*item), GFP_KERNEL);
		if (!item) {
			err = -ENOMEM;
			goto channel_init_error;
		}
		if (i == 0)
			csi->tpg_start = item;
		list_add_tail(&item->list, &csi->csi_chans);
		item->numlanes = 2;
		item->numports = 1;
		item->csi = csi;
		item->pg_mode = pg_mode;
		item->id = csi->num_channels + i;
		err = tegra_csi_channel_init_one(item);
		if (err)
			goto channel_init_error;
	}
	csi->fops->hw_init(csi);
	csi->num_channels += csi->num_tpg_channels;

	return err;

channel_init_error:
	if (csi->tpg_start)
		tpg_csi_media_controller_cleanup(csi);
	dev_err(csi->dev, "%s: Error\n", __func__);
	return err;
}
EXPORT_SYMBOL(tpg_csi_media_controller_init);

void tpg_csi_media_controller_cleanup(struct tegra_csi_device *csi)
{
	struct tegra_csi_channel *item;
	struct tegra_csi_channel *itemn;
	struct v4l2_subdev *sd;

	list_for_each_entry_safe(item, itemn, &csi->csi_chans, list) {
		if (!item->pg_mode)
			continue;
		sd = &item->subdev;
		/* decrement media device entity count */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
		if (sd->entity.parent)
			sd->entity.parent->entity_id--;
#endif
		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
		list_del(&item->list);
		devm_kfree(csi->dev, item);
	}
	csi->num_channels -= csi->num_tpg_channels;
	csi->tpg_start = NULL;
}
EXPORT_SYMBOL(tpg_csi_media_controller_cleanup);

int tegra_csi_mipi_calibrate(struct tegra_csi_device *csi,
				bool on)
{
	struct tegra_csi_channel *chan;

	if (list_empty(&csi->csi_chans))
		return 0;

	if (!on) {
		tegra_mipi_bias_pad_disable();
		return 0;
	}

	tegra_mipi_bias_pad_enable();

	list_for_each_entry(chan, &csi->csi_chans, list) {
		int ret = 0;

		if (chan->pg_mode)
			continue;

		if (chan->s_data == NULL)
			continue;

		ret = csi->fops->mipical(chan);
		if (ret)
			dev_err(csi->dev,
				"calibration failed with %d error\n", ret);
	}

	return 0;
}

int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				    struct platform_device *pdev)
{
	int ret;

	if (!csi)
		return -EINVAL;
	mc_csi = csi;

	csi->dev = &pdev->dev;
	csi->pdev = pdev;
	csi->tpg_active = 0;
	csi->sensor_active = 0;
	atomic_set(&csi->power_ref, 0);
	mutex_init(&csi->source_update);
	INIT_LIST_HEAD(&csi->csi_chans);
	ret = csi_parse_dt(csi, pdev);
	if (ret < 0)
		return ret;

	/*
	 * if there is no csi channels listed in DT,
	 * no need to init the channel and graph
	 */
	if (csi->num_channels > 0) {
		ret = tegra_csi_channels_init(csi);
		if (ret < 0)
			dev_err(&pdev->dev, "Failed to init csi channel\n");
	}

	ret = tegra_csi_init(csi, pdev);
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to init csi property,clks\n");

	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_init);

int tegra_csi_media_controller_remove(struct tegra_csi_device *csi)
{
	struct tegra_csi_channel *chan;
	struct v4l2_subdev *sd;

	list_for_each_entry(chan, &csi->csi_chans, list) {
		sd = &chan->subdev;
		v4l2_async_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}
	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_remove);
