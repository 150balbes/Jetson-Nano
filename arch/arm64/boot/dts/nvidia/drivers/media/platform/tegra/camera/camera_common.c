/*
 * camera_common.c - utilities for tegra camera driver
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/types.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#include <media/mc_common.h>
#include <linux/of_graph.h>
#include <linux/string.h>
#include <soc/tegra/pmc.h>
#include <trace/events/camera_common.h>
#include <soc/tegra/chip-id.h>
#include <soc/tegra/tegra-i2c-rtcpu.h>

#include <asm/barrier.h>
#include <linux/nospec.h>

#define has_s_op(master, op) \
	(master->ops && master->ops->op)
#define call_s_op(master, op) \
	(has_s_op(master, op) ? \
	 master->ops->op(master) : 0)
#define call_s_ops(master, op, ...) \
	(has_s_op(master, op) ? \
	 master->ops->op(master, __VA_ARGS__) : 0)

#define HDR_ENABLE		0x1

static const struct camera_common_colorfmt camera_common_color_fmts[] = {
	{
		MEDIA_BUS_FMT_SRGGB12_1X12,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_SRGGB12,
	},
	{
		MEDIA_BUS_FMT_SGRBG12_1X12,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_SGRBG12,
	},
	{
		MEDIA_BUS_FMT_SRGGB10_1X10,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_SRGGB10,
	},
	{
		MEDIA_BUS_FMT_SGRBG10_1X10,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_SGRBG10,
	},
	{
		MEDIA_BUS_FMT_SBGGR10_1X10,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_SBGGR10,
	},
	{
		MEDIA_BUS_FMT_SRGGB8_1X8,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_SRGGB8,
	},
	{
		MEDIA_BUS_FMT_YUYV8_1X16,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_YUYV,
	},
	{
		MEDIA_BUS_FMT_YVYU8_1X16,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_YVYU,
	},
	{
		MEDIA_BUS_FMT_UYVY8_1X16,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_UYVY,
	},
	{
		MEDIA_BUS_FMT_VYUY8_1X16,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_VYUY,
	},
	{
		MEDIA_BUS_FMT_YUYV8_2X8,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_YUYV,
	},
	{
		MEDIA_BUS_FMT_YVYU8_2X8,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_YVYU,
	},
	{
		MEDIA_BUS_FMT_UYVY8_2X8,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_UYVY,
	},
	{
		MEDIA_BUS_FMT_VYUY8_2X8,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_VYUY,
	},
	/*
	 * The below two formats are not supported by VI4,
	 * keep them at the last to ensure they get discarded
	 */
	{
		MEDIA_BUS_FMT_XRGGB10P_3X10,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_XRGGB10P,
	},
	{
		MEDIA_BUS_FMT_XBGGR10P_3X10,
		V4L2_COLORSPACE_SRGB,
		V4L2_PIX_FMT_XRGGB10P,
	},
};

struct camera_common_csi_io_pad_ctx {
	const char *name;
	atomic_t ref;
};

static struct camera_common_csi_io_pad_ctx camera_common_csi_io_pads[] = {
	{"csia", ATOMIC_INIT(0)},
	{"csib", ATOMIC_INIT(0)},
	{"csic", ATOMIC_INIT(0)},
	{"csid", ATOMIC_INIT(0)},
	{"csie", ATOMIC_INIT(0)},
	{"csif", ATOMIC_INIT(0)},
	{"csig", ATOMIC_INIT(0)},
	{"csih", ATOMIC_INIT(0)},
};

static bool camera_common_verify_code(
	struct tegra_channel *chan, unsigned int code)
{
	int i;

	for (i = 0; i < chan->num_video_formats; i++) {
		if (chan->video_formats[i]->code == code)
			return true;
	}

	return false;
}

int camera_common_g_ctrl(struct camera_common_data *s_data,
			 struct v4l2_control *control)
{
	int i;

	for (i = 0; i < s_data->numctrls; i++) {
		if (s_data->ctrls[i]->id == control->id) {
			control->value = s_data->ctrls[i]->val;
			dev_dbg(s_data->dev,
				 "%s: found control %s\n", __func__,
				 s_data->ctrls[i]->name);
			return 0;
		}
	}

	return -EFAULT;
}
EXPORT_SYMBOL_GPL(camera_common_g_ctrl);

int camera_common_regulator_get(struct device *dev,
		       struct regulator **vreg, const char *vreg_name)
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = devm_regulator_get(dev, vreg_name);
	if (unlikely(IS_ERR(reg))) {
		dev_err(dev, "%s %s ERR: %p\n",
			__func__, vreg_name, reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}
EXPORT_SYMBOL_GPL(camera_common_regulator_get);

int camera_common_parse_clocks(struct device *dev,
			struct camera_common_pdata *pdata)
{
	struct device_node *np = dev->of_node;
	const char *prop;
	int proplen = 0;
	int i = 0;
	int numclocks = 0;
	int mclk_index = 0;
	int parentclk_index = -1;
	int err = 0;


	pdata->mclk_name = NULL;
	pdata->parentclk_name = NULL;
	err = of_property_read_string(np, "mclk", &pdata->mclk_name);
	if (!err) {
		dev_dbg(dev, "mclk in DT %s\n", pdata->mclk_name);
		of_property_read_string(np, "parent-clk",
					      &pdata->parentclk_name);
		return 0;
	}

	prop = (const char *)of_get_property(np, "clock-names", &proplen);
	if (!prop)
		return -ENODATA;

	/* find length of clock-names string array */
	for (i = 0; i < proplen; i++) {
		if (prop[i] == '\0')
			numclocks++;
	}

	if (numclocks > 1) {
		err = of_property_read_u32(np, "mclk-index", &mclk_index);
		if (err) {
			dev_err(dev, "Failed to find mclk index\n");
			return err;
		}
		err = of_property_read_u32(np, "parent-clk-index",
					   &parentclk_index);
	}

	for (i = 0; i < numclocks; i++) {
		if (i == mclk_index) {
			pdata->mclk_name = prop;
			dev_dbg(dev, "%s: mclk_name is %s\n",
				 __func__, pdata->mclk_name);
		} else if (i == parentclk_index) {
			pdata->parentclk_name = prop;
			dev_dbg(dev, "%s: parentclk_name is %s\n",
				 __func__, pdata->parentclk_name);
		} else
			dev_dbg(dev, "%s: %s\n", __func__, prop);
		prop += strlen(prop) + 1;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_parse_clocks);

int camera_common_parse_ports(struct device *dev,
			      struct camera_common_data *s_data)
{
	struct device_node *node = dev->of_node;
	struct device_node *ep = NULL;
	struct device_node *next;
	int bus_width = 0;
	int err = 0;
	int port = 0;

	/* Parse all the remote entities and put them into the list */
	next = of_graph_get_next_endpoint(node, ep);
	if (!next)
		return -ENODATA;

	of_node_put(ep);
	ep = next;

	err = of_property_read_u32(ep, "bus-width", &bus_width);
	if (err) {
		dev_err(dev,
			"Failed to find num of lanes\n");
		return err;
	}
	s_data->numlanes = bus_width;

	err = of_property_read_u32(ep, "port-index", &port);
	if (err) {
		dev_err(dev,
			"Failed to find port index\n");
		return err;
	}
	s_data->csi_port = port;

	dev_dbg(dev, "%s: port %d num of lanes %d\n",
		__func__, s_data->csi_port, s_data->numlanes);

	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_parse_ports);

int camera_common_parse_general_properties(struct device *dev,
			      struct camera_common_data *s_data)
{
	struct device_node *np = dev->of_node;
	int err = 0;
	const char *str;

	s_data->use_sensor_mode_id = false;
	err = of_property_read_string(np, "use_sensor_mode_id",	&str);
	if (!err) {
		if (!strcmp(str, "true"))
			s_data->use_sensor_mode_id = true;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_parse_general_properties);

int camera_common_debugfs_show(struct seq_file *s, void *unused)
{
	struct camera_common_data *s_data = s->private;

	dev_dbg(s_data->dev, "%s: ++\n", __func__);

	return 0;
}

ssize_t camera_common_debugfs_write(
	struct file *file,
	char const __user *buf,
	size_t count,
	loff_t *offset)
{
	struct camera_common_data *s_data =
		((struct seq_file *)file->private_data)->private;
	struct device *dev = s_data->dev;
	int err = 0;
	char buffer[MAX_BUFFER_SIZE];
	u32 address;
	u32 data;
	u8 readback = 0;

	dev_dbg(dev, "%s: ++\n", __func__);

	if (copy_from_user(&buffer, buf, sizeof(buffer)))
		goto debugfs_write_fail;

	if (sscanf(buffer, "0x%x 0x%x", &address, &data) == 2)
		goto set_attr;
	if (sscanf(buffer, "0X%x 0X%x", &address, &data) == 2)
		goto set_attr;
	if (sscanf(buffer, "%d %d", &address, &data) == 2)
		goto set_attr;

	if (sscanf(buffer, "0x%x 0x%x", &address, &data) == 1)
		goto read;
	if (sscanf(buffer, "0X%x 0X%x", &address, &data) == 1)
		goto read;
	if (sscanf(buffer, "%d %d", &address, &data) == 1)
		goto read;

	dev_err(dev, "SYNTAX ERROR: %s\n", buf);
	return -EFAULT;

set_attr:
	dev_dbg(dev,
			"new address = %x, data = %x\n", address, data);
	err |= call_s_ops(s_data, write_reg, address, data);
read:
	err |= call_s_ops(s_data, read_reg, address, &readback);
	dev_dbg(dev,
			"wrote to address 0x%x with value 0x%x\n",
			address, readback);

	if (err)
		goto debugfs_write_fail;

	return count;

debugfs_write_fail:
	dev_err(dev,
			"%s: test pattern write failed\n", __func__);
	return -EFAULT;
}

int camera_common_debugfs_open(struct inode *inode, struct file *file)
{
	struct camera_common_data *s_data = inode->i_private;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: ++\n", __func__);

	return single_open(file, camera_common_debugfs_show, inode->i_private);
}

static const struct file_operations camera_common_debugfs_fops = {
	.open		= camera_common_debugfs_open,
	.read		= seq_read,
	.write		= camera_common_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void camera_common_remove_debugfs(
		struct camera_common_data *s_data)
{
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: ++\n", __func__);

	debugfs_remove_recursive(s_data->debugdir);
	s_data->debugdir = NULL;
}
EXPORT_SYMBOL_GPL(camera_common_remove_debugfs);

void camera_common_create_debugfs(
		struct camera_common_data *s_data,
		const char *name)
{
	struct dentry *err;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s %s\n", __func__, name);

	s_data->debugdir =
		debugfs_create_dir(name, NULL);
	if (!s_data->debugdir)
		goto remove_debugfs;

	err = debugfs_create_file("d",
				S_IWUSR | S_IRUGO,
				s_data->debugdir, s_data,
				&camera_common_debugfs_fops);
	if (!err)
		goto remove_debugfs;

	return;
remove_debugfs:
	dev_err(dev, "couldn't create debugfs\n");
	camera_common_remove_debugfs(s_data);
}
EXPORT_SYMBOL_GPL(camera_common_create_debugfs);

/* Find a data format by a pixel code in an array */
const struct camera_common_colorfmt *camera_common_find_datafmt(
		unsigned int code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(camera_common_color_fmts); i++)
		if (camera_common_color_fmts[i].code == code)
			return camera_common_color_fmts + i;

	return NULL;
}
EXPORT_SYMBOL_GPL(camera_common_find_datafmt);

/* Find a data format by pixel format in an array*/
const struct camera_common_colorfmt *camera_common_find_pixelfmt(
		unsigned int pix_fmt)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(camera_common_color_fmts); i++)
		if (camera_common_color_fmts[i].pix_fmt == pix_fmt)
			return camera_common_color_fmts + i;

	return NULL;
}
EXPORT_SYMBOL_GPL(camera_common_find_pixelfmt);

/* Filters for the sensor's supported colors */
static const struct camera_common_colorfmt *find_matching_color_fmt(
		const struct camera_common_data *s_data,
		size_t index)
{
	const struct sensor_properties *sensor_props = &s_data->sensor_props;
	const size_t num_modes = sensor_props->num_modes;
	const size_t common_fmts_size = ARRAY_SIZE(camera_common_color_fmts);

	struct sensor_image_properties *cur_props;
	bool matched[ARRAY_SIZE(camera_common_color_fmts)];
	int match_num = -1;
	int match_index = -1;
	size_t i, j;

	// Clear matched array so no format has been matched
	memset(matched, 0, sizeof(matched));

	// Find and count matching color formats
	for (i = 0; i < common_fmts_size; i++) {
		for (j = 0; j < num_modes; j++) {
			cur_props = &sensor_props->sensor_modes[j].
							image_properties;
			if (cur_props->pixel_format ==
					camera_common_color_fmts[i].pix_fmt &&
					!matched[i]) {
				match_num++;
				match_index = i;
				// Found index
				if (match_num == index)
					goto break_loops;
			}
		}
	}
break_loops:
	if (match_num < index)
		return NULL;
	return &camera_common_color_fmts[match_index];
}

int camera_common_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	struct tegra_channel *chan = v4l2_get_subdev_hostdata(sd);
	const struct camera_common_colorfmt *sensor_fmt;

	sensor_fmt = find_matching_color_fmt(s_data, code->index);

	if (sensor_fmt == NULL)
		return -EINVAL;

	if (!camera_common_verify_code(chan, sensor_fmt->code))
		return -EINVAL;

	code->code = sensor_fmt->code;
	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_enum_mbus_code);

int camera_common_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			unsigned int *code)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	const struct camera_common_colorfmt *sensor_fmt;

	sensor_fmt = find_matching_color_fmt(s_data, index);

	if (sensor_fmt == NULL)
		return -EINVAL;
	*code = sensor_fmt->code;
	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_enum_fmt);

static void select_mode(struct camera_common_data *s_data,
			struct v4l2_mbus_framefmt *mf,
			unsigned int mode_type)
{
	int i;
	const struct camera_common_frmfmt *frmfmt = s_data->frmfmt;
	bool flag = 0;

	for (i = 0; i < s_data->numfmts; i++) {
		if (mode_type & HDR_ENABLE)
			flag = !frmfmt[i].hdr_en;
		/* Add more flags for different controls as needed */

		if (flag)
			continue;

		if (mf->width == frmfmt[i].size.width &&
			mf->height == frmfmt[i].size.height) {
			s_data->mode = frmfmt[i].mode;
			s_data->mode_prop_idx = i;
			break;
		}
	}
}

int camera_common_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	struct tegra_channel *chan = v4l2_get_subdev_hostdata(sd);
	struct v4l2_control hdr_control;
	const struct camera_common_frmfmt *frmfmt = s_data->frmfmt;
	unsigned int mode_type = 0;
	int err = 0;
	int i;

	dev_dbg(sd->dev, "%s: size %i x %i\n", __func__,
		 mf->width, mf->height);

	/* check hdr enable ctrl */
	hdr_control.id = TEGRA_CAMERA_CID_HDR_EN;

	/* mode_type can be filled in sensor driver */
	if (!(v4l2_g_ctrl(s_data->ctrl_handler, &hdr_control)))
		mode_type |=
			switch_ctrl_qmenu[hdr_control.value] ? HDR_ENABLE : 0;

	s_data->mode = s_data->def_mode;
	s_data->mode_prop_idx = 0;
	s_data->fmt_width = s_data->def_width;
	s_data->fmt_height = s_data->def_height;

	if (s_data->use_sensor_mode_id &&
		s_data->sensor_mode_id >= 0 &&
		s_data->sensor_mode_id < s_data->numfmts) {
		dev_dbg(sd->dev, "%s: use_sensor_mode_id %d\n",
				__func__, s_data->sensor_mode_id);
		s_data->mode = frmfmt[s_data->sensor_mode_id].mode;
		s_data->mode_prop_idx = s_data->sensor_mode_id;
		s_data->fmt_width = mf->width;
		s_data->fmt_height = mf->height;
	} else {
		/* select mode based on format match first */
		for (i = 0; i < s_data->numfmts; i++) {
			if (mf->width == frmfmt[i].size.width &&
				mf->height == frmfmt[i].size.height) {
				s_data->mode = frmfmt[i].mode;
				s_data->mode_prop_idx = i;
				s_data->fmt_width = mf->width;
				s_data->fmt_height = mf->height;
				break;
			}
		}

		if (i == s_data->numfmts) {
			mf->width = s_data->fmt_width;
			mf->height = s_data->fmt_height;
			dev_dbg(sd->dev,
				"%s: invalid resolution supplied to set mode %d %d\n",
				__func__, mf->width, mf->height);
			goto verify_code;
		}
		/* update mode based on special mode types */
		if (mode_type)
			select_mode(s_data, mf, mode_type);
	}

	if (!camera_common_verify_code(chan, mf->code))
		err = -EINVAL;

verify_code:
	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_SRGB;
	mf->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	mf->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mf->quantization = V4L2_QUANTIZATION_DEFAULT;

	return err;
}
EXPORT_SYMBOL_GPL(camera_common_try_fmt);

int camera_common_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	int ret;

	dev_dbg(sd->dev, "%s(%u) size %i x %i\n", __func__,
			mf->code, mf->width, mf->height);

	/* MIPI CSI could have changed the format, double-check */
	if (!camera_common_find_datafmt(mf->code))
		return -EINVAL;

	ret = camera_common_try_fmt(sd, mf);

	s_data->colorfmt = camera_common_find_datafmt(mf->code);

	return ret;
}
EXPORT_SYMBOL_GPL(camera_common_s_fmt);

int camera_common_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	const struct camera_common_colorfmt *fmt = s_data->colorfmt;

	dev_dbg(sd->dev, "%s++\n", __func__);

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= s_data->fmt_width;
	mf->height	= s_data->fmt_height;
	mf->field	= V4L2_FIELD_NONE;
	mf->xfer_func = fmt->xfer_func;
	mf->ycbcr_enc = fmt->ycbcr_enc;
	mf->quantization = fmt->quantization;

	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_g_fmt);

static int camera_common_evaluate_color_format(struct v4l2_subdev *sd,
					       int code)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	const size_t common_fmts_size = ARRAY_SIZE(camera_common_color_fmts);
	struct sensor_image_properties *cur_props;
	struct sensor_properties *sensor_props;
	size_t sensor_num_modes;
	int i, pixelformat;

	if (!s_data)
		return -EINVAL;

	sensor_props = &s_data->sensor_props;
	sensor_num_modes = sensor_props->num_modes;

	for (i = 0; i < common_fmts_size; i++) {
		if (camera_common_color_fmts[i].code == code)
			break;
	}

	if (i == common_fmts_size) {
		dev_dbg(s_data->dev,
			"%s: unsupported color format(%08x) for vi\n"
			, __func__, code);
		return -EINVAL;
	}

	pixelformat = camera_common_color_fmts[i].pix_fmt;

	for (i = 0; i < sensor_num_modes; i++) {
		cur_props = &sensor_props->sensor_modes[i].image_properties;
		if (cur_props->pixel_format == pixelformat)
			return 0;
	}

	if (i == sensor_num_modes) {
		dev_dbg(s_data->dev,
			"%s: unsupported color format(%08x) for sensor\n"
			, __func__, code);
		return -EINVAL;
	}

	return 0;
}

int camera_common_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *fse)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	int ret;

	if (!s_data || !s_data->frmfmt)
		return -EINVAL;

	if (fse->index >= s_data->numfmts)
		return -EINVAL;
	fse->index = array_index_nospec(fse->index, s_data->numfmts);

	ret = camera_common_evaluate_color_format(sd, fse->code);
	if (ret)
		return ret;

	fse->min_width = fse->max_width =
		s_data->frmfmt[fse->index].size.width;
	fse->min_height = fse->max_height =
		s_data->frmfmt[fse->index].size.height;
	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_enum_framesizes);

int camera_common_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	int i, ret;

	if (!s_data || !s_data->frmfmt)
		return -EINVAL;

	/* Check color format */
	ret = camera_common_evaluate_color_format(sd, fie->code);
	if (ret)
		return ret;

	/* Check resolution sizes */
	for (i = 0; i < s_data->numfmts; i++) {
		if (s_data->frmfmt[i].size.width == fie->width &&
		    s_data->frmfmt[i].size.height == fie->height)
			break;
	}
	if (i >= s_data->numfmts)
		return -EINVAL;

	/* Check index is in the rage of framerates array index */
	if (fie->index >= s_data->frmfmt[i].num_framerates)
		return -EINVAL;
	fie->index = array_index_nospec(fie->index,
					s_data->frmfmt[i].num_framerates);

	fie->interval.numerator = 1;
	fie->interval.denominator =
		s_data->frmfmt[i].framerates[fie->index];

	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_enum_frameintervals);

void camera_common_mclk_disable(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;

	if (!pw) {
		dev_err(s_data->dev, "%s: no device power rail\n",
			__func__);
		return;
	}

	dev_dbg(s_data->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(pw->mclk);
}
EXPORT_SYMBOL_GPL(camera_common_mclk_disable);

int camera_common_mclk_enable(struct camera_common_data *s_data)
{
	int err;
	struct camera_common_power_rail *pw = s_data->power;
	unsigned long mclk_init_rate = s_data->def_clk_freq;

	if (!pw) {
		dev_err(s_data->dev, "%s: no device power rail\n",
			__func__);
		return -ENODEV;
	}

	dev_dbg(s_data->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(pw->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(pw->mclk);

	return err;
}
EXPORT_SYMBOL_GPL(camera_common_mclk_enable);

void camera_common_dpd_disable(struct camera_common_data *s_data)
{
	int i;
	int io_idx;
	/* 2 lanes per port, divide by two to get numports */
	int numports = (s_data->numlanes + 1) >> 1;

	/* disable CSI IOs DPD mode to turn on camera */
	for (i = 0; i < numports; i++) {
		io_idx = s_data->csi_port + i;
		if (atomic_inc_return(
			&camera_common_csi_io_pads[io_idx].ref) == 1)
			tegra_pmc_io_pad_low_power_disable(
				camera_common_csi_io_pads[io_idx].name);
		dev_dbg(s_data->dev,
			 "%s: csi %d\n", __func__, io_idx);
	}
}

void camera_common_dpd_enable(struct camera_common_data *s_data)
{
	int i;
	int io_idx;
	/* 2 lanes per port, divide by two to get numports */
	int numports = (s_data->numlanes + 1) >> 1;

	/* disable CSI IOs DPD mode to turn on camera */
	for (i = 0; i < numports; i++) {
		io_idx = s_data->csi_port + i;
		if (atomic_dec_return(
			&camera_common_csi_io_pads[io_idx].ref) == 0)
			tegra_pmc_io_pad_low_power_enable(
				camera_common_csi_io_pads[io_idx].name);
		dev_dbg(s_data->dev,
			 "%s: csi %d\n", __func__, io_idx);
	}
}

int camera_common_s_power(struct v4l2_subdev *sd, int on)
{
	int err = 0;
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);

	trace_camera_common_s_power("status", on);
	if (on) {
		if (tegra_platform_is_silicon()) {
			err = camera_common_mclk_enable(s_data);
			if (err)
				return err;

			camera_common_dpd_disable(s_data);
		}
		err = call_s_op(s_data, power_on);
		if (err) {
			dev_err(s_data->dev,
				"%s: error power on\n", __func__);
			if (tegra_platform_is_silicon()) {
				camera_common_dpd_enable(s_data);
				camera_common_mclk_disable(s_data);
			}
		}
	} else {
		call_s_op(s_data, power_off);
		if (tegra_platform_is_silicon()) {
			camera_common_dpd_enable(s_data);
			camera_common_mclk_disable(s_data);
		}
	}

	return err;
}
EXPORT_SYMBOL_GPL(camera_common_s_power);

int camera_common_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_g_mbus_config);

int camera_common_get_framesync(struct v4l2_subdev *sd,
			struct camera_common_framesync *fs)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	int err = -ENOTSUPP;

	if (has_s_op(s_data, get_framesync))
		err = call_s_ops(s_data, get_framesync, fs);

	return err;
}
EXPORT_SYMBOL_GPL(camera_common_get_framesync);

int camera_common_focuser_s_power(struct v4l2_subdev *sd, int on)
{
	int err = 0;
	struct camera_common_focuser_data *s_data =
			to_camera_common_focuser_data(sd->dev);

	if (on) {
		err = call_s_op(s_data, power_on);
		if (err)
			dev_err(s_data->dev,
				"%s: error power on\n", __func__);
	} else
		err = call_s_op(s_data, power_off);

	return err;
}
EXPORT_SYMBOL_GPL(camera_common_focuser_s_power);

int camera_common_initialize(struct camera_common_data *s_data,
		const char *dev_name)
{
	int err = 0;
	char debugfs_name[10];

	if (s_data->dev == NULL)
		return -EINVAL;

	err = camera_common_parse_ports(s_data->dev, s_data);
	if (err) {
		dev_err(s_data->dev, "Failed to find port info.\n");
		return err;
	}

	err = camera_common_parse_general_properties(s_data->dev, s_data);
	if (err) {
		dev_err(s_data->dev, "Failed to find general properties.\n");
		return err;
	}

	err = sensor_common_init_sensor_properties(s_data->dev,
						s_data->dev->of_node,
						&s_data->sensor_props);
	if (err) {
		dev_err(s_data->dev,
			"Could not initialize sensor properties.\n");
		return err;
	}

	sprintf(debugfs_name, "%s_%c", dev_name, s_data->csi_port + 'a');
	dev_dbg(s_data->dev, "%s_probe: name %s\n", dev_name, debugfs_name);

	camera_common_create_debugfs(s_data, debugfs_name);

	return 0;
}
EXPORT_SYMBOL_GPL(camera_common_initialize);

void camera_common_cleanup(struct camera_common_data *s_data)
{
	camera_common_remove_debugfs(s_data);
}
EXPORT_SYMBOL_GPL(camera_common_cleanup);

int camera_common_focuser_init(struct camera_common_focuser_data *s_data)
{
	int err = 0;

	/* power on */
	err = call_s_op(s_data, power_on);
	if (err) {
		dev_err(s_data->dev,
			"%s: error power on\n", __func__);
		return err;
	}

	/* load default configuration */
	err = call_s_op(s_data, load_config);
	if (err) {
		dev_err(s_data->dev,
			"%s: error loading config\n", __func__);
		goto fail;
	}

	/* set controls */
	err = call_s_op(s_data, ctrls_init);
	if (err)
		dev_err(s_data->dev,
			"%s: error initializing controls\n", __func__);

fail:
	/* power off */
	err |= call_s_op(s_data, power_off);

	return err;
}
EXPORT_SYMBOL_GPL(camera_common_focuser_init);

/*
 * Regmap / RTCPU I2C driver interface
 */

int camera_common_i2c_init(
	struct camera_common_i2c *sensor,
	struct i2c_client *client,
	struct regmap_config *regmap_config,
	const struct tegra_i2c_rtcpu_config *rtcpu_config)
{
	sensor->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(sensor->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(sensor->regmap));
		return -ENODEV;
	}

	sensor->rt_sensor = tegra_i2c_rtcpu_register_sensor(
		client, rtcpu_config);

	return 0;
}
EXPORT_SYMBOL(camera_common_i2c_init);

int camera_common_i2c_aggregate(
	struct camera_common_i2c *sensor,
	bool start)
{
	if (sensor->rt_sensor)
		return tegra_i2c_rtcpu_aggregate(sensor->rt_sensor, start);

	return 0;
}
EXPORT_SYMBOL(camera_common_i2c_aggregate);

int camera_common_i2c_set_frame_id(
	struct camera_common_i2c *sensor,
	int frame_id)
{
	if (sensor->rt_sensor)
		return tegra_i2c_rtcpu_set_frame_id(
			sensor->rt_sensor, frame_id);

	return 0;
}
EXPORT_SYMBOL(camera_common_i2c_set_frame_id);

int camera_common_i2c_read_reg8(
	struct camera_common_i2c *sensor,
	unsigned int addr,
	u8 *data,
	unsigned int count)
{
	if (sensor->rt_sensor)
		return tegra_i2c_rtcpu_read_reg8(sensor->rt_sensor,
			addr, data, count);
	else
		return regmap_bulk_read(sensor->regmap, addr, data, count);
}
EXPORT_SYMBOL(camera_common_i2c_read_reg8);

int camera_common_i2c_write_reg8(
	struct camera_common_i2c *sensor,
	unsigned int addr,
	const u8 *data,
	unsigned int count)
{
	if (sensor->rt_sensor)
		return tegra_i2c_rtcpu_write_reg8(sensor->rt_sensor,
			addr, data, count);
	else
		return regmap_bulk_write(sensor->regmap, addr, data, count);
}
EXPORT_SYMBOL(camera_common_i2c_write_reg8);

int camera_common_i2c_write_table_8(
	struct camera_common_i2c *sensor,
	const struct reg_8 table[],
	const struct reg_8 override_list[],
	int num_override_regs, u16 wait_ms_addr, u16 end_addr)
{
	if (sensor->rt_sensor)
		return tegra_i2c_rtcpu_write_table_8(sensor->rt_sensor,
			table, override_list, num_override_regs,
			wait_ms_addr, end_addr);
	else
		return regmap_util_write_table_8(sensor->regmap,
			table, override_list, num_override_regs,
			wait_ms_addr, end_addr);
}
EXPORT_SYMBOL(camera_common_i2c_write_table_8);
