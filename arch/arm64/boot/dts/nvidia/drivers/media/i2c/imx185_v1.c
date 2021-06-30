/*
 * imx185.c - imx185 sensor driver
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#include "imx185_v1_mode_tbls.h"

#define IMX185_DEFAULT_MODE	IMX185_MODE_1920X1080_CROP_30FPS
#define IMX185_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB12_1X12

#define IMX185_MIN_FRAME_LENGTH	(1125)
#define IMX185_MAX_FRAME_LENGTH	(0x1FFFF)
#define IMX185_MIN_SHS1_1080P_HDR	(5)
#define IMX185_MIN_SHS2_1080P_HDR	(82)
#define IMX185_MAX_SHS2_1080P_HDR	(IMX185_MAX_FRAME_LENGTH - 5)
#define IMX185_MAX_SHS1_1080P_HDR	(IMX185_MAX_SHS2_1080P_HDR / 16)

#define IMX185_FRAME_LENGTH_ADDR_MSB		0x301A
#define IMX185_FRAME_LENGTH_ADDR_MID		0x3019
#define IMX185_FRAME_LENGTH_ADDR_LSB		0x3018
#define IMX185_COARSE_TIME_SHS1_ADDR_MSB	0x3022
#define IMX185_COARSE_TIME_SHS1_ADDR_MID	0x3021
#define IMX185_COARSE_TIME_SHS1_ADDR_LSB	0x3020
#define IMX185_COARSE_TIME_SHS2_ADDR_MSB	0x3025
#define IMX185_COARSE_TIME_SHS2_ADDR_MID	0x3024
#define IMX185_COARSE_TIME_SHS2_ADDR_LSB	0x3023
#define IMX185_GAIN_ADDR					0x3014
#define IMX185_GROUP_HOLD_ADDR				0x3001
#define IMX185_SW_RESET_ADDR			0x3003

#define IMX185_FUSE_ID_ADDR	0x3382
#define IMX185_FUSE_ID_SIZE	6
#define IMX185_FUSE_ID_STR_SIZE	(IMX185_FUSE_ID_SIZE * 2)
#define IMX185_DEFAULT_WIDTH	1920
#define IMX185_DEFAULT_HEIGHT	1080
#define IMX185_DEFAULT_CLK_FREQ	37125000

struct imx185 {
	struct camera_common_power_rail	power;
	int	numctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client	*i2c_client;
	struct v4l2_subdev	*subdev;
	struct media_pad	pad;
	u32				frame_length;
	s32	group_hold_prev;
	bool	group_hold_en;
	s64 last_wdr_et_val;
	struct regmap	*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static int imx185_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int imx185_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx185_ctrl_ops = {
	.g_volatile_ctrl = imx185_g_volatile_ctrl,
	.s_ctrl = imx185_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx185_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0 * FIXED_POINT_SCALING_FACTOR,
		.max = 48 * FIXED_POINT_SCALING_FACTOR,
		.def = 0 * FIXED_POINT_SCALING_FACTOR,
		.step = 3 * FIXED_POINT_SCALING_FACTOR / 10, /* 0.3 db */
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		.max = 1000000LL * FIXED_POINT_SCALING_FACTOR / 1000000,
		.def = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		.step = 1 * FIXED_POINT_SCALING_FACTOR / 1000000,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 1 * FIXED_POINT_SCALING_FACTOR,
		.max = 60 * FIXED_POINT_SCALING_FACTOR,
		.def = 30 * FIXED_POINT_SCALING_FACTOR,
		.step = 1 * FIXED_POINT_SCALING_FACTOR,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = TEGRA_CAMERA_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = IMX185_FUSE_ID_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name = "Sensor Mode",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = 0xFF,
		.def = 0xFE,
		.step = 1,
	},
};

static inline void imx185_get_frame_length_regs(imx185_reg *regs,
				u32 frame_length)
{
	regs->addr = IMX185_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 16) & 0x01;

	(regs + 1)->addr = IMX185_FRAME_LENGTH_ADDR_MID;
	(regs + 1)->val = (frame_length >> 8) & 0xff;

	(regs + 2)->addr = IMX185_FRAME_LENGTH_ADDR_LSB;
	(regs + 2)->val = (frame_length) & 0xff;
}

static inline void imx185_get_coarse_time_regs_shs1(imx185_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX185_COARSE_TIME_SHS1_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x01;

	(regs + 1)->addr = IMX185_COARSE_TIME_SHS1_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;

	(regs + 2)->addr = IMX185_COARSE_TIME_SHS1_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;

}

static inline void imx185_get_coarse_time_regs_shs2(imx185_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX185_COARSE_TIME_SHS2_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x01;

	(regs + 1)->addr = IMX185_COARSE_TIME_SHS2_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;

	(regs + 2)->addr = IMX185_COARSE_TIME_SHS2_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;

}

static inline void imx185_get_gain_reg(imx185_reg *regs,
				u8 gain)
{
	regs->addr = IMX185_GAIN_ADDR;
	regs->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx185_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx185_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct device *dev = &priv->i2c_client->dev;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx185_write_table(struct imx185 *priv,
				const imx185_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX185_TABLE_WAIT_MS,
					 IMX185_TABLE_END);
}

static int imx185_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	/*exit reset mode: XCLR */
	if (pw->reset_gpio) {
		gpio_set_value(pw->reset_gpio, 0);
		usleep_range(30, 50);
		gpio_set_value(pw->reset_gpio, 1);
		usleep_range(30, 50);
	}

	pw->state = SWITCH_ON;
	return 0;

}

static int imx185_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}
	/* enter reset mode: XCLR */
	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int imx185_power_get(struct imx185 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	struct device *dev = &priv->i2c_client->dev;
	const char *mclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "extperiph1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parent = devm_clk_get(dev, "pllp_grtba");
	if (IS_ERR(parent))
		dev_err(dev, "devm_clk_get failed for pllp_grtba");
	else
		clk_set_parent(pw->mclk, parent);

	pw->reset_gpio = pdata->reset_gpio;

	pw->state = SWITCH_OFF;
	return err;
}

static int imx185_set_coarse_time(struct imx185 *priv, s64 val);
static int imx185_set_coarse_time_hdr(struct imx185 *priv, s64 val);
static int imx185_set_gain(struct imx185 *priv, s64 val);
static int imx185_set_frame_rate(struct imx185 *priv, s64 val);
static int imx185_set_exposure(struct imx185 *priv, s64 val);

static int imx185_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct v4l2_ext_controls ctrls;
	struct v4l2_ext_control control[3];
	int err;

	dev_dbg(dev, "%s++ enable %d\n", __func__, enable);

	if (!enable) {
		err =  imx185_write_table(priv,
			mode_table[IMX185_MODE_STOP_STREAM]);

		if (err)
			return err;

		/* SW_RESET will have no ACK */
		regmap_write(priv->regmap, IMX185_SW_RESET_ADDR, 0x01);

		/* Wait for one frame to make sure sensor is set to
		 * software standby in V-blank
		 *
		 * delay = frame length rows * Tline (10 us)
		 */
		usleep_range(priv->frame_length * 10,
			priv->frame_length * 10 + 1000);
		return 0;
	}
	err = imx185_write_table(priv, mode_table[s_data->mode]);
	if (err)
		goto exit;

	if (s_data->override_enable) {
		/* write list of override regs for the asking gain, */
		/* frame rate and exposure time    */
		memset(&ctrls, 0, sizeof(ctrls));
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
		ctrls.which = V4L2_CTRL_ID2WHICH(TEGRA_CAMERA_CID_GAIN);
#else
		ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(TEGRA_CAMERA_CID_GAIN);
#endif
		ctrls.count = 3;
		ctrls.controls = control;

		control[0].id = TEGRA_CAMERA_CID_GAIN;
		control[1].id = TEGRA_CAMERA_CID_FRAME_RATE;
		control[2].id = TEGRA_CAMERA_CID_EXPOSURE;

		err = v4l2_g_ext_ctrls(&priv->ctrl_handler, &ctrls);
		if (err == 0) {
			err |= imx185_set_gain(priv, control[0].value64);
			if (err)
				dev_err(dev, "%s: error gain override\n",
					__func__);

			err |= imx185_set_frame_rate(priv, control[1].value64);
			if (err)
				dev_err(dev,
					"%s: error frame length override\n",
					__func__);

			err |= imx185_set_exposure(priv, control[2].value64);
			if (err)
				dev_err(dev, "%s: error exposure override\n",
					__func__);

		} else {
			dev_err(dev, "%s: faile to get overrides\n", __func__);
		}
	}

	if (test_mode) {
		err = imx185_write_table(priv,
			mode_table[IMX185_MODE_TEST_PATTERN]);
		if (err)
			goto exit;
	}

	err = imx185_write_table(priv, mode_table[IMX185_MODE_START_STREAM]);
	if (err)
		goto exit;

	return 0;
exit:
	dev_err(dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx185_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops imx185_subdev_video_ops = {
	.s_stream	= imx185_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = imx185_g_input_status,
};

static struct v4l2_subdev_core_ops imx185_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int imx185_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int imx185_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static struct v4l2_subdev_pad_ops imx185_subdev_pad_ops = {
	.set_fmt = imx185_set_fmt,
	.get_fmt = imx185_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops imx185_subdev_ops = {
	.core	= &imx185_subdev_core_ops,
	.video	= &imx185_subdev_video_ops,
	.pad = &imx185_subdev_pad_ops,
};

const static struct of_device_id imx185_of_match[] = {
	{ .compatible = "nvidia,imx185_v1",},
	{ },
};

static struct camera_common_sensor_ops imx185_common_ops = {
	.power_on = imx185_power_on,
	.power_off = imx185_power_off,
	.write_reg = imx185_write_reg,
	.read_reg = imx185_read_reg,
};

static int imx185_set_group_hold(struct imx185 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err;
	int gh_en = switch_ctrl_qmenu[val];

	priv->group_hold_prev = val;
	if (gh_en == SWITCH_ON) {

		err = imx185_write_reg(priv->s_data,
				       IMX185_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
	} else if (gh_en == SWITCH_OFF) {
		err = imx185_write_reg(priv->s_data,
				       IMX185_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
	}
	return 0;
fail:
	dev_dbg(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int imx185_set_gain(struct imx185 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	imx185_reg reg_list[1];
	int err;
	u8 gain;

	/* translate value */
	gain = (u8) (val * 160 / (48 * FIXED_POINT_SCALING_FACTOR));
	dev_dbg(dev, "%s:  gain reg: %d\n",  __func__, gain);

	imx185_get_gain_reg(reg_list, gain);

	err = imx185_write_reg(priv->s_data, reg_list[0].addr,
		 reg_list[0].val);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int imx185_set_frame_rate(struct imx185 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	imx185_reg reg_list[3];
	int err;
	u32 frame_length;
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	struct v4l2_control control;
	int hdr_en;
	int i = 0;

	frame_length = mode->signal_properties.pixel_clock.val *
		FIXED_POINT_SCALING_FACTOR /
		mode->image_properties.line_length / val;

	priv->frame_length = frame_length;
	if (priv->frame_length > IMX185_MAX_FRAME_LENGTH)
		priv->frame_length = IMX185_MAX_FRAME_LENGTH;

	dev_dbg(dev, "%s: val: %lld, , frame_length: %d\n", __func__,
		val, priv->frame_length);

	imx185_get_frame_length_regs(reg_list, priv->frame_length);

	for (i = 0; i < 3; i++) {
		err = imx185_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	/* check hdr enable ctrl */
	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if ((hdr_en == SWITCH_ON) && (priv->last_wdr_et_val != 0)) {
		err = imx185_set_coarse_time_hdr(priv, priv->last_wdr_et_val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time SHS1 SHS2 override\n", __func__);
	}

	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx185_set_exposure(struct imx185 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err;
	struct v4l2_control control;
	int hdr_en;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	/* check hdr enable ctrl */
	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if (hdr_en == SWITCH_ON) {
		err = imx185_set_coarse_time_hdr(priv, val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time SHS1 SHS2 override\n", __func__);
	} else {
		err = imx185_set_coarse_time(priv, val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time SHS1 override\n", __func__);
	}
	return err;
}

static int imx185_set_coarse_time(struct imx185 *priv, s64 val)
{
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	struct device *dev = &priv->i2c_client->dev;
	imx185_reg reg_list[3];
	int err;
	u32 coarse_time_shs1;
	u32 reg_shs1;
	int i = 0;

	coarse_time_shs1 = mode->signal_properties.pixel_clock.val *
		val / mode->image_properties.line_length /
		FIXED_POINT_SCALING_FACTOR;

	if (priv->frame_length == 0)
		priv->frame_length = IMX185_MIN_FRAME_LENGTH;

	reg_shs1 = priv->frame_length - coarse_time_shs1 - 1;

	dev_dbg(dev, "%s: coarse1:%d, shs1:%d, FL:%d\n", __func__,
		 coarse_time_shs1, reg_shs1, priv->frame_length);

	imx185_get_coarse_time_regs_shs1(reg_list, reg_shs1);

	for (i = 0; i < 3; i++) {
		err = imx185_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: set coarse time error\n", __func__);
	return err;
}

static int imx185_set_coarse_time_hdr(struct imx185 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	imx185_reg reg_list_shs1[3];
	imx185_reg reg_list_shs2[3];
	u32 coarse_time_shs1;
	u32 coarse_time_shs2;
	u32 reg_shs1;
	u32 reg_shs2;
	int err;
	int i = 0;

	if (priv->frame_length == 0)
		priv->frame_length = IMX185_MIN_FRAME_LENGTH;

	priv->last_wdr_et_val = val;

	/*WDR, update SHS1 as short ET, and SHS2 is 16x of short*/
	coarse_time_shs1 = mode->signal_properties.pixel_clock.val *
		val / mode->image_properties.line_length /
		FIXED_POINT_SCALING_FACTOR / 16;
	if (coarse_time_shs1 < IMX185_MIN_SHS1_1080P_HDR)
		coarse_time_shs1 = IMX185_MIN_SHS1_1080P_HDR;
	if (coarse_time_shs1 > IMX185_MAX_SHS1_1080P_HDR)
		coarse_time_shs1 = IMX185_MAX_SHS1_1080P_HDR;

	coarse_time_shs2 = (coarse_time_shs1 - IMX185_MIN_SHS1_1080P_HDR) * 16 +
				IMX185_MIN_SHS2_1080P_HDR;

	reg_shs1 = priv->frame_length - coarse_time_shs1 - 1;
	reg_shs2 = priv->frame_length - coarse_time_shs2 - 1;

	imx185_get_coarse_time_regs_shs1(reg_list_shs1, reg_shs1);
	imx185_get_coarse_time_regs_shs2(reg_list_shs2, reg_shs2);

	dev_dbg(dev, "%s: coarse1:%d, shs1:%d, coarse2:%d, shs2: %d, FL:%d\n",
		__func__,
		 coarse_time_shs1, reg_shs1,
		 coarse_time_shs2, reg_shs2,
		 priv->frame_length);

	for (i = 0; i < 3; i++) {
		err = imx185_write_reg(priv->s_data, reg_list_shs1[i].addr,
			 reg_list_shs1[i].val);
		if (err)
			goto fail;

		err = imx185_write_reg(priv->s_data, reg_list_shs2[i].addr,
			 reg_list_shs2[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: set WDR coarse time error\n", __func__);
	return err;
}

static int imx185_fuse_id_setup(struct imx185 *priv)
{
	int err;
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(priv->subdev);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct v4l2_ctrl *ctrl;
	u8 fuse_id[IMX185_FUSE_ID_SIZE];
	u8 bak = 0;

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;

	for (i = 0; i < IMX185_FUSE_ID_SIZE; i++) {
		err |= imx185_read_reg(s_data,
			IMX185_FUSE_ID_ADDR + i, &bak);
		if (!err)
			fuse_id[i] = bak;
		else {
			dev_err(dev, "%s: can not read fuse id\n", __func__);
			return -EINVAL;
		}
	}

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, TEGRA_CAMERA_CID_FUSE_ID);
	if (!ctrl) {
		dev_err(dev, "could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < IMX185_FUSE_ID_SIZE; i++)
		sprintf(&ctrl->p_new.p_char[i*2], "%02x",
			fuse_id[i]);
	ctrl->p_cur.p_char = ctrl->p_new.p_char;
	dev_info(dev, "%s, fuse id: %s\n", __func__, ctrl->p_cur.p_char);

	err = camera_common_s_power(priv->subdev, false);
	if (err)
		return -ENODEV;

	return 0;
}

static int imx185_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx185 *priv =
		container_of(ctrl->handler, struct imx185, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {

	default:
			dev_err(dev, "%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int imx185_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx185 *priv =
		container_of(ctrl->handler, struct imx185, ctrl_handler);
	struct camera_common_data	*s_data = priv->s_data;
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = imx185_set_gain(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		err = imx185_set_exposure(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		err = imx185_set_frame_rate(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		err = imx185_set_group_hold(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		s_data->sensor_mode_id = (int) (*ctrl->p_new.p_s64);
		break;
	default:
		dev_err(dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int imx185_ctrls_init(struct imx185 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->numctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	err = imx185_fuse_id_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"Error %d reading fuse id data\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, imx185_of_match);

static struct camera_common_pdata *imx185_parse_dt(struct i2c_client *client,
				struct camera_common_data *s_data)
{
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;
	int gpio;
	const char *str;

	if (!np)
		return NULL;

	match = of_match_device(imx185_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	err = of_property_read_string(np, "use_sensor_mode_id", &str);
	if (!err) {
		if (!strcmp(str, "true"))
			s_data->use_sensor_mode_id = true;
		else
			s_data->use_sensor_mode_id = false;
	}
	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(np, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(&client->dev, "mclk not in DT\n");

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		dev_err(&client->dev, "reset-gpios not found %d\n", gpio);
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	return board_priv_pdata;
}

static int imx185_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx185_subdev_internal_ops = {
	.open = imx185_open,
};

static const struct media_entity_operations imx185_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx185_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct imx185 *priv;
	int err;

	dev_info(&client->dev, "probing imx185_v1 v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx185) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	if (client->dev.of_node)
		priv->pdata = imx185_parse_dt(client, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops = &imx185_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &imx185_frmfmt[0];
	common_data->colorfmt = camera_common_find_datafmt(
					  IMX185_DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts = ARRAY_SIZE(imx185_frmfmt);
	common_data->def_mode = IMX185_DEFAULT_MODE;
	common_data->def_width = IMX185_DEFAULT_WIDTH;
	common_data->def_height = IMX185_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = IMX185_DEFAULT_CLK_FREQ;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	priv->last_wdr_et_val = 0;

	err = imx185_power_get(priv);
	if (err)
		return err;

	err = camera_common_initialize(common_data, "imx185");
	if (err) {
		dev_err(&client->dev, "Failed to initialize imx185.\n");
		return err;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &imx185_subdev_ops);

	err = imx185_ctrls_init(priv);
	if (err)
		return err;

	priv->subdev->internal_ops = &imx185_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &imx185_media_ops;
	err = tegra_media_entity_init(&priv->subdev->entity, 1,
				&priv->pad, true, true);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_info(&client->dev, "Detected IMX185 sensor\n");

	return 0;
}

static int
imx185_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx185 *priv = (struct imx185 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_cleanup(s_data);
	return 0;
}

static const struct i2c_device_id imx185_id[] = {
	{ "imx185_v1", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx185_id);

static struct i2c_driver imx185_i2c_driver = {
	.driver = {
		.name = "imx185_v1",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx185_of_match),
	},
	.probe = imx185_probe,
	.remove = imx185_remove,
	.id_table = imx185_id,
};

module_i2c_driver(imx185_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX185");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
