/*
 * ov10823.c - ov10823 sensor driver
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <media/ov10823.h>

#include "ov10823_mode_tbls.h"

#define OV10823_SC_CHIP_ID_HIGH_ADDR	0x300A
#define OV10823_SC_CHIP_ID_LOW_ADDR	0x300B
#define OV10823_SC_SCCB_ID_ADDR		0x300C

#define OV10823_MAX_COARSE_DIFF	8

#define OV10823_GAIN_SHIFT		8
#define OV10823_MIN_GAIN		(1 << OV10823_GAIN_SHIFT)
#define OV10823_MAX_GAIN \
	((15 << OV10823_GAIN_SHIFT) | (1 << (OV10823_GAIN_SHIFT - 1)))
#define OV10823_MIN_FRAME_LENGTH		(0x04F0)
#define OV10823_MAX_FRAME_LENGTH		(0x7FFF)
#define OV10823_MIN_EXPOSURE_COARSE		(0x8)
#define OV10823_MAX_EXPOSURE_COARSE	\
	(OV10823_MAX_FRAME_LENGTH-OV10823_MAX_COARSE_DIFF)

#define OV10823_DEFAULT_GAIN		OV10823_MIN_GAIN
#define OV10823_DEFAULT_FRAME_LENGTH		OV10823_MIN_FRAME_LENGTH
#define OV10823_DEFAULT_EXPOSURE_COARSE	\
	(OV10823_DEFAULT_FRAME_LENGTH-OV10823_MAX_COARSE_DIFF)

#define OV10823_DEFAULT_MODE		OV10823_MODE_2168X1220_60FPS
#define OV10823_DEFAULT_WIDTH		2168
#define OV10823_DEFAULT_HEIGHT		1220

#define OV10823_MAX_WIDTH		4336
#define OV10823_MAX_HEIGHT		2440

#define OV10823_DEFAULT_DATAFMT		MEDIA_BUS_FMT_SBGGR10_1X10
#define OV10823_DEFAULT_CLK_FREQ	26000000

#define OV10823_DEFAULT_I2C_ADDRESS_20	(0x20 >> 1)
#define OV10823_DEFAULT_I2C_ADDRESS_6C	(0x6C >> 1)

struct ov10823 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	int				fsync;
	int				cam_sid_gpio;
	int				mcu_boot_gpio;
	int				mcu_reset_gpio;
	bool				mirror;
	bool				flip;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static u16 ov10823_to_gain(u32 rep, int shift)
{
	u16 gain;
	int gain_int;
	int gain_dec;
	int min_int = (1 << shift);

	if (rep < OV10823_MIN_GAIN)
		rep = OV10823_MIN_GAIN;
	else if (rep > OV10823_MAX_GAIN)
		rep = OV10823_MAX_GAIN;

	/* shift indicates number of least significant bits */
	/* used for decimal representation of gain */
	gain_int = (int)(rep >> shift);
	gain_dec = (int)(rep & ~(0xffff << shift));

	/* derived from formulat gain = (x * 16 + 0.5) */
	gain = ((gain_int * min_int + gain_dec) * 32 + min_int) / (2 * min_int);

	return gain;
}

static int ov10823_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int ov10823_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops ov10823_ctrl_ops = {
	.g_volatile_ctrl = ov10823_g_volatile_ctrl,
	.s_ctrl		= ov10823_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &ov10823_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV10823_MIN_GAIN,
		.max = OV10823_MAX_GAIN,
		.def = OV10823_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &ov10823_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV10823_MIN_FRAME_LENGTH,
		.max = OV10823_MAX_FRAME_LENGTH,
		.def = OV10823_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &ov10823_ctrl_ops,
		.id = TEGRA_CAMERA_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV10823_MIN_EXPOSURE_COARSE,
		.max = OV10823_MAX_EXPOSURE_COARSE,
		.def = OV10823_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &ov10823_ctrl_ops,
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
		.ops = &ov10823_ctrl_ops,
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
		.ops = &ov10823_ctrl_ops,
		.id = TEGRA_CAMERA_CID_OTP_DATA,
		.name = "OTP Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV10823_OTP_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &ov10823_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV10823_FUSE_ID_STR_SIZE,
		.step = 2,
	},
};

static inline void ov10823_get_frame_length_regs(ov10823_reg *regs,
				u16 frame_length, int fsync)
{
	/* 2 registers for FL, i.e., 2-byte FL */
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
	if (fsync == OV10823_FSYNC_SLAVE) {
		(regs + 2)->addr = 0x3826;
		(regs + 2)->val = ((frame_length - 4) >> 8) & 0xff;
		(regs + 3)->addr = 0x3827;
		(regs + 3)->val = (frame_length - 4) & 0xff;
	} else {
		(regs + 2)->addr = 0x3830;
		(regs + 2)->val = ((frame_length - 4) >> 8) & 0xff;
		(regs + 3)->addr = 0x3831;
		(regs + 3)->val = (frame_length - 4) & 0xff;
	}
	(regs + 4)->addr = OV10823_TABLE_END;
	(regs + 4)->val = 0;
}

static inline void ov10823_get_coarse_time_regs(ov10823_reg *regs,
				u16 coarse_time)
{
	/* 3 registers for CT, i.e., 3-byte CT */
	regs->addr = 0x3500;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = 0x3501;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = 0x3502;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
	(regs + 3)->addr = OV10823_TABLE_END;
	(regs + 3)->val = 0;
}

static inline void ov10823_get_gain_reg(ov10823_reg *regs,
				u16 gain)
{
	/* 2 register for gain, i.e., 2-byte gain */
	regs->addr = 0x350a;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = 0x350b;
	(regs + 1)->val = (gain) & 0xff;
	(regs + 2)->addr = OV10823_TABLE_END;
	(regs + 2)->val = 0;
}

static inline int ov10823_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct ov10823 *priv = (struct ov10823 *)s_data->priv;
	unsigned int temp_val;
	int err;

	err = regmap_read(priv->regmap, addr, &temp_val);
	if (!err)
		*val = temp_val;

	return err;
}

static int ov10823_write_reg(struct camera_common_data *s_data,
		u16 addr, u8 val)
{
	int err;
	struct ov10823 *priv = (struct ov10823 *)s_data->priv;
	struct device *dev = &priv->i2c_client->dev;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int ov10823_write_table(struct ov10823 *priv,
				const ov10823_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 OV10823_TABLE_WAIT_MS,
					 OV10823_TABLE_END);
}

static int ov10823_i2c_addr_assign(struct ov10823 *priv, u8 i2c_addr)
{
	struct device *dev = &priv->i2c_client->dev;
	struct i2c_msg msg;
	unsigned char data[3];
	int err = 0;

	/*
	 * It seems that the way SID works for the OV10823 I2C slave address is
	 * that:
	 *
	 * SID 0 = 0x20
	 * SID 1 = 0x6c
	 *
	 * Address 0x20 is programmable via register 0x300c, and
	 * address 0x6c is programmable via register 0x3661.
	 *
	 * So, the scheme to assign addresses to an (almost) arbitrary
	 * number of sensors is to consider 0x20 to be the "off" address.
	 * Start each sensor with SID as 0 so that they appear to be off.
	 *
	 * Then, to assign an address to one sensor:
	 *
	 * 0. Set corresponding SID to 1 (now only that sensor responds
	 *    to 0x6c).
	 * 1. Use 0x6C to program address 0x20 to the new address.
	 * 2. Set corresponding SID back to 0 (so it no longer responds
	 *    to 0x6c, but instead responds to the new address).
	 */

	if (i2c_addr == OV10823_DEFAULT_I2C_ADDRESS_20) {
		dev_info(dev, "Using default I2C address 0x%02x\n", i2c_addr);
		if (gpio_is_valid(priv->cam_sid_gpio)) {
			gpio_set_value(priv->cam_sid_gpio, 0);
			msleep_range(1);
		}
		return 0;
	} else if (i2c_addr == OV10823_DEFAULT_I2C_ADDRESS_6C) {
		dev_info(dev, "Using default I2C address 0x%02x\n", i2c_addr);
		if (gpio_is_valid(priv->cam_sid_gpio)) {
			gpio_set_value(priv->cam_sid_gpio, 1);
			msleep_range(1);
		}
		return 0;
	}

	/*
	 * From this point on, we are trying to program the programmable
	 * slave address.  We necessarily need to have a cam-sid-gpio for this.
	 */
	if (!gpio_is_valid(priv->cam_sid_gpio)) {
		dev_err(dev, "Missing cam-sid-gpio, cannot program I2C addr\n");
		return -EINVAL;
	}

	gpio_set_value(priv->cam_sid_gpio, 1);
	msleep_range(1);

	/*
	 * Have to make the I2C message manually because we are using a
	 * different I2C slave address for this transaction, rather than
	 * the one in the device tree for this device.
	 */
	data[0] = (OV10823_SC_SCCB_ID_ADDR >> 8) & 0xff;
	data[1] = OV10823_SC_SCCB_ID_ADDR & 0xff;
	data[2] = ((i2c_addr) << 1) & 0xff;
	/*
	 * Use the programmable default I2C slave address so that if we have
	 * multiple sensors of this same kind, when we change one sensor's
	 * address, the next sensor address change message won't go to that
	 * same sensor.
	 */
	msg.addr = OV10823_DEFAULT_I2C_ADDRESS_6C;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	if (i2c_transfer(priv->i2c_client->adapter, &msg, 1) != 1) {
		dev_err(dev, "Error assigning I2C address to 0x%02x\n",
			i2c_addr);
		err = -EIO;
	}

	gpio_set_value(priv->cam_sid_gpio, 0);
	msleep_range(1);

	return err;
}

static int ov10823_power_on(struct camera_common_data *s_data)
{
	struct ov10823 *priv = (struct ov10823 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;
	int err;

	dev_dbg(dev, "%s: power on\n", __func__);

	if (priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto avdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto dvdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto iovdd_fail;
	}

	usleep_range(5350, 5360);

	err = ov10823_i2c_addr_assign(priv, priv->i2c_client->addr);
	if (err)
		goto addr_assign_fail;

	pw->state = SWITCH_ON;
	return 0;

addr_assign_fail:
	if (pw->iovdd)
		regulator_disable(pw->iovdd);

iovdd_fail:
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

dvdd_fail:
	if (pw->avdd)
		regulator_disable(pw->avdd);

avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int ov10823_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct ov10823 *priv = (struct ov10823 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;

	dev_dbg(dev, "%s: power off\n", __func__);
	ov10823_write_table(priv, mode_table[OV10823_MODE_STOP_STREAM]);

	if (priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			goto power_off_done;
	}

	if (pw->iovdd)
		regulator_disable(pw->iovdd);

	if (pw->dvdd)
		regulator_disable(pw->dvdd);

	if (pw->avdd)
		regulator_disable(pw->avdd);

	return err;

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int ov10823_power_put(struct ov10823 *priv)
{
	return 0;
}

static int ov10823_power_get(struct ov10823 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	struct device *dev = &priv->i2c_client->dev;
	const char *mclk_name;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	if (priv->pdata->regulators.avdd) {
		err = camera_common_regulator_get(dev,
			&pw->avdd, pdata->regulators.avdd);
		if (err) {
			dev_err(dev, "unable to get regulator %s, err = %d\n",
				pdata->regulators.avdd, err);
			goto done;
		}
	}

	if (priv->pdata->regulators.dvdd) {
		err = camera_common_regulator_get(dev,
			&pw->dvdd, pdata->regulators.dvdd);
		if (err) {
			dev_err(dev, "unable to get regulator %s, err = %d\n",
				pdata->regulators.dvdd, err);
			goto done;
		}
	}

	if (priv->pdata->regulators.iovdd) {
		err = camera_common_regulator_get(dev,
			&pw->iovdd, pdata->regulators.iovdd);
		if (err) {
			dev_err(dev, "unable to get regulator %s, err = %d\n",
				pdata->regulators.iovdd, err);
			goto done;
		}
	}

done:
	pw->state = SWITCH_OFF;
	return err;
}

static int ov10823_verify_chip_id(struct ov10823 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct camera_common_data *s_data = priv->s_data;
	u8 chip_id_hi, chip_id_lo;
	u16 chip_id;
	int err;

	err = ov10823_read_reg(s_data, OV10823_SC_CHIP_ID_HIGH_ADDR,
			       &chip_id_hi);
	if (err) {
		dev_err(&client->dev, "Failed to read chip ID\n");
		return err;
	}
	err = ov10823_read_reg(s_data, OV10823_SC_CHIP_ID_LOW_ADDR,
			       &chip_id_lo);
	if (err) {
		dev_err(&client->dev, "Failed to read chip ID\n");
		return err;
	}

	chip_id = (chip_id_hi << 8) | chip_id_lo;
	if (chip_id != 0xA820) {
		dev_err(&client->dev, "Read unknown chip ID 0x%04x\n", chip_id);
		return -EINVAL;
	}

	return 0;
}

static int ov10823_set_gain(struct ov10823 *priv, s32 val);
static int ov10823_set_frame_length(struct ov10823 *priv, s32 val);
static int ov10823_set_coarse_time(struct ov10823 *priv, s32 val);

static int ov10823_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov10823 *priv = (struct ov10823 *)s_data->priv;
	struct v4l2_control control;
	int err;

	if (!enable) {
		dev_dbg(&client->dev, "%s: stream off\n", __func__);
		return ov10823_write_table(priv,
			mode_table[OV10823_MODE_STOP_STREAM]);
		}

	dev_dbg(&client->dev, "%s: write mode table %d\n",
		__func__, s_data->mode);
	err = ov10823_write_table(priv, mode_table[s_data->mode]);

	if (fsync_table[priv->fsync]) {
		dev_dbg(&client->dev, "%s: write fsync table %d\n", __func__,
			priv->fsync);
		err = ov10823_write_table(priv, fsync_table[priv->fsync]);
		if (err)
			goto exit;
	}

	if ((priv->fsync == OV10823_FSYNC_SLAVE) &&
	    fsync_slave_mode_table[s_data->mode]) {
		dev_dbg(&client->dev, "%s: write fsync slave mode table %d\n",
			__func__, s_data->mode);
		err = ov10823_write_table(
			priv, fsync_slave_mode_table[s_data->mode]);
		if (err)
			goto exit;
	}

	if (s_data->override_enable) {
		/* write list of override regs for the asking frame length, */
		/* coarse integration time, and gain. Failures to write */
		/* overrides are non-fatal. */
		control.id = TEGRA_CAMERA_CID_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= ov10823_set_gain(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error gain override\n", __func__);

		control.id = TEGRA_CAMERA_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= ov10823_set_frame_length(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error frame length override\n", __func__);

		control.id = TEGRA_CAMERA_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= ov10823_set_coarse_time(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: error coarse time override\n", __func__);
	}

	/*
	 * Handle mirror and flip.
	 * Horizontal and vertical binning needs to be enabled for mirror and
	 * flip, respectively, so doing this operation is probably not ideal
	 * if the full resolution of the sensor is to be used.
	 */
	if (priv->mirror) {
		if (s_data->frmfmt->size.width > (OV10823_MAX_WIDTH / 2))
			ov10823_write_reg(s_data, 0x3821, 0x04);
		else
			ov10823_write_reg(s_data, 0x3821, 0x06);
	}


	if (priv->flip) {
		if (s_data->frmfmt->size.height > (OV10823_MAX_HEIGHT / 2))
			ov10823_write_reg(s_data, 0x3820, 0x04);
		else
			ov10823_write_reg(s_data, 0x3820, 0x06);
	}

	dev_dbg(&client->dev, "%s: stream on\n", __func__);
	err = ov10823_write_table(priv, mode_table[OV10823_MODE_START_STREAM]);
	if (err)
		goto exit;

	/*
	 * If the sensor is in fsync slave mode, and is in the middle of
	 * sending a frame when it gets a strobe on the fsin pin, it may
	 * prematurely end the frame, resulting in a short frame on our
	 * camera host.  So, after starting streaming, we assume fsync
	 * master has already been told to start streaming, and we wait some
	 * amount of time in order to skip the possible short frame.  The
	 * length of time to wait should be at least our sample period.
	 * Assume worse case of 30fps (33.3ms), and add a bit more.
	 */
	if (priv->fsync == OV10823_FSYNC_SLAVE)
		msleep(40);

	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int ov10823_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov10823 *priv = (struct ov10823 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops ov10823_subdev_video_ops = {
	.s_stream	= ov10823_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = ov10823_g_input_status,
};

static struct v4l2_subdev_core_ops ov10823_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int ov10823_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int ov10823_set_fmt(struct v4l2_subdev *sd,
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

static struct v4l2_subdev_pad_ops ov10823_subdev_pad_ops = {
	.set_fmt = ov10823_set_fmt,
	.get_fmt = ov10823_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops ov10823_subdev_ops = {
	.core	= &ov10823_subdev_core_ops,
	.video	= &ov10823_subdev_video_ops,
	.pad	= &ov10823_subdev_pad_ops,
};

static struct of_device_id ov10823_of_match[] = {
	{ .compatible = "nvidia,ov10823", },
	{ },
};

static struct camera_common_sensor_ops ov10823_common_ops = {
	.power_on = ov10823_power_on,
	.power_off = ov10823_power_off,
	.write_reg = ov10823_write_reg,
	.read_reg = ov10823_read_reg,
};

static int ov10823_set_group_hold(struct ov10823 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int err;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		/* group hold start */
		err = ov10823_write_reg(priv->s_data,
				       OV10823_GROUP_HOLD_ADDR, 0x00);
		if (err)
			goto fail;
		priv->group_hold_prev = 1;
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		/* group hold end */
		err = ov10823_write_reg(priv->s_data,
				       OV10823_GROUP_HOLD_ADDR, 0x10);
		/* quick launch */
		err |= ov10823_write_reg(priv->s_data,
				       OV10823_GROUP_HOLD_ADDR, 0xA0);
		if (err)
			goto fail;
		priv->group_hold_prev = 0;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int ov10823_set_gain(struct ov10823 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	ov10823_reg reg_list[3];
	int err;
	u16 gain;

	/* max_gain 15.5x ---> 0x350A=0x00, 0x350B=0xF8 */
	/* min_gain 1.0x  ---> 0x350A=0x00, 0x350B=0x10 */
	/* translate value */
	gain = ov10823_to_gain((u32)val, OV10823_GAIN_SHIFT);

	dev_dbg(dev, "%s: gain: %d\n", __func__, gain);

	ov10823_get_gain_reg(reg_list, gain);
	ov10823_set_group_hold(priv);
	err = ov10823_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int ov10823_set_frame_length(struct ov10823 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	ov10823_reg reg_list[5];
	int err;
	u16 frame_length;

	frame_length = (u16)val;

	dev_dbg(dev, "%s: frame_length: %d\n", __func__, frame_length);

	ov10823_get_frame_length_regs(reg_list, frame_length, priv->fsync);
	ov10823_set_group_hold(priv);
	err = ov10823_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int ov10823_set_coarse_time(struct ov10823 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	ov10823_reg reg_list[4];
	int err;
	u16 coarse_time;

	coarse_time = (u16)val;

	dev_dbg(dev, "%s: coarse_time: %d\n", __func__, coarse_time);

	ov10823_get_coarse_time_regs(reg_list, coarse_time);
	ov10823_set_group_hold(priv);
	err = ov10823_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(dev, "%s: COARSE_TIME control error\n", __func__);
	return err;
}

static int ov10823_read_otp(struct ov10823 *priv, u8 *buf,
		u16 addr, int size)
{
	int err;

	err = ov10823_write_reg(priv->s_data, OV10823_ISP_CTRL_ADDR, 0x00);
	if (err)
		return err;
	/* Start streaming before write or read */
	err = ov10823_write_reg(priv->s_data, 0x0100, 0x01);
	if (err)
		return err;
	msleep(20);

	/* By default otp loading works in auto mode, but we can switch to */
	/* manual mode through OV10823_OTP_MODE_CTRL_ADDR[6] and the start */
	/* addr and end addr of manual mode can be configured by registers */
	/* accordingly */

	/* Loading enable */
	/* 1: manual mode */
	/* 0: auto mode   */
	err = ov10823_write_reg(priv->s_data, OV10823_OTP_LOAD_CTRL_ADDR, 0x01);
	if (err)
		return err;

	msleep(20);
	err = regmap_bulk_read(priv->regmap, addr, buf, size);
	if (err)
		return err;

	return 0;
}

static int ov10823_otp_setup(struct ov10823 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int i;
	struct v4l2_ctrl *ctrl;
	u8 otp_buf[OV10823_OTP_SIZE];

	ov10823_read_otp(priv, &otp_buf[0],
				   OV10823_OTP_SRAM_START_ADDR,
				   OV10823_OTP_SIZE);

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, TEGRA_CAMERA_CID_OTP_DATA);
	if (!ctrl) {
		dev_err(dev, "could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < OV10823_OTP_SIZE; i++)
		sprintf(&ctrl->p_new.p_char[i*2], "%02x",
			otp_buf[i]);
	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static int ov10823_fuse_id_setup(struct ov10823 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int i;
	struct v4l2_ctrl *ctrl;
	u8 fuse_id[OV10823_FUSE_ID_SIZE];

	ov10823_read_otp(priv, &fuse_id[0],
				   OV10823_FUSE_ID_OTP_BASE_ADDR,
				   OV10823_FUSE_ID_SIZE);

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, TEGRA_CAMERA_CID_FUSE_ID);
	if (!ctrl) {
		dev_err(dev, "could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < OV10823_FUSE_ID_SIZE; i++)
		sprintf(&ctrl->p_new.p_char[i*2], "%02x",
			fuse_id[i]);
	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static int ov10823_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov10823 *priv =
		container_of(ctrl->handler, struct ov10823, ctrl_handler);
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

static int ov10823_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov10823 *priv =
		container_of(ctrl->handler, struct ov10823, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = ov10823_set_gain(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_FRAME_LENGTH:
		/*
		 * This is a workaround for nvbug 1865041, where setting the
		 * VTS timing registers when the sensor is set up for fsync
		 * master or slave leads to instability if streaming has
		 * already started.
		 */
		if (priv->fsync == OV10823_FSYNC_NONE)
			err = ov10823_set_frame_length(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_COARSE_TIME:
		err = ov10823_set_coarse_time(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = ov10823_set_group_hold(priv);
		}
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	default:
		dev_err(dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int ov10823_ctrls_init(struct ov10823 *priv)
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

	priv->num_ctrls = num_ctrls;
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

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, ov10823_of_match);

static int ov10823_parse_dt(struct i2c_client *client, struct ov10823 *priv)
{
	struct device_node *np = client->dev.of_node;
	const char *fsync_str;
	int gpio;
	int err;

	err = of_property_read_string(np, "mclk", &priv->pdata->mclk_name);
	if (err) {
		dev_err(&client->dev, "mclk not in DT\n");
		return -EINVAL;
	}

	err = of_property_read_string(np, "fsync", &fsync_str);
	if (!err && fsync_str && (strcmp(fsync_str, "master") == 0))
		priv->fsync = OV10823_FSYNC_MASTER;
	else if (!err && fsync_str && (strcmp(fsync_str, "slave") == 0))
		priv->fsync = OV10823_FSYNC_SLAVE;
	else
		priv->fsync = OV10823_FSYNC_NONE;

	gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (gpio < 0) {
		dev_dbg(&client->dev, "pwdn gpios not in DT\n");
		gpio = 0;
	}
	priv->pdata->pwdn_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		dev_dbg(&client->dev, "reset gpios not in DT\n");
		gpio = 0;
	}
	priv->pdata->reset_gpio = (unsigned int)gpio;

	priv->mcu_boot_gpio =
		of_get_named_gpio(np, "mcu-boot-gpios", 0);
	priv->mcu_reset_gpio =
		of_get_named_gpio(np, "mcu-reset-gpios", 0);

	priv->cam_sid_gpio = of_get_named_gpio(np, "cam-sid-gpios", 0);

	priv->mirror = of_property_read_bool(np, "mirror");
	priv->flip = of_property_read_bool(np, "flip");

	err = of_property_read_string(np, "avdd-reg",
				      &priv->pdata->regulators.avdd);
	if (err)
		dev_warn(&client->dev, "avdd-reg not in DT\n");

	err = of_property_read_string(np, "dvdd-reg",
				      &priv->pdata->regulators.dvdd);
	if (err)
		dev_warn(&client->dev, "dvdd-reg not in DT\n");

	err = of_property_read_string(np, "iovdd-reg",
				      &priv->pdata->regulators.iovdd);
	if (err)
		dev_warn(&client->dev, "iovdd-reg not in DT\n");

	return 0;
}

static int ov10823_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ov10823_subdev_internal_ops = {
	.open = ov10823_open,
};

static const struct media_entity_operations ov10823_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov10823_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct ov10823 *priv;
	int err;

	dev_info(&client->dev, "probing v4l2 sensor.\n");

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);

	priv = devm_kzalloc(&client->dev,
			sizeof(struct ov10823) + sizeof(struct v4l2_ctrl *) *
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

	priv->pdata = devm_kzalloc(&client->dev,
				   sizeof(struct camera_common_pdata),
				   GFP_KERNEL);
	if (!priv->pdata) {
		dev_err(&client->dev,
			"unable to allocate camera_common_pdata\n");
		return -ENOMEM;
	}

	err = ov10823_parse_dt(client, priv);
	if (err)
		return err;

	common_data->ops		= &ov10823_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->dev		= &client->dev;
	common_data->frmfmt		= ov10823_frmfmt;
	common_data->colorfmt		= camera_common_find_datafmt(
					  OV10823_DEFAULT_DATAFMT);
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(ov10823_frmfmt);
	common_data->def_mode		= OV10823_DEFAULT_MODE;
	common_data->def_width		= OV10823_DEFAULT_WIDTH;
	common_data->def_height		= OV10823_DEFAULT_HEIGHT;
	common_data->def_clk_freq	= OV10823_DEFAULT_CLK_FREQ;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;

	priv->i2c_client		= client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;
	priv->group_hold_prev		= 0;

	err = ov10823_power_get(priv);
	if (err)
		return err;

	/*
	 * If our device tree node is given MCU GPIOs, then we are expected to
	 * reset the MCU.
	 */
	if (gpio_is_valid(priv->mcu_boot_gpio) &&
	    gpio_is_valid(priv->mcu_reset_gpio)) {
		dev_info(&client->dev, "Resetting MCU\n");
		gpio_set_value(priv->mcu_boot_gpio, 0);
		gpio_set_value(priv->mcu_reset_gpio, 0);
		msleep_range(1);
		gpio_set_value(priv->mcu_reset_gpio, 1);
	}

	err = camera_common_initialize(common_data, "ov10823");
	if (err) {
		dev_err(&client->dev, "Failed to initialize ov10823\n");
		return err;
	}

	v4l2_i2c_subdev_init(&common_data->subdev, client,
			     &ov10823_subdev_ops);

	err = ov10823_ctrls_init(priv);
	if (err)
		return err;

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;

	err = ov10823_verify_chip_id(priv);
	if (err)
		goto error;

	err = ov10823_otp_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"Error %d reading otp data\n", err);
		goto error;
	}

	err = ov10823_fuse_id_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"Error %d reading fuse id data\n", err);
		goto error;
	}

	priv->subdev->internal_ops = &ov10823_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &ov10823_media_ops;
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

	dev_info(&client->dev, "Probed v4l2 sensor.\n");

	camera_common_s_power(priv->subdev, false);
	return 0;
error:
	camera_common_s_power(priv->subdev, false);
	return err;
}

static int
ov10823_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov10823 *priv = (struct ov10823 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	ov10823_power_put(priv);
	camera_common_cleanup(s_data);

	return 0;
}

static const struct i2c_device_id ov10823_id[] = {
	{ "ov10823", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov10823_id);

static struct i2c_driver ov10823_i2c_driver = {
	.driver = {
		.name = "ov10823",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov10823_of_match),
	},
	.probe = ov10823_probe,
	.remove = ov10823_remove,
	.id_table = ov10823_id,
};

module_i2c_driver(ov10823_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Omnivison OV10823");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
