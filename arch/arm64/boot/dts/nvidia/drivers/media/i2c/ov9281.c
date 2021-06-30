/*
 * ov9281.c - ov9281 sensor driver
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

#include "ov9281_mode_tbls.h"

/* OV9281 Registers */
#define OV9281_SC_MODE_SELECT_ADDR	0x0100
#define OV9281_SC_MODE_SELECT_STREAMING	0x01
#define OV9281_SC_CHIP_ID_HIGH_ADDR	0x300A
#define OV9281_SC_CHIP_ID_LOW_ADDR	0x300B
#define OV9281_SC_CTRL_SCCB_ID_ADDR	0x302B
#define OV9281_SC_CTRL_3B_ADDR		0x303B
#define OV9281_SC_CTRL_3B_SCCB_ID2_NACK_EN	(1 << 0)
#define OV9281_SC_CTRL_3B_SCCB_PGM_ID_EN	(1 << 1)

#define OV9281_GROUP_HOLD_ADDR		0x3208
#define OV9281_GROUP_HOLD_START		0x00
#define OV9281_GROUP_HOLD_END		0x10
#define OV9281_GROUP_HOLD_LAUNCH_LBLANK	0x60
#define OV9281_GROUP_HOLD_LAUNCH_VBLANK	0xA0
#define OV9281_GROUP_HOLD_LAUNCH_IMMED	0xE0
#define OV9281_GROUP_HOLD_BANK_0	0x00
#define OV9281_GROUP_HOLD_BANK_1	0x01

#define OV9281_EXPO_HIGH_ADDR		0x3500
#define OV9281_EXPO_MID_ADDR		0x3501
#define OV9281_EXPO_LOW_ADDR		0x3502

#define OV9281_GAIN_SHIFT_ADDR		0x3507
#define OV9281_GAIN_HIGH_ADDR		0x3508
#define OV9281_GAIN_LOW_ADDR		0x3509

#define OV9281_TIMING_VTS_HIGH_ADDR	0x380E
#define OV9281_TIMING_VTS_LOW_ADDR	0x380F
#define OV9281_TIMING_FORMAT1		0x3820
#define OV9281_TIMING_FORMAT1_VBIN	(1 << 1)
#define OV9281_TIMING_FORMAT1_FLIP	(1 << 2)
#define OV9281_TIMING_FORMAT2		0x3821
#define OV9281_TIMING_FORMAT2_HBIN	(1 << 0)
#define OV9281_TIMING_FORMAT2_MIRROR	(1 << 2)
#define OV9281_TIMING_RST_FSIN_HIGH_ADDR	0x3826
#define OV9281_TIMING_RST_FSIN_LOW_ADDR	0x3827

#define OV9281_OTP_BUFFER_ADDR		0x3D00
#define OV9281_OTP_BUFFER_SIZE		32
#define OV9281_OTP_STR_SIZE		(OV9281_OTP_BUFFER_SIZE * 2)
#define OV9281_FUSE_ID_OTP_BUFFER_ADDR	0x3D00
#define OV9281_FUSE_ID_OTP_BUFFER_SIZE	16
#define OV9281_FUSE_ID_STR_SIZE		(OV9281_FUSE_ID_OTP_BUFFER_SIZE * 2)
#define OV9281_OTP_PROGRAM_CTRL_ADDR	0x3D80
#define OV9281_OTP_LOAD_CTRL_ADDR	0x3D81
#define OV9281_OTP_LOAD_CTRL_OTP_RD	0x01

#define OV9281_PRE_CTRL00_ADDR		0x5E00
#define OV9281_PRE_CTRL00_TEST_PATTERN_EN	(1 << 7)

/* OV9281 Other Stuffs */
#define OV9281_DEFAULT_GAIN		0x0010 /* 1.0x real gain */
#define OV9281_MIN_GAIN			0x0001
#define OV9281_MAX_GAIN			0x1FFF

#define OV9281_DEFAULT_FRAME_LENGTH	0x071C
#define OV9281_MIN_FRAME_LENGTH		0x0001
#define OV9281_MAX_FRAME_LENGTH		0xFFFF
#define OV9281_FRAME_LENGTH_1SEC	(0x40d * 120) /* TODO: try to calc */

#define OV9281_MIN_EXPOSURE_COARSE	0x00000001
#define OV9281_MAX_EXPOSURE_COARSE	0x000FFFFF
#define OV9281_DEFAULT_EXPOSURE_COARSE	0x00002A90

#define OV9281_MAX_WIDTH		1280
#define OV9281_MAX_HEIGHT		800

#define OV9281_DEFAULT_MODE		OV9281_MODE_1280X800
#define OV9281_DEFAULT_WIDTH		OV9281_MAX_WIDTH
#define OV9281_DEFAULT_HEIGHT		OV9281_MAX_HEIGHT
#define OV9281_DEFAULT_DATAFMT		MEDIA_BUS_FMT_SBGGR10_1X10
#define OV9281_DEFAULT_CLK_FREQ		26000000

#define OV9281_DEFAULT_I2C_ADDRESS_C0		(0xc0 >> 1)
#define OV9281_DEFAULT_I2C_ADDRESS_20		(0x20 >> 1)
#define OV9281_DEFAULT_I2C_ADDRESS_PROGRAMMABLE	(0xe0 >> 1)

struct ov9281 {
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
	int				frame_period_ms;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

/* Register/regmap stuff */
static int ov9281_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;
	unsigned int temp_val;
	int err;

	err = regmap_read(priv->regmap, addr, &temp_val);
	if (!err)
		*val = temp_val;

	return err;
}

static int ov9281_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;
	struct device *dev = &priv->i2c_client->dev;
	int err;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int ov9281_write_table(struct ov9281 *priv, const ov9281_reg table[])
{
	return regmap_util_write_table_8(priv->regmap, table, NULL, 0,
					 OV9281_TABLE_WAIT_MS,
					 OV9281_TABLE_END);
}

static const struct regmap_config ov9281_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static int ov9281_i2c_addr_assign(struct ov9281 *priv, u8 i2c_addr)
{
	struct device *dev = &priv->i2c_client->dev;
	struct i2c_msg msg;
	unsigned char data[3];
	int err = 0;

	/*
	 * It seems that the way SID works for the OV9281 I2C slave address is
	 * that:
	 *
	 * SID 0 = 0xc0, 0xe0
	 * SID 1 = 0x20, 0xe0
	 *
	 * Address 0xe0 is programmable via register 0x302B
	 * (OV9281_SC_CTRL_SCCB_ID_ADDR).
	 *
	 * So, the scheme to assign addresses to an (almost) arbitrary
	 * number of sensors is to consider 0x20 to be the "off" address.
	 * Start each sensor with SID as 1 so that they appear to be off.
	 *
	 * Then, to assign an address to one sensor:
	 *
	 * 0. Set corresponding SID to 0 (now only that sensor responds
	 *    to 0xc0).
	 * 1. Use 0xc0 to program the address from the default programmable
	 *    address of 0xe0 to the new address.
	 * 2. Set corresponding SID back to 1 (so it no longer responds
	 *    to 0xc0).
	 */

	if (i2c_addr == OV9281_DEFAULT_I2C_ADDRESS_C0) {
		dev_info(dev, "Using default I2C address 0x%02x\n", i2c_addr);
		if (gpio_is_valid(priv->cam_sid_gpio)) {
			gpio_set_value(priv->cam_sid_gpio, 0);
			msleep_range(1);
		}
		return 0;
	} else if (i2c_addr == OV9281_DEFAULT_I2C_ADDRESS_20) {
		dev_info(dev, "Using default I2C address 0x%02x\n", i2c_addr);
		if (gpio_is_valid(priv->cam_sid_gpio)) {
			gpio_set_value(priv->cam_sid_gpio, 1);
			msleep_range(1);
		}
		return 0;
	} else if (i2c_addr == OV9281_DEFAULT_I2C_ADDRESS_PROGRAMMABLE) {
		dev_info(dev, "Using default I2C address 0x%02x\n", i2c_addr);
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

	gpio_set_value(priv->cam_sid_gpio, 0);
	msleep_range(1);

	/*
	 * Have to make the I2C message manually because we are using a
	 * different I2C slave address for this transaction, rather than
	 * the one in the device tree for this device.
	 */
	data[0] = (OV9281_SC_CTRL_SCCB_ID_ADDR >> 8) & 0xff;
	data[1] = OV9281_SC_CTRL_SCCB_ID_ADDR & 0xff;
	data[2] = ((i2c_addr) << 1) & 0xff;

	/*
	 * Use the programmable default I2C slave address so that if we have
	 * multiple sensors of this same kind, when we change one sensor's
	 * address, the next sensor address change message won't go to that
	 * same sensor.
	 */
	msg.addr = OV9281_DEFAULT_I2C_ADDRESS_C0;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	if (i2c_transfer(priv->i2c_client->adapter, &msg, 1) != 1) {
		dev_err(dev, "Error assigning I2C address to 0x%02x\n",
			i2c_addr);
		err = -EIO;
	}

	gpio_set_value(priv->cam_sid_gpio, 1);
	msleep_range(1);

	return err;
}

/* NVIDIA camera_common stuff */
static int ov9281_power_on(struct camera_common_data *s_data)
{
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;
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

	err = ov9281_i2c_addr_assign(priv, priv->i2c_client->addr);
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

static int ov9281_power_off(struct camera_common_data *s_data)
{
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	dev_dbg(dev, "%s: power off\n", __func__);
	ov9281_write_table(priv, ov9281_mode_table[OV9281_MODE_STOP_STREAM]);

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

static int ov9281_power_put(struct ov9281 *priv)
{
	return 0;
}

static int ov9281_power_get(struct ov9281 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	struct device *dev = &priv->i2c_client->dev;
	const char *mclk_name;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
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
	return 0;
}

static struct camera_common_sensor_ops ov9281_common_ops = {
	.power_on = ov9281_power_on,
	.power_off = ov9281_power_off,
	.write_reg = ov9281_write_reg,
	.read_reg = ov9281_read_reg,
};

/* Miscellaneous OV9281-specific stuff */
static int ov9281_set_group_hold(struct ov9281 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];
	int err;

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		/* group hold start */
		err = ov9281_write_reg(priv->s_data, OV9281_GROUP_HOLD_ADDR,
				       (OV9281_GROUP_HOLD_START |
					OV9281_GROUP_HOLD_BANK_0));
		if (err)
			goto fail;
		priv->group_hold_prev = 1;
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		/* group hold end */
		err = ov9281_write_reg(priv->s_data, OV9281_GROUP_HOLD_ADDR,
				       (OV9281_GROUP_HOLD_END |
					OV9281_GROUP_HOLD_BANK_0));
		/* quick launch */
		err |= ov9281_write_reg(priv->s_data,
				       OV9281_GROUP_HOLD_ADDR,
				       (OV9281_GROUP_HOLD_LAUNCH_VBLANK |
					OV9281_GROUP_HOLD_BANK_0));
		if (err)
			goto fail;
		priv->group_hold_prev = 0;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int ov9281_set_gain(struct ov9281 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	ov9281_reg regs[4];
	u16 gain;
	int err;

	if (val < OV9281_MIN_GAIN)
		gain = OV9281_MIN_GAIN;
	else if (val > OV9281_MAX_GAIN)
		gain = OV9281_MAX_GAIN;
	else
		gain = val;

	dev_dbg(dev, "%s: gain: %d\n", __func__, gain);

	regs[0].addr = OV9281_GAIN_SHIFT_ADDR;
	regs[0].val = 0x03;
	regs[1].addr = OV9281_GAIN_HIGH_ADDR;
	regs[1].val = gain >> 8;
	regs[2].addr = OV9281_GAIN_LOW_ADDR;
	regs[2].val = gain & 0xff;
	regs[3].addr = OV9281_TABLE_END;
	regs[3].val = 0;

	ov9281_set_group_hold(priv);
	err = ov9281_write_table(priv, regs);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int ov9281_set_frame_length(struct ov9281 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	ov9281_reg regs[5];
	u16 frame_length;
	int err;

	frame_length = (u16)val;

	dev_dbg(dev, "%s: frame_length: %d\n", __func__, frame_length);

	regs[0].addr = OV9281_TIMING_VTS_HIGH_ADDR;
	regs[0].val = (frame_length >> 8) & 0xff;
	regs[1].addr = OV9281_TIMING_VTS_LOW_ADDR;
	regs[1].val = (frame_length) & 0xff;
	regs[2].addr = OV9281_TABLE_END;
	regs[2].val = 0;

	if (priv->fsync == OV9281_FSYNC_SLAVE) {
		regs[2].addr = OV9281_TIMING_RST_FSIN_HIGH_ADDR;
		regs[2].val = ((frame_length - 4) >> 8) & 0xff;
		regs[3].addr = OV9281_TIMING_RST_FSIN_LOW_ADDR;
		regs[3].val = (frame_length - 4) & 0xff;
		regs[4].addr = OV9281_TABLE_END;
		regs[4].val = 0;
	}

	ov9281_set_group_hold(priv);
	err = ov9281_write_table(priv, regs);
	if (err)
		goto fail;

	priv->frame_period_ms = (frame_length * 1000) /
				OV9281_FRAME_LENGTH_1SEC;
	dev_dbg(dev, "%s: frame_period_ms: %d\n",
		__func__, priv->frame_period_ms);

	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int ov9281_set_coarse_time(struct ov9281 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	ov9281_reg regs[4];
	u32 coarse_time;
	int err;

	coarse_time = (u32)val;

	dev_dbg(dev, "%s: coarse_time: %d\n", __func__, coarse_time);

	regs[0].addr = OV9281_EXPO_HIGH_ADDR;
	regs[0].val = (coarse_time >> 16) & 0xff;
	regs[1].addr = OV9281_EXPO_MID_ADDR;
	regs[1].val = (coarse_time >> 8) & 0xff;
	regs[2].addr = OV9281_EXPO_LOW_ADDR;
	regs[2].val = (coarse_time & 0xff);
	regs[3].addr = OV9281_TABLE_END;
	regs[3].val = 0;

	ov9281_set_group_hold(priv);
	err = ov9281_write_table(priv, regs);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(dev, "%s: COARSE_TIME control error\n", __func__);
	return err;
}

/* OTP stuff */
static int ov9281_read_otp(struct ov9281 *priv, u8 *buf, u16 addr, int size)
{
	int i;
	int err;

	err = ov9281_write_reg(priv->s_data, OV9281_SC_MODE_SELECT_ADDR,
			       OV9281_SC_MODE_SELECT_STREAMING);
	if (err)
		return err;

	for (i = 0; i < size; i++) {
		err = ov9281_write_reg(priv->s_data, addr + i, 0x00);
		if (err)
			return err;
	}

	err = ov9281_write_reg(priv->s_data, OV9281_OTP_LOAD_CTRL_ADDR,
			       OV9281_OTP_LOAD_CTRL_OTP_RD);

	msleep(20);

	return regmap_bulk_read(priv->regmap, addr, buf, size);
}

static int ov9281_otp_setup(struct ov9281 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	struct v4l2_ctrl *ctrl;
	u8 otp_buf[OV9281_OTP_BUFFER_SIZE];
	int i;
	int err;

	err = ov9281_read_otp(priv, otp_buf, OV9281_OTP_BUFFER_ADDR,
			      OV9281_OTP_BUFFER_SIZE);
	if (err)
		return err;

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, TEGRA_CAMERA_CID_OTP_DATA);
	if (!ctrl) {
		dev_err(dev, "could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < OV9281_OTP_BUFFER_SIZE; i++)
		sprintf(&ctrl->p_new.p_char[i*2], "%02x", otp_buf[i]);
	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static int ov9281_fuse_id_setup(struct ov9281 *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	struct v4l2_ctrl *ctrl;
	u8 fuse_id[OV9281_FUSE_ID_OTP_BUFFER_SIZE];
	int i;
	int err;

	err = ov9281_read_otp(priv, fuse_id, OV9281_FUSE_ID_OTP_BUFFER_ADDR,
			      OV9281_FUSE_ID_OTP_BUFFER_SIZE);
	if (err)
		return err;

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, TEGRA_CAMERA_CID_FUSE_ID);
	if (!ctrl) {
		dev_err(dev, "could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < OV9281_FUSE_ID_OTP_BUFFER_SIZE; i++)
		sprintf(&ctrl->p_new.p_char[i*2], "%02x", fuse_id[i]);
	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

/* V4L2 subdev stuff */
static int ov9281_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;
	struct v4l2_control control;
	int err;

	if (!enable) {
		dev_dbg(dev, "%s: stream off\n", __func__);
		return ov9281_write_table(priv,
			ov9281_mode_table[OV9281_MODE_STOP_STREAM]);
	}

	dev_dbg(dev, "%s: write mode table %d\n", __func__, s_data->mode);
	err = ov9281_write_table(priv, ov9281_mode_table[s_data->mode]);
	if (err)
		goto exit;

	if (ov9281_fsync_table[priv->fsync]) {
		dev_dbg(dev, "%s: write fsync table %d\n", __func__,
			priv->fsync);
		err = ov9281_write_table(priv, ov9281_fsync_table[priv->fsync]);
		if (err)
			goto exit;
	}

	if ((priv->fsync == OV9281_FSYNC_SLAVE) &&
	    ov9281_fsync_slave_mode_table[s_data->mode]) {
		dev_dbg(dev, "%s: write fsync slave mode table %d\n",
			__func__, s_data->mode);
		err = ov9281_write_table(
			priv, ov9281_fsync_slave_mode_table[s_data->mode]);
		if (err)
			goto exit;
	}

	if (s_data->override_enable) {
		/* write list of override regs for the asking frame length, */
		/* coarse integration time, and gain. Failures to write */
		/* overrides are non-fatal. */
		control.id = TEGRA_CAMERA_CID_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= ov9281_set_gain(priv, control.value);
		if (err)
			dev_warn(dev, "%s: error gain override\n", __func__);

		control.id = TEGRA_CAMERA_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= ov9281_set_frame_length(priv, control.value);
		if (err)
			dev_warn(dev, "%s: error frame length override\n",
				__func__);

		control.id = TEGRA_CAMERA_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= ov9281_set_coarse_time(priv, control.value);
		if (err)
			dev_warn(dev, "%s: error coarse time override\n",
				__func__);
	}

	/*
	 * Handle mirror and flip.
	 * Vertical and horizontal binning are in the same registers, so
	 * need to take frame resolution into account (to avoid a register
	 * read).
	 */
	if (priv->mirror) {
		if (s_data->frmfmt->size.width > (OV9281_MAX_WIDTH / 2))
			ov9281_write_reg(s_data, OV9281_TIMING_FORMAT2,
					 OV9281_TIMING_FORMAT2_MIRROR);
		else
			ov9281_write_reg(s_data, OV9281_TIMING_FORMAT2,
					 OV9281_TIMING_FORMAT2_HBIN |
					 OV9281_TIMING_FORMAT2_MIRROR);
	}

	if (priv->flip) {
		if (s_data->frmfmt->size.height > (OV9281_MAX_HEIGHT / 2))
			ov9281_write_reg(s_data, OV9281_TIMING_FORMAT1,
					 OV9281_TIMING_FORMAT1_FLIP);
		else
			ov9281_write_reg(s_data, OV9281_TIMING_FORMAT1,
					 OV9281_TIMING_FORMAT1_VBIN |
					 OV9281_TIMING_FORMAT1_FLIP);
	}

#ifdef TPG
	err = ov9281_write_reg(priv->s_data, OV9281_PRE_CTRL00_ADDR,
			       OV9281_PRE_CTRL00_TEST_PATTERN_EN);
	if (err)
		dev_warn(dev, "%s: error enabling TPG\n", __func__);
#endif

	dev_dbg(dev, "%s: stream on\n", __func__);
	err = ov9281_write_table(priv,
		ov9281_mode_table[OV9281_MODE_START_STREAM]);
	if (err)
		goto exit;

	/*
	 * If the sensor is in fsync slave mode, and is in the middle of
	 * sending a frame when it gets a strobe on the fsin pin, it may
	 * prematurely end the frame, resulting in a short frame on our
	 * camera host.  So, after starting streaming, we assume fsync
	 * master has already been told to start streaming, and we wait some
	 * amount of time in order to skip the possible short frame.  The
	 * length of time to wait should be at least our frame period.
	 * Add a little bit extra as a safety margin.
	 */
	if (priv->fsync == OV9281_FSYNC_SLAVE)
		msleep_range(priv->frame_period_ms + 10);

	return 0;

exit:
	dev_err(dev, "%s: error setting stream\n", __func__);
	return err;
}

static int ov9281_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static int ov9281_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	int err;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		err = camera_common_try_fmt(sd, &format->format);
	else
		err = camera_common_s_fmt(sd, &format->format);

	return err;
}

static int ov9281_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static struct v4l2_subdev_core_ops ov9281_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static struct v4l2_subdev_video_ops ov9281_subdev_video_ops = {
	.s_stream	= ov9281_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status	= ov9281_g_input_status,
};

static struct v4l2_subdev_pad_ops ov9281_subdev_pad_ops = {
	.set_fmt	= ov9281_set_fmt,
	.get_fmt	= ov9281_get_fmt,
	.enum_mbus_code	= camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops ov9281_subdev_ops = {
	.core		= &ov9281_subdev_core_ops,
	.video		= &ov9281_subdev_video_ops,
	.pad		= &ov9281_subdev_pad_ops,
};

/* V4L2 controls stuff */
static int ov9281_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9281 *priv =
		container_of(ctrl->handler, struct ov9281, ctrl_handler);
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

static int ov9281_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9281 *priv =
		container_of(ctrl->handler, struct ov9281, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = ov9281_set_gain(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_FRAME_LENGTH:
		/*
		 * This is a workaround for nvbug 1865041, where setting the
		 * VTS timing registers when the sensor is set up for fsync
		 * master or slave leads to instability if streaming has
		 * already started.
		 */
		if (priv->fsync == OV9281_FSYNC_NONE)
			err = ov9281_set_frame_length(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_COARSE_TIME:
		err = ov9281_set_coarse_time(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = ov9281_set_group_hold(priv);
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

static const struct v4l2_ctrl_ops ov9281_ctrl_ops = {
	.g_volatile_ctrl = ov9281_g_volatile_ctrl,
	.s_ctrl		= ov9281_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &ov9281_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV9281_MIN_GAIN,
		.max = OV9281_MAX_GAIN,
		.def = OV9281_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &ov9281_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV9281_MIN_FRAME_LENGTH,
		.max = OV9281_MAX_FRAME_LENGTH,
		.def = OV9281_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &ov9281_ctrl_ops,
		.id = TEGRA_CAMERA_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV9281_MIN_EXPOSURE_COARSE,
		.max = OV9281_MAX_EXPOSURE_COARSE,
		.def = OV9281_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &ov9281_ctrl_ops,
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
		.ops = &ov9281_ctrl_ops,
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
		.ops = &ov9281_ctrl_ops,
		.id = TEGRA_CAMERA_CID_OTP_DATA,
		.name = "OTP Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV9281_OTP_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &ov9281_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV9281_FUSE_ID_STR_SIZE,
		.step = 2,
	},
};

static int ov9281_ctrls_init(struct ov9281 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int i;
	int err;

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

static const struct media_entity_operations ov9281_media_ops = {
	.link_validate	= v4l2_subdev_link_validate,
};

/* Driver probe helper stuff */
static int ov9281_parse_dt(struct i2c_client *client, struct ov9281 *priv)
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
		priv->fsync = OV9281_FSYNC_MASTER;
	else if (!err && fsync_str && (strcmp(fsync_str, "slave") == 0))
		priv->fsync = OV9281_FSYNC_SLAVE;
	else
		priv->fsync = OV9281_FSYNC_NONE;

	gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (!gpio_is_valid(gpio)) {
		dev_dbg(&client->dev, "pwdn gpios not in DT\n");
		gpio = 0;
	}
	priv->pdata->pwdn_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (!gpio_is_valid(gpio)) {
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

static int ov9281_verify_chip_id(struct ov9281 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct camera_common_data *s_data = priv->s_data;
	u8 chip_id_hi, chip_id_lo;
	u16 chip_id;
	int err;

	err = ov9281_read_reg(s_data, OV9281_SC_CHIP_ID_HIGH_ADDR, &chip_id_hi);
	if (err) {
		dev_err(&client->dev, "Failed to read chip ID\n");
		return err;
	}
	err = ov9281_read_reg(s_data, OV9281_SC_CHIP_ID_LOW_ADDR, &chip_id_lo);
	if (err) {
		dev_err(&client->dev, "Failed to read chip ID\n");
		return err;
	}

	chip_id = (chip_id_hi << 8) | chip_id_lo;
	if (chip_id != 0x9281) {
		dev_err(&client->dev, "Read unknown chip ID 0x%04x\n", chip_id);
		return -EINVAL;
	}

	return 0;
}

static const struct of_device_id ov9281_of_match[] = {
	{ .compatible = "nvidia,ov9281", },
	{ },
};

MODULE_DEVICE_TABLE(of, ov9281_of_match);

static int ov9281_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device *dev = &client->dev;
	struct ov9281 *priv;
	int err;

	dev_info(dev, "probing v4l2 sensor.\n");

	common_data = devm_kzalloc(dev, sizeof(*common_data), GFP_KERNEL);

	priv = devm_kzalloc(dev,
			    sizeof(struct ov9281) +
			    (sizeof(struct v4l2_ctrl *) *
			     ARRAY_SIZE(ctrl_config_list)),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "unable to allocate camera_common_data\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &ov9281_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "regmap init failed %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = devm_kzalloc(dev, sizeof(struct camera_common_pdata),
				   GFP_KERNEL);
	if (!priv->pdata) {
		dev_err(dev, "unable to allocate camera_common_pdata\n");
		return -ENOMEM;
	}

	err = ov9281_parse_dt(client, priv);
	if (err)
		return err;

	common_data->ops		= &ov9281_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->dev		= &client->dev;
	common_data->frmfmt		= ov9281_frmfmt;
	common_data->colorfmt		=
		camera_common_find_datafmt(OV9281_DEFAULT_DATAFMT);
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(ov9281_frmfmt);
	common_data->def_mode		= OV9281_DEFAULT_MODE;
	common_data->def_width		= OV9281_DEFAULT_WIDTH;
	common_data->def_height		= OV9281_DEFAULT_HEIGHT;
	common_data->def_clk_freq	= OV9281_DEFAULT_CLK_FREQ;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;

	priv->i2c_client		= client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;
	priv->group_hold_prev		= 0;

	err = ov9281_power_get(priv);
	if (err)
		return err;

	/*
	 * If our device tree node is given MCU GPIOs, then we are expected to
	 * reset the MCU.
	 */
	if (gpio_is_valid(priv->mcu_boot_gpio) &&
	    gpio_is_valid(priv->mcu_reset_gpio)) {
		dev_info(dev, "Resetting MCU\n");
		gpio_set_value(priv->mcu_boot_gpio, 0);
		gpio_set_value(priv->mcu_reset_gpio, 0);
		msleep_range(1);
		gpio_set_value(priv->mcu_reset_gpio, 1);
	}

	err = camera_common_initialize(common_data, "ov9281");
	if (err) {
		dev_err(dev, "Failed to initialize ov9281\n");
		return err;
	}

	v4l2_i2c_subdev_init(&common_data->subdev, client,
			     &ov9281_subdev_ops);

	err = ov9281_ctrls_init(priv);
	if (err)
		return err;

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;

	err = ov9281_verify_chip_id(priv);
	if (err)
		goto error;

	err = ov9281_otp_setup(priv);
	if (err) {
		dev_err(dev, "Error %d reading otp data\n", err);
		return err;
	}

	err = ov9281_fuse_id_setup(priv);
	if (err) {
		dev_err(dev, "Error %d reading fuse id data\n", err);
		return err;
	}

	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &ov9281_media_ops;
	err = tegra_media_entity_init(&priv->subdev->entity, 1,
				&priv->pad, true, true);
	if (err < 0) {
		dev_err(dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_info(dev, "Probed v4l2 sensor.\n");

	camera_common_s_power(priv->subdev, false);
	return 0;
error:
	camera_common_s_power(priv->subdev, false);
	return err;

}

static int
ov9281_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	ov9281_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id ov9281_id[] = {
	{ "ov9281", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov9281_id);

static struct i2c_driver ov9281_i2c_driver = {
	.driver = {
		.name = "ov9281",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov9281_of_match),
	},
	.probe = ov9281_probe,
	.remove = ov9281_remove,
	.id_table = ov9281_id,
};

module_i2c_driver(ov9281_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Omnivison OV9281");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
