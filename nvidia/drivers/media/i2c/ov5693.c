/*
 * ov5693_v4l2.c - ov5693 sensor driver
 *
 * Copyright (c) 2013-2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/debugfs.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>
#include <media/ov5693.h>


#include "../platform/tegra/camera/camera_gpio.h"
#include "ov5693_mode_tbls.h"
#define CREATE_TRACE_POINTS
#include <trace/events/ov5693.h>

#define OV5693_MAX_COARSE_DIFF		6
#define OV5693_MAX_FRAME_LENGTH	(0x7fff)
#define OV5693_MIN_EXPOSURE_COARSE	(0x0002)
#define OV5693_MAX_EXPOSURE_COARSE	\
	(OV5693_MAX_FRAME_LENGTH-OV5693_MAX_COARSE_DIFF)
#define OV5693_DEFAULT_LINE_LENGTH	(0xA80)
#define OV5693_DEFAULT_PIXEL_CLOCK	(160)
#define OV5693_DEFAULT_FRAME_LENGTH	(0x07C0)
#define OV5693_DEFAULT_EXPOSURE_COARSE	\
	(OV5693_DEFAULT_FRAME_LENGTH-OV5693_MAX_COARSE_DIFF)

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_EEPROM_DATA,
	TEGRA_CAMERA_CID_OTP_DATA,
	TEGRA_CAMERA_CID_FUSE_ID,
};

struct ov5693 {
	struct camera_common_eeprom_data eeprom[OV5693_EEPROM_NUM_BLOCKS];
	u8				eeprom_buf[OV5693_EEPROM_SIZE];
	u8				otp_buf[OV5693_OTP_SIZE];
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	u8				fuse_id[OV5693_FUSE_ID_SIZE];
	const char			*devname;
	struct dentry			*debugfs_dir;
	struct mutex			streaming_lock;
	bool				streaming;

	s32				group_hold_prev;
	u32				frame_length;
	bool				group_hold_en;
	struct camera_common_i2c	i2c_dev;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static struct regmap_config ov5693_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static inline void ov5693_get_frame_length_regs(ov5693_reg *regs,
				u32 frame_length)
{
	regs->addr = OV5693_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = OV5693_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov5693_get_coarse_time_regs(ov5693_reg *regs,
				u32 coarse_time)
{
	regs->addr = OV5693_COARSE_TIME_ADDR_1;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = OV5693_COARSE_TIME_ADDR_2;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = OV5693_COARSE_TIME_ADDR_3;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static inline void ov5693_get_coarse_time_short_regs(ov5693_reg *regs,
				u32 coarse_time)
{
	regs->addr = OV5693_COARSE_TIME_SHORT_ADDR_1;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = OV5693_COARSE_TIME_SHORT_ADDR_2;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = OV5693_COARSE_TIME_SHORT_ADDR_3;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static inline void ov5693_get_gain_regs(ov5693_reg *regs,
				u16 gain)
{
	regs->addr = OV5693_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;

	(regs + 1)->addr = OV5693_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int ov5693_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int ov5693_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);

	if (err)
		dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int ov5693_write_table(struct ov5693 *priv,
			      const ov5693_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 OV5693_TABLE_WAIT_MS,
					 OV5693_TABLE_END);
}

static void ov5693_gpio_set(struct camera_common_data *s_data,
			    unsigned int gpio, int val)
{
	struct camera_common_pdata *pdata = s_data->pdata;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_ctrl(s_data->dev, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}

static int ov5693_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;


	dev_dbg(dev, "%s: power on\n", __func__);

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}
	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor
	 */

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto ov5693_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto ov5693_iovdd_fail;

	usleep_range(1, 2);
	if (gpio_is_valid(pw->pwdn_gpio))
		ov5693_gpio_set(s_data, pw->pwdn_gpio, 1);

	/*
	 * datasheet 2.9: reset requires ~2ms settling time
	 * a power on reset is generated after core power becomes stable
	 */
	usleep_range(2000, 2010);

	if (gpio_is_valid(pw->reset_gpio))
		ov5693_gpio_set(s_data, pw->reset_gpio, 1);

	/* datasheet fig 2-9: t3 */
	usleep_range(2000, 2010);

	pw->state = SWITCH_ON;


	return 0;

ov5693_iovdd_fail:
	regulator_disable(pw->avdd);

ov5693_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);
	return -ENODEV;
}

static int ov5693_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct device *dev = s_data->dev;
	struct camera_common_pdata *pdata = s_data->pdata;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err) {
			goto power_off_done;
		} else {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	}

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor
	 */
	usleep_range(21, 25);
	if (gpio_is_valid(pw->pwdn_gpio))
		ov5693_gpio_set(s_data, pw->pwdn_gpio, 0);
	usleep_range(1, 2);
	if (gpio_is_valid(pw->reset_gpio))
		ov5693_gpio_set(s_data, pw->reset_gpio, 0);

	/* datasheet 2.9: reset requires ~2ms settling time*/
	usleep_range(2000, 2010);

	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->avdd)
		regulator_disable(pw->avdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int ov5693_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;

	if (unlikely(!pw))
		return -EFAULT;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_deregister(dev, pw->pwdn_gpio);
	else {
		if (gpio_is_valid(pw->pwdn_gpio))
			gpio_free(pw->pwdn_gpio);
		if (gpio_is_valid(pw->reset_gpio))
			gpio_free(pw->reset_gpio);
	}

	return 0;
}

static int ov5693_power_get(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = tc_dev->dev;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0, ret = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}
	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	/* analog 2.8v */
	err |= camera_common_regulator_get(dev,
			&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(dev,
			&pw->iovdd, pdata->regulators.iovdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}
	if (pdata->use_cam_gpio) {
		err = cam_gpio_register(dev, pw->pwdn_gpio);
		if (err)
			dev_err(dev, "%s ERR can't register cam gpio %u!\n",
				 __func__, pw->pwdn_gpio);
	} else {
		if (gpio_is_valid(pw->pwdn_gpio)) {
			ret = gpio_request(pw->pwdn_gpio, "cam_pwdn_gpio");
			if (ret < 0) {
				dev_dbg(dev, "%s can't request pwdn_gpio %d\n",
					__func__, ret);
			}
			gpio_direction_output(pw->pwdn_gpio, 1);
		}
		if (gpio_is_valid(pw->reset_gpio)) {
			ret = gpio_request(pw->reset_gpio, "cam_reset_gpio");
			if (ret < 0) {
				dev_dbg(dev, "%s can't request reset_gpio %d\n",
					__func__, ret);
			}
			gpio_direction_output(pw->reset_gpio, 1);
		}
	}

	pw->state = SWITCH_OFF;
	return err;
}

static int ov5693_set_gain(struct tegracam_device *tc_dev, s64 val);
static int ov5693_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int ov5693_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int ov5693_set_exposure_short(struct tegracam_device *tc_dev, s64 val);

static const struct of_device_id ov5693_of_match[] = {
	{
		.compatible = "nvidia,ov5693",
	},
	{ },
};

static int ov5693_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	int err;
	struct ov5693 *priv = tc_dev->priv;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];
	struct device *dev = tc_dev->dev;

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		camera_common_i2c_aggregate(&priv->i2c_dev, true);
		/* enter group hold */
		err = ov5693_write_reg(priv->s_data,
				       OV5693_GROUP_HOLD_ADDR, val);
		if (err)
			goto fail;

		priv->group_hold_prev = 1;

		dev_dbg(dev, "%s: enter group hold\n", __func__);
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		/* leave group hold */
		err = ov5693_write_reg(priv->s_data,
				       OV5693_GROUP_HOLD_ADDR, 0x11);
		if (err)
			goto fail;

		err = ov5693_write_reg(priv->s_data,
				       OV5693_GROUP_HOLD_ADDR, 0x61);
		if (err)
			goto fail;

		camera_common_i2c_aggregate(&priv->i2c_dev, false);

		priv->group_hold_prev = 0;

		dev_dbg(dev, "%s: leave group hold\n", __func__);
	}

	return 0;

fail:
	dev_dbg(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int ov5693_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5693 *priv = (struct ov5693 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[2];
	int err;
	u16 gain;
	int i;

	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	/* translate value */
	gain = (u16) (((val * 16) +
			(mode->control_properties.gain_factor / 2)) /
			mode->control_properties.gain_factor);
	ov5693_get_gain_regs(reg_list, gain);
	dev_dbg(dev, "%s: gain %d val: %lld\n", __func__, gain, val);

	for (i = 0; i < 2; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int ov5693_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5693 *priv = tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[2];
	int err;
	u32 frame_length;
	int i;

	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	frame_length =  mode->signal_properties.pixel_clock.val *
		mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val;

	ov5693_get_frame_length_regs(reg_list, frame_length);
	dev_dbg(dev, "%s: val: %d\n", __func__, frame_length);

	for (i = 0; i < 2; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	priv->frame_length = frame_length;

	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int ov5693_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5693 *priv = tc_dev->priv;
	const s32 max_coarse_time = priv->frame_length - OV5693_MAX_COARSE_DIFF;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[3];
	int err;
	u32 coarse_time;
	int i;

	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	coarse_time = (u32)(((mode->signal_properties.pixel_clock.val*val)
			/mode->image_properties.line_length)/
			mode->control_properties.exposure_factor);
	if (coarse_time < OV5693_MIN_EXPOSURE_COARSE)
		coarse_time = OV5693_MIN_EXPOSURE_COARSE;
	else if (coarse_time > max_coarse_time)
		coarse_time = max_coarse_time;
	ov5693_get_coarse_time_regs(reg_list, coarse_time);
	dev_dbg(dev, "%s: val: %d\n", __func__, coarse_time);

	for (i = 0; i < 3; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: COARSE_TIME control error\n", __func__);
	return err;
}

static int ov5693_set_exposure_short(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5693 *priv = tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[3];
	int err;
	struct v4l2_control hdr_control;
	int hdr_en;
	u32 coarse_time_short;
	int i;
	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	/* check hdr enable ctrl */
	hdr_control.id = TEGRA_CAMERA_CID_HDR_EN;

	err = camera_common_g_ctrl(s_data, &hdr_control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[hdr_control.value];
	if (hdr_en == SWITCH_OFF)
		return 0;

	coarse_time_short = (u32)(((mode->signal_properties.pixel_clock.val*val)
				/mode->image_properties.line_length)
				/mode->control_properties.exposure_factor);

	ov5693_get_coarse_time_short_regs(reg_list, coarse_time_short);
	dev_dbg(dev, "%s: val: %d\n", __func__, coarse_time_short);

	for (i = 0; i < 3; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: COARSE_TIME_SHORT control error\n", __func__);
	return err;
}

static int ov5693_fill_string_ctrl(struct tegracam_device *tc_dev,
				struct v4l2_ctrl *ctrl)
{
	struct ov5693 *priv = tc_dev->priv;
	int i;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_EEPROM_DATA:
		for (i = 0; i < OV5693_EEPROM_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->eeprom_buf[i]);
		break;
	case TEGRA_CAMERA_CID_OTP_DATA:
		for (i = 0; i < OV5693_OTP_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->otp_buf[i]);
		break;
	case TEGRA_CAMERA_CID_FUSE_ID:
		for (i = 0; i < OV5693_FUSE_ID_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->fuse_id[i]);
		break;
	default:
		return -EINVAL;
	}
	ctrl->p_cur.p_char = ctrl->p_new.p_char;
	return 0;
}

static int ov5693_eeprom_device_release(struct ov5693 *priv)
{
	int i;

	for (i = 0; i < OV5693_EEPROM_NUM_BLOCKS; i++) {
		if (priv->eeprom[i].i2c_client != NULL) {
			i2c_unregister_device(priv->eeprom[i].i2c_client);
			priv->eeprom[i].i2c_client = NULL;
		}
	}

	return 0;
}

static int ov5693_eeprom_device_init(struct ov5693 *priv)
{
	struct camera_common_pdata *pdata =  priv->s_data->pdata;
	char *dev_name = "eeprom_ov5693";
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int i;
	int err;

	if (!pdata->has_eeprom)
		return -EINVAL;

	for (i = 0; i < OV5693_EEPROM_NUM_BLOCKS; i++) {
		priv->eeprom[i].adap = i2c_get_adapter(
				priv->i2c_client->adapter->nr);
		memset(&priv->eeprom[i].brd, 0, sizeof(priv->eeprom[i].brd));
		strncpy(priv->eeprom[i].brd.type, dev_name,
				sizeof(priv->eeprom[i].brd.type));
		priv->eeprom[i].brd.addr = OV5693_EEPROM_ADDRESS + i;
		priv->eeprom[i].i2c_client = i2c_new_device(
				priv->eeprom[i].adap, &priv->eeprom[i].brd);

		priv->eeprom[i].regmap = devm_regmap_init_i2c(
			priv->eeprom[i].i2c_client, &eeprom_regmap_config);
		if (IS_ERR(priv->eeprom[i].regmap)) {
			err = PTR_ERR(priv->eeprom[i].regmap);
			ov5693_eeprom_device_release(priv);
			return err;
		}
	}

	return 0;
}

static int ov5693_read_eeprom(struct ov5693 *priv)
{
	int err, i;

	for (i = 0; i < OV5693_EEPROM_NUM_BLOCKS; i++) {
		err = regmap_bulk_read(priv->eeprom[i].regmap, 0,
			&priv->eeprom_buf[i * OV5693_EEPROM_BLOCK_SIZE],
			OV5693_EEPROM_BLOCK_SIZE);
		if (err)
			return err;
	}

	return 0;
}

static int ov5693_read_otp_bank(struct ov5693 *priv,
				u8 *buf, int bank, u16 addr, int size)
{
	int err;

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor
	 */
	usleep_range(10000, 11000);
	mutex_lock(&priv->streaming_lock);
	err = ov5693_write_table(priv, mode_table[OV5693_MODE_START_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		return err;
	} else {
		priv->streaming = true;
		mutex_unlock(&priv->streaming_lock);
	}

	err = ov5693_write_reg(priv->s_data, OV5693_OTP_BANK_SELECT_ADDR,
			       0xC0 | bank);
	if (err)
		return err;

	err = ov5693_write_reg(priv->s_data, OV5693_OTP_LOAD_CTRL_ADDR, 0x01);
	if (err)
		return err;

	usleep_range(10000, 11000);
	err = regmap_bulk_read(priv->s_data->regmap, addr, buf, size);

	if (err)
		return err;

	mutex_lock(&priv->streaming_lock);
	err = ov5693_write_table(priv, mode_table[OV5693_MODE_STOP_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		return err;
	} else {
		priv->streaming = false;
		mutex_unlock(&priv->streaming_lock);
	}

	return 0;
}

static int ov5693_otp_setup(struct ov5693 *priv)
{
	struct device *dev = priv->s_data->dev;
	int err = 0;
	int i;

	for (i = 0; i < OV5693_OTP_NUM_BANKS; i++) {
		err = ov5693_read_otp_bank(priv,
					&priv->otp_buf[i
					* OV5693_OTP_BANK_SIZE],
					i,
					OV5693_OTP_BANK_START_ADDR,
					OV5693_OTP_BANK_SIZE);
		if (err) {
			dev_err(dev, "could not read otp bank\n");
			goto ret;
		}
	}

ret:
	return err;
}

static int ov5693_fuse_id_setup(struct ov5693 *priv)
{
	struct device *dev = priv->s_data->dev;
	int err;

	err = ov5693_read_otp_bank(priv,
				&priv->fuse_id[0],
				OV5693_FUSE_ID_OTP_BANK,
				OV5693_FUSE_ID_OTP_START_ADDR,
				OV5693_FUSE_ID_SIZE);
	if (err) {
		dev_err(dev, "could not read otp bank\n");
		goto ret;
	}

ret:
	return err;
}

MODULE_DEVICE_TABLE(of, ov5693_of_match);

static struct camera_common_pdata *ov5693_parse_dt(struct tegracam_device
							*tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio;
	int err;
	struct camera_common_pdata *ret = NULL;

	if (!node)
		return NULL;

	match = of_match_device(ov5693_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(dev,
					 board_priv_pdata);
	if (err) {
		dev_err(dev, "Failed to find clocks\n");
		goto error;
	}

	gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER) {
			ret = ERR_PTR(-EPROBE_DEFER);
			goto error;
		}
		gpio = 0;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (gpio < 0) {
		/* reset-gpio is not absolutely needed */
		if (gpio == -EPROBE_DEFER) {
			ret = ERR_PTR(-EPROBE_DEFER);
			goto error;
		}
		dev_dbg(dev, "reset gpios not in DT\n");
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	board_priv_pdata->use_cam_gpio =
		of_property_read_bool(node, "cam, use-cam-gpio");

	err = of_property_read_string(node, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	if (err) {
		dev_err(dev, "avdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(node, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(dev, "iovdd-reg not in DT\n");
		goto error;
	}

	board_priv_pdata->has_eeprom =
		of_property_read_bool(node, "has-eeprom");
	board_priv_pdata->v_flip = of_property_read_bool(node, "vertical-flip");
	board_priv_pdata->h_mirror = of_property_read_bool(node,
							 "horizontal-mirror");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);
	return ret;
}

static int ov5693_set_mode(struct tegracam_device *tc_dev)
{
	struct ov5693 *priv = (struct ov5693 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err;

	err = ov5693_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static int ov5693_start_streaming(struct tegracam_device *tc_dev)
{
	struct ov5693 *priv = (struct ov5693 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	int err;
	u8 val;

	mutex_lock(&priv->streaming_lock);
	err = ov5693_write_table(priv, mode_table[OV5693_MODE_START_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = true;
		mutex_unlock(&priv->streaming_lock);
	}
	if (pdata->v_flip) {
		ov5693_read_reg(s_data, OV5693_TIMING_REG20, &val);
		ov5693_write_reg(s_data, OV5693_TIMING_REG20,
				 val | VERTICAL_FLIP);
	}
	if (pdata->h_mirror) {
		ov5693_read_reg(s_data, OV5693_TIMING_REG21, &val);
		ov5693_write_reg(s_data, OV5693_TIMING_REG21,
				 val | HORIZONTAL_MIRROR_MASK);
	} else {
		ov5693_read_reg(s_data, OV5693_TIMING_REG21, &val);
		ov5693_write_reg(s_data, OV5693_TIMING_REG21,
				 val & (~HORIZONTAL_MIRROR_MASK));
	}
	if (test_mode)
		err = ov5693_write_table(priv,
			mode_table[OV5693_MODE_TEST_PATTERN]);

	return 0;

exit:
	dev_err(dev, "%s: error starting stream\n", __func__);
	return err;
}

static int ov5693_stop_streaming(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5693 *priv = (struct ov5693 *)tegracam_get_privdata(tc_dev);
	struct device *dev = s_data->dev;
	u32 frame_time;
	int err;

	mutex_lock(&priv->streaming_lock);
	err = ov5693_write_table(priv,
		mode_table[OV5693_MODE_STOP_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = false;
		mutex_unlock(&priv->streaming_lock);
	}

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * frame_time = frame length rows * Tline
	 * Tline = line length / pixel clock (in MHz)
	 */
	frame_time = priv->frame_length *
		OV5693_DEFAULT_LINE_LENGTH / OV5693_DEFAULT_PIXEL_CLOCK;

	usleep_range(frame_time, frame_time + 1000);

	return 0;

exit:
	dev_err(dev, "%s: error stopping stream\n", __func__);
	return err;
}

static struct camera_common_sensor_ops ov5693_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ov5693_frmfmt),
	.frmfmt_table = ov5693_frmfmt,
	.power_on = ov5693_power_on,
	.power_off = ov5693_power_off,
	.write_reg = ov5693_write_reg,
	.read_reg = ov5693_read_reg,
	.parse_dt = ov5693_parse_dt,
	.power_get = ov5693_power_get,
	.power_put = ov5693_power_put,
	.set_mode = ov5693_set_mode,
	.start_streaming = ov5693_start_streaming,
	.stop_streaming = ov5693_stop_streaming,
};

static int ov5693_debugfs_streaming_show(void *data, u64 *val)
{
	struct ov5693 *priv = data;

	mutex_lock(&priv->streaming_lock);
	*val = priv->streaming;
	mutex_unlock(&priv->streaming_lock);

	return 0;
}

static int ov5693_debugfs_streaming_write(void *data, u64 val)
{
	int err = 0;
	struct ov5693 *priv = data;
	struct i2c_client *client = priv->i2c_client;
	bool enable = (val != 0);
	int mode_index = enable ?
		(OV5693_MODE_START_STREAM) : (OV5693_MODE_STOP_STREAM);

	dev_info(&client->dev, "%s: %s sensor\n",
			__func__, (enable ? "enabling" : "disabling"));

	mutex_lock(&priv->streaming_lock);

	err = ov5693_write_table(priv, mode_table[mode_index]);
	if (err) {
		dev_err(&client->dev, "%s: error setting sensor streaming\n",
			__func__);
		goto done;
	}

	priv->streaming = enable;

done:
	mutex_unlock(&priv->streaming_lock);

	return err;
}

DEFINE_SIMPLE_ATTRIBUTE(ov5693_debugfs_streaming_fops,
	ov5693_debugfs_streaming_show,
	ov5693_debugfs_streaming_write,
	"%lld\n");

static void ov5693_debugfs_remove(struct ov5693 *priv);

static int ov5693_debugfs_create(struct ov5693 *priv)
{
	int err = 0;
	struct i2c_client *client = priv->i2c_client;
	const char *devnode;
	char debugfs_dir[16];

	err = of_property_read_string(client->dev.of_node, "devnode", &devnode);
	if (err) {
		dev_err(&client->dev, "devnode not in DT\n");
		return err;
	}
	snprintf(debugfs_dir, sizeof(debugfs_dir), "camera-%s", devnode);

	priv->debugfs_dir = debugfs_create_dir(debugfs_dir, NULL);
	if (priv->debugfs_dir == NULL)
		return -ENOMEM;

	if (!debugfs_create_file("streaming", 0644, priv->debugfs_dir, priv,
			&ov5693_debugfs_streaming_fops))
		goto error;

	return 0;

error:
	ov5693_debugfs_remove(priv);

	return -ENOMEM;
}

static struct tegracam_ctrl_ops ov5693_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = {OV5693_EEPROM_STR_SIZE,
				OV5693_FUSE_ID_STR_SIZE,
				OV5693_OTP_STR_SIZE},
	.set_gain = ov5693_set_gain,
	.set_exposure = ov5693_set_exposure,
	.set_exposure_short = ov5693_set_exposure_short,
	.set_frame_rate = ov5693_set_frame_rate,
	.set_group_hold = ov5693_set_group_hold,
	.fill_string_ctrl = ov5693_fill_string_ctrl,
};

static int ov5693_board_setup(struct ov5693 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	bool eeprom_ctrl = 0;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);

	/* eeprom interface */
	err = ov5693_eeprom_device_init(priv);
	if (err && s_data->pdata->has_eeprom)
		dev_err(dev,
			"Failed to allocate eeprom reg map: %d\n", err);
	eeprom_ctrl = !err;

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = ov5693_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

	if (eeprom_ctrl) {
		err = ov5693_read_eeprom(priv);
		if (err) {
			dev_err(dev,
				"Error %d reading eeprom\n", err);
			goto error;
		}
	}

	err = ov5693_otp_setup(priv);
	if (err) {
		dev_err(dev,
			"Error %d reading otp data\n", err);
		goto error;
	}

	err = ov5693_fuse_id_setup(priv);
	if (err) {
		dev_err(dev,
			"Error %d reading fuse id data\n", err);
		goto error;
	}

error:
	ov5693_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

static void ov5693_debugfs_remove(struct ov5693 *priv)
{
	debugfs_remove_recursive(priv->debugfs_dir);
	priv->debugfs_dir = NULL;
}

static int ov5693_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops ov5693_subdev_internal_ops = {
	.open = ov5693_open,
};

static int ov5693_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tegracam_device *tc_dev;
	struct ov5693 *priv;
	int err;
	const struct of_device_id *match;

	dev_info(dev, "probing v4l2 sensor.\n");

	match = of_match_device(ov5693_of_match, dev);
	if (!match) {
		dev_err(dev, "No device match found\n");
		return -ENODEV;
	}

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			    sizeof(struct ov5693), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			    sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ov5693", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &ov5693_regmap_config;
	tc_dev->sensor_ops = &ov5693_common_ops;
	tc_dev->v4l2sd_internal_ops = &ov5693_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ov5693_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}

	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);
	mutex_init(&priv->streaming_lock);

	err = ov5693_board_setup(priv);
		if (err) {
			tegracam_device_unregister(tc_dev);
			dev_err(dev, "board setup failed\n");
			return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	err = ov5693_debugfs_create(priv);
	if (err) {
		dev_err(dev, "error creating debugfs interface");
		ov5693_debugfs_remove(priv);
		return err;
	}

	dev_dbg(dev, "Detected OV5693 sensor\n");

	return 0;
}

static int
ov5693_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov5693 *priv = (struct ov5693 *)s_data->priv;

	ov5693_debugfs_remove(priv);

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	ov5693_power_put(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);
	ov5693_eeprom_device_release(priv);

	mutex_destroy(&priv->streaming_lock);

	return 0;
}

static const struct i2c_device_id ov5693_id[] = {
	{ "ov5693", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov5693_id);

static struct i2c_driver ov5693_i2c_driver = {
	.driver = {
		.name = "ov5693",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov5693_of_match),
	},
	.probe = ov5693_probe,
	.remove = ov5693_remove,
	.id_table = ov5693_id,
};
module_i2c_driver(ov5693_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for OmniVision OV5693");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
