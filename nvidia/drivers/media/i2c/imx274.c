/*
 * imx274.c - imx274 sensor driver
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/imx274.h>

#include "imx274_mode_tbls.h"

#define IMX274_GAIN_FACTOR		1000000
#define IMX274_MIN_GAIN			(1 * IMX274_GAIN_FACTOR)
#define IMX274_MAX_ANALOG_GAIN	(IMX274_MIN_GAIN * 111 / 5)
#define IMX274_MAX_DIGITAL_GAIN	64
#define IMX274_MAX_GAIN	(IMX274_MAX_ANALOG_GAIN * IMX274_MAX_DIGITAL_GAIN)

#define IMX274_SENSOR_INTERNAL_CLK_FREQ	72000000
#define IMX274_4K_MODE_HMAX			263
#define IMX274_4K_MODE_MIN_VMAX		4550
#define IMX274_4K_MODE_OFFSET		112
#define IMX274_DOL_MODE_CLOCKS_OFFSET	112
#define IMX274_DOL_4K_MODE_HMAX	1052
#define IMX274_DOL_4K_MODE_MIN_VMAX	2284
#define IMX274_DOL_4K_MODE_DEFAULT_RHS1	50
#define IMX274_DOL_4K_MIN_SHR_DOL1	6
#define IMX274_DOL_1080P_MODE_HMAX	1040
#define IMX274_DOL_1080P_MODE_MIN_VMAX	1155
#define IMX274_DOL_1080P_MODE_DEFAULT_RHS1	38
#define IMX274_DOL_1080P_MIN_SHR_DOL1	4
#define IMX274_1080P_MODE_HMAX			260
#define IMX274_1080P_MODE_MIN_VMAX		4620
#define IMX274_1080P_MODE_OFFSET		112

#define IMX274_EEPROM_ADDRESS		0x57
#define IMX274_EEPROM_SIZE			256
#define IMX274_EEPROM_STR_SIZE		(IMX274_EEPROM_SIZE * 2)
#define IMX274_EEPROM_BLOCK_SIZE	(1 << 8)
#define IMX274_EEPROM_NUM_BLOCKS \
	(IMX274_EEPROM_SIZE / IMX274_EEPROM_BLOCK_SIZE)

#define IMX274_FUSE_ID_START_ADDR	91
#define IMX274_FUSE_ID_SIZE		8
#define IMX274_FUSE_ID_STR_SIZE		(IMX274_FUSE_ID_SIZE * 2)

static const struct of_device_id imx274_of_match[] = {
	{ .compatible = "nvidia,imx274", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx274_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_GROUP_HOLD,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_FUSE_ID,
};

struct imx274 {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;

	const char			*devname;
	struct dentry			*debugfs_dir;
	struct mutex			streaming_lock;
	bool				streaming;

	struct camera_common_eeprom_data eeprom[IMX274_EEPROM_NUM_BLOCKS];
	u8				eeprom_buf[IMX274_EEPROM_STR_SIZE];
	u8				fuse_id[IMX274_FUSE_ID_SIZE];
	u32				frame_length;
	u32				vmax;
	s64				last_exposure_long;
	s64				last_exposure_short;
	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static inline void imx274_get_vmax_regs(imx274_reg *regs,
				u32 vmax)
{
	regs->addr = IMX274_VMAX_ADDR_MSB;
	regs->val = (vmax >> 16) & 0x0f;
	(regs + 1)->addr = IMX274_VMAX_ADDR_MID;
	(regs + 1)->val = (vmax >> 8) & 0xff;
	(regs + 2)->addr = IMX274_VMAX_ADDR_LSB;
	(regs + 2)->val = (vmax) & 0xff;
}

static inline void imx274_get_shr_regs(imx274_reg *regs,
				u16 shr)
{
	regs->addr = IMX274_SHR_ADDR_MSB;
	regs->val = (shr >> 8) & 0xff;
	(regs + 1)->addr = IMX274_SHR_ADDR_LSB;
	(regs + 1)->val = (shr) & 0xff;
}

static inline void imx274_get_shr_dol1_regs(imx274_reg *regs,
				u16 shr)
{
	regs->addr = IMX274_SHR_DOL1_ADDR_MSB;
	regs->val = (shr >> 8) & 0xff;
	(regs + 1)->addr = IMX274_SHR_DOL1_ADDR_LSB;
	(regs + 1)->val = (shr) & 0xff;
}

static inline void imx274_get_shr_dol2_regs(imx274_reg *regs,
				u16 shr)
{
	regs->addr = IMX274_SHR_DOL2_ADDR_MSB;
	regs->val = (shr >> 8) & 0xff;
	(regs + 1)->addr = IMX274_SHR_DOL2_ADDR_LSB;
	(regs + 1)->val = (shr) & 0xff;
}

static inline void imx274_get_gain_reg(imx274_reg *regs,
				u16 gain)
{
	regs->addr = IMX274_ANALOG_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = IMX274_ANALOG_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx274_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx274_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx274_write_table(struct imx274 *priv,
				const imx274_reg table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap,
					 table,
					 NULL, 0,
					 IMX274_TABLE_WAIT_MS,
					 IMX274_TABLE_END);
}

static int imx274_set_gain(struct tegracam_device *tc_dev, s64 val);
static int imx274_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int imx274_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int imx274_set_exposure_shr(struct tegracam_device *tc_dev, s64 val);
static int imx274_set_exposure_shr_dol_short(struct tegracam_device *tc_dev,
			s64 val);
static int imx274_set_exposure_shr_dol_long(struct tegracam_device *tc_dev,
			s64 val);
static bool imx274_in_dol_mode(const struct imx274 *priv);

static int imx274_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err;

	priv->group_hold_prev = val;
	err = imx274_write_reg(s_data,
				IMX274_GROUP_HOLD_ADDR, val);
	if (err) {
		dev_err(dev,
			"%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int imx274_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	imx274_reg reg_list[2];
	int err;
	int i = 0;
	u32 again;
	u8 dgain;
	u16 reg_again;
	u8 reg_dgain;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	if (val < IMX274_MIN_GAIN)
		val = IMX274_MIN_GAIN;
	else if (val > IMX274_MAX_GAIN)
		val = IMX274_MAX_GAIN;

	if  (val > (IMX274_MAX_ANALOG_GAIN * 32)) {
		dgain = 64;
		reg_dgain = 0x06;
	} else if  (val > (IMX274_MAX_ANALOG_GAIN * 16)) {
		dgain = 32;
		reg_dgain = 0x05;
	} else if  (val > (IMX274_MAX_ANALOG_GAIN * 8)) {
		dgain = 16;
		reg_dgain = 0x04;
	} else if  (val > (IMX274_MAX_ANALOG_GAIN * 4)) {
		dgain = 8;
		reg_dgain = 0x03;
	} else if  (val > (IMX274_MAX_ANALOG_GAIN * 2)) {
		dgain = 4;
		reg_dgain = 0x02;
	} else if (val > (IMX274_MAX_ANALOG_GAIN)) {
		dgain = 2;
		reg_dgain = 0x01;
	} else  {
		dgain = 1;
		reg_dgain = 0x00;
	}

	reg_again = 2048 -
		(2048 * dgain * mode->control_properties.gain_factor / val);
	if (reg_again > 1957)
		reg_again = 1957;
	again = val / (dgain * mode->control_properties.gain_factor);

	imx274_get_gain_reg(reg_list, reg_again);

	dev_dbg(dev, "%s: val:%lld, gain:%lld, again:(%d, %d), dgain:(%d, %d)\n",
			__func__,
			val,
			val / IMX274_MIN_GAIN,
			again,
			reg_again,
			dgain,
			reg_dgain);

	/* writing analog gain */
	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	/* writing digital gain */
	err = imx274_write_reg(priv->s_data, IMX274_DIGITAL_GAIN_ADDR,
				reg_dgain);
	if (err)
		goto fail;

	return 0;

fail:
	dev_err(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static
int imx274_set_exposure_shr_dol_short(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	struct v4l2_control control;
	int hdr_en;
	imx274_reg reg_list[2];
	u16 rhs1, hmax;
	s32 shr_dol1, min_shr;
	int err;
	int i = 0;
	u64 freq = IMX274_SENSOR_INTERNAL_CLK_FREQ;

	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if (hdr_en != SWITCH_ON)  {
		dev_dbg(dev, "%s: SHR DOL1 is ignored for non-HDR mode\n",
			__func__);
		return 0;
	}

	if (s_data->mode == IMX274_MODE_3840X2160_DOL_30FPS) {
		rhs1 = IMX274_DOL_4K_MODE_DEFAULT_RHS1;
		min_shr = IMX274_DOL_4K_MIN_SHR_DOL1;
		hmax = IMX274_DOL_4K_MODE_HMAX;
	} else if (s_data->mode == IMX274_MODE_1920X1080_DOL_60FPS) {
		rhs1 = IMX274_DOL_1080P_MODE_DEFAULT_RHS1;
		min_shr = IMX274_DOL_1080P_MIN_SHR_DOL1;
		hmax = IMX274_DOL_1080P_MODE_HMAX;
	} else {
		dev_err(dev, "%s: error, invalid dol mode\n", __func__);
		err = -EINVAL;
		goto fail;
	}


	priv->last_exposure_short = val;
	shr_dol1 = rhs1 -
		(val * freq /
		mode->control_properties.exposure_factor  -
		IMX274_DOL_MODE_CLOCKS_OFFSET) /
		hmax - 1 / 4;

	if (shr_dol1 <= min_shr)
		shr_dol1 = min_shr;

	if (shr_dol1 > rhs1 - 2)
		shr_dol1 = rhs1 - 2;

	dev_dbg(dev,
		 "%s: et:%d, shr_dol1:%d, rhs1:%d, Pclk:%lld, LL:%d, HMAX:%d\n",
		 __func__,
		(int)(val * 1000000 / mode->control_properties.exposure_factor),
		shr_dol1,
		rhs1,
		mode->signal_properties.pixel_clock.val,
		mode->image_properties.line_length,
		hmax);

	imx274_get_shr_dol1_regs(reg_list, (u16)shr_dol1);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}
	return 0;

fail:
	dev_err(dev, "%s: EXPOSURE short control is not set\n", __func__);
	return err;
}

static
int imx274_set_exposure_shr_dol_long(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	imx274_reg reg_list[2];
	u16 hmax, rhs1;
	s32 shr_dol2, min_shr;
	int err;
	int i = 0;
	u64 freq = IMX274_SENSOR_INTERNAL_CLK_FREQ;

	if (s_data->mode == IMX274_MODE_3840X2160_DOL_30FPS) {
		rhs1 = IMX274_DOL_4K_MODE_DEFAULT_RHS1;
		min_shr = IMX274_DOL_4K_MIN_SHR_DOL1;
		hmax = IMX274_DOL_4K_MODE_HMAX;
	} else if (s_data->mode == IMX274_MODE_1920X1080_DOL_60FPS) {
		rhs1 = IMX274_DOL_1080P_MODE_DEFAULT_RHS1;
		min_shr = IMX274_DOL_1080P_MIN_SHR_DOL1;
		hmax = IMX274_DOL_1080P_MODE_HMAX;
	} else {
		dev_err(dev, "%s: error, invalid dol mode\n", __func__);
		err = -EINVAL;
		goto fail;
	}

	priv->last_exposure_long = val;

	shr_dol2 = priv->vmax  -
		(val * freq /
		mode->control_properties.exposure_factor -
		IMX274_DOL_MODE_CLOCKS_OFFSET) /
		hmax - 1 / 4;

	if (shr_dol2 < rhs1 + min_shr)
		shr_dol2 = rhs1 + min_shr;

	if (shr_dol2 > priv->vmax - 4)
		shr_dol2 = priv->vmax - 4;

	dev_dbg(dev,
		 "%s: et:%d, shr_dol2:%d, vmax:%d, Pclk:%lld, LL:%d, HMAX:%d\n",
		 __func__,
		(int)(val * 1000000 / mode->control_properties.exposure_factor),
		shr_dol2,
		priv->vmax,
		mode->signal_properties.pixel_clock.val,
		mode->image_properties.line_length,
		hmax);

	imx274_get_shr_dol2_regs(reg_list, (u16)shr_dol2);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}
	return 0;

fail:
	dev_err(dev, "%s: EXPOSURE_SHORT control error\n", __func__);
	return err;
}

static bool imx274_in_dol_mode(const struct imx274 *priv)
{
	const struct camera_common_data *s_data = priv->s_data;

	switch (s_data->mode) {
	case IMX274_MODE_3840X2160_DOL_30FPS:
	case IMX274_MODE_1920X1080_DOL_60FPS:
		return true;
	default:
		return false;
	}
}

static int imx274_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	struct v4l2_control control;
	int hdr_en;
	imx274_reg reg_list[3];
	int err;
	int i = 0;
	u8 svr;
	u64 freq = IMX274_SENSOR_INTERNAL_CLK_FREQ;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	imx274_read_reg(priv->s_data, IMX274_SVR_ADDR, &svr);

	if (s_data->mode == IMX274_MODE_3840X2160_DOL_30FPS) {
		priv->vmax = (u32)(freq *
				mode->control_properties.framerate_factor /
				(val *
				IMX274_DOL_4K_MODE_HMAX));
		if (priv->vmax < IMX274_DOL_4K_MODE_MIN_VMAX)
			priv->vmax = IMX274_DOL_4K_MODE_MIN_VMAX;
	} else if (s_data->mode == IMX274_MODE_1920X1080_DOL_60FPS) {
		priv->vmax = (u32)(freq *
				mode->control_properties.framerate_factor /
				(val *
				IMX274_DOL_1080P_MODE_HMAX));
		if (priv->vmax < IMX274_DOL_1080P_MODE_MIN_VMAX)
			priv->vmax = IMX274_DOL_1080P_MODE_MIN_VMAX;
	} else if (s_data->mode == IMX274_MODE_1920X1080) {
		priv->vmax = (u32)(freq *
				mode->control_properties.framerate_factor /
				(val *
				IMX274_1080P_MODE_HMAX));
		if (priv->vmax < IMX274_1080P_MODE_MIN_VMAX)
			priv->vmax = IMX274_1080P_MODE_MIN_VMAX;
	} else {
		priv->vmax = (u32)(freq *
				mode->control_properties.framerate_factor /
				(val *
				IMX274_4K_MODE_HMAX));
		if (priv->vmax < IMX274_4K_MODE_MIN_VMAX)
			priv->vmax = IMX274_4K_MODE_MIN_VMAX;
	}

	imx274_get_vmax_regs(reg_list, priv->vmax);

	for (i = 0; i < 3; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	dev_dbg(dev, "%s: PCLK:%lld, LL:%d, fps:%lld, VMAX:%d\n", __func__,
			mode->signal_properties.pixel_clock.val,
			mode->image_properties.line_length,
			val / mode->control_properties.framerate_factor,
			priv->vmax);

	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if ((hdr_en == SWITCH_ON && imx274_in_dol_mode(priv))
		&& (priv->last_exposure_long != 0)) {
		err = imx274_set_exposure_shr_dol_long(tc_dev,
					priv->last_exposure_long);
		if (err)
			dev_err(dev, "%s: error exposure time dol long\n",
				__func__);

		err = imx274_set_exposure_shr_dol_short(tc_dev,
					priv->last_exposure_short);
		if (err)
			dev_err(dev, "%s: error exposure time dol short\n",
				__func__);
	}

	return 0;

fail:
	dev_err(dev, "%s: FRAME_RATE control error\n", __func__);
	return err;
}

static
u16 imx274_calculate_exposure_shr(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	u8 svr;
	u16 shr;
	u64 freq = IMX274_SENSOR_INTERNAL_CLK_FREQ;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	imx274_read_reg(priv->s_data, IMX274_SVR_ADDR, &svr);

	if (s_data->mode == IMX274_MODE_1920X1080) {
		shr = priv->vmax  -
			(u32) (val * freq /
			mode->control_properties.exposure_factor -
			IMX274_1080P_MODE_OFFSET) /
			IMX274_1080P_MODE_HMAX;

		if (shr > priv->vmax - 4)
			shr = priv->vmax - 4;
		if (shr < 8)
			shr = 8;
	} else {

		shr = priv->vmax  -
			(u32) (val  * freq /
			mode->control_properties.exposure_factor -
			IMX274_4K_MODE_OFFSET) /
			IMX274_4K_MODE_HMAX;

		if (shr < 12)
			shr = 12;

		if (shr > priv->vmax - 4)
			shr = priv->vmax - 4;
	}

	dev_dbg(dev, "%s: shr: %u vmax: %d\n", __func__, shr, priv->vmax);
	return shr;
}

static int imx274_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	struct v4l2_control control;
	int hdr_en;
	int err;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];

	if (hdr_en == SWITCH_ON && imx274_in_dol_mode(priv)) {
		err = imx274_set_exposure_shr_dol_long(tc_dev, val);
		if (err)
			dev_err(dev,
			"%s: error exposure time dol long override\n", __func__);
	} else {
		err = imx274_set_exposure_shr(tc_dev, val);
		if (err)
			dev_err(dev,
			"%s: error exposure time SHR override\n", __func__);
	}
	return err;
}

static int imx274_set_exposure_shr(struct tegracam_device *tc_dev, s64 val)
{
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	imx274_reg reg_list[2];
	int err;
	u16 shr;
	int i = 0;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	shr = imx274_calculate_exposure_shr(tc_dev, val);

	imx274_get_shr_regs(reg_list, shr);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "%s: Exposure control error\n", __func__);
	return err;
}

static int imx274_fill_string_ctrl(struct tegracam_device *tc_dev,
				struct v4l2_ctrl *ctrl)
{
	struct imx274 *priv = (struct imx274 *)tc_dev->priv;
	int i;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_FUSE_ID:
		for (i = 0; i < IMX274_FUSE_ID_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->fuse_id[i]);
		break;
	default:
		return -EINVAL;
	}

	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static struct tegracam_ctrl_ops imx274_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = {0, IMX274_FUSE_ID_STR_SIZE},
	.set_gain = imx274_set_gain,
	.set_exposure = imx274_set_exposure,
	.set_exposure_short = imx274_set_exposure_shr_dol_short,
	.set_frame_rate = imx274_set_frame_rate,
	.set_group_hold = imx274_set_group_hold,
	.fill_string_ctrl = imx274_fill_string_ctrl,
};

static int imx274_power_on(struct camera_common_data *s_data)
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

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);
	usleep_range(10, 20);

	if (pw->dvdd)
		err = regulator_enable(pw->dvdd);
	if (err)
		goto imx274_dvdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto imx274_iovdd_fail;

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto imx274_avdd_fail;

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 1);

	/* 1.2v input is generated on module board, adds more latency */
	usleep_range(10000, 10010);

	pw->state = SWITCH_ON;
	return 0;

imx274_dvdd_fail:
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 0);

imx274_iovdd_fail:
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

imx274_avdd_fail:
	if (pw->iovdd)
		regulator_disable(pw->iovdd);

	dev_err(dev, "%s failed.\n", __func__);
	return -ENODEV;
}

static int imx274_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
		goto power_off_done;
	}

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 0);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);
	usleep_range(1, 2);

	if (pw->avdd)
		regulator_disable(pw->avdd);
	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int imx274_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);
	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);
	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->dvdd = NULL;
	pw->iovdd = NULL;

	gpio_free(pw->pwdn_gpio);
	gpio_free(pw->reset_gpio);
	gpio_free(pw->af_gpio);

	return 0;
}

static int imx274_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

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

	/* ananlog 2.7v */
	err |= camera_common_regulator_get(dev,
			&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(dev,
			&pw->iovdd, pdata->regulators.iovdd);
	/* digital 1.2v, not all imx274 modules draw this from CVB */
	if (pdata->regulators.dvdd != NULL)
		err |= camera_common_regulator_get(dev,
			&pw->dvdd, pdata->regulators.dvdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->af_gpio = pdata->af_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

	pw->state = SWITCH_OFF;
	return err;
}

static
struct camera_common_pdata *imx274_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(imx274_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	board_priv_pdata->reset_gpio = of_get_named_gpio(node,
			"reset-gpios", 0);

	of_property_read_string(node, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	of_property_read_string(node, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);
	of_property_read_string(node, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);

	board_priv_pdata->has_eeprom =
		of_property_read_bool(node, "has-eeprom");

	of_property_read_u32(node, "fuse_id_start_addr",
		&board_priv_pdata->fuse_id_addr);

	return board_priv_pdata;
}


static int imx274_set_mode(struct tegracam_device *tc_dev)
{
	struct imx274 *priv = (struct imx274 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	int err;

	err = imx274_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	if (test_mode) {
		err = imx274_write_table(priv,
			mode_table[IMX274_MODE_TEST_PATTERN]);
		if (err)
			goto exit;
	}

	return 0;

exit:
	dev_err(dev, "%s: error setting mode\n", __func__);
	return err;
}

static int imx274_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx274 *priv = (struct imx274 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	int err;

	mutex_lock(&priv->streaming_lock);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = true;
		mutex_unlock(&priv->streaming_lock);
	}

	return 0;

exit:
	dev_err(dev, "%s: error starting stream\n", __func__);
	return err;
}

static int imx274_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx274 *priv = (struct imx274 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	int err;

	mutex_lock(&priv->streaming_lock);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_STOP_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else  {
		priv->streaming = false;
		mutex_unlock(&priv->streaming_lock);
	}

	return 0;

exit:
	dev_err(dev, "%s: error stopping stream\n", __func__);
	return err;
}

static struct camera_common_sensor_ops imx274_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx274_frmfmt),
	.frmfmt_table = imx274_frmfmt,
	.power_on = imx274_power_on,
	.power_off = imx274_power_off,
	.write_reg = imx274_write_reg,
	.read_reg = imx274_read_reg,
	.parse_dt = imx274_parse_dt,
	.power_get = imx274_power_get,
	.power_put = imx274_power_put,
	.set_mode = imx274_set_mode,
	.start_streaming = imx274_start_streaming,
	.stop_streaming = imx274_stop_streaming,
};

static int imx274_eeprom_device_release(struct imx274 *priv)
{
	int i;

	for (i = 0; i < IMX274_EEPROM_NUM_BLOCKS; i++) {
		if (priv->eeprom[i].i2c_client != NULL) {
			i2c_unregister_device(priv->eeprom[i].i2c_client);
			priv->eeprom[i].i2c_client = NULL;
		}
	}

	return 0;
}

static int imx274_eeprom_device_init(struct imx274 *priv)
{
	char *dev_name = "eeprom_imx274";
	struct camera_common_pdata *pdata = priv->s_data->pdata;
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int i;
	int err;

	if (!pdata || !pdata->has_eeprom)
		return -EINVAL;

	for (i = 0; i < IMX274_EEPROM_NUM_BLOCKS; i++) {
		priv->eeprom[i].adap = i2c_get_adapter(
				priv->i2c_client->adapter->nr);
		memset(&priv->eeprom[i].brd, 0, sizeof(priv->eeprom[i].brd));
		strncpy(priv->eeprom[i].brd.type, dev_name,
				sizeof(priv->eeprom[i].brd.type));
		priv->eeprom[i].brd.addr = IMX274_EEPROM_ADDRESS + i;
		priv->eeprom[i].i2c_client = i2c_new_device(
				priv->eeprom[i].adap, &priv->eeprom[i].brd);

		priv->eeprom[i].regmap = devm_regmap_init_i2c(
			priv->eeprom[i].i2c_client, &eeprom_regmap_config);
		if (IS_ERR(priv->eeprom[i].regmap)) {
			err = PTR_ERR(priv->eeprom[i].regmap);
			imx274_eeprom_device_release(priv);
			return err;
		}
	}

	return 0;
}

static int imx274_read_fuse_id(struct imx274 *priv)
{
	struct camera_common_pdata *pdata = priv->s_data->pdata;
	int err, i;

	if (!pdata->has_eeprom)
		return -EINVAL;

	for (i = 0; i < IMX274_EEPROM_NUM_BLOCKS; i++) {
		err = regmap_bulk_read(priv->eeprom[i].regmap, 0,
			&priv->eeprom_buf[i * IMX274_EEPROM_BLOCK_SIZE],
			IMX274_EEPROM_BLOCK_SIZE);
		if (err)
			return err;
	}

	for (i = 0; i < IMX274_FUSE_ID_SIZE; i++)
		priv->fuse_id[i] =
			priv->eeprom_buf[i + pdata->fuse_id_addr];

	return 0;
}

static int imx274_board_setup(struct imx274 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	if (!s_data->pdata->has_eeprom)
		return 0;

	/* eeprom for fuse id*/
	err = imx274_eeprom_device_init(priv);
	if (err && s_data->pdata->has_eeprom) {
		dev_err(dev,
			"Failed to allocate eeprom reg map: %d\n", err);
		return err;
	}

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = imx274_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

	err = imx274_read_fuse_id(priv);
	if (err) {
		dev_err(dev,
			"Error %d reading fuse id data\n", err);
		goto error;
	}

error:
	imx274_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

static int imx274_debugfs_streaming_show(void *data, u64 *val)
{
	struct imx274 *priv = data;

	mutex_lock(&priv->streaming_lock);
	*val = priv->streaming;
	mutex_unlock(&priv->streaming_lock);

	return 0;
}

static int imx274_debugfs_streaming_write(void *data, u64 val)
{
	int err = 0;
	struct imx274 *priv = data;
	struct i2c_client *client = priv->i2c_client;
	bool enable = (val != 0);
	int mode_index = enable ?
		(IMX274_MODE_START_STREAM) : (IMX274_MODE_STOP_STREAM);

	dev_info(&client->dev, "%s: %s sensor\n",
			__func__, (enable ? "enabling" : "disabling"));

	mutex_lock(&priv->streaming_lock);

	err = imx274_write_table(priv, mode_table[mode_index]);
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

DEFINE_SIMPLE_ATTRIBUTE(imx274_debugfs_streaming_fops,
	imx274_debugfs_streaming_show,
	imx274_debugfs_streaming_write,
	"%lld\n");

static void imx274_debugfs_remove(struct imx274 *priv);

static int imx274_debugfs_create(struct imx274 *priv)
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
			&imx274_debugfs_streaming_fops))
		goto error;

	return 0;

error:
	imx274_debugfs_remove(priv);

	return -ENOMEM;
}

static void imx274_debugfs_remove(struct imx274 *priv)
{
	debugfs_remove_recursive(priv->debugfs_dir);
	priv->debugfs_dir = NULL;
}

static int imx274_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx274_subdev_internal_ops = {
	.open = imx274_open,
};

static int imx274_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tegracam_device *tc_dev;
	struct imx274 *priv;
	int err;

	dev_info(dev, "probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx274), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx274", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx274_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx274_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx274_ctrl_ops;

	mutex_init(&priv->streaming_lock);

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx274_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	err = imx274_debugfs_create(priv);
	if (err) {
		dev_err(&client->dev, "error creating debugfs interface");
		imx274_debugfs_remove(priv);
		return err;
	}

	dev_info(dev, "Detected IMX274 sensor\n");

	return 0;
}

static int imx274_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx274 *priv = (struct imx274 *)s_data->priv;

	imx274_debugfs_remove(priv);

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	mutex_destroy(&priv->streaming_lock);

	return 0;
}

static const struct i2c_device_id imx274_id[] = {
	{ "imx274", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx274_id);

static struct i2c_driver imx274_i2c_driver = {
	.driver = {
		.name = "imx274",
		.owner = THIS_MODULE,
	},
	.probe = imx274_probe,
	.remove = imx274_remove,
	.id_table = imx274_id,
};

module_i2c_driver(imx274_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX274");
MODULE_AUTHOR("Josh Kuo <joshk@nvidia.com>");
MODULE_LICENSE("GPL v2");
