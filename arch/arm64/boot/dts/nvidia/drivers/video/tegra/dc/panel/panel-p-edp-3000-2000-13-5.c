/*
 * panel-p-edp-3000-2000-13-5.c: Panel driver for p-3000-2000-13-5 panel.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include "../dc.h"
#include "board.h"
#include "board-panel.h"
#include "gpio-names.h"

static bool reg_requested;

static struct regulator *vdd_lcd_bl_en; /* VDD_LCD_BL_EN */
static struct regulator *vdd_ds_1v8; /* VDD_1V8_AON */
static struct regulator *avdd_lcd; /* VDD_LCD_HV */

/*
 * In platform dts side, followings need to be defined
 * for this panel, additionally.
 * - nvidia,hpd-gpio in panel node
 * - pwm-gpio in backlight node
 */

static int edp_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	vdd_ds_1v8 = regulator_get(dev, "dvdd_lcd");

	if (IS_ERR(vdd_ds_1v8)) {
		pr_err("vdd_ds_1v8 regulator get failed\n");
		err = PTR_ERR(vdd_ds_1v8);
		vdd_ds_1v8 = NULL;
		goto fail;
	}

	/* backlight */
	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}

	/* lcd */
	avdd_lcd = regulator_get(dev, "avdd_lcd");
	if (IS_ERR(avdd_lcd)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd);
		avdd_lcd = NULL;
		goto fail;
	}

	reg_requested = true;
	return 0;
fail:
	return err;
}

static int edp_p_3000_2000_13_5_enable(struct device *dev)
{
	int err = 0;

	err = edp_regulator_get(dev);
	if (err < 0) {
		pr_err("edp regulator get failed\n");
		goto fail;
	}

	if (vdd_ds_1v8) {
		err = regulator_enable(vdd_ds_1v8);
		if (err < 0) {
			pr_err("vdd_ds_1v8 regulator enable failed\n");
			goto fail;
		}
	}

	if (avdd_lcd) {
		err = regulator_enable(avdd_lcd);
		if (err < 0) {
			pr_err("avdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	msleep(110);

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}

	return 0;
fail:
	return err;
}

static int edp_p_3000_2000_13_5_disable(struct device *dev)
{
	if (vdd_lcd_bl_en)
		regulator_disable(vdd_lcd_bl_en);

	if (avdd_lcd)
		regulator_disable(avdd_lcd);

	if (vdd_ds_1v8)
		regulator_disable(vdd_ds_1v8);

	msleep(500);

	return 0;
}

static int edp_p_3000_2000_13_5_postsuspend(void)
{
	return 0;
}

static int edp_p_3000_2000_13_5_bl_notify(struct device *dev, int brightness)
{
	struct backlight_device *bl = NULL;
	struct pwm_bl_data *pb = NULL;

	bl = (struct backlight_device *)dev_get_drvdata(dev);
	pb = (struct pwm_bl_data *)dev_get_drvdata(&bl->dev);

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_err("Error: Brightness > 255!\n");
	else if (pb->bl_measured)
		brightness = pb->bl_measured[brightness];

	return brightness;
}

static int edp_p_3000_2000_13_5_check_fb(struct device *dev,
				struct fb_info *info)
{
	struct platform_device *pdev = NULL;

	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &pdev->dev;
}

static struct pwm_bl_data_dt_ops edp_p_3000_2000_13_5_pwm_bl_ops = {
	.notify = edp_p_3000_2000_13_5_bl_notify,
	.check_fb = edp_p_3000_2000_13_5_check_fb,
	.blnode_compatible = "p-edp,3000-2000-13-5-bl",
};

struct tegra_panel_ops edp_p_3000_2000_13_5_ops = {
	.enable = edp_p_3000_2000_13_5_enable,
	.disable = edp_p_3000_2000_13_5_disable,
	.postsuspend = edp_p_3000_2000_13_5_postsuspend,
	.pwm_bl_ops = &edp_p_3000_2000_13_5_pwm_bl_ops,
};
