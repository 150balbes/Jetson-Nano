/*
 * panel-a-1080p-11-6.c: Panel driver for a-1080p-11-6 panel.
 *
 * Copyright (c) 2012-2018, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/regulator/consumer.h>

#include "../dc.h"
#include "../dc_priv.h"
#include "board.h"
#include "board-panel.h"
#include <linux/gpio.h>

static bool reg_requested;

static struct regulator *vdd_lcd_bl;
static struct regulator *vdd_lcd_bl_en;
static struct regulator *avdd_lcd;
static struct regulator *vdd_ds_1v8;
static struct regulator *avdd_3v3_dp;
static struct regulator *avdd_io_edp;
static u16 en_panel_rst;

static int auo_edp_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	vdd_ds_1v8 = regulator_get(dev, "vdd_ds_1v8");
	if (IS_ERR(vdd_ds_1v8)) {
		pr_err("vdd_ds_1v8 regulator get failed\n");
		err = PTR_ERR(vdd_ds_1v8);
		vdd_ds_1v8 = NULL;
		goto fail;
	}

	/*
	 * Backlight
	 */
	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}

	/*
	 * LCD
	 */
	avdd_lcd = regulator_get(dev, "avdd_lcd");
	if (IS_ERR(avdd_lcd)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd);
		avdd_lcd = NULL;
		goto fail;
	}

	/*
	 * LCD_RST
	 *
	 * If the panel reset gpio is explicitly specified in DT,
	 * prefer using it over the avdd_3v3_dp regulator.
	 */
	err = tegra_panel_gpio_get_dt("a-edp,1080p-14-0", &panel_of);
	if (err < 0 || !gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_RESET])) {
		avdd_3v3_dp = regulator_get(dev, "avdd_3v3_dp");
		if (IS_ERR(avdd_3v3_dp)) {
			pr_err("avdd_3v3_dp regulator get failed\n");
			err = PTR_ERR(avdd_3v3_dp);
			avdd_3v3_dp = NULL;
			goto fail;
		}
	} else {
		en_panel_rst = panel_of.panel_gpio[TEGRA_GPIO_RESET];
	}

	if (tegra_dc_is_nvdisplay()) {
	    avdd_io_edp = NULL;
	} else {
	    avdd_io_edp = regulator_get(dev, "avdd_io_edp");
	    if (IS_ERR(avdd_io_edp)) {
		    pr_err("avdd_io_edp regulator get failed\n");
		    err = PTR_ERR(avdd_io_edp);
		    avdd_io_edp = NULL;
		    goto fail;
	    }
	}

	reg_requested = true;
	return 0;
fail:
	return err;
}


static int laguna_edp_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	vdd_ds_1v8 = regulator_get(dev, "vdd_ds_1v8");
	if (IS_ERR(vdd_ds_1v8)) {
		pr_err("vdd_ds_1v8 regulator get failed\n");
		err = PTR_ERR(vdd_ds_1v8);
		vdd_ds_1v8 = NULL;
		goto fail;
	}

	vdd_lcd_bl = regulator_get(dev, "vdd_lcd_bl");
	if (IS_ERR(vdd_lcd_bl)) {
		pr_err("vdd_lcd_bl regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl);
		vdd_lcd_bl = NULL;
		goto fail;
	}

	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}

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

static int ardbeg_edp_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	vdd_ds_1v8 = regulator_get(dev, "vdd_lcd_1v8_s");
	if (IS_ERR_OR_NULL(vdd_ds_1v8)) {
		pr_err("vdd_ds_1v8 regulator get failed\n");
		err = PTR_ERR(vdd_ds_1v8);
		vdd_ds_1v8 = NULL;
		goto fail;
	}

	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR_OR_NULL(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}

	avdd_lcd = regulator_get(dev, "avdd_lcd");
	if (IS_ERR_OR_NULL(avdd_lcd)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd);
		avdd_lcd = NULL;
		goto fail;
	}

	avdd_3v3_dp = regulator_get(dev, "avdd_3v3_dp");
	if (IS_ERR_OR_NULL(avdd_3v3_dp)) {
		pr_err("avdd_3v3_dp regulator get failed\n");
		err = PTR_ERR(avdd_3v3_dp);
		avdd_3v3_dp = NULL;
		goto fail;
	}

	reg_requested = true;
	return 0;
fail:
	return err;
}

static int edp_a_1080p_14_0_enable(struct device *dev)
{
	int err = 0;

	if (of_machine_is_compatible("nvidia,quill") ||
			of_machine_is_compatible("nvidia,jetson-cv"))
		err = auo_edp_regulator_get(dev);
	else if (of_machine_is_compatible("nvidia,ardbeg"))
		err = ardbeg_edp_regulator_get(dev);
	else
		err = laguna_edp_regulator_get(dev);
	if (err < 0) {
		pr_err("edp regulator get failed\n");
		goto fail;
	}

	/* LCD_RST */
	if (gpio_is_valid(en_panel_rst)) {
		gpio_direction_output(en_panel_rst, 1);
		usleep_range(1000, 5000);
		gpio_set_value(en_panel_rst, 0);
		usleep_range(1000, 5000);
		gpio_set_value(en_panel_rst, 1);
	}

	if (vdd_lcd_bl) {
		err = regulator_enable(vdd_lcd_bl);
		if (err < 0) {
			pr_err("vdd_lcd_bl regulator enable failed\n");
			goto fail;
		}
	}

	if (avdd_3v3_dp) {
		err = regulator_enable(avdd_3v3_dp);
		if (err < 0) {
			pr_err("avdd_3v3_dp regulator enable failed\n");
			goto fail;
		}
	}

	msleep(20);

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

	msleep(10);

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}
	msleep(180);
	if (avdd_io_edp) {
		err = regulator_enable(avdd_io_edp);
		if (err < 0) {
			pr_err("avdd_io_edp regulator enable failed\n");
			goto fail;
		}
	}

	return 0;
fail:
	return err;
}

static int edp_a_1080p_14_0_disable(struct device *dev)
{
	if (avdd_io_edp)
	    regulator_disable(avdd_io_edp);

	msleep(10);

	if (vdd_lcd_bl_en)
		regulator_disable(vdd_lcd_bl_en);

	msleep(10);

	if (avdd_lcd)
		regulator_disable(avdd_lcd);

	if (vdd_ds_1v8)
		regulator_disable(vdd_ds_1v8);

	msleep(10);

	if (vdd_lcd_bl)
		regulator_disable(vdd_lcd_bl);

	if (gpio_is_valid(en_panel_rst)) {
		gpio_set_value(en_panel_rst, 0);
		usleep_range(1000, 5000);
	}

	if (avdd_3v3_dp)
		regulator_disable(avdd_3v3_dp);

	msleep(500);

	return 0;
}

static int edp_a_1080p_14_0_postsuspend(void)
{
	return 0;
}

static int edp_a_1080p_14_0_bl_notify(struct device *dev, int brightness)
{
	struct backlight_device *bl = NULL;
	struct pwm_bl_data *pb = NULL;

	bl = (struct backlight_device *)dev_get_drvdata(dev);
	pb = (struct pwm_bl_data *)dev_get_drvdata(&bl->dev);

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else if (pb->bl_measured)
		brightness = pb->bl_measured[brightness];
	return brightness;
}

static int edp_a_1080p_14_0_check_fb(struct device *dev, struct fb_info *info)
{
	struct platform_device *pdev = NULL;
	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &pdev->dev;
}

static struct pwm_bl_data_dt_ops edp_a_1080p_14_0_pwm_bl_ops = {
	.notify = edp_a_1080p_14_0_bl_notify,
	.check_fb = edp_a_1080p_14_0_check_fb,
	.blnode_compatible = "a-edp,1080p-14-0-bl",
};

struct tegra_panel_ops edp_a_1080p_14_0_ops = {
	.enable = edp_a_1080p_14_0_enable,
	.disable = edp_a_1080p_14_0_disable,
	.postsuspend = edp_a_1080p_14_0_postsuspend,
	.pwm_bl_ops = &edp_a_1080p_14_0_pwm_bl_ops,
};
