/*
 * arch/arm/mach-tegra/panel-s-4kuhd-5-46.c
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include "../dc.h"
#include "../dc_priv.h"
#include "board.h"
#include "board-panel.h"

#define DSI_PANEL_RESET		1

static bool reg_requested;
static struct regulator *vddi_lcd_1v88;
static struct regulator *avdd_lcd_var;
static struct regulator *avee_lcd_var;
static struct device *dc_dev;
static u16 en_panel_rst;
static u16 en_backlight;
static u16 en_panel;
static bool vmm_vpp_fixed_regulators;

static int dsi_s_4kuhd_5_46_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	vddi_lcd_1v88 = regulator_get(dev, "dvdd_lcd");
	if (IS_ERR_OR_NULL(vddi_lcd_1v88)) {
		pr_err("vddi_lcd_1v88 regulator get failed\n");
		err = PTR_ERR(vddi_lcd_1v88);
		vddi_lcd_1v88 = NULL;
		goto fail;
	}

	avdd_lcd_var = regulator_get(dev, "outp");
	if (IS_ERR_OR_NULL(avdd_lcd_var)) {
		pr_err("avdd_lcd_var regulator get failed\n");
		err = PTR_ERR(avdd_lcd_var);
		avdd_lcd_var = NULL;
		goto avdd_lcd_var_fail;
	}

	avee_lcd_var = regulator_get(dev, "outn");
	if (IS_ERR_OR_NULL(avee_lcd_var)) {
		pr_err("avee_lcd_var regulator get failed\n");
		err = PTR_ERR(avee_lcd_var);
		avee_lcd_var = NULL;
		goto avee_lcd_var_fail;
	}

	reg_requested = true;
	return 0;
avee_lcd_var_fail:
	if (avdd_lcd_var) {
		regulator_put(avdd_lcd_var);
		avdd_lcd_var = NULL;
	}
avdd_lcd_var_fail:
	if (vddi_lcd_1v88) {
		regulator_put(vddi_lcd_1v88);
		vddi_lcd_1v88 = NULL;
	}
fail:
	return err;
}

static int dsi_s_4kuhd_5_46_enable(struct device *dev)
{
	int err = 0;

	err = tegra_panel_check_regulator_dt_support("s,4kuhd-5-46",
		&panel_of);
	if (err < 0) {
		pr_err("display regulator dt check failed\n");
		goto fail;
	} else {
		vmm_vpp_fixed_regulators = panel_of.en_vmm_vpp_i2c_config;
	}

	err = dsi_s_4kuhd_5_46_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}

	err = tegra_panel_gpio_get_dt("s,4kuhd-5-46", &panel_of);
	if (err < 0) {
		pr_err("display gpio get failed\n");
		goto fail;
	}

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_RESET]))
		en_panel_rst = panel_of.panel_gpio[TEGRA_GPIO_RESET];
	else {
		pr_err("display reset gpio invalid\n");
		goto fail;
	}

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_BL_ENABLE]))
		en_backlight = panel_of.panel_gpio[TEGRA_GPIO_BL_ENABLE];
	else
		pr_err("display backlight enable gpio invalid\n");

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN]))
		en_panel = panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN];
	else
		pr_err("display panel enable gpio invalid\n");

	if (vddi_lcd_1v88) {
		err = regulator_enable(vddi_lcd_1v88);
		if (err < 0) {
			pr_err("dvdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	usleep_range(1000, 1500);

	if (gpio_is_valid(en_backlight))
		gpio_direction_output(en_backlight, 1);

	if (en_panel)
		gpio_direction_output(en_panel, 1);

	if (avdd_lcd_var) {
		err = regulator_enable(avdd_lcd_var);
		if (err < 0) {
			pr_err("avdd_lcd_var regulator enable failed\n");
			goto fail;
		}

		if (!vmm_vpp_fixed_regulators) {
			err = regulator_set_voltage(avdd_lcd_var,
						5800000, 5800000);
			if (err < 0) {
				pr_err("avdd_lcd_var regulator failed changing voltage\n");
				goto fail;
			}
		}
	}

	usleep_range(2000, 2500);

	if (avee_lcd_var) {
		err = regulator_enable(avee_lcd_var);
		if (err < 0) {
			pr_err("avee_lcd_var regulator enable failed\n");
			goto fail;
		}

		if (!vmm_vpp_fixed_regulators) {
			err = regulator_set_voltage(avee_lcd_var,
						5600000, 5600000);
			if (err < 0) {
				pr_err("avee_lcd_var regulator failed changing voltage\n");
				goto fail;
			}
		}
	}

	usleep_range(15000, 15500);

	dc_dev = dev;
	return 0;
fail:
	return err;
}

static int dsi_s_4kuhd_5_46_disable(struct device *dev)
{
	if (gpio_is_valid(en_panel_rst)) {
		/* Wait for 50ms before triggering panel reset */
		msleep(50);
		gpio_set_value(en_panel_rst, 0);
		usleep_range(500, 1000);
	} else
		pr_err("ERROR! display reset gpio invalid\n");

	if (avee_lcd_var)
		regulator_disable(avee_lcd_var);

	usleep_range(2000, 2500);

	if (avdd_lcd_var)
		regulator_disable(avdd_lcd_var);

	msleep(125);

	if (en_panel)
		gpio_direction_output(en_panel, 0);

	if (vddi_lcd_1v88)
		regulator_disable(vddi_lcd_1v88);

	msleep(140);

	dc_dev = NULL;

	return 0;
}

static int dsi_s_4kuhd_5_46_postsuspend(void)
{
	return 0;
}

static int dsi_s_4kuhd_5_46_bl_notify(struct device *dev, int brightness)
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

static int dsi_s_4kuhd_5_46_check_fb(struct device *dev,
	struct fb_info *info)
{
	struct platform_device *pdev = NULL;

	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &pdev->dev;
}

static int dsi_s_4kuhs_5_46_postpoweron(struct device *dev)
{
	int err;

#if DSI_PANEL_RESET
	err = gpio_direction_output(en_panel_rst, 1);
	if (err < 0) {
		pr_err("setting display reset gpio value failed\n");
		goto fail;
	}

	mdelay(10);

	err = gpio_direction_output(en_panel_rst, 0);
	if (err < 0) {
		pr_err("setting display reset gpio value 0 failed\n");
		goto fail;
	}

	mdelay(15);

	err = gpio_direction_output(en_panel_rst, 1);
	if (err < 0) {
		pr_err("setting display reset gpio value 1 failed\n");
		goto fail;
	}

	mdelay(20);
#endif
	return 0;
fail:
	pr_err("%s: Panel reset seq failed %d\n", __func__, err);
	return err;
}

static struct pwm_bl_data_dt_ops dsi_s_4kuhd_5_46_pwm_bl_ops = {
	.notify = dsi_s_4kuhd_5_46_bl_notify,
	.check_fb = dsi_s_4kuhd_5_46_check_fb,
	.blnode_compatible = "s,4kuhd-5-46-bl",
};
struct tegra_panel_ops dsi_s_4kuhd_5_46_ops = {
	.enable = dsi_s_4kuhd_5_46_enable,
	.disable = dsi_s_4kuhd_5_46_disable,
	.postsuspend = dsi_s_4kuhd_5_46_postsuspend,
	.pwm_bl_ops = &dsi_s_4kuhd_5_46_pwm_bl_ops,
	.postpoweron = dsi_s_4kuhs_5_46_postpoweron,
};
