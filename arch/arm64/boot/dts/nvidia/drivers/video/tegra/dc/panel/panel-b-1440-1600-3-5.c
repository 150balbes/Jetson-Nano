/*
 * panel-b-1440-1600-3-5.c: Panel driver for b-1440-1600-3-5 panel.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "../dc_priv.h"
#include "board.h"
#include "board-panel.h"

#define PANEL_COMP_STRING	"b,1440-1600-3-5"

static struct tegra_panel_instance panel_instance[] = {
	{
		{.panel_gpio = {-1, -1, -1, -1, -1, -1, -1},},
		{NULL, NULL, NULL},
	},
	{
		{.panel_gpio = {-1, -1, -1, -1, -1, -1, -1},},
		{NULL, NULL, NULL},
	},
};

static int dsi_b_1440_1600_3_5_enable(struct device *dev)
{
	struct regulator *avdd_lcd_var;
	struct regulator *avee_lcd_var;
	struct regulator *vddi_lcd_var;
	int err = 0;
	int panel_id = 0;
	u16 en_panel_rst = 0;
	u16 en_backlight = 0;
	bool vmm_vpp_fixed_regulators;

	err = tegra_panel_get_panel_id(PANEL_COMP_STRING, dev->of_node,
					&panel_id);
	if (err) {
		dev_err(dev, "panel-id not found on dt. Using default\n");
		panel_id = 0;
	}

	err = tegra_panel_check_regulator_dt_support(PANEL_COMP_STRING,
			&panel_instance[panel_id].panel_of);
	if (err < 0) {
		dev_err(dev, "display regulator dt check failed\n");
		goto fail;
	} else {
		vmm_vpp_fixed_regulators = panel_of.en_vmm_vpp_i2c_config;
	}

	err = tegra_panel_regulator_get_dt(dev,
			&panel_instance[panel_id].panel_reg);
	if (err < 0) {
		dev_err(dev, "dsi regulator get failed\n");
		goto fail;
	}

	avdd_lcd_var = panel_instance[panel_id].panel_reg.avdd_lcd;
	avee_lcd_var = panel_instance[panel_id].panel_reg.avee_lcd;
	vddi_lcd_var = panel_instance[panel_id].panel_reg.vddi_lcd;

	err = tegra_panel_gpio_get_dt("b,1440-1600-3-5",
			&panel_instance[panel_id].panel_of);
	if (err < 0) {
		dev_err(dev, "display gpio get failed\n");
		goto fail;
	}

	if (gpio_is_valid(panel_instance[panel_id].panel_of.panel_gpio[TEGRA_GPIO_RESET]))
		en_panel_rst = panel_instance[panel_id].panel_of.panel_gpio[TEGRA_GPIO_RESET];
	else {
		dev_err(dev, "display reset gpio invalid\n");
		goto fail;
	}

	if (gpio_is_valid(panel_instance[panel_id].panel_of.panel_gpio[TEGRA_GPIO_BL_ENABLE]))
		en_backlight = panel_instance[panel_id].panel_of.panel_gpio[TEGRA_GPIO_BL_ENABLE];
	else
		dev_err(dev, "display backlight enable gpio invalid\n");

	if (vddi_lcd_var) {
		err = regulator_enable(vddi_lcd_var);
		if (err < 0) {
			dev_err(dev, "dvdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	usleep_range(1000, 1500);

	if (avdd_lcd_var) {
		err = regulator_enable(avdd_lcd_var);
		if (err < 0) {
			dev_err(dev, "avdd_lcd_var regulator enable failed\n");
			goto fail;
		}

		if (!vmm_vpp_fixed_regulators) {
			err = regulator_set_voltage(avdd_lcd_var,
					5600000, 5600000);
			if (err < 0) {
				dev_err(dev,
				"avdd_lcd_var failed changing voltage\n");
				goto fail;
			}
		}
	}

	usleep_range(2000, 2500);

	if (avee_lcd_var) {
		err = regulator_enable(avee_lcd_var);
		if (err < 0) {
			dev_err(dev, "avee_lcd_var regulator enable failed\n");
			goto fail;
		}

		if (!vmm_vpp_fixed_regulators) {
			err = regulator_set_voltage(avee_lcd_var,
					5600000, 5600000);
			if (err < 0) {
				dev_err(dev,
				"avee_lcd_var failed changing voltage\n");
				goto fail;
			}
		}
	}

	usleep_range(15000, 15500);

	err = gpio_direction_output(en_panel_rst, 1);
	if (err < 0) {
		dev_err(dev, "setting display reset gpio value 1 failed\n");
		goto fail;
	}

	mdelay(20);

	if (gpio_is_valid(en_backlight))
		gpio_direction_output(en_backlight, 1);

	return 0;
fail:
	return err;
}

static int dsi_b_1440_1600_3_5_disable(struct device *dev)
{
	int err = 0;
	struct regulator *avdd_lcd_var;
	struct regulator *avee_lcd_var;
	struct regulator *vddi_lcd_var;
	int panel_id = 0;
	u16 en_panel_rst = 0;
	u16 en_backlight = 0;

	err = tegra_panel_get_panel_id(PANEL_COMP_STRING, dev->of_node,
					&panel_id);
	if (err) {
		dev_err(dev, "panel-id not found on dt. Using default\n");
		panel_id = 0;
	}

	avdd_lcd_var = panel_instance[panel_id].panel_reg.avdd_lcd;
	avee_lcd_var = panel_instance[panel_id].panel_reg.avee_lcd;
	vddi_lcd_var = panel_instance[panel_id].panel_reg.vddi_lcd;

	en_backlight = panel_instance[panel_id].panel_of.panel_gpio[TEGRA_GPIO_BL_ENABLE];
	en_panel_rst = panel_instance[panel_id].panel_of.panel_gpio[TEGRA_GPIO_RESET];

	if (gpio_is_valid(en_panel_rst)) {
		/* Wait for 50ms before triggering panel reset */
		msleep(50);
		gpio_set_value(en_panel_rst, 0);
		usleep_range(500, 1000);
	} else
		dev_err(dev, "ERROR! display reset gpio invalid\n");

	if (avee_lcd_var)
		regulator_disable(avee_lcd_var);

	usleep_range(2000, 2500);

	if (avdd_lcd_var)
		regulator_disable(avdd_lcd_var);

	msleep(125);

	if (vddi_lcd_var)
		regulator_disable(vddi_lcd_var);

	/* Min delay of 140ms required to avoid turning
	 * the panel on too soon after power off */
	msleep(140);

	return 0;
}

struct tegra_panel_ops dsi_b_1440_1600_3_5_ops = {
	.enable = dsi_b_1440_1600_3_5_enable,
	.disable = dsi_b_1440_1600_3_5_disable,
};

