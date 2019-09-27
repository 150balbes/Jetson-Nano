/*
 * board-panel.c: Functions definitions for general panel.
 *
 * Copyright (c) 2013-2018, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/tegra/common.h>

#include "../dc.h"
#include "../dc_priv.h"
#include "board-panel.h"
#include "tegra-board-id.h"
#include "board.h"
#include "iomap.h"
#include <linux/platform_data/lp855x.h>

int tegra_bl_notify(struct device *dev, int brightness)
{
	struct lp855x *lp = NULL;
	struct platform_device *pdev = NULL;
	struct device *dc_dev;
	u8 *bl_measured = NULL;
	u8 *bl_curve = NULL;

	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	dc_dev = &pdev->dev;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else if (of_device_is_compatible(dev->of_node,
				"ti,lp8550") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8551") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8552") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8553") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8554") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8555") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8556") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8557")) {
		lp = (struct lp855x *)dev_get_drvdata(dev);
		if (lp && lp->pdata) {
			bl_measured = lp->pdata->bl_measured;
			bl_curve = lp->pdata->bl_curve;
		}
	}

	if (bl_curve)
		brightness = bl_curve[brightness];

	if (bl_measured)
		brightness = bl_measured[brightness];

	return brightness;
}

static struct generic_bl_data_dt_ops generic_bl_ops = {
	.notify = tegra_bl_notify,
};

int tegra_panel_gpio_get_dt(const char *comp_str,
				struct tegra_panel_of *panel)
{
	int cnt = 0;
	char *label = NULL;
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, comp_str);

	/*
	 * If gpios are already populated, just return.
	 */
	if (panel->panel_gpio_populated)
		return 0;

	if (!node) {
		pr_info("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
		goto fail;
	}

	panel->panel_gpio[TEGRA_GPIO_RESET] =
		of_get_named_gpio(node, "nvidia,panel-rst-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PANEL_EN] =
		of_get_named_gpio(node, "nvidia,panel-en-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PANEL_EN_1] =
		of_get_named_gpio(node, "nvidia,panel-en-1-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BL_ENABLE] =
		of_get_named_gpio(node, "nvidia,panel-bl-en-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PWM] =
		of_get_named_gpio(node, "nvidia,panel-bl-pwm-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_EN_0] =
		of_get_named_gpio(node, "nvidia,panel-bridge-en-0-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_EN_1] =
		of_get_named_gpio(node, "nvidia,panel-bridge-en-1-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_REFCLK_EN] =
		of_get_named_gpio(node,
			"nvidia,panel-bridge-refclk-en-gpio", 0);

	for (cnt = 0; cnt < TEGRA_N_GPIO_PANEL; cnt++) {
		if (gpio_is_valid(panel->panel_gpio[cnt])) {
			switch (cnt) {
			case TEGRA_GPIO_RESET:
				label = "tegra-panel-reset";
				break;
			case TEGRA_GPIO_PANEL_EN:
				label = "tegra-panel-en";
				break;
			case TEGRA_GPIO_PANEL_EN_1:
				label = "tegra-panel-en-1";
				break;
			case TEGRA_GPIO_BL_ENABLE:
				label = "tegra-panel-bl-enable";
				break;
			case TEGRA_GPIO_PWM:
				label = "tegra-panel-pwm";
				break;
			case TEGRA_GPIO_BRIDGE_EN_0:
				label = "tegra-panel-bridge-en-0";
				break;
			case TEGRA_GPIO_BRIDGE_EN_1:
				label = "tegra-panel-bridge-en-1";
				break;
			case TEGRA_GPIO_BRIDGE_REFCLK_EN:
				label = "tegra-panel-bridge-refclk-en";
				break;
			default:
				pr_err("tegra panel no gpio entry\n");
			}
			if (label) {
				gpio_request(panel->panel_gpio[cnt],
					label);
				label = NULL;
			}
		}
	}
	if (gpio_is_valid(panel->panel_gpio[TEGRA_GPIO_PWM]))
		gpio_free(panel->panel_gpio[TEGRA_GPIO_PWM]);
	panel->panel_gpio_populated = true;
fail:
	of_node_put(node);
	return err;
}

int tegra_panel_check_regulator_dt_support(const char *comp_str,
				struct tegra_panel_of *panel)
{
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, comp_str);

	if (!node) {
		pr_info("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
	}

	panel->en_vmm_vpp_i2c_config =
		of_property_read_bool(node, "nvidia,en-vmm-vpp-with-i2c-config");

	return err;
}

static bool tegra_available_pwm_bl_ops_register(struct device *dev)
{
	struct device_node *np_bl = NULL;
	struct device_node *np_parent = NULL;
	const char *pn_compat = NULL;
	bool ret = false;

	np_parent = of_find_node_by_path("/backlight");
	if (np_parent) {
		for_each_available_child_of_node(np_parent, np_bl) {
			if (np_bl)
				break;
		}
	}

	if (!np_bl) {
		pr_info("no avaiable target backlight node\n");
		goto end;
	}

	pn_compat = of_get_property(np_bl, "compatible", NULL);
	if (!pn_compat) {
		WARN(1, "No compatible prop in backlight node\n");
		goto end;
	}

	if (of_device_is_compatible(np_bl, "p,wuxga-10-1-bl")) {
		dev_set_drvdata(dev, dsi_p_wuxga_10_1_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "lg,wxga-7-bl")) {
		dev_set_drvdata(dev, dsi_lgd_wxga_7_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "s,wqxga-10-1-bl")) {
		dev_set_drvdata(dev, dsi_s_wqxga_10_1_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "c,wxga-14-0-bl")) {
		dev_set_drvdata(dev, lvds_c_1366_14_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "a,1080p-14-0-bl")) {
		dev_set_drvdata(dev, dsi_a_1080p_14_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "j,1440-810-5-8-bl")) {
		dev_set_drvdata(dev, dsi_j_1440_810_5_8_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "s,wuxga-7-0-bl")) {
		dev_set_drvdata(dev, dsi_s_wuxga_7_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "s,wuxga-8-0-bl")) {
		dev_set_drvdata(dev, dsi_s_wuxga_8_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "a,wuxga-8-0-bl")) {
		dev_set_drvdata(dev, dsi_a_1200_1920_8_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "a,wxga-8-0-bl")) {
		dev_set_drvdata(dev, dsi_a_1200_800_8_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "i-edp,1080p-11-6-bl")) {
		dev_set_drvdata(dev, edp_i_1080p_11_6_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "a-edp,1080p-14-0-bl")) {
		dev_set_drvdata(dev, edp_a_1080p_14_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "j,720p-5-0-bl")) {
		dev_set_drvdata(dev, dsi_j_720p_5_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "l,720p-5-0-bl")) {
		dev_set_drvdata(dev, dsi_l_720p_5_loki_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "s-edp,uhdtv-15-6-bl")) {
		dev_set_drvdata(dev, edp_s_uhdtv_15_6_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "p-edp,3000-2000-13-5-bl")) {
		dev_set_drvdata(dev, edp_p_3000_2000_13_5_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "o,720-1280-6-0-bl")) {
		dev_set_drvdata(dev, dsi_o_720p_6_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "o,720-1280-6-0-01-bl")) {
		dev_set_drvdata(dev, dsi_o_720p_6_0_ops.pwm_bl_ops);
	} else if (of_device_is_compatible(np_bl, "dsi,1080p-bl")) {
	} else if (of_device_is_compatible(np_bl, "dsi,2820x720-bl")) {
	} else if (of_device_is_compatible(np_bl, "dsi,25x16-bl")) {
	} else {
		pr_info("invalid compatible for backlight node\n");
		goto end;
	}

	ret = true;
end:
	of_node_put(np_parent);
	of_node_put(np_bl);
	return ret;
}
static void tegra_pwm_bl_ops_reg_based_on_disp_board_id(struct device *dev)
{
	struct board_info display_board;

	bool is_dsi_a_1200_1920_8_0 = false;
	bool is_dsi_a_1200_800_8_0 = false;
	bool is_edp_i_1080p_11_6 = false;
	bool is_edp_a_1080p_14_0 = false;
	bool is_edp_s_2160p_15_6 = false;

	tegra_get_display_board_info(&display_board);

	switch (display_board.board_id) {
	case BOARD_E1627:
	case BOARD_E1797:
		dev_set_drvdata(dev, dsi_p_wuxga_10_1_ops.pwm_bl_ops);
		break;
	case BOARD_E1549:
		dev_set_drvdata(dev, dsi_lgd_wxga_7_0_ops.pwm_bl_ops);
		break;
	case BOARD_E1639:
	case BOARD_E1813:
	case BOARD_E2145:
		dev_set_drvdata(dev, dsi_s_wqxga_10_1_ops.pwm_bl_ops);
		break;
	case BOARD_PM366:
		dev_set_drvdata(dev, lvds_c_1366_14_ops.pwm_bl_ops);
		break;
	case BOARD_PM354:
		dev_set_drvdata(dev, dsi_a_1080p_14_0_ops.pwm_bl_ops);
		break;
	case BOARD_E2129:
		dev_set_drvdata(dev, dsi_j_1440_810_5_8_ops.pwm_bl_ops);
		break;
	case BOARD_E2534:
		if (display_board.fab == 0x2)
			dev_set_drvdata(dev,
				dsi_j_720p_5_ops.pwm_bl_ops);
		else if (display_board.fab == 0x1)
			dev_set_drvdata(dev,
				dsi_j_1440_810_5_8_ops.pwm_bl_ops);
		else
			dev_set_drvdata(dev,
				dsi_l_720p_5_loki_ops.pwm_bl_ops);
		break;
	case BOARD_E1937:
	case BOARD_E2149:
		is_dsi_a_1200_1920_8_0 = true;
		break;
	case BOARD_E1807:
		is_dsi_a_1200_800_8_0 = true;
		break;
	case BOARD_P1761:
		if (tegra_get_board_panel_id())
			is_dsi_a_1200_1920_8_0 = true;
		else
			is_dsi_a_1200_800_8_0 = true;
		break;
	case BOARD_PM363:
	case BOARD_E1824:
		if (of_machine_is_compatible("nvidia,jetson-cv")) {
			if (display_board.sku == 0x123)
				is_edp_a_1080p_14_0 = true;
			else
				is_edp_s_2160p_15_6 = true;
		}
		else if (display_board.sku == 1200)
			is_edp_i_1080p_11_6 = true;
		else
			is_edp_a_1080p_14_0 = true;
		break;
	default:
		pr_info("pwm_bl_ops are not required\n");
	}

	if (is_dsi_a_1200_1920_8_0)
		dev_set_drvdata(dev, dsi_a_1200_1920_8_0_ops.pwm_bl_ops);
	if (is_dsi_a_1200_800_8_0)
		dev_set_drvdata(dev, dsi_a_1200_800_8_0_ops.pwm_bl_ops);

	if (is_edp_i_1080p_11_6)
		dev_set_drvdata(dev, edp_i_1080p_11_6_ops.pwm_bl_ops);
	if (is_edp_a_1080p_14_0)
		dev_set_drvdata(dev, edp_a_1080p_14_0_ops.pwm_bl_ops);
	if (is_edp_s_2160p_15_6)
		dev_set_drvdata(dev, edp_s_uhdtv_15_6_ops.pwm_bl_ops);
}

void tegra_pwm_bl_ops_register(struct device *dev)
{
	bool ret = 0;

	if (tegra_dc_is_nvdisplay())
		return;

	ret = tegra_available_pwm_bl_ops_register(dev);
	if (!ret)
		tegra_pwm_bl_ops_reg_based_on_disp_board_id(dev);
}
EXPORT_SYMBOL(tegra_pwm_bl_ops_register);

void ti_lp855x_bl_ops_register(struct device *dev)
{
	dev_set_drvdata(dev, &generic_bl_ops);
}
