/*
 * of_dc.c: tegra dc of interface.
 *
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/nvhost.h>
#include <linux/timer.h>
#if defined(CONFIG_TRUSTED_LITTLE_KERNEL) || defined(CONFIG_TRUSTY)
#include <linux/ote_protocol.h>
#endif
#ifdef CONFIG_PINCTRL_CONSUMER
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-tegra.h>
#endif
#include <video/tegra_hdmi_audio.h>
#include <linux/clk/tegra.h>
#include <linux/platform/tegra/latency_allowance.h>
#include <linux/platform/tegra/mc.h>
#include <soc/tegra/common.h>
#include <linux/version.h>

#include "dc.h"
#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"
#include "dsi.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "dp.h"
#include "dp_lt.h"
#include "panel/board-panel.h"
#include "panel/tegra-board-id.h"
#include "dc_common.h"
#include "hdmivrr.h"

/* #define OF_DC_DEBUG */

#undef OF_DC_LOG
#ifdef OF_DC_DEBUG
#define OF_DC_LOG(fmt, args...) pr_info("OF_DC_LOG: " fmt, ## args)
#else
#define OF_DC_LOG(fmt, args...)
#endif

static struct regulator *of_hdmi_vddio;
static struct regulator *of_hdmi_dp_reg;
static struct regulator *of_hdmi_pll;

static struct regulator *of_dp_pwr;
static struct regulator *of_dp_pll;
static struct regulator *of_edp_sec_mode;
static struct regulator *of_dp_pad;
static struct regulator *of_dp_hdmi_5v0;

static bool os_l4t;

static struct tegra_dc_cmu default_cmu = {
	/* lut1 maps sRGB to linear space. */
	{
		0,    1,    2,    4,    5,    6,    7,    9,
		10,   11,   12,   14,   15,   16,   18,   20,
		21,   23,   25,   27,   29,   31,   33,   35,
		37,   40,   42,   45,   48,   50,   53,   56,
		59,   62,   66,   69,   72,   76,   79,   83,
		87,   91,   95,   99,   103,  107,  112,  116,
		121,  126,  131,  136,  141,  146,  151,  156,
		162,  168,  173,  179,  185,  191,  197,  204,
		210,  216,  223,  230,  237,  244,  251,  258,
		265,  273,  280,  288,  296,  304,  312,  320,
		329,  337,  346,  354,  363,  372,  381,  390,
		400,  409,  419,  428,  438,  448,  458,  469,
		479,  490,  500,  511,  522,  533,  544,  555,
		567,  578,  590,  602,  614,  626,  639,  651,
		664,  676,  689,  702,  715,  728,  742,  755,
		769,  783,  797,  811,  825,  840,  854,  869,
		884,  899,  914,  929,  945,  960,  976,  992,
		1008, 1024, 1041, 1057, 1074, 1091, 1108, 1125,
		1142, 1159, 1177, 1195, 1213, 1231, 1249, 1267,
		1286, 1304, 1323, 1342, 1361, 1381, 1400, 1420,
		1440, 1459, 1480, 1500, 1520, 1541, 1562, 1582,
		1603, 1625, 1646, 1668, 1689, 1711, 1733, 1755,
		1778, 1800, 1823, 1846, 1869, 1892, 1916, 1939,
		1963, 1987, 2011, 2035, 2059, 2084, 2109, 2133,
		2159, 2184, 2209, 2235, 2260, 2286, 2312, 2339,
		2365, 2392, 2419, 2446, 2473, 2500, 2527, 2555,
		2583, 2611, 2639, 2668, 2696, 2725, 2754, 2783,
		2812, 2841, 2871, 2901, 2931, 2961, 2991, 3022,
		3052, 3083, 3114, 3146, 3177, 3209, 3240, 3272,
		3304, 3337, 3369, 3402, 3435, 3468, 3501, 3535,
		3568, 3602, 3636, 3670, 3705, 3739, 3774, 3809,
		3844, 3879, 3915, 3950, 3986, 4022, 4059, 4095,
	},
	/* csc */
	{
		0,
	},
	/*lut2*/
	{
		0,
	}
};

struct tegra_panel_ops *tegra_dc_get_panel_ops(struct device_node *panel_np)
{
	struct tegra_panel_ops *p_ops = NULL;

	if (!panel_np) {
		pr_err("%s: invalid input: null panel_np\n", __func__);
		return NULL;
	}

	if (tegra_dc_is_nvdisplay()) {
		if (of_device_is_compatible(panel_np, "nvidia,sim-panel")) {
			p_ops = &panel_sim_ops;
			return p_ops;
		}
	}

	if (of_device_is_compatible(panel_np, "s,wuxga-8-0"))
		p_ops = &dsi_s_wuxga_8_0_ops;
	else if (of_device_is_compatible(panel_np, "s,wuxga-7-0"))
		p_ops = &dsi_s_wuxga_7_0_ops;
	else if (of_device_is_compatible(panel_np, "o,720-1280-6-0"))
		p_ops = &dsi_o_720p_6_0_ops;
	else if (of_device_is_compatible(panel_np, "o,720-1280-6-0-01"))
		p_ops = &dsi_o_720p_6_0_ops;
	else if (of_device_is_compatible(panel_np, "p,wuxga-10-1"))
		p_ops = &dsi_p_wuxga_10_1_ops;
	else if (of_device_is_compatible(panel_np, "lg,wxga-7"))
		p_ops = &dsi_lgd_wxga_7_0_ops;
	else if (of_device_is_compatible(panel_np, "s,wqxga-10-1"))
		p_ops = &dsi_s_wqxga_10_1_ops;
	else if (of_device_is_compatible(panel_np, "c,wxga-14-0"))
		p_ops = &lvds_c_1366_14_ops;
	else if (of_device_is_compatible(panel_np, "a,1080p-14-0"))
		p_ops = &dsi_a_1080p_14_0_ops;
	else if (of_device_is_compatible(panel_np, "j,1440-810-5-8"))
		p_ops = &dsi_j_1440_810_5_8_ops;
	else if (of_device_is_compatible(panel_np, "j,720p-5-0"))
		p_ops = &dsi_j_720p_5_ops;
	else if (of_device_is_compatible(panel_np, "l,720p-5-0"))
		p_ops = &dsi_l_720p_5_loki_ops;
	else if (of_device_is_compatible(panel_np, "a,wuxga-8-0"))
		p_ops = &dsi_a_1200_1920_8_0_ops;
	else if (of_device_is_compatible(panel_np, "a,wxga-8-0"))
		p_ops = &dsi_a_1200_800_8_0_ops;
	else if (of_device_is_compatible(panel_np, "i-edp,1080p-11-6"))
		p_ops = &edp_i_1080p_11_6_ops;
	else if (of_device_is_compatible(panel_np, "a-edp,1080p-14-0"))
		p_ops = &edp_a_1080p_14_0_ops;
	else if (of_device_is_compatible(panel_np, "s-edp,uhdtv-15-6"))
		p_ops = &edp_s_uhdtv_15_6_ops;
	else if (of_device_is_compatible(panel_np, "s,4kuhd-5-46"))
		p_ops = &dsi_s_4kuhd_5_46_ops;
	else if (of_device_is_compatible(panel_np, "b,1440-1600-3-5"))
		p_ops = &dsi_b_1440_1600_3_5_ops;
	else if (of_device_is_compatible(panel_np, "p-edp,3000-2000-13-5"))
		p_ops = &edp_p_3000_2000_13_5_ops;
	else if (of_device_is_compatible(panel_np, "null,dsi-hotplug"))
		p_ops = &dsi_null_panel_ops;
	else
		pr_err("%s: unknown panel: %s\n", __func__,
			of_node_full_name(panel_np));

	return p_ops;
}

int tegra_panel_get_panel_id(const char *comp_str, struct device_node *dnode,
				int *panel_id)
{
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(dnode, NULL, comp_str);

	if (!node) {
		pr_err("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
		goto fail;
	}

	err = of_property_read_u32(node, "nvidia,panel-id", panel_id);

fail:
	of_node_put(node);
	return err;
}

int tegra_panel_regulator_get_dt(struct device *dev,
		struct tegra_panel_reg *panel_reg)
{
	int err = 0;

	panel_reg->vddi_lcd = regulator_get(dev, "dvdd_lcd");
	if (IS_ERR_OR_NULL(panel_reg->vddi_lcd)) {
		pr_err("vddi_lcd_1v88 regulator get failed\n");
		err = PTR_ERR(panel_reg->vddi_lcd);
		panel_reg->vddi_lcd = NULL;
		goto fail;
	}

	panel_reg->avdd_lcd = regulator_get(dev, "outp");
	if (IS_ERR_OR_NULL(panel_reg->avdd_lcd)) {
		pr_err("avdd_lcd_var regulator get failed\n");
		err = PTR_ERR(panel_reg->avdd_lcd);
		panel_reg->avdd_lcd = NULL;
		goto avdd_lcd_var_fail;
	}

	panel_reg->avee_lcd = regulator_get(dev, "outn");
	if (IS_ERR_OR_NULL(panel_reg->avee_lcd)) {
		pr_err("avee_lcd_var regulator get failed\n");
		err = PTR_ERR(panel_reg->avee_lcd);
		panel_reg->avee_lcd = NULL;
		goto avee_lcd_var_fail;
	}

avee_lcd_var_fail:
	if (panel_reg->avdd_lcd) {
		regulator_put(panel_reg->avdd_lcd);
		panel_reg->avdd_lcd = NULL;
	}
avdd_lcd_var_fail:
	if (panel_reg->vddi_lcd) {
		regulator_put(panel_reg->vddi_lcd);
		panel_reg->vddi_lcd = NULL;
	}
fail:
	return err;
}

void tegra_panel_unregister_ops(struct tegra_dc_out *dc_out)
{
	dc_out->enable		= NULL;
	dc_out->postpoweron	= NULL;
	dc_out->prepoweroff	= NULL;
	dc_out->disable		= NULL;
	dc_out->hotplug_init	= NULL;
	dc_out->postsuspend	= NULL;
	dc_out->hotplug_report	= NULL;
}

void tegra_panel_register_ops(struct tegra_dc_out *dc_out,
				struct tegra_panel_ops *p_ops)
{
	dc_out->enable		= p_ops->enable;
	dc_out->postpoweron	= p_ops->postpoweron;
	dc_out->prepoweroff	= p_ops->prepoweroff;
	dc_out->disable		= p_ops->disable;
	dc_out->hotplug_init	= p_ops->hotplug_init;
	dc_out->postsuspend	= p_ops->postsuspend;
	dc_out->hotplug_report	= p_ops->hotplug_report;
}

static struct device_node *tegra_dc_get_panel_from_disp_board_id(
		struct tegra_dc_platform_data *pdata)
{
	struct device_node *panel_np = NULL;
	struct board_info display_board;
	bool is_dsi_a_1200_1920_8_0 = false;
	bool is_dsi_a_1200_800_8_0 = false;
	bool is_edp_i_1080p_11_6 = false;
	bool is_edp_a_1080p_14_0 = false;
	bool is_edp_s_2160p_15_6 = false;

	if (!pdata) {
		pr_err("%s: invalid input: NULL pdata\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	tegra_get_display_board_info(&display_board);
	pr_info("display board info: id 0x%x, fab 0x%x\n",
		display_board.board_id, display_board.fab);

	switch (display_board.board_id) {
	case BOARD_E1627:
	case BOARD_E1797:
		panel_np = of_find_compatible_node(NULL, NULL,
				"p,wuxga-10-1");
		break;
	case BOARD_E1549:
		panel_np = of_find_compatible_node(NULL, NULL,
				"lg,wxga-7");
		break;
	case BOARD_E1639:
	case BOARD_E1813:
	case BOARD_E2145:
		panel_np = of_find_compatible_node(NULL, NULL,
				"s,wqxga-10-1");
		break;
	case BOARD_PM366:
		panel_np = of_find_compatible_node(NULL, NULL,
				"c,wxga-14-0");
		break;
	case BOARD_PM354:
		panel_np = of_find_compatible_node(NULL, NULL,
				"a,1080p-14-0");
		break;
	case BOARD_E2129:
		panel_np = of_find_compatible_node(NULL, NULL,
				"j,1440-810-5-8");
		break;
	case BOARD_E2534:
		if (display_board.fab == 0x2) {
			panel_np = of_find_compatible_node(NULL, NULL,
				"j,720p-5-0");
		} else if (display_board.fab == 0x1) {
			panel_np = of_find_compatible_node(NULL, NULL,
				"j,1440-810-5-8");
		} else {
			panel_np = of_find_compatible_node(NULL, NULL,
				"l,720p-5-0");
		}
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
		if (of_machine_is_compatible("nvidia,jetson-cv"))
			is_edp_s_2160p_15_6 = true;
		else if (display_board.sku == 1200)
			is_edp_i_1080p_11_6 = true;
		else
			is_edp_a_1080p_14_0 = true;

		break;
	case BOARD_E2606:
	case BOARD_E2603:
		is_edp_a_1080p_14_0 = true;
		break;
	}

	if (is_dsi_a_1200_1920_8_0)
		panel_np = of_find_compatible_node(NULL, NULL,
				"a,wuxga-8-0");
	else if (is_dsi_a_1200_800_8_0)
		panel_np = of_find_compatible_node(NULL, NULL,
				"a,wxga-8-0");
	else if (is_edp_i_1080p_11_6)
		panel_np = of_find_compatible_node(NULL, NULL,
				"i-edp,1080p-11-6");
	else if (is_edp_a_1080p_14_0)
		panel_np = of_find_compatible_node(NULL, NULL,
				"a-edp,1080p-14-0");
	else if (is_edp_s_2160p_15_6)
		panel_np = of_find_compatible_node(NULL, NULL,
				"s-edp,uhdtv-15-6");

	of_node_put(panel_np); /* NULL safe */

	return panel_np;
}

static int tegra_dc_parse_panel_ops(struct platform_device *ndev,
	struct tegra_dc_platform_data *pdata)
{
	int ret = 0;
	bool assign_panel_ops = true;
	struct device_node *panel_np = NULL;
	struct device_node *conn_np = NULL;
	struct tegra_panel_ops *panel_ops = NULL;

	if (!pdata || !pdata->conn_np) {
		dev_err(&ndev->dev, "%s: invalid input: NULL pdata or conn_np\n",
			__func__);
		ret = -EINVAL;
		goto exit;
	}

	conn_np = pdata->conn_np;
	/*
	 * Note: carrying legacy logic of retrieving panel_np only for
	 *       the first display.
	 */
	if ((ndev->id == 0) && (tegra_dc_is_t21x())) {
		/*
		 * First select panel based on display board-id. If we don't
		 * find one then select one specified in device-tree.
		 */
		panel_np = tegra_dc_get_panel_from_disp_board_id(pdata);
		if (!IS_ERR_OR_NULL(panel_np)) {
			if (!of_device_is_available(panel_np)) {
				dev_err(&ndev->dev,
					"panel: %s from disp board is not enabled. Try default panel from DT.\n",
					of_node_full_name(panel_np));
				panel_np = NULL;
			}
		} else if (IS_ERR(panel_np)) {
			panel_np = NULL;
		}
	}

	/*
	 * if panel is not selected by display board-id, find panel
	 * using "nvidia,active-panel" property.
	 */
	if (!panel_np) {
		panel_np = of_parse_phandle(conn_np, "nvidia,active-panel", 0);
		if (IS_ERR_OR_NULL(panel_np)) {
			dev_err(&ndev->dev, "%s: could not find panel for %s\n",
				__func__, of_node_full_name(conn_np));
			ret = -ENODEV;
			goto exit;
		}
	}

	if (of_device_is_compatible(panel_np, "dsi,1080p")        ||
	    of_device_is_compatible(panel_np, "dsi,2820x720")     ||
	    of_device_is_compatible(panel_np, "dsi,25x16")     ||
	    of_device_is_compatible(panel_np, "s,wuxga-8-0-mods") ||
	    of_device_is_compatible(panel_np, "dp, display")      ||
	    of_device_is_compatible(panel_np, "hdmi,display"))
		assign_panel_ops = false;

	if (assign_panel_ops) {
		panel_ops = tegra_dc_get_panel_ops(panel_np);
		if (!panel_ops) {
			ret = -ENODEV;
			goto exit;
		}
		tegra_panel_register_ops(pdata->default_out, panel_ops);
	}

	if (!of_device_is_available(panel_np)) {
		dev_err(&ndev->dev, "%s: panel: %s is not active\n",
				__func__, of_node_full_name(panel_np));

		if (assign_panel_ops)
			tegra_panel_unregister_ops(pdata->default_out);

		ret = -ENODEV;
		goto exit;
	}

	pdata->panel_np = panel_np;

exit:
	of_node_put(panel_np);
	return ret;
}

static int tegra_dc_parse_out_type(struct platform_device *ndev,
	struct tegra_dc_platform_data *pdata, bool pdata_initialized)
{
	u32 temp;
	int ret = 0;
	struct device_node *temp_np;
	struct device_node *panel_np = pdata->panel_np;

	temp_np = of_get_child_by_name(panel_np, "disp-default-out");
	if (!temp_np) {
		dev_err(&ndev->dev, "mandatory property %s not found\n",
				"disp-default-out");
		ret = -ENODEV;
		goto exit;
	}

	/* Check for OR out_type from dt when not using crossbar
	 * or when restoring to boot topology in crossbar
	 */
	if (!pdata_initialized ||
			pdata->default_out->type == TEGRA_DC_TOPOLOGY_RESTORE) {
		if (of_property_read_u32(temp_np, "nvidia,out-type", &temp)) {
			dev_err(&ndev->dev, "mandatory property %s not found\n",
				"nvidia,out-type");
			ret = -ENODEV;
			goto exit;
		}
		pdata->default_out->type = (int)temp;
	}

	if ((pdata->default_out->type < TEGRA_DC_OUT_RGB) ||
	    (pdata->default_out->type >= TEGRA_DC_OUT_MAX)) {
		dev_err(&ndev->dev, "invalid out_type:%d\n",
				pdata->default_out->type);
		ret = -EINVAL;
		goto exit;
	}

	pdata->def_out_np = temp_np;
exit:
	of_node_put(temp_np);
	return ret;
}

static bool is_dc_default_out_flag(u32 flag)
{
	if ((flag == TEGRA_DC_OUT_HOTPLUG_HIGH) |
		(flag == TEGRA_DC_OUT_HOTPLUG_LOW) |
		(flag == TEGRA_DC_OUT_NVHDCP_POLICY_ALWAYS_ON) |
		(flag == TEGRA_DC_OUT_NVHDCP_POLICY_ON_DEMAND) |
		(flag == TEGRA_DC_OUT_CONTINUOUS_MODE) |
		(flag == TEGRA_DC_OUT_ONE_SHOT_MODE) |
		(flag == TEGRA_DC_OUT_N_SHOT_MODE) |
		(flag == TEGRA_DC_OUT_ONE_SHOT_LP_MODE) |
		(flag == TEGRA_DC_OUT_INITIALIZED_MODE) |
		(flag == TEGRA_DC_OUT_HOTPLUG_WAKE_LP0))
		return true;
	else
		return false;
}

static int parse_disp_default_out(struct platform_device *ndev,
		struct tegra_dc_platform_data *pdata)
{
	u8 *addr;
	u32 temp, u, n_outpins = 0;
	int err = 0, hotplug_gpio = 0;
	enum of_gpio_flags flags;
	const __be32 *p;
	const char *temp_str0;
	struct property *prop;
	struct device_node *conn_np, *out_np;
	struct tegra_fb_data *fb;

	if (!pdata || !pdata->fb || !pdata->conn_np || !pdata->def_out_np) {
		dev_err(&ndev->dev, "NULL pdata, fb, conn_np or def_out_np\n");
		return -EINVAL;
	}

	fb = pdata->fb;
	out_np = pdata->def_out_np;
	conn_np = pdata->conn_np;

	/*
	 * construct default_out
	 */
	if (!of_property_read_u32(out_np, "nvidia,out-width", &temp)) {
		pdata->default_out->width = (unsigned) temp;
		OF_DC_LOG("out_width %d\n", default_out->width);
	}
	if (!of_property_read_u32(out_np, "nvidia,out-height", &temp)) {
		pdata->default_out->height = (unsigned) temp;
		OF_DC_LOG("out_height %d\n", default_out->height);
	}
	if (!of_property_read_u32(out_np, "nvidia,out-rotation", &temp)) {
		pdata->default_out->rotation = (unsigned) temp;
		OF_DC_LOG("out_rotation %d\n", temp);
	}

	if (!tegra_platform_is_sim() &&
	    pdata->default_out->type == TEGRA_DC_OUT_HDMI) {
		int id;
		struct device_node *ddc_np =
			of_parse_phandle(conn_np, "nvidia,ddc-i2c-bus", 0);

		if (!ddc_np) {
			pdata->default_out->dcc_bus = -1;
			pr_warn("error reading %s node\n",
				"nvidia,ddc-i2c-bus");
		} else {
			id = of_alias_get_id(ddc_np, "i2c");
			of_node_put(ddc_np);

			if (id >= 0) {
				pdata->default_out->dcc_bus = id;
				OF_DC_LOG("out_dcc bus %d\n", id);
			} else {
				dev_err(&ndev->dev, "invalid i2c id:%d\n", id);
				err = -EINVAL;
				goto parse_disp_defout_fail;
			}
		}
	}

	hotplug_gpio = of_get_named_gpio_flags(conn_np,
					       "nvidia,hpd-gpio", 0, &flags);
	if (hotplug_gpio >= 0) {
		pdata->default_out->hotplug_gpio = hotplug_gpio;
	} else {
		if (hotplug_gpio == -ENOENT)
			dev_info(&ndev->dev, "No hpd-gpio in DT\n");
		else
			dev_warn(&ndev->dev, "invalid hpd-gpio %d\n",
					hotplug_gpio);

		if (hotplug_gpio == -EPROBE_DEFER) {
			err = -EPROBE_DEFER;
			goto parse_disp_defout_fail;
		}
	}

	if (!of_property_read_u32(out_np, "nvidia,out-max-pixclk", &temp)) {
		pdata->default_out->max_pixclock = (unsigned)temp;
		OF_DC_LOG("%u max_pixclock in pico second unit\n",
			pdata->default_out->max_pixclock);
	}

	if (!of_property_read_u32(out_np, "nvidia,dither", &temp)) {
		pdata->default_out->dither = (unsigned)temp;
		OF_DC_LOG("dither option %d\n",
			pdata->default_out->dither);
	}

	of_property_for_each_u32(out_np, "nvidia,out-flags", prop, p, u) {
		if (!is_dc_default_out_flag(u)) {
			dev_err(&ndev->dev, "invalid out-flags:0x%x\n", u);
			err = -EINVAL;
			goto parse_disp_defout_fail;
		}
		pdata->default_out->flags |= (unsigned) u;
	}

	if (!of_property_read_u32(out_np, "nvidia,out-hdcp-policy", &temp)) {
		pdata->default_out->hdcp_policy = (unsigned)temp;
		OF_DC_LOG("hdcp_policy = %u\n", default_out->hdcp_policy);
	} else {
		pdata->default_out->hdcp_policy =
#if defined(CONFIG_ANDROID)
			TEGRA_DC_HDCP_POLICY_ALWAYS_ON;
#else
			TEGRA_DC_HDCP_POLICY_ALWAYS_OFF;
#endif
	}

	if (tegra_platform_is_sim())
		pdata->default_out->hotplug_gpio = -1;

	/* if hotplug not supported clear TEGRA_DC_OUT_HOTPLUG_WAKE_LP0 */
	if (pdata->default_out->hotplug_gpio < 0)
		pdata->default_out->flags &= ~TEGRA_DC_OUT_HOTPLUG_WAKE_LP0;

	OF_DC_LOG("default_out flag %u\n", pdata->default_out->flags);

	if (!of_property_read_u32(out_np, "nvidia,out-align", &temp)) {
		if (temp == TEGRA_DC_ALIGN_MSB) {
			OF_DC_LOG("tegra dc align msb\n");
		} else if (temp == TEGRA_DC_ALIGN_LSB) {
			OF_DC_LOG("tegra dc align lsb\n");
		} else {
			dev_err(&ndev->dev, "invalid out-align:0x%x\n", temp);
			err = -EINVAL;
			goto parse_disp_defout_fail;
		}
		pdata->default_out->align = (unsigned)temp;
	}

	if (!of_property_read_u32(out_np, "nvidia,out-order", &temp)) {
		if (temp == TEGRA_DC_ORDER_RED_BLUE) {
			OF_DC_LOG("tegra order red to blue\n");
		} else if (temp == TEGRA_DC_ORDER_BLUE_RED) {
			OF_DC_LOG("tegra order blue to red\n");
		} else {
			dev_err(&ndev->dev, "invalid out-order:0x%x\n", temp);
			err = -EINVAL;
			goto parse_disp_defout_fail;
		}
		pdata->default_out->order = (unsigned)temp;
	}

	of_property_for_each_u32(out_np, "nvidia,out-pins", prop, p, u)
		n_outpins++;

	if ((n_outpins & 0x1) != 0) {
		dev_err(&ndev->dev, "should have name, polarity pair!\n");
		err = -EINVAL;
		goto parse_disp_defout_fail;
	}
	n_outpins = n_outpins/2;
	pdata->default_out->n_out_pins = (unsigned)n_outpins;
	if (n_outpins)
		pdata->default_out->out_pins = devm_kzalloc(&ndev->dev,
			n_outpins * sizeof(struct tegra_dc_out_pin),
			GFP_KERNEL);

	if (n_outpins && !pdata->default_out->out_pins) {
		err = -ENOMEM;
		goto parse_disp_defout_fail;
	}
	addr = (u8 *)pdata->default_out->out_pins;

	n_outpins = 0;
	of_property_for_each_u32(out_np, "nvidia,out-pins", prop, p, u) {
		if ((n_outpins & 0x1) == 0)
			((struct tegra_dc_out_pin *)addr)->name = (int)u;
		else {
			((struct tegra_dc_out_pin *)addr)->pol = (int)u;
			addr += sizeof(struct tegra_dc_out_pin);
		}
		n_outpins++;
	}

	if (!of_property_read_string(out_np, "nvidia,out-parent-clk",
				     &temp_str0)) {
		pdata->default_out->parent_clk = temp_str0;
		OF_DC_LOG("parent clk %s\n", pdata->default_out->parent_clk);
	} else {
		dev_info(&ndev->dev, "%s: No parent clk. Using default clk\n",
				__func__);
	}

	if (pdata->default_out->type == TEGRA_DC_OUT_HDMI) {
		pdata->default_out->depth = 0;
		if (tegra_fb_is_console_enabled(pdata)) {
			if (!of_property_read_u32(out_np,
						"nvidia,out-depth", &temp)) {
				pdata->default_out->depth = (unsigned) temp;
				OF_DC_LOG("out-depth for HDMI FB console %d\n", temp);
			}
		}
	} else {
		if (!of_property_read_u32(out_np, "nvidia,out-depth", &temp)) {
			pdata->default_out->depth = (unsigned) temp;
			OF_DC_LOG("out-depth for non-HDMI display %d\n", temp);
		}
	}

	if (!of_property_read_u32(out_np, "nvidia,out-hotplug-state", &temp)) {
		pdata->default_out->hotplug_state = (unsigned) temp;
		OF_DC_LOG("out-hotplug-state %d\n", temp);
	}

	/*
	 * construct fb
	 */
	fb->win = TEGRA_FB_WIN_INVALID; /* set fb->win default */

	if (!of_property_read_u32(out_np, "nvidia,out-xres", &temp)) {
		fb->xres = (int)temp;
		OF_DC_LOG("framebuffer xres %d\n", fb->xres);
	}
	if (!of_property_read_u32(out_np, "nvidia,out-yres", &temp)) {
		fb->yres = (int)temp;
		OF_DC_LOG("framebuffer yres %d\n", fb->yres);
	}

parse_disp_defout_fail:

	return err;
}

static int parse_vrr_settings(struct platform_device *ndev,
		struct device_node *np,
		struct tegra_vrr *vrr)
{
	u32 temp;

	if (!of_property_read_u32(np, "nvidia,vrr_min_fps", &temp)) {
		vrr->vrr_min_fps = temp;
		OF_DC_LOG("vrr_min_fps %u\n", temp);
	}

	if (!of_property_read_u32(np, "nvidia,frame_len_fluct", &temp)) {
		vrr->frame_len_fluct = temp;
		OF_DC_LOG("frame_len_fluct %u\n", temp);
	} else
		vrr->frame_len_fluct = 2000;

	if (!of_property_read_u32(np, "nvidia,db_correct_cap", &temp)) {
		vrr->db_correct_cap = temp;
		OF_DC_LOG("db_correct_cap %u\n", temp);
	} else
		vrr->db_correct_cap = 0;

	if (!of_property_read_u32(np, "nvidia,db_hist_cap", &temp)) {
		vrr->db_hist_cap = temp;
		OF_DC_LOG("db_hist_cap %u\n", temp);
	} else
		vrr->db_hist_cap = 0;

	if (!of_property_read_u32(np, "nvidia,nvdisp_direct_drive", &temp)) {
		vrr->nvdisp_direct_drive = temp;
		OF_DC_LOG("nvdisp_direct_drive %u\n", temp);
	} else
		vrr->nvdisp_direct_drive = 0;

	return 0;
}

static void parse_spd_infoframe(struct platform_device *ndev,
	struct device_node *np, struct tegra_dc_out *default_out)
{
	struct device_node *spd_np = NULL;
	struct spd_infoframe *spd;
	const char *temp_str0;
	u32 temp;

	default_out->hdmi_out->spd_infoframe = devm_kzalloc(&ndev->dev,
		sizeof(struct spd_infoframe), GFP_KERNEL);
	spd = default_out->hdmi_out->spd_infoframe;

	strlcpy(spd->vendor_name, SPD_DEFAULT_VENDOR_NAME,
		sizeof(spd->vendor_name));
	strlcpy(spd->prod_desc, SPD_DEFAULT_PRODUCT_DESC,
		sizeof(spd->prod_desc));
	spd->source_information = SPD_DEFAULT_SOURCE_INFO;

	spd_np = of_get_child_by_name(np, "spd-infoframe");
	if (spd_np) {
		if (!of_property_read_string(spd_np, "vendor-name",
					&temp_str0))
			strlcpy(spd->vendor_name, temp_str0,
					sizeof(spd->vendor_name));
		else
			OF_DC_LOG("spd vendor-name is not defined\n");

		if (!of_property_read_string(spd_np, "product-description",
					&temp_str0))
			strlcpy(spd->prod_desc, temp_str0,
					sizeof(spd->prod_desc));
		else
			OF_DC_LOG("spd product-description is not defined\n");

		if (!of_property_read_u32(spd_np, "source-information", &temp))
			spd->source_information = (u32)temp;
		else
			OF_DC_LOG("spd source-information is not defined\n");
	} else
		OF_DC_LOG("%s: No spd-infoframe node\n", __func__);

	OF_DC_LOG("spd vendor-name %s\n", spd->vendor_name);
	OF_DC_LOG("spd product-description %s\n", spd->prod_desc);
	OF_DC_LOG("spd source-information %d\n", spd->source_information);
}

static int parse_modes(struct tegra_dc_out *default_out,
						struct device_node *np,
						struct tegra_dc_mode *modes)
{
	u32 temp;
	const struct tegra_dc_out_pin *pins = default_out->out_pins;
	int i;

	/* {V,H}_REF_TO_SYNC do NOT exist on nvdisplay. */
	if (!tegra_dc_is_nvdisplay()) {
		if (!of_property_read_u32(np, "nvidia,h-ref-to-sync", &temp)) {
			modes->h_ref_to_sync = temp;
		} else {
			OF_DC_LOG("of h_ref_to_sync %d\n", temp);
			goto parse_modes_fail;
		}
		if (!of_property_read_u32(np, "nvidia,v-ref-to-sync", &temp)) {
			modes->v_ref_to_sync = temp;
		} else {
			OF_DC_LOG("of v_ref_to_sync %d\n", temp);
			goto parse_modes_fail;
		}
	}

	if (!of_property_read_u32(np, "clock-frequency", &temp)) {
		modes->pclk = temp;
		OF_DC_LOG("of pclk %d\n", temp);
	} else {
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "hsync-len", &temp)) {
		modes->h_sync_width = temp;
	} else {
		OF_DC_LOG("of h_sync_width %d\n", temp);
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "vsync-len", &temp)) {
		modes->v_sync_width = temp;
	} else {
		OF_DC_LOG("of v_sync_width %d\n", temp);
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "hback-porch", &temp)) {
		modes->h_back_porch = temp;
	} else {
		OF_DC_LOG("of h_back_porch %d\n", temp);
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "vback-porch", &temp)) {
		modes->v_back_porch = temp;
	} else {
		OF_DC_LOG("of v_back_porch %d\n", temp);
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "hactive", &temp)) {
		modes->h_active = temp;
	} else {
		OF_DC_LOG("of h_active %d\n", temp);
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "vactive", &temp)) {
		modes->v_active = temp;
	} else {
		OF_DC_LOG("of v_active %d\n", temp);
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "hfront-porch", &temp)) {
		modes->h_front_porch = temp;
	} else {
		OF_DC_LOG("of h_front_porch %d\n", temp);
		goto parse_modes_fail;
	}
	if (!of_property_read_u32(np, "vfront-porch", &temp)) {
		modes->v_front_porch = temp;
	} else {
		OF_DC_LOG("of v_front_porch %d\n", temp);
		goto parse_modes_fail;
	}
	/* Optional property. Don't fail if not specified */
	if (!of_property_read_u32(np, "vmode", &temp)) {
		modes->vmode = temp;
	}

	if (pins && default_out->n_out_pins) {
		for (i = 0; i < default_out->n_out_pins; i++) {
			switch (pins[i].name) {
			case TEGRA_DC_OUT_PIN_DATA_ENABLE:
				if (pins[i].pol == TEGRA_DC_OUT_PIN_POL_LOW)
					modes->flags |=
						TEGRA_DC_MODE_FLAG_NEG_DE;
				break;
			case TEGRA_DC_OUT_PIN_H_SYNC:
				if (pins[i].pol == TEGRA_DC_OUT_PIN_POL_LOW)
					modes->flags |=
						TEGRA_DC_MODE_FLAG_NEG_H_SYNC;
				break;
			case TEGRA_DC_OUT_PIN_V_SYNC:
				if (pins[i].pol == TEGRA_DC_OUT_PIN_POL_LOW)
					modes->flags |=
						TEGRA_DC_MODE_FLAG_NEG_V_SYNC;
				break;
			default:
				/* Ignore other pin setting */
				break;
			}
		}
	} else {
		/* Optional property. Don't fail if not specified */
		if (!of_property_read_u32(np, "flags", &temp))
			modes->flags = temp;
	}

	return 0;
parse_modes_fail:
	pr_err("a mode parameter parse fail!\n");
	return -EINVAL;
}

#if defined(CONFIG_FRAMEBUFFER_CONSOLE)
int parse_fbcon_default_mode(struct device_node *np_target_disp,
			struct platform_device *ndev,
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *fbcon_default_mode_np = NULL;
	int err = 0;

	fbcon_default_mode_np = of_get_child_by_name(np_target_disp,
					"nvidia,fbcon-default-mode");

	if (!fbcon_default_mode_np) {
		pdata->default_out->fbcon_default_mode = NULL;
	} else {
		struct tegra_dc_mode fbcon_dc_mode;
		memset(&fbcon_dc_mode, 0x0,
				sizeof(struct tegra_dc_mode));
		pdata->default_out->fbcon_default_mode =
			devm_kzalloc(&ndev->dev,
				sizeof(struct fb_videomode), GFP_KERNEL);
		if (!pdata->default_out->fbcon_default_mode) {
			pr_err("%s: not enough memory for fbcon mode\n",
						__func__);
			err = -ENOMEM;
			goto fail_fbcon_parse;
		}

		err = parse_modes(pdata->default_out,
				fbcon_default_mode_np, &fbcon_dc_mode);
		if (err) {
			pr_err("%s: failed to parse fbcon mode\n", __func__);
			goto fail_fbcon_parse;
		}

		err = tegra_dc_to_fb_videomode(
			pdata->default_out->fbcon_default_mode, &fbcon_dc_mode);
		if (err) {
			pr_err("%s: error converting fbcon mode\n", __func__);
			goto fail_fbcon_parse;
		}
	}

fail_fbcon_parse:
	return err;
}
#endif

static int parse_cmu_data(struct device_node *np,
	struct tegra_dc_cmu *cmu)
{
	u16 *csc_parse;
	u16 *addr_cmu_lut1;
	u8 *addr_cmu_lut2;
	struct property *prop;
	const __be32 *p;
	u32 u;
	int csc_count = 0;
	int lut1_count = 0;
	int lut2_count = 0;

	memcpy(cmu, &default_cmu, sizeof(struct tegra_dc_cmu));

	csc_parse = &(cmu->csc.krr);
	addr_cmu_lut1 = &(cmu->lut1[0]);
	addr_cmu_lut2 = &(cmu->lut2[0]);

	of_property_for_each_u32(np, "nvidia,cmu-csc", prop, p, u)
		csc_count++;
	if (csc_count >
		(sizeof(cmu->csc) / sizeof(cmu->csc.krr))) {
		pr_err("cmu csc overflow\n");
		return -EINVAL;
	} else {
		of_property_for_each_u32(np,
			"nvidia,cmu-csc", prop, p, u) {
			OF_DC_LOG("cmu csc 0x%x\n", u);
			*(csc_parse++) = (u16)u;
		}
	}

	of_property_for_each_u32(np, "nvidia,cmu-lut1", prop, p, u)
		lut1_count++;
	if (lut1_count >
		(sizeof(cmu->lut1) / sizeof(cmu->lut1[0]))) {
		pr_err("cmu lut1 overflow\n");
		return -EINVAL;
	} else {
		of_property_for_each_u32(np, "nvidia,cmu-lut1",
			prop, p, u) {
			/* OF_DC_LOG("cmu lut1 0x%x\n", u); */
			*(addr_cmu_lut1++) = (u16)u;
		}
	}

	of_property_for_each_u32(np, "nvidia,cmu-lut2", prop, p, u)
		lut2_count++;
	if (lut2_count >
		(sizeof(cmu->lut2) / sizeof(cmu->lut2[0]))) {
		pr_err("cmu lut2 overflow\n");
		return -EINVAL;
	} else {
		of_property_for_each_u32(np, "nvidia,cmu-lut2",
			prop, p, u) {
			/* OF_DC_LOG("cmu lut2 0x%x\n", u); */
			*(addr_cmu_lut2++) = (u8)u;
		}
	}
	return 0;
}


static int parse_nvdisp_win_csc_data(struct device_node *np,
				struct tegra_dc_nvdisp_cmu *nvdisp_cmu)
{
	u32 u;
	u32 *csc_coeff = &(nvdisp_cmu->panel_csc.r2r);
	struct property *prop;
	const __be32 *p;
	int parsed_coeff_cnt = 0;
	int req_coeff_cnt = sizeof(nvdisp_cmu->panel_csc) /
				sizeof(nvdisp_cmu->panel_csc.r2r);

	req_coeff_cnt -= 1; /* subtract csc_enable */
	of_property_for_each_u32(np, "nvidia,panel-csc", prop, p, u)
		parsed_coeff_cnt++;

	if (parsed_coeff_cnt != req_coeff_cnt) {
		pr_err("invalid panel csc matrix. Coeffs req=%d, parsed=%d\n",
			req_coeff_cnt, parsed_coeff_cnt);
		return -EINVAL;
	}

	of_property_for_each_u32(np, "nvidia,panel-csc", prop, p, u) {
		OF_DC_LOG("panel csc 0x%x\n", u);
		*(csc_coeff++) = u;
	}
	nvdisp_cmu->panel_csc.csc_enable = true;

	return 0;
}

static int parse_nvdisp_cmu_data(struct device_node *np,
				struct tegra_dc_nvdisp_cmu *nvdisp_cmu)
{
	u64 *addr_nvdisp_cmu_lut;
	struct property *prop;
	const __be32 *p;
	u32 u, index = 0, lut_count = 0;
	u64 lutvalue = 0;
	int err;

	addr_nvdisp_cmu_lut = &(nvdisp_cmu->rgb[0]);

	of_property_for_each_u32(np, "nvidia,cmu-lut", prop, p, u)
		lut_count++;
	/* Each Index is being represented by 3 consecutive 16 bit values
	 * for RED, GREEN and BLUE in DT.
	 * 1024 LUT indicies will be represented using 3072 entires in DT
	 */
	if ((lut_count / 3) > (ARRAY_SIZE(nvdisp_cmu->rgb))) {
		pr_err("nvdisp_cmu lut overflow\n");
		return -EINVAL;
	} else {
		/* RED, GREEN, BLUE to read and place in a 64bit variable
		 * to pass to hw register
		 */
		of_property_for_each_u32(np, "nvidia,cmu-lut",
			prop, p, u) {
			OF_DC_LOG("0x%x\n", u);
			lutvalue = (u64) u;
			switch (index % 3) {
			case 0: /* red */
				*(addr_nvdisp_cmu_lut) = lutvalue;
				break;
			case 1: /* green */
				*(addr_nvdisp_cmu_lut) |= lutvalue << 16;
				break;
			case 2: /* blue */
				*(addr_nvdisp_cmu_lut++) |= lutvalue << 32;
				break;
			}
			index += 1;
		}
	}

	err = parse_nvdisp_win_csc_data(np, nvdisp_cmu);
	if (err) {
		pr_err("parsing nvdisp_win_csc failed\n");
		return err;
	}

	return 0;
}

struct tegra_dsi_cmd *dsi_parse_cmd_dt(struct device *dev,
				const struct device_node *node,
				struct property *prop,
				u32 n_cmd)
{
	struct tegra_dsi_cmd *dsi_cmd = NULL, *temp;
	__be32 *prop_val_ptr;
	u32 cnt = 0, i = 0;
	u8 arg1 = 0, arg2 = 0, arg3 = 0;
	bool long_pkt = false;

	if (!n_cmd)
		return NULL;

	if (!prop)
		return NULL;

	prop_val_ptr = prop->value;

	dsi_cmd = devm_kzalloc(dev, sizeof(*dsi_cmd) * n_cmd,
				GFP_KERNEL);
	if (!dsi_cmd) {
		pr_err("dsi: cmd memory allocation failed\n");
		return ERR_PTR(-ENOMEM);
	}
	temp = dsi_cmd;

	for (cnt  = 0; cnt < n_cmd; cnt++, temp++) {
		temp->cmd_type = be32_to_cpu(*prop_val_ptr++);
		if ((temp->cmd_type == TEGRA_DSI_PACKET_CMD) ||
			(temp->cmd_type ==
			TEGRA_DSI_PACKET_VIDEO_VBLANK_CMD)) {
			temp->data_id = be32_to_cpu(*prop_val_ptr++);
			arg1 = be32_to_cpu(*prop_val_ptr++);
			arg2 = be32_to_cpu(*prop_val_ptr++);
			prop_val_ptr++; /* skip ecc */
			long_pkt = (temp->data_id == DSI_GENERIC_LONG_WRITE ||
				temp->data_id == DSI_DCS_LONG_WRITE ||
				temp->data_id == DSI_NULL_PKT_NO_DATA ||
				temp->data_id == DSI_BLANKING_PKT_NO_DATA) ?
				true : false;
			if (!long_pkt && (temp->cmd_type ==
				TEGRA_DSI_PACKET_VIDEO_VBLANK_CMD))
				arg3 = be32_to_cpu(*prop_val_ptr++);
			if (long_pkt) {
				temp->sp_len_dly.data_len =
					(arg2 << NUMOF_BIT_PER_BYTE) | arg1;
				temp->pdata = devm_kzalloc(dev,
					temp->sp_len_dly.data_len, GFP_KERNEL);
				for (i = 0; i < temp->sp_len_dly.data_len; i++)
					(temp->pdata)[i] =
					be32_to_cpu(*prop_val_ptr++);
				prop_val_ptr += 2; /* skip checksum */
			} else {
				temp->sp_len_dly.sp.data0 = arg1;
				temp->sp_len_dly.sp.data1 = arg2;
				if (temp->cmd_type ==
					TEGRA_DSI_PACKET_VIDEO_VBLANK_CMD)
					temp->club_cmd = (bool)arg3;
			}
		} else if (temp->cmd_type == TEGRA_DSI_DELAY_MS) {
			temp->sp_len_dly.delay_ms =
				be32_to_cpu(*prop_val_ptr++);
		} else if (temp->cmd_type == TEGRA_DSI_DELAY_US) {
			temp->sp_len_dly.delay_us =
				be32_to_cpu(*prop_val_ptr++);
		} else if (temp->cmd_type == TEGRA_DSI_SEND_FRAME) {
			temp->sp_len_dly.frame_cnt =
				be32_to_cpu(*prop_val_ptr++);
		} else if (temp->cmd_type == TEGRA_DSI_GPIO_SET) {
			temp->sp_len_dly.gpio =
				be32_to_cpu(*prop_val_ptr++);
			temp->data_id =
				be32_to_cpu(*prop_val_ptr++);
		}
	}

	return dsi_cmd;
}

static struct tegra_dsi_cmd *tegra_dsi_parse_cmd_dt(
					struct platform_device *ndev,
					const struct device_node *node,
					struct property *prop,
					u32 n_cmd)
{
	struct tegra_dsi_cmd *dsi_cmd = NULL;

	dsi_cmd = dsi_parse_cmd_dt(&ndev->dev, node, prop, n_cmd);

	return dsi_cmd;
}

static const u32 *tegra_dsi_parse_pkt_seq_dt(struct platform_device *ndev,
						struct device_node *node,
						struct property *prop)
{
	__be32 *prop_val_ptr;
	u32 *pkt_seq;
	int line, i;

#define LINE_STOP 0xff

	if (!prop)
		return NULL;

	pkt_seq = devm_kzalloc(&ndev->dev,
				sizeof(u32) * NUMOF_PKT_SEQ, GFP_KERNEL);
	if (!pkt_seq) {
		dev_err(&ndev->dev,
			"dsi: pkt seq memory allocation failed\n");
		return ERR_PTR(-ENOMEM);
	}
	prop_val_ptr = prop->value;
	for (line = 0; line < NUMOF_PKT_SEQ; line += 2) {
		/* compute line value from dt line */
		for (i = 0;; i += 2) {
			u32 cmd = be32_to_cpu(*prop_val_ptr++);
			if (cmd == LINE_STOP)
				break;
			else if (cmd == PKT_LP)
				pkt_seq[line] |= PKT_LP;
			else {
				u32 len = be32_to_cpu(*prop_val_ptr++);
				if (i == 0) /* PKT_ID0 */
					pkt_seq[line] |=
						PKT_ID0(cmd) | PKT_LEN0(len);
				if (i == 2) /* PKT_ID1 */
					pkt_seq[line] |=
						PKT_ID1(cmd) | PKT_LEN1(len);
				if (i == 4) /* PKT_ID2 */
					pkt_seq[line] |=
						PKT_ID2(cmd) | PKT_LEN2(len);
				if (i == 6) /* PKT_ID3 */
					pkt_seq[line + 1] |=
						PKT_ID3(cmd) | PKT_LEN3(len);
				if (i == 8) /* PKT_ID4 */
					pkt_seq[line + 1] |=
						PKT_ID4(cmd) | PKT_LEN4(len);
				if (i == 10) /* PKT_ID5 */
					pkt_seq[line + 1] |=
						PKT_ID5(cmd) | PKT_LEN5(len);
			}
		}
	}

#undef LINE_STOP

	return pkt_seq;
}

static int parse_dsi_settings(struct platform_device *ndev,
	struct tegra_dc_platform_data *pdata, struct tegra_dc_out *def_out)
{
	u32 temp;
	int ret = 0;
	int dsi_te_gpio = 0;
	int bl_name_len = 0;
	const __be32 *p;
	struct property *prop;
	struct tegra_dsi_out *dsi = def_out->dsi;
	struct device_node *np_dsi = pdata->conn_np;
	struct device_node *np_dsi_panel = pdata->panel_np;

	if (!of_property_read_u32(np_dsi, "nvidia,dsi-controller-vs", &temp)) {
		dsi->controller_vs = (u8)temp;
		if (temp == DSI_VS_0) {
			OF_DC_LOG("dsi controller vs DSI_VS_0\n");
		} else if (temp == DSI_VS_1) {
			OF_DC_LOG("dsi controller vs DSI_VS_1\n");
		} else {
			dev_err(&ndev->dev, "%s: Invalid version:%d\n",
					__func__, temp);
			return -EINVAL;
		}
	}

	if (!of_property_read_u32(np_dsi,
			"nvidia,enable-hs-clk-in-lp-mode", &temp)) {
		dsi->enable_hs_clock_on_lp_cmd_mode = (u8)temp;
		OF_DC_LOG("Enable hs clock in lp mode %d\n",
			dsi->enable_hs_clock_on_lp_cmd_mode);
	}

	if (of_property_read_bool(np_dsi,
			"nvidia,drm-override-disable")) {
		dsi->drm_override_disable = true;
		OF_DC_LOG("DRM override disable is true\n");
	}

	if (of_property_read_bool(np_dsi,
			"nvidia,set-max-dsi-timeout")) {
		dsi->set_max_timeout = true;
		OF_DC_LOG("Set max DSI timeout %d\n",
			dsi->set_max_timeout);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-refresh-rate-adj", &temp)) {
		dsi->refresh_rate_adj = (u8)temp;
		OF_DC_LOG("DSI refresh rate adjustment %d\n",
			dsi->refresh_rate_adj);
	}

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-n-data-lanes", &temp)) {
		dsi->n_data_lanes = (u8)temp;
		OF_DC_LOG("n data lanes %d\n", dsi->n_data_lanes);
	}
	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-video-burst-mode", &temp)) {
		dsi->video_burst_mode = (u8)temp;
		if (temp == TEGRA_DSI_VIDEO_NONE_BURST_MODE)
			OF_DC_LOG("dsi video NON_BURST_MODE\n");
		else if (temp == TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END)
			OF_DC_LOG("dsi video NONE_BURST_MODE_WITH_SYNC_END\n");
		else if (temp == TEGRA_DSI_VIDEO_BURST_MODE_LOWEST_SPEED)
			OF_DC_LOG("dsi video BURST_MODE_LOWEST_SPEED\n");
		else if (temp == TEGRA_DSI_VIDEO_BURST_MODE_LOW_SPEED)
			OF_DC_LOG("dsi video BURST_MODE_LOW_SPEED\n");
		else if (temp == TEGRA_DSI_VIDEO_BURST_MODE_MEDIUM_SPEED)
			OF_DC_LOG("dsi video BURST_MODE_MEDIUM_SPEED\n");
		else if (temp == TEGRA_DSI_VIDEO_BURST_MODE_FAST_SPEED)
			OF_DC_LOG("dsi video BURST_MODE_FAST_SPEED\n");
		else if (temp == TEGRA_DSI_VIDEO_BURST_MODE_FASTEST_SPEED)
			OF_DC_LOG("dsi video BURST_MODE_FASTEST_SPEED\n");
		else {
			pr_err("invalid dsi video burst mode\n");
			ret = -EINVAL;
			goto parse_dsi_settings_fail;
		}
	}
	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-pixel-format", &temp)) {
		dsi->pixel_format = (u8)temp;
		if (temp == TEGRA_DSI_PIXEL_FORMAT_16BIT_P)
			OF_DC_LOG("dsi pixel format 16BIT_P\n");
		else if (temp == TEGRA_DSI_PIXEL_FORMAT_18BIT_P)
			OF_DC_LOG("dsi pixel format 18BIT_P\n");
		else if (temp == TEGRA_DSI_PIXEL_FORMAT_18BIT_NP)
			OF_DC_LOG("dsi pixel format 18BIT_NP\n");
		else if (temp == TEGRA_DSI_PIXEL_FORMAT_24BIT_P)
			OF_DC_LOG("dsi pixel format 24BIT_P\n");
		else if (temp == TEGRA_DSI_PIXEL_FORMAT_8BIT_DSC)
			OF_DC_LOG("dsi pixel format 8BIT_DSC\n");
		else if (temp == TEGRA_DSI_PIXEL_FORMAT_12BIT_DSC)
			OF_DC_LOG("dsi pixel format 12BIT_DSC\n");
		else if (temp == TEGRA_DSI_PIXEL_FORMAT_16BIT_DSC)
			OF_DC_LOG("dsi pixel format 16BIT_DSC\n");
		else {
			pr_err("invalid dsi pixel format\n");
			ret = -EINVAL;
			goto parse_dsi_settings_fail;
		}
	}
	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-refresh-rate", &temp)) {
		dsi->refresh_rate = (u8)temp;
		OF_DC_LOG("dsi refresh rate %d\n", dsi->refresh_rate);
	}
	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-rated-refresh-rate", &temp)) {
		dsi->rated_refresh_rate = (u8)temp;
		OF_DC_LOG("dsi rated refresh rate %d\n",
				dsi->rated_refresh_rate);
	}
	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-virtual-channel", &temp)) {
		dsi->virtual_channel = (u8)temp;
		if (temp == TEGRA_DSI_VIRTUAL_CHANNEL_0)
			OF_DC_LOG("dsi virtual channel 0\n");
		else if (temp == TEGRA_DSI_VIRTUAL_CHANNEL_1)
			OF_DC_LOG("dsi virtual channel 1\n");
		else if (temp == TEGRA_DSI_VIRTUAL_CHANNEL_2)
			OF_DC_LOG("dsi virtual channel 2\n");
		else if (temp == TEGRA_DSI_VIRTUAL_CHANNEL_3)
			OF_DC_LOG("dsi virtual channel 3\n");
		else {
			pr_err("invalid dsi virtual ch\n");
			ret = -EINVAL;
			goto parse_dsi_settings_fail;
		}
	}
	if (!of_property_read_u32(np_dsi_panel, "nvidia,dsi-instance", &temp)) {
		dsi->dsi_instance = (u8)temp;
		if (temp == tegra_dc_get_dsi_instance_0())
			OF_DC_LOG("dsi instance 0\n");
		else if (temp == tegra_dc_get_dsi_instance_1())
			OF_DC_LOG("dsi instance 1\n");
		else {
			pr_err("invalid dsi instance\n");
			ret = -EINVAL;
			goto parse_dsi_settings_fail;
		}
	}
	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-panel-reset", &temp)) {
		dsi->panel_reset = (u8)temp;
		OF_DC_LOG("dsi panel reset %d\n", dsi->panel_reset);
	}
	if (!of_property_read_u32(np_dsi_panel,
				"nvidia,dsi-te-polarity-low", &temp)) {
		dsi->te_polarity_low = (u8)temp;
		OF_DC_LOG("dsi panel te polarity low %d\n",
			dsi->te_polarity_low);
	}
	if (!of_property_read_u32(np_dsi_panel,
				"nvidia,dsi-lp00-pre-panel-wakeup", &temp)) {
		dsi->lp00_pre_panel_wakeup = (u8)temp;
		OF_DC_LOG("dsi panel lp00 pre panel wakeup %d\n",
				dsi->lp00_pre_panel_wakeup);
	}
	if (of_find_property(np_dsi_panel,
		"nvidia,dsi-bl-name", &bl_name_len)) {
		dsi->bl_name = devm_kzalloc(&ndev->dev,
				sizeof(u8) * bl_name_len, GFP_KERNEL);
		if (!of_property_read_string(np_dsi_panel,
				"nvidia,dsi-bl-name",
				(const char **)&dsi->bl_name))
			OF_DC_LOG("dsi panel bl name %s\n", dsi->bl_name);
		else {
			pr_err("dsi error parsing bl name\n");
			devm_kfree(&ndev->dev, dsi->bl_name);
		}
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-ganged-type", &temp)) {
		dsi->ganged_type = (u8)temp;
		OF_DC_LOG("dsi ganged_type %d\n", dsi->ganged_type);
		/* Set pixel width to 1 by default for even-odd split */
		if (dsi->ganged_type == TEGRA_DSI_GANGED_SYMMETRIC_EVEN_ODD)
			dsi->even_odd_split_width = 1;
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-even-odd-pixel-width", &temp)) {
		dsi->even_odd_split_width = temp;
		OF_DC_LOG("dsi pixel width for even/odd split %d\n",
				dsi->even_odd_split_width);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-ganged-overlap", &temp)) {
		dsi->ganged_overlap = (u16)temp;
		OF_DC_LOG("dsi ganged overlap %d\n", dsi->ganged_overlap);
		if (!dsi->ganged_type)
			pr_warn("specified ganged overlap, but no ganged type\n");
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-ganged-swap-links", &temp)) {
		dsi->ganged_swap_links = (bool)temp;
		OF_DC_LOG("dsi ganged swapped links %d\n",
			dsi->ganged_swap_links);
		if (!dsi->ganged_type)
			pr_warn("specified ganged swapped links, but no ganged type\n");
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-ganged-write-to-all-links", &temp)) {
		dsi->ganged_write_to_all_links = (bool)temp;
		OF_DC_LOG("dsi ganged write to both links %d\n",
			dsi->ganged_write_to_all_links);
		if (!dsi->ganged_type)
			pr_warn("specified ganged write to all links, but no ganged type\n");
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-split-link-type", &temp)) {
		dsi->split_link_type = (u8)temp;
		OF_DC_LOG("dsi split link type %d\n", dsi->split_link_type);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-suspend-aggr", &temp)) {
		dsi->suspend_aggr = (u8)temp;
		OF_DC_LOG("dsi suspend_aggr %d\n", dsi->suspend_aggr);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-edp-bridge", &temp)) {
		dsi->dsi2edp_bridge_enable = (bool)temp;
		OF_DC_LOG("dsi2edp_bridge_enabled %d\n",
			dsi->dsi2edp_bridge_enable);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-lvds-bridge", &temp)) {
		dsi->dsi2lvds_bridge_enable = (bool)temp;
		OF_DC_LOG("dsi-lvds_bridge_enabled %d\n",
			dsi->dsi2lvds_bridge_enable);
	}

	dsi_te_gpio = of_get_named_gpio(np_dsi_panel, "nvidia,dsi-te-gpio", 0);
	if (gpio_is_valid(dsi_te_gpio)) {
		dsi->te_gpio = dsi_te_gpio;
		OF_DC_LOG("dsi te_gpio %d\n", dsi_te_gpio);
	}

	of_property_for_each_u32(np_dsi_panel, "nvidia,dsi-dpd-pads",
		prop, p, temp) {
		dsi->dpd_dsi_pads |= (u32)temp;
		OF_DC_LOG("dpd_dsi_pads %u\n", dsi->dpd_dsi_pads);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-power-saving-suspend", &temp)) {
		dsi->power_saving_suspend = (bool)temp;
		OF_DC_LOG("dsi power saving suspend %d\n",
			dsi->power_saving_suspend);
	}
	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-ulpm-not-support", &temp)) {
		dsi->ulpm_not_supported = (bool)temp;
		OF_DC_LOG("dsi ulpm_not_supported %d\n",
			dsi->ulpm_not_supported);
	}
	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-video-data-type", &temp)) {
		dsi->video_data_type = (u8)temp;
		if (temp == TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE)
			OF_DC_LOG("dsi video type VIDEO_MODE\n");
		else if (temp == TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE)
			OF_DC_LOG("dsi video type COMMAND_MODE\n");
		else {
			pr_err("invalid dsi video data type\n");
			ret = -EINVAL;
			goto parse_dsi_settings_fail;
		}
	}
	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-video-clock-mode", &temp)) {
		dsi->video_clock_mode = (u8)temp;
		if (temp == TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS)
			OF_DC_LOG("dsi video clock mode CONTINUOUS\n");
		else if (temp == TEGRA_DSI_VIDEO_CLOCK_TX_ONLY)
			OF_DC_LOG("dsi video clock mode TX_ONLY\n");
		else {
			pr_err("invalid dsi video clk mode\n");
			ret = -EINVAL;
			goto parse_dsi_settings_fail;
		}
	}
	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-n-init-cmd", &temp)) {
		dsi->n_init_cmd = (u16)temp;
		OF_DC_LOG("dsi n_init_cmd %d\n",
			dsi->n_init_cmd);
	}
	dsi->dsi_init_cmd =
		tegra_dsi_parse_cmd_dt(ndev, np_dsi_panel,
			of_find_property(np_dsi_panel,
			"nvidia,dsi-init-cmd", NULL),
			dsi->n_init_cmd);
	if (dsi->n_init_cmd &&
		IS_ERR_OR_NULL(dsi->dsi_init_cmd)) {
		dev_err(&ndev->dev,
			"dsi: copy init cmd from dt failed\n");
		ret = -EINVAL;
		goto parse_dsi_settings_fail;
	};

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-n-postvideo-cmd", &temp)) {
		dsi->n_postvideo_cmd = (u16)temp;
		OF_DC_LOG("dsi n_postvideo_cmd %d\n",
			dsi->n_postvideo_cmd);
	}
	dsi->dsi_postvideo_cmd =
		tegra_dsi_parse_cmd_dt(ndev, np_dsi_panel,
			of_find_property(np_dsi_panel,
			"nvidia,dsi-postvideo-cmd", NULL),
			dsi->n_postvideo_cmd);
	if (dsi->n_postvideo_cmd &&
		IS_ERR_OR_NULL(dsi->dsi_postvideo_cmd)) {
		dev_err(&ndev->dev,
			"dsi: copy postvideo cmd from dt failed\n");
		goto parse_dsi_settings_fail;
	};

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-n-suspend-cmd", &temp)) {
		dsi->n_suspend_cmd = (u16)temp;
		OF_DC_LOG("dsi n_suspend_cmd %d\n",
			dsi->n_suspend_cmd);
	}
	dsi->dsi_suspend_cmd =
		tegra_dsi_parse_cmd_dt(ndev, np_dsi_panel,
			of_find_property(np_dsi_panel,
			"nvidia,dsi-suspend-cmd", NULL),
			dsi->n_suspend_cmd);
	if (dsi->n_suspend_cmd &&
		IS_ERR_OR_NULL(dsi->dsi_suspend_cmd)) {
		dev_err(&ndev->dev,
			"dsi: copy suspend cmd from dt failed\n");
		ret = -EINVAL;
		goto parse_dsi_settings_fail;
	};

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-n-early-suspend-cmd", &temp)) {
		dsi->n_early_suspend_cmd = (u16)temp;
		OF_DC_LOG("dsi n_early_suspend_cmd %d\n",
			dsi->n_early_suspend_cmd);
	}
	dsi->dsi_early_suspend_cmd =
		tegra_dsi_parse_cmd_dt(ndev, np_dsi_panel,
			of_find_property(np_dsi_panel,
			"nvidia,dsi-early-suspend-cmd", NULL),
			dsi->n_early_suspend_cmd);
	if (dsi->n_early_suspend_cmd &&
		IS_ERR_OR_NULL(dsi->dsi_early_suspend_cmd)) {
		dev_err(&ndev->dev,
			"dsi: copy early suspend cmd from dt failed\n");
		ret = -EINVAL;
		goto parse_dsi_settings_fail;
	};

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-suspend-stop-stream-late", &temp)) {
		dsi->suspend_stop_stream_late = (bool)temp;
		OF_DC_LOG("suspend stop stream late %d\n",
			dsi->suspend_stop_stream_late);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-n-late-resume-cmd", &temp)) {
		dsi->n_late_resume_cmd = (u16)temp;
		OF_DC_LOG("dsi n_late_resume_cmd %d\n",
			dsi->n_late_resume_cmd);
	}
	dsi->dsi_late_resume_cmd =
		tegra_dsi_parse_cmd_dt(ndev, np_dsi_panel,
			of_find_property(np_dsi_panel,
			"nvidia,dsi-late-resume-cmd", NULL),
			dsi->n_late_resume_cmd);
	if (dsi->n_late_resume_cmd &&
		IS_ERR_OR_NULL(dsi->dsi_late_resume_cmd)) {
		dev_err(&ndev->dev,
			"dsi: copy late resume cmd from dt failed\n");
		ret = -EINVAL;
		goto parse_dsi_settings_fail;
	};

	dsi->pkt_seq =
		tegra_dsi_parse_pkt_seq_dt(ndev, np_dsi_panel,
			of_find_property(np_dsi_panel,
			"nvidia,dsi-pkt-seq", NULL));
	if (IS_ERR(dsi->pkt_seq)) {
		dev_err(&ndev->dev, "dsi pkt seq from dt fail\n");
		ret = -EINVAL;
		goto parse_dsi_settings_fail;
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-hsdexit", &temp)) {
		dsi->phy_timing.t_hsdexit_ns = (u16)temp;
		OF_DC_LOG("phy t_hsdexit_ns %d\n",
			dsi->phy_timing.t_hsdexit_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-hstrail", &temp)) {
		dsi->phy_timing.t_hstrail_ns = (u16)temp;
		OF_DC_LOG("phy t_hstrail_ns %d\n",
			dsi->phy_timing.t_hstrail_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-datzero", &temp)) {
		dsi->phy_timing.t_datzero_ns = (u16)temp;
		OF_DC_LOG("phy t_datzero_ns %d\n",
			dsi->phy_timing.t_datzero_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-phy-hsprepare", &temp)) {
		dsi->phy_timing.t_hsprepare_ns = (u16)temp;
		OF_DC_LOG("phy t_hsprepare_ns %d\n",
			dsi->phy_timing.t_hsprepare_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-phy-clktrail", &temp)) {
		dsi->phy_timing.t_clktrail_ns = (u16)temp;
		OF_DC_LOG("phy t_clktrail_ns %d\n",
			dsi->phy_timing.t_clktrail_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-clkpost", &temp)) {
		dsi->phy_timing.t_clkpost_ns = (u16)temp;
		OF_DC_LOG("phy t_clkpost_ns %d\n",
			dsi->phy_timing.t_clkpost_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-clkzero", &temp)) {
		dsi->phy_timing.t_clkzero_ns = (u16)temp;
		OF_DC_LOG("phy t_clkzero_ns %d\n",
			dsi->phy_timing.t_clkzero_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-tlpx", &temp)) {
		dsi->phy_timing.t_tlpx_ns = (u16)temp;
		OF_DC_LOG("phy t_tlpx_ns %d\n",
			dsi->phy_timing.t_tlpx_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,dsi-phy-clkprepare", &temp)) {
		dsi->phy_timing.t_clkprepare_ns = (u16)temp;
		OF_DC_LOG("phy t_clkprepare_ns %d\n",
			dsi->phy_timing.t_clkprepare_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-clkpre", &temp)) {
		dsi->phy_timing.t_clkpre_ns = (u16)temp;
		OF_DC_LOG("phy t_clkpre_ns %d\n",
			dsi->phy_timing.t_clkpre_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-wakeup", &temp)) {
		dsi->phy_timing.t_wakeup_ns = (u16)temp;
		OF_DC_LOG("phy t_wakeup_ns %d\n",
			dsi->phy_timing.t_wakeup_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-taget", &temp)) {
		dsi->phy_timing.t_taget_ns = (u16)temp;
		OF_DC_LOG("phy t_taget_ns %d\n",
			dsi->phy_timing.t_taget_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-tasure", &temp)) {
		dsi->phy_timing.t_tasure_ns = (u16)temp;
		OF_DC_LOG("phy t_tasure_ns %d\n",
			dsi->phy_timing.t_tasure_ns);
	}

	if (!of_property_read_u32(np_dsi_panel,
		"nvidia,dsi-phy-tago", &temp)) {
		dsi->phy_timing.t_tago_ns = (u16)temp;
		OF_DC_LOG("phy t_tago_ns %d\n",
			dsi->phy_timing.t_tago_ns);
	}

	if (!of_find_property(np_dsi_panel,
		"nvidia,dsi-boardinfo", NULL)) {
		of_property_read_u32_index(np_dsi_panel,
			"nvidia,dsi-boardinfo", 0,
			&dsi->boardinfo.platform_boardid);
		of_property_read_u32_index(np_dsi_panel,
			"nvidia,dsi-boardinfo", 1,
			&dsi->boardinfo.platform_boardversion);
		of_property_read_u32_index(np_dsi_panel,
			"nvidia,dsi-boardinfo", 2,
			&dsi->boardinfo.display_boardid);
		of_property_read_u32_index(np_dsi_panel,
			"nvidia,dsi-boardinfo", 3,
			&dsi->boardinfo.display_boardversion);

		OF_DC_LOG("boardinfo platform_boardid = %d \
					 platform_boardversion = %d \
					 display_boardid = %d \
					 display_boardversion = %d\n",
					 dsi->boardinfo.platform_boardid,
					 dsi->boardinfo.platform_boardversion,
					 dsi->boardinfo.display_boardid,
					 dsi->boardinfo.display_boardversion);
	}

	if (of_property_read_bool(np_dsi_panel,
			"nvidia,enable-link-compression"))
		def_out->dsc_en = true;

	if (of_property_read_bool(np_dsi_panel,
			"nvidia,enable-dual-dsc"))
		def_out->dual_dsc_en = true;

	if (of_property_read_bool(np_dsi_panel,
			"nvidia,enable-block-pred"))
		def_out->en_block_pred = true;

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,slice-height", &temp))
		def_out->slice_height = (u32)temp;

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,num-of-slices", &temp))
		def_out->num_of_slices = (u8)temp;

	if (!of_property_read_u32(np_dsi_panel,
			"nvidia,comp-rate", &temp))
		def_out->dsc_bpp = (u8)temp;

	if (of_property_read_bool(np_dsi, "nvidia,dsi-csi-loopback")) {
		dsi->dsi_csi_loopback = 1;
		OF_DC_LOG("DSI CSI loopback %d\n",
			dsi->dsi_csi_loopback);
	}

parse_dsi_settings_fail:
	return ret;
}

static int parse_lt_setting(struct device_node *np, u8 *addr)
{
	u32 u, temp;
	int i = 0;
	const __be32 *p;
	struct property *prop;
	struct tegra_dc_dp_lt_settings *lt_setting_addr;
	int n_drive_current =
		sizeof(lt_setting_addr->drive_current)/
		sizeof(lt_setting_addr->drive_current[0]);
	int n_lane_preemphasis =
		sizeof(lt_setting_addr->lane_preemphasis)/
		sizeof(lt_setting_addr->lane_preemphasis[0]);
	int n_post_cursor =
		sizeof(lt_setting_addr->post_cursor)/
		sizeof(lt_setting_addr->post_cursor[0]);


	lt_setting_addr = (struct tegra_dc_dp_lt_settings *)addr;

	of_property_for_each_u32(np, "nvidia,drive-current", prop, p, u) {
		lt_setting_addr->drive_current[i] = (u32)u;
		i++;
	}
	if (n_drive_current != i)
		return -EINVAL;
	i = 0;
	of_property_for_each_u32(np, "nvidia,lane-preemphasis", prop, p, u) {
		lt_setting_addr->lane_preemphasis[i] = (u32)u;
		i++;
	}
	if (n_lane_preemphasis != i)
		return -EINVAL;
	i = 0;
	of_property_for_each_u32(np, "nvidia,post-cursor", prop, p, u) {
		lt_setting_addr->post_cursor[i] = (u32)u;
		i++;
	}
	if (n_post_cursor != i)
		return -EINVAL;

	if (!of_property_read_u32(np, "nvidia,tx-pu", &temp)) {
		lt_setting_addr->tx_pu = (u32)temp;
		OF_DC_LOG("tx_pu %d\n", temp);
	}
	if (!of_property_read_u32(np, "nvidia,load-adj", &temp)) {
		lt_setting_addr->load_adj = (u32)temp;
		OF_DC_LOG("load_adj %d\n", temp);
	}
	return 0;
}

static const char * const lt_data_name[] = {
	"tegra-dp-vs-regs",
	"tegra-dp-pe-regs",
	"tegra-dp-pc-regs",
	"tegra-dp-tx-pu",
};

static const char * const lt_data_child_name[] = {
	"pc2_l0",
	"pc2_l1",
	"pc2_l2",
	"pc2_l3",
};

/*
 * get lt_data from platform device tree
 */
static int parse_lt_data(struct device_node *np,
	struct tegra_dp_out *dpout)
{
	int i, j, k, m, n;
	struct device_node *entry;
	struct property *prop;
	const __be32 *p;
	u32 u;
	u32 temp[10];

	for (i = 0; i < dpout->n_lt_data; i++) {
		entry = of_get_child_by_name(np, lt_data_name[i]);
		if (!entry) {
			pr_info("%s: lt-data node has no child node named %s\n",
					__func__, lt_data_name[i]);
			return -1;
		}

		dpout->lt_data[i].name = lt_data_name[i];
		for (j = 0; j < 4; j++) {
			k = 0;
			memset(temp, 0, sizeof(temp));
			of_property_for_each_u32(entry,
				lt_data_child_name[j] , prop, p, u) {
				temp[k++] = (u32)u;
			}

			k = 0;
			for (m = 0; m < 4; m++) {
				for (n = 0; n < 4-m; n++) {
					dpout->lt_data[i].data[j][m][n] = temp[k++];
				}
			}
		}
		of_node_put(entry);
	}
	return 0;
}

/*
 * get SoC lt_data from header file, dp_lt.h
 */
static int set_lt_data(struct tegra_dp_out *dpout)
{
	int j, m, n;
	for (j = 0; j < 4; j++) {
		for (m = 0; m < 4; m++) {
			for (n = 0; n < 4-m; n++) {
				if (tegra_dc_is_nvdisplay())
					dpout->lt_data[DP_VS].data[j][m][n] =
					tegra_dp_vs_regs_nvdisplay[j][m][n];
				else
					dpout->lt_data[DP_VS].data[j][m][n] =
						tegra_dp_vs_regs[j][m][n];
				dpout->lt_data[DP_PE].data[j][m][n] = tegra_dp_pe_regs[j][m][n];
				dpout->lt_data[DP_PC].data[j][m][n] = tegra_dp_pc_regs[j][m][n];
				dpout->lt_data[DP_TX_PU].data[j][m][n] = tegra_dp_tx_pu[j][m][n];
			}
		}
	}
	return 0;
}

static int parse_dp_settings(struct platform_device *ndev,
	struct tegra_dc_platform_data *pdata, struct tegra_dc_out *def_out)
{
	u8 *addr;
	u32 temp;
	int ret = 0;
	struct device_node *np_dp_lt_set = NULL;
	struct device_node *np_lt_data = NULL;
	struct device_node *entry = NULL;
	struct tegra_dp_out *dpout = def_out->dp_out;
	struct device_node *np_dp_panel = pdata->panel_np;

	dpout->pc2_disabled = of_property_read_bool(np_dp_panel,
						    "nvidia,pc2-disabled");

	np_dp_lt_set = of_get_child_by_name(np_dp_panel, "dp-lt-settings");
	if (!np_dp_lt_set) {
		dev_info(&ndev->dev, "%s: No dp-lt-settings node\n", __func__);
	} else {
		int n_lt_settings = of_get_child_count(np_dp_lt_set);

		if (!n_lt_settings) {
			dev_info(&ndev->dev, "lt-settings node has no child node\n");
		} else {
			dpout->n_lt_settings = n_lt_settings;
			dpout->lt_settings = devm_kzalloc(&ndev->dev,
				n_lt_settings *
				sizeof(struct tegra_dc_dp_lt_settings),
				GFP_KERNEL);
			if (!dpout->lt_settings)
				goto parse_dp_settings_fail;

			addr = (u8 *)dpout->lt_settings;
			for_each_child_of_node(np_dp_lt_set, entry) {
				ret = parse_lt_setting(entry, addr);
				if (ret)
					goto parse_dp_settings_fail;
				addr += sizeof(struct tegra_dc_dp_lt_settings);
			}
		}
	}

	dpout->n_lt_data = 4;
	dpout->lt_data = devm_kzalloc(&ndev->dev,
		dpout->n_lt_data * sizeof(struct tegra_dc_lt_data), GFP_KERNEL);
	if (!dpout->lt_data)
		goto parse_dp_settings_fail;

	np_lt_data = of_get_child_by_name(np_dp_panel, "lt-data");
	if (!np_lt_data || !of_get_child_count(np_lt_data)) {
		dev_info(&ndev->dev, "No lt-data, using default setting\n");
		set_lt_data(dpout);
	} else {
		ret = parse_lt_data(np_lt_data, dpout);
		if (ret)
			goto parse_dp_settings_fail;
	}

	if (!of_property_read_u32(np_dp_panel, "nvidia,tx-pu-disable", &temp)) {
		dpout->tx_pu_disable = (bool)temp;
		OF_DC_LOG("tx_pu_disable %d\n", dpout->tx_pu_disable);
	}

	if (!of_property_read_u32(np_dp_panel, "nvidia,lanes", &temp)) {
		dpout->lanes = (int)temp;
		OF_DC_LOG("lanes %d\n", dpout->lanes);
	} else {
		dpout->lanes = 4;
		OF_DC_LOG("default lanes %d\n", dpout->lanes);
	}

	dpout->enhanced_framing_disable = of_property_read_bool(np_dp_panel,
					"nvidia,enhanced-framing-disable");
	OF_DC_LOG("enhanced-framing-disable %d\n",
			!!dpout->enhanced_framing_disable);

	if (!of_property_read_u32(np_dp_panel, "nvidia,link-bw", &temp)) {
		dpout->link_bw = (u8)temp;
		OF_DC_LOG("link_bw %d\n", dpout->link_bw);
	} else {
		dpout->link_bw = (tegra_dc_is_t19x()) ?
			NV_DPCD_MAX_LINK_BANDWIDTH_VAL_8_10_GBPS :
			NV_DPCD_MAX_LINK_BANDWIDTH_VAL_5_40_GBPS;
		OF_DC_LOG("default link_bw %d\n", dpout->link_bw);
	}

	of_node_put(np_lt_data);
	of_node_put(np_dp_lt_set);
	return ret;

parse_dp_settings_fail:
	of_node_put(np_lt_data);
	of_node_put(np_dp_lt_set);
	devm_kfree(&ndev->dev, dpout->lt_data);
	devm_kfree(&ndev->dev, dpout->lt_settings);
	return ret;
}

static int dc_dp_out_enable(struct device *dev)
{
	int ret = 0;
	if (!of_dp_pwr) {
		of_dp_pwr = devm_regulator_get(dev, "vdd-dp-pwr");
		if (IS_ERR(of_dp_pwr)) {
			dev_warn(dev,
				"dp: couldn't get regulator vdd-dp-pwr\n");
			ret = PTR_ERR(of_dp_pwr);
			of_dp_pwr = NULL;
		}
	}
	if (!of_dp_pll) {
		of_dp_pll = devm_regulator_get(dev, "avdd-dp-pll");
		if (IS_ERR(of_dp_pll)) {
			dev_warn(dev,
				"dp: couldn't get regulator avdd-dp-pll\n");
			ret = PTR_ERR(of_dp_pll);
			of_dp_pll = NULL;
		}
	}
	if (!of_edp_sec_mode) {
		of_edp_sec_mode = devm_regulator_get(dev, "vdd-edp-sec-mode");
		if (IS_ERR(of_edp_sec_mode)) {
			dev_warn(dev,
				"dp: couldn't get regulator vdd-edp-sec-mode\n");
			ret = PTR_ERR(of_edp_sec_mode);
			of_edp_sec_mode = NULL;
		}
	}
	if (!of_dp_pad) {
		of_dp_pad = devm_regulator_get(dev, "vdd-dp-pad");
		if (IS_ERR(of_dp_pad)) {
			dev_warn(dev,
				"dp: couldn't get regulator vdd-dp-pad\n");
			ret = PTR_ERR(of_dp_pad);
			of_dp_pad = NULL;
		}
	}

	if (of_dp_pwr) {
		ret = regulator_enable(of_dp_pwr);
		if (ret < 0)
			dev_err(dev,
			"dp: couldn't enable regulator vdd-dp-pwr\n");
	}
	if (of_dp_pll) {
		ret = regulator_enable(of_dp_pll);
		if (ret < 0)
			dev_err(dev,
			"dp: couldn't enable regulator vdd-dp-pll\n");
	}
	if (of_edp_sec_mode) {
		ret = regulator_enable(of_edp_sec_mode);
		if (ret < 0)
			dev_err(dev,
			"dp: couldn't enable regulator vdd-edp-sec-mode\n");
	}
	if (of_dp_pad) {
		ret = regulator_enable(of_dp_pad);
		if (ret < 0)
			dev_err(dev,
			"dp: couldn't enable regulator vdd-dp-pad\n");
	}

	return ret;
}

static int dc_dp_out_disable(struct device *dev)
{
	if (of_dp_pwr) {
		regulator_disable(of_dp_pwr);
	}
	if (of_dp_pll) {
		regulator_disable(of_dp_pll);
	}
	if (of_edp_sec_mode) {
		regulator_disable(of_edp_sec_mode);
	}
	if (of_dp_pad) {
		regulator_disable(of_dp_pad);
	}
	return 0;
}

static int dc_dp_out_hotplug_init(struct device *dev)
{
	int err = 0;
	int gpio;
	struct tegra_dc *dc;

	dc = tegra_get_dc_from_dev(dev);
	if (!dc) {
		err = -ENODEV;
		goto fail;
	}

	/* hotplug pin should be in spio mode */
	gpio = dc->pdata->default_out->hotplug_gpio;
	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, "temp_request");
		if (!err)
			gpio_free(gpio);
	}

	/*
	 * DP doesn't actually need this regulator.
	 * Instead dp needs gpio coupled with this regulator.
	 * pmic has already requested this gpio
	 * Required for level translator logic.
	 */
	if (!of_dp_hdmi_5v0) {
		of_dp_hdmi_5v0 = devm_regulator_get(dev, "vdd_hdmi_5v0");
		if (IS_ERR(of_dp_hdmi_5v0)) {
			err = PTR_ERR(of_dp_hdmi_5v0);
			dev_info(dev, "%s: couldn't get regulator %s\n",
				__func__, "vdd_hdmi_5v0");
			of_dp_hdmi_5v0 = NULL;
		}
	}
	if (of_dp_hdmi_5v0) {
		err = regulator_enable(of_dp_hdmi_5v0);
		if (err < 0)
			dev_err(dev, "%s: couldn't enable regulator %s\n",
				__func__, "vdd_hdmi_5v0");
	}

fail:
	return err;
}

static int dc_dp_out_postsuspend(void)
{
	if (of_dp_hdmi_5v0) {
		regulator_disable(of_dp_hdmi_5v0);
	}

	return 0;
}

static int dc_hdmi_out_enable(struct device *dev)
{
	int err = 0;

	if (!of_hdmi_dp_reg) {
		of_hdmi_dp_reg = devm_regulator_get(dev, "avdd_hdmi");
		if (IS_ERR(of_hdmi_dp_reg)) {
			dev_err(dev, "%s: couldn't get regulator %s\n",
					__func__, "avdd_hdmi");
			of_hdmi_dp_reg = NULL;
			err = PTR_ERR(of_hdmi_dp_reg);
			goto dc_hdmi_out_en_fail;
		}
	}
	err = regulator_enable(of_hdmi_dp_reg);
	if (err < 0) {
		dev_err(dev, "%s: couldn't enable regulator %s\n",
				__func__, "avdd_hdmi");
		goto dc_hdmi_out_en_fail;
	}
	if (!of_hdmi_pll) {
		of_hdmi_pll = devm_regulator_get(dev, "avdd_hdmi_pll");
		if (IS_ERR(of_hdmi_pll)) {
			dev_err(dev, "%s: couldn't get regulator %s\n",
					__func__, "avdd_hdmi_pll");
			of_hdmi_pll = NULL;
			of_hdmi_dp_reg = NULL;
			err = PTR_ERR(of_hdmi_pll);
			goto dc_hdmi_out_en_fail;
		}
	}
	err = regulator_enable(of_hdmi_pll);
	if (err < 0) {
		dev_err(dev, "%s: couldn't enable regulator %s\n",
				__func__, "avdd_hdmi_pll");
		goto dc_hdmi_out_en_fail;
	}
dc_hdmi_out_en_fail:
	return err;
}

static int dc_hdmi_out_disable(struct device *dev)
{
	struct platform_device *ndev = NULL;
	struct tegra_hdmi *hdmi = NULL;
	struct tegra_dc *dc = NULL;

	if (!dev)
		return -EINVAL;
	ndev = to_platform_device(dev);

	dc = platform_get_drvdata(ndev);
	hdmi = tegra_dc_get_outdata(dc);

	/* Do not disable regulator when device is shutting down */
	if (hdmi->device_shutdown)
		return 0;

	if (of_hdmi_dp_reg) {
		regulator_disable(of_hdmi_dp_reg);
	}

	if (of_hdmi_pll) {
		regulator_disable(of_hdmi_pll);
	}

	return 0;
}

static int dc_hdmi_hotplug_init(struct device *dev)
{
	int err = 0;

	if (!of_hdmi_vddio) {
		of_hdmi_vddio = devm_regulator_get(dev, "vdd_hdmi_5v0");
		if (IS_ERR(of_hdmi_vddio)) {
			err = PTR_ERR(of_hdmi_vddio);
			dev_err(dev, "%s: couldn't get regulator %s, %d\n",
					__func__, "vdd_hdmi_5v0", err);
			of_hdmi_vddio = NULL;
			goto dc_hdmi_hotplug_init_fail;
		}
	}
	err = regulator_enable(of_hdmi_vddio);
	if (err < 0) {
		dev_err(dev, "%s: couldn't enable regulator %s, %d\n",
				__func__, "vdd_hdmi_5v0", err);
		goto dc_hdmi_hotplug_init_fail;
	}
dc_hdmi_hotplug_init_fail:
	return err;
}

static int dc_hdmi_postsuspend(void)
{
	if (of_hdmi_vddio) {
		regulator_disable(of_hdmi_vddio);
	}
	return 0;
}

static bool is_dc_default_flag(u32 flag)
{
	if (flag == (flag &
		(TEGRA_DC_FLAG_ENABLED |
		TEGRA_DC_FLAG_SET_EARLY_MODE |
		TEGRA_DC_FLAG_FBCON_DISABLED)
	))
		return true;
	else
		return false;
}

static int parse_imp_win_values(struct device_node *settings_np,
			struct platform_device *pdev,
			struct tegra_nvdisp_imp_settings *imp_settings,
			int num_entries)
{
	u8 win_ids_arr[DC_N_WINDOWS];
	s8 thread_group_arr[DC_N_WINDOWS];
	u16 fetch_slots_arr[DC_N_WINDOWS];
	u32 pipe_meter_arr[DC_N_WINDOWS];
	u64 dvfs_watermark_arr[DC_N_WINDOWS];
	u64 mempool_entries_arr[DC_N_WINDOWS];
	int i, ret = 0;

	if (WARN_ON(DC_N_WINDOWS < num_entries)) {
		dev_err(&pdev->dev, "invalid num_entries\n");
		return -EINVAL;
	}

	ret = of_property_read_u8_array(settings_np,
		"nvidia,imp_win_mapping",
		win_ids_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse nvidia,imp_win_mapping\n");
		return ret;
	}

	ret = of_property_read_u16_array(settings_np,
		"nvidia,win_fetch_meter_slots",
		fetch_slots_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,win_fetch_meter_slots\n");
		return ret;
	}

	ret = of_property_read_u64_array(settings_np,
		"nvidia,win_dvfs_watermark_values",
		dvfs_watermark_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,win_dvfs_watermark_values\n");
		return ret;
	}

	ret = of_property_read_u32_array(settings_np,
		"nvidia,win_pipe_meter_values",
		pipe_meter_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,win_pipe_meter_values\n");
		return ret;
	}

	ret = of_property_read_u64_array(settings_np,
		"nvidia,win_mempool_buffer_entries",
		mempool_entries_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,win_mempool_buffer_entries\n");
		return ret;
	}

	ret = of_property_read_u8_array(settings_np,
		"nvidia,win_thread_groups",
		thread_group_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse nvidia,win_thread_groups\n");
		return ret;
	}

	for (i = 0; i < imp_settings->num_heads; i++) {
		struct tegra_nvdisp_imp_head_settings *head_settings;
		struct tegra_dc_ext_nvdisp_imp_win_entries *win_entries;
		int j;

		head_settings = &imp_settings->head_settings[i];
		win_entries = head_settings->win_entries;

		for (j = 0; j < num_entries; j++) {
			struct tegra_dc_ext_nvdisp_imp_win_entries *win_entry;

			win_entry = &win_entries[j];
			win_entry->id = win_ids_arr[j];
			win_entry->thread_group = thread_group_arr[j];
			win_entry->fetch_slots = fetch_slots_arr[j];
			win_entry->pipe_meter = pipe_meter_arr[j];
			win_entry->dvfs_watermark = dvfs_watermark_arr[j];
			win_entry->mempool_entries = mempool_entries_arr[j];
		}
	}

	return ret;
}

static int parse_imp_cursor_values(struct device_node *settings_np,
			struct platform_device *pdev,
			struct tegra_nvdisp_imp_settings *imp_settings,
			int num_entries)
{
	u32 max_heads = tegra_dc_get_numof_dispheads();
	u8 ctrl_num_arr[max_heads];
	u16 fetch_slots_arr[max_heads];
	u32 pipe_meter_arr[max_heads];
	u64 dvfs_watermark_arr[max_heads];
	u64 mempool_entries_arr[max_heads];
	int ret = 0, i;

	ret = of_property_read_u8_array(settings_np,
		"nvidia,imp_head_mapping",
		ctrl_num_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse nvidia,imp_head_mapping\n");
		return ret;
	}

	ret = of_property_read_u16_array(settings_np,
		"nvidia,cursor_fetch_meter_slots",
		fetch_slots_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,cursor_fetch_meter_slots\n");
		return ret;
	}

	ret = of_property_read_u64_array(settings_np,
		"nvidia,cursor_dvfs_watermark_values",
		dvfs_watermark_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,cursor_dvfs_watermark_values\n");
		return ret;
	}

	ret = of_property_read_u32_array(settings_np,
		"nvidia,cursor_pipe_meter_values",
		pipe_meter_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,cursor_pipe_meter_values\n");
		return ret;
	}

	ret = of_property_read_u64_array(settings_np,
		"nvidia,cursor_mempool_buffer_entries",
		mempool_entries_arr, num_entries);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't parse nvidia,cursor_mempool_buffer_entries\n");
		return ret;
	}

	for (i = 0; i < num_entries; i++) {
		struct tegra_nvdisp_imp_head_settings *head_settings;
		struct tegra_dc_ext_nvdisp_imp_head_entries *head_entries;

		head_settings = &imp_settings->head_settings[i];
		head_entries = &head_settings->entries;

		head_entries->ctrl_num = ctrl_num_arr[i];
		head_entries->curs_fetch_slots = fetch_slots_arr[i];
		head_entries->curs_pipe_meter = pipe_meter_arr[i];
		head_entries->curs_dvfs_watermark = dvfs_watermark_arr[i];
		head_entries->curs_mempool_entries = mempool_entries_arr[i];
	}

	return ret;
}

static void dealloc_imp_table_settings(
				struct tegra_nvdisp_imp_settings *imp_settings)
{
	int i;

	for (i = 0; i < imp_settings->num_heads; i++) {
		struct tegra_nvdisp_imp_head_settings *head_settings;

		head_settings = &imp_settings->head_settings[i];
		if (head_settings->num_wins > 0)
			kfree(head_settings->win_entries);
	}

	if (imp_settings->num_heads > 0)
		kfree(imp_settings->head_settings);
}

static void dealloc_imp_table(struct nvdisp_imp_table *imp_table)
{
	int i;

	if (!imp_table)
		return;

	for (i = 0; i < imp_table->num_settings; i++)
		dealloc_imp_table_settings(&imp_table->settings[i]);
}

static int alloc_imp_table_settings(struct platform_device *pdev,
				struct tegra_nvdisp_imp_settings *imp_settings,
				int num_head_entries, int num_win_entries)
{
	int j;

	if (num_head_entries <= 0 || num_win_entries <= 0) {
		dev_err(&pdev->dev,
			"No IMP table heads and/or wins found\n");
		return -EINVAL;
	}

	imp_settings->head_settings = devm_kzalloc(&pdev->dev,
		sizeof(*(imp_settings->head_settings)) * num_head_entries,
		GFP_KERNEL);
	if (!imp_settings->head_settings)
		return -ENOMEM;
	imp_settings->num_heads = num_head_entries;

	for (j = 0; j < num_head_entries; j++) {
		struct tegra_nvdisp_imp_head_settings *head_settings;

		head_settings = &imp_settings->head_settings[j];
		head_settings->win_entries = devm_kzalloc(&pdev->dev,
			sizeof(*(head_settings->win_entries)) * num_win_entries,
			GFP_KERNEL);
		if (!head_settings->win_entries)
			return -ENOMEM;
		head_settings->num_wins = num_win_entries;
	}

	return 0;
}

static int tegra_dc_parse_imp_data(struct device_node *imp_np,
				struct platform_device *pdev,
				struct nvdisp_imp_table *imp_table)
{
	struct device_node *settings_np = NULL;
	struct tegra_nvdisp_imp_settings *imp_settings;
	int ret = 0, i = 0;
	u32 num_settings = 0;

	ret = of_property_read_u32(imp_np, "num_settings", &num_settings);
	if (ret || !num_settings) {
		dev_err(&pdev->dev, "No IMP table settings found\n");
		return -ENOENT;
	}

	imp_table->settings = devm_kzalloc(&pdev->dev,
		sizeof(*(imp_table->settings)) * num_settings,
		GFP_KERNEL);
	if (!imp_table->settings)
		return -ENOMEM;
	imp_table->num_settings = num_settings;

	for (i = 0; i < num_settings; i++) {
		struct tegra_dc_ext_nvdisp_imp_global_entries *global_entries;
		char settings_name[CHAR_BUF_SIZE_MAX] = {0};
		int num_head_entries, num_win_entries;

		snprintf(settings_name, sizeof(settings_name),
						"disp_imp_settings_%d", i);
		settings_np = of_get_child_by_name(imp_np, settings_name);
		if (!settings_np) {
			dev_err(&pdev->dev, "Can't find %s\n", settings_name);
			ret = -ENOENT;

			goto fail_parse_imp_table;
		}

		num_head_entries = of_property_count_u8_elems(settings_np,
						"nvidia,imp_head_mapping");
		num_win_entries = of_property_count_u8_elems(settings_np,
						"nvidia,imp_win_mapping");

		imp_settings = &imp_table->settings[i];
		ret = alloc_imp_table_settings(pdev, imp_settings,
					num_head_entries, num_win_entries);
		if (ret)
			goto fail_parse_imp_table;

		global_entries = &imp_settings->global_entries;
		ret = of_property_read_u64(settings_np,
			"nvidia,total_disp_bw_with_catchup",
			&global_entries->total_iso_bw_with_catchup_kBps);
		if (ret) {
			dev_err(&pdev->dev,
			"Can't parse nvidia,total_disp_bw_with_catchup\n");
			goto fail_parse_imp_table;
		}

		ret = of_property_read_u64(settings_np,
			"nvidia,total_disp_bw_without_catchup",
			&global_entries->total_iso_bw_without_catchup_kBps);
		if (ret) {
			dev_err(&pdev->dev,
			"Can't parse nvidia,total_disp_bw_without_catchup\n");
			goto fail_parse_imp_table;
		}

		ret = of_property_read_u64(settings_np,
			"nvidia,disp_emc_floor",
			&global_entries->emc_floor_hz);
		if (ret) {
			dev_err(&pdev->dev,
				"Can't parse nvidia,disp_emc_floor\n");
			goto fail_parse_imp_table;
		}

		ret = of_property_read_u64(settings_np,
			"nvidia,disp_min_hubclk",
			&global_entries->min_hubclk_hz);
		if (ret) {
			dev_err(&pdev->dev,
				"Can't parse nvidia,disp_min_hubclk\n");
			goto fail_parse_imp_table;
		}

		ret = of_property_read_u16(settings_np,
			"nvidia,total_win_fetch_slots",
			&global_entries->total_win_fetch_slots);
		if (ret) {
			dev_err(&pdev->dev,
				"Can't parse nvidia,total_win_fetch_slots\n");
			goto fail_parse_imp_table;
		}

		ret = of_property_read_u16(settings_np,
			"nvidia,total_cursor_fetch_slots",
			&global_entries->total_curs_fetch_slots);
		if (ret) {
			dev_err(&pdev->dev,
			"Can't parse nvidia,total_cursor_fetch_slots\n");
			goto fail_parse_imp_table;
		}

		ret = parse_imp_cursor_values(settings_np, pdev, imp_settings,
							num_head_entries);
		if (ret)
			goto fail_parse_imp_table;

		ret = parse_imp_win_values(settings_np, pdev, imp_settings,
							num_win_entries);
		if (ret)
			goto fail_parse_imp_table;

		of_node_put(settings_np);
	}

	return ret;

fail_parse_imp_table:
	of_node_put(settings_np);
	dealloc_imp_table(imp_table);
	kfree(imp_table->settings);

	return ret;
}

int of_tegra_get_fb_resource(struct device_node *np,
	 struct resource *res, const char *reg_name)
{
	int index;
	unsigned int flags;
	u64 size;
	u64 *addrp;

	struct device_node *fb_node = of_parse_phandle(np, "fb_reserved", 0);

	if (fb_node == NULL)
		goto fail;

	index = of_property_match_string(fb_node, "reg-names", reg_name);
	if (index < 0)
		goto fail;

	addrp = (u64 *)of_get_address(fb_node, index, &size, &flags);
	if (addrp == NULL)
		goto fail;

	res->start = *addrp;
	if (res->start)
		res->end = res->start + size - 1;
	else
		res->end = 0;

	return 0;

fail:
	res->start = 0;
	res->end = 0;
	return -EINVAL;
}

static int tegra_dc_register_typec_edev(struct platform_device *ndev,
					struct device_node *sor_np,
					struct extcon_dev **typec_edev_ptr)
{
	struct extcon_dev *typec_edev;
	char port_name[CHAR_BUF_SIZE_MAX] = {0};
	u8 typec_port;

	if (of_property_read_u8(sor_np, "nvidia,typec-port", &typec_port))
		return 0;

	snprintf(port_name, sizeof(port_name), "typec%u", typec_port);
	typec_edev = extcon_get_extcon_dev_by_cable(&ndev->dev, port_name);
	if (!typec_edev) {
		dev_err(&ndev->dev, "typec extcon dev is NULL\n");
		return -ENODEV;
	} else if (IS_ERR(typec_edev)) {
		if (PTR_ERR(typec_edev) != -EPROBE_DEFER)
			dev_err(&ndev->dev,
				"Couldn't find typec extcon dev: %ld\n",
				PTR_ERR(typec_edev));

		return PTR_ERR(typec_edev);
	}

	*typec_edev_ptr = typec_edev;
	return 0;
}

struct tegra_dc_platform_data *of_dc_parse_platform_data(
	struct platform_device *ndev, struct tegra_dc_platform_data *boot_pdata)
{
	u32 temp;
	int err = 0;
#if defined(CONFIG_TRUSTED_LITTLE_KERNEL) || defined(CONFIG_TRUSTY)
	int check_val;
#endif
	const __be32 *p;
	struct tegra_dc_platform_data *pdata;
	struct device_node *np = ndev->dev.of_node;
	struct device_node *timings_np = NULL;
	struct device_node *vrr_np = NULL;
	struct device_node *np_target_disp = NULL;
	struct device_node *entry = NULL;
	struct property *prop;
	struct tegra_dc_out *def_out;
	bool pdata_initialized = false;
	bool restore_topology = false;
	struct tegra_dc *dc = NULL;
	char dc_or_conn_node[CHAR_BUF_SIZE_MAX];
	struct device_node *cmu_np = NULL;
	struct device_node *cmu_adbRGB_np = NULL;
	struct extcon_dev *typec_edev = NULL;

	/*
	 * Memory for pdata, pdata->default_out, pdata->fb
	 * need to be allocated in default
	 * since it is expected data for these needs to be
	 * parsed from DTB.
	 */
	if (boot_pdata) {
		pdata_initialized = true;
		pdata = boot_pdata;

		dc = tegra_get_dc_from_dev(&ndev->dev);

		if (dc && dc->current_topology.conn_inst ==
			TEGRA_DC_TOPOLOGY_RESTORE)
			restore_topology = true;
	} else {
		pdata = devm_kzalloc(&ndev->dev,
			sizeof(struct tegra_dc_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&ndev->dev, "not enough memory\n");
			err = -ENOMEM;
			goto fail_parse;
		}

		pdata->default_out = devm_kzalloc(&ndev->dev,
			sizeof(struct tegra_dc_out), GFP_KERNEL);
		if (!pdata->default_out) {
			dev_err(&ndev->dev, "not enough memory\n");
			err = -ENOMEM;
			goto fail_parse;
		}

		pdata->fb = devm_kzalloc(&ndev->dev,
			sizeof(struct tegra_fb_data), GFP_KERNEL);
		if (!pdata->fb) {
			dev_err(&ndev->dev, "not enough memory\n");
			err = -ENOMEM;
			goto fail_parse;
		}
	}
	def_out = pdata->default_out;

	if (!of_property_read_u32(np, "nvidia,frame_lock_enable", &temp)) {
		pdata->frame_lock_enable = (bool)temp;
		OF_DC_LOG("nvidia,frame_lock_enable%d\n", pdata->frame_lock_enable);
	}

	/* Check for connector during crossbar */
	if (pdata_initialized && !restore_topology) {
		switch (def_out->type) {
		case TEGRA_DC_OUT_FAKE_DP:
		case TEGRA_DC_OUT_HDMI:
		case TEGRA_DC_OUT_DP:
			if (dc->current_topology.conn_inst == 0) {
				pdata->conn_np =
					of_find_node_by_path("/host1x/sor");
			} else {
				snprintf(dc_or_conn_node, 13, "/host1x/sor%d",
					dc->current_topology.conn_inst);
				pdata->conn_np =
					of_find_node_by_path(dc_or_conn_node);
			}
		break;
		case TEGRA_DC_OUT_DSI:
			pdata->conn_np = of_find_node_by_path("/host1x/dsi");
			break;
		break;
		}
	} else {
		pdata->conn_np = of_parse_phandle(np, "nvidia,dc-connector", 0);
	}

	if (IS_ERR_OR_NULL(pdata->conn_np)) {
		dev_err(&ndev->dev, "mandatory prop:%s not defined\n",
			"nvidia,dc-connector");
		err = PTR_ERR(pdata->conn_np);
		goto fail_parse;
	}

	err = tegra_dc_register_typec_edev(ndev, pdata->conn_np, &typec_edev);
	if (err)
		goto fail_parse;

	if (!of_property_read_u32(np, "nvidia,dc-ctrlnum", &temp)) {
		pdata->ctrl_num = (unsigned long)temp;
		OF_DC_LOG("dc controller index %lu\n", pdata->ctrl_num);
	} else {
		dev_err(&ndev->dev, "mandatory property %s not found\n",
				"nvidia,dc-ctrlnum");
		goto fail_parse;
	}

	dev_info(&ndev->dev, "disp%d connected to head%d->%s\n", ndev->id,
		(int)pdata->ctrl_num, of_node_full_name(pdata->conn_np));

	/* Check for panel node during crossbar */
	if (pdata_initialized && !restore_topology) {
		pdata->panel_np = NULL;
		if (def_out->type == TEGRA_DC_OUT_HDMI)
			pdata->panel_np = of_get_child_by_name(pdata->conn_np,
				"hdmi-display");
		else if ((def_out->type == TEGRA_DC_OUT_DP) ||
			(def_out->type == TEGRA_DC_OUT_FAKE_DP))
			pdata->panel_np = of_get_child_by_name(pdata->conn_np,
				"dp-display");

		if (!pdata->panel_np) {
			pr_warn("crossbar: no panel found for out_type %d\n",
				def_out->type);
			err = -ENODEV;
		}
	} else {
		err = tegra_dc_parse_panel_ops(ndev, pdata);
		if (err) {
			dev_err(&ndev->dev, "err:%d parsing panel_ops\n", err);
			goto fail_parse;
		}
	}

	err = tegra_dc_parse_out_type(ndev, pdata, pdata_initialized);
	if (err) {
		dev_err(&ndev->dev, "err:%d parsing out_type\n", err);
		goto fail_parse;
	}

	if (!of_property_read_u32(np, "nvidia,fb-bpp", &temp)) {
		pdata->fb->bits_per_pixel = (int)temp;
		OF_DC_LOG("fb bpp %d\n", pdata->fb->bits_per_pixel);
	} else {
		dev_err(&ndev->dev, "mandatory prop:%s not defined\n",
			"nvidia,fb-bpp");
		err = -ENOENT;
		goto fail_parse;
	}

	if (!of_property_read_u32(np, "nvidia,fbmem-size", &temp)) {
		pdata->fb->fbmem_size = temp;
		OF_DC_LOG("fbmem size %u\n", pdata->fb->fbmem_size);
	}

	if (!of_property_read_u32(np, "nvidia,fb-flags", &temp)) {
		if (temp == TEGRA_FB_FLIP_ON_PROBE) {
			OF_DC_LOG("fb flip on probe\n");
		} else if (temp == 0) {
			OF_DC_LOG("do not flip fb on probe time\n");
		} else {
			dev_err(&ndev->dev, "invalid fb-flags:0x%x\n", temp);
			err = -EINVAL;
			goto fail_parse;
		}
		pdata->fb->flags = (unsigned long)temp;
	}

	if (of_property_read_bool(np, "nvidia,50hz-ss-war")) {
		pdata->plld2_ss_enable = true;
		OF_DC_LOG("dc plld2 ss enabled - %d\n", pdata->plld2_ss_enable);
	} else {
		pdata->plld2_ss_enable = false;
	}

	if (def_out->type == TEGRA_DC_OUT_DSI) {
		/* TODO: Add crossbar implementation for DSI */
		def_out->dsi = devm_kzalloc(&ndev->dev,
				sizeof(struct tegra_dsi_out), GFP_KERNEL);
		if (!def_out->dsi) {
			err = -ENOMEM;
			goto fail_parse;
		}

		err = parse_dsi_settings(ndev, pdata, def_out);
		if (err) {
			dev_err(&ndev->dev, "dsi parsing failed:%d\n", err);
			goto fail_parse;
		}

		np_target_disp = pdata->panel_np;
	} else if (def_out->type == TEGRA_DC_OUT_DP ||
		   def_out->type == TEGRA_DC_OUT_FAKE_DP) {

		def_out->dp_out = devm_kzalloc(&ndev->dev,
				sizeof(struct tegra_dp_out), GFP_KERNEL);
		if (!def_out->dp_out) {
			err = -ENOMEM;
			goto fail_parse;
		}

		err = parse_dp_settings(ndev, pdata, def_out);
		if (err) {
			dev_err(&ndev->dev, "dp parsing failed:%d\n", err);
			goto fail_parse;
		}

		np_target_disp = pdata->panel_np;

		if (!of_property_read_u32(np_target_disp,
				"nvidia,hdmi-fpd-bridge", &temp)) {
			def_out->dp_out->hdmi2fpd_bridge_enable = (bool)temp;
			OF_DC_LOG("hdmi2fpd_bridge_enabled %d\n",
					def_out->hdmi_out->
					hdmi2fpd_bridge_enable);
		}
		if (!of_property_read_u32(np_target_disp,
				"nvidia,edp-lvds-bridge", &temp)) {
			def_out->dp_out->edp2lvds_bridge_enable = (bool)temp;
			OF_DC_LOG("edp2lvds_bridge_enabled %d\n",
					def_out->dp_out->
					edp2lvds_bridge_enable);
		}
		if (!of_property_read_u32(np_target_disp,
				"nvidia,edp-lvds-i2c-bus-no", &temp)) {
			def_out->dp_out->edp2lvds_i2c_bus_no = temp;
			OF_DC_LOG("edp2lvds_i2c_bus_no %d\n",
					def_out->dp_out->
					edp2lvds_i2c_bus_no);
		}

		def_out->dp_out->typec_ecable.edev = typec_edev;

		/* enable/disable ops for DP monitors */
		if (!def_out->enable && !def_out->disable) {
			def_out->enable		= dc_dp_out_enable;
			def_out->disable	= dc_dp_out_disable;
			def_out->hotplug_init	= dc_dp_out_hotplug_init;
			def_out->postsuspend	= dc_dp_out_postsuspend;
		}
	} else if (def_out->type == TEGRA_DC_OUT_HDMI) {

		def_out->hdmi_out = devm_kzalloc(&ndev->dev,
				sizeof(struct tegra_hdmi_out), GFP_KERNEL);
		if (!def_out->hdmi_out) {
			err = -ENOMEM;
			goto fail_parse;
		}
		np_target_disp = pdata->panel_np;
		if (!of_property_read_u32(np_target_disp,
					"nvidia,hdmi-fpd-bridge", &temp)) {
			def_out->hdmi_out->hdmi2fpd_bridge_enable = (bool)temp;
			OF_DC_LOG("hdmi2fpd_bridge_enabled %d\n",
				def_out->hdmi_out->hdmi2fpd_bridge_enable);
		}
		if (!of_property_read_u32(np_target_disp,
					"nvidia,hdmi-gmsl-bridge", &temp)) {
			def_out->hdmi_out->hdmi2gmsl_bridge_enable = (bool)temp;
			OF_DC_LOG("hdmi2gmsl_bridge_enabled %d\n",
				def_out->hdmi_out->hdmi2gmsl_bridge_enable);
		}
		if (!of_property_read_u32(np_target_disp,
				"nvidia,hdmi-dsi-bridge", &temp)) {
			def_out->hdmi_out->
				hdmi2dsi_bridge_enable = (bool)temp;
			OF_DC_LOG("hdmi2dsi_bridge_enabled %d\n",
				def_out->hdmi_out->
					hdmi2dsi_bridge_enable);
		}
		if (!of_property_read_u32(np_target_disp,
					"generic-infoframe-type", &temp))
			def_out->hdmi_out->
				generic_infoframe_type = (u8)temp;
		else
			def_out->hdmi_out->
				generic_infoframe_type =
				HDMI_INFOFRAME_TYPE_HDR;
		pr_info("generic_infoframe_type: 0x%02x\n",
				def_out->hdmi_out->
					generic_infoframe_type);
		if (def_out->hdmi_out->generic_infoframe_type ==
			HDMI_INFOFRAME_TYPE_SPD)
			parse_spd_infoframe(ndev, np_target_disp,
				def_out);
		/* fixed panel ops is dominant. If fixed panel ops
		 * is not defined, we set default hdmi panel ops */
		if (!def_out->enable && !def_out->disable) {
			def_out->enable		= dc_hdmi_out_enable;
			def_out->disable	= dc_hdmi_out_disable;
			def_out->hotplug_init	= dc_hdmi_hotplug_init;
			def_out->postsuspend	= dc_hdmi_postsuspend;
		}
	} else if (def_out->type == TEGRA_DC_OUT_LVDS) {
		np_target_disp = pdata->panel_np;
	} else {
		dev_err(&ndev->dev, "failed to identify out type %d\n",
			def_out->type);
		err = -ENOENT;
		goto fail_parse;
	}

	if (!np_target_disp) {
		dev_err(&ndev->dev, "failed to identify target panel\n");
		err = -ENODEV;
		goto fail_parse;
	}

	of_property_for_each_u32(np, "nvidia,dc-flags", prop, p, temp) {
		if (!is_dc_default_flag(temp)) {
			pr_err("invalid dc-flags\n");
			goto fail_parse;
		}
		pdata->flags |= (unsigned long)temp;
	}
	OF_DC_LOG("dc flag %lu\n", pdata->flags);

	err = parse_disp_default_out(ndev, pdata);
	if (err) {
		dev_err(&ndev->dev, "failed to parse disp_default_out,%d\n",
				err);
		goto fail_parse;
	}

	if (pdata->default_out->type == TEGRA_DC_OUT_DSI) {
		if (!of_property_read_u32(np_target_disp,
			"nvidia,dsi-hdmi-bridge", &temp)) {
			pdata->default_out->is_ext_panel = (int)temp;
			OF_DC_LOG("is_ext_dp_panel %d\n", temp);
		}
	}

	if ((pdata->default_out->type == TEGRA_DC_OUT_DP) ||
		(pdata->default_out->type == TEGRA_DC_OUT_FAKE_DP)) {
		if (!of_property_read_u32(np_target_disp,
			"nvidia,is_ext_dp_panel", &temp)) {
			pdata->default_out->is_ext_panel = (int)temp;
			OF_DC_LOG("is_ext_dp_panel %d\n", temp);
		}
	}

	timings_np = of_get_child_by_name(np_target_disp, "display-timings");
	if (!timings_np) {
		if (def_out->type == TEGRA_DC_OUT_DSI &&
			!pdata->default_out->is_ext_panel) {
			pr_err("%s: could not find display-timings node\n",
				__func__);
			goto fail_parse;
		}
	} else if (def_out->type == TEGRA_DC_OUT_DSI ||
		   def_out->type == TEGRA_DC_OUT_FAKE_DP ||
		   def_out->type == TEGRA_DC_OUT_DP ||
		   def_out->type == TEGRA_DC_OUT_LVDS) {
		def_out->n_modes =
			of_get_child_count(timings_np);
		if (def_out->n_modes == 0) {
			/*
			 * Should never happen !
			 */
			dev_err(&ndev->dev, "no timing given\n");
			goto fail_parse;
		}
		def_out->modes = devm_kzalloc(&ndev->dev,
			def_out->n_modes *
			sizeof(struct tegra_dc_mode), GFP_KERNEL);
		if (!def_out->modes) {
			dev_err(&ndev->dev, "not enough memory\n");
			goto fail_parse;
		}
	} else if (def_out->type == TEGRA_DC_OUT_HDMI) {
		def_out->n_modes =
			of_get_child_count(timings_np);
		if (def_out->n_modes) {
			def_out->modes = devm_kzalloc(&ndev->dev,
				def_out->n_modes *
				sizeof(struct tegra_dc_mode), GFP_KERNEL);
			if (!def_out->modes) {
				dev_err(&ndev->dev, "not enough memory\n");
				goto fail_parse;
			}
		} else {
			if (tegra_fb_is_console_enabled(pdata)) {
				/*
				 * Should never happen !
				 */
				dev_err(&ndev->dev, "no timing provided\n");
				goto fail_parse;
			}
		}
	}

#if defined(CONFIG_FRAMEBUFFER_CONSOLE)
	/* Default mode for fbconsole */
	if ((pdata->default_out->type == TEGRA_DC_OUT_HDMI) ||
		(pdata->default_out->type == TEGRA_DC_OUT_DP)) {
		err = parse_fbcon_default_mode(np_target_disp, ndev, pdata);
		if (err) {
			pr_err("%s: parsing fbcon mode failed\n", __func__);
			goto fail_parse;
		}
	}
#endif

	vrr_np = of_get_child_by_name(np_target_disp, "vrr-settings");
	if (!vrr_np || (def_out->n_modes < 2)) {
		pr_debug("%s: could not find vrr-settings node\n", __func__);
	} else if (def_out->type != TEGRA_DC_OUT_DSI) {
		dev_err(&ndev->dev,
			"vrr-settings specified for a non-DSI head, disregarded\n");
	} else {
		dma_addr_t dma_addr;
		struct tegra_vrr *vrr;

		def_out->vrr = dma_alloc_coherent(NULL, PAGE_SIZE,
						&dma_addr, GFP_KERNEL);
		vrr = def_out->vrr;
		if (vrr) {
#if defined(CONFIG_TRUSTED_LITTLE_KERNEL) || defined(CONFIG_TRUSTY)
				int retval;

				retval = tegra_hdmivrr_te_set_buf(vrr);
				if (retval) {
					dev_err(&ndev->dev, "failed to set buffer\n");
					goto fail_parse;
				}
#endif
		} else {
			dev_err(&ndev->dev, "not enough memory\n");
			goto fail_parse;
		}

		err = parse_vrr_settings(ndev, vrr_np, vrr);
		if (err)
			goto fail_parse;
	}

	if (!of_property_read_u32(np_target_disp,
				"nvidia,hdmi-vrr-caps", &temp)) {
		if (def_out->type != TEGRA_DC_OUT_HDMI) {
			dev_err(&ndev->dev,
				"nvidia,hdmi-vrr-caps specified for a non-HDMI head, disregarded\n");
		} else {
			def_out->vrr = devm_kzalloc(&ndev->dev,
					sizeof(struct tegra_vrr), GFP_KERNEL);
			if (!def_out->vrr) {
				dev_err(&ndev->dev, "not enough memory\n");
				goto fail_parse;
			}
			OF_DC_LOG("nvidia,hdmi-vrr-caps: %d\n", temp);
#if defined(CONFIG_TRUSTED_LITTLE_KERNEL) || defined(CONFIG_TRUSTY)
				check_val = tegra_hdmivrr_te_set_buf(
						def_out->vrr);
				if (check_val) {
					dev_err(&ndev->dev,
						"failed to set buffer\n");
					goto fail_parse;
				}
#endif
		}
	} else
		pr_debug("%s: nvidia,hdmi-vrr-caps not present\n", __func__);

	pdata->default_out->vrr_hotplug_state = TEGRA_HPD_STATE_NORMAL;

	if (tegra_dc_is_t21x()) {
		cmu_np = of_get_child_by_name(np_target_disp, "cmu");
		if (!cmu_np) {
			pr_debug("%s: could not find cmu node\n", __func__);
		} else {
			pdata->cmu = devm_kzalloc(&ndev->dev,
				sizeof(struct tegra_dc_cmu), GFP_KERNEL);
			if (!pdata->cmu)
				goto fail_parse;
		}

		cmu_adbRGB_np = of_get_child_by_name(np_target_disp,
				"cmu_adobe_rgb");

		if (!cmu_adbRGB_np) {
			pr_debug("%s: could not find cmu node for adobeRGB\n",
					__func__);
		} else {
			pdata->cmu_adbRGB = devm_kzalloc(&ndev->dev,
				sizeof(struct tegra_dc_cmu), GFP_KERNEL);
			if (!pdata->cmu_adbRGB)
				goto fail_parse;
		}
		if (pdata->cmu != NULL) {
			err = parse_cmu_data(cmu_np, pdata->cmu);
			if (err)
				goto fail_parse;
		}

		if (pdata->cmu_adbRGB != NULL) {
			err = parse_cmu_data(cmu_adbRGB_np, pdata->cmu_adbRGB);
			if (err)
				goto fail_parse;
		}
	}
	if (tegra_dc_is_nvdisplay()) {
		cmu_np = of_get_child_by_name(np_target_disp, "nvdisp-cmu");
		if (!cmu_np) {
			pr_debug("%s: could not find cmu node\n", __func__);
		} else {
			pdata->nvdisp_cmu = devm_kzalloc(&ndev->dev,
				sizeof(struct tegra_dc_nvdisp_cmu), GFP_KERNEL);
			if (!pdata->nvdisp_cmu)
				goto fail_parse;
		}

		if (pdata->nvdisp_cmu != NULL) {
			err = parse_nvdisp_cmu_data(cmu_np, pdata->nvdisp_cmu);
			if (err)
				goto fail_parse;
		}
	}

	if (def_out->modes != NULL) {
		struct tegra_dc_mode *cur_mode
			= def_out->modes;
		for_each_child_of_node(timings_np, entry) {
			err = parse_modes(def_out, entry, cur_mode);
			if (err)
				goto fail_parse;
			cur_mode++;
		}
	}

	if (of_property_read_u32(pdata->panel_np, "nvidia,default_color_space",
					&pdata->default_clr_space))
		pdata->default_clr_space = 0;

	if (!pdata_initialized) {
		if (!of_property_read_u32(np, "nvidia,fb-win", &temp)) {
			pdata->fb->win = (int)temp;
			OF_DC_LOG("fb window Index %d\n", pdata->fb->win);
		}

		if (!of_property_read_u32(np, "win-mask", &temp)) {
			pdata->win_mask = (u32)temp;
			OF_DC_LOG("win mask 0x%x\n", temp);
		}
	}

	if (!of_property_read_u32(np, "nvidia,emc-clk-rate", &temp)) {
		pdata->emc_clk_rate = (unsigned long)temp;
		OF_DC_LOG("emc clk rate %lu\n", pdata->emc_clk_rate);
	}

	if (!of_property_read_u32(np, "nvidia,cmu-enable", &temp)) {
		pdata->cmu_enable = (bool)temp;
		OF_DC_LOG("cmu enable %d\n", pdata->cmu_enable);
	} else {
		pdata->cmu_enable = false;
	}

	dev_info(&ndev->dev, "DT parsed successfully\n");
	of_node_put(timings_np);
	of_node_put(cmu_np);
	of_node_put(cmu_adbRGB_np);
	of_node_put(np_target_disp);
	return pdata;

fail_parse:
	of_node_put(cmu_np);
	of_node_put(cmu_adbRGB_np);

	if (!boot_pdata && !IS_ERR_OR_NULL(pdata)) {
		if (!IS_ERR_OR_NULL(pdata->fb))
			devm_kfree(&ndev->dev, pdata->fb);
		if (!IS_ERR_OR_NULL(pdata->default_out))
			devm_kfree(&ndev->dev, pdata->default_out);
		devm_kfree(&ndev->dev, pdata);
	}

	return ERR_PTR(err);
}

#ifdef CONFIG_OF
struct tegra_dc_common_platform_data
		*of_dc_common_parse_platform_data(struct platform_device *pdev)
{
	struct tegra_dc_common_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *imp_np = NULL;

	u32 temp;
	pdata = devm_kzalloc(&pdev->dev,
		sizeof(struct tegra_dc_common_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "not enough memory\n");
		return NULL;
	}

	if (!of_property_read_u32(np, "nvidia,valid_heads", &temp)) {
		pdata->valid_heads = (int)temp;
		OF_DC_LOG("valid_heads %d\n", pdata->valid_heads);
	} else {
		goto fail_parse_pdata;
	}

	imp_np = of_parse_phandle(np, "nvidia,disp_imp_table", 0);
	if (imp_np && of_device_is_available(imp_np)) {
		pdata->imp_table = devm_kzalloc(&pdev->dev,
				sizeof(*(pdata->imp_table)), GFP_KERNEL);
		if (!pdata->imp_table)
			goto fail_parse_pdata;

		if (tegra_dc_parse_imp_data(imp_np, pdev, pdata->imp_table))
			goto fail_parse_imp_table;
	}

	return pdata;

fail_parse_imp_table:
	kfree(pdata->imp_table);
fail_parse_pdata:
	kfree(pdata);
	return NULL;
}
#else
struct tegra_dc_common_platform_data
		*of_dc_common_parse_platform_data(struct platform_device *pdev)
{
	return NULL;
}
#endif

static int __init check_fb_console_map_default(void)
{
	struct device_node *np_l4t = NULL;
	struct property *pp_l4t = NULL;
	int len, res = 0;

	np_l4t = of_find_node_by_path("/chosen/plugin-manager/odm-data");
	if (np_l4t) {
		pp_l4t = of_find_property(np_l4t, "l4t", &len);
		if (pp_l4t) {
			os_l4t = true;
			res = 1;
		}
		pr_info("OS set in device tree is%s L4T.\n", pp_l4t ? "":" not");
	}
	return res;
}
core_initcall(check_fb_console_map_default);

bool is_os_l4t(void)
{
	return os_l4t;
}
