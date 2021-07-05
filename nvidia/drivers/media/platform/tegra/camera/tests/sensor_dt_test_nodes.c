/*
 * sensor_dt_test_nodes - sensor device tree test node definitions
 *
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/errno.h>

#include "sensor_dt_test.h"
#include "sensor_dt_test_nodes.h"

int sv_dt_make_root_node_props(struct sv_dt_node *node)
{
	MAKE_ATTRS(attrs);
	int err = 0;

	if (node == NULL)
		return -EINVAL;

	MAKE_LINK_ALL(node, attrs, NULL, "compatible", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "reg", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "mclk", LTYPE_OPTIONAL);

	MAKE_LINK_ALL(node, attrs, NULL, "*-gpio", LTYPE_OPTIONAL);
	MAKE_LINK_ALL(node, attrs, NULL, "*-supply", LTYPE_OPTIONAL);
	MAKE_LINK_ALL(node, attrs, NULL, "*-reg", LTYPE_OPTIONAL);

	MAKE_LINK_ALL(node, attrs, NULL, "physical_w", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "physical_h", LTYPE_REQUIRED);

	MAKE_LINK_ALL(node, attrs, NULL, "sensor_model", LTYPE_OPTIONAL);
	MAKE_LINK_ALL(node, attrs, NULL,
			"post_crop_frame_drop", LTYPE_OPTIONAL);
	MAKE_LINK_ALL(node, attrs, NULL, "use_decibel_gain", LTYPE_OPTIONAL);
	MAKE_LINK_ALL(node, attrs, NULL, "delayed_gain", LTYPE_OPTIONAL);
	MAKE_LINK_ALL(node, attrs, NULL,
			"user_sensor_mode_id", LTYPE_OPTIONAL);

make_link_fail:
	return err;
}

int sv_dt_make_modeX_node_props(struct sv_dt_node *node)
{
	MAKE_ATTRS(attrs);
	int err = 0;

	if (node == NULL)
		return -EINVAL;

	MAKE_LINK_ALL(node, attrs, NULL, "mclk_khz", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "num_lanes", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "tegra_sinterface", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "discontinuous_clk", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "cil_settletime", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "dpcm_enable", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "active_h", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "active_w", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "readout_orientation", LTYPE_REQUIRED);

	MAKE_LINK_ALL(node, attrs, NULL, "pixel_t", LTYPE_DEPRECATED);
	MAKE_LINK_ALL(node, attrs, "pixel_t", "mode_type", LTYPE_ALTERNATIVE);
	MAKE_LINK_ALL(node, attrs, "pixel_t", "csi_pixel_bit_depth",
			LTYPE_ALTERNATIVE);
	MAKE_LINK_ALL(node, attrs, "pixel_t", "pixel_phase", LTYPE_ALTERNATIVE);

	MAKE_LINK_ALL(node, attrs, NULL, "line_length", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "mclk_multiplier", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "pix_clk_hz", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "inherent_gain", LTYPE_REQUIRED);

	MAKE_LINK_ALL(node, attrs, NULL, "min_hdr_ratio", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "max_hdr_ratio", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "embedded_metadata_height",
			LTYPE_REQUIRED);

	MAKE_LINK_ALL_FULL(node, attrs, "pixel_t", "dynamic_pixel_bit_depth",
			LTYPE_ALTERNATIVE,
			MODE_TYPE_WDR_PWL | MODE_TYPE_WDR_DOL);
	MAKE_LINK_ALL_FULL(node, attrs, "pixel_t", "num_control_point",
			LTYPE_ALTERNATIVE, MODE_TYPE_WDR_PWL);
	MAKE_LINK_ALL_FULL(node, attrs, "pixel_t", "control_point_x_[0-9]*",
			LTYPE_ALTERNATIVE, MODE_TYPE_WDR_PWL);
	MAKE_LINK_ALL_FULL(node, attrs, "pixel_t", "control_point_y_[0-9]*",
			LTYPE_ALTERNATIVE, MODE_TYPE_WDR_PWL);

	MAKE_LINK_ALL_FULL(node, attrs, NULL, "num_of_exposure",
			LTYPE_REQUIRED, MODE_TYPE_WDR_DOL);
	MAKE_LINK_ALL_FULL(node, attrs, NULL, "num_of_ignored_lines",
			LTYPE_REQUIRED, MODE_TYPE_WDR_DOL);
	MAKE_LINK_ALL_FULL(node, attrs, NULL, "num_of_lines_offset_[0-9]*",
			LTYPE_REQUIRED, MODE_TYPE_WDR_DOL);
	MAKE_LINK_ALL_FULL(node, attrs, NULL, "num_of_ignored_pixels",
			LTYPE_REQUIRED, MODE_TYPE_WDR_DOL);
	MAKE_LINK_ALL_FULL(node, attrs, NULL, "num_of_left_margin_pixels",
			LTYPE_REQUIRED, MODE_TYPE_WDR_DOL);
	MAKE_LINK_ALL_FULL(node, attrs, NULL, "num_of_right_margin_pixels",
			LTYPE_REQUIRED, MODE_TYPE_WDR_DOL);

	MAKE_LINK_ALL(node, attrs, NULL, "min_gain_val", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "max_gain_val", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "min_exp_time", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "max_exp_time", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "min_framerate", LTYPE_REQUIRED);
	MAKE_LINK_ALL(node, attrs, NULL, "max_framerate", LTYPE_REQUIRED);

	MAKE_LINK_FULL(node, attrs, NULL, "gain_factor",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "default_gain",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "step_gain_val",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "exposure_factor",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "default_exp_time",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "step_exp_time",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "framerate_factor",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "default_framerate",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));
	MAKE_LINK_FULL(node, attrs, NULL, "step_framerate",
			MAKE_LATTR(attrs, TVCF_VERSION_V2_0_0, LTYPE_REQUIRED));

make_link_fail:
	return err;
}
