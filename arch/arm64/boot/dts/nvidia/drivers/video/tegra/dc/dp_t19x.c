/*
 * drivers/video/tegra/dc/dp_t19x.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "dc.h"
#include "dp.h"
#include "dp_lt.h"
#include "dp_t19x.h"
#include "sor.h"
#include "sor_t19x.h"

int tegra_dp_init_max_link_cfg_t19x(struct tegra_dc_dp_data *dp,
				    struct tegra_dc_dp_link_config *cfg)
{
	u8 dpcd_data = 0;
	int ret = 0;

	cfg->max_lane_count = tegra_dc_dp_get_max_lane_count(dp, &dpcd_data);
	if (cfg->max_lane_count == 0) {
		dev_err(&dp->dc->ndev->dev,
		"dp: Invalid max lane count: %u\n", cfg->max_lane_count);
		return -EINVAL;
	}

	if (dpcd_data & NV_DPCD_MAX_LANE_COUNT_TPS3_SUPPORTED_YES)
		cfg->tps = TEGRA_DC_DP_TRAINING_PATTERN_3;
	else
		cfg->tps = TEGRA_DC_DP_TRAINING_PATTERN_2;

	cfg->support_enhanced_framing =
	(dpcd_data & NV_DPCD_MAX_LANE_COUNT_ENHANCED_FRAMING_YES) ?
	true : false;

	if (dp->sink_cap_valid) {
		dpcd_data = dp->sink_cap[NV_DPCD_MAX_DOWNSPREAD];
	} else {
		ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_DOWNSPREAD,
					    &dpcd_data);
		if (ret)
			return ret;
	}

	/*
	 * The check for TPS4 should be after the check for TPS3. That helps
	 * assign a higher priority to TPS4
	 */
	if (dpcd_data & NV_DPCD_MAX_DOWNSPREAD_TPS4_SUPPORTED_YES)
		cfg->tps = TEGRA_DC_DP_TRAINING_PATTERN_4;

	cfg->downspread =
		(dpcd_data & NV_DPCD_MAX_DOWNSPREAD_VAL_0_5_PCT) ?
		true : false;
	cfg->support_fast_lt = (dpcd_data &
		NV_DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE_LT_T) ?
		true : false;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_TRAINING_AUX_RD_INTERVAL,
				    &dpcd_data);
	if (ret)
		return ret;

	cfg->aux_rd_interval = dpcd_data &
				NV_DPCD_TRAINING_AUX_RD_INTERVAL_MASK;

	cfg->max_link_bw = tegra_dc_dp_get_max_link_bw(dp);
	if (cfg->max_link_bw == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: Invalid max link bw: %u\n", cfg->max_link_bw);
		return -EINVAL;
	}
	tegra_dp_set_max_link_bw(dp->sor, cfg);

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_EDP_CONFIG_CAP, &dpcd_data);
	if (ret)
		return ret;

	cfg->alt_scramber_reset_cap =
		(dpcd_data & NV_DPCD_EDP_CONFIG_CAP_ASC_RESET_YES) ?
		true : false;
	cfg->only_enhanced_framing = (dpcd_data &
		NV_DPCD_EDP_CONFIG_CAP_FRAMING_CHANGE_YES) ?
		true : false;
	cfg->edp_cap = (dpcd_data &
		NV_DPCD_EDP_CONFIG_CAP_DISPLAY_CONTROL_CAP_YES) ?
		true : false;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_FEATURE_ENUM_LIST, &dpcd_data);
	if (ret)
		return ret;

	cfg->support_vsc_ext_colorimetry = (dpcd_data &
		NV_DPCD_FEATURE_ENUM_LIST_VSC_EXT_COLORIMETRY) ?
		true : false;

	return 0;
}
