/*
 * dp_lt.c: DP Link Training functions.
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/kernel.h>
#include "dc.h"
#include <linux/moduleparam.h>
#include "dp_lt.h"
#include "dpaux.h"
#include "dp.h"
#include "sor.h"
#include "sor_regs.h"

/*
 * By default we do not trigger LT if link is already stable.
 * However, analyzer tests in their latest firmware are explicitly
 * checking for LT. Add param to suppress this optimization. It
 * has no usecase other than analyzer specific requirement.
 */
static bool force_trigger_lt;
module_param(force_trigger_lt, bool, 0644);
MODULE_PARM_DESC(force_trigger_lt,
	"Retrigger LT even if link is already stable");

static int lane_fallback_table[] = {1, 2, 4};

static void set_lt_state(struct tegra_dp_lt_data *lt_data,
			int target_state, int delay_ms);
static void set_lt_tpg(struct tegra_dp_lt_data *lt_data, u32 tp);

/* Check if post-cursor2 programming is supported */
static inline bool is_pc2_supported(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_link_config *cfg = &lt_data->dp->link_cfg;
	struct tegra_dp_out *dp_out = lt_data->dp->dc->out->dp_out;

	return (!dp_out->pc2_disabled &&
		cfg->tps == TEGRA_DC_DP_TRAINING_PATTERN_3);
}

/*
 * Wait period before reading link status.
 * If dpcd addr 0xe TRAINING_AUX_RD_INTERVAL absent or zero,
 * wait for 100us for CR status and 400us for CE status.
 * Otherwise, use values as specified.
 */
static inline u32 wait_aux_training(struct tegra_dp_lt_data *lt_data,
					bool is_clk_recovery)
{
	if (!lt_data->aux_rd_interval)
		is_clk_recovery ? usleep_range(150, 200) :
					usleep_range(450, 500);
	else
		msleep(lt_data->aux_rd_interval * 4);

	return lt_data->aux_rd_interval;
}

static inline int get_next_lower_lane_count(struct tegra_dc_dp_data *dp)
{
	u8 cur_lanes = dp->link_cfg.lane_count;
	int k;

	for (k = ARRAY_SIZE(lane_fallback_table) - 1; k >= 0; k--) {
		if (cur_lanes == lane_fallback_table[k])
			break;
	}

	if (k <= 0)
		return -ENOENT;

	return lane_fallback_table[k - 1];
}

static inline int get_next_lower_link_rate(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	u8 cur_link_rate = dp->link_cfg.link_bw;
	int k;

	for (k = sor->num_link_speeds - 1; k >= 0; k--) {
		if (cur_link_rate == sor->link_speeds[k].link_rate)
			break;
	}

	if (k <= 0)
		return -ENOENT;

	return sor->link_speeds[k - 1].link_rate;
}

static bool get_clock_recovery_status(struct tegra_dp_lt_data *lt_data,
				u32 *cr_lane_mask)
{
	u32 cnt;
	u32 n_lanes = lt_data->n_lanes;
	u8 data_ptr = 0;
	u8 cr_done = 1;
	u32 loopcnt = (n_lanes == 1) ? 1 : n_lanes >> 1;

	*cr_lane_mask = 0;
	for (cnt = 0; cnt < loopcnt; cnt++) {
		tegra_dc_dp_dpcd_read(lt_data->dp,
			(NV_DPCD_LANE0_1_STATUS + cnt), &data_ptr);

		if (n_lanes == 1) {
			cr_done = data_ptr & 0x1;
			*cr_lane_mask = cr_done;
		} else {
			u32 tmp;

			/* Lane 0 or 2 */
			tmp = data_ptr & 0x1;
			cr_done &= tmp;
			*cr_lane_mask |= tmp << (cnt * 2);

			/* Lane 1 or 3 */
			data_ptr >>= NV_DPCD_STATUS_LANEXPLUS1_CR_DONE_SHIFT;
			tmp = data_ptr & 0x1;
			cr_done &= tmp;
			*cr_lane_mask |= (tmp << 1) << (cnt * 2);
		}
	}

	return cr_done;
}

static bool get_channel_eq_status(struct tegra_dp_lt_data *lt_data)
{
	u32 cnt;
	u32 n_lanes = lt_data->n_lanes;
	u8 data_ptr = 0;
	bool ce_done = true;

	/* support for 1 lane */
	u32 loopcnt = (n_lanes == 1) ? 1 : n_lanes >> 1;

	for (cnt = 0; cnt < loopcnt; cnt++) {
		tegra_dc_dp_dpcd_read(lt_data->dp,
			(NV_DPCD_LANE0_1_STATUS + cnt), &data_ptr);

		if (n_lanes == 1) {
			ce_done = (data_ptr &
			(0x1 << NV_DPCD_STATUS_LANEX_CHN_EQ_DONE_SHIFT)) &&
			(data_ptr &
			(0x1 << NV_DPCD_STATUS_LANEX_SYMBOL_LOCKED_SHFIT));
			break;
		} else if (!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEX_CHN_EQ_DONE_SHIFT)) ||
		!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEX_SYMBOL_LOCKED_SHFIT)) ||
		!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEXPLUS1_CHN_EQ_DONE_SHIFT)) ||
		!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEXPLUS1_SYMBOL_LOCKED_SHIFT))) {
			ce_done = false;
			break;
		}
	}

	if (ce_done) {
		tegra_dc_dp_dpcd_read(lt_data->dp,
			NV_DPCD_LANE_ALIGN_STATUS_UPDATED, &data_ptr);
		if (!(data_ptr &
			NV_DPCD_LANE_ALIGN_STATUS_INTERLANE_ALIGN_DONE_YES))
			ce_done = false;
	}

	return ce_done;
}

static bool get_lt_status(struct tegra_dp_lt_data *lt_data)
{
	bool cr_done, ce_done;
	u32 cr_lane_mask;

	cr_done = get_clock_recovery_status(lt_data, &cr_lane_mask);
	if (!cr_done)
		return false;

	ce_done = get_channel_eq_status(lt_data);

	return ce_done;
}

/*
 * get updated voltage swing, pre-emphasis and
 * post-cursor2 settings from panel
 */
static void get_lt_new_config(struct tegra_dp_lt_data *lt_data)
{
	u32 cnt;
	u8 data_ptr;
	u32 n_lanes = lt_data->n_lanes;
	u32 *vs = lt_data->drive_current;
	u32 *pe = lt_data->pre_emphasis;
	u32 *pc = lt_data->post_cursor2;
	bool pc2_supported = is_pc2_supported(lt_data);

	/* support for 1 lane */
	u32 loopcnt = (n_lanes == 1) ? 1 : n_lanes >> 1;

	for (cnt = 0; cnt < loopcnt; cnt++) {
		tegra_dc_dp_dpcd_read(lt_data->dp,
			(NV_DPCD_LANE0_1_ADJUST_REQ + cnt), &data_ptr);
		pe[2 * cnt] = (data_ptr & NV_DPCD_ADJUST_REQ_LANEX_PE_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEX_PE_SHIFT;
		vs[2 * cnt] = (data_ptr & NV_DPCD_ADJUST_REQ_LANEX_DC_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEX_DC_SHIFT;
		pe[1 + 2 * cnt] =
			(data_ptr & NV_DPCD_ADJUST_REQ_LANEXPLUS1_PE_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEXPLUS1_PE_SHIFT;
		vs[1 + 2 * cnt] =
			(data_ptr & NV_DPCD_ADJUST_REQ_LANEXPLUS1_DC_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEXPLUS1_DC_SHIFT;
	}

	if (pc2_supported) {
		tegra_dc_dp_dpcd_read(lt_data->dp,
				NV_DPCD_ADJUST_REQ_POST_CURSOR2, &data_ptr);
		for (cnt = 0; cnt < n_lanes; cnt++) {
			pc[cnt] = (data_ptr >>
			NV_DPCD_ADJUST_REQ_POST_CURSOR2_LANE_SHIFT(cnt)) &
			NV_DPCD_ADJUST_REQ_POST_CURSOR2_LANE_MASK;
		}
	}

	for (cnt = 0; cnt < n_lanes; cnt++)
		pr_info("dp lt: new config: lane %d: "
			"vs level: %d, pe level: %d, pc2 level: %d\n",
			cnt, vs[cnt], pe[cnt], pc2_supported ? pc[cnt] : 0);
}

static void set_tx_pu(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;
	struct tegra_dc_sor_data *sor = dp->sor;
	u32 n_lanes = lt_data->n_lanes;
	int cnt = 1;
	u32 *vs = lt_data->drive_current;
	u32 *pe = lt_data->pre_emphasis;
	u32 *pc = lt_data->post_cursor2;
	u32 max_tx_pu;
	u32 nv_sor_dp_padctl_reg = nv_sor_dp_padctl(sor->portnum);

	if (!dp->pdata || (dp->pdata && dp->pdata->tx_pu_disable)) {
		tegra_sor_write_field(dp->sor,
				nv_sor_dp_padctl_reg,
				NV_SOR_DP_PADCTL_TX_PU_ENABLE,
				NV_SOR_DP_PADCTL_TX_PU_DISABLE);
		lt_data->tx_pu = 0;
		return;
	}

	max_tx_pu = dp->pdata->lt_data[DP_TX_PU].data[pc[0]][vs[0]][pe[0]];
	for (; cnt < n_lanes; cnt++) {
		max_tx_pu = (max_tx_pu <
			dp->pdata->lt_data[DP_TX_PU].data[pc[cnt]][vs[cnt]][pe[cnt]]) ?
			dp->pdata->lt_data[DP_TX_PU].data[pc[cnt]][vs[cnt]][pe[cnt]] :
			max_tx_pu;
	}

	lt_data->tx_pu = max_tx_pu;
	tegra_sor_write_field(sor, nv_sor_dp_padctl_reg,
				NV_SOR_DP_PADCTL_TX_PU_VALUE_DEFAULT_MASK,
				(max_tx_pu <<
				NV_SOR_DP_PADCTL_TX_PU_VALUE_SHIFT |
				NV_SOR_DP_PADCTL_TX_PU_ENABLE));

	pr_info("dp lt: tx_pu: 0x%x\n", max_tx_pu);
}

/*
 * configure voltage swing, pre-emphasis,
 * post-cursor2 and tx_pu on host and sink
 */
static void set_lt_config(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;
	struct tegra_dc_sor_data *sor = dp->sor;
	u32 n_lanes = lt_data->n_lanes;
	bool pc2_supported = is_pc2_supported(lt_data);
	int i, cnt;
	u32 val;
	u32 *vs = lt_data->drive_current;
	u32 *pe = lt_data->pre_emphasis;
	u32 *pc = lt_data->post_cursor2;
	u32 aux_stat = 0;
	u8 training_lanex_set[4] = {0, 0, 0, 0};
	u32 training_lanex_set_size = sizeof(training_lanex_set);

	/* support for 1 lane */
	u32 loopcnt = (n_lanes == 1) ? 1 : n_lanes >> 1;

	/*
	 * apply voltage swing, preemphasis, postcursor2 and tx_pu
	 * prod settings to each lane based on levels
	 */
	for (i = 0; i < n_lanes; i++) {
		u32 mask = 0;
		u32 pe_reg, vs_reg, pc_reg;
		u32 shift = 0;
		cnt = sor->xbar_ctrl[i];

		switch (cnt) {
		case 0:
			mask = NV_SOR_PR_LANE2_DP_LANE0_MASK;
			shift = NV_SOR_PR_LANE2_DP_LANE0_SHIFT;
			break;
		case 1:
			mask = NV_SOR_PR_LANE1_DP_LANE1_MASK;
			shift = NV_SOR_PR_LANE1_DP_LANE1_SHIFT;
			break;
		case 2:
			mask = NV_SOR_PR_LANE0_DP_LANE2_MASK;
			shift = NV_SOR_PR_LANE0_DP_LANE2_SHIFT;
			break;
		case 3:
			mask = NV_SOR_PR_LANE3_DP_LANE3_MASK;
			shift = NV_SOR_PR_LANE3_DP_LANE3_SHIFT;
			break;
		default:
			dev_err(&dp->dc->ndev->dev,
				"dp: incorrect lane cnt\n");
		}

		pe_reg = dp->pdata->lt_data[DP_PE].data[pc[i]][vs[i]][pe[i]];
		vs_reg = dp->pdata->lt_data[DP_VS].data[pc[i]][vs[i]][pe[i]];
		pc_reg = dp->pdata->lt_data[DP_PC].data[pc[i]][vs[i]][pe[i]];

		tegra_sor_write_field(sor, NV_SOR_PR(sor->portnum),
						mask, (pe_reg << shift));
		tegra_sor_write_field(sor, NV_SOR_DC(sor->portnum),
						mask, (vs_reg << shift));
		if (pc2_supported) {
			tegra_sor_write_field(
					sor, NV_SOR_POSTCURSOR(sor->portnum),
					mask, (pc_reg << shift));
		}

		pr_info("dp lt: config: lane %d: "
			"vs level: %d, pe level: %d, pc2 level: %d\n",
			i, vs[i], pe[i], pc2_supported ? pc[i] : 0);
	}
	set_tx_pu(lt_data);
	usleep_range(15, 20); /* HW stabilization delay */

	/* apply voltage swing and preemphasis levels to panel for each lane */
	for (cnt = n_lanes - 1; cnt >= 0; cnt--) {
		u32 max_vs_flag = tegra_dp_is_max_vs(pe[cnt], vs[cnt]);
		u32 max_pe_flag = tegra_dp_is_max_pe(pe[cnt], vs[cnt]);

		val = (vs[cnt] << NV_DPCD_TRAINING_LANEX_SET_DC_SHIFT) |
			(max_vs_flag ?
			NV_DPCD_TRAINING_LANEX_SET_DC_MAX_REACHED_T :
			NV_DPCD_TRAINING_LANEX_SET_DC_MAX_REACHED_F) |
			(pe[cnt] << NV_DPCD_TRAINING_LANEX_SET_PE_SHIFT) |
			(max_pe_flag ?
			NV_DPCD_TRAINING_LANEX_SET_PE_MAX_REACHED_T :
			NV_DPCD_TRAINING_LANEX_SET_PE_MAX_REACHED_F);

		training_lanex_set[cnt] = val;
	}
	tegra_dc_dpaux_write(dp->dpaux, DPAUX_DP_AUXCTL_CMD_AUXWR,
			NV_DPCD_TRAINING_LANE0_SET, training_lanex_set,
			&training_lanex_set_size, &aux_stat);

	/* apply postcursor2 levels to panel for each lane */
	if (pc2_supported) {
		for (cnt = 0; cnt < loopcnt; cnt++) {
			u32 max_pc_flag0 = tegra_dp_is_max_pc(pc[cnt]);
			u32 max_pc_flag1 = tegra_dp_is_max_pc(pc[cnt + 1]);

			val = (pc[cnt] << NV_DPCD_LANEX_SET2_PC2_SHIFT) |
				(max_pc_flag0 ?
				NV_DPCD_LANEX_SET2_PC2_MAX_REACHED_T :
				NV_DPCD_LANEX_SET2_PC2_MAX_REACHED_F) |
				(pc[cnt + 1] <<
				NV_DPCD_LANEXPLUS1_SET2_PC2_SHIFT) |
				(max_pc_flag1 ?
				NV_DPCD_LANEXPLUS1_SET2_PC2_MAX_REACHED_T :
				NV_DPCD_LANEXPLUS1_SET2_PC2_MAX_REACHED_F);
			tegra_dc_dp_dpcd_write(dp,
				(NV_DPCD_TRAINING_LANE0_1_SET2 + cnt), val);
		}
	}
}

static int do_fast_lt_no_handshake(struct tegra_dp_lt_data *lt_data)
{
	BUG_ON(!lt_data->lt_config_valid);

	/* transmit link training pattern 1 for min of 500us */
	set_lt_tpg(lt_data, TEGRA_DC_DP_TRAINING_PATTERN_1);
	usleep_range(500, 600);

	/* transmit channel equalization training pattern for min of 500us */
	set_lt_tpg(lt_data, lt_data->dp->link_cfg.tps);
	usleep_range(500, 600);

	return 0;
}

static void lt_data_sw_reset(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp =  lt_data->dp;

	BUG_ON(!dp);

	lt_data->lt_config_valid = false;
	lt_data->cr_adj_retry = 0;
	lt_data->cr_max_retry = 0;
	lt_data->ce_retry = 0;
	lt_data->tx_pu = 0;
	lt_data->n_lanes = dp->link_cfg.lane_count;
	lt_data->link_bw = dp->link_cfg.link_bw;
	lt_data->no_aux_handshake = dp->link_cfg.support_fast_lt;
	lt_data->aux_rd_interval = dp->link_cfg.aux_rd_interval;

	memset(lt_data->pre_emphasis, PRE_EMPHASIS_L0,
		sizeof(lt_data->pre_emphasis));
	memset(lt_data->drive_current, DRIVE_CURRENT_L0,
		sizeof(lt_data->drive_current));
	memset(lt_data->post_cursor2, POST_CURSOR2_L0,
		sizeof(lt_data->post_cursor2));
}

static void lt_data_reset(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp =  lt_data->dp;

	BUG_ON(!dp);

	lt_data_sw_reset(lt_data);

	/* reset LT data on controller and panel only if hpd is asserted */
	if (tegra_dc_hpd(dp->dc)) {
		/*
		 * Training pattern is disabled here. Do not HW reset
		 * lt config i.e. vs, pe, pc2. CTS mandates modifying these
		 * only when training pattern is enabled.
		 */
		tegra_dp_update_link_config(dp);
	}
}

static void set_lt_tpg(struct tegra_dp_lt_data *lt_data, u32 tp)
{
	bool cur_hpd = tegra_dc_hpd(lt_data->dp->dc);
	struct tegra_dc_dp_data *dp = lt_data->dp;

	if (lt_data->tps == tp)
		return;

	if (cur_hpd)
		tegra_dp_tpg(dp, tp, lt_data->n_lanes);
	else
		/*
		 * hpd deasserted. Just set training
		 * sequence from host side and exit
		 */
		tegra_sor_tpg(dp->sor, tp, lt_data->n_lanes);
	lt_data->tps = tp;
}

static void lt_failed(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;

	mutex_lock(&lt_data->lock);

	tegra_dc_sor_detach(dp->sor);
	set_lt_tpg(lt_data, TEGRA_DC_DP_TRAINING_PATTERN_DISABLE);
	lt_data_reset(lt_data);

	mutex_unlock(&lt_data->lock);
}

static void lt_passed(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;

	mutex_lock(&lt_data->lock);

	lt_data->lt_config_valid = true;
	set_lt_tpg(lt_data, TEGRA_DC_DP_TRAINING_PATTERN_DISABLE);
	tegra_dc_sor_attach(dp->sor);

	mutex_unlock(&lt_data->lock);
}

static void lt_reset_state(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp;
	struct tegra_dc_sor_data *sor;
	bool cur_hpd;
	int tgt_state;
	int timeout;

	BUG_ON(!lt_data || !lt_data->dp || !lt_data->dp->sor);
	dp = lt_data->dp;
	sor = dp->sor;

	cur_hpd = tegra_dc_hpd(dp->dc);

	if (lt_data->force_disable) {
		pr_info("dp lt: link training force disable\n");
		lt_data->force_disable = false;
		lt_data->force_trigger = false;
		if (!lt_data->no_aux_handshake)
			lt_data->lt_config_valid = false;
		tgt_state = STATE_DONE_FAIL;
		timeout = -1;
		goto done;
	} else if (!cur_hpd || !dp->link_cfg.is_valid) {
		pr_info("dp lt: cur_hpd: %d, link cfg valid: %d\n",
			!!cur_hpd, !!dp->link_cfg.is_valid);
		lt_failed(lt_data);
		lt_data->force_disable = false;
		lt_data->force_trigger = false;
		tgt_state = STATE_DONE_FAIL;
		timeout = -1;
		goto done;
	}

	if (!force_trigger_lt && !lt_data->force_trigger &&
		lt_data->lt_config_valid &&
		get_lt_status(lt_data)) {
		pr_info("dp_lt: link stable, do nothing\n");
		lt_passed(lt_data);
		tgt_state = STATE_DONE_PASS;
		timeout = -1;
		goto done;
	}
	lt_data->force_trigger = false;

	/*
	 * detach SOR early.
	 * DP lane count can be changed only
	 * when SOR is asleep.
	 * DP link bandwidth can be changed only
	 * when SOR is in safe mode.
	 */
	mutex_lock(&lt_data->lock);
	tegra_dc_sor_detach(sor);
	mutex_unlock(&lt_data->lock);

	lt_data_reset(lt_data);
	tgt_state = STATE_CLOCK_RECOVERY;
	timeout = 0;

	WARN_ON(lt_data->tps != TEGRA_DC_DP_TRAINING_PATTERN_DISABLE);

	/*
	 * pre-charge main link for at
	 * least 10us before initiating
	 * link training
	 */
	mutex_lock(&lt_data->lock);
	tegra_sor_precharge_lanes(sor);
	mutex_unlock(&lt_data->lock);
done:
	set_lt_state(lt_data, tgt_state, timeout);
}

static void fast_lt_state(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp;
	struct tegra_dc_sor_data *sor;
	int tgt_state;
	int timeout;
	bool lt_status;

	BUG_ON(!lt_data || !lt_data->dp || !lt_data->dp->sor);
	dp = lt_data->dp;
	sor = dp->sor;

	BUG_ON(!lt_data->no_aux_handshake);
	WARN_ON(lt_data->tps != TEGRA_DC_DP_TRAINING_PATTERN_DISABLE);

	mutex_lock(&lt_data->lock);

	tegra_dc_sor_detach(sor);
	set_lt_config(lt_data);
	tegra_sor_precharge_lanes(sor);

	mutex_unlock(&lt_data->lock);

	do_fast_lt_no_handshake(lt_data);
	lt_status = get_lt_status(lt_data);
	if (lt_status) {
		lt_passed(lt_data);
		tgt_state = STATE_DONE_PASS;
		timeout = -1;
	} else {
		lt_failed(lt_data);
		tgt_state = STATE_RESET;
		timeout = 0;
	}

	pr_info("dp lt: fast link training %s\n",
		tgt_state == STATE_DONE_PASS ? "pass" : "fail");
	set_lt_state(lt_data, tgt_state, timeout);
}

static inline int set_new_link_cfg(struct tegra_dp_lt_data *lt_data,
			struct tegra_dc_dp_link_config *new_cfg)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;

	if (!tegra_dc_dp_calc_config(dp, dp->mode, new_cfg))
		return -EINVAL;

	new_cfg->is_valid = true;
	dp->link_cfg = *new_cfg;

	/*
	 * Disable LT before updating the current lane count and/or link rate.
	 */
	set_lt_tpg(lt_data, TEGRA_DC_DP_TRAINING_PATTERN_DISABLE);

	tegra_dp_update_link_config(dp);

	lt_data->n_lanes = dp->link_cfg.lane_count;
	lt_data->link_bw = dp->link_cfg.link_bw;

	return 0;
}

static void lt_reduce_link_rate_state(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;
	struct tegra_dc_dp_link_config tmp_cfg;

	dp->link_cfg.is_valid = false;
	tmp_cfg = dp->link_cfg;

	tmp_cfg.link_bw = get_next_lower_link_rate(dp);
	if (tmp_cfg.link_bw < NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS)
		goto fail;

	if (set_new_link_cfg(lt_data, &tmp_cfg))
		goto fail;

	pr_info("dp lt: retry CR, lanes: %d, link rate: 0x%x\n",
		lt_data->n_lanes, lt_data->link_bw);
	set_lt_state(lt_data, STATE_CLOCK_RECOVERY, 0);
	return;
fail:
	pr_info("dp lt: link rate already lowest\n");
	lt_failed(lt_data);
	set_lt_state(lt_data, STATE_DONE_FAIL, -1);
}

static void lt_reduce_lane_count_state(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;
	struct tegra_dc_dp_link_config tmp_cfg;

	dp->link_cfg.is_valid = false;
	tmp_cfg = dp->link_cfg;

	tmp_cfg.lane_count = get_next_lower_lane_count(dp);
	if (tmp_cfg.lane_count < 1)
		goto fail;

	if (set_new_link_cfg(lt_data, &tmp_cfg))
		goto fail;

	pr_info("dp lt: retry CR, lanes: %d, link rate: 0x%x\n",
		lt_data->n_lanes, lt_data->link_bw);
	set_lt_state(lt_data, STATE_CLOCK_RECOVERY, 0);
	return;
fail:
	pr_info("dp lt: lane count already lowest\n");
	lt_failed(lt_data);
	set_lt_state(lt_data, STATE_DONE_FAIL, -1);
}

static void lt_channel_equalization_fallback(struct tegra_dp_lt_data *lt_data,
				int *tgt_state, int *timeout, u32 cr_lane_mask)
{
	struct tegra_dc_dp_data *dp = lt_data->dp;

	/*
	 * If we fallback during EQ, there are two paths that we can end up in:
	 * 1) If 4 lanes are enabled, reduce to 2 lanes and go back to the start
	 *    of CR. If 2 lanes are enabled, reduce to 1 lane and go back to the
	 *    start of CR. If only 1 lane is currently enabled, go to step 2.
	 * 2) If the current link rate is > RBR, downshift to the next lowest
	 *    link rate and reset the lane count to the highest needed.
	 *
	 * Before deciding which path to take, we need to look at the current
	 * CR status on all lanes:
	 * - If *all* the target LANEX_CR_DONE bits are still set, then we
	 *   didn't transition to this state due to loss of CR during EQ. Try
	 *   fallback path #1 first.
	 * - If *at least one* of the LANEX_CR_DONE bits is still set, it's
	 *   likely that the earlier CR phase passed due to crosstalk issues on
	 *   some unconnected lanes. In this case, prioritize lane count
	 *   reduction, and choose fallback path #1 first.
	 * - If *none* of the LANEX_CR_DONE bits are still set, it's likely that
	 *   CR was lost due to the bandwidth that we're trying to drive. Try
	 *   reducing the link rate first, and choose fallback path #2.
	 *
	 * If neither of the above two paths are valid, end link training and
	 * fail.
	 */
	if (lt_data->n_lanes > 1 && cr_lane_mask) {
		/* Fallback path #1 */
		lt_data_sw_reset(lt_data);
		*tgt_state = STATE_REDUCE_LANE_COUNT;
	} else if (lt_data->link_bw >
				NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS) {
		/* Fallback path #2 */
		lt_data_sw_reset(lt_data);
		dp->link_cfg.lane_count = dp->max_link_cfg.lane_count;
		*tgt_state = STATE_REDUCE_LINK_RATE;
	} else {
		lt_failed(lt_data);
		*tgt_state = STATE_DONE_FAIL;
	}
	*timeout = 0;
}

static void lt_channel_equalization_state(struct tegra_dp_lt_data *lt_data)
{
	int tgt_state = STATE_CHANNEL_EQUALIZATION;
	int timeout = 0;
	bool cr_done = true;
	bool ce_done = true;
	u32 cr_lane_mask;

	/*
	 * See comment above the TEGRA_DC_DP_TRAINING_PATTERN_BS_CSTM entry in
	 * sor.c for why this WAR is required.
	 */
	if (lt_data->dp->link_cfg.tps == TEGRA_DC_DP_TRAINING_PATTERN_4) {
		struct tegra_dc_sor_data *sor = lt_data->dp->sor;

		tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_0, 0x1BC6F1BC);
		tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_1, 0xC6F1BC6F);
		tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_2, 0x6F1B);

		tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
			NV_SOR_DP_CONFIG_RD_RESET_VAL_NEGATIVE,
			NV_SOR_DP_CONFIG_RD_RESET_VAL_POSITIVE);

		tegra_sor_tpg(sor, TEGRA_DC_DP_TRAINING_PATTERN_BS_CSTM,
			lt_data->n_lanes);
	}

	set_lt_tpg(lt_data, lt_data->dp->link_cfg.tps);
	wait_aux_training(lt_data, false);

	/*
	 * Fallback if any of these two conditions are true:
	 * 1) CR is lost on any of the currently enabled lanes.
	 * 2) The LANEx_CHANNEL_EQ_DONE, LANEx_SYMBOL_LOCKED, or
	 *    INTERLANE_ALIGN_DONE bits have not been set for all enabled lanes
	 *    after 5 retry attempts.
	 */

	/* Fallback condition #1 */
	cr_done = get_clock_recovery_status(lt_data, &cr_lane_mask);
	if (!cr_done) {
		pr_info("dp lt: CR lost\n");

		lt_channel_equalization_fallback(lt_data, &tgt_state, &timeout,
						cr_lane_mask);
		goto done;
	}

	ce_done = get_channel_eq_status(lt_data);
	if (ce_done) {
		lt_passed(lt_data);
		tgt_state = STATE_DONE_PASS;
		timeout = -1;
		pr_info("dp lt: CE done\n");
		goto done;
	}
	pr_info("dp lt: CE not done\n");

	/* Fallback condition #2 */
	if (++(lt_data->ce_retry) > (CE_RETRY_LIMIT + 1)) {
		pr_info("dp lt: CE retry limit %d reached\n",
				lt_data->ce_retry - 2);

		lt_channel_equalization_fallback(lt_data, &tgt_state, &timeout,
						cr_lane_mask);
		goto done;
	}

	get_lt_new_config(lt_data);
	set_lt_config(lt_data);

	pr_info("dp lt: CE retry\n");
done:
	set_lt_state(lt_data, tgt_state, timeout);
}

static inline bool is_vs_already_max(struct tegra_dp_lt_data *lt_data,
					u32 old_vs[4], u32 new_vs[4])
{
	u32 n_lanes = lt_data->n_lanes;
	int cnt;

	for (cnt = 0; cnt < n_lanes; cnt++) {
		if (old_vs[cnt] == DRIVE_CURRENT_L3 &&
			new_vs[cnt] == DRIVE_CURRENT_L3)
			continue;

		return false;
	}

	return true;
}

static bool check_cr_fallback(struct tegra_dp_lt_data *lt_data, u32 *vs_temp,
			size_t vs_temp_size)
{
	/*
	 * Fallback if any of these three conditions are true:
	 * 1) CR_DONE is not set for all enabled lanes after 10 total retries.
	 * 2) The max voltage swing has been reached.
	 * 3) The same ADJ_REQ voltage swing values have been requested by the
	 *    sink for 5 consecutive retry attempts.
	 */
	u32 *vs = lt_data->drive_current;
	bool need_fallback = false;

	/* Fallback condition #1 */
	if ((lt_data->cr_max_retry)++ >= (CR_MAX_RETRY_LIMIT - 1)) {
		need_fallback = true;
		pr_info("dp lt: CR max retry limit %d reached\n",
			lt_data->cr_max_retry);
	}

	if (!memcmp(vs_temp, vs, vs_temp_size)) {
		/* Fallback condition #2 */
		if (is_vs_already_max(lt_data, vs_temp, vs)) {
			need_fallback = true;
			pr_info("dp lt: max vs reached\n");
		}

		/* Fallback condition #3 */
		if ((lt_data->cr_adj_retry)++ >= (CR_ADJ_RETRY_LIMIT - 1)) {
			need_fallback = true;
			pr_info("dp lt: CR adj retry limit %d reached\n",
				lt_data->cr_adj_retry);
		}
	} else {
		lt_data->cr_adj_retry = 1;
	}

	return need_fallback;
}

static void lt_clock_recovery_state(struct tegra_dp_lt_data *lt_data)
{
	struct tegra_dc_dp_data *dp;
	struct tegra_dc_sor_data *sor;
	int tgt_state = STATE_CLOCK_RECOVERY;
	int timeout = 0;
	u32 *vs = lt_data->drive_current;
	bool cr_done;
	u32 vs_temp[4], cr_lane_mask;
	bool need_fallback = false;

	BUG_ON(!lt_data || !lt_data->dp || !lt_data->dp->sor);
	dp = lt_data->dp;
	sor = dp->sor;

	set_lt_tpg(lt_data, TEGRA_DC_DP_TRAINING_PATTERN_1);
	set_lt_config(lt_data);
	wait_aux_training(lt_data, true);
	cr_done = get_clock_recovery_status(lt_data, &cr_lane_mask);
	if (cr_done) {
		pr_info("dp lt: CR done\n");

		lt_data->cr_adj_retry = 0;
		lt_data->cr_max_retry = 0;
		tgt_state = STATE_CHANNEL_EQUALIZATION;
		goto done;
	}
	pr_info("dp lt: CR not done\n");

	memcpy(vs_temp, vs, sizeof(vs_temp));
	get_lt_new_config(lt_data);

	/*
	 * If we choose to fallback during CR, there are two paths:
	 * 1) If the current link rate is > RBR, downshift to the next lower
	 *    link rate. Prioritize link rate reduction over lane count
	 *    reduction.
	 * 2) If the current link rate is already RBR, check if only the
	 *    lower-numbered lanes have their CR_DONE bits set. For example, in
	 *    a 4-lane configuration, the lower-numbered lanes would be lanes 0
	 *    and 1. In a 2-lane configuration, the lower-numbered lane is lane
	 *    0 only. If this is the case, reduce the lane count accordingly
	 *    and reset the link rate to the highest needed.
	 *
	 * If neither of the above two paths are valid, end link training and
	 * fail.
	 */
	need_fallback = check_cr_fallback(lt_data, vs_temp, sizeof(vs_temp));
	if (need_fallback) {
		if (lt_data->link_bw >
				NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS) {
			/* Fallback path #1 */
			lt_data_sw_reset(lt_data);
			tgt_state = STATE_REDUCE_LINK_RATE;
			goto done;
		} else if (lt_data->n_lanes > 1) {
			/* Fallback path #2 */
			u32 expected_cr_mask;

			expected_cr_mask = (lt_data->n_lanes == 4) ? 0x3 : 0x1;
			if (expected_cr_mask == cr_lane_mask) {
				lt_data_sw_reset(lt_data);
				dp->link_cfg.link_bw = dp->max_link_cfg.link_bw;
				tgt_state = STATE_REDUCE_LANE_COUNT;
				goto done;
			}
		}

		lt_failed(lt_data);
		tgt_state = STATE_DONE_FAIL;
		goto done;
	} else {
		pr_info("dp lt: CR retry\n");
	}
done:
	set_lt_state(lt_data, tgt_state, timeout);
}

typedef void (*dispatch_func_t)(struct tegra_dp_lt_data *lt_data);
static const dispatch_func_t state_machine_dispatch[] = {
	lt_reset_state,			/* STATE_RESET */
	fast_lt_state,			/* STATE_FAST_LT */
	lt_clock_recovery_state,	/* STATE_CLOCK_RECOVERY */
	lt_channel_equalization_state,	/* STATE_CHANNEL_EQUALIZATION */
	NULL,				/* STATE_DONE_FAIL */
	NULL,				/* STATE_DONE_PASS */
	lt_reduce_link_rate_state,	/* STATE_REDUCE_LINK_RATE */
	lt_reduce_lane_count_state,	/* STATE_REDUCE_LANE_COUNT */
};

static void handle_lt_hpd_evt(struct tegra_dp_lt_data *lt_data)
{
	int tgt_state = STATE_RESET;

	if (lt_data->lt_config_valid &&
		lt_data->no_aux_handshake &&
		!lt_data->force_disable)
		tgt_state = STATE_FAST_LT;

	set_lt_state(lt_data, tgt_state, 0);
}

static void lt_worker(struct work_struct *work)
{
	int pending_lt_evt;
	struct tegra_dp_lt_data *lt_data = container_of(to_delayed_work(work),
					struct tegra_dp_lt_data, dwork);

	/*
	 * Observe and clear pending flag
	 * and latch the current HPD state.
	 */
	mutex_lock(&lt_data->lock);
	pending_lt_evt = lt_data->pending_evt;
	lt_data->pending_evt = 0;
	mutex_unlock(&lt_data->lock);

	pr_info("dp lt: state %d (%s), pending_lt_evt %d\n",
		lt_data->state, tegra_dp_lt_state_names[lt_data->state],
		pending_lt_evt);

	if (pending_lt_evt) {
		handle_lt_hpd_evt(lt_data);
	} else if (lt_data->state < ARRAY_SIZE(state_machine_dispatch)) {
		dispatch_func_t func = state_machine_dispatch[lt_data->state];

		if (!func)
			pr_warn("dp lt: NULL state handler in state %d\n",
			lt_data->state);
		else
			func(lt_data);
	} else {
		pr_warn("dp lt: unexpected state scheduled %d",
			lt_data->state);
	}
}

static void sched_lt_work(struct tegra_dp_lt_data *lt_data, int delay_ms)
{
	cancel_delayed_work(&lt_data->dwork);

	if ((delay_ms >= 0) && !lt_data->shutdown)
		schedule_delayed_work(&lt_data->dwork,
					msecs_to_jiffies(delay_ms));
}

static void set_lt_state(struct tegra_dp_lt_data *lt_data,
			int target_state, int delay_ms)
{
	mutex_lock(&lt_data->lock);

	pr_info("dp lt: switching from state %d (%s) to state %d (%s)\n",
		lt_data->state, tegra_dp_lt_state_names[lt_data->state],
		target_state, tegra_dp_lt_state_names[target_state]);

	lt_data->state = target_state;

	/* we have reached final state. notify others. */
	if (target_state == STATE_DONE_PASS ||
		target_state == STATE_DONE_FAIL)
		complete_all(&lt_data->lt_complete);

	/*
	 * If the pending_hpd_evt flag is already set, don't bother to
	 * reschedule the state machine worker. We should be able to assert
	 * that there is a worker callback already scheduled, and that it is
	 * scheduled to run immediately
	 */
	if (!lt_data->pending_evt)
		sched_lt_work(lt_data, delay_ms);

	mutex_unlock(&lt_data->lock);
}

int tegra_dp_get_lt_state(struct tegra_dp_lt_data *lt_data)
{
	int ret;

	mutex_lock(&lt_data->lock);

	ret = lt_data->state;

	mutex_unlock(&lt_data->lock);

	return ret;
}

void tegra_dp_lt_set_pending_evt(struct tegra_dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);

	/* always schedule work any time there is a pending lt event */
	lt_data->pending_evt = 1;
	sched_lt_work(lt_data, 0);

	mutex_unlock(&lt_data->lock);
}

/*
 * Marks previous LT configuration data as invalid.
 * Full LT is required to get new LT config data.
 */
void tegra_dp_lt_invalidate(struct tegra_dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);
	lt_data->lt_config_valid = false;
	mutex_unlock(&lt_data->lock);
}

/* block till link training has reached final state */
long tegra_dp_lt_wait_for_completion(struct tegra_dp_lt_data *lt_data,
			int target_state, unsigned long timeout_ms)
{
	might_sleep();

	if (target_state == tegra_dp_get_lt_state(lt_data))
		return 1;

	mutex_lock(&lt_data->lock);
	reinit_completion(&lt_data->lt_complete);
	mutex_unlock(&lt_data->lock);

	return wait_for_completion_timeout(&lt_data->lt_complete,
					msecs_to_jiffies(timeout_ms));
}

void tegra_dp_lt_force_disable(struct tegra_dp_lt_data *lt_data)
{
	mutex_lock(&lt_data->lock);
	lt_data->force_disable = true;
	mutex_unlock(&lt_data->lock);

	tegra_dp_lt_set_pending_evt(lt_data);
}

void tegra_dp_lt_init(struct tegra_dp_lt_data *lt_data,
			struct tegra_dc_dp_data *dp)
{
	BUG_ON(!dp || !lt_data || !dp->dc);

	lt_data->dp = dp;

	/* Change for seamless */
	if (tegra_dc_hpd(dp->dc) &&
		dp->dc->initialized) {
		lt_data->state = STATE_DONE_PASS;
		/* Set config valid to false */
		/* So that fresh LT starts   */
		lt_data->lt_config_valid = false;
	} else {
		lt_data->state = STATE_RESET;
		lt_data->pending_evt = 0;
		lt_data->shutdown = 0;
		lt_data_sw_reset(lt_data);
	}

	mutex_init(&lt_data->lock);
	INIT_DELAYED_WORK(&lt_data->dwork, lt_worker);
	init_completion(&lt_data->lt_complete);
}
