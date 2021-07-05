/*
 * sor.c: Functions implementing tegra dc sor interface.
 *
 * Copyright (c) 2011-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/nvhost.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/tegra_prod.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/tegra_pm_domains.h>
#include <uapi/video/tegra_dc_ext.h>

#include "dc.h"
#include "sor.h"
#include "sor_regs.h"
#include "dc_priv.h"
#include "dp.h"
#include "dc_common.h"

static const struct tegra_dc_sor_link_speed link_speed_table[] = {
	[TEGRA_DC_SOR_LINK_SPEED_G1_62] = {
		.prod_prop = "prod_c_rbr",
		.max_link_bw = 1620,
		.link_rate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS,
	},
	[TEGRA_DC_SOR_LINK_SPEED_G2_7] = {
		.prod_prop = "prod_c_hbr",
		.max_link_bw = 2700,
		.link_rate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_2_70_GBPS,
	},
	[TEGRA_DC_SOR_LINK_SPEED_G5_4] = {
		.prod_prop = "prod_c_hbr2",
		.max_link_bw = 5400,
		.link_rate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_5_40_GBPS,
	},
	[TEGRA_DC_SOR_LINK_SPEED_G8_1] = {
		.prod_prop = "prod_c_hbr3",
		.max_link_bw = 8100,
		.link_rate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_8_10_GBPS,
	},
};

static const struct tegra_dc_dp_training_pattern training_pattern_table[] = {
	[TEGRA_DC_DP_TRAINING_PATTERN_DISABLE] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_NONE |
			NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_F,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_NOPATTERN,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_1] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP1 |
			NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_T,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING1,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_2] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP2 |
			NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_T,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING2,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_3] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP3 |
			NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_T,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING3,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_D102] = {
		.chan_coding = true,
		.scrambling = false,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_D102,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_SBLERRRATE] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_SBLERRRATE,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_PRBS7] = {
		.chan_coding = false,
		.scrambling = false,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_PRBS7,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_CSTM] = {
		.chan_coding = false,
		.scrambling = false,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_CSTM,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_HBR2_COMPLIANCE] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_HBR2_COMPLIANCE,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT1] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_CP2520_PAT1,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT3] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_CP2520_PAT3,
	},
	[TEGRA_DC_DP_TRAINING_PATTERN_4] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = NV_DPCD_TRAINING_PATTERN_SET_TPS_TP4 |
			NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_F,
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_TRAINING4,
	},
	/*
	 * On T194, the HW pattern generator sends the RD_RESET (running
	 * disparity reset) signal one clock cycle too early. This specifically
	 * affects TPS4 since TPS4 requires scrambling, so whatever symbol is
	 * sent during this early cycle will be random. As a result, the RD of
	 * the first symbol of TPS4 will be random as well.
	 *
	 * In order to WAR this issue, we will send a custom BS (blanking start)
	 * pattern when switching from TPS1 to TPS4 during equalization in order
	 * to control what the RD of the "early symbol" will be.
	 */
	[TEGRA_DC_DP_TRAINING_PATTERN_BS_CSTM] = {
		.chan_coding = true,
		.scrambling = true,
		.dpcd_val = 0, /* unused */
		.sor_reg_val = NV_SOR_DP_TPG_LANE0_PATTERN_CSTM,
	},
};

static struct of_device_id tegra_sor_pd[] = {
	{ .compatible = "nvidia,tegra210-sor-pd", },
	{ .compatible = "nvidia,tegra186-disa-pd", },
	{ .compatible = "nvidia,tegra194-disa-pd", },
	{},
};

static struct tegra_dc_mode min_mode = {
	.h_ref_to_sync = 0,
	.v_ref_to_sync = 1,
	.h_sync_width = 1,
	.v_sync_width = 1,
	.h_back_porch = 20,
	.v_back_porch = 0,
	.h_active = 16,
	.v_active = 16,
	.h_front_porch = 1,
	.v_front_porch = 2,
};

unsigned long
tegra_dc_sor_poll_register(struct tegra_dc_sor_data *sor,
				u32 reg, u32 mask, u32 exp_val,
				u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32 reg_val = 0;

	if (tegra_platform_is_vdk())
		return 0;

	do {
		reg_val = tegra_sor_readl(sor, reg);
		if ((reg_val & mask) == exp_val)
			return 0;       /* success */

		udelay(poll_interval_us);
	} while (time_after(timeout_jf, jiffies));

	dev_err(&sor->dc->ndev->dev,
		"sor_poll_register 0x%x: timeout\n", reg);

	return jiffies - timeout_jf + 1;
}

void tegra_sor_config_safe_clk(struct tegra_dc_sor_data *sor)
{
	struct clk *clk;
	int flag;

	/*
	 * For nvdisplay, sor->sor_clk was previously being used as the SOR
	 * reference clk instead of the orclk. In order to be consistent with
	 * the previous naming scheme, I'm using sor->ref_clk here to avoid
	 * breaking existing drivers. This needs to be cleaned up later.
	 */
	clk = (tegra_dc_is_nvdisplay()) ? sor->ref_clk : sor->sor_clk;
	flag = tegra_dc_is_clk_enabled(clk);

	if (sor->clk_type == TEGRA_SOR_SAFE_CLK)
		return;

	/*
	 * HW bug 1425607
	 * Disable clocks to avoid glitch when switching
	 * between safe clock and macro pll clock
	 */
	if (flag)
		tegra_sor_clk_disable(sor);

	if (tegra_platform_is_silicon())
		clk_set_parent(sor->sor_clk, sor->safe_clk);

	if (flag)
		tegra_sor_clk_enable(sor);

	sor->clk_type = TEGRA_SOR_SAFE_CLK;
}

void tegra_sor_config_dp_clk_t21x(struct tegra_dc_sor_data *sor)
{
	int flag = tegra_dc_is_clk_enabled(sor->sor_clk);
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(sor->dc);
	const int64_t pll_dp_rate = 270000000; /* fixed pll_dp@270MHz */

	if (sor->clk_type == TEGRA_SOR_MACRO_CLK)
		return;

	/*
	 * HW bug 1425607
	 * Disable clocks to avoid glitch when switching
	 * between safe clock and macro pll clock
	 *
	 * Select alternative -- DP -- DVFS table for SOR clock (if SOR clock
	 * has single DVFS table for all modes, nothing changes).
	 */
	if (flag)
		tegra_sor_clk_disable(sor);

#ifdef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_use_alt_freqs_on_clk(sor->sor_clk, true);
#endif

#ifdef CONFIG_TEGRA_CLK_FRAMEWORK
	if (tegra_platform_is_silicon())
		tegra_clk_cfg_ex(sor->sor_clk, TEGRA_CLK_SOR_CLK_SEL, 1);
#else
	if (sor->ctrl_num == 0)
	/* For DP on sor0, set pll_dp as parent of sor clock */
		clk_set_parent(sor->sor_clk, dp->parent_clk);
	else
	/* For DP on sor1, set sor1 pad output clk to be the parent */
		clk_set_parent(sor->sor_clk, sor->pad_clk);
#endif

	if (flag)
		tegra_sor_clk_enable(sor);

	sor->clk_type = TEGRA_SOR_MACRO_CLK;

	/*
	 * Set the pad_clk so that clock rate and DVFS are upto date.
	 * Divide link clock by 10 to get sor clock.
	 */
	clk_set_rate(sor->pad_clk, (pll_dp_rate * dp->link_cfg.link_bw / 10));

}

int tegra_dc_sor_crc_get(struct tegra_dc_sor_data *sor, u32 *crc)
{
	int ret = 0;
	u32 val;

	tegra_dc_io_start(sor->dc);
	tegra_sor_clk_enable(sor);

	val = tegra_sor_readl(sor, NV_SOR_CRCA);
	val = (val & NV_SOR_CRCA_VALID_DEFAULT_MASK) >> NV_SOR_CRCA_VALID_SHIFT;
	if (val != NV_SOR_CRCA_VALID_TRUE) {
		ret = -EINVAL;
		goto done;
	}

	*crc = tegra_sor_readl(sor, NV_SOR_CRCB);
	tegra_sor_writel(sor, NV_SOR_CRCA, NV_SOR_CRCA_VALID_RST);

done:
	tegra_sor_clk_disable(sor);
	tegra_dc_io_end(sor->dc);

	return ret;
}

u32 tegra_dc_sor_debugfs_get_crc(struct tegra_dc_sor_data *sor, int *timeout)
{
	struct tegra_dc *dc = sor->dc;
	u32 reg_val;

	tegra_dc_io_start(sor->dc);
	tegra_sor_clk_enable(sor);

	reg_val = tegra_sor_readl(sor, NV_SOR_CRC_CNTRL);
	reg_val &= NV_SOR_CRC_CNTRL_ARM_CRC_ENABLE_DEFAULT_MASK;
	if (reg_val == NV_SOR_CRC_CNTRL_ARM_CRC_ENABLE_NO) {
		pr_err("SOR CRC is DISABLED, aborting with CRC=0\n");
		goto exit;
	}
	if (tegra_dc_sor_poll_register(sor, NV_SOR_CRCA,
			NV_SOR_CRCA_VALID_DEFAULT_MASK,
			NV_SOR_CRCA_VALID_TRUE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"NV_SOR[%d]_CRCA_VALID_TRUE timeout\n", sor->ctrl_num);
		if (timeout)
			*timeout = 1;
		goto exit;
	}
	mutex_lock(&dc->lock);
	reg_val = tegra_sor_readl(sor, NV_SOR_CRCB);
	mutex_unlock(&dc->lock);

exit:
	tegra_sor_clk_disable(sor);
	tegra_dc_io_end(sor->dc);
	return reg_val;
}

void tegra_dc_sor_crc_en_dis(struct tegra_dc_sor_data *sor,
			     struct tegra_dc_ext_crc_sor_params params, bool en)
{
	u32 reg;

	tegra_dc_io_start(sor->dc);
	tegra_sor_clk_enable(sor);

	if (en) {
		reg = NV_SOR_CRCA_VALID_RST << NV_SOR_CRCA_VALID_SHIFT;
		tegra_sor_write_field(sor, NV_SOR_CRCA,
				      NV_SOR_CRCA_VALID_DEFAULT_MASK, reg);

		reg = params.stage << NV_SOR_TEST_CRC_SHIFT;
		tegra_sor_write_field(sor, NV_SOR_TEST,
				      NV_SOR_TEST_CRC_DEFAULT_MASK, reg);

		reg = params.data << NV_SOR_STATE1_ASY_CRCMODE_SHIFT;
		tegra_sor_write_field(sor, NV_SOR_STATE1,
				      NV_SOR_STATE1_ASY_CRCMODE_DEFAULT_MASK,
				      reg);

		reg = NV_SOR_STATE0_UPDATE_UPDATE << NV_SOR_STATE0_UPDATE_SHIFT;
		tegra_sor_write_field(sor, NV_SOR_STATE0,
				      NV_SOR_STATE0_UPDATE_DEFAULT_MASK, reg);
	}

	tegra_sor_readl(sor, NV_SOR_CRC_CNTRL);
	reg = en << NV_SOR_CRC_CNTRL_ARM_CRC_ENABLE_SHIFT;
	tegra_sor_write_field(sor, NV_SOR_CRC_CNTRL,
			      NV_SOR_CRC_CNTRL_ARM_CRC_ENABLE_DEFAULT_MASK,
			      reg);
	tegra_sor_readl(sor, NV_SOR_CRC_CNTRL);

	tegra_sor_clk_disable(sor);
	tegra_dc_io_end(sor->dc);
}

void tegra_dc_sor_toggle_crc(struct tegra_dc_sor_data *sor, u32 val)
{
	struct tegra_dc_ext_crc_sor_params params;

	params.stage = TEGRA_DC_EXT_CRC_SOR_STAGE_PRE_SERIALIZE;
	params.data = val & NV_SOR_STATE1_ASY_CRCMODE_DEFAULT_MASK;
	params.data >>= NV_SOR_STATE1_ASY_CRCMODE_SHIFT;

	tegra_dc_sor_crc_en_dis(sor, params, val & 0x1);
}

#ifdef CONFIG_DEBUG_FS
static int dbg_sor_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_sor_data *sor = s->private;
	int hdmi_dump = 0;

#define DUMP_REG(a) seq_printf(s, "%-32s  %03x  %08x\n",		\
		#a, a, tegra_sor_readl(sor, a));

	if (tegra_dc_is_t21x()) {
		if (!tegra_powergate_is_powered(sor->powergate_id)) {
			seq_puts(s, "SOR is powergated\n");
			return 0;
		}
	}

	tegra_dc_io_start(sor->dc);
	tegra_sor_clk_enable(sor);

	DUMP_REG(NV_SOR_SUPER_STATE0);
	DUMP_REG(NV_SOR_SUPER_STATE1);
	DUMP_REG(NV_SOR_STATE0);
	DUMP_REG(NV_SOR_STATE1);
	DUMP_REG(nv_sor_head_state0(0));
	DUMP_REG(nv_sor_head_state0(1));
	DUMP_REG(nv_sor_head_state1(0));
	DUMP_REG(nv_sor_head_state1(1));
	DUMP_REG(nv_sor_head_state2(0));
	DUMP_REG(nv_sor_head_state2(1));
	DUMP_REG(nv_sor_head_state3(0));
	DUMP_REG(nv_sor_head_state3(1));
	DUMP_REG(nv_sor_head_state4(0));
	DUMP_REG(nv_sor_head_state4(1));
	DUMP_REG(nv_sor_head_state5(0));
	DUMP_REG(nv_sor_head_state5(1));
	DUMP_REG(NV_SOR_CRC_CNTRL);
	DUMP_REG(NV_SOR_CLK_CNTRL);
	DUMP_REG(NV_SOR_CAP);
	DUMP_REG(NV_SOR_PWR);
	DUMP_REG(NV_SOR_TEST);
	DUMP_REG(nv_sor_pll0());
	DUMP_REG(nv_sor_pll1());
	DUMP_REG(nv_sor_pll2());
	DUMP_REG(nv_sor_pll3());
	if (tegra_dc_is_nvdisplay())
		DUMP_REG(nv_sor_pll4());
	if (tegra_dc_is_t19x())
		DUMP_REG(nv_sor_pll5());
	DUMP_REG(NV_SOR_CSTM);
	DUMP_REG(NV_SOR_LVDS);
	DUMP_REG(NV_SOR_CRCA);
	DUMP_REG(NV_SOR_CRCB);
	DUMP_REG(NV_SOR_SEQ_CTL);
	DUMP_REG(NV_SOR_LANE_SEQ_CTL);
	DUMP_REG(NV_SOR_SEQ_INST(0));
	DUMP_REG(NV_SOR_SEQ_INST(1));
	DUMP_REG(NV_SOR_SEQ_INST(2));
	DUMP_REG(NV_SOR_SEQ_INST(3));
	DUMP_REG(NV_SOR_SEQ_INST(4));
	DUMP_REG(NV_SOR_SEQ_INST(5));
	DUMP_REG(NV_SOR_SEQ_INST(6));
	DUMP_REG(NV_SOR_SEQ_INST(7));
	DUMP_REG(NV_SOR_SEQ_INST(8));
	DUMP_REG(NV_SOR_PWM_DIV);
	DUMP_REG(NV_SOR_PWM_CTL);
	DUMP_REG(NV_SOR_MSCHECK);
	DUMP_REG(NV_SOR_XBAR_CTRL);
	DUMP_REG(NV_SOR_XBAR_POL);
	DUMP_REG(NV_SOR_DP_LINKCTL(0));
	DUMP_REG(NV_SOR_DP_LINKCTL(1));
	DUMP_REG(NV_SOR_DC(0));
	DUMP_REG(NV_SOR_DC(1));
	DUMP_REG(NV_SOR_LANE_DRIVE_CURRENT(0));
	DUMP_REG(NV_SOR_PR(0));
	DUMP_REG(NV_SOR_LANE4_PREEMPHASIS(0));
	DUMP_REG(NV_SOR_POSTCURSOR(0));
	DUMP_REG(NV_SOR_DP_CONFIG(0));
	DUMP_REG(NV_SOR_DP_CONFIG(1));
	DUMP_REG(NV_SOR_DP_MN(0));
	DUMP_REG(NV_SOR_DP_MN(1));
	DUMP_REG(nv_sor_dp_padctl(0));
	DUMP_REG(nv_sor_dp_padctl(1));
	if (tegra_dc_is_nvdisplay())
		DUMP_REG(nv_sor_dp_padctl(2));
	DUMP_REG(NV_SOR_DP_DEBUG(0));
	DUMP_REG(NV_SOR_DP_DEBUG(1));
	DUMP_REG(NV_SOR_DP_SPARE(0));
	DUMP_REG(NV_SOR_DP_SPARE(1));
	DUMP_REG(NV_SOR_DP_TPG);
	DUMP_REG(NV_SOR_HDMI_CTRL);
	DUMP_REG(NV_SOR_HDMI2_CTRL);
	if (tegra_dc_is_nvdisplay()) {
		DUMP_REG(nv_sor_dp_misc1_override());
		DUMP_REG(nv_sor_dp_misc1_bit6());

		if (tegra_platform_is_vdk())
			DUMP_REG(NV_SOR_FPGA_HDMI_HEAD_SEL);
		hdmi_dump = 1; /* SOR and SOR1 have same registers */
	} else {
		hdmi_dump = sor->ctrl_num; /*SOR and SOR1 have diff registers*/
	}
	/* TODO: we should check if the feature is present
	 * and not the ctrl_num
	 */
	if (hdmi_dump) {
		DUMP_REG(NV_SOR_DP_AUDIO_CTRL);
		DUMP_REG(NV_SOR_DP_AUDIO_HBLANK_SYMBOLS);
		DUMP_REG(NV_SOR_DP_AUDIO_VBLANK_SYMBOLS);
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_HEADER);
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(0));
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(1));
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(2));
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(3));
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(4));
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(5));
		DUMP_REG(NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(6));

		DUMP_REG(NV_SOR_DP_OUTPUT_CHANNEL_STATUS1);
		DUMP_REG(NV_SOR_DP_OUTPUT_CHANNEL_STATUS2);

		DUMP_REG(NV_SOR_HDMI_AUDIO_N);
		DUMP_REG(NV_SOR_HDMI2_CTRL);

		DUMP_REG(NV_SOR_AUDIO_CTRL);
		DUMP_REG(NV_SOR_AUDIO_DEBUG);
		DUMP_REG(NV_SOR_AUDIO_NVAL_0320);
		DUMP_REG(NV_SOR_AUDIO_NVAL_0441);
		DUMP_REG(NV_SOR_AUDIO_NVAL_0882);
		DUMP_REG(NV_SOR_AUDIO_NVAL_1764);
		DUMP_REG(NV_SOR_AUDIO_NVAL_0480);
		DUMP_REG(NV_SOR_AUDIO_NVAL_0960);
		DUMP_REG(NV_SOR_AUDIO_NVAL_1920);

		DUMP_REG(NV_SOR_AUDIO_AVAL_0320);
		DUMP_REG(NV_SOR_AUDIO_AVAL_0441);
		DUMP_REG(NV_SOR_AUDIO_AVAL_0882);
		DUMP_REG(NV_SOR_AUDIO_AVAL_1764);
		DUMP_REG(NV_SOR_AUDIO_AVAL_0480);
		DUMP_REG(NV_SOR_AUDIO_AVAL_0960);
		DUMP_REG(NV_SOR_AUDIO_AVAL_1920);

		DUMP_REG(NV_SOR_DP_AUDIO_CRC);
		DUMP_REG(NV_SOR_DP_AUDIO_TIMESTAMP_0320);
		DUMP_REG(NV_SOR_DP_AUDIO_TIMESTAMP_0441);
		DUMP_REG(NV_SOR_DP_AUDIO_TIMESTAMP_0882);
		DUMP_REG(NV_SOR_DP_AUDIO_TIMESTAMP_1764);
		DUMP_REG(NV_SOR_DP_AUDIO_TIMESTAMP_0480);
		DUMP_REG(NV_SOR_DP_AUDIO_TIMESTAMP_0960);
		DUMP_REG(NV_SOR_DP_AUDIO_TIMESTAMP_1920);

		DUMP_REG(NV_SOR_HDMI_GENERIC_CTRL);
		DUMP_REG(NV_SOR_HDMI_GENERIC_HEADER);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK0_LOW);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK0_HIGH);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK1_LOW);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK1_HIGH);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK2_LOW);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK2_HIGH);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK3_LOW);
		DUMP_REG(NV_SOR_HDMI_GENERIC_SUBPACK3_HIGH);

		DUMP_REG(NV_SOR_HDMI_AVI_INFOFRAME_CTRL);
		DUMP_REG(NV_SOR_HDMI_AVI_INFOFRAME_HEADER);
		DUMP_REG(NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK0_LOW);
		DUMP_REG(NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK0_HIGH);
		DUMP_REG(NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK1_LOW_0);
		DUMP_REG(NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK1_HIGH_0);
	}

	tegra_sor_clk_disable(sor);
	tegra_dc_io_end(sor->dc);

	return 0;
}

static int dbg_sor_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_sor_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open		= dbg_sor_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int sor_crc_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_sor_data *sor = s->private;
	u32 reg_val;
	int timeout = 0;

	reg_val = tegra_dc_sor_debugfs_get_crc(sor, &timeout);

	if (!timeout)
		seq_printf(s, "NV_SOR[%x]_CRCB = 0x%08x\n",
			sor->ctrl_num, reg_val);

	return 0;
}

static int sor_crc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sor_crc_show, inode->i_private);
}

static ssize_t sor_crc_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_sor_data *sor = s->private;
	u32    data;

	/* autodetect radix */
	if (kstrtouint_from_user(user_buf, count, 0, &data) < 0)
		return -EINVAL;

	/* at this point:
	 * data[0:0] = 1|0: enable|disable CRC
	 * data[5:4] contains ASY_CRCMODE */

	tegra_dc_sor_toggle_crc(sor, data);

	return count;
}

static const struct file_operations crc_fops = {
	.open		= sor_crc_open,
	.read		= seq_read,
	.write		= sor_crc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_hw_index_show(struct seq_file *m, void *unused)
{
	struct tegra_dc_sor_data *sor = m->private;

	if (WARN_ON(!sor))
		return -EINVAL;

	seq_printf(m, "Hardware index: %d\n", sor->ctrl_num);

	return 0;
}

static int dbg_hw_index_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_hw_index_show, inode->i_private);
}

static const struct file_operations dbg_hw_index_ops = {
	.open = dbg_hw_index_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void tegra_dc_sor_debug_create(struct tegra_dc_sor_data *sor,
	const char *res_name)
{
	struct dentry *retval;
	char sor_path[16];

	BUG_ON(!res_name);

	snprintf(sor_path, sizeof(sor_path), "tegra_%s", res_name ? : "sor");
	sor->debugdir = debugfs_create_dir(sor_path, NULL);
	if (!sor->debugdir)
		return;

	retval = debugfs_create_file("regs", 0444, sor->debugdir, sor,
		&dbg_fops);
	if (!retval)
		goto free_out;

	retval = debugfs_create_file("crc", 0644, sor->debugdir,
		sor, &crc_fops);
	if (!retval)
		goto free_out;

	retval = debugfs_create_file("hw_index", 0444, sor->debugdir,
				sor, &dbg_hw_index_ops);
	if (!retval)
		goto free_out;

	return;
free_out:
	debugfs_remove_recursive(sor->debugdir);
	sor->debugdir = NULL;
	return;
}
EXPORT_SYMBOL(tegra_dc_sor_debug_create);

static void tegra_dc_sor_debug_destroy(struct tegra_dc_sor_data *sor)
{
	debugfs_remove_recursive(sor->debugdir);
	sor->debugdir = NULL;
	debugfs_remove(sor->dc->sor_link);
}
#else
static inline void tegra_dc_sor_debug_create(struct tegra_dc_sor_data *sor,
	const char *res_name)
{ }
static inline void tegra_dc_sor_debug_destroy(struct tegra_dc_sor_data *sor)
{ }
#endif

static void tegra_sor_fpga_settings(struct tegra_dc *dc,
				struct tegra_dc_sor_data *sor)
{
	u32 mode_sel = NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_MODE_FIELD |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_OUT_EN_FIELD;
	u32 head_sel = NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_MODE_HDMI |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_OUT_EN_ENABLE;

	/* continue for system fpga and HDMI */
	if ((!tegra_platform_is_vdk()) || (dc->out->type != TEGRA_DC_OUT_HDMI))
		return;

	if (dc->ndev->id == 0) {/* HEAD 0 */
		mode_sel =
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD0_MODE_FIELD |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD0_OUT_EN_FIELD;

		head_sel =
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD0_MODE_HDMI |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD0_OUT_EN_ENABLE;

	} else if (dc->ndev->id == 1) {/* HEAD 1 */
		mode_sel =
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_MODE_FIELD |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_OUT_EN_FIELD;

		head_sel =
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_MODE_HDMI |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD1_OUT_EN_ENABLE;

	} else if (dc->ndev->id == 2) {/* HEAD 2 */
		mode_sel =
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD2_MODE_FIELD |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD2_OUT_EN_FIELD;

		head_sel =
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD2_MODE_HDMI |
			NV_SOR_FPGA_HDMI_HEAD_SEL_FPGA_HEAD2_OUT_EN_ENABLE;

	}
	tegra_sor_write_field(sor, NV_SOR_FPGA_HDMI_HEAD_SEL,
				mode_sel, head_sel);

	return;
}

struct tegra_dc_sor_data *tegra_dc_sor_init(struct tegra_dc *dc,
				const struct tegra_dc_dp_link_config *cfg)
{
	u32 temp;
	int err, i;
	char res_name[CHAR_BUF_SIZE_MAX] = {0};
	char io_pinctrl_en_name[CHAR_BUF_SIZE_MAX] = {0};
	char io_pinctrl_dis_name[CHAR_BUF_SIZE_MAX] = {0};
	struct clk *sor_clk = NULL;
	struct clk *safe_clk = NULL;
	struct clk *pad_clk = NULL;
	struct clk *ref_clk = NULL;
	struct tegra_dc_sor_data *sor;
	struct tegra_dc_sor_info *sor_cap;
	struct device_node *sor_np;

	if (!dc) {
		pr_err("%s: dc pointer cannot be NULL\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	sor_np = tegra_dc_get_conn_np(dc);
	if (!sor_np) {
		dev_err(&dc->ndev->dev, "%s: error getting connector np\n",
			__func__);
		err = -ENODEV;
		goto err_allocate;
	}

	sor = devm_kzalloc(&dc->ndev->dev, sizeof(*sor), GFP_KERNEL);
	if (!sor) {
		err = -ENOMEM;
		goto err_allocate;
	}

	sor->link_speeds = link_speed_table;
	sor->num_link_speeds = ARRAY_SIZE(link_speed_table);

	sor->training_patterns = training_pattern_table;
	sor->num_training_patterns = ARRAY_SIZE(training_pattern_table);

	if (!of_property_read_u32(sor_np, "nvidia,sor-ctrlnum", &temp)) {
		sor->ctrl_num = (unsigned long)temp;
	} else {
		dev_err(&dc->ndev->dev, "mandatory property %s for %s not found\n",
				"nvidia,sor-ctrlnum",
				of_node_full_name(sor_np));
		err = -ENOENT;
		goto err_free_sor;
	}

	snprintf(res_name, CHAR_BUF_SIZE_MAX, "sor%d", sor->ctrl_num);

	sor->base = of_iomap(sor_np, 0);
	if (!sor->base) {
		dev_err(&dc->ndev->dev, "%s: %s registers can't be mapped\n",
			__func__, res_name);
		err = IS_ERR(sor->base) ? PTR_ERR(sor->base) : -ENOENT;
		goto err_free_sor;
	}

	sor_clk = tegra_disp_of_clk_get_by_name(sor_np, res_name);
	if (IS_ERR_OR_NULL(sor_clk)) {
		dev_err(&dc->ndev->dev, "%s: can't get clock %s\n",
				__func__, res_name);
		err = IS_ERR(sor_clk) ? PTR_ERR(sor_clk) : -ENOENT;
		goto err_iounmap_reg;
	}

	safe_clk = tegra_disp_of_clk_get_by_name(sor_np, "sor_safe");
	if (IS_ERR_OR_NULL(safe_clk)) {
		dev_err(&dc->ndev->dev, "sor: can't get safe clock\n");
		err = IS_ERR(safe_clk) ? PTR_ERR(safe_clk) : -ENOENT;
		goto err_safe;
	}

	if (!(tegra_dc_is_t21x() && sor->ctrl_num == 0)) {
		snprintf(res_name, CHAR_BUF_SIZE_MAX, "sor%d_pad_clkout",
			sor->ctrl_num);
		pad_clk = tegra_disp_of_clk_get_by_name(sor_np, res_name);
		if (IS_ERR_OR_NULL(pad_clk)) {
			dev_err(&dc->ndev->dev, "sor: can't get %s\n",
				res_name);
			err = IS_ERR(pad_clk) ? PTR_ERR(pad_clk) : -ENOENT;
			goto err_pad;
		}

		snprintf(res_name, CHAR_BUF_SIZE_MAX, "sor%d_ref",
			sor->ctrl_num);
		ref_clk = tegra_disp_of_clk_get_by_name(sor_np, res_name);
		if (IS_ERR_OR_NULL(ref_clk)) {
			dev_err(&dc->ndev->dev, "sor: can't get %s\n",
				res_name);
			err = IS_ERR(ref_clk) ? PTR_ERR(ref_clk) : -ENOENT;
			goto err_ref;
		}

		/* change res_name back to sor%d */
		snprintf(res_name, CHAR_BUF_SIZE_MAX, "sor%d", sor->ctrl_num);
	}

	err = tegra_get_sor_reset_ctrl(sor, sor_np, res_name);
	if (err) {
		dev_err(&dc->ndev->dev, "sor%d: can't get reset control\n",
				sor->ctrl_num);
		goto err_rst;
	}

	for (i = 0; i < sizeof(sor->xbar_ctrl)/sizeof(u32); i++)
		sor->xbar_ctrl[i] = i;

	if (tegra_dc_is_t21x() && (sor->ctrl_num == 1)) { /* todo: fix this */
		snprintf(io_pinctrl_en_name, CHAR_BUF_SIZE_MAX,
			 "hdmi-dpd-enable");
		snprintf(io_pinctrl_dis_name, CHAR_BUF_SIZE_MAX,
			 "hdmi-dpd-disable");
	} else {
		snprintf(io_pinctrl_en_name, CHAR_BUF_SIZE_MAX,
			 "hdmi-dp%d-dpd-enable", sor->ctrl_num);
		snprintf(io_pinctrl_dis_name, CHAR_BUF_SIZE_MAX,
			 "hdmi-dp%d-dpd-disable", sor->ctrl_num);
	}

	sor->pinctrl_sor = devm_pinctrl_get(&dc->ndev->dev);
	if (IS_ERR_OR_NULL(sor->pinctrl_sor)) {
		dev_err(&dc->ndev->dev, "pinctrl get fail: %ld\n",
			PTR_ERR(sor->pinctrl_sor));
		sor->pinctrl_sor = NULL;
		goto bypass_pads;
	}

	if (sor->pinctrl_sor) {
		sor->dpd_enable = pinctrl_lookup_state(sor->pinctrl_sor,
						       io_pinctrl_en_name);
		if (IS_ERR_OR_NULL(sor->dpd_enable)) {
			dev_err(&dc->ndev->dev, "dpd enable lookup fail:%ld\n",
				PTR_ERR(sor->dpd_enable));
			sor->dpd_enable = NULL;
			goto bypass_pads;
		}

		sor->dpd_disable = pinctrl_lookup_state(sor->pinctrl_sor,
							io_pinctrl_dis_name);
		if (IS_ERR_OR_NULL(sor->dpd_disable)) {
			dev_err(&dc->ndev->dev, "dpd disable lookup fail:%ld\n",
				PTR_ERR(sor->dpd_disable));
			sor->dpd_disable = NULL;
		}
	}

bypass_pads:
	if (of_property_read_u32_array(sor_np, "nvidia,xbar-ctrl",
		sor->xbar_ctrl, sizeof(sor->xbar_ctrl)/sizeof(u32)))
		dev_err(&dc->ndev->dev, "%s: error reading nvidia,xbar-ctrl\n",
					__func__);

	if (of_property_read_bool(sor_np, "nvidia,sor-audio-not-supported"))
		sor->audio_support = false;
	else
		sor->audio_support = true;

	sor_cap = tegra_dc_get_sor_cap();
	if (IS_ERR_OR_NULL(sor_cap)) {
		dev_info(&dc->ndev->dev, "sor: can't get sor cap.\n");
		sor->hdcp_support = false;
	} else {
		sor->hdcp_support = sor_cap[sor->ctrl_num].hdcp_supported;
	}

	if (tegra_dc_is_nvdisplay()) {
		sor->win_state_arr = devm_kzalloc(&dc->ndev->dev,
					tegra_dc_get_numof_dispwindows() *
					sizeof(*sor->win_state_arr),
					GFP_KERNEL);
		if (!sor->win_state_arr) {
			err = -ENOMEM;
			goto err_rst;
		}
	}

	sor->dc = dc;
	sor->np = sor_np;
	sor->sor_clk = sor_clk;
	sor->safe_clk = safe_clk;
	sor->pad_clk = pad_clk;
	sor->ref_clk = ref_clk;
	sor->link_cfg = cfg;
	sor->portnum = 0;
	sor->powergate_id = tegra_pd_get_powergate_id(tegra_sor_pd);
	sor->sor_state = SOR_DETACHED;

	tegra_dc_sor_debug_create(sor, res_name);

	if (tegra_dc_is_nvdisplay())
		tegra_sor_fpga_settings(dc, sor);
	init_rwsem(&sor->reset_lock);

	return sor;

err_rst: __maybe_unused
	clk_put(ref_clk);
err_ref: __maybe_unused
	clk_put(pad_clk);
err_pad: __maybe_unused
	clk_put(safe_clk);
err_safe: __maybe_unused
	clk_put(sor_clk);
err_iounmap_reg:
	iounmap(sor->base);
err_free_sor:
	devm_kfree(&dc->ndev->dev, sor);
err_allocate:
	return ERR_PTR(err);
}

int tegra_dc_sor_set_power_state(struct tegra_dc_sor_data *sor, int pu_pd)
{
	u32 reg_val;
	u32 orig_val;

	orig_val = tegra_sor_readl(sor, NV_SOR_PWR);

	reg_val = pu_pd ? NV_SOR_PWR_NORMAL_STATE_PU :
		NV_SOR_PWR_NORMAL_STATE_PD; /* normal state only */

	if (reg_val == orig_val)
		return 0;	/* No update needed */

	reg_val |= NV_SOR_PWR_SETTING_NEW_TRIGGER;
	tegra_sor_writel(sor, NV_SOR_PWR, reg_val);

	/* Poll to confirm it is done */
	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWR,
			NV_SOR_PWR_SETTING_NEW_DEFAULT_MASK,
			NV_SOR_PWR_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_err(&sor->dc->ndev->dev,
			"dc timeout waiting for SOR_PWR = NEW_DONE\n");
		return -EFAULT;
	}
	return 0;
}

void tegra_dc_sor_destroy(struct tegra_dc_sor_data *sor)
{
	struct device *dev;

	if (!sor) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}
	dev = &sor->dc->ndev->dev;

	clk_put(sor->ref_clk);
	clk_put(sor->pad_clk);
	clk_put(sor->safe_clk);
	clk_put(sor->sor_clk);

	tegra_dc_sor_debug_destroy(sor);

	iounmap(sor->base);

	devm_pinctrl_put(sor->pinctrl_sor);
	sor->dpd_enable = NULL;
	sor->dpd_disable = NULL;

	if (tegra_dc_is_nvdisplay())
		devm_kfree(dev, sor->win_state_arr);
	devm_kfree(dev, sor);
}

void tegra_sor_tpg(struct tegra_dc_sor_data *sor, u32 tp, u32 total_lanes)
{
	bool chan_coding = sor->training_patterns[tp].chan_coding;
	bool scrambling = sor->training_patterns[tp].scrambling;
	u32 tps_sor_val = sor->training_patterns[tp].sor_reg_val;
	u32 val = 0; /* The value written to the reg gets constructed here */
	unsigned int lane;

	for (lane = 0; lane < total_lanes; lane++) {
		u32 tp_shift = NV_SOR_DP_TPG_LANE1_PATTERN_SHIFT * lane;

		val |= tps_sor_val << tp_shift;
		val |= chan_coding << (tp_shift +
				       NV_SOR_DP_TPG_LANE0_CHANNELCODING_SHIFT);
		val |= scrambling << (tp_shift +
				      NV_SOR_DP_TPG_LANE0_SCRAMBLEREN_SHIFT);
	}

	tegra_sor_writel(sor, NV_SOR_DP_TPG, val);
}

void tegra_sor_port_enable(struct tegra_dc_sor_data *sor, bool enb)
{
	tegra_sor_write_field(sor, NV_SOR_DP_LINKCTL(sor->portnum),
			NV_SOR_DP_LINKCTL_ENABLE_YES,
			(enb ? NV_SOR_DP_LINKCTL_ENABLE_YES :
			NV_SOR_DP_LINKCTL_ENABLE_NO));
}

static int tegra_dc_sor_enable_lane_sequencer(struct tegra_dc_sor_data *sor,
						bool pu)
{
	u32 reg_val;

	/* SOR lane sequencer */
	reg_val = NV_SOR_LANE_SEQ_CTL_SETTING_NEW_TRIGGER |
		NV_SOR_LANE_SEQ_CTL_SEQUENCE_DOWN |
		(15 << NV_SOR_LANE_SEQ_CTL_DELAY_SHIFT);
	reg_val |= pu ? NV_SOR_LANE_SEQ_CTL_NEW_POWER_STATE_PU :
			NV_SOR_LANE_SEQ_CTL_NEW_POWER_STATE_PD;

	if (tegra_dc_sor_poll_register(sor, NV_SOR_LANE_SEQ_CTL,
			NV_SOR_LANE_SEQ_CTL_SEQ_STATE_BUSY,
			NV_SOR_LANE_SEQ_CTL_SEQ_STATE_IDLE,
			100, TEGRA_SOR_SEQ_BUSY_TIMEOUT_MS)) {
		dev_dbg(&sor->dc->ndev->dev,
			"dp: timeout, sor lane sequencer busy\n");
		return -EFAULT;
	}

	tegra_sor_writel(sor, NV_SOR_LANE_SEQ_CTL, reg_val);
	if (tegra_dc_sor_poll_register(sor, NV_SOR_LANE_SEQ_CTL,
			NV_SOR_LANE_SEQ_CTL_SETTING_MASK,
			NV_SOR_LANE_SEQ_CTL_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_dbg(&sor->dc->ndev->dev,
			"dp: timeout, SOR lane sequencer power up/down\n");
		return -EFAULT;
	}

	return 0;
}

static u32 tegra_sor_get_pd_tx_bitmap(struct tegra_dc_sor_data *sor,
						u32 lane_count)
{
	int i;
	u32 val = 0;

	for (i = 0; i < lane_count; i++) {
		u32 index = sor->xbar_ctrl[i];

		switch (index) {
		case 0:
			val |= NV_SOR_DP_PADCTL_PD_TXD_0_NO;
			break;
		case 1:
			val |= NV_SOR_DP_PADCTL_PD_TXD_1_NO;
			break;
		case 2:
			val |= NV_SOR_DP_PADCTL_PD_TXD_2_NO;
			break;
		case 3:
			val |= NV_SOR_DP_PADCTL_PD_TXD_3_NO;
			break;
		default:
			dev_err(&sor->dc->ndev->dev,
				"dp: incorrect lane cnt\n");
		}
	}

	return val;
}

int tegra_sor_power_lanes(struct tegra_dc_sor_data *sor,
					u32 lane_count, bool pu)
{
	u32 val = 0;

	if (pu)
		val = tegra_sor_get_pd_tx_bitmap(sor, lane_count);

	tegra_sor_write_field(sor, nv_sor_dp_padctl(sor->portnum),
				NV_SOR_DP_PADCTL_PD_TXD_MASK, val);

	if (pu)
		tegra_dc_sor_set_lane_count(sor, lane_count);

	return tegra_dc_sor_enable_lane_sequencer(sor, pu);
}

/* power on/off pad calibration logic */
void tegra_sor_pad_cal_power(struct tegra_dc_sor_data *sor,
					bool power_up)
{
	u32 val = power_up ? NV_SOR_DP_PADCTL_PAD_CAL_PD_POWERUP :
			NV_SOR_DP_PADCTL_PAD_CAL_PD_POWERDOWN;

	/* !!TODO: need to enable panel power through GPIO operations */
	/* Check bug 790854 for HW progress */

	tegra_sor_write_field(sor, nv_sor_dp_padctl(sor->portnum),
				NV_SOR_DP_PADCTL_PAD_CAL_PD_POWERDOWN, val);
}

void tegra_dc_sor_termination_cal(struct tegra_dc_sor_data *sor)
{
	u32 nv_sor_pll1_reg = nv_sor_pll1();
	u32 termadj;
	u32 cur_try;
	u32 reg_val;

	termadj = cur_try = 0x8;

	tegra_sor_write_field(sor, nv_sor_pll1_reg,
		NV_SOR_PLL1_TMDS_TERMADJ_DEFAULT_MASK,
		termadj << NV_SOR_PLL1_TMDS_TERMADJ_SHIFT);

	while (cur_try) {
		/* binary search the right value */
		usleep_range(100, 200);
		reg_val = tegra_sor_readl(sor, nv_sor_pll1_reg);

		if (reg_val & NV_SOR_PLL1_TERM_COMPOUT_HIGH)
			termadj -= cur_try;
		cur_try >>= 1;
		termadj += cur_try;

		tegra_sor_write_field(sor, nv_sor_pll1_reg,
			NV_SOR_PLL1_TMDS_TERMADJ_DEFAULT_MASK,
			termadj << NV_SOR_PLL1_TMDS_TERMADJ_SHIFT);
	}
}

static void tegra_dc_sor_config_pwm(struct tegra_dc_sor_data *sor, u32 pwm_div,
	u32 pwm_dutycycle)
{
	tegra_sor_writel(sor, NV_SOR_PWM_DIV, pwm_div);
	tegra_sor_writel(sor, NV_SOR_PWM_CTL,
		(pwm_dutycycle & NV_SOR_PWM_CTL_DUTY_CYCLE_MASK) |
		NV_SOR_PWM_CTL_SETTING_NEW_TRIGGER);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWM_CTL,
			NV_SOR_PWM_CTL_SETTING_NEW_SHIFT,
			NV_SOR_PWM_CTL_SETTING_NEW_DONE,
			100, TEGRA_SOR_TIMEOUT_MS)) {
		dev_dbg(&sor->dc->ndev->dev,
			"dp: timeout while waiting for SOR PWM setting\n");
	}
}

static inline void tegra_dc_sor_super_update(struct tegra_dc_sor_data *sor)
{
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE0, 0);
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE0, 1);
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE0, 0);
}

static inline void tegra_dc_sor_update(struct tegra_dc_sor_data *sor)
{
	tegra_sor_writel(sor, NV_SOR_STATE0, 0);
	tegra_sor_writel(sor, NV_SOR_STATE0, 1);
	tegra_sor_writel(sor, NV_SOR_STATE0, 0);
}

static void tegra_dc_sor_io_set_dpd(struct tegra_dc_sor_data *sor, bool up)
{
	int ret = 0;

	if (tegra_platform_is_vdk())
		return;

	if (!sor->pinctrl_sor)
		return;

	if (up) {
		if (sor->dpd_disable)
			ret = pinctrl_select_state(sor->pinctrl_sor,
						   sor->dpd_disable);
	} else {
		if (sor->dpd_enable)
			ret = pinctrl_select_state(sor->pinctrl_sor,
						   sor->dpd_enable);
	}
	if (ret < 0)
		dev_err(&sor->dc->ndev->dev, "io pad power %s fail:%d\n",
			up ? "disable" : "enable", ret);
}

/* hdmi uses sor sequencer for pad power up */
void tegra_sor_hdmi_pad_power_up(struct tegra_dc_sor_data *sor)
{
	u32 nv_sor_pll0_reg = nv_sor_pll0();
	u32 nv_sor_pll1_reg = nv_sor_pll1();
	u32 nv_sor_pll2_reg = nv_sor_pll2();
	int ret = 0;

	/* seamless */
	if (sor->dc->initialized)
		return;

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX9_LVDSEN_OVERRIDE,
				NV_SOR_PLL2_AUX9_LVDSEN_OVERRIDE);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX0_MASK,
				NV_SOR_PLL2_AUX0_SEQ_PLL_PULLDOWN_OVERRIDE);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_CLKGEN_MODE_MASK,
				NV_SOR_PLL2_CLKGEN_MODE_DP_TMDS);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX2_MASK,
				NV_SOR_PLL2_AUX2_OVERRIDE_POWERDOWN);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX1_SEQ_MASK,
				NV_SOR_PLL2_AUX1_SEQ_PLLCAPPD_OVERRIDE);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK,
				NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_ENABLE);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX7_PORT_POWERDOWN_MASK,
				NV_SOR_PLL2_AUX7_PORT_POWERDOWN_ENABLE);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_MASK,
				NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_ENABLE);
	tegra_sor_write_field(sor, nv_sor_pll0_reg, NV_SOR_PLL0_PWR_MASK,
						NV_SOR_PLL0_PWR_OFF);
	tegra_sor_write_field(sor, nv_sor_pll0_reg, NV_SOR_PLL0_VCOPD_MASK,
						NV_SOR_PLL0_VCOPD_ASSERT);
	tegra_sor_pad_cal_power(sor, false);
	usleep_range(20, 70);

	if (sor->dpd_disable) {
		ret = pinctrl_select_state(sor->pinctrl_sor, sor->dpd_disable);
		if (ret < 0)
			dev_err(&sor->dc->ndev->dev,
				"io pad power-up fail:%d\n", ret);
	}
	usleep_range(20, 70);

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK,
				NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_DISABLE);
	usleep_range(50, 100);

	tegra_sor_write_field(sor, nv_sor_pll0_reg, NV_SOR_PLL0_PWR_MASK,
						NV_SOR_PLL0_PWR_ON);
	tegra_sor_write_field(sor, nv_sor_pll0_reg, NV_SOR_PLL0_VCOPD_MASK,
						NV_SOR_PLL0_VCOPD_RESCIND);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_MASK,
				NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_DISABLE);
	usleep_range(250, 300);

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX7_PORT_POWERDOWN_MASK,
				NV_SOR_PLL2_AUX7_PORT_POWERDOWN_DISABLE);

	/*
	 * TERM_ENABLE is disabled at the end of rterm calibration. Re-enable it
	 * here.
	 */
	tegra_sor_write_field(sor, nv_sor_pll1_reg,
				NV_SOR_PLL1_TMDS_TERM_ENABLE,
				NV_SOR_PLL1_TMDS_TERM_ENABLE);
	usleep_range(10, 20);
}

void tegra_sor_hdmi_pad_power_down(struct tegra_dc_sor_data *sor)
{
	u32 nv_sor_pll0_reg = nv_sor_pll0();
	u32 nv_sor_pll2_reg = nv_sor_pll2();
	int ret = 0;

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX7_PORT_POWERDOWN_MASK,
				NV_SOR_PLL2_AUX7_PORT_POWERDOWN_ENABLE);
	usleep_range(25, 30);

	tegra_sor_write_field(sor, nv_sor_pll0_reg, NV_SOR_PLL0_PWR_MASK |
				NV_SOR_PLL0_VCOPD_MASK, NV_SOR_PLL0_PWR_OFF |
				NV_SOR_PLL0_VCOPD_ASSERT);
	tegra_sor_write_field(sor, nv_sor_pll2_reg, NV_SOR_PLL2_AUX1_SEQ_MASK |
				NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_MASK,
				NV_SOR_PLL2_AUX1_SEQ_PLLCAPPD_OVERRIDE |
				NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_ENABLE);
	usleep_range(25, 30);

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
				NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK,
				NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_ENABLE);
	tegra_sor_pad_cal_power(sor, false);

	if (sor->dpd_enable) {
		ret = pinctrl_select_state(sor->pinctrl_sor, sor->dpd_enable);
		if (ret < 0)
			dev_err(&sor->dc->ndev->dev,
				"io pad power-down fail:%d\n", ret);
	}
	usleep_range(20, 70);
}

/* The SOR power sequencer does not work for t124 so SW has to
   go through the power sequence manually */
/* Power up steps from spec: */
/* STEP	PDPORT	PDPLL	PDBG	PLLVCOD	PLLCAPD	E_DPD	PDCAL */
/* 1	1	1	1	1	1	1	1 */
/* 2	1	1	1	1	1	0	1 */
/* 3	1	1	0	1	1	0	1 */
/* 4	1	0	0	0	0	0	1 */
/* 5	0	0	0	0	0	0	1 */
static void tegra_sor_dp_pad_power_up(struct tegra_dc_sor_data *sor)
{
	u32 nv_sor_pll0_reg = nv_sor_pll0();
	u32 nv_sor_pll1_reg = nv_sor_pll1();
	u32 nv_sor_pll2_reg = nv_sor_pll2();

	if (sor->power_is_up)
		return;

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX2_MASK,
		NV_SOR_PLL2_AUX2_OVERRIDE_POWERDOWN);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX1_SEQ_MASK,
		NV_SOR_PLL2_AUX1_SEQ_PLLCAPPD_OVERRIDE);

	/* step 1 */
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX7_PORT_POWERDOWN_MASK | /* PDPORT */
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK | /* PDBG */
		NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_MASK, /* PLLCAPD */
		NV_SOR_PLL2_AUX7_PORT_POWERDOWN_ENABLE |
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_ENABLE |
		NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_ENABLE);
	tegra_sor_write_field(sor, nv_sor_pll0_reg,
		NV_SOR_PLL0_PWR_MASK | /* PDPLL */
		NV_SOR_PLL0_VCOPD_MASK, /* PLLVCOPD */
		NV_SOR_PLL0_PWR_OFF |
		NV_SOR_PLL0_VCOPD_ASSERT);
	tegra_sor_pad_cal_power(sor, false); /* PDCAL */

	/* step 2 */
	tegra_dc_sor_io_set_dpd(sor, true);
	usleep_range(5, 100);	/* sleep > 5us */

	/* step 3 */
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_DISABLE);
	usleep_range(100, 150);

	/* step 4 */
	tegra_sor_write_field(sor, nv_sor_pll0_reg,
		NV_SOR_PLL0_PWR_MASK | /* PDPLL */
		NV_SOR_PLL0_VCOPD_MASK, /* PLLVCOPD */
		NV_SOR_PLL0_PWR_ON | NV_SOR_PLL0_VCOPD_RESCIND);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_MASK, /* PLLCAPD */
		NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_DISABLE);
	usleep_range(200, 1000);

	/* step 5 */
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX7_PORT_POWERDOWN_MASK, /* PDPORT */
		NV_SOR_PLL2_AUX7_PORT_POWERDOWN_DISABLE);

	/*
	 * TERM_ENABLE is disabled at the end of rterm calibration. Re-enable it
	 * here.
	 */
	tegra_sor_write_field(sor, nv_sor_pll1_reg,
				NV_SOR_PLL1_TMDS_TERM_ENABLE,
				NV_SOR_PLL1_TMDS_TERM_ENABLE);
	usleep_range(10, 20);

	sor->power_is_up = true;
}

/* Powerdown steps from the spec: */
/* STEP	PDPORT	PDPLL	PDBG	PLLVCOD	PLLCAPD	E_DPD	PDCAL */
/* 1	0	0	0	0	0	0	1 */
/* 2	1	0	0	0	0	0	1 */
/* 3	1	1	0	1	1	0	1 */
/* 4	1	1	1	1	1	0	1 */
/* 5	1	1	1	1	1	1	1 */
static void tegra_sor_dp_pad_power_down(struct tegra_dc_sor_data *sor)
{
	u32 nv_sor_pll0_reg = nv_sor_pll0();
	u32 nv_sor_pll2_reg = nv_sor_pll2();

	if (!sor->power_is_up)
		return;

	/* step 1 -- not necessary */

	/* step 2 */
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX7_PORT_POWERDOWN_MASK, /* PDPORT */
		NV_SOR_PLL2_AUX7_PORT_POWERDOWN_ENABLE);
	usleep_range(25, 30);

	/* step 3 */
	tegra_sor_write_field(sor, nv_sor_pll0_reg,
		NV_SOR_PLL0_PWR_MASK | /* PDPLL */
		NV_SOR_PLL0_VCOPD_MASK, /* PLLVCOPD */
		NV_SOR_PLL0_PWR_OFF | NV_SOR_PLL0_VCOPD_ASSERT);
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX1_SEQ_MASK |
		NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_MASK, /* PLLCAPD */
		NV_SOR_PLL2_AUX1_SEQ_PLLCAPPD_OVERRIDE |
		NV_SOR_PLL2_AUX8_SEQ_PLLCAPPD_ENFORCE_ENABLE);
	usleep_range(25, 30);

	/* step 4 */
	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_ENABLE);
	tegra_sor_pad_cal_power(sor, false); /* PDCAL */
	usleep_range(70, 120);

	/* step 5 */
	tegra_dc_sor_io_set_dpd(sor, false);
	usleep_range(70, 120);

	sor->power_is_up = false;
}

static u32 tegra_sor_hdmi_get_pixel_depth(struct tegra_dc *dc)
{
	int yuv_flag = dc->mode.vmode & FB_VMODE_YUV_MASK;
	int yuv_bypass_mode = dc->mode.vmode & FB_VMODE_BYPASS;

	if (!yuv_flag)
		return NV_SOR_STATE1_ASY_PIXELDEPTH_DEFAULTVAL;

	if (!yuv_bypass_mode) {
		if (tegra_dc_is_yuv420_8bpc(&dc->mode)) {
			if (tegra_dc_is_t19x())
				return tegra_sor_yuv420_8bpc_pixel_depth_t19x();
		} else if (yuv_flag & FB_VMODE_Y422) {
			if (yuv_flag & FB_VMODE_Y24)
				return NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_16_422;
			else if (yuv_flag & FB_VMODE_Y30)
				return NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_20_422;
			else if (yuv_flag & FB_VMODE_Y36)
				return NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_422;
		} else {
			if (yuv_flag & FB_VMODE_Y24)
				return NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_444;
			else if (yuv_flag & FB_VMODE_Y30)
				return NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_30_444;
			else if (yuv_flag & FB_VMODE_Y36)
				return NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_36_444;
		}
	} else {
		return NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_444;
	}

	return NV_SOR_STATE1_ASY_PIXELDEPTH_DEFAULTVAL;
}

static u32 tegra_sor_dp_get_pixel_depth(struct tegra_dc *dc)
{
	int yuv_flag = dc->mode.vmode & FB_VMODE_YUV_MASK;
	int yuv_bypass_mode = dc->mode.vmode & FB_VMODE_BYPASS;

	if (yuv_flag) {
		if (!yuv_bypass_mode) {
			if (yuv_flag & FB_VMODE_Y422) {
				if (yuv_flag & FB_VMODE_Y24)
					return
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_16_422;
			} else if (IS_RGB(yuv_flag) ||
				(yuv_flag & FB_VMODE_Y444)) {
				if (yuv_flag & FB_VMODE_Y24)
					return
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_444;
				else if (yuv_flag & FB_VMODE_Y30)
					return
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_30_444;
				else if (yuv_flag & FB_VMODE_Y36)
					return
					NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_36_444;
			} else {
				dev_err(&dc->ndev->dev, "%s: Unsupported mode with vmode: 0x%x for DP\n",
						__func__, dc->mode.vmode);
			}
		} else {
			dev_err(&dc->ndev->dev, "%s: Unsupported bypass mode with vmode: 0x%x for DP\n",
					__func__, dc->mode.vmode);
		}
	} else {
		return (dc->out->depth > 18 || !dc->out->depth) ?
				NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_24_444 :
				NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_18_444;
	}

	return NV_SOR_STATE1_ASY_PIXELDEPTH_DEFAULTVAL;
}

static u32 tegra_sor_get_pixel_depth(struct tegra_dc *dc)
{
	if (dc->out->type == TEGRA_DC_OUT_HDMI)
		return tegra_sor_hdmi_get_pixel_depth(dc);
	else if ((dc->out->type == TEGRA_DC_OUT_DP) ||
			(dc->out->type == TEGRA_DC_OUT_FAKE_DP))
		return tegra_sor_dp_get_pixel_depth(dc);

	dev_err(&dc->ndev->dev, "%s: unsupported out_type=%d\n",
			__func__, dc->out->type);
	return 0;
}

static u32 tegra_sor_get_range_compress(struct tegra_dc *dc)
{
	if ((dc->mode.vmode & FB_VMODE_BYPASS) ||
			!(dc->mode.vmode & FB_VMODE_LIMITED_RANGE))
		return NV_HEAD_STATE0_RANGECOMPRESS_DISABLE;

	return NV_HEAD_STATE0_RANGECOMPRESS_ENABLE;
}

static u32 tegra_sor_get_dynamic_range(struct tegra_dc *dc)
{
	if ((dc->mode.vmode & FB_VMODE_BYPASS) ||
		!(dc->mode.vmode & FB_VMODE_LIMITED_RANGE))
		return NV_HEAD_STATE0_DYNRANGE_VESA;

	return NV_HEAD_STATE0_DYNRANGE_CEA;
}

static u32 tegra_sor_get_color_space(struct tegra_dc *dc)
{
	int yuv_flag = dc->mode.vmode & FB_VMODE_YUV_MASK;
	u32 color_space = NV_HEAD_STATE0_COLORSPACE_RGB;

	if (!IS_RGB(yuv_flag)) {
		u32 ec = dc->mode.vmode & FB_VMODE_EC_MASK;

		switch (ec) {
		case FB_VMODE_EC_ADOBE_YCC601:
		case FB_VMODE_EC_SYCC601:
		case FB_VMODE_EC_XVYCC601:
			color_space = NV_HEAD_STATE0_COLORSPACE_YUV_601;
			break;
		case FB_VMODE_EC_XVYCC709:
			color_space = NV_HEAD_STATE0_COLORSPACE_YUV_709;
			break;
		default:
			break;
		}
	}

	return color_space;
}

static inline u32 tegra_sor_get_adjusted_hblank(struct tegra_dc *dc,
						u32 hblank_end)
{
	/* For native HDMI420 8bpc, HBLANK_END needs to be halved. */
	if (tegra_dc_is_t19x() &&
		tegra_dc_is_yuv420_8bpc(&dc->mode) &&
		!(dc->mode.vmode & FB_VMODE_BYPASS)
		&& dc->out->type == TEGRA_DC_OUT_HDMI)
		hblank_end = hblank_end / 2;

	return hblank_end;
}

static void tegra_dc_sor_config_panel(struct tegra_dc_sor_data *sor,
						bool is_lvds)
{
	struct tegra_dc *dc = sor->dc;
	const struct tegra_dc_mode *dc_mode = &dc->mode;
	int head_num = dc->ctrl_num;
	u32 reg_val = NV_SOR_HEADNUM(head_num);
	u32 vtotal, htotal;
	u32 vsync_end, hsync_end;
	u32 vblank_end, hblank_end;
	u32 vblank_start, hblank_start;
	int out_type = dc->out->type;

	if (out_type == TEGRA_DC_OUT_HDMI)
		reg_val |= NV_SOR_STATE1_ASY_PROTOCOL_SINGLE_TMDS_A;
	else if ((out_type == TEGRA_DC_OUT_DP) ||
		(out_type == TEGRA_DC_OUT_FAKE_DP))
		reg_val |= NV_SOR_STATE1_ASY_PROTOCOL_DP_A;
	else
		reg_val |= NV_SOR_STATE1_ASY_PROTOCOL_LVDS_CUSTOM;

	reg_val |= NV_SOR_STATE1_ASY_SUBOWNER_NONE |
		NV_SOR_STATE1_ASY_CRCMODE_COMPLETE_RASTER;

	if (dc_mode->flags & TEGRA_DC_MODE_FLAG_NEG_H_SYNC)
		reg_val |= NV_SOR_STATE1_ASY_HSYNCPOL_NEGATIVE_TRUE;
	else
		reg_val |= NV_SOR_STATE1_ASY_HSYNCPOL_POSITIVE_TRUE;

	if (dc_mode->flags & TEGRA_DC_MODE_FLAG_NEG_V_SYNC)
		reg_val |= NV_SOR_STATE1_ASY_VSYNCPOL_NEGATIVE_TRUE;
	else
		reg_val |= NV_SOR_STATE1_ASY_VSYNCPOL_POSITIVE_TRUE;

	reg_val |= tegra_sor_get_pixel_depth(dc);
	tegra_sor_writel(sor, NV_SOR_STATE1, reg_val);

	/* Interlaced is not supported in hw */
	reg_val = NV_HEAD_STATE0_INTERLACED_PROGRESSIVE;
	reg_val |= tegra_sor_get_range_compress(sor->dc);
	reg_val |= tegra_sor_get_dynamic_range(sor->dc);
	reg_val |= tegra_sor_get_color_space(sor->dc);
	tegra_sor_writel(sor, nv_sor_head_state0(head_num), reg_val);

	BUG_ON(!dc_mode);
	vtotal = dc_mode->v_sync_width + dc_mode->v_back_porch +
		dc_mode->v_active + dc_mode->v_front_porch;
	htotal = dc_mode->h_sync_width + dc_mode->h_back_porch +
		dc_mode->h_active + dc_mode->h_front_porch;
	tegra_sor_writel(sor, nv_sor_head_state1(head_num),
		vtotal << NV_HEAD_STATE1_VTOTAL_SHIFT |
		htotal << NV_HEAD_STATE1_HTOTAL_SHIFT);

	vsync_end = dc_mode->v_sync_width - 1;
	hsync_end = dc_mode->h_sync_width - 1;
	tegra_sor_writel(sor, nv_sor_head_state2(head_num),
		vsync_end << NV_HEAD_STATE2_VSYNC_END_SHIFT |
		hsync_end << NV_HEAD_STATE2_HSYNC_END_SHIFT);

	vblank_end = vsync_end + dc_mode->v_back_porch;
	hblank_end = hsync_end + dc_mode->h_back_porch;
	hblank_end = tegra_sor_get_adjusted_hblank(dc, hblank_end);
	tegra_sor_writel(sor, nv_sor_head_state3(head_num),
		vblank_end << NV_HEAD_STATE3_VBLANK_END_SHIFT |
		hblank_end << NV_HEAD_STATE3_HBLANK_END_SHIFT);

	vblank_start = vblank_end + dc_mode->v_active;
	hblank_start = hblank_end + dc_mode->h_active;
	tegra_sor_writel(sor, nv_sor_head_state4(head_num),
		vblank_start << NV_HEAD_STATE4_VBLANK_START_SHIFT |
		hblank_start << NV_HEAD_STATE4_HBLANK_START_SHIFT);

	tegra_sor_writel(sor, nv_sor_head_state5(head_num), 0x1);

	tegra_sor_write_field(sor, NV_SOR_CSTM,
		NV_SOR_CSTM_ROTCLK_DEFAULT_MASK |
		NV_SOR_CSTM_LVDS_EN_ENABLE,
		2 << NV_SOR_CSTM_ROTCLK_SHIFT |
		is_lvds ? NV_SOR_CSTM_LVDS_EN_ENABLE :
		NV_SOR_CSTM_LVDS_EN_DISABLE);

	tegra_dc_sor_config_pwm(sor, 1024, 1024);
}

static void tegra_dc_sor_enable_dc(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;
	u32 reg_val;

	tegra_dc_get(dc);

	reg_val = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);

	if (tegra_platform_is_vdk())
		tegra_dc_writel(dc, reg_val | WRITE_MUX_ASSEMBLY,
			DC_CMD_STATE_ACCESS);
	else
		tegra_dc_writel(dc, reg_val | WRITE_MUX_ACTIVE,
			DC_CMD_STATE_ACCESS);

	if (tegra_dc_is_t21x()) {
		if (tegra_platform_is_fpga()) {
			tegra_dc_writel(dc, 0, DC_DISP_DISP_CLOCK_CONTROL);
			tegra_dc_writel(dc, 0xe, DC_DISP_DC_MCCIF_FIFOCTRL);
		}

		tegra_dc_writel(dc, VSYNC_H_POSITION(1),
				DC_DISP_DISP_TIMING_OPTIONS);
	}

	/* Enable DC */
	if (dc->out->vrr) {
		if (tegra_dc_is_nvdisplay())
			tegra_nvdisp_set_vrr_mode(dc);
		else
			tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY,
					DC_CMD_DISPLAY_COMMAND);
	}
	else if (dc->frm_lck_info.frame_lock_enable &&
		((dc->out->type == TEGRA_DC_OUT_HDMI) ||
		(dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP))) {
		int ret;
		mutex_unlock(&dc->lock);
		ret = tegra_dc_common_sync_frames(dc, DC_CMD_DISPLAY_COMMAND,
				DC_CMD_STATE_ACCESS, DISP_CTRL_MODE_C_DISPLAY);
		mutex_lock(&dc->lock);
		if (ret)
			dev_err(&dc->ndev->dev,
				"failed to submit job for tegradc.%d with error : %d\n",
				dc->ctrl_num, ret);
	} else {
		tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY,
			DC_CMD_DISPLAY_COMMAND);
	}

	tegra_dc_writel(dc, reg_val, DC_CMD_STATE_ACCESS);

	tegra_dc_put(dc);
}

void tegra_sor_cal(struct tegra_dc_sor_data *sor)
{
	u32 nv_sor_pll1_reg = nv_sor_pll1();
	u32 nv_sor_pll2_reg = nv_sor_pll2();

	if (sor->dc->initialized)
		return;

	/* For HDMI, rterm calibration is currently enabled only on T19x. */
	if (!tegra_dc_is_t19x() && sor->dc->out->type == TEGRA_DC_OUT_HDMI)
		return;

	tegra_dc_sor_io_set_dpd(sor, true);
	usleep_range(5, 20);

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_DISABLE);
	tegra_sor_write_field(sor, nv_sor_pll1_reg,
		NV_SOR_PLL1_TMDS_TERM_ENABLE,
		NV_SOR_PLL1_TMDS_TERM_ENABLE);
	usleep_range(20, 100);

	tegra_sor_pad_cal_power(sor, true);
	usleep_range(10, 20);

	tegra_dc_sor_termination_cal(sor);

	tegra_sor_pad_cal_power(sor, false);
	usleep_range(10, 20);

	tegra_sor_write_field(sor, nv_sor_pll2_reg,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_MASK,
		NV_SOR_PLL2_AUX6_BANDGAP_POWERDOWN_ENABLE);
	tegra_sor_write_field(sor, nv_sor_pll1_reg,
		NV_SOR_PLL1_TMDS_TERM_ENABLE,
		NV_SOR_PLL1_TMDS_TERM_DISABLE);
	usleep_range(20, 100);

	tegra_dc_sor_io_set_dpd(sor, false);
	usleep_range(5, 20);
}

void tegra_sor_config_xbar(struct tegra_dc_sor_data *sor)
{
	u32 val = 0, mask = 0, shift = 0;
	u32 i = 0;

	mask = (NV_SOR_XBAR_BYPASS_MASK | NV_SOR_XBAR_LINK_SWAP_MASK);
	for (i = 0, shift = 2; i < sizeof(sor->xbar_ctrl)/sizeof(u32);
		shift += 3, i++) {
		mask |= NV_SOR_XBAR_LINK_XSEL_MASK << shift;
		val |= sor->xbar_ctrl[i] << shift;
	}

	tegra_sor_write_field(sor, NV_SOR_XBAR_CTRL, mask, val);
	tegra_sor_writel(sor, NV_SOR_XBAR_POL, 0);
}

void tegra_dc_sor_enable_dp(struct tegra_dc_sor_data *sor)
{
	if (!sor->dc->initialized) {
		tegra_sor_cal(sor);
		tegra_sor_dp_pad_power_up(sor);
		tegra_sor_power_lanes(sor, sor->link_cfg->lane_count, true);
	} else {
		/* Update sor power state for seamless */
		sor->power_is_up = true;
	}

}

static void tegra_dc_sor_enable_sor(struct tegra_dc_sor_data *sor, bool enable)
{
	struct tegra_dc *dc = sor->dc;

	/* Do not disable SOR during seamless boot */
	if (dc->initialized && !enable)
		return;

	if (tegra_dc_is_t21x()) {
		u32 reg_val = tegra_dc_readl(sor->dc, DC_DISP_DISP_WIN_OPTIONS);
		u32 enb = sor->ctrl_num ? SOR1_ENABLE : SOR_ENABLE;

		if (dc->out->type == TEGRA_DC_OUT_HDMI)
			enb |= SOR1_TIMING_CYA;

		reg_val = enable ? reg_val | enb : reg_val & ~enb;
		tegra_dc_writel(dc, reg_val, DC_DISP_DISP_WIN_OPTIONS);
	} else if (tegra_dc_is_t18x()) {
		tegra_dc_enable_sor_t18x(dc, sor->ctrl_num, enable);
	} else if (tegra_dc_is_t19x()) {
		tegra_dc_enable_sor_t19x(dc, sor->ctrl_num, enable);
	} else {
		pr_err("%s: Unknown Tegra SOC\n", __func__);
		return;
	}
}

void tegra_dc_sor_attach(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;
	u32 reg_val;

	if (sor->sor_state == SOR_ATTACHED)
		return;

	tegra_dc_get(dc);

	reg_val = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);

	if (tegra_platform_is_vdk())
		tegra_dc_writel(dc, reg_val | WRITE_MUX_ASSEMBLY,
			DC_CMD_STATE_ACCESS);
	else
		tegra_dc_writel(dc, reg_val | WRITE_MUX_ACTIVE,
			DC_CMD_STATE_ACCESS);

	tegra_dc_sor_config_panel(sor, false);
	tegra_dc_sor_update(sor);

	/* Sleep request */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_TEST,
		NV_SOR_TEST_ATTACHED_DEFAULT_MASK,
		NV_SOR_TEST_ATTACHED_TRUE,
		100, TEGRA_SOR_ATTACH_TIMEOUT_MS)) {
		dev_err(&dc->ndev->dev,
			"dc timeout waiting for ATTACH = TRUE\n");
	}

	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_NORMAL |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	tegra_dc_sor_enable_dc(sor);

	tegra_dc_sor_enable_sor(sor, true);

	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_AWAKE |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_NORMAL |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	tegra_dc_writel(dc, reg_val, DC_CMD_STATE_ACCESS);
	tegra_dc_put(dc);

	sor->sor_state = SOR_ATTACHED;
}

/* Disable windows and set minimum raster timings */
static void
tegra_dc_sor_disable_win_short_raster_t21x(struct tegra_dc *dc, int *dc_reg_ctx)
{
	int selected_windows, i;

	selected_windows = tegra_dc_readl(dc, DC_CMD_DISPLAY_WINDOW_HEADER);

	/* Store and clear window options */
	for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
			DC_CMD_DISPLAY_WINDOW_HEADER);
		dc_reg_ctx[i] = tegra_dc_readl(dc, DC_WIN_WIN_OPTIONS);
		tegra_dc_writel(dc, 0, DC_WIN_WIN_OPTIONS);
		tegra_dc_writel(dc, WIN_A_ACT_REQ << i, DC_CMD_STATE_CONTROL);
	}

	tegra_dc_writel(dc, selected_windows, DC_CMD_DISPLAY_WINDOW_HEADER);

	/* Store current raster timings and set minimum timings */
	dc_reg_ctx[i++] = tegra_dc_readl(dc, DC_DISP_REF_TO_SYNC);
	tegra_dc_writel(dc, min_mode.h_ref_to_sync |
		(min_mode.v_ref_to_sync << 16), DC_DISP_REF_TO_SYNC);

	dc_reg_ctx[i++] = tegra_dc_readl(dc, DC_DISP_SYNC_WIDTH);
	tegra_dc_writel(dc, min_mode.h_sync_width |
		(min_mode.v_sync_width << 16), DC_DISP_SYNC_WIDTH);

	dc_reg_ctx[i++] = tegra_dc_readl(dc, DC_DISP_BACK_PORCH);
	tegra_dc_writel(dc, min_mode.h_back_porch |
		((min_mode.v_back_porch - min_mode.v_ref_to_sync) << 16),
		DC_DISP_BACK_PORCH);

	dc_reg_ctx[i++] = tegra_dc_readl(dc, DC_DISP_FRONT_PORCH);
	tegra_dc_writel(dc, min_mode.h_front_porch |
		((min_mode.v_front_porch + min_mode.v_ref_to_sync) << 16),
		DC_DISP_FRONT_PORCH);

	dc_reg_ctx[i++] = tegra_dc_readl(dc, DC_DISP_DISP_ACTIVE);
	tegra_dc_writel(dc, min_mode.h_active | (min_mode.v_active << 16),
			DC_DISP_DISP_ACTIVE);

	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
}

/* Restore previous windows status and raster timings */
static void
tegra_dc_sor_restore_win_and_raster_t21x(struct tegra_dc *dc, int *dc_reg_ctx)
{
	int selected_windows, i;

	selected_windows = tegra_dc_readl(dc, DC_CMD_DISPLAY_WINDOW_HEADER);

	for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
			DC_CMD_DISPLAY_WINDOW_HEADER);
		tegra_dc_writel(dc, dc_reg_ctx[i], DC_WIN_WIN_OPTIONS);
		tegra_dc_writel(dc, WIN_A_ACT_REQ << i, DC_CMD_STATE_CONTROL);
	}

	tegra_dc_writel(dc, selected_windows, DC_CMD_DISPLAY_WINDOW_HEADER);

	tegra_dc_writel(dc, dc_reg_ctx[i++], DC_DISP_REF_TO_SYNC);
	tegra_dc_writel(dc, dc_reg_ctx[i++], DC_DISP_SYNC_WIDTH);
	tegra_dc_writel(dc, dc_reg_ctx[i++], DC_DISP_BACK_PORCH);
	tegra_dc_writel(dc, dc_reg_ctx[i++], DC_DISP_FRONT_PORCH);
	tegra_dc_writel(dc, dc_reg_ctx[i++], DC_DISP_DISP_ACTIVE);

	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
}

static inline void tegra_sor_save_dc_state(struct tegra_dc_sor_data *sor)
{
	if (tegra_dc_is_nvdisplay())
		tegra_nvdisp_disable_wins(sor->dc, sor->win_state_arr);
	else
		tegra_dc_sor_disable_win_short_raster_t21x(sor->dc,
			sor->dc_reg_ctx);
}

static inline void tegra_sor_restore_dc_state(struct tegra_dc_sor_data *sor)
{
	if (tegra_dc_is_nvdisplay())
		tegra_nvdisp_restore_wins(sor->dc, sor->win_state_arr);
	else
		tegra_dc_sor_restore_win_and_raster_t21x(sor->dc,
			sor->dc_reg_ctx);
}

void tegra_sor_stop_dc(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;

	tegra_dc_get(dc);

	if (tegra_dc_is_nvdisplay()) {
		/*SOR should be attached if the Display command != STOP */
		/* Stop DC */
		tegra_dc_writel(dc, DISP_CTRL_MODE_STOP,
				DC_CMD_DISPLAY_COMMAND);
		tegra_dc_enable_general_act(dc);

		/* Stop DC->SOR path */
		tegra_dc_sor_enable_sor(sor, false);
	} else {
		/* Stop DC->SOR path */
		tegra_dc_sor_enable_sor(sor, false);
		tegra_dc_enable_general_act(dc);

		/* Stop DC */
		tegra_dc_writel(dc, DISP_CTRL_MODE_STOP,
				DC_CMD_DISPLAY_COMMAND);
	}
	tegra_dc_enable_general_act(dc);

	tegra_dc_put(dc);
}

void tegra_dc_sor_sleep(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;

	if (sor->sor_state == SOR_SLEEP)
		return;

	/* set OR mode to SAFE */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_AWAKE |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);
	if (tegra_dc_sor_poll_register(sor, NV_SOR_PWR,
		NV_SOR_PWR_MODE_DEFAULT_MASK,
		NV_SOR_PWR_MODE_SAFE,
		100, TEGRA_SOR_ATTACH_TIMEOUT_MS)) {
		dev_err(&dc->ndev->dev,
			"dc timeout waiting for OR MODE = SAFE\n");
	}

	/* set HEAD mode to SLEEP */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_YES);
	tegra_dc_sor_super_update(sor);

	if (tegra_dc_sor_poll_register(sor, NV_SOR_TEST,
		NV_SOR_TEST_ACT_HEAD_OPMODE_DEFAULT_MASK,
		NV_SOR_TEST_ACT_HEAD_OPMODE_SLEEP,
		100, TEGRA_SOR_ATTACH_TIMEOUT_MS)) {
		dev_err(&dc->ndev->dev,
			"dc timeout waiting for HEAD MODE = SLEEP\n");
	}
	sor->sor_state = SOR_SLEEP;

}

void tegra_dc_sor_pre_detach(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;

	if (sor->sor_state != SOR_ATTACHED && sor->sor_state != SOR_SLEEP)
		return;

	tegra_dc_get(dc);

	tegra_dc_sor_sleep(sor);

	tegra_sor_save_dc_state(sor);

	sor->sor_state = SOR_DETACHING;
	tegra_dc_put(dc);
}

void tegra_dc_sor_detach(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;
	unsigned long dc_int_mask;

	if (sor->sor_state == SOR_DETACHED)
		return;

	tegra_dc_get(dc);

	/* Mask DC interrupts during the 2 dummy frames required for detach */
	dc_int_mask = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	if (sor->sor_state != SOR_DETACHING)
		tegra_dc_sor_pre_detach(sor);

	/* detach SOR */
	tegra_sor_writel(sor, NV_SOR_SUPER_STATE1,
		NV_SOR_SUPER_STATE1_ASY_HEAD_OP_SLEEP |
		NV_SOR_SUPER_STATE1_ASY_ORMODE_SAFE |
		NV_SOR_SUPER_STATE1_ATTACHED_NO);
	tegra_dc_sor_super_update(sor);
	if (tegra_dc_sor_poll_register(sor, NV_SOR_TEST,
		NV_SOR_TEST_ATTACHED_DEFAULT_MASK,
		NV_SOR_TEST_ATTACHED_FALSE,
		100, TEGRA_SOR_ATTACH_TIMEOUT_MS)) {
		dev_err(&dc->ndev->dev,
			"dc timeout waiting for ATTACH = FALSE\n");
	}

	tegra_sor_writel(sor, NV_SOR_STATE1,
		NV_SOR_STATE1_ASY_OWNER_NONE |
		NV_SOR_STATE1_ASY_SUBOWNER_NONE |
		NV_SOR_STATE1_ASY_PROTOCOL_LVDS_CUSTOM);
	tegra_dc_sor_update(sor);

	tegra_sor_stop_dc(sor);

	tegra_sor_restore_dc_state(sor);

	tegra_dc_writel(dc, dc_int_mask, DC_CMD_INT_MASK);
	sor->sor_state = SOR_DETACHED;
	tegra_dc_put(dc);
}

void tegra_dc_sor_disable(struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;

	tegra_sor_config_safe_clk(sor);

	/* Power down DP lanes */
	if (tegra_sor_power_lanes(sor, 4, false)) {
		dev_err(&dc->ndev->dev,
			"Failed to power down dp lanes\n");
		return;
	}

	/* Power down pad macro */
	tegra_sor_dp_pad_power_down(sor);

	if (tegra_platform_is_vdk())
		return;

	/* Reset SOR */
	tegra_sor_reset(sor);
	tegra_sor_clk_disable(sor);
}

void tegra_dc_sor_set_internal_panel(struct tegra_dc_sor_data *sor, bool is_int)
{
	u32 reg_val;

	reg_val = tegra_sor_readl(sor, NV_SOR_DP_SPARE(sor->portnum));
	if (is_int)
		reg_val |= NV_SOR_DP_SPARE_PANEL_INTERNAL;
	else
		reg_val &= ~NV_SOR_DP_SPARE_PANEL_INTERNAL;

	reg_val |= NV_SOR_DP_SPARE_SOR_CLK_SEL_MACRO_SORCLK;

	tegra_sor_writel(sor, NV_SOR_DP_SPARE(sor->portnum), reg_val);

	if (sor->dc->out->type == TEGRA_DC_OUT_HDMI) {
		if (tegra_dc_is_nvdisplay())
			tegra_sor_write_field(sor,
				NV_SOR_DP_SPARE(sor->portnum),
				NV_SOR_DP_SPARE_VIDEO_PREANBLE_CYA_MASK,
				NV_SOR_DP_SPARE_VIDEO_PREANBLE_CYA_DISABLE);
		else
			tegra_sor_write_field(sor,
				NV_SOR_DP_SPARE(sor->portnum),
				NV_SOR_DP_SPARE_VIDEO_PREANBLE_CYA_MASK,
				NV_SOR_DP_SPARE_VIDEO_PREANBLE_CYA_ENABLE);
	}
}

void tegra_dc_sor_set_link_bandwidth(struct tegra_dc_sor_data *sor, u8 link_bw)
{
	WARN_ON(sor->sor_state == SOR_ATTACHED);

	/* FIXME: does order matter with dettached SOR? */
	if (tegra_dc_is_nvdisplay()) {
		/* link rate = fixed pll_dp@270MHz * link_bw / 10 */
		clk_set_rate(sor->pad_clk, 270000000UL * link_bw / 10);
	}

	tegra_sor_write_field(sor, NV_SOR_CLK_CNTRL,
		NV_SOR_CLK_CNTRL_DP_LINK_SPEED_MASK,
		link_bw << NV_SOR_CLK_CNTRL_DP_LINK_SPEED_SHIFT);

	/* It can take upto 200us for PLLs in analog macro to settle */
	udelay(300);
}

void tegra_dc_sor_set_lane_count(struct tegra_dc_sor_data *sor, u8 lane_count)
{
	u32 reg_lane_cnt = 0;

	switch (lane_count) {
	case 0:
		reg_lane_cnt = NV_SOR_DP_LINKCTL_LANECOUNT_ZERO;
		break;
	case 1:
		reg_lane_cnt = NV_SOR_DP_LINKCTL_LANECOUNT_ONE;
		break;
	case 2:
		reg_lane_cnt = NV_SOR_DP_LINKCTL_LANECOUNT_TWO;
		break;
	case 4:
		reg_lane_cnt = NV_SOR_DP_LINKCTL_LANECOUNT_FOUR;
		break;
	default:
		/* 0 should be handled earlier. */
		dev_err(&sor->dc->ndev->dev, "dp: Invalid lane count %d\n",
			lane_count);
		return;
	}

	tegra_sor_write_field(sor, NV_SOR_DP_LINKCTL(sor->portnum),
				NV_SOR_DP_LINKCTL_LANECOUNT_MASK,
				reg_lane_cnt);
}

void tegra_sor_setup_clk(struct tegra_dc_sor_data *sor, struct clk *clk,
	bool is_lvds)
{
	struct clk *dc_parent_clk;
	struct tegra_dc *dc = sor->dc;

	if (tegra_platform_is_vdk())
		return;

	if (clk == dc->clk) {
		dc_parent_clk = clk_get_parent(clk);
		BUG_ON(!dc_parent_clk);

		/* Change for seamless */
		if (!dc->initialized) {
			if (dc->mode.pclk != clk_get_rate(dc_parent_clk)) {
				clk_set_rate(dc_parent_clk, dc->mode.pclk);
				clk_set_rate(clk, dc->mode.pclk);
			}
		}

		if (!tegra_dc_is_nvdisplay())
			return;

		/*
		 * For t18x plldx cannot go below 27MHz.
		 * Real HW limit is lesser though.
		 * 27Mz is chosen to have a safe margin.
		 */
		if (dc->mode.pclk < 27000000) {
			if ((2 * dc->mode.pclk) != clk_get_rate(dc_parent_clk))
				clk_set_rate(dc_parent_clk, 2 * dc->mode.pclk);
			if (dc->mode.pclk != clk_get_rate(dc->clk))
				clk_set_rate(dc->clk, dc->mode.pclk);
		}
	}
}

void tegra_sor_precharge_lanes(struct tegra_dc_sor_data *sor)
{
	const struct tegra_dc_dp_link_config *cfg = sor->link_cfg;
	u32 nv_sor_dp_padctl_reg = nv_sor_dp_padctl(sor->portnum);
	u32 val = 0;

	val = tegra_sor_get_pd_tx_bitmap(sor, cfg->lane_count);

	/* force lanes to output common mode voltage */
	tegra_sor_write_field(sor, nv_sor_dp_padctl_reg,
		(0xf << NV_SOR_DP_PADCTL_COMODE_TXD_0_DP_TXD_2_SHIFT),
		(val << NV_SOR_DP_PADCTL_COMODE_TXD_0_DP_TXD_2_SHIFT));

	/* precharge for atleast 10us */
	usleep_range(20, 100);

	/* fallback to normal operation */
	tegra_sor_write_field(sor, nv_sor_dp_padctl_reg,
		(0xf << NV_SOR_DP_PADCTL_COMODE_TXD_0_DP_TXD_2_SHIFT), 0);
}

void tegra_dc_sor_modeset_notifier(struct tegra_dc_sor_data *sor, bool is_lvds)
{
	if (sor->dc->initialized)
		return;

	if (!sor->clk_type)
		tegra_sor_config_safe_clk(sor);
	tegra_sor_clk_enable(sor);

	tegra_dc_sor_config_panel(sor, is_lvds);
	tegra_dc_sor_update(sor);
	tegra_dc_sor_super_update(sor);

	tegra_sor_clk_disable(sor);
}

