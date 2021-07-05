/*
 * sor.h: tegra dc sor structue and function declarations.
 *
 * Copyright (c) 2011-2020, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_SOR_H__
#define __DRIVERS_VIDEO_TEGRA_DC_SOR_H__

#include <soc/tegra/chip-id.h>
#include <linux/clk/tegra.h>
#include <linux/reset.h>
#include <soc/tegra/tegra_bpmp.h>
#include <linux/rwsem.h>
#include <linux/delay.h>
#include <uapi/video/tegra_dc_ext.h>
#include "dc_priv.h"
#include "sor_regs.h"

/* Handle to a training pattern data object. Serves as the sole interface of
 * APIs and data structures to the training pattern data object
 */
enum tegra_dc_dp_training_pattern_key {
	TEGRA_DC_DP_TRAINING_PATTERN_DISABLE,
	TEGRA_DC_DP_TRAINING_PATTERN_1,
	TEGRA_DC_DP_TRAINING_PATTERN_2,
	TEGRA_DC_DP_TRAINING_PATTERN_3,
	TEGRA_DC_DP_TRAINING_PATTERN_D102,
	TEGRA_DC_DP_TRAINING_PATTERN_SBLERRRATE,
	TEGRA_DC_DP_TRAINING_PATTERN_PRBS7,
	TEGRA_DC_DP_TRAINING_PATTERN_CSTM,
	TEGRA_DC_DP_TRAINING_PATTERN_HBR2_COMPLIANCE,
	TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT1,
	TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT3,
	TEGRA_DC_DP_TRAINING_PATTERN_4,
	TEGRA_DC_DP_TRAINING_PATTERN_BS_CSTM,
};

/*
 * tegra_dc_dp_training_pattern - Training Pattern data object
 * @dpcd_val    - DPCD value defined by DP spec, corresponding to the TPS
 *                (Training Pattern Sequence). Used to hint the TPS to the sink
 *                that the source intends to use for link training
 * @sor_reg_val - SOR value corresponding to the TPS. Used to force the source
 *                to use this TPS
 * @scrambling  - Denotes whether the bit stream needs to be scrambled
 * @chan_coding - Denotes whether the bit stream needs to be 8b/10b coded
 */
struct tegra_dc_dp_training_pattern {
	u8 dpcd_val;
	u8 sor_reg_val;
	bool scrambling;
	bool chan_coding;
};

/*
 * This enum defines the index values for array of link speeds supported by
 * the SORs on tegra SOCs. This enum shall be used as a sole way of interfacing
 * with the data structures requiring link speed member fields
 *
 * Note: The code makes assumptions about values being enumerated in increasing
 * order of link speeds. Please maintain the ascending order
 */
enum tegra_dc_sor_link_speed_key {
	TEGRA_DC_SOR_LINK_SPEED_G1_62,
	TEGRA_DC_SOR_LINK_SPEED_G2_7,
	TEGRA_DC_SOR_LINK_SPEED_G5_4,
	TEGRA_DC_SOR_LINK_SPEED_G8_1,
	TEGRA_DC_SOR_LINK_SPEED_MAX,
};

/*
 * tegra_dc_sor_link_speed - Parameters related to SOR link speed
 * @prod_prop   - Device Tree binding associated with a given speed
 * @max_link_bw - Maximum supported bandwidth (in MHz) for the given speed
 *                This can be derived by multiplying @link_rate by 270 MHz
 * @link_rate   - The link clock multiplier that generates required clock
 *                frequency from 270 MHz source clock
 */
struct tegra_dc_sor_link_speed {
	char *prod_prop;
	u32 max_link_bw;
	u8 link_rate;
};

/*
 * tegra_dc_dp_ext_dpcd_caps - Data structure for Extended Receiver Capability
 *				Field
 *
 * @valid       - Indicates whether these cap fields are valid and/or present.
 * @revision    - DPCD_REV (DPCD offset 0x2200)
 * @max_link_bw - MAX_LINK_RATE (DPCD offset 0x2201)
 */
struct tegra_dc_dp_ext_dpcd_caps {
	bool valid;
	u8 revision;
	u8 max_link_bw;
};

struct tegra_dc_dp_link_config {
	bool	is_valid;	/*
				 * True if link config adheres to dp spec.
				 * Does not guarantee link training success.
				 */

	/* Supported configuration */
	u8	max_link_bw;
	u8	max_lane_count;
	bool	downspread;
	bool	support_enhanced_framing;
	bool	support_vsc_ext_colorimetry;
	u32	bits_per_pixel;
	bool	alt_scramber_reset_cap; /* true for eDP */
	bool	only_enhanced_framing;	/* enhanced_frame_en ignored */
	bool	edp_cap;		/* eDP display control capable */
	bool	support_fast_lt;	/* Support fast link training */

	struct tegra_dc_dp_ext_dpcd_caps	ext_dpcd_caps;

	/* Actual configuration */
	u8	link_bw;
	u8	lane_count;
	bool	enhanced_framing;
	bool	scramble_ena;

	u32	activepolarity;
	u32	active_count;
	u32	tu_size;
	u32	active_frac;
	u32	watermark;

	s32	hblank_sym;
	s32	vblank_sym;

	bool	lt_data_valid;	/*
				 * True only if link training passed with this
				 * drive_current, preemphasis and postcursor.
				 */
	u32	drive_current[4];
	u32	preemphasis[4];
	u32	postcursor[4];

	/*
	 * Training Pattern Sequence to start channel equalization with,
	 * calculated based on an intersection of source and sink capabilities
	 */
	u32	tps;

	u8	aux_rd_interval;
};

enum {
	TEGRA_SOR_SAFE_CLK = 1,
	TEGRA_SOR_MACRO_CLK = 2,
};

struct tegra_dc_sor_data {
	int ctrl_num; /* SOR0, SOR1 etc. */
	struct tegra_dc *dc;
	struct device_node *np; /* dc->pdata->conn_np */

	void __iomem *base;
	struct clk *sor_clk; /* output of SORX_CLK_SEL0 MUX */
	struct clk *safe_clk; /* SOR safe clk */
	struct clk *pad_clk; /* output of SORX pad brick */
	struct clk *ref_clk; /* output of SORX_CLK_SRC MUX sourced from PLLD* */
	struct reset_control *rst;

	u8					 portnum;	/* 0 or 1 */
	const struct tegra_dc_dp_link_config	*link_cfg;

	bool   power_is_up;

	int dc_reg_ctx[DC_N_WINDOWS + 5];
	struct tegra_dc_win_detach_state *win_state_arr;

	enum {
		SOR_ATTACHED = 1,
		SOR_DETACHING,
		SOR_DETACHED,
		SOR_SLEEP,
	} sor_state;

	/* Table of link speeds supported by the source */
	const struct tegra_dc_sor_link_speed *link_speeds;
	unsigned int num_link_speeds;

	/* Table of training pattern sequences for DP link training */
	const struct tegra_dc_dp_training_pattern *training_patterns;
	unsigned int num_training_patterns;

	u8	clk_type;
	u32  xbar_ctrl[5];
	bool audio_support;
	bool hdcp_support;
	struct pinctrl *pinctrl_sor;
	struct pinctrl_state *dpd_enable;
	struct pinctrl_state *dpd_disable;
	int powergate_id;
	struct rw_semaphore reset_lock;
	struct dentry	*debugdir;
	u32 dev_id;
};

#define TEGRA_SOR_TIMEOUT_MS		1000
#define TEGRA_SOR_ATTACH_TIMEOUT_MS	50
#define TEGRA_SOR_SEQ_BUSY_TIMEOUT_MS	10000

struct tegra_dc_sor_data *tegra_dc_sor_init(struct tegra_dc *dc,
	const struct tegra_dc_dp_link_config *cfg);
void tegra_sor_config_xbar(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_destroy(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_enable_dp(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_attach(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_detach(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_pre_detach(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_sleep(struct tegra_dc_sor_data *sor);
int tegra_dc_sor_crc_get(struct tegra_dc_sor_data *sor, u32 *crc);
u32 tegra_dc_sor_debugfs_get_crc(struct tegra_dc_sor_data *sor, int *timeout);
void tegra_dc_sor_crc_en_dis(struct tegra_dc_sor_data *sor,
			     struct tegra_dc_ext_crc_sor_params params,
			     bool en);
void tegra_dc_sor_toggle_crc(struct tegra_dc_sor_data *sor, u32 val);
void tegra_dc_sor_disable(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_set_internal_panel(struct tegra_dc_sor_data *sor,
	bool is_int);
void tegra_dc_sor_set_link_bandwidth(struct tegra_dc_sor_data *sor,
	u8 link_bw);
void tegra_dc_sor_set_lane_count(struct tegra_dc_sor_data *sor, u8 lane_count);
void tegra_sor_pad_cal_power(struct tegra_dc_sor_data *sor, bool power_up);
void tegra_sor_setup_clk(struct tegra_dc_sor_data *sor, struct clk *clk,
	bool is_lvds);
void tegra_sor_precharge_lanes(struct tegra_dc_sor_data *sor);
int tegra_dc_sor_set_power_state(struct tegra_dc_sor_data *sor,
	int pu_pd);
void tegra_dc_sor_modeset_notifier(struct tegra_dc_sor_data *sor,
	bool is_lvds);
void tegra_sor_tpg(struct tegra_dc_sor_data *sor, u32 tp, u32 n_lanes);
void tegra_sor_port_enable(struct tegra_dc_sor_data *sor, bool enb);
int tegra_sor_power_lanes(struct tegra_dc_sor_data *sor,
					u32 lane_count, bool pu);
void tegra_sor_config_dp_clk_t21x(struct tegra_dc_sor_data *sor);
void tegra_sor_stop_dc(struct tegra_dc_sor_data *sor);
void tegra_sor_config_safe_clk(struct tegra_dc_sor_data *sor);
void tegra_sor_hdmi_pad_power_up(struct tegra_dc_sor_data *sor);
void tegra_sor_hdmi_pad_power_down(struct tegra_dc_sor_data *sor);
void tegra_dc_sor_termination_cal(struct tegra_dc_sor_data *sor);
void tegra_sor_cal(struct tegra_dc_sor_data *sor);
unsigned long tegra_dc_sor_poll_register(struct tegra_dc_sor_data *sor,
					u32 reg, u32 mask, u32 exp_val,
					u32 poll_interval_us,
					u32 timeout_ms);

u32 __attribute__((weak)) nv_sor_head_state0_t19x(u32 i);
u32 __attribute__((weak)) nv_sor_head_state1_t19x(u32 i);
u32 __attribute__((weak)) nv_sor_head_state2_t19x(u32 i);
u32 __attribute__((weak)) nv_sor_head_state3_t19x(u32 i);
u32 __attribute__((weak)) nv_sor_head_state4_t19x(u32 i);
u32 __attribute__((weak)) nv_sor_head_state5_t19x(u32 i);
u32 __attribute__((weak)) nv_sor_pll0_t19x(void);
u32 __attribute__((weak)) nv_sor_pll1_t19x(void);
u32 __attribute__((weak)) nv_sor_pll2_t19x(void);
u32 __attribute__((weak)) nv_sor_pll3_t19x(void);
u32 __attribute__((weak)) nv_sor_pll4_t19x(void);
u32 __attribute__((weak)) nv_sor_pll5_t19x(void);
u32 __attribute__((weak)) nv_sor_dp_padctl_t19x(u32 i);
u32 __attribute__((weak)) nv_sor_dp_misc1_override_t19x(void);
u32 __attribute__((weak)) nv_sor_dp_misc1_bit6_t19x(void);
u32 __attribute__((weak)) nv_sor_dp_int_enable_t19x(void);
void __attribute__((weak)) tegra_sor_clk_switch_setup_t19x(
				struct tegra_dc_sor_data *sor, bool unblock);
void __attribute__((weak)) tegra_sor_program_fpga_clk_mux_t19x(
				struct tegra_dc_sor_data *sor);
u32 __attribute__((weak)) tegra_sor_yuv420_8bpc_pixel_depth_t19x(void);

static inline bool tegra_dc_is_vrr_authentication_enabled(void)
{
	return (!tegra_dc_is_nvdisplay());
}

static inline u32 nv_sor_head_state0(u32 i)
{
	if (tegra_dc_is_t19x())
		return nv_sor_head_state0_t19x(i);
	else if (tegra_dc_is_t18x())
		return NV_HEAD_STATE0_T18X(i);
	else
		return NV_HEAD_STATE0(i);
}

static inline u32 nv_sor_head_state1(u32 i)
{
	if (tegra_dc_is_t19x())
		return nv_sor_head_state1_t19x(i);
	else if (tegra_dc_is_t18x())
		return NV_HEAD_STATE1_T18X(i);
	else
		return NV_HEAD_STATE1(i);
}

static inline u32 nv_sor_head_state2(u32 i)
{
	if (tegra_dc_is_t19x())
		return nv_sor_head_state2_t19x(i);
	else if (tegra_dc_is_t18x())
		return NV_HEAD_STATE2_T18X(i);
	else
		return NV_HEAD_STATE2(i);
}

static inline u32 nv_sor_head_state3(u32 i)
{
	if (tegra_dc_is_t19x())
		return nv_sor_head_state3_t19x(i);
	else if (tegra_dc_is_t18x())
		return NV_HEAD_STATE3_T18X(i);
	else
		return NV_HEAD_STATE3(i);
}

static inline u32 nv_sor_head_state4(u32 i)
{
	if (tegra_dc_is_t19x())
		return nv_sor_head_state4_t19x(i);
	else if (tegra_dc_is_t18x())
		return NV_HEAD_STATE4_T18X(i);
	else
		return NV_HEAD_STATE4(i);
}

static inline u32 nv_sor_head_state5(u32 i)
{
	if (tegra_dc_is_t19x())
		return nv_sor_head_state5_t19x(i);
	else if (tegra_dc_is_t18x())
		return NV_HEAD_STATE5_T18X(i);
	else
		return NV_HEAD_STATE5(i);
}

static inline u32 nv_sor_pll0(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_pll0_t19x();
	else if (tegra_dc_is_t18x())
		return NV_SOR_PLL0_T18X;
	else
		return NV_SOR_PLL0;
}

static inline u32 nv_sor_pll1(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_pll1_t19x();
	else if (tegra_dc_is_t18x())
		return NV_SOR_PLL1_T18X;
	else
		return NV_SOR_PLL1;
}

static inline u32 nv_sor_pll2(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_pll2_t19x();
	else if (tegra_dc_is_t18x())
		return NV_SOR_PLL2_T18X;
	else
		return NV_SOR_PLL2;
}

static inline u32 nv_sor_pll3(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_pll3_t19x();
	else if (tegra_dc_is_t18x())
		return NV_SOR_PLL3_T18X;
	else
		return NV_SOR_PLL3;
}

static inline u32 nv_sor_pll4(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_pll4_t19x();
	else if (tegra_dc_is_t18x())
		return NV_SOR_PLL4_T18X;
	else
		return NV_SOR_PLL4;
}

static inline u32 nv_sor_pll5(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_pll5_t19x();

	return NV_SOR_PLL5;
}

static inline u32 nv_sor_dp_padctl(u32 i)
{
	if (tegra_dc_is_t19x())
		return nv_sor_dp_padctl_t19x(i);
	else if (tegra_dc_is_t18x())
		return NV_SOR_DP_PADCTL_T18X(i);
	else
		return NV_SOR_DP_PADCTL(i);
}

static inline u32 nv_sor_dp_misc1_override(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_dp_misc1_override_t19x();
	else
		return NV_SOR_DP_MISC1_OVERRIDE;
}

static inline u32 nv_sor_dp_misc1_bit6(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_dp_misc1_bit6_t19x();
	else
		return NV_SOR_DP_MISC1_BIT6_0;
}

static inline u32 nv_sor_dp_int_enable(void)
{
	if (tegra_dc_is_t19x())
		return nv_sor_dp_int_enable_t19x();
	else
		return NV_SOR_DP_INT_ENABLE;
}

static inline void tegra_sor_clk_switch_setup(struct tegra_dc_sor_data *sor,
					bool flag)
{
	if (!sor)
		return;

	if (tegra_dc_is_t19x())
		tegra_sor_clk_switch_setup_t19x(sor, flag);
}

static inline void tegra_sor_program_fpga_clk_mux(
					struct tegra_dc_sor_data *sor)
{
	if (!sor || !sor->dc)
		return;

	if (tegra_dc_is_t19x())
		tegra_sor_program_fpga_clk_mux_t19x(sor);
}

static inline int tegra_sor_get_ctrl_num(struct tegra_dc_sor_data *sor)
{
	return (!sor || !sor->base) ? -ENODEV : sor->ctrl_num;
}

/*
 * For nvdisplay, sor->sor_clk was previously being used as the SOR reference
 * clk instead of the orclk. In order to be consistent with the previous naming
 * scheme, I'm using sor->ref_clk here to avoid breaking existing drivers. This
 * needs to be cleaned up later.
 */
static inline u32 tegra_sor_readl(struct tegra_dc_sor_data *sor, u32 reg)
{
	struct clk *clk;
	u32 reg_val;

	clk = (tegra_dc_is_nvdisplay()) ? sor->ref_clk : sor->sor_clk;
	if (likely(tegra_platform_is_silicon())) {
		if (WARN(!tegra_dc_is_clk_enabled(clk), "SOR is clock gated!"))
			return 0;
	}

	reg_val = readl(sor->base + reg * 4);
	trace_display_readl(sor->dc, reg_val, (char *)sor->base + reg * 4);
	return reg_val;
}

static inline void tegra_sor_writel(struct tegra_dc_sor_data *sor,
	u32 reg, u32 val)
{
	struct clk *clk;

	clk = (tegra_dc_is_nvdisplay()) ? sor->ref_clk : sor->sor_clk;
	if (likely(tegra_platform_is_silicon())) {
		if (WARN(!tegra_dc_is_clk_enabled(clk), "SOR is clock gated!"))
			return;
	}

	writel(val, sor->base + reg * 4);
	trace_display_writel(sor->dc, val, (char *)sor->base + reg * 4);
}

static inline void tegra_sor_write_field(struct tegra_dc_sor_data *sor,
	u32 reg, u32 mask, u32 val)
{
	u32 reg_val = tegra_sor_readl(sor, reg);
	reg_val &= ~mask;
	reg_val |= val;
	tegra_sor_writel(sor, reg, reg_val);
}

/*
 * For nvdisplay, sor->sor_clk was previously being used as the SOR reference
 * clk instead of the orclk. In order to be consistent with the previous naming
 * scheme, I'm using sor->ref_clk here to avoid breaking drivers who are using
 * these APIs to actually toggle the ref clk. This needs to be cleaned up later.
 */
static inline void tegra_sor_clk_enable(struct tegra_dc_sor_data *sor)
{
	if (tegra_dc_is_nvdisplay()) {
		if (tegra_platform_is_silicon() && tegra_bpmp_running())
			clk_prepare_enable(sor->ref_clk);
	} else {
		if (tegra_platform_is_silicon() || tegra_bpmp_running())
			clk_prepare_enable(sor->sor_clk);
	}
}

static inline void tegra_sor_clk_disable(struct tegra_dc_sor_data *sor)
{
	if (tegra_dc_is_nvdisplay()) {
		if (tegra_platform_is_silicon() && tegra_bpmp_running())
			clk_disable_unprepare(sor->ref_clk);
	} else {
		if (tegra_platform_is_silicon() || tegra_bpmp_running())
			clk_disable_unprepare(sor->sor_clk);
	}
}

static inline void tegra_sor_safe_clk_enable(struct tegra_dc_sor_data *sor)
{
	if (tegra_dc_is_nvdisplay()) {
		if (tegra_platform_is_silicon() && tegra_bpmp_running())
			clk_prepare_enable(sor->safe_clk);
	} else {
		if (tegra_platform_is_silicon() || tegra_bpmp_running())
			clk_prepare_enable(sor->safe_clk);
	}
}

static inline void tegra_sor_safe_clk_disable(struct tegra_dc_sor_data *sor)
{
	if (tegra_dc_is_nvdisplay()) {
		if (tegra_platform_is_silicon() && tegra_bpmp_running())
			clk_disable_unprepare(sor->safe_clk);
	} else {
		if (tegra_platform_is_silicon() || tegra_bpmp_running())
			clk_disable_unprepare(sor->safe_clk);
	}
}

static inline int tegra_get_sor_reset_ctrl(struct tegra_dc_sor_data *sor,
	struct device_node *np_sor, const char *res_name)
{
	if (tegra_dc_is_nvdisplay()) {
		/* Use only if bpmp is enabled */
		if (!tegra_bpmp_running())
			return 0;
	}

	sor->rst = of_reset_control_get(np_sor, res_name);
	if (IS_ERR(sor->rst)) {
		dev_err(&sor->dc->ndev->dev,
			"Unable to get %s reset control\n", res_name);
		return PTR_ERR(sor->rst);
	}
	reset_control_deassert(sor->rst);
	return 0;
}

static inline void tegra_sor_reset(struct tegra_dc_sor_data *sor)
{
	if (tegra_platform_is_sim())
		return;

	down_write(&sor->reset_lock);
	if (sor->rst) {
		reset_control_assert(sor->rst);
		mdelay(2);
		reset_control_deassert(sor->rst);
		mdelay(1);
	}
	up_write(&sor->reset_lock);
}

static inline u32 tegra_sor_readl_ext(struct tegra_dc_sor_data *sor, u32 reg)
{
	u32 val;

	down_read(&sor->reset_lock);
	val = readl(sor->base + reg * 4);
	up_read(&sor->reset_lock);
	trace_display_readl(sor->dc, val, (char *)sor->base + reg * 4);
	return val;
}

static inline void tegra_sor_writel_ext(struct tegra_dc_sor_data *sor,
	u32 reg, u32 val)
{
	down_read(&sor->reset_lock);
	writel(val, sor->base + reg * 4);
	up_read(&sor->reset_lock);
	trace_display_writel(sor->dc, val, (char *)sor->base + reg * 4);
}

static inline void tegra_sor_write_field_ext(struct tegra_dc_sor_data *sor,
	u32 reg, u32 mask, u32 val)
{
	u32 reg_val;

	down_read(&sor->reset_lock);
	reg_val = tegra_sor_readl(sor, reg);
	reg_val &= ~mask;
	reg_val |= val;
	tegra_sor_writel(sor, reg, reg_val);
	up_read(&sor->reset_lock);
}
#endif
