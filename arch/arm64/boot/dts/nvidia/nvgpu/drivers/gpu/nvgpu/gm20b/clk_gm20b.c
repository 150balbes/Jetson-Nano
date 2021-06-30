/*
 * GM20B Clocks
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/soc.h>
#include <nvgpu/fuse.h>
#include <nvgpu/bug.h>
#include <nvgpu/log.h>
#include <nvgpu/types.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/timers.h>
#include <nvgpu/gk20a.h>

#include "clk_gm20b.h"

#include <nvgpu/hw/gm20b/hw_trim_gm20b.h>
#include <nvgpu/hw/gm20b/hw_therm_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fuse_gm20b.h>

#define gk20a_dbg_clk(g, fmt, arg...) \
	nvgpu_log(g, gpu_dbg_clk, fmt, ##arg)

#define DFS_DET_RANGE	6	/* -2^6 ... 2^6-1 */
#define SDM_DIN_RANGE	12	/* -2^12 ... 2^12-1 */
#define DFS_TESTOUT_DET	BIT(0)
#define DFS_EXT_CAL_EN	BIT(9)
#define DFS_EXT_STROBE	BIT(16)

#define BOOT_GPU_UV_B1	1000000	/* gpu rail boot voltage 1.0V */
#define BOOT_GPU_UV_C1	800000	/* gpu rail boot voltage 0.8V */
#define ADC_SLOPE_UV	10000	/* default ADC detection slope 10mV */

#define DVFS_SAFE_MARGIN	10	/* 10% */

static struct pll_parms gpc_pll_params_b1 = {
	128000,  2600000,	/* freq */
	1300000, 2600000,	/* vco */
	12000,   38400,		/* u */
	1, 255,			/* M */
	8, 255,			/* N */
	1, 31,			/* PL */
	-165230, 214007,	/* DFS_COEFF */
	0, 0,			/* ADC char coeff - to be read from fuses */
	0x7 << 3,		/* vco control in NA mode */
	500,			/* Locking and ramping timeout */
	40,			/* Lock delay in NA mode */
	5,			/* IDDQ mode exit delay */
};

static struct pll_parms gpc_pll_params_c1 = {
	76800,   2600000,	/* freq */
	1300000, 2600000,	/* vco */
	19200,   38400,		/* u */
	1, 255,			/* M */
	8, 255,			/* N */
	1, 31,			/* PL */
	-172550, 195374,	/* DFS_COEFF */
	0, 0,			/* ADC char coeff - to be read from fuses */
	(0x1 << 3) | 0x7,	/* vco control in NA mode */
	500,			/* Locking and ramping timeout */
	40,			/* Lock delay in NA mode */
	5,			/* IDDQ mode exit delay */
	0x3 << 10,		/* DFS control settings */
};

static struct pll_parms gpc_pll_params;

static void clk_setup_slide(struct gk20a *g, u32 clk_u);

static void dump_gpc_pll(struct gk20a *g, struct pll *gpll, u32 last_cfg)
{
#define __DUMP_REG(__addr_str__)					\
	do {								\
		u32 __addr__ = trim_sys_ ## __addr_str__ ## _r();	\
		u32 __data__ = gk20a_readl(g, __addr__);		\
									\
		nvgpu_info(g, "  " #__addr_str__ " [0x%x] = 0x%x",	\
			   __addr__, __data__); 			\
	} while (0)

	nvgpu_info(g, "GPCPLL DUMP:");
	nvgpu_info(g, "  gpcpll s/w M=%u N=%u P=%u\n", gpll->M, gpll->N, gpll->PL);
	nvgpu_info(g, "  gpcpll_cfg_last = 0x%x\n", last_cfg);

	__DUMP_REG(gpcpll_cfg);
	__DUMP_REG(gpcpll_coeff);
	__DUMP_REG(sel_vco);

#undef __DUMP_REG
}

#define PLDIV_GLITCHLESS 1

#if PLDIV_GLITCHLESS
/*
 * Post divider tarnsition is glitchless only if there is common "1" in binary
 * representation of old and new settings.
 */
static u32 get_interim_pldiv(struct gk20a *g, u32 old_pl, u32 new_pl)
{
	u32 pl;

	if ((g->clk.gpc_pll.id == GM20B_GPC_PLL_C1) || (old_pl & new_pl)) {
		return 0;
	}

	pl = old_pl | BIT(ffs(new_pl) - 1);	/* pl never 0 */
	new_pl |= BIT(ffs(old_pl) - 1);

	return min(pl, new_pl);
}
#endif

/* Calculate and update M/N/PL as well as pll->freq
    ref_clk_f = clk_in_f;
    u_f = ref_clk_f / M;
    vco_f = u_f * N = ref_clk_f * N / M;
    PLL output = gpc2clk = target clock frequency = vco_f / pl_to_pdiv(PL);
    gpcclk = gpc2clk / 2; */
static int clk_config_pll(struct clk_gk20a *clk, struct pll *pll,
	struct pll_parms *pll_params, u32 *target_freq, bool best_fit)
{
	struct gk20a *g = clk->g;
	u32 min_vco_f, max_vco_f;
	u32 best_M, best_N;
	u32 low_PL, high_PL, best_PL;
	u32 m, n, n2;
	u32 target_vco_f, vco_f;
	u32 ref_clk_f, target_clk_f, u_f;
	u32 delta, lwv, best_delta = ~0;
	u32 pl;

	BUG_ON(target_freq == NULL);

	nvgpu_log_fn(g, "request target freq %d MHz", *target_freq);

	ref_clk_f = pll->clk_in;
	target_clk_f = *target_freq;
	max_vco_f = pll_params->max_vco;
	min_vco_f = pll_params->min_vco;
	best_M = pll_params->max_M;
	best_N = pll_params->min_N;
	best_PL = pll_params->min_PL;

	target_vco_f = target_clk_f + target_clk_f / 50;
	if (max_vco_f < target_vco_f) {
		max_vco_f = target_vco_f;
	}

	/* Set PL search boundaries. */
	high_PL = nvgpu_div_to_pl((max_vco_f + target_vco_f - 1) / target_vco_f);
	high_PL = min(high_PL, pll_params->max_PL);
	high_PL = max(high_PL, pll_params->min_PL);

	low_PL = nvgpu_div_to_pl(min_vco_f / target_vco_f);
	low_PL = min(low_PL, pll_params->max_PL);
	low_PL = max(low_PL, pll_params->min_PL);

	nvgpu_log_info(g, "low_PL %d(div%d), high_PL %d(div%d)",
			low_PL, nvgpu_pl_to_div(low_PL), high_PL, nvgpu_pl_to_div(high_PL));

	for (pl = low_PL; pl <= high_PL; pl++) {
		target_vco_f = target_clk_f * nvgpu_pl_to_div(pl);

		for (m = pll_params->min_M; m <= pll_params->max_M; m++) {
			u_f = ref_clk_f / m;

			if (u_f < pll_params->min_u) {
				break;
			}
			if (u_f > pll_params->max_u) {
				continue;
			}

			n = (target_vco_f * m) / ref_clk_f;
			n2 = ((target_vco_f * m) + (ref_clk_f - 1)) / ref_clk_f;

			if (n > pll_params->max_N) {
				break;
			}

			for (; n <= n2; n++) {
				if (n < pll_params->min_N) {
					continue;
				}
				if (n > pll_params->max_N) {
					break;
				}

				vco_f = ref_clk_f * n / m;

				if (vco_f >= min_vco_f && vco_f <= max_vco_f) {
					lwv = (vco_f + (nvgpu_pl_to_div(pl) / 2))
						/ nvgpu_pl_to_div(pl);
					delta = abs((s32)(lwv - target_clk_f));

					if (delta < best_delta) {
						best_delta = delta;
						best_M = m;
						best_N = n;
						best_PL = pl;

						if (best_delta == 0 ||
						    /* 0.45% for non best fit */
						    (!best_fit && (vco_f / best_delta > 218))) {
							goto found_match;
						}

						nvgpu_log_info(g, "delta %d @ M %d, N %d, PL %d",
							delta, m, n, pl);
					}
				}
			}
		}
	}

found_match:
	BUG_ON(best_delta == ~0U);

	if (best_fit && best_delta != 0) {
		gk20a_dbg_clk(g, "no best match for target @ %dMHz on gpc_pll",
			target_clk_f);
	}

	pll->M = best_M;
	pll->N = best_N;
	pll->PL = best_PL;

	/* save current frequency */
	pll->freq = ref_clk_f * pll->N / (pll->M * nvgpu_pl_to_div(pll->PL));

	*target_freq = pll->freq;

	gk20a_dbg_clk(g, "actual target freq %d kHz, M %d, N %d, PL %d(div%d)",
		*target_freq, pll->M, pll->N, pll->PL, nvgpu_pl_to_div(pll->PL));

	nvgpu_log_fn(g, "done");

	return 0;
}

/* GPCPLL NA/DVFS mode methods */

static inline int fuse_get_gpcpll_adc_rev(u32 val)
{
	return (val >> 30) & 0x3;
}

static inline int fuse_get_gpcpll_adc_slope_uv(u32 val)
{
	/*      Integer part in mV  * 1000 + fractional part in uV */
	return ((val >> 24) & 0x3f) * 1000 + ((val >> 14) & 0x3ff);
}

static inline int fuse_get_gpcpll_adc_intercept_uv(u32 val)
{
	/*      Integer part in mV  * 1000 + fractional part in 100uV */
	return ((val >> 4) & 0x3ff) * 1000 + ((val >> 0) & 0xf) * 100;
}

static int nvgpu_fuse_calib_gpcpll_get_adc(struct gk20a *g,
					   int *slope_uv, int *intercept_uv)
{
	u32 val;
	int ret;

	ret = nvgpu_tegra_fuse_read_reserved_calib(g, &val);
	if (ret) {
		return ret;
	}

	if (!fuse_get_gpcpll_adc_rev(val)) {
		return -EINVAL;
	}

	*slope_uv = fuse_get_gpcpll_adc_slope_uv(val);
	*intercept_uv = fuse_get_gpcpll_adc_intercept_uv(val);
	return 0;
}

#ifdef CONFIG_TEGRA_USE_NA_GPCPLL
static bool nvgpu_fuse_can_use_na_gpcpll(struct gk20a *g)
{
	return nvgpu_tegra_get_gpu_speedo_id(g);
}
#endif

/*
 * Read ADC characteristic parmeters from fuses.
 * Determine clibration settings.
 */
static int clk_config_calibration_params(struct gk20a *g)
{
	int slope, offs;
	struct pll_parms *p = &gpc_pll_params;

	if (!nvgpu_fuse_calib_gpcpll_get_adc(g, &slope, &offs)) {
		p->uvdet_slope = slope;
		p->uvdet_offs = offs;
	}

	if (!p->uvdet_slope || !p->uvdet_offs) {
		/*
		 * If ADC conversion slope/offset parameters are not fused
		 * (non-production config), report error, but allow to use
		 * boot internal calibration with default slope.
		 */
		nvgpu_err(g, "ADC coeff are not fused");
		return -EINVAL;
	}
	return 0;
}

/*
 * Determine DFS_COEFF for the requested voltage. Always select external
 * calibration override equal to the voltage, and set maximum detection
 * limit "0" (to make sure that PLL output remains under F/V curve when
 * voltage increases).
 */
static void clk_config_dvfs_detection(int mv, struct na_dvfs *d)
{
	u32 coeff, coeff_max;
	struct pll_parms *p = &gpc_pll_params;

	coeff_max = trim_sys_gpcpll_dvfs0_dfs_coeff_v(
		trim_sys_gpcpll_dvfs0_dfs_coeff_m());
	coeff = DIV_ROUND_CLOSEST(mv * p->coeff_slope, 1000) + p->coeff_offs;
	coeff = DIV_ROUND_CLOSEST(coeff, 1000);
	coeff = min(coeff, coeff_max);
	d->dfs_coeff = coeff;

	d->dfs_ext_cal = DIV_ROUND_CLOSEST(mv * 1000 - p->uvdet_offs,
					   p->uvdet_slope);
	BUG_ON(abs(d->dfs_ext_cal) >= (1 << DFS_DET_RANGE));
	d->uv_cal = p->uvdet_offs + d->dfs_ext_cal * p->uvdet_slope;
	d->dfs_det_max = 0;
}

/*
 * Solve equation for integer and fractional part of the effective NDIV:
 *
 * n_eff = n_int + 1/2 + SDM_DIN / 2^(SDM_DIN_RANGE + 1) +
 * DVFS_COEFF * DVFS_DET_DELTA / 2^DFS_DET_RANGE
 *
 * The SDM_DIN LSB is finally shifted out, since it is not accessible by s/w.
 */
static void clk_config_dvfs_ndiv(int mv, u32 n_eff, struct na_dvfs *d)
{
	int n, det_delta;
	u32 rem, rem_range;
	struct pll_parms *p = &gpc_pll_params;

	det_delta = DIV_ROUND_CLOSEST(mv * 1000 - p->uvdet_offs,
				      p->uvdet_slope);
	det_delta -= d->dfs_ext_cal;
	det_delta = min(det_delta, d->dfs_det_max);
	det_delta = det_delta * d->dfs_coeff;

	n = (int)(n_eff << DFS_DET_RANGE) - det_delta;
	BUG_ON((n < 0) || (n > (int)(p->max_N << DFS_DET_RANGE)));
	d->n_int = ((u32)n) >> DFS_DET_RANGE;

	rem = ((u32)n) & ((1 << DFS_DET_RANGE) - 1);
	rem_range = SDM_DIN_RANGE + 1 - DFS_DET_RANGE;
	d->sdm_din = (rem << rem_range) - (1 << SDM_DIN_RANGE);
	d->sdm_din = (d->sdm_din >> BITS_PER_BYTE) & 0xff;
}

/* Voltage dependent configuration */
static void clk_config_dvfs(struct gk20a *g, struct pll *gpll)
{
	struct na_dvfs *d = &gpll->dvfs;

	d->mv = g->ops.clk.predict_mv_at_hz_cur_tfloor(&g->clk,
			rate_gpc2clk_to_gpu(gpll->freq));

	clk_config_dvfs_detection(d->mv, d);
	clk_config_dvfs_ndiv(d->mv, gpll->N, d);
}

/* Update DVFS detection settings in flight */
static void clk_set_dfs_coeff(struct gk20a *g, u32 dfs_coeff)
{
	u32 data = gk20a_readl(g, trim_gpc_bcast_gpcpll_dvfs2_r());
	data |= DFS_EXT_STROBE;
	gk20a_writel(g, trim_gpc_bcast_gpcpll_dvfs2_r(), data);

	data = gk20a_readl(g, trim_sys_gpcpll_dvfs0_r());
	data = set_field(data, trim_sys_gpcpll_dvfs0_dfs_coeff_m(),
		trim_sys_gpcpll_dvfs0_dfs_coeff_f(dfs_coeff));
	gk20a_writel(g, trim_sys_gpcpll_dvfs0_r(), data);

	data = gk20a_readl(g, trim_gpc_bcast_gpcpll_dvfs2_r());
	nvgpu_udelay(1);
	data &= ~DFS_EXT_STROBE;
	gk20a_writel(g, trim_gpc_bcast_gpcpll_dvfs2_r(), data);
}

static void __maybe_unused clk_set_dfs_det_max(struct gk20a *g, u32 dfs_det_max)
{
	u32 data = gk20a_readl(g, trim_gpc_bcast_gpcpll_dvfs2_r());
	data |= DFS_EXT_STROBE;
	gk20a_writel(g, trim_gpc_bcast_gpcpll_dvfs2_r(), data);

	data = gk20a_readl(g, trim_sys_gpcpll_dvfs0_r());
	data = set_field(data, trim_sys_gpcpll_dvfs0_dfs_det_max_m(),
		trim_sys_gpcpll_dvfs0_dfs_det_max_f(dfs_det_max));
	gk20a_writel(g, trim_sys_gpcpll_dvfs0_r(), data);

	data = gk20a_readl(g, trim_gpc_bcast_gpcpll_dvfs2_r());
	nvgpu_udelay(1);
	data &= ~DFS_EXT_STROBE;
	gk20a_writel(g, trim_gpc_bcast_gpcpll_dvfs2_r(), data);
}

static void clk_set_dfs_ext_cal(struct gk20a *g, u32 dfs_det_cal)
{
	u32 data, ctrl;

	data = gk20a_readl(g, trim_gpc_bcast_gpcpll_dvfs2_r());
	data &= ~(BIT(DFS_DET_RANGE + 1) - 1);
	data |= dfs_det_cal & (BIT(DFS_DET_RANGE + 1) - 1);
	gk20a_writel(g, trim_gpc_bcast_gpcpll_dvfs2_r(), data);

	data = gk20a_readl(g, trim_sys_gpcpll_dvfs1_r());
	nvgpu_udelay(1);
	ctrl = trim_sys_gpcpll_dvfs1_dfs_ctrl_v(data);
	if (~ctrl & DFS_EXT_CAL_EN) {
		data = set_field(data, trim_sys_gpcpll_dvfs1_dfs_ctrl_m(),
			trim_sys_gpcpll_dvfs1_dfs_ctrl_f(
				ctrl | DFS_EXT_CAL_EN | DFS_TESTOUT_DET));
		gk20a_writel(g, trim_sys_gpcpll_dvfs1_r(), data);
	}
}

static void clk_setup_dvfs_detection(struct gk20a *g, struct pll *gpll)
{
	struct na_dvfs *d = &gpll->dvfs;

	u32 data = gk20a_readl(g, trim_gpc_bcast_gpcpll_dvfs2_r());
	data |= DFS_EXT_STROBE;
	gk20a_writel(g, trim_gpc_bcast_gpcpll_dvfs2_r(), data);

	data = gk20a_readl(g, trim_sys_gpcpll_dvfs0_r());
	data = set_field(data, trim_sys_gpcpll_dvfs0_dfs_coeff_m(),
		trim_sys_gpcpll_dvfs0_dfs_coeff_f(d->dfs_coeff));
	data = set_field(data, trim_sys_gpcpll_dvfs0_dfs_det_max_m(),
		trim_sys_gpcpll_dvfs0_dfs_det_max_f(d->dfs_det_max));
	gk20a_writel(g, trim_sys_gpcpll_dvfs0_r(), data);

	data = gk20a_readl(g, trim_gpc_bcast_gpcpll_dvfs2_r());
	nvgpu_udelay(1);
	data &= ~DFS_EXT_STROBE;
	gk20a_writel(g, trim_gpc_bcast_gpcpll_dvfs2_r(), data);

	clk_set_dfs_ext_cal(g, d->dfs_ext_cal);
}

/* Enable NA/DVFS mode */
static int clk_enbale_pll_dvfs(struct gk20a *g)
{
	u32 data, cfg = 0;
	int delay = gpc_pll_params.iddq_exit_delay; /* iddq & calib delay */
	struct pll_parms *p = &gpc_pll_params;
	bool calibrated = p->uvdet_slope && p->uvdet_offs;

	/* Enable NA DVFS */
	data = gk20a_readl(g, trim_sys_gpcpll_dvfs1_r());
	data |= trim_sys_gpcpll_dvfs1_en_dfs_m();
	gk20a_writel(g, trim_sys_gpcpll_dvfs1_r(), data);

	/* Set VCO_CTRL */
	if (p->vco_ctrl) {
		data = gk20a_readl(g, trim_sys_gpcpll_cfg3_r());
		data = set_field(data, trim_sys_gpcpll_cfg3_vco_ctrl_m(),
				 trim_sys_gpcpll_cfg3_vco_ctrl_f(p->vco_ctrl));
		gk20a_writel(g, trim_sys_gpcpll_cfg3_r(), data);
	}

	/* Set NA mode DFS control */
	if (p->dfs_ctrl) {
		data = gk20a_readl(g, trim_sys_gpcpll_dvfs1_r());
		data = set_field(data, trim_sys_gpcpll_dvfs1_dfs_ctrl_m(),
			trim_sys_gpcpll_dvfs1_dfs_ctrl_f(p->dfs_ctrl));
		gk20a_writel(g, trim_sys_gpcpll_dvfs1_r(), data);
	}

	/*
	 * If calibration parameters are known (either from fuses, or from
	 * internal calibration on boot) - use them. Internal calibration is
	 * started anyway; it will complete, but results will not be used.
	 */
	if (calibrated) {
		data = gk20a_readl(g, trim_sys_gpcpll_dvfs1_r());
		data |= trim_sys_gpcpll_dvfs1_en_dfs_cal_m();
		gk20a_writel(g, trim_sys_gpcpll_dvfs1_r(), data);
	}

	/* Exit IDDQ mode */
	data = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	data = set_field(data, trim_sys_gpcpll_cfg_iddq_m(),
			 trim_sys_gpcpll_cfg_iddq_power_on_v());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), data);
	(void) gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	nvgpu_udelay(delay);

	/*
	 * Dynamic ramp setup based on update rate, which in DVFS mode on GM20b
	 * is always 38.4 MHz, the same as reference clock rate.
	 */
	clk_setup_slide(g, g->clk.gpc_pll.clk_in);

	if (calibrated) {
		return 0;
	}

	/*
	 * If calibration parameters are not fused, start internal calibration,
	 * wait for completion, and use results along with default slope to
	 * calculate ADC offset during boot.
	 */
	data = gk20a_readl(g, trim_sys_gpcpll_dvfs1_r());
	data |= trim_sys_gpcpll_dvfs1_en_dfs_cal_m();
	gk20a_writel(g, trim_sys_gpcpll_dvfs1_r(), data);

	/* C1 PLL must be enabled to read internal calibration results */
	if (g->clk.gpc_pll.id == GM20B_GPC_PLL_C1) {
		cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
				trim_sys_gpcpll_cfg_enable_yes_f());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
	}

	/* Wait for internal calibration done (spec < 2us). */
	do {
		data = gk20a_readl(g, trim_sys_gpcpll_dvfs1_r());
		if (trim_sys_gpcpll_dvfs1_dfs_cal_done_v(data)) {
			break;
		}
		nvgpu_udelay(1);
		delay--;
	} while (delay > 0);

	/* Read calibration results */
	data = gk20a_readl(g, trim_sys_gpcpll_cfg3_r());
	data = trim_sys_gpcpll_cfg3_dfs_testout_v(data);

	if (g->clk.gpc_pll.id == GM20B_GPC_PLL_C1) {
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
				trim_sys_gpcpll_cfg_enable_no_f());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
		cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	}

	if (delay <= 0) {
		nvgpu_err(g, "GPCPLL calibration timeout");
		return -ETIMEDOUT;
	}

	p->uvdet_offs = g->clk.pll_poweron_uv - data * ADC_SLOPE_UV;
	p->uvdet_slope = ADC_SLOPE_UV;
	return 0;
}

/* GPCPLL slide methods */
static void clk_setup_slide(struct gk20a *g, u32 clk_u)
{
	u32 data, step_a, step_b;

	switch (clk_u) {
	case 12000:
	case 12800:
	case 13000:			/* only on FPGA */
		step_a = 0x2B;
		step_b = 0x0B;
		break;
	case 19200:
		step_a = 0x12;
		step_b = 0x08;
		break;
	case 38400:
		step_a = 0x04;
		step_b = 0x05;
		break;
	default:
		nvgpu_err(g, "Unexpected reference rate %u kHz", clk_u);
		BUG();
	}

	/* setup */
	data = gk20a_readl(g, trim_sys_gpcpll_cfg2_r());
	data = set_field(data, trim_sys_gpcpll_cfg2_pll_stepa_m(),
			trim_sys_gpcpll_cfg2_pll_stepa_f(step_a));
	gk20a_writel(g, trim_sys_gpcpll_cfg2_r(), data);
	data = gk20a_readl(g, trim_sys_gpcpll_cfg3_r());
	data = set_field(data, trim_sys_gpcpll_cfg3_pll_stepb_m(),
			trim_sys_gpcpll_cfg3_pll_stepb_f(step_b));
	gk20a_writel(g, trim_sys_gpcpll_cfg3_r(), data);
}

static int clk_slide_gpc_pll(struct gk20a *g, struct pll *gpll)
{
	u32 data, coeff;
	u32 nold, sdm_old;
	int ramp_timeout = gpc_pll_params.lock_timeout;

	/* get old coefficients */
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	nold = trim_sys_gpcpll_coeff_ndiv_v(coeff);

	/* do nothing if NDIV is same */
	if (gpll->mode == GPC_PLL_MODE_DVFS) {
		/* in DVFS mode check both integer and fraction */
		coeff = gk20a_readl(g, trim_sys_gpcpll_cfg2_r());
		sdm_old = trim_sys_gpcpll_cfg2_sdm_din_v(coeff);
		if ((gpll->dvfs.n_int == nold) &&
		    (gpll->dvfs.sdm_din == sdm_old)) {
			return 0;
		}
	} else {
		if (gpll->N == nold) {
			return 0;
		}

		/* dynamic ramp setup based on update rate */
		clk_setup_slide(g, gpll->clk_in / gpll->M);
	}

	/* pll slowdown mode */
	data = gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_m(),
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_yes_f());
	gk20a_writel(g, trim_sys_gpcpll_ndiv_slowdown_r(), data);

	/* new ndiv ready for ramp */
	if (gpll->mode == GPC_PLL_MODE_DVFS) {
		/* in DVFS mode SDM is updated via "new" field */
		coeff = gk20a_readl(g, trim_sys_gpcpll_cfg2_r());
		coeff = set_field(coeff, trim_sys_gpcpll_cfg2_sdm_din_new_m(),
			trim_sys_gpcpll_cfg2_sdm_din_new_f(gpll->dvfs.sdm_din));
		gk20a_writel(g, trim_sys_gpcpll_cfg2_r(), coeff);

		coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
		coeff = set_field(coeff, trim_sys_gpcpll_coeff_ndiv_m(),
			trim_sys_gpcpll_coeff_ndiv_f(gpll->dvfs.n_int));
		nvgpu_udelay(1);
		gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);
	} else {
		coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
		coeff = set_field(coeff, trim_sys_gpcpll_coeff_ndiv_m(),
				trim_sys_gpcpll_coeff_ndiv_f(gpll->N));
		nvgpu_udelay(1);
		gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);
	}

	/* dynamic ramp to new ndiv */
	data = gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_m(),
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_yes_f());
	nvgpu_udelay(1);
	gk20a_writel(g, trim_sys_gpcpll_ndiv_slowdown_r(), data);

	do {
		nvgpu_udelay(1);
		ramp_timeout--;
		data = gk20a_readl(
			g, trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_r());
		if (trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_pll_dynramp_done_synced_v(data)) {
			break;
		}
	} while (ramp_timeout > 0);

	if ((gpll->mode == GPC_PLL_MODE_DVFS) && (ramp_timeout > 0)) {
		/* in DVFS mode complete SDM update */
		coeff = gk20a_readl(g, trim_sys_gpcpll_cfg2_r());
		coeff = set_field(coeff, trim_sys_gpcpll_cfg2_sdm_din_m(),
			trim_sys_gpcpll_cfg2_sdm_din_f(gpll->dvfs.sdm_din));
		gk20a_writel(g, trim_sys_gpcpll_cfg2_r(), coeff);
	}

	/* exit slowdown mode */
	data = gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_m(),
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_no_f());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_m(),
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_no_f());
	gk20a_writel(g, trim_sys_gpcpll_ndiv_slowdown_r(), data);
	(void) gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());

	if (ramp_timeout <= 0) {
		nvgpu_err(g, "gpcpll dynamic ramp timeout");
		return -ETIMEDOUT;
	}
	return 0;
}

static void throttle_enable(struct gk20a *g, u32 val)
{
	gk20a_writel(g, therm_use_a_r(), val);
}

static u32 throttle_disable(struct gk20a *g)
{
	u32 val = gk20a_readl(g, therm_use_a_r());
	gk20a_writel(g, therm_use_a_r(), 0);
	return val;
}

/* GPCPLL bypass methods */
static int clk_change_pldiv_under_bypass(struct gk20a *g, struct pll *gpll)
{
	u32 data, coeff, throt;

	/* put PLL in bypass before programming it */
	throt = throttle_disable(g);
	data = gk20a_readl(g, trim_sys_sel_vco_r());
	data = set_field(data, trim_sys_sel_vco_gpc2clk_out_m(),
		trim_sys_sel_vco_gpc2clk_out_bypass_f());
	gk20a_writel(g, trim_sys_sel_vco_r(), data);
	throttle_enable(g, throt);

	/* change PLDIV */
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	nvgpu_udelay(1);
	coeff = set_field(coeff, trim_sys_gpcpll_coeff_pldiv_m(),
			  trim_sys_gpcpll_coeff_pldiv_f(gpll->PL));
	gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);

	/* put PLL back on vco */
	throt = throttle_disable(g);
	data = gk20a_readl(g, trim_sys_sel_vco_r());
	nvgpu_udelay(1);
	data = set_field(data, trim_sys_sel_vco_gpc2clk_out_m(),
		trim_sys_sel_vco_gpc2clk_out_vco_f());
	gk20a_writel(g, trim_sys_sel_vco_r(), data);
	throttle_enable(g, throt);

	return 0;
}

static int clk_lock_gpc_pll_under_bypass(struct gk20a *g, struct pll *gpll)
{
	u32 data, cfg, coeff, timeout, throt;

	/* put PLL in bypass before programming it */
	throt = throttle_disable(g);
	data = gk20a_readl(g, trim_sys_sel_vco_r());
	data = set_field(data, trim_sys_sel_vco_gpc2clk_out_m(),
		trim_sys_sel_vco_gpc2clk_out_bypass_f());
	gk20a_writel(g, trim_sys_sel_vco_r(), data);
	throttle_enable(g, throt);

	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	nvgpu_udelay(1);
	if (trim_sys_gpcpll_cfg_iddq_v(cfg)) {
		/* get out from IDDQ (1st power up) */
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_iddq_m(),
				trim_sys_gpcpll_cfg_iddq_power_on_v());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
		(void) gk20a_readl(g, trim_sys_gpcpll_cfg_r());
		nvgpu_udelay(gpc_pll_params.iddq_exit_delay);
	} else {
		/* clear SYNC_MODE before disabling PLL */
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_sync_mode_m(),
				trim_sys_gpcpll_cfg_sync_mode_disable_f());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
		(void) gk20a_readl(g, trim_sys_gpcpll_cfg_r());

		/* disable running PLL before changing coefficients */
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
				trim_sys_gpcpll_cfg_enable_no_f());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
		(void) gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	}

	/* change coefficients */
	if (gpll->mode == GPC_PLL_MODE_DVFS) {
		clk_setup_dvfs_detection(g, gpll);

		coeff = gk20a_readl(g, trim_sys_gpcpll_cfg2_r());
		coeff = set_field(coeff, trim_sys_gpcpll_cfg2_sdm_din_m(),
			trim_sys_gpcpll_cfg2_sdm_din_f(gpll->dvfs.sdm_din));
		gk20a_writel(g, trim_sys_gpcpll_cfg2_r(), coeff);

		coeff = trim_sys_gpcpll_coeff_mdiv_f(gpll->M) |
			trim_sys_gpcpll_coeff_ndiv_f(gpll->dvfs.n_int) |
			trim_sys_gpcpll_coeff_pldiv_f(gpll->PL);
		gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);
	} else {
		coeff = trim_sys_gpcpll_coeff_mdiv_f(gpll->M) |
			trim_sys_gpcpll_coeff_ndiv_f(gpll->N) |
			trim_sys_gpcpll_coeff_pldiv_f(gpll->PL);
		gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);
	}

	/* enable PLL after changing coefficients */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
			trim_sys_gpcpll_cfg_enable_yes_f());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);

	/* just delay in DVFS mode (lock cannot be used) */
	if (gpll->mode == GPC_PLL_MODE_DVFS) {
		(void) gk20a_readl(g, trim_sys_gpcpll_cfg_r());
		nvgpu_udelay(gpc_pll_params.na_lock_delay);
		gk20a_dbg_clk(g, "NA config_pll under bypass: %u (%u) kHz %d mV",
			      gpll->freq, gpll->freq / 2,
			      (trim_sys_gpcpll_cfg3_dfs_testout_v(
				      gk20a_readl(g, trim_sys_gpcpll_cfg3_r()))
			       * gpc_pll_params.uvdet_slope
			       + gpc_pll_params.uvdet_offs) / 1000);
		goto pll_locked;
	}

	/* lock pll */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	if (cfg & trim_sys_gpcpll_cfg_enb_lckdet_power_off_f()){
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_enb_lckdet_m(),
			trim_sys_gpcpll_cfg_enb_lckdet_power_on_f());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
		cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	}

	/* wait pll lock */
	timeout = gpc_pll_params.lock_timeout + 1;
	do {
		nvgpu_udelay(1);
		cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
		if (cfg & trim_sys_gpcpll_cfg_pll_lock_true_f()) {
			goto pll_locked;
		}
	} while (--timeout > 0);

	/* PLL is messed up. What can we do here? */
	dump_gpc_pll(g, gpll, cfg);
	BUG();
	return -EBUSY;

pll_locked:
	gk20a_dbg_clk(g, "locked config_pll under bypass r=0x%x v=0x%x",
		trim_sys_gpcpll_cfg_r(), cfg);

	/* set SYNC_MODE for glitchless switch out of bypass */
	cfg = set_field(cfg, trim_sys_gpcpll_cfg_sync_mode_m(),
			trim_sys_gpcpll_cfg_sync_mode_enable_f());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
	(void) gk20a_readl(g, trim_sys_gpcpll_cfg_r());

	/* put PLL back on vco */
	throt = throttle_disable(g);
	data = gk20a_readl(g, trim_sys_sel_vco_r());
	data = set_field(data, trim_sys_sel_vco_gpc2clk_out_m(),
		trim_sys_sel_vco_gpc2clk_out_vco_f());
	gk20a_writel(g, trim_sys_sel_vco_r(), data);
	throttle_enable(g, throt);

	return 0;
}

/*
 *  Change GPCPLL frequency:
 *  - in legacy (non-DVFS) mode
 *  - in DVFS mode at constant DVFS detection settings, matching current/lower
 *    voltage; the same procedure can be used in this case, since maximum DVFS
 *    detection limit makes sure that PLL output remains under F/V curve when
 *    voltage increases arbitrary.
 */
static int clk_program_gpc_pll(struct gk20a *g, struct pll *gpll_new,
			int allow_slide)
{
	u32 cfg, coeff, data;
	bool can_slide, pldiv_only;
	struct pll gpll;

	nvgpu_log_fn(g, " ");

	if (!nvgpu_platform_is_silicon(g)) {
		return 0;
	}

	/* get old coefficients */
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	gpll.M = trim_sys_gpcpll_coeff_mdiv_v(coeff);
	gpll.N = trim_sys_gpcpll_coeff_ndiv_v(coeff);
	gpll.PL = trim_sys_gpcpll_coeff_pldiv_v(coeff);
	gpll.clk_in = gpll_new->clk_in;

	/* combine target dvfs with old coefficients */
	gpll.dvfs = gpll_new->dvfs;
	gpll.mode = gpll_new->mode;

	/* do NDIV slide if there is no change in M and PL */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	can_slide = allow_slide && trim_sys_gpcpll_cfg_enable_v(cfg);

	if (can_slide && (gpll_new->M == gpll.M) && (gpll_new->PL == gpll.PL)) {
		return clk_slide_gpc_pll(g, gpll_new);
	}

	/* slide down to NDIV_LO */
	if (can_slide) {
		int ret;
		gpll.N = DIV_ROUND_UP(gpll.M * gpc_pll_params.min_vco,
				      gpll.clk_in);
		if (gpll.mode == GPC_PLL_MODE_DVFS) {
			clk_config_dvfs_ndiv(gpll.dvfs.mv, gpll.N, &gpll.dvfs);
		}
		ret = clk_slide_gpc_pll(g, &gpll);
		if (ret) {
			return ret;
		}
	}
	pldiv_only = can_slide && (gpll_new->M == gpll.M);

	/*
	 *  Split FO-to-bypass jump in halfs by setting out divider 1:2.
	 *  (needed even if PLDIV_GLITCHLESS is set, since 1:1 <=> 1:2 direct
	 *  transition is not really glitch-less - see get_interim_pldiv
	 *  function header).
	 */
	if ((gpll_new->PL < 2) || (gpll.PL < 2)) {
		data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
		data = set_field(data, trim_sys_gpc2clk_out_vcodiv_m(),
			trim_sys_gpc2clk_out_vcodiv_f(2));
		gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);
		/* Intentional 2nd write to assure linear divider operation */
		gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);
		(void) gk20a_readl(g, trim_sys_gpc2clk_out_r());
		nvgpu_udelay(2);
	}

#if PLDIV_GLITCHLESS
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	if (pldiv_only) {
		/* Insert interim PLDIV state if necessary */
		u32 interim_pl = get_interim_pldiv(g, gpll_new->PL, gpll.PL);
		if (interim_pl) {
			coeff = set_field(coeff,
				trim_sys_gpcpll_coeff_pldiv_m(),
				trim_sys_gpcpll_coeff_pldiv_f(interim_pl));
			gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);
			coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
		}
		goto set_pldiv;	/* path A: no need to bypass */
	}

	/* path B: bypass if either M changes or PLL is disabled */
#endif
	/*
	 * Program and lock pll under bypass. On exit PLL is out of bypass,
	 * enabled, and locked. VCO is at vco_min if sliding is allowed.
	 * Otherwise it is at VCO target (and therefore last slide call below
	 * is effectively NOP). PL is set to target. Output divider is engaged
	 * at 1:2 if either entry, or exit PL setting is 1:1.
	 */
	gpll = *gpll_new;
	if (allow_slide) {
		gpll.N = DIV_ROUND_UP(gpll_new->M * gpc_pll_params.min_vco,
				      gpll_new->clk_in);
		if (gpll.mode == GPC_PLL_MODE_DVFS) {
			clk_config_dvfs_ndiv(gpll.dvfs.mv, gpll.N, &gpll.dvfs);
		}
	}
	if (pldiv_only) {
		clk_change_pldiv_under_bypass(g, &gpll);
	} else {
		clk_lock_gpc_pll_under_bypass(g, &gpll);
	}

#if PLDIV_GLITCHLESS
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());

set_pldiv:
	/* coeff must be current from either path A or B */
	if (trim_sys_gpcpll_coeff_pldiv_v(coeff) != gpll_new->PL) {
		coeff = set_field(coeff, trim_sys_gpcpll_coeff_pldiv_m(),
			trim_sys_gpcpll_coeff_pldiv_f(gpll_new->PL));
		gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);
	}
#endif
	/* restore out divider 1:1 */
	data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	if ((data & trim_sys_gpc2clk_out_vcodiv_m()) !=
	    trim_sys_gpc2clk_out_vcodiv_by1_f()) {
		data = set_field(data, trim_sys_gpc2clk_out_vcodiv_m(),
				 trim_sys_gpc2clk_out_vcodiv_by1_f());
		nvgpu_udelay(2);
		gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);
		/* Intentional 2nd write to assure linear divider operation */
		gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);
		(void) gk20a_readl(g, trim_sys_gpc2clk_out_r());
	}

	/* slide up to target NDIV */
	return clk_slide_gpc_pll(g, gpll_new);
}

/* Find GPCPLL config safe at DVFS coefficient = 0, matching target frequency */
static void clk_config_pll_safe_dvfs(struct gk20a *g, struct pll *gpll)
{
	u32 nsafe, nmin;

	if (gpll->freq > g->clk.dvfs_safe_max_freq) {
		gpll->freq = gpll->freq * (100 - DVFS_SAFE_MARGIN) / 100;
	}

	nmin = DIV_ROUND_UP(gpll->M * gpc_pll_params.min_vco, gpll->clk_in);
	nsafe = gpll->M * gpll->freq / gpll->clk_in;

	/*
	 * If safe frequency is above VCOmin, it can be used in safe PLL config
	 * as is. Since safe frequency is below both old and new frequencies,
	 * in this case all three configurations have same post divider 1:1, and
	 * direct old=>safe=>new n-sliding will be used for transitions.
	 *
	 * Otherwise, if safe frequency is below VCO min, post-divider in safe
	 * configuration (and possibly in old and/or new configurations) is
	 * above 1:1, and each old=>safe and safe=>new transitions includes
	 * sliding to/from VCOmin, as well as divider changes. To avoid extra
	 * dynamic ramps from VCOmin during old=>safe transition and to VCOmin
	 * during safe=>new transition, select nmin as safe NDIV, and set safe
	 * post divider to assure PLL output is below safe frequency
	 */
	if (nsafe < nmin) {
		gpll->PL = DIV_ROUND_UP(nmin * gpll->clk_in,
					gpll->M * gpll->freq);
		nsafe = nmin;
	}
	gpll->N = nsafe;
	clk_config_dvfs_ndiv(gpll->dvfs.mv, gpll->N, &gpll->dvfs);

	gk20a_dbg_clk(g, "safe freq %d kHz, M %d, N %d, PL %d(div%d), mV(cal) %d(%d), DC %d",
		gpll->freq, gpll->M, gpll->N, gpll->PL, nvgpu_pl_to_div(gpll->PL),
		gpll->dvfs.mv, gpll->dvfs.uv_cal / 1000, gpll->dvfs.dfs_coeff);
}

/* Change GPCPLL frequency and DVFS detection settings in DVFS mode */
static int clk_program_na_gpc_pll(struct gk20a *g, struct pll *gpll_new,
				  int allow_slide)
{
	int ret;
	struct pll gpll_safe;
	struct pll *gpll_old = &g->clk.gpc_pll_last;

	BUG_ON(gpll_new->M != 1);	/* the only MDIV in NA mode  */
	clk_config_dvfs(g, gpll_new);

	/*
	 * In cases below no intermediate steps in PLL DVFS configuration are
	 * necessary because either
	 * - PLL DVFS will be configured under bypass directly to target, or
	 * - voltage is not changing, so DVFS detection settings are the same
	 */
	if (!allow_slide || !gpll_new->enabled ||
	    (gpll_old->dvfs.mv == gpll_new->dvfs.mv)) {
		return clk_program_gpc_pll(g, gpll_new, allow_slide);
	}

	/*
	 * Interim step for changing DVFS detection settings: low enough
	 * frequency to be safe at at DVFS coeff = 0.
	 *
	 * 1. If voltage is increasing:
	 * - safe frequency target matches the lowest - old - frequency
	 * - DVFS settings are still old
	 * - Voltage already increased to new level by tegra DVFS, but maximum
	 *    detection limit assures PLL output remains under F/V curve
	 *
	 * 2. If voltage is decreasing:
	 * - safe frequency target matches the lowest - new - frequency
	 * - DVFS settings are still old
	 * - Voltage is also old, it will be lowered by tegra DVFS afterwards
	 *
	 * Interim step can be skipped if old frequency is below safe minimum,
	 * i.e., it is low enough to be safe at any voltage in operating range
	 * with zero DVFS coefficient.
	 */
	if (gpll_old->freq > g->clk.dvfs_safe_max_freq) {
		if (gpll_old->dvfs.mv < gpll_new->dvfs.mv) {
			gpll_safe = *gpll_old;
			gpll_safe.dvfs.mv = gpll_new->dvfs.mv;
		} else {
			gpll_safe = *gpll_new;
			gpll_safe.dvfs = gpll_old->dvfs;
		}
		clk_config_pll_safe_dvfs(g, &gpll_safe);

		ret = clk_program_gpc_pll(g, &gpll_safe, 1);
		if (ret) {
			nvgpu_err(g, "Safe dvfs program fail");
			return ret;
		}
	}

	/*
	 * DVFS detection settings transition:
	 * - Set DVFS coefficient zero (safe, since already at frequency safe
	 *   at DVFS coeff = 0 for the lowest of the old/new end-points)
	 * - Set calibration level to new voltage (safe, since DVFS coeff = 0)
	 * - Set DVFS coefficient to match new voltage (safe, since already at
	 *   frequency safe at DVFS coeff = 0 for the lowest of the old/new
	 *   end-points.
	 */
	clk_set_dfs_coeff(g, 0);
	clk_set_dfs_ext_cal(g, gpll_new->dvfs.dfs_ext_cal);
	clk_set_dfs_coeff(g, gpll_new->dvfs.dfs_coeff);

	gk20a_dbg_clk(g, "config_pll  %d kHz, M %d, N %d, PL %d(div%d), mV(cal) %d(%d), DC %d",
		gpll_new->freq, gpll_new->M, gpll_new->N, gpll_new->PL,
		nvgpu_pl_to_div(gpll_new->PL),
		max(gpll_new->dvfs.mv, gpll_old->dvfs.mv),
		gpll_new->dvfs.uv_cal / 1000, gpll_new->dvfs.dfs_coeff);

	/* Finally set target rate (with DVFS detection settings already new) */
	return clk_program_gpc_pll(g, gpll_new, 1);
}

static int clk_disable_gpcpll(struct gk20a *g, int allow_slide)
{
	u32 cfg, coeff, throt;
	struct clk_gk20a *clk = &g->clk;
	struct pll gpll = clk->gpc_pll;

	/* slide to VCO min */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	if (allow_slide && trim_sys_gpcpll_cfg_enable_v(cfg)) {
		coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
		gpll.M = trim_sys_gpcpll_coeff_mdiv_v(coeff);
		gpll.N = DIV_ROUND_UP(gpll.M * gpc_pll_params.min_vco,
				      gpll.clk_in);
		if (gpll.mode == GPC_PLL_MODE_DVFS) {
			clk_config_dvfs_ndiv(gpll.dvfs.mv, gpll.N, &gpll.dvfs);
		}
		clk_slide_gpc_pll(g, &gpll);
	}

	/* put PLL in bypass before disabling it */
	throt = throttle_disable(g);
	cfg = gk20a_readl(g, trim_sys_sel_vco_r());
	cfg = set_field(cfg, trim_sys_sel_vco_gpc2clk_out_m(),
			trim_sys_sel_vco_gpc2clk_out_bypass_f());
	gk20a_writel(g, trim_sys_sel_vco_r(), cfg);
	throttle_enable(g, throt);

	/* clear SYNC_MODE before disabling PLL */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	cfg = set_field(cfg, trim_sys_gpcpll_cfg_sync_mode_m(),
			trim_sys_gpcpll_cfg_sync_mode_disable_f());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);

	/* disable PLL */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
			trim_sys_gpcpll_cfg_enable_no_f());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
	(void) gk20a_readl(g, trim_sys_gpcpll_cfg_r());

	clk->gpc_pll.enabled = false;
	clk->gpc_pll_last.enabled = false;
	return 0;
}

struct pll_parms *gm20b_get_gpc_pll_parms(void)
{
	return &gpc_pll_params;
}

int gm20b_init_clk_setup_sw(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	unsigned long safe_rate;
	int err;

	nvgpu_log_fn(g, " ");

	err = nvgpu_mutex_init(&clk->clk_mutex);
	if (err) {
		return err;
	}

	if (clk->sw_ready) {
		nvgpu_log_fn(g, "skip init");
		return 0;
	}

	if (clk->gpc_pll.id == GM20B_GPC_PLL_C1) {
		gpc_pll_params = gpc_pll_params_c1;
		if (!clk->pll_poweron_uv) {
			clk->pll_poweron_uv = BOOT_GPU_UV_C1;
		}
	} else {
		gpc_pll_params = gpc_pll_params_b1;
		if (!clk->pll_poweron_uv) {
			clk->pll_poweron_uv = BOOT_GPU_UV_B1;
		}
	}

	clk->gpc_pll.clk_in = g->ops.clk.get_ref_clock_rate(g) / KHZ;
	if (clk->gpc_pll.clk_in == 0) {
		nvgpu_err(g, "GPCPLL reference clock is zero");
		err = -EINVAL;
		goto fail;
	}

	safe_rate = g->ops.clk.get_fmax_at_vmin_safe(g);
	safe_rate = safe_rate * (100 - DVFS_SAFE_MARGIN) / 100;
	clk->dvfs_safe_max_freq = rate_gpu_to_gpc2clk(safe_rate);
	clk->gpc_pll.PL = (clk->dvfs_safe_max_freq == 0) ? 0 :
		DIV_ROUND_UP(gpc_pll_params.min_vco, clk->dvfs_safe_max_freq);

	/* Initial freq: low enough to be safe at Vmin (default 1/3 VCO min) */
	clk->gpc_pll.M = 1;
	clk->gpc_pll.N = DIV_ROUND_UP(gpc_pll_params.min_vco,
				clk->gpc_pll.clk_in);
	clk->gpc_pll.PL = max(clk->gpc_pll.PL, 3U);
	clk->gpc_pll.freq = clk->gpc_pll.clk_in * clk->gpc_pll.N;
	clk->gpc_pll.freq /= nvgpu_pl_to_div(clk->gpc_pll.PL);

	 /*
	  * All production parts should have ADC fuses burnt. Therefore, check
	  * ADC fuses always, regardless of whether NA mode is selected; and if
	  * NA mode is indeed selected, and part can support it, switch to NA
	  * mode even when ADC calibration is not fused; less accurate s/w
	  * self-calibration will be used for those parts.
	  */
	clk_config_calibration_params(g);
#ifdef CONFIG_TEGRA_USE_NA_GPCPLL
	if (nvgpu_fuse_can_use_na_gpcpll(g)) {
		/* NA mode is supported only at max update rate 38.4 MHz */
		BUG_ON(clk->gpc_pll.clk_in != gpc_pll_params.max_u);
		clk->gpc_pll.mode = GPC_PLL_MODE_DVFS;
		gpc_pll_params.min_u = gpc_pll_params.max_u;
	}
#endif

	clk->sw_ready = true;

	nvgpu_log_fn(g, "done");
	nvgpu_info(g,
		"GPCPLL initial settings:%s M=%u, N=%u, P=%u (id = %u)",
		clk->gpc_pll.mode == GPC_PLL_MODE_DVFS ? " NA mode," : "",
		clk->gpc_pll.M, clk->gpc_pll.N, clk->gpc_pll.PL,
		clk->gpc_pll.id);
	return 0;

fail:
	nvgpu_mutex_destroy(&clk->clk_mutex);
	return err;
}


static int set_pll_freq(struct gk20a *g, int allow_slide);
static int set_pll_target(struct gk20a *g, u32 freq, u32 old_freq);

int gm20b_clk_prepare(struct clk_gk20a *clk)
{
	int ret = 0;

	nvgpu_mutex_acquire(&clk->clk_mutex);
	if (!clk->gpc_pll.enabled && clk->clk_hw_on) {
		ret = set_pll_freq(clk->g, 1);
	}
	nvgpu_mutex_release(&clk->clk_mutex);
	return ret;
}

void gm20b_clk_unprepare(struct clk_gk20a *clk)
{
	nvgpu_mutex_acquire(&clk->clk_mutex);
	if (clk->gpc_pll.enabled && clk->clk_hw_on) {
		clk_disable_gpcpll(clk->g, 1);
	}
	nvgpu_mutex_release(&clk->clk_mutex);
}

int gm20b_clk_is_prepared(struct clk_gk20a *clk)
{
	return clk->gpc_pll.enabled && clk->clk_hw_on;
}

unsigned long gm20b_recalc_rate(struct clk_gk20a *clk, unsigned long parent_rate)
{
	return rate_gpc2clk_to_gpu(clk->gpc_pll.freq);
}

int gm20b_gpcclk_set_rate(struct clk_gk20a *clk, unsigned long rate,
				 unsigned long parent_rate)
{
	u32 old_freq;
	int ret = -ENODATA;

	nvgpu_mutex_acquire(&clk->clk_mutex);
	old_freq = clk->gpc_pll.freq;
	ret = set_pll_target(clk->g, rate_gpu_to_gpc2clk(rate), old_freq);
	if (!ret && clk->gpc_pll.enabled && clk->clk_hw_on) {
		ret = set_pll_freq(clk->g, 1);
	}
	nvgpu_mutex_release(&clk->clk_mutex);

	return ret;
}

long gm20b_round_rate(struct clk_gk20a *clk, unsigned long rate,
			     unsigned long *parent_rate)
{
	u32 freq;
	struct pll tmp_pll;
	unsigned long maxrate;
	struct gk20a *g = clk->g;

	maxrate = g->ops.clk.get_maxrate(g, CTRL_CLK_DOMAIN_GPCCLK);
	if (rate > maxrate) {
		rate = maxrate;
	}

	nvgpu_mutex_acquire(&clk->clk_mutex);
	freq = rate_gpu_to_gpc2clk(rate);
	if (freq > gpc_pll_params.max_freq) {
		freq = gpc_pll_params.max_freq;
	} else if (freq < gpc_pll_params.min_freq) {
		freq = gpc_pll_params.min_freq;
	}

	tmp_pll = clk->gpc_pll;
	clk_config_pll(clk, &tmp_pll, &gpc_pll_params, &freq, true);
	nvgpu_mutex_release(&clk->clk_mutex);

	return rate_gpc2clk_to_gpu(tmp_pll.freq);
}

static int gm20b_init_clk_setup_hw(struct gk20a *g)
{
	u32 data;

	nvgpu_log_fn(g, " ");

	/* LDIV: Div4 mode (required); both  bypass and vco ratios 1:1 */
	data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	data = set_field(data,
			trim_sys_gpc2clk_out_sdiv14_m() |
			trim_sys_gpc2clk_out_vcodiv_m() |
			trim_sys_gpc2clk_out_bypdiv_m(),
			trim_sys_gpc2clk_out_sdiv14_indiv4_mode_f() |
			trim_sys_gpc2clk_out_vcodiv_by1_f() |
			trim_sys_gpc2clk_out_bypdiv_f(0));
	gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);

	/*
	 * Clear global bypass control; PLL is still under bypass, since SEL_VCO
	 * is cleared by default.
	 */
	data = gk20a_readl(g, trim_sys_bypassctrl_r());
	data = set_field(data, trim_sys_bypassctrl_gpcpll_m(),
			 trim_sys_bypassctrl_gpcpll_vco_f());
	gk20a_writel(g, trim_sys_bypassctrl_r(), data);

	/* If not fused, set RAM SVOP PDP data 0x2, and enable fuse override */
	data = gk20a_readl(g, fuse_ctrl_opt_ram_svop_pdp_r());
	if (!fuse_ctrl_opt_ram_svop_pdp_data_v(data)) {
		data = set_field(data, fuse_ctrl_opt_ram_svop_pdp_data_m(),
			 fuse_ctrl_opt_ram_svop_pdp_data_f(0x2));
		gk20a_writel(g, fuse_ctrl_opt_ram_svop_pdp_r(), data);
		data = gk20a_readl(g, fuse_ctrl_opt_ram_svop_pdp_override_r());
		data = set_field(data,
			fuse_ctrl_opt_ram_svop_pdp_override_data_m(),
			fuse_ctrl_opt_ram_svop_pdp_override_data_yes_f());
		gk20a_writel(g, fuse_ctrl_opt_ram_svop_pdp_override_r(), data);
	}

	/* Disable idle slow down */
	data = gk20a_readl(g, therm_clk_slowdown_r(0));
	data = set_field(data, therm_clk_slowdown_idle_factor_m(),
			 therm_clk_slowdown_idle_factor_disabled_f());
	gk20a_writel(g, therm_clk_slowdown_r(0), data);
	(void) gk20a_readl(g, therm_clk_slowdown_r(0));

	if (g->clk.gpc_pll.mode == GPC_PLL_MODE_DVFS) {
		return clk_enbale_pll_dvfs(g);
	}

	return 0;
}

static int set_pll_target(struct gk20a *g, u32 freq, u32 old_freq)
{
	struct clk_gk20a *clk = &g->clk;

	if (freq > gpc_pll_params.max_freq) {
		freq = gpc_pll_params.max_freq;
	} else if (freq < gpc_pll_params.min_freq) {
		freq = gpc_pll_params.min_freq;
	}

	if (freq != old_freq) {
		/* gpc_pll.freq is changed to new value here */
		if (clk_config_pll(clk, &clk->gpc_pll, &gpc_pll_params,
				   &freq, true)) {
			nvgpu_err(g, "failed to set pll target for %d", freq);
			return -EINVAL;
		}
	}
	return 0;
}

static int set_pll_freq(struct gk20a *g, int allow_slide)
{
	struct clk_gk20a *clk = &g->clk;
	int err = 0;

	nvgpu_log_fn(g, "last freq: %dMHz, target freq %dMHz",
		     clk->gpc_pll_last.freq, clk->gpc_pll.freq);

	/* If programming with dynamic sliding failed, re-try under bypass */
	if (clk->gpc_pll.mode == GPC_PLL_MODE_DVFS) {
		err = clk_program_na_gpc_pll(g, &clk->gpc_pll, allow_slide);
		if (err && allow_slide) {
			err = clk_program_na_gpc_pll(g, &clk->gpc_pll, 0);
		}
	} else {
		err = clk_program_gpc_pll(g, &clk->gpc_pll, allow_slide);
		if (err && allow_slide) {
			err = clk_program_gpc_pll(g, &clk->gpc_pll, 0);
		}
	}

	if (!err) {
		clk->gpc_pll.enabled = true;
		clk->gpc_pll_last = clk->gpc_pll;
		return 0;
	}

	/*
	 * Just report error but not restore PLL since dvfs could already change
	 * voltage even when programming failed.
	 */
	nvgpu_err(g, "failed to set pll to %d", clk->gpc_pll.freq);
	return err;
}

int gm20b_init_clk_support(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	u32 err;

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&clk->clk_mutex);
	clk->clk_hw_on = true;

	err = gm20b_init_clk_setup_hw(g);
	nvgpu_mutex_release(&clk->clk_mutex);
	if (err) {
		return err;
	}

	/* FIXME: this effectively prevents host level clock gating */
	err = g->ops.clk.prepare_enable(&g->clk);
	if (err) {
		return err;
	}

	/* The prev call may not enable PLL if gbus is unbalanced - force it */
	nvgpu_mutex_acquire(&clk->clk_mutex);
	if (!clk->gpc_pll.enabled) {
		err = set_pll_freq(g, 1);
	}
	nvgpu_mutex_release(&clk->clk_mutex);

	return err;
}

int gm20b_suspend_clk_support(struct gk20a *g)
{
	int ret = 0;

	g->ops.clk.disable_unprepare(&g->clk);

	/* The prev call may not disable PLL if gbus is unbalanced - force it */
	nvgpu_mutex_acquire(&g->clk.clk_mutex);
	if (g->clk.gpc_pll.enabled) {
		ret = clk_disable_gpcpll(g, 1);
	}
	g->clk.clk_hw_on = false;
	nvgpu_mutex_release(&g->clk.clk_mutex);

	nvgpu_mutex_destroy(&g->clk.clk_mutex);

	return ret;
}

int gm20b_clk_get_voltage(struct clk_gk20a *clk, u64 *val)
{
	struct gk20a *g = clk->g;
	struct pll_parms *gpc_pll_params = gm20b_get_gpc_pll_parms();
	u32 det_out;
	int err;

	if (clk->gpc_pll.mode != GPC_PLL_MODE_DVFS) {
		return -ENOSYS;
	}

	err = gk20a_busy(g);
	if (err) {
		return err;
	}

	nvgpu_mutex_acquire(&g->clk.clk_mutex);

	det_out = gk20a_readl(g, trim_sys_gpcpll_cfg3_r());
	det_out = trim_sys_gpcpll_cfg3_dfs_testout_v(det_out);
	*val = div64_u64((u64)det_out * gpc_pll_params->uvdet_slope +
		gpc_pll_params->uvdet_offs, 1000ULL);

	nvgpu_mutex_release(&g->clk.clk_mutex);

	gk20a_idle(g);
	return 0;
}

int gm20b_clk_get_gpcclk_clock_counter(struct clk_gk20a *clk, u64 *val)
{
	struct gk20a *g = clk->g;
	u32 clk_slowdown, clk_slowdown_save;
	int err;

	u32 ncycle = 800; /* count GPCCLK for ncycle of clkin */
	u64 freq = clk->gpc_pll.clk_in;
	u32 count1, count2;

	err = gk20a_busy(g);
	if (err) {
		return err;
	}

	nvgpu_mutex_acquire(&g->clk.clk_mutex);

	/* Disable clock slowdown during measurements */
	clk_slowdown_save = gk20a_readl(g, therm_clk_slowdown_r(0));
	clk_slowdown = set_field(clk_slowdown_save,
				 therm_clk_slowdown_idle_factor_m(),
				 therm_clk_slowdown_idle_factor_disabled_f());
	gk20a_writel(g, therm_clk_slowdown_r(0), clk_slowdown);
	(void) gk20a_readl(g, therm_clk_slowdown_r(0));

	gk20a_writel(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0),
		     trim_gpc_clk_cntr_ncgpcclk_cfg_reset_asserted_f());
	gk20a_writel(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0),
		     trim_gpc_clk_cntr_ncgpcclk_cfg_enable_asserted_f() |
		     trim_gpc_clk_cntr_ncgpcclk_cfg_write_en_asserted_f() |
		     trim_gpc_clk_cntr_ncgpcclk_cfg_noofipclks_f(ncycle));
	/* start */

	/* It should take less than 25us to finish 800 cycle of 38.4MHz.
	 *  But longer than 100us delay is required here.
	 */
	(void) gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0));
	nvgpu_udelay(200);

	count1 = gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cnt_r(0));
	nvgpu_udelay(100);
	count2 = gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cnt_r(0));
	freq *= trim_gpc_clk_cntr_ncgpcclk_cnt_value_v(count2);
	do_div(freq, ncycle);
	*val = freq;

	/* Restore clock slowdown */
	gk20a_writel(g, therm_clk_slowdown_r(0), clk_slowdown_save);
	nvgpu_mutex_release(&g->clk.clk_mutex);

	gk20a_idle(g);

	if (count1 != count2) {
		return -EBUSY;
	}

	return 0;
}

int gm20b_clk_pll_reg_write(struct gk20a *g, u32 reg, u32 val)
{
	if (((reg < trim_sys_gpcpll_cfg_r()) ||
	    (reg > trim_sys_gpcpll_dvfs2_r())) &&
	    (reg != trim_sys_sel_vco_r()) &&
	    (reg != trim_sys_gpc2clk_out_r()) &&
	    (reg != trim_sys_bypassctrl_r())) {
		return -EPERM;
	}

	if (reg == trim_sys_gpcpll_dvfs2_r()) {
		reg = trim_gpc_bcast_gpcpll_dvfs2_r();
	}

	nvgpu_mutex_acquire(&g->clk.clk_mutex);
	if (!g->clk.clk_hw_on) {
		nvgpu_mutex_release(&g->clk.clk_mutex);
		return -EINVAL;
	}
	gk20a_writel(g, reg, val);
	nvgpu_mutex_release(&g->clk.clk_mutex);

	return 0;
}

int gm20b_clk_get_pll_debug_data(struct gk20a *g,
			struct nvgpu_clk_pll_debug_data *d)
{
	u32 reg;

	nvgpu_mutex_acquire(&g->clk.clk_mutex);
	if (!g->clk.clk_hw_on) {
		nvgpu_mutex_release(&g->clk.clk_mutex);
		return -EINVAL;
	}

	d->trim_sys_bypassctrl_reg = trim_sys_bypassctrl_r();
	d->trim_sys_bypassctrl_val = gk20a_readl(g, trim_sys_bypassctrl_r());
	d->trim_sys_sel_vco_reg = trim_sys_sel_vco_r();
	d->trim_sys_sel_vco_val = gk20a_readl(g, trim_sys_sel_vco_r());
	d->trim_sys_gpc2clk_out_reg = trim_sys_gpc2clk_out_r();
	d->trim_sys_gpc2clk_out_val = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	d->trim_sys_gpcpll_cfg_reg = trim_sys_gpcpll_cfg_r();
	d->trim_sys_gpcpll_dvfs2_reg = trim_sys_gpcpll_dvfs2_r();
	d->trim_bcast_gpcpll_dvfs2_reg = trim_gpc_bcast_gpcpll_dvfs2_r();

	reg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	d->trim_sys_gpcpll_cfg_val = reg;
	d->trim_sys_gpcpll_cfg_enabled = trim_sys_gpcpll_cfg_enable_v(reg);
	d->trim_sys_gpcpll_cfg_locked = trim_sys_gpcpll_cfg_pll_lock_v(reg);
	d->trim_sys_gpcpll_cfg_sync_on = trim_sys_gpcpll_cfg_sync_mode_v(reg);

	reg = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	d->trim_sys_gpcpll_coeff_val = reg;
	d->trim_sys_gpcpll_coeff_mdiv = trim_sys_gpcpll_coeff_mdiv_v(reg);
	d->trim_sys_gpcpll_coeff_ndiv = trim_sys_gpcpll_coeff_ndiv_v(reg);
	d->trim_sys_gpcpll_coeff_pldiv = trim_sys_gpcpll_coeff_pldiv_v(reg);

	reg = gk20a_readl(g, trim_sys_gpcpll_dvfs0_r());
	d->trim_sys_gpcpll_dvfs0_val = reg;
	d->trim_sys_gpcpll_dvfs0_dfs_coeff =
			trim_sys_gpcpll_dvfs0_dfs_coeff_v(reg);
	d->trim_sys_gpcpll_dvfs0_dfs_det_max =
			trim_sys_gpcpll_dvfs0_dfs_det_max_v(reg);
	d->trim_sys_gpcpll_dvfs0_dfs_dc_offset =
			trim_sys_gpcpll_dvfs0_dfs_dc_offset_v(reg);

	nvgpu_mutex_release(&g->clk.clk_mutex);
	return 0;
}
