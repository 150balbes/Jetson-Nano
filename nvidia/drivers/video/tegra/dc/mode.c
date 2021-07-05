/*
 * mode.c: dc and fb mode functions.
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Copyright (c) 2010-2019, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/err.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/clk/tegra.h>

#include <trace/events/display.h>

#include <linux/platform/tegra/mc.h>

#include "dc.h"
#include "dc_reg.h"
#include "dc_priv.h"
#include "dsi.h"
#include "edid.h"

#define CHK(c, s, ...) do {					\
		if (unlikely(c)) {				\
			if (verbose)				\
				pr_err(s, ## __VA_ARGS__);	\
			return false;				\
		}						\
	} while (0)

/* return non-zero if constraint is violated */
static int calc_h_ref_to_sync(const struct tegra_dc_mode *mode, int *href)
{
	long a, b;

	/* Constraint 5: H_REF_TO_SYNC >= 0 */
	a = 0;

	/* Constraint 6: H_FRONT_PORT >= (H_REF_TO_SYNC + 1) */
	b = mode->h_front_porch - 1;

	/* Constraint 1: H_REF_TO_SYNC + H_SYNC_WIDTH + H_BACK_PORCH > 20 */
	if (a + mode->h_sync_width + mode->h_back_porch <= 20)
		a = 1 + 20 - mode->h_sync_width - mode->h_back_porch;
	/* check Constraint 1 and 6 */
	if (a > b)
		return 1;

	/* Constraint 4: H_SYNC_WIDTH >= 1 */
	if (mode->h_sync_width < 1)
		return 4;

	/* Constraint 7: H_DISP_ACTIVE >= 16 */
	if (mode->h_active < 16)
		return 7;

	if (href) {
		if (b > a && a % 2)
			*href = a + 1; /* use smallest even value */
		else
			*href = a; /* even or only possible value */
	}

	return 0;
}

static int calc_v_ref_to_sync(const struct tegra_dc_mode *mode, int *vref)
{
	long a;

	/* Constraint 5: V_REF_TO_SYNC >= 1 */
	a = mode->v_front_porch - 1;

	/* Constraint 2: V_REF_TO_SYNC + V_SYNC_WIDTH + V_BACK_PORCH > 1 */
	if (a + mode->v_sync_width + mode->v_back_porch <= 1)
		a = 1 + 1 - mode->v_sync_width - mode->v_back_porch;

	/* Constraint 6 */
	if (mode->v_front_porch < a + 1)
		a = mode->v_front_porch - 1;

	/* Constraint 4: V_SYNC_WIDTH >= 1 */
	if (mode->v_sync_width < 1)
		return 4;

	/* Constraint 7: V_DISP_ACTIVE >= 16 */
	if (mode->v_active < 16)
		return 7;

	if (vref)
		*vref = a;
	return 0;
}

static int calc_ref_to_sync(struct tegra_dc_mode *mode)
{
	int ret;
	ret = calc_h_ref_to_sync(mode, &mode->h_ref_to_sync);
	if (ret)
		return ret;
	ret = calc_v_ref_to_sync(mode, &mode->v_ref_to_sync);
	if (ret)
		return ret;

	return 0;
}

static bool check_ref_to_sync(struct tegra_dc_mode *mode, bool verbose)
{
	/* Constraint 1: H_REF_TO_SYNC + H_SYNC_WIDTH + H_BACK_PORCH > 20. */
	CHK(mode->h_ref_to_sync + mode->h_sync_width +
	    mode->h_back_porch <= 20,
	    "H_REF_TO_SYNC + H_SYNC_WIDTH + H_BACK_PORCH <= 20\n");

	/* Constraint 2: V_REF_TO_SYNC + V_SYNC_WIDTH + V_BACK_PORCH > 1. */
	CHK(mode->v_ref_to_sync + mode->v_sync_width + mode->v_back_porch <= 1,
	    "V_REF_TO_SYNC + V_SYNC_WIDTH + V_BACK_PORCH <= 1\n");

	/* Constraint 3: V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH > 1
	 * (vertical blank). */
	CHK(mode->v_front_porch + mode->v_sync_width + mode->v_back_porch <= 1,
	    "V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH <= 1\n");


	/* Constraint 4: V_SYNC_WIDTH >= 1; H_SYNC_WIDTH >= 1. */
	CHK(mode->v_sync_width < 1 || mode->h_sync_width < 1,
	    "V_SYNC_WIDTH >= 1; H_SYNC_WIDTH < 1\n");

	/* Constraint 5: V_REF_TO_SYNC >= 1; H_REF_TO_SYNC >= 0. */
	CHK(mode->v_ref_to_sync < 1 || mode->h_ref_to_sync < 0,
	    "V_REF_TO_SYNC >= 1; H_REF_TO_SYNC < 0\n");

	/* Constraint 6: V_FRONT_PORCH >= (V_REF_TO_SYNC + 1) */
	CHK(mode->v_front_porch < mode->v_ref_to_sync + 1,
	    "V_FRONT_PORCH < (V_REF_TO_SYNC + 1)\n");

	/* Constraint 7: H_FRONT_PORCH >= (H_REF_TO_SYNC + 1) */
	CHK(mode->h_front_porch < mode->h_ref_to_sync + 1,
	    "H_FRONT_PORCH < (H_REF_TO_SYNC + 1)\n");

	/* Constraint 8: H_DISP_ACTIVE >= 16 */
	CHK(mode->h_active < 16, "H_DISP_ACTIVE < 16\n");

	/* Constraint 9: V_DISP_ACTIVE >= 16 */
	CHK(mode->v_active < 16, "V_DISP_ACTIVE < 16\n");

	return true;
}

static s64 calc_frametime_ns(const struct tegra_dc_mode *m)
{
	long h_total, v_total;
	h_total = m->h_active + m->h_front_porch + m->h_back_porch +
		m->h_sync_width;
	v_total = m->v_active + m->v_front_porch + m->v_back_porch +
		m->v_sync_width;
	return (!m->pclk) ? 0 : (s64)(div_s64(((s64)h_total * v_total *
					1000000000ULL), m->pclk));
}

/*
 * return in 1000ths of a Hertz
 * TODO: Extend to handle other refresh rates and pclk
 */
static inline int _tegra_dc_calc_refresh(long pclk, long h_total, long v_total)
{
	long refresh;

	if (!pclk || !h_total || !v_total || pclk < h_total)
		return 0;

	refresh = pclk / h_total;
	refresh *= 1000;
	refresh /= v_total;

	return refresh;
}

/* return in 1000ths of a Hertz */
int tegra_dc_calc_refresh(const struct tegra_dc_mode *m)
{
	long h_total, v_total;
	long pclk;

	if (m->rated_pclk > 0)
		pclk = m->rated_pclk;
	else
		pclk = m->pclk;

	h_total = m->h_active + m->h_front_porch + m->h_back_porch +
		m->h_sync_width;
	v_total = m->v_active + m->v_front_porch + m->v_back_porch +
		m->v_sync_width;

	return _tegra_dc_calc_refresh(pclk, h_total, v_total);
}

/* return in 1000ths of a Hertz */
int tegra_dc_calc_fb_refresh(const struct fb_videomode *fbmode)
{
	long h_total, v_total;

	h_total = fbmode->xres + fbmode->right_margin +
		fbmode->left_margin + fbmode->hsync_len;
	v_total = fbmode->yres + fbmode->upper_margin +
		fbmode->lower_margin + fbmode->vsync_len;

	return _tegra_dc_calc_refresh(PICOS2KHZ(fbmode->pixclock) * 1000,
					h_total, v_total);
}

static u8 _calc_default_avi_m(unsigned h_size, unsigned v_size)
{
#define is_avi_m( \
	h_size, v_size, \
	h_avi_m, v_avi_m) \
	(((h_size) * (v_avi_m)) > ((v_size) * ((h_avi_m) - 1)) &&  \
	((h_size) * (v_avi_m)) < ((v_size) * ((h_avi_m) + 1))) \

	if (!h_size || !v_size)
		pr_warn("invalid h_size %u or v_size %u\n", h_size, v_size);

	if (is_avi_m(h_size, v_size, 256, 135))
		return TEGRA_DC_MODE_AVI_M_256_135;
	else if (is_avi_m(h_size, v_size, 64, 27))
		return TEGRA_DC_MODE_AVI_M_64_27;
	else if (is_avi_m(h_size, v_size, 16, 9))
		return TEGRA_DC_MODE_AVI_M_16_9;
	else if (is_avi_m(h_size, v_size, 4, 3))
		return TEGRA_DC_MODE_AVI_M_4_3;

	return TEGRA_DC_MODE_AVI_M_NO_DATA;

#undef is_avi_m
}

static u8 calc_default_avi_m(struct tegra_dc *dc)
{
#define EDID_AVI_M_256_135 91
#define EDID_AVI_M_64_27 138
#define EDID_AVI_M_16_9 79
#define EDID_AVI_M_4_3 34

	if (dc->out) {
		unsigned h_size = tegra_dc_get_out_width(dc);
		unsigned v_size = tegra_dc_get_out_height(dc);

		/* extract picture aspect ratio from real screen sizes */
		if (h_size && v_size)
			return _calc_default_avi_m(h_size, v_size);

		/* assign edid data */
		h_size = dc->out->h_size;
		v_size = dc->out->v_size;

		if (!h_size && !v_size)
			return TEGRA_DC_MODE_AVI_M_NO_DATA;

		/* edid has picture aspect ratio stored */
		if (!h_size || !v_size) {
			unsigned temp = h_size ? : v_size;

			switch (temp) {
			case EDID_AVI_M_256_135:
				return TEGRA_DC_MODE_AVI_M_256_135;
			case EDID_AVI_M_64_27:
				return TEGRA_DC_MODE_AVI_M_64_27;
			case EDID_AVI_M_16_9:
				return TEGRA_DC_MODE_AVI_M_16_9;
			case EDID_AVI_M_4_3:
				return TEGRA_DC_MODE_AVI_M_4_3;
			default:
				/* unsupported picture aspect ratio */
				return TEGRA_DC_MODE_AVI_M_NO_DATA;
			};
		}
	}

	return 0;

#undef EDID_AVI_M_4_3
#undef EDID_AVI_M_16_9
#undef EDID_AVI_M_64_27
#undef EDID_AVI_M_256_135
}

u32 tegra_dc_get_aspect_ratio(struct tegra_dc *dc)
{
	u32 aspect_ratio = 0;

	if (!dc)
		return 0;

	switch (calc_default_avi_m(dc)) {
	case TEGRA_DC_MODE_AVI_M_256_135:
		aspect_ratio = FB_FLAG_RATIO_256_135;
		break;
	case TEGRA_DC_MODE_AVI_M_64_27:
		aspect_ratio = FB_FLAG_RATIO_64_27;
		break;
	case TEGRA_DC_MODE_AVI_M_16_9:
		aspect_ratio = FB_FLAG_RATIO_16_9;
		break;
	case TEGRA_DC_MODE_AVI_M_4_3:
		aspect_ratio = FB_FLAG_RATIO_4_3;
		break;
	default:
		aspect_ratio = 0;
	};

	return aspect_ratio;
}

static bool check_yuv_timings(struct tegra_dc_mode *mode, bool verbose)
{
	if (mode->vmode & FB_VMODE_BYPASS)
		return true;

	/*
	 * For YUV420/422, the following timing constraints must be enforced:
	 * 1) H_ACTIVE should be even
	 * 2) H_SYNC_WIDTH should be even
	 * 3) H_BACK_PORCH should be even
	 * 4) H_FRONT_PORCH should be even
	 *
	 * For YUV420 only, the following timing constraints must be enforced:
	 * 5) V_ACTIVE should be even
	 */
	if ((mode->vmode & FB_VMODE_Y422) ||
		tegra_dc_is_yuv420_8bpc(mode)) {
		/* Constraint 1 */
		CHK(mode->h_active & 0x1, "H_ACTIVE not even for vmode: 0x%x\n",
			mode->vmode);

		/* Constraint 2 */
		CHK(mode->h_sync_width & 0x1,
			"H_SYNC_WIDTH not even for vmode: 0x%x\n", mode->vmode);

		/* Constraint 3 */
		CHK(mode->h_back_porch & 0x1,
			"H_BACK_PORCH not even for vmode: 0x%x\n", mode->vmode);

		/* Constraint 4 */
		CHK(mode->h_front_porch & 0x1,
			"H_FRONT_PORCH not even for vmode: 0x%x\n",
			mode->vmode);

		/* Constraint 5 */
		if (tegra_dc_is_yuv420_8bpc(mode))
			CHK(mode->v_active & 0x1,
				"V_ACTIVE not even for vmode: 0x%x\n",
				mode->vmode);
	}

	return true;
}

static bool check_t21x_mode_timings(const struct tegra_dc *dc,
			       struct tegra_dc_mode *mode, bool verbose)
{
	if ((mode->vmode & FB_VMODE_Y420) ||
		(mode->vmode & FB_VMODE_Y420_ONLY))
		mode->v_ref_to_sync = 1;
	else
		calc_ref_to_sync(mode);

	mode->h_ref_to_sync = 1;

	if (dc->out->type == TEGRA_DC_OUT_DSI && dc->out->vrr) {
		mode->h_ref_to_sync =
			dc->out->modes[dc->out->n_modes - 1].h_ref_to_sync;
		mode->v_ref_to_sync =
			dc->out->modes[dc->out->n_modes - 1].v_ref_to_sync;
	}

	if (dc->out->type == TEGRA_DC_OUT_DP) {
		mode->h_ref_to_sync = 1;
		mode->v_ref_to_sync = 1;
	}

	if (!check_ref_to_sync(mode, verbose)) {
		if (verbose)
			dev_err(&dc->ndev->dev,
				"Display timing doesn't meet restrictions.\n");
		return false;
	}

	return true;
}

static bool check_nvdisp_mode_timings(const struct tegra_dc *dc,
				struct tegra_dc_mode *mode, bool verbose)
{
	int h_total, v_total;

	/*
	 * Constraint 1: V_SYNC_WIDTH  >= 1
	 *               H_SYNC_WIDTH  >= 1
	 *               V_FRONT_PORCH >= 1
	 *               H_FRONT_PORCH >= 1
	 */
	CHK(mode->v_sync_width < 1 || mode->h_sync_width < 1 ||
		mode->v_front_porch < 1 || mode->h_front_porch < 1,
		"{V,H}_SYNC_WIDTH >= 1, {V,H}_FRONT_PORCH >= 1\n");

	/*
	 * Constraint 2: H_DISP_ACTIVE >= 8
	 *               V_DISP_ACTIVE >= 4
	 */
	CHK(mode->h_active < 8 || mode->v_active < 4,
		"H_DISP_ACTIVE >= 8, V_DISP_ACTIVE >= 4\n");

	/*
	 * Constraint 3: H_TOTAL < 32768
	 *               V_TOTAL < 32768
	 */
	h_total = mode->h_sync_width + mode->h_back_porch + mode->h_active +
		mode->h_front_porch;
	v_total = mode->v_sync_width + mode->v_back_porch + mode->v_active +
		mode->v_front_porch;
	CHK(h_total >= 32768 || v_total >= 32768,
		"H_TOTAL < 32768, V_TOTAL < 32768\n");

	/* Constraint 4: H_BLANK >= 25 */
	CHK(h_total - mode->h_active < 25,
		"H_BLANK >= 25\n");

	return true;
}

static bool check_mode_timings(const struct tegra_dc *dc,
				struct tegra_dc_mode *mode, bool verbose)
{
	bool check;

	if (!check_yuv_timings(mode, verbose))
		return false;

	if (tegra_dc_is_nvdisplay())
		check = check_nvdisp_mode_timings(dc, mode, verbose);
	else
		check = check_t21x_mode_timings(dc, mode, verbose);

	if (verbose)
		dev_dbg(&dc->ndev->dev,
			"Using mode %dx%d pclk=%d href=%d vref=%d\n",
			mode->h_active, mode->v_active, mode->pclk,
			mode->h_ref_to_sync, mode->v_ref_to_sync);

	return check;
}

bool check_fb_videomode_timings(const struct tegra_dc *dc,
				const struct fb_videomode *fbmode)
{
	struct tegra_dc_mode mode;

	if (!dc || !fbmode)
		return false;

	/* Only copy the relevant info */
	mode.h_front_porch = fbmode->right_margin;
	mode.v_front_porch = fbmode->lower_margin;
	mode.h_sync_width = fbmode->hsync_len;
	mode.v_sync_width = fbmode->vsync_len;
	mode.h_back_porch = fbmode->left_margin;
	mode.v_back_porch = fbmode->upper_margin;
	mode.h_active = fbmode->xres;
	mode.v_active = fbmode->yres;
	mode.vmode = fbmode->vmode;

	return check_mode_timings(dc, &mode, false);
}

#ifdef DEBUG
static void print_mode(struct tegra_dc *dc,
			const struct tegra_dc_mode *mode, const char *note)
{
	if (mode) {
		int refresh = tegra_dc_calc_refresh(mode);
		dev_info(&dc->ndev->dev, "%s():MODE:%dx%d@%d.%03uHz pclk=%d\n",
			note ? note : "",
			mode->h_active, mode->v_active,
			refresh / 1000, refresh % 1000,
			mode->pclk);
	}
}
#else /* !DEBUG */
static inline void print_mode(struct tegra_dc *dc,
			const struct tegra_dc_mode *mode, const char *note) { }
#endif /* DEBUG */

int tegra_dc_program_mode(struct tegra_dc *dc, struct tegra_dc_mode *mode)
{
	unsigned long val;
	unsigned long rate;
	unsigned long div;
	unsigned long pclk;

	unsigned long v_back_porch;
	unsigned long v_front_porch;
	unsigned long v_sync_width;
	unsigned long v_active;

	tegra_dc_get(dc);

	if (dc->out_ops && dc->out_ops->modeset_notifier)
		dc->out_ops->modeset_notifier(dc);

	v_back_porch = mode->v_back_porch;
	v_front_porch = mode->v_front_porch;
	v_sync_width = mode->v_sync_width;
	v_active = mode->v_active;

	if (mode->vmode & FB_VMODE_INTERLACED) {
		v_back_porch /= 2;
		v_front_porch /= 2;
		v_sync_width /= 2;
		v_active /= 2;
	}

	print_mode(dc, mode, __func__);

	/* use default EMC rate when switching modes */
#ifdef CONFIG_TEGRA_ISOMGR
	dc->new_bw_kbps = tegra_dc_calc_min_bandwidth(dc);
#else
	dc->new_bw_kbps = tegra_emc_freq_req_to_bw(
		tegra_dc_get_default_emc_clk_rate(dc) / 1000);
#endif
	tegra_dc_program_bandwidth(dc, true);

	tegra_dc_writel(dc, 0x0, DC_DISP_DISP_TIMING_OPTIONS);
	tegra_dc_writel(dc, mode->h_ref_to_sync | (mode->v_ref_to_sync << 16),
			DC_DISP_REF_TO_SYNC);
	tegra_dc_writel(dc, mode->h_sync_width | (v_sync_width << 16),
			DC_DISP_SYNC_WIDTH);
	if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP) ||
		(dc->out->type == TEGRA_DC_OUT_NULL) ||
		(dc->out->type == TEGRA_DC_OUT_LVDS)) {
		tegra_dc_writel(dc, mode->h_back_porch |
			((v_back_porch - mode->v_ref_to_sync) << 16),
			DC_DISP_BACK_PORCH);
		tegra_dc_writel(dc, mode->h_front_porch |
			((v_front_porch + mode->v_ref_to_sync) << 16),
			DC_DISP_FRONT_PORCH);
	} else {
		tegra_dc_writel(dc, mode->h_back_porch |
			(v_back_porch << 16),
			DC_DISP_BACK_PORCH);
		tegra_dc_writel(dc, mode->h_front_porch |
			(v_front_porch << 16),
			DC_DISP_FRONT_PORCH);
	}
	tegra_dc_writel(dc, mode->h_active | (v_active << 16),
			DC_DISP_DISP_ACTIVE);

	if (mode->vmode == FB_VMODE_INTERLACED)
		tegra_dc_writel(dc, INTERLACE_MODE_ENABLE |
			INTERLACE_START_FIELD_1
			| INTERLACE_STATUS_FIELD_1,
			DC_DISP_INTERLACE_CONTROL);
	else
		tegra_dc_writel(dc, INTERLACE_MODE_DISABLE,
			DC_DISP_INTERLACE_CONTROL);

	if (mode->vmode == FB_VMODE_INTERLACED) {
		tegra_dc_writel(dc, (mode->h_ref_to_sync |
			((mode->h_sync_width + mode->h_back_porch +
			mode->h_active + mode->h_front_porch) >> 1)
			<< 16), DC_DISP_INTERLACE_FIELD2_REF_TO_SYNC);
		tegra_dc_writel(dc, mode->h_sync_width |
			(v_sync_width << 16),
			DC_DISP_INTERLACE_FIELD2_SYNC_WIDTH);
		tegra_dc_writel(dc, mode->h_back_porch |
			((v_back_porch + 1) << 16),
			DC_DISP_INTERLACE_FIELD2_BACK_PORCH);
		tegra_dc_writel(dc, mode->h_active |
			(v_active << 16),
			DC_DISP_INTERLACE_FIELD2_DISP_ACTIVE);
		tegra_dc_writel(dc, mode->h_front_porch |
			(v_front_porch << 16),
			DC_DISP_INTERLACE_FIELD2_FRONT_PORCH);
	}

	tegra_dc_writel(dc, DE_SELECT_ACTIVE | DE_CONTROL_NORMAL,
			DC_DISP_DATA_ENABLE_OPTIONS);

	/* TODO: MIPI/CRT/HDMI clock cals */
	val = 0;
	if (!(dc->out->type == TEGRA_DC_OUT_DSI ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIA ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIB ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED ||
		dc->out->type == TEGRA_DC_OUT_HDMI)) {
		val = DISP_DATA_FORMAT_DF1P1C;

		if (dc->out->align == TEGRA_DC_ALIGN_MSB)
			val |= DISP_DATA_ALIGNMENT_MSB;
		else
			val |= DISP_DATA_ALIGNMENT_LSB;

		if (dc->out->order == TEGRA_DC_ORDER_RED_BLUE)
			val |= DISP_DATA_ORDER_RED_BLUE;
		else
			val |= DISP_DATA_ORDER_BLUE_RED;
	}
	tegra_dc_writel(dc, val, DC_DISP_DISP_INTERFACE_CONTROL);

	rate = tegra_dc_clk_get_rate(dc);
	pclk = tegra_dc_pclk_round_rate(dc, mode->pclk);

	if (pclk == 0)  {
		dev_err(&dc->ndev->dev, "pclk is zero!\n");
		return -ERANGE;
	}
	div = (rate * 2 / pclk) - 2;
	dev_info(&dc->ndev->dev,
		"nominal-pclk:%d parent:%lu div:%lu.%lu pclk:%lu %d~%d\n",
		mode->pclk, rate, (div + 2) / 2, ((div + 2) % 2) * 5, pclk,
		mode->pclk / 100 * 99, mode->pclk / 100 * 109);

	/* skip pclk range check for TEGRA_DC_OUT_NULL */
	if (dc->out->type != TEGRA_DC_OUT_NULL) {
		if (!pclk || pclk < (mode->pclk / 100 * 99) ||
			pclk > (mode->pclk / 100 * 109)) {
			dev_err(&dc->ndev->dev, "pclk out of range!\n");
			return -EINVAL;
		}
	}

	tegra_dc_writel(dc, PIXEL_CLK_DIVIDER_PCD1 | SHIFT_CLK_DIVIDER(div),
			DC_DISP_DISP_CLOCK_CONTROL);

#ifdef CONFIG_SWITCH
	if (dc->switchdev_registered)
		switch_set_state(&dc->modeset_switch,
			 (mode->h_active << 16) | mode->v_active);
#endif

	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	if (dc->mode_dirty)
		memcpy(&dc->cached_mode, &dc->mode, sizeof(dc->mode));

	tegra_dc_put(dc);

	dc->mode_dirty = false;

	trace_display_mode(dc, &dc->mode);
	tegra_dc_ext_process_modechange(dc->ndev->id);
	tegra_dc_client_handle_event(dc, NOTIFY_MODESET_EVENT);
	return 0;
}

static int panel_sync_rate;

int tegra_dc_get_panel_sync_rate(void)
{
	return panel_sync_rate;
}
EXPORT_SYMBOL(tegra_dc_get_panel_sync_rate);

int _tegra_dc_set_mode(struct tegra_dc *dc,
				const struct tegra_dc_mode *mode)
{
	struct tegra_dc_mode new_mode = *mode;
	bool yuv_bypass_vmode = false;

	yuv_bypass_vmode = (new_mode.vmode & FB_VMODE_YUV_MASK) &&
				(new_mode.vmode & FB_VMODE_BYPASS);

	/*
	 * On T21x, the bypass flag is exclusively sent as part of the flip, and
	 * not as part of the modeset.
	 */
	if (yuv_bypass_vmode || tegra_dc_is_t21x()) {
		if (tegra_dc_is_yuv420_8bpc(&new_mode)) {
			new_mode.h_back_porch /= 2;
			new_mode.h_front_porch /= 2;
			new_mode.h_sync_width /= 2;
			new_mode.h_active /= 2;
			new_mode.pclk /= 2;
		} else if (tegra_dc_is_yuv420_10bpc(&new_mode)) {
			new_mode.h_back_porch = (new_mode.h_back_porch * 5) / 8;
			new_mode.h_front_porch =
					(new_mode.h_front_porch * 5) / 8;
			new_mode.h_sync_width = (new_mode.h_sync_width * 5) / 8;
			new_mode.h_active = (new_mode.h_active * 5) / 8;
			new_mode.pclk = (new_mode.pclk / 8) * 5;
		}
	}

	if (memcmp(&dc->mode, &new_mode, sizeof(dc->mode)) == 0) {
		/* mode is unchanged, just return */
		return 0;
	}

	memcpy(&dc->mode, &new_mode, sizeof(dc->mode));
	dc->mode_dirty = true;

	if (dc->out->type == TEGRA_DC_OUT_RGB)
		panel_sync_rate = tegra_dc_calc_refresh(&new_mode);
	else if (dc->out->type == TEGRA_DC_OUT_DSI)
		panel_sync_rate = dc->out->dsi->rated_refresh_rate * 1000;

	if (dc->out->type == TEGRA_DC_OUT_FAKE_DSIA ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIB ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED) {
		tegra_dsi_init_clock_param(dc);
	}

	/* Update cached mode */
	memcpy(&dc->cached_mode, &dc->mode, sizeof(dc->mode));

	print_mode(dc, &new_mode, __func__);
	dc->frametime_ns = calc_frametime_ns(&new_mode);

	return 0;
}

int tegra_dc_set_mode(struct tegra_dc *dc, const struct tegra_dc_mode *mode)
{
	mutex_lock(&dc->lock);
	_tegra_dc_set_mode(dc, mode);
	mutex_unlock(&dc->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_set_mode);

int tegra_dc_to_fb_videomode(struct fb_videomode *fbmode,
	const struct tegra_dc_mode *mode)
{
	long mode_pclk;

	if (!fbmode || !mode || !mode->pclk) {
		if (fbmode)
			memset(fbmode, 0, sizeof(*fbmode));
		return -EINVAL;
	}
	if (mode->rated_pclk >= 1000) /* handle DSI one-shot modes */
		mode_pclk = mode->rated_pclk;
	else if (mode->pclk >= 1000) /* normal continous modes */
		mode_pclk = mode->pclk;
	else
		mode_pclk = 0;
	memset(fbmode, 0, sizeof(*fbmode));
	fbmode->right_margin = mode->h_front_porch;
	fbmode->lower_margin = mode->v_front_porch;
	fbmode->hsync_len = mode->h_sync_width;
	fbmode->vsync_len = mode->v_sync_width;
	fbmode->left_margin = mode->h_back_porch;
	fbmode->upper_margin = mode->v_back_porch;
	fbmode->xres = mode->h_active;
	fbmode->yres = mode->v_active;
	fbmode->vmode = mode->vmode;
	if (mode->stereo_mode) {
		/* Double the pixel clock and update v_active only for
		 * frame packed mode */
		mode_pclk /= 2;
		/* total v_active = yres*2 + activespace */
		fbmode->yres = (mode->v_active - mode->v_sync_width -
			mode->v_back_porch - mode->v_front_porch) / 2;
		fbmode->vmode |= FB_VMODE_STEREO_FRAME_PACK;
	}

	if (!(mode->flags & TEGRA_DC_MODE_FLAG_NEG_H_SYNC))
		fbmode->sync |=  FB_SYNC_HOR_HIGH_ACT;
	if (!(mode->flags & TEGRA_DC_MODE_FLAG_NEG_V_SYNC))
		fbmode->sync |= FB_SYNC_VERT_HIGH_ACT;

	if (mode->avi_m == TEGRA_DC_MODE_AVI_M_256_135)
		fbmode->flag |= FB_FLAG_RATIO_256_135;
	else if (mode->avi_m == TEGRA_DC_MODE_AVI_M_64_27)
		fbmode->flag |= FB_FLAG_RATIO_64_27;
	else if (mode->avi_m == TEGRA_DC_MODE_AVI_M_16_9)
		fbmode->flag |= FB_FLAG_RATIO_16_9;
	else if (mode->avi_m == TEGRA_DC_MODE_AVI_M_4_3)
		fbmode->flag |= FB_FLAG_RATIO_4_3;

	if (mode_pclk >= 1000) { /* else 0 */
		fbmode->pixclock = KHZ2PICOS(mode_pclk / 1000);
#if defined(CONFIG_FB_MODE_PIXCLOCK_HZ)
		fbmode->pixclock_hz = mode_pclk;
#endif
	}
	fbmode->refresh = tegra_dc_calc_refresh(mode) / 1000;

	return 0;
}

int tegra_dc_update_mode(struct tegra_dc *dc)
{
	int ret = 0;

	if (dc->mode_dirty) {
		if (tegra_dc_is_nvdisplay())
			ret = tegra_nvdisp_program_mode(dc, &dc->mode);
		else
			ret = tegra_dc_program_mode(dc, &dc->mode);
	}

	/* Update tracing constants */
	if (!dc->mode_dirty) {
		int line_width = dc->mode.h_sync_width +
					dc->mode.h_back_porch +
					dc->mode.h_active +
					dc->mode.h_front_porch;
		dc->mode_metadata.line_in_nsec = ((u64)line_width *
					(u64)NSEC_PER_SEC) /
					(u64)dc->mode.pclk;
		dc->mode_metadata.vtotal_lines = dc->mode.v_sync_width +
					dc->mode.v_back_porch +
					dc->mode.v_active +
					dc->mode.v_front_porch;
		dc->mode_metadata.vblank_lines =
					dc->mode_metadata.vtotal_lines -
					dc->mode.v_front_porch;
	}

	return ret;
}

int tegra_dc_set_fb_mode(struct tegra_dc *dc,
		const struct fb_videomode *fbmode, bool stereo_mode)
{
	struct tegra_dc_mode mode;

	if (!fbmode->pixclock)
		return -EINVAL;

	memset(&mode, 0, sizeof(mode));
#if defined(CONFIG_FB_MODE_PIXCLOCK_HZ)
	if (fbmode->pixclock_hz) {
		mode.pclk = fbmode->pixclock_hz;
		mode.pclk_hz_used = true;
	} else {
		mode.pclk = PICOS2KHZ(fbmode->pixclock) * 1000;
		mode.pclk_hz_used = false;
	}
#else
	mode.pclk = PICOS2KHZ(fbmode->pixclock) * 1000;
	mode.pclk_hz_used = false;
#endif

	mode.h_sync_width = fbmode->hsync_len;
	mode.v_sync_width = fbmode->vsync_len;
	mode.h_back_porch = fbmode->left_margin;
	mode.v_back_porch = fbmode->upper_margin;
	mode.h_active = fbmode->xres;
	mode.v_active = fbmode->yres;
	mode.h_front_porch = fbmode->right_margin;
	mode.v_front_porch = fbmode->lower_margin;
	mode.stereo_mode = stereo_mode;
	mode.vmode = fbmode->vmode;

	if (fbmode->flag & FB_FLAG_RATIO_256_135)
		mode.avi_m = TEGRA_DC_MODE_AVI_M_256_135;
	else if (fbmode->flag & FB_FLAG_RATIO_64_27)
		mode.avi_m = TEGRA_DC_MODE_AVI_M_64_27;
	else if (fbmode->flag & FB_FLAG_RATIO_16_9)
		mode.avi_m = TEGRA_DC_MODE_AVI_M_16_9;
	else if (fbmode->flag & FB_FLAG_RATIO_4_3)
		mode.avi_m = TEGRA_DC_MODE_AVI_M_4_3;
	else
		mode.avi_m = calc_default_avi_m(dc);

	if (!check_mode_timings(dc, &mode, true))
		return -EINVAL;

	/* Double the pixel clock and update v_active only for
	 * frame packed mode */
	if (mode.stereo_mode) {
		mode.pclk *= 2;
		/* total v_active = yres*2 + activespace */
		mode.v_active = fbmode->yres * 2 +
				fbmode->vsync_len +
				fbmode->upper_margin +
				fbmode->lower_margin;
	}

	mode.flags = 0;

	if (!(fbmode->sync & FB_SYNC_HOR_HIGH_ACT))
		mode.flags |= TEGRA_DC_MODE_FLAG_NEG_H_SYNC;

	if (!(fbmode->sync & FB_SYNC_VERT_HIGH_ACT))
		mode.flags |= TEGRA_DC_MODE_FLAG_NEG_V_SYNC;

	return _tegra_dc_set_mode(dc, &mode);
}
EXPORT_SYMBOL(tegra_dc_set_fb_mode);

int tegra_dc_set_fbcon_boot_mode(struct tegra_dc *dc, struct tegra_edid *edid)
{
	struct fb_monspecs specs;
	int ret = 0;

	specs.modedb = NULL;
	if (tegra_fb_is_console_enabled(dc->pdata)) {
		switch (dc->out->type) {
		case TEGRA_DC_OUT_HDMI:
			break;
		case TEGRA_DC_OUT_DP:
			if (!tegra_dc_is_ext_panel(dc))
				return 0;
			break;
		default:
			return -EINVAL;
		}

		/* In case of seamless display, dc mode would already be set */
		if (!dc->initialized) {
			if (dc->out->fbcon_default_mode) {
				ret = tegra_dc_set_fb_mode(dc,
					dc->out->fbcon_default_mode, false);
				if (ret) {
					dev_err(&dc->ndev->dev,
					"%s: setting default fbcon mode failed,"
					" err = %d\n", __func__, ret);
					}
			} else if (edid && tegra_dc_hpd(dc)) {
				if (!tegra_edid_get_monspecs(edid, &specs)) {
					ret = tegra_dc_set_fb_mode(dc,
						specs.modedb, false);
					if (ret) {
						dev_err(&dc->ndev->dev,
						"%s: set DC mode from modedb,"
						" err = %d\n", __func__, ret);
					}
				} else {
					/* Reading edid from monitor failed */
					ret = tegra_dc_set_fb_mode(dc,
						&tegra_dc_vga_mode, false);
					if (ret) {
						dev_err(&dc->ndev->dev,
						"%s: no edid, set VGA mode,"
						" err=%d\n", __func__, ret);
					}
				}
			} else {
				/* HPD not detected */
				ret = tegra_dc_set_fb_mode(dc,
					&tegra_dc_vga_mode, false);
				if (ret) {
					dev_err(&dc->ndev->dev,
					"%s: fallback to VGA mode, err=%d\n",
					__func__, ret);
				}
			}
		}
	}

	if (specs.modedb != NULL)
		kfree(specs.modedb);
	return ret;
}
EXPORT_SYMBOL(tegra_dc_set_fbcon_boot_mode);
