/*
 * Linux clock support
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>

#include <soc/tegra/tegra-dvfs.h>
#include <soc/tegra/tegra-bpmp-dvfs.h>

#include "clk.h"
#include "os_linux.h"
#include "platform_gk20a.h"

#include <nvgpu/gk20a.h>
#include <nvgpu/clk_arb.h>

#define HZ_TO_MHZ(x) ((x) / 1000000)

static unsigned long nvgpu_linux_clk_get_rate(struct gk20a *g, u32 api_domain)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));
	unsigned long ret;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		if (g->clk.tegra_clk)
			ret = clk_get_rate(g->clk.tegra_clk);
		else
			ret = clk_get_rate(platform->clk[0]);
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		ret = clk_get_rate(platform->clk[1]);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = 0;
		break;
	}

	return ret;
}

static int nvgpu_linux_clk_set_rate(struct gk20a *g,
				     u32 api_domain, unsigned long rate)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));
	int ret;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		if (g->clk.tegra_clk)
			ret = clk_set_rate(g->clk.tegra_clk, rate);
		else
			ret = clk_set_rate(platform->clk[0], rate);
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		ret = clk_set_rate(platform->clk[1], rate);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned long nvgpu_linux_get_fmax_at_vmin_safe(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));

	/*
	 * On Tegra platforms with GPCPLL bus (gbus) GPU tegra_clk clock exposed
	 * to frequency governor is a shared user on the gbus. The latter can be
	 * accessed as GPU clock parent, and incorporate DVFS related data.
	 */
	if (g->clk.tegra_clk)
		return tegra_dvfs_get_fmax_at_vmin_safe_t(
			g->clk.tegra_clk_parent);

	if (platform->maxmin_clk_id)
		return tegra_bpmp_dvfs_get_fmax_at_vmin(
			platform->maxmin_clk_id);

	return 0;
}

static u32 nvgpu_linux_get_ref_clock_rate(struct gk20a *g)
{
	struct clk *c;

	c = clk_get_sys("gpu_ref", "gpu_ref");
	if (IS_ERR(c)) {
		nvgpu_err(g, "failed to get GPCPLL reference clock");
		return 0;
	}

	return clk_get_rate(c);
}

static int nvgpu_linux_predict_mv_at_hz_cur_tfloor(struct clk_gk20a *clk,
	unsigned long rate)
{
	return tegra_dvfs_predict_mv_at_hz_cur_tfloor(
				clk->tegra_clk_parent, rate);
}

static unsigned long nvgpu_linux_get_maxrate(struct gk20a *g, u32 api_domain)
{
	int ret;
	u16 min_mhz, max_mhz;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		ret = tegra_dvfs_get_maxrate(g->clk.tegra_clk_parent);
		/* If dvfs not supported */
		if (ret == 0) {
			int err = nvgpu_clk_arb_get_arbiter_clk_range(g,
					NVGPU_CLK_DOMAIN_GPCCLK,
					&min_mhz, &max_mhz);
			if (err == 0) {
				ret = max_mhz * 1000000L;
			}
		}
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = 0;
		break;
	}

	return ret;
}

/*
 * This API is used to return a list of supported frequencies by igpu.
 * Set *num_points as 0 to get the size of the freqs list, returned
 * by *num_points itself. freqs array must be provided by caller.
 * If *num_points is non-zero, then freqs array size must atleast
 * equal *num_points.
 */
static int nvgpu_linux_clk_get_f_points(struct gk20a *g,
	u32 api_domain, u32 *num_points, u16 *freqs)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	unsigned long *gpu_freq_table;
	int ret = 0;
	int num_supported_freq = 0;
	u32 i;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		ret = platform->get_clk_freqs(dev, &gpu_freq_table,
		 &num_supported_freq);

		if (ret) {
			return ret;
		}

		if (num_points == NULL) {
			return -EINVAL;
		}

		if (*num_points != 0U) {
			if (freqs == NULL || (*num_points > (u32)num_supported_freq)) {
				return -EINVAL;
			}
		}

		if (*num_points == 0) {
			*num_points = num_supported_freq;
		} else {
			for (i = 0; i < *num_points; i++) {
				freqs[i] = HZ_TO_MHZ(gpu_freq_table[i]);
			}
		}
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int nvgpu_clk_get_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	unsigned long *freqs;
	int num_freqs;
	int ret;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		ret = platform->get_clk_freqs(dev, &freqs, &num_freqs);

		if (!ret) {
			*min_mhz = HZ_TO_MHZ(freqs[0]);
			*max_mhz = HZ_TO_MHZ(freqs[num_freqs - 1]);
		}
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* rate_target should be passed in as Hz
   rounded_rate is returned in Hz */
static int nvgpu_clk_get_round_rate(struct gk20a *g,
		u32 api_domain, unsigned long rate_target,
		unsigned long *rounded_rate)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	unsigned long *freqs;
	int num_freqs;
	int i, ret = 0;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		ret = platform->get_clk_freqs(dev, &freqs, &num_freqs);

		for (i = 0; i < num_freqs; ++i) {
			if (freqs[i] >= rate_target) {
				*rounded_rate = freqs[i];
				return 0;
			}
		}
		*rounded_rate = freqs[num_freqs - 1];
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int nvgpu_linux_prepare_enable(struct clk_gk20a *clk)
{
	return clk_prepare_enable(clk->tegra_clk);
}

static void nvgpu_linux_disable_unprepare(struct clk_gk20a *clk)
{
	clk_disable_unprepare(clk->tegra_clk);
}

void nvgpu_linux_init_clk_support(struct gk20a *g)
{
	g->ops.clk.get_rate = nvgpu_linux_clk_get_rate;
	g->ops.clk.set_rate = nvgpu_linux_clk_set_rate;
	g->ops.clk.get_fmax_at_vmin_safe = nvgpu_linux_get_fmax_at_vmin_safe;
	g->ops.clk.get_ref_clock_rate = nvgpu_linux_get_ref_clock_rate;
	g->ops.clk.predict_mv_at_hz_cur_tfloor = nvgpu_linux_predict_mv_at_hz_cur_tfloor;
	g->ops.clk.get_maxrate = nvgpu_linux_get_maxrate;
	g->ops.clk.prepare_enable = nvgpu_linux_prepare_enable;
	g->ops.clk.disable_unprepare = nvgpu_linux_disable_unprepare;
	g->ops.clk.clk_domain_get_f_points = nvgpu_linux_clk_get_f_points;
	g->ops.clk.get_clk_range = nvgpu_clk_get_range;
	g->ops.clk.clk_get_round_rate = nvgpu_clk_get_round_rate;
	g->ops.clk.measure_freq = nvgpu_clk_measure_freq;
}
