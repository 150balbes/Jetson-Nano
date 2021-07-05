/*
 * GP10B Tegra Platform Interface
 *
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <linux/reset.h>
#include <linux/platform/tegra/emc_bwmgr.h>

#include <uapi/linux/nvgpu.h>

#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra_powergate.h>
#include <soc/tegra/tegra-bpmp-dvfs.h>

#include <dt-bindings/memory/tegra-swgroup.h>

#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/hashtable.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/nvhost.h>

#include "os_linux.h"

#include "clk.h"

#include "platform_gk20a.h"
#include "platform_gk20a_tegra.h"
#include "platform_gp10b.h"
#include "platform_gp10b_tegra.h"
#include "scale.h"

/* Select every GP10B_FREQ_SELECT_STEP'th frequency from h/w table */
#define GP10B_FREQ_SELECT_STEP	8
/* Allow limited set of frequencies to be available */
#define GP10B_NUM_SUPPORTED_FREQS 15
/* Max number of freq supported in h/w */
#define GP10B_MAX_SUPPORTED_FREQS 120
static unsigned long
gp10b_freq_table[GP10B_MAX_SUPPORTED_FREQS / GP10B_FREQ_SELECT_STEP];

static bool freq_table_init_complete;
static int num_supported_freq;

#define TEGRA_GP10B_BW_PER_FREQ 64
#define TEGRA_DDR4_BW_PER_FREQ 16

#define EMC_BW_RATIO  (TEGRA_GP10B_BW_PER_FREQ / TEGRA_DDR4_BW_PER_FREQ)

#define GPCCLK_INIT_RATE 1000000000

static struct {
	char *name;
	unsigned long default_rate;
} tegra_gp10b_clocks[] = {
	{"gpu", GPCCLK_INIT_RATE},
	{"gpu_sys", 204000000} };

/*
 * gp10b_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gp10b platform data.
 */

int gp10b_tegra_get_clocks(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	unsigned int i;

	platform->num_clks = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_gp10b_clocks); i++) {
		long rate = tegra_gp10b_clocks[i].default_rate;
		struct clk *c;

		c = clk_get(dev, tegra_gp10b_clocks[i].name);
		if (IS_ERR(c)) {
			nvgpu_err(platform->g, "cannot get clock %s",
					tegra_gp10b_clocks[i].name);
		} else {
			clk_set_rate(c, rate);
			platform->clk[i] = c;
		}
	}
	platform->num_clks = i;

	if (platform->clk[0]) {
		i = tegra_bpmp_dvfs_get_clk_id(dev->of_node,
					       tegra_gp10b_clocks[0].name);
		if (i > 0)
			platform->maxmin_clk_id = i;
	}

	return 0;
}

void gp10b_tegra_scale_init(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct tegra_bwmgr_client *bwmgr_handle;

	if (!profile)
		return;

	if ((struct tegra_bwmgr_client *)profile->private_data)
		return;

	bwmgr_handle = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_GPU);
	if (!bwmgr_handle)
		return;

	profile->private_data = (void *)bwmgr_handle;
}

static void gp10b_tegra_scale_exit(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	if (profile && profile->private_data)
		tegra_bwmgr_unregister(
			(struct tegra_bwmgr_client *)profile->private_data);
}

static int gp10b_tegra_probe(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	bool joint_xpu_rail = false;
	struct gk20a *g = platform->g;
#ifdef CONFIG_TEGRA_GK20A_NVHOST
	int ret;

	ret = nvgpu_get_nvhost_dev(platform->g);
	if (ret)
		return ret;
#endif

	ret = gk20a_tegra_init_secure_alloc(platform);
	if (ret)
		return ret;

	platform->disable_bigpage = !device_is_iommuable(dev);

	platform->g->gr.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;
	platform->g->gr.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;

	platform->g->gr.ctx_vars.force_preemption_gfxp = false;
	platform->g->gr.ctx_vars.force_preemption_cilp = false;

#ifdef CONFIG_OF
	joint_xpu_rail = of_property_read_bool(of_chosen,
				"nvidia,tegra-joint_xpu_rail");
#endif

	if (joint_xpu_rail) {
		nvgpu_log_info(g, "XPU rails are joint\n");
		platform->can_railgate_init = false;
		__nvgpu_set_enabled(g, NVGPU_CAN_RAILGATE, false);
	}

	gp10b_tegra_get_clocks(dev);
	nvgpu_linux_init_clk_support(platform->g);

	nvgpu_mutex_init(&platform->clk_get_freq_lock);

	platform->g->ops.clk.support_clk_freq_controller = true;

	return 0;
}

static int gp10b_tegra_late_probe(struct device *dev)
{
	return 0;
}

static int gp10b_tegra_remove(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	/* deinitialise tegra specific scaling quirks */
	gp10b_tegra_scale_exit(dev);

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	nvgpu_free_nvhost_dev(get_gk20a(dev));
#endif

	nvgpu_mutex_destroy(&platform->clk_get_freq_lock);

	return 0;
}

static bool gp10b_tegra_is_railgated(struct device *dev)
{
	bool ret = false;

	if (tegra_bpmp_running())
		ret = !tegra_powergate_is_powered(TEGRA186_POWER_DOMAIN_GPU);

	return ret;
}

static int gp10b_tegra_railgate(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	/* remove emc frequency floor */
	if (profile)
		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			0, TEGRA_BWMGR_SET_EMC_FLOOR);

	if (tegra_bpmp_running() &&
	    tegra_powergate_is_powered(TEGRA186_POWER_DOMAIN_GPU)) {
		int i;
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_disable_unprepare(platform->clk[i]);
		}
		tegra_powergate_partition(TEGRA186_POWER_DOMAIN_GPU);
	}
	return 0;
}

static int gp10b_tegra_unrailgate(struct device *dev)
{
	int ret = 0;
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	if (tegra_bpmp_running()) {
		int i;
		ret = tegra_unpowergate_partition(TEGRA186_POWER_DOMAIN_GPU);
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_prepare_enable(platform->clk[i]);
		}
	}

	/* to start with set emc frequency floor to max rate*/
	if (profile)
		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			tegra_bwmgr_get_max_emc_rate(),
			TEGRA_BWMGR_SET_EMC_FLOOR);
	return ret;
}

static int gp10b_tegra_suspend(struct device *dev)
{
	return 0;
}

int gp10b_tegra_reset_assert(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int ret = 0;

	if (!platform->reset_control)
		return -EINVAL;

	ret = reset_control_assert(platform->reset_control);

	return ret;
}

int gp10b_tegra_reset_deassert(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int ret = 0;

	if (!platform->reset_control)
		return -EINVAL;

	ret = reset_control_deassert(platform->reset_control);

	return ret;
}

void gp10b_tegra_prescale(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	u32 avg = 0;

	nvgpu_log_fn(g, " ");

	nvgpu_pmu_load_norm(g, &avg);

	nvgpu_log_fn(g, "done");
}

void gp10b_tegra_postscale(struct device *pdev,
					unsigned long freq)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a *g = get_gk20a(pdev);
	unsigned long emc_rate;

	nvgpu_log_fn(g, " ");
	if (profile && profile->private_data &&
			!platform->is_railgated(pdev)) {
		unsigned long emc_scale;

		if (freq <= gp10b_freq_table[0])
			emc_scale = 0;
		else
			emc_scale = g->emc3d_ratio;

		emc_rate = (freq * EMC_BW_RATIO * emc_scale) / 1000;

		if (emc_rate > tegra_bwmgr_get_max_emc_rate())
			emc_rate = tegra_bwmgr_get_max_emc_rate();

		tegra_bwmgr_set_emc(
			(struct tegra_bwmgr_client *)profile->private_data,
			emc_rate, TEGRA_BWMGR_SET_EMC_FLOOR);
	}
	nvgpu_log_fn(g, "done");
}

long gp10b_round_clk_rate(struct device *dev, unsigned long rate)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_scale_profile *profile = g->scale_profile;
	unsigned long *freq_table = profile->devfreq_profile.freq_table;
	int max_states = profile->devfreq_profile.max_state;
	int i;

	for (i = 0; i < max_states; ++i)
		if (freq_table[i] >= rate)
			return freq_table[i];

	return freq_table[max_states - 1];
}

int gp10b_clk_get_freqs(struct device *dev,
				unsigned long **freqs, int *num_freqs)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;
	unsigned long max_rate;
	unsigned long new_rate = 0, prev_rate = 0;
	int i, freq_counter = 0;
	int sel_freq_cnt;
	unsigned long loc_freq_table[GP10B_MAX_SUPPORTED_FREQS];

	nvgpu_mutex_acquire(&platform->clk_get_freq_lock);

	if (freq_table_init_complete) {

		*freqs = gp10b_freq_table;
		*num_freqs = num_supported_freq;

		nvgpu_mutex_release(&platform->clk_get_freq_lock);

		return 0;
	}

	max_rate = clk_round_rate(platform->clk[0], (UINT_MAX - 1));

	/*
	 * Walk the h/w frequency table and update the local table
	 */
	for (i = 0; i < GP10B_MAX_SUPPORTED_FREQS; ++i) {
		prev_rate = new_rate;
		new_rate = clk_round_rate(platform->clk[0],
						prev_rate + 1);
		loc_freq_table[i] = new_rate;
		if (new_rate == max_rate)
			break;
	}
	freq_counter = i + 1;
	WARN_ON(freq_counter == GP10B_MAX_SUPPORTED_FREQS);

	/*
	 * If the number of achievable frequencies is less than or
	 * equal to GP10B_NUM_SUPPORTED_FREQS, select all frequencies
	 * else, select one out of every 8 frequencies
	 */
	if (freq_counter <= GP10B_NUM_SUPPORTED_FREQS) {
		for (sel_freq_cnt = 0; sel_freq_cnt < freq_counter; ++sel_freq_cnt)
			gp10b_freq_table[sel_freq_cnt] =
					loc_freq_table[sel_freq_cnt];
	} else {
		/*
		 * Walk the h/w frequency table and only select
		 * GP10B_FREQ_SELECT_STEP'th frequencies and
		 * add MAX freq to last
		 */
		sel_freq_cnt = 0;
		for (i = 0; i < GP10B_MAX_SUPPORTED_FREQS; ++i) {
			new_rate = loc_freq_table[i];

			if (i % GP10B_FREQ_SELECT_STEP == 0 ||
					new_rate == max_rate) {
				gp10b_freq_table[sel_freq_cnt++] =
							new_rate;

				if (new_rate == max_rate)
					break;
			}
		}
		WARN_ON(sel_freq_cnt == GP10B_MAX_SUPPORTED_FREQS);
	}

	/* Fill freq table */
	*freqs = gp10b_freq_table;
	*num_freqs = sel_freq_cnt;
	num_supported_freq = sel_freq_cnt;

	freq_table_init_complete = true;

	nvgpu_log_info(g, "min rate: %ld max rate: %ld num_of_freq %d\n",
				gp10b_freq_table[0], max_rate, *num_freqs);

	nvgpu_mutex_release(&platform->clk_get_freq_lock);

	return 0;
}

struct gk20a_platform gp10b_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay_init	= 500,

	/* ldiv slowdown factor */
	.ldiv_slowdown_factor_init = SLOWDOWN_FACTOR_FPDIV_BY16,

	/* power management configuration */
	.can_railgate_init	= true,
	.enable_elpg            = true,
	.can_elpg_init          = true,
	.enable_blcg		= true,
	.enable_slcg		= true,
	.enable_elcg		= true,
	.can_slcg               = true,
	.can_blcg               = true,
	.can_elcg               = true,
	.enable_aelpg       = true,
	.enable_perfmon         = true,

	/* ptimer src frequency in hz*/
	.ptimer_src_freq	= 31250000,

	.ch_wdt_timeout_ms = 5000,

	.probe = gp10b_tegra_probe,
	.late_probe = gp10b_tegra_late_probe,
	.remove = gp10b_tegra_remove,

	/* power management callbacks */
	.suspend = gp10b_tegra_suspend,
	.railgate = gp10b_tegra_railgate,
	.unrailgate = gp10b_tegra_unrailgate,
	.is_railgated = gp10b_tegra_is_railgated,

	.busy = gk20a_tegra_busy,
	.idle = gk20a_tegra_idle,

	.dump_platform_dependencies = gk20a_tegra_debug_dump,

#ifdef CONFIG_NVGPU_SUPPORT_CDE
	.has_cde = true,
#endif

	.clk_round_rate = gp10b_round_clk_rate,
	.get_clk_freqs = gp10b_clk_get_freqs,

	/* frequency scaling configuration */
	.initscale = gp10b_tegra_scale_init,
	.prescale = gp10b_tegra_prescale,
	.postscale = gp10b_tegra_postscale,
	.devfreq_governor = "nvhost_podgov",

	.qos_notify = gk20a_scale_qos_notify,

	.reset_assert = gp10b_tegra_reset_assert,
	.reset_deassert = gp10b_tegra_reset_deassert,

	.force_reset_in_do_idle = false,

	.soc_name = "tegra18x",

	.unified_memory = true,
	.dma_mask = DMA_BIT_MASK(36),

	.ltc_streamid = TEGRA_SID_GPUB,

	.secure_buffer_size = 401408,
};
