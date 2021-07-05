/*
 * GK20A Tegra Platform Interface
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

#include <linux/clkdev.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/delay.h>
#include <uapi/linux/nvgpu.h>
#include <linux/dma-buf.h>
#include <linux/dma-attrs.h>
#include <linux/nvmap.h>
#include <linux/reset.h>
#if defined(CONFIG_TEGRA_DVFS)
#include <linux/tegra_soctherm.h>
#endif
#include <linux/platform/tegra/common.h>
#include <linux/platform/tegra/mc.h>
#include <linux/clk/tegra.h>
#if defined(CONFIG_COMMON_CLK)
#include <soc/tegra/tegra-dvfs.h>
#endif
#ifdef CONFIG_TEGRA_BWMGR
#include <linux/platform/tegra/emc_bwmgr.h>
#endif

#include <linux/platform/tegra/tegra_emc.h>
#include <soc/tegra/chip-id.h>

#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/nvhost.h>

#include <nvgpu/linux/dma.h>

#include "gm20b/clk_gm20b.h"

#include "scale.h"
#include "platform_gk20a.h"
#include "clk.h"
#include "os_linux.h"

#include "../../../arch/arm/mach-tegra/iomap.h"
#include <soc/tegra/pmc.h>

#define TEGRA_GK20A_BW_PER_FREQ 32
#define TEGRA_GM20B_BW_PER_FREQ 64
#define TEGRA_DDR3_BW_PER_FREQ 16
#define TEGRA_DDR4_BW_PER_FREQ 16
#define MC_CLIENT_GPU 34
#define PMC_GPU_RG_CNTRL_0		0x2d4

#ifdef CONFIG_COMMON_CLK
#define GPU_RAIL_NAME "vdd-gpu"
#else
#define GPU_RAIL_NAME "vdd_gpu"
#endif

extern struct device tegra_vpr_dev;

#ifdef CONFIG_TEGRA_BWMGR
struct gk20a_emc_params {
	unsigned long bw_ratio;
	unsigned long freq_last_set;
	struct tegra_bwmgr_client *bwmgr_cl;
};
#else
struct gk20a_emc_params {
	unsigned long bw_ratio;
	unsigned long freq_last_set;
};
#endif

#define MHZ_TO_HZ(x) ((x) * 1000000)
#define HZ_TO_MHZ(x) ((x) / 1000000)

static void gk20a_tegra_secure_page_destroy(struct gk20a *g,
				       struct secure_page_buffer *secure_buffer)
{
	DEFINE_DMA_ATTRS(attrs);
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, __DMA_ATTR(attrs));
	dma_free_attrs(&tegra_vpr_dev, secure_buffer->size,
			(void *)(uintptr_t)secure_buffer->phys,
			secure_buffer->phys, __DMA_ATTR(attrs));

	secure_buffer->destroy = NULL;
}

static int gk20a_tegra_secure_alloc(struct gk20a *g,
			     struct gr_ctx_buffer_desc *desc,
			     size_t size)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct secure_page_buffer *secure_buffer = &platform->secure_buffer;
	dma_addr_t phys;
	struct sg_table *sgt;
	struct page *page;
	int err = 0;
	size_t aligned_size = PAGE_ALIGN(size);

	if (nvgpu_mem_is_valid(&desc->mem))
		return 0;

	/* We ran out of preallocated memory */
	if (secure_buffer->used + aligned_size > secure_buffer->size) {
		nvgpu_err(platform->g, "failed to alloc %zu bytes of VPR, %zu/%zu used",
				size, secure_buffer->used, secure_buffer->size);
		return -ENOMEM;
	}

	phys = secure_buffer->phys + secure_buffer->used;

	sgt = nvgpu_kzalloc(platform->g, sizeof(*sgt));
	if (!sgt) {
		nvgpu_err(platform->g, "failed to allocate memory");
		return -ENOMEM;
	}
	err = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (err) {
		nvgpu_err(platform->g, "failed to allocate sg_table");
		goto fail_sgt;
	}
	page = phys_to_page(phys);
	sg_set_page(sgt->sgl, page, size, 0);
	/* This bypasses SMMU for VPR during gmmu_map. */
	sg_dma_address(sgt->sgl) = 0;

	desc->destroy = NULL;

	desc->mem.priv.sgt = sgt;
	desc->mem.size = size;
	desc->mem.aperture = APERTURE_SYSMEM;

	secure_buffer->used += aligned_size;

	return err;

fail_sgt:
	nvgpu_kfree(platform->g, sgt);
	return err;
}

/*
 * gk20a_tegra_get_emc_rate()
 *
 * This function returns the minimum emc clock based on gpu frequency
 */

static unsigned long gk20a_tegra_get_emc_rate(struct gk20a *g,
				struct gk20a_emc_params *emc_params)
{
	unsigned long gpu_freq, gpu_fmax_at_vmin;
	unsigned long emc_rate, emc_scale;

	gpu_freq = clk_get_rate(g->clk.tegra_clk);
	gpu_fmax_at_vmin = tegra_dvfs_get_fmax_at_vmin_safe_t(
		clk_get_parent(g->clk.tegra_clk));

	/* When scaling emc, account for the gpu load when the
	 * gpu frequency is less than or equal to fmax@vmin. */
	if (gpu_freq <= gpu_fmax_at_vmin)
		emc_scale = min(g->pmu.load_avg, g->emc3d_ratio);
	else
		emc_scale = g->emc3d_ratio;

	emc_rate =
		(HZ_TO_MHZ(gpu_freq) * emc_params->bw_ratio * emc_scale) / 1000;

	return MHZ_TO_HZ(emc_rate);
}

/*
 * gk20a_tegra_prescale(profile, freq)
 *
 * This function informs EDP about changed constraints.
 */

static void gk20a_tegra_prescale(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	u32 avg = 0;

	nvgpu_pmu_load_norm(g, &avg);
	tegra_edp_notify_gpu_load(avg, clk_get_rate(g->clk.tegra_clk));
}

/*
 * gk20a_tegra_calibrate_emc()
 *
 */

static void gk20a_tegra_calibrate_emc(struct device *dev,
			       struct gk20a_emc_params *emc_params)
{
	enum tegra_chipid cid = tegra_get_chip_id();
	long gpu_bw, emc_bw;

	/* store gpu bw based on soc */
	switch (cid) {
	case TEGRA210:
		gpu_bw = TEGRA_GM20B_BW_PER_FREQ;
		break;
	case TEGRA124:
	case TEGRA132:
		gpu_bw = TEGRA_GK20A_BW_PER_FREQ;
		break;
	default:
		gpu_bw = 0;
		break;
	}

	/* TODO detect DDR type.
	 * Okay for now since DDR3 and DDR4 have the same BW ratio */
	emc_bw = TEGRA_DDR3_BW_PER_FREQ;

	/* Calculate the bandwidth ratio of gpu_freq <-> emc_freq
	 *   NOTE the ratio must come out as an integer */
	emc_params->bw_ratio = (gpu_bw / emc_bw);
}

#ifdef CONFIG_TEGRA_BWMGR
#ifdef CONFIG_TEGRA_DVFS
static void gm20b_bwmgr_set_rate(struct gk20a_platform *platform, bool enb)
{
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a_emc_params *params;
	unsigned long rate;

	if (!profile || !profile->private_data)
		return;

	params = (struct gk20a_emc_params *)profile->private_data;
	rate = (enb) ? params->freq_last_set : 0;
	tegra_bwmgr_set_emc(params->bwmgr_cl, rate, TEGRA_BWMGR_SET_EMC_FLOOR);
}
#endif

static void gm20b_tegra_postscale(struct device *dev, unsigned long freq)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a_emc_params *emc_params;
	unsigned long emc_rate;

	if (!profile || !profile->private_data)
		return;

	emc_params = profile->private_data;
	emc_rate = gk20a_tegra_get_emc_rate(get_gk20a(dev), emc_params);

	if (emc_rate > tegra_bwmgr_get_max_emc_rate())
		emc_rate = tegra_bwmgr_get_max_emc_rate();

	emc_params->freq_last_set = emc_rate;
	if (platform->is_railgated && platform->is_railgated(dev))
		return;

	tegra_bwmgr_set_emc(emc_params->bwmgr_cl, emc_rate,
			TEGRA_BWMGR_SET_EMC_FLOOR);

}

#endif

#if defined(CONFIG_TEGRA_DVFS)
/*
 * gk20a_tegra_is_railgated()
 *
 * Check status of gk20a power rail
 */

static bool gk20a_tegra_is_railgated(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	bool ret = false;

	if (!nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		ret = !tegra_dvfs_is_rail_up(platform->gpu_rail);

	return ret;
}

/*
 * gm20b_tegra_railgate()
 *
 * Gate (disable) gm20b power rail
 */

static int gm20b_tegra_railgate(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int ret = 0;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL) ||
	    !tegra_dvfs_is_rail_up(platform->gpu_rail))
		return 0;

	tegra_mc_flush(MC_CLIENT_GPU);

	udelay(10);

	/* enable clamp */
	tegra_pmc_writel_relaxed(0x1, PMC_GPU_RG_CNTRL_0);
	tegra_pmc_readl(PMC_GPU_RG_CNTRL_0);

	udelay(10);

	platform->reset_assert(dev);

	udelay(10);

	/*
	 * GPCPLL is already disabled before entering this function; reference
	 * clocks are enabled until now - disable them just before rail gating
	 */
	clk_disable_unprepare(platform->clk_reset);
	clk_disable_unprepare(platform->clk[0]);
	clk_disable_unprepare(platform->clk[1]);
	if (platform->clk[3])
		clk_disable_unprepare(platform->clk[3]);

	udelay(10);

	tegra_soctherm_gpu_tsens_invalidate(1);

	if (tegra_dvfs_is_rail_up(platform->gpu_rail)) {
		ret = tegra_dvfs_rail_power_down(platform->gpu_rail);
		if (ret)
			goto err_power_off;
	} else
		pr_info("No GPU regulator?\n");

#ifdef CONFIG_TEGRA_BWMGR
	gm20b_bwmgr_set_rate(platform, false);
#endif

	return 0;

err_power_off:
	nvgpu_err(platform->g, "Could not railgate GPU");
	return ret;
}


/*
 * gm20b_tegra_unrailgate()
 *
 * Ungate (enable) gm20b power rail
 */

static int gm20b_tegra_unrailgate(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = platform->g;
	int ret = 0;
	bool first = false;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		return 0;

	ret = tegra_dvfs_rail_power_up(platform->gpu_rail);
	if (ret)
		return ret;

#ifdef CONFIG_TEGRA_BWMGR
	gm20b_bwmgr_set_rate(platform, true);
#endif

	tegra_soctherm_gpu_tsens_invalidate(0);

	if (!platform->clk_reset) {
		platform->clk_reset = clk_get(dev, "gpu_gate");
		if (IS_ERR(platform->clk_reset)) {
			nvgpu_err(g, "fail to get gpu reset clk");
			goto err_clk_on;
		}
	}

	if (!first) {
		ret = clk_prepare_enable(platform->clk_reset);
		if (ret) {
			nvgpu_err(g, "could not turn on gpu_gate");
			goto err_clk_on;
		}

		ret = clk_prepare_enable(platform->clk[0]);
		if (ret) {
			nvgpu_err(g, "could not turn on gpu pll");
			goto err_clk_on;
		}
		ret = clk_prepare_enable(platform->clk[1]);
		if (ret) {
			nvgpu_err(g, "could not turn on pwr clock");
			goto err_clk_on;
		}

		if (platform->clk[3]) {
			ret = clk_prepare_enable(platform->clk[3]);
			if (ret) {
				nvgpu_err(g, "could not turn on fuse clock");
				goto err_clk_on;
			}
		}
	}

	udelay(10);

	platform->reset_assert(dev);

	udelay(10);

	tegra_pmc_writel_relaxed(0, PMC_GPU_RG_CNTRL_0);
	tegra_pmc_readl(PMC_GPU_RG_CNTRL_0);

	udelay(10);

	clk_disable(platform->clk_reset);
	platform->reset_deassert(dev);
	clk_enable(platform->clk_reset);

	/* Flush MC after boot/railgate/SC7 */
	tegra_mc_flush(MC_CLIENT_GPU);

	udelay(10);

	tegra_mc_flush_done(MC_CLIENT_GPU);

	udelay(10);

	return 0;

err_clk_on:
	tegra_dvfs_rail_power_down(platform->gpu_rail);

	return ret;
}
#endif


static struct {
	char *name;
	unsigned long default_rate;
} tegra_gk20a_clocks[] = {
	{"gpu_ref", UINT_MAX},
	{"pll_p_out5", 204000000},
	{"emc", UINT_MAX},
	{"fuse", UINT_MAX},
};



/*
 * gk20a_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gk20a platform data.
 */

static int gk20a_tegra_get_clocks(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	char devname[16];
	unsigned int i;
	int ret = 0;

	BUG_ON(GK20A_CLKS_MAX < ARRAY_SIZE(tegra_gk20a_clocks));

	snprintf(devname, sizeof(devname), "tegra_%s", dev_name(dev));

	platform->num_clks = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_gk20a_clocks); i++) {
		long rate = tegra_gk20a_clocks[i].default_rate;
		struct clk *c;

		c = clk_get_sys(devname, tegra_gk20a_clocks[i].name);
		if (IS_ERR(c)) {
			ret = PTR_ERR(c);
			goto err_get_clock;
		}
		rate = clk_round_rate(c, rate);
		clk_set_rate(c, rate);
		platform->clk[i] = c;
	}
	platform->num_clks = i;

	return 0;

err_get_clock:

	while (i--)
		clk_put(platform->clk[i]);
	return ret;
}

#if defined(CONFIG_RESET_CONTROLLER) && defined(CONFIG_COMMON_CLK)
static int gm20b_tegra_reset_assert(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	if (!platform->reset_control) {
		WARN(1, "Reset control not initialized\n");
		return -ENOSYS;
	}

	return reset_control_assert(platform->reset_control);
}

static int gm20b_tegra_reset_deassert(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	if (!platform->reset_control) {
		WARN(1, "Reset control not initialized\n");
		return -ENOSYS;
	}

	return reset_control_deassert(platform->reset_control);
}
#endif

static void gk20a_tegra_scale_init(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a_emc_params *emc_params;
	struct gk20a *g = platform->g;

	if (!profile)
		return;

	if (profile->private_data)
		return;

	emc_params = nvgpu_kzalloc(platform->g, sizeof(*emc_params));
	if (!emc_params)
		return;

	emc_params->freq_last_set = -1;
	gk20a_tegra_calibrate_emc(dev, emc_params);

#ifdef CONFIG_TEGRA_BWMGR
	emc_params->bwmgr_cl = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_GPU);
	if (!emc_params->bwmgr_cl) {
		nvgpu_log_info(g, "%s Missing GPU BWMGR client\n", __func__);
		return;
	}
#endif

	profile->private_data = emc_params;
}

static void gk20a_tegra_scale_exit(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a_emc_params *emc_params;

	if (!profile)
		return;

	emc_params = profile->private_data;
#ifdef CONFIG_TEGRA_BWMGR
	tegra_bwmgr_unregister(emc_params->bwmgr_cl);
#endif

	nvgpu_kfree(platform->g, profile->private_data);
}

void gk20a_tegra_debug_dump(struct device *dev)
{
#ifdef CONFIG_TEGRA_GK20A_NVHOST
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;

	if (g->nvhost_dev)
		nvgpu_nvhost_debug_dump_device(g->nvhost_dev);
#endif
}

int gk20a_tegra_busy(struct device *dev)
{
#ifdef CONFIG_TEGRA_GK20A_NVHOST
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;

	if (g->nvhost_dev)
		return nvgpu_nvhost_module_busy_ext(g->nvhost_dev);
#endif
	return 0;
}

void gk20a_tegra_idle(struct device *dev)
{
#ifdef CONFIG_TEGRA_GK20A_NVHOST
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;

	if (g->nvhost_dev)
		nvgpu_nvhost_module_idle_ext(g->nvhost_dev);
#endif
}

int gk20a_tegra_init_secure_alloc(struct gk20a_platform *platform)
{
	struct gk20a *g = platform->g;
	struct secure_page_buffer *secure_buffer = &platform->secure_buffer;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t iova;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		return 0;

	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, __DMA_ATTR(attrs));
	(void)dma_alloc_attrs(&tegra_vpr_dev, platform->secure_buffer_size, &iova,
				      GFP_KERNEL, __DMA_ATTR(attrs));
	/* Some platforms disable VPR. In that case VPR allocations always
	 * fail. Just disable VPR usage in nvgpu in that case. */
	if (dma_mapping_error(&tegra_vpr_dev, iova))
		return 0;

	secure_buffer->size = platform->secure_buffer_size;
	secure_buffer->phys = iova;
	secure_buffer->destroy = gk20a_tegra_secure_page_destroy;

	g->ops.secure_alloc = gk20a_tegra_secure_alloc;
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_VPR, true);

	return 0;
}

#ifdef CONFIG_COMMON_CLK
static struct clk *gk20a_clk_get(struct gk20a *g)
{
	if (!g->clk.tegra_clk) {
		struct clk *clk, *clk_parent;
		char clk_dev_id[32];
		struct device *dev = dev_from_gk20a(g);

		snprintf(clk_dev_id, 32, "tegra_%s", dev_name(dev));

		clk = clk_get_sys(clk_dev_id, "gpu");
		if (IS_ERR(clk)) {
			nvgpu_err(g, "fail to get tegra gpu clk %s/gpu\n",
				  clk_dev_id);
			return NULL;
		}

		clk_parent = clk_get_parent(clk);
		if (IS_ERR_OR_NULL(clk_parent)) {
			nvgpu_err(g, "fail to get tegra gpu clk parent%s/gpu\n",
				  clk_dev_id);
			return NULL;
		}

		g->clk.tegra_clk = clk;
		g->clk.tegra_clk_parent = clk_parent;
	}

	return g->clk.tegra_clk;
}

static int gm20b_clk_prepare_ops(struct clk_hw *hw)
{
	struct clk_gk20a *clk = to_clk_gk20a(hw);
	return gm20b_clk_prepare(clk);
}

static void gm20b_clk_unprepare_ops(struct clk_hw *hw)
{
	struct clk_gk20a *clk = to_clk_gk20a(hw);
	gm20b_clk_unprepare(clk);
}

static int gm20b_clk_is_prepared_ops(struct clk_hw *hw)
{
	struct clk_gk20a *clk = to_clk_gk20a(hw);
	return gm20b_clk_is_prepared(clk);
}

static unsigned long gm20b_recalc_rate_ops(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_gk20a *clk = to_clk_gk20a(hw);
	return gm20b_recalc_rate(clk, parent_rate);
}

static int gm20b_gpcclk_set_rate_ops(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct clk_gk20a *clk = to_clk_gk20a(hw);
	return gm20b_gpcclk_set_rate(clk, rate, parent_rate);
}

static long gm20b_round_rate_ops(struct clk_hw *hw, unsigned long rate,
			     unsigned long *parent_rate)
{
	struct clk_gk20a *clk = to_clk_gk20a(hw);
	return gm20b_round_rate(clk, rate, parent_rate);
}

static const struct clk_ops gm20b_clk_ops = {
	.prepare = gm20b_clk_prepare_ops,
	.unprepare = gm20b_clk_unprepare_ops,
	.is_prepared = gm20b_clk_is_prepared_ops,
	.recalc_rate = gm20b_recalc_rate_ops,
	.set_rate = gm20b_gpcclk_set_rate_ops,
	.round_rate = gm20b_round_rate_ops,
};

static int gm20b_register_gpcclk(struct gk20a *g)
{
	const char *parent_name = "pllg_ref";
	struct clk_gk20a *clk = &g->clk;
	struct clk_init_data init;
	struct clk *c;
	int err = 0;

	/* make sure the clock is available */
	if (!gk20a_clk_get(g))
		return -ENOSYS;

	err = gm20b_init_clk_setup_sw(g);
	if (err)
		return err;

	init.name = "gpcclk";
	init.ops = &gm20b_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = 0;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	clk->hw.init = &init;
	c = clk_register(dev_from_gk20a(g), &clk->hw);
	if (IS_ERR(c)) {
		nvgpu_err(g, "Failed to register GPCPLL clock");
		return -EINVAL;
	}

	clk->g = g;
	clk_register_clkdev(c, "gpcclk", "gpcclk");

	return err;
}
#endif /* CONFIG_COMMON_CLK */

static int gk20a_tegra_probe(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	bool joint_xpu_rail = false;
	int ret;
	struct gk20a *g = platform->g;

#ifdef CONFIG_COMMON_CLK
	/* DVFS is not guaranteed to be initialized at the time of probe on
	 * kernels with Common Clock Framework enabled.
	 */
	if (!platform->gpu_rail) {
		platform->gpu_rail = tegra_dvfs_get_rail_by_name(GPU_RAIL_NAME);
		if (!platform->gpu_rail) {
			nvgpu_log_info(g, "deferring probe no gpu_rail");
			return -EPROBE_DEFER;
		}
	}

	if (!tegra_dvfs_is_rail_ready(platform->gpu_rail)) {
		nvgpu_log_info(g, "deferring probe gpu_rail not ready");
		return -EPROBE_DEFER;
	}
#endif

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	ret = nvgpu_get_nvhost_dev(platform->g);
	if (ret)
		return ret;
#endif

#ifdef CONFIG_OF
	joint_xpu_rail = of_property_read_bool(of_chosen,
				"nvidia,tegra-joint_xpu_rail");
#endif

	if (joint_xpu_rail) {
		nvgpu_log_info(g, "XPU rails are joint\n");
		platform->can_railgate_init = false;
		__nvgpu_set_enabled(g, NVGPU_CAN_RAILGATE, false);
	}

	platform->g->clk.gpc_pll.id = GK20A_GPC_PLL;
	if (tegra_get_chip_id() == TEGRA210) {
		/* WAR for bug 1547668: Disable railgating and scaling
		   irrespective of platform data if the rework was not made. */
		np = of_find_node_by_path("/gpu-dvfs-rework");
		if (!(np && of_device_is_available(np))) {
			platform->devfreq_governor = "";
			dev_warn(dev, "board does not support scaling");
		}
		platform->g->clk.gpc_pll.id = GM20B_GPC_PLL_B1;
		if (tegra_chip_get_revision() > TEGRA210_REVISION_A04p)
			platform->g->clk.gpc_pll.id = GM20B_GPC_PLL_C1;
	}

	if (tegra_get_chip_id() == TEGRA132)
		platform->soc_name = "tegra13x";

	gk20a_tegra_get_clocks(dev);
	nvgpu_linux_init_clk_support(platform->g);
	ret = gk20a_tegra_init_secure_alloc(platform);
	if (ret)
		return ret;

	if (platform->clk_register) {
		ret = platform->clk_register(platform->g);
		if (ret)
			return ret;
	}

	return 0;
}

static int gk20a_tegra_late_probe(struct device *dev)
{
	return 0;
}

static int gk20a_tegra_remove(struct device *dev)
{
	/* deinitialise tegra specific scaling quirks */
	gk20a_tegra_scale_exit(dev);

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	nvgpu_free_nvhost_dev(get_gk20a(dev));
#endif

	return 0;
}

static int gk20a_tegra_suspend(struct device *dev)
{
	tegra_edp_notify_gpu_load(0, 0);
	return 0;
}

#if defined(CONFIG_COMMON_CLK)
static long gk20a_round_clk_rate(struct device *dev, unsigned long rate)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;

	/* make sure the clock is available */
	if (!gk20a_clk_get(g))
		return rate;

	return clk_round_rate(clk_get_parent(g->clk.tegra_clk), rate);
}

static int gk20a_clk_get_freqs(struct device *dev,
				unsigned long **freqs, int *num_freqs)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;

	/* make sure the clock is available */
	if (!gk20a_clk_get(g))
		return -ENOSYS;

	return tegra_dvfs_get_freqs(clk_get_parent(g->clk.tegra_clk),
				freqs, num_freqs);
}
#endif

struct gk20a_platform gm20b_tegra_platform = {
	.has_syncpoints = true,
	.aggressive_sync_destroy_thresh = 64,

	/* power management configuration */
	.railgate_delay_init	= 500,
	.can_railgate_init	= true,
	.can_elpg_init          = true,
	.enable_slcg            = true,
	.enable_blcg            = true,
	.enable_elcg            = true,
	.can_slcg               = true,
	.can_blcg               = true,
	.can_elcg               = true,
	.enable_elpg            = true,
	.enable_aelpg           = true,
	.enable_perfmon         = true,
	.ptimer_src_freq	= 19200000,

	.force_reset_in_do_idle = false,

	.ch_wdt_timeout_ms = 5000,

	.probe = gk20a_tegra_probe,
	.late_probe = gk20a_tegra_late_probe,
	.remove = gk20a_tegra_remove,
	/* power management callbacks */
	.suspend = gk20a_tegra_suspend,

#if defined(CONFIG_TEGRA_DVFS)
	.railgate = gm20b_tegra_railgate,
	.unrailgate = gm20b_tegra_unrailgate,
	.is_railgated = gk20a_tegra_is_railgated,
#endif

	.busy = gk20a_tegra_busy,
	.idle = gk20a_tegra_idle,

#if defined(CONFIG_RESET_CONTROLLER) && defined(CONFIG_COMMON_CLK)
	.reset_assert = gm20b_tegra_reset_assert,
	.reset_deassert = gm20b_tegra_reset_deassert,
#else
	.reset_assert = gk20a_tegra_reset_assert,
	.reset_deassert = gk20a_tegra_reset_deassert,
#endif

#if defined(CONFIG_COMMON_CLK)
	.clk_round_rate = gk20a_round_clk_rate,
	.get_clk_freqs = gk20a_clk_get_freqs,
#endif

#ifdef CONFIG_COMMON_CLK
	.clk_register = gm20b_register_gpcclk,
#endif

	/* frequency scaling configuration */
	.initscale = gk20a_tegra_scale_init,
	.prescale = gk20a_tegra_prescale,
#ifdef CONFIG_TEGRA_BWMGR
	.postscale = gm20b_tegra_postscale,
#endif
	.devfreq_governor = "nvhost_podgov",
	.qos_notify = gk20a_scale_qos_notify,

	.dump_platform_dependencies = gk20a_tegra_debug_dump,

#ifdef CONFIG_NVGPU_SUPPORT_CDE
	.has_cde = true,
#endif

	.soc_name = "tegra21x",

	.unified_memory = true,
	.dma_mask = DMA_BIT_MASK(34),
	.force_128K_pmu_vm = true,

	.secure_buffer_size = 335872,
};
