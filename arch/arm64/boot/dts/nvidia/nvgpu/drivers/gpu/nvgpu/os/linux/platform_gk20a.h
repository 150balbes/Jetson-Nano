/*
 * GK20A Platform (SoC) Interface
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

#ifndef _GK20A_PLATFORM_H_
#define _GK20A_PLATFORM_H_

#include <linux/device.h>

#include <nvgpu/lock.h>
#include <nvgpu/gk20a.h>

#define GK20A_CLKS_MAX		4

struct gk20a;
struct channel_gk20a;
struct gr_ctx_buffer_desc;
struct gk20a_scale_profile;

struct secure_page_buffer {
	void (*destroy)(struct gk20a *, struct secure_page_buffer *);
	size_t size;
	dma_addr_t phys;
	size_t used;
};

struct gk20a_platform {
	/* Populated by the gk20a driver before probing the platform. */
	struct gk20a *g;

	/* Should be populated at probe. */
	bool can_railgate_init;

	/* Should be populated at probe. */
	bool can_tpc_powergate;

	/* Should be populated at probe. */
	bool can_elpg_init;

	/* Should be populated at probe. */
	bool has_syncpoints;

	/* channel limit after which to start aggressive sync destroy */
	unsigned int aggressive_sync_destroy_thresh;

	/* flag to set sync destroy aggressiveness */
	bool aggressive_sync_destroy;

	/* set if ASPM should be disabled on boot; only makes sense for PCI */
	bool disable_aspm;

	/* Set if the platform can unify the small/large address spaces. */
	bool unify_address_spaces;

	/* Clock configuration is stored here. Platform probe is responsible
	 * for filling this data. */
	struct clk *clk[GK20A_CLKS_MAX];
	int num_clks;
	int maxmin_clk_id;

#ifdef CONFIG_RESET_CONTROLLER
	/* Reset control for device */
	struct reset_control *reset_control;
#endif
	/* valid TPC-MASK */
	u32 valid_tpc_mask[MAX_TPC_PG_CONFIGS];

	/* Delay before rail gated */
	int railgate_delay_init;

	/* init value for slowdown factor */
	u8 ldiv_slowdown_factor_init;

	/* Second Level Clock Gating: true = enable false = disable */
	bool enable_slcg;

	/* Block Level Clock Gating: true = enable flase = disable */
	bool enable_blcg;

	/* Engine Level Clock Gating: true = enable flase = disable */
	bool enable_elcg;

	/* Should be populated at probe. */
	bool can_slcg;

	/* Should be populated at probe. */
	bool can_blcg;

	/* Should be populated at probe. */
	bool can_elcg;

	/* Engine Level Power Gating: true = enable flase = disable */
	bool enable_elpg;

	/* Adaptative ELPG: true = enable flase = disable */
	bool enable_aelpg;

	/* PMU Perfmon: true = enable false = disable */
	bool enable_perfmon;

	/* Memory System Clock Gating: true = enable flase = disable*/
	bool enable_mscg;

	/* Timeout for per-channel watchdog (in mS) */
	u32 ch_wdt_timeout_ms;

	/* Disable big page support */
	bool disable_bigpage;

	/*
	 * gk20a_do_idle() API can take GPU either into rail gate or CAR reset
	 * This flag can be used to force CAR reset case instead of rail gate
	 */
	bool force_reset_in_do_idle;

	/* guest/vm id, needed for IPA to PA transation */
	int vmid;

	/* Initialize the platform interface of the gk20a driver.
	 *
	 * The platform implementation of this function must
	 *   - set the power and clocks of the gk20a device to a known
	 *     state, and
	 *   - populate the gk20a_platform structure (a pointer to the
	 *     structure can be obtained by calling gk20a_get_platform).
	 *
	 * After this function is finished, the driver will initialise
	 * pm runtime and genpd based on the platform configuration.
	 */
	int (*probe)(struct device *dev);

	/* Second stage initialisation - called once all power management
	 * initialisations are done.
	 */
	int (*late_probe)(struct device *dev);

	/* Remove device after power management has been done
	 */
	int (*remove)(struct device *dev);

	/* Poweron platform dependencies */
	int (*busy)(struct device *dev);

	/* Powerdown platform dependencies */
	void (*idle)(struct device *dev);

	/* Preallocated VPR buffer for kernel */
	size_t secure_buffer_size;
	struct secure_page_buffer secure_buffer;

	/* Device is going to be suspended */
	int (*suspend)(struct device *);

	/* Device is going to be resumed */
	int (*resume)(struct device *);

	/* Called to turn off the device */
	int (*railgate)(struct device *dev);

	/* Called to turn on the device */
	int (*unrailgate)(struct device *dev);
	struct nvgpu_mutex railgate_lock;

	/* Called to check state of device */
	bool (*is_railgated)(struct device *dev);

	/* get supported frequency list */
	int (*get_clk_freqs)(struct device *pdev,
				unsigned long **freqs, int *num_freqs);

	/* clk related supported functions */
	long (*clk_round_rate)(struct device *dev,
				unsigned long rate);

	/* Called to register GPCPLL with common clk framework */
	int (*clk_register)(struct gk20a *g);

	/* platform specific scale init quirks */
	void (*initscale)(struct device *dev);

	/* Postscale callback is called after frequency change */
	void (*postscale)(struct device *dev,
			  unsigned long freq);

	/* Pre callback is called before frequency change */
	void (*prescale)(struct device *dev);

	/* Set TPC_PG_MASK during probe */
	void (*set_tpc_pg_mask)(struct device *dev, u32 tpc_pg_mask);

	/* Devfreq governor name. If scaling is enabled, we request
	 * this governor to be used in scaling */
	const char *devfreq_governor;

	/* Quality of service notifier callback. If this is set, the scaling
	 * routines will register a callback to Qos. Each time we receive
	 * a new value, this callback gets called.  */
	int (*qos_notify)(struct notifier_block *nb,
			  unsigned long n, void *p);

	/* Called as part of debug dump. If the gpu gets hung, this function
	 * is responsible for delivering all necessary debug data of other
	 * hw units which may interact with the gpu without direct supervision
	 * of the CPU.
	 */
	void (*dump_platform_dependencies)(struct device *dev);

	/* Defined when SMMU stage-2 is enabled, and we need to use physical
	 * addresses (not IPA). This is the case for GV100 nvlink in HV+L
	 * configuration, when dGPU is in pass-through mode.
	 */
	u64 (*phys_addr)(struct gk20a *g, u64 ipa);

	/* Callbacks to assert/deassert GPU reset */
	int (*reset_assert)(struct device *dev);
	int (*reset_deassert)(struct device *dev);
	struct clk *clk_reset;
	struct dvfs_rail *gpu_rail;

	bool virtual_dev;
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	void *vgpu_priv;
#endif
	/* source frequency for ptimer in hz */
	u32 ptimer_src_freq;

#ifdef CONFIG_NVGPU_SUPPORT_CDE
	bool has_cde;
#endif

	/* soc name for finding firmware files */
	const char *soc_name;

	/* false if vidmem aperture actually points to sysmem */
	bool honors_aperture;
	/* unified or split memory with separate vidmem? */
	bool unified_memory;
	/* WAR for gm20b chips. */
	bool force_128K_pmu_vm;

	/*
	 * DMA mask for Linux (both coh and non-coh). If not set defaults to
	 * 0x3ffffffff (i.e a 34 bit mask).
	 */
	u64 dma_mask;

	/* minimum supported VBIOS version */
	u32 vbios_min_version;

	/* true if we run preos microcode on this board */
	bool run_preos;

	/* true if we need to program sw threshold for
         * power limits
	 */
	bool hardcode_sw_threshold;

	/* i2c device index, port and address for INA3221 */
	u32 ina3221_dcb_index;
	u32 ina3221_i2c_address;
	u32 ina3221_i2c_port;

	/* stream id to use */
	u32 ltc_streamid;

	/* synchronized access to platform->clk_get_freqs */
	struct nvgpu_mutex clk_get_freq_lock;
};

static inline struct gk20a_platform *gk20a_get_platform(
		struct device *dev)
{
	return (struct gk20a_platform *)dev_get_drvdata(dev);
}

#ifdef CONFIG_TEGRA_GK20A
extern struct gk20a_platform gm20b_tegra_platform;
extern struct gk20a_platform gp10b_tegra_platform;
extern struct gk20a_platform gv11b_tegra_platform;
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
extern struct gk20a_platform vgpu_tegra_platform;
extern struct gk20a_platform gv11b_vgpu_tegra_platform;
#endif
#endif

int gk20a_tegra_busy(struct device *dev);
void gk20a_tegra_idle(struct device *dev);
void gk20a_tegra_debug_dump(struct device *pdev);

static inline struct gk20a *get_gk20a(struct device *dev)
{
	return gk20a_get_platform(dev)->g;
}
static inline struct gk20a *gk20a_from_dev(struct device *dev)
{
	if (!dev)
		return NULL;

	return ((struct gk20a_platform *)dev_get_drvdata(dev))->g;
}
static inline bool gk20a_gpu_is_virtual(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	return platform->virtual_dev;
}

static inline int support_gk20a_pmu(struct device *dev)
{
	if (IS_ENABLED(CONFIG_GK20A_PMU)) {
		/* gPMU is not supported for vgpu */
		return !gk20a_gpu_is_virtual(dev);
	}

	return 0;
}

#endif
