/*
 * general clock structures & definitions
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_CLK_H
#define NVGPU_CLK_H

#include "clk_vin.h"
#include "clk_fll.h"
#include "clk_domain.h"
#include "clk_prog.h"
#include "clk_vf_point.h"
#include "clk_mclk.h"
#include "clk_freq_controller.h"

#define NV_PERF_DOMAIN_4X_CLOCK_DOMAIN_SKIP 0x10
#define NV_PERF_DOMAIN_4X_CLOCK_DOMAIN_MASK 0x1F
#define NV_PERF_DOMAIN_4X_CLOCK_DOMAIN_SHIFT 0
#define BOOT_GPCCLK_MHZ 952

struct gk20a;

int clk_set_boot_fll_clk(struct gk20a *g);

/* clock related defines for GPUs supporting clock control from pmu*/
struct clk_pmupstate {
	struct avfsvinobjs avfs_vinobjs;
	struct avfsfllobjs avfs_fllobjs;
	struct clk_domains clk_domainobjs;
	struct clk_progs clk_progobjs;
	struct clk_vf_points clk_vf_pointobjs;
	struct clk_mclk_state clk_mclk;
	struct clk_freq_controllers clk_freq_controllers;
};

struct clockentry {
		u8 vbios_clk_domain;
		u8 clk_which;
		u8 perf_index;
		u32 api_clk_domain;
};

struct change_fll_clk {
		u32 api_clk_domain;
		u16 clkmhz;
		u32 voltuv;
};

struct set_fll_clk {
		u32 voltuv;
		u16 gpc2clkmhz;
		u32 current_regime_id_gpc;
		u32 target_regime_id_gpc;
		u16 sys2clkmhz;
		u32 current_regime_id_sys;
		u32 target_regime_id_sys;
		u16 xbar2clkmhz;
		u32 current_regime_id_xbar;
		u32 target_regime_id_xbar;
};

#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_MAX_NUMCLKS         9

struct vbios_clock_domain {
	u8 clock_type;
	u8 num_domains;
	struct clockentry clock_entry[NV_PERF_HEADER_4X_CLOCKS_DOMAINS_MAX_NUMCLKS];
};

struct vbios_clocks_table_1x_hal_clock_entry {
	enum nv_pmu_clk_clkwhich domain;
	bool b_noise_aware_capable;
	u8 clk_vf_curve_count;
};

#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_GPC2CLK           0
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_XBAR2CLK          1
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_DRAMCLK           2
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_SYS2CLK           3
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_HUB2CLK           4
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_MSDCLK            5
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_PWRCLK            6
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_DISPCLK           7
#define NV_PERF_HEADER_4X_CLOCKS_DOMAINS_4_NUMCLKS           8

#define PERF_CLK_MCLK           0
#define PERF_CLK_DISPCLK        1
#define PERF_CLK_GPC2CLK        2
#define PERF_CLK_HOSTCLK        3
#define PERF_CLK_LTC2CLK        4
#define PERF_CLK_SYS2CLK        5
#define PERF_CLK_HUB2CLK        6
#define PERF_CLK_LEGCLK         7
#define PERF_CLK_MSDCLK         8
#define PERF_CLK_XCLK           9
#define PERF_CLK_PWRCLK         10
#define PERF_CLK_XBAR2CLK       11
#define PERF_CLK_PCIEGENCLK     12
#define PERF_CLK_NUM            13

#define BOOT_GPC2CLK_MHZ  2581

u32 clk_pmu_vin_load(struct gk20a *g);
u32 clk_domain_print_vf_table(struct gk20a *g, u32 clkapidomain);
u32 clk_domain_get_f_or_v(
	struct gk20a *g,
	u32 clkapidomain,
	u16 *pclkmhz,
	u32 *pvoltuv,
	u8 railidx
);
int clk_get_fll_clks(struct gk20a *g, struct set_fll_clk *fllclk);
int clk_set_fll_clks(struct gk20a *g, struct set_fll_clk *fllclk);
int clk_pmu_freq_controller_load(struct gk20a *g, bool bload, u8 bit_idx);
u32 nvgpu_clk_vf_change_inject_data_fill_gv10x(struct gk20a *g,
	struct nv_pmu_clk_rpc *rpccall,
	struct set_fll_clk *setfllclk);
u32 nvgpu_clk_vf_change_inject_data_fill_gp10x(struct gk20a *g,
	struct nv_pmu_clk_rpc *rpccall,
	struct set_fll_clk *setfllclk);
u32 nvgpu_clk_set_boot_fll_clk_gv10x(struct gk20a *g);
int nvgpu_clk_set_fll_clk_gv10x(struct gk20a *g);
int clk_pmu_freq_effective_avg_load(struct gk20a *g, bool bload);
u32 clk_freq_effective_avg(struct gk20a *g, u32  clkDomainMask);
#endif /* NVGPU_CLK_H */
