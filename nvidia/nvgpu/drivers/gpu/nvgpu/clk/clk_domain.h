/*
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

#ifndef NVGPU_CLK_DOMAIN_H
#define NVGPU_CLK_DOMAIN_H

#include "ctrl/ctrlclk.h"
#include "ctrl/ctrlboardobj.h"
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include "boardobj/boardobjgrp_e32.h"
#include "boardobj/boardobjgrpmask.h"

#define CLK_DOMAIN_BOARDOBJGRP_VERSION 0x30
#define CLK_TABLE_HAL_ENTRY_GP 0x02
#define CLK_TABLE_HAL_ENTRY_GV 0x03

struct clk_domains;
struct clk_domain;
enum nv_pmu_clk_clkwhich;

/*data and function definition to talk to driver*/
int clk_domain_sw_setup(struct gk20a *g);
int clk_domain_pmu_setup(struct gk20a *g);

typedef int clkproglink(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_domain *pdomain);

typedef int clkvfsearch(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_domain *pdomain, u16 *clkmhz,
			u32 *voltuv, u8 rail);

typedef int clkgetslaveclk(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_domain *pdomain, u16 *clkmhz,
			u16 masterclkmhz);

typedef u32 clkgetfpoints(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_domain *pdomain, u32 *pfpointscount,
			  u16 *pfreqpointsinmhz, u8 rail);

struct clk_domains {
	struct boardobjgrp_e32 super;
	u8 n_num_entries;
	u8 version;
	bool b_enforce_vf_monotonicity;
	bool b_enforce_vf_smoothening;
	bool b_override_o_v_o_c;
	bool b_debug_mode;
	u32 vbios_domains;
	u16 cntr_sampling_periodms;
	struct boardobjgrpmask_e32 prog_domains_mask;
	struct boardobjgrpmask_e32 master_domains_mask;
	struct ctrl_clk_clk_delta  deltas;

	struct clk_domain *ordered_noise_aware_list[CTRL_BOARDOBJ_MAX_BOARD_OBJECTS];

	struct clk_domain *ordered_noise_unaware_list[CTRL_BOARDOBJ_MAX_BOARD_OBJECTS];
};

struct clk_domain {
	struct boardobj super;
	u32 api_domain;
	u32 part_mask;
	enum nv_pmu_clk_clkwhich domain;
	u8 perf_domain_index;
	u8 perf_domain_grp_idx;
	u8 ratio_domain;
	u8 usage;
	clkproglink *clkdomainclkproglink;
	clkvfsearch *clkdomainclkvfsearch;
	clkgetfpoints *clkdomainclkgetfpoints;
};

struct clk_domain_3x {
	struct clk_domain super;
	bool b_noise_aware_capable;
};

struct clk_domain_3x_fixed {
	struct clk_domain_3x super;
	u16  freq_mhz;
};

struct clk_domain_3x_prog {
	struct clk_domain_3x super;
	u8  clk_prog_idx_first;
	u8  clk_prog_idx_last;
	bool b_force_noise_unaware_ordering;
	struct ctrl_clk_freq_delta factory_delta;
	short freq_delta_min_mhz;
	short freq_delta_max_mhz;
	struct ctrl_clk_clk_delta deltas;
	u8 noise_unaware_ordering_index;
	u8 noise_aware_ordering_index;
};

struct clk_domain_35_prog {
	struct clk_domain_3x_prog super;
	u8 pre_volt_ordering_index;
	u8 post_volt_ordering_index;
	u8 clk_pos;
	u8 clk_vf_curve_count;
};

struct clk_domain_3x_master {
	struct clk_domain_3x_prog super;
	u32 slave_idxs_mask;
};

struct clk_domain_35_master {
	struct clk_domain_35_prog super;
	struct clk_domain_3x_master master;
	struct boardobjgrpmask_e32 master_slave_domains_grp_mask;
};

struct clk_domain_3x_slave {
	struct clk_domain_3x_prog super;
	u8 master_idx;
	clkgetslaveclk *clkdomainclkgetslaveclk;
};

struct clk_domain_30_slave {
	u8 rsvd;
	u8 master_idx;
	clkgetslaveclk *clkdomainclkgetslaveclk;
};

struct clk_domain_35_slave {
	struct clk_domain_35_prog super;
	struct clk_domain_30_slave slave;
};

int clk_domain_clk_prog_link(struct gk20a *g, struct clk_pmupstate *pclk);

#define CLK_CLK_DOMAIN_GET(pclk, idx)                                   \
	((struct clk_domain *)BOARDOBJGRP_OBJ_GET_BY_IDX(		\
		&pclk->clk_domainobjs.super.super, (u8)(idx)))

#endif /* NVGPU_CLK_DOMAIN_H */
