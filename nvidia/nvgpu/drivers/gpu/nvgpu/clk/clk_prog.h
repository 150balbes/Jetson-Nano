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

#ifndef NVGPU_CLK_PROG_H
#define NVGPU_CLK_PROG_H
#include "ctrl/ctrlclk.h"
#include "ctrl/ctrlboardobj.h"
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include "boardobj/boardobjgrp_e32.h"
#include "boardobj/boardobjgrp_e255.h"
#include "boardobj/boardobjgrpmask.h"

int clk_prog_sw_setup(struct gk20a *g);
int clk_prog_pmu_setup(struct gk20a *g);
struct clk_prog_1x_master;

typedef u32 vf_flatten(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_prog_1x_master *p1xmaster,
			u8 clk_domain_idx, u16 *pfreqmaxlastmhz);

typedef u32 vf_lookup(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_prog_1x_master *p1xmaster,
			u8 *slave_clk_domain_idx, u16 *pclkmhz,
			u32 *pvoltuv, u8 rail);

typedef int get_slaveclk(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_prog_1x_master *p1xmaster,
			u8 slave_clk_domain_idx, u16 *pclkmhz,
			u16 masterclkmhz);

typedef u32 get_fpoints(struct gk20a *g, struct clk_pmupstate *pclk,
			struct clk_prog_1x_master *p1xmaster,
			u32 *pfpointscount,
			u16 **ppfreqpointsinmhz, u8 rail);


struct clk_progs {
	struct boardobjgrp_e255 super;
	u8 slave_entry_count;
	u8 vf_entry_count;

};

struct clk_prog {
	struct boardobj super;
};

struct clk_prog_1x {
	struct clk_prog super;
	u8  source;
	u16 freq_max_mhz;
	union ctrl_clk_clk_prog_1x_source_data source_data;
};

struct clk_prog_1x_master {
	struct clk_prog_1x super;
	bool b_o_c_o_v_enabled;
	struct ctrl_clk_clk_prog_1x_master_vf_entry *p_vf_entries;
	struct ctrl_clk_clk_delta deltas;
	union ctrl_clk_clk_prog_1x_master_source_data source_data;
	vf_flatten *vfflatten;
	vf_lookup *vflookup;
	get_fpoints *getfpoints;
	get_slaveclk *getslaveclk;
};

struct clk_prog_1x_master_ratio {
	struct clk_prog_1x_master super;
	struct ctrl_clk_clk_prog_1x_master_ratio_slave_entry *p_slave_entries;
};

struct clk_prog_1x_master_table {
	struct clk_prog_1x_master super;
	struct ctrl_clk_clk_prog_1x_master_table_slave_entry *p_slave_entries;
};

#define CLK_CLK_PROG_GET(pclk, idx)                                            \
	((struct clk_prog *)BOARDOBJGRP_OBJ_GET_BY_IDX(			\
		&pclk->clk_progobjs.super.super, (u8)(idx)))

#endif /* NVGPU_CLK_PROG_H */
