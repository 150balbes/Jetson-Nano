/*
 * GP106 Clocks
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

#include <nvgpu/kmem.h>
#include <nvgpu/io.h>
#include <nvgpu/list.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/timers.h>
#include <nvgpu/pmu.h>
#include <nvgpu/gk20a.h>

#include "clk/clk.h"
#include "gp106/mclk_gp106.h"

#include "clk_gp106.h"

#include <nvgpu/hw/gp106/hw_trim_gp106.h>

#define NUM_NAMEMAPS	4
#define XTAL4X_KHZ 108000

u32 gp106_crystal_clk_hz(struct gk20a *g)
{
	return (XTAL4X_KHZ * 1000);
}

unsigned long gp106_clk_measure_freq(struct gk20a *g, u32 api_domain)
{
	struct clk_gk20a *clk = &g->clk;
	u32 freq_khz;
	u32 i;
	struct namemap_cfg *c = NULL;

	for (i = 0; i < clk->namemap_num; i++) {
		if (api_domain == clk->namemap_xlat_table[i]) {
			c = &clk->clk_namemap[i];
			break;
		}
	}

	if (!c) {
		return 0;
	}

	freq_khz = c->is_counter ? c->scale * gp106_get_rate_cntr(g, c) :
		0; /* TODO: PLL read */

	/* Convert to HZ */
	return freq_khz * 1000UL;
}

int gp106_init_clk_support(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	u32 err = 0;

	nvgpu_log_fn(g, " ");

	err = nvgpu_mutex_init(&clk->clk_mutex);
	if (err) {
		return err;
	}

	clk->clk_namemap = (struct namemap_cfg *)
		nvgpu_kzalloc(g, sizeof(struct namemap_cfg) * NUM_NAMEMAPS);

	if (!clk->clk_namemap) {
		nvgpu_mutex_destroy(&clk->clk_mutex);
		return -ENOMEM;
	}

	clk->namemap_xlat_table = nvgpu_kcalloc(g, NUM_NAMEMAPS, sizeof(u32));

	if (!clk->namemap_xlat_table) {
		nvgpu_kfree(g, clk->clk_namemap);
		nvgpu_mutex_destroy(&clk->clk_mutex);
		return -ENOMEM;
	}

	clk->clk_namemap[0] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_GPC2CLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr = {
			.reg_ctrl_addr = trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_r(),
			.reg_ctrl_idx  = trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_source_gpc2clk_f(),
			.reg_cntr_addr = trim_gpc_bcast_clk_cntr_ncgpcclk_cnt_r()
		},
		.name = "gpc2clk",
		.scale = 1
	};
	clk->namemap_xlat_table[0] = CTRL_CLK_DOMAIN_GPC2CLK;

	clk->clk_namemap[1] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_SYS2CLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr = {
			.reg_ctrl_addr = trim_sys_clk_cntr_ncsyspll_cfg_r(),
			.reg_ctrl_idx  = trim_sys_clk_cntr_ncsyspll_cfg_source_sys2clk_f(),
			.reg_cntr_addr = trim_sys_clk_cntr_ncsyspll_cnt_r()
		},
		.name = "sys2clk",
		.scale = 1
	};
	clk->namemap_xlat_table[1] = CTRL_CLK_DOMAIN_SYS2CLK;

	clk->clk_namemap[2] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_XBAR2CLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr = {
			.reg_ctrl_addr = trim_sys_clk_cntr_ncltcpll_cfg_r(),
			.reg_ctrl_idx  = trim_sys_clk_cntr_ncltcpll_cfg_source_xbar2clk_f(),
			.reg_cntr_addr = trim_sys_clk_cntr_ncltcpll_cnt_r()
		},
		.name = "xbar2clk",
		.scale = 1
	};
	clk->namemap_xlat_table[2] = CTRL_CLK_DOMAIN_XBAR2CLK;

	clk->clk_namemap[3] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_DRAMCLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr = {
			.reg_ctrl_addr = trim_fbpa_bcast_clk_cntr_ncltcclk_cfg_r(),
			.reg_ctrl_idx  = trim_fbpa_bcast_clk_cntr_ncltcclk_cfg_source_dramdiv4_rec_clk1_f(),
			.reg_cntr_addr = trim_fbpa_bcast_clk_cntr_ncltcclk_cnt_r()
		},
		.name = "dramdiv4_rec_clk1",
		.scale = 4
	};
	clk->namemap_xlat_table[3] = CTRL_CLK_DOMAIN_MCLK;

	clk->namemap_num = NUM_NAMEMAPS;

	clk->g = g;

	return err;
}

u32 gp106_get_rate_cntr(struct gk20a *g, struct namemap_cfg *c)
{
	u32 save_reg;
	u32 retries;
	u32 cntr = 0;

	struct clk_gk20a *clk = &g->clk;

	if (!c || !c->cntr.reg_ctrl_addr || !c->cntr.reg_cntr_addr) {
		return 0;
	}

	nvgpu_mutex_acquire(&clk->clk_mutex);

	/* Save the register */
	save_reg = gk20a_readl(g, c->cntr.reg_ctrl_addr);

	/* Disable and reset the current clock */
	gk20a_writel(g, c->cntr.reg_ctrl_addr,
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_reset_asserted_f() |
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_enable_deasserted_f());

	/* Force wb() */
	(void) gk20a_readl(g, c->cntr.reg_ctrl_addr);

	/* Wait for reset to happen */
	retries = CLK_DEFAULT_CNTRL_SETTLE_RETRIES;
	do {
		nvgpu_udelay(CLK_DEFAULT_CNTRL_SETTLE_USECS);
	} while ((--retries) && (cntr = gk20a_readl(g, c->cntr.reg_cntr_addr)));

	if (!retries) {
		nvgpu_err(g, "unable to settle counter reset, bailing");
		goto read_err;
	}
	/* Program counter */
	gk20a_writel(g, c->cntr.reg_ctrl_addr,
					trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_reset_deasserted_f()          |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_enable_asserted_f()           |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_write_en_asserted_f()         |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_write_en_asserted_f()         |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_write_en_asserted_f()         |
				 	trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_noofipclks_f(XTAL_CNTR_CLKS)  |
					c->cntr.reg_ctrl_idx);
	(void) gk20a_readl(g, c->cntr.reg_ctrl_addr);

	nvgpu_udelay(XTAL_CNTR_DELAY);

	cntr = XTAL_SCALE_TO_KHZ * gk20a_readl(g, c->cntr.reg_cntr_addr);

read_err:
	/* reset and restore control register */
	gk20a_writel(g, c->cntr.reg_ctrl_addr,
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_reset_asserted_f() |
				 trim_gpc_bcast_clk_cntr_ncgpcclk_cfg_enable_deasserted_f());
	(void) gk20a_readl(g, c->cntr.reg_ctrl_addr);
	gk20a_writel(g, c->cntr.reg_ctrl_addr, save_reg);
	(void) gk20a_readl(g, c->cntr.reg_ctrl_addr);
	nvgpu_mutex_release(&clk->clk_mutex);

	return cntr;

}

int gp106_clk_domain_get_f_points(
	struct gk20a *g,
	u32 clkapidomain,
	u32 *pfpointscount,
	u16 *pfreqpointsinmhz)
{
	int status = -EINVAL;
	struct clk_domain *pdomain;
	u8 i;
	struct clk_pmupstate *pclk = &g->clk_pmu;

	if (pfpointscount == NULL)
		return -EINVAL;

	if ((pfreqpointsinmhz == NULL) && (*pfpointscount != 0))
		return -EINVAL;

	BOARDOBJGRP_FOR_EACH(&(pclk->clk_domainobjs.super.super),
			struct clk_domain *, pdomain, i) {
		if (pdomain->api_domain == clkapidomain) {
			status = pdomain->clkdomainclkgetfpoints(g, pclk,
				pdomain, pfpointscount,
				pfreqpointsinmhz,
				CLK_PROG_VFE_ENTRY_LOGIC);
			return status;
		}
	}
	return status;
}

int gp106_suspend_clk_support(struct gk20a *g)
{
	nvgpu_mutex_destroy(&g->clk.clk_mutex);
	return 0;
}
