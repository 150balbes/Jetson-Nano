/*
 * GV100 Clocks
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

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include "os/linux/os_linux.h"
#endif

#include <nvgpu/kmem.h>
#include <nvgpu/io.h>
#include <nvgpu/list.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/timers.h>

#include "gk20a/gk20a.h"

#include "clk_gv100.h"

#include <nvgpu/hw/gv100/hw_trim_gv100.h>


u32 gv100_crystal_clk_hz(struct gk20a *g)
{
	return (XTAL4X_KHZ * 1000);
}

unsigned long gv100_clk_measure_freq(struct gk20a *g, u32 api_domain)
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

	if (c == NULL) {
		return 0;
	}
	if (c->is_counter != 0U) {
		freq_khz = c->scale * gv100_get_rate_cntr(g, c);
	} else {
		freq_khz = 0U;
		 /* TODO: PLL read */
	}

	/* Convert to HZ */
	return freq_khz * 1000UL;
}

int gv100_init_clk_support(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	int err = 0;

	nvgpu_log_fn(g, " ");

	err = nvgpu_mutex_init(&clk->clk_mutex);
	if (err != 0) {
		return err;
	}

	clk->clk_namemap = (struct namemap_cfg *)
		nvgpu_kzalloc(g, sizeof(struct namemap_cfg) * NUM_NAMEMAPS);

	if (clk->clk_namemap == NULL) {
		nvgpu_mutex_destroy(&clk->clk_mutex);
		return -ENOMEM;
	}

	clk->namemap_xlat_table = nvgpu_kcalloc(g, NUM_NAMEMAPS, sizeof(u32));

	if (clk->namemap_xlat_table == NULL) {
		nvgpu_kfree(g, clk->clk_namemap);
		nvgpu_mutex_destroy(&clk->clk_mutex);
		return -ENOMEM;
	}

	clk->clk_namemap[0] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_GPCCLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr = {
			.reg_ctrl_addr = trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cfg_r(),
			.reg_ctrl_idx  = trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cfg_source_gpcclk_f(),
			.reg_cntr_addr[0] = trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cnt0_r(),
			.reg_cntr_addr[1] = trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cnt1_r()
		},
		.name = "gpcclk",
		.scale = 1
	};
	clk->namemap_xlat_table[0] = CTRL_CLK_DOMAIN_GPCCLK;

	clk->clk_namemap[1] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_SYSCLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr = {
			.reg_ctrl_addr = trim_sys_fr_clk_cntr_sysclk_cfg_r(),
			.reg_ctrl_idx  = trim_sys_fr_clk_cntr_sysclk_cfg_source_sysclk_f(),
			.reg_cntr_addr[0] = trim_sys_fr_clk_cntr_sysclk_cntr0_r(),
			.reg_cntr_addr[1] = trim_sys_fr_clk_cntr_sysclk_cntr1_r()
		},
		.name = "sysclk",
		.scale = 1
	};
	clk->namemap_xlat_table[1] = CTRL_CLK_DOMAIN_SYSCLK;

	clk->clk_namemap[2] = (struct namemap_cfg) {
		.namemap = CLK_NAMEMAP_INDEX_XBARCLK,
		.is_enable = 1,
		.is_counter = 1,
		.g = g,
		.cntr = {
			.reg_ctrl_addr = trim_sys_nafll_fr_clk_cntr_xbarclk_cfg_r(),
			.reg_ctrl_idx  = trim_sys_nafll_fr_clk_cntr_xbarclk_cfg_source_xbarclk_f(),
			.reg_cntr_addr[0] = trim_sys_nafll_fr_clk_cntr_xbarclk_cntr0_r(),
			.reg_cntr_addr[1] = trim_sys_nafll_fr_clk_cntr_xbarclk_cntr1_r()
		},
		.name = "xbarclk",
		.scale = 1
	};
	clk->namemap_xlat_table[2] = CTRL_CLK_DOMAIN_XBARCLK;

	clk->namemap_num = NUM_NAMEMAPS;

	clk->g = g;

	return err;
}

u32 gv100_get_rate_cntr(struct gk20a *g, struct namemap_cfg *c) {
	u32 cntr = 0;
	u64 cntr_start = 0;
	u64 cntr_stop = 0;

	struct clk_gk20a *clk = &g->clk;

	if ((c == NULL) || (c->cntr.reg_ctrl_addr == 0U) ||
		(c->cntr.reg_cntr_addr[0] == 0U) ||
		(c->cntr.reg_cntr_addr[1]) == 0U) {
			return 0;
	}

	nvgpu_mutex_acquire(&clk->clk_mutex);

	/* Read the counter values */
	/* Counter is 36bits , 32 bits on addr[0] and 4 lsb on addr[1] others zero*/
	cntr_start = (u64)gk20a_readl(g, c->cntr.reg_cntr_addr[0]);
	cntr_start += ((u64)gk20a_readl(g, c->cntr.reg_cntr_addr[1]) << 32);
	nvgpu_udelay(XTAL_CNTR_DELAY);
	cntr_stop = (u64) gk20a_readl(g, c->cntr.reg_cntr_addr[0]);
	cntr_stop += ((u64)gk20a_readl(g, c->cntr.reg_cntr_addr[1]) << 32);
	/*Calculate the difference and convert to KHz*/
	cntr = (u32)((cntr_stop - cntr_start) / 10ULL);
	nvgpu_mutex_release(&clk->clk_mutex);

	return cntr;

}

int gv100_suspend_clk_support(struct gk20a *g)
{
	nvgpu_mutex_destroy(&g->clk.clk_mutex);
	return 0;
}
