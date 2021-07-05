/*
 * GM20B GPC MMU
 *
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <trace/events/gk20a.h>

#include <nvgpu/sizes.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include "fb_gm20b.h"

#include <nvgpu/io.h>
#include <nvgpu/timers.h>

#include <nvgpu/hw/gm20b/hw_fb_gm20b.h>

#define VPR_INFO_FETCH_WAIT	(5)
#define WPR_INFO_ADDR_ALIGNMENT 0x0000000c

void gm20b_fb_init_hw(struct gk20a *g)
{
	u64 addr = nvgpu_mem_get_addr(g, &g->mm.sysmem_flush) >> 8;

	gk20a_writel(g, fb_niso_flush_sysmem_addr_r(), addr);

	/* init mmu debug buffer */
	addr = nvgpu_mem_get_addr(g, &g->mm.mmu_wr_mem);
	addr >>= fb_mmu_debug_wr_addr_alignment_v();

	gk20a_writel(g, fb_mmu_debug_wr_r(),
		     nvgpu_aperture_mask(g, &g->mm.mmu_wr_mem,
				fb_mmu_debug_wr_aperture_sys_mem_ncoh_f(),
				fb_mmu_debug_wr_aperture_sys_mem_coh_f(),
				fb_mmu_debug_wr_aperture_vid_mem_f()) |
		     fb_mmu_debug_wr_vol_false_f() |
		     fb_mmu_debug_wr_addr_f(addr));

	addr = nvgpu_mem_get_addr(g, &g->mm.mmu_rd_mem);
	addr >>= fb_mmu_debug_rd_addr_alignment_v();

	gk20a_writel(g, fb_mmu_debug_rd_r(),
		     nvgpu_aperture_mask(g, &g->mm.mmu_rd_mem,
				fb_mmu_debug_wr_aperture_sys_mem_ncoh_f(),
				fb_mmu_debug_wr_aperture_sys_mem_coh_f(),
				fb_mmu_debug_rd_aperture_vid_mem_f()) |
		     fb_mmu_debug_rd_vol_false_f() |
		     fb_mmu_debug_rd_addr_f(addr));
}

int gm20b_fb_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb)
{
	struct nvgpu_timeout timeout;
	u32 addr_lo;
	u32 data;
	int err = 0;

	nvgpu_log_fn(g, " ");

	/* pagetables are considered sw states which are preserved after
	   prepare_poweroff. When gk20a deinit releases those pagetables,
	   common code in vm unmap path calls tlb invalidate that touches
	   hw. Use the power_on flag to skip tlb invalidation when gpu
	   power is turned off */

	if (!g->power_on) {
		return err;
	}

	addr_lo = u64_lo32(nvgpu_mem_get_addr(g, pdb) >> 12);

	nvgpu_mutex_acquire(&g->mm.tlb_lock);

	trace_gk20a_mm_tlb_invalidate(g->name);

	nvgpu_timeout_init(g, &timeout, 1000, NVGPU_TIMER_RETRY_TIMER);

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_space_v(data) != 0) {
			break;
		}
		nvgpu_udelay(2);
	} while (!nvgpu_timeout_expired_msg(&timeout,
					 "wait mmu fifo space"));

	if (nvgpu_timeout_peek_expired(&timeout)) {
		err = -ETIMEDOUT;
		goto out;
	}

	nvgpu_timeout_init(g, &timeout, 1000, NVGPU_TIMER_RETRY_TIMER);

	gk20a_writel(g, fb_mmu_invalidate_pdb_r(),
		fb_mmu_invalidate_pdb_addr_f(addr_lo) |
		nvgpu_aperture_mask(g, pdb,
				fb_mmu_invalidate_pdb_aperture_sys_mem_f(),
				fb_mmu_invalidate_pdb_aperture_sys_mem_f(),
				fb_mmu_invalidate_pdb_aperture_vid_mem_f()));

	gk20a_writel(g, fb_mmu_invalidate_r(),
		fb_mmu_invalidate_all_va_true_f() |
		fb_mmu_invalidate_trigger_true_f());

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_empty_v(data) !=
			fb_mmu_ctrl_pri_fifo_empty_false_f()) {
			break;
		}
		nvgpu_udelay(2);
	} while (!nvgpu_timeout_expired_msg(&timeout,
					 "wait mmu invalidate"));

	trace_gk20a_mm_tlb_invalidate_done(g->name);

out:
	nvgpu_mutex_release(&g->mm.tlb_lock);
	return err;
}

void fb_gm20b_init_fs_state(struct gk20a *g)
{
	nvgpu_log_info(g, "initialize gm20b fb");

	gk20a_writel(g, fb_fbhub_num_active_ltcs_r(),
			g->ltc_count);

	if (!nvgpu_is_enabled(g, NVGPU_SEC_PRIVSECURITY)) {
		/* Bypass MMU check for non-secure boot. For
		 * secure-boot,this register write has no-effect */
		gk20a_writel(g, fb_priv_mmu_phy_secure_r(), 0xffffffffU);
	}
}

void gm20b_fb_set_mmu_page_size(struct gk20a *g)
{
	/* set large page size in fb */
	u32 fb_mmu_ctrl = gk20a_readl(g, fb_mmu_ctrl_r());
	fb_mmu_ctrl |= fb_mmu_ctrl_use_pdb_big_page_size_true_f();
	gk20a_writel(g, fb_mmu_ctrl_r(), fb_mmu_ctrl);
}

bool gm20b_fb_set_use_full_comp_tag_line(struct gk20a *g)
{
	/* set large page size in fb */
	u32 fb_mmu_ctrl = gk20a_readl(g, fb_mmu_ctrl_r());
	fb_mmu_ctrl |= fb_mmu_ctrl_use_full_comp_tag_line_true_f();
	gk20a_writel(g, fb_mmu_ctrl_r(), fb_mmu_ctrl);

	return true;
}

u32 gm20b_fb_mmu_ctrl(struct gk20a *g)
{
	return gk20a_readl(g, fb_mmu_ctrl_r());
}

u32 gm20b_fb_mmu_debug_ctrl(struct gk20a *g)
{
	return gk20a_readl(g, fb_mmu_debug_ctrl_r());
}

u32 gm20b_fb_mmu_debug_wr(struct gk20a *g)
{
	return gk20a_readl(g, fb_mmu_debug_wr_r());
}

u32 gm20b_fb_mmu_debug_rd(struct gk20a *g)
{
	return gk20a_readl(g, fb_mmu_debug_rd_r());
}

unsigned int gm20b_fb_compression_page_size(struct gk20a *g)
{
	return SZ_128K;
}

unsigned int gm20b_fb_compressible_page_size(struct gk20a *g)
{
	return SZ_64K;
}

u32 gm20b_fb_compression_align_mask(struct gk20a *g)
{
	return SZ_64K - 1;
}

void gm20b_fb_dump_vpr_info(struct gk20a *g)
{
	u32 val;

	/* print vpr info */
	val = gk20a_readl(g, fb_mmu_vpr_info_r());
	val &= ~0x3;
	val |= fb_mmu_vpr_info_index_addr_lo_v();
	gk20a_writel(g, fb_mmu_vpr_info_r(), val);
	nvgpu_err(g, "VPR: %08x %08x %08x %08x",
		gk20a_readl(g, fb_mmu_vpr_info_r()),
		gk20a_readl(g, fb_mmu_vpr_info_r()),
		gk20a_readl(g, fb_mmu_vpr_info_r()),
		gk20a_readl(g, fb_mmu_vpr_info_r()));
}

void gm20b_fb_dump_wpr_info(struct gk20a *g)
{
	u32 val;

	/* print wpr info */
	val = gk20a_readl(g, fb_mmu_wpr_info_r());
	val &= ~0xf;
	val |= (fb_mmu_wpr_info_index_allow_read_v());
	gk20a_writel(g, fb_mmu_wpr_info_r(), val);
	nvgpu_err(g, "WPR: %08x %08x %08x %08x %08x %08x",
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()),
		gk20a_readl(g, fb_mmu_wpr_info_r()));
}

static int gm20b_fb_vpr_info_fetch_wait(struct gk20a *g,
					    unsigned int msec)
{
	struct nvgpu_timeout timeout;

	nvgpu_timeout_init(g, &timeout, msec, NVGPU_TIMER_CPU_TIMER);

	do {
		u32 val;

		val = gk20a_readl(g, fb_mmu_vpr_info_r());
		if (fb_mmu_vpr_info_fetch_v(val) ==
		    fb_mmu_vpr_info_fetch_false_v()) {
			return 0;
		}

	} while (!nvgpu_timeout_expired(&timeout));

	return -ETIMEDOUT;
}

int gm20b_fb_vpr_info_fetch(struct gk20a *g)
{
	if (gm20b_fb_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT)) {
		return -ETIMEDOUT;
	}

	gk20a_writel(g, fb_mmu_vpr_info_r(),
			fb_mmu_vpr_info_fetch_true_v());

	return gm20b_fb_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT);
}

void gm20b_fb_read_wpr_info(struct gk20a *g, struct wpr_carveout_info *inf)
{
	u32 val = 0;
	u64 wpr_start = 0;
	u64 wpr_end = 0;

	val = gk20a_readl(g, fb_mmu_wpr_info_r());
	val &= ~0xF;
	val |= fb_mmu_wpr_info_index_wpr1_addr_lo_v();
	gk20a_writel(g, fb_mmu_wpr_info_r(), val);

	val = gk20a_readl(g, fb_mmu_wpr_info_r()) >> 0x4;
	wpr_start = hi32_lo32_to_u64(
			(val >> (32 - WPR_INFO_ADDR_ALIGNMENT)),
			(val << WPR_INFO_ADDR_ALIGNMENT));

	val = gk20a_readl(g, fb_mmu_wpr_info_r());
	val &= ~0xF;
	val |= fb_mmu_wpr_info_index_wpr1_addr_hi_v();
	gk20a_writel(g, fb_mmu_wpr_info_r(), val);

	val = gk20a_readl(g, fb_mmu_wpr_info_r()) >> 0x4;
	wpr_end = hi32_lo32_to_u64(
			(val >> (32 - WPR_INFO_ADDR_ALIGNMENT)),
			(val << WPR_INFO_ADDR_ALIGNMENT));

	inf->wpr_base = wpr_start;
	inf->nonwpr_base = 0;
	inf->size = (wpr_end - wpr_start);
}

bool gm20b_fb_debug_mode_enabled(struct gk20a *g)
{
	u32 debug_ctrl = gk20a_readl(g, fb_mmu_debug_ctrl_r());
	return fb_mmu_debug_ctrl_debug_v(debug_ctrl) ==
			fb_mmu_debug_ctrl_debug_enabled_v();
}

void gm20b_fb_set_mmu_debug_mode(struct gk20a *g, bool enable)
{
	u32 reg_val, fb_debug_ctrl;

	if (enable) {
		fb_debug_ctrl = fb_mmu_debug_ctrl_debug_enabled_f();
		g->mmu_debug_ctrl = true;
	} else {
		fb_debug_ctrl = fb_mmu_debug_ctrl_debug_disabled_f();
		g->mmu_debug_ctrl = false;
	}

	reg_val = nvgpu_readl(g, fb_mmu_debug_ctrl_r());
	reg_val = set_field(reg_val,
			fb_mmu_debug_ctrl_debug_m(), fb_debug_ctrl);
	nvgpu_writel(g, fb_mmu_debug_ctrl_r(), reg_val);
}

void gm20b_fb_set_debug_mode(struct gk20a *g, bool enable)
{
	gm20b_fb_set_mmu_debug_mode(g, enable);
	g->ops.gr.set_debug_mode(g, enable);
}

