/*
 * GV11B Cycle stats snapshots support
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bitops.h>
#include <nvgpu/kmem.h>
#include <nvgpu/lock.h>
#include <nvgpu/dma.h>
#include <nvgpu/mm.h>
#include <nvgpu/sizes.h>
#include <nvgpu/enabled.h>
#include <nvgpu/log.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/bug.h>
#include <nvgpu/dma.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/unit.h>

#include "gk20a/css_gr_gk20a.h"
#include "css_gr_gv11b.h"

#include <nvgpu/hw/gv11b/hw_perf_gv11b.h>


/* reports whether the hw queue overflowed */
bool gv11b_css_hw_get_overflow_status(struct gk20a *g)
{
	const u32 st = perf_pmasys_control_membuf_status_overflowed_f();
	return st == (gk20a_readl(g, perf_pmasys_control_r()) & st);
}

/* returns how many pending snapshot entries are pending */
u32 gv11b_css_hw_get_pending_snapshots(struct gk20a *g)
{
	return gk20a_readl(g, perf_pmasys_mem_bytes_r()) /
			sizeof(struct gk20a_cs_snapshot_fifo_entry);
}

/* disable streaming to memory */
static void gv11b_css_hw_reset_streaming(struct gk20a *g)
{
	u32 engine_status;

	/* reset the perfmon */
	g->ops.mc.reset(g, g->ops.mc.reset_mask(g, NVGPU_UNIT_PERFMON));

	/* RBUFEMPTY must be set -- otherwise we'll pick up */
	/* snapshot that have been queued up from earlier   */
	engine_status = gk20a_readl(g, perf_pmasys_enginestatus_r());

	/* turn off writes */
	gk20a_writel(g, perf_pmasys_control_r(),
			perf_pmasys_control_membuf_clear_status_doit_f());

	/* pointing all pending snapshots as handled */
	gv11b_css_hw_set_handled_snapshots(g, gv11b_css_hw_get_pending_snapshots(g));
}

/* informs hw how many snapshots have been processed (frees up fifo space) */
void gv11b_css_hw_set_handled_snapshots(struct gk20a *g, u32 done)
{
	if (done > 0) {
		gk20a_writel(g, perf_pmasys_mem_bump_r(),
		     done * sizeof(struct gk20a_cs_snapshot_fifo_entry));
	}
}

int gv11b_css_hw_enable_snapshot(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *cs_client)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;
	struct gk20a_cs_snapshot *data = gr->cs_data;
	u32 snapshot_size = cs_client->snapshot_size;
	int ret;

	u32 virt_addr_lo;
	u32 virt_addr_hi;
	u32 inst_pa_page;

	if (data->hw_snapshot)
		return 0;

	if (snapshot_size < CSS_MIN_HW_SNAPSHOT_SIZE)
		snapshot_size = CSS_MIN_HW_SNAPSHOT_SIZE;

	ret = nvgpu_dma_alloc_map_sys(g->mm.pmu.vm, snapshot_size,
							&data->hw_memdesc);
	if (ret)
		return ret;

	/* perf output buffer may not cross a 4GB boundary - with a separate */
	/* va smaller than that, it won't but check anyway */
	if (!data->hw_memdesc.cpu_va ||
		data->hw_memdesc.size < snapshot_size ||
		data->hw_memdesc.gpu_va + u64_lo32(snapshot_size) > SZ_4G) {
		ret = -EFAULT;
		goto failed_allocation;
	}

	data->hw_snapshot =
		(struct gk20a_cs_snapshot_fifo_entry *)data->hw_memdesc.cpu_va;
	data->hw_end = data->hw_snapshot +
		snapshot_size / sizeof(struct gk20a_cs_snapshot_fifo_entry);
	data->hw_get = data->hw_snapshot;
	memset(data->hw_snapshot, 0xff, snapshot_size);

	virt_addr_lo = u64_lo32(data->hw_memdesc.gpu_va);
	virt_addr_hi = u64_hi32(data->hw_memdesc.gpu_va);

	gv11b_css_hw_reset_streaming(g);

	gk20a_writel(g, perf_pmasys_outbase_r(), virt_addr_lo);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(virt_addr_hi));
	gk20a_writel(g, perf_pmasys_outsize_r(), snapshot_size);

	/* this field is aligned to 4K */
	inst_pa_page = nvgpu_inst_block_addr(g, &g->mm.hwpm.inst_block) >> 12;

	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(inst_pa_page) |
			perf_pmasys_mem_block_valid_true_f() |
			nvgpu_aperture_mask(g, &g->mm.hwpm.inst_block,
				perf_pmasys_mem_block_target_sys_ncoh_f(),
				perf_pmasys_mem_block_target_sys_coh_f(),
				perf_pmasys_mem_block_target_lfb_f()));


	nvgpu_log_info(g, "cyclestats: buffer for hardware snapshots enabled\n");

	return 0;

failed_allocation:
	if (data->hw_memdesc.size) {
		nvgpu_dma_unmap_free(g->mm.pmu.vm, &data->hw_memdesc);
		memset(&data->hw_memdesc, 0, sizeof(data->hw_memdesc));
	}
	data->hw_snapshot = NULL;

	return ret;
}

void gv11b_css_hw_disable_snapshot(struct gr_gk20a *gr)
{
	struct gk20a *g = gr->g;
	struct gk20a_cs_snapshot *data = gr->cs_data;

	if (!data->hw_snapshot)
		return;

	gv11b_css_hw_reset_streaming(g);

	gk20a_writel(g, perf_pmasys_outbase_r(), 0);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(0));
	gk20a_writel(g, perf_pmasys_outsize_r(), 0);

	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(0) |
			perf_pmasys_mem_block_valid_false_f() |
			perf_pmasys_mem_block_target_f(0));

	nvgpu_dma_unmap_free(g->mm.pmu.vm, &data->hw_memdesc);
	memset(&data->hw_memdesc, 0, sizeof(data->hw_memdesc));
	data->hw_snapshot = NULL;

	nvgpu_log_info(g, "cyclestats: buffer for hardware snapshots disabled\n");
}

int gv11b_css_hw_check_data_available(struct channel_gk20a *ch, u32 *pending,
					bool *hw_overflow)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;
	struct gk20a_cs_snapshot *css = gr->cs_data;

	if (!css->hw_snapshot)
		return -EINVAL;

	*pending = gv11b_css_hw_get_pending_snapshots(g);
	if (!*pending)
		return 0;

	*hw_overflow = gv11b_css_hw_get_overflow_status(g);
	return 0;
}
