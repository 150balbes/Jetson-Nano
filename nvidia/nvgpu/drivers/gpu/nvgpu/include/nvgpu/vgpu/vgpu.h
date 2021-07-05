/*
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

#ifndef __VGPU_COMMON_H__
#define __VGPU_COMMON_H__

#include <nvgpu/types.h>
#include <nvgpu/thread.h>
#include <nvgpu/log.h>
#include <nvgpu/lock.h>
#include <nvgpu/vgpu/tegra_vgpu.h>

struct device;
struct tegra_vgpu_gr_intr_info;
struct tegra_vgpu_fifo_intr_info;
struct tegra_vgpu_cmd_msg;
struct nvgpu_mem;
struct gk20a;
struct vm_gk20a;
struct nvgpu_gr_ctx;
struct nvgpu_cpu_time_correlation_sample;
struct vgpu_ecc_stat;
struct channel_gk20a;

struct vgpu_priv_data {
	u64 virt_handle;
	struct nvgpu_thread intr_handler;
	struct tegra_vgpu_constants_params constants;
	struct vgpu_ecc_stat *ecc_stats;
	int ecc_stats_count;
	u32 num_freqs;
	unsigned long *freqs;
	struct nvgpu_mutex vgpu_clk_get_freq_lock;
};

struct vgpu_priv_data *vgpu_get_priv_data(struct gk20a *g);

static inline u64 vgpu_get_handle(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	if (unlikely(!priv)) {
		nvgpu_err(g, "invalid vgpu_priv_data in %s", __func__);
		return INT_MAX;
	}

	return priv->virt_handle;
}

int vgpu_comm_init(struct gk20a *g);
void vgpu_comm_deinit(void);
int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out);
u64 vgpu_connect(void);
int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value);
int vgpu_intr_thread(void *dev_id);
void vgpu_remove_support_common(struct gk20a *g);
void vgpu_detect_chip(struct gk20a *g);
int vgpu_init_gpu_characteristics(struct gk20a *g);
int vgpu_read_ptimer(struct gk20a *g, u64 *value);
int vgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples);
int vgpu_init_hal(struct gk20a *g);
int vgpu_get_constants(struct gk20a *g);
u64 vgpu_bar1_map(struct gk20a *g, struct nvgpu_mem *mem);
int vgpu_gr_isr(struct gk20a *g, struct tegra_vgpu_gr_intr_info *info);
int vgpu_gr_alloc_gr_ctx(struct gk20a *g,
			struct nvgpu_gr_ctx *gr_ctx,
			struct vm_gk20a *vm,
			u32 class,
			u32 flags);
void vgpu_gr_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
			struct nvgpu_gr_ctx *gr_ctx);
void vgpu_gr_handle_sm_esr_event(struct gk20a *g,
			struct tegra_vgpu_sm_esr_info *info);
int vgpu_gr_init_ctx_state(struct gk20a *g);
int vgpu_fifo_isr(struct gk20a *g, struct tegra_vgpu_fifo_intr_info *info);
u32 vgpu_ce_get_num_pce(struct gk20a *g);
int vgpu_init_mm_support(struct gk20a *g);
int vgpu_init_gr_support(struct gk20a *g);
int vgpu_init_fifo_support(struct gk20a *g);

int vgpu_gp10b_init_hal(struct gk20a *g);
int vgpu_gv11b_init_hal(struct gk20a *g);

bool vgpu_is_reduced_bar1(struct gk20a *g);

int vgpu_gr_set_mmu_debug_mode(struct gk20a *g,
			struct channel_gk20a *ch, bool enable);
#endif
