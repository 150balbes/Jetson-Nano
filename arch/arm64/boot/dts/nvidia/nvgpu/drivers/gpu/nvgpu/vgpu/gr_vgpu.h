/*
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

#ifndef _GR_VGPU_H_
#define _GR_VGPU_H_

#include <nvgpu/types.h>

struct gk20a;
struct channel_gk20a;
struct gr_gk20a;
struct gr_zcull_info;
struct zbc_entry;
struct zbc_query_params;
struct dbg_session_gk20a;
struct tsg_gk20a;

void vgpu_gr_detect_sm_arch(struct gk20a *g);
void vgpu_gr_free_channel_ctx(struct channel_gk20a *c, bool is_tsg);
void vgpu_gr_free_tsg_ctx(struct tsg_gk20a *tsg);
int vgpu_gr_alloc_obj_ctx(struct channel_gk20a  *c, u32 class_num, u32 flags);
int vgpu_gr_bind_ctxsw_zcull(struct gk20a *g, struct gr_gk20a *gr,
				struct channel_gk20a *c, u64 zcull_va,
				u32 mode);
int vgpu_gr_get_zcull_info(struct gk20a *g, struct gr_gk20a *gr,
				struct gr_zcull_info *zcull_params);
u32 vgpu_gr_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index);
u32 vgpu_gr_get_max_fbps_count(struct gk20a *g);
u32 vgpu_gr_get_fbp_en_mask(struct gk20a *g);
u32 vgpu_gr_get_max_ltc_per_fbp(struct gk20a *g);
u32 vgpu_gr_get_max_lts_per_ltc(struct gk20a *g);
u32 *vgpu_gr_rop_l2_en_mask(struct gk20a *g);
int vgpu_gr_add_zbc(struct gk20a *g, struct gr_gk20a *gr,
			   struct zbc_entry *zbc_val);
int vgpu_gr_query_zbc(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params);
int vgpu_gr_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable);
int vgpu_gr_update_smpc_ctxsw_mode(struct gk20a *g,
	struct channel_gk20a *ch, bool enable);
int vgpu_gr_update_hwpm_ctxsw_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 gpu_va, u32 mode);
int vgpu_gr_clear_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id);
int vgpu_gr_suspend_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd);
int vgpu_gr_resume_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd);
int vgpu_gr_commit_inst(struct channel_gk20a *c, u64 gpu_va);
int vgpu_gr_init_sm_id_table(struct gk20a *g);
int vgpu_gr_init_fs_state(struct gk20a *g);
int vgpu_gr_update_pc_sampling(struct channel_gk20a *ch, bool enable);
#endif
