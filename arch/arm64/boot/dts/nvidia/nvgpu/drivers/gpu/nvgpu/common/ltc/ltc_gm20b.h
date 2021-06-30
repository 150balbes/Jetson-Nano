/*
 * GM20B L2
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_LTC_GM20B
#define NVGPU_LTC_GM20B

#include <nvgpu/types.h>

struct gk20a;
struct gr_gk20a;
struct gpu_ops;
struct zbc_entry;
enum gk20a_cbc_op;

int gm20b_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr);
int gm20b_determine_L2_size_bytes(struct gk20a *g);
void gm20b_ltc_set_zbc_color_entry(struct gk20a *g,
					  struct zbc_entry *color_val,
					  u32 index);
void gm20b_ltc_set_zbc_depth_entry(struct gk20a *g,
					  struct zbc_entry *depth_val,
					  u32 index);
void gm20b_ltc_init_cbc(struct gk20a *g, struct gr_gk20a *gr);
void gm20b_ltc_set_enabled(struct gk20a *g, bool enabled);
void gm20b_ltc_init_fs_state(struct gk20a *g);
int gm20b_ltc_cbc_ctrl(struct gk20a *g, enum gk20a_cbc_op op,
		       u32 min, u32 max);
void gm20b_ltc_isr(struct gk20a *g);
u32 gm20b_ltc_cbc_fix_config(struct gk20a *g, int base);
void gm20b_flush_ltc(struct gk20a *g);
int gm20b_ltc_alloc_phys_cbc(struct gk20a *g,
			     size_t compbit_backing_size);
int gm20b_ltc_alloc_virt_cbc(struct gk20a *g,
			     size_t compbit_backing_size);
bool gm20b_ltc_pri_is_ltc_addr(struct gk20a *g, u32 addr);
bool gm20b_ltc_is_ltcs_ltss_addr(struct gk20a *g, u32 addr);
bool gm20b_ltc_is_ltcn_ltss_addr(struct gk20a *g, u32 addr);
void gm20b_ltc_split_lts_broadcast_addr(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index);
void gm20b_ltc_split_ltc_broadcast_addr(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index);

#endif
