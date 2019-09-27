/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_NVLINK_GV100_H
#define NVGPU_NVLINK_GV100_H

#define GV100_CONNECTED_LINK_MASK		0x8

struct gk20a;

int gv100_nvlink_discover_ioctrl(struct gk20a *g);
int gv100_nvlink_discover_link(struct gk20a *g);
int gv100_nvlink_init(struct gk20a *g);
int gv100_nvlink_isr(struct gk20a *g);
int gv100_nvlink_minion_send_command(struct gk20a *g, u32 link_id, u32 command,
						u32 scratch_0, bool sync);
int gv100_nvlink_setup_pll(struct gk20a *g, unsigned long link_mask);
int gv100_nvlink_minion_data_ready_en(struct gk20a *g,
					unsigned long link_mask, bool sync);
void gv100_nvlink_get_connected_link_mask(u32 *link_mask);
void gv100_nvlink_set_sw_war(struct gk20a *g, u32 link_id);
/* API */
int gv100_nvlink_link_early_init(struct gk20a *g, unsigned long mask);
u32 gv100_nvlink_link_get_mode(struct gk20a *g, u32 link_id);
u32 gv100_nvlink_link_get_state(struct gk20a *g, u32 link_id);
int gv100_nvlink_link_set_mode(struct gk20a *g, u32 link_id, u32 mode);
u32 gv100_nvlink_link_get_sublink_mode(struct gk20a *g, u32 link_id,
	bool is_rx_sublink);
u32 gv100_nvlink_link_get_tx_sublink_state(struct gk20a *g, u32 link_id);
u32 gv100_nvlink_link_get_rx_sublink_state(struct gk20a *g, u32 link_id);
int gv100_nvlink_link_set_sublink_mode(struct gk20a *g, u32 link_id,
	bool is_rx_sublink, u32 mode);
int gv100_nvlink_interface_init(struct gk20a *g);
int gv100_nvlink_interface_disable(struct gk20a *g);
int gv100_nvlink_reg_init(struct gk20a *g);
int gv100_nvlink_shutdown(struct gk20a *g);
int gv100_nvlink_early_init(struct gk20a *g);
#endif
