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

#ifndef NVGPU_XVE_GP106_H
#define NVGPU_XVE_GP106_H

#include <nvgpu/log2.h>
#include <nvgpu/types.h>
#include <nvgpu/gk20a.h>

int gp106_init_xve_ops(struct gpu_ops *gops);

/*
 * Best guess for a reasonable timeout.
 */
#define GPU_XVE_TIMEOUT_MS	500

/*
 * Debugging for the speed change.
 */
enum xv_speed_change_steps {
	PRE_CHANGE = 0,
	DISABLE_ASPM,
	DL_SAFE_MODE,
	CHECK_LINK,
	LINK_SETTINGS,
	EXEC_CHANGE,
	EXEC_VERIF,
	CLEANUP
};

#define xv_dbg(g, fmt, args...)			\
	nvgpu_log(g, gpu_dbg_xv, fmt, ##args)

#define xv_sc_dbg(g, step, fmt, args...)					\
	xv_dbg(g, "[%d] %15s | " fmt, step, __stringify(step), ##args)

void xve_xve_writel_gp106(struct gk20a *g, u32 reg, u32 val);
u32 xve_xve_readl_gp106(struct gk20a *g, u32 reg);
void xve_reset_gpu_gp106(struct gk20a *g);
int xve_get_speed_gp106(struct gk20a *g, u32 *xve_link_speed);
void xve_disable_aspm_gp106(struct gk20a *g);
int xve_set_speed_gp106(struct gk20a *g, u32 next_link_speed);
void xve_available_speeds_gp106(struct gk20a *g, u32 *speed_mask);
u32 xve_get_link_control_status(struct gk20a *g);
#if defined(CONFIG_PCI_MSI)
void xve_rearm_msi_gp106(struct gk20a *g);
#endif
void xve_enable_shadow_rom_gp106(struct gk20a *g);
void xve_disable_shadow_rom_gp106(struct gk20a *g);

#endif /* NVGPU_XVE_GP106_H */
