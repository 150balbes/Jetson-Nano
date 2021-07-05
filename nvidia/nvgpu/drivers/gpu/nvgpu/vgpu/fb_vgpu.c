/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/gk20a.h>
#include <nvgpu/vgpu/tegra_vgpu.h>
#include <nvgpu/vgpu/vgpu.h>

#include "fb_vgpu.h"

void vgpu_fb_set_mmu_debug_mode(struct gk20a *g, bool enable)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_fb_set_mmu_debug_mode_params *p =
				&msg.params.fb_set_mmu_debug_mode;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_FB_SET_MMU_DEBUG_MODE;
	msg.handle = vgpu_get_handle(g);
	p->enable = enable ? 1U : 0U;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err != 0 ? err : msg.ret;
	if (err != 0) {
		nvgpu_err(g,
			"fb set mmu debug mode failed err %d", err);
	}
}
