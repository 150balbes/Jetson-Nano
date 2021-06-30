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

#include <nvgpu/io.h>
#include <nvgpu/timers.h>
#include <nvgpu/gk20a.h>

#include "fb_gp10b.h"
#include "fb_gp106.h"

#include <nvgpu/hw/gp106/hw_fb_gp106.h>

#define HW_SCRUB_TIMEOUT_DEFAULT	100 /* usec */
#define HW_SCRUB_TIMEOUT_MAX		2000000 /* usec */

void gp106_fb_init_fs_state(struct gk20a *g)
{
	u32 val;

	int retries = HW_SCRUB_TIMEOUT_MAX / HW_SCRUB_TIMEOUT_DEFAULT;
	/* wait for memory to be accessible */
	do {
		u32 w = gk20a_readl(g, fb_niso_scrub_status_r());
		if (fb_niso_scrub_status_flag_v(w)) {
			nvgpu_log_fn(g, "done");
			break;
		}
		nvgpu_udelay(HW_SCRUB_TIMEOUT_DEFAULT);
	} while (--retries);

	val = gk20a_readl(g, fb_mmu_priv_level_mask_r());
	val &= ~fb_mmu_priv_level_mask_write_violation_m();
	gk20a_writel(g, fb_mmu_priv_level_mask_r(), val);
}

size_t gp106_fb_get_vidmem_size(struct gk20a *g)
{
	u32 range = gk20a_readl(g, fb_mmu_local_memory_range_r());
	u32 mag = fb_mmu_local_memory_range_lower_mag_v(range);
	u32 scale = fb_mmu_local_memory_range_lower_scale_v(range);
	u32 ecc = fb_mmu_local_memory_range_ecc_mode_v(range);
	size_t bytes = ((size_t)mag << scale) * SZ_1M;

	if (ecc) {
		bytes = bytes / 16U * 15U;
	}

	return bytes;
}
