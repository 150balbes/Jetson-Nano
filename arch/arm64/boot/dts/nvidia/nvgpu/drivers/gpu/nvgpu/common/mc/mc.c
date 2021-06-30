/*
 * GK20A Master Control
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/mc.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/hw/gm20b/hw_mc_gm20b.h>

u32 nvgpu_mc_boot_0(struct gk20a *g, u32 *arch, u32 *impl, u32 *rev)
{
	u32 val = __nvgpu_readl(g, mc_boot_0_r());

	if (val != 0xffffffffU) {

		if (arch != NULL) {
			*arch = mc_boot_0_architecture_v(val) <<
				NVGPU_GPU_ARCHITECTURE_SHIFT;
		}

		if (impl != NULL) {
			*impl = mc_boot_0_implementation_v(val);
		}

		if (rev != NULL) {
			*rev = (mc_boot_0_major_revision_v(val) << 4) |
				mc_boot_0_minor_revision_v(val);
		}
	}

	return val;
}
