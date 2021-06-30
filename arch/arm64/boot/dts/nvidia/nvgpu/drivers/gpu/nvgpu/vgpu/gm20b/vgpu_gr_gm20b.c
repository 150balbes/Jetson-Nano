/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "vgpu/css_vgpu.h"
#include "vgpu_gr_gm20b.h"

void vgpu_gr_gm20b_init_cyclestats(struct gk20a *g)
{
#if defined(CONFIG_GK20A_CYCLE_STATS)
	bool snapshots_supported = true;

	/* cyclestats not supported on vgpu */
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_CYCLE_STATS, false);

	g->gr.max_css_buffer_size = vgpu_css_get_buffer_size(g);

	/* snapshots not supported if the buffer size is 0 */
	if (g->gr.max_css_buffer_size == 0)
		snapshots_supported = false;

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_CYCLE_STATS_SNAPSHOT,
							snapshots_supported);
#endif
}

