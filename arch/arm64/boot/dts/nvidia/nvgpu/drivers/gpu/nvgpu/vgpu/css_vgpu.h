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

#ifndef _CSS_VGPU_H_
#define _CSS_VGPU_H_

#include <nvgpu/types.h>

struct gr_gk20a;
struct channel_gk20a;
struct gk20a_cs_snapshot_client;

void vgpu_css_release_snapshot_buffer(struct gr_gk20a *gr);
int vgpu_css_flush_snapshots(struct channel_gk20a *ch,
			u32 *pending, bool *hw_overflow);
int vgpu_css_detach(struct channel_gk20a *ch,
		struct gk20a_cs_snapshot_client *cs_client);
int vgpu_css_enable_snapshot_buffer(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *cs_client);
u32 vgpu_css_get_buffer_size(struct gk20a *g);
#endif
