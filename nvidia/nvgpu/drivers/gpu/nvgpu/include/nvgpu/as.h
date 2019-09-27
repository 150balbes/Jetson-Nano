/*
 * GK20A Address Spaces
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_AS_H
#define NVGPU_AS_H

#include <nvgpu/types.h>

struct vm_gk20a;
struct gk20a;

struct gk20a_as {
	int last_share_id; /* dummy allocator for now */
};

struct gk20a_as_share {
	struct gk20a_as *as;
	struct vm_gk20a *vm;
	int id;
};

/*
 * AS allocation flags.
 */
#define NVGPU_AS_ALLOC_USERSPACE_MANAGED	(1 << 0)

int gk20a_as_release_share(struct gk20a_as_share *as_share);

/* if big_page_size == 0, the default big page size is used */
int gk20a_as_alloc_share(struct gk20a *g, u32 big_page_size,
			 u32 flags, struct gk20a_as_share **out);

struct gk20a *gk20a_from_as(struct gk20a_as *as);
#endif /* NVGPU_AS_H */
