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

#ifndef __NVGPU_OS_POSIX_H__
#define __NVGPU_OS_POSIX_H__

#include <nvgpu/gk20a.h>

struct nvgpu_posix_io_callbacks;

struct nvgpu_os_posix {
	struct gk20a g;


	/*
	 * IO callbacks for handling the nvgpu IO accessors.
	 */
	struct nvgpu_posix_io_callbacks *callbacks;

	/*
	 * Memory-mapped register space for unit tests.
	 */
	struct nvgpu_list_node reg_space_head;
	int error_code;


	/*
	 * List to record sequence of register writes.
	 */
	struct nvgpu_list_node recorder_head;
	bool recording;
};

static inline struct nvgpu_os_posix *nvgpu_os_posix_from_gk20a(struct gk20a *g)
{
	return container_of(g, struct nvgpu_os_posix, g);
}

#endif
