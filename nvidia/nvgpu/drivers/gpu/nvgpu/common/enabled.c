/*
 * Copyright (c) 2017-18, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/bitops.h>
#include <nvgpu/gk20a.h>

int nvgpu_init_enabled_flags(struct gk20a *g)
{
	/*
	 * Zero all flags initially. Flags that should be set to non-zero states
	 * can be done so during driver init.
	 */
	g->enabled_flags = nvgpu_kzalloc(g,
					 BITS_TO_LONGS(NVGPU_MAX_ENABLED_BITS) *
					 sizeof(unsigned long));
	if (!g->enabled_flags) {
		return -ENOMEM;
	}

	return 0;
}

/*
 * Call this on driver shutdown!
 */
void nvgpu_free_enabled_flags(struct gk20a *g)
{
	nvgpu_kfree(g, g->enabled_flags);
}

bool nvgpu_is_enabled(struct gk20a *g, int flag)
{
	return test_bit(flag, g->enabled_flags);
}

void __nvgpu_set_enabled(struct gk20a *g, int flag, bool state)
{
	if (state) {
		set_bit(flag, g->enabled_flags);
	} else {
		clear_bit(flag, g->enabled_flags);
	}
}
