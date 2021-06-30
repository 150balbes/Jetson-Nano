/*
 * GM20B CDE
 *
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

#include <nvgpu/gk20a.h>

#include "cde_gm20b.h"

enum programs {
	PROG_HPASS              = 0,
	PROG_VPASS_LARGE        = 1,
	PROG_VPASS_SMALL        = 2,
	PROG_HPASS_DEBUG        = 3,
	PROG_VPASS_LARGE_DEBUG  = 4,
	PROG_VPASS_SMALL_DEBUG  = 5,
	PROG_PASSTHROUGH        = 6,
};

void gm20b_cde_get_program_numbers(struct gk20a *g,
				   u32 block_height_log2,
				   u32 shader_parameter,
				   int *hprog_out, int *vprog_out)
{
	int hprog = PROG_HPASS;
	int vprog = (block_height_log2 >= 2) ?
		PROG_VPASS_LARGE : PROG_VPASS_SMALL;
	if (shader_parameter == 1) {
		hprog = PROG_PASSTHROUGH;
		vprog = PROG_PASSTHROUGH;
	} else if (shader_parameter == 2) {
		hprog = PROG_HPASS_DEBUG;
		vprog = (block_height_log2 >= 2) ?
			PROG_VPASS_LARGE_DEBUG :
			PROG_VPASS_SMALL_DEBUG;
	}

	*hprog_out = hprog;
	*vprog_out = vprog;
}
