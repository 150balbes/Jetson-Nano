/*
 * GP106 Graphics Context
 *
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

#include <nvgpu/gk20a.h>

#include "gr_ctx_gp106.h"

int gr_gp106_get_netlist_name(struct gk20a *g, int index, char *name)
{
	u32 ver = g->params.gpu_arch + g->params.gpu_impl;

	switch (ver) {
		case NVGPU_GPUID_GP104:
			sprintf(name, "%s/%s", "gp104",
					GP104_NETLIST_IMAGE_FW_NAME);
			break;
		case NVGPU_GPUID_GP106:
			sprintf(name, "%s/%s", "gp106",
					GP106_NETLIST_IMAGE_FW_NAME);
			break;
		default:
			nvgpu_err(g, "no support for GPUID %x", ver);
	}

	return 0;
}

bool gr_gp106_is_firmware_defined(void)
{
	return true;
}
