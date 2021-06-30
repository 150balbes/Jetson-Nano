/*
 * drivers/video/tegra/host/gp10b/gr_ctx_gp10b.c
 *
 * GM20B Graphics Context
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "gr_ctx_gp10b.h"

int gr_gp10b_get_netlist_name(struct gk20a *g, int index, char *name)
{
	switch (index) {
#ifdef GP10B_NETLIST_IMAGE_FW_NAME
	case NETLIST_FINAL:
		sprintf(name, GP10B_NETLIST_IMAGE_FW_NAME);
		return 0;
#endif
#ifdef GK20A_NETLIST_IMAGE_A
	case NETLIST_SLOT_A:
		sprintf(name, GK20A_NETLIST_IMAGE_A);
		return 0;
#endif
#ifdef GK20A_NETLIST_IMAGE_B
	case NETLIST_SLOT_B:
		sprintf(name, GK20A_NETLIST_IMAGE_B);
		return 0;
#endif
#ifdef GK20A_NETLIST_IMAGE_C
	case NETLIST_SLOT_C:
		sprintf(name, GK20A_NETLIST_IMAGE_C);
		return 0;
#endif
#ifdef GK20A_NETLIST_IMAGE_D
	case NETLIST_SLOT_D:
		sprintf(name, GK20A_NETLIST_IMAGE_D);
		return 0;
#endif
	default:
		return -1;
	}

	return -1;
}

bool gr_gp10b_is_firmware_defined(void)
{
#ifdef GP10B_NETLIST_IMAGE_FW_NAME
	return true;
#else
	return false;
#endif
}
