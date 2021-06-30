/*
 * GV11B TPC
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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
#include "tpc_gv11b.h"

int gv11b_tpc_powergate(struct gk20a *g, u32 fuse_status)
{
	int err = 0;

	if (fuse_status == 0x0) {
		g->can_tpc_powergate = true;

	} else {
		/* if hardware has already floorswept any TPC
		 * (fuse_status !=  0x0) and if TPC PG mask
		 * sent from userspace is 0x0 GPU will be powered on
		 * with the default fuse_status setting. It cannot
		 * un-floorsweep any TPC
		 * thus, set g->tpc_pg_mask to fuse_status value
		 */
		if (g->tpc_pg_mask == 0x0) {
			g->can_tpc_powergate = true;
			g->tpc_pg_mask = fuse_status;

		} else if (fuse_status == g->tpc_pg_mask) {
			g->can_tpc_powergate = true;

		} else if ((fuse_status & g->tpc_pg_mask) ==
					fuse_status) {
			g->can_tpc_powergate = true;

		} else {
			/* If userspace sends a TPC PG mask such that
			 * it tries to un-floorsweep any TPC which is
			 * already powergated from hardware, then
			 * such mask is invalid.
			 * In this case set tpc pg mask to 0x0
			 * Return -EINVAL here and halt GPU poweron.
			 */
			nvgpu_err(g, "Invalid TPC_PG mask: 0x%x",
							g->tpc_pg_mask);
			g->can_tpc_powergate = false;
			g->tpc_pg_mask = 0x0;
			err = -EINVAL;
		}
	}

	return err;
}
