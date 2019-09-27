/*
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
#include <nvgpu/bug.h>

#include "gp10b/fifo_gp10b.h"

#include "fifo_gp106.h"

#include <nvgpu/hw/gp106/hw_ccsr_gp106.h>
#include <nvgpu/hw/gp106/hw_fifo_gp106.h>

u32 gp106_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

static const char * const gp106_hub_client_descs[] = {
	"vip", "ce0", "ce1", "dniso", "fe", "fecs", "host", "host cpu",
	"host cpu nb", "iso", "mmu", "mspdec", "msppp", "msvld",
	"niso", "p2p", "pd", "perf", "pmu", "raster twod", "scc",
	"scc nb", "sec", "ssync", "gr copy", "xv", "mmu nb",
	"msenc", "d falcon", "sked", "a falcon", "n/a",
	"hsce0", "hsce1", "hsce2", "hsce3", "hsce4", "hsce5",
	"hsce6", "hsce7", "hsce8", "hsce9", "hshub",
	"ptp x0", "ptp x1", "ptp x2", "ptp x3", "ptp x4",
	"ptp x5", "ptp x6", "ptp x7", "vpr scrubber0", "vpr scrubber1",
	"dwbif", "fbfalcon",
};

static const char * const gp106_gpc_client_descs[] = {
	"l1 0", "t1 0", "pe 0",
	"l1 1", "t1 1", "pe 1",
	"l1 2", "t1 2", "pe 2",
	"l1 3", "t1 3", "pe 3",
	"rast", "gcc", "gpccs",
	"prop 0", "prop 1", "prop 2", "prop 3",
	"l1 4", "t1 4", "pe 4",
	"l1 5", "t1 5", "pe 5",
	"l1 6", "t1 6", "pe 6",
	"l1 7", "t1 7", "pe 7",
	"l1 9", "t1 9", "pe 9",
	"l1 10", "t1 10", "pe 10",
	"l1 11", "t1 11", "pe 11",
	"unknown", "unknown", "unknown", "unknown",
	"tpccs 0", "tpccs 1", "tpccs 2",
	"tpccs 3", "tpccs 4", "tpccs 5",
	"tpccs 6", "tpccs 7", "tpccs 8",
	"tpccs 9", "tpccs 10", "tpccs 11",
	"tpccs 12", "tpccs 13", "tpccs 14",
	"tpccs 15", "tpccs 16", "tpccs 17",
	"tpccs 18", "tpccs 19", "unknown", "unknown",
	"unknown", "unknown", "unknown", "unknown",
	"unknown", "unknown", "unknown", "unknown",
	"unknown", "unknown",
	"l1 12", "t1 12", "pe 12",
	"l1 13", "t1 13", "pe 13",
	"l1 14", "t1 14", "pe 14",
	"l1 15", "t1 15", "pe 15",
	"l1 16", "t1 16", "pe 16",
	"l1 17", "t1 17", "pe 17",
	"l1 18", "t1 18", "pe 18",
	"l1 19", "t1 19", "pe 19",
};

void gp106_fifo_get_mmu_fault_gpc_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->client_id >= ARRAY_SIZE(gp106_gpc_client_descs)) {
		WARN_ON(mmfault->client_id >=
				ARRAY_SIZE(gp106_gpc_client_descs));
	} else {
		mmfault->client_id_desc =
			 gp106_gpc_client_descs[mmfault->client_id];
	}
}

/* fill in mmu fault client description */
void gp106_fifo_get_mmu_fault_client_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->client_id >= ARRAY_SIZE(gp106_hub_client_descs)) {
		WARN_ON(mmfault->client_id >=
				ARRAY_SIZE(gp106_hub_client_descs));
	} else {
		mmfault->client_id_desc =
			 gp106_hub_client_descs[mmfault->client_id];
	}
}
