/*
 * GP10B CDE
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

#include <nvgpu/log.h>
#include <nvgpu/dma.h>
#include <nvgpu/gk20a.h>

#include "cde_gp10b.h"

enum gp10b_programs {
	GP10B_PROG_HPASS              = 0,
	GP10B_PROG_HPASS_4K           = 1,
	GP10B_PROG_VPASS              = 2,
	GP10B_PROG_VPASS_4K           = 3,
	GP10B_PROG_HPASS_DEBUG        = 4,
	GP10B_PROG_HPASS_4K_DEBUG     = 5,
	GP10B_PROG_VPASS_DEBUG        = 6,
	GP10B_PROG_VPASS_4K_DEBUG     = 7,
	GP10B_PROG_PASSTHROUGH        = 8,
};

void gp10b_cde_get_program_numbers(struct gk20a *g,
					  u32 block_height_log2,
					  u32 shader_parameter,
					  int *hprog_out, int *vprog_out)
{
	int hprog, vprog;

	if (shader_parameter == 1) {
		hprog = GP10B_PROG_PASSTHROUGH;
		vprog = GP10B_PROG_PASSTHROUGH;
	} else {
		hprog = GP10B_PROG_HPASS;
		vprog = GP10B_PROG_VPASS;
		if (shader_parameter == 2) {
			hprog = GP10B_PROG_HPASS_DEBUG;
			vprog = GP10B_PROG_VPASS_DEBUG;
		}
		if (!nvgpu_iommuable(g)) {
			if (!g->mm.disable_bigpage) {
				nvgpu_warn(g,
					   "When no IOMMU big pages cannot be used");
			}
			hprog |= 1;
			vprog |= 1;
		}
	}

	*hprog_out = hprog;
	*vprog_out = vprog;
}

bool gp10b_need_scatter_buffer(struct gk20a *g)
{
	return !nvgpu_iommuable(g);
}

static u8 parity(u32 a)
{
	a ^= a>>16u;
	a ^= a>>8u;
	a ^= a>>4u;
	a &= 0xfu;
	return (0x6996u >> a) & 1u;
}

int gp10b_populate_scatter_buffer(struct gk20a *g,
					 struct sg_table *sgt,
					 size_t surface_size,
					 void *scatter_buffer_ptr,
					 size_t scatter_buffer_size)
{
	/* map scatter buffer to CPU VA and fill it */
	const u32 page_size_log2 = 12;
	const u32 page_size = 1 << page_size_log2;
	const u32 page_size_shift = page_size_log2 - 7u;

	/* 0011 1111 1111 1111 1111 1110 0100 1000 */
	const u32 getSliceMaskGP10B = 0x3ffffe48;
	u8 *scatter_buffer = scatter_buffer_ptr;

	size_t i;
	struct scatterlist *sg = NULL;
	u8 d = 0;
	size_t page = 0;
	size_t pages_left;

	surface_size = round_up(surface_size, page_size);

	pages_left = surface_size >> page_size_log2;
	if ((pages_left >> 3) > scatter_buffer_size)
	    return -ENOMEM;

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		unsigned int j;
		u64 surf_pa = sg_phys(sg);
		unsigned int n = (int)(sg->length >> page_size_log2);

		nvgpu_log(g, gpu_dbg_cde, "surfPA=0x%llx + %d pages", surf_pa, n);

		for (j=0; j < n && pages_left > 0; j++, surf_pa += page_size) {
			u32 addr = (((u32)(surf_pa>>7)) & getSliceMaskGP10B) >> page_size_shift;
			u8 scatter_bit = parity(addr);
			u8 bit = page & 7;

			d |= scatter_bit << bit;
			if (bit == 7) {
				scatter_buffer[page >> 3] = d;
				d = 0;
			}

			++page;
			--pages_left;
		}

		if (pages_left == 0)
			break;
	}

	/* write the last byte in case the number of pages is not divisible by 8 */
	if ((page & 7) != 0)
		scatter_buffer[page >> 3] = d;

	if (nvgpu_log_mask_enabled(g, gpu_dbg_cde)) {
		nvgpu_log(g, gpu_dbg_cde, "scatterBuffer content:");
		for (i = 0; i < page >> 3; i++) {
			nvgpu_log(g, gpu_dbg_cde, " %x", scatter_buffer[i]);
		}
	}

	return 0;
}
