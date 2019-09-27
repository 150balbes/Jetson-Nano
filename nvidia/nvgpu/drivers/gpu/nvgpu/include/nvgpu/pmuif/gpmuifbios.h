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
#ifndef NVGPU_PMUIF_GPMUIFBIOS_H
#define NVGPU_PMUIF_GPMUIFBIOS_H

struct nv_pmu_bios_vfield_register_segment_super {
	u8 type;
	u8 low_bit;
	u8 high_bit;
};

struct nv_pmu_bios_vfield_register_segment_reg {
	struct nv_pmu_bios_vfield_register_segment_super super;
	u32 addr;
};

struct nv_pmu_bios_vfield_register_segment_index_reg {
	struct nv_pmu_bios_vfield_register_segment_super super;
	u32 addr;
	u32 reg_index;
	u32 index;
};

union nv_pmu_bios_vfield_register_segment {
	struct nv_pmu_bios_vfield_register_segment_super super;
	struct nv_pmu_bios_vfield_register_segment_reg reg;
	struct nv_pmu_bios_vfield_register_segment_index_reg index_reg;
};


#endif /* NVGPU_PMUIF_GPMUIFBIOS_H*/
