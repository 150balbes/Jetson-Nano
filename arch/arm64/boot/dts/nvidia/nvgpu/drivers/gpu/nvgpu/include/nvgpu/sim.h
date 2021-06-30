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
#ifndef NVGPU_SIM_H
#define NVGPU_SIM_H

#include <nvgpu/nvgpu_mem.h>

struct gk20a;
struct sim_nvgpu {
	struct gk20a *g;
	u32 send_ring_put;
	u32 recv_ring_get;
	u32 recv_ring_put;
	u32 sequence_base;
	struct nvgpu_mem send_bfr;
	struct nvgpu_mem recv_bfr;
	struct nvgpu_mem msg_bfr;
	void (*sim_init_late)(struct gk20a *);
	void (*remove_support)(struct gk20a *);
	void (*esc_readl)(
		struct gk20a *g, char *path, u32 index, u32 *data);
};
#ifdef __KERNEL__
#include "linux/sim.h"
#include "linux/sim_pci.h"
#elif defined(__NVGPU_POSIX__)
/* Nothing for POSIX-nvgpu. */
#else
#include <nvgpu_rmos/include/sim.h>
#include <nvgpu_rmos/include/sim_pci.h>
#endif
int nvgpu_init_sim_support(struct gk20a *g);
int nvgpu_init_sim_support_pci(struct gk20a *g);
int nvgpu_alloc_sim_buffer(struct gk20a *g, struct nvgpu_mem *mem);
void nvgpu_free_sim_buffer(struct gk20a *g, struct nvgpu_mem *mem);
void nvgpu_free_sim_support(struct gk20a *g);
void nvgpu_remove_sim_support(struct gk20a *g);

#endif /* NVGPU_SIM_H */
