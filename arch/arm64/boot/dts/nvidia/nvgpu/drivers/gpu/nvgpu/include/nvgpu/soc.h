/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_SOC_H
#define NVGPU_SOC_H

#include <nvgpu/types.h>

struct gk20a;

bool nvgpu_platform_is_silicon(struct gk20a *g);
bool nvgpu_platform_is_simulation(struct gk20a *g);
bool nvgpu_platform_is_fpga(struct gk20a *g);
bool nvgpu_is_hypervisor_mode(struct gk20a *g);
bool nvgpu_is_bpmp_running(struct gk20a *g);
bool nvgpu_is_soc_t194_a01(struct gk20a *g);
int nvgpu_init_soc_vars(struct gk20a *g);

#endif /* NVGPU_SOC_H */
