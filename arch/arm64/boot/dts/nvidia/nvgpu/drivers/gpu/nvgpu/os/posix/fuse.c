/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/fuse.h>

int nvgpu_tegra_get_gpu_speedo_id(struct gk20a *g)
{
	return 0;
}

void nvgpu_tegra_fuse_write_bypass(struct gk20a *g, u32 val)
{
}

void nvgpu_tegra_fuse_write_access_sw(struct gk20a *g, u32 val)
{
}

void nvgpu_tegra_fuse_write_opt_gpu_tpc0_disable(struct gk20a *g, u32 val)
{
}

void nvgpu_tegra_fuse_write_opt_gpu_tpc1_disable(struct gk20a *g, u32 val)
{
}

int nvgpu_tegra_fuse_read_gcplex_config_fuse(struct gk20a *g, u32 *val)
{
	return -ENODEV;
}

int nvgpu_tegra_fuse_read_reserved_calib(struct gk20a *g, u32 *val)
{
	return -ENODEV;
}
