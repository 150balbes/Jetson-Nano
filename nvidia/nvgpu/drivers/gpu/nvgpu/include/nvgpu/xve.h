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
#ifndef NVGPU_XVE_H
#define NVGPU_XVE_H

#include <nvgpu/types.h>
#include <nvgpu/log2.h>

/*
 * For the available speeds bitmap.
 */
#define GPU_XVE_SPEED_2P5	(1 << 0)
#define GPU_XVE_SPEED_5P0	(1 << 1)
#define GPU_XVE_SPEED_8P0	(1 << 2)
#define GPU_XVE_NR_SPEEDS	3

#define GPU_XVE_SPEED_MASK	(GPU_XVE_SPEED_2P5 |	\
				 GPU_XVE_SPEED_5P0 |	\
				 GPU_XVE_SPEED_8P0)

/*
 * The HW uses a 2 bit field where speed is defined by a number:
 *
 *   NV_XVE_LINK_CONTROL_STATUS_LINK_SPEED_2P5 = 1
 *   NV_XVE_LINK_CONTROL_STATUS_LINK_SPEED_5P0 = 2
 *   NV_XVE_LINK_CONTROL_STATUS_LINK_SPEED_8P0 = 3
 *
 * This isn't ideal for a bitmap with available speeds. So the external
 * APIs think about speeds as a bit in a bitmap and this function converts
 * from those bits to the actual HW speed setting.
 *
 * @speed_bit must have only 1 bit set and must be one of the 3 available
 * HW speeds. Not all chips support all speeds so use available_speeds() to
 * determine what a given chip supports.
 */
static inline const char *xve_speed_to_str(u32 speed)
{
	if (!speed || !is_power_of_2(speed) ||
	    !(speed & GPU_XVE_SPEED_MASK)) {
		return "Unknown ???";
	}

	return speed & GPU_XVE_SPEED_2P5 ? "Gen1" :
	       speed & GPU_XVE_SPEED_5P0 ? "Gen2" :
	       speed & GPU_XVE_SPEED_8P0 ? "Gen3" :
	       "Unknown ???";
}

#endif /* NVGPU_XVE_H */
