/*
 * GP10B Platform (SoC) Interface
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _GP10B_PLATFORM_H_
#define _GP10B_PLATFORM_H_

struct device;

int gp10b_tegra_get_clocks(struct device *dev);
int gp10b_tegra_reset_assert(struct device *dev);
int gp10b_tegra_reset_deassert(struct device *dev);
void gp10b_tegra_scale_init(struct device *dev);
long gp10b_round_clk_rate(struct device *dev, unsigned long rate);
int gp10b_clk_get_freqs(struct device *dev,
			unsigned long **freqs, int *num_freqs);
void gp10b_tegra_prescale(struct device *dev);
void gp10b_tegra_postscale(struct device *pdev, unsigned long freq);
#endif
