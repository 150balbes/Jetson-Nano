/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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

/*
 * This header provides constants for the binding nvidia,tegra20-pmc
 */

#ifndef _DT_BINDINGS_TEGRA_PMC_H_
#define _DT_BINDINGS_TEGRA_PMC_H_

#define PMC_WAKE_TYPE_GPIO	0
#define PMC_WAKE_TYPE_EVENT	1

#define PMC_TRIGGER_TYPE_NONE		0
#define PMC_TRIGGER_TYPE_RISING		1
#define PMC_TRIGGER_TYPE_FALLING	2
#define PMC_TRIGGER_TYPE_HIGH		4
#define PMC_TRIGGER_TYPE_LOW		8

#endif
