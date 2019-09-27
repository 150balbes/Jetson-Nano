#ifndef _DT_BINDINGS_MEMORY_TEGRA194_SWGROUP_H
#define _DT_BINDINGS_MEMORY_TEGRA194_SWGROUP_H
/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
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
 * This is the t19x specific component of the new SID dt-binding.
 */
#define TEGRA_SID_RCE		0x2a	/* 42 */
#define TEGRA_SID_RCE_VM2	0x2b	/* 43 */

#define TEGRA_SID_RCE_RM	0x2F	/* 47 */
#define TEGRA_SID_VIFALC	0x30	/* 48 */
#define TEGRA_SID_ISPFALC	0x31	/* 49 */

#define TEGRA_SID_MIU		0x50	/* 80 */

#define TEGRA_SID_NVDLA0	0x51	/* 81 */
#define TEGRA_SID_NVDLA1	0x52	/* 82 */

#define TEGRA_SID_PVA0		0x53	/* 83 */
#define TEGRA_SID_PVA1		0x54	/* 84 */

#define TEGRA_SID_NVENC1	0x55	/* 85 */

#define TEGRA_SID_PCIE0		0x56	/* 86 */
#define TEGRA_SID_PCIE1		0x57	/* 87 */
#define TEGRA_SID_PCIE2		0x58	/* 88 */
#define TEGRA_SID_PCIE3		0x59	/* 89 */
#define TEGRA_SID_PCIE4		0x5A	/* 90 */
#define TEGRA_SID_PCIE5		0x5B	/* 91 */

#define TEGRA_SID_NVDEC1	0x5C	/* 92 */

#define TEGRA_SID_RCE_VM3	0x61	/* 97 */

#define TEGRA_SID_VI_VM2	0x62	/* 98 */
#define TEGRA_SID_VI_VM3	0x63	/* 99 */

#endif
