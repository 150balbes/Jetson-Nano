/*
 * GV11B/GV100 Graphics Context Pri Register Addressing
 *
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
#ifndef GR_PRI_GV11B_H
#define GR_PRI_GV11B_H

/*
 * These convenience macros are generally for use in the management/modificaiton
 * of the context state store for gr/compute contexts.
 */

/* Broadcast PMM defines */
#define NV_PERF_PMMFBP_FBPGS_LTC             0x00250800
#define NV_PERF_PMMFBP_FBPGS_ROP             0x00250A00
#define NV_PERF_PMMGPC_GPCGS_GPCTPCA         0x00250000
#define NV_PERF_PMMGPC_GPCGS_GPCTPCB         0x00250200
#define NV_PERF_PMMGPC_GPCS                  0x00278000
#define NV_PERF_PMMFBP_FBPS                  0x0027C000

#define PRI_PMMGS_ADDR_WIDTH                 9
#define PRI_PMMS_ADDR_WIDTH                  14

/* Get the offset to be added to the chiplet base addr to get the unicast address */
#define PRI_PMMGS_OFFSET_MASK(addr)    ((addr) & ((1 << PRI_PMMGS_ADDR_WIDTH) - 1))
#define PRI_PMMGS_BASE_ADDR_MASK(addr) ((addr) & (~((1 << PRI_PMMGS_ADDR_WIDTH) - 1)))

#define PRI_PMMS_ADDR_MASK(addr)      ((addr) & ((1 << PRI_PMMS_ADDR_WIDTH) - 1))
#define PRI_PMMS_BASE_ADDR_MASK(addr) ((addr) & (~((1 << PRI_PMMS_ADDR_WIDTH) - 1)))

#endif /* GR_PRI_GV11B_H */
