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

#ifndef NVGPU_SEC2_IF_CMN_H
#define NVGPU_SEC2_IF_CMN_H

/*
 * Define the maximum number of command sequences that can be in flight at
 * any given time.  This is dictated by the width of the sequence number
 * id ('seqNumId') stored in each sequence packet (currently 8-bits).
 */
#define NV_SEC2_MAX_NUM_SEQUENCES	256U

/*
 * Compares an unit id against the values in the unit_id enumeration and
 * verifies that the id is valid.  It is expected that the id is specified
 * as an unsigned integer.
 */
#define  NV_SEC2_UNITID_IS_VALID(id)	(((id) < NV_SEC2_UNIT_END))

/*
 * Defines the size of the surface/buffer that will be allocated to store
 * debug spew from the SEC2 ucode application when falcon-trace is enabled.
 */
#define NV_SEC2_DEBUG_SURFACE_SIZE	(32U*1024U)

/*
 * SEC2's frame-buffer interface block has several slots/indices which can
 * be bound to support DMA to various surfaces in memory. This is an
 * enumeration that gives name to each index based on type of memory-aperture
 * the index is used to access.
 *
 * Pre-Turing, NV_SEC2_DMAIDX_PHYS_VID_FN0 == NV_SEC2_DMAIDX_GUEST_PHYS_VID_BOUND.
 * From Turing, engine context is stored in GPA, requiring a separate aperture.
 *
 * Traditionally, video falcons have used the 6th index for ucode, and we will
 * continue to use that to allow legacy ucode to work seamlessly.
 *
 * Note: DO NOT CHANGE THE VALUE OF NV_SEC2_DMAIDX_UCODE. That value is used by
 * both the legacy SEC2 ucode, which assumes that it will use index 6, and by
 * SEC2 RTOS. Changing it will break legacy SEC2 ucode, unless it is updated to
 * reflect the new value.
 */

#define NV_SEC2_DMAIDX_GUEST_PHYS_VID_BOUND      0U
#define NV_SEC2_DMAIDX_VIRT                      1U
#define NV_SEC2_DMAIDX_PHYS_VID_FN0              2U
#define NV_SEC2_DMAIDX_PHYS_SYS_COH_FN0          3U
#define NV_SEC2_DMAIDX_PHYS_SYS_NCOH_FN0         4U
#define NV_SEC2_DMAIDX_GUEST_PHYS_SYS_COH_BOUND  5U
#define NV_SEC2_DMAIDX_UCODE                     6U
#define NV_SEC2_DMAIDX_GUEST_PHYS_SYS_NCOH_BOUND 7U

#endif  /* NVGPU_SEC2_IF_CMN_H */
