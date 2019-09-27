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

#ifndef NVGPU_BOARDOBJGRPMASK_H
#define NVGPU_BOARDOBJGRPMASK_H

#include "ctrl/ctrlboardobj.h"


/*
* Board Object Group Mask super-structure.
* Used to unify access to all BOARDOBJGRPMASK_E** child classes
*/
struct boardobjgrpmask {
	/* Number of bits supported by the mask */
	u8  bitcount;
	/* Number of 32-bit words required to store all @ref bitCount bits */
	u8  maskdatacount;
	/*
	* Bit-mask of used-bits within last 32-bit word. Used to
	* normalize data
	*/
	u32 lastmaskfilter;
	/*
	* Start of the array of 32-bit words representing the bit-mask
	* Must be the last element of the structure.
	*/
	 u32 data[CTRL_BOARDOBJGRP_MASK_ARRAY_START_SIZE];
};

struct boardobjgrpmask_e32 {
	/*
	* BOARDOBJGRPMASK super-class. Must be the first element of the
	* structure.
	*/
	struct boardobjgrpmask super;
	 /*u32   data_e32[1]; */
};

struct boardobjgrpmask_e255 {
	/*
	* BOARDOBJGRPMASK super-class. Must be the first element of the
	* structure.
	*/
	struct boardobjgrpmask super;
	u32   data_e255[254];
};

/* Init and I/O operations.*/
u32 boardobjgrpmask_init(struct boardobjgrpmask *mask, u8 bitsize,
		struct ctrl_boardobjgrp_mask *extmask);
u32 boardobjgrpmask_import(struct boardobjgrpmask *mask, u8 bitsize,
		struct ctrl_boardobjgrp_mask *extmask);
u32 boardobjgrpmask_export(struct boardobjgrpmask *mask, u8 bitsize,
		struct ctrl_boardobjgrp_mask *extmask);

/* Operations on all bits of a single mask.*/
u32 boardobjgrpmask_clr(struct boardobjgrpmask *mask);
u32 boardobjgrpmask_set(struct boardobjgrpmask *mask);
u32 boardobjgrpmask_inv(struct boardobjgrpmask *mask);
bool boardobjgrpmask_iszero(struct boardobjgrpmask *mask);
u8 boardobjgrpmask_bitsetcount(struct boardobjgrpmask *mask);
u8 boardobjgrpmask_bitidxlowest(struct boardobjgrpmask *mask);
u8 boardobjgrpmask_bitidxhighest(struct boardobjgrpmask *mask);

/* Operations on a single bit of a single mask */
int boardobjgrpmask_bitclr(struct boardobjgrpmask *mask, u8 bitidx);
int boardobjgrpmask_bitset(struct boardobjgrpmask *mask, u8 bitidx);
u32 boardobjgrpmask_bitinv(struct boardobjgrpmask *mask, u8 bitidx);
bool boardobjgrpmask_bitget(struct boardobjgrpmask *mask, u8 bitidx);

/* Operations on a multiple masks */
u32 boardobjgrpmask_and(struct boardobjgrpmask *dst,
			struct boardobjgrpmask *op1,
			struct boardobjgrpmask *op2);
u32 boardobjgrpmask_or(struct boardobjgrpmask *dst, struct boardobjgrpmask *op1,
		       struct boardobjgrpmask *op2);
u32 boardobjgrpmask_xor(struct boardobjgrpmask *dst,
			struct boardobjgrpmask *op1,
			struct boardobjgrpmask *op2);

/* Special interfaces */
u32 boardobjgrpmask_copy(struct boardobjgrpmask *dst,
		struct boardobjgrpmask *src);
bool boardobjgrpmask_sizeeq(struct boardobjgrpmask *op1,
		struct boardobjgrpmask *op2);
bool boardobjgrpmask_issubset(struct boardobjgrpmask *op1,
		struct boardobjgrpmask *op2);

/* init boardobjgrpmask_e32 structure */
#define boardobjgrpmask_e32_init(pmaske32, pextmask)                           \
	boardobjgrpmask_init(&(pmaske32)->super,                               \
		CTRL_BOARDOBJGRP_E32_MAX_OBJECTS, (pextmask))

/* init boardobjgrpmask_e255 structure */
#define boardobjgrpmask_e255_init(pmaske255, pextmask)                         \
	boardobjgrpmask_init(&(pmaske255)->super,                              \
		CTRL_BOARDOBJGRP_E255_MAX_OBJECTS, (pextmask))

#endif /* NVGPU_BOARDOBJGRPMASK_H */
