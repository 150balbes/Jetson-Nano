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

#ifndef NVGPU_CTRLBOARDOBJ_H
#define NVGPU_CTRLBOARDOBJ_H

struct ctrl_boardobj {
	u8    type;
};

#define CTRL_BOARDOBJGRP_TYPE_INVALID			0x00U
#define CTRL_BOARDOBJGRP_TYPE_E32			0x01U
#define CTRL_BOARDOBJGRP_TYPE_E255			0x02U

#define CTRL_BOARDOBJGRP_E32_MAX_OBJECTS		32U

#define CTRL_BOARDOBJGRP_E255_MAX_OBJECTS		255U

#define CTRL_BOARDOBJ_MAX_BOARD_OBJECTS                                 \
	CTRL_BOARDOBJGRP_E32_MAX_OBJECTS

#define CTRL_BOARDOBJ_IDX_INVALID			255U

#define CTRL_BOARDOBJGRP_MASK_MASK_ELEMENT_BIT_SIZE	32U

#define CTRL_BOARDOBJGRP_MASK_MASK_ELEMENT_INDEX(_bit)                  \
	((_bit) / CTRL_BOARDOBJGRP_MASK_MASK_ELEMENT_BIT_SIZE)

#define CTRL_BOARDOBJGRP_MASK_MASK_ELEMENT_OFFSET(_bit)                 \
	((_bit) % CTRL_BOARDOBJGRP_MASK_MASK_ELEMENT_BIT_SIZE)

#define CTRL_BOARDOBJGRP_MASK_DATA_SIZE(_bits)                          \
	(CTRL_BOARDOBJGRP_MASK_MASK_ELEMENT_INDEX((_bits) - 1U) + 1U)


#define CTRL_BOARDOBJGRP_MASK_ARRAY_START_SIZE		1U
#define CTRL_BOARDOBJGRP_MASK_ARRAY_EXTENSION_SIZE(_bits)          \
	(CTRL_BOARDOBJGRP_MASK_DATA_SIZE(_bits) -                           \
	 CTRL_BOARDOBJGRP_MASK_ARRAY_START_SIZE)

struct ctrl_boardobjgrp_mask {
	u32   data[1];
};

struct ctrl_boardobjgrp_mask_e32 {
	struct ctrl_boardobjgrp_mask super;
};

struct ctrl_boardobjgrp_mask_e255 {
	struct ctrl_boardobjgrp_mask super;
	u32   data_e255[7];
};

struct ctrl_boardobjgrp_super {
	struct ctrl_boardobjgrp_mask obj_mask;
};

struct ctrl_boardobjgrp_e32 {
	struct ctrl_boardobjgrp_mask_e32 obj_mask;
};

struct  CTRL_boardobjgrp_e255 {
	struct ctrl_boardobjgrp_mask_e255 obj_mask;
};

struct ctrl_boardobjgrp {
	u32    obj_mask;
};

#endif /* NVGPU_CTRLBOARDOBJ_H */
