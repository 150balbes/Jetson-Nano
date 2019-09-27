/*
 * Tegra GK20A GPU Debugger Driver Register Ops
 *
 * Copyright (c) 2013-2018, NVIDIA CORPORATION. All rights reserved.
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
#ifndef REGOPS_GK20A_H
#define REGOPS_GK20A_H

/*
 * Register operations
 * All operations are targeted towards first channel
 * attached to debug session
 */
/* valid op values */
#define NVGPU_DBG_REG_OP_READ_32                             (0x00000000)
#define NVGPU_DBG_REG_OP_WRITE_32                            (0x00000001)
#define NVGPU_DBG_REG_OP_READ_64                             (0x00000002)
#define NVGPU_DBG_REG_OP_WRITE_64                            (0x00000003)
/* note: 8b ops are unsupported */
#define NVGPU_DBG_REG_OP_READ_08                             (0x00000004)
#define NVGPU_DBG_REG_OP_WRITE_08                            (0x00000005)

/* valid type values */
#define NVGPU_DBG_REG_OP_TYPE_GLOBAL                         (0x00000000)
#define NVGPU_DBG_REG_OP_TYPE_GR_CTX                         (0x00000001)
#define NVGPU_DBG_REG_OP_TYPE_GR_CTX_TPC                     (0x00000002)
#define NVGPU_DBG_REG_OP_TYPE_GR_CTX_SM                      (0x00000004)
#define NVGPU_DBG_REG_OP_TYPE_GR_CTX_CROP                    (0x00000008)
#define NVGPU_DBG_REG_OP_TYPE_GR_CTX_ZROP                    (0x00000010)
/*#define NVGPU_DBG_REG_OP_TYPE_FB                           (0x00000020)*/
#define NVGPU_DBG_REG_OP_TYPE_GR_CTX_QUAD                    (0x00000040)

/* valid status values */
#define NVGPU_DBG_REG_OP_STATUS_SUCCESS                      (0x00000000)
#define NVGPU_DBG_REG_OP_STATUS_INVALID_OP                   (0x00000001)
#define NVGPU_DBG_REG_OP_STATUS_INVALID_TYPE                 (0x00000002)
#define NVGPU_DBG_REG_OP_STATUS_INVALID_OFFSET               (0x00000004)
#define NVGPU_DBG_REG_OP_STATUS_UNSUPPORTED_OP               (0x00000008)
#define NVGPU_DBG_REG_OP_STATUS_INVALID_MASK                 (0x00000010)

struct nvgpu_dbg_reg_op {
	u8    op;
	u8    type;
	u8    status;
	u8    quad;
	u32   group_mask;
	u32   sub_group_mask;
	u32   offset;
	u32   value_lo;
	u32   value_hi;
	u32   and_n_mask_lo;
	u32   and_n_mask_hi;
};

struct regop_offset_range {
	u32 base:24;
	u32 count:8;
};

int exec_regops_gk20a(struct dbg_session_gk20a *dbg_s,
		      struct nvgpu_dbg_reg_op *ops,
		      u64 num_ops,
		      bool *is_current_ctx);

/* turn seriously unwieldy names -> something shorter */
#define REGOP(x) NVGPU_DBG_REG_OP_##x

bool reg_op_is_gr_ctx(u8 type);
bool reg_op_is_read(u8 op);
bool is_bar0_global_offset_whitelisted_gk20a(struct gk20a *g, u32 offset);

#endif /* REGOPS_GK20A_H */
