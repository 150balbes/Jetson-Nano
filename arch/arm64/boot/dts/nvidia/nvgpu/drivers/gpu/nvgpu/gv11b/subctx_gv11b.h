/*
 *
 * Volta GPU series Subcontext
 *
 * Copyright (c) 2016 - 2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_SUBCTX_GV11B_H
#define NVGPU_SUBCTX_GV11B_H

int gv11b_alloc_subctx_header(struct channel_gk20a *c);

void gv11b_free_subctx_header(struct channel_gk20a *c);

int gv11b_update_subctx_header(struct channel_gk20a *c, u64 gpu_va);

void gv11b_init_subcontext_pdb(struct vm_gk20a *vm,
				struct nvgpu_mem *inst_block,
				bool replayable);

#endif /* NVGPU_SUBCTX_GV11B_H */
