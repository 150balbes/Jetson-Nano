/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_CTXSW_TRACE_H
#define NVGPU_CTXSW_TRACE_H

#include <nvgpu/types.h>

struct gk20a;
struct tsg_gk20a;
struct channel_gk20a;

#define NVGPU_GPU_CTXSW_TAG_SOF                     0x00
#define NVGPU_GPU_CTXSW_TAG_CTXSW_REQ_BY_HOST       0x01
#define NVGPU_GPU_CTXSW_TAG_FE_ACK                  0x02
#define NVGPU_GPU_CTXSW_TAG_FE_ACK_WFI              0x0a
#define NVGPU_GPU_CTXSW_TAG_FE_ACK_GFXP             0x0b
#define NVGPU_GPU_CTXSW_TAG_FE_ACK_CTAP             0x0c
#define NVGPU_GPU_CTXSW_TAG_FE_ACK_CILP             0x0d
#define NVGPU_GPU_CTXSW_TAG_SAVE_END                0x03
#define NVGPU_GPU_CTXSW_TAG_RESTORE_START           0x04
#define NVGPU_GPU_CTXSW_TAG_CONTEXT_START           0x05
#define NVGPU_GPU_CTXSW_TAG_ENGINE_RESET            0xfe
#define NVGPU_GPU_CTXSW_TAG_INVALID_TIMESTAMP       0xff
#define NVGPU_GPU_CTXSW_TAG_LAST                    \
	NVGPU_GPU_CTXSW_TAG_INVALID_TIMESTAMP

#define NVGPU_GPU_CTXSW_FILTER_ISSET(n, p) \
	((p)->tag_bits[(n) / 64] &   (1 << ((n) & 63)))

#define NVGPU_GPU_CTXSW_FILTER_SIZE (NVGPU_GPU_CTXSW_TAG_LAST + 1)
#define NVGPU_FECS_TRACE_FEATURE_CONTROL_BIT 31

struct nvgpu_gpu_ctxsw_trace_filter {
	u64 tag_bits[(NVGPU_GPU_CTXSW_FILTER_SIZE + 63) / 64];
};

/*
 * The binary format of 'struct nvgpu_gpu_ctxsw_trace_entry' introduced here
 * should match that of 'struct nvgpu_ctxsw_trace_entry' defined in uapi
 * header, since this struct is intended to be a mirror copy of the uapi
 * struct.
 */
struct nvgpu_gpu_ctxsw_trace_entry {
	u8 tag;
	u8 vmid;
	u16 seqno;            /* sequence number to detect drops */
	u32 context_id;       /* context_id as allocated by FECS */
	u64 pid;              /* 64-bit is max bits of different OS pid */
	u64 timestamp;        /* 64-bit time */
};

int gk20a_ctxsw_trace_init(struct gk20a *g);

void gk20a_ctxsw_trace_channel_reset(struct gk20a *g, struct channel_gk20a *ch);
void gk20a_ctxsw_trace_tsg_reset(struct gk20a *g, struct tsg_gk20a *tsg);

void gk20a_ctxsw_trace_cleanup(struct gk20a *g);
int gk20a_ctxsw_trace_write(struct gk20a *g,
			    struct nvgpu_gpu_ctxsw_trace_entry *entry);
void gk20a_ctxsw_trace_wake_up(struct gk20a *g, int vmid);

#ifdef CONFIG_GK20A_CTXSW_TRACE
struct file;
struct vm_area_struct;

int gk20a_ctxsw_dev_mmap(struct file *filp, struct vm_area_struct *vma);
int gk20a_ctxsw_dev_ring_alloc(struct gk20a *g, void **buf, size_t *size);
int gk20a_ctxsw_dev_ring_free(struct gk20a *g);
int gk20a_ctxsw_dev_mmap_buffer(struct gk20a *g, struct vm_area_struct *vma);
#endif

u8 nvgpu_gpu_ctxsw_tags_to_common_tags(u8 tags);

#endif /*NVGPU_CTXSW_TRACE_H */
