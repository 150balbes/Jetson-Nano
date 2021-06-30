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

#ifndef __VGPU_IVC_H__
#define __VGPU_IVC_H__

#include <nvgpu/types.h>

struct gk20a;

int vgpu_ivc_init(struct gk20a *g, u32 elems,
		const size_t *queue_sizes, u32 queue_start, u32 num_queues);
void vgpu_ivc_deinit(u32 queue_start, u32 num_queues);
void vgpu_ivc_release(void *handle);
u32 vgpu_ivc_get_server_vmid(void);
int vgpu_ivc_recv(u32 index, void **handle, void **data,
				size_t *size, u32 *sender);
int vgpu_ivc_send(u32 peer, u32 index, void *data, size_t size);
int vgpu_ivc_sendrecv(u32 peer, u32 index, void **handle,
				void **data, size_t *size);
u32 vgpu_ivc_get_peer_self(void);
void *vgpu_ivc_oob_get_ptr(u32 peer, u32 index, void **ptr,
					size_t *size);
void vgpu_ivc_oob_put_ptr(void *handle);

#endif
