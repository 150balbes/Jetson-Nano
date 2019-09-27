/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_ERROR_NOTIFIER_H
#define NVGPU_ERROR_NOTIFIER_H

#include <nvgpu/types.h>

struct channel_gk20a;

enum {
	NVGPU_ERR_NOTIFIER_FIFO_ERROR_IDLE_TIMEOUT = 0,
	NVGPU_ERR_NOTIFIER_GR_ERROR_SW_METHOD,
	NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY,
	NVGPU_ERR_NOTIFIER_GR_EXCEPTION,
	NVGPU_ERR_NOTIFIER_GR_SEMAPHORE_TIMEOUT,
	NVGPU_ERR_NOTIFIER_GR_ILLEGAL_NOTIFY,
	NVGPU_ERR_NOTIFIER_FIFO_ERROR_MMU_ERR_FLT,
	NVGPU_ERR_NOTIFIER_PBDMA_ERROR,
	NVGPU_ERR_NOTIFIER_FECS_ERR_UNIMP_FIRMWARE_METHOD,
	NVGPU_ERR_NOTIFIER_RESETCHANNEL_VERIF_ERROR,
	NVGPU_ERR_NOTIFIER_PBDMA_PUSHBUFFER_CRC_MISMATCH,
};

void nvgpu_set_error_notifier_locked(struct channel_gk20a *ch, u32 error);
void nvgpu_set_error_notifier(struct channel_gk20a *ch, u32 error);
void nvgpu_set_error_notifier_if_empty(struct channel_gk20a *ch, u32 error);
bool nvgpu_is_error_notifier_set(struct channel_gk20a *ch, u32 error_notifier);

#endif /* NVGPU_ERROR_NOTIFIER_H */
