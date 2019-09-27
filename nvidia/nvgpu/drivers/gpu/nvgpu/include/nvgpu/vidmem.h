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

#ifndef NVGPU_VIDMEM_H
#define NVGPU_VIDMEM_H

#include <nvgpu/types.h>
#include <nvgpu/errno.h>

struct gk20a;
struct mm_gk20a;
struct nvgpu_mem;

struct nvgpu_vidmem_buf {
	/*
	 * Must be a pointer since control of this mem is passed over to the
	 * vidmem background clearing thread when the vidmem buf is freed.
	 */
	struct nvgpu_mem	*mem;

	struct gk20a		*g;

	/*
	 * Filled in by each OS - this holds the necessary data to export this
	 * buffer to userspace. This will eventually be replaced by a struct
	 * which shall be defined in the OS specific vidmem.h header file.
	 */
	void			*priv;
};

#if defined(CONFIG_GK20A_VIDMEM)

/**
 * nvgpu_vidmem_user_alloc - Allocates a vidmem buffer for userspace
 *
 * @g     - The GPU.
 * @bytes - Size of the buffer in bytes.
 *
 * Allocate a generic (OS agnostic) vidmem buffer. This does not allocate the OS
 * specific interfacing for userspace sharing. Instead is is expected that the
 * OS specific code will allocate that OS specific data and add it to this
 * buffer.
 *
 * The buffer allocated here is intended to use used by userspace, hence the
 * extra struct over nvgpu_mem. If a vidmem buffer is needed by the kernel
 * driver only then a simple nvgpu_dma_alloc_vid() or the like is sufficient.
 *
 * Returns a pointer to a vidmem buffer on success, 0 otherwise.
 */
struct nvgpu_vidmem_buf *nvgpu_vidmem_user_alloc(struct gk20a *g, size_t bytes);

void nvgpu_vidmem_buf_free(struct gk20a *g, struct nvgpu_vidmem_buf *buf);

int nvgpu_vidmem_clear_list_enqueue(struct gk20a *g, struct nvgpu_mem *mem);

bool nvgpu_addr_is_vidmem_page_alloc(u64 addr);
int nvgpu_vidmem_get_space(struct gk20a *g, u64 *space);

void nvgpu_vidmem_destroy(struct gk20a *g);
int nvgpu_vidmem_init(struct mm_gk20a *mm);

int nvgpu_vidmem_clear(struct gk20a *g, struct nvgpu_mem *mem);

void nvgpu_vidmem_thread_pause_sync(struct mm_gk20a *mm);
void nvgpu_vidmem_thread_unpause(struct mm_gk20a *mm);

#else /* !defined(CONFIG_GK20A_VIDMEM) */

/*
 * When VIDMEM support is not present this interface is used.
 */

static inline bool nvgpu_addr_is_vidmem_page_alloc(u64 addr)
{
	return false;
}

static inline int nvgpu_vidmem_buf_alloc(struct gk20a *g, size_t bytes)
{
	return -ENOSYS;
}

static inline void nvgpu_vidmem_buf_free(struct gk20a *g,
					 struct nvgpu_vidmem_buf *buf)
{
}

static inline int nvgpu_vidmem_get_space(struct gk20a *g, u64 *space)
{
	return -ENOSYS;
}

static inline void nvgpu_vidmem_destroy(struct gk20a *g)
{
}

static inline int nvgpu_vidmem_init(struct mm_gk20a *mm)
{
	return 0;
}

static inline int nvgpu_vidmem_clear_all(struct gk20a *g)
{
	return -ENOSYS;
}

static inline int nvgpu_vidmem_clear(struct gk20a *g,
					      struct nvgpu_mem *mem)
{
	return -ENOSYS;
}

static inline void nvgpu_vidmem_thread_pause_sync(struct mm_gk20a *mm)
{
}

static inline void nvgpu_vidmem_thread_unpause(struct mm_gk20a *mm)
{
}

#endif /* !defined(CONFIG_GK20A_VIDMEM) */

/*
 * Simple macro for VIDMEM debugging.
 */
#define vidmem_dbg(g, fmt, args...)			\
	nvgpu_log(g, gpu_dbg_vidmem, fmt, ##args);	\

#endif /* NVGPU_VIDMEM_H */
