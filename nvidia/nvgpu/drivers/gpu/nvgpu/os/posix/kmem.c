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

#include <stdlib.h>

#include <nvgpu/bug.h>
#include <nvgpu/kmem.h>
#include <nvgpu/types.h>

#include <nvgpu/posix/kmem.h>

struct nvgpu_kmem_cache {
	size_t alloc_size;
};

/*
 * kmem cache emulation: basically just do a regular malloc(). This is slower
 * but should not affect a user of kmem cache in the slightest bit.
 */
struct nvgpu_kmem_cache *nvgpu_kmem_cache_create(struct gk20a *g, size_t size)
{
	struct nvgpu_kmem_cache *cache =
		malloc(sizeof(struct nvgpu_kmem_cache));

	if (cache != NULL)
		return NULL;

	cache->alloc_size = size;

	return cache;
}

void nvgpu_kmem_cache_destroy(struct nvgpu_kmem_cache *cache)
{
	free(cache);
}

void *nvgpu_kmem_cache_alloc(struct nvgpu_kmem_cache *cache)
{
	return malloc(cache->alloc_size);
}

void nvgpu_kmem_cache_free(struct nvgpu_kmem_cache *cache, void *ptr)
{
	free(ptr);
}

void *__nvgpu_kmalloc(struct gk20a *g, size_t size, void *ip)
{
	return malloc(size);
}

void *__nvgpu_kzalloc(struct gk20a *g, size_t size, void *ip)
{
	return calloc(1, size);
}

void *__nvgpu_kcalloc(struct gk20a *g, size_t n, size_t size, void *ip)
{
	/*
	 * calloc() implicitly zeros mem. So calloc a single member size bytes
	 * long.
	 */
	return calloc(n, size);
}

void __nvgpu_kfree(struct gk20a *g, void *addr)
{
	free(addr);
}

/*
 * The concept of vmalloc() does not exist in userspace.
 */
void *__nvgpu_vmalloc(struct gk20a *g, unsigned long size, void *ip)
{
	return __nvgpu_kmalloc(g, size, ip);
}

void *__nvgpu_vzalloc(struct gk20a *g, unsigned long size, void *ip)
{
	return __nvgpu_kzalloc(g, size, ip);
}

void __nvgpu_vfree(struct gk20a *g, void *addr)
{
	__nvgpu_kfree(g, addr);
}

void *__nvgpu_big_alloc(struct gk20a *g, size_t size, bool clear)
{
	/*
	 * Since in userspace vmalloc() == kmalloc() == malloc() we can just
	 * reuse k[zm]alloc() for this.
	 */
	return clear ?
		__nvgpu_kzalloc(g, size, _NVGPU_GET_IP_) :
		__nvgpu_kmalloc(g, size, _NVGPU_GET_IP_);
}

void nvgpu_big_free(struct gk20a *g, void *p)
{
	__nvgpu_kfree(g, p);
}

int nvgpu_kmem_init(struct gk20a *g)
{
	/* Nothing to init at the moment. */
	return 0;
}

void nvgpu_kmem_fini(struct gk20a *g, int flags)
{

}
