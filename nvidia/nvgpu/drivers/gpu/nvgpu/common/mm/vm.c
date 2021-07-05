/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bug.h>
#include <nvgpu/log.h>
#include <nvgpu/log2.h>
#include <nvgpu/dma.h>
#include <nvgpu/vm.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/lock.h>
#include <nvgpu/list.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/enabled.h>
#include <nvgpu/sizes.h>
#include <nvgpu/timers.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/vgpu/vm.h>

#include "gk20a/mm_gk20a.h"

struct nvgpu_ctag_buffer_info {
	u64			size;
	u32			pgsz_idx;
	u32			flags;

	s16			compr_kind;
	s16			incompr_kind;

	u32			ctag_lines;
};

static int nvgpu_vm_compute_compression(struct vm_gk20a *vm,
					struct nvgpu_ctag_buffer_info *binfo);

static void __nvgpu_vm_unmap(struct nvgpu_mapped_buf *mapped_buffer,
			     struct vm_gk20a_mapping_batch *batch);

int vm_aspace_id(struct vm_gk20a *vm)
{
	return (vm->as_share != NULL) ? vm->as_share->id : -1;
}

/*
 * Determine how many bits of the address space each last level PDE covers. For
 * example, for gp10b, with a last level address bit PDE range of 28 to 21 the
 * amount of memory each last level PDE addresses is 21 bits - i.e 2MB.
 */
int nvgpu_vm_pde_coverage_bit_count(struct vm_gk20a *vm)
{
	int final_pde_level = 0;

	/*
	 * Find the second to last level of the page table programming
	 * heirarchy: the last level is PTEs so we really want the level
	 * before that which is the last level of PDEs.
	 */
	while (vm->mmu_levels[final_pde_level + 2].update_entry) {
		final_pde_level++;
	}

	return vm->mmu_levels[final_pde_level].lo_bit[0];
}

static void __nvgpu_vm_free_entries(struct vm_gk20a *vm,
				    struct nvgpu_gmmu_pd *pd,
				    int level)
{
	int i;

	if (pd->mem) {
		nvgpu_pd_free(vm, pd);
		pd->mem = NULL;
	}

	if (pd->entries) {
		for (i = 0; i < pd->num_entries; i++) {
			__nvgpu_vm_free_entries(vm, &pd->entries[i],
					      level + 1);
		}
		nvgpu_vfree(vm->mm->g, pd->entries);
		pd->entries = NULL;
	}
}

static void nvgpu_vm_free_entries(struct vm_gk20a *vm,
				  struct nvgpu_gmmu_pd *pdb)
{
	struct gk20a *g = vm->mm->g;
	int i;

	nvgpu_pd_cache_free_direct(g, pdb);

	if (pdb->entries == NULL) {
		return;
	}

	for (i = 0; i < pdb->num_entries; i++) {
		__nvgpu_vm_free_entries(vm, &pdb->entries[i], 1);
	}

	nvgpu_vfree(g, pdb->entries);
	pdb->entries = NULL;
}

u64 __nvgpu_vm_alloc_va(struct vm_gk20a *vm, u64 size, u32 pgsz_idx)

{
	struct gk20a *g = vm->mm->g;
	struct nvgpu_allocator *vma = NULL;
	u64 addr;
	u64 page_size = vm->gmmu_page_sizes[pgsz_idx];

	vma = vm->vma[pgsz_idx];

	if (vm->guest_managed) {
		nvgpu_err(g, "Illegal GPU allocation on behalf of guest OS");
		return 0;
	}

	if (pgsz_idx >= GMMU_NR_PAGE_SIZES) {
		nvgpu_err(g, "(%s) invalid page size requested", vma->name);
		return 0;
	}

	if ((pgsz_idx == GMMU_PAGE_SIZE_BIG) && !vm->big_pages) {
		nvgpu_err(g, "(%s) unsupportd page size requested", vma->name);
		return 0;
	}

	/* Be certain we round up to page_size if needed */
	size = (size + ((u64)page_size - 1U)) & ~((u64)page_size - 1U);

	addr = nvgpu_alloc_pte(vma, size, page_size);
	if (addr == 0ULL) {
		nvgpu_err(g, "(%s) oom: sz=0x%llx", vma->name, size);
		return 0;
	}

	return addr;
}

int __nvgpu_vm_free_va(struct vm_gk20a *vm, u64 addr, u32 pgsz_idx)
{
	struct nvgpu_allocator *vma = vm->vma[pgsz_idx];

	nvgpu_free(vma, addr);

	return 0;
}

void nvgpu_vm_mapping_batch_start(struct vm_gk20a_mapping_batch *mapping_batch)
{
	memset(mapping_batch, 0, sizeof(*mapping_batch));
	mapping_batch->gpu_l2_flushed = false;
	mapping_batch->need_tlb_invalidate = false;
}

void nvgpu_vm_mapping_batch_finish_locked(
	struct vm_gk20a *vm, struct vm_gk20a_mapping_batch *mapping_batch)
{
	/* hanging kref_put batch pointer? */
	WARN_ON(vm->kref_put_batch == mapping_batch);

	if (mapping_batch->need_tlb_invalidate) {
		struct gk20a *g = gk20a_from_vm(vm);
		g->ops.fb.tlb_invalidate(g, vm->pdb.mem);
	}
}

void nvgpu_vm_mapping_batch_finish(struct vm_gk20a *vm,
				   struct vm_gk20a_mapping_batch *mapping_batch)
{
	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	nvgpu_vm_mapping_batch_finish_locked(vm, mapping_batch);
	nvgpu_mutex_release(&vm->update_gmmu_lock);
}

/*
 * Determine if the passed address space can support big pages or not.
 */
bool nvgpu_big_pages_possible(struct vm_gk20a *vm, u64 base, u64 size)
{
	u64 mask = ((u64)vm->big_page_size << 10ULL) - 1ULL;
	u64 base_big_page = base & mask;
	u64 size_big_page = size & mask;

	if (base_big_page != 0ULL || size_big_page != 0ULL) {
		return false;
	}
	return true;
}

/*
 * Initialize a semaphore pool. Just return successfully if we do not need
 * semaphores (i.e when sync-pts are active).
 */
static int nvgpu_init_sema_pool(struct vm_gk20a *vm)
{
	struct nvgpu_semaphore_sea *sema_sea;
	struct mm_gk20a *mm = vm->mm;
	struct gk20a *g = mm->g;
	int err;

	/*
	 * Don't waste the memory on semaphores if we don't need them.
	 */
	if (nvgpu_has_syncpoints(g)) {
		return 0;
	}

	if (vm->sema_pool) {
		return 0;
	}

	sema_sea = nvgpu_semaphore_sea_create(g);
	if (sema_sea == NULL) {
		return -ENOMEM;
	}

	err = nvgpu_semaphore_pool_alloc(sema_sea, &vm->sema_pool);
	if (err != 0) {
		return err;
	}

	/*
	 * Allocate a chunk of GPU VA space for mapping the semaphores. We will
	 * do a fixed alloc in the kernel VM so that all channels have the same
	 * RO address range for the semaphores.
	 *
	 * !!! TODO: cleanup.
	 */
	sema_sea->gpu_va = nvgpu_alloc_fixed(&vm->kernel,
					     vm->va_limit -
					     mm->channel.kernel_size,
					     512U * PAGE_SIZE,
					     SZ_4K);
	if (sema_sea->gpu_va == 0ULL) {
		nvgpu_free(&vm->kernel, sema_sea->gpu_va);
		nvgpu_vm_put(vm);
		return -ENOMEM;
	}

	err = nvgpu_semaphore_pool_map(vm->sema_pool, vm);
	if (err) {
		nvgpu_semaphore_pool_unmap(vm->sema_pool, vm);
		nvgpu_free(vm->vma[GMMU_PAGE_SIZE_SMALL],
			   vm->sema_pool->gpu_va);
		return err;
	}

	return 0;
}

/*
 * Initialize a preallocated vm
 */
int __nvgpu_vm_init(struct mm_gk20a *mm,
			   struct vm_gk20a *vm,
			   u32 big_page_size,
			   u64 low_hole,
			   u64 kernel_reserved,
			   u64 aperture_size,
			   bool big_pages,
			   bool userspace_managed,
			   char *name)
{
	int err = 0;
	char alloc_name[32];
	u64 kernel_vma_flags;
	u64 user_vma_start, user_vma_limit;
	u64 user_lp_vma_start, user_lp_vma_limit;
	u64 kernel_vma_start, kernel_vma_limit;
	struct gk20a *g = gk20a_from_mm(mm);

	if (WARN_ON(kernel_reserved + low_hole > aperture_size)) {
		return -ENOMEM;
	}

	if (WARN_ON(vm->guest_managed && kernel_reserved != 0U)) {
		return -EINVAL;
	}

	nvgpu_log_info(g, "Init space for %s: valimit=0x%llx, "
		       "LP size=0x%x lowhole=0x%llx",
		       name, aperture_size,
		       (unsigned int)big_page_size, low_hole);

	vm->mm = mm;

	vm->gmmu_page_sizes[GMMU_PAGE_SIZE_SMALL]  = SZ_4K;
	vm->gmmu_page_sizes[GMMU_PAGE_SIZE_BIG]    = big_page_size;
	vm->gmmu_page_sizes[GMMU_PAGE_SIZE_KERNEL] = SZ_4K;

	/* Set up vma pointers. */
	vm->vma[GMMU_PAGE_SIZE_SMALL]  = &vm->user;
	vm->vma[GMMU_PAGE_SIZE_BIG]    = &vm->user;
	vm->vma[GMMU_PAGE_SIZE_KERNEL] = &vm->kernel;
	if (!nvgpu_is_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES)) {
		vm->vma[GMMU_PAGE_SIZE_BIG] = &vm->user_lp;
	}

	vm->va_start  = low_hole;
	vm->va_limit  = aperture_size;

	vm->big_page_size     = vm->gmmu_page_sizes[GMMU_PAGE_SIZE_BIG];
	vm->userspace_managed = userspace_managed;
	vm->mmu_levels        = g->ops.mm.get_mmu_levels(g, vm->big_page_size);

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	if (g->is_virtual && userspace_managed) {
		nvgpu_err(g, "vGPU: no userspace managed addr space support");
		return -ENOSYS;
	}
	if (g->is_virtual && vgpu_vm_init(g, vm)) {
		nvgpu_err(g, "Failed to init vGPU VM!");
		return -ENOMEM;
	}
#endif

	/* Initialize the page table data structures. */
	strncpy(vm->name, name, sizeof(vm->name));
	vm->name[sizeof(vm->name) - 1] = '\0';
	err = nvgpu_gmmu_init_page_table(vm);
	if (err) {
		goto clean_up_vgpu_vm;
	}

	/* Setup vma limits. */
	if (kernel_reserved + low_hole < aperture_size) {
		/*
		 * If big_pages are disabled for this VM then it only makes
		 * sense to make one VM, same as if the unified address flag
		 * is set.
		 */
		if (!big_pages ||
		    nvgpu_is_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES)) {
			user_vma_start = low_hole;
			user_vma_limit = vm->va_limit - kernel_reserved;
			user_lp_vma_start = user_vma_limit;
			user_lp_vma_limit = user_vma_limit;
		} else {
			user_vma_start = low_hole;
			user_vma_limit = nvgpu_gmmu_va_small_page_limit();
			user_lp_vma_start = nvgpu_gmmu_va_small_page_limit();
			user_lp_vma_limit = vm->va_limit - kernel_reserved;
		}
	} else {
		user_vma_start = 0;
		user_vma_limit = 0;
		user_lp_vma_start = 0;
		user_lp_vma_limit = 0;
	}
	kernel_vma_start = vm->va_limit - kernel_reserved;
	kernel_vma_limit = vm->va_limit;

	nvgpu_log_info(g, "user_vma     [0x%llx,0x%llx)",
		       user_vma_start, user_vma_limit);
	nvgpu_log_info(g, "user_lp_vma  [0x%llx,0x%llx)",
		       user_lp_vma_start, user_lp_vma_limit);
	nvgpu_log_info(g, "kernel_vma   [0x%llx,0x%llx)",
		       kernel_vma_start, kernel_vma_limit);

	if (WARN_ON(user_vma_start > user_vma_limit) ||
	    WARN_ON(user_lp_vma_start > user_lp_vma_limit) ||
		WARN_ON(!vm->guest_managed && kernel_vma_start >= kernel_vma_limit)) {
		err = -EINVAL;
		goto clean_up_page_tables;
	}

	kernel_vma_flags = (kernel_reserved + low_hole) == aperture_size ?
		0ULL : GPU_ALLOC_GVA_SPACE;

	/*
	 * A "user" area only makes sense for the GVA spaces. For VMs where
	 * there is no "user" area user_vma_start will be equal to
	 * user_vma_limit (i.e a 0 sized space). In such a situation the kernel
	 * area must be non-zero in length.
	 */
	if (user_vma_start >= user_vma_limit &&
	    kernel_vma_start >= kernel_vma_limit) {
		err = -EINVAL;
		goto clean_up_page_tables;
	}

	/*
	 * Determine if big pages are possible in this VM. If a split address
	 * space is used then check the user_lp vma instead of the user vma.
	 */
	if (nvgpu_is_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES)) {
		vm->big_pages = big_pages &&
			nvgpu_big_pages_possible(vm, user_vma_start,
					user_vma_limit - user_vma_start);
	} else {
		vm->big_pages = big_pages &&
			nvgpu_big_pages_possible(vm, user_lp_vma_start,
					user_lp_vma_limit - user_lp_vma_start);
	}

	/*
	 * User VMA.
	 */
	if (user_vma_start < user_vma_limit) {
		snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s", name);
		err = nvgpu_buddy_allocator_init(g, &vm->user,
						 vm, alloc_name,
						 user_vma_start,
						 user_vma_limit -
						 user_vma_start,
						 SZ_4K,
						 GPU_BALLOC_MAX_ORDER,
						 GPU_ALLOC_GVA_SPACE);
		if (err) {
			goto clean_up_page_tables;
		}
	} else {
		/*
		 * Make these allocator pointers point to the kernel allocator
		 * since we still use the legacy notion of page size to choose
		 * the allocator.
		 */
		vm->vma[0] = &vm->kernel;
		vm->vma[1] = &vm->kernel;
	}

	/*
	 * User VMA for large pages when a split address range is used.
	 */
	if (user_lp_vma_start < user_lp_vma_limit) {
		snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s_lp", name);
		err = nvgpu_buddy_allocator_init(g, &vm->user_lp,
						 vm, alloc_name,
						 user_lp_vma_start,
						 user_lp_vma_limit -
						 user_lp_vma_start,
						 vm->big_page_size,
						 GPU_BALLOC_MAX_ORDER,
						 GPU_ALLOC_GVA_SPACE);
		if (err) {
			goto clean_up_allocators;
		}
	}

	/*
	 * Kernel VMA. Must always exist for an address space.
	 */
	snprintf(alloc_name, sizeof(alloc_name), "gk20a_%s-sys", name);
	err = nvgpu_buddy_allocator_init(g, &vm->kernel,
					 vm, alloc_name,
					 kernel_vma_start,
					 kernel_vma_limit - kernel_vma_start,
					 SZ_4K,
					 GPU_BALLOC_MAX_ORDER,
					 kernel_vma_flags);
	if (err) {
		goto clean_up_allocators;
	}

	vm->mapped_buffers = NULL;

	err = nvgpu_mutex_init(&vm->syncpt_ro_map_lock);
	if (err != 0) {
		nvgpu_err(g,
			   "Error in syncpt_ro_map_lock mutex initialization");
		goto clean_up_allocators;
	}

	err = nvgpu_mutex_init(&vm->update_gmmu_lock);
	if (err != 0) {
		nvgpu_err(g, "Error in update_gmmu_lock mutex initialization");
		goto clean_up_ro_map_lock;
	}

	nvgpu_ref_init(&vm->ref);
	nvgpu_init_list_node(&vm->vm_area_list);

	/*
	 * This is only necessary for channel address spaces. The best way to
	 * distinguish channel address spaces from other address spaces is by
	 * size - if the address space is 4GB or less, it's not a channel.
	 */
	if (vm->va_limit > 4ULL * SZ_1G) {
		err = nvgpu_init_sema_pool(vm);
		if (err) {
			goto clean_up_gmmu_lock;
		}
	}

	return 0;

clean_up_gmmu_lock:
	nvgpu_mutex_destroy(&vm->update_gmmu_lock);
clean_up_ro_map_lock:
	nvgpu_mutex_destroy(&vm->syncpt_ro_map_lock);
clean_up_allocators:
	if (nvgpu_alloc_initialized(&vm->kernel)) {
		nvgpu_alloc_destroy(&vm->kernel);
	}
	if (nvgpu_alloc_initialized(&vm->user)) {
		nvgpu_alloc_destroy(&vm->user);
	}
	if (nvgpu_alloc_initialized(&vm->user_lp)) {
		nvgpu_alloc_destroy(&vm->user_lp);
	}
clean_up_page_tables:
	/* Cleans up nvgpu_gmmu_init_page_table() */
	nvgpu_pd_cache_free_direct(g, &vm->pdb);
clean_up_vgpu_vm:
#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	if (g->is_virtual)
		vgpu_vm_remove(vm);
#endif
	return err;
}

/**
 * nvgpu_init_vm() - Initialize an address space.
 *
 * @mm - Parent MM.
 * @vm - The VM to init.
 * @big_page_size - Size of big pages associated with this VM.
 * @low_hole - The size of the low hole (unaddressable memory at the bottom of
 *	       the address space).
 * @kernel_reserved - Space reserved for kernel only allocations.
 * @aperture_size - Total size of the aperture.
 * @big_pages - If true then big pages are possible in the VM. Note this does
 *              not guarantee that big pages will be possible.
 * @name - Name of the address space.
 *
 * This function initializes an address space according to the following map:
 *
 *     +--+ 0x0
 *     |  |
 *     +--+ @low_hole
 *     |  |
 *     ~  ~   This is the "user" section.
 *     |  |
 *     +--+ @aperture_size - @kernel_reserved
 *     |  |
 *     ~  ~   This is the "kernel" section.
 *     |  |
 *     +--+ @aperture_size
 *
 * The user section is therefor what ever is left over after the @low_hole and
 * @kernel_reserved memory have been portioned out. The @kernel_reserved is
 * always persent at the top of the memory space and the @low_hole is always at
 * the bottom.
 *
 * For certain address spaces a "user" section makes no sense (bar1, etc) so in
 * such cases the @kernel_reserved and @low_hole should sum to exactly
 * @aperture_size.
 */
struct vm_gk20a *nvgpu_vm_init(struct gk20a *g,
			       u32 big_page_size,
			       u64 low_hole,
			       u64 kernel_reserved,
			       u64 aperture_size,
			       bool big_pages,
			       bool userspace_managed,
			       char *name)
{
	struct vm_gk20a *vm = nvgpu_kzalloc(g, sizeof(*vm));

	if (vm == NULL) {
		return NULL;
	}

	if (__nvgpu_vm_init(&g->mm, vm, big_page_size, low_hole,
			    kernel_reserved, aperture_size, big_pages,
			    userspace_managed, name)) {
		nvgpu_kfree(g, vm);
		return NULL;
	}

	return vm;
}

/*
 * Cleanup the VM!
 */
static void __nvgpu_vm_remove(struct vm_gk20a *vm)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct nvgpu_vm_area *vm_area, *vm_area_tmp;
	struct nvgpu_rbtree_node *node = NULL;
	struct gk20a *g = vm->mm->g;

	/*
	 * Do this outside of the update_gmmu_lock since unmapping the semaphore
	 * pool involves unmapping a GMMU mapping which means aquiring the
	 * update_gmmu_lock.
	 */
	if (!nvgpu_has_syncpoints(g)) {
		if (vm->sema_pool) {
			nvgpu_semaphore_pool_unmap(vm->sema_pool, vm);
			nvgpu_semaphore_pool_put(vm->sema_pool);
		}
	}

	if (nvgpu_mem_is_valid(&g->syncpt_mem) &&
			vm->syncpt_ro_map_gpu_va != 0ULL) {
		nvgpu_gmmu_unmap(vm, &g->syncpt_mem,
				vm->syncpt_ro_map_gpu_va);
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	while (node) {
		mapped_buffer = mapped_buffer_from_rbtree_node(node);
		__nvgpu_vm_unmap(mapped_buffer, NULL);
		nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	}

	/* destroy remaining reserved memory areas */
	nvgpu_list_for_each_entry_safe(vm_area, vm_area_tmp,
			&vm->vm_area_list,
			nvgpu_vm_area, vm_area_list) {
		nvgpu_list_del(&vm_area->vm_area_list);
		nvgpu_kfree(vm->mm->g, vm_area);
	}

	if (nvgpu_alloc_initialized(&vm->kernel)) {
		nvgpu_alloc_destroy(&vm->kernel);
	}
	if (nvgpu_alloc_initialized(&vm->user)) {
		nvgpu_alloc_destroy(&vm->user);
	}
	if (nvgpu_alloc_initialized(&vm->user_lp)) {
		nvgpu_alloc_destroy(&vm->user_lp);
	}

	nvgpu_vm_free_entries(vm, &vm->pdb);

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	if (g->is_virtual)
		vgpu_vm_remove(vm);
#endif

	nvgpu_mutex_release(&vm->update_gmmu_lock);
	nvgpu_mutex_destroy(&vm->update_gmmu_lock);

	nvgpu_mutex_destroy(&vm->syncpt_ro_map_lock);
	nvgpu_kfree(g, vm);
}

static void __nvgpu_vm_remove_ref(struct nvgpu_ref *ref)
{
	struct vm_gk20a *vm = container_of(ref, struct vm_gk20a, ref);

	__nvgpu_vm_remove(vm);
}

void nvgpu_vm_get(struct vm_gk20a *vm)
{
	nvgpu_ref_get(&vm->ref);
}

void nvgpu_vm_put(struct vm_gk20a *vm)
{
	nvgpu_ref_put(&vm->ref, __nvgpu_vm_remove_ref);
}

int nvgpu_insert_mapped_buf(struct vm_gk20a *vm,
			    struct nvgpu_mapped_buf *mapped_buffer)
{
	mapped_buffer->node.key_start = mapped_buffer->addr;
	mapped_buffer->node.key_end = mapped_buffer->addr + mapped_buffer->size;

	nvgpu_rbtree_insert(&mapped_buffer->node, &vm->mapped_buffers);

	return 0;
}

void nvgpu_remove_mapped_buf(struct vm_gk20a *vm,
			     struct nvgpu_mapped_buf *mapped_buffer)
{
	nvgpu_rbtree_unlink(&mapped_buffer->node, &vm->mapped_buffers);
}

struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf(
	struct vm_gk20a *vm, u64 addr)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_search(addr, &node, root);
	if (node == NULL) {
		return NULL;
	}

	return mapped_buffer_from_rbtree_node(node);
}

struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_range(
	struct vm_gk20a *vm, u64 addr)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_range_search(addr, &node, root);
	if (node == NULL) {
		return NULL;
	}

	return mapped_buffer_from_rbtree_node(node);
}

struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_less_than(
	struct vm_gk20a *vm, u64 addr)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_less_than_search(addr, &node, root);
	if (node == NULL) {
		return NULL;
	}

	return mapped_buffer_from_rbtree_node(node);
}

int nvgpu_vm_get_buffers(struct vm_gk20a *vm,
			 struct nvgpu_mapped_buf ***mapped_buffers,
			 int *num_buffers)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct nvgpu_mapped_buf **buffer_list;
	struct nvgpu_rbtree_node *node = NULL;
	int i = 0;

	if (vm->userspace_managed) {
		*mapped_buffers = NULL;
		*num_buffers = 0;
		return 0;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	if (vm->num_user_mapped_buffers == 0) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		return 0;
	}

	buffer_list = nvgpu_big_zalloc(vm->mm->g, sizeof(*buffer_list) *
				       vm->num_user_mapped_buffers);
	if (buffer_list == NULL) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		return -ENOMEM;
	}

	nvgpu_rbtree_enum_start(0, &node, vm->mapped_buffers);
	while (node) {
		mapped_buffer = mapped_buffer_from_rbtree_node(node);
		buffer_list[i] = mapped_buffer;
		nvgpu_ref_get(&mapped_buffer->ref);
		i++;
		nvgpu_rbtree_enum_next(&node, node);
	}

	BUG_ON(i != vm->num_user_mapped_buffers);

	*num_buffers = vm->num_user_mapped_buffers;
	*mapped_buffers = buffer_list;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return 0;
}

void nvgpu_vm_put_buffers(struct vm_gk20a *vm,
				 struct nvgpu_mapped_buf **mapped_buffers,
				 int num_buffers)
{
	int i;
	struct vm_gk20a_mapping_batch batch;

	if (num_buffers == 0) {
		return;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	nvgpu_vm_mapping_batch_start(&batch);
	vm->kref_put_batch = &batch;

	for (i = 0; i < num_buffers; ++i) {
		nvgpu_ref_put(&mapped_buffers[i]->ref, __nvgpu_vm_unmap_ref);
	}

	vm->kref_put_batch = NULL;
	nvgpu_vm_mapping_batch_finish_locked(vm, &batch);
	nvgpu_mutex_release(&vm->update_gmmu_lock);

	nvgpu_big_free(vm->mm->g, mapped_buffers);
}

struct nvgpu_mapped_buf *nvgpu_vm_map(struct vm_gk20a *vm,
				      struct nvgpu_os_buffer *os_buf,
				      struct nvgpu_sgt *sgt,
				      u64 map_addr,
				      u64 map_size,
				      u64 phys_offset,
				      int rw,
				      u32 flags,
				      s16 compr_kind,
				      s16 incompr_kind,
				      struct vm_gk20a_mapping_batch *batch,
				      enum nvgpu_aperture aperture)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct nvgpu_mapped_buf *mapped_buffer = NULL;
	struct nvgpu_ctag_buffer_info binfo = { 0 };
	struct nvgpu_vm_area *vm_area = NULL;
	int err = 0;
	u64 align;
	u32 ctag_offset = 0;
	bool clear_ctags = false;
	bool va_allocated = true;

	/*
	 * The kind used as part of the key for map caching. HW may
	 * actually be programmed with the fallback kind in case the
	 * key kind is compressible but we're out of comptags.
	 */
	s16 map_key_kind;

	/*
	 * The actual GMMU PTE kind
	 */
	u8 pte_kind;

	if (vm->userspace_managed &&
	    (flags & NVGPU_VM_MAP_FIXED_OFFSET) == 0U) {
		nvgpu_err(g,
			  "non-fixed-offset mapping not available on "
			  "userspace managed address spaces");
		return ERR_PTR(-EINVAL);
	}

	binfo.flags = flags;
	binfo.size = nvgpu_os_buf_get_size(os_buf);
	binfo.compr_kind =
		(vm->enable_ctag && compr_kind != NVGPU_KIND_INVALID ?
		 compr_kind : NVGPU_KIND_INVALID);
	binfo.incompr_kind = incompr_kind;

	if (compr_kind != NVGPU_KIND_INVALID) {
		map_key_kind = compr_kind;
	} else {
		map_key_kind = incompr_kind;
	}

	/*
	 * Check if this buffer is already mapped.
	 */
	if (!vm->userspace_managed) {
		nvgpu_mutex_acquire(&vm->update_gmmu_lock);
		mapped_buffer = nvgpu_vm_find_mapping(vm,
						      os_buf,
						      map_addr,
						      flags,
						      map_key_kind);

		if (mapped_buffer) {
			nvgpu_ref_get(&mapped_buffer->ref);
			nvgpu_mutex_release(&vm->update_gmmu_lock);
			return mapped_buffer;
		}
		nvgpu_mutex_release(&vm->update_gmmu_lock);
	}

	/*
	 * Generate a new mapping!
	 */
	mapped_buffer = nvgpu_kzalloc(g, sizeof(*mapped_buffer));
	if (mapped_buffer == NULL) {
		nvgpu_warn(g, "oom allocating tracking buffer");
		return ERR_PTR(-ENOMEM);
	}

	align = nvgpu_sgt_alignment(g, sgt);
	if (g->mm.disable_bigpage) {
		binfo.pgsz_idx = GMMU_PAGE_SIZE_SMALL;
	} else {
		binfo.pgsz_idx = nvgpu_vm_get_pte_size(vm, map_addr,
						min_t(u64, binfo.size, align));
	}
	map_size = (map_size != 0ULL) ? map_size : binfo.size;
	map_size = ALIGN(map_size, SZ_4K);

	if ((map_size > binfo.size) ||
	    (phys_offset > (binfo.size - map_size))) {
		err = -EINVAL;
		goto clean_up_nolock;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	/*
	 * Check if we should use a fixed offset for mapping this buffer.
	 */
	if (flags & NVGPU_VM_MAP_FIXED_OFFSET)  {
		err = nvgpu_vm_area_validate_buffer(vm,
						    map_addr,
						    map_size,
						    binfo.pgsz_idx,
						    &vm_area);
		if (err) {
			goto clean_up;
		}

		va_allocated = false;
	}

	err = nvgpu_vm_compute_compression(vm, &binfo);
	if (err) {
		nvgpu_err(g, "failure setting up compression");
		goto clean_up;
	}

	if ((binfo.compr_kind != NVGPU_KIND_INVALID) &&
	    ((flags & NVGPU_VM_MAP_FIXED_OFFSET) != 0U)) {
		/*
		 * Fixed-address compressible mapping is
		 * requested. Make sure we're respecting the alignment
		 * requirement for virtual addresses and buffer
		 * offsets.
		 *
		 * This check must be done before we may fall back to
		 * the incompressible kind.
		 */

		const u64 offset_mask = g->ops.fb.compression_align_mask(g);

		if ((map_addr & offset_mask) != (phys_offset & offset_mask)) {
			nvgpu_log(g, gpu_dbg_map,
				  "Misaligned compressible-kind fixed-address "
				  "mapping");
			err = -EINVAL;
			goto clean_up;
		}
	}

	if (binfo.compr_kind != NVGPU_KIND_INVALID) {
		struct gk20a_comptags comptags = { 0 };

		/*
		 * Get the comptags state, alloc if necessary
		 */
		err = gk20a_alloc_or_get_comptags(g, os_buf,
						  &g->gr.comp_tags,
						  &comptags);
		if (err) {
			/*
			 * This is an irrecoverable failure and we need to
			 * abort. In particular, it is not safe to proceed with
			 * the incompressible fallback, since we cannot not mark
			 * our alloc failure anywere. Later we would retry
			 * allocation and break compressible map aliasing.
			 */
			nvgpu_err(g, "Error %d setting up comptags", err);
			goto clean_up;
		}

		/*
		 * Newly allocated comptags needs to be cleared
		 */
		if (comptags.needs_clear) {
			if (g->ops.ltc.cbc_ctrl) {
				if (gk20a_comptags_start_clear(os_buf)) {
					err = g->ops.ltc.cbc_ctrl(
						g, gk20a_cbc_op_clear,
						comptags.offset,
						(comptags.offset +
						 comptags.lines - 1U));
					gk20a_comptags_finish_clear(
						os_buf, err == 0);
					if (err) {
						goto clean_up;
					}
				}
			} else {
				/*
				 * Cleared as part of gmmu map
				 */
				clear_ctags = true;
			}
		}

		/*
		 * Store the ctag offset for later use if we got the comptags
		 */
		if (comptags.lines) {
			ctag_offset = comptags.offset;
		}
	}

	/*
	 * Figure out the kind and ctag offset for the GMMU page tables
	 */
	if (binfo.compr_kind != NVGPU_KIND_INVALID && ctag_offset != 0U) {
		/*
		 * Adjust the ctag_offset as per the buffer map offset
		 */
		ctag_offset += phys_offset >>
			ilog2(g->ops.fb.compression_page_size(g));
		pte_kind = binfo.compr_kind;
	} else if (binfo.incompr_kind != NVGPU_KIND_INVALID) {
		/*
		 * Incompressible kind, ctag offset will not be programmed
		 */
		ctag_offset = 0;
		pte_kind = binfo.incompr_kind;
	} else {
		/*
		 * Caller required compression, but we cannot provide it
		 */
		nvgpu_err(g, "No comptags and no incompressible fallback kind");
		err = -ENOMEM;
		goto clean_up;
	}

	if (clear_ctags) {
		clear_ctags = gk20a_comptags_start_clear(os_buf);
	}

	map_addr = g->ops.mm.gmmu_map(vm,
				      map_addr,
				      sgt,
				      phys_offset,
				      map_size,
				      binfo.pgsz_idx,
				      pte_kind,
				      ctag_offset,
				      flags,
				      rw,
				      clear_ctags,
				      false,
				      false,
				      batch,
				      aperture);

	if (clear_ctags) {
		gk20a_comptags_finish_clear(os_buf, map_addr != 0U);
	}

	if (map_addr == 0ULL) {
		err = -ENOMEM;
		goto clean_up;
	}

	nvgpu_init_list_node(&mapped_buffer->buffer_list);
	nvgpu_ref_init(&mapped_buffer->ref);
	mapped_buffer->addr         = map_addr;
	mapped_buffer->size         = map_size;
	mapped_buffer->pgsz_idx     = binfo.pgsz_idx;
	mapped_buffer->vm           = vm;
	mapped_buffer->flags        = flags;
	mapped_buffer->kind         = map_key_kind;
	mapped_buffer->va_allocated = va_allocated;
	mapped_buffer->vm_area      = vm_area;

	err = nvgpu_insert_mapped_buf(vm, mapped_buffer);
	if (err) {
		nvgpu_err(g, "failed to insert into mapped buffer tree");
		goto clean_up;
	}

	vm->num_user_mapped_buffers++;

	if (vm_area) {
		nvgpu_list_add_tail(&mapped_buffer->buffer_list,
			      &vm_area->buffer_list_head);
		mapped_buffer->vm_area = vm_area;
	}

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return mapped_buffer;

clean_up:
	if (mapped_buffer->addr) {
		g->ops.mm.gmmu_unmap(vm,
				     mapped_buffer->addr,
				     mapped_buffer->size,
				     mapped_buffer->pgsz_idx,
				     mapped_buffer->va_allocated,
				     gk20a_mem_flag_none,
				     (mapped_buffer->vm_area != NULL) ?
				     mapped_buffer->vm_area->sparse : false,
				     NULL);
	}
	nvgpu_mutex_release(&vm->update_gmmu_lock);
clean_up_nolock:
	nvgpu_kfree(g, mapped_buffer);

	return ERR_PTR(err);
}

/*
 * Really unmap. This does the real GMMU unmap and removes the mapping from the
 * VM map tracking tree (and vm_area list if necessary).
 */
static void __nvgpu_vm_unmap(struct nvgpu_mapped_buf *mapped_buffer,
			     struct vm_gk20a_mapping_batch *batch)
{
	struct vm_gk20a *vm = mapped_buffer->vm;
	struct gk20a *g = vm->mm->g;

	vm->num_user_mapped_buffers--;

	g->ops.mm.gmmu_unmap(vm,
			     mapped_buffer->addr,
			     mapped_buffer->size,
			     mapped_buffer->pgsz_idx,
			     mapped_buffer->va_allocated,
			     gk20a_mem_flag_none,
			     (mapped_buffer->vm_area != NULL) ?
			     mapped_buffer->vm_area->sparse : false,
			     batch);

	/*
	 * Remove from mapped buffer tree. Then delete the buffer from the
	 * linked list of mapped buffers; though note: not all mapped buffers
	 * are part of a vm_area.
	 */
	nvgpu_remove_mapped_buf(vm, mapped_buffer);
	nvgpu_list_del(&mapped_buffer->buffer_list);

	/*
	 * OS specific freeing. This is after the generic freeing incase the
	 * generic freeing relies on some component of the OS specific
	 * nvgpu_mapped_buf in some abstraction or the like.
	 */
	nvgpu_vm_unmap_system(mapped_buffer);

	nvgpu_kfree(g, mapped_buffer);
}

/*
 * Note: the update_gmmu_lock of the VM that owns this buffer must be locked
 * before calling nvgpu_ref_put() with this function as the unref function
 * argument since this can modify the tree of maps.
 */
void __nvgpu_vm_unmap_ref(struct nvgpu_ref *ref)
{
	struct nvgpu_mapped_buf *mapped_buffer =
		container_of(ref, struct nvgpu_mapped_buf, ref);

	__nvgpu_vm_unmap(mapped_buffer, mapped_buffer->vm->kref_put_batch);
}

/*
 * For fixed-offset buffers we must sync the buffer. That means we wait for the
 * buffer to hit a ref-count of 1 before proceeding.
 *
 * Note: this requires the update_gmmu_lock to be held since we release it and
 * re-aquire it in this function.
 */
static int nvgpu_vm_unmap_sync_buffer(struct vm_gk20a *vm,
				      struct nvgpu_mapped_buf *mapped_buffer)
{
	struct nvgpu_timeout timeout;
	int ret = 0;
	bool done = false;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	/*
	 * 100ms timer.
	 */
	nvgpu_timeout_init(vm->mm->g, &timeout, 100, NVGPU_TIMER_CPU_TIMER);

	do {
		if (nvgpu_atomic_read(&mapped_buffer->ref.refcount) <= 1) {
			done = true;
		} else if (nvgpu_timeout_expired_msg(&timeout,
			    "sync-unmap failed on 0x%llx",
			    mapped_buffer->addr) != 0) {
			done = true;
		} else {
			nvgpu_msleep(10);
		}
	} while (!done);

	if (nvgpu_atomic_read(&mapped_buffer->ref.refcount) > 1) {
		ret = -ETIMEDOUT;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	return ret;
}

void nvgpu_vm_unmap(struct vm_gk20a *vm, u64 offset,
		    struct vm_gk20a_mapping_batch *batch)
{
	struct nvgpu_mapped_buf *mapped_buffer;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	mapped_buffer = __nvgpu_vm_find_mapped_buf(vm, offset);
	if (mapped_buffer == NULL) {
		goto done;
	}

	if (mapped_buffer->flags & NVGPU_VM_MAP_FIXED_OFFSET) {
		if (nvgpu_vm_unmap_sync_buffer(vm, mapped_buffer)) {
			nvgpu_warn(vm->mm->g, "%d references remaining on 0x%llx",
				nvgpu_atomic_read(&mapped_buffer->ref.refcount),
				mapped_buffer->addr);
		}
	}

	/*
	 * Make sure we have access to the batch if we end up calling through to
	 * the unmap_ref function.
	 */
	vm->kref_put_batch = batch;
	nvgpu_ref_put(&mapped_buffer->ref, __nvgpu_vm_unmap_ref);
	vm->kref_put_batch = NULL;

done:
	nvgpu_mutex_release(&vm->update_gmmu_lock);
	return;
}

static int nvgpu_vm_compute_compression(struct vm_gk20a *vm,
					struct nvgpu_ctag_buffer_info *binfo)
{
	bool kind_compressible = (binfo->compr_kind != NVGPU_KIND_INVALID);
	struct gk20a *g = gk20a_from_vm(vm);

	if (kind_compressible &&
	    vm->gmmu_page_sizes[binfo->pgsz_idx] <
	    g->ops.fb.compressible_page_size(g)) {
		/*
		 * Let's double check that there is a fallback kind
		 */
		if (binfo->incompr_kind == NVGPU_KIND_INVALID) {
			nvgpu_err(g,
				  "Unsupported page size for compressible "
				  "kind, but no fallback kind");
			return -EINVAL;
		} else {
			nvgpu_log(g, gpu_dbg_map,
				  "Unsupported page size for compressible "
				  "kind, demoting to incompressible");
			binfo->compr_kind = NVGPU_KIND_INVALID;
			kind_compressible = false;
		}
	}

	return 0;
}
