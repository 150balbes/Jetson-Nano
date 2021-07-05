/*
 * Copyright (c) 2011-2020, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <uapi/linux/nvgpu.h>

#include <nvgpu/bitops.h>
#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/ptimer.h>
#include <nvgpu/vidmem.h>
#include <nvgpu/log.h>
#include <nvgpu/enabled.h>
#include <nvgpu/sizes.h>
#include <nvgpu/list.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>

#include "ioctl_ctrl.h"
#include "ioctl_dbg.h"
#include "ioctl_as.h"
#include "ioctl_tsg.h"
#include "ioctl_channel.h"
#include "gk20a/fence_gk20a.h"

#include "platform_gk20a.h"
#include "os_linux.h"
#include "dmabuf.h"
#include "channel.h"
#include "dmabuf_vidmem.h"

#define HZ_TO_MHZ(a) ((a > 0xF414F9CD7ULL) ? 0xffff : (a >> 32) ? \
	(u32) ((a * 0x10C8ULL) >> 32) : (u16) ((u32) a/MHZ))
#define MHZ_TO_HZ(a) ((u64)a * MHZ)

struct gk20a_ctrl_priv {
	struct device *dev;
	struct gk20a *g;
	struct nvgpu_clk_session *clk_session;

	struct nvgpu_list_node list;
	struct {
		struct vm_area_struct *vma;
		unsigned long flags;
		bool vma_mapped;
	} usermode_vma;
};

static inline struct gk20a_ctrl_priv *
gk20a_ctrl_priv_from_list(struct nvgpu_list_node *node)
{
	return (struct gk20a_ctrl_priv *)
		((uintptr_t)node - offsetof(struct gk20a_ctrl_priv, list));
}

static u32 gk20a_as_translate_as_alloc_flags(struct gk20a *g, u32 flags)
{
	u32 core_flags = 0;

	if (flags & NVGPU_GPU_IOCTL_ALLOC_AS_FLAGS_USERSPACE_MANAGED)
		core_flags |= NVGPU_AS_ALLOC_USERSPACE_MANAGED;

	return core_flags;
}

int gk20a_ctrl_dev_open(struct inode *inode, struct file *filp)
{
	struct nvgpu_os_linux *l;
	struct gk20a *g;
	struct gk20a_ctrl_priv *priv;
	int err = 0;

	l = container_of(inode->i_cdev,
			 struct nvgpu_os_linux, ctrl.cdev);
	g = gk20a_get(&l->g);
	if (!g)
		return -ENODEV;

	nvgpu_log_fn(g, " ");

	priv = nvgpu_kzalloc(g, sizeof(struct gk20a_ctrl_priv));
	if (!priv) {
		err = -ENOMEM;
		goto free_ref;
	}
	filp->private_data = priv;
	priv->dev = dev_from_gk20a(g);
	/*
	 * We dont close the arbiter fd's after driver teardown to support
	 * GPU_LOST events, so we store g here, instead of dereferencing the
	 * dev structure on teardown
	 */
	priv->g = g;

	if (!g->sw_ready) {
		err = gk20a_busy(g);
		if (err)
			goto free_ref;
		gk20a_idle(g);
	}

	err = nvgpu_clk_arb_init_session(g, &priv->clk_session);
free_ref:
	if (err != 0) {
		gk20a_put(g);
		if (priv)
			nvgpu_kfree(g, priv);
	} else {
		nvgpu_mutex_acquire(&l->ctrl.privs_lock);
		nvgpu_list_add(&priv->list, &l->ctrl.privs);
		nvgpu_mutex_release(&l->ctrl.privs_lock);
	}

	return err;
}
int gk20a_ctrl_dev_release(struct inode *inode, struct file *filp)
{
	struct gk20a_ctrl_priv *priv = filp->private_data;
	struct gk20a *g = priv->g;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&l->ctrl.privs_lock);
	nvgpu_list_del(&priv->list);
	nvgpu_mutex_release(&l->ctrl.privs_lock);

	if (priv->clk_session)
		nvgpu_clk_arb_release_session(g, priv->clk_session);

	gk20a_put(g);
	nvgpu_kfree(g, priv);

	return 0;
}

struct nvgpu_flags_mapping {
	u64 ioctl_flag;
	int enabled_flag;
};

static struct nvgpu_flags_mapping flags_mapping[] = {
	{NVGPU_GPU_FLAGS_CAN_RAILGATE,
		NVGPU_CAN_RAILGATE},
	{NVGPU_GPU_FLAGS_HAS_SYNCPOINTS,
		NVGPU_HAS_SYNCPOINTS},
	{NVGPU_GPU_FLAGS_SUPPORT_PARTIAL_MAPPINGS,
		NVGPU_SUPPORT_PARTIAL_MAPPINGS},
	{NVGPU_GPU_FLAGS_SUPPORT_SPARSE_ALLOCS,
		NVGPU_SUPPORT_SPARSE_ALLOCS},
	{NVGPU_GPU_FLAGS_SUPPORT_SYNC_FENCE_FDS,
		NVGPU_SUPPORT_SYNC_FENCE_FDS},
	{NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS,
		NVGPU_SUPPORT_CYCLE_STATS},
	{NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS_SNAPSHOT,
		NVGPU_SUPPORT_CYCLE_STATS_SNAPSHOT},
	{NVGPU_GPU_FLAGS_SUPPORT_USERSPACE_MANAGED_AS,
		NVGPU_SUPPORT_USERSPACE_MANAGED_AS},
	{NVGPU_GPU_FLAGS_SUPPORT_TSG,
		NVGPU_SUPPORT_TSG},
	{NVGPU_GPU_FLAGS_SUPPORT_CLOCK_CONTROLS,
		NVGPU_SUPPORT_CLOCK_CONTROLS},
	{NVGPU_GPU_FLAGS_SUPPORT_GET_VOLTAGE,
		NVGPU_SUPPORT_GET_VOLTAGE},
	{NVGPU_GPU_FLAGS_SUPPORT_GET_CURRENT,
		NVGPU_SUPPORT_GET_CURRENT},
	{NVGPU_GPU_FLAGS_SUPPORT_GET_POWER,
		NVGPU_SUPPORT_GET_POWER},
	{NVGPU_GPU_FLAGS_SUPPORT_GET_TEMPERATURE,
		NVGPU_SUPPORT_GET_TEMPERATURE},
	{NVGPU_GPU_FLAGS_SUPPORT_SET_THERM_ALERT_LIMIT,
		NVGPU_SUPPORT_SET_THERM_ALERT_LIMIT},
	{NVGPU_GPU_FLAGS_SUPPORT_DEVICE_EVENTS,
		NVGPU_SUPPORT_DEVICE_EVENTS},
	{NVGPU_GPU_FLAGS_SUPPORT_FECS_CTXSW_TRACE,
		NVGPU_SUPPORT_FECS_CTXSW_TRACE},
	{NVGPU_GPU_FLAGS_SUPPORT_DETERMINISTIC_SUBMIT_NO_JOBTRACKING,
		NVGPU_SUPPORT_DETERMINISTIC_SUBMIT_NO_JOBTRACKING},
	{NVGPU_GPU_FLAGS_SUPPORT_DETERMINISTIC_SUBMIT_FULL,
		NVGPU_SUPPORT_DETERMINISTIC_SUBMIT_FULL},
	{NVGPU_GPU_FLAGS_SUPPORT_DETERMINISTIC_OPTS,
		NVGPU_SUPPORT_DETERMINISTIC_OPTS},
	{NVGPU_GPU_FLAGS_SUPPORT_SYNCPOINT_ADDRESS,
		NVGPU_SUPPORT_SYNCPOINT_ADDRESS},
	{NVGPU_GPU_FLAGS_SUPPORT_USER_SYNCPOINT,
		NVGPU_SUPPORT_USER_SYNCPOINT},
	{NVGPU_GPU_FLAGS_SUPPORT_USERMODE_SUBMIT,
		NVGPU_SUPPORT_USERMODE_SUBMIT},
	{NVGPU_GPU_FLAGS_SUPPORT_IO_COHERENCE,
		NVGPU_SUPPORT_IO_COHERENCE},
	{NVGPU_GPU_FLAGS_SUPPORT_RESCHEDULE_RUNLIST,
		NVGPU_SUPPORT_RESCHEDULE_RUNLIST},
	{NVGPU_GPU_FLAGS_SUPPORT_MAP_DIRECT_KIND_CTRL,
		NVGPU_SUPPORT_MAP_DIRECT_KIND_CTRL},
	{NVGPU_GPU_FLAGS_ECC_ENABLED_SM_LRF,
		NVGPU_ECC_ENABLED_SM_LRF},
	{NVGPU_GPU_FLAGS_ECC_ENABLED_SM_SHM,
		NVGPU_ECC_ENABLED_SM_SHM},
	{NVGPU_GPU_FLAGS_ECC_ENABLED_TEX,
		NVGPU_ECC_ENABLED_TEX},
	{NVGPU_GPU_FLAGS_ECC_ENABLED_LTC,
		NVGPU_ECC_ENABLED_LTC},
	{NVGPU_GPU_FLAGS_SUPPORT_TSG_SUBCONTEXTS,
		NVGPU_SUPPORT_TSG_SUBCONTEXTS},
	{NVGPU_GPU_FLAGS_SUPPORT_SCG,
		NVGPU_SUPPORT_SCG},
	{NVGPU_GPU_FLAGS_SUPPORT_VPR,
		NVGPU_SUPPORT_VPR},
	{NVGPU_GPU_FLAGS_SUPPORT_SET_CTX_MMU_DEBUG_MODE,
		NVGPU_SUPPORT_SET_CTX_MMU_DEBUG_MODE},
};

static u64 nvgpu_ctrl_ioctl_gpu_characteristics_flags(struct gk20a *g)
{
	unsigned int i;
	u64 ioctl_flags = 0;

	for (i = 0; i < sizeof(flags_mapping)/sizeof(*flags_mapping); i++) {
		if (nvgpu_is_enabled(g, flags_mapping[i].enabled_flag))
			ioctl_flags |= flags_mapping[i].ioctl_flag;
	}

	if (!capable(CAP_SYS_NICE)) {
		ioctl_flags &= ~NVGPU_GPU_FLAGS_SUPPORT_RESCHEDULE_RUNLIST;
	}

	return ioctl_flags;
}

static void nvgpu_set_preemption_mode_flags(struct gk20a *g,
	struct nvgpu_gpu_characteristics *gpu)
{
	struct nvgpu_preemption_modes_rec preemption_mode_rec;

	g->ops.gr.get_preemption_mode_flags(g, &preemption_mode_rec);

	gpu->graphics_preemption_mode_flags =
		nvgpu_get_ioctl_graphics_preempt_mode_flags(
			preemption_mode_rec.graphics_preemption_mode_flags);
	gpu->compute_preemption_mode_flags =
		nvgpu_get_ioctl_compute_preempt_mode_flags(
			preemption_mode_rec.compute_preemption_mode_flags);

	gpu->default_graphics_preempt_mode =
		nvgpu_get_ioctl_graphics_preempt_mode(
			preemption_mode_rec.default_graphics_preempt_mode);
	gpu->default_compute_preempt_mode =
		nvgpu_get_ioctl_compute_preempt_mode(
			preemption_mode_rec.default_compute_preempt_mode);
}

static long
gk20a_ctrl_ioctl_gpu_characteristics(
	struct gk20a *g,
	struct nvgpu_gpu_get_characteristics *request)
{
	struct nvgpu_gpu_characteristics gpu;
	long err = 0;

	if (gk20a_busy(g)) {
		nvgpu_err(g, "failed to power on gpu");
		return -EINVAL;
	}

	memset(&gpu, 0, sizeof(gpu));

	gpu.L2_cache_size = g->ops.ltc.determine_L2_size_bytes(g);
	gpu.on_board_video_memory_size = 0; /* integrated GPU */

	gpu.num_gpc = g->gr.gpc_count;
	gpu.max_gpc_count = g->gr.max_gpc_count;

	gpu.num_tpc_per_gpc = g->gr.max_tpc_per_gpc_count;

	gpu.bus_type = NVGPU_GPU_BUS_TYPE_AXI; /* always AXI for now */

	gpu.compression_page_size = g->ops.fb.compression_page_size(g);

	if (g->ops.gr.get_gpc_mask) {
		gpu.gpc_mask = g->ops.gr.get_gpc_mask(g);
	} else {
		gpu.gpc_mask = BIT32(g->gr.gpc_count) - 1;
	}

	gpu.flags = nvgpu_ctrl_ioctl_gpu_characteristics_flags(g);

	gpu.arch = g->params.gpu_arch;
	gpu.impl = g->params.gpu_impl;
	gpu.rev = g->params.gpu_rev;
	gpu.reg_ops_limit = NVGPU_IOCTL_DBG_REG_OPS_LIMIT;
	gpu.map_buffer_batch_limit = nvgpu_is_enabled(g, NVGPU_SUPPORT_MAP_BUFFER_BATCH) ?
		NVGPU_IOCTL_AS_MAP_BUFFER_BATCH_LIMIT : 0;
	gpu.twod_class = g->ops.get_litter_value(g, GPU_LIT_TWOD_CLASS);
	gpu.threed_class = g->ops.get_litter_value(g, GPU_LIT_THREED_CLASS);
	gpu.compute_class = g->ops.get_litter_value(g, GPU_LIT_COMPUTE_CLASS);
	gpu.gpfifo_class = g->ops.get_litter_value(g, GPU_LIT_GPFIFO_CLASS);
	gpu.inline_to_memory_class =
		g->ops.get_litter_value(g, GPU_LIT_I2M_CLASS);
	gpu.dma_copy_class =
		g->ops.get_litter_value(g, GPU_LIT_DMA_COPY_CLASS);

	gpu.vbios_version = g->bios.vbios_version;
	gpu.vbios_oem_version = g->bios.vbios_oem_version;

	gpu.big_page_size = nvgpu_mm_get_default_big_page_size(g);
	gpu.pde_coverage_bit_count =
		g->ops.mm.get_mmu_levels(g, gpu.big_page_size)[0].lo_bit[0];
	gpu.available_big_page_sizes = nvgpu_mm_get_available_big_page_sizes(g);

	gpu.sm_arch_sm_version = g->params.sm_arch_sm_version;
	gpu.sm_arch_spa_version = g->params.sm_arch_spa_version;
	gpu.sm_arch_warp_count = g->params.sm_arch_warp_count;

	gpu.max_css_buffer_size = g->gr.max_css_buffer_size;

	gpu.gpu_ioctl_nr_last = NVGPU_GPU_IOCTL_LAST;
	gpu.tsg_ioctl_nr_last = NVGPU_TSG_IOCTL_LAST;
	gpu.dbg_gpu_ioctl_nr_last = NVGPU_DBG_GPU_IOCTL_LAST;
	gpu.ioctl_channel_nr_last = NVGPU_IOCTL_CHANNEL_LAST;
	gpu.as_ioctl_nr_last = NVGPU_AS_IOCTL_LAST;
	gpu.event_ioctl_nr_last = NVGPU_EVENT_IOCTL_LAST;
	gpu.gpu_va_bit_count = 40;

	strlcpy(gpu.chipname, g->name, sizeof(gpu.chipname));
	gpu.max_fbps_count = g->ops.gr.get_max_fbps_count(g);
	gpu.fbp_en_mask = g->ops.gr.get_fbp_en_mask(g);
	gpu.max_ltc_per_fbp =  g->ops.gr.get_max_ltc_per_fbp(g);
	gpu.max_lts_per_ltc = g->ops.gr.get_max_lts_per_ltc(g);
	gpu.gr_compbit_store_base_hw = g->gr.compbit_store.base_hw;
	gpu.gr_gobs_per_comptagline_per_slice =
		g->gr.gobs_per_comptagline_per_slice;
	gpu.num_ltc = g->ltc_count;
	gpu.lts_per_ltc = g->gr.slices_per_ltc;
	gpu.cbc_cache_line_size = g->gr.cacheline_size;
	gpu.cbc_comptags_per_line = g->gr.comptags_per_cacheline;

	if (g->ops.clk.get_maxrate)
		gpu.max_freq = g->ops.clk.get_maxrate(g, CTRL_CLK_DOMAIN_GPCCLK);

	gpu.local_video_memory_size = g->mm.vidmem.size;

	gpu.pci_vendor_id = g->pci_vendor_id;
	gpu.pci_device_id = g->pci_device_id;
	gpu.pci_subsystem_vendor_id = g->pci_subsystem_vendor_id;
	gpu.pci_subsystem_device_id = g->pci_subsystem_device_id;
	gpu.pci_class = g->pci_class;
	gpu.pci_revision = g->pci_revision;

	nvgpu_set_preemption_mode_flags(g, &gpu);

	if (request->gpu_characteristics_buf_size > 0) {
		size_t write_size = sizeof(gpu);

		nvgpu_speculation_barrier();
		if (write_size > request->gpu_characteristics_buf_size)
			write_size = request->gpu_characteristics_buf_size;

		err = copy_to_user((void __user *)(uintptr_t)
				   request->gpu_characteristics_buf_addr,
				   &gpu, write_size);
	}

	if (err == 0)
		request->gpu_characteristics_buf_size = sizeof(gpu);

	gk20a_idle(g);

	return err;
}

static int gk20a_ctrl_prepare_compressible_read(
		struct gk20a *g,
		struct nvgpu_gpu_prepare_compressible_read_args *args)
{
	int ret = -ENOSYS;

#ifdef CONFIG_NVGPU_SUPPORT_CDE
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct nvgpu_channel_fence fence;
	struct gk20a_fence *fence_out = NULL;
	int submit_flags = nvgpu_submit_gpfifo_user_flags_to_common_flags(
		args->submit_flags);
	int fd = -1;

	fence.id = args->fence.syncpt_id;
	fence.value = args->fence.syncpt_value;

	/* Try and allocate an fd here*/
	if ((submit_flags & NVGPU_SUBMIT_FLAGS_FENCE_GET)
		&& (submit_flags & NVGPU_SUBMIT_FLAGS_SYNC_FENCE)) {
			fd = get_unused_fd_flags(O_RDWR);
			if (fd < 0)
				return fd;
	}

	ret = gk20a_prepare_compressible_read(l, args->handle,
			args->request_compbits, args->offset,
			args->compbits_hoffset, args->compbits_voffset,
			args->scatterbuffer_offset,
			args->width, args->height, args->block_height_log2,
			submit_flags, &fence, &args->valid_compbits,
			&args->zbc_color, &fence_out);

	if (ret) {
		if (fd != -1)
			put_unused_fd(fd);
		return ret;
	}

	/* Convert fence_out to something we can pass back to user space. */
	if (submit_flags & NVGPU_SUBMIT_FLAGS_FENCE_GET) {
		if (submit_flags & NVGPU_SUBMIT_FLAGS_SYNC_FENCE) {
			if (fence_out) {
				ret = gk20a_fence_install_fd(fence_out, fd);
				if (ret)
					put_unused_fd(fd);
				else
					args->fence.fd = fd;
			} else {
				args->fence.fd = -1;
				put_unused_fd(fd);
			}
		} else {
			if (fence_out) {
				args->fence.syncpt_id = fence_out->syncpt_id;
				args->fence.syncpt_value =
						fence_out->syncpt_value;
			} else {
				args->fence.syncpt_id = -1;
				args->fence.syncpt_value = 0;
			}
		}
	}
	gk20a_fence_put(fence_out);
#endif

	return ret;
}

static int gk20a_ctrl_mark_compressible_write(
		struct gk20a *g,
		struct nvgpu_gpu_mark_compressible_write_args *args)
{
	int ret = -ENOSYS;

#ifdef CONFIG_NVGPU_SUPPORT_CDE
	ret = gk20a_mark_compressible_write(g, args->handle,
			args->valid_compbits, args->offset, args->zbc_color);
#endif

	return ret;
}

static int gk20a_ctrl_alloc_as(
		struct gk20a *g,
		struct nvgpu_alloc_as_args *args)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct gk20a_as_share *as_share;
	int err;
	int fd;
	struct file *file;
	char name[64];

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		return err;
	fd = err;

	snprintf(name, sizeof(name), "nvhost-%s-fd%d", g->name, fd);

	file = anon_inode_getfile(name, l->as_dev.cdev.ops, NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	err = gk20a_as_alloc_share(g, args->big_page_size,
				   gk20a_as_translate_as_alloc_flags(g,
					   args->flags),
				   &as_share);
	if (err)
		goto clean_up_file;

	fd_install(fd, file);
	file->private_data = as_share;

	args->as_fd = fd;
	return 0;

clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(fd);
	return err;
}

static int gk20a_ctrl_open_tsg(struct gk20a *g,
			       struct nvgpu_gpu_open_tsg_args *args)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	int err;
	int fd;
	struct file *file;
	char name[64];

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		return err;
	fd = err;

	snprintf(name, sizeof(name), "nvgpu-%s-tsg%d", g->name, fd);

	file = anon_inode_getfile(name, l->tsg.cdev.ops, NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	err = nvgpu_ioctl_tsg_open(g, file);
	if (err)
		goto clean_up_file;

	fd_install(fd, file);
	args->tsg_fd = fd;
	return 0;

clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(fd);
	return err;
}

static int gk20a_ctrl_get_tpc_masks(struct gk20a *g,
				    struct nvgpu_gpu_get_tpc_masks_args *args)
{
	struct gr_gk20a *gr = &g->gr;
	int err = 0;
	const u32 gpc_tpc_mask_size = sizeof(u32) * gr->max_gpc_count;

	if (args->mask_buf_size > 0) {
		size_t write_size = gpc_tpc_mask_size;

		nvgpu_speculation_barrier();
		if (write_size > args->mask_buf_size)
			write_size = args->mask_buf_size;

		err = copy_to_user((void __user *)(uintptr_t)
				   args->mask_buf_addr,
				   gr->gpc_tpc_mask, write_size);
	}

	if (err == 0)
		args->mask_buf_size = gpc_tpc_mask_size;

	return err;
}

static int gk20a_ctrl_get_fbp_l2_masks(
	struct gk20a *g, struct nvgpu_gpu_get_fbp_l2_masks_args *args)
{
	struct gr_gk20a *gr = &g->gr;
	int err = 0;
	const u32 fbp_l2_mask_size = sizeof(u32) * gr->max_fbps_count;

	if (args->mask_buf_size > 0) {
		size_t write_size = fbp_l2_mask_size;

		nvgpu_speculation_barrier();
		if (write_size > args->mask_buf_size)
			write_size = args->mask_buf_size;

		err = copy_to_user((void __user *)(uintptr_t)
				   args->mask_buf_addr,
				   gr->fbp_rop_l2_en_mask, write_size);
	}

	if (err == 0)
		args->mask_buf_size = fbp_l2_mask_size;

	return err;
}

static int nvgpu_gpu_ioctl_l2_fb_ops(struct gk20a *g,
		struct nvgpu_gpu_l2_fb_args *args)
{
	int ret;
	bool always_poweron;

	if ((!args->l2_flush && !args->fb_flush) ||
	    (!args->l2_flush && args->l2_invalidate))
		return -EINVAL;

	/* Handle this case for joint rails or DGPU */
	always_poweron = (!nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE) ||
				!pm_runtime_enabled(dev_from_gk20a(g)));

	/* In case of not always power_on, exit if g->power_on is false */
	if (!always_poweron && !gk20a_check_poweron(g)) {
		return 0;
	}

	/* There is a small window between a call to gk20a_idle() has occured
	 * and railgate being actually triggered(setting g->power_on = false),
	 * when l2_flush can race with railgate. Its better to take a busy_lock
	 * to prevent the gk20a_idle() from proceeding. There is a very small
	 * chance that gk20a_idle() might begin before gk20a_busy(). Having
	 * a locked access to g->power_on further reduces the probability of
	 * gk20a_idle() being triggered before gk20a_busy()
	 */
	ret = gk20a_busy(g);

	if (ret != 0) {
		nvgpu_err(g, "failed to take power ref");
		return ret;
	}

	if (args->l2_flush)
		g->ops.mm.l2_flush(g, args->l2_invalidate ? true : false);

	if (args->fb_flush)
		g->ops.mm.fb_flush(g);

	gk20a_idle(g);

	return 0;
}

static int nvgpu_gpu_ioctl_set_mmu_debug_mode(
		struct gk20a *g,
		struct nvgpu_gpu_mmu_debug_mode_args *args)
{
	if (gk20a_busy(g)) {
		nvgpu_err(g, "failed to power on gpu");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	g->ops.fb.set_debug_mode(g, args->state == 1);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);
	return 0;
}

static int nvgpu_gpu_ioctl_set_debug_mode(
		struct gk20a *g,
		struct nvgpu_gpu_sm_debug_mode_args *args)
{
	struct channel_gk20a *ch;
	int err;

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch)
		return -EINVAL;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	if (g->ops.gr.set_sm_debug_mode)
		err = g->ops.gr.set_sm_debug_mode(g, ch,
				args->sms, !!args->enable);
	else
		err = -ENOSYS;
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_channel_put(ch);
	return err;
}

static int nvgpu_gpu_ioctl_trigger_suspend(struct gk20a *g)
{
	int err;

	err = gk20a_busy(g);
	if (err)
	    return err;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	err = gr_gk20a_elpg_protected_call(g,
			g->ops.gr.trigger_suspend(g));
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_ioctl_wait_for_pause(struct gk20a *g,
		struct nvgpu_gpu_wait_pause_args *args)
{
	int err;
	struct warpstate *ioctl_w_state;
	struct nvgpu_warpstate *w_state = NULL;
	u32 sm_count, ioctl_size, size, sm_id;

	sm_count = g->gr.gpc_count * g->gr.tpc_count;

	ioctl_size = sm_count * sizeof(struct warpstate);
	ioctl_w_state = nvgpu_kzalloc(g, ioctl_size);
	if (!ioctl_w_state)
		return -ENOMEM;

	size = sm_count * sizeof(struct nvgpu_warpstate);
	w_state = nvgpu_kzalloc(g, size);
	if (!w_state) {
		err = -ENOMEM;
		goto out_free;
	}

	err = gk20a_busy(g);
	if (err)
		goto out_free;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	(void)gr_gk20a_elpg_protected_call(g,
			g->ops.gr.wait_for_pause(g, w_state));

	for (sm_id = 0; sm_id < g->gr.no_of_sm; sm_id++) {
		ioctl_w_state[sm_id].valid_warps[0] =
			w_state[sm_id].valid_warps[0];
		ioctl_w_state[sm_id].valid_warps[1] =
			w_state[sm_id].valid_warps[1];
		ioctl_w_state[sm_id].trapped_warps[0] =
			w_state[sm_id].trapped_warps[0];
		ioctl_w_state[sm_id].trapped_warps[1] =
			w_state[sm_id].trapped_warps[1];
		ioctl_w_state[sm_id].paused_warps[0] =
			w_state[sm_id].paused_warps[0];
		ioctl_w_state[sm_id].paused_warps[1] =
			w_state[sm_id].paused_warps[1];
	}
	/* Copy to user space - pointed by "args->pwarpstate" */
	if (copy_to_user((void __user *)(uintptr_t)args->pwarpstate,
	    w_state, ioctl_size)) {
		nvgpu_log_fn(g, "copy_to_user failed!");
		err = -EFAULT;
	}

	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);

out_free:
	nvgpu_kfree(g, w_state);
	nvgpu_kfree(g, ioctl_w_state);

	return err;
}

static int nvgpu_gpu_ioctl_resume_from_pause(struct gk20a *g)
{
	int err;

	err = gk20a_busy(g);
	if (err)
	    return err;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	err = gr_gk20a_elpg_protected_call(g,
			g->ops.gr.resume_from_pause(g));
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_ioctl_clear_sm_errors(struct gk20a *g)
{
	int err;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = gr_gk20a_elpg_protected_call(g,
			g->ops.gr.clear_sm_errors(g));

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_ioctl_has_any_exception(
		struct gk20a *g,
		struct nvgpu_gpu_tpc_exception_en_status_args *args)
{
	u32 tpc_exception_en;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	tpc_exception_en = g->ops.gr.tpc_enabled_exceptions(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	args->tpc_exception_en_sm_mask = tpc_exception_en;

	return 0;
}

static int gk20a_ctrl_get_num_vsms(struct gk20a *g,
				    struct nvgpu_gpu_num_vsms *args)
{
	struct gr_gk20a *gr = &g->gr;
	args->num_vsms = gr->no_of_sm;
	return 0;
}

static int gk20a_ctrl_vsm_mapping(struct gk20a *g,
				    struct nvgpu_gpu_vsms_mapping *args)
{
	int err = 0;
	struct gr_gk20a *gr = &g->gr;
	size_t write_size = gr->no_of_sm *
				sizeof(struct nvgpu_gpu_vsms_mapping_entry);
	struct nvgpu_gpu_vsms_mapping_entry *vsms_buf;
	u32 i;

	vsms_buf = nvgpu_kzalloc(g, write_size);
	if (vsms_buf == NULL)
		return -ENOMEM;

	for (i = 0; i < gr->no_of_sm; i++) {
		vsms_buf[i].gpc_index = gr->sm_to_cluster[i].gpc_index;
		if (g->ops.gr.get_nonpes_aware_tpc)
			vsms_buf[i].tpc_index =
				g->ops.gr.get_nonpes_aware_tpc(g,
					gr->sm_to_cluster[i].gpc_index,
					gr->sm_to_cluster[i].tpc_index);
		else
			vsms_buf[i].tpc_index =
				gr->sm_to_cluster[i].tpc_index;
	}

	err = copy_to_user((void __user *)(uintptr_t)
			   args->vsms_map_buf_addr,
			   vsms_buf, write_size);
	nvgpu_kfree(g, vsms_buf);

	return err;
}

static int nvgpu_gpu_get_cpu_time_correlation_info(
	struct gk20a *g,
	struct nvgpu_gpu_get_cpu_time_correlation_info_args *args)
{
	struct nvgpu_cpu_time_correlation_sample *samples;
	int err;
	u32 i;

	if (args->count > NVGPU_GPU_GET_CPU_TIME_CORRELATION_INFO_MAX_COUNT ||
	    args->source_id != NVGPU_GPU_GET_CPU_TIME_CORRELATION_INFO_SRC_ID_TSC)
		return -EINVAL;

	samples = nvgpu_kzalloc(g, args->count *
		sizeof(struct nvgpu_cpu_time_correlation_sample));
	if (!samples) {
		return -ENOMEM;
	}

	err = g->ops.ptimer.get_timestamps_zipper(g,
			args->source_id, args->count, samples);
	if (!err) {
		for (i = 0; i < args->count; i++) {
			args->samples[i].cpu_timestamp = samples[i].cpu_timestamp;
			args->samples[i].gpu_timestamp = samples[i].gpu_timestamp;
		}
	}

	nvgpu_kfree(g, samples);

	return err;
}

static int nvgpu_gpu_get_gpu_time(
	struct gk20a *g,
	struct nvgpu_gpu_get_gpu_time_args *args)
{
	u64 time;
	int err;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = g->ops.ptimer.read_ptimer(g, &time);
	if (!err)
		args->gpu_timestamp = time;

	gk20a_idle(g);
	return err;
}

static int nvgpu_gpu_get_engine_info(
	struct gk20a *g,
	struct nvgpu_gpu_get_engine_info_args *args)
{
	int err = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;
	u32 report_index = 0;
	u32 engine_id_idx;
	const u32 max_buffer_engines = args->engine_info_buf_size /
		sizeof(struct nvgpu_gpu_get_engine_info_item);
	struct nvgpu_gpu_get_engine_info_item __user *dst_item_list =
		(void __user *)(uintptr_t)args->engine_info_buf_addr;

	for (engine_id_idx = 0; engine_id_idx < g->fifo.num_engines;
		++engine_id_idx) {
		u32 active_engine_id = g->fifo.active_engines_list[engine_id_idx];
		const struct fifo_engine_info_gk20a *src_info =
			&g->fifo.engine_info[active_engine_id];
		struct nvgpu_gpu_get_engine_info_item dst_info;

		memset(&dst_info, 0, sizeof(dst_info));

		engine_enum = src_info->engine_enum;

		switch (engine_enum) {
		case ENGINE_GR_GK20A:
			dst_info.engine_id = NVGPU_GPU_ENGINE_ID_GR;
			break;

		case ENGINE_GRCE_GK20A:
			dst_info.engine_id = NVGPU_GPU_ENGINE_ID_GR_COPY;
			break;

		case ENGINE_ASYNC_CE_GK20A:
			dst_info.engine_id = NVGPU_GPU_ENGINE_ID_ASYNC_COPY;
			break;

		default:
			nvgpu_err(g, "Unmapped engine enum %u",
				  engine_enum);
			continue;
		}

		dst_info.engine_instance = src_info->inst_id;
		dst_info.runlist_id = src_info->runlist_id;

		if (report_index < max_buffer_engines) {
			err = copy_to_user(&dst_item_list[report_index],
					   &dst_info, sizeof(dst_info));
			if (err)
				goto clean_up;
		}

		++report_index;
	}

	args->engine_info_buf_size =
		report_index * sizeof(struct nvgpu_gpu_get_engine_info_item);

clean_up:
	return err;
}

static int nvgpu_gpu_alloc_vidmem(struct gk20a *g,
			struct nvgpu_gpu_alloc_vidmem_args *args)
{
	u32 align = args->in.alignment ? args->in.alignment : SZ_4K;
	int fd;

	nvgpu_log_fn(g, " ");

	/* not yet supported */
	if (WARN_ON(args->in.flags & NVGPU_GPU_ALLOC_VIDMEM_FLAG_CPU_MASK))
		return -EINVAL;

	/* not yet supported */
	if (WARN_ON(args->in.flags & NVGPU_GPU_ALLOC_VIDMEM_FLAG_VPR))
		return -EINVAL;

	if (args->in.size & (SZ_4K - 1))
		return -EINVAL;

	if (!args->in.size)
		return -EINVAL;

	if (align & (align - 1))
		return -EINVAL;

	if (align > roundup_pow_of_two(args->in.size)) {
		/* log this special case, buddy allocator detail */
		nvgpu_warn(g,
			"alignment larger than buffer size rounded up to power of 2 is not supported");
		return -EINVAL;
	}

	fd = nvgpu_vidmem_export_linux(g, args->in.size);
	if (fd < 0)
		return fd;

	args->out.dmabuf_fd = fd;

	nvgpu_log_fn(g, "done, fd=%d", fd);

	return 0;
}

static int nvgpu_gpu_get_memory_state(struct gk20a *g,
			struct nvgpu_gpu_get_memory_state_args *args)
{
	int err;

	nvgpu_log_fn(g, " ");

	if (args->reserved[0] || args->reserved[1] ||
	    args->reserved[2] || args->reserved[3])
		return -EINVAL;

	err = nvgpu_vidmem_get_space(g, &args->total_free_bytes);

	nvgpu_log_fn(g, "done, err=%d, bytes=%lld", err, args->total_free_bytes);

	return err;
}

static u32 nvgpu_gpu_convert_clk_domain(u32 clk_domain)
{
	u32 domain = 0;

	if (clk_domain == NVGPU_GPU_CLK_DOMAIN_MCLK)
		domain = NVGPU_CLK_DOMAIN_MCLK;
	else if (clk_domain == NVGPU_GPU_CLK_DOMAIN_GPCCLK)
		domain = NVGPU_CLK_DOMAIN_GPCCLK;
	else
		domain = NVGPU_CLK_DOMAIN_MAX + 1;

	return domain;
}

static int nvgpu_gpu_clk_get_vf_points(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_vf_points_args *args)
{
	struct nvgpu_gpu_clk_vf_point clk_point;
	struct nvgpu_gpu_clk_vf_point __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;
	u32 clk_domains = 0;
	int err;
	u16 last_mhz;
	u16 *fpoints;
	u32 i;
	u32 max_points = 0;
	u32 num_points = 0;
	u16 min_mhz;
	u16 max_mhz;

	nvgpu_log_fn(g, " ");

	if (!session || args->flags)
		return -EINVAL;

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	args->num_entries = 0;

	if (!nvgpu_clk_arb_is_valid_domain(g,
				nvgpu_gpu_convert_clk_domain(args->clk_domain)))
		return -EINVAL;

	err = nvgpu_clk_arb_get_arbiter_clk_f_points(g,
			nvgpu_gpu_convert_clk_domain(args->clk_domain),
			&max_points, NULL);
	if (err)
		return err;

	if (!args->max_entries) {
		args->max_entries = max_points;
		return 0;
	}

	if (args->max_entries < max_points)
		return -EINVAL;

	err = nvgpu_clk_arb_get_arbiter_clk_range(g,
			nvgpu_gpu_convert_clk_domain(args->clk_domain),
			&min_mhz, &max_mhz);
	if (err)
		return err;

	fpoints = nvgpu_kcalloc(g, max_points, sizeof(u16));
	if (!fpoints)
		return -ENOMEM;

	err = nvgpu_clk_arb_get_arbiter_clk_f_points(g,
			nvgpu_gpu_convert_clk_domain(args->clk_domain),
			&max_points, fpoints);
	if (err)
		goto fail;

	entry = (struct nvgpu_gpu_clk_vf_point __user *)
			(uintptr_t)args->clk_vf_point_entries;

	last_mhz = 0;
	num_points = 0;
	for (i = 0; (i < max_points) && !err; i++) {

		/* filter out duplicate frequencies */
		if (fpoints[i] == last_mhz)
			continue;

		/* filter out out-of-range frequencies */
		if ((fpoints[i] < min_mhz) || (fpoints[i] > max_mhz))
			continue;

		last_mhz = fpoints[i];
		clk_point.freq_hz = MHZ_TO_HZ(fpoints[i]);

		err = copy_to_user((void __user *)entry, &clk_point,
				sizeof(clk_point));

		num_points++;
		entry++;
	}

	args->num_entries = num_points;

fail:
	nvgpu_kfree(g, fpoints);
	return err;
}

static int nvgpu_gpu_clk_get_range(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_range_args *args)
{
	struct nvgpu_gpu_clk_range clk_range;
	struct nvgpu_gpu_clk_range __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;

	u32 clk_domains = 0;
	u32 num_domains;
	u32 num_entries;
	u32 i;
	int bit;
	int err;
	u16 min_mhz, max_mhz;

	nvgpu_log_fn(g, " ");

	if (!session)
		return -EINVAL;

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	num_domains = hweight_long(clk_domains);

	if (!args->flags) {
		if (!args->num_entries) {
			args->num_entries = num_domains;
			return 0;
		}

		if (args->num_entries < num_domains)
			return -EINVAL;

		args->num_entries = 0;
		num_entries = num_domains;

	} else {
		if (args->flags != NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS)
			return -EINVAL;

		num_entries = args->num_entries;
		if (num_entries > num_domains)
			return -EINVAL;
	}

	entry = (struct nvgpu_gpu_clk_range __user *)
			(uintptr_t)args->clk_range_entries;

	for (i = 0; i < num_entries; i++, entry++) {

		if (args->flags == NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS) {
			if (copy_from_user(&clk_range, (void __user *)entry,
					sizeof(clk_range)))
				return -EFAULT;
		} else {
			bit = ffs(clk_domains) - 1;
			clk_range.clk_domain = bit;
			clk_domains &= ~BIT(bit);
		}

		clk_range.flags = 0;
		err = nvgpu_clk_arb_get_arbiter_clk_range(g,
				nvgpu_gpu_convert_clk_domain(clk_range.clk_domain),
				&min_mhz, &max_mhz);
		clk_range.min_hz = MHZ_TO_HZ(min_mhz);
		clk_range.max_hz = MHZ_TO_HZ(max_mhz);

		if (err)
			return err;

		err = copy_to_user(entry, &clk_range, sizeof(clk_range));
		if (err)
			return -EFAULT;
	}

	args->num_entries = num_entries;

	return 0;
}

static int nvgpu_gpu_clk_set_info(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_set_info_args *args)
{
	struct nvgpu_gpu_clk_info clk_info;
	struct nvgpu_gpu_clk_info __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;

	int fd;
	u32 clk_domains = 0;
	u16 freq_mhz;
	int i;
	int ret;

	nvgpu_log_fn(g, " ");

	if (!session || args->flags)
		return -EINVAL;

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	if (!clk_domains)
		return -EINVAL;

	entry = (struct nvgpu_gpu_clk_info __user *)
			(uintptr_t)args->clk_info_entries;

	for (i = 0; i < args->num_entries; i++, entry++) {

		if (copy_from_user(&clk_info, entry, sizeof(clk_info)))
			return -EFAULT;

		if (!nvgpu_clk_arb_is_valid_domain(g,
					nvgpu_gpu_convert_clk_domain(clk_info.clk_domain)))
			return -EINVAL;
	}
	nvgpu_speculation_barrier();

	entry = (struct nvgpu_gpu_clk_info __user *)
			(uintptr_t)args->clk_info_entries;

	ret = nvgpu_clk_arb_install_request_fd(g, session, &fd);
	if (ret < 0)
		return ret;

	for (i = 0; i < args->num_entries; i++, entry++) {

		if (copy_from_user(&clk_info, (void __user *)entry,
				sizeof(clk_info)))
			return -EFAULT;
		freq_mhz = HZ_TO_MHZ(clk_info.freq_hz);

		nvgpu_clk_arb_set_session_target_mhz(session, fd,
				nvgpu_gpu_convert_clk_domain(clk_info.clk_domain), freq_mhz);
	}

	nvgpu_speculation_barrier();
	ret = nvgpu_clk_arb_commit_request_fd(g, session, fd);
	if (ret < 0)
		return ret;

	args->completion_fd = fd;

	return ret;
}

static int nvgpu_gpu_clk_get_info(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		struct nvgpu_gpu_clk_get_info_args *args)
{
	struct nvgpu_gpu_clk_info clk_info;
	struct nvgpu_gpu_clk_info __user *entry;
	struct nvgpu_clk_session *session = priv->clk_session;
	u32 clk_domains = 0;
	u32 num_domains;
	u32 num_entries;
	u32 i;
	u16 freq_mhz;
	int err;
	int bit;

	nvgpu_log_fn(g, " ");

	if (!session)
		return -EINVAL;

	clk_domains = nvgpu_clk_arb_get_arbiter_clk_domains(g);
	num_domains = hweight_long(clk_domains);

	if (!args->flags) {
		if (!args->num_entries) {
			args->num_entries = num_domains;
			return 0;
		}

		if (args->num_entries < num_domains)
			return -EINVAL;

		args->num_entries = 0;
		num_entries = num_domains;

	} else {
		if (args->flags != NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS)
			return -EINVAL;

		num_entries = args->num_entries;
		if (num_entries > num_domains * 3)
			return -EINVAL;
	}

	entry = (struct nvgpu_gpu_clk_info __user *)
			(uintptr_t)args->clk_info_entries;

	for (i = 0; i < num_entries; i++, entry++) {

		if (args->flags == NVGPU_GPU_CLK_FLAG_SPECIFIC_DOMAINS) {
			if (copy_from_user(&clk_info, (void __user *)entry,
					sizeof(clk_info)))
				return -EFAULT;
		} else {
			bit = ffs(clk_domains) - 1;
			clk_info.clk_domain = bit;
			clk_domains &= ~BIT(bit);
			clk_info.clk_type = args->clk_type;
		}

		nvgpu_speculation_barrier();
		switch (clk_info.clk_type) {
		case NVGPU_GPU_CLK_TYPE_TARGET:
			err = nvgpu_clk_arb_get_session_target_mhz(session,
					nvgpu_gpu_convert_clk_domain(clk_info.clk_domain),
					&freq_mhz);
			break;
		case NVGPU_GPU_CLK_TYPE_ACTUAL:
			err = nvgpu_clk_arb_get_arbiter_actual_mhz(g,
					nvgpu_gpu_convert_clk_domain(clk_info.clk_domain),
					&freq_mhz);
			break;
		case NVGPU_GPU_CLK_TYPE_EFFECTIVE:
			err = nvgpu_clk_arb_get_arbiter_effective_mhz(g,
					nvgpu_gpu_convert_clk_domain(clk_info.clk_domain),
					&freq_mhz);
			break;
		default:
			freq_mhz = 0;
			err = -EINVAL;
			break;
		}
		if (err)
			return err;

		clk_info.flags = 0;
		clk_info.freq_hz = MHZ_TO_HZ(freq_mhz);

		err = copy_to_user((void __user *)entry, &clk_info,
				sizeof(clk_info));
		if (err)
			return -EFAULT;
	}

	nvgpu_speculation_barrier();
	args->num_entries = num_entries;

	return 0;
}

static int nvgpu_gpu_get_event_fd(struct gk20a *g,
	struct gk20a_ctrl_priv *priv,
	struct nvgpu_gpu_get_event_fd_args *args)
{
	struct nvgpu_clk_session *session = priv->clk_session;

	nvgpu_log_fn(g, " ");

	if (!session)
		return -EINVAL;

	return nvgpu_clk_arb_install_event_fd(g, session, &args->event_fd,
		args->flags);
}

static int nvgpu_gpu_get_voltage(struct gk20a *g,
		struct nvgpu_gpu_get_voltage_args *args)
{
	int err = -EINVAL;

	nvgpu_log_fn(g, " ");

	if (args->reserved)
		return -EINVAL;

	if (!nvgpu_is_enabled(g, NVGPU_SUPPORT_GET_VOLTAGE))
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
	    return err;

	nvgpu_speculation_barrier();
	switch (args->which) {
	case NVGPU_GPU_VOLTAGE_CORE:
		err = volt_get_voltage(g, CTRL_VOLT_DOMAIN_LOGIC, &args->voltage);
		break;
	case NVGPU_GPU_VOLTAGE_SRAM:
		err = volt_get_voltage(g, CTRL_VOLT_DOMAIN_SRAM, &args->voltage);
		break;
	case NVGPU_GPU_VOLTAGE_BUS:
		err = pmgr_pwr_devices_get_voltage(g, &args->voltage);
		break;
	default:
		err = -EINVAL;
	}

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_get_current(struct gk20a *g,
		struct nvgpu_gpu_get_current_args *args)
{
	int err;

	nvgpu_log_fn(g, " ");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!nvgpu_is_enabled(g, NVGPU_SUPPORT_GET_CURRENT))
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = pmgr_pwr_devices_get_current(g, &args->currnt);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_get_power(struct gk20a *g,
		struct nvgpu_gpu_get_power_args *args)
{
	int err;

	nvgpu_log_fn(g, " ");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!nvgpu_is_enabled(g, NVGPU_SUPPORT_GET_POWER))
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = pmgr_pwr_devices_get_power(g, &args->power);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_get_temperature(struct gk20a *g,
		struct nvgpu_gpu_get_temperature_args *args)
{
	int err;
	u32 temp_f24_8;

	nvgpu_log_fn(g, " ");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!nvgpu_is_enabled(g, NVGPU_SUPPORT_GET_TEMPERATURE))
		return -EINVAL;

	if (!g->ops.therm.get_internal_sensor_curr_temp)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = g->ops.therm.get_internal_sensor_curr_temp(g, &temp_f24_8);

	gk20a_idle(g);

	args->temp_f24_8 = (s32)temp_f24_8;

	return err;
}

static int nvgpu_gpu_set_therm_alert_limit(struct gk20a *g,
		struct nvgpu_gpu_set_therm_alert_limit_args *args)
{
	int err;

	nvgpu_log_fn(g, " ");

	if (args->reserved[0] || args->reserved[1] || args->reserved[2])
		return -EINVAL;

	if (!g->ops.therm.configure_therm_alert)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = g->ops.therm.configure_therm_alert(g, args->temp_f24_8);

	gk20a_idle(g);

	return err;
}

static int nvgpu_gpu_set_deterministic_ch_railgate(struct channel_gk20a *ch,
		u32 flags)
{
	int err = 0;
	bool allow;
	bool disallow;

	allow = flags &
		NVGPU_GPU_SET_DETERMINISTIC_OPTS_FLAGS_ALLOW_RAILGATING;

	disallow = flags &
		NVGPU_GPU_SET_DETERMINISTIC_OPTS_FLAGS_DISALLOW_RAILGATING;

	/* Can't be both at the same time */
	if (allow && disallow)
		return -EINVAL;

	/* Nothing to do */
	if (!allow && !disallow)
		return 0;

	/*
	 * Moving into explicit idle or back from it? A call that doesn't
	 * change the status is a no-op.
	 */
	if (!ch->deterministic_railgate_allowed &&
			allow) {
		gk20a_idle(ch->g);
	} else if (ch->deterministic_railgate_allowed &&
			!allow) {
		err = gk20a_busy(ch->g);
		if (err) {
			nvgpu_warn(ch->g,
				"cannot busy to restore deterministic ch");
			return err;
		}
	}
	ch->deterministic_railgate_allowed = allow;

	return err;
}

static int nvgpu_gpu_set_deterministic_ch(struct channel_gk20a *ch, u32 flags)
{
	if (!ch->deterministic)
		return -EINVAL;

	return nvgpu_gpu_set_deterministic_ch_railgate(ch, flags);
}

static int nvgpu_gpu_set_deterministic_opts(struct gk20a *g,
		struct nvgpu_gpu_set_deterministic_opts_args *args)
{
	int __user *user_channels;
	u32 i = 0;
	int err = 0;

	nvgpu_log_fn(g, " ");

	user_channels = (int __user *)(uintptr_t)args->channels;

	/* Upper limit; prevent holding deterministic_busy for long */
	if (args->num_channels > g->fifo.num_channels) {
		err = -EINVAL;
		goto out;
	}

	/* Trivial sanity check first */
	if (!access_ok(VERIFY_READ, user_channels,
				args->num_channels * sizeof(int))) {
		err = -EFAULT;
		goto out;
	}

	nvgpu_rwsem_down_read(&g->deterministic_busy);

	/* note: we exit at the first failure */
	for (; i < args->num_channels; i++) {
		int ch_fd = 0;
		struct channel_gk20a *ch;

		if (copy_from_user(&ch_fd, &user_channels[i], sizeof(int))) {
			/* User raced with above access_ok */
			err = -EFAULT;
			break;
		}

		ch = gk20a_get_channel_from_file(ch_fd);
		if (!ch) {
			err = -EINVAL;
			break;
		}

		err = nvgpu_gpu_set_deterministic_ch(ch, args->flags);

		gk20a_channel_put(ch);

		if (err)
			break;
	}

	nvgpu_speculation_barrier();
	nvgpu_rwsem_up_read(&g->deterministic_busy);

out:
	args->num_channels = i;
	return err;
}

long gk20a_ctrl_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gk20a_ctrl_priv *priv = filp->private_data;
	struct gk20a *g = priv->g;
	struct nvgpu_gpu_zcull_get_ctx_size_args *get_ctx_size_args;
	struct nvgpu_gpu_zcull_get_info_args *get_info_args;
	struct nvgpu_gpu_zbc_set_table_args *set_table_args;
	struct nvgpu_gpu_zbc_query_table_args *query_table_args;
	u8 buf[NVGPU_GPU_IOCTL_MAX_ARG_SIZE];
	struct gr_zcull_info *zcull_info;
	struct zbc_entry *zbc_val;
	struct zbc_query_params *zbc_tbl;
	int i, err = 0;

	nvgpu_log_fn(g, "start %d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_GPU_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_GPU_IOCTL_LAST) ||
		(_IOC_SIZE(cmd) > NVGPU_GPU_IOCTL_MAX_ARG_SIZE))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	if (!g->sw_ready) {
		err = gk20a_busy(g);
		if (err)
			return err;

		gk20a_idle(g);
	}

	nvgpu_speculation_barrier();
	switch (cmd) {
	case NVGPU_GPU_IOCTL_ZCULL_GET_CTX_SIZE:
		get_ctx_size_args = (struct nvgpu_gpu_zcull_get_ctx_size_args *)buf;

		get_ctx_size_args->size = gr_gk20a_get_ctxsw_zcull_size(g, &g->gr);

		break;
	case NVGPU_GPU_IOCTL_ZCULL_GET_INFO:
		get_info_args = (struct nvgpu_gpu_zcull_get_info_args *)buf;

		memset(get_info_args, 0, sizeof(struct nvgpu_gpu_zcull_get_info_args));

		zcull_info = nvgpu_kzalloc(g, sizeof(struct gr_zcull_info));
		if (zcull_info == NULL)
			return -ENOMEM;

		err = g->ops.gr.get_zcull_info(g, &g->gr, zcull_info);
		if (err) {
			nvgpu_kfree(g, zcull_info);
			break;
		}

		get_info_args->width_align_pixels = zcull_info->width_align_pixels;
		get_info_args->height_align_pixels = zcull_info->height_align_pixels;
		get_info_args->pixel_squares_by_aliquots = zcull_info->pixel_squares_by_aliquots;
		get_info_args->aliquot_total = zcull_info->aliquot_total;
		get_info_args->region_byte_multiplier = zcull_info->region_byte_multiplier;
		get_info_args->region_header_size = zcull_info->region_header_size;
		get_info_args->subregion_header_size = zcull_info->subregion_header_size;
		get_info_args->subregion_width_align_pixels = zcull_info->subregion_width_align_pixels;
		get_info_args->subregion_height_align_pixels = zcull_info->subregion_height_align_pixels;
		get_info_args->subregion_count = zcull_info->subregion_count;

		nvgpu_kfree(g, zcull_info);
		break;
	case NVGPU_GPU_IOCTL_ZBC_SET_TABLE:
		set_table_args = (struct nvgpu_gpu_zbc_set_table_args *)buf;

		zbc_val = nvgpu_kzalloc(g, sizeof(struct zbc_entry));
		if (zbc_val == NULL)
			return -ENOMEM;

		zbc_val->format = set_table_args->format;
		zbc_val->type = set_table_args->type;

		nvgpu_speculation_barrier();
		switch (zbc_val->type) {
		case GK20A_ZBC_TYPE_COLOR:
			for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
				zbc_val->color_ds[i] = set_table_args->color_ds[i];
				zbc_val->color_l2[i] = set_table_args->color_l2[i];
			}
			break;
		case GK20A_ZBC_TYPE_DEPTH:
		case T19X_ZBC:
			zbc_val->depth = set_table_args->depth;
			break;
		default:
			err = -EINVAL;
		}

		if (!err) {
			err = gk20a_busy(g);
			if (!err) {
				err = g->ops.gr.zbc_set_table(g, &g->gr,
							     zbc_val);
				gk20a_idle(g);
			}
		}

		if (zbc_val)
			nvgpu_kfree(g, zbc_val);
		break;
	case NVGPU_GPU_IOCTL_ZBC_QUERY_TABLE:
		query_table_args = (struct nvgpu_gpu_zbc_query_table_args *)buf;

		zbc_tbl = nvgpu_kzalloc(g, sizeof(struct zbc_query_params));
		if (zbc_tbl == NULL)
			return -ENOMEM;

		zbc_tbl->type = query_table_args->type;
		zbc_tbl->index_size = query_table_args->index_size;

		err = g->ops.gr.zbc_query_table(g, &g->gr, zbc_tbl);

		if (!err) {
			switch (zbc_tbl->type) {
			case GK20A_ZBC_TYPE_COLOR:
				for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
					query_table_args->color_ds[i] = zbc_tbl->color_ds[i];
					query_table_args->color_l2[i] = zbc_tbl->color_l2[i];
				}
				break;
			case GK20A_ZBC_TYPE_DEPTH:
			case T19X_ZBC:
				query_table_args->depth = zbc_tbl->depth;
				break;
			case GK20A_ZBC_TYPE_INVALID:
				query_table_args->index_size = zbc_tbl->index_size;
				break;
			default:
				err = -EINVAL;
			}
			if (!err) {
				query_table_args->format = zbc_tbl->format;
				query_table_args->ref_cnt = zbc_tbl->ref_cnt;
			}
		}

		if (zbc_tbl)
			nvgpu_kfree(g, zbc_tbl);
		break;

	case NVGPU_GPU_IOCTL_GET_CHARACTERISTICS:
		err = gk20a_ctrl_ioctl_gpu_characteristics(
			g, (struct nvgpu_gpu_get_characteristics *)buf);
		break;
	case NVGPU_GPU_IOCTL_PREPARE_COMPRESSIBLE_READ:
		err = gk20a_ctrl_prepare_compressible_read(g,
			(struct nvgpu_gpu_prepare_compressible_read_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_MARK_COMPRESSIBLE_WRITE:
		err = gk20a_ctrl_mark_compressible_write(g,
			(struct nvgpu_gpu_mark_compressible_write_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_ALLOC_AS:
		err = gk20a_ctrl_alloc_as(g,
			(struct nvgpu_alloc_as_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_OPEN_TSG:
		err = gk20a_ctrl_open_tsg(g,
			(struct nvgpu_gpu_open_tsg_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_GET_TPC_MASKS:
		err = gk20a_ctrl_get_tpc_masks(g,
			(struct nvgpu_gpu_get_tpc_masks_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_GET_FBP_L2_MASKS:
		err = gk20a_ctrl_get_fbp_l2_masks(g,
			(struct nvgpu_gpu_get_fbp_l2_masks_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_OPEN_CHANNEL:
		/* this arg type here, but ..gpu_open_channel_args in nvgpu.h
		 * for consistency - they are the same */
		err = gk20a_channel_open_ioctl(g,
			(struct nvgpu_channel_open_args *)buf);
		break;
	case NVGPU_GPU_IOCTL_FLUSH_L2:
		err = nvgpu_gpu_ioctl_l2_fb_ops(g,
			   (struct nvgpu_gpu_l2_fb_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_SET_MMUDEBUG_MODE:
		err =  nvgpu_gpu_ioctl_set_mmu_debug_mode(g,
				(struct nvgpu_gpu_mmu_debug_mode_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_SET_SM_DEBUG_MODE:
		err = gr_gk20a_elpg_protected_call(g,
				nvgpu_gpu_ioctl_set_debug_mode(g, (struct nvgpu_gpu_sm_debug_mode_args *)buf));
		break;

	case NVGPU_GPU_IOCTL_TRIGGER_SUSPEND:
		err = nvgpu_gpu_ioctl_trigger_suspend(g);
		break;

	case NVGPU_GPU_IOCTL_WAIT_FOR_PAUSE:
		err = nvgpu_gpu_ioctl_wait_for_pause(g,
				(struct nvgpu_gpu_wait_pause_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_RESUME_FROM_PAUSE:
		err = nvgpu_gpu_ioctl_resume_from_pause(g);
		break;

	case NVGPU_GPU_IOCTL_CLEAR_SM_ERRORS:
		err = nvgpu_gpu_ioctl_clear_sm_errors(g);
		break;

	case NVGPU_GPU_IOCTL_GET_TPC_EXCEPTION_EN_STATUS:
		err =  nvgpu_gpu_ioctl_has_any_exception(g,
				(struct nvgpu_gpu_tpc_exception_en_status_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_NUM_VSMS:
		err = gk20a_ctrl_get_num_vsms(g,
			(struct nvgpu_gpu_num_vsms *)buf);
		break;
	case NVGPU_GPU_IOCTL_VSMS_MAPPING:
		err = gk20a_ctrl_vsm_mapping(g,
			(struct nvgpu_gpu_vsms_mapping *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_CPU_TIME_CORRELATION_INFO:
		err = nvgpu_gpu_get_cpu_time_correlation_info(g,
			(struct nvgpu_gpu_get_cpu_time_correlation_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_GPU_TIME:
		err = nvgpu_gpu_get_gpu_time(g,
			(struct nvgpu_gpu_get_gpu_time_args *)buf);
		break;

        case NVGPU_GPU_IOCTL_GET_ENGINE_INFO:
		err = nvgpu_gpu_get_engine_info(g,
			(struct nvgpu_gpu_get_engine_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_ALLOC_VIDMEM:
		err = nvgpu_gpu_alloc_vidmem(g,
			(struct nvgpu_gpu_alloc_vidmem_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_MEMORY_STATE:
		err = nvgpu_gpu_get_memory_state(g,
			(struct nvgpu_gpu_get_memory_state_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_CLK_GET_RANGE:
		err = nvgpu_gpu_clk_get_range(g, priv,
			(struct nvgpu_gpu_clk_range_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_CLK_GET_VF_POINTS:
		err = nvgpu_gpu_clk_get_vf_points(g, priv,
			(struct nvgpu_gpu_clk_vf_points_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_CLK_SET_INFO:
		err = nvgpu_gpu_clk_set_info(g, priv,
			(struct nvgpu_gpu_clk_set_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_CLK_GET_INFO:
		err = nvgpu_gpu_clk_get_info(g, priv,
			(struct nvgpu_gpu_clk_get_info_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_EVENT_FD:
		err = nvgpu_gpu_get_event_fd(g, priv,
			(struct nvgpu_gpu_get_event_fd_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_VOLTAGE:
		err = nvgpu_gpu_get_voltage(g,
			(struct nvgpu_gpu_get_voltage_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_CURRENT:
		err = nvgpu_gpu_get_current(g,
			(struct nvgpu_gpu_get_current_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_POWER:
		err = nvgpu_gpu_get_power(g,
			(struct nvgpu_gpu_get_power_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_GET_TEMPERATURE:
		err = nvgpu_gpu_get_temperature(g,
			(struct nvgpu_gpu_get_temperature_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_SET_THERM_ALERT_LIMIT:
		err = nvgpu_gpu_set_therm_alert_limit(g,
			(struct nvgpu_gpu_set_therm_alert_limit_args *)buf);
		break;

	case NVGPU_GPU_IOCTL_SET_DETERMINISTIC_OPTS:
		err = nvgpu_gpu_set_deterministic_opts(g,
			(struct nvgpu_gpu_set_deterministic_opts_args *)buf);
		break;

	default:
		nvgpu_log_info(g, "unrecognized gpu ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static void usermode_vma_close(struct vm_area_struct *vma)
{
	struct gk20a_ctrl_priv *priv = vma->vm_private_data;
	struct gk20a *g = priv->g;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	nvgpu_mutex_acquire(&l->ctrl.privs_lock);
	priv->usermode_vma.vma = NULL;
	priv->usermode_vma.vma_mapped = false;
	nvgpu_mutex_release(&l->ctrl.privs_lock);
}

struct vm_operations_struct usermode_vma_ops = {
	/* no .open - we use VM_DONTCOPY and don't support fork */
	.close = usermode_vma_close,
};

int gk20a_ctrl_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct gk20a_ctrl_priv *priv = filp->private_data;
	struct gk20a *g = priv->g;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	u64 addr;
	int err;

	if (g->ops.fifo.usermode_base == NULL)
		return -ENOSYS;

	if (priv->usermode_vma.vma != NULL)
		return -EBUSY;

	if (vma->vm_end - vma->vm_start != SZ_4K)
		return -EINVAL;

	if (vma->vm_pgoff != 0UL)
		return -EINVAL;

	addr = l->regs_bus_addr + g->ops.fifo.usermode_base(g);

	/* Sync with poweron/poweroff, and require valid regs */
	err = gk20a_busy(g);
	if (err) {
		return err;
	}

	nvgpu_mutex_acquire(&l->ctrl.privs_lock);

	vma->vm_flags |= VM_IO | VM_DONTCOPY | VM_DONTEXPAND | VM_NORESERVE |
		VM_DONTDUMP | VM_PFNMAP;
	vma->vm_ops = &usermode_vma_ops;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	err = io_remap_pfn_range(vma, vma->vm_start, addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if (!err) {
		priv->usermode_vma.vma = vma;
		priv->usermode_vma.flags = vma->vm_flags;
		vma->vm_private_data = priv;
		priv->usermode_vma.vma_mapped = true;
	}
	nvgpu_mutex_release(&l->ctrl.privs_lock);

	gk20a_idle(g);

	return err;
}

static void alter_usermode_mapping(struct gk20a *g,
		struct gk20a_ctrl_priv *priv,
		bool poweroff)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct vm_area_struct *vma = priv->usermode_vma.vma;
	bool vma_mapped = priv->usermode_vma.vma_mapped;
	u64 addr;
	int err;

	if (!vma) {
		/* Nothing to do - no mmap called */
		return;
	}

	addr = l->regs_bus_addr + g->ops.fifo.usermode_base(g);

	down_write(&vma->vm_mm->mmap_sem);

	/*
	 * This is a no-op for the below cases
	 * a) poweroff and !vma_mapped - > do nothing as no map exists
	 * b) !poweroff and vmap_mapped -> do nothing as already mapped
	 */
	if (poweroff && vma_mapped) {
		err = zap_vma_ptes(vma, vma->vm_start, SZ_4K);
		if (err == 0) {
			vma->vm_flags = VM_NONE;
			priv->usermode_vma.vma_mapped = false;
		} else {
			nvgpu_err(g, "can't remove usermode mapping");
		}
	} else if (!poweroff && !vma_mapped) {
		vma->vm_flags = priv->usermode_vma.flags;
		err = io_remap_pfn_range(vma, vma->vm_start,
				addr >> PAGE_SHIFT,
				SZ_4K, vma->vm_page_prot);
		if (err != 0) {
			nvgpu_err(g, "can't restore usermode mapping");
			vma->vm_flags = VM_NONE;
		} else {
			priv->usermode_vma.vma_mapped = true;
		}
	}

	up_write(&vma->vm_mm->mmap_sem);
}

static void alter_usermode_mappings(struct gk20a *g, bool poweroff)
{
	struct gk20a_ctrl_priv *priv;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	nvgpu_mutex_acquire(&l->ctrl.privs_lock);
	nvgpu_list_for_each_entry(priv, &l->ctrl.privs,
			gk20a_ctrl_priv, list) {
		alter_usermode_mapping(g, priv, poweroff);
	}
	nvgpu_mutex_release(&l->ctrl.privs_lock);
}

void nvgpu_hide_usermode_for_poweroff(struct gk20a *g)
{
	alter_usermode_mappings(g, true);
}

void nvgpu_restore_usermode_for_poweron(struct gk20a *g)
{
	alter_usermode_mappings(g, false);
}
