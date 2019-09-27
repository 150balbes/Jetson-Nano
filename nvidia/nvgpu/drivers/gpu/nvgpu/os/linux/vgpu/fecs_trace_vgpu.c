/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <uapi/linux/nvgpu.h>

#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/ctxsw_trace.h>
#include <nvgpu/vgpu/vgpu_ivm.h>
#include <nvgpu/vgpu/tegra_vgpu.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/gk20a.h>

#include "os/linux/os_linux.h"
#include "gk20a/fecs_trace_gk20a.h"
#include "vgpu/fecs_trace_vgpu.h"

struct vgpu_fecs_trace {
	struct tegra_hv_ivm_cookie *cookie;
	struct nvgpu_ctxsw_ring_header *header;
	struct nvgpu_gpu_ctxsw_trace_entry *entries;
	int num_entries;
	bool enabled;
	void *buf;
};

int vgpu_fecs_trace_init(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);
	struct device_node *np = dev->of_node;
	struct of_phandle_args args;
	struct vgpu_fecs_trace *vcst;
	u32 mempool;
	int err;

	nvgpu_log_fn(g, " ");

	vcst = nvgpu_kzalloc(g, sizeof(*vcst));
	if (!vcst)
		return -ENOMEM;

	err = of_parse_phandle_with_fixed_args(np,
			"mempool-fecs-trace", 1, 0, &args);
	if (err) {
		nvgpu_info(g, "does not support fecs trace");
		goto fail;
	}
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_FECS_CTXSW_TRACE, true);

	mempool = args.args[0];
	vcst->cookie = vgpu_ivm_mempool_reserve(mempool);
	if (IS_ERR(vcst->cookie)) {
		nvgpu_info(g,
			"mempool  %u reserve failed", mempool);
		vcst->cookie = NULL;
		err = -EINVAL;
		goto fail;
	}

	vcst->buf = ioremap_cache(vgpu_ivm_get_ipa(vcst->cookie),
			vgpu_ivm_get_size(vcst->cookie));
	if (!vcst->buf) {
		nvgpu_info(g, "ioremap_cache failed");
		err = -EINVAL;
		goto fail;
	}
	vcst->header = vcst->buf;
	vcst->num_entries = vcst->header->num_ents;
	if (unlikely(vcst->header->ent_size != sizeof(*vcst->entries))) {
		nvgpu_err(g, "entry size mismatch");
		goto fail;
	}
	vcst->entries = vcst->buf + sizeof(*vcst->header);
	g->fecs_trace = (struct gk20a_fecs_trace *)vcst;

	return 0;
fail:
	iounmap(vcst->buf);
	if (vcst->cookie)
		vgpu_ivm_mempool_unreserve(vcst->cookie);
	nvgpu_kfree(g, vcst);
	return err;
}

int vgpu_fecs_trace_deinit(struct gk20a *g)
{
	struct vgpu_fecs_trace *vcst = (struct vgpu_fecs_trace *)g->fecs_trace;

	iounmap(vcst->buf);
	vgpu_ivm_mempool_unreserve(vcst->cookie);
	nvgpu_kfree(g, vcst);
	return 0;
}

int vgpu_fecs_trace_enable(struct gk20a *g)
{
	struct vgpu_fecs_trace *vcst = (struct vgpu_fecs_trace *)g->fecs_trace;
	struct tegra_vgpu_cmd_msg msg = {
		.cmd = TEGRA_VGPU_CMD_FECS_TRACE_ENABLE,
		.handle = vgpu_get_handle(g),
	};
	int err;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);
	vcst->enabled = !err;
	return err;
}

int vgpu_fecs_trace_disable(struct gk20a *g)
{
	struct vgpu_fecs_trace *vcst = (struct vgpu_fecs_trace *)g->fecs_trace;
	struct tegra_vgpu_cmd_msg msg = {
		.cmd = TEGRA_VGPU_CMD_FECS_TRACE_DISABLE,
		.handle = vgpu_get_handle(g),
	};
	int err;

	vcst->enabled = false;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);
	return err;
}

bool vgpu_fecs_trace_is_enabled(struct gk20a *g)
{
	struct vgpu_fecs_trace *vcst = (struct vgpu_fecs_trace *)g->fecs_trace;

	return (vcst && vcst->enabled);
}

int vgpu_fecs_trace_poll(struct gk20a *g)
{
	struct tegra_vgpu_cmd_msg msg = {
		.cmd = TEGRA_VGPU_CMD_FECS_TRACE_POLL,
		.handle = vgpu_get_handle(g),
	};
	int err;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);
	return err;
}

int vgpu_alloc_user_buffer(struct gk20a *g, void **buf, size_t *size)
{
	struct vgpu_fecs_trace *vcst = (struct vgpu_fecs_trace *)g->fecs_trace;

	*buf = vcst->buf;
	*size = vgpu_ivm_get_size(vcst->cookie);
	return 0;
}

int vgpu_free_user_buffer(struct gk20a *g)
{
	return 0;
}

int vgpu_mmap_user_buffer(struct gk20a *g, struct vm_area_struct *vma)
{
	struct vgpu_fecs_trace *vcst = (struct vgpu_fecs_trace *)g->fecs_trace;
	unsigned long size = vgpu_ivm_get_size(vcst->cookie);
	unsigned long vsize = vma->vm_end - vma->vm_start;

	size = min(size, vsize);
	size = round_up(size, PAGE_SIZE);

	return remap_pfn_range(vma, vma->vm_start,
			vgpu_ivm_get_ipa(vcst->cookie) >> PAGE_SHIFT,
			size,
			vma->vm_page_prot);
}

#ifdef CONFIG_GK20A_CTXSW_TRACE
int vgpu_fecs_trace_max_entries(struct gk20a *g,
			struct nvgpu_gpu_ctxsw_trace_filter *filter)
{
	struct vgpu_fecs_trace *vcst = (struct vgpu_fecs_trace *)g->fecs_trace;

	return vcst->header->num_ents;
}

#if NVGPU_CTXSW_FILTER_SIZE != TEGRA_VGPU_FECS_TRACE_FILTER_SIZE
#error "FECS trace filter size mismatch!"
#endif

int vgpu_fecs_trace_set_filter(struct gk20a *g,
			struct nvgpu_gpu_ctxsw_trace_filter *filter)
{
	struct tegra_vgpu_cmd_msg msg = {
		.cmd = TEGRA_VGPU_CMD_FECS_TRACE_SET_FILTER,
		.handle = vgpu_get_handle(g),
	};
	struct tegra_vgpu_fecs_trace_filter *p = &msg.params.fecs_trace_filter;
	int err;

	memcpy(&p->tag_bits, &filter->tag_bits, sizeof(p->tag_bits));
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);
	return err;
}

void vgpu_fecs_trace_data_update(struct gk20a *g)
{
	gk20a_ctxsw_trace_wake_up(g, 0);
}
#endif /* CONFIG_GK20A_CTXSW_TRACE */
