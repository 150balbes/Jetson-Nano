/*
 * GK20A Cycle stats snapshots support (subsystem for gr_gk20a).
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bitops.h>
#include <nvgpu/kmem.h>
#include <nvgpu/lock.h>
#include <nvgpu/dma.h>
#include <nvgpu/mm.h>
#include <nvgpu/sizes.h>
#include <nvgpu/barrier.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/channel.h>
#include <nvgpu/unit.h>

#include "gk20a.h"
#include "css_gr_gk20a.h"

#include <nvgpu/hw/gk20a/hw_perf_gk20a.h>

/* check client for pointed perfmon ownership */
#define CONTAINS_PERFMON(cl, pm)				\
		((cl)->perfmon_start <= (pm) &&			\
		((pm) - (cl)->perfmon_start) < (cl)->perfmon_count)

/* address of fifo entry by offset */
#define CSS_FIFO_ENTRY(fifo, offs)				\
	((struct gk20a_cs_snapshot_fifo_entry *)(((char *)(fifo)) + (offs)))

/* calculate area capacity in number of fifo entries */
#define CSS_FIFO_ENTRY_CAPACITY(s)				\
	(((s) - sizeof(struct gk20a_cs_snapshot_fifo))		\
		/ sizeof(struct gk20a_cs_snapshot_fifo_entry))

/* reserved to indicate failures with data */
#define CSS_FIRST_PERFMON_ID	32
/* should correlate with size of gk20a_cs_snapshot_fifo_entry::perfmon_id */
#define CSS_MAX_PERFMON_IDS	256

/* reports whether the hw queue overflowed */
bool css_hw_get_overflow_status(struct gk20a *g)
{
	const u32 st = perf_pmasys_control_membuf_status_overflowed_f();
	return st == (gk20a_readl(g, perf_pmasys_control_r()) & st);
}

/* returns how many pending snapshot entries are pending */
u32 css_hw_get_pending_snapshots(struct gk20a *g)
{
	return gk20a_readl(g, perf_pmasys_mem_bytes_r()) /
			sizeof(struct gk20a_cs_snapshot_fifo_entry);
}

/* informs hw how many snapshots have been processed (frees up fifo space) */
void css_hw_set_handled_snapshots(struct gk20a *g, u32 done)
{
	if (done > 0) {
		gk20a_writel(g, perf_pmasys_mem_bump_r(),
		     done * sizeof(struct gk20a_cs_snapshot_fifo_entry));
	}
}

/* disable streaming to memory */
static void css_hw_reset_streaming(struct gk20a *g)
{
	u32 engine_status;

	/* reset the perfmon */
	g->ops.mc.reset(g, g->ops.mc.reset_mask(g, NVGPU_UNIT_PERFMON));

	/* RBUFEMPTY must be set -- otherwise we'll pick up */
	/* snapshot that have been queued up from earlier   */
	engine_status = gk20a_readl(g, perf_pmasys_enginestatus_r());
	WARN_ON(0 == (engine_status
			& perf_pmasys_enginestatus_rbufempty_empty_f()));

	/* turn off writes */
	gk20a_writel(g, perf_pmasys_control_r(),
			perf_pmasys_control_membuf_clear_status_doit_f());

	/* pointing all pending snapshots as handled */
	css_hw_set_handled_snapshots(g, css_hw_get_pending_snapshots(g));
}

/*
 * WARNING: all css_gr_XXX functions are local and expected to be called
 * from locked context (protected by cs_lock)
 */

static int css_gr_create_shared_data(struct gr_gk20a *gr)
{
	struct gk20a_cs_snapshot *data;

	if (gr->cs_data)
		return 0;

	data = nvgpu_kzalloc(gr->g, sizeof(*data));
	if (!data)
		return -ENOMEM;

	nvgpu_init_list_node(&data->clients);
	gr->cs_data = data;

	return 0;
}

int css_hw_enable_snapshot(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *cs_client)
{
	struct gk20a *g = ch->g;
	struct mm_gk20a *mm = &g->mm;
	struct gr_gk20a *gr = &g->gr;
	struct gk20a_cs_snapshot *data = gr->cs_data;
	u32 snapshot_size = cs_client->snapshot_size;
	int ret;

	u32 virt_addr_lo;
	u32 virt_addr_hi;
	u32 inst_pa_page;

	if (data->hw_snapshot)
		return 0;

	if (snapshot_size < CSS_MIN_HW_SNAPSHOT_SIZE)
		snapshot_size = CSS_MIN_HW_SNAPSHOT_SIZE;

	ret = nvgpu_dma_alloc_map_sys(g->mm.pmu.vm, snapshot_size,
							&data->hw_memdesc);
	if (ret)
		return ret;

	/* perf output buffer may not cross a 4GB boundary - with a separate */
	/* va smaller than that, it won't but check anyway */
	if (!data->hw_memdesc.cpu_va ||
		data->hw_memdesc.size < snapshot_size ||
		data->hw_memdesc.gpu_va + u64_lo32(snapshot_size) > SZ_4G) {
		ret = -EFAULT;
		goto failed_allocation;
	}

	data->hw_snapshot =
		(struct gk20a_cs_snapshot_fifo_entry *)data->hw_memdesc.cpu_va;
	data->hw_end = data->hw_snapshot +
		snapshot_size / sizeof(struct gk20a_cs_snapshot_fifo_entry);
	data->hw_get = data->hw_snapshot;
	memset(data->hw_snapshot, 0xff, snapshot_size);

	/* address and size are aligned to 32 bytes, the lowest bits read back
	 * as zeros */
	virt_addr_lo = u64_lo32(data->hw_memdesc.gpu_va);
	virt_addr_hi = u64_hi32(data->hw_memdesc.gpu_va);

	css_hw_reset_streaming(g);

	gk20a_writel(g, perf_pmasys_outbase_r(), virt_addr_lo);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(virt_addr_hi));
	gk20a_writel(g, perf_pmasys_outsize_r(), snapshot_size);

	/* this field is aligned to 4K */
	inst_pa_page = nvgpu_inst_block_addr(g, &g->mm.hwpm.inst_block) >> 12;

	/* A write to MEM_BLOCK triggers the block bind operation. MEM_BLOCK
	 * should be written last */
	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(inst_pa_page) |
		        nvgpu_aperture_mask(g, &mm->hwpm.inst_block,
				perf_pmasys_mem_block_target_sys_ncoh_f(),
				perf_pmasys_mem_block_target_sys_coh_f(),
				perf_pmasys_mem_block_target_lfb_f()) |
			perf_pmasys_mem_block_valid_true_f());

	nvgpu_log_info(g, "cyclestats: buffer for hardware snapshots enabled\n");

	return 0;

failed_allocation:
	if (data->hw_memdesc.size) {
		nvgpu_dma_unmap_free(g->mm.pmu.vm, &data->hw_memdesc);
		memset(&data->hw_memdesc, 0, sizeof(data->hw_memdesc));
	}
	data->hw_snapshot = NULL;

	return ret;
}

void css_hw_disable_snapshot(struct gr_gk20a *gr)
{
	struct gk20a *g = gr->g;
	struct gk20a_cs_snapshot *data = gr->cs_data;

	if (!data->hw_snapshot)
		return;

	css_hw_reset_streaming(g);

	gk20a_writel(g, perf_pmasys_outbase_r(), 0);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(0));
	gk20a_writel(g, perf_pmasys_outsize_r(), 0);

	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(0) |
			perf_pmasys_mem_block_valid_false_f() |
			perf_pmasys_mem_block_target_f(0));

	nvgpu_dma_unmap_free(g->mm.pmu.vm, &data->hw_memdesc);
	memset(&data->hw_memdesc, 0, sizeof(data->hw_memdesc));
	data->hw_snapshot = NULL;

	nvgpu_log_info(g, "cyclestats: buffer for hardware snapshots disabled\n");
}

static void css_gr_free_shared_data(struct gr_gk20a *gr)
{
	struct gk20a *g = gr->g;

	if (gr->cs_data) {
		/* the clients list is expected to be empty */
		g->ops.css.disable_snapshot(gr);

		/* release the objects */
		nvgpu_kfree(gr->g, gr->cs_data);
		gr->cs_data = NULL;
	}
}


struct gk20a_cs_snapshot_client*
css_gr_search_client(struct nvgpu_list_node *clients, u32 perfmon)
{
	struct gk20a_cs_snapshot_client *client;

	nvgpu_list_for_each_entry(client, clients,
			gk20a_cs_snapshot_client,  list) {
		if (CONTAINS_PERFMON(client, perfmon))
			return client;
	}

	return NULL;
}

static int css_gr_flush_snapshots(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;
	struct gk20a_cs_snapshot *css = gr->cs_data;
	struct gk20a_cs_snapshot_client *cur;
	u32 pending, completed;
	bool hw_overflow;
	int err;

	/* variables for iterating over HW entries */
	u32 sid;
	struct gk20a_cs_snapshot_fifo_entry *src;

	/* due to data sharing with userspace we allowed update only */
	/* overflows and put field in the fifo header                */
	struct gk20a_cs_snapshot_fifo *dst;
	struct gk20a_cs_snapshot_fifo_entry *dst_get;
	struct gk20a_cs_snapshot_fifo_entry *dst_put;
	struct gk20a_cs_snapshot_fifo_entry *dst_nxt;
	struct gk20a_cs_snapshot_fifo_entry *dst_head;
	struct gk20a_cs_snapshot_fifo_entry *dst_tail;

	if (!css)
		return -EINVAL;

	if (nvgpu_list_empty(&css->clients))
		return -EBADF;

	/* check data available */
	err = g->ops.css.check_data_available(ch, &pending, &hw_overflow);
	if (err)
		return err;

	if (!pending)
		return 0;

	if (hw_overflow) {
		nvgpu_list_for_each_entry(cur, &css->clients,
				gk20a_cs_snapshot_client, list) {
			cur->snapshot->hw_overflow_events_occured++;
		}

		nvgpu_warn(g, "cyclestats: hardware overflow detected");
	}

	/* process all items in HW buffer */
	sid = 0;
	completed = 0;
	cur = NULL;
	dst = NULL;
	dst_put = NULL;
	src = css->hw_get;

	/* proceed all completed records */
	while (sid < pending && 0 == src->zero0) {
		/* we may have a new perfmon_id which required to */
		/* switch to a new client -> let's forget current */
		if (cur && !CONTAINS_PERFMON(cur, src->perfmon_id)) {
			dst->put = (char *)dst_put - (char *)dst;
			dst = NULL;
			cur = NULL;
		}

		/* now we have to select a new current client         */
		/* the client selection rate depends from experiment  */
		/* activity but on Android usually happened 1-2 times */
		if (!cur) {
			cur = css_gr_search_client(&css->clients,
							src->perfmon_id);
			if (cur) {
				/* found - setup all required data */
				dst = cur->snapshot;
				dst_get = CSS_FIFO_ENTRY(dst, dst->get);
				dst_put = CSS_FIFO_ENTRY(dst, dst->put);
				dst_head = CSS_FIFO_ENTRY(dst, dst->start);
				dst_tail = CSS_FIFO_ENTRY(dst, dst->end);

				dst_nxt = dst_put + 1;
				if (dst_nxt == dst_tail)
					dst_nxt = dst_head;
			} else {
				/* client not found - skipping this entry */
				nvgpu_warn(g, "cyclestats: orphaned perfmon %u",
							src->perfmon_id);
				goto next_hw_fifo_entry;
			}
		}

		/* check for software overflows */
		if (dst_nxt == dst_get) {
			/* no data copy, no pointer updates */
			dst->sw_overflow_events_occured++;
			nvgpu_warn(g, "cyclestats: perfmon %u soft overflow",
							src->perfmon_id);
		} else {
			*dst_put = *src;
			completed++;

			dst_put = dst_nxt++;

			if (dst_nxt == dst_tail)
				dst_nxt = dst_head;
		}

next_hw_fifo_entry:
		sid++;
		if (++src >= css->hw_end)
			src = css->hw_snapshot;
	}

	/* update client put pointer if necessary */
	if (cur && dst)
		dst->put = (char *)dst_put - (char *)dst;

	/* re-set HW buffer after processing taking wrapping into account */
	if (css->hw_get < src) {
		memset(css->hw_get, 0xff, (src - css->hw_get) * sizeof(*src));
	} else {
		memset(css->hw_snapshot, 0xff,
				(src - css->hw_snapshot) * sizeof(*src));
		memset(css->hw_get, 0xff,
				(css->hw_end - css->hw_get) * sizeof(*src));
	}
	gr->cs_data->hw_get = src;

	if (g->ops.css.set_handled_snapshots)
		g->ops.css.set_handled_snapshots(g, sid);

	if (completed != sid) {
		/* not all entries proceed correctly. some of problems */
		/* reported as overflows, some as orphaned perfmons,   */
		/* but it will be better notify with summary about it  */
		nvgpu_warn(g, "cyclestats: completed %u from %u entries",
							completed, pending);
	}

	return 0;
}

u32 css_gr_allocate_perfmon_ids(struct gk20a_cs_snapshot *data,
				       u32 count)
{
	unsigned long *pids = data->perfmon_ids;
	unsigned int f;

	f = bitmap_find_next_zero_area(pids, CSS_MAX_PERFMON_IDS,
				       CSS_FIRST_PERFMON_ID, count, 0);
	if (f > CSS_MAX_PERFMON_IDS)
		f = 0;
	else
		bitmap_set(pids, f, count);

	return f;
}

u32 css_gr_release_perfmon_ids(struct gk20a_cs_snapshot *data,
				      u32 start,
				      u32 count)
{
	unsigned long *pids = data->perfmon_ids;
	u32  end = start + count;
	u32  cnt = 0;

	if (start >= CSS_FIRST_PERFMON_ID && end <= CSS_MAX_PERFMON_IDS) {
		bitmap_clear(pids, start, count);
		cnt = count;
	}

	return cnt;
}


static int css_gr_free_client_data(struct gk20a *g,
				struct gk20a_cs_snapshot *data,
				struct gk20a_cs_snapshot_client *client)
{
	int ret = 0;

	if (client->list.next && client->list.prev)
		nvgpu_list_del(&client->list);

	if (client->perfmon_start && client->perfmon_count
					&& g->ops.css.release_perfmon_ids) {
		if (client->perfmon_count != g->ops.css.release_perfmon_ids(data,
				client->perfmon_start, client->perfmon_count))
			ret = -EINVAL;
	}

	return ret;
}

static int css_gr_create_client_data(struct gk20a *g,
			struct gk20a_cs_snapshot *data,
			u32 perfmon_count,
			struct gk20a_cs_snapshot_client *cur)
{
	/*
	 * Special handling in-case of rm-server
	 *
	 * client snapshot buffer will not be mapped
	 * in-case of rm-server its only mapped in
	 * guest side
	 */
	if (cur->snapshot) {
		memset(cur->snapshot, 0, sizeof(*cur->snapshot));
		cur->snapshot->start = sizeof(*cur->snapshot);
		/* we should be ensure that can fit all fifo entries here */
		cur->snapshot->end =
			CSS_FIFO_ENTRY_CAPACITY(cur->snapshot_size)
				* sizeof(struct gk20a_cs_snapshot_fifo_entry)
				+ sizeof(struct gk20a_cs_snapshot_fifo);
		cur->snapshot->get = cur->snapshot->start;
		cur->snapshot->put = cur->snapshot->start;
	}

	cur->perfmon_count = perfmon_count;

	/* In virtual case, perfmon ID allocation is handled by the server
	 * at the time of the attach (allocate_perfmon_ids is NULL in this case)
	 */
	if (cur->perfmon_count && g->ops.css.allocate_perfmon_ids) {
		cur->perfmon_start = g->ops.css.allocate_perfmon_ids(data,
							cur->perfmon_count);
		if (!cur->perfmon_start)
			return -ENOENT;
	}

	nvgpu_list_add_tail(&cur->list, &data->clients);

	return 0;
}


int gr_gk20a_css_attach(struct channel_gk20a *ch,
			u32 perfmon_count,
			u32 *perfmon_start,
			struct gk20a_cs_snapshot_client *cs_client)
{
	int ret = 0;
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr;

	/* we must have a placeholder to store pointer to client structure */
	if (!cs_client)
		return -EINVAL;

	if (!perfmon_count ||
	    perfmon_count > CSS_MAX_PERFMON_IDS - CSS_FIRST_PERFMON_ID)
		return -EINVAL;

	nvgpu_speculation_barrier();

	gr = &g->gr;

	nvgpu_mutex_acquire(&gr->cs_lock);

	ret = css_gr_create_shared_data(gr);
	if (ret)
		goto failed;

	ret = css_gr_create_client_data(g, gr->cs_data,
				     perfmon_count,
				     cs_client);
	if (ret)
		goto failed;

	ret = g->ops.css.enable_snapshot(ch, cs_client);
	if (ret)
		goto failed;

	if (perfmon_start)
		*perfmon_start = cs_client->perfmon_start;

	nvgpu_mutex_release(&gr->cs_lock);

	return 0;

failed:
	if (gr->cs_data) {
		if (cs_client) {
			css_gr_free_client_data(g, gr->cs_data, cs_client);
			cs_client = NULL;
		}

		if (nvgpu_list_empty(&gr->cs_data->clients))
			css_gr_free_shared_data(gr);
	}
	nvgpu_mutex_release(&gr->cs_lock);

	if (perfmon_start)
		*perfmon_start = 0;

	return ret;
}

int gr_gk20a_css_detach(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *cs_client)
{
	int ret = 0;
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr;

	if (!cs_client)
		return -EINVAL;

	gr = &g->gr;
	nvgpu_mutex_acquire(&gr->cs_lock);
	if (gr->cs_data) {
		struct gk20a_cs_snapshot *data = gr->cs_data;

		if (g->ops.css.detach_snapshot)
			g->ops.css.detach_snapshot(ch, cs_client);

		ret = css_gr_free_client_data(g, data, cs_client);
		if (nvgpu_list_empty(&data->clients))
			css_gr_free_shared_data(gr);
	} else {
		ret = -EBADF;
	}
	nvgpu_mutex_release(&gr->cs_lock);

	return ret;
}

int gr_gk20a_css_flush(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *cs_client)
{
	int ret = 0;
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr;

	if (!cs_client)
		return -EINVAL;

	gr = &g->gr;
	nvgpu_mutex_acquire(&gr->cs_lock);
	ret = css_gr_flush_snapshots(ch);
	nvgpu_mutex_release(&gr->cs_lock);

	return ret;
}

/* helper function with locking to cleanup snapshot code code in gr_gk20a.c */
void gr_gk20a_free_cyclestats_snapshot_data(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	nvgpu_mutex_acquire(&gr->cs_lock);
	css_gr_free_shared_data(gr);
	nvgpu_mutex_release(&gr->cs_lock);
	nvgpu_mutex_destroy(&gr->cs_lock);
}

int css_hw_check_data_available(struct channel_gk20a *ch, u32 *pending,
					bool *hw_overflow)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;
	struct gk20a_cs_snapshot *css = gr->cs_data;

	if (!css->hw_snapshot)
		return -EINVAL;

	*pending = css_hw_get_pending_snapshots(g);
	if (!*pending)
		return 0;

	*hw_overflow = css_hw_get_overflow_status(g);
	return 0;
}
