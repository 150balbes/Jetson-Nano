/*
 * GK20A PMU (aka. gPMU outside gk20a context)
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/nvgpu_common.h>
#include <nvgpu/timers.h>
#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>
#include <nvgpu/firmware.h>
#include <nvgpu/falcon.h>
#include <nvgpu/mm.h>
#include <nvgpu/io.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/utils.h>
#include <nvgpu/unit.h>

#include "gk20a.h"
#include "gr_gk20a.h"
#include "pmu_gk20a.h"

#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pwr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_top_gk20a.h>

#define gk20a_dbg_pmu(g, fmt, arg...) \
	nvgpu_log(g, gpu_dbg_pmu, fmt, ##arg)

bool nvgpu_find_hex_in_string(char *strings, struct gk20a *g, u32 *hex_pos)
{
	u32 i = 0, j = strlen(strings);

	for (; i < j; i++) {
		if (strings[i] == '%') {
			if (strings[i + 1] == 'x' || strings[i + 1] == 'X') {
				*hex_pos = i;
				return true;
			}
		}
	}
	*hex_pos = -1;
	return false;
}

static void print_pmu_trace(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = pmu->g;
	u32 i = 0, j = 0, k, l, m, count;
	char part_str[40], buf[0x40];
	void *tracebuffer;
	char *trace;
	u32 *trace1;

	/* allocate system memory to copy pmu trace buffer */
	tracebuffer = nvgpu_kzalloc(g, GK20A_PMU_TRACE_BUFSIZE);
	if (tracebuffer == NULL) {
		return;
	}

	/* read pmu traces into system memory buffer */
	nvgpu_mem_rd_n(g, &pmu->trace_buf, 0, tracebuffer,
		GK20A_PMU_TRACE_BUFSIZE);

	trace = (char *)tracebuffer;
	trace1 = (u32 *)tracebuffer;

	nvgpu_err(g, "dump PMU trace buffer");
	for (i = 0; i < GK20A_PMU_TRACE_BUFSIZE; i += 0x40) {
		for (j = 0; j < 0x40; j++) {
			if (trace1[(i / 4) + j]) {
				break;
			}
		}
		if (j == 0x40) {
			break;
		}
		count = scnprintf(buf, 0x40, "Index %x: ", trace1[(i / 4)]);
		l = 0;
		m = 0;
		while (nvgpu_find_hex_in_string((trace+i+20+m), g, &k)) {
			if (k >= 40) {
				break;
			}
			strncpy(part_str, (trace+i+20+m), k);
			part_str[k] = '\0';
			count += scnprintf((buf + count), 0x40, "%s0x%x",
					part_str, trace1[(i / 4) + 1 + l]);
			l++;
			m += k + 2;
		}

		scnprintf((buf + count), 0x40, "%s", (trace+i+20+m));
		nvgpu_err(g, "%s", buf);
	}

	nvgpu_kfree(g, tracebuffer);
}

u32 gk20a_pmu_get_irqdest(struct gk20a *g)
{
	u32 intr_dest;

	/* dest 0=falcon, 1=host; level 0=irq0, 1=irq1 */
	intr_dest = pwr_falcon_irqdest_host_gptmr_f(0)    |
		pwr_falcon_irqdest_host_wdtmr_f(1)    |
		pwr_falcon_irqdest_host_mthd_f(0)     |
		pwr_falcon_irqdest_host_ctxsw_f(0)    |
		pwr_falcon_irqdest_host_halt_f(1)     |
		pwr_falcon_irqdest_host_exterr_f(0)   |
		pwr_falcon_irqdest_host_swgen0_f(1)   |
		pwr_falcon_irqdest_host_swgen1_f(0)   |
		pwr_falcon_irqdest_host_ext_f(0xff)   |
		pwr_falcon_irqdest_target_gptmr_f(1)  |
		pwr_falcon_irqdest_target_wdtmr_f(0)  |
		pwr_falcon_irqdest_target_mthd_f(0)   |
		pwr_falcon_irqdest_target_ctxsw_f(0)  |
		pwr_falcon_irqdest_target_halt_f(0)   |
		pwr_falcon_irqdest_target_exterr_f(0) |
		pwr_falcon_irqdest_target_swgen0_f(0) |
		pwr_falcon_irqdest_target_swgen1_f(0) |
		pwr_falcon_irqdest_target_ext_f(0xff);

	return intr_dest;
}

void gk20a_pmu_enable_irq(struct nvgpu_pmu *pmu, bool enable)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	u32 intr_mask;
	u32 intr_dest;

	nvgpu_log_fn(g, " ");

	g->ops.mc.intr_unit_config(g, MC_INTR_UNIT_DISABLE, true,
			mc_intr_mask_0_pmu_enabled_f());
	g->ops.mc.intr_unit_config(g, MC_INTR_UNIT_DISABLE, false,
			mc_intr_mask_1_pmu_enabled_f());

	nvgpu_flcn_set_irq(pmu->flcn, false, 0x0, 0x0);

	if (enable) {
		intr_dest = g->ops.pmu.get_irqdest(g);
		/* 0=disable, 1=enable */
		intr_mask = pwr_falcon_irqmset_gptmr_f(1)  |
			pwr_falcon_irqmset_wdtmr_f(1)  |
			pwr_falcon_irqmset_mthd_f(0)   |
			pwr_falcon_irqmset_ctxsw_f(0)  |
			pwr_falcon_irqmset_halt_f(1)   |
			pwr_falcon_irqmset_exterr_f(1) |
			pwr_falcon_irqmset_swgen0_f(1) |
			pwr_falcon_irqmset_swgen1_f(1);

		nvgpu_flcn_set_irq(pmu->flcn, true, intr_mask, intr_dest);

		g->ops.mc.intr_unit_config(g, MC_INTR_UNIT_ENABLE, true,
				mc_intr_mask_0_pmu_enabled_f());
	}

	nvgpu_log_fn(g, "done");
}



int pmu_bootstrap(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct mm_gk20a *mm = &g->mm;
	struct pmu_ucode_desc *desc = pmu->desc;
	u64 addr_code, addr_data, addr_load;
	u32 i, blocks, addr_args;

	nvgpu_log_fn(g, " ");

	gk20a_writel(g, pwr_falcon_itfen_r(),
		gk20a_readl(g, pwr_falcon_itfen_r()) |
		pwr_falcon_itfen_ctxen_enable_f());
	gk20a_writel(g, pwr_pmu_new_instblk_r(),
		pwr_pmu_new_instblk_ptr_f(
			nvgpu_inst_block_addr(g, &mm->pmu.inst_block) >> 12) |
		pwr_pmu_new_instblk_valid_f(1) |
		pwr_pmu_new_instblk_target_sys_coh_f());

	/* TBD: load all other surfaces */
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_size(
		pmu, GK20A_PMU_TRACE_BUFSIZE);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_base(pmu);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_idx(
		pmu, GK20A_PMU_DMAIDX_VIRT);

	g->ops.pmu_ver.set_pmu_cmdline_args_cpu_freq(pmu,
		g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_PWRCLK));

	addr_args = (pwr_falcon_hwcfg_dmem_size_v(
		gk20a_readl(g, pwr_falcon_hwcfg_r()))
			<< GK20A_PMU_DMEM_BLKSIZE2) -
		g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu);

	nvgpu_flcn_copy_to_dmem(pmu->flcn, addr_args,
			(u8 *)(g->ops.pmu_ver.get_pmu_cmdline_args_ptr(pmu)),
			g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu), 0);

	gk20a_writel(g, pwr_falcon_dmemc_r(0),
		pwr_falcon_dmemc_offs_f(0) |
		pwr_falcon_dmemc_blk_f(0)  |
		pwr_falcon_dmemc_aincw_f(1));

	addr_code = u64_lo32((pmu->ucode.gpu_va +
			desc->app_start_offset +
			desc->app_resident_code_offset) >> 8) ;
	addr_data = u64_lo32((pmu->ucode.gpu_va +
			desc->app_start_offset +
			desc->app_resident_data_offset) >> 8);
	addr_load = u64_lo32((pmu->ucode.gpu_va +
			desc->bootloader_start_offset) >> 8);

	gk20a_writel(g, pwr_falcon_dmemd_r(0), GK20A_PMU_DMAIDX_UCODE);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_code);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_size);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_resident_code_size);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_imem_entry);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_data);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), desc->app_resident_data_size);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_code);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), 0x1);
	gk20a_writel(g, pwr_falcon_dmemd_r(0), addr_args);

	g->ops.pmu.write_dmatrfbase(g,
			addr_load - (desc->bootloader_imem_offset >> 8));

	blocks = ((desc->bootloader_size + 0xFF) & ~0xFF) >> 8;

	for (i = 0; i < blocks; i++) {
		gk20a_writel(g, pwr_falcon_dmatrfmoffs_r(),
			desc->bootloader_imem_offset + (i << 8));
		gk20a_writel(g, pwr_falcon_dmatrffboffs_r(),
			desc->bootloader_imem_offset + (i << 8));
		gk20a_writel(g, pwr_falcon_dmatrfcmd_r(),
			pwr_falcon_dmatrfcmd_imem_f(1)  |
			pwr_falcon_dmatrfcmd_write_f(0) |
			pwr_falcon_dmatrfcmd_size_f(6)  |
			pwr_falcon_dmatrfcmd_ctxdma_f(GK20A_PMU_DMAIDX_UCODE));
	}

	nvgpu_flcn_bootstrap(g->pmu.flcn, desc->bootloader_entry_point);

	gk20a_writel(g, pwr_falcon_os_r(), desc->app_version);

	return 0;
}

void gk20a_pmu_pg_idle_counter_config(struct gk20a *g, u32 pg_engine_id)
{
	gk20a_writel(g, pwr_pmu_pg_idlefilth_r(pg_engine_id),
		PMU_PG_IDLE_THRESHOLD);
	gk20a_writel(g, pwr_pmu_pg_ppuidlefilth_r(pg_engine_id),
		PMU_PG_POST_POWERUP_IDLE_THRESHOLD);
}

int gk20a_pmu_mutex_acquire(struct nvgpu_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_mutex *mutex;
	u32 data, owner, max_retry;

	if (!pmu->initialized) {
		return -EINVAL;
	}

	BUG_ON(!token);
	BUG_ON(!PMU_MUTEX_ID_IS_VALID(id));
	BUG_ON(id > pmu->mutex_cnt);

	mutex = &pmu->mutex[id];

	owner = pwr_pmu_mutex_value_v(
		gk20a_readl(g, pwr_pmu_mutex_r(mutex->index)));

	if (*token != PMU_INVALID_MUTEX_OWNER_ID && *token == owner) {
		BUG_ON(mutex->ref_cnt == 0);
		gk20a_dbg_pmu(g, "already acquired by owner : 0x%08x", *token);
		mutex->ref_cnt++;
		return 0;
	}

	max_retry = 40;
	do {
		data = pwr_pmu_mutex_id_value_v(
			gk20a_readl(g, pwr_pmu_mutex_id_r()));
		if (data == pwr_pmu_mutex_id_value_init_v() ||
		    data == pwr_pmu_mutex_id_value_not_avail_v()) {
			nvgpu_warn(g,
				"fail to generate mutex token: val 0x%08x",
				owner);
			nvgpu_usleep_range(20, 40);
			continue;
		}

		owner = data;
		gk20a_writel(g, pwr_pmu_mutex_r(mutex->index),
			pwr_pmu_mutex_value_f(owner));

		data = pwr_pmu_mutex_value_v(
			gk20a_readl(g, pwr_pmu_mutex_r(mutex->index)));

		if (owner == data) {
			mutex->ref_cnt = 1;
			gk20a_dbg_pmu(g, "mutex acquired: id=%d, token=0x%x",
				mutex->index, *token);
			*token = owner;
			return 0;
		} else {
			nvgpu_log_info(g, "fail to acquire mutex idx=0x%08x",
				mutex->index);

			data = gk20a_readl(g, pwr_pmu_mutex_id_release_r());
			data = set_field(data,
				pwr_pmu_mutex_id_release_value_m(),
				pwr_pmu_mutex_id_release_value_f(owner));
			gk20a_writel(g, pwr_pmu_mutex_id_release_r(), data);

			nvgpu_usleep_range(20, 40);
			continue;
		}
	} while (max_retry-- > 0);

	return -EBUSY;
}

int gk20a_pmu_mutex_release(struct nvgpu_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_mutex *mutex;
	u32 owner, data;

	if (!pmu->initialized) {
		return -EINVAL;
	}

	BUG_ON(!token);
	BUG_ON(!PMU_MUTEX_ID_IS_VALID(id));
	BUG_ON(id > pmu->mutex_cnt);

	mutex = &pmu->mutex[id];

	owner = pwr_pmu_mutex_value_v(
		gk20a_readl(g, pwr_pmu_mutex_r(mutex->index)));

	if (*token != owner) {
		nvgpu_err(g, "requester 0x%08x NOT match owner 0x%08x",
			*token, owner);
		return -EINVAL;
	}

	if (--mutex->ref_cnt > 0) {
		return -EBUSY;
	}

	gk20a_writel(g, pwr_pmu_mutex_r(mutex->index),
		pwr_pmu_mutex_value_initial_lock_f());

	data = gk20a_readl(g, pwr_pmu_mutex_id_release_r());
	data = set_field(data, pwr_pmu_mutex_id_release_value_m(),
		pwr_pmu_mutex_id_release_value_f(owner));
	gk20a_writel(g, pwr_pmu_mutex_id_release_r(), data);

	gk20a_dbg_pmu(g, "mutex released: id=%d, token=0x%x",
		mutex->index, *token);

	return 0;
}

int gk20a_pmu_queue_head(struct gk20a *g, struct nvgpu_falcon_queue *queue,
			u32 *head, bool set)
{
	u32 queue_head_size = 0;

	if (g->ops.pmu.pmu_get_queue_head_size) {
		queue_head_size = g->ops.pmu.pmu_get_queue_head_size();
	}

	BUG_ON(!head || !queue_head_size);

	if (PMU_IS_COMMAND_QUEUE(queue->id)) {

		if (queue->index >= queue_head_size) {
			return -EINVAL;
		}

		if (!set) {
			*head = pwr_pmu_queue_head_address_v(
				gk20a_readl(g,
				g->ops.pmu.pmu_get_queue_head(queue->index)));
		} else {
			gk20a_writel(g,
				g->ops.pmu.pmu_get_queue_head(queue->index),
				pwr_pmu_queue_head_address_f(*head));
		}
	} else {
		if (!set) {
			*head = pwr_pmu_msgq_head_val_v(
				gk20a_readl(g, pwr_pmu_msgq_head_r()));
		} else {
			gk20a_writel(g,
				pwr_pmu_msgq_head_r(),
				pwr_pmu_msgq_head_val_f(*head));
		}
	}

	return 0;
}

int gk20a_pmu_queue_tail(struct gk20a *g, struct nvgpu_falcon_queue *queue,
			u32 *tail, bool set)
{
	u32 queue_tail_size = 0;

	if (g->ops.pmu.pmu_get_queue_tail_size) {
		queue_tail_size = g->ops.pmu.pmu_get_queue_tail_size();
	}

	BUG_ON(!tail || !queue_tail_size);

	if (PMU_IS_COMMAND_QUEUE(queue->id)) {

		if (queue->index >= queue_tail_size) {
			return -EINVAL;
		}

		if (!set) {
			*tail = pwr_pmu_queue_tail_address_v(gk20a_readl(g,
					g->ops.pmu.pmu_get_queue_tail(queue->index)));
		} else {
			gk20a_writel(g,
				g->ops.pmu.pmu_get_queue_tail(queue->index),
				pwr_pmu_queue_tail_address_f(*tail));
		}

	} else {
		if (!set) {
			*tail = pwr_pmu_msgq_tail_val_v(
				gk20a_readl(g, pwr_pmu_msgq_tail_r()));
		} else {
			gk20a_writel(g,
				pwr_pmu_msgq_tail_r(),
				pwr_pmu_msgq_tail_val_f(*tail));
		}
	}

	return 0;
}

void gk20a_pmu_msgq_tail(struct nvgpu_pmu *pmu, u32 *tail, bool set)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	u32 queue_tail_size = 0;

	if (g->ops.pmu.pmu_get_queue_tail_size) {
		queue_tail_size = g->ops.pmu.pmu_get_queue_tail_size();
	}

	BUG_ON(!tail || !queue_tail_size);

	if (!set) {
		*tail = pwr_pmu_msgq_tail_val_v(
			gk20a_readl(g, pwr_pmu_msgq_tail_r()));
	} else {
		gk20a_writel(g,
			pwr_pmu_msgq_tail_r(),
			pwr_pmu_msgq_tail_val_f(*tail));
	}
}

void gk20a_write_dmatrfbase(struct gk20a *g, u32 addr)
{
	gk20a_writel(g, pwr_falcon_dmatrfbase_r(), addr);
}

bool gk20a_pmu_is_engine_in_reset(struct gk20a *g)
{
	bool status = false;

	status = g->ops.mc.is_enabled(g, NVGPU_UNIT_PWR);

	return status;
}

int gk20a_pmu_engine_reset(struct gk20a *g, bool do_reset)
{
	u32 reset_mask = g->ops.mc.reset_mask(g, NVGPU_UNIT_PWR);

	if (do_reset) {
		g->ops.mc.enable(g, reset_mask);
	} else {
		g->ops.mc.disable(g, reset_mask);
	}

	return 0;
}

bool gk20a_is_pmu_supported(struct gk20a *g)
{
	return true;
}

u32 gk20a_pmu_pg_engines_list(struct gk20a *g)
{
	return BIT(PMU_PG_ELPG_ENGINE_ID_GRAPHICS);
}

u32 gk20a_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id)
{
	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
		return NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING;
	}

	return 0;
}

static void pmu_handle_zbc_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct nvgpu_pmu *pmu = param;
	gk20a_dbg_pmu(g, "reply ZBC_TABLE_UPDATE");
	pmu->zbc_save_done = 1;
}

void gk20a_pmu_save_zbc(struct gk20a *g, u32 entries)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	if (!pmu->pmu_ready || !entries || !pmu->zbc_ready) {
		return;
	}

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_zbc_cmd);
	cmd.cmd.zbc.cmd_type = g->pmu_ver_cmd_id_zbc_table_update;
	cmd.cmd.zbc.entry_mask = ZBC_MASK(entries);

	pmu->zbc_save_done = 0;

	gk20a_dbg_pmu(g, "cmd post ZBC_TABLE_UPDATE");
	nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			   pmu_handle_zbc_msg, pmu, &seq, ~0);
	pmu_wait_message_cond(pmu, gk20a_get_gr_idle_timeout(g),
			      &pmu->zbc_save_done, 1);
	if (!pmu->zbc_save_done) {
		nvgpu_err(g, "ZBC save timeout");
	}
}

int nvgpu_pmu_handle_therm_event(struct nvgpu_pmu *pmu,
			struct nv_pmu_therm_msg *msg)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	nvgpu_log_fn(g, " ");

	switch (msg->msg_type) {
	case NV_PMU_THERM_MSG_ID_EVENT_HW_SLOWDOWN_NOTIFICATION:
		if (msg->hw_slct_msg.mask == BIT(NV_PMU_THERM_EVENT_THERMAL_1)) {
			nvgpu_clk_arb_send_thermal_alarm(pmu->g);
		} else {
			gk20a_dbg_pmu(g, "Unwanted/Unregistered thermal event received %d",
				msg->hw_slct_msg.mask);
		}
		break;
	default:
		gk20a_dbg_pmu(g, "unkown therm event received %d", msg->msg_type);
		break;
	}

	return 0;
}

void gk20a_pmu_dump_elpg_stats(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	gk20a_dbg_pmu(g, "pwr_pmu_idle_mask_supp_r(3): 0x%08x",
		gk20a_readl(g, pwr_pmu_idle_mask_supp_r(3)));
	gk20a_dbg_pmu(g, "pwr_pmu_idle_mask_1_supp_r(3): 0x%08x",
		gk20a_readl(g, pwr_pmu_idle_mask_1_supp_r(3)));
	gk20a_dbg_pmu(g, "pwr_pmu_idle_ctrl_supp_r(3): 0x%08x",
		gk20a_readl(g, pwr_pmu_idle_ctrl_supp_r(3)));
	gk20a_dbg_pmu(g, "pwr_pmu_pg_idle_cnt_r(0): 0x%08x",
		gk20a_readl(g, pwr_pmu_pg_idle_cnt_r(0)));
	gk20a_dbg_pmu(g, "pwr_pmu_pg_intren_r(0): 0x%08x",
		gk20a_readl(g, pwr_pmu_pg_intren_r(0)));

	gk20a_dbg_pmu(g, "pwr_pmu_idle_count_r(3): 0x%08x",
		gk20a_readl(g, pwr_pmu_idle_count_r(3)));
	gk20a_dbg_pmu(g, "pwr_pmu_idle_count_r(4): 0x%08x",
		gk20a_readl(g, pwr_pmu_idle_count_r(4)));
	gk20a_dbg_pmu(g, "pwr_pmu_idle_count_r(7): 0x%08x",
		gk20a_readl(g, pwr_pmu_idle_count_r(7)));
}

void gk20a_pmu_dump_falcon_stats(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	unsigned int i;

	for (i = 0; i < pwr_pmu_mailbox__size_1_v(); i++) {
		nvgpu_err(g, "pwr_pmu_mailbox_r(%d) : 0x%x",
			i, gk20a_readl(g, pwr_pmu_mailbox_r(i)));
	}

	for (i = 0; i < pwr_pmu_debug__size_1_v(); i++) {
		nvgpu_err(g, "pwr_pmu_debug_r(%d) : 0x%x",
			i, gk20a_readl(g, pwr_pmu_debug_r(i)));
	}

	i = gk20a_readl(g, pwr_pmu_bar0_error_status_r());
	nvgpu_err(g, "pwr_pmu_bar0_error_status_r : 0x%x", i);
	if (i != 0) {
		nvgpu_err(g, "pwr_pmu_bar0_addr_r : 0x%x",
			gk20a_readl(g, pwr_pmu_bar0_addr_r()));
		nvgpu_err(g, "pwr_pmu_bar0_data_r : 0x%x",
			gk20a_readl(g, pwr_pmu_bar0_data_r()));
		nvgpu_err(g, "pwr_pmu_bar0_timeout_r : 0x%x",
			gk20a_readl(g, pwr_pmu_bar0_timeout_r()));
		nvgpu_err(g, "pwr_pmu_bar0_ctl_r : 0x%x",
			gk20a_readl(g, pwr_pmu_bar0_ctl_r()));
	}

	i = gk20a_readl(g, pwr_pmu_bar0_fecs_error_r());
	nvgpu_err(g, "pwr_pmu_bar0_fecs_error_r : 0x%x", i);

	i = gk20a_readl(g, pwr_falcon_exterrstat_r());
	nvgpu_err(g, "pwr_falcon_exterrstat_r : 0x%x", i);
	if (pwr_falcon_exterrstat_valid_v(i) ==
			pwr_falcon_exterrstat_valid_true_v()) {
		nvgpu_err(g, "pwr_falcon_exterraddr_r : 0x%x",
			gk20a_readl(g, pwr_falcon_exterraddr_r()));
	}

	/* Print PMU F/W debug prints */
	print_pmu_trace(pmu);
}

bool gk20a_pmu_is_interrupted(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	u32 servicedpmuint;

	servicedpmuint = pwr_falcon_irqstat_halt_true_f() |
			pwr_falcon_irqstat_exterr_true_f() |
			pwr_falcon_irqstat_swgen0_true_f();

	if (gk20a_readl(g, pwr_falcon_irqstat_r()) & servicedpmuint) {
		return true;
	}

	return false;
}

void gk20a_pmu_isr(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nvgpu_falcon_queue *queue;
	u32 intr, mask;
	bool recheck = false;

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&pmu->isr_mutex);
	if (!pmu->isr_enabled) {
		nvgpu_mutex_release(&pmu->isr_mutex);
		return;
	}

	mask = gk20a_readl(g, pwr_falcon_irqmask_r()) &
		gk20a_readl(g, pwr_falcon_irqdest_r());

	intr = gk20a_readl(g, pwr_falcon_irqstat_r());

	gk20a_dbg_pmu(g, "received falcon interrupt: 0x%08x", intr);

	intr = gk20a_readl(g, pwr_falcon_irqstat_r()) & mask;
	if (!intr || pmu->pmu_state == PMU_STATE_OFF) {
		gk20a_writel(g, pwr_falcon_irqsclr_r(), intr);
		nvgpu_mutex_release(&pmu->isr_mutex);
		return;
	}

	if (intr & pwr_falcon_irqstat_halt_true_f()) {
		nvgpu_err(g, "pmu halt intr not implemented");
		nvgpu_pmu_dump_falcon_stats(pmu);
		if (gk20a_readl(g, pwr_pmu_mailbox_r
				(PMU_MODE_MISMATCH_STATUS_MAILBOX_R)) ==
				PMU_MODE_MISMATCH_STATUS_VAL) {
			if (g->ops.pmu.dump_secure_fuses) {
				g->ops.pmu.dump_secure_fuses(g);
			}
		}
	}
	if (intr & pwr_falcon_irqstat_exterr_true_f()) {
		nvgpu_err(g,
			"pmu exterr intr not implemented. Clearing interrupt.");
		nvgpu_pmu_dump_falcon_stats(pmu);

		gk20a_writel(g, pwr_falcon_exterrstat_r(),
			gk20a_readl(g, pwr_falcon_exterrstat_r()) &
				~pwr_falcon_exterrstat_valid_m());
	}

	if (g->ops.pmu.handle_ext_irq) {
		g->ops.pmu.handle_ext_irq(g, intr);
	}

	if (intr & pwr_falcon_irqstat_swgen0_true_f()) {
		nvgpu_pmu_process_message(pmu);
		recheck = true;
	}

	gk20a_writel(g, pwr_falcon_irqsclr_r(), intr);

	if (recheck) {
		queue = &pmu->queue[PMU_MESSAGE_QUEUE];
		if (!nvgpu_flcn_queue_is_empty(pmu->flcn, queue)) {
			gk20a_writel(g, pwr_falcon_irqsset_r(),
				pwr_falcon_irqsset_swgen0_set_f());
		}
	}

	nvgpu_mutex_release(&pmu->isr_mutex);
}

void gk20a_pmu_init_perfmon_counter(struct gk20a *g)
{
	u32 data;

	/* use counter #3 for GR && CE2 busy cycles */
	gk20a_writel(g, pwr_pmu_idle_mask_r(3),
		pwr_pmu_idle_mask_gr_enabled_f() |
		pwr_pmu_idle_mask_ce_2_enabled_f());

	/* assign same mask setting from GR ELPG to counter #3 */
	data = gk20a_readl(g, pwr_pmu_idle_mask_1_supp_r(0));
	gk20a_writel(g, pwr_pmu_idle_mask_1_r(3), data);

	/* disable idle filtering for counters 3 and 6 */
	data = gk20a_readl(g, pwr_pmu_idle_ctrl_r(3));
	data = set_field(data, pwr_pmu_idle_ctrl_value_m() |
			pwr_pmu_idle_ctrl_filter_m(),
			pwr_pmu_idle_ctrl_value_busy_f() |
			pwr_pmu_idle_ctrl_filter_disabled_f());
	gk20a_writel(g, pwr_pmu_idle_ctrl_r(3), data);

	/* use counter #6 for total cycles */
	data = gk20a_readl(g, pwr_pmu_idle_ctrl_r(6));
	data = set_field(data, pwr_pmu_idle_ctrl_value_m() |
			pwr_pmu_idle_ctrl_filter_m(),
			pwr_pmu_idle_ctrl_value_always_f() |
			pwr_pmu_idle_ctrl_filter_disabled_f());
	gk20a_writel(g, pwr_pmu_idle_ctrl_r(6), data);

	/*
	 * We don't want to disturb counters #3 and #6, which are used by
	 * perfmon, so we add wiring also to counters #1 and #2 for
	 * exposing raw counter readings.
	 */
	gk20a_writel(g, pwr_pmu_idle_mask_r(1),
		pwr_pmu_idle_mask_gr_enabled_f() |
		pwr_pmu_idle_mask_ce_2_enabled_f());

	data = gk20a_readl(g, pwr_pmu_idle_ctrl_r(1));
	data = set_field(data, pwr_pmu_idle_ctrl_value_m() |
			pwr_pmu_idle_ctrl_filter_m(),
			pwr_pmu_idle_ctrl_value_busy_f() |
			pwr_pmu_idle_ctrl_filter_disabled_f());
	gk20a_writel(g, pwr_pmu_idle_ctrl_r(1), data);

	data = gk20a_readl(g, pwr_pmu_idle_ctrl_r(2));
	data = set_field(data, pwr_pmu_idle_ctrl_value_m() |
			pwr_pmu_idle_ctrl_filter_m(),
			pwr_pmu_idle_ctrl_value_always_f() |
			pwr_pmu_idle_ctrl_filter_disabled_f());
	gk20a_writel(g, pwr_pmu_idle_ctrl_r(2), data);

	/*
	 * use counters 4 and 0 for perfmon to log busy cycles and total cycles
	 * counter #0 overflow sets pmu idle intr status bit
	 */
	gk20a_writel(g, pwr_pmu_idle_intr_r(),
		     pwr_pmu_idle_intr_en_f(0));

	gk20a_writel(g, pwr_pmu_idle_threshold_r(0),
		     pwr_pmu_idle_threshold_value_f(0x7FFFFFFF));

	data = gk20a_readl(g, pwr_pmu_idle_ctrl_r(0));
	data = set_field(data, pwr_pmu_idle_ctrl_value_m() |
			pwr_pmu_idle_ctrl_filter_m(),
			pwr_pmu_idle_ctrl_value_always_f() |
			pwr_pmu_idle_ctrl_filter_disabled_f());
	gk20a_writel(g, pwr_pmu_idle_ctrl_r(0), data);

	gk20a_writel(g, pwr_pmu_idle_mask_r(4),
		pwr_pmu_idle_mask_gr_enabled_f() |
		pwr_pmu_idle_mask_ce_2_enabled_f());

	data = gk20a_readl(g, pwr_pmu_idle_ctrl_r(4));
	data = set_field(data, pwr_pmu_idle_ctrl_value_m() |
			pwr_pmu_idle_ctrl_filter_m(),
			pwr_pmu_idle_ctrl_value_busy_f() |
			pwr_pmu_idle_ctrl_filter_disabled_f());
	gk20a_writel(g, pwr_pmu_idle_ctrl_r(4), data);

	gk20a_writel(g, pwr_pmu_idle_count_r(0), pwr_pmu_idle_count_reset_f(1));
	gk20a_writel(g, pwr_pmu_idle_count_r(4), pwr_pmu_idle_count_reset_f(1));
	gk20a_writel(g, pwr_pmu_idle_intr_status_r(),
		     pwr_pmu_idle_intr_status_intr_f(1));
}

u32 gk20a_pmu_read_idle_counter(struct gk20a *g, u32 counter_id)
{
	return pwr_pmu_idle_count_value_v(
		gk20a_readl(g, pwr_pmu_idle_count_r(counter_id)));
}

void gk20a_pmu_reset_idle_counter(struct gk20a *g, u32 counter_id)
{
	gk20a_writel(g, pwr_pmu_idle_count_r(counter_id),
		pwr_pmu_idle_count_reset_f(1));
}

u32 gk20a_pmu_read_idle_intr_status(struct gk20a *g)
{
	return pwr_pmu_idle_intr_status_intr_v(
		gk20a_readl(g, pwr_pmu_idle_intr_status_r()));
}

void gk20a_pmu_clear_idle_intr_status(struct gk20a *g)
{
	gk20a_writel(g, pwr_pmu_idle_intr_status_r(),
		     pwr_pmu_idle_intr_status_intr_f(1));
}

void gk20a_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_pg_stats stats;

	nvgpu_flcn_copy_from_dmem(pmu->flcn,
		pmu->stat_dmem_offset[pg_engine_id],
		(u8 *)&stats, sizeof(struct pmu_pg_stats), 0);

	pg_stat_data->ingating_time = stats.pg_ingating_time_us;
	pg_stat_data->ungating_time = stats.pg_ungating_time_us;
	pg_stat_data->gating_cnt = stats.pg_gating_cnt;
	pg_stat_data->avg_entry_latency_us = stats.pg_avg_entry_time_us;
	pg_stat_data->avg_exit_latency_us = stats.pg_avg_exit_time_us;
}
