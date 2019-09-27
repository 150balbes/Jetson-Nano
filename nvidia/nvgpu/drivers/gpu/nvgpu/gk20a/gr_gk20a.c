/*
 * GK20A Graphics
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

#include <nvgpu/dma.h>
#include <nvgpu/kmem.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/timers.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/log.h>
#include <nvgpu/bsearch.h>
#include <nvgpu/sort.h>
#include <nvgpu/bug.h>
#include <nvgpu/firmware.h>
#include <nvgpu/enabled.h>
#include <nvgpu/debug.h>
#include <nvgpu/barrier.h>
#include <nvgpu/mm.h>
#include <nvgpu/ctxsw_trace.h>
#include <nvgpu/error_notifier.h>
#include <nvgpu/ecc.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/channel.h>
#include <nvgpu/unit.h>
#include <nvgpu/power_features/pg.h>
#include <nvgpu/power_features/cg.h>

#include "gk20a.h"
#include "gr_gk20a.h"
#include "gk20a/fecs_trace_gk20a.h"
#include "gr_ctx_gk20a.h"
#include "gr_pri_gk20a.h"
#include "regops_gk20a.h"
#include "dbg_gpu_gk20a.h"

#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ctxsw_prog_gk20a.h>
#include <nvgpu/hw/gk20a/hw_fifo_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gmmu_gk20a.h>
#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pri_ringmaster_gk20a.h>
#include <nvgpu/hw/gk20a/hw_top_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>

#define BLK_SIZE (256)
#define NV_PERF_PMM_FBP_ROUTER_STRIDE 0x0200
#define NV_PERF_PMMGPCROUTER_STRIDE	0x0200
#define NV_PCFG_BASE		0x00088000
#define NV_XBAR_MXBAR_PRI_GPC_GNIC_STRIDE	0x0020
#define FE_PWR_MODE_TIMEOUT_MAX 2000
#define FE_PWR_MODE_TIMEOUT_DEFAULT 10
#define CTXSW_MEM_SCRUBBING_TIMEOUT_MAX 1000
#define CTXSW_MEM_SCRUBBING_TIMEOUT_DEFAULT 10
#define FECS_ARB_CMD_TIMEOUT_MAX 40
#define FECS_ARB_CMD_TIMEOUT_DEFAULT 2

static int gk20a_init_gr_bind_fecs_elpg(struct gk20a *g);

static void gr_gk20a_free_channel_pm_ctx(struct gk20a *g,
					 struct vm_gk20a *vm,
					 struct nvgpu_gr_ctx *gr_ctx);

/* channel patch ctx buffer */
static int  gr_gk20a_alloc_channel_patch_ctx(struct gk20a *g,
					struct channel_gk20a *c);
static void gr_gk20a_free_channel_patch_ctx(struct gk20a *g,
					    struct vm_gk20a *vm,
					    struct nvgpu_gr_ctx *gr_ctx);

/* golden ctx image */
static int gr_gk20a_init_golden_ctx_image(struct gk20a *g,
					  struct channel_gk20a *c);

int gr_gk20a_get_ctx_id(struct gk20a *g,
		struct channel_gk20a *c,
		u32 *ctx_id)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx = NULL;
	struct nvgpu_mem *mem = NULL;

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	mem = &gr_ctx->mem;

	/* Channel gr_ctx buffer is gpu cacheable.
	   Flush and invalidate before cpu update. */
	g->ops.mm.l2_flush(g, true);

	*ctx_id = nvgpu_mem_rd(g, mem,
			ctxsw_prog_main_image_context_id_o());
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr, "ctx_id: 0x%x", *ctx_id);

	return 0;
}

void gk20a_fecs_dump_falcon_stats(struct gk20a *g)
{
	unsigned int i;

	nvgpu_err(g, "gr_fecs_os_r : %d",
		gk20a_readl(g, gr_fecs_os_r()));
	nvgpu_err(g, "gr_fecs_cpuctl_r : 0x%x",
		gk20a_readl(g, gr_fecs_cpuctl_r()));
	nvgpu_err(g, "gr_fecs_idlestate_r : 0x%x",
		gk20a_readl(g, gr_fecs_idlestate_r()));
	nvgpu_err(g, "gr_fecs_mailbox0_r : 0x%x",
		gk20a_readl(g, gr_fecs_mailbox0_r()));
	nvgpu_err(g, "gr_fecs_mailbox1_r : 0x%x",
		gk20a_readl(g, gr_fecs_mailbox1_r()));
	nvgpu_err(g, "gr_fecs_irqstat_r : 0x%x",
		gk20a_readl(g, gr_fecs_irqstat_r()));
	nvgpu_err(g, "gr_fecs_irqmode_r : 0x%x",
		gk20a_readl(g, gr_fecs_irqmode_r()));
	nvgpu_err(g, "gr_fecs_irqmask_r : 0x%x",
		gk20a_readl(g, gr_fecs_irqmask_r()));
	nvgpu_err(g, "gr_fecs_irqdest_r : 0x%x",
		gk20a_readl(g, gr_fecs_irqdest_r()));
	nvgpu_err(g, "gr_fecs_debug1_r : 0x%x",
		gk20a_readl(g, gr_fecs_debug1_r()));
	nvgpu_err(g, "gr_fecs_debuginfo_r : 0x%x",
		gk20a_readl(g, gr_fecs_debuginfo_r()));
	nvgpu_err(g, "gr_fecs_ctxsw_status_1_r : 0x%x",
		gk20a_readl(g, gr_fecs_ctxsw_status_1_r()));

	for (i = 0; i < g->ops.gr.fecs_ctxsw_mailbox_size(); i++) {
		nvgpu_err(g, "gr_fecs_ctxsw_mailbox_r(%d) : 0x%x",
			i, gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(i)));
	}

	nvgpu_err(g, "gr_fecs_engctl_r : 0x%x",
		gk20a_readl(g, gr_fecs_engctl_r()));
	nvgpu_err(g, "gr_fecs_curctx_r : 0x%x",
		gk20a_readl(g, gr_fecs_curctx_r()));
	nvgpu_err(g, "gr_fecs_nxtctx_r : 0x%x",
		gk20a_readl(g, gr_fecs_nxtctx_r()));

	gk20a_writel(g, gr_fecs_icd_cmd_r(),
		gr_fecs_icd_cmd_opc_rreg_f() |
		gr_fecs_icd_cmd_idx_f(PMU_FALCON_REG_IMB));
	nvgpu_err(g, "FECS_FALCON_REG_IMB : 0x%x",
		gk20a_readl(g, gr_fecs_icd_rdata_r()));

	gk20a_writel(g, gr_fecs_icd_cmd_r(),
		gr_fecs_icd_cmd_opc_rreg_f() |
		gr_fecs_icd_cmd_idx_f(PMU_FALCON_REG_DMB));
	nvgpu_err(g, "FECS_FALCON_REG_DMB : 0x%x",
		gk20a_readl(g, gr_fecs_icd_rdata_r()));

	gk20a_writel(g, gr_fecs_icd_cmd_r(),
		gr_fecs_icd_cmd_opc_rreg_f() |
		gr_fecs_icd_cmd_idx_f(PMU_FALCON_REG_CSW));
	nvgpu_err(g, "FECS_FALCON_REG_CSW : 0x%x",
		gk20a_readl(g, gr_fecs_icd_rdata_r()));

	gk20a_writel(g, gr_fecs_icd_cmd_r(),
		gr_fecs_icd_cmd_opc_rreg_f() |
		gr_fecs_icd_cmd_idx_f(PMU_FALCON_REG_CTX));
	nvgpu_err(g, "FECS_FALCON_REG_CTX : 0x%x",
		gk20a_readl(g, gr_fecs_icd_rdata_r()));

	gk20a_writel(g, gr_fecs_icd_cmd_r(),
		gr_fecs_icd_cmd_opc_rreg_f() |
		gr_fecs_icd_cmd_idx_f(PMU_FALCON_REG_EXCI));
	nvgpu_err(g, "FECS_FALCON_REG_EXCI : 0x%x",
		gk20a_readl(g, gr_fecs_icd_rdata_r()));

	for (i = 0; i < 4; i++) {
		gk20a_writel(g, gr_fecs_icd_cmd_r(),
			gr_fecs_icd_cmd_opc_rreg_f() |
			gr_fecs_icd_cmd_idx_f(PMU_FALCON_REG_PC));
		nvgpu_err(g, "FECS_FALCON_REG_PC : 0x%x",
			gk20a_readl(g, gr_fecs_icd_rdata_r()));

		gk20a_writel(g, gr_fecs_icd_cmd_r(),
			gr_fecs_icd_cmd_opc_rreg_f() |
			gr_fecs_icd_cmd_idx_f(PMU_FALCON_REG_SP));
		nvgpu_err(g, "FECS_FALCON_REG_SP : 0x%x",
			gk20a_readl(g, gr_fecs_icd_rdata_r()));
	}
}

static void gr_gk20a_load_falcon_dmem(struct gk20a *g)
{
	u32 i, ucode_u32_size;
	const u32 *ucode_u32_data;
	u32 checksum;

	nvgpu_log_fn(g, " ");

	gk20a_writel(g, gr_gpccs_dmemc_r(0), (gr_gpccs_dmemc_offs_f(0) |
					      gr_gpccs_dmemc_blk_f(0)  |
					      gr_gpccs_dmemc_aincw_f(1)));

	ucode_u32_size = g->gr.ctx_vars.ucode.gpccs.data.count;
	ucode_u32_data = (const u32 *)g->gr.ctx_vars.ucode.gpccs.data.l;

	for (i = 0, checksum = 0; i < ucode_u32_size; i++) {
		gk20a_writel(g, gr_gpccs_dmemd_r(0), ucode_u32_data[i]);
		checksum += ucode_u32_data[i];
	}

	gk20a_writel(g, gr_fecs_dmemc_r(0), (gr_fecs_dmemc_offs_f(0) |
					     gr_fecs_dmemc_blk_f(0)  |
					     gr_fecs_dmemc_aincw_f(1)));

	ucode_u32_size = g->gr.ctx_vars.ucode.fecs.data.count;
	ucode_u32_data = (const u32 *)g->gr.ctx_vars.ucode.fecs.data.l;

	for (i = 0, checksum = 0; i < ucode_u32_size; i++) {
		gk20a_writel(g, gr_fecs_dmemd_r(0), ucode_u32_data[i]);
		checksum += ucode_u32_data[i];
	}
	nvgpu_log_fn(g, "done");
}

static void gr_gk20a_load_falcon_imem(struct gk20a *g)
{
	u32 cfg, fecs_imem_size, gpccs_imem_size, ucode_u32_size;
	const u32 *ucode_u32_data;
	u32 tag, i, pad_start, pad_end;
	u32 checksum;

	nvgpu_log_fn(g, " ");

	cfg = gk20a_readl(g, gr_fecs_cfg_r());
	fecs_imem_size = gr_fecs_cfg_imem_sz_v(cfg);

	cfg = gk20a_readl(g, gr_gpc0_cfg_r());
	gpccs_imem_size = gr_gpc0_cfg_imem_sz_v(cfg);

	/* Use the broadcast address to access all of the GPCCS units. */
	gk20a_writel(g, gr_gpccs_imemc_r(0), (gr_gpccs_imemc_offs_f(0) |
					      gr_gpccs_imemc_blk_f(0) |
					      gr_gpccs_imemc_aincw_f(1)));

	/* Setup the tags for the instruction memory. */
	tag = 0;
	gk20a_writel(g, gr_gpccs_imemt_r(0), gr_gpccs_imemt_tag_f(tag));

	ucode_u32_size = g->gr.ctx_vars.ucode.gpccs.inst.count;
	ucode_u32_data = (const u32 *)g->gr.ctx_vars.ucode.gpccs.inst.l;

	for (i = 0, checksum = 0; i < ucode_u32_size; i++) {
		if ((i != 0U) && ((i % (256U/sizeof(u32))) == 0U)) {
			tag++;
			gk20a_writel(g, gr_gpccs_imemt_r(0),
				      gr_gpccs_imemt_tag_f(tag));
		}
		gk20a_writel(g, gr_gpccs_imemd_r(0), ucode_u32_data[i]);
		checksum += ucode_u32_data[i];
	}

	pad_start = i * 4U;
	pad_end = pad_start + (256U - pad_start % 256U) + 256U;
	for (i = pad_start;
	     (i < gpccs_imem_size * 256U) && (i < pad_end);
	     i += 4U) {
		if ((i != 0U) && ((i % 256U) == 0U)) {
			tag++;
			gk20a_writel(g, gr_gpccs_imemt_r(0),
				      gr_gpccs_imemt_tag_f(tag));
		}
		gk20a_writel(g, gr_gpccs_imemd_r(0), 0);
	}

	gk20a_writel(g, gr_fecs_imemc_r(0), (gr_fecs_imemc_offs_f(0) |
					     gr_fecs_imemc_blk_f(0) |
					     gr_fecs_imemc_aincw_f(1)));

	/* Setup the tags for the instruction memory. */
	tag = 0;
	gk20a_writel(g, gr_fecs_imemt_r(0), gr_fecs_imemt_tag_f(tag));

	ucode_u32_size = g->gr.ctx_vars.ucode.fecs.inst.count;
	ucode_u32_data = (const u32 *)g->gr.ctx_vars.ucode.fecs.inst.l;

	for (i = 0, checksum = 0; i < ucode_u32_size; i++) {
		if ((i != 0U) && ((i % (256U/sizeof(u32))) == 0U)) {
			tag++;
			gk20a_writel(g, gr_fecs_imemt_r(0),
				      gr_fecs_imemt_tag_f(tag));
		}
		gk20a_writel(g, gr_fecs_imemd_r(0), ucode_u32_data[i]);
		checksum += ucode_u32_data[i];
	}

	pad_start = i * 4U;
	pad_end = pad_start + (256U - pad_start % 256U) + 256U;
	for (i = pad_start;
	     (i < fecs_imem_size * 256U) && i < pad_end;
	     i += 4U) {
		if ((i != 0U) && ((i % 256U) == 0U)) {
			tag++;
			gk20a_writel(g, gr_fecs_imemt_r(0),
				      gr_fecs_imemt_tag_f(tag));
		}
		gk20a_writel(g, gr_fecs_imemd_r(0), 0);
	}
}

int gr_gk20a_wait_idle(struct gk20a *g, unsigned long duration_ms,
		       u32 expect_delay)
{
	u32 delay = expect_delay;
	bool ctxsw_active;
	bool gr_busy;
	u32 gr_engine_id;
	u32 engine_status;
	bool ctx_status_invalid;
	struct nvgpu_timeout timeout;

	nvgpu_log_fn(g, " ");

	gr_engine_id = gk20a_fifo_get_gr_engine_id(g);

	nvgpu_timeout_init(g, &timeout, duration_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		/* fmodel: host gets fifo_engine_status(gr) from gr
		   only when gr_status is read */
		(void) gk20a_readl(g, gr_status_r());

		engine_status = gk20a_readl(g,
					fifo_engine_status_r(gr_engine_id));

		ctxsw_active = engine_status &
			fifo_engine_status_ctxsw_in_progress_f();

		ctx_status_invalid =
			(fifo_engine_status_ctx_status_v(engine_status) ==
			 fifo_engine_status_ctx_status_invalid_v());

		gr_busy = gk20a_readl(g, gr_engine_status_r()) &
			gr_engine_status_value_busy_f();

		if (ctx_status_invalid || (!gr_busy && !ctxsw_active)) {
			nvgpu_log_fn(g, "done");
			return 0;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);

	} while (nvgpu_timeout_expired(&timeout) == 0);

	nvgpu_err(g,
		"timeout, ctxsw busy : %d, gr busy : %d",
		ctxsw_active, gr_busy);

	return -EAGAIN;
}

int gr_gk20a_wait_fe_idle(struct gk20a *g, unsigned long duration_ms,
			  u32 expect_delay)
{
	u32 val;
	u32 delay = expect_delay;
	struct nvgpu_timeout timeout;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		return 0;
	}

	nvgpu_log_fn(g, " ");

	nvgpu_timeout_init(g, &timeout, duration_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		val = gk20a_readl(g, gr_status_r());

		if (gr_status_fe_method_lower_v(val) == 0U) {
			nvgpu_log_fn(g, "done");
			return 0;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (nvgpu_timeout_expired(&timeout) == 0);

	nvgpu_err(g,
		"timeout, fe busy : %x", val);

	return -EAGAIN;
}

int gr_gk20a_ctx_wait_ucode(struct gk20a *g, u32 mailbox_id,
			    u32 *mailbox_ret, u32 opc_success,
			    u32 mailbox_ok, u32 opc_fail,
			    u32 mailbox_fail, bool sleepduringwait)
{
	struct nvgpu_timeout timeout;
	u32 delay = GR_FECS_POLL_INTERVAL;
	u32 check = WAIT_UCODE_LOOP;
	u32 reg;

	nvgpu_log_fn(g, " ");

	if (sleepduringwait) {
		delay = GR_IDLE_CHECK_DEFAULT;
	}

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	while (check == WAIT_UCODE_LOOP) {
		if (nvgpu_timeout_expired(&timeout)) {
			check = WAIT_UCODE_TIMEOUT;
		}

		reg = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(mailbox_id));

		if (mailbox_ret) {
			*mailbox_ret = reg;
		}

		switch (opc_success) {
		case GR_IS_UCODE_OP_EQUAL:
			if (reg == mailbox_ok) {
				check = WAIT_UCODE_OK;
			}
			break;
		case GR_IS_UCODE_OP_NOT_EQUAL:
			if (reg != mailbox_ok) {
				check = WAIT_UCODE_OK;
			}
			break;
		case GR_IS_UCODE_OP_AND:
			if (reg & mailbox_ok) {
				check = WAIT_UCODE_OK;
			}
			break;
		case GR_IS_UCODE_OP_LESSER:
			if (reg < mailbox_ok) {
				check = WAIT_UCODE_OK;
			}
			break;
		case GR_IS_UCODE_OP_LESSER_EQUAL:
			if (reg <= mailbox_ok) {
				check = WAIT_UCODE_OK;
			}
			break;
		case GR_IS_UCODE_OP_SKIP:
			/* do no success check */
			break;
		default:
			nvgpu_err(g,
				   "invalid success opcode 0x%x", opc_success);

			check = WAIT_UCODE_ERROR;
			break;
		}

		switch (opc_fail) {
		case GR_IS_UCODE_OP_EQUAL:
			if (reg == mailbox_fail) {
				check = WAIT_UCODE_ERROR;
			}
			break;
		case GR_IS_UCODE_OP_NOT_EQUAL:
			if (reg != mailbox_fail) {
				check = WAIT_UCODE_ERROR;
			}
			break;
		case GR_IS_UCODE_OP_AND:
			if (reg & mailbox_fail) {
				check = WAIT_UCODE_ERROR;
			}
			break;
		case GR_IS_UCODE_OP_LESSER:
			if (reg < mailbox_fail) {
				check = WAIT_UCODE_ERROR;
			}
			break;
		case GR_IS_UCODE_OP_LESSER_EQUAL:
			if (reg <= mailbox_fail) {
				check = WAIT_UCODE_ERROR;
			}
			break;
		case GR_IS_UCODE_OP_SKIP:
			/* do no check on fail*/
			break;
		default:
			nvgpu_err(g,
				   "invalid fail opcode 0x%x", opc_fail);
			check = WAIT_UCODE_ERROR;
			break;
		}

		if (sleepduringwait) {
			nvgpu_usleep_range(delay, delay * 2);
			delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
		} else {
			nvgpu_udelay(delay);
		}
	}

	if (check == WAIT_UCODE_TIMEOUT) {
		nvgpu_err(g,
			   "timeout waiting on mailbox=%d value=0x%08x",
			   mailbox_id, reg);
		gk20a_fecs_dump_falcon_stats(g);
		gk20a_gr_debug_dump(g);
		return -1;
	} else if (check == WAIT_UCODE_ERROR) {
		nvgpu_err(g,
			   "ucode method failed on mailbox=%d value=0x%08x",
			   mailbox_id, reg);
		gk20a_fecs_dump_falcon_stats(g);
		return -1;
	}

	nvgpu_log_fn(g, "done");
	return 0;
}

/* The following is a less brittle way to call gr_gk20a_submit_fecs_method(...)
 * We should replace most, if not all, fecs method calls to this instead. */
int gr_gk20a_submit_fecs_method_op(struct gk20a *g,
				   struct fecs_method_op_gk20a op,
				   bool sleepduringwait)
{
	struct gr_gk20a *gr = &g->gr;
	int ret;

	nvgpu_mutex_acquire(&gr->fecs_mutex);

	if (op.mailbox.id != 0) {
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(op.mailbox.id),
			     op.mailbox.data);
	}

	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0),
		gr_fecs_ctxsw_mailbox_clear_value_f(op.mailbox.clr));

	gk20a_writel(g, gr_fecs_method_data_r(), op.method.data);
	gk20a_writel(g, gr_fecs_method_push_r(),
		gr_fecs_method_push_adr_f(op.method.addr));

	/* op.mailbox.id == 4 cases require waiting for completion on
	 * for op.mailbox.id == 0 */
	if (op.mailbox.id == 4) {
		op.mailbox.id = 0;
	}

	ret = gr_gk20a_ctx_wait_ucode(g, op.mailbox.id, op.mailbox.ret,
				      op.cond.ok, op.mailbox.ok,
				      op.cond.fail, op.mailbox.fail,
				      sleepduringwait);
	if (ret) {
		nvgpu_err(g,"fecs method: data=0x%08x push adr=0x%08x",
			op.method.data, op.method.addr);
	}

	nvgpu_mutex_release(&gr->fecs_mutex);

	return ret;
}

/* Sideband mailbox writes are done a bit differently */
int gr_gk20a_submit_fecs_sideband_method_op(struct gk20a *g,
		struct fecs_method_op_gk20a op)
{
	struct gr_gk20a *gr = &g->gr;
	int ret;

	nvgpu_mutex_acquire(&gr->fecs_mutex);

	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(op.mailbox.id),
		gr_fecs_ctxsw_mailbox_clear_value_f(op.mailbox.clr));

	gk20a_writel(g, gr_fecs_method_data_r(), op.method.data);
	gk20a_writel(g, gr_fecs_method_push_r(),
		gr_fecs_method_push_adr_f(op.method.addr));

	ret = gr_gk20a_ctx_wait_ucode(g, op.mailbox.id, op.mailbox.ret,
				      op.cond.ok, op.mailbox.ok,
				      op.cond.fail, op.mailbox.fail,
				      false);
	if (ret) {
		nvgpu_err(g,"fecs method: data=0x%08x push adr=0x%08x",
			op.method.data, op.method.addr);
	}

	nvgpu_mutex_release(&gr->fecs_mutex);

	return ret;
}

static int gr_gk20a_ctrl_ctxsw(struct gk20a *g, u32 fecs_method, u32 *ret)
{
	return gr_gk20a_submit_fecs_method_op(g,
	      (struct fecs_method_op_gk20a) {
		      .method.addr = fecs_method,
		      .method.data = ~0,
		      .mailbox = { .id   = 1, /*sideband?*/
				   .data = ~0, .clr = ~0, .ret = ret,
				   .ok   = gr_fecs_ctxsw_mailbox_value_pass_v(),
				   .fail = gr_fecs_ctxsw_mailbox_value_fail_v(), },
		      .cond.ok = GR_IS_UCODE_OP_EQUAL,
		      .cond.fail = GR_IS_UCODE_OP_EQUAL }, true);
}

/**
 * Stop processing (stall) context switches at FECS:-
 * If fecs is sent stop_ctxsw method, elpg entry/exit cannot happen
 * and may timeout. It could manifest as different error signatures
 * depending on when stop_ctxsw fecs method gets sent with respect
 * to pmu elpg sequence. It could come as pmu halt or abort or
 * maybe ext error too.
*/
int gr_gk20a_disable_ctxsw(struct gk20a *g)
{
	int err = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");

	nvgpu_mutex_acquire(&g->ctxsw_disable_lock);
	g->ctxsw_disable_count++;
	if (g->ctxsw_disable_count == 1) {
		err = nvgpu_pg_elpg_disable(g);
		if (err != 0) {
			nvgpu_err(g, "failed to disable elpg. not safe to "
					"stop_ctxsw");
			/* stop ctxsw command is not sent */
			g->ctxsw_disable_count--;
		} else {
			err = gr_gk20a_ctrl_ctxsw(g,
				gr_fecs_method_push_adr_stop_ctxsw_v(), NULL);
			if (err != 0) {
				nvgpu_err(g, "failed to stop fecs ctxsw");
				/* stop ctxsw failed */
				g->ctxsw_disable_count--;
			}
		}
	} else {
		nvgpu_log_info(g, "ctxsw disabled, ctxsw_disable_count: %d",
			g->ctxsw_disable_count);
	}
	nvgpu_mutex_release(&g->ctxsw_disable_lock);

	return err;
}

/* Start processing (continue) context switches at FECS */
int gr_gk20a_enable_ctxsw(struct gk20a *g)
{
	int err = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");

	nvgpu_mutex_acquire(&g->ctxsw_disable_lock);

	if (g->ctxsw_disable_count == 0) {
		goto ctxsw_already_enabled;
	}
	g->ctxsw_disable_count--;
	WARN_ON(g->ctxsw_disable_count < 0);
	if (g->ctxsw_disable_count == 0) {
		err = gr_gk20a_ctrl_ctxsw(g,
				gr_fecs_method_push_adr_start_ctxsw_v(), NULL);
		if (err != 0) {
			nvgpu_err(g, "failed to start fecs ctxsw");
		} else {
			if (nvgpu_pg_elpg_enable(g) != 0) {
				nvgpu_err(g, "failed to enable elpg "
					"after start_ctxsw");
			}
		}
	} else {
		nvgpu_log_info(g, "ctxsw_disable_count: %d is not 0 yet",
			g->ctxsw_disable_count);
	}
ctxsw_already_enabled:
	nvgpu_mutex_release(&g->ctxsw_disable_lock);

	return err;
}

int gr_gk20a_halt_pipe(struct gk20a *g)
{
	return gr_gk20a_submit_fecs_method_op(g,
	      (struct fecs_method_op_gk20a) {
		      .method.addr =
				gr_fecs_method_push_adr_halt_pipeline_v(),
		      .method.data = ~0,
		      .mailbox = { .id   = 1, /*sideband?*/
				.data = ~0, .clr = ~0, .ret = NULL,
				.ok   = gr_fecs_ctxsw_mailbox_value_pass_v(),
				.fail = gr_fecs_ctxsw_mailbox_value_fail_v(), },
		      .cond.ok = GR_IS_UCODE_OP_EQUAL,
		      .cond.fail = GR_IS_UCODE_OP_EQUAL }, false);
}


int gr_gk20a_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	u32 addr_lo;
	u32 addr_hi;

	nvgpu_log_fn(c->g, " ");

	addr_lo = u64_lo32(gpu_va) >> 12;
	addr_hi = u64_hi32(gpu_va);

	nvgpu_mem_wr32(c->g, &c->inst_block, ram_in_gr_wfi_target_w(),
		 ram_in_gr_cs_wfi_f() | ram_in_gr_wfi_mode_virtual_f() |
		 ram_in_gr_wfi_ptr_lo_f(addr_lo));

	nvgpu_mem_wr32(c->g, &c->inst_block, ram_in_gr_wfi_ptr_hi_w(),
		 ram_in_gr_wfi_ptr_hi_f(addr_hi));

	return 0;
}

/*
 * Context state can be written directly, or "patched" at times. So that code
 * can be used in either situation it is written using a series of
 * _ctx_patch_write(..., patch) statements. However any necessary map overhead
 * should be minimized; thus, bundle the sequence of these writes together, and
 * set them up and close with _ctx_patch_write_begin/_ctx_patch_write_end.
 */

int gr_gk20a_ctx_patch_write_begin(struct gk20a *g,
					  struct nvgpu_gr_ctx *gr_ctx,
					  bool update_patch_count)
{
	if (update_patch_count) {
		/* reset patch count if ucode has already processed it */
		gr_ctx->patch_ctx.data_count = nvgpu_mem_rd(g,
						&gr_ctx->mem,
					ctxsw_prog_main_image_patch_count_o());
		nvgpu_log(g, gpu_dbg_info, "patch count reset to %d",
					gr_ctx->patch_ctx.data_count);
	}
	return 0;
}

void gr_gk20a_ctx_patch_write_end(struct gk20a *g,
					struct nvgpu_gr_ctx *gr_ctx,
					bool update_patch_count)
{
	/* Write context count to context image if it is mapped */
	if (update_patch_count) {
		nvgpu_mem_wr(g, &gr_ctx->mem,
			     ctxsw_prog_main_image_patch_count_o(),
			     gr_ctx->patch_ctx.data_count);
		nvgpu_log(g, gpu_dbg_info, "write patch count %d",
			gr_ctx->patch_ctx.data_count);
	}
}

void gr_gk20a_ctx_patch_write(struct gk20a *g,
				    struct nvgpu_gr_ctx *gr_ctx,
				    u32 addr, u32 data, bool patch)
{
	if (patch) {
		u32 patch_slot = gr_ctx->patch_ctx.data_count *
				PATCH_CTX_SLOTS_REQUIRED_PER_ENTRY;
		if (patch_slot > (PATCH_CTX_ENTRIES_FROM_SIZE(
					gr_ctx->patch_ctx.mem.size) -
				PATCH_CTX_SLOTS_REQUIRED_PER_ENTRY)) {
			nvgpu_err(g, "failed to access patch_slot %d",
				patch_slot);
			return;
		}
		nvgpu_mem_wr32(g, &gr_ctx->patch_ctx.mem, patch_slot, addr);
		nvgpu_mem_wr32(g, &gr_ctx->patch_ctx.mem, patch_slot + 1, data);
		gr_ctx->patch_ctx.data_count++;
		nvgpu_log(g, gpu_dbg_info,
			"patch addr = 0x%x data = 0x%x data_count %d",
			addr, data, gr_ctx->patch_ctx.data_count);
	} else {
		gk20a_writel(g, addr, data);
	}
}

static u32 fecs_current_ctx_data(struct gk20a *g, struct nvgpu_mem *inst_block)
{
	u64 ptr = nvgpu_inst_block_addr(g, inst_block) >>
		ram_in_base_shift_v();
	u32 aperture = nvgpu_aperture_mask(g, inst_block,
				gr_fecs_current_ctx_target_sys_mem_ncoh_f(),
				gr_fecs_current_ctx_target_sys_mem_coh_f(),
				gr_fecs_current_ctx_target_vid_mem_f());

	return gr_fecs_current_ctx_ptr_f(u64_lo32(ptr)) | aperture |
		gr_fecs_current_ctx_valid_f(1);
}

int gr_gk20a_fecs_ctx_bind_channel(struct gk20a *g,
					struct channel_gk20a *c)
{
	u32 inst_base_ptr = u64_lo32(nvgpu_inst_block_addr(g, &c->inst_block)
				     >> ram_in_base_shift_v());
	u32 data = fecs_current_ctx_data(g, &c->inst_block);
	u32 ret;

	nvgpu_log_info(g, "bind channel %d inst ptr 0x%08x",
		   c->chid, inst_base_ptr);

	ret = gr_gk20a_submit_fecs_method_op(g,
		     (struct fecs_method_op_gk20a) {
		     .method.addr = gr_fecs_method_push_adr_bind_pointer_v(),
		     .method.data = data,
		     .mailbox = { .id = 0, .data = 0,
				  .clr = 0x30,
				  .ret = NULL,
				  .ok = 0x10,
				  .fail = 0x20, },
		     .cond.ok = GR_IS_UCODE_OP_AND,
		     .cond.fail = GR_IS_UCODE_OP_AND}, true);
	if (ret) {
		nvgpu_err(g,
			"bind channel instance failed");
	}

	return ret;
}

void gr_gk20a_write_zcull_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 va = u64_lo32(gpu_va >> 8);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_zcull_ptr_o(), va);
}

void gr_gk20a_write_pm_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 va = u64_lo32(gpu_va >> 8);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_pm_ptr_o(), va);
}

static int gr_gk20a_ctx_zcull_setup(struct gk20a *g, struct channel_gk20a *c)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx = NULL;
	struct nvgpu_mem *mem = NULL;
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	int ret = 0;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	mem = &gr_ctx->mem;

	if (gr_ctx->zcull_ctx.gpu_va == 0 &&
	    gr_ctx->zcull_ctx.ctx_sw_mode ==
		ctxsw_prog_main_image_zcull_mode_separate_buffer_v()) {
		return -EINVAL;
	}

	ret = gk20a_disable_channel_tsg(g, c);
	if (ret) {
		nvgpu_err(g, "failed to disable channel/TSG");
		return ret;
	}
	ret = gk20a_fifo_preempt(g, c);
	if (ret) {
		gk20a_enable_channel_tsg(g, c);
		nvgpu_err(g, "failed to preempt channel/TSG");
		return ret;
	}

	nvgpu_mem_wr(g, mem,
			ctxsw_prog_main_image_zcull_o(),
		 gr_ctx->zcull_ctx.ctx_sw_mode);

	if (ctxheader->gpu_va) {
		g->ops.gr.write_zcull_ptr(g, ctxheader,
					gr_ctx->zcull_ctx.gpu_va);
	} else {
		g->ops.gr.write_zcull_ptr(g, mem, gr_ctx->zcull_ctx.gpu_va);
	}

	gk20a_enable_channel_tsg(g, c);

	return ret;
}

u32 gk20a_gr_gpc_offset(struct gk20a *g, u32 gpc)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 gpc_offset = gpc_stride * gpc;

	return gpc_offset;
}

u32 gk20a_gr_tpc_offset(struct gk20a *g, u32 tpc)
{
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 tpc_offset = tpc_in_gpc_stride * tpc;

	return tpc_offset;
}

int gr_gk20a_commit_global_ctx_buffers(struct gk20a *g,
			struct channel_gk20a *c, bool patch)
{
	struct gr_gk20a *gr = &g->gr;
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx = NULL;
	u64 addr;
	u32 size;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	if (patch) {
		int err;
		err = gr_gk20a_ctx_patch_write_begin(g, gr_ctx, false);
		if (err != 0) {
			return err;
		}
	}

	/* global pagepool buffer */
	addr = (u64_lo32(gr_ctx->global_ctx_buffer_va[PAGEPOOL_VA]) >>
		gr_scc_pagepool_base_addr_39_8_align_bits_v()) |
		(u64_hi32(gr_ctx->global_ctx_buffer_va[PAGEPOOL_VA]) <<
		 (32 - gr_scc_pagepool_base_addr_39_8_align_bits_v()));

	size = gr->global_ctx_buffer[PAGEPOOL].mem.size /
		gr_scc_pagepool_total_pages_byte_granularity_v();

	if (size == g->ops.gr.pagepool_default_size(g)) {
		size = gr_scc_pagepool_total_pages_hwmax_v();
	}

	nvgpu_log_info(g, "pagepool buffer addr : 0x%016llx, size : %d",
		addr, size);

	g->ops.gr.commit_global_pagepool(g, gr_ctx, addr, size, patch);

	/* global bundle cb */
	addr = (u64_lo32(gr_ctx->global_ctx_buffer_va[CIRCULAR_VA]) >>
		gr_scc_bundle_cb_base_addr_39_8_align_bits_v()) |
		(u64_hi32(gr_ctx->global_ctx_buffer_va[CIRCULAR_VA]) <<
		 (32 - gr_scc_bundle_cb_base_addr_39_8_align_bits_v()));

	size = gr->bundle_cb_default_size;

	nvgpu_log_info(g, "bundle cb addr : 0x%016llx, size : %d",
		addr, size);

	g->ops.gr.commit_global_bundle_cb(g, gr_ctx, addr, size, patch);

	/* global attrib cb */
	addr = (u64_lo32(gr_ctx->global_ctx_buffer_va[ATTRIBUTE_VA]) >>
		gr_gpcs_setup_attrib_cb_base_addr_39_12_align_bits_v()) |
		(u64_hi32(gr_ctx->global_ctx_buffer_va[ATTRIBUTE_VA]) <<
		 (32 - gr_gpcs_setup_attrib_cb_base_addr_39_12_align_bits_v()));

	nvgpu_log_info(g, "attrib cb addr : 0x%016llx", addr);
	g->ops.gr.commit_global_attrib_cb(g, gr_ctx, addr, patch);
	g->ops.gr.commit_global_cb_manager(g, c, patch);

	if (patch) {
		gr_gk20a_ctx_patch_write_end(g, gr_ctx, false);
	}

	return 0;
}

int gr_gk20a_commit_global_timeslice(struct gk20a *g, struct channel_gk20a *c)
{
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_gr_ctx *gr_ctx = NULL;
	u32 gpm_pd_cfg;
	u32 pd_ab_dist_cfg0;
	u32 ds_debug;
	u32 mpc_vtg_debug;
	u32 pe_vaf;
	u32 pe_vsc_vpc;

	nvgpu_log_fn(g, " ");

	gpm_pd_cfg = gk20a_readl(g, gr_gpcs_gpm_pd_cfg_r());
	pd_ab_dist_cfg0 = gk20a_readl(g, gr_pd_ab_dist_cfg0_r());
	ds_debug = gk20a_readl(g, gr_ds_debug_r());
	mpc_vtg_debug = gk20a_readl(g, gr_gpcs_tpcs_mpc_vtg_debug_r());

	if (gr->timeslice_mode == gr_gpcs_ppcs_cbm_cfg_timeslice_mode_enable_v()) {
		pe_vaf = gk20a_readl(g, gr_gpcs_tpcs_pe_vaf_r());
		pe_vsc_vpc = gk20a_readl(g, gr_gpcs_tpcs_pes_vsc_vpc_r());

		gpm_pd_cfg = gr_gpcs_gpm_pd_cfg_timeslice_mode_enable_f() | gpm_pd_cfg;
		pe_vaf = gr_gpcs_tpcs_pe_vaf_fast_mode_switch_true_f() | pe_vaf;
		pe_vsc_vpc = gr_gpcs_tpcs_pes_vsc_vpc_fast_mode_switch_true_f() | pe_vsc_vpc;
		pd_ab_dist_cfg0 = gr_pd_ab_dist_cfg0_timeslice_enable_en_f() | pd_ab_dist_cfg0;
		ds_debug = gr_ds_debug_timeslice_mode_enable_f() | ds_debug;
		mpc_vtg_debug = gr_gpcs_tpcs_mpc_vtg_debug_timeslice_mode_enabled_f() | mpc_vtg_debug;

		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_gpm_pd_cfg_r(), gpm_pd_cfg, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_tpcs_pe_vaf_r(), pe_vaf, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_tpcs_pes_vsc_vpc_r(), pe_vsc_vpc, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_pd_ab_dist_cfg0_r(), pd_ab_dist_cfg0, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_ds_debug_r(), ds_debug, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_tpcs_mpc_vtg_debug_r(), mpc_vtg_debug, false);
	} else {
		gpm_pd_cfg = gr_gpcs_gpm_pd_cfg_timeslice_mode_disable_f() | gpm_pd_cfg;
		pd_ab_dist_cfg0 = gr_pd_ab_dist_cfg0_timeslice_enable_dis_f() | pd_ab_dist_cfg0;
		ds_debug = gr_ds_debug_timeslice_mode_disable_f() | ds_debug;
		mpc_vtg_debug = gr_gpcs_tpcs_mpc_vtg_debug_timeslice_mode_disabled_f() | mpc_vtg_debug;

		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_gpm_pd_cfg_r(), gpm_pd_cfg, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_pd_ab_dist_cfg0_r(), pd_ab_dist_cfg0, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_ds_debug_r(), ds_debug, false);
		gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_tpcs_mpc_vtg_debug_r(), mpc_vtg_debug, false);
	}

	return 0;
}

/*
 * Return map tiles count for given index
 * Return 0 if index is out-of-bounds
 */
static u32 gr_gk20a_get_map_tile_count(struct gr_gk20a *gr, u32 index)
{
	if (index >= gr->map_tile_count) {
		return 0;
	}

	return gr->map_tiles[index];
}

int gr_gk20a_setup_rop_mapping(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 norm_entries, norm_shift;
	u32 coeff5_mod, coeff6_mod, coeff7_mod, coeff8_mod, coeff9_mod, coeff10_mod, coeff11_mod;
	u32 map0, map1, map2, map3, map4, map5;

	if (gr->map_tiles == NULL) {
		return -1;
	}

	nvgpu_log_fn(g, " ");

	gk20a_writel(g, gr_crstr_map_table_cfg_r(),
		     gr_crstr_map_table_cfg_row_offset_f(gr->map_row_offset) |
		     gr_crstr_map_table_cfg_num_entries_f(gr->tpc_count));

	map0 =  gr_crstr_gpc_map0_tile0_f(gr_gk20a_get_map_tile_count(gr, 0)) |
		gr_crstr_gpc_map0_tile1_f(gr_gk20a_get_map_tile_count(gr, 1)) |
		gr_crstr_gpc_map0_tile2_f(gr_gk20a_get_map_tile_count(gr, 2)) |
		gr_crstr_gpc_map0_tile3_f(gr_gk20a_get_map_tile_count(gr, 3)) |
		gr_crstr_gpc_map0_tile4_f(gr_gk20a_get_map_tile_count(gr, 4)) |
		gr_crstr_gpc_map0_tile5_f(gr_gk20a_get_map_tile_count(gr, 5));

	map1 =  gr_crstr_gpc_map1_tile6_f(gr_gk20a_get_map_tile_count(gr, 6)) |
		gr_crstr_gpc_map1_tile7_f(gr_gk20a_get_map_tile_count(gr, 7)) |
		gr_crstr_gpc_map1_tile8_f(gr_gk20a_get_map_tile_count(gr, 8)) |
		gr_crstr_gpc_map1_tile9_f(gr_gk20a_get_map_tile_count(gr, 9)) |
		gr_crstr_gpc_map1_tile10_f(gr_gk20a_get_map_tile_count(gr, 10)) |
		gr_crstr_gpc_map1_tile11_f(gr_gk20a_get_map_tile_count(gr, 11));

	map2 =  gr_crstr_gpc_map2_tile12_f(gr_gk20a_get_map_tile_count(gr, 12)) |
		gr_crstr_gpc_map2_tile13_f(gr_gk20a_get_map_tile_count(gr, 13)) |
		gr_crstr_gpc_map2_tile14_f(gr_gk20a_get_map_tile_count(gr, 14)) |
		gr_crstr_gpc_map2_tile15_f(gr_gk20a_get_map_tile_count(gr, 15)) |
		gr_crstr_gpc_map2_tile16_f(gr_gk20a_get_map_tile_count(gr, 16)) |
		gr_crstr_gpc_map2_tile17_f(gr_gk20a_get_map_tile_count(gr, 17));

	map3 =  gr_crstr_gpc_map3_tile18_f(gr_gk20a_get_map_tile_count(gr, 18)) |
		gr_crstr_gpc_map3_tile19_f(gr_gk20a_get_map_tile_count(gr, 19)) |
		gr_crstr_gpc_map3_tile20_f(gr_gk20a_get_map_tile_count(gr, 20)) |
		gr_crstr_gpc_map3_tile21_f(gr_gk20a_get_map_tile_count(gr, 21)) |
		gr_crstr_gpc_map3_tile22_f(gr_gk20a_get_map_tile_count(gr, 22)) |
		gr_crstr_gpc_map3_tile23_f(gr_gk20a_get_map_tile_count(gr, 23));

	map4 =  gr_crstr_gpc_map4_tile24_f(gr_gk20a_get_map_tile_count(gr, 24)) |
		gr_crstr_gpc_map4_tile25_f(gr_gk20a_get_map_tile_count(gr, 25)) |
		gr_crstr_gpc_map4_tile26_f(gr_gk20a_get_map_tile_count(gr, 26)) |
		gr_crstr_gpc_map4_tile27_f(gr_gk20a_get_map_tile_count(gr, 27)) |
		gr_crstr_gpc_map4_tile28_f(gr_gk20a_get_map_tile_count(gr, 28)) |
		gr_crstr_gpc_map4_tile29_f(gr_gk20a_get_map_tile_count(gr, 29));

	map5 =  gr_crstr_gpc_map5_tile30_f(gr_gk20a_get_map_tile_count(gr, 30)) |
		gr_crstr_gpc_map5_tile31_f(gr_gk20a_get_map_tile_count(gr, 31)) |
		gr_crstr_gpc_map5_tile32_f(0) |
		gr_crstr_gpc_map5_tile33_f(0) |
		gr_crstr_gpc_map5_tile34_f(0) |
		gr_crstr_gpc_map5_tile35_f(0);

	gk20a_writel(g, gr_crstr_gpc_map0_r(), map0);
	gk20a_writel(g, gr_crstr_gpc_map1_r(), map1);
	gk20a_writel(g, gr_crstr_gpc_map2_r(), map2);
	gk20a_writel(g, gr_crstr_gpc_map3_r(), map3);
	gk20a_writel(g, gr_crstr_gpc_map4_r(), map4);
	gk20a_writel(g, gr_crstr_gpc_map5_r(), map5);

	switch (gr->tpc_count) {
	case 1:
		norm_shift = 4;
		break;
	case 2:
	case 3:
		norm_shift = 3;
		break;
	case 4:
	case 5:
	case 6:
	case 7:
		norm_shift = 2;
		break;
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		norm_shift = 1;
		break;
	default:
		norm_shift = 0;
		break;
	}

	norm_entries = gr->tpc_count << norm_shift;
	coeff5_mod = (1 << 5) % norm_entries;
	coeff6_mod = (1 << 6) % norm_entries;
	coeff7_mod = (1 << 7) % norm_entries;
	coeff8_mod = (1 << 8) % norm_entries;
	coeff9_mod = (1 << 9) % norm_entries;
	coeff10_mod = (1 << 10) % norm_entries;
	coeff11_mod = (1 << 11) % norm_entries;

	gk20a_writel(g, gr_ppcs_wwdx_map_table_cfg_r(),
		     gr_ppcs_wwdx_map_table_cfg_row_offset_f(gr->map_row_offset) |
		     gr_ppcs_wwdx_map_table_cfg_normalized_num_entries_f(norm_entries) |
		     gr_ppcs_wwdx_map_table_cfg_normalized_shift_value_f(norm_shift) |
		     gr_ppcs_wwdx_map_table_cfg_coeff5_mod_value_f(coeff5_mod) |
		     gr_ppcs_wwdx_map_table_cfg_num_entries_f(gr->tpc_count));

	gk20a_writel(g, gr_ppcs_wwdx_map_table_cfg2_r(),
		     gr_ppcs_wwdx_map_table_cfg2_coeff6_mod_value_f(coeff6_mod) |
		     gr_ppcs_wwdx_map_table_cfg2_coeff7_mod_value_f(coeff7_mod) |
		     gr_ppcs_wwdx_map_table_cfg2_coeff8_mod_value_f(coeff8_mod) |
		     gr_ppcs_wwdx_map_table_cfg2_coeff9_mod_value_f(coeff9_mod) |
		     gr_ppcs_wwdx_map_table_cfg2_coeff10_mod_value_f(coeff10_mod) |
		     gr_ppcs_wwdx_map_table_cfg2_coeff11_mod_value_f(coeff11_mod));

	gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map0_r(), map0);
	gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map1_r(), map1);
	gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map2_r(), map2);
	gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map3_r(), map3);
	gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map4_r(), map4);
	gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map5_r(), map5);

	gk20a_writel(g, gr_rstr2d_map_table_cfg_r(),
		     gr_rstr2d_map_table_cfg_row_offset_f(gr->map_row_offset) |
		     gr_rstr2d_map_table_cfg_num_entries_f(gr->tpc_count));

	gk20a_writel(g, gr_rstr2d_gpc_map0_r(), map0);
	gk20a_writel(g, gr_rstr2d_gpc_map1_r(), map1);
	gk20a_writel(g, gr_rstr2d_gpc_map2_r(), map2);
	gk20a_writel(g, gr_rstr2d_gpc_map3_r(), map3);
	gk20a_writel(g, gr_rstr2d_gpc_map4_r(), map4);
	gk20a_writel(g, gr_rstr2d_gpc_map5_r(), map5);

	return 0;
}

static inline u32 count_bits(u32 mask)
{
	u32 temp = mask;
	u32 count;
	for (count = 0; temp != 0; count++) {
		temp &= temp - 1;
	}

	return count;
}

int gr_gk20a_init_sm_id_table(struct gk20a *g)
{
	u32 gpc, tpc;
	u32 sm_id = 0;

	for (tpc = 0; tpc < g->gr.max_tpc_per_gpc_count; tpc++) {
		for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {

			if (tpc < g->gr.gpc_tpc_count[gpc]) {
				g->gr.sm_to_cluster[sm_id].tpc_index = tpc;
				g->gr.sm_to_cluster[sm_id].gpc_index = gpc;
				g->gr.sm_to_cluster[sm_id].sm_index = 0;
				g->gr.sm_to_cluster[sm_id].global_tpc_index =
									sm_id;
				sm_id++;
			}
		}
	}
	g->gr.no_of_sm = sm_id;
	return 0;
}

/*
 * Return number of TPCs in a GPC
 * Return 0 if GPC index is invalid i.e. GPC is disabled
 */
u32 gr_gk20a_get_tpc_count(struct gr_gk20a *gr, u32 gpc_index)
{
	if (gpc_index >= gr->gpc_count) {
		return 0;
	}

	return gr->gpc_tpc_count[gpc_index];
}

int gr_gk20a_init_fs_state(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_index, gpc_index;
	u32 sm_id = 0, gpc_id = 0;
	u32 tpc_per_gpc;
	u32 fuse_tpc_mask;
	u32 reg_index;
	int err;

	nvgpu_log_fn(g, " ");

	if (g->ops.gr.init_sm_id_table) {
		err = g->ops.gr.init_sm_id_table(g);
		if (err != 0) {
			return err;
		}

		/* Is table empty ? */
		if (g->gr.no_of_sm == 0) {
			return -EINVAL;
		}
	}

	for (sm_id = 0; sm_id < g->gr.no_of_sm; sm_id++) {
		tpc_index = g->gr.sm_to_cluster[sm_id].tpc_index;
		gpc_index = g->gr.sm_to_cluster[sm_id].gpc_index;

		g->ops.gr.program_sm_id_numbering(g, gpc_index, tpc_index, sm_id);

		if (g->ops.gr.program_active_tpc_counts) {
			g->ops.gr.program_active_tpc_counts(g, gpc_index);
		}
	}

	for (reg_index = 0, gpc_id = 0;
	     reg_index < gr_pd_num_tpc_per_gpc__size_1_v();
	     reg_index++, gpc_id += 8) {

		tpc_per_gpc =
			gr_pd_num_tpc_per_gpc_count0_f(gr_gk20a_get_tpc_count(gr, gpc_id + 0)) |
			gr_pd_num_tpc_per_gpc_count1_f(gr_gk20a_get_tpc_count(gr, gpc_id + 1)) |
			gr_pd_num_tpc_per_gpc_count2_f(gr_gk20a_get_tpc_count(gr, gpc_id + 2)) |
			gr_pd_num_tpc_per_gpc_count3_f(gr_gk20a_get_tpc_count(gr, gpc_id + 3)) |
			gr_pd_num_tpc_per_gpc_count4_f(gr_gk20a_get_tpc_count(gr, gpc_id + 4)) |
			gr_pd_num_tpc_per_gpc_count5_f(gr_gk20a_get_tpc_count(gr, gpc_id + 5)) |
			gr_pd_num_tpc_per_gpc_count6_f(gr_gk20a_get_tpc_count(gr, gpc_id + 6)) |
			gr_pd_num_tpc_per_gpc_count7_f(gr_gk20a_get_tpc_count(gr, gpc_id + 7));

		gk20a_writel(g, gr_pd_num_tpc_per_gpc_r(reg_index), tpc_per_gpc);
		gk20a_writel(g, gr_ds_num_tpc_per_gpc_r(reg_index), tpc_per_gpc);
	}

	/* gr__setup_pd_mapping stubbed for gk20a */
	g->ops.gr.setup_rop_mapping(g, gr);
	if (g->ops.gr.setup_alpha_beta_tables) {
		g->ops.gr.setup_alpha_beta_tables(g, gr);
	}

	for (gpc_index = 0;
	     gpc_index < gr_pd_dist_skip_table__size_1_v() * 4;
	     gpc_index += 4) {

		gk20a_writel(g, gr_pd_dist_skip_table_r(gpc_index/4),
			     (gr_pd_dist_skip_table_gpc_4n0_mask_f(gr->gpc_skip_mask[gpc_index]) != 0U) ||
			     (gr_pd_dist_skip_table_gpc_4n1_mask_f(gr->gpc_skip_mask[gpc_index + 1]) != 0U) ||
			     (gr_pd_dist_skip_table_gpc_4n2_mask_f(gr->gpc_skip_mask[gpc_index + 2]) != 0U) ||
			     (gr_pd_dist_skip_table_gpc_4n3_mask_f(gr->gpc_skip_mask[gpc_index + 3]) != 0U));
	}

	fuse_tpc_mask = g->ops.gr.get_gpc_tpc_mask(g, 0);
	if ((g->tpc_fs_mask_user != 0U) &&
		(fuse_tpc_mask == BIT32(gr->max_tpc_count) - 1U)) {
		u32 val = g->tpc_fs_mask_user;
		val &= (0x1U << gr->max_tpc_count) - 1U;
		gk20a_writel(g, gr_cwd_fs_r(),
			gr_cwd_fs_num_gpcs_f(gr->gpc_count) |
			gr_cwd_fs_num_tpcs_f(hweight32(val)));
	} else {
		gk20a_writel(g, gr_cwd_fs_r(),
			gr_cwd_fs_num_gpcs_f(gr->gpc_count) |
			gr_cwd_fs_num_tpcs_f(gr->tpc_count));
	}

	gk20a_writel(g, gr_bes_zrop_settings_r(),
		     gr_bes_zrop_settings_num_active_fbps_f(gr->num_fbps));
	gk20a_writel(g, gr_bes_crop_settings_r(),
		     gr_bes_crop_settings_num_active_fbps_f(gr->num_fbps));

	return 0;
}

int gr_gk20a_fecs_ctx_image_save(struct channel_gk20a *c, u32 save_type)
{
	struct gk20a *g = c->g;
	int ret;

	nvgpu_log_fn(g, " ");

	ret = gr_gk20a_submit_fecs_method_op(g,
		(struct fecs_method_op_gk20a) {
		.method.addr = save_type,
		.method.data = fecs_current_ctx_data(g, &c->inst_block),
		.mailbox = {.id = 0, .data = 0, .clr = 3, .ret = NULL,
			.ok = 1, .fail = 2,
		},
		.cond.ok = GR_IS_UCODE_OP_AND,
		.cond.fail = GR_IS_UCODE_OP_AND,
		 }, true);

	if (ret) {
		nvgpu_err(g, "save context image failed");
	}

	return ret;
}

u32 gk20a_init_sw_bundle(struct gk20a *g)
{
	struct av_list_gk20a *sw_bundle_init = &g->gr.ctx_vars.sw_bundle_init;
	u32 last_bundle_data = 0;
	u32 err = 0;
	unsigned int i;

	/* disable fe_go_idle */
	gk20a_writel(g, gr_fe_go_idle_timeout_r(),
		gr_fe_go_idle_timeout_count_disabled_f());
	/* enable pipe mode override */
	gk20a_writel(g, gr_pipe_bundle_config_r(),
		gr_pipe_bundle_config_override_pipe_mode_enabled_f());

	/* load bundle init */
	for (i = 0; i < sw_bundle_init->count; i++) {
		if (i == 0 || last_bundle_data != sw_bundle_init->l[i].value) {
			gk20a_writel(g, gr_pipe_bundle_data_r(),
				sw_bundle_init->l[i].value);
			last_bundle_data = sw_bundle_init->l[i].value;
		}

		gk20a_writel(g, gr_pipe_bundle_address_r(),
			     sw_bundle_init->l[i].addr);

		if (gr_pipe_bundle_address_value_v(sw_bundle_init->l[i].addr) ==
		    GR_GO_IDLE_BUNDLE) {
			err = gr_gk20a_wait_idle(g,
						  gk20a_get_gr_idle_timeout(g),
						  GR_IDLE_CHECK_DEFAULT);
			if (err != 0U) {
				goto error;
			}
		}

		err = gr_gk20a_wait_fe_idle(g, gk20a_get_gr_idle_timeout(g),
					    GR_IDLE_CHECK_DEFAULT);
		if (err != 0U) {
			goto error;
		}
	}

	if ((err == 0U) && (g->ops.gr.init_sw_veid_bundle != NULL)) {
		err = g->ops.gr.init_sw_veid_bundle(g);
		if (err != 0U) {
			goto error;
		}
	}

	if (g->ops.gr.init_sw_bundle64) {
		err = g->ops.gr.init_sw_bundle64(g);
		if (err != 0U) {
			goto error;
		}
	}

	/* disable pipe mode override */
	gk20a_writel(g, gr_pipe_bundle_config_r(),
		     gr_pipe_bundle_config_override_pipe_mode_disabled_f());

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);

	/* restore fe_go_idle */
	gk20a_writel(g, gr_fe_go_idle_timeout_r(),
		     gr_fe_go_idle_timeout_count_prod_f());

	return err;

error:
	/* in case of error skip waiting for GR idle - just restore state */
	gk20a_writel(g, gr_pipe_bundle_config_r(),
		     gr_pipe_bundle_config_override_pipe_mode_disabled_f());

	/* restore fe_go_idle */
	gk20a_writel(g, gr_fe_go_idle_timeout_r(),
		     gr_fe_go_idle_timeout_count_prod_f());

	return err;
}

/* init global golden image from a fresh gr_ctx in channel ctx.
   save a copy in local_golden_image in ctx_vars */
static int gr_gk20a_init_golden_ctx_image(struct gk20a *g,
					  struct channel_gk20a *c)
{
	struct gr_gk20a *gr = &g->gr;
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx = NULL;
	u32 ctx_header_bytes = ctxsw_prog_fecs_header_v();
	u32 ctx_header_words;
	u32 i;
	u32 data;
	struct nvgpu_mem *gold_mem = &gr->global_ctx_buffer[GOLDEN_CTX].mem;
	struct nvgpu_mem *gr_mem;
	u32 err = 0;
	struct aiv_list_gk20a *sw_ctx_load = &g->gr.ctx_vars.sw_ctx_load;
	struct av_list_gk20a *sw_method_init = &g->gr.ctx_vars.sw_method_init;
	u32 last_method_data = 0;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	gr_mem = &gr_ctx->mem;

	/* golden ctx is global to all channels. Although only the first
	   channel initializes golden image, driver needs to prevent multiple
	   channels from initializing golden ctx at the same time */
	nvgpu_mutex_acquire(&gr->ctx_mutex);

	if (gr->ctx_vars.golden_image_initialized) {
		goto clean_up;
	}
	if (!nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		struct nvgpu_timeout timeout;

		nvgpu_timeout_init(g, &timeout,
				   FE_PWR_MODE_TIMEOUT_MAX /
					FE_PWR_MODE_TIMEOUT_DEFAULT,
				   NVGPU_TIMER_RETRY_TIMER);
		gk20a_writel(g, gr_fe_pwr_mode_r(),
			gr_fe_pwr_mode_req_send_f() | gr_fe_pwr_mode_mode_force_on_f());
		do {
			u32 req = gr_fe_pwr_mode_req_v(gk20a_readl(g, gr_fe_pwr_mode_r()));
			if (req == gr_fe_pwr_mode_req_done_v()) {
				break;
			}
			nvgpu_udelay(FE_PWR_MODE_TIMEOUT_DEFAULT);
		} while (nvgpu_timeout_expired_msg(&timeout,
					"timeout forcing FE on") == 0);
	}


	gk20a_writel(g, gr_fecs_ctxsw_reset_ctl_r(),
			gr_fecs_ctxsw_reset_ctl_sys_halt_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_gpc_halt_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_be_halt_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_sys_engine_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_gpc_engine_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_be_engine_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_sys_context_reset_enabled_f() |
			gr_fecs_ctxsw_reset_ctl_gpc_context_reset_enabled_f() |
			gr_fecs_ctxsw_reset_ctl_be_context_reset_enabled_f());
	(void) gk20a_readl(g, gr_fecs_ctxsw_reset_ctl_r());
	nvgpu_udelay(10);

	gk20a_writel(g, gr_fecs_ctxsw_reset_ctl_r(),
			gr_fecs_ctxsw_reset_ctl_sys_halt_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_gpc_halt_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_be_halt_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_sys_engine_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_gpc_engine_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_be_engine_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_sys_context_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_gpc_context_reset_disabled_f() |
			gr_fecs_ctxsw_reset_ctl_be_context_reset_disabled_f());
	(void) gk20a_readl(g, gr_fecs_ctxsw_reset_ctl_r());
	nvgpu_udelay(10);

	if (!nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		struct nvgpu_timeout timeout;

		nvgpu_timeout_init(g, &timeout,
				   FE_PWR_MODE_TIMEOUT_MAX /
					FE_PWR_MODE_TIMEOUT_DEFAULT,
				   NVGPU_TIMER_RETRY_TIMER);
		gk20a_writel(g, gr_fe_pwr_mode_r(),
			gr_fe_pwr_mode_req_send_f() | gr_fe_pwr_mode_mode_auto_f());

		do {
			u32 req = gr_fe_pwr_mode_req_v(gk20a_readl(g, gr_fe_pwr_mode_r()));
			if (req == gr_fe_pwr_mode_req_done_v()) {
				break;
			}
			nvgpu_udelay(FE_PWR_MODE_TIMEOUT_DEFAULT);
		} while (nvgpu_timeout_expired_msg(&timeout,
				"timeout setting FE power to auto") == 0);
	}

	/* clear scc ram */
	gk20a_writel(g, gr_scc_init_r(),
		gr_scc_init_ram_trigger_f());

	err = gr_gk20a_fecs_ctx_bind_channel(g, c);
	if (err != 0U) {
		goto clean_up;
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);

	/* load ctx init */
	for (i = 0; i < sw_ctx_load->count; i++) {
		gk20a_writel(g, sw_ctx_load->l[i].addr,
			     sw_ctx_load->l[i].value);
	}

	if (g->ops.gr.init_preemption_state) {
		g->ops.gr.init_preemption_state(g);
	}

	if (g->ops.clock_gating.blcg_gr_load_gating_prod) {
		g->ops.clock_gating.blcg_gr_load_gating_prod(g, g->blcg_enabled);
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);
	if (err != 0U) {
		goto clean_up;
	}

	/* disable fe_go_idle */
	gk20a_writel(g, gr_fe_go_idle_timeout_r(),
		gr_fe_go_idle_timeout_count_disabled_f());

	err = g->ops.gr.commit_global_ctx_buffers(g, c, false);
	if (err != 0U) {
		goto clean_up;
	}

	/* override a few ctx state registers */
	g->ops.gr.commit_global_timeslice(g, c);

	/* floorsweep anything left */
	err = g->ops.gr.init_fs_state(g);
	if (err != 0U) {
		goto clean_up;
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);
	if (err != 0U) {
		goto restore_fe_go_idle;
	}

	err = gk20a_init_sw_bundle(g);
	if (err != 0U) {
		goto clean_up;
	}

restore_fe_go_idle:
	/* restore fe_go_idle */
	gk20a_writel(g, gr_fe_go_idle_timeout_r(),
		     gr_fe_go_idle_timeout_count_prod_f());

	if ((err != 0U) || (gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				      GR_IDLE_CHECK_DEFAULT) != 0)) {
		goto clean_up;
	}

	/* load method init */
	if (sw_method_init->count) {
		gk20a_writel(g, gr_pri_mme_shadow_raw_data_r(),
			     sw_method_init->l[0].value);
		gk20a_writel(g, gr_pri_mme_shadow_raw_index_r(),
			     gr_pri_mme_shadow_raw_index_write_trigger_f() |
			     sw_method_init->l[0].addr);
		last_method_data = sw_method_init->l[0].value;
	}
	for (i = 1; i < sw_method_init->count; i++) {
		if (sw_method_init->l[i].value != last_method_data) {
			gk20a_writel(g, gr_pri_mme_shadow_raw_data_r(),
				sw_method_init->l[i].value);
			last_method_data = sw_method_init->l[i].value;
		}
		gk20a_writel(g, gr_pri_mme_shadow_raw_index_r(),
			gr_pri_mme_shadow_raw_index_write_trigger_f() |
			sw_method_init->l[i].addr);
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);
	if (err != 0U) {
		goto clean_up;
	}

	ctx_header_words =  roundup(ctx_header_bytes, sizeof(u32));
	ctx_header_words >>= 2;

	g->ops.mm.l2_flush(g, true);

	for (i = 0; i < ctx_header_words; i++) {
		data = nvgpu_mem_rd32(g, gr_mem, i);
		nvgpu_mem_wr32(g, gold_mem, i, data);
	}
	nvgpu_mem_wr(g, gold_mem, ctxsw_prog_main_image_zcull_o(),
		 ctxsw_prog_main_image_zcull_mode_no_ctxsw_v());

	g->ops.gr.write_zcull_ptr(g, gold_mem, 0);

	err = g->ops.gr.commit_inst(c, gr_ctx->global_ctx_buffer_va[GOLDEN_CTX_VA]);
	if (err != 0U) {
		goto clean_up;
	}

	gr_gk20a_fecs_ctx_image_save(c, gr_fecs_method_push_adr_wfi_golden_save_v());



	if (gr->ctx_vars.local_golden_image == NULL) {

		gr->ctx_vars.local_golden_image =
			nvgpu_vzalloc(g, gr->ctx_vars.golden_image_size);

		if (gr->ctx_vars.local_golden_image == NULL) {
			err = -ENOMEM;
			goto clean_up;
		}
		nvgpu_mem_rd_n(g, gold_mem, 0,
			gr->ctx_vars.local_golden_image,
			gr->ctx_vars.golden_image_size);

	}

	err = g->ops.gr.commit_inst(c, gr_mem->gpu_va);
	if (err != 0U) {
		goto clean_up;
	}

	gr->ctx_vars.golden_image_initialized = true;

	gk20a_writel(g, gr_fecs_current_ctx_r(),
		gr_fecs_current_ctx_valid_false_f());

clean_up:
	if (err != 0U) {
		nvgpu_err(g, "fail");
	} else {
		nvgpu_log_fn(g, "done");
	}

	nvgpu_mutex_release(&gr->ctx_mutex);
	return err;
}

int gr_gk20a_update_smpc_ctxsw_mode(struct gk20a *g,
				    struct channel_gk20a *c,
				    bool enable_smpc_ctxsw)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx = NULL;
	struct nvgpu_mem *mem = NULL;
	u32 data;
	int ret;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	mem = &gr_ctx->mem;
	if (!nvgpu_mem_is_valid(mem)) {
		nvgpu_err(g, "no graphics context allocated");
		return -EFAULT;
	}

	ret = gk20a_disable_channel_tsg(g, c);
	if (ret) {
		nvgpu_err(g, "failed to disable channel/TSG");
		goto out;
	}
	ret = gk20a_fifo_preempt(g, c);
	if (ret) {
		gk20a_enable_channel_tsg(g, c);
		nvgpu_err(g, "failed to preempt channel/TSG");
		goto out;
	}

	/* Channel gr_ctx buffer is gpu cacheable.
	   Flush and invalidate before cpu update. */
	g->ops.mm.l2_flush(g, true);

	data = nvgpu_mem_rd(g, mem,
		ctxsw_prog_main_image_pm_o());

	data = data & ~ctxsw_prog_main_image_pm_smpc_mode_m();
	data |= enable_smpc_ctxsw ?
		ctxsw_prog_main_image_pm_smpc_mode_ctxsw_f() :
		ctxsw_prog_main_image_pm_smpc_mode_no_ctxsw_f();

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_pm_o(), data);

out:
	gk20a_enable_channel_tsg(g, c);
	return ret;
}

int gr_gk20a_update_hwpm_ctxsw_mode(struct gk20a *g,
				  struct channel_gk20a *c,
				  u64 gpu_va,
				  u32 mode)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_mem *gr_mem = NULL;
	struct nvgpu_gr_ctx *gr_ctx;
	struct pm_ctx_desc *pm_ctx;
	u32 data;
	u64 virt_addr = 0;
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	int ret;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	pm_ctx = &gr_ctx->pm_ctx;
	gr_mem = &gr_ctx->mem;
	if (!nvgpu_mem_is_valid(gr_mem)) {
		nvgpu_err(g, "no graphics context allocated");
		return -EFAULT;
	}

	if ((mode == NVGPU_DBG_HWPM_CTXSW_MODE_STREAM_OUT_CTXSW) &&
		(g->ops.gr.get_hw_accessor_stream_out_mode == NULL)) {
		nvgpu_err(g, "Mode-E hwpm context switch mode is not supported");
		return -EINVAL;
	}

	switch (mode) {
	case NVGPU_DBG_HWPM_CTXSW_MODE_CTXSW:
		if (pm_ctx->pm_mode == ctxsw_prog_main_image_pm_mode_ctxsw_f()) {
			return 0;
		}
		break;
	case  NVGPU_DBG_HWPM_CTXSW_MODE_NO_CTXSW:
		if (pm_ctx->pm_mode == ctxsw_prog_main_image_pm_mode_no_ctxsw_f()) {
			return 0;
		}
		break;
	case NVGPU_DBG_HWPM_CTXSW_MODE_STREAM_OUT_CTXSW:
		if (pm_ctx->pm_mode == g->ops.gr.get_hw_accessor_stream_out_mode()) {
			return 0;
		}
		break;
	default:
		nvgpu_err(g, "invalid hwpm context switch mode");
		return -EINVAL;
	}

	ret = gk20a_disable_channel_tsg(g, c);
	if (ret) {
		nvgpu_err(g, "failed to disable channel/TSG");
		return ret;
	}

	ret = gk20a_fifo_preempt(g, c);
	if (ret) {
		gk20a_enable_channel_tsg(g, c);
		nvgpu_err(g, "failed to preempt channel/TSG");
		return ret;
	}

	/* Channel gr_ctx buffer is gpu cacheable.
	   Flush and invalidate before cpu update. */
	g->ops.mm.l2_flush(g, true);

	if (mode != NVGPU_DBG_HWPM_CTXSW_MODE_NO_CTXSW) {
		/* Allocate buffer if necessary */
		if (pm_ctx->mem.gpu_va == 0) {
			ret = nvgpu_dma_alloc_sys(g,
					g->gr.ctx_vars.pm_ctxsw_image_size,
					&pm_ctx->mem);
			if (ret) {
				c->g->ops.fifo.enable_channel(c);
				nvgpu_err(g,
					"failed to allocate pm ctxt buffer");
				return ret;
			}

			pm_ctx->mem.gpu_va = nvgpu_gmmu_map_fixed(c->vm,
							&pm_ctx->mem,
							gpu_va,
							pm_ctx->mem.size,
							NVGPU_VM_MAP_CACHEABLE,
							gk20a_mem_flag_none, true,
							pm_ctx->mem.aperture);
			if (pm_ctx->mem.gpu_va == 0ULL) {
				nvgpu_err(g,
					"failed to map pm ctxt buffer");
				nvgpu_dma_free(g, &pm_ctx->mem);
				c->g->ops.fifo.enable_channel(c);
				return -ENOMEM;
			}
		}

		if ((mode == NVGPU_DBG_HWPM_CTXSW_MODE_STREAM_OUT_CTXSW) &&
			(g->ops.gr.init_hwpm_pmm_register != NULL)) {
			g->ops.gr.init_hwpm_pmm_register(g);
		}
	}

	data = nvgpu_mem_rd(g, gr_mem, ctxsw_prog_main_image_pm_o());
	data = data & ~ctxsw_prog_main_image_pm_mode_m();

	switch (mode) {
	case  NVGPU_DBG_HWPM_CTXSW_MODE_CTXSW:
		pm_ctx->pm_mode = ctxsw_prog_main_image_pm_mode_ctxsw_f();
		virt_addr = pm_ctx->mem.gpu_va;
		break;
	case NVGPU_DBG_HWPM_CTXSW_MODE_STREAM_OUT_CTXSW:
		pm_ctx->pm_mode = g->ops.gr.get_hw_accessor_stream_out_mode();
		virt_addr = pm_ctx->mem.gpu_va;
		break;
	case NVGPU_DBG_HWPM_CTXSW_MODE_NO_CTXSW:
		pm_ctx->pm_mode = ctxsw_prog_main_image_pm_mode_no_ctxsw_f();
		virt_addr = 0;
	}

	data |= pm_ctx->pm_mode;

	nvgpu_mem_wr(g, gr_mem, ctxsw_prog_main_image_pm_o(), data);

	if (ctxheader->gpu_va) {
		struct channel_gk20a *ch;

		nvgpu_rwsem_down_read(&tsg->ch_list_lock);
		nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
			g->ops.gr.write_pm_ptr(g, &ch->ctx_header, virt_addr);
		}
		nvgpu_rwsem_up_read(&tsg->ch_list_lock);
	} else {
		g->ops.gr.write_pm_ptr(g, gr_mem, virt_addr);
	}

	/* enable channel */
	gk20a_enable_channel_tsg(g, c);

	return 0;
}

void gk20a_gr_init_ctxsw_hdr_data(struct gk20a *g,
				struct nvgpu_mem *mem)
{
	nvgpu_mem_wr(g, mem,
			ctxsw_prog_main_image_num_save_ops_o(), 0);
	nvgpu_mem_wr(g, mem,
			ctxsw_prog_main_image_num_restore_ops_o(), 0);
}

/* load saved fresh copy of gloden image into channel gr_ctx */
int gr_gk20a_load_golden_ctx_image(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct gr_gk20a *gr = &g->gr;
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;
	u32 virt_addr_lo;
	u32 virt_addr_hi;
	u64 virt_addr = 0;
	u32 v, data;
	int ret = 0;
	struct nvgpu_mem *mem;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	mem = &gr_ctx->mem;
	if (gr->ctx_vars.local_golden_image == NULL) {
		return -EINVAL;
	}

	/* Channel gr_ctx buffer is gpu cacheable.
	   Flush and invalidate before cpu update. */
	g->ops.mm.l2_flush(g, true);

	nvgpu_mem_wr_n(g, mem, 0,
		gr->ctx_vars.local_golden_image,
		gr->ctx_vars.golden_image_size);

	if (g->ops.gr.init_ctxsw_hdr_data) {
		g->ops.gr.init_ctxsw_hdr_data(g, mem);
	}

	if ((g->ops.gr.enable_cde_in_fecs != NULL) && c->cde) {
		g->ops.gr.enable_cde_in_fecs(g, mem);
	}

	/* set priv access map */
	virt_addr_lo =
		 u64_lo32(gr_ctx->global_ctx_buffer_va[PRIV_ACCESS_MAP_VA]);
	virt_addr_hi =
		 u64_hi32(gr_ctx->global_ctx_buffer_va[PRIV_ACCESS_MAP_VA]);

	if (g->allow_all) {
		data = ctxsw_prog_main_image_priv_access_map_config_mode_allow_all_f();
	} else {
		data = ctxsw_prog_main_image_priv_access_map_config_mode_use_map_f();
	}

	nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_priv_access_map_config_o(),
		 data);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_priv_access_map_addr_lo_o(),
		virt_addr_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_priv_access_map_addr_hi_o(),
		virt_addr_hi);

	/* disable verif features */
	v = nvgpu_mem_rd(g, mem, ctxsw_prog_main_image_misc_options_o());
	v = v & ~(ctxsw_prog_main_image_misc_options_verif_features_m());
	v = v | ctxsw_prog_main_image_misc_options_verif_features_disabled_f();
	nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_misc_options_o(), v);

	if (g->ops.gr.update_ctxsw_preemption_mode) {
		g->ops.gr.update_ctxsw_preemption_mode(g, c, mem);
	}

	if (g->ops.gr.update_boosted_ctx) {
		g->ops.gr.update_boosted_ctx(g, mem, gr_ctx);
	}

	virt_addr_lo = u64_lo32(gr_ctx->patch_ctx.mem.gpu_va);
	virt_addr_hi = u64_hi32(gr_ctx->patch_ctx.mem.gpu_va);

	nvgpu_log(g, gpu_dbg_info, "write patch count = %d",
			gr_ctx->patch_ctx.data_count);
	nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_patch_count_o(),
		 gr_ctx->patch_ctx.data_count);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_patch_adr_lo_o(),
		virt_addr_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_patch_adr_hi_o(),
		virt_addr_hi);

	/* Update main header region of the context buffer with the info needed
	 * for PM context switching, including mode and possibly a pointer to
	 * the PM backing store.
	 */
	if (gr_ctx->pm_ctx.pm_mode != ctxsw_prog_main_image_pm_mode_no_ctxsw_f()) {
		if (gr_ctx->pm_ctx.mem.gpu_va == 0) {
			nvgpu_err(g,
				"context switched pm with no pm buffer!");
			return -EFAULT;
		}

		virt_addr = gr_ctx->pm_ctx.mem.gpu_va;
	} else {
		virt_addr = 0;
	}

	data = nvgpu_mem_rd(g, mem, ctxsw_prog_main_image_pm_o());
	data = data & ~ctxsw_prog_main_image_pm_mode_m();
	data |= gr_ctx->pm_ctx.pm_mode;

	nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_pm_o(), data);

	g->ops.gr.write_pm_ptr(g, mem, virt_addr);

	return ret;
}

static void gr_gk20a_start_falcon_ucode(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0),
		     gr_fecs_ctxsw_mailbox_clear_value_f(~0));

	gk20a_writel(g, gr_gpccs_dmactl_r(), gr_gpccs_dmactl_require_ctx_f(0));
	gk20a_writel(g, gr_fecs_dmactl_r(), gr_fecs_dmactl_require_ctx_f(0));

	gk20a_writel(g, gr_gpccs_cpuctl_r(), gr_gpccs_cpuctl_startcpu_f(1));
	gk20a_writel(g, gr_fecs_cpuctl_r(), gr_fecs_cpuctl_startcpu_f(1));

	nvgpu_log_fn(g, "done");
}

static int gr_gk20a_init_ctxsw_ucode_vaspace(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	int err;

	err = g->ops.mm.alloc_inst_block(g, &ucode_info->inst_blk_desc);
	if (err != 0) {
		return err;
	}

	g->ops.mm.init_inst_block(&ucode_info->inst_blk_desc, vm, 0);

	/* Map ucode surface to GMMU */
	ucode_info->surface_desc.gpu_va = nvgpu_gmmu_map(vm,
					&ucode_info->surface_desc,
					ucode_info->surface_desc.size,
					0, /* flags */
					gk20a_mem_flag_read_only,
					false,
					ucode_info->surface_desc.aperture);
	if (ucode_info->surface_desc.gpu_va == 0ULL) {
		nvgpu_err(g, "failed to update gmmu ptes");
		return -ENOMEM;
	}

	return 0;
}

static void gr_gk20a_init_ctxsw_ucode_segment(
	struct gk20a_ctxsw_ucode_segment *p_seg, u32 *offset, u32 size)
{
	p_seg->offset = *offset;
	p_seg->size = size;
	*offset = ALIGN(*offset + size, BLK_SIZE);
}

static void gr_gk20a_init_ctxsw_ucode_segments(
	struct gk20a_ctxsw_ucode_segments *segments, u32 *offset,
	struct gk20a_ctxsw_bootloader_desc *bootdesc,
	u32 code_size, u32 data_size)
{
	u32 boot_size = ALIGN(bootdesc->size, sizeof(u32));
	segments->boot_entry = bootdesc->entry_point;
	segments->boot_imem_offset = bootdesc->imem_offset;
	gr_gk20a_init_ctxsw_ucode_segment(&segments->boot, offset, boot_size);
	gr_gk20a_init_ctxsw_ucode_segment(&segments->code, offset, code_size);
	gr_gk20a_init_ctxsw_ucode_segment(&segments->data, offset, data_size);
}

static int gr_gk20a_copy_ctxsw_ucode_segments(
	struct gk20a *g,
	struct nvgpu_mem *dst,
	struct gk20a_ctxsw_ucode_segments *segments,
	u32 *bootimage,
	u32 *code, u32 *data)
{
	unsigned int i;

	nvgpu_mem_wr_n(g, dst, segments->boot.offset, bootimage,
			segments->boot.size);
	nvgpu_mem_wr_n(g, dst, segments->code.offset, code,
			segments->code.size);
	nvgpu_mem_wr_n(g, dst, segments->data.offset, data,
			segments->data.size);

	/* compute a "checksum" for the boot binary to detect its version */
	segments->boot_signature = 0;
	for (i = 0; i < segments->boot.size / sizeof(u32); i++) {
		segments->boot_signature += bootimage[i];
	}

	return 0;
}

int gr_gk20a_init_ctxsw_ucode(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	struct gk20a_ctxsw_bootloader_desc *fecs_boot_desc;
	struct gk20a_ctxsw_bootloader_desc *gpccs_boot_desc;
	struct nvgpu_firmware *fecs_fw;
	struct nvgpu_firmware *gpccs_fw;
	u32 *fecs_boot_image;
	u32 *gpccs_boot_image;
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	u32 ucode_size;
	int err = 0;

	fecs_fw = nvgpu_request_firmware(g, GK20A_FECS_UCODE_IMAGE, 0);
	if (fecs_fw == NULL) {
		nvgpu_err(g, "failed to load fecs ucode!!");
		return -ENOENT;
	}

	fecs_boot_desc = (void *)fecs_fw->data;
	fecs_boot_image = (void *)(fecs_fw->data +
				sizeof(struct gk20a_ctxsw_bootloader_desc));

	gpccs_fw = nvgpu_request_firmware(g, GK20A_GPCCS_UCODE_IMAGE, 0);
	if (gpccs_fw == NULL) {
		nvgpu_release_firmware(g, fecs_fw);
		nvgpu_err(g, "failed to load gpccs ucode!!");
		return -ENOENT;
	}

	gpccs_boot_desc = (void *)gpccs_fw->data;
	gpccs_boot_image = (void *)(gpccs_fw->data +
				sizeof(struct gk20a_ctxsw_bootloader_desc));

	ucode_size = 0;
	gr_gk20a_init_ctxsw_ucode_segments(&ucode_info->fecs, &ucode_size,
		fecs_boot_desc,
		g->gr.ctx_vars.ucode.fecs.inst.count * sizeof(u32),
		g->gr.ctx_vars.ucode.fecs.data.count * sizeof(u32));
	gr_gk20a_init_ctxsw_ucode_segments(&ucode_info->gpccs, &ucode_size,
		gpccs_boot_desc,
		g->gr.ctx_vars.ucode.gpccs.inst.count * sizeof(u32),
		g->gr.ctx_vars.ucode.gpccs.data.count * sizeof(u32));

	err = nvgpu_dma_alloc_sys(g, ucode_size, &ucode_info->surface_desc);
	if (err != 0) {
		goto clean_up;
	}

	gr_gk20a_copy_ctxsw_ucode_segments(g, &ucode_info->surface_desc,
		&ucode_info->fecs,
		fecs_boot_image,
		g->gr.ctx_vars.ucode.fecs.inst.l,
		g->gr.ctx_vars.ucode.fecs.data.l);

	nvgpu_release_firmware(g, fecs_fw);
	fecs_fw = NULL;

	gr_gk20a_copy_ctxsw_ucode_segments(g, &ucode_info->surface_desc,
		&ucode_info->gpccs,
		gpccs_boot_image,
		g->gr.ctx_vars.ucode.gpccs.inst.l,
		g->gr.ctx_vars.ucode.gpccs.data.l);

	nvgpu_release_firmware(g, gpccs_fw);
	gpccs_fw = NULL;

	err = gr_gk20a_init_ctxsw_ucode_vaspace(g);
	if (err != 0) {
		goto clean_up;
	}

	return 0;

clean_up:
	if (ucode_info->surface_desc.gpu_va) {
		nvgpu_gmmu_unmap(vm, &ucode_info->surface_desc,
				 ucode_info->surface_desc.gpu_va);
	}
	nvgpu_dma_free(g, &ucode_info->surface_desc);

	nvgpu_release_firmware(g, gpccs_fw);
	gpccs_fw = NULL;
	nvgpu_release_firmware(g, fecs_fw);
	fecs_fw = NULL;

	return err;
}

static void gr_gk20a_wait_for_fecs_arb_idle(struct gk20a *g)
{
	int retries = FECS_ARB_CMD_TIMEOUT_MAX / FECS_ARB_CMD_TIMEOUT_DEFAULT;
	u32 val;

	val = gk20a_readl(g, gr_fecs_arb_ctx_cmd_r());
	while ((gr_fecs_arb_ctx_cmd_cmd_v(val) != 0U) && (retries != 0)) {
		nvgpu_udelay(FECS_ARB_CMD_TIMEOUT_DEFAULT);
		retries--;
		val = gk20a_readl(g, gr_fecs_arb_ctx_cmd_r());
	}

	if (retries == 0) {
		nvgpu_err(g, "arbiter cmd timeout, fecs arb ctx cmd: 0x%08x",
				gk20a_readl(g, gr_fecs_arb_ctx_cmd_r()));
	}

	retries = FECS_ARB_CMD_TIMEOUT_MAX / FECS_ARB_CMD_TIMEOUT_DEFAULT;
	while (((gk20a_readl(g, gr_fecs_ctxsw_status_1_r()) &
			gr_fecs_ctxsw_status_1_arb_busy_m()) != 0U) &&
	       (retries != 0)) {
		nvgpu_udelay(FECS_ARB_CMD_TIMEOUT_DEFAULT);
		retries--;
	}
	if (retries == 0) {
		nvgpu_err(g,
			  "arbiter idle timeout, fecs ctxsw status: 0x%08x",
			  gk20a_readl(g, gr_fecs_ctxsw_status_1_r()));
	}
}

void gr_gk20a_load_falcon_bind_instblk(struct gk20a *g)
{
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	int retries = FECS_ARB_CMD_TIMEOUT_MAX / FECS_ARB_CMD_TIMEOUT_DEFAULT;
	u64 inst_ptr;

	while (((gk20a_readl(g, gr_fecs_ctxsw_status_1_r()) &
			gr_fecs_ctxsw_status_1_arb_busy_m()) != 0U) &&
	       (retries != 0)) {
		nvgpu_udelay(FECS_ARB_CMD_TIMEOUT_DEFAULT);
		retries--;
	}
	if (retries == 0) {
		nvgpu_err(g,
			  "arbiter idle timeout, status: %08x",
			  gk20a_readl(g, gr_fecs_ctxsw_status_1_r()));
	}

	gk20a_writel(g, gr_fecs_arb_ctx_adr_r(), 0x0);

	inst_ptr = nvgpu_inst_block_addr(g, &ucode_info->inst_blk_desc);
	gk20a_writel(g, gr_fecs_new_ctx_r(),
		     gr_fecs_new_ctx_ptr_f(inst_ptr >> 12) |
		     nvgpu_aperture_mask(g, &ucode_info->inst_blk_desc,
				gr_fecs_new_ctx_target_sys_mem_ncoh_f(),
				gr_fecs_new_ctx_target_sys_mem_coh_f(),
				gr_fecs_new_ctx_target_vid_mem_f()) |
		     gr_fecs_new_ctx_valid_m());

	gk20a_writel(g, gr_fecs_arb_ctx_ptr_r(),
		     gr_fecs_arb_ctx_ptr_ptr_f(inst_ptr >> 12) |
		     nvgpu_aperture_mask(g, &ucode_info->inst_blk_desc,
				gr_fecs_arb_ctx_ptr_target_sys_mem_ncoh_f(),
				gr_fecs_arb_ctx_ptr_target_sys_mem_coh_f(),
				gr_fecs_arb_ctx_ptr_target_vid_mem_f()));

	gk20a_writel(g, gr_fecs_arb_ctx_cmd_r(), 0x7);

	/* Wait for arbiter command to complete */
	gr_gk20a_wait_for_fecs_arb_idle(g);

	gk20a_writel(g, gr_fecs_current_ctx_r(),
			gr_fecs_current_ctx_ptr_f(inst_ptr >> 12) |
			gr_fecs_current_ctx_target_m() |
			gr_fecs_current_ctx_valid_m());
	/* Send command to arbiter to flush */
	gk20a_writel(g, gr_fecs_arb_ctx_cmd_r(), gr_fecs_arb_ctx_cmd_cmd_s());

	gr_gk20a_wait_for_fecs_arb_idle(g);

}

void gr_gk20a_load_ctxsw_ucode_header(struct gk20a *g, u64 addr_base,
	struct gk20a_ctxsw_ucode_segments *segments, u32 reg_offset)
{
	u32 addr_code32;
	u32 addr_data32;

	addr_code32 = u64_lo32((addr_base + segments->code.offset) >> 8);
	addr_data32 = u64_lo32((addr_base + segments->data.offset) >> 8);

	/*
	 * Copy falcon bootloader header into dmem at offset 0.
	 * Configure dmem port 0 for auto-incrementing writes starting at dmem
	 * offset 0.
	 */
	gk20a_writel(g, reg_offset + gr_fecs_dmemc_r(0),
			gr_fecs_dmemc_offs_f(0) |
			gr_fecs_dmemc_blk_f(0) |
			gr_fecs_dmemc_aincw_f(1));

	/* Write out the actual data */
	switch (segments->boot_signature) {
	case FALCON_UCODE_SIG_T18X_GPCCS_WITH_RESERVED:
	case FALCON_UCODE_SIG_T21X_FECS_WITH_DMEM_SIZE:
	case FALCON_UCODE_SIG_T21X_FECS_WITH_RESERVED:
	case FALCON_UCODE_SIG_T21X_GPCCS_WITH_RESERVED:
	case FALCON_UCODE_SIG_T12X_FECS_WITH_RESERVED:
	case FALCON_UCODE_SIG_T12X_GPCCS_WITH_RESERVED:
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		/* fallthrough */
	case FALCON_UCODE_SIG_T12X_FECS_WITHOUT_RESERVED:
	case FALCON_UCODE_SIG_T12X_GPCCS_WITHOUT_RESERVED:
	case FALCON_UCODE_SIG_T21X_FECS_WITHOUT_RESERVED:
	case FALCON_UCODE_SIG_T21X_FECS_WITHOUT_RESERVED2:
	case FALCON_UCODE_SIG_T21X_GPCCS_WITHOUT_RESERVED:
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 4);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				addr_code32);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				segments->code.size);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				addr_data32);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				segments->data.size);
		break;
	case FALCON_UCODE_SIG_T12X_FECS_OLDER:
	case FALCON_UCODE_SIG_T12X_GPCCS_OLDER:
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				addr_code32);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				segments->code.size);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				addr_data32);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				segments->data.size);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0),
				addr_code32);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
		break;
	default:
		nvgpu_err(g,
				"unknown falcon ucode boot signature 0x%08x"
				" with reg_offset 0x%08x",
				segments->boot_signature, reg_offset);
		BUG();
	}
}

void gr_gk20a_load_ctxsw_ucode_boot(struct gk20a *g, u64 addr_base,
	struct gk20a_ctxsw_ucode_segments *segments, u32 reg_offset)
{
	u32 addr_load32;
	u32 blocks;
	u32 b;
	u32 dst;

	addr_load32 = u64_lo32((addr_base + segments->boot.offset) >> 8);
	blocks = ((segments->boot.size + 0xFF) & ~0xFF) >> 8;

	/*
	 * Set the base FB address for the DMA transfer. Subtract off the 256
	 * byte IMEM block offset such that the relative FB and IMEM offsets
	 * match, allowing the IMEM tags to be properly created.
	 */

	dst = segments->boot_imem_offset;
	gk20a_writel(g, reg_offset + gr_fecs_dmatrfbase_r(),
			(addr_load32 - (dst >> 8)));

	for (b = 0; b < blocks; b++) {
		/* Setup destination IMEM offset */
		gk20a_writel(g, reg_offset + gr_fecs_dmatrfmoffs_r(),
				dst + (b << 8));

		/* Setup source offset (relative to BASE) */
		gk20a_writel(g, reg_offset + gr_fecs_dmatrffboffs_r(),
				dst + (b << 8));

		gk20a_writel(g, reg_offset + gr_fecs_dmatrfcmd_r(),
				gr_fecs_dmatrfcmd_imem_f(0x01) |
				gr_fecs_dmatrfcmd_write_f(0x00) |
				gr_fecs_dmatrfcmd_size_f(0x06) |
				gr_fecs_dmatrfcmd_ctxdma_f(0));
	}

	/* Specify the falcon boot vector */
	gk20a_writel(g, reg_offset + gr_fecs_bootvec_r(),
			gr_fecs_bootvec_vec_f(segments->boot_entry));
}

static void gr_gk20a_load_falcon_with_bootloader(struct gk20a *g)
{
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	u64 addr_base = ucode_info->surface_desc.gpu_va;

	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0), 0x0);

	gr_gk20a_load_falcon_bind_instblk(g);

	g->ops.gr.falcon_load_ucode(g, addr_base,
		&g->ctxsw_ucode_info.fecs, 0);

	g->ops.gr.falcon_load_ucode(g, addr_base,
		&g->ctxsw_ucode_info.gpccs,
		gr_gpcs_gpccs_falcon_hwcfg_r() -
		gr_fecs_falcon_hwcfg_r());
}

int gr_gk20a_load_ctxsw_ucode(struct gk20a *g)
{
	int err;

	nvgpu_log_fn(g, " ");

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(7),
			gr_fecs_ctxsw_mailbox_value_f(0xc0de7777));
		gk20a_writel(g, gr_gpccs_ctxsw_mailbox_r(7),
			gr_gpccs_ctxsw_mailbox_value_f(0xc0de7777));
	}

	/*
	 * In case bootloader is not supported, revert to the old way of
	 * loading gr ucode, without the faster bootstrap routine.
	 */
	if (!nvgpu_is_enabled(g, NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP)) {
		gr_gk20a_load_falcon_dmem(g);
		gr_gk20a_load_falcon_imem(g);
		gr_gk20a_start_falcon_ucode(g);
	} else {
		if (!g->gr.skip_ucode_init) {
			err = gr_gk20a_init_ctxsw_ucode(g);

			if (err != 0) {
				return err;
			}
		}
		gr_gk20a_load_falcon_with_bootloader(g);
		g->gr.skip_ucode_init = true;
	}
	nvgpu_log_fn(g, "done");
	return 0;
}

static int gr_gk20a_wait_ctxsw_ready(struct gk20a *g)
{
	u32 ret;

	nvgpu_log_fn(g, " ");

	ret = gr_gk20a_ctx_wait_ucode(g, 0, NULL,
				      GR_IS_UCODE_OP_EQUAL,
				      eUcodeHandshakeInitComplete,
				      GR_IS_UCODE_OP_SKIP, 0, false);
	if (ret) {
		nvgpu_err(g, "falcon ucode init timeout");
		return ret;
	}

	if (nvgpu_is_enabled(g, NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP) ||
		nvgpu_is_enabled(g, NVGPU_SEC_SECUREGPCCS)) {
		gk20a_writel(g, gr_fecs_current_ctx_r(),
			gr_fecs_current_ctx_valid_false_f());
	}

	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0), 0xffffffff);
	gk20a_writel(g, gr_fecs_method_data_r(), 0x7fffffff);
	gk20a_writel(g, gr_fecs_method_push_r(),
		     gr_fecs_method_push_adr_set_watchdog_timeout_f());

	nvgpu_log_fn(g, "done");
	return 0;
}

int gr_gk20a_init_ctx_state(struct gk20a *g)
{
	u32 ret;
	struct fecs_method_op_gk20a op = {
		.mailbox = { .id = 0, .data = 0,
			     .clr = ~0, .ok = 0, .fail = 0},
		.method.data = 0,
		.cond.ok = GR_IS_UCODE_OP_NOT_EQUAL,
		.cond.fail = GR_IS_UCODE_OP_SKIP,
		};

	nvgpu_log_fn(g, " ");
	/* query ctxsw image sizes, if golden context is not created */
	if (!g->gr.ctx_vars.golden_image_initialized) {
		op.method.addr =
			gr_fecs_method_push_adr_discover_image_size_v();
		op.mailbox.ret = &g->gr.ctx_vars.golden_image_size;
		ret = gr_gk20a_submit_fecs_method_op(g, op, false);
		if (ret) {
			nvgpu_err(g,
				   "query golden image size failed");
			return ret;
		}
		op.method.addr =
			gr_fecs_method_push_adr_discover_zcull_image_size_v();
		op.mailbox.ret = &g->gr.ctx_vars.zcull_ctxsw_image_size;
		ret = gr_gk20a_submit_fecs_method_op(g, op, false);
		if (ret) {
			nvgpu_err(g,
				   "query zcull ctx image size failed");
			return ret;
		}
		op.method.addr =
			gr_fecs_method_push_adr_discover_pm_image_size_v();
		op.mailbox.ret = &g->gr.ctx_vars.pm_ctxsw_image_size;
		ret = gr_gk20a_submit_fecs_method_op(g, op, false);
		if (ret) {
			nvgpu_err(g,
				   "query pm ctx image size failed");
			return ret;
		}
		g->gr.ctx_vars.priv_access_map_size = 512 * 1024;
#ifdef CONFIG_GK20A_CTXSW_TRACE
		g->gr.ctx_vars.fecs_trace_buffer_size =
			gk20a_fecs_trace_buffer_size(g);
#endif
	}

	nvgpu_log_fn(g, "done");
	return 0;
}

void gk20a_gr_destroy_ctx_buffer(struct gk20a *g,
					struct gr_ctx_buffer_desc *desc)
{
	if (desc == NULL) {
		return;
	}
	nvgpu_dma_free(g, &desc->mem);
	desc->destroy = NULL;
}

int gk20a_gr_alloc_ctx_buffer(struct gk20a *g,
				     struct gr_ctx_buffer_desc *desc,
				     size_t size)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (nvgpu_mem_is_valid(&desc->mem)) {
		return 0;
	}

	err = nvgpu_dma_alloc_sys(g, size, &desc->mem);
	if (err != 0) {
		return err;
	}

	desc->destroy = gk20a_gr_destroy_ctx_buffer;

	return err;
}

static void gr_gk20a_free_global_ctx_buffers(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 i;

	for (i = 0; i < NR_GLOBAL_CTX_BUF; i++) {
		/* destroy exists iff buffer is allocated */
		if (gr->global_ctx_buffer[i].destroy) {
			gr->global_ctx_buffer[i].destroy(g,
					&gr->global_ctx_buffer[i]);
		}
	}

	nvgpu_log_fn(g, "done");
}

int gr_gk20a_alloc_global_ctx_buffers(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int attr_buffer_size, err;

	u32 cb_buffer_size = gr->bundle_cb_default_size *
		gr_scc_bundle_cb_size_div_256b_byte_granularity_v();

	u32 pagepool_buffer_size = g->ops.gr.pagepool_default_size(g) *
		gr_scc_pagepool_total_pages_byte_granularity_v();

	nvgpu_log_fn(g, " ");

	attr_buffer_size = g->ops.gr.calc_global_ctx_buffer_size(g);

	nvgpu_log_info(g, "cb_buffer_size : %d", cb_buffer_size);

	err = gk20a_gr_alloc_ctx_buffer(g, &gr->global_ctx_buffer[CIRCULAR],
					cb_buffer_size);
	if (err != 0) {
		goto clean_up;
	}

	if (g->ops.secure_alloc) {
		err = g->ops.secure_alloc(g,
				       &gr->global_ctx_buffer[CIRCULAR_VPR],
				       cb_buffer_size);
		if (err != 0) {
			goto clean_up;
		}
	}

	nvgpu_log_info(g, "pagepool_buffer_size : %d", pagepool_buffer_size);

	err = gk20a_gr_alloc_ctx_buffer(g, &gr->global_ctx_buffer[PAGEPOOL],
					pagepool_buffer_size);
	if (err != 0) {
		goto clean_up;
	}

	if (g->ops.secure_alloc) {
		err = g->ops.secure_alloc(g,
				       &gr->global_ctx_buffer[PAGEPOOL_VPR],
				       pagepool_buffer_size);
		if (err != 0) {
			goto clean_up;
		}
	}

	nvgpu_log_info(g, "attr_buffer_size : %d", attr_buffer_size);

	err = gk20a_gr_alloc_ctx_buffer(g, &gr->global_ctx_buffer[ATTRIBUTE],
					attr_buffer_size);
	if (err != 0) {
		goto clean_up;
	}

	if (g->ops.secure_alloc) {
		err = g->ops.secure_alloc(g,
				       &gr->global_ctx_buffer[ATTRIBUTE_VPR],
				       attr_buffer_size);
		if (err != 0) {
			goto clean_up;
		}
	}

	nvgpu_log_info(g, "golden_image_size : %d",
		   gr->ctx_vars.golden_image_size);

	err = gk20a_gr_alloc_ctx_buffer(g,
					&gr->global_ctx_buffer[GOLDEN_CTX],
					gr->ctx_vars.golden_image_size);
	if (err != 0) {
		goto clean_up;
	}

	nvgpu_log_info(g, "priv_access_map_size : %d",
		   gr->ctx_vars.priv_access_map_size);

	err = gk20a_gr_alloc_ctx_buffer(g,
					&gr->global_ctx_buffer[PRIV_ACCESS_MAP],
					gr->ctx_vars.priv_access_map_size);

	if (err != 0) {
		goto clean_up;
	}

#ifdef CONFIG_GK20A_CTXSW_TRACE
	nvgpu_log_info(g, "fecs_trace_buffer_size : %d",
		   gr->ctx_vars.fecs_trace_buffer_size);

	err = nvgpu_dma_alloc_sys(g,
			gr->ctx_vars.fecs_trace_buffer_size,
			&gr->global_ctx_buffer[FECS_TRACE_BUFFER].mem);
	if (err != 0) {
		goto clean_up;
	}

	gr->global_ctx_buffer[FECS_TRACE_BUFFER].destroy =
			 gk20a_gr_destroy_ctx_buffer;
#endif

	nvgpu_log_fn(g, "done");
	return 0;

 clean_up:
	nvgpu_err(g, "fail");
	gr_gk20a_free_global_ctx_buffers(g);
	return -ENOMEM;
}

static void gr_gk20a_unmap_global_ctx_buffers(struct gk20a *g,
					      struct vm_gk20a *vm,
					      struct nvgpu_gr_ctx *gr_ctx)
{
	u64 *g_bfr_va = gr_ctx->global_ctx_buffer_va;
	u64 *g_bfr_size = gr_ctx->global_ctx_buffer_size;
	int *g_bfr_index = gr_ctx->global_ctx_buffer_index;
	u32 i;

	nvgpu_log_fn(g, " ");

	for (i = 0; i < NR_GLOBAL_CTX_BUF_VA; i++) {
		if (g_bfr_index[i]) {
			struct nvgpu_mem *mem;

			/*
			 * Translate from VA index to buffer index to determine
			 * the correct struct nvgpu_mem to use. Handles the VPR
			 * vs non-VPR difference in context images.
			 */
			mem = &g->gr.global_ctx_buffer[g_bfr_index[i]].mem;

			nvgpu_gmmu_unmap(vm, mem, g_bfr_va[i]);
		}
	}

	memset(g_bfr_va, 0, sizeof(gr_ctx->global_ctx_buffer_va));
	memset(g_bfr_size, 0, sizeof(gr_ctx->global_ctx_buffer_size));
	memset(g_bfr_index, 0, sizeof(gr_ctx->global_ctx_buffer_index));

	gr_ctx->global_ctx_buffer_mapped = false;
}

int gr_gk20a_map_global_ctx_buffers(struct gk20a *g,
				struct channel_gk20a *c)
{
	struct tsg_gk20a *tsg;
	struct vm_gk20a *ch_vm = c->vm;
	u64 *g_bfr_va;
	u64 *g_bfr_size;
	int *g_bfr_index;
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_mem *mem;
	u64 gpu_va;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	g_bfr_va = tsg->gr_ctx.global_ctx_buffer_va;
	g_bfr_size = tsg->gr_ctx.global_ctx_buffer_size;
	g_bfr_index = tsg->gr_ctx.global_ctx_buffer_index;

	/* Circular Buffer */
	if (c->vpr &&
	    nvgpu_mem_is_valid(&gr->global_ctx_buffer[CIRCULAR_VPR].mem)) {
		mem = &gr->global_ctx_buffer[CIRCULAR_VPR].mem;
		g_bfr_index[CIRCULAR_VA] = CIRCULAR_VPR;
	} else {
		mem = &gr->global_ctx_buffer[CIRCULAR].mem;
		g_bfr_index[CIRCULAR_VA] = CIRCULAR;
	}

	gpu_va = nvgpu_gmmu_map(ch_vm, mem, mem->size,
				NVGPU_VM_MAP_CACHEABLE,
				gk20a_mem_flag_none, true, mem->aperture);
	if (gpu_va == 0ULL) {
		goto clean_up;
	}
	g_bfr_va[CIRCULAR_VA] = gpu_va;
	g_bfr_size[CIRCULAR_VA] = mem->size;

	/* Attribute Buffer */
	if (c->vpr &&
	    nvgpu_mem_is_valid(&gr->global_ctx_buffer[ATTRIBUTE_VPR].mem)) {
		mem = &gr->global_ctx_buffer[ATTRIBUTE_VPR].mem;
		g_bfr_index[ATTRIBUTE_VA] = ATTRIBUTE_VPR;
	} else {
		mem = &gr->global_ctx_buffer[ATTRIBUTE].mem;
		g_bfr_index[ATTRIBUTE_VA] = ATTRIBUTE;
	}

	gpu_va = nvgpu_gmmu_map(ch_vm, mem, mem->size,
				NVGPU_VM_MAP_CACHEABLE,
				gk20a_mem_flag_none, false, mem->aperture);
	if (gpu_va == 0ULL) {
		goto clean_up;
	}
	g_bfr_va[ATTRIBUTE_VA] = gpu_va;
	g_bfr_size[ATTRIBUTE_VA] = mem->size;

	/* Page Pool */
	if (c->vpr &&
	    nvgpu_mem_is_valid(&gr->global_ctx_buffer[PAGEPOOL_VPR].mem)) {
		mem = &gr->global_ctx_buffer[PAGEPOOL_VPR].mem;
		g_bfr_index[PAGEPOOL_VA] = PAGEPOOL_VPR;
	} else {
		mem = &gr->global_ctx_buffer[PAGEPOOL].mem;
		g_bfr_index[PAGEPOOL_VA] = PAGEPOOL;
	}

	gpu_va = nvgpu_gmmu_map(ch_vm, mem, mem->size,
				NVGPU_VM_MAP_CACHEABLE,
				gk20a_mem_flag_none, true, mem->aperture);
	if (gpu_va == 0ULL) {
		goto clean_up;
	}
	g_bfr_va[PAGEPOOL_VA] = gpu_va;
	g_bfr_size[PAGEPOOL_VA] = mem->size;

	/* Golden Image */
	mem = &gr->global_ctx_buffer[GOLDEN_CTX].mem;
	gpu_va = nvgpu_gmmu_map(ch_vm, mem, mem->size, 0,
				gk20a_mem_flag_none, true, mem->aperture);
	if (gpu_va == 0ULL) {
		goto clean_up;
	}
	g_bfr_va[GOLDEN_CTX_VA] = gpu_va;
	g_bfr_size[GOLDEN_CTX_VA] = mem->size;
	g_bfr_index[GOLDEN_CTX_VA] = GOLDEN_CTX;

	/* Priv register Access Map */
	mem = &gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem;
	gpu_va = nvgpu_gmmu_map(ch_vm, mem, mem->size, 0,
				gk20a_mem_flag_none, true, mem->aperture);
	if (gpu_va == 0ULL) {
		goto clean_up;
	}
	g_bfr_va[PRIV_ACCESS_MAP_VA] = gpu_va;
	g_bfr_size[PRIV_ACCESS_MAP_VA] = mem->size;
	g_bfr_index[PRIV_ACCESS_MAP_VA] = PRIV_ACCESS_MAP;

	tsg->gr_ctx.global_ctx_buffer_mapped = true;

#ifdef CONFIG_GK20A_CTXSW_TRACE
	/* FECS trace buffer */
	if (nvgpu_is_enabled(g, NVGPU_FECS_TRACE_VA)) {
		mem = &gr->global_ctx_buffer[FECS_TRACE_BUFFER].mem;
		gpu_va = nvgpu_gmmu_map(ch_vm, mem, mem->size, 0,
				gk20a_mem_flag_none, true, mem->aperture);
		if (!gpu_va)
			goto clean_up;
		g_bfr_va[FECS_TRACE_BUFFER_VA] = gpu_va;
		g_bfr_size[FECS_TRACE_BUFFER_VA] = mem->size;
		g_bfr_index[FECS_TRACE_BUFFER_VA] = FECS_TRACE_BUFFER;
	}
#endif

	return 0;

clean_up:
	gr_gk20a_unmap_global_ctx_buffers(g, ch_vm, &tsg->gr_ctx);

	return -ENOMEM;
}

int gr_gk20a_alloc_gr_ctx(struct gk20a *g,
			  struct nvgpu_gr_ctx *gr_ctx, struct vm_gk20a *vm,
			  u32 class,
			  u32 padding)
{
	struct gr_gk20a *gr = &g->gr;
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (gr->ctx_vars.buffer_size == 0) {
		return 0;
	}

	/* alloc channel gr ctx buffer */
	gr->ctx_vars.buffer_size = gr->ctx_vars.golden_image_size;
	gr->ctx_vars.buffer_total_size = gr->ctx_vars.golden_image_size;

	err = nvgpu_dma_alloc(g, gr->ctx_vars.buffer_total_size, &gr_ctx->mem);
	if (err != 0) {
		return err;
	}

	gr_ctx->mem.gpu_va = nvgpu_gmmu_map(vm,
					&gr_ctx->mem,
					gr_ctx->mem.size,
					0, /* not GPU-cacheable */
					gk20a_mem_flag_none, true,
					gr_ctx->mem.aperture);
	if (gr_ctx->mem.gpu_va == 0ULL) {
		goto err_free_mem;
	}

	return 0;

 err_free_mem:
	nvgpu_dma_free(g, &gr_ctx->mem);

	return err;
}

static int gr_gk20a_alloc_tsg_gr_ctx(struct gk20a *g,
			struct tsg_gk20a *tsg, u32 class, u32 padding)
{
	struct nvgpu_gr_ctx *gr_ctx = &tsg->gr_ctx;
	int err;

	if (tsg->vm == NULL) {
		nvgpu_err(tsg->g, "No address space bound");
		return -ENOMEM;
	}

	err = g->ops.gr.alloc_gr_ctx(g, gr_ctx, tsg->vm, class, padding);
	if (err != 0) {
		return err;
	}

	gr_ctx->tsgid = tsg->tsgid;

	return 0;
}

void gr_gk20a_free_gr_ctx(struct gk20a *g,
			  struct vm_gk20a *vm, struct nvgpu_gr_ctx *gr_ctx)
{
	nvgpu_log_fn(g, " ");

	if (gr_ctx->mem.gpu_va) {
		gr_gk20a_unmap_global_ctx_buffers(g, vm, gr_ctx);
		gr_gk20a_free_channel_patch_ctx(g, vm, gr_ctx);
		gr_gk20a_free_channel_pm_ctx(g, vm, gr_ctx);

		if ((g->ops.gr.dump_ctxsw_stats != NULL) &&
		     g->gr.ctx_vars.dump_ctxsw_stats_on_channel_close) {
			g->ops.gr.dump_ctxsw_stats(g, vm, gr_ctx);
		}

		nvgpu_dma_unmap_free(vm, &gr_ctx->pagepool_ctxsw_buffer);
		nvgpu_dma_unmap_free(vm, &gr_ctx->betacb_ctxsw_buffer);
		nvgpu_dma_unmap_free(vm, &gr_ctx->spill_ctxsw_buffer);
		nvgpu_dma_unmap_free(vm, &gr_ctx->preempt_ctxsw_buffer);
		nvgpu_dma_unmap_free(vm, &gr_ctx->mem);

		memset(gr_ctx, 0, sizeof(*gr_ctx));
	}
}

void gr_gk20a_free_tsg_gr_ctx(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;

	if (tsg->vm == NULL) {
		nvgpu_err(g, "No address space bound");
		return;
	}
	tsg->g->ops.gr.free_gr_ctx(g, tsg->vm, &tsg->gr_ctx);
}

u32 gr_gk20a_get_patch_slots(struct gk20a *g)
{
	return PATCH_CTX_SLOTS_PER_PAGE;
}

static int gr_gk20a_alloc_channel_patch_ctx(struct gk20a *g,
				struct channel_gk20a *c)
{
	struct tsg_gk20a *tsg;
	struct patch_desc *patch_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	u32 alloc_size;
	int err = 0;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	patch_ctx = &tsg->gr_ctx.patch_ctx;
	alloc_size = g->ops.gr.get_patch_slots(g) *
		PATCH_CTX_SLOTS_REQUIRED_PER_ENTRY;

	nvgpu_log(g, gpu_dbg_info, "patch buffer size in entries: %d",
		alloc_size);

	err = nvgpu_dma_alloc_map_sys(ch_vm,
			alloc_size * sizeof(u32), &patch_ctx->mem);
	if (err != 0) {
		return err;
	}

	nvgpu_log_fn(g, "done");
	return 0;
}

static void gr_gk20a_free_channel_patch_ctx(struct gk20a *g,
					    struct vm_gk20a *vm,
					    struct nvgpu_gr_ctx *gr_ctx)
{
	struct patch_desc *patch_ctx = &gr_ctx->patch_ctx;

	nvgpu_log_fn(g, " ");

	if (patch_ctx->mem.gpu_va) {
		nvgpu_gmmu_unmap(vm, &patch_ctx->mem,
				 patch_ctx->mem.gpu_va);
	}

	nvgpu_dma_free(g, &patch_ctx->mem);
	patch_ctx->data_count = 0;
}

static void gr_gk20a_free_channel_pm_ctx(struct gk20a *g,
					 struct vm_gk20a *vm,
					 struct nvgpu_gr_ctx *gr_ctx)
{
	struct pm_ctx_desc *pm_ctx = &gr_ctx->pm_ctx;

	nvgpu_log_fn(g, " ");

	if (pm_ctx->mem.gpu_va) {
		nvgpu_gmmu_unmap(vm, &pm_ctx->mem, pm_ctx->mem.gpu_va);

		nvgpu_dma_free(g, &pm_ctx->mem);
	}
}

int gk20a_alloc_obj_ctx(struct channel_gk20a  *c, u32 class_num, u32 flags)
{
	struct gk20a *g = c->g;
	struct nvgpu_gr_ctx *gr_ctx;
	struct tsg_gk20a *tsg = NULL;
	int err = 0;

	nvgpu_log_fn(g, " ");

	/* an address space needs to have been bound at this point.*/
	if (!gk20a_channel_as_bound(c) && (c->vm == NULL)) {
		nvgpu_err(g,
			   "not bound to address space at time"
			   " of grctx allocation");
		return -EINVAL;
	}

	if (!g->ops.gr.is_valid_class(g, class_num)) {
		nvgpu_err(g,
			   "invalid obj class 0x%x", class_num);
		err = -EINVAL;
		goto out;
	}
	c->obj_class = class_num;

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;

	if (!nvgpu_mem_is_valid(&gr_ctx->mem)) {
		tsg->vm = c->vm;
		nvgpu_vm_get(tsg->vm);
		err = gr_gk20a_alloc_tsg_gr_ctx(g, tsg,
						class_num,
						flags);
		if (err != 0) {
			nvgpu_err(g,
				"fail to allocate TSG gr ctx buffer");
			nvgpu_vm_put(tsg->vm);
			tsg->vm = NULL;
			goto out;
		}

		/* allocate patch buffer */
		if (!nvgpu_mem_is_valid(&gr_ctx->patch_ctx.mem)) {
			gr_ctx->patch_ctx.data_count = 0;
			err = gr_gk20a_alloc_channel_patch_ctx(g, c);
			if (err != 0) {
				nvgpu_err(g,
					"fail to allocate patch buffer");
				goto out;
			}
		}

		/* map global buffer to channel gpu_va and commit */
		err = g->ops.gr.map_global_ctx_buffers(g, c);
		if (err != 0) {
			nvgpu_err(g,
				"fail to map global ctx buffer");
			goto out;
		}
		g->ops.gr.commit_global_ctx_buffers(g, c, true);

		/* commit gr ctx buffer */
		err = g->ops.gr.commit_inst(c, gr_ctx->mem.gpu_va);
		if (err != 0) {
			nvgpu_err(g,
				"fail to commit gr ctx buffer");
			goto out;
		}

		/* init golden image, ELPG enabled after this is done */
		err = gr_gk20a_init_golden_ctx_image(g, c);
		if (err != 0) {
			nvgpu_err(g,
				"fail to init golden ctx image");
			goto out;
		}

		/* load golden image */
		gr_gk20a_load_golden_ctx_image(g, c);
		if (err != 0) {
			nvgpu_err(g,
				"fail to load golden ctx image");
			goto out;
		}
#ifdef CONFIG_GK20A_CTXSW_TRACE
		if (g->ops.fecs_trace.bind_channel && !c->vpr) {
			err = g->ops.fecs_trace.bind_channel(g, c);
			if (err != 0) {
				nvgpu_warn(g,
					"fail to bind channel for ctxsw trace");
			}
		}
#endif

		if (g->ops.gr.set_czf_bypass) {
			g->ops.gr.set_czf_bypass(g, c);
		}

		/* PM ctxt switch is off by default */
		gr_ctx->pm_ctx.pm_mode = ctxsw_prog_main_image_pm_mode_no_ctxsw_f();
	} else {
		/* commit gr ctx buffer */
		err = g->ops.gr.commit_inst(c, gr_ctx->mem.gpu_va);
		if (err != 0) {
			nvgpu_err(g,
				"fail to commit gr ctx buffer");
			goto out;
		}
#ifdef CONFIG_GK20A_CTXSW_TRACE
		if (g->ops.fecs_trace.bind_channel && !c->vpr) {
			err = g->ops.fecs_trace.bind_channel(g, c);
			if (err != 0) {
				nvgpu_warn(g,
					"fail to bind channel for ctxsw trace");
			}
		}
#endif
	}

	nvgpu_log_fn(g, "done");
	return 0;
out:
	/* 1. gr_ctx, patch_ctx and global ctx buffer mapping
	   can be reused so no need to release them.
	   2. golden image init and load is a one time thing so if
	   they pass, no need to undo. */
	nvgpu_err(g, "fail");
	return err;
}

static void gk20a_remove_gr_support(struct gr_gk20a *gr)
{
	struct gk20a *g = gr->g;

	nvgpu_log_fn(g, " ");

	gr_gk20a_free_cyclestats_snapshot_data(g);

	gr_gk20a_free_global_ctx_buffers(g);

	nvgpu_dma_free(g, &gr->compbit_store.mem);

	memset(&gr->compbit_store, 0, sizeof(struct compbit_store_desc));

	nvgpu_kfree(g, gr->gpc_tpc_count);
	nvgpu_kfree(g, gr->gpc_zcb_count);
	nvgpu_kfree(g, gr->gpc_ppc_count);
	nvgpu_kfree(g, gr->pes_tpc_count[0]);
	nvgpu_kfree(g, gr->pes_tpc_count[1]);
	nvgpu_kfree(g, gr->pes_tpc_mask[0]);
	nvgpu_kfree(g, gr->pes_tpc_mask[1]);
	nvgpu_kfree(g, gr->sm_to_cluster);
	nvgpu_kfree(g, gr->gpc_skip_mask);
	nvgpu_kfree(g, gr->map_tiles);
	nvgpu_kfree(g, gr->fbp_rop_l2_en_mask);
	gr->gpc_tpc_count = NULL;
	gr->gpc_zcb_count = NULL;
	gr->gpc_ppc_count = NULL;
	gr->pes_tpc_count[0] = NULL;
	gr->pes_tpc_count[1] = NULL;
	gr->pes_tpc_mask[0] = NULL;
	gr->pes_tpc_mask[1] = NULL;
	gr->gpc_skip_mask = NULL;
	gr->map_tiles = NULL;
	gr->fbp_rop_l2_en_mask = NULL;

	gr->ctx_vars.valid = false;
	nvgpu_kfree(g, gr->ctx_vars.ucode.fecs.inst.l);
	nvgpu_kfree(g, gr->ctx_vars.ucode.fecs.data.l);
	nvgpu_kfree(g, gr->ctx_vars.ucode.gpccs.inst.l);
	nvgpu_kfree(g, gr->ctx_vars.ucode.gpccs.data.l);
	nvgpu_kfree(g, gr->ctx_vars.sw_bundle_init.l);
	nvgpu_kfree(g, gr->ctx_vars.sw_veid_bundle_init.l);
	nvgpu_kfree(g, gr->ctx_vars.sw_method_init.l);
	nvgpu_kfree(g, gr->ctx_vars.sw_ctx_load.l);
	nvgpu_kfree(g, gr->ctx_vars.sw_non_ctx_load.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.sys.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.gpc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.tpc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.zcull_gpc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.ppc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.pm_sys.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.pm_gpc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.pm_tpc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.pm_ppc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.perf_sys.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.fbp.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.perf_gpc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.fbp_router.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.gpc_router.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.pm_ltc.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.pm_fbpa.l);
	nvgpu_kfree(g, gr->ctx_vars.sw_bundle64_init.l);
	nvgpu_kfree(g, gr->ctx_vars.ctxsw_regs.pm_cau.l);

	nvgpu_vfree(g, gr->ctx_vars.local_golden_image);
	gr->ctx_vars.local_golden_image = NULL;

	if (gr->ctx_vars.hwpm_ctxsw_buffer_offset_map) {
		nvgpu_big_free(g, gr->ctx_vars.hwpm_ctxsw_buffer_offset_map);
	}
	gr->ctx_vars.hwpm_ctxsw_buffer_offset_map = NULL;

	gk20a_comptag_allocator_destroy(g, &gr->comp_tags);

	nvgpu_ecc_remove_support(g);
}

static int gr_gk20a_init_gr_config(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 gpc_index, pes_index;
	u32 pes_tpc_mask;
	u32 pes_tpc_count;
	u32 pes_heavy_index;
	u32 gpc_new_skip_mask;
	u32 tmp;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);

	tmp = gk20a_readl(g, pri_ringmaster_enum_fbp_r());
	gr->num_fbps = pri_ringmaster_enum_fbp_count_v(tmp);

	tmp = gk20a_readl(g, top_num_gpcs_r());
	gr->max_gpc_count = top_num_gpcs_value_v(tmp);

	tmp = gk20a_readl(g, top_num_fbps_r());
	gr->max_fbps_count = top_num_fbps_value_v(tmp);

	gr->fbp_en_mask = g->ops.gr.get_fbp_en_mask(g);

	if (gr->fbp_rop_l2_en_mask == NULL) {
		gr->fbp_rop_l2_en_mask =
			nvgpu_kzalloc(g, gr->max_fbps_count * sizeof(u32));
		if (gr->fbp_rop_l2_en_mask == NULL) {
			goto clean_up;
		}
	} else {
		memset(gr->fbp_rop_l2_en_mask, 0, gr->max_fbps_count *
				sizeof(u32));
	}

	tmp = gk20a_readl(g, top_tpc_per_gpc_r());
	gr->max_tpc_per_gpc_count = top_tpc_per_gpc_value_v(tmp);

	gr->max_tpc_count = gr->max_gpc_count * gr->max_tpc_per_gpc_count;

	tmp = gk20a_readl(g, top_num_fbps_r());
	gr->sys_count = top_num_fbps_value_v(tmp);

	tmp = gk20a_readl(g, pri_ringmaster_enum_gpc_r());
	gr->gpc_count = pri_ringmaster_enum_gpc_count_v(tmp);

	gr->pe_count_per_gpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_PES_PER_GPC);
	if (WARN(gr->pe_count_per_gpc > GK20A_GR_MAX_PES_PER_GPC,
		 "too many pes per gpc\n")) {
		goto clean_up;
	}

	gr->max_zcull_per_gpc_count = nvgpu_get_litter_value(g, GPU_LIT_NUM_ZCULL_BANKS);

	if (gr->gpc_count == 0U) {
		nvgpu_err(g, "gpc_count==0!");
		goto clean_up;
	}

	if (gr->gpc_tpc_count == NULL) {
		gr->gpc_tpc_count = nvgpu_kzalloc(g, gr->gpc_count *
					sizeof(u32));
	} else {
		memset(gr->gpc_tpc_count, 0, gr->gpc_count *
					sizeof(u32));
	}

	if (gr->gpc_tpc_mask == NULL) {
		gr->gpc_tpc_mask = nvgpu_kzalloc(g, gr->max_gpc_count *
					sizeof(u32));
	} else {
		memset(gr->gpc_tpc_mask, 0,  gr->max_gpc_count *
					sizeof(u32));
	}

	if (gr->gpc_zcb_count == NULL) {
		gr->gpc_zcb_count = nvgpu_kzalloc(g, gr->gpc_count *
					sizeof(u32));
	} else {
		memset(gr->gpc_zcb_count, 0, gr->gpc_count *
					sizeof(u32));
	}

	if (gr->gpc_ppc_count == NULL) {
		gr->gpc_ppc_count = nvgpu_kzalloc(g, gr->gpc_count *
					sizeof(u32));
	} else {
		memset(gr->gpc_ppc_count, 0, gr->gpc_count *
					sizeof(u32));
	}

	if (gr->gpc_skip_mask == NULL) {
		gr->gpc_skip_mask =
			nvgpu_kzalloc(g, gr_pd_dist_skip_table__size_1_v() *
			      4 * sizeof(u32));
	} else {
		memset(gr->gpc_skip_mask, 0, gr_pd_dist_skip_table__size_1_v() *
			      4 * sizeof(u32));
	}

	if ((gr->gpc_tpc_count == NULL) || (gr->gpc_tpc_mask == NULL) ||
	    (gr->gpc_zcb_count == NULL) || (gr->gpc_ppc_count == NULL) ||
	    (gr->gpc_skip_mask == NULL)) {
		goto clean_up;
	}

	for (gpc_index = 0; gpc_index < gr->max_gpc_count; gpc_index++) {
		if (g->ops.gr.get_gpc_tpc_mask) {
			gr->gpc_tpc_mask[gpc_index] =
				g->ops.gr.get_gpc_tpc_mask(g, gpc_index);
		}
	}

	gr->ppc_count = 0;
	gr->tpc_count = 0;
	gr->zcb_count = 0;
	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		tmp = gk20a_readl(g, gr_gpc0_fs_gpc_r() +
				     gpc_stride * gpc_index);

		gr->gpc_tpc_count[gpc_index] =
			gr_gpc0_fs_gpc_num_available_tpcs_v(tmp);
		gr->tpc_count += gr->gpc_tpc_count[gpc_index];

		gr->gpc_zcb_count[gpc_index] =
			gr_gpc0_fs_gpc_num_available_zculls_v(tmp);
		gr->zcb_count += gr->gpc_zcb_count[gpc_index];

		for (pes_index = 0; pes_index < gr->pe_count_per_gpc; pes_index++) {
			if (gr->pes_tpc_count[pes_index] == NULL) {
				gr->pes_tpc_count[pes_index] =
					nvgpu_kzalloc(g, gr->gpc_count *
						      sizeof(u32));
				gr->pes_tpc_mask[pes_index] =
					nvgpu_kzalloc(g, gr->gpc_count *
						      sizeof(u32));
				if ((gr->pes_tpc_count[pes_index] == NULL) ||
				    (gr->pes_tpc_mask[pes_index] == NULL)) {
					goto clean_up;
				}
			}

			tmp = gk20a_readl(g,
				gr_gpc0_gpm_pd_pes_tpc_id_mask_r(pes_index) +
				gpc_index * gpc_stride);

			pes_tpc_mask = gr_gpc0_gpm_pd_pes_tpc_id_mask_mask_v(tmp);
			pes_tpc_count = count_bits(pes_tpc_mask);

			/* detect PES presence by seeing if there are
			 * TPCs connected to it.
			 */
			if (pes_tpc_count != 0) {
				gr->gpc_ppc_count[gpc_index]++;
			}

			gr->pes_tpc_count[pes_index][gpc_index] = pes_tpc_count;
			gr->pes_tpc_mask[pes_index][gpc_index] = pes_tpc_mask;
		}

		gr->ppc_count += gr->gpc_ppc_count[gpc_index];

		gpc_new_skip_mask = 0;
		if (gr->pe_count_per_gpc > 1 &&
		    gr->pes_tpc_count[0][gpc_index] +
		    gr->pes_tpc_count[1][gpc_index] == 5) {
			pes_heavy_index =
				gr->pes_tpc_count[0][gpc_index] >
				gr->pes_tpc_count[1][gpc_index] ? 0 : 1;

			gpc_new_skip_mask =
				gr->pes_tpc_mask[pes_heavy_index][gpc_index] ^
				   (gr->pes_tpc_mask[pes_heavy_index][gpc_index] &
				   (gr->pes_tpc_mask[pes_heavy_index][gpc_index] - 1));

		} else if (gr->pe_count_per_gpc > 1 &&
			   (gr->pes_tpc_count[0][gpc_index] +
			    gr->pes_tpc_count[1][gpc_index] == 4) &&
			   (gr->pes_tpc_count[0][gpc_index] !=
			    gr->pes_tpc_count[1][gpc_index])) {
				pes_heavy_index =
				    gr->pes_tpc_count[0][gpc_index] >
				    gr->pes_tpc_count[1][gpc_index] ? 0 : 1;

			gpc_new_skip_mask =
				gr->pes_tpc_mask[pes_heavy_index][gpc_index] ^
				   (gr->pes_tpc_mask[pes_heavy_index][gpc_index] &
				   (gr->pes_tpc_mask[pes_heavy_index][gpc_index] - 1));
		}
		gr->gpc_skip_mask[gpc_index] = gpc_new_skip_mask;
	}

	/* allocate for max tpc per gpc */
	if (gr->sm_to_cluster == NULL) {
		gr->sm_to_cluster = nvgpu_kzalloc(g, gr->gpc_count *
					gr->max_tpc_per_gpc_count *
					sm_per_tpc * sizeof(struct sm_info));
	} else {
		memset(gr->sm_to_cluster, 0, gr->gpc_count *
					gr->max_tpc_per_gpc_count *
					sm_per_tpc * sizeof(struct sm_info));
	}
	gr->no_of_sm = 0;

	nvgpu_log_info(g, "fbps: %d", gr->num_fbps);
	nvgpu_log_info(g, "max_gpc_count: %d", gr->max_gpc_count);
	nvgpu_log_info(g, "max_fbps_count: %d", gr->max_fbps_count);
	nvgpu_log_info(g, "max_tpc_per_gpc_count: %d", gr->max_tpc_per_gpc_count);
	nvgpu_log_info(g, "max_zcull_per_gpc_count: %d", gr->max_zcull_per_gpc_count);
	nvgpu_log_info(g, "max_tpc_count: %d", gr->max_tpc_count);
	nvgpu_log_info(g, "sys_count: %d", gr->sys_count);
	nvgpu_log_info(g, "gpc_count: %d", gr->gpc_count);
	nvgpu_log_info(g, "pe_count_per_gpc: %d", gr->pe_count_per_gpc);
	nvgpu_log_info(g, "tpc_count: %d", gr->tpc_count);
	nvgpu_log_info(g, "ppc_count: %d", gr->ppc_count);

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		nvgpu_log_info(g, "gpc_tpc_count[%d] : %d",
			   gpc_index, gr->gpc_tpc_count[gpc_index]);
	}
	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		nvgpu_log_info(g, "gpc_zcb_count[%d] : %d",
			   gpc_index, gr->gpc_zcb_count[gpc_index]);
	}
	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		nvgpu_log_info(g, "gpc_ppc_count[%d] : %d",
			   gpc_index, gr->gpc_ppc_count[gpc_index]);
	}
	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		nvgpu_log_info(g, "gpc_skip_mask[%d] : %d",
			   gpc_index, gr->gpc_skip_mask[gpc_index]);
	}
	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		for (pes_index = 0;
		     pes_index < gr->pe_count_per_gpc;
		     pes_index++) {
			nvgpu_log_info(g, "pes_tpc_count[%d][%d] : %d",
				   pes_index, gpc_index,
				   gr->pes_tpc_count[pes_index][gpc_index]);
		}
	}

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		for (pes_index = 0;
		     pes_index < gr->pe_count_per_gpc;
		     pes_index++) {
			nvgpu_log_info(g, "pes_tpc_mask[%d][%d] : %d",
				   pes_index, gpc_index,
				   gr->pes_tpc_mask[pes_index][gpc_index]);
		}
	}

	g->ops.gr.bundle_cb_defaults(g);
	g->ops.gr.cb_size_default(g);
	g->ops.gr.calc_global_ctx_buffer_size(g);
	gr->timeslice_mode = gr_gpcs_ppcs_cbm_cfg_timeslice_mode_enable_v();

	nvgpu_log_info(g, "bundle_cb_default_size: %d",
		   gr->bundle_cb_default_size);
	nvgpu_log_info(g, "min_gpm_fifo_depth: %d", gr->min_gpm_fifo_depth);
	nvgpu_log_info(g, "bundle_cb_token_limit: %d", gr->bundle_cb_token_limit);
	nvgpu_log_info(g, "attrib_cb_default_size: %d",
		   gr->attrib_cb_default_size);
	nvgpu_log_info(g, "attrib_cb_size: %d", gr->attrib_cb_size);
	nvgpu_log_info(g, "alpha_cb_default_size: %d", gr->alpha_cb_default_size);
	nvgpu_log_info(g, "alpha_cb_size: %d", gr->alpha_cb_size);
	nvgpu_log_info(g, "timeslice_mode: %d", gr->timeslice_mode);

	return 0;

clean_up:
	return -ENOMEM;
}

static u32 prime_set[18] = {
	2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61 };

static int gr_gk20a_init_map_tiles(struct gk20a *g, struct gr_gk20a *gr)
{
	s32 comm_denom;
	s32 mul_factor;
	s32 *init_frac = NULL;
	s32 *init_err = NULL;
	s32 *run_err = NULL;
	s32 *sorted_num_tpcs = NULL;
	s32 *sorted_to_unsorted_gpc_map = NULL;
	u32 gpc_index;
	u32 gpc_mark = 0;
	u32 num_tpc;
	u32 max_tpc_count = 0;
	u32 swap;
	u32 tile_count;
	u32 index;
	bool delete_map = false;
	bool gpc_sorted;
	int ret = 0;
	int num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);
	int num_tpc_per_gpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_TPC_PER_GPC);
	int map_tile_count = num_gpcs * num_tpc_per_gpc;

	init_frac = nvgpu_kzalloc(g, num_gpcs * sizeof(s32));
	init_err = nvgpu_kzalloc(g, num_gpcs * sizeof(s32));
	run_err = nvgpu_kzalloc(g, num_gpcs * sizeof(s32));
	sorted_num_tpcs =
		nvgpu_kzalloc(g, num_gpcs * num_tpc_per_gpc * sizeof(s32));
	sorted_to_unsorted_gpc_map =
		nvgpu_kzalloc(g, num_gpcs * sizeof(s32));

	if (!((init_frac != NULL) &&
	      (init_err != NULL) &&
	      (run_err != NULL) &&
	      (sorted_num_tpcs != NULL) &&
	      (sorted_to_unsorted_gpc_map != NULL))) {
		ret = -ENOMEM;
		goto clean_up;
	}

	gr->map_row_offset = INVALID_SCREEN_TILE_ROW_OFFSET;

	if (gr->tpc_count == 3) {
		gr->map_row_offset = 2;
	} else if (gr->tpc_count < 3) {
		gr->map_row_offset = 1;
	} else {
		gr->map_row_offset = 3;

		for (index = 1; index < 18; index++) {
			u32 prime = prime_set[index];
			if ((gr->tpc_count % prime) != 0) {
				gr->map_row_offset = prime;
				break;
			}
		}
	}

	switch (gr->tpc_count) {
	case 15:
		gr->map_row_offset = 6;
		break;
	case 14:
		gr->map_row_offset = 5;
		break;
	case 13:
		gr->map_row_offset = 2;
		break;
	case 11:
		gr->map_row_offset = 7;
		break;
	case 10:
		gr->map_row_offset = 6;
		break;
	case 7:
	case 5:
		gr->map_row_offset = 1;
		break;
	default:
		break;
	}

	if (gr->map_tiles) {
		if (gr->map_tile_count != gr->tpc_count) {
			delete_map = true;
		}

		for (tile_count = 0; tile_count < gr->map_tile_count; tile_count++) {
			if (gr_gk20a_get_map_tile_count(gr, tile_count)
					>= gr->tpc_count) {
				delete_map = true;
			}
		}

		if (delete_map) {
			nvgpu_kfree(g, gr->map_tiles);
			gr->map_tiles = NULL;
			gr->map_tile_count = 0;
		}
	}

	if (gr->map_tiles == NULL) {
		gr->map_tiles = nvgpu_kzalloc(g, map_tile_count * sizeof(u8));
		if (gr->map_tiles == NULL) {
			ret = -ENOMEM;
			goto clean_up;
		}
		gr->map_tile_count = map_tile_count;

		for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
			sorted_num_tpcs[gpc_index] = gr->gpc_tpc_count[gpc_index];
			sorted_to_unsorted_gpc_map[gpc_index] = gpc_index;
		}

		gpc_sorted = false;
		while (!gpc_sorted) {
			gpc_sorted = true;
			for (gpc_index = 0; gpc_index < gr->gpc_count - 1; gpc_index++) {
				if (sorted_num_tpcs[gpc_index + 1] > sorted_num_tpcs[gpc_index]) {
					gpc_sorted = false;
					swap = sorted_num_tpcs[gpc_index];
					sorted_num_tpcs[gpc_index] = sorted_num_tpcs[gpc_index + 1];
					sorted_num_tpcs[gpc_index + 1] = swap;
					swap = sorted_to_unsorted_gpc_map[gpc_index];
					sorted_to_unsorted_gpc_map[gpc_index] =
						sorted_to_unsorted_gpc_map[gpc_index + 1];
					sorted_to_unsorted_gpc_map[gpc_index + 1] = swap;
				}
			}
		}

		for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
			if (gr->gpc_tpc_count[gpc_index] > max_tpc_count) {
				max_tpc_count = gr->gpc_tpc_count[gpc_index];
			}
		}

		mul_factor = gr->gpc_count * max_tpc_count;
		if (mul_factor & 0x1) {
			mul_factor = 2;
		} else {
			mul_factor = 1;
		}

		comm_denom = gr->gpc_count * max_tpc_count * mul_factor;

		for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
			num_tpc = sorted_num_tpcs[gpc_index];

			init_frac[gpc_index] = num_tpc * gr->gpc_count * mul_factor;

			if (num_tpc != 0) {
				init_err[gpc_index] = gpc_index * max_tpc_count * mul_factor - comm_denom/2;
			} else {
				init_err[gpc_index] = 0;
			}

			run_err[gpc_index] = init_frac[gpc_index] + init_err[gpc_index];
		}

		while (gpc_mark < gr->tpc_count) {
			for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
				if ((run_err[gpc_index] * 2) >= comm_denom) {
					gr->map_tiles[gpc_mark++] = (u8)sorted_to_unsorted_gpc_map[gpc_index];
					run_err[gpc_index] += init_frac[gpc_index] - comm_denom;
				} else {
					run_err[gpc_index] += init_frac[gpc_index];
				}
			}
		}
	}

clean_up:
	nvgpu_kfree(g, init_frac);
	nvgpu_kfree(g, init_err);
	nvgpu_kfree(g, run_err);
	nvgpu_kfree(g, sorted_num_tpcs);
	nvgpu_kfree(g, sorted_to_unsorted_gpc_map);

	if (ret) {
		nvgpu_err(g, "fail");
	} else {
		nvgpu_log_fn(g, "done");
	}

	return ret;
}

static int gr_gk20a_init_zcull(struct gk20a *g, struct gr_gk20a *gr)
{
	struct gr_zcull_gk20a *zcull = &gr->zcull;

	zcull->aliquot_width = gr->tpc_count * 16;
	zcull->aliquot_height = 16;

	zcull->width_align_pixels = gr->tpc_count * 16;
	zcull->height_align_pixels = 32;

	zcull->aliquot_size =
		zcull->aliquot_width * zcull->aliquot_height;

	/* assume no floor sweeping since we only have 1 tpc in 1 gpc */
	zcull->pixel_squares_by_aliquots =
		gr->zcb_count * 16 * 16 * gr->tpc_count /
		(gr->gpc_count * gr->gpc_tpc_count[0]);

	zcull->total_aliquots =
		gr_gpc0_zcull_total_ram_size_num_aliquots_f(
			gk20a_readl(g, gr_gpc0_zcull_total_ram_size_r()));

	return 0;
}

u32 gr_gk20a_get_ctxsw_zcull_size(struct gk20a *g, struct gr_gk20a *gr)
{
	/* assuming gr has already been initialized */
	return gr->ctx_vars.zcull_ctxsw_image_size;
}

int gr_gk20a_bind_ctxsw_zcull(struct gk20a *g, struct gr_gk20a *gr,
			struct channel_gk20a *c, u64 zcull_va, u32 mode)
{
	struct tsg_gk20a *tsg;
	struct zcull_ctx_desc *zcull_ctx;

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	zcull_ctx = &tsg->gr_ctx.zcull_ctx;
	zcull_ctx->ctx_sw_mode = mode;
	zcull_ctx->gpu_va = zcull_va;

	/* TBD: don't disable channel in sw method processing */
	return gr_gk20a_ctx_zcull_setup(g, c);
}

int gr_gk20a_get_zcull_info(struct gk20a *g, struct gr_gk20a *gr,
			struct gr_zcull_info *zcull_params)
{
	struct gr_zcull_gk20a *zcull = &gr->zcull;

	zcull_params->width_align_pixels = zcull->width_align_pixels;
	zcull_params->height_align_pixels = zcull->height_align_pixels;
	zcull_params->pixel_squares_by_aliquots =
		zcull->pixel_squares_by_aliquots;
	zcull_params->aliquot_total = zcull->total_aliquots;

	zcull_params->region_byte_multiplier =
		gr->gpc_count * gr_zcull_bytes_per_aliquot_per_gpu_v();
	zcull_params->region_header_size =
		nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS) *
		gr_zcull_save_restore_header_bytes_per_gpc_v();

	zcull_params->subregion_header_size =
		nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS) *
		gr_zcull_save_restore_subregion_header_bytes_per_gpc_v();

	zcull_params->subregion_width_align_pixels =
		gr->tpc_count * gr_gpc0_zcull_zcsize_width_subregion__multiple_v();
	zcull_params->subregion_height_align_pixels =
		gr_gpc0_zcull_zcsize_height_subregion__multiple_v();
	zcull_params->subregion_count = gr_zcull_subregion_qty_v();

	return 0;
}

int gr_gk20a_add_zbc_color(struct gk20a *g, struct gr_gk20a *gr,
			   struct zbc_entry *color_val, u32 index)
{
	u32 i;

	/* update l2 table */
	g->ops.ltc.set_zbc_color_entry(g, color_val, index);

	/* update ds table */
	gk20a_writel(g, gr_ds_zbc_color_r_r(),
		gr_ds_zbc_color_r_val_f(color_val->color_ds[0]));
	gk20a_writel(g, gr_ds_zbc_color_g_r(),
		gr_ds_zbc_color_g_val_f(color_val->color_ds[1]));
	gk20a_writel(g, gr_ds_zbc_color_b_r(),
		gr_ds_zbc_color_b_val_f(color_val->color_ds[2]));
	gk20a_writel(g, gr_ds_zbc_color_a_r(),
		gr_ds_zbc_color_a_val_f(color_val->color_ds[3]));

	gk20a_writel(g, gr_ds_zbc_color_fmt_r(),
		gr_ds_zbc_color_fmt_val_f(color_val->format));

	gk20a_writel(g, gr_ds_zbc_tbl_index_r(),
		gr_ds_zbc_tbl_index_val_f(index + GK20A_STARTOF_ZBC_TABLE));

	/* trigger the write */
	gk20a_writel(g, gr_ds_zbc_tbl_ld_r(),
		gr_ds_zbc_tbl_ld_select_c_f() |
		gr_ds_zbc_tbl_ld_action_write_f() |
		gr_ds_zbc_tbl_ld_trigger_active_f());

	/* update local copy */
	for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
		gr->zbc_col_tbl[index].color_l2[i] = color_val->color_l2[i];
		gr->zbc_col_tbl[index].color_ds[i] = color_val->color_ds[i];
	}
	gr->zbc_col_tbl[index].format = color_val->format;
	gr->zbc_col_tbl[index].ref_cnt++;

	return 0;
}

int gr_gk20a_add_zbc_depth(struct gk20a *g, struct gr_gk20a *gr,
			   struct zbc_entry *depth_val, u32 index)
{
	/* update l2 table */
	g->ops.ltc.set_zbc_depth_entry(g, depth_val, index);

	/* update ds table */
	gk20a_writel(g, gr_ds_zbc_z_r(),
		gr_ds_zbc_z_val_f(depth_val->depth));

	gk20a_writel(g, gr_ds_zbc_z_fmt_r(),
		gr_ds_zbc_z_fmt_val_f(depth_val->format));

	gk20a_writel(g, gr_ds_zbc_tbl_index_r(),
		gr_ds_zbc_tbl_index_val_f(index + GK20A_STARTOF_ZBC_TABLE));

	/* trigger the write */
	gk20a_writel(g, gr_ds_zbc_tbl_ld_r(),
		gr_ds_zbc_tbl_ld_select_z_f() |
		gr_ds_zbc_tbl_ld_action_write_f() |
		gr_ds_zbc_tbl_ld_trigger_active_f());

	/* update local copy */
	gr->zbc_dep_tbl[index].depth = depth_val->depth;
	gr->zbc_dep_tbl[index].format = depth_val->format;
	gr->zbc_dep_tbl[index].ref_cnt++;

	return 0;
}

void gr_gk20a_pmu_save_zbc(struct gk20a *g, u32 entries)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_engine_info_gk20a *gr_info = NULL;
	u32 ret;
	u32 engine_id;

	engine_id = gk20a_fifo_get_gr_engine_id(g);
	gr_info = (f->engine_info + engine_id);

	ret = gk20a_fifo_disable_engine_activity(g, gr_info, true);
	if (ret) {
		nvgpu_err(g,
			"failed to disable gr engine activity");
		return;
	}

	ret = g->ops.gr.wait_empty(g, gk20a_get_gr_idle_timeout(g),
				   GR_IDLE_CHECK_DEFAULT);
	if (ret) {
		nvgpu_err(g,
			"failed to idle graphics");
		goto clean_up;
	}

	/* update zbc */
	g->ops.gr.pmu_save_zbc(g, entries);

clean_up:
	ret = gk20a_fifo_enable_engine_activity(g, gr_info);
	if (ret) {
		nvgpu_err(g,
			"failed to enable gr engine activity");
	}
}

int gr_gk20a_add_zbc(struct gk20a *g, struct gr_gk20a *gr,
		     struct zbc_entry *zbc_val)
{
	struct zbc_color_table *c_tbl;
	struct zbc_depth_table *d_tbl;
	u32 i;
	int ret = -ENOSPC;
	bool added = false;
	u32 entries;

	/* no endian swap ? */

	nvgpu_mutex_acquire(&gr->zbc_lock);
	switch (zbc_val->type) {
	case GK20A_ZBC_TYPE_COLOR:
		/* search existing tables */
		for (i = 0; i < gr->max_used_color_index; i++) {

			c_tbl = &gr->zbc_col_tbl[i];

			if ((c_tbl->ref_cnt != 0U) &&
			    (c_tbl->format == zbc_val->format) &&
			    (memcmp(c_tbl->color_ds, zbc_val->color_ds,
				sizeof(zbc_val->color_ds)) == 0) &&
			    (memcmp(c_tbl->color_l2, zbc_val->color_l2,
				sizeof(zbc_val->color_l2)) == 0)) {

				added = true;
				c_tbl->ref_cnt++;
				ret = 0;
				break;
			}
		}
		/* add new table */
		if (!added &&
		    gr->max_used_color_index < GK20A_ZBC_TABLE_SIZE) {

			c_tbl =
			    &gr->zbc_col_tbl[gr->max_used_color_index];
			WARN_ON(c_tbl->ref_cnt != 0);

			ret = g->ops.gr.add_zbc_color(g, gr,
				zbc_val, gr->max_used_color_index);

			if (ret == 0) {
				gr->max_used_color_index++;
			}
		}
		break;
	case GK20A_ZBC_TYPE_DEPTH:
		/* search existing tables */
		for (i = 0; i < gr->max_used_depth_index; i++) {

			d_tbl = &gr->zbc_dep_tbl[i];

			if ((d_tbl->ref_cnt != 0U) &&
			    (d_tbl->depth == zbc_val->depth) &&
			    (d_tbl->format == zbc_val->format)) {
				added = true;
				d_tbl->ref_cnt++;
				ret = 0;
				break;
			}
		}
		/* add new table */
		if (!added &&
		    gr->max_used_depth_index < GK20A_ZBC_TABLE_SIZE) {

			d_tbl =
			    &gr->zbc_dep_tbl[gr->max_used_depth_index];
			WARN_ON(d_tbl->ref_cnt != 0);

			ret = g->ops.gr.add_zbc_depth(g, gr,
				zbc_val, gr->max_used_depth_index);

			if (ret == 0) {
				gr->max_used_depth_index++;
			}
		}
		break;
	case T19X_ZBC:
		if (g->ops.gr.add_zbc_type_s) {
			added =  g->ops.gr.add_zbc_type_s(g, gr, zbc_val, &ret);
		} else {
			nvgpu_err(g,
			"invalid zbc table type %d", zbc_val->type);
			ret = -EINVAL;
			goto err_mutex;
		}
		break;
	default:
		nvgpu_err(g,
			"invalid zbc table type %d", zbc_val->type);
		ret = -EINVAL;
		goto err_mutex;
	}

	if (!added && ret == 0) {
		/* update zbc for elpg only when new entry is added */
		entries = max(gr->max_used_color_index,
					gr->max_used_depth_index);
		g->ops.gr.pmu_save_zbc(g, entries);
	}

err_mutex:
	nvgpu_mutex_release(&gr->zbc_lock);
	return ret;
}

/* get a zbc table entry specified by index
 * return table size when type is invalid */
int gr_gk20a_query_zbc(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params)
{
	u32 index = query_params->index_size;
	u32 i;

	switch (query_params->type) {
	case GK20A_ZBC_TYPE_INVALID:
		query_params->index_size = GK20A_ZBC_TABLE_SIZE;
		break;
	case GK20A_ZBC_TYPE_COLOR:
		if (index >= GK20A_ZBC_TABLE_SIZE) {
			nvgpu_err(g,
				"invalid zbc color table index");
			return -EINVAL;
		}

		nvgpu_speculation_barrier();
		for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
			query_params->color_l2[i] =
				gr->zbc_col_tbl[index].color_l2[i];
			query_params->color_ds[i] =
				gr->zbc_col_tbl[index].color_ds[i];
		}
		query_params->format = gr->zbc_col_tbl[index].format;
		query_params->ref_cnt = gr->zbc_col_tbl[index].ref_cnt;
		break;
	case GK20A_ZBC_TYPE_DEPTH:
		if (index >= GK20A_ZBC_TABLE_SIZE) {
			nvgpu_err(g,
				"invalid zbc depth table index");
			return -EINVAL;
		}

		nvgpu_speculation_barrier();
		query_params->depth = gr->zbc_dep_tbl[index].depth;
		query_params->format = gr->zbc_dep_tbl[index].format;
		query_params->ref_cnt = gr->zbc_dep_tbl[index].ref_cnt;
		break;
	case T19X_ZBC:
		if (g->ops.gr.zbc_s_query_table) {
			return g->ops.gr.zbc_s_query_table(g, gr,
					 query_params);
		} else {
			nvgpu_err(g,
				"invalid zbc table type");
			return -EINVAL;
		}
		break;
	default:
		nvgpu_err(g,
				"invalid zbc table type");
		return -EINVAL;
	}

	return 0;
}

static int gr_gk20a_load_zbc_table(struct gk20a *g, struct gr_gk20a *gr)
{
	unsigned int i;
	int ret;

	for (i = 0; i < gr->max_used_color_index; i++) {
		struct zbc_color_table *c_tbl = &gr->zbc_col_tbl[i];
		struct zbc_entry zbc_val;

		zbc_val.type = GK20A_ZBC_TYPE_COLOR;
		memcpy(zbc_val.color_ds,
		       c_tbl->color_ds, sizeof(zbc_val.color_ds));
		memcpy(zbc_val.color_l2,
		       c_tbl->color_l2, sizeof(zbc_val.color_l2));
		zbc_val.format = c_tbl->format;

		ret = g->ops.gr.add_zbc_color(g, gr, &zbc_val, i);

		if (ret) {
			return ret;
		}
	}
	for (i = 0; i < gr->max_used_depth_index; i++) {
		struct zbc_depth_table *d_tbl = &gr->zbc_dep_tbl[i];
		struct zbc_entry zbc_val;

		zbc_val.type = GK20A_ZBC_TYPE_DEPTH;
		zbc_val.depth = d_tbl->depth;
		zbc_val.format = d_tbl->format;

		ret = g->ops.gr.add_zbc_depth(g, gr, &zbc_val, i);
		if (ret) {
			return ret;
		}
	}

	if (g->ops.gr.load_zbc_s_tbl) {
		ret = g->ops.gr.load_zbc_s_tbl(g, gr);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

int gr_gk20a_load_zbc_default_table(struct gk20a *g, struct gr_gk20a *gr)
{
	struct zbc_entry zbc_val;
	u32 i = 0;
	int err = 0;

	err = nvgpu_mutex_init(&gr->zbc_lock);
	if (err != 0) {
		nvgpu_err(g, "Error in zbc_lock mutex initialization");
		return err;
	}

	/* load default color table */
	zbc_val.type = GK20A_ZBC_TYPE_COLOR;

	/* Opaque black (i.e. solid black, fmt 0x28 = A8B8G8R8) */
	zbc_val.format = gr_ds_zbc_color_fmt_val_a8_b8_g8_r8_v();
	for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
		zbc_val.color_ds[i] = 0;
		zbc_val.color_l2[i] = 0;
	}
	zbc_val.color_l2[0] = 0xff000000;
	zbc_val.color_ds[3] = 0x3f800000;
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto color_fail;
	}

	/* Transparent black = (fmt 1 = zero) */
	zbc_val.format = gr_ds_zbc_color_fmt_val_zero_v();
	for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
		zbc_val.color_ds[i] = 0;
		zbc_val.color_l2[i] = 0;
	}
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto color_fail;
	}

	/* Opaque white (i.e. solid white) = (fmt 2 = uniform 1) */
	zbc_val.format = gr_ds_zbc_color_fmt_val_unorm_one_v();
	for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
		zbc_val.color_ds[i] = 0x3f800000;
		zbc_val.color_l2[i] = 0xffffffff;
	}
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto color_fail;
	}

	gr->max_default_color_index = 3;

	/* load default depth table */
	zbc_val.type = GK20A_ZBC_TYPE_DEPTH;

	zbc_val.format = gr_ds_zbc_z_fmt_val_fp32_v();
	zbc_val.depth = 0x3f800000;
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto depth_fail;
	}

	zbc_val.format = gr_ds_zbc_z_fmt_val_fp32_v();
	zbc_val.depth = 0;
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto depth_fail;
	}

	gr->max_default_depth_index = 2;

	if (g->ops.gr.load_zbc_s_default_tbl) {
		err = g->ops.gr.load_zbc_s_default_tbl(g, gr);
		if (err != 0) {
			return err;
		}
	}

	return 0;

color_fail:
	nvgpu_err(g, "fail to load default zbc color table");
	return err;
depth_fail:
	nvgpu_err(g, "fail to load default zbc depth table");
	return err;
}

int _gk20a_gr_zbc_set_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_entry *zbc_val)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_engine_info_gk20a *gr_info = NULL;
	int ret;
	u32 engine_id;

	engine_id = gk20a_fifo_get_gr_engine_id(g);
	gr_info = (f->engine_info + engine_id);

	ret = gk20a_fifo_disable_engine_activity(g, gr_info, true);
	if (ret) {
		nvgpu_err(g,
			"failed to disable gr engine activity");
		return ret;
	}

	ret = g->ops.gr.wait_empty(g, gk20a_get_gr_idle_timeout(g),
				   GR_IDLE_CHECK_DEFAULT);
	if (ret) {
		nvgpu_err(g,
			"failed to idle graphics");
		goto clean_up;
	}

	ret = gr_gk20a_add_zbc(g, gr, zbc_val);

clean_up:
	if (gk20a_fifo_enable_engine_activity(g, gr_info)) {
		nvgpu_err(g,
			"failed to enable gr engine activity");
	}

	return ret;
}

int gk20a_gr_zbc_set_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_entry *zbc_val)
{
	nvgpu_log_fn(g, " ");

	return gr_gk20a_elpg_protected_call(g,
		gr_gk20a_add_zbc(g, gr, zbc_val));
}

void gr_gk20a_program_zcull_mapping(struct gk20a *g, u32 zcull_num_entries,
						u32 *zcull_map_tiles)
{
	u32 val;

	nvgpu_log_fn(g, " ");

	if (zcull_num_entries >= 8) {
		nvgpu_log_fn(g, "map0");
		val =
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_0_f(
						zcull_map_tiles[0]) |
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_1_f(
						zcull_map_tiles[1]) |
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_2_f(
						zcull_map_tiles[2]) |
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_3_f(
						zcull_map_tiles[3]) |
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_4_f(
						zcull_map_tiles[4]) |
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_5_f(
						zcull_map_tiles[5]) |
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_6_f(
						zcull_map_tiles[6]) |
		gr_gpcs_zcull_sm_in_gpc_number_map0_tile_7_f(
						zcull_map_tiles[7]);

		gk20a_writel(g, gr_gpcs_zcull_sm_in_gpc_number_map0_r(), val);
	}

	if (zcull_num_entries >= 16) {
		nvgpu_log_fn(g, "map1");
		val =
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_8_f(
						zcull_map_tiles[8]) |
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_9_f(
						zcull_map_tiles[9]) |
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_10_f(
						zcull_map_tiles[10]) |
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_11_f(
						zcull_map_tiles[11]) |
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_12_f(
						zcull_map_tiles[12]) |
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_13_f(
						zcull_map_tiles[13]) |
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_14_f(
						zcull_map_tiles[14]) |
		gr_gpcs_zcull_sm_in_gpc_number_map1_tile_15_f(
						zcull_map_tiles[15]);

		gk20a_writel(g, gr_gpcs_zcull_sm_in_gpc_number_map1_r(), val);
	}

	if (zcull_num_entries >= 24) {
		nvgpu_log_fn(g, "map2");
		val =
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_16_f(
						zcull_map_tiles[16]) |
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_17_f(
						zcull_map_tiles[17]) |
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_18_f(
						zcull_map_tiles[18]) |
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_19_f(
						zcull_map_tiles[19]) |
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_20_f(
						zcull_map_tiles[20]) |
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_21_f(
						zcull_map_tiles[21]) |
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_22_f(
						zcull_map_tiles[22]) |
		gr_gpcs_zcull_sm_in_gpc_number_map2_tile_23_f(
						zcull_map_tiles[23]);

		gk20a_writel(g, gr_gpcs_zcull_sm_in_gpc_number_map2_r(), val);
	}

	if (zcull_num_entries >= 32) {
		nvgpu_log_fn(g, "map3");
		val =
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_24_f(
						zcull_map_tiles[24]) |
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_25_f(
						zcull_map_tiles[25]) |
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_26_f(
						zcull_map_tiles[26]) |
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_27_f(
						zcull_map_tiles[27]) |
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_28_f(
						zcull_map_tiles[28]) |
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_29_f(
						zcull_map_tiles[29]) |
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_30_f(
						zcull_map_tiles[30]) |
		gr_gpcs_zcull_sm_in_gpc_number_map3_tile_31_f(
						zcull_map_tiles[31]);

		gk20a_writel(g, gr_gpcs_zcull_sm_in_gpc_number_map3_r(), val);
	}

}

static int gr_gk20a_zcull_init_hw(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 gpc_index, gpc_tpc_count, gpc_zcull_count;
	u32 *zcull_map_tiles, *zcull_bank_counters;
	u32 map_counter;
	u32 rcp_conserv;
	u32 offset;
	bool floorsweep = false;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);
	u32 num_tpc_per_gpc = nvgpu_get_litter_value(g,
						GPU_LIT_NUM_TPC_PER_GPC);
	u32 zcull_alloc_num = num_gpcs * num_tpc_per_gpc;
	u32 map_tile_count;

	if (gr->map_tiles == NULL) {
		return -1;
	}

	if (zcull_alloc_num % 8 != 0) {
		/* Total 8 fields per map reg i.e. tile_0 to tile_7*/
		zcull_alloc_num += (zcull_alloc_num % 8);
	}
	zcull_map_tiles = nvgpu_kzalloc(g, zcull_alloc_num * sizeof(u32));

	if (zcull_map_tiles == NULL) {
		nvgpu_err(g,
			"failed to allocate zcull map titles");
		return -ENOMEM;
	}

	zcull_bank_counters = nvgpu_kzalloc(g, zcull_alloc_num * sizeof(u32));

	if (zcull_bank_counters == NULL) {
		nvgpu_err(g,
			"failed to allocate zcull bank counters");
		nvgpu_kfree(g, zcull_map_tiles);
		return -ENOMEM;
	}

	for (map_counter = 0; map_counter < gr->tpc_count; map_counter++) {
		map_tile_count = gr_gk20a_get_map_tile_count(gr, map_counter);
		zcull_map_tiles[map_counter] =
			zcull_bank_counters[map_tile_count];
		zcull_bank_counters[map_tile_count]++;
	}

	if (g->ops.gr.program_zcull_mapping != NULL) {
		g->ops.gr.program_zcull_mapping(g, zcull_alloc_num,
					 zcull_map_tiles);
	}

	nvgpu_kfree(g, zcull_map_tiles);
	nvgpu_kfree(g, zcull_bank_counters);

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		gpc_tpc_count = gr->gpc_tpc_count[gpc_index];
		gpc_zcull_count = gr->gpc_zcb_count[gpc_index];

		if (gpc_zcull_count != gr->max_zcull_per_gpc_count &&
		    gpc_zcull_count < gpc_tpc_count) {
			nvgpu_err(g,
				"zcull_banks (%d) less than tpcs (%d) for gpc (%d)",
				gpc_zcull_count, gpc_tpc_count, gpc_index);
			return -EINVAL;
		}
		if (gpc_zcull_count != gr->max_zcull_per_gpc_count &&
		    gpc_zcull_count != 0) {
			floorsweep = true;
		}
	}

	/* ceil(1.0f / SM_NUM * gr_gpc0_zcull_sm_num_rcp_conservative__max_v()) */
	rcp_conserv = DIV_ROUND_UP(gr_gpc0_zcull_sm_num_rcp_conservative__max_v(),
		gr->gpc_tpc_count[0]);

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		offset = gpc_index * gpc_stride;

		if (floorsweep) {
			gk20a_writel(g, gr_gpc0_zcull_ram_addr_r() + offset,
				gr_gpc0_zcull_ram_addr_row_offset_f(gr->map_row_offset) |
				gr_gpc0_zcull_ram_addr_tiles_per_hypertile_row_per_gpc_f(
					gr->max_zcull_per_gpc_count));
		} else {
			gk20a_writel(g, gr_gpc0_zcull_ram_addr_r() + offset,
				gr_gpc0_zcull_ram_addr_row_offset_f(gr->map_row_offset) |
				gr_gpc0_zcull_ram_addr_tiles_per_hypertile_row_per_gpc_f(
					gr->gpc_tpc_count[gpc_index]));
		}

		gk20a_writel(g, gr_gpc0_zcull_fs_r() + offset,
			gr_gpc0_zcull_fs_num_active_banks_f(gr->gpc_zcb_count[gpc_index]) |
			gr_gpc0_zcull_fs_num_sms_f(gr->tpc_count));

		gk20a_writel(g, gr_gpc0_zcull_sm_num_rcp_r() + offset,
			gr_gpc0_zcull_sm_num_rcp_conservative_f(rcp_conserv));
	}

	gk20a_writel(g, gr_gpcs_ppcs_wwdx_sm_num_rcp_r(),
		gr_gpcs_ppcs_wwdx_sm_num_rcp_conservative_f(rcp_conserv));

	return 0;
}

void gk20a_gr_enable_exceptions(struct gk20a *g)
{
	gk20a_writel(g, gr_exception_r(), 0xFFFFFFFF);
	gk20a_writel(g, gr_exception_en_r(), 0xFFFFFFFF);
	gk20a_writel(g, gr_exception1_r(), 0xFFFFFFFF);
	gk20a_writel(g, gr_exception1_en_r(), 0xFFFFFFFF);
	gk20a_writel(g, gr_exception2_r(), 0xFFFFFFFF);
	gk20a_writel(g, gr_exception2_en_r(), 0xFFFFFFFF);
}

void gk20a_gr_enable_gpc_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_mask;

	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(),
			gr_gpcs_tpcs_tpccs_tpc_exception_en_tex_enabled_f() |
			gr_gpcs_tpcs_tpccs_tpc_exception_en_sm_enabled_f());

	tpc_mask =
		gr_gpcs_gpccs_gpc_exception_en_tpc_f((1 << gr->max_tpc_per_gpc_count) - 1);

	gk20a_writel(g, gr_gpcs_gpccs_gpc_exception_en_r(), tpc_mask);
}


void gr_gk20a_enable_hww_exceptions(struct gk20a *g)
{
	/* enable exceptions */
	gk20a_writel(g, gr_fe_hww_esr_r(),
		     gr_fe_hww_esr_en_enable_f() |
		     gr_fe_hww_esr_reset_active_f());
	gk20a_writel(g, gr_memfmt_hww_esr_r(),
		     gr_memfmt_hww_esr_en_enable_f() |
		     gr_memfmt_hww_esr_reset_active_f());
}

void gr_gk20a_fecs_host_int_enable(struct gk20a *g)
{
	gk20a_writel(g, gr_fecs_host_int_enable_r(),
		     gr_fecs_host_int_enable_ctxsw_intr1_enable_f() |
		     gr_fecs_host_int_enable_fault_during_ctxsw_enable_f() |
		     gr_fecs_host_int_enable_umimp_firmware_method_enable_f() |
		     gr_fecs_host_int_enable_umimp_illegal_method_enable_f() |
		     gr_fecs_host_int_enable_watchdog_enable_f());
}

static int gk20a_init_gr_setup_hw(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	struct aiv_list_gk20a *sw_ctx_load = &g->gr.ctx_vars.sw_ctx_load;
	struct av_list_gk20a *sw_method_init = &g->gr.ctx_vars.sw_method_init;
	u32 data;
	u32 last_method_data = 0;
	u32 i, err;

	nvgpu_log_fn(g, " ");

	if (g->ops.gr.init_gpc_mmu) {
		g->ops.gr.init_gpc_mmu(g);
	}

	/* load gr floorsweeping registers */
	data = gk20a_readl(g, gr_gpc0_ppc0_pes_vsc_strem_r());
	data = set_field(data, gr_gpc0_ppc0_pes_vsc_strem_master_pe_m(),
			gr_gpc0_ppc0_pes_vsc_strem_master_pe_true_f());
	gk20a_writel(g, gr_gpc0_ppc0_pes_vsc_strem_r(), data);

	gr_gk20a_zcull_init_hw(g, gr);

	if (g->ops.priv_ring.set_ppriv_timeout_settings != NULL) {
		g->ops.priv_ring.set_ppriv_timeout_settings(g);
	}

	/* enable fifo access */
	gk20a_writel(g, gr_gpfifo_ctl_r(),
		     gr_gpfifo_ctl_access_enabled_f() |
		     gr_gpfifo_ctl_semaphore_access_enabled_f());

	/* TBD: reload gr ucode when needed */

	/* enable interrupts */
	gk20a_writel(g, gr_intr_r(), 0xFFFFFFFF);
	gk20a_writel(g, gr_intr_en_r(), 0xFFFFFFFF);

	/* enable fecs error interrupts */
	g->ops.gr.fecs_host_int_enable(g);

	g->ops.gr.enable_hww_exceptions(g);
	g->ops.gr.set_hww_esr_report_mask(g);

	/* enable TPC exceptions per GPC */
	if (g->ops.gr.enable_gpc_exceptions) {
		g->ops.gr.enable_gpc_exceptions(g);
	}

	/* enable ECC for L1/SM */
	if (g->ops.gr.ecc_init_scrub_reg) {
		g->ops.gr.ecc_init_scrub_reg(g);
	}

	/* TBD: enable per BE exceptions */

	/* reset and enable exceptions */
	g->ops.gr.enable_exceptions(g);

	gr_gk20a_load_zbc_table(g, gr);

	if (g->ops.ltc.init_cbc) {
		g->ops.ltc.init_cbc(g, gr);
	}

	if (g->ops.fb.init_cbc) {
		g->ops.fb.init_cbc(g, gr);
	}

	if (g->ops.gr.disable_rd_coalesce) {
		g->ops.gr.disable_rd_coalesce(g);
	}

	/* load ctx init */
	for (i = 0; i < sw_ctx_load->count; i++) {
		gk20a_writel(g, sw_ctx_load->l[i].addr,
			     sw_ctx_load->l[i].value);
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);
	if (err != 0U) {
		goto out;
	}

	if (g->ops.gr.init_preemption_state) {
		err = g->ops.gr.init_preemption_state(g);
		if (err != 0U) {
			goto out;
		}
	}

	/* disable fe_go_idle */
	gk20a_writel(g, gr_fe_go_idle_timeout_r(),
		gr_fe_go_idle_timeout_count_disabled_f());

	/* override a few ctx state registers */
	g->ops.gr.commit_global_timeslice(g, NULL);

	/* floorsweep anything left */
	err = g->ops.gr.init_fs_state(g);
	if (err != 0U) {
		goto out;
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);
	if (err != 0U) {
		goto restore_fe_go_idle;
	}

restore_fe_go_idle:
	/* restore fe_go_idle */
	gk20a_writel(g, gr_fe_go_idle_timeout_r(),
		     gr_fe_go_idle_timeout_count_prod_f());

	if ((err != 0U) || (gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				      GR_IDLE_CHECK_DEFAULT) != 0)) {
		goto out;
	}

	/* load method init */
	if (sw_method_init->count) {
		gk20a_writel(g, gr_pri_mme_shadow_raw_data_r(),
			     sw_method_init->l[0].value);
		gk20a_writel(g, gr_pri_mme_shadow_raw_index_r(),
			     gr_pri_mme_shadow_raw_index_write_trigger_f() |
			     sw_method_init->l[0].addr);
		last_method_data = sw_method_init->l[0].value;
	}
	for (i = 1; i < sw_method_init->count; i++) {
		if (sw_method_init->l[i].value != last_method_data) {
			gk20a_writel(g, gr_pri_mme_shadow_raw_data_r(),
				sw_method_init->l[i].value);
			last_method_data = sw_method_init->l[i].value;
		}
		gk20a_writel(g, gr_pri_mme_shadow_raw_index_r(),
			gr_pri_mme_shadow_raw_index_write_trigger_f() |
			sw_method_init->l[i].addr);
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);
out:
	nvgpu_log_fn(g, "done");
	return err;
}

static int gk20a_init_gr_prepare(struct gk20a *g)
{
	u32 err = 0;

	/* reset gr engine */
	g->ops.mc.reset(g, g->ops.mc.reset_mask(g, NVGPU_UNIT_GRAPH) |
			g->ops.mc.reset_mask(g, NVGPU_UNIT_BLG) |
			g->ops.mc.reset_mask(g, NVGPU_UNIT_PERFMON));

	nvgpu_cg_init_gr_load_gating_prod(g);

	/* Disable elcg until it gets enabled later in the init*/
	nvgpu_cg_elcg_disable_no_wait(g);

	/* enable fifo access */
	gk20a_writel(g, gr_gpfifo_ctl_r(),
		gr_gpfifo_ctl_access_enabled_f() |
		gr_gpfifo_ctl_semaphore_access_enabled_f());

	if (!g->gr.ctx_vars.valid) {
		err = gr_gk20a_init_ctx_vars(g, &g->gr);
		if (err != 0U) {
			nvgpu_err(g,
				"fail to load gr init ctx");
		}
	}
	return err;
}

static int gr_gk20a_wait_mem_scrubbing(struct gk20a *g)
{
	struct nvgpu_timeout timeout;
	bool fecs_scrubbing;
	bool gpccs_scrubbing;

	nvgpu_log_fn(g, " ");

	nvgpu_timeout_init(g, &timeout,
			   CTXSW_MEM_SCRUBBING_TIMEOUT_MAX /
				CTXSW_MEM_SCRUBBING_TIMEOUT_DEFAULT,
			   NVGPU_TIMER_RETRY_TIMER);
	do {
		fecs_scrubbing = gk20a_readl(g, gr_fecs_dmactl_r()) &
			(gr_fecs_dmactl_imem_scrubbing_m() |
			 gr_fecs_dmactl_dmem_scrubbing_m());

		gpccs_scrubbing = gk20a_readl(g, gr_gpccs_dmactl_r()) &
			(gr_gpccs_dmactl_imem_scrubbing_m() |
			 gr_gpccs_dmactl_imem_scrubbing_m());

		if (!fecs_scrubbing && !gpccs_scrubbing) {
			nvgpu_log_fn(g, "done");
			return 0;
		}

		nvgpu_udelay(CTXSW_MEM_SCRUBBING_TIMEOUT_DEFAULT);
	} while (nvgpu_timeout_expired(&timeout) == 0);

	nvgpu_err(g, "Falcon mem scrubbing timeout");
	return -ETIMEDOUT;
}

static int gr_gk20a_init_ctxsw(struct gk20a *g)
{
	u32 err = 0;

	err = g->ops.gr.load_ctxsw_ucode(g);
	if (err != 0U) {
		goto out;
	}

	err = gr_gk20a_wait_ctxsw_ready(g);
	if (err != 0U) {
		goto out;
	}

out:
	if (err != 0U) {
		nvgpu_err(g, "fail");
	} else {
		nvgpu_log_fn(g, "done");
	}

	return err;
}

static int gk20a_init_gr_reset_enable_hw(struct gk20a *g)
{
	struct av_list_gk20a *sw_non_ctx_load = &g->gr.ctx_vars.sw_non_ctx_load;
	u32 i, err = 0;

	nvgpu_log_fn(g, " ");

	/* enable interrupts */
	gk20a_writel(g, gr_intr_r(), ~0);
	gk20a_writel(g, gr_intr_en_r(), ~0);

	/* load non_ctx init */
	for (i = 0; i < sw_non_ctx_load->count; i++) {
		gk20a_writel(g, sw_non_ctx_load->l[i].addr,
			sw_non_ctx_load->l[i].value);
	}

	err = gr_gk20a_wait_mem_scrubbing(g);
	if (err != 0U) {
		goto out;
	}

	err = gr_gk20a_wait_idle(g, gk20a_get_gr_idle_timeout(g),
				 GR_IDLE_CHECK_DEFAULT);
	if (err != 0U) {
		goto out;
	}

out:
	if (err != 0U) {
		nvgpu_err(g, "fail");
	} else {
		nvgpu_log_fn(g, "done");
	}

	return 0;
}

static int gr_gk20a_init_access_map(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_mem *mem = &gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem;
	u32 nr_pages =
		DIV_ROUND_UP(gr->ctx_vars.priv_access_map_size,
			     PAGE_SIZE);
	u32 *whitelist = NULL;
	int w, num_entries = 0;

	nvgpu_memset(g, mem, 0, 0, PAGE_SIZE * nr_pages);

	g->ops.gr.get_access_map(g, &whitelist, &num_entries);

	for (w = 0; w < num_entries; w++) {
		u32 map_bit, map_byte, map_shift, x;
		map_bit = whitelist[w] >> 2;
		map_byte = map_bit >> 3;
		map_shift = map_bit & 0x7; /* i.e. 0-7 */
		nvgpu_log_info(g, "access map addr:0x%x byte:0x%x bit:%d",
			       whitelist[w], map_byte, map_shift);
		x = nvgpu_mem_rd32(g, mem, map_byte / sizeof(u32));
		x |= 1 << (
			   (map_byte % sizeof(u32) * BITS_PER_BYTE)
			  + map_shift);
		nvgpu_mem_wr32(g, mem, map_byte / sizeof(u32), x);
	}

	return 0;
}

static int gk20a_init_gr_setup_sw(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (gr->sw_ready) {
		nvgpu_log_fn(g, "skip init");
		return 0;
	}

	gr->g = g;

#if defined(CONFIG_GK20A_CYCLE_STATS)
	err = nvgpu_mutex_init(&g->gr.cs_lock);
	if (err != 0) {
		nvgpu_err(g, "Error in gr.cs_lock mutex initialization");
		return err;
	}
#endif

	err = gr_gk20a_init_gr_config(g, gr);
	if (err != 0) {
		goto clean_up;
	}

	err = gr_gk20a_init_map_tiles(g, gr);
	if (err != 0) {
		goto clean_up;
	}

	if (g->ops.ltc.init_comptags) {
		err = g->ops.ltc.init_comptags(g, gr);
		if (err != 0) {
			goto clean_up;
		}
	}

	err = gr_gk20a_init_zcull(g, gr);
	if (err != 0) {
		goto clean_up;
	}

	err = g->ops.gr.alloc_global_ctx_buffers(g);
	if (err != 0) {
		goto clean_up;
	}

	err = gr_gk20a_init_access_map(g);
	if (err != 0) {
		goto clean_up;
	}

	gr_gk20a_load_zbc_default_table(g, gr);

	if (g->ops.gr.init_czf_bypass) {
		g->ops.gr.init_czf_bypass(g);
	}

	if (g->ops.gr.init_gfxp_wfi_timeout_count) {
		g->ops.gr.init_gfxp_wfi_timeout_count(g);
	}

	err = nvgpu_mutex_init(&gr->ctx_mutex);
	if (err != 0) {
		nvgpu_err(g, "Error in gr.ctx_mutex initialization");
		goto clean_up;
	}

	nvgpu_spinlock_init(&gr->ch_tlb_lock);

	gr->remove_support = gk20a_remove_gr_support;
	gr->sw_ready = true;

	err = nvgpu_ecc_init_support(g);
	if (err != 0) {
		goto clean_up;
	}

	nvgpu_log_fn(g, "done");
	return 0;

clean_up:
	nvgpu_err(g, "fail");
	gk20a_remove_gr_support(gr);
	return err;
}

static int gk20a_init_gr_bind_fecs_elpg(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	int err = 0;

	u32 size;

	nvgpu_log_fn(g, " ");

	size = 0;

	err = gr_gk20a_fecs_get_reglist_img_size(g, &size);
	if (err != 0) {
		nvgpu_err(g,
			"fail to query fecs pg buffer size");
		return err;
	}

	if (pmu->pg_buf.cpu_va == NULL) {
		err = nvgpu_dma_alloc_map_sys(vm, size, &pmu->pg_buf);
		if (err != 0) {
			nvgpu_err(g, "failed to allocate memory");
			return -ENOMEM;
		}
	}


	err = gr_gk20a_fecs_set_reglist_bind_inst(g, &mm->pmu.inst_block);
	if (err != 0) {
		nvgpu_err(g,
			"fail to bind pmu inst to gr");
		return err;
	}

	err = gr_gk20a_fecs_set_reglist_virtual_addr(g, pmu->pg_buf.gpu_va);
	if (err != 0) {
		nvgpu_err(g,
			"fail to set pg buffer pmu va");
		return err;
	}

	return err;
}

int gk20a_init_gr_support(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	g->gr.initialized = false;

	/* this is required before gr_gk20a_init_ctx_state */
	err = nvgpu_mutex_init(&g->gr.fecs_mutex);
	if (err != 0) {
		nvgpu_err(g, "Error in gr.fecs_mutex initialization");
		return err;
	}

	err = gr_gk20a_init_ctxsw(g);
	if (err != 0) {
		return err;
	}

	/* this appears query for sw states but fecs actually init
	   ramchain, etc so this is hw init */
	err = g->ops.gr.init_ctx_state(g);
	if (err != 0) {
		return err;
	}

	err = gk20a_init_gr_setup_sw(g);
	if (err != 0) {
		return err;
	}

	err = gk20a_init_gr_setup_hw(g);
	if (err != 0) {
		return err;
	}

	if (g->can_elpg) {
		err = gk20a_init_gr_bind_fecs_elpg(g);
		if (err != 0) {
			return err;
		}
	}

	nvgpu_cg_elcg_enable_no_wait(g);
	/* GR is inialized, signal possible waiters */
	g->gr.initialized = true;
	nvgpu_cond_signal(&g->gr.init_wq);

	return 0;
}

/* Wait until GR is initialized */
void gk20a_gr_wait_initialized(struct gk20a *g)
{
	NVGPU_COND_WAIT(&g->gr.init_wq, g->gr.initialized, 0);
}

#define NVA297_SET_ALPHA_CIRCULAR_BUFFER_SIZE	0x02dc
#define NVA297_SET_CIRCULAR_BUFFER_SIZE		0x1280
#define NVA297_SET_SHADER_EXCEPTIONS		0x1528
#define NVA0C0_SET_SHADER_EXCEPTIONS		0x1528

#define NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE 0

void gk20a_gr_set_shader_exceptions(struct gk20a *g, u32 data)
{
	nvgpu_log_fn(g, " ");

	if (data == NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE) {
		gk20a_writel(g,
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_r(), 0);
		gk20a_writel(g,
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_r(), 0);
	} else {
		/* setup sm warp esr report masks */
		gk20a_writel(g, gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_r(),
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_stack_error_report_f()	|
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_api_stack_error_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_ret_empty_stack_error_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_pc_wrap_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_pc_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_pc_overflow_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_immc_addr_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_reg_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_encoding_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_sph_instr_combo_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_param_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_const_addr_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_oor_reg_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_oor_addr_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_addr_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_addr_space_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_param2_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_const_addr_ldc_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_geometry_sm_error_report_f() |
			gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_divergent_report_f());

		/* setup sm global esr report mask */
		gk20a_writel(g, gr_gpcs_tpcs_sm_hww_global_esr_report_mask_r(),
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_sm_to_sm_fault_report_f() |
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_l1_error_report_f() |
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_multiple_warp_errors_report_f() |
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_physical_stack_overflow_error_report_f() |
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_bpt_int_report_f() |
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_bpt_pause_report_f() |
			gr_gpcs_tpcs_sm_hww_global_esr_report_mask_single_step_complete_report_f());
	}
}

int gk20a_enable_gr_hw(struct gk20a *g)
{
	int err;

	nvgpu_log_fn(g, " ");

	err = gk20a_init_gr_prepare(g);
	if (err != 0) {
		return err;
	}

	err = gk20a_init_gr_reset_enable_hw(g);
	if (err != 0) {
		return err;
	}

	nvgpu_log_fn(g, "done");

	return 0;
}

int gk20a_gr_reset(struct gk20a *g)
{
	int err;
	u32 size;

	g->gr.initialized = false;

	nvgpu_mutex_acquire(&g->gr.fecs_mutex);

	err = gk20a_enable_gr_hw(g);
	if (err != 0) {
		nvgpu_mutex_release(&g->gr.fecs_mutex);
		return err;
	}

	err = gk20a_init_gr_setup_hw(g);
	if (err != 0) {
		nvgpu_mutex_release(&g->gr.fecs_mutex);
		return err;
	}

	err = gr_gk20a_init_ctxsw(g);
	if (err != 0) {
		nvgpu_mutex_release(&g->gr.fecs_mutex);
		return err;
	}

	nvgpu_mutex_release(&g->gr.fecs_mutex);

	/* this appears query for sw states but fecs actually init
	   ramchain, etc so this is hw init */
	err = g->ops.gr.init_ctx_state(g);
	if (err != 0) {
		return err;
	}

	size = 0;
	err = gr_gk20a_fecs_get_reglist_img_size(g, &size);
	if (err != 0) {
		nvgpu_err(g,
			"fail to query fecs pg buffer size");
		return err;
	}

	err = gr_gk20a_fecs_set_reglist_bind_inst(g, &g->mm.pmu.inst_block);
	if (err != 0) {
		nvgpu_err(g,
			"fail to bind pmu inst to gr");
		return err;
	}

	err = gr_gk20a_fecs_set_reglist_virtual_addr(g, g->pmu.pg_buf.gpu_va);
	if (err != 0) {
		nvgpu_err(g,
			"fail to set pg buffer pmu va");
		return err;
	}

	nvgpu_cg_init_gr_load_gating_prod(g);
	nvgpu_cg_elcg_enable_no_wait(g);

	/* GR is inialized, signal possible waiters */
	g->gr.initialized = true;
	nvgpu_cond_signal(&g->gr.init_wq);

	return err;
}

static void gk20a_gr_set_error_notifier(struct gk20a *g,
		  struct gr_gk20a_isr_data *isr_data, u32 error_notifier)
{
	struct channel_gk20a *ch;
	struct tsg_gk20a *tsg;
	struct channel_gk20a *ch_tsg;

	ch = isr_data->ch;

	if (ch == NULL) {
		return;
	}

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg != NULL) {
		nvgpu_rwsem_down_read(&tsg->ch_list_lock);
		nvgpu_list_for_each_entry(ch_tsg, &tsg->ch_list,
				channel_gk20a, ch_entry) {
			if (gk20a_channel_get(ch_tsg)) {
				g->ops.fifo.set_error_notifier(ch_tsg,
					 error_notifier);
				gk20a_channel_put(ch_tsg);
			}

		}
		nvgpu_rwsem_up_read(&tsg->ch_list_lock);
	} else {
		nvgpu_err(g, "chid: %d is not bound to tsg", ch->chid);
	}
}

static int gk20a_gr_handle_semaphore_timeout_pending(struct gk20a *g,
		  struct gr_gk20a_isr_data *isr_data)
{
	nvgpu_log_fn(g, " ");
	gk20a_gr_set_error_notifier(g, isr_data,
			 NVGPU_ERR_NOTIFIER_GR_SEMAPHORE_TIMEOUT);
	nvgpu_err(g,
		   "gr semaphore timeout");
	return -EINVAL;
}

static int gk20a_gr_intr_illegal_notify_pending(struct gk20a *g,
		  struct gr_gk20a_isr_data *isr_data)
{
	nvgpu_log_fn(g, " ");
	gk20a_gr_set_error_notifier(g, isr_data,
			 NVGPU_ERR_NOTIFIER_GR_ILLEGAL_NOTIFY);
	/* This is an unrecoverable error, reset is needed */
	nvgpu_err(g,
		   "gr semaphore timeout");
	return -EINVAL;
}

static int gk20a_gr_handle_illegal_method(struct gk20a *g,
					  struct gr_gk20a_isr_data *isr_data)
{
	int ret = g->ops.gr.handle_sw_method(g, isr_data->addr,
			isr_data->class_num, isr_data->offset,
			isr_data->data_lo);
	if (ret) {
		gk20a_gr_set_error_notifier(g, isr_data,
			 NVGPU_ERR_NOTIFIER_GR_ILLEGAL_NOTIFY);
		nvgpu_err(g, "invalid method class 0x%08x"
			", offset 0x%08x address 0x%08x",
			isr_data->class_num, isr_data->offset, isr_data->addr);
	}
	return ret;
}

static int gk20a_gr_handle_illegal_class(struct gk20a *g,
					  struct gr_gk20a_isr_data *isr_data)
{
	nvgpu_log_fn(g, " ");
	gk20a_gr_set_error_notifier(g, isr_data,
			 NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY);
	nvgpu_err(g,
		   "invalid class 0x%08x, offset 0x%08x",
		   isr_data->class_num, isr_data->offset);
	return -EINVAL;
}

int gk20a_gr_handle_fecs_error(struct gk20a *g, struct channel_gk20a *ch,
					  struct gr_gk20a_isr_data *isr_data)
{
	u32 gr_fecs_intr = gk20a_readl(g, gr_fecs_host_int_status_r());
	int ret = 0;
	u32 chid = isr_data->ch != NULL ?
		isr_data->ch->chid : FIFO_INVAL_CHANNEL_ID;

	if (gr_fecs_intr == 0U) {
		return 0;
	}

	if (gr_fecs_intr & gr_fecs_host_int_status_umimp_firmware_method_f(1)) {
		gk20a_gr_set_error_notifier(g, isr_data,
			 NVGPU_ERR_NOTIFIER_FECS_ERR_UNIMP_FIRMWARE_METHOD);
		nvgpu_err(g,
			  "firmware method error 0x%08x for offset 0x%04x",
			  gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(6)),
			  isr_data->data_lo);
		ret = -1;
	} else if ((gr_fecs_intr &
			gr_fecs_host_int_status_watchdog_active_f()) != 0U) {
		/* currently, recovery is not initiated */
		nvgpu_err(g, "fecs watchdog triggered for channel %u, "
				"cannot ctxsw anymore !!", chid);
		gk20a_fecs_dump_falcon_stats(g);
	} else if ((gr_fecs_intr &
		gr_fecs_host_int_status_ctxsw_intr_f(CTXSW_INTR0)) != 0U) {
		u32 mailbox_value = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(6));

		if (mailbox_value == MAILBOX_VALUE_TIMESTAMP_BUFFER_FULL) {
			nvgpu_info(g, "ctxsw intr0 set by ucode, "
					"timestamp buffer full");
#ifdef CONFIG_GK20A_CTXSW_TRACE
			gk20a_fecs_trace_reset_buffer(g);
#else
			ret = -1;
#endif
		} else {
			nvgpu_err(g,
			  "ctxsw intr0 set by ucode, error_code: 0x%08x",
			  mailbox_value);
			ret = -1;
		}
	} else {
		nvgpu_err(g,
			"unhandled fecs error interrupt 0x%08x for channel %u",
			gr_fecs_intr, ch->chid);
		gk20a_fecs_dump_falcon_stats(g);
	}

	gk20a_writel(g, gr_fecs_host_int_clear_r(), gr_fecs_intr);
	return ret;
}

static int gk20a_gr_handle_class_error(struct gk20a *g,
				       struct gr_gk20a_isr_data *isr_data)
{
	u32 gr_class_error;
	u32 chid = isr_data->ch != NULL ?
		isr_data->ch->chid : FIFO_INVAL_CHANNEL_ID;

	nvgpu_log_fn(g, " ");

	gr_class_error =
		gr_class_error_code_v(gk20a_readl(g, gr_class_error_r()));
	gk20a_gr_set_error_notifier(g, isr_data,
			 NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY);
	nvgpu_err(g, "class error 0x%08x, offset 0x%08x,"
		"sub channel 0x%08x mme generated %d,"
		" mme pc 0x%08xdata high %d priv status %d"
		" unhandled intr 0x%08x for channel %u",
		isr_data->class_num, (isr_data->offset << 2),
		gr_trapped_addr_subch_v(isr_data->addr),
		gr_trapped_addr_mme_generated_v(isr_data->addr),
		gr_trapped_data_mme_pc_v(
			gk20a_readl(g, gr_trapped_data_mme_r())),
		gr_trapped_addr_datahigh_v(isr_data->addr),
		gr_trapped_addr_priv_v(isr_data->addr),
		gr_class_error, chid);

	nvgpu_err(g, "trapped data low 0x%08x",
		gk20a_readl(g, gr_trapped_data_lo_r()));
	if (gr_trapped_addr_datahigh_v(isr_data->addr)) {
		nvgpu_err(g, "trapped data high 0x%08x",
		gk20a_readl(g, gr_trapped_data_hi_r()));
	}

	return -EINVAL;
}

static int gk20a_gr_handle_firmware_method(struct gk20a *g,
					   struct gr_gk20a_isr_data *isr_data)
{
	u32 chid = isr_data->ch != NULL ?
		isr_data->ch->chid : FIFO_INVAL_CHANNEL_ID;

	nvgpu_log_fn(g, " ");

	gk20a_gr_set_error_notifier(g, isr_data,
			 NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY);
	nvgpu_err(g,
		   "firmware method 0x%08x, offset 0x%08x for channel %u",
		   isr_data->class_num, isr_data->offset,
		   chid);
	return -EINVAL;
}

int gk20a_gr_handle_semaphore_pending(struct gk20a *g,
				     struct gr_gk20a_isr_data *isr_data)
{
	struct channel_gk20a *ch = isr_data->ch;
	struct tsg_gk20a *tsg;

	if (ch == NULL) {
		return 0;
	}

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg != NULL) {
		g->ops.fifo.post_event_id(tsg,
			NVGPU_EVENT_ID_GR_SEMAPHORE_WRITE_AWAKEN);

		nvgpu_cond_broadcast(&ch->semaphore_wq);
	} else {
		nvgpu_err(g, "chid: %d is not bound to tsg", ch->chid);
	}

	return 0;
}

#if defined(CONFIG_GK20A_CYCLE_STATS)
static inline bool is_valid_cyclestats_bar0_offset_gk20a(struct gk20a *g,
							 u32 offset)
{
	/* support only 24-bit 4-byte aligned offsets */
	bool valid = !(offset & 0xFF000003);

	if (g->allow_all)
		return true;

	/* whitelist check */
	valid = valid &&
		is_bar0_global_offset_whitelisted_gk20a(g, offset);
	/* resource size check in case there was a problem
	 * with allocating the assumed size of bar0 */
	valid = valid && gk20a_io_valid_reg(g, offset);
	return valid;
}
#endif

int gk20a_gr_handle_notify_pending(struct gk20a *g,
					  struct gr_gk20a_isr_data *isr_data)
{
	struct channel_gk20a *ch = isr_data->ch;

#if defined(CONFIG_GK20A_CYCLE_STATS)
	void *virtual_address;
	u32 buffer_size;
	u32 offset;
	bool exit;
#endif
	if (ch == NULL || tsg_gk20a_from_ch(ch) == NULL) {
		return 0;
	}

#if defined(CONFIG_GK20A_CYCLE_STATS)
	/* GL will never use payload 0 for cycle state */
	if ((ch->cyclestate.cyclestate_buffer == NULL) || (isr_data->data_lo == 0))
		return 0;

	nvgpu_mutex_acquire(&ch->cyclestate.cyclestate_buffer_mutex);

	virtual_address = ch->cyclestate.cyclestate_buffer;
	buffer_size = ch->cyclestate.cyclestate_buffer_size;
	offset = isr_data->data_lo;
	exit = false;
	while (!exit) {
		struct share_buffer_head *sh_hdr;
		u32 min_element_size;

		/* validate offset */
		if (offset + sizeof(struct share_buffer_head) > buffer_size ||
		    offset + sizeof(struct share_buffer_head) < offset) {
			nvgpu_err(g,
				  "cyclestats buffer overrun at offset 0x%x",
				  offset);
			break;
		}

		sh_hdr = (struct share_buffer_head *)
			((char *)virtual_address + offset);

		min_element_size =
			(sh_hdr->operation == OP_END ?
			 sizeof(struct share_buffer_head) :
			 sizeof(struct gk20a_cyclestate_buffer_elem));

		/* validate sh_hdr->size */
		if (sh_hdr->size < min_element_size ||
		    offset + sh_hdr->size > buffer_size ||
		    offset + sh_hdr->size < offset) {
			nvgpu_err(g,
				  "bad cyclestate buffer header size at offset 0x%x",
				  offset);
			sh_hdr->failed = true;
			break;
		}

		switch (sh_hdr->operation) {
		case OP_END:
			exit = true;
			break;

		case BAR0_READ32:
		case BAR0_WRITE32:
		{
			struct gk20a_cyclestate_buffer_elem *op_elem =
				(struct gk20a_cyclestate_buffer_elem *)sh_hdr;
			bool valid = is_valid_cyclestats_bar0_offset_gk20a(
				g, op_elem->offset_bar0);
			u32 raw_reg;
			u64 mask_orig;
			u64 v;

			if (!valid) {
				nvgpu_err(g,
					   "invalid cycletstats op offset: 0x%x",
					   op_elem->offset_bar0);

				sh_hdr->failed = exit = true;
				break;
			}


			mask_orig =
				((1ULL <<
				  (op_elem->last_bit + 1))
				 -1)&~((1ULL <<
					op_elem->first_bit)-1);

			raw_reg =
				gk20a_readl(g,
					    op_elem->offset_bar0);

			switch (sh_hdr->operation) {
			case BAR0_READ32:
				op_elem->data =
					(raw_reg & mask_orig)
					>> op_elem->first_bit;
				break;

			case BAR0_WRITE32:
				v = 0;
				if ((unsigned int)mask_orig !=
				    (unsigned int)~0) {
					v = (unsigned int)
						(raw_reg & ~mask_orig);
				}

				v |= ((op_elem->data
				       << op_elem->first_bit)
				      & mask_orig);

				gk20a_writel(g,
					     op_elem->offset_bar0,
					     (unsigned int)v);
				break;
			default:
				/* nop ok?*/
				break;
			}
		}
		break;

		default:
			/* no operation content case */
			exit = true;
			break;
		}
		sh_hdr->completed = true;
		offset += sh_hdr->size;
	}
	nvgpu_mutex_release(&ch->cyclestate.cyclestate_buffer_mutex);
#endif
	nvgpu_log_fn(g, " ");
	nvgpu_cond_broadcast_interruptible(&ch->notifier_wq);
	return 0;
}

/* Used by sw interrupt thread to translate current ctx to chid.
 * Also used by regops to translate current ctx to chid and tsgid.
 * For performance, we don't want to go through 128 channels every time.
 * curr_ctx should be the value read from gr_fecs_current_ctx_r().
 * A small tlb is used here to cache translation.
 *
 * Returned channel must be freed with gk20a_channel_put() */
static struct channel_gk20a *gk20a_gr_get_channel_from_ctx(
	struct gk20a *g, u32 curr_ctx, u32 *curr_tsgid)
{
	struct fifo_gk20a *f = &g->fifo;
	struct gr_gk20a *gr = &g->gr;
	u32 chid = -1;
	u32 tsgid = NVGPU_INVALID_TSG_ID;
	u32 i;
	struct channel_gk20a *ret = NULL;

	/* when contexts are unloaded from GR, the valid bit is reset
	 * but the instance pointer information remains intact.
	 * This might be called from gr_isr where contexts might be
	 * unloaded. No need to check ctx_valid bit
	 */

	nvgpu_spinlock_acquire(&gr->ch_tlb_lock);

	/* check cache first */
	for (i = 0; i < GR_CHANNEL_MAP_TLB_SIZE; i++) {
		if (gr->chid_tlb[i].curr_ctx == curr_ctx) {
			chid = gr->chid_tlb[i].chid;
			tsgid = gr->chid_tlb[i].tsgid;
			ret = gk20a_channel_from_id(g, chid);
			goto unlock;
		}
	}

	/* slow path */
	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = gk20a_channel_from_id(g, chid);

		if (ch == NULL) {
			continue;
		}

		if ((u32)(nvgpu_inst_block_addr(g, &ch->inst_block) >>
					ram_in_base_shift_v()) ==
				gr_fecs_current_ctx_ptr_v(curr_ctx)) {
			tsgid = ch->tsgid;
			/* found it */
			ret = ch;
			break;
		}
		gk20a_channel_put(ch);
	}

	if (ret == NULL) {
		goto unlock;
	}

	/* add to free tlb entry */
	for (i = 0; i < GR_CHANNEL_MAP_TLB_SIZE; i++) {
		if (gr->chid_tlb[i].curr_ctx == 0) {
			gr->chid_tlb[i].curr_ctx = curr_ctx;
			gr->chid_tlb[i].chid = chid;
			gr->chid_tlb[i].tsgid = tsgid;
			goto unlock;
		}
	}

	/* no free entry, flush one */
	gr->chid_tlb[gr->channel_tlb_flush_index].curr_ctx = curr_ctx;
	gr->chid_tlb[gr->channel_tlb_flush_index].chid = chid;
	gr->chid_tlb[gr->channel_tlb_flush_index].tsgid = tsgid;

	gr->channel_tlb_flush_index =
		(gr->channel_tlb_flush_index + 1) &
		(GR_CHANNEL_MAP_TLB_SIZE - 1);

unlock:
	nvgpu_spinlock_release(&gr->ch_tlb_lock);
	if (curr_tsgid) {
		*curr_tsgid = tsgid;
	}
	return ret;
}

int gk20a_gr_lock_down_sm(struct gk20a *g,
			 u32 gpc, u32 tpc, u32 sm, u32 global_esr_mask,
			 bool check_errors)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);
	u32 dbgr_control0;

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"GPC%d TPC%d SM%d: assert stop trigger", gpc, tpc, sm);

	/* assert stop trigger */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_control0_r() + offset);
	dbgr_control0 |= gr_gpc0_tpc0_sm_dbgr_control0_stop_trigger_enable_f();
	gk20a_writel(g,
		gr_gpc0_tpc0_sm_dbgr_control0_r() + offset, dbgr_control0);

	return g->ops.gr.wait_for_sm_lock_down(g, gpc, tpc, sm, global_esr_mask,
			check_errors);
}

bool gk20a_gr_sm_debugger_attached(struct gk20a *g)
{
	u32 dbgr_control0 = gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_control0_r());

	/* check if an sm debugger is attached.
	 * assumption: all SMs will have debug mode enabled/disabled
	 * uniformly. */
	if (gr_gpc0_tpc0_sm_dbgr_control0_debugger_mode_v(dbgr_control0) ==
			gr_gpc0_tpc0_sm_dbgr_control0_debugger_mode_on_v()) {
		return true;
	}

	return false;
}

int gr_gk20a_handle_sm_exception(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
		bool *post_event, struct channel_gk20a *fault_ch,
		u32 *hww_global_esr)
{
	int ret = 0;
	bool do_warp_sync = false, early_exit = false, ignore_debugger = false;
	bool disable_sm_exceptions = true;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);
	bool sm_debugger_attached;
	u32 global_esr, warp_esr, global_mask;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");

	sm_debugger_attached = g->ops.gr.sm_debugger_attached(g);

	global_esr = g->ops.gr.get_sm_hww_global_esr(g, gpc, tpc, sm);
	*hww_global_esr = global_esr;
	warp_esr = g->ops.gr.get_sm_hww_warp_esr(g, gpc, tpc, sm);
	global_mask = g->ops.gr.get_sm_no_lock_down_hww_global_esr_mask(g);

	if (!sm_debugger_attached) {
		nvgpu_err(g, "sm hww global 0x%08x warp 0x%08x",
			  global_esr, warp_esr);
		return -EFAULT;
	}

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
		  "sm hww global 0x%08x warp 0x%08x", global_esr, warp_esr);

	gr_gk20a_elpg_protected_call(g,
		g->ops.gr.record_sm_error_state(g, gpc, tpc, sm, fault_ch));

	if (g->ops.gr.pre_process_sm_exception) {
		ret = g->ops.gr.pre_process_sm_exception(g, gpc, tpc, sm,
				global_esr, warp_esr,
				sm_debugger_attached,
				fault_ch,
				&early_exit,
				&ignore_debugger);
		if (ret) {
			nvgpu_err(g, "could not pre-process sm error!");
			return ret;
		}
	}

	if (early_exit) {
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				"returning early");
		return ret;
	}

	/*
	 * Disable forwarding of tpc exceptions,
	 * the debugger will reenable exceptions after servicing them.
	 *
	 * Do not disable exceptions if the only SM exception is BPT_INT
	 */
	if ((global_esr == gr_gpc0_tpc0_sm_hww_global_esr_bpt_int_pending_f())
			&& (warp_esr == 0)) {
		disable_sm_exceptions = false;
	}

	if (!ignore_debugger && disable_sm_exceptions) {
		u32 tpc_exception_en = gk20a_readl(g,
				gr_gpc0_tpc0_tpccs_tpc_exception_en_r() +
				offset);
		tpc_exception_en &= ~gr_gpc0_tpc0_tpccs_tpc_exception_en_sm_enabled_f();
		gk20a_writel(g,
			     gr_gpc0_tpc0_tpccs_tpc_exception_en_r() + offset,
			     tpc_exception_en);
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg, "SM Exceptions disabled");
	}

	/* if a debugger is present and an error has occurred, do a warp sync */
	if (!ignore_debugger &&
	    ((warp_esr != 0) || ((global_esr & ~global_mask) != 0))) {
		nvgpu_log(g, gpu_dbg_intr, "warp sync needed");
		do_warp_sync = true;
	}

	if (do_warp_sync) {
		ret = g->ops.gr.lock_down_sm(g, gpc, tpc, sm,
				 global_mask, true);
		if (ret) {
			nvgpu_err(g, "sm did not lock down!");
			return ret;
		}
	}

	if (ignore_debugger) {
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"ignore_debugger set, skipping event posting");
	} else {
		*post_event = true;
	}

	return ret;
}

int gr_gk20a_handle_tex_exception(struct gk20a *g, u32 gpc, u32 tpc,
		bool *post_event)
{
	int ret = 0;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 esr;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");

	esr = gk20a_readl(g,
			 gr_gpc0_tpc0_tex_m_hww_esr_r() + offset);
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg, "0x%08x", esr);

	gk20a_writel(g,
		     gr_gpc0_tpc0_tex_m_hww_esr_r() + offset,
		     esr);

	return ret;
}

void gk20a_gr_get_esr_sm_sel(struct gk20a *g, u32 gpc, u32 tpc,
				u32 *esr_sm_sel)
{
	*esr_sm_sel = 1;
}

static int gk20a_gr_handle_tpc_exception(struct gk20a *g, u32 gpc, u32 tpc,
		bool *post_event, struct channel_gk20a *fault_ch,
		u32 *hww_global_esr)
{
	int ret = 0;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);
	u32 tpc_exception = gk20a_readl(g, gr_gpc0_tpc0_tpccs_tpc_exception_r()
			+ offset);
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"GPC%d TPC%d: pending exception 0x%x",
			gpc, tpc, tpc_exception);

	/* check if an sm exeption is pending */
	if (gr_gpc0_tpc0_tpccs_tpc_exception_sm_v(tpc_exception) ==
			gr_gpc0_tpc0_tpccs_tpc_exception_sm_pending_v()) {
		u32 esr_sm_sel, sm;

		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				"GPC%d TPC%d: SM exception pending", gpc, tpc);

		if (g->ops.gr.handle_tpc_sm_ecc_exception) {
			g->ops.gr.handle_tpc_sm_ecc_exception(g, gpc, tpc,
				post_event, fault_ch, hww_global_esr);
		}

		g->ops.gr.get_esr_sm_sel(g, gpc, tpc, &esr_sm_sel);

		for (sm = 0; sm < sm_per_tpc; sm++) {

			if ((esr_sm_sel & BIT32(sm)) == 0U) {
				continue;
			}

			nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				"GPC%d TPC%d: SM%d exception pending",
				 gpc, tpc, sm);

			ret |= g->ops.gr.handle_sm_exception(g,
				 gpc, tpc, sm, post_event, fault_ch,
				hww_global_esr);
			/* clear the hwws, also causes tpc and gpc
			 * exceptions to be cleared. Should be cleared
			 * only if SM is locked down or empty.
			 */
			g->ops.gr.clear_sm_hww(g,
				gpc, tpc, sm, *hww_global_esr);

		}

	}

	/* check if a tex exeption is pending */
	if (gr_gpc0_tpc0_tpccs_tpc_exception_tex_v(tpc_exception) ==
			gr_gpc0_tpc0_tpccs_tpc_exception_tex_pending_v()) {
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				"GPC%d TPC%d: TEX exception pending", gpc, tpc);
		ret |= g->ops.gr.handle_tex_exception(g, gpc, tpc, post_event);
	}

	if (g->ops.gr.handle_tpc_mpc_exception) {
		ret |= g->ops.gr.handle_tpc_mpc_exception(g,
					gpc, tpc, post_event);
	}

	return ret;
}

static int gk20a_gr_handle_gpc_exception(struct gk20a *g, bool *post_event,
		struct channel_gk20a *fault_ch, u32 *hww_global_esr)
{
	int ret = 0;
	u32 gpc_offset, gpc, tpc;
	struct gr_gk20a *gr = &g->gr;
	u32 exception1 = gk20a_readl(g, gr_exception1_r());
	u32 gpc_exception;

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg, " ");

	for (gpc = 0; gpc < gr->gpc_count; gpc++) {
		if ((exception1 & (1 << gpc)) == 0) {
			continue;
		}

		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				"GPC%d exception pending", gpc);

		gpc_offset = gk20a_gr_gpc_offset(g, gpc);

		gpc_exception = gk20a_readl(g, gr_gpc0_gpccs_gpc_exception_r()
				+ gpc_offset);

		/* check if any tpc has an exception */
		for (tpc = 0; tpc < gr->gpc_tpc_count[gpc]; tpc++) {
			if ((gr_gpc0_gpccs_gpc_exception_tpc_v(gpc_exception) &
				(1 << tpc)) == 0) {
				continue;
			}

			nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				  "GPC%d: TPC%d exception pending", gpc, tpc);

			ret |= gk20a_gr_handle_tpc_exception(g, gpc, tpc,
					post_event, fault_ch, hww_global_esr);

		}

		/* Handle GCC exception */
		if ((gr_gpc0_gpccs_gpc_exception_gcc_v(gpc_exception) != 0U) &&
				(g->ops.gr.handle_gcc_exception != NULL)) {
			int gcc_ret = 0;
			gcc_ret = g->ops.gr.handle_gcc_exception(g, gpc, tpc,
				post_event, fault_ch, hww_global_esr);
			ret |= (ret != 0) ? ret : gcc_ret;
		}

		/* Handle GPCCS exceptions */
		if (g->ops.gr.handle_gpc_gpccs_exception) {
			int ret_ecc = 0;
			ret_ecc = g->ops.gr.handle_gpc_gpccs_exception(g, gpc,
								gpc_exception);
			ret |= (ret != 0) ? ret : ret_ecc;
		}

		/* Handle GPCMMU exceptions */
		if (g->ops.gr.handle_gpc_gpcmmu_exception) {
			int ret_mmu = 0;

			ret_mmu = g->ops.gr.handle_gpc_gpcmmu_exception(g, gpc,
								gpc_exception);
			ret |= (ret != 0) ? ret : ret_mmu;
		}

	}

	return ret;
}

static int gk20a_gr_post_bpt_events(struct gk20a *g, struct tsg_gk20a *tsg,
				    u32 global_esr)
{
	if (global_esr & gr_gpc0_tpc0_sm_hww_global_esr_bpt_int_pending_f()) {
		g->ops.fifo.post_event_id(tsg, NVGPU_EVENT_ID_BPT_INT);
	}

	if (global_esr & gr_gpc0_tpc0_sm_hww_global_esr_bpt_pause_pending_f()) {
		g->ops.fifo.post_event_id(tsg, NVGPU_EVENT_ID_BPT_PAUSE);
	}

	return 0;
}

int gk20a_gr_isr(struct gk20a *g)
{
	struct gr_gk20a_isr_data isr_data;
	u32 grfifo_ctl;
	u32 obj_table;
	bool need_reset = false;
	u32 gr_intr = gk20a_readl(g, gr_intr_r());
	struct channel_gk20a *ch = NULL;
	struct channel_gk20a *fault_ch = NULL;
	u32 tsgid = NVGPU_INVALID_TSG_ID;
	struct tsg_gk20a *tsg = NULL;
	u32 gr_engine_id;
	u32 global_esr = 0;
	u32 chid;

	nvgpu_log_fn(g, " ");
	nvgpu_log(g, gpu_dbg_intr, "pgraph intr 0x%08x", gr_intr);

	if (gr_intr == 0U) {
		return 0;
	}

	gr_engine_id = gk20a_fifo_get_gr_engine_id(g);
	if (gr_engine_id != FIFO_INVAL_ENGINE_ID) {
		gr_engine_id = BIT(gr_engine_id);
	}

	grfifo_ctl = gk20a_readl(g, gr_gpfifo_ctl_r());
	grfifo_ctl &= ~gr_gpfifo_ctl_semaphore_access_f(1);
	grfifo_ctl &= ~gr_gpfifo_ctl_access_f(1);

	gk20a_writel(g, gr_gpfifo_ctl_r(),
		grfifo_ctl | gr_gpfifo_ctl_access_f(0) |
		gr_gpfifo_ctl_semaphore_access_f(0));

	isr_data.addr = gk20a_readl(g, gr_trapped_addr_r());
	isr_data.data_lo = gk20a_readl(g, gr_trapped_data_lo_r());
	isr_data.data_hi = gk20a_readl(g, gr_trapped_data_hi_r());
	isr_data.curr_ctx = gk20a_readl(g, gr_fecs_current_ctx_r());
	isr_data.offset = gr_trapped_addr_mthd_v(isr_data.addr);
	isr_data.sub_chan = gr_trapped_addr_subch_v(isr_data.addr);
	obj_table = (isr_data.sub_chan < 4) ? gk20a_readl(g,
		gr_fe_object_table_r(isr_data.sub_chan)) : 0;
	isr_data.class_num = gr_fe_object_table_nvclass_v(obj_table);

	ch = gk20a_gr_get_channel_from_ctx(g, isr_data.curr_ctx, &tsgid);
	isr_data.ch = ch;
	chid = ch != NULL ? ch->chid : FIFO_INVAL_CHANNEL_ID;

	if (ch == NULL) {
		nvgpu_err(g, "pgraph intr: 0x%08x, chid: INVALID", gr_intr);
	} else {
		tsg = tsg_gk20a_from_ch(ch);
		if (tsg == NULL) {
			nvgpu_err(g, "pgraph intr: 0x%08x, chid: %d "
				"not bound to tsg", gr_intr, chid);
		}
	}

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
		"channel %d: addr 0x%08x, "
		"data 0x%08x 0x%08x,"
		"ctx 0x%08x, offset 0x%08x, "
		"subchannel 0x%08x, class 0x%08x",
		chid, isr_data.addr,
		isr_data.data_hi, isr_data.data_lo,
		isr_data.curr_ctx, isr_data.offset,
		isr_data.sub_chan, isr_data.class_num);

	if (gr_intr & gr_intr_notify_pending_f()) {
		g->ops.gr.handle_notify_pending(g, &isr_data);
		gk20a_writel(g, gr_intr_r(),
			gr_intr_notify_reset_f());
		gr_intr &= ~gr_intr_notify_pending_f();
	}

	if (gr_intr & gr_intr_semaphore_pending_f()) {
		g->ops.gr.handle_semaphore_pending(g, &isr_data);
		gk20a_writel(g, gr_intr_r(),
			gr_intr_semaphore_reset_f());
		gr_intr &= ~gr_intr_semaphore_pending_f();
	}

	if (gr_intr & gr_intr_semaphore_timeout_pending_f()) {
		if (gk20a_gr_handle_semaphore_timeout_pending(g,
				&isr_data) != 0) {
			need_reset = true;
		}
		gk20a_writel(g, gr_intr_r(),
			gr_intr_semaphore_reset_f());
		gr_intr &= ~gr_intr_semaphore_pending_f();
	}

	if (gr_intr & gr_intr_illegal_notify_pending_f()) {
		if (gk20a_gr_intr_illegal_notify_pending(g,
				&isr_data) != 0) {
			need_reset = true;
		}
		gk20a_writel(g, gr_intr_r(),
			gr_intr_illegal_notify_reset_f());
		gr_intr &= ~gr_intr_illegal_notify_pending_f();
	}

	if (gr_intr & gr_intr_illegal_method_pending_f()) {
		if (gk20a_gr_handle_illegal_method(g, &isr_data) != 0) {
			need_reset = true;
		}
		gk20a_writel(g, gr_intr_r(),
			gr_intr_illegal_method_reset_f());
		gr_intr &= ~gr_intr_illegal_method_pending_f();
	}

	if (gr_intr & gr_intr_illegal_class_pending_f()) {
		if (gk20a_gr_handle_illegal_class(g, &isr_data) != 0) {
			need_reset = true;
		}
		gk20a_writel(g, gr_intr_r(),
			gr_intr_illegal_class_reset_f());
		gr_intr &= ~gr_intr_illegal_class_pending_f();
	}

	if (gr_intr & gr_intr_fecs_error_pending_f()) {
		if (g->ops.gr.handle_fecs_error(g, ch, &isr_data) != 0) {
			need_reset = true;
		}
		gk20a_writel(g, gr_intr_r(),
			gr_intr_fecs_error_reset_f());
		gr_intr &= ~gr_intr_fecs_error_pending_f();
	}

	if (gr_intr & gr_intr_class_error_pending_f()) {
		if (gk20a_gr_handle_class_error(g, &isr_data) != 0) {
			need_reset = true;
		}
		gk20a_writel(g, gr_intr_r(),
			gr_intr_class_error_reset_f());
		gr_intr &= ~gr_intr_class_error_pending_f();
	}

	/* this one happens if someone tries to hit a non-whitelisted
	 * register using set_falcon[4] */
	if (gr_intr & gr_intr_firmware_method_pending_f()) {
		if (gk20a_gr_handle_firmware_method(g, &isr_data) != 0) {
			need_reset = true;
		}
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg, "firmware method intr pending\n");
		gk20a_writel(g, gr_intr_r(),
			gr_intr_firmware_method_reset_f());
		gr_intr &= ~gr_intr_firmware_method_pending_f();
	}

	if (gr_intr & gr_intr_exception_pending_f()) {
		u32 exception = gk20a_readl(g, gr_exception_r());

		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg, "exception %08x\n", exception);

		if (exception & gr_exception_fe_m()) {
			u32 fe = gk20a_readl(g, gr_fe_hww_esr_r());
			u32 info = gk20a_readl(g, gr_fe_hww_esr_info_r());

			nvgpu_err(g, "fe exception: esr 0x%08x, info 0x%08x",
					fe, info);
			gk20a_writel(g, gr_fe_hww_esr_r(),
				gr_fe_hww_esr_reset_active_f());
			need_reset = true;
		}

		if (exception & gr_exception_memfmt_m()) {
			u32 memfmt = gk20a_readl(g, gr_memfmt_hww_esr_r());

			nvgpu_err(g, "memfmt exception: esr %08x", memfmt);
			gk20a_writel(g, gr_memfmt_hww_esr_r(),
					gr_memfmt_hww_esr_reset_active_f());
			need_reset = true;
		}

		if (exception & gr_exception_pd_m()) {
			u32 pd = gk20a_readl(g, gr_pd_hww_esr_r());

			nvgpu_err(g, "pd exception: esr 0x%08x", pd);
			gk20a_writel(g, gr_pd_hww_esr_r(),
					gr_pd_hww_esr_reset_active_f());
			need_reset = true;
		}

		if (exception & gr_exception_scc_m()) {
			u32 scc = gk20a_readl(g, gr_scc_hww_esr_r());

			nvgpu_err(g, "scc exception: esr 0x%08x", scc);
			gk20a_writel(g, gr_scc_hww_esr_r(),
					gr_scc_hww_esr_reset_active_f());
			need_reset = true;
		}

		if (exception & gr_exception_ds_m()) {
			u32 ds = gk20a_readl(g, gr_ds_hww_esr_r());

			nvgpu_err(g, "ds exception: esr: 0x%08x", ds);
			gk20a_writel(g, gr_ds_hww_esr_r(),
					 gr_ds_hww_esr_reset_task_f());
			need_reset = true;
		}

		if (exception & gr_exception_ssync_m()) {
			if (g->ops.gr.handle_ssync_hww) {
				if (g->ops.gr.handle_ssync_hww(g) != 0) {
					need_reset = true;
				}
			} else {
				nvgpu_err(g, "unhandled ssync exception");
			}
		}

		if (exception & gr_exception_mme_m()) {
			u32 mme = gk20a_readl(g, gr_mme_hww_esr_r());
			u32 info = gk20a_readl(g, gr_mme_hww_esr_info_r());

			nvgpu_err(g, "mme exception: esr 0x%08x info:0x%08x",
					mme, info);
			gk20a_writel(g, gr_mme_hww_esr_r(),
				gr_mme_hww_esr_reset_active_f());
			need_reset = true;
		}

		if (exception & gr_exception_sked_m()) {
			u32 sked = gk20a_readl(g, gr_sked_hww_esr_r());

			nvgpu_err(g, "sked exception: esr 0x%08x", sked);
			gk20a_writel(g, gr_sked_hww_esr_r(),
				gr_sked_hww_esr_reset_active_f());
			need_reset = true;
		}

		/* check if a gpc exception has occurred */
		if (((exception & gr_exception_gpc_m()) != 0U) &&
						!need_reset) {
			bool post_event = false;

			nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
					 "GPC exception pending");

			if (tsg != NULL) {
				fault_ch = isr_data.ch;
			}

			/* fault_ch can be NULL */
			/* check if any gpc has an exception */
			if (gk20a_gr_handle_gpc_exception(g, &post_event,
					fault_ch, &global_esr) != 0) {
				need_reset = true;
			}

			/* signal clients waiting on an event */
			if (g->ops.gr.sm_debugger_attached(g) &&
				post_event && (fault_ch != NULL)) {
				g->ops.debugger.post_events(fault_ch);
			}
		}

		gk20a_writel(g, gr_intr_r(), gr_intr_exception_reset_f());
		gr_intr &= ~gr_intr_exception_pending_f();

		if (need_reset) {
			nvgpu_err(g, "set gr exception notifier");
			gk20a_gr_set_error_notifier(g, &isr_data,
					 NVGPU_ERR_NOTIFIER_GR_EXCEPTION);
		}
	}

	if (need_reset) {
		if (tsg != NULL) {
			gk20a_fifo_recover(g, gr_engine_id,
					   tsgid, true, true, true,
						RC_TYPE_GR_FAULT);
		} else {
			if (ch != NULL) {
				nvgpu_err(g, "chid: %d referenceable but not "
					"bound to tsg", chid);
			}
			gk20a_fifo_recover(g, gr_engine_id,
					   0, false, false, true,
						RC_TYPE_GR_FAULT);
		}
	}

	if (gr_intr != 0U) {
		/* clear unhandled interrupts */
		if (ch == NULL) {
			/*
			 * This is probably an interrupt during
			 * gk20a_free_channel()
			 */
			nvgpu_err(g, "unhandled gr intr 0x%08x for "
				"unreferenceable channel, clearing",
				gr_intr);
		} else {
			nvgpu_err(g, "unhandled gr intr 0x%08x for chid: %d",
				gr_intr, chid);
		}
		gk20a_writel(g, gr_intr_r(), gr_intr);
	}

	gk20a_writel(g, gr_gpfifo_ctl_r(),
		grfifo_ctl | gr_gpfifo_ctl_access_f(1) |
		gr_gpfifo_ctl_semaphore_access_f(1));


	/* Posting of BPT events should be the last thing in this function */
	if ((global_esr != 0U) && (tsg != NULL)) {
		gk20a_gr_post_bpt_events(g, tsg, global_esr);
	}

	if (ch) {
		gk20a_channel_put(ch);
	}

	return 0;
}

u32 gk20a_gr_nonstall_isr(struct gk20a *g)
{
	u32 ops = 0;
	u32 gr_intr = gk20a_readl(g, gr_intr_nonstall_r());

	nvgpu_log(g, gpu_dbg_intr, "pgraph nonstall intr %08x", gr_intr);

	if ((gr_intr & gr_intr_nonstall_trap_pending_f()) != 0U) {
		/* Clear the interrupt */
		gk20a_writel(g, gr_intr_nonstall_r(),
			gr_intr_nonstall_trap_pending_f());
		ops |= (GK20A_NONSTALL_OPS_WAKEUP_SEMAPHORE |
			GK20A_NONSTALL_OPS_POST_EVENTS);
	}
	return ops;
}

int gr_gk20a_fecs_get_reglist_img_size(struct gk20a *g, u32 *size)
{
	BUG_ON(size == NULL);
	return gr_gk20a_submit_fecs_method_op(g,
		   (struct fecs_method_op_gk20a) {
			   .mailbox.id = 0,
			   .mailbox.data = 0,
			   .mailbox.clr = ~0,
			   .method.data = 1,
			   .method.addr = gr_fecs_method_push_adr_discover_reglist_image_size_v(),
			   .mailbox.ret = size,
			   .cond.ok = GR_IS_UCODE_OP_NOT_EQUAL,
			   .mailbox.ok = 0,
			   .cond.fail = GR_IS_UCODE_OP_SKIP,
			   .mailbox.fail = 0}, false);
}

int gr_gk20a_fecs_set_reglist_bind_inst(struct gk20a *g,
		struct nvgpu_mem *inst_block)
{
	u32 data = fecs_current_ctx_data(g, inst_block);

	return gr_gk20a_submit_fecs_method_op(g,
		   (struct fecs_method_op_gk20a){
			   .mailbox.id = 4,
			   .mailbox.data = data,
			   .mailbox.clr = ~0,
			   .method.data = 1,
			   .method.addr = gr_fecs_method_push_adr_set_reglist_bind_instance_v(),
			   .mailbox.ret = NULL,
			   .cond.ok = GR_IS_UCODE_OP_EQUAL,
			   .mailbox.ok = 1,
			   .cond.fail = GR_IS_UCODE_OP_SKIP,
			   .mailbox.fail = 0}, false);
}

int gr_gk20a_fecs_set_reglist_virtual_addr(struct gk20a *g, u64 pmu_va)
{
	return gr_gk20a_submit_fecs_method_op(g,
		   (struct fecs_method_op_gk20a) {
			   .mailbox.id = 4,
			   .mailbox.data = u64_lo32(pmu_va >> 8),
			   .mailbox.clr = ~0,
			   .method.data = 1,
			   .method.addr = gr_fecs_method_push_adr_set_reglist_virtual_address_v(),
			   .mailbox.ret = NULL,
			   .cond.ok = GR_IS_UCODE_OP_EQUAL,
			   .mailbox.ok = 1,
			   .cond.fail = GR_IS_UCODE_OP_SKIP,
			   .mailbox.fail = 0}, false);
}

int gk20a_gr_suspend(struct gk20a *g)
{
	u32 ret = 0;

	nvgpu_log_fn(g, " ");

	ret = g->ops.gr.wait_empty(g, gk20a_get_gr_idle_timeout(g),
				   GR_IDLE_CHECK_DEFAULT);
	if (ret) {
		return ret;
	}

	gk20a_writel(g, gr_gpfifo_ctl_r(),
		gr_gpfifo_ctl_access_disabled_f());

	/* disable gr intr */
	gk20a_writel(g, gr_intr_r(), 0);
	gk20a_writel(g, gr_intr_en_r(), 0);

	/* disable all exceptions */
	gk20a_writel(g, gr_exception_r(), 0);
	gk20a_writel(g, gr_exception_en_r(), 0);
	gk20a_writel(g, gr_exception1_r(), 0);
	gk20a_writel(g, gr_exception1_en_r(), 0);
	gk20a_writel(g, gr_exception2_r(), 0);
	gk20a_writel(g, gr_exception2_en_r(), 0);

	gk20a_gr_flush_channel_tlb(&g->gr);

	g->gr.initialized = false;

	nvgpu_log_fn(g, "done");
	return ret;
}

static int gr_gk20a_find_priv_offset_in_buffer(struct gk20a *g,
					       u32 addr,
					       bool is_quad, u32 quad,
					       u32 *context_buffer,
					       u32 context_buffer_size,
					       u32 *priv_offset);

static int gr_gk20a_find_priv_offset_in_pm_buffer(struct gk20a *g,
					          u32 addr,
					          u32 *priv_offset);

/* This function will decode a priv address and return the partition type and numbers. */
int gr_gk20a_decode_priv_addr(struct gk20a *g, u32 addr,
			      enum ctxsw_addr_type *addr_type,
			      u32 *gpc_num, u32 *tpc_num, u32 *ppc_num, u32 *be_num,
			      u32 *broadcast_flags)
{
	u32 gpc_addr;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	/* setup defaults */
	*addr_type = CTXSW_ADDR_TYPE_SYS;
	*broadcast_flags = PRI_BROADCAST_FLAGS_NONE;
	*gpc_num = 0;
	*tpc_num = 0;
	*ppc_num = 0;
	*be_num  = 0;

	if (pri_is_gpc_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_GPC;
		gpc_addr = pri_gpccs_addr_mask(addr);
		if (pri_is_gpc_addr_shared(g, addr)) {
			*addr_type = CTXSW_ADDR_TYPE_GPC;
			*broadcast_flags |= PRI_BROADCAST_FLAGS_GPC;
		} else {
			*gpc_num = pri_get_gpc_num(g, addr);
		}

		if (pri_is_ppc_addr(g, gpc_addr)) {
			*addr_type = CTXSW_ADDR_TYPE_PPC;
			if (pri_is_ppc_addr_shared(g, gpc_addr)) {
				*broadcast_flags |= PRI_BROADCAST_FLAGS_PPC;
				return 0;
			}
		}
		if (g->ops.gr.is_tpc_addr(g, gpc_addr)) {
			*addr_type = CTXSW_ADDR_TYPE_TPC;
			if (pri_is_tpc_addr_shared(g, gpc_addr)) {
				*broadcast_flags |= PRI_BROADCAST_FLAGS_TPC;
				return 0;
			}
			*tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
		}
		return 0;
	} else if (pri_is_be_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_BE;
		if (pri_is_be_addr_shared(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_BE;
			return 0;
		}
		*be_num = pri_get_be_num(g, addr);
		return 0;
	} else if (g->ops.ltc.pri_is_ltc_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_LTCS;
		if (g->ops.ltc.is_ltcs_ltss_addr(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_LTCS;
		} else if (g->ops.ltc.is_ltcn_ltss_addr(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_LTSS;
		}
		return 0;
	} else if (pri_is_fbpa_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_FBPA;
		if (pri_is_fbpa_addr_shared(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_FBPA;
			return 0;
		}
		return 0;
	} else if ((g->ops.gr.is_egpc_addr != NULL) &&
		   g->ops.gr.is_egpc_addr(g, addr)) {
			return g->ops.gr.decode_egpc_addr(g,
					addr, addr_type, gpc_num,
					tpc_num, broadcast_flags);
	} else {
		*addr_type = CTXSW_ADDR_TYPE_SYS;
		return 0;
	}
	/* PPC!?!?!?! */

	/*NOTREACHED*/
	return -EINVAL;
}

void gr_gk20a_split_fbpa_broadcast_addr(struct gk20a *g, u32 addr,
				      u32 num_fbpas,
				      u32 *priv_addr_table, u32 *t)
{
	u32 fbpa_id;

	for (fbpa_id = 0; fbpa_id < num_fbpas; fbpa_id++) {
		priv_addr_table[(*t)++] = pri_fbpa_addr(g,
				pri_fbpa_addr_mask(g, addr), fbpa_id);
	}
}

int gr_gk20a_split_ppc_broadcast_addr(struct gk20a *g, u32 addr,
				      u32 gpc_num,
				      u32 *priv_addr_table, u32 *t)
{
    u32 ppc_num;

    nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

    for (ppc_num = 0; ppc_num < g->gr.gpc_ppc_count[gpc_num]; ppc_num++) {
	    priv_addr_table[(*t)++] = pri_ppc_addr(g, pri_ppccs_addr_mask(addr),
						   gpc_num, ppc_num);
    }

    return 0;
}

/*
 * The context buffer is indexed using BE broadcast addresses and GPC/TPC
 * unicast addresses. This function will convert a BE unicast address to a BE
 * broadcast address and split a GPC/TPC broadcast address into a table of
 * GPC/TPC addresses.  The addresses generated by this function can be
 * successfully processed by gr_gk20a_find_priv_offset_in_buffer
 */
int gr_gk20a_create_priv_addr_table(struct gk20a *g,
					   u32 addr,
					   u32 *priv_addr_table,
					   u32 *num_registers)
{
	enum ctxsw_addr_type addr_type;
	u32 gpc_num, tpc_num, ppc_num, be_num;
	u32 priv_addr, gpc_addr;
	u32 broadcast_flags;
	u32 t;
	int err;

	t = 0;
	*num_registers = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	err = g->ops.gr.decode_priv_addr(g, addr, &addr_type,
					&gpc_num, &tpc_num, &ppc_num, &be_num,
					&broadcast_flags);
	nvgpu_log(g, gpu_dbg_gpu_dbg, "addr_type = %d", addr_type);
	if (err != 0) {
		return err;
	}

	if ((addr_type == CTXSW_ADDR_TYPE_SYS) ||
	    (addr_type == CTXSW_ADDR_TYPE_BE)) {
		/* The BE broadcast registers are included in the compressed PRI
		 * table. Convert a BE unicast address to a broadcast address
		 * so that we can look up the offset. */
		if ((addr_type == CTXSW_ADDR_TYPE_BE) &&
		    ((broadcast_flags & PRI_BROADCAST_FLAGS_BE) == 0U)) {
			priv_addr_table[t++] = pri_be_shared_addr(g, addr);
		} else {
			priv_addr_table[t++] = addr;
		}

		*num_registers = t;
		return 0;
	}

	/* The GPC/TPC unicast registers are included in the compressed PRI
	 * tables. Convert a GPC/TPC broadcast address to unicast addresses so
	 * that we can look up the offsets. */
	if (broadcast_flags & PRI_BROADCAST_FLAGS_GPC) {
		for (gpc_num = 0; gpc_num < g->gr.gpc_count; gpc_num++) {

			if (broadcast_flags & PRI_BROADCAST_FLAGS_TPC) {
				for (tpc_num = 0;
				     tpc_num < g->gr.gpc_tpc_count[gpc_num];
				     tpc_num++) {
					priv_addr_table[t++] =
						pri_tpc_addr(g, pri_tpccs_addr_mask(addr),
							     gpc_num, tpc_num);
				}

			} else if (broadcast_flags & PRI_BROADCAST_FLAGS_PPC) {
				err = gr_gk20a_split_ppc_broadcast_addr(g, addr, gpc_num,
							       priv_addr_table, &t);
				if (err != 0) {
					return err;
				}
			} else {
				priv_addr = pri_gpc_addr(g,
						pri_gpccs_addr_mask(addr),
						gpc_num);

				gpc_addr = pri_gpccs_addr_mask(priv_addr);
				tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
				if (tpc_num >= g->gr.gpc_tpc_count[gpc_num]) {
					continue;
				}

				priv_addr_table[t++] = priv_addr;
			}
		}
	} else if (((addr_type == CTXSW_ADDR_TYPE_EGPC) ||
			(addr_type == CTXSW_ADDR_TYPE_ETPC)) &&
			(g->ops.gr.egpc_etpc_priv_addr_table != NULL)) {
		nvgpu_log(g, gpu_dbg_gpu_dbg, "addr_type : EGPC/ETPC");
		g->ops.gr.egpc_etpc_priv_addr_table(g, addr, gpc_num, tpc_num,
				broadcast_flags, priv_addr_table, &t);
	} else if (broadcast_flags & PRI_BROADCAST_FLAGS_LTSS) {
		g->ops.ltc.split_lts_broadcast_addr(g, addr,
							priv_addr_table, &t);
	} else if (broadcast_flags & PRI_BROADCAST_FLAGS_LTCS) {
		g->ops.ltc.split_ltc_broadcast_addr(g, addr,
							priv_addr_table, &t);
	} else if (broadcast_flags & PRI_BROADCAST_FLAGS_FBPA) {
		g->ops.gr.split_fbpa_broadcast_addr(g, addr,
				nvgpu_get_litter_value(g, GPU_LIT_NUM_FBPAS),
				priv_addr_table, &t);
	} else if ((broadcast_flags & PRI_BROADCAST_FLAGS_GPC) == 0U) {
		if (broadcast_flags & PRI_BROADCAST_FLAGS_TPC) {
			for (tpc_num = 0;
			     tpc_num < g->gr.gpc_tpc_count[gpc_num];
			     tpc_num++) {
				priv_addr_table[t++] =
					pri_tpc_addr(g, pri_tpccs_addr_mask(addr),
						     gpc_num, tpc_num);
			}
		} else if (broadcast_flags & PRI_BROADCAST_FLAGS_PPC) {
			err = gr_gk20a_split_ppc_broadcast_addr(g,
					addr, gpc_num, priv_addr_table, &t);
		} else {
			priv_addr_table[t++] = addr;
		}
	}

	*num_registers = t;
	return 0;
}

int gr_gk20a_get_ctx_buffer_offsets(struct gk20a *g,
				    u32 addr,
				    u32 max_offsets,
				    u32 *offsets, u32 *offset_addrs,
				    u32 *num_offsets,
				    bool is_quad, u32 quad)
{
	u32 i;
	u32 priv_offset = 0;
	u32 *priv_registers;
	u32 num_registers = 0;
	int err = 0;
	struct gr_gk20a *gr = &g->gr;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	u32 potential_offsets = gr->max_gpc_count * gr->max_tpc_per_gpc_count *
					sm_per_tpc;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	/* implementation is crossed-up if either of these happen */
	if (max_offsets > potential_offsets) {
		nvgpu_log_fn(g, "max_offsets > potential_offsets");
		return -EINVAL;
	}

	if (!g->gr.ctx_vars.golden_image_initialized) {
		return -ENODEV;
	}

	priv_registers = nvgpu_kzalloc(g, sizeof(u32) * potential_offsets);
	if (priv_registers == NULL) {
		nvgpu_log_fn(g, "failed alloc for potential_offsets=%d", potential_offsets);
		err = PTR_ERR(priv_registers);
		goto cleanup;
	}
	memset(offsets,      0, sizeof(u32) * max_offsets);
	memset(offset_addrs, 0, sizeof(u32) * max_offsets);
	*num_offsets = 0;

	g->ops.gr.create_priv_addr_table(g, addr, &priv_registers[0],
			&num_registers);

	if ((max_offsets > 1) && (num_registers > max_offsets)) {
		nvgpu_log_fn(g, "max_offsets = %d, num_registers = %d",
				max_offsets, num_registers);
		err = -EINVAL;
		goto cleanup;
	}

	if ((max_offsets == 1) && (num_registers > 1)) {
		num_registers = 1;
	}

	if (g->gr.ctx_vars.local_golden_image == NULL) {
		nvgpu_log_fn(g, "no context switch header info to work with");
		err = -EINVAL;
		goto cleanup;
	}

	for (i = 0; i < num_registers; i++) {
		err = gr_gk20a_find_priv_offset_in_buffer(g,
						  priv_registers[i],
						  is_quad, quad,
						  g->gr.ctx_vars.local_golden_image,
						  g->gr.ctx_vars.golden_image_size,
						  &priv_offset);
		if (err != 0) {
			nvgpu_log_fn(g, "Could not determine priv_offset for addr:0x%x",
				      addr); /*, grPriRegStr(addr)));*/
			goto cleanup;
		}

		offsets[i] = priv_offset;
		offset_addrs[i] = priv_registers[i];
	}

	*num_offsets = num_registers;
cleanup:
	if (!IS_ERR_OR_NULL(priv_registers)) {
		nvgpu_kfree(g, priv_registers);
	}

	return err;
}

int gr_gk20a_get_pm_ctx_buffer_offsets(struct gk20a *g,
				       u32 addr,
				       u32 max_offsets,
				       u32 *offsets, u32 *offset_addrs,
				       u32 *num_offsets)
{
	u32 i;
	u32 priv_offset = 0;
	u32 *priv_registers;
	u32 num_registers = 0;
	int err = 0;
	struct gr_gk20a *gr = &g->gr;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	u32 potential_offsets = gr->max_gpc_count * gr->max_tpc_per_gpc_count *
					sm_per_tpc;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	/* implementation is crossed-up if either of these happen */
	if (max_offsets > potential_offsets) {
		return -EINVAL;
	}

	if (!g->gr.ctx_vars.golden_image_initialized) {
		return -ENODEV;
	}

	priv_registers = nvgpu_kzalloc(g, sizeof(u32) * potential_offsets);
	if (priv_registers == NULL) {
		nvgpu_log_fn(g, "failed alloc for potential_offsets=%d", potential_offsets);
		return -ENOMEM;
	}
	memset(offsets,      0, sizeof(u32) * max_offsets);
	memset(offset_addrs, 0, sizeof(u32) * max_offsets);
	*num_offsets = 0;

	g->ops.gr.create_priv_addr_table(g, addr, priv_registers,
			&num_registers);

	if ((max_offsets > 1) && (num_registers > max_offsets)) {
		err = -EINVAL;
		goto cleanup;
	}

	if ((max_offsets == 1) && (num_registers > 1)) {
		num_registers = 1;
	}

	if (g->gr.ctx_vars.local_golden_image == NULL) {
		nvgpu_log_fn(g, "no context switch header info to work with");
		err = -EINVAL;
		goto cleanup;
	}

	for (i = 0; i < num_registers; i++) {
		err = gr_gk20a_find_priv_offset_in_pm_buffer(g,
						  priv_registers[i],
						  &priv_offset);
		if (err != 0) {
			nvgpu_log_fn(g, "Could not determine priv_offset for addr:0x%x",
				      addr); /*, grPriRegStr(addr)));*/
			goto cleanup;
		}

		offsets[i] = priv_offset;
		offset_addrs[i] = priv_registers[i];
	}

	*num_offsets = num_registers;
cleanup:
	nvgpu_kfree(g, priv_registers);

	return err;
}

/* Setup some register tables.  This looks hacky; our
 * register/offset functions are just that, functions.
 * So they can't be used as initializers... TBD: fix to
 * generate consts at least on an as-needed basis.
 */
static const u32 _num_ovr_perf_regs = 17;
static u32 _ovr_perf_regs[17] = { 0, };
/* Following are the blocks of registers that the ucode
 stores in the extended region.*/

void gk20a_gr_init_ovr_sm_dsm_perf(void)
{
	if (_ovr_perf_regs[0] != 0) {
		return;
	}

	_ovr_perf_regs[0] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control_sel0_r();
	_ovr_perf_regs[1] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control_sel1_r();
	_ovr_perf_regs[2] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control0_r();
	_ovr_perf_regs[3] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control5_r();
	_ovr_perf_regs[4] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter_status1_r();
	_ovr_perf_regs[5] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter0_control_r();
	_ovr_perf_regs[6] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter1_control_r();
	_ovr_perf_regs[7] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter2_control_r();
	_ovr_perf_regs[8] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter3_control_r();
	_ovr_perf_regs[9] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter4_control_r();
	_ovr_perf_regs[10] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter5_control_r();
	_ovr_perf_regs[11] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter6_control_r();
	_ovr_perf_regs[12] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter7_control_r();
	_ovr_perf_regs[13] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter4_r();
	_ovr_perf_regs[14] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter5_r();
	_ovr_perf_regs[15] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter6_r();
	_ovr_perf_regs[16] = gr_pri_gpc0_tpc0_sm_dsm_perf_counter7_r();

}

/* TBD: would like to handle this elsewhere, at a higher level.
 * these are currently constructed in a "test-then-write" style
 * which makes it impossible to know externally whether a ctx
 * write will actually occur. so later we should put a lazy,
 *  map-and-hold system in the patch write state */
static int gr_gk20a_ctx_patch_smpc(struct gk20a *g,
			    struct channel_gk20a *ch,
			    u32 addr, u32 data,
			    struct nvgpu_mem *mem)
{
	u32 num_gpc = g->gr.gpc_count;
	u32 num_tpc;
	u32 tpc, gpc, reg;
	u32 chk_addr;
	u32 vaddr_lo;
	u32 vaddr_hi;
	u32 tmp;
	u32 num_ovr_perf_regs = 0;
	u32 *ovr_perf_regs = NULL;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;
	struct nvgpu_mem *ctxheader = &ch->ctx_header;

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	g->ops.gr.init_ovr_sm_dsm_perf();
	g->ops.gr.init_sm_dsm_reg_info();
	g->ops.gr.get_ovr_perf_regs(g, &num_ovr_perf_regs, &ovr_perf_regs);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	for (reg = 0; reg < num_ovr_perf_regs; reg++) {
		for (gpc = 0; gpc < num_gpc; gpc++)  {
			num_tpc = g->gr.gpc_tpc_count[gpc];
			for (tpc = 0; tpc < num_tpc; tpc++) {
				chk_addr = ((gpc_stride * gpc) +
					    (tpc_in_gpc_stride * tpc) +
					    ovr_perf_regs[reg]);
				if (chk_addr != addr) {
					continue;
				}
				/* reset the patch count from previous
				   runs,if ucode has already processed
				   it */
				tmp = nvgpu_mem_rd(g, mem,
				       ctxsw_prog_main_image_patch_count_o());

				if (tmp == 0U) {
					gr_ctx->patch_ctx.data_count = 0;
				}

				gr_gk20a_ctx_patch_write(g, gr_ctx,
							 addr, data, true);

				vaddr_lo = u64_lo32(gr_ctx->patch_ctx.mem.gpu_va);
				vaddr_hi = u64_hi32(gr_ctx->patch_ctx.mem.gpu_va);

				nvgpu_mem_wr(g, mem,
					 ctxsw_prog_main_image_patch_count_o(),
					 gr_ctx->patch_ctx.data_count);
				if (ctxheader->gpu_va) {
					nvgpu_mem_wr(g, ctxheader,
						ctxsw_prog_main_image_patch_adr_lo_o(),
						vaddr_lo);
					nvgpu_mem_wr(g, ctxheader,
						ctxsw_prog_main_image_patch_adr_hi_o(),
						vaddr_hi);
				} else {
					nvgpu_mem_wr(g, mem,
						ctxsw_prog_main_image_patch_adr_lo_o(),
						vaddr_lo);
					nvgpu_mem_wr(g, mem,
						ctxsw_prog_main_image_patch_adr_hi_o(),
						vaddr_hi);
				}

				/* we're not caching these on cpu side,
				   but later watch for it */
				return 0;
			}
		}
	}

	return 0;
}

#define ILLEGAL_ID ((u32)~0)

static inline bool check_main_image_header_magic(u8 *context)
{
	u32 magic = *(u32 *)(context + ctxsw_prog_main_image_magic_value_o());
	return magic == ctxsw_prog_main_image_magic_value_v_value_v();
}
static inline bool check_local_header_magic(u8 *context)
{
	u32 magic = *(u32 *)(context + ctxsw_prog_local_magic_value_o());
	return magic == ctxsw_prog_local_magic_value_v_value_v();

}

/* most likely dupe of ctxsw_gpccs_header__size_1_v() */
static inline int ctxsw_prog_ucode_header_size_in_bytes(void)
{
	return 256;
}

void gk20a_gr_get_ovr_perf_regs(struct gk20a *g, u32 *num_ovr_perf_regs,
					       u32 **ovr_perf_regs)
{
	*num_ovr_perf_regs = _num_ovr_perf_regs;
	*ovr_perf_regs = _ovr_perf_regs;
}

static int gr_gk20a_find_priv_offset_in_ext_buffer(struct gk20a *g,
						   u32 addr,
						   bool is_quad, u32 quad,
						   u32 *context_buffer,
						   u32 context_buffer_size,
						   u32 *priv_offset)
{
	u32 i, data32;
	u32 gpc_num, tpc_num;
	u32 num_gpcs, num_tpcs;
	u32 chk_addr;
	u32 ext_priv_offset, ext_priv_size;
	u8 *context;
	u32 offset_to_segment, offset_to_segment_end;
	u32 sm_dsm_perf_reg_id = ILLEGAL_ID;
	u32 sm_dsm_perf_ctrl_reg_id = ILLEGAL_ID;
	u32 num_ext_gpccs_ext_buffer_segments;
	u32 inter_seg_offset;
	u32 max_tpc_count;
	u32 *sm_dsm_perf_ctrl_regs = NULL;
	u32 num_sm_dsm_perf_ctrl_regs = 0;
	u32 *sm_dsm_perf_regs = NULL;
	u32 num_sm_dsm_perf_regs = 0;
	u32 buffer_segments_size = 0;
	u32 marker_size = 0;
	u32 control_register_stride = 0;
	u32 perf_register_stride = 0;
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_base = nvgpu_get_litter_value(g, GPU_LIT_GPC_BASE);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 tpc_gpc_mask = (tpc_in_gpc_stride - 1);

	/* Only have TPC registers in extended region, so if not a TPC reg,
	   then return error so caller can look elsewhere. */
	if (pri_is_gpc_addr(g, addr))   {
		u32 gpc_addr = 0;
		gpc_num = pri_get_gpc_num(g, addr);
		gpc_addr = pri_gpccs_addr_mask(addr);
		if (g->ops.gr.is_tpc_addr(g, gpc_addr)) {
			tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
		} else {
			return -EINVAL;
		}

		nvgpu_log_info(g, " gpc = %d tpc = %d",
				gpc_num, tpc_num);
	} else if ((g->ops.gr.is_etpc_addr != NULL) &&
				g->ops.gr.is_etpc_addr(g, addr)) {
			g->ops.gr.get_egpc_etpc_num(g, addr, &gpc_num, &tpc_num);
			gpc_base = g->ops.gr.get_egpc_base(g);
	} else {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"does not exist in extended region");
		return -EINVAL;
	}

	buffer_segments_size = ctxsw_prog_extended_buffer_segments_size_in_bytes_v();
	/* note below is in words/num_registers */
	marker_size = ctxsw_prog_extended_marker_size_in_bytes_v() >> 2;

	context = (u8 *)context_buffer;
	/* sanity check main header */
	if (!check_main_image_header_magic(context)) {
		nvgpu_err(g,
			   "Invalid main header: magic value");
		return -EINVAL;
	}
	num_gpcs = *(u32 *)(context + ctxsw_prog_main_image_num_gpcs_o());
	if (gpc_num >= num_gpcs) {
		nvgpu_err(g,
		   "GPC 0x%08x is greater than total count 0x%08x!",
			   gpc_num, num_gpcs);
		return -EINVAL;
	}

	data32 = *(u32 *)(context + ctxsw_prog_main_extended_buffer_ctl_o());
	ext_priv_size   = ctxsw_prog_main_extended_buffer_ctl_size_v(data32);
	if (0 == ext_priv_size) {
		nvgpu_log_info(g, " No extended memory in context buffer");
		return -EINVAL;
	}
	ext_priv_offset = ctxsw_prog_main_extended_buffer_ctl_offset_v(data32);

	offset_to_segment = ext_priv_offset * ctxsw_prog_ucode_header_size_in_bytes();
	offset_to_segment_end = offset_to_segment +
		(ext_priv_size * buffer_segments_size);

	/* check local header magic */
	context += ctxsw_prog_ucode_header_size_in_bytes();
	if (!check_local_header_magic(context)) {
		nvgpu_err(g,
			   "Invalid local header: magic value");
		return -EINVAL;
	}

	/*
	 * See if the incoming register address is in the first table of
	 * registers. We check this by decoding only the TPC addr portion.
	 * If we get a hit on the TPC bit, we then double check the address
	 * by computing it from the base gpc/tpc strides.  Then make sure
	 * it is a real match.
	 */
	g->ops.gr.get_sm_dsm_perf_regs(g, &num_sm_dsm_perf_regs,
				       &sm_dsm_perf_regs,
				       &perf_register_stride);

	g->ops.gr.init_sm_dsm_reg_info();

	for (i = 0; i < num_sm_dsm_perf_regs; i++) {
		if ((addr & tpc_gpc_mask) == (sm_dsm_perf_regs[i] & tpc_gpc_mask)) {
			sm_dsm_perf_reg_id = i;

			nvgpu_log_info(g, "register match: 0x%08x",
					sm_dsm_perf_regs[i]);

			chk_addr = (gpc_base + gpc_stride * gpc_num) +
				   tpc_in_gpc_base +
				   (tpc_in_gpc_stride * tpc_num) +
				   (sm_dsm_perf_regs[sm_dsm_perf_reg_id] & tpc_gpc_mask);

			if (chk_addr != addr) {
				nvgpu_err(g,
				   "Oops addr miss-match! : 0x%08x != 0x%08x",
					   addr, chk_addr);
				return -EINVAL;
			}
			break;
		}
	}

	/* Didn't find reg in supported group 1.
	 *  so try the second group now */
	g->ops.gr.get_sm_dsm_perf_ctrl_regs(g, &num_sm_dsm_perf_ctrl_regs,
				       &sm_dsm_perf_ctrl_regs,
				       &control_register_stride);

	if (ILLEGAL_ID == sm_dsm_perf_reg_id) {
		for (i = 0; i < num_sm_dsm_perf_ctrl_regs; i++) {
			if ((addr & tpc_gpc_mask) ==
			    (sm_dsm_perf_ctrl_regs[i] & tpc_gpc_mask)) {
				sm_dsm_perf_ctrl_reg_id = i;

				nvgpu_log_info(g, "register match: 0x%08x",
						sm_dsm_perf_ctrl_regs[i]);

				chk_addr = (gpc_base + gpc_stride * gpc_num) +
					   tpc_in_gpc_base +
					   tpc_in_gpc_stride * tpc_num +
					   (sm_dsm_perf_ctrl_regs[sm_dsm_perf_ctrl_reg_id] &
					    tpc_gpc_mask);

				if (chk_addr != addr) {
					nvgpu_err(g,
						   "Oops addr miss-match! : 0x%08x != 0x%08x",
						   addr, chk_addr);
					return -EINVAL;

				}

				break;
			}
		}
	}

	if ((ILLEGAL_ID == sm_dsm_perf_ctrl_reg_id) &&
	    (ILLEGAL_ID == sm_dsm_perf_reg_id)) {
		return -EINVAL;
	}

	/* Skip the FECS extended header, nothing there for us now. */
	offset_to_segment += buffer_segments_size;

	/* skip through the GPCCS extended headers until we get to the data for
	 * our GPC.  The size of each gpc extended segment is enough to hold the
	 * max tpc count for the gpcs,in 256b chunks.
	 */

	max_tpc_count = gr->max_tpc_per_gpc_count;

	num_ext_gpccs_ext_buffer_segments = (u32)((max_tpc_count + 1) / 2);

	offset_to_segment += (num_ext_gpccs_ext_buffer_segments *
			      buffer_segments_size * gpc_num);

	num_tpcs = g->gr.gpc_tpc_count[gpc_num];

	/* skip the head marker to start with */
	inter_seg_offset = marker_size;

	if (ILLEGAL_ID != sm_dsm_perf_ctrl_reg_id) {
		/* skip over control regs of TPC's before the one we want.
		 *  then skip to the register in this tpc */
		inter_seg_offset = inter_seg_offset +
			(tpc_num * control_register_stride) +
			sm_dsm_perf_ctrl_reg_id;
	} else {
		/* skip all the control registers */
		inter_seg_offset = inter_seg_offset +
			(num_tpcs * control_register_stride);

		/* skip the marker between control and counter segments */
		inter_seg_offset += marker_size;

		/* skip over counter regs of TPCs before the one we want */
		inter_seg_offset = inter_seg_offset +
			(tpc_num * perf_register_stride) *
			ctxsw_prog_extended_num_smpc_quadrants_v();

		/* skip over the register for the quadrants we do not want.
		 *  then skip to the register in this tpc */
		inter_seg_offset = inter_seg_offset +
			(perf_register_stride * quad) +
			sm_dsm_perf_reg_id;
	}

	/* set the offset to the segment offset plus the inter segment offset to
	 *  our register */
	offset_to_segment += (inter_seg_offset * 4);

	/* last sanity check: did we somehow compute an offset outside the
	 * extended buffer? */
	if (offset_to_segment > offset_to_segment_end) {
		nvgpu_err(g,
			   "Overflow ctxsw buffer! 0x%08x > 0x%08x",
			   offset_to_segment, offset_to_segment_end);
		return -EINVAL;
	}

	*priv_offset = offset_to_segment;

	return 0;
}


static int
gr_gk20a_process_context_buffer_priv_segment(struct gk20a *g,
					     enum ctxsw_addr_type addr_type,
					     u32 pri_addr,
					     u32 gpc_num, u32 num_tpcs,
					     u32 num_ppcs, u32 ppc_mask,
					     u32 *priv_offset)
{
	u32 i;
	u32 address, base_address;
	u32 sys_offset, gpc_offset, tpc_offset, ppc_offset;
	u32 ppc_num, tpc_num, tpc_addr, gpc_addr, ppc_addr;
	struct aiv_gk20a *reg;
	u32 gpc_base = nvgpu_get_litter_value(g, GPU_LIT_GPC_BASE);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 ppc_in_gpc_base = nvgpu_get_litter_value(g, GPU_LIT_PPC_IN_GPC_BASE);
	u32 ppc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_PPC_IN_GPC_STRIDE);
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "pri_addr=0x%x", pri_addr);

	if (!g->gr.ctx_vars.valid) {
		return -EINVAL;
	}

	/* Process the SYS/BE segment. */
	if ((addr_type == CTXSW_ADDR_TYPE_SYS) ||
	    (addr_type == CTXSW_ADDR_TYPE_BE)) {
		for (i = 0; i < g->gr.ctx_vars.ctxsw_regs.sys.count; i++) {
			reg = &g->gr.ctx_vars.ctxsw_regs.sys.l[i];
			address    = reg->addr;
			sys_offset = reg->index;

			if (pri_addr == address) {
				*priv_offset = sys_offset;
				return 0;
			}
		}
	}

	/* Process the TPC segment. */
	if (addr_type == CTXSW_ADDR_TYPE_TPC) {
		for (tpc_num = 0; tpc_num < num_tpcs; tpc_num++) {
			for (i = 0; i < g->gr.ctx_vars.ctxsw_regs.tpc.count; i++) {
				reg = &g->gr.ctx_vars.ctxsw_regs.tpc.l[i];
				address = reg->addr;
				tpc_addr = pri_tpccs_addr_mask(address);
				base_address = gpc_base +
					(gpc_num * gpc_stride) +
					tpc_in_gpc_base +
					(tpc_num * tpc_in_gpc_stride);
				address = base_address + tpc_addr;
				/*
				 * The data for the TPCs is interleaved in the context buffer.
				 * Example with num_tpcs = 2
				 * 0    1    2    3    4    5    6    7    8    9    10   11 ...
				 * 0-0  1-0  0-1  1-1  0-2  1-2  0-3  1-3  0-4  1-4  0-5  1-5 ...
				 */
				tpc_offset = (reg->index * num_tpcs) + (tpc_num * 4);

				if (pri_addr == address) {
					*priv_offset = tpc_offset;
					return 0;
				}
			}
		}
	} else if ((addr_type == CTXSW_ADDR_TYPE_EGPC) ||
		(addr_type == CTXSW_ADDR_TYPE_ETPC)) {
		if (g->ops.gr.get_egpc_base == NULL) {
			return -EINVAL;
		}

		for (tpc_num = 0; tpc_num < num_tpcs; tpc_num++) {
			for (i = 0; i < g->gr.ctx_vars.ctxsw_regs.etpc.count; i++) {
				reg = &g->gr.ctx_vars.ctxsw_regs.etpc.l[i];
				address = reg->addr;
				tpc_addr = pri_tpccs_addr_mask(address);
				base_address = g->ops.gr.get_egpc_base(g) +
					(gpc_num * gpc_stride) +
					tpc_in_gpc_base +
					(tpc_num * tpc_in_gpc_stride);
				address = base_address + tpc_addr;
				/*
				 * The data for the TPCs is interleaved in the context buffer.
				 * Example with num_tpcs = 2
				 * 0    1    2    3    4    5    6    7    8    9    10   11 ...
				 * 0-0  1-0  0-1  1-1  0-2  1-2  0-3  1-3  0-4  1-4  0-5  1-5 ...
				 */
				tpc_offset = (reg->index * num_tpcs) + (tpc_num * 4);

				if (pri_addr == address) {
					*priv_offset = tpc_offset;
					nvgpu_log(g,
						gpu_dbg_fn | gpu_dbg_gpu_dbg,
						"egpc/etpc priv_offset=0x%#08x",
						*priv_offset);
					return 0;
				}
			}
		}
	}


	/* Process the PPC segment. */
	if (addr_type == CTXSW_ADDR_TYPE_PPC) {
		for (ppc_num = 0; ppc_num < num_ppcs; ppc_num++) {
			for (i = 0; i < g->gr.ctx_vars.ctxsw_regs.ppc.count; i++) {
				reg = &g->gr.ctx_vars.ctxsw_regs.ppc.l[i];
				address = reg->addr;
				ppc_addr = pri_ppccs_addr_mask(address);
				base_address = gpc_base +
					(gpc_num * gpc_stride) +
					ppc_in_gpc_base +
					(ppc_num * ppc_in_gpc_stride);
				address = base_address + ppc_addr;
				/*
				 * The data for the PPCs is interleaved in the context buffer.
				 * Example with numPpcs = 2
				 * 0    1    2    3    4    5    6    7    8    9    10   11 ...
				 * 0-0  1-0  0-1  1-1  0-2  1-2  0-3  1-3  0-4  1-4  0-5  1-5 ...
				 */
				ppc_offset = (reg->index * num_ppcs) + (ppc_num * 4);

				if (pri_addr == address)  {
					*priv_offset = ppc_offset;
					return 0;
				}
			}
		}
	}


	/* Process the GPC segment. */
	if (addr_type == CTXSW_ADDR_TYPE_GPC) {
		for (i = 0; i < g->gr.ctx_vars.ctxsw_regs.gpc.count; i++) {
			reg = &g->gr.ctx_vars.ctxsw_regs.gpc.l[i];

			address = reg->addr;
			gpc_addr = pri_gpccs_addr_mask(address);
			gpc_offset = reg->index;

			base_address = gpc_base + (gpc_num * gpc_stride);
			address = base_address + gpc_addr;

			if (pri_addr == address) {
				*priv_offset = gpc_offset;
				return 0;
			}
		}
	}
	return -EINVAL;
}

static int gr_gk20a_determine_ppc_configuration(struct gk20a *g,
					       u8 *context,
					       u32 *num_ppcs, u32 *ppc_mask,
					       u32 *reg_ppc_count)
{
	u32 data32;
	u32 num_pes_per_gpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_PES_PER_GPC);

	/*
	 * if there is only 1 PES_PER_GPC, then we put the PES registers
	 * in the GPC reglist, so we can't error out if ppc.count == 0
	 */
	if ((!g->gr.ctx_vars.valid) ||
	    ((g->gr.ctx_vars.ctxsw_regs.ppc.count == 0) &&
	     (num_pes_per_gpc > 1))) {
		return -EINVAL;
	}

	data32 = *(u32 *)(context + ctxsw_prog_local_image_ppc_info_o());

	*num_ppcs = ctxsw_prog_local_image_ppc_info_num_ppcs_v(data32);
	*ppc_mask = ctxsw_prog_local_image_ppc_info_ppc_mask_v(data32);

	*reg_ppc_count = g->gr.ctx_vars.ctxsw_regs.ppc.count;

	return 0;
}

int gr_gk20a_get_offset_in_gpccs_segment(struct gk20a *g,
					enum ctxsw_addr_type addr_type,
					u32 num_tpcs,
					u32 num_ppcs,
					u32 reg_list_ppc_count,
					u32 *__offset_in_segment)
{
	u32 offset_in_segment = 0;
	struct gr_gk20a *gr = &g->gr;

	if (addr_type == CTXSW_ADDR_TYPE_TPC) {
		/*
		 * reg = gr->ctx_vars.ctxsw_regs.tpc.l;
		 * offset_in_segment = 0;
		 */
	} else if ((addr_type == CTXSW_ADDR_TYPE_EGPC) ||
			(addr_type == CTXSW_ADDR_TYPE_ETPC)) {
		offset_in_segment =
			((gr->ctx_vars.ctxsw_regs.tpc.count *
				num_tpcs) << 2);

		nvgpu_log(g, gpu_dbg_info | gpu_dbg_gpu_dbg,
			"egpc etpc offset_in_segment 0x%#08x",
			offset_in_segment);
	} else if (addr_type == CTXSW_ADDR_TYPE_PPC) {
		/*
		 * The ucode stores TPC data before PPC data.
		 * Advance offset past TPC data to PPC data.
		 */
		offset_in_segment =
			(((gr->ctx_vars.ctxsw_regs.tpc.count +
				gr->ctx_vars.ctxsw_regs.etpc.count) *
			  num_tpcs) << 2);
	} else if (addr_type == CTXSW_ADDR_TYPE_GPC) {
		/*
		 * The ucode stores TPC/PPC data before GPC data.
		 * Advance offset past TPC/PPC data to GPC data.
		 *
		 * Note 1 PES_PER_GPC case
		 */
		u32 num_pes_per_gpc = nvgpu_get_litter_value(g,
				GPU_LIT_NUM_PES_PER_GPC);
		if (num_pes_per_gpc > 1) {
			offset_in_segment =
				((((gr->ctx_vars.ctxsw_regs.tpc.count +
					gr->ctx_vars.ctxsw_regs.etpc.count) *
					num_tpcs) << 2) +
					((reg_list_ppc_count * num_ppcs) << 2));
		} else {
			offset_in_segment =
				(((gr->ctx_vars.ctxsw_regs.tpc.count +
					gr->ctx_vars.ctxsw_regs.etpc.count) *
					num_tpcs) << 2);
		}
	} else {
		nvgpu_log_fn(g, "Unknown address type.");
		return -EINVAL;
	}

	*__offset_in_segment = offset_in_segment;
	return 0;
}

/*
 *  This function will return the 32 bit offset for a priv register if it is
 *  present in the context buffer. The context buffer is in CPU memory.
 */
static int gr_gk20a_find_priv_offset_in_buffer(struct gk20a *g,
					       u32 addr,
					       bool is_quad, u32 quad,
					       u32 *context_buffer,
					       u32 context_buffer_size,
					       u32 *priv_offset)
{
	u32 i, data32;
	int err;
	enum ctxsw_addr_type addr_type;
	u32 broadcast_flags;
	u32 gpc_num, tpc_num, ppc_num, be_num;
	u32 num_gpcs, num_tpcs, num_ppcs;
	u32 offset;
	u32 sys_priv_offset, gpc_priv_offset;
	u32 ppc_mask, reg_list_ppc_count;
	u8 *context;
	u32 offset_to_segment, offset_in_segment = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	err = g->ops.gr.decode_priv_addr(g, addr, &addr_type,
					&gpc_num, &tpc_num, &ppc_num, &be_num,
					&broadcast_flags);
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"addr_type = %d, broadcast_flags: %08x",
			addr_type, broadcast_flags);
	if (err != 0) {
		return err;
	}

	context = (u8 *)context_buffer;
	if (!check_main_image_header_magic(context)) {
		nvgpu_err(g,
			   "Invalid main header: magic value");
		return -EINVAL;
	}
	num_gpcs = *(u32 *)(context + ctxsw_prog_main_image_num_gpcs_o());

	/* Parse the FECS local header. */
	context += ctxsw_prog_ucode_header_size_in_bytes();
	if (!check_local_header_magic(context)) {
		nvgpu_err(g,
			   "Invalid FECS local header: magic value");
		return -EINVAL;
	}
	data32 = *(u32 *)(context + ctxsw_prog_local_priv_register_ctl_o());
	sys_priv_offset = ctxsw_prog_local_priv_register_ctl_offset_v(data32);
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "sys_priv_offset=0x%x", sys_priv_offset);

	/* If found in Ext buffer, ok.
	 * If it failed and we expected to find it there (quad offset)
	 * then return the error.  Otherwise continue on.
	 */
	err = gr_gk20a_find_priv_offset_in_ext_buffer(g,
				      addr, is_quad, quad, context_buffer,
				      context_buffer_size, priv_offset);
	if ((err == 0) || ((err != 0) && is_quad)) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"err = %d, is_quad = %s",
				err, is_quad ? "true" : "false");
		return err;
	}

	if ((addr_type == CTXSW_ADDR_TYPE_SYS) ||
	    (addr_type == CTXSW_ADDR_TYPE_BE)) {
		/* Find the offset in the FECS segment. */
		offset_to_segment = sys_priv_offset *
			ctxsw_prog_ucode_header_size_in_bytes();

		err = gr_gk20a_process_context_buffer_priv_segment(g,
					   addr_type, addr,
					   0, 0, 0, 0,
					   &offset);
		if (err != 0) {
			return err;
		}

		*priv_offset = (offset_to_segment + offset);
		return 0;
	}

	if ((gpc_num + 1) > num_gpcs)  {
		nvgpu_err(g,
			   "GPC %d not in this context buffer.",
			   gpc_num);
		return -EINVAL;
	}

	/* Parse the GPCCS local header(s).*/
	for (i = 0; i < num_gpcs; i++) {
		context += ctxsw_prog_ucode_header_size_in_bytes();
		if (!check_local_header_magic(context)) {
			nvgpu_err(g,
				   "Invalid GPCCS local header: magic value");
			return -EINVAL;

		}
		data32 = *(u32 *)(context + ctxsw_prog_local_priv_register_ctl_o());
		gpc_priv_offset = ctxsw_prog_local_priv_register_ctl_offset_v(data32);

		err = gr_gk20a_determine_ppc_configuration(g, context,
							   &num_ppcs, &ppc_mask,
							   &reg_list_ppc_count);
		if (err != 0) {
			nvgpu_err(g, "determine ppc configuration failed");
			return err;
		}


		num_tpcs = *(u32 *)(context + ctxsw_prog_local_image_num_tpcs_o());

		if ((i == gpc_num) && ((tpc_num + 1) > num_tpcs)) {
			nvgpu_err(g,
			   "GPC %d TPC %d not in this context buffer.",
				   gpc_num, tpc_num);
			return -EINVAL;
		}

		/* Find the offset in the GPCCS segment.*/
		if (i == gpc_num) {
			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"gpc_priv_offset 0x%#08x",
					gpc_priv_offset);
			offset_to_segment = gpc_priv_offset *
				ctxsw_prog_ucode_header_size_in_bytes();

			err = g->ops.gr.get_offset_in_gpccs_segment(g,
					addr_type,
					num_tpcs, num_ppcs, reg_list_ppc_count,
					&offset_in_segment);
			if (err != 0) {
				return -EINVAL;
			}

			offset_to_segment += offset_in_segment;
			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"offset_to_segment 0x%#08x",
				offset_to_segment);

			err = gr_gk20a_process_context_buffer_priv_segment(g,
							   addr_type, addr,
							   i, num_tpcs,
							   num_ppcs, ppc_mask,
							   &offset);
			if (err != 0) {
				return -EINVAL;
			}

			*priv_offset = offset_to_segment + offset;
			return 0;
		}
	}

	return -EINVAL;
}

static int map_cmp(const void *a, const void *b)
{
	struct ctxsw_buf_offset_map_entry *e1 =
					(struct ctxsw_buf_offset_map_entry *)a;
	struct ctxsw_buf_offset_map_entry *e2 =
					(struct ctxsw_buf_offset_map_entry *)b;

	if (e1->addr < e2->addr) {
		return -1;
	}

	if (e1->addr > e2->addr) {
		return 1;
	}
	return 0;
}

static int add_ctxsw_buffer_map_entries_pmsys(struct ctxsw_buf_offset_map_entry *map,
						struct aiv_list_gk20a *regs,
						u32 *count, u32 *offset,
						u32 max_cnt, u32 base, u32 mask)
{
	u32 idx;
	u32 cnt = *count;
	u32 off = *offset;

	if ((cnt + regs->count) > max_cnt) {
		return -EINVAL;
	}

	for (idx = 0; idx < regs->count; idx++) {
		if ((base + (regs->l[idx].addr & mask)) < 0xFFF) {
			map[cnt].addr = base + (regs->l[idx].addr & mask)
					+ NV_PCFG_BASE;
		} else {
			map[cnt].addr = base + (regs->l[idx].addr & mask);
		}
		map[cnt++].offset = off;
		off += 4;
	}
	*count = cnt;
	*offset = off;
	return 0;
}

static int add_ctxsw_buffer_map_entries_pmgpc(struct gk20a *g,
					struct ctxsw_buf_offset_map_entry *map,
					struct aiv_list_gk20a *regs,
					u32 *count, u32 *offset,
					u32 max_cnt, u32 base, u32 mask)
{
	u32 idx;
	u32 cnt = *count;
	u32 off = *offset;

	if ((cnt + regs->count) > max_cnt) {
		return -EINVAL;
	}

	/* NOTE: The PPC offsets get added to the pm_gpc list if numPpc <= 1
	 * To handle the case of PPC registers getting added into GPC, the below
	 * code specifically checks for any PPC offsets and adds them using
	 * proper mask
	 */
	for (idx = 0; idx < regs->count; idx++) {
		/* Check if the address is PPC address */
		if (pri_is_ppc_addr_shared(g, regs->l[idx].addr & mask)) {
			u32 ppc_in_gpc_base = nvgpu_get_litter_value(g,
						GPU_LIT_PPC_IN_GPC_BASE);
			u32 ppc_in_gpc_stride = nvgpu_get_litter_value(g,
						GPU_LIT_PPC_IN_GPC_STRIDE);
			/* Use PPC mask instead of the GPC mask provided */
			u32 ppcmask = ppc_in_gpc_stride - 1;

			map[cnt].addr = base + ppc_in_gpc_base
					+ (regs->l[idx].addr & ppcmask);
		} else {
			map[cnt].addr = base + (regs->l[idx].addr & mask);
		}
		map[cnt++].offset = off;
		off += 4;
	}
	*count = cnt;
	*offset = off;
	return 0;
}

static int add_ctxsw_buffer_map_entries(struct ctxsw_buf_offset_map_entry *map,
					struct aiv_list_gk20a *regs,
					u32 *count, u32 *offset,
					u32 max_cnt, u32 base, u32 mask)
{
	u32 idx;
	u32 cnt = *count;
	u32 off = *offset;

	if ((cnt + regs->count) > max_cnt) {
		return -EINVAL;
	}

	for (idx = 0; idx < regs->count; idx++) {
		map[cnt].addr = base + (regs->l[idx].addr & mask);
		map[cnt++].offset = off;
		off += 4;
	}
	*count = cnt;
	*offset = off;
	return 0;
}

/* Helper function to add register entries to the register map for all
 * subunits
 */
static int add_ctxsw_buffer_map_entries_subunits(
					struct ctxsw_buf_offset_map_entry *map,
					struct aiv_list_gk20a *regs,
					u32 *count, u32 *offset,
					u32 max_cnt, u32 base,
					u32 num_units, u32 stride, u32 mask)
{
	u32 unit;
	u32 idx;
	u32 cnt = *count;
	u32 off = *offset;

	if ((cnt + (regs->count * num_units)) > max_cnt) {
		return -EINVAL;
	}

	/* Data is interleaved for units in ctxsw buffer */
	for (idx = 0; idx < regs->count; idx++) {
		for (unit = 0; unit < num_units; unit++) {
			map[cnt].addr = base + (regs->l[idx].addr & mask) +
					(unit * stride);
			map[cnt++].offset = off;
			off += 4;
		}
	}
	*count = cnt;
	*offset = off;
	return 0;
}

int gr_gk20a_add_ctxsw_reg_pm_fbpa(struct gk20a *g,
				struct ctxsw_buf_offset_map_entry *map,
				struct aiv_list_gk20a *regs,
				u32 *count, u32 *offset,
				u32 max_cnt, u32 base,
				u32 num_fbpas, u32 stride, u32 mask)
{
	return add_ctxsw_buffer_map_entries_subunits(map, regs, count, offset,
			max_cnt, base, num_fbpas, stride, mask);
}

static int add_ctxsw_buffer_map_entries_gpcs(struct gk20a *g,
					struct ctxsw_buf_offset_map_entry *map,
					u32 *count, u32 *offset, u32 max_cnt)
{
	u32 num_gpcs = g->gr.gpc_count;
	u32 num_ppcs, num_tpcs, gpc_num, base;
	u32 gpc_base = nvgpu_get_litter_value(g, GPU_LIT_GPC_BASE);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 ppc_in_gpc_base = nvgpu_get_litter_value(g, GPU_LIT_PPC_IN_GPC_BASE);
	u32 ppc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_PPC_IN_GPC_STRIDE);
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);

	for (gpc_num = 0; gpc_num < num_gpcs; gpc_num++) {
		num_tpcs = g->gr.gpc_tpc_count[gpc_num];
		base = gpc_base + (gpc_stride * gpc_num) + tpc_in_gpc_base;
		if (add_ctxsw_buffer_map_entries_subunits(map,
					&g->gr.ctx_vars.ctxsw_regs.pm_tpc,
					count, offset, max_cnt, base, num_tpcs,
					tpc_in_gpc_stride,
					(tpc_in_gpc_stride - 1))) {
			return -EINVAL;
		}

		num_ppcs = g->gr.gpc_ppc_count[gpc_num];
		base = gpc_base + (gpc_stride * gpc_num) + ppc_in_gpc_base;
		if (add_ctxsw_buffer_map_entries_subunits(map,
					&g->gr.ctx_vars.ctxsw_regs.pm_ppc,
					count, offset, max_cnt, base, num_ppcs,
					ppc_in_gpc_stride,
					(ppc_in_gpc_stride - 1))) {
			return -EINVAL;
		}

		base = gpc_base + (gpc_stride * gpc_num);
		if (add_ctxsw_buffer_map_entries_pmgpc(g, map,
					&g->gr.ctx_vars.ctxsw_regs.pm_gpc,
					count, offset, max_cnt, base,
					(gpc_stride - 1))) {
			return -EINVAL;
		}

		base = NV_XBAR_MXBAR_PRI_GPC_GNIC_STRIDE * gpc_num;
		if (add_ctxsw_buffer_map_entries(map,
					&g->gr.ctx_vars.ctxsw_regs.pm_ucgpc,
					count, offset, max_cnt, base, ~0)) {
			return -EINVAL;
		}

		base = (g->ops.gr.get_pmm_per_chiplet_offset() * gpc_num);
		if (add_ctxsw_buffer_map_entries(map,
					&g->gr.ctx_vars.ctxsw_regs.perf_gpc,
					count, offset, max_cnt, base, ~0)) {
			return -EINVAL;
		}

		base = (NV_PERF_PMMGPCROUTER_STRIDE * gpc_num);
		if (add_ctxsw_buffer_map_entries(map,
					&g->gr.ctx_vars.ctxsw_regs.gpc_router,
					count, offset, max_cnt, base, ~0)) {
			return -EINVAL;
		}

		/* Counter Aggregation Unit, if available */
		if (g->gr.ctx_vars.ctxsw_regs.pm_cau.count) {
			base = gpc_base + (gpc_stride * gpc_num)
					+ tpc_in_gpc_base;
			if (add_ctxsw_buffer_map_entries_subunits(map,
					&g->gr.ctx_vars.ctxsw_regs.pm_cau,
					count, offset, max_cnt, base, num_tpcs,
					tpc_in_gpc_stride,
					(tpc_in_gpc_stride - 1))) {
				return -EINVAL;
			}
		}

		*offset = ALIGN(*offset, 256);
	}
	return 0;
}

int gr_gk20a_add_ctxsw_reg_perf_pma(struct ctxsw_buf_offset_map_entry *map,
	struct aiv_list_gk20a *regs,
	u32 *count, u32 *offset,
	u32 max_cnt, u32 base, u32 mask)
{
	return add_ctxsw_buffer_map_entries(map, regs,
			count, offset, max_cnt, base, mask);
}

/*
 *            PM CTXSW BUFFER LAYOUT :
 *|---------------------------------------------|0x00 <----PM CTXSW BUFFER BASE
 *|                                             |
 *|        LIST_compressed_pm_ctx_reg_SYS       |Space allocated: numRegs words
 *|---------------------------------------------|
 *|                                             |
 *|    LIST_compressed_nv_perf_ctx_reg_SYS      |Space allocated: numRegs words
 *|---------------------------------------------|
 *|                                             |
 *|    LIST_compressed_nv_perf_ctx_reg_sysrouter|Space allocated: numRegs words
 *|---------------------------------------------|
 *|                                             |
 *|    LIST_compressed_nv_perf_ctx_reg_PMA      |Space allocated: numRegs words
 *|---------------------------------------------|
 *|        PADDING for 256 byte alignment       |
 *|---------------------------------------------|<----256 byte aligned
 *|    LIST_compressed_nv_perf_fbp_ctx_regs     |
 *|                                             |Space allocated: numRegs * n words (for n FB units)
 *|---------------------------------------------|
 *| LIST_compressed_nv_perf_fbprouter_ctx_regs  |
 *|                                             |Space allocated: numRegs * n words (for n FB units)
 *|---------------------------------------------|
 *|    LIST_compressed_pm_fbpa_ctx_regs         |
 *|                                             |Space allocated: numRegs * n words (for n FB units)
 *|---------------------------------------------|
 *|    LIST_compressed_pm_rop_ctx_regs          |
 *|---------------------------------------------|
 *|    LIST_compressed_pm_ltc_ctx_regs          |
 *|                                  LTC0 LTS0  |
 *|                                  LTC1 LTS0  |Space allocated: numRegs * n words (for n LTC units)
 *|                                  LTCn LTS0  |
 *|                                  LTC0 LTS1  |
 *|                                  LTC1 LTS1  |
 *|                                  LTCn LTS1  |
 *|                                  LTC0 LTSn  |
 *|                                  LTC1 LTSn  |
 *|                                  LTCn LTSn  |
 *|---------------------------------------------|
 *|        PADDING for 256 byte alignment       |
 *|---------------------------------------------|<----256 byte aligned
 *|                            GPC0  REG0 TPC0  |Each GPC has space allocated to accommodate
 *|                                  REG0 TPC1  |    all the GPC/TPC register lists
 *| Lists in each GPC region:        REG0 TPCn  |Per GPC allocated space is always 256 byte aligned
 *|  LIST_pm_ctx_reg_TPC             REG1 TPC0  |
 *|             * numTpcs            REG1 TPC1  |
 *|  LIST_pm_ctx_reg_PPC             REG1 TPCn  |
 *|             * numPpcs            REGn TPC0  |
 *|  LIST_pm_ctx_reg_GPC             REGn TPC1  |
 *|  List_pm_ctx_reg_uc_GPC          REGn TPCn  |
 *|  LIST_nv_perf_ctx_reg_GPC                   |
 *|  LIST_nv_perf_gpcrouter_ctx_reg             |
 *|  LIST_nv_perf_ctx_reg_CAU                   |
 *|                                       ----  |--
 *|                            GPC1         .   |
 *|                                         .   |<----
 *|---------------------------------------------|
 *=                                             =
 *|                            GPCn             |
 *=                                             =
 *|---------------------------------------------|
 */

static int gr_gk20a_create_hwpm_ctxsw_buffer_offset_map(struct gk20a *g)
{
	u32 hwpm_ctxsw_buffer_size = g->gr.ctx_vars.pm_ctxsw_image_size;
	u32 hwpm_ctxsw_reg_count_max;
	u32 map_size;
	u32 i, count = 0;
	u32 offset = 0;
	struct ctxsw_buf_offset_map_entry *map;
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 num_fbpas = nvgpu_get_litter_value(g, GPU_LIT_NUM_FBPAS);
	u32 fbpa_stride = nvgpu_get_litter_value(g, GPU_LIT_FBPA_STRIDE);
	u32 num_ltc = g->ops.gr.get_max_ltc_per_fbp(g) * g->gr.num_fbps;

	if (hwpm_ctxsw_buffer_size == 0) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"no PM Ctxsw buffer memory in context buffer");
		return -EINVAL;
	}

	hwpm_ctxsw_reg_count_max = hwpm_ctxsw_buffer_size >> 2;
	map_size = hwpm_ctxsw_reg_count_max * sizeof(*map);

	map = nvgpu_big_zalloc(g, map_size);
	if (map == NULL) {
		return -ENOMEM;
	}

	/* Add entries from _LIST_pm_ctx_reg_SYS */
	if (add_ctxsw_buffer_map_entries_pmsys(map, &g->gr.ctx_vars.ctxsw_regs.pm_sys,
				&count, &offset, hwpm_ctxsw_reg_count_max, 0, ~0)) {
		goto cleanup;
	}

	/* Add entries from _LIST_nv_perf_ctx_reg_SYS */
	if (add_ctxsw_buffer_map_entries(map, &g->gr.ctx_vars.ctxsw_regs.perf_sys,
				&count, &offset, hwpm_ctxsw_reg_count_max, 0, ~0)) {
		goto cleanup;
	}

	/* Add entries from _LIST_nv_perf_sysrouter_ctx_reg*/
	if (add_ctxsw_buffer_map_entries(map, &g->gr.ctx_vars.ctxsw_regs.perf_sys_router,
				&count, &offset, hwpm_ctxsw_reg_count_max, 0, ~0)) {
		goto cleanup;
	}

	/* Add entries from _LIST_nv_perf_pma_ctx_reg*/
	if (g->ops.gr.add_ctxsw_reg_perf_pma(map, &g->gr.ctx_vars.ctxsw_regs.perf_pma,
				&count, &offset, hwpm_ctxsw_reg_count_max, 0, ~0)) {
		goto cleanup;
	}

	offset = ALIGN(offset, 256);

	/* Add entries from _LIST_nv_perf_fbp_ctx_regs */
	if (add_ctxsw_buffer_map_entries_subunits(map,
					&g->gr.ctx_vars.ctxsw_regs.fbp,
					&count, &offset,
					hwpm_ctxsw_reg_count_max, 0,
					g->gr.num_fbps,
					g->ops.gr.get_pmm_per_chiplet_offset(),
					~0)) {
		goto cleanup;
	}

	/* Add entries from _LIST_nv_perf_fbprouter_ctx_regs */
	if (add_ctxsw_buffer_map_entries_subunits(map,
					&g->gr.ctx_vars.ctxsw_regs.fbp_router,
					&count, &offset,
					hwpm_ctxsw_reg_count_max, 0, g->gr.num_fbps,
					NV_PERF_PMM_FBP_ROUTER_STRIDE, ~0)) {
		goto cleanup;
	}

	/* Add entries from _LIST_nv_pm_fbpa_ctx_regs */
	if (g->ops.gr.add_ctxsw_reg_pm_fbpa(g, map,
					&g->gr.ctx_vars.ctxsw_regs.pm_fbpa,
					&count, &offset,
					hwpm_ctxsw_reg_count_max, 0,
					num_fbpas, fbpa_stride, ~0)) {
		goto cleanup;
	}

	/* Add entries from _LIST_nv_pm_rop_ctx_regs */
	if (add_ctxsw_buffer_map_entries(map,
					&g->gr.ctx_vars.ctxsw_regs.pm_rop,
					&count, &offset,
					hwpm_ctxsw_reg_count_max, 0, ~0)) {
		goto cleanup;
	}

	/* Add entries from _LIST_compressed_nv_pm_ltc_ctx_regs */
	if (add_ctxsw_buffer_map_entries_subunits(map,
					&g->gr.ctx_vars.ctxsw_regs.pm_ltc,
					&count, &offset,
					hwpm_ctxsw_reg_count_max, 0,
					num_ltc, ltc_stride, ~0)) {
		goto cleanup;
	}

	offset = ALIGN(offset, 256);

	/* Add GPC entries */
	if (add_ctxsw_buffer_map_entries_gpcs(g, map, &count, &offset,
					hwpm_ctxsw_reg_count_max)) {
		goto cleanup;
	}

	if (offset > hwpm_ctxsw_buffer_size) {
		nvgpu_err(g, "offset > buffer size");
		goto cleanup;
	}

	sort(map, count, sizeof(*map), map_cmp, NULL);

	g->gr.ctx_vars.hwpm_ctxsw_buffer_offset_map = map;
	g->gr.ctx_vars.hwpm_ctxsw_buffer_offset_map_count = count;

	nvgpu_log_info(g, "Reg Addr => HWPM Ctxt switch buffer offset");

	for (i = 0; i < count; i++) {
		nvgpu_log_info(g, "%08x => %08x", map[i].addr, map[i].offset);
	}

	return 0;
cleanup:
	nvgpu_err(g, "Failed to create HWPM buffer offset map");
	nvgpu_big_free(g, map);
	return -EINVAL;
}

/*
 *  This function will return the 32 bit offset for a priv register if it is
 *  present in the PM context buffer.
 */
static int gr_gk20a_find_priv_offset_in_pm_buffer(struct gk20a *g,
					          u32 addr,
					          u32 *priv_offset)
{
	struct gr_gk20a *gr = &g->gr;
	int err = 0;
	u32 count;
	struct ctxsw_buf_offset_map_entry *map, *result, map_key;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	/* Create map of pri address and pm offset if necessary */
	if (gr->ctx_vars.hwpm_ctxsw_buffer_offset_map == NULL) {
		err = gr_gk20a_create_hwpm_ctxsw_buffer_offset_map(g);
		if (err != 0) {
			return err;
		}
	}

	*priv_offset = 0;

	map = gr->ctx_vars.hwpm_ctxsw_buffer_offset_map;
	count = gr->ctx_vars.hwpm_ctxsw_buffer_offset_map_count;

	map_key.addr = addr;
	result = bsearch(&map_key, map, count, sizeof(*map), map_cmp);

	if (result) {
		*priv_offset = result->offset;
	} else {
		nvgpu_err(g, "Lookup failed for address 0x%x", addr);
		err = -EINVAL;
	}
	return err;
}

bool gk20a_is_channel_ctx_resident(struct channel_gk20a *ch)
{
	int curr_gr_ctx;
	u32 curr_gr_tsgid;
	struct gk20a *g = ch->g;
	struct channel_gk20a *curr_ch;
	bool ret = false;
	struct tsg_gk20a *tsg;

	curr_gr_ctx  = gk20a_readl(g, gr_fecs_current_ctx_r());

	/* when contexts are unloaded from GR, the valid bit is reset
	 * but the instance pointer information remains intact. So the
	 * valid bit must be checked to be absolutely certain that a
	 * valid context is currently resident.
	 */
	if (gr_fecs_current_ctx_valid_v(curr_gr_ctx) == 0U) {
		return NULL;
	}

	curr_ch = gk20a_gr_get_channel_from_ctx(g, curr_gr_ctx,
					      &curr_gr_tsgid);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
		  "curr_gr_chid=%d curr_tsgid=%d, ch->tsgid=%d"
		  " ch->chid=%d",
		  (curr_ch != NULL) ? curr_ch->chid : U32_MAX,
		  curr_gr_tsgid,
		  ch->tsgid,
		  ch->chid);

	if (curr_ch == NULL) {
		return false;
	}

	if (ch->chid == curr_ch->chid) {
		ret = true;
	}

	tsg = tsg_gk20a_from_ch(ch);
	if ((tsg != NULL) && (tsg->tsgid == curr_gr_tsgid)) {
		ret = true;
	}

	gk20a_channel_put(curr_ch);
	return ret;
}

int __gr_gk20a_exec_ctx_ops(struct channel_gk20a *ch,
			    struct nvgpu_dbg_reg_op *ctx_ops, u32 num_ops,
			    u32 num_ctx_wr_ops, u32 num_ctx_rd_ops,
			    bool ch_is_curr_ctx)
{
	struct gk20a *g = ch->g;
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;
	bool gr_ctx_ready = false;
	bool pm_ctx_ready = false;
	struct nvgpu_mem *current_mem = NULL;
	u32 i, j, offset, v;
	struct gr_gk20a *gr = &g->gr;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	u32 max_offsets = gr->max_gpc_count * gr->max_tpc_per_gpc_count *
				sm_per_tpc;
	u32 *offsets = NULL;
	u32 *offset_addrs = NULL;
	u32 ctx_op_nr, num_ctx_ops[2] = {num_ctx_wr_ops, num_ctx_rd_ops};
	int err = 0, pass;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "wr_ops=%d rd_ops=%d",
		   num_ctx_wr_ops, num_ctx_rd_ops);

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;

	if (ch_is_curr_ctx) {
		for (pass = 0; pass < 2; pass++) {
			ctx_op_nr = 0;
			for (i = 0; (ctx_op_nr < num_ctx_ops[pass]) && (i < num_ops); ++i) {
				/* only do ctx ops and only on the right pass */
				if ((ctx_ops[i].type == REGOP(TYPE_GLOBAL)) ||
				    (((pass == 0) && reg_op_is_read(ctx_ops[i].op)) ||
				     ((pass == 1) && !reg_op_is_read(ctx_ops[i].op)))) {
					continue;
				}

				/* if this is a quad access, setup for special access*/
				if ((ctx_ops[i].type == REGOP(TYPE_GR_CTX_QUAD))
					&& (g->ops.gr.access_smpc_reg != NULL)) {
					g->ops.gr.access_smpc_reg(g,
							ctx_ops[i].quad,
							ctx_ops[i].offset);
				}
				offset = ctx_ops[i].offset;

				if (pass == 0) { /* write pass */
					v = gk20a_readl(g, offset);
					v &= ~ctx_ops[i].and_n_mask_lo;
					v |= ctx_ops[i].value_lo;
					gk20a_writel(g, offset, v);

					nvgpu_log(g, gpu_dbg_gpu_dbg,
						   "direct wr: offset=0x%x v=0x%x",
						   offset, v);

					if (ctx_ops[i].op == REGOP(WRITE_64)) {
						v = gk20a_readl(g, offset + 4);
						v &= ~ctx_ops[i].and_n_mask_hi;
						v |= ctx_ops[i].value_hi;
						gk20a_writel(g, offset + 4, v);

						nvgpu_log(g, gpu_dbg_gpu_dbg,
							   "direct wr: offset=0x%x v=0x%x",
							   offset + 4, v);
					}

				} else { /* read pass */
					ctx_ops[i].value_lo =
						gk20a_readl(g, offset);

					nvgpu_log(g, gpu_dbg_gpu_dbg,
						   "direct rd: offset=0x%x v=0x%x",
						   offset, ctx_ops[i].value_lo);

					if (ctx_ops[i].op == REGOP(READ_64)) {
						ctx_ops[i].value_hi =
							gk20a_readl(g, offset + 4);

						nvgpu_log(g, gpu_dbg_gpu_dbg,
							   "direct rd: offset=0x%x v=0x%x",
							   offset, ctx_ops[i].value_lo);
					} else {
						ctx_ops[i].value_hi = 0;
					}
				}
				ctx_op_nr++;
			}
		}
		goto cleanup;
	}

	/* they're the same size, so just use one alloc for both */
	offsets = nvgpu_kzalloc(g, 2 * sizeof(u32) * max_offsets);
	if (offsets == NULL) {
		err = -ENOMEM;
		goto cleanup;
	}
	offset_addrs = offsets + max_offsets;

	err = gr_gk20a_ctx_patch_write_begin(g, gr_ctx, false);
	if (err != 0) {
		goto cleanup;
	}

	g->ops.mm.l2_flush(g, true);

	/* write to appropriate place in context image,
	 * first have to figure out where that really is */

	/* first pass is writes, second reads */
	for (pass = 0; pass < 2; pass++) {
		ctx_op_nr = 0;
		for (i = 0; (ctx_op_nr < num_ctx_ops[pass]) && (i < num_ops); ++i) {
			u32 num_offsets;

			/* only do ctx ops and only on the right pass */
			if ((ctx_ops[i].type == REGOP(TYPE_GLOBAL)) ||
			    (((pass == 0) && reg_op_is_read(ctx_ops[i].op)) ||
			     ((pass == 1) && !reg_op_is_read(ctx_ops[i].op)))) {
				continue;
			}

			err = gr_gk20a_get_ctx_buffer_offsets(g,
						ctx_ops[i].offset,
						max_offsets,
						offsets, offset_addrs,
						&num_offsets,
						ctx_ops[i].type == REGOP(TYPE_GR_CTX_QUAD),
						ctx_ops[i].quad);
			if (err == 0) {
				if (!gr_ctx_ready) {
					gr_ctx_ready = true;
				}
				current_mem = &gr_ctx->mem;
			} else {
				err = gr_gk20a_get_pm_ctx_buffer_offsets(g,
							ctx_ops[i].offset,
							max_offsets,
							offsets, offset_addrs,
							&num_offsets);
				if (err != 0) {
					nvgpu_log(g, gpu_dbg_gpu_dbg,
					   "ctx op invalid offset: offset=0x%x",
					   ctx_ops[i].offset);
					ctx_ops[i].status =
						REGOP(STATUS_INVALID_OFFSET);
					continue;
				}
				if (!pm_ctx_ready) {
					/* Make sure ctx buffer was initialized */
					if (!nvgpu_mem_is_valid(&gr_ctx->pm_ctx.mem)) {
						nvgpu_err(g,
							"Invalid ctx buffer");
						err = -EINVAL;
						goto cleanup;
					}
					pm_ctx_ready = true;
				}
				current_mem = &gr_ctx->pm_ctx.mem;
			}

			/* if this is a quad access, setup for special access*/
			if ((ctx_ops[i].type == REGOP(TYPE_GR_CTX_QUAD)) &&
				(g->ops.gr.access_smpc_reg != NULL)) {
				g->ops.gr.access_smpc_reg(g, ctx_ops[i].quad,
							 ctx_ops[i].offset);
			}

			for (j = 0; j < num_offsets; j++) {
				/* sanity check gr ctxt offsets,
				 * don't write outside, worst case
				 */
				if ((current_mem == &gr_ctx->mem) &&
					(offsets[j] >= g->gr.ctx_vars.golden_image_size)) {
					continue;
				}
				if (pass == 0) { /* write pass */
					v = nvgpu_mem_rd(g, current_mem, offsets[j]);
					v &= ~ctx_ops[i].and_n_mask_lo;
					v |= ctx_ops[i].value_lo;
					nvgpu_mem_wr(g, current_mem, offsets[j], v);

					nvgpu_log(g, gpu_dbg_gpu_dbg,
						   "context wr: offset=0x%x v=0x%x",
						   offsets[j], v);

					if (ctx_ops[i].op == REGOP(WRITE_64)) {
						v = nvgpu_mem_rd(g, current_mem, offsets[j] + 4);
						v &= ~ctx_ops[i].and_n_mask_hi;
						v |= ctx_ops[i].value_hi;
						nvgpu_mem_wr(g, current_mem, offsets[j] + 4, v);

						nvgpu_log(g, gpu_dbg_gpu_dbg,
							   "context wr: offset=0x%x v=0x%x",
							   offsets[j] + 4, v);
					}

					/* check to see if we need to add a special WAR
					   for some of the SMPC perf regs */
					gr_gk20a_ctx_patch_smpc(g, ch, offset_addrs[j],
							v, current_mem);

				} else { /* read pass */
					ctx_ops[i].value_lo =
						nvgpu_mem_rd(g, current_mem, offsets[0]);

					nvgpu_log(g, gpu_dbg_gpu_dbg, "context rd: offset=0x%x v=0x%x",
						   offsets[0], ctx_ops[i].value_lo);

					if (ctx_ops[i].op == REGOP(READ_64)) {
						ctx_ops[i].value_hi =
							nvgpu_mem_rd(g, current_mem, offsets[0] + 4);

						nvgpu_log(g, gpu_dbg_gpu_dbg,
							   "context rd: offset=0x%x v=0x%x",
							   offsets[0] + 4, ctx_ops[i].value_hi);
					} else {
						ctx_ops[i].value_hi = 0;
					}
				}
			}
			ctx_op_nr++;
		}
	}

 cleanup:
	if (offsets) {
		nvgpu_kfree(g, offsets);
	}

	if (gr_ctx->patch_ctx.mem.cpu_va) {
		gr_gk20a_ctx_patch_write_end(g, gr_ctx, gr_ctx_ready);
	}

	return err;
}

int gr_gk20a_exec_ctx_ops(struct channel_gk20a *ch,
			  struct nvgpu_dbg_reg_op *ctx_ops, u32 num_ops,
			  u32 num_ctx_wr_ops, u32 num_ctx_rd_ops,
			  bool *is_curr_ctx)
{
	struct gk20a *g = ch->g;
	int err, tmp_err;
	bool ch_is_curr_ctx;

	/* disable channel switching.
	 * at that point the hardware state can be inspected to
	 * determine if the context we're interested in is current.
	 */
	err = gr_gk20a_disable_ctxsw(g);
	if (err != 0) {
		nvgpu_err(g, "unable to stop gr ctxsw");
		/* this should probably be ctx-fatal... */
		return err;
	}

	ch_is_curr_ctx = gk20a_is_channel_ctx_resident(ch);
	if (is_curr_ctx != NULL) {
		*is_curr_ctx = ch_is_curr_ctx;
	}
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "is curr ctx=%d",
		  ch_is_curr_ctx);

	err = __gr_gk20a_exec_ctx_ops(ch, ctx_ops, num_ops, num_ctx_wr_ops,
				      num_ctx_rd_ops, ch_is_curr_ctx);

	tmp_err = gr_gk20a_enable_ctxsw(g);
	if (tmp_err) {
		nvgpu_err(g, "unable to restart ctxsw!");
		err = tmp_err;
	}

	return err;
}

void gr_gk20a_commit_global_pagepool(struct gk20a *g,
					    struct nvgpu_gr_ctx *gr_ctx,
					    u64 addr, u32 size, bool patch)
{
	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_scc_pagepool_base_r(),
		gr_scc_pagepool_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_scc_pagepool_r(),
		gr_scc_pagepool_total_pages_f(size) |
		gr_scc_pagepool_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_gcc_pagepool_base_r(),
		gr_gpcs_gcc_pagepool_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_gcc_pagepool_r(),
		gr_gpcs_gcc_pagepool_total_pages_f(size), patch);

	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_pd_pagepool_r(),
		gr_pd_pagepool_total_pages_f(size) |
		gr_pd_pagepool_valid_true_f(), patch);
}

void gk20a_init_gr(struct gk20a *g)
{
	nvgpu_cond_init(&g->gr.init_wq);
}

int gk20a_gr_wait_for_sm_lock_down(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors)
{
	bool locked_down;
	bool no_error_pending;
	u32 delay = GR_IDLE_CHECK_DEFAULT;
	bool mmu_debug_mode_enabled = g->ops.fb.is_debug_mode_enabled(g);
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);
	u32 dbgr_status0 = 0, dbgr_control0 = 0;
	u64 warps_valid = 0, warps_paused = 0, warps_trapped = 0;
	struct nvgpu_timeout timeout;
	u32 warp_esr;

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
		"GPC%d TPC%d SM%d: locking down SM", gpc, tpc, sm);

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	/* wait for the sm to lock down */
	do {
		u32 global_esr = g->ops.gr.get_sm_hww_global_esr(g,
						gpc, tpc, sm);
		dbgr_status0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm_dbgr_status0_r() + offset);

		warp_esr = g->ops.gr.get_sm_hww_warp_esr(g, gpc, tpc, sm);

		locked_down =
		    (gr_gpc0_tpc0_sm_dbgr_status0_locked_down_v(dbgr_status0) ==
		     gr_gpc0_tpc0_sm_dbgr_status0_locked_down_true_v());
		no_error_pending =
			check_errors &&
			(gr_gpc0_tpc0_sm_hww_warp_esr_error_v(warp_esr) ==
			 gr_gpc0_tpc0_sm_hww_warp_esr_error_none_v()) &&
			((global_esr & ~global_esr_mask) == 0);

		if (locked_down || no_error_pending) {
			nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				  "GPC%d TPC%d SM%d: locked down SM",
					gpc, tpc, sm);
			return 0;
		}

		/* if an mmu fault is pending and mmu debug mode is not
		 * enabled, the sm will never lock down. */
		if (!mmu_debug_mode_enabled &&
		     (g->ops.mm.mmu_fault_pending(g))) {
			nvgpu_err(g,
				"GPC%d TPC%d: mmu fault pending,"
				" SM%d will never lock down!", gpc, tpc, sm);
			return -EFAULT;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (nvgpu_timeout_expired(&timeout) == 0);

	dbgr_control0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm_dbgr_control0_r() + offset);

	/* 64 bit read */
	warps_valid = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_warp_valid_mask_1_r() + offset) << 32;
	warps_valid |= gk20a_readl(g, gr_gpc0_tpc0_sm_warp_valid_mask_r() + offset);

	/* 64 bit read */
	warps_paused = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_pause_mask_1_r() + offset) << 32;
	warps_paused |= gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_pause_mask_r() + offset);

	/* 64 bit read */
	warps_trapped = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_trap_mask_1_r() + offset) << 32;
	warps_trapped |= gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_trap_mask_r() + offset);

	nvgpu_err(g,
		"GPC%d TPC%d: timed out while trying to lock down SM", gpc, tpc);
	nvgpu_err(g,
		"STATUS0(0x%x)=0x%x CONTROL0=0x%x VALID_MASK=0x%llx PAUSE_MASK=0x%llx TRAP_MASK=0x%llx",
		gr_gpc0_tpc0_sm_dbgr_status0_r() + offset, dbgr_status0, dbgr_control0,
		warps_valid, warps_paused, warps_trapped);

	return -ETIMEDOUT;
}

void gk20a_gr_suspend_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors)
{
	int err;
	u32 dbgr_control0;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);

	/* if an SM debugger isn't attached, skip suspend */
	if (!g->ops.gr.sm_debugger_attached(g)) {
		nvgpu_err(g,
			"SM debugger not attached, skipping suspend!");
		return;
	}

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
		"suspending gpc:%d, tpc:%d, sm%d", gpc, tpc, sm);

	/* assert stop trigger. */
	dbgr_control0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm_dbgr_control0_r() + offset);
	dbgr_control0 |= gr_gpcs_tpcs_sm_dbgr_control0_stop_trigger_enable_f();
	gk20a_writel(g, gr_gpc0_tpc0_sm_dbgr_control0_r() + offset,
			dbgr_control0);

	err = g->ops.gr.wait_for_sm_lock_down(g, gpc, tpc, sm,
			global_esr_mask, check_errors);
	if (err != 0) {
		nvgpu_err(g,
			"SuspendSm failed");
		return;
	}
}

void gk20a_gr_suspend_all_sms(struct gk20a *g,
		u32 global_esr_mask, bool check_errors)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc, tpc, sm;
	int err;
	u32 dbgr_control0;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);

	/* if an SM debugger isn't attached, skip suspend */
	if (!g->ops.gr.sm_debugger_attached(g)) {
		nvgpu_err(g,
			"SM debugger not attached, skipping suspend!");
		return;
	}

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "suspending all sms");
	/* assert stop trigger. uniformity assumption: all SMs will have
	 * the same state in dbg_control0.
	 */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_control0_r());
	dbgr_control0 |= gr_gpcs_tpcs_sm_dbgr_control0_stop_trigger_enable_f();

	/* broadcast write */
	gk20a_writel(g,
		gr_gpcs_tpcs_sm_dbgr_control0_r(), dbgr_control0);

	for (gpc = 0; gpc < gr->gpc_count; gpc++) {
		for (tpc = 0; tpc < gr_gk20a_get_tpc_count(gr, gpc); tpc++) {
			for (sm = 0; sm < sm_per_tpc; sm++) {
				err = g->ops.gr.wait_for_sm_lock_down(g,
					gpc, tpc, sm,
					global_esr_mask, check_errors);
				if (err != 0) {
					nvgpu_err(g, "SuspendAllSms failed");
					return;
				}
			}
		}
	}
}

void gk20a_gr_resume_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm)
{
	u32 dbgr_control0;
	u32 offset;
	/*
	 * The following requires some clarification. Despite the fact that both
	 * RUN_TRIGGER and STOP_TRIGGER have the word "TRIGGER" in their
	 *  names, only one is actually a trigger, and that is the STOP_TRIGGER.
	 * Merely writing a 1(_TASK) to the RUN_TRIGGER is not sufficient to
	 * resume the gpu - the _STOP_TRIGGER must explicitly be set to 0
	 * (_DISABLE) as well.

	* Advice from the arch group:  Disable the stop trigger first, as a
	* separate operation, in order to ensure that the trigger has taken
	* effect, before enabling the run trigger.
	*/

	offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);

	/*De-assert stop trigger */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_control0_r() + offset);
	dbgr_control0 = set_field(dbgr_control0,
			gr_gpcs_tpcs_sm_dbgr_control0_stop_trigger_m(),
			gr_gpcs_tpcs_sm_dbgr_control0_stop_trigger_disable_f());
	gk20a_writel(g,
		gr_gpc0_tpc0_sm_dbgr_control0_r() + offset, dbgr_control0);

	/* Run trigger */
	dbgr_control0 |= gr_gpcs_tpcs_sm_dbgr_control0_run_trigger_task_f();
	gk20a_writel(g,
		gr_gpc0_tpc0_sm_dbgr_control0_r() + offset, dbgr_control0);
}

void gk20a_gr_resume_all_sms(struct gk20a *g)
{
	u32 dbgr_control0;
	/*
	 * The following requires some clarification. Despite the fact that both
	 * RUN_TRIGGER and STOP_TRIGGER have the word "TRIGGER" in their
	 *  names, only one is actually a trigger, and that is the STOP_TRIGGER.
	 * Merely writing a 1(_TASK) to the RUN_TRIGGER is not sufficient to
	 * resume the gpu - the _STOP_TRIGGER must explicitly be set to 0
	 * (_DISABLE) as well.

	* Advice from the arch group:  Disable the stop trigger first, as a
	* separate operation, in order to ensure that the trigger has taken
	* effect, before enabling the run trigger.
	*/

	/*De-assert stop trigger */
	dbgr_control0 =
		gk20a_readl(g, gr_gpcs_tpcs_sm_dbgr_control0_r());
	dbgr_control0 &= ~gr_gpcs_tpcs_sm_dbgr_control0_stop_trigger_enable_f();
	gk20a_writel(g,
		gr_gpcs_tpcs_sm_dbgr_control0_r(), dbgr_control0);

	/* Run trigger */
	dbgr_control0 |= gr_gpcs_tpcs_sm_dbgr_control0_run_trigger_task_f();
	gk20a_writel(g,
		gr_gpcs_tpcs_sm_dbgr_control0_r(), dbgr_control0);
}

int gr_gk20a_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable)
{
	struct nvgpu_dbg_reg_op *ops;
	unsigned int i = 0, sm_id;
	int err;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);

	ops = nvgpu_kcalloc(g, g->gr.no_of_sm, sizeof(*ops));
	if (ops == NULL) {
		return -ENOMEM;
	}
	for (sm_id = 0; sm_id < g->gr.no_of_sm; sm_id++) {
		int gpc, tpc;
		u32 tpc_offset, gpc_offset, reg_offset, reg_mask, reg_val;

		if ((sms & BIT64(sm_id)) == 0ULL) {
			continue;
		}

		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		tpc = g->gr.sm_to_cluster[sm_id].tpc_index;

		tpc_offset = tpc_in_gpc_stride * tpc;
		gpc_offset = gpc_stride * gpc;
		reg_offset = tpc_offset + gpc_offset;

		ops[i].op = REGOP(WRITE_32);
		ops[i].type = REGOP(TYPE_GR_CTX);
		ops[i].offset  = gr_gpc0_tpc0_sm_dbgr_control0_r() + reg_offset;

		reg_mask = 0;
		reg_val = 0;
		if (enable) {
			reg_mask |= gr_gpc0_tpc0_sm_dbgr_control0_debugger_mode_m();
			reg_val |= gr_gpc0_tpc0_sm_dbgr_control0_debugger_mode_on_f();
			reg_mask |= gr_gpc0_tpc0_sm_dbgr_control0_stop_on_any_warp_m();
			reg_val |= gr_gpc0_tpc0_sm_dbgr_control0_stop_on_any_warp_disable_f();
			reg_mask |= gr_gpc0_tpc0_sm_dbgr_control0_stop_on_any_sm_m();
			reg_val |= gr_gpc0_tpc0_sm_dbgr_control0_stop_on_any_sm_disable_f();
		} else {
			reg_mask |= gr_gpc0_tpc0_sm_dbgr_control0_debugger_mode_m();
			reg_val |= gr_gpc0_tpc0_sm_dbgr_control0_debugger_mode_off_f();
		}

		ops[i].and_n_mask_lo = reg_mask;
		ops[i].value_lo = reg_val;
		i++;
	}

	err = gr_gk20a_exec_ctx_ops(ch, ops, i, i, 0, NULL);
	if (err != 0) {
		nvgpu_err(g, "Failed to access register");
	}
	nvgpu_kfree(g, ops);
	return err;
}

/*
 * gr_gk20a_suspend_context()
 * This API should be called with dbg_session lock held
 * and ctxsw disabled
 * Returns bool value indicating if context was resident
 * or not
 */
bool gr_gk20a_suspend_context(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	bool ctx_resident = false;

	if (gk20a_is_channel_ctx_resident(ch)) {
		g->ops.gr.suspend_all_sms(g, 0, false);
		ctx_resident = true;
	} else {
		gk20a_disable_channel_tsg(g, ch);
	}

	return ctx_resident;
}

bool gr_gk20a_resume_context(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	bool ctx_resident = false;

	if (gk20a_is_channel_ctx_resident(ch)) {
		g->ops.gr.resume_all_sms(g);
		ctx_resident = true;
	} else {
		gk20a_enable_channel_tsg(g, ch);
	}

	return ctx_resident;
}

int gr_gk20a_suspend_contexts(struct gk20a *g,
			      struct dbg_session_gk20a *dbg_s,
			      int *ctx_resident_ch_fd)
{
	int local_ctx_resident_ch_fd = -1;
	bool ctx_resident;
	struct channel_gk20a *ch;
	struct dbg_session_channel_data *ch_data;
	int err = 0;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	err = gr_gk20a_disable_ctxsw(g);
	if (err != 0) {
		nvgpu_err(g, "unable to stop gr ctxsw");
		goto clean_up;
	}

	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);

	nvgpu_list_for_each_entry(ch_data, &dbg_s->ch_list,
			dbg_session_channel_data, ch_entry) {
		ch = g->fifo.channel + ch_data->chid;

		ctx_resident = gr_gk20a_suspend_context(ch);
		if (ctx_resident) {
			local_ctx_resident_ch_fd = ch_data->channel_fd;
		}
	}

	nvgpu_mutex_release(&dbg_s->ch_list_lock);

	err = gr_gk20a_enable_ctxsw(g);
	if (err != 0) {
		nvgpu_err(g, "unable to restart ctxsw!");
	}

	*ctx_resident_ch_fd = local_ctx_resident_ch_fd;

clean_up:
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return err;
}

int gr_gk20a_resume_contexts(struct gk20a *g,
			      struct dbg_session_gk20a *dbg_s,
			      int *ctx_resident_ch_fd)
{
	int local_ctx_resident_ch_fd = -1;
	bool ctx_resident;
	struct channel_gk20a *ch;
	int err = 0;
	struct dbg_session_channel_data *ch_data;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	err = gr_gk20a_disable_ctxsw(g);
	if (err != 0) {
		nvgpu_err(g, "unable to stop gr ctxsw");
		goto clean_up;
	}

	nvgpu_list_for_each_entry(ch_data, &dbg_s->ch_list,
			dbg_session_channel_data, ch_entry) {
		ch = g->fifo.channel + ch_data->chid;

		ctx_resident = gr_gk20a_resume_context(ch);
		if (ctx_resident) {
			local_ctx_resident_ch_fd = ch_data->channel_fd;
		}
	}

	err = gr_gk20a_enable_ctxsw(g);
	if (err != 0) {
		nvgpu_err(g, "unable to restart ctxsw!");
	}

	*ctx_resident_ch_fd = local_ctx_resident_ch_fd;

clean_up:
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return err;
}

int gr_gk20a_trigger_suspend(struct gk20a *g)
{
	int err = 0;
	u32 dbgr_control0;

	/* assert stop trigger. uniformity assumption: all SMs will have
	 * the same state in dbg_control0. */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_control0_r());
	dbgr_control0 |= gr_gpcs_tpcs_sm_dbgr_control0_stop_trigger_enable_f();

	/* broadcast write */
	gk20a_writel(g,
		gr_gpcs_tpcs_sm_dbgr_control0_r(), dbgr_control0);

	return err;
}

int gr_gk20a_wait_for_pause(struct gk20a *g, struct nvgpu_warpstate *w_state)
{
	int err = 0;
	struct gr_gk20a *gr = &g->gr;
	u32 gpc, tpc, sm, sm_id;
	u32 global_mask;

	/* Wait for the SMs to reach full stop. This condition is:
	 * 1) All SMs with valid warps must be in the trap handler (SM_IN_TRAP_MODE)
	 * 2) All SMs in the trap handler must have equivalent VALID and PAUSED warp
	 *    masks.
	*/
	global_mask = g->ops.gr.get_sm_no_lock_down_hww_global_esr_mask(g);

	/* Lock down all SMs */
	for (sm_id = 0; sm_id < gr->no_of_sm; sm_id++) {

		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		tpc = g->gr.sm_to_cluster[sm_id].tpc_index;
		sm = g->gr.sm_to_cluster[sm_id].sm_index;

		err = g->ops.gr.lock_down_sm(g, gpc, tpc, sm,
				global_mask, false);
		if (err != 0) {
			nvgpu_err(g, "sm did not lock down!");
			return err;
		}
	}

	/* Read the warp status */
	g->ops.gr.bpt_reg_info(g, w_state);

	return 0;
}

int gr_gk20a_resume_from_pause(struct gk20a *g)
{
	int err = 0;
	u32 reg_val;

	/* Clear the pause mask to tell the GPU we want to resume everyone */
	gk20a_writel(g,
		gr_gpcs_tpcs_sm_dbgr_bpt_pause_mask_r(), 0);

	/* explicitly re-enable forwarding of SM interrupts upon any resume */
	reg_val = gk20a_readl(g, gr_gpc0_tpc0_tpccs_tpc_exception_en_r());
	reg_val |= gr_gpc0_tpc0_tpccs_tpc_exception_en_sm_enabled_f();
	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(), reg_val);

	/* Now resume all sms, write a 0 to the stop trigger
	 * then a 1 to the run trigger */
	g->ops.gr.resume_all_sms(g);

	return err;
}

int gr_gk20a_clear_sm_errors(struct gk20a *g)
{
	int ret = 0;
	u32 gpc, tpc, sm;
	struct gr_gk20a *gr = &g->gr;
	u32 global_esr;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);

	for (gpc = 0; gpc < gr->gpc_count; gpc++) {

		/* check if any tpc has an exception */
		for (tpc = 0; tpc < gr->gpc_tpc_count[gpc]; tpc++) {

			for (sm = 0; sm < sm_per_tpc; sm++) {
				global_esr = g->ops.gr.get_sm_hww_global_esr(g,
							 gpc, tpc, sm);

				/* clearing hwws, also causes tpc and gpc
				 * exceptions to be cleared
				 */
				g->ops.gr.clear_sm_hww(g,
					gpc, tpc, sm, global_esr);
			}
		}
	}

	return ret;
}

u32 gr_gk20a_tpc_enabled_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 sm_id, tpc_exception_en = 0;
	u32 offset, regval, tpc_offset, gpc_offset;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);

	for (sm_id = 0; sm_id < gr->no_of_sm; sm_id++) {

		tpc_offset = tpc_in_gpc_stride * g->gr.sm_to_cluster[sm_id].tpc_index;
		gpc_offset = gpc_stride * g->gr.sm_to_cluster[sm_id].gpc_index;
		offset = tpc_offset + gpc_offset;

		regval = gk20a_readl(g,	gr_gpc0_tpc0_tpccs_tpc_exception_en_r() +
								offset);
		/* Each bit represents corresponding enablement state, bit 0 corrsponds to SM0 */
		tpc_exception_en |= gr_gpc0_tpc0_tpccs_tpc_exception_en_sm_v(regval) << sm_id;
	}

	return tpc_exception_en;
}

u32 gk20a_gr_get_sm_hww_warp_esr(struct gk20a *g, u32 gpc, u32 tpc, u32 sm)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);
	u32 hww_warp_esr = gk20a_readl(g,
			 gr_gpc0_tpc0_sm_hww_warp_esr_r() + offset);
	return hww_warp_esr;
}

u32 gk20a_gr_get_sm_hww_global_esr(struct gk20a *g, u32 gpc, u32 tpc, u32 sm)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);

	u32 hww_global_esr = gk20a_readl(g,
				 gr_gpc0_tpc0_sm_hww_global_esr_r() + offset);

	return hww_global_esr;
}

u32 gk20a_gr_get_sm_no_lock_down_hww_global_esr_mask(struct gk20a *g)
{
	/*
	 * These three interrupts don't require locking down the SM. They can
	 * be handled by usermode clients as they aren't fatal. Additionally,
	 * usermode clients may wish to allow some warps to execute while others
	 * are at breakpoints, as opposed to fatal errors where all warps should
	 * halt.
	 */
	u32 global_esr_mask =
		gr_gpc0_tpc0_sm_hww_global_esr_bpt_int_pending_f() |
		gr_gpc0_tpc0_sm_hww_global_esr_bpt_pause_pending_f() |
		gr_gpc0_tpc0_sm_hww_global_esr_single_step_complete_pending_f();

	return global_esr_mask;
}

/* invalidate channel lookup tlb */
void gk20a_gr_flush_channel_tlb(struct gr_gk20a *gr)
{
	nvgpu_spinlock_acquire(&gr->ch_tlb_lock);
	memset(gr->chid_tlb, 0,
		sizeof(struct gr_channel_map_tlb_entry) *
		GR_CHANNEL_MAP_TLB_SIZE);
	nvgpu_spinlock_release(&gr->ch_tlb_lock);
}
