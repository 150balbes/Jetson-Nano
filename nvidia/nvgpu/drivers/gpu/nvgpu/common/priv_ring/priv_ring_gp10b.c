/*
 * GP10B priv ring
 *
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

#include <nvgpu/log.h>
#include <nvgpu/timers.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/hw/gp10b/hw_pri_ringmaster_gp10b.h>
#include <nvgpu/hw/gp10b/hw_pri_ringstation_sys_gp10b.h>
#include <nvgpu/hw/gp10b/hw_pri_ringstation_gpc_gp10b.h>

#include "priv_ring_gp10b.h"

static const char *const error_type_badf1xyy[] = {
	"client timeout",
	"decode error",
	"client in reset",
	"client floorswept",
	"client stuck ack",
	"client expected ack",
	"fence error",
	"subid error",
	"byte access unsupported",
};

static const char *const error_type_badf2xyy[] = {
	"orphan gpc/fbp"
};

static const char *const error_type_badf3xyy[] = {
	"priv ring dead"
};

static const char *const error_type_badf5xyy[] = {
	"client error",
	"priv level violation",
	"indirect priv level violation",
	"local local ring error",
	"falcon mem access priv level violation",
	"pri route error"
};

void gp10b_priv_ring_decode_error_code(struct gk20a *g,
			u32 error_code)
{
	u32 error_type_index;

	error_type_index = (error_code & 0x00000f00) >> 16;
	error_code = error_code & 0xBADFf000;

	if (error_code == 0xBADF1000) {
		if (error_type_index <
				ARRAY_SIZE(error_type_badf1xyy)) {
			nvgpu_err(g, "%s",
				error_type_badf1xyy[error_type_index]);
		}
	} else if (error_code == 0xBADF2000) {
		if (error_type_index <
				ARRAY_SIZE(error_type_badf2xyy)) {
			nvgpu_err(g, "%s",
				error_type_badf2xyy[error_type_index]);
		}
	} else if (error_code == 0xBADF3000) {
		if (error_type_index <
				ARRAY_SIZE(error_type_badf3xyy)) {
			nvgpu_err(g, "%s",
				error_type_badf3xyy[error_type_index]);
		}
	} else if (error_code == 0xBADF5000) {
		if (error_type_index <
				ARRAY_SIZE(error_type_badf5xyy)) {
			nvgpu_err(g, "%s",
				error_type_badf5xyy[error_type_index]);
		}
	}
}

void gp10b_priv_ring_isr(struct gk20a *g)
{
	u32 status0, status1;
	u32 cmd;
	s32 retry = 100;
	u32 gpc;
	u32 gpc_stride, offset;
	u32 error_info;
	u32 error_code;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		nvgpu_info(g, "unhandled priv ring intr");
		return;
	}

	status0 = gk20a_readl(g, pri_ringmaster_intr_status0_r());
	status1 = gk20a_readl(g, pri_ringmaster_intr_status1_r());

	nvgpu_err(g, "ringmaster intr status0: 0x%08x,"
		"status1: 0x%08x", status0, status1);

	if (pri_ringmaster_intr_status0_ring_start_conn_fault_v(status0) != 0) {
		nvgpu_err(g,
			"BUG: connectivity problem on the startup sequence");
	}

	if (pri_ringmaster_intr_status0_disconnect_fault_v(status0) != 0) {
		nvgpu_err(g, "ring disconnected");
	}

	if (pri_ringmaster_intr_status0_overflow_fault_v(status0) != 0) {
		nvgpu_err(g, "ring overflowed");
	}

	if (pri_ringmaster_intr_status0_gbl_write_error_sys_v(status0) != 0) {
		error_info =
			gk20a_readl(g, pri_ringstation_sys_priv_error_info_r());
		error_code =
			gk20a_readl(g, pri_ringstation_sys_priv_error_code_r());
		nvgpu_err(g, "SYS write error. ADR 0x%08x WRDAT 0x%08x "
				"INFO 0x%08x (subid 0x%08x priv level %d), "
				"CODE 0x%08x",
			gk20a_readl(g, pri_ringstation_sys_priv_error_adr_r()),
			gk20a_readl(g, pri_ringstation_sys_priv_error_wrdat_r()),
			error_info,
			pri_ringstation_sys_priv_error_info_subid_v(error_info),
			pri_ringstation_sys_priv_error_info_priv_level_v(error_info),
			error_code);
		if (g->ops.priv_ring.decode_error_code) {
			g->ops.priv_ring.decode_error_code(g, error_code);
		}
	}

	if (status1) {
		gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_PRIV_STRIDE);
		for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
			offset = gpc * gpc_stride;
			if (status1 & BIT(gpc)) {
				error_info = gk20a_readl(g,
					pri_ringstation_gpc_gpc0_priv_error_info_r() + offset);
				error_code = gk20a_readl(g,
					pri_ringstation_gpc_gpc0_priv_error_code_r() + offset);
				nvgpu_err(g, "GPC%u write error. ADR 0x%08x "
					"WRDAT 0x%08x "
					"INFO 0x%08x (subid 0x%08x priv level %d), "
					"CODE 0x%08x", gpc,
					gk20a_readl(g,
					pri_ringstation_gpc_gpc0_priv_error_adr_r() + offset),
					gk20a_readl(g,
					pri_ringstation_gpc_gpc0_priv_error_wrdat_r() + offset),
					error_info,
					pri_ringstation_gpc_gpc0_priv_error_info_subid_v(error_info),
					pri_ringstation_gpc_gpc0_priv_error_info_priv_level_v(error_info),
					error_code);

				if (g->ops.priv_ring.decode_error_code) {
					g->ops.priv_ring.decode_error_code(g,
								error_code);
				}

				status1 = status1 & (~(BIT(gpc)));
				if (status1 == 0U) {
					break;
				}
			}
		}
	}
	/* clear interrupt */
	cmd = gk20a_readl(g, pri_ringmaster_command_r());
	cmd = set_field(cmd, pri_ringmaster_command_cmd_m(),
		pri_ringmaster_command_cmd_ack_interrupt_f());
	gk20a_writel(g, pri_ringmaster_command_r(), cmd);

	/* poll for clear interrupt done */
	cmd = pri_ringmaster_command_cmd_v(
		gk20a_readl(g, pri_ringmaster_command_r()));
	while ((cmd != pri_ringmaster_command_cmd_no_cmd_v()) && (retry != 0)) {
		nvgpu_udelay(20);
		cmd = pri_ringmaster_command_cmd_v(
			gk20a_readl(g, pri_ringmaster_command_r()));
		retry--;
	}

	if (retry == 0) {
		nvgpu_err(g, "priv ringmaster intr ack failed");
	}
}
