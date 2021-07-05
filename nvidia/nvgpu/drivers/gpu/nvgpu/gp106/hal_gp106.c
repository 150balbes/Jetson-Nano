/*
 * GP106 HAL interface
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include "common/bus/bus_gk20a.h"
#include "common/clock_gating/gp106_gating_reglist.h"
#include "common/ptimer/ptimer_gk20a.h"
#include "common/bus/bus_gm20b.h"
#include "common/bus/bus_gp10b.h"
#include "common/priv_ring/priv_ring_gm20b.h"
#include "common/priv_ring/priv_ring_gp10b.h"
#include "common/fb/fb_gp10b.h"
#include "common/fb/fb_gm20b.h"
#include "common/fb/fb_gp106.h"
#include "common/xve/xve_gp106.h"
#include "common/therm/therm_gm20b.h"
#include "common/therm/therm_gp106.h"
#include "common/ltc/ltc_gm20b.h"
#include "common/ltc/ltc_gp10b.h"
#include "common/fuse/fuse_gm20b.h"
#include "common/fuse/fuse_gp10b.h"
#include "common/fuse/fuse_gp106.h"
#include "common/mc/mc_gm20b.h"
#include "common/mc/mc_gp10b.h"

#include "gk20a/fifo_gk20a.h"
#include "gk20a/fecs_trace_gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "gk20a/pmu_gk20a.h"
#include "gk20a/gr_gk20a.h"

#include "gp10b/gr_gp10b.h"
#include "gp10b/mm_gp10b.h"
#include "gp10b/ce_gp10b.h"
#include "gp10b/regops_gp10b.h"
#include "gp10b/fifo_gp10b.h"
#include "gp10b/pmu_gp10b.h"
#include "gp10b/gr_gp10b.h"

#include "gp106/fifo_gp106.h"
#include "gp106/regops_gp106.h"

#include "gm20b/gr_gm20b.h"
#include "gm20b/fifo_gm20b.h"
#include "gm20b/mm_gm20b.h"
#include "gm20b/pmu_gm20b.h"
#include "gm20b/acr_gm20b.h"
#include "gm20b/gr_gm20b.h"
#include "gm20b/fecs_trace_gm20b.h"

#include "gp106/acr_gp106.h"
#include "gp106/sec2_gp106.h"
#include "gp106/clk_gp106.h"
#include "gp106/clk_arb_gp106.h"
#include "gp106/mclk_gp106.h"
#include "gp106/bios_gp106.h"
#include "gp106/fifo_gp106.h"
#include "gp106/clk_gp106.h"
#include "gp106/pmu_gp106.h"
#include "gp106/gr_ctx_gp106.h"
#include "gp106/gr_gp106.h"
#include "gp106/flcn_gp106.h"

#include "hal_gp106.h"

#include <nvgpu/debug.h>
#include <nvgpu/bug.h>
#include <nvgpu/ptimer.h>
#include <nvgpu/enabled.h>
#include <nvgpu/ctxsw_trace.h>
#include <nvgpu/error_notifier.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>

#include <nvgpu/hw/gp106/hw_proj_gp106.h>
#include <nvgpu/hw/gp106/hw_fifo_gp106.h>
#include <nvgpu/hw/gp106/hw_ram_gp106.h>
#include <nvgpu/hw/gp106/hw_top_gp106.h>
#include <nvgpu/hw/gp106/hw_pram_gp106.h>
#include <nvgpu/hw/gp106/hw_pwr_gp106.h>
#include <nvgpu/hw/gp106/hw_gr_gp106.h>


static u32 gp106_get_litter_value(struct gk20a *g, int value)
{
	u32 ret = 0;

	switch (value) {
	case GPU_LIT_NUM_GPCS:
		ret = proj_scal_litter_num_gpcs_v();
		break;
	case GPU_LIT_NUM_PES_PER_GPC:
		ret = proj_scal_litter_num_pes_per_gpc_v();
		break;
	case GPU_LIT_NUM_SM_PER_TPC:
		ret = proj_scal_litter_num_sm_per_tpc_v();
		break;
	case GPU_LIT_NUM_ZCULL_BANKS:
		ret = proj_scal_litter_num_zcull_banks_v();
		break;
	case GPU_LIT_NUM_TPC_PER_GPC:
		ret = proj_scal_litter_num_tpc_per_gpc_v();
		break;
	case GPU_LIT_NUM_FBPS:
		ret = proj_scal_litter_num_fbps_v();
		break;
	case GPU_LIT_GPC_BASE:
		ret = proj_gpc_base_v();
		break;
	case GPU_LIT_GPC_STRIDE:
		ret = proj_gpc_stride_v();
		break;
	case GPU_LIT_GPC_SHARED_BASE:
		ret = proj_gpc_shared_base_v();
		break;
	case GPU_LIT_TPC_IN_GPC_BASE:
		ret = proj_tpc_in_gpc_base_v();
		break;
	case GPU_LIT_TPC_IN_GPC_STRIDE:
		ret = proj_tpc_in_gpc_stride_v();
		break;
	case GPU_LIT_TPC_IN_GPC_SHARED_BASE:
		ret = proj_tpc_in_gpc_shared_base_v();
		break;
	case GPU_LIT_PPC_IN_GPC_BASE:
		ret = proj_ppc_in_gpc_base_v();
		break;
	case GPU_LIT_PPC_IN_GPC_STRIDE:
		ret = proj_ppc_in_gpc_stride_v();
		break;
	case GPU_LIT_PPC_IN_GPC_SHARED_BASE:
		ret = proj_ppc_in_gpc_shared_base_v();
		break;
	case GPU_LIT_ROP_BASE:
		ret = proj_rop_base_v();
		break;
	case GPU_LIT_ROP_STRIDE:
		ret = proj_rop_stride_v();
		break;
	case GPU_LIT_ROP_SHARED_BASE:
		ret = proj_rop_shared_base_v();
		break;
	case GPU_LIT_HOST_NUM_ENGINES:
		ret = proj_host_num_engines_v();
		break;
	case GPU_LIT_HOST_NUM_PBDMA:
		ret = proj_host_num_pbdma_v();
		break;
	case GPU_LIT_LTC_STRIDE:
		ret = proj_ltc_stride_v();
		break;
	case GPU_LIT_LTS_STRIDE:
		ret = proj_lts_stride_v();
		break;
	case GPU_LIT_NUM_FBPAS:
		ret = proj_scal_litter_num_fbpas_v();
		break;
	case GPU_LIT_FBPA_SHARED_BASE:
		ret = proj_fbpa_shared_base_v();
		break;
	case GPU_LIT_FBPA_BASE:
		ret = proj_fbpa_base_v();
		break;
	case GPU_LIT_FBPA_STRIDE:
		ret = proj_fbpa_stride_v();
		break;
	case GPU_LIT_TWOD_CLASS:
		ret = FERMI_TWOD_A;
		break;
	case GPU_LIT_THREED_CLASS:
		ret = PASCAL_B;
		break;
	case GPU_LIT_COMPUTE_CLASS:
		ret = PASCAL_COMPUTE_B;
		break;
	case GPU_LIT_GPFIFO_CLASS:
		ret = PASCAL_CHANNEL_GPFIFO_A;
		break;
	case GPU_LIT_I2M_CLASS:
		ret = KEPLER_INLINE_TO_MEMORY_B;
		break;
	case GPU_LIT_DMA_COPY_CLASS:
		ret = PASCAL_DMA_COPY_A;
		break;
	case GPU_LIT_GPC_PRIV_STRIDE:
		ret = proj_gpc_priv_stride_v();
		break;
	default:
		nvgpu_err(g, "Missing definition %d", value);
		BUG();
		break;
	}

	return ret;
}

static int gp106_init_gpu_characteristics(struct gk20a *g)
{
	int err;

	err = gk20a_init_gpu_characteristics(g);
	if (err) {
		return err;
	}

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_GET_VOLTAGE, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_GET_CURRENT, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_GET_POWER, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_GET_TEMPERATURE, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_DEVICE_EVENTS, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_SET_THERM_ALERT_LIMIT, true);

	return 0;
}

static const struct gpu_ops gp106_ops = {
	.bios = {
		.init = gp106_bios_init,
		.preos_wait_for_halt = gp106_bios_preos_wait_for_halt,
		.devinit = gp106_bios_devinit,
		.preos = gp106_bios_preos,
		.verify_devinit = NULL,
	},
	.ltc = {
		.determine_L2_size_bytes = gp10b_determine_L2_size_bytes,
		.set_zbc_color_entry = gm20b_ltc_set_zbc_color_entry,
		.set_zbc_depth_entry = gm20b_ltc_set_zbc_depth_entry,
		.init_cbc = NULL,
		.init_fs_state = gm20b_ltc_init_fs_state,
		.init_comptags = gp10b_ltc_init_comptags,
		.cbc_ctrl = gp10b_ltc_cbc_ctrl,
		.isr = gp10b_ltc_isr,
		.cbc_fix_config = NULL,
		.flush = gm20b_flush_ltc,
		.set_enabled = gp10b_ltc_set_enabled,
		.pri_is_ltc_addr = gm20b_ltc_pri_is_ltc_addr,
		.is_ltcs_ltss_addr = gm20b_ltc_is_ltcs_ltss_addr,
		.is_ltcn_ltss_addr = gm20b_ltc_is_ltcn_ltss_addr,
		.split_lts_broadcast_addr = gm20b_ltc_split_lts_broadcast_addr,
		.split_ltc_broadcast_addr = gm20b_ltc_split_ltc_broadcast_addr,
	},
	.ce2 = {
		.isr_stall = gp10b_ce_isr,
		.isr_nonstall = gp10b_ce_nonstall_isr,
	},
	.gr = {
		.get_patch_slots = gr_gk20a_get_patch_slots,
		.init_gpc_mmu = gr_gm20b_init_gpc_mmu,
		.bundle_cb_defaults = gr_gm20b_bundle_cb_defaults,
		.cb_size_default = gr_gp106_cb_size_default,
		.calc_global_ctx_buffer_size =
			gr_gp10b_calc_global_ctx_buffer_size,
		.commit_global_attrib_cb = gr_gp10b_commit_global_attrib_cb,
		.commit_global_bundle_cb = gr_gp10b_commit_global_bundle_cb,
		.commit_global_cb_manager = gr_gp10b_commit_global_cb_manager,
		.commit_global_pagepool = gr_gp10b_commit_global_pagepool,
		.handle_sw_method = gr_gp106_handle_sw_method,
		.set_alpha_circular_buffer_size =
			gr_gp10b_set_alpha_circular_buffer_size,
		.set_circular_buffer_size = gr_gp10b_set_circular_buffer_size,
		.enable_hww_exceptions = gr_gk20a_enable_hww_exceptions,
		.is_valid_class = gr_gp106_is_valid_class,
		.is_valid_gfx_class = gr_gp10b_is_valid_gfx_class,
		.is_valid_compute_class = gr_gp10b_is_valid_compute_class,
		.get_sm_dsm_perf_regs = gr_gm20b_get_sm_dsm_perf_regs,
		.get_sm_dsm_perf_ctrl_regs = gr_gm20b_get_sm_dsm_perf_ctrl_regs,
		.init_fs_state = gr_gp10b_init_fs_state,
		.set_hww_esr_report_mask = gr_gm20b_set_hww_esr_report_mask,
		.falcon_load_ucode = gr_gm20b_load_ctxsw_ucode_segments,
		.set_gpc_tpc_mask = gr_gp10b_set_gpc_tpc_mask,
		.get_gpc_tpc_mask = gr_gm20b_get_gpc_tpc_mask,
		.get_gpc_mask = gr_gm20b_get_gpc_mask,
		.alloc_obj_ctx = gk20a_alloc_obj_ctx,
		.bind_ctxsw_zcull = gr_gk20a_bind_ctxsw_zcull,
		.get_zcull_info = gr_gk20a_get_zcull_info,
		.is_tpc_addr = gr_gm20b_is_tpc_addr,
		.get_tpc_num = gr_gm20b_get_tpc_num,
		.detect_sm_arch = gr_gm20b_detect_sm_arch,
		.add_zbc_color = gr_gp10b_add_zbc_color,
		.add_zbc_depth = gr_gp10b_add_zbc_depth,
		.get_gpcs_swdx_dss_zbc_c_format_reg =
			gr_gp10b_get_gpcs_swdx_dss_zbc_c_format_reg,
		.get_gpcs_swdx_dss_zbc_z_format_reg =
			gr_gp10b_get_gpcs_swdx_dss_zbc_z_format_reg,
		.zbc_set_table = gk20a_gr_zbc_set_table,
		.zbc_query_table = gr_gk20a_query_zbc,
		.pmu_save_zbc = gk20a_pmu_save_zbc,
		.add_zbc = gr_gk20a_add_zbc,
		.pagepool_default_size = gr_gp106_pagepool_default_size,
		.init_ctx_state = gr_gp10b_init_ctx_state,
		.alloc_gr_ctx = gr_gp10b_alloc_gr_ctx,
		.free_gr_ctx = gr_gk20a_free_gr_ctx,
		.update_ctxsw_preemption_mode =
			gr_gp10b_update_ctxsw_preemption_mode,
		.dump_gr_regs = gr_gp10b_dump_gr_status_regs,
		.update_pc_sampling = gr_gm20b_update_pc_sampling,
		.get_fbp_en_mask = gr_gm20b_get_fbp_en_mask,
		.get_max_ltc_per_fbp = gr_gm20b_get_max_ltc_per_fbp,
		.get_max_lts_per_ltc = gr_gm20b_get_max_lts_per_ltc,
		.get_rop_l2_en_mask = gr_gm20b_rop_l2_en_mask,
		.get_max_fbps_count = gr_gm20b_get_max_fbps_count,
		.init_sm_dsm_reg_info = gr_gm20b_init_sm_dsm_reg_info,
		.wait_empty = gr_gp10b_wait_empty,
		.init_cyclestats = gr_gm20b_init_cyclestats,
		.set_sm_debug_mode = gr_gk20a_set_sm_debug_mode,
		.enable_cde_in_fecs = gr_gm20b_enable_cde_in_fecs,
		.bpt_reg_info = gr_gm20b_bpt_reg_info,
		.get_access_map = gr_gp10b_get_access_map,
		.handle_fecs_error = gr_gp10b_handle_fecs_error,
		.handle_sm_exception = gr_gp10b_handle_sm_exception,
		.handle_tex_exception = gr_gp10b_handle_tex_exception,
		.enable_gpc_exceptions = gk20a_gr_enable_gpc_exceptions,
		.enable_exceptions = gk20a_gr_enable_exceptions,
		.get_lrf_tex_ltc_dram_override = get_ecc_override_val,
		.update_smpc_ctxsw_mode = gr_gk20a_update_smpc_ctxsw_mode,
		.update_hwpm_ctxsw_mode = gr_gk20a_update_hwpm_ctxsw_mode,
		.record_sm_error_state = gm20b_gr_record_sm_error_state,
		.clear_sm_error_state = gm20b_gr_clear_sm_error_state,
		.suspend_contexts = gr_gp10b_suspend_contexts,
		.resume_contexts = gr_gk20a_resume_contexts,
		.get_preemption_mode_flags = gr_gp10b_get_preemption_mode_flags,
		.init_sm_id_table = gr_gk20a_init_sm_id_table,
		.load_smid_config = gr_gp10b_load_smid_config,
		.program_sm_id_numbering = gr_gm20b_program_sm_id_numbering,
		.setup_rop_mapping = gr_gk20a_setup_rop_mapping,
		.program_zcull_mapping = gr_gk20a_program_zcull_mapping,
		.commit_global_timeslice = gr_gk20a_commit_global_timeslice,
		.commit_inst = gr_gk20a_commit_inst,
		.write_zcull_ptr = gr_gk20a_write_zcull_ptr,
		.write_pm_ptr = gr_gk20a_write_pm_ptr,
		.load_tpc_mask = gr_gm20b_load_tpc_mask,
		.trigger_suspend = gr_gk20a_trigger_suspend,
		.wait_for_pause = gr_gk20a_wait_for_pause,
		.resume_from_pause = gr_gk20a_resume_from_pause,
		.clear_sm_errors = gr_gk20a_clear_sm_errors,
		.tpc_enabled_exceptions = gr_gk20a_tpc_enabled_exceptions,
		.get_esr_sm_sel = gk20a_gr_get_esr_sm_sel,
		.sm_debugger_attached = gk20a_gr_sm_debugger_attached,
		.suspend_single_sm = gk20a_gr_suspend_single_sm,
		.suspend_all_sms = gk20a_gr_suspend_all_sms,
		.resume_single_sm = gk20a_gr_resume_single_sm,
		.resume_all_sms = gk20a_gr_resume_all_sms,
		.get_sm_hww_warp_esr = gp10b_gr_get_sm_hww_warp_esr,
		.get_sm_hww_global_esr = gk20a_gr_get_sm_hww_global_esr,
		.get_sm_no_lock_down_hww_global_esr_mask =
			gk20a_gr_get_sm_no_lock_down_hww_global_esr_mask,
		.lock_down_sm = gk20a_gr_lock_down_sm,
		.wait_for_sm_lock_down = gk20a_gr_wait_for_sm_lock_down,
		.clear_sm_hww = gm20b_gr_clear_sm_hww,
		.init_ovr_sm_dsm_perf =  gk20a_gr_init_ovr_sm_dsm_perf,
		.get_ovr_perf_regs = gk20a_gr_get_ovr_perf_regs,
		.disable_rd_coalesce = gm20a_gr_disable_rd_coalesce,
		.set_boosted_ctx = NULL,
		.set_preemption_mode = gr_gp10b_set_preemption_mode,
		.set_czf_bypass = gr_gp10b_set_czf_bypass,
		.init_czf_bypass = gr_gp10b_init_czf_bypass,
		.pre_process_sm_exception = gr_gp10b_pre_process_sm_exception,
		.set_preemption_buffer_va = gr_gp10b_set_preemption_buffer_va,
		.init_preemption_state = NULL,
		.update_boosted_ctx = NULL,
		.set_bes_crop_debug3 = gr_gp10b_set_bes_crop_debug3,
		.set_bes_crop_debug4 = gr_gp10b_set_bes_crop_debug4,
		.set_ctxsw_preemption_mode = gr_gp106_set_ctxsw_preemption_mode,
		.load_ctxsw_ucode = gr_gm20b_load_ctxsw_ucode,
		.fecs_host_int_enable = gr_gk20a_fecs_host_int_enable,
		.handle_notify_pending = gk20a_gr_handle_notify_pending,
		.handle_semaphore_pending = gk20a_gr_handle_semaphore_pending,
		.add_ctxsw_reg_pm_fbpa = gr_gk20a_add_ctxsw_reg_pm_fbpa,
		.add_ctxsw_reg_perf_pma = gr_gk20a_add_ctxsw_reg_perf_pma,
		.decode_priv_addr = gr_gk20a_decode_priv_addr,
		.create_priv_addr_table = gr_gk20a_create_priv_addr_table,
		.get_pmm_per_chiplet_offset =
			gr_gm20b_get_pmm_per_chiplet_offset,
		.split_fbpa_broadcast_addr = gr_gk20a_split_fbpa_broadcast_addr,
		.fecs_ctxsw_mailbox_size = gr_fecs_ctxsw_mailbox__size_1_v,
		.alloc_global_ctx_buffers = gr_gk20a_alloc_global_ctx_buffers,
		.map_global_ctx_buffers = gr_gk20a_map_global_ctx_buffers,
		.commit_global_ctx_buffers = gr_gk20a_commit_global_ctx_buffers,
		.get_offset_in_gpccs_segment =
			gr_gk20a_get_offset_in_gpccs_segment,
		.set_debug_mode = gm20b_gr_set_debug_mode,
	},
	.fb = {
		.init_hw = gm20b_fb_init_hw,
		.init_fs_state = gp106_fb_init_fs_state,
		.set_mmu_page_size = NULL,
		.set_use_full_comp_tag_line =
			gm20b_fb_set_use_full_comp_tag_line,
		.mmu_ctrl = gm20b_fb_mmu_ctrl,
		.mmu_debug_ctrl = gm20b_fb_mmu_debug_ctrl,
		.mmu_debug_wr = gm20b_fb_mmu_debug_wr,
		.mmu_debug_rd = gm20b_fb_mmu_debug_rd,
		.compression_page_size = gp10b_fb_compression_page_size,
		.compressible_page_size = gp10b_fb_compressible_page_size,
		.compression_align_mask = gm20b_fb_compression_align_mask,
		.vpr_info_fetch = NULL,
		.dump_vpr_info = NULL,
		.dump_wpr_info = gm20b_fb_dump_wpr_info,
		.read_wpr_info = gm20b_fb_read_wpr_info,
		.is_debug_mode_enabled = gm20b_fb_debug_mode_enabled,
		.set_debug_mode = gm20b_fb_set_debug_mode,
		.tlb_invalidate = gm20b_fb_tlb_invalidate,
		.mem_unlock = NULL,
		.get_vidmem_size = gp106_fb_get_vidmem_size,
	},
	.clock_gating = {
		.slcg_bus_load_gating_prod =
			gp106_slcg_bus_load_gating_prod,
		.slcg_ce2_load_gating_prod =
			gp106_slcg_ce2_load_gating_prod,
		.slcg_chiplet_load_gating_prod =
			gp106_slcg_chiplet_load_gating_prod,
		.slcg_ctxsw_firmware_load_gating_prod =
			gp106_slcg_ctxsw_firmware_load_gating_prod,
		.slcg_fb_load_gating_prod =
			gp106_slcg_fb_load_gating_prod,
		.slcg_fifo_load_gating_prod =
			gp106_slcg_fifo_load_gating_prod,
		.slcg_gr_load_gating_prod =
			gr_gp106_slcg_gr_load_gating_prod,
		.slcg_ltc_load_gating_prod =
			ltc_gp106_slcg_ltc_load_gating_prod,
		.slcg_perf_load_gating_prod =
			gp106_slcg_perf_load_gating_prod,
		.slcg_priring_load_gating_prod =
			gp106_slcg_priring_load_gating_prod,
		.slcg_pmu_load_gating_prod =
			gp106_slcg_pmu_load_gating_prod,
		.slcg_therm_load_gating_prod =
			gp106_slcg_therm_load_gating_prod,
		.slcg_xbar_load_gating_prod =
			gp106_slcg_xbar_load_gating_prod,
		.blcg_bus_load_gating_prod =
			gp106_blcg_bus_load_gating_prod,
		.blcg_ce_load_gating_prod =
			gp106_blcg_ce_load_gating_prod,
		.blcg_fb_load_gating_prod =
			gp106_blcg_fb_load_gating_prod,
		.blcg_fifo_load_gating_prod =
			gp106_blcg_fifo_load_gating_prod,
		.blcg_gr_load_gating_prod =
			gp106_blcg_gr_load_gating_prod,
		.blcg_ltc_load_gating_prod =
			gp106_blcg_ltc_load_gating_prod,
		.blcg_pmu_load_gating_prod =
			gp106_blcg_pmu_load_gating_prod,
		.blcg_xbar_load_gating_prod =
			gp106_blcg_xbar_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gp106_pg_gr_load_gating_prod,
	},
	.fifo = {
		.init_fifo_setup_hw = gk20a_init_fifo_setup_hw,
		.bind_channel = channel_gm20b_bind,
		.unbind_channel = gk20a_fifo_channel_unbind,
		.disable_channel = gk20a_fifo_disable_channel,
		.enable_channel = gk20a_fifo_enable_channel,
		.alloc_inst = gk20a_fifo_alloc_inst,
		.free_inst = gk20a_fifo_free_inst,
		.setup_ramfc = channel_gp10b_setup_ramfc,
		.default_timeslice_us = gk20a_fifo_default_timeslice_us,
		.setup_userd = gk20a_fifo_setup_userd,
		.userd_gp_get = gk20a_fifo_userd_gp_get,
		.userd_gp_put = gk20a_fifo_userd_gp_put,
		.userd_pb_get = gk20a_fifo_userd_pb_get,
		.pbdma_acquire_val = gk20a_fifo_pbdma_acquire_val,
		.preempt_channel = gk20a_fifo_preempt_channel,
		.preempt_tsg = gk20a_fifo_preempt_tsg,
		.enable_tsg = gk20a_enable_tsg,
		.disable_tsg = gk20a_disable_tsg,
		.tsg_verify_channel_status = gk20a_fifo_tsg_unbind_channel_verify_status,
		.tsg_verify_status_ctx_reload = gm20b_fifo_tsg_verify_status_ctx_reload,
		.update_runlist = gk20a_fifo_update_runlist,
		.trigger_mmu_fault = gm20b_fifo_trigger_mmu_fault,
		.get_mmu_fault_info = gp10b_fifo_get_mmu_fault_info,
		.get_mmu_fault_desc = gp10b_fifo_get_mmu_fault_desc,
		.get_mmu_fault_client_desc = gp106_fifo_get_mmu_fault_client_desc,
		.get_mmu_fault_gpc_desc = gp106_fifo_get_mmu_fault_gpc_desc,
		.wait_engine_idle = gk20a_fifo_wait_engine_idle,
		.get_num_fifos = gp106_fifo_get_num_fifos,
		.get_pbdma_signature = gp10b_fifo_get_pbdma_signature,
		.set_runlist_interleave = gk20a_fifo_set_runlist_interleave,
		.tsg_set_timeslice = gk20a_fifo_tsg_set_timeslice,
		.force_reset_ch = gk20a_fifo_force_reset_ch,
		.engine_enum_from_type = gp10b_fifo_engine_enum_from_type,
		.device_info_data_parse = gp10b_device_info_data_parse,
		.eng_runlist_base_size = fifo_eng_runlist_base__size_1_v,
		.init_engine_info = gk20a_fifo_init_engine_info,
		.runlist_entry_size = ram_rl_entry_size_v,
		.get_tsg_runlist_entry = gk20a_get_tsg_runlist_entry,
		.get_ch_runlist_entry = gk20a_get_ch_runlist_entry,
		.is_fault_engine_subid_gpc = gk20a_is_fault_engine_subid_gpc,
		.dump_pbdma_status = gk20a_dump_pbdma_status,
		.dump_eng_status = gk20a_dump_eng_status,
		.dump_channel_status_ramfc = gk20a_dump_channel_status_ramfc,
		.intr_0_error_mask = gk20a_fifo_intr_0_error_mask,
		.is_preempt_pending = gk20a_fifo_is_preempt_pending,
		.init_pbdma_intr_descs = gp10b_fifo_init_pbdma_intr_descs,
		.reset_enable_hw = gk20a_init_fifo_reset_enable_hw,
		.teardown_ch_tsg = gk20a_fifo_teardown_ch_tsg,
		.handle_sched_error = gk20a_fifo_handle_sched_error,
		.handle_pbdma_intr_0 = gk20a_fifo_handle_pbdma_intr_0,
		.handle_pbdma_intr_1 = gk20a_fifo_handle_pbdma_intr_1,
		.tsg_bind_channel = gk20a_tsg_bind_channel,
		.tsg_unbind_channel = NULL,
		.post_event_id = gk20a_tsg_event_id_post_event,
		.ch_abort_clean_up = gk20a_channel_abort_clean_up,
		.check_tsg_ctxsw_timeout = gk20a_fifo_check_tsg_ctxsw_timeout,
		.check_ch_ctxsw_timeout = gk20a_fifo_check_ch_ctxsw_timeout,
		.channel_suspend = gk20a_channel_suspend,
		.channel_resume = gk20a_channel_resume,
		.set_error_notifier = nvgpu_set_error_notifier,
		.setup_sw = gk20a_init_fifo_setup_sw,
#ifdef CONFIG_TEGRA_GK20A_NVHOST
		.alloc_syncpt_buf = gk20a_fifo_alloc_syncpt_buf,
		.free_syncpt_buf = gk20a_fifo_free_syncpt_buf,
		.add_syncpt_wait_cmd = gk20a_fifo_add_syncpt_wait_cmd,
		.get_syncpt_wait_cmd_size = gk20a_fifo_get_syncpt_wait_cmd_size,
		.get_syncpt_incr_per_release =
				gk20a_fifo_get_syncpt_incr_per_release,
		.add_syncpt_incr_cmd = gk20a_fifo_add_syncpt_incr_cmd,
		.get_syncpt_incr_cmd_size = gk20a_fifo_get_syncpt_incr_cmd_size,
		.get_sync_ro_map = NULL,
#endif
		.resetup_ramfc = gp10b_fifo_resetup_ramfc,
		.device_info_fault_id = top_device_info_data_fault_id_enum_v,
		.runlist_hw_submit = gk20a_fifo_runlist_hw_submit,
		.runlist_wait_pending = gk20a_fifo_runlist_wait_pending,
		.get_sema_wait_cmd_size = gk20a_fifo_get_sema_wait_cmd_size,
		.get_sema_incr_cmd_size = gk20a_fifo_get_sema_incr_cmd_size,
		.add_sema_cmd = gk20a_fifo_add_sema_cmd,
	},
	.gr_ctx = {
		.get_netlist_name = gr_gp106_get_netlist_name,
		.is_fw_defined = gr_gp106_is_firmware_defined,
	},
#ifdef CONFIG_GK20A_CTXSW_TRACE
	.fecs_trace = {
		.alloc_user_buffer = gk20a_ctxsw_dev_ring_alloc,
		.free_user_buffer = gk20a_ctxsw_dev_ring_free,
		.mmap_user_buffer = gk20a_ctxsw_dev_mmap_buffer,
		.init = gk20a_fecs_trace_init,
		.deinit = gk20a_fecs_trace_deinit,
		.enable = gk20a_fecs_trace_enable,
		.disable = gk20a_fecs_trace_disable,
		.is_enabled = gk20a_fecs_trace_is_enabled,
		.reset = gk20a_fecs_trace_reset,
		.flush = gm20b_fecs_trace_flush,
		.poll = gk20a_fecs_trace_poll,
		.bind_channel = gk20a_fecs_trace_bind_channel,
		.unbind_channel = gk20a_fecs_trace_unbind_channel,
		.max_entries = gk20a_gr_max_entries,
	},
#endif /* CONFIG_GK20A_CTXSW_TRACE */
	.mm = {
		.support_sparse = gm20b_mm_support_sparse,
		.gmmu_map = gk20a_locked_gmmu_map,
		.gmmu_unmap = gk20a_locked_gmmu_unmap,
		.vm_bind_channel = gk20a_vm_bind_channel,
		.fb_flush = gk20a_mm_fb_flush,
		.l2_invalidate = gk20a_mm_l2_invalidate,
		.l2_flush = gk20a_mm_l2_flush,
		.cbc_clean = gk20a_mm_cbc_clean,
		.set_big_page_size = gm20b_mm_set_big_page_size,
		.get_big_page_sizes = gm20b_mm_get_big_page_sizes,
		.get_default_big_page_size = gp10b_mm_get_default_big_page_size,
		.gpu_phys_addr = gm20b_gpu_phys_addr,
		.get_mmu_levels = gp10b_mm_get_mmu_levels,
		.init_pdb = gp10b_mm_init_pdb,
		.init_mm_setup_hw = gk20a_init_mm_setup_hw,
		.is_bar1_supported = gm20b_mm_is_bar1_supported,
		.alloc_inst_block = gk20a_alloc_inst_block,
		.init_inst_block = gk20a_init_inst_block,
		.mmu_fault_pending = gk20a_fifo_mmu_fault_pending,
		.init_bar2_vm = gp10b_init_bar2_vm,
		.remove_bar2_vm = gp10b_remove_bar2_vm,
		.get_kind_invalid = gm20b_get_kind_invalid,
		.get_kind_pitch = gm20b_get_kind_pitch,
	},
	.pramin = {
		.data032_r = pram_data032_r,
	},
	.therm = {
		.init_elcg_mode = gm20b_therm_init_elcg_mode,
		.init_blcg_mode = gm20b_therm_init_blcg_mode,
		.elcg_init_idle_filters = gp106_elcg_init_idle_filters,
		.get_internal_sensor_curr_temp =
			gp106_get_internal_sensor_curr_temp,
		.get_internal_sensor_limits = gp106_get_internal_sensor_limits,
		.configure_therm_alert = gp106_configure_therm_alert,
	},
	.pmu = {
		.init_wpr_region = gm20b_pmu_init_acr,
		.load_lsfalcon_ucode = gp106_load_falcon_ucode,
		.is_lazy_bootstrap = gp106_is_lazy_bootstrap,
		.is_priv_load = gp106_is_priv_load,
		.prepare_ucode = gp106_prepare_ucode_blob,
		.pmu_populate_loader_cfg = gp106_pmu_populate_loader_cfg,
		.flcn_populate_bl_dmem_desc = gp106_flcn_populate_bl_dmem_desc,
		.pmu_queue_tail = gk20a_pmu_queue_tail,
		.pmu_get_queue_head = pwr_pmu_queue_head_r,
		.pmu_mutex_release = gk20a_pmu_mutex_release,
		.pmu_is_interrupted = gk20a_pmu_is_interrupted,
		.pmu_isr = gk20a_pmu_isr,
		.pmu_init_perfmon_counter = gk20a_pmu_init_perfmon_counter,
		.pmu_pg_idle_counter_config = gk20a_pmu_pg_idle_counter_config,
		.pmu_read_idle_counter = gk20a_pmu_read_idle_counter,
		.pmu_reset_idle_counter = gk20a_pmu_reset_idle_counter,
		.pmu_read_idle_intr_status = gk20a_pmu_read_idle_intr_status,
		.pmu_clear_idle_intr_status = gk20a_pmu_clear_idle_intr_status,
		.pmu_dump_elpg_stats = gk20a_pmu_dump_elpg_stats,
		.pmu_dump_falcon_stats = gk20a_pmu_dump_falcon_stats,
		.pmu_enable_irq = gk20a_pmu_enable_irq,
		.is_pmu_supported = gp106_is_pmu_supported,
		.pmu_pg_supported_engines_list = gp106_pmu_pg_engines_list,
		.pmu_elpg_statistics = gp106_pmu_elpg_statistics,
		.pmu_init_perfmon = nvgpu_pmu_init_perfmon,
		.pmu_perfmon_start_sampling = nvgpu_pmu_perfmon_start_sampling,
		.pmu_perfmon_stop_sampling = nvgpu_pmu_perfmon_stop_sampling,
		.pmu_mutex_acquire = gk20a_pmu_mutex_acquire,
		.pmu_is_lpwr_feature_supported =
			gp106_pmu_is_lpwr_feature_supported,
		.pmu_msgq_tail = gk20a_pmu_msgq_tail,
		.pmu_pg_engines_feature_list = gp106_pmu_pg_feature_list,
		.pmu_get_queue_head_size = pwr_pmu_queue_head__size_1_v,
		.pmu_queue_head = gk20a_pmu_queue_head,
		.pmu_pg_param_post_init = nvgpu_lpwr_post_init,
		.pmu_get_queue_tail_size = pwr_pmu_queue_tail__size_1_v,
		.pmu_pg_init_param = gp106_pg_param_init,
		.reset_engine = gp106_pmu_engine_reset,
		.pmu_lpwr_disable_pg = nvgpu_lpwr_disable_pg,
		.write_dmatrfbase = gp10b_write_dmatrfbase,
		.pmu_mutex_size = pwr_pmu_mutex__size_1_v,
		.is_engine_in_reset = gp106_pmu_is_engine_in_reset,
		.pmu_get_queue_tail = pwr_pmu_queue_tail_r,
		.pmu_lpwr_enable_pg = nvgpu_lpwr_enable_pg,
		.get_irqdest = gk20a_pmu_get_irqdest,
		.alloc_super_surface = NULL,
		.is_debug_mode_enabled = gm20b_pmu_is_debug_mode_en,
		.update_lspmu_cmdline_args =
			gp106_update_lspmu_cmdline_args,
		.setup_apertures = gp106_pmu_setup_apertures,
		.secured_pmu_start = gm20b_secured_pmu_start,
	},
	.clk = {
		.init_clk_support = gp106_init_clk_support,
		.get_crystal_clk_hz = gp106_crystal_clk_hz,
		.get_rate_cntr = gp106_get_rate_cntr,
		.measure_freq = gp106_clk_measure_freq,
		.suspend_clk_support = gp106_suspend_clk_support,
		.clk_domain_get_f_points = gp106_clk_domain_get_f_points,
		.mclk_init = gp106_mclk_init,
		.mclk_change = gp106_mclk_change,
		.mclk_deinit = gp106_mclk_deinit,
		.perf_pmu_vfe_load = perf_pmu_vfe_load,
	},
	.clk_arb = {
		.get_arbiter_clk_domains = gp106_get_arbiter_clk_domains,
		.get_arbiter_f_points = gp106_get_arbiter_f_points,
		.get_arbiter_clk_range = gp106_get_arbiter_clk_range,
		.get_arbiter_clk_default = gp106_get_arbiter_clk_default,
		.get_current_pstate = nvgpu_clk_arb_get_current_pstate,
		.arbiter_clk_init = gp106_init_clk_arbiter,
		.clk_arb_run_arbiter_cb = gp106_clk_arb_run_arbiter_cb,
		.clk_arb_cleanup = gp106_clk_arb_cleanup,
	},
	.regops = {
		.exec_regops = exec_regops_gk20a,
		.get_global_whitelist_ranges =
			gp106_get_global_whitelist_ranges,
		.get_global_whitelist_ranges_count =
			gp106_get_global_whitelist_ranges_count,
		.get_context_whitelist_ranges =
			gp106_get_context_whitelist_ranges,
		.get_context_whitelist_ranges_count =
			gp106_get_context_whitelist_ranges_count,
		.get_runcontrol_whitelist = gp106_get_runcontrol_whitelist,
		.get_runcontrol_whitelist_count =
			gp106_get_runcontrol_whitelist_count,
		.get_qctl_whitelist = gp106_get_qctl_whitelist,
		.get_qctl_whitelist_count = gp106_get_qctl_whitelist_count,
	},
	.mc = {
		.intr_mask = mc_gp10b_intr_mask,
		.intr_enable = mc_gp10b_intr_enable,
		.intr_unit_config = mc_gp10b_intr_unit_config,
		.isr_stall = mc_gp10b_isr_stall,
		.intr_stall = mc_gp10b_intr_stall,
		.intr_stall_pause = mc_gp10b_intr_stall_pause,
		.intr_stall_resume = mc_gp10b_intr_stall_resume,
		.intr_nonstall = mc_gp10b_intr_nonstall,
		.intr_nonstall_pause = mc_gp10b_intr_nonstall_pause,
		.intr_nonstall_resume = mc_gp10b_intr_nonstall_resume,
		.isr_nonstall = gm20b_mc_isr_nonstall,
		.enable = gm20b_mc_enable,
		.disable = gm20b_mc_disable,
		.reset = gm20b_mc_reset,
		.is_intr1_pending = mc_gp10b_is_intr1_pending,
		.log_pending_intrs = mc_gp10b_log_pending_intrs,
		.reset_mask = gm20b_mc_reset_mask,
		.is_enabled = gm20b_mc_is_enabled,
		.fb_reset = NULL,
	},
	.debug = {
		.show_dump = gk20a_debug_show_dump,
	},
	.debugger = {
		.post_events = gk20a_dbg_gpu_post_events,
	},
	.dbg_session_ops = {
		.dbg_set_powergate = dbg_set_powergate,
		.check_and_set_global_reservation =
			nvgpu_check_and_set_global_reservation,
		.check_and_set_context_reservation =
			nvgpu_check_and_set_context_reservation,
		.release_profiler_reservation =
			nvgpu_release_profiler_reservation,
		.perfbuffer_enable = gk20a_perfbuf_enable_locked,
		.perfbuffer_disable = gk20a_perfbuf_disable_locked,
	},
	.bus = {
		.init_hw = gk20a_bus_init_hw,
		.isr = gk20a_bus_isr,
		.bar1_bind = gm20b_bus_bar1_bind,
		.bar2_bind = gp10b_bus_bar2_bind,
		.set_bar0_window = gk20a_bus_set_bar0_window,
	},
	.ptimer = {
		.isr = gk20a_ptimer_isr,
		.read_ptimer = gk20a_read_ptimer,
		.get_timestamps_zipper = nvgpu_get_timestamps_zipper,
	},
#if defined(CONFIG_GK20A_CYCLE_STATS)
	.css = {
		.enable_snapshot = css_hw_enable_snapshot,
		.disable_snapshot = css_hw_disable_snapshot,
		.check_data_available = css_hw_check_data_available,
		.set_handled_snapshots = css_hw_set_handled_snapshots,
		.allocate_perfmon_ids = css_gr_allocate_perfmon_ids,
		.release_perfmon_ids = css_gr_release_perfmon_ids,
		.get_overflow_status = css_hw_get_overflow_status,
		.get_pending_snapshots = css_hw_get_pending_snapshots,
	},
#endif
	.xve = {
		.get_speed        = xve_get_speed_gp106,
		.set_speed        = xve_set_speed_gp106,
		.available_speeds = xve_available_speeds_gp106,
		.xve_readl        = xve_xve_readl_gp106,
		.xve_writel       = xve_xve_writel_gp106,
		.disable_aspm     = xve_disable_aspm_gp106,
		.reset_gpu        = xve_reset_gpu_gp106,
#if defined(CONFIG_PCI_MSI)
		.rearm_msi        = xve_rearm_msi_gp106,
#endif
		.enable_shadow_rom = xve_enable_shadow_rom_gp106,
		.disable_shadow_rom = xve_disable_shadow_rom_gp106,
		.get_link_control_status = xve_get_link_control_status,
	},
	.falcon = {
		.falcon_hal_sw_init = gp106_falcon_hal_sw_init,
	},
	.priv_ring = {
		.enable_priv_ring = gm20b_priv_ring_enable,
		.isr = gp10b_priv_ring_isr,
		.decode_error_code = gp10b_priv_ring_decode_error_code,
		.set_ppriv_timeout_settings =
			gm20b_priv_set_timeout_settings,
		.enum_ltc = gm20b_priv_ring_enum_ltc,
	},
	.fuse = {
		.check_priv_security = gp106_fuse_check_priv_security,
		.is_opt_ecc_enable = gp10b_fuse_is_opt_ecc_enable,
		.is_opt_feature_override_disable =
			gp10b_fuse_is_opt_feature_override_disable,
		.fuse_status_opt_fbio = gm20b_fuse_status_opt_fbio,
		.fuse_status_opt_fbp = gm20b_fuse_status_opt_fbp,
		.fuse_status_opt_rop_l2_fbp = gm20b_fuse_status_opt_rop_l2_fbp,
		.fuse_status_opt_gpc = gm20b_fuse_status_opt_gpc,
		.fuse_status_opt_tpc_gpc = gm20b_fuse_status_opt_tpc_gpc,
		.fuse_ctrl_opt_tpc_gpc = gm20b_fuse_ctrl_opt_tpc_gpc,
		.fuse_opt_sec_debug_en = gm20b_fuse_opt_sec_debug_en,
		.fuse_opt_priv_sec_en = gm20b_fuse_opt_priv_sec_en,
		.read_vin_cal_fuse_rev = gp106_fuse_read_vin_cal_fuse_rev,
		.read_vin_cal_slope_intercept_fuse =
			gp106_fuse_read_vin_cal_slope_intercept_fuse,
		.read_vin_cal_gain_offset_fuse =
			gp106_fuse_read_vin_cal_gain_offset_fuse,
	},
	.acr = {
		.acr_sw_init = nvgpu_gp106_acr_sw_init,
	},
	.tpc = {
		.tpc_powergate = NULL,
	},
	.get_litter_value = gp106_get_litter_value,
	.chip_init_gpu_characteristics = gp106_init_gpu_characteristics,
};

int gp106_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;

	nvgpu_log_fn(g, " ");

	gops->bios = gp106_ops.bios;
	gops->ltc = gp106_ops.ltc;
	gops->ce2 = gp106_ops.ce2;
	gops->gr = gp106_ops.gr;
	gops->fb = gp106_ops.fb;
	gops->clock_gating = gp106_ops.clock_gating;
	gops->fifo = gp106_ops.fifo;
	gops->gr_ctx = gp106_ops.gr_ctx;
#ifdef CONFIG_GK20A_CTXSW_TRACE
	gops->fecs_trace = gp106_ops.fecs_trace;
#endif
	gops->mm = gp106_ops.mm;
	gops->pramin = gp106_ops.pramin;
	gops->therm = gp106_ops.therm;
	gops->pmu = gp106_ops.pmu;
	/*
	 * clk must be assigned member by member
	 * since some clk ops are assigned during probe prior to HAL init
	 */
	gops->clk.init_clk_support = gp106_ops.clk.init_clk_support;
	gops->clk.get_crystal_clk_hz = gp106_ops.clk.get_crystal_clk_hz;
	gops->clk.get_rate_cntr = gp106_ops.clk.get_rate_cntr;
	gops->clk.measure_freq = gp106_ops.clk.measure_freq;
	gops->clk.suspend_clk_support = gp106_ops.clk.suspend_clk_support;
	gops->clk.mclk_init = gp106_ops.clk.mclk_init;
	gops->clk.mclk_change = gp106_ops.clk.mclk_change;
	gops->clk.mclk_deinit = gp106_ops.clk.mclk_deinit;
	gops->clk.clk_domain_get_f_points = gp106_ops.clk.clk_domain_get_f_points;
	gops->clk.perf_pmu_vfe_load = gp106_ops.clk.perf_pmu_vfe_load;

	gops->clk_arb = gp106_ops.clk_arb;
	gops->regops = gp106_ops.regops;
	gops->mc = gp106_ops.mc;
	gops->debug = gp106_ops.debug;
	gops->debugger = gp106_ops.debugger;
	gops->dbg_session_ops = gp106_ops.dbg_session_ops;
	gops->bus = gp106_ops.bus;
	gops->ptimer = gp106_ops.ptimer;
#if defined(CONFIG_GK20A_CYCLE_STATS)
	gops->css = gp106_ops.css;
#endif
	gops->xve = gp106_ops.xve;
	gops->falcon = gp106_ops.falcon;
	gops->priv_ring = gp106_ops.priv_ring;
	gops->fuse = gp106_ops.fuse;
	gops->tpc = gp106_ops.tpc;
	gops->acr = gp106_ops.acr;

	/* Lone functions */
	gops->get_litter_value = gp106_ops.get_litter_value;
	gops->chip_init_gpu_characteristics =
		gp106_ops.chip_init_gpu_characteristics;
	gops->semaphore_wakeup = gk20a_channel_semaphore_wakeup;

	__nvgpu_set_enabled(g, NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP, true);
	__nvgpu_set_enabled(g, NVGPU_PMU_PSTATE, true);
	__nvgpu_set_enabled(g, NVGPU_PMU_FECS_BOOTSTRAP_DONE, false);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_MULTIPLE_WPR, false);
	__nvgpu_set_enabled(g, NVGPU_FECS_TRACE_VA, false);

	/* Read fuses to check if gpu needs to boot in secure/non-secure mode */
	if (gops->fuse.check_priv_security(g)) {
		/* Do not boot gpu */
		return -EINVAL;
	}

	g->pmu_lsf_pmu_wpr_init_done = 0;
	gops->clk.split_rail_support = true;
	gops->clk.support_clk_freq_controller = true;
	gops->clk.support_pmgr_domain = true;
	gops->clk.support_lpwr_pg = true;
	gops->clk.lut_num_entries = CTRL_CLK_LUT_NUM_ENTRIES_GP10x;

	g->name = "gp10x";

	nvgpu_log_fn(g, "done");

	return 0;
}
