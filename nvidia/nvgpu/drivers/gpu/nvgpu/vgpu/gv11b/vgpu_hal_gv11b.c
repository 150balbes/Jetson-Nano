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

#include "common/bus/bus_gk20a.h"
#include "common/bus/bus_gm20b.h"
#include "common/priv_ring/priv_ring_gm20b.h"
#include "common/priv_ring/priv_ring_gp10b.h"
#include "common/clock_gating/gv11b_gating_reglist.h"
#include "common/fb/fb_gm20b.h"
#include "common/fb/fb_gp10b.h"
#include "common/fb/fb_gv11b.h"
#include "common/therm/therm_gm20b.h"
#include "common/therm/therm_gp10b.h"
#include "common/therm/therm_gv11b.h"
#include "common/ltc/ltc_gm20b.h"
#include "common/ltc/ltc_gp10b.h"
#include "common/ltc/ltc_gv11b.h"
#include "common/fuse/fuse_gm20b.h"
#include "common/fuse/fuse_gp10b.h"

#include <gk20a/gk20a.h>
#include <gv11b/hal_gv11b.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/error_notifier.h>

#include "vgpu/fifo_vgpu.h"
#include "vgpu/gr_vgpu.h"
#include "vgpu/ltc_vgpu.h"
#include "vgpu/mm_vgpu.h"
#include "vgpu/dbg_vgpu.h"
#include "vgpu/fecs_trace_vgpu.h"
#include "vgpu/css_vgpu.h"
#include "vgpu/fb_vgpu.h"
#include "vgpu/gm20b/vgpu_gr_gm20b.h"
#include "vgpu/gp10b/vgpu_mm_gp10b.h"
#include "vgpu/gp10b/vgpu_gr_gp10b.h"

#include <gk20a/flcn_gk20a.h>
#include "gk20a/dbg_gpu_gk20a.h"
#include <gk20a/pmu_gk20a.h>

#include <gm20b/gr_gm20b.h>
#include <gm20b/fifo_gm20b.h>
#include <gm20b/pmu_gm20b.h>
#include <gm20b/mm_gm20b.h>
#include <gm20b/acr_gm20b.h>

#include <gp10b/pmu_gp10b.h>
#include <gp10b/mm_gp10b.h>
#include <gp10b/ce_gp10b.h>
#include "gp10b/gr_gp10b.h"
#include <gp10b/fifo_gp10b.h>
#include "gp10b/clk_arb_gp10b.h"

#include <gp106/pmu_gp106.h>
#include <gp106/acr_gp106.h>

#include <gv11b/pmu_gv11b.h>
#include <gv11b/acr_gv11b.h>
#include <gv11b/mm_gv11b.h>
#include <gv11b/ce_gv11b.h>
#include <gv11b/fifo_gv11b.h>
#include <gv11b/regops_gv11b.h>
#include <gv11b/gr_ctx_gv11b.h>
#include <gv11b/gr_gv11b.h>

#include <gv100/gr_gv100.h>

#include <nvgpu/enabled.h>
#include <nvgpu/channel.h>

#include "vgpu_gv11b.h"
#include "vgpu_gr_gv11b.h"
#include "vgpu_fifo_gv11b.h"
#include "vgpu_subctx_gv11b.h"
#include "vgpu_tsg_gv11b.h"

#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_pwr_gv11b.h>

static const struct gpu_ops vgpu_gv11b_ops = {
	.ltc = {
		.determine_L2_size_bytes = vgpu_determine_L2_size_bytes,
		.set_zbc_s_entry = NULL,
		.set_zbc_color_entry = NULL,
		.set_zbc_depth_entry = NULL,
		.init_cbc = NULL,
		.init_fs_state = vgpu_ltc_init_fs_state,
		.init_comptags = vgpu_ltc_init_comptags,
		.cbc_ctrl = NULL,
		.isr = NULL,
		.flush = NULL,
		.set_enabled = NULL,
		.pri_is_ltc_addr = gm20b_ltc_pri_is_ltc_addr,
		.is_ltcs_ltss_addr = gm20b_ltc_is_ltcs_ltss_addr,
		.is_ltcn_ltss_addr = gm20b_ltc_is_ltcn_ltss_addr,
		.split_lts_broadcast_addr = gm20b_ltc_split_lts_broadcast_addr,
		.split_ltc_broadcast_addr = gm20b_ltc_split_ltc_broadcast_addr,
	},
	.ce2 = {
		.isr_stall = NULL,
		.isr_nonstall = NULL,
		.get_num_pce = vgpu_ce_get_num_pce,
	},
	.gr = {
		.init_gpc_mmu = NULL,
		.bundle_cb_defaults = gr_gv11b_bundle_cb_defaults,
		.cb_size_default = gr_gv11b_cb_size_default,
		.calc_global_ctx_buffer_size =
			gr_gv11b_calc_global_ctx_buffer_size,
		.commit_global_attrib_cb = gr_gv11b_commit_global_attrib_cb,
		.commit_global_bundle_cb = gr_gp10b_commit_global_bundle_cb,
		.commit_global_cb_manager = gr_gp10b_commit_global_cb_manager,
		.commit_global_pagepool = gr_gp10b_commit_global_pagepool,
		.handle_sw_method = NULL,
		.set_alpha_circular_buffer_size = NULL,
		.set_circular_buffer_size = NULL,
		.enable_hww_exceptions = NULL,
		.is_valid_class = gr_gv11b_is_valid_class,
		.is_valid_gfx_class = gr_gv11b_is_valid_gfx_class,
		.is_valid_compute_class = gr_gv11b_is_valid_compute_class,
		.get_sm_dsm_perf_regs = gv11b_gr_get_sm_dsm_perf_regs,
		.get_sm_dsm_perf_ctrl_regs = gv11b_gr_get_sm_dsm_perf_ctrl_regs,
		.init_fs_state = vgpu_gr_init_fs_state,
		.set_hww_esr_report_mask = NULL,
		.falcon_load_ucode = NULL,
		.load_ctxsw_ucode = NULL,
		.set_gpc_tpc_mask = NULL,
		.get_gpc_tpc_mask = vgpu_gr_get_gpc_tpc_mask,
		.alloc_obj_ctx = vgpu_gr_alloc_obj_ctx,
		.bind_ctxsw_zcull = vgpu_gr_bind_ctxsw_zcull,
		.get_zcull_info = vgpu_gr_get_zcull_info,
		.is_tpc_addr = gr_gm20b_is_tpc_addr,
		.get_tpc_num = gr_gm20b_get_tpc_num,
		.detect_sm_arch = vgpu_gr_detect_sm_arch,
		.add_zbc_color = NULL,
		.add_zbc_depth = NULL,
		.zbc_set_table = vgpu_gr_add_zbc,
		.zbc_query_table = vgpu_gr_query_zbc,
		.pmu_save_zbc = NULL,
		.add_zbc = NULL,
		.pagepool_default_size = gr_gv11b_pagepool_default_size,
		.init_ctx_state = vgpu_gr_gp10b_init_ctx_state,
		.alloc_gr_ctx = vgpu_gr_gp10b_alloc_gr_ctx,
		.free_gr_ctx = vgpu_gr_free_gr_ctx,
		.update_ctxsw_preemption_mode =
			gr_gv11b_update_ctxsw_preemption_mode,
		.dump_gr_regs = NULL,
		.update_pc_sampling = vgpu_gr_update_pc_sampling,
		.get_fbp_en_mask = vgpu_gr_get_fbp_en_mask,
		.get_max_ltc_per_fbp = vgpu_gr_get_max_ltc_per_fbp,
		.get_max_lts_per_ltc = vgpu_gr_get_max_lts_per_ltc,
		.get_rop_l2_en_mask = vgpu_gr_rop_l2_en_mask,
		.get_max_fbps_count = vgpu_gr_get_max_fbps_count,
		.init_sm_dsm_reg_info = gv11b_gr_init_sm_dsm_reg_info,
		.wait_empty = NULL,
		.init_cyclestats = vgpu_gr_gm20b_init_cyclestats,
		.set_sm_debug_mode = vgpu_gr_set_sm_debug_mode,
		.enable_cde_in_fecs = gr_gm20b_enable_cde_in_fecs,
		.bpt_reg_info = NULL,
		.get_access_map = gr_gv11b_get_access_map,
		.handle_fecs_error = NULL,
		.handle_sm_exception = NULL,
		.handle_tex_exception = NULL,
		.enable_gpc_exceptions = NULL,
		.enable_exceptions = NULL,
		.get_lrf_tex_ltc_dram_override = NULL,
		.update_smpc_ctxsw_mode = vgpu_gr_update_smpc_ctxsw_mode,
		.get_hw_accessor_stream_out_mode =
			gr_gv100_get_hw_accessor_stream_out_mode,
		.update_hwpm_ctxsw_mode = vgpu_gr_update_hwpm_ctxsw_mode,
		.set_mmu_debug_mode = vgpu_gr_set_mmu_debug_mode,
		.record_sm_error_state = gv11b_gr_record_sm_error_state,
		.clear_sm_error_state = vgpu_gr_clear_sm_error_state,
		.suspend_contexts = vgpu_gr_suspend_contexts,
		.resume_contexts = vgpu_gr_resume_contexts,
		.get_preemption_mode_flags = gr_gp10b_get_preemption_mode_flags,
		.init_sm_id_table = vgpu_gr_init_sm_id_table,
		.load_smid_config = NULL,
		.program_sm_id_numbering = NULL,
		.setup_rop_mapping = NULL,
		.program_zcull_mapping = NULL,
		.commit_global_timeslice = NULL,
		.commit_inst = vgpu_gr_gv11b_commit_inst,
		.write_zcull_ptr = gr_gv11b_write_zcull_ptr,
		.write_pm_ptr = gr_gv11b_write_pm_ptr,
		.load_tpc_mask = NULL,
		.trigger_suspend = NULL,
		.wait_for_pause = gr_gk20a_wait_for_pause,
		.resume_from_pause = NULL,
		.clear_sm_errors = gr_gk20a_clear_sm_errors,
		.tpc_enabled_exceptions = NULL,
		.get_esr_sm_sel = gv11b_gr_get_esr_sm_sel,
		.sm_debugger_attached = NULL,
		.suspend_single_sm = NULL,
		.suspend_all_sms = NULL,
		.resume_single_sm = NULL,
		.resume_all_sms = NULL,
		.get_sm_hww_warp_esr = NULL,
		.get_sm_hww_global_esr = NULL,
		.get_sm_no_lock_down_hww_global_esr_mask =
			gv11b_gr_get_sm_no_lock_down_hww_global_esr_mask,
		.lock_down_sm = NULL,
		.wait_for_sm_lock_down = NULL,
		.clear_sm_hww = NULL,
		.init_ovr_sm_dsm_perf =  gv11b_gr_init_ovr_sm_dsm_perf,
		.get_ovr_perf_regs = gv11b_gr_get_ovr_perf_regs,
		.disable_rd_coalesce = NULL,
		.set_boosted_ctx = NULL,
		.set_preemption_mode = vgpu_gr_gp10b_set_preemption_mode,
		.set_czf_bypass = NULL,
		.pre_process_sm_exception = NULL,
		.set_preemption_buffer_va = gr_gv11b_set_preemption_buffer_va,
		.init_preemption_state = NULL,
		.update_boosted_ctx = NULL,
		.set_bes_crop_debug3 = NULL,
		.set_bes_crop_debug4 = NULL,
		.set_ctxsw_preemption_mode = vgpu_gr_gp10b_set_ctxsw_preemption_mode,
		.is_etpc_addr = gv11b_gr_pri_is_etpc_addr,
		.egpc_etpc_priv_addr_table = gv11b_gr_egpc_etpc_priv_addr_table,
		.handle_tpc_mpc_exception = gr_gv11b_handle_tpc_mpc_exception,
		.zbc_s_query_table = gr_gv11b_zbc_s_query_table,
		.load_zbc_s_default_tbl = gr_gv11b_load_stencil_default_tbl,
		.handle_gpc_gpcmmu_exception =
			gr_gv11b_handle_gpc_gpcmmu_exception,
		.add_zbc_type_s = gr_gv11b_add_zbc_type_s,
		.get_egpc_base = gv11b_gr_get_egpc_base,
		.get_egpc_etpc_num = gv11b_gr_get_egpc_etpc_num,
		.handle_gpc_gpccs_exception =
			gr_gv11b_handle_gpc_gpccs_exception,
		.load_zbc_s_tbl = gr_gv11b_load_stencil_tbl,
		.access_smpc_reg = gv11b_gr_access_smpc_reg,
		.is_egpc_addr = gv11b_gr_pri_is_egpc_addr,
		.add_zbc_s = gr_gv11b_add_zbc_stencil,
		.handle_gcc_exception = gr_gv11b_handle_gcc_exception,
		.init_sw_veid_bundle = gr_gv11b_init_sw_veid_bundle,
		.handle_tpc_sm_ecc_exception =
			gr_gv11b_handle_tpc_sm_ecc_exception,
		.decode_egpc_addr = gv11b_gr_decode_egpc_addr,
		.init_ctxsw_hdr_data = gr_gp10b_init_ctxsw_hdr_data,
		.init_gfxp_wfi_timeout_count =
			gr_gv11b_init_gfxp_wfi_timeout_count,
		.get_max_gfxp_wfi_timeout_count =
			gr_gv11b_get_max_gfxp_wfi_timeout_count,
		.add_ctxsw_reg_pm_fbpa = gr_gk20a_add_ctxsw_reg_pm_fbpa,
		.add_ctxsw_reg_perf_pma = gr_gk20a_add_ctxsw_reg_perf_pma,
		.decode_priv_addr = gr_gv11b_decode_priv_addr,
		.create_priv_addr_table = gr_gv11b_create_priv_addr_table,
		.get_pmm_per_chiplet_offset =
			gr_gv11b_get_pmm_per_chiplet_offset,
		.split_fbpa_broadcast_addr = gr_gk20a_split_fbpa_broadcast_addr,
		.alloc_global_ctx_buffers = gr_gk20a_alloc_global_ctx_buffers,
		.map_global_ctx_buffers = gr_gk20a_map_global_ctx_buffers,
		.commit_global_ctx_buffers = gr_gk20a_commit_global_ctx_buffers,
		.get_offset_in_gpccs_segment =
			gr_gk20a_get_offset_in_gpccs_segment,
		.set_debug_mode = gm20b_gr_set_debug_mode,
	},
	.fb = {
		.init_hw = NULL,
		.init_fs_state = NULL,
		.init_cbc = NULL,
		.set_mmu_page_size = NULL,
		.set_use_full_comp_tag_line = NULL,
		.compression_page_size = gp10b_fb_compression_page_size,
		.compressible_page_size = gp10b_fb_compressible_page_size,
		.compression_align_mask = gm20b_fb_compression_align_mask,
		.vpr_info_fetch = NULL,
		.dump_vpr_info = NULL,
		.dump_wpr_info = NULL,
		.read_wpr_info = NULL,
		.is_debug_mode_enabled = NULL,
		.set_debug_mode = vgpu_mm_mmu_set_debug_mode,
		.set_mmu_debug_mode = vgpu_fb_set_mmu_debug_mode,
		.tlb_invalidate = vgpu_mm_tlb_invalidate,
		.hub_isr = gv11b_fb_hub_isr,
		.enable_hub_intr = gv11b_fb_enable_hub_intr,
		.disable_hub_intr = gv11b_fb_disable_hub_intr,
		.write_mmu_fault_buffer_lo_hi =
				fb_gv11b_write_mmu_fault_buffer_lo_hi,
		.write_mmu_fault_buffer_get =
				fb_gv11b_write_mmu_fault_buffer_get,
		.write_mmu_fault_buffer_size =
				fb_gv11b_write_mmu_fault_buffer_size,
		.write_mmu_fault_status = fb_gv11b_write_mmu_fault_status,
		.read_mmu_fault_buffer_get =
				fb_gv11b_read_mmu_fault_buffer_get,
		.read_mmu_fault_buffer_put =
				fb_gv11b_read_mmu_fault_buffer_put,
		.read_mmu_fault_buffer_size =
				fb_gv11b_read_mmu_fault_buffer_size,
		.read_mmu_fault_addr_lo_hi = fb_gv11b_read_mmu_fault_addr_lo_hi,
		.read_mmu_fault_inst_lo_hi = fb_gv11b_read_mmu_fault_inst_lo_hi,
		.read_mmu_fault_info = fb_gv11b_read_mmu_fault_info,
		.read_mmu_fault_status = fb_gv11b_read_mmu_fault_status,
	},
	.clock_gating = {
		.slcg_bus_load_gating_prod = NULL,
		.slcg_ce2_load_gating_prod = NULL,
		.slcg_chiplet_load_gating_prod = NULL,
		.slcg_ctxsw_firmware_load_gating_prod = NULL,
		.slcg_fb_load_gating_prod = NULL,
		.slcg_fifo_load_gating_prod = NULL,
		.slcg_gr_load_gating_prod = NULL,
		.slcg_ltc_load_gating_prod = NULL,
		.slcg_perf_load_gating_prod = NULL,
		.slcg_priring_load_gating_prod = NULL,
		.slcg_pmu_load_gating_prod = NULL,
		.slcg_therm_load_gating_prod = NULL,
		.slcg_xbar_load_gating_prod = NULL,
		.blcg_bus_load_gating_prod = NULL,
		.blcg_ce_load_gating_prod = NULL,
		.blcg_ctxsw_firmware_load_gating_prod = NULL,
		.blcg_fb_load_gating_prod = NULL,
		.blcg_fifo_load_gating_prod = NULL,
		.blcg_gr_load_gating_prod = NULL,
		.blcg_ltc_load_gating_prod = NULL,
		.blcg_pwr_csb_load_gating_prod = NULL,
		.blcg_pmu_load_gating_prod = NULL,
		.blcg_xbar_load_gating_prod = NULL,
		.pg_gr_load_gating_prod = NULL,
	},
	.fifo = {
		.init_fifo_setup_hw = vgpu_gv11b_init_fifo_setup_hw,
		.bind_channel = vgpu_channel_bind,
		.unbind_channel = vgpu_channel_unbind,
		.disable_channel = vgpu_channel_disable,
		.enable_channel = vgpu_channel_enable,
		.alloc_inst = vgpu_channel_alloc_inst,
		.free_inst = vgpu_channel_free_inst,
		.setup_ramfc = vgpu_channel_setup_ramfc,
		.default_timeslice_us = vgpu_fifo_default_timeslice_us,
		.setup_userd = gk20a_fifo_setup_userd,
		.userd_gp_get = gv11b_userd_gp_get,
		.userd_gp_put = gv11b_userd_gp_put,
		.userd_pb_get = gv11b_userd_pb_get,
		.pbdma_acquire_val = gk20a_fifo_pbdma_acquire_val,
		.preempt_channel = vgpu_fifo_preempt_channel,
		.preempt_tsg = vgpu_fifo_preempt_tsg,
		.enable_tsg = vgpu_gv11b_enable_tsg,
		.disable_tsg = gk20a_disable_tsg,
		.tsg_verify_channel_status = NULL,
		.tsg_verify_status_ctx_reload = NULL,
		/* TODO: implement it for CE fault */
		.tsg_verify_status_faulted = NULL,
		.update_runlist = vgpu_fifo_update_runlist,
		.trigger_mmu_fault = NULL,
		.get_mmu_fault_info = NULL,
		.get_mmu_fault_desc = NULL,
		.get_mmu_fault_client_desc = NULL,
		.get_mmu_fault_gpc_desc = NULL,
		.wait_engine_idle = vgpu_fifo_wait_engine_idle,
		.get_num_fifos = gv11b_fifo_get_num_fifos,
		.get_pbdma_signature = gp10b_fifo_get_pbdma_signature,
		.set_runlist_interleave = vgpu_fifo_set_runlist_interleave,
		.tsg_set_timeslice = vgpu_tsg_set_timeslice,
		.tsg_open = vgpu_tsg_open,
		.tsg_release = vgpu_tsg_release,
		.force_reset_ch = vgpu_fifo_force_reset_ch,
		.engine_enum_from_type = gp10b_fifo_engine_enum_from_type,
		.device_info_data_parse = gp10b_device_info_data_parse,
		.eng_runlist_base_size = fifo_eng_runlist_base__size_1_v,
		.init_engine_info = vgpu_fifo_init_engine_info,
		.runlist_entry_size = ram_rl_entry_size_v,
		.get_tsg_runlist_entry = gv11b_get_tsg_runlist_entry,
		.get_ch_runlist_entry = gv11b_get_ch_runlist_entry,
		.is_fault_engine_subid_gpc = gv11b_is_fault_engine_subid_gpc,
		.dump_pbdma_status = NULL,
		.dump_eng_status = NULL,
		.dump_channel_status_ramfc = NULL,
		.intr_0_error_mask = gv11b_fifo_intr_0_error_mask,
		.is_preempt_pending = gv11b_fifo_is_preempt_pending,
		.init_pbdma_intr_descs = gv11b_fifo_init_pbdma_intr_descs,
		.reset_enable_hw = NULL,
		.teardown_ch_tsg = NULL,
		.handle_sched_error = NULL,
		.handle_pbdma_intr_0 = NULL,
		.handle_pbdma_intr_1 = gv11b_fifo_handle_pbdma_intr_1,
		.init_eng_method_buffers = gv11b_fifo_init_eng_method_buffers,
		.deinit_eng_method_buffers =
			gv11b_fifo_deinit_eng_method_buffers,
		.tsg_bind_channel = vgpu_gv11b_tsg_bind_channel,
		.tsg_unbind_channel = vgpu_tsg_unbind_channel,
		.post_event_id = gk20a_tsg_event_id_post_event,
		.ch_abort_clean_up = gk20a_channel_abort_clean_up,
		.check_tsg_ctxsw_timeout = gk20a_fifo_check_tsg_ctxsw_timeout,
		.check_ch_ctxsw_timeout = gk20a_fifo_check_ch_ctxsw_timeout,
		.channel_suspend = gk20a_channel_suspend,
		.channel_resume = gk20a_channel_resume,
		.set_error_notifier = nvgpu_set_error_notifier,
		.setup_sw = gk20a_init_fifo_setup_sw,
#ifdef CONFIG_TEGRA_GK20A_NVHOST
		.alloc_syncpt_buf = vgpu_gv11b_fifo_alloc_syncpt_buf,
		.free_syncpt_buf = vgpu_gv11b_fifo_free_syncpt_buf,
		.add_syncpt_wait_cmd = gv11b_fifo_add_syncpt_wait_cmd,
		.get_syncpt_wait_cmd_size = gv11b_fifo_get_syncpt_wait_cmd_size,
		.get_syncpt_incr_per_release =
                                gv11b_fifo_get_syncpt_incr_per_release,
		.add_syncpt_incr_cmd = gv11b_fifo_add_syncpt_incr_cmd,
		.get_syncpt_incr_cmd_size = gv11b_fifo_get_syncpt_incr_cmd_size,
		.get_sync_ro_map = vgpu_gv11b_fifo_get_sync_ro_map,
#endif
		.resetup_ramfc = NULL,
		.reschedule_runlist = NULL,
		.device_info_fault_id = top_device_info_data_fault_id_enum_v,
		.free_channel_ctx_header = vgpu_gv11b_free_subctx_header,
		.handle_ctxsw_timeout = gv11b_fifo_handle_ctxsw_timeout,
		.runlist_hw_submit = NULL,
		.runlist_wait_pending = NULL,
		.ring_channel_doorbell = gv11b_ring_channel_doorbell,
		.get_sema_wait_cmd_size = gv11b_fifo_get_sema_wait_cmd_size,
		.get_sema_incr_cmd_size = gv11b_fifo_get_sema_incr_cmd_size,
		.add_sema_cmd = gv11b_fifo_add_sema_cmd,
	},
	.gr_ctx = {
		.get_netlist_name = gr_gv11b_get_netlist_name,
		.is_fw_defined = gr_gv11b_is_firmware_defined,
	},
#ifdef CONFIG_GK20A_CTXSW_TRACE
	.fecs_trace = {
		.alloc_user_buffer = vgpu_alloc_user_buffer,
		.free_user_buffer = vgpu_free_user_buffer,
		.mmap_user_buffer = vgpu_mmap_user_buffer,
		.init = vgpu_fecs_trace_init,
		.deinit = vgpu_fecs_trace_deinit,
		.enable = vgpu_fecs_trace_enable,
		.disable = vgpu_fecs_trace_disable,
		.is_enabled = vgpu_fecs_trace_is_enabled,
		.reset = NULL,
		.flush = NULL,
		.poll = vgpu_fecs_trace_poll,
		.bind_channel = NULL,
		.unbind_channel = NULL,
		.max_entries = vgpu_fecs_trace_max_entries,
		.set_filter = vgpu_fecs_trace_set_filter,
	},
#endif /* CONFIG_GK20A_CTXSW_TRACE */
	.mm = {
		/* FIXME: add support for sparse mappings */
		.support_sparse = NULL,
		.gmmu_map = vgpu_gp10b_locked_gmmu_map,
		.gmmu_unmap = vgpu_locked_gmmu_unmap,
		.vm_bind_channel = vgpu_vm_bind_channel,
		.fb_flush = vgpu_mm_fb_flush,
		.l2_invalidate = vgpu_mm_l2_invalidate,
		.l2_flush = vgpu_mm_l2_flush,
		.cbc_clean = NULL,
		.set_big_page_size = gm20b_mm_set_big_page_size,
		.get_big_page_sizes = gm20b_mm_get_big_page_sizes,
		.get_default_big_page_size = gp10b_mm_get_default_big_page_size,
		.gpu_phys_addr = gm20b_gpu_phys_addr,
		.get_iommu_bit = gk20a_mm_get_iommu_bit,
		.get_mmu_levels = gp10b_mm_get_mmu_levels,
		.init_pdb = gp10b_mm_init_pdb,
		.init_mm_setup_hw = vgpu_gp10b_init_mm_setup_hw,
		.is_bar1_supported = gv11b_mm_is_bar1_supported,
		.init_inst_block = gv11b_init_inst_block,
		.mmu_fault_pending = NULL,
		.get_kind_invalid = gm20b_get_kind_invalid,
		.get_kind_pitch = gm20b_get_kind_pitch,
		.init_bar2_vm = gp10b_init_bar2_vm,
		.remove_bar2_vm = gp10b_remove_bar2_vm,
		.fault_info_mem_destroy = gv11b_mm_fault_info_mem_destroy,
	},
	.therm = {
		.init_therm_setup_hw = NULL,
		.init_elcg_mode = NULL,
		.init_blcg_mode = NULL,
		.elcg_init_idle_filters = NULL,
	},
	.pmu = {
		.pmu_setup_elpg = NULL,
		.pmu_get_queue_head = NULL,
		.pmu_get_queue_head_size = NULL,
		.pmu_get_queue_tail = NULL,
		.pmu_get_queue_tail_size = NULL,
		.pmu_queue_head = NULL,
		.pmu_queue_tail = NULL,
		.pmu_msgq_tail = NULL,
		.pmu_mutex_size = NULL,
		.pmu_mutex_acquire = NULL,
		.pmu_mutex_release = NULL,
		.pmu_is_interrupted = NULL,
		.pmu_isr = NULL,
		.pmu_init_perfmon_counter = NULL,
		.pmu_pg_idle_counter_config = NULL,
		.pmu_read_idle_counter = NULL,
		.pmu_reset_idle_counter = NULL,
		.pmu_read_idle_intr_status = NULL,
		.pmu_clear_idle_intr_status = NULL,
		.pmu_dump_elpg_stats = NULL,
		.pmu_dump_falcon_stats = NULL,
		.pmu_enable_irq = NULL,
		.write_dmatrfbase = NULL,
		.pmu_elpg_statistics = NULL,
		.pmu_init_perfmon = NULL,
		.pmu_perfmon_start_sampling = NULL,
		.pmu_perfmon_stop_sampling = NULL,
		.pmu_perfmon_get_samples_rpc = NULL,
		.pmu_pg_init_param = NULL,
		.pmu_pg_supported_engines_list = NULL,
		.pmu_pg_engines_feature_list = NULL,
		.dump_secure_fuses = NULL,
		.reset_engine = NULL,
		.is_engine_in_reset = NULL,
		.pmu_nsbootstrap = NULL,
		.pmu_pg_set_sub_feature_mask = NULL,
		.is_pmu_supported = NULL,
	},
	.clk_arb = {
		.get_arbiter_clk_domains = gp10b_get_arbiter_clk_domains,
		.get_arbiter_f_points = gp10b_get_arbiter_f_points,
		.get_arbiter_clk_range = gp10b_get_arbiter_clk_range,
		.get_arbiter_clk_default = gp10b_get_arbiter_clk_default,
		.arbiter_clk_init = gp10b_init_clk_arbiter,
		.clk_arb_run_arbiter_cb = gp10b_clk_arb_run_arbiter_cb,
		.clk_arb_cleanup = gp10b_clk_arb_cleanup,
	},
	.regops = {
		.exec_regops = vgpu_exec_regops,
		.get_global_whitelist_ranges =
			gv11b_get_global_whitelist_ranges,
		.get_global_whitelist_ranges_count =
			gv11b_get_global_whitelist_ranges_count,
		.get_context_whitelist_ranges =
			gv11b_get_context_whitelist_ranges,
		.get_context_whitelist_ranges_count =
			gv11b_get_context_whitelist_ranges_count,
		.get_runcontrol_whitelist = gv11b_get_runcontrol_whitelist,
		.get_runcontrol_whitelist_count =
			gv11b_get_runcontrol_whitelist_count,
		.get_qctl_whitelist = gv11b_get_qctl_whitelist,
		.get_qctl_whitelist_count = gv11b_get_qctl_whitelist_count,
	},
	.mc = {
		.intr_mask = NULL,
		.intr_enable = NULL,
		.intr_unit_config = NULL,
		.isr_stall = NULL,
		.intr_stall = NULL,
		.intr_stall_pause = NULL,
		.intr_stall_resume = NULL,
		.intr_nonstall = NULL,
		.intr_nonstall_pause = NULL,
		.intr_nonstall_resume = NULL,
		.isr_nonstall = NULL,
		.enable = NULL,
		.disable = NULL,
		.reset = NULL,
		.is_intr1_pending = NULL,
		.is_intr_hub_pending = NULL,
		.log_pending_intrs = NULL	,
		.reset_mask = NULL,
		.is_enabled = NULL,
		.fb_reset = NULL,
	},
	.debug = {
		.show_dump = NULL,
	},
	.debugger = {
		.post_events = gk20a_dbg_gpu_post_events,
	},
	.dbg_session_ops = {
		.dbg_set_powergate = vgpu_dbg_set_powergate,
		.check_and_set_global_reservation =
			vgpu_check_and_set_global_reservation,
		.check_and_set_context_reservation =
			vgpu_check_and_set_context_reservation,
		.release_profiler_reservation =
			vgpu_release_profiler_reservation,
		.perfbuffer_enable = vgpu_perfbuffer_enable,
		.perfbuffer_disable = vgpu_perfbuffer_disable,
	},
	.bus = {
		.init_hw = NULL,
		.isr = NULL,
		.bar1_bind = NULL,
		.bar2_bind = NULL,
		.set_bar0_window = NULL,
	},
	.ptimer = {
		.isr = NULL,
		.read_ptimer = vgpu_read_ptimer,
		.get_timestamps_zipper = vgpu_get_timestamps_zipper,
	},
#if defined(CONFIG_GK20A_CYCLE_STATS)
	.css = {
		.enable_snapshot = vgpu_css_enable_snapshot_buffer,
		.disable_snapshot = vgpu_css_release_snapshot_buffer,
		.check_data_available = vgpu_css_flush_snapshots,
		.detach_snapshot = vgpu_css_detach,
		.set_handled_snapshots = NULL,
		.allocate_perfmon_ids = NULL,
		.release_perfmon_ids = NULL,
	},
#endif
	.falcon = {
		.falcon_hal_sw_init = gk20a_falcon_hal_sw_init,
	},
	.priv_ring = {
		.enable_priv_ring = NULL,
		.isr = NULL,
		.set_ppriv_timeout_settings = NULL,
		.enum_ltc = NULL,
	},
	.fuse = {
		.is_opt_ecc_enable = NULL,
		.is_opt_feature_override_disable = NULL,
		.fuse_status_opt_fbio = NULL,
		.fuse_status_opt_fbp = NULL,
		.fuse_status_opt_rop_l2_fbp = NULL,
		.fuse_status_opt_tpc_gpc = NULL,
		.fuse_ctrl_opt_tpc_gpc = NULL,
		.fuse_opt_sec_debug_en = NULL,
		.fuse_opt_priv_sec_en = NULL,
		.read_vin_cal_fuse_rev = NULL,
		.read_vin_cal_slope_intercept_fuse = NULL,
		.read_vin_cal_gain_offset_fuse = NULL,
	},
	.chip_init_gpu_characteristics = vgpu_gv11b_init_gpu_characteristics,
	.get_litter_value = gv11b_get_litter_value,
};

int vgpu_gv11b_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gops->ltc = vgpu_gv11b_ops.ltc;
	gops->ce2 = vgpu_gv11b_ops.ce2;
	gops->gr = vgpu_gv11b_ops.gr;
	gops->fb = vgpu_gv11b_ops.fb;
	gops->clock_gating = vgpu_gv11b_ops.clock_gating;
	gops->fifo = vgpu_gv11b_ops.fifo;
	gops->gr_ctx = vgpu_gv11b_ops.gr_ctx;
	gops->mm = vgpu_gv11b_ops.mm;
#ifdef CONFIG_GK20A_CTXSW_TRACE
	gops->fecs_trace = vgpu_gv11b_ops.fecs_trace;
#endif
	gops->therm = vgpu_gv11b_ops.therm;
	gops->pmu = vgpu_gv11b_ops.pmu;
	gops->clk_arb = vgpu_gv11b_ops.clk_arb;
	gops->regops = vgpu_gv11b_ops.regops;
	gops->mc = vgpu_gv11b_ops.mc;
	gops->debug = vgpu_gv11b_ops.debug;
	gops->debugger = vgpu_gv11b_ops.debugger;
	gops->dbg_session_ops = vgpu_gv11b_ops.dbg_session_ops;
	gops->bus = vgpu_gv11b_ops.bus;
	gops->ptimer = vgpu_gv11b_ops.ptimer;
#if defined(CONFIG_GK20A_CYCLE_STATS)
	gops->css = vgpu_gv11b_ops.css;
#endif
	gops->falcon = vgpu_gv11b_ops.falcon;
	gops->priv_ring = vgpu_gv11b_ops.priv_ring;
	gops->fuse = vgpu_gv11b_ops.fuse;

	/* Lone functions */
	gops->chip_init_gpu_characteristics =
		vgpu_gv11b_ops.chip_init_gpu_characteristics;
	gops->get_litter_value = vgpu_gv11b_ops.get_litter_value;
	gops->semaphore_wakeup = gk20a_channel_semaphore_wakeup;

	if (priv->constants.can_set_clkrate) {
		gops->clk.support_clk_freq_controller = true;
	}

	g->name = "gv11b";

	return 0;
}
