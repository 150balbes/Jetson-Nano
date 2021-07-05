/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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
/*
 * Function naming determines intended use:
 *
 *     <x>_r(void) : Returns the offset for register <x>.
 *
 *     <x>_o(void) : Returns the offset for element <x>.
 *
 *     <x>_w(void) : Returns the word offset for word (4 byte) element <x>.
 *
 *     <x>_<y>_s(void) : Returns size of field <y> of register <x> in bits.
 *
 *     <x>_<y>_f(u32 v) : Returns a value based on 'v' which has been shifted
 *         and masked to place it at field <y> of register <x>.  This value
 *         can be |'d with others to produce a full register value for
 *         register <x>.
 *
 *     <x>_<y>_m(void) : Returns a mask for field <y> of register <x>.  This
 *         value can be ~'d and then &'d to clear the value of field <y> for
 *         register <x>.
 *
 *     <x>_<y>_<z>_f(void) : Returns the constant value <z> after being shifted
 *         to place it at field <y> of register <x>.  This value can be |'d
 *         with others to produce a full register value for <x>.
 *
 *     <x>_<y>_v(u32 r) : Returns the value of field <y> from a full register
 *         <x> value 'r' after being shifted to place its LSB at bit 0.
 *         This value is suitable for direct comparison with other unshifted
 *         values appropriate for use in field <y> of register <x>.
 *
 *     <x>_<y>_<z>_v(void) : Returns the constant value for <z> defined for
 *         field <y> of register <x>.  This value is suitable for direct
 *         comparison with unshifted values appropriate for use in field <y>
 *         of register <x>.
 */
#ifndef _hw_ctxsw_prog_gp10b_h_
#define _hw_ctxsw_prog_gp10b_h_

static inline u32 ctxsw_prog_fecs_header_v(void)
{
	return 0x00000100U;
}
static inline u32 ctxsw_prog_main_image_num_gpcs_o(void)
{
	return 0x00000008U;
}
static inline u32 ctxsw_prog_main_image_patch_count_o(void)
{
	return 0x00000010U;
}
static inline u32 ctxsw_prog_main_image_context_id_o(void)
{
	return 0x000000f0U;
}
static inline u32 ctxsw_prog_main_image_patch_adr_lo_o(void)
{
	return 0x00000014U;
}
static inline u32 ctxsw_prog_main_image_patch_adr_hi_o(void)
{
	return 0x00000018U;
}
static inline u32 ctxsw_prog_main_image_zcull_o(void)
{
	return 0x0000001cU;
}
static inline u32 ctxsw_prog_main_image_zcull_mode_no_ctxsw_v(void)
{
	return 0x00000001U;
}
static inline u32 ctxsw_prog_main_image_zcull_mode_separate_buffer_v(void)
{
	return 0x00000002U;
}
static inline u32 ctxsw_prog_main_image_zcull_ptr_o(void)
{
	return 0x00000020U;
}
static inline u32 ctxsw_prog_main_image_pm_o(void)
{
	return 0x00000028U;
}
static inline u32 ctxsw_prog_main_image_pm_mode_m(void)
{
	return 0x7U << 0U;
}
static inline u32 ctxsw_prog_main_image_pm_mode_ctxsw_f(void)
{
	return 0x1U;
}
static inline u32 ctxsw_prog_main_image_pm_mode_no_ctxsw_f(void)
{
	return 0x0U;
}
static inline u32 ctxsw_prog_main_image_pm_smpc_mode_m(void)
{
	return 0x7U << 3U;
}
static inline u32 ctxsw_prog_main_image_pm_smpc_mode_ctxsw_f(void)
{
	return 0x8U;
}
static inline u32 ctxsw_prog_main_image_pm_smpc_mode_no_ctxsw_f(void)
{
	return 0x0U;
}
static inline u32 ctxsw_prog_main_image_pm_ptr_o(void)
{
	return 0x0000002cU;
}
static inline u32 ctxsw_prog_main_image_num_save_ops_o(void)
{
	return 0x000000f4U;
}
static inline u32 ctxsw_prog_main_image_num_wfi_save_ops_o(void)
{
	return 0x000000d0U;
}
static inline u32 ctxsw_prog_main_image_num_cta_save_ops_o(void)
{
	return 0x000000d4U;
}
static inline u32 ctxsw_prog_main_image_num_gfxp_save_ops_o(void)
{
	return 0x000000d8U;
}
static inline u32 ctxsw_prog_main_image_num_cilp_save_ops_o(void)
{
	return 0x000000dcU;
}
static inline u32 ctxsw_prog_main_image_num_restore_ops_o(void)
{
	return 0x000000f8U;
}
static inline u32 ctxsw_prog_main_image_magic_value_o(void)
{
	return 0x000000fcU;
}
static inline u32 ctxsw_prog_main_image_magic_value_v_value_v(void)
{
	return 0x600dc0deU;
}
static inline u32 ctxsw_prog_local_priv_register_ctl_o(void)
{
	return 0x0000000cU;
}
static inline u32 ctxsw_prog_local_priv_register_ctl_offset_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 ctxsw_prog_local_image_ppc_info_o(void)
{
	return 0x000000f4U;
}
static inline u32 ctxsw_prog_local_image_ppc_info_num_ppcs_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 ctxsw_prog_local_image_ppc_info_ppc_mask_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 ctxsw_prog_local_image_num_tpcs_o(void)
{
	return 0x000000f8U;
}
static inline u32 ctxsw_prog_local_magic_value_o(void)
{
	return 0x000000fcU;
}
static inline u32 ctxsw_prog_local_magic_value_v_value_v(void)
{
	return 0xad0becabU;
}
static inline u32 ctxsw_prog_main_extended_buffer_ctl_o(void)
{
	return 0x000000ecU;
}
static inline u32 ctxsw_prog_main_extended_buffer_ctl_offset_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 ctxsw_prog_main_extended_buffer_ctl_size_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 ctxsw_prog_extended_buffer_segments_size_in_bytes_v(void)
{
	return 0x00000100U;
}
static inline u32 ctxsw_prog_extended_marker_size_in_bytes_v(void)
{
	return 0x00000004U;
}
static inline u32 ctxsw_prog_extended_sm_dsm_perf_counter_register_stride_v(void)
{
	return 0x00000000U;
}
static inline u32 ctxsw_prog_extended_sm_dsm_perf_counter_control_register_stride_v(void)
{
	return 0x00000002U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_config_o(void)
{
	return 0x000000a0U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_config_mode_s(void)
{
	return 2U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_config_mode_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_config_mode_m(void)
{
	return 0x3U << 0U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_config_mode_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_config_mode_allow_all_f(void)
{
	return 0x0U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_config_mode_use_map_f(void)
{
	return 0x2U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_addr_lo_o(void)
{
	return 0x000000a4U;
}
static inline u32 ctxsw_prog_main_image_priv_access_map_addr_hi_o(void)
{
	return 0x000000a8U;
}
static inline u32 ctxsw_prog_main_image_misc_options_o(void)
{
	return 0x0000003cU;
}
static inline u32 ctxsw_prog_main_image_misc_options_verif_features_m(void)
{
	return 0x1U << 3U;
}
static inline u32 ctxsw_prog_main_image_misc_options_verif_features_disabled_f(void)
{
	return 0x0U;
}
static inline u32 ctxsw_prog_main_image_pmu_options_o(void)
{
	return 0x00000070U;
}
static inline u32 ctxsw_prog_main_image_pmu_options_boost_clock_frequencies_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 ctxsw_prog_main_image_graphics_preemption_options_o(void)
{
	return 0x00000080U;
}
static inline u32 ctxsw_prog_main_image_graphics_preemption_options_control_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 ctxsw_prog_main_image_graphics_preemption_options_control_gfxp_f(void)
{
	return 0x1U;
}
static inline u32 ctxsw_prog_main_image_full_preemption_ptr_o(void)
{
	return 0x00000068U;
}
static inline u32 ctxsw_prog_main_image_compute_preemption_options_o(void)
{
	return 0x00000084U;
}
static inline u32 ctxsw_prog_main_image_compute_preemption_options_control_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 ctxsw_prog_main_image_compute_preemption_options_control_cta_f(void)
{
	return 0x1U;
}
static inline u32 ctxsw_prog_main_image_compute_preemption_options_control_cilp_f(void)
{
	return 0x2U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_control_o(void)
{
	return 0x000000acU;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_control_num_records_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_o(void)
{
	return 0x000000b0U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_v_m(void)
{
	return 0xfffffffU << 0U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_target_m(void)
{
	return 0x3U << 28U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_target_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_target_sys_mem_coherent_f(void)
{
	return 0x20000000U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_target_sys_mem_noncoherent_f(void)
{
	return 0x30000000U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_o(void)
{
	return 0x000000b4U;
}
static inline u32 ctxsw_prog_main_image_context_timestamp_buffer_ptr_v_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 ctxsw_prog_record_timestamp_record_size_in_bytes_v(void)
{
	return 0x00000080U;
}
static inline u32 ctxsw_prog_record_timestamp_record_size_in_words_v(void)
{
	return 0x00000020U;
}
static inline u32 ctxsw_prog_record_timestamp_magic_value_lo_o(void)
{
	return 0x00000000U;
}
static inline u32 ctxsw_prog_record_timestamp_magic_value_lo_v_value_v(void)
{
	return 0x00000000U;
}
static inline u32 ctxsw_prog_record_timestamp_magic_value_hi_o(void)
{
	return 0x00000004U;
}
static inline u32 ctxsw_prog_record_timestamp_magic_value_hi_v_value_v(void)
{
	return 0x600dbeefU;
}
static inline u32 ctxsw_prog_record_timestamp_context_id_o(void)
{
	return 0x00000008U;
}
static inline u32 ctxsw_prog_record_timestamp_context_ptr_o(void)
{
	return 0x0000000cU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_lo_o(void)
{
	return 0x00000018U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_o(void)
{
	return 0x0000001cU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_v_f(u32 v)
{
	return (v & 0xffffffU) << 0U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_v_v(u32 r)
{
	return (r >> 0U) & 0xffffffU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_f(u32 v)
{
	return (v & 0xffU) << 24U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_m(void)
{
	return 0xffU << 24U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_v(u32 r)
{
	return (r >> 24U) & 0xffU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_ctxsw_req_by_host_v(void)
{
	return 0x00000001U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_ctxsw_req_by_host_f(void)
{
	return 0x1000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_v(void)
{
	return 0x00000002U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_f(void)
{
	return 0x2000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_wfi_v(void)
{
	return 0x0000000aU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_wfi_f(void)
{
	return 0xa000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_gfxp_v(void)
{
	return 0x0000000bU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_gfxp_f(void)
{
	return 0xb000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_ctap_v(void)
{
	return 0x0000000cU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_ctap_f(void)
{
	return 0xc000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_cilp_v(void)
{
	return 0x0000000dU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_fe_ack_cilp_f(void)
{
	return 0xd000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_save_end_v(void)
{
	return 0x00000003U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_save_end_f(void)
{
	return 0x3000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_restore_start_v(void)
{
	return 0x00000004U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_restore_start_f(void)
{
	return 0x4000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_context_start_v(void)
{
	return 0x00000005U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_context_start_f(void)
{
	return 0x5000000U;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_invalid_timestamp_v(void)
{
	return 0x000000ffU;
}
static inline u32 ctxsw_prog_record_timestamp_timestamp_hi_tag_invalid_timestamp_f(void)
{
	return 0xff000000U;
}
#endif
