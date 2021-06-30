/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_win_nvdisp_h_
#define _hw_win_nvdisp_h_

static inline u32 win_base_addr_dc_winc_r(void)
{
	return 0x00000500U;
}
static inline u32 win_base_addr_dc_win_r(void)
{
	return 0x00000700U;
}
static inline u32 win_base_addr_dc_winbuf_r(void)
{
	return 0x00000800U;
}
static inline u32 win_base_addr_dc_a_winc_r(void)
{
	return 0x00000a00U;
}
static inline u32 win_base_addr_dc_a_win_r(void)
{
	return 0x00000b80U;
}
static inline u32 win_base_addr_dc_a_winbuf_r(void)
{
	return 0x00000bc0U;
}
static inline u32 win_base_addr_dc_b_winc_r(void)
{
	return 0x00000d00U;
}
static inline u32 win_base_addr_dc_b_win_r(void)
{
	return 0x00000e80U;
}
static inline u32 win_base_addr_dc_b_winbuf_r(void)
{
	return 0x00000ec0U;
}
static inline u32 win_base_addr_dc_c_winc_r(void)
{
	return 0x00001000U;
}
static inline u32 win_base_addr_dc_c_win_r(void)
{
	return 0x00001180U;
}
static inline u32 win_base_addr_dc_c_winbuf_r(void)
{
	return 0x000011c0U;
}
static inline u32 win_base_addr_dc_d_winc_r(void)
{
	return 0x00001300U;
}
static inline u32 win_base_addr_dc_d_win_r(void)
{
	return 0x00001480U;
}
static inline u32 win_base_addr_dc_d_winbuf_r(void)
{
	return 0x000014c0U;
}
static inline u32 win_base_addr_dc_e_winc_r(void)
{
	return 0x00001600U;
}
static inline u32 win_base_addr_dc_e_win_r(void)
{
	return 0x00001780U;
}
static inline u32 win_base_addr_dc_e_winbuf_r(void)
{
	return 0x000017c0U;
}
static inline u32 win_base_addr_dc_f_winc_r(void)
{
	return 0x00001900U;
}
static inline u32 win_base_addr_dc_f_win_r(void)
{
	return 0x00001a80U;
}
static inline u32 win_base_addr_dc_f_winbuf_r(void)
{
	return 0x00001ac0U;
}
static inline u32 win_precomp_wgrp_capa_r(void)
{
	return 0x00000500U;
}
static inline u32 win_precomp_wgrp_capb_r(void)
{
	return 0x00000501U;
}
static inline u32 win_precomp_wgrp_capb_is_degamma_enable_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 win_precomp_wgrp_capb_is_fp16_enable_v(u32 r)
{
	return (r >> 15U) & 0x1U;
}
static inline u32 win_precomp_wgrp_capb_is_cgmt_present_v(u32 r)
{
	return (r >> 14U) & 0x1U;
}
static inline u32 win_precomp_wgrp_capb_scaler_type_v(u32 r)
{
	return (r >> 8U) & 0x3U;
}
static inline u32 win_precomp_wgrp_capb_max_windows_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 win_precomp_wgrp_capc_r(void)
{
	return 0x00000502U;
}
static inline u32 win_precomp_wgrp_capc_max_pixels_5tap444_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 win_precomp_wgrp_cape_r(void)
{
	return 0x00000504U;
}
static inline u32 win_precomp_wgrp_cape_max_pixels_2tap444_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 win_act_control_r(void)
{
	return 0x0000050eU;
}
static inline u32 win_act_control_ctrl_sel_vcounter_f(void)
{
	return 0x0U;
}
static inline u32 win_act_control_ctrl_sel_hcounter_f(void)
{
	return 0x1U;
}
static inline u32 win_ihub_latency_ctla_r(void)
{
	return 0x00000543U;
}
static inline u32 win_ihub_latency_ctla_submode_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 win_ihub_latency_ctla_submode_watermark_f(void)
{
	return 0x0U;
}
static inline u32 win_ihub_latency_ctla_submode_vblank_f(void)
{
	return 0x1U;
}
static inline u32 win_ihub_latency_ctla_submode_watermark_and_vblank_f(void)
{
	return 0x2U;
}
static inline u32 win_ihub_latency_ctla_ctl_mode_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 win_ihub_latency_ctla_ctl_mode_enable_f(void)
{
	return 0x4U;
}
static inline u32 win_ihub_latency_ctlb_r(void)
{
	return 0x00000544U;
}
static inline u32 win_ihub_latency_ctlb_status_above_watermark_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 win_ihub_latency_ctlb_status_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 win_ihub_latency_ctlb_status_above_watermark_f(void)
{
	return 0x80000000U;
}
static inline u32 win_ihub_latency_ctlb_watermark_f(u32 v)
{
	return (v & 0x1fffffffU) << 0U;
}
static inline u32 win_precomp_loadv_counter_r(void)
{
	return 0x00000520U;
}
static inline u32 win_precomp_pipe_meter_r(void)
{
	return 0x00000560U;
}
static inline u32 win_precomp_pipe_meter_status_pending_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 win_precomp_pipe_meter_status_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 win_precomp_pipe_meter_status_pending_f(void)
{
	return 0x80000000U;
}
static inline u32 win_precomp_pipe_meter_val_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 win_ihub_pool_config_r(void)
{
	return 0x00000561U;
}
static inline u32 win_ihub_pool_config_status_pending_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 win_ihub_pool_config_status_pending_f(void)
{
	return 0x80000000U;
}
static inline u32 win_ihub_pool_config_entries_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 win_ihub_fetch_meter_r(void)
{
	return 0x00000562U;
}
static inline u32 win_ihub_fetch_meter_slots_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 win_ihub_linebuf_config_r(void)
{
	return 0x00000563U;
}
static inline u32 win_ihub_linebuf_config_mode_two_lines_f(void)
{
	return 0x0U;
}
static inline u32 win_ihub_linebuf_config_mode_four_lines_f(void)
{
	return 0x4000U;
}
static inline u32 win_ihub_req_r(void)
{
	return 0x00000564U;
}
static inline u32 win_ihub_req_limit_f(u32 v)
{
	return (v & 0xfffU) << 0U;
}
static inline u32 win_ihub_debug_req_r(void)
{
	return 0x00000565U;
}
static inline u32 win_ihub_debug_req_status_f(u32 v)
{
	return (v & 0xfffU) << 0U;
}
static inline u32 win_ihub_status_r(void)
{
	return 0x00000566U;
}
static inline u32 win_ihub_status_state_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 win_ihub_status_compression_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 win_ihub_status_entires_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 win_ihub_config_r(void)
{
	return 0x00000567U;
}
static inline u32 win_ihub_config_pitch_size_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 win_ihub_thread_group_r(void)
{
	return 0x00000568U;
}
static inline u32 win_ihub_thread_group_num_f(u32 v)
{
	return (v & 0x1fU) << 1U;
}
static inline u32 win_ihub_thread_group_enable_yes_f(void)
{
	return 0x1U;
}
static inline u32 win_ihub_thread_group_enable_no_f(void)
{
	return 0x0U;
}
static inline u32 win_precomp_wgrp_rgb_color_r(void)
{
	return 0x00000524U;
}
static inline u32 win_streamid_r(void)
{
	return 0x00000580U;
}
static inline u32 win_streamid_id_f(u32 v)
{
	return (v & 0x3ffU) << 0U;
}
static inline u32 win_options_r(void)
{
	return 0x00000700U;
}
static inline u32 win_options_v_direction_increment_f(void)
{
	return 0x0U;
}
static inline u32 win_options_v_direction_decrement_f(void)
{
	return 0x4U;
}
static inline u32 win_options_h_direction_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 win_options_h_direction_increment_f(void)
{
	return 0x0U;
}
static inline u32 win_options_h_direction_decrement_f(void)
{
	return 0x1U;
}
static inline u32 win_options_scan_column_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 win_options_scan_column_disable_f(void)
{
	return 0x0U;
}
static inline u32 win_options_scan_column_enable_f(void)
{
	return 0x10U;
}
static inline u32 win_options_win_enable_enable_f(void)
{
	return 0x40000000U;
}
static inline u32 win_options_win_enable_disable_f(void)
{
	return 0x0U;
}
static inline u32 win_options_color_expand_enable_f(void)
{
	return 0x40U;
}
static inline u32 win_options_cp_enable_enable_f(void)
{
	return 0x10000U;
}
static inline u32 win_set_control_r(void)
{
	return 0x00000702U;
}
static inline u32 win_set_control_owner_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 win_set_control_owner_none_f(void)
{
	return 0xfU;
}
static inline u32 win_color_depth_r(void)
{
	return 0x00000703U;
}
static inline u32 win_position_r(void)
{
	return 0x00000704U;
}
static inline u32 win_position_h_position_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 win_position_v_position_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 win_size_r(void)
{
	return 0x00000705U;
}
static inline u32 win_size_h_size_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 win_size_v_size_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 win_set_cropped_size_in_r(void)
{
	return 0x00000706U;
}
static inline u32 win_set_cropped_size_in_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 win_set_cropped_size_in_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 win_scaler_hstart_phase_r(void)
{
	return 0x00000707U;
}
static inline u32 win_scaler_hstart_phase_phase_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 win_scaler_vstart_phase_r(void)
{
	return 0x00000708U;
}
static inline u32 win_scaler_vstart_phase_phase_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 win_set_planar_storage_r(void)
{
	return 0x00000709U;
}
static inline u32 win_set_planar_storage_uv_r(void)
{
	return 0x0000070aU;
}
static inline u32 win_set_planar_storage_uv_uv0_f(u32 v)
{
	return (v & 0x1fffU) << 0U;
}
static inline u32 win_set_planar_storage_uv_uv1_f(u32 v)
{
	return (v & 0x1fffU) << 16U;
}
static inline u32 win_scaler_hphase_incr_r(void)
{
	return 0x0000070bU;
}
static inline u32 win_scaler_hphase_incr_incr_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 win_scaler_vphase_incr_r(void)
{
	return 0x0000070cU;
}
static inline u32 win_scaler_vphase_incr_incr_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 win_win_set_params_r(void)
{
	return 0x0000070dU;
}
static inline u32 win_win_set_params_cs_range_yuv_709_f(void)
{
	return 0x200U;
}
static inline u32 win_win_set_params_cs_range_yuv_2020_f(void)
{
	return 0x300U;
}
static inline u32 win_win_set_params_cs_range_yuv_601_f(void)
{
	return 0x100U;
}
static inline u32 win_win_set_params_cs_range_rgb_f(void)
{
	return 0x0U;
}
static inline u32 win_win_set_params_in_range_bypass_f(void)
{
	return 0x0U;
}
static inline u32 win_win_set_params_in_range_limited_f(void)
{
	return 0x400U;
}
static inline u32 win_win_set_params_in_range_full_f(void)
{
	return 0x800U;
}
static inline u32 win_win_set_params_degamma_range_none_f(void)
{
	return 0x0U;
}
static inline u32 win_win_set_params_degamma_range_srgb_f(void)
{
	return 0x2000U;
}
static inline u32 win_win_set_params_degamma_range_yuv8_10_f(void)
{
	return 0x4000U;
}
static inline u32 win_win_set_params_degamma_range_yuv12_f(void)
{
	return 0x6000U;
}
static inline u32 win_win_set_params_degamma_range_mask_f(void)
{
	return 0x6000U;
}
static inline u32 win_win_set_params_clamp_before_blend_enable_f(void)
{
	return 0x8000U;
}
static inline u32 win_win_set_params_clamp_before_blend_disable_f(void)
{
	return 0x0U;
}
static inline u32 win_scaler_input_r(void)
{
	return 0x0000070eU;
}
static inline u32 win_scaler_input_h_taps_2_f(void)
{
	return 0x8U;
}
static inline u32 win_scaler_input_h_taps_5_f(void)
{
	return 0x20U;
}
static inline u32 win_scaler_input_v_taps_2_f(void)
{
	return 0x1U;
}
static inline u32 win_scaler_input_v_taps_5_f(void)
{
	return 0x4U;
}
static inline u32 win_scaler_set_coeff_r(void)
{
	return 0x0000070fU;
}
static inline u32 win_scaler_set_coeff_read_index_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 win_scaler_set_coeff_index_f(u32 v)
{
	return (v & 0xffU) << 15U;
}
static inline u32 win_scaler_set_coeff_data_f(u32 v)
{
	return (v & 0x3ffU) << 0U;
}
static inline u32 win_scaler_usage_r(void)
{
	return 0x00000711U;
}
static inline u32 win_scaler_usage_hbypass_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 win_scaler_usage_vbypass_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 win_scaler_usage_use422_disable_f(void)
{
	return 0x0U;
}
static inline u32 win_scaler_usage_use422_enable_f(void)
{
	return 0x4U;
}
static inline u32 win_scaler_point_in_r(void)
{
	return 0x00000712U;
}
static inline u32 win_scaler_point_in_y_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 win_scaler_point_in_x_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 win_blend_layer_control_r(void)
{
	return 0x00000716U;
}
static inline u32 win_blend_layer_control_color_key_src_f(void)
{
	return 0x2000000U;
}
static inline u32 win_blend_layer_control_color_key_dest_f(void)
{
	return 0x4000000U;
}
static inline u32 win_blend_layer_control_color_key_none_f(void)
{
	return 0x0U;
}
static inline u32 win_blend_layer_control_blend_enable_bypass_f(void)
{
	return 0x1000000U;
}
static inline u32 win_blend_layer_control_blend_enable_enable_f(void)
{
	return 0x0U;
}
static inline u32 win_blend_layer_control_blend_enable_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 win_blend_layer_control_k1_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 win_blend_layer_control_k2_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 win_blend_layer_control_depth_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 win_blend_match_select_r(void)
{
	return 0x00000717U;
}
static inline u32 win_blend_nomatch_select_r(void)
{
	return 0x00000718U;
}
static inline u32 win_input_lut_base_r(void)
{
	return 0x00000720U;
}
static inline u32 win_input_lut_base_hi_r(void)
{
	return 0x00000721U;
}
static inline u32 win_input_lut_ctl_r(void)
{
	return 0x00000723U;
}
static inline u32 win_wgrp_params_r(void)
{
	return 0x00000724U;
}
static inline u32 win_wgrp_params_swap_uv_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 win_window_set_control_r(void)
{
	return 0x00000730U;
}
static inline u32 win_window_set_control_csc_enable_f(void)
{
	return 0x20U;
}
static inline u32 win_window_set_control_csc_disable_f(void)
{
	return 0x0U;
}
static inline u32 win_r2r_r(void)
{
	return 0x00000731U;
}
static inline u32 win_r2r_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_g2r_r(void)
{
	return 0x00000732U;
}
static inline u32 win_g2r_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_b2r_r(void)
{
	return 0x00000733U;
}
static inline u32 win_b2r_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_const2r_r(void)
{
	return 0x00000734U;
}
static inline u32 win_const2r_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_r2g_r(void)
{
	return 0x00000735U;
}
static inline u32 win_r2g_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_g2g_r(void)
{
	return 0x00000736U;
}
static inline u32 win_g2g_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_b2g_r(void)
{
	return 0x00000737U;
}
static inline u32 win_b2g_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_const2g_r(void)
{
	return 0x00000738U;
}
static inline u32 win_const2g_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_r2b_r(void)
{
	return 0x00000739U;
}
static inline u32 win_r2b_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_g2b_r(void)
{
	return 0x0000073aU;
}
static inline u32 win_g2b_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_b2b_r(void)
{
	return 0x0000073bU;
}
static inline u32 win_b2b_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_const2b_r(void)
{
	return 0x0000073cU;
}
static inline u32 win_const2b_coeff_f(u32 v)
{
	return (v & 0x7ffffU) << 0U;
}
static inline u32 win_start_addr_r(void)
{
	return 0x00000800U;
}
static inline u32 win_start_addr_u_r(void)
{
	return 0x00000802U;
}
static inline u32 win_start_addr_v_r(void)
{
	return 0x00000804U;
}
static inline u32 win_cropped_point_r(void)
{
	return 0x00000806U;
}
static inline u32 win_cropped_point_v_offset_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 win_cropped_point_h_offset_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 win_surface_kind_r(void)
{
	return 0x0000080bU;
}
static inline u32 win_surface_kind_kind_pitch_f(void)
{
	return 0x0U;
}
static inline u32 win_surface_kind_kind_tiled_f(void)
{
	return 0x1U;
}
static inline u32 win_surface_kind_kind_bl_f(void)
{
	return 0x2U;
}
static inline u32 win_surface_kind_block_height_f(u32 v)
{
	return (v & 0x7U) << 4U;
}
static inline u32 win_start_addr_hi_r(void)
{
	return 0x0000080dU;
}
static inline u32 win_start_addr_hi_u_r(void)
{
	return 0x0000080fU;
}
static inline u32 win_start_addr_hi_v_r(void)
{
	return 0x00000811U;
}
static inline u32 win_start_addr_fld2_r(void)
{
	return 0x00000813U;
}
static inline u32 win_start_addr_fld2_u_r(void)
{
	return 0x00000815U;
}
static inline u32 win_start_addr_fld2_v_r(void)
{
	return 0x00000817U;
}
static inline u32 win_start_addr_fld2_hi_r(void)
{
	return 0x00000819U;
}
static inline u32 win_start_addr_fld2_hi_u_r(void)
{
	return 0x0000081bU;
}
static inline u32 win_start_addr_fld2_hi_v_r(void)
{
	return 0x0000081dU;
}
static inline u32 win_cropped_point_fld2_r(void)
{
	return 0x0000081fU;
}
static inline u32 win_cropped_point_fld2_v_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 win_cropped_point_fld2_h_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 win_cde_ctrl_r(void)
{
	return 0x0000082fU;
}
static inline u32 win_cde_ctrl_pattern_fixed_f(void)
{
	return 0x0U;
}
static inline u32 win_cde_ctrl_pattern_random_f(void)
{
	return 0x800U;
}
static inline u32 win_cde_ctrl_kind_cra_f(void)
{
	return 0x0U;
}
static inline u32 win_cde_ctrl_kind_bra_f(void)
{
	return 0x10U;
}
static inline u32 win_cde_ctrl_kind_yuv_8b_1c_f(void)
{
	return 0x20U;
}
static inline u32 win_cde_ctrl_kind_yuv_8b_2c_f(void)
{
	return 0x30U;
}
static inline u32 win_cde_ctrl_kind_yuv_10b_1c_f(void)
{
	return 0x40U;
}
static inline u32 win_cde_ctrl_kind_yuv_10b_2c_f(void)
{
	return 0x50U;
}
static inline u32 win_cde_ctrl_kind_yuv_12b_1c_f(void)
{
	return 0x60U;
}
static inline u32 win_cde_ctrl_kind_yuv_12b_2c_f(void)
{
	return 0x70U;
}
static inline u32 win_cde_ctrl_surface_enable_f(void)
{
	return 0x1U;
}
static inline u32 win_cde_ctrl_surface_disble_f(void)
{
	return 0x0U;
}
static inline u32 win_cde_base_r(void)
{
	return 0x00000830U;
}
static inline u32 win_cde_base_addr_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 win_cde_base_hi_r(void)
{
	return 0x00000832U;
}
static inline u32 win_cde_base_hi_addr_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 win_cde_base_fld2_r(void)
{
	return 0x00000834U;
}
static inline u32 win_cde_base_fld2_addr_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 win_cde_base_hi_fld2_r(void)
{
	return 0x00000836U;
}
static inline u32 win_cde_base_hi_fld2_addr_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 win_cde_zbc_r(void)
{
	return 0x00000838U;
}
static inline u32 win_cde_zbc_color_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 win_cde_ctb_r(void)
{
	return 0x0000083dU;
}
static inline u32 win_cde_ctb_entry_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
#endif
