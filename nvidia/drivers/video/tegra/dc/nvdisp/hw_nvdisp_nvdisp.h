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
#ifndef _hw_nvdisp_nvdisp_h_
#define _hw_nvdisp_nvdisp_h_

static inline u32 nvdisp_display_cmd_option_r(void)
{
	return 0x00000031U;
}
static inline u32 nvdisp_display_command_r(void)
{
	return 0x00000032U;
}
static inline u32 nvdisp_display_command_control_mode_stop_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_display_command_control_mode_c_display_f(void)
{
	return 0x20U;
}
static inline u32 nvdisp_display_command_control_mode_nc_display_f(void)
{
	return 0x40U;
}
static inline u32 nvdisp_cmd_int_status_r(void)
{
	return 0x00000037U;
}
static inline u32 nvdisp_cmd_int_status_frame_end_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvdisp_cmd_int_status_v_blank_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvdisp_cmd_int_status_region_crc_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 nvdisp_cmd_int_status_msf_f(u32 v)
{
	return (v & 0x1U) << 12U;
}
static inline u32 nvdisp_cmd_int_status_uf_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 nvdisp_cmd_int_status_sd3_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 nvdisp_cmd_int_status_obuf_uf_f(u32 v)
{
	return (v & 0x1U) << 26U;
}
static inline u32 nvdisp_cmd_int_status_rbuf_uf_f(u32 v)
{
	return (v & 0x1U) << 27U;
}
static inline u32 nvdisp_cmd_int_status_bbuf_uf_f(u32 v)
{
	return (v & 0x1U) << 28U;
}
static inline u32 nvdisp_cmd_int_status_dsc_uf_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 nvdisp_cmd_int_mask_r(void)
{
	return 0x00000038U;
}
static inline u32 nvdisp_cmd_int_enable_r(void)
{
	return 0x00000039U;
}
static inline u32 nvdisp_int_type_r(void)
{
	return 0x0000003aU;
}
static inline u32 nvdisp_int_polarity_r(void)
{
	return 0x0000003bU;
}
static inline u32 nvdisp_state_access_r(void)
{
	return 0x00000040U;
}
static inline u32 nvdisp_state_access_read_mux_assembly_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_state_access_read_mux_active_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_state_access_write_mux_assembly_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_state_access_write_mux_active_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_cmd_state_ctrl_r(void)
{
	return 0x00000041U;
}
static inline u32 nvdisp_cmd_state_ctrl_general_act_req_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_cmd_state_ctrl_a_act_req_enable_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_cmd_state_ctrl_b_act_req_enable_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_cmd_state_ctrl_c_act_req_enable_f(void)
{
	return 0x8U;
}
static inline u32 nvdisp_cmd_state_ctrl_d_act_req_enable_f(void)
{
	return 0x10U;
}
static inline u32 nvdisp_cmd_state_ctrl_e_act_req_enable_f(void)
{
	return 0x20U;
}
static inline u32 nvdisp_cmd_state_ctrl_f_act_req_enable_f(void)
{
	return 0x40U;
}
static inline u32 nvdisp_cmd_state_ctrl_cursor_act_req_enable_f(void)
{
	return 0x80U;
}
static inline u32 nvdisp_cmd_state_ctrl_general_update_enable_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_a_update_enable_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_b_update_enable_f(void)
{
	return 0x400U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_c_update_enable_f(void)
{
	return 0x800U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_d_update_enable_f(void)
{
	return 0x1000U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_e_update_enable_f(void)
{
	return 0x2000U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_f_update_enable_f(void)
{
	return 0x4000U;
}
static inline u32 nvdisp_cmd_state_ctrl_cursor_update_enable_f(void)
{
	return 0x8000U;
}
static inline u32 nvdisp_cmd_state_ctrl_common_act_req_enable_f(void)
{
	return 0x10000U;
}
static inline u32 nvdisp_cmd_state_ctrl_common_act_update_enable_f(void)
{
	return 0x20000U;
}
static inline u32 nvdisp_cmd_state_ctrl_host_trig_enable_f(void)
{
	return 0x1000000U;
}
static inline u32 nvdisp_cmd_state_ctrl_host_trig_secure_v(void)
{
	return 0x00000000U;
}
static inline u32 nvdisp_cmd_state_ctrl_gen_act_req_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_act_req_range_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvdisp_cmd_state_ctrl_win_act_req_range_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvdisp_cmd_disp_win_hdr_r(void)
{
	return 0x00000042U;
}
static inline u32 nvdisp_win_t_state_ctrl_r(void)
{
	return 0x00000044U;
}
static inline u32 nvdisp_secure_ctrl_r(void)
{
	return 0x00000045U;
}
static inline u32 nvdisp_reg_access_ctrl_r(void)
{
	return 0x00000046U;
}
static inline u32 nvdisp_postcomp_capa_r(void)
{
	return 0x00000050U;
}
static inline u32 nvdisp_postcomp_capa_is_tz_enable_v(u32 r)
{
	return (r >> 8U) & 0x1U;
}
static inline u32 nvdisp_postcomp_capa_is_lut_early_v(u32 r)
{
	return (r >> 7U) & 0x1U;
}
static inline u32 nvdisp_postcomp_capa_lut_type_v(u32 r)
{
	return (r >> 5U) & 0x3U;
}
static inline u32 nvdisp_postcomp_capa_lut_type_none_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_postcomp_capa_lut_type_s257_f(void)
{
	return 0x20U;
}
static inline u32 nvdisp_postcomp_capa_lut_type_s1025_f(void)
{
	return 0x40U;
}
static inline u32 nvdisp_postcomp_capa_is_ocsc_enable_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 nvdisp_postcomp_capa_is_hsat_enable_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 nvdisp_postcomp_capa_is_yuv422_enable_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 nvdisp_postcomp_capa_is_scaler_enable_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvdisp_postcomp_capa_is_scaler_has_yuv422_enable_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_r(void)
{
	return 0x00000060U;
}
static inline u32 nvdisp_ihub_capa_mempool_entries_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 nvdisp_ihub_capa_mempool_width_v(u32 r)
{
	return (r >> 16U) & 0x3U;
}
static inline u32 nvdisp_ihub_capa_rotation_support_v(u32 r)
{
	return (r >> 18U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_planar_support_v(u32 r)
{
	return (r >> 19U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_vga_support_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_mempool_compression_v(u32 r)
{
	return (r >> 21U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_mspg_support_v(u32 r)
{
	return (r >> 22U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_mclk_switch_support_v(u32 r)
{
	return (r >> 23U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_asr_support_v(u32 r)
{
	return (r >> 24U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_latency_support_v(u32 r)
{
	return (r >> 26U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_tz_window_support_v(u32 r)
{
	return (r >> 27U) & 0x1U;
}
static inline u32 nvdisp_ihub_capa_size_per_line_rot_v(u32 r)
{
	return (r >> 28U) & 0x3U;
}
static inline u32 nvdisp_ihub_capa_size_per_line_nonrot_v(u32 r)
{
	return (r >> 30U) & 0x3U;
}
static inline u32 nvdisp_ihub_capb_r(void)
{
	return 0x00000061U;
}
static inline u32 nvdisp_ihub_capb_max_planar_threadgrps_v(u32 r)
{
	return (r >> 0U) & 0x3fU;
}
static inline u32 nvdisp_ihub_capb_max_semi_planar_threadgrps_v(u32 r)
{
	return (r >> 6U) & 0x3fU;
}
static inline u32 nvdisp_ihub_capb_max_packed_2bpp_threadgrps_v(u32 r)
{
	return (r >> 12U) & 0x3fU;
}
static inline u32 nvdisp_ihub_capb_max_packed_1bpp_threadgrps_v(u32 r)
{
	return (r >> 18U) & 0x3fU;
}
static inline u32 nvdisp_ihub_capb_max_packed_422_threadgrps_v(u32 r)
{
	return (r >> 24U) & 0x3fU;
}
static inline u32 nvdisp_ihub_capc_r(void)
{
	return 0x00000062U;
}
static inline u32 nvdisp_ihub_capc_clear_rect_v(u32 r)
{
	return (r >> 8U) & 0x7U;
}
static inline u32 nvdisp_ihub_capc_max_lines_buffered_v(u32 r)
{
	return (r >> 4U) & 0x7U;
}
static inline u32 nvdisp_ihub_capc_pitch_size_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 nvdisp_ihub_capd_r(void)
{
	return 0x00000063U;
}
static inline u32 nvdisp_ihub_capd_rdout_bufsize_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 nvdisp_ihub_capd_reorder_buf_depth_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 nvdisp_ihub_config_r(void)
{
	return 0x00000067U;
}
static inline u32 nvdisp_ihub_config_batch_size_v(u32 r)
{
	return (r >> 0U) & 0x7U;
}
static inline u32 nvdisp_ihub_misc_ctl_r(void)
{
	return 0x00000068U;
}
static inline u32 nvdisp_ihub_misc_ctl_fetch_meter_enable_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_ihub_misc_ctl_req_limit_enable_f(void)
{
	return 0x40000000U;
}
static inline u32 nvdisp_ihub_misc_ctl_no_compress_enable_f(void)
{
	return 0x10000U;
}
static inline u32 nvdisp_ihub_misc_ctl_latency_event_enable_f(void)
{
	return 0x8U;
}
static inline u32 nvdisp_ihub_misc_ctl_critical_enable_f(void)
{
	return 0x20U;
}
static inline u32 nvdisp_ihub_misc_ctl_mspg_enable_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_ihub_misc_ctl_switch_enable_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_ihub_misc_ctl_asr_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_ihub_misc_emergency_r(void)
{
	return 0x00000069U;
}
static inline u32 nvdisp_ihub_misc_emergency_is_stopped_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvdisp_ihub_misc_emergency_stop_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_streamid_r(void)
{
	return 0x000000a0U;
}
static inline u32 nvdisp_crc_control_r(void)
{
	return 0x00000300U;
}
static inline u32 nvdisp_crc_control_enable_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_crc_control_enable_disable_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_crc_control_input_data_full_frame_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_crc_control_input_data_active_data_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_head_crc_control_r(void)
{
	return 0x00000301U;
}
static inline u32 nvdisp_comp_crca_r(void)
{
	return 0x0000032aU;
}
static inline u32 nvdisp_comp_crca_valid_false_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_comp_crca_valid_true_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_comp_crca_error_false_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_comp_crca_error_true_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_comp_crcb_r(void)
{
	return 0x0000032bU;
}
static inline u32 nvdisp_rg_crca_r(void)
{
	return 0x0000032cU;
}
static inline u32 nvdisp_rg_crca_valid_false_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_rg_crca_valid_true_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_rg_crca_error_false_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_rg_crca_error_true_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_rg_crcb_r(void)
{
	return 0x0000032dU;
}
static inline u32 nvdisp_rg_region_crc_r(void)
{
	return 0x000003a2U;
}
static inline u32 nvdisp_rg_region_crc_region0_error_yes_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_rg_region_crc_region1_error_yes_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_rg_region_crc_region2_error_yes_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_rg_region_crc_region3_error_yes_f(void)
{
	return 0x8U;
}
static inline u32 nvdisp_rg_region_crc_region4_error_yes_f(void)
{
	return 0x10U;
}
static inline u32 nvdisp_rg_region_crc_region5_error_yes_f(void)
{
	return 0x20U;
}
static inline u32 nvdisp_rg_region_crc_region6_error_yes_f(void)
{
	return 0x40U;
}
static inline u32 nvdisp_rg_region_crc_region7_error_yes_f(void)
{
	return 0x80U;
}
static inline u32 nvdisp_rg_region_crc_region8_error_yes_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_rg_region_crc_region0_pending_yes_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_rg_region_crc_region1_pending_yes_f(void)
{
	return 0x400U;
}
static inline u32 nvdisp_rg_region_crc_region2_pending_yes_f(void)
{
	return 0x800U;
}
static inline u32 nvdisp_rg_region_crc_region3_pending_yes_f(void)
{
	return 0x1000U;
}
static inline u32 nvdisp_rg_region_crc_region4_pending_yes_f(void)
{
	return 0x2000U;
}
static inline u32 nvdisp_rg_region_crc_region5_pending_yes_f(void)
{
	return 0x4000U;
}
static inline u32 nvdisp_rg_region_crc_region6_pending_yes_f(void)
{
	return 0x8000U;
}
static inline u32 nvdisp_rg_region_crc_region7_pending_yes_f(void)
{
	return 0x10000U;
}
static inline u32 nvdisp_rg_region_crc_region8_pending_yes_f(void)
{
	return 0x20000U;
}
static inline u32 nvdisp_rg_region_crc_control_r(void)
{
	return 0x000003a3U;
}
static inline u32 nvdisp_rg_region_crc_control_region0_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_rg_region_crc_control_region1_enable_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_rg_region_crc_control_region2_enable_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_rg_region_crc_control_region3_enable_f(void)
{
	return 0x8U;
}
static inline u32 nvdisp_rg_region_crc_control_region4_enable_f(void)
{
	return 0x10U;
}
static inline u32 nvdisp_rg_region_crc_control_region5_enable_f(void)
{
	return 0x20U;
}
static inline u32 nvdisp_rg_region_crc_control_region6_enable_f(void)
{
	return 0x40U;
}
static inline u32 nvdisp_rg_region_crc_control_region7_enable_f(void)
{
	return 0x80U;
}
static inline u32 nvdisp_rg_region_crc_control_region8_enable_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_rg_region_crc_control_readback_location_readback_hw_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_rg_region_crc_control_readback_location_readback_golden_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_rg_region_0_point_r(void)
{
	return 0x000003a4U;
}
static inline u32 nvdisp_rg_region_0_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_0_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_1_point_r(void)
{
	return 0x000003a5U;
}
static inline u32 nvdisp_rg_region_1_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_1_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_2_point_r(void)
{
	return 0x000003a6U;
}
static inline u32 nvdisp_rg_region_2_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_2_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_3_point_r(void)
{
	return 0x000003a7U;
}
static inline u32 nvdisp_rg_region_3_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_3_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_4_point_r(void)
{
	return 0x000003a8U;
}
static inline u32 nvdisp_rg_region_4_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_4_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_5_point_r(void)
{
	return 0x000003a9U;
}
static inline u32 nvdisp_rg_region_5_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_5_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_6_point_r(void)
{
	return 0x000003aaU;
}
static inline u32 nvdisp_rg_region_6_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_6_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_7_point_r(void)
{
	return 0x000003abU;
}
static inline u32 nvdisp_rg_region_7_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_7_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_8_point_r(void)
{
	return 0x000003acU;
}
static inline u32 nvdisp_rg_region_8_point_x_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_8_point_y_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_0_size_r(void)
{
	return 0x000003adU;
}
static inline u32 nvdisp_rg_region_0_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_0_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_1_size_r(void)
{
	return 0x000003aeU;
}
static inline u32 nvdisp_rg_region_1_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_1_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_2_size_r(void)
{
	return 0x000003afU;
}
static inline u32 nvdisp_rg_region_2_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_2_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_3_size_r(void)
{
	return 0x000003b0U;
}
static inline u32 nvdisp_rg_region_3_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_3_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_4_size_r(void)
{
	return 0x000003b1U;
}
static inline u32 nvdisp_rg_region_4_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_4_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_5_size_r(void)
{
	return 0x000003b2U;
}
static inline u32 nvdisp_rg_region_5_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_5_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_6_size_r(void)
{
	return 0x000003b3U;
}
static inline u32 nvdisp_rg_region_6_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_6_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_7_size_r(void)
{
	return 0x000003b4U;
}
static inline u32 nvdisp_rg_region_7_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_7_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_8_size_r(void)
{
	return 0x000003b5U;
}
static inline u32 nvdisp_rg_region_8_size_height_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_region_8_size_width_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_rg_region_0_golden_crc_r(void)
{
	return 0x000003b6U;
}
static inline u32 nvdisp_rg_region_1_golden_crc_r(void)
{
	return 0x000003b7U;
}
static inline u32 nvdisp_rg_region_2_golden_crc_r(void)
{
	return 0x000003b8U;
}
static inline u32 nvdisp_rg_region_3_golden_crc_r(void)
{
	return 0x000003b9U;
}
static inline u32 nvdisp_rg_region_4_golden_crc_r(void)
{
	return 0x000003baU;
}
static inline u32 nvdisp_rg_region_5_golden_crc_r(void)
{
	return 0x000003bbU;
}
static inline u32 nvdisp_rg_region_6_golden_crc_r(void)
{
	return 0x000003bcU;
}
static inline u32 nvdisp_rg_region_7_golden_crc_r(void)
{
	return 0x000003bdU;
}
static inline u32 nvdisp_rg_region_8_golden_crc_r(void)
{
	return 0x000003beU;
}
static inline u32 nvdisp_head_loadv_cntr_r(void)
{
	return 0x0000035fU;
}
static inline u32 nvdisp_rg_status_r(void)
{
	return 0x00000362U;
}
static inline u32 nvdisp_rg_dclk_r(void)
{
	return 0x00000364U;
}
static inline u32 nvdisp_rg_underflow_r(void)
{
	return 0x00000365U;
}
static inline u32 nvdisp_rg_underflow_enable_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_rg_underflow_enable_disable_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_rg_underflow_uflowed_no_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_rg_underflow_uflowed_yes_f(void)
{
	return 0x10U;
}
static inline u32 nvdisp_rg_underflow_uflowed_clr_f(void)
{
	return 0x10U;
}
static inline u32 nvdisp_rg_underflow_mode_repeat_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_rg_underflow_mode_red_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_rg_underflow_frames_uflowed_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 nvdisp_rg_underflow_is_frames_uflowed_rst_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 nvdisp_rg_underflow_frames_uflowed_rst_done_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_rg_underflow_frames_uflowed_rst_pending_f(void)
{
	return 0x1000000U;
}
static inline u32 nvdisp_rg_underflow_frames_uflowed_rst_trigger_f(void)
{
	return 0x1000000U;
}
static inline u32 nvdisp_rg_dpca_r(void)
{
	return 0x00000366U;
}
static inline u32 nvdisp_rg_underflow_pixel_r(void)
{
	return 0x0000036dU;
}
static inline u32 nvdisp_disp_signal_option_r(void)
{
	return 0x00000400U;
}
static inline u32 nvdisp_disp_signal_option_v_pulse2_enable_f(void)
{
	return 0x80000U;
}
static inline u32 nvdisp_disp_signal_option_v_pulse2_enable_v(u32 r)
{
	return (r >> 19U) & 0x1U;
}
static inline u32 nvdisp_disp_signal_option_v_pulse3_enable_f(void)
{
	return 0x100000U;
}
static inline u32 nvdisp_disp_signal_option_v_pulse3_enable_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 nvdisp_v_pulse2_position_a_r(void)
{
	return 0x00000423U;
}
static inline u32 nvdisp_v_pulse2_position_a_start_a_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 nvdisp_win_options_r(void)
{
	return 0x00000402U;
}
static inline u32 nvdisp_win_options_cursor_is_enable_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 nvdisp_win_options_cursor_set_cursor_enable_f(void)
{
	return 0x10000U;
}
static inline u32 nvdisp_win_options_sor_is_enable_v(u32 r)
{
	return (r >> 25U) & 0x1U;
}
static inline u32 nvdisp_win_options_sor_set_sor_enable_f(void)
{
	return 0x2000000U;
}
static inline u32 nvdisp_win_options_sor1_is_enable_v(u32 r)
{
	return (r >> 26U) & 0x1U;
}
static inline u32 nvdisp_win_options_sor1_set_sor1_enable_f(void)
{
	return 0x4000000U;
}
static inline u32 nvdisp_win_options_dsi_is_enable_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 nvdisp_win_options_dsi_set_dsi_enable_f(void)
{
	return 0x20000000U;
}
static inline u32 nvdisp_sor_control_r(void)
{
	return 0x00000403U;
}
static inline u32 nvdisp_sor_control_protocol_dpa_f(void)
{
	return 0x600U;
}
static inline u32 nvdisp_sor_control_protocol_dpb_f(void)
{
	return 0x700U;
}
static inline u32 nvdisp_sor_control_protocol_custom_f(void)
{
	return 0xf00U;
}
static inline u32 nvdisp_sor1_control_r(void)
{
	return 0x00000404U;
}
static inline u32 nvdisp_sor1_control_protocol_dpa_f(void)
{
	return 0x600U;
}
static inline u32 nvdisp_sor1_control_protocol_tmdsa_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_dsi_control_r(void)
{
	return 0x00000405U;
}
static inline u32 nvdisp_dsi_control_protocol_dsia_f(void)
{
	return 0x800U;
}
static inline u32 nvdisp_sync_width_r(void)
{
	return 0x00000407U;
}
static inline u32 nvdisp_sync_width_h_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_sync_width_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_back_porch_r(void)
{
	return 0x00000408U;
}
static inline u32 nvdisp_back_porch_h_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_back_porch_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_active_r(void)
{
	return 0x00000409U;
}
static inline u32 nvdisp_active_h_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_active_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_front_porch_r(void)
{
	return 0x0000040aU;
}
static inline u32 nvdisp_front_porch_h_f(u32 v)
{
	return (v & 0x7fffU) << 0U;
}
static inline u32 nvdisp_front_porch_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_rg_ext_back_porch_r(void)
{
	return 0x00000410U;
}
static inline u32 nvdisp_rg_ext_front_porch_r(void)
{
	return 0x00000411U;
}
static inline u32 nvdisp_rg_ext_r(void)
{
	return 0x00000412U;
}
static inline u32 nvdisp_rg_elv_0_r(void)
{
	return 0x00000413U;
}
static inline u32 nvdisp_start_fetch_dly_r(void)
{
	return 0x00000418U;
}
static inline u32 nvdisp_color_ctl_r(void)
{
	return 0x00000430U;
}
static inline u32 nvdisp_color_ctl_cmu_enable_f(void)
{
	return 0x100000U;
}
static inline u32 nvdisp_color_ctl_cmu_disable_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_color_ctl_dither_disable_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_color_ctl_dither_enable_f(void)
{
	return 0x80000U;
}
static inline u32 nvdisp_color_ctl_ord_dither_rot_f(u32 v)
{
	return (v & 0x3U) << 12U;
}
static inline u32 nvdisp_color_ctl_dither_ctl_disable_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_color_ctl_dither_ctl_err_acc_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_color_ctl_dither_ctl_ordered_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_color_ctl_dither_ctl_temporal_f(void)
{
	return 0x300U;
}
static inline u32 nvdisp_color_ctl_dither_phase_f(u32 v)
{
	return (v & 0x3U) << 6U;
}
static inline u32 nvdisp_color_ctl_base_color_size_18bits_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_color_ctl_base_color_size_24bits_f(void)
{
	return 0x8U;
}
static inline u32 nvdisp_color_ctl_base_color_size_30bits_f(void)
{
	return 0xaU;
}
static inline u32 nvdisp_color_ctl_base_color_size_36bits_f(void)
{
	return 0xcU;
}
static inline u32 nvdisp_output_lut_ctl_r(void)
{
	return 0x00000431U;
}
static inline u32 nvdisp_output_lut_ctl_mode_f(u32 v)
{
	return (v & 0x3U) << 5U;
}
static inline u32 nvdisp_output_lut_ctl_range_f(u32 v)
{
	return (v & 0x3U) << 3U;
}
static inline u32 nvdisp_output_lut_ctl_size_v(u32 r)
{
	return (r >> 1U) & 0x3U;
}
static inline u32 nvdisp_output_lut_ctl_size_257_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_output_lut_ctl_size_1025_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_output_lut_base_r(void)
{
	return 0x00000432U;
}
static inline u32 nvdisp_output_lut_base_hi_r(void)
{
	return 0x00000433U;
}
static inline u32 nvdisp_procamp_r(void)
{
	return 0x00000434U;
}
static inline u32 nvdisp_procamp_sat_sine_f(u32 v)
{
	return (v & 0xfffU) << 15U;
}
static inline u32 nvdisp_procamp_sat_cos_f(u32 v)
{
	return (v & 0xfffU) << 3U;
}
static inline u32 nvdisp_procamp_chroma_lpf_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvdisp_procamp_chroma_lpf_enable_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_cursor_ctrl_r(void)
{
	return 0x0000043bU;
}
static inline u32 nvdisp_display_rate_r(void)
{
	return 0x00000415U;
}
static inline u32 nvdisp_display_rate_min_refresh_enable_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 nvdisp_display_rate_min_refresh_interval_f(u32 v)
{
	return (v & 0x3fffffU) << 1U;
}
static inline u32 nvdisp_cursor_startaddr_r(void)
{
	return 0x0000043eU;
}
static inline u32 nvdisp_cursor_startaddr_range_v(u32 r)
{
	return (r >> 0U) & 0x3fffffU;
}
static inline u32 nvdisp_cursor_startaddr_get_size_v(u32 r)
{
	return (r >> 24U) & 0x3U;
}
static inline u32 nvdisp_cursor_startaddr_get_size_size_32x32_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_cursor_startaddr_get_size_size_64x64_f(void)
{
	return 0x1000000U;
}
static inline u32 nvdisp_cursor_startaddr_get_size_size_128x128_f(void)
{
	return 0x2000000U;
}
static inline u32 nvdisp_cursor_startaddr_get_size_size_256x256_f(void)
{
	return 0x3000000U;
}
static inline u32 nvdisp_cursor_position_r(void)
{
	return 0x00000440U;
}
static inline u32 nvdisp_cursor_position_h_range_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 nvdisp_cursor_position_v_range_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 nvdisp_cursor_position_h_size_s(void)
{
	return 16U;
}
static inline u32 nvdisp_cursor_cropped_point_in_r(void)
{
	return 0x00000442U;
}
static inline u32 nvdisp_cursor_cropped_point_in_x_range_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 nvdisp_cursor_cropped_point_in_y_range_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 nvdisp_cursor_cropped_size_in_r(void)
{
	return 0x00000446U;
}
static inline u32 nvdisp_cursor_cropped_size_in_width_range_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 nvdisp_cursor_cropped_size_in_height_range_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 nvdisp_ihub_cursor_latency_ctla_r(void)
{
	return 0x00000382U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctla_submode_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctla_submode_watermark_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctla_submode_vblank_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctla_submode_watermark_and_vblank_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctla_ctl_mode_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctla_ctl_mode_enable_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctlb_r(void)
{
	return 0x00000383U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctlb_status_above_watermark_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctlb_status_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctlb_status_above_watermark_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_ihub_cursor_latency_ctlb_watermark_f(u32 v)
{
	return (v & 0x1fffffffU) << 0U;
}
static inline u32 nvdisp_cursor_pipe_meter_r(void)
{
	return 0x00000450U;
}
static inline u32 nvdisp_cursor_pipe_meter_status_f(u32 v)
{
	return (v & 0x3U) << 30U;
}
static inline u32 nvdisp_cursor_pipe_meter_status_active_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_cursor_pipe_meter_status_armed_f(void)
{
	return 0x40000000U;
}
static inline u32 nvdisp_cursor_pipe_meter_status_assembly_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_cursor_pipe_meter_val_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 nvdisp_ihub_common_fetch_meter_r(void)
{
	return 0x00000451U;
}
static inline u32 nvdisp_ihub_common_fetch_meter_cursor_slots_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 nvdisp_ihub_common_fetch_meter_wgrp_slots_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 nvdisp_ihub_cursor_pool_config_r(void)
{
	return 0x00000452U;
}
static inline u32 nvdisp_ihub_cursor_pool_config_status_pending_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvdisp_ihub_cursor_pool_config_status_pending_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_ihub_cursor_pool_config_entries_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 nvdisp_ihub_cursor_fetch_meter_r(void)
{
	return 0x00000453U;
}
static inline u32 nvdisp_ihub_cursor_fetch_meter_slots_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 nvdisp_sd_hist_ctrl_r(void)
{
	return 0x000004c2U;
}
static inline u32 nvdisp_sd_hist_ctrl_is_enable_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvdisp_sd_hist_ctrl_set_enable_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_sd_hist_ctrl_is_reset_busy_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 nvdisp_sd_hist_ctrl_reset_start_f(void)
{
	return 0x40000000U;
}
static inline u32 nvdisp_sd_hist_ctrl_is_ram_busy_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 nvdisp_sd_hist_ctrl_is_vid_luma_enable_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 nvdisp_sd_hist_ctrl_set_vid_luma_enable_f(void)
{
	return 0x10000000U;
}
static inline u32 nvdisp_sd_hist_ctrl_is_win_enable_v(u32 r)
{
	return (r >> 27U) & 0x1U;
}
static inline u32 nvdisp_sd_hist_ctrl_set_window_enable_f(void)
{
	return 0x8000000U;
}
static inline u32 nvdisp_sd_hist_ctrl_max_pixel_f(u32 v)
{
	return (v & 0xffffffU) << 0U;
}
static inline u32 nvdisp_sd_hist_luma_r(void)
{
	return 0x000004c3U;
}
static inline u32 nvdisp_sd_hist_luma_is_valid_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvdisp_sd_hist_luma_num_pixels_v(u32 r)
{
	return (r >> 0U) & 0x3ffffffU;
}
static inline u32 nvdisp_sd_hist_over_sat_r(void)
{
	return 0x000004c4U;
}
static inline u32 nvdisp_sd_hist_over_sat_bin_num_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 nvdisp_sd_hist_int_bounds_r(void)
{
	return 0x000004c5U;
}
static inline u32 nvdisp_sd_hist_int_bounds_is_enable_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvdisp_sd_hist_int_bounds_set_enable_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_sd_hist_int_bounds_upper_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 nvdisp_sd_hist_int_bounds_lower_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 nvdisp_sd_hist_vid_luma_r(void)
{
	return 0x000004c6U;
}
static inline u32 nvdisp_sd_hist_vid_luma_b_coeff_f(u32 v)
{
	return (v & 0xfU) << 8U;
}
static inline u32 nvdisp_sd_hist_vid_luma_g_coeff_f(u32 v)
{
	return (v & 0xfU) << 4U;
}
static inline u32 nvdisp_sd_hist_vid_luma_r_coeff_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 nvdisp_sd_gain_ctrl_r(void)
{
	return 0x000004c7U;
}
static inline u32 nvdisp_sd_gain_ctrl_is_enable_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvdisp_sd_gain_ctrl_set_enable_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_sd_gain_ctrl_update_busy_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 nvdisp_sd_gain_ctrl_update_start_f(void)
{
	return 0x40000000U;
}
static inline u32 nvdisp_sd_gain_ctrl_timing_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 nvdisp_sd_gain_ctrl_reset_busy_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 nvdisp_sd_gain_ctrl_reset_start_f(void)
{
	return 0x10000000U;
}
static inline u32 nvdisp_sd_gain_rg_r(void)
{
	return 0x000004c8U;
}
static inline u32 nvdisp_sd_gain_rg_gint_f(u32 v)
{
	return (v & 0x3U) << 30U;
}
static inline u32 nvdisp_sd_gain_rg_gfrac_f(u32 v)
{
	return (v & 0xfffU) << 18U;
}
static inline u32 nvdisp_sd_gain_rg_gval_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 nvdisp_sd_gain_rg_rint_f(u32 v)
{
	return (v & 0x3U) << 14U;
}
static inline u32 nvdisp_sd_gain_rg_rfrac_f(u32 v)
{
	return (v & 0xfffU) << 2U;
}
static inline u32 nvdisp_sd_gain_rg_rval_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 nvdisp_sd_gain_b_r(void)
{
	return 0x000004c9U;
}
static inline u32 nvdisp_sd_gain_b_int_f(u32 v)
{
	return (v & 0x3U) << 14U;
}
static inline u32 nvdisp_sd_gain_b_frac_f(u32 v)
{
	return (v & 0xfffU) << 2U;
}
static inline u32 nvdisp_sd_gain_b_val_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 nvdisp_sd_hist_win_pos_r(void)
{
	return 0x000004cbU;
}
static inline u32 nvdisp_sd_hist_win_pos_v_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 nvdisp_sd_hist_win_pos_h_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 nvdisp_sd_hist_win_size_r(void)
{
	return 0x000004ccU;
}
static inline u32 nvdisp_sd_hist_win_size_height_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 nvdisp_sd_hist_win_size_width_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 nvdisp_incr_syncpt_cntrl_r(void)
{
	return 0x00000001U;
}
static inline u32 nvdisp_incr_syncpt_cntrl_no_stall_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 nvdisp_cont_syncpt_vsync_r(void)
{
	return 0x00000028U;
}
static inline u32 nvdisp_cont_syncpt_vsync_indx_f(u32 v)
{
	return (v & 0x3ffU) << 0U;
}
static inline u32 nvdisp_cont_syncpt_vsync_en_enable_f(void)
{
	return 0x80000000U;
}
static inline u32 nvdisp_background_color_r(void)
{
	return 0x000004e4U;
}
static inline u32 nvdisp_background_color_reset_val_v(void)
{
	return 0xff000000;
}
static inline u32 nvdisp_interlace_ctl_r(void)
{
	return 0x000004e5U;
}
static inline u32 nvdisp_interlace_fld2_width_r(void)
{
	return 0x000004e7U;
}
static inline u32 nvdisp_interlace_fld2_width_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_interlace_fld2_bporch_r(void)
{
	return 0x000004e8U;
}
static inline u32 nvdisp_interlace_fld2_bporch_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_interlace_fld2_fporch_r(void)
{
	return 0x000004e9U;
}
static inline u32 nvdisp_interlace_fld2_fporch_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_interlace_fld2_active_r(void)
{
	return 0x000004eaU;
}
static inline u32 nvdisp_interlace_fld2_active_v_f(u32 v)
{
	return (v & 0x7fffU) << 16U;
}
static inline u32 nvdisp_cursor_startaddr_hi_r(void)
{
	return 0x000004ecU;
}
static inline u32 nvdisp_cursor_startaddr_hi_range_sw_default_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_csc2_control_r(void)
{
	return 0x000004efU;
}
static inline u32 nvdisp_csc2_control_limit_rgb_enable_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_csc2_control_limit_rgb_disable_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_csc2_control_output_color_sel_rgb_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_csc2_control_output_color_sel_y709_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_csc2_control_output_color_sel_y601_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_csc2_control_output_color_sel_y2020_f(void)
{
	return 0x3U;
}
static inline u32 nvdisp_blend_cursor_ctrl_r(void)
{
	return 0x000004f1U;
}
static inline u32 nvdisp_rg_loadv_r(void)
{
	return 0x00000363U;
}
#endif
