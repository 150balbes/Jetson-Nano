/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_nvdisp_t19x_nvdisp_h_
#define _hw_nvdisp_t19x_nvdisp_h_

static inline u32 nvdisp_t19x_ihub_common_config_r(void)
{
	return 0x00000067U;
}
static inline u32 nvdisp_t19x_ihub_common_config_request_batch_size_1_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_t19x_ihub_common_config_request_batch_size_2_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_t19x_ihub_common_config_request_batch_size_4_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_t19x_ihub_common_config_request_batch_size_8_f(void)
{
	return 0x3U;
}
static inline u32 nvdisp_t19x_ihub_common_config_request_batch_size_16_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_t19x_ihub_common_config_request_batch_size_32_f(void)
{
	return 0x5U;
}
static inline u32 nvdisp_t19x_win_options_r(void)
{
	return 0x00000402U;
}
static inline u32 nvdisp_t19x_win_options_cursor_is_enable_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 nvdisp_t19x_win_options_cursor_set_cursor_enable_f(void)
{
	return 0x10000U;
}
static inline u32 nvdisp_t19x_win_options_sor_is_enable_v(u32 r)
{
	return (r >> 25U) & 0x1U;
}
static inline u32 nvdisp_t19x_win_options_sor_set_sor_enable_f(void)
{
	return 0x2000000U;
}
static inline u32 nvdisp_t19x_win_options_sor1_is_enable_v(u32 r)
{
	return (r >> 26U) & 0x1U;
}
static inline u32 nvdisp_t19x_win_options_sor1_set_sor1_enable_f(void)
{
	return 0x4000000U;
}
static inline u32 nvdisp_t19x_win_options_sor2_is_enable_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 nvdisp_t19x_win_options_sor2_set_sor2_enable_f(void)
{
	return 0x10000000U;
}
static inline u32 nvdisp_t19x_win_options_sor3_is_enable_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 nvdisp_t19x_win_options_sor3_set_sor3_enable_f(void)
{
	return 0x20000000U;
}
static inline u32 nvdisp_t19x_sor_control_r(void)
{
	return 0x00000403U;
}
static inline u32 nvdisp_t19x_sor_control_protocol_tmdsa_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_t19x_sor_control_protocol_tmdsb_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_t19x_sor_control_protocol_dpa_f(void)
{
	return 0x600U;
}
static inline u32 nvdisp_t19x_sor_control_protocol_dpb_f(void)
{
	return 0x700U;
}
static inline u32 nvdisp_t19x_sor_control_protocol_custom_f(void)
{
	return 0xf00U;
}
static inline u32 nvdisp_t19x_sor1_control_r(void)
{
	return 0x00000404U;
}
static inline u32 nvdisp_t19x_sor1_control_protocol_tmdsa_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_t19x_sor1_control_protocol_tmdsb_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_t19x_sor1_control_protocol_dpa_f(void)
{
	return 0x600U;
}
static inline u32 nvdisp_t19x_sor1_control_protocol_dpb_f(void)
{
	return 0x700U;
}
static inline u32 nvdisp_t19x_sor1_control_protocol_custom_f(void)
{
	return 0xf00U;
}
static inline u32 nvdisp_t19x_sor2_control_r(void)
{
	return 0x00000405U;
}
static inline u32 nvdisp_t19x_sor2_control_protocol_tmdsa_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_t19x_sor2_control_protocol_tmdsb_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_t19x_sor2_control_protocol_dpa_f(void)
{
	return 0x600U;
}
static inline u32 nvdisp_t19x_sor2_control_protocol_dpb_f(void)
{
	return 0x700U;
}
static inline u32 nvdisp_t19x_sor2_control_protocol_custom_f(void)
{
	return 0xf00U;
}
static inline u32 nvdisp_t19x_sor3_control_r(void)
{
	return 0x00000406U;
}
static inline u32 nvdisp_t19x_sor3_control_protocol_tmdsa_f(void)
{
	return 0x100U;
}
static inline u32 nvdisp_t19x_sor3_control_protocol_tmdsb_f(void)
{
	return 0x200U;
}
static inline u32 nvdisp_t19x_sor3_control_protocol_dpa_f(void)
{
	return 0x600U;
}
static inline u32 nvdisp_t19x_sor3_control_protocol_dpb_f(void)
{
	return 0x700U;
}
static inline u32 nvdisp_t19x_sor3_control_protocol_custom_f(void)
{
	return 0xf00U;
}
static inline u32 nvdisp_t19x_rg_vsync_ptimer0_r(void)
{
	return 0x0000035cU;
}
static inline u32 nvdisp_t19x_rg_vsync_ptimer1_r(void)
{
	return 0x0000035dU;
}
static inline u32 nvdisp_t19x_state_access_r(void)
{
	return 0x00000040U;
}
static inline u32 nvdisp_t19x_state_access_write_mux_assembly_f(void)
{
	return 0x0U;
}
static inline u32 nvdisp_t19x_state_access_read_mux_active_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_t19x_display_cmd_option_r(void)
{
	return 0x00000031U;
}
static inline u32 nvdisp_t19x_display_cmd_option_msf_enable_enable_f(void)
{
	return 0x2U;
}
static inline u32 nvdisp_t19x_display_cmd_option_msf_src_glb_ctrl_f(void)
{
	return 0x4U;
}
static inline u32 nvdisp_t19x_display_command_r(void)
{
	return 0x00000032U;
}
static inline u32 nvdisp_t19x_display_command_control_mode_c_display_f(void)
{
	return 0x20U;
}
static inline u32 nvdisp_t19x_display_command_control_mode_nc_display_f(void)
{
	return 0x40U;
}
static inline u32 nvdisp_t19x_cmd_state_ctrl_r(void)
{
	return 0x00000041U;
}
static inline u32 nvdisp_t19x_cmd_state_ctrl_general_act_req_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvdisp_t19x_cmd_state_ctrl_host_trig_enable_f(void)
{
	return 0x1000000U;
}
static inline u32 nvdisp_t19x_rg_status_r(void)
{
	return 0x00000362U;
}
static inline u32 nvdisp_t19x_rg_status_stalled_yes_f(void)
{
	return 0x8U;
}
static inline u32 nvdisp_t19x_rg_status_unstall_force_even_set_enable_f(void)
{
	return 0x80000U;
}
static inline u32 nvdisp_t19x_glb_ctrl_r(void)
{
	return 0x0000003fU;
}
#endif
