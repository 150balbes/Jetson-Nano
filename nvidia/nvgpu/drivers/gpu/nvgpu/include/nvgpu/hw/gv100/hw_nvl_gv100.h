/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_nvl_gv100_h_
#define _hw_nvl_gv100_h_

static inline u32 nvl_link_state_r(void)
{
	return 0x00000000U;
}
static inline u32 nvl_link_state_state_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 nvl_link_state_state_m(void)
{
	return 0xffU << 0U;
}
static inline u32 nvl_link_state_state_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 nvl_link_state_state_init_v(void)
{
	return 0x00000000U;
}
static inline u32 nvl_link_state_state_init_f(void)
{
	return 0x0U;
}
static inline u32 nvl_link_state_state_hwcfg_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_state_state_hwcfg_f(void)
{
	return 0x1U;
}
static inline u32 nvl_link_state_state_swcfg_v(void)
{
	return 0x00000002U;
}
static inline u32 nvl_link_state_state_swcfg_f(void)
{
	return 0x2U;
}
static inline u32 nvl_link_state_state_active_v(void)
{
	return 0x00000003U;
}
static inline u32 nvl_link_state_state_active_f(void)
{
	return 0x3U;
}
static inline u32 nvl_link_state_state_fault_v(void)
{
	return 0x00000004U;
}
static inline u32 nvl_link_state_state_fault_f(void)
{
	return 0x4U;
}
static inline u32 nvl_link_state_state_rcvy_ac_v(void)
{
	return 0x00000008U;
}
static inline u32 nvl_link_state_state_rcvy_ac_f(void)
{
	return 0x8U;
}
static inline u32 nvl_link_state_state_rcvy_sw_v(void)
{
	return 0x00000009U;
}
static inline u32 nvl_link_state_state_rcvy_sw_f(void)
{
	return 0x9U;
}
static inline u32 nvl_link_state_state_rcvy_rx_v(void)
{
	return 0x0000000aU;
}
static inline u32 nvl_link_state_state_rcvy_rx_f(void)
{
	return 0xaU;
}
static inline u32 nvl_link_state_an0_busy_f(u32 v)
{
	return (v & 0x1U) << 12U;
}
static inline u32 nvl_link_state_an0_busy_m(void)
{
	return 0x1U << 12U;
}
static inline u32 nvl_link_state_an0_busy_v(u32 r)
{
	return (r >> 12U) & 0x1U;
}
static inline u32 nvl_link_state_tl_busy_f(u32 v)
{
	return (v & 0x1U) << 13U;
}
static inline u32 nvl_link_state_tl_busy_m(void)
{
	return 0x1U << 13U;
}
static inline u32 nvl_link_state_tl_busy_v(u32 r)
{
	return (r >> 13U) & 0x1U;
}
static inline u32 nvl_link_state_dbg_substate_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 nvl_link_state_dbg_substate_m(void)
{
	return 0xffffU << 16U;
}
static inline u32 nvl_link_state_dbg_substate_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 nvl_link_activity_r(void)
{
	return 0x0000000cU;
}
static inline u32 nvl_link_activity_blkact_f(u32 v)
{
	return (v & 0x7U) << 0U;
}
static inline u32 nvl_link_activity_blkact_m(void)
{
	return 0x7U << 0U;
}
static inline u32 nvl_link_activity_blkact_v(u32 r)
{
	return (r >> 0U) & 0x7U;
}
static inline u32 nvl_sublink_activity_r(u32 i)
{
	return 0x00000010U + i*4U;
}
static inline u32 nvl_sublink_activity_blkact0_f(u32 v)
{
	return (v & 0x7U) << 0U;
}
static inline u32 nvl_sublink_activity_blkact0_m(void)
{
	return 0x7U << 0U;
}
static inline u32 nvl_sublink_activity_blkact0_v(u32 r)
{
	return (r >> 0U) & 0x7U;
}
static inline u32 nvl_sublink_activity_blkact1_f(u32 v)
{
	return (v & 0x7U) << 8U;
}
static inline u32 nvl_sublink_activity_blkact1_m(void)
{
	return 0x7U << 8U;
}
static inline u32 nvl_sublink_activity_blkact1_v(u32 r)
{
	return (r >> 8U) & 0x7U;
}
static inline u32 nvl_link_config_r(void)
{
	return 0x00000018U;
}
static inline u32 nvl_link_config_ac_safe_en_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 nvl_link_config_ac_safe_en_m(void)
{
	return 0x1U << 30U;
}
static inline u32 nvl_link_config_ac_safe_en_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 nvl_link_config_ac_safe_en_on_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_config_ac_safe_en_on_f(void)
{
	return 0x40000000U;
}
static inline u32 nvl_link_config_link_en_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 nvl_link_config_link_en_m(void)
{
	return 0x1U << 31U;
}
static inline u32 nvl_link_config_link_en_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvl_link_config_link_en_on_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_config_link_en_on_f(void)
{
	return 0x80000000U;
}
static inline u32 nvl_link_change_r(void)
{
	return 0x00000040U;
}
static inline u32 nvl_link_change_oldstate_mask_f(u32 v)
{
	return (v & 0xfU) << 16U;
}
static inline u32 nvl_link_change_oldstate_mask_m(void)
{
	return 0xfU << 16U;
}
static inline u32 nvl_link_change_oldstate_mask_v(u32 r)
{
	return (r >> 16U) & 0xfU;
}
static inline u32 nvl_link_change_oldstate_mask_dontcare_v(void)
{
	return 0x0000000fU;
}
static inline u32 nvl_link_change_oldstate_mask_dontcare_f(void)
{
	return 0xf0000U;
}
static inline u32 nvl_link_change_newstate_f(u32 v)
{
	return (v & 0xfU) << 4U;
}
static inline u32 nvl_link_change_newstate_m(void)
{
	return 0xfU << 4U;
}
static inline u32 nvl_link_change_newstate_v(u32 r)
{
	return (r >> 4U) & 0xfU;
}
static inline u32 nvl_link_change_newstate_hwcfg_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_change_newstate_hwcfg_f(void)
{
	return 0x10U;
}
static inline u32 nvl_link_change_newstate_swcfg_v(void)
{
	return 0x00000002U;
}
static inline u32 nvl_link_change_newstate_swcfg_f(void)
{
	return 0x20U;
}
static inline u32 nvl_link_change_newstate_active_v(void)
{
	return 0x00000003U;
}
static inline u32 nvl_link_change_newstate_active_f(void)
{
	return 0x30U;
}
static inline u32 nvl_link_change_action_f(u32 v)
{
	return (v & 0x3U) << 2U;
}
static inline u32 nvl_link_change_action_m(void)
{
	return 0x3U << 2U;
}
static inline u32 nvl_link_change_action_v(u32 r)
{
	return (r >> 2U) & 0x3U;
}
static inline u32 nvl_link_change_action_ltssm_change_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_change_action_ltssm_change_f(void)
{
	return 0x4U;
}
static inline u32 nvl_link_change_status_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 nvl_link_change_status_m(void)
{
	return 0x3U << 0U;
}
static inline u32 nvl_link_change_status_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 nvl_link_change_status_done_v(void)
{
	return 0x00000000U;
}
static inline u32 nvl_link_change_status_done_f(void)
{
	return 0x0U;
}
static inline u32 nvl_link_change_status_busy_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_change_status_busy_f(void)
{
	return 0x1U;
}
static inline u32 nvl_link_change_status_fault_v(void)
{
	return 0x00000002U;
}
static inline u32 nvl_link_change_status_fault_f(void)
{
	return 0x2U;
}
static inline u32 nvl_sublink_change_r(void)
{
	return 0x00000044U;
}
static inline u32 nvl_sublink_change_countdown_f(u32 v)
{
	return (v & 0xfffU) << 20U;
}
static inline u32 nvl_sublink_change_countdown_m(void)
{
	return 0xfffU << 20U;
}
static inline u32 nvl_sublink_change_countdown_v(u32 r)
{
	return (r >> 20U) & 0xfffU;
}
static inline u32 nvl_sublink_change_oldstate_mask_f(u32 v)
{
	return (v & 0xfU) << 16U;
}
static inline u32 nvl_sublink_change_oldstate_mask_m(void)
{
	return 0xfU << 16U;
}
static inline u32 nvl_sublink_change_oldstate_mask_v(u32 r)
{
	return (r >> 16U) & 0xfU;
}
static inline u32 nvl_sublink_change_oldstate_mask_dontcare_v(void)
{
	return 0x0000000fU;
}
static inline u32 nvl_sublink_change_oldstate_mask_dontcare_f(void)
{
	return 0xf0000U;
}
static inline u32 nvl_sublink_change_sublink_f(u32 v)
{
	return (v & 0xfU) << 12U;
}
static inline u32 nvl_sublink_change_sublink_m(void)
{
	return 0xfU << 12U;
}
static inline u32 nvl_sublink_change_sublink_v(u32 r)
{
	return (r >> 12U) & 0xfU;
}
static inline u32 nvl_sublink_change_sublink_tx_v(void)
{
	return 0x00000000U;
}
static inline u32 nvl_sublink_change_sublink_tx_f(void)
{
	return 0x0U;
}
static inline u32 nvl_sublink_change_sublink_rx_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_sublink_change_sublink_rx_f(void)
{
	return 0x1000U;
}
static inline u32 nvl_sublink_change_newstate_f(u32 v)
{
	return (v & 0xfU) << 4U;
}
static inline u32 nvl_sublink_change_newstate_m(void)
{
	return 0xfU << 4U;
}
static inline u32 nvl_sublink_change_newstate_v(u32 r)
{
	return (r >> 4U) & 0xfU;
}
static inline u32 nvl_sublink_change_newstate_hs_v(void)
{
	return 0x00000000U;
}
static inline u32 nvl_sublink_change_newstate_hs_f(void)
{
	return 0x0U;
}
static inline u32 nvl_sublink_change_newstate_eighth_v(void)
{
	return 0x00000004U;
}
static inline u32 nvl_sublink_change_newstate_eighth_f(void)
{
	return 0x40U;
}
static inline u32 nvl_sublink_change_newstate_train_v(void)
{
	return 0x00000005U;
}
static inline u32 nvl_sublink_change_newstate_train_f(void)
{
	return 0x50U;
}
static inline u32 nvl_sublink_change_newstate_safe_v(void)
{
	return 0x00000006U;
}
static inline u32 nvl_sublink_change_newstate_safe_f(void)
{
	return 0x60U;
}
static inline u32 nvl_sublink_change_newstate_off_v(void)
{
	return 0x00000007U;
}
static inline u32 nvl_sublink_change_newstate_off_f(void)
{
	return 0x70U;
}
static inline u32 nvl_sublink_change_action_f(u32 v)
{
	return (v & 0x3U) << 2U;
}
static inline u32 nvl_sublink_change_action_m(void)
{
	return 0x3U << 2U;
}
static inline u32 nvl_sublink_change_action_v(u32 r)
{
	return (r >> 2U) & 0x3U;
}
static inline u32 nvl_sublink_change_action_slsm_change_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_sublink_change_action_slsm_change_f(void)
{
	return 0x4U;
}
static inline u32 nvl_sublink_change_status_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 nvl_sublink_change_status_m(void)
{
	return 0x3U << 0U;
}
static inline u32 nvl_sublink_change_status_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 nvl_sublink_change_status_done_v(void)
{
	return 0x00000000U;
}
static inline u32 nvl_sublink_change_status_done_f(void)
{
	return 0x0U;
}
static inline u32 nvl_sublink_change_status_busy_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_sublink_change_status_busy_f(void)
{
	return 0x1U;
}
static inline u32 nvl_sublink_change_status_fault_v(void)
{
	return 0x00000002U;
}
static inline u32 nvl_sublink_change_status_fault_f(void)
{
	return 0x2U;
}
static inline u32 nvl_link_test_r(void)
{
	return 0x00000048U;
}
static inline u32 nvl_link_test_mode_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 nvl_link_test_mode_m(void)
{
	return 0x1U << 0U;
}
static inline u32 nvl_link_test_mode_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvl_link_test_mode_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_test_mode_enable_f(void)
{
	return 0x1U;
}
static inline u32 nvl_link_test_auto_hwcfg_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 nvl_link_test_auto_hwcfg_m(void)
{
	return 0x1U << 30U;
}
static inline u32 nvl_link_test_auto_hwcfg_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 nvl_link_test_auto_hwcfg_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_test_auto_hwcfg_enable_f(void)
{
	return 0x40000000U;
}
static inline u32 nvl_link_test_auto_nvhs_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 nvl_link_test_auto_nvhs_m(void)
{
	return 0x1U << 31U;
}
static inline u32 nvl_link_test_auto_nvhs_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvl_link_test_auto_nvhs_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_link_test_auto_nvhs_enable_f(void)
{
	return 0x80000000U;
}
static inline u32 nvl_sl0_slsm_status_tx_r(void)
{
	return 0x00002024U;
}
static inline u32 nvl_sl0_slsm_status_tx_substate_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 nvl_sl0_slsm_status_tx_substate_m(void)
{
	return 0xfU << 0U;
}
static inline u32 nvl_sl0_slsm_status_tx_substate_v(u32 r)
{
	return (r >> 0U) & 0xfU;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_f(u32 v)
{
	return (v & 0xfU) << 4U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_m(void)
{
	return 0xfU << 4U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_v(u32 r)
{
	return (r >> 4U) & 0xfU;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_hs_v(void)
{
	return 0x00000000U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_hs_f(void)
{
	return 0x0U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_eighth_v(void)
{
	return 0x00000004U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_eighth_f(void)
{
	return 0x40U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_train_v(void)
{
	return 0x00000005U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_train_f(void)
{
	return 0x50U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_off_v(void)
{
	return 0x00000007U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_off_f(void)
{
	return 0x70U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_safe_v(void)
{
	return 0x00000006U;
}
static inline u32 nvl_sl0_slsm_status_tx_primary_state_safe_f(void)
{
	return 0x60U;
}
static inline u32 nvl_sl1_slsm_status_rx_r(void)
{
	return 0x00003014U;
}
static inline u32 nvl_sl1_slsm_status_rx_substate_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 nvl_sl1_slsm_status_rx_substate_m(void)
{
	return 0xfU << 0U;
}
static inline u32 nvl_sl1_slsm_status_rx_substate_v(u32 r)
{
	return (r >> 0U) & 0xfU;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_f(u32 v)
{
	return (v & 0xfU) << 4U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_m(void)
{
	return 0xfU << 4U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_v(u32 r)
{
	return (r >> 4U) & 0xfU;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_hs_v(void)
{
	return 0x00000000U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_hs_f(void)
{
	return 0x0U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_eighth_v(void)
{
	return 0x00000004U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_eighth_f(void)
{
	return 0x40U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_train_v(void)
{
	return 0x00000005U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_train_f(void)
{
	return 0x50U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_off_v(void)
{
	return 0x00000007U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_off_f(void)
{
	return 0x70U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_safe_v(void)
{
	return 0x00000006U;
}
static inline u32 nvl_sl1_slsm_status_rx_primary_state_safe_f(void)
{
	return 0x60U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_r(void)
{
	return 0x00002008U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_init_f(u32 v)
{
	return (v & 0x7ffU) << 0U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_init_m(void)
{
	return 0x7ffU << 0U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_init_v(u32 r)
{
	return (r >> 0U) & 0x7ffU;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_init_init_v(void)
{
	return 0x00000728U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_init_init_f(void)
{
	return 0x728U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_initscl_f(u32 v)
{
	return (v & 0x1fU) << 11U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_initscl_m(void)
{
	return 0x1fU << 11U;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_initscl_v(u32 r)
{
	return (r >> 11U) & 0x1fU;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_initscl_init_v(void)
{
	return 0x0000000fU;
}
static inline u32 nvl_sl0_safe_ctrl2_tx_ctr_initscl_init_f(void)
{
	return 0x7800U;
}
static inline u32 nvl_sl1_error_rate_ctrl_r(void)
{
	return 0x00003284U;
}
static inline u32 nvl_sl1_error_rate_ctrl_short_threshold_man_f(u32 v)
{
	return (v & 0x7U) << 0U;
}
static inline u32 nvl_sl1_error_rate_ctrl_short_threshold_man_m(void)
{
	return 0x7U << 0U;
}
static inline u32 nvl_sl1_error_rate_ctrl_short_threshold_man_v(u32 r)
{
	return (r >> 0U) & 0x7U;
}
static inline u32 nvl_sl1_error_rate_ctrl_long_threshold_man_f(u32 v)
{
	return (v & 0x7U) << 16U;
}
static inline u32 nvl_sl1_error_rate_ctrl_long_threshold_man_m(void)
{
	return 0x7U << 16U;
}
static inline u32 nvl_sl1_error_rate_ctrl_long_threshold_man_v(u32 r)
{
	return (r >> 16U) & 0x7U;
}
static inline u32 nvl_sl1_rxslsm_timeout_2_r(void)
{
	return 0x00003034U;
}
static inline u32 nvl_txiobist_configreg_r(void)
{
	return 0x00002e14U;
}
static inline u32 nvl_txiobist_configreg_io_bist_mode_in_f(u32 v)
{
	return (v & 0x1U) << 17U;
}
static inline u32 nvl_txiobist_configreg_io_bist_mode_in_m(void)
{
	return 0x1U << 17U;
}
static inline u32 nvl_txiobist_configreg_io_bist_mode_in_v(u32 r)
{
	return (r >> 17U) & 0x1U;
}
static inline u32 nvl_txiobist_config_r(void)
{
	return 0x00002e10U;
}
static inline u32 nvl_txiobist_config_dpg_prbsseedld_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvl_txiobist_config_dpg_prbsseedld_m(void)
{
	return 0x1U << 2U;
}
static inline u32 nvl_txiobist_config_dpg_prbsseedld_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 nvl_intr_r(void)
{
	return 0x00000050U;
}
static inline u32 nvl_intr_tx_replay_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 nvl_intr_tx_replay_m(void)
{
	return 0x1U << 0U;
}
static inline u32 nvl_intr_tx_replay_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvl_intr_tx_recovery_short_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvl_intr_tx_recovery_short_m(void)
{
	return 0x1U << 1U;
}
static inline u32 nvl_intr_tx_recovery_short_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvl_intr_tx_recovery_long_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvl_intr_tx_recovery_long_m(void)
{
	return 0x1U << 2U;
}
static inline u32 nvl_intr_tx_recovery_long_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 nvl_intr_tx_fault_ram_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 nvl_intr_tx_fault_ram_m(void)
{
	return 0x1U << 4U;
}
static inline u32 nvl_intr_tx_fault_ram_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 nvl_intr_tx_fault_interface_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 nvl_intr_tx_fault_interface_m(void)
{
	return 0x1U << 5U;
}
static inline u32 nvl_intr_tx_fault_interface_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 nvl_intr_tx_fault_sublink_change_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 nvl_intr_tx_fault_sublink_change_m(void)
{
	return 0x1U << 8U;
}
static inline u32 nvl_intr_tx_fault_sublink_change_v(u32 r)
{
	return (r >> 8U) & 0x1U;
}
static inline u32 nvl_intr_rx_fault_sublink_change_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 nvl_intr_rx_fault_sublink_change_m(void)
{
	return 0x1U << 16U;
}
static inline u32 nvl_intr_rx_fault_sublink_change_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 nvl_intr_rx_fault_dl_protocol_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 nvl_intr_rx_fault_dl_protocol_m(void)
{
	return 0x1U << 20U;
}
static inline u32 nvl_intr_rx_fault_dl_protocol_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 nvl_intr_rx_short_error_rate_f(u32 v)
{
	return (v & 0x1U) << 21U;
}
static inline u32 nvl_intr_rx_short_error_rate_m(void)
{
	return 0x1U << 21U;
}
static inline u32 nvl_intr_rx_short_error_rate_v(u32 r)
{
	return (r >> 21U) & 0x1U;
}
static inline u32 nvl_intr_rx_long_error_rate_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 nvl_intr_rx_long_error_rate_m(void)
{
	return 0x1U << 22U;
}
static inline u32 nvl_intr_rx_long_error_rate_v(u32 r)
{
	return (r >> 22U) & 0x1U;
}
static inline u32 nvl_intr_rx_ila_trigger_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 nvl_intr_rx_ila_trigger_m(void)
{
	return 0x1U << 23U;
}
static inline u32 nvl_intr_rx_ila_trigger_v(u32 r)
{
	return (r >> 23U) & 0x1U;
}
static inline u32 nvl_intr_rx_crc_counter_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 nvl_intr_rx_crc_counter_m(void)
{
	return 0x1U << 24U;
}
static inline u32 nvl_intr_rx_crc_counter_v(u32 r)
{
	return (r >> 24U) & 0x1U;
}
static inline u32 nvl_intr_ltssm_fault_f(u32 v)
{
	return (v & 0x1U) << 28U;
}
static inline u32 nvl_intr_ltssm_fault_m(void)
{
	return 0x1U << 28U;
}
static inline u32 nvl_intr_ltssm_fault_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 nvl_intr_ltssm_protocol_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 nvl_intr_ltssm_protocol_m(void)
{
	return 0x1U << 29U;
}
static inline u32 nvl_intr_ltssm_protocol_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 nvl_intr_minion_request_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 nvl_intr_minion_request_m(void)
{
	return 0x1U << 30U;
}
static inline u32 nvl_intr_minion_request_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 nvl_intr_sw2_r(void)
{
	return 0x00000054U;
}
static inline u32 nvl_intr_minion_r(void)
{
	return 0x00000060U;
}
static inline u32 nvl_intr_minion_tx_replay_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 nvl_intr_minion_tx_replay_m(void)
{
	return 0x1U << 0U;
}
static inline u32 nvl_intr_minion_tx_replay_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvl_intr_minion_tx_recovery_short_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvl_intr_minion_tx_recovery_short_m(void)
{
	return 0x1U << 1U;
}
static inline u32 nvl_intr_minion_tx_recovery_short_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvl_intr_minion_tx_recovery_long_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvl_intr_minion_tx_recovery_long_m(void)
{
	return 0x1U << 2U;
}
static inline u32 nvl_intr_minion_tx_recovery_long_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 nvl_intr_minion_tx_fault_ram_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 nvl_intr_minion_tx_fault_ram_m(void)
{
	return 0x1U << 4U;
}
static inline u32 nvl_intr_minion_tx_fault_ram_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 nvl_intr_minion_tx_fault_interface_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 nvl_intr_minion_tx_fault_interface_m(void)
{
	return 0x1U << 5U;
}
static inline u32 nvl_intr_minion_tx_fault_interface_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 nvl_intr_minion_tx_fault_sublink_change_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 nvl_intr_minion_tx_fault_sublink_change_m(void)
{
	return 0x1U << 8U;
}
static inline u32 nvl_intr_minion_tx_fault_sublink_change_v(u32 r)
{
	return (r >> 8U) & 0x1U;
}
static inline u32 nvl_intr_minion_rx_fault_sublink_change_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 nvl_intr_minion_rx_fault_sublink_change_m(void)
{
	return 0x1U << 16U;
}
static inline u32 nvl_intr_minion_rx_fault_sublink_change_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 nvl_intr_minion_rx_fault_dl_protocol_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 nvl_intr_minion_rx_fault_dl_protocol_m(void)
{
	return 0x1U << 20U;
}
static inline u32 nvl_intr_minion_rx_fault_dl_protocol_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 nvl_intr_minion_rx_short_error_rate_f(u32 v)
{
	return (v & 0x1U) << 21U;
}
static inline u32 nvl_intr_minion_rx_short_error_rate_m(void)
{
	return 0x1U << 21U;
}
static inline u32 nvl_intr_minion_rx_short_error_rate_v(u32 r)
{
	return (r >> 21U) & 0x1U;
}
static inline u32 nvl_intr_minion_rx_long_error_rate_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 nvl_intr_minion_rx_long_error_rate_m(void)
{
	return 0x1U << 22U;
}
static inline u32 nvl_intr_minion_rx_long_error_rate_v(u32 r)
{
	return (r >> 22U) & 0x1U;
}
static inline u32 nvl_intr_minion_rx_ila_trigger_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 nvl_intr_minion_rx_ila_trigger_m(void)
{
	return 0x1U << 23U;
}
static inline u32 nvl_intr_minion_rx_ila_trigger_v(u32 r)
{
	return (r >> 23U) & 0x1U;
}
static inline u32 nvl_intr_minion_rx_crc_counter_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 nvl_intr_minion_rx_crc_counter_m(void)
{
	return 0x1U << 24U;
}
static inline u32 nvl_intr_minion_rx_crc_counter_v(u32 r)
{
	return (r >> 24U) & 0x1U;
}
static inline u32 nvl_intr_minion_ltssm_fault_f(u32 v)
{
	return (v & 0x1U) << 28U;
}
static inline u32 nvl_intr_minion_ltssm_fault_m(void)
{
	return 0x1U << 28U;
}
static inline u32 nvl_intr_minion_ltssm_fault_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 nvl_intr_minion_ltssm_protocol_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 nvl_intr_minion_ltssm_protocol_m(void)
{
	return 0x1U << 29U;
}
static inline u32 nvl_intr_minion_ltssm_protocol_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 nvl_intr_minion_minion_request_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 nvl_intr_minion_minion_request_m(void)
{
	return 0x1U << 30U;
}
static inline u32 nvl_intr_minion_minion_request_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 nvl_intr_nonstall_en_r(void)
{
	return 0x0000005cU;
}
static inline u32 nvl_intr_stall_en_r(void)
{
	return 0x00000058U;
}
static inline u32 nvl_intr_stall_en_tx_replay_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 nvl_intr_stall_en_tx_replay_m(void)
{
	return 0x1U << 0U;
}
static inline u32 nvl_intr_stall_en_tx_replay_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_short_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_short_m(void)
{
	return 0x1U << 1U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_short_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_short_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_short_enable_f(void)
{
	return 0x2U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_long_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_long_m(void)
{
	return 0x1U << 2U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_long_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_long_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_tx_recovery_long_enable_f(void)
{
	return 0x4U;
}
static inline u32 nvl_intr_stall_en_tx_fault_ram_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 nvl_intr_stall_en_tx_fault_ram_m(void)
{
	return 0x1U << 4U;
}
static inline u32 nvl_intr_stall_en_tx_fault_ram_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_tx_fault_ram_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_tx_fault_ram_enable_f(void)
{
	return 0x10U;
}
static inline u32 nvl_intr_stall_en_tx_fault_interface_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 nvl_intr_stall_en_tx_fault_interface_m(void)
{
	return 0x1U << 5U;
}
static inline u32 nvl_intr_stall_en_tx_fault_interface_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_tx_fault_interface_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_tx_fault_interface_enable_f(void)
{
	return 0x20U;
}
static inline u32 nvl_intr_stall_en_tx_fault_sublink_change_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 nvl_intr_stall_en_tx_fault_sublink_change_m(void)
{
	return 0x1U << 8U;
}
static inline u32 nvl_intr_stall_en_tx_fault_sublink_change_v(u32 r)
{
	return (r >> 8U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_tx_fault_sublink_change_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_tx_fault_sublink_change_enable_f(void)
{
	return 0x100U;
}
static inline u32 nvl_intr_stall_en_rx_fault_sublink_change_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 nvl_intr_stall_en_rx_fault_sublink_change_m(void)
{
	return 0x1U << 16U;
}
static inline u32 nvl_intr_stall_en_rx_fault_sublink_change_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_rx_fault_sublink_change_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_rx_fault_sublink_change_enable_f(void)
{
	return 0x10000U;
}
static inline u32 nvl_intr_stall_en_rx_fault_dl_protocol_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 nvl_intr_stall_en_rx_fault_dl_protocol_m(void)
{
	return 0x1U << 20U;
}
static inline u32 nvl_intr_stall_en_rx_fault_dl_protocol_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_rx_fault_dl_protocol_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_rx_fault_dl_protocol_enable_f(void)
{
	return 0x100000U;
}
static inline u32 nvl_intr_stall_en_rx_short_error_rate_f(u32 v)
{
	return (v & 0x1U) << 21U;
}
static inline u32 nvl_intr_stall_en_rx_short_error_rate_m(void)
{
	return 0x1U << 21U;
}
static inline u32 nvl_intr_stall_en_rx_short_error_rate_v(u32 r)
{
	return (r >> 21U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_rx_short_error_rate_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_rx_short_error_rate_enable_f(void)
{
	return 0x200000U;
}
static inline u32 nvl_intr_stall_en_rx_long_error_rate_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 nvl_intr_stall_en_rx_long_error_rate_m(void)
{
	return 0x1U << 22U;
}
static inline u32 nvl_intr_stall_en_rx_long_error_rate_v(u32 r)
{
	return (r >> 22U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_rx_long_error_rate_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_rx_long_error_rate_enable_f(void)
{
	return 0x400000U;
}
static inline u32 nvl_intr_stall_en_rx_ila_trigger_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 nvl_intr_stall_en_rx_ila_trigger_m(void)
{
	return 0x1U << 23U;
}
static inline u32 nvl_intr_stall_en_rx_ila_trigger_v(u32 r)
{
	return (r >> 23U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_rx_ila_trigger_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_rx_ila_trigger_enable_f(void)
{
	return 0x800000U;
}
static inline u32 nvl_intr_stall_en_rx_crc_counter_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 nvl_intr_stall_en_rx_crc_counter_m(void)
{
	return 0x1U << 24U;
}
static inline u32 nvl_intr_stall_en_rx_crc_counter_v(u32 r)
{
	return (r >> 24U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_rx_crc_counter_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_rx_crc_counter_enable_f(void)
{
	return 0x1000000U;
}
static inline u32 nvl_intr_stall_en_ltssm_fault_f(u32 v)
{
	return (v & 0x1U) << 28U;
}
static inline u32 nvl_intr_stall_en_ltssm_fault_m(void)
{
	return 0x1U << 28U;
}
static inline u32 nvl_intr_stall_en_ltssm_fault_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_ltssm_fault_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_ltssm_fault_enable_f(void)
{
	return 0x10000000U;
}
static inline u32 nvl_intr_stall_en_ltssm_protocol_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 nvl_intr_stall_en_ltssm_protocol_m(void)
{
	return 0x1U << 29U;
}
static inline u32 nvl_intr_stall_en_ltssm_protocol_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_ltssm_protocol_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_ltssm_protocol_enable_f(void)
{
	return 0x20000000U;
}
static inline u32 nvl_intr_stall_en_minion_request_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 nvl_intr_stall_en_minion_request_m(void)
{
	return 0x1U << 30U;
}
static inline u32 nvl_intr_stall_en_minion_request_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 nvl_intr_stall_en_minion_request_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_intr_stall_en_minion_request_enable_f(void)
{
	return 0x40000000U;
}
static inline u32 nvl_br0_cfg_cal_r(void)
{
	return 0x0000281cU;
}
static inline u32 nvl_br0_cfg_cal_rxcal_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 nvl_br0_cfg_cal_rxcal_m(void)
{
	return 0x1U << 0U;
}
static inline u32 nvl_br0_cfg_cal_rxcal_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvl_br0_cfg_cal_rxcal_on_v(void)
{
	return 0x00000001U;
}
static inline u32 nvl_br0_cfg_cal_rxcal_on_f(void)
{
	return 0x1U;
}
static inline u32 nvl_br0_cfg_status_cal_r(void)
{
	return 0x00002838U;
}
static inline u32 nvl_br0_cfg_status_cal_rxcal_done_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvl_br0_cfg_status_cal_rxcal_done_m(void)
{
	return 0x1U << 2U;
}
static inline u32 nvl_br0_cfg_status_cal_rxcal_done_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
#endif
