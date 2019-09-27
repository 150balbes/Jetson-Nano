/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_therm_gv11b_h_
#define _hw_therm_gv11b_h_

static inline u32 therm_use_a_r(void)
{
	return 0x00020798U;
}
static inline u32 therm_use_a_ext_therm_0_enable_f(void)
{
	return 0x1U;
}
static inline u32 therm_use_a_ext_therm_1_enable_f(void)
{
	return 0x2U;
}
static inline u32 therm_use_a_ext_therm_2_enable_f(void)
{
	return 0x4U;
}
static inline u32 therm_evt_ext_therm_0_r(void)
{
	return 0x00020700U;
}
static inline u32 therm_evt_ext_therm_0_slow_factor_f(u32 v)
{
	return (v & 0x3fU) << 24U;
}
static inline u32 therm_evt_ext_therm_0_slow_factor_init_v(void)
{
	return 0x00000001U;
}
static inline u32 therm_evt_ext_therm_0_mode_f(u32 v)
{
	return (v & 0x3U) << 30U;
}
static inline u32 therm_evt_ext_therm_0_mode_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 therm_evt_ext_therm_0_mode_inverted_v(void)
{
	return 0x00000001U;
}
static inline u32 therm_evt_ext_therm_0_mode_forced_v(void)
{
	return 0x00000002U;
}
static inline u32 therm_evt_ext_therm_0_mode_cleared_v(void)
{
	return 0x00000003U;
}
static inline u32 therm_evt_ext_therm_1_r(void)
{
	return 0x00020704U;
}
static inline u32 therm_evt_ext_therm_1_slow_factor_f(u32 v)
{
	return (v & 0x3fU) << 24U;
}
static inline u32 therm_evt_ext_therm_1_slow_factor_init_v(void)
{
	return 0x00000002U;
}
static inline u32 therm_evt_ext_therm_1_mode_f(u32 v)
{
	return (v & 0x3U) << 30U;
}
static inline u32 therm_evt_ext_therm_1_mode_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 therm_evt_ext_therm_1_mode_inverted_v(void)
{
	return 0x00000001U;
}
static inline u32 therm_evt_ext_therm_1_mode_forced_v(void)
{
	return 0x00000002U;
}
static inline u32 therm_evt_ext_therm_1_mode_cleared_v(void)
{
	return 0x00000003U;
}
static inline u32 therm_evt_ext_therm_2_r(void)
{
	return 0x00020708U;
}
static inline u32 therm_evt_ext_therm_2_slow_factor_f(u32 v)
{
	return (v & 0x3fU) << 24U;
}
static inline u32 therm_evt_ext_therm_2_slow_factor_init_v(void)
{
	return 0x00000003U;
}
static inline u32 therm_evt_ext_therm_2_mode_f(u32 v)
{
	return (v & 0x3U) << 30U;
}
static inline u32 therm_evt_ext_therm_2_mode_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 therm_evt_ext_therm_2_mode_inverted_v(void)
{
	return 0x00000001U;
}
static inline u32 therm_evt_ext_therm_2_mode_forced_v(void)
{
	return 0x00000002U;
}
static inline u32 therm_evt_ext_therm_2_mode_cleared_v(void)
{
	return 0x00000003U;
}
static inline u32 therm_weight_1_r(void)
{
	return 0x00020024U;
}
static inline u32 therm_config1_r(void)
{
	return 0x00020050U;
}
static inline u32 therm_config2_r(void)
{
	return 0x00020130U;
}
static inline u32 therm_config2_grad_step_duration_f(u32 v)
{
	return (v & 0xfU) << 8U;
}
static inline u32 therm_config2_grad_step_duration_m(void)
{
	return 0xfU << 8U;
}
static inline u32 therm_config2_slowdown_factor_extended_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 therm_config2_grad_enable_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 therm_gate_ctrl_r(u32 i)
{
	return 0x00020200U + i*4U;
}
static inline u32 therm_gate_ctrl_eng_clk_m(void)
{
	return 0x3U << 0U;
}
static inline u32 therm_gate_ctrl_eng_clk_run_f(void)
{
	return 0x0U;
}
static inline u32 therm_gate_ctrl_eng_clk_auto_f(void)
{
	return 0x1U;
}
static inline u32 therm_gate_ctrl_eng_clk_stop_f(void)
{
	return 0x2U;
}
static inline u32 therm_gate_ctrl_blk_clk_m(void)
{
	return 0x3U << 2U;
}
static inline u32 therm_gate_ctrl_blk_clk_run_f(void)
{
	return 0x0U;
}
static inline u32 therm_gate_ctrl_blk_clk_auto_f(void)
{
	return 0x4U;
}
static inline u32 therm_gate_ctrl_idle_holdoff_m(void)
{
	return 0x1U << 4U;
}
static inline u32 therm_gate_ctrl_idle_holdoff_off_f(void)
{
	return 0x0U;
}
static inline u32 therm_gate_ctrl_idle_holdoff_on_f(void)
{
	return 0x10U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_exp_f(u32 v)
{
	return (v & 0x1fU) << 8U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_exp_m(void)
{
	return 0x1fU << 8U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_exp__prod_f(void)
{
	return 0x200U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_mant_f(u32 v)
{
	return (v & 0x7U) << 13U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_mant_m(void)
{
	return 0x7U << 13U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_mant__prod_f(void)
{
	return 0x2000U;
}
static inline u32 therm_gate_ctrl_eng_delay_before_f(u32 v)
{
	return (v & 0xfU) << 16U;
}
static inline u32 therm_gate_ctrl_eng_delay_before_m(void)
{
	return 0xfU << 16U;
}
static inline u32 therm_gate_ctrl_eng_delay_before__prod_f(void)
{
	return 0x40000U;
}
static inline u32 therm_gate_ctrl_eng_delay_after_f(u32 v)
{
	return (v & 0xfU) << 20U;
}
static inline u32 therm_gate_ctrl_eng_delay_after_m(void)
{
	return 0xfU << 20U;
}
static inline u32 therm_gate_ctrl_eng_delay_after__prod_f(void)
{
	return 0x0U;
}
static inline u32 therm_fecs_idle_filter_r(void)
{
	return 0x00020288U;
}
static inline u32 therm_fecs_idle_filter_value_m(void)
{
	return 0xffffffffU << 0U;
}
static inline u32 therm_fecs_idle_filter_value__prod_f(void)
{
	return 0x0U;
}
static inline u32 therm_hubmmu_idle_filter_r(void)
{
	return 0x0002028cU;
}
static inline u32 therm_hubmmu_idle_filter_value_m(void)
{
	return 0xffffffffU << 0U;
}
static inline u32 therm_hubmmu_idle_filter_value__prod_f(void)
{
	return 0x0U;
}
static inline u32 therm_clk_slowdown_r(u32 i)
{
	return 0x00020160U + i*4U;
}
static inline u32 therm_clk_slowdown_idle_factor_f(u32 v)
{
	return (v & 0x3fU) << 16U;
}
static inline u32 therm_clk_slowdown_idle_factor_m(void)
{
	return 0x3fU << 16U;
}
static inline u32 therm_clk_slowdown_idle_factor_v(u32 r)
{
	return (r >> 16U) & 0x3fU;
}
static inline u32 therm_clk_slowdown_idle_factor_disabled_f(void)
{
	return 0x0U;
}
static inline u32 therm_clk_slowdown_2_r(u32 i)
{
	return 0x000201a0U + i*4U;
}
static inline u32 therm_clk_slowdown_2_idle_condition_a_select_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 therm_clk_slowdown_2_idle_condition_a_type_f(u32 v)
{
	return (v & 0x7U) << 4U;
}
static inline u32 therm_clk_slowdown_2_idle_condition_a_type_v(u32 r)
{
	return (r >> 4U) & 0x7U;
}
static inline u32 therm_clk_slowdown_2_idle_condition_a_type_never_f(void)
{
	return 0x40U;
}
static inline u32 therm_clk_slowdown_2_idle_condition_b_type_f(u32 v)
{
	return (v & 0x7U) << 12U;
}
static inline u32 therm_clk_slowdown_2_idle_condition_b_type_v(u32 r)
{
	return (r >> 12U) & 0x7U;
}
static inline u32 therm_clk_slowdown_2_idle_condition_b_type_never_f(void)
{
	return 0x4000U;
}
static inline u32 therm_grad_stepping_table_r(u32 i)
{
	return 0x000202c8U + i*4U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_f(u32 v)
{
	return (v & 0x3fU) << 0U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_m(void)
{
	return 0x3fU << 0U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_fpdiv_by1_f(void)
{
	return 0x0U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_fpdiv_by1p5_f(void)
{
	return 0x1U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_fpdiv_by2_f(void)
{
	return 0x2U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_fpdiv_by4_f(void)
{
	return 0x6U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f(void)
{
	return 0xeU;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_fpdiv_by16_f(void)
{
	return 0x1eU;
}
static inline u32 therm_grad_stepping_table_slowdown_factor0_fpdiv_by32_f(void)
{
	return 0x3eU;
}
static inline u32 therm_grad_stepping_table_slowdown_factor1_f(u32 v)
{
	return (v & 0x3fU) << 6U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor1_m(void)
{
	return 0x3fU << 6U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor2_f(u32 v)
{
	return (v & 0x3fU) << 12U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor2_m(void)
{
	return 0x3fU << 12U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor3_f(u32 v)
{
	return (v & 0x3fU) << 18U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor3_m(void)
{
	return 0x3fU << 18U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor4_f(u32 v)
{
	return (v & 0x3fU) << 24U;
}
static inline u32 therm_grad_stepping_table_slowdown_factor4_m(void)
{
	return 0x3fU << 24U;
}
static inline u32 therm_grad_stepping0_r(void)
{
	return 0x000202c0U;
}
static inline u32 therm_grad_stepping0_feature_s(void)
{
	return 1U;
}
static inline u32 therm_grad_stepping0_feature_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 therm_grad_stepping0_feature_m(void)
{
	return 0x1U << 0U;
}
static inline u32 therm_grad_stepping0_feature_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 therm_grad_stepping0_feature_enable_f(void)
{
	return 0x1U;
}
static inline u32 therm_grad_stepping1_r(void)
{
	return 0x000202c4U;
}
static inline u32 therm_grad_stepping1_pdiv_duration_f(u32 v)
{
	return (v & 0x1ffffU) << 0U;
}
static inline u32 therm_clk_timing_r(u32 i)
{
	return 0x000203c0U + i*4U;
}
static inline u32 therm_clk_timing_grad_slowdown_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 therm_clk_timing_grad_slowdown_m(void)
{
	return 0x1U << 16U;
}
static inline u32 therm_clk_timing_grad_slowdown_enabled_f(void)
{
	return 0x10000U;
}
#endif
