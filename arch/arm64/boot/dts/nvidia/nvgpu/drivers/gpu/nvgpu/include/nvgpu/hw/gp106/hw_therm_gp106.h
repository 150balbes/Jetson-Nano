/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_therm_gp106_h_
#define _hw_therm_gp106_h_

static inline u32 therm_temp_sensor_tsense_r(void)
{
	return 0x00020460U;
}
static inline u32 therm_temp_sensor_tsense_fixed_point_f(u32 v)
{
	return (v & 0x3fffU) << 3U;
}
static inline u32 therm_temp_sensor_tsense_fixed_point_m(void)
{
	return 0x3fffU << 3U;
}
static inline u32 therm_temp_sensor_tsense_fixed_point_v(u32 r)
{
	return (r >> 3U) & 0x3fffU;
}
static inline u32 therm_temp_sensor_tsense_fixed_point_min_v(void)
{
	return 0x00003b00U;
}
static inline u32 therm_temp_sensor_tsense_fixed_point_max_v(void)
{
	return 0x000010e0U;
}
static inline u32 therm_temp_sensor_tsense_state_f(u32 v)
{
	return (v & 0x3U) << 29U;
}
static inline u32 therm_temp_sensor_tsense_state_m(void)
{
	return 0x3U << 29U;
}
static inline u32 therm_temp_sensor_tsense_state_v(u32 r)
{
	return (r >> 29U) & 0x3U;
}
static inline u32 therm_temp_sensor_tsense_state_valid_v(void)
{
	return 0x00000001U;
}
static inline u32 therm_temp_sensor_tsense_state_shadow_v(void)
{
	return 0x00000002U;
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
static inline u32 therm_gate_ctrl_eng_idle_filt_exp_f(u32 v)
{
	return (v & 0x1fU) << 8U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_exp_m(void)
{
	return 0x1fU << 8U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_mant_f(u32 v)
{
	return (v & 0x7U) << 13U;
}
static inline u32 therm_gate_ctrl_eng_idle_filt_mant_m(void)
{
	return 0x7U << 13U;
}
static inline u32 therm_gate_ctrl_eng_delay_before_f(u32 v)
{
	return (v & 0xfU) << 16U;
}
static inline u32 therm_gate_ctrl_eng_delay_before_m(void)
{
	return 0xfU << 16U;
}
static inline u32 therm_gate_ctrl_eng_delay_after_f(u32 v)
{
	return (v & 0xfU) << 20U;
}
static inline u32 therm_gate_ctrl_eng_delay_after_m(void)
{
	return 0xfU << 20U;
}
static inline u32 therm_fecs_idle_filter_r(void)
{
	return 0x00020288U;
}
static inline u32 therm_fecs_idle_filter_value_m(void)
{
	return 0xffffffffU << 0U;
}
static inline u32 therm_hubmmu_idle_filter_r(void)
{
	return 0x0002028cU;
}
static inline u32 therm_hubmmu_idle_filter_value_m(void)
{
	return 0xffffffffU << 0U;
}
#endif
