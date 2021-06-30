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
#ifndef _hw_xve_gp106_h_
#define _hw_xve_gp106_h_

static inline u32 xve_rom_ctrl_r(void)
{
	return 0x00000050U;
}
static inline u32 xve_rom_ctrl_rom_shadow_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 xve_rom_ctrl_rom_shadow_disabled_f(void)
{
	return 0x0U;
}
static inline u32 xve_rom_ctrl_rom_shadow_enabled_f(void)
{
	return 0x1U;
}
static inline u32 xve_link_control_status_r(void)
{
	return 0x00000088U;
}
static inline u32 xve_link_control_status_link_speed_m(void)
{
	return 0xfU << 16U;
}
static inline u32 xve_link_control_status_link_speed_v(u32 r)
{
	return (r >> 16U) & 0xfU;
}
static inline u32 xve_link_control_status_link_speed_link_speed_2p5_v(void)
{
	return 0x00000001U;
}
static inline u32 xve_link_control_status_link_speed_link_speed_5p0_v(void)
{
	return 0x00000002U;
}
static inline u32 xve_link_control_status_link_speed_link_speed_8p0_v(void)
{
	return 0x00000003U;
}
static inline u32 xve_link_control_status_link_width_m(void)
{
	return 0x3fU << 20U;
}
static inline u32 xve_link_control_status_link_width_v(u32 r)
{
	return (r >> 20U) & 0x3fU;
}
static inline u32 xve_link_control_status_link_width_x1_v(void)
{
	return 0x00000001U;
}
static inline u32 xve_link_control_status_link_width_x2_v(void)
{
	return 0x00000002U;
}
static inline u32 xve_link_control_status_link_width_x4_v(void)
{
	return 0x00000004U;
}
static inline u32 xve_link_control_status_link_width_x8_v(void)
{
	return 0x00000008U;
}
static inline u32 xve_link_control_status_link_width_x16_v(void)
{
	return 0x00000010U;
}
static inline u32 xve_priv_xv_r(void)
{
	return 0x00000150U;
}
static inline u32 xve_priv_xv_cya_l0s_enable_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 xve_priv_xv_cya_l0s_enable_m(void)
{
	return 0x1U << 7U;
}
static inline u32 xve_priv_xv_cya_l0s_enable_v(u32 r)
{
	return (r >> 7U) & 0x1U;
}
static inline u32 xve_priv_xv_cya_l1_enable_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 xve_priv_xv_cya_l1_enable_m(void)
{
	return 0x1U << 8U;
}
static inline u32 xve_priv_xv_cya_l1_enable_v(u32 r)
{
	return (r >> 8U) & 0x1U;
}
static inline u32 xve_cya_2_r(void)
{
	return 0x00000704U;
}
static inline u32 xve_reset_r(void)
{
	return 0x00000718U;
}
static inline u32 xve_reset_reset_m(void)
{
	return 0x1U << 0U;
}
static inline u32 xve_reset_gpu_on_sw_reset_m(void)
{
	return 0x1U << 1U;
}
static inline u32 xve_reset_counter_en_m(void)
{
	return 0x1U << 2U;
}
static inline u32 xve_reset_counter_val_f(u32 v)
{
	return (v & 0x7ffU) << 4U;
}
static inline u32 xve_reset_counter_val_m(void)
{
	return 0x7ffU << 4U;
}
static inline u32 xve_reset_counter_val_v(u32 r)
{
	return (r >> 4U) & 0x7ffU;
}
static inline u32 xve_reset_clock_on_sw_reset_m(void)
{
	return 0x1U << 15U;
}
static inline u32 xve_reset_clock_counter_en_m(void)
{
	return 0x1U << 16U;
}
static inline u32 xve_reset_clock_counter_val_f(u32 v)
{
	return (v & 0x7ffU) << 17U;
}
static inline u32 xve_reset_clock_counter_val_m(void)
{
	return 0x7ffU << 17U;
}
static inline u32 xve_reset_clock_counter_val_v(u32 r)
{
	return (r >> 17U) & 0x7ffU;
}
#endif
