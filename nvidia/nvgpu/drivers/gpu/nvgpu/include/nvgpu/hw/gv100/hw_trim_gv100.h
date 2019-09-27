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
#ifndef _hw_trim_gv100_h_
#define _hw_trim_gv100_h_

static inline u32 trim_sys_nvlink_uphy_cfg_r(void)
{
	return 0x00132410U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_lockdect_wait_dly_length_f(u32 v)
{
	return (v & 0x3ffU) << 0U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_lockdect_wait_dly_length_m(void)
{
	return 0x3ffU << 0U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_lockdect_wait_dly_length_v(u32 r)
{
	return (r >> 0U) & 0x3ffU;
}
static inline u32 trim_sys_nvlink_uphy_cfg_phy2clks_use_lockdet_f(u32 v)
{
	return (v & 0x1U) << 12U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_phy2clks_use_lockdet_m(void)
{
	return 0x1U << 12U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_phy2clks_use_lockdet_v(u32 r)
{
	return (r >> 12U) & 0x1U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_nvlink_wait_dly_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_nvlink_wait_dly_m(void)
{
	return 0xffU << 16U;
}
static inline u32 trim_sys_nvlink_uphy_cfg_nvlink_wait_dly_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 trim_sys_nvlink0_ctrl_r(void)
{
	return 0x00132420U;
}
static inline u32 trim_sys_nvlink0_ctrl_unit2clks_pll_turn_off_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 trim_sys_nvlink0_ctrl_unit2clks_pll_turn_off_m(void)
{
	return 0x1U << 0U;
}
static inline u32 trim_sys_nvlink0_ctrl_unit2clks_pll_turn_off_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 trim_sys_nvlink0_status_r(void)
{
	return 0x00132424U;
}
static inline u32 trim_sys_nvlink0_status_pll_off_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 trim_sys_nvlink0_status_pll_off_m(void)
{
	return 0x1U << 5U;
}
static inline u32 trim_sys_nvlink0_status_pll_off_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_r(void)
{
	return 0x001371c4U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_slowclk_f(u32 v)
{
	return (v & 0x3U) << 16U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_slowclk_m(void)
{
	return 0x3U << 16U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_slowclk_v(u32 r)
{
	return (r >> 16U) & 0x3U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_slowclk_xtal4x_v(void)
{
	return 0x00000003U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_slowclk_xtal4x_f(void)
{
	return 0x30000U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_slowclk_xtal_in_v(void)
{
	return 0x00000000U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_slowclk_xtal_in_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_m(void)
{
	return 0x3U << 0U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_slowclk_v(void)
{
	return 0x00000000U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_slowclk_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_miscclk_v(void)
{
	return 0x00000002U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_miscclk_f(void)
{
	return 0x2U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_onesrcclk_v(void)
{
	return 0x00000003U;
}
static inline u32 trim_sys_nvl_common_clk_alt_switch_finalsel_onesrcclk_f(void)
{
	return 0x3U;
}
static inline u32 trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cfg_r(void)
{
	return 0x00132a70U;
}
static inline u32 trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cfg_source_gpcclk_f(void)
{
	return 0x10000000U;
}
static inline u32 trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cnt0_r(void)
{
	return 0x00132a74U;
}
static inline u32 trim_gpc_bcast_fr_clk_cntr_ncgpcclk_cnt1_r(void)
{
	return 0x00132a78U;
}
static inline u32 trim_sys_nafll_fr_clk_cntr_xbarclk_cfg_r(void)
{
	return 0x00136470U;
}
static inline u32 trim_sys_nafll_fr_clk_cntr_xbarclk_cfg_source_xbarclk_f(void)
{
	return 0x10000000U;
}
static inline u32 trim_sys_nafll_fr_clk_cntr_xbarclk_cntr0_r(void)
{
	return 0x00136474U;
}
static inline u32 trim_sys_nafll_fr_clk_cntr_xbarclk_cntr1_r(void)
{
	return 0x00136478U;
}
static inline u32 trim_sys_fr_clk_cntr_sysclk_cfg_r(void)
{
	return 0x0013762cU;
}
static inline u32 trim_sys_fr_clk_cntr_sysclk_cfg_source_sysclk_f(void)
{
	return 0x20000000U;
}
static inline u32 trim_sys_fr_clk_cntr_sysclk_cntr0_r(void)
{
	return 0x00137630U;
}
static inline u32 trim_sys_fr_clk_cntr_sysclk_cntr1_r(void)
{
	return 0x00137634U;
}
#endif
