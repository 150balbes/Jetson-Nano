/*
 * Copyright (c) 2012-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_trim_gk20a_h_
#define _hw_trim_gk20a_h_

static inline u32 trim_sys_gpcpll_cfg_r(void)
{
	return 0x00137000U;
}
static inline u32 trim_sys_gpcpll_cfg_enable_m(void)
{
	return 0x1U << 0U;
}
static inline u32 trim_sys_gpcpll_cfg_enable_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 trim_sys_gpcpll_cfg_enable_no_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_gpcpll_cfg_enable_yes_f(void)
{
	return 0x1U;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_m(void)
{
	return 0x1U << 1U;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 trim_sys_gpcpll_cfg_iddq_power_on_v(void)
{
	return 0x00000000U;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_m(void)
{
	return 0x1U << 4U;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_on_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_gpcpll_cfg_enb_lckdet_power_off_f(void)
{
	return 0x10U;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_v(u32 r)
{
	return (r >> 17U) & 0x1U;
}
static inline u32 trim_sys_gpcpll_cfg_pll_lock_true_f(void)
{
	return 0x20000U;
}
static inline u32 trim_sys_gpcpll_coeff_r(void)
{
	return 0x00137004U;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_m(void)
{
	return 0xffU << 0U;
}
static inline u32 trim_sys_gpcpll_coeff_mdiv_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_m(void)
{
	return 0xffU << 8U;
}
static inline u32 trim_sys_gpcpll_coeff_ndiv_v(u32 r)
{
	return (r >> 8U) & 0xffU;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_f(u32 v)
{
	return (v & 0x3fU) << 16U;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_m(void)
{
	return 0x3fU << 16U;
}
static inline u32 trim_sys_gpcpll_coeff_pldiv_v(u32 r)
{
	return (r >> 16U) & 0x3fU;
}
static inline u32 trim_sys_sel_vco_r(void)
{
	return 0x00137100U;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_m(void)
{
	return 0x1U << 0U;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_init_v(void)
{
	return 0x00000000U;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_init_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_bypass_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_sel_vco_gpc2clk_out_vco_f(void)
{
	return 0x1U;
}
static inline u32 trim_sys_gpc2clk_out_r(void)
{
	return 0x00137250U;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_s(void)
{
	return 6U;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_f(u32 v)
{
	return (v & 0x3fU) << 0U;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_m(void)
{
	return 0x3fU << 0U;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_v(u32 r)
{
	return (r >> 0U) & 0x3fU;
}
static inline u32 trim_sys_gpc2clk_out_bypdiv_by31_f(void)
{
	return 0x3cU;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_s(void)
{
	return 6U;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_f(u32 v)
{
	return (v & 0x3fU) << 8U;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_m(void)
{
	return 0x3fU << 8U;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_v(u32 r)
{
	return (r >> 8U) & 0x3fU;
}
static inline u32 trim_sys_gpc2clk_out_vcodiv_by1_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_m(void)
{
	return 0x1U << 31U;
}
static inline u32 trim_sys_gpc2clk_out_sdiv14_indiv4_mode_f(void)
{
	return 0x80000000U;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_r(u32 i)
{
	return 0x00134124U + i*512U;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_noofipclks_f(u32 v)
{
	return (v & 0x3fffU) << 0U;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_write_en_asserted_f(void)
{
	return 0x10000U;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_enable_asserted_f(void)
{
	return 0x100000U;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cfg_reset_asserted_f(void)
{
	return 0x1000000U;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cnt_r(u32 i)
{
	return 0x00134128U + i*512U;
}
static inline u32 trim_gpc_clk_cntr_ncgpcclk_cnt_value_v(u32 r)
{
	return (r >> 0U) & 0xfffffU;
}
static inline u32 trim_sys_gpcpll_cfg2_r(void)
{
	return 0x0013700cU;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_f(u32 v)
{
	return (v & 0xffU) << 24U;
}
static inline u32 trim_sys_gpcpll_cfg2_pll_stepa_m(void)
{
	return 0xffU << 24U;
}
static inline u32 trim_sys_gpcpll_cfg3_r(void)
{
	return 0x00137018U;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 trim_sys_gpcpll_cfg3_pll_stepb_m(void)
{
	return 0xffU << 16U;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_r(void)
{
	return 0x0013701cU;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_m(void)
{
	return 0x1U << 22U;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_yes_f(void)
{
	return 0x400000U;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_no_f(void)
{
	return 0x0U;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_m(void)
{
	return 0x1U << 31U;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_yes_f(void)
{
	return 0x80000000U;
}
static inline u32 trim_sys_gpcpll_ndiv_slowdown_en_dynramp_no_f(void)
{
	return 0x0U;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_r(void)
{
	return 0x001328a0U;
}
static inline u32 trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_pll_dynramp_done_synced_v(u32 r)
{
	return (r >> 24U) & 0x1U;
}
#endif
