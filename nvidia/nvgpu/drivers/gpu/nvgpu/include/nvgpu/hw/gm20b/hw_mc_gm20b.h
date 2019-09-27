/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_mc_gm20b_h_
#define _hw_mc_gm20b_h_

static inline u32 mc_boot_0_r(void)
{
	return 0x00000000U;
}
static inline u32 mc_boot_0_architecture_v(u32 r)
{
	return (r >> 24U) & 0x1fU;
}
static inline u32 mc_boot_0_implementation_v(u32 r)
{
	return (r >> 20U) & 0xfU;
}
static inline u32 mc_boot_0_major_revision_v(u32 r)
{
	return (r >> 4U) & 0xfU;
}
static inline u32 mc_boot_0_minor_revision_v(u32 r)
{
	return (r >> 0U) & 0xfU;
}
static inline u32 mc_intr_r(u32 i)
{
	return 0x00000100U + i*4U;
}
static inline u32 mc_intr_pfifo_pending_f(void)
{
	return 0x100U;
}
static inline u32 mc_intr_pmu_pending_f(void)
{
	return 0x1000000U;
}
static inline u32 mc_intr_ltc_pending_f(void)
{
	return 0x2000000U;
}
static inline u32 mc_intr_priv_ring_pending_f(void)
{
	return 0x40000000U;
}
static inline u32 mc_intr_pbus_pending_f(void)
{
	return 0x10000000U;
}
static inline u32 mc_intr_mask_0_r(void)
{
	return 0x00000640U;
}
static inline u32 mc_intr_mask_0_pmu_enabled_f(void)
{
	return 0x1000000U;
}
static inline u32 mc_intr_en_0_r(void)
{
	return 0x00000140U;
}
static inline u32 mc_intr_en_0_inta_disabled_f(void)
{
	return 0x0U;
}
static inline u32 mc_intr_en_0_inta_hardware_f(void)
{
	return 0x1U;
}
static inline u32 mc_intr_mask_1_r(void)
{
	return 0x00000644U;
}
static inline u32 mc_intr_mask_1_pmu_s(void)
{
	return 1U;
}
static inline u32 mc_intr_mask_1_pmu_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 mc_intr_mask_1_pmu_m(void)
{
	return 0x1U << 24U;
}
static inline u32 mc_intr_mask_1_pmu_v(u32 r)
{
	return (r >> 24U) & 0x1U;
}
static inline u32 mc_intr_mask_1_pmu_enabled_f(void)
{
	return 0x1000000U;
}
static inline u32 mc_intr_en_1_r(void)
{
	return 0x00000144U;
}
static inline u32 mc_intr_en_1_inta_disabled_f(void)
{
	return 0x0U;
}
static inline u32 mc_intr_en_1_inta_hardware_f(void)
{
	return 0x1U;
}
static inline u32 mc_enable_r(void)
{
	return 0x00000200U;
}
static inline u32 mc_enable_xbar_enabled_f(void)
{
	return 0x4U;
}
static inline u32 mc_enable_l2_enabled_f(void)
{
	return 0x8U;
}
static inline u32 mc_enable_pmedia_s(void)
{
	return 1U;
}
static inline u32 mc_enable_pmedia_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 mc_enable_pmedia_m(void)
{
	return 0x1U << 4U;
}
static inline u32 mc_enable_pmedia_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 mc_enable_priv_ring_enabled_f(void)
{
	return 0x20U;
}
static inline u32 mc_enable_ce0_m(void)
{
	return 0x1U << 6U;
}
static inline u32 mc_enable_pfifo_enabled_f(void)
{
	return 0x100U;
}
static inline u32 mc_enable_pgraph_enabled_f(void)
{
	return 0x1000U;
}
static inline u32 mc_enable_pwr_v(u32 r)
{
	return (r >> 13U) & 0x1U;
}
static inline u32 mc_enable_pwr_disabled_v(void)
{
	return 0x00000000U;
}
static inline u32 mc_enable_pwr_enabled_f(void)
{
	return 0x2000U;
}
static inline u32 mc_enable_pfb_enabled_f(void)
{
	return 0x100000U;
}
static inline u32 mc_enable_ce2_m(void)
{
	return 0x1U << 21U;
}
static inline u32 mc_enable_ce2_enabled_f(void)
{
	return 0x200000U;
}
static inline u32 mc_enable_blg_enabled_f(void)
{
	return 0x8000000U;
}
static inline u32 mc_enable_perfmon_enabled_f(void)
{
	return 0x10000000U;
}
static inline u32 mc_enable_hub_enabled_f(void)
{
	return 0x20000000U;
}
static inline u32 mc_intr_ltc_r(void)
{
	return 0x0000017cU;
}
static inline u32 mc_enable_pb_r(void)
{
	return 0x00000204U;
}
static inline u32 mc_enable_pb_0_s(void)
{
	return 1U;
}
static inline u32 mc_enable_pb_0_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 mc_enable_pb_0_m(void)
{
	return 0x1U << 0U;
}
static inline u32 mc_enable_pb_0_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 mc_enable_pb_0_enabled_v(void)
{
	return 0x00000001U;
}
static inline u32 mc_enable_pb_sel_f(u32 v, u32 i)
{
	return (v & 0x1U) << (0U + i*1U);
}
static inline u32 mc_elpg_enable_r(void)
{
	return 0x0000020cU;
}
static inline u32 mc_elpg_enable_xbar_enabled_f(void)
{
	return 0x4U;
}
static inline u32 mc_elpg_enable_pfb_enabled_f(void)
{
	return 0x100000U;
}
static inline u32 mc_elpg_enable_hub_enabled_f(void)
{
	return 0x20000000U;
}
#endif
