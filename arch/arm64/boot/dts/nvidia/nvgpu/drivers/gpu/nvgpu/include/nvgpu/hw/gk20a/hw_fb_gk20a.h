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
#ifndef _hw_fb_gk20a_h_
#define _hw_fb_gk20a_h_

static inline u32 fb_mmu_ctrl_r(void)
{
	return 0x00100c80U;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_128kb_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_64kb_f(void)
{
	return 0x1U;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_v(u32 r)
{
	return (r >> 15U) & 0x1U;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_false_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_ctrl_pri_fifo_space_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 fb_mmu_invalidate_pdb_r(void)
{
	return 0x00100cb8U;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_sys_mem_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_invalidate_pdb_addr_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 fb_mmu_invalidate_r(void)
{
	return 0x00100cbcU;
}
static inline u32 fb_mmu_invalidate_all_va_true_f(void)
{
	return 0x1U;
}
static inline u32 fb_mmu_invalidate_all_pdb_true_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_invalidate_trigger_s(void)
{
	return 1U;
}
static inline u32 fb_mmu_invalidate_trigger_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 fb_mmu_invalidate_trigger_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_mmu_invalidate_trigger_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 fb_mmu_invalidate_trigger_true_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_mmu_debug_wr_r(void)
{
	return 0x00100cc8U;
}
static inline u32 fb_mmu_debug_wr_aperture_s(void)
{
	return 2U;
}
static inline u32 fb_mmu_debug_wr_aperture_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 fb_mmu_debug_wr_aperture_m(void)
{
	return 0x3U << 0U;
}
static inline u32 fb_mmu_debug_wr_aperture_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 fb_mmu_debug_wr_aperture_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_coh_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_ncoh_f(void)
{
	return 0x3U;
}
static inline u32 fb_mmu_debug_wr_vol_false_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_wr_vol_true_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_debug_wr_vol_true_f(void)
{
	return 0x4U;
}
static inline u32 fb_mmu_debug_wr_addr_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 fb_mmu_debug_wr_addr_alignment_v(void)
{
	return 0x0000000cU;
}
static inline u32 fb_mmu_debug_rd_r(void)
{
	return 0x00100cccU;
}
static inline u32 fb_mmu_debug_rd_aperture_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_coh_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_ncoh_f(void)
{
	return 0x3U;
}
static inline u32 fb_mmu_debug_rd_vol_false_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_rd_addr_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 fb_mmu_debug_rd_addr_alignment_v(void)
{
	return 0x0000000cU;
}
static inline u32 fb_mmu_debug_ctrl_r(void)
{
	return 0x00100cc4U;
}
static inline u32 fb_mmu_debug_ctrl_debug_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 fb_mmu_debug_ctrl_debug_m(void)
{
	return 0x1U << 16U;
}
static inline u32 fb_mmu_debug_ctrl_debug_enabled_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_debug_ctrl_debug_enabled_f(void)
{
	return 0x10000U;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_vpr_info_r(void)
{
	return 0x00100cd0U;
}
static inline u32 fb_mmu_vpr_info_fetch_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 fb_mmu_vpr_info_fetch_false_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_vpr_info_fetch_true_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_niso_flush_sysmem_addr_r(void)
{
	return 0x00100c10U;
}
#endif
