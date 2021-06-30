/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_dma_desc_pva_h_
#define _hw_dma_desc_pva_h_

static inline u32 dma_desc_base_r(void)
{
	return 0xc1000;
}
static inline u32 dma_desc_size_r(void)
{
	return 0x40;
}
static inline u32 dma_desc_cntl_r(void)
{
	return 0x0;
}
static inline u32 dma_desc_cntl_dstm_f(u32 v)
{
	return (v & 0x7) << 0;
}
static inline u32 dma_desc_cntl_dstm_dstm_mc_v(void)
{
	return 0x00000001;
}
static inline u32 dma_desc_cntl_dstm_dstm_tcm_v(void)
{
	return 0x00000004;
}
static inline u32 dma_desc_cntl_ddtm_f(u32 v)
{
	return (v & 0x7) << 4;
}
static inline u32 dma_desc_cntl_ddtm_ddtm_mc_v(void)
{
	return 0x00000001;
}
static inline u32 dma_desc_cntl_ddtm_ddtm_tcm_v(void)
{
	return 0x00000004;
}
static inline u32 dma_desc_cntl_srch_f(u32 v)
{
	return (v & 0xff) << 16;
}
static inline u32 dma_desc_cntl_dsth_f(u32 v)
{
	return (v & 0xff) << 24;
}
static inline u32 dma_desc_srcl_r(void)
{
	return 0x4;
}
static inline u32 dma_desc_dstl_r(void)
{
	return 0x8;
}
static inline u32 dma_desc_tile_cntl_r(void)
{
	return 0xc;
}
static inline u32 dma_desc_tile_cntl_tx_f(u32 v)
{
	return (v & 0xffff) << 0;
}
static inline u32 dma_desc_tile_cntl_ty_f(u32 v)
{
	return (v & 0xffff) << 16;
}
#endif
