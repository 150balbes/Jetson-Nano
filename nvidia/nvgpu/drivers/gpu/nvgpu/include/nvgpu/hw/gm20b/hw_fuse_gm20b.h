/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_fuse_gm20b_h_
#define _hw_fuse_gm20b_h_

static inline u32 fuse_status_opt_gpc_r(void)
{
	return 0x00021c1cU;
}
static inline u32 fuse_status_opt_tpc_gpc_r(u32 i)
{
	return 0x00021c38U + i*4U;
}
static inline u32 fuse_ctrl_opt_tpc_gpc_r(u32 i)
{
	return 0x00021838U + i*4U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_r(void)
{
	return 0x00021944U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_data_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_data_m(void)
{
	return 0x3U << 0U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_data_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_override_r(void)
{
	return 0x00021948U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_override_data_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_override_data_m(void)
{
	return 0x1U << 0U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_override_data_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_override_data_yes_f(void)
{
	return 0x1U;
}
static inline u32 fuse_ctrl_opt_ram_svop_pdp_override_data_no_f(void)
{
	return 0x0U;
}
static inline u32 fuse_status_opt_fbio_r(void)
{
	return 0x00021c14U;
}
static inline u32 fuse_status_opt_fbio_data_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 fuse_status_opt_fbio_data_m(void)
{
	return 0xffffU << 0U;
}
static inline u32 fuse_status_opt_fbio_data_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 fuse_status_opt_rop_l2_fbp_r(u32 i)
{
	return 0x00021d70U + i*4U;
}
static inline u32 fuse_status_opt_fbp_r(void)
{
	return 0x00021d38U;
}
static inline u32 fuse_status_opt_fbp_idx_v(u32 r, u32 i)
{
	return (r >> (0U + i*1U)) & 0x1U;
}
static inline u32 fuse_opt_sec_debug_en_r(void)
{
	return 0x00021218U;
}
static inline u32 fuse_opt_priv_sec_en_r(void)
{
	return 0x00021434U;
}
#endif
