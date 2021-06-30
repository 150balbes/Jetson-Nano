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
#ifndef _hw_hsp_pva_h_
#define _hw_hsp_pva_h_

static inline u32 hsp_common_r(void)
{
	return 0x160000;
}
static inline u32 hsp_int_ie0_r(void)
{
	return 0x160100;
}
static inline u32 hsp_int_ie1_r(void)
{
	return 0x160104;
}
static inline u32 hsp_int_ie2_r(void)
{
	return 0x160108;
}
static inline u32 hsp_int_ie3_r(void)
{
	return 0x16010c;
}
static inline u32 hsp_int_ie4_r(void)
{
	return 0x160110;
}
static inline u32 hsp_int_external_r(void)
{
	return 0x160300;
}
static inline u32 hsp_int_internal_r(void)
{
	return 0x160304;
}
static inline u32 hsp_sm0_r(void)
{
	return 0x170000;
}
static inline u32 hsp_sm1_r(void)
{
	return 0x178000;
}
static inline u32 hsp_sm2_r(void)
{
	return 0x180000;
}
static inline u32 hsp_sm3_r(void)
{
	return 0x188000;
}
static inline u32 hsp_sm4_r(void)
{
	return 0x190000;
}
static inline u32 hsp_sm5_r(void)
{
	return 0x198000;
}
static inline u32 hsp_sm6_r(void)
{
	return 0x1a0000;
}
static inline u32 hsp_sm7_r(void)
{
	return 0x1a8000;
}
static inline u32 hsp_ss0_state_r(void)
{
	return 0x1b0000;
}
static inline u32 hsp_ss0_set_r(void)
{
	return 0x1b0004;
}
static inline u32 hsp_ss0_clr_r(void)
{
	return 0x1b0008;
}
static inline u32 hsp_ss1_state_r(void)
{
	return 0x1c0000;
}
static inline u32 hsp_ss1_set_r(void)
{
	return 0x1c0004;
}
static inline u32 hsp_ss1_clr_r(void)
{
	return 0x1c0008;
}
static inline u32 hsp_ss2_state_r(void)
{
	return 0x1d0000;
}
static inline u32 hsp_ss2_set_r(void)
{
	return 0x1d0004;
}
static inline u32 hsp_ss2_clr_r(void)
{
	return 0x1d0008;
}
static inline u32 hsp_ss3_state_r(void)
{
	return 0x1e0000;
}
static inline u32 hsp_ss3_set_r(void)
{
	return 0x1e0004;
}
static inline u32 hsp_ss3_clr_r(void)
{
	return 0x1e0008;
}
#endif
