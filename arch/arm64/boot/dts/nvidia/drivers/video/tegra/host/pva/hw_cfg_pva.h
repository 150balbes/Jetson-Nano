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
#ifndef _hw_cfg_pva_h_
#define _hw_cfg_pva_h_

static inline u32 cfg_user_sid_r(void)
{
	return 0x70000;
}
static inline u32 cfg_ccq_r(void)
{
	return 0x71000;
}
static inline u32 cfg_vps0user_lsegreg_r(void)
{
	return 0x71004;
}
static inline u32 cfg_vps1user_lsegreg_r(void)
{
	return 0x71008;
}
static inline u32 cfg_r5user_lsegreg_r(void)
{
	return 0x7100c;
}
static inline u32 cfg_vps0user_usegreg_r(void)
{
	return 0x71010;
}
static inline u32 cfg_vps1user_usegreg_r(void)
{
	return 0x71014;
}
static inline u32 cfg_r5user_usegreg_r(void)
{
	return 0x71018;
}
static inline u32 cfg_ccq_status0_r(void)
{
	return 0x72000;
}
static inline u32 cfg_ccq_status1_r(void)
{
	return 0x72004;
}
static inline u32 cfg_ccq_status2_r(void)
{
	return 0x72008;
}
static inline u32 cfg_ccq_status3_r(void)
{
	return 0x7200c;
}
static inline u32 cfg_ccq_status4_r(void)
{
	return 0x72010;
}
static inline u32 cfg_ccq_status5_r(void)
{
	return 0x72014;
}
static inline u32 cfg_ccq_status6_r(void)
{
	return 0x72018;
}
static inline u32 cfg_ccq_status7_r(void)
{
	return 0x7201c;
}
static inline u32 cfg_ccq_status8_r(void)
{
	return 0x72020;
}
static inline u32 cfg_priv_sid_r(void)
{
	return 0x80000;
}
static inline u32 cfg_priv_ar1_lsegreg_r(void)
{
	return 0x80004;
}
static inline u32 cfg_priv_ar1_usegreg_r(void)
{
	return 0x80008;
}
static inline u32 cfg_priv_ar2_lsegreg_r(void)
{
	return 0x8000c;
}
static inline u32 cfg_priv_ar2_usegreg_r(void)
{
	return 0x80010;
}
static inline u32 cfg_priv_ar1_start_r(void)
{
	return 0x80014;
}
static inline u32 cfg_priv_ar1_end_r(void)
{
	return 0x80018;
}
static inline u32 cfg_priv_ar2_start_r(void)
{
	return 0x8001c;
}
static inline u32 cfg_priv_ar2_end_r(void)
{
	return 0x80020;
}
#endif
