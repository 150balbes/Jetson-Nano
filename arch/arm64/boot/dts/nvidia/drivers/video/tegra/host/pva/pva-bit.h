/*
 * PVA bit manipulation header
 *
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
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
#ifndef _PVA_BIT_H_
#define _PVA_BIT_H_

#include <linux/bitops.h>
/*
 * Bit manipulation macros
 */

/*
 * 16-bits
 */
#define PVA_BIT16(_b_) (((_b_) < 16) ? ((uint16_t)1U << (_b_)) : (uint16_t)0U)
#define PVA_MASK16(_msb_, _lsb_)			\
	((PVA_BIT16(_msb_ + 1) - 1U) & ~(PVA_BIT16(_lsb_) - 1U))
#define PVA_EXTRACT16(_x_, _msb_, _lsb_, _type_)	\
	((_type_)((_x_ & PVA_MASK16(_msb_, _lsb_)) >> _lsb_))
#define PVA_INSERT16(_x_, _msb_, _lsb_)			\
	((((uint16_t)_x_) << _lsb_) & PVA_MASK16(_msb_, _lsb_))

/*
 * 32-bits
 */
#define PVA_BIT(_b_) BIT(_b_)
#define PVA_MASK(_msb_, _lsb_)	GENMASK(_msb_, _lsb_)
#define PVA_EXTRACT(_x_, _msb_, _lsb_, _type_)	\
	((_type_)((_x_ & GENMASK(_msb_, _lsb_)) >> _lsb_))
#define PVA_INSERT(_x_, _msb_, _lsb_)		\
	((((uint32_t)_x_) << _lsb_) & PVA_MASK(_msb_, _lsb_))

/*
 * 64-bits
 */
#define PVA_BIT64(_b_) BIT_ULL(_b_)
#define PVA_MASK64(_msb_, _lsb_) GENMASK_ULL(_msb_, _lsb_)
#define PVA_EXTRACT64(_x_, _msb_, _lsb_, _type_)	\
	((_type_)((_x_ & GENMASK_ULL(_msb_, _lsb_)) >> _lsb_))
#define PVA_INSERT64(_x_, _msb_, _lsb_)		\
	((((uint64_t)_x_) << _lsb_) & GENMASK_ULL(_msb_, _lsb_))

#define PVA_PACK64(_l_, _h_)			\
	(PVA_INSERT64(_h_, 63, 32) | PVA_INSERT64(_l_, 31, 0))

#define PVA_HI32(_x_)		PVA_EXTRACT64(_x_, 63, 32, uint32_t)
#define PVA_LOW32(_x_)		PVA_EXTRACT64(_x_, 31, 0, uint32_t)

#endif
