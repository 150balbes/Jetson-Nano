/*
 * NVDLA OS Interface
 *
 * Copyright (c) 2016-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef _DLA_FW_VERSION_H_
#define _DLA_FW_VERSION_H_

#define FIRMWARE_VERSION_MAJOR		0x00
#define FIRMWARE_VERSION_MINOR		0x0c
#define FIRMWARE_VERSION_SUBMINOR	0x02

static inline uint32_t dla_version(void)
{
	return (uint32_t)(((FIRMWARE_VERSION_MAJOR & 0xff) << 16) |
				((FIRMWARE_VERSION_MINOR & 0xff) << 8) |
				((FIRMWARE_VERSION_SUBMINOR & 0xff)));
}

#endif
