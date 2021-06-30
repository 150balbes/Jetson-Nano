/*
 * Copyright (c) 2017, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __LINUX_NVHOST_T194_H__
#define __LINUX_NVHOST_T194_H__

int nvhost_syncpt_unit_interface_get_aperture(
				struct platform_device *host_pdev,
				phys_addr_t *base,
				size_t *size);

u32 nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id);

#endif /* __LINUX_NVHOST_T194_H__ */
