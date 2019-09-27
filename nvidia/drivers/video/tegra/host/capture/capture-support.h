/*
 * Capture support for T194
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef _CAPTURE_SUPPORT_H_
#define _CAPTURE_SUPPORT_H_

#include <linux/types.h>
#include <linux/platform_device.h>

int t194_capture_alloc_syncpt(struct platform_device *pdev,
			const char *name,
			uint32_t *syncpt_id);

void t194_capture_release_syncpt(struct platform_device *pdev, uint32_t id);

void t194_capture_get_gos_table(struct platform_device *pdev,
			int *gos_count,
			const dma_addr_t **gos_table);

int t194_capture_get_syncpt_gos_backing(struct platform_device *pdev,
			uint32_t id,
			dma_addr_t *syncpt_addr,
			uint32_t *gos_index,
			uint32_t *gos_offset);

#endif /* _CAPTURE_SUPPORT_H_ */
