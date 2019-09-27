/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_DMA_MAPPING_OVERRIDE_H
#define _LINUX_DMA_MAPPING_OVERRIDE_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)

#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>

#define DMA_FOR_NVLINK	(1 << 5)

#define NVLINK_PHY_BIT	37

void dma_qualify_ioprot(enum dma_data_direction dir, unsigned long *ioprot);

void dma_marshal_handle(enum dma_data_direction dir, dma_addr_t *handle);

void dma_unmarshal_handle(enum dma_data_direction dir, dma_addr_t *handle);

#endif
#endif

