/*
 * Dma override functions
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)

#include <soc/tegra/chip-id.h>
#include <linux/dma-override.h>

static bool is_t19x;

void dma_qualify_ioprot(enum dma_data_direction dir, unsigned long *ioprot)
{
	if (is_t19x && (dir & DMA_FOR_NVLINK)) {
		*ioprot |= DMA_FOR_NVLINK;
		*ioprot &= ~IOMMU_CACHE;
	}
}

void dma_marshal_handle(enum dma_data_direction dir, dma_addr_t *handle)
{
	if (is_t19x && (dir & DMA_FOR_NVLINK))
		*handle |= DMA_FOR_NVLINK;
}

void dma_unmarshal_handle(enum dma_data_direction dir, dma_addr_t *handle)
{
	if (is_t19x && (dir & DMA_FOR_NVLINK))
		*handle &= ~DMA_FOR_NVLINK;
}

static int __init dma_override(void)
{
	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA19)
		is_t19x = true;

	return 0;
}
arch_initcall(dma_override);

#endif
