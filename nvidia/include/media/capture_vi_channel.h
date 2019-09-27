/*
 * include/linux/media/capture_vi_channel.h
 *
 * VI channel driver header
 *
 * Copyright (c) 2017-2018 NVIDIA Corporation.  All rights reserved.
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

#ifndef __VI_CHANNEL_H__
#define __VI_CHANNEL_H__

#include <linux/of_platform.h>

struct vi_channel_drv;

struct vi_channel_drv_ops {
	int (*alloc_syncpt)(struct platform_device *pdev, const char *name,
			uint32_t *syncpt_id);

	void (*release_syncpt)(struct platform_device *pdev, uint32_t id);

	void (*get_gos_table)(struct platform_device *pdev, int *count,
			const dma_addr_t **table);

	int (*get_syncpt_gos_backing)(struct platform_device *pdev, uint32_t id,
			dma_addr_t *syncpt_addr, uint32_t *gos_index,
			uint32_t *gos_offset);
};

struct tegra_vi_channel {
	struct device *dev;
	struct platform_device *ndev;
	struct vi_channel_drv *drv;
	struct rcu_head rcu;
	struct vi_capture *capture_data;
	const struct vi_channel_drv_ops *ops;
	struct device *rtcpu_dev;
};

int vi_channel_drv_register(struct platform_device *,
			const struct vi_channel_drv_ops *);
void vi_channel_drv_unregister(struct device *);
void vi_capture_request_unpin(struct tegra_vi_channel *chan,
		uint32_t buffer_index);

/* Internal APIs for VI mode driver */
struct tegra_vi_channel *vi_channel_open_ex(unsigned channel,
	bool is_mem_pinned);
int vi_channel_close_ex(unsigned channel, struct tegra_vi_channel *chan);

#endif //__ISP_CHANNEL_H__
