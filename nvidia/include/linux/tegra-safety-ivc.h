/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_TEGRA_SAFETY_IVC_H_
#define _LINUX_TEGRA_SAFETY_IVC_H_

#define SAFETY_CONF(id, value)          ((SAFETY_CONF_ ## id << 24) | (value))
#define SAFETY_CONF_GET_ID(value)       (((value) >> 24) & 0x7f)
#define SAFETY_CONF_GET_VALUE(value)    ((value) & 0xffffff)

enum {
	SAFETY_CONF_IVC_READY = 1,
};

#define MAX_SAFETY_CHANNELS		5

struct safety_ast_region {
	u8 ast_id;
	u32 slave_base;
	size_t size;
	void *base;
	dma_addr_t dma;
	struct device dev;
};

struct tegra_safety_ivc {
	struct safety_ast_region region;
	struct tegra_hsp_sm_pair *cmd_pair;
	struct tegra_hsp_sm_pair *ivc_pair;
	struct {
		wait_queue_head_t response_waitq;
		wait_queue_head_t empty_waitq;
		atomic_t response;
		atomic_t emptied;
	} cmd;
	struct tegra_safety_ivc_chan *ivc_chan[MAX_SAFETY_CHANNELS];
};

struct tegra_safety_ivc_chan {
	struct ivc ivc;
	char *name;
	struct tegra_safety_ivc *safety_ivc;
};

int tegra_safety_dev_init(struct device *dev, int index);
void tegra_safety_dev_exit(struct device *dev, int index);
void tegra_safety_dev_notify(void);

#endif
