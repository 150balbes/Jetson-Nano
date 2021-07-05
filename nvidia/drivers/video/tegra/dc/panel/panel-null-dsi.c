/*
 * panel-null-dsi-hotplug.c: Panel driver for null panel with hotplug supported.
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include "../dc.h"
#include "board.h"
#include "board-panel.h"

static int dsi_null_panel_enable(struct device *dev)
{
	return 0;
}

static int dsi_null_panel_disable(struct device *dev)
{
	return 0;
}

static int dsi_null_panel_postsuspend(void)
{
	return 0;
}

static int dsi_null_panel_hotplug_init(struct device *dev)
{
	return 0;
}

struct tegra_panel_ops dsi_null_panel_ops = {
	.enable = dsi_null_panel_enable,
	.disable = dsi_null_panel_disable,
	.postsuspend = dsi_null_panel_postsuspend,
	.hotplug_init = dsi_null_panel_hotplug_init,
};
