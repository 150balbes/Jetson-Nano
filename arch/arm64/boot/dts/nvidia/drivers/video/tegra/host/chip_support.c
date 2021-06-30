/*
 * Tegra Graphics Host Chip support module
 *
 * Copyright (c) 2012-2017, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/slab.h>
#include <soc/tegra/chip-id.h>
#include <linux/version.h>

#if defined(CONFIG_ARCH_TEGRA_210_SOC) || LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
#include <soc/tegra/fuse.h>
#endif

#include "host1x/host1x.h"
#include "chip_support.h"
#include "t124/t124.h"
#include "t210/t210.h"

static struct nvhost_chip_support *nvhost_chip_ops;

struct nvhost_chip_support *nvhost_get_chip_ops(void)
{
	return nvhost_chip_ops;
}

int nvhost_init_chip_support(struct nvhost_master *host)
{
	int err = 0;

	if (nvhost_chip_ops == NULL) {
		nvhost_chip_ops = kzalloc(sizeof(*nvhost_chip_ops), GFP_KERNEL);
		if (nvhost_chip_ops == NULL) {
			pr_err("%s: Cannot allocate nvhost_chip_support\n",
				__func__);
			return 0;
		}
	}

	if (!host->info.initialize_chip_support)
		return -ENODEV;

	err = host->info.initialize_chip_support(host, nvhost_chip_ops);

	return err;
}

bool nvhost_is_124(void)
{
	return tegra_get_chip_id() == TEGRA124 ||
	       tegra_get_chip_id() == TEGRA132;
}

bool nvhost_is_210(void)
{
	return tegra_get_chip_id() == TEGRA210;
}

bool nvhost_is_186(void)
{
	return tegra_get_chip_id() == TEGRA186;
}
