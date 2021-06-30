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
 */

#include <linux/fdtable.h>
#include <linux/fs.h>

#include <linux/platform/tegra/tegra_fd.h>

/* allocates a free fd within [start, sysctl_nr_open) range */
int tegra_alloc_fd(struct files_struct *files, unsigned int start,
		   unsigned int flags)
{
	return __alloc_fd(files, start, sysctl_nr_open, flags);
}
EXPORT_SYMBOL_GPL(tegra_alloc_fd);
