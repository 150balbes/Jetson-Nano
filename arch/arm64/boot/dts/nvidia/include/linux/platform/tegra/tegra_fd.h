/*
 * Copyright (C) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#include <linux/fdtable.h>

#ifndef __TEGRA_FD_H
#define __TEGRA_FD_H

int tegra_alloc_fd(struct files_struct *files, unsigned int start,
		   unsigned int flags);

#endif
