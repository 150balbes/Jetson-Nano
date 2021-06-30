/*
 * Copyright (c) 2018, NVIDIA Corporation.  All rights reserved.
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

#include <nvgpu/dt.h>
#include <linux/of.h>

#include "os_linux.h"

int nvgpu_dt_read_u32_index(struct gk20a *g, const char *name,
				u32 index, u32 *value)
{
	struct device *dev = dev_from_gk20a(g);
	struct device_node *np = dev->of_node;

	return of_property_read_u32_index(np, name, index, value);
}
