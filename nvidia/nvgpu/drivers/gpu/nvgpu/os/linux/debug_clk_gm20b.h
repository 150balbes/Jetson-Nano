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

#ifndef __DEBUG_CLK_GM20B_H
#define __DEBUG_CLK_GM20B_H

#ifdef CONFIG_DEBUG_FS
int gm20b_clk_init_debugfs(struct gk20a *g);
#else
inline int gm20b_clk_init_debugfs(struct gk20a *g)
{
	return 0;
}
#endif

#endif
