/*
 * sensor_dt_test_nodes - sensor device tree test node definitions
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __SENSOR_DT_TEST_NODES_H__
#define __SENSOR_DT_TEST_NODES_H__

struct sv_dt_node;

int sv_dt_make_root_node_props(struct sv_dt_node *node);
int sv_dt_make_modeX_node_props(struct sv_dt_node *node);

#endif // __SENSOR_DT_TEST_NODES_H__
