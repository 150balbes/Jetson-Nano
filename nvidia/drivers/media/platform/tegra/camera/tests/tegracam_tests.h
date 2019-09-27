/*
 * tegracam_tests - tegra camera kernel tests
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
#ifndef __TEGRACAM_TESTS_H__
#define __TEGRACAM_TESTS_H__

/*
 * Tegra Camera Kernel Tests
 */
int sensor_verify_dt(struct device_node *node, const u32 tvcf_version);

#endif // __TEGRACAM_TESTS_H__
