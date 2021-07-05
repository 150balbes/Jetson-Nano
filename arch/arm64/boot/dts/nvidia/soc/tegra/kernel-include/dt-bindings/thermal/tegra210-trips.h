/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * This header defines the trip temperatures for Tegra210
 */
#ifndef _DT_BINDINGS_THERMAL_TEGRA210_TRIPS_H
#define _DT_BINDINGS_THERMAL_TEGRA210_TRIPS_H

/* DFLL trips, in millicelsius */
#define TEGRA210_DFLL_THERMAL_FLOOR_0	15000
#define TEGRA210_DFLL_THERMAL_FLOOR_1	30000
#define TEGRA210_DFLL_THERMAL_FLOOR_2	50000
#define TEGRA210_DFLL_THERMAL_FLOOR_3	70000
#define TEGRA210_DFLL_THERMAL_FLOOR_4	120000

#define TEGRA210_DFLL_THERMAL_CAP_0	66000
#define TEGRA210_DFLL_THERMAL_CAP_1	86000

/* GPU DVFS thermal trips, in millicelsius */
#define TEGRA210_GPU_DVFS_THERMAL_MIN		-25000
#define TEGRA210_GPU_DVFS_THERMAL_TRIP_0	15000
#define TEGRA210_GPU_DVFS_THERMAL_TRIP_1	30000
#define TEGRA210_GPU_DVFS_THERMAL_TRIP_2	50000
#define TEGRA210_GPU_DVFS_THERMAL_TRIP_3	70000

#endif /* _DT_BINDINGS_THERMAL_TEGRA210_TRIPS_H */
