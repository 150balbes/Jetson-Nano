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
 * This header defines the trip temperatures for Tegra210b01
 */
#ifndef _DT_BINDINGS_THERMAL_TEGRA210B01_TRIPS_H
#define _DT_BINDINGS_THERMAL_TEGRA210B01_TRIPS_H

/* DFLL trips, in millicelsius */
#define TEGRA210B01_DFLL_THERMAL_FLOOR_0	20000
#define TEGRA210B01_DFLL_THERMAL_FLOOR_1	70000

#define TEGRA210B01_DFLL_THERMAL_CAP_0		63000
#define TEGRA210B01_DFLL_THERMAL_CAP_1		83000

/* GPU DVFS thermal trips, in millicelsius */
#define TEGRA210B01_GPU_DVFS_THERMAL_MIN	-25000
#define TEGRA210B01_GPU_DVFS_THERMAL_TRIP_1	20000
#define TEGRA210B01_GPU_DVFS_THERMAL_TRIP_2	30000
#define TEGRA210B01_GPU_DVFS_THERMAL_TRIP_3	50000
#define TEGRA210B01_GPU_DVFS_THERMAL_TRIP_4	70000
#define TEGRA210B01_GPU_DVFS_THERMAL_TRIP_5	90000

#define TEGRA210B01_GPU_DVFS_THERMAL_CAP_1	84000

/* SoC DVFS thermal trips, in millicelsius */
#define TEGRA210B01_SOC_THERMAL_FLOOR_0		20000
#define TEGRA210B01_SOC_THERMAL_CAP_0		84000

#endif /* _DT_BINDINGS_THERMAL_TEGRA210B01_TRIPS_H */
