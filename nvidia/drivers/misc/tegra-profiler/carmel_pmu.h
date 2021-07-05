/*
 * drivers/misc/tegra-profiler/carmel_pmu.h
 *
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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
 */

#ifndef __QUADD_CARMEL_PMU_H
#define __QUADD_CARMEL_PMU_H

struct quadd_event_source;

struct quadd_event_source *
quadd_carmel_uncore_pmu_init(void);
void quadd_carmel_uncore_pmu_deinit(void);

#endif	/* __QUADD_CARMEL_PMU_H */
