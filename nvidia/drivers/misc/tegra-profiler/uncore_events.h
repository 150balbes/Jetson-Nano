/*
 * drivers/misc/tegra-profiler/uncore_events.h
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

#ifndef __UNCORE_EVENTS_H
#define __UNCORE_EVENTS_H

struct quadd_ctx;

int quadd_uncore_start(void);
void quadd_uncore_stop(void);

int quadd_uncore_init(struct quadd_ctx *quadd_ctx);
void quadd_uncore_deinit(void);


#endif	/* __UNCORE_EVENTS_H */
