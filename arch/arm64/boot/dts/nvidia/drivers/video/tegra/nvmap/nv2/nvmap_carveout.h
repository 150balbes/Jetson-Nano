/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVMAP_CARVEOUT_H
#define __NVMAP_CARVEOUT_H

#include "nvmap_structs.h"

int nvmap_carveout_create(const struct nvmap_platform_carveout *co);
void nvmap_carveout_destroy(struct nvmap_carveout_node *node);

struct nvmap_heap_block *nvmap_carveout_alloc(struct nvmap_carveout_node *co,
					phys_addr_t *start,
					size_t len,
					size_t align,
					unsigned int prot,
					int peer);
int nvmap_carveout_is_ivm(struct nvmap_carveout_node *carveout);

int nvmap_carveout_query_peer(struct nvmap_carveout_node *carveout);

int nvmap_carveout_heap_bit(struct nvmap_carveout_node *carveout);

int nvmap_carveout_query_heap_size(struct nvmap_carveout_node *carveout);

struct nvmap_carveout_node *nvmap_carveout_index(
				struct nvmap_carveout_node *node, int i);

u64 nvmap_carveout_ivm(struct nvmap_carveout_node *co,
				struct nvmap_heap_block *b, size_t len);

#endif /* __NVMAP_CARVEOUT_H */
