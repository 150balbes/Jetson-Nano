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

#ifndef __NVMAP_HANDLE_REF_H
#define __NVMAP_HANDLE_REF_H

/* handle_ref objects are client-local references to an nvmap_handle;
 * they are distinct objects so that handles can be unpinned and
 * unreferenced the correct number of times when a client abnormally
 * terminates */
struct nvmap_handle_ref {
	struct nvmap_handle *handle;
	struct rb_node	node;
	atomic_t	dupes;	/* number of times to free on file close */
};

struct nvmap_handle_ref *nvmap_handle_ref_create(struct nvmap_handle *handle);
void nvmap_handle_ref_free(struct nvmap_handle_ref *ref);

int nvmap_handle_ref_get(struct nvmap_handle_ref *ref);
int nvmap_handle_ref_put(struct nvmap_handle_ref *ref);
int nvmap_handle_ref_count(struct nvmap_handle_ref *ref);

struct rb_node *nvmap_handle_ref_to_node(struct nvmap_handle_ref *ref);

#endif /* __NVMAP_HANDLE_REF_H */
