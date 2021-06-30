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

#ifndef __NVMAP_CLIENT_H
#define __NVMAP_CLIENT_H

struct nvmap_client *nvmap_client_create(struct list_head *dev_client_list,
						const char *name);
void nvmap_client_destroy(struct nvmap_client *client);

void nvmap_client_remove_ref(struct nvmap_client *client,
					struct nvmap_handle_ref *ref);
void nvmap_client_add_ref(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref);
struct nvmap_handle_ref *nvmap_client_to_handle_ref(struct nvmap_client *client,
					struct nvmap_handle *handle);

int nvmap_client_add_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle);
void nvmap_client_remove_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle);

int nvmap_client_create_handle(struct nvmap_client *client, size_t size);

int nvmap_client_create_fd(struct nvmap_client *client);


void nvmap_client_warn_if_bad_heap(struct nvmap_client *client,
				u32 heap_type, u32 userflags);
void nvmap_client_warn_if_no_tag(struct nvmap_client *client,
					unsigned int flags);

pid_t nvmap_client_pid(struct nvmap_client *client);

void nvmap_client_stats_alloc(struct nvmap_client *client, size_t size);

const char *nvmap_client_name(struct nvmap_client *client);

void nvmap_client_stringify(struct nvmap_client *client, struct seq_file *s);
void nvmap_client_allocations_stringify(struct nvmap_client *client,
				  struct seq_file *s, u32 heap_type);
void nvmap_client_maps_stringify(struct nvmap_client *client,
				struct seq_file *s, u32 heap_type);
u64 nvmap_client_calc_mss(struct nvmap_client *client, u32 heap_type);
int nvmap_client_show_by_pid(struct nvmap_client *client, struct seq_file *s,
				pid_t pid);
u64 nvmap_handle_procrank_walk(struct nvmap_handle *h, struct mm_walk *walk,
		pid_t client_pid);
void nvmap_client_calc_iovmm_mss(struct nvmap_client *client, u64 *pss,
				   u64 *total);

struct nvmap_client *nvmap_client_from_list(struct list_head *n);
void nvmap_client_del_list(struct nvmap_client *client);

#endif /* __NVMAP_CLIENT_H */
