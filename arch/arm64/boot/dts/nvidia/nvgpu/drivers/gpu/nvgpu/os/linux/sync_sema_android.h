/*
 * Semaphore Sync Framework Integration
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef _GK20A_SYNC_H_
#define _GK20A_SYNC_H_

struct sync_timeline;
struct sync_fence;
struct sync_pt;
struct nvgpu_semaphore;
struct fence;

#ifdef CONFIG_SYNC
struct sync_timeline *gk20a_sync_timeline_create(const char *name);
void gk20a_sync_timeline_destroy(struct sync_timeline *);
void gk20a_sync_timeline_signal(struct sync_timeline *);
struct sync_fence *gk20a_sync_fence_create(
		struct channel_gk20a *c,
		struct nvgpu_semaphore *,
		const char *fmt, ...);
struct sync_fence *gk20a_sync_fence_fdget(int fd);
struct nvgpu_semaphore *gk20a_sync_pt_sema(struct sync_pt *spt);
#else
static inline void gk20a_sync_timeline_destroy(struct sync_timeline *obj) {}
static inline void gk20a_sync_timeline_signal(struct sync_timeline *obj) {}
static inline struct sync_fence *gk20a_sync_fence_fdget(int fd)
{
	return NULL;
}
static inline struct sync_timeline *gk20a_sync_timeline_create(
	const char *name) {
		return NULL;
}
#endif

#endif
