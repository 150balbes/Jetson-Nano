/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_POSIX_IO_H
#define NVGPU_POSIX_IO_H

#include <nvgpu/types.h>
#include <nvgpu/list.h>

struct gk20a;

/**
 * Here lies the interface for a unit test module to interact with the nvgpu IO
 * accessors. This interface provides the ability for a module to react to nvgpu
 * calling nvgpu IO accessors so that nvgpu can handle various HW sequences even
 * when run in unit testing mode.
 *
 * The primary interface is simply callbacks to the unit test module which the
 * module can handle how ever it wishes.
 */

struct nvgpu_reg_access {
	/*
	 * Address of the register write relative to the base of the register
	 * space. I.e you can compare this against values in the HW headers
	 * directly to check what register is being read/written to/from.
	 */
	u32 addr;

	/*
	 * Writes: this is the value being written.
	 * Reads: populate with the value to return.
	 */
	u32 value;
};

struct nvgpu_posix_io_callbacks {
	void (*writel)(struct gk20a *g, struct nvgpu_reg_access *access);
	void (*writel_check)(struct gk20a *g, struct nvgpu_reg_access *access);
	void (*__readl)(struct gk20a *g, struct nvgpu_reg_access *access);
	void (*readl)(struct gk20a *g, struct nvgpu_reg_access *access);
	void (*bar1_writel)(struct gk20a *g, struct nvgpu_reg_access *access);
	void (*bar1_readl)(struct gk20a *g, struct nvgpu_reg_access *access);
	void (*usermode_writel)(struct gk20a *g,
				struct nvgpu_reg_access *access);
};

struct nvgpu_posix_io_callbacks *nvgpu_posix_register_io(
	struct gk20a *g,
	struct nvgpu_posix_io_callbacks *io_callbacks);

struct nvgpu_posix_io_reg_space {
	u32 base;
	u32 size;
	u32 *data;
	struct nvgpu_list_node link;
};

static inline struct nvgpu_posix_io_reg_space *
nvgpu_posix_io_reg_space_from_link(struct nvgpu_list_node *node)
{
	return (struct nvgpu_posix_io_reg_space *)
	   ((uintptr_t)node - offsetof(struct nvgpu_posix_io_reg_space, link));
};

void nvgpu_posix_io_init_reg_space(struct gk20a *g);
int nvgpu_posix_io_get_error_code(struct gk20a *g);
void nvgpu_posix_io_reset_error_code(struct gk20a *g);
int nvgpu_posix_io_add_reg_space(struct gk20a *g, u32 base, u32 size);
struct nvgpu_posix_io_reg_space *nvgpu_posix_io_get_reg_space(struct gk20a *g,
	u32 addr);
void nvgpu_posix_io_delete_reg_space(struct gk20a *g, u32 base);
void nvgpu_posix_io_writel_reg_space(struct gk20a *g, u32 addr, u32 data);
u32 nvgpu_posix_io_readl_reg_space(struct gk20a *g, u32 addr);

struct nvgpu_posix_io_reg_access {
	struct nvgpu_reg_access access;
	struct nvgpu_list_node link;
};

static inline struct nvgpu_posix_io_reg_access *
nvgpu_posix_io_reg_access_from_link(struct nvgpu_list_node *node)
{
	return (struct nvgpu_posix_io_reg_access *)
	   ((uintptr_t)node - offsetof(struct nvgpu_posix_io_reg_access, link));
};

void nvgpu_posix_io_start_recorder(struct gk20a *g);
void nvgpu_posix_io_reset_recorder(struct gk20a *g);
void nvgpu_posix_io_record_access(struct gk20a *g,
	struct nvgpu_reg_access *access);
bool nvgpu_posix_io_check_sequence(struct gk20a *g,
	struct nvgpu_reg_access *sequence, u32 size, bool strict);

#endif
