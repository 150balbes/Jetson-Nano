/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/rwsem.h>

void nvgpu_rwsem_init(struct nvgpu_rwsem *rwsem)
{
	init_rwsem(&rwsem->rwsem);
}

void nvgpu_rwsem_up_read(struct nvgpu_rwsem *rwsem)
{
	up_read(&rwsem->rwsem);
}

void nvgpu_rwsem_down_read(struct nvgpu_rwsem *rwsem)
{
	down_read(&rwsem->rwsem);
}

void nvgpu_rwsem_up_write(struct nvgpu_rwsem *rwsem)
{
	up_write(&rwsem->rwsem);
}

void nvgpu_rwsem_down_write(struct nvgpu_rwsem *rwsem)
{
	down_write(&rwsem->rwsem);
}
