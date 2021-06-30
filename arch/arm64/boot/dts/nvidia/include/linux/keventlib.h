/*
 * include/linux/keventlib.h
 *
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
 *
 */

#ifndef __KEVENTLIB_H
#define __KEVENTLIB_H

#include <linux/types.h>

int
keventlib_write(int id, void *data, size_t size, uint32_t type, uint64_t ts);

int keventlib_register(size_t size, const char *name,
		       const char *schema, size_t schema_size);
void keventlib_unregister(int id);

#endif  /* __KEVENTLIB_H */
