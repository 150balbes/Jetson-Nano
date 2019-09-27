/*
 * tegracam_log - tegra camera test logging utility
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __TEGRACAM_LOG_H__
#define __TEGRACAM_LOG_H__

#include <stdarg.h>

/**
 * camtest_log - Log a message from a kernel camera test.
 *
 * By default messages are routed to printk (note kernel
 * levels such as KERN_* are supported). However, output can
 * be redirected by acquiring the global log handle. If some
 * entity owns the global log handle output will be directed
 * there instead.
 *
 * @fmt: format and args
 */
int camtest_log(const char *fmt, ...);

/**
 * camtest_try_acquire_global_log - Try to set global camera test log
 *
 * Allows an entity to redirect all camtest_log() output to a custom
 * implementation. Only one owner of the global log may exist at one time.
 * If 0 is returned acquisition of the log handle was successful.
 * Otherwise -EBUSY is returned. This is a NON-blocking call.
 *
 * A call to camtest_release_global_log() must be performed when output no
 * longer needs to be redirected.
 *
 * @log: custom log handler
 */
int camtest_try_acquire_global_log(int (*log)(const char *fmt, va_list args));

/**
 * camtest_release_global_log - Relinquish control of global camera test log
 */
void camtest_release_global_log(void);

#endif // __TEGRACAM_LOG_H__
