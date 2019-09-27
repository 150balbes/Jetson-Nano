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
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <stdarg.h>

#include "tegracam_log.h"

static DEFINE_MUTEX(global_log_mutex);
static int (*global_log)(const char *fmt, va_list args);

int camtest_log(const char *fmt, ...)
{
	va_list args;
	int ret;

	if (fmt == NULL)
		return -EINVAL;

	va_start(args, fmt);
	if (global_log == NULL)
		ret = vprintk(fmt, args);
	else
		ret = global_log(printk_skip_level(fmt), args);
	va_end(args);

	return ret;
}
EXPORT_SYMBOL_GPL(camtest_log);

int camtest_try_acquire_global_log(int (*log)(const char *fmt, va_list args))
{
	if (log == NULL)
		return -EINVAL;

	if (!mutex_trylock(&global_log_mutex))
		return -EBUSY;

	global_log = log;

	return 0;
}
EXPORT_SYMBOL_GPL(camtest_try_acquire_global_log);

void camtest_release_global_log(void)
{
	global_log = NULL;
	mutex_unlock(&global_log_mutex);
}
EXPORT_SYMBOL_GPL(camtest_release_global_log);
