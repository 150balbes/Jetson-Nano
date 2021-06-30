/* Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>


s64 nvs_timestamp(void)
{
	struct timespec64 ts;

	ktime_get_ts64(&ts);
	return timespec_to_ns(&ts);
}
EXPORT_SYMBOL_GPL(nvs_timestamp);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor timestamp module");
MODULE_AUTHOR("NVIDIA Corporation");

