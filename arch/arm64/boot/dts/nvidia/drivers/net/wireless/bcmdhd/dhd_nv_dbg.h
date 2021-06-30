/*
 * drivers/net/wireless/bcmdhd/dhd_nv_dbg.h
 *
 * NVIDIA Tegra Debug prints for BCMDHD driver
 *
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _dhd_nv_dbg_h_
#define _dhd_nv_dbg_h_

#include <linux/time.h>

#define NV_TIMESTAMP()		\
				do {	\
					struct timeval now;	\
					struct tm date_time;	\
					do_gettimeofday(&now);		\
					time_to_tm(now.tv_sec, -sys_tz.tz_minuteswest * 60, &date_time);	\
					pr_info("[%.2d-%.2d %.2d:%.2d:%.2d.%u] %s: ",	\
						date_time.tm_mon+1, date_time.tm_mday, date_time.tm_hour,	\
						date_time.tm_min, date_time.tm_sec,	\
						(unsigned int)(now.tv_usec/1000), __func__);	\
				} while (0)

#define DHD_NV_PRINT(args) 	\
								do {	\
									NV_TIMESTAMP();\
									pr_cont args;	\
								} while (0)

#define DHD_NV_INFO(args)	do {	DHD_NV_PRINT(args);	} while (0)
#define DHD_NV_DEBUG(args)	do {	DHD_NV_PRINT(args);	} while (0)
#define DHD_NV_ERROR(args)	do {	DHD_NV_PRINT(args);	} while (0)

#endif  /* _dhd_nv_dbg_h_ */

