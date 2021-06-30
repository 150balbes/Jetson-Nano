/*
 * sensor_kernel_tests_core - sensor kernel tests module core
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

#ifndef __SENSOR_KERNEL_TESTS_CORE_H__
#define __SENSOR_KERNEL_TESTS_CORE_H__

#include <stdarg.h>

#define SKT_NL_MAX_STR_LEN (1024U)

/**
 * skt_core_(v)log_msg - Log a unicast message to the recipient at portid
 * @portid: destination portid
 * @fmt: format
 * @args: arguments
 *
 * To be used only under test context, that is, when the recipient is
 * an initiator of SKT_CMD_RUN_TESTS.
 *
 * Returns non-zero on failure.
 */
int skt_core_vlog_msg(const u32 portid, const char *fmt, va_list args);
int skt_core_log_msg(const u32 portid, const char *fmt, ...);

#endif // __SENSOR_KERNEL_TESTS_CORE_H__
