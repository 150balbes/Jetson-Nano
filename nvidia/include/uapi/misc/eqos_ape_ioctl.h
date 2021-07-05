/*
 * include/uapi/misc/eqos_ape_ioctl.h
 *
 * eqos_ape_ioctl.h  --  EQOS and APE Clock synchronization driver IO control
 *
 * Copyright (c) 2016-2020 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHIN
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __UAPI_EQOS_APE_IOCTL_H__
#define __UAPI_EQOS_APE_IOCTL_H__

#include <linux/ioctl.h>

struct eqos_ape_cmd {
	int ppm;
};

struct eqos_ape_sync_cmd {
	__u64 drift_num;
	__u64 drift_den;
};

struct rate_to_time_period {
	unsigned int rate;
	unsigned int n_int;
	unsigned int n_fract;
	unsigned int n_modulo;
};

enum {
	EQOS_APE_AMISC_INIT = _IO(0xF9, 0x01),
	EQOS_APE_AMISC_DEINIT = _IO(0xF9, 0x02),
	EQOS_APE_AMISC_FREQ_SYNC = _IO(0xF9, 0x03),
	EQOS_APE_AMISC_PHASE_SYNC = _IO(0xF9, 0x04),
	EQOS_APE_TEST_FREQ_ADJ = _IOW(0xF9, 0x05, struct eqos_ape_cmd),
	EQOS_APE_AMISC_GET_RATE = _IO(0xF9, 0x06)
};

#endif

