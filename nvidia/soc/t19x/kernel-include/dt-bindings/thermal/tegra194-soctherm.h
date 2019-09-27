/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef BPMP_ABI_MACH_T194_SOCTHERM_H
#define BPMP_ABI_MACH_T194_SOCTHERM_H

/**
 * @file
 * @defgroup bpmp_soctherm_ids Soctherm ID's
 * @{
 *   @defgroup bpmp_soctherm_throt_ids Throttle Identifiers
 *   @defgroup bpmp_soctherm_edp_oc_ids EDP/OC Identifiers
 *   @defgroup bpmp_soctherm_throt_modes Throttle Modes
 * @}
 */

/**
 * @addtogroup bpmp_soctherm_throt_ids
 * @{
 */
#define SOCTHERM_THROT_VEC_LITE		0
#define SOCTHERM_THROT_VEC_HEAVY	1
#define SOCTHERM_THROT_VEC_OC1		2
#define SOCTHERM_THROT_VEC_OC2		3
#define SOCTHERM_THROT_VEC_OC3		4
#define SOCTHERM_THROT_VEC_OC4		5
#define SOCTHERM_THROT_VEC_OC5		6
#define SOCTHERM_THROT_VEC_OC6		7
#define SOCTHERM_THROT_VEC_INVALID	8
/** @} */

/**
 * @addtogroup bpmp_soctherm_edp_oc_ids
 * @{
 */
#define SOCTHERM_EDP_OC1	0
#define SOCTHERM_EDP_OC2	1
#define SOCTHERM_EDP_OC3	2
#define SOCTHERM_EDP_OC4	3
#define SOCTHERM_EDP_OC5	4
#define SOCTHERM_EDP_OC6	5
#define SOCTHERM_EDP_OC_INVALID	6
/** @} */

/**
 * @addtogroup bpmp_soctherm_throt_modes
 */
#define SOCTHERM_EDP_OC_MODE_BRIEF	2

#endif
