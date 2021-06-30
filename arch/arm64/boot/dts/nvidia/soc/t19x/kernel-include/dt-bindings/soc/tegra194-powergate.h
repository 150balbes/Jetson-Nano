/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef BPMP_ABI_MACH_T194_POWERGATE_T194_H
#define BPMP_ABI_MACH_T194_POWERGATE_T194_H

/**
 * @file
 * @defgroup bpmp_pdomain_ids Power Domain ID's
 * This is a list of power domain IDs provided by the firmware.
 * @note Following partitions are forcefully powered down upon entering SC7 power state.
 *  - TEGRA194_POWER_DOMAIN_PVAA
 *  - TEGRA194_POWER_DOMAIN_PVAB
 *  - TEGRA194_POWER_DOMAIN_DLAA
 *  - TEGRA194_POWER_DOMAIN_DLAB
 *  - TEGRA194_POWER_DOMAIN_CV
 *  - TEGRA194_POWER_DOMAIN_GPU
 * @{
 */
#define TEGRA194_POWER_DOMAIN_AUD	1U
#define TEGRA194_POWER_DOMAIN_DISP	2U
#define TEGRA194_POWER_DOMAIN_DISPB	3U
#define TEGRA194_POWER_DOMAIN_DISPC	4U
#define TEGRA194_POWER_DOMAIN_ISPA	5U
#define TEGRA194_POWER_DOMAIN_NVDECA	6U
#define TEGRA194_POWER_DOMAIN_NVJPG	7U
#define TEGRA194_POWER_DOMAIN_NVENCA	8U
#define TEGRA194_POWER_DOMAIN_NVENCB	9U
#define TEGRA194_POWER_DOMAIN_NVDECB	10U
#define TEGRA194_POWER_DOMAIN_SAX	11U
#define TEGRA194_POWER_DOMAIN_VE	12U
#define TEGRA194_POWER_DOMAIN_VIC	13U
#define TEGRA194_POWER_DOMAIN_XUSBA	14U
#define TEGRA194_POWER_DOMAIN_XUSBB	15U
#define TEGRA194_POWER_DOMAIN_XUSBC	16U
#define TEGRA194_POWER_DOMAIN_PCIEX8A	17U
#define TEGRA194_POWER_DOMAIN_PCIEX4A	18U
#define TEGRA194_POWER_DOMAIN_PCIEX1A	19U
#define TEGRA194_POWER_DOMAIN_NVL	20U
#define TEGRA194_POWER_DOMAIN_PCIEX8B	21U
#define TEGRA194_POWER_DOMAIN_PVAA	22U
#define TEGRA194_POWER_DOMAIN_PVAB	23U
#define TEGRA194_POWER_DOMAIN_DLAA	24U
#define TEGRA194_POWER_DOMAIN_DLAB	25U
#define TEGRA194_POWER_DOMAIN_CV	26U
#define TEGRA194_POWER_DOMAIN_GPU	27U
#define TEGRA194_POWER_DOMAIN_MAX	27U
/** @} */

#endif
