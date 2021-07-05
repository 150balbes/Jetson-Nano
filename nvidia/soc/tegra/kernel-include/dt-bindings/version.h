/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * This header provides macros for different DT binding version used
 * by different OS/kernel versions.
 */

#ifndef _DT_BINDINGS_VERSION_H_
#define _DT_BINDINGS_VERSION_H_

#define DT_VERSION_1		1
#define DT_VERSION_2		2

/**
 * TEGRA_AUDIO_BUS_DT_VERSION: Audio node DT version.
 *		V1: All audio bus are in root node.
 *		V2: All audio bus are in the under aconnect node.
 *
 * TEGRA_POWER_DOMAIN_DT_VERSION: Power domain DT versions.
 *		V1: Use legacy power gating API.
 *		V2: Use BPMP power domain provider.
 *
 * TEGRA_XUSB_PADCONTROL_VERSION
 *		V1: Nv Version of DT bidning
 *		V2: Mainline compatible DT binding.
 *
 * TEGRA_XUSB_DT_VERSION: XUSB DT binding version
 *		V1: Nv Version of DT bidning
 *		V2: Mainline compatible DT binding.
 *
 * TEGRA_XUDC_DT_VERSION: XUDC DT binding version
 *		V1: Nv Version of DT bidning
 *		V2: Mainline compatible DT binding.
 *
 * TEGRA_HSP_DT_VERSION: HSP DT binding version
 *		V1: device property is not provided.
 *		V2: device property is provided.
 *
 * UART_CONSOLE_ON_TTYS0_ONLY: Uart console only on TTYS0. Some OS support
 *			       console port in the ttyS0 only.
 *		1 for those OS which support console only on ttyS0.
 *		0 for those OS which can support console on any ttySx.
 *
 * TEGRA_BOOTARGUMENT_VERSION: Boot argument via chosen node.
 *		V1: Only pass the console.
 *		V2: Pass the android boots parameters.

 * TEGRA_CPUFREQ_DT_VERSION: CPUFREQ DT binding version
 *		V1: the separate EDVD aperture for two clusters.
 *		V2: the unified aperture for two clusters.
 */

/* OS Linux */
#if defined(LINUX_VERSION)
#define _OS_FOUND_
#define TEGRA_CPUFREQ_DT_VERSION		DT_VERSION_2
#if LINUX_VERSION >= 409
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_2
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_2
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_2
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_2
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_2
#define TEGRA_HSP_DT_VERSION			DT_VERSION_2
#define TEGRA_BOOTARGUMENT_VERSION		DT_VERSION_2
#define UART_CONSOLE_ON_TTYS0_ONLY		0
#else
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_1
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_1
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_1
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_1
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_1
#define TEGRA_HSP_DT_VERSION			DT_VERSION_1
#define TEGRA_BOOTARGUMENT_VERSION		DT_VERSION_1
#define UART_CONSOLE_ON_TTYS0_ONLY		1
#endif
#endif

/* OS QNX */
#if defined (__QNX__)
#define _OS_FOUND_
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_1
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_1
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_2
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_2
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_2
#define TEGRA_HSP_DT_VERSION			DT_VERSION_2
#define TEGRA_BOOTARGUMENT_VERSION		DT_VERSION_2
#define TEGRA_CPUFREQ_DT_VERSION		DT_VERSION_2
#define UART_CONSOLE_ON_TTYS0_ONLY		1
#endif

/* OS Integrity */
#if defined( __INTEGRITY)
#define _OS_FOUND_
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_1
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_1
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_1
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_1
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_1
#define TEGRA_HSP_DT_VERSION			DT_VERSION_1
#define TEGRA_BOOTARGUMENT_VERSION		DT_VERSION_1
#define TEGRA_CPUFREQ_DT_VERSION		DT_VERSION_2
#define UART_CONSOLE_ON_TTYS0_ONLY		1
#endif

/* OS LK */
#if defined (__LK__)
#define _OS_FOUND_
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_2
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_2
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_2
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_2
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_2
#define TEGRA_HSP_DT_VERSION			DT_VERSION_1
#define TEGRA_BOOTARGUMENT_VERSION		DT_VERSION_1
#define TEGRA_CPUFREQ_DT_VERSION		DT_VERSION_1
#define TEGRA_GENERIC_CARVEOUT_SUPPORT_ENABLE	0
#define UART_CONSOLE_ON_TTYS0_ONLY		1
#endif

/**
 * If no OS found then abort compilation.
 */
#if !defined(_OS_FOUND_)
#error "Valid OS not found"
#endif

#endif /* _DT_BINDINGS_VERSION_H_ */
