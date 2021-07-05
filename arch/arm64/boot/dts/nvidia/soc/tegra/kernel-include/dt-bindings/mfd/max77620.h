/*
 * This header provides macros for MAXIM MAX77620 device bindings.
 *
 * Copyright (c) 2013-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _DT_BINDINGS_MFD_MAX77620_H
#define _DT_BINDINGS_MFD_MAX77620_H

/* MAX77620 interrupts */
#define MAX77620_IRQ_TOP_GLBL		0 /* Low-Battery */
#define MAX77620_IRQ_TOP_SD		1 /* SD power fail */
#define MAX77620_IRQ_TOP_LDO		2 /* LDO power fail */
#define MAX77620_IRQ_TOP_GPIO		3 /* GPIO internal int to MAX77620 */
#define MAX77620_IRQ_TOP_RTC		4 /* RTC */
#define MAX77620_IRQ_TOP_32K		5 /* 32kHz oscillator */
#define MAX77620_IRQ_TOP_ONOFF		6 /* ON/OFF oscillator */
#define MAX77620_IRQ_LBT_MBATLOW	7 /* Thermal alarm status, > 120C */
#define MAX77620_IRQ_LBT_TJALRM1	8 /* Thermal alarm status, > 120C */
#define MAX77620_IRQ_LBT_TJALRM2	9 /* Thermal alarm status, > 140C */

/* FPS event source */
#define MAX77620_FPS_EVENT_SRC_EN0		0
#define MAX77620_FPS_EVENT_SRC_EN1		1
#define MAX77620_FPS_EVENT_SRC_SW		2
#define MAX77620_FPS_EVENT_SRC_RSVD		3

/* Device state when FPS event LOW  */
#define MAX77620_FPS_INACTIVE_STATE_SLEEP	0
#define MAX77620_FPS_INACTIVE_STATE_LOW_POWER	1

/* FPS time period */
#define FPS_TIME_PERIOD_40US	0
#define FPS_TIME_PERIOD_80US	1
#define FPS_TIME_PERIOD_160US	2
#define FPS_TIME_PERIOD_320US	3
#define FPS_TIME_PERIOD_640US	4
#define FPS_TIME_PERIOD_1280US	5
#define FPS_TIME_PERIOD_2560US	6
#define FPS_TIME_PERIOD_5120US	7
#define FPS_TIME_PERIOD_DEF	8

/* FPS source */
#define MAX77620_FPS_SRC_0			0
#define MAX77620_FPS_SRC_1			1
#define MAX77620_FPS_SRC_2			2
#define MAX77620_FPS_SRC_NONE			3
#define MAX77620_FPS_SRC_DEF			4

#define	FPS_POWER_PERIOD_0	0
#define	FPS_POWER_PERIOD_1	1
#define	FPS_POWER_PERIOD_2	2
#define	FPS_POWER_PERIOD_3	3
#define	FPS_POWER_PERIOD_4	4
#define	FPS_POWER_PERIOD_5	5
#define	FPS_POWER_PERIOD_6	6
#define	FPS_POWER_PERIOD_7	7
#define	FPS_POWER_PERIOD_DEF	8

#endif
