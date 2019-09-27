/*
 * hdmi-audio.h: tegra hdmi audio headers.
 *
 * Copyright (c) 2008-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __MACH_TEGRA_HDMI_AUDIO_H
#define __MACH_TEGRA_HDMI_AUDIO_H

#include <linux/kernel.h>
#include <linux/types.h>

enum {
	AUDIO_FREQ_32K = 32000,
	AUDIO_FREQ_44_1K = 44100,
	AUDIO_FREQ_48K = 48000,
	AUDIO_FREQ_88_2K = 88200,
	AUDIO_FREQ_96K = 96000,
	AUDIO_FREQ_176_4K = 176400,
	AUDIO_FREQ_192K = 192000,
};

enum {
	AUTO = 0,
	SPDIF,
	HDA,
};

#if IS_ENABLED(CONFIG_TEGRA_DC)
int tegra_hdmi_setup_hda_presence(int sor_num);
int tegra_hdmi_setup_audio_freq_source(unsigned audio_freq,
					unsigned audio_source,
					int sor_num);
int tegra_hdmi_audio_null_sample_inject(bool on, int sor_num);

/* switches are registered from display driver, get the names w.r.t dev ID */
int tegra_hda_get_switch_name(int dev_id, char *name);
#else
static inline int tegra_hdmi_setup_hda_presence(int sor_num) { return -ENODEV; }
static inline int tegra_hdmi_setup_audio_freq_source(unsigned audio_freq,
					unsigned audio_source,
					int sor_num) { return -ENODEV; }
static inline int tegra_hdmi_audio_null_sample_inject(bool on, int sor_num) { return -ENODEV; }
static inline int tegra_hda_get_switch_name(int dev_id, char *name) { return -ENODEV; }
#endif


bool is_os_l4t(void);
#endif /* __MACH_TEGRA_HDMI_AUDIO_H */
