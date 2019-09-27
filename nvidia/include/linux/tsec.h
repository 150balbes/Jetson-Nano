/*
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION, All rights reserved.
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
#ifndef __LINUX_PUBLIC_TSEC_H
#define __LINUX_PUBLIC_TSEC_H

extern struct hdcp_context_t *hdcp_context;

#ifdef CONFIG_DRM_TEGRA
static inline int tsec_hdcp_create_context(struct hdcp_context_t *hdcp_context)
{
	return -ENOENT;
}

static inline int tsec_hdcp_free_context(struct hdcp_context_t *hdcp_context)
{
	return -ENOENT;
}

static inline void tsec_send_method(struct hdcp_context_t *hdcp_context,
			u32 method,
			u32 flags)
{
	return;
}
#else
extern int tsec_hdcp_create_context(struct hdcp_context_t *hdcp_context);
extern int tsec_hdcp_free_context(struct hdcp_context_t *hdcp_context);
extern void tsec_send_method(struct hdcp_context_t *hdcp_context,
			u32 method,
			u32 flags);
#endif

#endif /*__LINUX_PUBLIC_TSEC_H*/
