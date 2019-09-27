/*
 * tegra_dc_ext_priv.h: Declarations for tegradc ext interface.
 *
 * Copyright (c) 2011-2019, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Robert Morell <rmorell@nvidia.com>
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
 */

#ifndef __TEGRA_DC_EXT_PRIV_H
#define __TEGRA_DC_EXT_PRIV_H

#include <linux/cdev.h>
#include <linux/dma-buf.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <uapi/video/tegra_dc_ext.h>

#include "../dc.h"

struct tegra_dc_ext;

struct tegra_dc_ext_user {
	struct tegra_dc_ext	*ext;
};

struct tegra_dc_dmabuf {
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
};

enum {
	TEGRA_DC_Y,
	TEGRA_DC_U,
	TEGRA_DC_V,
	TEGRA_DC_CDE,
	TEGRA_DC_NUM_PLANES,
};

struct tegra_dc_ext_win {
	struct tegra_dc_ext	*ext;

	int			idx;

	struct tegra_dc_ext_user *user;

	struct mutex		lock;

	/* Current dmabuf (if any) for Y, U, V planes */
	struct tegra_dc_dmabuf	*cur_handle[TEGRA_DC_NUM_PLANES];

	struct task_struct	*flip_kthread;
	struct kthread_worker	flip_worker;

	atomic_t		nr_pending_flips;

	struct mutex		queue_lock;

	struct list_head	timestamp_queue;

	bool			enabled;
};

struct tegra_dc_ext {
	struct tegra_dc			*dc;

	struct cdev			cdev;
	struct device			*dev;

	struct tegra_dc_ext_win		win[DC_N_WINDOWS];

	struct {
		struct tegra_dc_ext_user	*user;
		struct tegra_dc_dmabuf		*cur_handle;
		struct mutex			lock;
	} cursor;

	bool				enabled;
	bool				vblank_enabled;

	/* all users that have opened the file descriptor for the ext device */
	atomic_t			users_count;

	/* scanline trigger */
	int			scanline_trigger;
	/* scanline work */
	struct kthread_worker	scanline_worker;
	struct task_struct	*scanline_task;
};

#define TEGRA_DC_EXT_EVENT_MASK_ALL		\
	(TEGRA_DC_EXT_EVENT_HOTPLUG |		\
	 TEGRA_DC_EXT_EVENT_VBLANK |		\
	 TEGRA_DC_EXT_EVENT_BANDWIDTH_INC |	\
	 TEGRA_DC_EXT_EVENT_BANDWIDTH_DEC |	\
	 TEGRA_DC_EXT_EVENT_MODECHANGE)

#define TEGRA_DC_EXT_EVENT_MAX_SZ	16

struct tegra_dc_ext_event_list {
	struct tegra_dc_ext_event	event;
	/* The data field _must_ follow the event field. */
	char				data[TEGRA_DC_EXT_EVENT_MAX_SZ];

	struct list_head		list;
};

struct tegra_dc_ext_control_user {
	struct tegra_dc_ext_control	*control;

	struct list_head		event_list;
	atomic_t			num_events;

	u32				event_mask;

	struct tegra_dc_ext_event_list	event_to_copy;
	loff_t				partial_copy;

	struct mutex			lock;

	struct list_head		list;
};

struct tegra_dc_ext_control {
	struct cdev			cdev;
	struct device			*dev;

	struct list_head		users;

	struct mutex			lock;
};

extern dev_t tegra_dc_ext_devno;
extern struct class *tegra_dc_ext_class;

extern int tegra_dc_ext_pin_window(struct tegra_dc_ext_user *user, u32 id,
				   struct tegra_dc_dmabuf **handle,
				   dma_addr_t *phys_addr);

extern int tegra_dc_ext_cpy_caps_from_user(void __user *user_arg,
				struct tegra_dc_ext_caps **caps_ptr,
				u32 *nr_elements_ptr);

extern int tegra_dc_ext_get_cursor(struct tegra_dc_ext_user *user);
extern int tegra_dc_ext_put_cursor(struct tegra_dc_ext_user *user);
extern int tegra_dc_ext_set_cursor_image(struct tegra_dc_ext_user *user,
					 struct tegra_dc_ext_cursor_image *);
extern int tegra_dc_ext_set_cursor(struct tegra_dc_ext_user *user,
				   struct tegra_dc_ext_cursor *);
extern int tegra_dc_ext_cursor_clip(struct tegra_dc_ext_user *user,
					int *args);

extern int tegra_dc_ext_control_init(void);

extern int tegra_dc_ext_queue_hotplug(struct tegra_dc_ext_control *,
				      int output);
extern int tegra_dc_ext_queue_vblank(struct tegra_dc_ext_control *,
					int output, u64 timestamp);
extern int tegra_dc_ext_queue_modechange(struct tegra_dc_ext_control *,
				      int output);
extern int tegra_dc_ext_queue_bandwidth_renegotiate(
			struct tegra_dc_ext_control *, int output,
			struct tegra_dc_bw_data *data);
extern ssize_t tegra_dc_ext_event_read(struct file *filp, char __user *buf,
				       size_t size, loff_t *ppos);
extern unsigned int tegra_dc_ext_event_poll(struct file *, poll_table *);

extern int tegra_dc_ext_get_num_outputs(void);

#ifdef CONFIG_TEGRA_DC_SCREEN_CAPTURE
extern int  tegra_dc_scrncapt_init(void);
extern int  tegra_dc_scrncapt_exit(void);
extern void  tegra_dc_scrncapt_disp_pause_lock(struct tegra_dc *dc);
extern void  tegra_dc_scrncapt_disp_pause_unlock(struct tegra_dc *dc);
extern int  tegra_dc_scrncapt_pause(
		struct tegra_dc_ext_control_user           *ctlusr,
		struct tegra_dc_ext_control_scrncapt_pause *args);
extern int  tegra_dc_scrncapt_resume(
		struct tegra_dc_ext_control_user            *ctlusr,
		struct tegra_dc_ext_control_scrncapt_resume *args);
extern int  tegra_dc_scrncapt_get_info(
		struct tegra_dc_ext_user *user,
		struct tegra_dc_ext_scrncapt_get_info *args);
extern int  tegra_dc_scrncapt_dup_fbuf(
		struct tegra_dc_ext_user *user,
		struct tegra_dc_ext_scrncapt_dup_fbuf *args);
#else /* !CONFIG_TEGRA_DC_SCREEN_CAPTURE */
static inline int  tegra_dc_scrncapt_init(void)
{
	return 0;
}

static inline int  tegra_dc_scrncapt_exit(void)
{
	return 0;
}

static inline void  tegra_dc_scrncapt_disp_pause_lock(struct tegra_dc *dc) {}
static inline void  tegra_dc_scrncapt_disp_pause_unlock(struct tegra_dc *dc) {}
#endif

extern int tegra_dc_ext_vpulse3(struct tegra_dc_ext *dc,
				struct tegra_dc_ext_scanline_info *args);

#endif /* __TEGRA_DC_EXT_PRIV_H */
