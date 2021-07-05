/*
 * fb.c: Implements tegra fb interface
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *         Colin Cross <ccross@android.com>
 *         Travis Geiselbrecht <travis@palm.com>
 *
 * Copyright (c) 2010-2019, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/vt_kern.h>
#include <linux/console_struct.h>
#include <linux/console.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>

#include <linux/iommu.h>

#include <asm/atomic.h>
#include <uapi/video/tegrafb.h>
#include <uapi/video/tegra_dc_ext.h>

#include "dc/dc.h"
#include "dc/dc_priv.h"
#include "dc/edid.h"
#include "dc/dc_config.h"

/* Pad pitch to 64-byte boundary. */
#define TEGRA_LINEAR_PITCH_ALIGNMENT 64

#ifdef CONFIG_COMPAT
#define user_ptr(p) ((void __user *)(uintptr_t)(p))
#else
#define user_ptr(p) (p)
#endif

#ifdef CONFIG_ANDROID
#define FB_BUFFERING_FACTOR 2
#else /* L4T and other OSes */
#define FB_BUFFERING_FACTOR 1
#endif
/* support up to 4K (3840x2150) with double buffering */
#define DEFAULT_FBMEM_SIZE	(SZ_32M * FB_BUFFERING_FACTOR) + SZ_8M

struct tegra_fb_info {
	struct tegra_dc_win	win;
	struct tegra_dc_win	blank_win;
	struct platform_device	*ndev;
	struct fb_info		*info;
	bool			valid;

	struct resource		*fb_mem;
	size_t			fb_size;

	int			xres;
	int			yres;
	int			curr_xoffset;
	int			curr_yoffset;

	struct fb_videomode	mode;
	phys_addr_t		phys_start;
	int			kuse_count;
	int			mmap_count;

	char __iomem		*blank_base;	/* Virtual address */
	phys_addr_t		blank_start;
};

/* palette array used by the fbcon */
static u32 pseudo_palette[16];

static void *tegra_fb_check_and_alloc_framebuffer(struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct tegra_dc *dc = tegra_fb->win.dc;
	struct tegra_fb_data *fb_data = dc->pdata->fb;
	phys_addr_t fb_size;
	void *fb_base = NULL;

	if (tegra_fb->fb_size)
		return tegra_fb->win.virt_addr;

	fb_size = fb_data->fbmem_size ? : DEFAULT_FBMEM_SIZE;
	fb_base = dma_alloc_coherent(&tegra_fb->ndev->dev,
			fb_size, &tegra_fb->phys_start, GFP_KERNEL);
	if (!fb_base) {
		dev_err(&tegra_fb->ndev->dev,
		"failed to allocate framebuffer\n");
		return NULL;
	}

	info->screen_base = fb_base;
	info->screen_size = fb_size;
	info->fix.smem_len = fb_size;
	tegra_fb->win.virt_addr = fb_base;
	tegra_fb->win.phys_addr = tegra_fb->phys_start;
	tegra_fb->fb_size = fb_size;
	tegra_fb->valid = true;

	return fb_base;
}

int tegra_fb_release_fbmem(struct tegra_fb_info *info)
{
	if (info && info->phys_start) {
		/* do not release the framebuffer if:
		 * 1) kernel is using (e.g. fbcon)
		 * 2) user space has mmap
		 */
		if (info->kuse_count || info->mmap_count)
			return -EBUSY;

		dma_free_coherent(&info->ndev->dev,
				info->fb_size,
				info->win.virt_addr,
				info->phys_start);

		info->phys_start = 0;
		info->win.virt_addr = NULL;
	}
	return 0;
}

static int tegra_fb_open(struct fb_info *fb_info, int user)
{
	struct tegra_fb_info *info = fb_info->par;

	if (user == 0)
		info->kuse_count++;
	return 0;
}

static int tegra_fb_release(struct fb_info *fb_info, int user)
{
	struct tegra_fb_info *info = fb_info->par;

	if (user == 0)
		info->kuse_count--;
	return 0;
}

static int tegra_fb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct tegra_dc *dc = tegra_fb->win.dc;
	struct tegra_dc_out_ops *ops = dc->out_ops;
	struct fb_videomode mode;

	if (tegra_fb->valid &&
		(var->yres * var->xres * var->bits_per_pixel /
		 8 * FB_BUFFERING_FACTOR) > info->screen_size) {
		dev_err(&tegra_fb->ndev->dev,
			"FB %lu is NOT enough for %ux%u %ubpp!\n",
			info->screen_size, var->xres, var->yres,
			var->bits_per_pixel);
		return -EINVAL;
	}

	/* Apply mode filter for HDMI only -LVDS supports only fix mode */
	if (ops && ops->mode_filter) {
		/* xoffset and yoffset are not preserved by conversion
		 * to fb_videomode */
		__u32 xoffset = var->xoffset;
		__u32 yoffset = var->yoffset;

		fb_var_to_videomode(&mode, var);
		if (!ops->mode_filter(dc, &mode))
			return -EINVAL;

		/* Mode filter may have modified the mode */
		fb_videomode_to_var(var, &mode);

		var->xoffset = xoffset;
		var->yoffset = yoffset;
	}

	var->yres_virtual = var->yres * FB_BUFFERING_FACTOR;

	return 0;
}

static int tegra_fb_set_par(struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct tegra_dc *dc = tegra_fb->win.dc;

	if (var->bits_per_pixel) {
		/* we only support RGB ordering for now */
		switch (var->bits_per_pixel) {
		case 32:
			var->red.offset = 16;
			var->red.length = 8;
			var->green.offset = 8;
			var->green.length = 8;
			var->blue.offset = 0;
			var->blue.length = 8;
			var->transp.offset = 24;
			var->transp.length = 8;
			tegra_fb->win.fmt = TEGRA_DC_EXT_FMT_T_A8R8G8B8;
			break;
		case 16:
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			tegra_fb->win.fmt = TEGRA_DC_EXT_FMT_T_R5G6B5;
			break;

		default:
			return -EINVAL;
		}
		/* if line_length unset, then pad the stride */
		if (!info->fix.line_length) {
			info->fix.line_length = var->xres * var->bits_per_pixel
				/ 8;
			info->fix.line_length = round_up(info->fix.line_length,
						TEGRA_LINEAR_PITCH_ALIGNMENT);
		}
		tegra_fb->win.stride = info->fix.line_length;
		tegra_fb->win.stride_uv = 0;
		tegra_fb->win.phys_addr_u = 0;
		tegra_fb->win.phys_addr_v = 0;
	}

	if (var->pixclock) {
		bool stereo;
		unsigned old_len = 0;
		struct fb_videomode m;
		const struct fb_videomode __maybe_unused *match_mode = NULL;
		struct fb_videomode *old_mode = NULL;
		struct tegra_fb_info *tegra_fb = info->par;

		fb_var_to_videomode(&m, var);

#if defined(CONFIG_FB_MODE_PIXCLOCK_HZ)
		match_mode = fb_match_mode(var, &info->modelist);
		if (match_mode && match_mode->pixclock_hz) {
			m.pixclock_hz = match_mode->pixclock_hz;
			/* Update pixel clock if fractional mode is requested */
			if ((var->vmode & FB_VMODE_1000DIV1001) &&
				!(match_mode->vmode & FB_VMODE_1000DIV1001)) {
				u64 pixclock_hz = 0;
				u64 frac_pixclock_hz = 0;

				/* u64 needed to avoid overflow */
				pixclock_hz = m.pixclock_hz;
				frac_pixclock_hz =
					pixclock_hz * 1000 / 1001;
				m.pixclock_hz = (u32)frac_pixclock_hz;
			}
		}
#endif
		/* Load framebuffer info with new mode details*/
		old_mode = info->mode;
		old_len  = info->fix.line_length;
		memcpy(&tegra_fb->mode, &m, sizeof(tegra_fb->mode));
		info->mode = (struct fb_videomode *)&tegra_fb->mode;
		if (!info->mode) {
			dev_warn(&tegra_fb->ndev->dev, "can't match video mode\n");
			info->mode = old_mode;
			return -EINVAL;
		}

		/* Update fix line_length and window stride as per new mode */
		info->fix.line_length = var->xres * var->bits_per_pixel / 8;
		info->fix.line_length = round_up(info->fix.line_length,
			TEGRA_LINEAR_PITCH_ALIGNMENT);
		tegra_fb->win.stride = info->fix.line_length;

		/*
		 * only enable stereo if the mode supports it and
		 * client requests it
		 */
		stereo = !!(var->vmode & info->mode->vmode &
					FB_VMODE_STEREO_FRAME_PACK);

		/* Configure DC with new mode */
		if (tegra_dc_set_fb_mode(dc, info->mode, stereo)) {
			/* Error while configuring DC, fallback to old mode */
			dev_warn(&tegra_fb->ndev->dev, "can't configure dc with mode %ux%u\n",
				info->mode->xres, info->mode->yres);
			info->mode = old_mode;
			info->fix.line_length = old_len;
			tegra_fb->win.stride = old_len;
			return -EINVAL;
		}

		tegra_fb->win.w.full = dfixed_const(info->mode->xres);
		tegra_fb->win.h.full = dfixed_const(info->mode->yres);
		tegra_fb->win.out_w = info->mode->xres;
		tegra_fb->win.out_h = info->mode->yres;
	}
	return 0;
}

static int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
	    info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
		u32 v;

		if (regno >= 16)
			return -EINVAL;

		red = (red >> (16 - info->var.red.length));
		green = (green >> (16 - info->var.green.length));
		blue = (blue >> (16 - info->var.blue.length));

		v = (red << var->red.offset) |
			(green << var->green.offset) |
			(blue << var->blue.offset);

		((u32 *)info->pseudo_palette)[regno] = v;
	}

	return 0;
}


static int tegra_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct tegra_dc *dc = tegra_fb->win.dc;
	int i;
	u16 *red = cmap->red;
	u16 *green = cmap->green;
	u16 *blue = cmap->blue;
	int start = cmap->start;

	if (((unsigned)start > 255) || ((start + cmap->len) > 256))
		return -EINVAL;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
		info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
		/*
		 * For now we are considering color schemes with
		 * cmap->len <=16 as special case of basic color
		 * scheme to support fbconsole.But for DirectColor
		 * visuals(like the one we actually have, that include
		 * a HW LUT),the way it's intended to work is that the
		 * actual LUT HW is programmed to the intended values,
		 * even for small color maps like those with 16 or fewer
		 * entries. The pseudo_palette is then programmed to the
		 * identity transform.
		 */
		if (cmap->len <= 16) {
			/* Low-color schemes like fbconsole*/
			u16 *transp = cmap->transp;
			u_int vtransp = 0xffff;

			for (i = 0; i < cmap->len; i++) {
				if (transp)
					vtransp = *transp++;
				if (tegra_fb_setcolreg(start++, *red++,
					*green++, *blue++,
					vtransp, info))
						return -EINVAL;
			}
		} else {
			/* High-color schemes*/
			for (i = 0; i < cmap->len; i++) {
				if (tegra_dc_is_t21x()) {
					dc->fb_lut.r[start+i] = *red++ >> 8;
					dc->fb_lut.g[start+i] = *green++ >> 8;
					dc->fb_lut.b[start+i] = *blue++ >> 8;
				}
				/*
				 * TODO: This path is not verified. HW pipeline
				 * in T210 and earlier chips was of 8 bits,
				 * which is not the case for T186. This code
				 * needs to be updated accordingly.
				 */
				if (tegra_dc_is_nvdisplay()) {
					dc->fb_nvdisp_lut.rgb[start+i] =
						(((u64)*red++ >> 8) |
						(((u64)*green++ >> 8) << 16) |
						(((u64)*blue++ >> 8) << 32));
				}
			}
			tegra_dc_update_lut(dc, -1, -1);
		}
	}
	return 0;
}

static int tegra_fb_blank(int blank, struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct tegra_dc *dc = tegra_fb->win.dc;
	struct device *pdev = &tegra_fb->ndev->dev;
	int ret = 0;
	/* note if there has been a mode change */
	bool mode_dirty = dc->mode_dirty;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		dev_info(pdev, "unblank\n");
		tegra_dc_enable(dc);

		if (!dc->suspended && dc->blanked) {
			/*
			 * Do not restore old EXT windows if mode has been
			 * updated. EXT window parameters will be updated as
			 * required for the new mode in upcoming flip call.
			 */
			if (!mode_dirty)
				ret = tegra_dc_restore(dc);
			/*
			 * Update only valid fb window, i.e. win.idx != -1
			 * and only if fb window is mapped.
			 */
			if (!ret &&
			    tegra_fb->valid &&
			    tegra_fb->win.idx != ((u8)-1)) {
				struct tegra_dc_win *win = &tegra_fb->win;

				ret = tegra_dc_update_windows(&win, 1, NULL,
						true, false);
				if (ret) {
					dev_info(pdev,
						"update windows ret = %d\n",
						ret);
				}
				ret = tegra_dc_sync_windows(&win, 1);
				if (ret) {
					dev_info(pdev,
						"sync windows ret = %d\n",
						ret);
				}
				tegra_dc_program_bandwidth(dc, true);
			}
		}

		dc->blanked = false;
		return 0;

	case FB_BLANK_NORMAL:
		dev_info(pdev, "blank - normal\n");
		/* To pan fb at the unblank */
		if (dc->enabled)
			tegra_fb->curr_xoffset = -1;
		dc->blanked = true;
		if (tegra_fb_is_console_enabled(dc->pdata))
			tegra_dc_cursor_suspend(dc);
		tegra_dc_blank_wins(dc, BLANK_ALL);
		return 0;

	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		dev_info(pdev, "blank - powerdown\n");
		/* To pan fb while switching from X */
		if (!dc->suspended && dc->enabled)
			tegra_fb->curr_xoffset = -1;

		if (dc->enabled)
			tegra_dc_disable(dc);
		if (tegra_fb_is_console_enabled(dc->pdata) &&
					fbcon_is_fgconsole())
			dc->blanked = true;

		return 0;

	default:
		return -ENOTTY;
	}
}

/* If there is a console active, make it visible again */
int tegra_fb_redisplay_console(struct tegra_fb_info *tegra_info)
{
	int ret;

	console_lock();
	if (tegra_info->info->state == FBINFO_STATE_RUNNING) {
		struct fb_event event;
		int blank;

		event.info = tegra_info->info;
		event.data = &blank;
		blank = FB_BLANK_NORMAL;
		ret = fb_notifier_call_chain(FB_EVENT_BLANK, &event);
		blank = FB_BLANK_UNBLANK;
		ret = fb_notifier_call_chain(FB_EVENT_BLANK, &event);
	} else {
		ret = -ENODEV;
	}
	console_unlock();
	return ret;
}

static int tegra_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	char __iomem *flush_start;
	char __iomem *flush_end;
	phys_addr_t    addr;

	/*
	 * Do nothing if display parameters are same as current values.
	 */
#if defined(CONFIG_ANDROID)
	if ((var->xoffset == tegra_fb->curr_xoffset) &&
	    (var->yoffset == tegra_fb->curr_yoffset) &&
	    !(var->activate & FB_ACTIVATE_FORCE))
		return 0;
#endif

	if (!tegra_fb->valid && !tegra_fb_check_and_alloc_framebuffer(info))
		return -ENOMEM;

	if (!tegra_fb->win.cur_handle) {
		flush_start = info->screen_base +
		(var->yoffset * info->fix.line_length);
		flush_end = flush_start + (var->yres * info->fix.line_length);

		info->var.xoffset = var->xoffset;
		info->var.yoffset = var->yoffset;
		/*
		 * Save previous values of xoffset and yoffset so we can
		 * pan display only when needed.
		 */
		tegra_fb->curr_xoffset = var->xoffset;
		tegra_fb->curr_yoffset = var->yoffset;

		addr = tegra_fb->phys_start + (var->yoffset * info->fix.line_length) +
			(var->xoffset * (var->bits_per_pixel/8));

		tegra_fb->win.phys_addr = addr;
		tegra_fb->win.flags = TEGRA_WIN_FLAG_ENABLED;
		tegra_fb->win.flags |= TEGRA_WIN_FLAG_FB;
		tegra_fb->win.virt_addr = info->screen_base;

		if (!tegra_fb->win.dc->suspended) {
			struct tegra_dc_win *win = &tegra_fb->win;
			tegra_dc_update_windows(&win, 1, NULL,
							true, false);
			tegra_dc_sync_windows(&win, 1);
			tegra_dc_program_bandwidth(win->dc, true);
		}
	}

	return 0;
}

static void tegra_fb_fillrect(struct fb_info *info,
			      const struct fb_fillrect *rect)
{
	struct tegra_fb_info *tegra_fb = info->par;

	if (!tegra_fb->valid && !tegra_fb_check_and_alloc_framebuffer(info))
		return;

	cfb_fillrect(info, rect);
}

static void tegra_fb_copyarea(struct fb_info *info,
			      const struct fb_copyarea *region)
{
	struct tegra_fb_info *tegra_fb = info->par;

	if (!tegra_fb->valid && !tegra_fb_check_and_alloc_framebuffer(info))
		return;

	cfb_copyarea(info, region);
}

static void tegra_fb_imageblit(struct fb_info *info,
			       const struct fb_image *image)
{
	struct tegra_fb_info *tegra_fb = info->par;

	if (!tegra_fb->valid && !tegra_fb_check_and_alloc_framebuffer(info))
		return;

	cfb_imageblit(info, image);
}

static int tegra_get_modedb(struct tegra_dc *dc, struct tegra_fb_modedb *modedb,
	struct fb_info *info)
{
	unsigned i;
	struct fb_var_screeninfo __user *modedb_ptr = NULL;
	struct fb_modelist *modelist = NULL;

	i = 0;

	if (list_empty(&info->modelist)) {
		/* if modelist empty, return modelength as 0 */
		modedb->modedb_len = 0;
		return 0;
	}

	modedb_ptr = user_ptr(modedb->modedb);

	if (modedb->modedb_len == 0) {
		list_for_each_entry(modelist, &info->modelist, list) {
			i++;

			if (modelist->mode.vmode & FB_VMODE_STEREO_MASK)
				i++;
		}
		modedb->modedb_len = i;
		return 0;
	}

	i = 0;

	list_for_each_entry(modelist, &info->modelist, list) {
		struct fb_var_screeninfo var;

		/* fb_videomode_to_var doesn't fill out all the members
		   of fb_var_screeninfo */
		memset(&var, 0x0, sizeof(var));

		fb_videomode_to_var(&var, &modelist->mode);
		var.width = tegra_dc_get_out_width(dc);
		var.height = tegra_dc_get_out_height(dc);
		var.bits_per_pixel = dc->pdata->fb->bits_per_pixel;
		if (i < modedb->modedb_len) {
			void __user *ptr = &modedb_ptr[i];

			if (copy_to_user(ptr, &var, sizeof(var)))
				return -EFAULT;
		}
		i++;

		if (var.vmode & FB_VMODE_STEREO_MASK) {
			if (i < modedb->modedb_len) {
				void __user *ptr = &modedb_ptr[i];

				var.vmode &= ~FB_VMODE_STEREO_MASK;
				if (copy_to_user(ptr,
					&var, sizeof(var)))
					return -EFAULT;
			}
			i++;
		}
	}
	modedb->modedb_len = min(modedb->modedb_len, i);

	return 0;
}

static int tegra_fb_ioctl(struct fb_info *info,
	unsigned int cmd, unsigned long arg)
{
	int res;
	struct tegra_fb_info *tegra_fb = (struct tegra_fb_info *)info->par;
	struct tegra_dc *dc = tegra_fb->win.dc;
	struct tegra_fb_modedb __user modedb;
	struct fb_vblank vblank = {};

	switch (cmd) {
#ifdef CONFIG_COMPAT
	case FBIO_TEGRA_GET_MODEDB_COMPAT: {
		struct tegra_fb_modedb_compat modedb_compat;

		if (copy_from_user(&modedb_compat, (void __user *)arg,
			sizeof(modedb_compat)))
			return -EFAULT;
		/* convert compat version to full version */
		modedb.modedb = user_ptr(modedb_compat.modedb);
		modedb.modedb_len = modedb_compat.modedb_len;

		res = tegra_get_modedb(dc, &modedb, info);
		if (res)
			return res;

		/* convert full version back to compat version */
		modedb_compat.modedb_len = modedb.modedb_len;
		if (copy_to_user((void __user *)arg, &modedb_compat,
			sizeof(modedb_compat)))
			return -EFAULT;
		break;
	}
#endif
	case FBIO_TEGRA_GET_MODEDB:
		if (copy_from_user(&modedb, (void __user *)arg, sizeof(modedb)))
			return -EFAULT;

		res = tegra_get_modedb(dc, &modedb, info);
		if (res)
			return res;

		if (copy_to_user((void __user *)arg, &modedb, sizeof(modedb)))
			return -EFAULT;
		break;

	case FBIOGET_VBLANK:
		if (tegra_dc_has_vsync(dc))
			vblank.flags = FB_VBLANK_HAVE_VSYNC;

		if (copy_to_user(
			(void __user *)arg, &vblank, sizeof(vblank)))
			return -EFAULT;
		break;

	case FBIO_WAITFORVSYNC:
		return tegra_dc_wait_for_vsync(dc);

	default:
		return -ENOTTY;
	}

	return 0;
}

int tegra_fb_set_var(struct tegra_dc *dc, struct fb_var_screeninfo *var)
{
	return fb_set_var(dc->fb->info, var);
}

int tegra_fb_update_modelist(struct tegra_dc *dc, int fblistindex)
{
	struct list_head *pos, *n;
	struct fb_info *info = dc->fb->info;
	struct list_head *srclist = &info->modelist;
	int index = 0;

	list_for_each_safe(pos, n, srclist) {
		if (fblistindex) {
			if (index >= fblistindex) {
				/* remove the invalid modes*/
				list_del(pos);
				kfree(pos);
			}
		}
		index += 1;
	}
	return index;
}

static int tegra_fb_get_mode_refresh(struct tegra_dc *dc)
{
	if (!dc->fb->info->mode)
		return -1;
	return dc->fb->info->mode->refresh;
}

struct fb_videomode *tegra_fb_get_mode(struct tegra_dc *dc)
{
	if (dc && dc->fb && dc->fb->info && dc->fb->info->mode)
		return dc->fb->info->mode;
	else
		return NULL;
}

static int tegra_fb_set_mode(struct tegra_dc *dc, int fps)
{
	size_t stereo;
	struct list_head *pos;
	struct fb_videomode *best_mode = NULL;
	int curr_diff = INT_MAX; /* difference of best_mode refresh rate */
	struct fb_modelist *modelist;
	struct fb_info *info = dc->fb->info;

	list_for_each(pos, &info->modelist) {
		struct fb_videomode *mode;

		modelist = list_entry(pos, struct fb_modelist, list);
		mode = &modelist->mode;
		if (fps <= mode->refresh && curr_diff > (mode->refresh - fps)) {
			curr_diff = mode->refresh - fps;
			best_mode = mode;
		}
	}
	if (best_mode) {
		info->mode = best_mode;
		stereo = !!(info->var.vmode & info->mode->vmode &
				FB_VMODE_STEREO_FRAME_PACK);
		return tegra_dc_set_fb_mode(dc, best_mode, stereo);
	}
	return -EIO;
}

static int tegra_fb_mmap(struct fb_info *info,
		struct vm_area_struct *vma)
{
	struct tegra_fb_info *tegra_fb = info->par;

	if (!tegra_fb->valid && !tegra_fb_check_and_alloc_framebuffer(info))
		return -ENOMEM;

	tegra_fb->mmap_count++;
	return dma_mmap_writecombine(&tegra_fb->ndev->dev, vma,
					tegra_fb->win.virt_addr,
					tegra_fb->phys_start,
					tegra_fb->fb_size);
}

static struct fb_ops tegra_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = tegra_fb_open,
	.fb_release = tegra_fb_release,
	.fb_check_var = tegra_fb_check_var,
	.fb_set_par = tegra_fb_set_par,
	.fb_setcmap = tegra_fb_setcmap,
	.fb_blank = tegra_fb_blank,
	.fb_pan_display = tegra_fb_pan_display,
	.fb_fillrect = tegra_fb_fillrect,
	.fb_copyarea = tegra_fb_copyarea,
	.fb_imageblit = tegra_fb_imageblit,
	.fb_ioctl = tegra_fb_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = tegra_fb_ioctl,
#endif
	.fb_mmap = tegra_fb_mmap,
};

/* Enabling the pan_display by resetting the cache of offset */
void tegra_fb_pan_display_reset(struct tegra_fb_info *fb_info)
{
	fb_info->curr_xoffset = -1;
}

#if defined(CONFIG_FRAMEBUFFER_CONSOLE)
bool fbcon_is_fgconsole()
{
	return (vc_cons[fg_console].d->vc_mode == KD_TEXT);
}
#endif

/* Should be invoked by holding console lock */
void tegra_fbcon_set_fb_mode(struct tegra_fb_info *fb_info,
					struct fb_videomode *fb_mode)
{
	struct fb_var_screeninfo var;
	struct tegra_dc *dc = fb_info->win.dc;

	/*
	 * Disable DC and blank console before modeset.
	 * Set MISC_USEREVENT flag to indicate user requested blank and to
	 * update consoles in fb_set_var() below.
	 */
	fb_info->info->flags |= FBINFO_MISC_USEREVENT;
	fb_blank(fb_info->info, FB_BLANK_POWERDOWN);

	fb_videomode_to_var(&var, fb_mode);
	var.bits_per_pixel = dc->pdata->fb->bits_per_pixel;
	/* Set flags to update all available TTYs immediately */
	var.activate = FB_ACTIVATE_NOW | FB_ACTIVATE_ALL;

	/* Set var_screeninfo */
	fb_set_var(fb_info->info, &var);

	/* Enable DC and unblank console */
	fb_info->info->flags |= FBINFO_MISC_USEREVENT;
	fb_blank(fb_info->info, FB_BLANK_UNBLANK);
}

void tegra_fb_update_monspecs(struct tegra_fb_info *fb_info,
			      struct fb_monspecs *specs,
			      bool (*mode_filter)(const struct tegra_dc *dc,
						  struct fb_videomode *mode))

{
	struct fb_event event;
	int i, b_locked_fb_info = 0;
	struct tegra_dc *dc = fb_info->win.dc;

	if (fb_info == NULL) {
		pr_err("%s(): invalid operation\n", __func__);
		return;
	}

	console_lock();
	b_locked_fb_info = lock_fb_info(fb_info->info);
	/*
	 * fb_info modedb shares the same pointer as specs modedb. Avoid freeing
	 * modedb pointer if specs modedb is still valid. This helps avoid using
	 * freed modedb pointer when we copy specs to fb_info monspecs below.
	 */
	if (specs == NULL || specs->modedb != fb_info->info->monspecs.modedb) {
		fb_destroy_modedb(fb_info->info->monspecs.modedb);
		fb_info->info->monspecs.modedb = NULL;
	}
	fb_destroy_modelist(&fb_info->info->modelist);
	event.info = fb_info->info;
	/* Notify layers above fb.c that the hardware is unavailable */
	fb_set_suspend(fb_info->info, true);

	if (specs == NULL) {
		memset(&fb_info->info->monspecs, 0x0,
		       sizeof(fb_info->info->monspecs));

		/*
		 * reset video mode properties to prevent garbage being
		 * displayed on 'mode' device.
		 */
		fb_info->info->mode = (struct fb_videomode*) NULL;

		if (IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE)) {
			/*
			 * Disable DC and blank console. Set MISC_USEREVENT
			 * flag to indicate user requested blank.
			 */
			fb_info->info->flags |= FBINFO_MISC_USEREVENT;
			fb_blank(fb_info->info, FB_BLANK_POWERDOWN);
			/*
			 * fbconsole needs at least one mode in modelist. Add
			 * the existing fb videomode to modelist since it is
			 * already programmed as fb mode and variable screen
			 * info. Notify fbconsole of the new modelist. Mode
			 * shall be updated as part of modeset in next hotplug.
			 */
			fb_add_videomode(&fb_info->mode,
						&fb_info->info->modelist);
			fb_notifier_call_chain(FB_EVENT_NEW_MODELIST, &event);
		} else {
			/* For L4T - After the next hotplug, framebuffer console will
			 * use the old variable screeninfo by default, only video-mode
			 * settings will be overwritten as per monitor connected.
			 */
			memset(&fb_info->info->var, 0x0, sizeof(fb_info->info->var));
		}

		if (b_locked_fb_info)
			unlock_fb_info(fb_info->info);
		console_unlock();
		return;
	}

	memcpy(&fb_info->info->monspecs, specs,
	       sizeof(fb_info->info->monspecs));
	fb_info->info->mode = specs->modedb;

	for (i = 0; i < specs->modedb_len; i++) {
		if (mode_filter) {
			if (mode_filter(dc, &specs->modedb[i]))
				fb_add_videomode(&specs->modedb[i],
						 &fb_info->info->modelist);
		} else {
			fb_add_videomode(&specs->modedb[i],
					 &fb_info->info->modelist);
		}
	}

	if (dc->out_ops->vrr_update_monspecs)
		dc->out_ops->vrr_update_monspecs(dc,
			&fb_info->info->modelist);

	if (tegra_fb_is_console_enabled(dc->pdata)) {
		struct fb_videomode fb_mode;

		if (dc->use_cached_mode) {
			tegra_dc_to_fb_videomode(&fb_mode, &dc->cached_mode);
			dc->use_cached_mode = false;
		} else if (dc->out->fbcon_default_mode) {
			memcpy(&fb_mode, dc->out->fbcon_default_mode,
				sizeof(struct fb_videomode));
		} else if (!list_empty(&fb_info->info->modelist)) {
			struct fb_modelist *modelist;

			modelist = list_first_entry(&fb_info->info->modelist,
						struct fb_modelist, list);
			fb_mode = modelist->mode;
		} else {
			memcpy(&fb_mode, &tegra_dc_vga_mode,
				sizeof(struct fb_videomode));
		}

		fb_info->info->state = FBINFO_STATE_RUNNING;
		tegra_fbcon_set_fb_mode(fb_info, &fb_mode);
	} else {
		if (!IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE))
			fb_notifier_call_chain(FB_EVENT_NEW_MODELIST, &event);
	}
	if (b_locked_fb_info)
		unlock_fb_info(fb_info->info);
	console_unlock();
}

void tegra_fb_update_fix(struct tegra_fb_info *fb_info,
				struct fb_monspecs *specs)
{
	struct tegra_dc *dc = fb_info->win.dc;
	struct tegra_edid *dc_edid = dc->edid;
	struct fb_fix_screeninfo *fix = &fb_info->info->fix;
	int	b_locked_fb_info;

	console_lock();
	b_locked_fb_info = lock_fb_info(fb_info->info);

	/* FB_CAP_* and TEGRA_DC_* color depth flags are shifted by 1 */
	BUILD_BUG_ON((TEGRA_DC_Y420_30 << 1) != FB_CAP_Y420_DC_30);
	BUILD_BUG_ON((TEGRA_DC_RGB_48 << 1) != FB_CAP_RGB_DC_48);
	fix->capabilities = (tegra_edid_get_cd_flag(dc_edid) << 1);
	if (tegra_edid_support_yuv422(dc_edid) &&
		(!(tegra_edid_get_quirks(dc_edid) &
		TEGRA_EDID_QUIRK_NO_YUV_422)))
		fix->capabilities |= FB_CAP_Y422;
	if (tegra_edid_support_yuv444(dc_edid))
		fix->capabilities |= FB_CAP_Y444;
	fix->capabilities |= tegra_edid_get_ex_hdr_cap(dc_edid);

	fix->max_clk_rate = tegra_edid_get_max_clk_rate(dc_edid);

	fix->colorimetry = tegra_edid_get_ex_colorimetry(dc_edid);

	if (b_locked_fb_info)
		unlock_fb_info(fb_info->info);
	console_unlock();
}

struct fb_var_screeninfo *tegra_fb_get_var(struct tegra_fb_info *fb_info)
{
	if (!fb_info || !fb_info->info)
		return NULL;

	return &fb_info->info->var;
}

static ssize_t nvdps_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	int refresh_rate;
	struct platform_device *ndev = to_platform_device(device);
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	refresh_rate = tegra_fb_get_mode_refresh(dc);
	return snprintf(buf, PAGE_SIZE, "%d\n", refresh_rate);
}


static ssize_t nvdps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	int refresh_rate;
	int e;

	e = kstrtoint(buf, 10, &refresh_rate);
	if (e)
		return e;
	e = tegra_fb_set_mode(dc, refresh_rate);
	if (e)
		return e;

	return count;
}

static DEVICE_ATTR(nvdps, 0644, nvdps_show, nvdps_store);


int tegra_fb_create_sysfs(struct device *dev)
{
	return device_create_file(dev, &dev_attr_nvdps);
}

void tegra_fb_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_nvdps);
}

#define BLANK_LINE_WIDTH	2400
#define BLANK_LINE_SIZE		(BLANK_LINE_WIDTH*4)

static void tegra_fb_fill_420_10bpc_blank_frame(struct tegra_dc_win *win)
{
#define phase0(p0)	(p0 & 0xff)
#define phase1(p0, p1)	(((p1 & 0x3f) << 2) | ((p0 & 0x300) >> 8))
#define phase2(p1, p2)	(((p2 & 0xf) << 4) | ((p1 & 0x3c0) >> 6))
#define phase3(p2, p3)	(((p3 & 0x3) << 6) | ((p2 & 0x3f0) >> 4))
#define phase4(p3)	((p3 & 0x3fc) >> 2)
#define p0(y, c, a)	(phase0(a) << 24 | phase0(y) << 16 | \
		phase0(y) << 8 | phase0(c))
#define p1(y, c, a)	(phase1(a, a) << 24 | phase1(y, y) << 16 | \
		phase1(y, y) << 8 | phase1(c, c))
#define p2(y, c, a)	(phase2(a, a) << 24 | phase2(y, y) << 16 | \
		phase2(y, y) << 8 | phase2(c, c))
#define p3(y, c, a)	(phase3(a, a) << 24 | phase3(y, y) << 16 | \
		phase3(y, y) << 8 | phase3(c, c))
#define p4(y, c, a)	(phase4(a) << 24 | phase4(y) << 16 | \
		phase4(y) << 8 | phase4(c))
#define YCC_10BPC_Y_BLACK (64)
#define YCC_10BPC_C_BLACK (512)

	u32 y = YCC_10BPC_Y_BLACK;
	u32 crcb = YCC_10BPC_C_BLACK;
	u32 a = 0;
	u32 active_width = BLANK_LINE_WIDTH;
	u32 temp_w;
	u32 bytes_per_pix;
	char __iomem *mem_start = win->virt_addr;
	u32 offset = 0;

	/* phase statically rendered for TEGRA_DC_EXT_FMT_T_A8R8G8B8 */
	bytes_per_pix = tegra_dc_fmt_bpp(TEGRA_DC_EXT_FMT_T_A8R8G8B8) / 8;

	/* A single line can be repeated through the whole frame height, hence
	 * only the first line needs to be setup. */
	for (temp_w = 0; temp_w < active_width; temp_w += 5) {
		writel_relaxed(p0(y, crcb, a), mem_start +
				offset);
		writel_relaxed(p1(y, crcb, a), mem_start +
				({offset += bytes_per_pix; }));
		writel_relaxed(p2(y, crcb, a), mem_start +
				({offset += bytes_per_pix; }));
		writel_relaxed(p3(y, crcb, a), mem_start +
				({offset += bytes_per_pix; }));
		writel_relaxed(p4(y, crcb, a), mem_start +
				({offset += bytes_per_pix; }));
		offset += bytes_per_pix;
	}

#undef YCC_10BPC_C_BLACK
#undef YCC_10BPC_Y_BLACK
#undef p4
#undef p3
#undef p2
#undef p1
#undef phase4
#undef phase3
#undef phase2
#undef phase1
#undef phase0
}

int tegra_fb_set_win_index(struct tegra_dc *dc, unsigned long win_mask)
{
	int i;
	unsigned long mask = win_mask;
	unsigned stride = 0;
	struct tegra_fb_info *tegra_fb;

	if (!dc || !dc->fb)
		return -1;

	tegra_fb = dc->fb;

	if (win_mask) {
		/* Set the first valid bit as fb win index */
		for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
			if (test_bit(i, &mask))
				break;
		}

		tegra_fb->win.idx = i;
		stride = tegra_dc_get_stride(dc, tegra_fb->win.idx);
		if (stride) {
			tegra_fb->info->fix.line_length = stride;
			tegra_fb->win.stride = stride;
		}
	} else
		tegra_fb->win.idx = -1;

	return 0;
}

struct tegra_dc_win *tegra_fb_get_win(struct tegra_fb_info *tegra_fb)
{
	if (!tegra_fb)
		return NULL;

	return &tegra_fb->win;
}

struct tegra_dc_win *tegra_fb_get_blank_win(struct tegra_fb_info *tegra_fb)
{
	if (!tegra_fb)
		return NULL;

	return &tegra_fb->blank_win;
}

static void tegra_fb_copy_fbmem(void *to, size_t size, struct resource *fb)
{
	size_t i;
	phys_addr_t from = fb->start;

	size = min_t(size_t, size, resource_size(fb));

	if (!from || size <= 0)
		return;

	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);
	BUG_ON(!pfn_valid(page_to_pfn(phys_to_page(from))));

	for (i = 0; i < size; i += PAGE_SIZE) {
		struct page *page = phys_to_page(from + i);
		void *from_virt = kmap(page);

		memcpy(to + i, from_virt, PAGE_SIZE);
		kunmap(page);
	}
}

struct tegra_fb_info *tegra_fb_register(struct platform_device *ndev,
					struct tegra_dc *dc,
					struct tegra_fb_data *fb_data,
					struct resource *fb_mem)
{
	struct fb_info *info;
	struct tegra_fb_info *tegra_fb;
	void *fb_base = NULL;
	phys_addr_t fb_size = 0;
	int ret = 0;
	int mode_idx;
	unsigned stride = 0;
	struct fb_videomode m;
	DEFINE_DMA_ATTRS(attrs);

	if (fb_data->win > -1) {
		if (!tegra_dc_get_window(dc, fb_data->win)) {
			dev_err(&ndev->dev, "dc does not have a window at index %d\n",
				fb_data->win);
			return ERR_PTR(-ENOENT);
		}
	}

	info = framebuffer_alloc(sizeof(struct tegra_fb_info), &ndev->dev);
	if (!info) {
		ret = -ENOMEM;
		goto err;
	}
	info->node = -1;	/* Set by register_framebuffer */
	tegra_fb = info->par;
	tegra_fb->ndev = ndev;
	tegra_fb->fb_mem = fb_mem;
	tegra_fb->xres = fb_data->xres;
	tegra_fb->yres = fb_data->yres;

	tegra_fb->win.idx = fb_data->win;
	tegra_fb->win.dc = dc;

	if (fb_mem && fb_mem->start) {
		fb_base = tegra_fb_check_and_alloc_framebuffer(info);
		if (!fb_base) {
			ret = -EBUSY;
			goto err_free;
		}

		/* copy content from the bootloader framebuffer */
		tegra_fb_copy_fbmem(fb_base, fb_size, fb_mem);
	} else {
		tegra_fb->valid = false;
		tegra_fb->fb_size = 0;
		tegra_fb->phys_start = 0;
		tegra_fb->win.virt_addr = NULL;
		tegra_fb->win.phys_addr = 0;
	}

	dma_set_attr(DMA_ATTR_WRITE_COMBINE, __DMA_ATTR(attrs));
	tegra_fb->blank_base = dma_alloc_attrs(&ndev->dev,
					       BLANK_LINE_SIZE,
					       &tegra_fb->blank_start,
					       GFP_KERNEL,
					       __DMA_ATTR(attrs));
	if (!tegra_fb->blank_base) {
		dev_err(&ndev->dev, "failed to allocate blank buffer\n");
		ret = -EBUSY;
		goto err_free_fbmem;
	}

	info->fix.line_length = fb_data->xres * fb_data->bits_per_pixel / 8;

	if (fb_data->win > -1)
		stride = tegra_dc_get_stride(dc, fb_data->win);
	if (!stride) /* default to pad the stride */
		stride = round_up(info->fix.line_length,
			TEGRA_LINEAR_PITCH_ALIGNMENT);

	info->fbops = &tegra_fb_ops;
	info->pseudo_palette = pseudo_palette;
	info->screen_base = tegra_fb->win.virt_addr;
	info->screen_size = tegra_fb->fb_size;
	info->state = FBINFO_STATE_SUSPENDED;
	strlcpy(info->fix.id, "tegra_fb", sizeof(info->fix.id));
	info->fix.type		= FB_TYPE_PACKED_PIXELS;
	info->fix.visual	= FB_VISUAL_TRUECOLOR;
	info->fix.xpanstep	= 1;
	info->fix.ypanstep	= 1;
	info->fix.accel		= FB_ACCEL_NONE;
	/* Note:- Use tegra_fb_info.phys_start instead of
	 *        fb_info.fix->smem_start when LPAE is enabled. */
	info->fix.smem_start	= (u32)tegra_fb->phys_start;
	info->fix.smem_len	= tegra_fb->fb_size;
	info->fix.line_length = stride;
	INIT_LIST_HEAD(&info->modelist);
	/* pick first mode as the default for initialization */
	tegra_dc_to_fb_videomode(&m, &dc->mode);
	fb_videomode_to_var(&info->var, &m);
	info->var.xres_virtual		= fb_data->xres;
	info->var.yres_virtual		= fb_data->yres * FB_BUFFERING_FACTOR;
	info->var.bits_per_pixel	= fb_data->bits_per_pixel;
	info->var.activate		= FB_ACTIVATE_VBL;
	info->var.height		= tegra_dc_get_out_height(dc);
	info->var.width			= tegra_dc_get_out_width(dc);

	tegra_fb->win.x.full = dfixed_const(0);
	tegra_fb->win.y.full = dfixed_const(0);
	tegra_fb->win.w.full = dfixed_const(fb_data->xres);
	tegra_fb->win.h.full = dfixed_const(fb_data->yres);
	/* TODO: set to output res dc */
	tegra_fb->win.out_x = 0;
	tegra_fb->win.out_y = 0;
	tegra_fb->win.out_w = fb_data->xres;
	tegra_fb->win.out_h = fb_data->yres;
	tegra_fb->win.z = 0xFF;
	tegra_fb->win.phys_addr_u = 0;
	tegra_fb->win.phys_addr_v = 0;
	tegra_fb->win.stride = info->fix.line_length;
	tegra_fb->win.stride_uv = 0;
	tegra_fb->win.flags = TEGRA_WIN_FLAG_ENABLED;
	tegra_fb->win.global_alpha = 0xFF;

	tegra_fb->blank_win = tegra_fb->win;
	tegra_fb->blank_win.phys_addr = tegra_fb->blank_start;
	tegra_fb->blank_win.virt_addr = tegra_fb->blank_base;
	tegra_fb_fill_420_10bpc_blank_frame(&tegra_fb->blank_win);

	for (mode_idx = 0; mode_idx < dc->out->n_modes; mode_idx++) {
		struct tegra_dc_mode mode = dc->out->modes[mode_idx];
		struct fb_videomode vmode;


		if (mode.pclk > 1000) {
			tegra_dc_to_fb_videomode(&vmode, &mode);
			fb_add_videomode(&vmode, &info->modelist);
		}
	}

	if (dc->out_ops->vrr_update_monspecs)
		dc->out_ops->vrr_update_monspecs(dc, &info->modelist);

	tegra_fb_set_par(info);

	if (register_framebuffer(info)) {
		dev_err(&ndev->dev, "failed to register framebuffer\n");
		ret = -ENODEV;
		goto err_iounmap_fb;
	}

	if (tegra_dc_is_nvdisplay()) {
		if (ndev->id != info->node) {
			dev_err(&ndev->dev, "FB device numbering does not\n"
					"match device numbering of extended\n"
					"display interfaces\n");
			ret = -EINVAL;
			goto err_iounmap_fb;
		}
	}

	tegra_fb->info = info;

	if ((fb_data->flags & TEGRA_FB_FLIP_ON_PROBE) &&
			(fb_data->win > -1)) {
		struct tegra_dc_win *win = &tegra_fb->win;
		tegra_dc_update_windows(&win, 1, NULL, true, false);
		tegra_dc_sync_windows(&win, 1);
		tegra_dc_program_bandwidth(win->dc, true);
	}

	dev_info(&ndev->dev, "fb registered\n");

	return tegra_fb;

err_iounmap_fb:
	dma_free_attrs(&ndev->dev, BLANK_LINE_SIZE, tegra_fb->blank_base,
		       tegra_fb->blank_start, __DMA_ATTR(attrs));
err_free_fbmem:
	tegra_fb_release_fbmem(tegra_fb);
err_free:
	framebuffer_release(info);
err:
	return ERR_PTR(ret);
}

void tegra_fb_unregister(struct tegra_fb_info *fb_info)
{
	struct fb_info *info = fb_info->info;
	struct device *dev = &fb_info->ndev->dev;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_WRITE_COMBINE, __DMA_ATTR(attrs));

	dma_free_attrs(dev, BLANK_LINE_SIZE, fb_info->blank_base,
		       fb_info->blank_start, __DMA_ATTR(attrs));

	tegra_fb_release_fbmem(fb_info);
	unregister_framebuffer(info);
	framebuffer_release(info);
	dev_info(dev, "fb unregistered\n");
}

/*
 * framebuffer console status is depends on both
 * kernel config option and DT based flag
 */

inline bool tegra_fb_is_console_enabled(struct tegra_dc_platform_data *pdata)
{
	return (IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE) &&
		!(pdata->flags & TEGRA_DC_FLAG_FBCON_DISABLED));
}
