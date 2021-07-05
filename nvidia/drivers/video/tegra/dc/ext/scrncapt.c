/*
 * scrncapt.c: Screen capture functionality for tegradc ext interface.
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Sungwook Kim <sungwookk@nvidia.com>
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

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/export.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/memblock.h>
#include <linux/fb.h>
#include <uapi/video/tegra_dc_ext.h>
#include <trace/events/display.h>

#include "../dc.h"
#include "../dc_priv.h"
#include "../dc_config.h"
#include "tegra_dc_ext_priv.h"


/*
 * private info for
 * Tegra DC screen Capture
 */
static struct tegra_dc_scrncapt_private  {
	struct mutex         lock;
	int                  holder_pid;
	u32                  magic;
	u32                  pause_heads;
	struct rw_semaphore  *rwsema_head;
	struct timer_list    tmr_resume;
	unsigned long        tm_resume;
}  scrncapt;

/*
 * Interface with Tegra DC Driver
 *
 * Tegra DC Driver should call tegra_dc_scrncapt_disp_pause_lock()
 * and tegra_dc_scrncapt_disp_pause_unlock() to protect the region
 * that has an update to display which to be captured.
 */
void  tegra_dc_scrncapt_disp_pause_lock(struct tegra_dc *dc)
{
	down_read(&scrncapt.rwsema_head[dc->ctrl_num]);
}


void  tegra_dc_scrncapt_disp_pause_unlock(struct tegra_dc *dc)
{
	up_read(&scrncapt.rwsema_head[dc->ctrl_num]);
}


/*
 * I/O Controls
 */

static size_t  scrncapt_get_dcbuf_len(struct tegra_dc_dmabuf *dcbuf)
{
	if (dcbuf && dcbuf->buf)
		return dcbuf->buf->size;
	else
		return 0;
}


/* copy a Tegra DC DMA buffer contents
 * o inputs:
 *  - pDst: pointer to the destination
 *          must have enough allocation to hold the copy
 *  - pSrcBuf: pointer to the source Tegra DC DMA buffer
 *  - len: number of bytes to be copied
 * o outputs:
 *  - return: number of bytes copied
 */
static size_t scrncapt_copy_dcbuf(void  __user *pDst,
			struct tegra_dc_dmabuf *pSrcBuf, size_t len)
{
	size_t  copied;
	unsigned long ret;

	if (!len || !pDst || !pSrcBuf || !pSrcBuf->buf || !pSrcBuf->sgt
		|| !pSrcBuf->sgt->nents || !pSrcBuf->sgt->sgl)
		return 0;

	/* to emulate the segmented memory copy
	 *	copied = sg_copy_to_buffer(pSrcBuf->sgt->sgl,
	 *			pSrcBuf->sgt->nents, pDst, len);
	 * sg_copy_to_buffer() is not compatible with EBP DisplayServer
	 * virtualization due to no nvmap carveout highmem support.
	 */
	{
		void *vaddr_src;
		struct scatterlist *sg;
		unsigned int  i;
		size_t  ofs, l;
		unsigned long  flags;

		pr_debug("@@ %s: copy to %p from %p(PA=%llx) nents=%u len=%zu\n",
			__func__, pDst, sg_virt(&pSrcBuf->sgt->sgl[0]),
			sg_phys(&pSrcBuf->sgt->sgl[0]),
			pSrcBuf->sgt->nents, len);
		ofs = 0;
		local_irq_save(flags);
		for_each_sg(pSrcBuf->sgt->sgl, sg, pSrcBuf->sgt->nents, i) {
			if (len <= ofs)
				break;
			l = sg->length;
			l = (len < (ofs + l)) ? len - ofs : l;

			vaddr_src = ioremap_cache(sg_phys(sg), l);
			ret = copy_to_user((void __user *)((char *)pDst + ofs),
						vaddr_src, l);
			iounmap(vaddr_src);
			if (ret) {
				local_irq_restore(flags);
				return -EFAULT;
			}

			ofs += l;
			if (sg->offset) {
				pr_debug("@@!! %s.%d: sgl[].offset:%lu\n",
					__func__, __LINE__, sg->offset);
			}
		}
		local_irq_restore(flags);
		if (ofs != len) {
			pr_debug("@@!! %s.%d: copied only %lu out of %zu\n",
				__func__, __LINE__, ofs, len);
		}
		copied = ofs;
	}

	return copied;
}


static int  scrncapt_get_info_head(struct tegra_dc *dc,
		void __user *ptr)
{
	int  err = 0;
	struct tegra_dc_ext_scrncapt_get_info_head  info;

	if (copy_from_user(&info, ptr, sizeof(info)))
		return -EFAULT;

	info.sts_en = dc->enabled;
	info.hres = dc->mode.h_active;
	info.vres = dc->mode.v_active;
	info.flag_val_wins = dc->valid_windows;

	if (copy_to_user(ptr, &info, sizeof(info)))
		err = -EFAULT;

	return err;
}


static int  scrncapt_get_info_win(struct tegra_dc *dc, int winidx,
		struct tegra_dc_ext_flip_windowattr *winattr)
{
	int  err = 0;
	struct tegra_dc_ext     *ext = dc->ext;
	struct tegra_dc_win     *win;
	struct tegra_dc_ext_win *extwin;
	int  len, len_u, len_v;

	win = tegra_dc_get_window(dc, winidx);
	extwin = &ext->win[winidx];

	memset(winattr, 0x0, sizeof(*winattr));
	winattr->index     = winidx;
	len = scrncapt_get_dcbuf_len(extwin->cur_handle[TEGRA_DC_Y]);
	len_u = scrncapt_get_dcbuf_len(extwin->cur_handle[TEGRA_DC_U]);
	len_v = scrncapt_get_dcbuf_len(extwin->cur_handle[TEGRA_DC_V]);
	/* hack to return buffer size instead of buff_id that needs
	 * unnecessary conversion */
	winattr->buff_id   = len;
	winattr->buff_id_u = len_u;
	winattr->buff_id_v = len_v;
	winattr->offset    = 0x0;
	if (!len_u)
		winattr->offset_u  = win->phys_addr_u - win->phys_addr;
	if (!len_v)
		winattr->offset_v  = win->phys_addr_v - win->phys_addr_u;
	winattr->stride    = win->stride;
	winattr->stride_uv = win->stride_uv;
	winattr->block_height_log2 = win->block_height_log2;
	winattr->pixformat = win->fmt;
	if (win->flags & TEGRA_WIN_FLAG_BLEND_PREMULT)
		winattr->blend = TEGRA_DC_EXT_BLEND_PREMULT;
	else if (win->flags & TEGRA_WIN_FLAG_BLEND_COVERAGE)
		winattr->blend = TEGRA_DC_EXT_BLEND_COVERAGE;
	if (win->flags & TEGRA_WIN_FLAG_TILED)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_TILED;
	if (win->flags & TEGRA_WIN_FLAG_INVERT_H)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_INVERT_H;
	if (win->flags & TEGRA_WIN_FLAG_INVERT_V)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_INVERT_V;
	if (win->flags & TEGRA_WIN_FLAG_SCAN_COLUMN)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_SCAN_COLUMN;
	if (255 != win->global_alpha)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA;
	if (win->flags & TEGRA_WIN_FLAG_BLOCKLINEAR)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_BLOCKLINEAR;
	if (win->flags & TEGRA_WIN_FLAG_INTERLACE)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_INTERLACE;
	if (win->flags & TEGRA_WIN_FLAG_INPUT_RANGE_LIMITED)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_LIMITED;
	else if (win->flags & TEGRA_WIN_FLAG_INPUT_RANGE_BYPASS)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_BYPASS;
	if (win->flags & TEGRA_WIN_FLAG_CS_REC601)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_CS_REC601;
	else if (win->flags & TEGRA_WIN_FLAG_CS_REC709)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_CS_REC709;
	else if (win->flags & TEGRA_WIN_FLAG_CS_REC2020)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020;
	else if ((win->flags & TEGRA_WIN_FLAG_CS_MASK) ==
				TEGRA_WIN_FLAG_CS_NONE)
		winattr->flags |= TEGRA_DC_EXT_FLIP_FLAG_CS_NONE;
	winattr->x         = win->x.full;
	winattr->y         = win->y.full;
	winattr->w         = win->w.full;
	winattr->h         = win->h.full;
	winattr->out_x     = win->out_x;
	winattr->out_y     = win->out_y;
	winattr->out_w     = win->out_w;
	winattr->out_h     = win->out_h;
	winattr->z         = win->z;
	winattr->global_alpha = win->global_alpha;

	return err;
}


static int  scrncapt_get_info_wins(struct tegra_dc *dc, void __user *ptr)
{
	int                  err = 0;
	int                  i;
	struct tegra_dc_ext_scrncapt_get_info_win  info;
	struct tegra_dc_ext_flip_windowattr __user  *pwinattr;
	int  num_wins;

	if (copy_from_user(&info, ptr, sizeof(info)))
		return -EFAULT;
	if (!info.flag_wins || !info.wins)
		return -EINVAL;

	pwinattr = (struct tegra_dc_ext_flip_windowattr __user *)info.wins;
	num_wins = 0;
	for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
		struct tegra_dc_ext_flip_windowattr  winattr;

		if (!(info.flag_wins & dc->valid_windows & (1 << i)))
			continue;

		scrncapt_get_info_win(dc, i, &winattr);
		if (copy_to_user(pwinattr + num_wins, &winattr,
				sizeof(winattr))) {
			err = -EFAULT;
			break;
		}
		num_wins++;
	}

	if (!err) {
		info.num_wins = num_wins;
		if (copy_to_user(ptr, &info, sizeof(info)))
			err = -EFAULT;
	}

	return err;
}


static int  scrncapt_get_info_cursor(struct tegra_dc *dc, void __user *ptr)
{
	int  err = 0;
	struct tegra_dc_ext  *ext = dc->ext;
	struct tegra_dc_ext_cursor_image  info = { {0} };

	if (!ptr)
		err = -EFAULT;

	if (!err) {
		u32  flags = 0x0;

		switch (dc->cursor.size) {
		case TEGRA_DC_CURSOR_SIZE_32X32:
			flags |= TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_32x32;
			break;
		case TEGRA_DC_CURSOR_SIZE_64X64:
			flags |= TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_64x64;
			break;
		case TEGRA_DC_CURSOR_SIZE_128X128:
			flags |= TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_128x128;
			break;
		case TEGRA_DC_CURSOR_SIZE_256X256:
			flags |= TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_256x256;
			break;
		default:
			pr_warn("scrncapt: unknown dc->cursor.size:%d\n",
				dc->cursor.size);
			err = -EFAULT;
			break;
		}
		switch (dc->cursor.colorfmt) {
		case legacy:
			flags |= TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_LEGACY;
			break;
		case r8g8b8a8: /* normal */
			flags |= TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_R8G8B8A8;
			break;
		case a1r5g5b5:
			flags |= TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_A1R5G5B5;
			break;
		case a8r8g8b8:
			flags |= TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_A8R8G8B8;
			break;
		default:
			pr_warn("scrncapt: unknown dc->cursor.colorfmt:%d\n",
				dc->cursor.colorfmt);
			err = -EFAULT;
			break;
		}
		switch (dc->cursor.blendfmt) {
		case TEGRA_DC_CURSOR_FORMAT_2BIT_LEGACY:
			flags |= TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_2BIT_LEGACY;
			break;
		case TEGRA_DC_CURSOR_FORMAT_RGBA_NON_PREMULT_ALPHA:
			flags |=
		TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_NON_PREMULT_ALPHA;
			break;
		case TEGRA_DC_CURSOR_FORMAT_RGBA_PREMULT_ALPHA:
			flags |=
			TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_PREMULT_ALPHA;
			break;
		case TEGRA_DC_CURSOR_FORMAT_RGBA_XOR:
			flags |= TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_XOR;
			break;
		default:
			pr_warn("scrncapt: unknown dc->cursor.blendfmt:%d\n",
				dc->cursor.blendfmt);
			err = -EFAULT;
			break;
		}
		/* The cursor visivility flag TEGRA_DC_EXT_CURSOR_FLAGS_VISIBLE
		 * conflicts with TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_32x32.
		 * Temporarily assign to bit31 for cursor visivility status. */
		flags |= dc->cursor.enabled ? 1 << 31 : 0 << 31;

		info.flags   = flags;
		/* hack to return buffer size instead of buff_id that needs
		 * unnecessary conversion */
		info.buff_id  = scrncapt_get_dcbuf_len(ext->cursor.cur_handle);
		info.x        = dc->cursor.x;
		info.y        = dc->cursor.y;
		info.alpha    = dc->cursor.alpha;
		info.colorfmt = dc->cursor.colorfmt;
		/* reverse of CURSOR_COLOR(r,g,b) macro */
		info.foreground.r = (dc->cursor.fg >> 0) & 0xff;
		info.foreground.g = (dc->cursor.fg >> 8) & 0xff;
		info.foreground.b = (dc->cursor.fg >> 16) & 0xff;
		info.background.r = (dc->cursor.bg >> 0) & 0xff;
		info.background.g = (dc->cursor.bg >> 8) & 0xff;
		info.background.b = (dc->cursor.bg >> 16) & 0xff;
	}

	if (copy_to_user(ptr, &info, sizeof(info)))
		err = -EFAULT;

	return err;
}


static int scrncapt_get_info_cursor_data(struct tegra_dc *dc,
		void __user *ptr)
{
	int err = 0;
	struct tegra_dc_ext *ext = dc->ext;
	struct tegra_dc_ext_scrncapt_get_info_cursor_data info;
	struct tegra_dc_dmabuf *dcbuf;
	void __user *pbuf;
	size_t len, l = 0;

	if (copy_from_user(&info, ptr, sizeof(info))) {
		err = -EFAULT;
	} else {
		dcbuf = ext->cursor.cur_handle;
		if (dcbuf) {
			len = scrncapt_get_dcbuf_len(dcbuf);
			if (info.size < len)
				err = -ENOSPC;
			else
				/* verify user space is accessible */
				if (!access_ok(VERIFY_WRITE, info.ptr, len))
					err = -EFAULT;
		}
		if (!err && dcbuf) {
			pbuf = (void __user *)info.ptr;
			l = scrncapt_copy_dcbuf(pbuf, dcbuf, len);
			if (l != len)
				err = -EIO;
		}
		info.len = l;
		if (copy_to_user(ptr, &info, sizeof(info)))
			err = -EFAULT;
	}

	return err;
}


int  tegra_dc_scrncapt_get_info(struct tegra_dc_ext_user *user,
		struct tegra_dc_ext_scrncapt_get_info *args)
{
	int  err = 0;
	int  i;
	struct tegra_dc_ext  *ext = user->ext;
	struct tegra_dc      *dc  = ext->dc;
	struct tegra_dc_ext_scrncapt_get_info_data __user  *pdt;
	struct tegra_dc_ext_scrncapt_get_info_data         data;

	/* no support of 1st implementation */
	if (TEGRA_DC_EXT_SCRNCAPT_VER_V(args->ver) != 2)
		return -EFAULT;

	if ((args->ver ^ scrncapt.magic) & ~0xff)
		return -EFAULT;
	if (tegra_dc_get_numof_dispheads() <= (unsigned)args->head)
		return -EFAULT;

	/* check disp paused */
	mutex_lock(&scrncapt.lock);
	if (!(scrncapt.pause_heads & (1 << dc->ctrl_num)))
		err = -EINVAL;
	mutex_unlock(&scrncapt.lock);
	if (err)
		return err;

	pdt = (struct tegra_dc_ext_scrncapt_get_info_data __user *)args->data;
	for (i = 0; i < args->num_data; i++) {
		if (copy_from_user(&data, pdt + i, sizeof(data))) {
			err = -EFAULT;
			break;
		}
		switch (data.type) {
		case TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_HEAD:
			scrncapt_get_info_head(dc, (void *)data.ptr);
			break;
		case TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_WINS:
			scrncapt_get_info_wins(dc, (void *)data.ptr);
			break;
		case TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CURSOR:
			scrncapt_get_info_cursor(dc, (void *)data.ptr);
			break;
		case TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CURSOR_DATA:
			scrncapt_get_info_cursor_data(dc, (void *)data.ptr);
			break;
		/* TODO: add CMU & CSC capture.
		 *       these information are not used at this time. */
		case TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_NVDISP_CMU:
		case TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_NVDISP_WIN_CSC:
		default:
			pr_info("scrncapt: data type %d not implemented yet\n",
				data.type);
			err = -EINVAL;
			break;
		}
		if (err)
			break;
	}
	args->ver = TEGRA_DC_EXT_SCRNCAPT_VER_2(args->ver);

	return err;
}


int tegra_dc_scrncapt_dup_fbuf(struct tegra_dc_ext_user *user,
		struct tegra_dc_ext_scrncapt_dup_fbuf *args)
{
	int err = 0;
	int p;
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc *dc  = ext->dc;
	struct tegra_dc_win *win;
	struct tegra_dc_ext_win *extwin;
	u8 *dest;
	int ofs;

	/* no support of 1st implementation */
	if (TEGRA_DC_EXT_SCRNCAPT_VER_V(args->ver) != 2)
		return -EFAULT;

	if ((args->ver ^ scrncapt.magic) & ~0xff)
		return -EFAULT;
	if ((tegra_dc_get_numof_dispheads() <= (unsigned)args->head) ||
		(tegra_dc_get_numof_dispwindows() <= (unsigned)args->win))
		return -EINVAL;

	if (!access_ok(VERIFY_WRITE, args->buffer, args->buffer_max))
		return -EFAULT;

	/* check disp paused & valid window */
	mutex_lock(&scrncapt.lock);
	if (!(scrncapt.pause_heads & (1 << dc->ctrl_num)))
		err = -EINVAL;
	mutex_unlock(&scrncapt.lock);
	if (!(dc->valid_windows & (1 << args->win)))
		err = -EBUSY;
	if (err)
		return err;

	win = tegra_dc_get_window(dc, args->win);
	extwin = &ext->win[args->win];
	dest = (u8 *)args->buffer;
	ofs = 0;
	for (p = 0; p < TEGRA_DC_NUM_PLANES; p++) {
		struct tegra_dc_dmabuf  *buf;
		size_t                  len,  l;

		buf = extwin->cur_handle[p];
		if (!buf) {
			args->plane_sizes[p] = 0;
		} else {
			len = scrncapt_get_dcbuf_len(buf);
			if (args->buffer_max < ofs + len) {
				err = -ENOSPC;
				break;
			}
			l = scrncapt_copy_dcbuf((void __user *)(dest + ofs),
						buf, len);
			if (l != len) {
				err = -EIO;
				break;
			}
			args->plane_sizes[p]   = len;
			args->plane_offsets[p] = ofs;
			ofs += len;
			ofs = (ofs + (8 - 1)) & ~(8 - 1);
		}
	}
	args->ver = TEGRA_DC_EXT_SCRNCAPT_VER_2(args->ver);

	return err;
}


int  tegra_dc_scrncapt_pause(struct tegra_dc_ext_control_user *ctlusr,
		struct tegra_dc_ext_control_scrncapt_pause *args)
{
	int  err = 0;
	int  i, nheads;
	u32  heads;
	unsigned long  tm;

	if (TEGRA_DC_EXT_CONTROL_SCRNCAPT_MAGIC != args->magic)
		return -EFAULT;

	nheads = tegra_dc_get_numof_dispheads();
	heads = 1 << 31;  /* default to pausing all heads */
	if ((1 << 31) & heads)
		heads |= -1;
	tm = (-1 == args->tm_resume_msec) ? 0 :
		(args->tm_resume_msec ?
			args->tm_resume_msec * HZ / 1000 + 1 : HZ / 2);

	mutex_lock(&scrncapt.lock);
	if (scrncapt.pause_heads) {
		err = -EBUSY;
	} else {
		for (i = 0; i < nheads; i++) {
			if ((1 << i) & heads)
				down_write(&scrncapt.rwsema_head[i]);
		}
		scrncapt.pause_heads = heads;
		scrncapt.holder_pid = current->pid;
		scrncapt.magic = args->magic ^ (jiffies << 8);
		/* set-up a timer to limit the disp pausing time */
		scrncapt.tm_resume = tm;
		if (tm) {
			scrncapt.tmr_resume.expires = jiffies + tm;
			add_timer(&scrncapt.tmr_resume);
		}
	}
	mutex_unlock(&scrncapt.lock);
	if (!err)
		pr_info("scrncapt: disp paused, timer-set:%lu\n",
			tm * 1000 / HZ);

	args->magic = scrncapt.magic;
	args->tm_resume_msec = tm ? : -1;
	args->num_heads = nheads;
	args->num_wins = tegra_dc_get_numof_dispwindows();

	return err;
}


int  tegra_dc_scrncapt_resume(struct tegra_dc_ext_control_user *ctlusr,
		struct tegra_dc_ext_control_scrncapt_resume *args)
{
	int  err = 0;
	int  i;
	u32  heads;

	if (args->magic != scrncapt.magic)
		return -EFAULT;

	mutex_lock(&scrncapt.lock);

	if (!scrncapt.pause_heads) {
		err = -EINVAL;
	} else {
		if (scrncapt.tm_resume)
			del_timer(&scrncapt.tmr_resume);
		heads = scrncapt.pause_heads;
		scrncapt.pause_heads = 0x0;
		scrncapt.holder_pid = 0x0;
		if ((1 << 31) & heads)
			heads |= -1;
		for (i = 0; i < tegra_dc_get_numof_dispheads(); i++) {
			if ((1 << i) & heads)
				up_write(&scrncapt.rwsema_head[i]);
		}
	}
	mutex_unlock(&scrncapt.lock);
	if (!err)
		pr_info("scrncapt: disp resumed\n");

	return err;
}


/* timer call-back
 * to resume display automatically after timeout
 */
static void  tegra_dc_scrncapt_timer_cb(unsigned long arg)
{
	int  i;
	u32  heads;

	heads = scrncapt.pause_heads;
	scrncapt.pause_heads = 0x0;
	scrncapt.holder_pid = 0x0;
	if ((1 << 31) & heads)
		heads |= -1;
	for (i = 0; i < tegra_dc_get_numof_dispheads(); i++) {
		if ((1 << i) & heads)
			up_write(&scrncapt.rwsema_head[i]);
	}
	pr_info("scrncapt: pause timeout, auto resuming.\n");
}


int  tegra_dc_scrncapt_init(void)
{
	int  i, nheads, nwins;

	nheads = tegra_dc_get_numof_dispheads();
	nwins = tegra_dc_get_numof_dispwindows();

	if (nheads <= 0 || nwins <= 0) {
		pr_err("%s: heads:%d windows:%d cannot be negative or zero\n",
			__func__, nheads, nwins);
		return -EINVAL;
	}

	memset(&scrncapt, 0, sizeof(scrncapt));
	mutex_init(&scrncapt.lock);
	scrncapt.rwsema_head = kzalloc(nheads *
		sizeof(struct rw_semaphore), GFP_KERNEL);

	if (IS_ERR_OR_NULL(scrncapt.rwsema_head)) {
		pr_err("%s: Insufficient memory\n", __func__);
		return -ENOMEM;
	}

	pr_info("scrncapt: init (heads:%d wins:%d planes:%d)\n",
		nheads, nwins, TEGRA_DC_NUM_PLANES);

	for (i = 0; i < nheads; i++)
		init_rwsem(&scrncapt.rwsema_head[i]);
	init_timer(&scrncapt.tmr_resume);
	scrncapt.tmr_resume.function = &tegra_dc_scrncapt_timer_cb;
	scrncapt.tmr_resume.data = (unsigned long)&scrncapt;
	scrncapt.magic = TEGRA_DC_EXT_CONTROL_SCRNCAPT_MAGIC ^ (jiffies << 8);

	return 0;
}


int  tegra_dc_scrncapt_exit(void)
{
	pr_info("scrncapt: exit\n");
	kfree(scrncapt.rwsema_head);
	return 0;
}
