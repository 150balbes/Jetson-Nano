/*
 * crc.c: CRC functions for tegradc EXT device
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>

#include "dc.h"
#include "dc_priv_defs.h"
#include "dc_priv.h"
#include "dc_reg.h"
#include <uapi/video/tegra_dc_ext.h>

#define TEGRA_DC_FLIP_BUF_CAPACITY 1024 /* in units of number of elements */
#define TEGRA_DC_CRC_BUF_CAPACITY 1024 /* in units of number of elements */
#define CRC_COMPLETE_TIMEOUT msecs_to_jiffies(1000)

static inline size_t _get_bytes_per_ele(struct tegra_dc_ring_buf *buf)
{
	switch (buf->type) {
	case TEGRA_DC_RING_BUF_FLIP:
		return sizeof(struct tegra_dc_flip_buf_ele);
	case TEGRA_DC_RING_BUF_CRC:
		return sizeof(struct tegra_dc_crc_buf_ele);
	default:
		return 0;
	}
}

/* Retrieve an in buffer pointer for element at index @idx
 * @in_buf_ptr is the output parameter to be filled by the API
 */
static int tegra_dc_ring_buf_peek(struct tegra_dc_ring_buf *buf, u16 idx,
				  char **in_buf_ptr)
{
	size_t bytes = _get_bytes_per_ele(buf);

	if (idx >= buf->capacity)
		return -EINVAL;

	if (buf->size == 0)
		return -EAGAIN;

	*in_buf_ptr = (buf->data + idx * bytes);

	return 0;
}

/* Remove the least recently buffered element */
static int tegra_dc_ring_buf_remove(struct tegra_dc_ring_buf *buf)
{
	if (buf->size == 0)
		return -EINVAL;

	buf->tail = (buf->tail + 1) % buf->capacity;
	buf->size--;

	return 0;
}

/* Add a buffer element at the head of the buffer
 * @src is the memory pointer from where data for the buffer element is copied
 * if @in_buf_ptr is not NULL, the caller receives in buffer pointer to the
 * element
 */
void tegra_dc_ring_buf_add(struct tegra_dc_ring_buf *buf, void *src,
			  char **in_buf_ptr)
{
	size_t bytes = _get_bytes_per_ele(buf);
	void *dst = buf->data + buf->head * bytes;

	/* If the buffer is full, drop the least recently used item */
	if (buf->size == buf->capacity)
		tegra_dc_ring_buf_remove(buf);

	memcpy(dst, src, bytes);
	if (in_buf_ptr)
		*in_buf_ptr = dst;

	buf->head = (buf->head + 1) % buf->capacity;
	buf->size++;
}

/* Called when enabling the DC head.
 * Avoid calling it when disabling the DC head so as to avoid any issues caused
 * by an impending or a missing flush of values to the registers
 */
void tegra_dc_crc_reset(struct tegra_dc *dc)
{
	if (tegra_dc_is_t21x())
		tegra_dc_writel(dc, 0x00, DC_COM_CRC_CONTROL);
	else if (tegra_dc_is_nvdisplay())
		tegra_nvdisp_crc_reset(dc);
}

/* This API cleans up reference counts that track CRC APIs invoked/enabled from
 * userspace via relevant IOCTLs. If the legacy APIs have been invoked, simply
 * clear its reference count and return, else call individual CRC disable
 * routines for each block we support reading back CRCs for.
 */
void tegra_dc_crc_drop_ref_cnts(struct tegra_dc *dc)
{
	int ret;
	struct tegra_dc_ext_crc_arg arg;
	struct tegra_dc_ext_crc_conf *conf = NULL;
	unsigned int num_conf, i;
	u8 id; /* region_id */

	mutex_lock(&dc->lock);

	if (dc->crc_ref_cnt.legacy) {
		dc->crc_ref_cnt.legacy = 0;
		mutex_unlock(&dc->lock);
		return;
	}

	arg.num_conf = atomic_read(&dc->crc_ref_cnt.global);
	if (!arg.num_conf) {
		mutex_unlock(&dc->lock);
		return;
	}

	memcpy(arg.magic, "TCRC", 4);
	arg.version = TEGRA_DC_CRC_ARG_VERSION_0;

	conf = kcalloc(arg.num_conf, sizeof(*conf), GFP_KERNEL);
	if (!conf) {
		mutex_unlock(&dc->lock);
		return;
	}

	arg.conf = (__u64)conf;

	num_conf = 0;

	for (i = 0; i < atomic_read(&dc->crc_ref_cnt.rg); i++)
		conf[num_conf++].type = TEGRA_DC_EXT_CRC_TYPE_RG;

	for (i = 0; i < atomic_read(&dc->crc_ref_cnt.comp); i++)
		conf[num_conf++].type = TEGRA_DC_EXT_CRC_TYPE_COMP;

	for (i = 0; i < atomic_read(&dc->crc_ref_cnt.out); i++)
		conf[num_conf++].type = TEGRA_DC_EXT_CRC_TYPE_OR;

	for (id = 0; id < TEGRA_DC_MAX_CRC_REGIONS; id++) {
		for (i = 0; i < atomic_read(&dc->crc_ref_cnt.region[id]); i++) {
			conf[num_conf].type = TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL;
			conf[num_conf].region.id = id;
			num_conf++;
		}
	}

	WARN_ON(num_conf != arg.num_conf);
	mutex_unlock(&dc->lock);

	ret = tegra_dc_crc_disable(dc, &arg);
	if (ret)
		dev_warn(&dc->ndev->dev, "%s: Disable CRCs failed. ret = %d\n",
			 __func__, ret);

	kfree(conf);
}

/* Called when disabling the DC head
 * This function resets only the SW state associated with the CRC mechanism
 */
void tegra_dc_crc_deinit(struct tegra_dc *dc)
{
	u8 i = 0;

	if (!dc->crc_initialized)
		return;

	kfree(dc->flip_buf.data);
	kfree(dc->crc_buf.data);

	dc->flip_buf.size = 0;
	dc->flip_buf.head = 0;
	dc->flip_buf.tail = 0;
	dc->crc_buf.size = 0;
	dc->crc_buf.head = 0;
	dc->crc_buf.tail = 0;

	atomic_set(&dc->crc_ref_cnt.global, 0);
	atomic_set(&dc->crc_ref_cnt.rg, 0);
	atomic_set(&dc->crc_ref_cnt.comp, 0);
	atomic_set(&dc->crc_ref_cnt.out, 0);
	atomic_set(&dc->crc_ref_cnt.regional, 0);

	for (i = 0; i < TEGRA_DC_MAX_CRC_REGIONS; i++)
		atomic_set(&dc->crc_ref_cnt.region[i], 0);

	dc->crc_ref_cnt.legacy = false;

	mutex_destroy(&dc->flip_buf.lock);
	mutex_destroy(&dc->crc_buf.lock);

	dc->crc_initialized = false;
}

static long tegra_dc_crc_init(struct tegra_dc *dc)
{
	dc->flip_buf.capacity = TEGRA_DC_FLIP_BUF_CAPACITY;
	dc->crc_buf.capacity = TEGRA_DC_CRC_BUF_CAPACITY;

	dc->flip_buf.type = TEGRA_DC_RING_BUF_FLIP;
	dc->crc_buf.type = TEGRA_DC_RING_BUF_CRC;

	dc->flip_buf.data = kcalloc(TEGRA_DC_FLIP_BUF_CAPACITY,
				    sizeof(struct tegra_dc_flip_buf_ele),
				    GFP_KERNEL);
	if (!dc->flip_buf.data)
		return -ENOMEM;

	dc->crc_buf.data = kcalloc(TEGRA_DC_CRC_BUF_CAPACITY,
				   sizeof(struct tegra_dc_crc_buf_ele),
				   GFP_KERNEL);
	if (!dc->crc_buf.data)
		return -ENOMEM;

	mutex_init(&dc->flip_buf.lock);
	mutex_init(&dc->crc_buf.lock);

	dc->crc_initialized = true;

	return 0;
}

static int tegra_dc_crc_t21x_rg_en_dis(struct tegra_dc *dc,
				       struct tegra_dc_ext_crc_conf *conf,
				       bool enable)
{
	int ret = 0x00;
	u32 reg = 0x00;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	if (enable) {
		reg = CRC_ALWAYS_ENABLE | CRC_ENABLE_ENABLE;
		if (conf->input_data == TEGRA_DC_EXT_CRC_INPUT_DATA_ACTIVE_DATA)
			reg |= CRC_INPUT_DATA_ACTIVE_DATA;

		atomic_inc(&dc->crc_ref_cnt.rg);

		tegra_dc_writel(dc, reg, DC_COM_CRC_CONTROL);
		tegra_dc_activate_general_channel(dc);
	} else {
		if (atomic_dec_return(&dc->crc_ref_cnt.rg) == 0) {
			tegra_dc_writel(dc, 0x00, DC_COM_CRC_CONTROL);
			tegra_dc_activate_general_channel(dc);
		}
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	if (enable) {
		if (atomic_inc_return(&dc->crc_ref_cnt.global) == 1) {
			ret = tegra_dc_config_frame_end_intr(dc, true);
			if (ret) {
				atomic_dec(&dc->crc_ref_cnt.rg);
				atomic_dec(&dc->crc_ref_cnt.global);
			}
		}
	} else {
		if (atomic_dec_return(&dc->crc_ref_cnt.global) == 0) {
			ret = tegra_dc_config_frame_end_intr(dc, false);
			if (ret) {
				atomic_inc(&dc->crc_ref_cnt.rg);
				atomic_inc(&dc->crc_ref_cnt.global);
			}
		}
	}

	return ret;
}

static int tegra_dc_crc_t21x_out_en_dis(struct tegra_dc *dc,
					struct tegra_dc_ext_crc_conf *conf,
					bool enable)
{
	int ret = 0;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	if (enable) {
		if (!dc->out_ops->crc_en) {
			ret = -ENOTSUPP;
			goto fail;
		}

		ret = dc->out_ops->crc_en(dc, &conf->or_params);
		if (ret)
			goto fail;

		atomic_inc(&dc->crc_ref_cnt.out);
	} else {
		if (!dc->out_ops->crc_dis) {
			ret = -ENOTSUPP;
			goto fail;
		}

		if (atomic_dec_return(&dc->crc_ref_cnt.out) == 0) {
			ret = dc->out_ops->crc_dis(dc, &conf->or_params);
			if (ret) {
				atomic_inc(&dc->crc_ref_cnt.out);
				goto fail;
			}
		}
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	if (enable) {
		if (atomic_inc_return(&dc->crc_ref_cnt.global) == 1) {
			ret = tegra_dc_config_frame_end_intr(dc, true);
			if (ret) {
				atomic_dec(&dc->crc_ref_cnt.out);
				atomic_dec(&dc->crc_ref_cnt.global);
			}
		}
	} else {
		if (atomic_dec_return(&dc->crc_ref_cnt.global) == 0) {
			ret = tegra_dc_config_frame_end_intr(dc, false);
			if (ret) {
				atomic_inc(&dc->crc_ref_cnt.out);
				atomic_inc(&dc->crc_ref_cnt.global);
			}
		}
	}

	return ret;

fail:
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return ret;
}

static int tegra_dc_crc_t21x_en_dis(struct tegra_dc *dc,
				    struct tegra_dc_ext_crc_arg *arg, bool en)
{
	int ret = 0;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	u8 iter;

	for (iter = 0; iter < arg->num_conf; iter++) {
		switch (conf[iter].type) {
		case TEGRA_DC_EXT_CRC_TYPE_RG:
			ret = tegra_dc_crc_t21x_rg_en_dis(dc, &conf[iter], en);
			if (ret)
				return ret;
			break;
		case TEGRA_DC_EXT_CRC_TYPE_OR:
			ret = tegra_dc_crc_t21x_out_en_dis(dc, &conf[iter], en);
			if (ret)
				return ret;
			break;
		default:
			return -ENOTSUPP;
		}
	}

	return ret;
}

static int tegra_dc_crc_nvdisp_en_dis(struct tegra_dc *dc,
				      struct tegra_dc_ext_crc_arg *arg, bool en)
{
	int ret;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	u8 iter;

	for (iter = 0; iter < arg->num_conf; iter++) {
		if (en)
			ret = tegra_nvdisp_crc_enable(dc, conf + iter);
		else
			ret = tegra_nvdisp_crc_disable(dc, conf + iter);
		if (ret)
			return ret;
	}

	tegra_dc_activate_general_channel(dc);

	return 0;
}

long tegra_dc_crc_enable(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	int ret;

	if (!dc->enabled)
		return -ENODEV;

	if (dc->crc_ref_cnt.legacy)
		return -EBUSY;

	if (!dc->crc_initialized) {
		ret = tegra_dc_crc_init(dc);
		if (ret)
			return ret;
	}

	if (tegra_dc_is_t21x())
		return tegra_dc_crc_t21x_en_dis(dc, arg, true);
	else if (tegra_dc_is_nvdisplay())
		return tegra_dc_crc_nvdisp_en_dis(dc, arg, true);
	else
		return -ENOTSUPP;
}

long tegra_dc_crc_disable(struct tegra_dc *dc,
			  struct tegra_dc_ext_crc_arg *arg)
{
	if (!dc->enabled)
		return -ENODEV;

	if (!dc->crc_initialized)
		return -EPERM;

	WARN_ON(dc->crc_ref_cnt.legacy);

	if (tegra_dc_is_t21x())
		return tegra_dc_crc_t21x_en_dis(dc, arg, false);
	else if (tegra_dc_is_nvdisplay())
		return tegra_dc_crc_nvdisp_en_dis(dc, arg, false);
	else
		return -ENOTSUPP;
}

/* Get the Least Recently Matched (lrm) flip ID.
 * Our best estimate is to find this flip ID at the tail of the buffer.
 */
static int _find_lrm(struct tegra_dc *dc, u64 *flip_id)
{
	u64 lrm = 0x0; /* Default value when no flips are matched */
	int ret = 0;
	struct tegra_dc_crc_buf_ele *crc_ele = NULL;

	ret = tegra_dc_ring_buf_peek(&dc->crc_buf, dc->crc_buf.tail,
				     (char **)&crc_ele);
	if (ret)
		goto done;

	if (crc_ele->matching_flips[0].valid)
		lrm = crc_ele->matching_flips[0].id;

done:
	*flip_id = lrm;
	return ret;
}

static inline u16 prev_idx(struct tegra_dc_ring_buf *buf, u16 idx)
{
	return idx ? idx - 1 : buf->capacity - 1;
}

static bool _is_flip_out_of_bounds(struct tegra_dc *dc, u64 flip_id)
{
	u64 lrm; /* Least recently matched flip */
	u64 mrq; /* Most recently queued flip */

	mutex_lock(&dc->crc_buf.lock);
	_find_lrm(dc, &lrm);
	mutex_unlock(&dc->crc_buf.lock);

	mrq = atomic64_read(&dc->flip_stats.flips_queued);

	if (mrq < lrm) {
		dev_err(&dc->ndev->dev, "flip IDs have overflowed "
					"lrm = %llu, mrq = %llu\n", lrm, mrq);
		return true;
	}

	if (flip_id < lrm || flip_id > mrq) {
		dev_err(&dc->ndev->dev, "flip_id %llu out of bounds "
					"lrm = %llu, mrq = %llu\n",
					flip_id, lrm, mrq);
		return true;
	}

	return false;
}

/* Scan the CRC buffer from @start_idx (inclusive) to @end_idx (exclusive)
 * to find the element matched with @flip_id
 */
static int _scan_crc_buf(struct tegra_dc *dc, u16 start_idx, u16 end_idx,
			 u64 flip_id, struct tegra_dc_crc_buf_ele *crc_ele)
{
	u16 peek_idx = start_idx;
	struct tegra_dc_ring_buf *buf = &dc->crc_buf;
	struct tegra_dc_crc_buf_ele *crc_iter = NULL;
	int iter, ret;

	while (peek_idx != end_idx) {
		ret = tegra_dc_ring_buf_peek(buf, peek_idx, (char **)&crc_iter);
		if (ret)
			return ret;

		for (iter = 0; iter < DC_N_WINDOWS; iter++) {
			if (!crc_iter->matching_flips[iter].valid)
				break;

			if (crc_iter->matching_flips[iter].id == flip_id) {
				memcpy(crc_ele, crc_iter, sizeof(*crc_iter));
				return 0;
			}
		}

		peek_idx = prev_idx(buf, peek_idx);
	}

	return -EAGAIN;
}

/* Wraps calls to wait APIs depending upon the platform */
static int tegra_dc_crc_wait_till_frame_end(struct tegra_dc *dc)
{
	int ret = 0;

	if (tegra_platform_is_silicon()) {
		if (!wait_for_completion_timeout(&dc->crc_complete,
						 CRC_COMPLETE_TIMEOUT)) {
			dev_err(&dc->ndev->dev, "CRC read timed out\n");
			ret = -ETIME;
		}
	} else {
		ret = wait_for_completion_interruptible(&dc->crc_complete);
		if (ret)
			dev_err(&dc->ndev->dev, "CRC read wait interrupted\n");
	}

	return ret;
}

static int _find_crc_in_buf(struct tegra_dc *dc, u64 flip_id,
			    struct tegra_dc_crc_buf_ele *crc_ele)
{
	int ret = 0, ret_loop = -EAGAIN;
	u16 start_idx, end_idx;
	struct tegra_dc_ring_buf *buf = &dc->crc_buf;

	if (flip_id == U64_MAX) {
		/* Overwrite flip_id with most recently queued flip. If no
		 * flips have been queued yet, return an error to the user
		 */
		flip_id = atomic64_read(&dc->flip_stats.flips_queued);
		if (!flip_id)
			return -ENODATA;
	}

	if (_is_flip_out_of_bounds(dc, flip_id))
		return -ENODATA;

	mutex_lock(&buf->lock);

	/* At this point, we are committed to return a CRC value to the user,
	 * even if one is yet to be generated in the imminent future
	 */
	end_idx = prev_idx(buf, buf->tail);
	start_idx = prev_idx(buf, buf->head);

	while (ret_loop == -EAGAIN) {
		ret_loop = _scan_crc_buf(dc, start_idx, end_idx, flip_id,
					 crc_ele);
		if (!ret_loop || ret_loop != -EAGAIN) {
			ret = ret_loop;
			goto done;
		}

		/* Control reaching here implies the flip being requested is yet
		 * to be matched at a certain frame end interrupt, hence wait on
		 * the event
		 */
		mutex_unlock(&buf->lock);
		reinit_completion(&dc->crc_complete);

		ret = tegra_dc_crc_wait_till_frame_end(dc); /* Blocking call */
		if (ret)
			return ret;

		mutex_lock(&buf->lock);

		end_idx = prev_idx(buf, start_idx);
		start_idx = prev_idx(buf, buf->head);
	}

done:
	mutex_unlock(&buf->lock);
	return ret;
}

static int tegra_dc_crc_t21x_collect(struct tegra_dc *dc,
				     struct tegra_dc_crc_buf_ele *ele)
{
	int ret = 0;
	bool valids = false; /* Logical OR of valid fields of individual CRCs */

	if (atomic_read(&dc->crc_ref_cnt.rg)) {
		ele->rg.crc = tegra_dc_readl(dc, DC_COM_CRC_CHECKSUM_LATCHED);
		ele->rg.valid = true;
	}

	if (atomic_read(&dc->crc_ref_cnt.out)) {
		if (dc->out_ops->crc_get) {
			ret = dc->out_ops->crc_get(dc, &ele->sor.crc);
			/* NOTE: The call to crc_get only collects SOR CRC. In
			 * case other CRCs (say RG CRC) were collected, we need
			 * the caller to queue the generated CRC element.
			 * Hence, we do not propagate this error back to the
			 * caller, but instead simply refrain from setting the
			 * valid bit
			 */
			if (!ret)
				ele->sor.valid = true;
		}
	}

	valids = ele->rg.valid || ele->sor.valid;

	return valids ? 0 : -EINVAL;
}

long tegra_dc_crc_get(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	int ret = 0;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	struct tegra_dc_crc_buf_ele crc_ele;
	u8 id, iter;

	if (!dc->enabled)
		return -ENODEV;

	if (!dc->crc_initialized)
		return -EPERM;

	WARN_ON(dc->crc_ref_cnt.legacy);

	ret = _find_crc_in_buf(dc, arg->flip_id, &crc_ele);
	if (ret)
		return ret;

	for (iter = 0; iter < arg->num_conf; iter++) {
		switch (conf[iter].type) {
		case TEGRA_DC_EXT_CRC_TYPE_RG:
			conf[iter].crc.valid = crc_ele.rg.valid;
			conf[iter].crc.val = crc_ele.rg.crc;
			break;
		case TEGRA_DC_EXT_CRC_TYPE_COMP:
			if (tegra_dc_is_nvdisplay()) {
				conf[iter].crc.valid = crc_ele.comp.valid;
				conf[iter].crc.val = crc_ele.comp.crc;
			} else {
				conf[iter].crc.valid = false;
				conf[iter].crc.val = 0;
				ret = -ENOTSUPP;
			}
			break;
		case TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL:
			if (tegra_dc_is_nvdisplay()) {
				id = conf[iter].region.id;
				conf[iter].crc.valid =
						crc_ele.regional[id].valid;
				conf[iter].crc.val = crc_ele.regional[id].crc;
			} else {
				conf[iter].crc.valid = false;
				conf[iter].crc.val = 0;
				ret = -ENOTSUPP;
			}
			break;
		case TEGRA_DC_EXT_CRC_TYPE_OR:
			conf[iter].crc.valid = crc_ele.sor.valid;
			conf[iter].crc.val = crc_ele.sor.crc;
			break;
		default:
			ret = -ENOTSUPP;
		}
	}

	return ret;
}

int tegra_dc_crc_process(struct tegra_dc *dc)
{
	int ret = 0, matched = 0;
	struct tegra_dc_crc_buf_ele crc_ele;
	struct tegra_dc_flip_buf_ele *flip_ele;

	memset(&crc_ele, 0, sizeof(crc_ele));

	/* Collect all CRCs generated by the HW */
	if (tegra_dc_is_t21x())
		ret = tegra_dc_crc_t21x_collect(dc, &crc_ele);
	else if (tegra_dc_is_nvdisplay())
		ret = tegra_nvdisp_crc_collect(dc, &crc_ele);

	if (ret)
		return ret;

	mutex_lock(&dc->flip_buf.lock);

	/* Before doing any work, check if there are flips to match */
	if (!dc->flip_buf.size) {
		mutex_unlock(&dc->flip_buf.lock);
		return 0;
	}

	/* Mark all least recently buffered flips in FLIPPED state as
	 * matching_flips
	 */
	ret = tegra_dc_ring_buf_peek(&dc->flip_buf, dc->flip_buf.tail,
				     (char **)&flip_ele);
	while (!ret && (flip_ele->state == TEGRA_DC_FLIP_STATE_FLIPPED ||
			flip_ele->state == TEGRA_DC_FLIP_STATE_SKIPPED)) {
		if (flip_ele->state == TEGRA_DC_FLIP_STATE_FLIPPED) {
			crc_ele.matching_flips[matched].id = flip_ele->id;
			crc_ele.matching_flips[matched].valid = true;
			matched++;
			WARN_ON(matched >= DC_N_WINDOWS);
		}

		tegra_dc_ring_buf_remove(&dc->flip_buf);
		ret = tegra_dc_ring_buf_peek(&dc->flip_buf, dc->flip_buf.tail,
					     (char **)&flip_ele);
	}

	/* Enqueue CRC element in the CRC ring buffer */
	if (matched) {
		mutex_lock(&dc->crc_buf.lock);
		tegra_dc_ring_buf_add(&dc->crc_buf, &crc_ele, NULL);
		mutex_unlock(&dc->crc_buf.lock);
	}

	mutex_unlock(&dc->flip_buf.lock);
	return ret;
}

