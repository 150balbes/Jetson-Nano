/*
 * camera_version_utils.c - utilities for different kernel versions
 * camera driver supports
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <media/camera_common.h>

int tegra_media_entity_init(struct media_entity *entity, u16 num_pads,
			struct media_pad *pad, bool is_subdev, bool is_sensor)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	if (!is_subdev) {
		entity->obj_type = MEDIA_ENTITY_TYPE_VIDEO_DEVICE;
		entity->function = MEDIA_ENT_F_IO_V4L;
	} else {
		entity->obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
		entity->function = is_sensor ? MEDIA_ENT_F_CAM_SENSOR :
					MEDIA_ENT_F_CAM_HW;
	}
	return media_entity_pads_init(entity, num_pads, pad);
#else
	if (is_subdev) {
		entity->type = is_sensor ? MEDIA_ENT_T_V4L2_SUBDEV_SENSOR :
				MEDIA_ENT_T_V4L2_SUBDEV;
	}
	return media_entity_init(entity, num_pads, pad, 0);
#endif
}
EXPORT_SYMBOL(tegra_media_entity_init);

bool tegra_is_v4l2_subdev(struct media_entity *entity)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	return is_media_entity_v4l2_subdev(entity);
#else
	return (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV);
#endif
}
EXPORT_SYMBOL(tegra_is_v4l2_subdev);

int tegra_media_create_link(struct media_entity *source, u16 source_pad,
		struct media_entity *sink, u16 sink_pad, u32 flags)
{
	int ret = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	ret = media_create_pad_link(source, source_pad,
			       sink, sink_pad, flags);
#else
	ret = media_entity_create_link(source, source_pad,
			       sink, sink_pad, flags);
#endif

	return ret;
}

bool tegra_v4l2_match_dv_timings(struct v4l2_dv_timings *t1,
				struct v4l2_dv_timings *t2,
				unsigned pclock_delta,
				bool match_reduced_fps)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	return v4l2_match_dv_timings(t1, t2, pclock_delta, match_reduced_fps);
#else
	return v4l2_match_dv_timings(t1, t2, pclock_delta);
#endif
}
EXPORT_SYMBOL(tegra_v4l2_match_dv_timings);

int tegra_vb2_dma_init(struct device *dev, void **alloc_ctx,
	unsigned int size, atomic_t *refcount)
{
	int ret = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	if (atomic_inc_return(refcount) > 1)
		return 0;

	if (vb2_dma_contig_set_max_seg_size(dev, SZ_64K)) {
		dev_err(dev, "failed to init vb2 buffer\n");
		ret = -ENOMEM;
	}
#else
	*alloc_ctx = vb2_dma_contig_init_ctx(dev);
	if (IS_ERR(*alloc_ctx)) {
		dev_err(dev, "failed to init vb2 buffer\n");
		ret = -ENOMEM;
	}
#endif
	return ret;
}
EXPORT_SYMBOL(tegra_vb2_dma_init);

void tegra_vb2_dma_cleanup(struct device *dev, void *alloc_ctx,
			atomic_t *refcount)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	atomic_dec_return(refcount);
	/* dont call vb2_dma_contig_clear_max_seg_size as it will */
	/* call kfree dma_parms but dma_parms is static member */
#else
	vb2_dma_contig_cleanup_ctx(alloc_ctx);
#endif
}
EXPORT_SYMBOL(tegra_vb2_dma_cleanup);
