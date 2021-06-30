/*
 * Tegra flcn common Module Support
 *
 * Copyright (c) 2011-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_FLCN_H__
#define __NVHOST_FLCN_H__

#include <linux/types.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

struct ucode_bin_header_v1_flcn {
	u32 bin_magic;        /* 0x10de */
	u32 bin_ver;          /* cya, versioning of bin format (1) */
	u32 bin_size;         /* entire image size including this header */
	u32 os_bin_header_offset;
	u32 os_bin_data_offset;
	u32 os_bin_size;
	u32 fce_bin_header_offset;
	u32 fce_bin_data_offset;
	u32 fce_bin_size;
	u32 bin_ver_tag;
};

struct ucode_os_header_v1_flcn {
	u32 os_code_offset;
	u32 os_code_size;
	u32 os_data_offset;
	u32 os_data_size;
	u32 num_apps;
};

struct ucode_fce_header_v1_flcn {
	u32 fce_ucode_offset;
	u32 fce_ucode_buffer_size;
	u32 fce_ucode_size;
};

struct ucode_v1_flcn {
	struct ucode_bin_header_v1_flcn *bin_header;
	struct ucode_os_header_v1_flcn  *os_header;
	struct ucode_fce_header_v1_flcn *fce_header;
	bool valid;
};

struct flcn_os_image {
	u32 bin_magic;
	u32 reserved_offset;
	u32 bin_data_offset;
	u32 data_offset;
	u32 data_size;
	u32 code_size;
	u32 code_offset;
	u32 size;
	u32 bin_ver_tag;
};

struct flcn {
	bool valid;
	size_t size;
	bool is_booted;

	struct flcn_os_image os;
	struct flcn_os_image fce;

	dma_addr_t dma_addr;
	u32 *mapped;

	dma_addr_t fce_dma_addr;
	u32 *fce_mapped;
};

static inline struct flcn *get_flcn(struct platform_device *dev)
{
	return (struct flcn *)nvhost_get_falcon_data(dev);
}
static inline void set_flcn(struct platform_device *dev, struct flcn *flcn)
{
	nvhost_set_falcon_data(dev, flcn);
}
int flcn_setup_ucode_image(struct platform_device *dev,
			   struct flcn *v,
			   const struct firmware *ucode_fw,
			   struct ucode_v1_flcn *ucode);
int nvhost_vic_prepare_poweroff(struct platform_device *);
int nvhost_flcn_finalize_poweron(struct platform_device *);
int nvhost_vic_finalize_poweron(struct platform_device *);
int nvhost_vic_init_context(struct platform_device *pdev,
			    struct nvhost_cdma *cdma);
void flcn_enable_timestamps(struct platform_device *pdev,
				struct nvhost_cdma *cdma,
				dma_addr_t timestamp_addr);
int nvhost_flcn_prepare_poweroff(struct platform_device *);
int nvhost_flcn_common_isr(struct platform_device *);

int nvhost_vic_aggregate_constraints(struct platform_device *dev,
				     int clk_index,
				     unsigned long floor_rate,
				     unsigned long pixelrate,
				     unsigned long bw_constraint);

int nvhost_flcn_wait_mem_scrubbing(struct platform_device *dev);
int flcn_intr_init(struct platform_device *pdev);
int flcn_reload_fw(struct platform_device *pdev);
int nvhost_flcn_load_image(struct platform_device *pdev,
			   dma_addr_t dma_addr,
			   struct flcn_os_image *os,
			   u32 mem_offset);
int nvhost_flcn_start(struct platform_device *pdev, u32 bootvec);
void nvhost_flcn_ctxtsw_init(struct platform_device *pdev);
void nvhost_flcn_irq_dest_set(struct platform_device *pdev);
void nvhost_flcn_irq_mask_set(struct platform_device *pdev);

/* hack, get these from elsewhere */
#define NVA0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID		(0x00000200)
#define NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE		(0x0000071C)
#define NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET		(0x0000072C)
#define FLCN_UCLASS_METHOD_ADDR_TSP	0xC8
#define VIC_UCLASS_METHOD_OFFSET	0x10
#define VIC_UCLASS_METHOD_DATA		0x11
#define FLCN_UCLASS_METHOD_OFFSET	0x10
#define FLCN_UCLASS_METHOD_DATA		0x11


#define NVHOST_ENCODE_FLCN_VER(maj, min) \
	((((maj) & 0xff) << 8) | ((min) & 0xff))

#endif /* __NVHOST_FLCN_H__ */
