/*
 * Tegra Graphics Init for T194 Architecture Chips
 *
 * Copyright (c) 2016-2020, NVIDIA Corporation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <soc/tegra/chip-id.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/dma-override.h>

#include <soc/tegra/kfuse.h>

#include <linux/tegra_vhost.h>

#include "dev.h"
#include "class_ids.h"
#include "class_ids_t194.h"

#include "nvhost_syncpt_unit_interface.h"
#include "t194.h"
#include "host1x/host1x.h"
#if defined(CONFIG_TEGRA_GRHOST_ISP)
#include "isp/isp5.h"
#endif
#if defined(CONFIG_TEGRA_GRHOST_TSEC)
#include "tsec/tsec.h"
#endif
#include "flcn/flcn.h"
#if defined(CONFIG_TEGRA_GRHOST_NVCSI)
#include "nvcsi/nvcsi-t194.h"
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVDEC)
#include "nvdec/nvdec.h"
#endif
#if defined(CONFIG_TEGRA_GRHOST_PVA)
#include "pva/pva.h"
#endif
#include "hardware_t194.h"
#if defined(CONFIG_TEGRA_GRHOST_NVDLA)
#include "nvdla/nvdla.h"
#endif

#if defined(CONFIG_TEGRA_GRHOST_SLVSEC)
#include "slvsec/slvsec.h"
#endif
#if defined(CONFIG_VIDEO_TEGRA_VI)
#include "vi/vi5.h"
#endif

#include "chip_support.h"

#include "scale_emc.h"

#include "streamid_regs.c"
#include "cg_regs.c"
#include "actmon_regs.c"

/*
 * TODO: Move following functions to the corresponding files under
 * kernel-3.18 once kernel-t19x gets merged there. Until that
 * happens we can keep these here to avoid extensive amount of
 * added infra
 */

static inline u32 flcn_thi_sec(void)
{
	return 0x00000038;
}

static inline u32 flcn_thi_sec_ch_lock(void)
{
	return (1 << 8);
}

static enum dma_data_direction nvhost_t194_get_dma_direction(u32 reloc_type)
{
	enum dma_data_direction direction = DMA_BIDIRECTIONAL;

	if (reloc_type == NVHOST_RELOC_TYPE_NVLINK)
		direction |= DMA_FOR_NVLINK;

	return direction;
}

static dma_addr_t nvhost_t194_get_reloc_phys_addr(dma_addr_t phys_addr,
						  u32 reloc_type)
{
	if (reloc_type == NVHOST_RELOC_TYPE_BLOCK_LINEAR)
		phys_addr += BIT(39);

	return phys_addr;
}

#if defined(CONFIG_TEGRA_GRHOST_TSEC)
static int nvhost_tsec_t194_finalize_poweron(struct platform_device *dev)
{
	/* Disable access to non-THI registers through channel */
	host1x_writel(dev, flcn_thi_sec(), flcn_thi_sec_ch_lock());

	return nvhost_tsec_finalize_poweron(dev);
}
#endif

static int nvhost_flcn_t194_finalize_poweron(struct platform_device *dev)
{
	/* Disable access to non-THI registers through channel */
	host1x_writel(dev, flcn_thi_sec(), flcn_thi_sec_ch_lock());

	return nvhost_flcn_finalize_poweron(dev);
}

#if defined(CONFIG_TEGRA_GRHOST_NVDEC)
static int nvhost_nvdec_t194_finalize_poweron(struct platform_device *dev)
{
	int ret;

	if (!tegra_platform_is_vdk()) {
		ret = tegra_kfuse_enable_sensing();
		if (ret)
			return ret;
	}

	/* Disable access to non-THI registers through channel */
	host1x_writel(dev, flcn_thi_sec(), flcn_thi_sec_ch_lock());

	ret = nvhost_nvdec_finalize_poweron(dev);
	if (ret)
		tegra_kfuse_disable_sensing();

	return ret;
}

static int nvhost_nvdec_t194_prepare_poweroff(struct platform_device *dev)
{
	if (!tegra_platform_is_vdk())
		tegra_kfuse_disable_sensing();

	return 0;
}
#endif

static struct host1x_device_info host1x04_info = {
	.nb_channels	= T194_NVHOST_NUMCHANNELS,
	.ch_base	= 0,
	.ch_limit	= T194_NVHOST_NUMCHANNELS,
	.nb_mlocks	= NV_HOST1X_NB_MLOCKS,
	.initialize_chip_support = nvhost_init_t194_support,
	.nb_hw_pts	= NV_HOST1X_SYNCPT_NB_PTS,
	.nb_pts		= NV_HOST1X_SYNCPT_NB_PTS,
	.pts_base	= 0,
	.pts_limit	= NV_HOST1X_SYNCPT_NB_PTS,
	.syncpt_policy	= SYNCPT_PER_CHANNEL_INSTANCE,
	.channel_policy	= MAP_CHANNEL_ON_SUBMIT,
	.firmware_area_size = SZ_1M,
	.nb_actmons	= 1,
	.use_cross_vm_interrupts = 1,
	.resources	= {
		"guest",
		"hypervisor",
		"actmon",
		"sem-syncpt-shim"
	},
	.nb_resources	= 4,
	.secure_cmdfifo = true,
	.dma_mask	= DMA_BIT_MASK(40),
};

struct nvhost_device_data t19_host1x_info = {
	.clocks			= {
		{"host1x", 204000000},
		{"actmon", UINT_MAX}
	},
	.autosuspend_delay      = 50,
	.private_data		= &host1x04_info,
	.finalize_poweron	= nvhost_host1x_finalize_poweron,
	.prepare_poweroff	= nvhost_host1x_prepare_poweroff,
	.engine_can_cg		= true,
};

struct nvhost_device_data t19_host1x_hv_info = {
	.clocks			= {
		{"host1x", 204000000},
		{"actmon", UINT_MAX}
	},
	.autosuspend_delay      = 2000,
	.private_data		= &host1x04_info,
	.finalize_poweron = nvhost_host1x_finalize_poweron,
	.prepare_poweroff = nvhost_host1x_prepare_poweroff,
};

static struct host1x_device_info host1xb04_info = {
	.nb_channels	= T194_NVHOST_NUMCHANNELS,
	.ch_base	= 0,
	.ch_limit	= T194_NVHOST_NUMCHANNELS,
	.nb_mlocks	= NV_HOST1X_NB_MLOCKS,
	.initialize_chip_support = nvhost_init_t194_support,
	.nb_hw_pts	= NV_HOST1X_SYNCPT_NB_PTS,
	.nb_pts		= NV_HOST1X_SYNCPT_NB_PTS,
	.pts_base	= 0,
	.pts_limit	= NV_HOST1X_SYNCPT_NB_PTS,
	.syncpt_policy	= SYNCPT_PER_CHANNEL_INSTANCE,
	.channel_policy	= MAP_CHANNEL_ON_SUBMIT,
	.use_cross_vm_interrupts = 1,
};

struct nvhost_device_data t19_host1xb_info = {
	.clocks			= {
		{"host1x", UINT_MAX},
		{"actmon", UINT_MAX}
	},
	.private_data		= &host1xb04_info,
};

#if defined(CONFIG_VIDEO_TEGRA_VI)
struct nvhost_device_data t19_vi_thi_info = {
	.devfs_name		= "vi-thi",
	.exclusive		= true,
	.class			= NV_VIDEO_STREAMING_VI_FALCON_CLASS_ID,
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_VI},
	.keepalive		= true,
	.autosuspend_delay      = 500,
	.moduleid		= NVHOST_MODULE_VI,
	.clocks = {
		{"vi", UINT_MAX},
		{"vi-const", UINT_MAX},
	},
	.num_channels		= 1,
	.can_powergate		= true,
};

struct nvhost_device_data t19_vi5_info = {
	.devfs_name		= "vi",
	.class			= NV_VIDEO_STREAMING_VI_CLASS_ID,
	.keepalive		= true,
	.autosuspend_delay	= 500,
	.poweron_reset		= true,
	.moduleid		= NVHOST_MODULE_VI,
	.clocks = {
		{"vi", UINT_MAX},
		{"vi-const", UINT_MAX},
		{"nvcsi", 400000000},
		{"nvcsilp", 204000000},
	},
	.version		= NVHOST_ENCODE_FLCN_VER(5, 0),
	.num_ppc		= 8,
	.aggregate_constraints	= nvhost_vi5_aggregate_constraints,
	.can_powergate		= true,
	.pre_virt_init		= vi5_priv_early_probe,
	.post_virt_init		= vi5_priv_late_probe,
	.num_channels           = 36,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_NVCSI)
struct nvhost_device_data t19_nvcsi_info = {
	.num_channels		= 1,
	.moduleid		= NVHOST_MODULE_NVCSI,
	.clocks			= {
		{"nvcsi", 400000000},
		{"nvcsilp", 204000000},
	},
	.devfs_name		= "nvcsi",
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVCSI},
	.class			= NV_VIDEO_STREAMING_NVCSI_CLASS_ID,
	.ctrl_ops		= &tegra194_nvcsi_ctrl_ops,
	.autosuspend_delay      = 500,
	.finalize_poweron	= tegra194_nvcsi_finalize_poweron,
	.prepare_poweroff	= tegra194_nvcsi_prepare_poweroff,
	.poweron_reset		= true,
	.keepalive		= true,
	.serialize		= 1,
	.push_work_done		= 1,
	.can_powergate		= true,
	.pre_virt_init		= t194_nvcsi_early_probe,
	.post_virt_init		= t194_nvcsi_late_probe,
};
#endif

#ifdef CONFIG_TEGRA_GRHOST_ISP
struct nvhost_device_data t19_isp_thi_info = {
	.devfs_name		= "isp-thi",
	.moduleid		= NVHOST_MODULE_ISP,
	.can_powergate		= true,
};

struct nvhost_device_data t19_isp5_info = {
	.devfs_name		= "isp",
	.class			= NV_VIDEO_STREAMING_ISP_CLASS_ID,
	.keepalive		= true,
	.autosuspend_delay      = 500,
	.poweron_reset		= true,
	.moduleid		= NVHOST_MODULE_ISP,
	.clocks			= {
		{"isp", UINT_MAX},
	},
	.ctrl_ops		= &tegra194_isp5_ctrl_ops,
	.version		= NVHOST_ENCODE_FLCN_VER(5, 0),
	.can_powergate		= true,
	.pre_virt_init		= isp5_priv_early_probe,
	.post_virt_init		= isp5_priv_late_probe,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_NVENC)
struct nvhost_device_data t19_msenc_info = {
	.version		= NVHOST_ENCODE_FLCN_VER(7, 0),
	.devfs_name		= "msenc",
	.class			= NV_VIDEO_ENCODE_NVENC_CLASS_ID,
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVENC},
	.autosuspend_delay      = 500,
	.clocks			= {
		{"nvenc", UINT_MAX},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_SHARED_BW}
	},
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_flcn_t194_finalize_poweron,
	.moduleid		= NVHOST_MODULE_MSENC,
	.num_channels		= 1,
	.firmware_name		= "nvhost_nvenc070.fw",
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x1844,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_MSENC,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.engine_cg_regs		= t19x_nvenc_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.isolate_contexts	= true,
	.enable_timestamps	= flcn_enable_timestamps,
	.mlock_timeout_factor   = 4,
};

struct nvhost_device_data t19_nvenc1_info = {
	.version		= NVHOST_ENCODE_FLCN_VER(7, 0),
	.devfs_name		= "nvenc1",
	.class			= NV_VIDEO_ENCODE_NVENC1_CLASS_ID,
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVENC1},
	.autosuspend_delay      = 500,
	.clocks			= {
		{"nvenc", UINT_MAX},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_SHARED_BW}
	},
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_flcn_t194_finalize_poweron,
	.moduleid		= NVHOST_MODULE_NVENC1,
	.num_channels		= 1,
	.firmware_name		= "nvhost_nvenc070.fw",
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x1844,
	.transcfg_val		= 0x20,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_NVENC1,
	.engine_cg_regs		= t19x_nvenc_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.isolate_contexts	= true,
	.enable_timestamps	= flcn_enable_timestamps,
	.mlock_timeout_factor   = 4,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_NVDEC)
struct nvhost_device_data t19_nvdec_info = {
	.version		= NVHOST_ENCODE_NVDEC_VER(4, 0),
	.devfs_name		= "nvdec",
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVDEC},
	.class			= NV_NVDEC_CLASS_ID,
	.autosuspend_delay      = 500,
	.clocks			= {
		{"nvdec", UINT_MAX},
		{"kfuse", 0, 0},
		{"efuse", 0, 0},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_nvdec_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_nvdec_t194_prepare_poweroff,
	.moduleid		= NVHOST_MODULE_NVDEC,
	.ctrl_ops		= &tegra_nvdec_ctrl_ops,
	.num_channels		= 1,
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x2c44,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_NVDEC,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.engine_cg_regs		= t19x_nvdec_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.isolate_contexts	= true,
	.mlock_timeout_factor   = 4,
};

struct nvhost_device_data t19_nvdec1_info = {
	.version		= NVHOST_ENCODE_NVDEC_VER(4, 0),
	.devfs_name		= "nvdec1",
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVDEC1},
	.class			= NV_NVDEC1_CLASS_ID,
	.autosuspend_delay      = 500,
	.clocks			= {
		{"nvdec", UINT_MAX},
		{"kfuse", 0, 0},
		{"efuse", 0, 0},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_nvdec_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_nvdec_t194_prepare_poweroff,
	.moduleid		= NVHOST_MODULE_NVDEC1,
	.ctrl_ops		= &tegra_nvdec_ctrl_ops,
	.num_channels		= 1,
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x2c44,
	.transcfg_val		= 0x20,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_NVDEC1,
	.engine_cg_regs		= t19x_nvdec_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.isolate_contexts	= true,
	.mlock_timeout_factor   = 4,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_NVJPG)
struct nvhost_device_data t19_nvjpg_info = {
	.version		= NVHOST_ENCODE_FLCN_VER(1, 2),
	.devfs_name		= "nvjpg",
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_NVJPG},
	.class			= NV_NVJPG_CLASS_ID,
	.autosuspend_delay      = 500,
	.clocks			= {
		{"nvjpg", UINT_MAX},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_SHARED_BW}
	},
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_flcn_t194_finalize_poweron,
	.moduleid		= NVHOST_MODULE_NVJPG,
	.num_channels		= 1,
	.firmware_name		= "nvhost_nvjpg012.fw",
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x1444,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_NVJPG,
	.engine_cg_regs		= t19x_nvjpg_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.isolate_contexts	= true,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_TSEC)
struct nvhost_device_data t19_tsec_info = {
	.num_channels		= 1,
	.devfs_name		= "tsec",
	.version		= NVHOST_ENCODE_TSEC_VER(1, 0),
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_TSEC},
	.class			= NV_TSEC_CLASS_ID,
	.clocks			= {
		{"tsec", UINT_MAX},
		{"efuse", 0, 0},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	.autosuspend_delay      = 500,
	.keepalive		= true,
	.moduleid		= NVHOST_MODULE_TSEC,
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_tsec_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_tsec_prepare_poweroff,
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_TSEC,
	.engine_cg_regs		= t19x_tsec_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
};

struct nvhost_device_data t19_tsecb_info = {
	.num_channels		= 1,
	.devfs_name		= "tsecb",
	.version		= NVHOST_ENCODE_TSEC_VER(1, 0),
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_TSECB},
	.class			= NV_TSECB_CLASS_ID,
	.clocks			= {
		{"tsecb", UINT_MAX},
		{"efuse", 0, 0},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	.autosuspend_delay      = 500,
	.keepalive		= true,
	.moduleid               = NVHOST_MODULE_TSECB,
	.poweron_reset		= true,
	.finalize_poweron	= nvhost_tsec_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_tsec_prepare_poweroff,
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_TSECB,
	.engine_cg_regs		= t19x_tsec_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_VIC)
struct nvhost_device_data t19_vic_info = {
	.num_channels		= 1,
	.devfs_name		= "vic",
	.clocks			= {
		{"vic", UINT_MAX, 0},
		{"emc", 0, NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_SHARED_BW},
	},
	.version		= NVHOST_ENCODE_FLCN_VER(4, 2),
	.autosuspend_delay      = 500,
	.moduleid		= NVHOST_MODULE_VIC,
	.poweron_reset		= true,
	.modulemutexes		= {NV_HOST1X_MLOCK_ID_VIC},
	.class			= NV_GRAPHICS_VIC_CLASS_ID,
	.finalize_poweron	= nvhost_flcn_t194_finalize_poweron,
	.prepare_poweroff	= nvhost_flcn_prepare_poweroff,
	.flcn_isr		= nvhost_flcn_common_isr,
	.init_class_context	= nvhost_vic_init_context,
	.firmware_name		= "nvhost_vic042.fw",
	.serialize		= true,
	.push_work_done		= true,
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.transcfg_addr		= 0x2044,
	.transcfg_val		= 0x20,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_VIC,
	.scaling_init		= nvhost_scale_emc_init,
	.scaling_deinit		= nvhost_scale_emc_deinit,
	.scaling_post_cb	= &nvhost_scale_emc_callback,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.module_irq		= 1,
	.engine_cg_regs		= t19x_vic_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.isolate_contexts	= true,
	.actmon_regs		= HOST1X_THOST_ACTMON_VIC,
	.actmon_enabled         = true,
	.actmon_irq		= 3,
	.actmon_weight_count	= 216,
	.actmon_setting_regs	= t19x_vic_actmon_registers,
	.devfreq_governor	= "userspace",
	.freqs			= {100000000, 200000000, 300000000,
					400000000, 500000000, 600000000},
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_PVA)
struct nvhost_device_data t19_pva1_info = {
	.num_channels		= 1,
	.clocks			= {
		{"axi", UINT_MAX,},
		{"vps0", UINT_MAX,},
		{"vps1", UINT_MAX,},
	},
	.ctrl_ops		= &tegra_pva_ctrl_ops,
	.devfs_name_family	= "pva",
	.class			= NV_PVA1_CLASS_ID,
	.autosuspend_delay      = 500,
	.finalize_poweron	= pva_finalize_poweron,
	.prepare_poweroff	= pva_prepare_poweroff,
	.firmware_name		= "nvhost_pva010.fw",
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {
		{0x70000, true, 0},
		{0x80000, false, 0},
		{0x80000, false, 8} },
	.poweron_reset		= true,
	.serialize		= true,
	.push_work_done		= true,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.can_powergate		= true,
};

struct nvhost_device_data t19_pva0_info = {
	.num_channels		= 1,
	.clocks			= {
		{"axi", UINT_MAX,},
		{"vps0", UINT_MAX,},
		{"vps1", UINT_MAX,},
	},
	.ctrl_ops		= &tegra_pva_ctrl_ops,
	.devfs_name_family	= "pva",
	.class			= NV_PVA0_CLASS_ID,
	.autosuspend_delay      = 500,
	.finalize_poweron	= pva_finalize_poweron,
	.prepare_poweroff	= pva_prepare_poweroff,
	.firmware_name		= "nvhost_pva010.fw",
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.vm_regs		= {
		{0x70000, true, 0},
		{0x80000, false, 0},
		{0x80000, false, 8} },
	.poweron_reset		= true,
	.serialize		= true,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.can_powergate		= true,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_NVDLA)
struct nvhost_device_data t19_nvdla0_info = {
	.devfs_name_family	= "nvdla",
	.class			= NV_DLA0_CLASS_ID,
	.clocks			= {
		{"nvdla0", UINT_MAX},
		{"nvdla0_flcn", UINT_MAX},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.finalize_poweron	= nvhost_nvdla_finalize_poweron,
	.prepare_poweroff	= nvhost_nvdla_prepare_poweroff,
	.flcn_isr               = nvhost_nvdla_flcn_isr,
	.self_config_flcn_isr	= true,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.firmware_name		= "nvhost_nvdla010.fw",
	.autosuspend_delay      = 500,
	.keepalive		= true,
	.poweron_reset		= true,
	.serialize		= true,
	.ctrl_ops		= &tegra_nvdla_ctrl_ops,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.module_irq		= 1,
	.engine_cg_regs		= t19x_nvdla_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_DLA0,
	.transcfg_addr		= 0x0444,
	.transcfg_val		= 0x20,
};

struct nvhost_device_data t19_nvdla1_info = {
	.devfs_name_family	= "nvdla",
	.class			= NV_DLA1_CLASS_ID,
	.clocks			= {
		{"nvdla1", UINT_MAX},
		{"nvdla1_flcn", UINT_MAX},
		{"emc", 0,
		 NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		 0, TEGRA_BWMGR_SET_EMC_FLOOR}
	},
	.resource_policy	= RESOURCE_PER_CHANNEL_INSTANCE,
	.finalize_poweron	= nvhost_nvdla_finalize_poweron,
	.prepare_poweroff	= nvhost_nvdla_prepare_poweroff,
	.flcn_isr               = nvhost_nvdla_flcn_isr,
	.self_config_flcn_isr	= true,
	.vm_regs		= {{0x30, true}, {0x34, false} },
	.firmware_name		= "nvhost_nvdla010.fw",
	.autosuspend_delay      = 500,
	.keepalive		= true,
	.poweron_reset		= true,
	.serialize		= true,
	.ctrl_ops		= &tegra_nvdla_ctrl_ops,
	.get_reloc_phys_addr	= nvhost_t194_get_reloc_phys_addr,
	.get_dma_direction	= nvhost_t194_get_dma_direction,
	.module_irq		= 1,
	.engine_cg_regs		= t19x_nvdla_gating_registers,
	.engine_can_cg		= true,
	.can_powergate		= true,
	.bwmgr_client_id	= TEGRA_BWMGR_CLIENT_DLA1,
	.transcfg_addr		= 0x0444,
	.transcfg_val		= 0x20,
};
#endif

#if defined(CONFIG_TEGRA_GRHOST_SLVSEC)
struct nvhost_device_data t19_slvsec_info = {
	.num_channels		= 1,
	.clocks			= {
		{"slvs-ec", UINT_MAX},
		{"slvs-ec-lp", UINT_MAX},
	},
	.devfs_name		= "slvs-ec",
	.class			= NV_SLVSEC_CLASS_ID,
	.autosuspend_delay      = 500,
	.finalize_poweron	= slvsec_finalize_poweron,
	.prepare_poweroff	= slvsec_prepare_poweroff,
	.poweron_reset		= true,
	.keepalive		= true,
	.serialize		= 1,
	.push_work_done		= 1,
	.can_powergate		= true,
};
#endif

#include "host1x/host1x_channel_t186.c"

static void t194_set_nvhost_chanops(struct nvhost_channel *ch)
{
	if (!ch)
		return;

	ch->ops = host1x_channel_ops;

	/* Disable gather filter in simulator */
	if (tegra_platform_is_vdk())
		ch->ops.init_gather_filter = NULL;
}

int nvhost_init_t194_channel_support(struct nvhost_master *host,
				     struct nvhost_chip_support *op)
{
	op->nvhost_dev.set_nvhost_chanops = t194_set_nvhost_chanops;

	return 0;
}

static void t194_remove_support(struct nvhost_chip_support *op)
{
	kfree(op->priv);
	op->priv = NULL;
}

#define SYNCPT_RAM_INIT_TIMEOUT_MS	1000

static void t194_init_regs(struct platform_device *pdev, bool prod)
{
	struct nvhost_gating_register *cg_regs = t19x_host1x_gating_registers;
	struct nvhost_streamid_mapping *map_regs = t19x_host1x_streamid_mapping;
	ktime_t now, start = ktime_get();
	u32 ram_init;
	int ret = 0;
	u64 cl = 0;

	/* Ensure that HW has finished initializing syncpt RAM prior to use */
	for (;;) {
		/* XXX: This retry loop takes too long to timeout on VDK */
		if (tegra_platform_is_sim()) {
			pr_info("%s: Skipping ram_init done check on sim.\n",
				__func__);
			break;
		}

		ram_init = host1x_hypervisor_readl(pdev,
					host1x_sync_syncpt_ram_init_0_r());
		if (!host1x_sync_syncpt_ram_init_0_ram_init_v(ram_init)) {
			pr_info("%s: Host1x HW syncpt ram init disabled\n",
				__func__);
			break;
		}
		if (host1x_sync_syncpt_ram_init_0_ram_init_done_v(ram_init))
			break;

		now = ktime_get();
		if (ktime_ms_delta(now, start) >= SYNCPT_RAM_INIT_TIMEOUT_MS) {
			pr_err("%s: Timed out waiting for syncpt ram init!\n",
				__func__);
			break;
		}
	}

	/* Use old mapping registers on older simulator CLs */
	ret = of_property_read_u64(pdev->dev.of_node,
				   "nvidia,changelist",
				   &cl);
	if (ret == 0 && cl <= 38424879)
		map_regs = t19x_host1x_streamid_mapping_vdk_r6;

	/* Write the map registers */
	while (map_regs->host1x_offset) {
		host1x_hypervisor_writel(pdev,
					 map_regs->host1x_offset,
					 map_regs->client_offset);
		host1x_hypervisor_writel(pdev,
					 map_regs->host1x_offset + sizeof(u32),
					 map_regs->client_limit);
		map_regs++;
	}

	while (cg_regs->addr) {
		u32 val = prod ? cg_regs->prod : cg_regs->disable;

		host1x_hypervisor_writel(pdev, cg_regs->addr, val);
		cg_regs++;
	}
}

#include "host1x/host1x_cdma_t186.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_syncpt_prot_t186.c"
#include "host1x/host1x_intr_t186.c"
#include "host1x/host1x_debug_t186.c"
#include "host1x/host1x_vm_t186.c"
#if defined(CONFIG_TEGRA_GRHOST_SCALE)
#include "host1x/host1x_actmon_t186.c"
#endif

int nvhost_init_t194_support(struct nvhost_master *host,
			     struct nvhost_chip_support *op)
{
	int err;

	op->soc_name = "tegra19x";

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t194_channel_support(host, op);
	if (err)
		return err;

	op->cdma = host1x_cdma_ops;
	op->push_buffer = host1x_pushbuffer_ops;
	op->debug = host1x_debug_ops;

	host->sync_aperture = host->aperture;
	op->syncpt = host1x_syncpt_ops;
	op->intr = host1x_intr_ops;
	op->vm = host1x_vm_ops;
	op->vm.init_syncpt_interface = nvhost_syncpt_unit_interface_init;
#if defined(CONFIG_TEGRA_GRHOST_SCALE)
	op->actmon = host1x_actmon_ops;
#endif
	op->nvhost_dev.load_gating_regs = t194_init_regs;

	op->syncpt.alloc = nvhost_syncpt_alloc_gos_backing;
	op->syncpt.release = nvhost_syncpt_release_gos_backing;

	/* WAR to bugs 200094901 and 200082771: enable protection
	 * only on silicon/emulation */

	if (!tegra_platform_is_vdk()) {
		op->syncpt.reset = t186_syncpt_reset;
		op->syncpt.mark_used = t186_syncpt_mark_used;
		op->syncpt.mark_unused = t186_syncpt_mark_unused;
	}
	op->syncpt.mutex_owner = t186_syncpt_mutex_owner;

	op->remove_support = t194_remove_support;

	return 0;
}
