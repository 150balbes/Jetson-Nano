/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_NVLINK_H
#define NVGPU_NVLINK_H

#include <nvgpu/types.h>

#ifdef __KERNEL__
#include <nvgpu/linux/nvlink.h>
#elif defined(__NVGPU_POSIX__)
#include <nvgpu/posix/nvlink.h>
#else
#include <nvgpu_rmos/include/nvlink.h>
#endif

#define NV_NVLINK_REG_POLL_TIMEOUT_MS           3000
#define NV_NVLINK_TIMEOUT_DELAY_US              5

#define MINION_REG_RD32(g, off) gk20a_readl(g, g->nvlink.minion_base + (off))
#define MINION_REG_WR32(g, off, v) gk20a_writel(g, g->nvlink.minion_base + (off), (v))
#define IOCTRL_REG_RD32(g, off) gk20a_readl(g, g->nvlink.ioctrl_base + (off))
#define IOCTRL_REG_WR32(g, off, v) gk20a_writel(g, g->nvlink.ioctrl_base + (off), (v))
#define MIF_REG_RD32(g, id, off) gk20a_readl(g, g->nvlink.links[(id)].mif_base + (off))
#define MIF_REG_WR32(g, id, off, v) gk20a_writel(g, g->nvlink.links[(id)].mif_base + (off), (v))
#define IPT_REG_RD32(g, off) gk20a_readl(g, g->nvlink.ipt_base + (off))
#define IPT_REG_WR32(g, off, v) gk20a_writel(g, g->nvlink.ipt_base + (off), (v))
#define TLC_REG_RD32(g, id, off) gk20a_readl(g, g->nvlink.links[(id)].tl_base + (off))
#define TLC_REG_WR32(g, id, off, v) gk20a_writel(g, g->nvlink.links[(id)].tl_base + (off), (v))
#define DLPL_REG_RD32(g, id, off) gk20a_readl(g, g->nvlink.links[(id)].dlpl_base + (off))
#define DLPL_REG_WR32(g, id, off, v) gk20a_writel(g, g->nvlink.links[(id)].dlpl_base + (off), (v))

struct gk20a;

struct nvgpu_nvlink_ioctrl_list {
	bool valid;
	u32 pri_base_addr;
	u8 intr_enum;
	u8 reset_enum;
};

struct nvgpu_nvlink_device_list {
	bool valid;
	u8 device_type;
	u8 device_id;
	u8 device_version;
	u32 pri_base_addr;
	u8 intr_enum;
	u8 reset_enum;
	u8 num_tx;
	u8 num_rx;
	u8 pll_master;
	u8 pll_master_id;
};

enum nvgpu_nvlink_endp {
	nvgpu_nvlink_endp_gpu,
	nvgpu_nvlink_endp_tegra,
	nvgpu_nvlink_endp__last,
};

enum nvgpu_nvlink_link_mode {
	nvgpu_nvlink_link_off,
	nvgpu_nvlink_link_hs,
	nvgpu_nvlink_link_safe,
	nvgpu_nvlink_link_fault,
	nvgpu_nvlink_link_rcvy_ac,
	nvgpu_nvlink_link_rcvy_sw,
	nvgpu_nvlink_link_rcvy_rx,
	nvgpu_nvlink_link_detect,
	nvgpu_nvlink_link_reset,
	nvgpu_nvlink_link_enable_pm,
	nvgpu_nvlink_link_disable_pm,
	nvgpu_nvlink_link_disable_err_detect,
	nvgpu_nvlink_link_lane_disable,
	nvgpu_nvlink_link_lane_shutdown,
	nvgpu_nvlink_link__last,
};

enum nvgpu_nvlink_sublink_mode {
	nvgpu_nvlink_sublink_tx_hs,
	nvgpu_nvlink_sublink_tx_enable_pm,
	nvgpu_nvlink_sublink_tx_disable_pm,
	nvgpu_nvlink_sublink_tx_single_lane,
	nvgpu_nvlink_sublink_tx_safe,
	nvgpu_nvlink_sublink_tx_off,
	nvgpu_nvlink_sublink_tx_common,
	nvgpu_nvlink_sublink_tx_common_disable,
	nvgpu_nvlink_sublink_tx_data_ready,
	nvgpu_nvlink_sublink_tx_prbs_en,
	nvgpu_nvlink_sublink_tx__last,
	/* RX */
	nvgpu_nvlink_sublink_rx_hs,
	nvgpu_nvlink_sublink_rx_enable_pm,
	nvgpu_nvlink_sublink_rx_disable_pm,
	nvgpu_nvlink_sublink_rx_single_lane,
	nvgpu_nvlink_sublink_rx_safe,
	nvgpu_nvlink_sublink_rx_off,
	nvgpu_nvlink_sublink_rx_rxcal,
	nvgpu_nvlink_sublink_rx__last,
};

struct nvgpu_nvlink_conn_info {
	enum nvgpu_nvlink_endp device_type;
	u32 link_number;
	bool is_connected;
};

struct nvgpu_nvlink_link {
	bool valid;
	struct gk20a *g;
	u8 link_id;

	u32 dlpl_base;
	u8 dlpl_version;

	u32 tl_base;
	u8 tl_version;

	u32 mif_base;
	u8 mif_version;

	u8 intr_enum;
	u8 reset_enum;

	bool dl_init_done;

	u8 pll_master_link_id;
	u8 pll_slave_link_id;

	struct nvgpu_nvlink_conn_info remote_info;
	void *priv;
};

#define NVLINK_MAX_LINKS_SW 6

enum nvgpu_nvlink_speed {
	nvgpu_nvlink_speed_25G,
	nvgpu_nvlink_speed_20G,
	nvgpu_nvlink_speed__last,
};

struct nvgpu_nvlink_dev {
	struct nvgpu_nvlink_ioctrl_list *ioctrl_table;
	u32 io_num_entries;

	struct nvgpu_nvlink_device_list *device_table;
	u32 num_devices;

	struct nvgpu_nvlink_link links[NVLINK_MAX_LINKS_SW];

	u8 dlpl_type;
	u32 dlpl_base[NVLINK_MAX_LINKS_SW];

	u8 tl_type;
	u32 tl_base[NVLINK_MAX_LINKS_SW];

	u8 mif_type;
	u32 mif_base[NVLINK_MAX_LINKS_SW];

	u8 ipt_type;
	u32 ipt_base;
	u8 ipt_version;

	u8 dlpl_multicast_type;
	u8 dlpl_multicast_version;
	u32 dlpl_multicast_base;

	u8 tl_multicast_type;
	u8 tl_multicast_version;
	u32 tl_multicast_base;

	u8 mif_multicast_type;
	u8 mif_multicast_version;
	u32 mif_multicast_base;

	u8 ioctrl_type;
	u32 ioctrl_base;

	u8 minion_type;
	u32 minion_base;
	u8 minion_version;

	u32 discovered_links;

	/* VBIOS settings */
	u32 link_disable_mask;
	u32 link_mode_mask;
	u32 link_refclk_mask;
	u8 train_at_boot;
	u32 ac_coupling_mask;

	u32 connected_links;
	u32 initialized_links;
	u32 enabled_links;
	u32 init_pll_done;

	enum nvgpu_nvlink_speed speed;

	/* tlc cached errors */
	u32 tlc_rx_err_status_0[NVLINK_MAX_LINKS_SW];
	u32 tlc_rx_err_status_1[NVLINK_MAX_LINKS_SW];
	u32 tlc_tx_err_status_0[NVLINK_MAX_LINKS_SW];

	/* priv struct */
	void *priv;
};

int nvgpu_nvlink_enumerate(struct gk20a *g);
int nvgpu_nvlink_train(struct gk20a *g, u32 link_id, bool from_off);
int nvgpu_nvlink_read_dt_props(struct gk20a *g);

int nvgpu_nvlink_probe(struct gk20a *g);
int nvgpu_nvlink_remove(struct gk20a *g);

void nvgpu_mss_nvlink_init_credits(struct gk20a *g);

#endif /* NVGPU_NVLINK_H */
