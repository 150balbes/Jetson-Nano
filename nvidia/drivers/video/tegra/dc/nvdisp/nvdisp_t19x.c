/*
 * drivers/video/tegra/dc/nvdisp/nvdisp_t19x.c
 *
 * Copyright (c) 2017-2020, NVIDIA CORPORATION, All rights reserved.
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

#include "dc_priv.h"
#include "nvdisp.h"
#include "hw_nvdisp_t19x_nvdisp.h"

#define RG_STATUS_STALLED 0x08
#define RG_STATUS_POLL_TIMEOUT_MS 10
#define RG_STATUS_POLL_INTERVAL_US 1
#define TEGRA_WINBUF_ADDR_FLAG_BLOCKLINEAR ((dma_addr_t)0x1 << 39)

/*
 * As per nvdisplay programming guidelines, only hubclk,dispclk and
 * pclk0 clocks should be enabled before accessing any display register
 * in head0 power domain. But, as BL is unconditionally unpowergating
 * all display powerdomains, enabling pclk1 and pclk2 and pclk3 clocks
 * even here as WAR to avoid display register access issues in kernel
 * Bug 200343370 tracks issue in BL.
 */
static struct tegra_dc_pd_clk_info t19x_disp_pd0_clk_info[] = {
	{
		.name = "nvdisplayhub",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_disp",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_p0",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_p1",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_p2",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_p3",
		.clk = NULL,
	},
};

static struct tegra_dc_pd_clk_info t19x_disp_pd1_clk_info[] = {
	{
		.name = "nvdisplay_p1",
		.clk = NULL,
	},
};

static struct tegra_dc_pd_clk_info t19x_disp_pd2_clk_info[] = {
	{
		.name = "nvdisplay_p2",
		.clk = NULL,
	},
	{
		.name = "nvdisplay_p3",
		.clk = NULL,
	},
};

/*
 * NOTE: Keep the following power domains ordered according to their head owner.
 */
static struct tegra_dc_pd_info t19x_disp_pd_info[] = {
	/* Head0 power domain */
	{
		.of_id = {
			{ .compatible = "nvidia,tegra194-disa-pd", },
			{},
		},
		.pg_id = -1,
		.head_owner = 0,
		.head_mask = 0x1,	/* Head(s):	0 */
		.win_mask = 0x1,	/* Window(s):	0 */
		.domain_clks = t19x_disp_pd0_clk_info,
		.nclks = ARRAY_SIZE(t19x_disp_pd0_clk_info),
		.ref_cnt = 0,
	},
	/* Head1 power domain */
	{
		.of_id = {
			{ .compatible = "nvidia,tegra194-disb-pd", },
			{},
		},
		.pg_id = -1,
		.head_owner = 1,
		.head_mask = 0x2,	/* Head(s):	1 */
		.win_mask = 0x6,	/* Window(s):	1,2 */
		.domain_clks = t19x_disp_pd1_clk_info,
		.nclks = ARRAY_SIZE(t19x_disp_pd1_clk_info),
		.ref_cnt = 0,
	},
	/* Head2 power domain */
	{
		.of_id = {
			{ .compatible = "nvidia,tegra194-disc-pd", },
			{},
		},
		.pg_id = -1,
		.head_owner = 2,
		.head_mask = 0xc,	/* Head(s):	2,3 */
		.win_mask = 0x38,	/* Window(s):	3,4,5 */
		.domain_clks = t19x_disp_pd2_clk_info,
		.nclks = ARRAY_SIZE(t19x_disp_pd2_clk_info),
		.ref_cnt = 0,
	},
};

static struct tegra_dc_pd_table t19x_disp_pd_table = {
	.pd_entries = t19x_disp_pd_info,
	.npd = ARRAY_SIZE(t19x_disp_pd_info),
};

int tegra_nvdisp_set_control_t19x(struct tegra_dc *dc)
{
	u32 reg, protocol;

	if (dc->out->type == TEGRA_DC_OUT_HDMI) {

		/* sor in the function name is irrelevant */
		protocol = nvdisp_t19x_sor_control_protocol_tmdsa_f();
	} else if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		   (dc->out->type == TEGRA_DC_OUT_FAKE_DP)) {

		/* sor in the function name is irrelevant */
		protocol = nvdisp_t19x_sor_control_protocol_dpa_f();
	} else {
		dev_err(&dc->ndev->dev, "%s: unsupported out_type=%d\n",
				__func__, dc->out->type);
		return -EINVAL;
	}

	switch (dc->out_ops->get_connector_instance(dc)) {
	case 0:
		reg = nvdisp_t19x_sor_control_r();
		break;
	case 1:
		reg = nvdisp_t19x_sor1_control_r();
		break;
	case 2:
		reg = nvdisp_t19x_sor2_control_r();
		break;
	case 3:
		reg = nvdisp_t19x_sor3_control_r();
		break;
	default:
		pr_err("%s: invalid sor_num:%d\n", __func__,
				dc->out_ops->get_connector_instance(dc));
		return -ENODEV;
	}

	tegra_dc_writel(dc, protocol, reg);
	tegra_dc_enable_general_act(dc);

	return 0;
}

void tegra_dc_enable_sor_t19x(struct tegra_dc *dc, int sor_num, bool enable)
{
	u32 enb;
	u32 reg_val = tegra_dc_readl(dc, nvdisp_t19x_win_options_r());

	switch (sor_num) {
	case 0:
		enb = nvdisp_t19x_win_options_sor_set_sor_enable_f();
		break;
	case 1:
		enb = nvdisp_t19x_win_options_sor1_set_sor1_enable_f();
		break;
	case 2:
		enb = nvdisp_t19x_win_options_sor2_set_sor2_enable_f();
		break;
	case 3:
		enb = nvdisp_t19x_win_options_sor3_set_sor3_enable_f();
		break;
	default:
		pr_err("%s: invalid sor_num:%d\n", __func__, sor_num);
		return;
	}

	reg_val = enable ? reg_val | enb : reg_val & ~enb;
	tegra_dc_writel(dc, reg_val, nvdisp_t19x_win_options_r());
}

inline void tegra_nvdisp_set_rg_unstall_t19x(struct tegra_dc *dc)
{
	if (dc->mode.vmode & FB_VMODE_BYPASS)
		return;

	if (tegra_dc_is_yuv420_8bpc(&dc->mode))
		tegra_dc_writel(dc,
			tegra_dc_readl(dc, nvdisp_t19x_rg_status_r()) |
			nvdisp_t19x_rg_status_unstall_force_even_set_enable_f(),
			nvdisp_t19x_rg_status_r());
}

static struct tegra_dc_sor_info t19x_sor_info[] = {
	{ .hdcp_supported = true },   /* SOR0 */
	{ .hdcp_supported = true },   /* SOR1 */
	{ .hdcp_supported = true },   /* SOR2 */
	{ .hdcp_supported = true },   /* SOR3 */
};

void tegra_dc_populate_t19x_hw_data(struct tegra_dc_hw_data *hw_data)
{
	if (!hw_data)
		return;

	hw_data->nheads = 4;
	hw_data->nwins = 6;
	hw_data->nsors = 4;
	hw_data->sor_info = t19x_sor_info;
	hw_data->pd_table = &t19x_disp_pd_table;
	hw_data->valid = true;
	hw_data->version = TEGRA_DC_HW_T19x;
}

dma_addr_t nvdisp_t19x_get_addr_flag(struct tegra_dc_win *win)
{
	dma_addr_t addr_flag = 0x0;

	if (win->flags & TEGRA_WIN_FLAG_BLOCKLINEAR)
		addr_flag |= TEGRA_WINBUF_ADDR_FLAG_BLOCKLINEAR;

	return addr_flag;
}

/**
 * tegra_dc_get_vsync_timestamp_t19x - read the vsync from
 *					the ptimer regs.
 * @dc: pointer to struct tegra_dc for the current head.
 *
 * Return : timestamp value in ns.
 */
uint64_t tegra_dc_get_vsync_timestamp_t19x(struct tegra_dc *dc)
{
	u32 lsb = 0;
	u64 msb = 0;

	lsb = tegra_dc_readl(dc, nvdisp_t19x_rg_vsync_ptimer0_r());
	msb = tegra_dc_readl(dc, nvdisp_t19x_rg_vsync_ptimer1_r());

	return (((msb << 32) | lsb) << 5);
}

/**
 * nvdisp_t19x_program_raster_lock_seq - program the raster lock
 *					sequence for t19x.
 * @dc : head pointer for which raster_lock is required.
 * @value : informs if head is to be programmed in continuous or
 * non_continuous mode. This value is passed on from the source as is.
 *
 * Return: 0 if successful else -ENODEV if dc isn't found or isn't
 * enabled.
 */
int nvdisp_t19x_program_raster_lock_seq(struct tegra_dc *dc, u32 value)
{
	int ret = 0;
	u32 old_val;

	if (!dc)
		return -ENODEV;
	/**
	 * For nvdisp_t19x_state_access_r() reg, the implicitly expected value
	 * is WRITE_ASSEMBLY | READ_ACTIVE and ideally we need not program this
	 * combination again. However currently in raster lock workflow the
	 * value of command_state_access is overwritten to write_active in sor.c
	 * and hence we have to explicitly cache and restore here.
	 */

	old_val = tegra_dc_readl(dc, nvdisp_t19x_state_access_r());

	tegra_dc_writel(dc, nvdisp_t19x_state_access_write_mux_assembly_f()
			| nvdisp_t19x_state_access_read_mux_active_f(),
			nvdisp_t19x_state_access_r());

	tegra_dc_writel(dc, nvdisp_t19x_display_cmd_option_msf_src_glb_ctrl_f()
		       | nvdisp_t19x_display_cmd_option_msf_enable_enable_f(),
			nvdisp_t19x_display_cmd_option_r());

	tegra_dc_writel(dc,
			nvdisp_t19x_display_command_control_mode_nc_display_f(),
			nvdisp_t19x_display_command_r());

	tegra_dc_writel(dc, nvdisp_t19x_cmd_state_ctrl_host_trig_enable_f() |
			nvdisp_t19x_cmd_state_ctrl_general_act_req_enable_f(),
			nvdisp_t19x_cmd_state_ctrl_r());

	ret = tegra_dc_poll_register(dc, nvdisp_t19x_rg_status_r(),
		RG_STATUS_STALLED, nvdisp_t19x_rg_status_stalled_yes_f(),
		RG_STATUS_POLL_INTERVAL_US, RG_STATUS_POLL_TIMEOUT_MS);
	if (ret) {
		dev_err(&dc->ndev->dev,
			"dc timeout waiting for RG to stall\n");
		goto restore_cmd_access_state;
	}

	if (!(value & nvdisp_t19x_display_command_control_mode_c_display_f()))
		goto restore_cmd_access_state;

	tegra_dc_writel(dc,
			nvdisp_t19x_display_command_control_mode_c_display_f(),
			nvdisp_t19x_display_command_r());

	tegra_dc_writel(dc,
			nvdisp_t19x_cmd_state_ctrl_general_act_req_enable_f(),
			nvdisp_t19x_cmd_state_ctrl_r());

restore_cmd_access_state:
	tegra_dc_writel(dc, old_val, nvdisp_t19x_state_access_r());

	return ret;
}

/**
 * nvdisp_t19x_enable_raster_lock - programs global_control signal
 *				required for multi-head RG unstall.
 * @dc : head pointer
 * @valid_heads :  a bit_array consisting of ctrl_nums of heads
 * participating in the frame_lock.
 *
 * Return : void
 */
void nvdisp_t19x_enable_raster_lock(struct tegra_dc *dc,
					const ulong valid_heads)
{
	tegra_dc_writel(dc, valid_heads, nvdisp_t19x_glb_ctrl_r());

	tegra_dc_readl(dc, nvdisp_t19x_glb_ctrl_r()); /* flush */

	tegra_dc_writel(dc, 0x00, nvdisp_t19x_glb_ctrl_r());
}

/**
 * tegra_nvdisp_program_common_win_batch_size_t19x - programs the default win
 *						request batch size (8) for T19x
 * @dc : head pointer
 *
 * Return: void
 */
inline void tegra_nvdisp_program_common_win_batch_size_t19x(struct tegra_dc *dc)
{
	tegra_dc_writel(dc,
		nvdisp_t19x_ihub_common_config_request_batch_size_8_f(),
		nvdisp_t19x_ihub_common_config_r());
}
