/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/gk20a.h>
#include <nvgpu/nvlink.h>
#include <nvgpu/enabled.h>

#ifdef CONFIG_TEGRA_NVLINK
/*
 * WAR: use this function to find detault link, as only one is supported
 * on the library for now
 * Returns NVLINK_MAX_LINKS_SW on failure
 */
static u32 __nvgpu_nvlink_get_link(struct nvlink_device *ndev)
{
	u32 link_id;
	struct gk20a *g = (struct gk20a *) ndev->priv;

	if (!g)
		return NVLINK_MAX_LINKS_SW;

	/* Lets find the detected link */
	if (g->nvlink.initialized_links)
		link_id = ffs(g->nvlink.initialized_links) - 1;
	else
		return NVLINK_MAX_LINKS_SW;

	if (g->nvlink.links[link_id].remote_info.is_connected)
		return link_id;

	return NVLINK_MAX_LINKS_SW;
}

static int nvgpu_nvlink_early_init(struct nvlink_device *ndev)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	int err;

	/* For now master topology is the only one supported */
	if (!ndev->is_master) {
		nvgpu_log(g, gpu_dbg_info | gpu_dbg_nvlink,
					"dGPU is not master of Nvlink link");
		return -EINVAL;
	}

	err = g->ops.nvlink.early_init(g);
	return err;
}

static int nvgpu_nvlink_link_early_init(struct nvlink_device *ndev)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	int err;
	u32 link_id;

	/*
	 * First check the topology and setup connectivity
	 * HACK: we are only enabling one link for now!!!
	 */
	link_id = ffs(g->nvlink.discovered_links) - 1;
	g->nvlink.links[link_id].remote_info.is_connected = true;
	g->nvlink.links[link_id].remote_info.device_type =
							nvgpu_nvlink_endp_tegra;
	err = g->ops.nvlink.link_early_init(g, BIT(link_id));

	if (err == 0) {
		g->nvlink.links[link_id].priv = (void *) &(ndev->link);
		ndev->link.priv = (void *) g;
	}
	return err;
}

static int nvgpu_nvlink_interface_init(struct nvlink_device *ndev)
{
	int err;
	struct gk20a *g = (struct gk20a *) ndev->priv;

	err = g->ops.nvlink.interface_init(g);
	return err;
}

static int nvgpu_nvlink_interface_disable(struct nvlink_device *ndev)
{
	int err = 0;
	struct gk20a *g = (struct gk20a *) ndev->priv;

	if (g->ops.nvlink.interface_disable)
		err = g->ops.nvlink.interface_disable(g);
	return err;
}

static int nvgpu_nvlink_shutdown(struct nvlink_device *ndev)
{
	int err;
	struct gk20a *g = (struct gk20a *) ndev->priv;

	err = g->ops.nvlink.shutdown(g);
	return 0;
}

static int nvgpu_nvlink_reg_init(struct nvlink_device *ndev)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	int err;

	err = g->ops.nvlink.reg_init(g);

	return err;
}

static u32 nvgpu_nvlink_get_link_mode(struct nvlink_device *ndev)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	u32 link_id;
	u32 mode;

	link_id = __nvgpu_nvlink_get_link(ndev);
	if (link_id == NVLINK_MAX_LINKS_SW)
		return -EINVAL;

	mode = g->ops.nvlink.link_get_mode(g, link_id);

	switch (mode) {
	case nvgpu_nvlink_link_off:
		return NVLINK_LINK_OFF;
	case nvgpu_nvlink_link_hs:
		return NVLINK_LINK_HS;
	case nvgpu_nvlink_link_safe:
		return NVLINK_LINK_SAFE;
	case nvgpu_nvlink_link_fault:
		return NVLINK_LINK_FAULT;
	case nvgpu_nvlink_link_rcvy_ac:
		return NVLINK_LINK_RCVY_AC;
	case nvgpu_nvlink_link_rcvy_sw:
		return NVLINK_LINK_RCVY_SW;
	case nvgpu_nvlink_link_rcvy_rx:
		return NVLINK_LINK_RCVY_RX;
	case nvgpu_nvlink_link_detect:
		return NVLINK_LINK_DETECT;
	case nvgpu_nvlink_link_reset:
		return NVLINK_LINK_RESET;
	case nvgpu_nvlink_link_enable_pm:
		return NVLINK_LINK_ENABLE_PM;
	case nvgpu_nvlink_link_disable_pm:
		return NVLINK_LINK_DISABLE_PM;
	case nvgpu_nvlink_link_disable_err_detect:
		return NVLINK_LINK_DISABLE_ERR_DETECT;
	case nvgpu_nvlink_link_lane_disable:
		return NVLINK_LINK_LANE_DISABLE;
	case nvgpu_nvlink_link_lane_shutdown:
		return NVLINK_LINK_LANE_SHUTDOWN;
	default:
		nvgpu_log(g, gpu_dbg_info | gpu_dbg_nvlink,
						"unsupported mode %u", mode);
	}

	return NVLINK_LINK_OFF;
}

static u32 nvgpu_nvlink_get_link_state(struct nvlink_device *ndev)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	u32 link_id;

	link_id = __nvgpu_nvlink_get_link(ndev);
	if (link_id == NVLINK_MAX_LINKS_SW)
		return -EINVAL;

	return g->ops.nvlink.link_get_state(g, link_id);
}

static int nvgpu_nvlink_set_link_mode(struct nvlink_device *ndev, u32 mode)
{

	struct gk20a *g = (struct gk20a *) ndev->priv;
	u32 link_id;
	u32 mode_sw;

	link_id = __nvgpu_nvlink_get_link(ndev);
	if (link_id == NVLINK_MAX_LINKS_SW)
		return -EINVAL;

	switch (mode) {
	case NVLINK_LINK_OFF:
		mode_sw = nvgpu_nvlink_link_off;
		break;
	case NVLINK_LINK_HS:
		mode_sw = nvgpu_nvlink_link_hs;
		break;
	case NVLINK_LINK_SAFE:
		mode_sw = nvgpu_nvlink_link_safe;
		break;
	case NVLINK_LINK_FAULT:
		mode_sw = nvgpu_nvlink_link_fault;
		break;
	case NVLINK_LINK_RCVY_AC:
		mode_sw = nvgpu_nvlink_link_rcvy_ac;
		break;
	case NVLINK_LINK_RCVY_SW:
		mode_sw = nvgpu_nvlink_link_rcvy_sw;
		break;
	case NVLINK_LINK_RCVY_RX:
		mode_sw = nvgpu_nvlink_link_rcvy_rx;
		break;
	case NVLINK_LINK_DETECT:
		mode_sw = nvgpu_nvlink_link_detect;
		break;
	case NVLINK_LINK_RESET:
		mode_sw = nvgpu_nvlink_link_reset;
		break;
	case NVLINK_LINK_ENABLE_PM:
		mode_sw = nvgpu_nvlink_link_enable_pm;
		break;
	case NVLINK_LINK_DISABLE_PM:
		mode_sw = nvgpu_nvlink_link_disable_pm;
		break;
	case NVLINK_LINK_LANE_DISABLE:
		mode_sw = nvgpu_nvlink_link_lane_disable;
		break;
	case NVLINK_LINK_LANE_SHUTDOWN:
		mode_sw = nvgpu_nvlink_link_lane_shutdown;
		break;
	default:
		mode_sw = nvgpu_nvlink_link_off;
	}

	return g->ops.nvlink.link_set_mode(g, link_id, mode_sw);
}

static void nvgpu_nvlink_get_tx_sublink_state(struct nvlink_device *ndev, u32 *state)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	u32 link_id;

	link_id = __nvgpu_nvlink_get_link(ndev);
	if (link_id == NVLINK_MAX_LINKS_SW)
		return;
	if (state)
		*state = g->ops.nvlink.get_tx_sublink_state(g, link_id);
}

static void nvgpu_nvlink_get_rx_sublink_state(struct nvlink_device *ndev, u32 *state)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	u32 link_id;

	link_id = __nvgpu_nvlink_get_link(ndev);
	if (link_id == NVLINK_MAX_LINKS_SW)
		return;
	if (state)
		*state = g->ops.nvlink.get_rx_sublink_state(g, link_id);
}

static u32 nvgpu_nvlink_get_sublink_mode(struct nvlink_device *ndev,
						bool is_rx_sublink)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	u32 link_id;
	u32 mode;

	link_id = __nvgpu_nvlink_get_link(ndev);
	if (link_id == NVLINK_MAX_LINKS_SW)
		return -EINVAL;

	mode = g->ops.nvlink.get_sublink_mode(g, link_id, is_rx_sublink);

	switch (mode) {
	case nvgpu_nvlink_sublink_tx_hs:
		return NVLINK_TX_HS;
	case nvgpu_nvlink_sublink_tx_off:
		return NVLINK_TX_OFF;
	case nvgpu_nvlink_sublink_tx_single_lane:
		return NVLINK_TX_SINGLE_LANE;
	case nvgpu_nvlink_sublink_tx_safe:
		return NVLINK_TX_SAFE;
	case nvgpu_nvlink_sublink_tx_enable_pm:
		return NVLINK_TX_ENABLE_PM;
	case nvgpu_nvlink_sublink_tx_disable_pm:
		return NVLINK_TX_DISABLE_PM;
	case nvgpu_nvlink_sublink_tx_common:
		return NVLINK_TX_COMMON;
	case nvgpu_nvlink_sublink_tx_common_disable:
		return NVLINK_TX_COMMON_DISABLE;
	case nvgpu_nvlink_sublink_tx_data_ready:
		return NVLINK_TX_DATA_READY;
	case nvgpu_nvlink_sublink_tx_prbs_en:
		return NVLINK_TX_PRBS_EN;
	case nvgpu_nvlink_sublink_rx_hs:
		return NVLINK_RX_HS;
	case nvgpu_nvlink_sublink_rx_enable_pm:
		return NVLINK_RX_ENABLE_PM;
	case nvgpu_nvlink_sublink_rx_disable_pm:
		return NVLINK_RX_DISABLE_PM;
	case nvgpu_nvlink_sublink_rx_single_lane:
		return NVLINK_RX_SINGLE_LANE;
	case nvgpu_nvlink_sublink_rx_safe:
		return NVLINK_RX_SAFE;
	case nvgpu_nvlink_sublink_rx_off:
		return NVLINK_RX_OFF;
	case nvgpu_nvlink_sublink_rx_rxcal:
		return NVLINK_RX_RXCAL;
	default:
		nvgpu_log(g, gpu_dbg_nvlink, "Unsupported mode: %u", mode);
		break;
	}

	if (is_rx_sublink)
		return NVLINK_RX_OFF;
	return NVLINK_TX_OFF;
}

static int nvgpu_nvlink_set_sublink_mode(struct nvlink_device *ndev,
						bool is_rx_sublink, u32 mode)
{
	struct gk20a *g = (struct gk20a *) ndev->priv;
	u32 link_id;
	u32 mode_sw;

	link_id = __nvgpu_nvlink_get_link(ndev);
	if (link_id == NVLINK_MAX_LINKS_SW)
		return -EINVAL;

	if (!is_rx_sublink) {
		switch (mode) {
		case NVLINK_TX_HS:
			mode_sw = nvgpu_nvlink_sublink_tx_hs;
			break;
		case NVLINK_TX_ENABLE_PM:
			mode_sw = nvgpu_nvlink_sublink_tx_enable_pm;
			break;
		case NVLINK_TX_DISABLE_PM:
			mode_sw = nvgpu_nvlink_sublink_tx_disable_pm;
			break;
		case NVLINK_TX_SINGLE_LANE:
			mode_sw = nvgpu_nvlink_sublink_tx_single_lane;
			break;
		case NVLINK_TX_SAFE:
			mode_sw = nvgpu_nvlink_sublink_tx_safe;
			break;
		case NVLINK_TX_OFF:
			mode_sw = nvgpu_nvlink_sublink_tx_off;
			break;
		case NVLINK_TX_COMMON:
			mode_sw = nvgpu_nvlink_sublink_tx_common;
			break;
		case NVLINK_TX_COMMON_DISABLE:
			mode_sw = nvgpu_nvlink_sublink_tx_common_disable;
			break;
		case NVLINK_TX_DATA_READY:
			mode_sw = nvgpu_nvlink_sublink_tx_data_ready;
			break;
		case NVLINK_TX_PRBS_EN:
			mode_sw = nvgpu_nvlink_sublink_tx_prbs_en;
			break;
		default:
			return -EINVAL;
		}
	} else {
		switch (mode) {
		case NVLINK_RX_HS:
			mode_sw = nvgpu_nvlink_sublink_rx_hs;
			break;
		case NVLINK_RX_ENABLE_PM:
			mode_sw = nvgpu_nvlink_sublink_rx_enable_pm;
			break;
		case NVLINK_RX_DISABLE_PM:
			mode_sw = nvgpu_nvlink_sublink_rx_disable_pm;
			break;
		case NVLINK_RX_SINGLE_LANE:
			mode_sw = nvgpu_nvlink_sublink_rx_single_lane;
			break;
		case NVLINK_RX_SAFE:
			mode_sw = nvgpu_nvlink_sublink_rx_safe;
			break;
		case NVLINK_RX_OFF:
			mode_sw = nvgpu_nvlink_sublink_rx_off;
			break;
		case NVLINK_RX_RXCAL:
			mode_sw = nvgpu_nvlink_sublink_rx_rxcal;
			break;
		default:
			return -EINVAL;
		}
	}

	return g->ops.nvlink.set_sublink_mode(g, link_id, is_rx_sublink,
								mode_sw);
}

static int nvgpu_nvlink_init_ops(struct gk20a *g)
{
	struct nvlink_device *ndev = g->nvlink.priv;

	if (!ndev)
		return -EINVAL;

	/* Fill in device struct */
	ndev->dev_ops.dev_early_init = nvgpu_nvlink_early_init;
	ndev->dev_ops.dev_interface_init = nvgpu_nvlink_interface_init;
	ndev->dev_ops.dev_reg_init = nvgpu_nvlink_reg_init;
	ndev->dev_ops.dev_interface_disable = nvgpu_nvlink_interface_disable;
	ndev->dev_ops.dev_shutdown = nvgpu_nvlink_shutdown;

	/* Fill in the link struct */
	ndev->link.device_id = ndev->device_id;
	ndev->link.mode = NVLINK_LINK_OFF;
	ndev->link.is_sl_supported = false;
	ndev->link.link_ops.get_link_mode = nvgpu_nvlink_get_link_mode;
	ndev->link.link_ops.set_link_mode = nvgpu_nvlink_set_link_mode;
	ndev->link.link_ops.get_sublink_mode = nvgpu_nvlink_get_sublink_mode;
	ndev->link.link_ops.set_sublink_mode = nvgpu_nvlink_set_sublink_mode;
	ndev->link.link_ops.get_link_state = nvgpu_nvlink_get_link_state;
	ndev->link.link_ops.get_tx_sublink_state =
					nvgpu_nvlink_get_tx_sublink_state;
	ndev->link.link_ops.get_rx_sublink_state =
					nvgpu_nvlink_get_rx_sublink_state;
	ndev->link.link_ops.link_early_init =
					nvgpu_nvlink_link_early_init;

	return 0;
}

int nvgpu_nvlink_enumerate(struct gk20a *g)
{
	struct nvlink_device *ndev = (struct nvlink_device *) g->nvlink.priv;

	if (!ndev)
		return -ENODEV;

	return nvlink_enumerate(ndev);
}

int nvgpu_nvlink_train(struct gk20a *g, u32 link_id, bool from_off)
{
	struct nvlink_device *ndev = (struct nvlink_device *) g->nvlink.priv;

	if (!ndev)
		return -ENODEV;

	/* Check if the link is connected */
	if (!g->nvlink.links[link_id].remote_info.is_connected)
		return -ENODEV;

	if (from_off)
		return nvlink_transition_intranode_conn_off_to_safe(ndev);

	return nvlink_train_intranode_conn_safe_to_hs(ndev);
}

#endif

int nvgpu_nvlink_probe(struct gk20a *g)
{
#ifdef CONFIG_TEGRA_NVLINK
	int err;
	struct nvlink_device *ndev;

	/* Allocating structures */
	ndev = nvgpu_kzalloc(g, sizeof(struct nvlink_device));
	if (!ndev) {
		nvgpu_err(g, "OOM while allocating nvlink device struct");
		return -ENOMEM;
	}

	ndev->priv = (void *) g;
	g->nvlink.priv = (void *) ndev;

	err = nvgpu_nvlink_read_dt_props(g);
	if (err)
		goto free_ndev;

	err = nvgpu_nvlink_init_ops(g);
	if (err)
		goto free_ndev;

	/* Register device with core driver*/
	err = nvlink_register_device(ndev);
	if (err) {
		nvgpu_err(g, "failed on nvlink device registration");
		goto free_ndev;
	}

	/* Register link with core driver */
	err = nvlink_register_link(&ndev->link);
	if (err) {
		nvgpu_err(g, "failed on nvlink link registration");
		goto unregister_ndev;
	}

	/* Enable NVLINK support */
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_NVLINK, true);
	return 0;

unregister_ndev:
	nvlink_unregister_device(ndev);

free_ndev:
	nvgpu_kfree(g, ndev);
	g->nvlink.priv = NULL;
	return err;
#else
	return -ENODEV;
#endif
}

int nvgpu_nvlink_remove(struct gk20a *g)
{
#ifdef CONFIG_TEGRA_NVLINK
	struct nvlink_device *ndev;
	int err;

	if (!nvgpu_is_enabled(g, NVGPU_SUPPORT_NVLINK))
		return -ENODEV;

	ndev = g->nvlink.priv;
	if(!ndev)
		return -ENODEV;

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_NVLINK, false);

	err = nvlink_unregister_link(&ndev->link);
	if (err) {
		nvgpu_err(g, "failed on nvlink link unregistration");
		return err;
	}

	err = nvlink_unregister_device(ndev);
	if (err) {
		nvgpu_err(g, "failed on nvlink device unregistration");
		return err;
	}

	nvgpu_kfree(g, ndev);

	return 0;
#else
	return -ENODEV;
#endif
}
