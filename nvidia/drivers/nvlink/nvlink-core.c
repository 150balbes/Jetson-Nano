/*
 * nvlink-core.c:
 * This driver manages the entire NVLINK system that the Tegra SOC is connected
 * to. The NVLINK core driver interfaces with the NVLINK endpoint drivers. Each
 * endpoint driver is responsible for the HW programming of 1 particular NVLINK
 * device. The core driver uses the endpoint drivers to manage the NVLINK
 * system.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform/tegra/tegra-nvlink.h>

#include <asm/cacheflush.h>

#define NVLINK_MODULE_NAME			"nvlink-core"
#define NVLINK_DEBUGFS_ROOT			"nvlink"
#define NVLINK_DEBUGFS_TESTS			"tests"
#define DEFAULT_LOOP_SLEEP_US			100
#define DEFAULT_LOOP_TIMEOUT_US			1000000
#define NVLINK_TRANSITION_HS_TIMEOUT_MS		2000
#define NVLINK_TRANSITION_SAFE_TIMEOUT_MS	5

struct nvlink_intranode_conn {
	struct nvlink_device *ndev0;
	struct nvlink_device *ndev1;
};

struct topology {
	int slave_dev_id;
	int master_dev_id;
	int slave_link_id;
	int master_link_id;
};

struct nvlink_core {
	struct nvlink_device *ndevs[NVLINK_MAX_DEVICES];
	struct nvlink_link *nlinks[NVLINK_MAX_LINKS];
	struct topology topology;
	struct nvlink_intranode_conn intranode_conn;
	struct mutex mutex;
};

/*
 * We're exporting the NVLINK driver stack's logging APIs to the NVLINK kernel
 * test modules. The logging APIs use nvlink_log_mask. Therefore, we have to
 * export nvlink_log_mask along with the logging APIs.
 */
u32 nvlink_log_mask = NVLINK_DEFAULT_LOG_MASK;
EXPORT_SYMBOL(nvlink_log_mask);

#ifdef CONFIG_DEBUG_FS
/* This is the root debugfs directory for the entire NVLINK driver stack */
struct dentry *nvlink_debugfs_root;

/*
 * This is the parent debugfs directory for NVLINK tests. We need to export this
 * symbol so that the NVLINK kernel test modules can create their debugfs nodes
 * under the correct path.
 */
struct dentry *nvlink_debugfs_tests;
EXPORT_SYMBOL(nvlink_debugfs_tests);
#endif /* CONFIG_DEBUG_FS */

static struct nvlink_core nvlink_core;

static bool nvlink_is_single_lane_mode_supported(
				struct nvlink_intranode_conn *conn)
{
	/*
	 * Single-lane mode is supported on the connection
	 * only when both of the nvlink devices support this feature.
	 */
	return (conn->ndev0->link.is_sl_supported &&
			conn->ndev1->link.is_sl_supported);
}

int nvlink_get_init_state(struct nvlink_device *ndev, enum init_state *state)
{
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}
	mutex_lock(&ndev->init_state_mutex);
	*state = ndev->init_state;
	mutex_unlock(&ndev->init_state_mutex);

	return ret;
}
EXPORT_SYMBOL(nvlink_get_init_state);

int nvlink_set_init_state(struct nvlink_device *ndev, enum init_state state)
{
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}
	if ((state >= NVLINK_INIT_STATE_INVALID) || (state < 0)) {
		nvlink_err("Invalid init state");
		return -EINVAL;
	}
	mutex_lock(&ndev->init_state_mutex);
	ndev->init_state = state;
	mutex_unlock(&ndev->init_state_mutex);

	return ret;
}
EXPORT_SYMBOL(nvlink_set_init_state);

void nvlink_print_topology(void)
{
	struct topology *topology = NULL;

	mutex_lock(&nvlink_core.mutex);
	topology = &(nvlink_core.topology);

	if (topology->master_dev_id == -1) {
		nvlink_err("Topology information not present");
		mutex_unlock(&nvlink_core.mutex);
		return;
	}
	nvlink_dbg("Master device ID: %d", topology->master_dev_id);
	nvlink_dbg("Slave device ID: %d", topology->slave_dev_id);
	nvlink_dbg("Master link ID: %d", topology->master_link_id);
	nvlink_dbg("Slave link ID: %d", topology->slave_link_id);

	if ((topology->master_dev_id == NVLINK_ENDPT_T19X) &&
			(topology->slave_dev_id == NVLINK_ENDPT_T19X)) {
		nvlink_dbg("Tegra loopback topology detected");
	}
	else if ((topology->master_dev_id == NVLINK_ENDPT_GV100) &&
			(topology->slave_dev_id == NVLINK_ENDPT_T19X)) {
		nvlink_dbg("GV100 (master) connected to Tegra (slave) ");
	}
	mutex_unlock(&nvlink_core.mutex);
}
EXPORT_SYMBOL(nvlink_print_topology);

/*
 * Record the topology information in core driver structures.
 * If the topology data is already available with core driver,
 * just verify that both the devices have same topology stored
 */
static int nvlink_update_topology(struct nvlink_device *ndev)
{
	int ret = 0;
	struct topology *topology = NULL;
	struct nvlink_intranode_conn *intranode_conn = NULL;
	int local_dev_id = ndev->device_id;
	int local_link_id = ndev->link.link_id;
	int remote_dev_id = ndev->link.remote_dev_info.device_id;
	int remote_link_id = ndev->link.remote_dev_info.link_id;

	if ((local_dev_id >= NVLINK_MAX_DEVICES) ||
					(remote_dev_id >= NVLINK_MAX_DEVICES)) {
		nvlink_err("Invalid device_id");
		return -ENODEV;
	}
	if ((local_link_id >= NVLINK_MAX_LINKS) ||
					(remote_link_id >= NVLINK_MAX_LINKS)) {
		nvlink_err("Invalid link_id");
		return -ENODEV;
	}

	mutex_lock(&nvlink_core.mutex);
	topology = &(nvlink_core.topology);
	intranode_conn = &(nvlink_core.intranode_conn);

	/*
	 * If ndev is the first device to register, we need to store the
	 * topology; on consequent call from other device, we verify the
	 * topology. We should check the topology information provided
	 * from both the endpoints is same. This will prevent two devices
	 * from registering as master.
	 */
	if (topology->master_dev_id == -1) {
		nvlink_dbg("Storing the topology information with core driver");
		if (ndev->is_master) {
			nvlink_dbg("Device %d is the master", ndev->device_id);
			topology->master_dev_id = local_dev_id;
			topology->master_link_id = local_link_id;
			topology->slave_dev_id = remote_dev_id;
			topology->slave_link_id = remote_link_id;
		} else {
			nvlink_dbg("Device %d is the slave", ndev->device_id);
			topology->master_dev_id = remote_dev_id;
			topology->master_link_id = remote_link_id;
			topology->slave_dev_id = local_dev_id;
			topology->slave_link_id = local_link_id;
		}
		nvlink_dbg("Topology stored in core driver structure");
	} else {
		/*
		 * Verify the topology information in ndev against the topology
		 * information stored in core driver struct.
		 */
		if (ndev->is_master) {
			nvlink_dbg("Device %d is the master", ndev->device_id);
			if ((topology->master_dev_id != local_dev_id) ||
				(topology->master_link_id != local_link_id) ||
				(topology->slave_dev_id != remote_dev_id) ||
				(topology->slave_link_id != remote_link_id)) {
				nvlink_err("Topology Mismatch!");
				ret = -EINVAL;
				goto topology_err;
			} else {
				nvlink_dbg("Topology Match!");
			}
		} else {
			nvlink_dbg("Device %d is the slave", ndev->device_id);
			if ((topology->master_dev_id != remote_dev_id) ||
				(topology->master_link_id != remote_link_id) ||
				(topology->slave_dev_id != local_dev_id) ||
				(topology->slave_link_id != local_link_id)) {
				nvlink_err("Topology Mismatch!");
				ret = -EINVAL;
				goto topology_err;
			} else {
				nvlink_dbg("Topology Match!");
			}
		}
	}
	/* Check if topology is one of the below supported topologies -
	 * 1. Tegra Loopback
	 * 2. dGPU as master connected to Tegra as slave
	 *
	 * else report error
	 */
	if (topology->slave_dev_id == NVLINK_ENDPT_GV100) {
		nvlink_err("Topology with dGPU as slave is not supported!");
		ret = -EINVAL;
		goto topology_err;
	}

	intranode_conn->ndev0 =	nvlink_core.ndevs[topology->master_dev_id];
	intranode_conn->ndev1 =	nvlink_core.ndevs[topology->slave_dev_id];

	goto success;
topology_err:
	nvlink_core.ndevs[local_dev_id] = NULL;
	nvlink_core.ndevs[remote_dev_id] = NULL;
	nvlink_core.topology.master_link_id = -1;
	nvlink_core.topology.master_dev_id = -1;
	nvlink_core.topology.slave_link_id = -1;
	nvlink_core.topology.slave_dev_id = -1;
success:
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}

/*
 * This is a wrapper function for an ARM64 cache flush API. This API is used in
 * NVLINK kernel test modules. We've created this NVLINK wrapper because we
 * don't want to directly export the ARM64 API. We want to minimize the exposure
 * of this API outside of the kernel. By creating this NVLINK wrapper we're
 * trying to ensure that only NVLINK kernel test modules will use this API
 * outside of the kernel.
 */
void __nvlink_dma_flush_area(const void *ptr, size_t size)
{
	__dma_flush_area(ptr, size);
}
EXPORT_SYMBOL(__nvlink_dma_flush_area);

int nvlink_register_device(struct nvlink_device *ndev)
{
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	if (ndev->device_id >= NVLINK_MAX_DEVICES) {
		nvlink_err("Invalid device_id");
		ret = -ENODEV;
		goto fail;
	}

	mutex_lock(&nvlink_core.mutex);

	/* Allow each device to register just once */
	if (nvlink_core.ndevs[ndev->device_id] != NULL) {
		nvlink_err("Device %u has already registered with core driver",
							ndev->device_id);
		ret = -EINVAL;
		goto release_mutex;
	}

	mutex_init(&ndev->init_state_mutex);
	ret = nvlink_set_init_state(ndev, NVLINK_DEV_OFF);
	if (ret < 0) {
		nvlink_err("Error initializing init state to DEV_OFF");
		mutex_destroy(&ndev->init_state_mutex);
		goto release_mutex;
	}

	nvlink_core.ndevs[ndev->device_id] = ndev;
	mutex_unlock(&nvlink_core.mutex);

	ret = nvlink_update_topology(ndev);
	if (ret < 0) {
		mutex_destroy(&ndev->init_state_mutex);
		goto fail;
	}

	nvlink_dbg("Device registration successful!");
	goto success;

release_mutex:
	mutex_unlock(&nvlink_core.mutex);
fail:
	nvlink_err("Device registration failed!");
success:
	return ret;
}
EXPORT_SYMBOL(nvlink_register_device);

int nvlink_register_link(struct nvlink_link *link)
{
	int ret = 0;

	if (!link) {
		nvlink_err("Invalid link struct pointer");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);
	if (link->link_id >= NVLINK_MAX_LINKS) {
		nvlink_err("Invalid link_id");
		ret = -ENODEV;
		goto fail;
	}
	link->mode = NVLINK_LINK_OFF;
	link->is_connected = false;
	nvlink_core.nlinks[link->link_id] = link;

	goto success;

fail:
	nvlink_err("Link register failed!");
success:
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}
EXPORT_SYMBOL(nvlink_register_link);

int nvlink_unregister_device(struct nvlink_device* ndev)
{
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);

	if (ndev->device_id >= NVLINK_MAX_DEVICES) {
		nvlink_err("Invalid device_id");
		ret = -ENODEV;
		goto fail;
	}
	mutex_destroy(&ndev->init_state_mutex);
	nvlink_core.ndevs[ndev->device_id] = NULL;

	nvlink_core.topology.master_link_id = -1;
	nvlink_core.topology.master_dev_id = -1;
	nvlink_core.topology.slave_link_id = -1;
	nvlink_core.topology.slave_dev_id = -1;

	goto success;

fail:
	nvlink_err("Device unregister failed!");
success:
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}
EXPORT_SYMBOL(nvlink_unregister_device);

int nvlink_unregister_link(struct nvlink_link *link)
{
	int ret = 0;

	if (!link) {
		nvlink_err("Invalid link struct pointer");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);
	if (link->link_id >= NVLINK_MAX_LINKS) {
		nvlink_err("Invalid link_id");
		ret = -ENODEV;
		goto fail;
	}

	nvlink_core.nlinks[link->link_id] = NULL;

	goto success;

fail:
	nvlink_err("Link unregister failed!");
success:
	mutex_unlock(&nvlink_core.mutex);
	return ret;
}
EXPORT_SYMBOL(nvlink_unregister_link);

static int nvlink_poll_link_state(struct nvlink_device *ndev, u32 link_state,
			u32 timeout_ms)
{
	u32 link_mode;
	u32 timeout_us = timeout_ms * 1000;

	link_mode = ndev->link.link_ops.get_link_mode(ndev);
	while (link_mode != link_state) {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
		if (timeout_us <= 0) {
			nvlink_err("Timeout occurred while polling on link");
			return -ETIMEDOUT;
		}
		link_mode = ndev->link.link_ops.get_link_mode(ndev);
	}

	return 0;
}

static int nvlink_poll_tx_sublink_state(struct nvlink_device *ndev,
				u32 tx_sublink_state, u32 timeout_ms)
{
	u32 sublink_mode;
	u32 timeout_us = timeout_ms * 1000;

	sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev, false);
	while (sublink_mode != tx_sublink_state) {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
		if (timeout_us <= 0) {
			nvlink_err("Timeout while polling on Tx sublink");
			return -ETIMEDOUT;
		}
		sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev,
									false);
	}

	return 0;
}

static int nvlink_poll_rx_sublink_state(struct nvlink_device *ndev,
				u32 rx_sublink_state, u32 timeout_ms)
{
	u32 sublink_mode;
	u32 timeout_us = timeout_ms * 1000;

	sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev, true);
	while (sublink_mode != rx_sublink_state) {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
		if (timeout_us <= 0) {
			nvlink_err("Timeout while polling on Rx sublink");
			return -ETIMEDOUT;
		}
		sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev, true);
	}

	return 0;
}

static int nvlink_poll_sublink_state(struct nvlink_device *ndev0,
				u32 tx_sublink_state,
				struct nvlink_device *ndev1,
				u32 rx_sublink_state,
				u32 timeout_ms)
{
	int status;

	status = nvlink_poll_tx_sublink_state(ndev0, tx_sublink_state,
						timeout_ms);
	if (status) {
		/* polling on tx sublink failed. skip any rx polling */
		return status;
	}

	status = nvlink_poll_rx_sublink_state(ndev1, rx_sublink_state,
						timeout_ms);

	return status;
}


/* For a given link, check whether tx sublink mode is at the requested mode */
static bool nvlink_check_tx_sublink_mode(struct nvlink_device *ndev,
					u32 sublink_mode)
{
	u32 curr_sublink_mode = NVLINK_TX_OFF;

	curr_sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev, false);
	switch (sublink_mode) {
	case NVLINK_TX_OFF:
		if (curr_sublink_mode == NVLINK_TX_OFF) {
			nvlink_dbg("Tx sublink is in OFF mode");
			return true;
		}
		break;
	case NVLINK_TX_SAFE:
		if (curr_sublink_mode == NVLINK_TX_SAFE) {
			nvlink_dbg("Tx sublink is in SAFE mode");
			return true;
		}
		break;
	case NVLINK_TX_HS:
		if ((curr_sublink_mode == NVLINK_TX_SINGLE_LANE)
			|| (curr_sublink_mode == NVLINK_TX_HS)) {
			nvlink_dbg("Tx sublink is in HS mode");
			return true;
		}
		break;
	}

	/* return false for default case or the states are not matching */
	return false;
}

/* For a given link, check whether rx sublink mode is at the requested mode */
static bool nvlink_check_rx_sublink_mode(struct nvlink_device *ndev,
						u32 sublink_mode)
{
	u32 curr_sublink_mode = NVLINK_RX_OFF;

	curr_sublink_mode = ndev->link.link_ops.get_sublink_mode(ndev, true);
	switch (sublink_mode) {
	case NVLINK_RX_OFF:
		if (curr_sublink_mode == NVLINK_RX_OFF) {
			nvlink_dbg("Rx sublink is in OFF mode");
			return true;
		}
		break;
	case NVLINK_RX_SAFE:
		if (curr_sublink_mode == NVLINK_RX_SAFE) {
			nvlink_dbg("Rx sublink is in SAFE mode");
			return true;
		}
		break;
	case NVLINK_RX_HS:
		if ((curr_sublink_mode == NVLINK_RX_SINGLE_LANE)
			|| (curr_sublink_mode == NVLINK_RX_HS)) {
			nvlink_dbg("Rx sublink is in HS mode");
			return true;
		}
		break;
	}

	/* return false for default case or the states are not matching */
	return false;
}

/* For the given link, check whether the link mode is at the requested mode */
static bool nvlink_check_link_mode(struct nvlink_device *ndev, u32 link_mode)
{
	u32 curr_link_mode = NVLINK_LINK_OFF;

	curr_link_mode = ndev->link.link_ops.get_link_mode(ndev);
	if (link_mode == curr_link_mode)
		return true;
	else
		return false;
}

/* Check if the given intranode connection is in the specified mode */
static int nvlink_check_intranode_conn_mode(
				struct nvlink_intranode_conn *conn,
				u32 link_mode,
				bool *match)
{
	struct nvlink_device *ndev0 = conn->ndev0;
	struct nvlink_device *ndev1 = conn->ndev1;
	int ret = 0;
	bool is_mode = false;

	switch (link_mode) {
	case NVLINK_LINK_OFF:
		/* Check if both links are OFF */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_OFF) &&
			nvlink_check_link_mode(ndev1, NVLINK_LINK_OFF)) {
			*match = true;
			nvlink_dbg("Intranode connection is OFF");
			return ret;
		}

		/* Check if one of the links is OFF */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_OFF) ||
			nvlink_check_link_mode(ndev1, NVLINK_LINK_OFF)) {
			nvlink_err("Link is in bad state");
			*match = false;
			return -ENOLINK;
		}

		nvlink_dbg("Link not OFF yet.");
		*match = false;
		break;

	case NVLINK_LINK_SAFE:
		/* Check if both links and sublinks are already in SAFE mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_SAFE) &&
			nvlink_check_link_mode(ndev1, NVLINK_LINK_SAFE)) {
			is_mode = nvlink_check_tx_sublink_mode(ndev0,
						NVLINK_TX_SAFE) &&
					nvlink_check_tx_sublink_mode(ndev1,
						NVLINK_TX_SAFE) &&
					nvlink_check_rx_sublink_mode(ndev0,
						NVLINK_RX_SAFE) &&
					nvlink_check_rx_sublink_mode(ndev1,
						NVLINK_RX_SAFE);
			if (!is_mode) {
				nvlink_err("Sublinks in bad state");
				*match = false;
				return -ENOLINK;
			}
			*match = true;
			nvlink_dbg("Intranode connection in Safe mode");
			return ret;
		}

		/* Check if one of the links in SAFE mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_SAFE) ||
			nvlink_check_link_mode(ndev1, NVLINK_LINK_SAFE)) {
			nvlink_err("Link is in bad state");
			*match = false;
			return -ENOLINK;
		}

		nvlink_dbg("Link is not in Safe mode");
		*match = false;
		break;

	case NVLINK_LINK_HS:
		/* Check if both links and sublinks are in HS mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_HS) &&
			nvlink_check_link_mode(ndev1, NVLINK_LINK_HS)) {
			is_mode = nvlink_check_tx_sublink_mode(ndev0,
						NVLINK_TX_HS) &&
					nvlink_check_tx_sublink_mode(ndev1,
						NVLINK_TX_HS) &&
					nvlink_check_rx_sublink_mode(ndev0,
						NVLINK_RX_HS) &&
					nvlink_check_rx_sublink_mode(ndev1,
						NVLINK_RX_HS);
			if (!is_mode) {
				nvlink_err("Sublinks in bad state");
				*match = false;
				return -ENOLINK;
			}
			*match = true;
			nvlink_dbg("Intranode connection in HS mode");
			return ret;
		}

		/* Check if one of the links in HS mode */
		if (nvlink_check_link_mode(ndev0, NVLINK_LINK_HS) ||
			nvlink_check_link_mode(ndev1, NVLINK_LINK_HS)) {
			nvlink_err("Link is in bad state");
			*match = false;
			return -ENOLINK;
		}

		nvlink_dbg("Link is not in High Speed mode");
		*match = false;
		break;

	default:
		*match = false;
	}
	return ret;
}
/*
 * Get the intranode connection having ndev0 pointing to master device and
 * ndev1 to slave device.
 */
static int nvlink_get_intranode_conn(struct nvlink_device *ndev,
					struct nvlink_intranode_conn *conn)
{
	int ret = 0;

	if (!ndev || !conn) {
		nvlink_err("Invalid pointers passed as input");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);
	if ((ndev->device_id == nvlink_core.topology.master_dev_id) ||
		(ndev->device_id == nvlink_core.topology.slave_dev_id)) {
		*conn = nvlink_core.intranode_conn;
	} else {
		nvlink_err("Invalid Unregistered device ID");
		ret = -EINVAL;
	}
	mutex_unlock(&nvlink_core.mutex);

	return ret;
}

/*
 * This function should help transition link and sublink mode from high speed
 * to safe on both the endpoints. It will also disable low power management
 * before transitioning out of high speed.
 */
int nvlink_transition_intranode_conn_hs_to_safe(struct nvlink_device *ndev)
{
	int ret = 0;
	struct nvlink_intranode_conn conn;
	struct nvlink_device *ndev0 = NULL;
	struct nvlink_device *ndev1 = NULL;
	struct nvlink_link *link0 = NULL;
	struct nvlink_link *link1 = NULL;
	bool match = false;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	/*
	 * Get the intranode conn to co-ordinate link state transition between
	 * two endpoints.
	 */
	ret = nvlink_get_intranode_conn(ndev, &conn);
	if (ret < 0) {
		nvlink_err("Error retrieving intranode connection information");
		return ret;
	}
	ndev0 = conn.ndev0;
	ndev1 = conn.ndev1;
	link0 = &(ndev0->link);
	link1 = &(ndev1->link);

	/* Check if both the link and sublink state are SAFE for both ends */
	ret = nvlink_check_intranode_conn_mode(&conn, NVLINK_LINK_SAFE,
					&match);
	/* Return if the links are in bad state or already in SAFE mode */
	if (ret < 0) {
		nvlink_err("Can't transition to SAFE as link is in bad state");
		return ret;
	}
	if (match) {
		nvlink_dbg("link is already in SAFE mode");
		return ret;
	}

	if (nvlink_is_single_lane_mode_supported(&conn)) {
		/* Disable Single-Lane mode for device 0 */
		ret = link0->link_ops.set_link_mode(ndev0,
							NVLINK_LINK_DISABLE_PM);
		if (ret) {
			nvlink_err("Failed to disable SL(1/8th) mode for dev0");
			return ret;
		}

		/* Disable Single-Lane mode for device 1 */
		ret = link1->link_ops.set_link_mode(ndev1,
							NVLINK_LINK_DISABLE_PM);
		if (ret) {
			nvlink_err("Failed to disable SL(1/8th) mode for dev1");
			return ret;
		}
	}

	/* Move both ends to SWCFG */
	link0->link_ops.set_link_mode(ndev0, NVLINK_LINK_SAFE);
	link1->link_ops.set_link_mode(ndev1, NVLINK_LINK_SAFE);

	/* Wait for the end0 to go to SWCFG */
	ret = nvlink_poll_link_state(ndev0, NVLINK_LINK_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set link in swcfg");
		return ret;
	}

	/* Wait for the end1 to go to SWCFG */
	ret = nvlink_poll_link_state(ndev1, NVLINK_LINK_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set link in swcfg");
		return ret;
	}

	/* Put TX sublink on end0 in SAFE Mode */
	ret = link0->link_ops.set_sublink_mode(ndev0, false, NVLINK_TX_SAFE);
	if (ret < 0) {
		nvlink_err("Failed to set TX sublink mode to SAFE for ndev0");
		return ret;
	}

	/* Put TX sublink on end1 in SAFE Mode */
	ret = link1->link_ops.set_sublink_mode(ndev1, false, NVLINK_TX_SAFE);
	if (ret < 0) {
		nvlink_err("Failed to set TX sublink mode to SAFE for ndev1");
		return ret;
	}

	/* wait for sublinks to go in SAFE Mode */
	ret = nvlink_poll_sublink_state(ndev0, NVLINK_TX_SAFE,
					ndev1, NVLINK_RX_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in safe mode");
		return ret;
	}

	ret = nvlink_poll_sublink_state(ndev1, NVLINK_TX_SAFE,
					ndev0, NVLINK_RX_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in safe mode");
		return ret;
	}

	nvlink_dbg("Link in Safe mode!");
	return ret;
}
EXPORT_SYMBOL(nvlink_transition_intranode_conn_hs_to_safe);

/* After device is initialized, this function can be used to transition an
 * intranode connection from OFF to SAFE state. Note this function only changes
 * the state from off to safe on both endpoints; it does not perform any device
 * or link initialization.
 */
int nvlink_transition_intranode_conn_off_to_safe(struct nvlink_device *ndev)
{
	int ret = 0;
	enum init_state init_state_ndev0 = NVLINK_DEV_OFF;
	enum init_state init_state_ndev1 = NVLINK_DEV_OFF;
	struct nvlink_intranode_conn conn;
	struct nvlink_device *ndev0 = NULL;
	struct nvlink_device *ndev1 = NULL;
	struct nvlink_link *link0 = NULL;
	struct nvlink_link *link1 = NULL;
	bool match = false;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	/*
	 * Get the intranode conn to co-ordinate link state transition between
	 * two endpoints.
	 */
	ret = nvlink_get_intranode_conn(ndev, &conn);
	if (ret < 0) {
		nvlink_err("Error retrieving intranode connection information");
		return ret;
	}
	ndev0 = conn.ndev0;
	ndev1 = conn.ndev1;
	link0 = &(ndev0->link);
	link1 = &(ndev1->link);

	/* Verify that the hardware is initialized before transition to safe */
	ret = nvlink_get_init_state(ndev0, &init_state_ndev0);
	if (ret < 0) {
		nvlink_err("Error retrieving init state for ndev0");
		return ret;
	}
	ret = nvlink_get_init_state(ndev1, &init_state_ndev1);
	if (ret < 0) {
		nvlink_err("Error retrieving init state for ndev1");
		return ret;
	}
	if ((init_state_ndev0 != NVLINK_DEV_REG_INIT_DONE) ||
			(init_state_ndev1 != NVLINK_DEV_REG_INIT_DONE)) {
		nvlink_err("Error: hardware is uninitialized");
		return ret;
	}

	/* Check if link is already in SAFE mode */
	ret = nvlink_check_intranode_conn_mode(&conn,
						NVLINK_LINK_SAFE,
						&match);
	/* Return if the links are in bad state or already in SAFE mode */
	if (ret < 0) {
		nvlink_err("Can't transition to Safe as link is in bad state");
		return ret;
	}
	if (match) {
		nvlink_dbg(
			"Exiting Safe transition as link is already in Safe");
		return ret;
	}

	/* This function supports transition only from OFF mode */
	ret = nvlink_check_intranode_conn_mode(&conn,
						NVLINK_LINK_OFF,
						&match);
	/* Return if the links are in bad state or not in OFF mode */
	if (ret < 0) {
		nvlink_err("Can't transition to Safe as link is in bad state");
		return ret;
	}
	if (!match) {
		nvlink_dbg(
			"Exiting Safe transition as link is not in OFF mode");
		return ret;
	}

	/* Put the links in SAFE mode. */
	link0->link_ops.set_link_mode(ndev0, NVLINK_LINK_SAFE);
	link1->link_ops.set_link_mode(ndev1, NVLINK_LINK_SAFE);

	/* Wait for ndev0 to go in SWCFG mode */
	ret = nvlink_poll_link_state(ndev0,
					NVLINK_LINK_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set ndev0 end in SWCFG mode");
		return ret;
	}
	/* Wait for ndev1 to go in SWCFG mode */
	ret = nvlink_poll_link_state(ndev1,
					NVLINK_LINK_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set ndev1 end in SWCFG mode");
		return ret;
	}

	/* wait for sublinks to go in Safe Mode */
	ret = nvlink_poll_sublink_state(ndev0,
					NVLINK_TX_SAFE,
					ndev1,
					NVLINK_RX_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in Safe mode");
		return ret;
	}

	ret = nvlink_poll_sublink_state(ndev1,
					NVLINK_TX_SAFE,
					ndev0,
					NVLINK_RX_SAFE,
					NVLINK_TRANSITION_SAFE_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in Safe mode");
		return ret;
	}

	link0->is_connected = true;
	link1->is_connected = true;
	nvlink_dbg("Link in Safe mode!");
	return ret;
}
EXPORT_SYMBOL(nvlink_transition_intranode_conn_off_to_safe);

/*
 * This function trains the link from safe to high speed mode. It enables the
 * PRBS generator on both the endpoints before transitioning to high speed. Once
 * the link is in high speed mode, it  enables low power management over link.
 */
int nvlink_train_intranode_conn_safe_to_hs(struct nvlink_device *ndev)
{
	int ret = 0;
	struct nvlink_intranode_conn conn;
	struct nvlink_device *ndev0 = NULL;
	struct nvlink_device *ndev1 = NULL;
	struct nvlink_link *link0 = NULL;
	struct nvlink_link *link1 = NULL;
	bool match = false;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	/*
	 * Get the intranode conn to co-ordinate link state transition between
	 * two endpoints.
	 */
	ret = nvlink_get_intranode_conn(ndev, &conn);
	if (ret < 0) {
		nvlink_err("Error retrieving intranode connection information");
		return ret;
	}
	ndev0 = conn.ndev0;
	ndev1 = conn.ndev1;
	link0 = &(ndev0->link);
	link1 = &(ndev1->link);

	/* Check if both the link and sublink state are HS for both ends */
	ret = nvlink_check_intranode_conn_mode(&conn,
						NVLINK_LINK_HS,
						&match);
	/* Return if the links are in bad state or already in HS mode */
	if (ret < 0) {
		nvlink_err("Can't transition to HS as link is in bad state");
		return ret;
	}
	if (match) {
		nvlink_dbg("Exiting HS transition as link is already in HS");
		return ret;
	}

	/* We can train connection to HS only if the link is in Safe mode */
	ret = nvlink_check_intranode_conn_mode(&conn,
						NVLINK_LINK_SAFE,
						&match);
	/* Return if the links are in bad state or not in Safe mode */
	if (ret < 0) {
		nvlink_err("Can't transition to HS as link is in bad state");
		return ret;
	}
	if (!match) {
		nvlink_err("Exiting HS transition as link is not in SAFE mode");
		return ret;
	}

	/* Enable PRBS generator on both ends */
	ret = link0->link_ops.set_sublink_mode(ndev0,
					false,
					NVLINK_TX_PRBS_EN);
	if (ret < 0) {
		nvlink_err("Failed to enable PRBS generator for ndev0");
		return ret;
	}

	ret = link1->link_ops.set_sublink_mode(ndev1,
					false,
					NVLINK_TX_PRBS_EN);
	if (ret < 0) {
		nvlink_err("Failed to enable PRBS generator for ndev1");
		return ret;
	}

	/* Put TX sublink on end0 in High Speed */
	ret = link0->link_ops.set_sublink_mode(ndev0,
					false,
					NVLINK_TX_HS);
	if (ret < 0) {
		nvlink_err("Failed to set TX sublink mode to HS for ndev0");
		return ret;
	}

	/* Put TX sublink on end1 in High Speed */
	ret = link1->link_ops.set_sublink_mode(ndev1,
					false,
					NVLINK_TX_HS);
	if (ret < 0) {
		nvlink_err("Failed to set TX sublink mode to HS for ndev1");
		return ret;
	}

	/* wait for sublinks to go in High Speed */
	ret = nvlink_poll_sublink_state(ndev0,
					NVLINK_TX_HS,
					ndev1,
					NVLINK_RX_HS,
					NVLINK_TRANSITION_HS_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in high speed mode");
		return ret;
	}

	ret = nvlink_poll_sublink_state(ndev1,
					NVLINK_TX_HS,
					ndev0,
					NVLINK_RX_HS,
					NVLINK_TRANSITION_HS_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set sublinks in high speed mode");
		return ret;
	}

	/*
	 * Put only end0 in ACTIVE mode. The other end should automatically
	 * go to Active mode.
	 */
	link0->link_ops.set_link_mode(ndev0, NVLINK_LINK_HS);

	/* Wait for other end to go in ACTIVE mode */
	ret = nvlink_poll_link_state(ndev1,
					NVLINK_LINK_HS,
					NVLINK_TRANSITION_HS_TIMEOUT_MS);
	if (ret < 0) {
		nvlink_err("Unable to set links in high speed mode");
		return ret;
	}

	if (nvlink_is_single_lane_mode_supported(&conn)) {
		/* Enable Single-Lane policy for device 0 */
		ret = link0->link_ops.set_link_mode(ndev0,
							NVLINK_LINK_ENABLE_PM);
		if (ret) {
			nvlink_err("Error encountered while enabling "
					"Single-Lane mode policy for device 0");
			return ret;
		}

		/* Enable Single-Lane policy for device 1 */
		ret = link1->link_ops.set_link_mode(ndev1,
							NVLINK_LINK_ENABLE_PM);
		if (ret) {
			nvlink_err("Error encountered while enabling "
					"Single-Lane mode policy for device 1");
			return ret;
		}
	}
	nvlink_dbg("Link in High Speed mode!");
	return ret;
}
EXPORT_SYMBOL(nvlink_train_intranode_conn_safe_to_hs);

int nvlink_transition_intranode_conn_safe_to_off(struct nvlink_device *ndev)
{
	int ret = 0;
	struct nvlink_intranode_conn conn;
	struct nvlink_device *ndev0 = NULL;
	struct nvlink_device *ndev1 = NULL;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}

	ret = nvlink_get_intranode_conn(ndev, &conn);
	if (ret < 0) {
		nvlink_err("Error retrieving intranode connection information");
		return ret;
	}
	ndev0 = conn.ndev0;
	ndev1 = conn.ndev1;

	ndev0->link.link_ops.set_link_mode(ndev0,
					NVLINK_LINK_DISABLE_ERR_DETECT);
	ndev1->link.link_ops.set_link_mode(ndev1,
					NVLINK_LINK_DISABLE_ERR_DETECT);

	/*  Disable Lanes on both sides of the link */
	ret = ndev0->link.link_ops.set_link_mode(ndev0,
					NVLINK_LINK_LANE_DISABLE);
	if (ret < 0) {
		nvlink_err("ndev0 set NVLINK_LINK_LANE_DISABLE failed");
		goto fail;
	}
	ret = ndev1->link.link_ops.set_link_mode(ndev1,
					NVLINK_LINK_LANE_DISABLE);
	if (ret < 0) {
		nvlink_err("ndev1 set NVLINK_LINK_LANE_DISABLE failed");
		goto fail;
	}

	/* Shutdown Lanes on both sides of the link */
	ret = ndev0->link.link_ops.set_link_mode(ndev0,
					NVLINK_LINK_LANE_SHUTDOWN);
	if (ret < 0) {
		nvlink_err("ndev0 set NVLINK_LINK_LANE_SHUTDOWN failed");
		goto fail;
	}
	ret = ndev1->link.link_ops.set_link_mode(ndev1,
					NVLINK_LINK_LANE_SHUTDOWN);
	if (ret < 0) {
		nvlink_err("ndev1 set NVLINK_LINK_LANE_SHUTDOWN failed");
		goto fail;
	}

	ret = ndev0->link.link_ops.set_link_mode(ndev0, NVLINK_LINK_OFF);
	if (ret < 0) {
		nvlink_err("ndev0 set link mode to OFF failed");
		goto fail;
	}

	/* set_link_mode(NVLINK_LINK_OFF) disables CAR. Make sure we are not
	 * calling this twice for the same endpoint incase of loopback
	 * topologies.
	 */
	if (ndev0 != ndev1) {
		ret = ndev1->link.link_ops.set_link_mode(ndev1,
							NVLINK_LINK_OFF);
		if (ret < 0) {
			nvlink_err("ndev1 set link mode to OFF failed");
			goto fail;
		}
	}

	ret = nvlink_set_init_state(ndev0, NVLINK_DEV_OFF);
	if (ret < 0)
		goto fail;

	ret = nvlink_set_init_state(ndev1, NVLINK_DEV_OFF);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}
EXPORT_SYMBOL(nvlink_transition_intranode_conn_safe_to_off);

/*
 * Initialize the device using different callbacks registered through
 * dev_ops and link_ops. At the end of this function, the device should
 * have the clocks, resets, uphy, minion, interrupts and memory interface
 * initialized and the endpoint should be ready for link state transition
 */
int nvlink_initialize_endpoint(struct nvlink_device *ndev)
{
	int ret = 0;
	enum init_state init_state = NVLINK_DEV_OFF;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		ret = -EINVAL;
		goto fail;
	}

	ret = nvlink_get_init_state(ndev, &init_state);
	if (ret < 0)
		goto fail;

	switch (init_state) {
	case NVLINK_DEV_OFF:
		ret = ndev->dev_ops.dev_early_init(ndev);
		if (ret < 0)
			goto fail;
		ret = nvlink_set_init_state(ndev,
					NVLINK_DEV_EARLY_INIT_DONE);
		if (ret < 0)
			goto fail;

	case NVLINK_DEV_EARLY_INIT_DONE:
		ret = ndev->link.link_ops.link_early_init(ndev);
		if (ret < 0)
			goto fail;
		ret = nvlink_set_init_state(ndev,
					NVLINK_LINK_EARLY_INIT_DONE);
		if (ret < 0)
			goto fail;

	case NVLINK_LINK_EARLY_INIT_DONE:
		ret = ndev->dev_ops.dev_interface_init(ndev);
		if (ret < 0)
			goto fail;
		ret = nvlink_set_init_state(ndev,
					NVLINK_DEV_INTERFACE_INIT_DONE);
		if (ret < 0)
			goto fail;

	case NVLINK_DEV_INTERFACE_INIT_DONE:
		ret = ndev->dev_ops.dev_reg_init(ndev);
		if (ret < 0)
			goto fail;
		ret = nvlink_set_init_state(ndev,
					NVLINK_DEV_REG_INIT_DONE);
		if (ret < 0)
			goto fail;

	case NVLINK_DEV_REG_INIT_DONE:
		nvlink_dbg("Device %u is initialized!", ndev->device_id);
		break;

	default:
		ret = -EINVAL;
		nvlink_err("Invalid device state!");
		goto fail;
	}
	nvlink_dbg("Device initialization successful!");
	goto success;

fail:
	/*
	 * TODO: Add code to undo the HW and interface init state if device
	 * init fails. This code will follow the shutdown sequence.
	 */
	nvlink_err("Device initialization failed!");
success:
	return ret;
}
EXPORT_SYMBOL(nvlink_initialize_endpoint);

/*
 * Setup the link and endpoint devices for data transfer over high speed
 * Only master device can call nvlink_enumerate to start data transfer over
 * nvlink.
 */
int nvlink_enumerate(struct nvlink_device *ndev)
{
	int ret = 0;
	struct nvlink_device *master_dev = NULL;
	struct nvlink_device *slave_dev = NULL;
	struct topology *topology = NULL;

	if (!ndev) {
		nvlink_err("Invalid pointer to device struct");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);
	topology = &(nvlink_core.topology);

	if (ndev->device_id != topology->master_dev_id) {
		nvlink_err("Device is not master and cannot start enumeration");
		ret = -EINVAL;
		goto release_mutex;
	}

	master_dev = nvlink_core.ndevs[topology->master_dev_id];
	slave_dev = nvlink_core.ndevs[topology->slave_dev_id];

	if (!master_dev || !slave_dev) {
		nvlink_err("Slave or Master not registered with core driver");
		ret = -ENODATA;
		goto release_mutex;
	}

	mutex_unlock(&nvlink_core.mutex);

	/*
	 * Initialize the clocks, resets, minion, uphy, interrupts,
	 * memory interface on both the endpoints
	 */
	ret = nvlink_initialize_endpoint(slave_dev);
	if (ret < 0)
		goto fail;

	ret = nvlink_initialize_endpoint(master_dev);
	if (ret < 0)
		goto fail;

	ret = nvlink_transition_intranode_conn_off_to_safe(master_dev);
	if (ret < 0)
		goto fail;

	ret = nvlink_train_intranode_conn_safe_to_hs(master_dev);
	if (ret < 0)
		goto fail;

	nvlink_dbg("Nvlilnk enumerate successful!");
	goto success;
release_mutex:
	mutex_unlock(&nvlink_core.mutex);
fail:
	nvlink_err("Nvlink enumerate failed!");
success:
	return ret;
}
EXPORT_SYMBOL(nvlink_enumerate);

/*
 * Disable the device interface, transition the link to SAFE mode
 * and then to OFF. Only master device can able to initiate nvlink shutdown.
 */
int nvlink_shutdown(struct nvlink_device *ndev)
{
	int ret = 0;
	struct nvlink_device *master_dev = NULL;
	struct nvlink_device *slave_dev = NULL;
	struct topology *topology = NULL;
	enum init_state master_state = NVLINK_DEV_OFF;
	enum init_state slave_state = NVLINK_DEV_OFF;

	if (!ndev) {
		nvlink_err("Invalid pointer to device struct");
		return -EINVAL;
	}

	mutex_lock(&nvlink_core.mutex);
	topology = &(nvlink_core.topology);

	if (ndev->device_id != topology->master_dev_id) {
		nvlink_err("Device is not master and cannot start shutdown");
		ret = -EINVAL;
		goto release_mutex;
	}

	master_dev = nvlink_core.ndevs[topology->master_dev_id];
	slave_dev = nvlink_core.ndevs[topology->slave_dev_id];

	if (!master_dev || !slave_dev) {
		nvlink_err("Slave or Master not registered with core driver");
		ret = -ENODATA;
		goto release_mutex;
	}

	mutex_unlock(&nvlink_core.mutex);

	ret = nvlink_get_init_state(master_dev, &master_state);
	if (ret < 0) {
		nvlink_err("Error retrieving init state for master");
		goto fail;
	}
	ret = nvlink_get_init_state(slave_dev, &slave_state);
	if (ret < 0) {
		nvlink_err("Error retrieving init state for slave");
		goto fail;
	}

	if (master_state == NVLINK_DEV_OFF || slave_state == NVLINK_DEV_OFF) {
		nvlink_dbg("master/slave device is off, link already shutdown");
		return ret;
	}

	if (master_state != NVLINK_DEV_REG_INIT_DONE ||
		slave_state != NVLINK_DEV_REG_INIT_DONE) {
		nvlink_err("nvlink not initialized and is struck in"
			" intermediate state");
		ret = -EPERM;
		goto fail;
	}

	ret = master_dev->dev_ops.dev_interface_disable(master_dev);
	if (ret < 0) {
		nvlink_err("master_dev dev_interface_disable failed");
		goto fail;
	}

	ret = slave_dev->dev_ops.dev_interface_disable(slave_dev);
	if (ret < 0) {
		nvlink_err("slave_dev dev_interface_disable failed");
		goto fail;
	}

	ret = nvlink_transition_intranode_conn_hs_to_safe(master_dev);
	if (ret < 0) {
		nvlink_err("Transiting intranode conn to safe failed");
		goto fail;
	}

	ret = nvlink_transition_intranode_conn_safe_to_off(master_dev);
	if (ret < 0) {
		nvlink_err("Turning off nvlink lane failed");
		goto fail;
	}

	nvlink_dbg("Nvlink shutdown successful!");
	goto success;
release_mutex:
	mutex_unlock(&nvlink_core.mutex);
fail:
	nvlink_err("nvlink shutdown failed");
success:
	return ret;
}
EXPORT_SYMBOL(nvlink_shutdown);

#ifdef CONFIG_DEBUG_FS
void nvlink_core_debugfs_init(void)
{
	struct dentry *core_debugfs = NULL;
	struct dentry *debugfs_node = NULL;

	nvlink_debugfs_root = debugfs_create_dir(NVLINK_DEBUGFS_ROOT, NULL);
	if (!nvlink_debugfs_root) {
		nvlink_err("Failed to create NVLINK debugfs root directory");
		goto fail;
	}

	nvlink_debugfs_tests = debugfs_create_dir(NVLINK_DEBUGFS_TESTS,
						nvlink_debugfs_root);
	if (!nvlink_debugfs_tests) {
		nvlink_err("Failed to create NVLINK tests debugfs directory");
		goto fail;
	}

	core_debugfs = debugfs_create_dir(NVLINK_MODULE_NAME,
					nvlink_debugfs_root);
	if (!core_debugfs) {
		nvlink_err("Failed to create NVLINK core driver's debugfs directory");
		goto fail;
	}

	debugfs_node = debugfs_create_u32("log_mask",
					S_IWUSR | S_IRUGO,
					core_debugfs,
					&nvlink_log_mask);
	if (!debugfs_node) {
		nvlink_err("Failed to create the log_mask debugfs file");
		goto fail;
	}

	return;

fail:
	nvlink_err("Failed to create debugfs nodes");
	debugfs_remove_recursive(nvlink_debugfs_root);
	nvlink_debugfs_root = NULL;
	nvlink_debugfs_tests = NULL;
}

void nvlink_core_debugfs_deinit(void)
{
	debugfs_remove_recursive(nvlink_debugfs_root);
	nvlink_debugfs_root = NULL;
	nvlink_debugfs_tests = NULL;
}
#endif /* CONFIG_DEBUG_FS */

/*
 * nvlink_core_init:
 * The NVLINK core driver init function is called after debugfs has been
 * initialized but before the NVLINK endpoint drivers probe. This is the perfect
 * time for the NVLINK core driver to initialize any variables/state. At this
 * point during the kernel boot we should have access to debugfs, but we don't
 * have to worry about race conditions due to endpoint driver nvlink_register_*
 * calls.
 */
int __init nvlink_core_init(void)
{
	int i = 0;

	mutex_init(&nvlink_core.mutex);

	mutex_lock(&nvlink_core.mutex);

	for (i = 0; i < NVLINK_MAX_DEVICES; i++)
		nvlink_core.ndevs[i] = NULL;
	for (i = 0; i < NVLINK_MAX_LINKS; i++)
		nvlink_core.nlinks[i] = NULL;

	nvlink_core.topology.slave_dev_id = -1;
	nvlink_core.topology.master_dev_id = -1;
	nvlink_core.topology.slave_link_id = -1;
	nvlink_core.topology.master_link_id = -1;

#ifdef CONFIG_DEBUG_FS
	nvlink_core_debugfs_init();
#endif /* CONFIG_DEBUG_FS */

	mutex_unlock(&nvlink_core.mutex);
	return 0;
}
subsys_initcall(nvlink_core_init);

void __exit nvlink_core_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	nvlink_core_debugfs_deinit();
#endif /* CONFIG_DEBUG_FS */
	mutex_destroy(&nvlink_core.mutex);
}
module_exit(nvlink_core_exit);
