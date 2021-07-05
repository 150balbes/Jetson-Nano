/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _LINUX_TEGRA_GTE_ENGINE_H
#define _LINUX_TEGRA_GTE_ENGINE_H

#include <linux/device.h>

struct tegra_gte_ev_desc {
	int id;
	int gid;
	u32 ev_bit;
	u32 slice;
};

/* GTE hardware timestamping event details */
struct tegra_gte_ev_detail {
	u64 ts_raw; /* raw counter value */
	u64 ts_ns; /* counter value converted into nano seconds */
	int dir; /* direction of the event */
};

#ifdef CONFIG_TEGRA_HTS_GTE
/*
 * GTE event registration function
 *
 * Parameters:
 *
 * Input:
 * @np:		device node of the interested GTE device
 * @ev_id:	event id
 *
 * Returns:
 *		Returns ERR_PTR in case of failure or valid
 *		struct tegra_gte_ev_desc for success.
 *
 * Note:	API is not stable and subject to change.
 */
struct tegra_gte_ev_desc *tegra_gte_register_event(struct device_node *np,
						   u32 ev_id);

/*
 * GTE event un-registration function
 *
 * Parameters:
 *
 * Input:
 * @desc:	This parameter should be the same as returned from register
 *
 * Returns:
 *		Returns 0 for success and any other value for the failure
 *
  * Note:	API is not stable and subject to change.
 */
int tegra_gte_unregister_event(struct tegra_gte_ev_desc *desc);

/*
 * GTE event retrieval function
 *
 * Parameters:
 *
 * Input:
 * @desc:	This parameter should be the same as returned from register
 *
 * Output:
 * @hts:	hts event details
 *
 * Returns:
 *		Returns 0 for success and any other value for the failure
 *
 * Note:	API is not stable and subject to change.
 */
int tegra_gte_retrieve_event(const struct tegra_gte_ev_desc *desc,
			     struct tegra_gte_ev_detail *hts);

#else /* ! CONFIG_TEGRA_HTS_GTE */
static inline struct tegra_gte_ev_desc *tegra_gte_register_event(
					struct device_node *np, u32 ev_id)
{
	return ERR_PTR(-ENOSYS);
}

static inline int tegra_gte_unregister_event(struct tegra_gte_ev_desc *desc)
{
	return -ENOSYS;
}

static inline int tegra_gte_retrieve_event(const struct tegra_gte_ev_desc *desc,
					   struct tegra_gte_ev_detail *hts)
{
	return -ENOSYS;
}

#endif /* ! CONFIG_TEGRA_HTS_GTE */
#endif
