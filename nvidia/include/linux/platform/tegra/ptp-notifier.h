/*
 * include/linux/platform/tegra/ptp-notifier.h
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __PTP_NOTIFIER_H
#define __PTP_NOTIFIER_H

#if IS_ENABLED(CONFIG_TEGRA_PTP_NOTIFIER)
/* register / unregister HW time source */
void tegra_register_hwtime_source(u64 (*func)(void *), void *data);
void tegra_unregister_hwtime_source(void);

/* clients registering / unregistering for time update events */
int tegra_register_hwtime_notifier(struct notifier_block *nb);
int tegra_unregister_hwtime_notifier(struct notifier_block *nb);

/* Notify time updates to registered clients */
int tegra_hwtime_notifier_call_chain(unsigned int val, void *v);

/*
 * Get HW time counter.
 * Clients may call the API every anytime PTP time is needed.
 * If HW time source is not registered, returns -EINVAL
 */
int get_ptp_hwtime(u64 *ns);

#else /* CONFIG_TEGRA_PTP_NOTIFIER */

/* register / unregister HW time source */
static inline void tegra_register_hwtime_source(u64 (*func)(void *),
						void *data)
{
}

static inline void tegra_unregister_hwtime_source(void)
{
}

/* clients registering / unregistering for time update events */
static inline int tegra_register_hwtime_notifier(struct notifier_block *nb)
{
	return 0;
}

static inline int tegra_unregister_hwtime_notifier(struct notifier_block *nb)
{
	return -ENOENT;
}

/* Notify time updates to registered clients */
static inline int tegra_hwtime_notifier_call_chain(unsigned int val, void *v)
{
	return notifier_to_errno(NOTIFY_DONE);
}

/*
 * Get HW time counter.
 * Clients may call the API every anytime PTP time is needed.
 * If HW time source is not registered, returns -EINVAL
 */
static inline int get_ptp_hwtime(u64 *ns)
{
	return -EINVAL;
}

#endif /* CONFIG_TEGRA_PTP_NOTIFIER */

#endif /* __PTP_NOTIFIER_H */
