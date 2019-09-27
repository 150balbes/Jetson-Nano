/*
 * drivers/platform/tegra/ptp-notifier.c
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/notifier.h>
#include <linux/module.h>

static u64 (*get_systime)(void *);
static void *param;
static DEFINE_RAW_SPINLOCK(ptp_notifier_lock);
static ATOMIC_NOTIFIER_HEAD(tegra_hwtime_chain_head);

/* Clients register for notification of hwtime change events */
int tegra_register_hwtime_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&tegra_hwtime_chain_head, nb);
}
EXPORT_SYMBOL(tegra_register_hwtime_notifier);

/* Clients unregister for notification of hwtime change events */
int tegra_unregister_hwtime_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&tegra_hwtime_chain_head, nb);
}
EXPORT_SYMBOL(tegra_unregister_hwtime_notifier);

/* Trigger notification of hwtime change to all registered clients */
int tegra_hwtime_notifier_call_chain(unsigned int val, void *v)
{
	int ret = atomic_notifier_call_chain(&tegra_hwtime_chain_head, val, v);

	return notifier_to_errno(ret);
}

void tegra_register_hwtime_source(u64 (*func)(void *), void *data)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ptp_notifier_lock, flags);
	get_systime = func;
	param = data;
	raw_spin_unlock_irqrestore(&ptp_notifier_lock, flags);

	/* Notify HW time stamp update to registered clients.
	 * NULL callback parameter. We use a separate timestamp
	 * function to peek MAC time.
	 */
	tegra_hwtime_notifier_call_chain(0, NULL);
}
EXPORT_SYMBOL(tegra_register_hwtime_source);

void tegra_unregister_hwtime_source(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ptp_notifier_lock, flags);
	get_systime = NULL;
	param = NULL;
	raw_spin_unlock_irqrestore(&ptp_notifier_lock, flags);
}
EXPORT_SYMBOL(tegra_unregister_hwtime_source);

int get_ptp_hwtime(u64 *ns)
{
	unsigned long flags;
	int ret = 0;

	raw_spin_lock_irqsave(&ptp_notifier_lock, flags);
	if (get_systime)
		*ns = get_systime(param);
	else
		ret = -EINVAL;
	raw_spin_unlock_irqrestore(&ptp_notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL(get_ptp_hwtime);

MODULE_LICENSE("GPL");
