 /*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/bitops.h>
#include <linux/psci.h>

#include <asm/io.h>

#include "iomap.h"
#include <soc/tegra/pmc.h>

static void program_reboot_reason(const char *cmd)
{
	u32 reboot_reason;

	/* clean up */
	reboot_reason = (BOOTLOADER_MODE | RECOVERY_MODE | FORCED_RECOVERY_MODE
			| UPDATE_MODE);
	tegra_pmc_clear_reboot_reason(reboot_reason);

	/* valid command? */
	if (!cmd || (strlen(cmd) == 0))
		return;

	/* set reset_reboot_reason to 0, so that only 'cmd' can be set
	 * to register.
	 */
	reboot_reason = 0;
	/* Writing recovery kernel or Bootloader mode or update kernel in
	 * SCRATCH0 31:30:29:1. */
	if (!strcmp(cmd, "recovery"))
		reboot_reason |= RECOVERY_MODE;
	else if (!strcmp(cmd, "bootloader"))
		reboot_reason |= BOOTLOADER_MODE;
	else if (!strcmp(cmd, "forced-recovery"))
		reboot_reason |= FORCED_RECOVERY_MODE;
	else if (!strcmp(cmd, "update"))
		reboot_reason |= UPDATE_MODE;

	/* write the restart command */
	tegra_pmc_set_reboot_reason(reboot_reason);
}

static __init int tegra_register_reboot_handler(void)
{
	psci_handle_reboot_cmd = program_reboot_reason;
	pr_info("Tegra reboot handler registered.\n");
	return 0;
}
arch_initcall(tegra_register_reboot_handler);
