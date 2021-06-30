/*
 * PVA Command Queue Interface handling
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


#include <linux/kernel.h>
#include <linux/nvhost.h>
#include <linux/delay.h>

#include <soc/tegra/chip-id.h>

#include "dev.h"
#include "pva.h"
#include "pva_ccq.h"

#include "hw_cfg_pva.h"

#define MAX_CCQ_ELEMENTS	6

static int pva_ccq_wait(struct pva *pva, int timeout)
{
	unsigned long end_jiffies = jiffies + msecs_to_jiffies(timeout);

	/*
	 * Wait until there is free room in the CCQ. Otherwise the writes
	 * could stall the CPU. Ignore the timeout in simulation.
	 */

	while (time_before(jiffies, end_jiffies) ||
	       (pva->timeout_enabled == false)) {
		u32 val = host1x_readl(pva->pdev, cfg_ccq_status2_r());

		if (val <= MAX_CCQ_ELEMENTS)
			return 0;

		usleep_range(5, 10);
	}

	return -ETIMEDOUT;
}

int pva_ccq_send(struct pva *pva, u64 cmd)
{
	int err = 0;

	mutex_lock(&pva->ccq_mutex);

	err = pva_ccq_wait(pva, 100);
	if (err < 0)
		goto err_wait_ccq;

	/* Make the writes to CCQ */
	host1x_writel(pva->pdev, cfg_ccq_r(), (u32)(cmd >> 32));
	host1x_writel(pva->pdev, cfg_ccq_r(), (u32)(cmd & 0xffffffff));

	mutex_unlock(&pva->ccq_mutex);

	return err;

err_wait_ccq:
	mutex_unlock(&pva->ccq_mutex);
	pva_abort(pva);

	return err;
}
