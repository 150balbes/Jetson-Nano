/*
 * PVA abort handler
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#include <linux/wait.h>

#include "nvhost_acm.h"
#include "dev.h"
#include "pva.h"

static void pva_abort_handler(struct work_struct *work)
{
	struct pva *pva = container_of(work, struct pva,
				       pva_abort_handler_work);
	struct platform_device *pdev = pva->pdev;

	/* Dump nvhost state to show the pending jobs */
	nvhost_debug_dump_device(pdev);

	/* First, lock mailbox mutex to avoid synchronous communication. */
	do {
		if (pva->mailbox_status == PVA_MBOX_STATUS_WFI) {
			pva->mailbox_status = PVA_MBOX_STATUS_ABORTED;
			wake_up(&pva->mailbox_waitqueue);
			schedule();
		}
	} while (mutex_trylock(&pva->mailbox_mutex) == false);

	/* There is no ongoing activity anymore. Update mailbox status */
	pva->mailbox_status = PVA_MBOX_STATUS_INVALID;

	/* Lock CCQ mutex to avoid asynchornous communication */
	mutex_lock(&pva->ccq_mutex);

	/*
	 * If boot was still on-going, skip over recovery and let boot-up
	 * routine handle the failure
	 */
	if (!pva->booted) {
		nvhost_warn(&pdev->dev, "Recovery skipped: PVA is not booted");
		goto skip_recovery;
	}

	/*
	 * If we use channel submit mode, nvhost handles the channel
	 * clean-up and syncpoint increments
	 */
	if (pva->submit_mode == PVA_SUBMIT_MODE_CHANNEL_CCQ) {
		nvhost_warn(&pdev->dev, "Recovery skipped: Submit mode does not require clean-up");
		goto skip_recovery;
	}

	/* Reset the PVA and reload firmware */
	nvhost_module_reset(pdev, true);

	/* Remove pending tasks from the queue */
	nvhost_queue_abort_all(pva->pool);

	nvhost_warn(&pdev->dev, "Recovery finished");

skip_recovery:
	mutex_unlock(&pva->ccq_mutex);
	mutex_unlock(&pva->mailbox_mutex);
}

void pva_abort(struct pva *pva)
{
	struct platform_device *pdev = pva->pdev;

	/* For selftest mode to finish the test */
	if (host1x_readl(pdev, hsp_ss0_state_r())
		& PVA_TEST_MODE) {
		pva->mailbox_status = PVA_MBOX_STATUS_DONE;
		wake_up(&pva->mailbox_waitqueue);
		return;
	}

	WARN(true, "Attempting to recover the engine");
	schedule_work(&pva->pva_abort_handler_work);
}

void pva_abort_init(struct pva *pva)
{
	INIT_WORK(&pva->pva_abort_handler_work, pva_abort_handler);
}
