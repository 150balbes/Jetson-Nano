/*
 * PVA mailbox header
 *
 * Copyright (c) 2016-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef __PVA_MAILBOX_H__
#define __PVA_MAILBOX_H__

#include <linux/platform_device.h>

#include "pva-interface.h"

/* Total CCQ status registers */
#define PVA_CCQ_STATUS_REGS	9

/* Symbolic definitions of the CCQ status registers */
#define PVA_CCQ_STATUS0_INDEX	0
#define PVA_CCQ_STATUS1_INDEX	1
#define PVA_CCQ_STATUS2_INDEX	2
#define PVA_CCQ_STATUS3_INDEX	3
#define PVA_CCQ_STATUS4_INDEX	4
#define PVA_CCQ_STATUS5_INDEX	5
#define PVA_CCQ_STATUS6_INDEX	6
#define PVA_CCQ_STATUS7_INDEX	7
#define PVA_CCQ_STATUS8_INDEX	8

/* Number of valid MBOX registers used for sending commands */
#define VALID_MB_INPUT_REGS 4
/* Number of valid MBOX registers */
#define VALID_MB_INPUT_REGS_EX 8
struct pva;

/**
 * enum pva_mailbox_status - PVA mailbox status indication
 *
 * These enumerations reflect the state of PVA interrupt handler
 */
enum pva_mailbox_status {
	PVA_MBOX_STATUS_INVALID	= 0,
	PVA_MBOX_STATUS_WFI	= 1,
	PVA_MBOX_STATUS_DONE	= 2,
	PVA_MBOX_STATUS_ABORTED	= 3,
};

/**
 * struct pva_mailbox_status_regs - Handle the MBOX status based on ISR
 *
 * @cmd:		Holds the current MBOX command
 * @error:		Holds the any error shown through ISR
 * @status:		Holds the status of all CCQ registers
 *
 */
struct pva_mailbox_status_regs {
	uint32_t status[PVA_CCQ_STATUS_REGS];
	uint32_t error;
	uint32_t cmd;
};

/**
 * pva_mailbox_send_cmd_sync() - Send a command and wait for response
 *
 * @pva:		Pointer to PVA structure
 * @pva_cmd:		Pointer to the pva command struct
 * @nregs:		Number of valid mailbox registers for the command
 * @mb_status_regs:	Pointer to pva_mailbox_status_regs struct
 *
 * Return:	0 on Success or negative error code
 *
 * This function called by OS to pass the mailbox commands to
 * the PVA uCode. The function returns the output status from PVA
 * firmware once the task is completed.
 *
 * The caller is responsible to ensure that PVA has been powered
 * up through nvhost_module_busy() API prior calling this function.
 */
int pva_mailbox_send_cmd_sync(struct pva *pva,
			struct pva_cmd *cmd, u32 nregs,
			struct pva_mailbox_status_regs *mb_status_regs);

/**
 * pva_mailbox_send_cmd_sync_locked() - Send a command and wait for response
 *
 * @pva:		Pointer to PVA structure
 * @pva_cmd:		Pointer to the pva command struct
 * @nregs:		Number of valid mailbox registers for the command
 * @mb_status_regs:	Pointer to pva_mailbox_status_regs struct
 *
 * Return:	0 on Success or negative error code
 *
 * This function called by OS to pass the mailbox commands to
 * the PVA uCode. The function returns the output status from PVA
 * firmware once the task is completed. This function must not be
 * used during runtime without holding the mailbox mutex (i.e.
 * the function can be called during PVA boot-up).
 */
int pva_mailbox_send_cmd_sync_locked(struct pva *pva,
			struct pva_cmd *cmd, u32 nregs,
			struct pva_mailbox_status_regs *mailbox_status_regs);

/**
 * pva_mailbox_isr() - Handle interrupt for PVA ISR
 *
 * @pva:	Pointer to PVA structure
 *
 * This function is used to read the CCQ status registers based on
 * the status set in mailbox7 by the PVA uCode.
 */
void pva_mailbox_isr(struct pva *pva);

/**
 * pva_mailbox_wait_event() - mailbox wait event
 *
 * @pva:»       Pointer to PVA structure
 * @wait_time»       WaitTime Interval for the event
 *
 * This function do the wait until the mailbox isr get invoked based on
 * the mailbox register set by the ucode.
 */
int pva_mailbox_wait_event(struct pva *pva, int wait_time);

/**
 * pva_read_mailbox() - read a mailbox register
 *
 * @pva:			Pointer to PVA structure
 * @mbox:		mailbox register to be written
 *
 * This function will read the indicated mailbox register and return its
 * contents.  it uses side channel B as host would.
 *
 * Return Value:
 *	contents of the indicated mailbox register
 */
u32 pva_read_mailbox(struct platform_device *pdev, u32 mbox_id);

/**
 * pva_write_mailbox() - write to a mailbox register
 *
 * @pva:			Pointer to PVA structure
 * @mbox:		mailbox register to be written
 * @value:		value to be written into the mailbox register
 *
 * This function will write a value into the indicated mailbox register.
 *
 * Return Value:
 *	none
 */
void pva_write_mailbox(struct platform_device *pdev, u32 mbox_id, u32 value);

#endif /*__PVA_MAINBOX_H__*/
