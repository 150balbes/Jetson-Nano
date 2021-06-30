/*
 *
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
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
#ifndef _PVA_REGS_H_
#define _PVA_REGS_H_

#include "pva-bit.h"
#include "hw_cfg_pva.h"
#include "hw_dma_ch_pva.h"
#include "hw_dma_desc_pva.h"
#include "hw_proc_pva.h"
#include "hw_hsp_pva.h"
#include "hw_sec_pva.h"
#include "hw_evp_pva.h"
#include "pva-interface.h"
#include "pva_mailbox.h"
#include "pva-ucode-header.h"

/* Definition for LIC_INTR_ENABLE bits */
#define SEC_LIC_INTR_HSP1	0x1
#define SEC_LIC_INTR_HSP2	0x2
#define SEC_LIC_INTR_HSP3	0x4
#define SEC_LIC_INTR_HSP4	0x8
#define SEC_LIC_INTR_HSP_ALL	0xF

/* Watchdog support */
#define SEC_LIC_INTR_WDT	0x1

#endif
