/*
 * Broadcom Event  protocol definitions
 *
 * Copyright (C) 1999-2015, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 * Broadcom Event  protocol definitions
 *
 * Dependencies: proto/bcmeth.h
 *
 * $Id: dnglevent.h $
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * -----------------------------------------------------------------------------
 *
 */

/*
 * Broadcom dngl Ethernet Events protocol defines
 *
 */

#ifndef _DNGLEVENT_H_
#define _DNGLEVENT_H_

#ifndef _TYPEDEFS_H_
#include <typedefs.h>
#endif
#include <proto/bcmeth.h>
#include <proto/ethernet.h>
#include <dngl_defs.h>

/* This marks the start of a packed structure section. */
#include <packed_section_start.h>
#define BCM_DNGL_EVENT_MSG_VERSION		1
#define DNGL_E_RSRVD_1				0x0
#define DNGL_E_RSRVD_2				0x1
#define DNGL_E_SOCRAM_IND			0x2
typedef BWL_PRE_PACKED_STRUCT struct
{
	uint16  version; /* Current version is 1 */
	uint16  reserved; /* reserved for any future extension */
	uint16  event_type; /* DNGL_E_SOCRAM_IND */
	uint16  datalen; /* Length of the event payload */
} BWL_POST_PACKED_STRUCT bcm_dngl_event_msg_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_event {
	struct ether_header eth;
	bcmeth_hdr_t        bcm_hdr;
	bcm_dngl_event_msg_t      dngl_event;
	/* data portion follows */
} BWL_POST_PACKED_STRUCT bcm_dngl_event_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_socramind {
	uint16			tag;	/* data tag */
	uint16			length; /* data length */
	uint8			value[1]; /* data value with variable length specified by length */
} BWL_POST_PACKED_STRUCT bcm_dngl_socramind_t;

/* SOCRAM_IND type tags */
#define SOCRAM_IND_ASSERT_TAG		0x1
/* Health check top level module tags */
typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_healthcheck {
	uint16			top_module_tag;	/* top level module tag */
	uint16			top_module_len; /* Type of PCIE issue indication */
	uint8			value[1]; /* data value with variable length specified by length */
} BWL_POST_PACKED_STRUCT bcm_dngl_healthcheck_t;

#define HC_PCIEDEV_CONFIG_REGLIST_MAX	20
typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_pcie_hc {
	uint16			reserved;
	uint16			pcie_err_ind_type; /* PCIE Module TAGs */
	uint16			pcie_flag;
	uint32			pcie_control_reg;
	uint32			pcie_config_regs[HC_PCIEDEV_CONFIG_REGLIST_MAX];
} BWL_POST_PACKED_STRUCT bcm_dngl_pcie_hc_t;

/* This marks the end of a packed structure section. */
#include <packed_section_end.h>

#endif /* _DNGLEVENT_H_ */
