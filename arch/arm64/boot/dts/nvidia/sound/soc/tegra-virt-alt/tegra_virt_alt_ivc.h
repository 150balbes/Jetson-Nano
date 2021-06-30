/*
 * Copyright (c) 2015-2019 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA_VIRT_ALT_IVC_H__
#define __TEGRA_VIRT_ALT_IVC_H__

#include "tegra_virt_alt_ivc_common.h"

#define NVAUDIO_IVC_WAIT_TIMEOUT	1000000
struct nvaudio_ivc_dev;

struct nvaudio_ivc_ctxt {
	struct tegra_hv_ivc_cookie	*ivck;
	struct device			*dev;
	int				ivc_queue;
	wait_queue_head_t		wait;
	int				timeout;
	enum rx_state_t			rx_state;
	struct nvaudio_ivc_dev		*ivcdev;
	spinlock_t			ivck_rx_lock;
	spinlock_t			ivck_tx_lock;
	spinlock_t			lock;
};

void nvaudio_ivc_rx(struct tegra_hv_ivc_cookie *ivck);

struct nvaudio_ivc_ctxt *nvaudio_ivc_alloc_ctxt(struct device *dev);

void nvaudio_ivc_free_ctxt(struct device *dev);

int nvaudio_ivc_send(struct nvaudio_ivc_ctxt *ictxt,
				struct nvaudio_ivc_msg *msg,
				int size);

int nvaudio_ivc_send_retry(struct nvaudio_ivc_ctxt *ictxt,
				struct nvaudio_ivc_msg *msg,
				int size);

int nvaudio_ivc_send_receive(struct nvaudio_ivc_ctxt *ictxt,
				struct nvaudio_ivc_msg *msg,
				int size);

int tegra124_virt_xbar_set_ivc(struct nvaudio_ivc_ctxt *ictxt,
					int rx_idx,
					int tx_idx);
int tegra124_virt_xbar_get_ivc(struct nvaudio_ivc_ctxt *ictxt,
					int rx_idx,
					int *tx_idx);


#endif
