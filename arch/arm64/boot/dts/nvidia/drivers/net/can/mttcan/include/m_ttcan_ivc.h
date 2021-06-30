/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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
#ifndef _M_TTCAN_IVC_H
#define  _M_TTCAN_IVC_H

/*Size of data in an element */
enum m_ttcan_ivc_msgid {
	MTTCAN_MSG_TX = 1,
	MTTCAN_MSG_RX = 2,
	MTTCAN_MSG_TX_COMPL = 3,
	MTTCAN_MSG_STAT_CHG = 4,
	MTTCAN_MSG_BERR_CHG = 5,
	MTTCAN_MSG_RX_LOST_FRAME = 6,
	MTTCAN_MSG_TXEVT = 7,
	MTTCAN_CMD_CAN_ENABLE = 8,
	MTTCAN_MSG_LAST
};

#endif
