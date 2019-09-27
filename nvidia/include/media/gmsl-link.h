/**
 * Copyright (c) 2018-2019, NVIDIA Corporation.  All rights reserved.
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

#ifndef __GMSL_LINK_H__
#define __GMSL_LINK_H__

#define GMSL_CSI_1X4_MODE 0x1
#define GMSL_CSI_2X4_MODE 0x2
#define GMSL_CSI_2X2_MODE 0x3
#define GMSL_CSI_4X2_MODE 0x4

#define GMSL_CSI_PORT_A 0x0
#define GMSL_CSI_PORT_B 0x1
#define GMSL_CSI_PORT_C 0x2
#define GMSL_CSI_PORT_D 0x3
#define GMSL_CSI_PORT_E 0x4
#define GMSL_CSI_PORT_F 0x5

#define GMSL_SERDES_CSI_LINK_A 0x1
#define GMSL_SERDES_CSI_LINK_B 0x2

/* Didn't find kernel defintions, for now adding here */
#define GMSL_CSI_DT_RAW_12 0x2C
#define GMSL_CSI_DT_UED_U1 0x30
#define GMSL_CSI_DT_EMBED 0x12

#define GMSL_ST_ID_UNUSED 0xFF

#define GMSL_DEV_MAX_NUM_DATA_STREAMS 4

struct gmsl_stream {
	__u32 st_id_sel;
	__u32 st_data_type;
	__u32 des_pipe;
};

struct gmsl_link_ctx {
	__u32 st_vc;
	__u32 dst_vc;
	__u32 src_csi_port;
	__u32 dst_csi_port;
	__u32 serdes_csi_link;
	__u32 num_streams;
	__u32 num_csi_lanes;
	__u32 csi_mode;
	__u32 ser_reg;
	__u32 sdev_reg;
	__u32 sdev_def;
	struct gmsl_stream streams[GMSL_DEV_MAX_NUM_DATA_STREAMS];
	struct device *s_dev;
};

#endif  /* __GMSL_LINK_H__ */
