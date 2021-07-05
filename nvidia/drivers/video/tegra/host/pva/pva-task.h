/*
 * Copyright (c) 2016-2019 NVIDIA Corporation.  All rights reserved.
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

#ifndef _PVA_TASK_H_
#define _PVA_TASK_H_

#define TASK_VERSION_ID		0x01
#define PVA_TASK_VERSION_ID	0x01
#define PVA_ENGINE_ID		'P'

#define PVA_MAX_PREACTION_LISTS		1U
#define PVA_MAX_POSTACTION_LISTS	1U

/*
 * These are not the in DRAM representations of the action list elements,
 * they are repackaged to allow for them to be managed more efficiently.
 */
struct pva_task_action_ptr {
	u64	p;
	u32	v;
} __packed;

struct pva_task_action_gos {
	u8	gos;
	u16	ofs;
	u32	v;
} __packed;

struct pva_task_action_status {
	u64	p;
	u16	status;
} __packed;

/*
 * Generic task meta-data for the CV pipeline.
 */

struct pva_gen_task {
	u64	next;		/* ptr to next task in the list */
	u8	versionid;
	u8	engineid;
	u16	length;
	u16	sequence;
	u8	n_preaction_lists;
	u8	n_postaction_lists;
	u16	preaction_lists_p;
	u16	postaction_lists_p;
} __packed;

/*
 * Structure pointed to by {pre/post}action_lists_p.  This points
 * to the actual action list.
 */
struct pva_action_list {
	u16	offset;
	u16	length;
} __packed;

enum pva_task_action_ids_e {
	TASK_ACT_TERMINATE		= 0x00U,
	TASK_ACT_NEXT			= 0x01U,
	TASK_ACT_RESTART		= 0x02U,
	TASK_ACT_ERROR			= 0x03U,
	TASK_ACT_STOP			= 0x10U,
	TASK_ACT_DEV_SPEC_1ST		= 0x40U,
	TASK_ACT_PVA_STATISTICS		= 0x40U,
	TASK_ACT_PVA_VPU_PERF_COUNTERS	= 0x50U,
	TASK_ACT_DEV_SPEC_LAST		= 0x7FU,
	TASK_ACT_PTR_WRITE_VAL		= 0x80U,
	TASK_ACT_PTR_WRITE_VAL_SFRAME	= 0x81U,
	TASK_ACT_PTR_INCR		= 0x82U,
	TASK_ACT_PTR_WRITE_VAL_TS	= 0x83U,
	TASK_ACT_PTR_WRITE_VAL_SFRAME_TS = 0x84U,
	TASK_ACT_PTR_INCR_TS		= 0x85U,
	TASK_ACT_PTR_BLK_EQUAL		= 0x90U,
	TASK_ACT_PTR_BLK_EQUAL_SFRAME	= 0x91U,
	TASK_ACT_PTR_BLK_GTREQL		= 0x92U,
	TASK_ACT_PTR_BLK_GTREQL_SFRAME	= 0x93U,
	TASK_ACT_GOS_WRITE_VAL		= 0xA0U,
	TASK_ACT_GOS_WRITE_VAL_SFRAME	= 0xA1U,
	TASK_ACT_GOS_INCR		= 0xA2U,
	TASK_ACT_GOS_BLK_EQUAL		= 0xB0U,
	TASK_ACT_GOS_BLK_EQUAL_SFRAME	= 0xB1U,
	TASK_ACT_GOS_BLK_GTREQL		= 0xB2U,
	TASK_ACT_GOS_BLK_GTREQL_SFRAME	= 0xB3U,
	TASK_ACT_READ_STATUS		= 0xC0U,
	TASK_ACT_WRITE_STATUS		= 0xC1U,
	TASK_ACT_PTR_WRITE_SOT_V_TS	= 0xC2U,
	TASK_ACT_PTR_WRITE_SOT_R_TS	= 0xC3U,
	TASK_ACT_PTR_WRITE_EOT_V_TS	= 0xC4U,
	TASK_ACT_PTR_WRITE_EOT_R_TS	= 0xC5U,
	TASK_ACT_PTR_WRITE_TS		= 0xC6U,
	TASK_ACT_PTR_WRITE_VAL_SOT_V	= 0xC7U,
	TASK_ACT_PTR_WRITE_VAL_SOT_R	= 0xC8U,
	TASK_ACT_PTR_WRITE_VAL_EOT_V	= 0xC9U,
	TASK_ACT_PTR_WRITE_VAL_EOT_R	= 0xCAU
};

struct pva_gen_task_status {
	u64	timestamp;
	u32	engine_status;
	u16	subframe;
	u16	status_task;
};

struct pva_task_statistics {
	u64	queued_time;	/* when task was accepted by PVA */
	u64	head_time;	/* when task reached head of queue */
	u64	input_actions_complete;	/* when input actions done */
	u64	vpu_assigned_time;	/* when task assigned a VPU */
	u64	vpu_start_time;	/* when VPU started running task */
	u64	vpu_complete_time;	/* when execution completed */
	u64	complete_time;	/* when task considered complete */
	u8	vpu_assigned;	/* which VPU task was assigned */
	u8	reserved[7];
};

/*
 * PVA specific task structure.
 */
#define PVA_TASK_FL_ATOMIC		(1 << 0)
#define PVA_TASK_FL_CV_RD_SCALARS	(1 << 1)
#define PVA_TASK_FL_CV_WR_SCALARS	(1 << 2)
#define PVA_TASK_FL_CV_WR_ROI		(1 << 3)
#define PVA_TASK_FL_VPU_DEBUG		(1 << 4)

struct pva_task {
	struct pva_gen_task	gen_task;
	u8			runlist_version;
	u8			queue_id;
	u8			num_input_parameters;
	u8			num_output_parameters;
	u8			operation_version;
	u8			reserved;
	u16			input_parameters;
	u16			output_parameters;
	u16			flags;
	u32			operation;
	u64			timeout;
	u8			r5_reserved[32];
}__packed;

enum pva_task_parameter_type_e {
	PVA_PARAM_FIRST		= 0U,		/* must match first type */
	PVA_PARAM_SCALAR_LIST	= 0U,
	PVA_PARAM_SURFACE_LIST	= 1U,
	PVA_PARAM_ROI_LIST	= 2U,
	PVA_PARAM_2DPOINTS_LIST	= 3U,
	PVA_PARAM_OPAQUE_DATA	= 4U,
	PVA_PARAM_LAST		= 5U,		/* must be last! */
};

#define PVA_TASK_POINTER_AUX_SIZE_MASK		0x00ffffffffffffffULL
#define PVA_TASK_POINTER_AUX_SIZE_SHIFT		0
#define PVA_TASK_POINTER_AUX_FLAGS_MASK		0xff00000000000000ULL
#define PVA_TASK_POINTER_AUX_FLAGS_SHIFT	56
#define PVA_TASK_POINTER_AUX_FLAGS_CVNAS	(1 << 0)

struct pva_task_pointer {
	u64	address;
	u64	aux;
} __packed;

struct pva_task_opaque_data_desc {
	u16	primary_payload_size;
} __packed;

struct pva_task_parameter_array {
	u64	address;
	u32	size;
	u32	type;
};

/*
 * Parameter descriptor (all parameters have the same header)
 * the specific data for the parameters immediately follows
 * the descriptor.
 */
struct pva_task_parameter_desc {
	u32		num_parameters;
	u32		reserved;
};

/*
 * Individual Region of Interest (ROI) descriptor
 */
struct pva_task_roi_desc {
	u32		left;
	u32		top;
	u32		right;
	u32		bottom;
};

/*
 * Surface descriptor
 */
struct pva_task_surface {
	u64		address;
	u64		roi_addr;
	u32		roi_size;
	u32		surface_size;
	u32		width;
	u32		height;
	u32		line_stride;
	u32		plane_stride;
	u32		num_planes;
	u8		layout;
	u8		block_height_log2;
	u8		memory;
	u8		reserved;
	u64		format;
};

/*
 * 2-dimentional point descriptor
 */
struct pva_task_point2d {
	u32		x;
	u32		y;
};

/*
 * Surface Layout.
 */
#define PVA_TASK_SURFACE_LAYOUT_PITCH_LINEAR	0U
#define PVA_TASK_SURFACE_LAYOUT_BLOCK_LINEAR	1U

/*
 * Where the surface is located.
 */
#define PVA_TASK_SURFACE_MEM_FL_CV_SURFACE	(1 << 0)
#define PVA_TASK_SURFACE_MEM_FL_CV_ROI		(1 << 1)

#endif
