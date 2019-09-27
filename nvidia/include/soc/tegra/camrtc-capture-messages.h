/*
 * Copyright (c) 2016-2018, NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_CAPTURE_MESSAGES_H
#define INCLUDE_CAMRTC_CAPTURE_MESSAGES_H

#include "camrtc-capture.h"

#pragma GCC diagnostic error "-Wpadded"

/**
 * Standard message header for all capture IVC messages.
 *
 * Control Requests not associated with a specific channel
 * will use an opaque transaction ID rather than channel_id.
 * The transaction ID in the response message is copied from
 * the request message.
 *
 * @param msg_id	Message identifier.
 * @param channel_id	Channel identifier.
 * @param transaction	Transaction id.
 */
struct CAPTURE_MSG_HEADER {
	uint32_t msg_id;
	union {
		uint32_t channel_id;
		uint32_t transaction;
	};
} __CAPTURE_IVC_ALIGN;

/**
 * Message types for capture control channel messages.
 */
#define CAPTURE_CHANNEL_SETUP_REQ		U32_C(0x10)
#define CAPTURE_CHANNEL_SETUP_RESP		U32_C(0x11)
#define CAPTURE_CHANNEL_RESET_REQ		U32_C(0x12)
#define CAPTURE_CHANNEL_RESET_RESP		U32_C(0x13)
#define CAPTURE_CHANNEL_RELEASE_REQ		U32_C(0x14)
#define CAPTURE_CHANNEL_RELEASE_RESP		U32_C(0x15)
#define CAPTURE_COMPAND_CONFIG_REQ		U32_C(0x16)
#define CAPTURE_COMPAND_CONFIG_RESP		U32_C(0x17)
#define CAPTURE_PDAF_CONFIG_REQ			U32_C(0x18)
#define CAPTURE_PDAF_CONFIG_RESP		U32_C(0x19)
#define CAPTURE_SYNCGEN_ENABLE_REQ		U32_C(0x1A)
#define CAPTURE_SYNCGEN_ENABLE_RESP		U32_C(0x1B)
#define CAPTURE_SYNCGEN_DISABLE_REQ		U32_C(0x1C)
#define CAPTURE_SYNCGEN_DISABLE_RESP		U32_C(0x1D)

/**
 * Message types for capture channel messages.
 */
#define	CAPTURE_REQUEST_REQ			U32_C(0x01)
#define	CAPTURE_STATUS_IND			U32_C(0x02)
#define	CAPTURE_RESET_BARRIER_IND		U32_C(0x03)

/**
 * Invalid message type. This can be used to
 * respond to an invalid request.
 */
#define CAPTURE_MSG_ID_INVALID			U32_C(0xFFFFFFFF)

/**
 * Result codes.
 */
typedef uint32_t capture_result;

#define CAPTURE_OK				U32_C(0)
#define CAPTURE_ERROR_INVALID_PARAMETER		U32_C(1)
#define CAPTURE_ERROR_NO_MEMORY			U32_C(2)
#define CAPTURE_ERROR_BUSY			U32_C(3)
#define CAPTURE_ERROR_NOT_SUPPORTED		U32_C(4)
#define CAPTURE_ERROR_NOT_INITIALIZED		U32_C(5)
#define CAPTURE_ERROR_OVERFLOW			U32_C(6)
#define CAPTURE_ERROR_NO_RESOURCES		U32_C(7)
#define CAPTURE_ERROR_TIMEOUT			U32_C(8)


/** Set up RTCPU side resources for a capture pipe-line.
 *
 * The client shall use the transaction id field in the
 * standard message header to assocuiate request and response.
 *
 * @param channel_config	Capture channel configuration.
 */
struct CAPTURE_CHANNEL_SETUP_REQ_MSG {
	struct capture_channel_config	channel_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge capture channel setup request.
 *
 * The transaction id field in the standard message header
 * will be copied from the associated request.
 *
 * The setup response message returns a @a channel_id, which
 * identifies this set of resources and is used to refer to the
 * allocated capture channel in subsequent messages.
 *
 * @param result		Return value.
 * @param channel_id		Capture channel identifier for the new channel.
 * @param vi_channel_mask	Allocated VI channel(s).
 */
struct CAPTURE_CHANNEL_SETUP_RESP_MSG {
	capture_result result;
	uint32_t channel_id;
	uint64_t vi_channel_mask;
} __CAPTURE_IVC_ALIGN;

/** Reset a capture channel.
 *
 * Halt the associated VI channel. Flush the request queue for the
 * channel and increment syncpoints in the request queue to their target
 * values.
 *
 * @param reset_flags		Reset flags.
 */
struct CAPTURE_CHANNEL_RESET_REQ_MSG {
	uint32_t reset_flags;

/** Reset the channel without waiting for FE first. */
#define CAPTURE_CHANNEL_RESET_FLAG_IMMEDIATE	U32_C(0x01)

	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge a capture channel reset.
 *
 * The response is sent after the RTCPU side channel cleanup is
 * complete.
 *
 * @param result	Return value.
 */
struct CAPTURE_CHANNEL_RESET_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Reset a capture channel and release all the associated resources.
 *
 * Halt the associated VI channel. Flush the request queue for the
 * channel and increment syncpoints in the request queue to their target
 * values.
 *
 * @param reset_flags		Reset flags.
 */
struct CAPTURE_CHANNEL_RELEASE_REQ_MSG {
	/** See CAPTURE_CHANNEL_RESET_REQ_MSG for details. */
	uint32_t reset_flags;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge a capture channel release.
 *
 * The release is acknowledged after the channel cleanup is complete
 * and all resources have been freed on RTCPU.
 *
 * @param result	Return value.
 */
struct CAPTURE_CHANNEL_RELEASE_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Configure the piece-wise linear function used by the VI companding module.
 *
 * The companding table is shared by all capture channels and must be
 * configured before enabling companding for a specific capture.
 */
struct CAPTURE_COMPAND_CONFIG_REQ_MSG {
	struct vi_compand_config compand_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge a companding configuration.
 */
struct CAPTURE_COMPAND_CONFIG_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Configure the PDAF pattern.
 *
 * @param	pdaf_config	PDAF configuration data.
 */
struct CAPTURE_PDAF_CONFIG_REQ_MSG {
	struct vi_pdaf_config pdaf_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge CAPTURE_PDAF_CONFIG_REQ
 *
 * @param result		Return value.
 */
struct CAPTURE_PDAF_CONFIG_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Enable SLVS-EC synchronization
 *
 * Enable the generation of XVS and XHS synchronization signals for a
 * SLVS-EC sensor.
 */
struct CAPTURE_SYNCGEN_ENABLE_REQ_MSG {
	uint32_t unit;
	uint32_t __pad;
	struct vi_syncgen_config syncgen_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge CAPTURE_SYNCGEN_ENABLE_REQ. */
struct CAPTURE_SYNCGEN_ENABLE_RESP_MSG {
	uint32_t unit;
	capture_result result;
} __CAPTURE_IVC_ALIGN;

/** Disable SLVS-EC synchronization
 *
 * Disable the generation of XVS and XHS synchronization signals for a
 * SLVS-EC sensor.
 */
struct CAPTURE_SYNCGEN_DISABLE_REQ_MSG {
	uint32_t unit;
	uint32_t syncgen_disable_flags;

/* Disable SYNCGEN without waiting for frame end */
#define CAPTURE_SYNCGEN_DISABLE_FLAG_IMMEDIATE	U32_C(0x01)

} __CAPTURE_IVC_ALIGN;

/** Acknowledge CAPTURE_SYNCGEN_DISABLE_REQ. */
struct CAPTURE_SYNCGEN_DISABLE_RESP_MSG {
	uint32_t unit;
	capture_result result;
} __CAPTURE_IVC_ALIGN;

/**
 * Phy IVC messages
 */

/* Open an Phy stream */
struct CAPTURE_PHY_STREAM_OPEN_REQ_MSG {
	uint32_t stream_id;
	uint32_t csi_port;
	uint32_t phy_type;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_PHY_STREAM_OPEN_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Close an NvPhy stream */
struct CAPTURE_PHY_STREAM_CLOSE_REQ_MSG {
	uint32_t stream_id;
	uint32_t csi_port;
	uint32_t phy_type;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_PHY_STREAM_CLOSE_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Reset an NvPhy stream */
struct CAPTURE_PHY_STREAM_RESET_REQ_MSG {
	uint32_t stream_id;
	uint32_t csi_port;
	uint32_t phy_type;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_PHY_STREAM_RESET_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Debug: Dump Registers for an NvPhy stream */
struct CAPTURE_PHY_STREAM_DUMPREGS_REQ_MSG {
	uint32_t stream_id;
	uint32_t csi_port;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_PHY_STREAM_DUMPREGS_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/**
 * NVCSI IVC messages
 */

/* Set config for an NVCSI stream */
struct CAPTURE_CSI_STREAM_SET_CONFIG_REQ_MSG {
	uint32_t stream_id;
	uint32_t csi_port;
	uint32_t config_flags;
	uint32_t __pad32;
	struct nvcsi_brick_config brick_config;
	struct nvcsi_cil_config cil_config;
	struct nvcsi_error_config error_config;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_CSI_STREAM_SET_CONFIG_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Set DPCM config for an NVCSI stream */
struct CAPTURE_CSI_STREAM_SET_PARAM_REQ_MSG {
	uint32_t stream_id;
	uint32_t virtual_channel_id;
	uint32_t param_type;
	uint32_t __pad32;
	union {
		struct nvcsi_dpcm_config dpcm_config;
		struct nvcsi_dt_override_config dt_override_config;
		struct nvcsi_watchdog_config watchdog_config;
	};
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_CSI_STREAM_SET_PARAM_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Set TPG config for an NVCSI stream */
struct CAPTURE_CSI_STREAM_TPG_SET_CONFIG_REQ_MSG {
	union nvcsi_tpg_config tpg_config;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_CSI_STREAM_TPG_SET_CONFIG_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Start TPG for an NVCSI stream */
struct CAPTURE_CSI_STREAM_TPG_START_REQ_MSG {
	uint32_t stream_id;
	uint32_t virtual_channel_id;
	struct nvcsi_tpg_rate_config tpg_rate_config;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_CSI_STREAM_TPG_START_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Start TPG for an NVCSI stream */
struct CAPTURE_CSI_STREAM_TPG_START_RATE_REQ_MSG {
	uint32_t stream_id;
	uint32_t virtual_channel_id;
	uint32_t frame_rate;
	uint32_t csi_clk_rate;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_CSI_STREAM_TPG_START_RATE_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Stop TPG for an NVCSI stream */
struct CAPTURE_CSI_STREAM_TPG_STOP_REQ_MSG {
	uint32_t stream_id;
	uint32_t virtual_channel_id;
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_CSI_STREAM_TPG_STOP_RESP_MSG {
	uint32_t result;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* Setup test pattern generator */
/* DEPRECATED - to be removed */
struct CAPTURE_CHANNEL_TPG_SETUP_REQ_MSG {
	union nvcsi_tpg_config tpg_config;
} __CAPTURE_IVC_ALIGN;
/* DEPRECATED - to be removed */
struct CAPTURE_CHANNEL_TPG_SETUP_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/* Start test pattern generator */
/* DEPRECATED - to be removed */
struct CAPTURE_CHANNEL_TPG_START_REQ_MSG {
	uint8_t stream;
	uint8_t channel;
	uint16_t __pad16;
	uint32_t __pad;
	struct nvcsi_tpg_rate_config tpg_rate_config;
} __CAPTURE_IVC_ALIGN;
/* DEPRECATED - to be removed */
struct CAPTURE_CHANNEL_TPG_START_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/* Stop test pattern generator */
/* DEPRECATED - to be removed */
struct CAPTURE_CHANNEL_TPG_STOP_REQ_MSG {
	uint8_t stream;
	uint8_t channel;
	uint16_t __pad16;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;
/* DEPRECATED - to be removed */
struct CAPTURE_CHANNEL_TPG_STOP_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

#define VI_NUM_INJECT_EVENTS 10U /* Max number of events */

/* Event injection configuration. */
/* A capture request must be sent before this message */
struct CAPTURE_CHANNEL_EI_REQ_MSG {
	struct event_inject_msg events[VI_NUM_INJECT_EVENTS];
	uint8_t num_events;
	uint8_t __pad[7];
} __CAPTURE_IVC_ALIGN;

struct CAPTURE_CHANNEL_EI_RESP_MSG {
	capture_result result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/**
 * Capture ISP channel messages
 */

/**
 * Message types for isp capture control channel messages.
 */
#define CAPTURE_CHANNEL_ISP_SETUP_REQ		U32_C(0x20)
#define CAPTURE_CHANNEL_ISP_SETUP_RESP		U32_C(0x21)
#define CAPTURE_CHANNEL_ISP_RESET_REQ		U32_C(0x22)
#define CAPTURE_CHANNEL_ISP_RESET_RESP		U32_C(0x23)
#define CAPTURE_CHANNEL_ISP_RELEASE_REQ		U32_C(0x24)
#define CAPTURE_CHANNEL_ISP_RELEASE_RESP	U32_C(0x25)

/**
 * Message types for isp capture channel messages.
 */
#define CAPTURE_ISP_REQUEST_REQ			U32_C(0x04)
#define CAPTURE_ISP_STATUS_IND			U32_C(0x05)

#define CAPTURE_ISP_PROGRAM_REQUEST_REQ		U32_C(0x06)
#define CAPTURE_ISP_PROGRAM_STATUS_IND		U32_C(0x07)

#define CAPTURE_ISP_RESET_BARRIER_IND		U32_C(0x08)

#define CAPTURE_ISP_EX_STATUS_IND		U32_C(0x09)

/* Message types for test pattern generator */
/* DEPRECATED - to be removed */
#define CAPTURE_CHANNEL_TPG_SETUP_REQ   U32_C(0x30)
#define CAPTURE_CHANNEL_TPG_SETUP_RESP  U32_C(0x31)
#define CAPTURE_CHANNEL_TPG_START_REQ   U32_C(0x32)
#define CAPTURE_CHANNEL_TPG_START_RESP  U32_C(0x33)
#define CAPTURE_CHANNEL_TPG_STOP_REQ    U32_C(0x34)
#define CAPTURE_CHANNEL_TPG_STOP_RESP   U32_C(0x35)

/**`
 * Message types for NvPhy
 */
#define CAPTURE_PHY_STREAM_OPEN_REQ		U32_C(0x36)
#define CAPTURE_PHY_STREAM_OPEN_RESP		U32_C(0x37)
#define CAPTURE_PHY_STREAM_CLOSE_REQ		U32_C(0x38)
#define CAPTURE_PHY_STREAM_CLOSE_RESP		U32_C(0x39)
#define CAPTURE_PHY_STREAM_RESET_REQ		U32_C(0x3A)
#define CAPTURE_PHY_STREAM_RESET_RESP		U32_C(0x3B)
#define CAPTURE_PHY_STREAM_DUMPREGS_REQ		U32_C(0x3C)
#define CAPTURE_PHY_STREAM_DUMPREGS_RESP	U32_C(0x3D)

/**
 * Message types for NvCsi
 */
#define CAPTURE_CSI_STREAM_SET_CONFIG_REQ	U32_C(0x40)
#define CAPTURE_CSI_STREAM_SET_CONFIG_RESP	U32_C(0x41)
#define CAPTURE_CSI_STREAM_SET_PARAM_REQ	U32_C(0x42)
#define CAPTURE_CSI_STREAM_SET_PARAM_RESP	U32_C(0x43)
#define CAPTURE_CSI_STREAM_TPG_SET_CONFIG_REQ	U32_C(0x44)
#define CAPTURE_CSI_STREAM_TPG_SET_CONFIG_RESP	U32_C(0x45)
#define CAPTURE_CSI_STREAM_TPG_START_REQ	U32_C(0x46)
#define CAPTURE_CSI_STREAM_TPG_START_RESP	U32_C(0x47)
#define CAPTURE_CSI_STREAM_TPG_STOP_REQ		U32_C(0x48)
#define CAPTURE_CSI_STREAM_TPG_STOP_RESP	U32_C(0x49)
#define CAPTURE_CSI_STREAM_TPG_START_RATE_REQ	U32_C(0x4A)
#define CAPTURE_CSI_STREAM_TPG_START_RATE_RESP	U32_C(0x4B)

#define CAPTURE_CHANNEL_EI_REQ		U32_C(0x50)
#define CAPTURE_CHANNEL_EI_RESP		U32_C(0x51)

/** Set up RTCPU side resources for ISP capture pipe-line.
 *
 * The client shall use the transaction id field in the
 * standard message header to associate request and response.
 *
 * @param channel_config	Capture channel configuration.
 */
struct CAPTURE_CHANNEL_ISP_SETUP_REQ_MSG {
	struct capture_channel_isp_config	channel_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge isp capture channel setup request.
 *
 * The transaction id field in the standard message header
 * will be copied from the associated request.
 *
 * The setup response message returns a channel_id, which
 * identifies this set of resources and is used to refer to the
 * allocated capture channel in subsequent messages.
 *
 * @param result		Return value.
 * @param channel_id		Capture channel identifier for the new channel.
 */
struct CAPTURE_CHANNEL_ISP_SETUP_RESP_MSG {
	capture_result result;
	uint32_t channel_id;
} __CAPTURE_IVC_ALIGN;

typedef struct CAPTURE_CHANNEL_RESET_REQ_MSG
			CAPTURE_CHANNEL_ISP_RESET_REQ_MSG;
typedef struct CAPTURE_CHANNEL_RESET_RESP_MSG
			CAPTURE_CHANNEL_ISP_RESET_RESP_MSG;
typedef struct CAPTURE_CHANNEL_RELEASE_REQ_MSG
			CAPTURE_CHANNEL_ISP_RELEASE_REQ_MSG;
typedef struct CAPTURE_CHANNEL_RELEASE_RESP_MSG
			CAPTURE_CHANNEL_ISP_RELEASE_RESP_MSG;

/**
 * Message definition for capture control channel messages.
 */
struct CAPTURE_CONTROL_MSG {
	struct CAPTURE_MSG_HEADER header;
	union {
		struct CAPTURE_CHANNEL_SETUP_REQ_MSG channel_setup_req;
		struct CAPTURE_CHANNEL_SETUP_RESP_MSG channel_setup_resp;
		struct CAPTURE_CHANNEL_RESET_REQ_MSG channel_reset_req;
		struct CAPTURE_CHANNEL_RESET_RESP_MSG channel_reset_resp;
		struct CAPTURE_CHANNEL_RELEASE_REQ_MSG channel_release_req;
		struct CAPTURE_CHANNEL_RELEASE_RESP_MSG channel_release_resp;
		struct CAPTURE_COMPAND_CONFIG_REQ_MSG compand_config_req;
		struct CAPTURE_COMPAND_CONFIG_RESP_MSG compand_config_resp;
		struct CAPTURE_PDAF_CONFIG_REQ_MSG pdaf_config_req;
		struct CAPTURE_PDAF_CONFIG_RESP_MSG pdaf_config_resp;
		struct CAPTURE_SYNCGEN_ENABLE_REQ_MSG syncgen_enable_req;
		struct CAPTURE_SYNCGEN_ENABLE_RESP_MSG syncgen_enable_resp;
		struct CAPTURE_SYNCGEN_DISABLE_REQ_MSG syncgen_disable_req;
		struct CAPTURE_SYNCGEN_DISABLE_RESP_MSG syncgen_disable_resp;

		struct CAPTURE_PHY_STREAM_OPEN_REQ_MSG phy_stream_open_req;
		struct CAPTURE_PHY_STREAM_OPEN_RESP_MSG phy_stream_open_resp;
		struct CAPTURE_PHY_STREAM_CLOSE_REQ_MSG phy_stream_close_req;
		struct CAPTURE_PHY_STREAM_CLOSE_RESP_MSG phy_stream_close_resp;
		struct CAPTURE_PHY_STREAM_RESET_REQ_MSG phy_stream_reset_req;
		struct CAPTURE_PHY_STREAM_RESET_RESP_MSG phy_stream_reset_resp;
		struct CAPTURE_PHY_STREAM_DUMPREGS_REQ_MSG
			phy_stream_dumpregs_req;
		struct CAPTURE_PHY_STREAM_DUMPREGS_RESP_MSG
			phy_stream_dumpregs_resp;

		struct CAPTURE_CSI_STREAM_SET_CONFIG_REQ_MSG
			csi_stream_set_config_req;
		struct CAPTURE_CSI_STREAM_SET_CONFIG_RESP_MSG
			csi_stream_set_config_resp;
		struct CAPTURE_CSI_STREAM_SET_PARAM_REQ_MSG
			csi_stream_set_param_req;
		struct CAPTURE_CSI_STREAM_SET_PARAM_RESP_MSG
			csi_stream_set_param_resp;
		struct CAPTURE_CSI_STREAM_TPG_SET_CONFIG_REQ_MSG
			csi_stream_tpg_set_config_req;
		struct CAPTURE_CSI_STREAM_TPG_SET_CONFIG_RESP_MSG
			csi_stream_tpg_set_config_resp;
		struct CAPTURE_CSI_STREAM_TPG_START_REQ_MSG
			csi_stream_tpg_start_req;
		struct CAPTURE_CSI_STREAM_TPG_START_RESP_MSG
			csi_stream_tpg_start_resp;
		struct CAPTURE_CSI_STREAM_TPG_STOP_REQ_MSG
			csi_stream_tpg_stop_req;
		struct CAPTURE_CSI_STREAM_TPG_STOP_RESP_MSG
			csi_stream_tpg_stop_resp;
		struct CAPTURE_CSI_STREAM_TPG_START_RATE_REQ_MSG
			csi_stream_tpg_start_rate_req;
		struct CAPTURE_CSI_STREAM_TPG_START_RATE_RESP_MSG
			csi_stream_tpg_start_rate_resp;

		/* DEPRECATED - to be removed */
		struct CAPTURE_CHANNEL_TPG_SETUP_REQ_MSG tpg_setup_req;
		struct CAPTURE_CHANNEL_TPG_SETUP_RESP_MSG tpg_setup_resp;
		/* DEPRECATED - to be removed */
		struct CAPTURE_CHANNEL_TPG_START_REQ_MSG tpg_start_req;
		struct CAPTURE_CHANNEL_TPG_START_RESP_MSG tpg_start_resp;
		/* DEPRECATED - to be removed */
		struct CAPTURE_CHANNEL_TPG_STOP_REQ_MSG tpg_stop_req;
		struct CAPTURE_CHANNEL_TPG_STOP_RESP_MSG tpg_stop_resp;

		struct CAPTURE_CHANNEL_EI_REQ_MSG ei_req;
		struct CAPTURE_CHANNEL_EI_RESP_MSG ei_resp;

		struct CAPTURE_CHANNEL_ISP_SETUP_REQ_MSG channel_isp_setup_req;
		struct CAPTURE_CHANNEL_ISP_SETUP_RESP_MSG channel_isp_setup_resp;
		CAPTURE_CHANNEL_ISP_RESET_REQ_MSG channel_isp_reset_req;
		CAPTURE_CHANNEL_ISP_RESET_RESP_MSG channel_isp_reset_resp;
		CAPTURE_CHANNEL_ISP_RELEASE_REQ_MSG channel_isp_release_req;
		CAPTURE_CHANNEL_ISP_RELEASE_RESP_MSG channel_isp_release_resp;
	};
} __CAPTURE_IVC_ALIGN;

/**
 * Enqueue a new capture request on a capture channel.
 *
 * The request contains channel identifier and the capture sequence
 * number, which are required to schedule the capture request. The
 * actual capture programming is stored in the capture descriptor,
 * stored in a DRAM ring buffer set up with CAPTURE_CHANNEL_SETUP_REQ.
 *
 * The capture request descriptor with buffer_index=N can be located
 * within the ring buffer as follows:
 *
 * struct capture_descriptor *desc = requests + buffer_index * request_size;
 *
 * The capture request message is asynchronous. Capture completion is
 * indicated by incrementing the progress syncpoint a pre-calculated
 * number of times = 1 + <number of sub-frames>. The first increment
 * occurs at start-of-frame and the last increment occurs at
 * end-of-frame. The progress-syncpoint is used to synchronize with
 * down-stream engines. This model assumes that the capture client
 * knows the number of subframes used in the capture and has
 * programmed the VI accordingly.
 *
 * If the flag CAPTURE_FLAG_STATUS_REPORT_ENABLE is set in the capture
 * descriptor, RTCPU will store the capture status into status field
 * of the descriptor. RTCPU will also send a CAPTURE_STATUS_IND
 * message to indicate that capture has completed. The capture status
 * record contains information about the capture, such as CSI frame
 * number, start-of-frame and end-of-frame timestamps, as well as
 * error status.
 *
 * If the flag CAPTURE_FLAG_ERROR_REPORT_ENABLE is set, RTCPU will send a
 * CAPTURE_STATUS_IND upon an error, even if
 * CAPTURE_FLAG_STATUS_REPORT_ENABLE is not set.
 *
 * @param buffer_index	Buffer index identifying capture descriptor.
 */
struct CAPTURE_REQUEST_REQ_MSG {
	uint32_t buffer_index;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Capture status indication.
 *
 * The message is sent after the capture status record has been
 * written into the capture request descriptor.
 *
 * @param buffer_index	Buffer index identifying capture descriptor.
 */
struct CAPTURE_STATUS_IND_MSG {
	uint32_t buffer_index;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/**
 * Send new isp_capture request on a capture channel.
 *
 * The request contains channel identifier and the capture sequence
 * number (ring-buffer index), which are required to schedule the
 * isp capture request.
 * The actual capture programming is stored in isp_capture_descriptor,
 * stored in DRAM ring buffer, which includes the sequence, ISP
 * surfaces' details, surface related configs, ISP PB2 iova, input prefences,
 * and isp_capture status written by RTCPU.
 *
 * NvCapture UMD allocates the pool of isp_capture descriptors in setup call,
 * where each isp_capture_desc is followed by corresponding PB2 memory
 * (ATOM aligned).
 * RTCPU would generate the PB2 using surface details found in isp_capture
 * descriptor.
 * The ring-buffer (pool) would look like below:
 *
 * [isp_capture_desc][PB2][isp_capture_desc][PB2][isp_capture_desc]...
 *
 * The isp_capture_descriptor with buffer_index=N can be located within
 * the ring buffer as follows:
 *
 * isp_capture_descriptor *desc = requests + buffer_index * request_size;
 *
 * Note, here request_size = sizeof (isp_capture_descriptor) + sizeof (PB2).
 *
 * UMD fills isp_capture_desc and submits the request to KMD which pins the
 * surfaces and PB and then does the in-place replacement with iovas' within
 * isp_capture_descriptor.
 * KMD then sends the isp_capture request to RTCPU over capture ivc channel.
 *
 * The isp capture request message is asynchronous. Capture completion is
 * indicated by incrementing the progress syncpoint a pre-calculated
 * number of times = <number of sub-frames>. The progress-syncpoint is
 * used to synchronize with down-stream engines. This model assumes that
 * the capture client knows the number of subframes used in the capture and has
 * programmed the ISP accordingly.
 * All stats completion are indicated by incrementing stats progress syncpoint
 * a number of times = <num-stats-enabled>.
 *
 * If the flag CAPTURE_FLAG_ISP_STATUS_REPORT_ENABLE is set in the isp
 * capture descriptor, RTCPU will store the capture status into status field
 * of the descriptor. RTCPU will also send a CAPTURE_ISP_STATUS_IND
 * message to indicate that capture has completed.
 *
 * If the flag CAPTURE_FLAG_ISP_ERROR_REPORT_ENABLE is set, RTCPU will send a
 * CAPTURE_ISP_STATUS_IND upon an error, even if
 * CAPTURE_FLAG_ISP_STATUS_REPORT_ENABLE is not set.
 *
 * Typedef-ed CAPTURE_REQUEST_REQ_MSG.
 *
 * @param buffer_index: isp_capture_descriptor index in ring buffer.
 */
typedef struct CAPTURE_REQUEST_REQ_MSG CAPTURE_ISP_REQUEST_REQ_MSG;

/** ISP Capture status indication.
 *
 * The message is sent after the capture status record has been
 * written into the capture request descriptor.
 *
 * @param buffer_index	Buffer index identifying capture descriptor.
 */
typedef struct CAPTURE_STATUS_IND_MSG CAPTURE_ISP_STATUS_IND_MSG;

/** Extended ISP capture status indication.
 *
 * The message is sent after the capture status record has been
 * written into the capture request descriptor.
 *
 * @param process_buffer_index	Buffer index identifying ISP process descriptor.
 * @param program_buffer_index	Buffer index identifying ISP program descriptor.
 */
struct CAPTURE_ISP_EX_STATUS_IND_MSG {
	uint32_t process_buffer_index;
	uint32_t program_buffer_index;
} __CAPTURE_IVC_ALIGN;

/**
 * Send new isp_program request on a capture ivc channel.
 *
 * The request contains channel identifier and the program sequence
 * number (ring-buffer index).
 * The actual programming details is stored in isp_program
 * descriptor, which includes the offset to isp_program
 * buffer (which has PB1 containing ISP HW settings), sequence,
 * settings-id, activation-flags, isp_program buffer size, iova's
 * of ISP PB1 and isp_program status written by RTCPU.
 *
 * NvCapture UMD allocates the pool of isp_program descriptors in setup call,
 * where each isp_pgram_descriptor is followed by corresponding isp_program
 * buffer (ATOM aligned).
 * The ring-buffer (pool) would look like below:
 *
 * [isp_prog_desc][isp_program][isp_prog_desc][isp_program][isp_prog_desc]...
 *
 * The isp_program_descriptor with buffer_index=N can be located within
 * the ring buffer as follows:
 *
 * isp_program_descriptor *desc = programs + buffer_index * program_size;
 *
 * Note, program_size = sizeof (isp_program_descriptor) + sizeof (isp_program).
 *
 * NvISP fills these and submits the isp_program request to KMD which pins the
 * PB and then does the in-place replacement with iova within
 * isp_program_descriptor.
 * KMD then sends the isp_program request to RTCPU over capture ivc channel.
 *
 * The sequence is the frame_id which tells RTCPU, that the given isp_program
 * must be used from that frame_id onwards until UMD provides new one.
 * So RTCPU will use the sequence field to select the correct isp_program from
 * the isp_program descriptors' ring buffer for given frame request and will
 * keep on using it for further frames until the new isp_program (desc) is
 * provided to be used.
 * RTCPU populates both matched isp_program (reads from isp program desc) and
 * isp capture descriptor and forms single task descriptor for given frame
 * request and feeds it to falcon, which further programs it to ISP.
 *
 * settings_id is unique id for isp_program, NvCapture and RTCPU will use
 * the ring buffer array index as settings_id.
 * It can also be used to select the correct isp_program for the given
 * frame, in that case, UMD writes this unique settings_id to sensor's
 * scratch register, and sensor will send back it as part of embedded data,
 * when the given settings/gains are applied on that particular frame
 * coming from sensor.
 *
 * RTCPU reads this settings_id back from embedded data and uses it to select
 * the corresponding isp_program from the isp_program desc ring buffer.
 * The activation_flags tells the RTCPU which id (sequence or settings_id) to
 * use to select correct isp_program for the given frame.
 *
 * As same isp_program can be used for multiple frames, it can not be freed
 * when the frame capture is done. RTCPU will send a separate status
 * indication CAPTURE_ISP_PROGRAM_STATUS_IND message to CCPEX to notify
 * that the given isp_program is no longer in use and can be freed or reused.
 * settings_id (ring-buffer index) field is used to uniquely identify the
 * correct isp_program.
 * RTCPU also writes the isp_program status in isp program descriptor.
 *
 * Typedef-ed CAPTURE_REQUEST_REQ_MSG.
 *
 * @param buffer_index: isp_program descriptor index in ring buffer.
 */
typedef struct CAPTURE_REQUEST_REQ_MSG CAPTURE_ISP_PROGRAM_REQUEST_REQ_MSG;

/** ISP program status indication.
 *
 * The message is sent to notify CCPLEX about the isp_program which is expired
 * so UMD client can free or reuse it.
 *
 * Typedef-ed CAPTURE_STATUS_IND_MSG.
 *
 * @param buffer_index: Buffer index identifying ISP program descriptor.
 */
typedef struct CAPTURE_STATUS_IND_MSG CAPTURE_ISP_PROGRAM_STATUS_IND_MSG;

/**
 * Message definition for capture channel messages.
 */
struct CAPTURE_MSG {
	struct CAPTURE_MSG_HEADER header;
	union {
		struct CAPTURE_REQUEST_REQ_MSG capture_request_req;
		struct CAPTURE_STATUS_IND_MSG capture_status_ind;

		CAPTURE_ISP_REQUEST_REQ_MSG capture_isp_request_req;
		CAPTURE_ISP_STATUS_IND_MSG capture_isp_status_ind;
		struct CAPTURE_ISP_EX_STATUS_IND_MSG capture_isp_ex_status_ind;

		CAPTURE_ISP_PROGRAM_REQUEST_REQ_MSG
				capture_isp_program_request_req;
		CAPTURE_ISP_PROGRAM_STATUS_IND_MSG
				capture_isp_program_status_ind;
	};
} __CAPTURE_IVC_ALIGN;

#pragma GCC diagnostic ignored "-Wpadded"

#endif /* INCLUDE_CAMRTC_CAPTURE_MESSAGES_H */
