/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_CHANNELS_H
#define INCLUDE_CAMRTC_CHANNELS_H

#include "camrtc-common.h"

/*
 * All the enums and the fields inside the structs described in this header
 * file supports only uintX_t types, where X can be 8,16,32,64.
 */
#define CAMRTC_TAG64(s0, s1, s2, s3, s4, s5, s6, s7) ( \
	((uint64_t)(s0) << 0U) | ((uint64_t)(s1) << 8U) | \
	((uint64_t)(s2) << 16U) | ((uint64_t)(s3) << 24U) | \
	((uint64_t)(s4) << 32U) | ((uint64_t)(s5) << 40U) | \
	((uint64_t)(s6) << 48U) | ((uint64_t)(s7) << 56U))

#define CAMRTC_TAG_IVC_SETUP	CAMRTC_TAG64('I', 'V', 'C', '-', 'S', 'E', 'T', 'U')
#define CAMRTC_TAG_NV_TRACE	CAMRTC_TAG64('N', 'V', ' ', 'T', 'R', 'A', 'C', 'E')
#define CAMRTC_TAG_NV_CAM_TRACE	CAMRTC_TAG64('N', 'V', ' ', 'C', 'A', 'M', 'T', 'R')
#define CAMRTC_TAG_NV_COVERAGE	CAMRTC_TAG64('N', 'V', ' ', 'C', 'O', 'V', 'E', 'R')

struct camrtc_tlv {
	uint64_t tag;
	uint64_t len;
};

/* Multiple setup structures can follow each other. */
struct camrtc_tlv_ivc_setup {
	uint64_t tag;
	uint64_t len;
	uint64_t rx_iova;
	uint32_t rx_frame_size;
	uint32_t rx_nframes;
	uint64_t tx_iova;
	uint32_t tx_frame_size;
	uint32_t tx_nframes;
	uint32_t channel_group;
	uint32_t ivc_version;
	char ivc_service[32];
};

enum {
	/* 0 .. 127 indicate unknown commands */
	RTCPU_CH_ERR_NO_SERVICE = U32_C(128),
	RTCPU_CH_ERR_ALREADY = U32_C(129),
	RTCPU_CH_ERR_UNKNOWN_TAG = U32_C(130),
	RTCPU_CH_ERR_INVALID_IOVA = U32_C(131),
	RTCPU_CH_ERR_INVALID_PARAM = U32_C(132),
};

#ifdef _BullseyeCoverage
struct camrtc_coverage_memory_header {
	uint64_t signature;
	uint64_t length;
	uint32_t revision;
	uint32_t coverage_buffer_size;
	uint32_t coverage_total_bytes;
	uint32_t reserved; /* alignment */
} __packed;
#endif /* _BullseyeCoverage */

#endif /* INCLUDE_CAMRTC_CHANNELS_H */
