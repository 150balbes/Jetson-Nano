/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef DMCE_PERFMON_H
#define DMCE_PERFMON_H

/**
 * Commands used in Command field of an ARI perfmon request.
 */
enum DMCE_PERFMON_COMMAND {
	DMCE_PERFMON_COMMAND_READ	= 0,   /* Read uncore perfmon reg */
	DMCE_PERFMON_COMMAND_WRITE	= 1,   /* Write uncore perfmon reg */
	DMCE_PERFMON_COMMAND_MAX
};

/**
 * Registers used in Register field of an ARI perfmon request.
 */
enum DMCE_PERFMON_REGISTER {
	NV_PMEVCNTR	= 0,
	NV_PMEVTYPER	= 1,
	DMCE_PERFMON_FIRST_UNIT_REGISTER	= 2,
	NV_PMCNTENSET	= 2,
	NV_PMCNTENCLR	= 3,
	NV_PMOVSSET	= 4,
	NV_PMOVSCLR	= 5,
	NV_PMCR		= 6,
	NV_PMINTENSET	= 7,
	NV_PMINTENCLR	= 8,
	DMCE_PERFMON_FIRST_GROUP_REGISTER	= 9,
	NV_PMCRNUNITS	= 9,
	NV_PMCEID0	= 10,
	NV_PMCEID1	= 11,
	DMCE_PERFMON_FIRST_GLOBAL_REGISTER	= 12,
	NV_AFR0		= 12,
	NV_SECURE	= 13,
	DMCE_PERFMON_REGISTER_MAX
};

/**
 * Status codes returned in Status field of an ARI perfmon response.
 */
enum DMCE_PERFMON_STATUS {
	DMCE_PERFMON_STATUS_SUCCESS		= 0,
	DMCE_PERFMON_STATUS_INVALID_GROUP	= 1,
	DMCE_PERFMON_STATUS_INVALID_UNIT	= 2,
	DMCE_PERFMON_STATUS_INVALID_COUNTER	= 3,
	DMCE_PERFMON_STATUS_INVALID_REGISTER	= 4,
	DMCE_PERFMON_STATUS_INVALID_COMMAND	= 5,
	DMCE_PERFMON_STATUS_READ_ONLY		= 6,
	DMCE_PERFMON_STATUS_NOT_SECURE		= 7,
	DMCE_PERFMON_STATUS_MAX
};

/**
 * Format of the value in ARI_REQUEST_DATA_HI
 * when making an uncore perfmon call.
 */
union dmce_perfmon_ari_request_hi_t {
	uint32_t flat;
	struct {
		uint8_t command:8;	/* Operation: 0 - read, 1 - write */
		uint8_t group:4;	/* Group selector */
		uint8_t unit:4;		/* Unit selector */
		uint8_t reg:8;		/* Register to read or write */
		uint8_t counter:8;	/* CNTR num for EVCNTR and EVTYPER */
	} bits;
};

/**
 * Format of the value returned in ARI_RESPONSE_DATA_HI
 * returned by an uncore perfmon call.
 */
union dmce_perfmon_ari_response_hi_t {
	uint32_t flat;
	struct {
		uint8_t status:8;	/* Resulting command statue */
		uint32_t unused:24;
	} bits;
};

/**
 * Layout of the uncore perfmon NV_PMEVTYPER register.
 */
union dmce_perfmon_pmevtyper_t {
	uint32_t flat;
	struct {
		uint32_t evt_count:10;	/* Event number to count */
		uint32_t reserved_15_10:6;
		uint32_t int_core:4;	/* Core to handle interrupt */
		uint32_t reserved_31_20:12;
	} bits;
};

/**
 * Layout of the NV_PMCR register.
 */
union dmce_perfmon_pmcr_t {
	uint32_t flat;
	struct {
		uint32_t e:1;	/* Enable counters */
		uint32_t p:1;	/* Reset counters (WO bit) */
		uint32_t reserved_10_2:9;
		uint32_t n:5;	/* Number of counters (RO bit)*/
		uint32_t idcode:8;	/* Identification code (0) */
		uint32_t imp:8;	/* Implementor code ('N') */
	} bits;
};

/**
 * Data for each uncore perfmon counter
 */
struct dmce_perfmon_cnt_info {
	uint8_t counter;	/* Event id */
	uint8_t group;		/* Group selector */
	uint8_t unit;		/* Unit selector */
	uint8_t index;		/* Virtual Index */
	uint8_t idx;		/* Physical Index */
	uint8_t valid;		/* Valid info */
};
#endif
