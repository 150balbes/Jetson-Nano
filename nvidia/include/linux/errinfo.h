/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef __TEGRA_HV_ERRINFO_H__
#define __TEGRA_HV_ERRINFO_H__

/* Supported synchronous and asynchronous errors */
enum err_reason {
	REASON_UNDEFINED = 0UL,
	REASON_ASYNC_SMMU_CB,
	REASON_ASYNC_SMMU_GLOBAL,
	REASON_ASYNC_BRIDGE,
	REASON_ASYNC_MC,
	REASON_ASYNC_MC_T19X,
	REASON_ASYNC_CBB,
	REASON_SYNC,
	REASON_ENUM_SIZE
};

enum err_type {
	SYNC = 0UL,
	ASYNC
};

struct __attribute__((__packed__)) async_metadata_t {
	uint64_t	rd_idx;
	uint64_t	wr_idx;
};

#define NAME_SIZE 64

struct __attribute__((__packed__)) async_bridge_err_t {
	char		br_name[NAME_SIZE];
	unsigned int	err_addr;
	unsigned int	err_status1;
	unsigned int	err_status2;
	unsigned int	rw;
	unsigned int	err_type;
	unsigned int	length;
	unsigned int	br_id;
	unsigned int	src_id;
	unsigned int	axi_id;
	unsigned int	count;
	unsigned int	protection;
	unsigned int	burst;
	unsigned int	cache;
};

struct __attribute__((__packed__)) async_cbb_err_t {
	char		cbb_name[NAME_SIZE];
	unsigned int	error_logger;
	unsigned int	errlog0;
	char		transaction_type[NAME_SIZE];
	char		error_code[NAME_SIZE];
	char		error_source[NAME_SIZE];
	char		error_description[NAME_SIZE];
	bool		header_format;
	unsigned int	packet_header_lock;
	unsigned int	packet_header_len1;
	unsigned int	errlog1;
	unsigned int	errlog2;
	uint64_t	route_id;
	char		initflow[NAME_SIZE];
	char		targflow[NAME_SIZE];
	unsigned int	targ_subrange;
	unsigned int	seqid;
	unsigned int	errlog3;
	unsigned int	errlog4;
	uint64_t	address;
	unsigned int	errlog5;
	char		master_id[NAME_SIZE];
	unsigned int	non_mod;
	unsigned int	axi_id;
	unsigned int	security_group;
	uint32_t	cache;
	uint32_t	protection;
	unsigned int	falconsec;
	unsigned int	virtual_q_channel;
};

struct __attribute__((__packed__)) async_smmu_err_t {
	unsigned int	stream_id;
	unsigned int	cb_id;
	unsigned int	fsynr0;
	unsigned int	fsynr1;
	uint64_t	far;
	unsigned int	fsr;
};

struct __attribute__((__packed__)) async_mc_err_t {
	uint64_t	ch_base;
	unsigned int	int_status;
	unsigned int	err_status;
	uint64_t	fault_addr;
	unsigned int	vcpuid;		/* 0xffffU IDLE_vCPU_ID */
	unsigned int	client_id;
	int32_t		peripheral_id;
};

struct __attribute__((__packed__)) async_mc_err_t19x_t {
	bool		vpr_violation;
	unsigned int	vpr_base[2];
	unsigned int	vpr_size;
	unsigned int	vpr_ctrl;
	unsigned int	vpr_override[4];
	bool		no_status;
	bool		two_status;
	int		client_swgid;
	char		client_name[NAME_SIZE];
	char		fault_msg[NAME_SIZE];
	unsigned int	status;
	uint64_t	address;
	unsigned int	secure;
	unsigned int	write;
};

struct __attribute__((__packed__)) sync_t {
	bool		is_filled;    /* metadata field per vcpu */
	unsigned int	offending_vcpu_id;
	uint32_t	esr_el2;
	uint64_t	elr_el2;
	uint64_t	far_el2;
	uint64_t	hpfar_el2;
	uint64_t	par_hpfar_el2;
	uint64_t	spsr_el2;
	uint64_t	elr_el1;
	uint64_t	far_el1;
	uint64_t	spsr_el1;
	uint32_t	esr_el1;
	uint32_t	fault_instr;
	uint64_t	gpr_array[31];
};

struct __attribute__((__packed__)) err_data_t {
	unsigned int	offending_guest_id;
	unsigned int	err_type;
	unsigned int	err_reason;
	union {
		/* Asynchronous */
		struct async_bridge_err_t	async_bridge_err;
		struct async_smmu_err_t		async_smmu_err;
		struct async_mc_err_t		async_mc_err;
		struct async_cbb_err_t		async_cbb_err;
		struct async_mc_err_t19x_t	async_mc_err_t19x;
		/* Synchronous */
		struct sync_t			sync;
	};
};

/* VM shared memory for error information is allocated contiguously to store
 * Asynchronous(async) error information followed by the Synchronous(sync)
 * error information. HV has write access and the VM has read access to this
 * shared memory. The shared memory layout looks like:
 *
 * |--async-err-metadata--|--async-errors-array-|--sync-errors-array-|
 *
 * Size of async errors array = Max errors + 1(to avoid same empty and full
 * conditions of the buffer)
 * Size of sync errors array = 1 error per VCPU * number of VCPUs on a VM
 *
 * So for a give VM, shared memory has:
 *
 * |--------ASyncErrInfo--------------|---------SyncErrInfo-------------------|
 * |--------1bufferPerVM--------------|---VCpu0-buffer----|---VCpuN-buffer----|
 * |---metadata---|---err_data--------|-metadata+err_data-|-metadata+err_data-|
 * |rd_idx|wr_idx|-Err1-|-Err2-|-ErrN-|-is_filled-|-Err1--|-is_filled-|--Err1-|
 */

struct __attribute__((__packed__)) err_info_t {
	struct async_metadata_t	async_metadata;
	struct err_data_t	err_data[];
};

#endif
