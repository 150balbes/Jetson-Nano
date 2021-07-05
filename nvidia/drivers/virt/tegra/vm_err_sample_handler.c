/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#define pr_fmt(fmt) "vm-err-sample-handler: " fmt

#include <linux/module.h>
#include <linux/vm_err.h>

#define ESR_EC_INSTR_ABORT_LOWER_EL 0x20U
#define ESR_EC_DATA_ABORT_LOWER_EL 0x24U

#define ESR_DABT_ISS_ISV_MASK (1U << 24)
#define ESR_DABT_ISS_SAS_SHIFT 22U
#define ESR_DABT_ISS_SAS_MASK (3U << ESR_DABT_ISS_SAS_SHIFT)
#define ESR_DABT_ISS_FNV_MASK (1U << 10)
#define ESR_DABT_ISS_WNR_MASK (1U << 6)
#define ESR_DABT_ISS_DFSC_MASK 0x3F


/* Bridge error details:
 * Note: These are redefined here only to allow user friendly messages
 * describing the error.
 * This must match with "Timeout error" value in t18x_axi_errors[]
 * in nvidia/drivers/platform/tegra/bridge_mca.c
 */
static const unsigned int BRIDGE_ERROR_TIMEOUT = 18;

/* This must match with "CCPLEX" value in src_ids
 * in nvidia/drivers/platform/tegra/bridge_mca.c
 */
static const unsigned int BRIDGE_SRC_ID_CCPLEX = 1;

/* This must match with corresponding HV definition in pct.h */
static const unsigned int GUEST_UNASSIGNED = 18;

static struct tegra_hv_vm_err_handlers handlers;
static struct tegra_hv_config config;

static void print_bridge_error(const struct err_data_t * const err_data)
{
	const struct async_bridge_err_t * const br_err_data =
		&err_data->async_bridge_err;
	unsigned int protection;

	pr_crit("Bridge error details\n");
	pr_crit("--------------------------------------\n");
	pr_crit("Err count %d: %s FAULT ADDR 0x%x status1 0x%x status2 0x%x\n",
		br_err_data->count, br_err_data->br_name, br_err_data->err_addr,
		br_err_data->err_status1, br_err_data->err_status2);

	pr_crit("\tDirection: %s\n", br_err_data->rw ? "READ" : "WRITE");
	pr_crit("\tBridge ID: 0x%x\n", br_err_data->br_id);
	pr_crit("\tError type: %u %s\n",
		br_err_data->err_type,
		(br_err_data->err_type == BRIDGE_ERROR_TIMEOUT) ?
			"(Timeout)" : "");

	pr_crit("\tLength: %d\n", br_err_data->length);
	protection = br_err_data->protection;
	pr_crit("\tProtection: 0x%x %s %s %s access\n", protection,
		(protection & 0x4) ? "Instruction" : "Data",
		(protection & 0x2) ? "Non-Secure" : "Secure",
		(protection & 0x1) ? "Privileged" : "Unprivileged");

	pr_crit("\tSource ID: 0x%x -- %s\n",
		br_err_data->src_id,
		(br_err_data->src_id == BRIDGE_SRC_ID_CCPLEX) ?
			" (CCPLEX)" : "");

	pr_crit("\tAXI_ID: 0x%x\n", br_err_data->axi_id);
	pr_crit("\tCache: 0x%x\n", br_err_data->cache);
	pr_crit("\tBurst: 0x%x\n", br_err_data->burst);
	pr_crit("--------------------------------------\n");
}

static void print_cache(uint32_t cache)
{
	if ((cache & 0x3) == 0x0) {
		pr_crit("\t Cache\t\t\t: 0x%x--Non-cacheable/Non-Bufferable)\n",
			cache);
		return;
	}
	if ((cache & 0x3) == 0x1) {
		pr_crit("\t  Cache\t\t\t: 0x%x -- Device\n",
			cache);
		return;
	}

	switch (cache) {
	case 0x2:
		pr_crit("\t  Cache\t\t\t: 0x%x -- Cacheable/Non-Bufferable\n",
			cache);
		break;
	case 0x3:
		pr_crit("\t  Cache\t\t\t: 0x%x -- Cacheable/Bufferable\n",
			cache);
		break;
	default:
		pr_crit("\t  Cache\t\t\t: 0x%x -- Cacheable\n",
			cache);
	}
}

static void print_prot(uint32_t prot)
{
	char *data_str;
	char *secure_str;
	char *priv_str;

	data_str = (prot & 0x4) ? "Instruction" : "Data";
	secure_str = (prot & 0x2) ? "Non-Secure" : "Secure";
	priv_str = (prot & 0x1) ? "Privileged" : "Unprivileged";

	pr_crit("\t  Protection\t\t: 0x%x -- %s, %s, %s Access\n",
		prot, priv_str, secure_str, data_str);
}

static void print_cbb_error(const struct err_data_t * const err_data)
{
	const struct async_cbb_err_t * const cbb_err_data =
		&err_data->async_cbb_err;

	pr_crit("Control Back Bone(CBB) error details\n");
	pr_crit("--------------------------------------\n");
	pr_crit("Error:%s\n", cbb_err_data->cbb_name);
	pr_crit("\tError Logger\t\t: %d\n", cbb_err_data->error_logger);
	pr_crit("\tErrLog0\t\t\t: 0x%x\n", cbb_err_data->errlog0);
	pr_crit("\t  Transaction Type\t: %s\n", cbb_err_data->transaction_type);
	pr_crit("\t  Error Code\t\t: %s\n", cbb_err_data->error_code);
	pr_crit("\t  Error Source\t\t: %s\n",
		cbb_err_data->error_source);
	pr_crit("\t  Error Description\t: %s\n",
		cbb_err_data->error_description);
	pr_crit("\t  Packet header Lock\t: %d\n",
			cbb_err_data->packet_header_lock);
	pr_crit("\t  Packet header Len1\t: %d\n",
			cbb_err_data->packet_header_len1);

	if (cbb_err_data->header_format)
		pr_crit("\t  NOC protocol version\t: %s\n", "version >= 2.7");
	else
		pr_crit("\t  NOC protocol version\t: %s\n", "version < 2.7");
	pr_crit("\tErrLog1\t\t\t: 0x%x\n", cbb_err_data->errlog1);
	pr_crit("\tErrLog2\t\t\t: 0x%x\n", cbb_err_data->errlog2);
	pr_crit("\t  RouteId\t\t: 0x%llx\n", cbb_err_data->route_id);
	pr_crit("\t  InitFlow\t\t: %s\n", cbb_err_data->initflow);
	pr_crit("\t  Targflow\t\t: %s\n", cbb_err_data->targflow);
	pr_crit("\t  TargSubRange\t\t: %d\n", cbb_err_data->targ_subrange);
	pr_crit("\t  SeqId\t\t\t: %d\n", cbb_err_data->seqid);
	pr_crit("\tErrLog3\t\t\t: 0x%x\n", cbb_err_data->errlog3);
	pr_crit("\tErrLog4\t\t\t: 0x%x\n", cbb_err_data->errlog4);
	pr_crit("\t  Address\t\t: 0x%llx\n", cbb_err_data->address);
	pr_crit("\tErrLog5\t\t\t: 0x%x\n", cbb_err_data->errlog5);
	pr_crit("\t  Master ID\t\t: %s\n", cbb_err_data->master_id);
	pr_crit("\t  Non-Modify\t\t: 0x%x\n", cbb_err_data->non_mod);
	pr_crit("\t  AXI ID\t\t: 0x%x\n", cbb_err_data->axi_id);
	pr_crit("\t  Security Group(GRPSEC): 0x%x\n",
			cbb_err_data->security_group);

	print_cache(cbb_err_data->cache);
	print_prot(cbb_err_data->protection);

	pr_crit("\t  FALCONSEC\t\t: 0x%x\n", cbb_err_data->falconsec);
	pr_crit("\t  Virtual Queuing Channel(VQC): 0x%x\n",
		cbb_err_data->virtual_q_channel);
	pr_crit("--------------------------------------\n");
}

static void print_smmu_error(const struct err_data_t * const err_data,
				const enum err_reason reason)
{
	const struct async_smmu_err_t * const smmu_err_data =
		&err_data->async_smmu_err;

	pr_crit("SMMU error details\n");
	pr_crit("--------------------------------------\n");
	if (reason == REASON_ASYNC_SMMU_CB) {
		pr_crit("SMMU Context Bank %u error. StreamID: %d\n",
			smmu_err_data->cb_id, smmu_err_data->stream_id);
	} else if (reason == REASON_ASYNC_SMMU_GLOBAL) {
		pr_crit("Global SMMU fault. CB: %u. StreamID: %d\n",
			smmu_err_data->cb_id, smmu_err_data->stream_id);
	} else {
		pr_crit("Unexpected fault reason %d\n", reason);
	}
	pr_crit("FSR: 0x%x; FAR: 0x%llx; FSYND0: 0x%x; FSYND1: 0x%x\n",
			smmu_err_data->fsr, smmu_err_data->far,
			smmu_err_data->fsynr0, smmu_err_data->fsynr1);
	pr_crit("--------------------------------------\n");
}

static void print_t19x_mc_error(const struct err_data_t * const err_data)
{
	const struct async_mc_err_t19x_t * const mc_err_data_t19x =
					&err_data->async_mc_err_t19x;

	pr_crit("Memory Controller error details\n");
	pr_crit("--------------------------------------\n");

	if (mc_err_data_t19x->vpr_violation) {
		pr_crit("vpr base=%x:%x, size=%x, ctrl=%x, override:(%x, %x, %x, %x)\n",
			mc_err_data_t19x->vpr_base[0],
			mc_err_data_t19x->vpr_base[1],
			mc_err_data_t19x->vpr_size,
			mc_err_data_t19x->vpr_ctrl,
			mc_err_data_t19x->vpr_override[0],
			mc_err_data_t19x->vpr_override[1],
			mc_err_data_t19x->vpr_override[2],
			mc_err_data_t19x->vpr_override[3]);
	}

	if (mc_err_data_t19x->no_status) {
		pr_crit("MC fault - no status: %s\n",
			mc_err_data_t19x->fault_msg);
	} else if (mc_err_data_t19x->two_status) {
		pr_crit("MC fault - %s\n", mc_err_data_t19x->fault_msg);
		pr_crit("status: 0x%08x status2: 0x%08llx\n",
			mc_err_data_t19x->status,
			(unsigned long long int)mc_err_data_t19x->address);
	} else {
		pr_crit("(%d) %s: %s\n", mc_err_data_t19x->client_swgid,
			mc_err_data_t19x->client_name,
			mc_err_data_t19x->fault_msg);
		pr_crit("  status = 0x%08x; addr = 0x%08llx\n",
			mc_err_data_t19x->status,
			(unsigned long long int)mc_err_data_t19x->address);
		pr_crit("  secure: %s, access-type: %s\n",
			mc_err_data_t19x->secure ? "yes" : "no",
			mc_err_data_t19x->write ? "write" : "read");
	}
}

static void print_mc_error(const struct err_data_t * const err_data)
{
	const struct async_mc_err_t * const mc_err_data =
		&err_data->async_mc_err;

	pr_crit("Memory Controller error details\n");
	pr_crit("--------------------------------------\n");
	pr_crit("mc_err: base: 0x%llx, int_status: 0x%08x; err_status: 0x%08x;"
		" fault_addr: 0x%llx\n",
		mc_err_data->ch_base, mc_err_data->int_status,
		mc_err_data->err_status, mc_err_data->fault_addr);
	pr_crit("vcpuid %u, client_id %u, peripheral_id %d\n",
		mc_err_data->vcpuid, mc_err_data->client_id,
		mc_err_data->peripheral_id);
	pr_crit("--------------------------------------\n");
}

static void print_sync_full_details(const struct err_data_t *const err_data,
		bool with_frame)
{
	int i;
	const struct sync_t * const sync = &err_data->sync;

	if (with_frame) {
		pr_crit("Synchronous exception, full details\n");
		pr_crit("--------------------------------------\n");

	}
	pr_crit("Offending VCpu Id %u\n", sync->offending_vcpu_id);
	pr_crit("(Following register validity depends on error context)\n");
	pr_crit("ESR_EL2:               0x%08x\n", sync->esr_el2);
	pr_crit("ELR_EL2:       0x%016llx\n", sync->elr_el2);
	pr_crit("FAR_EL2:       0x%016llx\n", sync->far_el2);
	pr_crit("HPFAR_EL2:     0x%016llx\n", sync->hpfar_el2);
	pr_crit("PAR_HPFAR_EL2: 0x%016llx\n", sync->par_hpfar_el2);
	pr_crit("SPSR_EL2:      0x%016llx\n", sync->spsr_el2);
	pr_crit("ELR_EL1:       0x%016llx\n", sync->elr_el1);
	pr_crit("FAR_EL1:       0x%016llx\n", sync->far_el1);
	pr_crit("SPSR_EL1:      0x%016llx\n", sync->spsr_el1);
	pr_crit("ESR_EL1:               0x%08x\n", sync->esr_el1);
	pr_crit("Fault Instr:           0x%08x\n", sync->fault_instr);
	pr_crit("General Purpose Registers\n");

	for (i = 0; i < 30; i++)
		pr_crit("    x%02u: 0x%016llx\n", i, sync->gpr_array[i]);

	pr_crit("    x30: 0x%016llx (link register)\n", sync->gpr_array[30]);

	if (with_frame)
		pr_crit("--------------------------------------\n");
}

static inline uint8_t extract_dabt_iss_sas(const uint32_t esr)
{
	return ((esr & ESR_DABT_ISS_SAS_MASK) >> ESR_DABT_ISS_SAS_SHIFT);
}

static void print_data_abort(const struct err_data_t *const err_data)
{
	const struct sync_t * const data_abort = &err_data->sync;
	uint32_t esr = data_abort->esr_el2;

	pr_crit("Data abort details\n");
	pr_crit("--------------------------------------\n");

	// Check if instruction syndrome is valid.
	if (esr & ESR_DABT_ISS_ISV_MASK) {
		bool is_write = (esr & ESR_DABT_ISS_WNR_MASK) > 0;
		uint8_t access_size = (1U << extract_dabt_iss_sas(esr));

		(is_write) ? pr_crit("write access\n") :
				pr_crit("read access\n");
		pr_crit("access size %u\n", access_size);
	}

	// Check if both fault addresses are valid.
	if ((!(esr & ESR_DABT_ISS_FNV_MASK)) &&
			((esr & ESR_DABT_ISS_DFSC_MASK) < 13U)) {
		uint64_t fault_addr = data_abort->hpfar_el2 << 8 |
				(data_abort->far_el2 & 0xfffULL);

		pr_crit("Fault address: 0x%llx\n", fault_addr);
	}

	pr_crit("Additional context:\n");

	// Print rest of trap context
	print_sync_full_details(err_data, false);

	pr_crit("--------------------------------------\n");
}

static void print_instr_abort(const struct err_data_t *const err_data)
{
	pr_crit("Instruction abort details\n");
	pr_crit("--------------------------------------\n");

	// Print trap context
	print_sync_full_details(err_data, false);

	pr_crit("--------------------------------------\n");
}

static inline uint32_t exception_class(const uint32_t esr)
{
	uint32_t ret = esr & 0xFC000000U;

	return (ret >> 26);
}

static void print_sync(const struct err_data_t *const err_data)
{
	if (!(err_data->sync.is_filled)) {
		pr_crit("Synchronous exception, no details available\n");
		return;
	}

	/* For demo purposes, do some preprocessing for data aborts and detect
	 * instr abort class.
	 *
	 * Full details for an implementation that is more tailored to a
	 * specific use case to be found in the AArch64 reference manual.
	 */
	switch (exception_class(err_data->sync.esr_el2)) {
	case ESR_EC_DATA_ABORT_LOWER_EL:
		print_data_abort(err_data);
		break;

	case ESR_EC_INSTR_ABORT_LOWER_EL:
		print_instr_abort(err_data);
		break;

	default:
		print_sync_full_details(err_data, true);
		break;
	}
}

static bool handle_async_err_details(const struct err_data_t * const err_data)
{
	bool enter_bad_mode;

	if (err_data->err_type != ASYNC) {
		pr_crit("%s: incorrect error type: %d\n", __func__,
			err_data->err_type);
		/* Unexpected error type. Enter bad mode. */
		return true;
	}

	pr_info("%s: error reason: %s\n", __func__,
		tegra_hv_err_reason_desc[err_data->err_reason]);
	switch (err_data->err_reason) {
	case REASON_ASYNC_BRIDGE:
		print_bridge_error(err_data);
		/* Bridge error may not be fatal */
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_CBB:
		print_cbb_error(err_data);
		/* CBB error may not be fatal */
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_SMMU_CB:
		print_smmu_error(err_data, err_data->err_reason);
		/* SMMU context bank error may not be fatal */
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_SMMU_GLOBAL:
		print_smmu_error(err_data, err_data->err_reason);
		/* Can't recover from global SMMU error. */
		enter_bad_mode = true;
		break;

	case REASON_ASYNC_MC:
		print_mc_error(err_data);
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_MC_T19X:
		print_t19x_mc_error(err_data);
		enter_bad_mode = false;
		break;

	default:
		pr_crit("%s: unhandled error. Reason id %d\n", __func__,
			err_data->err_reason);
		enter_bad_mode = true;
		break;
	}

	return enter_bad_mode;
}

static bool handle_sync_err_details(const struct err_data_t * const err_data)
{
	pr_info("%s: error reason: %s\n", __func__,
		tegra_hv_err_reason_desc[err_data->err_reason]);

	print_sync(err_data);

	/* Recovery from sync error could be impossible. Enter bad mode. */
	return true;
}

static bool handle_peer_err_details(const struct err_data_t * const err_data)
{
	bool enter_bad_mode;
	const unsigned int offender = err_data->offending_guest_id;

	if (offender >= config.num_guests) {
		if (offender != GUEST_UNASSIGNED) {
			pr_crit("%s: invalid offending peer guest id %u\n",
				__func__, offender);
			/* Unexpected. Cause reboot. */
			return true;
		}
		pr_crit("%s: HV can't attribute error to any guest\n",
			__func__);
	} else
		pr_crit("Peer error. Offending guest id = %u\n", offender);

	pr_crit("Error Type: %s\n", (err_data->err_type == SYNC) ?
		"Synchronous" : "Asynchronous");

	if (err_data->err_reason >= REASON_ENUM_SIZE) {
		pr_crit("%s: unexpected reason id %u\n", __func__,
			err_data->err_reason);
		/* Unexpected. Cause reboot. */
		return true;
	}
	pr_crit("%s: error reason: %s\n", __func__,
		tegra_hv_err_reason_desc[err_data->err_reason]);

	switch (err_data->err_reason) {
	case REASON_ASYNC_BRIDGE:
		print_bridge_error(err_data);
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_CBB:
		print_cbb_error(err_data);
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_SMMU_CB:
	case REASON_ASYNC_SMMU_GLOBAL:
		print_smmu_error(err_data, err_data->err_reason);
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_MC:
		print_mc_error(err_data);
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_MC_T19X:
		print_t19x_mc_error(err_data);
		enter_bad_mode = false;
		break;

	case REASON_SYNC:
		print_sync(err_data);
		enter_bad_mode = false;
		break;

	default:
		pr_crit("%s: unhandled error. Reason id %d\n", __func__,
		err_data->err_reason);
		enter_bad_mode = false;
		break;
	}

	return enter_bad_mode;
}

static bool self_async_err_handler(const struct err_data_t *const err_data)
{
	return handle_async_err_details(err_data);
}

static bool self_sync_err_handler(const struct err_data_t *const err_data)
{
	return handle_sync_err_details(err_data);
}

static bool peer_err_handler(const struct err_data_t *const err_data)
{
	return handle_peer_err_details(err_data);
}

static int hooks_init(void)
{
	int ret;

	handlers.fn_self_async = self_async_err_handler;
	handlers.fn_self_sync = self_sync_err_handler;
	handlers.fn_peer =
		IS_ENABLED(CONFIG_TEGRA_EBP) ? NULL : peer_err_handler;

	ret = tegra_hv_register_vm_err_hooks(&handlers);
	if (ret)
		return ret;

	tegra_hv_get_config(&config);
	pr_info("%s: Guest Id %u\n", __func__, config.guest_id_self);

	/* EBP, being unprivileged, doesn't know about total guests */
	if (IS_ENABLED(CONFIG_TEGRA_EBP) == 0)
		pr_info("%s: Total guests %u\n", __func__, config.num_guests);

	return 0;
}

static void hooks_exit(void)
{
	struct tegra_hv_vm_err_handlers handlers;

	handlers.fn_self_async = NULL;
	handlers.fn_self_sync = NULL;
	handlers.fn_peer = NULL;

	tegra_hv_register_vm_err_hooks(&handlers);
}
subsys_initcall(hooks_init);
module_exit(hooks_exit);

MODULE_AUTHOR("Nvidia Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sample VM Error Handler");
