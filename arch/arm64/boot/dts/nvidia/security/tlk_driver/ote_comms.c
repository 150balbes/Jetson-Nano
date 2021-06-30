/*
 * Copyright (c) 2012-2019 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/atomic.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/ioctl.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/ote_protocol.h>
#include <asm/smp_plat.h>

#include "ote_protocol.h"

bool verbose_smc;
core_param(verbose_smc, verbose_smc, bool, 0644);

#define SET_RESULT(req, r, ro)	{ req->result = r; req->result_origin = ro; }

static struct te_session *te_get_session(struct tlk_context *context,
					uint32_t session_id)
{
	struct te_session *session, *tmp_session;

	list_for_each_entry_safe(session, tmp_session,
		&context->session_list, list) {
		if (session->session_id == session_id)
			return session;
	}

	return NULL;
}

#ifdef CONFIG_SMP
cpumask_t saved_cpu_mask;
static long switch_cpumask_to_cpu0(void)
{
	long ret;
	cpumask_t local_cpu_mask = CPU_MASK_NONE;

	cpumask_set_cpu(0, &local_cpu_mask);
	cpumask_copy(&saved_cpu_mask, &current->cpus_allowed);
	ret = sched_setaffinity(0, &local_cpu_mask);
	if (ret)
		pr_err("%s: sched_setaffinity #1 -> 0x%lX", __func__, ret);

	return ret;
}

static void restore_cpumask(void)
{
	long ret = sched_setaffinity(0, &saved_cpu_mask);
	if (ret)
		pr_err("%s: sched_setaffinity #2 -> 0x%lX", __func__, ret);
}
#else
static inline long switch_cpumask_to_cpu0(void) { return 0; };
static inline void restore_cpumask(void) {};
#endif

struct tlk_smc_work_args {
	uint32_t arg0;
	uintptr_t arg1;
	uint32_t arg2;
};

static long tlk_generic_smc_on_cpu0(void *args)
{
	struct tlk_smc_work_args *work;
	int callback_status = 0;
	uint32_t retval;

	work = (struct tlk_smc_work_args *)args;
	retval = _tlk_generic_smc(work->arg0, work->arg1, work->arg2);

	while (retval == TE_ERROR_PREEMPT_BY_IRQ ||
	       retval == TE_ERROR_PREEMPT_BY_FS) {
		if (retval == TE_ERROR_PREEMPT_BY_FS)
			callback_status = tlk_ss_op();
		retval = _tlk_generic_smc(TE_SMC_RESTART, callback_status, 0);
	}

	/* Print TLK logs if any */
	ote_print_logs();

	return retval;
}

/*
 * This routine is called both from normal threads and worker threads.
 * The worker threads are per-cpu and have PF_NO_SETAFFINITY set, so
 * any calls to sched_setaffinity will fail.
 *
 * If it's a worker thread on CPU0, just invoke the SMC directly. If
 * it's running on a non-CPU0, use work_on_cpu() to schedule the SMC
 * on CPU0.
 */
uint32_t tlk_send_smc(uint32_t arg0, uintptr_t arg1, uintptr_t arg2)
{
	long ret;
	struct tlk_smc_work_args work_args;

	work_args.arg0 = arg0;
	work_args.arg1 = arg1;
	work_args.arg2 = arg2;

	if (current->flags &
	    (PF_WQ_WORKER | PF_NO_SETAFFINITY | PF_KTHREAD)) {
		int cpu = cpu_logical_map(get_cpu());
		put_cpu();

		/* workers don't change CPU. depending on the CPU, execute
		 * directly or sched work */
		if (cpu == 0 && (current->flags & PF_WQ_WORKER))
			return tlk_generic_smc_on_cpu0(&work_args);
		else
			return work_on_cpu(0,
					tlk_generic_smc_on_cpu0, &work_args);
	}

	/* switch to CPU0 */
	ret = switch_cpumask_to_cpu0();
	if (ret) {
		/* not able to switch, schedule work on CPU0 */
		ret = work_on_cpu(0, tlk_generic_smc_on_cpu0, &work_args);
	} else {
		/* switched to CPU0 */
		ret = tlk_generic_smc_on_cpu0(&work_args);
		restore_cpumask();
	}

	return ret;
}

/*
 * Do an SMC call
 */
static void do_smc(struct te_request *request, struct tlk_device *dev)
{
	uint32_t smc_args;
	uint32_t smc_params = 0;
	uint32_t retval = 0;

	smc_args = (uintptr_t)request - (uintptr_t)dev->req_addr;
	if (request->params) {
		smc_params =
			(uintptr_t)request->params - (uintptr_t)dev->req_addr;
	}

	retval = tlk_send_smc(request->type, smc_args, smc_params);

	/**
	 * Check for return code from TLK kernel and propagate to NS userspace
	 *
	 * Under certain situations, TLK kernel may return an error code
	 * _without_ updating the shared request buffer.
	 *
	 * This could give a false impression to the user-app about status of
	 * the request (if default value of request->result is OTE_SUCCESS).
	 *
	 * Propagate the error code to NS user-app in case of such scenario.
	 */
	if ((retval != OTE_SUCCESS) && (request->result == OTE_SUCCESS)) {
		pr_err("%s: overriding result_origin field\n", __func__);
		request->result = retval;
		request->result_origin = OTE_RESULT_ORIGIN_KERNEL;
	}
}

void tlk_restore_keyslots(void)
{
	uint32_t retval;

	if (!te_is_secos_dev_enabled())
		return;

	/* Share the same lock used when request is send from user side */
	mutex_lock(&smc_lock);

	retval = tlk_send_smc(TE_SMC_TA_EVENT, TA_EVENT_RESTORE_KEYS, 0);

	mutex_unlock(&smc_lock);

	if (retval != OTE_SUCCESS) {
		pr_err("%s: smc failed err (0x%x)\n", __func__, retval);
	}
}
EXPORT_SYMBOL(tlk_restore_keyslots);

/*
 * Open session SMC (supporting client-based te_open_session() calls)
 */
void te_open_session(struct te_opensession *cmd,
		    struct te_request *request,
		    struct tlk_context *context)
{
	struct te_session *session;
	int ret;

	session = kzalloc(sizeof(struct te_session), GFP_KERNEL);
	if (!session) {
		SET_RESULT(request, OTE_ERROR_OUT_OF_MEMORY,
			OTE_RESULT_ORIGIN_API);
		return;
	}

	INIT_LIST_HEAD(&session->list);
	INIT_LIST_HEAD(&session->temp_shmem_list);
	INIT_LIST_HEAD(&session->inactive_persist_shmem_list);
	INIT_LIST_HEAD(&session->persist_shmem_list);

	request->type = TE_SMC_OPEN_SESSION;

	ret = te_prep_mem_buffers(request, session);
	if (ret != OTE_SUCCESS) {
		pr_err("%s: te_prep_mem_buffers failed err (0x%x)\n",
			__func__, ret);
		SET_RESULT(request, ret, OTE_RESULT_ORIGIN_API);
		kfree(session);
		return;
	}

	memcpy(&request->dest_uuid,
	       &cmd->dest_uuid,
	       sizeof(struct te_service_id));

	pr_debug("OPEN_CLIENT_SESSION: 0x%x 0x%x 0x%x 0x%x\n",
		request->dest_uuid[0],
		request->dest_uuid[1],
		request->dest_uuid[2],
		request->dest_uuid[3]);

	do_smc(request, context->dev);

	if (request->result)
		goto error;

	/* release temporary mem buffers */
	te_release_mem_buffers(&session->temp_shmem_list);

	/* move persistent mem buffers to active list */
	te_activate_persist_mem_buffers(session);

	/* save off session_id and add to list */
	session->session_id = request->session_id;
	list_add_tail(&session->list, &context->session_list);

	return;
error:
	/* release buffers allocated in te_prep_mem_buffers */
	te_release_mem_buffers(&session->temp_shmem_list);
	te_release_mem_buffers(&session->inactive_persist_shmem_list);

	kfree(session);
}

/*
 * Close session SMC (supporting client-based te_close_session() calls)
 */
void te_close_session(struct te_closesession *cmd,
		     struct te_request *request,
		     struct tlk_context *context)
{
	struct te_session *session;

	request->session_id = cmd->session_id;
	request->type = TE_SMC_CLOSE_SESSION;

	do_smc(request, context->dev);
	if (request->result)
		pr_info("%s: error closing session: 0x%08x\n",
			__func__, request->result);

	session = te_get_session(context, cmd->session_id);
	if (!session) {
		pr_info("%s: session_id not found: 0x%x\n",
			__func__, cmd->session_id);
		return;
	}

	/* free session state */
	te_release_mem_buffers(&session->persist_shmem_list);
	list_del(&session->list);
	kfree(session);
}

/*
 * Launch operation SMC (supporting client-based te_launch_operation() calls)
 */
void te_launch_operation(struct te_launchop *cmd,
			struct te_request *request,
			struct tlk_context *context)
{
	struct te_session *session;
	int ret;

	session = te_get_session(context, cmd->session_id);
	if (!session) {
		pr_info("%s: session_id not found: 0x%x\n",
			__func__, cmd->session_id);
		SET_RESULT(request, OTE_ERROR_BAD_PARAMETERS,
				OTE_RESULT_ORIGIN_API);
		return;
	}

	request->session_id = cmd->session_id;
	request->command_id = cmd->operation.command;
	request->type = TE_SMC_LAUNCH_OPERATION;

	ret = te_prep_mem_buffers(request, session);
	if (ret != OTE_SUCCESS) {
		pr_err("%s: te_prep_mem_buffers failed err (0x%x)\n",
			__func__, ret);
		SET_RESULT(request, ret, OTE_RESULT_ORIGIN_API);
		return;
	}

	do_smc(request, context->dev);

	if (request->result)
		goto error;

	/* move persistent mem buffers to active list */
	te_activate_persist_mem_buffers(session);

	/* release temporary mem buffers */
	te_release_mem_buffers(&session->temp_shmem_list);

	return;

error:
	/* release buffers allocated in te_prep_mem_buffers */
	te_release_mem_buffers(&session->temp_shmem_list);
	te_release_mem_buffers(&session->inactive_persist_shmem_list);
}

/*
 * Command to open a session with the trusted app.
 * This API should only be called from the kernel space.
 * Takes UUID of the TA and size as argument
 * Returns Session ID if success or ERR when failure
 */
int te_open_trusted_session_tlk(u32 *ta_uuid, u32 uuid_size,
				u32 *session_id)
{
	struct te_request *request;
	struct te_cmd_req_desc *cmd_desc = NULL;

	mutex_lock(&smc_lock);

	/* Open & submit the work to SMC */
	cmd_desc = te_get_free_cmd_desc(&tlk_dev);

	if (!cmd_desc) {
		pr_err("%s: failed to get cmd_desc\n", __func__);
		goto error;
	}

	request = cmd_desc->req_addr;
	memset(request, 0, sizeof(struct te_request));
	request->type = TE_SMC_OPEN_SESSION;

	if (uuid_size != sizeof(request->dest_uuid)) {
		pr_err("%s: Invalid size sent!\n", __func__);
		goto error;
	}

	if (ta_uuid == NULL || session_id == NULL) {
		pr_err("%s: Null parameters sent!\n", __func__);
		goto error;
	}

	memcpy(request->dest_uuid, ta_uuid, sizeof(*ta_uuid)*4);

	do_smc(request, &tlk_dev);

	if (request->result) {
		pr_err("%s: error opening session: 0x%08x, 0x%08x\n",
		__func__, request->result, request->result_origin);
		goto error;
	}

	*session_id = request->session_id;
	if (cmd_desc)
		te_put_used_cmd_desc(&tlk_dev, cmd_desc);

	mutex_unlock(&smc_lock);
	return OTE_SUCCESS;

error:
	if (cmd_desc)
		te_put_used_cmd_desc(&tlk_dev, cmd_desc);

	mutex_unlock(&smc_lock);
	return OTE_ERROR_GENERIC;
}
EXPORT_SYMBOL(te_open_trusted_session_tlk);

/*
 * Command to close session opened with the trusted app.
 * This API should only be called from the kernel space.
 * Takes session Id and UUID of the TA as arguments
 */
void te_close_trusted_session_tlk(u32 session_id, u32 *ta_uuid,
				u32 uuid_size)
{
	struct te_request *request;
	struct te_cmd_req_desc *cmd_desc = NULL;

	mutex_lock(&smc_lock);

	/* Open & submit the work to SMC */
	cmd_desc = te_get_free_cmd_desc(&tlk_dev);

	if (!cmd_desc) {
		pr_err("%s: failed to get cmd_desc\n", __func__);
		goto error;
	}

	/* Close the session */
	request = cmd_desc->req_addr;
	memset(request, 0, sizeof(struct te_request));
	request->type = TE_SMC_CLOSE_SESSION;
	request->session_id = session_id;

	if (uuid_size != sizeof(request->dest_uuid)) {
		pr_err("%s: Invalid size sent!\n", __func__);
		goto error;
	}

	memcpy(request->dest_uuid, ta_uuid, sizeof(*ta_uuid)*4);

	do_smc(request, &tlk_dev);

	if (request->result) {
		pr_err("%s: error closing session: 0x%08x, 0x%08x\n",
		__func__, request->result, request->result_origin);
		goto error;
	}

error:
	if (cmd_desc)
		te_put_used_cmd_desc(&tlk_dev, cmd_desc);
	mutex_unlock(&smc_lock);
}
EXPORT_SYMBOL(te_close_trusted_session_tlk);

/*
 * Command to launch operations from the linux kernel to
 * the trusted app.
 * This API should be called only from the kernel space.
 * The pointer to the buffer from the kernel, length of thus buffer,
 * session Id, UUID of the TA and the command requested from the
 * TA should be passed as arguments
 * Returns SUCCESS or FAILURE
 */
int te_launch_trusted_oper_tlk(u8 *buf_ptr, u32 buf_len, u32 session_id,
			u32 *ta_uuid, u32 ta_cmd, u32 uuid_size)
{
	u32 i;
	struct te_request *request;
	struct te_oper_param user_param;
	struct te_oper_param *param_array;
	struct te_oper_param *params = NULL;
	struct te_cmd_req_desc *cmd_desc = NULL;
	u32 no_of_params = 1;

	mutex_lock(&smc_lock);

	/* Open & submit the work to SMC */
	cmd_desc = te_get_free_cmd_desc(&tlk_dev);
	params = te_get_free_params(&tlk_dev, no_of_params);

	if (!cmd_desc || !params) {
		pr_err("%s: failed to get cmd_desc/params\n", __func__);
		goto error;
	}

	/* launch operation */
	request = cmd_desc->req_addr;
	memset(request, 0, sizeof(struct te_request));
	request->params = (uintptr_t)params;
	request->params_size = no_of_params;
	request->session_id = session_id;
	request->command_id = ta_cmd;
	request->type = TE_SMC_LAUNCH_OPERATION;
	user_param.index = 0;
	user_param.u.Mem.len = buf_len;
	user_param.type = TE_PARAM_TYPE_MEM_RW;
	user_param.u.Mem.type = TE_MEM_TYPE_NS_KERNEL;
	user_param.u.Mem.base = (uint64_t)(uintptr_t)buf_ptr;

	if (uuid_size != sizeof(request->dest_uuid)) {
		pr_err("%s: Invalid size sent\n", __func__);
		goto error;
	}

	memcpy(request->dest_uuid, ta_uuid, sizeof(*ta_uuid)*4);

	param_array = (struct te_oper_param *)(uintptr_t)request->params;

	for (i = 0; i < no_of_params; i++)
		memcpy(param_array + i, &user_param,
		sizeof(struct te_oper_param));

	do_smc(request, &tlk_dev);

	if (request->result) {
		pr_err("%s: error launching session: 0x%08x, 0x%08x\n",
		__func__, request->result, request->result_origin);
		goto error;
	} else {
		if (cmd_desc)
			te_put_used_cmd_desc(&tlk_dev, cmd_desc);

		if (params)
			te_put_free_params(&tlk_dev, params, no_of_params);

		mutex_unlock(&smc_lock);
		return OTE_SUCCESS;
	}
error:
	if (cmd_desc)
		te_put_used_cmd_desc(&tlk_dev, cmd_desc);

	if (params)
		te_put_free_params(&tlk_dev, params, no_of_params);

	mutex_unlock(&smc_lock);
	return OTE_ERROR_GENERIC;
}
EXPORT_SYMBOL(te_launch_trusted_oper_tlk);
