/*
 * sensor_kernel_tests_core - sensor kernel tests module core
 *
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/netlink.h>
#include <net/genetlink.h>

#include "sensor_kernel_tests_core.h"
#include "sensor_kernel_tests_runner.h"

#define SKT_NL_FAMILY ("SKT_TESTS")
#define SKT_GET_ERR(e) ((e == NULL) ? -EFAULT : PTR_ERR(e))

enum sensor_kernel_tests_attrs {
	SKT_ATTR_UNSPECIFIED = 0,
	/* Glob pattern for filtering tests */
	SKT_ATTR_GLOB,
	/* Generic result */
	SKT_ATTR_RESULT,
	/* Sensor compatible name */
	SKT_ATTR_SEN_COMPAT,
	/* Sensor name */
	SKT_ATTR_SEN_NAME,
	/* Tegra V4L2 Camera Framework version */
	SKT_ATTR_TVCF_VERSION,
	/* A generic null-terminated message */
	SKT_ATTR_MSG,
	SKT_ATTR_MAX,
};

enum sensor_kernel_tests_cmds {
	SKT_CMD_UNSPECIFIED = 0,
	/* Query available tests */
	SKT_CMD_QUERY_TESTS,
	/* Run available tests */
	SKT_CMD_RUN_TESTS,
	/* Acknowledge a KMD-initiated message */
	SKT_CMD_ACK,
	SKT_CMD_MAX,
};

/* Generic Netlink Callbacks */
static int skt_query_tests(struct sk_buff *skb, struct genl_info *info);
static int skt_run_tests(struct sk_buff *skb, struct genl_info *info);
static int skt_ack(struct sk_buff *skb, struct genl_info *info);

static struct nla_policy skt_policies[SKT_ATTR_MAX] = {
	[SKT_ATTR_UNSPECIFIED] =  {
					.type = NLA_UNSPEC,
					.len = 0 },
	[SKT_ATTR_GLOB] =         {
					.type = NLA_NUL_STRING,
					.len = SKT_NL_MAX_STR_LEN },
	[SKT_ATTR_RESULT] =       {
					.type = NLA_S32 },
	[SKT_ATTR_SEN_COMPAT] =   {
					.type = NLA_NUL_STRING,
					.len = SKT_NL_MAX_STR_LEN },
	[SKT_ATTR_SEN_NAME] =     {
					.type = NLA_NUL_STRING,
					.len = SKT_NL_MAX_STR_LEN },
	[SKT_ATTR_TVCF_VERSION] = {
					.type = NLA_U32 },
	[SKT_ATTR_MSG] =          {
					.type = NLA_NUL_STRING,
					.len = SKT_NL_MAX_STR_LEN },
};

static struct genl_ops sensor_kernel_test_ops[] = {
	{
		.cmd = SKT_CMD_QUERY_TESTS,
		.flags = GENL_CMD_CAP_HASPOL,
		.policy = skt_policies,
		.doit = skt_query_tests,
	},
	{
		.cmd = SKT_CMD_RUN_TESTS,
		.flags = GENL_CMD_CAP_HASPOL,
		.policy = skt_policies,
		.doit = skt_run_tests,
	},
	{
		.cmd = SKT_CMD_ACK,
		.flags = GENL_CMD_CAP_HASPOL,
		.policy = skt_policies,
		.doit = skt_ack,
	},
};

static struct genl_family skt_family = {
	.hdrsize = 0,
	.name = SKT_NL_FAMILY,
	.version = 1,
	.maxattr = SKT_ATTR_MAX,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	.ops = sensor_kernel_test_ops,
	.n_ops = ARRAY_SIZE(sensor_kernel_test_ops),
#else
	.id = GENL_ID_GENERATE,
#endif
};

struct skt_core_msg {
	struct sk_buff *msg;
	void *msg_hdr;
};

struct skt_core_worker {
	struct work_struct work;
	struct skt_runner_ctx ctx;
};

static struct skt_core_worker test_worker;
static DECLARE_WAIT_QUEUE_HEAD(test_worker_wq);
static bool user_acked;

static inline void skt_core_free_msg(struct skt_core_msg *skt_msg)
{
	nlmsg_free(skt_msg->msg);
	kfree(skt_msg);
}

static struct skt_core_msg *skt_core_get_msg(const u32 size,
		const u32 portid, const u32 seq, const int flags,
		const u8 cmd)
{
	struct skt_core_msg *skt_msg;

	skt_msg = kzalloc(sizeof(*skt_msg), GFP_KERNEL);
	if (skt_msg == NULL)
		return ERR_PTR(-ENOMEM);

	skt_msg->msg = genlmsg_new(size, GFP_KERNEL);
	if (skt_msg->msg == NULL) {
		kfree(skt_msg);
		return ERR_PTR(-ENOMEM);
	}

	skt_msg->msg_hdr = genlmsg_put(skt_msg->msg, portid, seq, &skt_family,
			flags, cmd);
	if (skt_msg->msg_hdr == NULL) {
		skt_core_free_msg(skt_msg);
		return ERR_PTR(-EOVERFLOW);
	}

	return skt_msg;
}

static inline int skt_core_send_msg(struct skt_core_msg *skt_msg,
		struct net *net, const u32 portid)
{
	genlmsg_end(skt_msg->msg, skt_msg->msg_hdr);
	return genlmsg_unicast(net, skt_msg->msg, portid);
}

static inline int skt_core_append_msg(struct skt_core_msg *skt_msg,
		const char *buff)
{
	return nla_put_string(skt_msg->msg, SKT_ATTR_MSG, buff);
}

static inline int skt_core_append_result(struct skt_core_msg *skt_msg,
		const s32 result)
{
	return nla_put_s32(skt_msg->msg, SKT_ATTR_RESULT, result);
}


int skt_core_vlog_msg(const u32 portid, const char *fmt, va_list args)
{
	struct skt_core_msg *msg;
	char buff[SKT_NL_MAX_STR_LEN];
	int bytes;
	int err;
	int ret;

	if (fmt == NULL)
		return -EINVAL;

	bytes = vsnprintf(buff, SKT_NL_MAX_STR_LEN, fmt, args);
	if (bytes >= SKT_NL_MAX_STR_LEN)
		pr_warn("skt core message truncated\n");

	msg = skt_core_get_msg(strlen(buff) + 1, portid,
			0, 0, SKT_CMD_RUN_TESTS);
	if (IS_ERR_OR_NULL(msg)) {
		err = SKT_GET_ERR(msg);
		pr_err("Could not get skt msg (%d)\n", err);
		return err;
	}

	err = skt_core_append_msg(msg, buff);
	if (err != 0) {
		pr_err("Could not append skt msg (%d)\n", err);
		skt_core_free_msg(msg);
		return err;
	}

	err = skt_core_send_msg(msg, &init_net, portid);
	if (err != 0)
		pr_err("Could not send skt msg (%d)\n", err);
	else {
		/*
		 * KMD->UMD communication is buffered. Primitive flow control
		 * implemented as msg + ACK keeps the socket buffers from
		 * overflowing if a test(s) dumps more output than userspace
		 * can consume.
		 */
		ret = wait_event_interruptible_timeout(test_worker_wq,
				(user_acked == true), msecs_to_jiffies(1000));
		user_acked = false;

		if (ret == 0) {
			kfree(msg);
			pr_warn("skt ACK timed out\n");
			return -ETIMEDOUT;
		}
	}

	kfree(msg);
	return err;
}

int skt_core_log_msg(const u32 portid, const char *fmt, ...)
{
	va_list args;
	int err;

	if (fmt == NULL)
		return -EINVAL;

	va_start(args, fmt);
	err = skt_core_vlog_msg(portid, fmt, args);
	va_end(args);

	return err;
}

static int skt_format_test(struct skt_core_msg *msg,
		const struct skt_test *test)
{
	char buff[SKT_NL_MAX_STR_LEN];
	int bytes = 0;

	bytes = snprintf(buff, SKT_NL_MAX_STR_LEN, "%s\n - %s\n",
			test->name, test->description);
	if (bytes >= SKT_NL_MAX_STR_LEN)
		pr_warn("skt format test truncated\n");

	return skt_core_append_msg(msg, buff);
}

static int skt_query_tests(struct sk_buff *skb, struct genl_info *info)
{
	struct skt_core_msg *msg;
	struct skt_test **tests;
	int num_tests;
	const char *glob = NULL;
	int err;
	int i;

	num_tests = skt_runner_num_tests();
	tests = kcalloc(num_tests, sizeof(*tests), GFP_KERNEL);

	msg = skt_core_get_msg(NLMSG_GOODSIZE, info->snd_portid,
			info->snd_seq, 0, SKT_CMD_QUERY_TESTS);
	if (IS_ERR_OR_NULL(msg)) {
		err = SKT_GET_ERR(msg);
		pr_err("Could not get skt msg (%d)\n", err);
		goto query_test_fail;
	}

	if (info->attrs[SKT_ATTR_GLOB] != NULL)
		glob = (const char *)nla_data(info->attrs[SKT_ATTR_GLOB]);

	num_tests = skt_runner_query_tests(glob, tests, num_tests);
	if (num_tests <= 0) {
		err = skt_core_append_msg(msg, "** No tests found **\n");
		if (err != 0) {
			pr_err("Could not append skt msg (%d)\n", err);
			goto genl_fail;
		}
	} else {
		for (i = 0; i < num_tests; i++) {
			err = skt_format_test(msg, tests[i]);
			if (err != 0) {
				pr_err("Could not format test name (%d)\n",
						err);
				goto genl_fail;
			}
		}
	}

	err = skt_core_append_result(msg, 0);
	if (err != 0) {
		pr_err("Could not append skt result (%d)\n", err);
		goto genl_fail;
	}

	kfree(tests);

	err = skt_core_send_msg(msg, genl_info_net(info), info->snd_portid);
	if (err != 0)
		pr_err("Could not send skt msg (%d)\n", err);
	kfree(msg);
	return err;

genl_fail:
	pr_err("skt message failed (%d)\n", err);
	skt_core_free_msg(msg);
query_test_fail:
	pr_err("skt query test failed (%d)\n", err);
	kfree(tests);
	return err;
}

static void skt_core_test_work(struct work_struct *work)
{
	struct skt_core_msg *msg;
	struct skt_core_worker *worker;
	int result;
	int err;

	if (work == NULL)
		return;

	worker = container_of(work, struct skt_core_worker, work);

	msg = skt_core_get_msg(NLMSG_GOODSIZE, worker->ctx.dest_portid,
			0, 0, SKT_CMD_RUN_TESTS);
	if (IS_ERR_OR_NULL(msg)) {
		err = SKT_GET_ERR(msg);
		pr_err("Could not get skt msg (%d)\n", err);
		return;
	}

	result = skt_runner_run_tests(&worker->ctx);

	err = skt_core_append_result(msg, result);
	if (err != 0) {
		pr_err("Could not append skt result (%d)\n", err);
		skt_core_free_msg(msg);
		return;
	}

	err = skt_core_send_msg(msg, &init_net, worker->ctx.dest_portid);
	if (err != 0)
		pr_err("Could not send skt msg (%d)\n", err);
	kfree(msg);
}

static int skt_run_tests(struct sk_buff *skb, struct genl_info *info)
{
	struct skt_runner_ctx *ctx = &test_worker.ctx;
	struct skt_core_msg *msg;
	int err = 0;

	if (info->attrs[SKT_ATTR_GLOB] != NULL) {
		nla_strlcpy(ctx->glob, info->attrs[SKT_ATTR_GLOB],
				SKT_RUNNER_STR_LEN);
	} else
		ctx->glob[0] = '\0';

	if (info->attrs[SKT_ATTR_SEN_COMPAT] != NULL) {
		nla_strlcpy(ctx->compat, info->attrs[SKT_ATTR_SEN_COMPAT],
				SKT_RUNNER_STR_LEN);
	} else
		ctx->compat[0] = '\0';

	if (info->attrs[SKT_ATTR_SEN_NAME] != NULL) {
		nla_strlcpy(ctx->name, info->attrs[SKT_ATTR_SEN_NAME],
				SKT_RUNNER_STR_LEN);
	} else
		ctx->name[0] = '\0';

	if (info->attrs[SKT_ATTR_TVCF_VERSION] != NULL) {
		ctx->tvcf_version = nla_get_u32(
				info->attrs[SKT_ATTR_TVCF_VERSION]);
	}

	ctx->dest_portid = info->snd_portid;
	user_acked = false;

	if (!schedule_work(&test_worker.work))
		goto run_tests_fail;

	return 0;

run_tests_fail:
	pr_err("Could not schedule test work\n");
	msg = skt_core_get_msg(NLMSG_GOODSIZE, info->snd_portid,
			info->snd_seq, 0, SKT_CMD_RUN_TESTS);
	if (IS_ERR_OR_NULL(msg)) {
		err = SKT_GET_ERR(msg);
		pr_err("Could not get skt msg (%d)\n", err);
		return err;
	}

	err = skt_core_append_msg(msg, "Could not schedule test work\n");
	if (err != 0) {
		pr_err("Could not append skt msg (%d)\n", err);
		goto genl_fail;
	}

	err = skt_core_append_result(msg, -EBUSY);
	if (err != 0) {
		pr_err("Could not append skt result (%d)\n", err);
		goto genl_fail;
	}

	err = skt_core_send_msg(msg, genl_info_net(info), info->snd_portid);
	if (err != 0)
		pr_err("Could not send skt msg (%d)\n", err);

	kfree(msg);
	return err;

genl_fail:
	pr_err("skt message failed (%d)\n", err);
	skt_core_free_msg(msg);
	return err;
}

static int skt_ack(struct sk_buff *skb, struct genl_info *info)
{
	user_acked = true;
	wake_up_interruptible(&test_worker_wq);
	return 0;
}

static int __init skt_init(void)
{
	int err = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0)
	err = genl_register_family_with_ops(&skt_family,
			sensor_kernel_test_ops);
#else
	err = genl_register_family(&skt_family);
#endif
	if (err != 0) {
		pr_err("Could not register genetlink family (%d)\n", err);
		return err;
	}
	pr_debug("SKT family ID is %u\n", skt_family.id);

	INIT_WORK(&test_worker.work, skt_core_test_work);

	return err;
}

static void __exit skt_remove(void)
{
	genl_unregister_family(&skt_family);
	cancel_work_sync(&test_worker.work);
}

module_init(skt_init);
module_exit(skt_remove);
MODULE_LICENSE("GPL v2");
