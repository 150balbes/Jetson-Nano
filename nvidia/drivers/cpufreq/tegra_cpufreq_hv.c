/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/tegra-ivc.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra-cpufreq.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <asm/smp_plat.h>
#include <linux/platform/tegra/tegra18_cpu_map.h>

struct cpu_rate_msg {
	uint32_t rate_khz;
	uint8_t cl;
	uint32_t mpidr;
};

union msg_data {
	struct cpu_rate_msg cpu_rate;
};

#define TEGRA_CPUFREQ_IVC_MSG_LEN sizeof(union msg_data)

struct tegra_cpufreq_ivc_msg {
	uint32_t msg_id;
	uint32_t len;
	union msg_data buffer;
};

struct tegra_cpufreq_ivc_data {
	struct mutex mlock;
	struct tegra_cpufreq_ivc_msg cpufreq_ivc_msg;
	wait_queue_head_t r_wq; /* wait queue for IVC read */
	wait_queue_head_t w_wq; /* wait queue for IVC write */
	struct tegra_hv_ivc_cookie *cookie;
};

static struct tegra_cpufreq_ivc_data ivc;

static bool set_speed = true;

bool hv_is_set_speed_supported(void)
{
	return set_speed;
}

static irqreturn_t hv_tegra_cpufreq_ivc_isr(int irq, void *dev_id)
{
	struct tegra_cpufreq_ivc_data *ivck =
			(struct tegra_cpufreq_ivc_data *)dev_id;

	if (tegra_hv_ivc_can_write(ivck->cookie))
		wake_up(&ivck->w_wq);

	if (tegra_hv_ivc_can_read(ivck->cookie))
		wake_up(&ivck->r_wq);

	return IRQ_HANDLED;
}

int __init parse_hv_dt_data(struct device_node *dn)
{
	int err = 0;
	uint32_t ivc_queue;
	struct device_node *hv_dn;
	struct tegra_cpufreq_ivc_data *ivck = &ivc;

	if (of_find_property(dn, "ivc_queue", NULL) == NULL) {
		pr_err("IVC queue not found. Disabling cpufreq set speed functionality\n");
		set_speed = false;
		return err;
	}

	hv_dn = of_parse_phandle(dn, "ivc_queue", 0);
	if (hv_dn == NULL) {
		pr_err("Failed to parse phandle of ivc prop\n");
		err = -EINVAL;
		goto err_out;
	}

	err = of_property_read_u32_index(dn, "ivc_queue", 1,
					&ivc_queue);

	if (err != 0) {
		pr_err("Failed to read IVC property ID\n");
		err = -EINVAL;
		goto err_out_free;
	}

	init_waitqueue_head(&ivck->r_wq);
	init_waitqueue_head(&ivck->w_wq);
	mutex_init(&ivck->mlock);
	ivck->cookie = tegra_hv_ivc_reserve(hv_dn, ivc_queue, NULL);

	if (IS_ERR_OR_NULL(ivck->cookie)) {
		pr_err("Failed to reserve ivc queue %d\n",
				ivc_queue);
		err = -EINVAL;
		goto err_out_free;
	}

	err = request_threaded_irq(ivck->cookie->irq,
				hv_tegra_cpufreq_ivc_isr,
				NULL,
				0, "hv-tegra-cpufreq", ivck);
	if (err) {
		tegra_hv_ivc_unreserve(ivck->cookie);
		err = -ENOMEM;
		goto err_out_free;
	}
	/* set ivc channel to invalid state */
	tegra_hv_ivc_channel_reset(ivck->cookie);

err_out:
	return err;

err_out_free:
	of_node_put(hv_dn);
	return err;
}

static bool tegra_cpufreq_ivc_can_write(struct tegra_hv_ivc_cookie *cookie)
{
	if (tegra_hv_ivc_can_write(cookie))
		return true;
	else {
		pr_info("\n %s:IVC Queue is Full\n", __func__);
		return false;
	}
}

static int tegra_cpufreq_tx_ivc_msg(uint32_t id, uint32_t len, void *msg_buf)
{
	struct tegra_cpufreq_ivc_data *ivck = &ivc;
	struct tegra_cpufreq_ivc_msg *ivc_msg = NULL;
	uint32_t size = sizeof(struct tegra_cpufreq_ivc_msg);
	int ret = 0;

	if (set_speed == false) {
		pr_warn("cpufreq-hv : Setting speed functionality not present\n");
		return -EINVAL;
	}

	if ((len > TEGRA_CPUFREQ_IVC_MSG_LEN) || (id > MAX_IVC_MSG_ID))
		return -EINVAL;

	mutex_lock(&ivck->mlock);

	ivc_msg = &ivck->cpufreq_ivc_msg;
	memset(&ivc_msg->buffer, 0, TEGRA_CPUFREQ_IVC_MSG_LEN);
	ivc_msg->msg_id = id;
	ivc_msg->len = len;
	memcpy(&ivc_msg->buffer, msg_buf, len);

	while (tegra_hv_ivc_channel_notified(ivck->cookie))
		/* Waiting for the channel to be ready */;

	if (!tegra_cpufreq_ivc_can_write(ivck->cookie))
		wait_event(ivck->w_wq,
			tegra_cpufreq_ivc_can_write(ivck->cookie));

	ret = tegra_hv_ivc_write(ivck->cookie,
				(const void *)ivc_msg,
				size);
	if (ret != size) {
		pr_err("\n%s: Write failed %d %d\n", __func__, size, ret);
		ret = -EINVAL;
	} else {
		ret = 0;
	}

	/*
	 * If the request is for cpu freq read, then need to read the respose
	 * also
	 */
	if (id == TEGRA_CPU_FREQ_GET_RATE) {
		if (!tegra_hv_ivc_can_read(ivck->cookie)) {
			ret = wait_event_timeout(ivck->r_wq,
					tegra_hv_ivc_can_read(ivck->cookie),
					msecs_to_jiffies(500));
			if (!ret) {
				pr_err("\n%s: cpufreq_hv ACK timeout\n",
					__func__);
				ret = -ETIMEDOUT;
				goto err_out;
			}
		}
		ret = tegra_hv_ivc_read(ivck->cookie, ivc_msg, size);
		memcpy(msg_buf, &ivc_msg->buffer, len);
		pr_debug("\n %s:%d rate_KHz:%d\n",
			__func__, __LINE__, ivc_msg->buffer.cpu_rate.rate_khz);

		if (ret != size) {
			pr_err("\n%s: ivc transaction read failed %d %d\n", __func__, size, ret);
			ret = -EINVAL;
		} else {
			ret = 0;
		}
	}

err_out:
	mutex_unlock(&ivck->mlock);
	return ret;
}

uint32_t t194_get_cpu_speed_hv(uint32_t cpu)
{
	int ret = 0;
	struct cpu_rate_msg cpu_rate;

	/* cpu mpidr is required for the bpmp server to know physical cpu */
	cpu_rate.mpidr = cpu_logical_map(cpu);

	ret = tegra_cpufreq_tx_ivc_msg(TEGRA_CPU_FREQ_GET_RATE,
					sizeof(cpu_rate),
					&cpu_rate);
	if (ret) {
		pr_err("\n%s: Error in getting rate for cpu:%d failed: ret:%d\n",
							__func__,
							cpu,
							ret);
		return 0; /* Return 0 as cpu frequency in error conditions */
	}
	return cpu_rate.rate_khz;

}

void t194_update_cpu_speed_hv(uint32_t rate, uint32_t cpu)
{
	int ret = 0;
	struct cpu_rate_msg cpu_rate = {0};

	//cpu_rate.cl = tegra18_logical_to_cluster(cpu);
	cpu_rate.rate_khz = rate;
	/* cpu mpidr is required for the bpmp server to know physical cpu */
	cpu_rate.mpidr = cpu_logical_map(cpu);


	ret = tegra_cpufreq_tx_ivc_msg(TEGRA_CPU_FREQ_SET_RATE,
					sizeof(cpu_rate),
					&cpu_rate);

	if (ret)
		pr_err("\n%s: Update cpu rate %dkHz for cluster:%d failed\n",
							__func__,
							cpu_rate.rate_khz,
							cpu_rate.cl);
}

void tegra_update_cpu_speed_hv(uint32_t rate, uint8_t cpu)
{
	int ret = 0;
	struct cpu_rate_msg cpu_rate;

	cpu_rate.cl = tegra18_logical_to_cluster(cpu);
	cpu_rate.rate_khz = rate;

	ret = tegra_cpufreq_tx_ivc_msg(TEGRA_CPU_FREQ_SET_RATE,
					sizeof(cpu_rate),
					&cpu_rate);

	if (ret)
		pr_err("\n%s: Update cpu rate %dkHz for cluster:%d failed\n",
							__func__,
							cpu_rate.rate_khz,
							cpu_rate.cl);
	return;
}
