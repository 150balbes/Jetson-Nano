/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/vgpu/vgpu_ivc.h>
#include <nvgpu/clk_arb.h>

#include "gk20a/gk20a.h"
#include "clk_vgpu.h"
#include "ctrl/ctrlclk.h"

static unsigned long vgpu_clk_get_rate(struct gk20a *g, u32 api_domain)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gpu_clk_rate_params *p = &msg.params.gpu_clk_rate;
	int err;
	unsigned long ret = 0;

	nvgpu_log_fn(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		msg.cmd = TEGRA_VGPU_CMD_GET_GPU_CLK_RATE;
		msg.handle = vgpu_get_handle(g);
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		err = err ? err : msg.ret;
		if (err)
			nvgpu_err(g, "%s failed - %d", __func__, err);
		else
			/* return frequency in Hz */
			ret = p->rate * 1000;
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		nvgpu_err(g, "unsupported clock: %u", api_domain);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return ret;
}

static int vgpu_clk_set_rate(struct gk20a *g,
				u32 api_domain, unsigned long rate)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gpu_clk_rate_params *p = &msg.params.gpu_clk_rate;
	int err = -EINVAL;

	nvgpu_log_fn(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		msg.cmd = TEGRA_VGPU_CMD_SET_GPU_CLK_RATE;
		msg.handle = vgpu_get_handle(g);

		/* server dvfs framework requires frequency in kHz */
		p->rate = (u32)(rate / 1000);
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		err = err ? err : msg.ret;
		if (err)
			nvgpu_err(g, "%s failed - %d", __func__, err);
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		nvgpu_err(g, "unsupported clock: %u", api_domain);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return err;
}

static unsigned long vgpu_clk_get_maxrate(struct gk20a *g, u32 api_domain)
{
	unsigned long *freqs;
	int num_freqs = 0;
	int err;
	unsigned long ret = 0;

	nvgpu_log_fn(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		err = vgpu_clk_get_freqs(g, &freqs, &num_freqs);
		if (err == 0) {
			/* return freq in Hz */
			ret = freqs[num_freqs - 1];
		}
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return ret;
}

static int vgpu_clk_get_round_rate(struct gk20a *g, u32 api_domain,
	unsigned long rate_target, unsigned long *rounded_rate)
{
	int err = -EINVAL;

	nvgpu_log_fn(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		*rounded_rate = rate_target;
		err = 0;
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return err;
}

static int vgpu_clk_get_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz)
{
	unsigned long *freqs;
	int num_freqs = 0;
	int err = -EINVAL;

	nvgpu_log_fn(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		err = vgpu_clk_get_freqs(g, &freqs, &num_freqs);
		if (err == 0) {
			/* return freq in MHz */
			*min_mhz = (u16)(freqs[0] / 1000000);
			*max_mhz = (u16)(freqs[num_freqs - 1] / 1000000);
		}
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return err;
}

static int vgpu_clk_get_f_points(struct gk20a *g,
		u32 api_domain, u32 *num_points, u16 *freqs_mhz)
{
	unsigned long *freqs;
	int num_freqs = 0;
	u32 i;
	int err = -EINVAL;

	nvgpu_log_fn(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		err = vgpu_clk_get_freqs(g, &freqs, &num_freqs);
		if (err) {
			return err;
		}

		if (num_points == NULL) {
			return -EINVAL;
		}

		if (*num_points != 0U) {
			if (freqs == NULL || (*num_points > (u32)num_freqs)) {
				return -EINVAL;
			}
		}

		if (*num_points == 0) {
			*num_points = num_freqs;
		} else {
			for (i = 0; i < *num_points; i++) {
				/* return freq in MHz */
				freqs_mhz[i] = (u16)(freqs[i] / 1000000);
			}
		}
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		break;
	}

	return err;
}

void vgpu_init_clk_support(struct gk20a *g)
{
	g->ops.clk.get_rate = vgpu_clk_get_rate;
	g->ops.clk.set_rate = vgpu_clk_set_rate;
	g->ops.clk.get_maxrate = vgpu_clk_get_maxrate;
	g->ops.clk.clk_get_round_rate = vgpu_clk_get_round_rate;
	g->ops.clk.get_clk_range = vgpu_clk_get_range;
	g->ops.clk.clk_domain_get_f_points = vgpu_clk_get_f_points;
	g->ops.clk.measure_freq = nvgpu_clk_measure_freq;
}

int vgpu_clk_get_freqs(struct gk20a *g, unsigned long **freqs_out,
		int *num_freqs)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_get_gpu_freq_table_params *p =
					&msg.params.get_gpu_freq_table;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	u32 *freqs;
	int err = 0;
	void *handle = NULL;
	size_t oob_size;
	unsigned int i;

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&priv->vgpu_clk_get_freq_lock);

	if (priv->freqs != NULL) {
		goto done;
	}

	msg.cmd = TEGRA_VGPU_CMD_GET_GPU_FREQ_TABLE;
	msg.handle = vgpu_get_handle(g);

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(g, "%s failed - %d", __func__, err);
		goto done;
	}

	handle = vgpu_ivc_oob_get_ptr(vgpu_ivc_get_server_vmid(),
			TEGRA_VGPU_QUEUE_CMD, (void **)&freqs, &oob_size);
	if (!handle) {
		nvgpu_err(g, "failed to get ivm handle");
		err = -EINVAL;
		goto done;
	}

	priv->freqs = nvgpu_kzalloc(g, sizeof(*priv->freqs) * (p->num_freqs));
	if (!priv->freqs) {
		nvgpu_err(g, "failed to allocate memory");
		vgpu_ivc_oob_put_ptr(handle);
		err = -ENOMEM;
		goto done;
	}
	priv->num_freqs = p->num_freqs;

	for (i = 0; i < priv->num_freqs; i++) {
		/* store frequency in Hz */
		priv->freqs[i] = (unsigned long)(freqs[i] * 1000);
	}

	vgpu_ivc_oob_put_ptr(handle);

done:
	if (err == 0) {
		*num_freqs = priv->num_freqs;
		*freqs_out = priv->freqs;
	}

	nvgpu_mutex_release(&priv->vgpu_clk_get_freq_lock);

	return err;
}

int vgpu_clk_cap_rate(struct gk20a *g, unsigned long rate)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gpu_clk_rate_params *p = &msg.params.gpu_clk_rate;
	int err = 0;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_CAP_GPU_CLK_RATE;
	msg.handle = vgpu_get_handle(g);
	p->rate = (u32)rate;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(g, "%s failed - %d", __func__, err);
		return err;
	}

	return 0;
}
