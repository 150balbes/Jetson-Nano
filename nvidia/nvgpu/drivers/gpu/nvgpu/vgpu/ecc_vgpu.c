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

#include <nvgpu/kmem.h>
#include <nvgpu/vgpu/vgpu_ivc.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/errno.h>

#include "vgpu/ecc_vgpu.h"

int vgpu_ecc_get_info(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_ecc_info_params *p = &msg.params.ecc_info;
	struct tegra_vgpu_ecc_info_entry *entry;
	struct vgpu_ecc_stat *stats;
	void *handle;
	int err, i, count;
	size_t oob_size;

	msg.cmd = TEGRA_VGPU_CMD_GET_ECC_INFO;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (unlikely(err)) {
		nvgpu_err(g, "vgpu get_ecc_info failed, err=%d", err);
		return err;
	}

	count = p->ecc_stats_count;

	handle = vgpu_ivc_oob_get_ptr(vgpu_ivc_get_server_vmid(),
					TEGRA_VGPU_QUEUE_CMD,
					(void **)&entry, &oob_size);
	if (unlikely(!handle))
		return -EINVAL;

	if (unlikely(oob_size < count * sizeof(*entry))) {
		err = -E2BIG;
		goto out;
	}

	stats = nvgpu_kzalloc(g, count * sizeof(*stats));
	if (unlikely(!stats)) {
		err = -ENOMEM;
		goto out;
	}

	for (i = 0; i < count; i++) {
		stats[i].ecc_id = entry[i].ecc_id;
		strncpy(stats[i].name, entry[i].name,
			NVGPU_ECC_STAT_NAME_MAX_SIZE);
	}

	priv->ecc_stats = stats;
	priv->ecc_stats_count = count;
out:
	vgpu_ivc_oob_put_ptr(handle);
	return err;
}

void vgpu_ecc_remove_info(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	priv->ecc_stats_count = 0;

	if (priv->ecc_stats) {
		nvgpu_kfree(g, priv->ecc_stats);
		priv->ecc_stats = NULL;
	}
}
