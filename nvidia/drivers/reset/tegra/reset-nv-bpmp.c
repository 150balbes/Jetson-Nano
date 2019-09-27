/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>

static int bpmp_send_reset_message(u32 cmd, u32 reset_id)
{
	struct mrq_reset_request req = { .cmd = cmd, .reset_id = reset_id };
	int err;

	err = tegra_bpmp_send_receive(MRQ_RESET, &req, sizeof(req), NULL, 0);
	if (err < 0)
		return -EINVAL;

	return 0;
}

int bpmp_get_max_reset_id(uint32_t *max_id)
{
	struct mrq_reset_request req = { .cmd = CMD_RESET_GET_MAX_ID };
	struct mrq_reset_response resp;
	int r;

	r = tegra_bpmp_send_receive(MRQ_RESET, &req, sizeof(req),
			&resp, sizeof(resp));
	if (r)
		return r;

	*max_id = resp.reset_get_max_id.max_id;

	return 0;
}

static int bpmp_reset_assert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return bpmp_send_reset_message(CMD_RESET_ASSERT, id);
}
static int bpmp_reset_deassert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	return bpmp_send_reset_message(CMD_RESET_DEASSERT, id);
}
static int bpmp_reset_module(struct reset_controller_dev *rcdev,
			  unsigned long id)
{
	return bpmp_send_reset_message(CMD_RESET_MODULE, id);
}

static struct reset_control_ops bpmp_reset_ops = {
	.assert		= bpmp_reset_assert,
	.deassert	= bpmp_reset_deassert,
	.reset		= bpmp_reset_module,
};

int bpmp_register_reset(int num_resets, struct platform_device *pdev)
{
	struct reset_controller_dev *rcdev;

	rcdev = devm_kzalloc(&pdev->dev, sizeof(*rcdev), GFP_KERNEL);
	if (!rcdev)
		return -ENOMEM;

	rcdev->owner = THIS_MODULE;
	rcdev->nr_resets = num_resets;
	rcdev->ops = &bpmp_reset_ops;
	rcdev->of_node = pdev->dev.of_node;

	dev_info(&pdev->dev, "registered %d resets.\n", num_resets);

	return reset_controller_register(rcdev);
}
