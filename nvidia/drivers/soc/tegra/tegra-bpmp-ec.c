/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/bpmp_abi.h>

static uint32_t hsm_id;

static int bpmp_get_ec_status(uint32_t hsm_id,
			      struct cmd_ec_status_get_response *ec_status)
{
	struct mrq_ec_request req;
	struct mrq_ec_response reply;
	int ret;

	req.cmd_id = CMD_EC_STATUS_GET;
	req.ec_status_get.ec_hsm_id = hsm_id;

	ret = tegra_bpmp_send_receive(MRQ_EC, &req, sizeof(req), &reply,
				      sizeof(reply));
	if (ret < 0)
		return ret;

	*ec_status = reply.ec_status_get;
	return 0;
}

static int ec_status_show(struct seq_file *s, void *data)
{
	int i, ret;
	uint32_t hid = hsm_id;
	struct cmd_ec_status_get_response ec_status;
	uint32_t *flags = &ec_status.ec_status_flags;

	ret = bpmp_get_ec_status(hid, &ec_status);
	if (ret) {
		seq_printf(s, "HSM id %u\nError: ", hid);
		seq_printf(s, "failed (%d)\n", ret);
		return 0;
	}

	seq_printf(s, "HSM id %u\nError: ", ec_status.ec_hsm_id);
	if (*flags & EC_STATUS_FLAG_NO_ERROR) {
		seq_printf(s, "none\n");
		return 0;
	}

	seq_printf(s, "idx %u (%s%s)\n", ec_status.error_idx,
		   *flags & EC_STATUS_FLAG_LATENT_ERROR ? "latent" : "mission",
		   *flags & EC_STATUS_FLAG_LAST_ERROR ? ", last" : "");
	seq_printf(s, "type %u\n", ec_status.error_type);

	for (i = 0; i < ec_status.error_desc_num; i++) {
		union ec_err_desc *desc = &ec_status.error_descs[i];

		switch (ec_status.error_type) {
		case EC_ERR_TYPE_CLOCK_MONITOR:
			if (desc->fmon_desc.desc_flags & EC_DESC_FLAG_NO_ID) {
				seq_printf(s, "id unknown\n");
				break;
			}
			seq_printf(s, "id %u faults 0x%x err %d flags 0x%x\n",
				   desc->fmon_desc.fmon_clk_id,
				   desc->fmon_desc.fmon_faults,
				   desc->fmon_desc.fmon_access_error,
				   desc->fmon_desc.desc_flags);
			break;
		case EC_ERR_TYPE_VOLTAGE_MONITOR:
			if (desc->vmon_desc.desc_flags & EC_DESC_FLAG_NO_ID) {
				seq_printf(s, "id unknown\n");
				break;
			}
			seq_printf(s, "id %u faults 0x%x err %d flags 0x%x\n",
				   desc->vmon_desc.vmon_adc_id,
				   desc->vmon_desc.vmon_faults,
				   desc->vmon_desc.vmon_access_error,
				   desc->vmon_desc.desc_flags);
			break;
		case EC_ERR_TYPE_REGISTER_PARITY:
			if (desc->reg_parity_desc.desc_flags &
			    EC_DESC_FLAG_NO_ID) {
				seq_printf(s, "id unknown\n");
				break;
			}
			seq_printf(s, "id %u group %u flags 0x%x\n",
				   desc->reg_parity_desc.reg_id,
				   desc->reg_parity_desc.reg_group,
				   desc->reg_parity_desc.desc_flags);
			break;
		default:
			if (desc->simple_desc.desc_flags & EC_DESC_FLAG_NO_ID) {
				seq_printf(s, "id unknown\n");
				break;
			}
			seq_printf(s, "id %u flags 0x%x\n",
				   desc->simple_desc.err_source_id,
				   desc->simple_desc.desc_flags);
			break;
		}
	}
	return 0;
}

static int ec_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, ec_status_show, inode->i_private);
}

static const struct file_operations ec_status_fops = {
	.open		= ec_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init bpmp_ec_debugfs_init(void)
{
	struct dentry *d, *dir;

	dir = debugfs_create_dir("tegra_ec", NULL);
	if (!dir)
		return -ENOMEM;

	d = debugfs_create_file("ec_status", S_IRUGO, dir, NULL,
				&ec_status_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_u32("hsm_id", S_IRUGO|S_IWUSR, dir, &hsm_id);
	if (!d)
		return -ENOMEM;

	return 0;
}
late_initcall(bpmp_ec_debugfs_init);
