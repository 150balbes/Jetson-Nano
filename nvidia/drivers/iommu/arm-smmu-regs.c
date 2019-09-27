/*
 * Copyright (c) 2018 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/iommu.h>
#include <linux/version.h>

#include "arm-smmu-regs.h"
#include "arm-smmu-debug.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)

/* TODO: Restructure code to remove handle global variable */
static struct smmu_debugfs_info *smmu_handle;

#define defreg(_name)				\
	{					\
		.name = __stringify(_name),	\
		.offset = ARM_SMMU_ ## _name,	\
	}
#define defreg_gr0(_name) defreg(GR0_ ## _name)

static const struct debugfs_reg32 arm_smmu_gr0_regs[] = {
	defreg_gr0(sCR0),
	defreg_gr0(ID0),
	defreg_gr0(ID1),
	defreg_gr0(ID2),
	defreg_gr0(sGFSR),
	defreg_gr0(sGFSYNR0),
	defreg_gr0(sGFSYNR1),
	defreg_gr0(sTLBGSTATUS),
	defreg_gr0(nsCR0),
	defreg_gr0(nsGFSR),
	defreg_gr0(nsGFSYNR0),
	defreg_gr0(nsGFSYNR1),
	defreg_gr0(nsTLBGSTATUS),
	defreg_gr0(PIDR2),
};

#define defreg_gnsr0(_name) defreg(GNSR0_ ## _name)

static const struct debugfs_reg32 arm_smmu_gnsr0_regs[] = {
	defreg_gnsr0(PMCNTENSET_0),
	defreg_gnsr0(PMCNTENCLR_0),
	defreg_gnsr0(PMINTENSET_0),
	defreg_gnsr0(PMINTENCLR_0),
	defreg_gnsr0(PMOVSCLR_0),
	defreg_gnsr0(PMOVSSET_0),
	defreg_gnsr0(PMCFGR_0),
	defreg_gnsr0(PMCR_0),
	defreg_gnsr0(PMCEID0_0),
	defreg_gnsr0(PMAUTHSTATUS_0),
	defreg_gnsr0(PMDEVTYPE_0)
};

#define defreg_cb(_name)			\
	{					\
		.name = __stringify(_name),	\
		.offset = ARM_SMMU_CB_ ## _name,\
	}

static const struct debugfs_reg32 arm_smmu_cb_regs[] = {
	defreg_cb(SCTLR),
	defreg_cb(TTBCR2),
	defreg_cb(TTBR0_LO),
	defreg_cb(TTBR0_HI),
	defreg_cb(TTBCR),
	defreg_cb(S1_MAIR0),
	defreg_cb(FSR),
	defreg_cb(FAR_LO),
	defreg_cb(FAR_HI),
	defreg_cb(FSYNR0),
};

static int smmu_master_show(struct seq_file *s, void *unused)
{
	int i;
	struct smmu_debugfs_master *master = s->private;
	struct iommu_fwspec *fwspec = master->dev->iommu_fwspec;

	for (i = 0; i < fwspec->num_ids; i++) {
		seq_printf(s, "streamids: % 3d ",
				fwspec->ids[i] & smmu_handle->streamid_mask);
	}
	seq_printf(s, "\n");
	for (i = 0; i < fwspec->num_ids; i++) {
		seq_printf(s, "smrs: % 3d ", master->smendx[i]);
	}
	seq_printf(s, "\n");

	return 0;
}

static int smmu_master_open(struct inode *inode, struct file *file)
{
	return single_open(file, smmu_master_show, inode->i_private);
}

static const struct file_operations smmu_master_fops = {
	.open           = smmu_master_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

void arm_smmu_debugfs_add_master(struct device *dev, u8 *cbndx, u16 smendx[])
{
	struct smmu_debugfs_master *master;
	struct dentry *dent;
	char name[] = "cb000";
	char target[] = "../../cb000";

	dent = debugfs_create_dir(dev_name(dev), smmu_handle->masters_root);
	if (!dent)
		return;

	master = kmalloc(sizeof(*master), GFP_KERNEL);
	master->dev = dev;
	master->smendx = smendx;
	master->dent = dent;

	// TODO create the streamids file
	debugfs_create_file("streamids", 0444, dent, master,
							&smmu_master_fops);
	debugfs_create_u8("cbndx", 0444, dent, cbndx);
	sprintf(name, "cb%03d", *cbndx);
	sprintf(target, "../../cb%03d", *cbndx);
	debugfs_create_symlink(name, dent, target);

	list_add_tail(&master->node, &smmu_handle->masters_list);
}

void arm_smmu_debugfs_remove_master(struct device *dev)
{
	struct smmu_debugfs_master *master = NULL;

	list_for_each_entry(master, &smmu_handle->masters_list, node) {
		if (master->dev == dev)
			break;
	}

	if (master != NULL) {
		debugfs_remove_recursive(master->dent);
		list_del(&master->node);
		kfree(master);
	}
}

/*
 * TODO: Cbs downstream have the ability to print out iova-to-phys
 * and a dump of their page tables
 */
static void debugfs_create_smmu_cb(struct smmu_debugfs_info *smmu, u8 cbndx)
{
	struct dentry *dent;
	char name[] = "cb000";
	struct debugfs_regset32	*cb;

	sprintf(name, "cb%03d", cbndx);
	dent = debugfs_create_dir(name, smmu->debugfs_root);
	if (!dent)
		return;

	cb = smmu->regset + 1 + cbndx;
	cb->regs = arm_smmu_cb_regs;
	cb->nregs = ARRAY_SIZE(arm_smmu_cb_regs);
	cb->base = smmu->base + (smmu->size >> 1) +
		cbndx * (1 << smmu->pgshift);
	debugfs_create_regset32("regdump", S_IRUGO, dent, cb);
}

static int smmu_reg32_debugfs_set(void *data, u64 val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	writel(val, (smmu_handle->base + regs->offset));
	return 0;
}

static int smmu_reg32_debugfs_get(void *data, u64 *val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	*val = readl(smmu_handle->base + regs->offset);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(smmu_reg32_debugfs_fops,
			smmu_reg32_debugfs_get,
			smmu_reg32_debugfs_set, "%08llx\n");

static int smmu_perf_regset_debugfs_set(void *data, u64 val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	writel(val, (smmu_handle->perf_regset->base + regs->offset));
	return 0;
}

static int smmu_perf_regset_debugfs_get(void *data, u64 *val)
{
	struct debugfs_reg32 *regs = (struct debugfs_reg32 *)data;

	*val = readl(smmu_handle->perf_regset->base + regs->offset);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(smmu_perf_regset_debugfs_fops,
			smmu_perf_regset_debugfs_get,
			smmu_perf_regset_debugfs_set, "%08llx\n");

void arm_smmu_regs_debugfs_delete(struct smmu_debugfs_info *smmu)
{
	int i;

	if (smmu->regset) {
		const struct debugfs_reg32 *regs = smmu->regset->regs;

		regs += ARRAY_SIZE(arm_smmu_gr0_regs);
		for (i = 0; i < 4 * smmu->num_context_banks; i++)
			kfree(regs[i].name);

		kfree(smmu->regset);
	}

	if (smmu->perf_regset) {
		const struct debugfs_reg32 *regs = smmu->perf_regset->regs;

		i = ARRAY_SIZE(arm_smmu_gnsr0_regs);
		for (; i < smmu->perf_regset->nregs ; i++)
			kfree(regs[i].name);

		kfree(smmu->perf_regset);
		smmu->perf_regset = NULL;
	}

	debugfs_remove_recursive(smmu->debugfs_root);
}

int arm_smmu_regs_debugfs_create(struct smmu_debugfs_info *smmu)
{
	int i;
	struct debugfs_reg32 *regs;
	size_t bytes;
	struct dentry *dent_gr, *dent_gnsr;

	smmu_handle = smmu;

	if (!smmu->debugfs_root)
		return -1;

	dent_gr = debugfs_create_dir("gr", smmu->debugfs_root);
	if (!dent_gr)
		goto err_out;

	dent_gnsr = debugfs_create_dir("gnsr", smmu->debugfs_root);
	if (!dent_gnsr)
		goto err_out;

	smmu->masters_root = debugfs_create_dir("masters", smmu->debugfs_root);
	if (!smmu->masters_root)
		goto err_out;

	smmu->cb_root = debugfs_create_dir("context_banks", smmu->debugfs_root);
	if (!smmu->cb_root)
		goto err_out;

	bytes = (smmu->num_context_banks + 1) * sizeof(*smmu->regset);
	bytes += ARRAY_SIZE(arm_smmu_gr0_regs) * sizeof(*regs);
	bytes += 4 * smmu->num_context_banks * sizeof(*regs);
	smmu->regset = kzalloc(bytes, GFP_KERNEL);
	if (!smmu->regset)
		goto err_out;

	smmu->regset->base = smmu->base;
	smmu->regset->nregs = ARRAY_SIZE(arm_smmu_gr0_regs) +
		4 * smmu->num_context_banks;
	smmu->regset->regs = (struct debugfs_reg32 *)(smmu->regset +
						smmu->num_context_banks + 1);
	regs = (struct debugfs_reg32 *)smmu->regset->regs;
	for (i = 0; i < ARRAY_SIZE(arm_smmu_gr0_regs); i++) {
		regs->name = arm_smmu_gr0_regs[i].name;
		regs->offset = arm_smmu_gr0_regs[i].offset;
		regs++;
	}

	for (i = 0; i < smmu->num_context_banks; i++) {
		regs->name = kasprintf(GFP_KERNEL, "GR0_SMR%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GR0_SMR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GR0_S2CR%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GR0_S2CR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GR1_CBAR%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = (1 << smmu->pgshift) + ARM_SMMU_GR1_CBAR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GR1_CBA2R%03d", i);
		if (!regs->name)
			goto err_out;
		regs->offset = (1 << smmu->pgshift) + ARM_SMMU_GR1_CBA2R(i);
		regs++;
	}

	regs = (struct debugfs_reg32 *)smmu->regset->regs;
	for (i = 0; i < smmu->regset->nregs; i++) {
		debugfs_create_file(regs->name, S_IRUGO | S_IWUSR,
				dent_gr, regs, &smmu_reg32_debugfs_fops);
		regs++;
	}

	debugfs_create_regset32("regdump", S_IRUGO, smmu->debugfs_root,
				smmu->regset);

	bytes = sizeof(*smmu->perf_regset);
	bytes += ARRAY_SIZE(arm_smmu_gnsr0_regs) * sizeof(*regs);
	/*
	 * Account the number of bytes for two sets of
	 * counter group registers
	 */
	bytes += 2 * PMCG_SIZE * sizeof(*regs);
	/*
	 * Account the number of bytes for two sets of
	 * event counter registers
	 */
	bytes += 2 * PMEV_SIZE * sizeof(*regs);

	/* Allocate memory for Perf Monitor registers */
	smmu->perf_regset =  kzalloc(bytes, GFP_KERNEL);
	if (!smmu->perf_regset)
		goto err_out;

	/*
	 * perf_regset base address is placed at offset (3 * smmu_pagesize)
	 * from smmu->base address
	 */
	smmu->perf_regset->base = smmu->base + 3 * (1 << smmu->pgshift);
	smmu->perf_regset->nregs = ARRAY_SIZE(arm_smmu_gnsr0_regs) +
		2 * PMCG_SIZE + 2 * PMEV_SIZE;
	smmu->perf_regset->regs =
		(struct debugfs_reg32 *)(smmu->perf_regset + 1);

	regs = (struct debugfs_reg32 *)smmu->perf_regset->regs;

	for (i = 0; i < ARRAY_SIZE(arm_smmu_gnsr0_regs); i++) {
		regs->name = arm_smmu_gnsr0_regs[i].name;
		regs->offset = arm_smmu_gnsr0_regs[i].offset;
		regs++;
	}

	for (i = 0; i < PMEV_SIZE; i++) {
		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMEVTYPER%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMEVTYPER(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMEVCNTR%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMEVCNTR(i);
		regs++;
	}

	for (i = 0; i < PMCG_SIZE; i++) {
		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMCGCR%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMCGCR(i);
		regs++;

		regs->name = kasprintf(GFP_KERNEL, "GNSR0_PMCGSMR%d_0", i);
		if (!regs->name)
			goto err_out;
		regs->offset = ARM_SMMU_GNSR0_PMCGSMR(i);
		regs++;
	}

	regs = (struct debugfs_reg32 *)smmu->perf_regset->regs;
	for (i = 0; i < smmu->perf_regset->nregs; i++) {
		debugfs_create_file(regs->name, S_IRUGO | S_IWUSR,
			dent_gnsr, regs, &smmu_perf_regset_debugfs_fops);
		regs++;
	}

	for (i = 0; i < smmu->num_context_banks; i++)
		debugfs_create_smmu_cb(smmu, i);

	INIT_LIST_HEAD(&smmu_handle->masters_list);

	return 0;

err_out:
	arm_smmu_regs_debugfs_delete(smmu);
	return -1;
}

#endif
