/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#define _BullseyeCoverage 1

#include <soc/tegra/camrtc-channels.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/seq_buf.h>
#include <linux/slab.h>
#include <linux/tegra-camera-rtcpu.h>

#define COV_MEM_SZ (24U*1024U) /* 24K */

/*
 * Private driver data structure
 */

struct tegra_rtcpu_coverage {
	struct device *dev;

	/* memory */
	void *coverage_memory;
	u32 coverage_memory_size;
	dma_addr_t dma_handle;

	struct dentry *debugfs_root;
};

/*
 * Coverage memory
 */

static int rtcpu_coverage_setup_memory(struct tegra_rtcpu_coverage *coverage)
{
	struct device *dev = coverage->dev;
	void *coverage_memory = NULL;
	size_t mem_size;
	dma_addr_t dma_addr = 0;

	coverage->dma_handle = 0;
	mem_size = COV_MEM_SZ;

	coverage_memory = dma_alloc_coherent(dev, mem_size, &dma_addr,
		GFP_KERNEL | __GFP_ZERO);
	if (coverage_memory == NULL) {
		dev_err(dev, "rtcpu_coverage_setup_memory ERROR\r\n");
		return -ENOMEM;
	}

	/* Save the information */
	coverage->coverage_memory = coverage_memory;
	coverage->coverage_memory_size = mem_size;
	coverage->dma_handle = dma_addr;
	return 0;
}

static void rtcpu_coverage_init_memory(struct tegra_rtcpu_coverage *coverage)
{
	struct camrtc_coverage_memory_header *header =
		(struct camrtc_coverage_memory_header *)
			coverage->coverage_memory;

	/* header */
	header->revision = 1;
	header->coverage_buffer_size = coverage->coverage_memory_size;
	header->coverage_total_bytes = 0;
	header->length = sizeof(struct camrtc_coverage_memory_header);

	/* validate */
	header->signature = CAMRTC_TAG_NV_COVERAGE;
}

/*
 * Debugfs
 */

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
	static int _fops_ ## _open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, _show_, inode->i_private); \
	} \
	static const struct file_operations _fops_ = { \
		.open = _fops_ ## _open, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.release = single_release }

static int rtcpu_coverage_debugfs_gcov_read(
		struct seq_file *file, void *data)
{
	struct tegra_rtcpu_coverage *coverage = file->private;
	struct camrtc_coverage_memory_header *cov_header;
	u8 *cov_data;
	int i = 0;

	cov_header = (struct camrtc_coverage_memory_header *)
			(coverage->coverage_memory);
	cov_data = (u8 *)coverage->coverage_memory +
		sizeof(struct camrtc_coverage_memory_header);

	/* RTCPU will tell us how many bytes to read */
	for (i = 0; i < cov_header->coverage_total_bytes; i++)
		seq_putc(file, cov_data[i]);

	return 0;
}

DEFINE_SEQ_FOPS(rtcpu_coverage_debugfs_gcov, rtcpu_coverage_debugfs_gcov_read);

static void rtcpu_coverage_debugfs_deinit(struct tegra_rtcpu_coverage *coverage)
{
	debugfs_remove_recursive(coverage->debugfs_root);
}

static int rtcpu_coverage_debugfs_init(struct tegra_rtcpu_coverage *coverage)
{
	struct dentry *entry;
	int err = 0;

	coverage->debugfs_root = debugfs_create_dir("tegra_rtcpu_coverage",
							NULL);
	if (IS_ERR_OR_NULL(coverage->debugfs_root)) {
		if (IS_ERR(coverage->debugfs_root))
			err = PTR_ERR(coverage->debugfs_root);
		else
			err = -ENODEV;

		return err;
	}
	entry = debugfs_create_file("cov_data", S_IRUGO,
		coverage->debugfs_root, coverage, &rtcpu_coverage_debugfs_gcov);
	if (IS_ERR_OR_NULL(entry)) {
		if (IS_ERR(entry))
			err = PTR_ERR(entry);
		else
			err = -ENODEV;

		debugfs_remove_recursive(coverage->debugfs_root);
		return err;
	}

	return err;
}

/*
 * Init/Cleanup
 * Cleanup first so init can call it in the event of an error.
 */

void tegra_rtcpu_coverage_destroy(struct tegra_rtcpu_coverage *coverage)
{
	if (IS_ERR_OR_NULL(coverage))
		return;
	rtcpu_coverage_debugfs_deinit(coverage);
	dma_free_coherent(coverage->dev, coverage->coverage_memory_size,
		coverage->coverage_memory, coverage->dma_handle);
	kfree(coverage);
}
EXPORT_SYMBOL(tegra_rtcpu_coverage_destroy);

struct tegra_rtcpu_coverage *tegra_rtcpu_coverage_create(struct device *dev)
{
	struct tegra_rtcpu_coverage *coverage;
	int ret;

	coverage = kzalloc(sizeof(*coverage), GFP_KERNEL);
	if (unlikely(coverage == NULL))
		return NULL;

	coverage->dev = dev;

	/* Get the coverage memory */
	ret = rtcpu_coverage_setup_memory(coverage);
	if (ret < 0) {
		dev_err(dev, "Coverage memory setup failed: %d\n", ret);
		kfree(coverage);
		return NULL;
	}

	/* Initialize the coverage memory */

	rtcpu_coverage_init_memory(coverage);
	/* Debugfs */
	ret = rtcpu_coverage_debugfs_init(coverage);
	if (ret < 0) {
		tegra_rtcpu_coverage_destroy(coverage);
		return NULL;
	}
	dev_dbg(dev, "Coverage buffer configured at IOVA=0x%08x\n",
		(u32)coverage->dma_handle);

	return coverage;
}
EXPORT_SYMBOL(tegra_rtcpu_coverage_create);

int tegra_rtcpu_coverage_boot_sync(struct tegra_rtcpu_coverage *coverage)
{
	int ret = tegra_camrtc_iovm_setup(coverage->dev,
			coverage->dma_handle);

	if (ret == 0)
		return 0;

	dev_dbg(coverage->dev, "RTCPU coverage: IOVM setup error: %d\n", ret);
	dev_dbg(coverage->dev,
		"(expected if RTCPU was not built with coverage enabled)\n");
	return -EIO;
}
EXPORT_SYMBOL(tegra_rtcpu_coverage_boot_sync);

MODULE_DESCRIPTION("NVIDIA Tegra RTCPU coverage driver");
MODULE_LICENSE("GPL v2");
