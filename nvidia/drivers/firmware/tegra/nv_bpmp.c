/*
 * Copyright (c) 2013-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include <linux/tegra-firmwares.h>
#include <linux/tegra-ivc.h>
#include <linux/version.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra_powergate.h>
#include <linux/slab.h>
#include "bpmp.h"

static void *hv_virt_base;
static struct device *device;
char firmware_tag[sizeof(struct mrq_query_fw_tag_response)];

static int bpmp_get_fwtag(void)
{
	const size_t sz = sizeof(struct mrq_query_fw_tag_response);
	struct mrq_query_abi_request abi_req = { .mrq = MRQ_QUERY_FW_TAG };
	struct mrq_query_abi_response abi_resp;
	int r;
	char *virt;
	dma_addr_t phys;

	r = tegra_bpmp_send_receive(MRQ_QUERY_ABI, &abi_req,
		 sizeof(abi_req), &abi_resp, sizeof(abi_resp));
	if (r) {
		dev_err(device, "ABI query failed! %d\n", r);
		goto exit;
	}
	/*
	 * If MRQ_QUERY_FW_TAG is not supported, read tag using
	 * MRQ_QUERY_TAG
	 */
	if (abi_resp.status) {
		virt = tegra_bpmp_alloc_coherent(sz + 1, &phys,
							GFP_KERNEL);
		if (virt)
			r = tegra_bpmp_send_receive(MRQ_QUERY_TAG,
				&phys, sizeof(phys), NULL, 0);
	} else {
		virt = kmalloc(sz + 1, GFP_KERNEL);
		if (virt)
			r = tegra_bpmp_send_receive(MRQ_QUERY_FW_TAG,
				NULL, 0, virt, sz);
	}
	if (!virt) {
		r = -ENOMEM;
		goto exit;
	}

	if (!r) {
		/* Copy to global buffer */
		memcpy(firmware_tag, virt, sz);
		virt[sz] = '\0';
		dev_info(device, "firmware tag is %s\n", virt);
	} else {
		dev_err(device, "TAG query failed! %d\n", r);
	}

	if (abi_resp.status)
		tegra_bpmp_free_coherent(sz + 1, virt, phys);
	else
		kfree(virt);
exit:
	return r;
}

static ssize_t bpmp_version(struct device *dev, char *data, size_t size)
{
	return snprintf(data, size, "firmware tag %*.*s",
		 (int)sizeof(firmware_tag), (int)sizeof(firmware_tag),
		 firmware_tag);
}

static void *bpmp_get_virt_for_alloc(void *virt, dma_addr_t phys)
{
	if (hv_virt_base)
		return hv_virt_base + phys;

	return virt;
}

void *tegra_bpmp_alloc_coherent(size_t size, dma_addr_t *phys,
		gfp_t flags)
{
	void *virt;

	if (!device)
		return NULL;

	virt = dma_alloc_coherent(device, size, phys,
			flags);

	virt = bpmp_get_virt_for_alloc(virt, *phys);

	return virt;
}
EXPORT_SYMBOL(tegra_bpmp_alloc_coherent);

static void *bpmp_get_virt_for_free(void *virt, dma_addr_t phys)
{
	if (hv_virt_base)
		return (void *)(virt - hv_virt_base);

	return virt;
}

void tegra_bpmp_free_coherent(size_t size, void *vaddr,
		dma_addr_t phys)
{
	if (!device) {
		pr_err("device not found\n");
		return;
	}

	vaddr = bpmp_get_virt_for_free(vaddr, phys);

	dma_free_coherent(device, size, vaddr, phys);
}
EXPORT_SYMBOL(tegra_bpmp_free_coherent);

static struct syscore_ops bpmp_syscore_ops = {
	.suspend = tegra_bpmp_suspend,
	.resume = tegra_bpmp_resume,
};

int __bpmp_do_ping(void)
{
	int ret;
	int challenge = 1;
	int reply;

	ret = tegra_bpmp_send_receive_atomic(MRQ_PING,
			&challenge, sizeof(challenge), &reply, sizeof(reply));
	if (ret)
		return ret;

	if (reply != challenge * 2)
		return -EINVAL;

	return 0;
}

static int bpmp_do_ping(void)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);
	ret = __bpmp_do_ping();
	local_irq_restore(flags);
	pr_info("bpmp: ping status is %d\n", ret);

	return ret;
}

static struct platform_device bpmp_tty = {
	.name = "tegra-bpmp-tty",
	.id = -1,
};

static int bpmp_init_powergate(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 pd_cells;

	if (of_property_read_u32(np, "#power-domain-cells", &pd_cells))
		return 0;

	if (pd_cells != 1) {
		dev_err(&pdev->dev, "%s #power-domain-cells must be 1\n",
			np->full_name);
		return -ENODEV;
	}

	return tegra_bpmp_init_powergate(pdev);
}

static int bpmp_setup_allocator(struct device *dev)
{
	uint32_t mempool_id;
	int ret;
	struct tegra_hv_ivm_cookie *ivm;
	void *virt_base;
	int flags;

	ret = of_property_read_u32_index(dev->of_node, "mempool", 0,
			&mempool_id);
	if (ret) {
		dev_err(dev, "failed to read mempool id (%d)\n", ret);
		return ret;
	}

	ivm = tegra_hv_mempool_reserve(mempool_id);
	if (IS_ERR_OR_NULL(ivm)) {
		if (!IS_ERR(ivm))
			dev_err(dev, "No mempool found\n");
		return -ENOMEM;
	}

	dev_info(dev, "Found mempool with id %u\n", mempool_id);
	dev_info(dev, "ivm %pa\n", &ivm->ipa);

	virt_base = ioremap_cache(ivm->ipa, ivm->size);

	flags = DMA_MEMORY_EXCLUSIVE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	flags |= DMA_MEMORY_NOMAP;
#endif
	ret = dma_declare_coherent_memory(dev, ivm->ipa, 0, ivm->size,
			flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	if (!(ret & DMA_MEMORY_NOMAP)) {
		dev_err(dev, "dma_declare_coherent_memory failed (%x)\n", ret);
		return ret;
	}
#endif

	hv_virt_base = virt_base;

	return 0;
}

static int bpmp_clk_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *sclk;

	sclk = devm_clk_get(dev, "sclk");
	if (IS_ERR(sclk)) {
		dev_err(dev, "cannot get avp sclk\n");
		return -ENODEV;
	}

	clk_prepare_enable(sclk);

	return 0;
}

static int bpmp_linear_map_init(struct platform_device *pdev)
{
	struct device_node *node;
	uint32_t of_start;
	uint32_t of_size;
	int ret;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	DEFINE_DMA_ATTRS(attrs);
#else
	unsigned long attrs;
#endif

	node = pdev->dev.of_node;

	ret = of_property_read_u32(node, "carveout-start", &of_start);
	if (ret)
		return ret;

	ret = of_property_read_u32(node, "carveout-size", &of_size);
	if (ret)
		return ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, &attrs);
	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	ret = dma_map_linear_attrs(&pdev->dev, of_start, of_size, 0, &attrs);
#else
	attrs = DMA_ATTR_SKIP_IOVA_GAP | DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_linear_attrs(&pdev->dev, of_start, of_size, 0, attrs);
#endif
	if (ret == DMA_ERROR_CODE)
		return -ENOMEM;

	return 0;
}

struct dentry * __weak bpmp_init_debug(struct platform_device *pdev)
{
	return NULL;
}

int __weak bpmp_init_cpuidle_debug(struct dentry *root)
{
	return 0;
}

struct pconfig {
	const struct channel_cfg *chcfg;
	const struct mail_ops *ops;
	uint8_t clk;
	uint8_t cpuidle;
	uint8_t hv;
	uint8_t lin_map;
};

static int bpmp_probe(struct platform_device *pdev)
{
	const struct pconfig *cfg;
	struct dentry *root;
	int r;

	device = &pdev->dev;

	cfg = of_device_get_match_data(&pdev->dev);
	if (!cfg) {
		r = -ENODEV;
		goto err_out;
	}

	if (cfg->hv) {
		r = bpmp_setup_allocator(&pdev->dev);
		if (r)
			goto err_out;
	}

	if (cfg->lin_map) {
		r = bpmp_linear_map_init(pdev);
		if (r)
			goto err_out;
	}

	if (cfg->clk) {
		r = bpmp_clk_init(pdev);
		if (r)
			goto err_out;
	}

	root = bpmp_init_debug(pdev);

	if (root && cfg->cpuidle) {
		r = bpmp_init_cpuidle_debug(root);
		if (r)
			goto err_out;
	}

	bpmp_tty.dev.platform_data = root;

	r = bpmp_do_ping();
	r = r ?: bpmp_get_fwtag();
	r = r ?: of_platform_populate(device->of_node, NULL, NULL, device);
	r = r ?: platform_device_register(&bpmp_tty);
	if (r)
		goto err_out;

	register_syscore_ops(&bpmp_syscore_ops);

	devm_tegrafw_register(device, "bpmp", TFW_NORMAL, bpmp_version, NULL);

	r = bpmp_init_powergate(pdev);
	if (r) {
		dev_err(device, "powergating init failed (%d)\n", r);
		goto err_out;
	}

	dev_info(device, "probe ok\n");

	return 0;

err_out:
	dev_err(device, "probe failed (%d)\n", r);

	return r;
}

static const struct channel_cfg t210_chcfg = {
	.channel_mask = 0xfff,
	.per_cpu_ch_0 = 0,
	.per_cpu_ch_cnt = 4,
	.thread_ch_0 = 4,
	.thread_ch_cnt = 4,
	.ib_ch_0 = 8,
	.ib_ch_cnt = 4
};

static const struct channel_cfg t186_chcfg = {
	.channel_mask = 0x200f,
	.per_cpu_ch_0 = 3,
	.per_cpu_ch_cnt = 1,
	.thread_ch_0 = 0,
	.thread_ch_cnt = 3,
	.ib_ch_0 = 13,
	.ib_ch_cnt = 1
};

static const struct pconfig t210_cfg = {
	.chcfg = &t210_chcfg,
	.ops = &t210_mail_ops,
	.clk = 1,
	.cpuidle = 1,
	.lin_map = 1
};

static const struct pconfig t186_native_cfg = {
	.chcfg = &t186_chcfg,
	.ops = &t186_native_mail_ops
};

static const struct pconfig t186_hv_cfg = {
	.chcfg = &t186_chcfg,
	.ops = &t186_hv_mail_ops,
	.hv = 1
};

const struct of_device_id bpmp_of_matches[] = {
	{ .compatible = "nvidia,tegra186-bpmp", .data = &t186_native_cfg },
	{ .compatible = "nvidia,tegra186-bpmp-hv", .data = &t186_hv_cfg },
	{ .compatible = "nvidia,tegra210-bpmp", .data = &t210_cfg },
	{}
};

static struct platform_driver bpmp_driver = {
	.probe = bpmp_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bpmp",
		.of_match_table = of_match_ptr(bpmp_of_matches)
	}
};

static __init int bpmp_init(void)
{
	struct pconfig *cfg;
	const struct of_device_id *m;
	struct device_node *np;
	int r = -ENODEV;

	np = of_find_matching_node(NULL, bpmp_of_matches);
	if (!np || !of_device_is_available(np))
		goto out;

	m = of_match_node(bpmp_of_matches, np);

	cfg = (struct pconfig *)m->data;

	r = bpmp_mail_init(cfg->chcfg, cfg->ops, np);
	if (r)
		goto out;

	r = platform_driver_register(&bpmp_driver);

out:
	of_node_put(np);

	return r;
}
postcore_initcall(bpmp_init);
