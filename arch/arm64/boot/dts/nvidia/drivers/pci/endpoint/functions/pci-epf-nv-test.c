/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/dma-iommu.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

#define BAR0_SIZE SZ_64K

struct pci_epf_nv_test {
	struct pci_epf_header header;
	struct page *bar0_ram_page;
	dma_addr_t bar0_iova;
	void *bar0_ram_map;
};

static void pci_epf_nv_test_unbind(struct pci_epf *epf)
{
	struct pci_epf_nv_test *epfnv = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct device *cdev = epc->dev.parent;
	struct iommu_domain *domain = iommu_get_domain_for_dev(cdev);

	pci_epc_stop(epc);
	pci_epc_clear_bar(epc, BAR_0);
	vunmap(epfnv->bar0_ram_map);
	iommu_unmap(domain, epfnv->bar0_iova, PAGE_SIZE);
	iommu_dma_free_iova(cdev, epfnv->bar0_iova, BAR0_SIZE);
	__free_pages(epfnv->bar0_ram_page, 1);
}

static int pci_epf_nv_test_bind(struct pci_epf *epf)
{
	struct pci_epf_nv_test *epfnv = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_header *header = epf->header;
	struct device *fdev = &epf->dev;
	struct device *cdev = epc->dev.parent;
	struct iommu_domain *domain = iommu_get_domain_for_dev(cdev);
	int ret;

	ret = pci_epc_write_header(epc, header);
	if (ret) {
		dev_err(fdev, "pci_epc_write_header() failed: %d\n", ret);
		return ret;
	}

	epfnv->bar0_ram_page = alloc_pages(GFP_KERNEL, 1);
	if (!epfnv->bar0_ram_page) {
		dev_err(fdev, "alloc_pages() failed\n");
		ret = -ENOMEM;
		goto fail;
	}
	dev_info(fdev, "BAR0 RAM phys: 0x%llx\n",
		 page_to_phys(epfnv->bar0_ram_page));

	epfnv->bar0_iova = iommu_dma_alloc_iova(cdev, BAR0_SIZE,
						cdev->coherent_dma_mask);
	if (!epfnv->bar0_iova) {
		dev_err(fdev, "iommu_dma_alloc_iova() failed\n");
		ret = -ENOMEM;
		goto fail_free_pages;
	}

	dev_info(fdev, "BAR0 RAM IOVA: 0x%08llx\n", epfnv->bar0_iova);

	ret = iommu_map(domain, epfnv->bar0_iova,
			page_to_phys(epfnv->bar0_ram_page),
			PAGE_SIZE, IOMMU_READ | IOMMU_WRITE);
	if (ret) {
		dev_err(fdev, "iommu_map(RAM) failed: %d\n", ret);
		goto fail_free_iova;
	}
	epfnv->bar0_ram_map = vmap(&epfnv->bar0_ram_page, 1, VM_MAP,
				   PAGE_KERNEL);
	if (!epfnv->bar0_ram_map) {
		dev_err(fdev, "vmap() failed\n");
		ret = -ENOMEM;
		goto fail_unmap_ram_iova;
	}
	dev_info(fdev, "BAR0 RAM virt: 0x%p\n", epfnv->bar0_ram_map);

	ret = pci_epc_set_bar(epc, BAR_0, epfnv->bar0_iova, BAR0_SIZE,
			      PCI_BASE_ADDRESS_SPACE_MEMORY |
			      PCI_BASE_ADDRESS_MEM_TYPE_32);
	if (ret) {
		dev_err(fdev, "pci_epc_set_bar() failed: %d\n", ret);
		goto fail_unmap_ram_virt;
	}

	return 0;

fail_unmap_ram_virt:
	vunmap(epfnv->bar0_ram_map);
fail_unmap_ram_iova:
	iommu_unmap(domain, epfnv->bar0_iova, PAGE_SIZE);
fail_free_iova:
	iommu_dma_free_iova(cdev, epfnv->bar0_iova, BAR0_SIZE);
fail_free_pages:
	__free_pages(epfnv->bar0_ram_page, 1);
fail:
	return ret;
}

static void pci_epf_nv_test_linkup(struct pci_epf *epf)
{
}

static const struct pci_epf_device_id pci_epf_nv_test_ids[] = {
	{
		.name = "pci_epf_nv_test",
	},
	{},
};

static int pci_epf_nv_test_probe(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;
	struct pci_epf_nv_test *epfnv;

	epfnv = devm_kzalloc(dev, sizeof(*epfnv), GFP_KERNEL);
	if (!epfnv)
		return -ENOMEM;
	epf_set_drvdata(epf, epfnv);

	epfnv->header.vendorid = PCI_VENDOR_ID_NVIDIA;
	epfnv->header.deviceid = PCI_ANY_ID;
	epfnv->header.baseclass_code = PCI_BASE_CLASS_MEMORY;
	epfnv->header.interrupt_pin = PCI_INTERRUPT_INTA;
	epf->header = &epfnv->header;

	return 0;
}

static struct pci_epf_ops ops = {
	.unbind	= pci_epf_nv_test_unbind,
	.bind	= pci_epf_nv_test_bind,
	.linkup = pci_epf_nv_test_linkup,
};

static struct pci_epf_driver test_driver = {
	.driver.name	= "pci_epf_nv_test",
	.probe		= pci_epf_nv_test_probe,
	.id_table	= pci_epf_nv_test_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_nv_test_init(void)
{
	int ret;

	ret = pci_epf_register_driver(&test_driver);
	if (ret) {
		pr_err("Failed to register PCIe EPF NV test driver: %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_nv_test_init);

static void __exit pci_epf_nv_test_exit(void)
{
	pci_epf_unregister_driver(&test_driver);
}
module_exit(pci_epf_nv_test_exit);

MODULE_DESCRIPTION("PCI EPF NV TEST DRIVER");
MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_LICENSE("GPL v2");
