/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>

/* WITHOUT_DMA_API  -->> Enabled
 *	-> A quick memory allocation to input PHY address and return virtual
 *	   virtual base address to that memory block which can use as a
 *	   specific memory block to driver
 *	-> This will not attach the memory to the device and DMA memory pool.
 *	   It's a one shot memory allocation
 * WITHOUT_DMA_API  -->> Disabled
 *	-> Will declare memory as a DMA memory pool attached to this device
 *	   It will help to allocate/deallocate memory to multiple buffer
 *	   multiple time
 *	-> Buffer DMA address can utilize with dma streaming API for fast
 *	   trasfer between IO device and buffer location.
 */

/* Uncomment below macro when want to reserve the memory without DMA API */
//#define	WITHOUT_DMA_API

#ifndef WITHOUT_DMA_API
#include <linux/dma-mapping.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#else
#include <linux/io.h>
#include <linux/of_address.h>
#endif

/* Ring buffer data structure */
struct dummy_ring_buff {
	char			data[SZ_8M];	/* data can be of any size */
	dma_addr_t		dma_handle;	/*  DMA PHY Address */
	struct dummy_ring_buff	*next;
};

/* driver private data structure */
struct dummy_mem_priv {
	void			*virt;	/* virtual base address */
	phys_addr_t		base;	/* physical base address */
	size_t			size;	/* size of the  reserved memory */
	struct device		*dev;
	struct reserved_mem	*rmem;
	struct dummy_ring_buff	*head;
};

#ifndef	WITHOUT_DMA_API

static int dummy_mem_carveout_device_init(struct reserved_mem *rmem,
					  struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dummy_mem_priv *priv = platform_get_drvdata(pdev);
	int ret;

	/* reserved memory will be declare as a coherent memory pool */
	ret = dma_declare_coherent_memory(dev, rmem->base, rmem->base,
					  rmem->size, DMA_MEMORY_MAP |
					  DMA_MEMORY_EXCLUSIVE);
	if (ret != DMA_MEMORY_MAP) {
		dev_err(dev,
			"Reserved memory: failed to init at %pa, size %ld MiB\n",
			&rmem->base, ((unsigned long)rmem->size / SZ_1M));
		return -ENOMEM;
	}

	priv->base = rmem->base;
	priv->size = rmem->size;
	priv->rmem = rmem;
	priv->dev = dev;

	rmem->priv = priv;

	return 0;
}

static void dummy_mem_carveout_device_release(struct reserved_mem *rmem,
					      struct device *dev)
{
	dma_release_declared_memory(dev);
}

static const struct reserved_mem_ops dummy_mem_carveout_ops = {
	.device_init	= dummy_mem_carveout_device_init,
	.device_release	= dummy_mem_carveout_device_release,
};

static int __init dummy_mem_carveout_setup(struct reserved_mem *rmem)
{
	unsigned long node = rmem->fdt_node;

	/* This driver isn't reserved memory which allow to use memory block
	 * to Linux, Reserved memory will be specific to driver only
	 */
	if (of_get_flat_dt_prop(node, "reusable", NULL)) {
		pr_err("Not yet supported with %s\n", "reusable");
		return -EINVAL;
	}

	/* Reserve memory will not be part of the system memory */
	if (!of_get_flat_dt_prop(node, "no-map", NULL)) {
		pr_err("Not yet supported without %s\n", "no-map");
		return -EINVAL;
	}

	rmem->ops = &dummy_mem_carveout_ops;

	pr_info("Reserved memory: created at %pa, of size %ld MiB\n",
		&rmem->base, (unsigned long)rmem->size / SZ_1M);

	return 0;
}

RESERVEDMEM_OF_DECLARE(dummy_mem_carveout, "nvidia,dummy_mem_region",
		       dummy_mem_carveout_setup);

#endif

#ifndef	WITHOUT_DMA_API

static int dummy_mem_carveout_alloc_with_dma(struct platform_device *pdev)
{
	struct dummy_ring_buff	*temp_buff;
	dma_addr_t	dma_handle;
	int ret;

	struct dummy_mem_priv *priv =
		devm_kzalloc(&pdev->dev, sizeof(struct dummy_mem_priv),
			     GFP_KERNEL);

	platform_set_drvdata(pdev, priv);

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Memory reservation intialization failed\n");
		return ret;
	}

	/* will allocate the input size memory from device memory pool which
	 * was reserved specific to this driver. dma_alloc_coherent will
	 * return the virtual memory address as a return value and physical
	 * memory base address in dma_handle
	 */

	/* one can write mmap function to map this buffer to userspace using
	 * "remap_pfn_range(vma, vma->vm_start,
	 * PFN_DOWN(virt_to_phys(bus_to_virt(dma_handle))) + vma->vm_pgoff,
	 * (vma->vm_end - vma->vm_start), vma->vm_page_prot);"
	 */
	temp_buff = dma_alloc_coherent(&pdev->dev, SZ_8M, &dma_handle,
				       GFP_KERNEL);
	if (temp_buff) {
		temp_buff->dma_handle = dma_handle;
		priv->head = temp_buff;
		priv->virt = temp_buff;
		dev_dbg(&pdev->dev, "Allocated memory at dma_handle addr %pad\n",
			&dma_handle);
	} else {
		dev_err(&pdev->dev,
			"DMA coherent memory allocation failed from pool\n");
		return -ENOMEM;
	}

	return 0;
}

#else

static int dummy_mem_carveout_alloc_without_dma(struct platform_device *pdev)
{
	struct device_node	*np;
	struct resource		res;
	int			ret;
	void			*vaddr;

	struct dummy_mem_priv *priv =
		devm_kzalloc(&pdev->dev, sizeof(struct dummy_mem_priv),
			     GFP_KERNEL);

	platform_set_drvdata(pdev, priv);

	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		return -EINVAL;
	}

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(&pdev->dev, "No memory address assigned to the region\n");
		return ret;
	}

	priv->size = resource_size(&res);
	priv->base = res.start;

	/* memremap will return the virtual base address of the reserved memory
	 * which will be helpful to create more buffers from this consecutive
	 * memory block up to the allocated size(priv->size).
	 */
	vaddr = devm_memremap(&pdev->dev, priv->base, priv->size, MEMREMAP_WC);

	dev_dbg(&pdev->dev, "Allocated reserved memory, vaddr: %p , paddr: %pa\n",
		vaddr, &res.start);

	priv->virt = vaddr;

	return 0;
}

#endif

static int __init dummy_mem_carveout_probe(struct platform_device *pdev)
{
#ifndef	WITHOUT_DMA_API
	return dummy_mem_carveout_alloc_with_dma(pdev);
#else
	return dummy_mem_carveout_alloc_without_dma(pdev);
#endif
}

static int dummy_mem_carveout_remove(struct platform_device *pdev)
{
	struct dummy_mem_priv *priv = platform_get_drvdata(pdev);

#ifndef	WITHOUT_DMA_API
	struct dummy_ring_buff *temp_buff = priv->head;

	dma_free_coherent(&pdev->dev, SZ_8M, (void *)temp_buff,
			  temp_buff->dma_handle);

	of_reserved_mem_device_release(&pdev->dev);
	devm_kfree(&pdev->dev, priv);
#else
	devm_memunmap(&pdev->dev, priv->virt);
#endif
	return 0;
}

const struct of_device_id dummy_mem_carveout_of_ids[] = {
	{ .compatible = "nvidia,dummy_mem_carveout" },
};

static struct platform_driver __refdata dummy_mem_carveout_driver = {
	.probe      = dummy_mem_carveout_probe,
	.remove     = dummy_mem_carveout_remove,
	.driver = {
		.name   = "dummy-mem-carveout",
		.owner  = THIS_MODULE,
		.of_match_table = dummy_mem_carveout_of_ids,
	},
};

module_platform_driver(dummy_mem_carveout_driver);

MODULE_LICENSE("GPL");
