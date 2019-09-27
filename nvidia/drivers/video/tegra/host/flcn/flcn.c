/*
* Tegra flcn common driver
*
* Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>         /* for kzalloc */
#include <asm/byteorder.h>      /* for parsing ucode image wrt endianness */
#include <linux/delay.h>	/* for udelay */
#include <linux/export.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/tegra-powergate.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra_pm_domains.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/iopoll.h>

#include "dev.h"
#include "class_ids.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_vm.h"
#include "nvhost_scale.h"
#include "nvhost_channel.h"

#include "flcn.h"
#include "hw_flcn.h"

#include "t124/hardware_t124.h" /* for nvhost opcodes*/
#include "t124/t124.h"
#include "t210/t210.h"

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "t186/t186.h"
#endif
#ifdef CONFIG_TEGRA_T19X_GRHOST
#include "t194/t194.h"
#endif

static int nvhost_flcn_init_sw(struct platform_device *dev);
static int nvhost_flcn_deinit_sw(struct platform_device *dev);

#define FLCN_IDLE_TIMEOUT_DEFAULT	100000	/* 100 milliseconds */
#define FLCN_IDLE_CHECK_PERIOD		10	/* 10 usec */

static irqreturn_t flcn_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device *)(dev_id);
	struct nvhost_device_data *pdata = nvhost_get_devdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&pdata->mirq_lock, flags);

	if (pdata->flcn_isr)
		pdata->flcn_isr(pdev);

	spin_unlock_irqrestore(&pdata->mirq_lock, flags);

	return IRQ_HANDLED;
}

int flcn_intr_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(pdev);
	const char *dev_name;
	int ret = 0;

	if (!pdata->module_irq)
		return 0;

	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return -ENXIO;
	}

	spin_lock_init(&pdata->mirq_lock);
	dev_name = get_device_name_for_dev(pdev);
	ret = request_irq(pdata->irq,
			  flcn_isr, 0,
			  dev_name, pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq. err %d\n", ret);
		return ret;
	}

	/* keep irq disabled */
	disable_irq(pdata->irq);

	return 0;
}

static int nvhost_flcn_wait_idle(struct platform_device *pdev)
{
	int ret;
	u32 val = 0;
	void __iomem *addr = get_aperture(pdev, 0) + flcn_idlestate_r();

	nvhost_dbg_fn("");
	ret = readl_poll_timeout(addr, val, (val == 0), FLCN_IDLE_CHECK_PERIOD,
				FLCN_IDLE_TIMEOUT_DEFAULT);
	if (ret)
		nvhost_err(&pdev->dev, "flcn_idle_state_r =%x\n", val);
	else
		nvhost_dbg_fn("done");

	return ret;
}

static int nvhost_flcn_dma_wait_idle(struct platform_device *pdev)
{
	int ret;
	void __iomem *addr = get_aperture(pdev, 0) + flcn_dmatrfcmd_r();
	u32 val;

	nvhost_dbg_fn("");
	ret = readl_poll_timeout(addr, val,
				(flcn_dmatrfcmd_idle_v(val) ==
				 flcn_dmatrfcmd_idle_true_v()),
				FLCN_IDLE_CHECK_PERIOD,
				FLCN_IDLE_TIMEOUT_DEFAULT);
	if (ret)
		nvhost_err(&pdev->dev, "flcn_idle_state_r =%x\n", val);
	else
		nvhost_dbg_fn("done");

	return ret;
}

static int flcn_dma_pa_to_internal_256b(struct platform_device *pdev,
					phys_addr_t pa, u32 internal_offset,
					bool imem)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(pdev);
	u32 cmd = flcn_dmatrfcmd_size_256b_f();
	u32 pa_offset =  flcn_dmatrffboffs_offs_f(pa);
	u32 i_offset = flcn_dmatrfmoffs_offs_f(internal_offset);

	if (imem)
		cmd |= flcn_dmatrfcmd_imem_true_f();

	if (pdata->isolate_contexts)
		cmd |= flcn_dmatrfcmd_dmactx_f(1);

	host1x_writel(pdev, flcn_dmatrfmoffs_r(), i_offset);
	host1x_writel(pdev, flcn_dmatrffboffs_r(), pa_offset);
	host1x_writel(pdev, flcn_dmatrfcmd_r(), cmd);

	return nvhost_flcn_dma_wait_idle(pdev);

}

int nvhost_flcn_load_image(struct platform_device *pdev,
			   dma_addr_t dma_addr,
			   struct flcn_os_image *os,
			   u32 imem_offset)
{
	int ret = 0;
	u32 offset;

	host1x_writel(pdev, flcn_dmactl_r(), 0);
	host1x_writel(pdev, flcn_dmatrfbase_r(),
			(dma_addr + os->bin_data_offset) >> 8);

	/* Write ucode data to dmem */
	dev_dbg(&pdev->dev, "flcn_boot: load dmem\n");
	for (offset = 0; offset < os->data_size; offset += 256) {
		ret = flcn_dma_pa_to_internal_256b(pdev,
						   os->data_offset + offset,
						   offset, false);
		if (ret)
			goto err;
	}

	/* Write nvdec ucode to imem */
	dev_dbg(&pdev->dev, "flcn_boot: load imem\n");
	for (offset = imem_offset; offset < os->code_size; offset += 256) {
		ret = flcn_dma_pa_to_internal_256b(pdev, os->code_offset + offset,
						  offset, true);
		if (ret)
			goto err;
	}

err:
	if (ret)
		nvhost_err(&pdev->dev, "flcn_load_image failed: 0x%x\n", ret);

	return ret;
}

static int flcn_setup_ucode_fce(struct platform_device *dev,
				struct flcn *v,
				struct ucode_v1_flcn *ucode)
{
	u32 *ucode_ptr = v->mapped;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (ucode->bin_header->fce_bin_header_offset == 0xa5a5a5a5)
		return 0;

	ucode->fce_header = (struct ucode_fce_header_v1_flcn *)
			(((void *)ucode_ptr) +
		 ucode->bin_header->fce_bin_header_offset);

	nvhost_dbg_info("fce ucode header: offset, buffer_size, size: 0x%x 0x%x 0x%x",
			ucode->fce_header->fce_ucode_offset,
			ucode->fce_header->fce_ucode_buffer_size,
			ucode->fce_header->fce_ucode_size);

	/* if isolation is enabled.. */
	if (pdata->isolate_contexts) {
		/* create and map fce shadow to all contexts */
		v->fce_mapped = nvhost_vm_allocate_firmware_area(dev,
					ucode->fce_header->fce_ucode_size,
					&v->fce_dma_addr);
		if (!v->fce_mapped)
			return -ENOMEM;

		memcpy(v->fce_mapped, (u8 *)v->mapped +
			ucode->bin_header->fce_bin_data_offset,
			ucode->fce_header->fce_ucode_size);
	} else {
		/* ..otherwise use the one in firmware image */
		v->fce_dma_addr = v->dma_addr +
					ucode->bin_header->fce_bin_data_offset;
	}

	v->fce.size = ucode->fce_header->fce_ucode_size;
	v->fce.data_offset = ucode->bin_header->fce_bin_data_offset;

	return 0;
}


int flcn_setup_ucode_image(struct platform_device *dev,
			   struct flcn *v,
			   const struct firmware *ucode_fw,
			   struct ucode_v1_flcn *ucode)
{
	int w;
	u32 *ucode_ptr = v->mapped;

	nvhost_dbg_fn("");

	/* image data is little endian. */
	/* copy the whole thing taking into account endianness */
	for (w = 0; w < ucode_fw->size/sizeof(u32); w++)
		ucode_ptr[w] = le32_to_cpu(((__le32 *)ucode_fw->data)[w]);

	ucode->bin_header = (struct ucode_bin_header_v1_flcn *)ucode_ptr;
	/* endian problems would show up right here */
	if (ucode->bin_header->bin_magic != 0x10de &&
		ucode->bin_header->bin_magic != 0x10fe) {
		dev_err(&dev->dev, "failed to get firmware magic");
		return -EINVAL;
	}

	if (ucode->bin_header->bin_ver != 1) {
		dev_err(&dev->dev, "unsupported firmware version");
		return -ENOENT;
	}

	/* shouldn't be bigger than what firmware thinks */
	if (ucode->bin_header->bin_size > ucode_fw->size) {
		dev_err(&dev->dev, "ucode image size inconsistency");
		return -EINVAL;
	}

	nvhost_dbg_info("ucode bin header: magic:0x%x ver:%d size:%d",
			ucode->bin_header->bin_magic,
			ucode->bin_header->bin_ver,
			ucode->bin_header->bin_size);
	nvhost_dbg_info("ucode bin header: os bin (header,data) offset size: 0x%x, 0x%x %d",
			ucode->bin_header->os_bin_header_offset,
			ucode->bin_header->os_bin_data_offset,
			ucode->bin_header->os_bin_size);
	nvhost_dbg_info("ucode bin header: fce bin (header,data) offset size: 0x%x, 0x%x %d",
			ucode->bin_header->fce_bin_header_offset,
			ucode->bin_header->fce_bin_data_offset,
			ucode->bin_header->fce_bin_size);

	ucode->os_header = (struct ucode_os_header_v1_flcn *)
		(((void *)ucode_ptr) + ucode->bin_header->os_bin_header_offset);

	nvhost_dbg_info("os ucode header: os code (offset,size): 0x%x, 0x%x",
			ucode->os_header->os_code_offset,
			ucode->os_header->os_code_size);
	nvhost_dbg_info("os ucode header: os data (offset,size): 0x%x, 0x%x",
			ucode->os_header->os_data_offset,
			ucode->os_header->os_data_size);
	nvhost_dbg_info("os ucode header: num apps: %d", ucode->os_header->num_apps);

	v->os.size = ucode->bin_header->os_bin_size;
	v->os.bin_data_offset = ucode->bin_header->os_bin_data_offset;
	v->os.code_offset = ucode->os_header->os_code_offset;
	v->os.data_offset = ucode->os_header->os_data_offset;
	v->os.data_size   = ucode->os_header->os_data_size;
	v->os.code_size = ucode->os_header->os_code_size;
	v->os.bin_magic = ucode->bin_header->bin_magic;
	v->os.bin_ver_tag = ucode->bin_header->bin_ver_tag;

	return 0;
}

int flcn_reload_fw(struct platform_device *pdev)
{
	int err;
	struct device *device = &pdev->dev;

	err = nvhost_module_do_idle(device);
	if (err)
		return err;

	nvhost_flcn_deinit_sw(pdev);

	err = nvhost_module_do_unidle(device);
	if (err)
		return err;

	return 0;
}

static int flcn_read_ucode(struct platform_device *dev,
		    const char *fw_name,
		    struct flcn *v)
{
	const struct firmware *ucode_fw;
	struct ucode_v1_flcn ucode;
	int err;
	DEFINE_DMA_ATTRS(attrs);

	nvhost_dbg_fn("");
	dma_set_attr(DMA_ATTR_READ_ONLY, __DMA_ATTR(attrs));
	v->dma_addr = 0;
	v->mapped = NULL;

	ucode_fw = nvhost_client_request_firmware(dev, fw_name);
	if (!ucode_fw) {
		nvhost_dbg_fn("request firmware failed");
		dev_err(&dev->dev, "failed to get firmware\n");
		err = -ENOENT;
		return err;
	}

	v->size = ucode_fw->size;
	v->mapped = dma_alloc_attrs(&dev->dev, v->size, &v->dma_addr,
				    GFP_KERNEL, __DMA_ATTR(attrs));
	if (!v->mapped) {
		dev_err(&dev->dev, "dma memory allocation failed");
		err = -ENOMEM;
		goto clean_up;
	}

	err = flcn_setup_ucode_image(dev, v, ucode_fw, &ucode);
	if (err) {
		dev_err(&dev->dev, "failed to parse firmware image\n");
		goto clean_up;
	}

	err = flcn_setup_ucode_fce(dev, v, &ucode);
	if (err) {
		dev_err(&dev->dev, "failed to parse fce image\n");
		goto clean_up;
	}

	v->valid = true;
	release_firmware(ucode_fw);

	return 0;

clean_up:
	if (v->mapped) {
		dma_free_attrs(&dev->dev, v->size, v->mapped, v->dma_addr,
			       __DMA_ATTR(attrs));
		v->mapped = NULL;
		v->dma_addr = 0;
	}
	release_firmware(ucode_fw);
	return err;
}

int nvhost_flcn_wait_mem_scrubbing(struct platform_device *dev)
{
	int retries = FLCN_IDLE_TIMEOUT_DEFAULT / FLCN_IDLE_CHECK_PERIOD;
	nvhost_dbg_fn("");

	do {
		u32 w = host1x_readl(dev, flcn_dmactl_r()) &
			(flcn_dmactl_dmem_scrubbing_m() |
			 flcn_dmactl_imem_scrubbing_m());

		if (!w) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(FLCN_IDLE_CHECK_PERIOD);
	} while (--retries || !tegra_platform_is_silicon());

	nvhost_err(&dev->dev, "Falcon mem scrubbing timeout");
	return -ETIMEDOUT;
}

int nvhost_flcn_prepare_poweroff(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (pdata->flcn_isr)
		disable_irq(pdata->irq);

	return 0;
}

void nvhost_flcn_irq_mask_set(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	/* setup falcon interrupts and enable interface */
	if (!pdata->self_config_flcn_isr)
		host1x_writel(pdev, flcn_irqmset_r(),
			      (flcn_irqmset_ext_f(0xff)    |
			       flcn_irqmset_swgen1_set_f() |
			       flcn_irqmset_swgen0_set_f() |
			       flcn_irqmset_exterr_set_f() |
			       flcn_irqmset_halt_set_f()   |
			       flcn_irqmset_wdtmr_set_f()));
}

void nvhost_flcn_irq_dest_set(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!pdata->self_config_flcn_isr)
		host1x_writel(pdev, flcn_irqdest_r(),
			      (flcn_irqdest_host_ext_f(0xff)     |
			       flcn_irqdest_host_swgen1_host_f() |
			       flcn_irqdest_host_swgen0_host_f() |
			       flcn_irqdest_host_exterr_host_f() |
			       flcn_irqdest_host_halt_host_f()));
}

void nvhost_flcn_ctxtsw_init(struct platform_device *pdev)
{
	host1x_writel(pdev, flcn_itfen_r(),
			(flcn_itfen_mthden_enable_f() |
			 flcn_itfen_ctxen_enable_f()));
}

int nvhost_flcn_start(struct platform_device *pdev, u32 bootvec)
{
	int err = 0;

	/* boot falcon */
	dev_dbg(&pdev->dev, "flcn_boot: start falcon\n");
	host1x_writel(pdev, flcn_bootvec_r(), flcn_bootvec_vec_f(bootvec));
	host1x_writel(pdev, flcn_cpuctl_r(), flcn_cpuctl_startcpu_true_f());

	err = nvhost_flcn_wait_idle(pdev);
	if (err != 0)
		nvhost_err(&pdev->dev, "boot failed due to timeout");

	return err;
}


int nvhost_flcn_finalize_poweron(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct flcn *v;
	int err = 0;

	err = nvhost_flcn_init_sw(pdev);
	if (err)
		return err;

	v = get_flcn(pdev);
	err = nvhost_flcn_wait_mem_scrubbing(pdev);
	if (err)
		return err;

	/* load transcfg configuration if defined */
	if (pdata->transcfg_addr)
		host1x_writel(pdev, pdata->transcfg_addr, pdata->transcfg_val);

	err = nvhost_flcn_load_image(pdev, v->dma_addr, &v->os, 0);
	if (err)
		return err;

	nvhost_flcn_irq_mask_set(pdev);
	nvhost_flcn_irq_dest_set(pdev);
	if (pdata->flcn_isr)
		enable_irq(pdata->irq);

	nvhost_flcn_ctxtsw_init(pdev);
	err = nvhost_flcn_start(pdev, 0);

	return err;
}

int nvhost_flcn_common_isr(struct platform_device *pdev)
{
	u32 irqstat, exci, mailbox0, mailbox1;

	irqstat = host1x_readl(pdev, flcn_irqstat_r());
	exci = host1x_readl(pdev, flcn_exci_r());
	mailbox0 = host1x_readl(pdev, flcn_mailbox0_r());
	mailbox1 = host1x_readl(pdev, flcn_mailbox1_r());

	dev_err(&pdev->dev, "irqstat: %08x, exci: %08x, mailbox0: %08x, mailbox1: %08x",
		irqstat, exci, mailbox0, mailbox1);

	/* logic to clear the interrupt */
	host1x_writel(pdev, flcn_thi_int_stat_r(), flcn_thi_int_stat_clr_f());
	host1x_readl(pdev, flcn_thi_int_stat_r());
	host1x_writel(pdev, flcn_irqsclr_r(), flcn_irqsclr_swgen0_set_f());

	return 0;
}

static int nvhost_flcn_init_sw(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
	struct flcn *v = get_flcn(dev);

	nvhost_dbg_fn("in dev:%p v:%p", dev, v);

	if (v)
		return 0;

	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v) {
		err = -ENOMEM;
		goto clean_up;
	}

	set_flcn(dev, v);
	nvhost_dbg_fn("primed dev:%p v:%p", dev, v);
	err = flcn_read_ucode(dev, pdata->firmware_name, v);
	if (err || !v->valid)
		goto clean_up;

	return 0;

 clean_up:
	nvhost_err(&dev->dev, "failed : 0x%x", err);
	return err;
}

static int nvhost_flcn_deinit_sw(struct platform_device *dev)
{
	struct flcn *v = get_flcn(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	DEFINE_DMA_ATTRS(attrs);
	dma_set_attr(DMA_ATTR_READ_ONLY, &attrs);
#endif

	if (!v)
		return 0;

	if (v->mapped) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
		dma_free_attrs(&dev->dev, v->size, v->mapped, v->dma_addr,
			       &attrs);
#else
		dma_free_attrs(&dev->dev, v->size, v->mapped, v->dma_addr,
			       DMA_ATTR_READ_ONLY);
#endif
		v->mapped = NULL;
		v->dma_addr = 0;
	}
	kfree(v);
	set_flcn(dev, NULL);
	return 0;
}

int nvhost_vic_finalize_poweron(struct platform_device *pdev)
{
	struct flcn *v;
	int err;

	err = nvhost_flcn_finalize_poweron(pdev);
	if (err)
		return err;

	v = get_flcn(pdev);

	host1x_writel(pdev, FLCN_UCLASS_METHOD_OFFSET * 4,
		      NVA0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID >> 2);
	host1x_writel(pdev, FLCN_UCLASS_METHOD_DATA * 4, 1);
	host1x_writel(pdev, FLCN_UCLASS_METHOD_OFFSET * 4,
		      NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE >> 2);
	host1x_writel(pdev, FLCN_UCLASS_METHOD_DATA * 4, v->fce.size);
	host1x_writel(pdev, FLCN_UCLASS_METHOD_OFFSET * 4,
		      NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET >> 2);
	host1x_writel(pdev, FLCN_UCLASS_METHOD_DATA * 4,
		      (v->dma_addr + v->fce.data_offset) >> 8);

	return 0;
}

int nvhost_vic_init_context(struct platform_device *pdev,
			    struct nvhost_cdma *cdma)
{
	struct flcn *v;
	int err;

	err = nvhost_flcn_init_sw(pdev);
	if (err)
		return err;

	v = get_flcn(pdev);

	/* load application id */
	nvhost_cdma_push(cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_VIC_CLASS_ID,
			FLCN_UCLASS_METHOD_OFFSET, 1),
		NVA0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID >> 2);
	nvhost_cdma_push(cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_VIC_CLASS_ID,
				       FLCN_UCLASS_METHOD_DATA, 1), 1);

	/* set fce ucode size */
	nvhost_cdma_push(cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_VIC_CLASS_ID,
			FLCN_UCLASS_METHOD_OFFSET, 1),
		NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE >> 2);
	nvhost_cdma_push(cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_VIC_CLASS_ID,
			FLCN_UCLASS_METHOD_DATA, 1), v->fce.size);

	/* set fce ucode offset */
	nvhost_cdma_push(cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_VIC_CLASS_ID,
			FLCN_UCLASS_METHOD_OFFSET, 1),
		NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET >> 2);
	nvhost_cdma_push(cdma,
		nvhost_opcode_setclass(NV_GRAPHICS_VIC_CLASS_ID,
			FLCN_UCLASS_METHOD_DATA, 1),
		v->fce_dma_addr >> 8);

	return 0;
}

void flcn_enable_timestamps(struct platform_device *pdev,
				struct nvhost_cdma *cdma,
				dma_addr_t timestamp_addr)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	/* set timestamp buffer offset */
	nvhost_cdma_push(cdma, nvhost_opcode_setclass(pdata->class,
			FLCN_UCLASS_METHOD_OFFSET, 1),
			FLCN_UCLASS_METHOD_ADDR_TSP);
	nvhost_cdma_push(cdma, nvhost_opcode_setclass(pdata->class,
			FLCN_UCLASS_METHOD_DATA, 1),
			timestamp_addr >> 8);
}

int nvhost_vic_aggregate_constraints(struct platform_device *dev,
				     int clk_index,
				     unsigned long floor_rate,
				     unsigned long pixelrate,
				     unsigned long bw_constraint)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);

	if (!pdata) {
		dev_err(&dev->dev, "no platform data\n");
		/* return 0 to fall-back on default policy */
		return 0;
	}

	/* Fall-back to default policy if pixelrate
	 * is unavailable or clk index is incorrect.
	 * Here clk_index 2 is for floor client.
	 */
	if (!pixelrate || clk_index != 2)
		return 0;

	/* Compute VIC frequency based on pixelrate */
	return pixelrate / pdata->num_ppc;
}

static struct of_device_id tegra_flcn_of_match[] = {
#ifdef CONFIG_TEGRA_GRHOST_VIC
	{ .compatible = "nvidia,tegra124-vic",
		.data = (struct nvhost_device_data *)&t124_vic_info },
	{ .compatible = "nvidia,tegra210-vic",
		.data = (struct nvhost_device_data *)&t21_vic_info },
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{ .compatible = "nvidia,tegra124-msenc",
		.data = (struct nvhost_device_data *)&t124_msenc_info },
#endif
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{ .compatible = "nvidia,tegra210-nvenc",
		.data = (struct nvhost_device_data *)&t21_msenc_info },
#endif
#endif
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-nvjpg",
		.data = (struct nvhost_device_data *)&t21_nvjpg_info },
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#if defined(CONFIG_TEGRA_GRHOST_VIC)
	{ .compatible = "nvidia,tegra186-vic",
		.data = (struct nvhost_device_data *)&t18_vic_info },
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVJPG)
	{ .compatible = "nvidia,tegra186-nvjpg",
		.data = (struct nvhost_device_data *)&t18_nvjpg_info },
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{ .compatible = "nvidia,tegra186-nvenc",
		.data = (struct nvhost_device_data *)&t18_msenc_info },
#endif
#endif
#ifdef CONFIG_TEGRA_T19X_GRHOST
#if defined(CONFIG_TEGRA_GRHOST_VIC)
	{ .compatible = "nvidia,tegra194-vic",
		.data = (struct nvhost_device_data *)&t19_vic_info },
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVJPG)
	{ .compatible = "nvidia,tegra194-nvjpg",
		.data = (struct nvhost_device_data *)&t19_nvjpg_info },
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{ .compatible = "nvidia,tegra194-nvenc",
		.data = (struct nvhost_device_data *)&t19_msenc_info,
		.name = "nvenc" },
	{ .compatible = "nvidia,tegra194-nvenc",
		.data = (struct nvhost_device_data *)&t19_nvenc1_info,
		.name = "nvenc1" },
#endif
#endif
	{ },
};

static ssize_t reload_fw_write(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int err;
	unsigned long val = 0;
	struct platform_device *pdev = to_platform_device(device);

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	if (!val)
		return -EINVAL;

	err = flcn_reload_fw(pdev);
	if (err)
		return err;

	return count;
}

static DEVICE_ATTR(reload_fw, 0200, NULL, reload_fw_write);

static int flcn_probe(struct platform_device *dev)
{
	int err;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_flcn_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	if (!pdata) {
		dev_err(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	nvhost_dbg_fn("dev:%p pdata:%p", dev, pdata);

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	err = device_create_file(&dev->dev, &dev_attr_reload_fw);
	if (err)
		return err;

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	dev->dev.platform_data = NULL;

	nvhost_module_init(dev);

	err = nvhost_client_device_init(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client device for %s",
			      dev->name);
		pm_runtime_put(&dev->dev);
		return err;
	}

	if (pdata->flcn_isr)
		flcn_intr_init(dev);

	return 0;
}

static int __exit flcn_remove(struct platform_device *pdev)
{
	nvhost_client_device_release(pdev);

	return 0;
}

static struct platform_device_id flcn_id_table[] = {
	{ .name = "vic03" },
	{ .name = "msenc" },
	{ .name = "msenc" },
	{ .name = "nvjpg" },
	{},
};
static struct platform_driver flcn_driver = {
	.probe = flcn_probe,
	.remove = __exit_p(flcn_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "falcon",
#ifdef CONFIG_OF
		.of_match_table = tegra_flcn_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
		.suppress_bind_attrs = true,
	},
	.id_table = flcn_id_table,
};

static struct of_device_id tegra_flcn_domain_match[] = {
#ifdef CONFIG_TEGRA_GRHOST_VIC
	{.compatible = "nvidia,tegra124-vic03-pd",
	.data = (struct nvhost_device_data *)&t124_vic_info},
	{.compatible = "nvidia,tegra132-vic03-pd",
	.data = (struct nvhost_device_data *)&t124_vic_info},
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{.compatible = "nvidia,tegra124-msenc-pd",
	.data = (struct nvhost_device_data *)&t124_msenc_info},
	{.compatible = "nvidia,tegra132-msenc-pd",
	.data = (struct nvhost_device_data *)&t124_msenc_info},
#endif
#ifdef CONFIG_TEGRA_GRHOST_VIC
	{.compatible = "nvidia,tegra210-vic03-pd",
	 .data = (struct nvhost_device_data *)&t21_vic_info},
#endif
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{.compatible = "nvidia,tegra210-msenc-pd",
	 .data = (struct nvhost_device_data *)&t21_msenc_info},
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVJPG)
	{.compatible = "nvidia,tegra210-nvjpg-pd",
	 .data = (struct nvhost_device_data *)&t21_nvjpg_info},
#endif
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#if defined(CONFIG_TEGRA_GRHOST_VIC)
	{.compatible = "nvidia,tegra186-vic03-pd",
	 .data = (struct nvhost_device_data *)&t18_vic_info},
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{.compatible = "nvidia,tegra186-msenc-pd",
	 .data = (struct nvhost_device_data *)&t18_msenc_info},
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVJPG)
	{.compatible = "nvidia,tegra186-nvjpg-pd",
	 .data = (struct nvhost_device_data *)&t18_nvjpg_info},
#endif
#endif
#ifdef CONFIG_TEGRA_T19X_GRHOST
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
	{.compatible = "nvidia,tegra194-nvenc1-pd",
	 .data = (struct nvhost_device_data *)&t19_nvenc1_info},
#endif
#endif
	{},
};

static int __init flcn_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra_flcn_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&flcn_driver);
}

static void __exit flcn_exit(void)
{
	platform_driver_unregister(&flcn_driver);
}

module_init(flcn_init);
module_exit(flcn_exit);
