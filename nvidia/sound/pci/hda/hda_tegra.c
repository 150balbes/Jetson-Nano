/*
 *
 * Implementation of primary ALSA driver code base for NVIDIA Tegra HDA.
 *
 * Copyright (c) 2014-2019, NVIDIA CORPORATION, All rights reserved.
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
 *
 */

#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/pm_runtime.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra_pm_domains.h>
#include <soc/tegra/common.h>

#include <sound/core.h>
#include <sound/initval.h>

#include <video/tegra_hdmi_audio.h>

#include "hda_codec.h"
#include "hda_controller.h"


static const struct of_device_id tegra_disb_pd[] = {
	{ .compatible = "nvidia,tegra194-disa-pd", },
	{ .compatible = "nvidia,tegra186-disa-pd", },
	{ .compatible = "nvidia,tegra210-disb-pd", },
	{ .compatible = "nvidia,tegra132-disb-pd", },
	{ .compatible = "nvidia,tegra124-disb-pd", },
	{},
};

/* Defines for Nvidia Tegra HDA support */
#define HDA_BAR0           0x8000
#define HDA_DFPCI_CFG      0x1000

#define HDA_CFG_CMD        0x1004
#define HDA_CFG_BAR0       0x1010

#define HDA_ENABLE_IO_SPACE       (1 << 0)
#define HDA_ENABLE_MEM_SPACE      (1 << 1)
#define HDA_ENABLE_BUS_MASTER     (1 << 2)
#define HDA_ENABLE_SERR           (1 << 8)
#define HDA_DISABLE_INTR          (1 << 10)
#define HDA_BAR0_INIT_PROGRAM     0xFFFFFFFF
#define HDA_BAR0_FINAL_PROGRAM    (1 << 14)

/* IPFS */
#define HDA_IPFS_CONFIG           0x180
#define HDA_IPFS_EN_FPCI          0x1

#define HDA_IPFS_FPCI_BAR0        0x80
#define HDA_FPCI_BAR0_START       0x40

#define HDA_IPFS_INTR_MASK        0x188
#define HDA_IPFS_EN_INTR          (1 << 16)

/* FPCI */
#define FPCI_DBG_CFG_2		  0xF4
#define FPCI_FIFO_WATERMARK	  0x7c
#define FPCI_BUFSZ_NUM_OF_FRAMES  0x80

#define FPCI_GCAP_NSDO_SHIFT	  18
#define FPCI_GCAP_NSDO_MASK	  (0x3 << FPCI_GCAP_NSDO_SHIFT)

/* default values for watermark registers */
#define FIFO_WATERMARK_VAL	  0x07070707
#define BUFSZ_NUM_FRAMES_VAL	  0x000a0a0a

/* max number of SDs */
#define NUM_CAPTURE_SD 1
#define NUM_PLAYBACK_SD 1

/* GSC_ID register */
#define HDA_GSC_REG		0x1e0
#define HDA_GSC_ID		10

#define HDA_MAX_CODECS		8
#define CHAR_BUF_SIZE_MAX	50

struct tegra_hda_chip_data {
	unsigned int war_sdo_lines;
	bool set_watermark;
	bool war_sdo_bw;
	bool set_gsc_id;
};

struct hda_pcm_devices {
	struct azx_pcm *apcm;
	struct kobject *kobj;
	struct kobj_attribute pcm_attr;
	struct kobj_attribute name_attr;
	char switch_name[CHAR_BUF_SIZE_MAX];
	int dev_id;
};

struct hda_tegra {
	struct azx chip;
	struct device *dev;
	struct clk *hda_clk;
	struct clk *hda2codec_2x_clk;
	struct clk *hda2hdmi_clk;
	int partition_id;
	void __iomem *regs;
	void __iomem *regs_fpci;
	struct work_struct probe_work;
	struct kobject *kobj;
	struct hda_pcm_devices *hda_pcm_dev;
	const struct tegra_hda_chip_data *cdata;
};

#ifdef CONFIG_PM
static int power_save = CONFIG_SND_HDA_POWER_SAVE_DEFAULT;
module_param(power_save, bint, 0644);
MODULE_PARM_DESC(power_save,
		 "Automatic power-saving timeout (in seconds, 0 = disable).");
#else
#define power_save	0
#endif

/*
 * DMA page allocation ops.
 */
static int dma_alloc_pages(struct hdac_bus *bus, int type, size_t size,
			   struct snd_dma_buffer *buf)
{
	return snd_dma_alloc_pages(type, bus->dev, size, buf);
}

static void dma_free_pages(struct hdac_bus *bus, struct snd_dma_buffer *buf)
{
	snd_dma_free_pages(buf);
}

static int substream_alloc_pages(struct azx *chip,
				 struct snd_pcm_substream *substream,
				 size_t size)
{
	return snd_pcm_lib_malloc_pages(substream, size);
}

static int substream_free_pages(struct azx *chip,
				struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/*
 * Register access ops. Tegra HDA register access is DWORD only.
 */
static void hda_tegra_writel(u32 value, u32 __iomem *addr)
{
	writel(value, addr);
}

static u32 hda_tegra_readl(u32 __iomem *addr)
{
	return readl(addr);
}

static void hda_tegra_writew(u16 value, u16 __iomem  *addr)
{
	unsigned int shift = ((unsigned long)(addr) & 0x3) << 3;
	void __iomem *dword_addr = (void __iomem *)((unsigned long)(addr) & ~0x3);
	u32 v;

	v = readl(dword_addr);
	v &= ~(0xffff << shift);
	v |= value << shift;
	writel(v, dword_addr);
}

static u16 hda_tegra_readw(u16 __iomem *addr)
{
	unsigned int shift = ((unsigned long)(addr) & 0x3) << 3;
	void __iomem *dword_addr = (void __iomem *)((unsigned long)(addr) & ~0x3);
	u32 v;

	v = readl(dword_addr);
	return (v >> shift) & 0xffff;
}

static void hda_tegra_writeb(u8 value, u8 __iomem *addr)
{
	unsigned int shift = ((unsigned long)(addr) & 0x3) << 3;
	void __iomem *dword_addr = (void __iomem *)((unsigned long)(addr) & ~0x3);
	u32 v;

	v = readl(dword_addr);
	v &= ~(0xff << shift);
	v |= value << shift;
	writel(v, dword_addr);
}

static u8 hda_tegra_readb(u8 __iomem *addr)
{
	unsigned int shift = ((unsigned long)(addr) & 0x3) << 3;
	void __iomem *dword_addr = (void __iomem *)((unsigned long)(addr) & ~0x3);
	u32 v;

	v = readl(dword_addr);
	return (v >> shift) & 0xff;
}

static const struct hdac_io_ops hda_tegra_io_ops = {
	.reg_writel = hda_tegra_writel,
	.reg_readl = hda_tegra_readl,
	.reg_writew = hda_tegra_writew,
	.reg_readw = hda_tegra_readw,
	.reg_writeb = hda_tegra_writeb,
	.reg_readb = hda_tegra_readb,
	.dma_alloc_pages = dma_alloc_pages,
	.dma_free_pages = dma_free_pages,
};

static const struct hda_controller_ops hda_tegra_ops = {
	.substream_alloc_pages = substream_alloc_pages,
	.substream_free_pages = substream_free_pages,
};

static void hda_tegra_init(struct hda_tegra *hda)
{
	u32 v;

	/* Enable PCI access */
	v = readl(hda->regs + HDA_IPFS_CONFIG);
	v |= HDA_IPFS_EN_FPCI;
	writel(v, hda->regs + HDA_IPFS_CONFIG);

	/* Enable MEM/IO space and bus master */
	v = readl(hda->regs + HDA_CFG_CMD);
	v &= ~HDA_DISABLE_INTR;
	v |= HDA_ENABLE_MEM_SPACE | HDA_ENABLE_IO_SPACE |
		HDA_ENABLE_BUS_MASTER | HDA_ENABLE_SERR;
	writel(v, hda->regs + HDA_CFG_CMD);

	writel(HDA_BAR0_INIT_PROGRAM, hda->regs + HDA_CFG_BAR0);
	writel(HDA_BAR0_FINAL_PROGRAM, hda->regs + HDA_CFG_BAR0);
	writel(HDA_FPCI_BAR0_START, hda->regs + HDA_IPFS_FPCI_BAR0);

	v = readl(hda->regs + HDA_IPFS_INTR_MASK);
	v |= HDA_IPFS_EN_INTR;
	writel(v, hda->regs + HDA_IPFS_INTR_MASK);
}

static int hda_tegra_turn_on(struct hda_tegra *hda)
{
	struct device *dev = hda->dev;
	int rc;

	rc = tegra_unpowergate_partition(hda->partition_id);
	if (rc < 0) {
		dev_err(dev, "Unpower gating HDA partition Failed\n");
		return rc;
	}

	rc = clk_prepare_enable(hda->hda_clk);
	if (rc)
		goto err;
	rc = clk_prepare_enable(hda->hda2codec_2x_clk);
	if (rc)
		goto disable_hda;
	rc = clk_prepare_enable(hda->hda2hdmi_clk);
	if (rc)
		goto disable_codec_2x;

	return 0;

disable_codec_2x:
	clk_disable_unprepare(hda->hda2codec_2x_clk);
disable_hda:
	clk_disable_unprepare(hda->hda_clk);
err:
	if (tegra_powergate_partition(hda->partition_id) < 0)
		dev_err(dev, "Power gating HDA partition Failed\n");

	return rc;
}

static int hda_tegra_turn_off(struct hda_tegra *hda)
{
	struct device *dev = hda->dev;
	int rc = 0;

	clk_disable_unprepare(hda->hda2hdmi_clk);
	clk_disable_unprepare(hda->hda2codec_2x_clk);
	clk_disable_unprepare(hda->hda_clk);

	rc = tegra_powergate_partition(hda->partition_id);
	if (rc < 0)
		dev_err(dev, "Power gating HDA partition Failed\n");

	return rc;
}

/*
 * power management
 */
#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_SUSPEND)
static int hda_tegra_suspend(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	int rc;

	rc = pm_runtime_force_suspend(dev);
	if (rc < 0)
		return rc;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);

	return 0;
}

static int hda_tegra_resume(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	int rc;

	rc = pm_runtime_force_resume(dev);
	if (rc < 0)
		return rc;

	snd_power_change_state(card, SNDRV_CTL_POWER_D0);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int hda_tegra_runtime_suspend(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct azx *chip = card->private_data;
	struct hda_tegra *hda = container_of(chip, struct hda_tegra, chip);
	struct hdac_bus *bus = azx_bus(chip);

	if (chip && chip->running) {
		azx_stop_chip(chip);
		synchronize_irq(bus->irq);
		azx_enter_link_reset(chip);
	}

	return hda_tegra_turn_off(hda);
}

static int hda_tegra_runtime_resume(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct azx *chip = card->private_data;
	struct hda_tegra *hda = container_of(chip, struct hda_tegra, chip);
	int rc;

	rc = hda_tegra_turn_on(hda);
	if (rc != 0)
		return rc;

	if (chip && chip->running) {
		hda_tegra_init(hda);
		azx_init_chip(chip, 1);
	}

	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops hda_tegra_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(hda_tegra_suspend, hda_tegra_resume)
	SET_RUNTIME_PM_OPS(hda_tegra_runtime_suspend,
				hda_tegra_runtime_resume, NULL)
};

static int hda_tegra_dev_disconnect(struct snd_device *device)
{
	struct azx *chip = device->device_data;

	chip->bus.shutdown = 1;
	return 0;
}

/*
 * destructor
 */
static int hda_tegra_dev_free(struct snd_device *device)
{
	struct azx *chip = device->device_data;
	struct hda_tegra *hda = container_of(chip, struct hda_tegra, chip);

	cancel_work_sync(&hda->probe_work);
	if (azx_bus(chip)->chip_init) {
		azx_stop_all_streams(chip);
		azx_stop_chip(chip);
	}

	azx_free_stream_pages(chip);
	azx_free_streams(chip);
	snd_hdac_bus_exit(azx_bus(chip));

	return 0;
}

static int hda_tegra_init_clk(struct azx *chip, struct platform_device *pdev)
{
	struct hda_tegra *hda = container_of(chip, struct hda_tegra, chip);
	struct device *dev = hda->dev;

	hda->hda_clk = devm_clk_get(dev, "hda");
	if (IS_ERR(hda->hda_clk)) {
		dev_err(dev, "hda clock handle not found\n");
		return PTR_ERR(hda->hda_clk);
	}

	hda->hda2codec_2x_clk = devm_clk_get(dev, "hda2codec_2x");
	if (IS_ERR(hda->hda2codec_2x_clk)) {
		dev_err(dev, "hda2codec_2x clock handle not found\n");
		return PTR_ERR(hda->hda2codec_2x_clk);
	}

	hda->hda2hdmi_clk = devm_clk_get(dev, "hda2hdmi");
	if (IS_ERR(hda->hda2hdmi_clk)) {
		dev_err(dev, "hda2hdmi clock handle not found\n");
		return PTR_ERR(hda->hda2hdmi_clk);
	}

	return 0;
}

static int hda_tegra_init_chip(struct azx *chip, struct platform_device *pdev)
{
	struct hda_tegra *hda = container_of(chip, struct hda_tegra, chip);
	struct device *dev = hda->dev;
	struct hdac_bus *bus = azx_bus(chip);
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	hda->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hda->regs))
		return PTR_ERR(hda->regs);

	bus->remap_addr = hda->regs + HDA_BAR0;
	bus->addr = res->start + HDA_BAR0;
	hda->regs_fpci = hda->regs + HDA_DFPCI_CFG;

	hda_tegra_init(hda);

	return 0;
}

static int hda_tegra_first_init(struct azx *chip, struct platform_device *pdev)
{
	struct hda_tegra *hda = container_of(chip, struct hda_tegra, chip);
	struct hdac_bus *bus = azx_bus(chip);
	struct snd_card *card = chip->card;
	int err;
	unsigned short gcap;
	unsigned int num_sdo_lines;
	int irq_id = platform_get_irq(pdev, 0);
	const char *card_name;

	err = hda_tegra_init_chip(chip, pdev);
	if (err)
		return err;

	err = devm_request_irq(chip->card->dev, irq_id, azx_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, chip);
	if (err) {
		dev_err(chip->card->dev,
			"unable to request IRQ %d, disabling device\n",
			irq_id);
		return err;
	}
	bus->irq = irq_id;

	synchronize_irq(bus->irq);

	/*
	 * WAR to override no. of SDO lines on T194.
	 * GCAP_NSDO is bits 19:18 in T_AZA_DBG_CFG_2
	 * 0 for 1 SDO, 1 for 2 SDO, 2 for 4 SDO lines
	 */
	if (hda->cdata && hda->cdata->war_sdo_lines) {
		u32 val;

		num_sdo_lines = hda->cdata->war_sdo_lines;
		dev_info(card->dev, "Override SDO lines to %u\n",
			 num_sdo_lines);
		val = readl(hda->regs_fpci + FPCI_DBG_CFG_2);
		val &= ~FPCI_GCAP_NSDO_MASK;
		val |= ((num_sdo_lines >> 1) << FPCI_GCAP_NSDO_SHIFT) &
		       FPCI_GCAP_NSDO_MASK;
		writel(val, hda->regs_fpci + FPCI_DBG_CFG_2);
	}

	gcap = azx_readw(chip, GCAP);
	dev_dbg(card->dev, "chipset global capabilities = 0x%x\n", gcap);

	chip->align_buffer_size = 1;

	/* read number of streams from GCAP register instead of using
	 * hardcoded value
	 */
	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	if (!chip->playback_streams && !chip->capture_streams) {
		/* gcap didn't give any info, switching to old method */
		chip->playback_streams = NUM_PLAYBACK_SD;
		chip->capture_streams = NUM_CAPTURE_SD;
	}
	chip->capture_index_offset = 0;
	chip->playback_index_offset = chip->capture_streams;
	chip->num_streams = chip->playback_streams + chip->capture_streams;

	/* initialize streams */
	err = azx_init_streams(chip);
	if (err < 0) {
		dev_err(card->dev, "failed to initialize streams: %d\n", err);
		return err;
	}

	err = azx_alloc_stream_pages(chip);
	if (err < 0) {
		dev_err(card->dev, "failed to allocate stream pages: %d\n",
			err);
		return err;
	}

	/* initialize chip */
	azx_init_chip(chip, 1);

	/* codec detection */
	if (!bus->codec_mask) {
		dev_err(card->dev, "no codecs found!\n");
		return -ENODEV;
	}


	/* xavier onwards, passing card name from DT */
	if (of_property_read_string(pdev->dev.of_node, "hda,card-name",
	    &card_name))
		card_name = "tegra-hda";

	snprintf(card->driver, sizeof(card->driver), "%s", card_name);
	snprintf(card->shortname, sizeof(card->shortname), "%s", card_name);
	snprintf(card->longname, sizeof(card->longname), "%s at 0x%lx irq %i",
		 card->shortname, bus->addr, bus->irq);

	return 0;
}

/*
 * constructor
 */

static void hda_tegra_probe_work(struct work_struct *work);

static int hda_tegra_create(struct snd_card *card,
			    unsigned int driver_caps,
			    struct hda_tegra *hda)
{
	static struct snd_device_ops ops = {
		.dev_disconnect = hda_tegra_dev_disconnect,
		.dev_free = hda_tegra_dev_free,
	};
	struct azx *chip;
	int err;

	chip = &hda->chip;

	mutex_init(&chip->open_mutex);
	chip->card = card;
	chip->ops = &hda_tegra_ops;
	chip->driver_caps = driver_caps;
	chip->driver_type = driver_caps & 0xff;
	chip->dev_index = 0;
	INIT_LIST_HEAD(&chip->pcm_list);

	chip->codec_probe_mask = -1;

	chip->single_cmd = false;
	chip->snoop = true;

	INIT_WORK(&hda->probe_work, hda_tegra_probe_work);

	err = azx_bus_init(chip, NULL, &hda_tegra_io_ops);
	if (err < 0)
		return err;

	chip->bus.needs_damn_long_delay = 0;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0) {
		dev_err(card->dev, "Error creating device\n");
		return err;
	}

	return 0;
}

static const struct tegra_hda_chip_data tegra194_cdata = {
	/* GCAP reg shows 2 SDO lines which does not reflect true capability */
	.war_sdo_lines		= 4,

	/*
	 * audio can support up to 4SDO lines, but 4SDO lines can not support
	 * 32K/44.1K/48K 2 channel 16bps audio format due to legacy design
	 * limitation. With below flag, following condition is avoided while
	 * deciding number of SDO lines for audio stripe functionality.
	 * { ((num_channels * bits_per_sample) / number of SDOs) = 8 }
	 * Ref: Section 5.3.2.3 (Revision 1.0a: HD audio spec.)
	 */
	.war_sdo_bw		= true,
};

static const struct of_device_id hda_tegra_match[] = {
	{ .compatible = "nvidia,tegra30-hda" },
	{ .compatible = "nvidia,tegra194-hda", .data = &tegra194_cdata},
	{},
};
MODULE_DEVICE_TABLE(of, hda_tegra_match);

static int hda_tegra_probe(struct platform_device *pdev)
{
	unsigned int driver_flags = AZX_DCAPS_CORBRP_SELF_CLEAR;
	struct snd_card *card;
	struct azx *chip;
	struct hda_tegra *hda;
	int err;

#if defined(CONFIG_ANDROID)
	driver_flags |= AZX_DCAPS_PM_RUNTIME;
#endif

	hda = devm_kzalloc(&pdev->dev, sizeof(*hda), GFP_KERNEL);
	if (!hda)
		return -ENOMEM;
	hda->dev = &pdev->dev;
	chip = &hda->chip;

	/* chip data can be NULL for legacy hda devices */
	hda->cdata = of_device_get_match_data(&pdev->dev);

	hda->partition_id = tegra_pd_get_powergate_id(tegra_disb_pd);
	if (hda->partition_id < 0) {
		dev_err(&pdev->dev, "Failed to get hda power domain id\n");
		return -EINVAL;
	}

	err = snd_card_new(&pdev->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			   THIS_MODULE, 0, &card);
	if (err < 0) {
		dev_err(&pdev->dev, "Error creating card!\n");
		return err;
	}

	err = hda_tegra_init_clk(chip, pdev);
	if (err < 0)
		goto out_free;

	err = hda_tegra_create(card, driver_flags, hda);
	if (err < 0)
		goto out_free;

	card->private_data = chip;
	dev_set_drvdata(&pdev->dev, card);

	pm_runtime_enable(hda->dev);
	if (!azx_has_pm_runtime(chip))
		pm_runtime_forbid(hda->dev);

	schedule_work(&hda->probe_work);

	return 0;

out_free:
	snd_card_free(card);
	return err;
}

static ssize_t hda_get_pcm_device_id(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	struct hda_pcm_devices *pcm_dev = container_of(attr,
		struct hda_pcm_devices, pcm_attr);
	return snprintf(buf, PAGE_SIZE, "%d\n", pcm_dev->apcm->info->device);
}

static ssize_t hda_get_pcm_switch_name(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
	struct hda_pcm_devices *pcm_dev = container_of(attr,
		struct hda_pcm_devices, name_attr);
	return snprintf(buf, PAGE_SIZE, "%s\n", pcm_dev->switch_name);
}

static int hda_tegra_create_sysfs(struct hda_tegra *hda)
{
	struct azx *chip = &hda->chip;
	char dirname[CHAR_BUF_SIZE_MAX] = "hda_pcm_map";
	int dev_count = 0, ret = 0;
	struct azx_pcm *apcm;
	struct kobject *parent = hda->dev->kobj.parent;

	/* maintains list of all hda codecs */
	hda->hda_pcm_dev =
		(struct hda_pcm_devices *) devm_kzalloc(hda->dev,
		sizeof(struct hda_pcm_devices) * azx_bus(chip)->num_codecs,
		GFP_KERNEL);
	if (!hda->hda_pcm_dev)
		return -ENOMEM;

	hda->kobj = kobject_create_and_add(dirname, parent);
	if (!hda->kobj)
		return -ENOMEM;

	list_for_each_entry(apcm, &chip->pcm_list, list) {
		char subdirname[CHAR_BUF_SIZE_MAX];
		struct hda_pcm_devices *pcm_dev = &hda->hda_pcm_dev[dev_count];

		pcm_dev->apcm = apcm;
		snprintf(subdirname, sizeof(subdirname), "hda%d", dev_count);
		pcm_dev->kobj = kobject_create_and_add(subdirname, hda->kobj);
		if (!pcm_dev->kobj)
			return -ENOMEM;

		/* attributes for pcm device ID */
		sysfs_attr_init(&(pcm_dev->pcm_attr.attr));
		pcm_dev->pcm_attr.attr.name = "pcm_dev_id";
		pcm_dev->pcm_attr.attr.mode = 0644;
		pcm_dev->pcm_attr.show = hda_get_pcm_device_id;

		/* attributes for switch name */
		sysfs_attr_init(&(pcm_dev->name_attr.attr));
		pcm_dev->name_attr.attr.name = "switch_name";
		pcm_dev->name_attr.attr.mode = 0644;
		pcm_dev->name_attr.show = hda_get_pcm_switch_name;

		/* gets registered switch name for give dev ID
		 * TODO: may be we can create extcon node here itself and
		 * not rely on display driver
		 */
		pcm_dev->dev_id = (apcm->codec->core.vendor_id) & 0xffff;
		if (tegra_hda_get_switch_name(pcm_dev->dev_id,
			pcm_dev->switch_name) < 0) {
			dev_dbg(hda->dev, "error in getting switch name"
				" for hda_pcm_id(%d)\n", apcm->info->device);
			kobject_put(pcm_dev->kobj);
			pcm_dev->kobj = NULL;
			continue;
		}

		/* create files for read from userspace */
		ret = sysfs_create_file(pcm_dev->kobj,
			&(pcm_dev->pcm_attr.attr));
		if (ret < 0)
			break;

		ret = sysfs_create_file(pcm_dev->kobj,
			&(pcm_dev->name_attr.attr));
		if (ret < 0)
			break;

		dev_count++;
	}
	return ret;
}

static void hda_tegra_remove_sysfs(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct azx *chip = card->private_data;
	struct hda_tegra *hda = container_of(chip, struct hda_tegra, chip);
	int i;

	if (!hda || !hda->hda_pcm_dev || !hda->kobj)
		return;

	for (i = 0; i < azx_bus(chip)->num_codecs; i++) {
		struct hda_pcm_devices *pcm_dev = &hda->hda_pcm_dev[i];

		if (pcm_dev->kobj) {
			sysfs_remove_file(pcm_dev->kobj,
				&pcm_dev->pcm_attr.attr);
			sysfs_remove_file(pcm_dev->kobj,
				&pcm_dev->name_attr.attr);
			kobject_put(pcm_dev->kobj);
			pcm_dev->kobj = NULL;
		}
	}
	kobject_put(hda->kobj);
	hda->kobj = NULL;
}

static void hda_tegra_probe_work(struct work_struct *work)
{
	struct hda_tegra *hda = container_of(work, struct hda_tegra, probe_work);
	struct azx *chip = &hda->chip;
	struct platform_device *pdev = to_platform_device(hda->dev);
	struct hdac_bus *bus = azx_bus(chip);
	int err;

	pm_runtime_get_sync(hda->dev);
	err = hda_tegra_first_init(chip, pdev);
	if (err < 0)
		goto out_free;

	if (hda->cdata)
		bus->avoid_compact_sdo_bw = hda->cdata->war_sdo_bw;

	/*
	 * Below code sets watermark registers to maximum
	 * value (same as default); only applicable to T194
	 */
	if (hda->cdata && hda->cdata->set_watermark) {
		writel(FIFO_WATERMARK_VAL,
		       hda->regs_fpci + FPCI_FIFO_WATERMARK);
		writel(BUFSZ_NUM_FRAMES_VAL,
		       hda->regs_fpci + FPCI_BUFSZ_NUM_OF_FRAMES);
	}

	/* program HDA_GSC_ID to get access to APR */
	if (hda->cdata && hda->cdata->set_gsc_id)
		hda_tegra_writel(HDA_GSC_ID, hda->regs + HDA_GSC_REG);

	/* create codec instances */
	err = azx_probe_codecs(chip, HDA_MAX_CODECS);
	if (err < 0)
		goto out_free;

	err = azx_codec_configure(chip);
	if (err < 0)
		goto out_free;

	err = snd_card_register(chip->card);
	if (err < 0)
		goto out_free;

	chip->running = 1;
	snd_hda_set_power_save(&chip->bus, power_save * 1000);

	/* export pcm device mapping to userspace - needed for android */
	err = hda_tegra_create_sysfs(hda);
	if (err < 0) {
		dev_err(&pdev->dev,
			"error:%d in creating sysfs nodes for hda\n", err);
		/* free allocated resources */
		hda_tegra_remove_sysfs(&pdev->dev);
	}
out_free:
	pm_runtime_put(hda->dev);
	return; /* no error return from async probe */
}

static int hda_tegra_remove(struct platform_device *pdev)
{
	int ret;

	/* remove sysfs files */
	hda_tegra_remove_sysfs(&pdev->dev);

	ret = snd_card_free(dev_get_drvdata(&pdev->dev));

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		hda_tegra_runtime_suspend(&pdev->dev);

	return ret;
}

static void hda_tegra_shutdown(struct platform_device *pdev)
{
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct azx *chip;

	if (!card)
		return;
	chip = card->private_data;
	if (chip && chip->running)
		azx_stop_chip(chip);
}

static struct platform_driver tegra_platform_hda = {
	.driver = {
		.name = "tegra-hda",
		.pm = &hda_tegra_pm,
		.of_match_table = hda_tegra_match,
	},
	.probe = hda_tegra_probe,
	.remove = hda_tegra_remove,
	.shutdown = hda_tegra_shutdown,
};
module_platform_driver(tegra_platform_hda);

MODULE_DESCRIPTION("Tegra HDA bus driver");
MODULE_LICENSE("GPL v2");
