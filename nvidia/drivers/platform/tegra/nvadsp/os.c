/*
 * os.c
 *
 * ADSP OS management
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Copyright (C) 2014-2020, NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/tegra_nvadsp.h>
#include <soc/tegra/chip-id.h>
#include <linux/elf.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/tegra-firmwares.h>
#include <linux/reset.h>
#include <linux/poll.h>
#include <linux/version.h>

#include <asm/uaccess.h>

#include <soc/tegra/chip-id.h>

#include "ape_actmon.h"
#include "os.h"
#include "dev.h"
#include "dram_app_mem_manager.h"
#include "adsp_console_dbfs.h"
#include "hwmailbox.h"
#include "log_state.h"

#define NVADSP_ELF "adsp.elf"
#define NVADSP_FIRMWARE NVADSP_ELF

#define MAILBOX_REGION		".mbox_shared_data"
#define DEBUG_RAM_REGION	".debug_mem_logs"

/* Maximum number of LOAD MAPPINGS supported */
#define NM_LOAD_MAPPINGS 20

#define EOT	0x04 /* End of Transmission */
#define SOH	0x01 /* Start of Header */
#define BELL	0x07 /* Bell character */

#define ADSP_TAG	"\n[ADSP OS]"

#define UART_BAUD_RATE	9600

/* Intiialize with FIXED rate, once OS boots up DFS will set required freq */
#define ADSP_TO_APE_CLK_RATIO	2
/* 13.5 MHz, should be changed at bringup time */
#define APE_CLK_FIX_RATE	13500
/*
 * ADSP CLK = APE_CLK * ADSP_TO_APE_CLK_RATIO
 * or
 * ADSP CLK = APE_CLK >> ADSP_TO_APE_CLK_RATIO
 */
#define ADSP_CLK_FIX_RATE (APE_CLK_FIX_RATE * ADSP_TO_APE_CLK_RATIO)

/* total number of crashes allowed on adsp */
#define ALLOWED_CRASHES	1

#define DISABLE_MBOX2_FULL_INT	0x0
#define ENABLE_MBOX2_FULL_INT	0xFFFFFFFF

#define LOGGER_TIMEOUT		20 /* in ms */
#define ADSP_WFI_TIMEOUT	800 /* in ms */
#define LOGGER_COMPLETE_TIMEOUT	500 /* in ms */

#define SEARCH_SOH_RETRY	2

#define DUMP_BUFF 128

struct nvadsp_debug_log {
	struct device		*dev;
	char			*debug_ram_rdr;
	int			debug_ram_sz;
	int			ram_iter;
	atomic_t		is_opened;
	wait_queue_head_t	wait_queue;
	struct completion	complete;
};

struct nvadsp_os_data {
	void __iomem		*unit_fpga_reset_reg;
	const struct firmware	*os_firmware;
	struct platform_device	*pdev;
	struct global_sym_info	*adsp_glo_sym_tbl;
	void __iomem		*hwmailbox_base;
	struct resource		**dram_region;
	struct nvadsp_debug_log	logger;
	struct nvadsp_cnsl   console;
	struct work_struct	restart_os_work;
	int			adsp_num_crashes;
	bool			adsp_os_fw_loaded;
	struct mutex		fw_load_lock;
	bool			os_running;
	struct mutex		os_run_lock;
	dma_addr_t		adsp_os_addr;
	size_t			adsp_os_size;
	dma_addr_t		app_alloc_addr;
	size_t			app_size;
	int			num_start; /* registers number of time start called */
};

static struct nvadsp_os_data priv;

struct nvadsp_mappings {
	phys_addr_t da;
	void *va;
	int len;
};

static struct nvadsp_mappings adsp_map[NM_LOAD_MAPPINGS];
static int map_idx;
static struct nvadsp_mbox adsp_com_mbox;

static DECLARE_COMPLETION(entered_wfi);

static void __nvadsp_os_stop(bool);
static irqreturn_t adsp_wdt_handler(int irq, void *arg);
static irqreturn_t adsp_wfi_handler(int irq, void *arg);

/*
 * set by adsp audio driver through exported api nvadsp_set_adma_dump_reg
 * used to dump adma registers incase of failures for debug
 */
static void (*nvadsp_tegra_adma_dump_ch_reg)(void);

#ifdef CONFIG_DEBUG_FS
static int adsp_logger_open(struct inode *inode, struct file *file)
{
	struct nvadsp_debug_log *logger = inode->i_private;
	int ret = -EBUSY;
	char *start;
	int i;

	mutex_lock(&priv.os_run_lock);
	if (!priv.num_start) {
		mutex_unlock(&priv.os_run_lock);
		goto err_ret;
	}
	mutex_unlock(&priv.os_run_lock);

	/*
	 * checks if os_opened decrements to zero and if returns true. If true
	 * then there has been no open.
	*/
	if (!atomic_dec_and_test(&logger->is_opened)) {
		atomic_inc(&logger->is_opened);
		goto err_ret;
	}

	/* loop till writer is initilized with SOH */
	for (i = 0; i < SEARCH_SOH_RETRY; i++) {

		ret = wait_event_interruptible_timeout(logger->wait_queue,
			memchr(logger->debug_ram_rdr, SOH,
			logger->debug_ram_sz),
			msecs_to_jiffies(LOGGER_TIMEOUT));
		if (ret == -ERESTARTSYS)  /* check if interrupted */
			goto err;

		start = memchr(logger->debug_ram_rdr, SOH,
			logger->debug_ram_sz);
		if (start)
			break;
	}

	if (i == SEARCH_SOH_RETRY) {
		ret = -EINVAL;
		goto err;
        }

	/* maxdiff can be 0, therefore valid */
	logger->ram_iter = start - logger->debug_ram_rdr;

	file->private_data = logger;
	return 0;
err:
	/* reset to 1 so as to mention the node is free */
	atomic_set(&logger->is_opened, 1);
err_ret:
	return ret;
}


static int adsp_logger_flush(struct file *file, fl_owner_t id)
{
	struct nvadsp_debug_log *logger = file->private_data;
	struct device *dev = logger->dev;

	dev_dbg(dev, "%s\n", __func__);

	/* reset to 1 so as to mention the node is free */
	atomic_set(&logger->is_opened, 1);
	return 0;
}

static int adsp_logger_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t adsp_logger_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct nvadsp_debug_log *logger = file->private_data;
	struct device *dev = logger->dev;
	ssize_t ret_num_char = 1;
	char last_char;

loop:
	last_char = logger->debug_ram_rdr[logger->ram_iter];

	if ((last_char != EOT) && (last_char != 0)) {
#if CONFIG_ADSP_DRAM_LOG_WITH_TAG
		if ((last_char == '\n') || (last_char == '\r')) {
			size_t num_char = min(count, sizeof(ADSP_TAG) - 1);

			if (copy_to_user(buf, ADSP_TAG, num_char)) {
				dev_err(dev, "%s failed in copying tag\n", __func__);
				ret_num_char = -EFAULT;
				goto exit;
			}
			ret_num_char = num_char;

		} else
#endif
		if (copy_to_user(buf, &last_char, 1)) {
			dev_err(dev, "%s failed in copying character\n", __func__);
			ret_num_char = -EFAULT;
			goto exit;
		}

		logger->ram_iter =
			(logger->ram_iter + 1) % logger->debug_ram_sz;
		goto exit;
	}

	complete(&logger->complete);
	ret_num_char = wait_event_interruptible_timeout(logger->wait_queue,
		logger->debug_ram_rdr[logger->ram_iter] != EOT,
		msecs_to_jiffies(LOGGER_TIMEOUT));
	if (ret_num_char == -ERESTARTSYS) {
		goto exit;
	}

	goto loop;
exit:
	return ret_num_char;
}

static const struct file_operations adsp_logger_operations = {
	.read		= adsp_logger_read,
	.open		= adsp_logger_open,
	.release	= adsp_logger_release,
	.llseek		= generic_file_llseek,
	.flush		= adsp_logger_flush,
};

static int adsp_create_debug_logger(struct dentry *adsp_debugfs_root)
{
	struct nvadsp_debug_log *logger = &priv.logger;
	struct device *dev = &priv.pdev->dev;
	int ret = 0;

	if (IS_ERR_OR_NULL(adsp_debugfs_root)) {
		ret = -ENOENT;
		goto err_out;
	}

	atomic_set(&logger->is_opened, 1);
	init_waitqueue_head(&logger->wait_queue);
	init_completion(&logger->complete);
	if (!debugfs_create_file("adsp_logger", S_IRUGO,
					adsp_debugfs_root, logger,
					&adsp_logger_operations)) {
		dev_err(dev, "unable to create adsp logger debug fs file\n");
		ret = -ENOENT;
	}

err_out:
	return ret;
}
#endif

bool is_adsp_dram_addr(u64 addr)
{
	int i;
	struct resource **dram = priv.dram_region;

	for (i = 0; i < ADSP_MAX_DRAM_MAP; i++) {
		if ((dram[i]->start) && (addr >= dram[i]->start) &&
			(addr <= dram[i]->end))
			return true;
	}
	return false;
}

int nvadsp_add_load_mappings(phys_addr_t pa, void *mapping, int len)
{
	if (map_idx >= NM_LOAD_MAPPINGS)
		return -EINVAL;

	adsp_map[map_idx].da = pa;
	adsp_map[map_idx].va = mapping;
	adsp_map[map_idx].len = len;
	map_idx++;
	return 0;
}

void *nvadsp_da_to_va_mappings(u64 da, int len)
{
	void *ptr = NULL;
	int i;

	for (i = 0; i < map_idx; i++) {
		int offset = da - adsp_map[i].da;

		/* try next carveout if da is too small */
		if (offset < 0)
			continue;

		/* try next carveout if da is too large */
		if (offset + len > adsp_map[i].len)
			continue;

		ptr = adsp_map[i].va + offset;
		break;
	}
	return ptr;
}

void *nvadsp_alloc_coherent(size_t size, dma_addr_t *da, gfp_t flags)
{
	struct device *dev;
	void *va = NULL;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		goto end;
	}

	dev = &priv.pdev->dev;
	va = dma_alloc_coherent(dev, size, da, flags);
	if (!va) {
		dev_err(dev, "unable to allocate the memory for size %lu\n",
				size);
		goto end;
	}
	WARN(!is_adsp_dram_addr(*da), "bus addr %llx beyond %x\n",
				*da, UINT_MAX);
end:
	return va;
}
EXPORT_SYMBOL(nvadsp_alloc_coherent);

void nvadsp_free_coherent(size_t size, void *va, dma_addr_t da)
{
	struct device *dev;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}
	dev = &priv.pdev->dev;
	dma_free_coherent(dev, size, va, da);
}
EXPORT_SYMBOL(nvadsp_free_coherent);

struct elf32_shdr *
nvadsp_get_section(const struct firmware *fw, char *sec_name)
{
	int i;
	struct device *dev = &priv.pdev->dev;
	const u8 *elf_data = fw->data;
	struct elf32_hdr *ehdr = (struct elf32_hdr *)elf_data;
	struct elf32_shdr *shdr;
	const char *name_table;

	/* look for the resource table and handle it */
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);
	name_table = elf_data + shdr[ehdr->e_shstrndx].sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++, shdr++)
		if (!strcmp(name_table + shdr->sh_name, sec_name)) {
			dev_dbg(dev, "found the section %s\n",
					name_table + shdr->sh_name);
			return shdr;
		}
	return NULL;
}

static inline void __maybe_unused dump_global_symbol_table(void)
{
	struct device *dev = &priv.pdev->dev;
	struct global_sym_info *table = priv.adsp_glo_sym_tbl;
	int num_ent;
	int i;

	if (!table) {
		dev_err(dev, "no table not created\n");
		return;
	}
	num_ent = table[0].addr;
	dev_info(dev, "total number of entries in global symbol table %d\n",
			num_ent);

	pr_info("NAME ADDRESS TYPE\n");
	for (i = 1; i < num_ent; i++)
		pr_info("%s %x %s\n", table[i].name, table[i].addr,
			ELF32_ST_TYPE(table[i].info) == STT_FUNC ?
				"STT_FUNC" : "STT_OBJECT");
}

static int
__maybe_unused create_global_symbol_table(const struct firmware *fw)
{
	int i;
	struct device *dev = &priv.pdev->dev;
	struct elf32_shdr *sym_shdr = nvadsp_get_section(fw, ".symtab");
	struct elf32_shdr *str_shdr = nvadsp_get_section(fw, ".strtab");
	const u8 *elf_data = fw->data;
	const char *name_table;
	/* The first entry stores the number of entries in the array */
	int num_ent = 1;
	struct elf32_sym *sym;
	struct elf32_sym *last_sym;

	sym = (struct elf32_sym *)(elf_data + sym_shdr->sh_offset);
	name_table = elf_data + str_shdr->sh_offset;

	num_ent += sym_shdr->sh_size / sizeof(struct elf32_sym);
	priv.adsp_glo_sym_tbl = devm_kzalloc(dev,
		sizeof(struct global_sym_info) * num_ent, GFP_KERNEL);
	if (!priv.adsp_glo_sym_tbl)
		return -ENOMEM;

	last_sym = sym + num_ent;

	for (i = 1; sym < last_sym; sym++) {
		unsigned char info = sym->st_info;
		unsigned char type = ELF32_ST_TYPE(info);
		if ((ELF32_ST_BIND(sym->st_info) == STB_GLOBAL) &&
		((type == STT_OBJECT) || (type == STT_FUNC))) {
			char *name = priv.adsp_glo_sym_tbl[i].name;

			strlcpy(name, name_table + sym->st_name, SYM_NAME_SZ);
			priv.adsp_glo_sym_tbl[i].addr = sym->st_value;
			priv.adsp_glo_sym_tbl[i].info = info;
			i++;
		}
	}
	priv.adsp_glo_sym_tbl[0].addr = i;
	return 0;
}

struct global_sym_info * __maybe_unused find_global_symbol(const char *sym_name)
{
	struct device *dev = &priv.pdev->dev;
	struct global_sym_info *table = priv.adsp_glo_sym_tbl;
	int num_ent;
	int i;

	if (unlikely(!table)) {
		dev_err(dev, "symbol table not present\n");
		return NULL;
	}
	num_ent = table[0].addr;

	for (i = 1; i < num_ent; i++) {
		if (!strncmp(table[i].name, sym_name, SYM_NAME_SZ))
			return &table[i];
	}
	return NULL;
}

static void *get_mailbox_shared_region(const struct firmware *fw)
{
	struct device *dev;
	struct elf32_shdr *shdr;
	int addr;
	int size;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return ERR_PTR(-EINVAL);
	}

	dev = &priv.pdev->dev;

	shdr = nvadsp_get_section(fw, MAILBOX_REGION);
	if (!shdr) {
		dev_err(dev, "section %s not found\n", MAILBOX_REGION);
		return ERR_PTR(-EINVAL);
	}

	dev_dbg(dev, "the shared section is present at 0x%x\n", shdr->sh_addr);
	addr = shdr->sh_addr;
	size = shdr->sh_size;
	return nvadsp_da_to_va_mappings(addr, size);
}

static void copy_io_in_l(void *to, const void *from, int sz)
{
	int i;
	for (i = 0; i < sz; i += 4) {
		int val = *(int *)(from + i);
		writel(val, to + i);
	}
}

static int nvadsp_os_elf_load(const struct firmware *fw)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(priv.pdev);
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		void *va;
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
				phdr->p_type, da, memsz, filesz);

		va = nvadsp_da_to_va_mappings(da, filesz);
		if (!va) {
			dev_err(dev, "no va for da 0x%x filesz 0x%x\n",
					da, filesz);
			ret = -EINVAL;
			break;
		}

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
					filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
					offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (filesz) {
			if (is_adsp_dram_addr(da))
				memcpy(va, elf_data + offset, filesz);
			else if ((da == drv_data->evp_base[ADSP_EVP_BASE]) &&
				(filesz == drv_data->evp_base[ADSP_EVP_SIZE])) {

				drv_data->state.evp_ptr = va;
				memcpy(drv_data->state.evp,
					elf_data + offset, filesz);
			} else {
				dev_err(dev, "can't load mem pa:0x%x va:%p\n",
						da, va);
				ret = -EINVAL;
				break;
			}
		}
	}

	return ret;
}

static int allocate_memory_for_adsp_os(void)
{
	struct platform_device *pdev = priv.pdev;
	struct device *dev = &pdev->dev;
#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	dma_addr_t addr;
#else
	phys_addr_t addr;
#endif
	void *dram_va;
	size_t size;
	int ret = 0;

	addr = priv.adsp_os_addr;
	size = priv.adsp_os_size;
#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	dram_va = dma_alloc_at_coherent(dev, size, &addr, GFP_KERNEL);
	if (!dram_va) {
		dev_err(dev, "unable to allocate SMMU pages\n");
		ret = -ENOMEM;
		goto end;
	}
#else
	dram_va = ioremap_nocache(addr, size);
	if (!dram_va) {
		dev_err(dev, "remap failed for addr 0x%llx\n", addr);
		ret = -ENOMEM;
		goto end;
	}
#endif
	nvadsp_add_load_mappings(addr, dram_va, size);
end:
	return ret;
}

static void deallocate_memory_for_adsp_os(struct device *dev)
{
#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	void *va = nvadsp_da_to_va_mappings(priv.adsp_os_addr,
			priv.adsp_os_size);
	dma_free_coherent(dev, priv.adsp_os_addr, va, priv.adsp_os_size);
#endif
}

static void nvadsp_set_shared_mem(struct platform_device *pdev,
				  struct nvadsp_shared_mem *shared_mem,
				  uint32_t dynamic_app_support)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct nvadsp_os_args *os_args;
	enum tegra_chipid chip_id;

	shared_mem->os_args.dynamic_app_support = dynamic_app_support;
	/* set logger strcuture with required properties */
	priv.logger.debug_ram_rdr = shared_mem->os_args.logger;
	priv.logger.debug_ram_sz = sizeof(shared_mem->os_args.logger);
	priv.logger.dev = dev;
	priv.adsp_os_fw_loaded = true;

	chip_id = tegra_get_chipid();
	os_args = &shared_mem->os_args;
	os_args->chip_id = chip_id;

	drv_data->shared_adsp_os_data = shared_mem;
}

static int __nvadsp_os_secload(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	dma_addr_t addr = drv_data->adsp_mem[ACSR_ADDR];
	size_t size = drv_data->adsp_mem[ACSR_SIZE];
	struct device *dev = &pdev->dev;
	void *dram_va;

	dram_va = dma_alloc_at_coherent(dev, size, &addr, GFP_KERNEL);
	if (!dram_va) {
		dev_err(dev, "unable to allocate shared region\n");
		return -ENOMEM;
	}

	nvadsp_set_shared_mem(pdev, dram_va, 0);

	return 0;
}

static int nvadsp_firmware_load(struct platform_device *pdev)
{
	struct nvadsp_shared_mem *shared_mem;
	struct device *dev = &pdev->dev;
	const struct firmware *fw;
	int ret = 0;

	ret = request_firmware(&fw, NVADSP_FIRMWARE, dev);
	if (ret < 0) {
		dev_err(dev, "reqest firmware for %s failed with %d\n",
				NVADSP_FIRMWARE, ret);
		goto end;
	}
#ifdef CONFIG_ANDROID
	ret = create_global_symbol_table(fw);
	if (ret) {
		dev_err(dev, "unable to create global symbol table\n");
		goto release_firmware;
	}
#endif
	ret = allocate_memory_for_adsp_os();
	if (ret) {
		dev_err(dev, "unable to allocate memory for adsp os\n");
		goto release_firmware;
	}

	dev_info(dev, "Loading ADSP OS firmware %s\n", NVADSP_FIRMWARE);

	ret = nvadsp_os_elf_load(fw);
	if (ret) {
		dev_err(dev, "failed to load %s\n", NVADSP_FIRMWARE);
		goto deallocate_os_memory;
	}

	shared_mem = get_mailbox_shared_region(fw);
	nvadsp_set_shared_mem(pdev, shared_mem, 1);

	ret = dram_app_mem_init(priv.app_alloc_addr, priv.app_size);
	if (ret) {
		dev_err(dev, "Memory allocation dynamic apps failed\n");
		goto deallocate_os_memory;
	}
	priv.os_firmware = fw;

	return 0;

deallocate_os_memory:
	deallocate_memory_for_adsp_os(dev);
release_firmware:
	release_firmware(fw);
end:
	return ret;

}

int nvadsp_os_load(void)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev;
	int ret = 0;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		ret = -EINVAL;
		goto end;
	}

	mutex_lock(&priv.fw_load_lock);
	if (priv.adsp_os_fw_loaded)
		goto end;

	drv_data = platform_get_drvdata(priv.pdev);
	dev = &priv.pdev->dev;

	if (drv_data->adsp_os_secload) {
		dev_info(dev, "ADSP OS firmware already loaded\n");
		ret = __nvadsp_os_secload(priv.pdev);
	} else {
		ret = nvadsp_firmware_load(priv.pdev);
	}

	if (ret == 0) {
		priv.adsp_os_fw_loaded = true;
#ifdef CONFIG_DEBUG_FS
		wake_up(&priv.logger.wait_queue);
#endif
	}
end:
	mutex_unlock(&priv.fw_load_lock);
	return ret;
}
EXPORT_SYMBOL(nvadsp_os_load);

/*
 * Static adsp freq to emc freq lookup table
 *
 * arg:
 *	adspfreq - adsp freq in KHz
 * return:
 *	0 - min emc freq
 *	> 0 - expected emc freq at this adsp freq
 */
u32 adsp_to_emc_freq(u32 adspfreq)
{
	/*
	 * Vote on memory bus frequency based on adsp frequency
	 * cpu rate is in kHz, emc rate is in Hz
	 */
	if (adspfreq >= 204800)
		return 102000;	/* adsp >= 204.8 MHz, emc 102 MHz */
	else
		return 0;		/* emc min */
}

static int nvadsp_set_ape_emc_freq(struct nvadsp_drv_data *drv_data)
{
	unsigned long ape_emc_freq;
	struct device *dev = &priv.pdev->dev;
	int ret;

#ifdef CONFIG_TEGRA_ADSP_DFS
	 /* pass adsp freq in KHz. adsp_emc_freq in Hz */
	ape_emc_freq = adsp_to_emc_freq(drv_data->adsp_freq / 1000) * 1000;
#else
	ape_emc_freq = drv_data->ape_emc_freq * 1000; /* in Hz */
#endif
	dev_dbg(dev, "requested adsp cpu freq %luKHz",
		 drv_data->adsp_freq / 1000);
	dev_dbg(dev, "emc freq %luHz\n", ape_emc_freq / 1000);

	/*
	 * ape_emc_freq is not required to set if adsp_freq
	 * is lesser than 204.8 MHz
	 */

	if (!ape_emc_freq)
		return 0;

	ret = tegra_bwmgr_set_emc(drv_data->bwmgr, ape_emc_freq * 1000,
				  TEGRA_BWMGR_SET_EMC_FLOOR);
	if (ret)
		dev_err(dev, "failed to set emc freq rate:%d\n", ret);
	dev_dbg(dev, "ape.emc freq %luKHz\n",
		tegra_bwmgr_get_emc_rate() / 1000);

	return ret;
}

static int nvadsp_set_ape_freq(struct nvadsp_drv_data *drv_data)
{
	unsigned long ape_freq = drv_data->ape_freq * 1000; /* in Hz*/
	struct device *dev = &priv.pdev->dev;
	int ret;

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ape_freq = drv_data->adsp_freq / ADSP_TO_APE_CLK_RATIO;
#endif
	dev_dbg(dev, "ape freq %luKHz", ape_freq / 1000);

	if (!ape_freq)
		return 0;

	ret = clk_set_rate(drv_data->ape_clk, ape_freq);

	dev_dbg(dev, "ape freq %luKHz\n",
		clk_get_rate(drv_data->ape_clk) / 1000);
	return ret;
}

static int nvadsp_t210_set_clks_and_prescalar(struct nvadsp_drv_data *drv_data)
{
	struct nvadsp_shared_mem *shared_mem = drv_data->shared_adsp_os_data;
	struct nvadsp_os_args *os_args = &shared_mem->os_args;
	struct device *dev = &priv.pdev->dev;
	unsigned long max_adsp_freq;
	unsigned long adsp_freq;
	u32 max_index;
	u32 cur_index;
	int ret = 0;

	adsp_freq = drv_data->adsp_freq * 1000; /* in Hz*/

	max_adsp_freq = clk_round_rate(drv_data->adsp_cpu_abus_clk,
				ULONG_MAX);
	max_index = max_adsp_freq / MIN_ADSP_FREQ;
	cur_index = adsp_freq / MIN_ADSP_FREQ;


	if (!adsp_freq)
		/* Set max adsp boot freq */
		cur_index = max_index;

	if (adsp_freq % MIN_ADSP_FREQ) {
		if (cur_index >= max_index)
			cur_index = max_index;
		else
			cur_index++;
	} else if (cur_index >= max_index)
		cur_index = max_index;

	/*
	 * timer interval = (prescalar + 1) * (count + 1) / periph_freq
	 * therefore for 0 count,
	 * 1 / TIMER_CLK_HZ =  (prescalar + 1) / periph_freq
	 * Hence, prescalar = periph_freq / TIMER_CLK_HZ - 1
	 */
	os_args->timer_prescalar = cur_index - 1;

	adsp_freq = cur_index * MIN_ADSP_FREQ;

	ret = clk_set_rate(drv_data->adsp_cpu_abus_clk, adsp_freq);
	if (ret)
		goto end;

	drv_data->adsp_freq = adsp_freq / 1000; /* adsp_freq in KHz*/
	drv_data->adsp_freq_hz = adsp_freq;

	/* adspos uses os_args->adsp_freq_hz for EDF */
	os_args->adsp_freq_hz = adsp_freq;

end:
	dev_dbg(dev, "adsp cpu freq %luKHz\n",
		clk_get_rate(drv_data->adsp_cpu_abus_clk) / 1000);
	dev_dbg(dev, "timer prescalar %x\n", os_args->timer_prescalar);

	return ret;
}

static int nvadsp_set_adsp_clks(struct nvadsp_drv_data *drv_data)
{
	struct nvadsp_shared_mem *shared_mem = drv_data->shared_adsp_os_data;
	struct nvadsp_os_args *os_args = &shared_mem->os_args;
	struct platform_device *pdev = drv_data->pdev;
	struct device *dev = &pdev->dev;
	unsigned long max_adsp_freq;
	unsigned long adsp_freq;
	int ret = 0;

	adsp_freq = drv_data->adsp_freq_hz; /* in Hz*/

	/* round rate shall be used with adsp parent clk i.e. aclk */
	max_adsp_freq = clk_round_rate(drv_data->aclk_clk, ULONG_MAX);

	/* Set max adsp boot freq */
	if (!adsp_freq)
		adsp_freq = max_adsp_freq;

	/* set rate shall be used with adsp parent clk i.e. aclk */
	ret = clk_set_rate(drv_data->aclk_clk, adsp_freq);
	if (ret) {
		dev_err(dev, "setting adsp_freq:%luHz failed.\n", adsp_freq);
		dev_err(dev, "max_adsp_freq:%luHz\n", max_adsp_freq);
		goto end;
	}

	drv_data->adsp_freq = adsp_freq / 1000; /* adsp_freq in KHz*/
	drv_data->adsp_freq_hz = adsp_freq;

	/* adspos uses os_args->adsp_freq_hz for EDF */
	os_args->adsp_freq_hz = adsp_freq;
end:
	dev_dbg(dev, "adsp cpu freq %luKHz\n",
		clk_get_rate(drv_data->adsp_clk) / 1000);
	return ret;
}

static int __deassert_adsp(struct nvadsp_drv_data *d)
{
	struct platform_device *pdev = d->pdev;
	struct device *dev = &pdev->dev;
	int ret = 0;

	/*
	 * The ADSP_ALL reset in BPMP-FW is overloaded to de-assert
	 * all 7 resets i.e. ADSP, ADSPINTF, ADSPDBG, ADSPNEON, ADSPPERIPH,
	 * ADSPSCU and ADSPWDT resets. The BPMP-FW also takes care
	 * of specific de-assert sequence and delays between them.
	 * So de-resetting only ADSP reset is sufficient to de-reset
	 * all ADSP sub-modules.
	 */
	ret = reset_control_deassert(d->adspall_rst);
	if (ret)
		dev_err(dev, "failed to deassert adsp\n");

	return ret;
}

static int nvadsp_deassert_adsp(struct nvadsp_drv_data *drv_data)
{
	int ret = -EINVAL;

	if (drv_data->deassert_adsp)
		ret = drv_data->deassert_adsp(drv_data);

	return ret;
}

static int __assert_adsp(struct nvadsp_drv_data *d)
{
	struct platform_device *pdev = d->pdev;
	struct device *dev = &pdev->dev;
	int ret = 0;

	/*
	 * The ADSP_ALL reset in BPMP-FW is overloaded to assert
	 * all 7 resets i.e. ADSP, ADSPINTF, ADSPDBG, ADSPNEON,
	 * ADSPPERIPH, ADSPSCU and ADSPWDT resets. So resetting
	 * only ADSP reset is sufficient to reset all ADSP sub-modules.
	 */
	ret = reset_control_assert(d->adspall_rst);
	if (ret)
		dev_err(dev, "failed to assert adsp\n");

	return ret;
}

static int nvadsp_assert_adsp(struct nvadsp_drv_data *drv_data)
{
	int ret = -EINVAL;

	if (drv_data->assert_adsp)
		ret = drv_data->assert_adsp(drv_data);

	return ret;
}

static int nvadsp_set_boot_freqs(struct nvadsp_drv_data *drv_data)
{
	struct device *dev = &priv.pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	/* on Unit-FPGA do not set clocks, return success */
	if (drv_data->adsp_unit_fpga)
		return 0;

	if (of_device_is_compatible(node, "nvidia,tegra210-adsp")) {
		if (drv_data->adsp_cpu_abus_clk) {
			ret = nvadsp_t210_set_clks_and_prescalar(drv_data);
			if (ret)
				goto end;
		} else {
			ret = -EINVAL;
			goto end;
		}
	} else {
		if (drv_data->adsp_clk) {
			ret = nvadsp_set_adsp_clks(drv_data);
			if (ret)
				goto end;
		} else {
			ret = -EINVAL;
			goto end;
		}
	}

	if (drv_data->ape_clk) {
		ret = nvadsp_set_ape_freq(drv_data);
		if (ret)
			goto end;
	}
	if (drv_data->bwmgr) {
		ret = nvadsp_set_ape_emc_freq(drv_data);
		if (ret)
			goto end;
	}
end:
	return ret;
}

static int wait_for_adsp_os_load_complete(void)
{
	struct device *dev = &priv.pdev->dev;
	uint32_t data;
	status_t ret;

	ret = nvadsp_mbox_recv(&adsp_com_mbox, &data,
			true, ADSP_OS_LOAD_TIMEOUT);
	if (ret) {
		dev_err(dev, "ADSP OS loading timed out\n");
		goto end;
	}
	dev_dbg(dev, "ADSP has been %s\n",
		data == ADSP_OS_BOOT_COMPLETE ? "BOOTED" : "RESUMED");

	switch (data) {
	case ADSP_OS_BOOT_COMPLETE:
		ret = load_adsp_static_apps();
		break;
	case ADSP_OS_RESUME:
	default:
		break;
	}
end:
	return ret;
}

static int __nvadsp_os_start(void)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev;
	int ret = 0;

	dev = &priv.pdev->dev;
	drv_data = platform_get_drvdata(priv.pdev);


	dev_dbg(dev, "ADSP is booting on %s\n",
		drv_data->adsp_unit_fpga ? "UNIT-FPGA" : "SILICON");

	nvadsp_assert_adsp(drv_data);

	if (!drv_data->adsp_os_secload) {
		dev_dbg(dev, "Copying EVP...\n");
		copy_io_in_l(drv_data->state.evp_ptr,
			     drv_data->state.evp,
			     AMC_EVP_SIZE);
	}

	dev_dbg(dev, "Setting freqs\n");
	ret = nvadsp_set_boot_freqs(drv_data);
	if (ret) {
		dev_err(dev, "failed to set boot freqs\n");
		goto end;
	}

	dev_dbg(dev, "De-asserting adsp\n");
	ret = nvadsp_deassert_adsp(drv_data);
	if (ret) {
		dev_err(dev, "failed to deassert ADSP\n");
		goto end;
	}

	dev_dbg(dev, "Waiting for ADSP OS to boot up...\n");

	ret = wait_for_adsp_os_load_complete();
	if (ret) {
		dev_err(dev, "Unable to start ADSP OS\n");
		goto end;
	}
	dev_dbg(dev, "ADSP OS boot up... Done!\n");

#ifdef CONFIG_TEGRA_ADSP_DFS
	ret = adsp_dfs_core_init(priv.pdev);
	if (ret) {
		dev_err(dev, "adsp dfs initialization failed\n");
		goto err;
	}
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ret = ape_actmon_init(priv.pdev);
	if (ret) {
		dev_err(dev, "ape actmon initialization failed\n");
		goto err;
	}
#endif

#ifdef CONFIG_TEGRA_ADSP_CPUSTAT
	ret = adsp_cpustat_init(priv.pdev);
	if (ret) {
		dev_err(dev, "adsp cpustat initialisation failed\n");
		goto err;
	}
#endif
end:
	return ret;

#if defined(CONFIG_TEGRA_ADSP_DFS) || defined(CONFIG_TEGRA_ADSP_CPUSTAT)
err:
	__nvadsp_os_stop(true);
	return ret;
#endif
}

static void dump_adsp_logs(void)
{
	int i = 0;
	char buff[DUMP_BUFF] = { };
	int buff_iter = 0;
	char last_char;
	struct nvadsp_debug_log *logger = &priv.logger;
	struct device *dev = &priv.pdev->dev;
	char *ptr = logger->debug_ram_rdr;

	dev_err(dev, "Dumping ADSP logs ........\n");

	for (i = 0; i < logger->debug_ram_sz; i++) {
		last_char = *(ptr + i);
		if ((last_char != EOT) && (last_char != 0)) {
			if ((last_char == '\n') || (last_char == '\r') ||
					(buff_iter == DUMP_BUFF)) {
				dev_err(dev, "[ADSP OS] %s\n", buff);
				memset(buff, 0, sizeof(buff));
				buff_iter = 0;
			} else {
				buff[buff_iter++] = last_char;
			}
		}
	}
	dev_err(dev, "End of ADSP log dump  .....\n");
}

static void print_agic_irq_states(void)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(priv.pdev);
	int start_irq = drv_data->chip_data->start_irq;
	int end_irq = drv_data->chip_data->end_irq;
	struct device *dev = &priv.pdev->dev;
	int i;

	for (i = start_irq; i < end_irq; i++) {
		dev_info(dev, "irq %d is %s and %s\n", i,
		tegra_agic_irq_is_pending(i) ?
			"pending" : "not pending",
		tegra_agic_irq_is_active(i) ?
			"active" : "not active");
	}
}

static void print_arm_mode_regs(void)
{
	struct nvadsp_exception_context *excep_context;
	struct arm_fault_frame_shared *shared_frame;
	struct arm_mode_regs_shared *shared_regs;
	struct nvadsp_shared_mem *shared_mem;
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_drv_data *drv_data;

	drv_data = platform_get_drvdata(priv.pdev);
	shared_mem = drv_data->shared_adsp_os_data;
	excep_context = &shared_mem->exception_context;
	shared_frame = &excep_context->frame;
	shared_regs = &excep_context->regs;

	dev_err(dev, "dumping arm mode register data...\n");
	dev_err(dev, "%c fiq r13 0x%08x r14 0x%08x\n",
		((shared_frame->spsr & MODE_MASK) == MODE_FIQ) ? '*' : ' ',
		shared_regs->fiq_r13, shared_regs->fiq_r14);
	dev_err(dev, "%c irq r13 0x%08x r14 0x%08x\n",
		((shared_frame->spsr & MODE_MASK) == MODE_IRQ) ? '*' : ' ',
		shared_regs->irq_r13, shared_regs->irq_r14);
	dev_err(dev, "%c svc r13 0x%08x r14 0x%08x\n",
		((shared_frame->spsr & MODE_MASK) == MODE_SVC) ? '*' : ' ',
		shared_regs->svc_r13, shared_regs->svc_r14);
	dev_err(dev, "%c und r13 0x%08x r14 0x%08x\n",
		((shared_frame->spsr & MODE_MASK) == MODE_UND) ? '*' : ' ',
		shared_regs->und_r13, shared_regs->und_r14);
	dev_err(dev, "%c sys r13 0x%08x r14 0x%08x\n",
		((shared_frame->spsr & MODE_MASK) == MODE_SYS) ? '*' : ' ',
		shared_regs->sys_r13, shared_regs->sys_r14);
	dev_err(dev, "%c abt r13 0x%08x r14 0x%08x\n",
		((shared_frame->spsr & MODE_MASK) == MODE_ABT) ? '*' : ' ',
		shared_regs->abt_r13, shared_regs->abt_r14);
}

static void print_arm_fault_frame(void)
{
	struct nvadsp_exception_context *excep_context;
	struct arm_fault_frame_shared *shared_frame;
	struct nvadsp_shared_mem *shared_mem;
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_drv_data *drv_data;

	drv_data = platform_get_drvdata(priv.pdev);
	shared_mem = drv_data->shared_adsp_os_data;
	excep_context = &shared_mem->exception_context;
	shared_frame = &excep_context->frame;

	dev_err(dev, "dumping fault frame...\n");
	dev_err(dev, "r0  0x%08x r1  0x%08x r2  0x%08x r3  0x%08x\n",
		 shared_frame->r[0], shared_frame->r[1], shared_frame->r[2],
		 shared_frame->r[3]);
	dev_err(dev, "r4  0x%08x r5  0x%08x r6  0x%08x r7  0x%08x\n",
		 shared_frame->r[4], shared_frame->r[5], shared_frame->r[6],
		 shared_frame->r[7]);
	dev_err(dev, "r8  0x%08x r9  0x%08x r10 0x%08x r11 0x%08x\n",
		 shared_frame->r[8], shared_frame->r[9], shared_frame->r[10],
		 shared_frame->r[11]);
	dev_err(dev, "r12 0x%08x usp 0x%08x ulr 0x%08x pc  0x%08x\n",
		 shared_frame->r[12], shared_frame->usp, shared_frame->ulr,
		 shared_frame->pc);
	dev_err(dev, "spsr 0x%08x\n", shared_frame->spsr);

}

static void dump_thread_name(struct platform_device *pdev, u32 val)
{
	dev_info(&pdev->dev, "%s: adsp current thread: %c%c%c%c\n",
		 __func__,
		 (val >> 24) & 0xFF, (val >> 16) & 0xFF,
		 (val >> 8) & 0xFF, (val >> 0) & 0xFF);
}

static void dump_irq_num(struct platform_device *pdev, u32 val)
{
	dev_info(&pdev->dev, "%s: adsp current/last irq : %d\n",
		 __func__, val);
}

static void get_adsp_state(void)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev;
	uint32_t val;
	char *msg;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}

	drv_data = platform_get_drvdata(priv.pdev);
	dev = &priv.pdev->dev;

	if (drv_data->chip_data->adsp_state_hwmbox == -1) {
		dev_info(dev, "%s: No state hwmbox available\n", __func__);
		return;
	}

	val = hwmbox_readl(drv_data->chip_data->adsp_state_hwmbox);
	dev_info(dev, "%s: adsp state hwmbox value: 0x%X\n", __func__, val);

	switch (val) {

	case ADSP_LOADER_MAIN_ENTRY:
		msg = "loader_main: entry to loader_main";
		break;
	case ADSP_LOADER_MAIN_CACHE_DISABLE_COMPLETE:
		msg = "loader_main: Cache has been disabled";
		break;
	case ADSP_LOADER_MAIN_CONFIGURE_MMU_COMPLETE:
		msg = "loader_main: MMU configuration is complete";
		break;
	case ADSP_LOADER_MAIN_CACHE_ENABLE_COMPLETE:
		msg = "loader_main: Cache has been enabled";
		break;
	case ADSP_LOADER_MAIN_FPU_ENABLE_COMPLETE:
		msg = "loader_main: FPU has been enabled";
		break;
	case ADSP_LOADER_MAIN_DECOMPRESSION_COMPLETE:
		msg = "loader_main: ADSP FW decompression is complete";
		break;
	case ADSP_LOADER_MAIN_EXIT:
		msg = "loader_main: exiting loader_main function";
		break;

	case ADSP_START_ENTRY_AT_RESET:
		msg = "start: ADSP is at reset";
		break;
	case ADSP_START_CPU_EARLY_INIT:
		msg = "start: ADSP to do cpu_early_init";
		break;
	case ADSP_START_FIRST_BOOT:
		msg = "start: ADSP is booting for first time,"
				"initializing DATA and clearing BSS";
		break;
	case ADSP_START_LK_MAIN_ENTRY:
		msg = "start: ADSP about to enter lk_main";
		break;

	case ADSP_LK_MAIN_ENTRY:
		msg = "lk_main: entry to lk_main";
		break;
	case ADSP_LK_MAIN_EARLY_THREAD_INIT_COMPLETE:
		msg = "lk_main: early_thread_init has been completed";
		break;
	case ADSP_LK_MAIN_EARLY_ARCH_INIT_COMPLETE:
		msg = "lk_main: early_arch_init has been completed";
		break;
	case ADSP_LK_MAIN_EARLY_PLATFORM_INIT_COMPLETE:
		msg = "lk_main: early_platform_init has been completed";
		break;
	case ADSP_LK_MAIN_EARLY_TARGET_INIT_COMPLETE:
		msg = "lk_main: early_target_init has been completed";
		break;
	case ADSP_LK_MAIN_CONSTRUCTOR_INIT_COMPLETE:
		msg = "lk_main: constructors has been called";
		break;
	case ADSP_LK_MAIN_HEAP_INIT_COMPLETE:
		msg = "lk_main: heap has been initialized";
		break;
	case ADSP_LK_MAIN_KERNEL_INIT_COMPLETE:
		msg = "lk_main: ADSP kernel has been initialized";
		break;
	case ADSP_LK_MAIN_CPU_RESUME_ENTRY:
		msg = "lk_main: ADSP is about to resume from suspend";
		break;

	case ADSP_BOOTSTRAP2_ARCH_INIT_COMPLETE:
		msg = "bootstrap2: ADSP arch_init is complete";
		break;
	case ADSP_BOOTSTRAP2_PLATFORM_INIT_COMPLETE:
		msg = "bootstrap2: platform has been initialized";
		break;
	case ADSP_BOOTSTRAP2_TARGET_INIT_COMPLETE:
		msg = "bootstrap2: target has been initialized";
		break;
	case ADSP_BOOTSTRAP2_APP_MODULE_INIT_COMPLETE:
		msg = "bootstrap2: APP modules initialized";
		break;
	case ADSP_BOOTSTRAP2_APP_INIT_COMPLETE:
		msg = "bootstrap2: APP init is complete";
		break;
	case ADSP_BOOTSTRAP2_STATIC_APP_INIT_COMPLETE:
		msg = "bootstrap2: Static apps has been initialized";
		break;
	case ADSP_BOOTSTRAP2_OS_LOAD_COMPLETE:
		msg = "bootstrap2: ADSP OS successfully loaded";
		break;
	case ADSP_SUSPEND_BEGINS:
		msg = "suspend: begins";
		break;
	case ADSP_SUSPEND_MBX_SEND_COMPLETE:
		msg = "suspend: mbox send complete";
		break;
	case ADSP_SUSPEND_DISABLED_TIMERS:
		msg = "suspend: timers disabled";
		break;
	case ADSP_SUSPEND_DISABLED_INTS:
		msg = "suspend: interrupts disabled";
		break;
	case ADSP_SUSPEND_ARAM_SAVED:
		msg = "suspend: aram saved";
		break;
	case ADSP_SUSPEND_AMC_SAVED:
		msg = "suspend: amc saved";
		break;
	case ADSP_SUSPEND_AMISC_SAVED:
		msg = "suspend: amisc saved";
		break;
	case ADSP_SUSPEND_L1_CACHE_DISABLED:
		msg = "suspend: l1 cache disabled";
		break;
	case ADSP_SUSPEND_L2_CACHE_DISABLED:
		msg = "suspend: l2 cache disabled";
		break;
	case ADSP_RESUME_ADSP:
		msg = "resume: beings";
		break;
	case ADSP_RESUME_AMISC_RESTORED:
		msg = "resume: amisc restored";
		break;
	case ADSP_RESUME_AMC_RESTORED:
		msg = "resume: amc restored";
		break;
	case ADSP_RESUME_ARAM_RESTORED:
		msg = "resume: aram restored";
		break;
	case ADSP_RESUME_COMPLETE:
		msg = "resume: complete";
		break;
	case ADSP_WFI_ENTER:
		msg = "WFI: Entering WFI";
		break;
	case ADSP_WFI_EXIT:
		msg = "WFI: Exiting WFI, Failed to Enter";
		break;
	case ADSP_DFS_MBOX_RECV:
		msg = "DFS: mbox received";
		break;
	case ADSP_DFS_MBOX_SENT:
		msg = "DFS: mbox sent";
		break;
	default:
		msg = "Unrecognized ADSP state!!";
		break;
	}

	dev_info(dev, "%s: %s\n", __func__, msg);

	val = hwmbox_readl(drv_data->chip_data->adsp_thread_hwmbox);
	dump_thread_name(priv.pdev, val);

	val = hwmbox_readl(drv_data->chip_data->adsp_irq_hwmbox);
	dump_irq_num(priv.pdev, val);
}


void dump_adsp_sys(void)
{
	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}

	dump_adsp_logs();
	dump_mailbox_regs();
	print_arm_fault_frame();
	print_arm_mode_regs();
	get_adsp_state();
	if (nvadsp_tegra_adma_dump_ch_reg)
		(*nvadsp_tegra_adma_dump_ch_reg)();
	print_agic_irq_states();
}
EXPORT_SYMBOL(dump_adsp_sys);

static void nvadsp_free_os_interrupts(struct nvadsp_os_data *priv)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(priv->pdev);
	int wdt_virq = drv_data->agic_irqs[WDT_VIRQ];
	int wfi_virq = drv_data->agic_irqs[WFI_VIRQ];
	struct device *dev = &priv->pdev->dev;

	devm_free_irq(dev, wdt_virq, priv);
	devm_free_irq(dev, wfi_virq, priv);
}

static int nvadsp_setup_os_interrupts(struct nvadsp_os_data *priv)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(priv->pdev);
	int wdt_virq = drv_data->agic_irqs[WDT_VIRQ];
	int wfi_virq = drv_data->agic_irqs[WFI_VIRQ];
	struct device *dev = &priv->pdev->dev;
	int ret;

	ret = devm_request_irq(dev, wdt_virq, adsp_wdt_handler,
			IRQF_TRIGGER_RISING, "adsp watchdog", priv);
	if (ret) {
		dev_err(dev, "failed to get adsp watchdog interrupt\n");
		goto end;
	}

	ret = devm_request_irq(dev, wfi_virq, adsp_wfi_handler,
			IRQF_TRIGGER_RISING, "adsp wfi", priv);
	if (ret) {
		dev_err(dev, "cannot request for wfi interrupt\n");
		goto free_interrupts;
	}

	writel(DISABLE_MBOX2_FULL_INT,
	       priv->hwmailbox_base + drv_data->chip_data->hwmb.hwmbox2_reg);

 end:

	return ret;

 free_interrupts:
	nvadsp_free_os_interrupts(priv);
	return ret;
}

static void free_interrupts(struct nvadsp_os_data *priv)
{
	nvadsp_free_os_interrupts(priv);
	nvadsp_free_hwmbox_interrupts(priv->pdev);
	nvadsp_free_amc_interrupts(priv->pdev);
}

static int setup_interrupts(struct nvadsp_os_data *priv)
{
	int ret;

	ret = nvadsp_setup_os_interrupts(priv);
	if (ret)
		goto err;

	ret = nvadsp_setup_hwmbox_interrupts(priv->pdev);
	if (ret)
		goto free_os_interrupts;
	ret = nvadsp_setup_amc_interrupts(priv->pdev);
	if (ret)
		goto free_hwmbox_interrupts;

	return ret;

 free_hwmbox_interrupts:
	nvadsp_free_hwmbox_interrupts(priv->pdev);
 free_os_interrupts:
	nvadsp_free_os_interrupts(priv);
 err:
	return ret;
}

void nvadsp_set_adma_dump_reg(void (*cb_adma_regdump)(void))
{
	nvadsp_tegra_adma_dump_ch_reg = cb_adma_regdump;
	pr_info("%s: callback for adma reg dump is sent to %p\n",
		__func__, nvadsp_tegra_adma_dump_ch_reg);
}
EXPORT_SYMBOL(nvadsp_set_adma_dump_reg);

int nvadsp_os_start(void)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev;
	int ret = 0;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		ret = -EINVAL;
		goto end;
	}

	drv_data = platform_get_drvdata(priv.pdev);
	dev = &priv.pdev->dev;

	/* check if fw is loaded then start the adsp os */
	if (!priv.adsp_os_fw_loaded) {
		dev_err(dev, "Call to nvadsp_os_load not made\n");
		ret = -EINVAL;
		goto end;
	}

	mutex_lock(&priv.os_run_lock);
	/* if adsp is started/running exit gracefully */
	if (priv.os_running)
		goto unlock;

#ifdef CONFIG_PM
	ret = pm_runtime_get_sync(&priv.pdev->dev);
	if (ret < 0)
		goto unlock;
#endif
	ret = setup_interrupts(&priv);
	if (ret < 0)
		goto unlock;

	ret = __nvadsp_os_start();
	if (ret) {
		priv.os_running = drv_data->adsp_os_running = false;
		/* if start fails call pm suspend of adsp driver */
		dev_err(dev, "adsp failed to boot with ret = %d\n", ret);
		dump_adsp_sys();
		free_interrupts(&priv);
#ifdef CONFIG_PM
		pm_runtime_put_sync(&priv.pdev->dev);
#endif
		goto unlock;

	}
	priv.os_running = drv_data->adsp_os_running = true;
	priv.num_start++;
#if defined(CONFIG_TEGRA_ADSP_FILEIO)
	if (!drv_data->adspff_init) {
		ret = adspff_init(priv.pdev);
		if (!ret)
			drv_data->adspff_init = true;
	}
#endif

#ifdef CONFIG_TEGRA_ADSP_LPTHREAD
	if (!drv_data->lpthread_initialized) {
		ret = adsp_lpthread_entry(priv.pdev);
		if (ret)
			dev_err(dev, "adsp_lpthread_entry failed ret = %d\n",
					ret);
	}
#endif

	drv_data->adsp_os_suspended = false;
#ifdef CONFIG_DEBUG_FS
	wake_up(&priv.logger.wait_queue);
#endif

#ifdef CONFIG_TEGRA_ADSP_LPTHREAD
	adsp_lpthread_set_suspend(drv_data->adsp_os_suspended);
#endif

unlock:
	mutex_unlock(&priv.os_run_lock);
end:
	return ret;
}
EXPORT_SYMBOL(nvadsp_os_start);

static int __nvadsp_os_suspend(void)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_drv_data *drv_data;
	int ret;

	drv_data = platform_get_drvdata(priv.pdev);

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ape_actmon_exit(priv.pdev);
#endif

#ifdef CONFIG_TEGRA_ADSP_DFS
	adsp_dfs_core_exit(priv.pdev);
#endif

#ifdef CONFIG_TEGRA_ADSP_CPUSTAT
	adsp_cpustat_exit(priv.pdev);
#endif

	ret = nvadsp_mbox_send(&adsp_com_mbox, ADSP_OS_SUSPEND,
			       NVADSP_MBOX_SMSG, true, UINT_MAX);
	if (ret) {
		dev_err(dev, "failed to send with adsp com mbox\n");
		goto out;
	}

	dev_dbg(dev, "Waiting for ADSP OS suspend...\n");
	ret = wait_for_completion_timeout(&entered_wfi,
		msecs_to_jiffies(ADSP_WFI_TIMEOUT));
	if (WARN_ON(ret <= 0)) {
		dev_err(dev, "Unable to suspend ADSP OS err = %d\n", ret);
		ret = (ret < 0) ? ret : -ETIMEDOUT;
		goto out;
	}
	ret = 0;
	dev_dbg(dev, "ADSP OS suspended!\n");

	drv_data->adsp_os_suspended = true;

#ifdef CONFIG_TEGRA_ADSP_LPTHREAD
	adsp_lpthread_set_suspend(drv_data->adsp_os_suspended);
#endif

	nvadsp_assert_adsp(drv_data);

 out:
	return ret;
}

static void __nvadsp_os_stop(bool reload)
{
	const struct firmware *fw = priv.os_firmware;
	struct nvadsp_drv_data *drv_data;
	struct device *dev;
	int err = 0;

	dev = &priv.pdev->dev;
	drv_data = platform_get_drvdata(priv.pdev);

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ape_actmon_exit(priv.pdev);
#endif

#ifdef CONFIG_TEGRA_ADSP_DFS
	adsp_dfs_core_exit(priv.pdev);
#endif

#ifdef CONFIG_TEGRA_ADSP_CPUSTAT
	adsp_cpustat_exit(priv.pdev);
#endif
#if defined(CONFIG_TEGRA_ADSP_FILEIO)
	if (drv_data->adspff_init) {
		adspff_exit();
		drv_data->adspff_init = false;
	}
#endif

	writel(ENABLE_MBOX2_FULL_INT,
	       priv.hwmailbox_base + drv_data->chip_data->hwmb.hwmbox2_reg);
	err = wait_for_completion_timeout(&entered_wfi,
		msecs_to_jiffies(ADSP_WFI_TIMEOUT));
	writel(DISABLE_MBOX2_FULL_INT,
	       priv.hwmailbox_base + drv_data->chip_data->hwmb.hwmbox2_reg);

	/*
	 * ADSP needs to be in WFI/WFE state to properly reset it.
	 * However, when ADSPOS is getting stopped on error path,
	 * it cannot gaurantee that ADSP is in WFI/WFE state.
	 * Reset it in either case. On failure, whole APE reset is
	 * required (happens on next APE power domain cycle).
	 */
	nvadsp_assert_adsp(drv_data);

	/* Don't reload ADSPOS if ADSP state is not WFI/WFE */
	if (WARN_ON(err <= 0)) {
		dev_err(dev, "%s: unable to enter wfi state err = %d\n",
			__func__, err);
		goto end;
	}

	if (reload && !drv_data->adsp_os_secload) {
		struct nvadsp_debug_log *logger = &priv.logger;

#ifdef CONFIG_DEBUG_FS
		wake_up(&logger->wait_queue);
		/* wait for LOGGER_TIMEOUT to complete filling the buffer */
		wait_for_completion_timeout(&logger->complete,
			msecs_to_jiffies(LOGGER_COMPLETE_TIMEOUT));
#endif
		/*
		 * move ram iterator to 0, since after restart the iterator
		 * will be pointing to initial position of start.
		 */
		logger->debug_ram_rdr[0] = EOT;
		logger->ram_iter = 0;
		/* load a fresh copy of adsp.elf */
		if (nvadsp_os_elf_load(fw))
			dev_err(dev, "failed to reload %s\n", NVADSP_FIRMWARE);
	}

 end:
	return;
}


void nvadsp_os_stop(void)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}

	dev = &priv.pdev->dev;
	drv_data = platform_get_drvdata(priv.pdev);

	mutex_lock(&priv.os_run_lock);
	/* check if os is running else exit */
	if (!priv.os_running)
		goto end;

	__nvadsp_os_stop(true);

	priv.os_running = drv_data->adsp_os_running = false;

	free_interrupts(&priv);
#ifdef CONFIG_PM
	if (pm_runtime_put_sync(dev) < 0)
		dev_err(dev, "failed in pm_runtime_put_sync\n");
#endif
end:
	mutex_unlock(&priv.os_run_lock);
}
EXPORT_SYMBOL(nvadsp_os_stop);

int nvadsp_os_suspend(void)
{
	struct nvadsp_drv_data *drv_data;
	int ret = -EINVAL;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		goto end;
	}

	drv_data = platform_get_drvdata(priv.pdev);

	mutex_lock(&priv.os_run_lock);
	/* check if os is running else exit */
	if (!priv.os_running) {
		ret = 0;
		goto unlock;
	}
	ret = __nvadsp_os_suspend();
	if (!ret) {
#ifdef CONFIG_PM
		struct device *dev = &priv.pdev->dev;

		free_interrupts(&priv);
		ret = pm_runtime_put_sync(&priv.pdev->dev);
		if (ret < 0)
			dev_err(dev, "failed in pm_runtime_put_sync\n");
#endif
		priv.os_running = drv_data->adsp_os_running = false;
	} else {
		dev_err(&priv.pdev->dev, "suspend failed with %d\n", ret);
		dump_adsp_sys();
	}
unlock:
	mutex_unlock(&priv.os_run_lock);
end:
	return ret;
}
EXPORT_SYMBOL(nvadsp_os_suspend);

static void nvadsp_os_restart(struct work_struct *work)
{
	struct nvadsp_os_data *data =
		container_of(work, struct nvadsp_os_data, restart_os_work);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(data->pdev);
	int wdt_virq = drv_data->agic_irqs[WDT_VIRQ];
	int wdt_irq = drv_data->chip_data->wdt_irq;
	struct device *dev = &data->pdev->dev;

	disable_irq(wdt_virq);
	dump_adsp_sys();
	nvadsp_os_stop();

	if (tegra_agic_irq_is_active(wdt_irq)) {
		dev_info(dev, "wdt interrupt is active hence clearing\n");
		tegra_agic_clear_active(wdt_irq);
	}

	if (tegra_agic_irq_is_pending(wdt_irq)) {
		dev_info(dev, "wdt interrupt is pending hence clearing\n");
		tegra_agic_clear_pending(wdt_irq);
	}

	dev_info(dev, "wdt interrupt is not pending or active...enabling\n");
	enable_irq(wdt_virq);

	data->adsp_num_crashes++;
	if (data->adsp_num_crashes >= ALLOWED_CRASHES) {
		/* making pdev NULL so that externally start is not called */
		priv.pdev = NULL;
		dev_crit(dev, "ADSP has crashed too many times(%d)\n",
			 data->adsp_num_crashes);
		return;
	}

	if (nvadsp_os_start())
		dev_crit(dev, "Unable to restart ADSP OS\n");
}

static irqreturn_t adsp_wfi_handler(int irq, void *arg)
{
	struct nvadsp_os_data *data = arg;
	struct device *dev = &data->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);
	complete(&entered_wfi);

	return IRQ_HANDLED;
}

static irqreturn_t adsp_wdt_handler(int irq, void *arg)
{
	struct nvadsp_os_data *data = arg;
	struct nvadsp_drv_data *drv_data;
	struct device *dev = &data->pdev->dev;

	drv_data = platform_get_drvdata(data->pdev);

	drv_data->adsp_crashed = true;
	wake_up_interruptible(&drv_data->adsp_health_waitq);

	if (!drv_data->adsp_unit_fpga) {
		dev_crit(dev, "ADSP OS Hanged or Crashed! Restarting...\n");
		schedule_work(&data->restart_os_work);
	} else {
		dev_crit(dev, "ADSP OS Hanged or Crashed!\n");
	}
	return IRQ_HANDLED;
}

void nvadsp_get_os_version(char *buf, int buf_size)
{
	struct nvadsp_drv_data *drv_data;
	struct nvadsp_shared_mem *shared_mem;
	struct nvadsp_os_info *os_info;

	memset(buf, 0, buf_size);

	if (!priv.pdev)
		return;

	drv_data = platform_get_drvdata(priv.pdev);
	shared_mem = drv_data->shared_adsp_os_data;
	if (shared_mem) {
		os_info = &shared_mem->os_info;
		strlcpy(buf, os_info->version, buf_size);
	} else {
		strlcpy(buf, "unavailable", buf_size);
	}
}
EXPORT_SYMBOL(nvadsp_get_os_version);

#ifdef CONFIG_DEBUG_FS
static int show_os_version(struct seq_file *s, void *data)
{
	char ver_buf[MAX_OS_VERSION_BUF] = "";

	nvadsp_get_os_version(ver_buf, MAX_OS_VERSION_BUF);
	seq_printf(s, "version=\"%s\"\n", ver_buf);

	return 0;
}

static int os_version_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_os_version, inode->i_private);
}

static const struct file_operations version_fops = {
	.open = os_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#define RO_MODE S_IRUSR

static int adsp_create_os_version(struct dentry *adsp_debugfs_root)
{
	struct device *dev = &priv.pdev->dev;
	struct dentry *d;

	d = debugfs_create_file("adspos_version", RO_MODE, adsp_debugfs_root,
				NULL, &version_fops);
	if (!d) {
		dev_err(dev, "failed to create adsp_version\n");
		return -EINVAL;
	}
	return 0;
}

static unsigned int adsp_health_poll(struct file *file,
			poll_table *wait)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(priv.pdev);

	poll_wait(file, &drv_data->adsp_health_waitq, wait);

	if (drv_data->adsp_crashed)
		return POLLIN | POLLRDNORM;

	return 0;
}

static const struct file_operations adsp_health_fops = {
	.poll = adsp_health_poll,
};

static int adsp_create_adsp_health(struct dentry *adsp_debugfs_root)
{
	struct device *dev = &priv.pdev->dev;
	struct dentry *d;

	d = debugfs_create_file("adsp_health", RO_MODE, adsp_debugfs_root,
				NULL, &adsp_health_fops);
	if (!d) {
		dev_err(dev, "failed to create adsp_health\n");
		return -EINVAL;
	}
	return 0;
}
#endif

static ssize_t tegrafw_read_adsp(struct device *dev,
				char *data, size_t size)
{
	nvadsp_get_os_version(data, size);
	return strlen(data);
}

int __init nvadsp_os_probe(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	uint64_t dma_mask;
#endif
	uint16_t com_mid = ADSP_COM_MBOX_ID;
	int ret = 0;

	priv.unit_fpga_reset_reg = drv_data->base_regs[UNIT_FPGA_RST];
	priv.hwmailbox_base = drv_data->base_regs[hwmb_reg_idx()];
	priv.dram_region = drv_data->dram_region;

	priv.adsp_os_addr = drv_data->adsp_mem[ADSP_OS_ADDR];
	priv.adsp_os_size = drv_data->adsp_mem[ADSP_OS_SIZE];
	priv.app_alloc_addr = drv_data->adsp_mem[ADSP_APP_ADDR];
	priv.app_size = drv_data->adsp_mem[ADSP_APP_SIZE];

	if (of_device_is_compatible(dev->of_node, "nvidia,tegra210-adsp")) {
		drv_data->assert_adsp = __assert_adsp;
		drv_data->deassert_adsp = __deassert_adsp;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	ret = of_property_read_u64(dev->of_node, "dma-mask", &dma_mask);
	if (ret) {
		dev_err(&pdev->dev, "Missing property dma-mask\n");
		goto end;
	} else {
		dma_set_mask_and_coherent(&pdev->dev, dma_mask);
	}
#endif
	ret = nvadsp_os_init(pdev);
	if (ret) {
		dev_err(dev, "failed to init os\n");
		goto end;
	}

	ret = nvadsp_mbox_open(&adsp_com_mbox, &com_mid, "adsp_com_mbox",
			       NULL, NULL);
	if (ret) {
		dev_err(dev, "failed to open adsp com mbox\n");
		goto end;
	}

	INIT_WORK(&priv.restart_os_work, nvadsp_os_restart);
	mutex_init(&priv.fw_load_lock);
	mutex_init(&priv.os_run_lock);

	priv.pdev = pdev;
#ifdef CONFIG_DEBUG_FS
	priv.logger.dev = &pdev->dev;
	if (adsp_create_debug_logger(drv_data->adsp_debugfs_root))
		dev_err(dev, "unable to create adsp debug logger file\n");

#ifdef CONFIG_TEGRA_ADSP_CONSOLE
	priv.console.dev = &pdev->dev;
	if (adsp_create_cnsl(drv_data->adsp_debugfs_root, &priv.console))
		dev_err(dev, "unable to create adsp console file\n");
#endif /* CONFIG_TEGRA_ADSP_CONSOLE */

	if (adsp_create_os_version(drv_data->adsp_debugfs_root))
		dev_err(dev, "unable to create adsp_version file\n");

	if (adsp_create_adsp_health(drv_data->adsp_debugfs_root))
		dev_err(dev, "unable to create adsp_health file\n");

	drv_data->adsp_crashed = false;
	init_waitqueue_head(&drv_data->adsp_health_waitq);

#endif /* CONFIG_DEBUG_FS */

	devm_tegrafw_register(dev, "APE", TFW_DONT_CACHE,
			tegrafw_read_adsp, NULL);
end:
	return ret;
}
