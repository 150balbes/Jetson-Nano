/*
 * mods_krnl.c - This file is part of NVIDIA MODS kernel driver.
 *
 * Copyright (c) 2008-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA MODS kernel driver is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * NVIDIA MODS kernel driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NVIDIA MODS kernel driver.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "mods_internal.h"

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/screen_info.h>
#include <linux/uaccess.h>
#ifdef MODS_HAS_CONSOLE_LOCK
#   include <linux/console.h>
#   include <linux/kd.h>
#   include <linux/tty.h>
#   include <linux/console_struct.h>
#   include <linux/vt_kern.h>
#endif

/***********************************************************************
 * mods_krnl_* functions, driver interfaces called by the Linux kernel *
 ***********************************************************************/
static int mods_krnl_open(struct inode *, struct file *);
static int mods_krnl_close(struct inode *, struct file *);
static unsigned int mods_krnl_poll(struct file *, poll_table *);
static int mods_krnl_mmap(struct file *, struct vm_area_struct *);
static long mods_krnl_ioctl(struct file *, unsigned int, unsigned long);

#ifdef MODS_HAS_SRIOV
static int mods_pci_sriov_configure(struct pci_dev *dev, int numvfs);
#endif

/* character driver entry points */
static const struct file_operations mods_fops = {
	.owner			= THIS_MODULE,
	.open			= mods_krnl_open,
	.release		= mods_krnl_close,
	.poll			= mods_krnl_poll,
	.mmap			= mods_krnl_mmap,
	.unlocked_ioctl = mods_krnl_ioctl,
#if defined(HAVE_COMPAT_IOCTL)
	.compat_ioctl	= mods_krnl_ioctl,
#endif
};

#define DEVICE_NAME "mods"

struct miscdevice mods_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &mods_fops
};

#if defined(MODS_CAN_REGISTER_PCI_DEV)
static pci_ers_result_t mods_pci_error_detected(struct pci_dev *,
						enum pci_channel_state);
static pci_ers_result_t mods_pci_mmio_enabled(struct pci_dev *);
static void mods_pci_resume(struct pci_dev *);

struct pci_error_handlers mods_pci_error_handlers = {
	.error_detected	= mods_pci_error_detected,
	.mmio_enabled	= mods_pci_mmio_enabled,
	.resume		= mods_pci_resume,
};

static const struct pci_device_id mods_pci_table[] = {
	{
		.vendor		= PCI_VENDOR_ID_NVIDIA,
		.device		= PCI_ANY_ID,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.class		= (PCI_CLASS_DISPLAY_VGA << 8),
		.class_mask	= ~0
	},
	{
		.vendor		= PCI_VENDOR_ID_NVIDIA,
		.device		= PCI_ANY_ID,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.class		= (PCI_CLASS_DISPLAY_3D << 8),
		.class_mask	= ~0
	},
	{ }
};

static int mods_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	mods_debug_printk(DEBUG_PCI, "probed vendor %x device %x devfn %x\n",
		dev->vendor, dev->device, dev->devfn);

	return 0;
}

struct pci_driver mods_pci_driver = {
	.name            = DEVICE_NAME,
	.id_table        = mods_pci_table,
	.probe           = mods_pci_probe,
	.err_handler     = &mods_pci_error_handlers,
#ifdef MODS_HAS_SRIOV
	.sriov_configure = mods_pci_sriov_configure,
#endif
};
#endif

/***********************************************
 * module wide parameters and access functions *
 * used to avoid globalization of variables    *
 ***********************************************/

static int debug;
static int multi_instance = MODS_MULTI_INSTANCE_DEFAULT_VALUE;
static u32 access_token = MODS_ACCESS_TOKEN_NONE;

#ifdef MODS_HAS_SRIOV
static int mods_pci_sriov_configure(struct pci_dev *dev, int numvfs)
{
	int rv = 0;

	LOG_ENT();

	mods_debug_printk(DEBUG_PCI,
		"(numvfs=%d, totalvfs=%d, dev->is_physfn=%d)\n",
		numvfs,
		pci_sriov_get_totalvfs(dev),
		dev->is_physfn);

	if (numvfs > 0) {
		rv = pci_enable_sriov(dev, numvfs);
		if (rv) {
			mods_error_printk(
				"pci_enable_sriov failed error %d\n", rv);
			return rv;
		}

		rv = numvfs;
	} else {
		pci_disable_sriov(dev);
		rv = 0;
	}

	LOG_EXT();
	return rv;
}

static int esc_mods_set_num_vf(struct file *pfile, struct MODS_SET_NUM_VF *p)
{
	int rv = 0;
	struct pci_dev *dev;
	unsigned int devfn;

	LOG_ENT();

	/* Get the PCI device structure for the specified device from kernel */
	devfn = PCI_DEVFN(p->dev.device, p->dev.function);
	dev = MODS_PCI_GET_SLOT(p->dev.domain, p->dev.bus, devfn);
	if (!dev) {
		mods_error_printk(
			"unknown dev %04x:%x:%02x.%x\n",
			(unsigned int)p->dev.domain,
			(unsigned int)p->dev.bus,
			(unsigned int)p->dev.device,
			(unsigned int)p->dev.function);
		LOG_EXT();
		return -EINVAL;
	}

	rv = mods_pci_sriov_configure(dev, p->numvfs);

	LOG_EXT();

	return rv;
}

static int esc_mods_set_total_vf(struct file *pfile, struct MODS_SET_NUM_VF *p)
{
	int rv = 0;
	struct pci_dev *dev;
	unsigned int devfn;

	LOG_ENT();

	mods_debug_printk(DEBUG_PCI,
		"pci_sriov_set_totalvfs(totalvfs=%d)\n", p->numvfs);

	/* Get the PCI device structure for the specified device from kernel */
	devfn = PCI_DEVFN(p->dev.device, p->dev.function);
	dev = MODS_PCI_GET_SLOT(p->dev.domain, p->dev.bus, devfn);
	if (!dev) {
		mods_error_printk(
			"unknown dev %04x:%x:%02x.%x\n",
			(unsigned int)p->dev.domain,
			(unsigned int)p->dev.bus,
			(unsigned int)p->dev.device,
			(unsigned int)p->dev.function);
		LOG_EXT();
		return -EINVAL;
	}

	rv = pci_sriov_set_totalvfs(dev, p->numvfs);

	if (rv) {
		mods_error_printk(
			"pci_sriov_set_totalvfs failed error %d\n", rv);
		return rv;
	}

	LOG_EXT();

	return rv;
}
#endif

#if defined(CONFIG_PPC64)
static int ppc_tce_bypass = MODS_PPC_TCE_BYPASS_ON;

void mods_set_ppc_tce_bypass(int bypass)
{
	ppc_tce_bypass = bypass;
}

int mods_get_ppc_tce_bypass(void)
{
	return ppc_tce_bypass;
}
#endif

void mods_set_debug_level(int mask)
{
	debug = mask;
}

int mods_get_debug_level(void)
{
	return debug;
}

int mods_check_debug_level(int mask)
{
	return ((debug & mask) == mask) ? 1 : 0;
}

void mods_set_multi_instance(int mi)
{
	multi_instance = (mi > 0) ? 1 : -1;
}

int mods_get_multi_instance(void)
{
	return multi_instance > 0;
}

u32 mods_get_access_token(void)
{
	return access_token;
}

static int mods_set_access_token(u32 tok)
{
	/* When setting a null token, the existing token must match the
	 * provided token, when setting a non-null token the existing token
	 * must be null, use atomic compare/exchange to set it
	 */
	u32 req_old_token =
	    (tok == MODS_ACCESS_TOKEN_NONE) ?
		access_token : MODS_ACCESS_TOKEN_NONE;

	if (cmpxchg(&access_token, req_old_token, tok) != req_old_token)
		return -EFAULT;
	return OK;
}

static int mods_check_access_token(struct file *fp)
{
	struct mods_client *client = fp->private_data;

	if (client->access_token != mods_get_access_token())
		return -EFAULT;

	return OK;
}

/******************************
 * INIT/EXIT MODULE FUNCTIONS *
 ******************************/
static int __init mods_init_module(void)
{
	int rc;

	LOG_ENT();

	mods_init_irq();

	rc = misc_register(&mods_dev);
	if (rc < 0)
		return -EBUSY;

#if defined(MODS_CAN_REGISTER_PCI_DEV)
	rc = pci_register_driver(&mods_pci_driver);
	if (rc < 0)
		return -EBUSY;
#endif

#if defined(MODS_HAS_CLOCK)
	mods_init_clock_api();
#endif

	rc = mods_create_debugfs(&mods_dev);
	if (rc < 0)
		return rc;

	rc = mods_init_dmabuf();
	if (rc < 0)
		return rc;

#if defined(MODS_TEGRA)
	/* tegra prod */
	mods_tegra_prod_init(&mods_dev);
#endif

	mods_info_printk("*** WARNING: DIAGNOSTIC DRIVER LOADED ***\n");
	mods_info_printk("driver loaded, version %x.%02x\n",
			 (MODS_DRIVER_VERSION>>8),
			 (MODS_DRIVER_VERSION&0xFF));

	if (debug)
		mods_info_printk("debug level 0x%x\n", debug);

	LOG_EXT();
	return OK;
}

static void __exit mods_exit_module(void)
{
	LOG_ENT();

	mods_exit_dmabuf();

	mods_remove_debugfs();

	mods_cleanup_irq();

#if defined(MODS_CAN_REGISTER_PCI_DEV)
	pci_unregister_driver(&mods_pci_driver);
#endif

	misc_deregister(&mods_dev);

#if defined(MODS_HAS_CLOCK)
	mods_shutdown_clock_api();
#endif

	mods_info_printk("driver unloaded\n");
	LOG_EXT();
}

/***************************
 * KERNEL INTERFACE SET UP *
 ***************************/
module_init(mods_init_module);
module_exit(mods_exit_module);

MODULE_LICENSE("GPL");

#define STRING_VALUE(x) #x
#define MAKE_MODULE_VERSION(x, y) STRING_VALUE(x) "." STRING_VALUE(y)
MODULE_VERSION(MAKE_MODULE_VERSION(MODS_DRIVER_VERSION_MAJOR,
				   MODS_DRIVER_VERSION_MINOR));

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug,
"debug bitflags (2=ioctl 4=pci 8=acpi 16=irq 32=mem 64=fun +256=detailed)");

module_param(multi_instance, int, 0644);
MODULE_PARM_DESC(multi_instance,
	"allows more than one client to simultaneously open the driver");

#if defined(CONFIG_PPC64)
module_param(ppc_tce_bypass, int, 0644);
MODULE_PARM_DESC(ppc_tce_bypass,
	"PPC TCE bypass (0=sys default, 1=force bypass, 2=force non bypass)");
#endif

/********************
 * HELPER FUNCTIONS *
 ********************/
static void mods_disable_all_devices(struct mods_client *client)
{
#ifdef CONFIG_PCI
	if (unlikely(mutex_lock_interruptible(mods_get_irq_mutex())))
		return;

	while (client->enabled_devices != 0) {
		struct en_dev_entry *old = client->enabled_devices;

		mods_disable_device(old->dev);
		client->enabled_devices = old->next;
		kfree(old);
	}

	mutex_unlock(mods_get_irq_mutex());
#else
	WARN_ON(client->enabled_devices != 0);
#endif
}

static int mods_resume_console(struct file *pfile);

/*********************
 * MAPPING FUNCTIONS *
 *********************/
static int mods_register_mapping(
	struct file          *fp,
	struct MODS_MEM_INFO *p_mem_info,
	u64                   dma_addr,
	u64                   virtual_address,
	u64                   mapping_length)
{
	struct SYS_MAP_MEMORY *p_map_mem;
	struct mods_client    *client = fp->private_data;

	LOG_ENT();

	p_map_mem = kmalloc(sizeof(*p_map_mem), GFP_KERNEL | __GFP_NORETRY);
	if (unlikely(!p_map_mem)) {
		LOG_EXT();
		return -ENOMEM;
	}
	memset(p_map_mem, 0, sizeof(*p_map_mem));

	p_map_mem->dma_addr = dma_addr;
	p_map_mem->virtual_addr = virtual_address;
	p_map_mem->mapping_length = mapping_length;
	p_map_mem->p_mem_info = p_mem_info;

	list_add(&p_map_mem->list, &client->mem_map_list);

	mods_debug_printk(DEBUG_MEM_DETAILED,
	    "map alloc %p as %p: phys 0x%llx, virt 0x%llx, size 0x%llx\n",
	    p_mem_info, p_map_mem, dma_addr, virtual_address, mapping_length);

	LOG_EXT();
	return OK;
}

static void mods_unregister_mapping(struct file *fp, u64 virtual_address)
{
	struct SYS_MAP_MEMORY *p_map_mem;
	struct mods_client    *client = fp->private_data;
	struct list_head      *head   = &client->mem_map_list;
	struct list_head      *iter;

	LOG_ENT();

	list_for_each(iter, head) {
		p_map_mem = list_entry(iter, struct SYS_MAP_MEMORY, list);

		if (p_map_mem->virtual_addr == virtual_address) {
			/* remove from the list */
			list_del(iter);

			/* free our data struct which keeps track of mapping */
			kfree(p_map_mem);

			return;
		}
	}
	LOG_EXT();
}

static void mods_unregister_all_mappings(struct file *fp)
{
	struct SYS_MAP_MEMORY *p_map_mem;
	struct mods_client    *client = fp->private_data;
	struct list_head      *head   = &client->mem_map_list;
	struct list_head      *iter;
	struct list_head      *tmp;

	LOG_ENT();

	list_for_each_safe(iter, tmp, head) {
		p_map_mem = list_entry(iter, struct SYS_MAP_MEMORY, list);
		mods_unregister_mapping(fp, p_map_mem->virtual_addr);
	}

	LOG_EXT();
}

static pgprot_t mods_get_prot(u32 mem_type, pgprot_t prot)
{
	switch (mem_type) {
	case MODS_MEMORY_CACHED:
		return prot;

	case MODS_MEMORY_UNCACHED:
		return MODS_PGPROT_UC(prot);

	case MODS_MEMORY_WRITECOMBINE:
		return MODS_PGPROT_WC(prot);

	default:
		mods_warning_printk("unsupported memory type: %u\n",
				    mem_type);
		return prot;
	}
}

static pgprot_t mods_get_prot_for_range(struct file *fp, u64 dma_addr,
					u64 size, pgprot_t prot)
{
	struct mods_client *client = fp->private_data;

	if ((dma_addr == client->mem_type.dma_addr) &&
		(size == client->mem_type.size)) {

		return mods_get_prot(client->mem_type.type, prot);
	}
	return prot;
}

const char *mods_get_prot_str(u32 mem_type)
{
	switch (mem_type) {
	case MODS_MEMORY_CACHED:
		return "WB";

	case MODS_MEMORY_UNCACHED:
		return "UC";

	case MODS_MEMORY_WRITECOMBINE:
		return "WC";

	default:
		return "unknown";
	}
}

static const char *mods_get_prot_str_for_range(struct file *fp,
					       u64          dma_addr,
					       u64          size)
{
	struct mods_client *client = fp->private_data;

	if ((dma_addr == client->mem_type.dma_addr) &&
		(size == client->mem_type.size)) {

		return mods_get_prot_str(client->mem_type.type);
	}
	return "default";
}
/********************
 * PCI ERROR FUNCTIONS *
 ********************/
#if defined(MODS_CAN_REGISTER_PCI_DEV)
static pci_ers_result_t mods_pci_error_detected(struct pci_dev *dev,
						enum pci_channel_state state)
{
	mods_debug_printk(DEBUG_PCI,
			  "pci_error_detected %04x:%x:%02x.%x\n",
			  pci_domain_nr(dev->bus),
			  dev->bus->number,
			  PCI_SLOT(dev->devfn),
			  PCI_FUNC(dev->devfn));

	return PCI_ERS_RESULT_CAN_RECOVER;
}

static pci_ers_result_t mods_pci_mmio_enabled(struct pci_dev *dev)
{
	mods_debug_printk(DEBUG_PCI,
			  "pci_mmio_enabled %04x:%x:%02x.%x\n",
			  pci_domain_nr(dev->bus),
			  dev->bus->number,
			  PCI_SLOT(dev->devfn),
			  PCI_FUNC(dev->devfn));

	return PCI_ERS_RESULT_NEED_RESET;
}

static void mods_pci_resume(struct pci_dev *dev)
{
	mods_debug_printk(DEBUG_PCI,
			  "pci_resume %04x:%x:%02x.%x\n",
			  pci_domain_nr(dev->bus),
			  dev->bus->number,
			  PCI_SLOT(dev->devfn),
			  PCI_FUNC(dev->devfn));
}
#endif

/********************
 * KERNEL FUNCTIONS *
 ********************/
static void mods_krnl_vma_open(struct vm_area_struct *vma)
{
	struct mods_vm_private_data *vma_private_data;

	LOG_ENT();
	mods_debug_printk(DEBUG_MEM_DETAILED,
			  "open vma, virt 0x%lx, phys 0x%llx\n",
			  vma->vm_start,
			  (u64)(MODS_VMA_PGOFF(vma) << PAGE_SHIFT));

	if (MODS_VMA_PRIVATE(vma)) {
		vma_private_data = MODS_VMA_PRIVATE(vma);
		atomic_inc(&vma_private_data->usage_count);
	}
	LOG_EXT();
}

static void mods_krnl_vma_close(struct vm_area_struct *vma)
{
	LOG_ENT();

	if (MODS_VMA_PRIVATE(vma)) {
		struct mods_vm_private_data *vma_private_data
			= MODS_VMA_PRIVATE(vma);
		if (atomic_dec_and_test(&vma_private_data->usage_count)) {
			struct mods_client *client =
				vma_private_data->fp->private_data;

			if (unlikely(mutex_lock_interruptible(
						&client->mtx))) {
				LOG_EXT();
				return;
			}

			/* we need to unregister the mapping */
			mods_unregister_mapping(vma_private_data->fp,
						vma->vm_start);
			mods_debug_printk(DEBUG_MEM_DETAILED,
					  "closed vma, virt 0x%lx\n",
					  vma->vm_start);
			MODS_VMA_PRIVATE(vma) = NULL;
			kfree(vma_private_data);

			mutex_unlock(&client->mtx);
		}
	}
	LOG_EXT();
}

static const struct vm_operations_struct mods_krnl_vm_ops = {
	.open	= mods_krnl_vma_open,
	.close	= mods_krnl_vma_close
};

static int mods_krnl_open(struct inode *ip, struct file *fp)
{
	struct mods_client *client;

	LOG_ENT();

	client = mods_alloc_client();
	if (client == NULL) {
		mods_error_printk("too many clients\n");
		LOG_EXT();
		return -EBUSY;
	}

	fp->private_data = client;

	mods_info_printk("driver opened\n");
	LOG_EXT();
	return OK;
}

static int mods_krnl_close(struct inode *ip, struct file *fp)
{
	struct mods_client *client    = fp->private_data;
	u8                  client_id = client->client_id;
	int                 ret = OK;

	LOG_ENT();

	WARN_ON(!is_client_id_valid(client_id));
	if (!is_client_id_valid(client_id)) {
		LOG_EXT();
		return -EINVAL;
	}

	mods_free_client_interrupts(client);

	mods_resume_console(fp);

	mods_unregister_all_mappings(fp);
	ret = mods_unregister_all_alloc(fp);
	if (ret)
		mods_error_printk("failed to free all memory\n");

#if defined(CONFIG_PPC64)
	ret = mods_unregister_all_ppc_tce_bypass(fp);
	if (ret)
		mods_error_printk("failed to restore dma bypass\n");

	ret = mods_unregister_all_nvlink_sysmem_trained(fp);
	if (ret)
		mods_error_printk("failed to free nvlink trained\n");
#endif

	mods_disable_all_devices(client);

	mods_free_client(client_id);

	mods_info_printk("driver closed\n");
	LOG_EXT();
	return ret;
}

static unsigned int mods_krnl_poll(struct file *fp, poll_table *wait)
{
	unsigned int mask = 0;
	struct mods_client *client = fp->private_data;
	u8 client_id = get_client_id(fp);
	int access_tok_ret = mods_check_access_token(fp);

	if (access_tok_ret < 0)
		return access_tok_ret;

	if (!(fp->f_flags & O_NONBLOCK)) {
		mods_debug_printk(DEBUG_ISR_DETAILED, "poll wait\n");
		poll_wait(fp, &client->interrupt_event, wait);
	}
	/* if any interrupts pending then check intr, POLLIN on irq */
	mask |= mods_irq_event_check(client_id);
	mods_debug_printk(DEBUG_ISR_DETAILED, "poll mask 0x%x\n", mask);
	return mask;
}

static int mods_krnl_map_inner(struct file *fp, struct vm_area_struct *vma);

static int mods_krnl_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct mods_vm_private_data *vma_private_data;
	int access_tok_ret;

	LOG_ENT();

	access_tok_ret = mods_check_access_token(fp);
	if (access_tok_ret < 0) {
		LOG_EXT();
		return access_tok_ret;
	}

	vma->vm_ops = &mods_krnl_vm_ops;

	vma_private_data = kmalloc(sizeof(*vma_private_data),
				   GFP_KERNEL | __GFP_NORETRY);
	if (unlikely(!vma_private_data)) {
		LOG_EXT();
		return -ENOMEM;
	}

	/* set private data for vm_area_struct */
	atomic_set(&vma_private_data->usage_count, 0);
	vma_private_data->fp = fp;
	MODS_VMA_PRIVATE(vma) = vma_private_data;

	/* call for the first time open function */
	mods_krnl_vma_open(vma);

	{
		int ret = OK;
		struct mods_client *client = fp->private_data;

		if (unlikely(mutex_lock_interruptible(&client->mtx)))
			ret = -EINTR;
		else {
			ret = mods_krnl_map_inner(fp, vma);
			mutex_unlock(&client->mtx);
		}
		LOG_EXT();
		return ret;
	}
}

static int mods_krnl_map_inner(struct file *fp, struct vm_area_struct *vma)
{
	u64                   req_pa     = MODS_VMA_OFFSET(vma);
	struct MODS_MEM_INFO *p_mem_info = mods_find_alloc(fp, req_pa);
	u32                   req_pages  = MODS_VMA_SIZE(vma) >> PAGE_SHIFT;

	if ((req_pa             & ~PAGE_MASK) != 0 ||
	    (MODS_VMA_SIZE(vma) & ~PAGE_MASK) != 0) {
		mods_error_printk("requested mapping is not page-aligned\n");
		return -EINVAL;
	}

	/* system memory */
	if (p_mem_info) {
		u32                     first, i;
		struct MODS_PHYS_CHUNK *pt         = p_mem_info->pages;
		u32                     have_pages = 0;
		unsigned long           map_va     = 0;
		const pgprot_t          prot       =
		    mods_get_prot(p_mem_info->cache_type, vma->vm_page_prot);

		/* Find the beginning of the requested range */
		for (first = 0; first < p_mem_info->max_chunks; first++) {
			u64 dma_addr;

			if (!pt[first].allocated)
				continue;
			dma_addr = pt[first].dma_addr;
			if ((req_pa >= dma_addr) &&
			    (req_pa <  dma_addr + (PAGE_SIZE << pt->order))) {
				break;
			}
		}

		if (first == p_mem_info->max_chunks) {
			mods_error_printk("can't satisfy requested mapping\n");
			return -EINVAL;
		}

		/* Count how many remaining pages we have in the allocation */
		for (i = first; i < p_mem_info->max_chunks; i++) {
			if (!pt[i].allocated)
				break;
			if (i == first) {
				u64 aoffs      = req_pa - pt[i].dma_addr;
				u32 skip_pages = aoffs >> PAGE_SHIFT;

				have_pages -= skip_pages;
			}
			have_pages += 1U << pt[i].order;
		}

		if (have_pages < req_pages) {
			mods_error_printk("requested mapping exceeds bounds\n");
			return -EINVAL;
		}

		/* Map pages into VA space */
		map_va     = vma->vm_start;
		have_pages = req_pages;
		for (i = first; have_pages > 0; i++) {
			u64 map_pa    = MODS_DMA_TO_PHYS(pt[i].dma_addr);
			u32 map_size  = PAGE_SIZE << pt[i].order;
			u32 map_pages = 1U << pt[i].order;

			if (!pt[i].allocated)
				break;

			if (i == first) {
				u64 aoffs = req_pa - pt[i].dma_addr;

				map_pa    += aoffs;
				map_size  -= aoffs;
				map_pages -= aoffs >> PAGE_SHIFT;
			}

			if (map_pages > have_pages) {
				map_size  = have_pages << PAGE_SHIFT;
				map_pages = have_pages;
			}

			mods_debug_printk(DEBUG_MEM_DETAILED,
			    "remap va 0x%lx pfn 0x%x size 0x%x pages 0x%x\n",
			    map_va, (unsigned int)(map_pa>>PAGE_SHIFT),
			    map_size, map_pages);

			if (remap_pfn_range(vma,
					    map_va,
					    map_pa>>PAGE_SHIFT,
					    map_size,
					    prot)) {
				mods_error_printk("failed to map memory\n");
				return -EAGAIN;
			}

			map_va     += map_size;
			have_pages -= map_pages;
		}

		/* MODS_VMA_OFFSET(vma) can change so it can't be used
		 * to register the mapping
		 */
		mods_register_mapping(fp,
				      p_mem_info,
				      pt[first].dma_addr,
				      vma->vm_start,
				      MODS_VMA_SIZE(vma));

	} else {
		/* device memory */

		mods_debug_printk(DEBUG_MEM,
		    "map dev: phys 0x%llx, virt 0x%lx, size 0x%lx, %s\n",
		    req_pa,
		    (unsigned long)vma->vm_start,
		    (unsigned long)MODS_VMA_SIZE(vma),
		    mods_get_prot_str_for_range(fp, req_pa,
						MODS_VMA_SIZE(vma)));

		if (io_remap_pfn_range(
				vma,
				vma->vm_start,
				req_pa>>PAGE_SHIFT,
				MODS_VMA_SIZE(vma),
				mods_get_prot_for_range(
					fp,
					req_pa,
					MODS_VMA_SIZE(vma),
					vma->vm_page_prot))) {
			mods_error_printk("failed to map device memory\n");
			return -EAGAIN;
		}

		mods_register_mapping(fp,
				      NULL,
				      req_pa,
				      vma->vm_start,
				      MODS_VMA_SIZE(vma));
	}
	return OK;
}

#if !defined(CONFIG_ARM) && !defined(CONFIG_ARM64) && !defined(CONFIG_PPC64)
static int mods_get_screen_info(struct MODS_SCREEN_INFO *p)
{
	p->orig_video_mode = screen_info.orig_video_mode;
	p->orig_video_is_vga = screen_info.orig_video_isVGA;
	p->lfb_width = screen_info.lfb_width;
	p->lfb_height = screen_info.lfb_height;
	p->lfb_depth = screen_info.lfb_depth;
	p->lfb_base = screen_info.lfb_base;
	p->lfb_size = screen_info.lfb_size;
	p->lfb_linelength = screen_info.lfb_linelength;
	return OK;
}
#endif

/*************************
 * ESCAPE CALL FUNCTIONS *
 *************************/

static int esc_mods_get_api_version(struct file *pfile,
				    struct MODS_GET_VERSION *p)
{
	p->version = MODS_DRIVER_VERSION;
	return OK;
}

static int esc_mods_get_kernel_version(struct file *pfile,
				       struct MODS_GET_VERSION *p)
{
	p->version = MODS_KERNEL_VERSION;
	return OK;
}

static int esc_mods_set_driver_para(struct file *pfile,
				    struct MODS_SET_PARA *p)
{
	int rc = OK;
	return rc;
}

static int esc_mods_get_screen_info(struct file *pfile,
				    struct MODS_SCREEN_INFO *p)
{
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64) || defined(CONFIG_PPC64)
	return -EINVAL;
#else
	int rc = mods_get_screen_info(p);

#if defined(VIDEO_CAPABILITY_64BIT_BASE)
	if (screen_info.ext_lfb_base)
		return -EOVERFLOW;
#endif

	return rc;
#endif
}

static int esc_mods_get_screen_info_2(struct file *pfile,
				      struct MODS_SCREEN_INFO_2 *p)
{
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64) || defined(CONFIG_PPC64)
	return -EINVAL;
#else
	int rc = mods_get_screen_info(&p->info);

#if defined(VIDEO_CAPABILITY_64BIT_BASE)
	p->ext_lfb_base = screen_info.ext_lfb_base;
#else
	p->ext_lfb_base = 0;
#endif

	return rc;
#endif
}

static int esc_mods_lock_console(struct file *pfile)
{
#if defined(MODS_HAS_CONSOLE_LOCK)
	console_lock();
	return OK;
#else
	return -EINVAL;
#endif
}

static int esc_mods_unlock_console(struct file *pfile)
{
#if defined(MODS_HAS_CONSOLE_LOCK)
	console_unlock();
	return OK;
#else
	return -EINVAL;
#endif
}

static int esc_mods_suspend_console(struct file *pfile)
{
	int ret = -EINVAL;

	LOG_ENT();

#if defined(CONFIG_FB) && defined(MODS_HAS_CONSOLE_LOCK)
	if (num_registered_fb) {
		/* tell the os to block fb accesses */
		struct mods_client *client = pfile->private_data;
		int i = 0;

		for (i = 0; i < num_registered_fb; i++) {
			console_lock();
			if (registered_fb[i]->state != FBINFO_STATE_SUSPENDED) {
				fb_set_suspend(registered_fb[i], 1);
				client->mods_fb_suspended[i] = 1;
			}
			console_unlock();
		}
		ret = OK;
	}
#endif

#if defined(MODS_HAS_CONSOLE_BINDING) && defined(MODS_HAS_CONSOLE_LOCK)
	if (&vga_con == vc_cons[fg_console].d->vc_sw) {
		/* if the current console is the vga console driver,
		 * have the dummy driver take over.
		 */
		console_lock();
		do_take_over_console(&dummy_con, 0, 0, 0);
		console_unlock();
		ret = OK;
	}
#endif

	LOG_EXT();

	return ret;
}

static int esc_mods_resume_console(struct file *pfile)
{
	return mods_resume_console(pfile);
}

static int mods_resume_console(struct file *pfile)
{
	int ret = -EINVAL;

	LOG_ENT();

#if defined(CONFIG_FB) && defined(MODS_HAS_CONSOLE_LOCK)
	if (num_registered_fb) {
		struct mods_client *client = pfile->private_data;
		int i = 0;

		for (i = 0; i < num_registered_fb; i++) {
			console_lock();
			if (client->mods_fb_suspended[i]) {
				fb_set_suspend(registered_fb[i], 0);
				client->mods_fb_suspended[i] = 0;
			}
			console_unlock();
		}
		ret = OK;
	}
#endif

#if defined(MODS_HAS_CONSOLE_BINDING) && defined(MODS_HAS_CONSOLE_LOCK)
	if (&dummy_con == vc_cons[fg_console].d->vc_sw) {
		/* try to unbind the dummy driver,
		 * the system driver should take over.
		 */
		console_lock();
		do_unbind_con_driver(vc_cons[fg_console].d->vc_sw, 0, 0, 0);
		console_unlock();
		ret = OK;
	}
#endif

	LOG_EXT();

	return ret;
}

static int esc_mods_acquire_access_token(struct file *pfile,
					 struct MODS_ACCESS_TOKEN *ptoken)
{
	int ret = -EINVAL;

	LOG_ENT();

	if (mods_get_multi_instance()) {
		LOG_EXT();
		mods_error_printk(
		"access token ops not supported with multi_instance=1!\n");
		return ret;
	}

	get_random_bytes(&ptoken->token, sizeof(ptoken->token));
	ret = mods_set_access_token(ptoken->token);
	if (ret < 0) {
		mods_error_printk("unable to set access token!\n");
	} else {
		struct mods_client *client = pfile->private_data;

		client->access_token = ptoken->token;
	}

	LOG_EXT();

	return ret;
}

static int esc_mods_release_access_token(struct file *pfile,
					 struct MODS_ACCESS_TOKEN *ptoken)
{
	int ret = -EINVAL;

	LOG_ENT();

	if (mods_get_multi_instance()) {
		LOG_EXT();
		mods_error_printk(
		"access token ops not supported with multi_instance=1!\n");
		return ret;
	}

	ret = mods_set_access_token(MODS_ACCESS_TOKEN_NONE);
	if (ret < 0) {
		mods_error_printk("unable to clear access token!\n");
	} else {
		struct mods_client *client = pfile->private_data;

		client->access_token = MODS_ACCESS_TOKEN_NONE;
	}

	LOG_EXT();

	return ret;
}

static int esc_mods_verify_access_token(struct file *pfile,
					struct MODS_ACCESS_TOKEN *ptoken)
{
	int ret = -EINVAL;

	LOG_ENT();

	if (ptoken->token == mods_get_access_token()) {
		struct mods_client *client = pfile->private_data;

		client->access_token = ptoken->token;
		ret = OK;
	} else
		mods_error_printk("invalid access token\n");

	LOG_EXT();

	return ret;
}

struct mods_sysfs_work {
	struct work_struct      work;
	struct MODS_SYSFS_NODE *pdata;
	int                     ret;
};

#ifdef MODS_OLD_INIT_WORK
static void sysfs_write_task(void *w)
#else
static void sysfs_write_task(struct work_struct *w)
#endif
{
	struct mods_sysfs_work *task = container_of(w,
						    struct mods_sysfs_work,
						    work);
	struct file *f;
	mm_segment_t old_fs;

	LOG_ENT();

	task->ret = -EINVAL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	f = filp_open(task->pdata->path, O_WRONLY, 0);
	if (IS_ERR(f))
		task->ret = PTR_ERR(f);
	else {
		f->f_pos = 0;
		if (task->pdata->size <= MODS_MAX_SYSFS_FILE_SIZE)
			task->ret = f->f_op->write(f,
						   task->pdata->contents,
						   task->pdata->size,
						   &f->f_pos);
		filp_close(f, NULL);
	}

	set_fs(old_fs);

	LOG_EXT();
}

static int esc_mods_write_sysfs_node(struct file            *pfile,
				     struct MODS_SYSFS_NODE *pdata)
{
	int ret = -EINVAL;
	struct mods_sysfs_work task;
	struct workqueue_struct *wq;

	LOG_ENT();

	memmove(&pdata->path[5], pdata->path, MODS_MAX_SYSFS_PATH_LEN);
	memcpy(pdata->path, "/sys/", 5);
	pdata->path[MODS_MAX_SYSFS_PATH_BUF_SIZE - 1] = 0;

	task.pdata = pdata;

	wq = create_singlethread_workqueue("mods_sysfs_write");
	if (!wq) {
		LOG_EXT();
		return ret;
	}

#ifdef MODS_OLD_INIT_WORK
	INIT_WORK(&task.work, sysfs_write_task, &task);
#else
	INIT_WORK(&task.work, sysfs_write_task);
#endif
	queue_work(wq, &task.work);
	flush_workqueue(wq);
	destroy_workqueue(wq);

	ret = task.ret;
	if (ret > 0)
		ret = OK;

	LOG_EXT();
	return ret;
}

/**************
 * IO control *
 **************/

static long mods_krnl_ioctl(struct file  *fp,
			    unsigned int  cmd,
			    unsigned long i_arg)
{
	int ret = 0;
	void *arg_copy = 0;
	void *arg = (void *) i_arg;
	int arg_size;
	char buf[64];

	LOG_ENT();

	if ((cmd != MODS_ESC_VERIFY_ACCESS_TOKEN) &&
	    (cmd != MODS_ESC_GET_API_VERSION)) {
		ret = mods_check_access_token(fp);
		if (ret < 0) {
			LOG_EXT();
			return ret;
		}
	}

	arg_size = _IOC_SIZE(cmd);

	if (arg_size > (int)sizeof(buf)) {
		arg_copy = kmalloc(arg_size, GFP_KERNEL | __GFP_NORETRY);
		if (unlikely(!arg_copy)) {
			LOG_EXT();
			return -ENOMEM;
		}
	} else if (arg_size > 0)
		arg_copy = buf;

	if ((arg_size > 0) && copy_from_user(arg_copy, arg, arg_size)) {
		mods_error_printk("failed to copy ioctl data\n");
		if (arg_size > (int)sizeof(buf))
			kfree(arg_copy);
		LOG_EXT();
		return -EFAULT;
	}

#define MODS_IOCTL(code, function, argtype)\
	({\
	do {\
		mods_debug_printk(DEBUG_IOCTL, "ioctl(" #code ")\n");\
		if (arg_size != sizeof(struct argtype)) {\
			ret = -EINVAL;\
			mods_error_printk( \
				"invalid parameter passed to ioctl " #code \
				"\n");\
		} else {\
			ret = function(fp, (struct argtype *)arg_copy);\
			if ((ret == OK) && \
			    copy_to_user(arg, arg_copy, arg_size)) {\
				ret = -EFAULT;\
				mods_error_printk( \
					"copying return value for ioctl " \
					#code " to user space failed\n");\
			} \
		} \
	} while (0);\
	})

#define MODS_IOCTL_NORETVAL(code, function, argtype)\
	({\
	do {\
		mods_debug_printk(DEBUG_IOCTL, "ioctl(" #code ")\n");\
		if (arg_size != sizeof(struct argtype)) {\
			ret = -EINVAL;\
			mods_error_printk( \
				"invalid parameter passed to ioctl " #code \
				"\n");\
		} else {\
			ret = function(fp, (struct argtype *)arg_copy);\
		} \
	} while (0);\
	})

#define MODS_IOCTL_VOID(code, function)\
	({\
	do {\
		mods_debug_printk(DEBUG_IOCTL, "ioctl(" #code ")\n");\
		if (arg_size != 0) {\
			ret = -EINVAL;\
			mods_error_printk( \
				"invalid parameter passed to ioctl " #code \
				"\n");\
		} else {\
			ret = function(fp);\
		} \
	} while (0);\
	})

	switch (cmd) {

#ifdef CONFIG_PCI
	case MODS_ESC_FIND_PCI_DEVICE:
		MODS_IOCTL(MODS_ESC_FIND_PCI_DEVICE,
			   esc_mods_find_pci_dev, MODS_FIND_PCI_DEVICE);
		break;

	case MODS_ESC_FIND_PCI_DEVICE_2:
		MODS_IOCTL(MODS_ESC_FIND_PCI_DEVICE_2,
			   esc_mods_find_pci_dev_2,
			   MODS_FIND_PCI_DEVICE_2);
		break;

	case MODS_ESC_FIND_PCI_CLASS_CODE:
		MODS_IOCTL(MODS_ESC_FIND_PCI_CLASS_CODE,
			   esc_mods_find_pci_class_code,
			   MODS_FIND_PCI_CLASS_CODE);
		break;

	case MODS_ESC_FIND_PCI_CLASS_CODE_2:
		MODS_IOCTL(MODS_ESC_FIND_PCI_CLASS_CODE_2,
			   esc_mods_find_pci_class_code_2,
			   MODS_FIND_PCI_CLASS_CODE_2);
		break;

	case MODS_ESC_PCI_GET_BAR_INFO:
		MODS_IOCTL(MODS_ESC_PCI_GET_BAR_INFO,
			   esc_mods_pci_get_bar_info,
			   MODS_PCI_GET_BAR_INFO);
		break;

	case MODS_ESC_PCI_GET_BAR_INFO_2:
		MODS_IOCTL(MODS_ESC_PCI_GET_BAR_INFO_2,
			   esc_mods_pci_get_bar_info_2,
			   MODS_PCI_GET_BAR_INFO_2);
		break;

	case MODS_ESC_PCI_GET_IRQ:
		MODS_IOCTL(MODS_ESC_PCI_GET_IRQ,
			   esc_mods_pci_get_irq,
			   MODS_PCI_GET_IRQ);
		break;

	case MODS_ESC_PCI_GET_IRQ_2:
		MODS_IOCTL(MODS_ESC_PCI_GET_IRQ_2,
			   esc_mods_pci_get_irq_2,
			   MODS_PCI_GET_IRQ_2);
		break;

	case MODS_ESC_PCI_READ:
		MODS_IOCTL(MODS_ESC_PCI_READ, esc_mods_pci_read, MODS_PCI_READ);
		break;

	case MODS_ESC_PCI_READ_2:
		MODS_IOCTL(MODS_ESC_PCI_READ_2,
			   esc_mods_pci_read_2, MODS_PCI_READ_2);
		break;

	case MODS_ESC_PCI_WRITE:
		MODS_IOCTL_NORETVAL(MODS_ESC_PCI_WRITE,
				    esc_mods_pci_write, MODS_PCI_WRITE);
		break;

	case MODS_ESC_PCI_WRITE_2:
		MODS_IOCTL_NORETVAL(MODS_ESC_PCI_WRITE_2,
				    esc_mods_pci_write_2,
				    MODS_PCI_WRITE_2);
		break;

	case MODS_ESC_PCI_BUS_ADD_DEVICES:
		MODS_IOCTL_NORETVAL(MODS_ESC_PCI_BUS_ADD_DEVICES,
				    esc_mods_pci_bus_add_dev,
				    MODS_PCI_BUS_ADD_DEVICES);
		break;

	case MODS_ESC_PCI_HOT_RESET:
		MODS_IOCTL_NORETVAL(MODS_ESC_PCI_HOT_RESET,
				    esc_mods_pci_hot_reset,
				    MODS_PCI_HOT_RESET);
		break;

	case MODS_ESC_PIO_READ:
		MODS_IOCTL(MODS_ESC_PIO_READ,
			   esc_mods_pio_read, MODS_PIO_READ);
		break;

	case MODS_ESC_PIO_WRITE:
		MODS_IOCTL_NORETVAL(MODS_ESC_PIO_WRITE,
				    esc_mods_pio_write, MODS_PIO_WRITE);
		break;

	case MODS_ESC_DEVICE_NUMA_INFO:
		MODS_IOCTL(MODS_ESC_DEVICE_NUMA_INFO,
			   esc_mods_device_numa_info,
			   MODS_DEVICE_NUMA_INFO);
		break;

	case MODS_ESC_DEVICE_NUMA_INFO_2:
		MODS_IOCTL(MODS_ESC_DEVICE_NUMA_INFO_2,
			   esc_mods_device_numa_info_2,
			   MODS_DEVICE_NUMA_INFO_2);
		break;

	case MODS_ESC_GET_IOMMU_STATE:
		MODS_IOCTL(MODS_ESC_GET_IOMMU_STATE,
			   esc_mods_get_iommu_state,
			   MODS_GET_IOMMU_STATE);
		break;

	case MODS_ESC_GET_IOMMU_STATE_2:
		MODS_IOCTL(MODS_ESC_GET_IOMMU_STATE_2,
			   esc_mods_get_iommu_state_2,
			   MODS_GET_IOMMU_STATE);
		break;

	case MODS_ESC_PCI_SET_DMA_MASK:
		MODS_IOCTL(MODS_ESC_PCI_SET_DMA_MASK,
			   esc_mods_pci_set_dma_mask,
			   MODS_PCI_DMA_MASK);
		break;
#endif

	case MODS_ESC_ALLOC_PAGES:
		MODS_IOCTL(MODS_ESC_ALLOC_PAGES,
			   esc_mods_alloc_pages, MODS_ALLOC_PAGES);
		break;

	case MODS_ESC_DEVICE_ALLOC_PAGES:
		MODS_IOCTL(MODS_ESC_DEVICE_ALLOC_PAGES,
			   esc_mods_device_alloc_pages,
			   MODS_DEVICE_ALLOC_PAGES);
		break;

	case MODS_ESC_DEVICE_ALLOC_PAGES_2:
		MODS_IOCTL(MODS_ESC_DEVICE_ALLOC_PAGES_2,
			   esc_mods_device_alloc_pages_2,
			   MODS_DEVICE_ALLOC_PAGES_2);
		break;

	case MODS_ESC_FREE_PAGES:
		MODS_IOCTL(MODS_ESC_FREE_PAGES,
			   esc_mods_free_pages, MODS_FREE_PAGES);
		break;

	case MODS_ESC_GET_PHYSICAL_ADDRESS:
		MODS_IOCTL(MODS_ESC_GET_PHYSICAL_ADDRESS,
			   esc_mods_get_phys_addr,
			   MODS_GET_PHYSICAL_ADDRESS);
		break;

	case MODS_ESC_GET_PHYSICAL_ADDRESS_2:
		MODS_IOCTL(MODS_ESC_GET_PHYSICAL_ADDRESS_2,
			   esc_mods_get_phys_addr_2,
			   MODS_GET_PHYSICAL_ADDRESS_3);
		break;

	case MODS_ESC_GET_MAPPED_PHYSICAL_ADDRESS:
		MODS_IOCTL(MODS_ESC_GET_MAPPED_PHYSICAL_ADDRESS,
			   esc_mods_get_mapped_phys_addr,
			   MODS_GET_PHYSICAL_ADDRESS);
		break;

	case MODS_ESC_GET_MAPPED_PHYSICAL_ADDRESS_2:
		MODS_IOCTL(MODS_ESC_GET_MAPPED_PHYSICAL_ADDRESS_2,
			   esc_mods_get_mapped_phys_addr_2,
			   MODS_GET_PHYSICAL_ADDRESS_2);
		break;

	case MODS_ESC_GET_MAPPED_PHYSICAL_ADDRESS_3:
		MODS_IOCTL(MODS_ESC_GET_MAPPED_PHYSICAL_ADDRESS_3,
			   esc_mods_get_mapped_phys_addr_3,
			   MODS_GET_PHYSICAL_ADDRESS_3);
		break;

	case MODS_ESC_SET_MEMORY_TYPE:
		MODS_IOCTL_NORETVAL(MODS_ESC_SET_MEMORY_TYPE,
				    esc_mods_set_mem_type,
				    MODS_MEMORY_TYPE);
		break;

	case MODS_ESC_VIRTUAL_TO_PHYSICAL:
		MODS_IOCTL(MODS_ESC_VIRTUAL_TO_PHYSICAL,
			   esc_mods_virtual_to_phys,
			   MODS_VIRTUAL_TO_PHYSICAL);
		break;

	case MODS_ESC_PHYSICAL_TO_VIRTUAL:
		MODS_IOCTL(MODS_ESC_PHYSICAL_TO_VIRTUAL,
			   esc_mods_phys_to_virtual, MODS_PHYSICAL_TO_VIRTUAL);
		break;

#if defined(CONFIG_PPC64)
	case MODS_ESC_SET_PPC_TCE_BYPASS:
		MODS_IOCTL(MODS_ESC_SET_PPC_TCE_BYPASS,
			   esc_mods_set_ppc_tce_bypass,
			   MODS_SET_PPC_TCE_BYPASS);
		break;

	case MODS_ESC_GET_ATS_ADDRESS_RANGE:
		MODS_IOCTL(MODS_ESC_GET_ATS_ADDRESS_RANGE,
			   esc_mods_get_ats_address_range,
			   MODS_GET_ATS_ADDRESS_RANGE);
		break;
	case MODS_ESC_SET_NVLINK_SYSMEM_TRAINED:
		MODS_IOCTL(MODS_ESC_SET_NVLINK_SYSMEM_TRAINED,
			   esc_mods_set_nvlink_sysmem_trained,
			   MODS_SET_NVLINK_SYSMEM_TRAINED);
		break;
	case MODS_ESC_GET_NVLINK_LINE_RATE:
		MODS_IOCTL(MODS_ESC_GET_NVLINK_LINE_RATE,
			   esc_mods_get_nvlink_line_rate,
			   MODS_GET_NVLINK_LINE_RATE);
		break;
#endif

	case MODS_ESC_DMA_MAP_MEMORY:
		MODS_IOCTL(MODS_ESC_DMA_MAP_MEMORY,
			   esc_mods_dma_map_memory,
			   MODS_DMA_MAP_MEMORY);
		break;

	case MODS_ESC_DMA_UNMAP_MEMORY:
		MODS_IOCTL(MODS_ESC_DMA_UNMAP_MEMORY,
			   esc_mods_dma_unmap_memory,
			   MODS_DMA_MAP_MEMORY);
		break;

	case MODS_ESC_IRQ_REGISTER:
	case MODS_ESC_MSI_REGISTER:
		ret = -EINVAL;
		break;

#if defined(MODS_TEGRA) && defined(CONFIG_OF) && defined(CONFIG_OF_IRQ)
	case MODS_ESC_MAP_INTERRUPT:
		MODS_IOCTL(MODS_ESC_MAP_INTERRUPT,
				esc_mods_map_irq, MODS_DT_INFO);
		break;

	case MODS_ESC_MAP_GPIO:
		MODS_IOCTL(MODS_ESC_MAP_GPIO,
		esc_mods_map_irq_to_gpio, MODS_GPIO_INFO);
		break;
#endif

	case MODS_ESC_REGISTER_IRQ:
		MODS_IOCTL_NORETVAL(MODS_ESC_REGISTER_IRQ,
				esc_mods_register_irq, MODS_REGISTER_IRQ);
		break;

	case MODS_ESC_REGISTER_IRQ_2:
		MODS_IOCTL_NORETVAL(MODS_ESC_REGISTER_IRQ_2,
				esc_mods_register_irq_2, MODS_REGISTER_IRQ_2);
		break;

	case MODS_ESC_REGISTER_IRQ_3:
		MODS_IOCTL_NORETVAL(MODS_ESC_REGISTER_IRQ_3,
				esc_mods_register_irq_3, MODS_REGISTER_IRQ_3);
		break;

	case MODS_ESC_UNREGISTER_IRQ:
		MODS_IOCTL_NORETVAL(MODS_ESC_UNREGISTER_IRQ,
				    esc_mods_unregister_irq, MODS_REGISTER_IRQ);
		break;

	case MODS_ESC_UNREGISTER_IRQ_2:
		MODS_IOCTL_NORETVAL(MODS_ESC_UNREGISTER_IRQ_2,
				    esc_mods_unregister_irq_2,
				    MODS_REGISTER_IRQ_2);
		break;

	case MODS_ESC_QUERY_IRQ:
		MODS_IOCTL(MODS_ESC_QUERY_IRQ,
			   esc_mods_query_irq, MODS_QUERY_IRQ);
		break;

	case MODS_ESC_QUERY_IRQ_2:
		MODS_IOCTL(MODS_ESC_QUERY_IRQ_2,
			   esc_mods_query_irq_2, MODS_QUERY_IRQ_2);
		break;

	case MODS_ESC_IRQ_HANDLED:
		MODS_IOCTL_NORETVAL(MODS_ESC_IRQ_HANDLED,
				    esc_mods_irq_handled, MODS_REGISTER_IRQ);
		break;

	case MODS_ESC_IRQ_HANDLED_2:
		MODS_IOCTL_NORETVAL(MODS_ESC_IRQ_HANDLED_2,
				    esc_mods_irq_handled_2,
				    MODS_REGISTER_IRQ_2);
		break;

#ifdef CONFIG_ACPI
	case MODS_ESC_EVAL_ACPI_METHOD:
		MODS_IOCTL(MODS_ESC_EVAL_ACPI_METHOD,
			   esc_mods_eval_acpi_method, MODS_EVAL_ACPI_METHOD);
		break;

	case MODS_ESC_EVAL_DEV_ACPI_METHOD:
		MODS_IOCTL(MODS_ESC_EVAL_DEV_ACPI_METHOD,
			   esc_mods_eval_dev_acpi_method,
			   MODS_EVAL_DEV_ACPI_METHOD);
		break;

	case MODS_ESC_EVAL_DEV_ACPI_METHOD_2:
		MODS_IOCTL(MODS_ESC_EVAL_DEV_ACPI_METHOD_2,
			   esc_mods_eval_dev_acpi_method_2,
			   MODS_EVAL_DEV_ACPI_METHOD_2);
		break;

	case MODS_ESC_ACPI_GET_DDC:
		MODS_IOCTL(MODS_ESC_ACPI_GET_DDC,
			   esc_mods_acpi_get_ddc, MODS_ACPI_GET_DDC);
		break;

	case MODS_ESC_ACPI_GET_DDC_2:
		MODS_IOCTL(MODS_ESC_ACPI_GET_DDC_2,
			   esc_mods_acpi_get_ddc_2, MODS_ACPI_GET_DDC_2);
		break;

#else
	case MODS_ESC_EVAL_ACPI_METHOD:
		/* fallthrough */
	case MODS_ESC_EVAL_DEV_ACPI_METHOD:
		/* fallthrough */
	case MODS_ESC_EVAL_DEV_ACPI_METHOD_2:
		/* fallthrough */
	case MODS_ESC_ACPI_GET_DDC:
		/* fallthrough */
	case MODS_ESC_ACPI_GET_DDC_2:
		/* Silent failure to avoid clogging kernel log */
		ret = -EINVAL;
		break;
#endif
	case MODS_ESC_GET_API_VERSION:
		MODS_IOCTL(MODS_ESC_GET_API_VERSION,
			   esc_mods_get_api_version, MODS_GET_VERSION);
		break;

	case MODS_ESC_GET_KERNEL_VERSION:
		MODS_IOCTL(MODS_ESC_GET_KERNEL_VERSION,
			   esc_mods_get_kernel_version, MODS_GET_VERSION);
		break;

	case MODS_ESC_SET_DRIVER_PARA:
		MODS_IOCTL_NORETVAL(MODS_ESC_SET_DRIVER_PARA,
				    esc_mods_set_driver_para, MODS_SET_PARA);
		break;

#if defined(MODS_HAS_CLOCK)
	case MODS_ESC_GET_CLOCK_HANDLE:
		MODS_IOCTL(MODS_ESC_GET_CLOCK_HANDLE,
			   esc_mods_get_clock_handle, MODS_GET_CLOCK_HANDLE);
		break;

	case MODS_ESC_SET_CLOCK_RATE:
		MODS_IOCTL_NORETVAL(MODS_ESC_SET_CLOCK_RATE,
				    esc_mods_set_clock_rate, MODS_CLOCK_RATE);
		break;

	case MODS_ESC_GET_CLOCK_RATE:
		MODS_IOCTL(MODS_ESC_GET_CLOCK_RATE,
			   esc_mods_get_clock_rate, MODS_CLOCK_RATE);
		break;

	case MODS_ESC_GET_CLOCK_MAX_RATE:
		MODS_IOCTL(MODS_ESC_GET_CLOCK_MAX_RATE,
			   esc_mods_get_clock_max_rate, MODS_CLOCK_RATE);
		break;

	case MODS_ESC_SET_CLOCK_MAX_RATE:
		MODS_IOCTL_NORETVAL(MODS_ESC_SET_CLOCK_MAX_RATE,
				    esc_mods_set_clock_max_rate,
				    MODS_CLOCK_RATE);
		break;

	case MODS_ESC_SET_CLOCK_PARENT:
		MODS_IOCTL_NORETVAL(MODS_ESC_SET_CLOCK_PARENT,
				    esc_mods_set_clock_parent,
				    MODS_CLOCK_PARENT);
		break;

	case MODS_ESC_GET_CLOCK_PARENT:
		MODS_IOCTL(MODS_ESC_GET_CLOCK_PARENT,
			   esc_mods_get_clock_parent, MODS_CLOCK_PARENT);
		break;

	case MODS_ESC_ENABLE_CLOCK:
		MODS_IOCTL_NORETVAL(MODS_ESC_ENABLE_CLOCK,
				    esc_mods_enable_clock, MODS_CLOCK_HANDLE);
		break;

	case MODS_ESC_DISABLE_CLOCK:
		MODS_IOCTL_NORETVAL(MODS_ESC_DISABLE_CLOCK,
				    esc_mods_disable_clock, MODS_CLOCK_HANDLE);
		break;

	case MODS_ESC_IS_CLOCK_ENABLED:
		MODS_IOCTL(MODS_ESC_IS_CLOCK_ENABLED,
			   esc_mods_is_clock_enabled, MODS_CLOCK_ENABLED);
		break;

	case MODS_ESC_CLOCK_RESET_ASSERT:
		MODS_IOCTL_NORETVAL(MODS_ESC_CLOCK_RESET_ASSERT,
				    esc_mods_clock_reset_assert,
				    MODS_CLOCK_HANDLE);
		break;

	case MODS_ESC_CLOCK_RESET_DEASSERT:
		MODS_IOCTL_NORETVAL(MODS_ESC_CLOCK_RESET_DEASSERT,
				    esc_mods_clock_reset_deassert,
				    MODS_CLOCK_HANDLE);
		break;
#endif
#if defined(MODS_TEGRA)
	case MODS_ESC_FLUSH_CPU_CACHE_RANGE:
		MODS_IOCTL_NORETVAL(MODS_ESC_FLUSH_CPU_CACHE_RANGE,
				    esc_mods_flush_cpu_cache_range,
				    MODS_FLUSH_CPU_CACHE_RANGE);
		break;
	case MODS_ESC_DMA_ALLOC_COHERENT:
		MODS_IOCTL(MODS_ESC_DMA_ALLOC_COHERENT,
			   esc_mods_dma_alloc_coherent,
			   MODS_DMA_COHERENT_MEM_HANDLE);
		break;
	case MODS_ESC_DMA_FREE_COHERENT:
		MODS_IOCTL(MODS_ESC_DMA_FREE_COHERENT,
			   esc_mods_dma_free_coherent,
			   MODS_DMA_COHERENT_MEM_HANDLE);
		break;
	case MODS_ESC_DMA_COPY_TO_USER:
		MODS_IOCTL(MODS_ESC_DMA_COPY_TO_USER,
			   esc_mods_dma_copy_to_user,
			   MODS_DMA_COPY_TO_USER);
		break;
#if defined(CONFIG_DMA_ENGINE)
	case MODS_ESC_DMA_REQUEST_HANDLE:
		MODS_IOCTL(MODS_ESC_DMA_REQUEST_HANDLE,
			   esc_mods_dma_request_channel,
			   MODS_DMA_HANDLE);
		break;
	case MODS_ESC_DMA_RELEASE_HANDLE:
		MODS_IOCTL_NORETVAL(MODS_ESC_DMA_RELEASE_HANDLE,
			   esc_mods_dma_release_channel,
			   MODS_DMA_HANDLE);
		break;
	case MODS_ESC_DMA_ISSUE_PENDING:
		MODS_IOCTL_NORETVAL(MODS_ESC_DMA_ISSUE_PENDING,
				    esc_mods_dma_async_issue_pending,
				    MODS_DMA_HANDLE);
		break;
	case MODS_ESC_DMA_SET_CONFIG:
		MODS_IOCTL_NORETVAL(MODS_ESC_DMA_SET_CONFIG,
				    esc_mods_dma_set_config,
				    MODS_DMA_CHANNEL_CONFIG);
		break;
	case MODS_ESC_DMA_TX_SUBMIT:
		MODS_IOCTL(MODS_ESC_DMA_TX_SUBMIT,
			   esc_mods_dma_submit_request,
			   MODS_DMA_TX_DESC);
		break;
	case MODS_ESC_DMA_TX_WAIT:
		MODS_IOCTL(MODS_MODS_ESC_DMA_TX_WAIT,
			   esc_mods_dma_wait,
			   MODS_DMA_WAIT_DESC);
		break;
#endif
#ifdef CONFIG_TEGRA_DC
	case MODS_ESC_TEGRA_DC_CONFIG_POSSIBLE:
		MODS_IOCTL(MODS_ESC_TEGRA_DC_CONFIG_POSSIBLE,
				   esc_mods_tegra_dc_config_possible,
				   MODS_TEGRA_DC_CONFIG_POSSIBLE);
		break;
#endif
#ifdef MODS_HAS_NET
	case MODS_ESC_NET_FORCE_LINK:
		MODS_IOCTL(MODS_ESC_NET_FORCE_LINK,
			   esc_mods_net_force_link, MODS_NET_DEVICE_NAME);
		break;
#endif
#endif
	case MODS_ESC_MEMORY_BARRIER:
		MODS_IOCTL_VOID(MODS_ESC_MEMORY_BARRIER,
				esc_mods_memory_barrier);
		break;

#ifdef MODS_TEGRA
	case MODS_ESC_DMABUF_GET_PHYSICAL_ADDRESS:
		MODS_IOCTL(MODS_ESC_DMABUF_GET_PHYSICAL_ADDRESS,
			   esc_mods_dmabuf_get_phys_addr,
			   MODS_DMABUF_GET_PHYSICAL_ADDRESS);
		break;
#endif
#ifdef CONFIG_TEGRA_NVADSP
	case MODS_ESC_ADSP_LOAD:
		MODS_IOCTL_VOID(MODS_ESC_ADSP_LOAD,
				esc_mods_adsp_load);
		break;

	case MODS_ESC_ADSP_START:
		MODS_IOCTL_VOID(MODS_ESC_ADSP_START,
				esc_mods_adsp_start);
		break;

	case MODS_ESC_ADSP_STOP:
		MODS_IOCTL_VOID(MODS_ESC_ADSP_STOP,
				esc_mods_adsp_stop);
		break;

	case MODS_ESC_ADSP_RUN_APP:
		MODS_IOCTL_NORETVAL(MODS_ESC_ADSP_RUN_APP,
				    esc_mods_adsp_run_app,
				    MODS_ADSP_RUN_APP_INFO);
		break;
#endif
	case MODS_ESC_GET_SCREEN_INFO:
		MODS_IOCTL(MODS_ESC_GET_SCREEN_INFO,
			   esc_mods_get_screen_info, MODS_SCREEN_INFO);
		break;
	case MODS_ESC_GET_SCREEN_INFO_2:
		MODS_IOCTL(MODS_ESC_GET_SCREEN_INFO_2,
			   esc_mods_get_screen_info_2, MODS_SCREEN_INFO_2);
		break;
	case MODS_ESC_LOCK_CONSOLE:
		MODS_IOCTL_VOID(MODS_ESC_LOCK_CONSOLE,
			   esc_mods_lock_console);
		break;
	case MODS_ESC_UNLOCK_CONSOLE:
		MODS_IOCTL_VOID(MODS_ESC_UNLOCK_CONSOLE,
			   esc_mods_unlock_console);
		break;
	case MODS_ESC_SUSPEND_CONSOLE:
		MODS_IOCTL_VOID(MODS_ESC_SUSPEND_CONSOLE,
			   esc_mods_suspend_console);
		break;
	case MODS_ESC_RESUME_CONSOLE:
		MODS_IOCTL_VOID(MODS_ESC_RESUME_CONSOLE,
			   esc_mods_resume_console);
		break;

#if defined(MODS_TEGRA)
	case MODS_ESC_TEGRA_PROD_IS_SUPPORTED:
		MODS_IOCTL(MODS_ESC_TEGRA_PROD_IS_SUPPORTED,
			   esc_mods_tegra_prod_is_supported,
			   MODS_TEGRA_PROD_IS_SUPPORTED);
		break;

	case MODS_ESC_TEGRA_PROD_SET_PROD_ALL:
		MODS_IOCTL_NORETVAL(MODS_ESC_TEGRA_PROD_SET_PROD_ALL,
				    esc_mods_tegra_prod_set_prod_all,
				    MODS_TEGRA_PROD_SET_TUPLE);
		break;

	case MODS_ESC_TEGRA_PROD_SET_PROD_BOOT:
		MODS_IOCTL_NORETVAL(MODS_ESC_TEGRA_PROD_SET_PROD_BOOT,
				    esc_mods_tegra_prod_set_prod_boot,
				    MODS_TEGRA_PROD_SET_TUPLE);
		break;

	case MODS_ESC_TEGRA_PROD_SET_PROD_BY_NAME:
		MODS_IOCTL_NORETVAL(MODS_ESC_TEGRA_PROD_SET_PROD_BY_NAME,
				    esc_mods_tegra_prod_set_prod_by_name,
				    MODS_TEGRA_PROD_SET_TUPLE);
		break;

	case MODS_ESC_TEGRA_PROD_SET_PROD_EXACT:
		MODS_IOCTL_NORETVAL(MODS_ESC_TEGRA_PROD_SET_PROD_EXACT,
				    esc_mods_tegra_prod_set_prod_exact,
				    MODS_TEGRA_PROD_SET_TUPLE);
		break;

	case MODS_ESC_TEGRA_PROD_ITERATE_DT:
		MODS_IOCTL(MODS_ESC_TEGRA_PROD_ITERATE_DT,
			   esc_mods_tegra_prod_iterate_dt,
			   MODS_TEGRA_PROD_ITERATOR);
		break;
#endif

	case MODS_ESC_ACQUIRE_ACCESS_TOKEN:
		MODS_IOCTL(MODS_ESC_ACQUIRE_ACCESS_TOKEN,
			   esc_mods_acquire_access_token,
			   MODS_ACCESS_TOKEN);
		break;

	case MODS_ESC_RELEASE_ACCESS_TOKEN:
		MODS_IOCTL_NORETVAL(MODS_ESC_RELEASE_ACCESS_TOKEN,
				    esc_mods_release_access_token,
				    MODS_ACCESS_TOKEN);
		break;

	case MODS_ESC_VERIFY_ACCESS_TOKEN:
		MODS_IOCTL_NORETVAL(MODS_ESC_VERIFY_ACCESS_TOKEN,
				    esc_mods_verify_access_token,
				    MODS_ACCESS_TOKEN);
		break;

	case MODS_ESC_WRITE_SYSFS_NODE:
		MODS_IOCTL_NORETVAL(MODS_ESC_WRITE_SYSFS_NODE,
				    esc_mods_write_sysfs_node,
				    MODS_SYSFS_NODE);
		break;

	case MODS_ESC_REGISTER_IRQ_4:
		MODS_IOCTL_NORETVAL(MODS_ESC_REGISTER_IRQ_4,
				esc_mods_register_irq_4, MODS_REGISTER_IRQ_4);
		break;

	case MODS_ESC_QUERY_IRQ_3:
		MODS_IOCTL(MODS_ESC_QUERY_IRQ_3,
			   esc_mods_query_irq_3, MODS_QUERY_IRQ_3);
		break;

#ifdef MODS_HAS_SRIOV
	case MODS_ESC_SET_NUM_VF:
		MODS_IOCTL_NORETVAL(MODS_ESC_SET_NUM_VF,
			   esc_mods_set_num_vf, MODS_SET_NUM_VF);
		break;

	case MODS_ESC_SET_TOTAL_VF:
		MODS_IOCTL_NORETVAL(MODS_ESC_SET_TOTAL_VF,
			   esc_mods_set_total_vf, MODS_SET_NUM_VF);
		break;
#endif

	default:
		mods_error_printk("unrecognized ioctl (0x%x) dir(0x%x) type (0x%x) nr (0x%x) size (0x%x)\n",
			cmd, _IOC_DIR(cmd), _IOC_TYPE(cmd),
			_IOC_NR(cmd), _IOC_SIZE(cmd));
		ret = -EINVAL;
		break;
	}

	if (arg_size > (int)sizeof(buf))
		kfree(arg_copy);

	LOG_EXT();
	return ret;
}
