/*
 * mods_irq.c - This file is part of NVIDIA MODS kernel driver.
 *
 * Copyright (c) 2008-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/pci_regs.h>
#if defined(MODS_TEGRA) && defined(CONFIG_OF) && defined(CONFIG_OF_IRQ)
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#endif

#define PCI_VENDOR_ID_NVIDIA 0x10de
#define INDEX_IRQSTAT(irq)	(irq / BITS_NUM)
#define POS_IRQSTAT(irq)	 (irq & (BITS_NUM - 1))

/* MSI */
#define PCI_MSI_MASK_BIT	16
#define MSI_CONTROL_REG(base)		(base + PCI_MSI_FLAGS)
#define IS_64BIT_ADDRESS(control)	(!!(control & PCI_MSI_FLAGS_64BIT))
#define MSI_DATA_REG(base, is64bit) \
	((is64bit == 1) ? base + PCI_MSI_DATA_64 : base + PCI_MSI_DATA_32)
#define TOP_TKE_TKEIE_WDT_MASK(i)	(1 << (16 + 4 * (i)))
#define TOP_TKE_TKEIE(i)		(0x100 + 4 * (i))

/*********************
 * PRIVATE FUNCTIONS *
 *********************/
static struct mods_priv mp;

struct mutex *mods_get_irq_mutex(void)
{
	return &mp.mtx;
}

#ifdef CONFIG_PCI
struct en_dev_entry *mods_enable_device(struct mods_client *client,
					struct pci_dev     *dev)
{
	int                  ret   = -1;
	struct en_dev_entry *dpriv = client->enabled_devices;

	BUG_ON(!mutex_is_locked(&mp.mtx));

	dpriv = pci_get_drvdata(dev);

	if (dpriv) {
		if (dpriv->client_id == client->client_id)
			return dpriv;

		mods_error_printk("invalid client %u for device %04x:%x:%02x.%x\n",
				  (unsigned int)client->client_id,
				  pci_domain_nr(dev->bus),
				  dev->bus->number,
				  PCI_SLOT(dev->devfn),
				  PCI_FUNC(dev->devfn));
		return 0;
	}

	ret = pci_enable_device(dev);

	if (ret != 0) {
		mods_error_printk("failed to enable device %04x:%x:%02x.%x\n",
				  pci_domain_nr(dev->bus),
				  dev->bus->number,
				  PCI_SLOT(dev->devfn),
				  PCI_FUNC(dev->devfn));
		return 0;
	}

	dpriv = kzalloc(sizeof(*dpriv), GFP_KERNEL | __GFP_NORETRY);
	if (unlikely(!dpriv))
		return 0;
	dpriv->client_id = client->client_id;
	dpriv->dev = dev;
	dpriv->next = client->enabled_devices;
	client->enabled_devices = dpriv;
	pci_set_drvdata(dev, dpriv);

	return dpriv;
}

void mods_disable_device(struct pci_dev *dev)
{
	struct en_dev_entry *dpriv = pci_get_drvdata(dev);

	BUG_ON(!mutex_is_locked(&mp.mtx));

	if (dpriv)
		pci_set_drvdata(dev, NULL);

	pci_disable_device(dev);
}
#endif

static unsigned int get_cur_time(void)
{
	/* This is not very precise, sched_clock() would be better */
	return jiffies_to_usecs(jiffies);
}

static inline int mods_check_interrupt(struct dev_irq_map *t)
{
	int ii = 0;
	int valid = 0;

	/* For MSI - we always treat it as pending (must rearm later). */
	/* For non-GPU devices - we can't tell. */
	if (t->mask_info_cnt == 0)
		return true;

	for (ii = 0; ii < t->mask_info_cnt; ii++) {
		if (!t->mask_info[ii].dev_irq_state ||
		    !t->mask_info[ii].dev_irq_mask_reg)
			continue;

		/* GPU device */
		if (t->mask_info[ii].mask_type == MODS_MASK_TYPE_IRQ_DISABLE64)
			valid |= ((*(u64 *)t->mask_info[ii].dev_irq_state &&
			*(u64 *)t->mask_info[ii].dev_irq_mask_reg) != 0);
		else
			valid |= ((*t->mask_info[ii].dev_irq_state &&
			     *t->mask_info[ii].dev_irq_mask_reg) != 0);
	}
	return valid;
}

static void mods_disable_interrupts(struct dev_irq_map *t)
{
	u32 ii = 0;

	for (ii = 0; ii < t->mask_info_cnt; ii++) {
		if (t->mask_info[ii].dev_irq_disable_reg &&
		   t->mask_info[ii].mask_type == MODS_MASK_TYPE_IRQ_DISABLE64) {
			if (t->mask_info[ii].irq_and_mask == 0)
				*(u64 *)t->mask_info[ii].dev_irq_disable_reg =
				t->mask_info[ii].irq_or_mask;
			else
				*(u64 *)t->mask_info[ii].dev_irq_disable_reg =
				(*(u64 *)t->mask_info[ii].dev_irq_mask_reg &
				t->mask_info[ii].irq_and_mask) |
				t->mask_info[ii].irq_or_mask;
		} else if (t->mask_info[ii].dev_irq_disable_reg) {
			if (t->mask_info[ii].irq_and_mask == 0) {
				*t->mask_info[ii].dev_irq_disable_reg =
				t->mask_info[ii].irq_or_mask;
			} else {
				*t->mask_info[ii].dev_irq_disable_reg =
				(*t->mask_info[ii].dev_irq_mask_reg &
				t->mask_info[ii].irq_and_mask) |
				t->mask_info[ii].irq_or_mask;
			}
		}
	}
	if ((ii == 0) && t->type == MODS_IRQ_TYPE_CPU) {
		mods_debug_printk(DEBUG_ISR, "IRQ_DISABLE_NOSYNC ");
		disable_irq_nosync(t->apic_irq);
	}
}

#ifdef CONFIG_PCI
static const char *mods_irq_type_name(u8 irq_type)
{
	switch (irq_type) {
	case MODS_IRQ_TYPE_INT:
		return "INTx";
	case MODS_IRQ_TYPE_MSI:
		return "MSI";
	case MODS_IRQ_TYPE_CPU:
		return "CPU";
	case MODS_IRQ_TYPE_MSIX:
		return "MSI-X";
	default:
		return "unknown";
	}
}
#endif

static void wake_up_client(struct dev_irq_map *t)
{
	struct mods_client *client = &mp.clients[t->client_id - 1];

	if (client)
		wake_up_interruptible(&client->interrupt_event);
}

static int rec_irq_done(struct dev_irq_map *t,
			 unsigned int irq_time)
{
	struct irq_q_info *q;

	/* Get interrupt queue */
	q = &mp.clients[t->client_id - 1].irq_queue;

	/* Don't do anything if the IRQ has already been recorded */
	if (q->head != q->tail) {
		unsigned int i;

		for (i = q->head; i != q->tail; i++) {
			struct irq_q_data *pd
					= q->data+(i & (MODS_MAX_IRQS - 1));

			if ((pd->irq == t->apic_irq) &&
			    (!t->dev || (pd->dev == t->dev)))
				return false;
		}
	}

	/* Print an error if the queue is full */
	/* This is deadly! */
	if (q->tail - q->head == MODS_MAX_IRQS) {
		mods_error_printk("IRQ queue is full\n");
		return false;
	}

	/* Record the device which generated the IRQ in the queue */
	q->data[q->tail & (MODS_MAX_IRQS - 1)].dev       = t->dev;
	q->data[q->tail & (MODS_MAX_IRQS - 1)].irq       = t->apic_irq;
	q->data[q->tail & (MODS_MAX_IRQS - 1)].irq_index = t->entry;
	q->data[q->tail & (MODS_MAX_IRQS - 1)].time      = irq_time;
	q->tail++;

#ifdef CONFIG_PCI
	if (t->dev) {
		mods_debug_printk(DEBUG_ISR_DETAILED,
			"%04x:%x:%02x.%x %s IRQ 0x%x time=%uus\n",
				  (unsigned int)(pci_domain_nr(t->dev->bus)),
				  (unsigned int)(t->dev->bus->number),
				  (unsigned int)PCI_SLOT(t->dev->devfn),
				  (unsigned int)PCI_FUNC(t->dev->devfn),
			mods_irq_type_name(t->type),
			t->apic_irq,
			irq_time);
	} else
#endif
		mods_debug_printk(DEBUG_ISR_DETAILED,
				  "CPU IRQ 0x%x, time=%uus\n",
				  t->apic_irq,
				  irq_time);

	return true;
}

/* mods_irq_handle - interrupt function */
static irqreturn_t mods_irq_handle(int irq, void *data
#ifndef MODS_IRQ_HANDLE_NO_REGS
			, struct pt_regs *regs
#endif
)
{
	struct dev_irq_map *t        = (struct dev_irq_map *)data;
	int                 serviced = false;

	if (unlikely(!t))
		mods_error_printk("received irq %d, but no context for it\n",
			irq);
	else if (unlikely(t->apic_irq != irq))
		mods_error_printk("received irq %d which doesn't match registered irq %d\n",
			irq, t->apic_irq);
	else {
		unsigned long       flags    = 0;
		int                 recorded = false;
		unsigned int        irq_time = get_cur_time();
		struct mods_client *client   = &mp.clients[t->client_id - 1];

		spin_lock_irqsave(&client->irq_lock, flags);

		/* Check if the interrupt is still pending (shared INTA) */
		if (mods_check_interrupt(t)) {

			/* Disable interrupts on this device to avoid interrupt
			 * storm
			 */
			mods_disable_interrupts(t);

			/* Record IRQ for MODS and wake MODS up */
			recorded = rec_irq_done(t, irq_time);

			serviced = true;
		}

		spin_unlock_irqrestore(&client->irq_lock, flags);

		if (recorded)
			wake_up_client(t);
	}

	return IRQ_RETVAL(serviced);
}

static int mods_lookup_cpu_irq(u8 client_id, unsigned int irq)
{
	u8  client_idx;
	int ret = IRQ_NOT_FOUND;

	LOG_ENT();

	for (client_idx = 1; client_idx < MODS_MAX_CLIENTS; client_idx++) {
		struct dev_irq_map *t    = NULL;
		struct dev_irq_map *next = NULL;

		if (!test_bit(client_idx - 1, &mp.client_flags))
			continue;

		list_for_each_entry_safe(t,
					 next,
					 &mp.clients[client_idx - 1].irq_list,
					 list) {
			if (t->apic_irq == irq) {
				if (client_id == 0) {
					ret = IRQ_FOUND;
				} else {
					ret = (client_id == client_idx)
						  ? IRQ_FOUND : IRQ_NOT_FOUND;
				}
				/* Break out of the outer loop */
				client_idx = MODS_MAX_CLIENTS;
				break;
			}
		}
	}
	LOG_EXT();
	return ret;
}

#ifdef CONFIG_PCI
static int is_nvidia_gpu(struct pci_dev *dev)
{
	unsigned short class_code, vendor_id;

	pci_read_config_word(dev, PCI_CLASS_DEVICE, &class_code);
	pci_read_config_word(dev, PCI_VENDOR_ID, &vendor_id);
	if (((class_code == PCI_CLASS_DISPLAY_VGA) ||
	    (class_code == PCI_CLASS_DISPLAY_3D)) && (vendor_id == 0x10DE)) {
		return true;
	}
	return false;
}
#endif

#ifdef CONFIG_PCI
static void setup_mask_info(struct dev_irq_map *newmap,
			    struct MODS_REGISTER_IRQ_4 *p,
			    struct pci_dev *pdev)
{
	/* account for legacy adapters */
	char *bar = newmap->dev_irq_aperture;
	u32 ii = 0;

	if ((p->mask_info_cnt == 0) && is_nvidia_gpu(pdev)) {
		newmap->mask_info_cnt = 1;
		newmap->mask_info[0].dev_irq_mask_reg = (u32 *)(bar+0x140);
		newmap->mask_info[0].dev_irq_disable_reg = (u32 *)(bar+0x140);
		newmap->mask_info[0].dev_irq_state = (u32 *)(bar+0x100);
		newmap->mask_info[0].irq_and_mask = 0;
		newmap->mask_info[0].irq_or_mask = 0;
		return;
	}
	/* setup for new adapters */
	newmap->mask_info_cnt = p->mask_info_cnt;
	for (ii = 0; ii < p->mask_info_cnt; ii++) {
		newmap->mask_info[ii].dev_irq_state =
		    (u32 *)(bar + p->mask_info[ii].irq_pending_offset);
		newmap->mask_info[ii].dev_irq_mask_reg =
		    (u32 *)(bar + p->mask_info[ii].irq_enabled_offset);
		newmap->mask_info[ii].dev_irq_disable_reg =
		    (u32 *)(bar + p->mask_info[ii].irq_disable_offset);
		newmap->mask_info[ii].irq_and_mask = p->mask_info[ii].and_mask;
		newmap->mask_info[ii].irq_or_mask = p->mask_info[ii].or_mask;
		newmap->mask_info[ii].mask_type = p->mask_info[ii].mask_type;
	}
}
#endif

static int add_irq_map(u8 client_id,
		       struct pci_dev *pdev,
		       struct MODS_REGISTER_IRQ_4 *p, u32 irq, u32 entry)
{
	u32 irq_type = MODS_IRQ_TYPE_FROM_FLAGS(p->irq_flags);
	struct dev_irq_map *newmap = NULL;
	u64 interrupt_flags = IRQF_TRIGGER_NONE;

	LOG_ENT();

	/* Get the flags based on the interrupt */
	switch (irq_type) {
	case MODS_IRQ_TYPE_INT:
		interrupt_flags = IRQF_SHARED;
		break;

	case MODS_IRQ_TYPE_CPU:
		interrupt_flags = MODS_IRQ_FLAG_FROM_FLAGS(p->irq_flags);
		break;

	default:
		break;

	}

	/* Allocate memory for the new entry */
	newmap = kmalloc(sizeof(*newmap), GFP_KERNEL | __GFP_NORETRY);
	if (unlikely(!newmap)) {
		LOG_EXT();
		return -ENOMEM;
	}

	/* Fill out the new entry */
	newmap->apic_irq = irq;
	newmap->dev = pdev;
	newmap->client_id = client_id;
	newmap->dev_irq_aperture = 0;
	newmap->mask_info_cnt = 0;
	newmap->type = irq_type;
	newmap->entry = entry;

	/* Enable IRQ for this device in the kernel */
	if (request_irq(
			irq,
			&mods_irq_handle,
			interrupt_flags,
			"nvidia mods",
			newmap)) {
		mods_error_printk("unable to enable IRQ 0x%x with flags 0x%llx\n",
				  irq,
				  interrupt_flags);
		kfree(newmap);
		LOG_EXT();
		return -EPERM;
	}

	/* Add the new entry to the list of all registered interrupts */
	list_add(&newmap->list, &mp.clients[client_id - 1].irq_list);

#ifdef CONFIG_PCI
	/* Map BAR0 to be able to disable interrupts */
	if ((irq_type == MODS_IRQ_TYPE_INT) &&
	    (p->aperture_addr != 0) &&
	    (p->aperture_size != 0)) {
		char *bar = ioremap_nocache(p->aperture_addr, p->aperture_size);

		if (!bar) {
			mods_debug_printk(DEBUG_ISR,
				"failed to remap aperture: 0x%llx size=0x%x\n",
				p->aperture_addr, p->aperture_size);
			LOG_EXT();
			return -EPERM;
		}

		newmap->dev_irq_aperture = bar;
		setup_mask_info(newmap, p, pdev);
	}
#endif

	/* Print out successful registration string */
	if (irq_type == MODS_IRQ_TYPE_CPU)
		mods_debug_printk(DEBUG_ISR, "registered CPU IRQ 0x%x\n", irq);
#ifdef CONFIG_PCI
	else if ((irq_type == MODS_IRQ_TYPE_INT) ||
		 (irq_type == MODS_IRQ_TYPE_MSI) ||
		 (irq_type == MODS_IRQ_TYPE_MSIX)) {
		mods_debug_printk(DEBUG_ISR,
		"%04x:%x:%02x.%x registered %s IRQ 0x%x\n",
		(unsigned int)(pci_domain_nr(pdev->bus)),
		(unsigned int)(pdev->bus->number),
		(unsigned int)PCI_SLOT(pdev->devfn),
		(unsigned int)PCI_FUNC(pdev->devfn),
		mods_irq_type_name(irq_type),
		  irq);
	}
#endif
#ifdef CONFIG_PCI_MSI
	else if (irq_type == MODS_IRQ_TYPE_MSI) {
		u16 control;
		u16 data;
		int cap_pos = pci_find_capability(pdev, PCI_CAP_ID_MSI);

		pci_read_config_word(pdev, MSI_CONTROL_REG(cap_pos), &control);
		if (IS_64BIT_ADDRESS(control))
			pci_read_config_word(pdev,
						 MSI_DATA_REG(cap_pos, 1),
						 &data);
		else
			pci_read_config_word(pdev,
						 MSI_DATA_REG(cap_pos, 0),
						 &data);
		mods_debug_printk(DEBUG_ISR,
			"%04x:%x:%02x.%x registered MSI IRQ 0x%x data:0x%02x\n",
			(unsigned int)(pci_domain_nr(pdev->bus)),
			(unsigned int)(pdev->bus->number),
			(unsigned int)PCI_SLOT(pdev->devfn),
			(unsigned int)PCI_FUNC(pdev->devfn),
			irq,
			(unsigned int)data);
	} else if (irq_type == MODS_IRQ_TYPE_MSIX) {
		mods_debug_printk(DEBUG_ISR,
			"%04x:%x:%02x.%x registered MSI-X IRQ 0x%x\n",
			(unsigned int)(pci_domain_nr(pdev->bus)),
			(unsigned int)(pdev->bus->number),
			(unsigned int)PCI_SLOT(pdev->devfn),
			(unsigned int)PCI_FUNC(pdev->devfn),
			irq);
	}
#endif

	LOG_EXT();
	return OK;
}

static void mods_free_map(struct dev_irq_map *del)
{
	unsigned long       flags  = 0;
	struct mods_client *client = &mp.clients[del->client_id - 1];

	LOG_ENT();

	/* Disable interrupts on the device */
	spin_lock_irqsave(&client->irq_lock, flags);
	mods_disable_interrupts(del);
	spin_unlock_irqrestore(&client->irq_lock, flags);

	/* Unhook interrupts in the kernel */
	free_irq(del->apic_irq, del);

	/* Unmap aperture used for masking irqs */
	if (del->dev_irq_aperture)
		iounmap(del->dev_irq_aperture);

	/* Free memory */
	kfree(del);

	LOG_EXT();
}

void mods_init_irq(void)
{
	LOG_ENT();

	memset(&mp, 0, sizeof(mp));

	mutex_init(&mp.mtx);

	LOG_EXT();
}

void mods_cleanup_irq(void)
{
	int i;

	LOG_ENT();
	for (i = 0; i < MODS_MAX_CLIENTS; i++) {
		if (mp.client_flags && (1 << i))
			mods_free_client(i + 1);
	}
	LOG_EXT();
}

int mods_irq_event_check(u8 client_id)
{
	struct irq_q_info *q = &mp.clients[client_id - 1].irq_queue;
	unsigned int pos = (1 << (client_id - 1));

	if (!(mp.client_flags & pos))
		return POLLERR; /* irq has quit */

	if (q->head != q->tail)
		return POLLIN; /* irq generated */

	return 0;
}

struct mods_client *mods_alloc_client(void)
{
	u8 idx = 0;
	u8 max_clients = 1;

	LOG_ENT();

	if (mods_get_multi_instance() ||
	    (mods_get_access_token() != MODS_ACCESS_TOKEN_NONE))
		max_clients = MODS_MAX_CLIENTS;

	for (idx = 0; idx < max_clients; idx++) {
		if (!test_and_set_bit(idx, &mp.client_flags)) {
			struct mods_client *client = &mp.clients[idx];

			mods_debug_printk(DEBUG_IOCTL,
					  "open client %u (bit mask 0x%lx)\n",
					  (unsigned int)(idx + 1),
					  mp.client_flags);

			memset(client, 0, sizeof(*client));
			client->client_id = idx + 1;
			client->access_token = MODS_ACCESS_TOKEN_NONE;
			mutex_init(&client->mtx);
			spin_lock_init(&client->irq_lock);
			init_waitqueue_head(&client->interrupt_event);
			INIT_LIST_HEAD(&client->irq_list);
			INIT_LIST_HEAD(&client->mem_alloc_list);
			INIT_LIST_HEAD(&client->mem_map_list);
#if defined(CONFIG_PPC64)
			INIT_LIST_HEAD(&client->ppc_tce_bypass_list);
			INIT_LIST_HEAD(&client->nvlink_sysmem_trained_list);
#endif

			LOG_EXT();
			return client;
		}
	}

	LOG_EXT();
	return NULL;
}

static int mods_free_irqs(u8 client_id, struct pci_dev *dev)
{
#ifdef CONFIG_PCI
	struct mods_client  *client = &mp.clients[client_id - 1];
	struct dev_irq_map  *del    = NULL;
	struct dev_irq_map  *next;
	struct en_dev_entry *dpriv;
	unsigned int         irq_type;

	LOG_ENT();

	if (unlikely(mutex_lock_interruptible(&mp.mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	dpriv = pci_get_drvdata(dev);

	if (!dpriv) {
		mutex_unlock(&mp.mtx);
		LOG_EXT();
		return OK;
	}

	if (dpriv->client_id != client_id) {
		mods_error_printk("invalid client %u for device %04x:%x:%02x.%x\n",
				  (unsigned int)client_id,
				  pci_domain_nr(dev->bus),
				  dev->bus->number,
				  PCI_SLOT(dev->devfn),
				  PCI_FUNC(dev->devfn));
		mutex_unlock(&mp.mtx);
		LOG_EXT();
		return -EINVAL;
	}

	mods_debug_printk(DEBUG_ISR_DETAILED,
		"(dev=%04x:%x:%02x.%x) irq_flags=0x%x nvecs=%d\n",
		pci_domain_nr(dev->bus),
		dev->bus->number,
		PCI_SLOT(dev->devfn),
		PCI_FUNC(dev->devfn),
		dpriv->irq_flags,
		dpriv->nvecs
		);

	/* Delete device interrupts from the list */
	list_for_each_entry_safe(del, next, &client->irq_list, list) {
		if (dev == del->dev) {
			u8 type = del->type;

			list_del(&del->list);
			mods_debug_printk(DEBUG_ISR,
				"%04x:%x:%02x.%x unregistered %s IRQ 0x%x\n",
				pci_domain_nr(dev->bus),
				dev->bus->number,
				PCI_SLOT(dev->devfn),
				PCI_FUNC(dev->devfn),
				mods_irq_type_name(type),
				del->apic_irq);
			mods_free_map(del);

			BUG_ON(type !=
			       MODS_IRQ_TYPE_FROM_FLAGS(dpriv->irq_flags));
			if (type != MODS_IRQ_TYPE_MSIX)
				break;
		}
	}

	mods_debug_printk(DEBUG_ISR_DETAILED, "before disable\n");
#ifdef CONFIG_PCI_MSI
	irq_type = MODS_IRQ_TYPE_FROM_FLAGS(dpriv->irq_flags);

	if (irq_type == MODS_IRQ_TYPE_MSIX) {
		pci_disable_msix(dev);
		kfree(dpriv->msix_entries);
		dpriv->msix_entries = 0;
	} else if (irq_type == MODS_IRQ_TYPE_MSI) {
		pci_disable_msi(dev);
	}
#endif

	dpriv->nvecs = 0;
	mods_debug_printk(DEBUG_ISR_DETAILED, "irqs freed\n");
#endif

	mutex_unlock(&mp.mtx);
	LOG_EXT();
	return 0;
}

void mods_free_client_interrupts(struct mods_client *client)
{
	struct en_dev_entry *dpriv = client->enabled_devices;

	LOG_ENT();

	/* Release all interrupts */
	while (dpriv) {
		mods_free_irqs(client->client_id, dpriv->dev);
		dpriv = dpriv->next;
	}

	LOG_EXT();
}

void mods_free_client(u8 client_id)
{
	struct mods_client *client = &mp.clients[client_id - 1];

	LOG_ENT();

	memset(client, 0, sizeof(*client));

	/* Indicate the client_id is free */
	clear_bit(client_id - 1, &mp.client_flags);

	mods_debug_printk(DEBUG_IOCTL, "closed client %u\n",
			  (unsigned int)client_id);
	LOG_EXT();
}

#ifdef CONFIG_PCI
static int mods_allocate_irqs(u8 client_id, struct file *pfile,
			      struct pci_dev *dev, u32 nvecs, u32 flags)
{
	struct mods_client  *client = pfile->private_data;
	struct en_dev_entry *dpriv;
	unsigned int         irq_type = MODS_IRQ_TYPE_FROM_FLAGS(flags);

	LOG_ENT();

	mods_debug_printk(DEBUG_ISR_DETAILED,
		"(dev=%04x:%x:%02x.%x, flags=0x%x, nvecs=%d)\n",
		pci_domain_nr(dev->bus),
		dev->bus->number,
		PCI_SLOT(dev->devfn),
		PCI_FUNC(dev->devfn),
		flags,
		nvecs);

	/* Determine if the device supports requested interrupt type */
	if (irq_type == MODS_IRQ_TYPE_MSI) {
#ifdef CONFIG_PCI_MSI
		if (pci_find_capability(dev, PCI_CAP_ID_MSI) == 0) {
			mods_error_printk(
				"dev %04x:%x:%02x.%x does not support MSI\n",
				pci_domain_nr(dev->bus),
				dev->bus->number,
				PCI_SLOT(dev->devfn),
				PCI_FUNC(dev->devfn));
			LOG_EXT();
			return -EINVAL;
		}
#else
		mods_error_printk("the kernel does not support MSI!\n");
		return -EINVAL;
#endif
	} else if (irq_type == MODS_IRQ_TYPE_MSIX) {
#ifdef CONFIG_PCI_MSI
		if (pci_find_capability(dev, PCI_CAP_ID_MSIX) == 0) {
			mods_error_printk(
				"dev %04x:%x:%02x.%x does not support MSI-X\n",
				pci_domain_nr(dev->bus),
				dev->bus->number,
				PCI_SLOT(dev->devfn),
				PCI_FUNC(dev->devfn));
			LOG_EXT();
			return -EINVAL;
		}
#else
		mods_error_printk("the kernel does not support MSI-X!\n");
		return -EINVAL;
#endif
	}

	/* Enable device on the PCI bus */
	dpriv = mods_enable_device(client, dev);
	if (!dpriv) {
		LOG_EXT();
		return -EINVAL;
	}

	if (irq_type == MODS_IRQ_TYPE_INT) {
		/* use legacy irq */
		if (nvecs != 1) {
			mods_error_printk("INTA: only 1 INTA vector supported requested %d!\n",
				nvecs);
			LOG_EXT();
			return -EINVAL;
		}
		dpriv->nvecs = 1;
	}
	/* Enable MSI */
#ifdef CONFIG_PCI_MSI
	else if (irq_type == MODS_IRQ_TYPE_MSI) {
		if (nvecs != 1) {
			mods_error_printk("MSI: only 1 MSI vector supported requested %d!\n",
				nvecs);
			LOG_EXT();
			return -EINVAL;
		}
		if (pci_enable_msi(dev) != 0) {
			mods_error_printk(
				"unable to enable MSI on dev %04x:%x:%02x.%x\n",
				pci_domain_nr(dev->bus),
				dev->bus->number,
				PCI_SLOT(dev->devfn),
				PCI_FUNC(dev->devfn));
			LOG_EXT();
			return -EINVAL;
		}
		dpriv->nvecs = 1;
	} else if (irq_type == MODS_IRQ_TYPE_MSIX) {
		struct msix_entry *entries;
		int i = 0, cnt = 1;

		entries = kcalloc(nvecs, sizeof(struct msix_entry),
				GFP_KERNEL | __GFP_NORETRY);

		if (!entries) {
			mods_error_printk("could not allocate %d MSI-X entries!\n",
				nvecs);
			LOG_EXT();
			return -ENOMEM;
		}

		for (i = 0; i < nvecs; i++)
			entries[i].entry = (uint16_t)i;

#ifdef MODS_HAS_MSIX_RANGE
		cnt = pci_enable_msix_range(dev, entries, nvecs, nvecs);

		if (cnt < 0) {
			/* returns number of interrupts allocated
			 * < 0 indicates a failure.
			 */
			mods_error_printk(
				"could not allocate the requested number of MSI-X vectors=%d return=%d!\n",
				nvecs, cnt);
			kfree(entries);
			LOG_EXT();
			return cnt;
		}
#else
		cnt = pci_enable_msix(dev, entries, nvecs);

		if (cnt) {
			/*  A return of < 0 indicates a failure.
			 *  A return of > 0 indicates that driver request is
			 *  exceeding the number of irqs or MSI-X
			 *  vectors available
			 */
			mods_error_printk(
				"could not allocate the requested number of MSI-X vectors=%d return=%d!\n",
				nvecs, cnt);
			kfree(entries);
			LOG_EXT();
			if (cnt > 0)
				cnt = -ENOSPC;
			return cnt;
		}
#endif

		mods_debug_printk(DEBUG_ISR,
			"allocated %d irq's of type %s(%d)\n",
			nvecs, mods_irq_type_name(irq_type), irq_type);

		for (i = 0; i < nvecs; i++)
			mods_debug_printk(DEBUG_ISR, "vec %d %x\n",
				entries[i].entry, entries[i].vector);

		dpriv->nvecs = nvecs;
		dpriv->msix_entries = entries;
	}
#endif
	else {
		mods_error_printk("unsupported irq_type %d dev: %04x:%x:%02x.%x\n",
				  irq_type,
				  pci_domain_nr(dev->bus),
				  dev->bus->number,
				  PCI_SLOT(dev->devfn),
				  PCI_FUNC(dev->devfn));
		LOG_EXT();
		return -EINVAL;
	}

	dpriv->client_id = client_id;
	dpriv->irq_flags = flags;
	LOG_EXT();
	return OK;
}

static int mods_register_pci_irq(struct file *pfile,
				 struct MODS_REGISTER_IRQ_4 *p)
{
	int rc = OK;
	unsigned int irq_type = MODS_IRQ_TYPE_FROM_FLAGS(p->irq_flags);
	struct pci_dev *dev;
	struct en_dev_entry *dpriv;
	unsigned int devfn;
	u8 client_id;
	int i;

	LOG_ENT();

	/* Identify the caller */
	client_id = get_client_id(pfile);
	WARN_ON(!is_client_id_valid(client_id));
	if (!is_client_id_valid(client_id)) {
		LOG_EXT();
		return -EINVAL;
	}

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

	if (!p->irq_count) {
		mods_error_printk("no irq's requested!\n");
		LOG_EXT();
		return -EINVAL;
	}

	if (unlikely(mutex_lock_interruptible(&mp.mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	dpriv = pci_get_drvdata(dev);
	if (dpriv) {
		if (dpriv->client_id != client_id) {
			mods_error_printk("dev %04x:%x:%02x.%x already owned by client %u\n",
					  (unsigned int)p->dev.domain,
					  (unsigned int)p->dev.bus,
					  (unsigned int)p->dev.device,
					  (unsigned int)p->dev.function,
					  (unsigned int)dpriv->client_id);
			mutex_unlock(&mp.mtx);
			LOG_EXT();
			return -EINVAL;
		}
		if (dpriv->nvecs) {
			mods_error_printk("interrupt for dev %04x:%x:%02x.%x already registered\n",
					  (unsigned int)p->dev.domain,
					  (unsigned int)p->dev.bus,
					  (unsigned int)p->dev.device,
					  (unsigned int)p->dev.function);
			mutex_unlock(&mp.mtx);
			LOG_EXT();
			return -EINVAL;
		}
	}

	if (mods_allocate_irqs(client_id, pfile, dev, p->irq_count,
			       p->irq_flags)) {
		mods_error_printk("could not allocate irqs for irq_type %d\n",
				  irq_type);
		mutex_unlock(&mp.mtx);
		LOG_EXT();
		return -EINVAL;
	}

	dpriv = pci_get_drvdata(dev);

	for (i = 0; i < p->irq_count; i++) {
		u32 irq = ((irq_type == MODS_IRQ_TYPE_INT) ||
				(irq_type == MODS_IRQ_TYPE_MSI)) ? dev->irq :
			   dpriv->msix_entries[i].vector;

		if (add_irq_map(client_id, dev, p, irq, i) != OK) {
#ifdef CONFIG_PCI_MSI
			if (irq_type == MODS_IRQ_TYPE_MSI)
				pci_disable_msi(dev);
			else if (irq_type == MODS_IRQ_TYPE_MSIX)
				pci_disable_msix(dev);
#endif
			mutex_unlock(&mp.mtx);
			LOG_EXT();
			return -EINVAL;
		}
	}

	mutex_unlock(&mp.mtx);
	LOG_EXT();
	return rc;
}
#endif /* CONFIG_PCI */

static int mods_register_cpu_irq(struct file *pfile,
				 struct MODS_REGISTER_IRQ_4 *p)
{
	u8 client_id;
	u32 irq = p->dev.bus;

	LOG_ENT();

	/* Identify the caller */
	client_id = get_client_id(pfile);
	WARN_ON(!is_client_id_valid(client_id));
	if (!is_client_id_valid(client_id)) {
		LOG_EXT();
		return -EINVAL;
	}

	if (unlikely(mutex_lock_interruptible(&mp.mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	/* Determine if the interrupt is already hooked */
	if (mods_lookup_cpu_irq(0, irq) == IRQ_FOUND) {
		mods_error_printk("CPU IRQ 0x%x has already been registered\n",
				  irq);
		mutex_unlock(&mp.mtx);
		LOG_EXT();
		return -EINVAL;
	}

	/* Register interrupt */
	if (add_irq_map(client_id, 0, p, irq, 0) != OK) {
		mutex_unlock(&mp.mtx);
		LOG_EXT();
		return -EINVAL;
	}

	mutex_unlock(&mp.mtx);
	return OK;
}

#ifdef CONFIG_PCI
static int mods_unregister_pci_irq(struct file *pfile,
				   struct MODS_REGISTER_IRQ_2 *p)
{
	struct pci_dev *dev;
	unsigned int devfn;
	u8 client_id;
	int rv = OK;

	LOG_ENT();

	/* Identify the caller */
	client_id = get_client_id(pfile);
	WARN_ON(!is_client_id_valid(client_id));
	if (!is_client_id_valid(client_id)) {
		LOG_EXT();
		return -EINVAL;
	}

	/* Get the PCI device structure for the specified device from kernel */
	devfn = PCI_DEVFN(p->dev.device, p->dev.function);
	dev = MODS_PCI_GET_SLOT(p->dev.domain, p->dev.bus, devfn);
	if (!dev) {
		LOG_EXT();
		return -EINVAL;
	}

	rv = mods_free_irqs(client_id, dev);

	LOG_EXT();
	return rv;
}
#endif

static int mods_unregister_cpu_irq(struct file *pfile,
				   struct MODS_REGISTER_IRQ_2 *p)
{
	struct dev_irq_map *del = NULL;
	struct dev_irq_map *next;
	unsigned int        irq;
	u8                  client_id;
	struct mods_client *client;

	LOG_ENT();

	irq = p->dev.bus;

	/* Identify the caller */
	client_id = get_client_id(pfile);
	WARN_ON(!is_client_id_valid(client_id));
	if (!is_client_id_valid(client_id)) {
		LOG_EXT();
		return -EINVAL;
	}
	client = &mp.clients[client_id - 1];

	if (unlikely(mutex_lock_interruptible(&mp.mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	/* Determine if the interrupt is already hooked by this client */
	if (mods_lookup_cpu_irq(client_id, irq) == IRQ_NOT_FOUND) {
		mods_error_printk(
			"IRQ 0x%x not hooked, can't unhook\n",
			irq);
		mutex_unlock(&mp.mtx);
		LOG_EXT();
		return -EINVAL;
	}

	/* Delete device interrupt from the list */
	list_for_each_entry_safe(del, next, &client->irq_list, list) {
		if ((irq == del->apic_irq) && (del->dev == 0)) {
			if (del->type != p->type) {
				mods_error_printk("wrong IRQ type passed\n");
				mutex_unlock(&mp.mtx);
				LOG_EXT();
				return -EINVAL;
			}
			list_del(&del->list);
			mods_debug_printk(DEBUG_ISR,
					  "unregistered CPU IRQ 0x%x\n",
					  irq);
			mods_free_map(del);
			break;
		}
	}

	mutex_unlock(&mp.mtx);
	LOG_EXT();
	return OK;
}

/*************************
 * ESCAPE CALL FUNCTIONS *
 *************************/

int esc_mods_register_irq_4(struct file *pfile,
			    struct MODS_REGISTER_IRQ_4 *p)
{
	u32 irq_type = MODS_IRQ_TYPE_FROM_FLAGS(p->irq_flags);

	if (irq_type == MODS_IRQ_TYPE_CPU)
		return mods_register_cpu_irq(pfile, p);
#ifdef CONFIG_PCI
	return mods_register_pci_irq(pfile, p);
#else
	mods_error_printk("PCI not available\n");
	return -EINVAL;
#endif
}

int esc_mods_register_irq_3(struct file *pfile,
			    struct MODS_REGISTER_IRQ_3 *p)
{
	struct MODS_REGISTER_IRQ_4 irq_data = { {0} };
	u32 ii = 0;

	irq_data.dev = p->dev;
	irq_data.aperture_addr = p->aperture_addr;
	irq_data.aperture_size = p->aperture_size;
	irq_data.mask_info_cnt = p->mask_info_cnt;
	for (ii = 0; ii < p->mask_info_cnt; ii++) {
		irq_data.mask_info[ii].mask_type = p->mask_info[ii].mask_type;
		irq_data.mask_info[ii].irq_pending_offset =
			p->mask_info[ii].irq_pending_offset;
		irq_data.mask_info[ii].irq_enabled_offset =
			p->mask_info[ii].irq_enabled_offset;
		irq_data.mask_info[ii].irq_enable_offset =
			p->mask_info[ii].irq_enable_offset;
		irq_data.mask_info[ii].irq_disable_offset =
			p->mask_info[ii].irq_disable_offset;
		irq_data.mask_info[ii].and_mask =
			p->mask_info[ii].and_mask;
		irq_data.mask_info[ii].or_mask =
			p->mask_info[ii].or_mask;
	}
	irq_data.irq_count = 1;
	irq_data.irq_flags = p->irq_type;

	return esc_mods_register_irq_4(pfile, &irq_data);
}

int esc_mods_register_irq_2(struct file *pfile,
			    struct MODS_REGISTER_IRQ_2 *p)
{
	struct MODS_REGISTER_IRQ_4 irq_data = { {0} };

	irq_data.dev = p->dev;
	irq_data.irq_count = 1;
	irq_data.irq_flags = p->type;

#ifdef CONFIG_PCI
	{
		/* Get the PCI device structure */
		unsigned int devfn;
		struct pci_dev *dev;

		devfn = PCI_DEVFN(p->dev.device, p->dev.function);
		dev  = MODS_PCI_GET_SLOT(p->dev.domain, p->dev.bus, devfn);
		if (!dev) {
			LOG_EXT();
			return -EINVAL;
		}
		irq_data.aperture_addr = pci_resource_start(dev, 0);
		irq_data.aperture_size = pci_resource_len(dev, 0);
	}
#endif

	return esc_mods_register_irq_4(pfile, &irq_data);
}

int esc_mods_register_irq(struct file *pfile,
			  struct MODS_REGISTER_IRQ *p)
{
	struct MODS_REGISTER_IRQ_2 register_irq = { {0} };

	register_irq.dev.domain		= 0;
	register_irq.dev.bus		= p->dev.bus;
	register_irq.dev.device		= p->dev.device;
	register_irq.dev.function	= p->dev.function;
	register_irq.type		= p->type;

	return esc_mods_register_irq_2(pfile, &register_irq);
}

int esc_mods_unregister_irq_2(struct file *pfile,
				  struct MODS_REGISTER_IRQ_2 *p)
{
	if (p->type == MODS_IRQ_TYPE_CPU)
		return mods_unregister_cpu_irq(pfile, p);
#ifdef CONFIG_PCI
	return mods_unregister_pci_irq(pfile, p);
#else
	return -EINVAL;
#endif
}

int esc_mods_unregister_irq(struct file *pfile,
				struct MODS_REGISTER_IRQ *p)
{
	struct MODS_REGISTER_IRQ_2 register_irq = { {0} };

	register_irq.dev.domain		= 0;
	register_irq.dev.bus		= p->dev.bus;
	register_irq.dev.device		= p->dev.device;
	register_irq.dev.function	= p->dev.function;
	register_irq.type		= p->type;

	return esc_mods_unregister_irq_2(pfile, &register_irq);
}

int esc_mods_query_irq_3(struct file *pfile, struct MODS_QUERY_IRQ_3 *p)
{
	u8                  client_id;
	struct mods_client *client;
	struct irq_q_info  *q        = NULL;
	unsigned int        i        = 0;
	unsigned long       flags    = 0;
	unsigned int        cur_time = get_cur_time();

	LOG_ENT();

	/* Identify the caller */
	client_id = get_client_id(pfile);
	WARN_ON(!is_client_id_valid(client_id));
	if (!is_client_id_valid(client_id)) {
		LOG_EXT();
		return -EINVAL;
	}
	client = &mp.clients[client_id - 1];

	/* Clear return array */
	memset(p->irq_list, 0xFF, sizeof(p->irq_list));

	/* Lock IRQ queue */
	spin_lock_irqsave(&client->irq_lock, flags);

	/* Fill in return array with IRQ information */
	q = &client->irq_queue;
	for (i = 0;
		 (q->head != q->tail) && (i < MODS_MAX_IRQS);
		 q->head++, i++) {

		unsigned int    index = q->head & (MODS_MAX_IRQS - 1);
		struct pci_dev *dev   = q->data[index].dev;

		if (dev) {
			p->irq_list[i].dev.domain = pci_domain_nr(dev->bus);
			p->irq_list[i].dev.bus = dev->bus->number;
			p->irq_list[i].dev.device = PCI_SLOT(dev->devfn);
			p->irq_list[i].dev.function = PCI_FUNC(dev->devfn);
		} else {
			p->irq_list[i].dev.domain = 0;
			p->irq_list[i].dev.bus = q->data[index].irq;
			p->irq_list[i].dev.device = 0xFFU;
			p->irq_list[i].dev.function = 0xFFU;
		}
		p->irq_list[i].irq_index = q->data[index].irq_index;
		p->irq_list[i].delay = cur_time - q->data[index].time;

		/* Print info about IRQ status returned */
		if (dev) {
			mods_debug_printk(DEBUG_ISR_DETAILED,
		   "retrieved IRQ index=%d dev %04x:%x:%02x.%x, time=%uus, delay=%uus\n",
				p->irq_list[i].irq_index,
				(unsigned int)p->irq_list[i].dev.domain,
				(unsigned int)p->irq_list[i].dev.bus,
				(unsigned int)p->irq_list[i].dev.device,
				(unsigned int)p->irq_list[i].dev.function,
				q->data[index].time,
				p->irq_list[i].delay);
		} else {
			mods_debug_printk(DEBUG_ISR_DETAILED,
				"retrieved IRQ 0x%x, time=%uus, delay=%uus\n",
				(unsigned int)p->irq_list[i].dev.bus,
				q->data[index].time,
				p->irq_list[i].delay);
		}
	}

	/* Indicate if there are more IRQs pending */
	if (q->head != q->tail)
		p->more = 1;

	/* Unlock IRQ queue */
	spin_unlock_irqrestore(&client->irq_lock, flags);

	LOG_EXT();
	return OK;
}

int esc_mods_query_irq_2(struct file *pfile, struct MODS_QUERY_IRQ_2 *p)
{
	int retval, i;
	struct MODS_QUERY_IRQ_3 query_irq = { { { { 0 } } } };

	retval = esc_mods_query_irq_3(pfile, &query_irq);
	if (retval)
		return retval;

	for (i = 0; i < MODS_MAX_IRQS; i++) {
		p->irq_list[i].dev   = query_irq.irq_list[i].dev;
		p->irq_list[i].delay = query_irq.irq_list[i].delay;
	}
	p->more = query_irq.more;
	return OK;
}

int esc_mods_query_irq(struct file *pfile,
			   struct MODS_QUERY_IRQ *p)
{
	int retval, i;
	struct MODS_QUERY_IRQ_3 query_irq = { { { { 0 } } } };

	retval = esc_mods_query_irq_3(pfile, &query_irq);
	if (retval)
		return retval;

	for (i = 0; i < MODS_MAX_IRQS; i++) {
		p->irq_list[i].dev.bus    = query_irq.irq_list[i].dev.bus;
		p->irq_list[i].dev.device = query_irq.irq_list[i].dev.device;
		p->irq_list[i].dev.function
					  = query_irq.irq_list[i].dev.function;
		p->irq_list[i].delay	  = query_irq.irq_list[i].delay;
	}
	p->more = query_irq.more;
	return OK;
}

int esc_mods_irq_handled_2(struct file *pfile,
			   struct MODS_REGISTER_IRQ_2 *p)
{
	u8                  client_id;
	struct mods_client *client;
	unsigned long       flags = 0;
	u32                 irq = p->dev.bus;
	struct dev_irq_map *t = NULL;
	struct dev_irq_map *next = NULL;
	int                 ret = -EINVAL;

	if (p->type != MODS_IRQ_TYPE_CPU)
		return -EINVAL;

	LOG_ENT();

	/* Identify the caller */
	client_id = get_client_id(pfile);
	WARN_ON(!is_client_id_valid(client_id));
	if (!is_client_id_valid(client_id)) {
		LOG_EXT();
		return -EINVAL;
	}
	client = &mp.clients[client_id - 1];

	/* Print info */
	mods_debug_printk(DEBUG_ISR_DETAILED,
			  "mark CPU IRQ 0x%x handled\n", irq);

	/* Lock IRQ queue */
	spin_lock_irqsave(&client->irq_lock, flags);

	list_for_each_entry_safe(t, next, &client->irq_list, list) {
		if (t->apic_irq == irq) {
			if (t->type != p->type) {
				mods_error_printk(
				"IRQ type doesn't match registered IRQ\n");
			} else {
				enable_irq(irq);
				ret = OK;
			}
			break;
		}
	}

	/* Unlock IRQ queue */
	spin_unlock_irqrestore(&client->irq_lock, flags);

	LOG_EXT();
	return ret;
}

int esc_mods_irq_handled(struct file *pfile,
			 struct MODS_REGISTER_IRQ *p)
{
	struct MODS_REGISTER_IRQ_2 register_irq = { {0} };

	register_irq.dev.domain		= 0;
	register_irq.dev.bus		= p->dev.bus;
	register_irq.dev.device		= p->dev.device;
	register_irq.dev.function	= p->dev.function;
	register_irq.type		= p->type;

	return esc_mods_irq_handled_2(pfile, &register_irq);
}

#if defined(MODS_TEGRA) && defined(CONFIG_OF_IRQ) && defined(CONFIG_OF)
int esc_mods_map_irq(struct file *pfile,
					 struct MODS_DT_INFO *p)
{
	int err = 0;
	/* the physical irq */
	int hwirq;
	/* platform device handle */
	struct platform_device *pdev = NULL;
	/* irq parameters */
	struct of_phandle_args oirq;
	/* Search for the node by device tree name */
	struct device_node *np = of_find_node_by_name(NULL, p->dt_name);
	if (!np) {
		mods_error_printk("node %s is not valid\n", p->full_name);
		err = -EINVAL;
		goto error;
	}

	/* Can be multiple nodes that share the same dt name, */
	/* make sure you get the correct node matched by the device's full */
	/* name in device tree (i.e. watchdog@30c0000 as opposed */
	/* to watchdog) */
	while (of_node_cmp(np->full_name, p->full_name)) {
		np = of_find_node_by_name(np, p->dt_name);
		if (!np) {
			mods_error_printk("Node %s is not valid\n",
					  p->full_name);
			err = -EINVAL;
			goto error;
		}
	}

	p->irq = irq_of_parse_and_map(np, p->index);
	err = of_irq_parse_one(np, p->index, &oirq);
	if (err) {
		mods_error_printk("Could not parse IRQ\n");
		goto error;
	}

	hwirq = oirq.args[1];
	/* Get the platform device handle */
	pdev = of_find_device_by_node(np);

	if (of_node_cmp(p->dt_name, "watchdog") == 0) {
		/* Enable and unmask interrupt for watchdog */
		struct resource *res_src = platform_get_resource(pdev,
		IORESOURCE_MEM, 0);
		struct resource *res_tke = platform_get_resource(pdev,
		IORESOURCE_MEM, 2);
		void __iomem *wdt_tke = devm_ioremap(&pdev->dev,
		res_tke->start, resource_size(res_tke));
		int wdt_index = ((res_src->start >> 16) & 0xF) - 0xc;

		writel(TOP_TKE_TKEIE_WDT_MASK(wdt_index), wdt_tke +
		TOP_TKE_TKEIE(hwirq));
	}

error:
	of_node_put(np);
	/* enable the interrupt */
	return err;
}

int esc_mods_map_irq_to_gpio(struct file* pfile,
				struct MODS_GPIO_INFO *p)
{
	//TODO: Make sure you are allocating gpio properly
	int gpio_handle;
	int irq;
	int err = 0;

	struct device_node *np = of_find_node_by_name(NULL, p->dt_name);

	if (!np) {
		mods_error_printk("Node %s is not valid\n", p->full_name);
		err = -EINVAL;
		goto error;
	}

	while (of_node_cmp(np->full_name, p->full_name)) {
		np = of_find_node_by_name(np, p->dt_name);
		if (!np) {
			mods_error_printk("Node %s is not valid\n",
					  p->full_name);
			err = -EINVAL;
			goto error;
		}
	}

	gpio_handle = of_get_named_gpio(np, p->name, 0);
	if (!gpio_is_valid(gpio_handle)) {
		mods_error_printk("gpio %s is missing\n", p->name);
		err = gpio_handle;
		goto error;
	}

	err = gpio_direction_input(gpio_handle);
	if (err < 0) {
		mods_error_printk("pex_rst_gpio input direction change failed\n");
		goto error;
	}

	irq = gpio_to_irq(gpio_handle);
	if (irq < 0) {
		mods_error_printk("Unable to get irq for pex_rst_gpio\n");
		err = -EINVAL;
		goto error;
	}
	p->irq = irq;

error:
	of_node_put(np);
	return err;
}
#endif
