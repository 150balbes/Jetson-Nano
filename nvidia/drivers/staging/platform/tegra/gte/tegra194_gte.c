/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio-tegra186.h>
#include <linux/platform_device.h>
#include <linux/tegra-gte.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <uapi/linux/tegra-gte-ioctl.h>

#define GTE_SUSPEND	0

/* GTE source clock TSC is 31.25MHz */
#define GTE_TS_NS	32ULL
#define GTE_TS_NS_SHIFT	__builtin_ctz(GTE_TS_NS)

/* char device related */
static DEFINE_IDA(gte_ida);
static dev_t gte_devt;
#define GTE_DEV_MAX	2
static struct bus_type gte_bus_type = {
	.name = "gte",
};

/* Global list of the probed GTE devices */
static DEFINE_MUTEX(gte_list_lock);
static LIST_HEAD(gte_devices);

/* AON GTE event map For slice 1 */
#define NV_AON_GTE_SLICE1_IRQ_GPIO_28	12
#define NV_AON_GTE_SLICE1_IRQ_GPIO_29	13
#define NV_AON_GTE_SLICE1_IRQ_GPIO_30	14
#define NV_AON_GTE_SLICE1_IRQ_GPIO_31	15
#define NV_AON_GTE_SLICE1_IRQ_GPIO_32	16
#define NV_AON_GTE_SLICE1_IRQ_GPIO_33	17
#define NV_AON_GTE_SLICE1_IRQ_GPIO_34	18
#define NV_AON_GTE_SLICE1_IRQ_GPIO_35	19
#define NV_AON_GTE_SLICE1_IRQ_GPIO_36	20
#define NV_AON_GTE_SLICE1_IRQ_GPIO_37	21
#define NV_AON_GTE_SLICE1_IRQ_GPIO_38	22
#define NV_AON_GTE_SLICE1_IRQ_GPIO_39	23
#define NV_AON_GTE_SLICE1_IRQ_GPIO_40	24
#define NV_AON_GTE_SLICE1_IRQ_GPIO_41	25
#define NV_AON_GTE_SLICE1_IRQ_GPIO_42	26
#define NV_AON_GTE_SLICE1_IRQ_GPIO_43	27

/* AON GTE event map For slice 2 */
#define NV_AON_GTE_SLICE2_IRQ_GPIO_0	0
#define NV_AON_GTE_SLICE2_IRQ_GPIO_1	1
#define NV_AON_GTE_SLICE2_IRQ_GPIO_2	2
#define NV_AON_GTE_SLICE2_IRQ_GPIO_3	3
#define NV_AON_GTE_SLICE2_IRQ_GPIO_4	4
#define NV_AON_GTE_SLICE2_IRQ_GPIO_5	5
#define NV_AON_GTE_SLICE2_IRQ_GPIO_6	6
#define NV_AON_GTE_SLICE2_IRQ_GPIO_7	7
#define NV_AON_GTE_SLICE2_IRQ_GPIO_8	8
#define NV_AON_GTE_SLICE2_IRQ_GPIO_9	9
#define NV_AON_GTE_SLICE2_IRQ_GPIO_10	10
#define NV_AON_GTE_SLICE2_IRQ_GPIO_11	11
#define NV_AON_GTE_SLICE2_IRQ_GPIO_12	12
#define NV_AON_GTE_SLICE2_IRQ_GPIO_13	13
#define NV_AON_GTE_SLICE2_IRQ_GPIO_14	14
#define NV_AON_GTE_SLICE2_IRQ_GPIO_15	15
#define NV_AON_GTE_SLICE2_IRQ_GPIO_16	16
#define NV_AON_GTE_SLICE2_IRQ_GPIO_17	17
#define NV_AON_GTE_SLICE2_IRQ_GPIO_18	18
#define NV_AON_GTE_SLICE2_IRQ_GPIO_19	19
#define NV_AON_GTE_SLICE2_IRQ_GPIO_20	20
#define NV_AON_GTE_SLICE2_IRQ_GPIO_21	21
#define NV_AON_GTE_SLICE2_IRQ_GPIO_22	22
#define NV_AON_GTE_SLICE2_IRQ_GPIO_23	23
#define NV_AON_GTE_SLICE2_IRQ_GPIO_24	24
#define NV_AON_GTE_SLICE2_IRQ_GPIO_25	25
#define NV_AON_GTE_SLICE2_IRQ_GPIO_26	26
#define NV_AON_GTE_SLICE2_IRQ_GPIO_27	27

#define GTE_TECTRL		0x0
#define GTE_TETSCH		0x4
#define GTE_TETSCL		0x8
#define GTE_TESRC		0xC
#define GTE_TECCV		0x10
#define GTE_TEPCV		0x14
#define GTE_TEENCV		0x18
#define GTE_TECMD		0x1C
#define GTE_TESTATUS		0x20
#define GTE_SLICE0_TETEN	0x40
#define GTE_SLICE1_TETEN	0x60

#define GTE_SLICE_SIZE		(GTE_SLICE1_TETEN - GTE_SLICE0_TETEN)

#define GTE_TECTRL_ENABLE_MASK		0x1
#define GTE_TECTRL_ENABLE_DISABLE	0x0
#define GTE_TECTRL_ENABLE_ENABLE	0x1

#define GTE_TECTRL_OCCU_SHIFT		0x8
#define GTE_TECTRL_INTR_SHIFT		0x1
#define GTE_TECTRL_INTR_ENABLE		0x1

#define GTE_TESRC_SLICE_SHIFT		16
#define GTE_TESRC_SLICE_DEFAULT_MASK	0xFF

#define GTE_TECMD_CMD_POP		0x1

#define GTE_TESTATUS_OCCUPANCY_SHIFT	8
#define GTE_TESTATUS_OCCUPANCY_MASK	0xFF

#define GTE_EVENT_REGISTERED		0
#define GTE_EVENT_UNREGISTERING		1

#define GTE_EV_FIFO_EL			32
#define GTE_MAX_EV_NAME_SZ		9

struct gte_slices {
	u32 r_val;
	unsigned long flags;
	/* to prevent events mapped to same slice updating its register */
	spinlock_t s_lock;
};

struct tegra_gte_ev_mapped {
	u32 pin;
	u32 slice;
	u32 bit_index;
};

struct tegra_gte_ev_table {
	int map_sz;
	const struct tegra_gte_ev_mapped *map;
};

struct tegra_gte_dev {
	int gte_irq;
	int num_events;
	int id;
	int gpio_base;
	u32 itr_thrshld;
	u32 conf_rval;
	atomic_t usage;
	struct device c_dev;
	struct cdev chrdev;
	struct device *pdev;
	struct kobject *kobj;
	struct gte_slices *sl;
	struct tegra_gte_event_info *ev;
	struct device_node *mp;
	const struct tegra_gte_ev_table *ev_map;
	struct list_head list;
	void __iomem *regs;
};

struct tegra_gte_ev_el {
	int dir;
	u64 tsc;
};

struct tegra_gte_event_info {
	u32 reg;
	unsigned long flags;
	atomic_t usage;
	atomic_t dropped_evs;
	spinlock_t lock; /* Sync ev_fifo accesses */
	struct mutex ev_lock;
	struct kobject kobj;
	struct kfifo ev_fifo;
	struct tegra_gte_dev *dev;
	struct tegra_gte_ev_desc pv;
};

static const struct tegra_gte_ev_mapped tegra194_aon_gpio_map[] = {
	/* pin num, slice, bit_index */
	[0]  = {11, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_0},
	[1]  = {10, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_1},
	[2]  = {9, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_2},
	[3]  = {8, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_3},
	[4]  = {7, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_4},
	[5]  = {6, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_5},
	[6]  = {5, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_6},
	[7]  = {4, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_7},
	[8]  = {3, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_8},
	[9]  = {2, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_9},
	[10] = {1, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_10},
	[11] = {0, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_11},
	[12] = {26, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_12},
	[13] = {25, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_13},
	[14] = {24, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_14},
	[15] = {23, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_15},
	[16] = {22, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_16},
	[17] = {21, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_17},
	[18] = {20, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_18},
	[19] = {19, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_19},
	[20] = {18, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_20},
	[21] = {17, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_21},
	[22] = {16, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_22},
	[23] = {38, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_23},
	[24] = {37, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_24},
	[25] = {36, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_25},
	[26] = {35, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_26},
	[27] = {34, 2, NV_AON_GTE_SLICE2_IRQ_GPIO_27},
	[28] = {33, 1, NV_AON_GTE_SLICE1_IRQ_GPIO_28},
	[29] = {32, 1, NV_AON_GTE_SLICE1_IRQ_GPIO_29},
};

static const struct tegra_gte_ev_table aon_gte_map = {
	.map_sz = ARRAY_SIZE(tegra194_aon_gpio_map),
	.map = tegra194_aon_gpio_map,
};

static inline u32 tegra_gte_readl(struct tegra_gte_dev *gte, u32 reg)
{
	return readl(gte->regs + reg);
}

static inline void tegra_gte_writel(struct tegra_gte_dev *gte, u32 reg,
				    u32 val)
{
	writel(val, gte->regs + reg);
}

/* Event specific sysfs management */

static struct kobj_type gte_ev_kobj_type = {
	.sysfs_ops = &kobj_sysfs_ops,
};

/* Stat for the dropped events due to FIFO overflow */
static ssize_t show_num_dropped_events(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       char *buf)
{
	struct tegra_gte_event_info *ev =
			container_of(kobj, struct tegra_gte_event_info, kobj);
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&ev->dropped_evs));
}

/* Total timestamp elements available to retrieve */
static ssize_t show_num_events_avail(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     char *buf)
{
	struct tegra_gte_event_info *ev =
			container_of(kobj, struct tegra_gte_event_info, kobj);
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&ev->usage));
}

struct kobj_attribute num_events_avail_attr =
	__ATTR(num_events_avail, 0400, show_num_events_avail, NULL);
struct kobj_attribute num_dropped_events_attr =
	__ATTR(num_dropped_events, 0400, show_num_dropped_events, NULL);

static struct attribute *ev_attrs[] = {
	&num_events_avail_attr.attr,
	&num_dropped_events_attr.attr,
	NULL,
};

static struct attribute_group event_attr_group = {
	.attrs = ev_attrs,
};

/*
 * Clears the resources related to event, resets the FIFO, removes the sysfs
 * related to event
 */
int tegra_gte_unregister_event(struct tegra_gte_ev_desc *data)
{
	u32 slice, val;
	int ev_id, g_id, ret;
	unsigned long flags;
	struct tegra_gte_dev *gte_dev;
	struct tegra_gte_ev_desc *pri;
	struct tegra_gte_event_info *ev;

	if (!data)
		return -EINVAL;

	pri = data;
	ev = container_of(pri, struct tegra_gte_event_info, pv);
	gte_dev = ev->dev;
	ev_id = pri->id;
	g_id = pri->gid;
	slice = pri->slice;

	set_bit(GTE_EVENT_UNREGISTERING, &ev->flags);

	if (!test_bit(GTE_EVENT_REGISTERED, &ev->flags)) {
		dev_dbg(gte_dev->pdev, "Event:%d is not registered", ev_id);
		ret = -EUSERS;
		goto clear_unregister;
	}

	spin_lock(&gte_dev->sl[slice].s_lock);
	if (test_bit(GTE_SUSPEND, &gte_dev->sl[slice].flags)) {
		spin_unlock(&gte_dev->sl[slice].s_lock);
		dev_dbg(gte_dev->pdev, "device suspended\n");
		ret = -EBUSY;
		goto clear_unregister;
	}

	val = tegra_gte_readl(gte_dev, ev->reg);
	val = val & (~(1 << pri->ev_bit));
	tegra_gte_writel(gte_dev, ev->reg, val);
	spin_unlock(&gte_dev->sl[slice].s_lock);

	if (g_id != -EINVAL)
		gpio_timestamp_control(g_id, 0);

	spin_lock_irqsave(&ev->lock, flags);
	clear_bit(GTE_EVENT_REGISTERED, &ev->flags);
	spin_unlock_irqrestore(&ev->lock, flags);

	atomic_dec(&gte_dev->usage);
	atomic_set(&ev->usage, 0);
	atomic_set(&ev->dropped_evs, 0);
	memset(&ev->pv, 0, sizeof(ev->pv));
	kobject_put(&ev->kobj);
	memset(&ev->kobj, 0, sizeof(ev->kobj));
	kfifo_free(&ev->ev_fifo);
	ret = 0;
	dev_dbg(gte_dev->pdev, "%s: event id:%d, g_id: %d, slice:%d",
		__func__, ev_id, g_id, slice);

clear_unregister:
	clear_bit(GTE_EVENT_UNREGISTERING, &ev->flags);
	return ret;
}
EXPORT_SYMBOL(tegra_gte_unregister_event);

static void tegra_gte_map_to_ev_id(u32 eid, struct tegra_gte_dev *gdev,
				   u32 *mapped)
{
	const struct tegra_gte_ev_mapped *m;
	int nums, i;

	if (gdev->ev_map) {
		m = gdev->ev_map->map;
		nums = gdev->ev_map->map_sz;
		for (i = 0; i < nums; i++) {
			if (m[i].pin == eid) {
				*mapped = (m[i].slice << 5) + m[i].bit_index;
				return;
			}
		}
	}
	*mapped = eid;
}

static struct tegra_gte_ev_desc *__gte_register_event(u32 eid, u32 gid,
						struct tegra_gte_dev *gte_dev,
						bool is_aon_gte)
{
	u32 slice, sl_bit_shift, ev_bit, offset, val, reg;
	int ret, sysfs_created;
	struct tegra_gte_event_info *ev;

	sl_bit_shift = __builtin_ctz(GTE_SLICE_SIZE);

	tegra_gte_map_to_ev_id(eid, gte_dev, &offset);

	if (offset > gte_dev->num_events) {
		dev_err(gte_dev->pdev,
			"Invalid event id:%u and GTE event offset: %u",
			 eid, offset);
		return ERR_PTR(-EINVAL);
	}

	ev = gte_dev->ev;
	if (test_bit(GTE_EVENT_UNREGISTERING, &ev[offset].flags)) {
		dev_err(gte_dev->pdev, "Event:%d is getting unregistered",
			offset);
		return ERR_PTR(-EUSERS);
	}

	/*
	 * There a chance that both kernel client driver and userspace calling
	 * register API with same event id, lock here.
	 */
	mutex_lock(&ev[offset].ev_lock);

	if (test_bit(GTE_EVENT_REGISTERED, &ev[offset].flags)) {
		dev_err(gte_dev->pdev, "Event:%d is already registered",
			offset);
		ret = -EUSERS;
		goto error_unlock;
	}

	slice = offset >> 5;
	ev_bit = offset & (32 - 1);

	if (kfifo_alloc(&ev[offset].ev_fifo,
	    GTE_EV_FIFO_EL * sizeof(struct tegra_gte_ev_el), GFP_KERNEL)) {
		dev_err(gte_dev->pdev, "Fifo allocation failed");
		ret = -ENOMEM;
		goto error_unlock;
	}

	reg = (slice << sl_bit_shift) + GTE_SLICE0_TETEN;

	if (is_aon_gte) {
		ret = gpio_timestamp_control(gid, 1);
		if (ret) {
			dev_err(gte_dev->pdev,
				"failed to set timestamp control %d",
				gid);
			ret = -EOPNOTSUPP;
			goto error_fifo_free;
		}
		ev[offset].pv.gid = gid;
	} else {
		ev[offset].pv.gid = -EINVAL;
	}

	ev[offset].reg = reg;
	ev[offset].pv.ev_bit = ev_bit;
	ev[offset].pv.slice = slice;
	ev[offset].pv.id = offset;

	ret = kobject_init_and_add(&ev[offset].kobj, &gte_ev_kobj_type,
				   gte_dev->kobj, "event%d", offset);
	if (!ret) {
		ret = sysfs_create_group(&ev[offset].kobj,
					 &event_attr_group);
		if (ret) {
			kobject_put(&ev[offset].kobj);
			memset(&ev[offset].kobj, 0, sizeof(ev[offset].kobj));
		} else {
			sysfs_created = 1;
		}
	} else {
		kobject_put(&ev[offset].kobj);
	}

	spin_lock(&gte_dev->sl[slice].s_lock);
	if (test_bit(GTE_SUSPEND, &gte_dev->sl[slice].flags)) {
		spin_unlock(&gte_dev->sl[slice].s_lock);
		if (sysfs_created == 1) {
			kobject_put(&ev[offset].kobj);
			memset(&ev[offset].kobj, 0, sizeof(ev[offset].kobj));
		}
		dev_dbg(gte_dev->pdev, "device suspended");
		ret = -EBUSY;
		goto error_fifo_free;
	}

	val = tegra_gte_readl(gte_dev, reg);
	val = val | (1 << ev_bit);
	tegra_gte_writel(gte_dev, reg, val);
	spin_unlock(&gte_dev->sl[slice].s_lock);

	atomic_inc(&gte_dev->usage);
	set_bit(GTE_EVENT_REGISTERED, &ev[offset].flags);
	dev_dbg(gte_dev->pdev,
		"%s: slice: %u, bit: %u, offset: %d,  g_id: %d reg = 0x%x",
		__func__, slice, ev_bit, offset, gid, reg);
	mutex_unlock(&ev[offset].ev_lock);

	return &ev[offset].pv;

error_fifo_free:
	kfifo_free(&ev[offset].ev_fifo);
error_unlock:
	mutex_unlock(&ev[offset].ev_lock);
	return ERR_PTR(ret);
}

/*
 * registers the event with GTE, allocates FIFO, creates sysfs for the event
 * stats. Also asks GPIO driver to enable timestamp bit in case of the GPIO
 * event to monitor.
 */
struct tegra_gte_ev_desc *tegra_gte_register_event(struct device_node *np,
						   u32 ev_id)
{
	struct tegra_gte_dev *gte_dev = NULL;
	int offset;
	bool aon_gte = false;

	if (!np) {
		pr_err("Node empty\n");
		return ERR_PTR(-EINVAL);
	}
	list_for_each_entry(gte_dev, &gte_devices, list) {
		if (gte_dev->pdev->of_node == np) {
			dev_dbg(gte_dev->pdev, "found the match\n");
			break;
		}
	}
	if (!gte_dev) {
		pr_err("No device found\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	if (of_device_is_compatible(np, "nvidia,tegra194-gte-aon")) {
		dev_dbg(gte_dev->pdev, "gpio gte is requested\n");
		offset = (int)(ev_id - gte_dev->gpio_base);
		if (offset < 0) {
			dev_err(gte_dev->pdev,
				"gpio pin: %u from non-supported controller\n",
				ev_id);
			return ERR_PTR(-EINVAL);
		}
		aon_gte = true;
	} else {
		offset = ev_id;
	}

	return __gte_register_event(offset, ev_id, gte_dev, aon_gte);
}
EXPORT_SYMBOL(tegra_gte_register_event);

int tegra_gte_retrieve_event(const struct tegra_gte_ev_desc *data,
			     struct tegra_gte_ev_detail *hts)
{
	int ret, ev_id;
	unsigned long flags;
	struct tegra_gte_ev_el el;
	const struct tegra_gte_ev_desc *pri;
	struct tegra_gte_event_info *ev;
	struct tegra_gte_dev *gte_dev;

	if (!data || !hts)
		return -EINVAL;

	pri = data;
	ev = container_of(pri, struct tegra_gte_event_info, pv);
	ev_id = pri->id;
	gte_dev = ev->dev;

	spin_lock_irqsave(&ev->lock, flags);

	if (!test_bit(GTE_EVENT_REGISTERED, &ev->flags)) {
		dev_dbg(gte_dev->pdev, "Event: %d is not registered", ev_id);
		ret = -EINVAL;
		goto unlock;
	}

	if (kfifo_is_empty(&ev->ev_fifo)) {
		dev_dbg(gte_dev->pdev, "Event: %d fifo is empty", ev_id);
		ret = -EAGAIN;
		goto unlock;
	}

	ret = kfifo_out(&ev->ev_fifo, (unsigned char *)&el, sizeof(el));
	if (unlikely(ret != sizeof(el))) {
		dev_dbg(gte_dev->pdev,
			"Event: %d retrieved element is in improper size",
			ev_id);
		ret = -EINVAL;
		goto unlock;
	}

	hts->dir = el.dir;
	hts->ts_raw = el.tsc;
	hts->ts_ns = el.tsc << GTE_TS_NS_SHIFT;
	atomic_dec(&ev->usage);
	ret = 0;

unlock:
	spin_unlock_irqrestore(&ev->lock, flags);
	return ret;
}
EXPORT_SYMBOL(tegra_gte_retrieve_event);

/*
 * GPIO event monitoring from userspace management. Only GPIO type is supported
 * to be monitored and timestamp using GTE from the userspace.
 */
struct gte_uspace_event_state {
	struct tegra_gte_dev *gdev;
	u32 eflags;
	u32 gpio_in;
	int irq;
	char *irqname;
	char *label;
	wait_queue_head_t wait;
	struct mutex read_lock;
	DECLARE_KFIFO(events, struct tegra_gte_hts_event_data, GTE_EV_FIFO_EL);
	struct tegra_gte_ev_desc *gte_data;
};

static unsigned int gte_event_poll(struct file *filep,
				   struct poll_table_struct *wait)
{
	struct gte_uspace_event_state *le = filep->private_data;
	unsigned int events = 0;

	poll_wait(filep, &le->wait, wait);

	if (!kfifo_is_empty(&le->events))
		events = POLLIN | POLLRDNORM;

	return events;
}


static ssize_t gte_event_read(struct file *filep, char __user *buf,
			      size_t count,
			      loff_t *f_ps)
{
	struct gte_uspace_event_state *le = filep->private_data;
	unsigned int copied;
	int ret;

	if (count < sizeof(struct tegra_gte_hts_event_data))
		return -EINVAL;

	do {
		if (kfifo_is_empty(&le->events)) {
			if (filep->f_flags & O_NONBLOCK)
				return -EAGAIN;

			ret = wait_event_interruptible(le->wait,
					!kfifo_is_empty(&le->events));
			if (ret)
				return ret;
		}

		if (mutex_lock_interruptible(&le->read_lock))
			return -ERESTARTSYS;
		ret = kfifo_to_user(&le->events, buf, count, &copied);
		mutex_unlock(&le->read_lock);

		if (ret)
			return ret;

		/*
		 * If we couldn't read anything from the fifo (a different
		 * thread might have been faster) we either return -EAGAIN if
		 * the file descriptor is non-blocking, otherwise we go back to
		 * sleep and wait for more data to arrive.
		 */
		if (copied == 0 && (filep->f_flags & O_NONBLOCK))
			return -EAGAIN;

	} while (copied == 0);

	return copied;
}

static int gte_event_release(struct inode *inode, struct file *filep)
{
	struct gte_uspace_event_state *le = filep->private_data;
	struct tegra_gte_dev *gdev = le->gdev;

	tegra_gte_unregister_event(le->gte_data);
	free_irq(le->irq, le);
	gpio_free(le->gpio_in);
	kfree(le->irqname);
	kfree(le->label);
	kfree(le);
	put_device(&gdev->c_dev);
	return 0;
}

static const struct file_operations gte_event_fileops = {
	.release = gte_event_release,
	.read = gte_event_read,
	.poll = gte_event_poll,
	.owner = THIS_MODULE,
	.llseek = noop_llseek,
};

static irqreturn_t gte_event_irq_thread(int irq, void *p)
{
	int ret;
	struct gte_uspace_event_state *le = p;
	struct tegra_gte_hts_event_data ge;
	struct tegra_gte_ev_detail hw;

	memset(&ge, 0, sizeof(ge));
	ret = tegra_gte_retrieve_event(le->gte_data, &hw);
	if (ret == 0) {
		ge.timestamp = hw.ts_ns;
		ge.dir = hw.dir;
	} else {
		dev_dbg(le->gdev->pdev, "failed to retrieve event\n");
		return IRQ_HANDLED;
	}

	ret = kfifo_put(&le->events, ge);
	if (ret != 0)
		wake_up_poll(&le->wait, POLLIN);

	return IRQ_HANDLED;
}

#define GPIOEVENT_REQUEST_VALID_FLAGS (TEGRA_GTE_EVENT_RISING_EDGE | \
				       TEGRA_GTE_EVENT_FALLING_EDGE)

static int gte_event_create(struct tegra_gte_dev *gdev, void __user *ip)
{
	struct tegra_gte_hts_event_req eventreq;
	struct gte_uspace_event_state *le;
	struct file *file;
	int offset;
	u32 eflags;
	int fd;
	int ret;
	int irqflags = 0;

	if (copy_from_user(&eventreq, ip, sizeof(eventreq)))
		return -EFAULT;

	if (!gdev->mp) {
		dev_err(gdev->pdev, "no controller node\n");
		return -EINVAL;
	}

	offset = (int)(eventreq.global_gpio_pin - gdev->gpio_base);
	if (offset < 0) {
		dev_err(gdev->pdev,
			"gpio pin: %u from non-supported controller\n",
			eventreq.global_gpio_pin);
		return -EINVAL;
	}

	eflags = eventreq.eventflags;
	/* Return an error if a unknown flag is set */
	if (eflags & ~GPIOEVENT_REQUEST_VALID_FLAGS) {
		dev_err(gdev->pdev, "Invalid event flags\n");
		return -EINVAL;
	}

	le = kzalloc(sizeof(*le), GFP_KERNEL);
	if (!le)
		return -ENOMEM;

	get_device(&gdev->c_dev);

	le->gdev = gdev;

	le->label = kzalloc(GTE_MAX_EV_NAME_SZ, GFP_KERNEL);
	if (!le->label) {
		ret = -ENOMEM;
		goto out_free_le;
	}
	scnprintf(le->label, GTE_MAX_EV_NAME_SZ, "gevent%d", offset);

	ret = gpio_request(eventreq.global_gpio_pin, le->label);
	if (ret) {
		dev_err(gdev->pdev, "failed to request gpio\n");
		ret = -EINVAL;
		goto out_free_label;
	}

	ret = gpio_direction_input(eventreq.global_gpio_pin);
	if (ret) {
		dev_err(gdev->pdev, "failed to set pin direction\n");
		ret = -EINVAL;
		goto out_free_label;
	}

	/* IRQ setup */
	ret = gpio_to_irq(eventreq.global_gpio_pin);
	if (ret < 0) {
		dev_err(gdev->pdev, "failed to map GPIO to IRQ: %d\n", ret);
		ret = -EINVAL;
		goto out_free_label;
	}
	le->irq = ret;

	if (eflags & TEGRA_GTE_EVENT_RISING_EDGE)
		irqflags |= IRQF_TRIGGER_RISING;
	if (eflags & TEGRA_GTE_EVENT_FALLING_EDGE)
		irqflags |= IRQF_TRIGGER_FALLING;
	irqflags |= IRQF_ONESHOT;
	irqflags |= IRQF_SHARED;

	le->irqname = kzalloc(GTE_MAX_EV_NAME_SZ, GFP_KERNEL);
	if (!le->irqname) {
		ret = -ENOMEM;
		goto out_free_label;
	}

	scnprintf(le->irqname, GTE_MAX_EV_NAME_SZ, "event%d", offset);

	/* Request a thread to read the events */
	ret = request_threaded_irq(le->irq, NULL, gte_event_irq_thread,
				   irqflags, le->irqname, le);
	if (ret) {
		dev_err(gdev->pdev, "failed request irq\n");
		goto out_free_name;
	}

	le->gpio_in = eventreq.global_gpio_pin;
	INIT_KFIFO(le->events);
	init_waitqueue_head(&le->wait);
	mutex_init(&le->read_lock);

	fd = get_unused_fd_flags(O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		ret = fd;
		goto out_free_irq;
	}

	file = anon_inode_getfile("gte-gpio-event", &gte_event_fileops, le,
				  O_RDONLY | O_CLOEXEC);
	if (IS_ERR(file)) {
		ret = PTR_ERR(file);
		dev_err(gdev->pdev, "failed to create file\n");
		goto out_put_unused_fd;
	}

	eventreq.fd = fd;

	le->gte_data = __gte_register_event(offset, le->gpio_in, gdev, true);
	if (IS_ERR(le->gte_data)) {
		ret = PTR_ERR(le->gte_data);
		dev_err(gdev->pdev, "failed gte register event\n");
		goto out_put_file;
	}

	if (copy_to_user(ip, &eventreq, sizeof(eventreq))) {
		dev_err(gdev->pdev, "failed to copy user\n");
		ret = -EFAULT;
		goto out_put_file;
	}

	fd_install(fd, file);
	dev_dbg(gdev->pdev, "GPIO event has been created\n");

	return 0;

out_put_file:
	fput(file);
out_put_unused_fd:
	put_unused_fd(fd);
out_free_irq:
	free_irq(le->irq, le);
out_free_name:
	kfree(le->irqname);
out_free_label:
	kfree(le->label);
out_free_le:
	kfree(le);
	put_device(&gdev->c_dev);
	return ret;
}

static long gte_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct tegra_gte_dev *gdev = filp->private_data;
	void __user *ip = (void __user *)arg;

	if (!gdev)
		return -ENODEV;

	if (cmd == TEGRA_GTE_HTS_CREATE_GPIO_EV_IOCTL)
		return gte_event_create(gdev, ip);

	return -EINVAL;
}

#ifdef CONFIG_COMPAT
static long gte_ioctl_compat(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	return gte_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int gte_chrdev_open(struct inode *inode, struct file *filp)
{
	struct tegra_gte_dev *gdev = container_of(inode->i_cdev,
						  struct tegra_gte_dev,
						  chrdev);

	if (!gdev)
		return -ENODEV;
	get_device(&gdev->c_dev);
	filp->private_data = gdev;

	return nonseekable_open(inode, filp);
}

static int gte_chrdev_release(struct inode *inode, struct file *filp)
{
	struct tegra_gte_dev *gdev = container_of(inode->i_cdev,
						  struct tegra_gte_dev,
						  chrdev);

	put_device(&gdev->c_dev);
	return 0;
}

static const struct file_operations gte_fileops = {
	.release = gte_chrdev_release,
	.open = gte_chrdev_open,
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = gte_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gte_ioctl_compat,
#endif
};

static void tegra_gte_read_fifo(struct tegra_gte_dev *gte)
{
	u32 tsh, tsl, src, pv, cv, acv, slice, bit_index, ev_id;
	u64 tsc;
	int dir;
	unsigned long flags;
	struct tegra_gte_ev_el ts;
	struct tegra_gte_event_info *ev = gte->ev;

	while ((tegra_gte_readl(gte, GTE_TESTATUS) >>
		GTE_TESTATUS_OCCUPANCY_SHIFT) &
		GTE_TESTATUS_OCCUPANCY_MASK) {
		tsh = tegra_gte_readl(gte, GTE_TETSCH);
		tsl = tegra_gte_readl(gte, GTE_TETSCL);
		tsc = (((u64)tsh << 32) | tsl);

		src = tegra_gte_readl(gte, GTE_TESRC);
		slice = (src >> GTE_TESRC_SLICE_SHIFT) &
			    GTE_TESRC_SLICE_DEFAULT_MASK;

		pv = tegra_gte_readl(gte, GTE_TEPCV);
		cv = tegra_gte_readl(gte, GTE_TECCV);
		acv = pv ^ cv;

		while (acv) {
			bit_index = __builtin_ctz(acv);
			if ((pv >> bit_index) & BIT(0))
				dir = TEGRA_GTE_EVENT_RISING_EDGE;
			else
				dir = TEGRA_GTE_EVENT_FALLING_EDGE;

			ev_id = bit_index + (slice << 5);
			spin_lock_irqsave(&ev[ev_id].lock, flags);
			if (test_bit(GTE_EVENT_REGISTERED, &ev[ev_id].flags)) {
				ts.tsc = tsc;
				ts.dir = dir;
				dev_dbg(gte->pdev, "ISR for ev id:%d, ts:%llu",
					ev_id, tsc);
				if (kfifo_avail(&ev[ev_id].ev_fifo) >=
				    sizeof(ts)) {
					kfifo_in(&ev[ev_id].ev_fifo,
						 (unsigned char *)&ts,
						 sizeof(ts));
					atomic_inc(&ev[ev_id].usage);
				} else {
					atomic_inc(&ev[ev_id].dropped_evs);
				}
			}
			spin_unlock_irqrestore(&ev[ev_id].lock, flags);
			acv &= ~BIT(bit_index);
		}

		tegra_gte_writel(gte, GTE_TECMD, GTE_TECMD_CMD_POP);
	}
}

static irqreturn_t tegra_gte_isr(int irq, void *dev_id)
{
	struct tegra_gte_dev *gte = dev_id;

	tegra_gte_read_fifo(gte);

	return IRQ_HANDLED;
}

/* Device specific sysfs management */

/* Shows total numbers of events registered with the given device */
static ssize_t show_events_registered(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct tegra_gte_dev *gte_dev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&gte_dev->usage));
}

static DEVICE_ATTR(events_registered, 0444, show_events_registered, NULL);

static struct attribute *gte_dev_attrs[] = {
	&dev_attr_events_registered.attr,
	NULL,
};

static struct attribute_group gte_def_attr_group = {
	.attrs = gte_dev_attrs,
};

static int tegra_gte_sysfs_create(struct platform_device *pdev)
{
	return sysfs_create_group(&pdev->dev.kobj, &gte_def_attr_group);
}

static int tegra_gte_chardv_create(struct tegra_gte_dev *gdev)
{
	int ret;

	ret  = bus_register(&gte_bus_type);
	if (ret < 0) {
		dev_err(gdev->pdev, "could not register gte bus type\n");
		return ret;
	}

	ret = alloc_chrdev_region(&gte_devt, 0, GTE_DEV_MAX, "gtechip");
	if (ret < 0) {
		dev_err(gdev->pdev, "failed to allocate char dev region\n");
		bus_unregister(&gte_bus_type);
		return ret;
	}

	gdev->c_dev.bus = &gte_bus_type;
	gdev->id = ida_simple_get(&gte_ida, 0, 0, GFP_KERNEL);

	dev_set_name(&gdev->c_dev, "gtechip%d", gdev->id);
	device_initialize(&gdev->c_dev);
	dev_set_drvdata(&gdev->c_dev, gdev);

	cdev_init(&gdev->chrdev, &gte_fileops);
	gdev->chrdev.owner = THIS_MODULE;
	gdev->chrdev.kobj.parent = &gdev->c_dev.kobj;
	gdev->c_dev.devt = MKDEV(MAJOR(gte_devt), gdev->id);
	ret = cdev_add(&gdev->chrdev, gdev->c_dev.devt, 1);
	if (ret < 0)
		dev_warn(gdev->pdev, "failed to add char device %d:%d\n",
			  MAJOR(gte_devt), gdev->id);
	else
		dev_dbg(gdev->pdev, "added GTE chardev (%d:%d)\n",
			MAJOR(gte_devt), gdev->id);
	ret = device_add(&gdev->c_dev);
	if (ret) {
		dev_err(gdev->pdev, "failed to add char dev:%d\n", ret);
		goto err_remove_chardev;
	}

	return 0;

err_remove_chardev:
	ida_simple_remove(&gte_ida, gdev->id);
	cdev_del(&gdev->chrdev);
	return ret;
}

static const struct of_device_id tegra_gte_of_match[] = {
	{ .compatible = "nvidia,tegra194-gte-lic"},
	{ .compatible = "nvidia,tegra194-gte-aon", .data = &aon_gte_map},
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_gte_of_match);

static void tegra_gte_add_to_list(struct tegra_gte_dev *gdev)
{
	mutex_lock(&gte_list_lock);
	list_add_tail(&gdev->list, &gte_devices);
	mutex_unlock(&gte_list_lock);
}

static void tegra_gte_init_and_enable(struct tegra_gte_dev *gte)
{
	u32 val = 0, slices;
	int i;
	struct tegra_gte_event_info *ev = gte->ev;

	slices = gte->num_events >> 5;
	atomic_set(&gte->usage, 0);

	for (i = 0; i < slices; i++) {
		gte->sl[i].flags = 0;
		spin_lock_init(&gte->sl[i].s_lock);
	}
	for (i = 0; i < gte->num_events; i++) {
		ev[i].flags = 0;
		ev[i].dev = gte;
		atomic_set(&ev[i].usage, 0);
		atomic_set(&ev[i].dropped_evs, 0);
		spin_lock_init(&ev[i].lock);
		mutex_init(&ev[i].ev_lock);
	}

	val = GTE_TECTRL_ENABLE_ENABLE |
	      (GTE_TECTRL_INTR_ENABLE << GTE_TECTRL_INTR_SHIFT) |
	      (gte->itr_thrshld << GTE_TECTRL_OCCU_SHIFT);
	tegra_gte_writel(gte, GTE_TECTRL, val);

	tegra_gte_add_to_list(gte);
}

static int tegra_gte_probe(struct platform_device *pdev)
{
	int ret;
	u32 slices;
	phandle mp;
	struct resource *res;
	struct device *dev;
	struct tegra_gte_dev *gte_dev;
	struct gpio_chip *c;

	dev = &pdev->dev;

	gte_dev = devm_kzalloc(dev, sizeof(*gte_dev), GFP_KERNEL);
	if (!gte_dev)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, gte_dev);
	gte_dev->pdev = dev;
	gte_dev->ev_map = of_device_get_match_data(&pdev->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no mem resource\n");
		return -EINVAL;
	}

	gte_dev->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(gte_dev->regs))
		return PTR_ERR(gte_dev->regs);

	ret = of_property_read_u32(dev->of_node, "nvidia,int-threshold",
				   &gte_dev->itr_thrshld);
	if (ret != 0)
		gte_dev->itr_thrshld = 1;

	ret = of_property_read_u32(dev->of_node, "nvidia,num-slices", &slices);
	if (ret != 0) {
		dev_err(dev, "Could not read slices\n");
		return -EINVAL;
	}

	gte_dev->num_events = slices << 5;

	gte_dev->sl = devm_kzalloc(dev, sizeof(struct gte_slices) * slices,
				   GFP_KERNEL);
	if (!gte_dev->sl)
		return -ENOMEM;

	gte_dev->ev = devm_kzalloc(dev, sizeof(struct tegra_gte_event_info) *
				   gte_dev->num_events, GFP_KERNEL);
	if (!gte_dev->ev)
		return -ENOMEM;

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "get irq failed.\n");
		return ret;
	}
	gte_dev->gte_irq = ret;
	ret = devm_request_irq(dev, gte_dev->gte_irq, tegra_gte_isr, 0,
			       dev_name(dev), gte_dev);
	if (ret < 0) {
		dev_err(dev, "request irq failed.\n");
		return ret;
	}
	gte_dev->kobj = &dev->kobj;

	gte_dev->gpio_base = -1;
	if (of_device_is_compatible(dev->of_node, "nvidia,tegra194-gte-aon")) {
		tegra_gte_chardv_create(gte_dev);
		ret = of_property_read_u32(dev->of_node,
					  "nvidia,gpio-controller", &mp);
		if (ret != 0) {
			dev_err(gte_dev->pdev, "no gpio controller phandle\n");
			return -EINVAL;
		} else {
			gte_dev->mp = of_find_node_by_phandle(mp);
			if (!gte_dev->mp) {
				dev_err(gte_dev->pdev, "no controller node\n");
				return -ENOSYS;
			}
			of_node_put(gte_dev->mp);
			c = of_get_chip_from_node(gte_dev->mp);
			if (c) {
				dev_dbg(gte_dev->pdev, "controller base:%d\n",
					c->base);
				gte_dev->gpio_base = c->base;
				tegra_gpio_enable_external_gte(c);
			} else {
				dev_err(gte_dev->pdev,
					"can not find gpio controller\n");
				return -EPROBE_DEFER;
			}
		}
	}

	ret = tegra_gte_sysfs_create(pdev);
	if (ret)
		dev_err(dev, "sysfs creation failed\n");

	tegra_gte_init_and_enable(gte_dev);

	dev_dbg(gte_dev->pdev, "events: %d, slices:%d",
		gte_dev->num_events, slices);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_gte_resume_early(struct device *dev)
{
	int i;
	struct tegra_gte_dev *gte_dev = dev_get_drvdata(dev);
	u32 slices = gte_dev->num_events >> 5;
	u32 sl_bit_shift = __builtin_ctz(GTE_SLICE_SIZE);

	tegra_gte_writel(gte_dev, GTE_TECTRL, gte_dev->conf_rval);

	for (i = 0; i < slices; i++) {
		spin_lock(&gte_dev->sl[i].s_lock);
		tegra_gte_writel(gte_dev,
				 ((i << sl_bit_shift) + GTE_SLICE0_TETEN),
				 gte_dev->sl[i].r_val);
		clear_bit(GTE_SUSPEND, &gte_dev->sl[i].flags);
		spin_unlock(&gte_dev->sl[i].s_lock);
	}

	return 0;
}

static int tegra_gte_suspend_late(struct device *dev)
{
	int i;
	struct tegra_gte_dev *gte_dev = dev_get_drvdata(dev);
	u32 slices = gte_dev->num_events >> 5;
	u32 sl_bit_shift = __builtin_ctz(GTE_SLICE_SIZE);

	gte_dev->conf_rval = tegra_gte_readl(gte_dev, GTE_TECTRL);
	for (i = 0; i < slices; i++) {
		spin_lock(&gte_dev->sl[i].s_lock);
		gte_dev->sl[i].r_val = tegra_gte_readl(gte_dev,
				((i << sl_bit_shift) + GTE_SLICE0_TETEN));
		set_bit(GTE_SUSPEND, &gte_dev->sl[i].flags);
		spin_unlock(&gte_dev->sl[i].s_lock);
	}

	return 0;
}
#endif

static const struct dev_pm_ops tegra_gte_pm = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(tegra_gte_suspend_late,
				     tegra_gte_resume_early)
};

static struct platform_driver tegra_gte_driver = {
	.probe = tegra_gte_probe,
	.driver = {
		.name = "tegra_gte",
		.pm = &tegra_gte_pm,
		.of_match_table = tegra_gte_of_match,
	},
};

module_platform_driver(tegra_gte_driver);

MODULE_AUTHOR("Dipen Patel <dipenp@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra GTE (Generic Timestamping Engine) driver");
MODULE_LICENSE("GPL v2");
