/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/delay.h>

#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>

#include <soc/tegra/chip-id.h>
#include <soc/tegra/virt/tegra_hv_pm_ctl.h>
#include <soc/tegra/virt/syscalls.h>
#include <soc/tegra/virt/tegra_hv_sysmgr.h>


#define DRV_NAME	"tegra_hv_pm_ctl"
#define CHAR_DEV_COUNT	1
#define MAX_GUESTS_NUM	8

struct tegra_hv_pm_ctl {
	struct device *dev;

	u32 ivc;
	struct tegra_hv_ivc_cookie *ivck;

	struct class *class;
	struct cdev cdev;
	dev_t char_devt;
	bool char_is_open;
	u32 wait_for_guests[MAX_GUESTS_NUM];
	u32 wait_for_guests_size;

	struct mutex mutex_lock;
	wait_queue_head_t wq;
};

int (*tegra_hv_pm_ctl_prepare_shutdown)(void);

/* Global driver data */
static struct tegra_hv_pm_ctl *tegra_hv_pm_ctl_data;

/* Guest ID for state */
static u32 guest_id;

static int tegra_hv_pm_ctl_get_guest_state(u32 vmid, u32 *state);

/*
 * For dependency management on System suspend, if there are guests required
 * to wait and the guests are active, the privileged guest sends
 * a guest suspend command to the guests and waits for the guests to be
 * suspended or shutsdown. Shutdown is acceptable as the key purpose
 * of this function is to ensure this VM stays up while VMs dependent on
 * this VM are up
 */
static int do_wait_for_guests_inactive(void)
{
	bool sent_guest_suspend = false;
	int i = 0;
	int ret = 0;

	while (i < tegra_hv_pm_ctl_data->wait_for_guests_size) {
		u32 vmid = tegra_hv_pm_ctl_data->wait_for_guests[i];
		u32 state;

		ret = tegra_hv_pm_ctl_get_guest_state(vmid, &state);
		if (ret < 0)
			return ret;

		if (state == VM_STATE_SUSPEND || state == VM_STATE_SHUTDOWN) {
			sent_guest_suspend = false;
			i++;
			continue;
		}

		if (sent_guest_suspend == false) {
			pr_debug("%s: Send a guest suspend command to guest%u\n",
				__func__, vmid);
			ret = tegra_hv_pm_ctl_trigger_guest_suspend(vmid);
			if (ret < 0)
				return ret;

			sent_guest_suspend = true;
		}
		msleep(10);
	}

	return 0;
}

int tegra_hv_pm_ctl_trigger_sys_suspend(void)
{
	int ret;

	if (!tegra_hv_pm_ctl_data) {
		pr_err("%s: tegra_hv_pm_ctl driver is not probed, %d\n",
			__func__, -ENXIO);
		return -ENXIO;
	}

	ret = do_wait_for_guests_inactive();
	if (ret < 0) {
		pr_err("%s: Failed to wait for guests suspended, %d\n",
			__func__, ret);
		return ret;
	}

	ret = hyp_guest_reset(SYS_SUSPEND_INIT_CMD, NULL);
	if (ret < 0) {
		pr_err("%s: Failed to trigger system suspend, %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

int tegra_hv_pm_ctl_trigger_sys_shutdown(void)
{
	int ret;

	if (!tegra_hv_pm_ctl_data) {
		pr_err("%s: tegra_hv_pm_ctl driver is not probed, %d\n",
			__func__, -ENXIO);
		return -ENXIO;
	}

	if (tegra_hv_pm_ctl_prepare_shutdown) {
		ret = tegra_hv_pm_ctl_prepare_shutdown();
		if (ret < 0) {
			pr_err("%s: Failed to prepare shutdown, %d\n",
				__func__, ret);
			return ret;
		}
	}

	ret = hyp_guest_reset(SYS_SHUTDOWN_INIT_CMD, NULL);
	if (ret < 0) {
		pr_err("%s: Failed to trigger system shutdown, %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

int tegra_hv_pm_ctl_trigger_sys_reboot(void)
{
	int ret;

	if (!tegra_hv_pm_ctl_data) {
		pr_err("%s: tegra_hv_pm_ctl driver is not probed, %d\n",
			__func__, -ENXIO);
		return -ENXIO;
	}

	ret = hyp_guest_reset(SYS_REBOOT_INIT_CMD, NULL);
	if (ret < 0) {
		pr_err("%s: Failed to trigger system reboot, %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

int tegra_hv_pm_ctl_trigger_guest_suspend(u32 vmid)
{
	int ret;

	if (!tegra_hv_pm_ctl_data) {
		pr_err("%s: tegra_hv_pm_ctl driver is not probed, %d\n",
			__func__, -ENXIO);
		return -ENXIO;
	}

	ret = hyp_guest_reset(GUEST_SUSPEND_REQ_CMD(vmid), NULL);
	if (ret < 0) {
		pr_err("%s: Failed to trigger guest%u suspend, %d\n",
			__func__, vmid, ret);
		return ret;
	}

	return 0;
}

int tegra_hv_pm_ctl_trigger_guest_reboot(u32 vmid)
{
	int ret;

	if (!tegra_hv_pm_ctl_data) {
		pr_err("%s: tegra_hv_pm_ctl driver is not probed, %d\n",
			__func__, -ENXIO);
		return -ENXIO;
	}

	ret = hyp_guest_reset(GUEST_REBOOT_INIT_CMD(vmid), NULL);
	if (ret < 0) {
		pr_err("%s: Failed to trigger guest%u suspend, %d\n",
			__func__, vmid, ret);
		return ret;
	}

	return 0;
}

int tegra_hv_pm_ctl_trigger_guest_resume(u32 vmid)
{
	int ret;

	if (!tegra_hv_pm_ctl_data) {
		pr_err("%s: tegra_hv_pm_ctl driver is not probed, %d\n",
			__func__, -ENXIO);
		return -ENXIO;
	}

	ret = hyp_guest_reset(GUEST_RESUME_INIT_CMD(vmid), NULL);
	if (ret < 0) {
		pr_err("%s: Failed to trigger guest%u resume, %d\n",
			__func__, vmid, ret);
		return ret;
	}

	return 0;
}

static int tegra_hv_pm_ctl_get_guest_state(u32 vmid, u32 *state)
{
	int ret;

	if (!tegra_hv_pm_ctl_data) {
		pr_err("%s: tegra_hv_pm_ctl driver is not probed, %d\n",
			__func__, -ENXIO);
		return -ENXIO;
	}

	/* guest state which can be returned:
	 * VM_STATE_BOOT, VM_STATE_HALT, VM_STATE_SUSPEND, VM_STATE_SHUTDOWN */
	ret = hyp_read_guest_state(vmid, state);
	if (ret < 0) {
		pr_err("%s: Failed to get guest%u state, %d\n",
			__func__, vmid, ret);
		return ret;
	}

	return 0;
}

static irqreturn_t tegra_hv_pm_ctl_irq(int irq, void *dev_id)
{
	struct tegra_hv_pm_ctl *data = dev_id;
	struct ivc *ivc = tegra_hv_ivc_convert_cookie(data->ivck);

	/* handle IVC state changes */
	if (tegra_ivc_channel_notified(ivc) != 0)
		goto out;

	if (tegra_ivc_can_read(ivc) || tegra_ivc_can_write(ivc))
		wake_up_interruptible_all(&data->wq);

out:
	return IRQ_HANDLED;
}

static ssize_t tegra_hv_pm_ctl_read(struct file *filp, char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct tegra_hv_pm_ctl *data = filp->private_data;
	struct ivc *ivc = tegra_hv_ivc_convert_cookie(data->ivck);
	size_t chunk;
	int ret = 0;

	if (!tegra_ivc_can_read(ivc)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(data->wq,
					       tegra_ivc_can_read(ivc));
		if (ret < 0)
			return ret;
	}

	chunk = min_t(size_t, count, data->ivck->frame_size);
	mutex_lock(&data->mutex_lock);
	ret = tegra_ivc_read_user(ivc, buf, chunk);
	mutex_unlock(&data->mutex_lock);
	if (ret < 0)
		dev_err(data->dev, "%s: Failed to read data from IVC %d, %d\n",
			__func__, data->ivc, ret);

	return ret;
}

static ssize_t tegra_hv_pm_ctl_write(struct file *filp, const char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct tegra_hv_pm_ctl *data = filp->private_data;
	struct ivc *ivc = tegra_hv_ivc_convert_cookie(data->ivck);
	size_t chunk;
	int ret = 0;

	if (!tegra_ivc_can_write(ivc)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(data->wq,
					       tegra_ivc_can_write(ivc));
		if (ret < 0)
			return ret;
	}

	chunk = min_t(size_t, count, data->ivck->frame_size);
	mutex_lock(&data->mutex_lock);
	ret = tegra_ivc_write_user(ivc, buf, chunk);
	mutex_unlock(&data->mutex_lock);
	if (ret < 0)
		dev_err(data->dev, "%s: Failed to write data from IVC %d, %d\n",
			__func__, data->ivc, ret);

	return ret;
}

static unsigned int tegra_hv_pm_ctl_poll(struct file *filp,
					 struct poll_table_struct *table)
{
	struct tegra_hv_pm_ctl *data = filp->private_data;
	struct ivc *ivc = tegra_hv_ivc_convert_cookie(data->ivck);
	unsigned long req_events = poll_requested_events(table);
	unsigned int read_mask = POLLIN | POLLRDNORM;
	unsigned int write_mask = POLLOUT | POLLWRNORM;
	unsigned int mask = 0;

	mutex_lock(&data->mutex_lock);
	if (!tegra_ivc_can_read(ivc) && (req_events & read_mask)) {
		mutex_unlock(&data->mutex_lock);
		poll_wait(filp, &data->wq, table);
		mutex_lock(&data->mutex_lock);
	}

	if (tegra_ivc_can_read(ivc))
		mask |= read_mask;
	if (tegra_ivc_can_write(ivc))
		mask |= write_mask;
	mutex_unlock(&data->mutex_lock);

	return mask;
}

static int tegra_hv_pm_ctl_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct tegra_hv_pm_ctl *data =
			container_of(cdev, struct tegra_hv_pm_ctl, cdev);
	int ret = 0;

	mutex_lock(&data->mutex_lock);
	if (data->char_is_open) {
		ret = -EBUSY;
		goto out;
	}

	data->char_is_open = true;
	filp->private_data = data;
out:
	mutex_unlock(&data->mutex_lock);

	return ret;
}

static int tegra_hv_pm_ctl_release(struct inode *inode, struct file *filp)
{
	struct tegra_hv_pm_ctl *data = filp->private_data;

	mutex_lock(&data->mutex_lock);
	data->char_is_open = false;
	filp->private_data = NULL;
	mutex_unlock(&data->mutex_lock);

	return 0;
}

static const struct file_operations tegra_hv_pm_ctl_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= tegra_hv_pm_ctl_read,
	.write		= tegra_hv_pm_ctl_write,
	.poll		= tegra_hv_pm_ctl_poll,
	.open		= tegra_hv_pm_ctl_open,
	.release	= tegra_hv_pm_ctl_release,
};

static ssize_t ivc_id_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->ivc);
}

static ssize_t ivc_frame_size_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->ivck->frame_size);
}

static ssize_t ivc_nframes_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->ivck->nframes);
}

static ssize_t ivc_peer_vmid_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->ivck->peer_vmid);
}

static ssize_t trigger_sys_suspend_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		dev_err(data->dev, "%s: Failed to convert string to uint\n",
			__func__);
		return ret;
	}

	if (val != 1) {
		dev_err(data->dev, "%s: Unsupported value, %u\n",
			__func__, val);
		return -EINVAL;
	}

	ret = tegra_hv_pm_ctl_trigger_sys_suspend();
	if (ret < 0)
		return ret;

	return count;
}


static ssize_t trigger_sys_shutdown_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		dev_err(data->dev, "%s: Failed to convert string to uint\n",
			__func__);
		return ret;
	}

	if (val != 1) {
		dev_err(data->dev, "%s: Unsupported value, %u\n",
			__func__, val);
		return -EINVAL;
	}

	ret = tegra_hv_pm_ctl_trigger_sys_shutdown();
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t trigger_sys_reboot_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		dev_err(data->dev, "%s: Failed to convert string to uint\n",
			__func__);
		return ret;
	}

	if (val != 1) {
		dev_err(data->dev, "%s: Unsupported value, %u\n",
			__func__, val);
		return -EINVAL;
	}

	ret = tegra_hv_pm_ctl_trigger_sys_reboot();
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t trigger_guest_suspend_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		dev_err(data->dev, "%s: Failed to convert string to uint\n",
			__func__);
		return ret;
	}

	ret = tegra_hv_pm_ctl_trigger_guest_suspend(val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t trigger_guest_reboot_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		dev_err(data->dev, "%s: Failed to convert string to uint\n",
			__func__);
		return ret;
	}

	ret = tegra_hv_pm_ctl_trigger_guest_reboot(val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t trigger_guest_resume_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		dev_err(data->dev, "%s: Failed to convert string to uint\n",
			__func__);
		return ret;
	}

	ret = tegra_hv_pm_ctl_trigger_guest_resume(val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t guest_state_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	u32 state = VM_STATE_BOOT;
	u32 vmid;
	int ret;

	mutex_lock(&data->mutex_lock);
	vmid = guest_id;
	mutex_unlock(&data->mutex_lock);
	ret = tegra_hv_pm_ctl_get_guest_state(vmid, &state);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "guest%u: %u\n", vmid, state);
}

static ssize_t guest_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret) {
		dev_err(data->dev, "%s: Failed to convert string to uint\n",
			__func__);
		return ret;
	}

	mutex_lock(&data->mutex_lock);
	guest_id = val;
	mutex_unlock(&data->mutex_lock);

	return count;
}

static ssize_t wait_for_guests_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct tegra_hv_pm_ctl *data = dev_get_drvdata(dev);
	ssize_t count = 0;
	int i;

	for (i = 0; i < data->wait_for_guests_size; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count, "%u ",
				  data->wait_for_guests[i]);
	}
	count += snprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static DEVICE_ATTR_RO(ivc_id);
static DEVICE_ATTR_RO(ivc_frame_size);
static DEVICE_ATTR_RO(ivc_nframes);
static DEVICE_ATTR_RO(ivc_peer_vmid);
static DEVICE_ATTR_WO(trigger_sys_suspend);
static DEVICE_ATTR_WO(trigger_sys_shutdown);
static DEVICE_ATTR_WO(trigger_sys_reboot);
static DEVICE_ATTR_WO(trigger_guest_reboot);
static DEVICE_ATTR_WO(trigger_guest_suspend);
static DEVICE_ATTR_WO(trigger_guest_resume);
static DEVICE_ATTR_RW(guest_state);
static DEVICE_ATTR_RO(wait_for_guests);

static struct attribute *tegra_hv_pm_ctl_attributes[] = {
	&dev_attr_ivc_id.attr,
	&dev_attr_ivc_frame_size.attr,
	&dev_attr_ivc_nframes.attr,
	&dev_attr_ivc_peer_vmid.attr,
	&dev_attr_trigger_sys_suspend.attr,
	&dev_attr_trigger_sys_shutdown.attr,
	&dev_attr_trigger_sys_reboot.attr,
	&dev_attr_trigger_guest_reboot.attr,
	&dev_attr_trigger_guest_suspend.attr,
	&dev_attr_trigger_guest_resume.attr,
	&dev_attr_guest_state.attr,
	&dev_attr_wait_for_guests.attr,
	NULL
};

static const struct attribute_group tegra_hv_pm_ctl_attr_group = {
	.attrs = tegra_hv_pm_ctl_attributes,
};

static int tegra_hv_pm_ctl_setup(struct tegra_hv_pm_ctl *data)
{
	struct tegra_hv_ivc_cookie *ivck;
	struct device *chr_dev;
	int ret;

	ivck = tegra_hv_ivc_reserve(NULL, data->ivc, NULL);
	if (IS_ERR_OR_NULL(ivck)) {
		dev_err(data->dev, "%s: Failed to reserve IVC %d\n",
			__func__, data->ivc);
		data->ivck = NULL;
		return -ENODEV;
	}
	data->ivck = ivck;
	dev_dbg(data->dev,
		"%s: IVC %d: irq=%d, peer_vmid=%d, nframes=%d, frame_size=%d\n",
		__func__, data->ivc, ivck->irq, ivck->peer_vmid, ivck->nframes,
		ivck->frame_size);

	ret = devm_request_irq(data->dev, ivck->irq, tegra_hv_pm_ctl_irq,
			       0, dev_name(data->dev), data);
	if (ret < 0) {
		dev_err(data->dev, "%s: Failed to request irq %d, %d\n",
			__func__, ivck->irq, ret);
		goto error;
	}

	tegra_hv_ivc_channel_reset(ivck);

	ret = alloc_chrdev_region(&data->char_devt, 0, CHAR_DEV_COUNT,
				  DRV_NAME);
	if (ret < 0) {
		dev_err(data->dev, "%s: Failed to alloc chrdev region, %d\n",
			__func__, ret);
		goto error;
	}

	cdev_init(&data->cdev, &tegra_hv_pm_ctl_fops);
	data->cdev.owner = THIS_MODULE;
	ret = cdev_add(&data->cdev, data->char_devt, 1);
	if (ret) {
		dev_err(data->dev, "%s: Failed to add cdev, %d\n",
			__func__, ret);
		goto error_unregister_chrdev_region;
	}

	chr_dev = device_create(data->class, data->dev, data->char_devt,
				data, DRV_NAME);
	if (IS_ERR(chr_dev)) {
		dev_err(data->dev, "%s: Failed to create device, %ld\n",
			__func__, PTR_ERR(chr_dev));
		ret = PTR_ERR(chr_dev);
		goto error_cdev_del;
	}

	return 0;

error_cdev_del:
	cdev_del(&data->cdev);
error_unregister_chrdev_region:
	unregister_chrdev_region(MAJOR(data->char_devt), 1);
error:
	if (data->ivck)
		tegra_hv_ivc_unreserve(data->ivck);
	return ret;
}

static void tegra_hv_pm_ctl_cleanup(struct tegra_hv_pm_ctl *data)
{
	device_destroy(data->class, data->char_devt);
	cdev_del(&data->cdev);
	unregister_chrdev_region(MAJOR(data->char_devt), 1);
	if (data->ivck)
		tegra_hv_ivc_unreserve(data->ivck);
}

static int tegra_hv_pm_ctl_parse_dt(struct tegra_hv_pm_ctl *data)
{
	struct device_node *np = data->dev->of_node;
	int ret;

	if (!np) {
		dev_err(data->dev, "%s: Failed to find device node\n",
			__func__);
		return -EINVAL;
	}

	if (of_property_read_u32_index(np, "ivc", 1, &data->ivc)) {
		dev_err(data->dev, "%s: Failed to find ivc property in %s\n",
			__func__, np->full_name);
		return -EINVAL;
	}

	/* List of guests to wait before sending a System suspend command for
	 * dependency management. */
	ret = of_property_read_variable_u32_array(np, "wait-for-guests",
				data->wait_for_guests, 1, MAX_GUESTS_NUM);
	if (ret > 0)
		data->wait_for_guests_size = ret;

	return 0;
}

static int tegra_hv_pm_ctl_probe(struct platform_device *pdev)
{
	struct tegra_hv_pm_ctl *data;
	int ret;

	if (!is_tegra_hypervisor_mode()) {
		dev_err(&pdev->dev, "%s: Hypervisor is not present\n",
			__func__);
		return -ENODEV;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(struct tegra_hv_pm_ctl),
			    GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev,
			"%s: Failed to alloc memory for driver data\n",
			__func__);
		return -ENOMEM;
	}

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);
	mutex_init(&data->mutex_lock);
	init_waitqueue_head(&data->wq);

	ret = tegra_hv_pm_ctl_parse_dt(data);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: Failed to parse device tree, %d\n",
			__func__, ret);
		return ret;
	}

	data->class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(data->class)) {
		dev_err(data->dev, "%s: Failed to create class, %ld\n",
			__func__, PTR_ERR(data->class));
		return PTR_ERR(data->class);
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &tegra_hv_pm_ctl_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: Failed to create sysfs group, %d\n",
			__func__, ret);
		goto error_class_destroy;
	}

	ret = tegra_hv_pm_ctl_setup(data);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: Failed to setup device, %d\n",
			__func__, ret);
		goto error_sysfs_remove_group;
	}

	tegra_hv_pm_ctl_data = data;

	dev_info(&pdev->dev, "%s: Probed\n", __func__);

	return 0;

error_sysfs_remove_group:
	sysfs_remove_group(&pdev->dev.kobj, &tegra_hv_pm_ctl_attr_group);
error_class_destroy:
	class_destroy(data->class);

	return ret;
}

static int tegra_hv_pm_ctl_remove(struct platform_device *pdev)
{
	struct tegra_hv_pm_ctl *data = platform_get_drvdata(pdev);

	tegra_hv_pm_ctl_data = NULL;
	tegra_hv_pm_ctl_cleanup(data);
	sysfs_remove_group(&pdev->dev.kobj, &tegra_hv_pm_ctl_attr_group);
	class_destroy(data->class);

	return 0;
}

static const struct of_device_id tegra_hv_pm_ctl_match[] = {
	{ .compatible = "nvidia,tegra-hv-pm-ctl", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_hv_pm_ctl_match);

static struct platform_driver tegra_hv_pm_ctl_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_hv_pm_ctl_match),
	},
	.probe = tegra_hv_pm_ctl_probe,
	.remove = tegra_hv_pm_ctl_remove,
};
module_platform_driver(tegra_hv_pm_ctl_driver);

MODULE_DESCRIPTION("Nvidia hypervisor PM control driver");
MODULE_AUTHOR("Jinyoung Park <jinyoungp@nvidia.com>");
MODULE_LICENSE("GPL");
