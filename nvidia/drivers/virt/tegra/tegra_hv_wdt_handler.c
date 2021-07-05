/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/init.h>
#include <linux/of.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ioctl.h>

#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>

#include <soc/tegra/chip-id.h>
#include <uapi/linux/nvhv_wdt_handler_ioctl.h>

#define DRV_NAME	"tegra_hv_wdt_handler"

/* Time to sleep each time in ms, after checking channel ready */
#define DEFAULT_CH_WRITE_SLEEP_TIME_MS	100
/* Amount of time to wait for channel to become available for writing
 * before giving up.  Some time channel may be full, if monitor is not able
 * to read the previous command, ideally this should not happened, since
 * monitor reads command in interrupt immediately its sent. This can only
 * fail if Monitor is dead or has not been given low priority than Guest.
 * Which is a integration bug.
 */
#define DEFAULT_CH_WRITE_LOOP_CNT		20

/* Message from monitor with the guest id for which WDT expired */
struct tegra_hv_wdt_h_mon_msg {
	unsigned int vmid;
};

struct tegra_hv_wdt_h {
	uint32_t ivc_ch;
	struct tegra_hv_ivc_cookie *ivck;
	struct hv_wdt_h_state_array state_array;
	struct platform_device *pdev;
	struct class *class;
	struct cdev cdev;
	dev_t char_devt;
	bool char_is_open;
	wait_queue_head_t wq;
	struct mutex mutex_lock;
};

static const char * const guest_state_string[] = {
	"GUEST_STATE_INIT",
	"GUEST_STATE_TIMER_EXPIRED",
	"GUEST_STATE_WAIT_FOR_ACK",
};

static int hv_wdt_h_get_exp_guest_cnt(struct tegra_hv_wdt_h *hv)
{
	int i;
	int count = 0;

	mutex_lock(&hv->mutex_lock);
	for (i = 0; i < MAX_GUESTS_NUM; i++) {
		if (hv->state_array.guest_state[i] == GUEST_STATE_TIMER_EXPIRED)
			count++;
	}
	mutex_unlock(&hv->mutex_lock);
	return count;
}

static long hv_wdt_h_get_guest_state(struct tegra_hv_wdt_h *hv,
				bool non_blocking,
				struct hv_wdt_h_state_array __user *state_array)
{
	int i, ret = 0;

	/* If any of the VM is expired dont wait */
	if (!hv_wdt_h_get_exp_guest_cnt(hv)) {
		if (non_blocking)
			return -EAGAIN;

		ret = wait_event_interruptible(hv->wq,
						hv_wdt_h_get_exp_guest_cnt(hv));
		if (ret < 0)
			return ret;
	}
	/* Give VM data to Application */
	mutex_lock(&hv->mutex_lock);
	ret = copy_to_user(state_array, &hv->state_array,
						sizeof(*state_array));
	if (ret != 0) {
		dev_err(&hv->pdev->dev, "Failed to copy user data\n");
		goto out;
	}

	/* Mark all the expired states as sent to user */
	for (i = 0; i < MAX_GUESTS_NUM; i++) {
		if (hv->state_array.guest_state[i] == GUEST_STATE_TIMER_EXPIRED)
			hv->state_array.guest_state[i] =
						GUEST_STATE_WAIT_FOR_ACK;
	}
out:
	mutex_unlock(&hv->mutex_lock);
	return ret;
}

static long hv_wdt_h_send_guest_cmd(struct tegra_hv_wdt_h *hv,
				struct tegra_hv_wdt_h_cmd_array *cmds)
{
	int i, ret = 0, write_possible_loop_cnt = 0;

	ret = tegra_hv_ivc_channel_notified(hv->ivck);
	if (ret != 0) {
		dev_err(&hv->pdev->dev, "IVC channel not ready %d\n",
						hv->ivc_ch);
		return ret;
	}

	if (cmds->num_vmids > MAX_GUESTS_NUM) {
		dev_err(&hv->pdev->dev, "Number of VMIDs greater than Max\n");
		return -EINVAL;
	}

	while (tegra_hv_ivc_can_write(hv->ivck) == 0) {
		msleep(DEFAULT_CH_WRITE_SLEEP_TIME_MS);
		write_possible_loop_cnt++;
		if (write_possible_loop_cnt >= DEFAULT_CH_WRITE_LOOP_CNT) {
			dev_err(&hv->pdev->dev, "Failed to write data through IVC\n");
			return -EAGAIN;
		}
	}

	ret = tegra_hv_ivc_write(hv->ivck, cmds, sizeof(*cmds));
	if (ret != sizeof(*cmds)) {
		dev_err(&hv->pdev->dev, "Wrong write size from ivc write, required = %lu got = %d\n",
					sizeof(*cmds), ret);
		return -EINVAL;
	}

	mutex_lock(&hv->mutex_lock);
	for (i = 0; i < cmds->num_vmids; i++) {
		/* VMID number is greater than number of guests supported */
		if (cmds->commands[i].vmid > MAX_GUESTS_NUM)
			continue;
		if (hv->state_array.guest_state[cmds->commands[i].vmid] ==
					GUEST_STATE_WAIT_FOR_ACK)
			hv->state_array.guest_state[cmds->commands[i].vmid] =
							GUEST_STATE_INIT;
		else {
			dev_info(&hv->pdev->dev, "State not updated for %d guest\n",
					cmds->commands[i].vmid);
		}
	}
	mutex_unlock(&hv->mutex_lock);
	return 0;
}

static unsigned int hv_wdt_h_poll(struct file *filp,
					 struct poll_table_struct *table)
{
	struct tegra_hv_wdt_h *hv = filp->private_data;
	unsigned long req_events = poll_requested_events(table);
	unsigned int read_mask = POLLIN | POLLRDNORM;
	unsigned int mask = 0;

	if (req_events & read_mask)
		poll_wait(filp, &hv->wq, table);

	/* If any of the VMs has expired return mask */
	if (hv_wdt_h_get_exp_guest_cnt(hv))
		mask |= read_mask;

	return mask;
}

static int hv_wdt_h_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct tegra_hv_wdt_h *hv =
			container_of(cdev, struct tegra_hv_wdt_h, cdev);
	int ret = 0;

	/* We allow only single file handle to open */
	mutex_lock(&hv->mutex_lock);
	if (hv->char_is_open) {
		ret = -EBUSY;
		goto out;
	}

	hv->char_is_open = true;
	filp->private_data = hv;
	nonseekable_open(inode, filp);
out:
	mutex_unlock(&hv->mutex_lock);

	return ret;
}

static int hv_wdt_h_release(struct inode *inode, struct file *filp)
{
	struct tegra_hv_wdt_h *hv = filp->private_data;

	mutex_lock(&hv->mutex_lock);
	hv->char_is_open = false;
	filp->private_data = NULL;
	mutex_unlock(&hv->mutex_lock);

	return 0;
}

static long hv_wdt_h_ioctl(struct file *file,
				unsigned int cmd,
				unsigned long arg)
{
	struct tegra_hv_wdt_h *hv = file->private_data;
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct hv_wdt_h_state_array __user *state_array;
	struct tegra_hv_wdt_h_cmd_array cmds;

	switch (cmd) {
	case TEGRA_HV_WDT_H_GET_STATE:
		state_array = argp;
		ret = hv_wdt_h_get_guest_state(hv, (file->f_flags & O_NONBLOCK),
								state_array);
		break;
	case TEGRA_HV_WDT_H_CMD:
		if (copy_from_user(&cmds, argp, sizeof(cmds))) {
			ret = -EFAULT;
			goto out;
		}

		ret = hv_wdt_h_send_guest_cmd(hv, &cmds);
		break;
	default:
		ret = -EINVAL;
		break;
	}
out:
	return ret;
}

static const struct file_operations hv_wdt_h_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.poll		= hv_wdt_h_poll,
	.open		= hv_wdt_h_open,
	.release	= hv_wdt_h_release,
	.unlocked_ioctl = hv_wdt_h_ioctl,
};

static irqreturn_t hv_wdt_h_ivc_irq_thread(int irq, void *data)
{
	struct tegra_hv_wdt_h_mon_msg monitor_msg;
	struct tegra_hv_wdt_h *hv = (struct tegra_hv_wdt_h *)data;
	int len = 0;

	len = tegra_hv_ivc_read(hv->ivck, &monitor_msg, sizeof(monitor_msg));
	if (len != sizeof(monitor_msg)) {
		dev_err(&hv->pdev->dev, "Incorrect message size from IVC\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&hv->mutex_lock);
	hv->state_array.guest_state[monitor_msg.vmid] =
					GUEST_STATE_TIMER_EXPIRED;
	mutex_unlock(&hv->mutex_lock);
	wake_up_interruptible_all(&hv->wq);

	return IRQ_HANDLED;

}

static irqreturn_t hv_wdt_h_ivc_irq(int irq, void *data)
{
	struct tegra_hv_wdt_h *hv = (struct tegra_hv_wdt_h *)data;

	if (tegra_hv_ivc_channel_notified(hv->ivck) != 0)
		return IRQ_HANDLED;

	if (tegra_hv_ivc_can_read(hv->ivck))
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}

static void hv_wdt_h_cleanup(struct platform_device *pdev,
					struct tegra_hv_wdt_h *hv)
{
	/* Channel reset is required to make sure that other side is aware that
	 * peer is going down.
	 */
	tegra_hv_ivc_channel_reset(hv->ivck);
	tegra_hv_ivc_unreserve(hv->ivck);
}

static int hv_wdt_h_init(struct platform_device *pdev,
					struct tegra_hv_wdt_h *hv)
{
	struct device_node *dn;
	uint32_t id;
	int err = 0;

	init_waitqueue_head(&hv->wq);
	mutex_init(&hv->mutex_lock);
	hv->pdev = pdev;
	dn = pdev->dev.of_node;
	if (of_property_read_u32_index(dn, "ivc", 1, &id) != 0) {
		dev_err(&pdev->dev, "failed to find ivc property\n");
		return -EINVAL;
	}
	hv->ivc_ch = id;

	hv->ivck = tegra_hv_ivc_reserve(dn, hv->ivc_ch, NULL);
	if (IS_ERR_OR_NULL(hv->ivck)) {
		dev_err(&pdev->dev, "Failed to reserve ivc channel %d\n",
					hv->ivc_ch);
		return -EINVAL;
	}
	tegra_hv_ivc_channel_reset(hv->ivck);

	err = devm_request_threaded_irq(&pdev->dev, hv->ivck->irq,
				hv_wdt_h_ivc_irq,
				hv_wdt_h_ivc_irq_thread,
				0, "tegra_hv_vm_irq", hv);

	if (err) {
		hv_wdt_h_cleanup(pdev, hv);
		return err;
	}
	return 0;
}

static ssize_t guest_cmd_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct tegra_hv_wdt_h *hv = dev_get_drvdata(dev);
	char local_buf[128] = {0};
	char *b = local_buf;
	char *guest_token;
	char *cmd_token;
	int guest_id, cmd, err;
	struct tegra_hv_wdt_h_cmd_array cmd_array;

	memcpy(local_buf, buf, min_t(size_t, count, sizeof(local_buf) - 1));

	guest_token = strsep(&b, " ");
	if (!guest_token)
		goto wrong_cmd;

	cmd_token = strsep(&b, " ");
	if (!cmd_token)
		goto wrong_cmd;

	err = kstrtouint(guest_token, 10, &guest_id);

	if (err)  {
		dev_err(dev, "Not a number for guest\n");
		return -EINVAL;
	} else if (guest_id > (MAX_GUESTS_NUM - 1)) {
		dev_err(dev, "Guest number: %d > %d max supported guests\n",
						guest_id, MAX_GUESTS_NUM - 1);
		return -EINVAL;
	}

	if (!(strcmp(cmd_token, "MESSAGE_HANDLED_SYNC")))
		cmd = MESSAGE_HANDLED_SYNC;
	else if (!(strcmp(cmd_token, "MESSAGE_HANDLED_ASYNC")))
		cmd = MESSAGE_HANDLED_ASYNC;
	else
		goto wrong_cmd;

	cmd_array.num_vmids = 1;
	cmd_array.commands[0].command = cmd;
	cmd_array.commands[0].vmid = guest_id;
	err = hv_wdt_h_send_guest_cmd(hv, &cmd_array);
	if (err) {
		dev_err(dev, "Error in sending command %d\n", err);
		return err;
	}

	return count;

wrong_cmd:
	dev_err(dev, "cmd not supported, cmd: echo -n \"guest_id MESSAGE_HANDLED_SYNC | MESSAGE_HANDLED_ASYNC\"\n");
	return -EINVAL;
}

static ssize_t guest_state_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct tegra_hv_wdt_h *hv = dev_get_drvdata(dev);
	int i, count = 0, bytes;
	const char *st_str;

	mutex_lock(&hv->mutex_lock);
	for (i = 0; i < MAX_GUESTS_NUM; i++) {
		st_str = guest_state_string[hv->state_array.guest_state[i]];
		bytes = snprintf(buf + count, PAGE_SIZE - count,
						"GUEST%d\tstate:\t %s\n",
						i, st_str);
		count +=  bytes;
	}

	/* Mark all the expired states as sent to user */
	for (i = 0; i < MAX_GUESTS_NUM; i++) {
		if (hv->state_array.guest_state[i] == GUEST_STATE_TIMER_EXPIRED)
			hv->state_array.guest_state[i] =
						GUEST_STATE_WAIT_FOR_ACK;
	}
	mutex_unlock(&hv->mutex_lock);
	return count;
}

static DEVICE_ATTR_RO(guest_state);
static DEVICE_ATTR_WO(guest_cmd);

static struct attribute *hv_wdt_ctl_attrs[] = {
	&dev_attr_guest_state.attr,
	&dev_attr_guest_cmd.attr,
	NULL
};
ATTRIBUTE_GROUPS(hv_wdt_ctl);
static const struct attribute_group hv_wdt_ctl_attr_group = {
	.attrs = hv_wdt_ctl_attrs,
};

static struct class wdt_handler_class = {
	.name = DRV_NAME,
	.owner = THIS_MODULE,
	.dev_groups = hv_wdt_ctl_groups,
};

static void hv_wdt_h_chrdev_cleanup(struct  platform_device *pdev,
						struct tegra_hv_wdt_h *hv)
{
	device_destroy(hv->class, hv->char_devt);
	cdev_del(&hv->cdev);
	unregister_chrdev_region(MAJOR(hv->char_devt), 1);
	class_unregister(&wdt_handler_class);
	platform_set_drvdata(pdev, NULL);
}

static int hv_wdt_h_chrdev_init(struct platform_device *pdev,
					struct tegra_hv_wdt_h *hv)
{
	int err;
	struct device *chr_dev;

	platform_set_drvdata(pdev, hv);

	class_register(&wdt_handler_class);
	hv->class = &wdt_handler_class;

	err = alloc_chrdev_region(&hv->char_devt, 0, 1,
				  DRV_NAME);
	if (err < 0) {
		dev_err(&pdev->dev, "%s: Failed to alloc chrdev region, %d\n",
			__func__, err);
		goto error_unregister_class;
	}

	cdev_init(&hv->cdev, &hv_wdt_h_ops);

	hv->cdev.owner = THIS_MODULE;
	err = cdev_add(&hv->cdev, hv->char_devt, 1);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to add cdev, %d\n",
			__func__, err);
		goto error_unregister_chrdev_region;
	}

	chr_dev = device_create(hv->class, &hv->pdev->dev, hv->char_devt,
				hv, DRV_NAME);
	if (IS_ERR(chr_dev)) {
		dev_err(&pdev->dev, "%s: Failed to create device, %ld\n",
			__func__, PTR_ERR(chr_dev));
		err = PTR_ERR(chr_dev);
		goto error_cdev_del;
	}

	return 0;

error_cdev_del:
	cdev_del(&hv->cdev);
error_unregister_chrdev_region:
	unregister_chrdev_region(MAJOR(hv->char_devt), 1);
error_unregister_class:
	class_unregister(&wdt_handler_class);

	return err;

}

static int hv_wdt_h_probe(struct platform_device *pdev)
{
	struct tegra_hv_wdt_h *hv;
	int err = 0;

	if (!is_tegra_hypervisor_mode()) {
		dev_info(&pdev->dev, "hypervisor is not present\n");
		return -ENODEV;
	}

	hv = devm_kzalloc(&pdev->dev, sizeof(*hv), GFP_KERNEL);
	if (!hv)
		return -ENOMEM;

	err = hv_wdt_h_init(pdev, hv);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to enable guest wdt handler\n");
		return err;
	}

	err = hv_wdt_h_chrdev_init(pdev, hv);
	if (err != 0) {
		dev_err(&pdev->dev, "Fail to init character driver\n");
		hv_wdt_h_cleanup(pdev, hv);
		return err;
	}

	return 0;
}

static int hv_wdt_h_remove(struct platform_device *pdev)
{
	struct tegra_hv_wdt_h *hv = platform_get_drvdata(pdev);

	hv_wdt_h_chrdev_cleanup(pdev, hv);
	hv_wdt_h_cleanup(pdev, hv);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id tegra_wdt_h_match[] = {
	{ .compatible = "nvidia,tegra-hv-wdt-handler", },
	{}
};

static struct platform_driver tegra_wdt_h_driver = {
	.probe		= hv_wdt_h_probe,
	.remove		= hv_wdt_h_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= DRV_NAME,
		.of_match_table = of_match_ptr(tegra_wdt_h_match),
	},
};
module_platform_driver(tegra_wdt_h_driver);

MODULE_AUTHOR("Hardik Tushar Shah <hardikts@nvidia.com>");
MODULE_DESCRIPTION("Tegra Hypervisor VM WDT Expiry handler");
MODULE_LICENSE("GPL");

