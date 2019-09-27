/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <uapi/linux/nvgpu.h>

#include <nvgpu/bitops.h>
#include <nvgpu/lock.h>
#include <nvgpu/kmem.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/kref.h>
#include <nvgpu/log.h>
#include <nvgpu/barrier.h>
#include <nvgpu/cond.h>
#include <nvgpu/list.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/gk20a.h>

#include "clk/clk.h"
#include "pstate/pstate.h"
#include "lpwr/lpwr.h"
#include "volt/volt.h"

#ifdef CONFIG_DEBUG_FS
#include "os_linux.h"
#endif

static int nvgpu_clk_arb_release_completion_dev(struct inode *inode,
		struct file *filp)
{
	struct nvgpu_clk_dev *dev = filp->private_data;
	struct nvgpu_clk_session *session = dev->session;


	clk_arb_dbg(session->g, " ");

	/* This is done to account for the extra refcount taken in
	 * nvgpu_clk_arb_commit_request_fd without events support in iGPU
	 */
	if (!session->g->clk_arb->clk_arb_events_supported) {
		nvgpu_ref_put(&dev->refcount, nvgpu_clk_arb_free_fd);
	}

	nvgpu_ref_put(&session->refcount, nvgpu_clk_arb_free_session);
	nvgpu_ref_put(&dev->refcount, nvgpu_clk_arb_free_fd);
	return 0;
}

static inline unsigned int nvgpu_convert_poll_mask(unsigned int nvgpu_poll_mask)
{
	unsigned int poll_mask = 0;

	if (nvgpu_poll_mask & NVGPU_POLLIN)
		poll_mask |= POLLIN;
	if (nvgpu_poll_mask & NVGPU_POLLPRI)
		poll_mask |= POLLPRI;
	if (nvgpu_poll_mask & NVGPU_POLLOUT)
		poll_mask |= POLLOUT;
	if (nvgpu_poll_mask & NVGPU_POLLRDNORM)
		poll_mask |= POLLRDNORM;
	if (nvgpu_poll_mask & NVGPU_POLLHUP)
		poll_mask |= POLLHUP;

	return poll_mask;
}

static unsigned int nvgpu_clk_arb_poll_dev(struct file *filp, poll_table *wait)
{
	struct nvgpu_clk_dev *dev = filp->private_data;

	clk_arb_dbg(dev->session->g, " ");

	poll_wait(filp, &dev->readout_wq.wq, wait);
	return nvgpu_convert_poll_mask(nvgpu_atomic_xchg(&dev->poll_mask, 0));
}

void nvgpu_clk_arb_event_post_event(struct nvgpu_clk_dev *dev)
{
	nvgpu_cond_broadcast_interruptible(&dev->readout_wq);
}

static int nvgpu_clk_arb_release_event_dev(struct inode *inode,
		struct file *filp)
{
	struct nvgpu_clk_dev *dev = filp->private_data;
	struct nvgpu_clk_session *session = dev->session;
	struct nvgpu_clk_arb *arb;

	arb = session->g->clk_arb;

	clk_arb_dbg(session->g, " ");

	if (arb) {
		nvgpu_spinlock_acquire(&arb->users_lock);
		nvgpu_list_del(&dev->link);
		nvgpu_spinlock_release(&arb->users_lock);
		nvgpu_clk_notification_queue_free(arb->g, &dev->queue);
	}

	nvgpu_ref_put(&session->refcount, nvgpu_clk_arb_free_session);
	nvgpu_ref_put(&dev->refcount, nvgpu_clk_arb_free_fd);

	return 0;
}

static inline u32 nvgpu_convert_gpu_event(u32 nvgpu_event)
{
	u32 nvgpu_gpu_event;

	switch (nvgpu_event) {
	case NVGPU_EVENT_VF_UPDATE:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_VF_UPDATE;
		break;
	case NVGPU_EVENT_ALARM_TARGET_VF_NOT_POSSIBLE:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_TARGET_VF_NOT_POSSIBLE;
		break;
	case NVGPU_EVENT_ALARM_LOCAL_TARGET_VF_NOT_POSSIBLE:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_LOCAL_TARGET_VF_NOT_POSSIBLE;
		break;
	case NVGPU_EVENT_ALARM_CLOCK_ARBITER_FAILED:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_CLOCK_ARBITER_FAILED;
		break;
	case NVGPU_EVENT_ALARM_VF_TABLE_UPDATE_FAILED:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_VF_TABLE_UPDATE_FAILED;
		break;
	case NVGPU_EVENT_ALARM_THERMAL_ABOVE_THRESHOLD:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_THERMAL_ABOVE_THRESHOLD;
		break;
	case NVGPU_EVENT_ALARM_POWER_ABOVE_THRESHOLD:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_POWER_ABOVE_THRESHOLD;
		break;
	case NVGPU_EVENT_ALARM_GPU_LOST:
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_GPU_LOST;
		break;
		default:
		/* Control shouldn't come here */
		nvgpu_gpu_event = NVGPU_GPU_EVENT_ALARM_GPU_LOST + 1;
		break;
	}
	return nvgpu_gpu_event;
}

static inline u32 __pending_event(struct nvgpu_clk_dev *dev,
		struct nvgpu_gpu_event_info *info) {

	u32 tail, head;
	u32 events = 0;
	struct nvgpu_clk_notification *p_notif;

	tail = nvgpu_atomic_read(&dev->queue.tail);
	head = nvgpu_atomic_read(&dev->queue.head);

	head = (tail - head) < dev->queue.size ? head : tail - dev->queue.size;

	if (_WRAPGTEQ(tail, head) && info) {
		head++;
		p_notif = &dev->queue.notifications[head % dev->queue.size];
		events |= nvgpu_convert_gpu_event(p_notif->notification);
		info->event_id = ffs(events) - 1;
		info->timestamp = p_notif->timestamp;
		nvgpu_atomic_set(&dev->queue.head, head);
	}

	return events;
}

static ssize_t nvgpu_clk_arb_read_event_dev(struct file *filp, char __user *buf,
					size_t size, loff_t *off)
{
	struct nvgpu_clk_dev *dev = filp->private_data;
	struct nvgpu_gpu_event_info info;
	ssize_t err;

	clk_arb_dbg(dev->session->g,
			"filp=%p, buf=%p, size=%zu", filp, buf, size);

	if ((size - *off) < sizeof(info))
		return 0;

	memset(&info, 0, sizeof(info));
	/* Get the oldest event from the queue */
	while (!__pending_event(dev, &info)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		err = NVGPU_COND_WAIT_INTERRUPTIBLE(&dev->readout_wq,
				__pending_event(dev, &info), 0);
		if (err)
			return err;
		if (info.timestamp)
			break;
	}

	if (copy_to_user(buf + *off, &info, sizeof(info)))
		return -EFAULT;

	return sizeof(info);
}

static int nvgpu_clk_arb_set_event_filter(struct nvgpu_clk_dev *dev,
		struct nvgpu_gpu_set_event_filter_args *args)
{
	struct gk20a *g = dev->session->g;
	u32 mask;

	nvgpu_log(g, gpu_dbg_fn, " ");

	if (args->flags)
		return -EINVAL;

	if (args->size != 1)
		return -EINVAL;

	if (copy_from_user(&mask, (void __user *) args->buffer,
			args->size * sizeof(u32)))
		return -EFAULT;

	/* update alarm mask */
	nvgpu_atomic_set(&dev->enabled_mask, mask);

	return 0;
}

static long nvgpu_clk_arb_ioctl_event_dev(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct nvgpu_clk_dev *dev = filp->private_data;
	struct gk20a *g = dev->session->g;
	u8 buf[NVGPU_EVENT_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	nvgpu_log(g, gpu_dbg_fn, "nr=%d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_EVENT_IOCTL_MAGIC) || (_IOC_NR(cmd) == 0)
		|| (_IOC_NR(cmd) > NVGPU_EVENT_IOCTL_LAST))
		return -EINVAL;

	BUG_ON(_IOC_SIZE(cmd) > NVGPU_EVENT_IOCTL_MAX_ARG_SIZE);

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *) arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {
	case NVGPU_EVENT_IOCTL_SET_FILTER:
		err = nvgpu_clk_arb_set_event_filter(dev,
				(struct nvgpu_gpu_set_event_filter_args *)buf);
		break;
	default:
		nvgpu_warn(g, "unrecognized event ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *) arg, buf, _IOC_SIZE(cmd));

	return err;
}

static const struct file_operations completion_dev_ops = {
	.owner = THIS_MODULE,
	.release = nvgpu_clk_arb_release_completion_dev,
	.poll = nvgpu_clk_arb_poll_dev,
};

static const struct file_operations event_dev_ops = {
	.owner = THIS_MODULE,
	.release = nvgpu_clk_arb_release_event_dev,
	.poll = nvgpu_clk_arb_poll_dev,
	.read = nvgpu_clk_arb_read_event_dev,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvgpu_clk_arb_ioctl_event_dev,
#endif
	.unlocked_ioctl = nvgpu_clk_arb_ioctl_event_dev,
};

static int nvgpu_clk_arb_install_fd(struct gk20a *g,
		struct nvgpu_clk_session *session,
		const struct file_operations *fops,
		struct nvgpu_clk_dev **_dev)
{
	struct file *file;
	int fd;
	int err;
	int status;
	char name[64];
	struct nvgpu_clk_dev *dev;

	clk_arb_dbg(g, " ");

	dev = nvgpu_kzalloc(g, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	status = nvgpu_clk_notification_queue_alloc(g, &dev->queue,
		DEFAULT_EVENT_NUMBER);
	if (status < 0)  {
		err = status;
		goto fail;
	}

	fd = get_unused_fd_flags(O_RDWR);
	if (fd < 0) {
		err = fd;
		goto fail;
	}

	snprintf(name, sizeof(name), "%s-clk-fd%d", g->name, fd);
	file = anon_inode_getfile(name, fops, dev, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto fail_fd;
	}

	fd_install(fd, file);

	nvgpu_cond_init(&dev->readout_wq);

	nvgpu_atomic_set(&dev->poll_mask, 0);

	dev->session = session;
	nvgpu_ref_init(&dev->refcount);

	nvgpu_ref_get(&session->refcount);

	*_dev = dev;

	return fd;

fail_fd:
	put_unused_fd(fd);
fail:
	nvgpu_kfree(g, dev);

	return err;
}

int nvgpu_clk_arb_install_event_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd, u32 alarm_mask)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_dev *dev;
	int fd;

	clk_arb_dbg(g, " ");

	fd = nvgpu_clk_arb_install_fd(g, session, &event_dev_ops, &dev);
	if (fd < 0)
		return fd;

	/* TODO: alarm mask needs to be set to default value to prevent
	 * failures of legacy tests. This will be removed when sanity is
	 * updated
	 */
	if (alarm_mask)
		nvgpu_atomic_set(&dev->enabled_mask, alarm_mask);
	else
		nvgpu_atomic_set(&dev->enabled_mask, EVENT(VF_UPDATE));

	dev->arb_queue_head = nvgpu_atomic_read(&arb->notification_queue.head);

	nvgpu_spinlock_acquire(&arb->users_lock);
	nvgpu_list_add_tail(&dev->link, &arb->users);
	nvgpu_spinlock_release(&arb->users_lock);

	*event_fd = fd;

	return 0;
}

int nvgpu_clk_arb_install_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *request_fd)
{
	struct nvgpu_clk_dev *dev;
	int fd;

	clk_arb_dbg(g, " ");

	fd = nvgpu_clk_arb_install_fd(g, session, &completion_dev_ops, &dev);
	if (fd < 0)
		return fd;

	*request_fd = fd;

	return 0;
}

int nvgpu_clk_arb_commit_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int request_fd)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_dev *dev;
	struct fd fd;
	int err = 0;

	clk_arb_dbg(g, " ");

	fd  = fdget(request_fd);
	if (!fd.file)
		return -EINVAL;

	if (fd.file->f_op != &completion_dev_ops) {
		err = -EINVAL;
		goto fdput_fd;
	}

	dev = (struct nvgpu_clk_dev *) fd.file->private_data;

	if (!dev || dev->session != session) {
		err = -EINVAL;
		goto fdput_fd;
	}

	clk_arb_dbg(g, "requested target = %u\n",
		(u32)dev->gpc2clk_target_mhz);

	nvgpu_atomic_inc(&g->clk_arb_global_nr);
	nvgpu_ref_get(&dev->refcount);
	nvgpu_spinlock_acquire(&session->session_lock);
	nvgpu_list_add(&dev->node, &session->targets);
	nvgpu_spinlock_release(&session->session_lock);
	nvgpu_clk_arb_worker_enqueue(g, &arb->update_arb_work_item);

fdput_fd:
	fdput(fd);
	return err;
}

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
		int request_fd, u32 api_domain, u16 target_mhz)
{
	struct nvgpu_clk_dev *dev;
	struct fd fd;
	int err = 0;

	clk_arb_dbg(session->g,
			"domain=0x%08x target_mhz=%u", api_domain, target_mhz);

	fd = fdget(request_fd);
	if (!fd.file)
		return -EINVAL;

	if (fd.file->f_op != &completion_dev_ops) {
		err = -EINVAL;
		goto fdput_fd;
	}

	dev = fd.file->private_data;
	if (!dev || dev->session != session) {
		err = -EINVAL;
		goto fdput_fd;
	}

	switch (api_domain) {
	case NVGPU_CLK_DOMAIN_MCLK:
		dev->mclk_target_mhz = target_mhz;
		break;

	case NVGPU_CLK_DOMAIN_GPCCLK:
		dev->gpc2clk_target_mhz = target_mhz * 2ULL;
		break;

	default:
		err = -EINVAL;
	}

fdput_fd:
	fdput(fd);
	return err;
}

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g)
{
	u32 clk_domains = g->ops.clk_arb.get_arbiter_clk_domains(g);
	u32 api_domains = 0;

	if (clk_domains & CTRL_CLK_DOMAIN_GPC2CLK)
		api_domains |= BIT(NVGPU_GPU_CLK_DOMAIN_GPCCLK);

	if (clk_domains & CTRL_CLK_DOMAIN_MCLK)
		api_domains |= BIT(NVGPU_GPU_CLK_DOMAIN_MCLK);

	return api_domains;
}

#ifdef CONFIG_DEBUG_FS
static int nvgpu_clk_arb_stats_show(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_arb_debug *debug;

	u64 num;
	s64 tmp, avg, std, max, min;

	debug = NV_ACCESS_ONCE(arb->debug);
	/* Make copy of structure and ensure no reordering */
	nvgpu_smp_rmb();
	if (!debug)
		return -EINVAL;

	std = debug->switch_std;
	avg = debug->switch_avg;
	max = debug->switch_max;
	min = debug->switch_min;
	num = debug->switch_num;

	tmp = std;
	do_div(tmp, num);
	seq_printf(s, "Number of transitions: %lld\n",
		num);
	seq_printf(s, "max / min : %lld / %lld usec\n",
		max, min);
	seq_printf(s, "avg / std : %lld / %ld usec\n",
		avg, int_sqrt(tmp));

	return 0;
}

static int nvgpu_clk_arb_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvgpu_clk_arb_stats_show, inode->i_private);
}

static const struct file_operations nvgpu_clk_arb_stats_fops = {
	.open		= nvgpu_clk_arb_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


int nvgpu_clk_arb_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *gpu_root = l->debugfs;
	struct dentry *d;

	nvgpu_log(g, gpu_dbg_info, "g=%p", g);

	d = debugfs_create_file(
			"arb_stats",
			S_IRUGO,
			gpu_root,
			g,
			&nvgpu_clk_arb_stats_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}
#endif
