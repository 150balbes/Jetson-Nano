/*
 * Tegra Graphics Host Client Module
 *
 * Copyright (c) 2010-2020, NVIDIA Corporation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <soc/tegra/chip-id.h>
#include <linux/anon_inodes.h>
#include <linux/crc32.h>

#include <trace/events/nvhost.h>
#include <uapi/linux/nvhost_events.h>
#include <uapi/linux/nvdev_fence.h>

#include <linux/io.h>
#include <linux/string.h>

#include <linux/nvhost.h>
#include <uapi/linux/nvhost_ioctl.h>
#include <linux/nospec.h>

#ifdef CONFIG_EVENTLIB
#include <linux/keventlib.h>
#include "nvhost_events_json.h"
#endif

#include "debug.h"
#include "bus_client.h"
#include "dev.h"
#include "class_ids.h"
#include "chip_support.h"
#include "nvhost_acm.h"

#include "nvhost_syncpt.h"
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_sync.h"
#include "vhost/vhost.h"

static int validate_reg(struct platform_device *ndev, u32 offset, int count)
{
	int err = 0;
	struct resource *r;

	/* check if offset is u32 aligned */
	if (offset & 3) {
		nvhost_err(&ndev->dev, "misaligned register offset 0x%x",
			   offset);
		return -EINVAL;
	}

	r = platform_get_resource(ndev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&ndev->dev, "failed to get memory resource\n");
		return -ENODEV;
	}

	if (offset + 4 * count > resource_size(r)
	    || (offset + 4 * count < offset)) {
		nvhost_err(&ndev->dev,
			   "invalid register range offset 0x%x count %u",
			   offset, count);
		err = -EPERM;
	}

	/* prevent speculative access to mod's aperture + offset */
	speculation_barrier();

	return err;
}

int validate_max_size(struct platform_device *ndev, u32 size)
{
	struct resource *r;

	/* check if size is non-zero and u32 aligned */
	if (!size || size & 3) {
		nvhost_err(&ndev->dev,
			   "invalid dev size 0x%x",
			   size);
		return -EINVAL;
	}

	r = platform_get_resource(ndev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&ndev->dev, "failed to get memory resource\n");
		return -ENODEV;
	}

	if (size > resource_size(r)) {
		nvhost_err(&ndev->dev, "invalid dev size 0x%x", size);
		return -EPERM;
	}

	return 0;
}

void __iomem *get_aperture(struct platform_device *pdev, int index)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	return pdata->aperture[index];
}

void host1x_writel(struct platform_device *pdev, u32 r, u32 v)
{
	void __iomem *addr = get_aperture(pdev, 0) + r;
	nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", pdev->name, r, v);
	writel(v, addr);
}
EXPORT_SYMBOL_GPL(host1x_writel);

u32 host1x_readl(struct platform_device *pdev, u32 r)
{
	void __iomem *addr = get_aperture(pdev, 0) + r;
	u32 v;

	nvhost_dbg(dbg_reg, " d=%s r=0x%x", pdev->name, r);
	v = readl(addr);
	nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", pdev->name, r, v);

	return v;
}
EXPORT_SYMBOL_GPL(host1x_readl);

void host1x_channel_writel(struct nvhost_channel *ch, u32 r, u32 v)
{
	void __iomem *addr = ch->aperture + r;
	nvhost_dbg(dbg_reg, " chid=%d r=0x%x v=0x%x", ch->chid, r, v);
	writel(v, addr);
}
EXPORT_SYMBOL_GPL(host1x_channel_writel);

u32 host1x_channel_readl(struct nvhost_channel *ch, u32 r)
{
	void __iomem *addr = ch->aperture + r;
	u32 v;

	nvhost_dbg(dbg_reg, " chid=%d r=0x%x", ch->chid, r);
	v = readl(addr);
	nvhost_dbg(dbg_reg, " chid=%d r=0x%x v=0x%x", ch->chid, r, v);

	return v;
}
EXPORT_SYMBOL_GPL(host1x_channel_readl);

void host1x_sync_writel(struct nvhost_master *dev, u32 r, u32 v)
{
	void __iomem *addr = dev->sync_aperture + r;

	nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", dev->dev->name, r, v);
	writel(v, addr);
}
EXPORT_SYMBOL_GPL(host1x_sync_writel);

u32 host1x_sync_readl(struct nvhost_master *dev, u32 r)
{
	void __iomem *addr = dev->sync_aperture + r;
	u32 v;

	nvhost_dbg(dbg_reg, " d=%s r=0x%x", dev->dev->name, r);
	v = readl(addr);
	nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", dev->dev->name, r, v);

	return v;
}
EXPORT_SYMBOL_GPL(host1x_sync_readl);

int nvhost_read_module_regs(struct platform_device *ndev,
			u32 offset, int count, u32 *values)
{
	int err;

	/* verify offset */
	err = validate_reg(ndev, offset, count);
	if (err)
		return err;

	err = nvhost_module_busy(ndev);
	if (err)
		return err;

	while (count--) {
		*(values++) = host1x_readl(ndev, offset);
		offset += 4;
	}
	rmb();
	nvhost_module_idle(ndev);

	return 0;
}

int nvhost_write_module_regs(struct platform_device *ndev,
			u32 offset, int count, const u32 *values)
{
	int err;

	/* verify offset */
	err = validate_reg(ndev, offset, count);
	if (err)
		return err;

	err = nvhost_module_busy(ndev);
	if (err)
		return err;

	while (count--) {
		host1x_writel(ndev, offset, *(values++));
		offset += 4;
	}
	wmb();
	nvhost_module_idle(ndev);

	return 0;
}

struct nvhost_channel_userctx {
	struct nvhost_channel *ch;
	u32 timeout;
	int clientid;
	bool timeout_debug_dump;
	struct platform_device *pdev;
	u32 syncpts[NVHOST_MODULE_MAX_SYNCPTS];
	u32 client_managed_syncpt;

	/* error notificatiers used channel submit timeout */
	struct dma_buf *error_notifier_ref;
	u64 error_notifier_offset;

	/* lock to protect this structure from concurrent ioctl usage */
	struct mutex ioctl_lock;

	/* used for attaching to ctx list in device pdata */
	struct list_head node;
};

static int nvhost_channelrelease(struct inode *inode, struct file *filp)
{
	struct nvhost_channel_userctx *priv = filp->private_data;
	struct nvhost_device_data *pdata = platform_get_drvdata(priv->pdev);
	struct nvhost_master *host = nvhost_get_host(pdata->pdev);
	int i = 0;

	trace_nvhost_channel_release(dev_name(&priv->pdev->dev));

	mutex_lock(&pdata->userctx_list_lock);
	list_del(&priv->node);
	mutex_unlock(&pdata->userctx_list_lock);

	/* remove this client from acm */
	nvhost_module_remove_client(priv->pdev, priv);

	/* drop error notifier reference */
	if (priv->error_notifier_ref)
		dma_buf_put(priv->error_notifier_ref);

	/* Abort the channel */
	if (pdata->support_abort_on_close)
		nvhost_channel_abort(pdata, (void *)priv);

	/* Clear the identifier */
	if ((pdata->resource_policy == RESOURCE_PER_CHANNEL_INSTANCE) ||
			(pdata->resource_policy == RESOURCE_PER_DEVICE &&
			pdata->exclusive))
		nvhost_channel_remove_identifier(pdata, (void *)priv);

	/* If the device is in exclusive mode, drop the reference */
	if (pdata->exclusive)
		pdata->num_mapped_chs--;

	/* drop instance syncpoints reference */
	for (i = 0; i < NVHOST_MODULE_MAX_SYNCPTS; ++i) {
		if (priv->syncpts[i]) {
			nvhost_syncpt_put_ref(&host->syncpt,
					      priv->syncpts[i]);
			priv->syncpts[i] = 0;
		}
	}

	if (priv->client_managed_syncpt) {
		nvhost_syncpt_put_ref(&host->syncpt,
				      priv->client_managed_syncpt);
		priv->client_managed_syncpt = 0;
	}

	if (pdata->keepalive)
		nvhost_module_idle(priv->pdev);

	kfree(priv);
	return 0;
}

static int __nvhost_channelopen(struct inode *inode,
		struct platform_device *pdev,
		struct file *filp)
{
	struct nvhost_channel_userctx *priv;
	struct nvhost_device_data *pdata, *host1x_pdata;
	struct nvhost_master *host;
	int ret;

	/* grab pdev and pdata based on inputs */
	if (pdev) {
		pdata = platform_get_drvdata(pdev);
	} else if (inode) {
		pdata = container_of(inode->i_cdev,
				struct nvhost_device_data, cdev);
		pdev = pdata->pdev;
	} else {
		nvhost_err(NULL, "could not open the channel");
		return -EINVAL;
	}

	/* ..and host1x specific data */
	host1x_pdata = dev_get_drvdata(pdev->dev.parent);
	host = nvhost_get_host(pdev);

	trace_nvhost_channel_open(dev_name(&pdev->dev));

	/* If the device is in exclusive mode, make channel reservation here */
	if (pdata->exclusive) {
		if (pdata->num_mapped_chs == pdata->num_channels) {
			nvhost_err(&pdev->dev,
				   "no more available channels for an exclusive device");
			goto fail_mark_used;
		}
		pdata->num_mapped_chs++;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		nvhost_err(&pdev->dev,
			   "failed to allocate priv structure");
		goto fail_allocate_priv;
	}
	filp->private_data = priv;

	/* Register this client to acm */
	if (nvhost_module_add_client(pdev, priv))
		goto fail_add_client;

	/* Check that the device can be powered */
	ret = nvhost_module_busy(pdev);
	if (ret)
		goto fail_power_on;

	/* Turn off the device if we do not need to keep it powered */
	if (!pdata->keepalive)
		nvhost_module_idle(pdev);

	if (nvhost_dev_is_virtual(pdev) && !host->info.vmserver_owns_engines) {
		/* If virtual, allocate a client id on the server side. This is
		 * needed for channel recovery, to distinguish which clients
		 * own which gathers.
		 */

		int virt_moduleid = vhost_virt_moduleid(pdata->moduleid);
		struct nvhost_virt_ctx *virt_ctx =
					nvhost_get_virt_data(pdev);

		if (virt_moduleid < 0) {
			ret = -EINVAL;
			goto fail_virt_clientid;
		}

		priv->clientid =
			vhost_channel_alloc_clientid(virt_ctx->handle,
							virt_moduleid);
		if (priv->clientid == 0) {
			dev_err(&pdev->dev,
				"vhost_channel_alloc_clientid failed\n");
			ret = -ENOMEM;
			goto fail_virt_clientid;
		}
	} else {
		/* Get client id */
		priv->clientid = atomic_add_return(1, &host->clientid);
		if (!priv->clientid)
			priv->clientid = atomic_add_return(1, &host->clientid);
	}

	/* Initialize private structure */
	priv->timeout = host1x_pdata->nvhost_timeout_default;
	priv->timeout_debug_dump = true;
	mutex_init(&priv->ioctl_lock);
	priv->pdev = pdev;

	if (!tegra_platform_is_silicon())
		priv->timeout = 0;

	INIT_LIST_HEAD(&priv->node);
	mutex_lock(&pdata->userctx_list_lock);
	list_add_tail(&priv->node, &pdata->userctx_list);
	mutex_unlock(&pdata->userctx_list_lock);

	return 0;

fail_virt_clientid:
	if (pdata->keepalive)
		nvhost_module_idle(pdev);
fail_power_on:
	nvhost_module_remove_client(pdev, priv);
fail_add_client:
	kfree(priv);
fail_allocate_priv:
	if  (pdata->exclusive)
		pdata->num_mapped_chs--;
fail_mark_used:
	return -ENOMEM;
}

static int nvhost_channelopen(struct inode *inode, struct file *filp)
{
	return __nvhost_channelopen(inode, NULL, filp);
}

static int nvhost_init_error_notifier(struct nvhost_channel_userctx *ctx,
				      struct nvhost_set_error_notifier *args)
{
	struct dma_buf *dmabuf;
	void *va;
	u64 end = args->offset + sizeof(struct nvhost_notification);

	/* are we releasing old reference? */
	if (!args->mem) {
		if (ctx->error_notifier_ref)
			dma_buf_put(ctx->error_notifier_ref);
		ctx->error_notifier_ref = NULL;
		return 0;
	}

	/* take reference for the userctx */
	dmabuf = dma_buf_get(args->mem);
	if (IS_ERR(dmabuf)) {
		pr_err("%s: Invalid handle: %d\n", __func__, args->mem);
		return -EINVAL;
	}

	if (end > dmabuf->size || end < sizeof(struct nvhost_notification)) {
		dma_buf_put(dmabuf);
		pr_err("%s: invalid offset\n", __func__);
		return -EINVAL;
	}

	/* map handle and clear error notifier struct */
	va = dma_buf_vmap(dmabuf);
	if (!va) {
		dma_buf_put(dmabuf);
		pr_err("%s: Cannot map notifier handle\n", __func__);
		return -ENOMEM;
	}

	memset(va + args->offset, 0, sizeof(struct nvhost_notification));
	dma_buf_vunmap(dmabuf, va);

	/* release old reference */
	if (ctx->error_notifier_ref)
		dma_buf_put(ctx->error_notifier_ref);

	/* finally, store error notifier data */
	ctx->error_notifier_ref = dmabuf;
	ctx->error_notifier_offset = args->offset;

	return 0;
}

static inline u32 get_job_fence(struct nvhost_job *job, u32 id)
{
	struct nvhost_channel *ch = job->ch;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	u32 fence = job->sp[id].fence;

	/* take into account work done increment */
	if (pdata->push_work_done && id == 0)
		return fence - 1;

	/* otherwise the fence is valid "as is" */
	return fence;
}

static int submit_add_gathers(struct nvhost_submit_args *args,
			      struct nvhost_job *job,
			      struct nvhost_device_data *pdata)
{
	struct nvhost_cmdbuf __user *cmdbufs =
		(struct nvhost_cmdbuf __user *)(uintptr_t)args->cmdbufs;
	struct nvhost_cmdbuf_ext __user *cmdbuf_exts =
		(struct nvhost_cmdbuf_ext __user *)(uintptr_t)args->cmdbuf_exts;

	u32 __user *class_ids = (u32 __user *)(uintptr_t)args->class_ids;
	u32 *local_class_ids = NULL;

	int err;
	u32 i;

	/* mass copy class_ids */
	if (class_ids) {
		local_class_ids = kcalloc(args->num_cmdbufs, sizeof(u32),
			GFP_KERNEL);
		if (!local_class_ids)
			return -ENOMEM;

		err = copy_from_user(local_class_ids, class_ids,
			sizeof(u32) * args->num_cmdbufs);
		if (err) {
			nvhost_err(&pdata->pdev->dev,
				   "failed to copy user inputs: class_ids=%px num_cmdbufs=%u",
				   class_ids, args->num_cmdbufs);
			err = -EINVAL;
			goto free_local_class_ids;
		}
	}

	for (i = 0; i < args->num_cmdbufs; ++i) {
		struct nvhost_cmdbuf cmdbuf;
		struct nvhost_cmdbuf_ext cmdbuf_ext;
		u32 class_id = class_ids ? local_class_ids[i] : 0;

		err = copy_from_user(&cmdbuf, cmdbufs + i, sizeof(cmdbuf));
		if (err) {
			nvhost_err(&pdata->pdev->dev,
				   "failed to copy user inputs: cmdbufs+%d=%px",
				   i, cmdbufs + i);
			err = -EINVAL;
			goto free_local_class_ids;
		}

		cmdbuf_ext.pre_fence = -1;
		if (cmdbuf_exts)
			err = copy_from_user(&cmdbuf_ext,
					cmdbuf_exts + i, sizeof(cmdbuf_ext));
		if (err)
			cmdbuf_ext.pre_fence = -1;

		/* verify that the given class id is valid for this engine */
		if (class_id &&
		    class_id != pdata->class &&
		    class_id != NV_HOST1X_CLASS_ID) {
			nvhost_err(&pdata->pdev->dev,
				   "invalid class id 0x%x",
				   class_id);
			err = -EINVAL;
			goto free_local_class_ids;
		}

		nvhost_job_add_gather(job, cmdbuf.mem, cmdbuf.words,
				      cmdbuf.offset, class_id,
				      cmdbuf_ext.pre_fence);
	}

	kfree(local_class_ids);

	return 0;

free_local_class_ids:
	kfree(local_class_ids);

	return err;
}

static int submit_copy_relocs(struct nvhost_submit_args *args,
			      struct nvhost_job *job)
{
	struct nvhost_reloc __user *relocs =
		(struct nvhost_reloc __user *)(uintptr_t)args->relocs;
	struct nvhost_reloc_shift __user *reloc_shifts =
		(struct nvhost_reloc_shift __user *)
				(uintptr_t)args->reloc_shifts;
	struct nvhost_reloc_type __user *reloc_types =
		(struct nvhost_reloc_type __user *)
				(uintptr_t)args->reloc_types;
	struct device *d = &job->ch->dev->dev;

	int err;

	job->num_relocs = args->num_relocs;

	err = copy_from_user(job->relocarray,
			relocs, sizeof(*relocs) * args->num_relocs);
	if (err) {
		nvhost_err(d,
			   "failed to copy user input: relocs=%px num_relocs=%u",
			   relocs, args->num_relocs);
		return -EINVAL;
	}

	err = copy_from_user(job->relocshiftarray,
			reloc_shifts, sizeof(*reloc_shifts) * args->num_relocs);
	if (err) {
		nvhost_err(d,
			   "failed to copy user input: reloc_shifts=%px num_relocs=%u",
			   reloc_shifts, args->num_relocs);
		return -EINVAL;
	}

	if (reloc_types) {
		err = copy_from_user(job->reloctypearray,
			reloc_types, sizeof(*reloc_types) * args->num_relocs);
		if (err) {
			nvhost_err(d,
				   "failed to copy user input: reloc_types=%px num_relocs=%u",
				   reloc_types, args->num_relocs);
			return -EINVAL;
		}
	}

	return 0;
}

static int submit_get_syncpoints(struct nvhost_submit_args *args,
				 struct nvhost_job *job,
				 struct nvhost_channel_userctx *ctx)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ctx->pdev);
	struct nvhost_syncpt_incr __user *syncpt_incrs =
		(struct nvhost_syncpt_incr __user *)
				(uintptr_t)args->syncpt_incrs;
	int err;
	u32 i;

	if (args->num_syncpt_incrs > NVHOST_SUBMIT_MAX_NUM_SYNCPT_INCRS) {
		nvhost_err(&pdata->pdev->dev,
			   "num_syncpt_incrs=%u is larger than max=%u",
			   args->num_syncpt_incrs,
			   NVHOST_SUBMIT_MAX_NUM_SYNCPT_INCRS);
		return -EINVAL;
	}

	/*
	 * Go through each syncpoint from userspace. Here we:
	 * - Copy syncpoint information
	 * - Validate each syncpoint
	 * - Determine the index of hwctx syncpoint in the table
	 */

	for (i = 0; i < args->num_syncpt_incrs; ++i) {
		struct nvhost_syncpt_incr sp;
		bool found = false;
		int j;

		/* Copy */
		err = copy_from_user(&sp, syncpt_incrs + i, sizeof(sp));
		if (err) {
			nvhost_err(&pdata->pdev->dev,
				   "failed to copy user input: syncpt_incrs+%d=%px",
				   i, syncpt_incrs + i);
			return -EINVAL;
		}

		/* Validate the trivial case */
		if (sp.syncpt_id == 0) {
			nvhost_err(&pdata->pdev->dev,
				   "syncpt_id 0 forbidden");
			return -EINVAL;
		}

		/* ..and then ensure that the syncpoints have been reserved
		 * for this client */
		for (j = 0; j < NVHOST_MODULE_MAX_SYNCPTS; j++) {
			if (ctx->syncpts[j] == sp.syncpt_id) {
				found = true;
				break;
			}
		}

		if (!found) {
			nvhost_err(&pdata->pdev->dev,
				   "tried to use unreserved syncpoint %u",
				   sp.syncpt_id);
			return -EINVAL;
		}

		/* Store and get a reference */
		job->sp[i].id = sp.syncpt_id;
		job->sp[i].incrs = sp.syncpt_incrs;
	}

	return 0;
}

static int submit_deliver_fences(struct nvhost_submit_args *args,
				 struct nvhost_job *job,
				 struct nvhost_channel_userctx *ctx)
{
	u32 __user *fences = (u32 __user *)(uintptr_t)args->fences;

	int err;
	u32 i;

	/* Deliver multiple fences back to the userspace */
	if (fences)
		for (i = 0; i < args->num_syncpt_incrs; ++i) {
			u32 fence = get_job_fence(job, i);
			err = copy_to_user(fences + i, &fence, sizeof(u32));
			if (err)
				break;
		}

	/* Deliver the fence using the old mechanism _only_ if a single
	 * syncpoint is used. */

	if (args->flags & BIT(NVHOST_SUBMIT_FLAG_SYNC_FENCE_FD)) {
		struct nvhost_ctrl_sync_fence_info *pts;

		pts = kcalloc(args->num_syncpt_incrs,
			      sizeof(struct nvhost_ctrl_sync_fence_info),
			      GFP_KERNEL);
		if (!pts) {
			nvhost_err(&job->ch->dev->dev,
				   "failed to allocate pts");
			return -ENOMEM;
		}

		for (i = 0; i < args->num_syncpt_incrs; i++) {
			pts[i].id = job->sp[i].id;
			pts[i].thresh = get_job_fence(job, i);
		}

		err = nvhost_sync_create_fence_fd(ctx->pdev,
				pts, args->num_syncpt_incrs, "fence",
				&args->fence);
		kfree(pts);
		if (err)
			return err;
	} else if (args->num_syncpt_incrs == 1) {
		args->fence = get_job_fence(job, 0);
	} else {
		args->fence = 0;
	}

	return 0;
}

static int nvhost_ioctl_channel_submit(struct nvhost_channel_userctx *ctx,
		struct nvhost_submit_args *args)
{
	struct nvhost_job *job;
	struct nvhost_waitchk __user *waitchks =
		(struct nvhost_waitchk __user *)(uintptr_t)args->waitchks;
	struct nvhost_device_data *pdata = platform_get_drvdata(ctx->pdev);

	int err;

	if ((args->num_syncpt_incrs < 1) || (args->num_syncpt_incrs >
		nvhost_syncpt_nb_pts(&nvhost_get_host(ctx->pdev)->syncpt))) {
		nvhost_err(&pdata->pdev->dev,
			   "invalid num_syncpt_incrs=%u",
			   args->num_syncpt_incrs);
		return -EINVAL;
	}

	job = nvhost_job_alloc(ctx->ch,
			args->num_cmdbufs,
			args->num_relocs,
			args->num_waitchks,
			args->num_syncpt_incrs);
	if (!job)
		return -ENOMEM;

	job->num_syncpts = args->num_syncpt_incrs;
	job->clientid = ctx->clientid;
	job->client_managed_syncpt = ctx->client_managed_syncpt;

	/* copy error notifier settings for this job */
	if (ctx->error_notifier_ref) {
		get_dma_buf(ctx->error_notifier_ref);
		job->error_notifier_ref = ctx->error_notifier_ref;
		job->error_notifier_offset = ctx->error_notifier_offset;
	}

	err = submit_add_gathers(args, job, pdata);
	if (err)
		goto put_job;

	err = submit_copy_relocs(args, job);
	if (err)
		goto put_job;

	job->num_waitchk = args->num_waitchks;
	err = copy_from_user(job->waitchk,
			waitchks, sizeof(*waitchks) * args->num_waitchks);
	if (err) {
		nvhost_err(&pdata->pdev->dev,
			   "failed to copy user input: waitchks=%px num_waitchks=%u",
			   waitchks, args->num_waitchks);
		err = -EINVAL;
		goto put_job;
	}

	err = submit_get_syncpoints(args, job, ctx);
	if (err)
		goto put_job;

	trace_nvhost_channel_submit(ctx->pdev->name,
		job->num_gathers, job->num_relocs, job->num_waitchk,
		job->sp[0].id,
		job->sp[0].incrs);

	err = nvhost_module_busy(ctx->pdev);
	if (err)
		goto put_job;

	err = nvhost_job_pin(job, &nvhost_get_host(ctx->pdev)->syncpt);
	nvhost_module_idle(ctx->pdev);
	if (err)
		goto put_job;

	if (args->timeout)
		job->timeout = min(ctx->timeout, args->timeout);
	else
		job->timeout = ctx->timeout;
	job->timeout_debug_dump = ctx->timeout_debug_dump;

	err = nvhost_channel_submit(job);
	if (err)
		goto unpin_job;

	nvhost_eventlib_log_submit(ctx->pdev, job->sp[0].id,
			pdata->push_work_done ? (job->sp[0].fence - 1) :
			job->sp[0].fence, arch_counter_get_cntvct());

	err = submit_deliver_fences(args, job, ctx);
	if (err)
		goto put_job;

	nvhost_job_put(job);

	return 0;

unpin_job:
	nvhost_job_unpin(job);
put_job:
	nvhost_job_put(job);

	nvhost_err(&pdata->pdev->dev, "failed with err %d", err);

	return err;
}

static int moduleid_to_index(struct platform_device *dev, u32 moduleid)
{
	int i;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	for (i = 0; i < NVHOST_MODULE_MAX_CLOCKS; i++) {
		if (pdata->clocks[i].moduleid == moduleid)
			return i;
	}

	/* Old user space is sending a random number in args. Return clock
	 * zero in these cases. */
	return 0;
}

static int nvhost_ioctl_channel_set_rate(struct nvhost_channel_userctx *ctx,
	struct nvhost_clk_rate_args *arg)
{
	u32 moduleid = (arg->moduleid >> NVHOST_MODULE_ID_BIT_POS)
			& ((1 << NVHOST_MODULE_ID_BIT_WIDTH) - 1);
	u32 attr = (arg->moduleid >> NVHOST_CLOCK_ATTR_BIT_POS)
			& ((1 << NVHOST_CLOCK_ATTR_BIT_WIDTH) - 1);
	int index = moduleid ?
			moduleid_to_index(ctx->pdev, moduleid) : 0;
	int err;

	err = nvhost_module_set_rate(ctx->pdev, ctx, arg->rate, index, attr);
	if (!tegra_platform_is_silicon() && err) {
		nvhost_dbg(dbg_clk, "ignoring error: module=%u, attr=%u, index=%d, err=%d",
			   moduleid, attr, index, err);
		err = 0;
	}

	return err;
}

static int nvhost_ioctl_channel_get_rate(struct nvhost_channel_userctx *ctx,
	u32 moduleid, u32 *rate)
{
	int index = moduleid ? moduleid_to_index(ctx->pdev, moduleid) : 0;
	int err;

	err = nvhost_module_get_rate(ctx->pdev, (unsigned long *)rate, index);
	if (!tegra_platform_is_silicon() && err) {
		nvhost_dbg(dbg_clk, "ignoring error: module=%u, rate=%u, error=%d",
			   moduleid, *rate, err);
		err = 0;
		/* fake the return value */
		*rate = 32 * 1024;
	}

	return err;
}

static int nvhost_ioctl_channel_module_regrdwr(
	struct nvhost_channel_userctx *ctx,
	struct nvhost_ctrl_module_regrdwr_args *args)
{
	u32 num_offsets = args->num_offsets;
	u32 __user *offsets = (u32 __user *)(uintptr_t)args->offsets;
	u32 __user *values = (u32 __user *)(uintptr_t)args->values;
	u32 vals[64];
	struct platform_device *ndev;

	trace_nvhost_ioctl_channel_module_regrdwr(args->id,
		args->num_offsets, args->write);

	/* Check that there is something to read and that block size is
	 * u32 aligned */
	if (num_offsets == 0 || args->block_size & 3) {
		nvhost_err(&ctx->pdev->dev,
			   "invalid regrdwr parameters: num_offsets=%u block_size=0x%x",
			   num_offsets, args->block_size);
		return -EINVAL;
	}

	ndev = ctx->pdev;

	if (nvhost_dev_is_virtual(ndev))
		return vhost_rdwr_module_regs(ndev, num_offsets,
				args->block_size, offsets, values, args->write);

	while (num_offsets--) {
		int err;
		u32 offs;
		int remaining = args->block_size >> 2;

		if (get_user(offs, offsets)) {
			nvhost_err(&ndev->dev,
				   "failed to copy user's input: offsets=%px",
				   offsets);
			return -EFAULT;
		}

		offsets++;
		while (remaining) {
			int batch = min(remaining, 64);
			if (args->write) {
				if (copy_from_user(vals, values,
						batch * sizeof(u32))) {
					nvhost_err(&ndev->dev,
						"failed to copy user's input: values=%px batch=%u",
						values, batch);
					return -EFAULT;
				}

				err = nvhost_write_module_regs(ndev,
					offs, batch, vals);
				if (err)
					return err;
			} else {
				err = nvhost_read_module_regs(ndev,
						offs, batch, vals);
				if (err)
					return err;

				if (copy_to_user(values, vals,
						 batch * sizeof(u32))) {
					nvhost_err(&ndev->dev,
					    "failed to copy vals to user");
					return -EFAULT;
				}
			}

			remaining -= batch;
			offs += batch * sizeof(u32);
			values += batch;
		}
	}

	return 0;
}

static u32 create_mask(u32 *words, int num)
{
	int i;
	u32 word = 0;
	for (i = 0; i < num; i++) {
		if (!words[i] || words[i] > 31)
			continue;
		word |= BIT(words[i]);
	}

	return word;
}

static u32 nvhost_ioctl_channel_get_syncpt_mask(
		struct nvhost_channel_userctx *priv)
{
	u32 mask;

	mask = create_mask(priv->syncpts, NVHOST_MODULE_MAX_SYNCPTS);

	return mask;
}

static u32 nvhost_ioctl_channel_get_syncpt_instance(
		struct nvhost_channel_userctx *ctx,
		struct nvhost_device_data *pdata, u32 index)
{
	u32 id;

	/* if we already have required syncpt then return it ... */
	if (ctx->syncpts[index]) {
		id = ctx->syncpts[index];
		return id;
	}

	/* ... otherwise get a new syncpt dynamically */
	id = nvhost_get_syncpt_host_managed(pdata->pdev, index, NULL);
	if (!id)
		return 0;

	/* ... and store it for further references */
	ctx->syncpts[index] = id;

	return id;
}

static int nvhost_ioctl_channel_get_client_syncpt(
		struct nvhost_channel_userctx *ctx,
		struct nvhost_get_client_managed_syncpt_arg *args)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ctx->pdev);
	const char __user *args_name =
		(const char __user *)(uintptr_t)args->name;
	char name[32];
	char set_name[32];

	/* prepare syncpoint name (in case it is needed) */
	if (args_name) {
		if (strncpy_from_user(name, args_name, sizeof(name)) < 0) {
			nvhost_err(&ctx->pdev->dev,
				   "failed to copy from user: args_name=%px",
				   args_name);
			return -EFAULT;
		}
		name[sizeof(name) - 1] = '\0';
	} else {
		name[0] = '\0';
	}

	snprintf(set_name, sizeof(set_name),
		 "%s_%s", dev_name(&ctx->pdev->dev), name);

	if (!ctx->client_managed_syncpt)
		ctx->client_managed_syncpt =
			nvhost_get_syncpt_client_managed(pdata->pdev,
							 set_name);
	args->value = ctx->client_managed_syncpt;

	if (!args->value)
		return -EAGAIN;

	return 0;
}

static int nvhost_ioctl_channel_set_syncpoint_name(
				struct nvhost_channel_userctx *ctx,
				struct nvhost_set_syncpt_name_args *buf)
{

	struct nvhost_device_data *pdata = platform_get_drvdata(ctx->pdev);
	struct nvhost_master *host = nvhost_get_host(pdata->pdev);
	const char __user *args_name =
		(const char __user *)(uintptr_t)buf->name;
	char *syncpt_name;
	char name[32];
	int j;

	if (!nvhost_syncpt_is_valid_hw_pt_nospec(&host->syncpt, &buf->syncpt_id))
		return -EINVAL;

	if (args_name) {
		if (strncpy_from_user(name, args_name, sizeof(name)) < 0)
			return -EFAULT;
		name[sizeof(name) - 1] = '\0';
	} else {
		name[0] = '\0';
	}

	for (j = 0; j < NVHOST_MODULE_MAX_SYNCPTS; j++) {
		if (ctx->syncpts[j] == buf->syncpt_id) {
			syncpt_name = kasprintf(GFP_KERNEL, "%s", name);
			if (syncpt_name) {
				return nvhost_channel_set_syncpoint_name(
					&host->syncpt,
					buf->syncpt_id,
					(const char *)syncpt_name);
			} else {
				nvhost_err(&pdata->pdev->dev,
					   "failed to allocate syncpt_name");
				return -ENOMEM;
			}
		}
	}

	nvhost_err(&pdata->pdev->dev, "invalid syncpoint id %u",
		   buf->syncpt_id);
	return -EINVAL;
}

static long nvhost_channelctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct nvhost_channel_userctx *priv = filp->private_data;
	struct device *dev;
	u8 buf[NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE] __aligned(sizeof(u64));
	int err = 0;

	if ((_IOC_TYPE(cmd) != NVHOST_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_IOCTL_CHANNEL_LAST) ||
		(_IOC_SIZE(cmd) > NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE)) {
		nvhost_err(NULL, "invalid cmd 0x%x", cmd);
		return -ENOIOCTLCMD;
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd))) {
			nvhost_err(NULL,
				   "failed to copy from user: arg=%px",
				   (void __user *)arg);
			return -EFAULT;
		}
	}

	/* serialize calls from this fd */
	mutex_lock(&priv->ioctl_lock);
	if (!priv->pdev) {
		pr_warn("Channel already unmapped\n");
		mutex_unlock(&priv->ioctl_lock);
		return -EFAULT;
	}

	dev = &priv->pdev->dev;
	switch (cmd) {
	case NVHOST_IOCTL_CHANNEL_OPEN:
	{
		int fd;
		struct file *file;
		char *name;

		err = get_unused_fd_flags(O_RDWR);
		if (err < 0) {
			nvhost_err(dev, "failed to get unused fd");
			break;
		}
		fd = err;

		name = kasprintf(GFP_KERNEL, "nvhost-%s-fd%d",
				dev_name(dev), fd);
		if (!name) {
			nvhost_err(dev, "failed to allocate name");
			err = -ENOMEM;
			put_unused_fd(fd);
			break;
		}

		file = anon_inode_getfile(name, filp->f_op, NULL, O_RDWR);
		kfree(name);
		if (IS_ERR(file)) {
			nvhost_err(dev, "failed to get file");
			err = PTR_ERR(file);
			put_unused_fd(fd);
			break;
		}

		err = __nvhost_channelopen(NULL, priv->pdev, file);
		if (err) {
			put_unused_fd(fd);
			fput(file);
			break;
		}

		((struct nvhost_channel_open_args *)buf)->channel_fd = fd;
		fd_install(fd, file);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_SYNCPOINTS:
	{
		((struct nvhost_get_param_args *)buf)->value =
			nvhost_ioctl_channel_get_syncpt_mask(priv);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_SYNCPOINT:
	{
		struct nvhost_device_data *pdata =
			platform_get_drvdata(priv->pdev);
		struct nvhost_get_param_arg *arg =
			(struct nvhost_get_param_arg *)buf;

		if (arg->param >= NVHOST_MODULE_MAX_SYNCPTS) {
			nvhost_err(dev, "invalid syncpoint id %u", arg->param);
			err = -EINVAL;
			break;
		}

		/* prevent speculative access to ctx->syncpts[index] */
		arg->param = array_index_nospec(arg->param,
						NVHOST_MODULE_MAX_SYNCPTS);

		arg->value = nvhost_ioctl_channel_get_syncpt_instance(
						priv, pdata, arg->param);
		if (!arg->value) {
			err = -EAGAIN;
			break;
		}
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_CLIENT_MANAGED_SYNCPOINT:
	{
		err = nvhost_ioctl_channel_get_client_syncpt(priv,
			(struct nvhost_get_client_managed_syncpt_arg *)buf);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_FREE_CLIENT_MANAGED_SYNCPOINT:
		break;
	case NVHOST_IOCTL_CHANNEL_GET_WAITBASES:
	{
		((struct nvhost_get_param_args *)buf)->value = 0;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_WAITBASE:
	{
		nvhost_err(dev, "GET_WAITBASE (%d) not supported", cmd);
		err = -EINVAL;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_MODMUTEXES:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->pdev);
		((struct nvhost_get_param_args *)buf)->value =
			create_mask(pdata->modulemutexes,
					NVHOST_MODULE_MAX_MODMUTEXES);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_MODMUTEX:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->pdev);
		struct nvhost_get_param_arg *arg =
			(struct nvhost_get_param_arg *)buf;

		if (arg->param >= NVHOST_MODULE_MAX_MODMUTEXES) {
			nvhost_err(dev, "invalid modmutex 0x%x", arg->param);
			err = -EINVAL;
			break;
		}

		arg->param = array_index_nospec(arg->param,
						NVHOST_MODULE_MAX_MODMUTEXES);

		if (!pdata->modulemutexes[arg->param]) {
			nvhost_err(dev, "invalid modmutex 0x%x", arg->param);
			err = -EINVAL;
			break;
		}

		arg->value = pdata->modulemutexes[arg->param];
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_NVMAP_FD:
		break;
	case NVHOST_IOCTL_CHANNEL_GET_CLK_RATE:
	{
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;

		/* if virtualized, just return 0 */
		if (nvhost_dev_is_virtual(priv->pdev)) {
			arg->rate = 0;
			break;
		}

		err = nvhost_ioctl_channel_get_rate(priv,
				arg->moduleid, &arg->rate);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_CLK_RATE:
	{
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;

		/* if virtualized, client requests to change clock rate
		 * are ignored
		 */
		if (nvhost_dev_is_virtual(priv->pdev))
			break;

		err = nvhost_ioctl_channel_set_rate(priv, arg);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT:
	{
		u32 timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;

		priv->timeout = timeout;
		dev_dbg(&priv->pdev->dev,
			"%s: setting buffer timeout (%d ms) for userctx 0x%p\n",
			__func__, priv->timeout, priv);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_TIMEDOUT:
		((struct nvhost_get_param_args *)buf)->value = false;
		break;
	case NVHOST_IOCTL_CHANNEL_SET_PRIORITY:
		nvhost_err(dev, "SET_PRIORITY not supported");
		err = -EINVAL;
		break;
	case NVHOST32_IOCTL_CHANNEL_MODULE_REGRDWR:
	{
		struct nvhost32_ctrl_module_regrdwr_args *args32 =
			(struct nvhost32_ctrl_module_regrdwr_args *)buf;
		struct nvhost_ctrl_module_regrdwr_args args;
		args.id = args32->id;
		args.num_offsets = args32->num_offsets;
		args.block_size = args32->block_size;
		args.offsets = args32->offsets;
		args.values = args32->values;
		args.write = args32->write;
		err = nvhost_ioctl_channel_module_regrdwr(priv, &args);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_MODULE_REGRDWR:
		err = nvhost_ioctl_channel_module_regrdwr(priv, (void *)buf);
		break;
	case NVHOST32_IOCTL_CHANNEL_SUBMIT:
	{
		struct nvhost_device_data *pdata =
			platform_get_drvdata(priv->pdev);
		struct nvhost32_submit_args *args32 = (void *)buf;
		struct nvhost_submit_args args;
		void *identifier;

		if (pdata->resource_policy == RESOURCE_PER_DEVICE &&
		    !pdata->exclusive)
			identifier = (void *)pdata;
		else
			identifier = (void *)priv;

		memset(&args, 0, sizeof(args));
		args.submit_version = args32->submit_version;
		args.num_syncpt_incrs = args32->num_syncpt_incrs;
		args.num_cmdbufs = args32->num_cmdbufs;
		args.num_relocs = args32->num_relocs;
		args.num_waitchks = args32->num_waitchks;
		args.timeout = args32->timeout;
		args.syncpt_incrs = args32->syncpt_incrs;
		args.fence = args32->fence;

		args.cmdbufs = args32->cmdbufs;
		args.relocs = args32->relocs;
		args.reloc_shifts = args32->reloc_shifts;
		args.waitchks = args32->waitchks;
		args.class_ids = args32->class_ids;
		args.fences = args32->fences;

		/* first, get a channel */
		err = nvhost_channel_map(pdata, &priv->ch, identifier);
		if (err)
			break;

		/* submit work */
		err = nvhost_ioctl_channel_submit(priv, &args);

		/* ..and drop the local reference */
		nvhost_putchannel(priv->ch, 1);

		args32->fence = args.fence;

		break;
	}
	case NVHOST_IOCTL_CHANNEL_SUBMIT:
	{
		struct nvhost_device_data *pdata =
			platform_get_drvdata(priv->pdev);
		void *identifier;

		if (pdata->resource_policy == RESOURCE_PER_DEVICE &&
		    !pdata->exclusive)
			identifier = (void *)pdata;
		else
			identifier = (void *)priv;

		/* first, get a channel */
		err = nvhost_channel_map(pdata, &priv->ch, identifier);
		if (err)
			break;

		/* submit work */
		err = nvhost_ioctl_channel_submit(priv, (void *)buf);

		/* ..and drop the local reference */
		nvhost_putchannel(priv->ch, 1);

		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_ERROR_NOTIFIER:
		err = nvhost_init_error_notifier(priv,
			(struct nvhost_set_error_notifier *)buf);
		break;
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT_EX:
	{
		u32 timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;
		bool timeout_debug_dump = !((u32)
			((struct nvhost_set_timeout_ex_args *)buf)->flags &
			(1 << NVHOST_TIMEOUT_FLAG_DISABLE_DUMP));
		priv->timeout = timeout;
		priv->timeout_debug_dump = timeout_debug_dump;
		dev_dbg(&priv->pdev->dev,
			"%s: setting buffer timeout (%d ms) for userctx 0x%p\n",
			__func__, priv->timeout, priv);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_SYNCPOINT_NAME:
	{
		err = nvhost_ioctl_channel_set_syncpoint_name(priv,
			(struct nvhost_set_syncpt_name_args *)buf);
		break;
	}
	default:
		nvhost_dbg_info("unrecognized ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	mutex_unlock(&priv->ioctl_lock);

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ)) {
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));
		if (err) {
			nvhost_err(dev, "failed to copy output to user: arg=%px",
				   (void __user *)arg);
			err = -EFAULT;
		}
	}

	return err;
}

static const struct file_operations nvhost_channelops = {
	.owner = THIS_MODULE,
	.release = nvhost_channelrelease,
	.open = nvhost_channelopen,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvhost_channelctl,
#endif
	.unlocked_ioctl = nvhost_channelctl
};

const char *get_device_name_for_dev(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);

	if (pdata->devfs_name)
		return pdata->devfs_name;

	return dev->name;
}

static struct device *nvhost_client_device_create(
	struct platform_device *pdev, struct cdev *cdev,
	const char *cdev_name, dev_t devno,
	const struct file_operations *ops)
{
	struct nvhost_master *host = nvhost_get_host(pdev);
	const char *use_dev_name;
	struct device *dev;
	int err;

	nvhost_dbg_fn("");

	if (!host) {
		dev_err(&pdev->dev, "No nvhost_master!\n");
		return ERR_PTR(-EINVAL);
	}

	cdev_init(cdev, ops);
	cdev->owner = THIS_MODULE;

	err = cdev_add(cdev, devno, 1);
	if (err < 0) {
		dev_err(&pdev->dev,
			"failed to add cdev\n");
		return ERR_PTR(err);
	}
	use_dev_name = get_device_name_for_dev(pdev);

	dev = device_create(host->nvhost_class,
			&pdev->dev, devno, NULL,
			(pdev->id <= 0) ?
			IFACE_NAME "-%s%s" :
			IFACE_NAME "-%s%s.%d",
			cdev_name, use_dev_name, pdev->id);

	if (IS_ERR(dev)) {
		dev_err(&pdev->dev,
			"failed to create %s %s device for %s\n",
			use_dev_name, cdev_name, pdev->name);
		cdev_del(cdev);
	}

	return dev;
}

#define NVHOST_NUM_CDEV 4

int nvhost_client_user_init(struct platform_device *dev)
{
	int err;
	dev_t devno;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct nvhost_master *nvhost_master = nvhost_get_host(dev);

	if (pdata->kernel_only)
		return 0;

	if (!nvhost_master) {
		dev_err(&dev->dev, "failed to get nvhost_master!\n");
		return -ENODEV;
	}

	if (!nvhost_master->major) {
		dev_err(&dev->dev, "Major chrdev number not allocated!\n");
		return -EINVAL;
	}

	mutex_lock(&nvhost_master->chrdev_mutex);

	devno = MKDEV(nvhost_master->major, nvhost_master->next_minor);
	/* reserve 3 minor #s for <dev>, and ctrl-<dev> */
	err = register_chrdev_region(devno, NVHOST_NUM_CDEV,
				     IFACE_NAME);
	if (err < 0) {
		dev_err(&dev->dev, "failed to allocate devno %d %d\n",
			nvhost_master->major, nvhost_master->next_minor);
		mutex_unlock(&nvhost_master->chrdev_mutex);
		return err;
	}

	nvhost_master->next_minor += NVHOST_NUM_CDEV;
	mutex_unlock(&nvhost_master->chrdev_mutex);

	pdata->cdev_region = devno;

	pdata->node = nvhost_client_device_create(dev, &pdata->cdev,
				"", devno, &nvhost_channelops);
	if (IS_ERR(pdata->node))
		return PTR_ERR(pdata->node);

	/* module control (npn-channel based, global) interface */
	if (pdata->ctrl_ops) {
		++devno;
		pdata->ctrl_node = nvhost_client_device_create(dev,
					&pdata->ctrl_cdev, "ctrl-",
					devno, pdata->ctrl_ops);
		if (IS_ERR(pdata->ctrl_node))
			return PTR_ERR(pdata->ctrl_node);
	}

	return 0;
}

static void nvhost_client_user_deinit(struct platform_device *dev)
{
	struct nvhost_master *nvhost_master = nvhost_get_host(dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (pdata->kernel_only)
		return;

	if (!IS_ERR_OR_NULL(pdata->node)) {
		device_destroy(nvhost_master->nvhost_class, pdata->cdev.dev);
		cdev_del(&pdata->cdev);
	}

	if (!IS_ERR_OR_NULL(pdata->as_node)) {
		device_destroy(nvhost_master->nvhost_class, pdata->as_cdev.dev);
		cdev_del(&pdata->as_cdev);
	}

	if (!IS_ERR_OR_NULL(pdata->ctrl_node)) {
		device_destroy(nvhost_master->nvhost_class,
			       pdata->ctrl_cdev.dev);
		cdev_del(&pdata->ctrl_cdev);
	}

	unregister_chrdev_region(pdata->cdev_region, NVHOST_NUM_CDEV);
}

int nvhost_client_device_init(struct platform_device *dev)
{
	int err;
	struct nvhost_master *nvhost_master = nvhost_get_host(dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	mutex_init(&pdata->userctx_list_lock);
	INIT_LIST_HEAD(&pdata->userctx_list);

	nvhost_client_devfs_name_init(dev);

	/* Create debugfs directory for the device */
	nvhost_device_debug_init(dev);

	err = nvhost_client_user_init(dev);
	if (err)
		goto fail;

	err = nvhost_device_list_add(dev);
	if (err)
		goto fail;

	if (pdata->scaling_init)
		pdata->scaling_init(dev);

#ifdef CONFIG_EVENTLIB
	pdata->eventlib_id = keventlib_register(4 * PAGE_SIZE,
						dev_name(&dev->dev),
						nvhost_events_json,
						nvhost_events_json_len);
	if (pdata->eventlib_id < 0) {
		nvhost_warn(&dev->dev, "failed to register eventlib (err=%d)",
			    pdata->eventlib_id);
		pdata->eventlib_id = 0;
	}
#endif

	/* reset syncpoint values for this unit */
	err = nvhost_module_busy(nvhost_master->dev);
	if (err)
		goto fail_busy;

	nvhost_module_idle(nvhost_master->dev);

	/* Initialize dma parameters */
	dev->dev.dma_parms = &pdata->dma_parms;
	dma_set_max_seg_size(&dev->dev, UINT_MAX);

	dev_info(&dev->dev, "initialized\n");

	if (pdata->resource_policy == RESOURCE_PER_CHANNEL_INSTANCE) {
		nvhost_master->info.channel_policy = MAP_CHANNEL_ON_SUBMIT;
		nvhost_update_characteristics(dev);
	}

	if (pdata->hw_init)
		return pdata->hw_init(dev);

	return 0;

fail_busy:
	/* Remove from nvhost device list */
	nvhost_device_list_remove(dev);
fail:
	/* Add clean-up */
	dev_err(&dev->dev, "failed to init client device\n");
	nvhost_client_user_deinit(dev);
	nvhost_device_debug_deinit(dev);
	return err;
}
EXPORT_SYMBOL(nvhost_client_device_init);

int nvhost_client_device_release(struct platform_device *dev)
{
#ifdef CONFIG_EVENTLIB
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (pdata->eventlib_id)
		keventlib_unregister(pdata->eventlib_id);
#endif

	/* Release nvhost module resources */
	nvhost_module_deinit(dev);

	/* Remove from nvhost device list */
	nvhost_device_list_remove(dev);

	/* Release chardev and device node for user space */
	nvhost_client_user_deinit(dev);

	/* Remove debugFS */
	nvhost_device_debug_deinit(dev);

	return 0;
}
EXPORT_SYMBOL(nvhost_client_device_release);

int nvhost_device_get_resources(struct platform_device *dev)
{
	int i;
	void __iomem *regs = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int ret;

	for (i = 0; i < dev->num_resources; i++) {
		struct resource *r = NULL;

		r = platform_get_resource(dev, IORESOURCE_MEM, i);
		/* We've run out of mem resources */
		if (!r)
			break;

		regs = devm_ioremap_resource(&dev->dev, r);
		if (IS_ERR(regs)) {
			ret = PTR_ERR(regs);
			goto fail;
		}

		pdata->aperture[i] = regs;
	}

	return 0;

fail:
	dev_err(&dev->dev, "failed to get register memory\n");

	return ret;
}

int nvhost_client_device_get_resources(struct platform_device *dev)
{
	return nvhost_device_get_resources(dev);
}
EXPORT_SYMBOL(nvhost_client_device_get_resources);

/* This is a simple wrapper around request_firmware that takes
 * 'fw_name' and if available applies a SOC relative path prefix to it.
 * The caller is responsible for calling release_firmware later.
 */
const struct firmware *
nvhost_client_request_firmware(struct platform_device *dev, const char *fw_name)
{
	struct nvhost_chip_support *op = nvhost_get_chip_ops();
	const struct firmware *fw;
	char *fw_path = NULL;
	int path_len, err;

	/* This field is NULL when calling from SYS_EXIT.
	   Add a check here to prevent crash in request_firmware */
	if (!current->fs) {
		WARN_ON(1);
		return NULL;
	}

	if (!fw_name)
		return NULL;

	if (op->soc_name) {
		path_len = strlen(fw_name) + strlen(op->soc_name);
		path_len += 2; /* for the path separator and zero terminator*/

		fw_path = kzalloc(sizeof(*fw_path) * path_len,
				     GFP_KERNEL);
		if (!fw_path)
			return NULL;

		snprintf(fw_path, sizeof(*fw_path) * path_len, "%s/%s", op->soc_name, fw_name);
		fw_name = fw_path;
	}

	err = request_firmware(&fw, fw_name, &dev->dev);
	kfree(fw_path);
	if (err) {
		dev_err(&dev->dev, "failed to get firmware\n");
		return NULL;
	}

	/* note: caller must release_firmware */
	return fw;
}
EXPORT_SYMBOL(nvhost_client_request_firmware);

struct nvhost_channel *nvhost_find_chan_by_clientid(
				struct platform_device *pdev,
				int clientid)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_channel_userctx *ctx;
	struct nvhost_channel *ch = NULL;

	mutex_lock(&pdata->userctx_list_lock);
	list_for_each_entry(ctx, &pdata->userctx_list, node) {
		if (ctx->clientid == clientid) {
			ch = ctx->ch;
			break;
		}
	}
	mutex_unlock(&pdata->userctx_list_lock);

	return ch;
}

#ifdef CONFIG_EVENTLIB
void nvhost_eventlib_log_task(struct platform_device *pdev,
			      u32 syncpt_id,
			      u32 syncpt_thresh,
			      u64 timestamp_start,
			      u64 timestamp_end)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_task_begin task_begin;
	struct nvhost_task_end task_end;

	if (!pdata->eventlib_id)
		return;

	/*
	 * Write task start event
	 */
	task_begin.syncpt_id = syncpt_id;
	task_begin.syncpt_thresh = syncpt_thresh;
	task_begin.class_id = pdata->class;

	keventlib_write(pdata->eventlib_id,
			&task_begin,
			sizeof(task_begin),
			NVHOST_TASK_BEGIN,
			timestamp_start);

	/*
	 * Write task end event
	 */
	task_end.syncpt_id = syncpt_id;
	task_end.syncpt_thresh = syncpt_thresh;
	task_end.class_id = pdata->class;

	keventlib_write(pdata->eventlib_id,
			&task_end,
			sizeof(task_end),
			NVHOST_TASK_END,
			timestamp_end);
}

void nvhost_eventlib_log_submit(struct platform_device *pdev,
				u32 syncpt_id,
				u32 syncpt_thresh,
				u64 timestamp)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_task_submit task_submit;

	if (!pdata->eventlib_id)
		return;

	/*
	 * Write task start event
	 */
	task_submit.syncpt_id = syncpt_id;
	task_submit.syncpt_thresh = syncpt_thresh;
	task_submit.class_id = pdata->class;
	task_submit.pid = current->tgid;
	task_submit.tid = current->pid;

	keventlib_write(pdata->eventlib_id,
			&task_submit,
			sizeof(task_submit),
			NVHOST_TASK_SUBMIT,
			timestamp);
}

void nvhost_eventlib_log_fences(struct platform_device *pdev,
				u32 task_syncpt_id,
				u32 task_syncpt_thresh,
				struct nvdev_fence *fences,
				u8 num_fences,
				enum nvdev_fence_kind kind,
				u64 timestamp)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	u8 i;

	if (!pdata->eventlib_id)
		return;

	for (i = 0; i < num_fences; i++) {
		struct nvhost_task_fence task_fence;

		memset(&task_fence, 0, sizeof(task_fence));

		/* Basic fence fields common for all types */
		task_fence.class_id = pdata->class;
		task_fence.kind = (u32) kind;
		task_fence.fence_type = fences[i].type;
		task_fence.task_syncpt_id = task_syncpt_id;
		task_fence.task_syncpt_thresh = task_syncpt_thresh;

		switch (fences[i].type) {
		case NVDEV_FENCE_TYPE_SYNCPT:
			task_fence.syncpt_id = fences[i].syncpoint_index;
			task_fence.syncpt_thresh = fences[i].syncpoint_value;
			break;
		case NVDEV_FENCE_TYPE_SYNC_FD:
			task_fence.sync_fd = fences[i].sync_fd;
			break;
		case NVDEV_FENCE_TYPE_SEMAPHORE:
		case NVDEV_FENCE_TYPE_SEMAPHORE_TS:
			task_fence.semaphore_handle =
				fences[i].semaphore_handle;
			task_fence.semaphore_offset =
				fences[i].semaphore_offset;
			task_fence.semaphore_value =
				fences[i].semaphore_value;
			break;
		default:
			nvhost_warn(&pdev->dev, "unknown fence type %d",
				    fences[i].type);
			break;
		}

		keventlib_write(pdata->eventlib_id, &task_fence,
				sizeof(task_fence), NVHOST_TASK_FENCE,
				timestamp);
	}
}

#else
void nvhost_eventlib_log_task(struct platform_device *pdev,
			      u32 syncpt_id,
			      u32 syncpt_thres,
			      u64 timestamp_start,
			      u64 timestamp_end)
{
}

void nvhost_eventlib_log_submit(struct platform_device *pdev,
				u32 syncpt_id,
				u32 syncpt_thresh,
				u64 timestamp)
{
}

void nvhost_eventlib_log_fences(struct platform_device *pdev,
				u32 task_syncpt_id,
				u32 task_syncpt_thresh,
				struct nvdev_fence *fences,
				u8 num_fences,
				enum nvdev_fence_kind kind,
				u64 timestamp)
{
}

#endif
EXPORT_SYMBOL(nvhost_eventlib_log_submit);
EXPORT_SYMBOL(nvhost_eventlib_log_task);
EXPORT_SYMBOL(nvhost_eventlib_log_fences);
