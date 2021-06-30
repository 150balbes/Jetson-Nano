/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <drm/drmP.h>
#include <linux/dma-buf.h>
#include <linux/shmem_fs.h>

#include <uapi/drm/tegra_udrm.h>

#define DRIVER_NAME "tegra-udrm"
#define DRIVER_DESC "Kernel DRM support for user mode DRM driver on NVIDIA Tegra Soc"
/* Change driver date every year. Doesn't really signify anything. */
#define DRIVER_DATE "20182809"
/* Increase the major number when changes to driver makes it incompatible with
 * user mode driver e.g. change in ioctl args.
 */
#define DRIVER_MAJOR 0
/* Increase the minor number for minor updates which won't break compatibility
 * with user mode driver.
 */
#define DRIVER_MINOR 0

MODULE_PARM_DESC(
		modeset,
		"Enable/Disable modesetting (1 = enable, 0 = disable (default))");
static bool tegra_udrm_modeset_module_param;
module_param_named(modeset, tegra_udrm_modeset_module_param, bool, 0400);

struct tegra_udrm_private {
	struct drm_device *drm;
};

struct tegra_udrm_device {
	struct drm_device *drm;
};

struct tegra_udrm_file {
	/* eventfd context to signal user space that driver is closing. */
	struct eventfd_ctx *efd_ctx_close;

	/* eventfd context to signal user space that drop master for this
	 * drm_file is called.
	 */
	struct eventfd_ctx *efd_ctx_drop_master;
	struct eventfd_ctx *efd_ctx_set_master;

	/* Holds list of dmabuf fd and corresponding unique id. Id is used
	 * to return fake offset in dmabuf mmap ioctl. User space sends this
	 * offset in mmap(), driver then uses it to find dambuf fd.
	 */
	struct idr idr;
};

struct tegra_udrm_mmap_entry {
	int dmabuf_fd;
};

static int tegra_udrm_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct drm_file *priv = file->private_data;
	struct tegra_udrm_file *fpriv = priv->driver_priv;
	struct dma_buf *dmabuf;
	struct tegra_udrm_mmap_entry *mmap_entry;
	int ret;

	mmap_entry = idr_find(&fpriv->idr, vma->vm_pgoff);

	if (!mmap_entry)
		return -EINVAL;

	dmabuf = dma_buf_get(mmap_entry->dmabuf_fd);
	if (IS_ERR(dmabuf))
		return -EINVAL;

	/* set the vm_pgoff (used as a fake buffer offset by DRM) to 0
	 * as we want to map the whole buffer.
	 */
	vma->vm_pgoff = 0;

	ret = dmabuf->ops->mmap(dmabuf, vma);

	dma_buf_put(dmabuf);

	return ret;
}

static int tegra_udrm_dmabuf_mmap_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	struct tegra_udrm_file *fpriv = file->driver_priv;
	struct drm_tegra_udrm_dmabuf_mmap *args =
		(struct drm_tegra_udrm_dmabuf_mmap *)data;
	struct tegra_udrm_mmap_entry *mmap_entry;
	int id;

	if (args->fd < 0)
		return -EINVAL;

	/* Check if set up for mmap for this dmabuf fd has already done. */
	idr_for_each_entry(&fpriv->idr, mmap_entry, id) {
		if (args->fd == mmap_entry->dmabuf_fd) {
			args->offset = (unsigned long)(id) << PAGE_SHIFT;
			return 0;
		}
	}

	mmap_entry = kmalloc(sizeof(*mmap_entry), GFP_KERNEL);
	if (!mmap_entry)
		return -ENOMEM;

	mmap_entry->dmabuf_fd = args->fd;

	/* mmap_entry will be freed when user space calls destroy mappings
	 * ioctl. Driver's preclose function will free all unfreed mmap
	 * entries before destroying idr.
	 */
	id = idr_alloc(&fpriv->idr, mmap_entry, 0, 0 /* INT_MAX */,
			GFP_KERNEL);
	if (id < 0) {
		kfree(mmap_entry);
		return -ENOMEM;
	}

	/* We have to return fake offset to use for subsequent mmap by user
	 * space. Return offset by doing id << PAGE_SHIFT as mmap() does
	 * offset = offset >> PAGE_SHIFT before sending offset to driver.
	 *
	 * Max value of id is INT_MAX, it is unlikely that PAGE_SHIFT is
	 * more than 31. So we won't overflow offset which is unsigned
	 * long by doing id << PAGE_SHIFT.
	 */
	args->offset = (unsigned long)(id) << PAGE_SHIFT;

	return 0;
}

static int tegra_udrm_dmabuf_destroy_mappings_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	struct tegra_udrm_file *fpriv = file->driver_priv;
	struct drm_tegra_udrm_dmabuf_destroy_mappings *args =
		(struct drm_tegra_udrm_dmabuf_destroy_mappings *)data;
	struct tegra_udrm_mmap_entry *mmap_entry;
	int id;

	idr_for_each_entry(&fpriv->idr, mmap_entry, id) {
		if (args->fd == mmap_entry->dmabuf_fd) {
			idr_remove(&fpriv->idr, id);
			kfree(mmap_entry);
			return 0;
		}
	}

	return -EINVAL;
}

static void tegra_udrm_preclose(struct drm_device *drm, struct drm_file *file)
{
	struct tegra_udrm_file *fpriv = file->driver_priv;
	struct tegra_udrm_mmap_entry *mmap_entry;
	int id;

	idr_for_each_entry(&fpriv->idr, mmap_entry, id) {
		idr_remove(&fpriv->idr, id);
		kfree(mmap_entry);
	}

	idr_destroy(&fpriv->idr);

	if (fpriv->efd_ctx_drop_master) {
		eventfd_ctx_put(fpriv->efd_ctx_drop_master);
		eventfd_signal(fpriv->efd_ctx_drop_master, 1);
		fpriv->efd_ctx_drop_master = NULL;
	}

	if (fpriv->efd_ctx_close) {
		// Signal user mode driver to start it's close sequence.
		eventfd_signal(fpriv->efd_ctx_close, 1);
		// Release reference to acquired eventfd context as we
		// are closing.
		eventfd_ctx_put(fpriv->efd_ctx_close);
		fpriv->efd_ctx_close = NULL;
	}
}

static int tegra_udrm_close_notify_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	struct tegra_udrm_file *fpriv = file->driver_priv;
	struct drm_tegra_udrm_close_notify *args =
		(struct drm_tegra_udrm_close_notify *)data;
	int err;

	if (args->clear) {
		// Releases reference to the previously acquired eventfd
		// context.
		if (fpriv->efd_ctx_close) {
			eventfd_ctx_put(fpriv->efd_ctx_close);
			fpriv->efd_ctx_close = NULL;
		}
		return 0;
	}

	// Fail if already acquired a reference to the eventfd context.
	if (fpriv->efd_ctx_close)
		return -EINVAL;

	fpriv->efd_ctx_close = eventfd_ctx_fdget(args->eventfd);
	if (IS_ERR(fpriv->efd_ctx_close)) {
		err = PTR_ERR(fpriv->efd_ctx_close);
		fpriv->efd_ctx_close = NULL;
		return err;
	}

	return 0;
}

static int tegra_udrm_drop_master_notify_ioctl(struct drm_device *drm,
		void *data, struct drm_file *file)
{
	struct tegra_udrm_file *fpriv = file->driver_priv;
	struct drm_tegra_udrm_drop_master_notify *args =
		(struct drm_tegra_udrm_drop_master_notify *)data;
	int err;

	if (args->clear) {
		// Releases reference to the previously acquired eventfd
		// context.
		if (fpriv->efd_ctx_drop_master) {
			eventfd_ctx_put(fpriv->efd_ctx_drop_master);
			fpriv->efd_ctx_drop_master = NULL;
		}
		return 0;
	}

	// Fail if already acquired a reference to the eventfd context.
	if (fpriv->efd_ctx_drop_master)
		return -EBUSY;

	fpriv->efd_ctx_drop_master = eventfd_ctx_fdget(args->eventfd);
	if (IS_ERR(fpriv->efd_ctx_drop_master)) {
		err = PTR_ERR(fpriv->efd_ctx_drop_master);
		fpriv->efd_ctx_drop_master = NULL;
		return err;
	}

	return 0;
}

static int tegra_udrm_set_master_notify_ioctl(struct drm_device *drm,
		void *data, struct drm_file *file)
{
	struct tegra_udrm_file *fpriv = file->driver_priv;
	struct drm_tegra_udrm_set_master_notify *args =
		(struct drm_tegra_udrm_set_master_notify *)data;
	int err;

	if (args->clear != 0) {
		/* Releases reference to the previously acquired eventfd
		 * context.
		 */
		if (fpriv->efd_ctx_set_master) {
			eventfd_ctx_put(fpriv->efd_ctx_set_master);
			fpriv->efd_ctx_set_master = NULL;
		}
		return 0;
	}

	/* Fail if already acquired a reference to the eventfd context. For
	 * simplicity driver keeps track of a single eventfd context. It's
	 * unlikely that multiple threads would ever need to register
	 * set_master notification.
	 */
	if (fpriv->efd_ctx_set_master != NULL)
		return -EBUSY;

	fpriv->efd_ctx_set_master = eventfd_ctx_fdget(args->eventfd);
	if (IS_ERR(fpriv->efd_ctx_set_master)) {
		err = PTR_ERR(fpriv->efd_ctx_set_master);
		fpriv->efd_ctx_set_master = NULL;
		return err;
	}

	return 0;
}

static int tegra_udrm_send_vblank_event_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	struct drm_tegra_udrm_send_vblank_event *args =
		(struct drm_tegra_udrm_send_vblank_event *)data;
	struct drm_pending_vblank_event *e;
	int ret;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;

	/* make event */
	e->pipe = 0;
	e->base.pid = current->pid;
	e->event.base.type = args->vblank.base.type;
	e->event.base.length = sizeof(e->event);
	e->event.user_data = args->vblank.user_data;
	e->event.sequence = args->vblank.sequence;
	e->event.tv_sec = args->vblank.tv_sec;
	e->event.tv_usec = args->vblank.tv_usec;

	ret = drm_event_reserve_init(drm, file, &e->base, &e->event.base);
	if (ret) {
		kfree(e);
		return ret;
	}

	drm_send_event(drm, &e->base);

	return 0;
}

static const struct file_operations tegra_udrm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = tegra_udrm_mmap,
	.poll = drm_poll,
	.read = drm_read,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.llseek = noop_llseek,
};

static const struct drm_ioctl_desc tegra_udrm_ioctls[] = {
	/*
	 * UMD when enumerated with tegra-drm fd from a process for first
	 * time e.g,
	 *  fd = open("/dev/dri/card0");
	 *  drmGetVersion(fd);
	 *
	 * The drmGetVersion call handling in UMD will create drm_nvdc
	 * context and starts a thread to listen for close, set/drop master
	 * notifications from driver. It uses tegra_udrm_close_notify_ioctl
	 * and tegra_udrm_drop_master_notify_ioctl to register for
	 * notifications from KMD. So if DRM_AUTH is set for those ioctls
	 * they might fail if there is existing master, so don't set DRM_AUTH
	 * on those ioctls. None of the below ioctls are modifying KMS state
	 * so DRM_AUTH is not required anyway.
	 */
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_DMABUF_MMAP,
		tegra_udrm_dmabuf_mmap_ioctl, 0),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_DMABUF_DESTROY_MAPPINGS,
		tegra_udrm_dmabuf_destroy_mappings_ioctl, 0),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_CLOSE_NOTIFY,
		tegra_udrm_close_notify_ioctl, 0),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_SEND_VBLANK_EVENT,
		tegra_udrm_send_vblank_event_ioctl, 0),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_DROP_MASTER_NOTIFY,
		tegra_udrm_drop_master_notify_ioctl, 0),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_SET_MASTER_NOTIFY,
		tegra_udrm_set_master_notify_ioctl, 0),
};

static int tegra_udrm_open(struct drm_device *drm, struct drm_file *filp)
{
	struct tegra_udrm_file *fpriv;

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (!fpriv)
		return -ENOMEM;

	filp->driver_priv = fpriv;

	fpriv->efd_ctx_close = NULL;
	fpriv->efd_ctx_drop_master = NULL;
	idr_init(&fpriv->idr);

	return 0;
}

static int tegra_udrm_master_set(struct drm_device *dev,
		struct drm_file *file_priv,
		bool from_open)
{
	struct tegra_udrm_file *fpriv = file_priv->driver_priv;

	if (fpriv->efd_ctx_set_master != NULL)
		eventfd_signal(fpriv->efd_ctx_set_master, 1);

	return 0;
}

static void tegra_udrm_master_drop(struct drm_device *dev,
		struct drm_file *file_priv)
{
	struct tegra_udrm_file *fpriv = file_priv->driver_priv;

	if (fpriv->efd_ctx_drop_master) {
		eventfd_signal(fpriv->efd_ctx_drop_master, 1);
	}
}

static struct drm_driver tegra_udrm_driver = {
	/* To avoid tripping on DRM mastership, user space EGL driver opens
	 * render node (instead of primary node) and calls drmGetVersion
	 * to identify tegra-udrm driver.
	 *
	 * UMD (libdrm_nvdc) should handle which drm APIs to support on
	 * render node. There are are no known applications which will get
	 * affected due to enumeration of render node, it is only used by
	 * user space graphics drivers to get version name.
	 */
	.driver_features   = DRIVER_RENDER,
	.open              = tegra_udrm_open,
	.preclose          = tegra_udrm_preclose,
	.ioctls            = tegra_udrm_ioctls,
	.num_ioctls        = ARRAY_SIZE(tegra_udrm_ioctls),
	.fops              = &tegra_udrm_fops,

	.master_drop       = tegra_udrm_master_drop,
	.master_set        = tegra_udrm_master_set,

	.name   = DRIVER_NAME,
	.desc   = DRIVER_DESC,
	.date   = DRIVER_DATE,
	.major  = DRIVER_MAJOR,
	.minor  = DRIVER_MINOR,
};

static int tegra_udrm_load(struct drm_device *drm)
{
	struct platform_device *pdev = to_platform_device(drm->dev);
	struct tegra_udrm_private *private;

	private = devm_kzalloc(drm->dev, sizeof(*private), GFP_KERNEL);
	if (private == NULL)
		return -ENOMEM;

	drm->dev_private = private;

	platform_set_drvdata(pdev, drm);

	return 0;
}

static int tegra_udrm_unload(struct drm_device *drm)
{
	drm->dev_private = NULL;

	return 0;
}

static int tegra_udrm_probe(struct platform_device *pdev)
{
	struct drm_driver *driver = &tegra_udrm_driver;
	struct drm_device *drm;
	int ret;

	drm = drm_dev_alloc(driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	ret = tegra_udrm_load(drm);
	if (ret)
		goto err_unref;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unload;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n", driver->name,
		driver->major, driver->minor, driver->patchlevel,
		driver->date, drm->primary->index);

	return 0;

err_unload:
	tegra_udrm_unload(drm);

err_unref:
	drm_dev_unref(drm);

	return ret;
}

static int tegra_udrm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	tegra_udrm_unload(drm);
	drm_dev_unref(drm);

	return 0;
}

static const struct of_device_id tegra_udrm_of_table[] = {
	{.compatible = "nvidia,tegra-udrm"},
	{}
};

MODULE_DEVICE_TABLE(of, tegra_udrm_of_table);

MODULE_ALIAS("platform:tegra_udrm");
static struct platform_driver tegra_udrm_platform_driver = {
	.probe = tegra_udrm_probe,
	.remove = tegra_udrm_remove,
	.driver = {
		.name = "tegra_udrm",
		.of_match_table = tegra_udrm_of_table,
	},
};

static int __init tegra_udrm_init(void)
{
	if (!tegra_udrm_modeset_module_param)
		return -EINVAL;

	platform_driver_register(&tegra_udrm_platform_driver);

	return 0;
}

static void __exit tegra_udrm_exit(void)
{
	platform_driver_unregister(&tegra_udrm_platform_driver);
}

module_init(tegra_udrm_init);
module_exit(tegra_udrm_exit);

MODULE_AUTHOR("NVIDIA CORPORATION");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
