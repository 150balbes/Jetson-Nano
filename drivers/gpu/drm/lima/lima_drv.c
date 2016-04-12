// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright 2017-2018 Qiang Yu <yuq825@gmail.com> */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/log2.h>
#include <drm/drm_prime.h>
#include <drm/lima_drm.h>

#include "lima_drv.h"
#include "lima_gem.h"
#include "lima_gem_prime.h"
#include "lima_vm.h"

int lima_sched_timeout_ms = 0;
int lima_sched_max_tasks = 32;

MODULE_PARM_DESC(sched_timeout_ms, "task run timeout in ms (0 = no timeout (default))");
module_param_named(sched_timeout_ms, lima_sched_timeout_ms, int, 0444);

MODULE_PARM_DESC(sched_max_tasks, "max queued task num in a context (default 32)");
module_param_named(sched_max_tasks, lima_sched_max_tasks, int, 0444);

static int lima_ioctl_info(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_info *info = data;
	struct lima_device *ldev = to_lima_dev(dev);

	switch (ldev->id) {
	case lima_gpu_mali400:
		info->gpu_id = LIMA_INFO_GPU_MALI400;
		break;
	case lima_gpu_mali450:
		info->gpu_id = LIMA_INFO_GPU_MALI450;
		break;
	default:
		return -ENODEV;
	}
	info->num_pp = ldev->pipe[lima_pipe_pp].num_processor;
	info->valid = 0;
	return 0;
}

static int lima_ioctl_gem_create(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_gem_create *args = data;

	if (args->flags)
		return -EINVAL;

	if (args->size == 0)
		return -EINVAL;

	return lima_gem_create_handle(dev, file, args->size, args->flags, &args->handle);
}

static int lima_ioctl_gem_info(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_gem_info *args = data;

	return lima_gem_get_info(file, args->handle, &args->va, &args->offset);
}

static int lima_ioctl_gem_submit(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_gem_submit *args = data;
	struct lima_device *ldev = to_lima_dev(dev);
	struct lima_drm_priv *priv = file->driver_priv;
	struct drm_lima_gem_submit_bo *bos;
	struct lima_sched_pipe *pipe;
	struct lima_sched_task *task;
	struct lima_ctx *ctx;
	struct lima_submit submit = {0};
	size_t size;
	int err = 0;

	if (args->pipe >= lima_pipe_num || args->nr_bos == 0)
		return -EINVAL;

	if (args->flags & ~(LIMA_SUBMIT_FLAG_EXPLICIT_FENCE))
		return -EINVAL;

	pipe = ldev->pipe + args->pipe;
	if (args->frame_size != pipe->frame_size)
		return -EINVAL;

	bos = kvcalloc(args->nr_bos, sizeof(*submit.bos) + sizeof(*submit.lbos), GFP_KERNEL);
	if (!bos)
		return -ENOMEM;

	size = args->nr_bos * sizeof(*submit.bos);
	if (copy_from_user(bos, u64_to_user_ptr(args->bos), size)) {
		err = -EFAULT;
		goto out0;
	}

	task = kmem_cache_zalloc(pipe->task_slab, GFP_KERNEL);
	if (!task) {
		err = -ENOMEM;
		goto out0;
	}

	task->frame = task + 1;
	if (copy_from_user(task->frame, u64_to_user_ptr(args->frame), args->frame_size)) {
		err = -EFAULT;
		goto out1;
	}

	err = pipe->task_validate(pipe, task);
	if (err)
		goto out1;

	ctx = lima_ctx_get(&priv->ctx_mgr, args->ctx);
	if (!ctx) {
		err = -ENOENT;
		goto out1;
	}

	submit.pipe = args->pipe;
	submit.bos = bos;
	submit.lbos = (void *)bos + size;
	submit.nr_bos = args->nr_bos;
	submit.task = task;
	submit.ctx = ctx;
	submit.flags = args->flags;
	submit.in_sync[0] = args->in_sync[0];
	submit.in_sync[1] = args->in_sync[1];
	submit.out_sync = args->out_sync;

	err = lima_gem_submit(file, &submit);

	lima_ctx_put(ctx);
out1:
	if (err)
		kmem_cache_free(pipe->task_slab, task);
out0:
	kvfree(bos);
	return err;
}

static int lima_ioctl_gem_wait(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_gem_wait *args = data;

	if (args->op & ~(LIMA_GEM_WAIT_READ|LIMA_GEM_WAIT_WRITE))
		return -EINVAL;

	return lima_gem_wait(file, args->handle, args->op, args->timeout_ns);
}

static int lima_ioctl_ctx(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_lima_ctx *args = data;
	struct lima_drm_priv *priv = file->driver_priv;
	struct lima_device *ldev = to_lima_dev(dev);

	if (args->op == LIMA_CTX_OP_CREATE)
		return lima_ctx_create(ldev, &priv->ctx_mgr, &args->id);
	else if (args->op == LIMA_CTX_OP_FREE)
		return lima_ctx_free(&priv->ctx_mgr, args->id);

	return -EINVAL;
}

static int lima_drm_driver_open(struct drm_device *dev, struct drm_file *file)
{
	int err;
	struct lima_drm_priv *priv;
	struct lima_device *ldev = to_lima_dev(dev);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->vm = lima_vm_create(ldev);
	if (!priv->vm) {
		err = -ENOMEM;
		goto err_out0;
	}

	lima_ctx_mgr_init(&priv->ctx_mgr);

	file->driver_priv = priv;
	return 0;

err_out0:
	kfree(priv);
	return err;
}

static void lima_drm_driver_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct lima_drm_priv *priv = file->driver_priv;

	lima_ctx_mgr_fini(&priv->ctx_mgr);
	lima_vm_put(priv->vm);
	kfree(priv);
}

static const struct drm_ioctl_desc lima_drm_driver_ioctls[] = {
	DRM_IOCTL_DEF_DRV(LIMA_INFO, lima_ioctl_info, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(LIMA_GEM_CREATE, lima_ioctl_gem_create, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(LIMA_GEM_INFO, lima_ioctl_gem_info, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(LIMA_GEM_SUBMIT, lima_ioctl_gem_submit, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(LIMA_GEM_WAIT, lima_ioctl_gem_wait, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(LIMA_CTX, lima_ioctl_ctx, DRM_AUTH|DRM_RENDER_ALLOW),
};

static const struct file_operations lima_drm_driver_fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.mmap               = lima_gem_mmap,
};

static struct drm_driver lima_drm_driver = {
	.driver_features    = DRIVER_RENDER | DRIVER_GEM | DRIVER_PRIME | DRIVER_SYNCOBJ,
	.open               = lima_drm_driver_open,
	.postclose          = lima_drm_driver_postclose,
	.ioctls             = lima_drm_driver_ioctls,
	.num_ioctls         = ARRAY_SIZE(lima_drm_driver_ioctls),
	.fops               = &lima_drm_driver_fops,
	.gem_free_object_unlocked = lima_gem_free_object,
	.gem_open_object    = lima_gem_object_open,
	.gem_close_object   = lima_gem_object_close,
	.gem_vm_ops         = &lima_gem_vm_ops,
	.name               = "lima",
	.desc               = "lima DRM",
	.date               = "20170325",
	.major              = 1,
	.minor              = 0,
	.patchlevel         = 0,

	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table = lima_gem_prime_import_sg_table,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.gem_prime_get_sg_table = lima_gem_prime_get_sg_table,
	.gem_prime_mmap = lima_gem_prime_mmap,
};

static int lima_pdev_probe(struct platform_device *pdev)
{
	struct lima_device *ldev;
	struct drm_device *ddev;
	int err;

	ldev = devm_kzalloc(&pdev->dev, sizeof(*ldev), GFP_KERNEL);
	if (!ldev)
		return -ENOMEM;

	ldev->pdev = pdev;
	ldev->dev = &pdev->dev;
	ldev->id = (enum lima_gpu_id)of_device_get_match_data(&pdev->dev);

	platform_set_drvdata(pdev, ldev);

	/* Allocate and initialize the DRM device. */
	ddev = drm_dev_alloc(&lima_drm_driver, &pdev->dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	ddev->dev_private = ldev;
	ldev->ddev = ddev;

	err = lima_device_init(ldev);
	if (err) {
		dev_err(&pdev->dev, "Fatal error during GPU init\n");
		goto err_out0;
	}

	/*
	 * Register the DRM device with the core and the connectors with
	 * sysfs.
	 */
	err = drm_dev_register(ddev, 0);
	if (err < 0)
		goto err_out1;

	return 0;

err_out1:
	lima_device_fini(ldev);
err_out0:
	drm_dev_put(ddev);
	return err;
}

static int lima_pdev_remove(struct platform_device *pdev)
{
	struct lima_device *ldev = platform_get_drvdata(pdev);
	struct drm_device *ddev = ldev->ddev;

	drm_dev_unregister(ddev);
	lima_device_fini(ldev);
	drm_dev_put(ddev);
	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "arm,mali-400", .data = (void *)lima_gpu_mali400 },
	{ .compatible = "arm,mali-450", .data = (void *)lima_gpu_mali450 },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver lima_platform_driver = {
	.probe      = lima_pdev_probe,
	.remove     = lima_pdev_remove,
	.driver     = {
		.name   = "lima",
		.of_match_table = dt_match,
	},
};

static void lima_check_module_param(void)
{
	if (lima_sched_max_tasks < 4)
		lima_sched_max_tasks = 4;
	else
		lima_sched_max_tasks = roundup_pow_of_two(lima_sched_max_tasks);
}

static int __init lima_init(void)
{
	int ret;

	lima_check_module_param();
	ret = lima_sched_slab_init();
	if (ret)
		return ret;

	ret = platform_driver_register(&lima_platform_driver);
	if (ret)
		lima_sched_slab_fini();

	return ret;
}
module_init(lima_init);

static void __exit lima_exit(void)
{
	platform_driver_unregister(&lima_platform_driver);
	lima_sched_slab_fini();
}
module_exit(lima_exit);

MODULE_AUTHOR("Lima Project Developers");
MODULE_DESCRIPTION("Lima DRM Driver");
MODULE_LICENSE("GPL v2");
