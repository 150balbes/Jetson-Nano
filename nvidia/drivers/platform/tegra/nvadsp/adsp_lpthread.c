/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <asm/segment.h>
#include <asm/uaccess.h>

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/tegra_nvadsp.h>

#include "dev.h"

#define RW_MODE (S_IWUSR | S_IRUGO)

enum adsp_lpthread_state {
	ADSP_LPTHREAD_STOP,
	ADSP_LPTHREAD_START,
	ADSP_LPTHREAD_PAUSE,
};

struct adsp_lpthread_shared_state_t {
	uint16_t mbox_id;
};

enum adsp_lpthread_mbx_cmd {
	ADSP_LPTHREAD_CMD_RESUME = 0,
	ADSP_LPTHREAD_CMD_PAUSE,
	ADSP_LPTHREAD_CMD_CLOSE,
};

struct adsp_lpthread {
	bool lpthread_initialized;
	bool adsp_os_suspended;
	bool lpthread_paused;
	bool lpthread_resumed;
	bool lpthread_closed;
	nvadsp_app_handle_t app_handle;
	nvadsp_app_info_t *app_info;
};

static struct adsp_lpthread lpthread_obj;
static struct adsp_lpthread *lpthread;

static struct nvadsp_mbox mbox;
static struct adsp_lpthread_shared_state_t *adsp_lpthread;

/* Initialize adsp_lpthread app and mailbox */
int adsp_lpthread_init(bool is_adsp_suspended)
{
	nvadsp_app_handle_t handle;
	nvadsp_app_info_t *app_info;
	int ret;

	handle = nvadsp_app_load("adsp_lpthread", "adsp_lpthread.elf");
	if (!handle)
		return -1;

	app_info = nvadsp_app_init(handle, NULL);
	if (!app_info) {
		pr_err("unable to init app adsp_lpthread\n");
		return -1;
	}

	ret = nvadsp_app_start(app_info);
	if (ret) {
		pr_err("unable to start app adsp_lpthread\n");
		return -1;
	}

	lpthread->app_info = app_info;
	lpthread->app_handle = handle;

	adsp_lpthread =
		(struct adsp_lpthread_shared_state_t *)app_info->mem.shared;
	ret = nvadsp_mbox_open(&mbox, &adsp_lpthread->mbox_id,
		"adsp_lpthread", NULL, NULL);
	if (ret) {
		pr_err("Failed to open mbox %d for adsp_lpthread app",
			adsp_lpthread->mbox_id);
		return -1;
	}

	/* Start timer is adsp is not in suspended state */
	if (!is_adsp_suspended) {
		ret = adsp_lpthread_resume();
		return ret;
	}

	return 0;
}

int adsp_lpthread_resume(void)
{
	int ret;

	ret = nvadsp_mbox_send(&mbox, ADSP_LPTHREAD_CMD_RESUME,
		NVADSP_MBOX_SMSG, 0, 0);
	if (ret)
		pr_err("%s: nvadsp_mbox_send() failed: %d, ret = %d\n",
			__func__, adsp_lpthread->mbox_id, ret);

	return ret;
}

int adsp_lpthread_pause(void)
{
	int ret;

	ret = nvadsp_mbox_send(&mbox, ADSP_LPTHREAD_CMD_PAUSE,
		NVADSP_MBOX_SMSG, 0, 0);
	if (ret)
		pr_err("%s: nvadsp_mbox_send() failed: %d, ret = %d\n",
			__func__, adsp_lpthread->mbox_id, ret);

	return ret;
}

int adsp_lpthread_uninit(void)
{
	int ret;

	ret = nvadsp_mbox_send(&mbox, ADSP_LPTHREAD_CMD_CLOSE,
		NVADSP_MBOX_SMSG, 0, 0);
	if (ret)
		pr_err("%s: nvadsp_mbox_send() failed: %d, ret = %d\n",
			__func__, adsp_lpthread->mbox_id, ret);

	nvadsp_mbox_close(&mbox);

	nvadsp_exit_app((nvadsp_app_info_t *)lpthread->app_info, false);

	nvadsp_app_unload((const void *)lpthread->app_handle);

	return ret;
}

int adsp_usage_set(unsigned int  val)
{
	int ret = 0;

	switch (val) {

	case ADSP_LPTHREAD_START:
		if (lpthread->lpthread_initialized &&
				lpthread->lpthread_resumed) {
			pr_info("ADSP Usage App already running\n");
			break;
		}

		if (!lpthread->lpthread_initialized) {
			ret = adsp_lpthread_init(lpthread->adsp_os_suspended);
			pr_info("Initializing lpthread\n");
			lpthread->lpthread_initialized = true;
		} else {
			ret = adsp_lpthread_resume();
			pr_info("Resuming lpthread\n");
		}
		lpthread->lpthread_resumed = true;
		lpthread->lpthread_paused = false;
		lpthread->lpthread_closed = false;
		break;

	case ADSP_LPTHREAD_PAUSE:
		if (!lpthread->lpthread_initialized) {
			pr_info("ADSP Usage App not initialized\n");
			break;
		}
		pr_info("Pausing lpthread\n");
		ret = adsp_lpthread_pause();
		lpthread->lpthread_resumed = false;
		lpthread->lpthread_paused = true;
		lpthread->lpthread_closed = false;
		break;

	case ADSP_LPTHREAD_STOP:
		if (!lpthread->lpthread_initialized) {
			pr_info("ADSP Usage App not initialized\n");
			break;
		}
		pr_info("Exiting lpthread\n");
		ret = adsp_lpthread_uninit();
		lpthread->lpthread_resumed = false;
		lpthread->lpthread_paused = false;
		lpthread->lpthread_closed = true;
		lpthread->lpthread_initialized = false;
		break;

	default:
		pr_err("ADSP Usage App: Invalid input\n");
		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(adsp_usage_set);
unsigned int adsp_usage_get(void)
{
	if (lpthread->lpthread_initialized && lpthread->lpthread_resumed)
		return ADSP_LPTHREAD_START;

	if (lpthread->lpthread_initialized && lpthread->lpthread_paused)
		return ADSP_LPTHREAD_PAUSE;

	return ADSP_LPTHREAD_STOP;
}
EXPORT_SYMBOL(adsp_usage_get);

int adsp_lpthread_entry(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	lpthread = &lpthread_obj;

	drv->lpthread_initialized = true;
	lpthread->adsp_os_suspended = false;

	return 0;
}

int adsp_lpthread_exit(struct platform_device *pdev)
{
	status_t ret = 0;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	if (!drv->lpthread_initialized)
		ret =  -EINVAL;
	drv->lpthread_initialized = false;

	return ret;
}

int adsp_lpthread_set_suspend(bool is_suspended)
{
	lpthread->adsp_os_suspended = is_suspended;
	return 0;
}

int adsp_lpthread_get_state(void)
{
	if (lpthread->lpthread_initialized && lpthread->lpthread_resumed)
		return 1;
	else
		return 0;
}
