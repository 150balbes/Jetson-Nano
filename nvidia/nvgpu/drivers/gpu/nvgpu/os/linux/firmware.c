/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/firmware.h>

#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/firmware.h>
#include <nvgpu/gk20a.h>

#include "platform_gk20a.h"
#include "os_linux.h"

static const struct firmware *do_request_firmware(struct device *dev,
		const char *prefix, const char *fw_name, int flags)
{
	const struct firmware *fw;
	char *fw_path = NULL;
	int path_len, err;

	if (prefix) {
		path_len = strlen(prefix) + strlen(fw_name);
		path_len += 2; /* for the path separator and zero terminator*/

		fw_path = nvgpu_kzalloc(get_gk20a(dev),
					sizeof(*fw_path) * path_len);
		if (!fw_path)
			return NULL;

		sprintf(fw_path, "%s/%s", prefix, fw_name);
		fw_name = fw_path;
	}

	if (flags & NVGPU_REQUEST_FIRMWARE_NO_WARN)
		err = request_firmware_direct(&fw, fw_name, dev);
	else
		err = request_firmware(&fw, fw_name, dev);

	nvgpu_kfree(get_gk20a(dev), fw_path);
	if (err)
		return NULL;
	return fw;
}

/* This is a simple wrapper around request_firmware that takes 'fw_name' and
 * applies an IP specific relative path prefix to it. The caller is
 * responsible for calling nvgpu_release_firmware later. */
struct nvgpu_firmware *nvgpu_request_firmware(struct gk20a *g,
					      const char *fw_name,
					      int flags)
{
	struct device *dev = dev_from_gk20a(g);
	struct nvgpu_firmware *fw;
	const struct firmware *linux_fw;

	/* current->fs is NULL when calling from SYS_EXIT.
	   Add a check here to prevent crash in request_firmware */
	if (!current->fs || !fw_name)
		return NULL;

	fw = nvgpu_kzalloc(g, sizeof(*fw));
	if (!fw)
		return NULL;

	linux_fw = do_request_firmware(dev, g->name, fw_name, flags);

#ifdef CONFIG_TEGRA_GK20A
	/* TO BE REMOVED - Support loading from legacy SOC specific path. */
	if (!linux_fw && !(flags & NVGPU_REQUEST_FIRMWARE_NO_SOC)) {
		struct gk20a_platform *platform = gk20a_get_platform(dev);
		linux_fw = do_request_firmware(dev,
				platform->soc_name, fw_name, flags);
	}
#endif

	if (!linux_fw)
		goto err;

	fw->data = nvgpu_kmalloc(g, linux_fw->size);
	if (!fw->data)
		goto err_release;

	memcpy(fw->data, linux_fw->data, linux_fw->size);
	fw->size = linux_fw->size;

	release_firmware(linux_fw);

	return fw;

err_release:
	release_firmware(linux_fw);
err:
	nvgpu_kfree(g, fw);
	return NULL;
}

void nvgpu_release_firmware(struct gk20a *g, struct nvgpu_firmware *fw)
{
	if(!fw)
		return;

	nvgpu_kfree(g, fw->data);
	nvgpu_kfree(g, fw);
}
