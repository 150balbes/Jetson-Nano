/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_FIRMWARE_H
#define NVGPU_FIRMWARE_H

#include <nvgpu/types.h>

struct gk20a;

#define NVGPU_REQUEST_FIRMWARE_NO_WARN		(1UL << 0)
#define NVGPU_REQUEST_FIRMWARE_NO_SOC		(1UL << 1)

struct nvgpu_firmware {
	u8 *data;
	size_t size;
};

/**
 * nvgpu_request_firmware - load a firmware blob from filesystem.
 *
 * @g		The GPU driver struct for device to load firmware for
 * @fw_name	The base name of the firmware file.
 * @flags	Flags for loading;
 *
 * 		NVGPU_REQUEST_FIRMWARE_NO_WARN: Do not display warning on
 * 		failed load.
 *
 * 		NVGPU_REQUEST_FIRMWARE_NO_SOC: Do not attempt loading from
 * 		path <SOC_NAME>.
 *
 * nvgpu_request_firmware() will load firmware from:
 *
 * <system firmware load path>/<GPU name>/<fw_name>
 *
 * If that fails and NO_SOC is not enabled, it'll try next from:
 *
 * <system firmware load path>/<SOC name>/<fw_name>
 *
 * It'll allocate a nvgpu_firmware structure and initializes it and returns
 * it to caller.
 */
struct nvgpu_firmware *nvgpu_request_firmware(struct gk20a *g,
					      const char *fw_name,
					      int flags);

/**
 * nvgpu_release_firmware - free firmware and associated nvgpu_firmware blob
 *
 * @g		The GPU driver struct for device to free firmware for
 * @fw		The firmware to free. fw blob will also be freed.
 */
void nvgpu_release_firmware(struct gk20a *g, struct nvgpu_firmware *fw);

#endif /* NVGPU_FIRMWARE_H */
