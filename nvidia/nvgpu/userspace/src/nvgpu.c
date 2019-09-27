/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <dlfcn.h>
#include <stdlib.h>

#include <unit/io.h>
#include <unit/core.h>

/*
 * Load libnvgpu-drv.so. This is done with dlopen() since this will make
 * resolving addresses into symbols easier in the future.
 *
 * Also, this makes people think carefully about what functions to call in
 * nvgpu-drv from the unit test FW. The interaction should really be limited
 * and doing explicit name lookups is a good way to prevent too much coupling.
 */
int core_load_nvgpu(struct unit_fw *fw)
{
	const char *msg;

	/*
	 * Specify a GLOBAL binding so that subsequently loaded unit tests see
	 * the nvgpu-drv library. They will of course need it (and will access
	 * it directly). I.e they will link against nvgpu-drv and this should
	 * satisfy that linkage.
	 */
	fw->nvgpu_so = dlopen("libnvgpu-drv.so", RTLD_NOW | RTLD_GLOBAL);

	if (fw->nvgpu_so == NULL) {
		msg = dlerror();
		core_err(fw, "Failed to load nvgpu-drv: %s\n", msg);
		return -1;
	}

	/*
	 * We directly check the value of the returned symbol for these
	 * functions against NULL because if it is NULL then something is
	 * terribly wrong.
	 */

	fw->nvgpu.nvgpu_posix_probe = dlsym(fw->nvgpu_so,
					    "nvgpu_posix_probe");
	if (fw->nvgpu.nvgpu_posix_probe == NULL) {
		msg = dlerror();
		core_err(fw, "Failed to resolve nvgpu_posix_probe: %s\n", msg);
		return -1;
	}

	fw->nvgpu.nvgpu_posix_cleanup = dlsym(fw->nvgpu_so,
					      "nvgpu_posix_cleanup");
	if (fw->nvgpu.nvgpu_posix_cleanup == NULL) {
		msg = dlerror();
		core_err(fw, "Failed to resolve nvgpu_posix_cleanup: %s\n", msg);
		return -1;
	}

	return 0;
}
