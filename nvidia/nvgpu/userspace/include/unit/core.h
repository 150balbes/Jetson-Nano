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

#ifndef __UNIT_CORE_H__
#define __UNIT_CORE_H__

struct unit_fw_args;
struct unit_modules;
struct unit_results;

struct gk20a;

/*
 * The core unit testing framework data structure. Keeps track of global state
 * for the unit test app.
 */
struct unit_fw {
	struct unit_fw_args	 *args;

	struct unit_module	**modules;

	struct unit_results	 *results;

	/*
	 * nvgpu-drv interface. Currently the only two directly referenced
	 * functions are:
	 *
	 *   nvgpu_posix_probe()
	 *   nvgpu_posix_cleanup()
	 *
	 * There will get populated so that we can call them before/after each
	 * module.
	 */
	void			 *nvgpu_so;
	struct {
		struct gk20a	*(*nvgpu_posix_probe)(void);
		void		 (*nvgpu_posix_cleanup)(struct gk20a *g);
	} nvgpu;
};

int core_load_nvgpu(struct unit_fw *fw);
int core_exec(struct unit_fw *fw);

#endif
