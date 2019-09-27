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

#ifndef __UNIT_UNIT_H__
#define __UNIT_UNIT_H__

#include <pthread.h>

struct gk20a;

struct unit_module;
typedef int (*module_test_fn)(struct unit_module *m,
			      struct gk20a *g, void *args);

#define UNIT_SUCCESS		0
#define UNIT_FAIL		-1

struct unit_module_test {
	/*
	 * Name of the test.
	 */
	const char *name;

	/*
	 * Function to call to execute the test.
	 */
	module_test_fn fn;

	/*
	 * A void pointer to arbitrary arguments. Lets the same unit test
	 * function perform multiple tests. This gets passed into the
	 * module_test_fn as @args.
	 */
	void *args;
};

/*
 * Interface to the unit test framework module loader. Each unit test module
 * will have exactly one of these.
 */
struct unit_module {
	/*
	 * Name of the module.
	 */
	const char			*name;

	/*
	 * NULL terminated list of tests within the module.
	 */
	struct unit_module_test		*tests;
	unsigned long			 nr_tests;

	/*
	 * Run priority. Currently 3 defined:
	 *
	 *   UNIT_PRIO_SELF_TEST
	 *   UNIT_PRIO_POSIX_TEST
	 *   UNIT_PRIO_NVGPU_TEST
	 *
	 * These let us run environment and POSIX API wrapper tests before the
	 * rest of the unit tests run.
	 */
	unsigned int				 prio;

	/*
	 * For the core FW to use. Not for modules!!!
	 */
	void				*lib_handle;
	struct unit_fw			*fw;

	pthread_t thread;
};

/*
 * Zero is the higest priority. Increasing the prio value decreases priority to
 * run.
 */
#define UNIT_PRIO_SELF_TEST		0U
#define UNIT_PRIO_POSIX_TEST		50U
#define UNIT_PRIO_NVGPU_TEST		100U

#define UNIT_MODULE(__name, __tests, __prio)				\
	struct unit_module __unit_module__ = {				\
		.name = #__name,					\
		.tests = __tests,					\
		.nr_tests = (sizeof(__tests) /				\
			     sizeof(struct unit_module_test)),		\
		.prio = __prio,						\
		.lib_handle = NULL,					\
		.fw = NULL,						\
	}

#define UNIT_TEST(__name, __fn, __args)					\
	{								\
		.name = #__name,					\
		.fn = __fn,						\
		.args = __args,						\
	}

#define unit_return_fail(m, msg, ...)					\
	do {								\
		unit_err(m, "%s():%d " msg,				\
			 __func__, __LINE__, ##__VA_ARGS__);		\
		return UNIT_FAIL;					\
	} while (0)

#endif
