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

#include <errno.h>
#include <dlfcn.h>
#include <dirent.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>

#include <unit/core.h>
#include <unit/io.h>
#include <unit/args.h>
#include <unit/unit.h>
#include <unit/module.h>

static int check_module(struct unit_fw *fw, struct unit_module *mod)
{
	unsigned int i;

	/*
	 * Make sure this module has reasonable data.
	 */
	if (mod->name == NULL) {
		core_err(fw, "Unnamed module!");
		return -1;
	}

	if (mod->tests == NULL || mod->nr_tests == 0) {
		core_err(fw, "%s: Empty module!\n", mod->name);
		return -1;
	}

	for (i = 0; i < mod->nr_tests; i++) {
		struct unit_module_test *test = &mod->tests[i];

		if (test->name == NULL) {
			core_err(fw, "%s: Unnamed test\n", mod->name);
			return -1;
		}

		if (test->fn == NULL) {
			core_err(fw, "%s: Test %s missing function \n",
				 mod->name, test->name);
			return -1;
		}
	}

	return 0;
}

static struct unit_module *load_one_module(struct unit_fw *fw,
					   struct dirent *dent)
{
	void *lib_handle;
	struct unit_module *mod;

	core_vbs(fw, 1, "Loading: %s\n", dent->d_name);

	lib_handle = dlopen(dent->d_name, RTLD_NOW);
	if (lib_handle == NULL) {
		core_err(fw, "Failed to load %s: %s\n",
			 dent->d_name, dlerror());
		return NULL;
	}

	mod = dlsym(lib_handle, "__unit_module__");
	if (mod == NULL) {
		core_vbs(fw, 1,
			 "Failed to resolve __unit_module__ in %s: %s\n",
			 dent->d_name, dlerror());
		return NULL;
	}

	mod->lib_handle = lib_handle;
	mod->fw = fw;

	core_vbs(fw, 1, "  '%s' contains %lu tests\n", mod->name, mod->nr_tests);

	return mod;
}

static int cmp_module_prio(const void *__mod_a, const void *__mod_b)
{
	const struct unit_module * const *mod_a = __mod_a;
	const struct unit_module * const *mod_b = __mod_b;

	return (*mod_a)->prio > (*mod_b)->prio;
}

/*
 * Sort the module list according to prio.
 */
static void sort_modules_by_prio(struct unit_module **modules, int nr)
{
	qsort(modules, (size_t)nr, sizeof(struct unit_module *),
	      cmp_module_prio);
}

/*
 * Load all the modules we can from the module load path. Return the list of
 * loaded module as an array of pointers to modules. The returned list of
 * modules is NULL terminated.
 */
struct unit_module **core_load_modules(struct unit_fw *fw)
{
	int nr_modules = 0, i;
	DIR *load_dir;
	struct dirent *ent;
	const char *load_path = args(fw)->unit_load_path;
	struct unit_module **modules, *mod;

	core_vbs(fw, 1, "Loading modules from %s\n", load_path);

	/*
	 * Open and count the number of files in the dir.
	 */
	load_dir = opendir(load_path);
	if (!load_dir) {
		core_err(fw, "%s: Unable to open dir (%s)\n",
			 load_path, strerror(errno));
		return NULL;
	}

	while (readdir(load_dir) != NULL)
		nr_modules += 1;

	/* '.' and '..' should be skipped. */
	nr_modules -= 2;

	/*
	 * Now allocate necessary space for storing pointers to the modules and
	 * load the modules. +1 for the last NULL entry.
	 */
	modules = malloc(sizeof(*modules) * (nr_modules + 1));
	if (!modules) {
		core_err(fw, "Out of mem! (huh?)\n");
		goto err;
	}

	rewinddir(load_dir);
	i = 0;
	while ((ent = readdir(load_dir)) != NULL) {
		if (strcmp(".", ent->d_name) == 0 ||
		    strcmp("..", ent->d_name) == 0)
			continue;

		mod = load_one_module(fw, ent);
		if (mod == NULL)
			continue;

		if (check_module(fw, mod) != 0)
			continue;

		modules[i] = mod;
		i++;
	}

	modules[i] = NULL;

	sort_modules_by_prio(modules, i);

	return modules;

err:
	closedir(load_dir);
	return NULL;
}
