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

#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <unit/io.h>
#include <unit/core.h>
#include <unit/unit.h>
#include <unit/results.h>

/*
 * Mutex to ensure core_add_test_record() is thread safe.
 */
pthread_mutex_t mutex_results = PTHREAD_MUTEX_INITIALIZER;

static int __init_results(struct unit_fw *fw)
{
	struct unit_results *results;

	if (fw->results != NULL)
		return 0;

	results = malloc(sizeof(*results));
	if (results == NULL)
		return -1;

	memset(results, 0, sizeof(*results));

	fw->results = results;

	return 0;
}

static void add_record(struct unit_test_list *list,
		       struct unit_test_record *tr)
{
	/*
	 * First entry.
	 */
	if (list->head == NULL) {
		list->head = tr;
		list->last = tr;
		return;
	}

	/*
	 * Add to the end of the list and update the pointer to the last entry
	 * in the list. This gives us O(1) add time.
	 */
	list->last->next = tr;
	list->last = tr;
}

int core_add_test_record(struct unit_fw *fw,
			 struct unit_module *mod,
			 struct unit_module_test *test,
			 bool success)
{
	struct unit_test_record *tr;
	int err = 0;

	pthread_mutex_lock(&mutex_results);
	/*
	 * Does nothing if results are already inited.
	 */
	if (__init_results(fw) != 0) {
		err = -1;
		goto done;
	}

	tr = malloc(sizeof(*tr));
	if (tr == NULL) {
		err = -1;
		goto done;
	}

	tr->mod = mod;
	tr->test = test;
	tr->status = success;
	tr->next = NULL;

	if (success)
		add_record(&fw->results->passing, tr);
	else
		add_record(&fw->results->failing, tr);

	fw->results->nr_tests += 1;
	if (success)
		fw->results->nr_passing += 1;

done:
	pthread_mutex_unlock(&mutex_results);
	return err;
}

void core_print_test_status(struct unit_fw *fw)
{
	struct unit_test_list *failing_tests = &fw->results->failing;
	struct unit_test_record *rec;

	/*
	 * Print stats for the tests.
	 */
	core_msg(fw, "\n");
	core_msg(fw, "Test results:\n");
	core_msg(fw, "-------------\n");
	core_msg(fw, "\n");
	core_msg(fw, "  Passing: %d\n", fw->results->nr_passing);
	core_msg(fw, "  Failing: %d\n",
		 fw->results->nr_tests - fw->results->nr_passing);
	core_msg(fw, "  Total:   %d\n", fw->results->nr_tests);
	core_msg(fw, "\n");
	core_msg(fw, "Failing tests:\n");
	core_msg(fw, "\n");

	for_record_in_test_list(failing_tests, rec) {
		core_msg(fw, "  %s.%s\n",
			 rec->mod->name,
			 rec->test->name);
	}
}
