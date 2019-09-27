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

#ifndef __UNIT_RESULTS_H__
#define __UNIT_RESULTS_H__

/*
 * Keep track of the the results of a set of unit tests. This is effectively
 * just a single linked list of records for each test.
 */

struct unit_test_record {
	/*
	 * Let's us determine the name of the test.
	 */
	struct unit_module *mod;
	struct unit_module_test *test;

	/*
	 * True for pass, false for fail.
	 */
	bool status;

	struct unit_test_record *next;
};

struct unit_test_list {
	struct unit_test_record *head;
	struct unit_test_record *last;
};

struct unit_results {
	struct unit_test_list passing;
	struct unit_test_list failing;

	int nr_tests;
	int nr_passing;
};

#define for_record_in_test_list(__test_list, __test)	\
	for ((__test) = (__test_list)->head;		\
	     (__test) != NULL;				\
	     (__test) = (__test)->next)

int core_add_test_record(struct unit_fw *fw,
			 struct unit_module *mod,
			 struct unit_module_test *test,
			 bool success);
void core_print_test_status(struct unit_fw *fw);

#endif
