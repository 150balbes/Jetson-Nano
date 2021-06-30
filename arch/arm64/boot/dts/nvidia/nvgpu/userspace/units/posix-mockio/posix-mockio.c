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

#include <unit/io.h>
#include <unit/unit.h>

#include <nvgpu/io.h>
#include <nvgpu/io_usermode.h>
#include <nvgpu/posix/io.h>

struct writel_test_args {
	const char *name;
	void (*fn)(struct gk20a *, u32, u32);
};

struct readl_test_args {
	const char *name;
	u32 (*fn)(struct gk20a *, u32);
};

/**
 * This is both a very simple functional test and documentation for how to use
 * the core IO mocking API.
 *
 * The testing is very simple: just generate a bunch of reads and writes and
 * check that they make it to the call back functions correctly.
 */

static struct nvgpu_reg_access mockio_access;

/*
 * List of writes to test with.
 */
static struct nvgpu_reg_access test_access_list[] = {
	{ 0U,		0U },
	{ ~0U,		~0U },
	{ 0x100U,	0x30U },
	{ 0x0U,		0x100U },
	{ 0x1000000U,	0x0U },
	{ 0xFFU,	0xFFU },
	{ 0x1U,		0x1U },
	{ 0x10U,	0x30U },
};

/*
 * The *writel*() access functions copy the incoming write into our own
 * access info. That way one can do the following:
 *
 *   nvgpu_writel(g, reg, val);
 *
 * And then:
 *
 *   do_something_with(writel_access.addr, writel_access.value);
 *
 * No bounds checking is performed by the mock API so that's up to you.
 * Higher level APIs may do this.
 */
static void writel_access_fn(struct gk20a *g,
			     struct nvgpu_reg_access *access)
{
	memcpy(&mockio_access, access, sizeof(mockio_access));
}

/*
 * Reads are handled by simply passing back a value. Exactly the opposite as the
 * write APIs.
 */
static void readl_access_fn(struct gk20a *g,
			    struct nvgpu_reg_access *access)
{
	/*
	 * The mock API checks that the returned address is actually the same as
	 * the requested address. If it mismatches then the mock IO API returns
	 * 0x0 to the nvgpu caller.
	 */
	memcpy(access, &mockio_access, sizeof(mockio_access));
}

static struct nvgpu_posix_io_callbacks test_callbacks = {
	/* Write APIs all can use the same accessor. */
	.writel          = writel_access_fn,
	.writel_check    = writel_access_fn,
	.bar1_writel     = writel_access_fn,
	.usermode_writel = writel_access_fn,

	/* Likewise for the read APIs. */
	.__readl         = readl_access_fn,
	.readl           = readl_access_fn,
	.bar1_readl      = readl_access_fn,
};

static int test_register_io_callbacks(struct unit_module *m, struct gk20a *g,
				      void *__args)
{
	nvgpu_posix_register_io(g, &test_callbacks);

	return UNIT_SUCCESS;
}

static int test_writel(struct unit_module *m, struct gk20a *g, void *__args)
{
	unsigned int i;
	struct nvgpu_reg_access *a;
	struct writel_test_args *args = __args;

	for (i = 0;
	     i < sizeof(test_access_list) / sizeof(test_access_list[0]);
	     i++) {
		a = &test_access_list[i];

		memset(&mockio_access, 0, sizeof(mockio_access));

		args->fn(g, a->addr, a->value);

		if (mockio_access.addr != a->addr ||
		    mockio_access.value != a->value) {
			unit_return_fail(m, "%s() mismatch!\n", args->name);
		}
	}

	return UNIT_SUCCESS;
}

static int test_readl(struct unit_module *m, struct gk20a *g, void *__args)
{
	unsigned int i;
	struct nvgpu_reg_access *a;
	struct readl_test_args *args = __args;

	for (i = 0;
	     i < sizeof(test_access_list) / sizeof(test_access_list[0]);
	     i++) {
		u32 ret;

		a = &test_access_list[i];
		memcpy(&mockio_access, a, sizeof(mockio_access));

		ret = args->fn(g, a->value);

		if (ret != a->value) {
			unit_return_fail(m, "%s() mismatch!\n", args->name);
		}
	}

	return UNIT_SUCCESS;
}

struct writel_test_args nvgpu_writel_args = {
	.name = "nvgpu_writel",
	.fn   = nvgpu_writel
};

struct writel_test_args nvgpu_writel_check_args = {
	.name = "nvgpu_writel_check",
	.fn   = nvgpu_writel_check
};

struct writel_test_args nvgpu_bar1_writel_args = {
	.name = "nvgpu_bar1_writel",
	.fn   = nvgpu_bar1_writel
};

struct writel_test_args nvgpu_usermode_writel_args = {
	.name = "nvgpu_usermode_writel",
	.fn   = nvgpu_usermode_writel
};

struct readl_test_args nvgpu_readl_args = {
	.name = "nvgpu_readl",
	.fn   = nvgpu_readl
};

struct readl_test_args __nvgpu_readl_args = {
	.name = "__nvgpu_readl",
	.fn   = __nvgpu_readl
};

struct readl_test_args nvgpu_bar1_readl_args = {
	.name = "nvgpu_bar1_readl",
	.fn   = nvgpu_bar1_readl
};

/*
 * Typical example of a write callback. At the very least the callback
 * should forward the write access to the mock IO framework and also
 * call the API to record transactions. This function would be a great
 * place to add test logic to run at every register write.
 */
static void writel_access_reg_fn(struct gk20a *g,
			     struct nvgpu_reg_access *access)
{
	nvgpu_posix_io_writel_reg_space(g, access->addr, access->value);
	nvgpu_posix_io_record_access(g, access);
}

/*
 * Example of a read callback. At the very least the callback should
 * get the register value from the mock IO framework. You could also add
 * some test logic to run at every register read.
 */
static void readl_access_reg_fn(struct gk20a *g,
			    struct nvgpu_reg_access *access)
{
	access->value = nvgpu_posix_io_readl_reg_space(g, access->addr);
}

/*
 * Define all the callbacks to be used during the test. Typically all
 * write operations use the same callback, likewise for all read operations.
 */
static struct nvgpu_posix_io_callbacks test_reg_callbacks = {
	/* Write APIs all can use the same accessor. */
	.writel          = writel_access_reg_fn,
	.writel_check    = writel_access_reg_fn,
	.bar1_writel     = writel_access_reg_fn,
	.usermode_writel = writel_access_reg_fn,

	/* Likewise for the read APIs. */
	.__readl         = readl_access_reg_fn,
	.readl           = readl_access_reg_fn,
	.bar1_readl      = readl_access_reg_fn,
};

static int test_register_space(struct unit_module *m, struct gk20a *g,
				void *__args)
{
	u32 value;

	nvgpu_posix_io_init_reg_space(g);
	nvgpu_posix_io_start_recorder(g);

	/* Define a couple of register spaces */
	if (nvgpu_posix_io_add_reg_space(g, 0x10000000, 0x100) != 0) {
		return UNIT_FAIL;
	}
	if (nvgpu_posix_io_add_reg_space(g, 0x80000000, 0x1000) != 0) {
		return UNIT_FAIL;
	}

	/*
	 * Some direct access operations to test register IO. This could be
	 * used to initialize memory before starting the actual test.
	 */
	nvgpu_posix_io_writel_reg_space(g, 0x10000000, 0x12345678);
	nvgpu_posix_io_writel_reg_space(g, 0x80000004, 0x87654321);
	value = nvgpu_posix_io_readl_reg_space(g, 0x80000004);
	if (value != 0x87654321) {
		return UNIT_FAIL;
	}
	nvgpu_posix_io_writel_reg_space(g, 0x10000100, 0x2727);

	/* Now re-define the callbacks to perform our own testing */
	struct nvgpu_posix_io_callbacks *old_cbs = nvgpu_posix_register_io(g,
		&test_reg_callbacks);

	/* The test begins where we would call some real NVGPU code */
	nvgpu_writel(g, 0x80000008, 0xA1B1C1D1);
	nvgpu_writel(g, 0x1000000C, 0x1);
	nvgpu_writel(g, 0x10000010, 0x55);
	/* End of real NVGPU code */

	/* First check that no memory access error occurred */
	if (nvgpu_posix_io_get_error_code(g) != 0) {
		unit_return_fail(m, "IO Access Error\n");
	}

	/* Verification can then be done either using nvgpu_readl or
	 * nvgpu_posix_io_readl_reg_space
	 */
	value = nvgpu_readl(g, 0x80000008);
	if (value != 0xA1B1C1D1) {
		unit_return_fail(m, "Register value mismatch at address=0x%x\n",
			0x80000008);
	}

	/* Define a sequence of expected register writes */
	struct nvgpu_reg_access sequence[] = {
		{ .addr = 0x80000008, .value = 0xA1B1C1D1 },
		{ .addr = 0x1000000C, .value = 0x1 },
		{ .addr = 0x10000010, .value = 0x55 }
	};

	/* Compare the recording with the expected sequence. If strict mode is
	 * used, then the same accesses, order and number of accesses is
	 * expected.
	 */
	if (nvgpu_posix_io_check_sequence(g, sequence,
		sizeof(sequence)/sizeof(struct nvgpu_reg_access),
		true) == false) {
		unit_return_fail(m, "Failed checking sequence\n");
	}

	/* Calling this function again resets the recorder to use it again */
	nvgpu_posix_io_start_recorder(g);

	/* Restore the old callbacks for other tests within this unit */
	nvgpu_posix_register_io(g, old_cbs);

	return UNIT_SUCCESS;
}


struct unit_module_test posix_mockio_tests[] = {
	UNIT_TEST(register_io_callbacks, test_register_io_callbacks, NULL),
	UNIT_TEST(writel,		 test_writel, &nvgpu_writel_args),
	UNIT_TEST(writel_check,		 test_writel, &nvgpu_writel_check_args),
	UNIT_TEST(bar1_writel,		 test_writel, &nvgpu_bar1_writel_args),
	UNIT_TEST(usermode_writel,	 test_writel,
		  &nvgpu_usermode_writel_args),
	UNIT_TEST(readl,		 test_readl, &nvgpu_readl_args),
	UNIT_TEST(__readl,		 test_readl, &__nvgpu_readl_args),
	UNIT_TEST(bar1_readl,		 test_readl, &nvgpu_bar1_readl_args),
	UNIT_TEST(test_register_space,	 test_register_space, NULL),
};

UNIT_MODULE(posix_mockio, posix_mockio_tests, UNIT_PRIO_POSIX_TEST);

