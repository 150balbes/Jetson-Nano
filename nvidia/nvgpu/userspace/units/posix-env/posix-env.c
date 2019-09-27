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
#include <stdint.h>

#include <unit/io.h>
#include <unit/unit.h>

#include <nvgpu/types.h>

/*
 * Test if the passed type is signed. In the signed case this expression becomes
 *
 *   x = 0;
 *   y = x - 1 = -1
 *
 * And -1 < 0 will be true. In the unsigned case, we rely on wrap around. We
 * have the following
 *
 *   x = 0U
 *   y = x - 1 = ~0 (i.e <TYPE>_MAX)
 *
 * Thus the expression y < x is false.
 */
#define IS_SIGNED_TYPE(__type__)		\
	({					\
		__type__ x = (__type__)0;	\
		__type__ y = x - (__type__)1;	\
		y < x;				\
	})

/*
 * Ensure that our sized types are of the correct size. Assumes an 8bit byte of
 * course.
 */
static int sanity_test_sizes(struct unit_module *m,
			     struct gk20a *g, void *args)
{
	size_t size_u8  = sizeof(u8);
	size_t size_u16 = sizeof(u16);
	size_t size_u32 = sizeof(u32);
	size_t size_u64 = sizeof(u64);
	size_t size_s8  = sizeof(s8);
	size_t size_s16 = sizeof(s16);
	size_t size_s32 = sizeof(s32);
	size_t size_s64 = sizeof(s64);

	/* Unsigned. */
	if (size_u8 != 1U)
		unit_return_fail(m,
				 "sizeof(u8) != 1 byte! (Actual size: %zu)\n",
				 size_u8);
	if (size_u16 != 2U)
		unit_return_fail(m,
				 "sizeof(u16) != 2 bytes! (Actual size: %zu)\n",
				 size_u16);
	if (size_u32 != 4U)
		unit_return_fail(m,
				 "sizeof(u32) != 4 bytes! (Actual size: %zu)\n",
				 size_u32);
	if (size_u64 != 8U)
		unit_return_fail(m,
				 "sizeof(u64) != 8 bytes! (Actual size: %zu)\n",
				 size_u64);

	/* Signed. */
	if (size_s8 != 1U)
		unit_return_fail(m,
				 "sizeof(s8) != 1 byte! (Actual size: %zu)\n",
				 size_s8);
	if (size_s16 != 2U)
		unit_return_fail(m,
				 "sizeof(s16) != 2 bytes! (Actual size: %zu)\n",
				 size_s16);
	if (size_s32 != 4U)
		unit_return_fail(m,
				 "sizeof(s32) != 4 bytes! (Actual size: %zu)\n",
				 size_s32);
	if (size_s64 != 8U)
		unit_return_fail(m,
				 "sizeof(s64) != 8 bytes! (Actual size: %zu)\n",
				 size_s64);

	return UNIT_SUCCESS;
}

/*
 * Make sure that the signed types really are signed and that the unsigned types
 * really are not.
 */
static int sanity_test_signage(struct unit_module *m,
			       struct gk20a *g, void *args)
{

        if (!IS_SIGNED_TYPE(s8))
                unit_return_fail(m, "s8 is not signed!\n");
        if (!IS_SIGNED_TYPE(s16))
                unit_return_fail(m, "s16 is not signed!\n");
        if (!IS_SIGNED_TYPE(s32))
                unit_return_fail(m, "s32 is not signed!\n");
        if (!IS_SIGNED_TYPE(s64))
                unit_return_fail(m, "s64 is not signed!\n");

        if (IS_SIGNED_TYPE(u8))
                unit_return_fail(m, "u8 is signed!\n");
        if (IS_SIGNED_TYPE(u16))
                unit_return_fail(m, "u16 is signed!\n");
        if (IS_SIGNED_TYPE(u32))
                unit_return_fail(m, "u32 is signed!\n");
        if (IS_SIGNED_TYPE(u64))
                unit_return_fail(m, "u64 is signed!\n");

	return UNIT_SUCCESS;
}

/*
 * Ensure that a u64 can hold a pointer since in some places we use a u64 to
 * pass back a pointer value.
 */
static int sanity_test_ptr_in_u64(struct unit_module *m,
				  struct gk20a *g, void *args)
{
	if (sizeof(u64) < sizeof(uintptr_t))
		unit_return_fail(m,
			"u64 size (%zu b) less than pointer size (%zu b)",
			 sizeof(u64), sizeof(uintptr_t));

	return UNIT_SUCCESS;
}

static int sanity_test_endianness(struct unit_module *m,
				  struct gk20a *g, void *args)
{
	u32 i;
	u32 x = 0x12345678;
	u8  *ptr_x_u8  = (u8 *)&x;
	u16 *ptr_x_u16 = (u16 *)&x;

	/*
	 * Print what endianness we have. For now we have not explicitly decided
	 * to support one or the other, but this will have to be determined
	 * eventually.
	 *
	 * We have just been lucky so far.
	 */
	unit_info(m, "u32 x = 0x%x\n", x);
	for (i = 0; i < sizeof(u32) / sizeof(u16); i++) {
		unit_info(m, "  &x + %zu: 0x%04hx\n",
			  i * sizeof(u16), ptr_x_u16[i]);
	}

	unit_info(m, "u32 x = 0x%x\n", x);
	for (i = 0; i < sizeof(u32); i++) {
		unit_info(m, "  &x + %u: 0x%02hhx\n", i, ptr_x_u8[i]);
	}

	switch (ptr_x_u8[0]) {
	case 0x12:
		unit_info(m, "Machine endianness: big\n");
		break;
	case 0x78:
		unit_info(m, "Machine endianness: little\n");
		break;
	default:
		unit_return_fail(m, "Machine endianness: middle/unknown ??\n");
	}

	return UNIT_SUCCESS;
}

struct unit_module_test posix_env_tests[] = {
	UNIT_TEST(sizes,      sanity_test_sizes,   NULL),
	UNIT_TEST(signage,    sanity_test_signage, NULL),
	UNIT_TEST(endianness, sanity_test_endianness, NULL),
	UNIT_TEST(ptr_in_u64, sanity_test_ptr_in_u64, NULL),
};

UNIT_MODULE(posix_env, posix_env_tests, UNIT_PRIO_POSIX_TEST);
