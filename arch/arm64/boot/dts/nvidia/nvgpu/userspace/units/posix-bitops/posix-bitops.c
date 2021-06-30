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

#include <nvgpu/bitops.h>

#define NUM_WORDS 4

static unsigned long single_ulong_maps[] = {
	0UL,
	~0UL,
	0xff00ff00UL,
	0x00ff00ffUL,
	0xa5a5a5a5UL,
	0x0000ffffUL,
	0xffff0000UL,
	0x1UL,
	0x80000000UL,
	BIT(16),
};

/*
 * Can't fail - just some info prints.
 */
static int test_bitmap_info(struct unit_module *m, struct gk20a *g, void *args)
{
	unit_info(m, "sizeof(unsigned long) = %zu\n", sizeof(unsigned long));
	unit_info(m, "BITS_PER_LONG         = %lu\n", BITS_PER_LONG);

	return UNIT_SUCCESS;
}

static int test_ffs(struct unit_module *m, struct gk20a *g, void *args)
{
#define CHECK_FFS_WORD(w, answer)					\
	do {								\
		unsigned long ret = ffs(w);				\
									\
		if (ret != (answer))					\
			unit_return_fail(m,				\
					 "ffs(0x%016lx) = %lu "		\
					 "[expected %lu]\n",		\
					 w, ret, answer);		\
	} while (0)

	unsigned long i;

	CHECK_FFS_WORD(single_ulong_maps[0], BITS_PER_LONG - 1UL);
	CHECK_FFS_WORD(single_ulong_maps[1], 0UL);
	CHECK_FFS_WORD(single_ulong_maps[2], 8UL);
	CHECK_FFS_WORD(single_ulong_maps[3], 0UL);
	CHECK_FFS_WORD(single_ulong_maps[4], 0UL);
	CHECK_FFS_WORD(single_ulong_maps[5], 0UL);
	CHECK_FFS_WORD(single_ulong_maps[6], 16UL);
	CHECK_FFS_WORD(single_ulong_maps[7], 0UL);
	CHECK_FFS_WORD(single_ulong_maps[8], 31UL);
	CHECK_FFS_WORD(single_ulong_maps[9], 16UL);

#undef CHECK_FFS_WORD

	/*
	 * Also just test every bit to make sure we definitely cover all
	 * possible return values of the function.
	 */
	for (i = 0; i < BITS_PER_LONG; i++) {
		if (ffs(BIT(i)) != i)
			unit_return_fail(m, "ffs(1 << %lu) != %lu [%lu]!\n",
					 i, i, ffs(BIT(i)));
	}

	return UNIT_SUCCESS;
}

static int test_fls(struct unit_module *m, struct gk20a *g, void *args)
{
#define CHECK_FLS_WORD(w, answer)					\
	do {								\
		unsigned long ret = fls(w);				\
									\
		if (ret != (answer))					\
			unit_return_fail(m,				\
					 "fls(0x%016lx) = %lu "		\
					 "[expected = %lu]\n",		\
					 w, ret, answer);		\
	} while (0)

	unsigned long i;

	CHECK_FLS_WORD(single_ulong_maps[0], 0UL);
	CHECK_FLS_WORD(single_ulong_maps[1], BITS_PER_LONG);
	CHECK_FLS_WORD(single_ulong_maps[2], 32UL);
	CHECK_FLS_WORD(single_ulong_maps[3], 24UL);
	CHECK_FLS_WORD(single_ulong_maps[4], 32UL);
	CHECK_FLS_WORD(single_ulong_maps[5], 16UL);
	CHECK_FLS_WORD(single_ulong_maps[6], 32UL);
	CHECK_FLS_WORD(single_ulong_maps[7], 1UL);
	CHECK_FLS_WORD(single_ulong_maps[8], 32UL);
	CHECK_FLS_WORD(single_ulong_maps[9], 17UL);

#undef CHECK_FLS_WORD

	for (i = 0; i < BITS_PER_LONG; i++) {
		if (fls(BIT(i)) != (i+1))
			unit_return_fail(m, "fls(1 << %lu) != %lu! [%lu]\n",
					 i, i, fls(BIT(i)));
	}

	return UNIT_SUCCESS;
}

static int test_ffz(struct unit_module *m, struct gk20a *g, void *args)
{
	unsigned long i;

	/*
	 * Since ffz(w) is implemented as ffs(~w) this does less extensive
	 * testing; but it should still cover every line of ffs().
	 */

	for (i = 0; i < BITS_PER_LONG; i++) {
		if (ffz(~BIT(i)) != i)
			unit_return_fail(m, "ffz(~(1 << %lu)) != %lu! [%lu]\n",
					 i, i, ffz(BIT(i)));
	}

	return UNIT_SUCCESS;
}

struct test_find_bit_args {
	bool find_zeros;
};

static struct test_find_bit_args first_bit_args = {
	.find_zeros = false
};

static struct test_find_bit_args first_zero_args = {
	.find_zeros = true
};

static int test_find_first_bit(struct unit_module *m,
			       struct gk20a *g, void *__args)
{
	struct test_find_bit_args *args = __args;
	unsigned long words[NUM_WORDS];
	unsigned long word_idx, bit_idx;
	unsigned long (*finder_function)(const unsigned long *, unsigned long);
	unsigned long result;

	if (args->find_zeros)
		finder_function = find_first_zero_bit;
	else
		finder_function = find_first_bit;

	/*
	 * First test: verify that the size parameter works. We only need the
	 * first word for this.
	 */
	words[0] = ~0xffffUL;
	if (args->find_zeros)
		words[0] = 0xffff;

	if (finder_function(words, 8UL) != 8UL)
		unit_return_fail(m,
				 "find_first_%s(0x%lx, 8) -> %lu [WRONG]\n",
				 args->find_zeros ? "zero_bit" : "bit",
				 words[0], finder_function(words, 8UL));

	if (finder_function(words, 20UL) != 16UL)
		unit_return_fail(m,
				 "find_first_%s(0x%lx, 16) -> %lu [WRONG]\n",
				 args->find_zeros ? "zero_bit" : "bit",
				 words[0], finder_function(words, 20UL));

	/*
	 * Now make sure that for full/empty bitmap find_next_*() returns
	 * the size parameter.
	 */
	memset(words, args->find_zeros ? 0xff : 0x00, sizeof(words));
	result = finder_function(words, NUM_WORDS * BITS_PER_LONG);
	if (result != NUM_WORDS * BITS_PER_LONG)
		unit_return_fail(m, "find_first_%s() failed with empty map\n",
				 args->find_zeros ? "zero_bit" : "bit");

	/*
	 * Third test: set (or zero) the entire bitmap and incrementally clear
	 * bits. Check that we are correct even with multiple words.
	 */

	memset(words, args->find_zeros ? 0x00 : 0xff, sizeof(words));
	for (word_idx = 0; word_idx < NUM_WORDS; word_idx++) {
		for (bit_idx = 0; bit_idx < BITS_PER_LONG; bit_idx++) {
			unsigned long check =
				(word_idx * BITS_PER_LONG) + bit_idx;
			unsigned long answer =
				finder_function(words,
						NUM_WORDS * BITS_PER_LONG);

			if (answer != check)
				unit_return_fail(m,
						 "find_first_%s loop: "
						 "word_idx = %lu bit_idx = %lu "
						 "-> %lu [WRONG]\n",
						 args->find_zeros ? "zero_bit" : "bit",
						 word_idx, bit_idx, answer);

			/*
			 * Now set/clear this bit in preparation for the next
			 * test.
			 */
			if (args->find_zeros)
				words[word_idx] |= BIT(bit_idx);
			else
				words[word_idx] &= ~BIT(bit_idx);
		}
	}

	return UNIT_SUCCESS;
}

/*
 * Note: the find_first_bit() test also effectively tests the underlying
 * find_next_bit() code since find_first_bit() is just find_next_bit()
 * with a 0 start.
 */

static int test_find_next_bit(struct unit_module *m,
			      struct gk20a *g, void *__args)
{
	unsigned long words[NUM_WORDS];
	unsigned long i, result;

	/*
	 * Fully unset list. Should always return size.
	 */
	memset(words, 0x00, sizeof(words));
	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++) {
		result = find_next_bit(words, NUM_WORDS * BITS_PER_LONG, i);

		if (result != NUM_WORDS * BITS_PER_LONG)
			unit_return_fail(m, "Fail: empty map (%lu)\n", i);
	}

	/*
	 * Use a fully set list but increment the offset.
	 */
	memset(words, 0xff, sizeof(words));
	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++) {
		unsigned long first =
			find_next_bit(words, NUM_WORDS * BITS_PER_LONG, i);

		if (first != i)
			unit_return_fail(m,
					 "Fail: first = %lu; should be %lu\n",
					 first, i);
	}

	/*
	 * Start > n should return n.
	 */
#define TEST_START_GREATER_THAN_N(m, map, n, start)			\
	do {								\
		if (find_next_bit(map, n, start) != n)			\
			unit_return_fail(m,				\
					 "Start not greater than N ?? "	\
					 "start=%lu, N=%lu\n",		\
					 start, n);			\
	} while (0)

	TEST_START_GREATER_THAN_N(m, words, BITS_PER_LONG, BITS_PER_LONG + 1);
	TEST_START_GREATER_THAN_N(m, words, 32UL, 64UL);
	TEST_START_GREATER_THAN_N(m, words,
				  BITS_PER_LONG * 2, (BITS_PER_LONG * 2) + 1);
	TEST_START_GREATER_THAN_N(m, words, 0UL, 1UL);
	TEST_START_GREATER_THAN_N(m, words, 0UL, BITS_PER_LONG * 2);
	TEST_START_GREATER_THAN_N(m, words, 0UL, BITS_PER_LONG * NUM_WORDS + 1);

#undef TEST_START_GREATER_THAN_N

	return UNIT_SUCCESS;
}

#define TEST_BITMAP_SIZE (BITS_PER_LONG * 4)
/*
 * 32/64 bit invarient.
 */
static DECLARE_BITMAP(bmap_all_zeros, TEST_BITMAP_SIZE) =
{
	0x0UL, 0x0UL, 0x0UL, 0x0UL
};
static DECLARE_BITMAP(bmap_all_ones, TEST_BITMAP_SIZE) =
{
	~0x0UL, ~0x0UL, ~0x0UL, ~0x0UL
};

static int test_find_zero_area(struct unit_module *m,
			       struct gk20a *g, void *unused)
{
#define FAIL_MSG "Fail: bmap-test='%s' (i=%lu)\n"
#define FAIL_MSG_EX "Fail: bmap-test='%s' (i=%lu, j=%lu)\n"
	unsigned long i, j, result;
	unsigned long words[NUM_WORDS];

	for (i = 0; i < TEST_BITMAP_SIZE; i++) {
		result = bitmap_find_next_zero_area_off(bmap_all_zeros,
							TEST_BITMAP_SIZE,
							i,
							TEST_BITMAP_SIZE - i,
							0, 0);
		if (result != i)
			unit_return_fail(m, FAIL_MSG,
					 "all_zeros: alloc-to-end", i);

		result = bitmap_find_next_zero_area_off(bmap_all_zeros,
							TEST_BITMAP_SIZE,
							i,
							1,
							0, 0);
		if (result != i)
			unit_return_fail(m, FAIL_MSG,
					 "all_zeros: alloc-one-bit", i);

		result = bitmap_find_next_zero_area_off(bmap_all_zeros,
							TEST_BITMAP_SIZE,
							0,
							TEST_BITMAP_SIZE - i,
							0, 0);
		if (result != 0)
			unit_return_fail(m, FAIL_MSG,
					 "all_zeros: alloc-i-bits-at-0", i);
	}

	/*
	 * For the all ones bit map not a single alloc should succeed. We can
	 * just iterate through them all and make sure they all fail.
	 */
	for (i = 0; i < TEST_BITMAP_SIZE; i++) {
		for (j = 0; j < (TEST_BITMAP_SIZE - i); j++) {
			result = bitmap_find_next_zero_area_off(bmap_all_ones,
							TEST_BITMAP_SIZE,
							i,
							j,
							0, 0);
			if (result != TEST_BITMAP_SIZE)
				unit_return_fail(m, FAIL_MSG_EX,
						 "all_ones: failed", i, j);
		}
	}

	/*
	 * Alternating nibbles (4 bits). Make sure we don't start searching from
	 * too high in the bitmap since that will cause failures that are
	 * actually valid. This keeps the logic in the below loop a little more
	 * simple.
	 */
	memset(words, 0x0f, sizeof(words));
	for (i = 0; i < ((NUM_WORDS * BITS_PER_LONG) - 8); i++) {
		for (j = 0; j < ((NUM_WORDS * BITS_PER_LONG) - i - 8); j++) {
			result = bitmap_find_next_zero_area_off(words,
						NUM_WORDS * BITS_PER_LONG,
						i,
						j,
						0, 0);

			/*
			 * Should only return a valid result when j < 4 (since
			 * the map consists of 4 ones, then 4 zeros,
			 * alternating.
			 */
			if (j <= 4 && result >= (NUM_WORDS * BITS_PER_LONG))
				unit_return_fail(m, FAIL_MSG_EX,
						 "alternating-nibbles: failed",
						 i, j);
			if (j > 4 && result != (NUM_WORDS * BITS_PER_LONG))
				unit_return_fail(m, FAIL_MSG_EX,
						 "alternating-nibbles: failed",
						 i, j);

			result = bitmap_find_next_zero_area_off(words,
						NUM_WORDS * BITS_PER_LONG,
						i,
						(j % 4) + 1,
						0x3, 0);
			if (result % 8 != 4)
				unit_return_fail(m, FAIL_MSG_EX,
						 "basic-align_mask: failed",
						 i, j);


			result = bitmap_find_next_zero_area_off(words,
						NUM_WORDS * BITS_PER_LONG,
						i,
						(j % 2) + 1,
						0x7, 2);

			if (result % 8 != 6)
				unit_return_fail(m, FAIL_MSG_EX,
						 "basic-align_offset: failed",
						 i, j);
		}
	}

#undef FAIL_MSG
#undef FAIL_MSG_EX
	return UNIT_SUCCESS;
}

struct test_setclear_args {
	bool clear;
};

static struct test_setclear_args set_args = {
	.clear = false,
};

static struct test_setclear_args clear_args = {
	.clear = true,
};

static void __print_bitmap(unsigned long *map, unsigned long length)
{
	unsigned int idx, bidx;

	for (idx = 0; idx < (length / BITS_PER_LONG); idx++) {
		printf("  ");
		for (bidx = 0; bidx < BITS_PER_LONG; bidx++) {
			printf("%c", (map[idx] & BIT(bidx)) ? '1' : '0');
			if (bidx % 4 == 3)
				printf(" ");
		}
		printf("\n");
	}

	printf("\n");
}

/*
 * Verify the bits from i to i + len are set and no others are set. 'size' is in
 * bits (not words).
 */
static bool verify_set_buf(unsigned long *words, unsigned long size,
			   unsigned long i, unsigned long len,
			   bool invert)
{
	unsigned int idx, bidx;
	unsigned int bit, bit_value;
	unsigned int set_start, set_end;
	unsigned int set_value = invert ? 0 : 1;
	unsigned int clear_value = invert ? 1 : 0;

	set_start = i;
	set_end = i + len;

	for (idx = 0; idx < (size / BITS_PER_LONG); idx++) {
		for (bidx = 0; bidx < BITS_PER_LONG; bidx++) {
			bit = idx * BITS_PER_LONG + bidx;
			bit_value = !!(words[idx] & BIT(bidx));

			/*
			 * Bit inside set zone.
			 */
			if ((bit >= set_start && bit < set_end) &&
			    bit_value != set_value)
				return false;

			/*
			 * Bit outside set zone.
			 */
			if ((bit < set_start || bit >= set_end) &&
			    bit_value != clear_value)
				return false;
		}
	}

	return true;
}

static int test_single_bitops(struct unit_module *m,
			      struct gk20a *g, void *__args)
{
	unsigned long words[NUM_WORDS];
	unsigned int i;

	/*
	 * First set all the bits and make sure the words are set.
	 */
	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++)
		set_bit(i, words);

	if (!verify_set_buf(words, NUM_WORDS * BITS_PER_LONG,
			    0, NUM_WORDS * BITS_PER_LONG, false)) {
		__print_bitmap(words, NUM_WORDS * BITS_PER_LONG);
		unit_return_fail(m, "set_bit: Failed to set a bit!\n");
	}

	/*
	 * Now make sure the test_bit works for set bits.
	 */
	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++)
		if (!test_bit(i, words))
			unit_return_fail(m, "test_bit: bit %d failed!\n", i);

	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++)
		clear_bit(i, words);

	if (!verify_set_buf(words, NUM_WORDS * BITS_PER_LONG,
			    0, NUM_WORDS * BITS_PER_LONG, true)) {
		__print_bitmap(words, NUM_WORDS * BITS_PER_LONG);
		unit_return_fail(m, "clear_bit: Failed to set a bit!\n");
	}

	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++)
		if (test_bit(i, words))
			unit_return_fail(m, "test_bit: bit %d failed!\n", i);

	return UNIT_SUCCESS;
}

static int test_bit_setclear(struct unit_module *m,
			     struct gk20a *g, void *__args)
{
	struct test_setclear_args *args = __args;
	void (*testfn)(int, volatile unsigned long *) =
		args->clear ? clear_bit : set_bit;
	unsigned long words[NUM_WORDS];
	unsigned int i;

	memset(words, args->clear ? 0xff : 0x0, sizeof(words));

	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++)
		testfn(i, words);

	if (!verify_set_buf(words, NUM_WORDS * BITS_PER_LONG,
			    0, NUM_WORDS * BITS_PER_LONG, args->clear)) {
		__print_bitmap(words, NUM_WORDS * BITS_PER_LONG);
		unit_return_fail(m, "%s_bit: Failed to %s a bit!\n",
				 args->clear ? "clear" : "set",
				 args->clear ? "clear" : "set");
	}

	return UNIT_SUCCESS;
}

static int test_test_and_setclear_bit(struct unit_module *m,
				      struct gk20a *g, void *__args)
{
	struct test_setclear_args *args = __args;
	bool (*testfn)(int, volatile unsigned long *) =
		args->clear ? test_and_clear_bit : test_and_set_bit;
	bool (*testfn_reset)(int, volatile unsigned long *) =
		args->clear ? test_and_set_bit : test_and_clear_bit;
	unsigned long words[NUM_WORDS];
	unsigned int i;

	memset(words, args->clear ? 0xff : 0x0, sizeof(words));

	/*
	 * First we will set/clear the bits. Then we will clear/set the bits
	 * (i.e do the opposite of this loop).
	 */
	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++) {
		bool status = testfn(i, words);

		if (status != args->clear) {
			__print_bitmap(words, NUM_WORDS * BITS_PER_LONG);
			unit_return_fail(m, "test_and_%s_bit: Failed at %d\n",
					 args->clear ? "clear" : "set", i);
		}
	}

	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++) {
		bool status = testfn_reset(i, words);

		if (status == args->clear) {
			__print_bitmap(words, NUM_WORDS * BITS_PER_LONG);
			unit_return_fail(m, "test_and_%s_bit: Failed at %d\n",
					 args->clear ? "set" : "clear", i);
		}
	}

	/* The bitmap should be the same as we started with. */
	if (!verify_set_buf(words, NUM_WORDS * BITS_PER_LONG,
			    0, NUM_WORDS * BITS_PER_LONG, !args->clear)) {
		__print_bitmap(words, NUM_WORDS * BITS_PER_LONG);
		unit_return_fail(m,
				 "test_and_%s_bit: Final bitmap is wrong!\n",
				 args->clear ? "set" : "clear");
	}

	return UNIT_SUCCESS;
}

static int test_bitmap_setclear(struct unit_module *m,
				struct gk20a *g, void *__args)
{
	struct test_setclear_args *args = __args;
	void (*testfn)(unsigned long *, unsigned int, int) =
		args->clear ? bitmap_clear : bitmap_set;
	unsigned long words[NUM_WORDS];
	unsigned long i, j;
	int set_char = args->clear ? 0xff : 0x0;

	/*
	 * Run through all combos of set/clear for a 4 word bitmap.
	 */
	for (i = 0; i < NUM_WORDS * BITS_PER_LONG; i++) {
		for (j = 0; j < (NUM_WORDS * BITS_PER_LONG) - i; j++) {
			/*
			 * Just make sure we start in a known state.
			 */
			memset(words, set_char, sizeof(words));

			testfn(words, i, j);

			if (!verify_set_buf(words, NUM_WORDS * BITS_PER_LONG,
					    i, j, args->clear)) {
				__print_bitmap(words, NUM_WORDS * BITS_PER_LONG);
				unit_return_fail(m,
						 "%s: fail at i,j = %lu,%lu\n",
						 args->clear ? "clear" : "set",
						 i, j);
			}
		}
	}

	return UNIT_SUCCESS;
}



struct unit_module_test posix_bitops_tests[] = {
	UNIT_TEST(info,                test_bitmap_info, NULL),
	UNIT_TEST(ffs,                 test_ffs, NULL),
	UNIT_TEST(fls,                 test_fls, NULL),
	UNIT_TEST(ffz,                 test_ffz, NULL),
	UNIT_TEST(find_first_bit,      test_find_first_bit, &first_bit_args),
	UNIT_TEST(find_first_zero_bit, test_find_first_bit, &first_zero_args),
	UNIT_TEST(find_next_bit,       test_find_next_bit, NULL),
	UNIT_TEST(find_zero_area,      test_find_zero_area, NULL),
	UNIT_TEST(single_bitops,       test_single_bitops, NULL),
	UNIT_TEST(bit_set,             test_bit_setclear, &set_args),
	UNIT_TEST(bit_clear,           test_bit_setclear, &clear_args),
	UNIT_TEST(test_and_set_bit,    test_test_and_setclear_bit, &set_args),
	UNIT_TEST(test_and_clear_bit,  test_test_and_setclear_bit, &clear_args),
	UNIT_TEST(bitmap_set,          test_bitmap_setclear, &set_args),
	UNIT_TEST(bitmap_clear,        test_bitmap_setclear, &clear_args),
};

UNIT_MODULE(posix_bitops, posix_bitops_tests, UNIT_PRIO_POSIX_TEST);
