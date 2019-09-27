/*
 * Copyright (C) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _FIXED_POINT_H
#define _FIXED_POINT_H

struct fixed_point {
	unsigned int int_part;
	unsigned int frac_part;
	unsigned int int_prec;
	unsigned int frac_prec;
	unsigned int int_mask;
	unsigned int frac_mask;
};

struct fixed_point fixed_point_init(
	unsigned int int_part,
	unsigned int frac_part,
	unsigned int int_prec,
	unsigned int frac_prec,
	unsigned int *error);

struct fixed_point fixed_point_shift_left(
	struct fixed_point fp_arg,
	unsigned int places,
	unsigned int *error);

struct fixed_point fixed_point_shift_right(
	struct fixed_point fp_arg,
	unsigned int places,
	unsigned int *error);

struct fixed_point fixed_point_negate(
	struct fixed_point fp_arg,
	unsigned int *error);

struct fixed_point fixed_point_add(
	struct fixed_point fp_arg1,
	struct fixed_point fp_arg2,
	unsigned int *error);

/* returns difference = minuend - subtrahend */
struct fixed_point fixed_point_sub(
	struct fixed_point minuend_arg,
	struct fixed_point subtrahend_arg,
	unsigned int *error);

struct fixed_point fixed_point_mult(
	struct fixed_point fp_arg1,
	struct fixed_point fp_arg2,
	unsigned int *error);

/* returns quotient = dividend / divisor*/
struct fixed_point fixed_point_div(
	struct fixed_point dividend_arg,
	struct fixed_point divisor_arg,
	unsigned int *error);

/* Return 1 is lhs < rhs, 0 otherwise */
int fixed_point_lt(
	struct fixed_point fp_lhs_arg,
	struct fixed_point fp_rhs_arg,
	unsigned int *error);

/* Return 1 is lhs > rhs, 0 otherwise */
int fixed_point_gt(
	struct fixed_point fp_lhs_arg,
	struct fixed_point fp_rhs_arg,
	unsigned int *error);

/* Return 1 is lhs <= rhs, 0 otherwise */
int fixed_point_loet(
	struct fixed_point fp_lhs_arg,
	struct fixed_point fp_rhs_arg,
	unsigned int *error);

/* Return 1 is lhs >= rhs, 0 otherwise */
int fixed_point_goet(
	struct fixed_point fp_lhs_arg,
	struct fixed_point fp_rhs_arg,
	unsigned int *error);

/* Return 1 is lhs == rhs, 0 otherwise */
int fixed_point_eq(
	struct fixed_point fp_lhs_arg,
	struct fixed_point fp_rhs_arg,
	unsigned int *error);

int fixed_point_to_int(
	struct fixed_point fp_arg,
	unsigned int *error);

int fixed_point_ceil(
	struct fixed_point fp_arg,
	unsigned int *error);

struct fixed_point fixed_point_min(
	struct fixed_point fp_arg1,
	struct fixed_point fp_arg2,
	unsigned int *error);

struct fixed_point fixed_point_max(
	struct fixed_point fp_arg1,
	struct fixed_point fp_arg2,
	unsigned int *error);

#endif /* _FIXED_POINT_H */
