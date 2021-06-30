/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_RESET_GROUP_H
#define INCLUDE_RESET_GROUP_H

struct device;
struct camrtc_reset_group;

struct camrtc_reset_group *camrtc_reset_group_get(
	struct device *dev,
	const char *group_name);

void camrtc_reset_group_assert(const struct camrtc_reset_group *grp);
int camrtc_reset_group_deassert(const struct camrtc_reset_group *grp);

#endif /* INCLUDE_RESET_GROUP_H */
