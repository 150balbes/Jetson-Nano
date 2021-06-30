/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CLK_GROUP_H
#define INCLUDE_CLK_GROUP_H

struct device;
struct camrtc_clk_group;

struct camrtc_clk_group *camrtc_clk_group_get(struct device *dev);

int camrtc_clk_group_enable(const struct camrtc_clk_group *grp);
void camrtc_clk_group_disable(const struct camrtc_clk_group *grp);

int camrtc_clk_group_adjust_fast(const struct camrtc_clk_group *grp);
int camrtc_clk_group_adjust_slow(const struct camrtc_clk_group *grp);

#endif /* INCLUDE_CLK_GROUP_H */
