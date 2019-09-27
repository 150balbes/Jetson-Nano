/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_COMMON_H
#define INCLUDE_CAMRTC_COMMON_H

#if defined(__KERNEL__)
#include <linux/types.h>
#include <linux/compiler.h>
#else
#include <stdint.h>
#include <stdbool.h>
#ifndef __packed
#define __packed __attribute__((packed))
#endif
#ifndef __aligned
#define __aligned(_n) __attribute__((aligned(_n)))
#endif
#ifndef U32_C
#define U32_C(x) x##UL
#endif
#ifndef U16_C
#define U16_C(x) (uint16_t)(x##U)
#endif
#ifndef U8_C
#define U8_C(x) (uint8_t)(x##U)
#endif
#endif

#endif /* INCLUDE_CAMRTC_COMMON_H */
