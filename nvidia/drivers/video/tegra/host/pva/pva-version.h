/*
 * Copyright (c) 2016 NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef _PVA_VERSION_H_
#define _PVA_VERSION_H_

#include "pva-bit.h"

#define PVA_MAKE_VERSION(_type_, _major_, _minor_, _subminor_)	\
	(PVA_INSERT(_type_, 31, 24)				\
	| PVA_INSERT(_major_, 23, 16)				\
	| PVA_INSERT(_minor_, 15, 8)				\
	| PVA_INSERT(_subminor_, 7, 0))

static inline uint8_t
pva_is_compatible(uint32_t version, uint32_t compat_version)
{
	return PVA_EXTRACT(version, 23, 0, uint32_t)
		>= PVA_EXTRACT(compat_version, 23, 0, uint32_t);
}

#endif
