/*
 * IOMMU API for ARM architected SMMU implementations.
 *
 * Copyright (c) 2018 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef _ARM_SMMU_REGS_T19X_H
#define _ARM_SMMU_REGS_T19X_H

#include "arm-smmu-regs.h"

/* SMMU global address space */
#undef ARM_SMMU_GR0
#define ARM_SMMU_GR0(smmu)		((smmu)->base[0])

#undef ARM_SMMU_GR1
#define ARM_SMMU_GR1(smmu)		((smmu)->base[0] + (1 << (smmu)->pgshift))

#undef ARM_SMMU_PME
#define ARM_SMMU_PME(smmu)		((smmu)->base[0] + (3 << (smmu)->pgshift))

/*
 * SMMU global address space with conditional offset to access secure
 * aliases of non-secure registers (e.g. nsCR0: 0x400, nsGFSR: 0x448,
 * nsGFSYNR0: 0x450)
 */
#undef ARM_SMMU_GR0_NS
#define ARM_SMMU_GR0_NS(smmu)						\
	((smmu)->base[0] +						\
		((smmu->options & ARM_SMMU_OPT_SECURE_CFG_ACCESS)	\
			? 0x400 : 0))

/* Translation context bank */
#undef ARM_SMMU_CB_BASE
#define ARM_SMMU_CB_BASE(smmu)		((smmu)->base[0] + ((smmu)->size >> 1))

#undef ARM_SMMU_CB
#define ARM_SMMU_CB(smmu, n)		((n) * (1 << (smmu)->pgshift))

#endif /* _ARM_SMMU_REGS_H */
