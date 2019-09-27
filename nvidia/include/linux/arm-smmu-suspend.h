/*
 * Copyright (c) 2018 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef _ARM_SMMU_SUSPEND_H
#define _ARM_SMMU_SUSPEND_H

#ifdef CONFIG_PM_SLEEP
int arm_smmu_suspend_init(void __iomem **smmu_base, u32 *smmu_base_pa,
				int num_smmus, unsigned long smmu_size,
				unsigned long smmu_pgshift, u32 scratch_reg_pa);
void arm_smmu_suspend_exit(void);
#else
int arm_smmu_suspend_init(void __iomem **smmu_base, u32 *smmu_base_pa,
				int num_smmus, unsigned long smmu_size,
				unsigned long smmu_pgshift, u32 scratch_reg_pa)
{
	return 0;
}
void arm_smmu_suspend_exit(void) {}
#endif

#endif
