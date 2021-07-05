/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef _TEGRA_AON_IVC_PLLAON_H_
#define _TEGRA_AON_IVC_PLLAON_H_

#include <linux/types.h>

/* Get state of PLLAON clock from aon controller.
 * 1 - if PLLAON clock is enabled
 * 0 - if PLLAON clock is disabled
 * -EINVAL - if IVC fails
 */
int tegra_aon_get_pllaon_state(void);

/* Set state of PLLAON clock from aon controller.
 * @enable true  - to enable PLLAON clock
 *	   false - to disable PLLAON clock
 * @return
 * 	0 - if IVC with AON controller is successful
 * 	-EINVAL - if IVC fails
 */
int tegra_aon_set_pllaon_state(bool enable);

#endif
