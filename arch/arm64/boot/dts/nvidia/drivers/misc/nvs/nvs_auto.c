/* Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/nvs.h>


struct nvs_fn_if *nvs_auto(kif_i)
{
	switch (kif_i) {
		case NVS_KIF_AUTO:
#ifdef NVS_CFG_KIF_IIO
		case NVS_KIF_IIO:
			return nvs_iio();
#endif

#ifdef NVS_CFG_KIF_INPUT
		case NVS_KIF_INPUT:
			return nvs_input();
#endif

#ifdef NVS_CFG_KIF_RELAY
		case NVS_KIF_RELAY:
			return nvs_relay();
#endif

		default :
			return NULL;
	}
}

EXPORT_SYMBOL_GPL(nvs_auto);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor Kernel InterFace module");
MODULE_AUTHOR("NVIDIA Corporation");

