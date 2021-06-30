/*
 * Copyright (c) 2017-2018 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
  */

#include <linux/err.h>
#include <linux/trusty/trusty.h>
#include <linux/trusty/trusty_ipc.h>
#include <linux/ote_protocol.h>

#define OTF_PORT_NAME		"com.nvidia.tos.74629d40-1378-4d17-94d0e0af5d861d88"
#define OTF_REPROGRAM_KEYS	5

void trusty_restore_keyslots(void)
{
	void *opaque_channel_context = NULL;
	int ret = 0;

	/* Return if Trusty Device is not enabled */
	if (!is_trusty_dev_enabled())
		return;

	ret = te_open_trusted_session(OTF_PORT_NAME, &opaque_channel_context);

	/* Return if this Trusty port or TA is not supported without pr_err */
	if (ret == -EOPNOTSUPP)
		return;
	else if (ret) {
		pr_err("%s:ERROR(%d): Failed to open trusted session\n",
			__func__, ret);
		return;
	}

	ret = te_launch_trusted_oper(NULL, 0U, OTF_REPROGRAM_KEYS,
					opaque_channel_context);
	if (ret) {
		pr_err("%s:ERROR(%d): Failed to launch operation\n",
			__func__, ret);
		/* Fall Through */
	}

	te_close_trusted_session(opaque_channel_context);
}
EXPORT_SYMBOL(trusty_restore_keyslots);
