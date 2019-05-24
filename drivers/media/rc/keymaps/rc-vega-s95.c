// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (c) 2018 Christian Hewitt <christianshewitt@gmail.com>

#include <media/rc-map.h>
#include <linux/module.h>

/*
 * This keymap is used with the Tronsmart Vega S95 Android STB
 */

static struct rc_map_table vega_s95[] = {

	{ 0x18, KEY_POWER },
	{ 0x17, KEY_INFO }, // mouse

	{ 0x46, KEY_UP },
	{ 0x16, KEY_DOWN },
	{ 0x47, KEY_LEFT },
	{ 0x15, KEY_RIGHT },
	{ 0x55, KEY_OK },

	{ 0x06, KEY_HOME },
	{ 0x42, KEY_PLAYPAUSE},
	{ 0x40, KEY_BACK },

	{ 0x14, KEY_VOLUMEDOWN },
	{ 0x04, KEY_MENU },
	{ 0x10, KEY_VOLUMEUP },

};

static struct rc_map_list vega_s95_map = {
	.map = {
		.scan     = vega_s95,
		.size     = ARRAY_SIZE(vega_s95),
		.rc_proto = RC_PROTO_NEC,
		.name     = RC_MAP_VEGA_S95,
	}
};

static int __init init_rc_map_vega_s95(void)
{
	return rc_map_register(&vega_s95_map);
}

static void __exit exit_rc_map_vega_s95(void)
{
	rc_map_unregister(&vega_s95_map);
}

module_init(init_rc_map_vega_s95)
module_exit(exit_rc_map_vega_s95)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Christian Hewitt <christianshewitt@gmail.com>");
