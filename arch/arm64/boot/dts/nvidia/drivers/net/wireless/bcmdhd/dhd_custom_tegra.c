/*
 * drivers/net/wireless/bcmdhd_pcie/dhd_custom_tegra.c
 *
 * Copyright (C) 2014-2016 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/types.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/of.h>

#include <bcmutils.h>
#include <linux_osl.h>
#include <dhd_dbg.h>
#include <dngl_stats.h>
#include <dhd.h>

#define WIFI_MAC_ADDR_FILE "/mnt/vendor/factory/wifi/wifi_mac.txt"

static int wifi_get_mac_addr_file(unsigned char *buf)
{
	struct file *fp;
	int rdlen;
	char str[32];
	int mac[6];
	int ret = 0;

	/* open wifi mac address file */
	fp = filp_open(WIFI_MAC_ADDR_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		DHD_ERROR(("%s: cannot open %s\n",
			__FUNCTION__, WIFI_MAC_ADDR_FILE));
		return -ENOENT;
	}

	/* read wifi mac address file */
	memset(str, 0, sizeof(str));
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 13, 0))
	rdlen = kernel_read(fp, str, 17, &fp->f_pos);
#else
	rdlen = kernel_read(fp, fp->f_pos, str, 17);
	if (rdlen > 0)
		fp->f_pos += rdlen;
#endif
	if (rdlen != 17) {
		DHD_ERROR(("%s: bad mac address file - len %d != 17",
						__FUNCTION__, rdlen));
		ret = -ENOENT;
	} else if (sscanf(str, "%x:%x:%x:%x:%x:%x",
		&mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
		DHD_ERROR(("%s: bad mac address file"
			" - must contain xx:xx:xx:xx:xx:xx\n",
			__FUNCTION__));
		ret = -ENOENT;
	} else {
		DHD_ERROR(("%s: using wifi mac %02x:%02x:%02x:%02x:%02x:%02x\n",
			__FUNCTION__,
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));
		buf[0] = (unsigned char) mac[0];
		buf[1] = (unsigned char) mac[1];
		buf[2] = (unsigned char) mac[2];
		buf[3] = (unsigned char) mac[3];
		buf[4] = (unsigned char) mac[4];
		buf[5] = (unsigned char) mac[5];
	}

	if (!is_valid_ether_addr(buf)) {
		DHD_ERROR(("%s: invalid mac %02x:%02x:%02x:%02x:%02x:%02x\n",
			__FUNCTION__,
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]));
		ret = -EINVAL;
	}

	/* close wifi mac address file */
	filp_close(fp, NULL);

	return ret;
}

/* Get MAC address from the specified DTB path */
static int wifi_get_mac_address_dtb(const char *node_name,
					const char *property_name,
					unsigned char *mac_addr)
{
	struct device_node *np = of_find_node_by_path(node_name);
	const char *mac_str = NULL;
	int values[6] = {0};
	unsigned char mac_temp[6] = {0};
	int i, ret = 0;

	if (!np)
		return -EADDRNOTAVAIL;

	/* If the property is present but contains an invalid value,
	 * then something is wrong. Log the error in that case.
	 */
	if (of_property_read_string(np, property_name, &mac_str)) {
		ret = -EADDRNOTAVAIL;
		goto err_out;
	}

	/* The DTB property is a string of the form xx:xx:xx:xx:xx:xx
	 * Convert to an array of bytes.
	 */
	if (sscanf(mac_str, "%x:%x:%x:%x:%x:%x",
		&values[0], &values[1], &values[2],
		&values[3], &values[4], &values[5]) != 6) {
		ret = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < 6; ++i)
		mac_temp[i] = (unsigned char)values[i];

	if (!is_valid_ether_addr(mac_temp)) {
		ret = -EINVAL;
		goto err_out;
	}

	memcpy(mac_addr, mac_temp, 6);

	of_node_put(np);

	return ret;

err_out:
	DHD_ERROR(("%s: bad mac address at %s/%s: %s.\n",
		__func__, node_name, property_name,
		(mac_str) ? mac_str : "null"));

	of_node_put(np);

	return ret;
}

int wifi_get_mac_addr(unsigned char *buf)
{
	int ret = -ENODATA;

	/* The MAC address search order is:
	 * DTB (from NCT/EEPROM)
	 * NCT
	 * File (FCT/rootfs)
	*/
	ret = wifi_get_mac_address_dtb("/chosen", "nvidia,wifi-mac", buf);
	if (ret)
		ret = wifi_get_mac_addr_file(buf);

	return ret;
}
