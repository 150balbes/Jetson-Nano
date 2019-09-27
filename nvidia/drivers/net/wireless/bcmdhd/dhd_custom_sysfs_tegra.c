/*
 * drivers/net/wireless/bcmdhd/dhd_custom_sysfs_tegra.c
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2014-2018 NVIDIA Corporation. All rights reserved.
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

#include <linux/errno.h>
#include <linux/system-wakeup.h>
#include "dhd_custom_sysfs_tegra.h"
#include "dhd_custom_sysfs_tegra_scan.h"
#include "dhd_custom_sysfs_tegra_stat.h"


int lp0_logs_enable = 1;
const char string_suspend[] = "suspend called";
const char string_ctrl_pkt[] = "control pkt";
const char string_resume[] = "resume called";
const char string_dpc_pkt[] = "dpc called";
const char dummy_inf[] = "dummy:";
char wifi_product_value[20] = "";
int bcmdhd_irq_number;
atomic_t tegra_downgrade_ac = ATOMIC_INIT(0);

extern int bcmdhd_resume_trigger;

static DEVICE_ATTR(rssi, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_rssi_show,
	tegra_sysfs_histogram_rssi_store);

static DEVICE_ATTR(scan, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_scan_show,
	tegra_sysfs_histogram_scan_store);

static DEVICE_ATTR(stat, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_stat_show,
	tegra_sysfs_histogram_stat_store);

static DEVICE_ATTR(tcpdump, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_tcpdump_show,
	tegra_sysfs_histogram_tcpdump_store);

static struct attribute *tegra_sysfs_entries_histogram[] = {
	&dev_attr_rssi.attr,
	&dev_attr_scan.attr,
	&dev_attr_stat.attr,
	&dev_attr_tcpdump.attr,
	NULL,
};

static struct attribute_group tegra_sysfs_group_histogram = {
	.name = "histogram",
	.attrs = tegra_sysfs_entries_histogram,
};

/* RF test attributes */
static DEVICE_ATTR(state, S_IRUGO | S_IWUSR,
	tegra_sysfs_rf_test_state_show,
	tegra_sysfs_rf_test_state_store);

static struct attribute *tegra_sysfs_entries_rf_test[] = {
	&dev_attr_state.attr,
	NULL,
};

static struct attribute_group tegra_sysfs_group_rf_test = {
	.name = "rf_test",
	.attrs = tegra_sysfs_entries_rf_test,
};

/* End RF test attributes */

static struct dentry *tegra_debugfs_root;

static struct file_operations tegra_debugfs_histogram_scan_fops = {
	.read = tegra_debugfs_histogram_scan_read,
	.write = tegra_debugfs_histogram_scan_write,
};

static struct file_operations tegra_debugfs_histogram_tcpdump_fops = {
	.read = tegra_debugfs_histogram_tcpdump_read,
	.write = tegra_debugfs_histogram_tcpdump_write,
};

/* Hostapd attribues */
static DEVICE_ATTR(downgradevotovi, S_IRUGO | S_IWUSR,
	tegra_sysfs_hostapd_downgradevotovi_show,
	tegra_sysfs_hostapd_downgradevotovi_store);

static struct attribute *tegra_sysfs_entries_hostapd[] = {
	&dev_attr_downgradevotovi.attr,
	NULL,
};

static struct attribute_group tegra_sysfs_group_hostapd = {
	.name = "hostapd",
	.attrs = tegra_sysfs_entries_hostapd,
};

/* End Hostapd attributes */

/* Wifi product attributes */
static DEVICE_ATTR(wifi_product, S_IRUGO | S_IWUSR,
	tegra_sysfs_wifi_product_show,
	tegra_sysfs_wifi_product_store);

/* End Wifi product attributes */

int
tegra_sysfs_register(struct device *dev)
{
	int err = 0;

	pr_info("%s\n", __func__);

	/* create sysfs */
	err = sysfs_create_group(&dev->kobj, &tegra_sysfs_group_histogram);
	if (err) {
		pr_err("%s: failed to create tegra sysfs group %s\n",
			__func__, tegra_sysfs_group_histogram.name);
		goto exit;
	}
	err = sysfs_create_group(&dev->kobj, &tegra_sysfs_group_rf_test);
	if (err) {
		pr_err("%s: failed to create tegra sysfs group %s\n",
			__func__, tegra_sysfs_group_rf_test.name);
		goto cleanup;
	}
	err = sysfs_create_group(&dev->kobj, &tegra_sysfs_group_hostapd);
	if (err) {
		pr_err("%s: failed to create tegra sysfs group %s\n",
			__func__, tegra_sysfs_group_hostapd.name);
		goto cleanup_rftest;
	}

	/* used to update clm blob path */
	err = sysfs_create_file(&dev->kobj, &dev_attr_wifi_product.attr);
	if (err) {
		pr_err("%s: failed to create wifi product sysfs file\n",
			 __func__);
		goto cleanup_hostapd;
	}

	/* create debugfs */
	tegra_debugfs_root = debugfs_create_dir("bcmdhd_histogram", NULL);
	if (tegra_debugfs_root) {
		debugfs_create_file("scan", S_IRUGO,
			tegra_debugfs_root, (void *) 0,
			&tegra_debugfs_histogram_scan_fops);
		debugfs_create_file("tcpdump", S_IRUGO,
			tegra_debugfs_root, (void *) 0,
			&tegra_debugfs_histogram_tcpdump_fops);
	}

	/* start sysfs work */
#if 0
	tegra_sysfs_histogram_rssi_work_start();
	tegra_sysfs_histogram_scan_work_start();
	tegra_sysfs_histogram_stat_work_start();
	tegra_sysfs_histogram_tcpdump_work_start();
#endif
	goto exit;

cleanup_hostapd:
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_hostapd);

cleanup_rftest:
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_rf_test);
cleanup:
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_histogram);
exit:
	return err;
}

void
tegra_sysfs_unregister(struct device *dev)
{
	pr_info("%s\n", __func__);

	/* stop sysfs work */
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_stat_work_stop();
	tegra_sysfs_histogram_scan_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();

	/* remove debugfs */
	if (tegra_debugfs_root) {
		debugfs_remove_recursive(tegra_debugfs_root);
		tegra_debugfs_root = NULL;
	}

	/* remove sysfs */
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_hostapd);
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_rf_test);
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_histogram);
	sysfs_remove_file(&dev->kobj, &dev_attr_wifi_product.attr);
}

int tegra_sysfs_wifi_on;

void
tegra_sysfs_on(void)
{
	pr_info("%s\n", __func__);

	tegra_sysfs_wifi_on = 1;

	/* init scan work(s) in case prior shutdown did not clean up properly
	 */
	wifi_scan_request_init();

	/* resume (start) sysfs work */
	tegra_sysfs_histogram_rssi_work_start();
	tegra_sysfs_histogram_scan_work_start();
	tegra_sysfs_histogram_stat_work_start();
	tegra_sysfs_histogram_tcpdump_work_start();

}

void
tegra_sysfs_off(void)
{
	pr_info("%s\n", __func__);

	tegra_sysfs_rf_test_disable();
	tegra_sysfs_wifi_on = 0;

	/* suspend (stop) sysfs work */
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_stat_work_stop();
	tegra_sysfs_histogram_scan_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();

}

void
tegra_sysfs_suspend(void)
{
	pr_info("%s\n", __func__);

	/* do nothing if wifi off */
	if (!tegra_sysfs_wifi_on)
		return;

	/* suspend (stop) sysfs work */
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_stat_work_stop();
	tegra_sysfs_histogram_scan_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();

}

void
tegra_sysfs_resume(void)
{
	pr_info("%s\n", __func__);

	/* do nothing if wifi off */
	if (!tegra_sysfs_wifi_on)
		return;

	/* resume (start) sysfs work */
	tegra_sysfs_histogram_rssi_work_start();
	tegra_sysfs_histogram_scan_work_start();
	tegra_sysfs_histogram_stat_work_start();
	tegra_sysfs_histogram_tcpdump_work_start();
}

void
tegra_sysfs_resume_capture(void)
{
	if (lp0_logs_enable) {
		if (get_wakeup_reason_irq() != bcmdhd_irq_number)
			return;
		if (!bcmdhd_resume_trigger)
			bcmdhd_resume_trigger = 1;
		tcpdump_pkt_save('w', dummy_inf,
			__func__, __LINE__, string_resume,
			sizeof(string_resume), 0);
	}
}

void
tegra_sysfs_suspend_capture(void)
{
	if (lp0_logs_enable)
		tcpdump_pkt_save('w', dummy_inf,
		__func__, __LINE__, string_suspend,
		sizeof(string_suspend), 0);
}
void
tegra_sysfs_control_pkt(int number)
{
	if (lp0_logs_enable)
		tcpdump_pkt_save('w', dummy_inf,
		__func__, number, string_ctrl_pkt,
		sizeof(string_ctrl_pkt), 0);
}
void
tegra_sysfs_dpc_pkt(void)
{
	if (lp0_logs_enable)
		tcpdump_pkt_save('w', dummy_inf,
		__func__, __LINE__, string_dpc_pkt,
		sizeof(string_dpc_pkt), 0);
}

ssize_t
tegra_sysfs_hostapd_downgradevotovi_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	if (atomic_read(&tegra_downgrade_ac)) {
		strcpy(buf, "enabled\n");
		return strlen(buf);
	} else {
		strcpy(buf, "disabled\n");
		return strlen(buf);
	}
}

ssize_t
tegra_sysfs_hostapd_downgradevotovi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (strncmp(buf, "enable", 6) == 0) {
		if (!atomic_read(&tegra_downgrade_ac) && tegra_sysfs_wifi_on) {
			atomic_set(&tegra_downgrade_ac, 1);
		} else {
			pr_info("%s: operation not allowed\n", __func__);
		}
	} else if (strncmp(buf, "disable", 7) == 0) {
		if (atomic_read(&tegra_downgrade_ac) && tegra_sysfs_wifi_on) {
			atomic_set(&tegra_downgrade_ac, 0);
		} else {
			pr_info("%s: operation not allowed\n", __func__);
		}
	} else {
		pr_err("%s: unknown command\n", __func__);
	}

	return count;
}

ssize_t
tegra_sysfs_wifi_product_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	strncpy(buf, wifi_product_value, strlen(wifi_product_value));
	return strlen(buf);
}

ssize_t
tegra_sysfs_wifi_product_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (count > (sizeof(wifi_product_value)-1))
		return -EINVAL;
	memset(wifi_product_value, '\0', sizeof(wifi_product_value));
	strncpy(wifi_product_value, buf, count);
	return count;
}

void get_product_blob_path(char *blob_path, char **result)
{
	*result = kmalloc(strlen(blob_path) + strlen(wifi_product_value) + 2,
			GFP_KERNEL);
	if (*result == NULL) {
		pr_err("%s : Failed to allocate memory", __func__);
		return;
	}
	memset(*result, '\0', sizeof(char) * (strlen(blob_path)
		+ strlen(wifi_product_value) + 2));
	//Copy 5 characters less to remove ".blob" from blob_path
	strncpy(*result, blob_path, strlen(blob_path) - 5);
	strcat(*result, "_");
	strcat(*result, wifi_product_value);
	strcat(*result, ".blob");
}
