/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *    Cyril Raju <craju@nvidia.com>
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


#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/version.h>

#include <linux/backlight.h>
#include <linux/iio/consumer.h>
#include <dt-bindings/thermal/tegra-skin-thermal.h>

struct backlight_feature {
	struct backlight_device *back_dev;
	int val_rc;
};

struct iio_feature {
	struct iio_channel *iio_c;
	int val_rc;
};

struct power_feature {
	int type;
	int rc_k;
	int resistance;
	/*
	 * data pointer specific to corresponding feature type
	 */
	void *feature_data;
	struct list_head node;
};

struct tegra_skin_thermal_hotspot {
	int hotspot_rc;
	int offset;
	struct thermal_zone_device *ref_tzd;
	int rc_k;
	struct list_head node;
	struct list_head power_feature_list;
};

struct tegra_skin_thermal_sensor {
	struct thermal_zone_device *tzd;
	struct list_head node;
	struct list_head hotspot_list;
};

struct tegra_skin_thermal {
	struct device *dev;
	struct list_head sensor_list;
};

static int tegra_skin_hotspot_get_temp(
				struct tegra_skin_thermal_hotspot *htspt,
				int *out_temp)
{
	struct power_feature *cur; /* iterator */
	int delta_temp = 0, err = 0;
	int ref_temp = THERMAL_TEMP_INVALID;
	int temp = THERMAL_TEMP_INVALID;

	if (htspt == NULL) {
		temp = THERMAL_TEMP_INVALID;
		err = -EINVAL;
		goto out;
	}

	if (!htspt->ref_tzd || !htspt->ref_tzd->ops->get_temp ||
		htspt->ref_tzd->ops->get_temp(htspt->ref_tzd, &ref_temp)) {
		pr_warn("tegra-skin: Reference sensor missing.\n");
		temp = THERMAL_TEMP_INVALID;
		err = -EINVAL;
		goto out;
	}

	list_for_each_entry(cur, &htspt->power_feature_list, node) {
		if (cur->type == BACKLIGHT_BRIGHTNESS) {
			struct backlight_feature *back_f = cur->feature_data;
			int val;

			val = back_f->back_dev->props.brightness;
			back_f->val_rc += (val - back_f->val_rc);
			back_f->val_rc *= cur->rc_k;
			delta_temp += cur->resistance * back_f->val_rc;
		} else if (cur->type == IIO_SUBSYSTEM) {
			struct iio_feature *iio_f = cur->feature_data;
			int val;

			err = iio_read_channel_processed(iio_f->iio_c, &val);
			if (err < 0) {
				pr_warn("tegra-skin: iio read failed\n");
				temp = THERMAL_TEMP_INVALID;
				err = -EINVAL;
				goto out;
			}

			iio_f->val_rc += (val - iio_f->val_rc);
			iio_f->val_rc *= cur->rc_k;
			delta_temp += cur->resistance * iio_f->val_rc;
		}
	}

	delta_temp += htspt->offset;
	htspt->hotspot_rc += ref_temp - delta_temp - htspt->hotspot_rc;
	htspt->hotspot_rc *= htspt->rc_k;
	temp = htspt->hotspot_rc;

out:
	*out_temp = temp;
	return err;
}

static int tegra_skin_sensor_get_temp(void *data, int *out_temp)
{
	struct tegra_skin_thermal_sensor *sensor = data;
	struct tegra_skin_thermal_hotspot *cur;
	int err, temp;
	int max_temp = THERMAL_TEMP_INVALID; /* By default returning max */

	list_for_each_entry(cur, &sensor->hotspot_list, node) {
		err = tegra_skin_hotspot_get_temp(cur, &temp);
		if (err < 0)
			goto out;
		else
			err = 0;

		if (temp > max_temp)
			max_temp = temp;
	}

	*out_temp = max_temp;
out:
	return err;
}

static int tegra_skin_sensor_get_trend(void *data, int trip,
				enum thermal_trend *trend)
{
	struct tegra_skin_thermal_sensor *sensor = data;
	int ret;
	int trip_temp, temp, last_temp;

	if (!sensor->tzd)
		return -ENODEV;

	ret = sensor->tzd->ops->get_trip_temp(sensor->tzd, trip, &trip_temp);
	if (ret)
		return ret;

	mutex_lock(&sensor->tzd->lock);
	temp = sensor->tzd->temperature;
	last_temp = sensor->tzd->last_temperature;
	mutex_unlock(&sensor->tzd->lock);

	if (temp > trip_temp) {
		if (temp >= last_temp)
			*trend = THERMAL_TREND_RAISING;
		else
			*trend = THERMAL_TREND_STABLE;
	} else if (temp < trip_temp)
		*trend = THERMAL_TREND_DROPPING;
	else
		*trend = THERMAL_TREND_STABLE;

	return 0;
}

static const struct thermal_zone_of_device_ops tegra_of_thermal_ops = {
	.get_temp = tegra_skin_sensor_get_temp,
	.get_trend = tegra_skin_sensor_get_trend,
};

static int of_tegra_skin_power_feature_parse(
				struct platform_device *pdev,
				struct tegra_skin_thermal_hotspot *hotspot,
				struct device_node *child)
{
	struct of_phandle_args out_args;
	int i, of_err, num;
	struct tegra_skin_thermal *tskin = platform_get_drvdata(pdev);

	num = of_property_count_u32_elems(child, "power-features-list");
	if (num <= 0) {
		pr_err("tegra-skin: power-features-list incorrect/missing/empty\n");
		of_err = num;
		goto out;
	}

	for (i = 0; i < num; i++) {
		struct power_feature *pf;
		struct backlight_feature *back_f;
		struct device_node *node;
		struct iio_feature *iio_f;
		const char *str = NULL;

		of_err = of_parse_phandle_with_args(child,
						"power-features-list",
						"#power-feature-cells", i,
						&out_args);
		if (of_err == -ENOENT) {
			of_err = 0;
			break;
		} else if (of_err != 0) {
			pr_err("tegra-skin: power-features-list parse fail\n");
			goto out;
		}

		pf = devm_kzalloc(tskin->dev, sizeof(*pf), GFP_KERNEL);
		if (!pf) {
			of_err = -ENOMEM;
			goto out;
		}

		of_err = of_property_read_u32(out_args.np, "type", &pf->type);
		if (of_err || pf->type < 0 || pf->type >= FEATURE_TYPE_COUNT) {
			pr_err("tegra-skin: 'type' prop missing/incorrect\n");
			goto out;
		}

		of_err = of_property_read_u32(out_args.np, "resistance",
					&pf->resistance);
		if (of_err) {
			pr_err("tegra-skin: 'resistance' prop missing\n");
			goto out;
		}

		of_err = of_property_read_u32(out_args.np, "rc_k", &pf->rc_k);
		if (of_err) {
			pr_err("tegra-skin: 'rc_k' feature prop missing\n");
			goto out;
		}

		switch (pf->type) {
		case BACKLIGHT_BRIGHTNESS:
			back_f =  devm_kzalloc(tskin->dev, sizeof(*back_f),
					GFP_KERNEL);
			if (!back_f) {
				of_err = -ENOMEM;
				goto out;
			}

			node = of_find_node_by_phandle(out_args.args[0]);
			if (node) {
				back_f->back_dev =
					of_find_backlight_by_node(node);
				if (!back_f->back_dev) {
					of_err = -EPROBE_DEFER;
					pr_err("tegra-skin: of_find_backlight_by_node failed\n");
					goto out;
				}
			}

			pf->feature_data = (void *)back_f;
			break;
		case IIO_SUBSYSTEM:
			iio_f =  devm_kzalloc(tskin->dev, sizeof(*iio_f),
					GFP_KERNEL);
			if (!iio_f) {
				of_err = -ENOMEM;
				pr_err("tegra-skin: No memory\n");
				goto out;
			}

			of_err = of_property_read_string_index(
						pdev->dev.of_node,
						"io-channel-names",
						out_args.args[0], &str);
			if (of_err) {
				pr_err("tegra-skin: 'io-channel-names' parse failed\n");
				goto out;
			}

			iio_f->iio_c = iio_channel_get(&pdev->dev, str);
			if (!iio_f->iio_c || IS_ERR(iio_f->iio_c)) {
				of_err = -EINVAL;
				pr_err("tegra-skin: iio_channel_get failed\n");
				goto out;
			}

			pf->feature_data = (void *) iio_f;
			break;
		default:
			of_err = -EINVAL;
			pr_err("tegra-skin: Invalid type\n");
			goto out;
		}
		list_add(&pf->node, &hotspot->power_feature_list);
	}
out:
	return of_err;
}

static int of_tegra_skin_hotspot_parse(
				struct platform_device *pdev,
				struct tegra_skin_thermal_hotspot *hotspot,
				struct device_node *dev_node)
{
	int of_err;
	struct device_node *tz_np;

	of_err = of_property_read_u32(dev_node, "offset",
				&hotspot->offset);
	if (of_err) {
		pr_err("tegra-skin: 'offset' prop missing\n");
		goto out;
	}

	of_err = of_property_read_u32(dev_node, "rc_k", &hotspot->rc_k);
	if (of_err) {
		pr_err("tegra-skin: 'rc_k' prop missing\n");
		goto out;
	}

	tz_np = of_parse_phandle(dev_node, "reference-sensor", 0);

	if (tz_np) {
		hotspot->ref_tzd = thermal_zone_get_zone_by_node(tz_np);
		if (IS_ERR(hotspot->ref_tzd)) {
			pr_err("tegra-skin: 'reference-sensor' missing. Defer Probe!\n");
			of_err = -EPROBE_DEFER;
			goto out;
		}
	}
	of_err = of_tegra_skin_power_feature_parse(pdev, hotspot, dev_node);
	if (of_err) {
		pr_err("tegra-skin: Power feature of parse failed\n");
		goto out;
	}

out:
	return of_err;
}

static int of_tegra_skin_hotspots_parse(
				struct platform_device *pdev,
				struct tegra_skin_thermal_sensor *sensor,
				struct device_node *child)
{
	int num, of_err = 0, i;
	struct device_node *htspt_node;
	struct tegra_skin_thermal *tskin = platform_get_drvdata(pdev);

	num = of_property_count_u32_elems(child, "hotspot-list");
	if (num <= 0) {
		pr_err("tegra-skin: hotspot-list incorrect/missing/empty\n");
		of_err = num;
		goto out;
	}

	for (i = 0; i < num; i++) {
		struct tegra_skin_thermal_hotspot *hotspot;

		htspt_node = of_parse_phandle(child, "hotspot-list", i);
		if (IS_ERR_OR_NULL(htspt_node)) {
			pr_err("tegra-skin: Property 'hotspot-list' failed\n");
			of_err = PTR_ERR(htspt_node);
			goto out;
		}

		hotspot = devm_kzalloc(tskin->dev, sizeof(*hotspot),
				GFP_KERNEL);
		if (!hotspot) {
			of_err = -ENOMEM;
			goto out;
		}

		INIT_LIST_HEAD(&hotspot->power_feature_list);

		of_err = of_tegra_skin_hotspot_parse(pdev, hotspot, htspt_node);
		if (of_err) {
			pr_err("tegra-skin: Failed to parse hotspot\n");
			goto out;
		}

		list_add(&hotspot->node, &sensor->hotspot_list);
	}

out:
	return of_err;
}

static void free_sensor(struct platform_device *pdev,
		struct tegra_skin_thermal_sensor *sensor)
{
	struct tegra_skin_thermal_hotspot *h;
	struct power_feature *pf;

	list_for_each_entry(h, &sensor->hotspot_list, node) {
		list_for_each_entry(pf, &h->power_feature_list, node) {
			devm_kfree(&pdev->dev, pf->feature_data);
			devm_kfree(&pdev->dev, pf);
		}
		devm_kfree(&pdev->dev, h);
	}
	devm_kfree(&pdev->dev, sensor);
}

static int of_tegra_skin_parse(struct platform_device *pdev)
{
	int of_err = 0;
	struct device_node *dn = pdev->dev.of_node;
	struct device_node *child = NULL;
	struct thermal_zone_device *tzd;
	int idx = 0;
	struct tegra_skin_thermal *tskin = platform_get_drvdata(pdev);

	INIT_LIST_HEAD(&tskin->sensor_list);

	for_each_child_of_node(dn, child) {
		struct tegra_skin_thermal_sensor *sensor;

		if (!of_property_read_bool(child, "thermal-sensor"))
			continue;

		pr_info("tegra-skin: Parsing sensor %s\n", child->full_name);
		sensor = devm_kzalloc(tskin->dev, sizeof(*sensor),
				GFP_KERNEL);
		if (!sensor) {
			of_err = -ENOMEM;
			goto out;
		}

		INIT_LIST_HEAD(&sensor->hotspot_list);

		of_err = of_property_read_u32(child, "thermal-sensor", &idx);
		if (of_err || idx < 0) {
			pr_err("tegra-skin: 'thermal-sensor' prop incorrect\n");
			of_err = -EINVAL;
			goto free_sensor;
		}

		of_err = of_tegra_skin_hotspots_parse(pdev, sensor, child);
		if (of_err) {
			if (of_err == -EPROBE_DEFER)
				goto out;
			pr_err("tegra-skin: Hotspots parse failed\n");
			goto free_sensor;
		}

		tzd = thermal_zone_of_sensor_register(tskin->dev, idx, sensor,
						&tegra_of_thermal_ops);
		if (IS_ERR_OR_NULL(tzd)) {
			pr_err("tegra-skin: Sensor registration failed\n");
			of_err = PTR_ERR(tzd);
			goto free_sensor;
		}

		sensor->tzd = tzd;
		list_add(&sensor->node, &tskin->sensor_list);

		continue;
free_sensor:
		pr_warn("tegra-skin: Freeing sensor %s (%d)\n",
					child->full_name, of_err);
		of_err = 0;
		free_sensor(pdev, sensor);
	}

out:
	return of_err;
}
static void free_resources(struct platform_device *pdev)
{
	struct tegra_skin_thermal *tskin = platform_get_drvdata(pdev);
	struct tegra_skin_thermal_sensor *cur;

	list_for_each_entry(cur, &tskin->sensor_list, node)
		thermal_zone_of_sensor_unregister(&pdev->dev, cur->tzd);

}
static int tegra_skin_thermal_probe(struct platform_device *pdev)
{
	int err = 0;
	struct tegra_skin_thermal *tskin = NULL;

	tskin = devm_kzalloc(&pdev->dev, sizeof(*tskin), GFP_KERNEL);
	if (!tskin) {
		err = -ENOMEM;
		goto out;
	}

	tskin->dev = &pdev->dev;
	platform_set_drvdata(pdev, tskin);

	err = of_tegra_skin_parse(pdev);
	if (err) {
		pr_err("tegra-skin: tegra_skin_thermal_probe failed\n");
		goto free_queue;
	}

	pr_info("tegra-skin: tegra_skin_thermal registered!\n");

	return err;

free_queue:
	free_resources(pdev);

out:
	return err;
}

static int tegra_skin_thermal_remove(struct platform_device *pdev)
{
	free_resources(pdev);
	return 0;
}

static const struct of_device_id tegra_skin_thermal_of_match[] = {
	{
		.compatible = "nvidia,tegra-skin-thermal",
	},
	{ },
};

static struct platform_driver tegra_skin_thermal_driver = {
	.probe = tegra_skin_thermal_probe,
	.remove = tegra_skin_thermal_remove,
	.driver = {
		.name = "tegra-skin-thermal",
		.of_match_table = tegra_skin_thermal_of_match,
	},
};

module_platform_driver(tegra_skin_thermal_driver);

MODULE_AUTHOR("Cyril Raju <craju@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra skin sensor driver");
MODULE_LICENSE("GPL");
