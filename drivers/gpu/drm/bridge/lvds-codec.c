// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 Renesas Electronics Corporation
 * Copyright (C) 2016 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_panel.h>

struct lvds_codec {
	struct drm_bridge bridge;
	struct drm_bridge *panel_bridge;
	struct gpio_desc *powerdown_gpio;
	u32 connector_type;
	u32 input_fmt;
};

static int lvds_codec_attach(struct drm_bridge *bridge)
{
	struct lvds_codec *lvds_codec = container_of(bridge,
						     struct lvds_codec, bridge);

	return drm_bridge_attach(bridge->encoder, lvds_codec->panel_bridge,
				 bridge);
}

static void lvds_codec_enable(struct drm_bridge *bridge)
{
	struct lvds_codec *lvds_codec = container_of(bridge,
						     struct lvds_codec, bridge);

	if (lvds_codec->powerdown_gpio)
		gpiod_set_value_cansleep(lvds_codec->powerdown_gpio, 0);
}

static void lvds_codec_disable(struct drm_bridge *bridge)
{
	struct lvds_codec *lvds_codec = container_of(bridge,
						     struct lvds_codec, bridge);

	if (lvds_codec->powerdown_gpio)
		gpiod_set_value_cansleep(lvds_codec->powerdown_gpio, 1);
}

static u32 *
lvds_codec_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
				     struct drm_bridge_state *bridge_state,
				     struct drm_crtc_state *crtc_state,
				     struct drm_connector_state *conn_state,
				     u32 output_fmt,
				     unsigned int *num_input_fmts)
{
	struct lvds_codec *lvds_codec = container_of(bridge,
						     struct lvds_codec, bridge);
	u32 *input_fmts;

	input_fmts = kmalloc(sizeof(*input_fmts), GFP_KERNEL);
	if (!input_fmts) {
		*num_input_fmts = 0;
		return NULL;
	}

	*num_input_fmts = 1;
	input_fmts[0] = lvds_codec->input_fmt;
	return input_fmts;
}

static struct drm_bridge_funcs funcs = {
	.attach = lvds_codec_attach,
	.enable = lvds_codec_enable,
	.disable = lvds_codec_disable,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_get_input_bus_fmts = lvds_codec_atomic_get_input_bus_fmts,
};

static int lvds_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct drm_panel *panel;
	struct lvds_codec *lvds_codec;
	u32 input_bus_width;
	int err;

	lvds_codec = devm_kzalloc(dev, sizeof(*lvds_codec), GFP_KERNEL);
	if (!lvds_codec)
		return -ENOMEM;

	lvds_codec->connector_type = (uintptr_t)of_device_get_match_data(dev);
	lvds_codec->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							     GPIOD_OUT_HIGH);
	if (IS_ERR(lvds_codec->powerdown_gpio)) {
		err = PTR_ERR(lvds_codec->powerdown_gpio);

		if (err != -EPROBE_DEFER)
			dev_err(dev, "powerdown GPIO failure: %d\n", err);
		return err;
	}

	np = of_graph_get_port_by_id(dev->of_node, 0);
	if (!np) {
		dev_dbg(dev, "port 0 not found\n");
		return -ENXIO;
	}

	err = of_property_read_u32(np, "bus-width", &input_bus_width);
	of_node_put(np);

	if (err) {
		lvds_codec->input_fmt = MEDIA_BUS_FMT_FIXED;
	} else if (input_bus_width == 18) {
		lvds_codec->input_fmt = MEDIA_BUS_FMT_RGB666_1X18;
	} else if (input_bus_width == 24) {
		lvds_codec->input_fmt = MEDIA_BUS_FMT_RGB888_1X24;
	} else {
		dev_dbg(dev, "unsupported bus-width value %u on port 0\n",
			input_bus_width);
		return -ENOTSUPP;
	}

	/* Locate the panel DT node. */
	np = of_graph_get_remote_node(dev->of_node, 1, 0);
	if (!np) {
		dev_dbg(dev, "panel DT node not found\n");
		return -ENXIO;
	}

	panel = of_drm_find_panel(np);
	of_node_put(np);
	if (IS_ERR(panel)) {
		dev_dbg(dev, "panel not found, deferring probe\n");
		return PTR_ERR(panel);
	}

	lvds_codec->panel_bridge =
		devm_drm_panel_bridge_add_typed(dev, panel,
						lvds_codec->connector_type);
	if (IS_ERR(lvds_codec->panel_bridge))
		return PTR_ERR(lvds_codec->panel_bridge);

	/*
	 * The panel_bridge bridge is attached to the panel's of_node,
	 * but we need a bridge attached to our of_node for our user
	 * to look up.
	 */
	lvds_codec->bridge.of_node = dev->of_node;
	lvds_codec->bridge.funcs = &funcs;
	drm_bridge_add(&lvds_codec->bridge);

	platform_set_drvdata(pdev, lvds_codec);

	return 0;
}

static int lvds_codec_remove(struct platform_device *pdev)
{
	struct lvds_codec *lvds_codec = platform_get_drvdata(pdev);

	drm_bridge_remove(&lvds_codec->bridge);

	return 0;
}

static const struct of_device_id lvds_codec_match[] = {
	{
		.compatible = "lvds-decoder",
		.data = (void *)DRM_MODE_CONNECTOR_DPI,
	},
	{
		.compatible = "lvds-encoder",
		.data = (void *)DRM_MODE_CONNECTOR_LVDS,
	},
	{
		.compatible = "thine,thc63lvdm83d",
		.data = (void *)DRM_MODE_CONNECTOR_LVDS,
	},
	{},
};
MODULE_DEVICE_TABLE(of, lvds_codec_match);

static struct platform_driver lvds_codec_driver = {
	.probe	= lvds_codec_probe,
	.remove	= lvds_codec_remove,
	.driver		= {
		.name		= "lvds-codec",
		.of_match_table	= lvds_codec_match,
	},
};
module_platform_driver(lvds_codec_driver);

MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_DESCRIPTION("LVDS encoders and decoders");
MODULE_LICENSE("GPL");
