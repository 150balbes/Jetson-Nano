// SPDX-License-Identifier: GPL-2.0
/*
 * dw-hdmi-i2s-audio.c
 *
 * Copyright (c) 2017 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>

#include <drm/bridge/dw_hdmi.h>

#include <sound/hdmi-codec.h>

#include "dw-hdmi.h"
#include "dw-hdmi-audio.h"

#define DRIVER_NAME "dw-hdmi-i2s-audio"

static inline void hdmi_write(struct dw_hdmi_i2s_audio_data *audio,
			      u8 val, int offset)
{
	struct dw_hdmi *hdmi = audio->hdmi;

	audio->write(hdmi, val, offset);
}

static inline u8 hdmi_read(struct dw_hdmi_i2s_audio_data *audio, int offset)
{
	struct dw_hdmi *hdmi = audio->hdmi;

	return audio->read(hdmi, offset);
}

static inline void hdmi_update_bits(struct dw_hdmi_i2s_audio_data *audio,
				    u8 data, u8 mask, unsigned int reg)
{
	struct dw_hdmi *hdmi = audio->hdmi;

	audio->mod(hdmi, data, mask, reg);
}

static int dw_hdmi_i2s_hw_params(struct device *dev, void *data,
				 struct hdmi_codec_daifmt *fmt,
				 struct hdmi_codec_params *hparms)
{
	struct dw_hdmi_i2s_audio_data *audio = data;
	struct dw_hdmi *hdmi = audio->hdmi;
	u8 conf0 = 0;
	u8 conf1 = 0;
	u8 inputclkfs = 0;
	u8 val;

	/* it cares I2S only */
	if ((fmt->fmt != HDMI_I2S) ||
	    (fmt->bit_clk_master | fmt->frame_clk_master)) {
		dev_err(dev, "unsupported format/settings\n");
		return -EINVAL;
	}

	inputclkfs = HDMI_AUD_INPUTCLKFS_64FS;

	switch (hparms->channels) {
	case 2:
		conf0 = HDMI_AUD_CONF0_I2S_2CHANNEL_ENABLE;
		break;
	case 4:
		conf0 = HDMI_AUD_CONF0_I2S_4CHANNEL_ENABLE;
		break;
	case 6:
		conf0 = HDMI_AUD_CONF0_I2S_6CHANNEL_ENABLE;
		break;
	case 8:
	default:
		conf0 = HDMI_AUD_CONF0_I2S_ALL_ENABLE;
		break;
	}

	switch (hparms->sample_width) {
	case 16:
		conf1 = HDMI_AUD_CONF1_WIDTH_16;
		break;
	case 24:
	case 32:
		conf1 = HDMI_AUD_CONF1_WIDTH_24;
		break;
	}

	hdmi_update_bits(audio, HDMI_AUD_CONF0_SW_RESET,
			 HDMI_AUD_CONF0_SW_RESET, HDMI_AUD_CONF0);
	hdmi_write(audio, (u8)~HDMI_MC_SWRSTZ_I2SSWRST_REQ, HDMI_MC_SWRSTZ);

	dw_hdmi_set_sample_rate(hdmi, hparms->sample_rate);

	hdmi_write(audio, inputclkfs, HDMI_AUD_INPUTCLKFS);
	hdmi_write(audio, conf0, HDMI_AUD_CONF0);
	hdmi_write(audio, conf1, HDMI_AUD_CONF1);

	val = HDMI_FC_AUDSCONF_AUD_PACKET_LAYOUT_LAYOUT0;
	if (hparms->channels > 2)
		val = HDMI_FC_AUDSCONF_AUD_PACKET_LAYOUT_LAYOUT1;
	hdmi_update_bits(audio, val, HDMI_FC_AUDSCONF_AUD_PACKET_LAYOUT_MASK,
			 HDMI_FC_AUDSCONF);

	switch (hparms->sample_rate) {
	case 32000:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_32K;
		break;
	case 44100:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_441K;
		break;
	case 48000:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_48K;
		break;
	case 88200:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_882K;
		break;
	case 96000:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_96K;
		break;
	case 176400:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_1764K;
		break;
	case 192000:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_192K;
		break;
	default:
		val = HDMI_FC_AUDSCHNLS_SAMPFREQ_441K;
		break;
	}

	hdmi_update_bits(audio, val, HDMI_FC_AUDSCHNLS7_SAMPFREQ_MASK,
			 HDMI_FC_AUDSCHNLS7);
	hdmi_update_bits(audio,
			 (hparms->channels - 1) << HDMI_FC_AUDICONF0_CC_OFFSET,
			 HDMI_FC_AUDICONF0_CC_MASK, HDMI_FC_AUDICONF0);
	hdmi_write(audio, hparms->cea.channel_allocation, HDMI_FC_AUDICONF2);

	dw_hdmi_audio_enable(hdmi);

	return 0;
}

static void dw_hdmi_i2s_audio_shutdown(struct device *dev, void *data)
{
	struct dw_hdmi_i2s_audio_data *audio = data;
	struct dw_hdmi *hdmi = audio->hdmi;

	dw_hdmi_audio_disable(hdmi);

	hdmi_write(audio, HDMI_AUD_CONF0_SW_RESET, HDMI_AUD_CONF0);
	hdmi_write(audio, (u8)~HDMI_MC_SWRSTZ_I2SSWRST_REQ, HDMI_MC_SWRSTZ);
}

static int dw_hdmi_i2s_digital_mute(struct device *dev, void *data, bool enable)
{
	struct dw_hdmi_i2s_audio_data *audio = data;

	hdmi_update_bits(audio, enable ? 0 : 0xf,
			 HDMI_FC_AUDSCONF_AUD_PACKET_SAMPFIT_MASK,
			 HDMI_FC_AUDSCONF);

	return 0;
}

static void dw_hdmi_i2s_update_eld(struct device *dev, u8 *eld)
{
	struct dw_hdmi_i2s_audio_data *audio = dev_get_platdata(dev);
	struct platform_device *hcpdev = dev_get_drvdata(dev);

	if (!audio || !hcpdev)
		return;

	if (!memcmp(audio->eld, eld, sizeof(audio->eld)))
		return;

	memcpy(audio->eld, eld, sizeof(audio->eld));

	hdmi_codec_eld_notify(&hcpdev->dev);
}

static int dw_hdmi_i2s_get_eld(struct device *dev, void *data,
			       u8 *buf, size_t len)
{
	struct dw_hdmi_i2s_audio_data *audio = data;

	memcpy(buf, audio->eld, min(sizeof(audio->eld), len));

	return 0;
}

static int dw_hdmi_i2s_get_dai_id(struct snd_soc_component *component,
				  struct device_node *endpoint)
{
	struct of_endpoint of_ep;
	int ret;

	ret = of_graph_parse_endpoint(endpoint, &of_ep);
	if (ret < 0)
		return ret;

	/*
	 * HDMI sound should be located as reg = <2>
	 * Then, it is sound port 0
	 */
	if (of_ep.port == 2)
		return 0;

	return -EINVAL;
}

static struct hdmi_codec_ops dw_hdmi_i2s_ops = {
	.hw_params	= dw_hdmi_i2s_hw_params,
	.audio_shutdown	= dw_hdmi_i2s_audio_shutdown,
	.digital_mute	= dw_hdmi_i2s_digital_mute,
	.get_eld	= dw_hdmi_i2s_get_eld,
	.get_dai_id	= dw_hdmi_i2s_get_dai_id,
};

static int snd_dw_hdmi_probe(struct platform_device *pdev)
{
	struct dw_hdmi_i2s_audio_data *audio = dev_get_platdata(&pdev->dev);
	struct platform_device_info pdevinfo;
	struct hdmi_codec_pdata pdata;
	struct platform_device *platform;

	memset(audio->eld, 0, sizeof(audio->eld));

	pdata.ops		= &dw_hdmi_i2s_ops;
	pdata.i2s		= 1;
	pdata.max_i2s_channels	= 8;
	pdata.data		= audio;

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.parent		= pdev->dev.parent;
	pdevinfo.id		= PLATFORM_DEVID_AUTO;
	pdevinfo.name		= HDMI_CODEC_DRV_NAME;
	pdevinfo.data		= &pdata;
	pdevinfo.size_data	= sizeof(pdata);
	pdevinfo.dma_mask	= DMA_BIT_MASK(32);

	platform = platform_device_register_full(&pdevinfo);
	if (IS_ERR(platform))
		return PTR_ERR(platform);

	dev_set_drvdata(&pdev->dev, platform);

	dw_hdmi_set_update_eld(audio->hdmi, dw_hdmi_i2s_update_eld);

	return 0;
}

static int snd_dw_hdmi_remove(struct platform_device *pdev)
{
	struct dw_hdmi_i2s_audio_data *audio = dev_get_platdata(&pdev->dev);
	struct platform_device *platform = dev_get_drvdata(&pdev->dev);

	dw_hdmi_set_update_eld(audio->hdmi, NULL);

	platform_device_unregister(platform);

	return 0;
}

static struct platform_driver snd_dw_hdmi_driver = {
	.probe	= snd_dw_hdmi_probe,
	.remove	= snd_dw_hdmi_remove,
	.driver	= {
		.name = DRIVER_NAME,
	},
};
module_platform_driver(snd_dw_hdmi_driver);

MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
MODULE_DESCRIPTION("Synopsis Designware HDMI I2S ALSA SoC interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
