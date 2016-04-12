/*
 * Copyright (C) 2015 Andrea Venturi
 * Andrea Venturi <be17068@iperbole.bo.it>
 *
 * Copyright (C) 2016 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#define SUN4I_I2S_CTRL_REG		0x00
#define SUN4I_I2S_CTRL_SDO_EN_MASK		GENMASK(11, 8)
#define SUN4I_I2S_CTRL_SDO_EN(lines)		(((1 << lines) - 1) << 8)
#define SUN4I_I2S_CTRL_MODE_MASK		BIT(5)
#define SUN4I_I2S_CTRL_MODE_SLAVE			(1 << 5)
#define SUN4I_I2S_CTRL_MODE_MASTER			(0 << 5)
#define SUN4I_I2S_CTRL_LOOP			BIT(3)
#define SUN4I_I2S_CTRL_TX_EN			BIT(2)
#define SUN4I_I2S_CTRL_RX_EN			BIT(1)
#define SUN4I_I2S_CTRL_GL_EN			BIT(0)

#define SUN4I_I2S_FMT0_REG		0x04
#define SUN4I_I2S_FMT0_LRCLK_POLARITY_MASK	BIT(7)
#define SUN4I_I2S_FMT0_LRCLK_POLARITY_INVERTED		(1 << 7)
#define SUN4I_I2S_FMT0_LRCLK_POLARITY_NORMAL		(0 << 7)
#define SUN4I_I2S_FMT0_BCLK_POLARITY_MASK	BIT(6)
#define SUN4I_I2S_FMT0_BCLK_POLARITY_INVERTED		(1 << 6)
#define SUN4I_I2S_FMT0_BCLK_POLARITY_NORMAL		(0 << 6)
#define SUN4I_I2S_FMT0_SR_MASK			GENMASK(5, 4)
#define SUN4I_I2S_FMT0_SR(sr)				((sr) << 4)
#define SUN4I_I2S_FMT0_WSS_MASK			GENMASK(3, 2)
#define SUN4I_I2S_FMT0_WSS(wss)				((wss) << 2)
#define SUN4I_I2S_FMT0_FMT_MASK			GENMASK(1, 0)
#define SUN4I_I2S_FMT0_FMT_RIGHT_J			(2 << 0)
#define SUN4I_I2S_FMT0_FMT_LEFT_J			(1 << 0)
#define SUN4I_I2S_FMT0_FMT_I2S				(0 << 0)
#define SUN4I_I2S_FMT0_POLARITY_INVERTED		(1)
#define SUN4I_I2S_FMT0_POLARITY_NORMAL			(0)

#define SUN4I_I2S_FMT1_REG		0x08
#define SUN4I_I2S_FIFO_TX_REG		0x0c
#define SUN4I_I2S_FIFO_RX_REG		0x10

#define SUN4I_I2S_FIFO_CTRL_REG		0x14
#define SUN4I_I2S_FIFO_CTRL_FLUSH_TX		BIT(25)
#define SUN4I_I2S_FIFO_CTRL_FLUSH_RX		BIT(24)
#define SUN4I_I2S_FIFO_CTRL_TX_MODE_MASK	BIT(2)
#define SUN4I_I2S_FIFO_CTRL_TX_MODE(mode)		((mode) << 2)
#define SUN4I_I2S_FIFO_CTRL_RX_MODE_MASK	GENMASK(1, 0)
#define SUN4I_I2S_FIFO_CTRL_RX_MODE(mode)		(mode)

#define SUN4I_I2S_FIFO_STA_REG		0x18

#define SUN4I_I2S_DMA_INT_CTRL_REG	0x1c
#define SUN4I_I2S_DMA_INT_CTRL_TX_DRQ_EN	BIT(7)
#define SUN4I_I2S_DMA_INT_CTRL_RX_DRQ_EN	BIT(3)

#define SUN4I_I2S_INT_STA_REG		0x20

#define SUN4I_I2S_CLK_DIV_REG		0x24
#define SUN4I_I2S_CLK_DIV_MCLK_EN		BIT(7)
#define SUN4I_I2S_CLK_DIV_BCLK_MASK		GENMASK(6, 4)
#define SUN4I_I2S_CLK_DIV_BCLK(bclk)			((bclk) << 4)
#define SUN4I_I2S_CLK_DIV_MCLK_MASK		GENMASK(3, 0)
#define SUN4I_I2S_CLK_DIV_MCLK(mclk)			((mclk) << 0)

#define SUN4I_I2S_RX_CNT_REG		0x28
#define SUN4I_I2S_TX_CNT_REG		0x2c

#define SUN4I_I2S_TX_CHAN_SEL_REG	0x30
#define SUN4I_I2S_CHAN_SEL(num_chan)		(((num_chan) - 1) << 0)

#define SUN4I_I2S_TX_CHAN_MAP_REG	0x34
#define SUN4I_I2S_TX_CHAN_MAP(chan, sample)	((sample) << (chan << 2))

#define SUN4I_I2S_RX_CHAN_SEL_REG	0x38
#define SUN4I_I2S_RX_CHAN_MAP_REG	0x3c

/* Defines required for sun8i-h3 support */
#define SUN8I_I2S_CTRL_BCLK_OUT			BIT(18)
#define SUN8I_I2S_CTRL_LRCK_OUT			BIT(17)

#define SUN8I_I2S_FMT0_LRCK_PERIOD_MASK		GENMASK(17, 8)
#define SUN8I_I2S_FMT0_LRCK_PERIOD(period)	((period - 1) << 8)

#define SUN8I_I2S_INT_STA_REG		0x0c
#define SUN8I_I2S_FIFO_TX_REG		0x20

#define SUN8I_I2S_CHAN_CFG_REG		0x30
#define SUN8I_I2S_CHAN_CFG_RX_SLOT_NUM_MASK	GENMASK(6, 4)
#define SUN8I_I2S_CHAN_CFG_RX_SLOT_NUM(chan)	((chan - 1) << 4)
#define SUN8I_I2S_CHAN_CFG_TX_SLOT_NUM_MASK	GENMASK(2, 0)
#define SUN8I_I2S_CHAN_CFG_TX_SLOT_NUM(chan)	(chan - 1)

#define SUN8I_I2S_TX_CHAN_MAP_REG	0x44
#define SUN8I_I2S_TX_CHAN_SEL_REG	0x34
#define SUN8I_I2S_TX_CHAN_OFFSET_MASK		GENMASK(13, 12)
#define SUN8I_I2S_TX_CHAN_OFFSET(offset)	(offset << 12)
#define SUN8I_I2S_TX_CHAN_EN_MASK		GENMASK(11, 4)
#define SUN8I_I2S_TX_CHAN_EN(num_chan)		(((1 << num_chan) - 1) << 4)

#define SUN8I_I2S_RX_CHAN_SEL_REG	0x54
#define SUN8I_I2S_RX_CHAN_MAP_REG	0x58

/**
 * struct sun4i_i2s_quirks - Differences between SoC variants.
 *
 * @has_reset: SoC needs reset deasserted.
 * @has_slave_select_bit: SoC has a bit to enable slave mode.
 * @has_fmt_set_lrck_period: SoC requires lrclk period to be set.
 * @has_chcfg: tx and rx slot number need to be set.
 * @has_chsel_tx_chen: SoC requires that the tx channels are enabled.
 * @has_chsel_offset: SoC uses offset for selecting dai operational mode.
 * @reg_offset_txdata: offset of the tx fifo.
 * @sun4i_i2s_regmap: regmap config to use.
 * @fmt_offset: Value by which wss and sr needs to be adjusted.
 * @field_clkdiv_mclk_en: regmap field to enable mclk output.
 * @field_clkdiv_mclk: regmap field for mclkdiv.
 * @field_clkdiv_bclk: regmap field for bclkdiv.
 * @field_fmt_wss: regmap field to set word select size.
 * @field_fmt_sr: regmap field to set sample resolution.
 * @field_fmt_bclk: regmap field to set clk polarity.
 * @field_fmt_lrclk: regmap field to set frame polarity.
 * @field_fmt_mode: regmap field to set the operational mode.
 * @field_fmt_sext: regmap field to set the sign extension.
 * @field_txchanmap: location of the tx channel mapping register.
 * @field_rxchanmap: location of the rx channel mapping register.
 * @field_txchansel: location of the tx channel select bit fields.
 * @field_rxchansel: location of the rx channel select bit fields.
 */
struct sun4i_i2s_quirks {
	bool				has_reset;
	bool				has_slave_select_bit;
	bool				has_fmt_set_lrck_period;
	bool				has_chcfg;
	bool				has_chsel_tx_chen;
	bool				has_chsel_offset;
	unsigned int			reg_offset_txdata;	/* TX FIFO */
	const struct regmap_config	*sun4i_i2s_regmap;
	unsigned int			fmt_offset;

	/* Register fields for i2s */
	struct reg_field		field_clkdiv_mclk_en;
	struct reg_field		field_fmt_wss;
	struct reg_field		field_fmt_sr;
	struct reg_field		field_fmt_bclk;
	struct reg_field		field_fmt_lrclk;
	struct reg_field		field_fmt_mode;
	struct reg_field		field_fmt_sext;
	struct reg_field		field_txchanmap;
	struct reg_field		field_rxchanmap;
	struct reg_field		field_txchansel;
	struct reg_field		field_rxchansel;
};

struct sun4i_i2s {
	struct clk	*bus_clk;
	struct clk	*mod_clk;
	struct regmap	*regmap;
	struct reset_control *rst;

	unsigned int	mclk_freq;

	struct snd_dmaengine_dai_dma_data	capture_dma_data;
	struct snd_dmaengine_dai_dma_data	playback_dma_data;

	/* Register fields for i2s */
	struct regmap_field	*field_clkdiv_mclk_en;
	struct regmap_field	*field_fmt_wss;
	struct regmap_field	*field_fmt_sr;
	struct regmap_field	*field_fmt_bclk;
	struct regmap_field	*field_fmt_lrclk;
	struct regmap_field	*field_fmt_mode;
	struct regmap_field	*field_fmt_sext;
	struct regmap_field	*field_txchanmap;
	struct regmap_field	*field_rxchanmap;
	struct regmap_field	*field_txchansel;
	struct regmap_field	*field_rxchansel;

	const struct sun4i_i2s_quirks	*variant;

	unsigned int	tdm_slots;
	bool bit_clk_master;
	bool loopback;

	unsigned int	slot_width;
	unsigned int	offset;
};

struct sun4i_i2s_clk_div {
	u8	div;
	u8	val;
};

static const struct sun4i_i2s_clk_div sun4i_i2s_bclk_div[] = {
	{ .div = 2, .val = 0 },
	{ .div = 4, .val = 1 },
	{ .div = 6, .val = 2 },
	{ .div = 8, .val = 3 },
	{ .div = 12, .val = 4 },
	{ .div = 16, .val = 5 },
};

static const struct sun4i_i2s_clk_div sun8i_i2s_clk_div[] = {
	{ .div = 0, .val = 0 },
	{ .div = 1, .val = 1 },
	{ .div = 2, .val = 2 },
	{ .div = 4, .val = 3 },
	{ .div = 6, .val = 4 },
	{ .div = 8, .val = 5 },
	{ .div = 12, .val = 6 },
	{ .div = 16, .val = 7 },
	{ .div = 24, .val = 8 },
	{ .div = 32, .val = 9 },
	{ .div = 48, .val = 10 },
	{ .div = 64, .val = 11 },
	{ .div = 96, .val = 12 },
	{ .div = 128, .val = 13 },
	{ .div = 176, .val = 14 },
	{ .div = 192, .val = 15 },
};

static const struct sun4i_i2s_clk_div sun4i_i2s_mclk_div[] = {
	{ .div = 1, .val = 0 },
	{ .div = 2, .val = 1 },
	{ .div = 4, .val = 2 },
	{ .div = 6, .val = 3 },
	{ .div = 8, .val = 4 },
	{ .div = 12, .val = 5 },
	{ .div = 16, .val = 6 },
	{ .div = 24, .val = 7 },
};

static int sun4i_i2s_get_bclk_div(struct sun4i_i2s *i2s,
				  unsigned int oversample_rate,
				  unsigned int word_size,
				  const struct sun4i_i2s_clk_div *bdiv,
				  unsigned int size)
{
	int div = oversample_rate / word_size / 2;
	int i;

	for (i = 0; i < size; i++) {
		if (bdiv->div == div)
			return bdiv->val;
		bdiv++;
	}

	return -EINVAL;
}

static int sun4i_i2s_get_mclk_div(struct sun4i_i2s *i2s,
				  unsigned int oversample_rate,
				  unsigned int module_rate,
				  unsigned int sampling_rate,
				  const struct sun4i_i2s_clk_div *mdiv,
				  unsigned int size)
{
	int div = module_rate / sampling_rate / oversample_rate;
	int i;

	for (i = 0; i < size; i++) {
		if (mdiv->div == div)
			return mdiv->val;
		mdiv++;
	}

	return -EINVAL;
}

static int sun4i_i2s_oversample_rates[] = { 128, 192, 256, 384, 512, 768 };
static bool sun4i_i2s_oversample_is_valid(unsigned int oversample)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sun4i_i2s_oversample_rates); i++) {
		if (sun4i_i2s_oversample_rates[i] == oversample)
			return true;
	}

	return false;
}

static int sun4i_i2s_set_clk_rate(struct snd_soc_dai *dai,
				  unsigned int rate,
				  unsigned int word_size)
{
	struct sun4i_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int oversample_rate, clk_rate;
	int bclk_div, mclk_div;
	int ret;

	if (i2s->bit_clk_master) {
		switch (rate) {
		case 176400:
		case 88200:
		case 44100:
		case 22050:
		case 11025:
			clk_rate = 22579200;
			break;

		case 192000:
		case 128000:
		case 96000:
		case 64000:
		case 48000:
		case 32000:
		case 24000:
		case 16000:
		case 12000:
		case 8000:
			clk_rate = 24576000;
			break;

		default:
			dev_err(dai->dev, "Unsupported sample rate: %u\n", rate);
			return -EINVAL;
		}

		ret = clk_set_rate(i2s->mod_clk, clk_rate);
		if (ret) {
			dev_err(dai->dev, "Unable to set clock\n");
			return ret;
		}

		oversample_rate = i2s->mclk_freq / rate;
		if (!sun4i_i2s_oversample_is_valid(oversample_rate)) {
			dev_err(dai->dev, "Unsupported oversample rate: %d\n",
				oversample_rate);
			return -EINVAL;
		}

		if (i2s->variant->has_fmt_set_lrck_period)
			bclk_div = sun4i_i2s_get_bclk_div(i2s, clk_rate / rate,
							  word_size,
							  sun8i_i2s_clk_div,
							  ARRAY_SIZE(sun8i_i2s_clk_div));
		else
			bclk_div = sun4i_i2s_get_bclk_div(i2s, oversample_rate,
							  word_size,
							  sun4i_i2s_bclk_div,
							  ARRAY_SIZE(sun4i_i2s_bclk_div));
		if (bclk_div < 0) {
			dev_err(dai->dev, "Unsupported BCLK divider: %d\n",
				bclk_div);
			return -EINVAL;
		}


		if (i2s->variant->has_fmt_set_lrck_period)
			mclk_div = sun4i_i2s_get_mclk_div(i2s, oversample_rate,
							  clk_rate, rate,
							  sun8i_i2s_clk_div,
							  ARRAY_SIZE(sun8i_i2s_clk_div));
		else
			mclk_div = sun4i_i2s_get_mclk_div(i2s, oversample_rate,
							  clk_rate, rate,
							  sun4i_i2s_mclk_div,
							  ARRAY_SIZE(sun4i_i2s_mclk_div));
		if (mclk_div < 0) {
			dev_err(dai->dev, "Unsupported MCLK divider: %d\n",
				mclk_div);
			return -EINVAL;
		}

		regmap_write(i2s->regmap, SUN4I_I2S_CLK_DIV_REG,
			     SUN4I_I2S_CLK_DIV_BCLK(bclk_div) |
			     SUN4I_I2S_CLK_DIV_MCLK(mclk_div));

		regmap_field_write(i2s->field_clkdiv_mclk_en, 1);
	}

	/* Set sync period */
	if (i2s->variant->has_fmt_set_lrck_period) {
		int lrck;
		if (i2s->bit_clk_master)
			lrck = word_size;
		else
			lrck = i2s->mclk_freq / rate / 2;

		regmap_update_bits(i2s->regmap, SUN4I_I2S_FMT0_REG,
				   SUN8I_I2S_FMT0_LRCK_PERIOD_MASK,
				   SUN8I_I2S_FMT0_LRCK_PERIOD(lrck));
	}

	/* Set sign extension to pad out LSB with 0 */
	regmap_field_write(i2s->field_fmt_sext, 0);

	return 0;
}

static int sun4i_i2s_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct sun4i_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int sr, wss, channels;
	u32 width;
	int lines;

	channels = params_channels(params);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if ((channels > dai->driver->playback.channels_max) ||
			(channels < dai->driver->playback.channels_min)) {
			dev_err(dai->dev, "Unsupported number of channels: %d\n",
				channels);
			return -EINVAL;
		}

		lines = (channels + 1) / 2;

		/* Enable the required output lines */
		regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
				   SUN4I_I2S_CTRL_SDO_EN_MASK,
				   SUN4I_I2S_CTRL_SDO_EN(lines));

		if (i2s->variant->has_chcfg)
			regmap_update_bits(i2s->regmap, SUN8I_I2S_CHAN_CFG_REG,
					   SUN8I_I2S_CHAN_CFG_TX_SLOT_NUM_MASK,
					   SUN8I_I2S_CHAN_CFG_TX_SLOT_NUM(channels));
		else
			/* Configure the channels */
			regmap_field_write(i2s->field_txchansel, SUN4I_I2S_CHAN_SEL(channels));

		regmap_field_write(i2s->field_txchanmap, 0x10);

		if (i2s->variant->has_chsel_tx_chen) {
			u32 chan_sel = SUN8I_I2S_TX_CHAN_OFFSET(i2s->offset) | 0x1;
			regmap_write(i2s->regmap, SUN8I_I2S_TX_CHAN_SEL_REG,
				     chan_sel | 0x30);

			if (channels > 2) {
				regmap_write(i2s->regmap,
					     SUN8I_I2S_TX_CHAN_MAP_REG+4, 0x32);
				regmap_write(i2s->regmap, SUN8I_I2S_TX_CHAN_SEL_REG+4,
					     chan_sel | 0x30);
			}
			if (channels > 4) {
				regmap_write(i2s->regmap,
					     SUN8I_I2S_TX_CHAN_MAP_REG+8, 0x54);
				regmap_write(i2s->regmap, SUN8I_I2S_TX_CHAN_SEL_REG+8,
					     chan_sel | 0x30);
			}
			if (channels > 6) {
				regmap_write(i2s->regmap,
					     SUN8I_I2S_TX_CHAN_MAP_REG+12, 0x76);
				regmap_write(i2s->regmap, SUN8I_I2S_TX_CHAN_SEL_REG+12,
					     chan_sel | 0x30);
			}
		}
	} else {
		/* Map the channels for capture */
		regmap_field_write(i2s->field_rxchanmap, 0x00003210);
		regmap_field_write(i2s->field_rxchansel,
				   SUN4I_I2S_CHAN_SEL(params_channels(params)));

		if (i2s->variant->has_chcfg)
			regmap_update_bits(i2s->regmap, SUN8I_I2S_CHAN_CFG_REG,
					   SUN8I_I2S_CHAN_CFG_RX_SLOT_NUM_MASK,
					   SUN8I_I2S_CHAN_CFG_RX_SLOT_NUM(channels));

		if (i2s->variant->has_chsel_tx_chen)
			regmap_update_bits(i2s->regmap, SUN8I_I2S_RX_CHAN_SEL_REG,
					   SUN8I_I2S_TX_CHAN_OFFSET_MASK,
					   SUN8I_I2S_TX_CHAN_OFFSET(i2s->offset));
	}

	switch (params_physical_width(params)) {
	case 16:
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 20:
	case 24:
	case 32:
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		dev_err(dai->dev, "Unsupported physical sample width: %d\n",
			params_physical_width(params));
		return -EINVAL;
	}
	i2s->playback_dma_data.addr_width = width;

	switch (params_width(params)) {
	case 16:
		sr = 0;
		wss = 0;
		break;
	case 20:
		sr = 1;
		wss = 1;
		break;
	case 24:
		sr = 2;
		wss = 2;
		break;
	case 32:
		sr = 4;
		wss = 4;
		break;
	default:
		dev_err(dai->dev, "Unsupported sample width: %d\n",
			params_width(params));
		return -EINVAL;
	}

	regmap_field_write(i2s->field_fmt_wss,
			   wss + i2s->variant->fmt_offset);
	regmap_field_write(i2s->field_fmt_sr,
			   sr + i2s->variant->fmt_offset);

	return sun4i_i2s_set_clk_rate(dai, params_rate(params),
				      i2s->tdm_slots ?
				      i2s->slot_width : params_width(params));
}

static int sun4i_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct sun4i_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val;
	u32 bclk_polarity = SUN4I_I2S_FMT0_POLARITY_NORMAL;
	u32 lrclk_polarity = SUN4I_I2S_FMT0_POLARITY_NORMAL;

	/* DAI Mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		val = SUN4I_I2S_FMT0_FMT_I2S;
		i2s->offset = 1;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = SUN4I_I2S_FMT0_FMT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val = SUN4I_I2S_FMT0_FMT_RIGHT_J;
		break;
	default:
		dev_err(dai->dev, "Unsupported format: %d\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	if (i2s->variant->has_chsel_offset) {
		/*
		 * offset being set indicates that we're connected to an i2s
		 * device, however offset is only used on the sun8i block and
		 * i2s shares the same setting with the LJ format. Increment
		 * val so that the bit to value to write is correct.
		 */
		if (i2s->offset > 0)
			val++;
	}

	regmap_field_write(i2s->field_fmt_mode, val);

	/* DAI clock polarity */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		/* Invert both clocks */
		bclk_polarity = SUN4I_I2S_FMT0_POLARITY_INVERTED;
		lrclk_polarity = SUN4I_I2S_FMT0_POLARITY_INVERTED;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/* Invert bit clock */
		bclk_polarity = SUN4I_I2S_FMT0_POLARITY_INVERTED;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/* Invert frame clock */
		lrclk_polarity = SUN4I_I2S_FMT0_POLARITY_INVERTED;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		dev_err(dai->dev, "Unsupported clock polarity: %d\n",
			fmt & SND_SOC_DAIFMT_INV_MASK);
		return -EINVAL;
	}

	regmap_field_write(i2s->field_fmt_bclk, bclk_polarity);
	regmap_field_write(i2s->field_fmt_lrclk, lrclk_polarity);

	if (i2s->variant->has_slave_select_bit) {
		/* DAI clock master masks */
		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS:
			/* BCLK and LRCLK master */
			val = SUN4I_I2S_CTRL_MODE_MASTER;
			i2s->bit_clk_master = true;
			break;
		case SND_SOC_DAIFMT_CBM_CFM:
			/* BCLK and LRCLK slave */
			val = SUN4I_I2S_CTRL_MODE_SLAVE;
			i2s->bit_clk_master = false;
			break;
		default:
			dev_err(dai->dev, "Unsupported slave setting: %d\n",
				fmt & SND_SOC_DAIFMT_MASTER_MASK);
			return -EINVAL;
		}
		regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
				   SUN4I_I2S_CTRL_MODE_MASK,
				   val);
	} else {
		/*
		 * The newer i2s block does not have a slave select bit,
		 * instead the clk pins are configured as inputs.
		 */
		/* DAI clock master masks */
		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS:
			/* BCLK and LRCLK master */
			val = SUN8I_I2S_CTRL_BCLK_OUT |
				SUN8I_I2S_CTRL_LRCK_OUT;
			i2s->bit_clk_master = true;
			break;
		case SND_SOC_DAIFMT_CBM_CFM:
			/* BCLK and LRCLK slave */
			val = 0;
			i2s->bit_clk_master = false;
			break;
		default:
			dev_err(dai->dev, "Unsupported slave setting: %d\n",
				fmt & SND_SOC_DAIFMT_MASTER_MASK);
			return -EINVAL;
		}
		regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
				   SUN8I_I2S_CTRL_BCLK_OUT |
				   SUN8I_I2S_CTRL_LRCK_OUT,
				   val);
	}

	/* Set significant bits in our FIFOs */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_FIFO_CTRL_REG,
			   SUN4I_I2S_FIFO_CTRL_TX_MODE_MASK |
			   SUN4I_I2S_FIFO_CTRL_RX_MODE_MASK,
			   SUN4I_I2S_FIFO_CTRL_TX_MODE(1) |
			   SUN4I_I2S_FIFO_CTRL_RX_MODE(1));

	return 0;
}

static void sun4i_i2s_start_capture(struct sun4i_i2s *i2s)
{
	/* Flush RX FIFO */
	regcache_cache_bypass(i2s->regmap, true);
	regmap_update_bits(i2s->regmap, SUN4I_I2S_FIFO_CTRL_REG,
			   SUN4I_I2S_FIFO_CTRL_FLUSH_RX,
			   SUN4I_I2S_FIFO_CTRL_FLUSH_RX);
	regcache_cache_bypass(i2s->regmap, false);

	/* Clear RX counter */
	regmap_write(i2s->regmap, SUN4I_I2S_RX_CNT_REG, 0);

	/* Enable RX Block */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_RX_EN,
			   SUN4I_I2S_CTRL_RX_EN);

	/* Enable RX DRQ */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_DMA_INT_CTRL_REG,
			   SUN4I_I2S_DMA_INT_CTRL_RX_DRQ_EN,
			   SUN4I_I2S_DMA_INT_CTRL_RX_DRQ_EN);

	/* Debugging without codec */
	if (i2s->loopback)
		regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
				   SUN4I_I2S_CTRL_LOOP, SUN4I_I2S_CTRL_LOOP);
}

static void sun4i_i2s_start_playback(struct sun4i_i2s *i2s)
{
	/* Flush TX FIFO */
	regcache_cache_bypass(i2s->regmap, true);
	regmap_update_bits(i2s->regmap, SUN4I_I2S_FIFO_CTRL_REG,
			   SUN4I_I2S_FIFO_CTRL_FLUSH_TX,
			   SUN4I_I2S_FIFO_CTRL_FLUSH_TX);
	regcache_cache_bypass(i2s->regmap, false);

	/* Clear TX counter */
	regmap_write(i2s->regmap, SUN4I_I2S_TX_CNT_REG, 0);

	/* Enable TX Block */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_TX_EN,
			   SUN4I_I2S_CTRL_TX_EN);

	/* Enable TX DRQ */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_DMA_INT_CTRL_REG,
			   SUN4I_I2S_DMA_INT_CTRL_TX_DRQ_EN,
			   SUN4I_I2S_DMA_INT_CTRL_TX_DRQ_EN);
}

static void sun4i_i2s_stop_capture(struct sun4i_i2s *i2s)
{
	/* Disable RX Block */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_RX_EN,
			   0);

	/* Disable RX DRQ */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_DMA_INT_CTRL_REG,
			   SUN4I_I2S_DMA_INT_CTRL_RX_DRQ_EN,
			   0);
}

static void sun4i_i2s_stop_playback(struct sun4i_i2s *i2s)
{
	/* Disable TX Block */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_TX_EN,
			   0);

	/* Disable TX DRQ */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_DMA_INT_CTRL_REG,
			   SUN4I_I2S_DMA_INT_CTRL_TX_DRQ_EN,
			   0);
}

static int sun4i_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct sun4i_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			sun4i_i2s_start_playback(i2s);
		else
			sun4i_i2s_start_capture(i2s);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			sun4i_i2s_stop_playback(i2s);
		else
			sun4i_i2s_stop_capture(i2s);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int sun4i_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				unsigned int freq, int dir)
{
	struct sun4i_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (clk_id != 0)
		return -EINVAL;

	i2s->mclk_freq = freq;

	return 0;
}

static int sun4i_i2s_set_dai_tdm_slot(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask,
	int slots, int width)
{
	struct sun4i_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	i2s->tdm_slots = slots;

	i2s->slot_width = width;

	return 0;
}

static const struct snd_soc_dai_ops sun4i_i2s_dai_ops = {
	.hw_params	= sun4i_i2s_hw_params,
	.set_fmt	= sun4i_i2s_set_fmt,
	.set_sysclk	= sun4i_i2s_set_sysclk,
	.trigger	= sun4i_i2s_trigger,
	.set_tdm_slot	= sun4i_i2s_set_dai_tdm_slot,
};

static int sun4i_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct sun4i_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
				  &i2s->playback_dma_data,
				  &i2s->capture_dma_data);

	snd_soc_dai_set_drvdata(dai, i2s);

	return 0;
}

#define SUN4I_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_LE)

#define SUN8I_FORMATS	(SUN4I_FORMATS | \
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver sun4i_i2s_dai = {
	.probe = sun4i_i2s_dai_probe,
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SUN4I_FORMATS,
	},
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SUN4I_FORMATS,
	},
	.ops = &sun4i_i2s_dai_ops,
	.symmetric_rates = 1,
};

static const struct snd_soc_component_driver sun4i_i2s_component = {
	.name	= "sun4i-dai",
};

static bool sun4i_i2s_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SUN4I_I2S_FIFO_TX_REG:
		return true;

	default:
		return true;
	}
}

static bool sun4i_i2s_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SUN4I_I2S_FIFO_RX_REG:
	case SUN4I_I2S_FIFO_STA_REG:
		return false;

	default:
		return true;
	}
}

static bool sun4i_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SUN4I_I2S_FIFO_RX_REG:
	case SUN4I_I2S_FIFO_TX_REG:
	case SUN4I_I2S_FIFO_STA_REG:
	case SUN4I_I2S_INT_STA_REG:
	case SUN4I_I2S_RX_CNT_REG:
	case SUN4I_I2S_TX_CNT_REG:
		return true;

	default:
		return false;
	}
}

static bool sun8i_i2s_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SUN8I_I2S_FIFO_TX_REG:
		return true;

	default:
		return true;
	}
}

static bool sun8i_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg == SUN8I_I2S_INT_STA_REG)
		return true;
	if (reg == SUN8I_I2S_FIFO_TX_REG)
		return true;

	return sun4i_i2s_volatile_reg(dev, reg);
}

static const struct reg_default sun4i_i2s_reg_defaults[] = {
	{ SUN4I_I2S_CTRL_REG, 0x00000000 },
	{ SUN4I_I2S_FMT0_REG, 0x0000000c },
	{ SUN4I_I2S_FMT1_REG, 0x00004020 },
	{ SUN4I_I2S_FIFO_CTRL_REG, 0x000400f0 },
	{ SUN4I_I2S_DMA_INT_CTRL_REG, 0x00000000 },
	{ SUN4I_I2S_CLK_DIV_REG, 0x00000000 },
	{ SUN4I_I2S_TX_CHAN_SEL_REG, 0x00000001 },
	{ SUN4I_I2S_TX_CHAN_MAP_REG, 0x76543210 },
	{ SUN4I_I2S_RX_CHAN_SEL_REG, 0x00000001 },
	{ SUN4I_I2S_RX_CHAN_MAP_REG, 0x00003210 },
};

static const struct reg_default sun8i_i2s_reg_defaults[] = {
	{ SUN4I_I2S_CTRL_REG, 0x00060000 },
	{ SUN4I_I2S_FMT0_REG, 0x00000033 },
	{ SUN4I_I2S_FMT1_REG, 0x00000030 },
	{ SUN4I_I2S_FIFO_CTRL_REG, 0x000400f0 },
	{ SUN4I_I2S_DMA_INT_CTRL_REG, 0x00000000 },
	{ SUN4I_I2S_CLK_DIV_REG, 0x00000000 },
	{ SUN8I_I2S_CHAN_CFG_REG, 0x00000000 },
	{ SUN8I_I2S_TX_CHAN_SEL_REG, 0x00000000 },
	{ SUN8I_I2S_TX_CHAN_MAP_REG, 0x00000000 },
	{ SUN8I_I2S_RX_CHAN_SEL_REG, 0x00000000 },
	{ SUN8I_I2S_RX_CHAN_MAP_REG, 0x00000000 },
};

static const struct regmap_config sun4i_i2s_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= SUN4I_I2S_RX_CHAN_MAP_REG,

	.cache_type	= REGCACHE_FLAT,
	.reg_defaults	= sun4i_i2s_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(sun4i_i2s_reg_defaults),
	.writeable_reg	= sun4i_i2s_wr_reg,
	.readable_reg	= sun4i_i2s_rd_reg,
	.volatile_reg	= sun4i_i2s_volatile_reg,
};

static const struct regmap_config sun8i_i2s_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= SUN8I_I2S_RX_CHAN_MAP_REG,
	.cache_type	= REGCACHE_FLAT,
	.reg_defaults	= sun8i_i2s_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(sun8i_i2s_reg_defaults),
	.writeable_reg	= sun4i_i2s_wr_reg,
	.readable_reg	= sun8i_i2s_rd_reg,
	.volatile_reg	= sun8i_i2s_volatile_reg,
};

static int sun4i_i2s_runtime_resume(struct device *dev)
{
	struct sun4i_i2s *i2s = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(i2s->bus_clk);
	if (ret) {
		dev_err(dev, "Failed to enable bus clock\n");
		return ret;
	}

	regcache_cache_only(i2s->regmap, false);
	regcache_mark_dirty(i2s->regmap);

	ret = regcache_sync(i2s->regmap);
	if (ret) {
		dev_err(dev, "Failed to sync regmap cache\n");
		goto err_disable_clk;
	}

	/* Enable the whole hardware block */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_GL_EN, SUN4I_I2S_CTRL_GL_EN);

	/* Enable the first output line */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_SDO_EN_MASK,
			   SUN4I_I2S_CTRL_SDO_EN(1));

	ret = clk_prepare_enable(i2s->mod_clk);
	if (ret) {
		dev_err(dev, "Failed to enable module clock\n");
		goto err_disable_clk;
	}

	return 0;

err_disable_clk:
	clk_disable_unprepare(i2s->bus_clk);
	return ret;
}

static int sun4i_i2s_runtime_suspend(struct device *dev)
{
	struct sun4i_i2s *i2s = dev_get_drvdata(dev);

	clk_disable_unprepare(i2s->mod_clk);

	/* Disable our output lines */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_SDO_EN_MASK, 0);

	/* Disable the whole hardware block */
	regmap_update_bits(i2s->regmap, SUN4I_I2S_CTRL_REG,
			   SUN4I_I2S_CTRL_GL_EN, 0);

	regcache_cache_only(i2s->regmap, true);

	clk_disable_unprepare(i2s->bus_clk);

	return 0;
}

static const struct sun4i_i2s_quirks sun4i_a10_i2s_quirks = {
	.has_reset		= false,
	.reg_offset_txdata	= SUN4I_I2S_FIFO_TX_REG,
	.sun4i_i2s_regmap	= &sun4i_i2s_regmap_config,
	.field_clkdiv_mclk_en	= REG_FIELD(SUN4I_I2S_CLK_DIV_REG, 7, 7),
	.field_fmt_wss		= REG_FIELD(SUN4I_I2S_FMT0_REG, 2, 3),
	.field_fmt_sr		= REG_FIELD(SUN4I_I2S_FMT0_REG, 4, 5),
	.field_fmt_bclk		= REG_FIELD(SUN4I_I2S_FMT0_REG, 6, 6),
	.field_fmt_lrclk	= REG_FIELD(SUN4I_I2S_FMT0_REG, 7, 7),
	.has_slave_select_bit	= true,
	.field_fmt_mode		= REG_FIELD(SUN4I_I2S_FMT0_REG, 0, 1),
	.field_fmt_sext		= REG_FIELD(SUN4I_I2S_FMT1_REG, 8, 8),
	.field_txchanmap	= REG_FIELD(SUN4I_I2S_TX_CHAN_MAP_REG, 0, 31),
	.field_rxchanmap	= REG_FIELD(SUN4I_I2S_RX_CHAN_MAP_REG, 0, 31),
	.field_txchansel	= REG_FIELD(SUN4I_I2S_TX_CHAN_SEL_REG, 0, 2),
	.field_rxchansel	= REG_FIELD(SUN4I_I2S_RX_CHAN_SEL_REG, 0, 2),
};

static const struct sun4i_i2s_quirks sun6i_a31_i2s_quirks = {
	.has_reset		= true,
	.reg_offset_txdata	= SUN4I_I2S_FIFO_TX_REG,
	.sun4i_i2s_regmap	= &sun4i_i2s_regmap_config,
	.field_clkdiv_mclk_en	= REG_FIELD(SUN4I_I2S_CLK_DIV_REG, 7, 7),
	.field_fmt_wss		= REG_FIELD(SUN4I_I2S_FMT0_REG, 2, 3),
	.field_fmt_sr		= REG_FIELD(SUN4I_I2S_FMT0_REG, 4, 5),
	.field_fmt_bclk		= REG_FIELD(SUN4I_I2S_FMT0_REG, 6, 6),
	.field_fmt_lrclk	= REG_FIELD(SUN4I_I2S_FMT0_REG, 7, 7),
	.has_slave_select_bit	= true,
	.field_fmt_mode		= REG_FIELD(SUN4I_I2S_FMT0_REG, 0, 1),
	.field_fmt_sext		= REG_FIELD(SUN4I_I2S_FMT1_REG, 8, 8),
	.field_txchanmap	= REG_FIELD(SUN4I_I2S_TX_CHAN_MAP_REG, 0, 31),
	.field_rxchanmap	= REG_FIELD(SUN4I_I2S_RX_CHAN_MAP_REG, 0, 31),
	.field_txchansel	= REG_FIELD(SUN4I_I2S_TX_CHAN_SEL_REG, 0, 2),
	.field_rxchansel	= REG_FIELD(SUN4I_I2S_RX_CHAN_SEL_REG, 0, 2),
};

static const struct sun4i_i2s_quirks sun8i_a83t_i2s_quirks = {
	.has_reset		= true,
	.reg_offset_txdata	= SUN8I_I2S_FIFO_TX_REG,
	.sun4i_i2s_regmap	= &sun4i_i2s_regmap_config,
	.field_clkdiv_mclk_en	= REG_FIELD(SUN4I_I2S_CLK_DIV_REG, 7, 7),
	.field_fmt_wss		= REG_FIELD(SUN4I_I2S_FMT0_REG, 2, 3),
	.field_fmt_sr		= REG_FIELD(SUN4I_I2S_FMT0_REG, 4, 5),
	.field_fmt_bclk		= REG_FIELD(SUN4I_I2S_FMT0_REG, 6, 6),
	.field_fmt_lrclk	= REG_FIELD(SUN4I_I2S_FMT0_REG, 7, 7),
	.has_slave_select_bit	= true,
	.field_fmt_mode		= REG_FIELD(SUN4I_I2S_FMT0_REG, 0, 1),
	.field_txchanmap	= REG_FIELD(SUN4I_I2S_TX_CHAN_MAP_REG, 0, 31),
	.field_rxchanmap	= REG_FIELD(SUN4I_I2S_RX_CHAN_MAP_REG, 0, 31),
	.field_txchansel	= REG_FIELD(SUN4I_I2S_TX_CHAN_SEL_REG, 0, 2),
	.field_rxchansel	= REG_FIELD(SUN4I_I2S_RX_CHAN_SEL_REG, 0, 2),
};

static const struct sun4i_i2s_quirks sun8i_h3_i2s_quirks = {
	.has_reset		= true,
	.reg_offset_txdata	= SUN8I_I2S_FIFO_TX_REG,
	.sun4i_i2s_regmap	= &sun8i_i2s_regmap_config,
	.fmt_offset		= 3,
	.has_fmt_set_lrck_period = true,
	.has_chcfg		= true,
	.has_chsel_tx_chen	= true,
	.has_chsel_offset	= true,
	.field_clkdiv_mclk_en	= REG_FIELD(SUN4I_I2S_CLK_DIV_REG, 8, 8),
	.field_fmt_wss		= REG_FIELD(SUN4I_I2S_FMT0_REG, 0, 2),
	.field_fmt_sr		= REG_FIELD(SUN4I_I2S_FMT0_REG, 4, 6),
	.field_fmt_bclk		= REG_FIELD(SUN4I_I2S_FMT0_REG, 7, 7),
	.field_fmt_lrclk	= REG_FIELD(SUN4I_I2S_FMT0_REG, 19, 19),
	.field_fmt_mode		= REG_FIELD(SUN4I_I2S_CTRL_REG, 4, 5),
	.field_fmt_sext		= REG_FIELD(SUN4I_I2S_FMT1_REG, 4, 5),
	.field_txchanmap	= REG_FIELD(SUN8I_I2S_TX_CHAN_MAP_REG, 0, 31),
	.field_rxchanmap	= REG_FIELD(SUN8I_I2S_RX_CHAN_MAP_REG, 0, 31),
	.field_txchansel	= REG_FIELD(SUN8I_I2S_TX_CHAN_SEL_REG, 0, 2),
	.field_rxchansel	= REG_FIELD(SUN8I_I2S_RX_CHAN_SEL_REG, 0, 2),
};

static const struct sun4i_i2s_quirks sun50i_a64_codec_i2s_quirks = {
	.has_reset		= true,
	.reg_offset_txdata	= SUN8I_I2S_FIFO_TX_REG,
	.sun4i_i2s_regmap	= &sun4i_i2s_regmap_config,
	.has_slave_select_bit	= true,
	.field_clkdiv_mclk_en	= REG_FIELD(SUN4I_I2S_CLK_DIV_REG, 7, 7),
	.field_fmt_wss		= REG_FIELD(SUN4I_I2S_FMT0_REG, 2, 3),
	.field_fmt_sr		= REG_FIELD(SUN4I_I2S_FMT0_REG, 4, 5),
	.field_fmt_bclk		= REG_FIELD(SUN4I_I2S_FMT0_REG, 6, 6),
	.field_fmt_lrclk	= REG_FIELD(SUN4I_I2S_FMT0_REG, 7, 7),
	.field_fmt_mode		= REG_FIELD(SUN4I_I2S_FMT0_REG, 0, 1),
	.field_fmt_sext		= REG_FIELD(SUN4I_I2S_FMT1_REG, 8, 8),
	.field_txchanmap	= REG_FIELD(SUN4I_I2S_TX_CHAN_MAP_REG, 0, 31),
	.field_rxchanmap	= REG_FIELD(SUN4I_I2S_RX_CHAN_MAP_REG, 0, 31),
	.field_txchansel	= REG_FIELD(SUN4I_I2S_TX_CHAN_SEL_REG, 0, 2),
	.field_rxchansel	= REG_FIELD(SUN4I_I2S_RX_CHAN_SEL_REG, 0, 2),
};

static int sun4i_i2s_init_regmap_fields(struct device *dev,
					struct sun4i_i2s *i2s)
{
	i2s->field_clkdiv_mclk_en =
		devm_regmap_field_alloc(dev, i2s->regmap,
					i2s->variant->field_clkdiv_mclk_en);
	if (IS_ERR(i2s->field_clkdiv_mclk_en))
		return PTR_ERR(i2s->field_clkdiv_mclk_en);

	i2s->field_fmt_wss =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_fmt_wss);
	if (IS_ERR(i2s->field_fmt_wss))
		return PTR_ERR(i2s->field_fmt_wss);

	i2s->field_fmt_sr =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_fmt_sr);
	if (IS_ERR(i2s->field_fmt_sr))
		return PTR_ERR(i2s->field_fmt_sr);

	i2s->field_fmt_bclk =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_fmt_bclk);
	if (IS_ERR(i2s->field_fmt_bclk))
		return PTR_ERR(i2s->field_fmt_bclk);

	i2s->field_fmt_lrclk =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_fmt_lrclk);
	if (IS_ERR(i2s->field_fmt_lrclk))
		return PTR_ERR(i2s->field_fmt_lrclk);

	i2s->field_fmt_mode =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_fmt_mode);
	if (IS_ERR(i2s->field_fmt_mode))
		return PTR_ERR(i2s->field_fmt_mode);

	i2s->field_fmt_sext =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_fmt_sext);
	if (IS_ERR(i2s->field_fmt_sext))
		return PTR_ERR(i2s->field_fmt_sext);

	i2s->field_txchanmap =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_txchanmap);
	if (IS_ERR(i2s->field_txchanmap))
		return PTR_ERR(i2s->field_txchanmap);

	i2s->field_rxchanmap =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_rxchanmap);
	if (IS_ERR(i2s->field_rxchanmap))
		return PTR_ERR(i2s->field_rxchanmap);

	i2s->field_txchansel =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_txchansel);
	if (IS_ERR(i2s->field_txchansel))
		return PTR_ERR(i2s->field_txchansel);

	i2s->field_rxchansel =
			devm_regmap_field_alloc(dev, i2s->regmap,
						i2s->variant->field_rxchansel);
	return PTR_ERR_OR_ZERO(i2s->field_rxchansel);
}

static int sun4i_i2s_probe(struct platform_device *pdev)
{
	struct sun4i_i2s *i2s;
	struct snd_soc_dai_driver *soc_dai;
	struct resource *res;
	void __iomem *regs;
	int irq, ret, val;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2s);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Can't retrieve our interrupt\n");
		return irq;
	}

	i2s->variant = of_device_get_match_data(&pdev->dev);
	if (!i2s->variant) {
		dev_err(&pdev->dev, "Failed to determine the quirks to use\n");
		return -ENODEV;
	}

	i2s->bus_clk = devm_clk_get(&pdev->dev, "apb");
	if (IS_ERR(i2s->bus_clk)) {
		dev_err(&pdev->dev, "Can't get our bus clock\n");
		return PTR_ERR(i2s->bus_clk);
	}

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    i2s->variant->sun4i_i2s_regmap);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev, "Regmap initialisation failed\n");
		return PTR_ERR(i2s->regmap);
	}

	i2s->mod_clk = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(i2s->mod_clk)) {
		dev_err(&pdev->dev, "Can't get our mod clock\n");
		return PTR_ERR(i2s->mod_clk);
	}

	if (i2s->variant->has_reset) {
		i2s->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
		if (IS_ERR(i2s->rst)) {
			dev_err(&pdev->dev, "Failed to get reset control\n");
			return PTR_ERR(i2s->rst);
		}
	}

	if (!IS_ERR(i2s->rst)) {
		ret = reset_control_deassert(i2s->rst);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to deassert the reset control\n");
			return -EINVAL;
		}
	}

	i2s->playback_dma_data.addr = res->start +
					i2s->variant->reg_offset_txdata;
	i2s->playback_dma_data.maxburst = 8;

	i2s->capture_dma_data.addr = res->start + SUN4I_I2S_FIFO_RX_REG;
	i2s->capture_dma_data.maxburst = 8;

	soc_dai = devm_kmemdup(&pdev->dev, &sun4i_i2s_dai,
			       sizeof(*soc_dai), GFP_KERNEL);
	if (!soc_dai) {
		ret = -ENOMEM;
		goto err_pm_disable;
	}

	if (i2s->variant->has_fmt_set_lrck_period) {
		soc_dai->playback.formats = SUN8I_FORMATS;
		soc_dai->capture.formats = SUN8I_FORMATS;
	}

	if (!of_property_read_u32(pdev->dev.of_node,
				  "allwinner,playback-channels", &val)) {
		dev_err(&pdev->dev, "Max playback channels changed from %d to %d\n", soc_dai->playback.channels_max, val);
		if (val >= 2 && val <= 8)
			soc_dai->playback.channels_max = val;
	}

	if (of_property_read_bool(pdev->dev.of_node, "loopback"))
		i2s->loopback = true;

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = sun4i_i2s_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &sun4i_i2s_component,
					      soc_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI\n");
		goto err_suspend;
	}

	ret = snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM\n");
		goto err_suspend;
	}

	ret = sun4i_i2s_init_regmap_fields(&pdev->dev, i2s);
	if (ret) {
		dev_err(&pdev->dev, "Could not initialise regmap fields\n");
		goto err_suspend;
	}

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		sun4i_i2s_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	if (!IS_ERR(i2s->rst))
		reset_control_assert(i2s->rst);

	return ret;
}

static int sun4i_i2s_remove(struct platform_device *pdev)
{
	struct sun4i_i2s *i2s = dev_get_drvdata(&pdev->dev);

	snd_dmaengine_pcm_unregister(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		sun4i_i2s_runtime_suspend(&pdev->dev);

	if (!IS_ERR(i2s->rst))
		reset_control_assert(i2s->rst);

	return 0;
}

static const struct of_device_id sun4i_i2s_match[] = {
	{
		.compatible = "allwinner,sun4i-a10-i2s",
		.data = &sun4i_a10_i2s_quirks,
	},
	{
		.compatible = "allwinner,sun6i-a31-i2s",
		.data = &sun6i_a31_i2s_quirks,
	},
	{
		.compatible = "allwinner,sun8i-a83t-i2s",
		.data = &sun8i_a83t_i2s_quirks,
	},
	{
		.compatible = "allwinner,sun8i-h3-i2s",
		.data = &sun8i_h3_i2s_quirks,
	},
	{
		.compatible = "allwinner,sun50i-a64-codec-i2s",
		.data = &sun50i_a64_codec_i2s_quirks,
	},
	{}
};
MODULE_DEVICE_TABLE(of, sun4i_i2s_match);

static const struct dev_pm_ops sun4i_i2s_pm_ops = {
	.runtime_resume		= sun4i_i2s_runtime_resume,
	.runtime_suspend	= sun4i_i2s_runtime_suspend,
};

static struct platform_driver sun4i_i2s_driver = {
	.probe	= sun4i_i2s_probe,
	.remove	= sun4i_i2s_remove,
	.driver	= {
		.name		= "sun4i-i2s",
		.of_match_table	= sun4i_i2s_match,
		.pm		= &sun4i_i2s_pm_ops,
	},
};
module_platform_driver(sun4i_i2s_driver);

MODULE_AUTHOR("Andrea Venturi <be17068@iperbole.bo.it>");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner A10 I2S driver");
MODULE_LICENSE("GPL");
