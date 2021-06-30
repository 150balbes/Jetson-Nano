#include <linux/module.h>
#include <linux/delay.h>

#include "tegra_virt_alt_ivc.h"
#include "tegra_asoc_util_virt_alt.h"
#include "tegra_asoc_metadata_util_alt.h"
#include "tegra210_virt_alt_admaif.h"

static struct tegra_audio_metadata_cntx *g_context;

static void
tegra_metadata_header_reset(struct tegra_audio_metadata_cntx *s)
{
	int i;

	for (i = 0; i < MAX_NUM_METADATA; i++)
		memset(&g_context->header[i][0], 0,
			sizeof(int16_t) * TEGRA_AUDIO_METADATA_HDR_LENGTH);

}

static uint32_t tegra_metadata_buffer_create(uint16_t *buffer, int16_t *header,
						uint32_t metadata_mode)
{
	uint32_t i, block_size = 0;
	uint16_t fill_data[TEGRA_AUDIO_METADATA_SBFRM_LENGTH];

	for (i = 0; i < TEGRA_AUDIO_METADATA_SBFRM_LENGTH; i++)
		fill_data[i] = (metadata_mode == SUBFRAME_MODE)
					? (i + 1) : 0;

	for (i = 0; i < TEGRA_AUDIO_METADATA_HDR_LENGTH; i++) {

		buffer[block_size] = header[i];
		block_size++;

		memcpy(buffer + block_size, fill_data,
			TEGRA_AUDIO_METADATA_SBFRM_LENGTH * sizeof(uint16_t));
		block_size += TEGRA_AUDIO_METADATA_SBFRM_LENGTH;
	}

	return block_size;
}

int32_t
tegra_metadata_prepare(struct tegra_audio_metadata_cntx *s, int index)
{
	int8_t *pdst = NULL;
	uint32_t block_size = 0, num_blocks = 0, block_size_in_bytes;

	if (!s) {
		pr_err("Invalid params for metadata update\n");
		return -EAGAIN;
	}

	block_size = tegra_metadata_buffer_create(s->block,
			&s->header[index][0], s->metadata_mode);

	block_size_in_bytes = block_size * sizeof(uint16_t);
	pdst = s->buffer_addr;
	num_blocks = s->buffer_size / block_size_in_bytes;
	while (num_blocks-- > 0) {
		memcpy(pdst, s->block, block_size_in_bytes);
		pdst += block_size_in_bytes;
	}

	return 0;
}

static void tegra_metadata_dma_channel_init(struct tegra_audio_metadata_cntx *s)
{
	uint32_t config, ctrl, fifo_ctrl, wcount;

	uint8_t *ch_base = (uint8_t *)s->dma_addr;

	wcount = s->period_size;
	ctrl = readl(ch_base + ADMA_CH_CTRL);
	ctrl &= ~ADMA_CH_CTRL_TRANSFER_MODE_MASK;
	ctrl |= ADMA_CONTINUOUS_MODE << ADMA_CH_CTRL_TRANSFER_MODE_SHIFT;
	ctrl |= ADMA_CH_CTRL_FLOWCTRL_ENABLE;

	config = readl(ch_base + ADMA_CH_CONFIG);
	config &= ~T186_ADMA_CH_CONFIG_BURST_SIZE_MASK;
	config |= T186_WORDS_4 << ADMA_CH_CONFIG_BURST_SIZE_SHIFT;

	fifo_ctrl = readl(ch_base + ADMA_CH_AHUB_FIFO_CTRL);
	fifo_ctrl &= ~(1 << ADMA_CH_AHUB_FIFO_CTRL_FETCHING_POLICY_SHIFT);
	fifo_ctrl |= BURST_BASED
		<< ADMA_CH_AHUB_FIFO_CTRL_FETCHING_POLICY_SHIFT;

	ctrl &= ~ADMA_CH_CTRL_TRANSFER_DIRECTION_MASK;
	ctrl |= MEMORY_TO_AHUB << ADMA_CH_CTRL_TRANSFER_DIRECTION_SHIFT;

	ctrl &= ~T186_ADMA_CH_CTRL_TX_REQUEST_SELECT_MASK;
	ctrl |= (s->admaif_id + 1) << T186_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT;

	config &= ~(ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_MASK);
	config |= (s->num_periods - 1)
		<< ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_SHIFT;

	writel(s->buffer_phys_addr, ch_base + ADMA_CH_LOWER_SOURCE_ADDR);

	writel(wcount, ch_base + ADMA_CH_TC);
	writel(fifo_ctrl, ch_base + ADMA_CH_AHUB_FIFO_CTRL);
	writel(config, ch_base + ADMA_CH_CONFIG);
	writel(ctrl, ch_base + ADMA_CH_CTRL);

}

static void tegra_metadata_dma_channel_enable
				(struct tegra_audio_metadata_cntx *s)
{
	uint8_t *ch_base = (uint8_t *)s->dma_addr;

	writel(1, ch_base + ADMA_CH_CMD);
}

static int32_t is_meta_dma_enabled(struct tegra_audio_metadata_cntx *s)
{
	uint8_t *ch_base = (uint8_t *)s->dma_addr;
	uint32_t csts;

	csts = readl(ch_base + ADMA_CH_STATUS);
	csts &= ADMA_CH_STATUS_TRANSFER_ENABLED;
	return csts;
}

static void
tegra_metadata_dma_channel_disable(struct tegra_audio_metadata_cntx *s)
{
	uint8_t *ch_base = (uint8_t *)s->dma_addr;
	uint32_t status, dcnt = 10;

	/* Disable interrupts  */
	writel(1, ch_base + ADMA_CH_INT_CLEAR);

	/* Disable ADMA */
	writel(0, ch_base + ADMA_CH_CMD);

	/* Clear interrupt status if it is there */
	status = readl(ch_base + ADMA_CH_INT_STATUS);
	if (status & ADMA_CH_INT_TD_STATUS)
		writel(1, ch_base + ADMA_CH_INT_CLEAR);

	while (is_meta_dma_enabled(s) && dcnt--)
		udelay(TEGRA_ADMA_BURST_COMPLETE_TIME);

	if (is_meta_dma_enabled(s))
		pr_err("Stop failed for channel %d", s->dma_id);

}

int32_t
tegra_metadata_init(struct tegra_audio_metadata_cntx *s, struct device *dev)
{
	uint32_t dma_chan_addr;

	s->num_periods = NUM_PERIODS;
	s->period_size = PERIOD_SIZE;

	s->buffer_size = s->period_size * s->num_periods;

	/* allocate shm buffer */
	s->buffer_addr = dma_alloc_coherent(dev, s->buffer_size,
				&s->buffer_phys_addr, GFP_KERNEL);
	if (!s->buffer_addr) {
		pr_err("Failed to allocate metadta buffer\n");
		return -1;
	}

	/* populate defaults  */
	tegra_metadata_header_reset(s);

	/* get adma resource corresponding to dma id */
	dma_chan_addr = ADMA_PAGE1_BASE + (s->dma_ch_page * ADMA_PAGE_SIZE)
					+ (s->dma_id *  T186_CH_REG_SIZE);
	s->dma_addr = ioremap_nocache(dma_chan_addr, T186_CH_REG_SIZE);
	if (!s->dma_addr) {
		pr_err("Unable to map dma channel for metadata\n");
		return -1;
	}

	return 0;
}

static int32_t
tegra_metadata_setup_admaif_cif(struct tegra_audio_metadata_cntx *s,
						struct device *dev)
{
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(dev);
	uint32_t value = 0;
	int err;
	struct nvaudio_ivc_msg msg;
	struct tegra210_virt_audio_cif cif_conf;

	cif_conf.threshold = 0;
	cif_conf.audio_channels = TEGRA_AUDIO_METADATA_SBFRM_LENGTH + 1;
	cif_conf.client_channels = TEGRA_AUDIO_METADATA_SBFRM_LENGTH + 1;
	cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_16;
	cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_32;

	value = ((cif_conf.threshold <<
			TEGRA210_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
		((cif_conf.audio_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
		((cif_conf.client_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
		(cif_conf.audio_bits <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_BITS_SHIFT) |
		(cif_conf.client_bits <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_BITS_SHIFT));

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.params.dmaif_info.id = s->admaif_id;
	msg.params.dmaif_info.value = value;
	msg.cmd = NVAUDIO_DMAIF_SET_TXCIF;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int32_t
tegra_metadata_toggle_admaif(struct tegra_audio_metadata_cntx *s,
						struct device *dev)
{
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = s->enable_metadata_flood ?
			NVAUDIO_START_PLAYBACK : NVAUDIO_STOP_PLAYBACK;
	msg.params.dmaif_info.id = s->admaif_id;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

int32_t
tegra_asoc_metadata_update(uint32_t index) {

	struct tegra_audio_metadata_cntx *s = g_context;
	int32_t ret = 0;

	if (!g_context || !g_context->init_metadata_flood)
		return 0;

	if (index >= MAX_NUM_METADATA) {
		pr_err("metadata_update invalid index %d\n", index);
		ret = -1;
	} else
		ret = tegra_metadata_prepare(s, index);

	return ret;
}
EXPORT_SYMBOL_GPL(tegra_asoc_metadata_update);

int32_t tegra_metadata_flood_init(struct tegra_audio_metadata_cntx *s,
						struct device *dev)
{

	if (!s) {
		pr_err("metadata context is NULL\n");
		return -1;
	}

	g_context = s;

	if (tegra_metadata_init(s, dev))
		return -1;

	tegra_metadata_dma_channel_init(s);

	pr_info("Mode: %s, ADMAIF: %d, DMA channel: %d\n",
			s->metadata_mode ? "Header" : "Subframe", s->admaif_id,
					s->dma_id);

	tegra_metadata_prepare(s, 0);

	if (tegra_metadata_setup_admaif_cif(s, dev))
		return -1;

	s->init_metadata_flood = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_metadata_flood_init);


int32_t
tegra_metadata_flood_deinit(struct tegra_audio_metadata_cntx *s,
						struct device *dev)
{
	if (s->buffer_addr) {
		dma_free_coherent(dev, s->buffer_size, s->buffer_addr,
				s->buffer_phys_addr);
		s->buffer_addr = NULL;
	}

	s->init_metadata_flood = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_metadata_flood_deinit);

int32_t
tegra_metadata_flood_enable(struct tegra_audio_metadata_cntx *s,
				uint32_t enable, struct device *dev)
{
	if (enable)
		tegra_metadata_dma_channel_enable(s);
	else
		tegra_metadata_dma_channel_disable(s);

	s->enable_metadata_flood = enable;

	if (tegra_metadata_toggle_admaif(s, dev))
		return -1;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_metadata_flood_enable);

int32_t
tegra_metadata_flood_get(uint8_t *m)
{
	int16_t *indx = (int16_t *)m;

	if (!g_context) {
		pr_err("metadata context is NULL\n");
		return 0;
	}

	if (*indx <= 0 || *indx > MAX_NUM_METADATA) {
		pr_err("invalid metadata get index\n");
		return 0;
	}
	/* first two bytes contain ENT source details. Read header
	*  info for ENT source, copy to send to user passed buffer
	*/
	memcpy(m+2, &g_context->header[(*indx) - 1][0], sizeof(int16_t)
					* TEGRA_AUDIO_METADATA_HDR_LENGTH);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_metadata_flood_get);

int32_t
tegra_metadata_flood_update(uint8_t *m)
{
	int16_t *indx = (int16_t *)m;

	if (!g_context) {
		pr_err("metadata context is NULL\n");
		return 0;
	}

	if (*indx <= 0 || *indx > MAX_NUM_METADATA) {
		pr_err("invalid metadata set index\n");
		return 0;
	}

	/* extract first two bytes to get ENT source details, and update
	* corresponding header with rest of data passed by user
	*/
	memcpy(&g_context->header[(*indx) - 1][0], m+2, sizeof(int16_t)
					* TEGRA_AUDIO_METADATA_HDR_LENGTH);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_metadata_flood_update);

MODULE_LICENSE("GPL");
