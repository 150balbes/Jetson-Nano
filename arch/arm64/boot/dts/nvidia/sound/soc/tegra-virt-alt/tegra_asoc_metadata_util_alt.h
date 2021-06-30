/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#ifndef TEGRA_NVAUDIO_METADATA_
#define TEGRA_NVAUDIO_METADATA_

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#define ADMA_PAGE1_BASE 0x2940000
#define ADMA_PAGE_SIZE  0x10000

/* Register offsets from Channel BASE */
#define ADMA_CH_CMD						0x00
#define ADMA_CH_SOFT_RESET					0x04
#define ADMA_CH_STATUS						0x0c
#define ADMA_CH_INT_STATUS					0x10
#define ADMA_CH_INT_SET						0x18
#define ADMA_CH_INT_CLEAR					0x1c
#define ADMA_CH_CTRL						0x24
#define ADMA_CH_CONFIG						0x28
#define ADMA_CH_AHUB_FIFO_CTRL					0x2c
#define ADMA_CH_TC_STATUS					0x30
#define ADMA_CH_LOWER_SOURCE_ADDR				0x34
#define ADMA_CH_LOWER_TARGET_ADDR				0x3c
#define ADMA_CH_TC						0x44
#define ADMA_CH_LOWER_DESC_ADDR					0x48
#define ADMA_CH_TRANSFER_STATUS					0x54

#define ADMA_CH_STATUS_TRANSFER_ENABLED				BIT(0)

/* Fields in ADMA_CH_CTRL  */
#define T210_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT		28
#define T210_ADMA_CH_CTRL_TX_REQUEST_SELECT_MASK	\
	(15 << T210_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT)
#define T210_ADMA_CH_CTRL_RX_REQUEST_SELECT_SHIFT		24
#define T210_ADMA_CH_CTRL_RX_REQUEST_SELECT_MASK	\
	(15 << T210_ADMA_CH_CTRL_RX_REQUEST_SELECT_SHIFT)

#define T186_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT		27
#define T186_ADMA_CH_CTRL_TX_REQUEST_SELECT_MASK	\
	(31 << T186_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT)
#define T186_ADMA_CH_CTRL_RX_REQUEST_SELECT_SHIFT		22
#define T186_ADMA_CH_CTRL_RX_REQUEST_SELECT_MASK	\
	(31 << T186_ADMA_CH_CTRL_RX_REQUEST_SELECT_SHIFT)


#define ADMA_CH_CTRL_TRIGGER_SELECT_SHIFT			16
#define ADMA_CH_CTRL_TRANSFER_DIRECTION_SHIFT			12
#define ADMA_CH_CTRL_TRANSFER_DIRECTION_MASK	\
	(15 << ADMA_CH_CTRL_TRANSFER_DIRECTION_SHIFT)
#define ADMA_CH_CTRL_TRANSFER_MODE_SHIFT			8
#define ADMA_CH_CTRL_TRANSFER_MODE_MASK		\
	(7 << ADMA_CH_CTRL_TRANSFER_MODE_SHIFT)
#define ADMA_CH_CTRL_TRIGGER_ENABLE_SHIFT			2
#define ADMA_CH_CTRL_FLOWCTRL_ENABLE_SHIFT			1
#define ADMA_CH_CTRL_TRANSFER_PAUSE_SHIFT			0
#define ADMA_CH_CTRL_TRANSFER_PAUSE_MASK	\
	(1 << ADMA_CH_CTRL_TRANSFER_PAUSE_SHIFT)

#define ADMA_CH_CTRL_TRANSFER_PAUSE				BIT(0)
#define ADMA_CH_CTRL_FLOWCTRL_ENABLE				BIT(1)

/* Fields in ADMA_CH_CONFIG  */
#define ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_SHIFT		28
#define ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_MASK	\
	(7 << ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_SHIFT)
#define ADMA_CH_CONFIG_TARGET_MEMORY_BUFFER_SHIFT		24
#define ADMA_CH_CONFIG_TARGET_MEMORY_BUFFER_MASK	\
	(7 << ADMA_CH_CONFIG_TARGET_MEMORY_BUFFER_SHIFT)
#define ADMA_CH_CONFIG_BURST_SIZE_SHIFT				20
#define T210_ADMA_CH_CONFIG_BURST_SIZE_MASK		\
	(7 << ADMA_CH_CONFIG_BURST_SIZE_SHIFT)
#define T186_ADMA_CH_CONFIG_BURST_SIZE_MASK		\
	(15 << ADMA_CH_CONFIG_BURST_SIZE_SHIFT)
#define ADMA_CH_CONFIG_SOURCE_ADDR_WRAP_SHIFT			16
#define ADMA_CH_CONFIG_TARGET_ADDR_WRAP_SHIFT			12
#define ADMA_CH_CONFIG_WEIGHT_FOR_WRR_SHIFT			0

#define ADMA_CH_CONFIG_MAX_MEM_BUFFERS				8

/* Fields in ADMA_CH_AHUB_FIFO_CTRL  */
#define ADMA_CH_AHUB_FIFO_CTRL_FETCHING_POLICY_SHIFT		31
#define ADMA_CH_AHUB_FIFO_CTRL_OVERFLOW_THRESHOLD_SHIFT		24
#define ADMA_CH_AHUB_FIFO_CTRL_STARVATION_THRESHOLD_SHIFT	16
#define ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_SHIFT		8
#define T210_ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_MASK	\
	(15 << ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_SHIFT)
#define T186_ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_MASK	\
	(63 << ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_SHIFT)
#define ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_SHIFT		0
#define T210_ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_MASK	\
	(15 << ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_SHIFT)
#define T186_ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_MASK	\
	(63 << ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_SHIFT)

#define ADMA_CH_INT_TD_STATUS					BIT(0)

/* Fields in ADMA_CH_TRANSFER_STATUS */
#define ADMA_CH_TRANSFER_DONE_COUNT_SHIFT			0
#define ADMA_CH_TRANSFER_DONE_COUNT_MASK	\
	(0xFFFF << ADMA_CH_TRANSFER_DONE_COUNT_SHIFT)
/*
 * If any burst is in flight and ADMA paused then this is the time to complete
 * on-flight burst and update ADMA status register.
 */
#define TEGRA_ADMA_BURST_COMPLETE_TIME				20
#define T210_CH_REG_SIZE					0x80
#define T186_CH_REG_SIZE					0x100

enum tegra_adma_fetching_policy {
	BURST_BASED = 0,
	THRESHOLD_BASED = 1,
};

enum tegra186_adma_burst_size {
	T186_WORD_1 = 0,
	T186_WORDS_2 = 1,
	T186_WORDS_3 = 2,
	T186_WORDS_4 = 3,
	T186_WORDS_5 = 4,
	T186_WORDS_6 = 5,
	T186_WORDS_7 = 6,
	T186_WORDS_8 = 7,
	T186_WORDS_9 = 8,
	T186_WORDS_10 = 9,
	T186_WORDS_11 = 10,
	T186_WORDS_12 = 11,
	T186_WORDS_13 = 12,
	T186_WORDS_14 = 13,
	T186_WORDS_15 = 14,
	T186_WORDS_16 = 15,
};

enum tegra_adma_mode {
	ADMA_CONTINUOUS_MODE = 2,
};

enum tegra_adma_transfer_direction {
	MEMORY_TO_MEMORY = 1,
	AHUB_TO_MEMORY = 2,
	MEMORY_TO_AHUB = 4,
	AHUB_TO_AHUB = 8,
};

#define TEGRA_AUDIO_METADATA_HDR_LENGTH     8

#define TEGRA_AUDIO_METADATA_SBFRM_LENGTH   4

#define TEGRA_AUDIO_METADATA_BLK_LENGTH \
	(TEGRA_AUDIO_METADATA_HDR_LENGTH * \
	(TEGRA_AUDIO_METADATA_SBFRM_LENGTH + 1))

#define NUM_PERIODS              4

#define PERIOD_SIZE              10240
#define MAX_NUM_METADATA		8

enum {
	SUBFRAME_MODE = 0,
	HEADER_MODE,
};

struct tegra_audio_metadata_cntx {
	uint32_t metadata_mode;
	uint32_t init_metadata_flood;
	uint32_t enable_metadata_flood;

	uint16_t header[MAX_NUM_METADATA][TEGRA_AUDIO_METADATA_HDR_LENGTH];
	uint16_t block[TEGRA_AUDIO_METADATA_BLK_LENGTH];

	/* 0 indexed */
	int32_t  admaif_id;
	int32_t  dma_id;
	int32_t  dma_ch_page;

	size_t  num_periods;
	size_t  period_size;
	size_t  buffer_size;
	int8_t  *buffer_addr;
	dma_addr_t buffer_phys_addr;

	void __iomem *dma_addr;
};

int32_t tegra_metadata_flood_enable(struct tegra_audio_metadata_cntx *s,
					uint32_t enable, struct device *dev);

int32_t tegra_metadata_flood_init(struct tegra_audio_metadata_cntx *s,
							struct device *dev);

int32_t tegra_metadata_flood_deinit(struct tegra_audio_metadata_cntx *s,
							struct device *dev);

int32_t
tegra_metadata_flood_get(uint8_t *m);

int32_t
tegra_metadata_flood_update(uint8_t *m);

int32_t tegra_asoc_metadata_update(uint32_t index);

#endif
