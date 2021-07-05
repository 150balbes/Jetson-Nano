/*
 * tegra210_adsp_alt.c - Tegra ADSP audio driver
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <../arch/arm/mach-tegra/iomap.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/tegra_nvadsp.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/of_device.h>

#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <soc/tegra/chip-id.h>
#include <sound/compress_driver.h>
#include <sound/dmaengine_pcm.h>
#include <sound/tegra_nvfx.h>
#include <sound/tegra_nvfx_apm.h>
#include <sound/tegra_nvfx_plugin.h>
#include "tegra_isomgr_bw_alt.h"

#include "tegra_asoc_utils_alt.h"
#include "tegra210_adsp_alt.h"

#define DRV_NAME_ADSP "tegra210-adsp"

/* Flag to enable/disable loading of ADSP firmware */
#define ENABLE_ADSP 1

#define NETLINK_ADSP_EVENT 31
#define NETLINK_ADSP_EVENT_GROUP 1

struct adsp_event_nlmsg {
	uint32_t err;
	uint32_t data[NVFX_MAX_CALL_PARAMS_WSIZE];
};

#define ADSP_RESPONSE_TIMEOUT	1000 /* in ms */
/* ADSP controls plugin index */
#define PLUGIN_SET_PARAMS_IDX	1
#define PLUGIN_SEND_BYTES_IDX	21
static const unsigned int tegra210_adsp_rates[] = {
	8000, 11025, 12000, 16000, 22050,
	24000, 32000, 44100, 48000
};

static const struct snd_pcm_hw_constraint_list
	tegra210_adsp_rate_constraints = {
	.count = ARRAY_SIZE(tegra210_adsp_rates),
	.list = tegra210_adsp_rates,
};

static struct tegra210_adsp_app_desc {
	const char *name;
	const char *fw_name;
	const char *wt_name;
	uint32_t param_type;
	uint32_t reg_start;
	uint32_t reg_end;
	nvadsp_app_handle_t handle;
} adsp_app_minimal[] = {
	{"apm", "nvapm.elf", NULL, SNDRV_CTL_ELEM_TYPE_NONE,
		APM_IN_START, APM_IN_END},
	{"adma", "nvadma.elf", NULL, SNDRV_CTL_ELEM_TYPE_NONE,
		ADMA_START, ADMA_END},
	{"adma_tx", "nvadma_tx.elf", NULL, SNDRV_CTL_ELEM_TYPE_NONE,
		ADMA_TX_START, ADMA_TX_END},
};

static struct tegra210_adsp_app_desc *adsp_app_desc;
static unsigned int adsp_app_count; /* total number of apps initialized */
static int apm_stack_size[APM_IN_END - APM_IN_START + 1];

struct tegra210_adsp_app_read_data {
	int32_t data[NVFX_MAX_RAW_DATA_WSIZE];
};

/* ADSP APP specific structure */
struct tegra210_adsp_app {
	struct tegra210_adsp *adsp;
	const struct tegra210_adsp_app_desc *desc;
	nvadsp_app_info_t *info;
	plugin_shared_mem_t *plugin;
	apm_shared_state_t *apm; /* For a plugin it stores parent apm data */
	struct nvadsp_mbox apm_mbox;
	struct completion *msg_complete; /* For ADSP ack wait */
	struct completion *raw_msg_read_complete;
	struct completion *raw_msg_write_complete;
	uint32_t reg;
	uint32_t adma_chan; /* Valid for only ADMA app */
	uint32_t fe:1; /* Whether the app is used as a FE APM */
	uint32_t fe_playback_triggered:1; /* if app is playback FE, indicates whether in triggered state or inactive */
	uint32_t connect:1; /* if app is connected to a source */
	uint32_t priority; /* Valid for only APM app */
	uint32_t min_adsp_clock; /* Min ADSP clock required in MHz */
	uint32_t input_mode; /* APM input mode */
	struct tegra210_adsp_app_read_data read_data;
	spinlock_t lock;
	void *private_data;
	int (*msg_handler)(struct tegra210_adsp_app *, apm_msg_t *);
	struct work_struct *override_freq_work;
	spinlock_t apm_msg_queue_lock;
};

struct tegra210_adsp_pcm_rtd {
	struct device *dev;
	struct snd_pcm_substream *substream;
	struct tegra210_adsp_app *fe_apm;
	snd_pcm_uframes_t prev_appl_ptr;
};

struct tegra210_adsp_compr_rtd {
	struct device *dev;
	struct snd_dma_buffer buf;
	struct snd_compr_stream *cstream;
	struct snd_codec codec;
	struct tegra210_adsp_app *fe_apm;
	int is_draining;
};

struct adsp_soc_data {
	bool is_soc_t210;
	uint32_t max_adma_ch;
};

struct tegra210_adsp {
	struct device *dev;
	struct adsp_soc_data *soc_data;
	struct tegra210_adsp_app apps[TEGRA210_ADSP_VIRT_REG_MAX];
	atomic_t reg_val[TEGRA210_ADSP_VIRT_REG_MAX];
	DECLARE_BITMAP(adma_usage, TEGRA210_ADSP_ADMA_BITMAP_COUNT);
	struct clk *ahub_clk;
	struct clk *ape_clk;
	struct clk *apb2ape_clk;
	struct work_struct override_freq_work;
	uint32_t i2s_rate;
	struct mutex mutex;
	int init_done;
	int adsp_started;
	uint32_t adma_ch_page;
	uint32_t adma_ch_start;
	uint32_t adma_ch_cnt;
	struct tegra210_adsp_path {
		uint32_t fe_reg;
		uint32_t be_reg;
		uint32_t channels;
		uint32_t format;
		uint32_t rate;
	} pcm_path[ADSP_FE_COUNT+1][2];
	struct sock *nl_sk;
};

static const struct snd_pcm_hardware adsp_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_DRAIN_TRIGGER,
	.formats		= SNDRV_PCM_FMTBIT_S8 |
				  SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.channels_min		= 1,
	.channels_max		= 2,
	.period_bytes_min	= 128,
	.period_bytes_max	= PAGE_SIZE * 2,
	.periods_min		= 1,
	.periods_max		= 8,
	.buffer_bytes_max	= PAGE_SIZE * 8,
	.fifo_size		= 4,
};

/* Following structure is ALSA-Compress specific */
static struct snd_compr_caps
	tegra210_adsp_compr_caps[SND_COMPRESS_CAPTURE + 1] = {
	[SND_COMPRESS_PLAYBACK] = {
		.num_codecs = 2,
		.direction = SND_COMPRESS_PLAYBACK,
		.min_fragment_size = 1024,
		.max_fragment_size = 1024 * 1024,        /* 1 MB */
		.min_fragments = 2,
		.max_fragments = 1024,
		.codecs = {
			[0] = SND_AUDIOCODEC_MP3,
			[1] = SND_AUDIOCODEC_AAC,
		},
	},
	[SND_COMPRESS_CAPTURE] = {
		.num_codecs = 0,
		.direction = SND_COMPRESS_CAPTURE,
	},
};

/* Following structure is ALSA-Compress specific */
static struct snd_compr_codec_caps adsp_compr_codec_caps[] = {
	[SND_AUDIOCODEC_MP3] = {
		.codec = SND_AUDIOCODEC_MP3,
		.num_descriptors = 1,
		.descriptor = {
			[0] = {
				.max_ch = 2,
				.sample_rates = {
					[0] = SNDRV_PCM_RATE_8000_48000,
				},
				.bit_rate = {
					[0] = 32000,
					[1] = 64000,
					[2] = 128000,
					[3] = 256000,
					[4] = 320000,
				},
				.num_bitrates = 5,
				.rate_control =
					SND_RATECONTROLMODE_CONSTANTBITRATE |
					SND_RATECONTROLMODE_VARIABLEBITRATE,
				.profiles = 0,
				.modes = SND_AUDIOCHANMODE_MP3_STEREO,
				.formats = SND_AUDIOSTREAMFORMAT_UNDEFINED,
				.min_buffer = 1024,
			},
		},
	},
	[SND_AUDIOCODEC_AAC] = {
		.codec = SND_AUDIOCODEC_AAC,
		.num_descriptors = 1,
		.descriptor = {
			[0] = {
				.max_ch = 2,
				.sample_rates = {
					[0] = SNDRV_PCM_RATE_8000_48000,
				},
				.bit_rate = {
					[0] = 32000,
					[1] = 64000,
					[2] = 128000,
					[3] = 256000,
					[4] = 320000,
				},
				.num_bitrates = 5,
				.rate_control =
					SND_RATECONTROLMODE_CONSTANTBITRATE |
					SND_RATECONTROLMODE_VARIABLEBITRATE,
				.profiles = SND_AUDIOPROFILE_AAC,
				.modes = SND_AUDIOMODE_AAC_LC,
				.formats = SND_AUDIOSTREAMFORMAT_MP4ADTS,
				.min_buffer = 1024,
			},
		},
	},
};

static status_t tegra210_adsp_msg_handler(uint32_t msg, void *data);
static int tegra210_adsp_app_default_msg_handler(struct tegra210_adsp_app *app,
					apm_msg_t *apm_msg);

/*
 * Utility functions
 */
/* ADSP virtual register read/write functions */
static uint32_t tegra210_adsp_reg_read(struct tegra210_adsp *adsp, uint32_t reg)
{
	return atomic_read(&adsp->reg_val[reg]);
}

static void tegra210_adsp_reg_write(struct tegra210_adsp *adsp,
				uint32_t reg, uint32_t val)
{
	atomic_set(&adsp->reg_val[reg], val);
	dev_vdbg(adsp->dev, "%s : 0x%x -> 0x%x\n", __func__, reg, val);
}

static void tegra210_adsp_reg_update_bits(struct tegra210_adsp *adsp,
					uint32_t reg, uint32_t mask,
					uint32_t val)
{
	uint32_t temp;

	temp = tegra210_adsp_reg_read(adsp, reg);
	val = (val & mask) | (temp & ~mask);
	tegra210_adsp_reg_write(adsp, reg, val);

	dev_vdbg(adsp->dev, "%s : 0x%x -> 0x%x\n", __func__, reg, val);
}

/* API to find Plugin app from name*/
static struct tegra210_adsp_app *tegra210_adsp_get_plugin(
				struct tegra210_adsp *adsp,
				const char *plugin_name)
{
	struct tegra210_adsp_app *app;
	int i;

	for (i = PLUGIN_START; i <= PLUGIN_END; i++) {
		app = &adsp->apps[i];
		if (!strcmp(app->desc->name, plugin_name))
			return app;
	}
	return NULL;
}

/* API to get source widget id connected to a widget */
static uint32_t tegra210_adsp_get_source(struct tegra210_adsp *adsp,
					 uint32_t reg)
{
	uint32_t source;

	source = tegra210_adsp_reg_read(adsp, reg);
	source &= TEGRA210_ADSP_WIDGET_SOURCE_MASK;
	source >>= TEGRA210_ADSP_WIDGET_SOURCE_SHIFT;

	return source;
}
/* ADSP shared memory allocate/free functions */
static int tegra210_adsp_preallocate_dma_buffer(struct device *dev, size_t size,
				struct snd_dma_buffer *buf)
{
	dev_vdbg(dev, "%s : size %d.", __func__, (uint32_t)size);

	buf->area = nvadsp_alloc_coherent(size, &buf->addr, GFP_KERNEL);
	if (!buf->area) {
		dev_err(dev, "Failed to pre-allocated DMA buffer.");
		return -ENOMEM;
	}

	buf->bytes = size;
	buf->private_data = NULL;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = dev;

	return 0;
}

static void tegra210_adsp_deallocate_dma_buffer(struct snd_dma_buffer *buf)
{
	dev_vdbg(buf->dev.dev, "%s : size %d.", __func__, (uint32_t)buf->bytes);

	if (!buf->area)
		return;

	nvadsp_free_coherent(buf->bytes, buf->area, buf->addr);
	buf->area = NULL;
	buf->addr = 0;
}

#if IS_ENABLED(CONFIG_TEGRA210_ADMA)
/* implemented in adma driver */
void tegra_adma_dump_ch_reg(void);
#endif

/* ADSP OS boot and init API */
static int tegra210_adsp_init(struct tegra210_adsp *adsp)
{
	int i, ret = 0;

	mutex_lock(&adsp->mutex);
	ret = nvadsp_os_load();
	if (ret < 0) {
		dev_err(adsp->dev, "Failed to load OS.");
		goto exit;
	}

	if (nvadsp_os_start()) {
		dev_err(adsp->dev, "Failed to start OS");
		goto exit;
	}

	/* Load ADSP audio apps */
	for (i = 0; i < adsp_app_count; i++) {
		adsp_app_desc[i].handle = nvadsp_app_load(
				adsp_app_desc[i].name,
				adsp_app_desc[i].fw_name);
		if (!adsp_app_desc[i].handle) {
			dev_err(adsp->dev, "Failed to load app %s",
						adsp_app_desc[i].name);
		}
	}

#if IS_ENABLED(CONFIG_TEGRA210_ADMA)
	/* set callback function for adsp to dump adma registers for debug */
	nvadsp_set_adma_dump_reg(&tegra_adma_dump_ch_reg);
#endif

	/* Suspend OS for now. Resume will happen via runtime pm calls */
	ret = nvadsp_os_suspend();
	if (ret < 0) {
		dev_err(adsp->dev, "Failed to suspend OS.");
		goto exit;
	}

	adsp->init_done = 1;

exit:
	mutex_unlock(&adsp->mutex);
	return ret;
}

static void tegra210_adsp_deinit(struct tegra210_adsp *adsp)
{
	mutex_lock(&adsp->mutex);
	if (adsp->init_done) {
		nvadsp_os_stop();
		adsp->init_done = 0;
	}
	mutex_unlock(&adsp->mutex);
}

/* ADSP-CPU message send-receive utility functions */
static int tegra210_adsp_get_msg(apm_shared_state_t *apm, apm_msg_t *apm_msg)
{
	apm_msg->msgq_msg.size = MSGQ_MSG_WSIZE(apm_msg_t) -
		MSGQ_MESSAGE_HEADER_WSIZE;
	return msgq_dequeue_message(&apm->msgq_send.msgq,
		&apm_msg->msgq_msg);
}

static int tegra210_adsp_get_raw_data_msg(apm_shared_state_t *apm,
		apm_raw_data_msg_t *apm_msg)
{
	apm_msg->msgq_msg.size = MSGQ_MSG_WSIZE(apm_raw_data_msg_t) -
		MSGQ_MESSAGE_HEADER_WSIZE;

	return msgq_dequeue_message(&apm->msgq_send.msgq,
		&apm_msg->msgq_msg);
}

static int tegra210_adsp_send_msg(struct tegra210_adsp_app *app,
				  apm_msg_t *apm_msg, uint32_t flags)
{
	int ret = 0;
	unsigned long flag;

	if (flags & TEGRA210_ADSP_MSG_FLAG_NEED_ACK) {
		if (flags & TEGRA210_ADSP_MSG_FLAG_HOLD) {
			pr_err("%s: ACK requires FLAG_SEND, ignoring\n",
				__func__);
			flags &= ~TEGRA210_ADSP_MSG_FLAG_NEED_ACK;
		} else {
			apm_msg->msg.call_params.method |=
				NVFX_APM_METHOD_ACK_BIT;
		}
	}

	if (!IS_APM_IN(app->reg)) {
		uint32_t source;
		while (IS_ADSP_APP(app->reg) && !IS_APM_IN(app->reg)) {
			source = tegra210_adsp_get_source(app->adsp, app->reg);
			app = &app->adsp->apps[source];
		}
		if (!IS_APM_IN(app->reg)) {
			pr_err("%s: No APM found, skip msg sending\n",
				__func__);
			return ret;
		}
	}

	spin_lock_irqsave(&app->apm_msg_queue_lock, flag);
	ret = msgq_queue_message(&app->apm->msgq_recv.msgq, &apm_msg->msgq_msg);
	spin_unlock_irqrestore(&app->apm_msg_queue_lock, flag);
	if (ret < 0) {
		/* Wakeup APM to consume messages and give it some time */
		ret = nvadsp_mbox_send(&app->apm_mbox, apm_cmd_msg_ready,
			NVADSP_MBOX_SMSG, false, 0);
		if (ret) {
			pr_err("%s: Failed to send mailbox message id %d ret %d\n",
				__func__, app->apm->mbox_id, ret);
		}
		mdelay(20);
		/* Attempt queueing again */
		spin_lock_irqsave(&app->apm_msg_queue_lock, flag);
		ret = msgq_queue_message(&app->apm->msgq_recv.msgq,
				&apm_msg->msgq_msg);
		spin_unlock_irqrestore(&app->apm_msg_queue_lock, flag);
		if (ret < 0) {
			pr_err("%s: Failed to queue message ret %d \
				rd %d and wr %d pointer %p mbox_id %d\n",
				__func__, ret,
				app->apm->msgq_recv.msgq.read_index,
				app->apm->msgq_recv.msgq.write_index,
				&app->apm->msgq_recv.msgq, app->apm->mbox_id);
			return ret;
		}
	}

	if (flags & TEGRA210_ADSP_MSG_FLAG_HOLD)
		return 0;

	ret = nvadsp_mbox_send(&app->apm_mbox, apm_cmd_msg_ready,
		NVADSP_MBOX_SMSG, false, 0);
	if (ret) {
		pr_err("%s: Failed to send mailbox message id %d ret %d\n",
			__func__, app->apm->mbox_id, ret);
	}
	if (!(flags & TEGRA210_ADSP_MSG_FLAG_NEED_ACK))
		return ret;

	ret = wait_for_completion_interruptible_timeout(
		app->msg_complete,
		msecs_to_jiffies(ADSP_RESPONSE_TIMEOUT));
	if (WARN_ON(ret == 0))
		pr_err("%s: ACK timed out %d rd %d wr %d mbox_id %d \
			msg_q 0x%p\n", __func__, app->reg,
			app->apm->msgq_recv.msgq.read_index,
			app->apm->msgq_recv.msgq.write_index,
			app->apm->mbox_id, &app->apm->msgq_recv.msgq);
	return ret;
}

static int tegra210_adsp_send_raw_data_msg(struct tegra210_adsp_app *app,
				  apm_raw_data_msg_t *apm_msg)
{
	int ret = 0;
	struct tegra210_adsp_app *apm = app;
	unsigned long flag;

	/* Find parent APM to wait for ACK*/
	if (!IS_APM_IN(apm->reg)) {
		uint32_t source;
		while (IS_ADSP_APP(apm->reg) && !IS_APM_IN(apm->reg)) {
			source = tegra210_adsp_get_source(apm->adsp, apm->reg);
			apm = &apm->adsp->apps[source];
		}
		if (!IS_APM_IN(apm->reg)) {
			pr_err("%s: No APM found, skip ACK wait\n", __func__);
			return ret;
		}
	}
	apm_msg->msg.call_params.method |= NVFX_APM_METHOD_ACK_BIT;

	spin_lock_irqsave(&apm->apm_msg_queue_lock, flag);
	ret = msgq_queue_message(&app->apm->msgq_recv.msgq, &apm_msg->msgq_msg);
	spin_unlock_irqrestore(&apm->apm_msg_queue_lock, flag);
	if (ret < 0) {
		pr_err("%s failed: msgq full\n", __func__);
		return ret;
	}

	ret = nvadsp_mbox_send(&app->apm_mbox, apm_cmd_raw_data_ready,
		NVADSP_MBOX_SMSG, true, 100);
	if (ret) {
		pr_err("Failed to send mailbox message id %d ret %d\n",
			app->apm->mbox_id, ret);
	}

	ret = wait_for_completion_interruptible_timeout(
		apm->raw_msg_write_complete,
		msecs_to_jiffies(ADSP_RESPONSE_TIMEOUT));
	if (WARN_ON(ret == 0))
		pr_err("%s: ACK timed out %d\n", __func__, app->reg);

	return ret;
}

static int tegra210_adsp_send_remove_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_remove_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_remove_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_remove_all;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_send_connect_msg(struct tegra210_adsp_app *src,
					struct tegra210_adsp_app *dst,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_connect_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_connect_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_connect;
	apm_msg.msg.fx_connect_params.plugin_src.pvoid = IS_APM_IN(src->reg) ?
		NULL : src->plugin->plugin.pvoid;
	apm_msg.msg.fx_connect_params.pin_src = 0;
	apm_msg.msg.fx_connect_params.plugin_dst.pvoid = IS_APM_OUT(dst->reg) ?
		NULL : dst->plugin->plugin.pvoid;
	apm_msg.msg.fx_connect_params.pin_dst = 0;

	return tegra210_adsp_send_msg(src, &apm_msg, flags);
}

static int tegra210_adsp_send_io_buffer_msg(struct tegra210_adsp_app *app,
					dma_addr_t addr, size_t size,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_io_buffer_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_io_buffer_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_io_buffer;
	apm_msg.msg.io_buffer_params.pin_type = IS_APM_IN(app->reg) ?
		NVFX_PIN_TYPE_INPUT : NVFX_PIN_TYPE_OUTPUT;
	apm_msg.msg.io_buffer_params.pin_id = 0;
	apm_msg.msg.io_buffer_params.addr.ptr = (uint64_t)addr;
	apm_msg.msg.io_buffer_params.size = size;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_send_period_size_msg(struct tegra210_adsp_app *app,
					size_t size, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size =
		MSGQ_MSG_WSIZE(apm_notification_params_t);
	apm_msg.msg.call_params.size =
		sizeof(apm_notification_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_notification_size;
	apm_msg.msg.notification_params.pin_type = IS_APM_IN(app->reg) ?
		NVFX_PIN_TYPE_INPUT : NVFX_PIN_TYPE_OUTPUT;
	apm_msg.msg.notification_params.pin_id = 0;
	apm_msg.msg.notification_params.size = size;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_adma_params_msg(struct tegra210_adsp_app *app,
					nvfx_adma_init_params_t *params,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_set_param;
	apm_msg.msg.fx_set_param_params.plugin.pvoid =
		app->plugin->plugin.pvoid;

	params->call_params.size = sizeof(nvfx_adma_init_params_t);
	params->call_params.method = nvfx_adma_method_init;
	memcpy(&apm_msg.msg.fx_set_param_params.params, params,
		sizeof(*params));

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_eavbdma_params_msg(struct tegra210_adsp_app *app,
					nvfx_eavbdma_init_params_t *params,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_set_param;
	apm_msg.msg.fx_set_param_params.plugin.pvoid =
		app->plugin->plugin.pvoid;

	params->call_params.size = sizeof(nvfx_eavbdma_init_params_t);
	params->call_params.method = nvfx_eavbdma_method_init;
	memcpy(&apm_msg.msg.fx_set_param_params.params, params,
		sizeof(*params));

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static void tegra_adsp_override_freq_worker(struct work_struct *work)
{
	adsp_override_freq(INT_MAX);
}

static int tegra210_adsp_send_state_msg(struct tegra210_adsp_app *app,
					int32_t state, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(nvfx_set_state_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_set_state_params_t);
	apm_msg.msg.call_params.method = nvfx_method_set_state;
	apm_msg.msg.state_params.state = state;

	/* Spike ADSP freq to max when app transitions to active */
	/* state; DFS will thereafter find appropriate rate      */
	if ((state == nvfx_state_active) && (app->override_freq_work != NULL))
		schedule_work(app->override_freq_work);

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_send_flush_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(nvfx_flush_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_flush_params_t);
	apm_msg.msg.call_params.method = nvfx_method_flush;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_send_reset_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(nvfx_reset_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_reset_params_t);
	apm_msg.msg.call_params.method = nvfx_method_reset;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_send_eos_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_eos_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_eos_params_t);
	apm_msg.msg.call_params.method = nvfx_method_set_eos;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_send_pos_msg(struct tegra210_adsp_app *app,
					uint32_t pos, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_position_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_position_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_position;
	apm_msg.msg.position_params.pin_type = IS_APM_IN(app->reg) ?
		NVFX_PIN_TYPE_INPUT : NVFX_PIN_TYPE_OUTPUT;
	apm_msg.msg.position_params.pin_id = 0;
	apm_msg.msg.position_params.offset = pos;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

static int tegra210_adsp_send_data_request_msg(struct tegra210_adsp_app *app,
					uint32_t size, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_read_request_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_read_request_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_read_data;
	apm_msg.msg.fx_read_request_params.plugin.pvoid = app->plugin->plugin.pvoid;
	apm_msg.msg.fx_read_request_params.req_size = size;

	return tegra210_adsp_send_msg(app, &apm_msg, flags);
}

/* ADSP app init/de-init APIs */
static int tegra210_adsp_app_init(struct tegra210_adsp *adsp,
				struct tegra210_adsp_app *app)
{
	int ret = 0;

	/* If app is already open or it is APM output pin don't open app */
	if (app->info || IS_APM_OUT(app->reg))
		return 0;

	if (!app->desc->handle) {
		return -ENODEV;
	}
	app->info = nvadsp_app_init(app->desc->handle, NULL);
	if (IS_ERR_OR_NULL(app->info)) {
		dev_err(adsp->dev, "Failed to init app %s(%s).",
			app->desc->name, app->desc->fw_name);
		return -ENODEV;
	}

	dev_info(adsp->dev, "apm init app %s done\n", app->desc->name);

	spin_lock_init(&app->lock);
	spin_lock_init(&app->apm_msg_queue_lock);

	app->adsp = adsp;
	app->msg_handler = tegra210_adsp_app_default_msg_handler;
	app->override_freq_work = &adsp->override_freq_work;
	app->plugin = PLUGIN_SHARED_MEM(app->info->mem.shared);
	if (IS_APM_IN(app->reg)) {
		uint32_t apm_out_reg = APM_OUT_START +
					(app->reg - APM_IN_START);
		struct tegra210_adsp_app *apm_out = &adsp->apps[apm_out_reg];

		app->apm = APM_SHARED_STATE(app->info->mem.shared);
		app->info->stack_size = apm_stack_size[app->reg - APM_IN_START];

		ret = nvadsp_mbox_open(&app->apm_mbox,
			&app->apm->mbox_id,
			app->desc->name, tegra210_adsp_msg_handler, app);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to open mailbox %s(%s).",
				app->desc->name, app->desc->fw_name);
			goto err_app_exit;
		}

		app->msg_complete = devm_kzalloc(adsp->dev,
					sizeof(struct completion), GFP_KERNEL);
		if (!app->msg_complete) {
			dev_err(adsp->dev, "Failed to allocate completion struct.");
			return -ENOMEM;
		}

		app->raw_msg_read_complete = devm_kzalloc(adsp->dev,
					sizeof(struct completion), GFP_KERNEL);

		if (!app->raw_msg_read_complete) {
			dev_err(adsp->dev,
				"Failed to allocate read completion struct.");
			return -ENOMEM;
		}

		app->raw_msg_write_complete = devm_kzalloc(adsp->dev,
					sizeof(struct completion), GFP_KERNEL);

		if (!app->raw_msg_write_complete) {
			dev_err(adsp->dev,
				"Failed to allocate read completion struct.");
			return -ENOMEM;
		}
		init_completion(app->msg_complete);
		init_completion(app->raw_msg_read_complete);
		init_completion(app->raw_msg_write_complete);

		ret = nvadsp_app_start(app->info);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to start adsp app");
			goto err_mbox_close;
		}
		dev_info(adsp->dev, "apm start app %d msg_q 0x%p\n"
			,app->apm->mbox_id, &app->apm->msgq_recv.msgq);
		/* Copy APM IN app data to APM OUT app */
		apm_out->info = app->info;
		apm_out->plugin = app->plugin;
		apm_out->apm = app->apm;
		apm_out->adsp = app->adsp;
		apm_out->apm_mbox = app->apm_mbox;
		apm_out->msg_complete = app->msg_complete;
		apm_out->raw_msg_read_complete = app->raw_msg_read_complete;
		apm_out->raw_msg_write_complete = app->raw_msg_write_complete;
	} else if (IS_ADMA(app->reg)) {
		app->adma_chan = find_first_zero_bit(adsp->adma_usage,
					adsp->adma_ch_cnt);
		if (app->adma_chan >= adsp->adma_ch_cnt) {
			dev_err(adsp->dev, "All ADMA channels are busy");
			return -EBUSY;
		}
		__set_bit(app->adma_chan, adsp->adma_usage);
		app->adma_chan += adsp->adma_ch_start;
	}
	return 0;

err_mbox_close:
	nvadsp_mbox_close(&app->apm_mbox);
err_app_exit:
	app->info = NULL;
	return 0;
}

static void tegra210_adsp_app_deinit(struct tegra210_adsp *adsp,
				struct tegra210_adsp_app *app)
{
	/* TODO:
	 * Current usecases use static paths so app_deinit functionality
	 * is not needed and adds overhead of freeing and allocating apps
	 * everytime. Add app deinit functionality properly if needed in
	 * future.
	 */
	return;
}

/* API to connect two APMs */
static int tegra210_adsp_connect_apm(struct tegra210_adsp *adsp,
				struct tegra210_adsp_app *app)
{
	uint32_t source = tegra210_adsp_get_source(adsp, app->reg);
	struct tegra210_adsp_app *src = &adsp->apps[source];
	int ret = 0;

	/* If both APMs are in connected state no need to
	   send connect message */
	if (app->connect && src->connect)
		return 0;

	dev_vdbg(adsp->dev, "Connecting APM 0x%x -> 0x%x",
		src->reg, app->reg);

	ret = tegra210_adsp_send_connect_msg(src, app,
		TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0) {
		dev_err(adsp->dev, "Connect msg failed. err %d.", ret);
		return ret;
	}
	return 1;
}

/* Recursive function to connect plugins under a APM
   Returns BE/FE on the pcm path */
static int tegra210_adsp_connect_plugin(struct tegra210_adsp *adsp,
					struct tegra210_adsp_app *app,
					uint32_t *apm_in_src)
{
	struct tegra210_adsp_app *src;
	uint32_t source;
	int ret = 0;

	source = tegra210_adsp_get_source(adsp, app->reg);
	if (!IS_ADSP_APP(source))
		return -ENODEV;

	src = &adsp->apps[source];
	if (!IS_APM_IN(src->reg)) {
		ret = tegra210_adsp_connect_plugin(adsp, src, apm_in_src);
		if (ret < 0)
			return ret;
	} else {
		source = tegra210_adsp_get_source(adsp, src->reg);
		if (IS_APM_OUT(source)) {
			/* connect plugins inside next APM */
			ret = tegra210_adsp_connect_plugin(adsp,
				&adsp->apps[source], apm_in_src);
			if (ret < 0)
				return ret;
			/* connect APM_IN to APM_OUT */
			ret = tegra210_adsp_connect_apm(adsp, src);
			if (ret < 0)
				return ret;
		} else {
			/* return if APM_IN is not
			   connected to valid inputs */
			if (!IS_ADSP_FE(source) &&
				!IS_ADSP_ADMAIF(source))
				return -ENODEV;
			if (apm_in_src)
				*apm_in_src = source;
		}
	}
	app->apm = src->apm;
	app->apm_mbox = src->apm_mbox;

	/* If App is already connected and source connections have not changed
	   no need to again send connect message */
	if (!ret && app->connect)
		return 0;

	dev_vdbg(adsp->dev, "Connecting plugin 0x%x -> 0x%x",
		src->reg, app->reg);

	ret = tegra210_adsp_send_connect_msg(src, app,
		TEGRA210_ADSP_MSG_FLAG_SEND | TEGRA210_ADSP_MSG_FLAG_NEED_ACK);
	if (ret < 0) {
		dev_err(adsp->dev, "Connect msg failed. err %d.", ret);
		return ret;
	}
	app->connect = 1;

	/* return 1 if new connection is established */
	return 1;
}

/* Manages FE/BE plugins and deletes if fe_apm is specified */
static void tegra210_adsp_manage_plugin(struct tegra210_adsp *adsp,
		uint32_t end_reg, uint32_t apm_out, struct tegra210_adsp_app *fe_apm)
{
	uint32_t j, fe_reg, be_reg;

	if (IS_ADSP_FE(end_reg)) {
		/* manage playback path */
		fe_reg = end_reg;
		be_reg = 0;
		for (j = ADSP_ADMAIF_START; j <= ADSP_ADMAIF_END; j++) {
			if (tegra210_adsp_get_source(adsp, j) == apm_out) {
				be_reg = j;
				break;
			}
		}
		if (be_reg && fe_reg) {
			if (fe_apm) {
				dev_vdbg(adsp->dev, "Remove playback FE %d -- BE %d pair",
					fe_reg, be_reg);
				tegra210_adsp_send_remove_msg(fe_apm,
						TEGRA210_ADSP_MSG_FLAG_SEND);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].fe_reg = 0;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].be_reg = 0;
			} else {
				dev_vdbg(adsp->dev, "Found playback FE %d -- BE %d pair",
					fe_reg, be_reg);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].fe_reg = fe_reg;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].be_reg = be_reg;
			}
		}
	} else if (IS_ADSP_ADMAIF(end_reg)) {
		/* manage record path */
		fe_reg = 0;
		be_reg = end_reg;
		for (j = ADSP_FE_START; j <= ADSP_FE_END; j++) {
			if (tegra210_adsp_get_source(adsp, j) == apm_out) {
				fe_reg = j;
				break;
			}
		}
		if (be_reg && fe_reg) {
			if (fe_apm) {
				dev_vdbg(adsp->dev, "Remove record FE %d -- BE %d pair",
					fe_reg, be_reg);
				tegra210_adsp_send_remove_msg(fe_apm,
						TEGRA210_ADSP_MSG_FLAG_SEND);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].fe_reg = 0;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].be_reg = 0;
			} else {
				dev_vdbg(adsp->dev, "Found playback FE %d -- BE %d pair",
					fe_reg, be_reg);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].fe_reg = fe_reg;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].be_reg = be_reg;
			}
		}
	}
}

/* Iterate over all APMs and establish pending connections */
static int tegra210_adsp_update_connection(struct tegra210_adsp *adsp)
{
	int i, ret;
	uint32_t end_reg;

	for (i = APM_OUT_START; i <= APM_OUT_END; i++) {
		ret = tegra210_adsp_connect_plugin(adsp, &adsp->apps[i], &end_reg);
		if (ret >= 0) {
			/* Record FE/BE pair for every successful connection */
			tegra210_adsp_manage_plugin(adsp, end_reg, i, NULL);
		}
	}

	return 0;
}

/* Remove the plugin connections inside associated APM */
static int tegra210_adsp_remove_connection(struct tegra210_adsp *adsp,
		struct tegra210_adsp_app *plugin)
{
	struct tegra210_adsp_app *app;
	uint32_t i, source, apm_out = TEGRA210_ADSP_NONE;

	if (!IS_ADSP_APP(plugin->reg))
		return 0;

	for (i = APM_OUT_START; i <= APM_OUT_END; i++) {
		app =  &adsp->apps[i];
		/* if the path is already broken, do not continue */
		if (!app->connect)
			continue;
		while (IS_ADSP_APP(app->reg)) {
			if (app->reg == plugin->reg) {
				apm_out = i;
				break;
			}
			source = tegra210_adsp_get_source(adsp, app->reg);
			app =  &adsp->apps[source];
		}
		if (apm_out != TEGRA210_ADSP_NONE)
			break;
	}
	/* if plugin is not part of any APM, return here */
	if (apm_out == TEGRA210_ADSP_NONE)
		return 0;

	/* disconnect the plugins inside APM */
	app = &adsp->apps[apm_out];
	while (!IS_APM_IN(app->reg)) {
		source = tegra210_adsp_get_source(adsp, app->reg);
		if (!IS_ADSP_APP(source))
			break;
		app->connect = 0;
		app = &adsp->apps[source];
	}

	/* delete the plugins inside APM */
	if (IS_APM_IN(app->reg)) {
		/* clear the FE/BE list */
		tegra210_adsp_manage_plugin(adsp,
			tegra210_adsp_get_source(adsp, app->reg),
			apm_out, app);
	}
	return 0;
}

/* ADSP socket message handlers */
static void tegra210_adsp_nl_recv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;

	nlh = (struct nlmsghdr *)skb->data;
	pr_info("Established client: %d\n", nlh->nlmsg_pid);

	return;
}

static void tegra210_adsp_nl_send_msg(struct tegra210_adsp *adsp,
	apm_fx_error_event_params_t *apm_err_msg)
{

	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int res;
	struct adsp_event_nlmsg *msg;

	if (adsp->nl_sk == NULL) {
		pr_err("socket not created\n");
		return;
	}

	skb = nlmsg_new(sizeof(struct adsp_event_nlmsg), 0);
	if (!skb) {
		pr_err("Failed to allocate new skb\n");
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, sizeof(struct adsp_event_nlmsg), 0);
	NETLINK_CB(skb).dst_group = NETLINK_ADSP_EVENT_GROUP;    /* to mcast group 1<<0 */

	msg = (struct adsp_event_nlmsg *)nlmsg_data(nlh);
	msg->err = apm_err_msg->err;
	memcpy(msg->data, apm_err_msg->data,
		(sizeof(uint32_t) * NVFX_MAX_CALL_PARAMS_WSIZE));

	res = nlmsg_multicast(adsp->nl_sk, skb, 0, NETLINK_ADSP_EVENT_GROUP, 0);
	if (res < 0)
		pr_err("Error while sending back to user: %d\n", res);

	return;
}

/* ADSP mailbox message handler */
static status_t tegra210_adsp_msg_handler(uint32_t msg, void *data)
{
	struct tegra210_adsp_app *app = data;
	unsigned long flags;
	apm_msg_t apm_msg;
	int ret = 0;

	spin_lock_irqsave(&app->lock, flags);

	switch (msg) {
	case apm_cmd_msg_ready: {
		ret = tegra210_adsp_get_msg(app->apm, &apm_msg);
		if (ret < 0) {
			pr_err("Dequeue failed %d.", ret);
			break;
		}

		if (app->msg_handler)
			ret = app->msg_handler(app, &apm_msg);
	}
	break;
	case apm_cmd_raw_data_ready: {
		apm_raw_data_msg_t *raw_msg =
			kzalloc(sizeof(apm_raw_data_msg_t), GFP_ATOMIC);
		if (!raw_msg) {
			ret = -ENOMEM;
			break;
		}

		ret = tegra210_adsp_get_raw_data_msg(app->apm, raw_msg);
		if (ret < 0) {
			pr_err("Dequeue failed raw %d.", ret);
			kfree(raw_msg);
			break;
		}
		memcpy(app->read_data.data,
				raw_msg->msg.fx_raw_data_params.data,
				sizeof(app->read_data.data));
		kfree(raw_msg);
		complete(app->raw_msg_read_complete);
	}
	break;
	default:
		pr_err("Unsupported mailbox msg %d.", msg);
	}

	spin_unlock_irqrestore(&app->lock, flags);
	return ret;
}

static int tegra210_adsp_app_default_msg_handler(struct tegra210_adsp_app *app,
					apm_msg_t *apm_msg)
{
	switch (apm_msg->msg.call_params.method) {
	case nvfx_apm_method_ack:
		complete(app->msg_complete);
		break;
	case nvfx_apm_method_raw_ack:
		complete(app->raw_msg_write_complete);
		break;
	case nvfx_apm_method_fx_error_event:
		tegra210_adsp_nl_send_msg(app->adsp,
			&apm_msg->msg.fx_error_event_params);
		break;
	default:
		pr_err("Unsupported cmd %d.", apm_msg->msg.call_params.method);
	}
	return 0;
}

static int tegra210_adsp_pcm_ack(struct snd_pcm_substream *substream)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t pos;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s %d", __func__, (int)runtime->control->appl_ptr);

	pos = frames_to_bytes(runtime,
		runtime->control->appl_ptr % runtime->buffer_size);
	ret = tegra210_adsp_send_pos_msg(prtd->fe_apm, pos,
		TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0) {
		dev_err(prtd->dev, "Failed to send write position.");
		return ret;
	}

	return ret;
}

static int tegra210_adsp_pcm_msg_handler(struct tegra210_adsp_app *app,
					apm_msg_t *apm_msg)
{
	struct tegra210_adsp_pcm_rtd *prtd = app->private_data;
	struct snd_pcm_runtime *runtime;

	switch (apm_msg->msg.call_params.method) {
	case nvfx_apm_method_set_position:
		if (!prtd || !prtd->substream)
			return 0;
		runtime = prtd->substream->runtime;
		snd_pcm_period_elapsed(prtd->substream);
		if ((IS_MMAP_ACCESS(runtime->access))) {
			if (prtd->prev_appl_ptr !=
				runtime->control->appl_ptr) {
				prtd->prev_appl_ptr = runtime->control->appl_ptr;
				tegra210_adsp_pcm_ack(prtd->substream);
			}
		}
		break;
	case nvfx_method_set_eos:
		/* Nothing specific to be done here as DRAIN */
		/* is implemented in native PCM driver       */
		break;
	case nvfx_apm_method_ack:
		complete(app->msg_complete);
		break;
	case nvfx_apm_method_fx_error_event:
		tegra210_adsp_nl_send_msg(app->adsp,
			&apm_msg->msg.fx_error_event_params);
		break;
	default:
		dev_err(prtd->dev, "Unsupported cmd %d.",
			apm_msg->msg.call_params.method);
	}
	return 0;
}

static int tegra210_adsp_compr_msg_handler(struct tegra210_adsp_app *app,
					   apm_msg_t *apm_msg)
{
	struct tegra210_adsp_compr_rtd *prtd = app->private_data;

	if (!prtd) {
		return 0;
	}

	switch (apm_msg->msg.call_params.method) {
	case nvfx_apm_method_set_position:
		snd_compr_fragment_elapsed(prtd->cstream);
		break;
	case nvfx_method_set_eos:
		if (!prtd->is_draining) {
			dev_warn(prtd->dev, "EOS reached before drain");
			break;
		}
		snd_compr_drain_notify(prtd->cstream);
		prtd->is_draining = 0;
		break;
	case nvfx_apm_method_ack:
		complete(app->msg_complete);
		break;
	case nvfx_apm_method_fx_error_event:
		tegra210_adsp_nl_send_msg(app->adsp,
			&apm_msg->msg.fx_error_event_params);
		break;
	default:
		dev_err(prtd->dev, "Unsupported cmd %d.",
			apm_msg->msg.call_params.method);
	}
	return 0;
}

/* Compress call-back APIs */
static int tegra210_adsp_compr_open(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->device->private_data;
	struct tegra210_adsp *adsp =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct tegra210_adsp_compr_rtd *prtd;
	uint32_t fe_reg = rtd->codec_dai->id;
	int ret;
	int i;

	dev_vdbg(adsp->dev, "%s : DAI ID %d", __func__, rtd->codec_dai->id);

	if (!adsp->init_done)
		return -ENODEV;

	if (!adsp->pcm_path[fe_reg][cstream->direction].fe_reg ||
		!adsp->pcm_path[fe_reg][cstream->direction].be_reg) {
		dev_err(adsp->dev, "Broken Path%d - FE not linked to BE", fe_reg);
		return -EPIPE;
	}

	prtd = devm_kzalloc(adsp->dev, sizeof(struct tegra210_adsp_compr_rtd),
		GFP_KERNEL);
	if (!prtd) {
		dev_err(adsp->dev, "Failed to allocate adsp rtd.");
		return -ENOMEM;
	}

	/* Find out the APM connected with ADSP-FE DAI */
	for (i = APM_IN_START; i <= APM_IN_END; i++) {
		struct tegra210_adsp_app *app = &adsp->apps[i];
		uint32_t source = tegra210_adsp_get_source(adsp, app->reg);

		if (source == fe_reg) {
			app->msg_handler = tegra210_adsp_compr_msg_handler;
			app->private_data = prtd;
			app->fe = 1;
			prtd->fe_apm = app;
			break;
		}
	}

	if (!prtd->fe_apm) {
		dev_err(adsp->dev, "No FE APM found\n");
		devm_kfree(adsp->dev, prtd);
		return -ENODEV;
	}

	prtd->cstream = cstream;
	prtd->dev = adsp->dev;
	cstream->runtime->private_data = prtd;
	ret = pm_runtime_get_sync(adsp->dev);
	if (ret < 0) {
		dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
			__func__, ret);
	}
	return ret;
}

static int tegra210_adsp_compr_free(struct snd_compr_stream *cstream)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	unsigned long flags;

	if (!prtd)
		return -ENODEV;

	tegra210_adsp_send_reset_msg(prtd->fe_apm,
		TEGRA210_ADSP_MSG_FLAG_SEND | TEGRA210_ADSP_MSG_FLAG_NEED_ACK);

	pm_runtime_put(prtd->dev);

	tegra210_adsp_deallocate_dma_buffer(&prtd->buf);

	spin_lock_irqsave(&prtd->fe_apm->lock, flags);

	/* Reset msg handler to disable msg processing */
	prtd->fe_apm->msg_handler = tegra210_adsp_app_default_msg_handler;

	spin_unlock_irqrestore(&prtd->fe_apm->lock, flags);

	cstream->runtime->private_data = NULL;
	devm_kfree(prtd->dev, prtd);

	return 0;
}

static int tegra210_adsp_compr_set_params(struct snd_compr_stream *cstream,
			struct snd_compr_params *params)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	int ret = 0;

	if (!prtd)
		return -ENODEV;

	dev_vdbg(prtd->dev, "%s codec %d rate %d chan %d frag size %d frag %d",
		 __func__, params->codec.id,
		 snd_pcm_rate_bit_to_rate(params->codec.sample_rate),
		 params->codec.ch_in, params->buffer.fragment_size,
		 params->buffer.fragments);

	ret = tegra210_adsp_preallocate_dma_buffer(prtd->dev,
		params->buffer.fragment_size * params->buffer.fragments,
		&prtd->buf);
	if (ret < 0)
		return ret;

	ret = tegra210_adsp_send_io_buffer_msg(prtd->fe_apm, prtd->buf.addr,
					prtd->buf.bytes,
					TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0) {
		dev_err(prtd->dev, "IO buffer send msg failed. err %d.", ret);
		return ret;
	}

	ret = tegra210_adsp_send_period_size_msg(prtd->fe_apm,
					params->buffer.fragment_size,
					TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0) {
		dev_err(prtd->dev, "Period size send msg failed. err %d.", ret);
		return ret;
	}

	memcpy(&prtd->codec, &params->codec, sizeof(struct snd_codec));
	return 0;
}

static int tegra210_adsp_compr_get_params(struct snd_compr_stream *cstream,
			struct snd_codec *codec)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;

	memcpy(codec, &prtd->codec, sizeof(struct snd_codec));
	return 0;
}

static int tegra210_adsp_compr_trigger(struct snd_compr_stream *cstream,
					int cmd)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s : cmd %d", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_active,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state start.");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
				nvfx_state_active,
				TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state resume");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state stop");
			return ret;
		}

		ret = tegra210_adsp_send_flush_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to reset");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state pause");
			return ret;
		}
		break;
	case SND_COMPR_TRIGGER_DRAIN:
		prtd->is_draining = 1;
		ret = tegra210_adsp_send_eos_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state drain");
			return ret;
		}
		break;
	default:
		dev_err(prtd->dev, "Unsupported state.");
		return -EINVAL;
	}
	return 0;
}

static int tegra210_adsp_compr_copy(struct snd_compr_stream *cstream,
			char __user *buf, size_t count)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	struct snd_compr_runtime *runtime = cstream->runtime;
	void *dstn;
	size_t copy;
	u64 app_pointer;

	dev_vdbg(prtd->dev, "%s : size %d", __func__, (uint32_t)count);

	if (!count)
		return 0;

	app_pointer = div64_u64(runtime->total_bytes_available,
				runtime->buffer_size);
	app_pointer = runtime->total_bytes_available -
			(app_pointer * runtime->buffer_size);
	dstn = prtd->buf.area + app_pointer;

	if (count < runtime->buffer_size - app_pointer) {
		if (copy_from_user(dstn, buf, count))
			return -EFAULT;
	} else {
		copy = runtime->buffer_size - app_pointer;
		if (copy_from_user(dstn, buf, copy))
			return -EFAULT;
		if (copy_from_user(prtd->buf.area, buf + copy, count - copy))
			return -EFAULT;
	}
	tegra210_adsp_send_pos_msg(prtd->fe_apm,
	    (runtime->total_bytes_available + count) % runtime->buffer_size,
	    TEGRA210_ADSP_MSG_FLAG_SEND);

	return count;
}

static int tegra210_adsp_compr_pointer(struct snd_compr_stream *cstream,
			struct snd_compr_tstamp *tstamp)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	struct tegra210_adsp_app *app = prtd->fe_apm;
	nvfx_shared_state_t *shared = &app->apm->nvfx_shared_state;
	uint32_t frames_played =
		(shared->output[0].bytes >> 1) / prtd->codec.ch_in;

	tstamp->byte_offset = shared->input[0].bytes %
		cstream->runtime->buffer_size;
	tstamp->copied_total = shared->input[0].bytes;
	tstamp->pcm_frames = frames_played;
	/* TODO : calculate IO frames correctly */
	tstamp->pcm_io_frames = frames_played;
	tstamp->sampling_rate =
		snd_pcm_rate_bit_to_rate(prtd->codec.sample_rate);

	dev_vdbg(prtd->dev, "%s off %d copied %d pcm %d pcm io %d",
		 __func__, (int)tstamp->byte_offset, (int)tstamp->copied_total,
		 (int)tstamp->pcm_frames, (int)tstamp->pcm_io_frames);
	return 0;
}

static int tegra210_adsp_compr_get_caps(struct snd_compr_stream *cstream,
			struct snd_compr_caps *caps)
{
	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		memcpy(caps, &tegra210_adsp_compr_caps[SND_COMPRESS_PLAYBACK],
			sizeof(struct snd_compr_caps));
	else
		memcpy(caps, &tegra210_adsp_compr_caps[SND_COMPRESS_CAPTURE],
			sizeof(struct snd_compr_caps));

	return 0;
}

static int tegra210_adsp_compr_codec_caps(struct snd_compr_stream *cstream,
			struct snd_compr_codec_caps *codec_caps)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;

	dev_vdbg(prtd->dev, "%s : codec %d", __func__, codec_caps->codec);

	if (!codec_caps->codec)
		codec_caps->codec = prtd->codec.id;

	switch (codec_caps->codec) {
	case SND_AUDIOCODEC_MP3:
		memcpy(codec_caps, &adsp_compr_codec_caps[SND_AUDIOCODEC_MP3],
			sizeof(struct snd_compr_codec_caps));
		return 0;
	case SND_AUDIOCODEC_AAC:
		memcpy(codec_caps, &adsp_compr_codec_caps[SND_AUDIOCODEC_AAC],
			sizeof(struct snd_compr_codec_caps));
		return 0;
	default:
		dev_err(prtd->dev, "Unsupported codec %d", codec_caps->codec);
		return -EINVAL;
	}
	return 0;
}

static struct snd_compr_ops tegra210_adsp_compr_ops = {

	.open = tegra210_adsp_compr_open,
	.free = tegra210_adsp_compr_free,
	.set_params = tegra210_adsp_compr_set_params,
	.get_params = tegra210_adsp_compr_get_params,
	.trigger = tegra210_adsp_compr_trigger,
	.pointer = tegra210_adsp_compr_pointer,
	.copy = tegra210_adsp_compr_copy,
	.get_caps = tegra210_adsp_compr_get_caps,
	.get_codec_caps = tegra210_adsp_compr_codec_caps,
};

/* PCM APIs */
static int tegra210_adsp_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra210_adsp *adsp =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct tegra210_adsp_pcm_rtd *prtd;
	uint32_t fe_reg = rtd->codec_dai->id;
	uint32_t source;
	int i, ret = 0;

	dev_vdbg(adsp->dev, "%s", __func__);

	if (!adsp->pcm_path[fe_reg][substream->stream].fe_reg ||
		!adsp->pcm_path[fe_reg][substream->stream].be_reg) {
		dev_err(adsp->dev, "Broken Path%d - FE not linked to BE", fe_reg);
		return -EPIPE;
	}

	prtd = devm_kzalloc(adsp->dev, sizeof(struct tegra210_adsp_pcm_rtd),
		GFP_KERNEL);
	if (!prtd) {
		dev_err(adsp->dev, "Failed to allocate adsp rtd.");
		return -ENOMEM;
	}

	/* Find out the APM connected with ADSP-FE DAI */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = APM_IN_START; i <= APM_IN_END; i++) {
			struct tegra210_adsp_app *app = &adsp->apps[i];

			source = tegra210_adsp_get_source(adsp, app->reg);
			if (source == fe_reg) {
				prtd->fe_apm = app;
				break;
			}
		}
	} else {
		source = tegra210_adsp_get_source(adsp, fe_reg);
		if (IS_APM_OUT(source)) {
			uint32_t apm_in_reg =
				APM_IN_START + (source - APM_OUT_START);
			adsp->apps[apm_in_reg].msg_handler =
				tegra210_adsp_pcm_msg_handler;
			adsp->apps[apm_in_reg].private_data = prtd;
			prtd->fe_apm = &adsp->apps[source];
		}
	}

	if (!prtd->fe_apm) {
		dev_err(adsp->dev, "No FE APM found\n");
		devm_kfree(adsp->dev, prtd);
		return -ENODEV;
	}
	prtd->fe_apm->msg_handler = tegra210_adsp_pcm_msg_handler;
	prtd->fe_apm->private_data = prtd;
	prtd->fe_apm->fe = 1;

	/* Set HW params now that initialization is complete */
	snd_soc_set_runtime_hwparams(substream, &adsp_pcm_hardware);

	/* Ensure period size is multiple of 4 */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 0x4);
	if (ret) {
		dev_err(adsp->dev,
			"failed to set period_size constraint %d\n", ret);
		return ret;
	}

	/* Ensure buffer size is multiple of 4 */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 0x4);
	if (ret) {
		dev_err(adsp->dev,
			"failed to set buffer_size constraint %d\n", ret);
		return ret;
	}
	substream->runtime->private_data = prtd;
	prtd->substream = substream;
	prtd->dev = adsp->dev;

	return 0;
}

static int tegra210_adsp_pcm_close(struct snd_pcm_substream *substream)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	unsigned long flags;

	dev_vdbg(prtd->dev, "%s", __func__);

	tegra_isomgr_adma_setbw(substream, false);

	if (prtd) {
		tegra210_adsp_send_reset_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND |
			TEGRA210_ADSP_MSG_FLAG_NEED_ACK);

		spin_lock_irqsave(&prtd->fe_apm->lock, flags);

		/* Reset msg handler to disable msg processing */
		prtd->fe_apm->msg_handler =
			tegra210_adsp_app_default_msg_handler;

		prtd->fe_apm->private_data = NULL;

		spin_unlock_irqrestore(&prtd->fe_apm->lock, flags);

		prtd->fe_apm->fe = 1;
		substream->runtime->private_data = NULL;
		devm_kfree(prtd->dev, prtd);
	}

	return 0;
}

static int tegra210_adsp_pcm_prepare(struct snd_pcm_substream *substream)
{
	tegra_isomgr_adma_setbw(substream, true);

	return 0;
}

static int tegra210_adsp_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s rate %d chan %d bps %d"
		"period size %d buffer size %d",
		 __func__, params_rate(params), params_channels(params),
		 snd_pcm_format_width(params_format(params)),
		 params_period_size(params),
		 params_buffer_bytes(params));

	ret = tegra210_adsp_send_io_buffer_msg(prtd->fe_apm, buf->addr,
					params_buffer_bytes(params),
					TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0)
		return ret;

	ret = tegra210_adsp_send_period_size_msg(prtd->fe_apm,
			params_buffer_bytes(params)/params_periods(params),
			TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0)
		return ret;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int tegra210_adsp_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}


static int tegra210_adsp_pcm_trigger(struct snd_pcm_substream *substream,
				     int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s : state %d", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_active,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state");
			return ret;
		}

		if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
			(IS_MMAP_ACCESS(runtime->access))) {
			prtd->prev_appl_ptr = runtime->control->appl_ptr;
			tegra210_adsp_pcm_ack(substream);
		}
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state");
			return ret;
		}

		ret = tegra210_adsp_send_flush_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to reset");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_DRAIN:
		/* EOS message is sent so that ADSP sends */
		/* notification for last consumed buffer  */
		ret = tegra210_adsp_send_eos_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state drain");
			return ret;
		}
		break;
	default:
		dev_err(prtd->dev, "Unsupported state.");
		return -EINVAL;
	}

	return 0;
}


static snd_pcm_uframes_t tegra210_adsp_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	struct tegra210_adsp_app *app = prtd->fe_apm;
	size_t bytes, pos;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		bytes = app->apm->nvfx_shared_state.output[0].bytes;
	else
		bytes = app->apm->nvfx_shared_state.input[0].bytes;

	pos = bytes % frames_to_bytes(substream->runtime,
		substream->runtime->buffer_size);

	/* TODO : If SRC in path do size conversion */

	dev_vdbg(prtd->dev, "%s bytes %zu position %zu", __func__, bytes, pos);
	return bytes_to_frames(substream->runtime, pos);
}

static struct snd_pcm_ops tegra210_adsp_pcm_ops = {
	.open		= tegra210_adsp_pcm_open,
	.close		= tegra210_adsp_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= tegra210_adsp_pcm_hw_params,
	.hw_free	= tegra210_adsp_pcm_hw_free,
	.prepare	= tegra210_adsp_pcm_prepare,
	.trigger	= tegra210_adsp_pcm_trigger,
	.pointer	= tegra210_adsp_pcm_pointer,
	.ack		= tegra210_adsp_pcm_ack,
};

static int tegra210_adsp_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
#if ENABLE_ADSP
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		struct snd_pcm_substream *substream =
			pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;

		ret = tegra210_adsp_preallocate_dma_buffer(card->dev,
					adsp_pcm_hardware.buffer_bytes_max,
					&substream->dma_buffer);

		if (ret)
			return ret;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		struct snd_pcm_substream *substream =
			pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;

		ret = tegra210_adsp_preallocate_dma_buffer(card->dev,
					adsp_pcm_hardware.buffer_bytes_max,
					&substream->dma_buffer);
		if (ret)
			goto err;
	}

	return 0;

err:
	tegra210_adsp_deallocate_dma_buffer(
		&pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream->dma_buffer);
	return ret;
#else
	return 0;
#endif
}

static void tegra210_adsp_pcm_free(struct snd_pcm *pcm)
{
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		int stream = SNDRV_PCM_STREAM_PLAYBACK;

		tegra210_adsp_deallocate_dma_buffer(
			&pcm->streams[stream].substream->dma_buffer);
	}
	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		int stream = SNDRV_PCM_STREAM_CAPTURE;

		tegra210_adsp_deallocate_dma_buffer(
			&pcm->streams[stream].substream->dma_buffer);
	}
}

static int tegra_adsp_get_connected_fe(struct tegra210_adsp *adsp,
				uint32_t be_reg,
				int stream)
{
	int val;

	for (val = 1; val < (ADSP_FE_COUNT); val++) {
		if (adsp->pcm_path[val][stream].be_reg == be_reg)
			break;
	}
	return val;
}

static int tegra210_adsp_fe_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&tegra210_adsp_rate_constraints);
}

static int tegra210_adsp_fe_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{

	struct tegra210_adsp *adsp = snd_soc_dai_get_drvdata(dai);
	uint32_t fe_reg = dai->id;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].rate
					= params_rate(params);
		adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].channels
					= params_channels(params);
		adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].format
					= params_format(params);
	} else {
		adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].rate
					= params_rate(params);
		adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].channels
					= params_channels(params);
		adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].format
					= params_format(params);
	}

	return 0;
}

static int tegra210_adsp_null_sink_hw_params(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event) {

	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(w->dapm);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *app;
	nvfx_adma_init_params_t adma_params;
	struct tegra210_adsp_pcm_rtd *prtd = NULL;
	struct snd_pcm_runtime *runtime = NULL;
	uint32_t channels = 2, format = SNDRV_PCM_FORMAT_S16_LE, rate = 48000;
	int32_t source, be_reg = w->reg, apm_in_reg;
	int  ret, num_params;
	apm_msg_t apm_msg;
	uint32_t max_bytes = adsp_pcm_hardware.buffer_bytes_max;

	if (!adsp->adsp_started)
		return 0;

	memset(&adma_params, 0, sizeof(adma_params));
	adma_params.mode = ADMA_MODE_CONTINUOUS;
	adma_params.periods = 4;
	adma_params.adma_ch_page = adsp->adma_ch_page;

	app = &adsp->apps[be_reg];
	source = tegra210_adsp_get_source(adsp, app->reg);
	app = &adsp->apps[source];
	if (!IS_APM_OUT(app->reg))
		return 0;
	apm_in_reg = APM_IN_START + (source - APM_OUT_START);

	if (adsp->apps[apm_in_reg].msg_handler
			== tegra210_adsp_pcm_msg_handler) {
		prtd = adsp->apps[apm_in_reg].private_data;
		runtime = prtd->substream->runtime;
		max_bytes = frames_to_bytes(runtime, runtime->buffer_size);
	}

	source = tegra210_adsp_get_source(adsp, app->reg);
	app = &adsp->apps[source];
	if (!IS_ADMA(app->reg))
		return 0;

	ret = tegra_adsp_get_connected_fe(adsp, be_reg,
				SNDRV_PCM_STREAM_PLAYBACK);
	if (ret != ADSP_FE_COUNT) {
		format = adsp->pcm_path
			[ret][SNDRV_PCM_STREAM_PLAYBACK].format;
		channels = adsp->pcm_path
			[ret][SNDRV_PCM_STREAM_PLAYBACK].channels;
		rate = adsp->pcm_path
			[ret][SNDRV_PCM_STREAM_PLAYBACK].rate;
	}

	switch (format) {
	case SNDRV_PCM_FORMAT_S8:
		adma_params.intr_dur =
			1000 * max_bytes / (channels * rate * 1);
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		adma_params.intr_dur =
			1000 * max_bytes / (channels * rate * 2);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		adma_params.intr_dur =
			1000 * max_bytes / (channels * rate * 3);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adma_params.intr_dur =
			1000 * max_bytes / (channels * rate * 4);
		break;
	default:
		dev_err(adsp->dev, "Wrong format!\n");
		return -EINVAL;
	}

	adma_params.adma_channel = app->adma_chan;
	adma_params.direction = ADMA_MEMORY_TO_AHUB;
	adma_params.event.pvoid = app->apm->output_event.pvoid;

	apm_msg.msgq_msg.size =
		MSGQ_MSG_WSIZE(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.size =
		sizeof(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.method =
		nvfx_apm_method_fx_set_param;
	apm_msg.msg.fx_set_param_params.plugin.pvoid =
		app->plugin->plugin.pvoid;
	num_params = 3;
	apm_msg.msg.fx_set_param_params.params[0] =
		(sizeof(nvfx_call_params_t) +
		num_params * sizeof(int32_t));

	apm_msg.msg.fx_set_param_params.params[1] =
				nvfx_adma_set_null_sink_mode;
	adma_params.ahub_channel = 0;

	if (event == SND_SOC_DAPM_POST_PMD) {
		/* set adma in playback mode */
		apm_msg.msg.fx_set_param_params.params[2] = 0;
	} else
		/* set adma in drain mode */
		apm_msg.msg.fx_set_param_params.params[2] = 1;

	ret = pm_runtime_get_sync(adsp->dev);
	if (ret < 0) {
		dev_err(adsp->dev, "%s pm_runtime_get_sync err 0x%x\n",
		__func__, ret);
		return ret;
	}

	ret = tegra210_adsp_send_msg(app, &apm_msg,
			TEGRA210_ADSP_MSG_FLAG_SEND);

	if (ret < 0) {
		dev_vdbg(adsp->dev, "apm null-sink msg failed.%d\n", ret);
		pm_runtime_put(adsp->dev);
		return ret;
	}

	if (event == SND_SOC_DAPM_POST_PMD) {
		pm_runtime_put(adsp->dev);
		return 0;
	}

	ret = tegra210_adsp_adma_params_msg(app, &adma_params,
		TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0) {
		dev_vdbg(adsp->dev, "ADMA param msg failed.%d\n", ret);
		pm_runtime_put(adsp->dev);
		return ret;
	}
	pm_runtime_put(adsp->dev);

	return 0;
}

/* ADSP-ADMAIF codec driver HW-params. Used for configuring ADMA */
static int tegra210_adsp_admaif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tegra210_adsp *adsp = snd_soc_dai_get_drvdata(dai);
	struct tegra210_adsp_pcm_rtd *prtd = NULL;
	struct snd_pcm_runtime *runtime = NULL;
	struct tegra210_adsp_app *app;
	nvfx_adma_init_params_t adma_params;
	uint32_t be_reg = dai->id;
	uint32_t admaif_id = be_reg - ADSP_ADMAIF_START + 1;
	uint32_t source, apm_in_reg;
	int i, ret;

	dev_vdbg(adsp->dev, "%s : stream %d admaif %d\n",
		__func__, substream->stream, admaif_id);
	if (!adsp->adsp_started)
		return -EINVAL;

	memset(&adma_params, 0, sizeof(adma_params));

	adma_params.mode = ADMA_MODE_CONTINUOUS;
	adma_params.ahub_channel = admaif_id;
	adma_params.periods = 2; /* We need ping-pong buffers for ADMA */
	adma_params.adma_ch_page = adsp->adma_ch_page;

	/* Set DMA params connected with ADSP-BE */
	/* As a COCEC DAI, CAPTURE is transmit */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		app = &adsp->apps[be_reg];
		source = tegra210_adsp_get_source(adsp, app->reg);

		app = &adsp->apps[source];
		if (!IS_APM_OUT(app->reg))
			return 0;

		apm_in_reg = APM_IN_START + (source - APM_OUT_START);

		/* not needed for IO-IO and compr */
		if (adsp->apps[apm_in_reg].msg_handler
				== tegra210_adsp_pcm_msg_handler) {
			prtd = adsp->apps[apm_in_reg].private_data;
			runtime = prtd->substream->runtime;
			if ((IS_MMAP_ACCESS(runtime->access)))
				adma_params.periods = 4;
		}

		source = tegra210_adsp_get_source(adsp, app->reg);
		app = &adsp->apps[source];
		if (!IS_ADMA(app->reg))
			return 0;

		adma_params.adma_channel = app->adma_chan;
		adma_params.direction = ADMA_MEMORY_TO_AHUB;
		adma_params.event.pvoid = app->apm->output_event.pvoid;

		ret = tegra210_adsp_adma_params_msg(app, &adma_params,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(adsp->dev, "ADMA params msg failed. %d.", ret);
			return ret;
		}
	} else {
		for (i = ADMA_START; i <= ADMA_END; i++) {
			app = &adsp->apps[i];
			source = tegra210_adsp_get_source(adsp, app->reg);
			if (!IS_APM_IN(source))
				continue;
			app = &adsp->apps[source];

			/* not needed for IO-IO and compr */
			if (app->msg_handler ==
					tegra210_adsp_pcm_msg_handler) {
				prtd = adsp->apps[source].private_data;
				runtime = prtd->substream->runtime;
			}
			source = tegra210_adsp_get_source(adsp, app->reg);
			if (source != be_reg)
				continue;

			if (runtime && (IS_MMAP_ACCESS(runtime->access)))
				adma_params.periods = 4;

			app = &adsp->apps[i];

			adma_params.adma_channel = app->adma_chan;
			adma_params.direction = ADMA_AHUB_TO_MEMORY;
			adma_params.event.pvoid = app->apm->input_event.pvoid;

			ret = tegra210_adsp_adma_params_msg(app,
					&adma_params,
					TEGRA210_ADSP_MSG_FLAG_SEND);
			if (ret < 0) {
				dev_err(adsp->dev, "ADMA params msg failed");
				return ret;
			}
		}
	}
	return 0;
}

/* ADSP-EAVB codec driver HW-params. Used for configuring EAVB ADMA */
static int tegra210_adsp_eavb_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tegra210_adsp *adsp = snd_soc_dai_get_drvdata(dai);
	struct tegra210_adsp_app *app;
	nvfx_eavbdma_init_params_t eavbdma_params;
	int ret;

	if (!adsp->adsp_started)
		return -EINVAL;

	/* As a COCEC DAI, CAPTURE is transmit */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		app = tegra210_adsp_get_plugin(adsp, EAVB_TX_PLUGIN);

		if (!app)
			return -EINVAL;

		eavbdma_params.direction = EAVB_TX_DMA;
		eavbdma_params.event.pvoid = app->apm->output_event.pvoid;
	} else {
		app = tegra210_adsp_get_plugin(adsp, EAVB_RX_PLUGIN);

		if (!app)
			return -EINVAL;

		eavbdma_params.direction = EAVB_RX_DMA;
		eavbdma_params.event.pvoid = app->apm->input_event.pvoid;
	}

	ret = tegra210_adsp_eavbdma_params_msg(app, &eavbdma_params,
			TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0) {
		dev_err(adsp->dev, "EAVB DMA params msg failed. %d.", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM
static int tegra210_adsp_runtime_suspend(struct device *dev)
{
	struct tegra210_adsp *adsp = dev_get_drvdata(dev);
	int ret = 0, i;

	dev_dbg(adsp->dev, "%s\n", __func__);

	mutex_lock(&adsp->mutex);
	if (!adsp->init_done || !adsp->adsp_started)
		goto exit;

	/* Check for msgq empty before suspend */
	for (i = 0; i < TEGRA210_ADSP_VIRT_REG_MAX; i++) {
		struct tegra210_adsp_app *app = &adsp->apps[i];
		if (app->plugin && IS_APM_IN(app->reg)) {
			msgq_t *msgq = &app->apm->msgq_recv.msgq;
			if (msgq->read_index == msgq->write_index)
				continue;
			pr_err("%s: app %d, msgq not empty rd %d wr %d\n",
				__func__, app->reg, msgq->read_index,
				msgq->write_index);
		}
	}

	ret = nvadsp_os_suspend();
	if (ret)
		dev_err(adsp->dev, "Failed to suspend ADSP OS");

	adsp->adsp_started = 0;

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		if (!adsp->soc_data->is_soc_t210)
			clk_disable_unprepare(adsp->apb2ape_clk);
		clk_disable_unprepare(adsp->ahub_clk);
		clk_disable_unprepare(adsp->ape_clk);
	}

exit:
	mutex_unlock(&adsp->mutex);
	return ret;
}

static int tegra210_adsp_runtime_resume(struct device *dev)
{
	struct tegra210_adsp *adsp = dev_get_drvdata(dev);
	int ret = 0;

	dev_dbg(adsp->dev, "%s\n", __func__);

	mutex_lock(&adsp->mutex);
	if (!adsp->init_done || adsp->adsp_started)
		goto exit;

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		ret = clk_prepare_enable(adsp->ahub_clk);
		if (ret < 0) {
			dev_err(dev, "ahub clk_enable failed: %d\n", ret);
			goto exit;
		}

		ret = clk_prepare_enable(adsp->ape_clk);
		if (ret < 0) {
			dev_err(dev, "ape clk_enable failed: %d\n", ret);
			goto exit;
		}

		if (!adsp->soc_data->is_soc_t210) {
			ret = clk_prepare_enable(adsp->apb2ape_clk);
			if (ret < 0) {
				dev_err(dev, "apb2ape clk_enable failed: %d\n"
					, ret);
				goto exit;
			}
		}
	}

	ret = nvadsp_os_start();
	if (ret) {
		dev_err(adsp->dev, "Failed to start ADSP OS ret 0x%x", ret);
		adsp->adsp_started = 0;
		goto exit;
	}

	adsp->adsp_started = 1;
exit:
	mutex_unlock(&adsp->mutex);
	return ret;
}
#endif

/* ADSP platform driver read/write call-back */
static int tegra210_adsp_read(struct snd_soc_component *component,
		unsigned int reg, unsigned int *val)
{
	struct snd_soc_platform *platform =
				snd_soc_component_to_platform(component);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);

	dev_vdbg(adsp->dev, "%s [0x%x] -> 0x%x\n", __func__, reg,
		tegra210_adsp_reg_read(adsp, reg));

	*val = tegra210_adsp_reg_read(adsp, reg);
	return 0;
}

static int tegra210_adsp_write(struct snd_soc_component *component,
		unsigned int reg,
		unsigned int val)
{
	struct snd_soc_platform *platform =
				snd_soc_component_to_platform(component);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);

	dev_vdbg(adsp->dev, "%s [0x%x] -> 0x%x\n", __func__, reg, val);

	tegra210_adsp_reg_write(adsp, reg, val);
	return 0;
}

/* DAPM ENUM MUX get/put callbacks */
static int tegra210_adsp_mux_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(dapm);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	uint32_t val = tegra210_adsp_reg_read(adsp, e->reg);

	ucontrol->value.integer.value[0] =
		(val & TEGRA210_ADSP_WIDGET_SOURCE_MASK) >> e->shift_l;
	return 0;
}

static int tegra210_adsp_mux_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(dapm);
	uint32_t val = ucontrol->value.enumerated.item[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *app;
	uint32_t cur_val = 0;
	int ret = 0;

	if (!adsp->init_done)
		return -ENODEV;

	if (e->reg >= TEGRA210_ADSP_VIRT_REG_MAX)
		return -EINVAL;

	ret = pm_runtime_get_sync(adsp->dev);
	if (ret < 0) {
		dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
			__func__, ret);
		return ret;
	}

	/* Init or de-init app based on connection */
	if (IS_ADSP_APP(e->reg)) {
		app = &adsp->apps[e->reg];
		cur_val = tegra210_adsp_get_source(adsp, e->reg);
		if (cur_val != val) {
			if (app->connect) {
				/* remove existing connections if any */
				tegra210_adsp_remove_connection(adsp, app);
			}
			app->connect = 0;
		}

		if (val == TEGRA210_ADSP_NONE) {
			tegra210_adsp_app_deinit(adsp, app);
		} else {
			ret = tegra210_adsp_app_init(adsp, app);
			if (ret < 0) {
				dev_err(adsp->dev, "Failed to init app %s(%s)",
					app->desc->name, app->desc->fw_name);
				goto err_put;
			}
		}
	}
	tegra210_adsp_reg_update_bits(adsp, e->reg,
		TEGRA210_ADSP_WIDGET_SOURCE_MASK, val << e->shift_l);
	tegra210_adsp_update_connection(adsp);

	snd_soc_dapm_mux_update_power(dapm, kcontrol, val, e, NULL);

err_put:
	pm_runtime_put(adsp->dev);
	return ret ? ret : 1;
}

/* ALSA control get/put call-back implementation */
static int tegra210_adsp_init_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.enumerated.item[0] = adsp->init_done;
	return 0;
}

static int tegra210_adsp_init_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);
	int init = ucontrol->value.enumerated.item[0];
	int ret = 0;

	if (init == adsp->init_done)
		return 0;

	if (init) {
		ret = tegra210_adsp_init(adsp);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to init ADSP.");
			return ret;
		}
	} else {
		tegra210_adsp_deinit(adsp);
	}

	return 1;
}

/*
 * ADSP FE DAPM Widget Event
 * Widget event allows realizing ADSP FE switch use cases
 *
 * Start Playback trigger calls handled by pcm_trigger()
 *
 * Path Connect when ADSP FE is in triggered state
 * (Sequence: Start trigger -> Path disconnect -> Path connect)
 * handled by fe_widget_event() in SND_SOC_DAPM_PRE_PMU event handling
 *
 * Stop Playback trigger calls and Path disconnect during playback
 * handled by fe_widget_event() in SND_SOC_DAPM_POST_PMD event handling
 */
static int tegra210_adsp_fe_widget_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(w->dapm);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *apm;
	struct device *dev = adsp->dev;
	struct device_node *node = dev->of_node;
	int i;
	int ret = 0;
	uint32_t source;
	struct snd_pcm_runtime *runtime;
	struct tegra210_adsp_pcm_rtd *prtd = NULL;

	/* Handle only HV environment ADSP FE events */
	if (!of_device_is_compatible(node, "nvidia,tegra210-adsp-audio-hv"))
		return 0;

	if (!IS_ADSP_FE(w->reg))
		return 0;

	/* Find out APM connected to ADSP-FE */
	for (i = APM_IN_START; i <= APM_IN_END; i++) {
		apm = &adsp->apps[i];
		source = tegra210_adsp_get_source(adsp, apm->reg);
		if (source == w->reg)
			break;
	}

	if (!IS_APM_IN(i)) {
		dev_err(adsp->dev, "%s ADSP-FE %d not connected to any APM-IN\n",
				__func__, w->reg);
		return -1;
	}

	ret = pm_runtime_get_sync(adsp->dev);
	if (ret < 0) {
		dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
				__func__, ret);
		goto err_put;
	}

	/*
	 * PRE_PMU: Send Start playback/capture IVC to Audio Server and
	 * active state message to ADSP
	 *
	 * POST_PMD: Send Stop playback/capture IVC to Audio Server and
	 * inactive state message to ADSP. Flush messages will be sent to
	 * ADSP during pcm_close().
	 */
	if (event == SND_SOC_DAPM_PRE_PMU) {
		dev_info(adsp->dev, "connect event on APM %d, ADSP-FE %d\n",
				(i + 1) - APM_IN_START, w->reg);

		/*
		 * For trigger playback, pcm_trigger calls will handle ivc and adsp messages
		 *
		 * When ADSP FE is in triggered state, path disconnect and connect is done,
		 * widget event will send IVC and ADSP messages
		 */
		if (!apm->fe_playback_triggered) {
			ret = 0;
			goto err_put;
		}

		ret = tegra210_adsp_send_state_msg(apm, nvfx_state_active,
			TEGRA210_ADSP_MSG_FLAG_SEND | TEGRA210_ADSP_MSG_FLAG_NEED_ACK);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to set state active");
			goto err_put;
		}

	} else if (event == SND_SOC_DAPM_POST_PMD) {
		unsigned long flags;

		dev_info(adsp->dev, "disconnect event on APM %d, ADSP-FE %d\n",
				(i + 1) - APM_IN_START, w->reg);

		ret = tegra210_adsp_send_state_msg(apm, nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND | TEGRA210_ADSP_MSG_FLAG_NEED_ACK);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to set state inactive.");
			goto err_put;
		}

		spin_lock_irqsave(&apm->lock, flags);
		prtd = apm->private_data;

		if (!prtd) {
			dev_dbg(adsp->dev, "PCM close caused this widget event\n");
			spin_unlock_irqrestore(&apm->lock, flags);
			goto err_put;
		}

		runtime = prtd->substream->runtime;
		runtime->status->state = SNDRV_PCM_STATE_DISCONNECTED;

		wake_up(&runtime->sleep);
		wake_up(&runtime->tsleep);

		spin_unlock_irqrestore(&apm->lock, flags);

	} else {
		pr_err("%s: error. unknown event: %d\n", __func__, event);
		ret = -1;
		goto err_put;
	}
err_put:
	pm_runtime_put(adsp->dev);
	return ret;
}

/* DAPM widget event */
static int tegra210_adsp_widget_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(w->dapm);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *app;
	int ret = 0;

	if (!IS_ADSP_APP(w->reg))
		return 0;

	app = &adsp->apps[w->reg];
	/* For FE apm state change will be handled from trigger call back */
	if (app->fe)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (IS_APM_IN(w->reg)) {
			/* Request higher ADSP clock when starting stream.
			 * Actmon takes care of adjusting frequency later. */
			ret = pm_runtime_get_sync(adsp->dev);
			if (ret < 0) {
				dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
					__func__, ret);
				return ret;
			}
			if (app->min_adsp_clock)
				adsp_update_dfs_min_rate(app->min_adsp_clock * 1000);

			ret = tegra210_adsp_send_state_msg(app, nvfx_state_active,
				TEGRA210_ADSP_MSG_FLAG_SEND);
			if (ret < 0)
				dev_err(adsp->dev, "Failed to set state active.");

			pm_runtime_put(adsp->dev);
		}
	} else {
		if (IS_APM_IN(w->reg)) {
			ret = pm_runtime_get_sync(adsp->dev);
			if (ret < 0) {
				dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
					__func__, ret);
				return ret;
			}

			ret = tegra210_adsp_send_state_msg(app, nvfx_state_inactive,
				TEGRA210_ADSP_MSG_FLAG_SEND);
			if (ret < 0)
				dev_err(adsp->dev, "Failed to set state inactive.");

			ret = tegra210_adsp_send_reset_msg(app,
				(TEGRA210_ADSP_MSG_FLAG_SEND |
				TEGRA210_ADSP_MSG_FLAG_NEED_ACK));
			if (ret < 0)
				dev_err(adsp->dev, "Failed to reset.");
			if (app->min_adsp_clock)
				adsp_update_dfs_min_rate(0);

			pm_runtime_put(adsp->dev);
		}
	}

	return ret;
}
static struct snd_soc_dai_ops tegra210_adsp_fe_dai_ops = {
	.hw_params      = tegra210_adsp_fe_hw_params,
	.startup	= tegra210_adsp_fe_startup,
};

static struct snd_soc_dai_ops tegra210_adsp_admaif_dai_ops = {
	.hw_params	= tegra210_adsp_admaif_hw_params,
};

static struct snd_soc_dai_ops tegra210_adsp_eavb_dai_ops = {
	.hw_params	= tegra210_adsp_eavb_hw_params,
};

#define ADSP_PCM_DAI(id)						\
	{								\
		.name = "ADSP PCM" #id,					\
		.playback = {						\
			.stream_name = "ADSP PCM" #id " Receive",	\
			.channels_min = 1,				\
			.channels_max = 8,				\
			.rates = SNDRV_PCM_RATE_8000_48000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},							\
		.capture = {						\
			.stream_name = "ADSP PCM" #id " Transmit",	\
			.channels_min = 1,				\
			.channels_max = 8,				\
			.rates = SNDRV_PCM_RATE_8000_48000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},							\
	}

#define ADSP_COMPR_DAI(id)						\
	{								\
		.name = "ADSP COMPR" #id,				\
		.compress_new = snd_soc_new_compress,			\
		.playback = {						\
			.stream_name = "ADSP COMPR" #id " Receive",	\
			.channels_min = 1,				\
			.channels_max = 2,				\
			.rates = SNDRV_PCM_RATE_8000_48000,		\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		},							\
	}

#define ADSP_EAVB_DAI()						\
	{								\
		.name = "ADSP EAVB",					\
		.playback = {						\
			.stream_name = "ADSP EAVB Transmit",		\
			.channels_min = 1,				\
			.channels_max = 2,				\
			.rates = SNDRV_PCM_RATE_8000_48000,		\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		},							\
		.capture = {						\
			.stream_name = "ADSP EAVB Receive",		\
			.channels_min = 1,				\
			.channels_max = 2,				\
			.rates = SNDRV_PCM_RATE_8000_48000,		\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		},							\
	}

static struct snd_soc_dai_driver tegra210_adsp_dai[] = {
	ADSP_PCM_DAI(1),
	ADSP_PCM_DAI(2),
	ADSP_PCM_DAI(3),
	ADSP_PCM_DAI(4),
	ADSP_PCM_DAI(5),
	ADSP_PCM_DAI(6),
	ADSP_PCM_DAI(7),
	ADSP_PCM_DAI(8),
	ADSP_PCM_DAI(9),
	ADSP_PCM_DAI(10),
	ADSP_PCM_DAI(11),
	ADSP_PCM_DAI(12),
	ADSP_PCM_DAI(13),
	ADSP_PCM_DAI(14),
	ADSP_PCM_DAI(15),
	ADSP_COMPR_DAI(1),
	ADSP_COMPR_DAI(2),
	ADSP_EAVB_DAI(),
};

#define ADSP_FE_CODEC_DAI(idx)					\
	{							\
		.name = "ADSP-FE" #idx,				\
		.id = ADSP_FE_START + (idx - 1),			\
		.playback = {					\
			.stream_name = "ADSP-FE" #idx " Receive",\
			.channels_min = 1,			\
			.channels_max = 8,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = "ADSP-FE" #idx " Transmit",\
			.channels_min = 1,			\
			.channels_max = 8,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.ops = &tegra210_adsp_fe_dai_ops,		\
	}

#define ADSP_ADMAIF_CODEC_DAI(idx)				\
	{							\
		.name = "ADSP-ADMAIF" #idx,			\
		.id = ADSP_ADMAIF_START + (idx - 1),		\
		.playback = {					\
		.stream_name = "ADSP-ADMAIF" #idx " Receive",	\
			.channels_min = 1,			\
			.channels_max = 8,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.capture = {					\
			.stream_name = "ADSP-ADMAIF" #idx " Transmit",\
			.channels_min = 1,			\
			.channels_max = 8,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = &tegra210_adsp_admaif_dai_ops,		\
	}

#define ADSP_EAVB_CODEC_DAI()					\
	{							\
		.name = "ADSP-EAVB",				\
		.id = ADSP_EAVB_START,				\
		.playback = {					\
		.stream_name = "ADSP-EAVB Receive",		\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.capture = {					\
			.stream_name = "ADSP-EAVB Transmit",	\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = &tegra210_adsp_eavb_dai_ops,		\
	}


static struct snd_soc_dai_driver tegra210_adsp_codec_dai[] = {
	ADSP_FE_CODEC_DAI(1),
	ADSP_FE_CODEC_DAI(2),
	ADSP_FE_CODEC_DAI(3),
	ADSP_FE_CODEC_DAI(4),
	ADSP_FE_CODEC_DAI(5),
	ADSP_FE_CODEC_DAI(6),
	ADSP_FE_CODEC_DAI(7),
	ADSP_FE_CODEC_DAI(8),
	ADSP_FE_CODEC_DAI(9),
	ADSP_FE_CODEC_DAI(10),
	ADSP_FE_CODEC_DAI(11),
	ADSP_FE_CODEC_DAI(12),
	ADSP_FE_CODEC_DAI(13),
	ADSP_FE_CODEC_DAI(14),
	ADSP_FE_CODEC_DAI(15),
	ADSP_EAVB_CODEC_DAI(),
	ADSP_ADMAIF_CODEC_DAI(1),
	ADSP_ADMAIF_CODEC_DAI(2),
	ADSP_ADMAIF_CODEC_DAI(3),
	ADSP_ADMAIF_CODEC_DAI(4),
	ADSP_ADMAIF_CODEC_DAI(5),
	ADSP_ADMAIF_CODEC_DAI(6),
	ADSP_ADMAIF_CODEC_DAI(7),
	ADSP_ADMAIF_CODEC_DAI(8),
	ADSP_ADMAIF_CODEC_DAI(9),
	ADSP_ADMAIF_CODEC_DAI(10),
	ADSP_ADMAIF_CODEC_DAI(11),
	ADSP_ADMAIF_CODEC_DAI(12),
	ADSP_ADMAIF_CODEC_DAI(13),
	ADSP_ADMAIF_CODEC_DAI(14),
	ADSP_ADMAIF_CODEC_DAI(15),
	ADSP_ADMAIF_CODEC_DAI(16),
	ADSP_ADMAIF_CODEC_DAI(17),
	ADSP_ADMAIF_CODEC_DAI(18),
	ADSP_ADMAIF_CODEC_DAI(19),
	ADSP_ADMAIF_CODEC_DAI(20),
};

/* This array is linked with tegra210_adsp_virt_regs enum defines. Any thing
   changed in enum define should be also reflected here and vice-versa */
static const char *tegra210_adsp_mux_texts[] = {
	"None",
	"ADSP-FE1",
	"ADSP-FE2",
	"ADSP-FE3",
	"ADSP-FE4",
	"ADSP-FE5",
	"ADSP-FE6",
	"ADSP-FE7",
	"ADSP-FE8",
	"ADSP-FE9",
	"ADSP-FE10",
	"ADSP-FE11",
	"ADSP-FE12",
	"ADSP-FE13",
	"ADSP-FE14",
	"ADSP-FE15",
	"ADSP-EAVB",
	"ADSP-ADMAIF1",
	"ADSP-ADMAIF2",
	"ADSP-ADMAIF3",
	"ADSP-ADMAIF4",
	"ADSP-ADMAIF5",
	"ADSP-ADMAIF6",
	"ADSP-ADMAIF7",
	"ADSP-ADMAIF8",
	"ADSP-ADMAIF9",
	"ADSP-ADMAIF10",
	"ADSP-ADMAIF11",
	"ADSP-ADMAIF12",
	"ADSP-ADMAIF13",
	"ADSP-ADMAIF14",
	"ADSP-ADMAIF15",
	"ADSP-ADMAIF16",
	"ADSP-ADMAIF17",
	"ADSP-ADMAIF18",
	"ADSP-ADMAIF19",
	"ADSP-ADMAIF20",
	"NULL-SINK1",
	"NULL-SINK2",
	"NULL-SINK3",
	"NULL-SINK4",
	"NULL-SINK5",
	"NULL-SINK6",
	"NULL-SINK7",
	"NULL-SINK8",
	"NULL-SINK9",
	"NULL-SINK10",
	"NULL-SINK11",
	"NULL-SINK12",
	"NULL-SINK13",
	"NULL-SINK14",
	"NULL-SINK15",
	"APM-IN1",
	"APM-IN2",
	"APM-IN3",
	"APM-IN4",
	"APM-IN5",
	"APM-IN6",
	"APM-IN7",
	"APM-IN8",
	"APM-IN9",
	"APM-IN10",
	"APM-IN11",
	"APM-IN12",
	"APM-IN13",
	"APM-IN14",
	"APM-IN15",
	"APM-OUT1",
	"APM-OUT2",
	"APM-OUT3",
	"APM-OUT4",
	"APM-OUT5",
	"APM-OUT6",
	"APM-OUT7",
	"APM-OUT8",
	"APM-OUT9",
	"APM-OUT10",
	"APM-OUT11",
	"APM-OUT12",
	"APM-OUT13",
	"APM-OUT14",
	"APM-OUT15",
	"ADMA1",
	"ADMA2",
	"ADMA3",
	"ADMA4",
	"ADMA5",
	"ADMA6",
	"ADMA7",
	"ADMA8",
	"ADMA9",
	"ADMA10",
	"ADMA11",
	"ADMA12",
	"ADMA13",
	"ADMA14",
	"ADMA15",
	"ADMA1-TX",
	"ADMA2-TX",
	"ADMA3-TX",
	"ADMA4-TX",
	"ADMA5-TX",
	"ADMA6-TX",
	"ADMA7-TX",
	"ADMA8-TX",
	"ADMA9-TX",
	"ADMA10-TX",
	"ADMA11-TX",
	"ADMA12-TX",
	"ADMA13-TX",
	"ADMA14-TX",
	"ADMA15-TX",
	"PLUGIN1-PLACE-HOLDER",
	"PLUGIN2-PLACE-HOLDER",
	"PLUGIN3-PLACE-HOLDER",
	"PLUGIN4-PLACE-HOLDER",
	"PLUGIN5-PLACE-HOLDER",
	"PLUGIN6-PLACE-HOLDER",
	"PLUGIN7-PLACE-HOLDER",
	"PLUGIN8-PLACE-HOLDER",
	"PLUGIN9-PLACE-HOLDER",
	"PLUGIN10-PLACE-HOLDER",
	"PLUGIN11-PLACE-HOLDER",
	"PLUGIN12-PLACE-HOLDER",
	"PLUGIN13-PLACE-HOLDER",
	"PLUGIN14-PLACE-HOLDER",
	"PLUGIN15-PLACE-HOLDER",
	"PLUGIN16-PLACE-HOLDER",
	"PLUGIN17-PLACE-HOLDER",
	"PLUGIN18-PLACE-HOLDER",
	"PLUGIN19-PLACE-HOLDER",
	"PLUGIN20-PLACE-HOLDER",
};

#define ADSP_MUX_ENUM_CTRL_DECL(ename, reg)		\
	SOC_ENUM_SINGLE_DECL(ename##_enum, reg,		\
		TEGRA210_ADSP_WIDGET_SOURCE_SHIFT,		\
		tegra210_adsp_mux_texts);				\
	static const struct snd_kcontrol_new ename##_ctrl =		\
		SOC_DAPM_ENUM_EXT("ADSP Route", ename##_enum,		\
			tegra210_adsp_mux_get, tegra210_adsp_mux_put)

static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe1, TEGRA210_ADSP_FRONT_END1);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe2, TEGRA210_ADSP_FRONT_END2);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe3, TEGRA210_ADSP_FRONT_END3);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe4, TEGRA210_ADSP_FRONT_END4);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe5, TEGRA210_ADSP_FRONT_END5);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe6, TEGRA210_ADSP_FRONT_END6);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe7, TEGRA210_ADSP_FRONT_END7);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe8, TEGRA210_ADSP_FRONT_END8);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe9, TEGRA210_ADSP_FRONT_END9);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe10, TEGRA210_ADSP_FRONT_END10);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe11, TEGRA210_ADSP_FRONT_END11);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe12, TEGRA210_ADSP_FRONT_END12);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe13, TEGRA210_ADSP_FRONT_END13);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe14, TEGRA210_ADSP_FRONT_END14);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe15, TEGRA210_ADSP_FRONT_END15);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_eavb, TEGRA210_ADSP_EAVB);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif1, TEGRA210_ADSP_ADMAIF1);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif2, TEGRA210_ADSP_ADMAIF2);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif3, TEGRA210_ADSP_ADMAIF3);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif4, TEGRA210_ADSP_ADMAIF4);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif5, TEGRA210_ADSP_ADMAIF5);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif6, TEGRA210_ADSP_ADMAIF6);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif7, TEGRA210_ADSP_ADMAIF7);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif8, TEGRA210_ADSP_ADMAIF8);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif9, TEGRA210_ADSP_ADMAIF9);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif10, TEGRA210_ADSP_ADMAIF10);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif11, TEGRA210_ADSP_ADMAIF11);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif12, TEGRA210_ADSP_ADMAIF12);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif13, TEGRA210_ADSP_ADMAIF13);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif14, TEGRA210_ADSP_ADMAIF14);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif15, TEGRA210_ADSP_ADMAIF15);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif16, TEGRA210_ADSP_ADMAIF16);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif17, TEGRA210_ADSP_ADMAIF17);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif18, TEGRA210_ADSP_ADMAIF18);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif19, TEGRA210_ADSP_ADMAIF19);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif20, TEGRA210_ADSP_ADMAIF20);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink1, TEGRA210_ADSP_NULL_SINK1);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink2, TEGRA210_ADSP_NULL_SINK2);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink3, TEGRA210_ADSP_NULL_SINK3);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink4, TEGRA210_ADSP_NULL_SINK4);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink5, TEGRA210_ADSP_NULL_SINK5);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink6, TEGRA210_ADSP_NULL_SINK6);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink7, TEGRA210_ADSP_NULL_SINK7);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink8, TEGRA210_ADSP_NULL_SINK8);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink9, TEGRA210_ADSP_NULL_SINK9);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink10, TEGRA210_ADSP_NULL_SINK10);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink11, TEGRA210_ADSP_NULL_SINK11);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink12, TEGRA210_ADSP_NULL_SINK12);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink13, TEGRA210_ADSP_NULL_SINK13);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink14, TEGRA210_ADSP_NULL_SINK14);
static ADSP_MUX_ENUM_CTRL_DECL(null_sink15, TEGRA210_ADSP_NULL_SINK15);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in1, TEGRA210_ADSP_APM_IN1);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in2, TEGRA210_ADSP_APM_IN2);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in3, TEGRA210_ADSP_APM_IN3);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in4, TEGRA210_ADSP_APM_IN4);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in5, TEGRA210_ADSP_APM_IN5);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in6, TEGRA210_ADSP_APM_IN6);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in7, TEGRA210_ADSP_APM_IN7);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in8, TEGRA210_ADSP_APM_IN8);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in9, TEGRA210_ADSP_APM_IN9);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in10, TEGRA210_ADSP_APM_IN10);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in11, TEGRA210_ADSP_APM_IN11);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in12, TEGRA210_ADSP_APM_IN12);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in13, TEGRA210_ADSP_APM_IN13);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in14, TEGRA210_ADSP_APM_IN14);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in15, TEGRA210_ADSP_APM_IN15);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out1, TEGRA210_ADSP_APM_OUT1);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out2, TEGRA210_ADSP_APM_OUT2);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out3, TEGRA210_ADSP_APM_OUT3);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out4, TEGRA210_ADSP_APM_OUT4);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out5, TEGRA210_ADSP_APM_OUT5);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out6, TEGRA210_ADSP_APM_OUT6);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out7, TEGRA210_ADSP_APM_OUT7);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out8, TEGRA210_ADSP_APM_OUT8);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out9, TEGRA210_ADSP_APM_OUT9);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out10, TEGRA210_ADSP_APM_OUT10);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out11, TEGRA210_ADSP_APM_OUT11);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out12, TEGRA210_ADSP_APM_OUT12);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out13, TEGRA210_ADSP_APM_OUT13);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out14, TEGRA210_ADSP_APM_OUT14);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out15, TEGRA210_ADSP_APM_OUT15);
static ADSP_MUX_ENUM_CTRL_DECL(adma1, TEGRA210_ADSP_PLUGIN_ADMA1);
static ADSP_MUX_ENUM_CTRL_DECL(adma2, TEGRA210_ADSP_PLUGIN_ADMA2);
static ADSP_MUX_ENUM_CTRL_DECL(adma3, TEGRA210_ADSP_PLUGIN_ADMA3);
static ADSP_MUX_ENUM_CTRL_DECL(adma4, TEGRA210_ADSP_PLUGIN_ADMA4);
static ADSP_MUX_ENUM_CTRL_DECL(adma5, TEGRA210_ADSP_PLUGIN_ADMA5);
static ADSP_MUX_ENUM_CTRL_DECL(adma6, TEGRA210_ADSP_PLUGIN_ADMA6);
static ADSP_MUX_ENUM_CTRL_DECL(adma7, TEGRA210_ADSP_PLUGIN_ADMA7);
static ADSP_MUX_ENUM_CTRL_DECL(adma8, TEGRA210_ADSP_PLUGIN_ADMA8);
static ADSP_MUX_ENUM_CTRL_DECL(adma9, TEGRA210_ADSP_PLUGIN_ADMA9);
static ADSP_MUX_ENUM_CTRL_DECL(adma10, TEGRA210_ADSP_PLUGIN_ADMA10);
static ADSP_MUX_ENUM_CTRL_DECL(adma11, TEGRA210_ADSP_PLUGIN_ADMA11);
static ADSP_MUX_ENUM_CTRL_DECL(adma12, TEGRA210_ADSP_PLUGIN_ADMA12);
static ADSP_MUX_ENUM_CTRL_DECL(adma13, TEGRA210_ADSP_PLUGIN_ADMA13);
static ADSP_MUX_ENUM_CTRL_DECL(adma14, TEGRA210_ADSP_PLUGIN_ADMA14);
static ADSP_MUX_ENUM_CTRL_DECL(adma15, TEGRA210_ADSP_PLUGIN_ADMA15);
static ADSP_MUX_ENUM_CTRL_DECL(adma1_tx, TEGRA210_ADSP_PLUGIN_ADMA1_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma2_tx, TEGRA210_ADSP_PLUGIN_ADMA2_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma3_tx, TEGRA210_ADSP_PLUGIN_ADMA3_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma4_tx, TEGRA210_ADSP_PLUGIN_ADMA4_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma5_tx, TEGRA210_ADSP_PLUGIN_ADMA5_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma6_tx, TEGRA210_ADSP_PLUGIN_ADMA6_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma7_tx, TEGRA210_ADSP_PLUGIN_ADMA7_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma8_tx, TEGRA210_ADSP_PLUGIN_ADMA8_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma9_tx, TEGRA210_ADSP_PLUGIN_ADMA9_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma10_tx, TEGRA210_ADSP_PLUGIN_ADMA10_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma11_tx, TEGRA210_ADSP_PLUGIN_ADMA11_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma12_tx, TEGRA210_ADSP_PLUGIN_ADMA12_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma13_tx, TEGRA210_ADSP_PLUGIN_ADMA13_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma14_tx, TEGRA210_ADSP_PLUGIN_ADMA14_TX);
static ADSP_MUX_ENUM_CTRL_DECL(adma15_tx, TEGRA210_ADSP_PLUGIN_ADMA15_TX);
static ADSP_MUX_ENUM_CTRL_DECL(plugin1, TEGRA210_ADSP_PLUGIN1);
static ADSP_MUX_ENUM_CTRL_DECL(plugin2, TEGRA210_ADSP_PLUGIN2);
static ADSP_MUX_ENUM_CTRL_DECL(plugin3, TEGRA210_ADSP_PLUGIN3);
static ADSP_MUX_ENUM_CTRL_DECL(plugin4, TEGRA210_ADSP_PLUGIN4);
static ADSP_MUX_ENUM_CTRL_DECL(plugin5, TEGRA210_ADSP_PLUGIN5);
static ADSP_MUX_ENUM_CTRL_DECL(plugin6, TEGRA210_ADSP_PLUGIN6);
static ADSP_MUX_ENUM_CTRL_DECL(plugin7, TEGRA210_ADSP_PLUGIN7);
static ADSP_MUX_ENUM_CTRL_DECL(plugin8, TEGRA210_ADSP_PLUGIN8);
static ADSP_MUX_ENUM_CTRL_DECL(plugin9, TEGRA210_ADSP_PLUGIN9);
static ADSP_MUX_ENUM_CTRL_DECL(plugin10, TEGRA210_ADSP_PLUGIN10);
static ADSP_MUX_ENUM_CTRL_DECL(plugin11, TEGRA210_ADSP_PLUGIN11);
static ADSP_MUX_ENUM_CTRL_DECL(plugin12, TEGRA210_ADSP_PLUGIN12);
static ADSP_MUX_ENUM_CTRL_DECL(plugin13, TEGRA210_ADSP_PLUGIN13);
static ADSP_MUX_ENUM_CTRL_DECL(plugin14, TEGRA210_ADSP_PLUGIN14);
static ADSP_MUX_ENUM_CTRL_DECL(plugin15, TEGRA210_ADSP_PLUGIN15);
static ADSP_MUX_ENUM_CTRL_DECL(plugin16, TEGRA210_ADSP_PLUGIN16);
static ADSP_MUX_ENUM_CTRL_DECL(plugin17, TEGRA210_ADSP_PLUGIN17);
static ADSP_MUX_ENUM_CTRL_DECL(plugin18, TEGRA210_ADSP_PLUGIN18);
static ADSP_MUX_ENUM_CTRL_DECL(plugin19, TEGRA210_ADSP_PLUGIN19);
static ADSP_MUX_ENUM_CTRL_DECL(plugin20, TEGRA210_ADSP_PLUGIN20);

#define ADSP_EP_WIDGETS(sname, ename)					\
	SND_SOC_DAPM_AIF_IN(sname " RX", NULL, 0, SND_SOC_NOPM, 0, 0),	\
	SND_SOC_DAPM_AIF_OUT(sname " TX", NULL, 0, SND_SOC_NOPM, 0, 0),	\
	SND_SOC_DAPM_MUX(sname " MUX", SND_SOC_NOPM, 0, 0, &ename##_ctrl)

#define ADSP_FE_WIDGETS(sname, ename, reg)					\
	SND_SOC_DAPM_AIF_IN_E(sname " RX", NULL, 0, reg,		\
		TEGRA210_ADSP_WIDGET_EN_SHIFT, 0, tegra210_adsp_fe_widget_event, \
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),	\
	SND_SOC_DAPM_AIF_OUT_E(sname " TX", NULL, 0, reg,		\
		TEGRA210_ADSP_WIDGET_EN_SHIFT, 0, NULL, \
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),	\
	SND_SOC_DAPM_MUX(sname " MUX", SND_SOC_NOPM, 0, 0, &ename##_ctrl)

#define ADSP_WIDGETS(sname, ename, reg)					\
	SND_SOC_DAPM_AIF_OUT_E(sname " TX", NULL, 0, reg,		\
		TEGRA210_ADSP_WIDGET_EN_SHIFT, 0, tegra210_adsp_widget_event, \
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),		\
	SND_SOC_DAPM_MUX(sname " MUX", SND_SOC_NOPM, 0, 0, &ename##_ctrl)

#define SND_SOC_DAPM_IN(wname, wreg)					\
{	.id = snd_soc_dapm_spk, .name = wname, .kcontrol_news = NULL,	\
	.num_kcontrols = 0, .reg = wreg,				\
	.event = tegra210_adsp_null_sink_hw_params,			\
	.event_flags = SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD }

static struct snd_soc_dapm_widget tegra210_adsp_widgets[] = {
	ADSP_FE_WIDGETS("ADSP-FE1", adsp_fe1, TEGRA210_ADSP_FRONT_END1),
	ADSP_FE_WIDGETS("ADSP-FE2", adsp_fe2, TEGRA210_ADSP_FRONT_END2),
	ADSP_FE_WIDGETS("ADSP-FE3", adsp_fe3, TEGRA210_ADSP_FRONT_END3),
	ADSP_FE_WIDGETS("ADSP-FE4", adsp_fe4, TEGRA210_ADSP_FRONT_END4),
	ADSP_FE_WIDGETS("ADSP-FE5", adsp_fe5, TEGRA210_ADSP_FRONT_END5),
	ADSP_FE_WIDGETS("ADSP-FE6", adsp_fe6, TEGRA210_ADSP_FRONT_END6),
	ADSP_FE_WIDGETS("ADSP-FE7", adsp_fe7, TEGRA210_ADSP_FRONT_END7),
	ADSP_FE_WIDGETS("ADSP-FE8", adsp_fe8, TEGRA210_ADSP_FRONT_END8),
	ADSP_FE_WIDGETS("ADSP-FE9", adsp_fe9, TEGRA210_ADSP_FRONT_END9),
	ADSP_FE_WIDGETS("ADSP-FE10", adsp_fe10, TEGRA210_ADSP_FRONT_END10),
	ADSP_FE_WIDGETS("ADSP-FE11", adsp_fe11, TEGRA210_ADSP_FRONT_END11),
	ADSP_FE_WIDGETS("ADSP-FE12", adsp_fe12, TEGRA210_ADSP_FRONT_END12),
	ADSP_FE_WIDGETS("ADSP-FE13", adsp_fe13, TEGRA210_ADSP_FRONT_END13),
	ADSP_FE_WIDGETS("ADSP-FE14", adsp_fe14, TEGRA210_ADSP_FRONT_END14),
	ADSP_FE_WIDGETS("ADSP-FE15", adsp_fe15, TEGRA210_ADSP_FRONT_END15),
	ADSP_EP_WIDGETS("ADSP-EAVB", adsp_eavb),
	ADSP_EP_WIDGETS("ADSP-ADMAIF1", adsp_admaif1),
	ADSP_EP_WIDGETS("ADSP-ADMAIF2", adsp_admaif2),
	ADSP_EP_WIDGETS("ADSP-ADMAIF3", adsp_admaif3),
	ADSP_EP_WIDGETS("ADSP-ADMAIF4", adsp_admaif4),
	ADSP_EP_WIDGETS("ADSP-ADMAIF5", adsp_admaif5),
	ADSP_EP_WIDGETS("ADSP-ADMAIF6", adsp_admaif6),
	ADSP_EP_WIDGETS("ADSP-ADMAIF7", adsp_admaif7),
	ADSP_EP_WIDGETS("ADSP-ADMAIF8", adsp_admaif8),
	ADSP_EP_WIDGETS("ADSP-ADMAIF9", adsp_admaif9),
	ADSP_EP_WIDGETS("ADSP-ADMAIF10", adsp_admaif10),
	ADSP_EP_WIDGETS("ADSP-ADMAIF11", adsp_admaif11),
	ADSP_EP_WIDGETS("ADSP-ADMAIF12", adsp_admaif12),
	ADSP_EP_WIDGETS("ADSP-ADMAIF13", adsp_admaif13),
	ADSP_EP_WIDGETS("ADSP-ADMAIF14", adsp_admaif14),
	ADSP_EP_WIDGETS("ADSP-ADMAIF15", adsp_admaif15),
	ADSP_EP_WIDGETS("ADSP-ADMAIF16", adsp_admaif16),
	ADSP_EP_WIDGETS("ADSP-ADMAIF17", adsp_admaif17),
	ADSP_EP_WIDGETS("ADSP-ADMAIF18", adsp_admaif18),
	ADSP_EP_WIDGETS("ADSP-ADMAIF19", adsp_admaif19),
	ADSP_EP_WIDGETS("ADSP-ADMAIF20", adsp_admaif20),
	ADSP_EP_WIDGETS("NULL-SINK1", null_sink1),
	ADSP_EP_WIDGETS("NULL-SINK2", null_sink2),
	ADSP_EP_WIDGETS("NULL-SINK3", null_sink3),
	ADSP_EP_WIDGETS("NULL-SINK4", null_sink4),
	ADSP_EP_WIDGETS("NULL-SINK5", null_sink5),
	ADSP_EP_WIDGETS("NULL-SINK6", null_sink6),
	ADSP_EP_WIDGETS("NULL-SINK7", null_sink7),
	ADSP_EP_WIDGETS("NULL-SINK8", null_sink8),
	ADSP_EP_WIDGETS("NULL-SINK9", null_sink9),
	ADSP_EP_WIDGETS("NULL-SINK10", null_sink10),
	ADSP_EP_WIDGETS("NULL-SINK11", null_sink11),
	ADSP_EP_WIDGETS("NULL-SINK12", null_sink12),
	ADSP_EP_WIDGETS("NULL-SINK13", null_sink13),
	ADSP_EP_WIDGETS("NULL-SINK14", null_sink14),
	ADSP_EP_WIDGETS("NULL-SINK15", null_sink15),
	SND_SOC_DAPM_IN("NULL-SINK1", TEGRA210_ADSP_NULL_SINK1),
	SND_SOC_DAPM_IN("NULL-SINK2", TEGRA210_ADSP_NULL_SINK2),
	SND_SOC_DAPM_IN("NULL-SINK3", TEGRA210_ADSP_NULL_SINK3),
	SND_SOC_DAPM_IN("NULL-SINK4", TEGRA210_ADSP_NULL_SINK4),
	SND_SOC_DAPM_IN("NULL-SINK5", TEGRA210_ADSP_NULL_SINK5),
	SND_SOC_DAPM_IN("NULL-SINK6", TEGRA210_ADSP_NULL_SINK6),
	SND_SOC_DAPM_IN("NULL-SINK7", TEGRA210_ADSP_NULL_SINK7),
	SND_SOC_DAPM_IN("NULL-SINK8", TEGRA210_ADSP_NULL_SINK8),
	SND_SOC_DAPM_IN("NULL-SINK9", TEGRA210_ADSP_NULL_SINK9),
	SND_SOC_DAPM_IN("NULL-SINK10", TEGRA210_ADSP_NULL_SINK10),
	SND_SOC_DAPM_IN("NULL-SINK11", TEGRA210_ADSP_NULL_SINK11),
	SND_SOC_DAPM_IN("NULL-SINK12", TEGRA210_ADSP_NULL_SINK12),
	SND_SOC_DAPM_IN("NULL-SINK13", TEGRA210_ADSP_NULL_SINK13),
	SND_SOC_DAPM_IN("NULL-SINK14", TEGRA210_ADSP_NULL_SINK14),
	SND_SOC_DAPM_IN("NULL-SINK15", TEGRA210_ADSP_NULL_SINK15),
	ADSP_WIDGETS("APM-IN1", apm_in1, TEGRA210_ADSP_APM_IN1),
	ADSP_WIDGETS("APM-IN2", apm_in2, TEGRA210_ADSP_APM_IN2),
	ADSP_WIDGETS("APM-IN3", apm_in3, TEGRA210_ADSP_APM_IN3),
	ADSP_WIDGETS("APM-IN4", apm_in4, TEGRA210_ADSP_APM_IN4),
	ADSP_WIDGETS("APM-IN5", apm_in5, TEGRA210_ADSP_APM_IN5),
	ADSP_WIDGETS("APM-IN6", apm_in6, TEGRA210_ADSP_APM_IN6),
	ADSP_WIDGETS("APM-IN7", apm_in7, TEGRA210_ADSP_APM_IN7),
	ADSP_WIDGETS("APM-IN8", apm_in8, TEGRA210_ADSP_APM_IN8),
	ADSP_WIDGETS("APM-IN9", apm_in9, TEGRA210_ADSP_APM_IN9),
	ADSP_WIDGETS("APM-IN10", apm_in10, TEGRA210_ADSP_APM_IN10),
	ADSP_WIDGETS("APM-IN11", apm_in11, TEGRA210_ADSP_APM_IN11),
	ADSP_WIDGETS("APM-IN12", apm_in12, TEGRA210_ADSP_APM_IN12),
	ADSP_WIDGETS("APM-IN13", apm_in13, TEGRA210_ADSP_APM_IN13),
	ADSP_WIDGETS("APM-IN14", apm_in14, TEGRA210_ADSP_APM_IN14),
	ADSP_WIDGETS("APM-IN15", apm_in15, TEGRA210_ADSP_APM_IN15),
	ADSP_WIDGETS("APM-OUT1", apm_out1, TEGRA210_ADSP_APM_OUT1),
	ADSP_WIDGETS("APM-OUT2", apm_out2, TEGRA210_ADSP_APM_OUT2),
	ADSP_WIDGETS("APM-OUT3", apm_out3, TEGRA210_ADSP_APM_OUT3),
	ADSP_WIDGETS("APM-OUT4", apm_out4, TEGRA210_ADSP_APM_OUT4),
	ADSP_WIDGETS("APM-OUT5", apm_out5, TEGRA210_ADSP_APM_OUT5),
	ADSP_WIDGETS("APM-OUT6", apm_out6, TEGRA210_ADSP_APM_OUT6),
	ADSP_WIDGETS("APM-OUT7", apm_out7, TEGRA210_ADSP_APM_OUT7),
	ADSP_WIDGETS("APM-OUT8", apm_out8, TEGRA210_ADSP_APM_OUT8),
	ADSP_WIDGETS("APM-OUT9", apm_out9, TEGRA210_ADSP_APM_OUT9),
	ADSP_WIDGETS("APM-OUT10", apm_out10, TEGRA210_ADSP_APM_OUT10),
	ADSP_WIDGETS("APM-OUT11", apm_out11, TEGRA210_ADSP_APM_OUT11),
	ADSP_WIDGETS("APM-OUT12", apm_out12, TEGRA210_ADSP_APM_OUT12),
	ADSP_WIDGETS("APM-OUT13", apm_out13, TEGRA210_ADSP_APM_OUT13),
	ADSP_WIDGETS("APM-OUT14", apm_out14, TEGRA210_ADSP_APM_OUT14),
	ADSP_WIDGETS("APM-OUT15", apm_out15, TEGRA210_ADSP_APM_OUT15),

	ADSP_WIDGETS("ADMA1", adma1, TEGRA210_ADSP_PLUGIN_ADMA1),
	ADSP_WIDGETS("ADMA2", adma2, TEGRA210_ADSP_PLUGIN_ADMA2),
	ADSP_WIDGETS("ADMA3", adma3, TEGRA210_ADSP_PLUGIN_ADMA3),
	ADSP_WIDGETS("ADMA4", adma4, TEGRA210_ADSP_PLUGIN_ADMA4),
	ADSP_WIDGETS("ADMA5", adma5, TEGRA210_ADSP_PLUGIN_ADMA5),
	ADSP_WIDGETS("ADMA6", adma6, TEGRA210_ADSP_PLUGIN_ADMA6),
	ADSP_WIDGETS("ADMA7", adma7, TEGRA210_ADSP_PLUGIN_ADMA7),
	ADSP_WIDGETS("ADMA8", adma8, TEGRA210_ADSP_PLUGIN_ADMA8),
	ADSP_WIDGETS("ADMA9", adma9, TEGRA210_ADSP_PLUGIN_ADMA9),
	ADSP_WIDGETS("ADMA10", adma10, TEGRA210_ADSP_PLUGIN_ADMA10),
	ADSP_WIDGETS("ADMA11", adma11, TEGRA210_ADSP_PLUGIN_ADMA11),
	ADSP_WIDGETS("ADMA12", adma12, TEGRA210_ADSP_PLUGIN_ADMA12),
	ADSP_WIDGETS("ADMA13", adma13, TEGRA210_ADSP_PLUGIN_ADMA13),
	ADSP_WIDGETS("ADMA14", adma14, TEGRA210_ADSP_PLUGIN_ADMA14),
	ADSP_WIDGETS("ADMA15", adma15, TEGRA210_ADSP_PLUGIN_ADMA15),
	ADSP_WIDGETS("ADMA1-TX", adma1_tx, TEGRA210_ADSP_PLUGIN_ADMA1_TX),
	ADSP_WIDGETS("ADMA2-TX", adma2_tx, TEGRA210_ADSP_PLUGIN_ADMA2_TX),
	ADSP_WIDGETS("ADMA3-TX", adma3_tx, TEGRA210_ADSP_PLUGIN_ADMA3_TX),
	ADSP_WIDGETS("ADMA4-TX", adma4_tx, TEGRA210_ADSP_PLUGIN_ADMA4_TX),
	ADSP_WIDGETS("ADMA5-TX", adma5_tx, TEGRA210_ADSP_PLUGIN_ADMA5_TX),
	ADSP_WIDGETS("ADMA6-TX", adma6_tx, TEGRA210_ADSP_PLUGIN_ADMA6_TX),
	ADSP_WIDGETS("ADMA7-TX", adma7_tx, TEGRA210_ADSP_PLUGIN_ADMA7_TX),
	ADSP_WIDGETS("ADMA8-TX", adma8_tx, TEGRA210_ADSP_PLUGIN_ADMA8_TX),
	ADSP_WIDGETS("ADMA9-TX", adma9_tx, TEGRA210_ADSP_PLUGIN_ADMA9_TX),
	ADSP_WIDGETS("ADMA10-TX", adma10_tx, TEGRA210_ADSP_PLUGIN_ADMA10_TX),
	ADSP_WIDGETS("ADMA11-TX", adma11_tx, TEGRA210_ADSP_PLUGIN_ADMA11_TX),
	ADSP_WIDGETS("ADMA12-TX", adma12_tx, TEGRA210_ADSP_PLUGIN_ADMA12_TX),
	ADSP_WIDGETS("ADMA13-TX", adma13_tx, TEGRA210_ADSP_PLUGIN_ADMA13_TX),
	ADSP_WIDGETS("ADMA14-TX", adma14_tx, TEGRA210_ADSP_PLUGIN_ADMA14_TX),
	ADSP_WIDGETS("ADMA15-TX", adma15_tx, TEGRA210_ADSP_PLUGIN_ADMA15_TX),
	ADSP_WIDGETS("PLUGIN1-PLACE-HOLDER", plugin1, TEGRA210_ADSP_PLUGIN1),
	ADSP_WIDGETS("PLUGIN2-PLACE-HOLDER", plugin2, TEGRA210_ADSP_PLUGIN2),
	ADSP_WIDGETS("PLUGIN3-PLACE-HOLDER", plugin3, TEGRA210_ADSP_PLUGIN3),
	ADSP_WIDGETS("PLUGIN4-PLACE-HOLDER", plugin4, TEGRA210_ADSP_PLUGIN4),
	ADSP_WIDGETS("PLUGIN5-PLACE-HOLDER", plugin5, TEGRA210_ADSP_PLUGIN5),
	ADSP_WIDGETS("PLUGIN6-PLACE-HOLDER", plugin6, TEGRA210_ADSP_PLUGIN6),
	ADSP_WIDGETS("PLUGIN7-PLACE-HOLDER", plugin7, TEGRA210_ADSP_PLUGIN7),
	ADSP_WIDGETS("PLUGIN8-PLACE-HOLDER", plugin8, TEGRA210_ADSP_PLUGIN8),
	ADSP_WIDGETS("PLUGIN9-PLACE-HOLDER", plugin9, TEGRA210_ADSP_PLUGIN9),
	ADSP_WIDGETS("PLUGIN10-PLACE-HOLDER", plugin10, TEGRA210_ADSP_PLUGIN10),
	ADSP_WIDGETS("PLUGIN11-PLACE-HOLDER", plugin11, TEGRA210_ADSP_PLUGIN11),
	ADSP_WIDGETS("PLUGIN12-PLACE-HOLDER", plugin12, TEGRA210_ADSP_PLUGIN12),
	ADSP_WIDGETS("PLUGIN13-PLACE-HOLDER", plugin13, TEGRA210_ADSP_PLUGIN13),
	ADSP_WIDGETS("PLUGIN14-PLACE-HOLDER", plugin14, TEGRA210_ADSP_PLUGIN14),
	ADSP_WIDGETS("PLUGIN15-PLACE-HOLDER", plugin15, TEGRA210_ADSP_PLUGIN15),
	ADSP_WIDGETS("PLUGIN16-PLACE-HOLDER", plugin16, TEGRA210_ADSP_PLUGIN16),
	ADSP_WIDGETS("PLUGIN17-PLACE-HOLDER", plugin17, TEGRA210_ADSP_PLUGIN17),
	ADSP_WIDGETS("PLUGIN18-PLACE-HOLDER", plugin18, TEGRA210_ADSP_PLUGIN18),
	ADSP_WIDGETS("PLUGIN19-PLACE-HOLDER", plugin19, TEGRA210_ADSP_PLUGIN19),
	ADSP_WIDGETS("PLUGIN20-PLACE-HOLDER", plugin20, TEGRA210_ADSP_PLUGIN20),
};

#define ADSP_EP_ROUTES(name)					\
	{ name " MUX",		"ADSP-FE1", "ADSP-FE1 RX"},	\
	{ name " MUX",		"ADSP-FE2", "ADSP-FE2 RX"},	\
	{ name " MUX",		"ADSP-FE3", "ADSP-FE3 RX"},	\
	{ name " MUX",		"ADSP-FE4", "ADSP-FE4 RX"},	\
	{ name " MUX",		"ADSP-FE5", "ADSP-FE5 RX"},	\
	{ name " MUX",		"ADSP-FE6", "ADSP-FE6 RX"},	\
	{ name " MUX",		"ADSP-FE7", "ADSP-FE7 RX"},	\
	{ name " MUX",		"ADSP-FE8", "ADSP-FE8 RX"},	\
	{ name " MUX",		"ADSP-FE9", "ADSP-FE9 RX"},	\
	{ name " MUX",		"ADSP-FE10", "ADSP-FE10 RX"},	\
	{ name " MUX",		"ADSP-FE11", "ADSP-FE11 RX"},	\
	{ name " MUX",		"ADSP-FE12", "ADSP-FE12 RX"},	\
	{ name " MUX",		"ADSP-FE13", "ADSP-FE13 RX"},	\
	{ name " MUX",		"ADSP-FE14", "ADSP-FE14 RX"},	\
	{ name " MUX",		"ADSP-FE15", "ADSP-FE15 RX"},	\
	{ name " MUX",		"ADSP-ADMAIF1", "ADSP-ADMAIF1 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF2", "ADSP-ADMAIF2 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF3", "ADSP-ADMAIF3 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF4", "ADSP-ADMAIF4 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF5", "ADSP-ADMAIF5 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF6", "ADSP-ADMAIF6 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF7", "ADSP-ADMAIF7 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF8", "ADSP-ADMAIF8 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF9", "ADSP-ADMAIF9 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF10", "ADSP-ADMAIF10 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF11", "ADSP-ADMAIF11 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF12", "ADSP-ADMAIF12 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF13", "ADSP-ADMAIF13 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF14", "ADSP-ADMAIF14 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF15", "ADSP-ADMAIF15 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF16", "ADSP-ADMAIF16 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF17", "ADSP-ADMAIF17 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF18", "ADSP-ADMAIF18 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF19", "ADSP-ADMAIF19 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF20", "ADSP-ADMAIF20 RX"}, \
	{ name " MUX",		"ADSP-EAVB", "ADSP-EAVB RX"}

#define ADSP_APM_IN_ROUTES(name)				\
	{ name " MUX",	"APM-IN1",	"APM-IN1 TX"},		\
	{ name " MUX",	"APM-IN2",	"APM-IN2 TX"},		\
	{ name " MUX",	"APM-IN3",	"APM-IN3 TX"},		\
	{ name " MUX",	"APM-IN4",	"APM-IN4 TX"},		\
	{ name " MUX",	"APM-IN5",	"APM-IN5 TX"},		\
	{ name " MUX",	"APM-IN6",	"APM-IN6 TX"},		\
	{ name " MUX",	"APM-IN7",	"APM-IN7 TX"},		\
	{ name " MUX",	"APM-IN8",	"APM-IN8 TX"},		\
	{ name " MUX",	"APM-IN9",	"APM-IN9 TX"},		\
	{ name " MUX",	"APM-IN10",	"APM-IN10 TX"},		\
	{ name " MUX",	"APM-IN11",	"APM-IN11 TX"},		\
	{ name " MUX",	"APM-IN12",	"APM-IN12 TX"},		\
	{ name " MUX",	"APM-IN13",	"APM-IN13 TX"},		\
	{ name " MUX",	"APM-IN14",	"APM-IN14 TX"},		\
	{ name " MUX",	"APM-IN15",	"APM-IN15 TX"}


#define ADSP_APM_OUT_ROUTES(name)				\
	{ name " MUX",		"APM-OUT1", "APM-OUT1 TX"},	\
	{ name " MUX",		"APM-OUT2", "APM-OUT2 TX"},	\
	{ name " MUX",		"APM-OUT3", "APM-OUT3 TX"},	\
	{ name " MUX",		"APM-OUT4", "APM-OUT4 TX"},	\
	{ name " MUX",		"APM-OUT5", "APM-OUT5 TX"},	\
	{ name " MUX",		"APM-OUT6", "APM-OUT6 TX"},	\
	{ name " MUX",		"APM-OUT7", "APM-OUT7 TX"},	\
	{ name " MUX",		"APM-OUT8", "APM-OUT8 TX"},	\
	{ name " MUX",		"APM-OUT9", "APM-OUT9 TX"},	\
	{ name " MUX",		"APM-OUT10", "APM-OUT10 TX"},	\
	{ name " MUX",		"APM-OUT11", "APM-OUT11 TX"},	\
	{ name " MUX",		"APM-OUT12", "APM-OUT12 TX"},	\
	{ name " MUX",		"APM-OUT13", "APM-OUT13 TX"},	\
	{ name " MUX",		"APM-OUT14", "APM-OUT14 TX"},	\
	{ name " MUX",		"APM-OUT15", "APM-OUT15 TX"}


#define ADSP_ADMA_ROUTES(name)					\
	{ name " MUX",	"ADMA1",	"ADMA1 TX"},		\
	{ name " MUX",	"ADMA2",	"ADMA2 TX"},		\
	{ name " MUX",	"ADMA3",	"ADMA3 TX"},		\
	{ name " MUX",	"ADMA4",	"ADMA4 TX"},		\
	{ name " MUX",	"ADMA5",	"ADMA5 TX"},		\
	{ name " MUX",	"ADMA6",	"ADMA6 TX"},		\
	{ name " MUX",	"ADMA7",	"ADMA7 TX"},		\
	{ name " MUX",	"ADMA8",	"ADMA8 TX"},		\
	{ name " MUX",	"ADMA9",	"ADMA9 TX"},		\
	{ name " MUX",	"ADMA10",	"ADMA10 TX"},		\
	{ name " MUX",	"ADMA11",	"ADMA11 TX"},		\
	{ name " MUX",	"ADMA12",	"ADMA12 TX"},		\
	{ name " MUX",	"ADMA13",	"ADMA13 TX"},		\
	{ name " MUX",	"ADMA14",	"ADMA14 TX"},		\
	{ name " MUX",	"ADMA15",	"ADMA15 TX"},		\
	{ name " MUX",	"ADMA1-TX",	"ADMA1-TX TX"},	\
	{ name " MUX",	"ADMA2-TX",	"ADMA2-TX TX"},	\
	{ name " MUX",	"ADMA3-TX",	"ADMA3-TX TX"},	\
	{ name " MUX",	"ADMA4-TX",	"ADMA4-TX TX"},	\
	{ name " MUX",	"ADMA5-TX",	"ADMA5-TX TX"},	\
	{ name " MUX",	"ADMA6-TX",	"ADMA6-TX TX"},	\
	{ name " MUX",	"ADMA7-TX",	"ADMA7-TX TX"},	\
	{ name " MUX",	"ADMA8-TX",	"ADMA8-TX TX"},	\
	{ name " MUX",	"ADMA9-TX",	"ADMA9-TX TX"},	\
	{ name " MUX",	"ADMA10-TX",	"ADMA10-TX TX"},	\
	{ name " MUX",	"ADMA11-TX",	"ADMA11-TX TX"},	\
	{ name " MUX",	"ADMA12-TX",	"ADMA12-TX TX"},	\
	{ name " MUX",	"ADMA13-TX",	"ADMA13-TX TX"},	\
	{ name " MUX",	"ADMA14-TX",	"ADMA14-TX TX"},	\
	{ name " MUX",	"ADMA15-TX",	"ADMA15-TX TX"}
#define ADSP_PLUGIN_ROUTES(name)					\
	{ name " MUX",	"PLUGIN1-PLACE-HOLDER",	"PLUGIN1-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN2-PLACE-HOLDER",	"PLUGIN2-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN3-PLACE-HOLDER",	"PLUGIN3-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN4-PLACE-HOLDER",	"PLUGIN4-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN5-PLACE-HOLDER",	"PLUGIN5-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN6-PLACE-HOLDER",	"PLUGIN6-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN7-PLACE-HOLDER",	"PLUGIN7-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN8-PLACE-HOLDER",	"PLUGIN8-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN9-PLACE-HOLDER",	"PLUGIN9-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN10-PLACE-HOLDER",	"PLUGIN10-PLACE-HOLDER TX"},	\
	{ name " MUX",	"PLUGIN11-PLACE-HOLDER",	"PLUGIN11-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN12-PLACE-HOLDER",	"PLUGIN12-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN13-PLACE-HOLDER",	"PLUGIN13-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN14-PLACE-HOLDER",	"PLUGIN14-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN15-PLACE-HOLDER",	"PLUGIN15-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN16-PLACE-HOLDER",	"PLUGIN16-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN17-PLACE-HOLDER",	"PLUGIN17-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN18-PLACE-HOLDER",	"PLUGIN18-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN19-PLACE-HOLDER",	"PLUGIN19-PLACE-HOLDER TX"},		\
	{ name " MUX",	"PLUGIN20-PLACE-HOLDER",	"PLUGIN20-PLACE-HOLDER TX"}

#define ADSP_EP_MUX_ROUTES(name)				\
	{ name " RX",		NULL, name " Receive"},		\
	{ name " Transmit",	NULL, name " TX"},		\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_OUT_ROUTES(name)

#define ADSP_EP_MUX_ROUTES_NULL_SINK(name)			\
	{ name " TX",           NULL, name " MUX"},		\
	{ name,     NULL, name " TX"},				\
	ADSP_APM_OUT_ROUTES(name)

#define ADSP_APM_IN_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_EP_ROUTES(name),					\
	ADSP_APM_OUT_ROUTES(name)

#define ADSP_APM_OUT_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_ADMA_ROUTES(name),					\
	ADSP_PLUGIN_ROUTES(name)

#define ADSP_PLUGIN_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_IN_ROUTES(name),					\
	ADSP_PLUGIN_ROUTES(name),					\
	ADSP_ADMA_ROUTES(name)

#define ADSP_ADMA_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_IN_ROUTES(name),				\
	ADSP_PLUGIN_ROUTES(name)

#define ADSP_ADMA_OUT_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_IN_ROUTES(name),				\
	ADSP_PLUGIN_ROUTES(name)

static struct snd_soc_dapm_route tegra210_adsp_routes[] = {
	ADSP_EP_MUX_ROUTES("ADSP-FE1"),
	ADSP_EP_MUX_ROUTES("ADSP-FE2"),
	ADSP_EP_MUX_ROUTES("ADSP-FE3"),
	ADSP_EP_MUX_ROUTES("ADSP-FE4"),
	ADSP_EP_MUX_ROUTES("ADSP-FE5"),
	ADSP_EP_MUX_ROUTES("ADSP-FE6"),
	ADSP_EP_MUX_ROUTES("ADSP-FE7"),
	ADSP_EP_MUX_ROUTES("ADSP-FE8"),
	ADSP_EP_MUX_ROUTES("ADSP-FE9"),
	ADSP_EP_MUX_ROUTES("ADSP-FE10"),
	ADSP_EP_MUX_ROUTES("ADSP-FE11"),
	ADSP_EP_MUX_ROUTES("ADSP-FE12"),
	ADSP_EP_MUX_ROUTES("ADSP-FE13"),
	ADSP_EP_MUX_ROUTES("ADSP-FE14"),
	ADSP_EP_MUX_ROUTES("ADSP-FE15"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF1"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF2"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF3"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF4"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF5"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF6"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF7"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF8"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF9"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF10"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF11"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF12"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF13"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF14"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF15"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF16"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF17"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF18"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF19"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF20"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK1"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK2"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK3"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK4"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK5"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK6"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK7"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK8"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK9"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK10"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK11"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK12"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK13"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK14"),
	ADSP_EP_MUX_ROUTES_NULL_SINK("NULL-SINK15"),

	ADSP_EP_MUX_ROUTES("ADSP-EAVB"),

	ADSP_APM_IN_MUX_ROUTES("APM-IN1"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN2"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN3"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN4"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN5"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN6"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN7"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN8"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN9"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN10"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN11"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN12"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN13"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN14"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN15"),

	ADSP_APM_OUT_MUX_ROUTES("APM-OUT1"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT2"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT3"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT4"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT5"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT6"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT7"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT8"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT9"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT10"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT11"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT12"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT13"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT14"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT15"),

	ADSP_ADMA_MUX_ROUTES("ADMA1"),
	ADSP_ADMA_MUX_ROUTES("ADMA2"),
	ADSP_ADMA_MUX_ROUTES("ADMA3"),
	ADSP_ADMA_MUX_ROUTES("ADMA4"),
	ADSP_ADMA_MUX_ROUTES("ADMA5"),
	ADSP_ADMA_MUX_ROUTES("ADMA6"),
	ADSP_ADMA_MUX_ROUTES("ADMA7"),
	ADSP_ADMA_MUX_ROUTES("ADMA8"),
	ADSP_ADMA_MUX_ROUTES("ADMA9"),
	ADSP_ADMA_MUX_ROUTES("ADMA10"),
	ADSP_ADMA_MUX_ROUTES("ADMA11"),
	ADSP_ADMA_MUX_ROUTES("ADMA12"),
	ADSP_ADMA_MUX_ROUTES("ADMA13"),
	ADSP_ADMA_MUX_ROUTES("ADMA14"),
	ADSP_ADMA_MUX_ROUTES("ADMA15"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA1-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA2-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA3-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA4-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA5-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA6-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA7-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA8-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA9-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA10-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA11-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA12-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA13-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA14-TX"),
	ADSP_ADMA_OUT_MUX_ROUTES("ADMA15-TX"),

	ADSP_PLUGIN_MUX_ROUTES("PLUGIN1-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN2-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN3-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN4-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN5-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN6-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN7-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN8-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN9-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN10-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN11-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN12-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN13-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN14-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN15-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN16-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN17-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN18-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN19-PLACE-HOLDER"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN20-PLACE-HOLDER"),
};

static void tegra210_adsp_wt_replace(struct device *dev, char **dest1,
					const char *src)
{
	char *dest = NULL;

	if (!*dest1 || !src)
		return;

	dest = (char *)devm_kzalloc(dev, strlen(src) + 5, GFP_KERNEL);
	if (strstr(*dest1, " TX")) {
		strcpy(dest, src);
		strcat(dest, " TX");
		*dest1 = dest;
	} else if (strstr(*dest1, " RX")) {
		strcpy(dest, src);
		strcat(dest, " RX");
		*dest1 = dest;
	} else if (strstr(*dest1, " MUX")) {
		strcpy(dest, src);
		strcat(dest, " MUX");
		*dest1 = dest;
	} else if (strstr(*dest1, " Transmit"))
		;/* do nothing */
	else if (strstr(*dest1, " Receive"))
		;/* do nothing */
	else {
		strcpy(dest, src);
		*dest1 = dest;
	}
}

static void tegra210_adsp_route_modify(struct device *dev,
			const char *wt_default, const char *wt_from_dt)
{
	int i, len, ret;

	if (!wt_default || !wt_from_dt)
		return;

	ret = device_property_read_string_array(dev, "fe-info",
				NULL, 0);
	len = strlen(wt_default);
	/* Modify dapm routing table */
	for (i = ((ret > 0) ? 0 : TEGRA210_ADSP_ROUTE_BASE);
		i < ARRAY_SIZE(tegra210_adsp_routes); i++) {
		/* replace sink name */
		if (tegra210_adsp_routes[i].sink)
			if (strstr(tegra210_adsp_routes[i].sink, wt_default) &&
				(tegra210_adsp_routes[i].sink[len] == ' ' ||
				tegra210_adsp_routes[i].sink[len] == 0))
				tegra210_adsp_wt_replace(dev,
					(char **)&tegra210_adsp_routes[i].sink,
					wt_from_dt);
		/* replace control name */
		if (tegra210_adsp_routes[i].control)
			if (strstr(tegra210_adsp_routes[i].control, wt_default)
				&& (tegra210_adsp_routes[i].control[len] == ' '
				|| tegra210_adsp_routes[i].control[len] == 0))
				tegra210_adsp_wt_replace(dev,
				(char **)&tegra210_adsp_routes[i].control,
					wt_from_dt);
		/* replace source name */
		if (tegra210_adsp_routes[i].source)
			if (strstr(tegra210_adsp_routes[i].source, wt_default)
				&& (tegra210_adsp_routes[i].source[len] == ' '
				|| tegra210_adsp_routes[i].source[len] == 0))
				tegra210_adsp_wt_replace(dev,
				(char **)&tegra210_adsp_routes[i].source,
					wt_from_dt);
	}
}

static void adsp_fe_name_override(struct device *dev, int count)
{
	int ret = 0, i;
	const char **fe_names;
	char *name;

	fe_names = devm_kcalloc(dev, count, sizeof(*fe_names),
				GFP_KERNEL);
	if (!fe_names)
		return;

	ret = device_property_read_string_array(dev, "fe-info",
			fe_names, count);
	ret = ret > ADSP_FE_END ? ADSP_FE_END : ret;

	for (i = 0 ; i < ret; i++) {
		tegra210_adsp_route_modify(dev,
			tegra210_adsp_mux_texts[1+i],
			fe_names[i]);

		name = devm_kzalloc(dev, strlen(fe_names[i]) + 3,
				GFP_KERNEL);
		strcpy((char *)name, fe_names[i]);
		strcat((char *)name, " TX");
		tegra210_adsp_widgets[(3*i)+1].name = name;

		name = devm_kzalloc(dev, strlen(fe_names[i]) + 4,
				GFP_KERNEL);
		strcpy((char *)name, fe_names[i]);
		strcat((char *)name, " MUX");
		tegra210_adsp_widgets[(3*i)+2].name = name;

		name = devm_kzalloc(dev, strlen(fe_names[i]) + 3,
				GFP_KERNEL);
		strcpy((char *)name, fe_names[i]);
		strcat((char *)name, " RX");
		tegra210_adsp_widgets[(3*i)].name = name;
		tegra210_adsp_mux_texts[1+i] = fe_names[i];
	}
}

static int tegra210_adsp_param_info(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;

	if (params->mask == SNDRV_CTL_ELEM_TYPE_INTEGER) {
		params->num_regs = 128;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 0xffffffff;
	}
	uinfo->type = params->mask;
	uinfo->count = params->num_regs;

	return 0;
}

static int tegra210_adsp_get_param(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;

	if (params->mask == SNDRV_CTL_ELEM_TYPE_INTEGER)
		memset(ucontrol->value.integer.value, 0,
			params->num_regs * sizeof(long));
	else
		memset(ucontrol->value.bytes.data, 0,
			params->num_regs);

	return 0;
}

/* tegra210_adsp_set_param - sets plugin parameters
 * @default: byte_format
 * @byte_format: nvfx_call_params_t based structure
 * @int_format: <plugin_method>,<#params>,<param1>,<param2>,....
 */
static int tegra210_adsp_set_param(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);
	struct tegra210_adsp_app *app = &adsp->apps[params->base];
	apm_msg_t apm_msg;
	int ret;

	if (!adsp->init_done) {
		dev_warn(adsp->dev, "ADSP is not booted yet\n");
		return -EPERM;
	}

	if (!app->plugin) {
		dev_warn(adsp->dev, "Unable to set %s, plugin not initialized\n",
			kcontrol->id.name);
		return -EPERM;
	}

	if (!app->connect) {
		dev_warn(adsp->dev, "%s, plugin not yet connected\n",
			kcontrol->id.name);
		return -EPERM;
	}

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_set_param;
	apm_msg.msg.fx_set_param_params.plugin.pvoid =
		app->plugin->plugin.pvoid;

	switch (params->mask) {
	case SNDRV_CTL_ELEM_TYPE_INTEGER:
	{
		int32_t num_params, i;
		/* check number of params to pass */
		num_params = (int32_t)ucontrol->value.integer.value[1];
		if (num_params < 1) {
			dev_warn(adsp->dev, "No params to pass to the plugin\n");
			return 0;
		}

		if (num_params + 2 >
			sizeof(apm_msg.msg.fx_set_param_params.params)/
			sizeof(apm_msg.msg.fx_set_param_params.params[0])) {
			dev_err(adsp->dev, "parameter too large\n");
			return -EINVAL;
		}

		apm_msg.msg.fx_set_param_params.params[0] =
			(sizeof(nvfx_call_params_t) +
			num_params * sizeof(int32_t));

		/* initialize the method */
		apm_msg.msg.fx_set_param_params.params[1] =
			(int32_t)ucontrol->value.integer.value[0];

		/* copy parameters */
		for (i = 0; i < num_params; i++)
			apm_msg.msg.fx_set_param_params.params[i + 2] =
				(int32_t)ucontrol->value.integer.value[i + 2];
	}
	break;

	case SNDRV_CTL_ELEM_TYPE_BYTES:
	{
		nvfx_call_params_t *call_params =
			(nvfx_call_params_t *)ucontrol->value.bytes.data;

		if (call_params->size >
			sizeof(apm_msg.msg.fx_set_param_params.params)) {
			dev_err(adsp->dev, "parameter too large\n");
			return -EINVAL;
		}

		/* copy parameters */
		memcpy(&apm_msg.msg.fx_set_param_params.params,
			call_params, call_params->size);
	}
	break;

	default:
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(adsp->dev);
	if (ret < 0) {
		dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
			__func__, ret);
		return ret;
	}
	ret = tegra210_adsp_send_msg(app, &apm_msg,
		TEGRA210_ADSP_MSG_FLAG_SEND | TEGRA210_ADSP_MSG_FLAG_NEED_ACK);
	pm_runtime_put(adsp->dev);

	return ret;
}

static int tegra210_adsp_tlv_callback(struct snd_kcontrol *kcontrol,
	int op_flag, unsigned int size, unsigned int __user *tlv)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);
	unsigned int count = size < params->num_regs ? size : params->num_regs;
	struct tegra210_adsp_app *app = &adsp->apps[params->base];
	unsigned int *tlv_data;
	int ret = 0;

	if (!adsp->init_done) {
		dev_warn(adsp->dev, "ADSP is not booted yet\n");
		return 0;
	}

	if (!app->plugin) {
		dev_warn(adsp->dev, "Plugin not yet initialized\n");
		return 0;
	}

	if (!app->connect) {
		dev_warn(adsp->dev, "%s, plugin not yet connected\n",
			kcontrol->id.name);
		return 0;
	}

	tlv_data = devm_kzalloc(adsp->dev, count, GFP_KERNEL);
	if (!tlv_data)
		return -ENOMEM;

	switch (op_flag) {
	case SNDRV_CTL_TLV_OP_WRITE:
	{
		apm_raw_data_msg_t *apm_msg;
		nvfx_call_params_t *call_params;

		apm_msg = devm_kzalloc(adsp->dev, sizeof(apm_raw_data_msg_t), GFP_KERNEL);
		if (!apm_msg) {
			dev_err(adsp->dev, "Failed to allocate memory for message\n");
			ret = -ENOMEM;
			goto end;
		}
		apm_msg->msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_raw_data_params_t);
		apm_msg->msg.call_params.size = sizeof(apm_fx_raw_data_params_t);
		apm_msg->msg.call_params.method = nvfx_apm_method_write_data;
		apm_msg->msg.fx_raw_data_params.plugin.pvoid =
			app->plugin->plugin.pvoid;

		if (copy_from_user(tlv_data, tlv, count)) {
			ret = -EFAULT;
			devm_kfree(adsp->dev, apm_msg);
			goto end;
		}

		call_params = (nvfx_call_params_t *)tlv_data;
		if (call_params->size <= 0 ||
			(call_params->size >
			sizeof(apm_msg->msg.fx_raw_data_params.data))) {
			ret = -EINVAL;
			devm_kfree(adsp->dev, apm_msg);
			goto end;
		}

		memcpy(&apm_msg->msg.fx_raw_data_params.data,
			call_params, call_params->size);

		ret = pm_runtime_get_sync(adsp->dev);
		if (ret < 0) {
			dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
				__func__, ret);
			devm_kfree(adsp->dev, apm_msg);
			goto end;
		}
		ret = tegra210_adsp_send_raw_data_msg(app, apm_msg);
		pm_runtime_put(adsp->dev);
		devm_kfree(adsp->dev, apm_msg);
		break;
	}
	case SNDRV_CTL_TLV_OP_READ:
	{
		int src = app->reg;
		struct tegra210_adsp_app *apm;

		while (!IS_APM_IN(src))
			src = tegra210_adsp_get_source(adsp, src);
		apm = &adsp->apps[src];
		ret = pm_runtime_get_sync(adsp->dev);
		if (ret < 0) {
			dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
				__func__, ret);
			goto end;
		}
		ret = tegra210_adsp_send_data_request_msg(app, count,
				TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			pm_runtime_put(adsp->dev);
			dev_err(adsp->dev, "Raw read request dropped\n");
			ret = -ETIMEDOUT;
			goto end;
		}
		ret = wait_for_completion_interruptible_timeout(
			apm->raw_msg_read_complete,
			msecs_to_jiffies(ADSP_RESPONSE_TIMEOUT));
		pm_runtime_put(adsp->dev);
		if (ret <= 0) {
			dev_err(adsp->dev, "No data received\n");
			ret = -ETIMEDOUT;
			goto end;
		}
		if (copy_to_user(tlv, apm->read_data.data, count)) {
			ret = -EFAULT;
			goto end;
		}
	}
	}

end:
	devm_kfree(adsp->dev, tlv_data);
	return ret;
}

static int tegra210_adsp_apm_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);
	struct tegra210_adsp_app *app = &adsp->apps[mc->reg];

	if (strstr(kcontrol->id.name, "Priority")) {
		ucontrol->value.integer.value[0] = app->priority;
	} else if (strstr(kcontrol->id.name, "Min ADSP Clock")) {
		ucontrol->value.integer.value[0] = app->min_adsp_clock;
	} else if (strstr(kcontrol->id.name, "Input Mode")) {
		ucontrol->value.integer.value[0] = app->input_mode;
	}
	return 0;
}

static int tegra210_adsp_apm_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);
	struct tegra210_adsp_app *app = &adsp->apps[mc->reg];
	apm_msg_t apm_msg;
	bool send_msg = 0;
	int ret = 0;

	if (!adsp->init_done) {
		dev_warn(adsp->dev, "ADSP is not booted yet\n");
		return -EPERM;
	}

	/* Controls here may execute whether or not APM is initialized */
	if (strstr(kcontrol->id.name, "Min ADSP Clock")) {
		app->min_adsp_clock = ucontrol->value.integer.value[0];
		return 0;
	}

	/* Check for APM initialized */
	if (!app->plugin) {
		dev_warn(adsp->dev, "Unable to set %s, APM %d not initialized\n",
			kcontrol->id.name, mc->reg);
		return -EPERM;
	}

	if (strstr(kcontrol->id.name, "Priority")) {
		apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_set_priority_params_t);
		apm_msg.msg.call_params.size = sizeof(apm_set_priority_params_t);
		apm_msg.msg.call_params.method = nvfx_apm_method_set_priority;
		apm_msg.msg.priority_params.priority =
			ucontrol->value.integer.value[0];
		app->priority = ucontrol->value.integer.value[0];
		send_msg = true;
	} else if (strstr(kcontrol->id.name, "Input Mode")) {
		apm_msg.msgq_msg.size =
			MSGQ_MSG_WSIZE(apm_set_input_mode_params_t);
		apm_msg.msg.call_params.size =
			sizeof(apm_set_input_mode_params_t);
		apm_msg.msg.call_params.method = nvfx_apm_method_set_input_mode;
		apm_msg.msg.input_mode_params.mode =
			ucontrol->value.integer.value[0];
		app->input_mode = ucontrol->value.integer.value[0];
		send_msg = true;
	}

	if (send_msg) {
		ret = pm_runtime_get_sync(adsp->dev);
		if (ret < 0) {
			dev_err(adsp->dev, "%s pm_runtime_get_sync error 0x%x\n",
				__func__, ret);
			return ret;
		}
		ret = tegra210_adsp_send_msg(app, &apm_msg,
				TEGRA210_ADSP_MSG_FLAG_SEND |
				TEGRA210_ADSP_MSG_FLAG_NEED_ACK);
		pm_runtime_put(adsp->dev);
	}

	return ret;
}

/* Maximum 128 integers or 512 bytes allowed */
#define SND_SOC_PARAM_EXT(xname, xbase)		\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	\
	.name = xname,		\
	.info = tegra210_adsp_param_info,		\
	.get = tegra210_adsp_get_param,		\
	.put = tegra210_adsp_set_param,		\
	.private_value =		\
		((unsigned long)&(struct soc_bytes)		\
		{.base = xbase, .num_regs = 512,		\
		.mask = SNDRV_CTL_ELEM_TYPE_BYTES}) }

#define SND_SOC_PARAM_TLV(xname, xbase, xcount)	\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	\
	.name = xname,	\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READWRITE |	\
			SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK,	\
	.tlv.c = tegra210_adsp_tlv_callback,	\
	.info = snd_soc_bytes_info_ext,	\
	.private_value =		\
		((unsigned long)&(struct soc_bytes)		\
		{.base = xbase, .num_regs = xcount,		\
		.mask = SNDRV_CTL_ELEM_TYPE_BYTES}) }

#define APM_CONTROL(xname, xmax)	\
	SOC_SINGLE_EXT("APM1 " xname, TEGRA210_ADSP_APM_IN1, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM2 " xname, TEGRA210_ADSP_APM_IN2, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM3 " xname, TEGRA210_ADSP_APM_IN3, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM4 " xname, TEGRA210_ADSP_APM_IN4, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM5 " xname, TEGRA210_ADSP_APM_IN5, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM6 " xname, TEGRA210_ADSP_APM_IN6, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM7 " xname, TEGRA210_ADSP_APM_IN7, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM8 " xname, TEGRA210_ADSP_APM_IN8, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM9 " xname, TEGRA210_ADSP_APM_IN9, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM10 " xname, TEGRA210_ADSP_APM_IN10, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM11 " xname, TEGRA210_ADSP_APM_IN11, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM12 " xname, TEGRA210_ADSP_APM_IN12, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM13 " xname, TEGRA210_ADSP_APM_IN13, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM14 " xname, TEGRA210_ADSP_APM_IN14, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put),	\
	SOC_SINGLE_EXT("APM15 " xname, TEGRA210_ADSP_APM_IN15, 0, xmax, 0,\
	tegra210_adsp_apm_get, tegra210_adsp_apm_put)

/* Any new addition of control should be added after PLUGIN controls otherwise
* the index of PLUGIN needs to be changed with define PLUGIN_SET_PARAMS_IDX and
* PLUGIN_SEND_BYTES_IDX */
static struct snd_kcontrol_new tegra210_adsp_controls[] = {
	SOC_SINGLE_BOOL_EXT("ADSP init", 0,
		tegra210_adsp_init_get, tegra210_adsp_init_put),
	SND_SOC_PARAM_EXT("PLUGIN1-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN1),
	SND_SOC_PARAM_EXT("PLUGIN2-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN2),
	SND_SOC_PARAM_EXT("PLUGIN3-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN3),
	SND_SOC_PARAM_EXT("PLUGIN4-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN4),
	SND_SOC_PARAM_EXT("PLUGIN5-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN5),
	SND_SOC_PARAM_EXT("PLUGIN6-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN6),
	SND_SOC_PARAM_EXT("PLUGIN7-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN7),
	SND_SOC_PARAM_EXT("PLUGIN8-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN8),
	SND_SOC_PARAM_EXT("PLUGIN9-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN9),
	SND_SOC_PARAM_EXT("PLUGIN10-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN10),
	SND_SOC_PARAM_EXT("PLUGIN11-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN11),
	SND_SOC_PARAM_EXT("PLUGIN12-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN12),
	SND_SOC_PARAM_EXT("PLUGIN13-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN13),
	SND_SOC_PARAM_EXT("PLUGIN14-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN14),
	SND_SOC_PARAM_EXT("PLUGIN15-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN15),
	SND_SOC_PARAM_EXT("PLUGIN16-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN16),
	SND_SOC_PARAM_EXT("PLUGIN17-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN17),
	SND_SOC_PARAM_EXT("PLUGIN18-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN18),
	SND_SOC_PARAM_EXT("PLUGIN19-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN19),
	SND_SOC_PARAM_EXT("PLUGIN20-PLACE-HOLDER set params",
		TEGRA210_ADSP_PLUGIN20),
	SND_SOC_PARAM_TLV("PLUGIN1-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN1, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN2-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN2, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN3-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN3, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN4-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN4, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN5-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN5, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN6-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN6, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN7-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN7, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN8-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN8, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN9-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN9, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN10-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN10, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN11-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN11, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN12-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN12, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN13-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN13, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN14-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN14, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN15-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN15, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN16-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN16, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN17-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN17, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN18-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN18, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN19-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN19, 0x1000),
	SND_SOC_PARAM_TLV("PLUGIN20-PLACE-HOLDER send bytes",
			TEGRA210_ADSP_PLUGIN20, 0x1000),
	SND_SOC_PARAM_EXT("ADMA1 set params",
		TEGRA210_ADSP_PLUGIN_ADMA1),
	SND_SOC_PARAM_EXT("ADMA2 set params",
		TEGRA210_ADSP_PLUGIN_ADMA2),
	SND_SOC_PARAM_EXT("ADMA3 set params",
		TEGRA210_ADSP_PLUGIN_ADMA3),
	SND_SOC_PARAM_EXT("ADMA4 set params",
		TEGRA210_ADSP_PLUGIN_ADMA4),
	SND_SOC_PARAM_EXT("ADMA5 set params",
		TEGRA210_ADSP_PLUGIN_ADMA5),
	SND_SOC_PARAM_EXT("ADMA6 set params",
		TEGRA210_ADSP_PLUGIN_ADMA6),
	SND_SOC_PARAM_EXT("ADMA7 set params",
		TEGRA210_ADSP_PLUGIN_ADMA7),
	SND_SOC_PARAM_EXT("ADMA8 set params",
		TEGRA210_ADSP_PLUGIN_ADMA8),
	SND_SOC_PARAM_EXT("ADMA9 set params",
		TEGRA210_ADSP_PLUGIN_ADMA9),
	SND_SOC_PARAM_EXT("ADMA10 set params",
		TEGRA210_ADSP_PLUGIN_ADMA10),
	SND_SOC_PARAM_EXT("ADMA11 set params",
		TEGRA210_ADSP_PLUGIN_ADMA11),
	SND_SOC_PARAM_EXT("ADMA12 set params",
		TEGRA210_ADSP_PLUGIN_ADMA12),
	SND_SOC_PARAM_EXT("ADMA13 set params",
		TEGRA210_ADSP_PLUGIN_ADMA13),
	SND_SOC_PARAM_EXT("ADMA14 set params",
		TEGRA210_ADSP_PLUGIN_ADMA14),
	SND_SOC_PARAM_EXT("ADMA15 set params",
		TEGRA210_ADSP_PLUGIN_ADMA15),
	SND_SOC_PARAM_EXT("ADMA1-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA1_TX),
	SND_SOC_PARAM_EXT("ADMA2-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA2_TX),
	SND_SOC_PARAM_EXT("ADMA3-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA3_TX),
	SND_SOC_PARAM_EXT("ADMA4-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA4_TX),
	SND_SOC_PARAM_EXT("ADMA5-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA5_TX),
	SND_SOC_PARAM_EXT("ADMA6-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA6_TX),
	SND_SOC_PARAM_EXT("ADMA7-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA7_TX),
	SND_SOC_PARAM_EXT("ADMA8-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA8_TX),
	SND_SOC_PARAM_EXT("ADMA9-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA9_TX),
	SND_SOC_PARAM_EXT("ADMA10-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA10_TX),
	SND_SOC_PARAM_EXT("ADMA11-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA11_TX),
	SND_SOC_PARAM_EXT("ADMA12-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA12_TX),
	SND_SOC_PARAM_EXT("ADMA13-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA13_TX),
	SND_SOC_PARAM_EXT("ADMA14-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA14_TX),
	SND_SOC_PARAM_EXT("ADMA15-TX set params",
		TEGRA210_ADSP_PLUGIN_ADMA15_TX),

	APM_CONTROL("Priority", APM_PRIORITY_MAX),
	APM_CONTROL("Min ADSP Clock", INT_MAX),
	APM_CONTROL("Input Mode", INT_MAX),
};


static int tegra210_adsp_component_probe(struct snd_soc_component *component)
{
	component->read = tegra210_adsp_read;
	component->write = tegra210_adsp_write;

	return 0;
}

static const struct snd_soc_component_driver tegra210_adsp_component = {
	.name		= "tegra210-adsp",
	.dapm_widgets		= tegra210_adsp_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tegra210_adsp_widgets),
	.dapm_routes		= tegra210_adsp_routes,
	.num_dapm_routes	= ARRAY_SIZE(tegra210_adsp_routes),
	.controls		= tegra210_adsp_controls,
	.num_controls		= ARRAY_SIZE(tegra210_adsp_controls),
	.probe			= tegra210_adsp_component_probe,
};

static struct snd_soc_codec_driver tegra210_adsp_codec = {
	.idle_bias_off = 1,
};

static int tegra210_adsp_pcm_probe(struct snd_soc_platform *platform)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(&platform->component);
	dapm->idle_bias_off = 1;
	platform->component.read = tegra210_adsp_read;
	platform->component.write = tegra210_adsp_write;
	return 0;
}

static struct snd_soc_platform_driver tegra210_adsp_platform = {
	.ops			= &tegra210_adsp_pcm_ops,
	.compr_ops		= &tegra210_adsp_compr_ops,
	.pcm_new		= tegra210_adsp_pcm_new,
	.pcm_free		= tegra210_adsp_pcm_free,
	.probe			= tegra210_adsp_pcm_probe,
};

static u64 tegra_dma_mask = DMA_BIT_MASK(32);

static struct adsp_soc_data adsp_soc_data_t210 = {
	.is_soc_t210 = true,
	.max_adma_ch = TEGRA210_MAX_ADMA_CHANNEL,
};

static struct adsp_soc_data adsp_soc_data_t186 = {
	.is_soc_t210 = false,
	.max_adma_ch = TEGRA186_MAX_ADMA_CHANNEL,
};

static const struct of_device_id tegra210_adsp_audio_of_match[] = {
	{ .compatible = "nvidia,tegra210-adsp-audio", .data = &adsp_soc_data_t210},
	{ .compatible = "nvidia,tegra186-adsp-audio", .data = &adsp_soc_data_t186},
	{},
};

static void adsp_control_name_override(struct device *dev, int wt_idx, int i,
							int mux_idx)
{
	char *name = devm_kzalloc(dev, strlen(adsp_app_desc[i].wt_name) + 3,
					GFP_KERNEL);
	strcpy((char *)name, adsp_app_desc[i].wt_name);
	strcat((char *)name, " TX");
	tegra210_adsp_widgets[wt_idx].name = name;

	name = devm_kzalloc(dev, strlen(adsp_app_desc[i].wt_name) + 4,
					GFP_KERNEL);
	strcpy((char *)name, adsp_app_desc[i].wt_name);
	strcat((char *)name, " MUX");
	tegra210_adsp_widgets[wt_idx+PLUGIN_SET_PARAMS_IDX].name = name;

	name = devm_kzalloc(dev, strlen(adsp_app_desc[i].wt_name) + 12,
					GFP_KERNEL);
	strcpy((char *)name, adsp_app_desc[i].wt_name);
	strcat((char *)name, " set params");
	tegra210_adsp_controls[i+PLUGIN_SET_PARAMS_IDX].name = name;

	name = devm_kzalloc(dev, strlen(adsp_app_desc[i].wt_name) + 12,
					GFP_KERNEL);
	strcpy((char *)name, adsp_app_desc[i].wt_name);
	strcat((char *)name, " send bytes");
	tegra210_adsp_controls[i+PLUGIN_SEND_BYTES_IDX].name = name;

	name = devm_kzalloc(dev, strlen(adsp_app_desc[i].wt_name) + 1,
					GFP_KERNEL);
	strcpy((char *)name, adsp_app_desc[i].wt_name);
	tegra210_adsp_mux_texts[mux_idx] = name;
}

static int tegra210_adsp_audio_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *subnp;
	const struct of_device_id *match;
	struct soc_bytes *controls;
	struct tegra210_adsp *adsp;
	int i, j, wt_idx, mux_idx, ret = 0;
	unsigned int compr_ops = 1;
	uint32_t adma_ch_page = 0;
	uint32_t adma_ch_start = TEGRA210_ADSP_ADMA_CHANNEL_START_HV;
	uint32_t adma_ch_cnt = TEGRA210_ADSP_ADMA_CHANNEL_COUNT;
	char plugin_info[20], apm_info[20];
	struct netlink_kernel_cfg cfg = {
		.input = tegra210_adsp_nl_recv_msg,
	};

	pr_info("tegra210_adsp_audio_platform_probe: platform probe started\n");

	if (dev_set_name(&pdev->dev, "%s", DRV_NAME_ADSP) < 0) {
		dev_err(&pdev->dev, "error in setting adsp device name\n");
		return -ENODEV;
	}

	match = of_match_device(tegra210_adsp_audio_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	adsp = devm_kzalloc(&pdev->dev, sizeof(*adsp), GFP_KERNEL);
	if (!adsp) {
		dev_err(&pdev->dev, "Can't allocate tegra210_adsp_ctx\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, adsp);
	adsp->dev = &pdev->dev;
	adsp->soc_data = (struct adsp_soc_data *)match->data;


	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		adsp->ahub_clk = devm_clk_get(&pdev->dev, "ahub");
		if (IS_ERR(adsp->ahub_clk)) {
			dev_err(&pdev->dev, "Error: Missing AHUB clock\n");
			ret = PTR_ERR(adsp->ahub_clk);
			goto err;
		}

		adsp->ape_clk = devm_clk_get(&pdev->dev, "ape");
		if (IS_ERR(adsp->ape_clk)) {
			dev_err(&pdev->dev, "Error: Missing APE clock\n");
			ret = PTR_ERR(adsp->ape_clk);
			goto err;
		}

		if (!adsp->soc_data->is_soc_t210) {
			adsp->apb2ape_clk = devm_clk_get(&pdev->dev, "apb2ape");
			if (IS_ERR(adsp->apb2ape_clk)) {
				dev_err(&pdev->dev, "Error: Missing APB2APE clock\n");
				ret = PTR_ERR(adsp->apb2ape_clk);
				goto err;
			}
		}
	}

	/* TODO: Add mixer control to set I2S playback rate */
	adsp->i2s_rate = 48000;
	INIT_WORK(&adsp->override_freq_work, tegra_adsp_override_freq_worker);
	mutex_init(&adsp->mutex);
	pdev->dev.dma_mask = &tegra_dma_mask;
	pdev->dev.coherent_dma_mask = tegra_dma_mask;

	tegra_pd_add_device(&pdev->dev);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_pm_disable;

	for (i = 0; i < TEGRA210_ADSP_VIRT_REG_MAX; i++) {
		adsp->apps[i].reg = i;
		adsp->apps[i].priority = 0;
		adsp->apps[i].min_adsp_clock = 0;
		adsp->apps[i].input_mode = NVFX_APM_INPUT_MODE_PUSH;
	}

	ret = device_property_read_string_array(&pdev->dev, "fe-info",
				NULL, 0);
	if (ret > 0)
		adsp_fe_name_override(&pdev->dev, ret);

	/* get the plugin count */
	if (of_property_read_u32(pdev->dev.of_node,
				"num-plugin",
				&adsp_app_count) < 0) {
		dev_warn(&pdev->dev, "Missing ADSP plugin count\n");
		adsp_app_count = 0;
	}

	/* check for maximum number of plugins */
	if (adsp_app_count > PLUGIN_NUM) {
		dev_err(&pdev->dev, "num-plugin: %d\n", adsp_app_count);
		dev_err(&pdev->dev, "Max plugins supported: %d\n",
				PLUGIN_NUM);
		adsp_app_count = PLUGIN_NUM;
	}

	/* allocate memory for app descritors */
	adsp_app_desc = devm_kzalloc(&pdev->dev,
				(adsp_app_count + ARRAY_SIZE(adsp_app_minimal))
				* sizeof(*adsp_app_desc), GFP_KERNEL);
	if (!adsp_app_desc) {
		dev_err(&pdev->dev, "Can't allocate tegra210_adsp_app descriptor\n");
		ret = -ENOMEM;
		goto err_pm_disable;
	}

	/* parse the plugin, firmware, widget names and params */
	for (i = 0; i < adsp_app_count; i++) {
		memset((void *)plugin_info, '\0', 20);
		sprintf(plugin_info, "plugin-info-%d", i+1);
		subnp = of_get_child_by_name(np, plugin_info);
		if (subnp) {
			if (of_property_read_string(subnp, "plugin-name",
				&adsp_app_desc[i].name)) {
				dev_err(&pdev->dev,
					"Missing property plugin-name\n");
				ret = -EINVAL;
				goto err_pm_disable;
			}
			if (of_property_read_string(subnp, "firmware-name",
				&adsp_app_desc[i].fw_name)) {
				dev_err(&pdev->dev,
					"Missing property firmware-name\n");
				ret = -EINVAL;
				goto err_pm_disable;
			}
			if (of_property_read_string(subnp, "widget-name",
				&adsp_app_desc[i].wt_name)) {
				dev_warn(&pdev->dev,
					"Missing property widget-name for %s\n",
					adsp_app_desc[i].name);
				adsp_app_desc[i].wt_name = NULL;
			} else {
				if (adsp_app_desc[i].wt_name) {
					/* override the widget names from DT if any */
					mux_idx = TEGRA210_ADSP_PLUGIN1 + i;
					wt_idx = TEGRA210_ADSP_WIDGET_BASE + (2*i);
					if (strlen(tegra210_adsp_mux_texts[mux_idx]) <
						strlen(adsp_app_desc[i].wt_name)) {
						dev_err(&pdev->dev,
							"Widget name too long %s, allowed len %zu\n",
							adsp_app_desc[i].wt_name,
							strlen(tegra210_adsp_mux_texts[mux_idx]));
						continue;
					}
					tegra210_adsp_route_modify(&pdev->dev,
						tegra210_adsp_mux_texts[mux_idx],
						adsp_app_desc[i].wt_name);

					adsp_control_name_override(&pdev->dev,
							wt_idx, i, mux_idx);

				}
			}
			if (of_property_read_u32(subnp, "param-type",
				&adsp_app_desc[i].param_type)) {
				dev_info(&pdev->dev,
					"Default param-type to BYTE for %s\n",
					adsp_app_desc[i].name);
				adsp_app_desc[i].param_type =
					SNDRV_CTL_ELEM_TYPE_BYTES;
			} else {
				/* override the param-type from DT if any */
				controls =
					(void *)tegra210_adsp_controls[i+1].private_value;
				controls->mask = adsp_app_desc[i].param_type;
			}
			adsp_app_desc[i].reg_start = TEGRA210_ADSP_PLUGIN1 + i;
			adsp_app_desc[i].reg_end = TEGRA210_ADSP_PLUGIN1 + i;
		} else {
			dev_err(&pdev->dev,
				"Property '%s' missing or invalid\n",
				plugin_info);
			ret = -EINVAL;
			goto err_pm_disable;
		}
	}


	/* copy basic apps needed */
	memcpy(&adsp_app_desc[adsp_app_count],
			&adsp_app_minimal[0], sizeof(adsp_app_minimal));
	adsp_app_count += ARRAY_SIZE(adsp_app_minimal);

	for (i = 0; i < adsp_app_count; i++) {
		for (j = adsp_app_desc[i].reg_start;
			j <= adsp_app_desc[i].reg_end; j++)
			adsp->apps[j].desc = &adsp_app_desc[i];
	}

	/* enable/disable compr-ops from DT */
	of_property_read_u32(pdev->dev.of_node, "compr-ops", &compr_ops);
	if (!compr_ops)
		tegra210_adsp_platform.compr_ops = NULL;

	if (of_property_read_u32_index(pdev->dev.of_node, "nvidia,adma_ch_page",
		0, &adma_ch_page)) {
		dev_info(&pdev->dev, "adma channel page address dt entry not found\n");
		dev_info(&pdev->dev, "using adma channel page 0\n");
	}
	adsp->adma_ch_page = adma_ch_page;

	adma_ch_start = TEGRA210_ADSP_ADMA_CHANNEL_START;

	if (of_property_read_u32_index(pdev->dev.of_node,
		"nvidia,adma_ch_start", 0, &adma_ch_start)) {
		dev_info(&pdev->dev, "adma channel start dt entry not found\n");
		dev_info(&pdev->dev, "using adma channels from 16\n");
	}
	adsp->adma_ch_start = (adma_ch_start > adsp->soc_data->max_adma_ch)
				? adsp->soc_data->max_adma_ch : adma_ch_start;

	if (of_property_read_u32_index(pdev->dev.of_node,
		"nvidia,adma_ch_cnt", 0, &adma_ch_cnt)) {
		dev_info(&pdev->dev, "adma channel cnt dt entry not found\n");
		dev_info(&pdev->dev, "using default adma channels count 10\n");
	}
	adsp->adma_ch_cnt = adma_ch_cnt;

	if ((adsp->adma_ch_start + adsp->adma_ch_cnt) >
					adsp->soc_data->max_adma_ch) {
		adsp->adma_ch_cnt =
			adsp->soc_data->max_adma_ch - adsp->adma_ch_start;
		dev_info(&pdev->dev, "adma channel cnt set to %d\n",
						adsp->adma_ch_cnt);
	}

	ret = snd_soc_register_platform(&pdev->dev, &tegra210_adsp_platform);
	if (ret) {
		dev_err(&pdev->dev, "Could not register platform: %d\n", ret);
		goto err_pm_disable;
	}

	ret = snd_soc_register_component(&pdev->dev, &tegra210_adsp_component,
			tegra210_adsp_dai, ARRAY_SIZE(tegra210_adsp_dai));
	if (ret) {
		dev_err(&pdev->dev, "Could not register component: %d\n", ret);
		goto err_unregister_platform;
	}

	ret = snd_soc_register_codec(&pdev->dev, &tegra210_adsp_codec,
				     tegra210_adsp_codec_dai,
				     ARRAY_SIZE(tegra210_adsp_codec_dai));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err_unregister_platform;
	}

	for (i = 0; i < (APM_IN_END - APM_IN_START + 1); i++) {
		apm_stack_size[i] = 0;
		memset((void *)apm_info, '\0', 20);
		sprintf(apm_info, "apm%d-stack-size", i+1);
		of_property_read_u32(pdev->dev.of_node, apm_info,
				&apm_stack_size[i]);
	}

	adsp->nl_sk = netlink_kernel_create(&init_net, NETLINK_ADSP_EVENT, &cfg);
	if (!adsp->nl_sk) {
		pr_err("Error creating socket.\n");
		return -1;
	}
	pr_info("Succssfully created NETLINK_ADSP_EVENT socket\n");

	pr_info("tegra210_adsp_audio_platform_probe probe successfull.");
	return 0;

err_unregister_platform:
	snd_soc_unregister_platform(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err:
	return ret;
}

static int tegra210_adsp_audio_platform_remove(struct platform_device *pdev)
{
	struct tegra210_adsp *adsp = dev_get_drvdata(&pdev->dev);

	netlink_kernel_release(adsp->nl_sk);
	pm_runtime_disable(&pdev->dev);
	tegra_pd_remove_device(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct dev_pm_ops tegra210_adsp_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_adsp_runtime_suspend,
			   tegra210_adsp_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)

};

static struct platform_driver tegra210_adsp_audio_driver = {
	.driver = {
		.name = DRV_NAME_ADSP,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_adsp_audio_of_match,
		.pm = &tegra210_adsp_pm_ops,
		.suppress_bind_attrs = true,
	},
	.probe = tegra210_adsp_audio_platform_probe,
	.remove = tegra210_adsp_audio_platform_remove,
};
module_platform_driver(tegra210_adsp_audio_driver);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 ADSP Audio driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME_ADSP);
MODULE_DEVICE_TABLE(of, tegra210_adsp_audio_of_match);

