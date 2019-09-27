/*
 *  HID driver for the Android TV remote
 *  providing keys and microphone audio functionality
 *
 * Copyright (C) 2014 Google, Inc.
 * Copyright (c) 2015-2019, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/hid.h>
#include <linux/hiddev.h>
#include <linux/hardirq.h>
#include <linux/iio/imu/tsfw_icm20628.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#include "hid-ids.h"
#include "sbcdec.h"

MODULE_LICENSE("GPL v2");

#define snd_atvr_log(...) pr_info("snd_atvr: " __VA_ARGS__)

#define JOYSTICK_FUZZ 64
#define TRIGGER_FUZZ 64
#define JOYSTICK_FLAT 64
#define TRIGGER_FLAT 0

#define ADPCM_AUDIO_REPORT_ID 30

#define MSBC_AUDIO1_REPORT_ID 0xF7
#define MSBC_AUDIO2_REPORT_ID 0xFA
#define MSBC_AUDIO3_REPORT_ID 0xFB

#define INPUT_REPORT_ID 2
#define INPUT_EVT_INTR_DATA_ID 10

#define KEYCODE_PRESENT_IN_AUDIO_PACKET_FLAG 0x80

/* defaults */
#define MAX_PCM_DEVICES     1
#define MAX_PCM_SUBSTREAMS  4
#define MAX_MIDI_DEVICES    0

/* Define these all in one place so they stay in sync. */
#define USE_RATE_MIN          8000
#define USE_RATE_MAX          16000
#define USE_RATE_MAX_PEPPER   8000
#define USE_RATES_ARRAY      {USE_RATE_MIN}
#define USE_RATES_MASK       (SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000)
#define USE_RATES_MASK_PEPPER    SNDRV_PCM_RATE_8000

#define MAX_FRAMES_PER_BUFFER  (8192)

#define USE_CHANNELS_MIN   1
#define USE_CHANNELS_MAX   2
#define USE_CHANNELS_MAX_PEPPER   1
#define USE_PERIODS_MIN    1
#define USE_PERIODS_MAX    1024

#define MAX_PCM_BUFFER_SIZE  (MAX_FRAMES_PER_BUFFER * sizeof(int16_t))
#define MIN_PERIOD_SIZE      64
#define MAX_PERIOD_SIZE      (MAX_PCM_BUFFER_SIZE / 8)
#define USE_FORMATS          (SNDRV_PCM_FMTBIT_S16_LE)

#define PACKET_TYPE_ADPCM 0
#define PACKET_TYPE_MSBC  1

/* timer callback occurs every 20ms, and silence max timeout is 5 seconds */
#define MAX_SILENCE_COUNTER 250

/* Normally SBC has a H2 header but because we want
 * to embed keycode support while audio is active without
 * incuring an additional packet in the connection interval,
 * we only use half the H2 header.  A normal H2 header has
 * a 12-bit synchronization word and a 2-bit sequence number
 * (SN0, SN1).  The sequence number is duplicated, so each
 * pair of bits in the sequence number shall be always 00
 * or 11 (see 5.7.2 of HFP_SPEC_V16).  We only receive
 * the second byte of the H2 header that has the latter part
 * of the sync word and the entire sequence number.
 *
 *  0      70      7
 * b100000000001XXYY - where X is SN0 repeated and Y is SN1 repeated
 *
 * So the sequence numbers are:
 * b1000000000010000 - 0x01 0x08  - only the 0x08 is received
 * b1000000000011100 - 0x01 0x38  - only the 0x38 is received
 * b1000000000010011 - 0x01 0xc8  - only the 0xc8 is received
 * b1000000000011111 - 0x01 0xf8  - only the 0xf8 is received
 *
 * Each mSBC frame is split over 3 BLE frames, where each BLE packet has
 * a 20 byte payload.
 * The first BLE packet has the format:
 * byte 0: keycode LSB
 * byte 1: keycode MSB, with most significant bit 0 for no key
 *         code active and 1 if keycode is active
 * byte 2: Second byte of H2
 * bytes 3-19: then four byte SBC header, then 13 bytes of audio data
 *
 * The second and third packet are purely 20 bytes of audio
 * data.  Second packet arrives on report 0xFA and third packet
 * arrives on report 0xFB.
 *
 * The mSBC decoder works on a mSBC frame, including the four byte SBC header,
 * so we have to accumulate 3 BLE packets before sending it to the decoder.
 */
#define NUM_SEQUENCES 4
const uint8_t msbc_sequence_table[NUM_SEQUENCES] = {0x08, 0x38, 0xc8, 0xf8};
#define BLE_PACKETS_PER_MSBC_FRAME 3
#define MSBC_PACKET1_BYTES 17
#define MSBC_PACKET2_BYTES 20
#define MSBC_PACKET3_BYTES 20

#define BYTES_PER_MSBC_FRAME \
	(MSBC_PACKET1_BYTES + MSBC_PACKET2_BYTES + MSBC_PACKET3_BYTES)

const uint8_t msbc_start_offset_in_packet[BLE_PACKETS_PER_MSBC_FRAME] = {
	1, /* SBC header starts after 1 byte sequence num portion of H2 */
	0,
	0
};
const uint8_t msbc_start_offset_in_buffer[BLE_PACKETS_PER_MSBC_FRAME] = {
	0,
	MSBC_PACKET1_BYTES,
	MSBC_PACKET1_BYTES + MSBC_PACKET2_BYTES
};
const uint8_t msbc_bytes_in_packet[BLE_PACKETS_PER_MSBC_FRAME] = {
	/* includes the SBC header but not the sequence num or keycode */
	MSBC_PACKET1_BYTES,
	MSBC_PACKET2_BYTES,
	MSBC_PACKET3_BYTES
};

struct fifo_packet {
	uint8_t  type;
	uint8_t  num_bytes;
	/* Expect no more than 20 bytes. But align struct size to power of 2. */
	uint8_t  raw_data[1022];
};

#define MAX_SAMPLES_PER_PACKET 128
#define MIN_SAMPLES_PER_PACKET_P2  32
#define MAX_PACKETS_PER_BUFFER  \
		(MAX_FRAMES_PER_BUFFER / MIN_SAMPLES_PER_PACKET_P2)
#define MAX_BUFFER_SIZE  \
		(MAX_PACKETS_PER_BUFFER * sizeof(struct fifo_packet))

#define SND_ATVR_RUNNING_TIMEOUT_MSEC    (500)

#define TIMER_STATE_BEFORE_DECODE    0
#define TIMER_STATE_DURING_DECODE    1
#define TIMER_STATE_AFTER_DECODE     2

#define MAX_DEBUG_REPORTS	5
#define HID_GEN_DESK_DEBUG	(HID_UP_GENDESK | 0xFFFF)
/* Time difference tolerance in ms */
#define MAX_PACKET_DIFF_TOLERANCE 100
/*
 * Max time(in ms) below which packet delivery check is done.
 * Once this time passes, time difference checks is ignored as only 2 bytes are
 * used for getting time difference and it rounds off after 65 seconds.
 */
#define MAX_TIME_BETWEEN_PACKETS 65000

struct hid_debug_data {
	u8	id;
	u8	seq_num;
	ktime_t	time;
};

/* move hid device, sound card into per device data structure */
struct shdr_device {
	struct hid_device	*hdev;
	struct snd_card *shdr_card;
	struct tsfw_icm20628_fn_dev *snsr_fns;
	struct tsfw_icm20628_state *st;
	struct work_struct snsr_probe_work;
	struct delayed_work hid_miss_war_work;
	struct mutex hid_miss_war_lock;
	int hid_miss_war_timeout;
	u32 last_ljsx, last_ljsy;	/* Last left joystick x, y */
	u32 last_rjsx, last_rjsy;	/* Last right joystick x, y */
	struct hid_debug_data	debug_info[MAX_DEBUG_REPORTS];
};

/* counter of how many continous silent timer callback in a row */
static unsigned int silence_counter;
static int num_remotes;
static struct mutex snd_cards_lock;
static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;  /* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;   /* ID for this card */
/* enable all cards by default */
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;
/* remember snd cards already in use */
static bool cards_in_use[SNDRV_CARDS] = {false};
/* Linux does not like NULL initialization. */
static char *model[SNDRV_CARDS]; /* = {[0 ... (SNDRV_CARDS - 1)] = NULL}; */
static int pcm_devs[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};
static int pcm_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for SHIELD Remote soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for SHIELD Remote soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable this SHIELD Remote soundcard.");
module_param_array(model, charp, NULL, 0444);
MODULE_PARM_DESC(model, "Soundcard model.");
module_param_array(pcm_devs, int, NULL, 0444);
MODULE_PARM_DESC(pcm_devs, "PCM devices # (0-4) for SHIELD Remote driver.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams,
	"PCM substreams # (1-128) for SHIELD Remote driver?");

static void atvr_ts_joystick_missreport_stats_inc(struct hid_device *hdev);

/* Debug feature to save captured raw and decoded audio into buffers
 * and make them available for reading from misc devices.
 * It will record the last session only and only up to the buffer size.
 * The recording is cleared on read.
 */
#define DEBUG_WITH_MISC_DEVICE 0

/* Debug feature to trace audio packets being received */
#define DEBUG_AUDIO_RECEPTION 0

/* Debug feature to trace tx audio packets */
#define DEBUG_AUDIO_TX 0

/* Debug feature to trace HID reports we see */
/* #define DEBUG_HID_RAW_INPUT */

/* Debug timer related issue */
/* #define DEBUG_TIMER */

#if (DEBUG_WITH_MISC_DEVICE == 1)
static int16_t large_pcm_buffer[1280*1024];
static int large_pcm_index;

static struct miscdevice pcm_dev_node;
static int pcm_dev_open(struct inode *inode, struct file *file)
{
	/* nothing special to do here right now. */
	return 0;
}

static ssize_t pcm_dev_read(struct file *file, char __user *buffer,
			    size_t count, loff_t *ppos)
{
	const uint8_t *data = (const uint8_t *)large_pcm_buffer;
	size_t bytes_left = large_pcm_index * sizeof(int16_t) - *ppos;
	if (count > bytes_left)
		count = bytes_left;
	if (copy_to_user(buffer, &data[*ppos], count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static const struct file_operations pcm_fops = {
	.owner = THIS_MODULE,
	.open = pcm_dev_open,
	.llseek = no_llseek,
	.read = pcm_dev_read,
};

static uint8_t raw_adpcm_buffer[640*1024];
static int raw_adpcm_index;
static struct miscdevice adpcm_dev_node;
static int adpcm_dev_open(struct inode *inode, struct file *file)
{
	/* nothing special to do here right now. */
	return 0;
}

static ssize_t adpcm_dev_read(struct file *file, char __user *buffer,
			  size_t count, loff_t *ppos)
{
	size_t bytes_left = raw_adpcm_index - *ppos;
	if (count > bytes_left)
		count = bytes_left;
	if (copy_to_user(buffer, &raw_adpcm_buffer[*ppos], count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static const struct file_operations adpcm_fops = {
	.owner = THIS_MODULE,
	.open = adpcm_dev_open,
	.llseek = no_llseek,
	.read = adpcm_dev_read,
};

static uint8_t raw_msbc_buffer[640*1024];
static int raw_msbc_index;
static struct miscdevice msbc_dev_node;
static int msbc_dev_open(struct inode *inode, struct file *file)
{
	/* nothing special to do here right now. */
	return 0;
}

static ssize_t msbc_dev_read(struct file *file, char __user *buffer,
			  size_t count, loff_t *ppos)
{
	size_t bytes_left = raw_msbc_index - *ppos;
	if (count > bytes_left)
		count = bytes_left;
	if (copy_to_user(buffer, &raw_msbc_buffer[*ppos], count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static const struct file_operations msbc_fops = {
	.owner = THIS_MODULE,
	.open = msbc_dev_open,
	.llseek = no_llseek,
	.read = msbc_dev_read,
};

#endif


struct simple_atomic_fifo {
	/* Read and write cursors are modified by different threads. */
	uint read_cursor;
	uint write_cursor;
	/* Size must be a power of two. */
	uint size;
	/* internal mask is 2*size - 1
	 * This allows us to tell the difference between full and empty. */
	uint internal_mask;
	uint external_mask;
};

struct snd_atvr {
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;

	uint32_t sample_rate;

	uint previous_jiffies; /* Used to detect underflows. */
	uint timeout_jiffies;
	struct timer_list decoding_timer;
	uint timer_state;
	uint32_t timer_enabled;
	uint timer_callback_count;

	int16_t peak_level;
	struct simple_atomic_fifo fifo_controller;
	struct fifo_packet *fifo_packet_buffer;

	/* IMA/DVI ADPCM Decoder */
	int pcm_value;
	int step_index;
	bool first_packet;

	/* msbc decoder */
	uint8_t msbc_frame_data[BYTES_PER_MSBC_FRAME];
	int16_t audio_output[MAX_SAMPLES_PER_PACKET];
	uint8_t packet_in_frame;
	uint8_t seq_index;

	/*
	 * Write_index is the circular buffer position.
	 * It is advanced by the BTLE thread after decoding.
	 * It is read by ALSA in snd_atvr_pcm_pointer().
	 * It is not declared volatile because that is not
	 * allowed in the Linux kernel.
	 */
	uint32_t write_index;
	uint32_t frames_per_buffer;
	/* count frames generated so far in this period */
	uint32_t frames_in_period;
	int16_t *pcm_buffer;

	/* pointer to hid device */
	struct hid_device *hdev;
	struct mutex hdev_lock;

	int card_index; /* sound card index */
	/* count of packets received from this device */
	int packet_counter;
	spinlock_t s_substream_lock;
	spinlock_t timer_lock;
	uint32_t substream_state;
	bool pcm_stopped;
};

#define ATVR_REMOVE 1
#define ATVR_TIMER_DISABLED 0
#define ATVR_TIMER_ENABLED 1

#define TS_HOSTCMD_REPORT_SIZE 33
#define JAR_HOSTCMD_REPORT_SIZE 19

static void atvr_hid_miss_stats_inc(void);

static int atvr_mic_ctrl(struct hid_device *hdev, bool enable)
{
	unsigned char report[TS_HOSTCMD_REPORT_SIZE] = {
		0x04, 0x0e, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00,
		};
	int ret;
	int report_size = JAR_HOSTCMD_REPORT_SIZE;

	report[3] = enable ? 0x01 : 0x00;
	hid_info(hdev, "%s remote mic\n", enable ? "enable" : "disable");

	/* for BLE devices (Pepper, Friday), send the message to userspace so that
	** gattservice can be used to send message to pepper.
	** this is to prevent concurrent mic ctrl msgs */

	if (hdev->product == USB_DEVICE_ID_NVIDIA_PEPPER ||
			hdev->product == USB_DEVICE_ID_NVIDIA_FRIDAY) {
		report[0] = 0x03;
		report[4] = 0x01;
		return hid_report_raw_event(hdev, 0, &report[0],
			report_size, 0);
	}

	if (hdev->product == USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE)
		report_size = TS_HOSTCMD_REPORT_SIZE;

	ret = hid_hw_output_report(hdev, report, report_size);
	if (ret == -ENOSYS)
		ret = hid_hw_raw_request(hdev, report[0], report,
			report_size, HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		hid_info(hdev, "failed to send mic ctrl report, err=%d\n", ret);
	else
		ret = 0;

	return ret;
}

int atvr_ts_sensor_set(struct hid_device *hdev, bool enable)
{
	u8 report[TS_HOSTCMD_REPORT_SIZE] = { 0x04, 0x5a };
	u8 sample_rate = 70;
	int ret;

	hid_info(hdev, "%s enable: %d\n",  __func__, enable);

	if (enable) {
		report[3] = 0x01; /* set */
		report[4] = 0x01; /* enable */
		report[5] = 0x00; /* real data source, 0x01 for dummy */
		report[6] = sample_rate;
		hid_info(hdev, "enable ts sensor %dHz\n", sample_rate);
	} else {
		report[3] = 0x01; /* set */
		hid_info(hdev, "disable ts sensor\n");
	}

	ret = hid_hw_output_report(hdev, report, TS_HOSTCMD_REPORT_SIZE);
	if (ret == -ENOSYS)
		ret = hid_hw_raw_request(hdev, report[0], report,
			TS_HOSTCMD_REPORT_SIZE, HID_OUTPUT_REPORT,
			HID_REQ_SET_REPORT);
	if (ret < 0)
		hid_info(hdev, "failed to send ts sensor ctrl report, err=%d\n",
				ret);
	else
		ret = 0;

	return ret;
}
EXPORT_SYMBOL(atvr_ts_sensor_set);

/***************************************************************************/
/************* Atomic FIFO *************************************************/
/***************************************************************************/
/*
 * This FIFO is atomic if used by no more than 2 threads.
 * One thread modifies the read cursor and the other
 * thread modifies the write_cursor.
 * Size and mask are not modified while being used.
 *
 * The read and write cursors range internally from 0 to (2*size)-1.
 * This allows us to tell the difference between full and empty.
 * When we get the cursors for external use we mask with size-1.
 *
 * Memory barriers required on SMP platforms.
 */
static int atomic_fifo_init(struct simple_atomic_fifo *fifo_ptr, uint size)
{
	/* Make sure size is a power of 2. */
	if ((size & (size-1)) != 0) {
		pr_err("%s:%d - ERROR FIFO size = %d, not power of 2!\n",
			__func__, __LINE__, size);
		return -EINVAL;
	}
	fifo_ptr->read_cursor = 0;
	fifo_ptr->write_cursor = 0;
	fifo_ptr->size = size;
	fifo_ptr->internal_mask = (size * 2) - 1;
	fifo_ptr->external_mask = size - 1;
	smp_wmb();
	return 0;
}


static uint atomic_fifo_available_to_read(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return (fifo_ptr->write_cursor - fifo_ptr->read_cursor)
			& fifo_ptr->internal_mask;
}

static uint atomic_fifo_available_to_write(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return fifo_ptr->size - atomic_fifo_available_to_read(fifo_ptr);
}

static void atomic_fifo_advance_read(
		struct simple_atomic_fifo *fifo_ptr,
		uint frames)
{
	smp_rmb();
	BUG_ON(frames > atomic_fifo_available_to_read(fifo_ptr));
	fifo_ptr->read_cursor = (fifo_ptr->read_cursor + frames)
			& fifo_ptr->internal_mask;
	smp_wmb();
}

static void atomic_fifo_advance_write(
		struct simple_atomic_fifo *fifo_ptr,
		uint frames)
{
	smp_rmb();
	BUG_ON(frames > atomic_fifo_available_to_write(fifo_ptr));
	fifo_ptr->write_cursor = (fifo_ptr->write_cursor + frames)
		& fifo_ptr->internal_mask;
	smp_wmb();
}

static uint atomic_fifo_get_read_index(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return fifo_ptr->read_cursor & fifo_ptr->external_mask;
}

static uint atomic_fifo_get_write_index(struct simple_atomic_fifo *fifo_ptr)
{
	smp_rmb();
	return fifo_ptr->write_cursor & fifo_ptr->external_mask;
}

/****************************************************************************/
static void snd_atvr_handle_frame_advance(
		struct snd_pcm_substream *substream, uint num_frames)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	atvr_snd->frames_in_period += num_frames;
	/* Tell ALSA if we have advanced by one or more periods. */
	if (atvr_snd->frames_in_period >= substream->runtime->period_size) {
		snd_pcm_period_elapsed(substream);
		atvr_snd->frames_in_period %= substream->runtime->period_size;
	}
}

static uint32_t snd_atvr_bump_write_index(
			struct snd_pcm_substream *substream,
			uint32_t num_samples)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	uint32_t pos = atvr_snd->write_index;

	/* Advance write position. */
	pos += num_samples;
	/* Wrap around at end of the circular buffer. */
	pos %= atvr_snd->frames_per_buffer;
	atvr_snd->write_index = pos;

	snd_atvr_handle_frame_advance(substream, num_samples);

	return pos;
}

/*
 * Decode an IMA/DVI ADPCM packet and write the PCM data into a circular buffer.
 * ADPCM is 4:1 16kHz@256kbps -> 16kHz@64kbps.
 * ADPCM is 4:1 8kHz@128kbps -> 8kHz@32kbps.
 */
static const int ima_index_table[16] = {
	-1, -1, -1, -1, /* +0 - +3, decrease the step size */
	2, 4, 6, 8,     /* +4 - +7, increase the step size */
	-1, -1, -1, -1, /* -0 - -3, decrease the step size */
	2, 4, 6, 8      /* -4 - -7, increase the step size */
};
static const int16_t ima_step_table[89] = {
	7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
	19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
	50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
	130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
	337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
	876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
	2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
	5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
	15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static void decode_adpcm_nibble(uint8_t nibble, struct snd_atvr *atvr_snd,
				struct snd_pcm_substream *substream)
{
	int step_index = atvr_snd->step_index;
	int value = atvr_snd->pcm_value;
	int step = ima_step_table[step_index];
	int diff;

	diff = step >> 3;
	if (nibble & 1)
		diff += (step >> 2);
	if (nibble & 2)
		diff += (step >> 1);
	if (nibble & 4)
		diff += step;

	if (nibble & 8) {
		value -= diff;
		if (value < -32768)
			value = -32768;
	} else {
		value += diff;
		if (value > 32767)
			value = 32767;
	}
	atvr_snd->pcm_value = value;

	/* copy to stream */
	atvr_snd->pcm_buffer[atvr_snd->write_index] = value;
#if (DEBUG_WITH_MISC_DEVICE == 1)
	if (large_pcm_index < ARRAY_SIZE(large_pcm_buffer))
		large_pcm_buffer[large_pcm_index++] = value;
#endif
	snd_atvr_bump_write_index(substream, 1);
	if (value > atvr_snd->peak_level)
		atvr_snd->peak_level = value;

	/* update step_index */
	step_index += ima_index_table[nibble];
	/* clamp step_index */
	if (step_index < 0)
		step_index = 0;
	else if (step_index >= ARRAY_SIZE(ima_step_table))
		step_index = ARRAY_SIZE(ima_step_table) - 1;
	atvr_snd->step_index = step_index;
}

static int snd_atvr_decode_adpcm_packet(
			struct snd_pcm_substream *substream,
			const uint8_t *adpcm_input,
			size_t num_bytes
			)
{
	uint i;
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);

	/* Decode IMA ADPCM data to PCM. */
	if (atvr_snd->first_packet) {
		/* the first two bytes of the first packet
		 * is the unencoded first 16-bit sample, high
		 * byte first.
		 */
		int value = ((int)adpcm_input[0] << 8) | adpcm_input[1];
		pr_info("%s: first packet, initial value is %d (0x%x, 0x%x)\n",
			__func__, value, adpcm_input[0], adpcm_input[1]);
		atvr_snd->pcm_value = value;
		atvr_snd->pcm_buffer[atvr_snd->write_index] = value;
#if (DEBUG_WITH_MISC_DEVICE == 1)
		if (raw_adpcm_index < ARRAY_SIZE(raw_adpcm_buffer))
			raw_adpcm_buffer[raw_adpcm_index++] = adpcm_input[0];
		if (raw_adpcm_index < ARRAY_SIZE(raw_adpcm_buffer))
			raw_adpcm_buffer[raw_adpcm_index++] = adpcm_input[1];
		if (large_pcm_index < ARRAY_SIZE(large_pcm_buffer))
			large_pcm_buffer[large_pcm_index++] = value;
#endif
		snd_atvr_bump_write_index(substream, 1);
		atvr_snd->peak_level = value;
		atvr_snd->first_packet = false;
		i = 2;
	} else {
		i = 0;
	}

	for (; i < num_bytes; i++) {
		uint8_t raw = adpcm_input[i];
		uint8_t nibble;

#if (DEBUG_WITH_MISC_DEVICE == 1)
		if (raw_adpcm_index < ARRAY_SIZE(raw_adpcm_buffer))
			raw_adpcm_buffer[raw_adpcm_index++] = raw;
#endif

		/* process first nibble */
		nibble = (raw >> 4) & 0x0f;
		decode_adpcm_nibble(nibble, atvr_snd, substream);

		/* process second nibble */
		nibble = raw & 0x0f;
		decode_adpcm_nibble(nibble, atvr_snd, substream);
	}

	return num_bytes * 2;
}

/*
 * Decode an mSBC packet and write the PCM data into a circular buffer.
 */
#define BLOCKS_PER_PACKET 15
#define NUM_BITS 26

static int snd_atvr_decode_msbc_packet(
			struct snd_pcm_substream *substream,
			const uint8_t *sbc_input,
			size_t num_bytes
			)
{
	uint num_samples = 0;
	uint remaining;
	uint i;
	uint32_t pos;
	uint read_index;
	uint write_index;
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	if (num_bytes < BYTES_PER_MSBC_FRAME) {
		/* assume we have a BLE frame that needs to be reconstructed */
		if (atvr_snd->packet_in_frame == 0) {
			if (sbc_input[0] !=
				msbc_sequence_table[atvr_snd->seq_index]) {

				snd_atvr_log(
				"sequence_num err, 0x%02x != 0x%02x\n",
				sbc_input[0],
				msbc_sequence_table[atvr_snd->seq_index]);

				return 0;
			}
			atvr_snd->seq_index++;
			if (atvr_snd->seq_index == NUM_SEQUENCES)
				atvr_snd->seq_index = 0;

			/* subtract the sequence number */
			num_bytes--;
		}
		if (num_bytes !=
			msbc_bytes_in_packet[atvr_snd->packet_in_frame]) {

			pr_err(
			 "%s: received %zd audio bytes but expected %d bytes\n",
			  __func__, num_bytes,
			  msbc_bytes_in_packet[atvr_snd->packet_in_frame]);

			return 0;
		}
		write_index =
			msbc_start_offset_in_buffer[atvr_snd->packet_in_frame];
		read_index =
			msbc_start_offset_in_packet[atvr_snd->packet_in_frame];
		memcpy(&atvr_snd->msbc_frame_data[write_index],
			   &sbc_input[read_index],
			   msbc_bytes_in_packet[atvr_snd->packet_in_frame]);
		atvr_snd->packet_in_frame++;
		if (atvr_snd->packet_in_frame < BLE_PACKETS_PER_MSBC_FRAME) {
			/* we don't have a complete mSBC frame yet,
			 * just return */
			return 0;
		}
		/* reset for next mSBC frame */
		atvr_snd->packet_in_frame = 0;
		/* we have a complete mSBC frame, send it to the decoder */
		num_samples = sbc_decode(BLOCKS_PER_PACKET, NUM_BITS,
					 atvr_snd->msbc_frame_data,
					 BYTES_PER_MSBC_FRAME,
					 &atvr_snd->audio_output[0]);
	} else {
		/* we have a complete mSBC frame, send it to the decoder */
		num_samples = sbc_decode(BLOCKS_PER_PACKET, NUM_BITS,
					 sbc_input,
					 BYTES_PER_MSBC_FRAME,
					 &atvr_snd->audio_output[0]);
	}
	/* Write PCM data to the buffer. */
	pos = atvr_snd->write_index;
	read_index = 0;
	if ((pos + num_samples) > atvr_snd->frames_per_buffer) {
		for (i = pos; i < atvr_snd->frames_per_buffer; i++) {
			int16_t sample = atvr_snd->audio_output[read_index++];
			if (sample > atvr_snd->peak_level)
				atvr_snd->peak_level = sample;
			atvr_snd->pcm_buffer[i] = sample;
		}

		remaining = (pos + num_samples) - atvr_snd->frames_per_buffer;
		for (i = 0; i < remaining; i++) {
			int16_t sample = atvr_snd->audio_output[read_index++];
			if (sample > atvr_snd->peak_level)
				atvr_snd->peak_level = sample;
			atvr_snd->pcm_buffer[i] = sample;
		}

	} else {
		for (i = 0; i < num_samples; i++) {
			int16_t sample = atvr_snd->audio_output[read_index++];
			if (sample > atvr_snd->peak_level)
				atvr_snd->peak_level = sample;
			atvr_snd->pcm_buffer[i + pos] = sample;
		}
	}

	snd_atvr_bump_write_index(substream, num_samples);

	return num_samples;
}

static int snd_atvr_alloc_audio_buffs(struct snd_atvr *atvr_snd)
{
	int ret;

	ret = atomic_fifo_init(&atvr_snd->fifo_controller,
				   MAX_PACKETS_PER_BUFFER);
	if (ret)
		return ret;

	/*
	 * Allocate the maximum buffer now and then just use part of it when
	 * the substream starts. We don't need DMA because it will just
	 * get written to by the BTLE code.
	 */

	if (atvr_snd->pcm_buffer == NULL)
		atvr_snd->pcm_buffer = vmalloc(MAX_PCM_BUFFER_SIZE);
	if (atvr_snd->pcm_buffer == NULL) {
		pr_err("%s:%d - ERROR PCM buffer allocation failed\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	if (atvr_snd->fifo_packet_buffer == NULL)
		atvr_snd->fifo_packet_buffer = vmalloc(MAX_BUFFER_SIZE);
	if (atvr_snd->fifo_packet_buffer == NULL) {
		pr_err("%s:%d - ERROR buffer allocation failed\n",
			__func__, __LINE__);
		vfree(atvr_snd->pcm_buffer);
		atvr_snd->pcm_buffer = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void snd_atvr_dealloc_audio_buffs(struct snd_atvr *atvr_snd)
{
	vfree(atvr_snd->pcm_buffer);
	vfree(atvr_snd->fifo_packet_buffer);
}

/**
 * This is called by the event filter when it gets an audio packet
 * from the AndroidTV remote.  It writes the packet into a FIFO
 * which is then read and decoded by the timer task.
 * @param input pointer to data to be decoded
 * @param num_bytes how many bytes in raw_input
 * @return number of samples decoded or negative error.
 */
static void audio_dec(struct hid_device *hdev, const uint8_t *raw_input,
						int type, size_t num_bytes)
{
	bool dropped_packet = false;
	struct shdr_device *shdr_dev = hid_get_drvdata(hdev);
	struct snd_card *shdr_card;
	struct snd_atvr *atvr_snd;
	unsigned long flags;
	uint writable;

	if (shdr_dev == NULL)
		return;
	shdr_card = shdr_dev->shdr_card;

	if (shdr_card == NULL)
		return;

	atvr_snd = shdr_card->private_data;

	smp_rmb();
	if (atvr_snd != NULL && atvr_snd->pcm_stopped == false) {
		spin_lock_irqsave(&atvr_snd->s_substream_lock, flags);

		if (atvr_snd->substream_state & ATVR_REMOVE) {
			spin_unlock_irqrestore(&atvr_snd->s_substream_lock,
					flags);
			return;
		}

		/* Write data to a FIFO for decoding by the timer task. */
		writable = atomic_fifo_available_to_write(
			&atvr_snd->fifo_controller);
		if (writable > 0) {
			uint fifo_index = atomic_fifo_get_write_index(
				&atvr_snd->fifo_controller);
			struct fifo_packet *packet =
				&atvr_snd->fifo_packet_buffer[fifo_index];
			packet->type = type;
			packet->num_bytes = (uint8_t)num_bytes;
			memcpy(packet->raw_data, raw_input, num_bytes);
			atomic_fifo_advance_write(
				&atvr_snd->fifo_controller, 1);
		} else {
			dropped_packet = true;
			atvr_snd->pcm_stopped = true;
			smp_wmb();
		}
		atvr_snd->packet_counter++;
		spin_unlock_irqrestore(&atvr_snd->s_substream_lock, flags);
	}

	if (dropped_packet)
		snd_atvr_log("WARNING, raw audio packet dropped, FIFO full\n");
}

/*
 * Note that smp_rmb() is called by snd_atvr_timer_callback()
 * before calling this function.
 *
 * Reads:
 *    jiffies
 *    atvr_snd->previous_jiffies
 * Writes:
 *    atvr_snd->previous_jiffies
 * Returns:
 *    num_frames needed to catch up to the current time
 */
static uint snd_atvr_calc_frame_advance(struct snd_atvr *atvr_snd)
{
	/* Determine how much time passed. */
	uint now_jiffies = jiffies;
	uint elapsed_jiffies = now_jiffies - atvr_snd->previous_jiffies;
	/* Convert jiffies to frames. */
	uint frames_by_time = jiffies_to_msecs(elapsed_jiffies)
		* atvr_snd->sample_rate / 1000;
	atvr_snd->previous_jiffies = now_jiffies;

	/* Don't write more than one buffer full. */
	if (frames_by_time > (atvr_snd->frames_per_buffer - 4))
		frames_by_time  = atvr_snd->frames_per_buffer - 4;

	return frames_by_time;
}

/* Write zeros into the PCM buffer. */
static uint32_t snd_atvr_write_silence(struct snd_atvr *atvr_snd,
			uint32_t pos,
			int frames_to_advance)
{
	/* Does it wrap? */
	if ((pos + frames_to_advance) > atvr_snd->frames_per_buffer) {
		/* Write to end of buffer. */
		int16_t *destination = &atvr_snd->pcm_buffer[pos];
		size_t num_frames = atvr_snd->frames_per_buffer - pos;
		size_t num_bytes = num_frames * sizeof(int16_t);
		memset(destination, 0, num_bytes);
		/* Write from start of buffer to new pos. */
		destination = &atvr_snd->pcm_buffer[0];
		num_frames = frames_to_advance - num_frames;
		num_bytes = num_frames * sizeof(int16_t);
		memset(destination, 0, num_bytes);
	} else {
		/* Write within the buffer. */
		int16_t *destination = &atvr_snd->pcm_buffer[pos];
		size_t num_bytes = frames_to_advance * sizeof(int16_t);
		memset(destination, 0, num_bytes);
	}
	/* Advance and wrap write_index */
	pos += frames_to_advance;
	pos %= atvr_snd->frames_per_buffer;
	return pos;
}

/*
 * Called by timer task to decode raw audio data from the FIFO into the PCM
 * buffer.  Returns the number of packets decoded.
 */
static uint snd_atvr_decode_from_fifo(struct snd_pcm_substream *substream)
{
	uint i;
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	uint readable = atomic_fifo_available_to_read(
		&atvr_snd->fifo_controller);
	for (i = 0; i < readable; i++) {
		uint fifo_index = atomic_fifo_get_read_index(
			&atvr_snd->fifo_controller);
		struct fifo_packet *packet =
			&atvr_snd->fifo_packet_buffer[fifo_index];
		if (packet->type == PACKET_TYPE_ADPCM) {
			snd_atvr_decode_adpcm_packet(substream,
						     packet->raw_data,
						     packet->num_bytes);
		} else if (packet->type == PACKET_TYPE_MSBC) {
			snd_atvr_decode_msbc_packet(substream,
							 packet->raw_data,
							 packet->num_bytes);
		} else {
			pr_err("Unknown packet type %d\n", packet->type);
		}

		atomic_fifo_advance_read(&atvr_snd->fifo_controller, 1);
	}

	return readable;
}

static int snd_atvr_schedule_timer(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	uint msec_to_sleep = (substream->runtime->period_size * 1000)
			/ atvr_snd->sample_rate;
	uint jiffies_to_sleep = msecs_to_jiffies(msec_to_sleep);
	if (jiffies_to_sleep < 2)
		jiffies_to_sleep = 2;
	ret = mod_timer(&atvr_snd->decoding_timer, jiffies + jiffies_to_sleep);
	if (ret < 0)
		pr_err("%s:%d - ERROR in mod_timer, ret = %d\n",
			   __func__, __LINE__, ret);
	return ret;
}

static void snd_atvr_timer_callback(unsigned long data)
{
	uint readable;
	uint packets_read;
	bool need_silence = false;
	unsigned long flags;
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
#ifdef DEBUG_TIMER
	struct timeval t0, t1;
	int diff;
#endif

	/* timer_enabled will be false when stopping a stream. */
	spin_lock_irqsave(&atvr_snd->timer_lock, flags);
	if (!(atvr_snd->timer_enabled & ATVR_TIMER_ENABLED)) {
		spin_unlock_irqrestore(&atvr_snd->timer_lock, flags);
		return;
	}

	if (silence_counter > MAX_SILENCE_COUNTER) {
		spin_unlock_irqrestore(&atvr_snd->timer_lock, flags);
		printk_ratelimited(KERN_INFO "max silence timeout reached.\n");
		return;
	}
	spin_unlock_irqrestore(&atvr_snd->timer_lock, flags);

	spin_lock_irqsave(&atvr_snd->s_substream_lock, flags);
	if (atvr_snd->substream_state & ATVR_REMOVE) {
		spin_unlock_irqrestore(&atvr_snd->s_substream_lock, flags);
		return;
	}

#ifdef DEBUG_TIMER
	do_gettimeofday(&t0);
#endif
	atvr_snd->timer_callback_count++;

	switch (atvr_snd->timer_state) {
	case TIMER_STATE_BEFORE_DECODE:
		readable = atomic_fifo_available_to_read(
				&atvr_snd->fifo_controller);
		if (readable > 0) {
			atvr_snd->timer_state = TIMER_STATE_DURING_DECODE;
			/* Fall through into next state. */
		} else {
			need_silence = true;
			break;
		}

	case TIMER_STATE_DURING_DECODE:
		packets_read = snd_atvr_decode_from_fifo(substream);

		if (packets_read > 0) {
			/* Defer timeout */
			atvr_snd->previous_jiffies = jiffies;
			break;
		}

		smp_rmb();

		if (atvr_snd->pcm_stopped) {
			atvr_snd->timer_state = TIMER_STATE_AFTER_DECODE;
			/* Decoder died. Overflowed?
			 * Fall through into next state. */
		} else if ((jiffies - atvr_snd->previous_jiffies) >
			   atvr_snd->timeout_jiffies) {
			pr_debug("%s: audio UNDERFLOW detected\n", __func__);
			/*  Not fatal.  Reset timeout. */
			atvr_snd->previous_jiffies = jiffies;
			break;
		} else
			break;

	case TIMER_STATE_AFTER_DECODE:
		need_silence = true;
		break;
	}

	/* Write silence before and after decoding. */
	if (need_silence) {
		uint frames_to_silence = snd_atvr_calc_frame_advance(atvr_snd);
		atvr_snd->write_index = snd_atvr_write_silence(
				atvr_snd,
				atvr_snd->write_index,
				frames_to_silence);
#ifdef DEBUG_TIMER
		do_gettimeofday(&t1);
#endif
		spin_unlock_irqrestore(&atvr_snd->s_substream_lock, flags);
		/* This can cause snd_atvr_pcm_trigger() to be called, which
		 * may try to stop the timer. */
		snd_atvr_handle_frame_advance(substream, frames_to_silence);
	} else {
#ifdef DEBUG_TIMER
		do_gettimeofday(&t1);
#endif
		spin_unlock_irqrestore(&atvr_snd->s_substream_lock, flags);
	}
#ifdef DEBUG_TIMER
	/* diff has unit of ms */
	diff = (t1.tv_sec - t0.tv_sec) * 1000 +
		(t1.tv_usec - t0.tv_usec) / 1000;
	if (diff >= 3)
		pr_err("callback took %d ms\n", diff);
#endif

	spin_lock_irqsave(&atvr_snd->timer_lock, flags);
	if (need_silence)
		silence_counter += 1;
	else
		silence_counter = 0;
	if (atvr_snd->timer_enabled & ATVR_TIMER_ENABLED)
		snd_atvr_schedule_timer(substream);
	spin_unlock_irqrestore(&atvr_snd->timer_lock, flags);
}

static int snd_atvr_timer_start(struct snd_pcm_substream *substream)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	unsigned long flags;

	spin_lock_irqsave(&atvr_snd->timer_lock, flags);
	atvr_snd->timer_enabled = ATVR_TIMER_ENABLED;
	silence_counter = 0;
	spin_unlock_irqrestore(&atvr_snd->timer_lock, flags);

	atvr_snd->previous_jiffies = jiffies;
	atvr_snd->timeout_jiffies =
		msecs_to_jiffies(SND_ATVR_RUNNING_TIMEOUT_MSEC);
	atvr_snd->timer_callback_count = 0;

	snd_atvr_schedule_timer(substream);
	return 0;
}

static void snd_atvr_timer_stop(struct snd_pcm_substream *substream)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	unsigned long flags;

	spin_lock_irqsave(&atvr_snd->timer_lock, flags);
	silence_counter = 0;
	if (atvr_snd->timer_enabled)
		atvr_snd->timer_enabled = ATVR_TIMER_DISABLED;
	spin_unlock_irqrestore(&atvr_snd->timer_lock, flags);
}

/* ===================================================================== */
/*
 * PCM interface
 */

static int snd_atvr_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
#if (DEBUG_AUDIO_TX == 1)
		snd_atvr_log("%s starting audio\n", __func__);
#endif

#if (DEBUG_WITH_MISC_DEVICE == 1)
		large_pcm_index = 0;
		raw_adpcm_index = 0;
		raw_msbc_index = 0;
#endif
		atvr_snd->packet_counter = 0;
		atvr_snd->peak_level = -32768;
		atvr_snd->previous_jiffies = jiffies;
		atvr_snd->timer_state = TIMER_STATE_BEFORE_DECODE;

		/* ADPCM decoder state */
		atvr_snd->step_index = 0;
		atvr_snd->pcm_value = 0;
		atvr_snd->first_packet = true;

		/* msbc decoder */
		atvr_snd->packet_in_frame = 0;
		atvr_snd->seq_index = 0;

		atvr_snd->pcm_stopped = false;
		smp_wmb();
		snd_atvr_timer_start(substream);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
#if (DEBUG_AUDIO_TX == 1)
		snd_atvr_log("%s stopping audio, peak = %d, # packets = %d\n",
			__func__, atvr_snd->peak_level,
			atvr_snd->packet_counter);
#endif
		atvr_snd->pcm_stopped = true;
		smp_wmb();
		snd_atvr_timer_stop(substream);
		return 0;
	}
	return -EINVAL;
}

static int snd_atvr_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
#if (DEBUG_AUDIO_TX == 1)
	snd_atvr_log("%s, rate = %d, period_size = %d, buffer_size = %d\n",
		__func__, (int) runtime->rate,
		(int) runtime->period_size,
		(int) runtime->buffer_size);
#endif
	if (runtime->buffer_size > MAX_FRAMES_PER_BUFFER)
		return -EINVAL;

	atvr_snd->sample_rate = runtime->rate;
	atvr_snd->frames_per_buffer = runtime->buffer_size;

	return 0; /* TODO - review */
}

static struct snd_pcm_hardware atvr_pcm_hardware = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		USE_FORMATS,
	.rates =		USE_RATES_MASK,
	.rate_min =		USE_RATE_MIN,
	.rate_max =		USE_RATE_MAX,
	.channels_min =		USE_CHANNELS_MIN,
	.channels_max =		USE_CHANNELS_MAX,
	.buffer_bytes_max =	MAX_PCM_BUFFER_SIZE,
	.period_bytes_min =	MIN_PERIOD_SIZE,
	.period_bytes_max =	MAX_PERIOD_SIZE,
	.periods_min =		USE_PERIODS_MIN,
	.periods_max =		USE_PERIODS_MAX,
	.fifo_size =		0,
};

static struct snd_pcm_hardware atvr_pcm_hardware_pepper = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		USE_FORMATS,
	.rates =		USE_RATES_MASK_PEPPER,
	.rate_min =		USE_RATE_MIN,
	.rate_max =		USE_RATE_MAX_PEPPER,
	.channels_min =		USE_CHANNELS_MIN,
	.channels_max =		USE_CHANNELS_MAX_PEPPER,
	.buffer_bytes_max =	MAX_PCM_BUFFER_SIZE,
	.period_bytes_min =	MIN_PERIOD_SIZE,
	.period_bytes_max =	MAX_PERIOD_SIZE,
	.periods_min =		USE_PERIODS_MIN,
	.periods_max =		USE_PERIODS_MAX,
	.fifo_size =		0,
};

static int snd_atvr_pcm_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *hw_params)
{
	int ret = 0;
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);

	atvr_snd->write_index = 0;
	smp_wmb();

	return ret;
}

static int snd_atvr_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static int snd_atvr_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	mutex_lock(&atvr_snd->hdev_lock);
	if (atvr_snd->hdev == NULL) {
		pr_warn("%s: remote is not ready\n", __func__);
		mutex_unlock(&atvr_snd->hdev_lock);
		return -EAGAIN;
	}

	ret = atvr_mic_ctrl(atvr_snd->hdev, true);
	mutex_unlock(&atvr_snd->hdev_lock);

	if (ret)
		return ret;

	runtime->hw = atvr_snd->pcm_hw;
	if (substream->pcm->device & 1) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
		runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	}
	if (substream->pcm->device & 2)
		runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP
			| SNDRV_PCM_INFO_MMAP_VALID);

#ifdef DEBUG_TIMER
	snd_atvr_log("%s, built %s %s\n", __func__, __DATE__, __TIME__);
#endif
	/* Initialize the timer for the opened substream */
	setup_timer(&atvr_snd->decoding_timer, snd_atvr_timer_callback,
		    (unsigned long)substream);

	return ret;
}

static int snd_atvr_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	int ret;

	snd_atvr_timer_stop(substream);
	del_timer_sync(&atvr_snd->decoding_timer);

#ifdef DEBUG_TIMER
	if (atvr_snd->timer_callback_count > 0)
		snd_atvr_log("processed %d packets in %d timer callbacks\n",
			atvr_snd->packet_counter,
			atvr_snd->timer_callback_count);
#endif

	ret = atomic_fifo_init(&atvr_snd->fifo_controller,
				   MAX_PACKETS_PER_BUFFER);
	if (ret)
		return ret;

	mutex_lock(&atvr_snd->hdev_lock);
	if (atvr_snd->hdev)
		atvr_mic_ctrl(atvr_snd->hdev, false);
	else
		pr_warn("%s: unexpected remote connection lost\n", __func__);
	mutex_unlock(&atvr_snd->hdev_lock);

	return 0;
}

static snd_pcm_uframes_t snd_atvr_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);
	/* write_index is written by another driver thread */
	smp_rmb();
	return atvr_snd->write_index;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static int snd_atvr_pcm_copy(struct snd_pcm_substream *substream, int channel,
			unsigned long pos, void __user *dst,
			unsigned long count)
#else
static int snd_atvr_pcm_copy(struct snd_pcm_substream *substream,
			int channel, snd_pcm_uframes_t pos,
			void __user *dst, snd_pcm_uframes_t count)
#endif
{
	struct snd_atvr *atvr_snd = snd_pcm_substream_chip(substream);

	/* TODO Needs to be modified if we support more than 1 channel. */
	/*
	 * Copy from PCM buffer to user memory.
	 * Are we reading past the end of the buffer?
	 */
	if ((pos + count) > atvr_snd->frames_per_buffer) {
		const int16_t *source = &atvr_snd->pcm_buffer[pos];
		int16_t __user *destination = dst;
		size_t num_frames = atvr_snd->frames_per_buffer - pos;
		size_t num_bytes = num_frames * sizeof(int16_t);
		if (copy_to_user(destination, source, num_bytes))
			return -EFAULT;

		source = &atvr_snd->pcm_buffer[0];
		destination += num_frames;
		num_frames = count - num_frames;
		num_bytes = num_frames * sizeof(int16_t);
		if (copy_to_user(destination, source, num_bytes))
			return -EFAULT;
	} else {
		const int16_t *source = &atvr_snd->pcm_buffer[pos];
		int16_t __user *destination = dst;
		size_t num_bytes = count * sizeof(int16_t);
		if (copy_to_user(destination, source, num_bytes))
			return -EFAULT;
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static int snd_atvr_pcm_silence(struct snd_pcm_substream *substream, int channel,
				unsigned long pos, unsigned long count)
#else
static int snd_atvr_pcm_silence(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos,
				snd_pcm_uframes_t count)
#endif
{
	return 0; /* Do nothing. Only used by output? */
}

static struct snd_pcm_ops snd_atvr_pcm_ops_no_buf = {
	.open =		snd_atvr_pcm_open,
	.close =	snd_atvr_pcm_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_atvr_pcm_hw_params,
	.hw_free =	snd_atvr_pcm_hw_free,
	.prepare =	snd_atvr_pcm_prepare,
	.trigger =	snd_atvr_pcm_trigger,
	.pointer =	snd_atvr_pcm_pointer,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	.copy_user =	snd_atvr_pcm_copy,
	.fill_silence = snd_atvr_pcm_silence,
#else
	.copy =		snd_atvr_pcm_copy,
	.silence =	snd_atvr_pcm_silence,
#endif
};

static int snd_card_atvr_pcm(struct snd_atvr *atvr_snd,
			     int device,
			     int substreams)
{
	struct snd_pcm *pcm;
	struct snd_pcm_ops *ops;
	int err;

	err = snd_pcm_new(atvr_snd->card, "SHDR PCM", device,
			  0, /* no playback substreams */
			  1, /* 1 capture substream */
			  &pcm);
	if (err < 0)
		return err;
	atvr_snd->pcm = pcm;
	ops = &snd_atvr_pcm_ops_no_buf;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, ops);
	pcm->private_data = atvr_snd;
	pcm->info_flags = 0;
	strlcpy(pcm->name, "SHDR PCM", sizeof(pcm->name));

	return 0;
}

static int atvr_snd_initialize(struct hid_device *hdev,
			struct snd_card **p_shdr_card)
{
	struct snd_atvr *atvr_snd;
	struct snd_card *shdr_card;
	int err;
	int i;
	int dev = 0;
	struct hid_input *hidinput = list_first_entry_or_null(&hdev->inputs,
			struct hid_input, list);
	struct input_dev *shdr_input_dev;

	if (hidinput == NULL)
		return -ENODEV;

	shdr_input_dev = hidinput->input;

	while (dev < SNDRV_CARDS && cards_in_use[dev])
		dev++;

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		return -ENOENT;
	}
	cards_in_use[dev] = true;

	err = snd_card_new(&hdev->dev, index[dev], id[dev], THIS_MODULE,
			   sizeof(struct snd_atvr), &shdr_card);
	if (err < 0) {
		pr_err("%s: snd_card_new() returned err %d\n",
		       __func__, err);
		cards_in_use[dev] = false;
		return err;
	}
	*p_shdr_card = shdr_card;
	atvr_snd = shdr_card->private_data;
	atvr_snd->card = shdr_card;
	atvr_snd->card_index = dev;
	mutex_init(&atvr_snd->hdev_lock);
	spin_lock_init(&atvr_snd->s_substream_lock);
	spin_lock_init(&atvr_snd->timer_lock);
	atvr_snd->substream_state = 0;
	err = snd_atvr_alloc_audio_buffs(atvr_snd);

	if (err)
		goto __nodev;
	/* dummy initialization */
	setup_timer(&atvr_snd->decoding_timer,
		snd_atvr_timer_callback, 0);

	for (i = 0; i < MAX_PCM_DEVICES && i < pcm_devs[dev]; i++) {
		if (pcm_substreams[dev] < 1)
			pcm_substreams[dev] = 1;
		if (pcm_substreams[dev] > MAX_PCM_SUBSTREAMS)
			pcm_substreams[dev] = MAX_PCM_SUBSTREAMS;
		err = snd_card_atvr_pcm(atvr_snd, i, pcm_substreams[dev]);
		if (err < 0) {
			pr_err("%s: snd_card_atvr_pcm() returned err %d\n",
			       __func__, err);
			goto __nodev;
		}
	}

	if (hdev->product == USB_DEVICE_ID_NVIDIA_PEPPER ||
			hdev->product == USB_DEVICE_ID_NVIDIA_FRIDAY)
		atvr_snd->pcm_hw = atvr_pcm_hardware_pepper;
	else
		atvr_snd->pcm_hw = atvr_pcm_hardware;

	strlcpy(shdr_card->driver, "SHIELD Rmt Aud", sizeof(shdr_card->driver));
	strlcpy(shdr_card->shortname, "SHDRAudio",
						sizeof(shdr_card->shortname));
	sprintf(shdr_card->longname, "SHIELD Remote %i audio", dev + 1);

	err = snd_card_register(shdr_card);

	if (err)
		goto __nodev;

	return 0;

__nodev:
	snd_card_free(shdr_card);
	*p_shdr_card = NULL;
	cards_in_use[dev] = false;
	return err;
}

#define JAR_BUTTON_REPORT_ID	0x01
#define JAR_BUTTON_REPORT_SIZE	3

#define JAR_AUDIO_REPORT_ID	0xFD
#define JAR_AUDIO_REPORT_SIZE	233
#define JAR_AUDIO_FRAME_SIZE	0x3A

#define TS_BUTTON_REPORT_SIZE 19

#define PEP_BUTTON_REPORT_ID	0x2
#define PEP_BUTTON_REPORT_SIZE	3

static void atvr_pepper_button_release(struct work_struct *work)
{
	struct shdr_device *shdr_dev =
		container_of(work, struct shdr_device, hid_miss_war_work.work);
	u8 fake_button_up[PEP_BUTTON_REPORT_SIZE] = {
		PEP_BUTTON_REPORT_ID, 0x0, 0x0 };

	hid_report_raw_event(shdr_dev->hdev, 0, fake_button_up,
			     sizeof(fake_button_up), 0);
	atvr_hid_miss_stats_inc();
}

static int atvr_jarvis_break_events(struct hid_device *hdev,
				    struct hid_report *report,
				    u8 *data, int size)
{
	struct shdr_device *shdr_dev = hid_get_drvdata(hdev);
	unsigned int button_report_id = JAR_BUTTON_REPORT_ID;
	unsigned int button_report_size = JAR_BUTTON_REPORT_SIZE;
	unsigned int audio_report_id = JAR_AUDIO_REPORT_ID;
	unsigned int audio_report_size = JAR_AUDIO_REPORT_SIZE;

	/* breaks events apart if they are not proper */
	pr_debug("%s: packet 0x%02x#%i\n", __func__, data[0], size);

	/*
	 * break the ts events also similarly,
	 * just account for size differences
	 */
	if (hdev->product == USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE)
		button_report_size = TS_HOSTCMD_REPORT_SIZE;

	if (hdev->product == USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE &&
	    report->id == button_report_id) {
		shdr_dev->last_ljsx = (data[10] << 8) | data[9];
		shdr_dev->last_ljsy = (data[12] << 8) | data[11];
		shdr_dev->last_rjsx = (data[14] << 8) | data[13];
		shdr_dev->last_rjsy = (data[16] << 8) | data[15];
	}

	if ((hdev->product == USB_DEVICE_ID_NVIDIA_PEPPER ||
				hdev->product == USB_DEVICE_ID_NVIDIA_FRIDAY) &&
	    report->id == PEP_BUTTON_REPORT_ID) {
		int timeout;

		mutex_lock(&shdr_dev->hid_miss_war_lock);
		timeout = shdr_dev->hid_miss_war_timeout;
		mutex_unlock(&shdr_dev->hid_miss_war_lock);
		if (timeout <= 0)
			return 0;

		/*
		 * data[1] & data[2] will be set whenever a button is pressed
		 * and will be all zero when no button is pressed
		 * For Pepper, data[1] == 0 && data[2] == 0x80 indicates we are
		 * sending a repeat of previous HID event. We should ignore
		 * this case. For Friday, we don't need to ignore this case as
		 * there is no repeat bit.
		 */
		if (data[1] == 0 && data[2] == 0)
			cancel_delayed_work_sync(&shdr_dev->hid_miss_war_work);
		else if (hdev->product == USB_DEVICE_ID_NVIDIA_FRIDAY ||
			 (data[1] != 0 || data[2] != 0x80)) {
			cancel_delayed_work_sync(&shdr_dev->hid_miss_war_work);
			schedule_delayed_work(&shdr_dev->hid_miss_war_work,
					      msecs_to_jiffies(timeout));
		}
		return 0;
	}

	if (!((report->id == button_report_id &&
	       size >= button_report_size) ||
	      (report->id == audio_report_id &&
	       size >= audio_report_size)))
		return 0;

	/*
	 * This is a WAR for a CSR issue at the moment
	 * (packets get put together)
	 */
	while (size > 0) {
		int amount = 0;
		if ((data[0] == button_report_id) &&
		    (size >= button_report_size)) {
			amount = button_report_size;
			hid_report_raw_event(hdev, 0, data,
					     amount, 0);
		} else if ((data[0] == audio_report_id) &&
			   (size >= audio_report_size)) {
			u8 *frame = &data[1];
			amount = audio_report_size;
			while (frame < &data[JAR_AUDIO_REPORT_SIZE]) {
				audio_dec(hdev, &frame[0], PACKET_TYPE_MSBC,
					  JAR_AUDIO_FRAME_SIZE);
				frame = &frame[JAR_AUDIO_FRAME_SIZE];
			}
		} else {
			pr_info("%s: unknown id or broken packet 0x%02x#%i\n",
				__func__, data[0], size);
			break;
		}
		/* skip over the HID_DATP indicator */
		if (size > amount)
			amount++;
		data += amount;
		size -= amount;
	}
	return 1;
}

static void atvr_set_hid_debug_report_idx(struct shdr_device *shdr_dev, u8 id)
{
	int i = 0;

	for (i = 0; i < MAX_DEBUG_REPORTS; i++) {
		if (!shdr_dev->debug_info[i].id) {
			pr_debug("%s: report id %d set at index %d", __func__
								  , id, i);
			shdr_dev->debug_info[i].id = id;
			break;
		} else if (shdr_dev->debug_info[i].id == id)
			break;
	}
}

static u8 atvr_get_debug_report_idx(struct shdr_device *shdr_dev, u8 id)
{
	int i = 0;

	for (i = 0; i < MAX_DEBUG_REPORTS; i++) {
		if (shdr_dev->debug_info[i].id) {
			if (shdr_dev->debug_info[i].id == id) {
				pr_debug("%s: report id %d index is %d", __func__
								  , id, i);
				return i;
			}
		} else
			break;
	}
	return MAX_DEBUG_REPORTS;
}

static void atvr_process_debug_info(struct hid_debug_data *debug_info, u8 seq,
				    u16 fw_time_diff, u8 id)
{
	/* Check Seq Number */
	ktime_t current_time = ktime_get();
	s64 time_diff;
	u16 packet_delay;

	time_diff = ktime_ms_delta(current_time,
			debug_info->time);
	packet_delay = abs(time_diff - fw_time_diff);

	pr_debug("%s:report->id = 0x%x,seq %d fw_diff %d, drv_diff %d, latency %d",
			__func__, id, seq, fw_time_diff,
			(unsigned int)time_diff, packet_delay);

	if ((u8)(debug_info->seq_num + 1) != seq)
		pr_warn("%s: id:%d seq num missed Prev %d curr %d",
				__func__, id,
				debug_info->seq_num, seq);

	/* Ignore first packet time diff */
	if (debug_info->time.tv64 &&
			time_diff < MAX_TIME_BETWEEN_PACKETS &&
			packet_delay > MAX_PACKET_DIFF_TOLERANCE)
		pr_warn("%s: id:%d Packet delay:%d ms at seq %d host diff :%lli ms fw diff %d",
				__func__, id, packet_delay, seq,
				time_diff, fw_time_diff);
	debug_info->seq_num = seq;
	debug_info->time = current_time;
}

static int atvr_raw_event(struct hid_device *hdev, struct hid_report *report,
	u8 *data, int size)
{
	struct shdr_device *shdr_dev = hid_get_drvdata(hdev);
	struct snd_card *shdr_card = shdr_dev->shdr_card;
	struct snd_atvr *atvr_snd;
	void *debug_info;
	u8 idx = atvr_get_debug_report_idx(shdr_dev, report->id);
	/*first byte is seq num and next 2 bytes time diff */

	if (shdr_card == NULL)
		return 0;

	/* debug info is present and Min size check */
	if (idx < MAX_DEBUG_REPORTS && size > 4) {
		/*
		 * out of last 3 bytes,
		 * 1st byte is seq num
		 * next 2 bytes is time diff
		 */
		u8 seq = data[size - 3];
		u16 fw_time_diff = data[size - 2] | (data[size - 1] << 8);

		atvr_process_debug_info(&shdr_dev->debug_info[idx], seq,
					fw_time_diff, data[0]);
	}

	atvr_snd = shdr_card->private_data;

#ifdef DEBUG_HID_RAW_INPUT
	pr_info("%s: report->id = 0x%x, size = %d\n",
		__func__, report->id, size);
	if (size <= 22) {
		u32 i;
		for (i = 0; i < size; i++)
			pr_info("data[%d] = 0x%02x\n", i, data[i]);
	}
#endif

	/* WAR for CSR issue */
	if (atvr_jarvis_break_events(hdev, report, data, size))
		return 1;

	if (report->id == ADPCM_AUDIO_REPORT_ID) {
		/* send the data, minus the report-id in data[0], to the
		 * alsa audio decoder driver for ADPCM
		 */
#if (DEBUG_AUDIO_RECEPTION == 1)
		if (atvr_snd->packet_counter == 0)
			snd_atvr_log("first ADPCM packet received\n");
#endif
		audio_dec(hdev, &data[1], PACKET_TYPE_ADPCM, size - 1);
		/* we've handled the event */
		return 1;
	} else if (report->id == MSBC_AUDIO1_REPORT_ID) {
		/* first do special case check if there is any
		 * keyCode active in this report.  if so, we
		 * generate the same keyCode but on report 2, which
		 * is where normal keys are reported.  the keycode
		 * is being sent in the audio packet to save packets
		 * and over the air bandwidth.
		 */
		if (data[2] & KEYCODE_PRESENT_IN_AUDIO_PACKET_FLAG) {
			u8 key_data[3];
			key_data[0] = INPUT_REPORT_ID;
			key_data[1] = data[1]; /* low byte */
			key_data[2] = data[2]; /* high byte */
			key_data[2] &= ~KEYCODE_PRESENT_IN_AUDIO_PACKET_FLAG;
			hid_report_raw_event(hdev, 0, key_data,
					     sizeof(key_data), 0);
#ifdef DEBUG_HID_RAW_INPUT
			pr_info("%s: generated hid keycode 0x%02x%02x\n",
				__func__, key_data[2], key_data[1]);
#endif
		}

		/* send the audio part to the alsa audio decoder for mSBC */
#if (DEBUG_AUDIO_RECEPTION == 1)
		if (atvr_snd->packet_counter == 0)
			snd_atvr_log("first MSBC packet received\n");
#endif
		/* strip the one byte report id and two byte keycode field */
		audio_dec(hdev, &data[1 + 2], PACKET_TYPE_MSBC, size - 1 - 2);
		/* we've handled the event */
		return 1;
	} else if ((report->id == MSBC_AUDIO2_REPORT_ID) ||
		   (report->id == MSBC_AUDIO3_REPORT_ID)) {
		/* strip the one byte report id */
		audio_dec(hdev, &data[1], PACKET_TYPE_MSBC, size - 1);
		/* we've handled the event */
		return 1;
	} else if (report->id == INPUT_EVT_INTR_DATA_ID) {
		/*Debug infor sent by device*/
		debug_info = kmalloc(sizeof(u8) * size + 1, GFP_ATOMIC);
		if (!debug_info) {
			pr_err("%s, Kmalloc failed!", __func__);
			return -ENOMEM;
		}
		*((char *)(debug_info + size)) = '\0';
		memcpy(debug_info, data, size);
		pr_debug("Report ID 10: PID: %d, report %s", hdev->product,
			(char *)debug_info);
		kfree(debug_info);
	} else if (hdev->product == USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE &&
			(report->id == SENSOR_REPORT_ID ||
			report->id == SENSOR_REPORT_ID_SYN ||
			report->id == SENSOR_REPORT_ID_COMBINED) &&
			shdr_dev->snsr_fns && shdr_dev->snsr_fns->recv) {
		shdr_dev->snsr_fns->recv(shdr_dev->st, data, size);
		/* TODO: ret check */
		if (report->id == SENSOR_REPORT_ID_COMBINED) {
			data[0] = JAR_BUTTON_REPORT_ID;
			hid_report_raw_event(hdev, 0, data,
					     TS_BUTTON_REPORT_SIZE, 0);
		}
		/* we've handled the event */
		return 1;
	}
	/* let the event through for regular input processing */
	return 0;
}

static ssize_t atvr_show_hid_miss_war_timeout(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct hid_device *hdev =
		container_of(dev, struct hid_device, dev);
	struct shdr_device *shdr_dev = hid_get_drvdata(hdev);

	return sprintf(buf, "%d\n", shdr_dev->hid_miss_war_timeout);
}

static ssize_t atvr_store_hid_miss_war_timeout(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct hid_device *hdev =
		container_of(dev, struct hid_device, dev);
	struct shdr_device *shdr_dev = hid_get_drvdata(hdev);
	int val;

	if (!kstrtoint(buf, 0, &val)) {
		mutex_lock(&shdr_dev->hid_miss_war_lock);
		shdr_dev->hid_miss_war_timeout = val;
		mutex_unlock(&shdr_dev->hid_miss_war_lock);
	}

	return count;
}

static DEVICE_ATTR(timeout, S_IRUGO | S_IWUSR,
	atvr_show_hid_miss_war_timeout, atvr_store_hid_miss_war_timeout);

static void atvr_snsr_probe(struct work_struct *work)
{
	struct shdr_device *shdr_dev =
		container_of(work, struct shdr_device, snsr_probe_work);
	struct tsfw_icm20628_state *st = NULL;

	if (shdr_dev->snsr_fns && shdr_dev->snsr_fns->probe)
		shdr_dev->snsr_fns->probe(shdr_dev->hdev, &st);
		/* TODO: ret check */
	shdr_dev->st = st;
}

static int atvr_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct snd_atvr *atvr_snd;
	int ret, i;
	struct shdr_device *shdr_dev;
	struct snd_card *shdr_card;

	shdr_dev = kzalloc(sizeof(*shdr_dev), GFP_KERNEL);
	if (shdr_dev == NULL) {
		hid_err(hdev, "can't alloc descriptor\n");
		return -ENOMEM;
	}
	/* since vendor/product id filter doesn't work yet, because
	 * Bluedroid is unable to get the vendor/product id, we
	 * have to filter on name
	 */
	pr_info("%s: name = %s, vendor_id = %d, product_id = %d, num %d\n",
		__func__, hdev->name, hdev->vendor, hdev->product, num_remotes);

	pr_info("%s: Found target remote %s\n", __func__, hdev->name);
	hid_set_drvdata(hdev, shdr_dev);
	/* set seq num to 255 since first packet comes with seq number 0 */
	for (i = 0; i < MAX_DEBUG_REPORTS; i++)
		shdr_dev->debug_info[i].seq_num = 0xFF;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "hid parse failed\n");
		goto err_parse;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err_start;
	}

	/*
	 * Lazy-creation of the soundcard, and enable the wired headset
	 * only then to avoid race conditions on subsequent connections.
	 * AudioService.java delays enabling the output
	 */
	mutex_lock(&snd_cards_lock);
	ret = atvr_snd_initialize(hdev, &shdr_card);
	if (ret)
		goto err_stop;

	/*
	 * hdev pointer is not guaranteed to be the same thus following
	 * stuff has to be updated every time.
	 */
	shdr_dev->shdr_card = shdr_card;
	shdr_dev->hdev = hdev;
	atvr_snd = shdr_card->private_data;
	atvr_snd->hdev = hdev;
	snd_card_set_dev(shdr_card, &hdev->dev);

	silence_counter = 0;
	pr_info("%s: remotes count %d->%d\n", __func__,
		num_remotes, num_remotes+1);
	num_remotes++;

	mutex_unlock(&snd_cards_lock);

	if (hdev->product == USB_DEVICE_ID_NVIDIA_PEPPER ||
			hdev->product == USB_DEVICE_ID_NVIDIA_FRIDAY) {
		shdr_dev->hid_miss_war_timeout = -1;
		mutex_init(&shdr_dev->hid_miss_war_lock);
		INIT_DELAYED_WORK(&shdr_dev->hid_miss_war_work,
				  atvr_pepper_button_release);

		ret = device_create_file(&hdev->dev,
			&dev_attr_timeout);
		if (ret) {
			hid_err(hdev,
				"cannot create sysfs timeout attribute\n");
			goto err_stop;
		}
		ret = kobject_uevent(&hdev->dev.kobj, KOBJ_CHANGE);
	}

	if (hdev->product == USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE)
		shdr_dev->snsr_fns = tsfw_icm20628_fns();

	INIT_WORK(&shdr_dev->snsr_probe_work, atvr_snsr_probe);
	schedule_work(&shdr_dev->snsr_probe_work);

	return 0;
err_stop:
	hid_hw_stop(hdev);
	mutex_unlock(&snd_cards_lock);
err_start:
err_parse:
	kfree(shdr_dev);
	return ret;
}

static void atvr_remove(struct hid_device *hdev)
{
	unsigned long flags;
	struct shdr_device *shdr_dev = hid_get_drvdata(hdev);
	struct snd_card *shdr_card = shdr_dev->shdr_card;
	struct snd_atvr *atvr_snd;

	if (shdr_card == NULL)
		return;

	cancel_work_sync(&shdr_dev->snsr_probe_work);

	if (shdr_dev->snsr_fns && shdr_dev->snsr_fns->remove)
		shdr_dev->snsr_fns->remove(shdr_dev->st);
	/* TODO: ret check */

	if (hdev->product == USB_DEVICE_ID_NVIDIA_PEPPER ||
			hdev->product == USB_DEVICE_ID_NVIDIA_FRIDAY) {
		cancel_delayed_work_sync(&shdr_dev->hid_miss_war_work);
		device_remove_file(&hdev->dev, &dev_attr_timeout);
	}

	if (hdev->product == USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE && hdev->uniq) {
		if (shdr_dev->last_ljsx == 0 || shdr_dev->last_ljsx == 0xffff ||
		    shdr_dev->last_ljsy == 0 || shdr_dev->last_ljsy == 0xffff ||
		    shdr_dev->last_rjsx == 0 || shdr_dev->last_rjsx == 0xffff ||
		    shdr_dev->last_rjsy == 0 || shdr_dev->last_rjsy == 0xffff) {
			atvr_ts_joystick_missreport_stats_inc(hdev);
		}
	}

	mutex_lock(&snd_cards_lock);
	atvr_snd = shdr_card->private_data;

	spin_lock_irqsave(&atvr_snd->s_substream_lock, flags);
	atvr_snd->substream_state |= ATVR_REMOVE;
	spin_unlock_irqrestore(&atvr_snd->s_substream_lock, flags);

	mutex_lock(&atvr_snd->hdev_lock);
	atvr_snd->hdev = NULL;
	mutex_unlock(&atvr_snd->hdev_lock);

	hid_set_drvdata(hdev, NULL);
	hid_hw_stop(hdev);
	pr_info("%s: hdev->name = %s removed, num %d->%d\n",
		__func__, hdev->name, num_remotes, num_remotes - 1);
	num_remotes--;

	cards_in_use[atvr_snd->card_index] = false;
	snd_atvr_dealloc_audio_buffs(atvr_snd);
	mutex_destroy(&atvr_snd->hdev_lock);
	snd_card_disconnect(shdr_card);
	snd_card_free_when_closed(shdr_card);
	mutex_destroy(&shdr_dev->hid_miss_war_lock);
	kfree(shdr_dev);
	mutex_unlock(&snd_cards_lock);
}

static int atvr_input_mapped(struct hid_device *hdev, struct hid_input *hi,
			struct hid_field *field, struct hid_usage *usage,
			unsigned long **bit, int *max)
{
	int a = field->logical_minimum;
	int b = field->logical_maximum;
	int fuzz;
	int flat;
	struct shdr_device *shdr_dev = hid_get_drvdata(hdev);

	if ((usage->type == EV_ABS) && (field->application == HID_GD_GAMEPAD
			|| field->application == HID_GD_JOYSTICK)) {
		switch (usage->hid) {
		case HID_GD_X:
		case HID_GD_Y:
		case HID_GD_RX:
		case HID_GD_RY:
			fuzz = JOYSTICK_FUZZ;
			flat = JOYSTICK_FLAT;
			break;
		case HID_GD_Z:
		case HID_GD_RZ:
		case 0x200c4: /* For ABS_GAS */
		case 0x200c5: /* For ABS_BRAKE */
			fuzz = TRIGGER_FUZZ;
			flat = TRIGGER_FLAT;
			break;
		default: return 0;/*Use generic mapping for HatX, HatY*/
		}
		set_bit(usage->type, hi->input->evbit);
		set_bit(usage->code, *bit);
		input_set_abs_params(hi->input, usage->code, a, b, fuzz, flat);
		input_abs_set_res(hi->input, usage->code,
			hidinput_calc_abs_res(field, usage->code));
		return -1;
	} else if (usage->hid == HID_GEN_DESK_DEBUG)
		atvr_set_hid_debug_report_idx(shdr_dev, field->report->id);
	return 0;
}

static const struct hid_device_id atvr_devices[] = {
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_JARVIS)},
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_PEPPER)},
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_FRIDAY)},
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE)},
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_STORMCASTER)},
	{HID_USB_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_THUNDERSTRIKE)},
	{HID_USB_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_STORMCASTER)},
	{ }
};
MODULE_DEVICE_TABLE(hid, atvr_devices);

static struct hid_driver atvr_driver = {
	.name = "Jarvis",
	.id_table = atvr_devices,
	.input_mapped = atvr_input_mapped,
	.raw_event = atvr_raw_event,
	.probe = atvr_probe,
	.remove = atvr_remove,
};

static int hid_miss_stats;
static struct mutex stats_lock;

static void atvr_hid_miss_stats_inc(void)
{
	mutex_lock(&stats_lock);
	hid_miss_stats++;
	mutex_unlock(&stats_lock);
}

static ssize_t hid_miss_stats_show(struct device_driver *driver, char *buf)
{
	int stats;

	mutex_lock(&stats_lock);
	stats = hid_miss_stats;
	mutex_unlock(&stats_lock);

	return sprintf(buf, "%d", stats);
}

static ssize_t hid_miss_stats_store(struct device_driver *driver,
				    const char *buf, size_t count)
{
	int val;

	if (!kstrtoint(buf, 0, &val) && val == 0) {
		mutex_lock(&stats_lock);
		hid_miss_stats = 0;
		mutex_unlock(&stats_lock);
	}

	return count;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static DRIVER_ATTR_RW(hid_miss_stats);
#else
static DRIVER_ATTR(hid_miss_stats, S_IRUGO | S_IWUSR,
		   hid_miss_stats_show, hid_miss_stats_store);
#endif

struct ts_joystick_missreport_stat {
	char uniq[17];
	int count;
	struct list_head list;
};

static LIST_HEAD(ts_joystick_stats);

static void atvr_ts_joystick_missreport_stats_inc(struct hid_device *hdev)
{
	struct ts_joystick_missreport_stat *stat;

	mutex_lock(&stats_lock);
	list_for_each_entry(stat, &ts_joystick_stats, list) {
		if (!strcmp(stat->uniq, hdev->uniq)) {
			stat->count++;
			mutex_unlock(&stats_lock);
			return;
		}
	}

	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (stat) {
		strcpy(stat->uniq, hdev->uniq);
		stat->count++;
		list_add_tail(&stat->list, &ts_joystick_stats);
	}
	mutex_unlock(&stats_lock);
}

static ssize_t atvr_show_ts_joystick_stats(struct device_driver *driver,
					   char *buf)
{
	struct ts_joystick_missreport_stat *stat;
	int count = 0;

	mutex_lock(&stats_lock);
	list_for_each_entry(stat, &ts_joystick_stats, list) {
		count += sprintf(buf + count, "%s,%d\n",
				 stat->uniq, stat->count);
	}
	mutex_unlock(&stats_lock);

	return count;
}

static ssize_t atvr_store_ts_joystick_stats(struct device_driver *driver,
					     const char *buf, size_t count)
{
	struct ts_joystick_missreport_stat *stat;
	int val;

	if (!kstrtoint(buf, 0, &val) && val == 0) {
		mutex_lock(&stats_lock);
		list_for_each_entry(stat, &ts_joystick_stats, list) {
			stat->count = 0;
		}
		mutex_unlock(&stats_lock);
	}

	return count;
}

static DRIVER_ATTR(ts_joystick_stats, 0644,
		   atvr_show_ts_joystick_stats, atvr_store_ts_joystick_stats);

static int atvr_init(void)
{
	int ret;

	mutex_init(&snd_cards_lock);
	ret = hid_register_driver(&atvr_driver);
	if (ret) {
		pr_err("%s: can't register SHIELD Remote driver\n",
			__func__);
		goto err_hid_register;
	}

	mutex_init(&stats_lock);
	ret = driver_create_file(&atvr_driver.driver,
				 &driver_attr_hid_miss_stats);
	if (ret) {
		pr_err("%s: failed to create driver sysfs node\n", __func__);
		goto err_attr_hid_miss_stats;
	}

	ret = driver_create_file(&atvr_driver.driver,
				 &driver_attr_ts_joystick_stats);
	if (ret) {
		pr_err("%s: failed to create driver sysfs node\n", __func__);
		goto err_attr_ts_joystick_stats;
	}

#if (DEBUG_WITH_MISC_DEVICE == 1)
	pcm_dev_node.minor = MISC_DYNAMIC_MINOR;
	pcm_dev_node.name = "snd_atvr_pcm";
	pcm_dev_node.fops = &pcm_fops;
	ret = misc_register(&pcm_dev_node);
	if (ret)
		pr_err("%s: failed to create pcm misc device %d\n",
		       __func__, ret);
	else
		pr_info("%s: succeeded creating misc device %s\n",
			__func__, pcm_dev_node.name);

	adpcm_dev_node.minor = MISC_DYNAMIC_MINOR;
	adpcm_dev_node.name = "snd_atvr_adpcm";
	adpcm_dev_node.fops = &adpcm_fops;
	ret = misc_register(&adpcm_dev_node);
	if (ret)
		pr_err("%s: failed to create adpcm misc device %d\n",
		       __func__, ret);
	else
		pr_info("%s: succeeded creating misc device %s\n",
			__func__, adpcm_dev_node.name);

	msbc_dev_node.minor = MISC_DYNAMIC_MINOR;
	msbc_dev_node.name = "snd_atvr_msbc";
	msbc_dev_node.fops = &msbc_fops;
	ret = misc_register(&msbc_dev_node);
	if (ret)
		pr_err("%s: failed to create mSBC misc device %d\n",
		       __func__, ret);
	else
		pr_info("%s: succeeded creating misc device %s\n",
			__func__, msbc_dev_node.name);
#endif

	return ret;

err_attr_ts_joystick_stats:
	driver_remove_file(&atvr_driver.driver, &driver_attr_hid_miss_stats);
err_attr_hid_miss_stats:
	hid_unregister_driver(&atvr_driver);
	mutex_destroy(&stats_lock);
err_hid_register:
	mutex_destroy(&snd_cards_lock);
	return ret;
}

static void atvr_exit(void)
{
#if (DEBUG_WITH_MISC_DEVICE == 1)
	misc_deregister(&msbc_dev_node);
	misc_deregister(&adpcm_dev_node);
	misc_deregister(&pcm_dev_node);
#endif

	driver_remove_file(&atvr_driver.driver, &driver_attr_hid_miss_stats);
	driver_remove_file(&atvr_driver.driver, &driver_attr_ts_joystick_stats);
	hid_unregister_driver(&atvr_driver);
	mutex_destroy(&snd_cards_lock);
	mutex_destroy(&stats_lock);
}

module_init(atvr_init);
module_exit(atvr_exit);
MODULE_LICENSE("GPL");
