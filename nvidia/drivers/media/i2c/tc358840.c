/*
 * tc358840.c - Toshiba UH2C/D HDMI-CSI bridge driver
 *
 * Copyright (c) 2015, Armin Weiss <weii@zhaw.ch>
 * Copyright (c) 2016 - 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is based on the tc358840 - Toshiba HDMI to CSI-2 bridge driver
 * from Cisco Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/nospec.h>

#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#ifdef CONFIG_V4L2_FWNODE
#include <media/v4l2-fwnode.h>
#else
#include <media/v4l2-of.h>
#endif
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>

#include <media/i2c/tc358840.h>
#include "tc358840_regs.h"
#include <asm/barrier.h>

#define MAX_DATABUF 10

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

/* TODO: Implement Colorbar TPG */

#define EDID_NUM_BLOCKS_MAX 8
#define EDID_BLOCK_SIZE 128
#define DELAY_ENABLE_INTERRUPT_MS 10000

static u8 edid[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x52, 0x62, 0x88, 0x88, 0x00, 0x88, 0x88, 0x88,
	0x1C, 0x15, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78,
	0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
	0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xEC, 0x68,
	0x00, 0xA0, 0xF0, 0x70, 0x37, 0x80, 0x30, 0x20,
	0x3A, 0x00, 0x00, 0x70, 0xF8, 0x00, 0x00, 0x1C,
	0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
	0x58, 0x2C, 0x45, 0x00, 0xC4, 0x8E, 0x21, 0x00,
	0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x54,
	0x6F, 0x73, 0x68, 0x69, 0x62, 0x61, 0x2D, 0x55,
	0x48, 0x32, 0x44, 0x0A, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x17, 0x3D, 0x0F, 0x8C, 0x17, 0x00, 0x0A,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xAF,
	0x02, 0x03, 0x1A, 0x74, 0x47, 0x32, 0x10, 0x04,
	0x32, 0x32, 0x32, 0x32, 0x23, 0x09, 0x07, 0x01,
	0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0,
	0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0xC4, 0x8E,
	0x21, 0x00, 0x00, 0x1E, 0xEC, 0x68, 0x00, 0xA0,
	0xF0, 0x70, 0x37, 0x80, 0x30, 0x20, 0x3A, 0x00,
	0x00, 0x70, 0xF8, 0x00, 0x00, 0x1C, 0xEC, 0x68,
	0x00, 0xA0, 0xF0, 0x70, 0x37, 0x80, 0x30, 0x20,
	0x3A, 0x00, 0x00, 0x70, 0xF8, 0x00, 0x00, 0x1C,
	0xEC, 0x68, 0x00, 0xA0, 0xF0, 0x70, 0x37, 0x80,
	0x30, 0x20, 0x3A, 0x00, 0x00, 0x70, 0xF8, 0x00,
	0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26,
};

static const struct v4l2_dv_timings_cap tc358840_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(
		1, 10000, 1, 10000, 0, 297000000,
		V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
		V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
		V4L2_DV_BT_CAP_PROGRESSIVE |
		V4L2_DV_BT_CAP_REDUCED_BLANKING |
		V4L2_DV_BT_CAP_CUSTOM)
};

struct tc358840_state {
	struct tc358840_platform_data pdata;
	struct v4l2_subdev sd;
	struct media_pad pad[2];
	struct v4l2_ctrl_handler hdl;
	struct i2c_client *i2c_client;
	bool enabled;
	bool format_changed;

	/* controls */
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;
	struct v4l2_ctrl *splitter_width_ctrl;

	/* work queues */
	struct workqueue_struct *work_queues;
	struct delayed_work delayed_work_enable_hotplug;
	struct delayed_work delayed_work_enable_interrupt;
	struct work_struct process_isr;
	struct mutex isr_lock;

	/* edid  */
	u8 edid_blocks_written;

	/* timing / mbus */
	struct v4l2_dv_timings timings;
	u32 mbus_fmt_code;
};

static inline struct tc358840_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358840_state, sd);
}


static void tc358840_enable_interrupts(
	struct v4l2_subdev *sd, bool cable_connected);
static int tc358840_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd);
static void tc358840_init_interrupts(struct v4l2_subdev *sd);
static int tc358840_s_dv_timings(
	struct v4l2_subdev *sd, struct v4l2_dv_timings *timings);
static void tc358840_set_csi(struct v4l2_subdev *sd);
static void tc358840_set_splitter(struct v4l2_subdev *sd);
static int tc358840_s_edid(struct v4l2_subdev *sd,
	struct v4l2_subdev_edid *edid);

/* --------------- I2C --------------- */

static void i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358840_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: reading register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C read 0x%04X = 0x%02X\n", reg, values[0]);
		break;
	case 2:
		v4l2_info(sd, "I2C read 0x%04X = 0x%02X%02X\n",
			reg, values[1], values[0]);
		break;
	case 4:
		v4l2_info(sd, "I2C read 0x%04X = 0x%02X%02X%02X%02X\n",
			reg, values[3], values[2], values[1], values[0]);
		break;
	default:
		v4l2_info(sd, "I2C read %d bytes from address 0x%04X\n",
			n, reg);
	}

}

static void i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358840_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err, i;
	struct i2c_msg msg;
	u8 data[MAX_DATABUF];

	WARN_ON(MAX_DATABUF < (2 + n));
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < n; i++)
		data[2 + i] = values[i];

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		v4l2_err(sd, "%s: writing register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
		return;
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C write 0x%04X = 0x%02X\n", reg, data[2]);
		break;
	case 2:
		v4l2_info(sd, "I2C write 0x%04X = 0x%02X%02X\n",
			reg, data[3], data[2]);
		break;
	case 4:
		v4l2_info(sd, "I2C write 0x%04X = 0x%02X%02X%02X%02X\n",
			reg, data[5], data[4], data[3], data[2]);
		break;
	default:
		v4l2_info(sd, "I2C write %d bytes from address 0x%04X\n",
			n, reg);
	}
}

static u8 i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	u8 val;

	i2c_rd(sd, reg, &val, 1);

	return val;
}

static void i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	i2c_wr(sd, reg, &val, 1);
}

static void i2c_wr8_and_or(struct v4l2_subdev *sd, u16 reg,
		u8 mask, u8 val)
{
	i2c_wr8(sd, reg, (i2c_rd8(sd, reg) & mask) | val);
}

static u16 i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val;

	i2c_rd(sd, reg, (u8 *)&val, 2);

	return val;
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 2);
}

static void i2c_wr16_and_or(struct v4l2_subdev *sd, u16 reg, u16 mask, u16 val)
{
	i2c_wr16(sd, reg, (i2c_rd16(sd, reg) & mask) | val);
}

static u32 i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	u32 val;

	i2c_rd(sd, reg, (u8 *)&val, 4);

	return val;
}

static void i2c_wr32(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 4);
}

static void i2c_wr32_and_or(struct v4l2_subdev *sd, u32 reg, u32 mask, u32 val)
{
	i2c_wr32(sd, reg, (i2c_rd32(sd, reg) & mask) | val);
}

/* --------------- STATUS --------------- */

static inline bool is_hdmi(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_HDMI;
}

static inline bool tx_5v_power_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_DDC5V;
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_TMDS);
}

static inline bool no_sync(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_SYNC);
}

static inline bool audio_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, AU_STATUS0) & MASK_S_A_SAMPLE;
}

static int get_audio_sampling_rate(struct v4l2_subdev *sd)
{
	static const int code_to_rate[] = {
		44100, 0, 48000, 32000, 22050, 384000, 24000, 352800,
		88200, 768000, 96000, 705600, 176400, 0, 192000, 0
	};

	/* Register FS_SET is not cleared when the cable is disconnected */
	if (no_signal(sd))
		return 0;

	return code_to_rate[i2c_rd8(sd, FS_SET) & MASK_FS];
}

static unsigned tc358840_num_csi_lanes_in_use(struct v4l2_subdev *sd)
{
	/* FIXME: Read # of lanes from both, TX0 and TX1 */
	return i2c_rd32(sd, CSITX0_BASE_ADDR+LANEEN) & MASK_LANES;
}

/* --------------- TIMINGS --------------- */

static inline unsigned fps(const struct v4l2_bt_timings *t)
{
	if (!V4L2_DV_BT_FRAME_HEIGHT(t) || !V4L2_DV_BT_FRAME_WIDTH(t))
		return 0;

	return DIV_ROUND_CLOSEST((unsigned)t->pixelclock,
			V4L2_DV_BT_FRAME_HEIGHT(t) * V4L2_DV_BT_FRAME_WIDTH(t));
}

static int tc358840_get_detected_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings *bt = &timings->bt;
	unsigned width, height, frame_width, frame_height, frame_interval, fps;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (no_signal(sd)) {
		v4l2_dbg(1, debug, sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}
	if (no_sync(sd)) {
		v4l2_dbg(1, debug, sd, "%s: no sync on signal\n", __func__);
		return -ENOLCK;
	}

	timings->type = V4L2_DV_BT_656_1120;

	bt->interlaced = i2c_rd8(sd, VI_STATUS1) &
		MASK_S_V_INTERLACE ? V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	width = ((i2c_rd8(sd, DE_HSIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_HSIZE_LO);
	height = ((i2c_rd8(sd, DE_VSIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_VSIZE_LO);
	frame_width = ((i2c_rd8(sd, IN_HSIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, IN_HSIZE_LO);
	frame_height = (((i2c_rd8(sd, IN_VSIZE_HI) & 0x3f) << 8) +
		i2c_rd8(sd, IN_VSIZE_LO)) / 2;

	/* TODO: Check if frame_interval is correct
	 * since the register is not in the datasheet rev. 1.5 */

	/* frame interval in milliseconds * 10
	 * Require SYS_FREQ0 and SYS_FREQ1 are precisely set */
	frame_interval = ((i2c_rd8(sd, FV_CNT_HI) & 0x3) << 8) +
		i2c_rd8(sd, FV_CNT_LO);
	fps = (frame_interval > 0) ?
		DIV_ROUND_CLOSEST(10000, frame_interval) : 0;

	bt->width = width;
	bt->height = height;
	bt->vsync = frame_height - height;
	bt->hsync = frame_width - width;
	bt->pixelclock = frame_width * frame_height * fps;
	if (bt->interlaced == V4L2_DV_INTERLACED) {
		bt->height *= 2;
		bt->il_vsync = bt->vsync + 1;
		bt->pixelclock /= 2;
	}

	return 0;
}

/* --------------- HOTPLUG / HDCP / EDID --------------- */

static void tc358840_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tc358840_state *state = container_of(dwork,
		struct tc358840_state, delayed_work_enable_hotplug);
	struct v4l2_subdev *sd = &state->sd;

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, MASK_HPD_OUT0);
}

static void tc358840_set_hdmi_hdcp(struct v4l2_subdev *sd, bool enable)
{
	v4l2_dbg(2, debug, sd, "%s: %s\n", __func__, enable ?
				"enable" : "disable");

	i2c_wr8_and_or(sd, HDCP_REG1,
			~(MASK_AUTH_UNAUTH_SEL | MASK_AUTH_UNAUTH),
			MASK_AUTH_UNAUTH_SEL_16_FRAMES | MASK_AUTH_UNAUTH_AUTO);

	i2c_wr8_and_or(sd, HDCP_REG2, ~MASK_AUTO_P3_RESET,
			SET_AUTO_P3_RESET_FRAMES(0x0f));

	/* HDCP is disabled by configuring the receiver as HDCP repeater. The
	 * repeater mode require software support to work, so HDCP
	 * authentication will fail.
	 */
	i2c_wr8_and_or(sd, HDCP_REG3, ~KEY_RD_CMD, enable ? KEY_RD_CMD : 0);
	i2c_wr8_and_or(sd, HDCP_MODE, ~(MASK_AUTO_CLR | MASK_MODE_RST_TN),
			enable ?  (MASK_AUTO_CLR | MASK_MODE_RST_TN) : 0);

	/* Apple MacBook Pro gen.8 has a bug that makes it freeze every fifth
	 * second when HDCP is disabled, but the MAX_EXCED bit is handled
	 * correctly and HDCP is disabled on the HDMI output.
	 */
	i2c_wr8_and_or(sd, BSTATUS1, ~MASK_MAX_EXCED,
			enable ? 0 : MASK_MAX_EXCED);
	i2c_wr8_and_or(sd, BCAPS, ~(MASK_REPEATER | MASK_READY),
			enable ? 0 : MASK_REPEATER | MASK_READY);
}

static void tc358840_disable_edid(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	cancel_delayed_work_sync(&state->delayed_work_enable_hotplug);

	/* DDC access to EDID is also disabled when hotplug is disabled. See
	 * register DDC_CTL */
	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, 0x0);
}

static void tc358840_enable_edid(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	if (state->edid_blocks_written == 0) {
		v4l2_dbg(2, debug, sd, "%s: no EDID -> no hotplug\n", __func__);
		return;
	}

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	/* Enable hotplug after 100 ms. DDC access to EDID is also enabled when
	 * hotplug is enabled. See register DDC_CTL */
	queue_delayed_work(state->work_queues,
		&state->delayed_work_enable_hotplug, HZ / 10);

	tc358840_enable_interrupts(sd, true);
	tc358840_s_ctrl_detect_tx_5v(sd);
}

static void tc358840_delayed_work_enable_interrupt(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tc358840_state *state = container_of(dwork,
		struct tc358840_state, delayed_work_enable_interrupt);
	struct v4l2_subdev *sd = &state->sd;

	struct v4l2_subdev_edid sd_edid = {
		.blocks = 2,
		.edid = edid,
	};

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	tc358840_enable_interrupts(sd, tx_5v_power_present(sd));

	/* Temporary EDID. Should be set by userspace */
	tc358840_s_edid(sd, &sd_edid);
}

static void tc358840_erase_bksv(struct v4l2_subdev *sd)
{
	int i;

	for (i = 0; i < 5; i++)
		i2c_wr8(sd, BKSV + i, 0);
}

/* --------------- AVI infoframe --------------- */

static void print_avi_infoframe(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	union hdmi_infoframe frame;
	u8 buffer[HDMI_INFOFRAME_SIZE(MAX)];

	if (!is_hdmi(sd)) {
		v4l2_info(sd, "DVI-D signal - AVI infoframe not supported\n");
		return;
	}

	i2c_rd(sd, PK_AVI_0HEAD, buffer, HDMI_INFOFRAME_SIZE(AVI));

	if (hdmi_infoframe_unpack(&frame, buffer) < 0) {
		v4l2_err(sd, "%s: unpack of AVI infoframe failed\n", __func__);
		return;
	}

	hdmi_infoframe_log(KERN_INFO, dev, &frame);
}

/* --------------- CTRLS --------------- */

static int tc358840_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->detect_tx_5v_ctrl,
		tx_5v_power_present(sd));
}

static int tc358840_s_ctrl_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->audio_sampling_rate_ctrl,
			get_audio_sampling_rate(sd));
}

static int tc358840_s_ctrl_audio_present(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->audio_present_ctrl,
			audio_present(sd));
}

static int tc358840_update_controls(struct v4l2_subdev *sd)
{
	int ret = 0;

	ret |= tc358840_s_ctrl_detect_tx_5v(sd);
	ret |= tc358840_s_ctrl_audio_sampling_rate(sd);
	ret |= tc358840_s_ctrl_audio_present(sd);

	return ret;
}

/* --------------- INIT --------------- */

static void tc358840_reset_phy(struct v4l2_subdev *sd)
{
	v4l2_dbg(1, debug, sd, "%s:\n", __func__);

	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, 0);
	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, MASK_RESET_CTRL);
}

static void tc358840_reset(struct v4l2_subdev *sd, u16 mask)
{
	u16 sysctl = i2c_rd16(sd, SYSCTL);

	i2c_wr16(sd, SYSCTL, sysctl | mask);
	i2c_wr16(sd, SYSCTL, sysctl & ~mask);
}

static inline void tc358840_sleep_mode(struct v4l2_subdev *sd, bool enable)
{
	v4l2_dbg(1, debug, sd, "%s(): %s\n", __func__,
		enable ? "enable" : "disable");

	i2c_wr16_and_or(sd, SYSCTL, ~MASK_SLEEP, enable ? MASK_SLEEP : 0);
}

static int enable_stream(struct v4l2_subdev *sd, bool enable)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;

	u32 sync_timeout_ctr;

	v4l2_dbg(2, debug, sd, "%s: %sable\n", __func__, enable ? "en" : "dis");

	if (enable == state->enabled)
		return 0;

	if (enable) {
#if 0		/* Wait until we can use the clock-noncontinuous property */
		if (pdata->endpoint.bus.mipi_csi2.flags &
		    V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) {
			i2c_wr32_and_or(sd, FUNCMODE, ~(MASK_CONTCLKMODE),
					MASK_FORCESTOP);
		} else {
			/* It is critical for CSI receiver to see lane transition
			 * LP11->HS. Set to non-continuous mode to enable clock lane
			 * LP11 state. */
			i2c_wr32_and_or(sd, FUNCMODE, ~(MASK_CONTCLKMODE), 0);
			/* Set to continuous mode to trigger LP11->HS transition */
			i2c_wr32_and_or(sd, FUNCMODE, 0, MASK_CONTCLKMODE);
		}
#else
		i2c_wr32_and_or(sd, FUNCMODE, ~(MASK_CONTCLKMODE),
				MASK_FORCESTOP);
#endif
		/* Unmute video */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE);
		/* Signal end of initialization */
		i2c_wr8(sd, INIT_END, MASK_INIT_END);
	} else {
		/* Enable Registers to be initialized */
		i2c_wr8_and_or(sd, INIT_END, ~(MASK_INIT_END), 0x00);

		/* Mute video so that all data lanes go to LSP11 state.
		 * No data is output to CSI Tx block. */

		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE | MASK_VI_MUTE);
		tc358840_set_csi(sd);
		tc358840_set_splitter(sd);
	}

	/* Wait for HDMI input to become stable */
	if (enable) {
		sync_timeout_ctr = 100;
		while (no_sync(sd) && sync_timeout_ctr)
			sync_timeout_ctr--;

		if (sync_timeout_ctr == 0) {
			/* Disable stream again. Probably no cable inserted.. */
			v4l2_err(sd, "%s: Timeout: HDMI input sync failed.\n",
					__func__);
			enable_stream(sd, false);
			return -EIO;
		}

		v4l2_dbg(2, debug, sd,
			"%s: Stream enabled! Remaining timeout attempts: %d\n",
			__func__, sync_timeout_ctr);
	}

	i2c_wr16_and_or(sd, CONFCTL0,
		~(MASK_VTX0EN | MASK_VTX1EN | MASK_ABUFEN),
		enable ? ((pdata->csi_port & (MASK_VTX0EN | MASK_VTX1EN)) |
		MASK_ABUFEN | MASK_TX_MSEL | MASK_AUTOINDEX) :
		(MASK_TX_MSEL | MASK_AUTOINDEX));
	state->enabled = enable;

	return 0;
}

static void tc358840_set_splitter(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (state->timings.bt.width <= 1920) {
		i2c_wr16_and_or(sd, SPLITTX0_CTRL, ~(MASK_IFEN | MASK_LCD_CSEL),
				MASK_SPBP);
		i2c_wr16_and_or(sd, SPLITTX1_CTRL, ~(MASK_IFEN | MASK_LCD_CSEL),
				MASK_SPBP);

		i2c_wr16_and_or(sd, SPLITTX0_SPLIT, (u16)(0x0000ffff &
							  ~(MASK_TX1SEL |
							    MASK_EHW)), 0);
	} else {
		i2c_wr16_and_or(sd, SPLITTX0_CTRL, ~(MASK_IFEN | MASK_LCD_CSEL | MASK_SPBP), 0);
		i2c_wr16_and_or(sd, SPLITTX1_CTRL, ~(MASK_IFEN | MASK_LCD_CSEL | MASK_SPBP), 0);

		i2c_wr16_and_or(sd, SPLITTX0_SPLIT, ~(MASK_TX1SEL), MASK_EHW);
	}
}

static void tc358840_set_pll(struct v4l2_subdev *sd,
				enum tc358840_csi_port port)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;
	u16 base_addr;
	u16 pll_frs;
	u32 pllconf;
	u32 pllconf_new;
	u32 hsck;

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	BUG_ON((pdata->csi_port <= CSI_TX_NONE) ||
		(pdata->csi_port > CSI_TX_BOTH));

	if (pdata->csi_port == CSI_TX_NONE) {
		v4l2_err(sd, "%s: No CSI port defined!\n", __func__);
		return;
	}

	base_addr = (port == CSI_TX_0) ? CSITX0_BASE_ADDR :
					CSITX1_BASE_ADDR;
	pllconf = i2c_rd32(sd, base_addr+PLLCONF);
	pllconf_new = SET_PLL_PRD(pdata->pll_prd) |
	SET_PLL_FBD(pdata->pll_fbd);

	hsck = (pdata->refclk_hz / pdata->pll_prd) *
			pdata->pll_fbd;

	if (hsck > 500000000)
		pll_frs = 0x0;
	else if (hsck > 250000000)
		pll_frs = 0x1;
	else if (hsck > 125000000)
		pll_frs = 0x2;
	else
		pll_frs = 0x3;

	v4l2_dbg(1, debug, sd, "%s: Updating PLL clock of CSI TX%d\n",
			__func__, port-1);

	i2c_wr32(sd, base_addr+PLLCONF,
			pllconf_new | SET_PLL_FRS(pll_frs));
}

static void tc358840_set_ref_clk(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;

	u32 sys_freq;
	u32 lock_ref_freq;
	u32 nco;
	u16 csc;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	BUG_ON((pdata->refclk_hz < 40000000) || (pdata->refclk_hz > 50000000));

	/* System Frequency */
	sys_freq = pdata->refclk_hz / 10000;
	i2c_wr8(sd, SYS_FREQ0, sys_freq & 0x00FF);
	i2c_wr8(sd, SYS_FREQ1, (sys_freq & 0xFF00) >> 8);

	/* Audio System Frequency */
	lock_ref_freq = pdata->refclk_hz / 100;
	i2c_wr8(sd, LOCK_REF_FREQA, lock_ref_freq & 0xFF);
	i2c_wr8(sd, LOCK_REF_FREQB, (lock_ref_freq >> 8) & 0xFF);
	i2c_wr8(sd, LOCK_REF_FREQC, (lock_ref_freq >> 16) & 0x0F);

	/* Audio PLL */
	i2c_wr8(sd, NCO_F0_MOD, MASK_NCO_F0_MOD_REG);
	/* 6.144 * 2^28 = 1649267442 */
	nco = (1649267442 / (pdata->refclk_hz / 1000000));
	i2c_wr8(sd, NCO_48F0A, nco & 0xFF);
	i2c_wr8(sd, NCO_48F0B, (nco >> 8) & 0xFF);
	i2c_wr8(sd, NCO_48F0C, (nco >> 16) & 0xFF);
	i2c_wr8(sd, NCO_48F0D, (nco >> 24) & 0xFF);

	/* Color Space Conversion */
	csc = pdata->refclk_hz / 10000;
	i2c_wr8(sd, SCLK_CSC0, csc & 0xFF);
	i2c_wr8(sd, SCLK_CSC1, (csc >> 8) & 0xFF);
}

static void tc358840_set_csi_mbus_config(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	switch (state->mbus_fmt_code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
		v4l2_dbg(2, debug, sd, "%s: YCbCr 422 16-bit\n", __func__);

		i2c_wr8(sd, VOUT_FMT, MASK_OUTFMT_422 | MASK_422FMT_NORMAL);
		i2c_wr8(sd, VOUT_FIL, MASK_422FIL_3_TAP_444 |
			MASK_444FIL_2_TAP);
		i2c_wr8(sd, VOUT_SYNC0, MASK_MODE_2);
		i2c_wr8(sd, VOUT_CSC, MASK_CSC_MODE_BUILTIN |
			MASK_COLOR_601_YCBCR_LIMITED);
		i2c_wr16_and_or(sd, CONFCTL0, ~(MASK_YCBCRFMT),
			MASK_YCBCRFMT_YCBCR422_8);
		i2c_wr16(sd, CONFCTL1, 0x0);
		break;

	case MEDIA_BUS_FMT_RGB888_1X24:
		v4l2_dbg(2, debug, sd, "%s: RGB 888 24-bit\n", __func__);

		i2c_wr8(sd, VOUT_FMT, MASK_OUTFMT_444_RGB);
		i2c_wr8(sd, VOUT_FIL, MASK_422FIL_3_TAP_444 |
			MASK_444FIL_2_TAP);
		i2c_wr8(sd, VOUT_SYNC0, MASK_MODE_2);
		i2c_wr8(sd, VOUT_CSC, MASK_CSC_MODE_BUILTIN |
			MASK_COLOR_RGB_LIMITED);
		i2c_wr16_and_or(sd, CONFCTL0, ~(MASK_YCBCRFMT), 0x0);
		i2c_wr16_and_or(sd, CONFCTL1, 0x0, MASK_TX_OUT_FMT_RGB888);
		break;

	default:
		v4l2_dbg(2, debug, sd, "%s: Unsupported format code 0x%x\n",
				__func__, state->mbus_fmt_code);
		break;
	}
}

static unsigned tc358840_num_csi_lanes_needed(struct v4l2_subdev *sd)
{

	/* TODO: Check if this can be useful */
#if 0
	struct tc358840_state *state = to_state(sd);
	struct v4l2_bt_timings *bt = &state->timings.bt;
	struct tc358840_platform_data *pdata = &state->pdata;
	u32 bits_pr_pixel =
		(state->mbus_fmt_code == MEDIA_BUS_FMT_UYVY8_1X16) ?  16 : 24;
	u32 bps = bt->width * bt->height * fps(bt) * bits_pr_pixel;
	u32 bps_pr_lane = (pdata->refclk_hz / pdata->pll_prd) * pdata->pll_fbd;

	return DIV_ROUND_UP(bps, bps_pr_lane);
#endif

	/* Always use 4 lanes for one CSI */
	return 4;
}

static void tc358840_set_csi(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;
#if 0
	struct v4l2_bt_timings *bt = &state->timings.bt;
#endif
	unsigned lanes = tc358840_num_csi_lanes_needed(sd);

	enum tc358840_csi_port port;
	u16 base_addr;

	v4l2_dbg(3, debug, sd, "%s:\n", __func__);

	tc358840_reset(sd, MASK_CTXRST);

	for (port = CSI_TX_0; port <= CSI_TX_1; port++) {
		base_addr = (port == CSI_TX_0) ? CSITX0_BASE_ADDR :
						 CSITX1_BASE_ADDR;

		if (pdata->csi_port != CSI_TX_BOTH &&
		    pdata->csi_port != port) {
			v4l2_dbg(1, debug, sd,
				"%s: Disabling CSI TX%d\n", __func__, port-1);

			/* Disable CSI lanes (High Z)*/
			i2c_wr32_and_or(sd, base_addr+LANEEN,
				~(MASK_CLANEEN), 0);
			continue;
		}

		v4l2_dbg(1, debug, sd,
			"%s: Enabling CSI TX%d\n", __func__, port-1);

		/* (0x0108) */
		i2c_wr32(sd, base_addr+CSITX_CLKEN, MASK_CSITX_EN);
		/*
		 * PLL has to be enabled between CSITX_CLKEN and
		 * LANEEN (0x02AC)
		 */
		tc358840_set_pll(sd, port);
		/* (0x02A0) */
		i2c_wr32_and_or(sd, base_addr+MIPICLKEN,
			~(MASK_MP_CKEN), MASK_MP_ENABLE);
		usleep_range(10000, 11000);
		/* (0x02A0) */
		i2c_wr32(sd, base_addr+MIPICLKEN,
			MASK_MP_CKEN | MASK_MP_ENABLE);
		/* (0x010C) */
		i2c_wr32(sd, base_addr+PPICLKEN, MASK_HSTXCLKEN);
		/* (0x0118) */
		i2c_wr32(sd, base_addr+LANEEN,
			(lanes & MASK_LANES) | MASK_CLANEEN);

		/* (0x0120) */
		i2c_wr32(sd, base_addr+LINEINITCNT, pdata->lineinitcnt);

		/*
		 * TODO: Check if this is the correct register
		 * (0x0150)
		 */
		i2c_rd32(sd, base_addr+FUNCMODE);
		/* (0x0254) */
		i2c_wr32(sd, base_addr+LPTXTIMECNT, pdata->lptxtimecnt);
		/* (0x0258) */
		i2c_wr32(sd, base_addr+TCLK_HEADERCNT,
			pdata->tclk_headercnt);
		/* (0x025C) */
		i2c_wr32(sd, base_addr+TCLK_TRAILCNT,
			pdata->tclk_trailcnt);
		/* (0x0260) */
		i2c_wr32(sd, base_addr+THS_HEADERCNT,
			pdata->ths_headercnt);
		/* (0x0264) */
		i2c_wr32(sd, base_addr+TWAKEUP, pdata->twakeup);
		/* (0x0268) */
		i2c_wr32(sd, base_addr+TCLK_POSTCNT,
			pdata->tclk_postcnt);
		/* (0x026C) */
		i2c_wr32(sd, base_addr+THS_TRAILCNT,
			pdata->ths_trailcnt);

		/* (0x0270) */
		i2c_wr32(sd, base_addr+HSTXVREGCNT, pdata->hstxvregcnt);

		/* (0x0274) */
		i2c_wr32(sd, base_addr+HSTXVREGEN,
			((lanes > 0) ? MASK_CLM_HSTXVREGEN : 0x0) |
			((lanes > 0) ? MASK_D0M_HSTXVREGEN : 0x0) |
			((lanes > 1) ? MASK_D1M_HSTXVREGEN : 0x0) |
			((lanes > 2) ? MASK_D2M_HSTXVREGEN : 0x0) |
			((lanes > 3) ? MASK_D3M_HSTXVREGEN : 0x0));

		/*
		 * Finishing configuration by setting CSITX to start
		 * (0X011C)
		 */
		i2c_wr32(sd, base_addr+CSITX_START, 0x00000001);

		i2c_rd32(sd, base_addr+CSITX_INTERNAL_STAT);
	}
}

static void tc358840_set_hdmi_phy(struct v4l2_subdev *sd)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/* Reset PHY */
	tc358840_reset_phy(sd);

	/* Set PHY to manual */
	i2c_wr8(sd, PHY_CTL, MASK_48_MHZ);

	/* Enable PHY */
	i2c_wr8_and_or(sd, PHY_ENB, ~MASK_ENABLE_PHY, 0x0);
	i2c_wr8_and_or(sd, PHY_ENB, ~MASK_ENABLE_PHY, MASK_ENABLE_PHY);

	/* Enable Audio PLL */
	i2c_wr8(sd, APPL_CTL, MASK_APLL_CPCTL_NORMAL | MASK_APLL_ON);

	/* Enable DDC IO */
	i2c_wr8(sd, DDCIO_CTL, MASK_DDC_PWR_ON);
}

static void tc358840_set_hdmi_audio(struct v4l2_subdev *sd)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	i2c_wr8(sd, FORCE_MUTE, 0x00);
	i2c_wr8(sd, AUTO_CMD0, MASK_AUTO_MUTE7 | MASK_AUTO_MUTE6 |
			MASK_AUTO_MUTE5 | MASK_AUTO_MUTE4 |
			MASK_AUTO_MUTE1 | MASK_AUTO_MUTE0);
	i2c_wr8(sd, AUTO_CMD1, MASK_AUTO_MUTE9);
	i2c_wr8(sd, AUTO_CMD2, MASK_AUTO_PLAY3 | MASK_AUTO_PLAY2);
	i2c_wr8(sd, BUFINIT_START, SET_BUFINIT_START_MS(500));
	i2c_wr8(sd, FS_MUTE, 0x00);
	i2c_wr8(sd, FS_IMODE, MASK_NLPCM_SMODE | MASK_FS_SMODE);
	i2c_wr8(sd, ACR_MODE, MASK_CTS_MODE);
	i2c_wr8(sd, ACR_MDF0, MASK_ACR_L2MDF_1976_PPM | MASK_ACR_L1MDF_976_PPM);
	i2c_wr8(sd, ACR_MDF1, MASK_ACR_L3MDF_3906_PPM);
	/*
	 * TODO: Set output data bit length (currently 16 bit, 8 bit discarded)
	 */
	i2c_wr8(sd, SDO_MODE1, MASK_SDO_FMT_I2S);
	i2c_wr8(sd, DIV_MODE, SET_DIV_DLY_MS(100));
	i2c_wr16_and_or(sd, CONFCTL0, 0xFFFF, MASK_AUDCHNUM_2 |
			MASK_AUDOUTSEL_I2S | MASK_AUTOINDEX);
}

static void tc358840_set_hdmi_info_frame_mode(struct v4l2_subdev *sd)
{
	v4l2_dbg(3, debug, sd, "%s(): DUMMY\n", __func__);

	/* TODO: Check which registers are needed/available */
#if 0
	i2c_wr8(sd, PK_INT_MODE, MASK_ISRC2_INT_MODE | MASK_ISRC_INT_MODE |
			MASK_ACP_INT_MODE | MASK_VS_INT_MODE |
			MASK_SPD_INT_MODE | MASK_MS_INT_MODE |
			MASK_AUD_INT_MODE | MASK_AVI_INT_MODE);
	i2c_wr8(sd, NO_PKT_LIMIT, 0x2c);
	i2c_wr8(sd, NO_PKT_CLR, 0x53);
	i2c_wr8(sd, ERR_PK_LIMIT, 0x01);
	i2c_wr8(sd, NO_PKT_LIMIT2, 0x30);
	i2c_wr8(sd, NO_GDB_LIMIT, 0x10);
#endif
}

static void tc358840_initial_setup(struct v4l2_subdev *sd)
{
	static struct v4l2_dv_timings default_timing = V4L2_DV_BT_CEA_1920X1080P60;
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/* *** Reset *** */
	enable_stream(sd, false);

	tc358840_sleep_mode(sd, false);
	tc358840_reset(sd, MASK_RESET_ALL);

	tc358840_init_interrupts(sd);

	/* *** Init CSI *** */
	tc358840_s_dv_timings(sd, &default_timing);

	tc358840_set_ref_clk(sd);

	i2c_wr8_and_or(sd, DDC_CTL, ~MASK_DDC5V_MODE,
		       pdata->ddc5v_delay & MASK_DDC5V_MODE);

	i2c_wr8_and_or(sd, EDID_MODE, ~MASK_EDID_MODE_ALL, MASK_RAM_EDDC);

	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_CTL0, 0);

	tc358840_set_hdmi_phy(sd);

	tc358840_set_hdmi_hdcp(sd, pdata->enable_hdcp);
	tc358840_set_hdmi_audio(sd);
	tc358840_set_hdmi_info_frame_mode(sd);

	/* All CE and IT formats are detected as RGB full range in DVI mode */
	i2c_wr8_and_or(sd, VI_MODE, ~MASK_RGB_DVI, 0);
}

/* --------------- IRQ --------------- */

static void tc358840_format_change(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct v4l2_dv_timings timings;
	static const struct v4l2_event tc358840_ev_fmt = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	if (tc358840_get_detected_timings(sd, &timings)) {
		enable_stream(sd, false);

		v4l2_info(sd, "%s: No Signal\n", __func__);
	} else {
		if (!tegra_v4l2_match_dv_timings(&state->timings,
				&timings, 0, false))
			enable_stream(sd, false);

		v4l2_print_dv_timings(sd->name,
				"tc358840_format_change: New format: ",
				&timings, false);
	}

	/* Application gets notified after CSI Tx's are reset */
	if (sd->devnode)
		v4l2_subdev_notify_event(sd, &tc358840_ev_fmt);
}

static void tc358840_init_interrupts(struct v4l2_subdev *sd)
{
	u16 i;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/* clear interrupt status registers */
	for (i = SYS_INT; i <= MISC_INT; i++) {
		/* No interrupt register at Address 0x850A */
		if (i != 0x850A)
			i2c_wr8(sd, i, 0xFF);
	}

	/* Clear and disable all interrupts */
	i2c_wr16(sd, INTSTATUS, MASK_INT_STATUS_MASK_ALL);
	i2c_wr16(sd, INTSTATUS, 0x0);

	i2c_wr16(sd, INTMASK, MASK_INT_STATUS_MASK_ALL);
}

static void tc358840_enable_interrupts(struct v4l2_subdev *sd,
		bool cable_connected)
{
	v4l2_dbg(2, debug, sd, "%s: cable connected = %d\n", __func__,
			cable_connected);

	if (cable_connected) {
		i2c_wr8(sd, SYS_INTM, ~(MASK_DDC | MASK_DVI |
					MASK_HDMI) & 0xFF);
		i2c_wr8(sd, SYS_INTM, ~(MASK_DDC) & 0xFF);
		i2c_wr8(sd, CLK_INTM, ~MASK_IN_DE_CHG);
		i2c_wr8(sd, CBIT_INTM, ~(MASK_CBIT_FS | MASK_AF_LOCK |
					MASK_AF_UNLOCK) & 0xFF);
		i2c_wr8(sd, AUDIO_INTM, ~MASK_BUFINIT_END);
		i2c_wr8(sd, MISC_INTM, ~MASK_SYNC_CHG);
	} else {
		i2c_wr8(sd, SYS_INTM, ~MASK_DDC & 0xFF);
		i2c_wr8(sd, CLK_INTM, 0xFF);
		i2c_wr8(sd, CBIT_INTM, 0xFF);
		i2c_wr8(sd, AUDIO_INTM, 0xFF);
		i2c_wr8(sd, MISC_INTM, 0xFF);
	}
}

static void tc358840_hdmi_audio_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 audio_int_mask = i2c_rd8(sd, AUDIO_INTM);
	u8 audio_int = i2c_rd8(sd, AUDIO_INT) & ~audio_int_mask;

	i2c_wr8(sd, AUDIO_INT, audio_int);

	v4l2_dbg(3, debug, sd, "%s: AUDIO_INT = 0x%02x\n", __func__, audio_int);

	tc358840_s_ctrl_audio_sampling_rate(sd);
	tc358840_s_ctrl_audio_present(sd);
}

static void tc358840_hdmi_misc_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	struct tc358840_state *state = to_state(sd);
	u8 misc_int_mask = i2c_rd8(sd, MISC_INTM);
	u8 misc_int = i2c_rd8(sd, MISC_INT) & ~misc_int_mask;

	i2c_wr8(sd, MISC_INT, misc_int);

	v4l2_dbg(3, debug, sd, "%s: MISC_INT = 0x%02x\n", __func__, misc_int);

	if (misc_int & MASK_SYNC_CHG) {
		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */

		state->format_changed = true;

		misc_int &= ~MASK_SYNC_CHG;
		if (handled)
			*handled = true;
	}

	if (misc_int) {
		v4l2_err(sd, "%s: Unhandled MISC_INT interrupts: 0x%02x\n",
				__func__, misc_int);
	}
}

static void tc358840_hdmi_cbit_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 cbit_int_mask = i2c_rd8(sd, CBIT_INTM);
	u8 cbit_int = i2c_rd8(sd, CBIT_INT) & ~cbit_int_mask;

	i2c_wr8(sd, CBIT_INT, cbit_int);

	v4l2_dbg(3, debug, sd, "%s: CBIT_INT = 0x%02x\n", __func__, cbit_int);

	if (cbit_int & MASK_CBIT_FS) {

		v4l2_dbg(1, debug, sd, "%s: Audio sample rate changed\n",
				__func__);
		tc358840_s_ctrl_audio_sampling_rate(sd);

		cbit_int &= ~MASK_CBIT_FS;
		if (handled)
			*handled = true;
	}

	if (cbit_int & (MASK_AF_LOCK | MASK_AF_UNLOCK)) {

		v4l2_dbg(1, debug, sd, "%s: Audio present changed\n",
				__func__);
		tc358840_s_ctrl_audio_present(sd);

		cbit_int &= ~(MASK_AF_LOCK | MASK_AF_UNLOCK);
		if (handled)
			*handled = true;
	}

	if (cbit_int) {
		v4l2_err(sd, "%s: Unhandled CBIT_INT interrupts: 0x%02x\n",
				__func__, cbit_int);
	}
}

static void tc358840_hdmi_clk_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	struct tc358840_state *state = to_state(sd);
	u8 clk_int_mask = i2c_rd8(sd, CLK_INTM);
	u8 clk_int = i2c_rd8(sd, CLK_INT) & ~clk_int_mask;

	/* Bit 7 and bit 6 are set even when they are masked */
	i2c_wr8(sd, CLK_INT, clk_int | 0x80 | MASK_OUT_H_CHG);

	v4l2_dbg(3, debug, sd, "%s: CLK_INT = 0x%02x\n", __func__, clk_int);

	if (clk_int & (MASK_IN_DE_CHG)) {

		v4l2_dbg(1, debug, sd, "%s: DE size or position has changed\n",
				__func__);

		/* TODO: Check if also true for tc358840 */
		/* If the source switch to a new resolution with the same pixel
		 * frequency as the existing (e.g. 1080p25 -> 720p50), the
		 * I_SYNC_CHG interrupt is not always triggered, while the
		 * I_IN_DE_CHG interrupt seems to work fine. FMT_CHANGE
		 * notifications are only sent when the signal is stable to
		 * reduce the number of notifications. */
		if (!no_signal(sd) && !no_sync(sd))
			state->format_changed = true;

		clk_int &= ~MASK_IN_DE_CHG;
		if (handled)
			*handled = true;
	}

	if (clk_int) {
		v4l2_err(sd, "%s: Unhandled CLK_INT interrupts: 0x%02x\n",
				__func__, clk_int);
	}
}

static void tc358840_hdmi_sys_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	struct tc358840_state *state = to_state(sd);
	u8 sys_int_mask = i2c_rd8(sd, SYS_INTM);
	u8 sys_int = i2c_rd8(sd, SYS_INT) & ~sys_int_mask;

	i2c_wr8(sd, SYS_INT, sys_int);

	v4l2_dbg(3, debug, sd, "%s: SYS_INT = 0x%02x\n", __func__, sys_int);

	if (sys_int & MASK_DDC) {
		bool tx_5v = tx_5v_power_present(sd);

		v4l2_dbg(1, debug, sd, "%s: Tx 5V power present: %s\n",
				__func__, tx_5v ?  "yes" : "no");

		if (tx_5v) {
			tc358840_enable_edid(sd);
		} else {
			tc358840_enable_interrupts(sd, false);
			tc358840_disable_edid(sd);
			memset(&state->timings, 0, sizeof(state->timings));
			tc358840_erase_bksv(sd);
			tc358840_update_controls(sd);
		}

		sys_int &= ~MASK_DDC;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_DVI) {
		v4l2_dbg(1, debug, sd, "%s: HDMI->DVI change detected\n",
				__func__);

		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */
		if (no_sync(sd) || no_signal(sd))
			state->format_changed = true;

		sys_int &= ~MASK_DVI;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_HDMI) {
		v4l2_dbg(1, debug, sd, "%s: DVI->HDMI change detected\n",
				__func__);

		/* TODO: Check if reg is required. Reg not found in Rev. 1.5 */
#if 0
		i2c_wr8(sd, ANA_CTL, MASK_APPL_PCSX_NORMAL | MASK_ANALOG_ON);
#endif
		sys_int &= ~MASK_HDMI;
		if (handled)
			*handled = true;
	}

	if (sys_int) {
		v4l2_err(sd, "%s: Unhandled SYS_INT interrupts: 0x%02x\n",
				__func__, sys_int);
	}
}

/* --------------- CORE OPS --------------- */

static int tc358840_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	struct tc358840_state *state = to_state(sd);
	u16 intstatus;
	unsigned retry = 10;

	//disable_irq_nosync(state->pdata.interrupt);
	intstatus = i2c_rd16(sd, INTSTATUS);

	v4l2_dbg(1, debug, sd, "%s: IntStatus = 0x%04X\n", __func__, intstatus);

	/*
	 * Need to figure out why these msleeps are needed, and which of these
	 * are needed. Without msleeps the interrupts just stop.
	 */
	usleep_range(500, 1000);
	state->format_changed = false;
	if (intstatus & MASK_HDMI_INT) {
		u8 hdmi_int0;
		u8 hdmi_int1;
retry:
		retry--;
		hdmi_int0 = i2c_rd8(sd, HDMI_INT0);
		usleep_range(500, 1000);
		hdmi_int1 = i2c_rd8(sd, HDMI_INT1);
		usleep_range(500, 1000);

		if (hdmi_int0 & MASK_MISC)
			tc358840_hdmi_misc_int_handler(sd, handled);
		if (hdmi_int1 & MASK_ACBIT)
			tc358840_hdmi_cbit_int_handler(sd, handled);
		if (hdmi_int1 & MASK_CLK)
			tc358840_hdmi_clk_int_handler(sd, handled);
		if (hdmi_int1 & MASK_SYS)
			tc358840_hdmi_sys_int_handler(sd, handled);
		if (hdmi_int1 & MASK_AUD)
			tc358840_hdmi_audio_int_handler(sd, handled);

		usleep_range(500, 1000);
		i2c_wr16(sd, INTSTATUS, MASK_HDMI_INT);
		intstatus &= ~MASK_HDMI_INT;
		usleep_range(500, 1000);

		/* Display unhandled HDMI interrupts */
		hdmi_int0 = i2c_rd8(sd, HDMI_INT0);
		if (hdmi_int0) {
			v4l2_dbg(1, debug, sd,
				"%s: Unhandled HDMI_INT0 interrupts: 0x%02X\n",
				__func__, hdmi_int0);
			if (retry)
				goto retry;
		}
		usleep_range(500, 1000);
		hdmi_int1 = i2c_rd8(sd, HDMI_INT1);
		if (hdmi_int1) {
			v4l2_dbg(1, debug, sd,
				"%s: Unhandled HDMI_INT1 interrupts: 0x%02X\n",
				__func__, hdmi_int1);
			if (retry)
				goto retry;
		}
	}

	if (handled && state->format_changed) {
		tc358840_format_change(sd);
		if (no_sync(sd) || no_signal(sd)) {
			tc358840_reset_phy(sd);
			tc358840_erase_bksv(sd);
		}
	}

	if (intstatus & MASK_CSITX0_INT) {
		v4l2_dbg(3, debug, sd, "%s: MASK_CSITX0_INT\n", __func__);

		i2c_wr16(sd, INTSTATUS, MASK_CSITX0_INT);
		intstatus &= ~MASK_CSITX0_INT;
	}

	if (intstatus & MASK_CSITX1_INT) {
		v4l2_dbg(3, debug, sd, "%s: MASK_CSITX1_INT\n", __func__);

		i2c_wr16(sd, INTSTATUS, MASK_CSITX1_INT);
		intstatus &= ~MASK_CSITX1_INT;
	}

	if (intstatus) {
		v4l2_dbg(1, debug, sd,
			"%s: Unhandled IntStatus interrupts: 0x%04x\n",
			__func__, intstatus);
	}
	//enable_irq(state->pdata.interrupt);
	return 0;
}

static void tc358840_process_isr(struct work_struct *work)
{
	struct tc358840_state *state = container_of(work,
		struct tc358840_state, process_isr);
	struct v4l2_subdev *sd = &state->sd;
	bool handled;

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	mutex_lock(&state->isr_lock);
	tc358840_isr(sd, 0, &handled);
	mutex_unlock(&state->isr_lock);
}

static irqreturn_t tc358840_irq_handler(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	struct tc358840_state *state = to_state(sd);

	queue_work(state->work_queues, &state->process_isr);

	return IRQ_HANDLED;
}

/* --------------- PAD OPS --------------- */

static int tc358840_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct tc358840_state *state = to_state(sd);
	u8 vout_csc = i2c_rd8(sd, VOUT_CSC);
	struct v4l2_mbus_framefmt *fmt;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	format->format.code = state->mbus_fmt_code;
	format->format.width = state->timings.bt.width;
	format->format.height = state->timings.bt.height;
	format->format.field = V4L2_FIELD_NONE;

	switch (vout_csc & MASK_COLOR) {
	case MASK_COLOR_RGB_FULL:
	case MASK_COLOR_RGB_LIMITED:
		format->format.colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case MASK_COLOR_601_YCBCR_FULL:
	case MASK_COLOR_601_YCBCR_LIMITED:
		format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
		break;
	case MASK_COLOR_709_YCBCR_FULL:
	case MASK_COLOR_709_YCBCR_LIMITED:
		format->format.colorspace = V4L2_COLORSPACE_REC709;
		break;
	default:
		format->format.colorspace = 0;
		break;
	}

	fmt = &format->format;
	v4l2_dbg(3, debug, sd,
		"%s(): width=%d, height=%d, code=0x%08X, field=%d\n",
		__func__, fmt->width, fmt->height, fmt->code, fmt->field);

	return 0;
}

static int tc358840_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct tc358840_state *state = to_state(sd);
	u32 code = format->format.code; /* is overwritten by get_fmt */
	int ret = tc358840_get_fmt(sd, cfg, format);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	format->format.code = code;

	if (ret)
		return ret;

	switch (code) {
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_UYVY8_1X16:
		break;
	default:
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	v4l2_dbg(3, debug, sd, "%s(): format->which=%d\n",
		__func__, format->which);

	state->mbus_fmt_code = format->format.code;
	enable_stream(sd, false);
	tc358840_set_csi(sd);
	tc358840_set_csi_mbus_config(sd);

	return 0;
}

static int tc358840_g_edid(struct v4l2_subdev *sd,
		struct v4l2_subdev_edid *edid)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = state->edid_blocks_written;
		return 0;
	}

	if (state->edid_blocks_written == 0)
		return -ENODATA;

	if (edid->start_block >= state->edid_blocks_written ||
			edid->blocks == 0)
		return -EINVAL;

	if (edid->start_block + edid->blocks > state->edid_blocks_written)
		edid->blocks = state->edid_blocks_written - edid->start_block;

	i2c_rd(sd, EDID_RAM + (edid->start_block * EDID_BLOCK_SIZE), edid->edid,
		   edid->blocks * EDID_BLOCK_SIZE);

	return 0;
}

static int tc358840_s_edid(struct v4l2_subdev *sd,
				struct v4l2_subdev_edid *edid)
{
	struct tc358840_state *state = to_state(sd);
	u16 edid_len = edid->blocks * EDID_BLOCK_SIZE;
	int i;

	v4l2_dbg(2, debug, sd, "%s, pad %d, start block %d, blocks %d\n",
		 __func__, edid->pad, edid->start_block, edid->blocks);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks > EDID_NUM_BLOCKS_MAX) {
		edid->blocks = EDID_NUM_BLOCKS_MAX;
		return -E2BIG;
	}

	tc358840_disable_edid(sd);

	i2c_wr8(sd, EDID_LEN1, edid_len & 0xFF);
	i2c_wr8(sd, EDID_LEN2, edid_len >> 8);

	if (edid->blocks == 0) {
		state->edid_blocks_written = 0;
		return 0;
	}

	for (i = 0; i < 256; i++) {
		unsigned int val;
		val = edid->edid[i];
		i2c_wr8(sd, EDID_RAM + i, val);
	}

	state->edid_blocks_written = edid->blocks;

	if (tx_5v_power_present(sd))
		tc358840_enable_edid(sd);

	return 0;
}

static int tc358840_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

/* --------------- VIDEO OPS --------------- */

static int tc358840_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;
	*status |= no_sync(sd) ? V4L2_IN_ST_NO_SYNC : 0;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

static int tc358840_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (!timings)
		return -EINVAL;

	if (debug)
		v4l2_print_dv_timings(sd->name, "tc358840_s_dv_timings: ",
				timings, false);

	if (tegra_v4l2_match_dv_timings(&state->timings, timings, 0, false)) {
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
		return 0;
	}

	if (!v4l2_valid_dv_timings(timings, &tc358840_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	state->timings = *timings;

	enable_stream(sd, false);
	tc358840_set_csi(sd);
	tc358840_set_splitter(sd);

	return 0;
}

static int tc358840_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	*timings = state->timings;

	return 0;
}

static int tc358840_enum_dv_timings(struct v4l2_subdev *sd,
				    struct v4l2_enum_dv_timings *timings)
{
	v4l2_dbg(3, debug, sd, "%s(): DUMMY\n", __func__);

	if (timings->pad != 0)
		return -EINVAL;

	return v4l2_enum_dv_timings_cap(timings,
			&tc358840_timings_cap, NULL, NULL);
}

static int tc358840_query_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	int ret;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	ret = tc358840_get_detected_timings(sd, timings);
	if (ret)
		return ret;

	if (debug)
		v4l2_print_dv_timings(sd->name, "tc358840_query_dv_timings: ",
			timings, false);
	if (!v4l2_valid_dv_timings(timings, &tc358840_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}

static int tc358840_dv_timings_cap(struct v4l2_subdev *sd,
				   struct v4l2_dv_timings_cap *cap)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (cap->pad != 0)
		return -EINVAL;

	*cap = tc358840_timings_cap;

	return 0;
}

static int tc358840_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *cfg)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	cfg->type = V4L2_MBUS_CSI2;

	/* Support for non-continuous CSI-2 clock is missing in the driver */
	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK | V4L2_MBUS_CSI2_CHANNEL_0;

	switch (tc358840_num_csi_lanes_in_use(sd)) {
	case 1:
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
		break;
	case 2:
		cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
		break;
	case 3:
		cfg->flags |= V4L2_MBUS_CSI2_3_LANE;
		break;
	case 4:
		cfg->flags |= V4L2_MBUS_CSI2_4_LANE;
		break;
	default:
		return -EINVAL;
	}

	v4l2_dbg(2, debug, sd, "%s: Lanes: 0x%02X\n",
		__func__, cfg->flags & 0x0F);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int tc358840_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	switch (reg->size) {
	case 1:
		reg->val = i2c_rd8(sd, reg->reg);
		break;
	case 2:
		reg->val = i2c_rd16(sd, reg->reg);
		break;
	case 4:
	default:
		reg->val = i2c_rd32(sd, reg->reg);
		break;
	}
	return 0;
}

static int tc358840_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	switch (reg->size) {
	case 1:
		i2c_wr8(sd, reg->reg, reg->val);
		break;
	case 2:
		i2c_wr16(sd, reg->reg, reg->val);
		break;
	case 4:
	default:
		i2c_wr32(sd, reg->reg, reg->val);
		break;
	}
	return 0;
}
#endif

static int tc358840_log_status(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct v4l2_dv_timings timings;
	u8 hdmi_sys_status =  i2c_rd8(sd, SYS_STATUS);
	u16 sysctl = i2c_rd16(sd, SYSCTL);
	u8 vi_status3 = i2c_rd8(sd, VI_STATUS3);
	const int deep_color_mode[4] = { 8, 10, 12, 16 };
	static const char * const input_color_space[] = {
		"RGB", "YCbCr 601", "Adobe RGB", "YCbCr 709", "NA (4)",
		"xvYCC 601", "NA(6)", "xvYCC 709", "NA(8)", "sYCC601",
		"NA(10)", "NA(11)", "NA(12)", "Adobe YCC 601"};

	v4l2_ctrl_subdev_log_status(sd);
	v4l2_info(sd, "-----Chip status-----\n");
	v4l2_info(sd, "Chip ID: 0x%02x\n",
			(i2c_rd16(sd, CHIPID_ADDR) & MASK_CHIPID) >> 8);
	v4l2_info(sd, "Chip revision: 0x%02x\n",
			i2c_rd16(sd, CHIPID_ADDR) & MASK_REVID);
	v4l2_info(sd, "Reset: IR: %d, CEC: %d, CSI TX: %d, HDMI: %d\n",
			!!(sysctl & MASK_IRRST),
			!!(sysctl & MASK_CECRST),
			!!(sysctl & MASK_CTXRST),
			!!(sysctl & MASK_HDMIRST));
	v4l2_info(sd, "Sleep mode: %s\n", sysctl & MASK_SLEEP ? "on" : "off");
	v4l2_info(sd, "Cable detected (+5V power): %s\n",
			hdmi_sys_status & MASK_S_DDC5V ? "yes" : "no");
	v4l2_info(sd, "DDC lines enabled: %s\n",
			(i2c_rd8(sd, EDID_MODE) & MASK_EDID_MODE_ALL) ?
			"yes" : "no");
	v4l2_info(sd, "Hotplug enabled: %s\n",
			(i2c_rd8(sd, HPD_CTL) & MASK_HPD_OUT0) ?
			"yes" : "no");
	v4l2_info(sd, "CEC enabled: %s\n",
			(i2c_rd16(sd, CECEN) & MASK_CECEN) ?  "yes" : "no");
	v4l2_info(sd, "-----Signal status-----\n");
	v4l2_info(sd, "TMDS signal detected: %s\n",
			hdmi_sys_status & MASK_S_TMDS ? "yes" : "no");
	v4l2_info(sd, "Stable sync signal: %s\n",
			hdmi_sys_status & MASK_S_SYNC ? "yes" : "no");
	v4l2_info(sd, "PHY PLL locked: %s\n",
			hdmi_sys_status & MASK_S_PHY_PLL ? "yes" : "no");
	v4l2_info(sd, "PHY DE detected: %s\n",
			hdmi_sys_status & MASK_S_PHY_SCDT ? "yes" : "no");

	if (tc358840_get_detected_timings(sd, &timings)) {
		v4l2_info(sd, "No video detected\n");
	} else {
		v4l2_print_dv_timings(sd->name, "Detected format: ", &timings,
				true);
	}
	v4l2_print_dv_timings(sd->name, "Configured format: ", &state->timings,
			true);

	v4l2_info(sd, "-----CSI-TX status-----\n");
	v4l2_info(sd, "Lanes needed: %d\n",
			tc358840_num_csi_lanes_needed(sd));
	v4l2_info(sd, "Lanes in use: %d\n",
			tc358840_num_csi_lanes_in_use(sd));
	v4l2_info(sd, "Splitter %sabled\n",
		  (i2c_rd16(sd, SPLITTX0_CTRL) & MASK_SPBP) ? "dis" : "en");
	/*
	v4l2_info(sd, "Waiting for particular sync signal: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_WSYNC) ?
			"yes" : "no");
	v4l2_info(sd, "Transmit mode: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_TXACT) ?
			"yes" : "no");
	v4l2_info(sd, "Receive mode: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_RXACT) ?
			"yes" : "no");
	v4l2_info(sd, "Stopped: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_HLT) ?
			"yes" : "no");*/
	v4l2_info(sd, "Color space: %s\n",
			state->mbus_fmt_code == MEDIA_BUS_FMT_UYVY8_1X16 ?
			"YCbCr 422 16-bit" :
			state->mbus_fmt_code == MEDIA_BUS_FMT_RGB888_1X24 ?
			"RGB 888 24-bit" : "Unsupported");

	v4l2_info(sd, "-----%s status-----\n", is_hdmi(sd) ? "HDMI" : "DVI-D");
	v4l2_info(sd, "HDCP encrypted content: %s\n",
			hdmi_sys_status & MASK_S_HDCP ? "yes" : "no");
	v4l2_info(sd, "Input color space: %s %s range\n",
			input_color_space[(vi_status3 & MASK_S_V_COLOR) >> 1],
			(vi_status3 & MASK_LIMITED) ? "limited" : "full");
	if (!is_hdmi(sd))
		return 0;
	v4l2_info(sd, "AV Mute: %s\n", hdmi_sys_status & MASK_S_AVMUTE ? "on" :
			"off");
	v4l2_info(sd, "Deep color mode: %d-bits per channel\n",
			deep_color_mode[(i2c_rd8(sd, VI_STATUS1) &
				MASK_S_DEEPCOLOR) >> 2]);
	print_avi_infoframe(sd);

	return 0;
}

static int tc358840_s_stream(struct v4l2_subdev *sd, int enable)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	return enable_stream(sd, enable);
}

static int tc358840_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	if (code->index >= 2)
		return -EINVAL;

	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_UYVY8_1X16;
		break;
	case 1:
		code->code = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	}
	return 0;
}

static int tc358840_enum_framesizes(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
{
	const struct camera_common_frmfmt *frmfmt = tc358840_frmfmt;
	int num_frmfmt = ARRAY_SIZE(tc358840_frmfmt);

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	if (fse->code != V4L2_PIX_FMT_UYVY &&
	    fse->code != V4L2_PIX_FMT_ABGR32)
		return -EINVAL;

	if (fse->index >= num_frmfmt)
		return -EINVAL;
	fse->index = array_index_nospec(fse->index, num_frmfmt);

	fse->min_width = fse->max_width = frmfmt[fse->index].size.width;
	fse->min_height = fse->max_height = frmfmt[fse->index].size.height;

	return 0;
}

static int tc358840_enum_frameintervals(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	const struct camera_common_frmfmt *frmfmt = tc358840_frmfmt;
	int num_frmfmt = ARRAY_SIZE(tc358840_frmfmt);
	int i;

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	if (fie->code != V4L2_PIX_FMT_UYVY &&
	    fie->code != V4L2_PIX_FMT_ABGR32)
		return -EINVAL;

	for (i = 0; i < num_frmfmt; i++) {
		if (frmfmt[i].size.width == fie->width &&
		    frmfmt[i].size.height == fie->height)
			break;
	}
	if (i >= num_frmfmt)
		return -EINVAL;

	if (fie->index >= frmfmt[i].num_framerates)
		return -EINVAL;

	fie->index = array_index_nospec(fie->index, frmfmt[i].num_framerates);

	fie->interval.numerator = 1;
	fie->interval.denominator = frmfmt[i].framerates[fie->index];

	return 0;
}

static int tc358840_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static struct v4l2_subdev_video_ops tc358840_subdev_video_ops = {
	.g_input_status = tc358840_g_input_status,
	.s_dv_timings = tc358840_s_dv_timings,
	.g_dv_timings = tc358840_g_dv_timings,
	.query_dv_timings = tc358840_query_dv_timings,
	.g_mbus_config = tc358840_g_mbus_config,
	.s_stream = tc358840_s_stream,
};

static struct v4l2_subdev_core_ops tc358840_subdev_core_ops = {
	.s_power = tc358840_s_power,
	.log_status = tc358840_log_status,
	.interrupt_service_routine = tc358840_isr,
	.subscribe_event = tc358840_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tc358840_g_register,
	.s_register = tc358840_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops tc358840_pad_ops = {
	.set_fmt = tc358840_set_fmt,
	.get_fmt = tc358840_get_fmt,
	.enum_mbus_code = tc358840_enum_mbus_code,
	.get_edid = tc358840_g_edid,
	.set_edid = tc358840_s_edid,
	.dv_timings_cap = tc358840_dv_timings_cap,
	.enum_dv_timings = tc358840_enum_dv_timings,
	.enum_frame_size = tc358840_enum_framesizes,
	.enum_frame_interval = tc358840_enum_frameintervals,
};

static struct v4l2_subdev_ops tc358840_ops = {
	.core = &tc358840_subdev_core_ops,
	.video = &tc358840_subdev_video_ops,
	.pad = &tc358840_pad_ops,
};


/* --------------- CUSTOM CTRLS --------------- */

static const struct v4l2_ctrl_config tc358840_ctrl_audio_sampling_rate = {
	.id = TC358840_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config tc358840_ctrl_audio_present = {
	.id = TC358840_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config tc358840_ctrl_splitter_width = {
	.id = TC358840_CID_SPLITTER_WIDTH,
	.name = "Splitter Width",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 320,
	.max = 1920,
	.step = 16,
	.def = 1920,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

/* --------------- PROBE / REMOVE --------------- */

#ifdef CONFIG_OF

static bool tc358840_parse_dt(struct tc358840_platform_data *pdata,
		struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	const u32 *property;

	v4l_dbg(1, debug, client, "Device Tree Parameters:\n");

	pdata->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (pdata->reset_gpio == 0)
		return false;
	v4l_dbg(1, debug, client, "reset_gpio = %d\n", pdata->reset_gpio);

//	if (v4l2_of_parse_endpoint(node, &pdata->endpoint))
//		return false;

	property = of_get_property(node, "refclk_hz", NULL);
	if (property == NULL)
		return false;
	pdata->refclk_hz = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "refclk_hz = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "ddc5v_delay", NULL);
	if (property == NULL)
		return false;
	pdata->ddc5v_delay = be32_to_cpup(property);
	if (pdata->ddc5v_delay > DDC5V_DELAY_MAX)
		pdata->ddc5v_delay = DDC5V_DELAY_MAX;
	v4l_dbg(1, debug, client, "ddc5v_delay = %d ms\n",
		50 * pdata->ddc5v_delay);

	property = of_get_property(node, "enable_hdcp", NULL);
	if (property == NULL)
		return false;
	pdata->enable_hdcp = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "enable_hdcp = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "csi_port", NULL);
	if (property == NULL)
		return false;
	pdata->csi_port = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "csi_port = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "lineinitcnt", NULL);
	if (property == NULL)
		return false;
	pdata->lineinitcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "lineinitcnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "lptxtimecnt", NULL);
	if (property == NULL)
		return false;
	pdata->lptxtimecnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "lptxtimecnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "tclk_headercnt", NULL);
	if (property == NULL)
		return false;
	pdata->tclk_headercnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "tclk_headercnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "tclk_trailcnt", NULL);
	if (property == NULL)
		return false;
	pdata->tclk_trailcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "tclk_trailcnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "ths_headercnt", NULL);
	if (property == NULL)
		return false;
	pdata->ths_headercnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "ths_headercnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "twakeup", NULL);
	if (property == NULL)
		return false;
	pdata->twakeup = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "twakeup = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "tclk_postcnt", NULL);
	if (property == NULL)
		return false;
	pdata->tclk_postcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "tclk_postcnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "ths_trailcnt", NULL);
	if (property == NULL)
		return false;
	pdata->ths_trailcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "ths_trailcnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "hstxvregcnt", NULL);
	if (property == NULL)
		return false;
	pdata->hstxvregcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "hstxvregcnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "pll_prd", NULL);
	if (property == NULL)
		return false;
	pdata->pll_prd = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "pll_prd = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "pll_fbd", NULL);
	if (property == NULL)
		return false;
	pdata->pll_fbd = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "pll_fbd = %d\n", be32_to_cpup(property));

	return true;
}
#endif

static int tc358840_pwr_init(struct tc358840_platform_data *pdata,
		struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	int cam2_rst;
	int err;
	struct regulator *dvdd;
	struct regulator *iovdd;

	err = camera_common_regulator_get(&client->dev, &iovdd, "vif");
	if (err < 0) {
		dev_err(&client->dev, "cannot get regulator vif %d\n", err);
		return -EINVAL;
	}

	err = camera_common_regulator_get(&client->dev, &dvdd, "vdig");
	if (err < 0) {
		dev_err(&client->dev, "cannot get regulator vdig %d\n", err);
		return -EINVAL;
	}

	/*  cam2 rst */
	cam2_rst = of_get_named_gpio(node, "cam2_rst", 0);
	if (cam2_rst == 0)
		return false;
	err = gpio_request(cam2_rst, "cam2-rst");
	if (err < 0)
		dev_err(&client->dev,
				"cam2 rst gpio request failed %d\n", err);

	gpio_direction_output(cam2_rst, 1);

	if (dvdd) {
		err = regulator_enable(dvdd);
		if (err < 0) {
			dev_err(&client->dev, "cannot enable regulator vif %d\n", err);
			return -EINVAL;
		}
	}
	if (iovdd) {
		err = regulator_enable(iovdd);
		if (err < 0) {
			dev_err(&client->dev, "cannot enable regulator vdig %d\n", err);
			return -EINVAL;
		}
	}
	gpio_direction_output(cam2_rst, 1);

	return true;
}

static int tc358840_verify_chipid(struct v4l2_subdev *sd)
{
	u16 cid = 0;

	cid = i2c_rd16(sd, CHIPID_ADDR);
	if (cid != TC358840_CHIPID) {
		v4l2_err(sd, "Invalid chip ID 0x%04X\n", cid);
		return -ENODEV;
	}

	v4l2_dbg(1, debug, sd, "TC358840 ChipID 0x%02x, Revision 0x%02x\n",
		(cid & MASK_CHIPID) >> 8, cid & MASK_REVID);

	return 0;
}

static int tc358840_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops tc358840_subdev_internal_ops = {
	.open = tc358840_open,
};

static const struct media_entity_operations tc358840_media_ops = {
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

static int tc358840_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tc358840_state *state;
	struct v4l2_subdev *sd;
	int err;

	state = devm_kzalloc(&client->dev, sizeof(struct tc358840_state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	if (client->dev.of_node) {
		if (!tc358840_parse_dt(&state->pdata, client)) {
			v4l_err(client, "Couldn't parse device tree\n");
			return -ENODEV;
		}
		if (!tc358840_pwr_init(&state->pdata, client)) {
			v4l_err(client, "Couldn't power init\n");
			return -ENODEV;
		}
	} else {
		if (!client->dev.platform_data) {
			v4l_err(client, "No platform data!\n");
			return -ENODEV;
		}
		state->pdata = *(struct tc358840_platform_data *)client->dev.platform_data;
	}

	state->i2c_client = client;
	sd = &state->sd;

	i2c_set_clientdata(client, state);

	v4l2_i2c_subdev_init(sd, client, &tc358840_ops);

	/* Release System Reset (pin K8) */
	v4l2_info(sd, "Releasing System Reset (gpio 0x%04X)\n",
		state->pdata.reset_gpio);
	if (!gpio_is_valid(state->pdata.reset_gpio)) {
		v4l_err(client, "Reset GPIO is invalid!\n");
		return state->pdata.reset_gpio;
	}
	err = devm_gpio_request_one(&client->dev, state->pdata.reset_gpio,
					GPIOF_OUT_INIT_HIGH, "tc358840-reset");
	if (err) {
		dev_err(&client->dev,
			"Failed to request Reset GPIO 0x%04X: %d\n",
			state->pdata.reset_gpio, err);
		return err;
	}

	/*  */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_info(client, "Chip found @ 7h%02X (%s)\n",
		client->addr, client->adapter->name);


	/* Verify chip ID */
	err = tc358840_verify_chipid(sd);
	if (err)
		return err;

	/* Control Handlers */
	v4l2_ctrl_handler_init(&state->hdl, 4);

	/* Custom controls */
	state->detect_tx_5v_ctrl = v4l2_ctrl_new_std(&state->hdl, NULL,
			V4L2_CID_DV_RX_POWER_PRESENT, 0, 1, 0, 0);

	state->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358840_ctrl_audio_sampling_rate, NULL);

	state->audio_present_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358840_ctrl_audio_present, NULL);

	state->splitter_width_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358840_ctrl_splitter_width, NULL);

	if (state->hdl.error) {
		err = state->hdl.error;
		goto err_hdl;
	}

	sd->ctrl_handler = &state->hdl;

	if (tc358840_update_controls(sd)) {
		err = -ENODEV;
		goto err_hdl;
	}

	/* Work Queues */
	state->work_queues = create_singlethread_workqueue(client->name);
	if (!state->work_queues) {
		v4l2_err(sd, "Could not create work queue!\n");
		err = -ENOMEM;
		goto err_hdl;
	}
	INIT_DELAYED_WORK(&state->delayed_work_enable_hotplug,
			tc358840_delayed_work_enable_hotplug);
	INIT_DELAYED_WORK(&state->delayed_work_enable_interrupt,
			tc358840_delayed_work_enable_interrupt);
	INIT_WORK(&state->process_isr, tc358840_process_isr);
	mutex_init(&state->isr_lock);

	/* Initial Setup */
	state->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_1X16;
	tc358840_initial_setup(sd);

	tc358840_set_csi_mbus_config(sd);

	/* Get interrupt */
	if (client->irq) {
		err = devm_request_threaded_irq(&state->i2c_client->dev,
						client->irq, NULL, tc358840_irq_handler,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						sd->name, (void *)sd);
		if (err) {
			v4l2_err(sd, "Could not request interrupt %d!\n",
				 client->irq);
			goto err_hdl;
		}
	}

	queue_delayed_work(state->work_queues,
		&state->delayed_work_enable_interrupt,
		msecs_to_jiffies(DELAY_ENABLE_INTERRUPT_MS));

	/*
	 * FIXME: Don't know what MASK_CSITX0_INT and MASK_CSITX1_INT do.
	 * Thus, disable them for now...
	 */
#if 0
	i2c_wr16(sd, INTMASK, ~(MASK_HDMI_INT | MASK_CSITX0_INT |
		MASK_CSITX1_INT) & 0xFFFF);
#endif
	i2c_wr16(sd, INTMASK, ~(MASK_HDMI_INT) & 0xFFFF);

	v4l2_ctrl_handler_setup(sd->ctrl_handler);

	v4l2_info(sd, "%s found @ 7h%02X (%s)\n", client->name,
		  client->addr, client->adapter->name);

	sd->dev	= &client->dev;
	sd->internal_ops = &tc358840_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
#if defined(CONFIG_MEDIA_CONTROLLER)
	state->pad[0].flags = MEDIA_PAD_FL_SOURCE;
	state->pad[1].flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.ops = &tc358840_media_ops;
	err = tegra_media_entity_init(&sd->entity, 2,
				state->pad, true, true);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(sd);
	if (err == 0)
		return 0;

err_hdl:
	v4l2_ctrl_handler_free(&state->hdl);
	return err;
}

static int tc358840_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l_dbg(1, debug, client, "%s()\n", __func__);

#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	return 0;
}

static const struct i2c_device_id tc358840_id[] = {
	{ "tc358840", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tc358840_id);

#ifdef CONFIG_OF
static const struct of_device_id tc358840_of_table[] = {
	{ .compatible = "toshiba,tc358840" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358840_of_table);
#endif

static struct i2c_driver tc358840_driver = {
	.driver = {
		.of_match_table = of_match_ptr(tc358840_of_table),
		.name = "tc358840",
		.owner = THIS_MODULE,
	},
	.probe = tc358840_probe,
	.remove = tc358840_remove,
	.id_table = tc358840_id,
};
module_i2c_driver(tc358840_driver);

MODULE_DESCRIPTION("Driver for Toshiba TC358840 HDMI to CSI-2 Bridge");
MODULE_AUTHOR("Armin Weiss (weii@zhaw.ch)");
MODULE_LICENSE("GPL v2");
