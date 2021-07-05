/*
 * drivers/media/platform/tegra/camera/mc_common.h
 *
 * Tegra Media controller common APIs
 *
 * Copyright (c) 2012-2019, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __CAMERA_MC_COMMON_H__
#define __CAMERA_MC_COMMON_H__

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/sensor_common.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-core.h>
#include <media/tegra_camera_core.h>
#include <media/csi.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/rwsem.h>

#define MAX_FORMAT_NUM	64
#define	MAX_SUBDEVICES	4
#define	QUEUED_BUFFERS	4
#define	ENABLE		1
#define	DISABLE		0
#define MAX_SYNCPT_PER_CHANNEL	3

#define CAPTURE_MIN_BUFFERS	1U
#define CAPTURE_MAX_BUFFERS	240U

#define TEGRA_MEM_FORMAT 0
#define TEGRA_ISP_FORMAT 1

enum channel_capture_state {
	CAPTURE_IDLE = 0,
	CAPTURE_GOOD,
	CAPTURE_TIMEOUT,
	CAPTURE_ERROR,
};

enum tegra_vi_pg_mode {
	TEGRA_VI_PG_DISABLED = 0,
	TEGRA_VI_PG_DIRECT,
	TEGRA_VI_PG_PATCH,
};

enum interlaced_type {
	Top_Bottom = 0,
	Interleaved,
};

/**
 * struct tegra_channel_buffer - video channel buffer
 * @buf: vb2 buffer base object
 * @queue: buffer list entry in the channel queued buffers list
 * @chan: channel that uses the buffer
 * @vb2_state: V4L2 buffer state (active, done, error)
 * @capture_descr_index: Index into the VI capture descriptor queue
 * @addr: Tegra IOVA buffer address for VI output
 */
struct tegra_channel_buffer {
	struct vb2_v4l2_buffer buf;
	struct list_head queue;
	struct tegra_channel *chan;

	unsigned int vb2_state;
	unsigned int capture_descr_index;

	dma_addr_t addr;

	u32 thresh[TEGRA_CSI_BLOCKS];
	int version;
	int state;
};

#define to_tegra_channel_buffer(vb) \
	container_of(vb, struct tegra_channel_buffer, buf)

/**
 * struct tegra_vi_graph_entity - Entity in the video graph
 * @list: list entry in a graph entities list
 * @node: the entity's DT node
 * @entity: media entity, from the corresponding V4L2 subdev
 * @asd: subdev asynchronous registration information
 * @subdev: V4L2 subdev
 */
struct tegra_vi_graph_entity {
	struct list_head list;
	struct device_node *node;
	struct media_entity *entity;

	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;
};

/**
 * struct tegra_channel - Tegra video channel
 * @list: list entry in a composite device dmas list
 * @video: V4L2 video device associated with the video channel
 * @video_lock:
 * @pad: media pad for the video device entity
 * @pipe: pipeline belonging to the channel
 *
 * @vi: composite device DT node port number for the channel
 *
 * @kthread_capture: kernel thread task structure of this video channel
 * @wait: wait queue structure for kernel thread
 *
 * @format: active V4L2 pixel format
 * @fmtinfo: format information corresponding to the active @format
 *
 * @queue: vb2 buffers queue
 * @alloc_ctx: allocation context for the vb2 @queue
 * @sequence: V4L2 buffers sequence number
 *
 * @capture: list of queued buffers for capture
 * @queued_lock: protects the buf_queued list
 *
 * @csi: CSI register bases
 * @stride_align: channel buffer stride alignment, default is 1
 * @width_align: image width alignment, default is 1
 * @height_align: channel buffer height alignment, default is 1
 * @size_align: channel buffer size alignment, default is 1
 * @port: CSI port of this video channel
 * @io_id: Tegra IO rail ID of this video channel
 *
 * @fmts_bitmap: a bitmap for formats supported
 * @bypass: bypass flag for VI bypass mode
 * @restart_version: incremented every time either capture or release threads
 *                   wants to reset VI. it is appended to each buffer processed
 *                   by the capture thread, and inspected by each buffer
 *                   processed by the receive thread.
 * @capture_version: thread-local copy of @restart_version created when the
 *                   capture thread resets the VI.
 */
struct tegra_channel {
	int id;
	struct list_head list;
	struct video_device *video;
	struct media_pad pad;
	struct media_pipeline pipe;
	struct mutex video_lock;

	struct tegra_mc_vi *vi;
	struct v4l2_subdev *subdev[MAX_SUBDEVICES];
	struct v4l2_subdev *subdev_on_csi;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_pix_format format;
	const struct tegra_video_format *fmtinfo;
	const struct tegra_video_format *video_formats[MAX_FORMAT_NUM];
	unsigned int num_video_formats;
	struct mutex stop_kthread_lock;

	unsigned char port[TEGRA_CSI_BLOCKS];
	unsigned int virtual_channel;
	unsigned int syncpt[TEGRA_CSI_BLOCKS][MAX_SYNCPT_PER_CHANNEL];
	unsigned int syncpoint_fifo[TEGRA_CSI_BLOCKS][MAX_SYNCPT_PER_CHANNEL];
	unsigned int buffer_offset[TEGRA_CSI_BLOCKS];
	unsigned int *buffer_state;
	struct vb2_v4l2_buffer **buffers;
	unsigned long timeout;
	atomic_t restart_version;
	int capture_version;
	unsigned int save_index;
	unsigned int free_index;
	unsigned int num_buffers;
	spinlock_t buffer_lock;
	unsigned int released_bufs;

	unsigned int capture_queue_depth;
	unsigned int capture_descr_index;
	unsigned int capture_descr_sequence;
	unsigned int capture_reqs_enqueued;
	struct task_struct *kthread_capture_start;
	struct task_struct *kthread_release;
	wait_queue_head_t start_wait;
	wait_queue_head_t release_wait;
	struct task_struct *kthread_capture_dequeue;
	wait_queue_head_t dequeue_wait;
	struct vb2_queue queue;
	void *alloc_ctx;
	bool init_done;
	struct list_head capture;
	struct list_head release;
	struct list_head dequeue;
	spinlock_t start_lock;
	spinlock_t release_lock;
	spinlock_t dequeue_lock;
	struct work_struct status_work;
	struct work_struct error_work;

	void __iomem *csibase[TEGRA_CSI_BLOCKS];
	unsigned int stride_align;
	unsigned int preferred_stride;
	unsigned int width_align;
	unsigned int height_align;
	unsigned int size_align;
	unsigned int valid_ports;
	unsigned int total_ports;
	unsigned int numlanes;
	unsigned int io_id;
	unsigned int num_subdevs;
	unsigned int sequence;
	unsigned int saved_ctx_bypass;
	unsigned int saved_ctx_pgmode;
	unsigned int gang_mode;
	unsigned int gang_width;
	unsigned int gang_height;
	unsigned int gang_bytesperline;
	unsigned int gang_sizeimage;
	unsigned int embedded_data_width;
	unsigned int embedded_data_height;

	DECLARE_BITMAP(fmts_bitmap, MAX_FORMAT_NUM);
	atomic_t power_on_refcnt;
	struct v4l2_fh *fh;
	bool bypass;
	bool write_ispformat;
	bool low_latency;
	enum tegra_vi_pg_mode pg_mode;
	bool bfirst_fstart;
	enum channel_capture_state capture_state;
	bool queue_error;
	spinlock_t capture_state_lock;
	atomic_t is_streaming;
	int requested_kbyteps;
	unsigned long requested_hz;

	struct vi_notify_channel *vnc[TEGRA_CSI_BLOCKS];
	int vnc_id[TEGRA_CSI_BLOCKS];
	int grp_id;

	struct vi_capture *capture_data;
	struct v4l2_async_notifier notifier;
	struct list_head entities;
	struct device_node *endpoint_node; /* endpoint of_node in vi */
	unsigned int subdevs_bound;
	unsigned int link_status;
	struct nvcsi_deskew_context *deskew_ctx;
	struct tegra_vi_channel *tegra_vi_channel;
	struct capture_descriptor *request;
	bool is_slvsec;
	int is_interlaced;
	enum interlaced_type interlace_type;
	int interlace_bplfactor;

	atomic_t syncpt_depth;
	struct rw_semaphore reset_lock;
};

#define to_tegra_channel(vdev) \
	container_of(vdev, struct tegra_channel, video)

/**
 * struct tegra_mc_vi - NVIDIA Tegra Media controller structure
 * @v4l2_dev: V4L2 device
 * @media_dev: media device
 * @dev: device struct
 * @tegra_camera: tegra camera structure
 * @nvhost_device_data: NvHost VI device information
 *
 * @notifier: V4L2 asynchronous subdevs notifier
 * @entities: entities in the graph as a list of tegra_vi_graph_entity
 * @num_subdevs: number of subdevs in the pipeline
 *
 * @channels: list of channels at the pipeline output and input
 *
 * @ctrl_handler: V4L2 control handler
 * @pattern: test pattern generator V4L2 control
 * @pg_mode: test pattern generator mode (disabled/direct/patch)
 *
 * @has_sensors: a flag to indicate whether is a real sensor connecting
 */
struct tegra_mc_vi {
	struct vi *vi;
	struct platform_device *ndev;
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct device *dev;
	struct nvhost_device_data *ndata;

	struct regulator *reg;
	struct clk *clk;
	struct clk *parent_clk;

	unsigned int num_channels;
	unsigned int num_subdevs;

	struct tegra_csi_device *csi;
	struct list_head vi_chans;
	struct tegra_channel *tpg_start;
	void __iomem *iomem;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pattern;
	enum tegra_vi_pg_mode pg_mode;

	bool has_sensors;
	atomic_t power_on_refcnt;
	atomic_t vb2_dma_alloc_refcnt;
	struct mutex bw_update_lock;
	unsigned long aggregated_kbyteps;
	unsigned long max_requested_hz;
	struct mutex mipical_lock;

	bool bypass;

	const struct tegra_vi_fops *fops;

	dma_addr_t emb_buf;
	void *emb_buf_addr;
	unsigned int emb_buf_size;
};

int tegra_vi_get_port_info(struct tegra_channel *chan,
			struct device_node *node, unsigned int index);
void tegra_vi_v4l2_cleanup(struct tegra_mc_vi *vi);
int tegra_vi_v4l2_init(struct tegra_mc_vi *vi);
int tegra_vi_tpg_graph_init(struct tegra_mc_vi *vi);
int tegra_vi_graph_init(struct tegra_mc_vi *vi);
void tegra_vi_graph_cleanup(struct tegra_mc_vi *vi);
int tegra_channel_init(struct tegra_channel *chan);
void tegra_vi_channels_unregister(struct tegra_mc_vi *vi);
int tegra_vi_channels_init(struct tegra_mc_vi *vi);
int tegra_channel_cleanup(struct tegra_channel *chan);
int tegra_vi_channels_cleanup(struct tegra_mc_vi *vi);
int tegra_channel_init_subdevices(struct tegra_channel *chan);
void tegra_channel_remove_subdevices(struct tegra_channel *chan);
struct v4l2_subdev *tegra_channel_find_linked_csi_subdev(
	struct tegra_channel *chan);
int tegra_vi2_power_on(struct tegra_mc_vi *vi);
void tegra_vi2_power_off(struct tegra_mc_vi *vi);
int tegra_vi4_power_on(struct tegra_mc_vi *vi);
void tegra_vi4_power_off(struct tegra_mc_vi *vi);
int tegra_vi5_power_on(struct tegra_mc_vi *vi);
void tegra_vi5_power_off(struct tegra_mc_vi *vi);
int tegra_clean_unlinked_channels(struct tegra_mc_vi *vi);
int tegra_channel_s_ctrl(struct v4l2_ctrl *ctrl);
int tegra_vi_media_controller_init(struct tegra_mc_vi *mc_vi,
			struct platform_device *pdev);
void tegra_vi_media_controller_cleanup(struct tegra_mc_vi *mc_vi);
void tegra_channel_ec_close(struct tegra_mc_vi *mc_vi);
void tegra_channel_query_hdmiin_unplug(struct tegra_channel *chan,
		struct v4l2_event *event);
int tegra_vi_mfi_work(struct tegra_mc_vi *vi, int csiport);
int tpg_vi_media_controller_init(struct tegra_mc_vi *mc_vi, int pg_mode);
void tpg_vi_media_controller_cleanup(struct tegra_mc_vi *mc_vi);
struct tegra_mc_vi *tegra_get_mc_vi(void);

u32 tegra_core_get_fourcc_by_idx(struct tegra_channel *chan,
		unsigned int index);
int tegra_core_get_idx_by_code(struct tegra_channel *chan,
		unsigned int code, unsigned offset);
int tegra_core_get_code_by_fourcc(struct tegra_channel *chan,
		unsigned int fourcc, unsigned int offset);
const struct tegra_video_format *tegra_core_get_format_by_code(
		struct tegra_channel *chan,
		unsigned int code, unsigned offset);
const struct tegra_video_format *tegra_core_get_format_by_fourcc(
		struct tegra_channel *chan, u32 fourcc);
void tegra_channel_queued_buf_done(struct tegra_channel *chan,
	enum vb2_buffer_state state, bool multi_queue);
int tegra_channel_set_stream(struct tegra_channel *chan, bool on);
int tegra_channel_write_blobs(struct tegra_channel *chan);
void tegra_channel_ring_buffer(struct tegra_channel *chan,
			       struct vb2_v4l2_buffer *vb,
			       struct timespec *ts, int state);
struct tegra_channel_buffer *dequeue_buffer(struct tegra_channel *chan,
	bool requeue);
struct tegra_channel_buffer *dequeue_dequeue_buffer(struct tegra_channel *chan);
int tegra_channel_error_recover(struct tegra_channel *chan, bool queue_error);
int tegra_channel_alloc_buffer_queue(struct tegra_channel *chan,
					unsigned int num_buffers);
void tegra_channel_dealloc_buffer_queue(struct tegra_channel *chan);
void tegra_channel_init_ring_buffer(struct tegra_channel *chan);
void free_ring_buffers(struct tegra_channel *chan, int frames);
void release_buffer(struct tegra_channel *chan,
			struct tegra_channel_buffer *buf);
void set_timestamp(struct tegra_channel_buffer *buf,
			const struct timespec *ts);
void enqueue_inflight(struct tegra_channel *chan,
			struct tegra_channel_buffer *buf);
struct tegra_channel_buffer *dequeue_inflight(struct tegra_channel *chan);
int tegra_channel_set_power(struct tegra_channel *chan, bool on);

int tegra_channel_init_video(struct tegra_channel *chan);
int tegra_channel_cleanup_video(struct tegra_channel *chan);

struct tegra_vi_fops {
	int (*vi_power_on)(struct tegra_channel *chan);
	void (*vi_power_off)(struct tegra_channel *chan);
	int (*vi_start_streaming)(struct vb2_queue *vq, u32 count);
	int (*vi_stop_streaming)(struct vb2_queue *vq);
	int (*vi_setup_queue)(struct tegra_channel *chan,
			unsigned int *nbuffers);
	int (*vi_error_recover)(struct tegra_channel *chan, bool queue_error);
	int (*vi_add_ctrls)(struct tegra_channel *chan);
	void (*vi_init_video_formats)(struct tegra_channel *chan);
	long (*vi_default_ioctl)(struct file *file, void *fh,
			bool use_prio, unsigned int cmd, void *arg);
	int (*vi_mfi_work)(struct tegra_mc_vi *vi, int port);
	void (*vi_stride_align)(unsigned int *bpl);
};

struct tegra_csi_fops {
	int (*csi_power_on)(struct tegra_csi_device *csi);
	int (*csi_power_off)(struct tegra_csi_device *csi);
	int (*csi_start_streaming)(struct tegra_csi_channel *chan,
		int port_idx);
	void (*csi_stop_streaming)(struct tegra_csi_channel *chan,
		int port_idx);
	void (*csi_override_format)(struct tegra_csi_channel *chan,
		int port_idx);
	int (*csi_error_recover)(struct tegra_csi_channel *chan, int port_idx);
	int (*mipical)(struct tegra_csi_channel *chan);
	int (*hw_init)(struct tegra_csi_device *csi);
};

struct tegra_t210_vi_data {
	struct nvhost_device_data *info;
	const struct tegra_vi_fops *vi_fops;
	const struct tegra_csi_fops *csi_fops;
};

struct tegra_vi_data {
	struct nvhost_device_data *info;
	const struct tegra_vi_fops *vi_fops;
};

struct tegra_csi_data {
	struct nvhost_device_data *info;
	const struct tegra_csi_fops *csi_fops;
};
#endif
