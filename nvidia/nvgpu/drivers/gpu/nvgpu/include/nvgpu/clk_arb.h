/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_CLK_ARB_H
#define NVGPU_CLK_ARB_H

struct gk20a;

#include <nvgpu/types.h>
#include <nvgpu/bitops.h>
#include <nvgpu/lock.h>
#include <nvgpu/kmem.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/kref.h>
#include <nvgpu/log.h>
#include <nvgpu/barrier.h>
#include <nvgpu/cond.h>

#include "clk/clk.h"
#include "pstate/pstate.h"
#include "lpwr/lpwr.h"
#include "volt/volt.h"

#define MAX_F_POINTS 256
#define DEFAULT_EVENT_NUMBER 32

struct nvgpu_clk_dev;
struct nvgpu_clk_arb_target;
struct nvgpu_clk_notification_queue;
struct nvgpu_clk_session;

#define VF_POINT_INVALID_PSTATE ~0U
#define VF_POINT_SET_PSTATE_SUPPORTED(a, b) ((a)->pstates |= (1UL << (b)))
#define VF_POINT_GET_PSTATE(a)	(((a)->pstates) ?\
	__fls((a)->pstates) :\
	VF_POINT_INVALID_PSTATE)
#define VF_POINT_COMMON_PSTATE(a, b)	(((a)->pstates & (b)->pstates) ?\
	__fls((a)->pstates & (b)->pstates) :\
	VF_POINT_INVALID_PSTATE)

/*
 * These events, defined in common code are the counterparts of the uapi
 * events. There should be a conversion function to take care to convert
 * these to the uapi events.
 */
/* Event associated to a VF update */
#define NVGPU_EVENT_VF_UPDATE				0

/* Recoverable alarms (POLLPRI) */
/* Alarm when target frequency on any session is not possible */
#define NVGPU_EVENT_ALARM_TARGET_VF_NOT_POSSIBLE		1
/* Alarm when target frequency on current session is not possible */
#define NVGPU_EVENT_ALARM_LOCAL_TARGET_VF_NOT_POSSIBLE	2
/* Alarm when Clock Arbiter failed */
#define NVGPU_EVENT_ALARM_CLOCK_ARBITER_FAILED		3
/* Alarm when VF table update failed */
#define NVGPU_EVENT_ALARM_VF_TABLE_UPDATE_FAILED		4
/* Alarm on thermal condition */
#define NVGPU_EVENT_ALARM_THERMAL_ABOVE_THRESHOLD		5
/* Alarm on power condition */
#define NVGPU_EVENT_ALARM_POWER_ABOVE_THRESHOLD		6

/* Non recoverable alarm (POLLHUP) */
/* Alarm on GPU shutdown/fall from bus */
#define NVGPU_EVENT_ALARM_GPU_LOST				7

#define NVGPU_EVENT_LAST	NVGPU_EVENT_ALARM_GPU_LOST

/* Local Alarms */
#define EVENT(alarm)	(0x1UL << NVGPU_EVENT_##alarm)

#define LOCAL_ALARM_MASK (EVENT(ALARM_LOCAL_TARGET_VF_NOT_POSSIBLE) | \
				EVENT(VF_UPDATE))

#define _WRAPGTEQ(a, b) ((a-b) > 0)

/*
 * NVGPU_POLL* defines equivalent to the POLL* linux defines
 */
#define NVGPU_POLLIN (1 << 0)
#define NVGPU_POLLPRI (1 << 1)
#define NVGPU_POLLOUT (1 << 2)
#define NVGPU_POLLRDNORM (1 << 3)
#define NVGPU_POLLHUP (1 << 4)

/* NVGPU_CLK_DOMAIN_* defines equivalent to NVGPU_GPU_CLK_DOMAIN_*
 * defines in uapi header
 */
/* Memory clock */
#define NVGPU_CLK_DOMAIN_MCLK	(0)
/* Main graphics core clock */
#define NVGPU_CLK_DOMAIN_GPCCLK	(1)

#define NVGPU_CLK_DOMAIN_MAX	(NVGPU_CLK_DOMAIN_GPCCLK)

#define clk_arb_dbg(g, fmt, args...)	\
	do {								\
		nvgpu_log(g, gpu_dbg_clk_arb,	\
				fmt, ##args);	\
	} while (0)

struct nvgpu_clk_notification {
	u32 notification;
	u64 timestamp;
};

struct nvgpu_clk_notification_queue {
	u32 size;
	nvgpu_atomic_t head;
	nvgpu_atomic_t tail;
	struct nvgpu_clk_notification *notifications;
};

struct nvgpu_clk_vf_point {
	u16 pstates;
	union {
		struct {
			u16 gpc_mhz;
			u16 sys_mhz;
			u16 xbar_mhz;
		};
		u16 mem_mhz;
	};
	u32 uvolt;
	u32 uvolt_sram;
};

struct nvgpu_clk_vf_table {
	u32 mclk_num_points;
	struct nvgpu_clk_vf_point *mclk_points;
	u32 gpc2clk_num_points;
	struct nvgpu_clk_vf_point *gpc2clk_points;
};
#ifdef CONFIG_DEBUG_FS
struct nvgpu_clk_arb_debug {
	s64 switch_max;
	s64 switch_min;
	u64 switch_num;
	s64 switch_avg;
	s64 switch_std;
};
#endif

struct nvgpu_clk_arb_target {
	u16 mclk;
	u16 gpc2clk;
	u32 pstate;
};

enum clk_arb_work_item_type {
	CLK_ARB_WORK_UPDATE_VF_TABLE,
	CLK_ARB_WORK_UPDATE_ARB
};

struct nvgpu_clk_arb_work_item {
	enum clk_arb_work_item_type item_type;
	struct nvgpu_clk_arb *arb;
	struct nvgpu_list_node worker_item;
};

struct nvgpu_clk_arb {
	struct nvgpu_spinlock sessions_lock;
	struct nvgpu_spinlock users_lock;
	struct nvgpu_spinlock requests_lock;

	struct nvgpu_mutex pstate_lock;
	struct nvgpu_list_node users;
	struct nvgpu_list_node sessions;
	struct nvgpu_list_node requests;

	struct gk20a *g;
	int status;

	struct nvgpu_clk_arb_target actual_pool[2];
	struct nvgpu_clk_arb_target *actual;

	u16 gpc2clk_default_mhz;
	u16 mclk_default_mhz;
	u32 voltuv_actual;

	u16 gpc2clk_min, gpc2clk_max;
	u16 mclk_min, mclk_max;

	struct nvgpu_clk_arb_work_item update_vf_table_work_item;
	struct nvgpu_clk_arb_work_item update_arb_work_item;

	struct nvgpu_cond request_wq;

	struct nvgpu_clk_vf_table *current_vf_table;
	struct nvgpu_clk_vf_table vf_table_pool[2];
	u32 vf_table_index;

	u16 *mclk_f_points;
	nvgpu_atomic_t req_nr;

	u32 mclk_f_numpoints;
	u16 *gpc2clk_f_points;
	u32 gpc2clk_f_numpoints;

	bool clk_arb_events_supported;

	nvgpu_atomic64_t alarm_mask;
	struct nvgpu_clk_notification_queue notification_queue;

#ifdef CONFIG_DEBUG_FS
	struct nvgpu_clk_arb_debug debug_pool[2];
	struct nvgpu_clk_arb_debug *debug;
	bool debugfs_set;
#endif
};

struct nvgpu_clk_dev {
	struct nvgpu_clk_session *session;
	union {
		struct nvgpu_list_node link;
		struct nvgpu_list_node node;
	};
	struct nvgpu_cond readout_wq;
	nvgpu_atomic_t poll_mask;
	u16 gpc2clk_target_mhz;
	u16 mclk_target_mhz;
	u32 alarms_reported;
	nvgpu_atomic_t enabled_mask;
	struct nvgpu_clk_notification_queue queue;
	u32 arb_queue_head;
	struct nvgpu_ref refcount;
};

struct nvgpu_clk_session {
	bool zombie;
	struct gk20a *g;
	struct nvgpu_ref refcount;
	struct nvgpu_list_node link;
	struct nvgpu_list_node targets;

	struct nvgpu_spinlock session_lock;
	struct nvgpu_clk_arb_target target_pool[2];
	struct nvgpu_clk_arb_target *target;
};

static inline struct nvgpu_clk_session *
nvgpu_clk_session_from_link(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_session *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_session, link));
};

static inline struct nvgpu_clk_dev *
nvgpu_clk_dev_from_node(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_dev *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_dev, node));
};

static inline struct nvgpu_clk_dev *
nvgpu_clk_dev_from_link(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_dev *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_dev, link));
};

static inline struct nvgpu_clk_arb_work_item *
nvgpu_clk_arb_work_item_from_worker_item(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_arb_work_item *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_arb_work_item, worker_item));
};

void nvgpu_clk_arb_worker_enqueue(struct gk20a *g,
		struct nvgpu_clk_arb_work_item *work_item);

int nvgpu_clk_arb_update_vf_table(struct nvgpu_clk_arb *arb);

int nvgpu_clk_arb_worker_init(struct gk20a *g);

int nvgpu_clk_arb_init_arbiter(struct gk20a *g);

bool nvgpu_clk_arb_has_active_req(struct gk20a *g);

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz);

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
		u32 api_domain, u16 *actual_mhz);

int nvgpu_clk_arb_get_arbiter_effective_mhz(struct gk20a *g,
		u32 api_domain, u16 *effective_mhz);

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints);

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g);
bool nvgpu_clk_arb_is_valid_domain(struct gk20a *g, u32 api_domain);

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g);

int nvgpu_clk_arb_install_session_fd(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_init_session(struct gk20a *g,
		struct nvgpu_clk_session **_session);

void nvgpu_clk_arb_release_session(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_commit_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int request_fd);

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
		int fd, u32 api_domain, u16 target_mhz);

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 *target_mhz);

int nvgpu_clk_arb_install_event_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd, u32 alarm_mask);

int nvgpu_clk_arb_install_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd);

void nvgpu_clk_arb_schedule_vf_table_update(struct gk20a *g);

int nvgpu_clk_arb_get_current_pstate(struct gk20a *g);

void nvgpu_clk_arb_pstate_change_lock(struct gk20a *g, bool lock);

void nvgpu_clk_arb_send_thermal_alarm(struct gk20a *g);

void nvgpu_clk_arb_set_global_alarm(struct gk20a *g, u32 alarm);

void nvgpu_clk_arb_schedule_alarm(struct gk20a *g, u32 alarm);

void nvgpu_clk_arb_clear_global_alarm(struct gk20a *g, u32 alarm);

void nvgpu_clk_arb_free_session(struct nvgpu_ref *refcount);

void nvgpu_clk_arb_free_fd(struct nvgpu_ref *refcount);

u32 nvgpu_clk_arb_notify(struct nvgpu_clk_dev *dev,
				struct nvgpu_clk_arb_target *target,
				u32 alarm);

int nvgpu_clk_notification_queue_alloc(struct gk20a *g,
				struct nvgpu_clk_notification_queue *queue,
				size_t events_number);

void nvgpu_clk_notification_queue_free(struct gk20a *g,
		struct nvgpu_clk_notification_queue *queue);

void nvgpu_clk_arb_event_post_event(struct nvgpu_clk_dev *dev);

unsigned long nvgpu_clk_measure_freq(struct gk20a *g, u32 api_domain);

#ifdef CONFIG_DEBUG_FS
int nvgpu_clk_arb_debugfs_init(struct gk20a *g);
#endif
#endif /* NVGPU_CLK_ARB_H */

