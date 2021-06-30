/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
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

#include "soc/tegra/camrtc-trace.h"

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/printk.h>
#include <linux/seq_buf.h>
#include <linux/slab.h>
#include <linux/tegra-camera-rtcpu.h>
#include <linux/tegra-rtcpu-trace.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <asm/cacheflush.h>

#ifdef CONFIG_EVENTLIB
#include <linux/keventlib.h>
#include <uapi/linux/nvhost_events.h>
#include "rtcpu/device-group.h"
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/tegra_rtcpu.h>
#include <trace/events/freertos.h>

#define NV(p) "nvidia," #p

#define WORK_INTERVAL_DEFAULT		100
#define EXCEPTION_STR_LENGTH		2048

/*
 * Private driver data structure
 */

struct tegra_rtcpu_trace {
	struct device *dev;
	struct device_node *of_node;
	struct mutex lock;

	/* memory */
	void *trace_memory;
	u32 trace_memory_size;
	dma_addr_t dma_handle;

	/* pointers to each block */
	void *exceptions_base;
	struct camrtc_event_struct *events;
	dma_addr_t dma_handle_pointers;
	dma_addr_t dma_handle_exceptions;
	dma_addr_t dma_handle_events;

	/* limit */
	u32 exception_entries;
	u32 event_entries;

	/* exception pointer */
	u32 exception_last_idx;

	/* last pointer */
	u32 event_last_idx;

	/* worker */
	struct delayed_work work;
	unsigned long work_interval_jiffies;

	/* statistics */
	u32 n_exceptions;
	u64 n_events;

	/* copy of the latest exception and event */
	char last_exception_str[EXCEPTION_STR_LENGTH];
	struct camrtc_event_struct copy_last_event;

	/* debugfs */
	struct dentry *debugfs_root;

	/* eventlib */
	struct platform_device *vi_platform_device;
	struct platform_device *isp_platform_device;

	/* printk logging */
	const char *log_prefix;
	bool enable_printk;
	u32 printk_used;
	char printk[EXCEPTION_STR_LENGTH];
};

/*
 * Trace memory
 */

static int rtcpu_trace_setup_memory(struct tegra_rtcpu_trace *tracer)
{
	struct device *dev = tracer->dev;
	struct of_phandle_args reg_spec;
	int ret;
	void *trace_memory;
	size_t mem_size;
	dma_addr_t dma_addr;

	ret = of_parse_phandle_with_fixed_args(dev->of_node, NV(trace),
		3, 0, &reg_spec);
	if (unlikely(ret != 0)) {
		dev_err(dev, "Cannot find trace entry\n");
		return -EINVAL;
	}

	mem_size = reg_spec.args[2];
	trace_memory = dma_alloc_coherent(dev, mem_size, &dma_addr,
					GFP_KERNEL | __GFP_ZERO);
	if (trace_memory == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	/* Save the information */
	tracer->trace_memory = trace_memory;
	tracer->trace_memory_size = mem_size;
	tracer->dma_handle = dma_addr;
	tracer->of_node = reg_spec.np;

	return 0;

error:
	of_node_put(reg_spec.np);
	return ret;
}

static void rtcpu_trace_init_memory(struct tegra_rtcpu_trace *tracer)
{
	/* memory map */
	tracer->dma_handle_pointers = tracer->dma_handle +
	    offsetof(struct camrtc_trace_memory_header, exception_next_idx);
	tracer->exceptions_base = tracer->trace_memory +
	    CAMRTC_TRACE_EXCEPTION_OFFSET;
	tracer->exception_entries = 7;
	tracer->dma_handle_exceptions = tracer->dma_handle +
	    CAMRTC_TRACE_EXCEPTION_OFFSET;
	tracer->events = tracer->trace_memory + CAMRTC_TRACE_EVENT_OFFSET;
	tracer->event_entries =
	    (tracer->trace_memory_size - CAMRTC_TRACE_EVENT_OFFSET) /
	    CAMRTC_TRACE_EVENT_SIZE;
	tracer->dma_handle_events = tracer->dma_handle +
	    CAMRTC_TRACE_EXCEPTION_OFFSET;

	{
		struct camrtc_trace_memory_header header = {
			.signature[0] = CAMRTC_TRACE_SIGNATURE_1,
			.signature[1] = CAMRTC_TRACE_SIGNATURE_2,
			.revision = 1,
			.exception_offset = CAMRTC_TRACE_EXCEPTION_OFFSET,
			.exception_size = CAMRTC_TRACE_EXCEPTION_SIZE,
			.exception_entries = tracer->exception_entries,
			.event_offset = CAMRTC_TRACE_EVENT_OFFSET,
			.event_size = CAMRTC_TRACE_EVENT_SIZE,
			.event_entries = tracer->event_entries,
		};

		memcpy(tracer->trace_memory, &header, sizeof(header));

		dma_sync_single_for_device(tracer->dev,
			tracer->dma_handle, sizeof(header),
			DMA_TO_DEVICE);
	}
}

/*
 * Worker
 */

static void rtcpu_trace_invalidate_entries(struct tegra_rtcpu_trace *tracer,
	dma_addr_t dma_handle, u32 old_next, u32 new_next,
	u32 entry_size, u32 entry_count)
{
	/* invalidate cache */
	if (new_next > old_next) {
		dma_sync_single_for_cpu(tracer->dev,
			dma_handle + old_next * entry_size,
			(new_next - old_next) * entry_size,
			DMA_FROM_DEVICE);
	} else {
		dma_sync_single_for_cpu(tracer->dev,
			dma_handle + old_next * entry_size,
			(entry_count - old_next) * entry_size,
			DMA_FROM_DEVICE);
		dma_sync_single_for_cpu(tracer->dev,
			dma_handle, new_next * entry_size,
			DMA_FROM_DEVICE);
	}
}

static void rtcpu_trace_exception(struct tegra_rtcpu_trace *tracer,
	struct camrtc_trace_armv7_exception *exc)
{
	static const char * const s_str_exc_type[] = {
		"Invalid (Reset)",
		"Undefined instruction",
		"Invalid (SWI)",
		"Prefetch abort",
		"Data abort",
		"Invalid (Reserved)",
		"IRQ",
		"FIQ",
	};

	struct seq_buf sb;
	unsigned int i, count;
	char *buf = tracer->last_exception_str;
	size_t buf_size = sizeof(tracer->last_exception_str);
	char const header[] =
		"###################### RTCPU EXCEPTION ######################";
	char const trailer[] =
		"#############################################################";

	seq_buf_init(&sb, buf, buf_size);

	seq_buf_printf(&sb, "%s %s\n",
		tracer->log_prefix,
		(exc->type < ARRAY_SIZE(s_str_exc_type)) ?
			s_str_exc_type[exc->type] : "Unknown");

	seq_buf_printf(&sb,
	    "  R0:  %08x R1:  %08x R2:  %08x R3:  %08x\n",
	    exc->gpr.r0, exc->gpr.r1, exc->gpr.r2, exc->gpr.r3);
	seq_buf_printf(&sb,
	    "  R4:  %08x R5:  %08x R6:  %08x R7:  %08x\n",
	    exc->gpr.r4, exc->gpr.r5, exc->gpr.r6, exc->gpr.r7);
	seq_buf_printf(&sb,
	    "  R8:  %08x R9:  %08x R10: %08x R11: %08x\n",
	    exc->gpr.r8, exc->gpr.r9, exc->gpr.r10, exc->gpr.r11);
	seq_buf_printf(&sb,
	    "  R12: %08x SP:  %08x LR:  %08x PC:  %08x\n",
	    exc->gpr.r12, exc->gpr.sp, exc->gpr.lr, exc->gpr.pc);

	if (exc->type == CAMRTC_ARMV7_EXCEPTION_FIQ) {
		seq_buf_printf(&sb,
		    "  R8: %08x R9: %08x R10: %08x R11: %08x, R12: %08x\n",
		    exc->gpr.r8_prev, exc->gpr.r9_prev,
		    exc->gpr.r10_prev, exc->gpr.r11_prev,
		    exc->gpr.r12_prev);
	}
	seq_buf_printf(&sb, "  SP: %08x LR: %08x\n",
	    exc->gpr.sp_prev, exc->gpr.lr_prev);

	seq_buf_printf(&sb, "  CPSR: %08x SPSR: %08x\n",
	    exc->cpsr, exc->spsr);

	seq_buf_printf(&sb, "  DFSR: %08x DFAR: %08x ADFSR: %08x\n",
	    exc->dfsr, exc->dfar, exc->adfsr);
	seq_buf_printf(&sb, "  IFSR: %08x IFAR: %08x AIFSR: %08x\n",
	    exc->ifsr, exc->ifar, exc->aifsr);

	count = (exc->len -
		offsetof(struct camrtc_trace_armv7_exception, callstack)) /
		sizeof(struct camrtc_trace_callstack);

	if (count > 0)
		seq_buf_printf(&sb, "Callstack\n");

	for (i = 0; i < count; ++i) {
		if (i >= CAMRTC_TRACE_CALLSTACK_MAX)
			break;
		seq_buf_printf(&sb, "  [%08x]: %08x\n",
			exc->callstack[i].lr_stack_addr, exc->callstack[i].lr);
	}

	if (i < count)
		seq_buf_printf(&sb, "  ... [skipping %u entries]\n", count - i);

	printk(KERN_INFO "%s\n%s\n%s\n%s%s%s\n%s\n",
		" ", " ", header, buf, trailer, " ", " ");
}

static inline void rtcpu_trace_exceptions(struct tegra_rtcpu_trace *tracer)
{
	const struct camrtc_trace_memory_header *header = tracer->trace_memory;
	union {
		struct camrtc_trace_armv7_exception exc;
		uint8_t mem[CAMRTC_TRACE_EXCEPTION_SIZE];
	} exc;
	u32 old_next = tracer->exception_last_idx;
	u32 new_next = header->exception_next_idx;

	if (old_next == new_next)
		return;

	if (new_next >= tracer->exception_entries) {
		WARN_ON_ONCE(new_next >= tracer->exception_entries);
		dev_warn_ratelimited(tracer->dev,
			"exception entry %u outside range 0..%u\n",
			new_next, tracer->exception_entries - 1);
		return;
	}

	rtcpu_trace_invalidate_entries(tracer,
				tracer->dma_handle_exceptions,
				old_next, new_next,
				CAMRTC_TRACE_EXCEPTION_SIZE,
				tracer->exception_entries);

	while (old_next != new_next) {
		void *emem = tracer->exceptions_base +
			CAMRTC_TRACE_EXCEPTION_SIZE * old_next;
		memcpy(&exc.mem, emem, CAMRTC_TRACE_EXCEPTION_SIZE);
		rtcpu_trace_exception(tracer, &exc.exc);
		++tracer->n_exceptions;
		if (++old_next == tracer->exception_entries)
			old_next = 0;
	}

	tracer->exception_last_idx = new_next;
}

static void rtcpu_trace_base_event(struct camrtc_event_struct *event)
{
	switch (event->header.id) {
	case camrtc_trace_base_target_init:
		trace_rtcpu_target_init(event->header.tstamp);
		break;
	case camrtc_trace_base_start_scheduler:
		trace_rtcpu_start_scheduler(event->header.tstamp);
		break;
	default:
		trace_rtcpu_unknown(event->header.tstamp,
		    event->header.id,
		    event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		    event->data.data8);
		break;
	}
}

static void rtcpu_trace_rtos_event(struct camrtc_event_struct *event)
{
	switch (event->header.id) {
	case camrtc_trace_rtos_task_switched_in:
		trace_rtos_task_switched_in(event->header.tstamp);
		break;
	case camrtc_trace_rtos_increase_tick_count:
		trace_rtos_increase_tick_count(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_low_power_idle_begin:
		trace_rtos_low_power_idle_begin(event->header.tstamp);
		break;
	case camrtc_trace_rtos_low_power_idle_end:
		trace_rtos_low_power_idle_end(event->header.tstamp);
		break;
	case camrtc_trace_rtos_task_switched_out:
		trace_rtos_task_switched_out(event->header.tstamp);
		break;
	case camrtc_trace_rtos_task_priority_inherit:
		trace_rtos_task_priority_inherit(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_task_priority_disinherit:
		trace_rtos_task_priority_disinherit(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_blocking_on_queue_receive:
		trace_rtos_blocking_on_queue_receive(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_blocking_on_queue_send:
		trace_rtos_blocking_on_queue_send(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_moved_task_to_ready_state:
		trace_rtos_moved_task_to_ready_state(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_create:
		trace_rtos_queue_create(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_create_failed:
		trace_rtos_queue_create_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_create_mutex:
		trace_rtos_create_mutex(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_create_mutex_failed:
		trace_rtos_create_mutex_failed(event->header.tstamp);
		break;
	case camrtc_trace_rtos_give_mutex_recursive:
		trace_rtos_give_mutex_recursive(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_give_mutex_recursive_failed:
		trace_rtos_give_mutex_recursive_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_take_mutex_recursive:
		trace_rtos_take_mutex_recursive(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_take_mutex_recursive_failed:
		trace_rtos_take_mutex_recursive_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_create_counting_semaphore:
		trace_rtos_create_counting_semaphore(event->header.tstamp);
		break;
	case camrtc_trace_rtos_create_counting_semaphore_failed:
		trace_rtos_create_counting_semaphore_failed(
			event->header.tstamp);
		break;
	case camrtc_trace_rtos_queue_send:
		trace_rtos_queue_send(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_send_failed:
		trace_rtos_queue_send_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_receive:
		trace_rtos_queue_receive(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_peek:
		trace_rtos_queue_peek(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_peek_from_isr:
		trace_rtos_queue_peek_from_isr(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_receive_failed:
		trace_rtos_queue_receive_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_send_from_isr:
		trace_rtos_queue_send_from_isr(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_send_from_isr_failed:
		trace_rtos_queue_send_from_isr_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_receive_from_isr:
		trace_rtos_queue_receive_from_isr(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_receive_from_isr_failed:
		trace_rtos_queue_receive_from_isr_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_peek_from_isr_failed:
		trace_rtos_queue_peek_from_isr_failed(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_queue_delete:
		trace_rtos_queue_delete(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_task_create:
		trace_rtos_task_create(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_task_create_failed:
		trace_rtos_task_create_failed(event->header.tstamp);
		break;
	case camrtc_trace_rtos_task_delete:
		trace_rtos_task_delete(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_task_delay_until:
		trace_rtos_task_delay_until(event->header.tstamp);
		break;
	case camrtc_trace_rtos_task_delay:
		trace_rtos_task_delay(event->header.tstamp);
		break;
	case camrtc_trace_rtos_task_priority_set:
		trace_rtos_task_priority_set(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_task_suspend:
		trace_rtos_task_suspend(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_task_resume:
		trace_rtos_task_resume(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_task_resume_from_isr:
		trace_rtos_task_resume_from_isr(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_task_increment_tick:
		trace_rtos_task_increment_tick(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_timer_create:
		trace_rtos_timer_create(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_timer_create_failed:
		trace_rtos_timer_create_failed(event->header.tstamp);
		break;
	case camrtc_trace_rtos_timer_command_send:
		trace_rtos_timer_command_send(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1],
			event->data.data32[2],
			event->data.data32[3]);
		break;
	case camrtc_trace_rtos_timer_expired:
		trace_rtos_timer_expired(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_timer_command_received:
		trace_rtos_timer_command_received(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1],
			event->data.data32[2]);
		break;
	case camrtc_trace_rtos_malloc:
		trace_rtos_malloc(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_free:
		trace_rtos_free(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_event_group_create:
		trace_rtos_event_group_create(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_event_group_create_failed:
		trace_rtos_event_group_create_failed(event->header.tstamp);
		break;
	case camrtc_trace_rtos_event_group_sync_block:
		trace_rtos_event_group_sync_block(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1],
			event->data.data32[2]);
		break;
	case camrtc_trace_rtos_event_group_sync_end:
		trace_rtos_event_group_sync_end(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1],
			event->data.data32[2],
			event->data.data32[3]);
		break;
	case camrtc_trace_rtos_event_group_wait_bits_block:
		trace_rtos_event_group_wait_bits_block(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_event_group_wait_bits_end:
		trace_rtos_event_group_wait_bits_end(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1],
			event->data.data32[2]);
		break;
	case camrtc_trace_rtos_event_group_clear_bits:
		trace_rtos_event_group_clear_bits(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_event_group_clear_bits_from_isr:
		trace_rtos_event_group_clear_bits_from_isr(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_event_group_set_bits:
		trace_rtos_event_group_set_bits(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_event_group_set_bits_from_isr:
		trace_rtos_event_group_set_bits_from_isr(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	case camrtc_trace_rtos_event_group_delete:
		trace_rtos_event_group_delete(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_rtos_pend_func_call:
		trace_rtos_pend_func_call(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1],
			event->data.data32[2],
			event->data.data32[3]);
		break;
	case camrtc_trace_rtos_pend_func_call_from_isr:
		trace_rtos_pend_func_call_from_isr(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1],
			event->data.data32[2],
			event->data.data32[3]);
		break;
	case camrtc_trace_rtos_queue_registry_add:
		trace_rtos_queue_registry_add(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	default:
		trace_rtcpu_unknown(event->header.tstamp,
		    event->header.id,
		    event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		    event->data.data8);
		break;
	}
}

static void rtcpu_trace_dbg_event(struct camrtc_event_struct *event)
{
	switch (event->header.id) {
	case camrtc_trace_dbg_unknown:
		trace_rtcpu_dbg_unknown(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_dbg_enter:
		trace_rtcpu_dbg_enter(event->header.tstamp,
			event->data.data32[0]);
		break;
	case camrtc_trace_dbg_exit:
		trace_rtcpu_dbg_exit(event->header.tstamp);
		break;
	case camrtc_trace_dbg_set_loglevel:
		trace_rtcpu_dbg_set_loglevel(event->header.tstamp,
			event->data.data32[0],
			event->data.data32[1]);
		break;
	default:
		trace_rtcpu_unknown(event->header.tstamp,
		    event->header.id,
		    event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		    event->data.data8);
		break;
	}
}

const char * const g_trace_vinotify_tag_strs[] = {
	"FS", "FE",
	"CSIMUX_FRAME", "CSIMUX_STREAM",
	"CHANSEL_PXL_SOF", "CHANSEL_PXL_EOF",
	"CHANSEL_EMBED_SOF", "CHANSEL_EMBED_EOF",
	"CHANSEL_NLINES", "CHANSEL_FAULT",
	"CHANSEL_FAULT_FE", "CHANSEL_NOMATCH",
	"CHANSEL_COLLISION", "CHANSEL_SHORT_FRAME",
	"CHANSEL_LOAD_FRAMED", "ATOMP_PACKER_OVERFLOW",
	"ATOMP_FS", "ATOMP_FE",
	"ATOMP_FRAME_DONE", "ATOMP_EMB_DATA_DONE",
	"ATOMP_FRAME_NLINES_DONE", "ATOMP_FRAME_TRUNCATED",
	"ATOMP_FRAME_TOSSED", "ATOMP_PDAF_DATA_DONE",
	"RESERVED_18", "RESERVED_19",
	"ISPBUF_FIFO_OVERFLOW", "ISPBUF_FS",
	"ISPBUF_FE", "VGP0_DONE",
	"VGP1_DONE", "FMLITE_DONE",
};
const unsigned int g_trace_vinotify_tag_str_count =
	ARRAY_SIZE(g_trace_vinotify_tag_strs);

#ifndef camrtc_trace_vinotify_event_ts64
#define camrtc_trace_vinotify_event_ts64 (camrtc_trace_vinotify_handle_msg + 1)
#endif

#ifndef camrtc_trace_vinotify_error_ts64
#define camrtc_trace_vinotify_error_ts64 (camrtc_trace_vinotify_handle_msg + 2)
#endif

static void rtcpu_trace_vinotify_event(struct camrtc_event_struct *event)
{
	switch (event->header.id) {
	case camrtc_trace_vinotify_handle_msg:
		trace_rtcpu_vinotify_handle_msg(event->header.tstamp,
		(event->data.data32[0] >> 1) & 0x7f, event->data.data32[0],
		event->data.data32[1], event->data.data32[2]);
		break;
	case camrtc_trace_vinotify_event_ts64:
		trace_rtcpu_vinotify_event(event->header.tstamp,
		(event->data.data32[0] >> 1) & 0x7f, event->data.data32[0],
		((u64)event->data.data32[3] << 32) | event->data.data32[1],
		event->data.data32[2]);
		break;
	case camrtc_trace_vinotify_error_ts64:
		trace_rtcpu_vinotify_error(event->header.tstamp,
		(event->data.data32[0] >> 1) & 0x7f, event->data.data32[0],
		((u64)event->data.data32[3] << 32) | event->data.data32[1],
		event->data.data32[2]);
		break;
	default:
		trace_rtcpu_unknown(event->header.tstamp,
		event->header.id,
		event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		event->data.data8);
		break;
	}
}


static void rtcpu_trace_vi_event(struct tegra_rtcpu_trace *tracer,
				struct camrtc_event_struct *event)
{
#if !defined(CONFIG_EVENTLIB) || \
	!defined(camrtc_trace_vi_frame_begin) || \
	!defined(camrtc_trace_vi_frame_end)
	trace_rtcpu_unknown(event->header.tstamp,
		event->header.id,
		event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		event->data.data8);
#else
	struct nvhost_device_data *pdata;
	struct nvhost_task_begin task_begin;
	struct nvhost_task_end task_end;
	u64 ts = 0;

	if (tracer->vi_platform_device == NULL)
		return;
	pdata = platform_get_drvdata(tracer->vi_platform_device);
	if (pdata == NULL)
		return;

	if (!pdata->eventlib_id) {
		pr_warn("%s kernel eventlib id %d cannot be found\n",
			__func__, pdata->eventlib_id);
		return;
	}

	switch (event->header.id) {
	case camrtc_trace_vi_frame_begin:
		/* Write task start event */
		task_begin.syncpt_id = event->data.data32[0];
		task_begin.syncpt_thresh = event->data.data32[1];
		task_begin.class_id = pdata->class;

		ts = ((u64)event->data.data32[5] << 32) |
			(u64)event->data.data32[4];
		keventlib_write(pdata->eventlib_id,
			&task_begin,
			sizeof(task_begin),
			NVHOST_TASK_BEGIN,
			ts);
		break;
	case camrtc_trace_vi_frame_end:
		/* Write task end event */
		task_end.syncpt_id = event->data.data32[0];
		task_end.syncpt_thresh = event->data.data32[1];
		task_end.class_id = pdata->class;

		ts = ((u64)event->data.data32[5] << 32) |
			(u64)event->data.data32[4];
		keventlib_write(pdata->eventlib_id,
			&task_end,
			sizeof(task_end),
			NVHOST_TASK_END,
			ts);
		break;
	default:
		pr_warn("%pFn event id %d cannot be found\n",
			__func__, pdata->eventlib_id);
		break;
	}
#endif
}

static void rtcpu_trace_isp_event(struct tegra_rtcpu_trace *tracer,
	struct camrtc_event_struct *event)
{
#ifndef CONFIG_EVENTLIB
	trace_rtcpu_unknown(event->header.tstamp,
		event->header.id,
		event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		event->data.data8);
#else
	struct nvhost_device_data *pdata = NULL;
	struct nvhost_task_begin task_begin;
	struct nvhost_task_end task_end;

	if (tracer->isp_platform_device == NULL)
		return;

	pdata = platform_get_drvdata(tracer->isp_platform_device);

	if (!pdata->eventlib_id) {
		pr_warn("%s kernel eventlib id %d cannot be found\n",
			__func__, pdata->eventlib_id);
		return;
	}

	switch (event->header.id) {
	case camrtc_trace_isp_task_begin:
		/* Write task start event */
		task_begin.syncpt_id = event->data.data32[0];
		task_begin.syncpt_thresh = event->data.data32[1];
		task_begin.class_id = pdata->class;

		keventlib_write(pdata->eventlib_id,
			&task_begin,
			sizeof(task_begin),
			NVHOST_TASK_BEGIN,
			event->header.tstamp);
		break;
	case camrtc_trace_isp_task_end:
		/* Write task end event */
		task_end.syncpt_id = event->data.data32[0];
		task_end.syncpt_thresh = event->data.data32[1];
		task_end.class_id = pdata->class;

		keventlib_write(pdata->eventlib_id,
			&task_end,
			sizeof(task_end),
			NVHOST_TASK_END,
			event->header.tstamp);
		break;
	default:
		pr_warn("%s event id %d cannot be found\n",
			__func__, pdata->eventlib_id);
		break;
	}
#endif
}

const char * const g_trace_nvcsi_intr_class_strs[] = {
	"GLOBAL",
	"CORRECTABLE_ERR",
	"UNCORRECTABLE_ERR",
};
const unsigned int g_trace_nvcsi_intr_class_str_count =
	ARRAY_SIZE(g_trace_nvcsi_intr_class_strs);

const char * const g_trace_nvcsi_intr_type_strs[] = {
	"SW_DEBUG",
	"HOST1X",
	"PHY_INTR", "PHY_INTR0", "PHY_INTR1",
	"STREAM_NOVC", "STREAM_VC",
};
const unsigned int g_trace_nvcsi_intr_type_str_count =
	ARRAY_SIZE(g_trace_nvcsi_intr_type_strs);

static void rtcpu_trace_nvcsi_event(struct camrtc_event_struct *event)
{
	u64 ts_tsc = ((u64)event->data.data32[5] << 32) |
			(u64)event->data.data32[4];

	switch (event->header.id) {
	case camrtc_trace_nvcsi_intr:
		trace_rtcpu_nvcsi_intr(ts_tsc,
			(event->data.data32[0] & 0xff),
			(event->data.data32[1] & 0xff),
			event->data.data32[2],
			event->data.data32[3]);
		break;
	default:
		trace_rtcpu_unknown(event->header.tstamp,
			event->header.id,
			event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
			event->data.data8);
		break;
	}
}

static void rtcpu_trace_array_event(struct tegra_rtcpu_trace *tracer,
	struct camrtc_event_struct *event)
{
	switch (CAMRTC_EVENT_MODULE_FROM_ID(event->header.id)) {
	case CAMRTC_EVENT_MODULE_BASE:
		rtcpu_trace_base_event(event);
		break;
	case CAMRTC_EVENT_MODULE_RTOS:
		rtcpu_trace_rtos_event(event);
		break;
	case CAMRTC_EVENT_MODULE_DBG:
		rtcpu_trace_dbg_event(event);
		break;
	case CAMRTC_EVENT_MODULE_VINOTIFY:
		rtcpu_trace_vinotify_event(event);
		break;
	case CAMRTC_EVENT_MODULE_I2C:
		break;
	case CAMRTC_EVENT_MODULE_VI:
		rtcpu_trace_vi_event(tracer, event);
		break;
	case CAMRTC_EVENT_MODULE_ISP:
		rtcpu_trace_isp_event(tracer, event);
		break;
	case CAMRTC_EVENT_MODULE_NVCSI:
		rtcpu_trace_nvcsi_event(event);
		break;
	default:
		trace_rtcpu_unknown(event->header.tstamp,
		    event->header.id,
		    event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		    event->data.data8);
		break;
	}
}

static void trace_rtcpu_log(struct tegra_rtcpu_trace *tracer,
			struct camrtc_event_struct *event)
{
	size_t len, used;

	if (unlikely(event->header.id != camrtc_trace_type_string))
		return;

	len = event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE;

	if (len == CAMRTC_TRACE_EVENT_PAYLOAD_SIZE)
		/* Ignore NULs at the end of buffer */
		len = strnlen(event->data.data8, len);

	used = tracer->printk_used;

	if (unlikely(used + len > sizeof(tracer->printk))) {
		/* Too long concatenated message, print it out now */
		pr_info("%s %.*s\n", tracer->log_prefix,
			(int)used, tracer->printk);
		used = 0;
	}

	memcpy(tracer->printk + used, event->data.data8, len);

	used += len;

	if (likely(used > 0)) {
		char end = tracer->printk[used - 1];

		/*
		 * Some log entries from rtcpu consists of multiple
		 * messages.  If the string does not end with \r or
		 * \n, do not print it now but rather wait for the
		 * next piece.
		 */
		if (end == '\r' || end == '\n') {
			while (--used > 0) {
				end = tracer->printk[used - 1];
				if (!(end == '\r' || end == '\n'))
					break;
			}

			pr_info("%s %.*s\n", tracer->log_prefix,
				(int)used, tracer->printk);
			used = 0;
		}
	}

	tracer->printk_used = used;
}

static void rtcpu_trace_event(struct tegra_rtcpu_trace *tracer,
	struct camrtc_event_struct *event)
{
	switch (CAMRTC_EVENT_TYPE_FROM_ID(event->header.id)) {
	case CAMRTC_EVENT_TYPE_ARRAY:
		rtcpu_trace_array_event(tracer, event);
		break;
	case CAMRTC_EVENT_TYPE_ARMV7_EXCEPTION:
		trace_rtcpu_armv7_exception(event->header.tstamp,
			event->data.data32[0]);
		break;
	case CAMRTC_EVENT_TYPE_PAD:
		/* ignore */
		break;
	case CAMRTC_EVENT_TYPE_START:
		trace_rtcpu_start(event->header.tstamp);
		break;
	case CAMRTC_EVENT_TYPE_STRING:
		trace_rtcpu_string(event->header.tstamp,
		    event->header.id,
			  event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
			  (char *) event->data.data8);
		if (likely(tracer->enable_printk))
			trace_rtcpu_log(tracer, event);
		break;
	case CAMRTC_EVENT_TYPE_BULK:
		trace_rtcpu_bulk(event->header.tstamp,
		    event->header.id,
		    event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		    event->data.data8);
		break;
	default:
		trace_rtcpu_unknown(event->header.tstamp,
		    event->header.id,
		    event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE,
		    event->data.data8);
		break;
	}
}

static inline void rtcpu_trace_events(struct tegra_rtcpu_trace *tracer)
{
	const struct camrtc_trace_memory_header *header = tracer->trace_memory;
	u32 old_next = tracer->event_last_idx;
	u32 new_next = header->event_next_idx;
	struct camrtc_event_struct *event, *last_event;

	while (old_next == new_next)
		return;

	if (new_next >= tracer->event_entries) {
		WARN_ON_ONCE(new_next >= tracer->event_entries);
		dev_warn_ratelimited(tracer->dev,
			"trace entry %u outside range 0..%u\n",
			new_next, tracer->event_entries - 1);
		return;
	}

	rtcpu_trace_invalidate_entries(tracer,
				tracer->dma_handle_events,
				old_next, new_next,
				CAMRTC_TRACE_EVENT_SIZE,
				tracer->event_entries);

	/* pull events */
	while (old_next != new_next) {
		event = &tracer->events[old_next];
		last_event = event;
		rtcpu_trace_event(tracer, event);
		tracer->n_events++;

		if (++old_next == tracer->event_entries)
			old_next = 0;
	}

	tracer->event_last_idx = new_next;
	tracer->copy_last_event = *last_event;
}

void tegra_rtcpu_trace_flush(struct tegra_rtcpu_trace *tracer)
{
	if (tracer == NULL)
		return;

	mutex_lock(&tracer->lock);

	/* invalidate the cache line for the pointers */
	dma_sync_single_for_cpu(tracer->dev, tracer->dma_handle_pointers,
	    CAMRTC_TRACE_NEXT_IDX_SIZE, DMA_FROM_DEVICE);

	/* process exceptions and events */
	rtcpu_trace_exceptions(tracer);
	rtcpu_trace_events(tracer);

	mutex_unlock(&tracer->lock);
}
EXPORT_SYMBOL(tegra_rtcpu_trace_flush);

static void rtcpu_trace_worker(struct work_struct *work)
{
	struct tegra_rtcpu_trace *tracer;

	tracer = container_of(work, struct tegra_rtcpu_trace, work.work);

	tegra_rtcpu_trace_flush(tracer);

	/* reschedule */
	schedule_delayed_work(&tracer->work, tracer->work_interval_jiffies);
}

/*
 * Debugfs
 */

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
	static int _fops_ ## _open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, _show_, inode->i_private); \
	} \
	static const struct file_operations _fops_ = { \
		.open = _fops_ ## _open, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.release = single_release }

static int rtcpu_trace_debugfs_stats_read(
	struct seq_file *file, void *data)
{
	struct tegra_rtcpu_trace *tracer = file->private;

	seq_printf(file, "Exceptions: %u\nEvents: %llu\n",
			tracer->n_exceptions, tracer->n_events);

	return 0;
}

DEFINE_SEQ_FOPS(rtcpu_trace_debugfs_stats, rtcpu_trace_debugfs_stats_read);

static int rtcpu_trace_debugfs_last_exception_read(
	struct seq_file *file, void *data)
{
	struct tegra_rtcpu_trace *tracer = file->private;

	seq_puts(file, tracer->last_exception_str);

	return 0;
}

DEFINE_SEQ_FOPS(rtcpu_trace_debugfs_last_exception,
	rtcpu_trace_debugfs_last_exception_read);

static int rtcpu_trace_debugfs_last_event_read(
	struct seq_file *file, void *data)
{
	struct tegra_rtcpu_trace *tracer = file->private;
	struct camrtc_event_struct *event = &tracer->copy_last_event;
	unsigned int i, payload_len;

	if (tracer->n_events == 0)
		return 0;

	payload_len = event->header.len - CAMRTC_TRACE_EVENT_HEADER_SIZE;

	seq_printf(file, "Len: %u\nID: 0x%08x\nTimestamp: %llu\n",
	    event->header.len, event->header.id, event->header.tstamp);

	switch (CAMRTC_EVENT_TYPE_FROM_ID(event->header.id)) {
	case CAMRTC_EVENT_TYPE_ARRAY:
		for (i = 0; i < payload_len / 4; ++i)
			seq_printf(file, "0x%08x ", event->data.data32[i]);
		seq_puts(file, "\n");
		break;
	case CAMRTC_EVENT_TYPE_ARMV7_EXCEPTION:
		seq_puts(file, "Exception.\n");
		break;
	case CAMRTC_EVENT_TYPE_PAD:
		break;
	case CAMRTC_EVENT_TYPE_START:
		seq_puts(file, "Start.\n");
		break;
	case CAMRTC_EVENT_TYPE_STRING:
		seq_puts(file, (char *) event->data.data8);
		break;
	case CAMRTC_EVENT_TYPE_BULK:
		for (i = 0; i < payload_len; ++i)
			seq_printf(file, "0x%02x ", event->data.data8[i]);
		seq_puts(file, "\n");
		break;
	default:
		seq_puts(file, "Unknown type.\n");
		break;
	}

	return 0;
}

DEFINE_SEQ_FOPS(rtcpu_trace_debugfs_last_event,
	rtcpu_trace_debugfs_last_event_read);

static void rtcpu_trace_debugfs_deinit(struct tegra_rtcpu_trace *tracer)
{
	debugfs_remove_recursive(tracer->debugfs_root);
}

static void rtcpu_trace_debugfs_init(struct tegra_rtcpu_trace *tracer)
{
	struct dentry *entry;

	tracer->debugfs_root = debugfs_create_dir("tegra_rtcpu_trace", NULL);
	if (IS_ERR_OR_NULL(tracer->debugfs_root))
		return;

	entry = debugfs_create_file("stats", S_IRUGO,
	    tracer->debugfs_root, tracer, &rtcpu_trace_debugfs_stats);
	if (IS_ERR_OR_NULL(entry))
		goto failed_create;

	entry = debugfs_create_file("last_exception", S_IRUGO,
	    tracer->debugfs_root, tracer, &rtcpu_trace_debugfs_last_exception);
	if (IS_ERR_OR_NULL(entry))
		goto failed_create;

	entry = debugfs_create_file("last_event", S_IRUGO,
	    tracer->debugfs_root, tracer, &rtcpu_trace_debugfs_last_event);
	if (IS_ERR_OR_NULL(entry))
		goto failed_create;

	return;

failed_create:
	debugfs_remove_recursive(tracer->debugfs_root);
}

/*
 * Init/Cleanup
 */

struct tegra_rtcpu_trace *tegra_rtcpu_trace_create(struct device *dev,
	struct camrtc_device_group *camera_devices)
{
	struct tegra_rtcpu_trace *tracer;
	u32 param;
	int ret;

	tracer = kzalloc(sizeof(*tracer), GFP_KERNEL);
	if (unlikely(tracer == NULL))
		return NULL;

	tracer->dev = dev;
	mutex_init(&tracer->lock);

	/* Get the trace memory */
	ret = rtcpu_trace_setup_memory(tracer);
	if (ret) {
		dev_err(dev, "Trace memory setup failed: %d\n", ret);
		kfree(tracer);
		return NULL;
	}

	/* Initialize the trace memory */
	rtcpu_trace_init_memory(tracer);

	/* Debugfs */
	rtcpu_trace_debugfs_init(tracer);

#ifdef CONFIG_EVENTLIB
	if (camera_devices != NULL) {
		/* Eventlib */
		tracer->isp_platform_device =
			camrtc_device_get_byname(camera_devices, "isp");
		if (IS_ERR(tracer->isp_platform_device)) {
			dev_info(dev, "no camera-device \"%s\"\n", "isp");
			tracer->isp_platform_device = NULL;
		}
		tracer->vi_platform_device =
			camrtc_device_get_byname(camera_devices, "vi");
		if (IS_ERR(tracer->vi_platform_device)) {
			dev_info(dev, "no camera-device \"%s\"\n", "vi");
			tracer->vi_platform_device = NULL;
		}
	}
#endif

	/* Worker */
	param = WORK_INTERVAL_DEFAULT;
	of_property_read_u32(tracer->of_node, NV(interval-ms), &param);

	tracer->enable_printk = of_property_read_bool(tracer->of_node,
						NV(enable-printk));

	tracer->log_prefix = "[RTCPU]";
	of_property_read_string(tracer->of_node, NV(log-prefix),
				&tracer->log_prefix);

	INIT_DELAYED_WORK(&tracer->work, rtcpu_trace_worker);
	tracer->work_interval_jiffies = msecs_to_jiffies(param);

	/* Done with initialization */
	schedule_delayed_work(&tracer->work, 0);

	dev_info(dev, "Trace buffer configured at IOVA=0x%08x\n",
		 (u32)tracer->dma_handle);

	return tracer;
}
EXPORT_SYMBOL(tegra_rtcpu_trace_create);

int tegra_rtcpu_trace_boot_sync(struct tegra_rtcpu_trace *tracer)
{
	int ret = tegra_camrtc_iovm_setup(tracer->dev, tracer->dma_handle);

	if (ret == 0)
		return 0;

	dev_err(tracer->dev, "RTCPU trace: IOVM setup error: %d\n", ret);

	return -EIO;
}
EXPORT_SYMBOL(tegra_rtcpu_trace_boot_sync);

void tegra_rtcpu_trace_destroy(struct tegra_rtcpu_trace *tracer)
{
	if (IS_ERR_OR_NULL(tracer))
		return;
	platform_device_put(tracer->isp_platform_device);
	platform_device_put(tracer->vi_platform_device);
	of_node_put(tracer->of_node);
	cancel_delayed_work_sync(&tracer->work);
	flush_delayed_work(&tracer->work);
	rtcpu_trace_debugfs_deinit(tracer);
	dma_free_coherent(tracer->dev, tracer->trace_memory_size,
			tracer->trace_memory, tracer->dma_handle);
	kfree(tracer);
}
EXPORT_SYMBOL(tegra_rtcpu_trace_destroy);

MODULE_DESCRIPTION("NVIDIA Tegra RTCPU trace driver");
MODULE_LICENSE("GPL v2");
