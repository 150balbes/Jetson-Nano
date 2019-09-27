/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/ktime.h>
#include <linux/delay.h>

#include <nvgpu/timers.h>
#include <nvgpu/soc.h>
#include <nvgpu/gk20a.h>

#include "platform_gk20a.h"

/*
 * Returns 1 if the platform is pre-Si and should ignore the timeout checking.
 * Setting %NVGPU_TIMER_NO_PRE_SI will make this always return 0 (i.e do the
 * timeout check regardless of platform).
 */
static int nvgpu_timeout_is_pre_silicon(struct nvgpu_timeout *timeout)
{
	if (timeout->flags & NVGPU_TIMER_NO_PRE_SI)
		return 0;

	return !nvgpu_platform_is_silicon(timeout->g);
}

/**
 * nvgpu_timeout_init - Init timer.
 *
 * @g        - nvgpu device.
 * @timeout  - The timer.
 * @duration - Timeout in milliseconds or number of retries.
 * @flags    - Flags for timer.
 *
 * This configures the timeout to start the timeout duration now, i.e: when this
 * function is called. Available flags to pass to @flags:
 *
 *   %NVGPU_TIMER_CPU_TIMER
 *   %NVGPU_TIMER_RETRY_TIMER
 *   %NVGPU_TIMER_NO_PRE_SI
 *   %NVGPU_TIMER_SILENT_TIMEOUT
 *
 * If neither %NVGPU_TIMER_CPU_TIMER or %NVGPU_TIMER_RETRY_TIMER is passed then
 * a CPU timer is used by default.
 */
int nvgpu_timeout_init(struct gk20a *g, struct nvgpu_timeout *timeout,
		       u32 duration, unsigned long flags)
{
	if (flags & ~NVGPU_TIMER_FLAG_MASK)
		return -EINVAL;

	memset(timeout, 0, sizeof(*timeout));

	timeout->g = g;
	timeout->flags = flags;

	if (flags & NVGPU_TIMER_RETRY_TIMER)
		timeout->retries.max = duration;
	else
		timeout->time = ktime_to_ns(ktime_add_ns(ktime_get(),
					(s64)NSEC_PER_MSEC * duration));

	return 0;
}

static int __nvgpu_timeout_expired_msg_cpu(struct nvgpu_timeout *timeout,
					 void *caller,
					 const char *fmt, va_list args)
{
	struct gk20a *g = timeout->g;
	ktime_t now = ktime_get();

	if (nvgpu_timeout_is_pre_silicon(timeout))
		return 0;

	if (ktime_after(now, ns_to_ktime(timeout->time))) {
		if (!(timeout->flags & NVGPU_TIMER_SILENT_TIMEOUT)) {
			char buf[128];

			vsnprintf(buf, sizeof(buf), fmt, args);

			nvgpu_err(g, "Timeout detected @ %pF %s", caller, buf);
		}

		return -ETIMEDOUT;
	}

	return 0;
}

static int __nvgpu_timeout_expired_msg_retry(struct nvgpu_timeout *timeout,
					   void *caller,
					   const char *fmt, va_list args)
{
	struct gk20a *g = timeout->g;

	if (nvgpu_timeout_is_pre_silicon(timeout))
		return 0;

	if (timeout->retries.attempted >= timeout->retries.max) {
		if (!(timeout->flags & NVGPU_TIMER_SILENT_TIMEOUT)) {
			char buf[128];

			vsnprintf(buf, sizeof(buf), fmt, args);

			nvgpu_err(g, "No more retries @ %pF %s", caller, buf);
		}

		return -ETIMEDOUT;
	}

	timeout->retries.attempted++;

	return 0;
}

/**
 * __nvgpu_timeout_expired_msg - Check if a timeout has expired.
 *
 * @timeout - The timeout to check.
 * @caller  - Address of the caller of this function.
 * @fmt     - The fmt string.
 *
 * Returns -ETIMEDOUT if the timeout has expired, 0 otherwise.
 *
 * If a timeout occurs and %NVGPU_TIMER_SILENT_TIMEOUT is not set in the timeout
 * then a message is printed based on %fmt.
 */
int __nvgpu_timeout_expired_msg(struct nvgpu_timeout *timeout,
			      void *caller, const char *fmt, ...)
{
	int ret;
	va_list args;

	va_start(args, fmt);
	if (timeout->flags & NVGPU_TIMER_RETRY_TIMER)
		ret = __nvgpu_timeout_expired_msg_retry(timeout, caller, fmt,
						      args);
	else
		ret = __nvgpu_timeout_expired_msg_cpu(timeout, caller, fmt,
						    args);
	va_end(args);

	return ret;
}

/**
 * nvgpu_timeout_peek_expired - Check the status of a timeout.
 *
 * @timeout - The timeout to check.
 *
 * Returns non-zero if the timeout is expired, zero otherwise. In the case of
 * retry timers this will not increment the underlying retry count. Also if the
 * timer has expired no messages will be printed.
 *
 * This function honors the pre-Si check as well.
 */
int nvgpu_timeout_peek_expired(struct nvgpu_timeout *timeout)
{
	if (nvgpu_timeout_is_pre_silicon(timeout))
		return 0;

	if (timeout->flags & NVGPU_TIMER_RETRY_TIMER)
		return timeout->retries.attempted >= timeout->retries.max;
	else
		return ktime_after(ktime_get(), ns_to_ktime(timeout->time));
}

/**
 * nvgpu_udelay - Delay for some number of microseconds.
 *
 * @usecs - Microseconds to wait for.
 *
 * Wait for at least @usecs microseconds. This is not guaranteed to be perfectly
 * accurate. This is normally backed by a busy-loop so this means waits should
 * be kept short, below 100us. If longer delays are necessary then
 * nvgpu_msleep() should be preferred.
 *
 * Alternatively, on some platforms, nvgpu_usleep_range() is usable. This
 * function will attempt to not use a busy-loop.
 */
void nvgpu_udelay(unsigned int usecs)
{
	udelay(usecs);
}

/**
 * nvgpu_usleep_range - Sleep for a range of microseconds.
 *
 * @min_us - Minimum wait time.
 * @max_us - Maximum wait time.
 *
 * Wait for some number of microseconds between @min_us and @max_us. This,
 * unlike nvgpu_udelay(), will attempt to sleep for the passed number of
 * microseconds instead of busy looping. Not all platforms support this,
 * and in that case this reduces to nvgpu_udelay(min_us).
 *
 * Linux note: this is not safe to use in atomic context. If you are in
 * atomic context you must use nvgpu_udelay().
 */
void nvgpu_usleep_range(unsigned int min_us, unsigned int max_us)
{
	usleep_range(min_us, max_us);
}

/**
 * nvgpu_msleep - Sleep for some milliseconds.
 *
 * @msecs - Sleep for at least this many milliseconds.
 *
 * Sleep for at least @msecs of milliseconds. For small @msecs (less than 20 ms
 * or so) the sleep will be significantly longer due to scheduling overhead and
 * mechanics.
 */
void nvgpu_msleep(unsigned int msecs)
{
	msleep(msecs);
}

/**
 * nvgpu_current_time_ms - Time in milliseconds from a monotonic clock.
 *
 * Return a clock in millisecond units. The start time of the clock is
 * unspecified; the time returned can be compared with older ones to measure
 * durations. The source clock does not jump when the system clock is adjusted.
 */
s64 nvgpu_current_time_ms(void)
{
	return ktime_to_ms(ktime_get());
}

/**
 * nvgpu_current_time_ns - Time in nanoseconds from a monotonic clock.
 *
 * Return a clock in nanosecond units. The start time of the clock is
 * unspecified; the time returned can be compared with older ones to measure
 * durations. The source clock does not jump when the system clock is adjusted.
 */
s64 nvgpu_current_time_ns(void)
{
	return ktime_to_ns(ktime_get());
}

/**
 * nvgpu_hr_timestamp - Opaque 'high resolution' time stamp.
 *
 * Return a "high resolution" time stamp. It does not really matter exactly what
 * it is, so long as it generally returns unique values and monotonically
 * increases - wrap around _is_ possible though in a system running for long
 * enough.
 *
 * Note: what high resolution means is system dependent.
 */
u64 nvgpu_hr_timestamp(void)
{
	return get_cycles();
}
