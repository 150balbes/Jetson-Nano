/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <sys/time.h>

#include <nvgpu/bug.h>
#include <nvgpu/log.h>
#include <nvgpu/timers.h>

static s64 now(void)
{
	return nvgpu_current_time_ms();
}

/*
 * Returns true if a > b;
 */
static bool time_after(s64 a, s64 b)
{
	return a - b > 0;
}

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
		timeout->time = nvgpu_current_time_ms() + (s64)duration;

	return 0;
}

static int __nvgpu_timeout_expired_msg_cpu(struct nvgpu_timeout *timeout,
					 void *caller,
					 const char *fmt, va_list args)
{
	struct gk20a *g = timeout->g;

	if (time_after(now(), timeout->time)) {
		if (!(timeout->flags & NVGPU_TIMER_SILENT_TIMEOUT)) {
			char buf[128];

			vsnprintf(buf, sizeof(buf), fmt, args);

			nvgpu_err(g, "Timeout detected @ %p %s", caller, buf);
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

	if (timeout->retries.attempted >= timeout->retries.max) {
		if (!(timeout->flags & NVGPU_TIMER_SILENT_TIMEOUT)) {
			char buf[128];

			vsnprintf(buf, sizeof(buf), fmt, args);

			nvgpu_err(g, "No more retries @ %p %s", caller, buf);
		}

		return -ETIMEDOUT;
	}

	timeout->retries.attempted++;

	return 0;
}

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

int nvgpu_timeout_peek_expired(struct nvgpu_timeout *timeout)
{
	if (timeout->flags & NVGPU_TIMER_RETRY_TIMER)
		return timeout->retries.attempted >= timeout->retries.max;
	else
		return time_after(now(), timeout->time);
}

void nvgpu_udelay(unsigned int usecs)
{
	BUG();
}

void nvgpu_usleep_range(unsigned int min_us, unsigned int max_us)
{
	BUG();
}

void nvgpu_msleep(unsigned int msecs)
{
	BUG();
}

static inline s64 __nvgpu_current_time_us(void)
{
	struct timeval now;
	s64 time_now;
	int ret;

	ret = gettimeofday(&now, NULL);
	if (ret != 0)
		BUG();

	time_now = ((s64)now.tv_sec * (s64)1000000) + (s64)now.tv_usec;

	return time_now;
}

s64 nvgpu_current_time_ms(void)
{
	return __nvgpu_current_time_us() / (s64)1000;
}

u64 nvgpu_hr_timestamp(void)
{
	return __nvgpu_current_time_us();
}
