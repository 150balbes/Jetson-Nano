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

#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#include <nvgpu/bug.h>
#include <nvgpu/types.h>
#include <nvgpu/atomic.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/os_sched.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/posix/probe.h>

#include "os_posix.h"

void nvgpu_wait_for_deferred_interrupts(struct gk20a *g)
{
	/*
	 * No interrupts in userspace so nothing to wait for.
	 */
}

int nvgpu_current_pid(struct gk20a *g)
{
	/*
	 * In the kernel this gets us the PID of the calling process for IOCTLs.
	 * But since we are in userspace this doesn't quite mean the same thing.
	 * This simply returns the PID of the currently running process.
	 */
	return (int)getpid();
}

int nvgpu_current_tid(struct gk20a *g)
{
	/*
	 * In POSIX thread ID is not the same as a process ID. In Linux threads
	 * and processes are represented by the same thing, but userspace can't
	 * really rely on that.
	 *
	 * We can, however, get a pthread_t for a given thread. But this
	 * pthread_t need not have any relation to the underlying system's
	 * representation of "threads".
	 */
	return (int)pthread_self();
}

void __nvgpu_print_current(struct gk20a *g, const char *func_name, int line,
		void *ctx, enum nvgpu_log_type type)
{
	__nvgpu_log_msg(g, func_name, line, type,
			"Current process: (nvgpu userspace)");
}

/*
 * Somewhat meaningless in userspace...
 */
void nvgpu_kernel_restart(void *cmd)
{
	BUG();
}

/*
 * We have no runtime PM stuff in userspace so these are really just noops.
 */
void gk20a_busy_noresume(struct gk20a *g)
{
}

void gk20a_idle_nosuspend(struct gk20a *g)
{
}

int gk20a_busy(struct gk20a *g)
{
	nvgpu_atomic_inc(&g->usage_count);

	return 0;
}

void gk20a_idle(struct gk20a *g)
{
	nvgpu_atomic_dec(&g->usage_count);
}

/*
 * This function aims to initialize enough stuff to make unit testing worth
 * while. There are several interfaces and APIs that rely on the struct gk20a's
 * state in order to function: logging, for example, but there are many other
 * things, too.
 *
 * Initialize as much of that as possible here. This is meant to be equivalent
 * to the kernel space driver's probe function.
 */
struct gk20a *nvgpu_posix_probe(void)
{
	struct gk20a *g;
	struct nvgpu_os_posix *p;
	int err;

	p = malloc(sizeof(*p));
	if (p == NULL)
		return NULL;

	g = &p->g;

	err = nvgpu_kmem_init(g);
	if (err != 0)
		goto fail;

	return g;

fail:
	free(p);

	return NULL;
}

void nvgpu_posix_cleanup(struct gk20a *g)
{
	nvgpu_kmem_fini(g, 0);
}
