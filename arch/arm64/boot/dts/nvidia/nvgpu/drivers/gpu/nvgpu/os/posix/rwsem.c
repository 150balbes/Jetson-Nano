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

#include <nvgpu/bug.h>
#include <nvgpu/rwsem.h>
#include <nvgpu/timers.h>

#include <nvgpu/posix/rwsem.h>

void nvgpu_rwsem_init(struct nvgpu_rwsem *rwsem)
{
	memset(rwsem, 0, sizeof(*rwsem));

	nvgpu_spinlock_init(&rwsem->lock);
}

/*
 * Acquire.
 */
void nvgpu_rwsem_down_read(struct nvgpu_rwsem *rwsem)
{
	while (true) {
		nvgpu_spinlock_acquire(&rwsem->lock);

		/*
		 * If there's a writer try again.
		 */
		if (rwsem->writers < 0) {
			nvgpu_spinlock_release(&rwsem->lock);
			nvgpu_msleep(10);
			continue;
		}

		/*
		 * Otherwise decrement the read counter and return.
		 */
		rwsem->readers -= 1;
		nvgpu_spinlock_release(&rwsem->lock);
		return;
	}
}

/*
 * Release.
 */
void nvgpu_rwsem_up_read(struct nvgpu_rwsem *rwsem)
{
	nvgpu_spinlock_acquire(&rwsem->lock);
	rwsem->readers += 1;

	/*
	 * Can't be any writers if there was a reader. Also can't be
	 * a positive number of readers. The increments are always
	 * downward so if we have a positive number then there is a
	 * balancing bug.
	 */
	BUG_ON(rwsem->writers < 0);
	BUG_ON(rwsem->readers > 0);

	nvgpu_spinlock_release(&rwsem->lock);
}

void nvgpu_rwsem_down_write(struct nvgpu_rwsem *rwsem)
{
	while (true) {
		nvgpu_spinlock_acquire(&rwsem->lock);

		/*
		 * If there's a reader or a writer try again. Note: in this very
		 * simple implementation it's possible for readers to
		 * indefinitely starve writers.
		 */
		if (rwsem->writers < 0 || rwsem->readers < 0) {
			nvgpu_spinlock_release(&rwsem->lock);
			nvgpu_msleep(10);
			continue;
		}

		rwsem->writers -= 1;
		nvgpu_spinlock_release(&rwsem->lock);
		return;
	}
}

void nvgpu_rwsem_up_write(struct nvgpu_rwsem *rwsem)
{
	nvgpu_spinlock_acquire(&rwsem->lock);
	rwsem->writers += 1;

	/*
	 * Writers can't be positive: that would be an unbalanced free. Readers
	 * must be zero - otherwise this writer should never have had access!
	 */
	BUG_ON(rwsem->writers > 0);
	BUG_ON(rwsem->readers != 0);

	nvgpu_spinlock_release(&rwsem->lock);
}
