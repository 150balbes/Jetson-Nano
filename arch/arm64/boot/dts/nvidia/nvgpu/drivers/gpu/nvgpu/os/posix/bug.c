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

#include <nvgpu/log.h>

#include <nvgpu/posix/bug.h>

__attribute__ ((noreturn))
static void __hang(void)
{
	nvgpu_err(NULL, "Hanging!");

	while (1)
		;
}

static void __dump_stack(unsigned int skip_frames)
{
	return;
}

void dump_stack(void)
{
	__dump_stack(0);
}

/*
 * Ahhh! A bug!
 */
void __bug(const char *fmt, ...)
{
	nvgpu_err(NULL, "BUG detected!");

	__hang();
}

bool __warn(bool cond, const char *fmt, ...)
{
	if (!cond)
		goto done;

	nvgpu_warn(NULL, "WARNING detected!");

	dump_stack();

done:
	return cond;
}
