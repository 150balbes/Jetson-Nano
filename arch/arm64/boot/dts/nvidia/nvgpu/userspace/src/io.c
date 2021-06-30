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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <unit/io.h>
#include <unit/args.h>
#include <unit/core.h>
#include <unit/unit.h>

#define MAX_LOG_LINE_LENGTH		4096

static void __core_print_file(struct unit_fw *fw, FILE *filp,
			      const char *prefix, const char *msg,
			      const char *color)
{
	if (color == NULL || args(fw)->no_color)
		color = "";

	fprintf(filp, "[%s%s%s] %s%s%s",
		color, prefix, C_RESET,
		color, msg, C_RESET);
}

__attribute__((format (printf, 3, 4)))
void __core_print_stdout(struct unit_fw *fw, const char *color,
			 const char *fmt, ...)
{
	va_list args;
	char buf[MAX_LOG_LINE_LENGTH];

	va_start(args, fmt);
	vsnprintf(buf, MAX_LOG_LINE_LENGTH, fmt, args);
	va_end(args);

	buf[MAX_LOG_LINE_LENGTH - 1] = 0;

	__core_print_file(fw, stdout, "C", buf, color);
}

__attribute__((format (printf, 2, 3)))
void __core_print_stderr(struct unit_fw *fw, const char *fmt, ...)
{
	va_list args;
	char buf[MAX_LOG_LINE_LENGTH];

	va_start(args, fmt);
	vsnprintf(buf, MAX_LOG_LINE_LENGTH, fmt, args);
	va_end(args);

	buf[MAX_LOG_LINE_LENGTH - 1] = 0;

	__core_print_file(fw, stdout, "E", buf, C_RED);
}

__attribute__((format (printf, 3, 4)))
void __unit_info_color(struct unit_module *unit, const char *color,
		       const char *fmt, ...)
{
	va_list args;
	char buf[MAX_LOG_LINE_LENGTH];
	char *msg_start;
	int written;

	/*
	 * Default color for module prints is blue. Users can still turn this
	 * off with '-C'.
	 */
	if (color == NULL)
		color = C_BLUE;

	/*
	 * First prepend the unit name to the print.
	 */
	written = snprintf(buf, MAX_LOG_LINE_LENGTH, "  [%s] ", unit->name);

	msg_start = buf + written;

	va_start(args, fmt);
	vsnprintf(msg_start, MAX_LOG_LINE_LENGTH - written, fmt, args);
	va_end(args);

	__core_print_file(unit->fw, stdout, "T", buf, color);
}
