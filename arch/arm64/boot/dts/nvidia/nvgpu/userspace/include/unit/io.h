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

#ifndef __UNIT_IO_H__
#define __UNIT_IO_H__

struct unit_fw;
struct unit_module;

/*
 * necessary for args(fw) macro. IO will always, in general, depend on args
 * since the args will specify where IO should be directed to.
 */
#include <unit/args.h>

#define core_msg(fw, msg, ...)						\
	core_vbs(fw, 0, msg, ##__VA_ARGS__)
#define core_msg_color(fw, color, msg, ...)				\
	core_vbs_color(fw, color, 0, msg, ##__VA_ARGS__)

#define core_vbs_color(fw, color, lvl, msg, ...)			\
	do {								\
		if ((lvl) > args(fw)->verbose_lvl)			\
			continue;					\
									\
		/* Print if verbosity level is high enough. */		\
		__core_print_stdout(fw, color, msg, ##__VA_ARGS__);	\
	} while (0)
#define core_vbs(fw, lvl, msg, ...)					\
	core_vbs_color(fw, NULL, lvl, msg, ##__VA_ARGS__)

#define core_err(fw, msg, ...)						\
	__core_print_stderr(fw, "(%s:%d) " msg,				\
			    __func__, __LINE__, ##__VA_ARGS__)

/*
 * Output macro for unit tests to use.
 */
#define unit_info(unit, msg, ...)					\
	__unit_info_color(unit, NULL, msg, ##__VA_ARGS__)
#define unit_err(unit, msg, ...)					\
	__unit_info_color(unit, C_RED, msg, ##__VA_ARGS__)

/*
 * Don't go overboard with these!!!
 */
#define C_RED     "\x1B[31m"
#define C_GREEN   "\x1B[32m"
#define C_YELLOW  "\x1B[33m"
#define C_BLUE    "\x1B[34m"
#define C_MAGENTA "\x1B[35m"
#define C_CYAN    "\x1B[36m"
#define C_WHITE   "\x1B[37m"
#define C_RESET   "\x1B[0m"

/*
 * Printing functions. Do not use these directly. Instead use the provided
 * macros.
 */
__attribute__((format (printf, 3, 4)))
void __core_print_stdout(struct unit_fw *fw, const char *color,
			 const char *fmt, ...);
__attribute__((format (printf, 2, 3)))
void __core_print_stderr(struct unit_fw *fw, const char *fmt, ...);
__attribute__((format (printf, 3, 4)))
void __unit_info_color(struct unit_module *unit, const char *color,
		       const char *fmt, ...);

#endif
