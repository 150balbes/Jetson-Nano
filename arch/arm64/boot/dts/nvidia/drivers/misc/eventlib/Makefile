#
# drivers/misc/eventlib/Makefile.nvidia
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
#

ccflags-y += -Werror

obj-$(CONFIG_EVENTLIB) := eventlib-kernel.o

eventlib-kernel-y :=		\
	eventlib_flt.o		\
	eventlib_init.o		\
	eventlib_tbuf.o		\
	tracebuf.o		\
	eventlib.o
