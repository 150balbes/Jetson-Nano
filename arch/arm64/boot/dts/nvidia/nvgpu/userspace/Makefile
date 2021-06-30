# Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# TODO:
# - Separate rule for nvgpu_unit shared library
# - Proper header dependency checking.

# Turn off suffix rules. They are deprecated.
.SUFFIXES:

# Full paths. Makes submakefiles easier. That said this make file _must_ be run
# from <NVGPU>/userspace/.
TWD=$(CURDIR)

# Top level out dir.
OUT=$(TWD)/build

# Core unit test framework.
CORE_SRC=$(TWD)/src
CORE_OUT=$(OUT)/nvgpu_unit_core

# Nvgpu driver code.
NVGPU_SRC=$(TWD)/../drivers/gpu/nvgpu
NVGPU_OUT=$(OUT)/libnvgpu
# Nvgpu_next driver code.
NVGPU_NEXT_SRC=$(TWD)/../../nvgpu-next/drivers/gpu/nvgpu

# Unit tests themselves.
UNIT_SRC=$(TWD)/units
UNIT_OUT=$(OUT)/units

INCLUDES= 				\
	-I$(NVGPU_SRC)			\
	-I$(NVGPU_SRC)/include		\
	-I$(NVGPU_NEXT_SRC)		\
	-I$(NVGPU_NEXT_SRC)/include	\
	-I$(TWD)/../include		\
	-I$(TWD)/../include/uapi	\
	-I$(TWD)/include

CONFIGS=-D__NVGPU_POSIX__

# Compiler, c-flags, etc.

# CC      = clang
CC        = gcc
CFLAGS    = -Wall -ggdb  -Werror -fPIC $(INCLUDES) $(CONFIGS)
CFLAGS    = -Wall -Wextra -ggdb  -Werror -Wno-unused-parameter               \
	    -Wno-missing-field-initializers -Wformat -Wchar-subscripts       \
	    -Wparentheses -Wtrigraphs -Wpointer-arith -Wmissing-declarations \
	    -Wmissing-prototypes -Wredundant-decls -Wmain -Wreturn-type      \
	    -Wmultichar -Wunused -Wmissing-braces -Wstrict-aliasing          \
	    -Wsign-compare -Waddress -Wno-unused-local-typedefs -fPIC        \
	    $(INCLUDES) $(CONFIGS)
LIB_PATHS = -L$(OUT)
LIBS      = -lpthread -pthread -lgcov -ldl


# Source files. We expect $(OBJS) and $(HEADERS) to get filled in here.
include Makefile.sources

# Linuxy configs. We want these so that we can mirror builds from the actual
# Linux kernel.
include Makefile.configs

all: $(OUT)/nvgpu_unit $(UNITS)

# Convenience targets.
.PHONY: libnvgpu core units
libnvgpu: $(OUT)/libnvgpu-drv.so
core: $(OUT)/nvgpu_unit
units: $(UNITS)

# Note the weird libnvgpu_unit.so file: this is a bit of a hack. It lets the
# unit tests link back against the nvgpu_unit executable so that they can call
# functions (like unit_info()) directly. This shared library isn't actually
# used for anything beyond that.
#
# Also it really should have its own rule...
$(OUT)/nvgpu_unit: $(OUT)/libnvgpu-drv.so $(CORE_OBJS)
	$(CC) -shared -o $(OUT)/libnvgpu_unit.so	\
		$(CORE_OBJS) $(LIB_PATHS) $(LIBS)
	$(CC) --coverage	 			\
		-o $(OUT)/nvgpu_unit $(CORE_OBJS) $(LIB_PATHS) $(LIBS)

$(OUT)/libnvgpu-drv.so: $(OBJS)
	$(CC) -shared -o $(OUT)/libnvgpu-drv.so $(OBJS) -lgcov

# Default build target for all the nvgpu driver object files we want to build in
# userspace. These get bundled into libnvgpu-drv.so.
$(NVGPU_OUT)/%.o : $(NVGPU_SRC)/%.c $(HEADERS)
	@if [ ! -d $(dir $@) ] ; then \
		mkdir -p $(dir $@) ; \
	fi
	$(CC) --coverage $(CFLAGS) $(configs) -c -o $@ $<

# Default build target for all the nvgpu-next driver object files we want to
# build in userspace. These too get bundled into libnvgpu-drv.so.
$(NVGPU_OUT)/%.o : $(NVGPU_NEXT_SRC)/%.c $(HEADERS) $(HEADERS_NEXT)
	@if [ ! -d $(dir $@) ] ; then \
		mkdir -p $(dir $@) ; \
	fi
	$(CC) --coverage $(CFLAGS) $(configs) -c -o $@ $<

# Build target for unit test files. These are not part of the libnvgpu-drv.so.
# These comprise the unit test framework.
$(CORE_OUT)/%.o : $(CORE_SRC)/%.c $(CORE_HEADERS)
	@if [ ! -d $(dir $@) ] ; then \
		mkdir -p $(dir $@) ; \
	fi
	$(CC) --coverage $(CFLAGS) $(configs) -c -o $@ $<

# Certain variables should be exported to the unit test module builds.
export TWD INCLUDES CONFIGS UNIT_SRC UNIT_OUT
export CC CFLAGS LIB_PATHS LIBS

.PHONY: $(UNITS)
$(UNITS): $(OUT)/libnvgpu-drv.so
	@echo "Building unit module: $@"
	@+$(MAKE) --no-print-directory -C $@

.PHONY: clean nvgpu_clean core_clean unit_clean

clean: nvgpu_clean core_clean unit_clean
	rm -rf $(OUT)

nvgpu_clean:
	rm -rf $(OUT)/libnvgpu*

core_clean:
	rm -rf $(OUT)/nvgpu_unit*

unit_clean:
	@for d in $(UNITS); do					\
		echo Cleaning $$d;				\
		$(MAKE) --no-print-directory -C $$d clean;	\
	done
	rm -rf $(OUT)/units
