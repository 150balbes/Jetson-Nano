#!/bin/bash

# Copyright (c) 2018, NVIDIA CORPORATION.  All Rights Reserved.

#
# Execute the unit test. Args to this script are passed on to the unit test
# core. This just serves to set the LD_LIBRARY_PATH environment variable such
# that unit tests are found and nvgpu-drv is found.
#

if [ -f nvgpu_unit ]; then
        # if the executable is in the current directory, we are running on
        # target, so use that dir structure
        LD_LIBRARY_PATH=".:units"
        NVGPU_UNIT="./nvgpu_unit -L units/"
else
        # running on host
        LD_LIBRARY_PATH="build:build/units"
        NVGPU_UNIT=build/nvgpu_unit
fi
export LD_LIBRARY_PATH

echo "$ $NVGPU_UNIT $*"

$NVGPU_UNIT $*
