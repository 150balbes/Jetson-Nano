/*
 * general thermal device structures & definitions
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_THERM_THRMCHANNEL_H
#define NVGPU_THERM_THRMCHANNEL_H

#include "boardobj/boardobj.h"
#include "boardobj/boardobjgrp.h"
#include "ctrl/ctrltherm.h"

struct therm_channel {
	struct boardobj super;
	s16 scaling;
	s16 offset;
	s32 temp_min;
	s32 temp_max;
};

struct therm_channels {
	struct boardobjgrp_e32 super;
};

struct therm_channel_device {
	struct therm_channel super;
	u8 therm_dev_idx;
	u8 therm_dev_prov_idx;
};

int therm_channel_sw_setup(struct gk20a *g);

#endif /* NVGPU_THERM_THRMCHANNEL_H */
