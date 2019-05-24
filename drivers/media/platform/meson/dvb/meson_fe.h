// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2018-present Team CoreELEC (https://coreelec.org)

 
#ifndef __MESON_FE_H
#define __MESON_FE__H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dvb/version.h>
#include <linux/platform_device.h>
#include "media/dvb_frontend.h"
#include "aml_dvb.h"

#define TOTAL_AML_INPUTS 	2
#define BASE_IRQ 32
#define AM_IRQ(reg)             (reg + BASE_IRQ)
#define INT_DEMUX               AM_IRQ(23)
#define INT_DEMUX_1             AM_IRQ(5)
#define INT_DEMUX_2             AM_IRQ(53)
#define INT_ASYNC_FIFO_FILL     AM_IRQ(18)
#define INT_ASYNC_FIFO_FLUSH    AM_IRQ(19)
#define INT_ASYNC_FIFO2_FILL    AM_IRQ(24)
#define INT_ASYNC_FIFO2_FLUSH   AM_IRQ(25)


void get_aml_dvb(struct aml_dvb *p);
int set_external_vol_gpio(int *demod_id, int on);

#endif /* __MESON_FE__H */
