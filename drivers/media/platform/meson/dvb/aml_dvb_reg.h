// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2018-present Team CoreELEC (https://coreelec.org)

#ifndef _DVB_REG_H_
#define _DVB_REG_H_

extern int aml_read_cbus(unsigned int reg);
extern void aml_write_cbus(unsigned int reg, unsigned int val);
extern int aml_read_vcbus(unsigned int reg);
extern void aml_write_vcbus(unsigned int reg, unsigned int val);

#define WRITE_MPEG_REG(_r, _v)   aml_write_cbus(_r, _v)
#define READ_MPEG_REG(_r)        aml_read_cbus(_r)

#define WRITE_CBUS_REG(_r, _v)   aml_write_cbus(_r, _v)
#define READ_CBUS_REG(_r)        aml_read_cbus(_r)

#define WRITE_VCBUS_REG(_r, _v)  aml_write_vcbus(_r, _v)
#define READ_VCBUS_REG(_r)       aml_read_vcbus(_r)

#endif

