/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

struct ras_error {
	char *name;
	u16 error_code;
};

struct error_record {
	struct list_head node;
	char *name;
	u64 errx;
	u8 processed;
	u64 err_ctrl;
	struct ras_error *errors;
};

#define RAS_BIT(_bit_) (1ULL << (_bit_))
#define RAS_MASK(_msb_, _lsb_) \
	((RAS_BIT(_msb_+1) - 1ULL) & ~(RAS_BIT(_lsb_) - 1ULL))
#define RAS_EXTRACT(_x_, _msb_, _lsb_)	\
	((_x_ & RAS_MASK(_msb_, _lsb_)) >> _lsb_)

#define RAS_CTL_CFI		RAS_BIT(8)
#define RAS_CTL_UE		RAS_BIT(4)
#define RAS_CTL_ED		RAS_BIT(0)

#define ERRi_STATUS_UET		((RAS_BIT(20)) | (RAS_BIT(21)))
#define ERRi_STATUS_CE		((RAS_BIT(24)) | (RAS_BIT(25)))
#define ERRi_STATUS_MV		RAS_BIT(26)
#define ERRi_STATUS_OF		RAS_BIT(27)
#define ERRi_STATUS_UE		RAS_BIT(29)
#define ERRi_STATUS_VALID	RAS_BIT(30)
#define ERRi_STATUS_AV		RAS_BIT(31)

#define ERRi_PFGCTL_CDNEN	RAS_BIT(31)
#define ERRi_PFGCTL_R		RAS_BIT(30)
#define ERRi_PFGCTL_CE		RAS_BIT(6)
#define ERRi_PFGCTL_UC		RAS_BIT(1)

#define ERRi_PFGCDN_CDN_1	0x1

#define get_error_status_ce(_x_) RAS_EXTRACT(_x_, 25, 24)
#define get_error_status_ierr(_x_) RAS_EXTRACT(_x_, 15, 8)
#define get_error_status_serr(_x_) RAS_EXTRACT(_x_, 7, 0)

struct ras_fhi_callback {
	struct list_head node;
	void (*fn)(void);
};

/* Macros for reading ID_PFR0 - RAS Version field */
#define PFR0_RAS_SHIFT 28
#define PFR0_RAS_MASK  (0xf << PFR0_RAS_SHIFT)
#define PFR0_RAS(pfr0) \
	(((pfr0) & PFR0_RAS_MASK) >> PFR0_RAS_SHIFT)
#define PFR0_RAS_VERSION_1	0x1

/* RAS functions needed by ras_carmel driver */
int is_ras_ready(void);
int is_this_ras_cpu(void);
int is_ras_cpu(int cpu);
u64 ras_read_error_status(void);
u64 ras_read_errselr(void);
u64 ras_read_pfg_control(void);
u64 ras_read_pfg_cdn(void);
u64 ras_read_error_control(void);
void ras_write_error_control(u64 err_ctl);
void ras_write_error_status(u64 status);
void ras_write_error_addr(u64 addr);
void ras_write_error_misc0(u64 misc0);
void ras_write_error_misc1(u64 misc1);
void ras_write_error_statustrigger(u64 status);
void ras_write_pfg_control(u64 pfg_ctl);
void ras_write_pfg_cdn(u64 pfg_cdn);
void ras_write_errselr(u64 errx);
void print_error_record(struct error_record *record, u64 status, int errselr);
int register_fhi_callback(struct ras_fhi_callback *callback, void *cookie);
void unregister_fhi_callback(struct ras_fhi_callback *callback);

