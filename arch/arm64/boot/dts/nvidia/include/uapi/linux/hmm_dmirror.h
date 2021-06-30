/*
 * Copyright 2013 Red Hat Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Authors: Jérôme Glisse <jglisse@redhat.com>
 */
/*
 * This is a dummy driver to exercise the HMM (heterogeneous memory management)
 * API of the kernel. It allows a userspace program to expose its entire address
 * space through the HMM dummy driver file.
 */
#ifndef _UAPI_LINUX_HMM_DMIRROR_H
#define _UAPI_LINUX_HMM_DMIRROR_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/irqnr.h>

struct hmm_dmirror_read {
	uint64_t		addr;
	uint64_t		ptr;
	uint64_t		npages;
	uint64_t		cpages;
	uint64_t		dpages;
};

struct hmm_dmirror_write {
	uint64_t		addr;
	uint64_t		ptr;
	uint64_t		npages;
	uint64_t		cpages;
	uint64_t		dpages;
};

struct hmm_dmirror_migrate {
	uint64_t		addr;
	uint64_t		npages;
};

/* Expose the address space of the calling process through hmm dummy dev file */
#define HMM_DMIRROR_READ		_IOWR('H', 0x00, struct hmm_dmirror_read)
#define HMM_DMIRROR_WRITE		_IOWR('H', 0x01, struct hmm_dmirror_write)
#define HMM_DMIRROR_MIGRATE		_IOWR('H', 0x02, struct hmm_dmirror_migrate)

#endif /* _UAPI_LINUX_HMM_DMIRROR_H */
