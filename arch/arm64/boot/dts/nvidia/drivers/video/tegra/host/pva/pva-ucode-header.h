/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PVA_UCODE_HEADER_H_
#define _PVA_UCODE_HEADER_H_

#include "pva-ucode-header-types.h"

#define MAX_SEGMENT_NAME_LEN	64

/*
 * PVA uCode Header.
 *
 * There is a basic header that describes the uCode.  Other than the
 * validation information (such as versions, checksums (MD5 hash?), etc)
 * it describes the various segments of the uCode image.  The important
 * thing to note is that there are multiple segments for various parts of
 * the uCode.
 *
 * Each segment has:
 *	- type: this indicates the type of segment it is.
 *	- id: this gives a uniqueness to the segment when there are multiple
 *	segments of the same type.  It also allows different segments types
 *	to be related by using the same segment ID (such as relating VPU code,
 *	R5 application code and parameter data together).
 *	- name: this is NUL terminated string that is the "name" of the segment
 *	- size: size of the segment in bytes
 *	- offset: this is the offset from the start of the binary as to
 *	where the data contained in the segment is to be placed.
 *	- address: this is the address of where the data in the segment is
 *	to be written to.
 *      - physical address: this is used in some segments to denote where in
 *      the 40-bit address space the segment is located.  This allows for
 *      setting up some of the segment registers.
 *
 * A segment can define a region but contain no data.  In those cases, the
 * file offset would be 0.
 *
 * In the case of DRAM the load address and size can be used to setup the
 * relevant segment registers and DRAM apertures.
 *
 */

/*
 * There can be multiple segments of the same type.
 */

/**
 * pva_ucode_seg - uCode segment struct
 *
 * @type:	Type of segment
 * @id:		ID of segment
 * @size:	Size of the segment
 * @offset:	Offset from header to segment start
 * @addr:	Load address of segment
 * @name:	Name of segment
 * @phys_addr:	Physical addr of the segment
 *
 */

struct pva_ucode_seg {
	uint32_t	type;
	uint32_t	id;
	uint32_t	size;
	uint32_t	offset;
	uint32_t	addr;
	uint8_t		name[MAX_SEGMENT_NAME_LEN];
	uint64_t	phys_addr	__attribute__((aligned(sizeof(uint64_t))));
};

/**
 * pva_ucode_hdr - uCode header struct
 *
 * @magic:		Module Magic number
 * @hdr_version:	Header version
 * @ucode_version:	Firmware version
 * @nsegments:		Number of segments
 *
 */

struct pva_ucode_hdr {
	uint32_t	magic;
	uint32_t	hdr_version;
	uint32_t	ucode_version;
	uint32_t	nsegments;
};


#endif
