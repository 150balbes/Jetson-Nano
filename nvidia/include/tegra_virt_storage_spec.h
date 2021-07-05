/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TEGRA_VIRT_STORAGE_SPEC_H_
#define _TEGRA_VIRT_STORAGE_SPEC_H_

#include <linux/types.h> /* size_t */

#define VS_REQ_OP_F_NONE      0

enum vs_req_type {
	VS_DATA_REQ = 1,
	VS_CONFIGINFO_REQ = 2,
	VS_UNKNOWN_CMD = 0xffffffff,
};

enum vs_dev_type {
	VS_BLK_DEV = 1,
	VS_MTD_DEV = 2,
	VS_UNKNOWN_DEV = 0xffffffff,
};

enum mtd_cmd_op {
	VS_MTD_READ = 1,
	VS_MTD_WRITE = 2,
	VS_MTD_ERASE = 3,
	VS_MTD_IOCTL = 4,
	VS_MTD_INVAL_REQ = 32,
	VS_UNKNOWN_MTD_CMD = 0xffffffff,
};

/* MTD device request Operation type features supported */
#define VS_MTD_READ_OP_F          (1 << VS_MTD_READ)
#define VS_MTD_WRITE_OP_F         (1 << VS_MTD_WRITE)
#define VS_MTD_ERASE_OP_F         (1 << VS_MTD_ERASE)
#define VS_MTD_IOCTL_OP_F         (1 << VS_MTD_IOCTL)

enum blk_cmd_op {
	VS_BLK_READ = 1,
	VS_BLK_WRITE = 2,
	VS_BLK_FLUSH = 3,
	VS_BLK_IOCTL = 4,
	VS_BLK_INVAL_REQ = 32,
	VS_UNKNOWN_BLK_CMD = 0xffffffff,
};

/* Blk device request Operation type features supported */
#define VS_BLK_READ_OP_F          (1 << VS_BLK_READ)
#define VS_BLK_WRITE_OP_F         (1 << VS_BLK_WRITE)
#define VS_BLK_FLUSH_OP_F         (1 << VS_BLK_FLUSH)
#define VS_BLK_IOCTL_OP_F         (1 << VS_BLK_IOCTL)

#pragma pack(push)
#pragma pack(1)

struct vs_blk_request {
	uint64_t blk_offset;		/* Offset into storage device in terms
						of blocks for block device */
	uint32_t num_blks;		/* Total Block number to transfer */
	uint32_t data_offset;		/* Offset into mempool for data region
						*/
};

struct vs_mtd_request {
	uint64_t offset;		/* Offset into storage device in terms
						of bytes in case of mtd device */
	uint32_t size;			/* Total number of bytes to transfer
						  to be used for MTD device */
	uint32_t data_offset;		/* Offset into mempool for data region
						*/
};

struct vs_ioctl_request {
	uint32_t ioctl_id;		/* Id of the ioctl */
	uint32_t ioctl_len;		/* Length of the mempool area associated
						with ioctl */
	uint32_t data_offset;		/* Offset into mempool for data region
						*/
};

struct vs_blkdev_request {
	enum blk_cmd_op req_op;
	union {
		struct vs_blk_request blk_req;
		struct vs_ioctl_request ioctl_req;
	};
};

struct vs_mtddev_request {
	enum mtd_cmd_op req_op;
	union {
		struct vs_mtd_request mtd_req;
		struct vs_ioctl_request ioctl_req;
	};
};

struct vs_blk_response {
	int32_t status;			/* 0 for success, < 0 for error */
	uint32_t num_blks;
};

struct vs_mtd_response {
	int32_t status;			/* 0 for success, < 0 for error */
	uint32_t size;			/* Number of bytes processed in case of
						of mtd device*/
};

struct vs_ioctl_response {
	int32_t status;			/* 0 for success, < 0 for error */
};

struct vs_blkdev_response {
	union {
		struct vs_blk_response blk_resp;
		struct vs_ioctl_response ioctl_resp;
	};
};

struct vs_mtddev_response {
	union {
		struct vs_mtd_response mtd_resp;
		struct vs_ioctl_response ioctl_resp;
	};
};

struct vs_blk_dev_config {
	uint32_t hardblk_size;		/* Block Size */
	uint32_t max_read_blks_per_io;	/* Limit number of Blocks
						per I/O*/
	uint32_t max_write_blks_per_io; /* Limit number of Blocks
					   per I/O*/
	uint32_t req_ops_supported;	/* Allowed operations by requests */
	uint64_t num_blks;		/* Total number of blks */
};

struct vs_mtd_dev_config {
	uint32_t max_read_bytes_per_io;	/* Limit number of bytes
						per I/O */
	uint32_t max_write_bytes_per_io; /* Limit number of bytes
					   per I/O */
	uint32_t erase_size;		/* Erase size for mtd
					   device*/
	uint32_t req_ops_supported;	/* Allowed operations by requests */
	uint64_t size;			/* Total number of bytes */
};

struct vs_config_info {
	uint32_t virtual_storage_ver;		/* Version of virtual storage */
	enum vs_dev_type type;			/* Type of underlying device */
	union {
		struct vs_blk_dev_config blk_config;
		struct vs_mtd_dev_config mtd_config;
	};
};

struct vs_request {
	uint32_t req_id;
	enum vs_req_type type;
	union {
		struct vs_blkdev_request blkdev_req;
		struct vs_mtddev_request mtddev_req;
	};
	int32_t status;
	union {
		struct vs_blkdev_response blkdev_resp;
		struct vs_mtddev_response mtddev_resp;
		struct vs_config_info config_info;
	};
};

/* Defines Command Responses of Emmc/Esd as per VSC interface */
typedef enum {
	RESP_TYPE_NO_RESP = 0,
	RESP_TYPE_R1 = 1,
	RESP_TYPE_R2 = 2,
	RESP_TYPE_R3 = 3,
	RESP_TYPE_R4 = 4,
	RESP_TYPE_R5 = 5,
	RESP_TYPE_R6 = 6,
	RESP_TYPE_R7 = 7,
	RESP_TYPE_R1B = 8,
	RESP_TYPE_NUM,
} sdmmc_resp_type;


#define VBLK_MMC_MULTI_IOC_ID 0x1000
struct combo_cmd_t {
	uint32_t cmd;
	uint32_t arg;
	uint32_t response[4];
	uint32_t buf_offset;
	uint32_t data_len;
	uint32_t write_flag;
	uint32_t flags;
};

struct combo_info_t {
	uint32_t count;
	int32_t  result;
};

/* SCSI bio layer needs to handle SCSI and UFS IOCTL separately
 * This flag will be ORed with IO_IOCTL to find out difference
 * between SCSI and UFS IOCTL
 */
#define SCSI_IOCTL_FLAG	0x10000000
#define UFS_IOCTL_FLAG	0x20000000
/* Mask for SCSI and UFS ioctl flags, 4 MSB (bits) reserved for it Two LSB
 * bits are used for SCSI and UFS, 2 MSB bits reserved for future use.
 */
#define SCSI_UFS_IOCTL_FLAG_MASK 0xF0000000

#define VBLK_SG_IO_ID	(0x1001 | SCSI_IOCTL_FLAG)
#define VBLK_UFS_IO_ID	(0x1002 | UFS_IOCTL_FLAG)

#define VBLK_SG_MAX_CMD_LEN 16

enum scsi_data_direction {
        SCSI_BIDIRECTIONAL = 0,
        SCSI_TO_DEVICE = 1,
        SCSI_FROM_DEVICE = 2,
        SCSI_DATA_NONE = 3,
	UNKNOWN_DIRECTION = 0xffffffff,
};

struct vblk_sg_io_hdr
{
    int32_t data_direction;     /* [i] data transfer direction  */
    uint8_t cmd_len;            /* [i] SCSI command length */
    uint8_t mx_sb_len;          /* [i] max length to write to sbp */
    uint32_t dxfer_len;         /* [i] byte count of data transfer */
    uint32_t xfer_arg_offset;  /* [i], [*io] offset to data transfer memory */
    uint32_t cmdp_arg_offset;   /* [i], [*i] offset to command to perform */
    uint32_t sbp_arg_offset;    /* [i], [*o] offset to sense_buffer memory */
    uint32_t status;            /* [o] scsi status */
    uint8_t sb_len_wr;          /* [o] byte count actually written to sbp */
    uint32_t dxfer_buf_len;     /* [i] Length of data transfer buffer */
};

struct vblk_ufs_ioc_query_req {
	/* Query opcode to specify the type of Query operation */
	uint8_t opcode;
	/* idn to provide more info on specific operation. */
	uint8_t idn;
	/* index - optional in some cases */
	uint8_t index;
	/* index - optional in some cases */
	uint8_t selector;
	/* buf_size - buffer size in bytes pointed by buffer. */
	uint16_t buf_size;
	/*
	 * user buffer pointer for query data.
	 * Note:
	 * For Read/Write Attribute this should be of 4 bytes
	 * For Read Flag this should be of 1 byte
	 * For Descriptor Read/Write size depends on the type of the descriptor
	 */
	uint8_t *buffer;
	/* delay after query command completion */
	uint32_t delay;
	/* error status for the query operation */
	int32_t error_status;

};

#pragma pack(pop)

#endif
