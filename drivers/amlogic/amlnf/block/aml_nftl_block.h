#ifndef __AML_NFTL_BLOCK_H
#define __AML_NFTL_BLOCK_H

#include "aml_nftl_cfg.h"
#include <linux/device.h>
//#include <linux/mtd/blktrans.h>
#include "../ntd/aml_ntd.h"
#include "../include/amlnf_dev.h"

#define CFG_M2M_TRANSFER_TBL		(1)

#pragma pack(1)

#define AML_NFTL1_MAGIC			     "aml_nftlcode"
#define AML_NFTL2_MAGIC			     "aml_nftldata"

#define AML_NFTL_MAJOR			 250
#define TIMESTAMP_LENGTH         15
#define MAX_TIMESTAMP_NUM        ((1<<(TIMESTAMP_LENGTH-1))-1)
#define AML_NFTL_BOUNCE_SIZE	 		0x40000

#define NFTL_MAX_SCHEDULE_TIMEOUT		         1000
#define NFTL_FLUSH_DATA_TIME			         1
#define NFTL_CACHE_FORCE_WRITE_LEN               16

#define RET_YES                           1
#define RET_NO                            0

#define  PRINT aml_nftl_dbg

#define nftl_notifier_to_dev(l)	container_of(l, struct aml_nftl_dev, nb)

struct aml_nftl_part_t;
struct _ftl_status;

struct aml_nftl_blk;


struct aml_nftl_dev{
	struct device dev;
	uint64_t                size;
	struct ntd_info*        ntd;
	struct aml_nftl_part_t* aml_nftl_part;
	struct mutex*           aml_nftl_lock;
	struct task_struct*		nftl_thread;
	struct timespec      	ts_write_start;
	struct notifier_block   nb;
	struct class            debug;
	struct _nftl_cfg        nftl_cfg;
	int	sync_flag;
	int	init_flag;
	int	reboot_flag;
	int	thread_stop_flag;
	int fastboot_support;
	uint32 (*read_data)(struct aml_nftl_dev *nftl_dev, unsigned long block, unsigned nblk, unsigned char *buf);
	uint32 (*write_data)(struct aml_nftl_dev *nftl_dev, unsigned long block, unsigned nblk, unsigned char *buf);
	uint32 (*discard_data)(struct aml_nftl_dev *nftl_dev, unsigned long block, unsigned nblk);
	uint32 (*flush_write_cache)(struct aml_nftl_dev *nftl_dev);
	uint32 (*flush_discard_cache)(struct aml_nftl_dev *nftl_dev);
	uint32 (*invalid_read_cache)(struct aml_nftl_dev *nftl_dev);	//fixme, yyh 0311
	uint32 (*write_pair_page)(struct aml_nftl_dev *nftl_dev);
	int (*get_current_part_no)(struct aml_nftl_dev *nftl_dev);
	uint32 (*check_mapping)(struct aml_nftl_dev *nftl_dev,uint64_t offset, uint64_t size);
	uint32 (*discard_partition)(struct aml_nftl_dev *nftl_dev,uint64_t offset, uint64_t size);
	//TODO: rebuild tables.
	uint32 (*rebuild_tbls)(struct aml_nftl_dev *nftl_dev);
	uint32  (*compose_tbls)(struct aml_nftl_dev *nftl_dev);
};

struct aml_nftl_blk{
	struct ntd_blktrans_dev nbd;
	char                    name[24];
    uint64_t                offset;
	uint64_t                size;
    struct aml_nftl_dev*    nftl_dev;
	struct request*         req;
	struct request_queue*   queue;
	struct scatterlist*     bounce_sg;
	unsigned int		    bounce_sg_len;
	struct timespec      	ts_write_start;
	spinlock_t 				thread_lock;
	struct mutex           *lock;
	struct list_head		list;

	uint32 (*read_data)(struct aml_nftl_blk *nftl_blk, unsigned long block, unsigned nblk, unsigned char *buf);
	uint32 (*write_data)(struct aml_nftl_blk *nftl_blk, unsigned long block, unsigned nblk, unsigned char *buf);
    uint32 (*discard_data)(struct aml_nftl_blk *nftl_blk, unsigned long block, unsigned nblk);
	uint32 (*flush_write_cache)(struct aml_nftl_blk *nftl_blk);
};

#pragma pack()

static inline struct aml_nftl_dev *dev_to_nftl_dev(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

#endif
