/*
 * Aml nftl block device access
 *
 * (C) 2012 8
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mii.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/device.h>
#include <linux/pagemap.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/freezer.h>
#include <linux/spinlock.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/hdreg.h>
#include <linux/blkdev.h>
#include <linux/reboot.h>
#include <linux/kmod.h>
#include <linux/pm.h>
#include <linux/platform_device.h>

//#include <linux/mtd/mtd.h>
//#include <linux/mtd/nand.h>
//#include <linux/mtd/blktrans.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include "aml_nftl_block.h"

//extern struct mutex ntd_table_mutex;
//EXPORT_SYMBOL(ntd_table_mutex);
extern void print_block_invalid_list(struct aml_nftl_part_t* part);
extern int print_discard_page_map(struct aml_nftl_part_t *part);
extern int check_storage_device(void);
extern int is_phydev_off_adjust(void);
extern int aml_nftl_initialize(struct aml_nftl_dev *nftl_dev,int no);
extern uint32 nand_flush_write_cache(struct aml_nftl_blk* nftl_blk);
extern void aml_nftl_part_release(struct aml_nftl_part_t* part);
extern uint32 do_prio_gc(struct aml_nftl_part_t* part);
extern uint32 garbage_collect(struct aml_nftl_part_t* part);
extern uint32 do_static_wear_leveling(struct aml_nftl_part_t* part);
extern void *aml_nftl_malloc(uint32 size);
extern void aml_nftl_free(const void *ptr);
//extern int aml_nftl_dbg(const char * fmt,args...);
extern int aml_blktrans_initialize(struct aml_nftl_blk *nftl_blk,struct aml_nftl_dev *nftl_dev,uint64_t offset,uint64_t size);
extern void amlnf_ktime_get_ts(struct timespec *ts);
extern int aml_nftl_erase_part(struct aml_nftl_part_t *part);
extern int aml_nftl_set_status(struct aml_nftl_part_t *part,unsigned char status);
//static struct ntd_blktrans_dev *blktrans_dev_get_blk(struct gendisk *disk);
//static void blktrans_dev_put_blk(struct ntd_blktrans_dev *dev);
//int register_ntd_blktrans_blk(struct ntd_blktrans_ops *tr);
//int deregister_ntd_blktrans_blk(struct ntd_blktrans_ops *tr);
//int add_ntd_blktrans_dev_blk(struct ntd_blktrans_dev *new);
//int del_ntd_blktrans_dev_blk(struct ntd_blktrans_dev *old);
//static int blktrans_open_blk(struct block_device *bdev, fmode_t mode);
//static int blktrans_release_blk(struct gendisk *disk, fmode_t mode);
//static int blktrans_getgeo_blk(struct block_device *bdev, struct hd_geometry *geo);
//static int blktrans_ioctl_blk(struct block_device *bdev, fmode_t mode,unsigned int cmd, unsigned long arg);
//static void ntd_blktrans_request_blk(struct request_queue *rq);
//static int ntd_blktrans_thread_blk(void *arg);
//static void blktrans_dev_release_blk(struct kref *kref);

//static struct mutex aml_nftl_lock;
static int nftl_num;
static int dev_num;
extern int test_flag;

int aml_ntd_nftl_flush(struct ntd_info *ntd);




int get_adjust_block_num(void)
{
	int ret = 0;
	#ifdef NAND_ADJUST_PART_TABLE
		ret = ADJUST_BLOCK_NUM;
	#endif
	return	ret ;
}


/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int aml_nftl_flush(struct ntd_blktrans_dev *dev)
{
    int error = 0;
    struct aml_nftl_dev *nftl_dev = (struct aml_nftl_dev *)(dev->ntd->nftl_priv);

    mutex_lock(nftl_dev->aml_nftl_lock);
    error = nftl_dev->flush_write_cache(nftl_dev);
    mutex_unlock(nftl_dev->aml_nftl_lock);

//    PRINT("aml_nftl_flush\n");

    return error;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int aml_ntd_nftl_flush(struct ntd_info *ntd)
{
    int error = 0;
    struct aml_nftl_dev *nftl_dev = (struct aml_nftl_dev *)ntd->nftl_priv;

    mutex_lock(nftl_dev->aml_nftl_lock);
    error = nftl_dev->flush_write_cache(nftl_dev);
    mutex_unlock(nftl_dev->aml_nftl_lock);

//    PRINT("aml_ntd_flush\n");
    return error;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int aml_nftl_calculate_sg(struct aml_nftl_blk *nftl_blk, size_t buflen, unsigned **buf_addr, unsigned *offset_addr)
{
    struct scatterlist *sgl;
    unsigned int offset = 0, segments = 0, buf_start = 0;
    struct sg_mapping_iter miter;
    unsigned long flags;
    unsigned int nents;
    unsigned int sg_flags = SG_MITER_ATOMIC;

    nents = nftl_blk->bounce_sg_len;
    sgl = nftl_blk->bounce_sg;

    if (rq_data_dir(nftl_blk->req) == WRITE)
        sg_flags |= SG_MITER_FROM_SG;
    else
        sg_flags |= SG_MITER_TO_SG;

    sg_miter_start(&miter, sgl, nents, sg_flags);

    local_irq_save(flags);

    while (offset < buflen) {
        unsigned int len;
        if(!sg_miter_next(&miter))
            break;

        if (!buf_start) {
            segments = 0;
            *(buf_addr + segments) = (unsigned *)miter.addr;
            *(offset_addr + segments) = offset;
            buf_start = 1;
        }
        else {
            if ((unsigned char *)(*(buf_addr + segments)) + (offset - *(offset_addr + segments)) != miter.addr) {
                segments++;
                *(buf_addr + segments) = (unsigned *)miter.addr;
                *(offset_addr + segments) = offset;
            }
        }

        len = min(miter.length, buflen - offset);
        offset += len;
    }
    *(offset_addr + segments + 1) = offset;

    sg_miter_stop(&miter);

    local_irq_restore(flags);

    return segments;
}

uint32 write_sync_flag(struct aml_nftl_blk *aml_nftl_blk)
{ 
	struct aml_nftl_dev *nftl_dev = aml_nftl_blk->nftl_dev;
	if ( test_flag ) {
		return 0;
	}
#if NFTL_CACHE_FLUSH_SYNC   	
#ifdef CONFIG_SUPPORT_USB_BURNING
        return 0;
#endif

   	nftl_dev->sync_flag = 0;
	if(memcmp(aml_nftl_blk->name, "media", 5)==0)
		return 0;
	else {
		if ( aml_nftl_blk->req->cmd_flags & REQ_SYNC )
			nftl_dev->sync_flag = 1;
	}
	return 0;
#else
	return 0;
#endif
}
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :Alloc bounce buf for read/write numbers of pages in one request
*****************************************************************************/
int aml_nftl_init_bounce_buf(struct ntd_blktrans_dev *dev, struct request_queue *rq)
{
    int ret=0;
    unsigned int bouncesz;
    struct aml_nftl_blk *nftl_blk = (void *)dev;

    if(nftl_blk->queue && nftl_blk->bounce_sg)
    {
        aml_nftl_dbg("_nftl_init_bounce_buf already init %lx\n",PAGE_CACHE_SIZE);
        return 0;
    }
    nftl_blk->queue = rq;

    bouncesz = (nftl_blk->nftl_dev->ntd->pagesize * NFTL_CACHE_FORCE_WRITE_LEN);
    if(bouncesz < AML_NFTL_BOUNCE_SIZE)
        bouncesz = AML_NFTL_BOUNCE_SIZE;

    spin_lock_irq(rq->queue_lock);
    queue_flag_test_and_set(QUEUE_FLAG_NONROT, rq);
    blk_queue_bounce_limit(nftl_blk->queue, BLK_BOUNCE_HIGH);
    blk_queue_max_hw_sectors(nftl_blk->queue, bouncesz / BYTES_PER_SECTOR);
    blk_queue_physical_block_size(nftl_blk->queue, bouncesz);
    blk_queue_max_segments(nftl_blk->queue, bouncesz / PAGE_CACHE_SIZE);
    blk_queue_max_segment_size(nftl_blk->queue, bouncesz);
    spin_unlock_irq(rq->queue_lock);

    nftl_blk->req = NULL;
    nftl_blk->bounce_sg = aml_nftl_malloc(sizeof(struct scatterlist) * (bouncesz/PAGE_CACHE_SIZE));
    if (!nftl_blk->bounce_sg) {
        ret = -ENOMEM;
        blk_cleanup_queue(nftl_blk->queue);
        return ret;
    }

    sg_init_table(nftl_blk->bounce_sg, bouncesz / PAGE_CACHE_SIZE);

    return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int do_nftltrans_request(struct ntd_blktrans_ops *tr,struct ntd_blktrans_dev *dev,struct request *req)
{
	struct aml_nftl_blk *nftl_blk = (void *)dev;
	int ret = 0, segments, i;
	unsigned long block, nblk, blk_addr, blk_cnt;


	unsigned short max_segm = queue_max_segments(nftl_blk->queue);
	unsigned *buf_addr[max_segm+1];
	unsigned offset_addr[max_segm+1];
	size_t buflen;
	char *buf;

	if ( !nftl_blk->queue || !nftl_blk->bounce_sg )
	{
		if (aml_nftl_init_bounce_buf(&nftl_blk->nbd, nftl_blk->nbd.rq))
		{
			aml_nftl_dbg("_nftl_init_bounce_buf  failed\n");
		}
	}
	memset((unsigned char *)buf_addr, 0, (max_segm+1)*4);
	memset((unsigned char *)offset_addr, 0, (max_segm+1)*4);
	nftl_blk->req = req;
	block = blk_rq_pos(req) << SHIFT_PER_SECTOR >> tr->blkshift;
	nblk = blk_rq_sectors(req);
	buflen = (nblk << tr->blkshift);

	//if (!blk_fs_request(req))
	//	return -EIO;

	if (blk_rq_pos(req) + blk_rq_cur_sectors(req) > get_capacity(req->rq_disk))
		return -EIO;

	//if (blk_discard_rq(req))
	// return tr->discard(dev, block, nblk);


	//just return since notifier only need once
	if ( nftl_blk->nftl_dev->reboot_flag ) {
		PRINT("Just ignore nand req here after reboot nb block:0x%lx nblk:0x%lx, cmd_flags:0x%x nftl_blk:%s\n", \
			block, nblk, req->cmd_flags, nftl_blk->name);
		return 0;
	}
    	
	if (req->cmd_flags & REQ_DISCARD){
        mutex_lock(nftl_blk->nftl_dev->aml_nftl_lock);
		//printk("%s discard block:%d, nblk:%d nftl_blk->name:%s\n", __func__, block, nblk, nftl_blk->name);
		nftl_blk->discard_data(nftl_blk, block, nblk);
	    mutex_unlock(nftl_blk->nftl_dev->aml_nftl_lock);

		return 0;
	}
	

	nftl_blk->bounce_sg_len = blk_rq_map_sg(nftl_blk->queue, nftl_blk->req, nftl_blk->bounce_sg);
	segments = aml_nftl_calculate_sg(nftl_blk, buflen, buf_addr, offset_addr);
	if (offset_addr[segments+1] != (nblk << tr->blkshift))
		return -EIO;

	//aml_nftl_dbg("nftl segments: %d\n", segments+1);

	mutex_lock(nftl_blk->nftl_dev->aml_nftl_lock);
	switch ( rq_data_dir(req) ) {
        case READ:
	for (i=0; i<(segments+1); i++) {
		blk_addr = (block + (offset_addr[i] >> tr->blkshift));
		blk_cnt = ((offset_addr[i+1] - offset_addr[i]) >> tr->blkshift);
		buf = (char *)buf_addr[i];
		/*
		aml_nftl_dbg("read blk_addr: %d blk_cnt: %d buf: %x\n",
			blk_addr,blk_cnt,buf);
		*/
		if (nftl_blk->read_data(nftl_blk, blk_addr, blk_cnt, buf)) {
			ret = -EIO;
			break;
		}
	}
	bio_flush_dcache_pages(nftl_blk->req->bio);
	break;

        case WRITE:
	bio_flush_dcache_pages(nftl_blk->req->bio);
	nftl_blk->nftl_dev->sync_flag = 0;
	for (i=0; i<(segments+1); i++) {
		blk_addr = (block + (offset_addr[i] >> tr->blkshift));
		blk_cnt = ((offset_addr[i+1] - offset_addr[i]) >> tr->blkshift);
		buf = (char *)buf_addr[i];
		//aml_nftl_dbg("write blk_addr: %d blk_cnt: %d buf: %x\n", blk_addr,blk_cnt,buf);
		if (nftl_blk->write_data(nftl_blk, blk_addr, blk_cnt, buf)) {
			ret = -EIO;
			break;
		}
	}
	write_sync_flag(nftl_blk);
	if (nftl_blk->req->cmd_flags & REQ_SYNC)
		nftl_blk->flush_write_cache(nftl_blk);
	break;

        default:
	aml_nftl_dbg(KERN_NOTICE "Unknown request %u\n", rq_data_dir(req));
	break;
	}

	mutex_unlock(nftl_blk->nftl_dev->aml_nftl_lock);

	return ret;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int aml_nftl_writesect(struct ntd_blktrans_dev *dev, unsigned long block, char *buf)
{
    return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int aml_nftl_thread(void *arg)
{
	struct aml_nftl_dev *nftl_dev= arg;
	unsigned long period = NFTL_MAX_SCHEDULE_TIMEOUT / 10;
	struct timespec ts_nftl_current;

	while (!kthread_should_stop()) {
		//struct aml_nftl_part_t *aml_nftl_part=nftl_blk->aml_nftl_part;
		if (nftl_dev->thread_stop_flag == 1) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(period);
			continue;
		}

        mutex_lock(nftl_dev->aml_nftl_lock);

        if(aml_nftl_get_part_write_cache_nums(nftl_dev->aml_nftl_part) > 0){
            amlnf_ktime_get_ts(&ts_nftl_current);
            if ((ts_nftl_current.tv_sec - nftl_dev->ts_write_start.tv_sec) >= NFTL_FLUSH_DATA_TIME){
                //aml_nftl_dbg("aml_nftl_thread flush data: %d:%s\n", aml_nftl_part->cache.cache_write_nums,nftl_blk->nbd.ntd->name);
                nftl_dev->flush_write_cache(nftl_dev);
            }
        }

#if  SUPPORT_WEAR_LEVELING
        if(do_static_wear_leveling(nftl_dev->aml_nftl_part) != 0){
            PRINT("aml_nftl_thread do_static_wear_leveling error!\n");
        }
#endif

        if(garbage_collect(nftl_dev->aml_nftl_part) != 0){
            PRINT("aml_nftl_thread garbage_collect error!\n");
        }

        if(do_prio_gc(nftl_dev->aml_nftl_part) != 0){
            PRINT("aml_nftl_thread do_prio_gc error!\n");
        }

        mutex_unlock(nftl_dev->aml_nftl_lock);

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(period);
    }

    nftl_dev->nftl_thread=NULL;
    return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int aml_nftl_reboot_notifier(struct notifier_block *nb, unsigned long priority, void * arg)
{
    int error = 0;
    struct aml_nftl_dev *nftl_dev = nftl_notifier_to_dev(nb);

    //just return since notifier only need once
    if(nftl_dev->reboot_flag){
        PRINT("nand reboot notify Just ignore here for %s\n", nftl_dev->ntd->name);
        return error;
    }
    
    mutex_lock(nftl_dev->aml_nftl_lock);
    error = nftl_dev->flush_write_cache(nftl_dev);

    error |= nftl_dev->flush_discard_cache(nftl_dev);
    //print_block_invalid_list(nftl_dev->aml_nftl_part);
    //print_discard_page_map(nftl_dev->aml_nftl_part);

    mutex_unlock(nftl_dev->aml_nftl_lock);

    if(nftl_dev->nftl_thread!=NULL){
        kthread_stop(nftl_dev->nftl_thread); //add stop thread to ensure nftl quit safely
        nftl_dev->nftl_thread=NULL;
    }
    mutex_lock(nftl_dev->aml_nftl_lock);
    error |= nftl_dev->write_pair_page(nftl_dev);
    mutex_unlock(nftl_dev->aml_nftl_lock);
    nftl_dev->reboot_flag = 1;

    return error;
}

int aml_nftl_reinit_part(struct aml_nftl_blk *nftl_blk)
{
       struct aml_nftl_part_t *part = NULL;
       struct ntd_partition *logic_partition = NULL;
       int ret =0,i=0;
       uint64_t tmp_offset = 0;
       struct aml_nftl_dev * nftl_dev = nftl_blk->nftl_dev;
       part = nftl_dev->aml_nftl_part;
       
       aml_nftl_set_status(part,0);
       if(nftl_dev->nftl_thread!=NULL){
            //kthread_stop(nftl_dev->nftl_thread); //add stop thread to ensure nftl quit safely
            nftl_dev->thread_stop_flag = 1;
        }
      mutex_lock(nftl_dev->aml_nftl_lock);
        // do not erase this part because this phy partition has more than 1 logic partition. 
       if(nftl_dev->ntd->nr_partitions > 1)
       {
            // discard logic partition
            for(i=0;i<nftl_dev->ntd->nr_partitions;i++)
            {
                logic_partition = nftl_dev->ntd->parts+i;
                PRINT("show logic_partition name:%s,size:%llx,tmp_offset:%llx\n",logic_partition->name,logic_partition->size,tmp_offset);
                if(memcmp(logic_partition->name,nftl_blk->name, strlen(nftl_blk->name))==0)
                {
                    PRINT("logic_partition name:%s,size:%llx,tmp_offset:%llx\n",logic_partition->name,logic_partition->size,tmp_offset);
                    if((logic_partition->offset != 0xffffffffffffffff)&&(logic_partition->size != 0xffffffffffffffff))
                    {
                        // first check if this logic partition has valid mapping. 
                        if(nftl_dev->check_mapping(nftl_dev,tmp_offset,logic_partition->size))
                        {
                            //discard all pages of this partition
                            PRINT("this partition has valid mapping\n");
                            nftl_dev->discard_partition(nftl_dev,tmp_offset,logic_partition->size);
                        }else{
                            // do no thing
    						PRINT("this partition has no valid mapping\n");
                        }
                    }
                    else
                    {
                        //the last partition
                        PRINT("last logic partition name:%s,size:%llx,offset:%llx,tmp_offset:%llx\n",logic_partition->name,logic_partition->size,logic_partition->offset,tmp_offset);
                        // first check if this logic partition has valid mapping. 
                        if(nftl_dev->check_mapping(nftl_dev,tmp_offset,0xffffffffffffffff))
                        {
                            //discard all pages of this partition
                            PRINT("this partition has valid mapping\n");
                            nftl_dev->discard_partition(nftl_dev,tmp_offset,0xffffffffffffffff);
                        }else{
                            // do no thing
    						PRINT("this partition has no valid mapping\n");
                        }
                    }
                }
                
                if(logic_partition->size != 0xffffffffffffffff)
                {
                    tmp_offset = tmp_offset+logic_partition->size;
                }
            }
	} else {
		ret = aml_nftl_erase_part(part);
		if (ret)
			PRINT("aml_nftl_erase_part : failed\n");

		if (aml_nftl_initialize(nftl_dev, nftl_dev->get_current_part_no(nftl_dev)))
			PRINT("aml_nftl_reinit_part : aml_nftl_initialize failed\n");
	}
	mutex_unlock(nftl_dev->aml_nftl_lock);
	if (nftl_dev->nftl_thread != NULL) {
		//wake_up_process(nftl_dev->nftl_thread);
		nftl_dev->thread_stop_flag = 0;
	}

	return ret ;
}

static int aml_nftl_wipe_part(struct ntd_blktrans_dev *dev)
{
	struct aml_nftl_blk *nftl_blk = (void *)dev;
	int error = 0;
    //printk("%s,%d nftl_blk->name:%s,nftl_blk->offset:%llx,nftl_blk->size:%llx\n",__func__,__LINE__,nftl_blk->name,nftl_blk->offset,nftl_blk->size);
	//struct aml_nftl_dev * nftl_dev = nftl_blk->nftl_dev;
	
	error = aml_nftl_reinit_part(nftl_blk);
	if(error){
		PRINT("aml_nftl_reinit_part: failed\n");
	}
	return error;
}

int aml_nftl_reinit(struct aml_nftl_dev * nftl_dev)
{
	int ret =0;
	struct aml_nftl_part_t *part = NULL;

	part = nftl_dev->aml_nftl_part;

	//printk("%s() %d, enter\n", __FUNCTION__, __LINE__);
	aml_nftl_set_status(part, 0);
	if(nftl_dev->nftl_thread != NULL) {
		//kthread_stop(nftl_dev->nftl_thread); //add stop thread to ensure nftl quit safely
		nftl_dev->thread_stop_flag = 1;
	}
	
	//mutex_lock(nftl_dev->aml_nftl_lock);
	//invalid cache.
	nftl_dev->invalid_read_cache(nftl_dev);

#if	(CFG_M2M_TRANSFER_TBL)
	//reinit, using memory 1st
	nftl_dev->init_flag = 2;
	if (aml_nftl_initialize(nftl_dev, nftl_dev->get_current_part_no(nftl_dev))) {
#else 
	if (aml_nftl_initialize(nftl_dev, -1)) {
#endif //
		PRINT("%s() %d: aml_nftl_initialize failed\n", __FUNCTION__, __LINE__);
		ret = -1;
	}
	//mutex_unlock(nftl_dev->aml_nftl_lock);
	
	if (nftl_dev->nftl_thread != NULL) {
		//wake_up_process(nftl_dev->nftl_thread);
		nftl_dev->thread_stop_flag = 0;
	}
	
	//printk("%s() %d, exit\n", __FUNCTION__, __LINE__);
	return ret ;
}

#ifdef CONFIG_HIBERNATION

/*
pm(hibernate) ops
*/




static int aml_nftl_freeze(struct device *dev)
{
	struct aml_nftl_dev *nftl_dev = dev_to_nftl_dev(dev);
	//struct aml_nftl_blk *b;
	struct ntd_info* ntd;

	if (nftl_dev == NULL) {
		printk("%s : get nftl_dev failed\n", __func__);
		return 0;
	}

	ntd = nftl_dev->ntd;

	mutex_lock(nftl_dev->aml_nftl_lock);

	if (!strncmp(ntd->name, "nfdata", strlen(ntd->name))) {
		nftl_dev->thread_stop_flag = 1;
		ntd->thread_stop_flag = 1;
	}

	//second flush nand cache
	nftl_dev->flush_write_cache(nftl_dev);
	nftl_dev->flush_discard_cache(nftl_dev);
	nftl_dev->invalid_read_cache(nftl_dev);

	//fourth compose tbl and nand important data
	nftl_dev->compose_tbls(nftl_dev);

	mutex_unlock(nftl_dev->aml_nftl_lock);

	//printk("%s() %s\n", __FUNCTION__, nftl_dev->ntd->name);

	return 0;
}

static int aml_nftl_thaw(struct device *dev)
{
	struct aml_nftl_dev *nftl_dev = dev_to_nftl_dev(dev);
	struct ntd_info* ntd;

	if (nftl_dev == NULL) {
		printk("%s : get nftl_dev failed\n", __func__);
		return 0;
	}
	ntd = nftl_dev->ntd;

	mutex_lock(nftl_dev->aml_nftl_lock);

	if (!strncmp(ntd->name, "nfdata", strlen(ntd->name))) {
		nftl_dev->thread_stop_flag = 0;
		ntd->thread_stop_flag = 0;
	}

	//printk("thaw nftl_thread %p \n", nftl_dev->nftl_thread);
	mutex_unlock(nftl_dev->aml_nftl_lock);

	//printk("%s() %s\n", __FUNCTION__, nftl_dev->ntd->name);

	return 0;
}


static int aml_nftl_restore(struct device *dev)
{
	struct aml_nftl_dev *nftl_dev = dev_to_nftl_dev(dev);
	//struct aml_nftl_blk *b;
	struct ntd_info* ntd;

	if (nftl_dev == NULL) {
		printk("%s : get nftl_dev failed\n", __func__);
		return 0;
	}
	ntd = nftl_dev->ntd;

	mutex_lock(nftl_dev->aml_nftl_lock);

	nftl_dev->rebuild_tbls(nftl_dev);

	if (!strncmp(ntd->name, "nfdata", strlen(ntd->name))) {
		nftl_dev->thread_stop_flag = 0;
		ntd->thread_stop_flag = 0;
	}

	mutex_unlock(nftl_dev->aml_nftl_lock);


	//printk("%s() %s\n", __FUNCTION__, nftl_dev->ntd->name);

	return 0;
}

static struct dev_pm_ops nftl_pm_ops = {
	//.suspend = aml_nftl_suspend,
	//.resume  = aml_nftl_resume,
	.freeze  = aml_nftl_freeze,
	.thaw    = aml_nftl_thaw,
	.restore = aml_nftl_restore,
};


static struct class aml_nftl_class = {
	.name = "nftl_cls",
	.owner = THIS_MODULE,
	.pm = &nftl_pm_ops,
};
#endif

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static void aml_nftl_add_ntd(struct ntd_blktrans_ops *tr, struct ntd_info *ntd)
{
    int i;
    struct aml_nftl_dev *nftl_dev;
    struct aml_nftl_blk *nftl_blk;
    uint64_t cur_offset = 0;
    //uint64_t cur_size;
    struct ntd_partition *part;

    PRINT("ntd->name: %s\n",ntd->name);

    nftl_dev = aml_nftl_malloc(sizeof(struct aml_nftl_dev));
    if (!nftl_dev)
        return;

    nftl_dev->thread_stop_flag = 0;
	nftl_dev->aml_nftl_lock = aml_nftl_malloc(sizeof(struct mutex));
	if (!nftl_dev->aml_nftl_lock)
	       return;

	mutex_init(nftl_dev->aml_nftl_lock);
    
    nftl_dev->init_flag = 0;
    nftl_dev->ntd = ntd;
    nftl_dev->nb.notifier_call = aml_nftl_reboot_notifier;
    register_reboot_notifier(&nftl_dev->nb);

    if (aml_nftl_initialize(nftl_dev,nftl_num)){
        aml_nftl_dbg("aml_nftl_initialize failed\n");
        return;
    }
    nftl_dev->init_flag = 1;
    nftl_dev->reboot_flag = 0;
    ntd->nftl_priv = (void*)nftl_dev;

    nftl_dev->nftl_thread = kthread_run(aml_nftl_thread, nftl_dev, "%s_%s", "aml_nftl", ntd->name);
    if (IS_ERR(nftl_dev->nftl_thread))
        return;

    for(i=0;i<ntd->nr_partitions;i++)
    {
        part = ntd->parts+i;

        nftl_blk = aml_nftl_malloc(sizeof(struct aml_nftl_blk));
        if (!nftl_blk)
            return;

		//nftl_blk->nbd.ntd = ntd;
		//nftl_blk->nbd.devnum = (ntd->index<<2)+i;

        nftl_blk->nbd.devnum = dev_num;
        dev_num++;
        nftl_blk->nbd.tr = tr;
        nftl_blk->nbd.ntd = ntd;

        snprintf(nftl_blk->name, sizeof(nftl_blk->name),"%s", part->name);

        if(aml_blktrans_initialize(nftl_blk,nftl_dev,cur_offset,part->size)){
            aml_nftl_dbg("aml_blktrans_initialize failed\n");
            return;
        }

		//printk("nftl_blk->name %s \n",nftl_blk->name);
		//printk("nftl_blk->offset 0x%llx \n",nftl_blk->offset);
		//printk("nftl_blk->size 0x%llx \n",nftl_blk->size);

        nftl_blk->nbd.size = (unsigned long)nftl_blk->size;
        nftl_blk->nbd.priv = (void*)nftl_blk;

        memcpy(nftl_blk->nbd.name,part->name,strlen(part->name)+1);

        if (add_ntd_blktrans_dev(&nftl_blk->nbd)){
            aml_nftl_dbg("nftl add blk disk dev failed\n");
            return;
        }
        if (aml_nftl_init_bounce_buf(&nftl_blk->nbd, nftl_blk->nbd.rq)){
            aml_nftl_dbg("aml_nftl_init_bounce_buf  failed\n");
            return;
        }

        cur_offset += part->size;
    }

#ifdef CONFIG_HIBERNATION
	if (nftl_num == 0)
		blk_class_register(&aml_nftl_class);
    
	//nftl_dev->dev.devt = NTD_DEVT(i);
	nftl_dev->dev.class = &aml_nftl_class;
	dev_set_drvdata(&nftl_dev->dev, nftl_dev);
	blk_device_register(&nftl_dev->dev, nftl_num);
#endif

	nftl_num++;
	aml_nftl_dbg("aml_nftl_add_ntd ok\n");

	return;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int aml_nftl_open(struct ntd_blktrans_dev *nbd)
{
    aml_nftl_dbg("aml_nftl_open ok!\n");
    return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int aml_nftl_release(struct ntd_blktrans_dev *nbd)
{
    int error = 0;
    struct aml_nftl_blk *nftl_blk = (void *)nbd;

    mutex_lock(nftl_blk->lock);

    error = nftl_blk->flush_write_cache(nftl_blk);

    mutex_unlock(nftl_blk->lock);

    return error;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static void aml_nftl_blk_release(struct aml_nftl_blk *nftl_blk)
{
//    aml_nftl_part_release(nftl_blk->nftl_dev->aml_nftl_part);
    if (nftl_blk->bounce_sg)
        aml_nftl_free(nftl_blk->bounce_sg);
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static void aml_nftl_remove_dev(struct ntd_blktrans_dev *dev)
{
    struct aml_nftl_blk *nftl_blk = (void *)dev;

    unregister_reboot_notifier(&nftl_blk->nftl_dev->nb);
    del_ntd_blktrans_dev(dev);
    aml_nftl_blk_release(nftl_blk);
    aml_nftl_free(nftl_blk);
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static struct ntd_blktrans_ops aml_nftl_tr = {
    .name       = "avnftl",
    .major      = AML_NFTL_MAJOR,
    .part_bits  = 0,
    .blksize    = BYTES_PER_SECTOR,
    .open       = aml_nftl_open,
    .release    = aml_nftl_release,
    .do_blktrans_request = do_nftltrans_request,
    .writesect  = aml_nftl_writesect,
    .flush      = aml_nftl_flush,
    .add_ntd    = aml_nftl_add_ntd,
    .remove_dev = aml_nftl_remove_dev,
    .wipe_part	= aml_nftl_wipe_part,
    .owner      = THIS_MODULE,
};

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int __init init_aml_nftl(void)
{
    int ret;

//    aml_nftl_lock = aml_nftl_malloc(sizeof(struct mutex));
//    if (!aml_nftl_lock)
//        return -1;

    //mutex_init(&aml_nftl_lock);
        if(check_storage_device() < 0){
		return 0;
     }
    nftl_num = 0;
    dev_num = 0;
    ret = register_ntd_blktrans(&aml_nftl_tr);
    PRINT("init_aml_nftl end\n");

    return ret;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static void __exit cleanup_aml_nftl(void)
{
    deregister_ntd_blktrans(&aml_nftl_tr);
}




module_init(init_aml_nftl);
module_exit(cleanup_aml_nftl);


MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("AML nand team");
MODULE_DESCRIPTION("aml nftl block interface");

