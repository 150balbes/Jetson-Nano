
/*
 * Aml nftl init
 *
 * (C) 2012 8
 */


//#include <linux/mtd/mtd.h>
//#include <linux/mtd/blktrans.h>

#include "aml_nftl_block.h"
extern int print_discard_page_map(struct aml_nftl_part_t *part);
extern int is_phydev_off_adjust(void);
extern int amlnf_class_register(struct class* cls);
extern void amlnf_ktime_get_ts(struct timespec *ts);
extern int aml_nftl_start(void* priv,void* cfg,struct aml_nftl_part_t ** ppart,uint64_t size,unsigned erasesize,unsigned writesize,unsigned oobavail,char* name,int no,char type,int init_flag);
extern uint32 gc_all(struct aml_nftl_part_t* part);
extern uint32 gc_one(struct aml_nftl_part_t* part);
extern void print_nftl_part(struct aml_nftl_part_t * part);
extern int part_param_init(struct aml_nftl_part_t *part,uint16 start_block,uint32_t logic_sects,uint32_t backup_cap_in_sects,int init_flag);
extern uint32 is_no_use_device(struct aml_nftl_part_t * part,uint32 size);
extern uint32 create_part_list_first(struct aml_nftl_part_t * part,uint32 size);
extern uint32 create_part_list(struct aml_nftl_part_t * part);
//extern int nand_test(struct aml_nftl_dev *nftl_dev,unsigned char flag,uint32 blocks);
extern int part_param_exit(struct aml_nftl_part_t *part);
extern int cache_init(struct aml_nftl_part_t *part);
extern int cache_exit(struct aml_nftl_part_t *part);
extern uint32 get_vaild_blocks(struct aml_nftl_part_t * part,uint32 start_block,uint32 blocks);
extern uint32 __nand_read(struct aml_nftl_part_t* part,uint32 start_sector,uint32 len,unsigned char *buf);
extern uint32 __nand_write(struct aml_nftl_part_t* part,uint32 start_sector,uint32 len,unsigned char *buf,int sync_flag);
extern uint32 __nand_discard(struct aml_nftl_part_t* part,uint32 start_sector,uint32 len,int sync_flag);
extern uint32 __nand_flush_write_cache(struct aml_nftl_part_t* part);
extern uint32 __nand_invalid_read_cache(struct aml_nftl_part_t* part);
extern uint32 __nand_flush_discard_cache(struct aml_nftl_part_t* part);
extern uint32 __nand_write_pair_page(struct aml_nftl_part_t* part);
extern int __get_current_part_no(struct aml_nftl_part_t* part);
extern uint32 __check_mapping(struct aml_nftl_part_t* part,uint64_t offset,uint64_t size);
extern uint32 __discard_partition(struct aml_nftl_part_t* part,uint64_t offset,uint64_t size);
extern void print_free_list(struct aml_nftl_part_t* part);
extern void print_block_invalid_list(struct aml_nftl_part_t* part);
extern int nand_discard_logic_page(struct aml_nftl_part_t* part,uint32 page_no);
extern  int get_adjust_block_num(void);
extern int aml_nftl_erase_part(struct aml_nftl_part_t *part);
extern int aml_nftl_set_status(struct aml_nftl_part_t *part,unsigned char status);
extern int aml_nftl_reinit(struct aml_nftl_dev * nftl_dev);
uint32 _nand_read(struct aml_nftl_dev *nftl_dev,unsigned long start_sector,unsigned len,unsigned char *buf);
uint32 _nand_write(struct aml_nftl_dev *nftl_dev,unsigned long  start_sector,unsigned len,unsigned char *buf);
uint32 _nand_discard(struct aml_nftl_dev *nftl_dev,unsigned long start_sector,unsigned len);
uint32 _nand_flush_write_cache(struct aml_nftl_dev *nftl_dev);
uint32 _nand_invalid_read_cache(struct aml_nftl_dev *nftl_dev);
uint32 _nand_flush_discard_cache(struct aml_nftl_dev *nftl_dev);
uint32 _nand_write_pair_page(struct aml_nftl_dev *nftl_dev);
int _get_current_part_no(struct aml_nftl_dev *nftl_dev);

uint32 _check_mapping(struct aml_nftl_dev *nftl_dev,uint64_t offset,uint64_t size);
uint32 _discard_partition(struct aml_nftl_dev *nftl_dev,uint64_t offset,uint64_t size);
uint32 _blk_nand_flush_write_cache(struct aml_nftl_blk *nftl_blk);
uint32 _blk_nand_write(struct aml_nftl_blk *nftl_blk,unsigned long start_sector,unsigned  len,unsigned char *buf);
uint32 _blk_nand_discard(struct aml_nftl_blk *nftl_blk,unsigned long start_sector,unsigned len);
uint32 _blk_nand_read(struct aml_nftl_blk *nftl_blk,unsigned long start_sector,unsigned len,unsigned char *buf);
uint32 _rebuild_tbls(struct aml_nftl_dev *nftl_dev);
uint32 _compose_tbls(struct aml_nftl_dev *nftl_dev);

void *aml_nftl_malloc(uint32 size);
void aml_nftl_free(const void *ptr);
//int aml_nftl_dbg(const char * fmt,args...);

static ssize_t show_part_struct(struct class *class,struct class_attribute *attr, char *buf);
static ssize_t show_list(struct class *class, struct class_attribute *attr,	const char *buf, size_t count);
//static ssize_t discard_page(struct class *class, struct class_attribute *attr, const char *buf);
static ssize_t do_gc_all(struct class *class, struct class_attribute *attr,	const char *buf, size_t count);
static ssize_t do_gc_one(struct class *class, struct class_attribute *attr,	const char *buf, size_t count);
static ssize_t do_test(struct class *class, struct class_attribute *attr,	const char *buf, size_t count);

static struct class_attribute nftl_class_attrs[] = {
//    __ATTR(part_struct,  S_IRUGO | S_IWUSR, show_logic_block_table,    show_address_map_table),
    __ATTR(part,  S_IRUGO , show_part_struct,    NULL),
    __ATTR(list,  S_IRUGO | S_IWUSR, show_part_struct,  show_list),
//    __ATTR(discard,  S_IRUGO | S_IWUSR , NULL,    discard_page),
    __ATTR(gcall,  S_IRUGO , NULL,    do_gc_all),
    __ATTR(gcone,  S_IRUGO , NULL,    do_gc_one),
    __ATTR(test,  S_IRUGO | S_IWUSR , NULL,    do_test),
//    __ATTR(cache_struct,  S_IRUGO , show_logic_block_table,    NULL),
//    __ATTR(table,  S_IRUGO | S_IWUSR , NULL,    show_logic_page_table),

    __ATTR_NULL
};

int aml_nftl_initialize(struct aml_nftl_dev *nftl_dev,int no);


/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
void *aml_nftl_malloc(uint32 size)
{
    return kzalloc(size, GFP_KERNEL);
}

void aml_nftl_free(const void *ptr)
{
    kfree(ptr);
}

//int aml_nftl_dbg(const char * fmt,args...)
//{
//    //return printk(fmt,##__VA_ARGS__);
//    //return printk(KERN_WARNING "AML NFTL warning: %s: line:%d " fmt "\n",  __func__, __LINE__, ##__VA_ARGS__);
//    return printk( fmt,## args);
//    //return printk(KERN_ERR "AML NFTL error: %s: line:%d " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);
//}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int aml_nftl_initialize(struct aml_nftl_dev *nftl_dev,int no)
{
	struct ntd_info *ntd = nftl_dev->ntd;
	int error = 0;
	
	//uint32_t phys_erase_shift;
	uint32_t ret;

	if (ntd->oobsize < MIN_BYTES_OF_USER_PER_PAGE)
		return -EPERM;

	nftl_dev->nftl_cfg.nftl_use_cache = NFTL_DONT_CACHE_DATA;
	nftl_dev->nftl_cfg.nftl_support_gc_read_reclaim = SUPPORT_GC_READ_RECLAIM;
	nftl_dev->nftl_cfg.nftl_support_wear_leveling = SUPPORT_WEAR_LEVELING;
	nftl_dev->nftl_cfg.nftl_need_erase = NFTL_ERASE;
	if(!is_phydev_off_adjust()){
		nftl_dev->nftl_cfg.nftl_part_reserved_block_ratio = 8;
	}else{
		nftl_dev->nftl_cfg.nftl_part_reserved_block_ratio = 10;
	}
	nftl_dev->nftl_cfg.nftl_part_adjust_block_num = get_adjust_block_num();
	printk("adjust_block_num : %d,reserved_block_ratio %d\n",nftl_dev->nftl_cfg.nftl_part_adjust_block_num,nftl_dev->nftl_cfg.nftl_part_reserved_block_ratio);
	nftl_dev->nftl_cfg.nftl_min_free_block_num = MIN_FREE_BLOCK_NUM;
	nftl_dev->nftl_cfg.nftl_min_free_block = MIN_FREE_BLOCK;
	nftl_dev->nftl_cfg.nftl_gc_threshold_free_block_num = GC_THRESHOLD_FREE_BLOCK_NUM ;
	nftl_dev->nftl_cfg.nftl_gc_threshold_ratio_numerator = GC_THRESHOLD_RATIO_NUMERATOR;
	nftl_dev->nftl_cfg.nftl_gc_threshold_ratio_denominator = GC_THRESHOLD_RATIO_DENOMINATOR;
	nftl_dev->nftl_cfg.nftl_max_cache_write_num = MAX_CACHE_WRITE_NUM;

	ret = aml_nftl_start((void*)nftl_dev,&nftl_dev->nftl_cfg,&nftl_dev->aml_nftl_part,ntd->size,ntd->blocksize,ntd->pagesize,ntd->oobsize,ntd->name,no,0,nftl_dev->init_flag);
	if(ret != 0)
	{
        //if(memcmp(ntd->name, "nfcache", 7)==0)
        {
            if(nftl_dev->init_flag == 0)
            {
                aml_nftl_set_status(nftl_dev->aml_nftl_part,1);
            }
    	   // return ret;
        }
	}
	nftl_dev->size = aml_nftl_get_part_cap(nftl_dev->aml_nftl_part);
	nftl_dev->read_data = _nand_read;
	nftl_dev->write_data = _nand_write;
	nftl_dev->discard_data = _nand_discard;
	nftl_dev->flush_write_cache = _nand_flush_write_cache;
	nftl_dev->flush_discard_cache = _nand_flush_discard_cache;
	nftl_dev->invalid_read_cache = _nand_invalid_read_cache;
	nftl_dev->write_pair_page = _nand_write_pair_page;
	nftl_dev->get_current_part_no = _get_current_part_no;
	nftl_dev->check_mapping = _check_mapping;
	nftl_dev->discard_partition = _discard_partition;
	nftl_dev->rebuild_tbls = _rebuild_tbls;
	nftl_dev->compose_tbls = _compose_tbls;
	
	//if(no < 0){
	if (nftl_dev->init_flag) {
		return ret; // for erase init FTL part
	}
	
    //setup class
    if(memcmp(ntd->name, "nfcode", 6)==0)
    {
		nftl_dev->debug.name = kzalloc(strlen((const char*)AML_NFTL1_MAGIC)+1, GFP_KERNEL);
    	strcpy((char *)nftl_dev->debug.name, (char*)AML_NFTL1_MAGIC);
    	nftl_dev->debug.class_attrs = nftl_class_attrs;
   		error = amlnf_class_register(&nftl_dev->debug);
		if(error)
			printk(" class register nand_class fail!\n");
	}

	if(memcmp(ntd->name, "nfdata", 6)==0)
    {
		nftl_dev->debug.name = kzalloc(strlen((const char*)AML_NFTL2_MAGIC)+1, GFP_KERNEL);
    	strcpy((char *)nftl_dev->debug.name, (char*)AML_NFTL2_MAGIC);
    	nftl_dev->debug.class_attrs = nftl_class_attrs;
   		error = amlnf_class_register(&nftl_dev->debug);
		if(error)
			printk(" class register nand_class fail!\n");
	}

	return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int aml_blktrans_initialize(struct aml_nftl_blk *nftl_blk,struct aml_nftl_dev *nftl_dev,uint64_t offset,uint64_t size)
{
    uint64_t offset_t,size_t;

    offset_t = offset >> 9;
    size_t = size >> 9;

	nftl_blk->nftl_dev = nftl_dev;

	if(offset_t < nftl_dev->size)
	{
	    nftl_blk->offset = offset_t;
	}
	else
	{
	    printk("aml_blktrans_initialize2 %llx  %llx \n",offset_t,nftl_dev->size);
	    return 1;
	}

	if((nftl_blk->offset + size_t) <= nftl_dev->size)
	{
	    nftl_blk->size = size_t;
	}
	else
	{
	    nftl_blk->size = nftl_dev->size - nftl_blk->offset;
	}

	nftl_blk->read_data = _blk_nand_read;
	nftl_blk->write_data = _blk_nand_write;
    nftl_blk->discard_data =_blk_nand_discard;
	nftl_blk->flush_write_cache = _blk_nand_flush_write_cache;

	//printk("aml_blktrans_initialize 0x%llx \n",nftl_blk->size);

    return 0;
}


/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
uint32 _nand_read(struct aml_nftl_dev *nftl_dev,unsigned long start_sector,unsigned len,unsigned char *buf)
{
    return __nand_read(nftl_dev->aml_nftl_part,start_sector,len,buf);
}

uint32 _blk_nand_read(struct aml_nftl_blk *nftl_blk,unsigned long  start_sector,unsigned len,unsigned char *buf)
{	
	int ret = 0;

	//mutex_lock(nftl_blk->nftl_dev->aml_nftl_lock);
	ret = _nand_read(nftl_blk->nftl_dev,start_sector + nftl_blk->offset,len,buf);
	//mutex_unlock(nftl_blk->nftl_dev->aml_nftl_lock);
	
    	return ret;
}
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
uint32 _nand_write(struct aml_nftl_dev *nftl_dev,unsigned long start_sector,unsigned len,unsigned char *buf)
{
    uint32 ret;
    ret = __nand_write(nftl_dev->aml_nftl_part,start_sector,len,buf,nftl_dev->sync_flag);
    amlnf_ktime_get_ts(&nftl_dev->ts_write_start);
    return ret;
}
uint32 _nand_discard(struct aml_nftl_dev *nftl_dev,unsigned long start_sector,unsigned len)
{
    uint32 ret;
	#if 0
    if(memcmp(nftl_dev->ntd->name, "nfdata", 6) != 0)
    {
        return 0;
    }
	#endif
    ret = __nand_discard(nftl_dev->aml_nftl_part,start_sector,len,nftl_dev->sync_flag);
    amlnf_ktime_get_ts(&nftl_dev->ts_write_start);
    return ret;
}

uint32 _blk_nand_write(struct aml_nftl_blk *nftl_blk,unsigned long start_sector,unsigned  len,unsigned char *buf)
{
    uint32 ret;
    ret = _nand_write(nftl_blk->nftl_dev,start_sector + nftl_blk->offset,len,buf);

    return ret;
}
uint32 _blk_nand_discard(struct aml_nftl_blk *nftl_blk,unsigned long start_sector,unsigned len)
{
    uint32 ret;
    ret = _nand_discard(nftl_blk->nftl_dev,start_sector + nftl_blk->offset,len);

    return ret;
}
//#define blk_dbg(x...)	printk(x)
#define blk_dbg(x...)
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
uint32 _nand_flush_write_cache(struct aml_nftl_dev *nftl_dev)
{
	blk_dbg("%s()\n", __func__);
    return __nand_flush_write_cache(nftl_dev->aml_nftl_part);
}
uint32 _nand_invalid_read_cache(struct aml_nftl_dev *nftl_dev)
{
	blk_dbg("%s()\n", __func__);
    return __nand_invalid_read_cache(nftl_dev->aml_nftl_part);
}
uint32 _nand_flush_discard_cache(struct aml_nftl_dev *nftl_dev)
{
	blk_dbg("%s()\n", __func__);
    return __nand_flush_discard_cache(nftl_dev->aml_nftl_part);
}
uint32 _nand_write_pair_page(struct aml_nftl_dev *nftl_dev)
{
    return __nand_write_pair_page(nftl_dev->aml_nftl_part);
}
int _get_current_part_no(struct aml_nftl_dev *nftl_dev)
{
    return __get_current_part_no(nftl_dev->aml_nftl_part);
}
uint32 _check_mapping(struct aml_nftl_dev *nftl_dev,uint64_t offset,uint64_t size)
{
    return __check_mapping(nftl_dev->aml_nftl_part,offset,size);
}
uint32 _discard_partition(struct aml_nftl_dev *nftl_dev,uint64_t offset,uint64_t size)
{
    return __discard_partition(nftl_dev->aml_nftl_part,offset,size);
}
uint32 _rebuild_tbls(struct aml_nftl_dev *nftl_dev)
{
	return aml_nftl_reinit(nftl_dev);
}
extern int compose_part_list_info(struct aml_nftl_part_t *part);
uint32 _compose_tbls(struct aml_nftl_dev *nftl_dev)
{
	//fixme, may need lock!!
	return compose_part_list_info(nftl_dev->aml_nftl_part);
}

uint32 _blk_nand_flush_write_cache(struct aml_nftl_blk *nftl_blk)
{
	int ret =0;
	
	//mutex_lock(nftl_blk->nftl_dev->aml_nftl_lock);
	ret =  _nand_flush_write_cache(nftl_blk->nftl_dev);
	//mutex_unlock(nftl_blk->nftl_dev->aml_nftl_lock);

	return ret;
}
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static ssize_t show_part_struct(struct class *class,struct class_attribute *attr, char *buf)
{
    struct aml_nftl_dev *nftl_dev = container_of(class, struct aml_nftl_dev, debug);

    print_nftl_part(nftl_dev -> aml_nftl_part);

    return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static ssize_t show_list(struct class *class, struct class_attribute *attr,	const char *buf, size_t count)
{
    struct aml_nftl_dev *nftl_dev = container_of(class, struct aml_nftl_dev, debug);

    PRINT("--------------name:%s\n",nftl_dev->ntd->name);
    print_free_list(nftl_dev -> aml_nftl_part);
    print_block_invalid_list(nftl_dev -> aml_nftl_part);
//    print_block_count_list(nftl_dev -> aml_nftl_part);

    return 0;
}
#if 0
static ssize_t discard_page(struct class *class, struct class_attribute *attr, const char *buf)
{
    struct aml_nftl_dev *nftl_dev = container_of(class, struct aml_nftl_dev, debug);
    PRINT("1111\n");
    print_discard_page_map(nftl_dev->aml_nftl_part);
    PRINT("2222\n");
    return 0;
}
#endif
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static ssize_t do_gc_all(struct class *class, struct class_attribute *attr,	const char *buf, size_t count)
{
    struct aml_nftl_dev *nftl_dev = container_of(class, struct aml_nftl_dev, debug);

	gc_all(nftl_dev -> aml_nftl_part);
    return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static ssize_t do_gc_one(struct class *class, struct class_attribute *attr,	const char *buf, size_t count)
{
    struct aml_nftl_dev *nftl_dev = container_of(class, struct aml_nftl_dev, debug);

	gc_one(nftl_dev -> aml_nftl_part);
    return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static ssize_t do_test(struct class *class, struct class_attribute *attr,	const char *buf, size_t count)
{
    struct aml_nftl_dev *nftl_dev = container_of(class, struct aml_nftl_dev, debug);
    struct aml_nftl_part_t *part = nftl_dev -> aml_nftl_part;

	unsigned int num;

	sscanf(buf, "%x", &num);

	aml_nftl_set_part_test(part,num);

    return count;
}
