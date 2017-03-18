/*


*/

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
//#include <linux/mtd/blktrans.h>
#include <linux/time.h>
#include <linux/errno.h>
#include <linux/rbtree.h>
//#include <linux/platform_device.h>
#include <linux/sched.h>
//
//#define aml_nftl_malloc(n)		kzalloc(n, GFP_KERNEL)
//#define aml_nftl_free			kfree
#define aml_nftl_dbg            printk



typedef unsigned char         uchar;
typedef unsigned short        uint16;
typedef unsigned long         uint32;
typedef long                  sint32;
typedef short            	  sint16;

/*
#define AML_NFTL_DBG
#ifdef AML_NFTL_DBG
#define aml_nftl_dbg(fmt, ...) printk( "AML NFTL dbg msg: %s: line:%d " fmt "\n", \
  __func__, __LINE__, ##__VA_ARGS__)
#else
#define aml_nftl_dbg(fmt, ...)
#endif
#define aml_nftl_msg(fmt, ...) printk( "AML NFTL normal msg: %s: line:%d " fmt "\n", \
  __func__, __LINE__, ##__VA_ARGS__)

// aml nftl warning messages
#define aml_nftl_warn(fmt, ...) printk(KERN_WARNING "AML NFTL warning: %s: line:%d " fmt "\n", \
  __func__, __LINE__, ##__VA_ARGS__)
// aml nftl error messages
#define aml_nftl_err(fmt, ...) printk(KERN_ERR "AML NFTL error: %s: line:%d " fmt "\n", \
 __func__, __LINE__, ##__VA_ARGS__)
*/

#define	FACTORY_BAD_BLOCK_ERROR  	              2
#define BYTES_PER_SECTOR                          512
#define SHIFT_PER_SECTOR                          9
#define BYTES_OF_USER_PER_PAGE                    16
#define MIN_BYTES_OF_USER_PER_PAGE                16

//#pragma pack(1)

//nand page
typedef struct{
    uint16  Page_NO;
    uint16  blkNO_in_chip;
    uchar   page_status; //unmapp: page_status=0xff  valid mapping: page_status=1 discard: page_status=0
}_nand_page;

//nand discard page
typedef struct{
    uint32  timestamp;
    uchar   page_status;//unmapp: page_status=0xff  valid mapping: page_status=1 discard: page_status=0
}_nand_discard_page;


//从物理的角度看到的接口
typedef struct{
    _nand_page    phy_page;
    uint16        page_bitmap;
    uchar*        main_data_addr;
    uchar*        spare_data_addr;
    int		op_ret_sta;
}_physic_op_par;


struct _nftl_cfg{
    uint16 nftl_use_cache;
    uint16 nftl_support_gc_read_reclaim;
    uint16 nftl_support_wear_leveling;
    uint16 nftl_need_erase;
    uint16 nftl_part_reserved_block_ratio;
	uint16 nftl_part_adjust_block_num;
    uint16 nftl_min_free_block_num;
    uint16 nftl_gc_threshold_free_block_num;
    uint16 nftl_min_free_block;
    uint16 nftl_gc_threshold_ratio_numerator;
    uint16 nftl_gc_threshold_ratio_denominator;
    uint16 nftl_max_cache_write_num;

};

//#pragma pack()
