/*****************************************************************
**                                                              
**  Copyright (C) 2012 Amlogic,Inc.  All rights reserved                         
**                                           
**        Filename : phydev.c       	
**        Revision : 1.001	                                        
**        Author: Benjamin Zhao
**        Description: 
**			1) Phydev basic operation based on phydev
**		         	contains read/write/erase/block_isbad/block_markbad.
**        		2) Get/release chip function ensure only one entry to nand chip; 	       
**            
*****************************************************************/
#include "../include/phynand.h"

int test_flag = 0;
EXPORT_SYMBOL(test_flag);

#ifdef AML_NAND_UBOOT
	//extern struct amlnf_partition amlnand_config;
	extern struct amlnf_partition * amlnand_config;
#endif
int is_phydev_off_adjust(void)
{
	int ret = 0;
	#ifdef NAND_ADJUST_PART_TABLE
		ret = 1;
	#endif
	return  ret ;
}
EXPORT_SYMBOL(is_phydev_off_adjust);

chip_state_t get_chip_state(struct amlnand_chip *aml_chip)
{
	return aml_chip->state;
}

void set_chip_state(struct amlnand_chip *aml_chip, chip_state_t state)
{
	aml_chip->state = state;
}

int aml_nftl_getphydev_size(char *name, uint64_t *size)
{
	struct amlnand_phydev *phydev = NULL;
	int ret = -1,i;
	uint64_t dev_size=0;
	*size = 0;
	list_for_each_entry(phydev, &nphy_dev_list, list){
		if ( phydev != NULL ) {
			if ( !strncmp((char*)phydev->name, name, strlen((const char*)name)) ) {
				aml_nand_msg("calculate device -- %s",name);
				for ( i = 0; i < phydev->nr_partitions; i++ )
				{
					dev_size += phydev->partitions[i].size;
				}
				*size = dev_size;
				ret = 0;
				goto exit_error;
			}
		}
	}
exit_error:
	return ret;
}
EXPORT_SYMBOL(aml_nftl_getphydev_size);


static unsigned amlnand_slc_addr_trs(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct nand_flash *flash = &(aml_chip->flash);
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	//struct chip_operation *operation = &(aml_chip->operation);	
	//struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	struct en_slc_info *slc_info = &(controller->slc_info);
	int real_page_per_blk, page_per_blk, blk_num, blk_num_in_dev, page_num;
	//uint64_t addr, readlen = 0;
	unsigned real_erase_size, real_erase_shift, page_addr = 0;

	real_erase_size = (phydev->erasesize << 1);
	real_erase_shift = ffs(real_erase_size) -1;
	
	real_page_per_blk = (1 << ( (ffs(real_erase_size) -1) -(phydev->writesize_shift)));
	page_per_blk = (1 <<  (phydev->erasesize_shift -phydev->writesize_shift));
	
	blk_num = phydev->offset >> real_erase_shift;
	blk_num_in_dev = devops->addr >> phydev->erasesize_shift;
	blk_num += blk_num_in_dev;
	
	page_num = ((devops->addr >> phydev->writesize_shift) -(blk_num_in_dev * page_per_blk ));

	if((flash->new_type > 0) && (flash->new_type <10)){
		page_addr = blk_num * real_page_per_blk + slc_info->pagelist[page_num];
	}else if (flash->new_type == SANDISK_19NM) {
		page_addr = blk_num * real_page_per_blk + (page_num << 1);
	}else{ // not surpport slc nand
		page_addr = (int)((phydev->offset + devops->addr) >> phydev->writesize_shift);
		aml_nand_msg("nand not surpport slc ");
	}

#if 0
	aml_nand_dbg(" devops->addr =%llx %d", devops->addr, devops->addr);
	aml_nand_dbg("real_erase_size =%x", real_erase_size);
	aml_nand_dbg("real_page_per_blk =%d", real_page_per_blk);
	aml_nand_dbg("page_per_blk =%d", page_per_blk);
	aml_nand_dbg("blk_num =%d", blk_num);
	aml_nand_dbg("blk_num_in_dev =%d", blk_num_in_dev);
	aml_nand_dbg("page_num =%d", page_num);
	aml_nand_dbg("page_addr =%d", page_addr);
#endif
	
	return page_addr;
}

static void nand_write_verify(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct phydev_ops *devops = &(phydev->ops);
	//struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);	
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);

	int ret = 0;

	unsigned char *verify_buf = NULL;
	verify_buf =  aml_nand_malloc(2 *phydev->writesize);
	if(!verify_buf){
		aml_nand_msg("malloc failed for nand_read_verify");
		return;
	}

	ops_para->data_buf = verify_buf;
	ret = operation->read_page(aml_chip);
	if( (ops_para->ecc_err) || (ret<0)){			
		aml_nand_msg("nand phy read failed at devops->addr : %llx", devops->addr);			
	}

	if(memcmp(verify_buf,devops->datbuf,phydev->writesize)){
		aml_nand_msg("nand write verify failed");
	}

	if(verify_buf){
		kfree(verify_buf);
	}

	return;
}

/*
*read data case:
*only data read function, if none ecc mode, buf should be data+oob
* operation type as below:				oob_mode    data_buf    oob_buf 		ooblen 	
*1) read oob hw ecc mode					0			NULL 		available		available
*2) read data and oob hw ecc mode			0  			available    available           available
*3) read data and oob sw ecc mode			1  			available    available           0
*/
static int nand_read(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);	
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	uint64_t addr, readlen = 0, len = 0;
	int ret = 0;

	if((devops->addr + devops->len) >  phydev->size){
		aml_nand_msg("out of space and addr:%llx len:%llx phydev->offset:%llx phydev->size:%llx",\
						devops->addr, devops->len, phydev->offset, phydev->size);
		return -NAND_ARGUMENT_FAILURE;
	}

	if((devops->len == 0) && (devops->ooblen == 0)){
		aml_nand_msg("len equal zero here");
		return NAND_SUCCESS;
	}
	
	amlnand_get_device(aml_chip, CHIP_READING);

	//clear ops_para here
	memset(ops_para, 0, sizeof(struct chip_ops_para));

	if(devops->len == 0){
		len = phydev->writesize;
		ops_para->ooblen = devops->ooblen;
	}
	else{
		len = devops->len;
		ops_para->ooblen = devops->ooblen;
	}
	
	addr = phydev->offset + devops->addr;
	ops_para->data_buf = devops->datbuf;	
	ops_para->option = phydev->option;  
	ops_para->oob_buf = devops->oobbuf;
	
	
	//aml_nand_dbg("len =%llx",len);
	if(devops->mode == NAND_SOFT_ECC){
		ops_para->option |= DEV_ECC_SOFT_MODE;
	}
	
	while(1){
		
		if(ops_para->option & DEV_SERIAL_CHIP_MODE){		
			ops_para->chipnr = (addr >> phydev->erasesize)%controller->chip_num;		
			controller->select_chip(controller, ops_para->chipnr );
			aml_nand_dbg("ops_para->chipnr  =%d",ops_para->chipnr);		
			aml_nand_dbg("DEV_SERIAL_CHIP_MODE");
		}
		
		if(ops_para->option & DEV_SLC_MODE){
			ops_para->page_addr = amlnand_slc_addr_trs(phydev);
		}else{
			ops_para->page_addr = (int)(addr >> phydev->writesize_shift);///controller->chip_num;
		}

		ret = operation->read_page(aml_chip);
		if( (ops_para->ecc_err) || (ret<0)){			
			aml_nand_msg("nand phy read failed at devops->addr : %llx", devops->addr);			
			break;
		}
					
		addr += phydev->writesize;
		
		ops_para->data_buf += phydev->writesize;
 		readlen += phydev->writesize;
		
		//aml_nand_dbg("readlen =%llx",readlen);
		if(readlen >= len){
			break;
		}
	}

 	devops->retlen = readlen;
	
	amlnand_release_device(aml_chip);

	if(!ret){
		if(ops_para->ecc_err){
			ret = NAND_ECC_FAILURE;
		}
		else if(ops_para->bit_flip){
			ret = -EUCLEAN; //117
		}
	}

	return ret;
}

static int nand_write(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);	
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	uint64_t addr, writelen = 0, len = 0;
	int ret = 0;

	if((devops->addr + devops->len) >  phydev->size){
		aml_nand_msg("out of space and addr:%llx len:%llx phydev->offset:%llx phydev->size:%llx",\
						devops->addr, devops->len, phydev->offset, phydev->size);
		return -NAND_ARGUMENT_FAILURE;
	}

	if((devops->len == 0) && (devops->ooblen == 0)){
		aml_nand_msg("len equal zero here");
		return NAND_SUCCESS;
	}

#ifndef AML_NAND_UBOOT
	if(phydev->option & NAND_SHUT_DOWN){
		aml_nand_msg("nand is in shut dowm protect mod");
		return NAND_SUCCESS;
	}
#endif

	amlnand_get_device(aml_chip, CHIP_WRITING);

	len = devops->len;
	
	//clear ops_para here
	memset(ops_para, 0, sizeof(struct chip_ops_para));

	addr = phydev->offset + devops->addr;
	ops_para->option = phydev->option;	
	ops_para->data_buf = devops->datbuf;	
	ops_para->oob_buf = devops->oobbuf;
	ops_para->ooblen = devops->ooblen;
	
	if(devops->mode == NAND_SOFT_ECC){
		ops_para->option |= DEV_ECC_SOFT_MODE;
	}
	
	while(1){
		
		if(ops_para->option & DEV_SERIAL_CHIP_MODE){		
			ops_para->chipnr = (addr>>phydev->erasesize_shift)%controller->chip_num;		
			controller->select_chip(controller, ops_para->chipnr );	
			aml_nand_dbg("DEV_SERIAL_CHIP_MODE");
		}

		if(ops_para->option & DEV_SLC_MODE){
			ops_para->page_addr = amlnand_slc_addr_trs(phydev);
		}else{
			ops_para->page_addr = (int)(addr >> phydev->writesize_shift);///controller->chip_num;
		}
		
		ret = operation->write_page(aml_chip);			
		if(ret<0){			
			aml_nand_msg("nand phy write failed at devops->addr : %llx, addr=%llx", devops->addr,addr); 		
			break;
		}
					
		addr += phydev->writesize;
		ops_para->data_buf += phydev->writesize;
		writelen += phydev->writesize;
		if(writelen >= len){
			break;
		}
	}

	devops->retlen = writelen;
	if(aml_chip->debug_flag & NAND_WRITE_VERIFY){
		nand_write_verify(phydev);
	}
	amlnand_release_device(aml_chip);

	return ret;
}

int nand_erase(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);	
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	uint64_t addr = 0, eraselen = 0;
	int ret = 0;
		
	if((devops->addr + devops->len) >  phydev->size){
		aml_nand_msg("out of space and addr:%llx len:%llx phydev->offset:%llx phydev->size:%llx",\
						devops->addr, devops->len, phydev->offset, phydev->size);
		return -NAND_ARGUMENT_FAILURE;
	}

	if(devops->len == 0){
		aml_nand_msg("len equal zero here");
		return NAND_SUCCESS;
	}
#ifndef AML_NAND_UBOOT	
	if(phydev->option & NAND_SHUT_DOWN){
		aml_nand_msg("nand is in shut dowm protect mod");
		return NAND_SUCCESS;
	}
#endif	
	amlnand_get_device(aml_chip, CHIP_ERASING);

	//clear ops_para here
	memset(ops_para, 0, sizeof(struct chip_ops_para));

	addr = phydev->offset + devops->addr;
	ops_para->option = phydev->option;	
//	aml_nand_dbg(" phydev->option = %d, phydev->erasesize =%x", phydev->option,  phydev->erasesize);
	while(1){

		if(ops_para->option & DEV_SLC_MODE){
			ops_para->page_addr = amlnand_slc_addr_trs(phydev);
		}else{
			ops_para->page_addr = (int)(addr >> phydev->writesize_shift);///controller->chip_num;
		}
		
		if(ops_para->option & DEV_SERIAL_CHIP_MODE){		
			ops_para->chipnr = (addr>>phydev->erasesize)%controller->chip_num;			
			controller->select_chip(controller, ops_para->chipnr );
		}

		ret = operation->erase_block(aml_chip);
		if(ret<0){
			aml_nand_msg("nand erase fail at addr :%x ", ops_para->page_addr);
			break;
		}
		
		addr += phydev->erasesize;
		eraselen += phydev->erasesize;
		
		if(eraselen >= devops->len){
			break;
		}
	}

	devops->retlen = eraselen;
		
	amlnand_release_device(aml_chip);
		
	return ret;
}

static int nand_block_isbad(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	//struct nand_flash *flash = &(aml_chip->flash);
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);	
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	uint64_t addr = 0;
	int ret = 0;
	
	if(devops->addr >  phydev->size){
		aml_nand_msg("out of space and addr:%llx phydev->offset:%llx phydev->size:%llx",\
						devops->addr, phydev->offset, phydev->size);
		return -NAND_ARGUMENT_FAILURE;
	}
#ifndef AML_NAND_UBOOT	
	if(phydev->option & NAND_SHUT_DOWN){
		aml_nand_msg("nand is in shut dowm protect mod");
		return NAND_SUCCESS;
	}
#endif
	amlnand_get_device(aml_chip, CHIP_READING);

	//clear ops_para here
	memset(ops_para, 0, sizeof(struct chip_ops_para));
	
	addr = phydev->offset + devops->addr;
	ops_para->option = phydev->option;	

	if(ops_para->option & DEV_SLC_MODE){
		ops_para->page_addr = amlnand_slc_addr_trs(phydev);
	}else{
		ops_para->page_addr = (int)(addr >> phydev->writesize_shift);
	}
	if((ops_para->option & DEV_SERIAL_CHIP_MODE)){	
		ops_para->chipnr = (addr >> phydev->erasesize_shift) % controller->chip_num;
	}

	ret = operation->block_isbad(aml_chip);
	if(ret < 0){
		aml_nand_msg("fail page_addr:ret=%d, %x len:%llx", ret, ops_para->page_addr, devops->len);
	}
		
	amlnand_release_device(aml_chip);

	return ret;
}

static int nand_block_markbad(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	uint64_t addr = 0;
	int ret = 0;

	if(devops->addr >  phydev->size){
		aml_nand_msg("out of space and addr:%llx phydev->offset:%llx phydev->size:%llx",\
						devops->addr, phydev->offset, phydev->size);
		return -NAND_ARGUMENT_FAILURE;
	}
#ifndef AML_NAND_UBOOT	
	if(phydev->option & NAND_SHUT_DOWN){
		aml_nand_msg("nand is in shut dowm protect mod");
		return NAND_SUCCESS;
	}
#endif
	amlnand_get_device(aml_chip, CHIP_READING);
	
	//clear ops_para here
	memset(ops_para, 0, sizeof(struct chip_ops_para));
	
	addr = phydev->offset + devops->addr;
	ops_para->option = phydev->option;	

	//ops_para->page_addr = (int)(addr >> phydev->writesize_shift);

	if(ops_para->option & DEV_SLC_MODE){
		ops_para->page_addr = amlnand_slc_addr_trs(phydev);
	}else{
		ops_para->page_addr = (int)(addr >> phydev->writesize_shift);
	}
	
	if(ops_para->option & DEV_SERIAL_CHIP_MODE){		
		ops_para->chipnr = (addr>>phydev->erasesize)%controller->chip_num;		
		controller->select_chip(controller, ops_para->chipnr );
	}

	ret = operation->block_markbad(aml_chip);
	if(ret<0){
		aml_nand_msg("nand mark bad failed at page %d",ops_para->page_addr);
	}
		
	amlnand_release_device(aml_chip);

	return ret;
}
static int block_modifybbt(struct amlnand_phydev *phydev,int value)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	uint64_t addr = 0;
	int ret = 0;

	if(devops->addr >  phydev->size){
		aml_nand_msg("out of space and addr:%llx phydev->offset:%llx phydev->size:%llx",\
						devops->addr, phydev->offset, phydev->size);
		return -NAND_ARGUMENT_FAILURE;
	}
	amlnand_get_device(aml_chip, CHIP_READING);
	
	//clear ops_para here
	memset(ops_para, 0, sizeof(struct chip_ops_para));
	
	addr = phydev->offset + devops->addr;
    aml_nand_msg("addr = %lld",addr);
	ops_para->option = phydev->option;	

	//ops_para->page_addr = (int)(addr >> phydev->writesize_shift);

	if(ops_para->option & DEV_SLC_MODE){
		ops_para->page_addr = amlnand_slc_addr_trs(phydev);
	}else{
		ops_para->page_addr = (int)(addr >> phydev->writesize_shift);
	}
	
	if(ops_para->option & DEV_SERIAL_CHIP_MODE){		
		ops_para->chipnr = (addr>>phydev->erasesize)%controller->chip_num;		
		controller->select_chip(controller, ops_para->chipnr );
	}

	ret = operation->blk_modify_bbt_chip_op(aml_chip,value);
	if(ret<0){
		aml_nand_msg("nand mark bad failed at page %d",ops_para->page_addr);
	}
		
	amlnand_release_device(aml_chip);

	return ret;
}

static int update_bbt(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct phydev_ops *devops = &(phydev->ops);
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	uint64_t addr = 0;
	int ret = 0;

	if(devops->addr >  phydev->size){
		aml_nand_msg("out of space and addr:%llx phydev->offset:%llx phydev->size:%llx",\
						devops->addr, phydev->offset, phydev->size);
		return -NAND_ARGUMENT_FAILURE;
	}
	amlnand_get_device(aml_chip, CHIP_READING);
	
	//clear ops_para here
	memset(ops_para, 0, sizeof(struct chip_ops_para));
	
	addr = phydev->offset + devops->addr;
	ops_para->option = phydev->option;	

	//ops_para->page_addr = (int)(addr >> phydev->writesize_shift);

	if(ops_para->option & DEV_SLC_MODE){
		ops_para->page_addr = amlnand_slc_addr_trs(phydev);
	}else{
		ops_para->page_addr = (int)(addr >> phydev->writesize_shift);
	}
	
	if(ops_para->option & DEV_SERIAL_CHIP_MODE){		
		ops_para->chipnr = (addr>>phydev->erasesize)%controller->chip_num;		
		controller->select_chip(controller, ops_para->chipnr );
	}

	ret = operation->update_bbt_chip_op(aml_chip);
	if(ret<0){
		aml_nand_msg("nand mark bad failed at page %d",ops_para->page_addr);
	}
		
	amlnand_release_device(aml_chip);

	return ret;
}
static int nand_test_block(struct amlnand_phydev *phydev)
{
    struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;    
    struct phydev_ops *devops = &(phydev->ops);
    struct hw_controller *controller = &(aml_chip->controller); 
    struct chip_operation *operation = &(aml_chip->operation);
    struct chip_ops_para *ops_para = &(aml_chip->ops_para);
    uint64_t addr = 0;
    int ret = 0;
    unsigned tmp_addr = 0;

    if(devops->addr >  phydev->size){
        aml_nand_msg("out of space and addr:%llx phydev->offset:%llx phydev->size:%llx",\
                        devops->addr, phydev->offset, phydev->size);
        return -NAND_ARGUMENT_FAILURE;
    }
#ifndef AML_NAND_UBOOT	
    if(phydev->option & NAND_SHUT_DOWN){
        aml_nand_msg("nand is in shut dowm protect mod");
        return NAND_SUCCESS;
    }
#endif
    amlnand_get_device(aml_chip, CHIP_READING);
    
    //clear ops_para here
    memset(ops_para, 0, sizeof(struct chip_ops_para));
    
    addr = phydev->offset + devops->addr;
    aml_nand_msg("addr = %lld",addr);
    ops_para->option = phydev->option;  

    //ops_para->page_addr = (int)(addr >> phydev->writesize_shift);

    if(ops_para->option & DEV_SLC_MODE){
        ops_para->page_addr = amlnand_slc_addr_trs(phydev);
    }else{
        ops_para->page_addr = (int)(addr >> phydev->writesize_shift);
    }
    
    if(ops_para->option & DEV_SERIAL_CHIP_MODE){        
        ops_para->chipnr = (addr>>phydev->erasesize)%controller->chip_num;      
        controller->select_chip(controller, ops_para->chipnr );
    }
    tmp_addr = ops_para->page_addr;
    ops_para->data_buf = devops->datbuf;
    ret = operation->erase_block(aml_chip);
    if(ret<0){
        aml_nand_msg("nand erase_block failed at page %d",ops_para->page_addr);
        goto exit;
    }
    
    ops_para->page_addr = tmp_addr;
    do{
    ret = operation->write_page(aml_chip);
    if(ret<0){
        aml_nand_msg("nand erase_block failed at page %d",ops_para->page_addr);
        goto exit;
    }
    ops_para->page_addr += phydev->writesize;
    }while(ops_para->page_addr>=devops->len);

    ops_para->page_addr = tmp_addr;
    do{
    ret = operation->read_page(aml_chip);
    if( (ops_para->ecc_err) || (ret<0)){            
        aml_nand_msg("nand phy read failed at devops->addr : %llx", devops->addr);          
        goto exit;
    }
    ops_para->page_addr += phydev->writesize;
    }while(ops_para->page_addr>=devops->len);

    ops_para->page_addr = tmp_addr;
    ret = operation->erase_block(aml_chip);
    if(ret<0){
        aml_nand_msg("nand erase_block failed at page %d",ops_para->page_addr);
        goto exit;
    }
    aml_nand_msg("nand test OK ");
exit:        
    amlnand_release_device(aml_chip);

    return ret;

}
//static int aml_repair_bbt(struct amlnand_phydev *phydev,uint64_t *bad_blk_addr,int cnt)
//{
//	int i;
//	int error = 0;
//	//int flag = 0;
//	struct phydev_ops *devops = &(phydev->ops);
//	unsigned char * buffer = NULL;
//	buffer = aml_nand_malloc(2 * phydev->writesize);
//	if(!buffer){
//		aml_nand_msg("nand malloc failed");
//		return -1;
//	}
//    memset(buffer, 0xff, 2*phydev->writesize);
//    
//    memset(devops, 0x0, sizeof(struct phydev_ops));
//    devops->len = phydev->erasesize;
//    devops->datbuf = buffer;
//    devops->oobbuf = NULL;
//    devops->mode = NAND_HW_ECC;
//	for(i = 0; i < cnt;i++) {
//        devops->addr = bad_blk_addr[i];
//        aml_nand_msg("devops->addr = %lld,bad_blk_addr[i]=%lld",devops->addr,bad_blk_addr[i]);
//		block_modifybbt(phydev,0);
//		error = nand_test_block(phydev);
//		if(error) {
//            devops->addr = bad_blk_addr[i];
//			block_modifybbt(phydev,1);
//		}
//	}
//	
//
//	return update_bbt(phydev);	
//}


void amldev_dumpinfo(struct amlnand_phydev *phydev)
{
	//flash info
	aml_nand_msg("device info");	
	aml_nand_msg("name:%s, offset:%llx, size:%llx, option:%x", \
		phydev->name, phydev->offset, phydev->size, phydev->option);
	aml_nand_msg("erasesize:%x, writesize:%x, oobavail:%x, erasesize_shift:%x, writesize_shift:%d", \
		phydev->erasesize, phydev->writesize, phydev->oobavail, phydev->erasesize_shift, phydev->writesize_shift);	
}

#ifndef AML_NAND_UBOOT
ssize_t nand_page_read(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	struct amlnand_phydev *phydev = container_of(class, struct amlnand_phydev, cls);
	//struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	//struct hw_controller *controller = &(aml_chip->controller);	
	//struct chip_operation *operation = &(aml_chip->operation);	
	//struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	//struct nand_flash *flash = &(aml_chip->flash);	
	struct phydev_ops  *devops = &(phydev->ops);
	
	unsigned char *datbuf, *p;
	uint64_t offset , write_len;
	loff_t off;
	size_t ret;
	int i;
	
   	ret = sscanf(buf, "%llx", &off);
	datbuf = kmalloc(2*phydev->writesize, GFP_KERNEL);
	if (!datbuf ) {
		printk("No memory for page buffer\n");
 		goto exit_erro;
	}
	p = datbuf;
	memset(datbuf, 0x0, 2*phydev->writesize);
	aml_nand_dbg("phydev->name =%s",phydev->name);
	aml_nand_dbg("read page");

	offset = 0;
	write_len = phydev->writesize;
	
	memset(devops, 0x0, sizeof(struct phydev_ops));
	devops->addr = offset;
	devops->len = phydev->writesize;
	devops->datbuf = datbuf;
	devops->oobbuf = NULL;
	devops->mode = NAND_HW_ECC;
	do{ 	
		if(( ((unsigned)devops->addr % phydev->erasesize)) == 0 ){
			aml_nand_dbg("devops->addr = %llx",devops->addr);
			ret =  phydev->block_isbad(phydev);
			if (ret > 0){
				aml_nand_dbg("\rSkipping bad block at %llx\n", devops->addr);
				devops->addr += phydev->erasesize;
				continue;
			} else if (ret < 0){
				aml_nand_dbg("\n:AMLNAND get bad block failed: ret=%d at addr=%llx\n",ret, devops->addr);
				return -1;
			}		
		}
		ret = phydev->read(phydev);
		if(ret < 0){
			aml_nand_dbg("nand read failed at %llx",devops->addr);
		}
		devops->addr +=  phydev->writesize;
		datbuf += phydev->writesize;
	}while(devops->addr < (offset + write_len));

	i = 512;
	aml_nand_dbg("read page");
    while (i--) {
		printk("\t%02x %02x %02x %02x %02x %02x %02x %02x"
		       "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
		       p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
		       p[8], p[9], p[10], p[11], p[12], p[13], p[14],
		       p[15]);
		p += 16;
	}

   aml_nand_dbg("read page complete");

   exit_erro:
   	kfree(datbuf);
	
	return count;
}

void amlchip_resume(struct amlnand_phydev *phydev)
{
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	struct hw_controller *controller = &(aml_chip->controller);	
	struct chip_operation *operation = &(aml_chip->operation);
	unsigned char onfi_features[4] = {0};
	int i=0,ret = 0;
	/* only resume once */
	if (!strncmp((char*)phydev->name, NAND_CODE_NAME, strlen((const char*)NAND_CODE_NAME)) ){

    	nand_get_chip(aml_chip);
    
    	for(i= 0; i <controller->chip_num; i++) {
    		ret = controller->select_chip(controller, i );
    		if(ret < 0){
    			aml_nand_msg("select chip %d failed",i);
    		}
    		ret = nand_reset(aml_chip,i);
    		if(ret < 0){
    			aml_nand_dbg("reset failed %d",i);
    		}
    	}
    
    	if (controller->onfi_mode) {
    		operation->set_onfi_para(aml_chip, (unsigned char *)&(controller->onfi_mode), ONFI_TIMING_ADDR);
    		operation->get_onfi_para(aml_chip, onfi_features, ONFI_TIMING_ADDR);
    		if (onfi_features[0] != controller->onfi_mode) {
    			aml_chip->flash.T_REA = DEFAULT_T_REA;
    			aml_chip->flash.T_RHOH = DEFAULT_T_RHOH;
    		}
    	}
    	
    	//if (aml_chip->state == CHIP_PM_SUSPENDED)
    	amlnand_release_device(aml_chip);
    	aml_nand_dbg("nand resume entered\n");
    }

	return;
}

ssize_t nand_page_write(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	struct amlnand_phydev *phydev = container_of(class, struct amlnand_phydev, cls);
	//struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	//struct hw_controller *controller = &(aml_chip->controller);	
	//struct chip_operation *operation = &(aml_chip->operation);	
	//struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	//struct nand_flash *flash = &(aml_chip->flash);	
	struct phydev_ops  *devops = &(phydev->ops);
    u_char *datbuf;

	uint64_t offset , write_len;
	loff_t off;
	size_t ret;
	//int i;
	
	aml_nand_dbg("phydev->name =%s",phydev->name);
	aml_nand_dbg("write page");
   	ret = sscanf(buf, "%llx", &off);
	datbuf = kmalloc(2*phydev->writesize, GFP_KERNEL);
	if (!datbuf ) {
		printk("No memory for page buffer\n");
 		goto exit_erro;
	}
	memset(datbuf, 0xa5, 2*phydev->writesize);

	offset = 0;
	write_len = phydev->writesize;
	
	memset(devops, 0x0, sizeof(struct phydev_ops));
	devops->addr = offset;
	devops->len = phydev->writesize;
	devops->datbuf = datbuf;
	devops->oobbuf = NULL;
	devops->mode = NAND_HW_ECC;
	do{ 	
		if(( ((unsigned)devops->addr % phydev->erasesize)) == 0 ){
			aml_nand_dbg("devops->addr = %llx",devops->addr);
			ret =  phydev->block_isbad(phydev);
			if (ret > 0){
				aml_nand_msg("\rSkipping bad block at %llx\n", devops->addr);
				devops->addr += phydev->erasesize;
				continue;
			} else if (ret < 0) {
				aml_nand_msg("\n:AMLNAND get bad block failed: ret=%d at addr=%llx\n",ret, devops->addr);
				return -1;
			}
		}
		ret = phydev->write(phydev);
		if(ret < 0){
			aml_nand_msg("nand read failed at %llx",devops->addr);
		}
		devops->addr +=  phydev->writesize;
		datbuf += phydev->writesize;
	}while(devops->addr < (offset + write_len));

   aml_nand_dbg("write page complete");

   exit_erro:
   	kfree(datbuf);
	
	return count;
}

/****
*verify_nand_page: 
*read data immediately when write data, then compare;
*
*enbale this function:  set the first bit of aml_chip->debug_flag  to 1; 
*****/
ssize_t verify_nand_page(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{	
	struct amlnand_phydev *phydev = container_of(class, struct amlnand_phydev, cls);
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	

	unsigned char  off;
	size_t ret;

	ret = sscanf(buf, "%d", (int *)&off);

	aml_chip->debug_flag = off;
	
	aml_nand_msg("nand set aml_chip->debug_flag to %d",aml_chip->debug_flag);

	return count;
}

ssize_t dump_nand_page(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	struct amlnand_phydev *phydev = container_of(class, struct amlnand_phydev, cls);
	//struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;	
	//struct hw_controller *controller = &(aml_chip->controller);	
	//struct chip_operation *operation = &(aml_chip->operation);	
	//struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	//struct nand_flash *flash = &(aml_chip->flash);	
	struct phydev_ops  *devops = &(phydev->ops);

	unsigned char *datbuf, *p;
	uint64_t offset , write_len;
	uint64_t off;
	size_t ret;
	int i;
	
	ret = sscanf(buf, "%llx", &off);
	
	aml_nand_msg("dump page %llx",off);
	datbuf = kmalloc(2*phydev->writesize, GFP_KERNEL);
	if (!datbuf ) {
		printk("No memory for page buffer\n");
 		goto exit_erro;
	}
	p = datbuf;
	memset(datbuf, 0x0, 2*phydev->writesize);
	aml_nand_msg("phydev->name =%s",phydev->name);

	offset = off;
	write_len = phydev->writesize;
	aml_nand_msg("offset %llx",offset);
	
	memset(devops, 0x0, sizeof(struct phydev_ops));
	devops->addr = offset;
	devops->len = phydev->writesize;
	devops->datbuf = datbuf;
	devops->oobbuf = NULL;
	devops->mode = NAND_SOFT_ECC;
	do{ 	
		if(( ((unsigned)devops->addr % phydev->erasesize)) == 0 ){
			aml_nand_dbg("devops->addr = %llx",devops->addr);
			ret =  phydev->block_isbad(phydev);
			if (ret > 0){
				aml_nand_dbg("\rSkipping bad block at %llx\n", devops->addr);
				devops->addr += phydev->erasesize;
				continue;
			} else if (ret < 0){
				aml_nand_dbg("\n:AMLNAND get bad block failed: ret=%d at addr=%llx\n",ret, devops->addr);
				return -1;
			}		
		}
		ret = phydev->read(phydev);
		if(ret < 0){
			aml_nand_dbg("nand read failed at %llx",devops->addr);
		}
		devops->addr +=  phydev->writesize;
		datbuf += phydev->writesize;
	}while(devops->addr < (offset + write_len));

	i = 512;
	aml_nand_dbg("read page");
    while (i--) {
		printk("\t%02x %02x %02x %02x %02x %02x %02x %02x"
		       "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
		       p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
		       p[8], p[9], p[10], p[11], p[12], p[13], p[14],
		       p[15]);
		p += 16;
	}

   aml_nand_dbg("read page complete");

   exit_erro:
   	kfree(datbuf);
	
	return count;
}

ssize_t show_nand_info(struct class *class, struct class_attribute *attr, char *buf)
{
	struct amlnand_phydev *phydev = container_of(class, struct amlnand_phydev, cls);
	aml_nand_dbg("phydev info:");
	
	amldev_dumpinfo(phydev);
	return 0;
}

ssize_t show_bbt_table(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    	struct amlnand_phydev *phydev = container_of(class, struct amlnand_phydev, cls);
	struct amlnand_chip *aml_chip = (struct amlnand_chip *)phydev->priv;		
	struct hw_controller *controller = &(aml_chip->controller);
	unsigned short *tmp_arr;
	int start_block, chipnr, total_block;
	aml_nand_msg("show the block status !!!!");
	
	sscanf(buf, "%d", &total_block);
	aml_nand_msg("set total_block %d",total_block);
	
	for(chipnr= 0; chipnr < controller->chip_num; chipnr++){		
		tmp_arr = &aml_chip->block_status->blk_status[chipnr][0];			
			for(start_block=total_block; start_block < (total_block+50); start_block++){	
				aml_nand_msg(" aml_chip->block_status[%d][%d]=%d", chipnr, start_block, tmp_arr[start_block]);
			}
	}

	return count;
}
ssize_t nand_ioctl(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    uint32_t num;
    
    sscanf(buf, "%x", &num);
    printk("---------------------------------------------test_flag=%d\n",num);
    test_flag = num;
    //test_flag bit layout
    //bit0: 0 means media don't sync, 1 means all partition don't sync
    //bit1: 0 means close blk request print, 1 means open blk request print
    //bit2: 0 means do nothing, 1 means check if block is already in free list before add it to free list.
    //the rest bit is undefined now.
	return count;
}

ssize_t show_amlnf_version_info(struct class *class, struct class_attribute *attr, char *buf)
{
    	aml_nand_dbg("show_nand_version_info v0.01");
	return 0;		
}

#endif


static void show_phydev_info(void)
{
	struct amlnand_phydev *phydev = NULL;
	struct amlnf_partition *partition = NULL;
	int i=0;
	char *config1, *config2;

	list_for_each_entry(phydev,&nphy_dev_list,list){
		if(phydev!=NULL){
			for(i=0;i<phydev->nr_partitions; i++){
				partition = &phydev->partitions[i];
				aml_nand_dbg("%s : partition->name= %s,	partition->size= %llx",phydev->name,partition->name,partition->size);
			}
			if(phydev->option & DEV_MULTI_CHIP_MODE)
				config1 = "multi_chip";
			else
				config1 = "single_chip";
			
			if(phydev->option & DEV_MULTI_PLANE_MODE)
				config2 = "multi_plane";
			else
				config2 = "single_plane";
			aml_nand_msg("%-10s : offset: 0x%012llx -0x%012llx : partitons %d : %s %s",
				phydev->name,phydev->offset,phydev->size,phydev->nr_partitions,config1,config2);
		}
	}
	
}

/******
*nand chip usage
* all dev size should 
*	uboot(nand boot)		reserved	code	data
*/

extern struct list_head nphy_dev_list;
// this function alloc phydev and init it.
int aml_alloc_phydev(struct amlnand_phydev **phydev_pp, struct amlnand_chip *aml_chip, struct dev_para **dev_para, int dev_idx)
{
    int ret = 0;
    struct hw_controller *controller = &(aml_chip->controller);
    //struct nand_config *config = aml_chip->config_ptr;
    struct nand_flash *flash = &(aml_chip->flash);
    struct amlnand_phydev* phydev_p = NULL;
        
    *phydev_pp = aml_nand_malloc(sizeof(struct amlnand_phydev));
    if(*phydev_pp == NULL){
        aml_nand_msg("malloc failed need %x here", (sizeof(struct amlnand_phydev)));
        ret = -NAND_MALLOC_FAILURE;
        return ret;
    }
    phydev_p = *phydev_pp;
    memset(phydev_p, 0, sizeof(struct amlnand_phydev));
    phydev_p->priv = aml_chip;
    
    *dev_para = &aml_chip->config_ptr->dev_para[dev_idx];
    memcpy(&phydev_p->name, &(*dev_para)->name, MAX_DEVICE_NAME_LEN*sizeof(char));
    //set default parameter
    phydev_p->writesize = flash->pagesize;
    phydev_p->erasesize = flash->blocksize;
    phydev_p->oobavail = controller->oobavail; 
	phydev_p->writesize_shift = ffs(phydev_p->writesize) - 1;	
	phydev_p->erasesize_shift = ffs(phydev_p->erasesize) - 1;
    phydev_p->write = nand_write;
    phydev_p->read = nand_read;
    phydev_p->erase = nand_erase;
    phydev_p->block_isbad = nand_block_isbad;
    phydev_p->block_markbad = nand_block_markbad;
    phydev_p->block_modifybbt = block_modifybbt;
    phydev_p->update_bbt = update_bbt;
    phydev_p->phydev_test_block = nand_test_block;
#ifndef AML_NAND_UBOOT
    	phydev_p->suspend = phydev_suspend;
    	phydev_p->resume = phydev_resume;
#endif
    return ret;

}

int amlnand_phydev_init(struct amlnand_chip *aml_chip)
{
	struct amlnand_phydev *phydev = NULL, *phydev_pre = NULL;	
	struct nand_flash *flash = &(aml_chip->flash);
	struct hw_controller *controller = &(aml_chip->controller);
	struct chip_operation *operation = &(aml_chip->operation);	
	struct chip_ops_para *ops_para = &(aml_chip->ops_para);
	struct nand_config *config = aml_chip->config_ptr;	
    nand_arg_info phy_part = aml_chip->nand_phy_partition;
	struct dev_para *dev_para = NULL;
	struct amlnf_partition *partition = NULL;
	
	uint64_t  offset = 0, dev_size = 0, chip_size =0, phydev_pre_size =0, tmp_offset=0;
	unsigned start_blk, total_blk, tmp_write_shift, tmp_erase_shift, tmp_blk = 0, pages_per_blk;
	unsigned char boot_flag = 0, plane_num = 1;
	int i, j, k, ret = 0;	
    uint64_t *bad_blk = NULL;
    bad_blk = aml_nand_malloc(128*sizeof(uint64_t));
	if(bad_blk == NULL){
		aml_nand_msg("malloc failed");
		ret = -NAND_MALLOC_FAILURE;
		goto exit_error0;
	}
    memset(bad_blk,0,128*sizeof(uint64_t));
	if(flash->option & NAND_MULTI_PLANE_MODE){
		plane_num = 2;
	}
	else{
		plane_num = 1;
	}
//#ifndef AML_NAND_UBOOT	
//	INIT_LIST_HEAD (&nphy_dev_list);
//#endif
	chip_size = (flash->chipsize*controller->chip_num);
	chip_size = chip_size << 20;
	if(config->dev_num == 0){
		aml_nand_msg("config get unvalid: config->dev_num =%d",config->dev_num);
		ret = -NAND_FAILED; 		
		goto exit_error0;
	}
    //if phy partition valid then no need to calc and no need to care about driver version
    if(phy_part.arg_valid == 1){
    	for(i=0; i<config->dev_num; i++){
            ret = aml_alloc_phydev(&phydev, aml_chip,&dev_para,i);
            phydev->offset = aml_chip->phy_part_ptr->partition[i].phy_off;
            phydev->size = aml_chip->phy_part_ptr->partition[i].phy_len;
			phydev->nr_partitions = dev_para->nr_partitions;
			phydev->partitions = dev_para->partitions;
			if((dev_para->option & DEV_MULTI_CHIP_MODE) && (controller->chip_num > 1)){
				phydev->option |= DEV_MULTI_CHIP_MODE;	
				phydev->writesize *= controller->chip_num;
				phydev->erasesize *= controller->chip_num;
				phydev->oobavail *= controller->chip_num;
			}
			
			if((dev_para->option & DEV_MULTI_PLANE_MODE) && (flash->option & NAND_MULTI_PLANE_MODE)){
				phydev->option |= DEV_MULTI_PLANE_MODE;		
				
				phydev->writesize <<= 1;			
				phydev->erasesize <<= 1;				
				phydev->oobavail <<= 1;				
			}
        	phydev->writesize_shift = ffs(phydev->writesize) - 1;	
        	phydev->erasesize_shift = ffs(phydev->erasesize) - 1;
            printk("%s,%d,phydev->offset=%llx,phydev->size=%llx\n",__func__,__LINE__,phydev->offset,phydev->size);
            list_add_tail(&phydev->list, &nphy_dev_list);
    	}
    }else{
    if(NAND_COMPATIBLE_REGION <= 2){// use the old method to calc, should never change here
    	for(i=0; i<config->dev_num; i++){
            ret = aml_alloc_phydev(&phydev, aml_chip,&dev_para,i);
            //printk("%s,%d,phydev->name:%s\n",__func__,__LINE__,phydev->name);
    		dev_size = 0;
    		tmp_write_shift = ffs(flash->pagesize) -1;
    		tmp_erase_shift = ffs(flash->blocksize) -1;
    		pages_per_blk = (1 << (tmp_erase_shift -tmp_write_shift));	
            
    		//set partitions and caulate dev size
    		if(dev_para->nr_partitions){
    			phydev->nr_partitions = dev_para->nr_partitions;
    			phydev->partitions = dev_para->partitions;
    			for(k=0; k<dev_para->nr_partitions; k++){
    				partition = &(dev_para->partitions[k]);
    				aml_nand_dbg("partition[%d]->name:%s, size %llx",k, partition->name, partition->size);
    			}

    			if(i != (config->dev_num -1)){
    				for(j=0; j<dev_para->nr_partitions; j++){
    					partition = &(dev_para->partitions[j]);
    					dev_size += partition->size;
    				}
    				if(!is_phydev_off_adjust()){
    					int adjust_shift =  ffs(ADJUST_SIZE_NFTL) -1;
    					//aml_nand_msg("not adjust, adjust_shift : %d",adjust_shift);
    					dev_size = dev_size + (dev_size >> adjust_shift); 
    				}
    				//dev_size = dev_size + ((uint64_t)(dev_size)/(uint64_t)(ADJUST_SIZE_NFTL));   //adjust dev_size for nftl				
    			}
    			else{
    				if((phydev_pre->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
    					phydev_pre_size =  phydev_pre->size << 1; 
    				}else{
    					phydev_pre_size = phydev_pre->size;
    				}
    				dev_size = chip_size -phydev_pre->offset -phydev_pre_size;
    			}
    		}
    		else{
    			dev_size = dev_para->size;
    		}
    		if((dev_para->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(dev_para->option & DEV_MULTI_PLANE_MODE ))){
    			dev_size <<= 1; 
    			aml_nand_msg("DEV enable SLC mode");
    		}

    		if (!strncmp((char*)phydev->name, NAND_BOOT_NAME, strlen((const char*)NAND_BOOT_NAME))){
    			boot_flag = 1;
    			
    			phydev->offset = 0;
    			phydev->size = (BOOT_COPY_NUM*BOOT_PAGES_PER_COPY);
    			phydev->size *= flash->pagesize;
    			//phydev->size *= controller->chip_num;

    			phydev->writesize_shift = ffs(phydev->writesize) - 1;	
    			phydev->erasesize_shift = ffs(phydev->erasesize) -1;
    			phydev->writesize_mask =  (phydev->size>>phydev->writesize_shift) - 1;	

    			tmp_blk = phydev->size >> phydev->erasesize_shift;
    		}
    		else{
    			if((dev_para->option & DEV_MULTI_CHIP_MODE) && (controller->chip_num > 1)){
    				phydev->option |= DEV_MULTI_CHIP_MODE;	
    				phydev->writesize *= controller->chip_num;
    				phydev->erasesize *= controller->chip_num;
    				phydev->oobavail *= controller->chip_num;
    			}
    			
    			if((dev_para->option & DEV_MULTI_PLANE_MODE) && (flash->option & NAND_MULTI_PLANE_MODE)){
    				phydev->option |= DEV_MULTI_PLANE_MODE;		
    				
    				phydev->writesize <<= 1;			
    				phydev->erasesize <<= 1;				
    				phydev->oobavail <<= 1;				
    			}

    			phydev->writesize_shift = ffs(phydev->writesize) - 1;		
    			phydev->erasesize_shift = ffs(phydev->erasesize) -1;

    			if(boot_flag == 1){
    				if(i == 1){
    					offset =start_blk = 0;
    					total_blk = RESERVED_BLOCK_CNT;
    					memset(ops_para, 0, sizeof(struct chip_ops_para));
    					do{    
    						ops_para->page_addr =((((unsigned)(((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) - ((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) \
    							% controller->chip_num) /controller->chip_num) + tmp_blk -tmp_blk/controller->chip_num) * pages_per_blk);
    						ops_para->chipnr = ((unsigned) (offset >> tmp_erase_shift)) % controller->chip_num;
    						ret = operation->block_isbad(aml_chip);
    						if (ret == NAND_BLOCK_FACTORY_BAD){
    							offset += flash->blocksize;  
    							continue;
    						}
    						start_blk++;
    						offset += flash->blocksize; 
    					} while (start_blk < total_blk);		
    					total_blk = ((((unsigned) (offset>>tmp_erase_shift)) - 1)/(controller->chip_num*plane_num)+ 1)* (controller->chip_num*plane_num);
    					aml_nand_dbg("total_blk =%d",total_blk);
    					aml_nand_dbg(" phydev_pre->size =%llx", phydev_pre->size);
    					phydev->offset = total_blk * flash->blocksize + phydev_pre->size;
    					aml_nand_dbg("phydev->offset =%llx",phydev->offset);
    				}
    				else{
    					if((phydev_pre->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
    						phydev_pre_size =  phydev_pre->size << 1; 
    					}else{
    						phydev_pre_size = phydev_pre->size;
    					}
    					phydev->offset = phydev_pre->offset + phydev_pre_size;			
    				}
    			}
    			else{		
    				if(phydev_pre == NULL){
    					offset =start_blk = 0;
    					total_blk = RESERVED_BLOCK_CNT;
    					memset(ops_para, 0, sizeof(struct chip_ops_para));
    					do{    
    						ops_para->page_addr =((((unsigned)(((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) - ((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) \
    							% controller->chip_num) /controller->chip_num) + tmp_blk -tmp_blk/controller->chip_num) * pages_per_blk);
    						ops_para->chipnr = ((unsigned) (offset >> tmp_erase_shift)) % controller->chip_num;
    						ret = operation->block_isbad(aml_chip);
    						if (ret == NAND_BLOCK_FACTORY_BAD){
    							offset += flash->blocksize;  
    							continue;
    						}
    						start_blk++;
    						offset += flash->blocksize; 
    					} while (start_blk < total_blk);		
    					total_blk = ((((unsigned) (offset>>tmp_erase_shift)) - 1)/(controller->chip_num*plane_num)+ 1)* (controller->chip_num*plane_num);
    					aml_nand_dbg("total_blk =%d",total_blk);
    					phydev->offset = total_blk * flash->blocksize;
    					aml_nand_dbg("phydev->offset =%llx",phydev->offset);
    				}
    				else{
    					if((phydev_pre->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
    						phydev_pre_size =  phydev_pre->size << 1; 
    					}else{
    						phydev_pre_size = phydev_pre->size;
    					}
    					phydev->offset = phydev_pre->offset + phydev_pre_size;			
    				}
    			}				

    			if(i != (config->dev_num -1)){
    					start_blk = 0;
    					if(boot_flag == 1){
    						if(i==1){
    								tmp_offset = offset = phydev->offset;
    							}
    							else{
    								tmp_offset = offset = phydev_pre->offset + phydev_pre->size;
    							}
    					}else if(boot_flag == 0){
    						if(i==0){
    							tmp_offset = offset	= phydev->offset;
    						}else{
    							tmp_offset = offset = phydev_pre->offset + phydev_pre->size;
    						}
    					}
    					aml_nand_dbg("offset = %llx, %llx",offset,tmp_erase_shift);
    					if(!is_phydev_off_adjust()){
    						total_blk = dev_size  >> tmp_erase_shift;
    					}else{
    						aml_nand_msg("nand adjust phy offset : block %d",ADJUST_BLOCK_NUM);
    						total_blk = dev_size  >> phydev->erasesize_shift;
    						#ifdef CONFIG_NAND_AML_M8B						
    						total_blk = total_blk + (total_blk + ADJUST_PART_SIZE - 1) /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
    						#else
    						if(IS_MESON_M8_CPU) //M8
    						{
    							if(phydev->erasesize >= 0x400000){
    							total_blk = total_blk + (total_blk + ADJUST_PART_SIZE - 1) /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
    							}
    							else{
    							total_blk = total_blk + total_blk /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
    							}
    						}else{//M8M2
    							total_blk = total_blk + (total_blk + ADJUST_PART_SIZE - 1) /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
    						}
    						#endif
    					}
    					memset(ops_para, 0, sizeof(struct chip_ops_para));
    					ops_para->option = phydev->option;
    					if(!is_phydev_off_adjust()){
    					do{
    						ops_para->page_addr = ((((unsigned)((((unsigned)(offset >> tmp_erase_shift))) - (((unsigned)(offset >> tmp_erase_shift))) \
    							% controller->chip_num) /controller->chip_num) + tmp_blk -tmp_blk /controller->chip_num ) * pages_per_blk);
    						ops_para->chipnr =((unsigned) (offset >>tmp_erase_shift)) % controller->chip_num;	
    						ret = operation->block_isbad(aml_chip);
    						if (ret == NAND_BLOCK_FACTORY_BAD){
    							offset += flash->blocksize;
    							continue;
    						}
    						start_blk++;
    						offset += flash->blocksize;
    					} while (start_blk < total_blk);
    					total_blk = ((((unsigned) (offset >> phydev->erasesize_shift)) - 1)/(controller->chip_num*plane_num) + 1)* (controller->chip_num*plane_num);
    					aml_nand_dbg("total_blk =%d",total_blk);
    					phydev->size = ((uint64_t)total_blk*(uint64_t)phydev->erasesize);
    					}else{
    						do{
    							ops_para->page_addr = ((unsigned)(offset >> phydev->writesize_shift)) ;
    							//ops_para->chipnr =((unsigned) (offset>>phydev->erasesize_shift)) % controller->chip_num;
    							ret = operation->block_isbad(aml_chip); 
    							if (ret == NAND_BLOCK_FACTORY_BAD){
    								offset += phydev->erasesize;
    								//aml_nand_msg("#### : offset %llx",offset);
    								continue;	
    							}
    							start_blk++;
    							offset += phydev->erasesize;
    						}while(start_blk < total_blk);
    						total_blk = ((((unsigned) ((offset -tmp_offset) >> phydev->erasesize_shift)) - 1)/(controller->chip_num*plane_num) + 1)* (controller->chip_num*plane_num);
    						phydev->size = ((uint64_t)total_blk*(uint64_t)phydev->erasesize);
                            printk("offset:%llx,tmp_offset:%llx,total_blk:%d,phydev->size:%llx\n",offset,tmp_offset,total_blk,phydev->size);
    				}				
    			}
    			else{
    				phydev->size = dev_size;
    			}
    			
    			if((phydev->offset + phydev->size) > chip_size){
    				aml_nand_msg("nand dev size is out of space");
    				ret = -NAND_FAILED; 		
    				goto exit_error0;
    			}
    			
#ifndef AML_NAND_UBOOT
    			phydev->suspend = phydev_suspend;
    			phydev->resume = phydev_resume;
#endif
    		}	

    		phydev_pre = phydev;
    		
    		if((dev_para->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
    			phydev->option |= DEV_SLC_MODE;	

    			phydev->erasesize >>= 1;
    			phydev->erasesize_shift = ffs(phydev->erasesize) -1;
    			phydev->size >>=1;
    			aml_nand_msg(" DEV %s enable SLC mode",phydev->name);
    		}
	        aml_chip->phy_part_ptr->partition[i].phy_off = phydev->offset;
            aml_chip->phy_part_ptr->partition[i].phy_len = phydev->size;
            //printk("%s,%d,phydev->offset=%llx,phydev->size=%llx\n",__func__,__LINE__,phydev->offset,phydev->size);
            if(dev_para->nr_partitions){
    			for(k=0; k<dev_para->nr_partitions; k++){
    				partition = &(dev_para->partitions[k]);
    				aml_chip->phy_part_ptr->partition[i].logic_len += partition->size;
    			}
                //printk("%s,%d,logic size:%llx\n",__func__,__LINE__,aml_chip->phy_part_ptr->partition[i].logic_len);
            }else{
                aml_chip->phy_part_ptr->partition[i].logic_len = dev_para->size;
                //printk("%s,%d,logic size:%llx\n",__func__,__LINE__,aml_chip->phy_part_ptr->partition[i].logic_len);
            }
    		list_add_tail(&phydev->list, &nphy_dev_list);

    		
#ifdef AML_NAND_DBG		
    		amldev_dumpinfo(phydev);
#endif		
    		aml_nand_dbg("####: %s :phydev->offset  = %llx",phydev_pre->name,phydev_pre->offset);	
    		aml_nand_dbg("####: %s :phydev->size = %llx",phydev_pre->name,phydev_pre->size);
    		aml_nand_dbg("####: %s :phydev->writesize = %x",phydev_pre->name,phydev_pre->writesize);	
    		aml_nand_dbg("####: %s :phydev->erasesize = %x",phydev_pre->name,phydev_pre->erasesize);
    		
    	}
        aml_chip->phy_part_ptr->dev_num = config->dev_num;
        aml_chip->phy_part_ptr->crc = aml_info_checksum((unsigned char *)aml_chip->phy_part_ptr->partition,(MAX_DEVICE_NUM*sizeof(struct _phy_partition)));
        ret = amlnand_save_info_by_name(aml_chip, (unsigned char *)&(aml_chip->nand_phy_partition),(unsigned char *)aml_chip->phy_part_ptr,PHY_PARTITION_HEAD_MAGIC, sizeof(struct phy_partition_info));
        if(ret < 0){
            aml_nand_msg("save nand phy partition failed and ret:%d",ret);            
            goto exit_error0;
        }

    }else{// if  we want to change the method of calc, change it here and must make sure NAND_COMPATIBLE_REGION greater than 3
        for(i=0; i<config->dev_num; i++){
                ret = aml_alloc_phydev(&phydev, aml_chip,&dev_para,i);
                //printk("%s,%d,phydev->name:%s\n",__func__,__LINE__,phydev->name);
        		dev_size = 0;
        		tmp_write_shift = ffs(flash->pagesize) -1;
        		tmp_erase_shift = ffs(flash->blocksize) -1;
        		pages_per_blk = (1 << (tmp_erase_shift -tmp_write_shift));	
                
        		//set partitions and caulate dev size
        		if(dev_para->nr_partitions){
        			phydev->nr_partitions = dev_para->nr_partitions;
        			phydev->partitions = dev_para->partitions;
        			for(k=0; k<dev_para->nr_partitions; k++){
        				partition = &(dev_para->partitions[k]);
        				aml_nand_dbg("partition[%d]->name:%s, size %llx",k, partition->name, partition->size);
        			}

        			if(i != (config->dev_num -1)){
        				for(j=0; j<dev_para->nr_partitions; j++){
        					partition = &(dev_para->partitions[j]);
        					dev_size += partition->size;
        				}
        				if(!is_phydev_off_adjust()){
        					int adjust_shift =  ffs(ADJUST_SIZE_NFTL) -1;
        					//aml_nand_msg("not adjust, adjust_shift : %d",adjust_shift);
        					dev_size = dev_size + (dev_size >> adjust_shift); 
        				}
        				//dev_size = dev_size + ((uint64_t)(dev_size)/(uint64_t)(ADJUST_SIZE_NFTL));   //adjust dev_size for nftl				
        			}
        			else{
        				if((phydev_pre->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
        					phydev_pre_size =  phydev_pre->size << 1; 
        				}else{
        					phydev_pre_size = phydev_pre->size;
        				}
        				dev_size = chip_size -phydev_pre->offset -phydev_pre_size;
        			}
        		}
        		else{
        			dev_size = dev_para->size;
        		}
        		if((dev_para->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(dev_para->option & DEV_MULTI_PLANE_MODE ))){
        			dev_size <<= 1; 
        			aml_nand_msg("DEV enable SLC mode");
        		}

        		if (!strncmp((char*)phydev->name, NAND_BOOT_NAME, strlen((const char*)NAND_BOOT_NAME))){
        			boot_flag = 1;
        			
        			phydev->offset = 0;
        			phydev->size = (BOOT_COPY_NUM*BOOT_PAGES_PER_COPY);
        			phydev->size *= flash->pagesize;
        			//phydev->size *= controller->chip_num;

        			phydev->writesize_shift = ffs(phydev->writesize) - 1;	
        			phydev->erasesize_shift = ffs(phydev->erasesize) -1;
        			phydev->writesize_mask =  (phydev->size>>phydev->writesize_shift) - 1;	

        			tmp_blk = phydev->size >> phydev->erasesize_shift;
        		}
        		else{
        			if((dev_para->option & DEV_MULTI_CHIP_MODE) && (controller->chip_num > 1)){
        				phydev->option |= DEV_MULTI_CHIP_MODE;	
        				phydev->writesize *= controller->chip_num;
        				phydev->erasesize *= controller->chip_num;
        				phydev->oobavail *= controller->chip_num;
        			}
        			
        			if((dev_para->option & DEV_MULTI_PLANE_MODE) && (flash->option & NAND_MULTI_PLANE_MODE)){
        				phydev->option |= DEV_MULTI_PLANE_MODE;		
        				
        				phydev->writesize <<= 1;			
        				phydev->erasesize <<= 1;				
        				phydev->oobavail <<= 1;				
        			}

        			phydev->writesize_shift = ffs(phydev->writesize) - 1;		
        			phydev->erasesize_shift = ffs(phydev->erasesize) -1;

        			if(boot_flag == 1){
        				if(i == 1){
        					offset =start_blk = 0;
        					total_blk = RESERVED_BLOCK_CNT;
        					memset(ops_para, 0, sizeof(struct chip_ops_para));
        					do{    
        						ops_para->page_addr =((((unsigned)(((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) - ((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) \
        							% controller->chip_num) /controller->chip_num) + tmp_blk -tmp_blk/controller->chip_num) * pages_per_blk);
        						ops_para->chipnr = ((unsigned) (offset >> tmp_erase_shift)) % controller->chip_num;
        						ret = operation->block_isbad(aml_chip);
        						if (ret == NAND_BLOCK_FACTORY_BAD){
        							offset += flash->blocksize;  
        							continue;
        						}
        						start_blk++;
        						offset += flash->blocksize; 
        					} while (start_blk < total_blk);		
        					total_blk = ((((unsigned) (offset>>tmp_erase_shift)) - 1)/(controller->chip_num*plane_num)+ 1)* (controller->chip_num*plane_num);
        					aml_nand_dbg("total_blk =%d",total_blk);
        					aml_nand_dbg(" phydev_pre->size =%llx", phydev_pre->size);
        					phydev->offset = total_blk * flash->blocksize + phydev_pre->size;
        					aml_nand_dbg("phydev->offset =%llx",phydev->offset);
        				}
        				else{
        					if((phydev_pre->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
        						phydev_pre_size =  phydev_pre->size << 1; 
        					}else{
        						phydev_pre_size = phydev_pre->size;
        					}
        					phydev->offset = phydev_pre->offset + phydev_pre_size;			
        				}
        			}
        			else{		
        				if(phydev_pre == NULL){
        					offset =start_blk = 0;
        					total_blk = RESERVED_BLOCK_CNT;
        					memset(ops_para, 0, sizeof(struct chip_ops_para));
        					do{    
        						ops_para->page_addr =((((unsigned)(((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) - ((unsigned)(offset>>tmp_erase_shift)+ tmp_blk) \
        							% controller->chip_num) /controller->chip_num) + tmp_blk -tmp_blk/controller->chip_num) * pages_per_blk);
        						ops_para->chipnr = ((unsigned) (offset >> tmp_erase_shift)) % controller->chip_num;
        						ret = operation->block_isbad(aml_chip);
        						if (ret == NAND_BLOCK_FACTORY_BAD){
        							offset += flash->blocksize;  
        							continue;
        						}
        						start_blk++;
        						offset += flash->blocksize; 
        					} while (start_blk < total_blk);		
        					total_blk = ((((unsigned) (offset>>tmp_erase_shift)) - 1)/(controller->chip_num*plane_num)+ 1)* (controller->chip_num*plane_num);
        					aml_nand_dbg("total_blk =%d",total_blk);
        					phydev->offset = total_blk * flash->blocksize;
        					aml_nand_dbg("phydev->offset =%llx",phydev->offset);
        				}
        				else{
        					if((phydev_pre->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
        						phydev_pre_size =  phydev_pre->size << 1; 
        					}else{
        						phydev_pre_size = phydev_pre->size;
        					}
        					phydev->offset = phydev_pre->offset + phydev_pre_size;			
        				}
        			}				

        			if(i != (config->dev_num -1)){
        					start_blk = 0;
        					if(boot_flag == 1){
        						if(i==1){
        								tmp_offset = offset = phydev->offset;
        							}
        							else{
        								tmp_offset = offset = phydev_pre->offset + phydev_pre->size;
        							}
        					}else if(boot_flag == 0){
        						if(i==0){
        							tmp_offset = offset	= phydev->offset;
        						}else{
        							tmp_offset = offset = phydev_pre->offset + phydev_pre->size;
        						}
        					}
        					aml_nand_dbg("offset = %llx, %llx",offset,tmp_erase_shift);
        					if(!is_phydev_off_adjust()){
        						total_blk = dev_size  >> tmp_erase_shift;
        					}else{
        						aml_nand_msg("nand adjust phy offset : block %d",ADJUST_BLOCK_NUM);
        						total_blk = dev_size  >> phydev->erasesize_shift;
        						#ifdef CONFIG_NAND_AML_M8B						
        						total_blk = total_blk + (total_blk + ADJUST_PART_SIZE - 1) /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
        						#else
        						if(IS_MESON_M8_CPU) //M8
        						{
        							if(phydev->erasesize >= 0x400000){
        							total_blk = total_blk + (total_blk + ADJUST_PART_SIZE - 1) /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
        							}
        							else{
        							total_blk = total_blk + total_blk /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
        							}
        						}else{//M8M2
        							total_blk = total_blk + (total_blk + ADJUST_PART_SIZE - 1) /ADJUST_PART_SIZE + ADJUST_BLOCK_NUM;
        						}
        						#endif
        					}
        					memset(ops_para, 0, sizeof(struct chip_ops_para));
        					ops_para->option = phydev->option;
        					if(!is_phydev_off_adjust()){
        					do{
        						ops_para->page_addr = ((((unsigned)((((unsigned)(offset >> tmp_erase_shift))) - (((unsigned)(offset >> tmp_erase_shift))) \
        							% controller->chip_num) /controller->chip_num) + tmp_blk -tmp_blk /controller->chip_num ) * pages_per_blk);
        						ops_para->chipnr =((unsigned) (offset >>tmp_erase_shift)) % controller->chip_num;	
        						ret = operation->block_isbad(aml_chip);
        						if (ret == NAND_BLOCK_FACTORY_BAD){
        							offset += flash->blocksize;
        							continue;
        						}
        						start_blk++;
        						offset += flash->blocksize;
        					} while (start_blk < total_blk);
        					total_blk = ((((unsigned) (offset >> phydev->erasesize_shift)) - 1)/(controller->chip_num*plane_num) + 1)* (controller->chip_num*plane_num);
        					aml_nand_dbg("total_blk =%d",total_blk);
        					phydev->size = ((uint64_t)total_blk*(uint64_t)phydev->erasesize);
        					}else{
        						do{
        							ops_para->page_addr = ((unsigned)(offset >> phydev->writesize_shift)) ;
        							//ops_para->chipnr =((unsigned) (offset>>phydev->erasesize_shift)) % controller->chip_num;
        							ret = operation->block_isbad(aml_chip); 
        							if (ret == NAND_BLOCK_FACTORY_BAD){
        								offset += phydev->erasesize;
        								//aml_nand_msg("#### : offset %llx",offset);
        								continue;	
        							}
        							start_blk++;
        							offset += phydev->erasesize;
        						}while(start_blk < total_blk);
        						total_blk = ((((unsigned) ((offset -tmp_offset) >> phydev->erasesize_shift)) - 1)/(controller->chip_num*plane_num) + 1)* (controller->chip_num*plane_num);
        						phydev->size = ((uint64_t)total_blk*(uint64_t)phydev->erasesize);
                                printk("offset:%llx,tmp_offset:%llx,total_blk:%d,phydev->size:%llx\n",offset,tmp_offset,total_blk,phydev->size);
        				}				
        			}
        			else{
        				phydev->size = dev_size;
        			}
        			
        			if((phydev->offset + phydev->size) > chip_size){
        				aml_nand_msg("nand dev size is out of space");
        				ret = -NAND_FAILED; 		
        				goto exit_error0;
        			}
        			
#ifndef AML_NAND_UBOOT
        			phydev->suspend = phydev_suspend;
        			phydev->resume = phydev_resume;
#endif
        		}	

        		phydev_pre = phydev;
        		
        		if((dev_para->option & DEV_SLC_MODE) && (flash->option & NAND_CHIP_SLC_MODE) && (!(phydev->option & DEV_MULTI_PLANE_MODE ))){
        			phydev->option |= DEV_SLC_MODE;	

        			phydev->erasesize >>= 1;
        			phydev->erasesize_shift = ffs(phydev->erasesize) -1;
        			phydev->size >>=1;
        			aml_nand_msg(" DEV %s enable SLC mode",phydev->name);
        		}
    	        aml_chip->phy_part_ptr->partition[i].phy_off = phydev->offset;
                aml_chip->phy_part_ptr->partition[i].phy_len = phydev->size;
                printk("%s,%d,phydev->offset=%llx,phydev->size=%llx\n",__func__,__LINE__,phydev->offset,phydev->size);
                if(dev_para->nr_partitions){
        			for(k=0; k<dev_para->nr_partitions; k++){
        				partition = &(dev_para->partitions[k]);
        				aml_chip->phy_part_ptr->partition[i].logic_len += partition->size;
        			}
                    printk("%s,%d,logic size:%llx\n",__func__,__LINE__,aml_chip->phy_part_ptr->partition[i].logic_len);
                }else{
                    aml_chip->phy_part_ptr->partition[i].logic_len = dev_para->size;
                    printk("%s,%d,logic size:%llx\n",__func__,__LINE__,aml_chip->phy_part_ptr->partition[i].logic_len);
                }
        		list_add_tail(&phydev->list, &nphy_dev_list);

        		
#ifdef AML_NAND_DBG		
        		amldev_dumpinfo(phydev);
#endif		
        		aml_nand_dbg("####: %s :phydev->offset  = %llx",phydev_pre->name,phydev_pre->offset);	
        		aml_nand_dbg("####: %s :phydev->size = %llx",phydev_pre->name,phydev_pre->size);
        		aml_nand_dbg("####: %s :phydev->writesize = %x",phydev_pre->name,phydev_pre->writesize);	
        		aml_nand_dbg("####: %s :phydev->erasesize = %x",phydev_pre->name,phydev_pre->erasesize);
        		
        	}
            aml_chip->phy_part_ptr->dev_num = config->dev_num;
            aml_chip->phy_part_ptr->crc = aml_info_checksum((unsigned char *)aml_chip->phy_part_ptr->partition,(MAX_DEVICE_NUM*sizeof(struct _phy_partition)));
            ret = amlnand_save_info_by_name(aml_chip, (unsigned char *)&(aml_chip->nand_phy_partition),(unsigned char *)aml_chip->phy_part_ptr,PHY_PARTITION_HEAD_MAGIC, sizeof(struct phy_partition_info));
            if(ret < 0){
                aml_nand_msg("save nand phy partition failed and ret:%d",ret);            
                goto exit_error0;
            }
        }
    }
	show_phydev_info();

#if 0
   	phydev = NULL;
	list_for_each_entry(phydev,&nphy_dev_list,list){
		if(phydev!=NULL){
            aml_nand_dbg("----------------------------------------------------------------------------------------------------\n");
        	aml_nand_dbg("name:%s, offset:%llx, size:%llx, option:%x", \
		phydev->name, phydev->offset, phydev->size, phydev->option);
            aml_nand_dbg("erasesize:%x, writesize:%x, oobavail:%x, erasesize_shift:%x, writesize_shift:%d", \
		phydev->erasesize, phydev->writesize, phydev->oobavail, phydev->erasesize_shift, phydev->writesize_shift);
            aml_nand_dbg(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
            relative_offset = 0;
            bad_blk_cnt = 0;
            devops = &(phydev->ops);
            memset(bad_blk,0,128*sizeof(uint64_t));
            do{
                memset(devops, 0x0, sizeof(struct phydev_ops));
                memset(devops, 0x0, sizeof(struct phydev_ops));
                devops->addr = relative_offset;
                devops->len = phydev->erasesize;
                devops->datbuf = NULL;
                devops->oobbuf = NULL;
                devops->mode = NAND_HW_ECC;
                //aml_nand_msg("relative_offset = %x, devops->addr = %x, shoule be equal",relative_offset,devops->addr);
                ret = nand_block_isbad(phydev);
                if (ret == NAND_BLOCK_USED_BAD){
                    if(bad_blk_cnt < 128){
                      bad_blk[bad_blk_cnt] = relative_offset;
                      bad_blk_cnt++;
                    }
                }
                relative_offset += phydev->erasesize;
            }while(relative_offset < phydev->size);
            aml_nand_msg("bad block count = %d\n",bad_blk_cnt);
        	if((bad_blk_cnt *32 > (phydev->size >> phydev->erasesize_shift))||(bad_blk_cnt>10)){
                aml_nand_dbg("Too many new bad blocks,try to repair...\n");
        		//ret = aml_repair_bbt(phydev,bad_blk,bad_blk_cnt);		
        	}
    	}
	}

#endif
	if(bad_blk != NULL){
        aml_nand_free(bad_blk);
	}

	return NAND_SUCCESS;
	
exit_error0:
	if(bad_blk != NULL){
        aml_nand_free(bad_blk);
	}
	return ret;
}

#ifdef AML_NAND_UBOOT
void amlnf_phy_exit()
{

	struct amlnand_phydev *phydev = NULL;
	struct amlnand_chip *aml_chip = NULL;
	struct list_head *entry;
	int time= 0;
	list_for_each_entry(phydev,&nphy_dev_list,list){

		if(phydev){
			if(time == 0){
				aml_chip = (struct amlnand_chip *)phydev->priv;
				if(aml_chip){
					if(aml_chip->block_status){
						kfree(aml_chip->block_status);
						aml_chip->block_status = NULL;
					}
					if(aml_chip->user_page_buf){
						kfree(aml_chip->user_page_buf);
						aml_chip->user_page_buf = NULL;
					}
					if(aml_chip->user_oob_buf){
						kfree(aml_chip->user_oob_buf);
						aml_chip->user_oob_buf = NULL;
					}
					if (aml_chip->shipped_bbt_ptr) {
						kfree(aml_chip->shipped_bbt_ptr);
						aml_chip->shipped_bbt_ptr = NULL;
					}
					if (aml_chip->config_ptr) {
						kfree(aml_chip->config_ptr);
						aml_chip->config_ptr = NULL;
					}
				}
				time++;
			}
		}
	}
	
	list_del_init(&nphy_dev_list);

}
#endif
