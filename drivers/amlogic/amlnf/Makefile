#
# Makefile for the Amlogic NAND drivers.
#
#obj-$(CONFIG_AML_NEXT_GEN_NAND)		+= phy/
KBUILD_CFLAGS += -Werror=declaration-after-statement \
		 -Werror=maybe-uninitialized \
		 -Werror=parentheses \
		 -Werror=return-type \
		 -Werror=strict-prototypes \
		 -Werror=uninitialized \
		 -Werror=unused-function \
		 -Werror=unused-value \
		 -Werror=array-bounds \
		 -Werror=comment \
		 -Werror=format \
		 -Werror=missing-braces \
		 -Werror=declaration-after-statement \
		 -Werror=unused-label \
		 -Werror=unused-result \
		 -Werror=unused-value \
		 -Werror=unused-variable \
		 -Werror=builtin-macro-redefined



obj-$(CONFIG_AML_NEXT_GEN_NAND)	+= dev/        
obj-$(CONFIG_AML_NEXT_GEN_NAND)	+= ntd/
obj-$(CONFIG_AML_NEXT_GEN_NAND)	+= phy/
#obj-$(CONFIG_AML_NEXT_GEN_NAND)	+= nftl/
#obj-$(CONFIG_AML_NEXT_GEN_NAND)	+= block/


obj-$(CONFIG_AML_NFTL_NEW)	+=   aml_nftl_dev.o 
aml_nftl_dev-objs =
aml_nftl_dev-objs += block/aml_nftl_block.o
aml_nftl_dev-objs += block/aml_nftl_init.o
aml_nftl_dev-objs += block/aml_nftl_hw_interface.o
aml_nftl_dev-objs += nftl/aml_nftl_core_20160811.o


#bootimage building.
#aml_nftl_dev-objs += nftl/aml_nftl_phy.o       
#aml_nftl_dev-objs  += nftl/aml_nftl_list.o      
#aml_nftl_dev-objs  += nftl/aml_nftl_logic.o     
#aml_nftl_dev-objs  += nftl/aml_nftl_gc.o        
#aml_nftl_dev-objs  += nftl/aml_nftl_cache.o     
#aml_nftl_dev-objs  += nftl/aml_nftl_common.o    
#aml_nftl_dev-objs  += nftl/aml_nftl_create_list.o
#aml_nftl_dev-objs  += nftl/aml_nftl_start.o
#aml_nftl_dev-objs  += nftl/aml_nftl_interface.o
#aml_nftl_dev-objs  += nftl/aml_nftl_tbl_trans.o

