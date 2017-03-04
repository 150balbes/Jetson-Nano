/*
 * Copyright (c) 2012 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/**
 * @file hif_dxe_desc_pvt.h
 * @brief Defines DXE HW Descriptor Structure Format
 * @details
 *  This file Provides the HIF DXE HW Descriptor Definitions/ DMA Channel Structure / DMA Control And Data Descriptors.
 *  DXE software module communicates with the RIVA DXE HW block for data path which
 *  is a DMA engine to transfer Data from Host DDR to Target DDR.
 */
#ifndef _HIF_DXE_DESC_PVT_H_
#define _HIF_DXE_DESC_PVT_H_
#include <athdefs.h>
#include <adf_os_atomic.h>
/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/
#define WLANDXE_CTXT_COOKIE              0xC00CC111


/* From here RIVA DXE register information
 * This is temporary definition location to make compile and unit test
 * If official msmreg.h integrated, this part will be eliminated */
/* Start with base address */


#define WLANDXE_BMU_BD_THRES0_MASK       0x000007ff

#ifdef QCA_ISOC_PRONTO

#define WLANDXE_CCU_DXE_INT_SELECT       0xfb2050dc
#define WLANDXE_CCU_DXE_INT_SELECT_STAT  0xfb2050e0
#define WLANDXE_CCU_ASIC_INT_ENABLE      0xfb2050e4
#define WLANDXE_RIVA_BASE_ADDRESS        0xfb000000
#define WLANDXE_BMU_AVAILABLE_BD_PDU     0xfb080084
#define WLANDXE_BMU_PDU_THRES0           0xfb080014
#else

#define WLANDXE_CCU_DXE_INT_SELECT       0x03200b10
#define WLANDXE_CCU_DXE_INT_SELECT_STAT  0x03200b14
#define WLANDXE_CCU_ASIC_INT_ENABLE      0x03200b18
#define WLANDXE_RIVA_BASE_ADDRESS        0x03000000
#define WLANDXE_BMU_AVAILABLE_BD_PDU     0x03080084
#define WLANDXE_BMU_PDU_THRES0           0x03080014
#endif //QCA_ISOC_PRONTO

#define WLANDXE_REGISTER_BASE_ADDRESS    WLANDXE_RIVA_BASE_ADDRESS + 0x202000

/* Common over the channels register addresses */
#define WALNDEX_DMA_CSR_ADDRESS          WLANDXE_REGISTER_BASE_ADDRESS + 0x00
#define WALNDEX_DMA_ENCH_ADDRESS         WLANDXE_REGISTER_BASE_ADDRESS + 0x04
#define WALNDEX_DMA_CH_EN_ADDRESS        WLANDXE_REGISTER_BASE_ADDRESS + 0x08
#define WALNDEX_DMA_CH_DONE_ADDRESS      WLANDXE_REGISTER_BASE_ADDRESS + 0x0C
#define WALNDEX_DMA_CH_ERR_ADDRESS       WLANDXE_REGISTER_BASE_ADDRESS + 0x10
#define WALNDEX_DMA_CH_STOP_ADDRESS      WLANDXE_REGISTER_BASE_ADDRESS + 0x14

/* Interrupt Control register address */
#define WLANDXE_INT_MASK_REG_ADDRESS     WLANDXE_REGISTER_BASE_ADDRESS + 0x18
#define WLANDXE_INT_SRC_MSKD_ADDRESS     WLANDXE_REGISTER_BASE_ADDRESS + 0x1C
#define WLANDXE_INT_SRC_RAW_ADDRESS      WLANDXE_REGISTER_BASE_ADDRESS + 0x20
#define WLANDXE_INT_ED_SRC_ADDRESS       WLANDXE_REGISTER_BASE_ADDRESS + 0x24
#define WLANDXE_INT_DONE_SRC_ADDRESS     WLANDXE_REGISTER_BASE_ADDRESS + 0x28
#define WLANDXE_INT_ERR_SRC_ADDRESS      WLANDXE_REGISTER_BASE_ADDRESS + 0x2C
#define WLANDXE_INT_CLR_ADDRESS          WLANDXE_REGISTER_BASE_ADDRESS + 0x30
#define WLANDXE_INT_ED_CLR_ADDRESS       WLANDXE_REGISTER_BASE_ADDRESS + 0x34
#define WLANDXE_INT_DONE_CLR_ADDRESS     WLANDXE_REGISTER_BASE_ADDRESS + 0x38
#define WLANDXE_INT_ERR_CLR_ADDRESS      WLANDXE_REGISTER_BASE_ADDRESS + 0x3C

#define WLANDXE_DMA_CH_PRES_ADDRESS      WLANDXE_REGISTER_BASE_ADDRESS + 0x40
#define WLANDXE_ARB_CH_MSK_CLR_ADDRRESS  WLANDXE_REGISTER_BASE_ADDRESS + 0x74

/* Channel Counter register */
#define WLANDXE_DMA_COUNTER_0            WLANDXE_REGISTER_BASE_ADDRESS + 0x200
#define WLANDXE_DMA_COUNTER_1            WLANDXE_REGISTER_BASE_ADDRESS + 0x204
#define WLANDXE_DMA_COUNTER_2            WLANDXE_REGISTER_BASE_ADDRESS + 0x208
#define WLANDXE_DMA_COUNTER_3            WLANDXE_REGISTER_BASE_ADDRESS + 0x20C
#define WLANDXE_DMA_COUNTER_4            WLANDXE_REGISTER_BASE_ADDRESS + 0x210
#define WLANDXE_DMA_COUNTER_5            WLANDXE_REGISTER_BASE_ADDRESS + 0x214
#define WLANDXE_DMA_COUNTER_6            WLANDXE_REGISTER_BASE_ADDRESS + 0x218

#define WLANDXE_ENGINE_STAT_ADDRESS      WLANDXE_REGISTER_BASE_ADDRESS + 0x64
#define WLANDXE_BMU_SB_QDAT_AV_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x5c

/* Channel Base address */
#define WLANDXE_DMA_CHAN0_BASE_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x400
#define WLANDXE_DMA_CHAN1_BASE_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x440
#define WLANDXE_DMA_CHAN2_BASE_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x480
#define WLANDXE_DMA_CHAN3_BASE_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x4C0
#define WLANDXE_DMA_CHAN4_BASE_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x500
#define WLANDXE_DMA_CHAN5_BASE_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x540
#define WLANDXE_DMA_CHAN6_BASE_ADDRESS   WLANDXE_REGISTER_BASE_ADDRESS + 0x580

/* Channel specific register offset */
#define WLANDXE_DMA_CH_CTRL_REG          0x0000
#define WLANDXE_DMA_CH_STATUS_REG        0x0004
#define WLANDXE_DMA_CH_SZ_REG            0x0008
#define WLANDXE_DMA_CH_SADRL_REG         0x000C
#define WLANDXE_DMA_CH_SADRH_REG         0x0010
#define WLANDXE_DMA_CH_DADRL_REG         0x0014
#define WLANDXE_DMA_CH_DADRH_REG         0x0018
#define WLANDXE_DMA_CH_DESCL_REG         0x001C
#define WLANDXE_DMA_CH_DESCH_REG         0x0020
#define WLANDXE_DMA_CH_LST_DESCL_REG     0x0024
#define WLANDXE_DMA_CH_LST_DESCH_REG     0x0028
#define WLANDXE_DMA_CH_BD_REG            0x002C
#define WLANDXE_DMA_CH_HEAD_REG          0x0030
#define WLANDXE_DMA_CH_TAIL_REG          0x0034
#define WLANDXE_DMA_CH_PDU_REG           0x0038
#define WLANDXE_DMA_CH_TSTMP_REG         0x003C

/* Common CSR Register Contorol mask and offset */
#define WLANDXE_DMA_CSR_RESERVED_MASK         0xFFFE0000
#define WLANDXE_DMA_CSR_RESERVED_OFFSET       0x11
#define WLANDXE_DMA_CSR_RESERVED_DEFAULT      0x0

#define WLANDXE_DMA_CSR_H2H_SYNC_EN_MASK      0x10000
#define WLANDXE_DMA_CSR_H2H_SYNC_EN_OFFSET    0x10
#define WLANDXE_DMA_CSR_H2H_SYNC_EN_DEFAULT   0x0

#define WLANDXE_DMA_CSR_PAUSED_MASK           0x8000
#define WLANDXE_DMA_CSR_PAUSED_OFFSET         0xF
#define WLANDXE_DMA_CSR_PAUSED_DEFAULT        0x0

#define WLANDXE_DMA_CSR_ECTR_EN_MASK          0x4000
#define WLANDXE_DMA_CSR_ECTR_EN_OFFSET        0xE
#define WLANDXE_DMA_CSR_ECTR_EN_DEFAULT       0x4000

#define WLANDXE_DMA_CSR_B2H_TSTMP_OFF_MASK    0x3E00
#define WLANDXE_DMA_CSR_B2H_TSTMP_OFF_OFFSET  0x9
#define WLANDXE_DMA_CSR_B2H_TSTMP_OFF_DEFAULT 0xE00

#define WLANDXE_DMA_CSR_H2B_TSTMP_OFF_MASK    0x1F0
#define WLANDXE_DMA_CSR_H2B_TSTMP_OFF_OFFSET  0x4
#define WLANDXE_DMA_CSR_H2B_TSTMP_OFF_DEFAULT 0x50

#define WLANDXE_DMA_CSR_TSTMP_EN_MASK         0x8
#define WLANDXE_DMA_CSR_TSTMP_EN_OFFSET       0x3
#define WLANDXE_DMA_CSR_TSTMP_EN_DEFAULT      0x0

#define WLANDXE_DMA_CSR_RESET_MASK            0x4
#define WLANDXE_DMA_CSR_RESET_OFFSET          0x2
#define WLANDXE_DMA_CSR_RESET_DEFAULT         0x0

#define WLANDXE_DMA_CSR_PAUSE_MASK            0x2
#define WLANDXE_DMA_CSR_PAUSE_OFFSET          0x1
#define WLANDXE_DMA_CSR_PAUSE_DEFAULT         0x0

#define WLANDXE_DMA_CSR_EN_MASK               0x1
#define WLANDXE_DMA_CSR_EN_OFFSET             0x0
#define WLANDXE_DMA_CSR_EN_DEFAULT            0x0
#define WLANDXE_DMA_CSR_DEFAULT               0x4E50

/* Channel CTRL Register Control mask and offset */
#define WLANDXE_CH_CTRL_RSVD_MASK             0x80000000
#define WLANDXE_CH_CTRL_RSVD_OFFSET           0x1F
#define WLANDXE_CH_CTRL_RSVD_DEFAULT          0x0

#define WLANDXE_CH_CTRL_SWAP_MASK             0x80000000

#define WLANDXE_CH_CTRL_BDT_IDX_MASK          0x60000000
#define WLANDXE_CH_CTRL_BDT_IDX_OFFSET        0x1D
#define WLANDXE_CH_CTRL_BDT_IDX_DEFAULT       0x0

#define WLANDXE_CH_CTRL_DFMT_MASK             0x10000000
#define WLANDXE_CH_CTRL_DFMT_OFFSET           0x1C
#define WLANDXE_CH_CTRL_DFMT_DEFAULT          0x10000000
#define WLANDXE_CH_CTRL_DFMT_ESHORT           0x0
#define WLANDXE_CH_CTRL_DFMT_ELONG            0x1

#define WLANDXE_CH_CTRL_ABORT_MASK            0x8000000
#define WLANDXE_CH_CTRL_ABORT_OFFSET          0x1B
#define WLANDXE_CH_CTRL_ABORT_DEFAULT         0x0

#define WLANDXE_CH_CTRL_ENDIAN_MASK           0x4000000

#define WLANDXE_CH_CTRL_CTR_SEL_MASK          0x3C00000
#define WLANDXE_CH_CTRL_CTR_SEL_OFFSET        0x16
#define WLANDXE_CH_CTRL_CTR_SEL_DEFAULT       0x0

#define WLANDXE_CH_CTRL_EDVEN_MASK            0x200000
#define WLANDXE_CH_CTRL_EDVEN_OFFSET          0x15
#define WLANDXE_CH_CTRL_EDVEN_DEFAULT         0x0

#define WLANDXE_CH_CTRL_EDEN_MASK             0x100000
#define WLANDXE_CH_CTRL_EDEN_OFFSET           0x14
#define WLANDXE_CH_CTRL_EDEN_DEFAULT          0x0

#define WLANDXE_CH_CTRL_INE_DONE_MASK         0x80000
#define WLANDXE_CH_CTRL_INE_DONE_OFFSET       0x13
#define WLANDXE_CH_CTRL_INE_DONE_DEFAULT      0x0

#define WLANDXE_CH_CTRL_INE_ERR_MASK          0x40000
#define WLANDXE_CH_CTRL_INE_ERR_OFFSET        0x12
#define WLANDXE_CH_CTRL_INE_ERR_DEFAULT       0x0

#define WLANDXE_CH_CTRL_INE_ED_MASK           0x20000
#define WLANDXE_CH_CTRL_INE_ED_OFFSET         0x11
#define WLANDXE_CH_CTRL_INE_ED_DEFAULT        0x0

#define WLANDXE_CH_CTRL_STOP_MASK             0x10000
#define WLANDXE_CH_CTRL_STOP_OFFSET           0x10
#define WLANDXE_CH_CTRL_STOP_DEFAULT          0x0

#define WLANDXE_CH_CTRL_PRIO_MASK             0xE000
#define WLANDXE_CH_CTRL_PRIO_OFFSET           0xD
#define WLANDXE_CH_CTRL_PRIO_DEFAULT          0x0

#define WLANDXE_CH_CTRL_BTHLD_SEL_MASK        0x1E00
#define WLANDXE_CH_CTRL_BTHLD_SEL_OFFSET      0x9
#define WLANDXE_CH_CTRL_BTHLD_SEL_DEFAULT     0x600
#define WLANDXE_CH_CTRL_BTHLD_SEL_ERSVD0      0x0
#define WLANDXE_CH_CTRL_BTHLD_SEL_ERSVD1      0x1
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD2      0x2
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD3      0x3
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD4      0x4
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD5      0x5
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD6      0x6
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD7      0x7
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD8      0x8
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD9      0x9
#define WLANDXE_CH_CTRL_BTHLD_SEL_ETHLD10     0xA
#define WLANDXE_CH_CTRL_BTHLD_SEL_ERSVD11     0xB
#define WLANDXE_CH_CTRL_BTHLD_SEL_ERSVD12     0xC
#define WLANDXE_CH_CTRL_BTHLD_SEL_ERSVD13     0xD
#define WLANDXE_CH_CTRL_BTHLD_SEL_ERSVD14     0xE
#define WLANDXE_CH_CTRL_BTHLD_SEL_ERSVD15     0xF

#define WLANDXE_CH_CTRL_PDU_REL_MASK          0x100
#define WLANDXE_CH_CTRL_PDU_REL_OFFSET        0x8
#define WLANDXE_CH_CTRL_PDU_REL_DEFAULT       0x100
#define WLANDXE_CH_CTRL_PDU_REL_EKEEP         0x0
#define WLANDXE_CH_CTRL_PDU_REL_ERELEASE      0x1

#define WLANDXE_CH_CTRL_PIQ_MASK              0x80
#define WLANDXE_CH_CTRL_PIQ_OFFSET            0x7
#define WLANDXE_CH_CTRL_PIQ_DEFAULT           0x0
#define WLANDXE_CH_CTRL_PIQ_EFLAT             0x0
#define WLANDXE_CH_CTRL_PIQ_EQUEUE            0x1

#define WLANDXE_CH_CTRL_DIQ_MASK              0x40
#define WLANDXE_CH_CTRL_DIQ_OFFSET            0x6
#define WLANDXE_CH_CTRL_DIQ_DEFAULT           0x0
#define WLANDXE_CH_CTRL_DIQ_EFLAT             0x0
#define WLANDXE_CH_CTRL_DIQ_EQUEUE            0x1

#define WLANDXE_CH_CTRL_SIQ_MASK              0x20
#define WLANDXE_CH_CTRL_SIQ_OFFSET            0x5
#define WLANDXE_CH_CTRL_SIQ_DEFAULT           0x0
#define WLANDXE_CH_CTRL_SIQ_EFLAT             0x0
#define WLANDXE_CH_CTRL_SIQ_EQUEUE            0x1

#define WLANDXE_CH_CTRL_BDH_MASK              0x10
#define WLANDXE_CH_CTRL_BDH_OFFSET            0x4
#define WLANDXE_CH_CTRL_BDH_DEFAULT           0x0

#define WLANDXE_CH_CTRL_EOP_MASK              0x8
#define WLANDXE_CH_CTRL_EOP_OFFSET            0x3
#define WLANDXE_CH_CTRL_EOP_DEFAULT           0x8

#define WLANDXE_CH_CTRL_XTYPE_MASK            0x6
#define WLANDXE_CH_CTRL_XTYPE_OFFSET          0x1
#define WLANDXE_CH_CTRL_XTYPE_DEFAULT         0x0
#define WLANDXE_CH_CTRL_XTYPE_EH2H            0x0
#define WLANDXE_CH_CTRL_XTYPE_EB2B            0x1
#define WLANDXE_CH_CTRL_XTYPE_EH2B            0x2
#define WLANDXE_CH_CTRL_XTYPE_EB2H            0x3

#define WLANDXE_CH_CTRL_DONE_MASK             0x4

#define WLANDXE_CH_CTRL_ERR_MASK              0x20

#define WLANDXE_CH_CTRL_MASKED_MASK           0x8

#define WLANDXE_CH_CTRL_EN_MASK               0x1
#define WLANDXE_CH_CTRL_EN_OFFSET             0x0
#define WLANDXE_CH_CTRL_EN_DEFAULT            0x0
#define WLANDXE_CH_CTRL_DEFAULT               0x10000708


#define WLANDXE_DESC_CTRL_VALID          0x00000001
#define WLANDXE_DESC_CTRL_XTYPE_MASK     0x00000006
#define WLANDXE_DESC_CTRL_XTYPE_H2H      0x00000000
#define WLANDXE_DESC_CTRL_XTYPE_B2B      0x00000002
#define WLANDXE_DESC_CTRL_XTYPE_H2B      0x00000004
#define WLANDXE_DESC_CTRL_XTYPE_B2H      0x00000006
#define WLANDXE_DESC_CTRL_EOP            0x00000008
#define WLANDXE_DESC_CTRL_BDH            0x00000010
#define WLANDXE_DESC_CTRL_SIQ            0x00000020
#define WLANDXE_DESC_CTRL_DIQ            0x00000040
#define WLANDXE_DESC_CTRL_PIQ            0x00000080
#define WLANDXE_DESC_CTRL_PDU_REL        0x00000100
#define WLANDXE_DESC_CTRL_BTHLD_SEL      0x00001E00
#define WLANDXE_DESC_CTRL_PRIO           0x0000E000
#define WLANDXE_DESC_CTRL_STOP           0x00010000
#define WLANDXE_DESC_CTRL_INT            0x00020000
#define WLANDXE_DESC_CTRL_BDT_SWAP       0x00100000
#define WLANDXE_DESC_CTRL_ENDIANNESS     0x00200000
#define WLANDXE_DESC_CTRL_DFMT           0x10000000
#define WLANDXE_DESC_CTRL_RSVD           0xfffc0000
/* CSR Register Control mask and offset */

#define WLANDXE_CH_STAT_INT_DONE_MASK   0x00008000
#define WLANDXE_CH_STAT_INT_ERR_MASK    0x00004000
#define WLANDXE_CH_STAT_INT_ED_MASK     0x00002000

#define WLANDXE_CH_STAT_MASKED_MASK     0x00000008
/* Till here RIVA DXE register information
 * This is temporary definition location to make compile and unit test
 * If official msmreg.h integrated, this part will be eliminated */

/* Interrupt control channel mask */
#define WLANDXE_INT_MASK_CHAN_0          0x00000001
#define WLANDXE_INT_MASK_CHAN_1          0x00000002
#define WLANDXE_INT_MASK_CHAN_2          0x00000004
#define WLANDXE_INT_MASK_CHAN_3          0x00000008
#define WLANDXE_INT_MASK_CHAN_4          0x00000010
#define WLANDXE_INT_MASK_CHAN_5          0x00000020
#define WLANDXE_INT_MASK_CHAN_6          0x00000040

/** DXE HW Long Descriptor format */
typedef struct
{
   adf_os_dma_addr_t             srcMemAddrL;
   adf_os_dma_addr_t             srcMemAddrH;
   adf_os_dma_addr_t             dstMemAddrL;
   adf_os_dma_addr_t             dstMemAddrH;
   adf_os_dma_addr_t             phyNextL;
   adf_os_dma_addr_t             phyNextH;
} WLANDXE_LongDesc;


/** DXE HW Short Descriptor format */
typedef struct tDXEShortDesc
{
   adf_os_dma_addr_t             srcMemAddrL;
   adf_os_dma_addr_t             dstMemAddrL;
   adf_os_dma_addr_t             phyNextL;
} WLANDXE_ShortDesc;


/* DXE Descriptor Data Type
  * Pick up from GEN5 */
//FIXME_RT Check if this Needs to be byte packed
typedef struct
{
   union
   {
      u_int32_t                   ctrl;
      u_int32_t                   valid          :1;     //0 = DMA stop, 1 = DMA continue with this descriptor
      u_int32_t                   transferType   :2;     //0 = Host to Host space
      u_int32_t                   eop            :1;     //End of Packet
      u_int32_t                   bdHandling     :1;          //if transferType = Host to BMU, then 0 means first 128 bytes contain BD, and 1 means create new empty BD
      u_int32_t                   siq            :1;     // SIQ
      u_int32_t                   diq            :1;     // DIQ
      u_int32_t                   pduRel         :1;     //0 = don't release BD and PDUs when done, 1 = release them
      u_int32_t                   bthldSel       :4;     //BMU Threshold Select
      u_int32_t                   prio           :3;     //Specifies the priority level to use for the transfer
      u_int32_t                   stopChannel    :1;     //1 = DMA stops processing further, channel requires re-enabling after this
      u_int32_t                   intr           :1;     //Interrupt on Descriptor Done
      u_int32_t                   rsvd           :1;     //reserved
      u_int32_t                   transferSize   :14;    //14 bits used - ignored for BMU transfers, only used for host to host transfers?
   } descCtrl;
   u_int32_t                      xfrSize;
   union
   {
      WLANDXE_LongDesc             dxe_long_desc;
      WLANDXE_ShortDesc            dxe_short_desc;
   }dxedesc;
} WLANDXE_DescType;

#endif
