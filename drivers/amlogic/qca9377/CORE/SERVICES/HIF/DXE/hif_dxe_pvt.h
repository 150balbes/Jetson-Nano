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
 * @file hif_dxe_pvt.h
 * @brief Defines Software Structures used by DXE
 * @details
 *  This file Provides the HIF DXE Software Structures.
 *  DXE software module communicates with the RIVA DXE HW block for data path which
 *  is a DMA engine to transfer Data from Host DDR to Target DDR.
 */
#ifndef _HIF_DXE_PVT_H_
#define _HIF_DXE_PVT_H_
#include <adf_os_types.h>
#include <adf_os_atomic.h>
#include <adf_os_lock.h>
#include <adf_nbuf.h> /* adf_nbuf_t */
#include <adf_os_timer.h>
#include "hif_dxe.h"
#include "hif_dxe_os.h"


/* DXE Descriptor Endian swap macro */
#ifdef WLANDXE_ENDIAN_SWAP_ENABLE
#define WLANDXE_U32_SWAP_ENDIAN(a) (((a & 0x000000FF) << 24) |    \
                                    ((a & 0x0000FF00) << 8)  |    \
                                    ((a & 0x00FF0000) >> 8)  |    \
                                    ((a & 0xFF000000) >> 24))
#else
/* If DXE HW does not need endian swap, DO NOTHING */
#define WLANDXE_U32_SWAP_ENDIAN(a) (a)
#endif /* WLANDXE_ENDIAN_SWAP_ENABLE */

#define TX_INT_SELECT(ch)       (1L << ch)
#define RX_INT_SELECT(ch)       (1L << (ch + 16))

/** Map logical channel to DXE channel */
#define CH_TX_LOW_PRI   (0)
#define CH_TX_HIGH_PRI  (4)
#define CH_RX_LOW_PRI   (1)
#define CH_RX_HIGH_PRI  (3)

#define HIF_DXE_TX_PENDING_DEBUG_THRESHOLD 5
// Number of resources which are allocated from ACPI table for RIVA
#define HIFDXE_NUM_OS_MEM_RES       (1) // memory 0x03000000 - 0x04000000
#define HIFDXE_NUM_INTR_RES      (2) // TX complete(235) and RX ready(234)

#define HIFDXE_RX_INTERRUPT_PRO_MASK    0x20
#define HIFDXE_TX_INTERRUPT_PRO_MASK    0x40
#define HIFDXE_RX_INTERRUPT_PRO_UNMASK  0x5F
#define HIFDXE_TX_INTERRUPT_PRO_UNMASK  0x3F
#define HIFDXE_TX_CHANNELS_EN_BIT_MASK  0x11
//FIXME_RT  Check This Wait Duration
#define HIFDXE_MAX_TX_CH_DISABLE_WAIT   500    /* MAX 500 usec wait */


/*
 * The DXE low-priority channel should be able to accept at least
 * as many fragm ents as would be present in a maximum-sized A-MPDU
 * (64 frames x number of fragments per frame).
 * A small additional margin is provded.
 */
#define HIFDXE_TX_MAX_FRMS 64
#define HIFDXE_TX_AVG_FRAGS_PER_FRAME 5 /* avg for windows */
#define HIFDXE_TX_LO_PRI_RES_MARGIN 30
#define HIFDXE_TX_LO_PRI_RES_NUM (HIFDXE_TX_MAX_FRMS * HIFDXE_TX_AVG_FRAGS_PER_FRAME + HIFDXE_TX_LO_PRI_RES_MARGIN)


/*The maximum number of packets that can be chained in dxe for the HI
  priority channel */
#define HIFDXE_TX_HI_PRI_RES_NUM 10

#define WLANDXE_TX_LOW_RES_THRESHOLD     (5)

#define HIF_DXE_NUM_RX_CHANNEL                  2
#define HIF_DXE_RX_BUFFER_SIZE                  2612
#define HIF_DXE_RX_RING_REFILL_RETRY_TIME_MS    50

/*----------------------------------------------------------------------------
 *  Type Declarations
 * -------------------------------------------------------------------------*/
/* DMA Channel Q handle Method type
  * Linear handle or circular */
typedef enum
{
   WLANDXE_CHANNEL_HANDLE_LINEAR,
   WLANDXE_CHANNEL_HANDLE_CIRCULA
}WLANDXE_ChannelHandleType;

typedef enum
{
   WLANDXE_TX_COMP_INT_LR_THRESHOLD,
   WLANDXE_TX_COMP_INT_PER_K_FRAMES,
   WLANDXE_TX_COMP_INT_TIMER
} WLANDXE_TXCompIntEnableType;

typedef enum
{
   WLANDXE_SHORT_DESCRIPTOR,
   WLANDXE_LONG_DESCRIPTOR
} WLANDXE_DescriptorType;

typedef enum
{
   WLANDXE_DMA_CHANNEL_0,
   WLANDXE_DMA_CHANNEL_1,
   WLANDXE_DMA_CHANNEL_2,
   WLANDXE_DMA_CHANNEL_3,
   WLANDXE_DMA_CHANNEL_4,
   WLANDXE_DMA_CHANNEL_5,
   WLANDXE_DMA_CHANNEL_6,
   WLANDXE_DMA_CHANNEL_MAX
} WLANDXE_DMAChannelType;

typedef struct _WLANDXE_DescCtrlBlkType {
   struct _WLANDXE_DescCtrlBlkType *nextCtrlBlk;
   adf_nbuf_t                      xfrFrame;
   volatile WLANDXE_DescType       *linkedDesc;
   adf_os_dma_addr_t                linkedDescPhyAddr;
   A_UINT32                         ctrlBlkOrder;
} WLANDXE_DescCtrlBlkType;

typedef struct
{
   /* Q handle method, linear or ring */
   WLANDXE_ChannelHandleType       queueMethod;

   /* Number of descriptors for DXE that can be queued for transfer at one time */
   u_int32_t                      nDescs;

   /* Maximum number of receive buffers  of shared memory to use for this pipe */
   u_int32_t                      nRxBuffers;

   /* Reference WQ - for H2B and B2H only */
   u_int32_t                      refWQ;

   /* for usb only, endpoint info for CH_SADR or CH_DADR */
   u_int32_t                      refEP;

   /* H2B(Tx), B2H(Rx), H2H(SRAM<->HostMem R/W) */
   u_int32_t                      xfrType;

   /* Channel Priority 7(Highest) - 0(Lowest) */
   u_int32_t                      chPriority;

   /* 1 = BD attached to frames for this pipe */
   u_int8_t                       bdPresent;

   u_int32_t                      chk_size;

   u_int32_t                      bmuThdSel;

   /*  Added in Gen5 for Prefetch */
   u_int8_t                          useLower4G;

   u_int8_t                          useShortDescFmt;
   /* Till here inharited from GEN5 code */
   /* From now on, added for PRIMA  */
} WLANDXE_ChannelConfigType;

typedef struct
{
   u_int32_t                      chDXEBaseAddr;
   u_int32_t                      chDXEStatusRegAddr;
   u_int32_t                      chDXEDesclRegAddr;
   u_int32_t                      chDXEDeschRegAddr;
   u_int32_t                      chDXELstDesclRegAddr;
   u_int32_t                      chDXECtrlRegAddr;
   u_int32_t                      chDXESzRegAddr;
   u_int32_t                      chDXEDadrlRegAddr;
   u_int32_t                      chDXEDadrhRegAddr;
   u_int32_t                      chDXESadrlRegAddr;
   u_int32_t                      chDXESadrhRegAddr;
} WLANDXE_ChannelRegisterType;

typedef struct
{
   u_int32_t                      refWQ_swapped;
   u_int8_t                       chEnabled;
   u_int8_t                       chConfigured;
   u_int32_t                      channel;
   u_int32_t                      chk_size_mask;
   u_int32_t                      bmuThdSel_mask;
   u_int32_t                      cw_ctrl_read;
   u_int32_t                      cw_ctrl_write;
   u_int32_t                      cw_ctrl_write_valid;
   u_int32_t                      cw_ctrl_write_eop;
   u_int32_t                      cw_ctrl_write_eop_int;
   u_int32_t                      chan_mask;
   u_int32_t                      chan_mask_read_disable;
   u_int32_t                      intMask;
} WLANDXE_ChannelExConfigType;

typedef enum
{
   HIF_DXE_POWER_STATE_FULL,
   HIF_DXE_POWER_STATE_IMPS,
   HIF_DXE_POWER_STATE_BMPS,
   HIF_DXE_POWER_STATE_BMPS_PENDING,
   HIF_DXE_POWER_STATE_DOWN,
   HIF_DXE_POWER_STATE_MAX
} E_DXE_HOST_PS_TYPE;

typedef enum
{
   HIF_DXE_FW_POWER_STATE_ACTIVE,
   HIF_DXE_FW_POWER_STATE_IMPS_UNKNOWN,
   HIF_DXE_FW_POWER_STATE_BMPS_UNKNOWN,
   HIF_DXE_FW_POWER_STATE_DOWN_UNKNOWN,
   HIF_DXE_FW_POWER_STATE_MAX
} E_DXE_FIRMWARE_PS_TYPE;


typedef struct
{
    struct _S_HIFDXE_CONTEXT        *dxe_ctx;   // back pointer to DXE context
    E_HIFDXE_CHANNELTYPE            channelType;
    WLANDXE_DescCtrlBlkType        *headCtrlBlk;
    WLANDXE_DescCtrlBlkType        *tailCtrlBlk;
    struct
    {
        void *vaddr;
        adf_os_dma_addr_t paddr;
        adf_os_dma_mem_context(memctx);
    } descblk_mem_pool;
    void                            *ctrlblk_mem_pool;  //Memory Pool For per Descriptor Control Block
    volatile WLANDXE_DescType       *DescBottomLoc;
    adf_os_dma_addr_t               descBottomLocPhyAddr;
    u_int32_t                      numDesc;
    adf_os_atomic_t                 numFreeDesc;
    adf_os_atomic_t                 numRsvdDesc;
    u_int32_t                      maxFrameSize;
    adf_os_atomic_t                 numFragmentCurrentChain;
    u_int32_t                      numFramePrevious;
    u_int32_t                      numTotalFrame;
    adf_os_atomic_t                 tx_pkts_pending;
    adf_os_spinlock_t               dxeChannelLock;  //Per Channel Spin Lock for Synchronization at DISPATCH LEVEL
    u_int8_t                          hitLowResource;
    WLANDXE_ChannelConfigType       channelConfig;
    WLANDXE_ChannelRegisterType     channelRegister;
    WLANDXE_ChannelExConfigType     extraConfig;
    WLANDXE_DMAChannelType          assignedDMAChannel;
    u_int32_t                      txIntTracing;
    u_int32_t                      txCompIntTracing;
    u_int32_t                      errorIntCount;
    adf_os_atomic_t                 rx_refill_ref_cnt;
    u_int8_t                          rx_prev_intr_completed_frames;
    adf_os_timer_t                  rx_refill_retry_timer;
} WLANDXE_ChannelCBType;

typedef struct
{
   WLANDXE_TXCompIntEnableType     txIntEnable;
   u_int32_t                    txLowResourceThreshold_LoPriCh;
   u_int32_t                    txLowResourceThreshold_HiPriCh;
   u_int32_t                    rxLowResourceThreshold;
   u_int32_t                    txInterruptEnableFrameCount;
   u_int32_t                    txInterruptEnablePeriod;
} WLANDXE_TxCompIntConfigType;

typedef struct _S_HIFDXE_CONTEXT
{
    adf_os_device_t                 osdev;
    WLANDXE_ChannelCBType           dxeChannel[HIFDXE_CHANNEL_MAX];
    u_int8_t                        rxIntDisabledByIMPS;
    u_int8_t                        txIntDisabledByIMPS;
    u_int32_t                       dxeCookie;
    hif_dxe_oshandle                hif_os_handle;
    WLANDXE_TxCompIntConfigType     txCompInt;
    adf_os_atomic_t                 tx_pkts_pending;
    S_HIFDXE_CALLBACK               hif_client_cb;
    volatile E_DXE_HOST_PS_TYPE     hostPowerState;
    volatile E_DXE_FIRMWARE_PS_TYPE fwPowerState;
    adf_os_atomic_t                 ref_count;
    adf_os_atomic_t				tx_pk_count;


//Do Not Need These variables . Check and clean.
#ifdef FIXME_RT
   u_int32_t                      interruptPath;
   WLANDXE_SetPowerStateCbType    setPowerStateCb;
   u_int8_t                       ucTxMsgCnt;
   u_int16_t                      lastKickOffDxe;
   u_int32_t                      dxeFlushTxFrames;
#endif
} S_HIFDXE_CONTEXT;
#endif
