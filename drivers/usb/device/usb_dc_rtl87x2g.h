/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_DEVICE_USB_DC_RTL87X2G_H_
#define ZEPHYR_DRIVERS_USB_DEVICE_USB_DC_RTL87X2G_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_DW_DIEPCTL_MPS_64BYTES (0)
#define USB_DW_DIEPCTL_MPS_POS     (0)
#define USB_DW_DIEPCTL_MPS_MSAK    (0x3 << USB_DW_DIEPCTL_MPS_POS)

#define USB_DW_DOEPCTL_MPS_64BYTES (0)
#define USB_DW_DOEPCTL_MPS_POS     (0)
#define USB_DW_DOEPCTL_MPS_MSAK    (0x3 << USB_DW_DOEPCTL_MPS_POS)

#define QUAD_BS_HOST_READY  0x0
#define QUAD_BS_DMA_BUSY    0x1
#define QUAD_BS_DMA_DONE    0x2     // used to flag Tx desc avail
#define QUAD_BS_HOST_BUSY   0x3     // used to flag Rx desc avail, need to be manually set after handling Rx pkt

#define QUAD_RXSTS_SUCCESS  0x0
#define QUAD_RXSTS_BUFERR   0x3

#define QUAD_TXSTS_SUCCESS  0x0
#define QUAD_TXSTS_BUFFLUSH 0x1
#define QUAD_TXSTS_BUFERR   0x3


#define USB_DMA_SETUP_PKT_BYTES 8


#define USB_DW_DOEPINT_STSPHSERCVD           BIT(5)
#define USB_DW_DOEPINT_AHBERR                BIT(2)
#define USB_DW_DOEPINT_BNA                   BIT(9)


#define DWC_GAHBCFG_INT_DMA_BURST_INCR      (1)
#define USB_DW_GAHBCFG_HBSTLEN_POS          (1)
#define USB_DW_GAHBCFG_HBSTLEN_MASK         (0xf << USB_DW_GAHBCFG_HBSTLEN_POS)

#define USB_DW_GINTMSK_RXFLVMSK_POS         (4)
#define USB_DW_GINTMSK_RXFLVMSK_MASK        (1 << USB_DW_GINTMSK_RXFLVMSK_POS)

#define USB_DW_GUSBCFG_ULPI_UTMI_SEL_POS    (4)
#define USB_DW_GUSBCFG_ULPI_UTMI_SEL_MASK   (1 << USB_DW_GUSBCFG_ULPI_UTMI_SEL_POS)

#define USB_DW_GUSBCFG_TOUTCAL_ZERO         (0)
#define USB_DW_GUSBCFG_TOUTCAL_POS          (0)
#define USB_DW_GUSBCFG_TOUTCAL_MASK         (0x7 << USB_DW_GUSBCFG_TOUTCAL_POS)

#define USB_DW_GUSBCFG_USBTRDTIM_16BIT      (5)
#define USB_DW_GUSBCFG_USBTRDTIM_POS        (10)
#define USB_DW_GUSBCFG_USBTRDTIM_MASK       (0xf << USB_DW_GUSBCFG_USBTRDTIM_POS)

#define USB_DW_GINTMSK_MODEMISMATCH         BIT(1)

#define USB_DW_GINTSTS_SOFINTR_POS          (3)
#define USB_DW_GINTSTS_SOFINTR_MASK         (1 << USB_DW_GINTSTS_SOFINTR_POS)

#define USB_DW_GINTMSK_NPTXEMPTY            BIT(4)

#define USB_DW_DCFG_DESCDMA_POS             (23)
#define USB_DW_DCFG_DESCDMA_MASK            (1 << USB_DW_DCFG_DESCDMA_POS)

#define USB_DW_DCFG_NZSTSOUTHSHK_POS        (2)
#define USB_DW_DCFG_NZSTSOUTHSHK_MASK       (1 << USB_DW_DCFG_NZSTSOUTHSHK_POS)

#define DWC_DCFG_FRAME_INTERVAL_80          (0)
#define USB_DW_DCFG_PERFRINT_POS            (11)
#define USB_DW_DCFG_PERFRINT_MASK           (0x3 << USB_DW_DCFG_PERFRINT_POS)

#define USB_DW_DCTL_IGNRFRMNUM_POS          (15)
#define USB_DW_DCTL_IGNRFRMNUM_MASK         (1 << USB_DW_DCTL_IGNRFRMNUM_POS)
#define USB_DW_DCTL_RMTWKUPSIG_POS          (0)
#define USB_DW_DCTL_RMTWKUPSIG_MASK         (1 << USB_DW_DCTL_RMTWKUPSIG_POS)

#define USB_DW_DAINTMSK_INEPMSK0_POS        (0)
#define USB_DW_DAINTMSK_INEPMSK0_MASK       (1 << USB_DW_DAINTMSK_INEPMSK0_POS)

#define USB_DW_DAINTMSK_ONEPMSK0_POS        (16)
#define USB_DW_DAINTMSK_ONEPMSK0_MASK       (1 << USB_DW_DAINTMSK_ONEPMSK0_POS)

#define USB_DW_DOEPMSK_XFERCOMPLMSK_POS     (0)
#define USB_DW_DOEPMSK_XFERCOMPLMSK_MSAK    (1 << USB_DW_DOEPMSK_XFERCOMPLMSK_POS)
#define USB_DW_DOEPMSK_AHBERRMSK_POS        (2)
#define USB_DW_DOEPMSK_AHBERRMSK_MASK       (1 << USB_DW_DOEPMSK_AHBERRMSK_POS)
#define USB_DW_DOEPMSK_SETUPMSK_POS         (3)
#define USB_DW_DOEPMSK_SETUPMSK_MASK        (1 << USB_DW_DOEPMSK_SETUPMSK_POS)
#define USB_DW_DOEPMSK_STSPHSERCVDMSK_POS   (5)
#define USB_DW_DOEPMSK_STSPHSERCVDMSK_MASK  (1 << USB_DW_DOEPMSK_STSPHSERCVDMSK_POS)
#define USB_DW_DOEPMSK_BNAOUTINTRMSK_POS    (9)
#define USB_DW_DOEPMSK_BNAOUTINTRMSK_MASK   (1 << USB_DW_DOEPMSK_BNAOUTINTRMSK_POS)

#define USB_DW_DIEPMSK_XFERCOMPLMSK_POS     (0)
#define USB_DW_DIEPMSK_XFERCOMPLMSK_MASK    (1 << USB_DW_DIEPMSK_XFERCOMPLMSK_POS)
#define USB_DW_DIEPMSK_AHBERRMSK_POS        (2)
#define USB_DW_DIEPMSK_AHBERRMSK_MASK       (1 << USB_DW_DIEPMSK_AHBERRMSK_POS)
#define USB_DW_DIEPMSK_BNAININTRMSK_POS     (9)
#define USB_DW_DIEPMSK_BNAININTRMSK_MASK    (1 << USB_DW_DIEPMSK_BNAININTRMSK_POS)

#define USB_DW_DSTS_SUSPSTS_POS             (0)
#define USB_DW_DSTS_SUSPSTS_MASK            (1 << USB_DW_DSTS_SUSPSTS_POS)

typedef struct
{
    union
    {
        struct
        {
            uint32_t RxBytes: 16; // A
            uint32_t Reserved0: 7;
            uint32_t MTRF: 1;
            uint32_t SR: 1;

            uint32_t InterruptOnComplete: 1; // A
            uint32_t ShortPacket: 1; // A
            uint32_t Last: 1; // A
            uint32_t ReceiveState: 2;
            uint32_t BufferState: 2; // AC
        } NonIso;
        struct
        {
            uint32_t RxBytes: 11;
            uint32_t Reserved0: 1;
            uint32_t FrameNumber: 11;
            uint32_t PID: 2;

            uint32_t InterruptOnComplete: 1; // A
            uint32_t ShortPacket: 1; // A
            uint32_t Last: 1; // A
            uint32_t ReceiveState: 2;
            uint32_t BufferState: 2; // AC
        } Iso;
    } Quad;
    uint32_t BufAddr;
} OutDmaDesc_t;

typedef struct
{
    union
    {
        struct
        {
            uint32_t TxBytes: 16; // A
            uint32_t Reserved0: 9;

            uint32_t InterruptOnComplete: 1; // A
            uint32_t ShortPacket: 1; // A
            uint32_t Last: 1; // A
            uint32_t TransmitState: 2;
            uint32_t BufferState: 2; // AC
        } NonIso;
        struct
        {
            uint32_t TxBytes: 12;
            uint32_t FrameNumber: 11;
            uint32_t PID: 2;

            uint32_t InterruptOnComplete: 1; // A
            uint32_t ShortPacket: 1; // A
            uint32_t Last: 1; // A
            uint32_t TransmitState: 2;
            uint32_t BufferState: 2; // AC
        } Iso;
    } Quad;
    uint32_t BufAddr;
} InDmaDesc_t;

typedef struct
{
    const void *pBuf;
    uint32_t BytesTotal;
    uint32_t BytesSent;
    uint32_t BytesSending;
    bool ToSendZlp;
} inxfer_t;

typedef struct
{
    const void *pBuf;
    uint32_t BytesTotal;
    uint32_t BytesRcving;
    uint32_t BytesRcved;
} outxfer_t;


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_USB_DEVICE_USB_DC_RTL87X2G_H_ */
