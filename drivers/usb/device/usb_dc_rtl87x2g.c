/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_usb

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/usb/usb_dc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>

#include <usb_dwc2_hw.h>

#include "rtl876x.h"
#include <soc.h>

#include "usb_dc_rtl87x2g.h"

#ifdef CONFIG_PM
#include "power_manager_unit_platform.h"
#endif

#include "trace.h"

#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
LOG_MODULE_REGISTER(usb_rtl87x2g, 0);

#define DBG_DIRECT_SHOW 0

/* FIXME: The actual number of endpoints should be obtained from GHWCFG4. */
enum usb_dwc2_in_ep_idx {
	USB_DWC2_IN_EP_0 = 0,
	USB_DWC2_IN_EP_1,
	USB_DWC2_IN_EP_2,
	USB_DWC2_IN_EP_3,
	USB_DWC2_IN_EP_4,
	USB_DWC2_IN_EP_5,
	USB_DWC2_IN_EP_NUM
};

/* FIXME: The actual number of endpoints should be obtained from GHWCFG2. */
enum usb_dwc2_out_ep_idx {
	USB_DWC2_OUT_EP_0 = 0,
	USB_DWC2_OUT_EP_1,
	USB_DWC2_OUT_EP_2,
	USB_DWC2_OUT_EP_3,
	USB_DWC2_OUT_EP_4,
	USB_DWC2_OUT_EP_5,
	USB_DWC2_OUT_EP_NUM
};

#if CONFIG_USB_DC_RTL87X2G_DMA
K_HEAP_DEFINE(ep_heap, CONFIG_USB_EP_HEAP_SIZE);

#endif

#define USB_DWC2_CORE_RST_TIMEOUT_US 10000

/* FIXME: The actual MPS depends on endpoint type and bus speed. */
#define DW_USB_MAX_PACKET_SIZE 64

/* Number of SETUP back-to-back packets */
#define USB_DWC2_SUP_CNT 1

/* Get Data FIFO access register */
#define USB_DWC2_EP_FIFO(base, idx) (*(uint32_t *)(POINTER_TO_UINT(base) + 0x1000 * (idx + 1)))

#define USB_DWC2_EP_TX_FIFO(base, idx) USB_DWC2_EP_FIFO(base, idx)

volatile static bool usb_power_is_on = false;
struct usb_dwc2_config {
	struct usb_dwc2_reg *const base;
	void (*irq_enable_func)(const struct device *dev);
	int (*pwr_on_func)(const struct device *dev);
};

/*
 * USB endpoint private structure.
 */
struct usb_ep_ctrl_prv {
	uint8_t ep_ena;
	uint8_t fifo_num;
	uint32_t fifo_size;
	uint16_t mps;          /* Max ep pkt size */
	usb_dc_ep_callback cb; /* Endpoint callback function */
	uint32_t data_len;
	uint32_t rsvd_tx_len;
	const uint8_t *rsvd_tx_buf;
};

static void usb_dwc2_isr_handler(const void *unused);

/*
 * USB controller private structure.
 */
struct usb_dwc2_ctrl_prv {
#if CONFIG_USB_DC_RTL87X2G_DMA
	OutDmaDesc_t *volatile aOutDmaDesc[USB_DWC2_OUT_EP_NUM];
	InDmaDesc_t *volatile aInDmaDesc[USB_DWC2_IN_EP_NUM];
	inxfer_t *aInXfer[USB_DWC2_IN_EP_NUM];
	outxfer_t *aOutXfer[USB_DWC2_OUT_EP_NUM];
	uint8_t *ep_rx_buf[USB_DWC2_OUT_EP_NUM];
#endif
	usb_dc_status_callback status_cb;
	enum usb_dc_status_code current_status;
	struct usb_ep_ctrl_prv in_ep_ctrl[USB_DWC2_IN_EP_NUM];
	struct usb_ep_ctrl_prv out_ep_ctrl[USB_DWC2_OUT_EP_NUM];
	uint8_t attached;
};

static const uint16_t kaTxFifoBytesTable[] = {
	CONFIG_USB_DC_RTL87X2G_EP0_TX_FIFO_SIZE, CONFIG_USB_DC_RTL87X2G_EP1_TX_FIFO_SIZE,
	CONFIG_USB_DC_RTL87X2G_EP2_TX_FIFO_SIZE, CONFIG_USB_DC_RTL87X2G_EP3_TX_FIFO_SIZE,
	CONFIG_USB_DC_RTL87X2G_EP4_TX_FIFO_SIZE, CONFIG_USB_DC_RTL87X2G_EP5_TX_FIFO_SIZE};

static uint32_t kaTxFifoAddrTable[6];

#define TX_FIFO_SIZE(epidx) (kaTxFifoBytesTable[epidx] / 4)

static void usb_dwc2_isr_handler(const void *unused);
static void usb_dwc2_resume_isr_handler(const void *unused);
static void usb_isr_suspend_enable(void);
extern int hal_usb_phy_power_on(void);
extern int32_t usb_rtk_resume_sequence(void);
extern void usb_rtk_disable_power_seq(void);

#define USB_DWC2_DEVICE_DEFINE(n)                                                                  \
                                                                                                   \
	static void usb_dwc2_irq_enable_func_##n(const struct device *dev)                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 0, irq), DT_INST_IRQ_BY_IDX(0, 0, priority),     \
			    usb_dwc2_isr_handler, 0, 0);                                           \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 1, irq), DT_INST_IRQ_BY_IDX(0, 1, priority),     \
			    usb_dwc2_resume_isr_handler, 0, 0);                                    \
		irq_enable(DT_INST_IRQ_BY_IDX(0, 0, irq));                                         \
		irq_enable(DT_INST_IRQ_BY_IDX(0, 1, irq));                                         \
		usb_isr_suspend_enable();                                                          \
	}                                                                                          \
	static int usb_rtl87x2g_pwr_on_func##n(const struct device *dev)                           \
	{                                                                                          \
		return hal_usb_phy_power_on();                                                     \
	}                                                                                          \
	static const struct usb_dwc2_config usb_dwc2_cfg_##n = {                                   \
		.base = (struct usb_dwc2_reg *)DT_INST_REG_ADDR(n),                                \
		.irq_enable_func = usb_dwc2_irq_enable_func_##n,                                   \
		.pwr_on_func = usb_rtl87x2g_pwr_on_func##n,                                        \
	};                                                                                         \
                                                                                                   \
	static struct usb_dwc2_ctrl_prv usb_dwc2_ctrl_##n;

USB_DWC2_DEVICE_DEFINE(0)

#define usb_dwc2_ctrl usb_dwc2_ctrl_0
#define usb_dwc2_cfg  usb_dwc2_cfg_0

typedef union grstctl_data {
	uint32_t d32;
	struct {
		unsigned csftrst: 1;
		unsigned hsftrst: 1;
		unsigned hstfrm: 1;
		unsigned intknqflsh: 1;
		unsigned rxfflsh: 1;
		unsigned txfflsh: 1;
		unsigned txfnum: 5;
		unsigned reserved11_28: 18;
		unsigned CSftRstDone: 1;
		unsigned dmareq: 1;
		unsigned ahbidle: 1;
	} b;
} grstctl_t;

static uint8_t usb_dwc2_ep_is_valid(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	/* Check if ep enabled */
	if ((USB_EP_DIR_IS_OUT(ep)) && ep_idx < USB_DWC2_OUT_EP_NUM) {
		return 1;
	} else if ((USB_EP_DIR_IS_IN(ep)) && ep_idx < USB_DWC2_IN_EP_NUM) {
		return 1;
	}

	return 0;
}

static uint8_t usb_dwc2_ep_is_enabled(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	/* Check if ep enabled */
	if ((USB_EP_DIR_IS_OUT(ep)) && usb_dwc2_ctrl.out_ep_ctrl[ep_idx].ep_ena) {
		return 1;
	} else if ((USB_EP_DIR_IS_IN(ep)) && usb_dwc2_ctrl.in_ep_ctrl[ep_idx].ep_ena) {
		return 1;
	}

	return 0;
}

static inline void usb_dwc2_udelay(uint32_t us)
{
	k_busy_wait(us);
}

static int usb_dwc2_reset(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s]", __func__);
#endif
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint32_t cnt = 0U;

	/* Wait for AHB master idle susb_dwc2_resettate. */
	while (!(base->grstctl & USB_DWC2_GRSTCTL_AHBIDLE)) {
		usb_dwc2_udelay(1);

		if (++cnt > USB_DWC2_CORE_RST_TIMEOUT_US) {
			LOG_ERR("USB reset HANG! AHB Idle GRSTCTL=0x%08x", base->grstctl);
			return -EIO;
		}
	}

	/* Core Soft Reset */
	cnt = 0U;
	base->grstctl |= USB_DWC2_GRSTCTL_CSFTRST;

	do {
		if (++cnt > USB_DWC2_CORE_RST_TIMEOUT_US) {
			LOG_DBG("USB reset HANG! Soft Reset GRSTCTL=0x%08x", base->grstctl);
			return -EIO;
		}
		usb_dwc2_udelay(1);
	} while (!(base->grstctl & (1 << 29)));
	base->grstctl |= 1 << 29;
	base->grstctl &= ~(1 << 0);
	/* Wait for 3 PHY Clocks */
	usb_dwc2_udelay(3000);

	return 0;
}

static int usb_dwc2_num_dev_eps(void)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

	return (base->ghwcfg2 >> 10) & 0xf;
}

static void usb_dwc2_flush_tx_fifo(int ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	int fnum = usb_dwc2_ctrl.in_ep_ctrl[ep].fifo_num;

	base->grstctl = (fnum << 6) | (1 << 5);
	while (base->grstctl & (1 << 5)) {
	}
}

int usb_dwc2_tx_fifo_avail(int ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

	return base->in_ep[ep].dtxfsts & USB_DWC2_DTXFSTS_INEPTXFSPCAVAIL_MASK;
}

/* Choose a FIFO number for an IN endpoint */
static int usb_dwc2_set_fifo(uint8_t ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	int ep_idx = USB_EP_GET_IDX(ep);
	volatile uint32_t *reg = &base->in_ep[ep_idx].diepctl;
	uint32_t val;
	int fifo = 0;
	int ded_fifo = !!(base->ghwcfg4 & USB_DWC2_GHWCFG4_DEDFIFOMODE);

	if (!ded_fifo) {
		/* No support for shared-FIFO mode yet, existing
		 * Zephyr hardware doesn't use it
		 */
		return -ENOTSUP;
	}

	/* In dedicated-FIFO mode, all IN endpoints must have a unique
	 * FIFO number associated with them in the TXFNUM field of
	 * DIEPCTLx, with EP0 always being assigned to FIFO zero (the
	 * reset default, so we don't touch it).
	 *
	 * FIXME: would be better (c.f. the dwc2 driver in Linux) to
	 * choose a FIFO based on the hardware depth: we want the
	 * smallest one that fits our configured maximum packet size
	 * for the endpoint.  This just picks the next available one.
	 */
	if (ep_idx != 0) {
		fifo = ep_idx;
		if (fifo >= usb_dwc2_num_dev_eps()) {
			return -EINVAL;
		}

		reg = &base->in_ep[ep_idx].diepctl;
		val = *reg & ~USB_DWC2_DEPCTL_TXFNUM_MASK;
		val |= fifo << USB_DWC2_DEPCTL_TXFNUM_POS;
		*reg = val;
	}

	usb_dwc2_ctrl.in_ep_ctrl[ep_idx].fifo_num = fifo;

	usb_dwc2_flush_tx_fifo(ep_idx);

	val = usb_dwc2_tx_fifo_avail(ep_idx);
	usb_dwc2_ctrl.in_ep_ctrl[ep_idx].fifo_size = val;

	return 0;
}

static int usb_dwc2_ep_set(uint8_t ep, uint32_t ep_mps, enum usb_dc_ep_transfer_type ep_type)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	volatile uint32_t *p_depctl;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	LOG_DBG("%s ep %x, mps %d, type %d", __func__, ep, ep_mps, ep_type);

	if (USB_EP_DIR_IS_OUT(ep)) {
		p_depctl = &base->out_ep[ep_idx].doepctl;
		usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps = ep_mps;
	} else {
		p_depctl = &base->in_ep[ep_idx].diepctl;
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].mps = ep_mps;
	}

	if (!ep_idx) {
		/* Set max packet size for EP0 */
		*p_depctl &= ~USB_DWC2_DEPCTL0_MPS_MASK;

		switch (ep_mps) {
		case 8:
			*p_depctl |= USB_DWC2_DEPCTL0_MPS_8 << USB_DWC2_DEPCTL_MPS_POS;
			break;
		case 16:
			*p_depctl |= USB_DWC2_DEPCTL0_MPS_16 << USB_DWC2_DEPCTL_MPS_POS;
			break;
		case 32:
			*p_depctl |= USB_DWC2_DEPCTL0_MPS_32 << USB_DWC2_DEPCTL_MPS_POS;
			break;
		case 64:
			*p_depctl |= USB_DWC2_DEPCTL0_MPS_64 << USB_DWC2_DEPCTL_MPS_POS;
			break;
		default:
			return -EINVAL;
		}
		/* No need to set EP0 type */
	} else {
		/* Set max packet size for EP */
		if (ep_mps > (0x3ff >> USB_DWC2_DEPCTL_MPS_POS)) {
			return -EINVAL;
		}

		*p_depctl &= ~0x3ff;
		*p_depctl |= ep_mps << USB_DWC2_DEPCTL_MPS_POS;

		/* Set endpoint type */
		*p_depctl &= ~USB_DWC2_DEPCTL_EPTYPE_MASK;

		switch (ep_type) {
		case USB_DC_EP_CONTROL:
			*p_depctl |= USB_DWC2_DEPCTL_EPTYPE_CONTROL << USB_DWC2_DEPCTL_EPTYPE_POS;
			break;
		case USB_DC_EP_BULK:
			*p_depctl |= USB_DWC2_DEPCTL_EPTYPE_BULK << USB_DWC2_DEPCTL_EPTYPE_POS;
			break;
		case USB_DC_EP_INTERRUPT:
			*p_depctl |= USB_DWC2_DEPCTL_EPTYPE_INTERRUPT << USB_DWC2_DEPCTL_EPTYPE_POS;
			break;
		default:
			return -EINVAL;
		}

		/* sets the Endpoint Data PID to DATA0 */
		*p_depctl |= USB_DWC2_DEPCTL_SETD0PID;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		int ret = usb_dwc2_set_fifo(ep);

		if (ret) {
			return ret;
		}
	}

	return 0;
}

static void usb_dwc2_flush_all_fifo(void)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	grstctl_t grstctl = {.b = {.txfnum = 0x10, .txfflsh = 1, .rxfflsh = 1}};

	base->grstctl = grstctl.d32;
	do {
		grstctl.d32 = base->grstctl;
	} while (grstctl.b.txfflsh || grstctl.b.rxfflsh);
}

static void usb_dwc2_resize_fifo(void)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

	/* 1. Get Tx FIFO size, and caculate Rx FIFO size. */
	uint32_t total_fifo_size = base->ghwcfg3 >> 16;
	uint32_t total_tx_fifo_size = 0;

	for (uint8_t epidx = 0; epidx < 6; ++epidx) {
		total_tx_fifo_size += TX_FIFO_SIZE(epidx);
	}

	uint32_t rx_fifo_size = total_fifo_size - total_tx_fifo_size;

	/* 2. Write back. */
	base->grxfsiz = rx_fifo_size;

	typedef union fifosize_data {
		/* raw register data */
		uint32_t d32;
		/* register bits */
		struct {
			unsigned startaddr: 16;
			unsigned depth: 16;
		} b;
	} fifosize_data_t;
	fifosize_data_t curr_fifo, last_fifo;

	curr_fifo.d32 = base->gnptxfsiz;
	curr_fifo.b.startaddr = rx_fifo_size;
	curr_fifo.b.depth = TX_FIFO_SIZE(0);
	base->gnptxfsiz = curr_fifo.d32;

	last_fifo = curr_fifo;

	for (uint8_t epidx = 1; epidx < 6; ++epidx) {
		kaTxFifoAddrTable[epidx] = curr_fifo.b.depth + kaTxFifoAddrTable[epidx - 1];
		curr_fifo.d32 = (base->dieptxf)[epidx - 1];
		curr_fifo.b.startaddr = last_fifo.b.startaddr + last_fifo.b.depth;
		curr_fifo.b.depth = TX_FIFO_SIZE(epidx);
		(base->dieptxf)[epidx - 1] = curr_fifo.d32;
		last_fifo = curr_fifo;
	}

	usb_dwc2_flush_all_fifo();

	/* 4. Link FIFO to IN ep. */
	for (uint8_t epidx = 0; epidx < 6; ++epidx) {
		uint32_t diepctl = base->in_ep[epidx].diepctl;

		diepctl &= (~((BIT4 - 1) << 22));
		diepctl |= (epidx << 22);
		base->in_ep[epidx].diepctl = diepctl;
	}
}

static int usb_rtk_init(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	extern void hal_rtk_usb_init_dma(void);
	hal_rtk_usb_init_dma();
#else
	extern void hal_rtk_usb_init(void);
	hal_rtk_usb_init();
#endif
	return 0;
}

static void usb_dwc2_handle_reset(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	usb_dwc2_ctrl.current_status = USB_DC_RESET;
	/* Inform upper layers */
	if (usb_dwc2_ctrl.status_cb) {
		usb_dwc2_ctrl.status_cb(USB_DC_RESET, NULL);
	}

	usb_dwc2_reset();
	usb_rtk_init();
	usb_dwc2_resize_fifo();

#else
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

	usb_dwc2_ctrl.current_status = USB_DC_RESET;
	/* Inform upper layers */
	if (usb_dwc2_ctrl.status_cb) {
		usb_dwc2_ctrl.status_cb(USB_DC_RESET, NULL);
	}

	usb_dwc2_resize_fifo();

	/* Clear device address during reset. */
	base->dcfg &= ~USB_DWC2_DCFG_DEVADDR_MASK;

	/* enable global EP interrupts */
	base->doepmsk = 0U;
	base->gintmsk |= USB_DWC2_GINTSTS_RXFLVL;
	base->diepmsk |= USB_DWC2_DIEPINT_XFERCOMPL;
#endif
}

#if CONFIG_USB_DC_RTL87X2G_DMA
static uint32_t usb_get_in_xfer_max_bytes(uint8_t ep_idx)
{
	uint16_t ep_mps = usb_dwc2_ctrl.in_ep_ctrl[ep_idx].mps;

	return 0xffff / ep_mps * ep_mps;
}

static int usb_ep_dma_send(uint8_t ep_idx)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

	usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSent += usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSending;
	usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSending = 0;
	if (usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSent < usb_dwc2_ctrl.aInXfer[ep_idx]->BytesTotal) {
		const uint16_t ep_mps = usb_dwc2_ctrl.in_ep_ctrl[ep_idx].mps;
		uint32_t bytes_residual = usb_dwc2_ctrl.aInXfer[ep_idx]->BytesTotal -
					  usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSent;
		uint32_t xfer_bytes = usb_get_in_xfer_max_bytes(ep_idx) < bytes_residual
					      ? usb_get_in_xfer_max_bytes(ep_idx)
					      : bytes_residual;
		volatile InDmaDesc_t *pDmaDesc = usb_dwc2_ctrl.aInDmaDesc[ep_idx];

		pDmaDesc->Quad.NonIso.TxBytes = xfer_bytes;
		pDmaDesc->Quad.NonIso.Reserved0 = 0;
		pDmaDesc->Quad.NonIso.InterruptOnComplete = 1;
		pDmaDesc->Quad.NonIso.ShortPacket = (xfer_bytes % ep_mps != 0);
		pDmaDesc->Quad.NonIso.Last = 1;
		pDmaDesc->Quad.NonIso.TransmitState = QUAD_TXSTS_SUCCESS;
		pDmaDesc->Quad.NonIso.BufferState = QUAD_BS_HOST_READY;

		pDmaDesc->BufAddr = (uint32_t)usb_dwc2_ctrl.aInXfer[ep_idx]->pBuf +
				    usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSent;

		base->in_ep[ep_idx].diepdma = (uint32_t)pDmaDesc;
		base->in_ep[ep_idx].diepctl |= USB_DWC2_DEPCTL_EPENA | BIT26;

		usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSending = xfer_bytes;
		return xfer_bytes;
	} else if (usb_dwc2_ctrl.aInXfer[ep_idx]->ToSendZlp) {
		/* All data has been sent, and only zlp is waiting to send. Send zlp. */
		volatile InDmaDesc_t *pDmaDesc = usb_dwc2_ctrl.aInDmaDesc[ep_idx];

		pDmaDesc->Quad.NonIso.TxBytes = 0;
		pDmaDesc->Quad.NonIso.Reserved0 = 0;
		pDmaDesc->Quad.NonIso.InterruptOnComplete = 1;
		pDmaDesc->Quad.NonIso.ShortPacket = 1;
		pDmaDesc->Quad.NonIso.Last = 1;
		pDmaDesc->Quad.NonIso.TransmitState = QUAD_TXSTS_SUCCESS;
		pDmaDesc->Quad.NonIso.BufferState = QUAD_BS_HOST_READY;

		pDmaDesc->BufAddr = (uint32_t)usb_dwc2_ctrl.aInXfer[ep_idx]->pBuf +
				    usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSent;

		base->in_ep[ep_idx].diepdma = (uint32_t)pDmaDesc;
		base->in_ep[ep_idx].diepctl |= USB_DWC2_DEPCTL_EPENA | BIT26;

		usb_dwc2_ctrl.aInXfer[ep_idx]->ToSendZlp = false;
	}

	return 0;
}

static int usb_dwc2_tx_dma(uint8_t ep, const uint8_t *const data, uint32_t data_len)
{
	enum usb_dwc2_in_ep_idx ep_idx = USB_EP_GET_IDX(ep);

	usb_dwc2_ctrl.aInXfer[ep_idx]->pBuf = data;
	usb_dwc2_ctrl.aInXfer[ep_idx]->BytesTotal = data_len;
	usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSent = 0;
	usb_dwc2_ctrl.aInXfer[ep_idx]->BytesSending = 0;
	usb_dwc2_ctrl.aInXfer[ep_idx]->ToSendZlp = data_len == 0;

	return usb_ep_dma_send(ep_idx);
}

static uint32_t usb_get_out_xfer_max_bytes(uint8_t ep_idx)
{
	uint16_t ep_mps = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps;

	return (ep_idx == 0) ? ep_mps : (0xffff / ep_mps * ep_mps);
}

static int usb_ep_dma_receive(uint8_t ep_idx)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	const uint16_t ep_mps = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps;
	volatile OutDmaDesc_t *pDmaDesc = usb_dwc2_ctrl.aOutDmaDesc[ep_idx];
	bool MoreDataToRcv;

	if (usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcving == 0) {
		MoreDataToRcv = true;
	} else {
		uint32_t BytesRcvedThisCycle =
			usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcving - pDmaDesc->Quad.NonIso.RxBytes;

		usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcved += BytesRcvedThisCycle;
		usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcving = 0;
		if (usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcved ==
			    usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesTotal ||
		    pDmaDesc->Quad.NonIso.ShortPacket) {
			/* All bytes were received, or short packet was received. Out xfer complete.
			 */
			MoreDataToRcv = false;
		} else {
			MoreDataToRcv = true;
		}
	}

	if (MoreDataToRcv) {
		uint32_t bytes_residual = usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesTotal -
					  usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcved;
		uint32_t xfer_bytes = usb_get_out_xfer_max_bytes(ep_idx) < bytes_residual
					      ? usb_get_out_xfer_max_bytes(ep_idx)
					      : bytes_residual;

		usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcving =
			(xfer_bytes + ep_mps - 1) / ep_mps * ep_mps;

		pDmaDesc->Quad.NonIso.RxBytes = usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcving;
		pDmaDesc->Quad.NonIso.Reserved0 = 0;
		pDmaDesc->Quad.NonIso.MTRF = 0;
		pDmaDesc->Quad.NonIso.SR = 0;
		pDmaDesc->Quad.NonIso.InterruptOnComplete = 1;
		pDmaDesc->Quad.NonIso.ShortPacket = 0;
		pDmaDesc->Quad.NonIso.Last = 1;
		pDmaDesc->Quad.NonIso.ReceiveState = QUAD_RXSTS_SUCCESS;
		pDmaDesc->Quad.NonIso.BufferState = QUAD_BS_HOST_READY;

		pDmaDesc->BufAddr = (uint32_t)usb_dwc2_ctrl.aOutXfer[ep_idx]->pBuf +
				    usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcved;

		base->out_ep[ep_idx].doepdma = (uint32_t)pDmaDesc;

		base->out_ep[ep_idx].doepctl |= USB_DWC2_DEPCTL_EPENA | BIT26;
	}

	return 0;
}

static int usb_dwc2_rx_dma(const uint8_t ep, uint8_t *data, const uint32_t read_bytes)
{
	enum usb_dwc2_in_ep_idx ep_idx = USB_EP_GET_IDX(ep);

	usb_dwc2_ctrl.aOutXfer[ep_idx]->pBuf = data;
	usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesTotal = read_bytes;
	usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcving = 0;
	usb_dwc2_ctrl.aOutXfer[ep_idx]->BytesRcved = 0;

	return usb_ep_dma_receive(ep_idx);
}

#else
static int usb_dwc2_tx(uint8_t ep, const uint8_t *const data, uint32_t data_len)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	enum usb_dwc2_in_ep_idx ep_idx = USB_EP_GET_IDX(ep);
	uint32_t max_xfer_size, max_pkt_cnt, pkt_cnt, avail_space;
	uint32_t ep_mps = usb_dwc2_ctrl.in_ep_ctrl[ep_idx].mps;
	unsigned int key;
	uint32_t i;
	uint32_t tx_data_len = data_len;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] base->in_ep[%d].diepctl=0x%x", __func__, ep_idx,
		   base->in_ep[ep_idx].diepctl);
#endif

	/* Wait for FIFO space available */
	do {
		avail_space = usb_dwc2_tx_fifo_avail(ep_idx);
		if (avail_space == usb_dwc2_ctrl.in_ep_ctrl[ep_idx].fifo_size) {
			break;
		}
		/* Make sure we don't hog the CPU */
		k_yield();
	} while (1);
	key = irq_lock();

	avail_space *= 4U;
	if (!avail_space) {
		LOG_ERR("USB IN EP%d no space available, DTXFSTS %x", ep_idx,
			base->in_ep[ep_idx].dtxfsts);
		irq_unlock(key);
		return -EAGAIN;
	}

	/* For now tx-fifo sizes are not configured (cf usb_dwc2_set_fifo). Here
	 * we force available fifo size to be a multiple of ep mps in order to
	 * prevent splitting data incorrectly.
	 */
	avail_space -= avail_space % ep_mps;
	if (data_len > avail_space) {
		data_len = avail_space;
	}

	if (data_len != 0U) {
		/* Get max packet size and packet count for ep */
		if (ep_idx == USB_DWC2_IN_EP_0) {
			max_pkt_cnt = 1;
			max_xfer_size = ep_mps;
		} else {
			max_pkt_cnt =
				USB_DWC2_DIEPTSIZn_PKT_CNT_MASK >> USB_DWC2_DEPTSIZ_PKT_CNT_POS;
			max_xfer_size =
				USB_DWC2_DEPTSIZn_XFER_SIZE_MASK >> USB_DWC2_DEPTSIZ_XFER_SIZE_POS;
		}

		/* Check if transfer len is too big */
		if (data_len > max_xfer_size) {
			LOG_WRN("USB IN EP%d len too big (%d->%d)", ep_idx, data_len,
				max_xfer_size);
			data_len = max_xfer_size;
		}

		/*
		 * Program the transfer size and packet count as follows:
		 *
		 * transfer size = N * ep_maxpacket + short_packet
		 * pktcnt = N + (short_packet exist ? 1 : 0)
		 */

		pkt_cnt = DIV_ROUND_UP(data_len, ep_mps);
		if (pkt_cnt > max_pkt_cnt) {
			LOG_WRN("USB IN EP%d pkt count too big (%d->%d)", ep_idx, pkt_cnt, pkt_cnt);
			pkt_cnt = max_pkt_cnt;
			data_len = pkt_cnt * ep_mps;
		}
	} else {
		/* Zero length packet */
		pkt_cnt = 1U;
	}

	if (tx_data_len > data_len) {
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_len = tx_data_len - data_len;
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_buf = data + data_len;
	} else {
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_len = 0;
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].rsvd_tx_buf = NULL;
	}

	/* Set number of packets and transfer size */
	base->in_ep[ep_idx].dieptsiz = (pkt_cnt << USB_DWC2_DEPTSIZ_PKT_CNT_POS) | data_len;

	/* Clear NAK and enable ep */
	base->in_ep[ep_idx].diepctl |= (USB_DWC2_DEPCTL_EPENA | USB_DWC2_DEPCTL_CNAK);

	/*
	 * Write data to FIFO, make sure that we are protected against
	 * other USB register accesses.  According to "DesignWare Cores
	 * USB 1.1/2.0 Device Subsystem-AHB/VCI Databook": "During FIFO
	 * access, the application must not access the UDC/Subsystem
	 * registers or vendor registers (for ULPI mode). After starting
	 * to access a FIFO, the application must complete the transaction
	 * before accessing the register."
	 */
	for (i = 0U; i < data_len; i += 4U) {
		uint32_t val = data[i];

		if (i + 1 < data_len) {
			val |= ((uint32_t)data[i + 1]) << 8;
		}
		if (i + 2 < data_len) {
			val |= ((uint32_t)data[i + 2]) << 16;
		}
		if (i + 3 < data_len) {
			val |= ((uint32_t)data[i + 3]) << 24;
		}
#if DBG_DIRECT_SHOW
		DBG_DIRECT("val=%x", val);
#endif
		USB_DWC2_EP_TX_FIFO(base, ep_idx) = val;
	}

	irq_unlock(key);

#if DBG_DIRECT_SHOW
	DBG_DIRECT("USB IN EP%d write %d bytes", ep_idx, data_len);
#endif

	return data_len;
}

#endif /* CONFIG_USB_DC_RTL87X2G_DMA */

static void usb_dwc2_prep_rx(const uint8_t ep, uint8_t setup)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	enum usb_dwc2_out_ep_idx ep_idx = USB_EP_GET_IDX(ep);
	uint32_t ep_mps = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps;

	usb_dwc2_rx_dma(ep, usb_dwc2_ctrl.ep_rx_buf[ep_idx], ep_mps);

	LOG_DBG("USB OUT EP%d armed", ep_idx);

#else
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	enum usb_dwc2_out_ep_idx ep_idx = USB_EP_GET_IDX(ep);
	uint32_t ep_mps = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps;

	/* Set max RX size to EP mps so we get an interrupt
	 * each time a packet is received
	 */

	base->out_ep[ep_idx].doeptsiz = (USB_DWC2_SUP_CNT << USB_DWC2_DOEPTSIZ_SUP_CNT_POS) |
					(1 << USB_DWC2_DEPTSIZ_PKT_CNT_POS) | ep_mps;

	/* Clear NAK and enable ep */
	if (!setup) {
		base->out_ep[ep_idx].doepctl |= USB_DWC2_DEPCTL_CNAK;
	}

	base->out_ep[ep_idx].doepctl |= USB_DWC2_DEPCTL_EPENA;

	LOG_DBG("USB OUT EP%d armed", ep_idx);
#endif
}

void usb_dwc2_handle_enum_done(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t speed;
	uint32_t ep_mps;

	speed = (base->dsts & ~USB_DWC2_DSTS_ENUMSPD_MASK) >> USB_DWC2_DSTS_ENUMSPD_POS;

	usb_dwc2_ctrl.current_status = USB_DC_CONNECTED;
	/* Inform upper layers */
	if (usb_dwc2_ctrl.status_cb) {
		usb_dwc2_ctrl.status_cb(USB_DC_CONNECTED, &speed);
	}

	base->in_ep[0].diepctl = (base->in_ep[0].diepctl & ~USB_DWC2_DIEPCTL_MPS_MSAK) |
				 USB_DWC2_DIEPCTL_MPS_64BYTES;
	base->out_ep[0].doepctl = (base->out_ep[0].doepctl & ~USB_DWC2_DOEPCTL_MPS_MSAK) |
				  USB_DWC2_DOEPCTL_MPS_64BYTES;
	ep_mps = usb_dwc2_ctrl.out_ep_ctrl[0].mps;
	usb_dwc2_rx_dma(0, usb_dwc2_ctrl.ep_rx_buf[0], ep_mps);

#else
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t speed;

	speed = (base->dsts & USB_DWC2_DSTS_ENUMSPD_MASK) >> USB_DWC2_DSTS_ENUMSPD_POS;

	LOG_DBG("USB ENUM DONE event, %s speed detected", speed == 0 ? "High" : "Full");
#if DBG_DIRECT_SHOW
	DBG_DIRECT("USB ENUM DONE event, %s speed detected", speed == 0 ? "High" : "Full");
#endif

	usb_dwc2_ctrl.current_status = USB_DC_CONNECTED;
	/* Inform upper layers */
	if (usb_dwc2_ctrl.status_cb) {
		usb_dwc2_ctrl.status_cb(USB_DC_CONNECTED, &speed);
	}
#endif
}

/* USB ISR handler */
static inline void usb_dwc2_int_rx_flvl_handler(void)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint32_t grxstsp = base->grxstsp;
	uint32_t status, xfer_size;
	uint8_t ep_idx;
	usb_dc_ep_callback ep_cb;

	/* Packet in RX FIFO */

	ep_idx = grxstsp & USB_DWC2_GRXSTSR_EPNUM_MASK;
	status = (grxstsp & USB_DWC2_GRXSTSR_PKTSTS_MASK) >> USB_DWC2_GRXSTSR_PKTSTS_POS;
	xfer_size = (grxstsp & USB_DWC2_GRXSTSR_BCNT_MASK) >> USB_DWC2_GRXSTSR_BCNT_POS;

	LOG_DBG("USB OUT EP%u: RX_FLVL status %u, size %u", ep_idx, status, xfer_size);
#if DBG_DIRECT_SHOW
	DBG_DIRECT("USB OUT EP%d: grxstsp status %d, size %d", ep_idx, status, xfer_size);
#endif

	usb_dwc2_ctrl.out_ep_ctrl[ep_idx].data_len = xfer_size;
	ep_cb = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].cb;

	switch (status) {
#if CONFIG_USB_DC_RTL87X2G_DMA
	case 12:
#endif
	case USB_DWC2_GRXSTSR_PKTSTS_SETUP:
		LOG_DBG("USB_DWC2_GRXSTSR_PKTSTS_SETUP");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DWC2_GRXSTSR_PKTSTS_SETUP");
#endif
		/* Call the registered callback if any */
		if (ep_cb) {
			ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_OUT), USB_DC_EP_SETUP);
		}

		break;
	case USB_DWC2_GRXSTSR_PKTSTS_OUT_DATA:
		LOG_DBG("USB_DWC2_GRXSTSR_PKTSTS_OUT_DATA");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DWC2_GRXSTSR_PKTSTS_OUT_DATA");
#endif
		if (ep_cb) {
			ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_OUT), USB_DC_EP_DATA_OUT);
		}

		break;
	case USB_DWC2_GRXSTSR_PKTSTS_OUT_DATA_DONE:
		LOG_DBG("USB_DWC2_GRXSTSR_PKTSTS_OUT_DATA_DONE");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DWC2_GRXSTSR_PKTSTS_OUT_DATA_DONE");
#endif
		break;
	case USB_DWC2_GRXSTSR_PKTSTS_SETUP_DONE:
		LOG_DBG("USB_DWC2_GRXSTSR_PKTSTS_SETUP_DONE");
#if DBG_DIRECT_SHOW
		DBG_DIRECT("USB_DWC2_GRXSTSR_PKTSTS_SETUP_DONE");
#endif
		break;
	default:
		break;
	}
}

static inline void usb_dwc2_int_iep_handler(void)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint32_t ep_int_status;
	uint32_t ActDaint = base->daint & base->daintmsk;
	uint8_t ep_idx = 0;
	usb_dc_ep_callback ep_cb;
	uint32_t ep_mps;

	if (ActDaint & USB_DWC2_DAINT_INEPINT(ep_idx)) {
		ep_int_status = base->in_ep[ep_idx].diepint & base->diepmsk;
#if DBG_DIRECT_SHOW
		DBG_DIRECT("[%s] Ep0 diepint: 0x%x", __func__, ep_int_status);
#endif
		base->in_ep[ep_idx].diepint = ep_int_status;
		ep_mps = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps;
		usb_dwc2_rx_dma(ep_idx, usb_dwc2_ctrl.ep_rx_buf[ep_idx], ep_mps);

		ep_cb = usb_dwc2_ctrl.in_ep_ctrl[ep_idx].cb;
		if (ep_cb && (ep_int_status & USB_DWC2_DIEPINT_XFERCOMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] USB_DWC2_DIEPINT_XFERCOMPL", __func__);
#endif
			ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_IN), USB_DC_EP_DATA_IN);
		}
	}
	for (ep_idx = 1U; ep_idx < USB_DWC2_IN_EP_NUM; ep_idx++) {
		if (base->daint & USB_DWC2_DAINT_INEPINT(ep_idx)) {
			/* Read IN EP interrupt status */
			ep_int_status = base->in_ep[ep_idx].diepint & base->diepmsk;
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] Ep%d diepint: 0x%x", __func__, ep_idx, ep_int_status);
#endif
			base->in_ep[ep_idx].diepint = ep_int_status;
			ep_cb = usb_dwc2_ctrl.in_ep_ctrl[ep_idx].cb;
			if (ep_cb && (ep_int_status & USB_DWC2_DIEPINT_XFERCOMPL)) {

				/* Call the registered callback */
				ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_IN), USB_DC_EP_DATA_IN);
			}
		}
	}
#else
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint32_t ep_int_status;
	uint8_t ep_idx;
	usb_dc_ep_callback ep_cb;

	for (ep_idx = 0U; ep_idx < USB_DWC2_IN_EP_NUM; ep_idx++) {
		if (base->daint & USB_DWC2_DAINT_INEPINT(ep_idx)) {
			/* Read IN EP interrupt status */
			ep_int_status = base->in_ep[ep_idx].diepint & base->diepmsk;

			/* Clear IN EP interrupts */
			base->in_ep[ep_idx].diepint = ep_int_status;

			LOG_DBG("USB IN EP%u interrupt status: 0x%x", ep_idx, ep_int_status);

			ep_cb = usb_dwc2_ctrl.in_ep_ctrl[ep_idx].cb;
			if (ep_cb && (ep_int_status & USB_DWC2_DIEPINT_XFERCOMPL)) {
				/* Call the registered callback */
				ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_IN), USB_DC_EP_DATA_IN);
			}
		}
	}

	/* Clear interrupt. */
	base->gintsts = USB_DWC2_GINTSTS_IEPINT;
#endif
}

#if CONFIG_USB_DC_RTL87X2G_DMA
static void Ep0ParseSetupPkt(void)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx;
	usb_dc_ep_callback ep_cb;
	uint32_t grxstsp = base->grxstsp;

	ep_idx = grxstsp & USB_DWC2_GRXSTSR_EPNUM_MASK;
	ep_cb = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].cb;
	ep_cb(USB_EP_GET_ADDR(ep_idx, USB_EP_DIR_OUT), USB_DC_EP_SETUP);
}
#endif

static inline void usb_dwc2_int_oep_handler(void)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint32_t ep_int_status;
	uint8_t ep_idx = 0;

#if CONFIG_USB_DC_RTL87X2G_DMA
	if (base->daint & USB_DWC2_DAINT_OUTEPINT(ep_idx)) {
		ep_int_status = base->out_ep[ep_idx].doepint & base->doepmsk;
		base->out_ep[ep_idx].doepint = ep_int_status;
#if DBG_DIRECT_SHOW
		DBG_DIRECT("[%s] Ep0 doepint: 0x%x", __func__, ep_int_status);
#endif

		if (!(ep_int_status & USB_DWC2_DOEPINT_SETUP) &&
		    (ep_int_status & USB_DWC2_DOEPINT_XFERCOMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] EP0EVENT_OEPINT_XFERCMPL", __func__);
#endif
		} else if ((ep_int_status & USB_DWC2_DOEPINT_SETUP) &&
			   !(ep_int_status & USB_DWC2_DOEPINT_XFERCOMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] EP0EVENT_OEPINT_SETUPDONE", __func__);
#endif
		} else if ((ep_int_status & USB_DWC2_DOEPINT_SETUP) &&
			   (ep_int_status & USB_DWC2_DOEPINT_XFERCOMPL)) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] EP0EVENT_OEPINT_XFERCMPL_AND_SETUPDONE", __func__);
#endif
			Ep0ParseSetupPkt();

		} else if ((ep_int_status & USB_DWC2_DOEPINT_STSPHSERCVD) &&
			   !(ep_int_status & USB_DWC2_DOEPINT_XFERCOMPL)) {
			base->out_ep[ep_idx].doepctl |= USB_DWC2_DEPCTL_CNAK;
		}

		if (ep_int_status & USB_DWC2_DOEPINT_AHBERR ||
		    ep_int_status & USB_DWC2_DOEPINT_BNAINTR) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("[%s] USB_DWC2_DOEPINT_BNAINTR", __func__);
#endif
		}
	}
#endif
#if CONFIG_USB_DC_RTL87X2G_DMA
	for (ep_idx = 1U; ep_idx < USB_DWC2_OUT_EP_NUM; ep_idx++)
#else
	for (ep_idx = 0U; ep_idx < USB_DWC2_OUT_EP_NUM; ep_idx++)
#endif
	{
		if (base->daint & USB_DWC2_DAINT_OUTEPINT(ep_idx)) {
			/* Read OUT EP interrupt status */
			ep_int_status = base->out_ep[ep_idx].doepint & base->doepmsk;

			/* Clear OUT EP interrupts */
			base->out_ep[ep_idx].doepint = ep_int_status;

			LOG_DBG("USB OUT EP%u interrupt status: 0x%x\n", ep_idx, ep_int_status);
		}
	}

	/* Clear interrupt. */
	base->gintsts = USB_DWC2_GINTSTS_OEPINT;
}

static void usb_dwc2_isr_handler(const void *unused)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint32_t int_status;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s]", __func__);
#endif
	ARG_UNUSED(unused);
	/*  Read interrupt status */
	while ((int_status = (base->gintsts & base->gintmsk))) {
		LOG_WRN("current addr:%d",
			(base->dcfg & USB_DWC2_DCFG_DEVADDR_MASK) >> USB_DWC2_DCFG_DEVADDR_POS);

		if (int_status & USB_DWC2_GINTSTS_USBRST) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DWC2_GINTSTS_USBRST-------------------");
#endif
			LOG_WRN("USB_DWC2_GINTSTS_USBRST");
			/* Clear interrupt. */
			base->gintsts = USB_DWC2_GINTSTS_USBRST;

			/* Reset detected */
			usb_dwc2_handle_reset();
		}

		if (int_status & USB_DWC2_GINTSTS_ENUMDONE) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT(
				"-------------------USB_DWC2_GINTSTS_ENUMDONE-------------------");
#endif
			LOG_WRN("USB_DWC2_GINTSTS_ENUMDONE");
			/* Clear interrupt. */
			base->gintsts = USB_DWC2_GINTSTS_ENUMDONE;

			/* Enumeration done detected */
			usb_dwc2_handle_enum_done();
		}

		if (int_status & USB_DWC2_GINTSTS_USBSUSP) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT(
				"-------------------USB_DWC2_GINTSTS_USBSUSP-------------------");
#endif
			LOG_WRN("USB_DWC2_GINTSTS_USBSUSP");
			/* Clear interrupt. */
			base->gintsts = USB_DWC2_GINTSTS_USBSUSP;
			extern int hal_usb_suspend_enter(void);

			hal_usb_suspend_enter();

			usb_power_is_on = false;

			usb_dwc2_ctrl.current_status = USB_DC_SUSPEND;
			if (usb_dwc2_ctrl.status_cb) {
				usb_dwc2_ctrl.status_cb(USB_DC_SUSPEND, NULL);
			}
			return;
		}

		if (int_status & USB_DWC2_GINTSTS_WKUPINT) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT(
				"-------------------USB_DWC2_GINTSTS_WKUPINT-------------------");
#endif
			LOG_WRN("USB_DWC2_GINTSTS_WKUPINT");
			/* Clear interrupt. */
			base->gintsts = USB_DWC2_GINTSTS_WKUPINT;

			usb_dwc2_ctrl.current_status = USB_DC_RESUME;
			if (usb_dwc2_ctrl.status_cb) {
				usb_dwc2_ctrl.status_cb(USB_DC_RESUME, NULL);
			}
		}

		if (int_status & USB_DWC2_GINTSTS_RXFLVL) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DWC2_GINTSTS_RXFLVL-------------------");
#endif
			LOG_WRN("USB_DWC2_GINTSTS_RXFLVL");
			/* Packet in RX FIFO */
			usb_dwc2_int_rx_flvl_handler();
		}

		if (int_status & USB_DWC2_GINTSTS_IEPINT) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DWC2_GINTSTS_IEPINT-------------------");
#endif
			LOG_WRN("USB_DWC2_GINTSTS_IEPINT");
			/* IN EP interrupt */
			usb_dwc2_int_iep_handler();
		}

		if (int_status & USB_DWC2_GINTSTS_OEPINT) {
#if DBG_DIRECT_SHOW
			DBG_DIRECT("-------------------USB_DWC2_GINTSTS_OEPINT-------------------");
#endif
			LOG_WRN("USB_DWC2_GINTSTS_OEPINT");
			/* No OUT interrupt expected in FIFO mode,
			 * just clear interrupt
			 */
			usb_dwc2_int_oep_handler();
		}
	}
}

static void usb_dwc2_resume_isr_handler(const void *unused)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("-------------------USB_DWC2_RESUME_INT-------------------");
#endif
	SoC_VENDOR->u_004.REG_LOW_PRI_INT_STATUS |= BIT31;
	/* prevent false alarm */
	usb_rtk_resume_sequence();

	usb_power_is_on = true;

	usb_dwc2_ctrl.current_status = USB_DC_RESUME;
	if (usb_dwc2_ctrl.status_cb) {
		usb_dwc2_ctrl.status_cb(USB_DC_RESUME, NULL);
	}
	return;
}

#if CONFIG_PM
static void usb_register_dlps_cb(void);
#endif

int usb_dc_attach(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s]", __func__);
#endif
	int ret;

	if (usb_dwc2_ctrl.attached) {
		return 0;
	}

	if (usb_dwc2_cfg.pwr_on_func != NULL) {
		ret = usb_dwc2_cfg.pwr_on_func(NULL);
		if (ret) {
			LOG_ERR("usb phy power on fials");
			return ret;
		}

		usb_power_is_on = true;
	}

	ret = usb_rtk_init();

	usb_dwc2_resize_fifo();

	if (ret) {
		LOG_ERR("usb dw init fials");
		return ret;
	}

#if CONFIG_PM
	usb_register_dlps_cb();
#endif

	usb_dwc2_cfg.irq_enable_func(NULL);

	usb_dwc2_ctrl.attached = 1U;

	return 0;
}

int usb_dc_detach(void)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

	if (!usb_dwc2_ctrl.attached) {
		return 0;
	}

	irq_disable(DT_INST_IRQ_BY_IDX(0, 0, irq));
	irq_disable(DT_INST_IRQ_BY_IDX(0, 1, irq));

	if (usb_power_is_on) {
		/* Enable soft disconnect */
		base->dctl |= USB_DWC2_DCTL_SFTDISCON;
	}

	usb_rtk_disable_power_seq();
	usb_power_is_on = false;

	usb_dwc2_ctrl.attached = 0U;

	usb_dwc2_ctrl.current_status = USB_DC_DISCONNECTED;
	if (usb_dwc2_ctrl.status_cb) {
		usb_dwc2_ctrl.status_cb(USB_DC_DISCONNECTED, NULL);
	}

	return 0;
}

int usb_dc_reset(void)
{
	int ret;

	ret = usb_dwc2_reset();

	/* Clear private data */
	(void)memset(&usb_dwc2_ctrl, 0, sizeof(usb_dwc2_ctrl));

	return ret;
}

int usb_dc_set_address(const uint8_t addr)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] %d", __func__, addr);
#endif
	LOG_WRN("usb_dc_set_address %d", addr);

	if (addr > (USB_DWC2_DCFG_DEVADDR_MASK >> USB_DWC2_DCFG_DEVADDR_POS)) {
		return -EINVAL;
	}

	base->dcfg &= ~USB_DWC2_DCFG_DEVADDR_MASK;
	base->dcfg |= addr << USB_DWC2_DCFG_DEVADDR_POS;

	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("invalid endpoint configuration");
		return -1;
	}

	if (cfg->ep_mps > DW_USB_MAX_PACKET_SIZE) {
		LOG_WRN("unsupported packet size");
		return -1;
	}

	if (USB_EP_DIR_IS_OUT(cfg->ep_addr) && ep_idx >= USB_DWC2_OUT_EP_NUM) {
		LOG_WRN("OUT endpoint address out of range");
		return -1;
	}

	if (USB_EP_DIR_IS_IN(cfg->ep_addr) && ep_idx >= USB_DWC2_IN_EP_NUM) {
		LOG_WRN("IN endpoint address out of range");
		return -1;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg)
{
	uint8_t ep;

	if (!ep_cfg) {
		return -EINVAL;
	}

	ep = ep_cfg->ep_addr;
	LOG_DBG(" ep = 0x%x", ep);
	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	usb_dwc2_ep_set(ep, ep_cfg->ep_mps, ep_cfg->ep_type);

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep[ep_idx].doepctl |= USB_DWC2_DEPCTL_STALL;
	} else {
		base->in_ep[ep_idx].diepctl |= USB_DWC2_DEPCTL_STALL;
	}

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (!ep_idx) {
		/* Not possible to clear stall for EP0 */
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep[ep_idx].doepctl &= ~USB_DWC2_DEPCTL_STALL;
	} else {
		base->in_ep[ep_idx].diepctl &= ~USB_DWC2_DEPCTL_STALL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	volatile uint32_t *p_depctl;

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (!ep_idx) {
		/* Cannot disable EP0, just set stall */
		usb_dc_ep_set_stall(ep);
	} else {
		if (USB_EP_DIR_IS_OUT(ep)) {
			p_depctl = &base->out_ep[ep_idx].doepctl;
		} else {
			p_depctl = &base->in_ep[ep_idx].diepctl;
		}

		/* Set STALL and disable endpoint if enabled */
		if (*p_depctl & USB_DWC2_DEPCTL_EPENA) {
			*p_depctl |= USB_DWC2_DEPCTL_EPDIS | USB_DWC2_DEPCTL_STALL;
		} else {
			*p_depctl |= USB_DWC2_DEPCTL_STALL;
		}
	}

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (!stalled) {
		return -EINVAL;
	}

	*stalled = 0U;
	if (USB_EP_DIR_IS_OUT(ep)) {
		if (base->out_ep[ep_idx].doepctl & USB_DWC2_DEPCTL_STALL) {
			*stalled = 1U;
		}
	} else {
		if (base->in_ep[ep_idx].diepctl & USB_DWC2_DEPCTL_STALL) {
			*stalled = 1U;
		}
	}

	return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	LOG_DBG("ep %x", ep);
	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* enable EP interrupts */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->daintmsk |= USB_DWC2_DAINT_OUTEPINT(ep_idx);
	} else {
		base->daintmsk |= USB_DWC2_DAINT_INEPINT(ep_idx);
	}

	/* Activate Ep */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep[ep_idx].doepctl |= USB_DWC2_DEPCTL_USBACTEP;
		usb_dwc2_ctrl.out_ep_ctrl[ep_idx].ep_ena = 1U;
	} else {
		base->in_ep[ep_idx].diepctl |= USB_DWC2_DEPCTL_USBACTEP;
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].ep_ena = 1U;
	}

#if CONFIG_USB_DC_RTL87X2G_DMA
	if (USB_EP_DIR_IS_OUT(ep)) {
		usb_dwc2_ctrl.aOutXfer[ep_idx] =
			k_heap_alloc(&ep_heap, sizeof(outxfer_t), K_NO_WAIT);
		if (usb_dwc2_ctrl.aOutXfer[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			return -ENOMEM;
		}

		usb_dwc2_ctrl.aOutDmaDesc[ep_idx] =
			k_heap_aligned_alloc(&ep_heap, 4, sizeof(OutDmaDesc_t), K_NO_WAIT);
		if (usb_dwc2_ctrl.aOutDmaDesc[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			k_heap_free(&ep_heap, usb_dwc2_ctrl.aOutXfer[ep_idx]);
			return -ENOMEM;
		}

		usb_dwc2_ctrl.ep_rx_buf[ep_idx] =
			k_heap_alloc(&ep_heap, usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps, K_NO_WAIT);
		if (usb_dwc2_ctrl.ep_rx_buf[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			k_heap_free(&ep_heap, usb_dwc2_ctrl.aOutXfer[ep_idx]);
			k_heap_free(&ep_heap, usb_dwc2_ctrl.aOutDmaDesc[ep_idx]);
			return -ENOMEM;
		}

	} else {
		usb_dwc2_ctrl.aInXfer[ep_idx] = k_heap_alloc(&ep_heap, sizeof(inxfer_t), K_NO_WAIT);
		if (usb_dwc2_ctrl.aInXfer[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			return -ENOMEM;
		}
		usb_dwc2_ctrl.aInDmaDesc[ep_idx] =
			k_heap_aligned_alloc(&ep_heap, 4, sizeof(InDmaDesc_t), K_NO_WAIT);
		if (usb_dwc2_ctrl.aInDmaDesc[ep_idx] == NULL) {
			LOG_ERR("Failed to allocate memory");
			k_heap_free(&ep_heap, usb_dwc2_ctrl.aInXfer[ep_idx]);
			return -ENOMEM;
		}
	}
#endif

	if (USB_EP_DIR_IS_OUT(ep) &&
	    usb_dwc2_ctrl.out_ep_ctrl[ep_idx].cb != usb_transfer_ep_callback) {
		/* Start reading now, except for transfer managed eps */
		usb_dwc2_prep_rx(ep, 0);
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	if (!usb_power_is_on) {
		return 0;
	}
	LOG_DBG("line%d", __LINE__);
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] ep%x", __func__, ep_idx);
#endif

	if (!usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Disable EP interrupts */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->daintmsk &= ~USB_DWC2_DAINT_OUTEPINT(ep_idx);
		base->doepmsk &= USB_DWC2_DOEPINT_SETUP;
	} else {
		base->daintmsk &= ~USB_DWC2_DAINT_INEPINT(ep_idx);
		base->diepmsk &= USB_DWC2_DIEPINT_XFERCOMPL;
		base->gintmsk &= USB_DWC2_GINTSTS_RXFLVL;
	}

	/* De-activate, disable and set NAK for Ep */
	if (USB_EP_DIR_IS_OUT(ep)) {
		base->out_ep[ep_idx].doepctl &=
			~(USB_DWC2_DEPCTL_USBACTEP | USB_DWC2_DEPCTL_EPENA | USB_DWC2_DEPCTL_SNAK);
		usb_dwc2_ctrl.out_ep_ctrl[ep_idx].ep_ena = 0U;
	} else {
		base->in_ep[ep_idx].diepctl &=
			~(USB_DWC2_DEPCTL_USBACTEP | USB_DWC2_DEPCTL_EPENA | USB_DWC2_DEPCTL_SNAK);
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].ep_ena = 0U;
	}

#if CONFIG_USB_DC_RTL87X2G_DMA
	if (USB_EP_DIR_IS_OUT(ep)) {
		k_heap_free(&ep_heap, usb_dwc2_ctrl.aOutXfer[ep_idx]);
		k_heap_free(&ep_heap, usb_dwc2_ctrl.aOutDmaDesc[ep_idx]);
		k_heap_free(&ep_heap, usb_dwc2_ctrl.ep_rx_buf[ep_idx]);
	} else {
		k_heap_free(&ep_heap, usb_dwc2_ctrl.aInXfer[ep_idx]);
		k_heap_free(&ep_heap, usb_dwc2_ctrl.aInDmaDesc[ep_idx]);
	}
#endif

	return 0;
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint32_t cnt;

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		/* RX FIFO is global and cannot be flushed per EP */
		return -EINVAL;
	}

	/* Each endpoint has dedicated Tx FIFO */
	base->grstctl |= ep_idx << USB_DWC2_GRSTCTL_TXFNUM_POS;
	base->grstctl |= USB_DWC2_GRSTCTL_TXFFLSH;

	cnt = 0U;

	do {
		if (++cnt > USB_DWC2_CORE_RST_TIMEOUT_US) {
			LOG_ERR("USB TX FIFO flush HANG!");
			return -EIO;
		}
		usb_dwc2_udelay(1);
	} while (base->grstctl & USB_DWC2_GRSTCTL_TXFFLSH);

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] ep%x data_len=%d", __func__, ep, data_len);
#endif
	int ret;

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Check if IN ep */
	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_IN) {
		return -EINVAL;
	}

	/* Check if ep enabled */
	if (!usb_dwc2_ep_is_enabled(ep)) {
		return -EINVAL;
	}

#if CONFIG_USB_DC_RTL87X2G_DMA
	ret = usb_dwc2_tx_dma(ep, data, data_len);
#else
	ret = usb_dwc2_tx(ep, data, data_len);
#endif

	if (ret < 0) {
		return ret;
	}

	if (ret_bytes) {
		*ret_bytes = ret;
	}

	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
#if CONFIG_USB_DC_RTL87X2G_DMA
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (data) {
		memcpy(data, usb_dwc2_ctrl.ep_rx_buf[ep_idx], max_data_len);
		return 0;
	} else {
		return -EINVAL;
	}
#else

	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint32_t i, j, data_len, bytes_to_copy;

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Check if OUT ep */
	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_OUT) {
		LOG_ERR("Wrong endpoint direction");
		return -EINVAL;
	}

	/* Allow to read 0 bytes */
	if (!data && max_data_len) {
		LOG_ERR("Wrong arguments");
		return -EINVAL;
	}

	/* Check if ep enabled */
	if (!usb_dwc2_ep_is_enabled(ep)) {
		LOG_ERR("Not enabled endpoint");
		return -EINVAL;
	}

	data_len = usb_dwc2_ctrl.out_ep_ctrl[ep_idx].data_len;

	if (!data && !max_data_len) {
		/* When both buffer and max data to read are zero return
		 * the available data in buffer
		 */
		if (read_bytes) {
			*read_bytes = data_len;
		}
		return 0;
	}

	if (data_len > max_data_len) {
		LOG_ERR("Not enough room to copy all the rcvd data!");
		bytes_to_copy = max_data_len;
	} else {
		bytes_to_copy = data_len;
	}

	LOG_DBG("Read EP%d, req %d, read %d bytes", ep, max_data_len, bytes_to_copy);

	/* Data in the FIFOs is always stored per 32-bit words */
	for (i = 0U; i < (bytes_to_copy & ~0x3); i += 4U) {
		*(uint32_t *)(data + i) = USB_DWC2_EP_FIFO(base, ep_idx);
	}
	if (bytes_to_copy & 0x3) {
		/* Not multiple of 4 */
		uint32_t last_dw = USB_DWC2_EP_FIFO(base, ep_idx);

		for (j = 0U; j < (bytes_to_copy & 0x3); j++) {
			*(data + i + j) = (sys_cpu_to_le32(last_dw) >> (j * 8U)) & 0xFF;
		}
	}

	usb_dwc2_ctrl.out_ep_ctrl[ep_idx].data_len -= bytes_to_copy;

	if (read_bytes) {
		*read_bytes = bytes_to_copy;
	}

	return 0;
#endif
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	/* Check if OUT ep */
	if (USB_EP_GET_DIR(ep) != USB_EP_DIR_OUT) {
		LOG_ERR("Wrong endpoint direction");
		return -EINVAL;
	}

	if (!usb_dwc2_ctrl.out_ep_ctrl[ep_idx].data_len) {
		usb_dwc2_prep_rx(ep_idx, 0);
	}

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (!data && !max_data_len) {
		/* When both buffer and max data to read are zero the above
		 * call would fetch the data len and we simply return.
		 */
		return 0;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(ep)) {
		usb_dwc2_ctrl.in_ep_ctrl[ep_idx].cb = cb;
	} else {
		usb_dwc2_ctrl.out_ep_ctrl[ep_idx].cb = cb;
	}

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	usb_dwc2_ctrl.status_cb = cb;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	enum usb_dwc2_out_ep_idx ep_idx = USB_EP_GET_IDX(ep);

	if (!usb_dwc2_ctrl.attached || !usb_dwc2_ep_is_valid(ep)) {
		LOG_ERR("Not attached / Invalid endpoint: EP 0x%x", ep);
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		return usb_dwc2_ctrl.out_ep_ctrl[ep_idx].mps;
	} else {
		return usb_dwc2_ctrl.in_ep_ctrl[ep_idx].mps;
	}
}

int usb_dc_wakeup_request(void)
{
	usb_rtk_resume_sequence();
	SoC_VENDOR->u_004.REG_LOW_PRI_INT_STATUS |= BIT31;
	struct usb_dwc2_reg *const base = usb_dwc2_cfg.base;

	if (!(base->dsts & USB_DWC2_DSTS_SUSPSTS)) {
		LOG_ERR("Remote wakeup while is not in suspend state, or while is not allowed by "
			"host.");
		return -EAGAIN;
	}

	base->dctl |= USB_DWC2_DCTL_RMTWKUPSIG;

	k_sleep(K_MSEC(2));

	base->dctl &= ~USB_DWC2_DCTL_RMTWKUPSIG;

	LOG_DBG("Remote wakeup from suspend.");

	return 0;
}

static void usb_isr_suspend_enable(void)
{
#ifdef CONFIG_PM
	AON_REG7X_SYS_TYPE reg7x;

	reg7x.d32 = HAL_READ32(SYSTEM_REG_BASE, AON_REG7X_SYS);
	reg7x.usb_wakeup_sel = 1;
	reg7x.USB_WKPOL = 0;
	reg7x.USB_WKEN = 1;
	HAL_WRITE32(SYSTEM_REG_BASE, AON_REG7X_SYS, reg7x.d32);
#endif

	/* edge trigger */
	SoC_VENDOR->u_008.REG_LOW_PRI_INT_MODE |= BIT31;

	/* resing edge trigger */
	SoC_VENDOR->u_018.INTERRUPT_EDGE_OPTION &= ~(BIT31);

	/* Note: must disable at disable flow */
	SoC_VENDOR->u_00C.REG_LOW_PRI_INT_EN |= BIT31;
}

#ifdef CONFIG_PM
extern int hal_usb_wakeup_status_clear(void);
extern int hal_usb_wakeup_status_get(void);
extern void usb_set_pon_domain(void);
extern void usb_dm_start_from_dlps(void);

void usb_start_from_dlps(void)
{
	if (usb_rtk_resume_sequence() != 0) {
		usb_rtk_disable_power_seq();
		return;
	}

	usb_dwc2_cfg.irq_enable_func(NULL);

	return;
}

static PMCheckResult usb_pm_check(void)
{
	volatile enum usb_dc_status_code usb_state = usb_dwc2_ctrl.current_status;

	if (usb_state == USB_DC_SUSPEND || usb_state == USB_DC_DISCONNECTED) {
		return PM_CHECK_PASS;
	} else {
		return PM_CHECK_FAIL;
	}
}

static void usb_pm_store(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("======================usb_pm_store======================");
#endif
	hal_usb_wakeup_status_clear();
	AON_REG8X_SYS_TYPE reg8x;

	reg8x.d32 = AON_REG_READ(AON_REG8X_SYS);
}

static void usb_pm_restore(void)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("======================usb_pm_restore======================");
#endif
	if (hal_usb_wakeup_status_get() == 0) {
	} else {
		usb_start_from_dlps();
	}
}

static void usb_register_dlps_cb(void)
{
	usb_set_pon_domain();

	platform_pm_register_callback_func_with_priority((void *)usb_pm_check, PLATFORM_PM_CHECK,
							 1);
	platform_pm_register_callback_func_with_priority((void *)usb_pm_store, PLATFORM_PM_STORE,
							 1);
	platform_pm_register_callback_func_with_priority((void *)usb_pm_restore,
							 PLATFORM_PM_RESTORE, 1);
}
#endif
