/* ieee802154_nrf5.c - nRF5 802.15.4 driver */

/*
 * Copyright (c) 2017-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_ieee802154

#define LOG_MODULE_NAME ieee802154_rtl87x2g
#if defined(CONFIG_IEEE802154_DRIVER_LOG_LEVEL)
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL
#else
#define LOG_LEVEL LOG_LEVEL_NONE
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/debug/stack.h>

#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/debug/stack.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <zephyr/net/openthread.h>
#include <zephyr/net/ieee802154_radio_openthread.h>
#endif

#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/random/random.h>

#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/irq.h>

#include "ieee802154_rtl87x2g.h"
#include "mac_driver.h"
#include "mac_802154_frame_parser.h"
#include "trace.h"

typedef struct _fc_t {
	uint16_t type: 3;
	uint16_t sec_en: 1;
	uint16_t pending: 1;
	uint16_t ack_req: 1;
	uint16_t panid_compress: 1;
	uint16_t rsv: 1;
	uint16_t seq_num_suppress: 1;
	uint16_t ie_present: 1;
	uint16_t dst_addr_mode: 2;
	uint16_t ver: 2;
	uint16_t src_addr_mode: 2;
} fc_t;

#define ADDR_MODE_NOT_PRESENT 0
#define ADDR_MODE_RSV         1
#define ADDR_MODE_SHORT       2
#define ADDR_MODE_EXTEND      3

#define FRAME_VER_2003 0
#define FRAME_VER_2006 1
#define FRAME_VER_2015 2

typedef struct _aux_sec_ctl_t {
	uint8_t sec_level: 3;
	uint8_t key_id_mode: 2;
	uint8_t frame_counter_supp: 1;
	uint8_t asn_in_nonce: 1;
	uint8_t rsv: 1;
} aux_sec_ctl_t;

#define FCS_LEN 2

#define SHORT_ADDRESS_SIZE 2
#define US_PER_MS          1000ULL

#define ACK_REQUEST_OFFSET      1
#define ACK_REQUEST_BIT         (1 << 5)
#define FRAME_PENDING_OFFSET    1
#define FRAME_PENDING_BIT       (1 << 4)
#define SECURITY_ENABLED_OFFSET 1
#define SECURITY_ENABLED_BIT    (1 << 3)

#define IEEE802154_MAX_LENGTH (127)
#define MAC_FRAME_TX_HDR_LEN  (2)
#define MAC_FRAME_RX_HDR_LEN  (1)
#define MAC_FRAME_RX_TAIL_LEN (7)
#define MAC_FRAME_IMMACK_LEN  (3)

static bool sDisabled;
static uint8_t sChannel;
static uint16_t sPanid;

typedef struct {
	uint8_t sReceivedPsdu[MAC_FRAME_RX_HDR_LEN + IEEE802154_MAX_LENGTH + MAC_FRAME_RX_TAIL_LEN];
} rx_item_t;

static rx_item_t rx_buffer[RTL87X2G_802154_RX_BUFFERS];

#define NONE        0
#define RX_OK       1
#define TX_WAIT_ACK 2
#define TX_OK       3
#define TX_CCA_FAIL 4
#define TX_NO_ACK   5
#define ED_SCAN     6
#define ENTER_SLEEP 7
#define US_ALARM    8
#define MS_ALARM    9
#define UART_TX     10

#define MAX_TRANSMIT_RETRY 16
static uint32_t sTransmitRetry;

/* rx ack frame */
static uint8_t sAckWaitingSeq;
static rx_item_t sAckItem;

/* tx ack frame */
static volatile bool sAckedWithFramePending;

/* tx enhack frame */
static uint8_t sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + IEEE802154_MAX_LENGTH];
static volatile uint8_t enhack_frm_len;
static volatile bool enhack_frm_sec_en;

typedef struct {
	uint16_t len: 7;
	uint16_t id: 8;
	uint16_t type: 1;
	uint16_t phase;
	uint16_t period;
} __packed csl_ie_t;

typedef struct {
	uint16_t len: 7;
	uint16_t id: 8;
	uint16_t type: 1;
	uint8_t oui[3];
	uint8_t subtype;
	uint8_t content[2];
} __packed vendor_ie_t;

static uint32_t sCslPeriod;
static uint32_t sCslSampleTime;
static uint8_t sCslIeIndex;

static uint8_t enhAckProbingDataLen;
static bool enhAckProbingWithLqi;
static bool enhAckProbingWithMargin;
static bool enhAckProbingWithRssi;
static uint8_t sVendorIeIndex;

static uint32_t sMacFrameCounter;
static uint8_t sPrevKeyId;
static uint8_t sCurrKeyId;
static uint8_t sNextKeyId;
static uint8_t sPrevKey[16];
static uint8_t sCurrKey[16];
static uint8_t sNextKey[16];

static struct rtl87x2g_802154_data rtl87x2g_data;

struct rtl87x2g_802154_config {
	void (*irq_config_func)(void);
};

extern void Zigbee_Handler_Patch(void);

static void rtl87x2g_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), Zigbee_Handler_Patch,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

static const struct rtl87x2g_802154_config rtl87x2g_radio_cfg = {
	.irq_config_func = rtl87x2g_irq_config,
};

static void dataInit(void)
{
	for (uint8_t i = 0; i < RTL87X2G_802154_RX_BUFFERS; i++) {
		rtl87x2g_data.rx_frames[i].psdu = rx_buffer[i].sReceivedPsdu;
		rtl87x2g_data.rx_frames[i].used = false;
	}

	rtl87x2g_data.ack_frame.psdu = sAckItem.sReceivedPsdu;

	sMacFrameCounter = 0;
}

static void txAckProcessSecurity(uint8_t *aAckFrame)
{
	uint8_t *key;
	uint8_t keyId;

	struct {
		uint8_t sec_level;
		uint32_t frame_counter;
		uint64_t src_ext_addr;
	} __packed nonce;

	keyId = mac_GetRxFrmSecKeyId();

	if (keyId == sCurrKeyId) {
		key = sCurrKey;
	} else if (keyId == sPrevKeyId) {
		key = sPrevKey;
	} else if (keyId == sNextKeyId) {
		key = sNextKey;
	} else {
		__ASSERT(false, "invalid keyId %d", keyId);
	}

	nonce.sec_level = SECURITY_LEVEL_ENC_MIC_32;
	nonce.frame_counter = sMacFrameCounter;
	mac_memcpy(&nonce.src_ext_addr, MAC_EADR_BASE_ADDR(), 8);

	/* set nonce */
	mac_LoadNonce((uint8_t *)&nonce);
	/* set key */
	mac_LoadTxEnhAckKey(key);
	/* set security level */
	mac_SetTxEnhAckChiper(nonce.sec_level);
}

static uint16_t getCslPhase(void)
{
	uint32_t curTime = (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
	uint32_t cslPeriodInUs = sCslPeriod * 160;
	uint32_t diff =
		(cslPeriodInUs - (curTime % cslPeriodInUs) + (sCslSampleTime % cslPeriodInUs)) %
		cslPeriodInUs;
	return (uint16_t)(diff / 160 + 1);
}

void BEE_tx_ack_started(uint8_t *p_data, int8_t rssi, uint8_t lqi)
{
	csl_ie_t *p_csl_ie;
	vendor_ie_t *p_vendor_ie;
	uint8_t vendor_ie_content_index;

	if (sCslPeriod > 0) {
		p_csl_ie = (csl_ie_t *)&sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + sCslIeIndex];
		p_csl_ie->phase = getCslPhase();
		p_csl_ie->period = sCslPeriod;
	}

	if (enhAckProbingDataLen > 0) {
		p_vendor_ie = (vendor_ie_t *)&sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + sVendorIeIndex];
		p_vendor_ie->oui[0] = 0x9b;
		p_vendor_ie->oui[1] = 0xb8;
		p_vendor_ie->oui[2] = 0xea;
		p_vendor_ie->subtype = 0x00;
		vendor_ie_content_index = 0;
		if (enhAckProbingWithLqi) {
			p_vendor_ie->content[vendor_ie_content_index++] = lqi;
		}
		if (enhAckProbingWithMargin) {
			p_vendor_ie->content[vendor_ie_content_index++] = rssi;
		}
		if (enhAckProbingWithRssi) {
			p_vendor_ie->content[vendor_ie_content_index++] = rssi;
		}
	}
}

bool readFrame(struct rtl87x2g_802154_rx_frame *receivedFrame)
{
	pmac_rxfifo_t prx_fifo = (pmac_rxfifo_t)&receivedFrame->psdu[0];
	mac_rxfifo_tail_t *prx_fifo_tail;
	uint8_t channel;
	int8_t rssi;
	uint8_t lqi;
	uint8_t *p_data;
	fc_t *p_fc;

	/*
	 *	if (mac_GetRxFrmSecEn() && mac_GetRxFrmAckReq() && 0x2 ==
	 *mac_GetRxFrmSecKeyIdMode())
	 *	{
	 *		return false;
	 *	}
	 */

	mac_Rx((uint8_t *)prx_fifo);
	prx_fifo_tail = (mac_rxfifo_tail_t *)&receivedFrame->psdu[1 + prx_fifo->frm_len];
	channel = mac_GetChannel();
	rssi = mac_GetRSSIFromRaw(prx_fifo_tail->rssi, channel);
	lqi = prx_fifo_tail->lqi;

	if (enhack_frm_len > 0 && mac_GetTxEnhAckPending()) {
		BEE_tx_ack_started(&sEnhAckPsdu[1], rssi, lqi);
		if (enhack_frm_sec_en) {
			mac_LoadTxEnhAckPayload(enhack_frm_len, enhack_frm_len, &sEnhAckPsdu[2]);
			txAckProcessSecurity(&sEnhAckPsdu[1]);
		} else {
			mac_LoadTxEnhAckPayload(0, enhack_frm_len, &sEnhAckPsdu[2]);
		}
		if (mac_TrigTxEnhAck_patch(false, enhack_frm_sec_en) == MAC_STS_SUCCESS) {
			enhack_frm_len = 0;
			if (enhack_frm_sec_en) {
				enhack_frm_sec_en = false;
				sMacFrameCounter++;
			}
		} else {
			DBG_DIRECT("enhack tx timeout");
		}
	}

	p_data = &receivedFrame->psdu[0];
	p_fc = (fc_t *)&receivedFrame->psdu[1];
	receivedFrame->rssi = rssi;
	receivedFrame->lqi = lqi;

	/* Inform if this frame was acknowledged with frame pending set. */
	if (p_data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT) {
		if (p_fc->ver == FRAME_VER_2015) {
			receivedFrame->ack_fpb = rtl87x2g_data.last_frame_ack_fpb;
		} else {
			if (mac_GetPendingBit()) {
				mac_SetPendingBit(false);
			}
			receivedFrame->ack_fpb = rtl87x2g_data.last_frame_ack_fpb;
		}
	} else {
		receivedFrame->ack_fpb = false;
	}

	/* 0x7E header cmdid propid STREAM_RAW crc16 0x7E */
	/* Get the timestamp when the SFD was received */
	receivedFrame->time =
		k_ticks_to_us_floor64(k_uptime_ticks()) - (p_data[0] * 32) - PHY_HDR_SYMBOL_TIME_US;

	return true;
}

void readAck(struct rtl87x2g_802154_rx_frame *receivedFrame)
{
	pmac_rxfifo_t prx_fifo = (pmac_rxfifo_t)&receivedFrame->psdu[0];
	mac_rxfifo_tail_t *prx_fifo_tail;
	uint8_t *p_data;

	mac_Rx((uint8_t *)prx_fifo);

	p_data = &receivedFrame->psdu[0];
	prx_fifo_tail = (mac_rxfifo_tail_t *)&receivedFrame->psdu[1 + prx_fifo->frm_len];
	receivedFrame->lqi = prx_fifo_tail->lqi;
	receivedFrame->rssi = mac_GetRSSIFromRaw(prx_fifo_tail->rssi, mac_GetChannel());

	/* Get the timestamp when the SFD was received */
	receivedFrame->time =
		k_ticks_to_us_floor64(k_uptime_ticks()) - (p_data[0] * 32) - PHY_HDR_SYMBOL_TIME_US;
}

void BEE_tx_started(uint8_t *p_data)
{
}

void txProcessSecurity(uint8_t *aFrame)
{
	struct {
		uint8_t sec_level;
		uint32_t frame_counter;
		uint64_t src_ext_addr;
	} __packed nonce;
	nonce.sec_level = SECURITY_LEVEL_ENC_MIC_32;
	nonce.frame_counter = sMacFrameCounter;
	mac_memcpy((uint8_t *)&nonce.src_ext_addr, MAC_EADR_BASE_ADDR(), 8);

	/* set nonce */
	mac_LoadNonce((uint8_t *)&nonce);
	/* set key */
	mac_LoadTxNKey(sCurrKey);
	/* set security level */
	mac_SetTxNChiper(SECURITY_LEVEL_ENC_MIC_32);
}

static uint8_t generate_ieee_enhack_frame(uint8_t *txbuf, uint8_t buf_len, fc_t *fc, uint8_t seq,
					  uint16_t dpid, uint8_t *dadr, uint8_t aux_len,
					  uint8_t *aux)
{
	uint8_t len = 0;
	csl_ie_t *p_csl_ie;
	vendor_ie_t *p_vendor_ie;

	/* frame control */
	mac_memcpy((void *)(txbuf + len), (void *)fc, 2);
	len += 2;

	if (!fc->seq_num_suppress) {
		/* sequence number */
		txbuf[len] = seq;
		len += 1;
	}

	{
		if (!fc->panid_compress) {
			mac_memcpy(&txbuf[len], &dpid, 2);
			len += 2;
		}
		if (fc->dst_addr_mode > 0) {
			if (fc->dst_addr_mode == ADDR_MODE_SHORT) {
				mac_memcpy(&txbuf[len], dadr, 2);
				len += 2;
			} else {
				mac_memcpy(&txbuf[len], dadr, 8);
				len += 8;
			}
		}
	}

	if (fc->sec_en) {
		mac_memcpy((void *)(txbuf + len), (void *)aux, aux_len);
		len += aux_len;
	}

	if (fc->ie_present) {
		if (sCslPeriod > 0) {
			sCslIeIndex = len;
			p_csl_ie = (csl_ie_t *)&txbuf[sCslIeIndex];
			p_csl_ie->len = 4;
			p_csl_ie->id = 0x1a;
			p_csl_ie->type = 0;
			len += sizeof(csl_ie_t);
		} else {
			sCslIeIndex = 0;
		}

		if (enhAckProbingDataLen > 0) {
			sVendorIeIndex = len;
			p_vendor_ie = (vendor_ie_t *)&txbuf[sVendorIeIndex];
			p_vendor_ie->len = 6;
			p_vendor_ie->id = 0;
			p_vendor_ie->type = 0;
			len += sizeof(vendor_ie_t);
		} else {
			sVendorIeIndex = 0;
		}
	} else {
		sCslIeIndex = 0;
		sVendorIeIndex = 0;
	}

	return len;
}

#define ACK_REQUEST_BYTE   1
#define ACK_REQUEST_BIT    (1 << 5)
#define FRAME_PENDING_BYTE 1
#define FRAME_PENDING_BIT  (1 << 4)

#define DRX_SLOT_RX 0 /* Delayed reception window ID */

/* Convenience defines for RADIO */
#define RTL87X2G_802154_DATA(dev) ((struct rtl87x2g_802154_data *const)(dev)->data)

#define RTL87X2G_802154_CFG(dev) ((const struct rtl87x2g_802154_config *const)(dev)->config)

#if CONFIG_IEEE802154_VENDOR_OUI_ENABLE
#define IEEE802154_RTL87X2G_VENDOR_OUI CONFIG_IEEE802154_VENDOR_OUI
#else
#define IEEE802154_RTL87X2G_VENDOR_OUI (uint32_t)0xF4CE36
#endif

static void rtl87x2g_get_eui64(uint8_t *mac)
{
	mac[0] = 0x04;
	mac[1] = 0x0e;
	mac[2] = 0x0e;
	mac[3] = 0x0b;
	mac[4] = 0x00;
	mac[5] = 0x00;
	mac[6] = 0x55;
	mac[7] = 0xaa;
}

static void rtl87x2g_rx_thread(void *arg1, void *arg2, void *arg3)
{
	struct rtl87x2g_802154_data *rtl87x2g_radio = (struct rtl87x2g_802154_data *)arg1;
	struct net_pkt *pkt;
	struct rtl87x2g_802154_rx_frame *rx_frame;
	uint8_t pkt_len;
	uint8_t pkt_seq;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		pkt = NULL;
		rx_frame = NULL;

		rx_frame = k_fifo_get(&rtl87x2g_radio->rx_fifo, K_FOREVER);

		__ASSERT_NO_MSG(rx_frame->psdu);

		/* rx_mpdu contains length, psdu, fcs|lqi
		 * The last 2 bytes contain LQI or FCS, depending if
		 * automatic CRC handling is enabled or not, respectively.
		 */
		pkt_len = rx_frame->psdu[0];
		pkt_seq = rx_frame->psdu[3];

#if defined(CONFIG_NET_BUF_DATA_SIZE)
		__ASSERT_NO_MSG(pkt_len <= CONFIG_NET_BUF_DATA_SIZE);
#endif

		LOG_DBG("Frame received %d %d", pkt_len, pkt_seq);

		/* Block the RX thread until net_pkt is available, so that we
		 * don't drop already ACKed frame in case of temporary net_pkt
		 * scarcity. The nRF 802154 radio driver will accumulate any
		 * incoming frames until it runs out of internal buffers (and
		 * thus stops acknowledging consecutive frames).
		 */
		pkt = net_pkt_rx_alloc_with_buffer(rtl87x2g_radio->iface, pkt_len, AF_UNSPEC, 0,
						   K_FOREVER);

		if (net_pkt_write(pkt, rx_frame->psdu + 1, pkt_len)) {
			goto drop;
		}

		net_pkt_set_ieee802154_lqi(pkt, rx_frame->lqi);
		net_pkt_set_ieee802154_rssi_dbm(pkt, rx_frame->rssi);
		net_pkt_set_ieee802154_ack_fpb(pkt, rx_frame->ack_fpb);

#if defined(CONFIG_NET_PKT_TIMESTAMP)
		net_pkt_set_timestamp_ns(pkt, rx_frame->time * NSEC_PER_USEC);
#endif

		if (net_recv_data(rtl87x2g_radio->iface, pkt) < 0) {
			LOG_ERR("Packet dropped by NET stack");
			goto drop;
		}

		rx_frame->used = false;

		if (LOG_LEVEL >= LOG_LEVEL_DBG) {
			log_stack_usage(&rtl87x2g_radio->rx_thread);
		}

		continue;

drop:
		rx_frame->used = false;

		net_pkt_unref(pkt);
	}
}

static void rtl87x2g_get_capabilities_at_boot(void)
{
	rtl87x2g_data.capabilities = IEEE802154_HW_FCS | IEEE802154_HW_PROMISC |
				     IEEE802154_HW_FILTER | IEEE802154_HW_CSMA |
				     IEEE802154_HW_TX_RX_ACK | IEEE802154_HW_RX_TX_ACK |
				     IEEE802154_HW_ENERGY_SCAN | IEEE802154_HW_TXTIME | 0UL |
				     IEEE802154_HW_SLEEP_TO_TX | IEEE802154_HW_TX_SEC;
}

/* Radio device API */

static enum ieee802154_hw_caps rtl87x2g_get_capabilities(const struct device *dev)
{
	return rtl87x2g_data.capabilities;
}

static int rtl87x2g_cca(const struct device *dev)
{
	return 0;
}

static int rtl87x2g_set_channel(const struct device *dev, uint16_t channel)
{
	ARG_UNUSED(dev);

	LOG_DBG("%u", channel);

	if (channel < 11 || channel > 26) {
		return channel < 11 ? -ENOTSUP : -EINVAL;
	}

	mac_SetChannel(channel);

	return 0;
}

static int rtl87x2g_energy_scan_start(const struct device *dev, uint16_t duration,
				      energy_scan_done_cb_t done_cb)
{
	int err = 0;
	int8_t avg;
	int8_t ed_value;

	ARG_UNUSED(dev);

	if (rtl87x2g_data.energy_scan_done == NULL) {
		rtl87x2g_data.energy_scan_done = done_cb;

		mac_EDScanExt(duration * 1000, &ed_value, &avg);
		rtl87x2g_data.energy_scan_done(net_if_get_device(rtl87x2g_data.iface), ed_value);

		rtl87x2g_data.energy_scan_done = NULL;
	} else {
		err = -EALREADY;
	}

	return err;
}

static int rtl87x2g_set_pan_id(const struct device *dev, uint16_t pan_id)
{
	ARG_UNUSED(dev);

	sPanid = pan_id;
	mac_SetPANId(pan_id);

	LOG_DBG("0x%x", pan_id);

	return 0;
}

static int rtl87x2g_set_short_addr(const struct device *dev, uint16_t short_addr)
{
	ARG_UNUSED(dev);

	mac_SetShortAddress(short_addr);

	LOG_DBG("0x%x", short_addr);

	return 0;
}

static int rtl87x2g_set_ieee_addr(const struct device *dev, const uint8_t *ieee_addr)
{
	ARG_UNUSED(dev);

	LOG_DBG("IEEE address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", ieee_addr[7], ieee_addr[6],
		ieee_addr[5], ieee_addr[4], ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);

	mac_SetLongAddress(ieee_addr);

	return 0;
}

static int rtl87x2g_filter(const struct device *dev, bool set, enum ieee802154_filter_type type,
			   const struct ieee802154_filter *filter)
{
	LOG_DBG("Applying filter %u", type);

	if (!set) {
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		return rtl87x2g_set_ieee_addr(dev, filter->ieee_addr);
	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		return rtl87x2g_set_short_addr(dev, filter->short_addr);
	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		return rtl87x2g_set_pan_id(dev, filter->pan_id);
	}

	return -ENOTSUP;
}

static int rtl87x2g_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);

	LOG_DBG("%d", dbm);

	mac_SetTXPower(dbm);

	return 0;
}

static int handle_ack(struct rtl87x2g_802154_data *rtl87x2g_radio)
{
	uint8_t ack_len;
	struct net_pkt *ack_pkt;
	int err = 0;

	ack_len = rtl87x2g_radio->ack_frame.psdu[0];

	ack_pkt = net_pkt_rx_alloc_with_buffer(rtl87x2g_radio->iface, ack_len, AF_UNSPEC, 0,
					       K_NO_WAIT);
	if (!ack_pkt) {
		LOG_ERR("No free packet available.");
		err = -ENOMEM;
		goto free_drv_ack;
	}

	/* Upper layers expect the frame to start at the MAC header, skip the
	 * PHY header (1 byte).
	 */
	if (net_pkt_write(ack_pkt, rtl87x2g_radio->ack_frame.psdu + 1, ack_len) < 0) {
		LOG_ERR("Failed to write to ack_pkt.");
		err = -ENOMEM;
		goto free_net_ack;
	}

	net_pkt_set_ieee802154_lqi(ack_pkt, rtl87x2g_radio->ack_frame.lqi);
	net_pkt_set_ieee802154_rssi_dbm(ack_pkt, rtl87x2g_radio->ack_frame.rssi);

#if defined(CONFIG_NET_PKT_TIMESTAMP)
	net_pkt_set_timestamp_ns(ack_pkt, rtl87x2g_radio->ack_frame.time * NSEC_PER_USEC);
#endif

	net_pkt_cursor_init(ack_pkt);

	if (ieee802154_handle_ack(rtl87x2g_radio->iface, ack_pkt) != NET_OK) {
		LOG_ERR("ACK packet not handled - releasing.");
	}

free_net_ack:
	net_pkt_unref(ack_pkt);

free_drv_ack:

	return err;
}

static void rtl87x2g_tx_started(const struct device *dev, struct net_pkt *pkt, struct net_buf *frag)
{
	ARG_UNUSED(pkt);

	if (rtl87x2g_data.event_handler) {
		rtl87x2g_data.event_handler(dev, IEEE802154_EVENT_TX_STARTED, (void *)frag);
	}
}

static int rtl87x2g_tx(const struct device *dev, enum ieee802154_tx_mode mode, struct net_pkt *pkt,
		       struct net_buf *frag)
{
	struct rtl87x2g_802154_data *rtl87x2g_radio = RTL87X2G_802154_DATA(dev);
	uint8_t payload_len = frag->len;
	uint8_t *payload = frag->data;
	fc_t *p_fc = (fc_t *)payload;
	uint8_t *raw = &payload[-1];
	uint8_t sec_ctl_index;
	aux_sec_ctl_t *p_sec_ctl;
	csl_ie_t *p_csl_ie;
	uint8_t *p_header;
	uint8_t header_len;
	uint8_t *p_ie_header;
	uint16_t *p_ie_ht1;
	uint8_t *txn_fifo = (uint8_t *)MAC_TXN_BASE_ADDR;
	uint64_t now;
	uint64_t target_us;
	bool tx_done = false;
	int ret;

	if (payload_len > IEEE802154_MTU) {
		LOG_ERR("Payload too large: %d", payload_len);
		return -EMSGSIZE;
	}

	if (p_fc->sec_en) {
		sec_ctl_index = mac_802154_frame_parser_sec_ctrl_offset_get(raw);
		p_sec_ctl = (aux_sec_ctl_t *)&raw[sec_ctl_index];

		if (p_sec_ctl->key_id_mode == 1) {
			/* set frame_counter */
			mac_memcpy(&raw[sec_ctl_index + 1], &sMacFrameCounter, 4);

			if (p_fc->ie_present) {
				p_ie_header = mac_802154_frame_parser_ie_header_get(raw);
#if defined(CONFIG_IEEE802154_CSL_ENDPOINT)
				if (sCslPeriod > 0) {
					p_csl_ie = (csl_ie_t *)p_ie_header;
					p_csl_ie->phase = getCslPhase();
					p_csl_ie->period = sCslPeriod;
				}
#endif
				p_ie_ht1 = (uint16_t *)p_ie_header;
				while (*p_ie_ht1 != 0x3f80) {
					p_ie_header++;
					p_ie_ht1 = (uint16_t *)p_ie_header;
				}
				p_ie_header += 2;
				header_len = (p_ie_header - payload);
			} else {
				p_header = mac_802154_frame_parser_sec_ctrl_get(raw);
				p_header += (1 + 4 + 1);
				header_len = (p_header - payload);
			}

			if (p_fc->type == FRAME_TYPE_COMMAND) {
				mac_LoadTxNPayload_patch(header_len + 1, payload_len - 4, payload);
			} else {
				mac_LoadTxNPayload_patch(header_len, payload_len - 4, payload);
			}
			txProcessSecurity(NULL);
			mac_TrigUpperEnc_patch();
			sMacFrameCounter++;

			mac_memcpy(payload, &txn_fifo[MAC_FRAME_TX_HDR_LEN], payload_len);
			net_pkt_set_ieee802154_frame_secured(pkt, true);
			net_pkt_set_ieee802154_mac_hdr_rdy(pkt, true);
		} else {
			mac_LoadTxNPayload_patch(0, payload_len, payload);
		}
	} else {
		mac_LoadTxNPayload_patch(0, payload_len, payload);
	}

	/* Reset semaphore in case ACK was received after timeout */
	sTransmitRetry = 0;
	k_sem_reset(&rtl87x2g_radio->tx_wait);

	switch (mode) {
	case IEEE802154_TX_MODE_DIRECT:
	case IEEE802154_TX_MODE_CCA:
		DBG_DIRECT("TX mode %d not supported", mode);
		return -ENOTSUP;

	case IEEE802154_TX_MODE_CSMA_CA:
		now = k_ticks_to_us_floor64(k_uptime_ticks());
		target_us = now + mac_BackoffDelay();
		do {
			now = k_ticks_to_us_floor64(k_uptime_ticks());
		} while (now < target_us);
		mac_TrigTxN_patch(p_fc->ack_req, false, p_fc->ver == FRAME_VER_2015);
		break;

#if defined(CONFIG_NET_PKT_TXTIME)
	case IEEE802154_TX_MODE_TXTIME:
		DBG_DIRECT("TX mode %d not supported", mode);
		return -ENOTSUP;

	case IEEE802154_TX_MODE_TXTIME_CCA:
		now = k_ticks_to_us_floor64(k_uptime_ticks());
		target_us = (net_pkt_timestamp_ns(pkt) / NSEC_PER_USEC) - 128;
		if (now < target_us) {
			do {
				now = k_ticks_to_us_floor64(k_uptime_ticks());
			} while (now < target_us);
		}
		mac_TrigTxN_patch(p_fc->ack_req, false, p_fc->ver == FRAME_VER_2015);
		break;
#endif /* CONFIG_NET_PKT_TXTIME */

	default:
		DBG_DIRECT("TX mode %d not supported", mode);
		return -ENOTSUP;
	}

	rtl87x2g_tx_started(dev, pkt, frag);

	do {
		/* Wait for the callback from the radio driver. */
		k_sem_take(&rtl87x2g_radio->tx_wait, K_FOREVER);
		switch (rtl87x2g_radio->tx_result) {
		case TX_WAIT_ACK:
			handle_ack(rtl87x2g_radio);
			tx_done = true;
			ret = 0;
			break;

		case TX_OK:
			tx_done = true;
			ret = 0;
			break;

		case TX_CCA_FAIL:
			mac_RstRF();
			if (mode == IEEE802154_TX_MODE_TXTIME_CCA) {
				k_sem_reset(&rtl87x2g_radio->tx_wait);
				mac_TrigTxN_patch(p_fc->ack_req, false,
						  p_fc->ver == FRAME_VER_2015);
			} else {
				k_sem_reset(&rtl87x2g_radio->tx_wait);
				now = k_ticks_to_us_floor64(k_uptime_ticks());
				target_us = now + mac_BackoffDelay();
				do {
					now = k_ticks_to_us_floor64(k_uptime_ticks());
				} while (now < target_us);
				mac_TrigTxN_patch(p_fc->ack_req, false,
						  p_fc->ver == FRAME_VER_2015);
			}
			break;

		case TX_NO_ACK:
			mac_RstRF();
			if (mode == IEEE802154_TX_MODE_TXTIME_CCA) {
				tx_done = true;
				ret = -ENOMSG;
			} else {
				sTransmitRetry++;
				if (sTransmitRetry == MAX_TRANSMIT_RETRY) {
					tx_done = true;
					ret = -ENOMSG;
				} else {
					k_sem_reset(&rtl87x2g_radio->tx_wait);
					now = k_ticks_to_us_floor64(k_uptime_ticks());
					target_us = now + mac_BackoffDelay();
					do {
						now = k_ticks_to_us_floor64(k_uptime_ticks());
					} while (now < target_us);
					mac_TrigTxN_patch(p_fc->ack_req, false,
							  p_fc->ver == FRAME_VER_2015);
				}
			}
			break;

		default:
			tx_done = true;
			ret = -EIO;
			break;
		}
	} while (!tx_done);

	return ret;
}

static net_time_t rtl87x2g_get_time(const struct device *dev)
{
	ARG_UNUSED(dev);

	return (net_time_t)mac_GetCurrentMACTime() * NSEC_PER_USEC;
}

static uint8_t rtl87x2g_get_acc(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int rtl87x2g_start(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int rtl87x2g_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

void RCPInterruptHandler(uint32_t int_sts);

int32_t edscan_level2dbm(int32_t level)
{
	return (level * 2) - 90;
}

extern uint32_t (*lowerstack_SystemCall)(uint32_t opcode, uint32_t param, uint32_t param1,
					 uint32_t param2);
extern void set_zigbee_priority(uint16_t priority, uint16_t priority_min);
extern uint32_t get_zigbee_window_slot_imp(int16_t *prio, int16_t *prio_min);
extern void (*modem_set_zb_cca_combination)(uint8_t comb);

mac_attribute_t attr;
mac_driver_t drv;

static void rtl87x2g_init_internal(void)
{
	mac_InitAttribute(&attr);
	attr.mac_cfg.rf_early_term = 0;
	/* attr.mac_cfg.frm06_rx_early = 0; */

	lowerstack_SystemCall(10, 1, 512, -1);

	mac_Enable();
	mac_Initialize(&drv, &attr);
	mac_Initialize_Additional();
	mac_RegisterCallback(RCPInterruptHandler, edscan_level2dbm, set_zigbee_priority,
			     *modem_set_zb_cca_combination);
	mac_SetCcaMode(MAC_CCA_ED);
	mac_SetTxNCsma(false);
}

static int rtl87x2g_init(const struct device *dev)
{
	const struct rtl87x2g_802154_config *rtl87x2g_radio_cfg = RTL87X2G_802154_CFG(dev);
	struct rtl87x2g_802154_data *rtl87x2g_radio = RTL87X2G_802154_DATA(dev);

	k_fifo_init(&rtl87x2g_radio->rx_fifo);
	k_sem_init(&rtl87x2g_radio->tx_wait, 0, 1);

	/* enable irq */
	rtl87x2g_radio_cfg->irq_config_func();

	/* radio init */
	dataInit();

	rtl87x2g_init_internal();
	mac_RegisterBtHciResetHandler(rtl87x2g_init_internal);

	rtl87x2g_get_capabilities_at_boot();

	k_thread_create(&rtl87x2g_radio->rx_thread, rtl87x2g_radio->rx_stack,
			RTL87X2G_RX_STACK_SIZE, rtl87x2g_rx_thread, rtl87x2g_radio, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);

	k_thread_name_set(&rtl87x2g_radio->rx_thread, "rtl87x2g_rx");

	return 0;
}

static void rtl87x2g_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct rtl87x2g_802154_data *rtl87x2g_radio = RTL87X2G_802154_DATA(dev);

	rtl87x2g_get_eui64(rtl87x2g_radio->mac);
	net_if_set_link_addr(iface, rtl87x2g_radio->mac, sizeof(rtl87x2g_radio->mac),
			     NET_LINK_IEEE802154);

	rtl87x2g_radio->iface = iface;

	ieee802154_init(iface);
}

static void rtl87x2g_config_mac_keys(struct ieee802154_key *mac_keys)
{
	uint8_t i;

	for (i = 0; mac_keys->key_value && i < 3; mac_keys++, i++) {
		switch (i) {
		case 0:
			sPrevKeyId = mac_keys->key_index;
			mac_memcpy(sPrevKey, mac_keys->key_value, 16);
			break;

		case 1:
			sCurrKeyId = mac_keys->key_index;
			mac_memcpy(sCurrKey, mac_keys->key_value, 16);
			break;

		case 2:
			sNextKeyId = mac_keys->key_index;
			mac_memcpy(sNextKey, mac_keys->key_value, 16);
			break;

		default:
			break;
		}
	}
}

static int rtl87x2g_configure(const struct device *dev, enum ieee802154_config_type type,
			      const struct ieee802154_config *config)
{
	ARG_UNUSED(dev);

	switch (type) {
	case IEEE802154_CONFIG_AUTO_ACK_FPB:
		if (config->auto_ack_fpb.enabled) {
			switch (config->auto_ack_fpb.mode) {
			case IEEE802154_FPB_ADDR_MATCH_THREAD:
				mac_SetAddrMatchMode_patch(AUTO_ACK_PENDING_MODE_THREAD);
				break;

			case IEEE802154_FPB_ADDR_MATCH_ZIGBEE:
				mac_SetAddrMatchMode_patch(AUTO_ACK_PENDING_MODE_ZIGBEE);
				break;

			default:
				return -EINVAL;
			}
		}
		break;

	case IEEE802154_CONFIG_ACK_FPB:
		if (config->ack_fpb.enabled) {
			/* add */
			if (config->ack_fpb.extended) {
				mac_AddSrcExtAddrMatch(config->ack_fpb.addr);
			} else {
				uint16_t saddr;

				mac_memcpy(&saddr, config->ack_fpb.addr, sizeof(uint16_t));
				mac_AddSrcShortAddrMatch(saddr, sPanid);
			}
		} else {
			/* clear */
			if (config->ack_fpb.addr != NULL) {
				/* clear entry */
				if (config->ack_fpb.extended) {
					mac_DelSrcExtAddrMatch(config->ack_fpb.addr);
				} else {
					uint16_t saddr;

					mac_memcpy(&saddr, config->ack_fpb.addr, sizeof(uint16_t));
					mac_DelSrcShortAddrMatch(saddr, sPanid);
				}
			} else {
				/* clear all */
				if (config->ack_fpb.extended) {
					mac_DelAllSrcExtAddrMatch();
				} else {
					mac_DelAllSrcShortAddrMatch();
				}
			}
		}
		break;

	case IEEE802154_CONFIG_PAN_COORDINATOR:
		/* not support */
		break;

	case IEEE802154_CONFIG_PROMISCUOUS:
		mac_SetPromiscuous(config->promiscuous);
		break;

	case IEEE802154_CONFIG_EVENT_HANDLER:
		rtl87x2g_data.event_handler = config->event_handler;
		break;

	case IEEE802154_CONFIG_MAC_KEYS:
		rtl87x2g_config_mac_keys(config->mac_keys);
		break;

	case IEEE802154_CONFIG_FRAME_COUNTER:
		sMacFrameCounter = config->frame_counter;
		break;

	case IEEE802154_CONFIG_FRAME_COUNTER_IF_LARGER:
		sMacFrameCounter = config->frame_counter;
		break;

	case IEEE802154_CONFIG_ENH_ACK_HEADER_IE:
		uint16_t *p_header_ie = (uint16_t *)config->ack_ie.data;
		uint8_t vendor_ie_content_index;

		if (config->ack_ie.data_len > 0) {
			if (*p_header_ie == 0x0d04) {
			} else {
				enhAckProbingDataLen = config->ack_ie.data_len;
				vendor_ie_content_index = 6;
				while (vendor_ie_content_index < enhAckProbingDataLen) {
					switch (config->ack_ie.data[vendor_ie_content_index]) {
					case 0x01:
						enhAckProbingWithLqi = true;
						break;

					case 0x02:
						enhAckProbingWithMargin = true;
						break;

					case 0x03:
						enhAckProbingWithRssi = true;
						break;

					default:
						break;
					}
					vendor_ie_content_index++;
				}
			}
		} else {
			if (*p_header_ie == 0x0d04) {
			} else {
				enhAckProbingDataLen = 0;
				enhAckProbingWithLqi = false;
				enhAckProbingWithMargin = false;
				enhAckProbingWithRssi = false;
			}
		}
		break;

#if defined(CONFIG_IEEE802154_CSL_ENDPOINT)
	case IEEE802154_CONFIG_CSL_RX_TIME:
		sCslSampleTime = config->csl_rx_time / NSEC_PER_USEC;
		break;

	case IEEE802154_CONFIG_RX_SLOT:
		/* not support rx_at */
		break;

	case IEEE802154_CONFIG_CSL_PERIOD:
		sCslPeriod = config->csl_period;
		break;
#endif /* CONFIG_IEEE802154_CSL_ENDPOINT */

	default:
		return -EINVAL;
	}

	return 0;
}

/* driver-allocated attribute memory - constant across all driver instances */
IEEE802154_DEFINE_PHY_SUPPORTED_CHANNELS(drv_attr, 11, 26);

static int rtl87x2g_attr_get(const struct device *dev, enum ieee802154_attr attr,
			     struct ieee802154_attr_value *value)
{
	ARG_UNUSED(dev);
	if (ieee802154_attr_get_channel_page_and_range(
		    attr, IEEE802154_ATTR_PHY_CHANNEL_PAGE_ZERO_OQPSK_2450_BPSK_868_915,
		    &drv_attr.phy_supported_channels, value) == 0) {
		return 0;
	}

	switch ((uint32_t)attr) {
	default:
		return -ENOENT;
	}

	return 0;
}

void Process_RadioInt(uint32_t int_status)
{
	isrsts_t status;

	status.w = int_status;

	struct {
		aux_sec_ctl_t sec_ctl;
		uint32_t frame_counter;
		uint8_t key_id;
	} __packed aux;

	if (status.b.gntif) {
	}

	if (status.b.txnterrif) {
		/* currently not use txnt */
	}

	if (status.b.txnif) {
		uint8_t *txnfifo = (uint8_t *)MAC_TXN_BASE_ADDR;
		uint8_t tx_seq = txnfifo[4];
		fc_t *p_fc_tx = (fc_t *)&txnfifo[2];
		fc_t *p_fc_immack;
		uint8_t tx_status = mac_GetTxNStatus();

		if (tx_status & MAC_txsr_txns_Msk) {
			rtl87x2g_data.tx_result = TX_NO_ACK;
			k_sem_give(&rtl87x2g_data.tx_wait);
		} else {
			if (tx_status & MAC_txsr_ccafail_Msk) {
				rtl87x2g_data.tx_result = TX_CCA_FAIL;
				k_sem_give(&rtl87x2g_data.tx_wait);
			} else {
				if (p_fc_tx->ack_req) {
					sAckWaitingSeq = tx_seq;
				} else {
					rtl87x2g_data.ack_frame.used = false;
					rtl87x2g_data.tx_result = TX_OK;
					k_sem_give(&rtl87x2g_data.tx_wait);
				}
			}
		}
	}

	if (status.b.txg1if) {
	}

	if (status.b.txg2if) {
	}

	if (status.b.secif) {
		mac_IgnoreRxDec();
	}

	if (status.b.rxelyif) {
		fc_t fc_ack;

		do {
			if (mac_GetRxFrmVersion() == FRAME_VER_2015) {
				if (!mac_GetRxFrmAckReq()) {
					break;
				}

				uint16_t panid;
				uint16_t saddr;
				uint64_t laddr;

				enhack_frm_sec_en = mac_GetRxFrmSecEn();
				fc_ack.type = FRAME_TYPE_ACK;
				fc_ack.sec_en = enhack_frm_sec_en;
				if (mac_GetSrcMatchStatus()) {
					mac_ClrSrcMatchStatus();
					rtl87x2g_data.last_frame_ack_fpb = true;
				} else {
					rtl87x2g_data.last_frame_ack_fpb = false;
				}
				fc_ack.pending = sAckedWithFramePending;
				fc_ack.ack_req = 0;
				fc_ack.panid_compress = mac_GetRxFrmPanidCompress();
				fc_ack.rsv = 0;
				fc_ack.seq_num_suppress = 0;
				fc_ack.ie_present = 0;
				if (sCslPeriod > 0) {
					fc_ack.ie_present = 1;
				}
				if (enhAckProbingDataLen > 0) {
					fc_ack.ie_present = 1;
				}
				fc_ack.dst_addr_mode = mac_GetRxFrmSrcAddrMode();
				fc_ack.ver = FRAME_VER_2015;
				fc_ack.src_addr_mode = 0;

				if (enhack_frm_sec_en) {
					aux.sec_ctl.sec_level = mac_GetRxFrmSecLevel();
					aux.sec_ctl.key_id_mode = mac_GetRxFrmSecKeyIdMode();
					aux.sec_ctl.frame_counter_supp = 0;
					aux.sec_ctl.asn_in_nonce = 0;
					aux.sec_ctl.rsv = 0;
					/* set frame counter */
					aux.frame_counter = sMacFrameCounter;
					aux.key_id = mac_GetRxFrmSecKeyId();
				}

				panid = mac_GetPANId();

				if (fc_ack.dst_addr_mode == ADDR_MODE_SHORT) {
					saddr = mac_GetRxFrmShortAddr();
					enhack_frm_len = generate_ieee_enhack_frame(
						&sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN],
						IEEE802154_MAX_LENGTH, &fc_ack, mac_GetRxFrmSeq(),
						panid, (uint8_t *)&saddr, sizeof(aux),
						(uint8_t *)&aux);
				} else {
					laddr = mac_GetRxFrmLongAddr();
					enhack_frm_len = generate_ieee_enhack_frame(
						&sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN],
						IEEE802154_MAX_LENGTH, &fc_ack, mac_GetRxFrmSeq(),
						panid, (uint8_t *)&laddr, sizeof(aux),
						(uint8_t *)&aux);
				}

				sEnhAckPsdu[0] = 0;
				sEnhAckPsdu[1] = enhack_frm_len + 2;
				if (enhAckProbingDataLen > 0) {
					mac_SetTxEnhAckPending(enhack_frm_len);
				} else {
					BEE_tx_ack_started(&sEnhAckPsdu[1], 0, 0);
					if (enhack_frm_sec_en) {
						mac_LoadTxEnhAckPayload(enhack_frm_len,
									enhack_frm_len,
									&sEnhAckPsdu[2]);
						txAckProcessSecurity(&sEnhAckPsdu[1]);
					} else {
						mac_LoadTxEnhAckPayload(0, enhack_frm_len,
									&sEnhAckPsdu[2]);
					}
					mac_TrigTxEnhAck_patch(true, enhack_frm_sec_en);
					enhack_frm_len = 0;
					if (enhack_frm_sec_en) {
						enhack_frm_sec_en = false;
						sMacFrameCounter++;
					}
				}
			} else {
				if (!mac_GetRxFrmAckReq()) {
					break;
				}
				if (mac_GetSrcMatchStatus()) {
					mac_ClrSrcMatchStatus();
					mac_SetPendingBit(true);
					rtl87x2g_data.last_frame_ack_fpb = true;
				} else {
					rtl87x2g_data.last_frame_ack_fpb = false;
				}
			}
		} while (0);
	}

	if (status.b.rxif) {
		if (mac_GetRxFrmType() == FRAME_TYPE_ACK) {
			if (sAckWaitingSeq == mac_GetRxFrmSeq()) {
				readAck(&rtl87x2g_data.ack_frame);
				rtl87x2g_data.tx_result = TX_WAIT_ACK;
				k_sem_give(&rtl87x2g_data.tx_wait);
			} else {
				mac_RxFlush();
			}
		} else {
			/* find unused item */
			uint32_t i;

			for (i = 0; i < ARRAY_SIZE(rtl87x2g_data.rx_frames); i++) {
				if (!rtl87x2g_data.rx_frames[i].used) {
					break;
				}
			}
			rtl87x2g_data.rx_frames[i].used = true;

			readFrame(&rtl87x2g_data.rx_frames[i]);
			rtl87x2g_data.last_frame_ack_fpb = false;
			k_fifo_put(&rtl87x2g_data.rx_fifo, &rtl87x2g_data.rx_frames[i]);
		}
	}

	if (status.b.mactmrif) {
	}
}

void RCPInterruptHandler(uint32_t int_sts)
{
	Process_RadioInt(int_sts);
}

static struct ieee802154_radio_api rtl87x2g_radio_api = {.iface_api.init = rtl87x2g_iface_init,

							 .get_capabilities =
								 rtl87x2g_get_capabilities,
							 .cca = rtl87x2g_cca,
							 .set_channel = rtl87x2g_set_channel,
							 .filter = rtl87x2g_filter,
							 .set_txpower = rtl87x2g_set_txpower,
							 .start = rtl87x2g_start,
							 .stop = rtl87x2g_stop,

							 .tx = rtl87x2g_tx,
							 .ed_scan = rtl87x2g_energy_scan_start,

							 .get_sch_acc = rtl87x2g_get_acc,
							 .configure = rtl87x2g_configure,
							 .attr_get = rtl87x2g_attr_get};

#if defined(CONFIG_NET_L2_IEEE802154)
#define L2          IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define MTU         IEEE802154_MTU
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define L2          OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define MTU         1280
#elif defined(CONFIG_NET_L2_CUSTOM_IEEE802154)
#define L2          CUSTOM_IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(CUSTOM_IEEE802154_L2)
#define MTU         CONFIG_NET_L2_CUSTOM_IEEE802154_MTU
#endif

#if defined(CONFIG_NET_L2_PHY_IEEE802154)
NET_DEVICE_DT_INST_DEFINE(0, rtl87x2g_init, NULL, &rtl87x2g_data, &rtl87x2g_radio_cfg,
			  RTL87X2G_INIT_PRIO, &rtl87x2g_radio_api, L2, L2_CTX_TYPE, MTU);
#else
DEVICE_DT_INST_DEFINE(0, rtl87x2g_init, NULL, &rtl87x2g_data, &rtl87x2g_radio_cfg, POST_KERNEL,
		      RTL87X2G_INIT_PRIO, &rtl87x2g_radio_api);
#endif
