/* ieee802154_realtek_bee.c - bee 802.15.4 driver */

/*
 * Copyright (c) 2021-2026 Realtek Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_ieee802154

#define LOG_MODULE_NAME ieee802154_realtek_bee
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

#include "ieee802154_realtek_bee.h"
#include "mac_driver.h"
#include "mac_driver_mpan.h"
#include "mac_data_type.h"
#include "mac_stats.h"
#include "mac_802154_frame_parser.h"
#include "trace.h"
#include "rtl_efuse.h"

#define PHY_SYMBOL_TIME       16           ///< 1 symbol time in us.
#define SHR_DURATION_US       160          ///< Duration of SHR in us, 10 * PHY_SYMBOL_TIME.
#define PHR_DURATION_US       32           ///< Duration of PHR in us.
#define IMMACK_DURATION_US    352          ///< 5B SRHR + 1B PHR + 2B MHR + 1B SEQ + 2B FCS = 11 Bytes * 2 Symbol * 16 us = 352 us

mac_attribute_t mac_attr;
mac_driver_t drv;
pan_mac_comm_t pan_mac_comm;
pan_mac_t pan_mac[MAX_PAN_NUM];

typedef union _sleep_item_t
{
        volatile uint8_t sStayAwake;

        struct
        {
            volatile uint8_t waitAckTxDone           : 1;  // waiting until ACK transmission is done
            volatile uint8_t waitTxDone              : 1;  // waiting until transmission is done
            volatile uint8_t waitRxPendingData       : 1;  // waiting for pending data to be received
            volatile uint8_t waitScheduledRxWindow   : 1;  // waiting for a scheduled receive window
            volatile uint8_t rxOnWhenIdle            : 1;  // keeping the receiver on when the system is idle
            volatile uint8_t appForceStayAwake       : 1;  // App forces radio to stay awake and disables sleep
            volatile uint8_t reserved                : 2;  // reserved bit field for future use
        } sStayAwake_b;
} sleep_item_t;
static sleep_item_t sleep_item;
mac_timer_handle_t sWaitAckTxDoneTimer;
mac_timer_handle_t sCslTxDoneTimer;

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
} __attribute__((packed)) aux_sec_ctl_t;

#define FCS_LEN 2

#define SHORT_ADDRESS_SIZE 2
#define US_PER_MS          1000ULL

#define ACK_REQUEST_OFFSET      1
#define ACK_REQUEST_BIT         (1 << 5)
#define FRAME_PENDING_OFFSET    1
#define FRAME_PENDING_BIT       (1 << 4)
#define SECURITY_ENABLED_OFFSET 1
#ifdef SECURITY_ENABLED_BIT
#undef SECURITY_ENABLED_BIT
#define SECURITY_ENABLED_BIT    (1 << 3)
#endif

#define RSSI_SETTLE_TIME_US   40           ///< RSSI settle time in microseconds.
#define SAFE_DELTA            1000         ///< A safe value for the `dt` parameter of delayed operations.

#if defined(DLPS_EN) && (DLPS_EN == 1)
#define CSL_UNCERT            74           ///< The Uncertainty of the scheduling CSL of transmission by the parent, in ±10 us units.
#else
#define CSL_UNCERT            20           ///< The Uncertainty of the scheduling CSL of transmission by the parent, in ±10 us units.
#endif
#define PHY_SYMBOL_TIME       16           ///< 1 symbol time in us.
#define SHR_DURATION_US       160          ///< Duration of SHR in us, 10 * PHY_SYMBOL_TIME.
#define PHR_DURATION_US       32           ///< Duration of PHR in us.
#define IMMACK_DURATION_US    352          ///< 5B SRHR + 1B PHR + 2B MHR + 1B SEQ + 2B FCS = 11 Bytes * 2 Symbol * 16 us = 352 us
#define EXTRA_PROTECTION_EN   0
#define TRANSMIT_RETRIES_EN   1

#define IEEE802154_MAX_LENGTH (127)
#define MAC_FRAME_TX_HDR_LEN  (2)
#define MAC_FRAME_RX_HDR_LEN  (1)
#define MAC_FRAME_RX_TAIL_LEN (7)
#define MAC_FRAME_IMMACK_LEN  (3)

extern volatile uint64_t bt_clk_offset;
//static bool sWakeupFromDLPS;
static uint8_t curr_channel;
static uint16_t sPanid;

typedef struct {
	struct rtl87x2g_802154_rx_frame sReceivedFrames;
	uint8_t sReceivedPsdu[MAC_FRAME_RX_HDR_LEN + IEEE802154_MAX_LENGTH + MAC_FRAME_RX_TAIL_LEN];
} rx_item_t;

static uint16_t rx_tail;
static uint16_t rx_head;
static rx_item_t rx_queue[RTL87X2G_802154_RX_BUFFERS];

#define TX_NONE 0
#define TX_WAIT_ACK 1
#define TX_TERMED 2
#define TX_OK 3
#define TX_CCA_FAIL 4
#define TX_NO_ACK 5
#define TX_PTA_FAIL 6
#define TX_AT_FAIL 7
#define TX_BACKOFF_DONE 8

/*
#define MAX_TRANSMIT_RETRY 16
static uint32_t sTransmitRetry;
*/
#define MAX_TRANSMIT_CCAFAIL 16

/* rx ack frame */
static rx_item_t ack_item;
//static bool ack_receive_done;
//static bool ack_receive_pending;

/* tx enhack frame */
static uint8_t sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + IEEE802154_MAX_LENGTH];
static volatile uint8_t enhack_frm_len;
static volatile bool enhack_frm_sec_en;

typedef union {
	uint16_t val;
	struct {
		uint16_t len: 7;
		uint16_t id: 8;
		uint16_t type: 1;
	} b;
} header_ie_t;

typedef struct {
	header_ie_t header;
	uint16_t phase;
	uint16_t period;
} __attribute__((packed)) csl_ie_t;

typedef struct
{
    header_ie_t header;
    uint8_t oui[3];
    uint8_t subtype;
    uint8_t content[2];
} __attribute__((packed)) vendor_ie_t;

static uint32_t sCslPeriod;
static uint32_t sCslSampleTime;
static uint8_t sCslIeIndex;

static uint8_t enhAckProbingDataLen;
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
}

static const struct rtl87x2g_802154_config rtl87x2g_radio_cfg = {
	.irq_config_func = rtl87x2g_irq_config,
};

static void dataInit(void)
{
	for (uint8_t i = 0; i < RTL87X2G_802154_RX_BUFFERS; i++) {
		rx_queue[i].sReceivedFrames.psdu = rx_queue[i].sReceivedPsdu;
	}
	ack_item.sReceivedFrames.psdu = ack_item.sReceivedPsdu;
}

static void txAckProcessSecurity(uint8_t *raw)
{
	uint8_t sec_ctl_index = mac_802154_frame_parser_sec_ctrl_offset_get(raw);
	uint8_t key_id = raw[sec_ctl_index + 1 + 4];
	uint8_t *key = NULL;

	struct {
		uint8_t sec_level;
		uint32_t frame_counter;
		uint64_t src_ext_addr;
	} __attribute__((packed)) nonce;

	nonce.sec_level = SECURITY_LEVEL_ENC_MIC_32;
	memcpy(&nonce.frame_counter, &raw[sec_ctl_index + 1], 4);
	memcpy(&nonce.src_ext_addr, mac_GetLongAddress(), 8);

	/* set nonce */
	mac_LoadNonce((uint8_t *)&nonce);
	/* set key */
	if (key_id == sPrevKeyId)
	{
		//DBG_DIRECT("prev key");
		key = sPrevKey;
	}
	else if (key_id == sCurrKeyId)
	{
		//DBG_DIRECT("curr key");
		key = sCurrKey;
	}
	else if (key_id == sNextKeyId)
	{
		//DBG_DIRECT("next key");
		key = sNextKey;
	}
	else
	{
		DBG_DIRECT("unknown key");
	}
	mac_LoadTxEnhAckKey(key);
	/* set security level */
	mac_SetTxEnhAckCipher(nonce.sec_level);
}

static uint16_t getCslPhase(void)
{
	uint32_t msk;
	uint32_t now_btus;
	uint64_t now_us;
	uint32_t curTime;
	uint32_t cslPeriodInUs;
	uint32_t diff;

	msk = irq_lock();
	now_btus = mac_GetCurrentBTUS();
	now_us = bt_clk_offset + now_btus;
	curTime = (uint32_t)now_us;
	cslPeriodInUs = sCslPeriod * 160;
	diff = (cslPeriodInUs - (curTime % cslPeriodInUs) + (sCslSampleTime % cslPeriodInUs)) % cslPeriodInUs;
	irq_unlock(msk);
	return (uint16_t)(diff / 160 + 1);
}

void BEE_tx_ack_started(uint8_t *p_data, int8_t rssi, uint8_t lqi)
{
	csl_ie_t *p_csl_ie;
	vendor_ie_t *p_vendor_ie;
	uint8_t len;

	if (sCslPeriod > 0) {
		p_csl_ie = (csl_ie_t *)&sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + sCslIeIndex];
		p_csl_ie->phase = getCslPhase();
		p_csl_ie->period = sCslPeriod;
	}

	if (enhAckProbingDataLen > 0) {
		len = enhAckProbingDataLen;

		p_vendor_ie = (vendor_ie_t *)&sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + sVendorIeIndex];
		p_vendor_ie->oui[0] = 0x9b;
		p_vendor_ie->oui[1] = 0xb8;
		p_vendor_ie->oui[2] = 0xea;

		p_vendor_ie->subtype = 0x00;
		len--;

		p_vendor_ie->content[0] = lqi;
		len--;

		if (len > 0) {
			p_vendor_ie->content[1] = rssi;
		}
	}
}

extern uint8_t mpan_TrigTxEnhAck(bool_t early, bool_t SecReq, uint8_t pan_idx);
bool readFrame(rx_item_t *item, uint8_t pan_idx)
{
	struct rtl87x2g_802154_rx_frame *receivedFrame = &item->sReceivedFrames;
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
		if (mpan_TrigTxEnhAck(false, enhack_frm_sec_en, pan_idx) == MAC_STS_SUCCESS) {
			if (enhack_frm_sec_en) {
				sMacFrameCounter++;
			}
			sleep_item.sStayAwake_b.waitAckTxDone = 1;
		} else {
			LOG_WRN("enhack tx timeout");
		}
	}

	p_data = &receivedFrame->psdu[0];
	p_fc = (fc_t *)&receivedFrame->psdu[1];
	receivedFrame->rssi = rssi;
	receivedFrame->lqi = lqi;

	/* Inform if this frame was acknowledged with frame pending set. */
	if (p_data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT) {
		if (p_fc->ver == FRAME_VER_2015) {
			receivedFrame->ack_fpb = sEnhAckPsdu[2] & FRAME_PENDING_BIT;
		} else {
			if (mac_GetSrcMatchStatus()) {
				mac_ClrSrcMatchStatus();
				receivedFrame->ack_fpb = true;
			} else {
				receivedFrame->ack_fpb = false;
			}
		}
	} else {
		receivedFrame->ack_fpb = false;
	}

	if (p_data[SECURITY_ENABLED_OFFSET] & SECURITY_ENABLED_BIT) {
		if (p_fc->ver == FRAME_VER_2015) {
			receivedFrame->ack_seb = sEnhAckPsdu[2] & SECURITY_ENABLED_BIT;
		} else {
			receivedFrame->ack_seb = false;
		}
	} else {
		receivedFrame->ack_seb = false;
	}

	/* 0x7E header cmdid propid STREAM_RAW crc16 0x7E */
	/* Get the timestamp when the SFD was received */
	receivedFrame->time = bt_clk_offset +
						  mac_BTClkToUS(prx_fifo_tail->bt_time) -
						  (PHY_SYMBOL_TIME * 2);

	return true;
}

void readAck(rx_item_t *item, uint8_t pan_idx)
{
	struct rtl87x2g_802154_rx_frame *receivedFrame = &item->sReceivedFrames;
	pmac_rxfifo_t prx_fifo = (pmac_rxfifo_t)&receivedFrame->psdu[0];
	mac_rxfifo_tail_t *prx_fifo_tail;
	uint8_t *p_data;

	mac_Rx((uint8_t *)prx_fifo);

	p_data = &receivedFrame->psdu[0];
	prx_fifo_tail = (mac_rxfifo_tail_t *)&receivedFrame->psdu[1 + prx_fifo->frm_len];
	receivedFrame->lqi = prx_fifo_tail->lqi;
	receivedFrame->rssi = mac_GetRSSIFromRaw(prx_fifo_tail->rssi, mac_GetChannel());

	/* Get the timestamp when the SFD was received */
	receivedFrame->time = bt_clk_offset +
						  mac_BTClkToUS(prx_fifo_tail->bt_time) -
						  (PHY_SYMBOL_TIME * 2);
}

void BEE_tx_started(uint8_t *p_data)
{
}

void txProcessSecurity(uint8_t *raw)
{
	uint8_t sec_ctl_index = mac_802154_frame_parser_sec_ctrl_offset_get(raw);
	uint8_t key_id = raw[sec_ctl_index + 1 + 4];
	uint8_t *key = NULL;
	struct {
		uint8_t sec_level;
		uint32_t frame_counter;
		uint64_t src_ext_addr;
	} __attribute__((packed)) nonce;
	nonce.sec_level = SECURITY_LEVEL_ENC_MIC_32;
	memcpy(&nonce.frame_counter, &raw[sec_ctl_index + 1], 4);
	memcpy((uint8_t *)&nonce.src_ext_addr, mac_GetLongAddress(), 8);

	/* set nonce */
	mac_LoadNonce((uint8_t *)&nonce);
	/* set key */
	if (key_id == sPrevKeyId)
	{
		//DBG_DIRECT("prev key");
		key = sPrevKey;
	}
	else if (key_id == sCurrKeyId)
	{
		//DBG_DIRECT("curr key");
		key = sCurrKey;
	}
	else if (key_id == sNextKeyId)
	{
		//DBG_DIRECT("next key");
		key = sNextKey;
	}
	else
	{
		DBG_DIRECT("unknown key");
	}
	mac_LoadTxNKey(key);
	/* set security level */
	mac_SetTxNCipher(nonce.sec_level);
}

static uint8_t generate_ieee_enhack_frame(uint8_t *txbuf, uint8_t buf_len, fc_t *fc, uint8_t seq,
					  uint16_t dpid, uint8_t *dadr, uint8_t aux_len,
					  uint8_t *aux)
{
	uint8_t len = 0;
	csl_ie_t *p_csl_ie;
	vendor_ie_t *p_vendor_ie;

	/* frame control */
	memcpy((void *)(txbuf + len), (void *)fc, 2);
	len += 2;

	if (!fc->seq_num_suppress) {
		/* sequence number */
		txbuf[len] = seq;
		len += 1;
	}

	{
		if (!fc->panid_compress) {
			memcpy(&txbuf[len], &dpid, 2);
			len += 2;
		}
		if (fc->dst_addr_mode > 0) {
			if (fc->dst_addr_mode == ADDR_MODE_SHORT) {
				memcpy(&txbuf[len], dadr, 2);
				len += 2;
			} else {
				memcpy(&txbuf[len], dadr, 8);
				len += 8;
			}
		}
	}

	if (fc->sec_en) {
		memcpy((void *)(txbuf + len), (void *)aux, aux_len);
		len += aux_len;
	}

	if (fc->ie_present) {
		if (sCslPeriod > 0) {
			sCslIeIndex = len;
			p_csl_ie = (csl_ie_t *)&txbuf[sCslIeIndex];
			p_csl_ie->header.b.len = 4;
			p_csl_ie->header.b.id = 0x1a;
			p_csl_ie->header.b.type = 0;
			len += sizeof(csl_ie_t);
		} else {
			sCslIeIndex = 0;
		}

		if (enhAckProbingDataLen > 0) {
			sVendorIeIndex = len;
			p_vendor_ie = (vendor_ie_t *)&txbuf[sVendorIeIndex];
			// out + subtype + content
			p_vendor_ie->header.b.len = (IEEE802154_OPENTHREAD_VENDOR_OUI_LEN + 1 + enhAckProbingDataLen);
			p_vendor_ie->header.b.id = 0;
			p_vendor_ie->header.b.type = 0;
			len += (2 + IEEE802154_OPENTHREAD_VENDOR_OUI_LEN + 1 + enhAckProbingDataLen);
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

extern bool hw_sha256(uint8_t *input, uint32_t byte_len, uint32_t *result, int mode);
static uint32_t sha256_output[8];
static void rtl87x2g_get_eui64(uint8_t *mac)
{
	hw_sha256(get_ic_euid(), 14, sha256_output, 0);
	memcpy(mac, sha256_output, sizeof(uint64_t));
}

static void rtl87x2g_rx_thread(void *arg1, void *arg2, void *arg3)
{
	struct rtl87x2g_802154_data *rtl87x2g_radio = (struct rtl87x2g_802154_data *)arg1;
	struct net_pkt *pkt;
	struct rtl87x2g_802154_rx_frame *rx_frame;
	uint8_t pkt_len;
	uint8_t pkt_seq;
	//uint32_t msk;
	//uint32_t now_btus;
	//uint64_t now_us;
	//uint64_t micro_alarm_us;
	//uint64_t milli_alarm_us;
	//uint32_t diff;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		pkt = NULL;
		rx_frame = NULL;

		LOG_DBG("Waiting for frame");
		k_sem_take(&rtl87x2g_radio->rx_wait, K_FOREVER);
		rx_frame = &rx_queue[rx_head].sReceivedFrames;

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

		LOG_DBG("%s %d %d", __func__, pkt_seq, pkt_len);

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

#if defined(CONFIG_NET_L2_OPENTHREAD)
		net_pkt_set_ieee802154_ack_seb(pkt, rx_frame->ack_seb);
#endif

		if (net_recv_data(rtl87x2g_radio->iface, pkt) < 0) {
			DBG_DIRECT("Packet dropped by NET stack");
			goto drop;
		}

		rx_head = (rx_head + 1) % RTL87X2G_802154_RX_BUFFERS;

		if (LOG_LEVEL >= LOG_LEVEL_DBG) {
			log_stack_usage(&rtl87x2g_radio->rx_thread);
		}

		if (sleep_item.sStayAwake_b.waitRxPendingData)
		{
			sleep_item.sStayAwake_b.waitRxPendingData = 0;
		}
		continue;

drop:
		rx_head = (rx_head + 1) % RTL87X2G_802154_RX_BUFFERS;

		net_pkt_unref(pkt);
	}
}

static void rtl87x2g_get_capabilities_at_boot(void)
{
	rtl87x2g_data.capabilities =
		IEEE802154_HW_FCS |
		IEEE802154_HW_PROMISC |
		IEEE802154_HW_FILTER |
		IEEE802154_HW_TX_RX_ACK |
		IEEE802154_HW_RX_TX_ACK |
		IEEE802154_HW_ENERGY_SCAN |
		IEEE802154_HW_TXTIME |
		IEEE802154_HW_RXTIME |
		IEEE802154_HW_SLEEP_TO_TX |
		IEEE802154_RX_ON_WHEN_IDLE |
		IEEE802154_HW_TX_SEC;
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
	uint8_t pan_idx = 0;
	ARG_UNUSED(dev);

	if (channel < 11 || channel > 26) {
		return channel < 11 ? -ENOTSUP : -EINVAL;
	}

	mac_RadioOn();
	if (curr_channel != channel)
	{
		curr_channel = channel;
		mpan_SetChannel(channel, pan_idx);
	}
	return 0;
}

extern void mac_EDScan_begin(uint32_t scan_round);
extern int8_t mac_edscan_level2dbm(uint8_t level);
extern void mac_EDScan_end(uint8_t *peak_value);
static int rtl87x2g_energy_scan_start(const struct device *dev, uint16_t duration,
				      energy_scan_done_cb_t done_cb)
{
	int err = 0;
	uint8_t ed_value_raw;
    int8_t ed_value;

	ARG_UNUSED(dev);

	if (rtl87x2g_data.energy_scan_done == NULL) {
		rtl87x2g_data.energy_scan_done = done_cb;

		mac_EDScan_begin(duration * 1000 / 128);
		k_sem_take(&rtl87x2g_data.energy_scan_wait, K_FOREVER);
		mac_EDScan_end(&ed_value_raw);
		ed_value = mac_edscan_level2dbm(ed_value_raw);
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

	mac_SetLongAddress((uint8_t *)ieee_addr);

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

extern int8_t mac_SetTXPower_patch(int8_t tx_dbm);
static int rtl87x2g_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);

	LOG_DBG("%d", dbm);

	mac_SetTXPower_patch(dbm);

	return 0;
}

static int handle_ack(struct rtl87x2g_802154_data *rtl87x2g_radio)
{
	uint8_t ack_len;
	struct net_pkt *ack_pkt;
	int err = 0;

	ack_len = ack_item.sReceivedFrames.psdu[0];

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
	if (net_pkt_write(ack_pkt, &ack_item.sReceivedFrames.psdu[1], ack_len) < 0) {
		LOG_ERR("Failed to write to ack_pkt.");
		err = -ENOMEM;
		goto free_net_ack;
	}

	net_pkt_set_ieee802154_lqi(ack_pkt, ack_item.sReceivedFrames.lqi);
	net_pkt_set_ieee802154_rssi_dbm(ack_pkt, ack_item.sReceivedFrames.rssi);

#if defined(CONFIG_NET_PKT_TIMESTAMP)
	net_pkt_set_timestamp_ns(ack_pkt, ack_item.sReceivedFrames.time * NSEC_PER_USEC);
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

extern uint32_t mac_SetTxNCsmaDetail(bool enable, uint8_t be);
static int rtl87x2g_tx(const struct device *dev, enum ieee802154_tx_mode mode, struct net_pkt *pkt,
		       struct net_buf *frag)
{
	uint8_t pan_idx = 0;
	struct rtl87x2g_802154_data *rtl87x2g_radio = RTL87X2G_802154_DATA(dev);
	uint8_t payload_len = frag->len;
	uint8_t *payload = frag->data;
	fc_t *p_fc = (fc_t *)payload;
	uint8_t *raw = &payload[-1];
	uint8_t sec_ctl_index;
	aux_sec_ctl_t *p_sec_ctl;
	uint8_t *p_header;
	uint8_t header_len;
	header_ie_t *p_header_ie;
	//uint8_t *p_vendor_ie_content;
	csl_ie_t *p_csl_ie;
	uint8_t *txn_fifo = (uint8_t *)MAC_TXN_BASE_ADDR;
	uint64_t now;
	uint64_t target_us;
	uint32_t diff;
	int ret = 0;
    uint8_t cca_fail_cnt = 0;
    bool tx_complete = false;

	if (payload_len > IEEE802154_MTU) {
		LOG_ERR("Payload too large: %d", payload_len);
		return -EMSGSIZE;
	}

	if (mode != IEEE802154_TX_MODE_DIRECT && mode !=  IEEE802154_TX_MODE_TXTIME_CCA) {
		LOG_ERR("TX mode %d not supported", mode);
		return -ENOTSUP;
	}

    sleep_item.sStayAwake_b.waitTxDone = 1;
    if (pan_mac_comm.sleep_callback)
    {
        pan_mac_comm.sleep_callback(pan_idx);
    }
	do
	{
		if (p_fc->sec_en) {
			sec_ctl_index = mac_802154_frame_parser_sec_ctrl_offset_get(raw);
			p_sec_ctl = (aux_sec_ctl_t *)&raw[sec_ctl_index];

			if (p_sec_ctl->key_id_mode == 1) {
				if (!net_pkt_ieee802154_frame_secured(pkt)) {
					/* set frame_counter */
					memcpy(&raw[sec_ctl_index + 1], &sMacFrameCounter, 4);
					sMacFrameCounter++;
					net_pkt_set_ieee802154_frame_secured(pkt, true);
				}

				p_header = &raw[sec_ctl_index + 1 + 4 + 1];
				if (p_fc->ie_present) {
					while (1)
					{
						p_header_ie = (header_ie_t *)p_header;
						if (p_header_ie->b.id == 0x00)
						{
							// vendor ie content provide by openthread upper layer (?)
							p_header += (2 + p_header_ie->b.len);
						}
						else if (p_header_ie->b.id == 0x1a)
						{
							// csl ie
							p_csl_ie = (csl_ie_t *)p_header;
							p_csl_ie->phase = getCslPhase();
							p_csl_ie->period = sCslPeriod;
							p_header += (2 + p_header_ie->b.len);
						}
						else if (p_header_ie->b.id == 0x7f)
						{
							// ht2 ie
							p_header += (2 + p_header_ie->b.len);
							break;
						}
						else
						{
							// currently unsupport
							p_header += (2 + p_header_ie->b.len);
						}
					}
					header_len = (p_header - payload);
					mac_LoadTxNPayload(header_len, payload_len - 4, payload);
				}
				else {
					header_len = (p_header - payload);
					if (p_fc->type == FRAME_TYPE_COMMAND) {
						mac_LoadTxNPayload(header_len + 1, payload_len - 4, payload);
					} else {
						mac_LoadTxNPayload(header_len, payload_len - 4, payload);
					}
				}
				txProcessSecurity(raw);
				mac_TrigUpperEnc();

				memcpy(payload, &txn_fifo[MAC_FRAME_TX_HDR_LEN], payload_len);
			} else {
				mac_LoadTxNPayload(0, payload_len, payload);
			}
		} else {
			mac_LoadTxNPayload(0, payload_len, payload);
		}
		net_pkt_set_ieee802154_mac_hdr_rdy(pkt, true);

        rtl87x2g_tx_started(dev, pkt, frag);

        /* Reset semaphore in case ACK was received after timeout */
        //k_sem_reset(&rtl87x2g_radio->tx_wait);

		switch (mode) {
			case IEEE802154_TX_MODE_DIRECT:
                mpan_TrigTxN(p_fc->ack_req, false, pan_idx);
				break;

			case IEEE802154_TX_MODE_TXTIME_CCA:
				now = mac_GetCurrentMACTime();
				target_us = net_pkt_timestamp_ns(pkt) / NSEC_PER_USEC;
				if (target_us > now)
				{
					diff = target_us - now;
					mpan_TrigTxNAtUS(p_fc->ack_req, false, true, mac_GetCurrentBTUS() + diff, pan_idx);
				}
				else
				{
					mpan_TrigTxN(p_fc->ack_req, false, pan_idx);
				}
				break;

			default:
				break;
		}

		/* Wait for the callback from the radio driver. */
		k_sem_take(&rtl87x2g_radio->tx_wait, K_FOREVER);

		switch (rtl87x2g_radio->tx_result) {
		case TX_WAIT_ACK:
			fc_t *p_fc_ack = (fc_t *)&ack_item.sReceivedPsdu[1];
			handle_ack(rtl87x2g_radio);
			if (p_fc_ack->pending == 1)
			{
				sleep_item.sStayAwake_b.waitRxPendingData = 1;
			}
			break;

		case TX_OK:
			break;

		case TX_CCA_FAIL:
            mac_RstRF();
            ret = -EBUSY;
			break;

		case TX_TERMED:
            ret = -ENOMSG;
			break;

		case TX_NO_ACK:
            mac_RstRF();
			ret = -ENOMSG;
			break;

		case TX_AT_FAIL:
			ret = -ENOMSG;
            break;

		default:
			ret = -EIO;
			break;
		}
	}
    while (0);
    sleep_item.sStayAwake_b.waitTxDone = 0;
    if (pan_mac_comm.sleep_callback)
    {
        pan_mac_comm.sleep_callback(pan_idx);
    }
	return ret;
}

static net_time_t rtl87x2g_get_time(const struct device *dev)
{
	ARG_UNUSED(dev);

	uint32_t msk;
	uint32_t now_btus;
	uint64_t now_us;

	msk = irq_lock();
	now_btus = mac_GetCurrentBTUS();
	now_us = bt_clk_offset + now_btus;
	irq_unlock(msk);

	return (net_time_t)now_us * NSEC_PER_USEC;
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

bool mac_report_pm_check_result(uint8_t pan_idx)
{
    if (sleep_item.sStayAwake)
    {
        if (sleep_item.sStayAwake & 0xf)
        {
            DBG_DIRECT("sStayAwake 0x%x", sleep_item.sStayAwake);
        }
        return false;
    }
    else
    {
        return true;
    }
}

static int rtl87x2g_stop(const struct device *dev)
{
	uint8_t pan_idx = 0;
	if (pan_mac_comm.sleep_callback)
	{
		pan_mac_comm.sleep_callback(pan_idx);
	}
	return 0;
}

int32_t edscan_level2dbm(int32_t level)
{
    return (level * 2) - 90;
}

#ifdef CONFIG_SOC_SERIES_RTL87X2G
extern uint32_t (*lowerstack_SystemCall)(uint32_t opcode, uint32_t param, uint32_t param1,
                                         uint32_t param2);
extern void set_zigbee_priority(uint16_t priority, int16_t priority_min);
extern uint32_t get_zigbee_window_slot_imp(int16_t *prio, int16_t *prio_min);
extern void (*modem_set_zb_cca_combination)(uint8_t comb);
extern int8_t mac_GetTXPower_patch(void);
#endif
#ifdef CONFIG_SOC_SERIES_RTL87X3G
extern void modem_set_zb_cca_combination_rom(uint8_t comb);
#endif

#ifdef CONFIG_SOC_SERIES_RTL87X2G
extern void mac_Initialize_rtl87x2g(mac_driver_t *ptr_drv, mac_attribute_t *attribute);
#endif
#ifdef CONFIG_SOC_SERIES_RTL87X3G
extern void mac_InitPHYArbitration(mac_attribute_t *p_attr, mac_phy_arbitration_method method);
//extern void mac_Initialize(mac_driver_t *ptr_drv, mac_attribute_t *attribute);
extern void mac_Initialize_External(void);
#endif
void zb_mac_drv_init(void)
{
#ifdef CONFIG_SOC_SERIES_RTL87X2G
    mac_InitAttribute(&mac_attr);
    mac_attr.mac_cfg.rf_early_term = 0;
    //mac_attr.mac_cfg.frm06_rx_early = 0;
#if (BUILD_RCP == 1)
    mac_attr.phy_arbitration_en = 0;
#else
    mac_attr.mac_cfg.anch_jump_val = 3;
#endif

    //lowerstack_SystemCall(10, 1, 512, -1);

    mac_Enable();
    mac_Initialize_rtl87x2g(&drv, &mac_attr);
    mac_Initialize_Additional();
#endif
#ifdef CONFIG_SOC_SERIES_RTL87X3G
    mac_InitAttribute(&mac_attr);
    mac_attr.mac_cfg.rf_early_term = 0;
    //mac_attr.mac_cfg.frm06_rx_early = 0;

    mac_InitPHYArbitration(&mac_attr, MAC_PHY_ARBI_IMMEDIATE);
    mac_Enable();
    mac_Initialize(&drv, &mac_attr);
    mac_Initialize_External();
#endif
}

static volatile bool zb_mac_drv_enabled = false;
bool zb_mac_driver_is_initialized(void)
{
    return zb_mac_drv_enabled;
}

extern void mac_TrigMacTmr(uint32_t period_us);
void zb_mac_drv_enable(void)
{
	if (zb_mac_drv_enabled)
	{
		DBG_DIRECT("zb_mac_drv_enabled");
		return;
	}
	zb_mac_drv_enabled = true;

    int8_t tx_power_default;
    mpan_CommonInit(&pan_mac_comm);
    mpan_Init(&pan_mac[0], MPAN_PAN0);
    zb_mac_drv_init();
    mac_TrigMacTmr(500000);
#ifdef CONFIG_SOC_SERIES_RTL87X2G
    mac_RegisterCallback(NULL, edscan_level2dbm, (mac_gnt_entry_callback_t)set_zigbee_priority,
                         *modem_set_zb_cca_combination);
#endif
#ifdef CONFIG_SOC_SERIES_RTL87X3G
    mac_RegisterCallback(NULL, edscan_level2dbm, NULL,
                         modem_set_zb_cca_combination_rom);
#endif
    mac_SetCcaMode(MAC_CCA_ED);
    tx_power_default = mac_GetTXPower_patch();
    mac_SetTXPower_patch(tx_power_default);
}

struct rtl87x2g_802154_data *rtl87x2g_radio_init = NULL;
struct net_if *iface_init = NULL;

extern void mac_Initialize_Patch(void);
extern bool mac_GrantPHYStatus(void);
void txnterr_handler(uint8_t pan_idx, uint32_t arg);
void txn_handler(uint8_t pan_idx, uint32_t arg);
void rxely_handler(uint8_t pan_idx, uint32_t arg);
void rxack_handler(uint8_t pan_idx, uint32_t arg);
void rxdone_handler(uint8_t pan_idx, uint32_t arg);
void edscan_handler(uint8_t pan_idx, uint32_t arg);
int rtl87x2g_late_init(void)
{
    //DBG_DIRECT("%s %d", __func__, __LINE__);
	k_sem_init(&rtl87x2g_radio_init->rx_wait, 0, RTL87X2G_802154_RX_BUFFERS);
	k_sem_init(&rtl87x2g_radio_init->tx_wait, 0, 1);
	k_sem_init(&rtl87x2g_radio_init->energy_scan_wait, 0, 1);

	/* enable irq */
	sleep_item.sStayAwake_b.appForceStayAwake = true;
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), Zigbee_Handler_Patch, NULL, 0);
    irq_enable(DT_INST_IRQN(0));

	/* radio init */
	dataInit();
	mac_Initialize_Patch();
	zb_mac_drv_enable();
	mac_RadioOn();

    while (!mac_GrantPHYStatus())
    {
        __NOP();
    }

    mac_SetTxNCsma(false);
    mac_SetTxNMaxRetry(0);
    mpan_RegisterISR(0, txnterr_handler, txn_handler, rxely_handler, rxack_handler, rxdone_handler, edscan_handler);
    mpan_EnableCtl(0, 1);
	rtl87x2g_get_capabilities_at_boot();

	k_thread_create(&rtl87x2g_radio_init->rx_thread, rtl87x2g_radio_init->rx_stack,
			RTL87X2G_RX_STACK_SIZE, rtl87x2g_rx_thread, rtl87x2g_radio_init, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&rtl87x2g_radio_init->rx_thread, "rtl87x2g_rx");

    rtl87x2g_get_eui64(rtl87x2g_radio_init->mac);
	net_if_set_link_addr(iface_init, rtl87x2g_radio_init->mac, sizeof(rtl87x2g_radio_init->mac),
			     NET_LINK_IEEE802154);

	rtl87x2g_radio_init->iface = iface_init;
	ieee802154_init(iface_init);

    //DBG_DIRECT("%s %d", __func__, __LINE__);
    return 0;
}

static int rtl87x2g_init(const struct device *dev)
{
    //DBG_DIRECT("%s %d", __func__, __LINE__);
    rtl87x2g_radio_init = RTL87X2G_802154_DATA(dev);
    //DBG_DIRECT("%s %d", __func__, __LINE__);
    return 0;
}

static void rtl87x2g_iface_init(struct net_if *iface)
{
    //DBG_DIRECT("%s %d", __func__, __LINE__);
    iface_init = iface;
    //DBG_DIRECT("%s %d", __func__, __LINE__);
}

static void rtl87x2g_config_mac_keys(struct ieee802154_key *mac_keys)
{
	uint8_t i;
    uint32_t msk;
    //DBG_DIRECT("%s %d", __func__, __LINE__);

    msk = irq_lock();
	for (i = 0; mac_keys->key_value && i < 3; mac_keys++, i++) {
		switch (i) {
		case 0:
			sPrevKeyId = *(mac_keys->key_id);
			memcpy(sPrevKey, mac_keys->key_value, 16);
			break;

		case 1:
			sCurrKeyId = *(mac_keys->key_id);
			memcpy(sCurrKey, mac_keys->key_value, 16);
			break;

		case 2:
			sNextKeyId = *(mac_keys->key_id);
			memcpy(sNextKey, mac_keys->key_value, 16);
			break;

		default:
			break;
		}
	}
    irq_unlock(msk);

    //DBG_DIRECT("%s %d", __func__, __LINE__);
}

void mpan_RxAt_done_callback(uint8_t pan_idx)
{
    sleep_item.sStayAwake_b.waitScheduledRxWindow = 0;
}

extern void mac_RadioOnHighPriority(void);
static int rtl87x2g_configure(const struct device *dev, enum ieee802154_config_type type,
			      const struct ieee802154_config *config)
{
	uint8_t pan_idx = 0;
	ARG_UNUSED(dev);

	switch (type) {
	case IEEE802154_CONFIG_AUTO_ACK_FPB:
		if (config->auto_ack_fpb.enabled) {
			switch (config->auto_ack_fpb.mode) {
			case IEEE802154_FPB_ADDR_MATCH_THREAD:
				mac_SetAddrMatchMode(AUTO_ACK_PENDING_MODE_THREAD);
				break;

			case IEEE802154_FPB_ADDR_MATCH_ZIGBEE:
				mac_SetAddrMatchMode(AUTO_ACK_PENDING_MODE_ZIGBEE);
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

				memcpy(&saddr, config->ack_fpb.addr, sizeof(uint16_t));
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

					memcpy(&saddr, config->ack_fpb.addr, sizeof(uint16_t));
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
		uint8_t element_id;
		//struct ieee802154_header_ie_vendor_specific vendor_specific;

		if (config->ack_ie.purge_ie) {
			enhAckProbingDataLen = 0;
			break;
		}

		if (config->ack_ie.short_addr == IEEE802154_BROADCAST_ADDRESS ||
		    config->ack_ie.ext_addr == NULL) {
			return -ENOTSUP;
		}

		if (config->ack_ie.header_ie == NULL || config->ack_ie.header_ie->length == 0) {
			DBG_DIRECT("clear short_addr_le and ext_addr_le");
		} else {
			element_id = ieee802154_header_ie_get_element_id(config->ack_ie.header_ie);
			DBG_DIRECT("element_id %x", element_id);
			if (element_id == IEEE802154_HEADER_IE_ELEMENT_ID_CSL_IE)
			{
				// IEEE802154_CONFIG_CSL_PERIOD -> IEEE802154_HEADER_IE_ELEMENT_ID_CSL_IE
				// set sCslPeriod in IEEE802154_CONFIG_CSL_PERIOD
			}
			else if (element_id == IEEE802154_HEADER_IE_ELEMENT_ID_VENDOR_SPECIFIC_IE)
			{
				enhAckProbingDataLen = config->ack_ie.header_ie->length - 1 - IEEE802154_VENDOR_SPECIFIC_IE_OUI_LEN;
			}
			else
			{
				return -ENOTSUP;
			}
		}
		break;

	case IEEE802154_CONFIG_EXPECTED_RX_TIME:
		sCslSampleTime = config->expected_rx_time / NSEC_PER_USEC;
		DBG_DIRECT("sCslSampleTime %d", sCslSampleTime);
		break;

	case IEEE802154_CONFIG_RX_SLOT:
		mac_RadioOnHighPriority();
		if (curr_channel != config->rx_slot.channel)
		{
			curr_channel = config->rx_slot.channel;
			mpan_SetChannel(config->rx_slot.channel, pan_idx);
		}
		if (MAC_STS_SUCCESS == mpan_RxAt(config->rx_slot.channel,
										 config->rx_slot.start / NSEC_PER_USEC,
										 config->rx_slot.duration / NSEC_PER_USEC,
										 pan_idx))
		{
			sleep_item.sStayAwake_b.waitScheduledRxWindow = 1;
			if (pan_mac_comm.sleep_callback)
			{
				pan_mac_comm.sleep_callback(pan_idx);
			}
		}
		break;

	case IEEE802154_CONFIG_CSL_PERIOD:
		sCslPeriod = config->csl_period;
		DBG_DIRECT("sCslPeriod %d", sCslPeriod);
		break;

	case IEEE802154_CONFIG_RX_ON_WHEN_IDLE:
		sleep_item.sStayAwake_b.rxOnWhenIdle = config->rx_on_when_idle;
		DBG_DIRECT("rx_on_when_idle %d", config->rx_on_when_idle);
		if (!config->rx_on_when_idle && sleep_item.sStayAwake_b.appForceStayAwake)
		{
			DBG_DIRECT("change appForceStayAwake to 0");
			sleep_item.sStayAwake_b.appForceStayAwake = 0;
		}
		if (pan_mac_comm.sleep_callback)
		{
			pan_mac_comm.sleep_callback(pan_idx);
		}
		break;

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

#define TXNS_OFFSET         0
#define TXG1S_OFFSET        1
#define TXG2S_OFFSET        2
#define CCAFAIL_OFFSET      5
#define PTA_TX_FAIL_OFFSET  6

#define ACK_REQ_OFFSET  2
#define ACK_TYPE_OFFSET 3

void txnterr_handler(uint8_t pan_idx, uint32_t txat_status)
{
	rtl87x2g_data.tx_result = TX_AT_FAIL;
	k_sem_give(&rtl87x2g_data.tx_wait);
}

extern bool mac_GetTxNTermedStatus(void);
void txn_handler(uint8_t pan_idx, uint32_t txn_trig)
{
	uint8_t tx_status = mac_GetTxNStatus();

	if (tx_status & (1 << CCAFAIL_OFFSET))
    {
        rtl87x2g_data.tx_result = TX_CCA_FAIL;
        k_sem_give(&rtl87x2g_data.tx_wait);
    }
    else
    {
        if (tx_status & (1 << TXNS_OFFSET))
        {
            if (mac_GetTxNTermedStatus())
            {
                rtl87x2g_data.tx_result = TX_TERMED;
            }
            else
            {
                rtl87x2g_data.tx_result = TX_NO_ACK;
            }
            k_sem_give(&rtl87x2g_data.tx_wait);
        }
        else
        {
            if (txn_trig & (1 << ACK_REQ_OFFSET))
            {
            }
            else
            {
                rtl87x2g_data.tx_result = TX_OK;
                k_sem_give(&rtl87x2g_data.tx_wait);
            }
        }
    }
}

void waitAckTxDoneProtectionTimeout(void *arg)
{
    if (sleep_item.sStayAwake_b.waitAckTxDone)
    {
        sleep_item.sStayAwake_b.waitAckTxDone = 0;
    }
}

void startWaitAckTxDoneProtection(uint8_t pan_idx, uint64_t ackEndTime, uint32_t guardTime)
{
    uint64_t now = mac_GetCurrentMACTime();

    ackEndTime += guardTime;

    if (now >= ackEndTime) // already timeout
    {
        waitAckTxDoneProtectionTimeout((void *)(uint32_t)pan_idx);
        return;
    }

    uint32_t diff = ackEndTime - now;
    if (diff < 20)
    {
        diff = 20;
    }

    mac_SoftwareTimer_Start(&sWaitAckTxDoneTimer,
                            mac_GetCurrentBTUS() + diff,
                            waitAckTxDoneProtectionTimeout,
                            (void *)(uint32_t)pan_idx);
    //radio_inst[pan_idx].sWaitAckTxDoneTimestamp = ackEndTime;
}

void rxely_handler(uint8_t pan_idx, uint32_t arg)
{
	fc_t fc_ack;
	struct {
		aux_sec_ctl_t sec_ctl;
		uint32_t frame_counter;
		uint8_t key_id;
	} __attribute__((packed)) aux;
	uint64_t now = mac_GetCurrentMACTime();

		if (mac_GetRxFrmAckReq()) {
			if (mac_GetRxFrmVersion() == FRAME_VER_2015) {
				uint16_t panid;
				uint16_t saddr;
				uint64_t laddr;

				enhack_frm_sec_en = mac_GetRxFrmSecEn();
				fc_ack.type = FRAME_TYPE_ACK;
				fc_ack.sec_en = enhack_frm_sec_en;
				if (mac_GetSrcMatchStatus()) {
					mac_ClrSrcMatchStatus();
					fc_ack.pending = true;
				} else {
					fc_ack.pending = false;
				}
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
					mpan_TrigTxEnhAck(true, enhack_frm_sec_en, pan_idx);
					if (enhack_frm_sec_en) {
						sMacFrameCounter++;
					}
					sleep_item.sStayAwake_b.waitAckTxDone = 1;
				}
			} else {
				sleep_item.sStayAwake_b.waitAckTxDone = 1;
				startWaitAckTxDoneProtection(pan_idx,
											 (now + mac_RxFrameLength() * 2 * PHY_SYMBOL_TIME +
											 200 + IMMACK_DURATION_US),
											 0);
			}
		}
}

extern bool mac_GetTxEnhAckTermedStatus(void);
void mac_report_enhack_transmit_done(uint8_t pan_idx)
{
	uint8_t tx_status = mac_GetTxNStatus();
	enhack_frm_len = 0;
	if (enhack_frm_sec_en) {
		enhack_frm_sec_en = false;
	}

	if (tx_status & (1 << TXG1S_OFFSET))
    {
        if (mac_GetTxEnhAckTermedStatus())
        {
            LOG_WRN("txg1 termed");
        }
        else
        {
            LOG_WRN("txg1 fail %x", tx_status);
        }
    }

	if (sleep_item.sStayAwake_b.waitAckTxDone)
	{
		sleep_item.sStayAwake_b.waitAckTxDone = 0;
	}
}

void rxack_handler(uint8_t pan_idx, uint32_t arg)
{
	readAck(&ack_item, pan_idx);
	rtl87x2g_data.tx_result = TX_WAIT_ACK;
	k_sem_give(&rtl87x2g_data.tx_wait);
}

void rxdone_handler(uint8_t pan_idx, uint32_t arg)
{
	readFrame(&rx_queue[rx_tail], pan_idx);
	rx_tail = (rx_tail + 1) % RTL87X2G_802154_RX_BUFFERS;
	k_sem_give(&rtl87x2g_data.rx_wait);
}

void edscan_handler(uint8_t pan_idx, uint32_t arg)
{
	k_sem_give(&rtl87x2g_data.energy_scan_wait);
}

static struct ieee802154_radio_api rtl87x2g_radio_api = {
	.iface_api.init = rtl87x2g_iface_init,

	.get_capabilities = rtl87x2g_get_capabilities,
	.cca = rtl87x2g_cca,
	.set_channel = rtl87x2g_set_channel,
	.filter = rtl87x2g_filter,
	.set_txpower = rtl87x2g_set_txpower,
	.start = rtl87x2g_start,
	.stop = rtl87x2g_stop,

	.tx = rtl87x2g_tx,
	.ed_scan = rtl87x2g_energy_scan_start,
	.get_time = rtl87x2g_get_time,
	.get_sch_acc = rtl87x2g_get_acc,
	.configure = rtl87x2g_configure,
	.attr_get = rtl87x2g_attr_get
};

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
SYS_INIT(rtl87x2g_late_init, APPLICATION, 0);
