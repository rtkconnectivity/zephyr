/* ieee802154_realtek_bee.h - bee 802.15.4 driver */

/*
 * Copyright (c) 2021-2026 Realtek Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_REALTEK_BEE_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_REALTEK_BEE_H_

#include <zephyr/net/ieee802154_radio.h>

#define RTL87X2G_PHY_LENGTH        (1)
#define RTL87X2G_RX_STACK_SIZE     2048
#define RTL87X2G_802154_RX_BUFFERS 16
#define RTL87X2G_INIT_PRIO         80

struct rtl87x2g_802154_rx_frame {
	void *fifo_reserved; /* 1st word reserved for use by fifo. */
	uint8_t *psdu; /* Pointer to a received frame. */
	uint64_t time; /* RX timestamp. */
	uint8_t lqi; /* Last received frame LQI value. */
	int8_t rssi; /* Last received frame RSSI value. */
	bool ack_fpb; /* FPB value in ACK sent for the received frame. */
	bool ack_seb; /* SEB value in ACK sent for the received frame. */
};

struct rtl87x2g_802154_data {
	/* Pointer to the network interface. */
	struct net_if *iface;

	/* 802.15.4 HW address. */
	uint8_t mac[8];

	/* RX thread stack. */
	K_KERNEL_STACK_MEMBER(rx_stack, RTL87X2G_RX_STACK_SIZE);

	/* RX thread control block. */
	struct k_thread rx_thread;

	/* RX fifo queue. */
	struct k_sem rx_wait;

	/* TX synchronization semaphore. Unlocked when frame has been
	 * sent or send procedure failed.
	 */
	struct k_sem tx_wait;

	/* TX result, updated in radio transmit callbacks. */
	uint8_t tx_result;

	/* energy scan synchronization semaphore. */
	struct k_sem energy_scan_wait;

	/* Callback handler of the currently ongoing energy scan.
	 * It shall be NULL if energy scan is not in progress.
	 */
	energy_scan_done_cb_t energy_scan_done;

	/* Callback handler to notify of any important radio events.
	 * Can be NULL if event notification is not needed.
	 */
	ieee802154_event_cb_t event_handler;

	/* Capabilities of the network interface. */
	enum ieee802154_hw_caps capabilities;

	/* The TX power in dBm. */
	int8_t txpwr;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_REALTEK_BEE_H_ */
