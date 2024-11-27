/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/logging/log.h>
#include "rtl_bt_hci.h"
#include "trace.h"

#define F_RTK_BT_HCI_H2C_POOL_SIZE              3*1024

LOG_MODULE_REGISTER(bt_driver, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);
#define DT_DRV_COMPAT realtek_bee_bt_hci

struct k_thread rx_thread_data;
static K_KERNEL_STACK_DEFINE(rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);

typedef struct
{
	intptr_t _unused;
	uint8_t *p_buf;
	uint32_t len;
} T_RTL_BT_RX_BUF;

static struct
{
	struct k_fifo   fifo;
} rx =
{
	.fifo = Z_FIFO_INITIALIZER(rx.fifo),
};

struct bt_rtl_data {
	bt_hci_recv_t recv;
};

static bool bt_rtl87x2x_check_hci_event_discardable(const uint8_t *event_data)
{
	uint8_t event_type = event_data[0];

	switch (event_type) {
#if defined(CONFIG_BT_CLASSIC)
	case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
	case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
		return true;
#endif
	case BT_HCI_EVT_LE_META_EVENT:
	{
		uint8_t sub_event_type = event_data[sizeof(struct bt_hci_evt_hdr)];

		switch (sub_event_type) {
		case BT_HCI_EVT_LE_ADVERTISING_REPORT:
			return true;
		default:
			return false;
		}
	}
	default:
		return false;
	}
}

static bool bt_rtl87x2x_recv_cb(T_RTL_BT_HCI_EVT evt, bool status, uint8_t *p_buf, uint32_t len)
{
	int ret = 0;

	LOG_DBG("%s: evt %u status %u, type %u, len %u", __func__, evt,
			status, p_buf[0], len);
	switch (evt) {
	case BT_HCI_EVT_OPENED:
	{
		LOG_DBG("BT_HCI_EVT_OPENED");
		if (status == false) {
			ret = -EXDEV;
		}
	}
	break;

	case BT_HCI_EVT_DATA_IND:
	{
		T_RTL_BT_RX_BUF *p_rx_buf;

		p_rx_buf = calloc(1, sizeof(T_RTL_BT_RX_BUF));
		if (p_rx_buf) {
			/* DBG_DIRECT("[BT] p_rx_buf %p, p_buf %p", p_rx_buf, p_buf); */
			p_rx_buf->p_buf = p_buf;
			p_rx_buf->len = len;
			k_fifo_put(&rx.fifo, p_rx_buf);
			break;
		}
		rtl_bt_hci_ack(p_buf);
	}
	break;

	default:
		ret = -EINVAL;
		break;
	}

	if (ret != 0) {
		DBG_DIRECT("[ERR] bt_rtl87x2g_recv_cb: error, evt %d status %d, type %d, len %d, ret %d",
				evt, status, p_buf[0], len, ret);
		LOG_ERR("bt_rtl87x2g_recv_cb: error, evt %u status %u, type %u, len %u, ret %d", evt,
				status, p_buf[0], len, ret);
		return false;
	}

	return true;
}

void bt_rtl87x2x_handle_rx_data(T_RTL_BT_RX_BUF *p_rx_buf)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct bt_rtl_data *hci = dev->data;
	struct net_buf *z_buf = NULL;
	size_t buf_tailroom = 0;
	/* First byte is packet type */
	switch (p_rx_buf->p_buf[0]) {
	case H4_EVT:
	{
		bool discardable = false;
		struct bt_hci_evt_hdr hdr;

		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

		discardable = bt_rtl87x2x_check_hci_event_discardable(&p_rx_buf->p_buf[1]);

		z_buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
		if (z_buf != NULL) {
			buf_tailroom = net_buf_tailroom(z_buf);

			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
				LOG_DBG("H4_EVT: event 0x%x", hdr.evt);
				hci->recv(dev, z_buf);
				break;
			}
			net_buf_unref(z_buf);
		}
		DBG_DIRECT("[ERR] H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
		LOG_ERR("H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
	}
	break;

	case H4_ACL:
	{
		struct bt_hci_acl_hdr hdr;

		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

		z_buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
		if (z_buf != NULL) {
			buf_tailroom = net_buf_tailroom(z_buf);
			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
				LOG_DBG("H4_ACL: handle 0x%x, Calling bt_recv(%p)", hdr.handle, z_buf);
				hci->recv(dev, z_buf);
				break;
			}
			net_buf_unref(z_buf);
		}
		DBG_DIRECT("[ERR] H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
		LOG_ERR("H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
	}
	break;

	case H4_ISO:
	{
		struct bt_hci_iso_hdr hdr;

		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

		z_buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
		if (z_buf != NULL) {
			buf_tailroom = net_buf_tailroom(z_buf);
			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
				LOG_DBG("H4_ISO: handle 0x%x, Calling bt_recv(%p)", hdr.handle, z_buf);
				hci->recv(dev, z_buf);
				break;
			}
			net_buf_unref(z_buf);
		}
		DBG_DIRECT("[ERR] H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
		LOG_ERR("H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
	}
	break;

	default:
		DBG_DIRECT("[ERR] rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
		LOG_ERR("rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
		break;
	}
	rtl_bt_hci_ack(p_rx_buf->p_buf);
	free(p_rx_buf);
}

static void rtl_rx_thread(void *p1, void *p2, void *p3)
{
	T_RTL_BT_RX_BUF *p_rx_buf;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		p_rx_buf = k_fifo_get(&rx.fifo, K_FOREVER);
		do {
			/* DBG_DIRECT("[BT] thread, p_rx_buf %p, p_buf %p", p_rx_buf, p_rx_buf->p_buf); */

			bt_rtl87x2x_handle_rx_data(p_rx_buf);

			/* Give other threads a chance to run if the ISR
			 * is receiving data so fast that rx.fifo never
			 * or very rarely goes empty.
			 */
			k_yield();

			p_rx_buf = k_fifo_get(&rx.fifo, K_NO_WAIT);
		} while (p_rx_buf);
	}
}

static int bt_rtl87x2x_send(const struct device *dev, struct net_buf *buf)
{

	int ret = 0;
	uint8_t h4_type = 0;
	T_RTL_BT_HCI_BUF hci_buf = {0};

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
	{
		h4_type = H4_ACL;
	}
	break;

	case BT_BUF_CMD:
	{
		h4_type = H4_CMD;
	}
	break;

	case BT_BUF_ISO_OUT:
	{
		h4_type = H4_ISO;
	}
	break;

	default:
		ret = -EINVAL;
		goto done;
	}

	if (rtl_bt_hci_h2c_buf_alloc(&hci_buf, h4_type, buf->len) == false) {
		ret = -EINVAL;
		goto done;
	}

	if (rtl_bt_hci_h2c_buf_add(&hci_buf, buf->data, buf->len) == false) {
		rtl_bt_hci_h2c_buf_rel(hci_buf);
		ret = -EINVAL;
		goto done;
	}

	if (rtl_bt_hci_send(hci_buf) == false) {
		rtl_bt_hci_h2c_buf_rel(hci_buf);
		ret = -EIO;
	}

done:
	net_buf_unref(buf);
	if (ret != 0) {
		DBG_DIRECT("[ERR] %s: error, h4_type %d, len %d, ret %d",
			__func__, h4_type, buf->len, ret);
		LOG_ERR("%s: error, h4_type %d, len %u, ret %d", __func__, h4_type, buf->len, ret);
	} else {
		LOG_DBG("%s: h4_type %d, len %u", __func__, h4_type, buf->len);
	}

	return ret;
}

static int bt_rtl87x2x_open(const struct device *dev, bt_hci_recv_t recv)
{
	k_tid_t tid;
	tid = k_thread_create(&rx_thread_data, rx_thread_stack,
						K_KERNEL_STACK_SIZEOF(rx_thread_stack),
						rtl_rx_thread, NULL, NULL, NULL,
						0, 0, K_NO_WAIT);
	k_thread_name_set(tid, "rtl_rx_thread");

	struct bt_rtl_data *hci = dev->data;

	if (rtl_bt_hci_h2c_pool_init(F_RTK_BT_HCI_H2C_POOL_SIZE)) {
		if (rtl_bt_hci_open(bt_rtl87x2x_recv_cb)) {
			hci->recv = recv;
			LOG_DBG("RTL BT started");
			return 0;
		}
	}
	DBG_DIRECT("[ERR] %s: failed", __func__);
	LOG_ERR("%s: failed", __func__);
	return -EINVAL;
}

static int bt_rtl87x2x_close(const struct device *dev)
{
	struct bt_rtl_data *hci = dev->data;

	hci->recv = NULL;

	LOG_DBG("RTL BT stopped");

	return 0;
}

static const struct bt_hci_driver_api drv = {
	.open           = bt_rtl87x2x_open,
	.send           = bt_rtl87x2x_send,
	.close          = bt_rtl87x2x_close,
};

#define BT_RTL_DEVICE_INIT(inst) \
	static struct bt_rtl_data bt_rtl_data_##inst = { \
	}; \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &bt_rtl_data_##inst, NULL, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported */
BT_RTL_DEVICE_INIT(0)
