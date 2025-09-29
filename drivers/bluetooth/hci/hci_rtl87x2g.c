/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include "rtl_bt_hci.h"
#include "trace.h"

/* RTL_FIXME[🧩📡🔌] rtl hci driver */

#if 0//defined(CONFIG_BT_RECV_BLOCKING)

#define F_RTK_BT_HCI_H2C_POOL_SIZE 3 * 1024

LOG_MODULE_REGISTER(bt_driver, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);
#define DT_DRV_COMPAT realtek_bee_bt_hci

struct k_thread rx_thread_data;
static K_KERNEL_STACK_DEFINE(rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);

typedef struct {
	intptr_t _unused;
	uint8_t *p_buf;
	uint32_t len;
} T_RTL_BT_RX_BUF;

static struct {
	struct k_fifo fifo;
} rx = {
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
	case BT_HCI_EVT_LE_META_EVENT: {
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


#if defined(CONFIG_SOC_SERIES_RTL8752H)
#include "rtl876x_pinmux.h"
#elif  defined(CONFIG_SOC_SERIES_RTL87X2G)
#include "rtl_pinmux.h"
#endif


#define BT_HCI_EVT_FLAG_RECV_PRIO BIT(0)
#define BT_HCI_EVT_FLAG_RECV      BIT(1)

void hci_event_prio(struct net_buf *buf);

static inline uint8_t bt_hci_evt_get_flags(uint8_t evt)
{
	switch (evt) {
	case BT_HCI_EVT_DISCONN_COMPLETE:
		return BT_HCI_EVT_FLAG_RECV | BT_HCI_EVT_FLAG_RECV_PRIO;
		/* fallthrough */
#if defined(CONFIG_BT_CONN) || defined(CONFIG_BT_ISO)
	case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
#if defined(CONFIG_BT_CONN)
	case BT_HCI_EVT_DATA_BUF_OVERFLOW:
		__fallthrough;
#endif /* defined(CONFIG_BT_CONN) */
#endif /* CONFIG_BT_CONN ||  CONFIG_BT_ISO */
	case BT_HCI_EVT_CMD_COMPLETE:
	case BT_HCI_EVT_CMD_STATUS:
		return BT_HCI_EVT_FLAG_RECV_PRIO;
	default:
		return BT_HCI_EVT_FLAG_RECV;
	}
}


int bt_recv_prio(struct net_buf *buf)
{
	//bt_monitor_send(bt_monitor_opcode(buf), buf->data, buf->len);

	//BT_ASSERT(bt_buf_get_type(buf) == BT_BUF_EVT);

	hci_event_prio(buf);

	return 0;
}
static bool bt_rtl87x2x_recv_cb(T_RTL_BT_HCI_EVT evt, bool status, uint8_t *p_buf, uint32_t len)
{
	int ret = 0;
    //const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

	LOG_DBG("%s: evt %u status %u, type %u, len %u", __func__, evt, status, p_buf[0], len);
	switch (evt) {
	case BT_HCI_EVT_OPENED: {
		LOG_DBG("BT_HCI_EVT_OPENED");
		if (status == false) {
			ret = -EXDEV;
		}
	} break;

	case BT_HCI_EVT_DATA_IND: {
		T_RTL_BT_RX_BUF *p_rx_buf;
        if (p_buf[0] == H4_EVT) {
            uint8_t evt_flags;
            struct bt_hci_evt_hdr hdr;

            memcpy((void *)&hdr, &p_buf[1], sizeof(hdr));
            evt_flags = bt_hci_evt_get_flags(hdr.evt);
            //if (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO) {
             if (IS_ENABLED(CONFIG_BT_RECV_BLOCKING) && (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO)) {
                size_t buf_tailroom;
                struct net_buf *z_buf  = bt_buf_get_evt(hdr.evt, false, K_NO_WAIT);

                if (z_buf != NULL) {
                    buf_tailroom = net_buf_tailroom(z_buf);

                    if (buf_tailroom >= (hdr.len + 2)) {
                        net_buf_add_mem(z_buf, &p_buf[1], hdr.len + 2);
                        bt_buf_set_type(z_buf, BT_BUF_EVT);
                        bt_recv_prio(z_buf);
                        if ((evt_flags & BT_HCI_EVT_FLAG_RECV) == 0) {
                            rtl_bt_hci_ack(p_buf);
                            break;
                        } else {
                            net_buf_unref(z_buf);
                        }
                    } else {
                        DBG_DIRECT("[ERR] H4_EVT prio: event 0x%x, buf_tailroom %d, alloc failed",
                                hdr.evt, buf_tailroom);
                        LOG_ERR("H4_EVT prio: event 0x%x, buf_tailroom %d, alloc failed", hdr.evt, buf_tailroom);
                        net_buf_unref(z_buf);
                    }
                } else {
                    DBG_DIRECT("[ERR] H4_EVT prio: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
                    LOG_ERR("H4_EVT prio: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
                }
            }
        }
       // Pad_Config(P2_0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_HIGH);
		p_rx_buf = calloc(1, sizeof(T_RTL_BT_RX_BUF));
       // Pad_Config(P2_0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
		if (p_rx_buf) {
			/* DBG_DIRECT("[BT] p_rx_buf %p, p_buf %p", p_rx_buf, p_buf); */
			p_rx_buf->p_buf = p_buf;
			p_rx_buf->len = len;
			k_fifo_put(&rx.fifo, p_rx_buf);
			break;
		}
		rtl_bt_hci_ack(p_buf);
	} break;

	default:
		ret = -EINVAL;
		break;
	}

	if (ret != 0) {
		DBG_DIRECT("[ERR] bt_rtl87x2g_recv_cb: error, evt %d status %d, type %d, len %d, "
			   "ret %d",
			   evt, status, p_buf[0], len, ret);
		LOG_ERR("bt_rtl87x2g_recv_cb: error, evt %u status %u, type %u, len %u, ret %d",
			evt, status, p_buf[0], len, ret);
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
	case H4_EVT: {
		bool discardable = false;
		struct bt_hci_evt_hdr hdr;

		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

		discardable = bt_rtl87x2x_check_hci_event_discardable(&p_rx_buf->p_buf[1]);
        //Pad_Config(P2_1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_HIGH);
		z_buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
        //Pad_Config(P2_1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
		if (z_buf != NULL) {
			buf_tailroom = net_buf_tailroom(z_buf);

			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
				LOG_DBG("H4_EVT: event 0x%x", hdr.evt);
                //Pad_Config(P2_1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_HIGH);
				hci->recv(dev, z_buf);
                //Pad_Config(P2_1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
				break;
			}
			net_buf_unref(z_buf);
		}
		DBG_DIRECT("[ERR] H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
		LOG_ERR("H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
	} break;

	case H4_ACL: {
		struct bt_hci_acl_hdr hdr;

		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

		z_buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
		if (z_buf != NULL) {
			buf_tailroom = net_buf_tailroom(z_buf);
			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
				LOG_DBG("H4_ACL: handle 0x%x, Calling bt_recv(%p)", hdr.handle,
					z_buf);
				hci->recv(dev, z_buf);
				break;
			}
			net_buf_unref(z_buf);
		}
		DBG_DIRECT("[ERR] H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
		LOG_ERR("H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
	} break;

	case H4_ISO: {
		struct bt_hci_iso_hdr hdr;

		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

		z_buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
		if (z_buf != NULL) {
			buf_tailroom = net_buf_tailroom(z_buf);
			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
				LOG_DBG("H4_ISO: handle 0x%x, Calling bt_recv(%p)", hdr.handle,
					z_buf);
				hci->recv(dev, z_buf);
				break;
			}
			net_buf_unref(z_buf);
		}
		DBG_DIRECT("[ERR] H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
		LOG_ERR("H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
	} break;

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
			/* DBG_DIRECT("[BT] thread, p_rx_buf %p, p_buf %p", p_rx_buf,
			 * p_rx_buf->p_buf);
			 */
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

#else
// static bool bt_rtl87x2x_recv_cb(T_RTL_BT_HCI_EVT evt, bool status, uint8_t *p_buf, uint32_t len)
// {
// 	int ret = 0;

// 	LOG_DBG("%s: evt %u status %u, type %u, len %u", __func__, evt, status, p_buf[0], len);
// 	switch (evt) {
// 	case BT_HCI_EVT_OPENED: {
// 		LOG_DBG("BT_HCI_EVT_OPENED");
// 		if (status == false) {
// 			ret = -EXDEV;
// 		}
// 	} break;

// 	case BT_HCI_EVT_DATA_IND: {
// 		T_RTL_BT_RX_BUF *p_rx_buf;

// 		p_rx_buf = calloc(1, sizeof(T_RTL_BT_RX_BUF));
// 		if (p_rx_buf) {
// 			/* DBG_DIRECT("[BT] p_rx_buf %p, p_buf %p", p_rx_buf, p_buf); */
// 			p_rx_buf->p_buf = p_buf;
// 			p_rx_buf->len = len;
// 			k_fifo_put(&rx.fifo, p_rx_buf);
// 			break;
// 		}
// 		rtl_bt_hci_ack(p_buf);
// 	} break;

// 	default:
// 		ret = -EINVAL;
// 		break;
// 	}

// 	if (ret != 0) {
// 		DBG_DIRECT("[ERR] bt_rtl87x2g_recv_cb: error, evt %d status %d, type %d, len %d, "
// 			   "ret %d",
// 			   evt, status, p_buf[0], len, ret);
// 		LOG_ERR("bt_rtl87x2g_recv_cb: error, evt %u status %u, type %u, len %u, ret %d",
// 			evt, status, p_buf[0], len, ret);
// 		return false;
// 	}

// 	return true;
// }

// void bt_rtl87x2x_handle_rx_data(T_RTL_BT_RX_BUF *p_rx_buf)
// {
// 	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
// 	struct bt_rtl_data *hci = dev->data;
// 	struct net_buf *z_buf = NULL;
// 	size_t buf_tailroom = 0;
// 	/* First byte is packet type */
// 	switch (p_rx_buf->p_buf[0]) {
// 	case H4_EVT: {
// 		bool discardable = false;
// 		struct bt_hci_evt_hdr hdr;

// 		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

// 		discardable = bt_rtl87x2x_check_hci_event_discardable(&p_rx_buf->p_buf[1]);

// 		z_buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
// 		if (z_buf != NULL) {
// 			buf_tailroom = net_buf_tailroom(z_buf);

// 			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
// 				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
// 				LOG_DBG("H4_EVT: event 0x%x", hdr.evt);
// 				hci->recv(dev, z_buf);
// 				break;
// 			}
// 			net_buf_unref(z_buf);
// 		}
// 		DBG_DIRECT("[ERR] H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
// 		LOG_ERR("H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
// 	} break;

// 	case H4_ACL: {
// 		struct bt_hci_acl_hdr hdr;

// 		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

// 		z_buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
// 		if (z_buf != NULL) {
// 			buf_tailroom = net_buf_tailroom(z_buf);
// 			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
// 				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
// 				LOG_DBG("H4_ACL: handle 0x%x, Calling bt_recv(%p)", hdr.handle,
// 					z_buf);
// 				hci->recv(dev, z_buf);
// 				break;
// 			}
// 			net_buf_unref(z_buf);
// 		}
// 		DBG_DIRECT("[ERR] H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 		LOG_ERR("H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 	} break;

// 	case H4_ISO: {
// 		struct bt_hci_iso_hdr hdr;

// 		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

// 		z_buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
// 		if (z_buf != NULL) {
// 			buf_tailroom = net_buf_tailroom(z_buf);
// 			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
// 				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
// 				LOG_DBG("H4_ISO: handle 0x%x, Calling bt_recv(%p)", hdr.handle,
// 					z_buf);
// 				hci->recv(dev, z_buf);
// 				break;
// 			}
// 			net_buf_unref(z_buf);
// 		}
// 		DBG_DIRECT("[ERR] H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 		LOG_ERR("H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 	} break;

// 	default:
// 		DBG_DIRECT("[ERR] rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
// 		LOG_ERR("rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
// 		break;
// 	}
// 	rtl_bt_hci_ack(p_rx_buf->p_buf);
// 	free(p_rx_buf);
// }

// static void rtl_rx_thread(void *p1, void *p2, void *p3)
// {
// 	T_RTL_BT_RX_BUF *p_rx_buf;

// 	ARG_UNUSED(p1);
// 	ARG_UNUSED(p2);
// 	ARG_UNUSED(p3);

// 	while (1) {
// 		p_rx_buf = k_fifo_get(&rx.fifo, K_FOREVER);
// 		do {
// 			/* DBG_DIRECT("[BT] thread, p_rx_buf %p, p_buf %p", p_rx_buf,
// 			 * p_rx_buf->p_buf);
// 			 */
// 			bt_rtl87x2x_handle_rx_data(p_rx_buf);

// 			/* Give other threads a chance to run if the ISR
// 			 * is receiving data so fast that rx.fifo never
// 			 * or very rarely goes empty.
// 			 */
// 			k_yield();

// 			p_rx_buf = k_fifo_get(&rx.fifo, K_NO_WAIT);
// 		} while (p_rx_buf);
// 	}
// }


/*zhiyuan-fix-2025-07-30*/
// #include "rtl_bt_hci.h"
// #include "trace.h"

// #define BT_HCI_EVT_FLAG_RECV_PRIO BIT(0)
// #define BT_HCI_EVT_FLAG_RECV      BIT(1)

// #define F_RTK_BT_HCI_H2C_POOL_SIZE              3*1024

// LOG_MODULE_REGISTER(bt_driver, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);
// #define DT_DRV_COMPAT realtek_bee_bt_hci

// struct k_thread rx_thread_data;
// static K_KERNEL_STACK_DEFINE(rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);

// typedef struct
// {
// 	intptr_t _unused;
// 	uint8_t *p_buf;
// 	uint32_t len;
// } T_RTL_BT_RX_BUF;

// static struct
// {
// 	struct k_fifo   fifo;
// } rx =
// {
// 	.fifo = Z_FIFO_INITIALIZER(rx.fifo),
// };

// struct bt_rtl_data {
// 	bt_hci_recv_t recv;
// };

// static inline uint8_t bt_hci_evt_get_flags(uint8_t evt)
// {
// 	switch (evt) {
// 	case BT_HCI_EVT_DISCONN_COMPLETE:
// 		return BT_HCI_EVT_FLAG_RECV | BT_HCI_EVT_FLAG_RECV_PRIO;
// 		/* fallthrough */
// #if defined(CONFIG_BT_CONN) || defined(CONFIG_BT_ISO)
// 	case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
// #if defined(CONFIG_BT_CONN)
// 	case BT_HCI_EVT_DATA_BUF_OVERFLOW:
// 		__fallthrough;
// #endif /* defined(CONFIG_BT_CONN) */
// #endif /* CONFIG_BT_CONN ||  CONFIG_BT_ISO */
// 	case BT_HCI_EVT_CMD_COMPLETE:
// 	case BT_HCI_EVT_CMD_STATUS:
// 		return BT_HCI_EVT_FLAG_RECV_PRIO;
// 	default:
// 		return BT_HCI_EVT_FLAG_RECV;
// 	}
// }

// extern void hci_event_prio(struct net_buf *buf);
// static int bt_recv_prio(struct net_buf *buf)
// {
// 	//bt_monitor_send(bt_monitor_opcode(buf), buf->data, buf->len);

// 	//BT_ASSERT(bt_buf_get_type(buf) == BT_BUF_EVT);

// 	hci_event_prio(buf);

// 	return 0;
// }

// static bool bt_rtl87x2x_check_hci_event_discardable(const uint8_t *event_data)
// {
// 	uint8_t event_type = event_data[0];

// 	switch (event_type) {
// #if defined(CONFIG_BT_CLASSIC)
// 	case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
// 	case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
// 		return true;
// #endif
// 	case BT_HCI_EVT_LE_META_EVENT:
// 	{
// 		uint8_t sub_event_type = event_data[sizeof(struct bt_hci_evt_hdr)];

// 		switch (sub_event_type) {
// 		case BT_HCI_EVT_LE_ADVERTISING_REPORT:
// 			return true;
// 		default:
// 			return false;
// 		}
// 	}
// 	default:
// 		return false;
// 	}
// }



// static bool bt_rtl87x2x_recv_cb(T_RTL_BT_HCI_EVT evt, bool status, uint8_t *p_buf, uint32_t len)
// {
// 	int ret = 0;
// 	// const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

// 	LOG_DBG("%s: evt %u status %u, type %u, len %u", __func__, evt,
// 			status, p_buf[0], len);
// 	switch (evt) {
// 		case BT_HCI_EVT_OPENED:
// 		{
// 			LOG_DBG("BT_HCI_EVT_OPENED");
// 			if (status == false) {
// 				ret = -EXDEV;
// 			}
// 		}
// 		break;

// 		case BT_HCI_EVT_DATA_IND:
// 		{
// 			T_RTL_BT_RX_BUF *p_rx_buf;

// 			if (p_buf[0] == H4_EVT) {
// 				uint8_t evt_flags;
// 				struct bt_hci_evt_hdr hdr;

// 				memcpy((void *)&hdr, &p_buf[1], sizeof(hdr));
// 				evt_flags = bt_hci_evt_get_flags(hdr.evt);
// 				if (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO) {
// 					size_t buf_tailroom;
// 					struct net_buf *z_buf  = bt_buf_get_evt(hdr.evt, false, K_NO_WAIT);

// 					if (z_buf != NULL) {
// 						buf_tailroom = net_buf_tailroom(z_buf);

// 						if (buf_tailroom >= (hdr.len + 2)) {
// 							net_buf_add_mem(z_buf, &p_buf[1], hdr.len + 2);
// 							bt_buf_set_type(z_buf, BT_BUF_EVT);
// 							bt_recv_prio(z_buf);
// 							if ((evt_flags & BT_HCI_EVT_FLAG_RECV) == 0) {
// 								rtl_bt_hci_ack(p_buf);
// 								break;
// 							} else {
// 								net_buf_unref(z_buf);
// 							}
// 						} else {
// 							DBG_DIRECT("[ERR] H4_EVT prio: event 0x%x, buf_tailroom %d, alloc failed",
// 									hdr.evt, buf_tailroom);
// 							LOG_ERR("H4_EVT prio: event 0x%x, buf_tailroom %d, alloc failed", hdr.evt, buf_tailroom);
// 							net_buf_unref(z_buf);
// 						}
// 					} else {
// 						DBG_DIRECT("[ERR] H4_EVT prio: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
// 						LOG_ERR("H4_EVT prio: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
// 					}
// 				}
// 			}

// 			p_rx_buf = calloc(1, sizeof(T_RTL_BT_RX_BUF));
// 			if (p_rx_buf) {
// 				/* DBG_DIRECT("[BT] p_rx_buf %p, p_buf %p", p_rx_buf, p_buf); */
// 				p_rx_buf->p_buf = p_buf;
// 				p_rx_buf->len = len;
// 				k_fifo_put(&rx.fifo, p_rx_buf);
// 				break;
// 			}
// 			rtl_bt_hci_ack(p_buf);
// 		}
// 	    break;
// 	    default:
// 		ret = -EINVAL;
// 		break;
//     }
//     if (ret != 0) {
// 		DBG_DIRECT("[ERR] bt_rtl87x2g_recv_cb: error, evt %d status %d, type %d, len %d, ret %d",
// 				evt, status, p_buf[0], len, ret);
// 		LOG_ERR("bt_rtl87x2g_recv_cb: error, evt %u status %u, type %u, len %u, ret %d", evt,
// 				status, p_buf[0], len, ret);
// 		return false;
// 	}
// 	return true;
// }

// void bt_rtl87x2x_handle_rx_data(T_RTL_BT_RX_BUF *p_rx_buf)
// {
// 	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
// 	struct bt_rtl_data *hci = dev->data;
// 	struct net_buf *z_buf = NULL;
// 	size_t buf_tailroom = 0;
// 	/* First byte is packet type */
// 	switch (p_rx_buf->p_buf[0]) {
// 	case H4_EVT:
// 	{
// 		bool discardable = false;
// 		struct bt_hci_evt_hdr hdr;
//         memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));
// 		discardable = bt_rtl87x2x_check_hci_event_discardable(&p_rx_buf->p_buf[1]);

// 		z_buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
// 		if (z_buf != NULL) {
// 			buf_tailroom = net_buf_tailroom(z_buf);

// 			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
// 				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
// 				LOG_DBG("H4_EVT: event 0x%x", hdr.evt);
// 				hci->recv(dev, z_buf);
// 				break;
// 			}
// 			net_buf_unref(z_buf);
// 		}
// 		DBG_DIRECT("[ERR] H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
// 		LOG_ERR("H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
// 	}
// 	break;
//     case H4_ACL:
// 	{
// 		struct bt_hci_acl_hdr hdr;

// 		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

// 		z_buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
// 		if (z_buf != NULL) {
// 			buf_tailroom = net_buf_tailroom(z_buf);
// 			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
// 				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
// 				LOG_DBG("H4_ACL: handle 0x%x, Calling bt_recv(%p)", hdr.handle, z_buf);
// 				hci->recv(dev, z_buf);
// 				break;
// 			}
// 			net_buf_unref(z_buf);
// 		}
// 		DBG_DIRECT("[ERR] H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 		LOG_ERR("H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 	}
// 	break;

// 	case H4_ISO:
// 	{
// 		struct bt_hci_iso_hdr hdr;

// 		memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));

// 		z_buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
// 		if (z_buf != NULL) {
// 			buf_tailroom = net_buf_tailroom(z_buf);
// 			if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
// 				net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
// 				LOG_DBG("H4_ISO: handle 0x%x, Calling bt_recv(%p)", hdr.handle, z_buf);
// 				hci->recv(dev, z_buf);
// 				break;
// 			}
// 			net_buf_unref(z_buf);
// 		}
// 		DBG_DIRECT("[ERR] H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 		LOG_ERR("H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
// 	}
// 	break;

// 	default:
// 	    DBG_DIRECT("[ERR] rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
// 		LOG_ERR("rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
// 		break;
// 	}
// 	rtl_bt_hci_ack(p_rx_buf->p_buf);
// 	free(p_rx_buf);
// }

// static void rtl_rx_thread(void *p1, void *p2, void *p3)
// {
// 	T_RTL_BT_RX_BUF *p_rx_buf;



// 	ARG_UNUSED(p1);
// 	ARG_UNUSED(p2);
// 	ARG_UNUSED(p3);

// 	while (1) {
// 		p_rx_buf = k_fifo_get(&rx.fifo, K_FOREVER);
// 		do {
// 			/* DBG_DIRECT("[BT] thread, p_rx_buf %p, p_buf %p", p_rx_buf, p_rx_buf->p_buf); */

// 			bt_rtl87x2x_handle_rx_data(p_rx_buf);

// 			/* Give other threads a chance to run if the ISR
// 			 * is receiving data so fast that rx.fifo never
// 			 * or very rarely goes empty.
// 			 */
// 			k_yield();

// 			p_rx_buf = k_fifo_get(&rx.fifo, K_NO_WAIT);
// 		} while (p_rx_buf);
// 	}
// }

/*
 grok3 fix 2025-08-01
 */
#include <zephyr/kernel.h>
/* RTL_FIXME[🧩📡🔌] rtl hci driver */
#define F_RTK_BT_HCI_H2C_POOL_SIZE 3 * 1024

LOG_MODULE_REGISTER(bt_driver, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT realtek_bee_bt_hci

typedef struct {
    intptr_t _unused;
    uint8_t *p_buf;
    uint32_t len;
} T_RTL_BT_RX_BUF;

// ---- FIFO和SLAB初始化 ----

// 定义FIFO: 放入结构体（Zephyr官方写法）
static struct {
    struct k_fifo fifo;
} rx = {
    .fifo = Z_FIFO_INITIALIZER(rx.fifo),
};

// SLAB 不要放进结构体，单独全局静态定义
K_MEM_SLAB_DEFINE(rx_slab, sizeof(T_RTL_BT_RX_BUF), 16, 4);

struct bt_rtl_data {
    bt_hci_recv_t recv;
};

struct k_thread rx_thread_data;
static K_KERNEL_STACK_DEFINE(rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);

static bool bt_rtl87x2x_check_hci_event_discardable(const uint8_t *event_data)
{
    uint8_t event_type = event_data[0];
    switch (event_type) {
#if defined(CONFIG_BT_CLASSIC)
    case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
    case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
        return true;
#endif
    case BT_HCI_EVT_LE_META_EVENT: {
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

#if defined(CONFIG_SOC_SERIES_RTL8752H)
#include "rtl876x_pinmux.h"
#elif defined(CONFIG_SOC_SERIES_RTL87X2G)
#include "rtl_pinmux.h"
#endif

#define BT_HCI_EVT_FLAG_RECV_PRIO BIT(0)
#define BT_HCI_EVT_FLAG_RECV BIT(1)

void hci_event_prio(struct net_buf *buf);

static inline uint8_t bt_hci_evt_get_flags(uint8_t evt)
{
    switch (evt) {
    case BT_HCI_EVT_DISCONN_COMPLETE:
        return BT_HCI_EVT_FLAG_RECV | BT_HCI_EVT_FLAG_RECV_PRIO;
#if defined(CONFIG_BT_CONN) || defined(CONFIG_BT_ISO)
    case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
#if defined(CONFIG_BT_CONN)
    case BT_HCI_EVT_DATA_BUF_OVERFLOW:
        __fallthrough;
#endif /* defined(CONFIG_BT_CONN) */
#endif /* CONFIG_BT_CONN || CONFIG_BT_ISO */
    case BT_HCI_EVT_CMD_COMPLETE:
    case BT_HCI_EVT_CMD_STATUS:
        return BT_HCI_EVT_FLAG_RECV_PRIO;
    default:
        return BT_HCI_EVT_FLAG_RECV;
    }
}

int bt_recv_prio(struct net_buf *buf)
{
    hci_event_prio(buf);
    return 0;
}

T_RTL_BT_RX_BUF *rx_p_buf = NULL;
int slab_ret;
static bool bt_rtl87x2x_recv_cb(T_RTL_BT_HCI_EVT evt, bool status, uint8_t *p_buf, uint32_t len)
{
    int ret = 0;

    LOG_DBG("%s: evt %u status %u, type %u, len %u", __func__, evt, status, p_buf[0], len);

    switch (evt) {
    case BT_HCI_EVT_OPENED: {
        LOG_DBG("BT_HCI_EVT_OPENED");
        if (status == false) {
            ret = -EXDEV;
        }
    } break;

    case BT_HCI_EVT_DATA_IND: {
 			if (p_buf[0] == H4_EVT) {
				uint8_t evt_flags;
				struct bt_hci_evt_hdr hdr;

 				memcpy((void *)&hdr, &p_buf[1], sizeof(hdr));
 				evt_flags = bt_hci_evt_get_flags(hdr.evt);
				if (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO) {
 					size_t buf_tailroom;
					struct net_buf *z_buf  = bt_buf_get_evt(hdr.evt, false, K_NO_WAIT);

 					if (z_buf != NULL) {
 						buf_tailroom = net_buf_tailroom(z_buf);

 						if (buf_tailroom >= (hdr.len + 2)) {
							net_buf_add_mem(z_buf, &p_buf[1], hdr.len + 2);
							bt_buf_set_type(z_buf, BT_BUF_EVT);
 							bt_recv_prio(z_buf);
 							if ((evt_flags & BT_HCI_EVT_FLAG_RECV) == 0) {
 								rtl_bt_hci_ack(p_buf);
 								break;
 							} else {
 								net_buf_unref(z_buf);
 							}
 						} else {
 							DBG_DIRECT("[ERR] H4_EVT prio: event 0x%x, buf_tailroom %d, alloc failed",
 									hdr.evt, buf_tailroom);
 							LOG_ERR("H4_EVT prio: event 0x%x, buf_tailroom %d, alloc failed", hdr.evt, buf_tailroom);
 							net_buf_unref(z_buf);
 						}
 					} else {
 						DBG_DIRECT("[ERR] H4_EVT prio: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
 						LOG_ERR("H4_EVT prio: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
 					}
 				}
			}	
        //Pad_Config(P2_0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_HIGH);
        slab_ret = k_mem_slab_alloc(&rx_slab, (void **)&rx_p_buf, K_NO_WAIT);
		//Pad_Config(P2_0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
        if (slab_ret == 0) {
            rx_p_buf->p_buf = p_buf;
            rx_p_buf->len = len;
            k_fifo_put(&rx.fifo, rx_p_buf);
        } else {
            // DBG_DIRECT("[ERR] H4_%s: alloc failed", p_buf[0] == H4_EVT ? "EVT" :
            //            p_buf[0] == H4_ACL ? "ACL" : "ISO");
            // LOG_ERR("H4_%s: alloc failed", p_buf[0] == H4_EVT ? "EVT" :
            //         p_buf[0] == H4_ACL ? "ACL" : "ISO");
            rtl_bt_hci_ack(p_buf);
			LOG_ERR("H4_%s: alloc failed", p_buf[0] == H4_EVT ? "EVT" :"ACL");
                    //p_buf[0] == H4_ACL ? "ACL" : "ISO");
            ret = -ENOMEM;
        }
    } break;

    default:
        ret = -EINVAL;
        break;
    }

    if (ret != 0) {
        DBG_DIRECT("[ERR] bt_rtl87x2x_recv_cb: evt %d status %d, type %d, len %d, ret %d",
                   evt, status, p_buf[0], len, ret);
        LOG_ERR("bt_rtl87x2x_recv_cb: evt %u status %u, type %u, len %u, ret %d",
                evt, status, p_buf[0], len, ret);
        return false;
    }

    return true;
}

// 其余接口不变，唯一变化点是 slab 用 rx_slab，全局独立访问
void bt_rtl87x2x_handle_rx_data(T_RTL_BT_RX_BUF *p_rx_buf)
{
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
    struct bt_rtl_data *hci = dev->data;
    struct net_buf *z_buf = NULL;
    size_t buf_tailroom = 0;

    switch (p_rx_buf->p_buf[0]) {
    case H4_EVT: {
       
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
		
        } else {
            DBG_DIRECT("[ERR] H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
            LOG_ERR("H4_EVT: event 0x%x, len %d, alloc failed", hdr.evt, hdr.len);
        }
    } break;

    case H4_ACL: {
        struct bt_hci_acl_hdr hdr;
        memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));
        z_buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
        if (z_buf != NULL) {
            buf_tailroom = net_buf_tailroom(z_buf);
            if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
                net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
                LOG_DBG("H4_ACL: handle 0x%x, Calling bt_recv(%p)", hdr.handle, z_buf);
                hci->recv(dev, z_buf);
            } else {
                DBG_DIRECT("[ERR] H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
                LOG_ERR("H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
                net_buf_unref(z_buf);
            }
        } else {
            DBG_DIRECT("[ERR] H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
            LOG_ERR("H4_ACL: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
        }
    } break;

    case H4_ISO: {
        struct bt_hci_iso_hdr hdr;
        memcpy((void *)&hdr, &p_rx_buf->p_buf[1], sizeof(hdr));
        z_buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_NO_WAIT);
        if (z_buf != NULL) {
            buf_tailroom = net_buf_tailroom(z_buf);
            if (buf_tailroom >= (hdr.len + sizeof(hdr))) {
                net_buf_add_mem(z_buf, &p_rx_buf->p_buf[1], hdr.len + sizeof(hdr));
                LOG_DBG("H4_ISO: handle 0x%x, Calling bt_recv(%p)", hdr.handle, z_buf);
                hci->recv(dev, z_buf);
            } else {
                DBG_DIRECT("[ERR] H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
                LOG_ERR("H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
                net_buf_unref(z_buf);
            }
        } else {
            DBG_DIRECT("[ERR] H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
            LOG_ERR("H4_ISO: handle 0x%x, len %d, alloc failed", hdr.handle, hdr.len);
        }
    } break;

    default:
        DBG_DIRECT("[ERR] rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
        LOG_ERR("rtl_rx_thread: invalid type %d", p_rx_buf->p_buf[0]);
        break;
    }

    rtl_bt_hci_ack(p_rx_buf->p_buf);
    k_mem_slab_free(&rx_slab, p_rx_buf);
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
            bt_rtl87x2x_handle_rx_data(p_rx_buf);
            k_yield();
            p_rx_buf = k_fifo_get(&rx.fifo, K_NO_WAIT);
        } while (p_rx_buf);
    }
}

#endif
static int bt_rtl87x2x_send(const struct device *dev, struct net_buf *buf)
{

	int ret = 0;
	uint8_t h4_type = 0;
	T_RTL_BT_HCI_BUF hci_buf = {0};

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT: {
		h4_type = H4_ACL;
	} break;

	case BT_BUF_CMD: {
		h4_type = H4_CMD;
	} break;

	case BT_BUF_ISO_OUT: {
		h4_type = H4_ISO;
	} break;

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
		DBG_DIRECT("[ERR] %s: error, h4_type %d, len %d, ret %d", __func__, h4_type,
			   buf->len, ret);
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
			      K_KERNEL_STACK_SIZEOF(rx_thread_stack), rtl_rx_thread, NULL, NULL,
			      NULL, 0, 0, K_NO_WAIT);
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
	.open = bt_rtl87x2x_open,
	.send = bt_rtl87x2x_send,
	.close = bt_rtl87x2x_close,
};

#define BT_RTL_DEVICE_INIT(inst)                                                                   \
	static struct bt_rtl_data bt_rtl_data_##inst = {};                                         \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &bt_rtl_data_##inst, NULL, POST_KERNEL,            \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported */
BT_RTL_DEVICE_INIT(0)
