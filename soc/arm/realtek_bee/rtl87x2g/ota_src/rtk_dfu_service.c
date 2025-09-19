#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <ble_dfu_transport.h>
#include <rtk_dfu_service.h>
#include <rtk_ota_app_layer.h>


static void dfu_service_cccd_update_cb(const struct bt_gatt_attr *attr,
				       uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value & BT_GATT_CCC_NOTIFY);

	printk("dfu_cccd_update_cb Notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

static ssize_t dfu_srv_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
	struct bt_uuid_128 *uuid = (struct bt_uuid_128 *) attr->uuid;

	T_APP_RESULT  cause = APP_RESULT_SUCCESS;
    T_DFU_CALLBACK_DATA callback_data;

    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
	if (uuid->val[13] == 0x63) {
    	callback_data.msg_data.write.write_attrib_index = INDEX_DFU_CHAR_DFU_PACKET_INDEX;
	} else if (uuid->val[13] == 0x64) {
		callback_data.msg_data.write.write_attrib_index = INDEX_DFU_CHAR_DFU_CONTROL_POINT_INDEX;
	}
    callback_data.msg_data.write.length = len;
    callback_data.msg_data.write.p_value = (uint8_t *)buf;

    /* Notify Application. */
    callback_data.msg_data.write.opcode = DFU_WRITE_ATTR_ENTER;

    cause = app_dfu_srv_cb((void *)&callback_data);

    if (cause != APP_RESULT_SUCCESS)
    {
        return cause;
    }

	if (uuid->val[13] == 0x63) {
		ble_dfu_service_handle_packet(conn, len, (uint8_t *)buf);
	} else if (uuid->val[13] == 0x64) {
		ble_dfu_service_handle_cp_req(conn, len, (uint8_t *)buf);
	}

    /* Notify Application. */
    callback_data.msg_data.write.opcode = DFU_WRITE_ATTR_EXIT;

	cause = app_dfu_srv_cb((void *)&callback_data);

	if (cause != APP_RESULT_SUCCESS)
	{
		return cause;
	}

	return len;
}	

#define BT_UUID_RTK_DFU_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x00006287, 0x3c17, 0xd293, 0x8e48, 0x14fe2e4da212)

static struct bt_uuid_128 rtk_dfu_srv_uuid = BT_UUID_INIT_128(
	BT_UUID_RTK_DFU_SERVICE_VAL);

BT_GATT_SERVICE_DEFINE(rtk_dfu_service,
    BT_GATT_PRIMARY_SERVICE(&rtk_dfu_srv_uuid),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_DFU_PACKET, BT_GATT_CHRC_WRITE_WITHOUT_RESP,BT_GATT_PERM_WRITE, NULL, dfu_srv_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_DFU_CONTROL_POINT, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_WRITE, NULL, dfu_srv_write_cb, NULL),
	BT_GATT_CCC(dfu_service_cccd_update_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void dfu_service_send_notification(struct bt_conn *conn, uint8_t *p_data, uint16_t data_len)
{
    bt_gatt_notify(conn, &rtk_dfu_service.attrs[4], p_data, data_len);
}