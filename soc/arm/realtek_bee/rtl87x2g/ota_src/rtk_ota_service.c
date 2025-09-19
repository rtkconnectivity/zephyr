#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <ble_dfu_transport.h>
#include <rtk_ota_service.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ota_service);

static uint8_t mac_addr[12]={0};
static DEVICE_INFO device_info;
static uint8_t img_ver[IMG_INFO_LEN];
static uint16_t protocol_info = BLE_PROTOCOL_INFO;
static uint8_t section_size[SECTION_SIZE_LEN];

static ssize_t enter_ota_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
    struct bt_uuid_16 *uuid = (struct bt_uuid_16 *) attr->uuid;
    LOG_INF("OTA SERVICE, UUID: 0x%x",uuid->val);
    
    if ((len != sizeof(uint8_t)) || (buf == NULL))
    {
        return APP_RESULT_INVALID_VALUE_SIZE;
    }

    uint8_t *value = (uint8_t *)buf;

    /* Notify Application. */
    if (OTA_VALUE_ENTER == value[0])
    {
        /*battery level is above 60 percent*/
        printk("Preparing switch into OTA mode");
        /*prepare to enter OTA mode, before switch action, we should disconnect first.*/
        dfu_switch_to_ota_mode_pending = true;
        //le_disconnect(p_data->conn_id);//replace with zephyr
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);//will connect automatically after disconnecting.
    }

    return len;
}

static ssize_t mac_address_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       void *buf, uint16_t len, uint16_t offset)
{
    struct bt_uuid_16 *uuid = (struct bt_uuid_16 *) attr->uuid;
    LOG_INF("OTA SERVICE, UUID: 0x%x",uuid->val);

    bt_addr_le_t addr = {0};
	size_t count = 1;
    
    bt_id_get(&addr, &count);

    for (int i = 0; i < 6; i++) {
        mac_addr[i] = addr.a.val[5-i];
        mac_addr[i+6] = addr.a.val[5-i];
    }

    const uint8_t *value = attr->user_data;

    LOG_HEXDUMP_INF(value, sizeof(mac_addr), "Read Results:");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(mac_addr));
}

static ssize_t device_info_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       void *buf, uint16_t len, uint16_t offset)
{
    struct bt_uuid_16 *uuid = (struct bt_uuid_16 *) attr->uuid;
    LOG_INF("OTA SERVICE, UUID: 0x%x",uuid->val);

    ble_dfu_get_device_info(conn, &device_info);

    const DEVICE_INFO *value = attr->user_data;

    LOG_HEXDUMP_INF(value, sizeof(DEVICE_INFO), "Read Results:");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(DEVICE_INFO));
}

static ssize_t image_version_first_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       void *buf, uint16_t len, uint16_t offset)
{
    struct bt_uuid_16 *uuid = (struct bt_uuid_16 *) attr->uuid;
    LOG_INF("OTA SERVICE, UUID: 0x%x",uuid->val);

    img_ver[0] = ACTIVE_BANK;
    ble_dfu_get_img_version(&img_ver[1], img_ver[0]);

    const uint8_t *value = attr->user_data;

    LOG_INF("version length=%d", img_ver[1] * 6 + 2);

    LOG_HEXDUMP_INF(value, img_ver[1] * 6 + 2, "Read Results:");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 img_ver[1] * 6 + 2);
}

static ssize_t protocol_info_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       void *buf, uint16_t len, uint16_t offset)
{
    struct bt_uuid_16 *uuid = (struct bt_uuid_16 *) attr->uuid;
    LOG_INF("OTA SERVICE: UUID: 0x%x",uuid->val);

    const uint8_t *value = attr->user_data;

    LOG_HEXDUMP_INF(value, sizeof(uint16_t), "Read Results:");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(uint16_t));
}

static ssize_t section_size_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       void *buf, uint16_t len, uint16_t offset)
{
    struct bt_uuid_16 *uuid = (struct bt_uuid_16 *) attr->uuid;
    LOG_INF("OTA SERVICE: UUID: 0x%x",uuid->val);

    ble_dfu_get_section_size(section_size);

    const uint8_t *value = attr->user_data;

    LOG_HEXDUMP_INF(value, section_size[0] * 6 +1, "Read Results:");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 section_size[0] * 6 +1);
}


#define BT_UUID_RTK_OTA_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x0000d0ff, 0x3c17, 0xd293, 0x8e48, 0x14fe2e4da212)

static struct bt_uuid_128 rtk_ota_srv_uuid = BT_UUID_INIT_128(
	BT_UUID_RTK_OTA_SERVICE_VAL);

BT_GATT_SERVICE_DEFINE(rtk_ota_service,
    BT_GATT_PRIMARY_SERVICE(&rtk_ota_srv_uuid),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_OTA, BT_GATT_CHRC_WRITE_WITHOUT_RESP,BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, enter_ota_cb, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_MAC, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, mac_address_read_cb, NULL, mac_addr),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_DEVICE_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, device_info_read_cb, NULL, &device_info),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_IMAGE_VERSION_FIRST, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, image_version_first_read_cb, NULL, img_ver),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_PROTOCOL_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, protocol_info_read_cb, NULL, &protocol_info),
    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_SECTION_SIZE_FIRST, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, section_size_read_cb, NULL, section_size),
);
