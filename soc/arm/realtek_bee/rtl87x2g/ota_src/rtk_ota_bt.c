#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>

#include "rtk_ota_service.h"
#include "wdt.h"
#include "ble_dfu_transport.h"
#include "dfu_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rtk_ota_bt);

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("In rtk_ota_bt, Disconnected (reason 0x%02x)\n", reason);

	if ((reason != BT_HCI_ERR_LOCALHOST_TERM_CONN) && reason != BT_HCI_ERR_REMOTE_USER_TERM_CONN) {
            LOG_ERR("connection lost: cause 0x%x", reason);
    }

    if (dfu_switch_to_ota_mode_pending) {
#if (SUPPORT_NORMAL_OTA == 1)
        dfu_switch_to_ota_mode_pending = false;
        dfu_switch_to_ota_mode();
#endif
    } else {
        if (dfu_active_reset_pending) {
            printk("OTA APP Active reset....");
            dfu_active_reset_pending = false;

#if (ENABLE_AUTO_BANK_SWITCH == 1)
            if (is_ota_support_bank_switch()) {
                uint32_t ota_addr;
                ota_addr = get_header_addr_by_img_id(IMG_OTA);
                DBG_DIRECT("FOR QC Test: Bank switch Erase OTA Header=0x%x", ota_addr);
                fmc_flash_nor_erase(ota_addr, FMC_FLASH_NOR_ERASE_SECTOR);
            }
#endif
            //unlock_flash_bp_all();   //GRACE TO CHECK
            dfu_fw_reboot(RESET_ALL, DFU_ACTIVE_RESET);
            } else {
                /* do nothing. BT would start adv automatically.*/
            }
        }
}

BT_CONN_CB_DEFINE(rtk_ota_bt_callbacks) = {
	.disconnected = disconnected,
};

