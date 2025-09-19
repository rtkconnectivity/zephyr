#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <rtk_dfu_service.h>
#include <ble_dfu_transport.h>

T_APP_RESULT app_dfu_srv_cb(T_DFU_CALLBACK_DATA *p_data)
{
    T_APP_RESULT app_result = APP_RESULT_SUCCESS;
    switch (p_data->msg_type)
    {
    case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
        {
            if (p_data->msg_data.notification_indification_index == DFU_CP_NOTIFY_ENABLE)
            {
                printk("dfu notification enable");
            }
            else if (p_data->msg_data.notification_indification_index ==
                     DFU_CP_NOTIFY_DISABLE)
            {
                printk("dfu notification disable");
            }
        }
        break;
    case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
        {
            uint8_t dfu_write_opcode = p_data->msg_data.write.opcode;
            if (dfu_write_opcode == DFU_WRITE_ATTR_EXIT)
            {
                if (p_data->msg_data.write.write_attrib_index == INDEX_DFU_CHAR_DFU_CONTROL_POINT_INDEX)
                {
                    uint8_t control_point_opcode = *p_data->msg_data.write.p_value;
                    switch (control_point_opcode)
                    {
                    case DFU_OPCODE_VALID_FW:
                        {
//                            T_IO_MSG dfu_valid_fw_msg;
//                            dfu_valid_fw_msg.type = IO_MSG_TYPE_DFU_VALID_FW;
//                            dfu_valid_fw_msg.u.param = p_data->conn_id;
//                            if (app_send_msg_to_apptask(&dfu_valid_fw_msg) == false)
//                            {
//                                DBG_DIRECT("DFU send Valid FW msg fail!");
//                            }
                        }
                        break;
                    case DFU_OPCODE_ACTIVE_IMAGE_RESET:
                        {
#if (ENABLE_AUTO_BANK_SWITCH == 1)
//                            if (is_ota_support_bank_switch())
//                            {
//                                uint32_t ota_addr;
//                                ota_addr = get_header_addr_by_img_id(IMG_OTA);
//                                DFU_PRINT_INFO1("DFU_OPCODE_ACTIVE_IMAGE_RESET: Bank switch erase ota_addr=0x%x", ota_addr);
//                                unlock_flash_bp_all();
//                                flash_nor_erase_locked(ota_addr, FLASH_NOR_ERASE_SECTOR);
//                                lock_flash_bp();
//                            }
#endif
//                            le_disconnect(0);
//                            app_dfu_active_rst_pending = true;
                        }
                        break;
                    default:
                        break;
                    }
                }
            }
            else if (dfu_write_opcode == DFU_WRITE_FAIL)
            {
                printk("DFU FAIL!\n");
            }
            else if (dfu_write_opcode == DFU_WRITE_ATTR_ENTER)
            {
                // application can add check conditions before start dfu procefure
                //if check fail, return some error code except APP_RESULT_SUCCESS to exit dfu procedure
                //app_result = APP_RESULT_REJECT;
                //APP_PRINT_INFO0("exit dfu procedure");
            }
            else
            {
            }
        }
        break;
    default:
        break;
    }

    return app_result;
}