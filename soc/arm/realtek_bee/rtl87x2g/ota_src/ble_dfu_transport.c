/**
*****************************************************************************************
*     Copyright(c) 2023, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      ble_dfu_transport.c
   * @brief     Source file for using ble dfu transport
   * @author    Grace
   * @date      2023-12-06
   * @version   v1.1
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2023 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/

#include <string.h>
#include <stdlib.h>
#include "ble_dfu_transport.h"
#include "rtk_dfu_service.h"
#include "dfu_common.h"
#include "os_mem.h"
#include "os_sync.h"
#include "os_timer.h"
#include "fmc_api.h"
#include "fmc_platform.h"
#include "trace.h"
#if (SUPPORT_NORMAL_OTA == 1)
#include "dfu_main.h"
#endif
#include "rtl876x_lib_platform.h"
#include "patch_header_check.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <rtk_ota_app_layer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble_dfu_transport);

/*if enable transfer encrypted air data packet by aes. disable: 0*/
#define ENABLE_OTA_AES                  1

/*============================================================================*
 *                              Variables
 *============================================================================*/
static OTA_FUNCTION_STRUCT ota_struct;
static uint8_t temp_image_total_num = 0;

bool dfu_active_reset_pending = false;
bool dfu_active_reset_to_ota_mode = false;
bool dfu_switch_to_ota_mode_pending = false;


/*============================================================================*
 *                              Private Functions
 *============================================================================*/
/**
    * @brief    get current active bank
    * @param    void
    * @return   0x01:bank0, 0x02:bank1, 0x00:not support bank switch
    */
static uint8_t ble_ota_service_get_active_bank(void)
{
    uint8_t active_bank;
    uint32_t ota_bank0_addr;

    if (is_ota_support_bank_switch())
    {
        ota_bank0_addr = flash_partition_addr_get(PARTITION_FLASH_OTA_BANK_0);

        if (ota_bank0_addr == get_active_ota_bank_addr())
        {
            active_bank = IMAGE_LOCATION_BANK0;
        }
        else
        {
            active_bank = IMAGE_LOCATION_BANK1;
        }
    }
    else
    {
        active_bank = NOT_SUPPORT_BANK_SWITCH;
    }

    return active_bank;
}

static uint8_t ble_ota_service_get_bootpatch_active_bank(void)
{
    uint8_t active_bank;
    uint32_t boot_patch0_addr;

    boot_patch0_addr = flash_partition_addr_get(PARTITION_FLASH_BOOT_PATCH0);

    if (boot_patch0_addr == get_header_addr_by_img_id(IMG_BOOTPATCH))
    {
        active_bank = IMAGE_LOCATION_BANK0;
    }
    else
    {
        active_bank = IMAGE_LOCATION_BANK1;
    }

    return active_bank;
}

static uint8_t ble_ota_service_get_secureapp_active_bank(void)
{
    uint8_t active_bank;
    uint32_t secure_app0_addr;

    if (is_ota_support_bank_switch())
    {
        secure_app0_addr = flash_partition_addr_get(PARTITION_FLASH_SECURE_APP0);

        if (secure_app0_addr == get_header_addr_by_img_id(IMG_SECUREMCUAPP))
        {
            active_bank = IMAGE_LOCATION_BANK0;
        }
        else
        {
            active_bank = IMAGE_LOCATION_BANK1;
        }
    }
    else
    {
        active_bank = NOT_SUPPORT_BANK_SWITCH;
    }

    return active_bank;
}


/**
    * @brief    Wrapper function to send notification to peer
    * @note
    * @param    conn     ID to identify the connection
    * @param    opcode      Notification on the specified opcode
    * @param    len         Notification data length
    * @param    data        Additional notification data
    * @return   void
    */
static void ble_dfu_service_prepare_send_notify(struct bt_conn *conn, uint8_t opcode, uint16_t len,
                                                uint8_t *data)
{
    uint8_t *p_buffer = NULL;
    uint16_t mtu_size;
    uint16_t remain_size = len;
    uint8_t *p_data = data;
    uint16_t send_len;

    if ((data == NULL) || (len == 0))
    {
        return;
    }

    //le_get_conn_param(GAP_PARAM_CONN_MTU_SIZE, &mtu_size, conn);
    mtu_size = bt_gatt_get_mtu(conn);

    LOG_INF("mtu_size:%d",mtu_size);

    p_buffer = malloc(mtu_size);

    if (p_buffer == NULL)
    {
        return;
    }

    if (len < mtu_size - 2)
    {
        p_buffer[0] = DFU_OPCODE_NOTIF;
        p_buffer[1] = opcode;
        memcpy(&p_buffer[2], data, len);

        dfu_service_send_notification(conn, p_buffer, len + 2);

        free(p_buffer);

        return;
    }

    while (remain_size)
    {
        if (remain_size == len)
        {
            p_buffer[0] = DFU_OPCODE_NOTIF;
            p_buffer[1] = opcode;
            memcpy(&p_buffer[2], p_data, mtu_size - 2);
            dfu_service_send_notification(conn, p_buffer, mtu_size);

            p_data += (mtu_size - 2);
            remain_size -= (mtu_size - 2);
            continue;
        }

        send_len = (remain_size > mtu_size) ? mtu_size : remain_size;
        memcpy(p_buffer, p_data, send_len);
        dfu_service_send_notification(conn, p_buffer, send_len);

        p_data += send_len;
        remain_size -= send_len;
    }

    free(p_buffer);
}

/**
    * @brief    check section size before ota
    * @param    void
    * @return   true: check success; false: check fail
    */
static bool dfu_check_section_size(void)
{
    uint32_t section_size = dfu_get_temp_ota_bank_img_size_by_img_id((IMG_ID)ota_struct.image_id);

    if (ota_struct.image_total_length > section_size)
    {
        LOG_ERR("dfu_check_section_size: total size 0x%x, section size 0x%x",
                         ota_struct.image_total_length, section_size);
        return false;
    }

    return true;
}

/**
    * @brief  Reset local variables
    * @return void
    */
static void ble_dfu_clear_local(T_OTA_CLEAR_LOCAL_CAUSE cause)
{
    APP_PRINT_TRACE1("app_ota_clear_local cause: %d", cause);
    temp_image_total_num = 0;
    ota_struct.image_total_length = 0;
    ota_struct.image_id = 0;
    ota_struct.cur_offset = 0;
    ota_struct.next_subimage_offset = 0;
    ota_struct.ota_flag.value = 0;
    ota_struct.ota_temp_buf_used_size = 0;
    if (cause)
    {
        ota_struct.test.value = 0;
    }

    memset(ota_struct.bd_addr, 0, sizeof(ota_struct.bd_addr));
    if (ota_struct.force_temp_mode)
    {
        //force_enable_ota_temp(false);
        ota_struct.force_temp_mode = 0;
    }

    if (ota_struct.p_ota_temp_buf_head != NULL)
    {
        free(ota_struct.p_ota_temp_buf_head);
        ota_struct.p_ota_temp_buf_head = NULL;
        ota_struct.buffer_size = 0;
    }

    if (ota_struct.bp_level)
    {
        uint32_t s;
        s = os_lock();
        fmc_flash_nor_set_bp_lv(FMC_MAIN0_ADDR, ota_struct.bp_level);
        os_unlock(s);
        ota_struct.bp_level = 0;
    }
}

/**
    * @brief  timeout callback for ota
    * @param  timer_id  timer id
    * @param  timer_chann  time channel
    * @return void
    */
static void ble_dfu_timeout_cb(void *p_xtimer)
{
    uint32_t timer_id = 0;

    os_timer_id_get(&p_xtimer, &timer_id);
    LOG_INF("ble_dfu_timeout_cb: timer id 0x%02X", timer_id);

    switch (timer_id)
    {
    case TIMER_ID_DFU_IMAGE_TRANS:
        {
            os_timer_stop(&ota_struct.timer_handle_ota_transfer);
            ble_dfu_clear_local(OTA_IMAGE_TRANS_TIMEOUT);
        }
        break;
    default:
        break;
    }
}

static bool dfu_get_enc_setting(void)
{
    if (ENABLE_OTA_AES)
    {
        return true;
    }
    else
    {
        return false;
    }

}

/**
    * @brief    Handle the start dfu control point
    * @param    p_data     data to be written
    * @return   handle result 0x01:success other:fail
    */
static uint8_t ble_dfu_cp_start_dfu_handle(uint8_t *p_data) //0x01
{
    uint8_t results = DFU_ARV_SUCCESS;

    if (p_data == NULL)
    {
        results = DFU_ARV_FAIL_INVALID_PARAMETER;
        return results;
    }

    if (dfu_get_enc_setting())
    {
        //LOG_INF("start dfu before decrypted: %b", TRACE_BINARY(16, p_data));
        LOG_HEXDUMP_INF(p_data, 16, "start dfu before decrypted:");
        dfu_aes256_decrypt_16byte(p_data);
        //LOG_INF("start dfu after decrypted: %b", TRACE_BINARY(16, p_data));
        LOG_HEXDUMP_INF(p_data, 16, "start dfu after decrypted:");
    }

    T_IMG_CTRL_HEADER_FORMAT *start_dfu_para = (T_IMG_CTRL_HEADER_FORMAT *)p_data;
    ota_struct.force_temp_mode = *(uint8_t *)((T_IMG_CTRL_HEADER_FORMAT *)p_data + 1);
    ota_struct.image_total_length = start_dfu_para->payload_len + DEFAULT_HEADER_SIZE;
    ota_struct.image_id = start_dfu_para->image_id;
    LOG_INF("ble_dfu_cp_start_dfu_handle: force_temp_mode=0x%x", ota_struct.force_temp_mode);
    /* zephyr build not work ,todo */
    LOG_INF("ble_dfu_cp_start_dfu_handle: ic_type=0x%x, CtrlFlag=0x%x, img_id=0x%x ,CRC16=0x%x, Total Length=0x%x",
                     start_dfu_para->ic_type,
                     start_dfu_para->ctrl_flag.value,
                     start_dfu_para->image_id,
                     start_dfu_para->crc16,
                     ota_struct.image_total_length
                    );

    if (dfu_check_section_size() == false)
    {
        LOG_INF("ble_dfu_cp_start_dfu_handle: Image is oversize");
        results = DFU_ARV_FAIL_INVALID_PARAMETER;
        return results;
    }

    if (ota_struct.force_temp_mode)
    {
        //force_enable_ota_temp(true);
    }

    if ((!is_ota_support_bank_switch() && ota_struct.image_id == IMG_OTA)
        || ota_struct.image_id < IMG_DFU_FIRST
        || ((ota_struct.image_id >= IMG_DFU_MAX) &&
            (ota_struct.image_id < IMG_USER_DATA_FIRST)))
    {
        results = DFU_ARV_FAIL_INVALID_PARAMETER;
        return results;
    }

#if (SUPPORT_BL_COPY_SECURE_IMAGE == 1)
    if (is_ota_support_bank_switch())
    {
        if ((ota_struct.image_id >= IMG_BANK_FIRST && ota_struct.image_id < IMG_DFU_MAX) ||
            ota_struct.image_id == IMG_OTA)
        {
            /*only the images located in OTA Bank need set to 0 when dual bank*/
            ota_struct.next_subimage_offset = 0;
        }
        else
        {
            /*restore next image offset*/
            ota_struct.next_subimage_offset = ota_struct.tmp_next_subimage_offset;
        }
    }
#else
    if (is_ota_support_bank_switch())
    {
        ota_struct.next_subimage_offset = 0;
    }
    else
    {
        if (ota_struct.image_id == IMG_BOOTPATCH ||
            (ota_struct.image_id >= IMG_USER_DATA_FIRST && ota_struct.image_id < IMG_USER_DATA_MAX))
        {
            /*bootpatch always dual bank*/
            ota_struct.next_subimage_offset = 0;
        }
        else
        {
            /*restore next image offset*/
            ota_struct.next_subimage_offset = ota_struct.tmp_next_subimage_offset;
        }
    }
#endif

    ota_struct.ota_flag.is_ota_process = true;
    ota_struct.ota_temp_buf_used_size = 0;
    ota_struct.cur_offset = 0;
#if (SUPPORT_NORMAL_OTA == 1)
    if (is_normal_ota_mode)
    {
        uint32_t restart_ms = (ota_struct.image_total_length + 0x19000 - 1) / 0x19000 *
                              NORMAL_OTA_TIMEOUT_TOTAL * 1000;
        os_timer_restart(&normal_ota_total_timer_handle, restart_ms);
        LOG_INF("ble_dfu_cp_start_dfu_handle: restart total timer %d ms", restart_ms);
    }
#endif

    uint32_t s;
    s = os_lock();
    fmc_flash_nor_set_bp_lv(FMC_MAIN0_ADDR, 0);
    os_unlock(s);

    return results;
}

/**
    * @brief    Valid the image
    * @param    p_date     point of input data
    * @return   valid result
    */
static uint8_t ble_dfu_cp_valid_handle(uint8_t *p_data) //0x03
{
    uint8_t results = DFU_ARV_SUCCESS;
    uint16_t image_id;

    if (!ota_struct.ota_flag.skip_flag
        && ota_struct.test.t_skip_fail)
    {
        ota_struct.test.t_skip_fail = 0;
        results = DFU_ARV_FAIL_CRC_ERROR;
        return results;
    }

    ota_struct.ota_flag.skip_flag = 0;

    if (p_data == NULL)
    {
        results = DFU_ARV_FAIL_INVALID_PARAMETER;
        return results;
    }

    LE_ARRAY_TO_UINT16(image_id, p_data);
    LE_ARRAY_TO_UINT8(ota_struct.is_last_image, p_data + 2);
    LOG_INF("==>ble_dfu_cp_valid_handle: img_id=0x%x, ota_struct.image_id=0x%x, is_last_image=%d",
                     image_id, ota_struct.image_id, ota_struct.is_last_image);

    if (image_id == ota_struct.image_id)
    {
        if (!dfu_checksum((IMG_ID)image_id, ota_struct.next_subimage_offset))
        {
            results = DFU_ARV_FAIL_CRC_ERROR;
        }
        else
        {
            //user data image needn't record into temp info
            if (ota_struct.image_id >= IMG_DFU_FIRST && ota_struct.image_id < IMG_DFU_MAX)
            {
                bool bank_switch = is_ota_support_bank_switch();
#if (SUPPORT_BL_COPY_SECURE_IMAGE == 1)
                bool is_ota_bank_image = (ota_struct.image_id == IMG_OTA) ||
                                         (ota_struct.image_id >= IMG_BANK_FIRST && ota_struct.image_id < IMG_DFU_MAX);
                if ((bank_switch && !is_ota_bank_image)
                    || !bank_switch)
#else
                if (!bank_switch && ota_struct.image_id != IMG_BOOTPATCH)
#endif
                {
                    ota_struct.tmp_next_subimage_offset += ota_struct.cur_offset;
                    ota_struct.next_subimage_offset = ota_struct.tmp_next_subimage_offset;

                    ota_struct.temp_image_info[temp_image_total_num].image_offset =
                        ota_struct.next_subimage_offset - ota_struct.image_total_length;

                    LOG_INF("==>ble_dfu_cp_valid_handle:temp info idx %d,  image_id 0x%x, image_offset 0x%x in temp bank",
                                     temp_image_total_num, ota_struct.image_id,
                                     ota_struct.temp_image_info[temp_image_total_num].image_offset);
                    temp_image_total_num ++;
                }

                ota_struct.cur_offset = 0;
                /* set updated images valid */
                SET_VALID_BITMAP(image_id);

                LOG_INF("==>ble_dfu_cp_valid_handle: valid_bitmap= 0x%x", valid_bitmap);
            }

            if (ota_struct.image_id >= IMG_USER_DATA_FIRST && ota_struct.image_id < IMG_USER_DATA_MAX)
            {
                SET_USER_DATA_VALID_BITMAP(image_id);
            }

            LOG_INF("==>ble_dfu_cp_valid_handle: checksum Success! temp bank next_subimage_offset= 0x%x",
                             ota_struct.next_subimage_offset);
        }
    }
    else
    {
        results = DFU_ARV_FAIL_INVALID_PARAMETER;
    }

    return results;
}

static void ble_dfu_bootpatch_always_bank_switch(void)
{
    if (ota_struct.test.t_stress_test && GET_VALID_BITMAP(IMG_BOOTPATCH))
    {
        LOG_INF("<==ble_dfu_bootpatch_always_bank_switch");
        uint32_t image_addr = get_header_addr_by_img_id(IMG_BOOTPATCH);
        fmc_flash_nor_erase(image_addr, FMC_FLASH_NOR_ERASE_SECTOR);
    }
}

static void  dfu_set_ota_temp_image_ready(void)
{
    uint32_t base_addr = 0;
    T_IMG_HEADER_FORMAT *p_header = NULL;
    base_addr = flash_partition_addr_get(PARTITION_FLASH_OTA_TMP);

    LOG_INF("<==dfu_set_ota_temp_image_ready: ota_temp_addr =0x%x, temp_image_total_num %d",
                    base_addr, temp_image_total_num);

    for (int i = temp_image_total_num - 1; i >= 0; i--)
    {
        p_header = (T_IMG_HEADER_FORMAT *)(base_addr + ota_struct.temp_image_info[i].image_offset);
        if (p_header)
        {
            dfu_set_ready(p_header);
        }
    }
}

static void ble_dfu_cp_active_reset_handle(struct bt_conn *conn, bool is_enter_dfu_mode)
{
    T_IMG_HEADER_FORMAT *p_header = NULL;
    uint16_t img_id = IMG_DFU_MAX;

    if (!is_ota_support_bank_switch())
    {
        dfu_set_ota_temp_image_ready();

#if (SUPPORT_BL_COPY_SECURE_IMAGE == 0)
        img_id = IMG_BOOTPATCH;
        if (GET_VALID_BITMAP(img_id))
        {
            p_header = (T_IMG_HEADER_FORMAT *)dfu_get_temp_ota_bank_img_addr_by_img_id((IMG_ID)img_id);
            if (p_header)
            {
                dfu_set_ready(p_header);
            }
        }
#endif
    }
    else
    {
#if (SUPPORT_BL_COPY_SECURE_IMAGE == 1)
        /* support bank switch */
        img_id = IMG_DFU_MAX - 1;

        for (; img_id >= IMG_BANK_FIRST; img_id--)
        {
            if (GET_VALID_BITMAP(img_id))
            {
                p_header = (T_IMG_HEADER_FORMAT *)dfu_get_temp_ota_bank_img_addr_by_img_id((IMG_ID)img_id);
                if (p_header)
                {
                    dfu_set_ready(p_header);
                }
            }
        }

        img_id = IMG_OTA;
        if (GET_VALID_BITMAP(img_id))
        {
            p_header = (T_IMG_HEADER_FORMAT *)dfu_get_temp_ota_bank_img_addr_by_img_id((IMG_ID)img_id);
            if (p_header)
            {
                dfu_set_ready(p_header);
            }
        }

        dfu_set_ota_temp_image_ready();
#else
        /* support bank switch */
        img_id = IMG_DFU_MAX - 1;

        /*include boot patch and secure app or data*/
        for (; img_id >= IMG_DFU_FIRST; img_id--)
        {
            if (GET_VALID_BITMAP(img_id))
            {
                p_header = (T_IMG_HEADER_FORMAT *)dfu_get_temp_ota_bank_img_addr_by_img_id((IMG_ID)img_id);
                if (p_header)
                {
                    dfu_set_ready(p_header);
                }
            }
        }
#endif
    }

    /*set user data image ready*/
    img_id = IMG_USER_DATA_FIRST;
    for (; img_id < IMG_USER_DATA_MAX; img_id++)
    {
        if (GET_USER_DATA_VALID_BITMAP(img_id))
        {
            p_header = (T_IMG_HEADER_FORMAT *)dfu_get_temp_ota_bank_img_addr_by_img_id((IMG_ID)img_id);
            if (p_header)
            {
                dfu_set_ready(p_header);
            }
        }
    }

#if (SUPPORT_NORMAL_OTA == 1)
    LOG_INF("==>ble_dfu_cp_active_reset_handle: is_enter_dfu_mode=%d, is_normal_ota_mode=%d",
                    is_enter_dfu_mode, is_normal_ota_mode);
    if (is_enter_dfu_mode && is_normal_ota_mode)
    {
        //need reset to ota mode
        dfu_active_reset_to_ota_mode = true;
    }
#endif

    ble_dfu_bootpatch_always_bank_switch();

    //le_disconnect(conn);
    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

    dfu_active_reset_pending = true;

    ble_dfu_clear_local(OTA_SUCCESS_REBOOT);

}

/**
    * @brief    get image info for ota
    * @param    *p_data   point of input data
    * @param    *data   point of output data
    * @return   void
    */
static void ble_dfu_cp_report_img_info_handle(uint8_t *p_data, uint8_t *p_notify_data) //0x06
{
    uint16_t image_id;
    uint32_t dfu_base_addr;
    uint32_t s;

    if ((p_notify_data == NULL) || (p_data == NULL))
    {
        return;
    }

    LE_ARRAY_TO_UINT16(image_id, p_data);
    LOG_INF("ble_dfu_cp_report_img_info_handle: received img_id 0x%x, ota_struct.image_id=0x%x",
                     image_id, ota_struct.image_id);

    if (image_id != ota_struct.image_id)
    {
        ota_struct.cur_offset = 0;
        ota_struct.ota_temp_buf_used_size = 0;
        ota_struct.image_id = image_id;
    }

    if (image_id < IMG_DFU_FIRST ||
        ((image_id >= IMG_DFU_MAX) && (image_id < IMG_USER_DATA_FIRST)))
    {
        LOG_ERR("ble_dfu_cp_report_img_info_handle: received img_id 0x%x invalid!", image_id);

        p_notify_data[0] = DFU_ARV_FAIL_INVALID_PARAMETER;
        return;
    }

    dfu_base_addr = dfu_get_temp_ota_bank_img_addr_by_img_id((IMG_ID)image_id);
    if ((dfu_base_addr % FLASH_SECTOR_SIZE) == 0)
    {
        ota_struct.buffer_check_offset = 0;
    }
    else
    {
        ota_struct.buffer_check_offset = dfu_base_addr % FLASH_SECTOR_SIZE;
        s = os_lock();
        fmc_flash_nor_read(dfu_base_addr - ota_struct.buffer_check_offset,
                           ota_struct.p_ota_temp_buf_head,
                           ota_struct.buffer_check_offset);
        os_unlock(s);
    }

    p_notify_data[0] = DFU_ARV_SUCCESS;
    LE_UINT32_TO_ARRAY(&p_notify_data[1], 0);
    LE_UINT32_TO_ARRAY(&p_notify_data[5], ota_struct.cur_offset);
    LE_UINT16_TO_ARRAY(&p_notify_data[9], ota_struct.buffer_check_offset);

    LOG_INF("ble_dfu_cp_report_img_info_handle: Notify cur_offset=0x%x, buffer_check_offset=0x%x",
                     ota_struct.cur_offset, ota_struct.buffer_check_offset);
}


static void ble_dfu_cp_conn_para_update_handle(struct bt_conn *conn, uint8_t *p_data, uint8_t *p_notify_data) //0x06
{
    uint16_t conn_interval_min;
    uint16_t conn_interval_max;
    uint16_t conn_latency;
    uint16_t superv_tout;

    if ((p_notify_data == NULL) || (p_data == NULL))
    {
        return;
    }

    p_notify_data[0] = DFU_ARV_SUCCESS;

    LE_ARRAY_TO_UINT16(conn_interval_min, p_data);
    LE_ARRAY_TO_UINT16(conn_interval_max, p_data + 2);
    LE_ARRAY_TO_UINT16(conn_latency, p_data + 4);
    LE_ARRAY_TO_UINT16(superv_tout, p_data + 6);
    LOG_INF("==>le_dfu_cp_conn_para_update_handle: conn_min=0x%x, conn_max=0x%x, latcy=0x%x, timeout=0x%x",
                    conn_interval_min, conn_interval_max, conn_latency, superv_tout);


    if (ota_struct.dfu_conn_para_update_in_progress)
    {
        p_notify_data[0] = DFU_ARV_FAIL_OPERATION;
        LOG_INF("==>le_dfu_cp_conn_para_update_handle: OTA ConnParaUpdInProgress!");
    }
    else
    {
        /* Change to zephyr host API. Note!!! Zephyr doesnot have min_ce_length & max_ce_length */
        struct bt_le_conn_param param;
        param.interval_max = conn_interval_max;
        param.interval_min = conn_interval_min;
        param.latency = conn_latency;
        param.timeout = superv_tout;

        if (bt_conn_le_param_update(conn, &param) == 0) {
            /* Connection Parameter Update Request sent successfully, means this procedure is in progress. */
            ota_struct.dfu_conn_para_update_in_progress = true;
            LOG_INF("==>le_dfu_cp_conn_para_update_handle: updating");
        } else {
            p_notify_data[0] = DFU_ARV_FAIL_OPERATION;
            return;
        }
    }
    LOG_INF("ble_dfu_cp_report_img_info_handle: cur_offset=0x%x, buffer_check_offset=0x%x",
                     ota_struct.cur_offset, ota_struct.buffer_check_offset);
}
/**
    * @brief    Handle buffer check enable cmd
    * @param    p_data      point of output data
    * @param    max_buffer_size     set max buffer size
    * @return   void
    */
static void ble_dfu_cp_buffer_check_en_handle(uint8_t *p_notify_data,
                                              uint16_t max_buffer_size)  //0x09
{
    uint8_t size_factor = 0;

    if (p_notify_data == NULL)
    {
        return;
    }

    ota_struct.ota_flag.buffer_check_en = true;

    while (ota_struct.p_ota_temp_buf_head == NULL)
    {
        ota_struct.buffer_size = max_buffer_size >> size_factor;
        ota_struct.p_ota_temp_buf_head = (uint8_t *)malloc(ota_struct.buffer_size);
        //ota_struct.p_ota_temp_buf_head = (uint8_t *)os_mem_alloc(0, ota_struct.buffer_size);
        size_factor ++;
        if (ota_struct.buffer_size <= 0)
        {
            break;
        }
    }

    if (ota_struct.p_ota_temp_buf_head == NULL
        || ota_struct.buffer_size == 0)
    {
        ota_struct.ota_flag.buffer_check_en = false;
    }
    p_notify_data[0] = ota_struct.ota_flag.buffer_check_en;
    LE_UINT16_TO_ARRAY(&p_notify_data[1], ota_struct.buffer_size);
    //LE_UINT16_TO_ARRAY(&p_notify_data[3], ota_struct.mtu_size);  //ota verion =4, is rsvd val

    LOG_INF("ble_dfu_cp_buffer_check_en: buffer_check_en=%d, buffer_size=%d",
                     ota_struct.ota_flag.buffer_check_en, ota_struct.buffer_size);
}

/**
    * @brief    Handle written request on DFU packet
    * @param    p_data     point of input data
    * @param    data      point of output data
    * @return   void
    */
static void ble_dfu_cp_buffer_check_handle(uint8_t *p_data, uint8_t *data)  //0x0a
{
    uint16_t data_size;
    uint16_t crc;

    T_DFU_CALLBACK_DATA callback_data;
    //callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    callback_data.msg_data.write.write_attrib_index = INDEX_DFU_CHAR_DFU_CONTROL_POINT_INDEX;

    if ((p_data == NULL) || (ota_struct.ota_flag.is_ota_process == false))
    {
        ota_struct.ota_temp_buf_used_size = 0;
        data[0] = DFU_ARV_FAIL_OPERATION;
        LE_UINT32_TO_ARRAY(&data[1], ota_struct.cur_offset);
        return;
    }

    LE_ARRAY_TO_UINT16(data_size, p_data);
    LE_ARRAY_TO_UINT16(crc, p_data + 2);

    LOG_INF("===>ble_dfu_cp_buffer_check_handle: data_size=%d, crc=0x%x, ota_temp_buf_used_size=%d, buffer_check_offset=%d",
                     data_size, crc, ota_struct.ota_temp_buf_used_size, ota_struct.buffer_check_offset);

    if (data_size != ota_struct.ota_temp_buf_used_size)
    {
        ota_struct.ota_temp_buf_used_size = 0;
        data[0] = DFU_ARV_FAIL_DATA_LENGTH_ERROR;
        LE_UINT32_TO_ARRAY(&data[1], ota_struct.cur_offset);
        return;
    }

    if (dfu_check_bufcrc(ota_struct.p_ota_temp_buf_head + ota_struct.buffer_check_offset,
                         ota_struct.ota_temp_buf_used_size, crc))
    {
        ota_struct.ota_temp_buf_used_size = 0;
        ota_struct.buffer_check_offset = 0;
        data[0] = DFU_ARV_FAIL_CRC_ERROR;
        LE_UINT32_TO_ARRAY(&data[1], ota_struct.cur_offset);
        LOG_INF("ble_dfu_cp_buffer_check_handle: buffer check fail!");
    }
    else
    {
        LOG_INF("ble_dfu_cp_buffer_check_handle: write flash, offset=0x%x", ota_struct.cur_offset);
        if (dfu_get_enc_setting())
        {
            uint16_t offset = 0;
            while ((ota_struct.ota_temp_buf_used_size - offset) >= 16)
            {
                dfu_aes256_decrypt_16byte(ota_struct.p_ota_temp_buf_head + offset);
                offset += 16;
            }
        }

        uint32_t flash_write_result = dfu_write_data_to_flash(ota_struct.image_id,
                                                              ota_struct.cur_offset,
                                                              ota_struct.next_subimage_offset,
                                                              ota_struct.ota_temp_buf_used_size + ota_struct.buffer_check_offset,
                                                              ota_struct.p_ota_temp_buf_head);

        LOG_INF("ble_dfu_cp_buffer_check_handle: write ret %d", flash_write_result);

        if (flash_write_result == 0)
        {
            uint32_t updated_success_len = ota_struct.ota_temp_buf_used_size;
            callback_data.msg_data.write.opcode = DFU_WRITE_DOING;
            callback_data.msg_data.write.length = 4;
            callback_data.msg_data.write.p_value = (uint8_t *)&updated_success_len;

            T_APP_RESULT cause = app_dfu_srv_cb((void *)&callback_data);

            if (cause != APP_RESULT_SUCCESS)
            {
                return;
            }
            
            ota_struct.cur_offset += ota_struct.ota_temp_buf_used_size;
            ota_struct.ota_temp_buf_used_size = 0;
            ota_struct.buffer_check_offset = 0;
            data[0] = DFU_ARV_SUCCESS;
            LE_UINT32_TO_ARRAY(&data[1], ota_struct.cur_offset);
        }
        else
        {
            uint32_t erase_time = 0;
            uint32_t resend_offset = (ota_struct.next_subimage_offset + ota_struct.cur_offset) /
                                     FLASH_SECTOR_SIZE * FLASH_SECTOR_SIZE;

            if (resend_offset < ota_struct.next_subimage_offset)
            {
                ota_struct.ota_temp_buf_used_size = 0;
                data[0] = DFU_ARV_FAIL_FLASH_ERASE_ERROR;
                LE_UINT32_TO_ARRAY(&data[1], ota_struct.cur_offset);
                return;
            }

            ota_struct.cur_offset = resend_offset - ota_struct.next_subimage_offset;
            LOG_INF("ble_dfu_cp_buffer_check_handle: write fail! cur_offset=0x%x, resend_offset=0x%x, next_subimage_offset=0x%x",
                             ota_struct.cur_offset, resend_offset, ota_struct.next_subimage_offset);

            while (erase_time < 3)
            {
                if (dfu_flash_erase((IMG_ID)ota_struct.image_id, resend_offset) == true)
                {
                    ota_struct.ota_temp_buf_used_size = 0;
                    data[0] = DFU_ARV_FAIL_FLASH_WRITE_ERROR;
                    LE_UINT32_TO_ARRAY(&data[1], ota_struct.cur_offset);
                    break;
                }
                else
                {
                    erase_time++;
                }
            }
            if (erase_time >= 3)
            {
                ota_struct.ota_temp_buf_used_size = 0;
                data[0] = DFU_ARV_FAIL_FLASH_ERASE_ERROR;
                LE_UINT32_TO_ARRAY(&data[1], ota_struct.cur_offset);
                LOG_INF("ble_dfu_cp_buffer_check_handle: erase fail more than three times!");

                T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_UPDATE_FLASH;
                callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;
                callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
                callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;

                T_APP_RESULT cause = app_dfu_srv_cb((void *)&callback_data);
                
                if (cause != APP_RESULT_SUCCESS) {
                    return;
                }

            }
        }
    }
}

/**
    * @brief    copy image to the other bank
    * @param    p_data    point of input data
    * @return   1: success  other: fail
    */
static uint8_t ble_dfu_cp_copy_img_handle(uint8_t *p_data) //0x0c
{
    uint32_t source_base_addr;
    T_IMG_CTRL_HEADER_FORMAT ctrl_header;
    uint32_t offset = 0;
    uint8_t ret = DFU_ARV_SUCCESS;
    uint16_t img_id;
    uint8_t *buffer_addr = ota_struct.p_ota_temp_buf_head;
    uint32_t buffer_size = MAX_BUFFER_SIZE;
    uint8_t *p_copy_buffer = NULL;
    uint8_t bp_lv;
    uint8_t size_factor = 0;
    uint32_t s;

    ota_struct.ota_flag.skip_flag = 1;

    if (ota_struct.test.t_copy_fail)
    {
        ota_struct.test.t_copy_fail = 0;
        ret = DFU_ARV_FAIL_OPERATION;
        return ret;
    }

    if (p_data == NULL)
    {
        ret = DFU_ARV_FAIL_OPERATION;
        return ret;
    }

    LE_ARRAY_TO_UINT16(img_id, p_data);

    if ((img_id < IMG_MCUAPPDATA1) || (img_id >= IMG_DFU_MAX))
    {
        ret = DFU_ARV_FAIL_INVALID_PARAMETER;
        return ret;
    }

    source_base_addr = get_header_addr_by_img_id((IMG_ID)img_id);

    if ((source_base_addr % FLASH_SECTOR_SIZE) || (source_base_addr == 0))
    {
        ret = DFU_ARV_FAIL_OPERATION;
        return ret;
    }

    T_IMG_HEADER_FORMAT *p_data_header = (T_IMG_HEADER_FORMAT *)source_base_addr;
    s = os_lock();
    fmc_flash_nor_read((uint32_t)&p_data_header->ctrl_header, &ctrl_header, sizeof(ctrl_header));
    os_unlock(s);
    if (ctrl_header.image_id != img_id)
    {
        ret = DFU_ARV_FAIL_OPERATION;
        return ret;
    }

    uint32_t remain_size = ctrl_header.payload_len + DEFAULT_HEADER_SIZE;

    if (ota_struct.p_ota_temp_buf_head != NULL)
    {
        buffer_size = ota_struct.buffer_size;
    }
    else
    {
        while (p_copy_buffer == NULL && buffer_size)
        {
            buffer_size >>= size_factor;
            p_copy_buffer = (uint8_t *)malloc(buffer_size);
            size_factor ++;
        }

        buffer_addr = p_copy_buffer;
    }

    if (buffer_addr == NULL)
    {
        ret = DFU_ARV_FAIL_OPERATION;
        return ret;
    }

    s = os_lock();
    fmc_flash_nor_get_bp_lv(FMC_MAIN0_ADDR, &bp_lv);  //use dest addr is more actually
    os_unlock(s);

    uint32_t copy_len;
    while (remain_size > 0)
    {
        copy_len = (remain_size > buffer_size) ? buffer_size : remain_size;

        s = os_lock();
        fmc_flash_nor_read(source_base_addr + offset, buffer_addr, copy_len);
        os_unlock(s);
        if (dfu_write_data_to_flash(img_id, offset, 0, copy_len, buffer_addr) != 0) //check grace
        {
            ret = DFU_ARV_FAIL_FLASH_WRITE_ERROR;
            return ret;
        }
        remain_size -= copy_len;
        offset += copy_len;
    }

    if (dfu_checksum((IMG_ID)img_id, 0) == false)
    {
        ret = DFU_ARV_FAIL_CRC_ERROR;
    }

    if (p_copy_buffer != NULL)
    {
        free(p_copy_buffer);
    }

    s = os_lock();
    fmc_flash_nor_set_bp_lv(FMC_MAIN0_ADDR, bp_lv);
    os_unlock(s);

    LOG_INF("ble_dfu_cp_copy_img_handle: ret=%d image_id=0x%x", ret, img_id);

    return ret;
}

/**
    * @brief    compare sha256 or crc value with the image in dest addr
    * @param    p_dest    address of the image
    * @param    p_data    sha256 or crc value receive from controler
    * @param    img_id    image id
    * @return   true: same  false: different
    */
static bool ble_dfu_cmp_checksum(uint8_t *p_dest, uint8_t *p_data, IMG_ID img_id)
{
    uint8_t buffer[SHA256_LEN];
    uint32_t offset = SHA256_OFFSET;
    uint32_t len = SHA256_LEN;
    uint32_t s;

    if (img_id >= IMG_DFU_MAX || img_id < IMG_DFU_FIRST)
    {
        return false;
    }

    s = os_lock();
    fmc_flash_nor_read((uint32_t)(p_dest + offset), buffer, len);
    os_unlock(s);

    return (memcmp(p_data, buffer, len) == 0);
}

/**
    * @brief    To check if the image in device is same with image needed to update
    * @param    p_data    point of input data
    * @param    data    point of output data
    * @return   void
    */
static void ble_dfu_cp_check_sha256_handle(uint8_t *p_data, uint8_t *p_notify_data) //0x0f
{
    T_IMG_HEADER_FORMAT *img_addr[2];
    uint16_t num = *(uint16_t *)p_data;
    uint8_t buffer[DFU_NOTIFY_LEN_CHECK_SHA256] = {0};

    if (p_notify_data == NULL)
    {
        return;
    }

    SHA256_CHECK *p_src = (SHA256_CHECK *)(p_data + 2);
    LOG_INF("===>ble_dfu_cp_check_sha256_handle: img_id=0x%x", p_src->img_id);

    if (p_src->img_id >= IMG_DFU_MAX || p_src->img_id < IMG_DFU_FIRST)
    {
        return;
    }

    for (uint16_t i = 0; i < num; i++)
    {
        memset(buffer, 0, sizeof(buffer));
        *(uint16_t *)buffer = p_src->img_id;

        img_addr[0] = (T_IMG_HEADER_FORMAT *)dfu_get_temp_ota_bank_img_addr_by_img_id(p_src->img_id);
        img_addr[1] = (T_IMG_HEADER_FORMAT *)get_header_addr_by_img_id(p_src->img_id);

        for (uint8_t j = 0; j < 2; j++)
        {
            if (img_addr[j] && img_addr[j]->ctrl_header.image_id == p_src->img_id
                && ble_dfu_cmp_checksum((uint8_t *)img_addr[j], p_src->sha256, p_src->img_id))
            {
                buffer[2] += (j + 1);
            }
        }
        LOG_INF("<===ble_dfu_cp_check_sha256_handle: img_id=0x%x, check resut=%x",
                         *(uint16_t *)buffer, buffer[2]);

        memcpy(p_notify_data, buffer, sizeof(buffer));
        p_notify_data += sizeof(buffer);
        p_src++;
    }
}

/**
    * @brief    Handle written request on DFU packet
    * @param    p_data     data to be written
    * @param    length     Length of value to be written
    * @return   handle result  0x01:success other:fail
    */
static uint8_t ble_dfu_packet_handle(uint8_t *p_data, uint16_t length)
{
    uint8_t results = DFU_ARV_SUCCESS;

    LOG_INF("===>ble_dfu_packet_handle: length=%d, nCurOffSet=%d, buffer_check_offset=%d, nImageTotalLength=%d",
                     length, ota_struct.cur_offset, ota_struct.buffer_check_offset, ota_struct.image_total_length);

    ota_struct.ota_flag.skip_flag = 1;

    if ((p_data == NULL) || (ota_struct.ota_flag.is_ota_process == false))
    {
        results = DFU_ARV_FAIL_OPERATION;
        return results;
    }

    if (ota_struct.cur_offset + length + ota_struct.ota_temp_buf_used_size >
        ota_struct.image_total_length)
    {
        results = DFU_ARV_FAIL_INVALID_PARAMETER;
    }
    else
    {
        if (!ota_struct.ota_flag.buffer_check_en)
        {
            if (dfu_get_enc_setting())
            {
                uint16_t offset = 0;
                while ((length - offset) >= 16)
                {
                    dfu_aes256_decrypt_16byte(p_data + offset);
                    offset += 16;
                }
            }

            if (dfu_write_data_to_flash(ota_struct.image_id,
                                        ota_struct.cur_offset,
                                        ota_struct.next_subimage_offset,
                                        length,
                                        p_data) != 0)
            {
                results = DFU_ARV_FAIL_OPERATION;
            }
            else
            {
                ota_struct.cur_offset += length;
            }
        }
        else
        {
            LOG_INF("===>ble_dfu_packet_handle: bufcheck Enable! ota_temp_buf_used_size=0x%x, buffer_size=0x%x",
                             ota_struct.ota_temp_buf_used_size, ota_struct.buffer_size);
            if (ota_struct.ota_temp_buf_used_size + ota_struct.buffer_check_offset + length <=
                ota_struct.buffer_size)
            {
                memcpy(ota_struct.p_ota_temp_buf_head + ota_struct.buffer_check_offset +
                       ota_struct.ota_temp_buf_used_size, p_data, length);
                ota_struct.ota_temp_buf_used_size += length;
            }
            else
            {
                results = DFU_ARV_FAIL_OPERATION;
            }
        }
    }

    return results;
}

/*============================================================================*
 *                              Public Functions
 *============================================================================*/
/**
    * @brief    Used to get device information
    * @param    p_data    point of device info data
    * @param    ota_mode  spp ota or ble ota
    * @return   void
    */
void ble_dfu_get_device_info(struct bt_conn *conn, DEVICE_INFO *p_deviceinfo)
{
    if (p_deviceinfo == NULL)
    {
        return;
    }

    memset(p_deviceinfo, 0, sizeof(DEVICE_INFO));
    p_deviceinfo->ic_type = dfu_get_ic_type();
    p_deviceinfo->spec_ver = BLE_OTA_VERSION;

    p_deviceinfo->mode.buffercheck_en = 1; //ota_struct.test.t_buffercheck_disable ? 0 : 1;
    p_deviceinfo->mode.aes_en = dfu_get_enc_setting();
    p_deviceinfo->mode.aes_mode = 1;
    p_deviceinfo->mode.support_multiimage = 1;
    p_deviceinfo->mode.is_support_normal_ota = SUPPORT_NORMAL_OTA;  //means support normal ota
    p_deviceinfo->mode.is_ota_support_dual_bank_merge = is_ota_support_dual_bank_merge();
    p_deviceinfo->ota_temp_size = flash_partition_size_get(PARTITION_FLASH_OTA_TMP) /
                                  FLASH_SECTOR_SIZE; //uint 4KB
    p_deviceinfo->active_banknum = ble_ota_service_get_active_bank();
    p_deviceinfo->bootpatch_active_banknum = ble_ota_service_get_bootpatch_active_bank();
    p_deviceinfo->secureapp_active_banknum = ble_ota_service_get_secureapp_active_bank();
    p_deviceinfo->ctrl_header_offset = MEMBER_OFFSET(T_IMG_HEADER_FORMAT, ctrl_header);
    
    // le_get_conn_param(GAP_PARAM_CONN_MTU_SIZE, &ota_struct.mtu_size, conn);//RTK UPPERSTACK VERSION
    ota_struct.mtu_size = bt_gatt_get_mtu(conn);//ZEPHYR host VERSION

    ota_struct.ota_flag.is_devinfo = 1;
    ota_struct.ota_flag.is_support_mutil_image = p_deviceinfo->mode.support_multiimage;

    uint32_t s;
    s = os_lock();
    fmc_flash_nor_get_bp_lv(FMC_MAIN0_ADDR, &ota_struct.bp_level);
    os_unlock(s);
    LOG_INF("<===ble_dfu_get_device_info: aes_en=%d,bufcheck_en=%d, active_banknum(OTA BANK%d,BOOTPATCH%d,SECUREAPP%d)",
                    p_deviceinfo->mode.aes_en, p_deviceinfo->mode.buffercheck_en,
                    p_deviceinfo->active_banknum, p_deviceinfo->bootpatch_active_banknum,
                    p_deviceinfo->secureapp_active_banknum);
    LOG_INF("p_deviceinfo->mode.is_ota_support_dual_bank_merge %d",
                    p_deviceinfo->mode.is_ota_support_dual_bank_merge);
    LOG_INF("<===ble_dfu_get_device_info: ota_temp_size=%d,mtu_size=%d,bp_lv=%d",
                    p_deviceinfo->ota_temp_size, ota_struct.mtu_size, ota_struct.bp_level);
}

/**
    * @brief    Used to get image version
    * @param    *p_data   point of image version
    * @param    bank   0:active bank    1:inactive bank
    * @return   void
    */
void ble_dfu_get_img_version(uint8_t *p_data, uint8_t bank)
{
    uint8_t num = 0;
    IMG_ID img_id;
    uint8_t buffer[6] = {0};
    uint8_t *p_temp = p_data + 1;
    uint32_t addr = 0;
    uint32_t size = 0;
    T_IMAGE_VERSION image_version;

    //is_ota_support_bank_switch()
    for (img_id = IMG_DFU_FIRST; img_id < IMG_DFU_MAX; img_id++)
    {
        if (bank == ACTIVE_BANK)
        {
            size = get_active_bank_image_size_by_img_id(img_id);
            get_ota_bank_image_version(true, img_id, (T_IMAGE_VERSION *)&image_version);
        }
        else
        {
            if (is_ota_support_bank_switch())
            {
                size = get_temp_ota_bank_img_size_by_img_id(img_id);
                get_ota_bank_image_version(false, img_id, (T_IMAGE_VERSION *)&image_version);
            }
        }

        if (size == 0 || size == 0xffffffff)
        {
            continue;
        }

        *(uint16_t *)buffer = img_id;
        *(uint32_t *)&buffer[2] = image_version.ver_info.version;

        LOG_INF("img_id:%x",img_id);

        num ++;
        memcpy(p_temp, buffer, sizeof(buffer));
        p_temp += sizeof(buffer);
    }

    for (img_id = IMG_USER_DATA_FIRST; img_id < IMG_USER_DATA_MAX; img_id++)
    {
        dfu_get_user_data_info(img_id, &size, false);

        if (size == 0 || size == 0xffffffff)
        {
            continue;
        }

        dfu_get_user_data_info(img_id, &addr, true);
        fmc_flash_nor_read(addr + offsetof(T_IMG_HEADER_FORMAT, git_ver), &image_version,
                           sizeof(T_IMAGE_VERSION));

        *(uint16_t *)buffer = img_id;
        *(uint32_t *)&buffer[2] = image_version.ver_info.version;

        num ++;
        memcpy(p_temp, buffer, sizeof(buffer));
        p_temp += sizeof(buffer);
    }

    *p_data = num;
    LOG_INF("<===ble_dfu_get_img_version, bank_num:%x img_num:%x", bank, num);
}

/**
    * @brief    Used to get image section size
    * @param    p_data  point of the section size
    * @return   void
    */
void ble_dfu_get_section_size(uint8_t *p_data)
{
    uint8_t num = 0;
    IMG_ID img_id;
    uint8_t buffer[6] = {0};
    uint8_t *p_temp = NULL;
    uint32_t size;
    //cppcheck-suppress ctuPointerArith
    p_temp = p_data + 1;

    for (img_id = IMG_DFU_FIRST; img_id < IMG_DFU_MAX; img_id++)
    {
        size = get_active_bank_image_size_by_img_id(img_id);

        if (size == 0 || size == 0xffffffff)
        {
            continue;
        }

        *(uint16_t *)buffer = img_id;
        *(uint32_t *)&buffer[2] = size;

        num ++;
        memcpy(p_temp, buffer, sizeof(buffer));
        p_temp += sizeof(buffer);
    }

    for (img_id = IMG_USER_DATA_FIRST; img_id < IMG_USER_DATA_MAX; img_id++)
    {
        dfu_get_user_data_info(img_id, &size, false);

        if (size == 0 || size == 0xffffffff)
        {
            continue;
        }

        *(uint16_t *)buffer = img_id;
        *(uint32_t *)&buffer[2] = size;

        num ++;
        memcpy(p_temp, buffer, sizeof(buffer));
        p_temp += sizeof(buffer);
    }

    *p_data = num;

    LOG_INF("<===ble_dfu_get_section_size, img_num:%x in active bank", num);
}

void dfu_notify_conn_para_update_req(struct bt_conn *conn, uint8_t error_code)
{
    if (ota_struct.dfu_conn_para_update_in_progress == true)
    {
        uint8_t notif_data[DFU_NOTIFY_LEN_CONN_PARA_UPDATE_REQ] = {0};
        ota_struct.dfu_conn_para_update_in_progress = false;

        notif_data[0] = error_code;
        LOG_INF("<==dfu_notify_conn_para_update_req error_code=0x%x", error_code);
        if (error_code == DFU_ARV_SUCCESS)
        {
            uint16_t conn_interval;
            uint16_t conn_slave_latency;
            uint16_t conn_supervision_timeout;
            struct bt_conn_info info;
            int err;

            err = bt_conn_get_info(conn, &info);
            if (err != 0) {
                LOG_INF("Failed to get conn info: %d", err);
                return;
            }

            conn_interval = info.le.interval;
            conn_slave_latency = info.le.latency;
            conn_supervision_timeout = info.le.timeout;

            LE_UINT16_TO_ARRAY(notif_data + 1, conn_interval);
            LE_UINT16_TO_ARRAY(notif_data + 3, conn_slave_latency);
            LE_UINT16_TO_ARRAY(notif_data + 5, conn_supervision_timeout);
        }
        ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_CONN_PARA_UPDATE_REQ,
                                            DFU_NOTIFY_LEN_CONN_PARA_UPDATE_REQ,
                                            notif_data);
    }
}


/**
    * @brief    Handle written request on DFU control point characteristic
    * @param    conn     ID to identify the connection
    * @param    length      Length of value to be written
    * @param    p_value     Value to be written
    * @return   T_APP_RESULT
    * @retval   Handle result of this request
    */
T_APP_RESULT ble_dfu_service_handle_cp_req(struct bt_conn *conn, uint16_t length, uint8_t *p_value)
{
    T_APP_RESULT cause = APP_RESULT_INVALID_PDU;
    uint8_t results = DFU_ARV_SUCCESS;
    uint8_t opcode = *p_value;
    uint8_t *p = p_value + 1;
    bool error_flag = false;

    T_DFU_CALLBACK_DATA callback_data;
    //callback_data.conn_id = conn->id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    callback_data.msg_data.write.write_attrib_index = INDEX_DFU_CHAR_DFU_CONTROL_POINT_INDEX;

    printk("===>ble_dfu_service_handle_cp_req: opcode=0x%x, length=%d\n", opcode, length);

    if (opcode > DFU_OPCODE_MIN && opcode <= DFU_OPCODE_TEST_EN
        && ota_struct.ota_flag.is_ota_process)
    {
       // os_timer_create(&ota_struct.timer_handle_ota_transfer, "ota_dimage_transfer",
          //              TIMER_ID_DFU_IMAGE_TRANS, 30 * 1000, false, ble_dfu_timeout_cb);
        //printk("&ota_struct.timer_handle_ota_transfer:%x",&ota_struct.timer_handle_ota_transfer);
        //os_timer_start(&ota_struct.timer_handle_ota_transfer);
    }

    switch (opcode)
    {
    case DFU_OPCODE_START_DFU:
        {
            if (length == DFU_LENGTH_CP_START_DFU)   /* 4 bytes is pending for encrypt */
            {
                LOG_HEXDUMP_INF(p_value, length, "===>ble_dfu_service_handle_cp_req: start dfu data:");
                cause = APP_RESULT_SUCCESS;
                results = ble_dfu_cp_start_dfu_handle(p);
                
                uint32_t callback_data_value[2];
                callback_data_value[0] = ota_struct.image_id;
                callback_data_value[1] = ota_struct.image_total_length;

                callback_data.msg_data.write.opcode = DFU_WRITE_START;
                callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
                callback_data.msg_data.write.p_value = (uint8_t *)&callback_data_value;

                cause = app_dfu_srv_cb((void *)&callback_data);
               
                ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_START_DFU, sizeof(results), &results);

                LOG_INF("DFU_OPCODE_START_DFU done!");
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_RECEIVE_FW_IMAGE_INFO:
        {
            if (length == DFU_LENGTH_CP_RECEIVE_FW_IMAGE_INFO)
            {
                uint16_t image_id;

                LE_ARRAY_TO_UINT16(image_id, p);
                cause = APP_RESULT_SUCCESS;
                if (image_id == ota_struct.image_id)
                {
                    LE_ARRAY_TO_UINT32(ota_struct.cur_offset, p + 2);
                    LOG_INF("===>ble_dfu_service_handle_cp_req: image_id=0x%x, cur_offset=%d",
                                     image_id, ota_struct.cur_offset);
                }
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_VALID_FW:
        {
            if (length == DFU_LENGTH_CP_VALID_FW)
            {
                cause = APP_RESULT_SUCCESS;
                results = ble_dfu_cp_valid_handle(p);
                ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_VALID_FW, sizeof(results), &results);
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_ACTIVE_IMAGE_RESET:
        {
            printk("in DFU_OPCODE_ACTIVE_IMAGE_RESET\n");
            if (length == DFU_LENGTH_CP_ACTIVE_IMAGE_RESET)
            {
                bool is_enter_dfu_mode = *p;
                cause = APP_RESULT_SUCCESS;
                ble_dfu_cp_active_reset_handle(conn, is_enter_dfu_mode);
                ota_struct.ota_flag.is_ota_process = false;
                os_timer_stop(&ota_struct.timer_handle_ota_transfer);
                
                callback_data.msg_data.write.opcode = DFU_WRITE_END;
                cause = app_dfu_srv_cb((void *)&callback_data);
                printk("done DFU_OPCODE_ACTIVE_IMAGE_RESET?\n");
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_SYSTEM_RESET:
        {
            cause = APP_RESULT_SUCCESS;
            ble_dfu_clear_local(OTA_RESET_CMD);

            T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_SYSTEM_RESET_CMD;
            callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
            callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;
            callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;

            cause = app_dfu_srv_cb((void *)&callback_data);

            bt_conn_disconnect(conn,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
        break;
    case DFU_OPCODE_REPORT_TARGET_INFO:
        {
            if (length == DFU_LENGTH_CP_REPORT_TARGET_INFO)
            {
                uint8_t notif_data[DFU_NOTIFY_LEN_TARGET_INFO] = {0};

                cause = APP_RESULT_SUCCESS;
                ble_dfu_cp_report_img_info_handle(p, notif_data);
                ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_REPORT_TARGET_INFO,
                                                    DFU_NOTIFY_LEN_TARGET_INFO,
                                                    notif_data);
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_CONN_PARA_UPDATE_REQ:
        {
            if (length == DFU_LENGTH_CP_CONN_PARA_UPDATE_REQ)
            {
                uint8_t notif_data[DFU_NOTIFY_LEN_CONN_PARA_UPDATE_REQ] = {0};

                cause = APP_RESULT_SUCCESS;
                ble_dfu_cp_conn_para_update_handle(conn, p, notif_data);
                if (notif_data[0] != DFU_ARV_SUCCESS)
                {
                    ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_CONN_PARA_UPDATE_REQ,
                                                        DFU_NOTIFY_LEN_CONN_PARA_UPDATE_REQ,
                                                        notif_data);
                }
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_BUFFER_CHECK_EN:
        {
            if (length  == DFU_LENGTH_CP_BUFFER_CHECK_EN)
            {
                uint8_t notif_data[DFU_NOTIFY_LEN_BUFFER_CHECK_EN] = {0};

                cause = APP_RESULT_SUCCESS;
                ble_dfu_cp_buffer_check_en_handle(notif_data, MAX_BUFFER_SIZE);
                ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_BUFFER_CHECK_EN,
                                                    DFU_NOTIFY_LEN_BUFFER_CHECK_EN,
                                                    notif_data);
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_REPORT_BUFFER_CRC:
        {
            if (length  == DFU_LENGTH_CP_REPORT_BUFFER_CRC)
            {
                uint8_t notif_data[DFU_NOTIFY_LEN_BUFFER_CHECK] = {0};

                cause = APP_RESULT_SUCCESS;
                ble_dfu_cp_buffer_check_handle(p, notif_data);
                ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_REPORT_BUFFER_CRC,
                                                    DFU_NOTIFY_LEN_BUFFER_CHECK,
                                                    notif_data);
            }
            else
            {
                error_flag = true;
            }
        }
        break;
    case DFU_OPCODE_COPY_IMG:
        {
            cause = APP_RESULT_SUCCESS;
            results = ble_dfu_cp_copy_img_handle(p);
            ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_COPY_IMG, sizeof(results), &results);
        }
        break;
    case DFU_OPCODE_GET_IMAGE_VER:
        {
            uint8_t notif_data[DFU_NOTIFY_LEN_IMG_INFO] = {0};

            cause = APP_RESULT_SUCCESS;
            notif_data[0] = *p; //bank_num, 0: active, 1: inactive
            ble_dfu_get_img_version(&notif_data[1], notif_data[0]);
            ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_GET_IMAGE_VER,
                                                2 + 6 * notif_data[1]/*DFU_NOTIFY_LEN_IMG_INFO*/, notif_data);
        }
        break;
    case DFU_OPCODE_GET_SECTION_SIZE:
        {
            uint8_t notif_data[DFU_NOTIFY_LEN_SECTION_SIZE] = {0};

            cause = APP_RESULT_SUCCESS;
            ble_dfu_get_section_size(notif_data);
            ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_GET_SECTION_SIZE,
                                                1 + 6 * notif_data[0]/*DFU_NOTIFY_LEN_SECTION_SIZE*/, notif_data);
        }
        break;
    case DFU_OPCODE_CHECK_SHA256:
        {
            uint8_t notif_data[DFU_NOTIFY_LEN_CHECK_SHA256] = {0};
            uint16_t num = *(uint16_t *)p;
            LOG_INF("ble_dfu_service_handle_cp_req: CHECK SHA2556 num=%d", num);

            cause = APP_RESULT_SUCCESS;
            ble_dfu_cp_check_sha256_handle(p, notif_data);
            ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_CHECK_SHA256, DFU_NOTIFY_LEN_CHECK_SHA256,
                                                notif_data); //check grace, send all img sha256 or not?
        }
        break;
    case DFU_OPCODE_GET_RELEASE_VER:
        {
            uint8_t notif_data[DFU_NOTIFY_LEN_RELEASE_VER] = {0};
            uint32_t p_imgheader = get_header_addr_by_img_id(IMG_MCUAPP);

            cause = APP_RESULT_SUCCESS;
            memcpy(notif_data, (uint8_t *)(p_imgheader + RELEASE_VER_OFFSET), DFU_NOTIFY_LEN_RELEASE_VER);
            ble_dfu_service_prepare_send_notify(conn, DFU_OPCODE_GET_RELEASE_VER, DFU_NOTIFY_LEN_RELEASE_VER,
                                                notif_data);
        }
        break;
    case DFU_OPCODE_TEST_EN:
        {
            cause = APP_RESULT_SUCCESS;
            memcpy(&ota_struct.test.value, p, sizeof(ota_struct.test.value));
        }
        break;
    case DFU_OPCODE_REPORT_IMAGE_NUM:
        {
            uint16_t image_id;
            uint8_t cur_num;
            uint8_t total_num;

            LE_ARRAY_TO_UINT16(image_id, p);
            cur_num = *(p + 2);
            total_num = *(p + 3);
            LOG_INF("ble_dfu_service_handle_cp_req: REPORT IMAGE NUM image_id = 0x%x, cur_num = %d, total_num = %d",
                             image_id, cur_num, total_num);

            cause = APP_RESULT_SUCCESS;
        }
        break;
    default:
        LOG_ERR("ble_dfu_service_handle_cp_req, opcode %d not expected", opcode);
        break;
    }

    if (error_flag)
    {
        LOG_ERR("ble_dfu_service_handle_cp_req: invalid length");
    }

    return cause;
}

/**
    * @brief    Handle written request on DFU packet characteristic
    * @param    conn     ID to identify the connection
    * @param    length      Length of value to be written
    * @param    p_value     Value to be written
    * @return   T_APP_RESULT
    * @retval   Handle result of this request
    */
T_APP_RESULT ble_dfu_service_handle_packet(struct bt_conn *conn, uint16_t length, uint8_t *p_value)
{
    uint8_t result;

    result = ble_dfu_packet_handle(p_value, length);

    if (result == DFU_ARV_FAIL_INVALID_PARAMETER)
    {
        return APP_RESULT_INVALID_PDU;
    }
    else if (result == DFU_ARV_FAIL_OPERATION)
    {
        return APP_RESULT_APP_ERR;
    }

    return APP_RESULT_SUCCESS;
}

