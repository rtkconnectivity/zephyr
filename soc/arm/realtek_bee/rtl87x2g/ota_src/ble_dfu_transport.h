/**
*****************************************************************************************
*     Copyright(c) 2023, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      ble_dfu_transport.h
   * @brief     Header file for using ble dfu transport
   * @author    Grace
   * @date      2023-12-06
   * @version   v1.1
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2023 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                      Define to prevent recursive inclusion
 *============================================================================*/
#ifndef _BLE_DFU_TRANSPORT_H_
#define _BLE_DFU_TRANSPORT_H_

#include "patch_header_check.h"

#include <zephyr/bluetooth/bluetooth.h>

#include <rtk_bt_types.h>

extern bool dfu_switch_to_ota_mode_pending;
extern bool dfu_active_reset_pending;
extern bool dfu_active_reset_to_ota_mode;

/** @defgroup  BLE_DFU_TRANSPORT BLE DFU Transport
  * @brief
  * @{
  */

/*============================================================================*
 *                              Macros
 *============================================================================*/
/** @defgroup BLE_DFU_TRANSPORT_Exported_Macros BLE DFU Transport Exported Macros
  * @brief
  * @{
  */
#define MEMBER_OFFSET(struct_type, member)      ((uint32_t)&((struct_type *)0)->member)

#define SPP_PROTOCOL_INFO      0x0011
#define BLE_PROTOCOL_INFO      0x0015

/*ota version num*/
#define BLE_OTA_VERSION     0x1
#define SPP_OTA_VERSION     0x7


/** @brief image exit indicator. */
#define NOT_SUPPORT_BANK_SWITCH     0
#define IMAGE_LOCATION_BANK0        1
#define IMAGE_LOCATION_BANK1        2


/*bit set of device info data2*/
#define OTA_DEVICE_FEATURE_SUPPORT_BUFFER_CHECK     (1 << 0)
#define OTA_DEVICE_FEATURE_ENABLE_AES_ENCRYPT       (1 << 1)
#define OTA_DEVICE_FEATURE_ENCRYPT_MODE_16N         (1 << 2)
#define OTA_DEVICE_FEATURE_SUPPORT_MULTIIMAGE       (1 << 3)

/** @brief  index for DFU implementation*/
#define DFU_ARV_SUCCESS                         0x01
#define DFU_ARV_FAIL_INVALID_PARAMETER          0x02
#define DFU_ARV_FAIL_OPERATION                  0x03
#define DFU_ARV_FAIL_DATA_SIZE_EXCEEDS_LIMIT    0x04
#define DFU_ARV_FAIL_CRC_ERROR                  0x05
#define DFU_ARV_FAIL_DATA_LENGTH_ERROR          0x06
#define DFU_ARV_FAIL_FLASH_WRITE_ERROR          0x07
#define DFU_ARV_FAIL_FLASH_ERASE_ERROR          0x08

/** @brief  opcode code for DFU service*/
#define DFU_OPCODE_MIN                      0x00
#define DFU_OPCODE_START_DFU                0x01
#define DFU_OPCODE_RECEIVE_FW_IMAGE_INFO    0x02
#define DFU_OPCODE_VALID_FW                 0x03
#define DFU_OPCODE_ACTIVE_IMAGE_RESET       0x04
#define DFU_OPCODE_SYSTEM_RESET             0x05
#define DFU_OPCODE_REPORT_TARGET_INFO       0x06
#define DFU_OPCODE_CONN_PARA_UPDATE_REQ     0x07
#define DFU_OPCODE_BUFFER_CHECK_EN          0x09
#define DFU_OPCODE_REPORT_BUFFER_CRC        0x0a
#define DFU_OPCODE_COPY_IMG                 0x0c
#define DFU_OPCODE_GET_IMAGE_VER            0x0d
#define DFU_OPCODE_GET_SECTION_SIZE         0x0e
#define DFU_OPCODE_CHECK_SHA256             0x0f
#define DFU_OPCODE_GET_RELEASE_VER          0x10
#define DFU_OPCODE_TEST_EN                  0x11
#define DFU_OPCODE_REPORT_IMAGE_NUM         0x12
#define DFU_OPCODE_NOTIF                    0x10

/** @brief  length of each control point procedure*/
#define DFU_LENGTH_CP_START_DFU                (1+12+4) //12bytes ctrl header + 4bytes padding for encrypt
#define DFU_LENGTH_CP_RECEIVE_FW_IMAGE_INFO    (1+2+4)
#define DFU_LENGTH_CP_VALID_FW                 (1+2+1)
#define DFU_LENGTH_CP_ACTIVE_IMAGE_RESET       (1+1)  //is_enter_dfu_mode
#define DFU_LENGTH_CP_SYSTEM_RESET             0x01
#define DFU_LENGTH_CP_REPORT_TARGET_INFO       (1+2)
#define DFU_LENGTH_CP_CONN_PARA_UPDATE_REQ     (1+2+2+2+2) //conn_interval_min,conn_interval_max,conn_latency,superv_tout
#define DFU_LENGTH_CP_PKT_RX_NOTIF_REQ         (1+2)
#define DFU_LENGTH_CP_CONN_PARA_TO_UPDATE_REQ  (1+2+2+2+2)
#define DFU_LENGTH_CP_BUFFER_CHECK_EN          0x01
#define DFU_LENGTH_CP_REPORT_BUFFER_CRC        (1+2+2)

#define MAX_IMG_NUM                         (IMG_DFU_MAX - IMG_DFU_FIRST + IMG_USER_DATA_MAX - IMG_USER_DATA_FIRST) //0x16 + 8

/** @brief  length of each control point notification*/
#define DFU_NOTIFY_LEN_TARGET_INFO          (1+4+4+2) //error_code, rsvd, cur_offset, buffer_check_offset
#define DFU_NOTIFY_LEN_CONN_PARA_UPDATE_REQ (1+2+2+2) //connInterval, latency, supervisionTimeout
#define DFU_NOTIFY_LEN_BUFFER_CHECK_EN      (1+2+2)   //error_code, bufcheck_en, max_buf_size, rsvd
#define DFU_NOTIFY_LEN_BUFFER_CHECK         (1+4)     //error_code, cur_offset
#define DFU_NOTIFY_LEN_CHECK_SHA256         (2+1)     //img_id, check_ret(0:fail, 1: same as inactive, 2: same as active, 3: same as inactive and active)
#define DFU_NOTIFY_LEN_RELEASE_VER          6         //release version of app data bin
#define DFU_NOTIFY_LEN_IMG_INFO             (1+1+(2+4)*MAX_IMG_NUM)  //bank_num, img_num, img_id1, img_ver1, ....
#define DFU_NOTIFY_LEN_SECTION_SIZE         (1+(2+4)*MAX_IMG_NUM)   //img_num, img_id1, img_size1, ....

#define IMG_INFO_LEN                        DFU_NOTIFY_LEN_IMG_INFO
#define SECTION_SIZE_LEN                    DFU_NOTIFY_LEN_SECTION_SIZE


#define UINT_4K                 4096
#define MAX_BUFFER_SIZE         4096

#define SHA256_OFFSET           (MEMBER_OFFSET(T_AUTH_HEADER_FORMAT, image_hash)) //372
#define SHA256_LEN              32
#define RELEASE_VER_OFFSET      (SHA256_OFFSET + SHA256_LEN)


#define BTAON_FAST_AON_GPR_15   0x1a2

/** End of BLE_DFU_TRANSPORT_Exported_Macros
  * @}
  */


/*============================================================================*
 *                              Types
 *============================================================================*/
/** @defgroup BLE_DFU_TRANSPORT_Exported_Types BLE DFU Transport Exported Types
  * @brief
  * @{
  */
typedef enum
{
    OTA_SUCCESS_REBOOT,
    OTA_BLE_DISC,
    OTA_IMAGE_TRANS_TIMEOUT,
    OTA_IMAGE_TOTAL_TIMEOUT,
    OTA_RESET_CMD,
} T_OTA_CLEAR_LOCAL_CAUSE;

typedef enum
{
    ACTIVE_BANK   = 0,
    INACTIVE_BANK = 1,
} T_OTA_BANK;

typedef enum
{
    TIMER_ID_DFU_IMAGE_TRANS,
} T_OTA_TIMER_ID;

typedef union
{
    uint8_t value;
    struct
    {
        uint8_t buffer_check_en: 1;
        uint8_t is_ota_process: 1;
        uint8_t is_support_mutil_image: 1;
        uint8_t is_devinfo: 1;
        uint8_t skip_flag: 1;
        uint8_t ota_mode: 1;
        uint8_t RSVD: 2;
    };
} OTA_FLAG;

typedef union
{
    uint8_t value;
    struct
    {
        uint8_t t_aes_en: 1;
        uint8_t t_stress_test: 1;
        uint8_t t_copy_fail: 1;
        uint8_t t_skip_fail: 1;
        uint8_t t_buffercheck_disable: 1;
        uint8_t rsvd: 3;
    };
} TEST_ENABLE;

typedef struct
{
    uint32_t image_offset;
} T_TEMP_IMAGE_INFO;

/** @brief  Table used to store Extended Device Information */
typedef struct
{
    void *timer_handle_ota_transfer;
    uint8_t *p_ota_temp_buf_head;
    uint32_t image_indicator;
    uint32_t image_total_length;
    uint32_t cur_offset;
    uint32_t buffer_check_offset;
    uint32_t next_subimage_offset;
    uint32_t tmp_next_subimage_offset;
    uint16_t image_id;
    uint16_t ota_temp_buf_used_size;
    uint16_t buffer_size;
    uint16_t mtu_size;
    T_TEMP_IMAGE_INFO temp_image_info[IMG_DFU_MAX - IMG_BANK_FIRST];
    uint8_t force_temp_mode;
    uint8_t bd_addr[6];
    uint8_t bp_level;
    bool dfu_conn_para_update_in_progress;
    bool is_last_image;
    OTA_FLAG ota_flag;
    TEST_ENABLE test;
} OTA_FUNCTION_STRUCT;

typedef struct
{
    IMG_ID img_id;
    uint8_t sha256[32];
} SHA256_CHECK;

typedef struct
{
    uint16_t img_id;
    uint8_t data[4];
} IMG_INFO;

typedef union
{
    uint8_t value;
    struct
    {
        uint8_t buffercheck_en: 1;
        uint8_t aes_en: 1;
        uint8_t aes_mode: 1; /*1:16*N bytes, 0:first 16byte*/
        uint8_t support_multiimage: 1;
        uint8_t is_support_normal_ota: 1; //valid only when spec_ver>=4
        uint8_t is_ota_support_dual_bank_merge: 1;
        uint8_t rsvd: 2;
    };
} DEVICE_INFO_MODE;


typedef struct
{
    uint8_t ic_type;
    uint8_t spec_ver;
    DEVICE_INFO_MODE mode;
    uint8_t rsvd0;
    uint16_t ota_temp_size;  //for watch
    uint8_t active_banknum; //0: not support dual bank
    uint8_t bootpatch_active_banknum; //0: not support dual bank
    uint8_t secureapp_active_banknum; //0: not support dual bank
    uint16_t ctrl_header_offset;
    uint8_t rsvd1;
} __attribute__((packed)) DEVICE_INFO;


typedef union
{
    uint32_t value;
    struct
    {
        uint32_t enc_en: 1;
        uint32_t sha256_en: 1;
        uint32_t link_loss_stop: 1;
        uint32_t rsvd: 28;
    };
} OTA_SETTING;

/** End of BLE_DFU_TRANSPORT_Exported_Types
  * @}
  */

/*============================================================================*
 *                              Functions
 *============================================================================*/
/** @defgroup BLE_DFU_TRANSPORT_Exported_Functions BLE DFU Transport Functions
  * @brief
  * @{
  */
/**
    * @brief        Used to get device information
    * @param        conn_id       connection id
    * @param        p_deviceinfo  point of the device info
    * @return       void
    */
void ble_dfu_get_device_info(struct bt_conn *conn, DEVICE_INFO *p_deviceinfo);

/**
    * @brief    Used to get image version
    * @param    *p_data   point of image version
    * @param    bank   0:active bank    1:inactive bank
    * @return   void
    */
void ble_dfu_get_img_version(uint8_t *p_data, uint8_t bank);

/**
    * @brief    Used to get image section size
    * @param    p_data  point of the section size
    * @return   void
    */
void ble_dfu_get_section_size(uint8_t *p_data);

/**
    * @brief    dfu notify connection parameter update request
    * @param    conn_id     ID to identify the connection
    * @param    error_code  error code of conn para update req
    * @return   void
    */
void dfu_notify_conn_para_update_req(struct bt_conn *conn, uint8_t error_code);

/**
    * @brief    Handle written request on DFU packet characteristic
    * @param    conn_id     ID to identify the connection
    * @param    length      Length of value to be written
    * @param    p_value     Value to be written
    * @return   T_APP_RESULT
    * @retval   Handle result of this request
    */
T_APP_RESULT ble_dfu_service_handle_packet(struct bt_conn *conn, uint16_t length, uint8_t *p_value);

/**
    * @brief    Handle written request on DFU control point characteristic
    * @param    conn_id     ID to identify the connection
    * @param    length      Length of value to be written
    * @param    p_value     Value to be written
    * @return   T_APP_RESULT
    * @retval   Handle result of this request
    */
T_APP_RESULT ble_dfu_service_handle_cp_req(struct bt_conn *conn, uint16_t length, uint8_t *p_value);


/** End of BLE_DFU_TRANSPORT_Exported_Functions
  * @}
  */

/** End of BLE_DFU_TRANSPORT
  * @}
  */

#endif
