/**
*****************************************************************************************
*     Copyright(c) 2023, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      dfu_common.h
   * @brief     Header file for using dfu common
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
#ifndef _DFU_COMMON_H_
#define _DFU_COMMON_H_

#include "patch_header_check.h"
#include "rtl876x.h"
#include "wdt.h"

/** @defgroup  DFU_COMMON DFU Common
  * @brief
  * @{
  */
/*============================================================================*
 *                              Macros
 *============================================================================*/
/** @defgroup DFU_COMMON_Exported_Macros DFU Common Exported Macros
  * @brief
  * @{
  */
#define FLASH_SECTOR_SIZE                          0x1000


#define PLATFORM_STATIC_ASSERT(condition, identifier) typedef char PALStaticAssert_##identifier[(condition) ? 1 : -1]

#define SET_VALID_BITMAP(image_id)      (valid_bitmap |= BIT(image_id - IMG_DFU_FIRST))
#define GET_VALID_BITMAP(image_id)      (valid_bitmap >> (image_id - IMG_DFU_FIRST) & BIT0)

#define SET_USER_DATA_VALID_BITMAP(image_id)      (user_data_valid_bitmap |= BIT(image_id - IMG_USER_DATA_FIRST))
#define GET_USER_DATA_VALID_BITMAP(image_id)      (user_data_valid_bitmap >> (image_id - IMG_USER_DATA_FIRST) & BIT0)


/** End of DFU_COMMON_Exported_Macros
  * @}
  */


/*============================================================================*
 *                              Types
 *============================================================================*/
/** @defgroup DFU_COMMON_Exported_Types DFU Common Exported Types
  * @brief
  * @{
  */
typedef enum
{
    OTA_BANK0,
    OTA_BANK1,
    OTA_BANK_MAX,
} T_ACTIVE_BANK_NUM;

typedef enum
{
    USER_DATA_SUCCESS = 0,
    USER_DATA_NOT_SUPPORT_OTA,
    USER_DATA_TYPE_ERROR,
} T_USER_DATA_ERROR_TYPE;

/** End of DFU_COMMON_Exported_Types
  * @}
  */
extern uint32_t valid_bitmap;
extern uint32_t user_data_valid_bitmap;
/*============================================================================*
 *                              Functions
 *============================================================================*/
/** @defgroup DFU_COMMON_Exported_Functions DFU Common Exported Functions
  * @brief
  * @{
  */
/**
 * @brief  AES256 decrypt
 * @param  input pointer to input data
 * @return None
 */
void dfu_aes256_decrypt_16byte(uint8_t *input);

/**
 * @brief  Get image size of bank area
 * @param  image_id  image ID
 * @return bank size
 */
uint32_t get_bank_size_by_img_id(IMG_ID image_id);

/**
 * @brief  Chip reset for dfu
 * @param  reset_mode   reset mode
 * @param  reason       reset reason
 * @return None
 */
void dfu_fw_reboot(WDTMode_TypeDef reset_mode, T_SW_RESET_REASON reason);

/**
 * @brief  Check ota mode flag, if image need update
 * @param  None
 * @return Result: true: image need update, false: image don't need update
 */
bool dfu_check_ota_mode_flag(void);

/**
 * @brief  Set ota mode flag
 * @param  enable  ota mode flag
 * @return None
 */
void dfu_set_ota_mode_flag(bool enable);

/**
 * @brief  Switch to the OTA mode, if support normal ota app need call it.
 * @param  None
 * @return None
 */
void dfu_switch_to_ota_mode(void);

/**
    * @brief    get the ic type of current fw
    * @param    void
    * @return   ic type
    */
uint8_t dfu_get_ic_type(void);

/**
    * @brief    get active bank num
    * @param
    * @return   bank num value
    */
T_ACTIVE_BANK_NUM get_active_bank_num(void);

/**
    * @brief    get inactive bank's image address
    * @param    image_id   image id
    * @return   image address
    */
uint32_t dfu_get_temp_ota_bank_img_addr_by_img_id(IMG_ID image_id);

/**
    * @brief    get inactive bank's image size
    * @param    image_id   image id
    * @return   image size
    */
uint32_t dfu_get_temp_ota_bank_img_size_by_img_id(IMG_ID image_id);

/**
* @brief calculate checksum of lenth of buffer in flash.
*
* @param  signature          signature to identify FW.
* @param  offset             offset of the image.
* @param  length             length of data.
* @param  crcValue          ret crc value point.
* @return  0 if buffer checksum calcs successfully, error line number otherwise
*/

uint32_t dfu_check_bufcrc(uint8_t *buf, uint32_t length, uint16_t mCrcVal);

/**
 * @brief erase a sector of the flash.
 *
 * @param  signature          signature to identify FW.
 * @param  offset             offset of the image.
 * @return  0 if erase successfully, error line number otherwise
*/
uint32_t dfu_flash_erase(IMG_ID image_id, uint32_t offset);

/**
    * @brief    write data to flash
    * @param    img_id  image id
    * @param    offset  image offset
    * @param    total_offset  total offset when ota temp mode
    * @param    p_void  point of data
    * @return   0: success; other: fail
    */
uint32_t dfu_write_data_to_flash(uint16_t img_id, uint32_t offset, uint32_t total_offset,
                                 uint32_t length, void *p_void);

/**
    * @brief    check the integrity of the image
    * @param    image_id    image id
    * @param    offset  address offset
    * @return   ture:success ; false: fail
    */
bool dfu_checksum(IMG_ID image_id, uint32_t offset);

/**
    * @brief    clear not ready flag of specific image
    * @param    p_header  point of p_header
    * @return   void
    */
void dfu_set_ready(T_IMG_HEADER_FORMAT *p_header);

/**
    * @brief    clear not obsolete flag of specific image
    * @param    p_header  point of p_header
    * @return   void
    */
void dfu_set_obsolete(T_IMG_HEADER_FORMAT *p_header);

/**
    * @brief    get user data infomation
    * @param    image_id  image id
    * @param    img_info  point of img_info
    * @param    is_addr  addr
    * @return   A T_USER_DATA_ERROR_TYPE type value
    */
T_USER_DATA_ERROR_TYPE dfu_get_user_data_info(IMG_ID image_id,
                                              uint32_t *img_info, bool is_addr);

/** End of DFU_COMMON_Exported_Functions
  * @}
  */

/** End of DFU_COMMON
  * @}
  */

#endif
