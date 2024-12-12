/**
**********************************************************************************************************
*               Copyright(c) 2023, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     dfu_common.c
* @brief    supply dfu common APIs to application
* @details
* @author   Grace
* @date     2023-12-06
* @version  v1.1
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2023 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/

#include <string.h>
#include <stddef.h>
#include "aes_api.h"
#include "ble_dfu_transport.h"
#include "crypto_engine_nsc.h"
#include "dfu_common.h"
#include "fmc_api.h"
#include "os_sync.h"
#include "rtl876x.h"
#include "aon_reg.h"
#include "rtl_wdt.h"
#include "trace.h"
#include "utils.h"
#include "flash_map.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dfu_common);



/*============================================================================*
 *                              Macros
 *============================================================================*/
#define SHA256_LENGTH                   32
#define SHA256_BUFFER_SIZE              128
#define READ_BACK_BUFFER_SIZE           64

/*============================================================================*
 *                              Variables
 *============================================================================*/
/*max img num = 32, BIT0: OTA, BIT1: SecureBoot, ..., */
uint32_t valid_bitmap = 0;
PLATFORM_STATIC_ASSERT((IMG_DFU_MAX - IMG_DFU_FIRST + 1) < (sizeof(valid_bitmap) << 3),
                       valid_bitmap);

uint32_t user_data_valid_bitmap = 0;

/*============================================================================*
 *                              Private Functions
 *============================================================================*/
/**
 * @brief  get 16bit data swapped.
 *
 * @param  val          16bit data to be swapped.
 * @return value after being swapped.
*/
static uint16_t swap_16(uint16_t val)
{
    uint16_t result;

    /* Idiom Recognition for REV16 */
    result = ((val & 0xff) << 8) | ((val & 0xff00) >> 8);

    return result;
}

/**
 * @brief  Get aes256 key swapped
 * @param  val  aes256 key to be swapped
 * @return None
 */
static void aes256_key_swap(uint8_t key[32])
{
    uint32_t tmp[4];

    memcpy(tmp, key, 16);
    memcpy(key, key + 16, 16);
    memcpy(key + 16, tmp, 16);
}

/**
 * @brief      Check image sha256
 * @param[in]  p_header   pointer to dfu check image header
 * @return     Check result
 * @retval     true check pass
 * @retval     false check fail
 */
static bool dfu_check_sha256(T_IMG_HEADER_FORMAT *p_header)
{
    uint32_t sha256sum[SHA256_LENGTH / 4] = {0};
    uint8_t sha256img[SHA256_LENGTH] = {0};
    uint8_t buf[SHA256_BUFFER_SIZE] = {0};
    uint32_t len;
    uint16_t i;
    uint16_t loop_cnt, remain_size;
    uint32_t pdata = (uint32_t)&p_header->ctrl_header;
    uint32_t s;

    s = os_lock();
    fmc_flash_nor_read((uint32_t)(pdata + MEMBER_OFFSET(T_IMG_CTRL_HEADER_FORMAT, payload_len)),
                       &len, 4);
    os_unlock(s);
    len += sizeof(T_IMG_HEADER_FORMAT) - sizeof(T_AUTH_HEADER_FORMAT);

    loop_cnt = len / SHA256_BUFFER_SIZE;
    remain_size = len % SHA256_BUFFER_SIZE;

    HW_SHA256_CTX ctx = {0};

    hw_sha256_init();
    hw_sha256_start(&ctx, NULL);

    for (i = 0; i < loop_cnt; ++i)
    {
        s = os_lock();
        fmc_flash_nor_read(pdata, buf, SHA256_BUFFER_SIZE);
        os_unlock(s);
        if (i == 0)
        {
            //because not_ready is included when calculate image sha256
            ((T_IMG_CTRL_HEADER_FORMAT *)buf)->ctrl_flag.not_ready = 0;
        }

        hw_sha256_cpu_update(&ctx, buf, SHA256_BUFFER_SIZE);
        pdata += SHA256_BUFFER_SIZE;
    }
    if (remain_size)
    {
        s = os_lock();
        fmc_flash_nor_read(pdata, buf, remain_size);
        os_unlock(s);
        if (loop_cnt == 0)
        {
            ((T_IMG_CTRL_HEADER_FORMAT *)buf)->ctrl_flag.not_ready = 0;
        }

        hw_sha256_cpu_update(&ctx, buf, remain_size);
    }

    hw_sha256_finish(&ctx, sha256sum);
    s = os_lock();
    fmc_flash_nor_read((uint32_t)&p_header->auth.image_hash, sha256img, SHA256_LENGTH);
    os_unlock(s);

    return (memcmp(sha256img, sha256sum, SHA256_LENGTH) == 0);
}

/*============================================================================*
 *                              Public Functions
 *============================================================================*/
/**
 * @brief  AES256 decrypt
 * @param  input pointer to input data
 * @return None
 */
void dfu_aes256_decrypt_16byte(uint8_t *input)
{
    static const uint8_t SECRET_KEY_DEFAULT[32] =
    {
        0x4E, 0x46, 0xF8, 0xC5, 0x09, 0x2B, 0x29, 0xE2,
        0x9A, 0x97, 0x1A, 0x0C, 0xD1, 0xF6, 0x10, 0xFB,
        0x1F, 0x67, 0x63, 0xDF, 0x80, 0x7A, 0x7E, 0x70,
        0x96, 0x0D, 0x4C, 0xD3, 0x11, 0x8E, 0x60, 0x1A
    };
    uint8_t key_tmp[32];

    memcpy(key_tmp, SECRET_KEY_DEFAULT, 32);
    aes256_key_swap(key_tmp);

    aes256_ecb_decrypt(input, key_tmp, input);
}

/**
 * @brief  Get image size of bank area
 * @param  image_id  image ID
 * @return bank size
 */
uint32_t get_bank_size_by_img_id(IMG_ID image_id)
{
    uint32_t bank_size;

    T_IMG_HEADER_FORMAT *ota_header = (T_IMG_HEADER_FORMAT *)get_active_ota_bank_addr();
    bank_size = ota_header->image_info[(image_id - IMG_OTA - 1) * 2 + 1];;

    return bank_size;
}

/**
 * @brief  Chip reset for dfu
 * @param  reset_mode  reset mode
 * @param  reason      reset reason
 * @return None
 */
void dfu_fw_reboot(WDTMode_TypeDef reset_mode, T_SW_RESET_REASON reason)
{
    DBG_DIRECT("dfu_fw_reboot: reset_mode %x, reason 0x%x", reset_mode, reason);
    chip_reset(reset_mode);
}

/**
 * @brief  Check ota mode flag, if image need update
 * @param  None
 * @return Result: true: image need update, false: image don't need update
 */
bool dfu_check_ota_mode_flag(void)
{
    AON_NS_REG8X_FW_GENERAL_NS_TYPE aon_0x1D10 = {.d32 = AON_REG_READ(AON_NS_REG8X_FW_GENERAL_NS)};
    return aon_0x1D10.ota_mode;
}

/**
 * @brief  Set ota mode flag
 * @param  enable ota mode flag
 * @return None
 */
void dfu_set_ota_mode_flag(bool enable)
{
    AON_NS_REG8X_FW_GENERAL_NS_TYPE aon_0x1D10 = {.d32 = AON_REG_READ(AON_NS_REG8X_FW_GENERAL_NS)};
    if (enable)
    {
        aon_0x1D10.ota_mode = 1;
    }
    else
    {
        aon_0x1D10.ota_mode = 0;
    }
    AON_REG_WRITE(AON_NS_REG8X_FW_GENERAL_NS, aon_0x1D10.d32);
}

/**
 * @brief  Switch to the OTA mode, if support normal ota app need call it.
 * @param  None
 * @return None
 */
void dfu_switch_to_ota_mode(void)
{
    DFU_PRINT_INFO0("==>dfu_switch_to_ota_mode");
    dfu_set_ota_mode_flag(true);

    dfu_fw_reboot(RESET_ALL_EXCEPT_AON, DFU_SWITCH_TO_OTA_MODE);
}

/**
 * @brief  Get the ic type of current firmware
 * @param  None
 * @return IC type
 */
uint8_t dfu_get_ic_type(void)
{
    uint8_t ic_type = IMG_IC_TYPE;
    uint32_t image_addr = get_header_addr_by_img_id(IMG_MCUAPP);

    ic_type = ((T_IMG_HEADER_FORMAT *)image_addr)->ctrl_header.ic_type;

    return ic_type;
}

/**
 * @brief  Get active bank number
 * @param  None
 * @return Active bank number
 */
T_ACTIVE_BANK_NUM get_active_bank_num(void)
{
    T_ACTIVE_BANK_NUM bank_num = OTA_BANK_MAX;

    uint32_t ota_bank0_addr = flash_partition_addr_get(PARTITION_FLASH_OTA_BANK_0);
    uint32_t ota_bank1_addr = flash_partition_addr_get(PARTITION_FLASH_OTA_BANK_1);
    uint32_t ota_header_addr = get_active_ota_bank_addr();
    if (ota_bank0_addr == ota_header_addr)
    {
        bank_num = OTA_BANK0;
    }
    else if (ota_bank1_addr == ota_header_addr)
    {
        bank_num = OTA_BANK1;
    }
    else
    {
        /* invalid case */
        //bank_num = OTA_BANK_MAX;
    }

    return bank_num;
}

/**
 * @brief  Get template ota bank image address by images id
 * @param  image_id  image ID
 * @return Image address
 */
uint32_t dfu_get_temp_ota_bank_img_addr_by_img_id(IMG_ID image_id)
{
    uint32_t image_addr = 0;

    /*not support image id*/
    if (image_id < IMG_DFU_FIRST ||
        ((image_id >= IMG_DFU_MAX) && (image_id < IMG_USER_DATA_FIRST)))
    {
        return image_addr;
    }

    if (image_id < IMG_USER_DATA_MAX && image_id >= IMG_USER_DATA_FIRST)
    {
        dfu_get_user_data_info(image_id, &image_addr, true);
        return image_addr;
    }

#if (SUPPORT_BL_COPY_SECURE_IMAGE == 1)
    if (image_id == IMG_OTA || (image_id >= IMG_BANK_FIRST && image_id < IMG_DFU_MAX))
    {
        image_addr = get_temp_ota_bank_img_addr_by_img_id(image_id);
    }
    else
    {
        image_addr = flash_nor_get_bank_addr(FLASH_OTA_TMP);
    }
#else
    if (!is_ota_support_bank_switch())
    {
        if (image_id == IMG_SECUREMCUAPP || image_id == IMG_SECUREMCUAPPDATA)
        {
            image_addr = flash_partition_addr_get(PARTITION_FLASH_OTA_TMP);
        }
        else
        {
            image_addr = get_temp_ota_bank_img_addr_by_img_id(image_id);
        }
    }
    else
    {
        image_addr = get_temp_ota_bank_img_addr_by_img_id(image_id);
    }
#endif

    return image_addr;
}

/**
 * @brief  Get template ota bank image size by images id
 * @param  image_id  image ID
 * @return Image size
 */
uint32_t dfu_get_temp_ota_bank_img_size_by_img_id(IMG_ID image_id)
{
    uint32_t image_size = 0;

    /*not support image id*/
    if (image_id < IMG_DFU_FIRST ||
        ((image_id >= IMG_DFU_MAX) && (image_id < IMG_USER_DATA_FIRST)))
    {
        //invalid image id
        return image_size;
    }

    if (image_id >= IMG_USER_DATA_FIRST && image_id < IMG_USER_DATA_MAX)
    {
        dfu_get_user_data_info(image_id, &image_size, false);
        return image_size;
    }

#if (SUPPORT_BL_COPY_SECURE_IMAGE == 1)
    if (image_id == IMG_OTA || (image_id >= IMG_BANK_FIRST && image_id < IMG_DFU_MAX))
    {
        image_size = get_temp_ota_bank_img_size_by_img_id(image_id);
    }
    else
    {
        image_size = flash_nor_get_bank_size(FLASH_OTA_TMP);
    }
#else
    if (!is_ota_support_bank_switch())
    {
        if (image_id == IMG_SECUREMCUAPP || image_id == IMG_SECUREMCUAPPDATA)
        {
            image_size = flash_partition_size_get(PARTITION_FLASH_OTA_TMP);
        }
        else
        {
            image_size = get_temp_ota_bank_img_size_by_img_id(image_id);
        }
    }
    else
    {
        image_size = get_temp_ota_bank_img_size_by_img_id(image_id);
    }

#endif

    return image_size;
}

/**
* @brief calculate checksum of lenth of buffer in flash.
*
* @param  signature          signature to identify FW.
* @param  offset             offset of the image.
* @param  length             length of data.
* @param  crcValue          ret crc value point.
* @return  0 if buffer checksum calcs successfully, error line number otherwise
*/

uint32_t dfu_check_bufcrc(uint8_t *buf, uint32_t length, uint16_t mCrcVal)
{
    uint32_t ret = 0;
    uint16_t checksum16 = 0;
    uint32_t i;
    uint16_t *p16;

    p16 = (uint16_t *)buf;
    for (i = 0; i < length / 2; ++i)
    {
        checksum16 = checksum16 ^ (*p16);
        ++p16;
    }

    checksum16 = swap_16(checksum16);
    if (checksum16 != mCrcVal)
    {
        ret = __LINE__;
    }

    DFU_PRINT_TRACE2("<==dfu_check_bufcrc: checksum16=0x%x, mCrcVal=%x", checksum16, mCrcVal);

    return ret;
}

/**
 * @brief  Erase a sector of the flash
 * @param  image_id  image ID
 * @param  offset    flash address offset
 * @return Result    0: success, others: fail
 */
uint32_t dfu_flash_erase(IMG_ID image_id, uint32_t offset)
{
    uint32_t result = 0;
    uint32_t dfu_base_addr;

    dfu_base_addr = dfu_get_temp_ota_bank_img_addr_by_img_id(image_id);

    if (dfu_base_addr == 0)
    {
        return __LINE__;
    }

    result = fmc_flash_nor_erase(dfu_base_addr + offset, FMC_FLASH_NOR_ERASE_SECTOR);

    DFU_PRINT_TRACE1("<==dfu_flash_erase: result=%d", result);

    return result;
}

/**
    * @brief    write data to flash
    * @param    img_id  image id
    * @param    offset  image offset
    * @param    total_offset  total offset when ota temp mode
    * @param    p_void  point of data
    * @return   0: success; other: fail
    */
uint32_t dfu_write_data_to_flash(uint16_t img_id, uint32_t offset, uint32_t total_offset,
                                 uint32_t length, void *p_void)
{
    uint32_t ret = 0;
    uint32_t dfu_base_addr;
    uint8_t readback_buffer[READ_BACK_BUFFER_SIZE];
    uint32_t read_back_len;
    uint32_t dest_addr;
    uint8_t *p_src = (uint8_t *)p_void;
    uint32_t remain_size = length;
    uint32_t s;

    DFU_PRINT_TRACE3("==>dfu_write_data_to_flash: total_offset=0x%x, offset=%d, length=%d",
                     total_offset, offset, length);

    if (p_void == 0)
    {
        ret = __LINE__;
        goto L_EXIT;
    }

    dfu_base_addr = dfu_get_temp_ota_bank_img_addr_by_img_id((IMG_ID)img_id);

    if (dfu_base_addr == 0)
    {
        ret = __LINE__;
        goto L_EXIT;
    }

    if (img_id >= IMG_DFU_FIRST && img_id < IMG_DFU_MAX)
    {
        dfu_base_addr += total_offset;
    }
    if (offset == 0)
    {
        T_IMG_HEADER_FORMAT *p_header = (T_IMG_HEADER_FORMAT *)p_void;

        p_header->ctrl_header.ctrl_flag.not_ready = 0x1;
    }

    dest_addr = dfu_base_addr + offset;
    DFU_PRINT_TRACE2("==>dfu_write_data_to_flash:dfu_base_addr=0x%x, dest_addr=0x%x", dfu_base_addr,
                     dest_addr);
    if ((dest_addr % FLASH_SECTOR_SIZE) == 0)
    {
        s = os_lock();
        fmc_flash_nor_erase(dest_addr, FMC_FLASH_NOR_ERASE_SECTOR);
        os_unlock(s);
    }
    else
    {
        if ((dest_addr / FLASH_SECTOR_SIZE) != ((dest_addr + length) / FLASH_SECTOR_SIZE))
        {
            if ((dest_addr + length) % FLASH_SECTOR_SIZE)
            {
                s = os_lock();
                fmc_flash_nor_erase((dest_addr + length) & ~(FLASH_SECTOR_SIZE - 1),
                                    FMC_FLASH_NOR_ERASE_SECTOR);
                os_unlock(s);
            }
        }
    }

    s = os_lock();
    fmc_flash_nor_write(dest_addr, p_void, length);
    os_unlock(s);

    SCB_InvalidateDCache_by_Addr((uint32_t *)dest_addr, length);

    while (remain_size)
    {
        read_back_len = (remain_size >= READ_BACK_BUFFER_SIZE) ? READ_BACK_BUFFER_SIZE : remain_size;
        s = os_lock();
        fmc_flash_nor_read(dest_addr, readback_buffer, read_back_len);
        os_unlock(s);
        if (memcmp(readback_buffer, p_src, read_back_len) != 0)
        {
            ret = __LINE__;
            goto L_EXIT;
        }

        dest_addr += read_back_len;
        p_src += read_back_len;
        remain_size -= read_back_len;
    }

L_EXIT:
    DFU_PRINT_TRACE1("<==dfu_write_data_to_flash: ret=%d", ret);
    return ret;
}

/**
    * @brief    check the integrity of the image
    * @param    img_id    image id
    * @param    offset    address offset
    * @return   ture:success ; false: fail
    */
bool dfu_checksum(IMG_ID img_id, uint32_t offset)
{
    uint32_t base_addr = 0;
    uint32_t image_total_length = 0;
    bool ret = false;
    DFU_PRINT_TRACE2("==>dfu_checksum: image_id=0x%x, offset=%d", img_id, offset);

    base_addr = dfu_get_temp_ota_bank_img_addr_by_img_id(img_id);

    if (base_addr == 0)
    {
        return false;
    }

    if (img_id >= IMG_DFU_FIRST && img_id < IMG_DFU_MAX)
    {
        base_addr += offset;
    }

    fmc_flash_nor_read(base_addr + offsetof(T_IMG_HEADER_FORMAT, ctrl_header) +
                       offsetof(T_IMG_CTRL_HEADER_FORMAT, payload_len),
                       &image_total_length, 4);
    image_total_length += sizeof(T_IMG_HEADER_FORMAT);

    /*store wdg config and check wdg enable*/
    bool wdt_en = WDT_IsEnable();
    WDTMode_TypeDef wdt_mode = WDT_GetMode();
    uint32_t wdt_timeout = WDT_GetTimeoutMs();

    /*if ota large img, need modify wdg timeout period*/
    if (wdt_en && image_total_length > 0x100000)
    {
        /*1M and less---4s, 2M and less---8s,..., 8M and less---32s*/
        uint32_t img_align_len = ((image_total_length + (0x100000 - 1)) & (~(0x100000 - 1)));
        uint32_t wdt_period = 4000 * (img_align_len / 0x100000);
        DFU_PRINT_TRACE2("<==dfu_checksum: Change WDG Period to %d ms, image_total_length 0x%x",
                         wdt_period, image_total_length);
        WDT_Start(wdt_period, RESET_ALL);
    }

    ret = dfu_check_sha256((T_IMG_HEADER_FORMAT *)base_addr);

    if (wdt_en && image_total_length > 0x100000)
    {
        /* restore wdg config */
        WDT_Start(wdt_timeout, wdt_mode);
    }

    DFU_PRINT_TRACE2("<==dfu_checksum: base_addr=0x%x, ret=%d", base_addr, ret);

    return ret;
}

/**
    * @brief    clear not ready flag of specific image
    * @param    addr    address of the image
    * @return   void
    */
void dfu_set_ready(T_IMG_HEADER_FORMAT *p_header)
{
    T_IMG_CTRL_HEADER_FORMAT ctrl_header;
    uint16_t ctrl_flag;
    uint32_t s;

    s = os_lock();
    fmc_flash_nor_read((uint32_t) & (p_header->ctrl_header.ctrl_flag), &ctrl_flag, sizeof(ctrl_flag));
    os_unlock(s);
    DFU_PRINT_TRACE2("==>dfu_set_ready: p_header 0x%x, ctrl_flag 0x%x", p_header, ctrl_flag);

    ctrl_flag &= ~0x80;
    s = os_lock();
    fmc_flash_nor_write((uint32_t) & (p_header->ctrl_header.ctrl_flag), &ctrl_flag, sizeof(ctrl_flag));
    fmc_flash_nor_read((uint32_t)&p_header->ctrl_header, &ctrl_header, sizeof(ctrl_header));
    os_unlock(s);
    DFU_PRINT_TRACE2("<==dfu_set_ready:img_id 0x%x, after ctrl_flag 0x%x", ctrl_header.image_id,
                     ctrl_header.ctrl_flag);
}

/**
 * @brief  Clear not obsolete flag of specific image
 * @param  p_header pointer to image header
 * @return None
 */
void dfu_set_obsolete(T_IMG_HEADER_FORMAT *p_header)
{
    uint16_t ctrl_flag;
    uint32_t s;
    s = os_lock();
    fmc_flash_nor_read((uint32_t) & (p_header->ctrl_header.ctrl_flag), &ctrl_flag, sizeof(ctrl_flag));
    ctrl_flag &= ~0x0100;
    fmc_flash_nor_write((uint32_t) & (p_header->ctrl_header.ctrl_flag), &ctrl_flag, sizeof(ctrl_flag));
    os_unlock(s);
}


T_USER_DATA_ERROR_TYPE dfu_get_user_data_info(IMG_ID image_id,
                                              uint32_t *img_info, bool is_addr)
{
    T_USER_DATA_ERROR_TYPE err_code = USER_DATA_SUCCESS;

    switch (image_id)
    {
    case IMG_USER_DATA1:
        *img_info = is_addr ? USER_DATA1_ADDR : USER_DATA1_SIZE;
        break;
    case IMG_USER_DATA2:
        *img_info = is_addr ? USER_DATA2_ADDR : USER_DATA2_SIZE;
        break;
    case IMG_USER_DATA3:
        *img_info = is_addr ? USER_DATA3_ADDR : USER_DATA3_SIZE;
        break;
    case IMG_USER_DATA4:
        *img_info = is_addr ? USER_DATA4_ADDR : USER_DATA4_SIZE;
        break;
    case IMG_USER_DATA5:
        *img_info = is_addr ? USER_DATA5_ADDR : USER_DATA5_SIZE;
        break;
    case IMG_USER_DATA6:
        *img_info = is_addr ? USER_DATA6_ADDR : USER_DATA6_SIZE;
        break;
    case IMG_USER_DATA7:
        *img_info = is_addr ? USER_DATA7_ADDR : USER_DATA7_SIZE;
        break;
    case IMG_USER_DATA8:
        *img_info = is_addr ? USER_DATA8_ADDR : USER_DATA8_SIZE;
        break;
    default:
        APP_PRINT_ERROR1("error user data image_id 0x%x", image_id);
        err_code = USER_DATA_TYPE_ERROR;
        *img_info = 0;
        break;
    }

    return err_code;
}

/******************* (C) COPYRIGHT 2023 Realtek Semiconductor Corporation *****END OF FILE****/
