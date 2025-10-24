/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

#include <platform_cfg.h>
#include <flash_nor_device.h>
#include <trace.h>

#define DT_DRV_COMPAT realtek_rtl8752h_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#define FLASH_SIZE DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_ADDR DT_REG_ADDR(SOC_NV_FLASH_NODE)

LOG_MODULE_REGISTER(flash_rtl8752h, CONFIG_FLASH_LOG_LEVEL);
struct flash_rtl8752h_data {
#ifdef CONFIG_MULTITHREADING
	struct k_sem sem;
#endif
	uint8_t g_flash_old_bp_lv;
};

#ifdef CONFIG_MULTITHREADING
#define FLASH_SEM_TIMEOUT (k_is_in_isr() ? K_NO_WAIT : K_FOREVER)
static inline void flash_rtl8752h_sem_take(const struct device *dev)
{
	struct flash_rtl8752h_data *dev_data = dev->data;

	k_sem_take(&dev_data->sem, FLASH_SEM_TIMEOUT);
}

static inline void flash_rtl8752h_sem_give(const struct device *dev)
{
	struct flash_rtl8752h_data *dev_data = dev->data;

	k_sem_give(&dev_data->sem);
}
#else

#define flash_rtl8752h_sem_take(dev) do {} while (0)
#define flash_rtl8752h_sem_give(dev) do {} while (0)

#endif

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_pages_layout_rtl8752h[] = {
	{
		.pages_size = FLASH_ERASE_BLK_SZ,
		.pages_count = FLASH_SIZE / FLASH_ERASE_BLK_SZ
	}
};
static void
flash_rtl8752h_page_layout(const struct device *dev,
						const struct flash_pages_layout **layout,
						size_t *layout_size)
{
	*layout = flash_pages_layout_rtl8752h;

	/*
	 * For flash memories which have uniform page sizes, this routine
	 * returns an array of length 1, which specifies the page size and
	 * number of pages in the memory.
	 */
	*layout_size = 1;
}
#endif

static const struct flash_parameters flash_rtl8752h_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

static int flash_rtl8752h_read(const struct device *dev, off_t offset,
							   void *data, size_t len)
{
	if ((offset > FLASH_SIZE) ||
		((offset + len) > FLASH_SIZE)) {
		LOG_ERR("offset(:0x%lx) or offset+len(:0x%lx) out of flash boundary", (long)offset,
				(long)(offset + len));
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}
	flash_rtl8752h_sem_take(dev);
	flash_nor_read_locked(FLASH_ADDR + offset, (uint8_t *)data, len);
	flash_rtl8752h_sem_give(dev);

	return 0;
}

static int flash_rtl8752h_write(const struct device *dev, off_t offset,
								const void *data, size_t len)
{
	FLASH_NOR_RET_TYPE ret;
	if ((offset > FLASH_SIZE) ||
		((offset + len) > FLASH_SIZE)) {
		LOG_ERR("offset(:0x%lx) or offset+len(:0x%lx) out of flash boundary", (long)offset,
				(long)(offset + len));
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

#if CONFIG_KERNEL_MEM_POOL && CONFIG_HEAP_MEM_POOL_SIZE
	if ((uint32_t)data >= FLASH_ADDR) {
		uint8_t *tmp = k_malloc(len);

		if (tmp != NULL) {
			flash_rtl8752h_sem_take(dev);
			flash_nor_read_locked((uint32_t)data, (uint8_t *)tmp, len);
			ret = flash_nor_write_locked(FLASH_ADDR + offset, (uint8_t *)tmp, len);
			k_free(tmp);
			flash_rtl8752h_sem_give(dev);
			if (ret != FLASH_NOR_RET_SUCCESS) {
				LOG_ERR("write failed");
				return -EIO;
			}
		} else {
			LOG_ERR("k_malloc %x0x for flash data transfer station failed", len);
		}
		return 0;
	}
#else
	__ASSERT((uint32_t)data < FLASH_ADDR,
		"not supported: data in flash (0x%x) cannot be used as source",
		(uint32_t)data);
#endif
	flash_rtl8752h_sem_take(dev);
	ret = flash_nor_write_locked(FLASH_ADDR + offset, (uint8_t *)data, len);
	flash_rtl8752h_sem_give(dev);
	if (ret != FLASH_NOR_RET_SUCCESS) {
		LOG_ERR("write failed");
		return -EIO;
	}
	return 0;
}

static int flash_rtl8752h_erase(const struct device *dev, off_t offset, size_t size)
{
	FLASH_NOR_RET_TYPE ret;

	if ((offset > FLASH_SIZE) ||
		((offset + size) > FLASH_SIZE)) {
		LOG_ERR("offset(:0x%lx) or offset+size(:0x%lx) is out of flash boundary",
			(long)offset, (long)(offset + size));
		return -EINVAL;
	}

	if ((offset % FLASH_ERASE_BLK_SZ) != 0) {
		LOG_ERR("offset 0x%lx: not on a page boundary", (long)offset);
		return -EINVAL;
	}

	if ((size % FLASH_ERASE_BLK_SZ) != 0) {
		LOG_ERR("size %zu: not multiple of a page size", size);
		return -EINVAL;
	}

	if (!size) {
		return 0;
	}

	uint32_t start_addr = FLASH_ADDR + offset;

	for (int i = 0; i < size / FLASH_ERASE_BLK_SZ; i++) {
		flash_rtl8752h_sem_take(dev);
		ret = flash_nor_erase_locked(start_addr + i * FLASH_ERASE_BLK_SZ,
				FLASH_NOR_ERASE_SECTOR);
		flash_rtl8752h_sem_give(dev);
		if (ret != FLASH_NOR_RET_SUCCESS) {
			LOG_ERR("erase failed");
			return -EIO;
		}
	}
	return 0;
}

static const struct flash_parameters *
flash_rtl8752h_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_rtl8752h_parameters;
}

static const struct flash_driver_api flash_rtl8752h_driver_api = {
	.read = flash_rtl8752h_read,
	.write = flash_rtl8752h_write,
	.erase = flash_rtl8752h_erase,
	.get_parameters = flash_rtl8752h_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_rtl8752h_page_layout,
#endif
};

bool flash_rtl8752h_unlock_flash_bp_all(const struct device *dev)
{
	FLASH_NOR_RET_TYPE ret;
	struct flash_rtl8752h_data *dev_data = dev->data;

	LOG_DBG("Flash unlock BP");

	flash_rtl8752h_sem_take(dev);
	ret = flash_nor_unlock_bp_by_addr_locked(FLASH_ADDR,
		&dev_data->g_flash_old_bp_lv);
	flash_rtl8752h_sem_give(dev);

	if (ret != FLASH_NOR_RET_SUCCESS) {
		LOG_ERR("Flash unlock BP failed");
		return false;
	}

	LOG_DBG("<==Unlock Total Flash Success! prev_bp_lv=%d",
		dev_data->g_flash_old_bp_lv);

	return true;
}

void flash_rtl8752h_lock_flash_bp(const struct device *dev)
{
	struct flash_rtl8752h_data *dev_data = dev->data;

	if (dev_data->g_flash_old_bp_lv != 0xff) {
		flash_rtl8752h_sem_take(dev);
		flash_nor_set_bp_lv_locked(FLASH_NOR_IDX_SPIC0, dev_data->g_flash_old_bp_lv);
		flash_rtl8752h_sem_give(dev);
	}
}

#define GET_FLASH_BIT_MODE_STR(mode) \
	((mode) == FLASH_NOR_1_BIT_MODE ? "FLASH_NOR_1_BIT_MODE" : \
	(mode) == FLASH_NOR_2_BIT_MODE ? "FLASH_NOR_2_BIT_MODE" : \
	(mode) == FLASH_NOR_4_BIT_MODE ? "FLASH_NOR_4_BIT_MODE" : "Invalid mode")
static int flash_rtl8752h_init(const struct device *dev)
{
	struct flash_rtl8752h_data *dev_data = dev->data;

#ifdef CONFIG_MULTITHREADING
	k_sem_init(&dev_data->sem, 1, 1);
#endif
	dev_data->g_flash_old_bp_lv = 0xFF;

	if (flash_nor_get_exist(FLASH_NOR_IDX_SPIC0) != FLASH_NOR_EXIST_NONE) {
		if (flash_nor_load_query_info(FLASH_NOR_IDX_SPIC0) == FLASH_NOR_RET_SUCCESS) {
			/* apply SW Block Protect */
			if (boot_cfg.flash_setting.bp_enable) {
				/**
				 * set flash default block protect level depend on
				 * different flash id and different flash layout
				 */
				boot_cfg.flash_setting.bp_lv = flash_nor_get_default_bp_lv();
			} else {
				boot_cfg.flash_setting.bp_lv = 0;
			}

			if (flash_nor_set_tb_bit(FLASH_NOR_IDX_SPIC0, 1) == FLASH_NOR_RET_SUCCESS &&
			    flash_nor_set_bp_lv(FLASH_NOR_IDX_SPIC0,
						boot_cfg.flash_setting.bp_lv) ==
				    FLASH_NOR_RET_SUCCESS) {
				FLASH_PRINT_INFO1("Flash BP Lv = %d", boot_cfg.flash_setting.bp_lv);
			} else {
				FLASH_PRINT_INFO0("Flash BP fail!");
			}
		}
		flash_nor_dump_flash_info();
	}

	/* Switch flash to 4-bit mode before kernel starts to avoid xip isr affecting the calibration flow. */
    if (flash_nor_try_high_speed_mode(FLASH_NOR_IDX_SPIC0,
	CONFIG_SOC_FLASH_RTL8752H_BIT_MODE) == FLASH_NOR_RET_SUCCESS) {
	DBG_DIRECT("Flash change to %s",
	    GET_FLASH_BIT_MODE_STR(CONFIG_SOC_FLASH_RTL8752H_BIT_MODE)
	);
    }

	return 0;
}

static struct flash_rtl8752h_data flash_data;

DEVICE_DT_INST_DEFINE(0, flash_rtl8752h_init, NULL,
					  &flash_data, NULL, PRE_KERNEL_1,
					  CONFIG_FLASH_INIT_PRIORITY, &flash_rtl8752h_driver_api);
