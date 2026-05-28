/*
 * Copyright(c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_eflash

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>

#include <eflash.h>

LOG_MODULE_REGISTER(eflash_bee, CONFIG_FLASH_LOG_LEVEL);

struct eflash_bee_config {
	uintptr_t base_addr;
	size_t size;
	size_t write_block_size;
	size_t erase_block_size;
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_layout pages_layout;
#endif
};

static int eflash_bee_check_bounds(const struct device *dev, off_t offset, size_t len)
{
	const struct eflash_bee_config *cfg = dev->config;

	if (offset < 0) {
		LOG_DBG("negative offset: %ld", (long)offset);
		return -EINVAL;
	}

	if ((size_t)offset > cfg->size) {
		LOG_DBG("offset %ld out of bounds, flash size %zu", (long)offset, cfg->size);
		return -EINVAL;
	}

	if (len > (cfg->size - (size_t)offset)) {
		LOG_DBG("offset %ld len %zu out of bounds, flash size %zu", (long)offset, len,
			cfg->size);
		return -EINVAL;
	}

	return 0;
}

static int eflash_bee_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct eflash_bee_config *cfg = dev->config;
	bool ret;
	int rc;

	if (len == 0U) {
		return 0;
	}

	if (data == NULL) {
		LOG_ERR("read buffer is NULL");
		return -EINVAL;
	}

	rc = eflash_bee_check_bounds(dev, offset, len);
	if (rc != 0) {
		return rc;
	}

	ret = eflash_read((uint32_t)(cfg->base_addr + (uintptr_t)offset), (uint8_t *)data, len);

	if (!ret) {
		LOG_ERR("eflash_read failed at addr=0x%08lx len=%zu",
			(unsigned long)(cfg->base_addr + (uintptr_t)offset), len);
		return -EIO;
	}

	return 0;
}

static int eflash_bee_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct eflash_bee_config *cfg = dev->config;
	bool ret;
	int rc;

	if (len == 0U) {
		return 0;
	}

	if (data == NULL) {
		LOG_ERR("write buffer is NULL");
		return -EINVAL;
	}

	rc = eflash_bee_check_bounds(dev, offset, len);
	if (rc != 0) {
		return rc;
	}

	ret = eflash_write((uint32_t)(cfg->base_addr + (uintptr_t)offset), (uint8_t *)data, len);
	if (!ret) {
		LOG_ERR("eflash_write failed at addr=0x%08lx len=%zu",
			(unsigned long)(cfg->base_addr + (uintptr_t)offset), len);
		return -EIO;
	}

	return 0;
}

static int eflash_bee_erase(const struct device *dev, off_t offset, size_t len)
{
	const struct eflash_bee_config *cfg = dev->config;
	uintptr_t addr;
	unsigned int key;
	size_t remaining;
	bool ret;
	int rc;

	if (len == 0U) {
		return 0;
	}

	if ((offset < 0) || (((size_t)offset % cfg->erase_block_size) != 0U)) {
		LOG_ERR("offset %ld is not aligned to erase block size %zu", (long)offset,
			cfg->erase_block_size);
		return -EINVAL;
	}

	if ((len % cfg->erase_block_size) != 0U) {
		LOG_ERR("len %zu is not aligned to erase block size %zu", len,
			cfg->erase_block_size);
		return -EINVAL;
	}

	rc = eflash_bee_check_bounds(dev, offset, len);
	if (rc != 0) {
		return rc;
	}

	addr = cfg->base_addr + (uintptr_t)offset;

	key = irq_lock();

	for (remaining = len; remaining > 0U; remaining -= cfg->erase_block_size) {
		ret = eflash_erase((uint32_t)addr);
		if (!ret) {
			irq_unlock(key);
			LOG_ERR("eflash_erase failed at addr=0x%08lx", (unsigned long)addr);
			return -EIO;
		}

		addr += cfg->erase_block_size;
	}

	irq_unlock(key);

	return 0;
}

static const struct flash_parameters eflash_bee_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

static const struct flash_parameters *eflash_bee_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &eflash_bee_parameters;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void eflash_bee_page_layout(const struct device *dev,
				   const struct flash_pages_layout **layout, size_t *layout_size)
{
	const struct eflash_bee_config *cfg = dev->config;

	*layout = &cfg->pages_layout;
	*layout_size = 1;
}
#endif

static int eflash_bee_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static DEVICE_API(flash, eflash_bee_driver_api) = {
	.read = eflash_bee_read,
	.write = eflash_bee_write,
	.erase = eflash_bee_erase,
	.get_parameters = eflash_bee_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = eflash_bee_page_layout,
#endif
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
#define BEE_EFLASH_LAYOUT_INIT(inst)                                                               \
	.pages_layout = {                                                                          \
		.pages_size = DT_INST_PROP(inst, erase_block_size),                                \
		.pages_count = DT_INST_REG_SIZE(inst) / DT_INST_PROP(inst, erase_block_size),      \
	},
#else
#define BEE_EFLASH_LAYOUT_INIT(inst)
#endif

#define BEE_EFLASH_DEFINE(inst)                                                                    \
	static const struct eflash_bee_config eflash_bee_cfg_##inst = {                            \
		.base_addr = DT_INST_REG_ADDR(inst),                                               \
		.size = DT_INST_REG_SIZE(inst),                                                    \
		.write_block_size = DT_INST_PROP(inst, write_block_size),                          \
		.erase_block_size = DT_INST_PROP(inst, erase_block_size),                          \
		BEE_EFLASH_LAYOUT_INIT(inst)};                                                     \
	DEVICE_DT_INST_DEFINE(inst, eflash_bee_init, NULL, NULL, &eflash_bee_cfg_##inst,           \
			      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &eflash_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BEE_EFLASH_DEFINE)
