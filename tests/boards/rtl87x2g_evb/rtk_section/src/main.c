/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/devicetree.h>

#include <app_section.h>

APP_RAM_DATA_SECTION uint32_t test_section_ram_data[5] = {1, 2, 3, 4, 5};
EXT_DATA_SRAM_SECTION uint32_t test_section_ext_ram_data[5] = {1, 2, 3, 4, 5};

APP_RAM_TEXT_SECTION void test_section_ram_func(void)
{
	TC_PRINT("In %s\n", __func__);
}
ISR_TEXT_SECTION void test_section_isr_text(void)
{
	TC_PRINT("In %s\n", __func__);
}

#define EXTRAM_START DT_REG_ADDR(DT_NODELABEL(ext_data_ram))
#define EXTRAM_SIZE  DT_REG_SIZE(DT_NODELABEL(ext_data_ram))

#define TCM0_START DT_REG_ADDR(DT_NODELABEL(tcm0))
#define TCM0_SIZE  DT_REG_SIZE(DT_NODELABEL(tcm0))

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)
#define FLASH_SIZE        DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_ADDR        DT_REG_ADDR(SOC_NV_FLASH_NODE)

ZTEST(rtk_section, test_rtk_section_imported_lib)
{
	test_section_ram_func();
	zassert_true((uint32_t)test_section_ram_func >= TCM0_START &&
			    (uint32_t)test_section_ram_func <= (TCM0_START + TCM0_SIZE),
		     "rtk section APP_RAM_TEXT_SECTION is not working!");

	test_section_isr_text();
	zassert_true((uint32_t)test_section_isr_text >= FLASH_ADDR &&
			    (uint32_t)test_section_isr_text <= (FLASH_SIZE + FLASH_ADDR),
		     "rtk section ISR_TEXT_SECTION is not working!");

	for (int i = 0; i < 5; i++) {
		TC_PRINT("test_section_ext_ram_data value[%d]:%d\n", i,
			 test_section_ext_ram_data[i]);
	}
	zassert_true((uint32_t)test_section_ext_ram_data >= EXTRAM_START &&
			      (uint32_t)test_section_ext_ram_data <= (EXTRAM_START + EXTRAM_SIZE),
		     "rtk section EXT_DATA_SRAM_SECTION is not working!");

	for (int i = 0; i < 5; i++) {
		TC_PRINT("test_section_ram_data value[%d]:%d\n", i, test_section_ram_data[i]);
	}
	zassert_true((uint32_t)test_section_ram_data >= TCM0_START &&
			     (uint32_t)test_section_ram_data <= (TCM0_START + TCM0_SIZE),
		     "rtk section APP_RAM_DATA_SECTION is not working!");
}

ZTEST_SUITE(rtk_section, NULL, NULL, NULL, NULL, NULL);
