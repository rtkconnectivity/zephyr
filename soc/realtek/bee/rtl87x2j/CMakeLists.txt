# Copyright (c) 2026, Realtek Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../../modules/hal/realtek/bee/boot/inc/rtl87x2j)

zephyr_library_sources(soc.c)
zephyr_library_sources_ifdef(CONFIG_SOC_FAMILY_REALTEK_BEE_IMAGE_HEADER bee_image_header.c)

zephyr_link_libraries(
  /home/zhiyuan/upstream/realtek-zephyr-project/build/rtl87x2j_platform_lib/lib/librtl87x2j_platform_lib.a
)
zephyr_include_directories(/home/zhiyuan/upstream/realtek-zephyr-project/lib/rtl87x2j_platform_lib_sample/platform_lib/include)

zephyr_linker_sources(SECTIONS trace_section.ld)
zephyr_linker_sources(RAMFUNC_SECTION rtk_ramfunc.ld)

zephyr_linker_sources(ROM_START SORT_KEY 0x0header bee_image_header.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

zephyr_library_sources_ifdef(CONFIG_PM power.c)
