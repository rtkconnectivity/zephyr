# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
zephyr_library_sources_ifdef(CONFIG_REALTEK_IMAGE_HEADER image_header.c)
zephyr_linker_sources(ROM_START SORT_KEY 0x0header realtek_image_header.ld)
zephyr_linker_sources(SECTIONS trace_section.ld)
zephyr_library_sources(soc.c)
zephyr_include_directories(.)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

if (CONFIG_PM)
zephyr_library_sources(power.c)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/arm/core/nmi.c LOCATION RAM) #for z_arm_nmi
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/drivers/pinctrl/pinctrl_bee.c LOCATION RAM) #192bytes
endif()
