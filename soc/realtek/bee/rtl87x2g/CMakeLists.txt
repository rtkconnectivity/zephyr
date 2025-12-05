# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
zephyr_include_directories(.)

zephyr_compile_definitions(STACK_ROM_ADDRESS=0x36000)

zephyr_library_sources(soc.c)
zephyr_library_sources_ifdef(CONFIG_REALTEK_IMAGE_HEADER image_header.c)

zephyr_linker_sources(ROM_START SORT_KEY 0x0header realtek_image_header.ld)
zephyr_linker_sources(SECTIONS rtk_ext_ram.ld)
zephyr_linker_sources(SECTIONS trace_section.ld)
zephyr_linker_sources(ROM_SECTIONS rtk_xip.ld)
zephyr_linker_sources(RAMFUNC_SECTION rtk_ramfunc.ld)
zephyr_linker_sources(RODATA rtk_rodata.ld)
zephyr_linker_sources(RWDATA rtk_rwdata.ld)
# zephyr_linker_sources(SECTIONS boot_section.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")


if (CONFIG_PM)
zephyr_library_sources(power.c)
zephyr_code_relocate(FILES power.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/pm/device.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/arm/core/nmi.c LOCATION ITCM) #for z_arm_nmi
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/drivers/pinctrl/pinctrl_bee.c LOCATION ITCM) #272bytes
endif()
