# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
zephyr_library_sources_ifdef(CONFIG_REALTEK_IMAGE_HEADER image_header.c)
zephyr_linker_sources(ROM_START SORT_KEY 0x0header realtek_image_header.ld)
zephyr_linker_sources(SECTIONS trace_section.ld)
zephyr_library_sources(soc.c)
zephyr_include_directories(.)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
if (CONFIG_CODE_DATA_RELOCATION)
# save DataRAM for APP
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/logging/log_core.c LOCATION RAM2_BSS)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/shell/backends/shell_uart.c LOCATION RAM2_BSS)

# for z_arm_int_exit
zephyr_code_relocate(LIBRARY arch__arm__core__cortex_m LOCATION RAM)
# for z_arm_nmi
zephyr_code_relocate(LIBRARY arch__arm__core LOCATION RAM)
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/arm/core/nmi.c LOCATION RAM)

zephyr_code_relocate(FILES ${ZEPHYR_BASE}/drivers/timer/cortex_m_systick.c LOCATION RAM)
# ToDo: It needs to be determined whether this is a must-relocate item. The entry function cannot be placed directly in RAM, as RAM initialization (RAM copy) has not yet been executed.
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/${ARCH}/core/cortex_m/reset.S LOCATION FLASH)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/${ARCH}/core/cortex_m/prep_c.c LOCATION FLASH)

# ToDo: osif
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/../modules/hal/realtek/bee/rtl8752h/osif/zephyr/osif_zephyr.c LOCATION RAM)

if (CONFIG_PM)
zephyr_library_sources(power.c)
zephyr_code_relocate(FILES power.c LOCATION RAM)
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/pm/device.c LOCATION ITCM)
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/lib/os/rb.c LOCATION ITCM)
endif()
endif()
