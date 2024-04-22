zephyr_compile_definitions(STACK_ROM_ADDRESS=0x36000)

zephyr_library_sources(soc.c)
zephyr_library_sources_ifdef(CONFIG_REALTEK_IMAGE_HEADER image_header.c)

zephyr_linker_sources(SECTIONS boot_section.ld)
zephyr_linker_sources(ROM_START SORT_KEY 0x0header realtek_image_header.ld)

if (CONFIG_CODE_DATA_RELOCATION AND CONFIG_SOC_SERIES_RTL87X2G)
zephyr_code_relocate(LIBRARY kernel LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/kernel/xip.c LOCATION FLASH NOCOPY)

zephyr_code_relocate(LIBRARY arch__arm__core__cortex_m LOCATION ITCM)
zephyr_code_relocate(LIBRARY arch__arm__core LOCATION ITCM) #for z_arm_nmi
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/drivers/timer/cortex_m_systick.c LOCATION ITCM)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/${ARCH}/core/cortex_m/reset.S LOCATION FLASH)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/${ARCH}/core/cortex_m/prep_c.c LOCATION FLASH)

if (CONFIG_PM)
zephyr_library_sources(power.c)
zephyr_code_relocate(FILES power.c LOCATION ITCM)
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/pm/pm.c ${ZEPHYR_BASE}/subsys/pm/policy.c ${ZEPHYR_BASE}/subsys/pm/state.c ${ZEPHYR_BASE}/subsys/pm/device.c LOCATION ITCM)
endif()

endif()


