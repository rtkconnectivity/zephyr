zephyr_include_directories(.)

zephyr_library_sources(soc.c)
zephyr_library_sources_ifdef(CONFIG_REALTEK_IMAGE_HEADER image_header.c)

zephyr_linker_sources(ROM_START SORT_KEY 0x0header realtek_image_header.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
