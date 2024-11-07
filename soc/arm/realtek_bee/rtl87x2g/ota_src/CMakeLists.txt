zephyr_library()
zephyr_compile_definitions(SUPPORT_NORMAL_OTA=0)
zephyr_include_directories(.)

zephyr_library_sources(
  ble_dfu_transport.c
  rtk_ota_service.c
  rtk_dfu_service.c
  dfu_common.c
  rtk_ota_app_layer.c
  rtk_ota_bt.c
)
