#
# Copyright (c) 2024 Realtek Semiconductor Corp.
#
# SPDX-License-Identifier: Apache-2.0
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ot-cli)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/alarm.c)
target_link_options(app
    PUBLIC
        -Wl,--wrap,memcpy
        -Wl,--wrap,memset
        -Wl,--wrap,mac_EnterCritical_rom
        -Wl,--wrap,mac_ExitCritical_rom
        -Wl,--wrap,otPlatReset
        -Wl,--wrap,platformAlarmMilliInit
        -Wl,--wrap,platformAlarmMilliProcess
        -Wl,--wrap,otPlatAlarmMilliGetNow
        -Wl,--wrap,otPlatAlarmMilliStartAt
        -Wl,--wrap,otPlatAlarmMilliStop
        -Wl,--wrap,platformAlarmMicroInit
        -Wl,--wrap,platformAlarmMicroProcess
        -Wl,--wrap,otPlatAlarmMicroGetNow
        -Wl,--wrap,otPlatAlarmMicroStartAt
        -Wl,--wrap,otPlatAlarmMicroStop
        -Wl,--wrap,otPlatTimeGet
        -Wl,--wrap,otPlatLog
)
zephyr_code_relocate(FILES src/alarm.c LOCATION ITCM_TEXT)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/modules/openthread/platform/radio.c LOCATION ITCM_TEXT)
