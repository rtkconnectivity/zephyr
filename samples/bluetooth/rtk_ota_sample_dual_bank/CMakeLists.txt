# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rtk_ota_sample_dual_bank)

target_sources(app PRIVATE
  src/main.c
  src/cts.c
)