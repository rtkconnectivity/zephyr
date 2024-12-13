# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
board_runner_args(jlink "--device=RTL87X2G" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
