# Copyright (c) 2026, Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
board_runner_args(jlink "--device=RTL87X2J" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/mpcli.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/rtkprog.board.cmake)