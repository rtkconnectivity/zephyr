# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/ir.h)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_IR_BEE ir_bee.c)
