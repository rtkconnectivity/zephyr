/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Variables tagged with __attribute__((section(".bss.sram"))) are placed in the
 * BT_CTRL_RAM region (buffer ram, see bt_ctrl_ram in rtl87x2j.dtsi)
 * instead of the generic .bss in main sram.
 *
 * This must be pulled in via zephyr_linker_sources(SECTIONS ...) so it is
 * processed before the generic *(".bss.*") match in linker.ld.
 * The region is not covered by __bss_start/__bss_end, so it is cleared in
 * soc_early_init_hook().
 */
GROUP_START(BT_CTRL_RAM)

SECTION_DATA_PROLOGUE(BT_CTRL_BSS,(NOLOAD),)
{
	. = ALIGN(4);
	__bt_ctrl_bss_start = .;
	*(.bss.sram)
	*(".bss.sram.*")
	. = ALIGN(4);
	__bt_ctrl_bss_end = .;
} GROUP_DATA_LINK_IN(BT_CTRL_RAM, BT_CTRL_RAM)

GROUP_END(BT_CTRL_RAM)
