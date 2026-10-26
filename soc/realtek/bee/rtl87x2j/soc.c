/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/logging/log.h>
#include <zephyr/sw_isr_table.h>
#include <soc.h>
#include <cmsis_core.h>

#include <osif_zephyr.h>
#include <system_init.h>
#include <sys_reset.h>

#include <rtl87x2j_platform_init.h>
#include <rtl87x2j_btcontroller_init.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#ifdef CONFIG_BT
/* Boundaries of the ".bss.sram" output section, see rtk_bt_controller_bss.ld */
extern char __bt_ctrl_bss_start[];
extern char __bt_ctrl_bss_end[];
#endif

/* The platform init below runs in the application image only, not in MCUboot */
#ifndef CONFIG_MCUBOOT
static bool zephyr_ram_vector_table_update(int irqn, IRQ_Fun isr_handler, bool *ret)
{
	if (irqn > IRQn_MAX) {
		LOG_INF("Update Secondary Level ISRs, IRQ %d!", irqn);
		/* return false to use ram_vector_table_update_rom() */
		return false;
	}

	if (irqn == NMI_IRQn) {
		z_arm_nmi_set_handler(isr_handler);
		LOG_INF("NMI handler updated via ram_vector_table_update()!");
		return true;
	}

	if (NVIC_GetEnableIRQ(irqn) == 1) {
		NVIC_DisableIRQ(irqn);
		z_isr_install(irqn, (void *)isr_handler, NULL);
		NVIC_EnableIRQ(irqn);
	} else {
		z_isr_install(irqn, (void *)isr_handler, NULL);
	}

	if (ret != NULL) {
		*ret = true;
	}

	return true;
}
#endif /* !CONFIG_MCUBOOT */

void soc_reset_hook(void)
{
	rtl87x2j_platform_reset_init();
}

void soc_early_init_hook(void)
{
#ifdef CONFIG_BT
	/* Not covered by the kernel's bss clearing (__bss_start..__bss_end) */
	memset(__bt_ctrl_bss_start, 0, __bt_ctrl_bss_end - __bt_ctrl_bss_start);
#endif

#ifndef CONFIG_MCUBOOT
	os_zephyr_patch_init();

	/* Assign Zephyr version of ram_vector_table_update to patch variable */
	patch_ram_vector_table_update = zephyr_ram_vector_table_update;

	rtl87x2j_platform_early_init();
#endif
}

void soc_late_init_hook(void)
{
#ifndef CONFIG_MCUBOOT
	rtl87x2j_platform_late_init();
#endif
#ifdef CONFIG_BT
	rtl87x2j_btcontroller_init();
#endif
}

/* Overrides the weak ARM implementation */
void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);
	sys_reset(RESET_REASON_ZEPHYR);
}
