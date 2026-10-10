/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
#include <rtl87x2j_patch.h>
#include <rtl87x2j_btcontroller_init.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

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

void soc_early_init_hook(void)
{
	os_zephyr_patch_init();//both mcuboot and Zephyr need OSIF

	/* Assign Zephyr version of ram_vector_table_update to patch variable */
	//may only place at mcuboot side to save code size??
	patch_ram_vector_table_update = zephyr_ram_vector_table_update;

	rtl87x2j_patch_pointer_init();
	rtl87x2j_patch_data_init();

#if !defined(CONFIG_BOOTLOADER_MCUBOOT)
	rtl87x2j_platform_early_init();
#endif
}

void soc_late_init_hook(void)
{
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
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
