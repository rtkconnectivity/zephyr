/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/common/init.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/logging/log.h>
#include <soc.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

void soc_early_init_hook(void)
{
	/* Placeholder for early initialization */
}

void soc_late_init_hook(void)
{
	/* Placeholder for late initialization */
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT
void arch_busy_wait(uint32_t usec_to_wait)
{
	/* TODO: Implement platform delay */
	volatile uint32_t count = usec_to_wait * 40;

	while (count--) {
		__NOP();
	}
}
#endif
