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
#include <zephyr/sw_isr_table.h>
#include <soc.h>
#include <cmsis_core.h>
#include <log_core.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

bool (*patch_ram_vector_table_update)(int irqn, IRQ_Fun isr_handler, bool *ret);
extern void default_handler(void);

static bool zephyr_ram_vector_table_update(int irqn, IRQ_Fun isr_handler, bool *ret)
{
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
	/* Assign Zephyr version of ram_vector_table_update to patch variable */
	patch_ram_vector_table_update = zephyr_ram_vector_table_update;
}

void soc_late_init_hook(void)
{
	/* Placeholder for late initialization */
	uint32_t *RamVectorTable_INT = (uint32_t *)(0x20019600 + 16 * 4);

	for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
		if (RamVectorTable_INT[irq] != (uint32_t)default_handler) {
			printk("Warning: IRQ %d has a non-default handler at address 0x%08X\n", irq, RamVectorTable_INT[irq]);
			DBG_DIRECT("Warning: IRQ %d has a non-default handler at address 0x%08X\n", irq, RamVectorTable_INT[irq]);
			if (NVIC_GetEnableIRQ(irq) == 1) {
				NVIC_DisableIRQ(irq);
				z_isr_install(irq, (void *)RamVectorTable_INT[irq], NULL);
				NVIC_EnableIRQ(irq);
			} else {
				z_isr_install(irq, (void *)RamVectorTable_INT[irq], NULL);
			}
		}
	}
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
