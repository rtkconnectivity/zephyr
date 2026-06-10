/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/common/init.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sw_isr_table.h>
#include <soc.h>
#include <cmsis_core.h>

#include "osif_zephyr.h"
#include "system_init.h"
#include "mem_config.h"
#include "utils.h"
#ifdef CONFIG_BT
#include "image_info.h"
#endif

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

/*
 * Migrate RAM vector table handlers to Zephyr ISR table.
 * This is needed because some IRQs are configured before Zephyr takes over,
 * and we need to register those handlers in Zephyr's ISR table.
 */
static void migrate_ram_vector_table_to_zephyr(void)
{
	/* Skip first 16 system exception vectors */
	uint32_t *ram_vector_table = (uint32_t *)(DATA_RAM_ROM_GLOBAL_ADDR + 16 * 4);

	for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
		if (ram_vector_table[irq] != (uint32_t)default_handler) {
			LOG_DBG("IRQ %d has a non-default handler at address 0x%08X, "
				"registering in Zephyr ISR table\n",
				irq, ram_vector_table[irq]);
			if (NVIC_GetEnableIRQ(irq) == 1) {
				NVIC_DisableIRQ(irq);
				z_isr_install(irq, (void *)ram_vector_table[irq], NULL);
				NVIC_EnableIRQ(irq);
			} else {
				z_isr_install(irq, (void *)ram_vector_table[irq], NULL);
			}
		}
	}
}

#ifdef CONFIG_BT
static void bt_controller_init(void)
{
	IMG_ID image_id =
		(ota_dual_bank_enable() == true) ? PRE_IMG_BT_CONTROLLER : IMG_BT_CONTROLLER_PATCH;
	image_entry_validation(image_id);
}
#endif

void soc_early_init_hook(void)
{
	migrate_ram_vector_table_to_zephyr();

	/* Assign Zephyr version of ram_vector_table_update to patch variable */
	patch_ram_vector_table_update = zephyr_ram_vector_table_update;

	os_zephyr_patch_init();
}

void soc_late_init_hook(void)
{
	wakeup_init();

	power_manager_init();

	platform_pm_init();

	thermal_meter_init();

	phy_hw_control_init(false);
	phy_init(false);

	thermal_tracking_init();

	amu_script_init();

	amu_init();

	/* Switch log UART clock to auto mode for better power saving */
	extern void log_uart_switch_clock_auto_mode(bool enable);
	log_uart_switch_clock_auto_mode(true);

#ifdef CONFIG_BT
	/*
	 * The BT controller relies on the rand() implementation in the bootloader (ROM).
	 * srand_bl() is the corresponding seed initializer for that ROM rand function.
	 * Use Zephyr's entropy module to supply a hardware-random seed so that each
	 * boot produces a different random sequence.
	 */
	extern void srand_bl(int seed);

#if DT_HAS_CHOSEN(zephyr_entropy)
	{
		const struct device *const entropy_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
		uint32_t seed = 0;

		if (entropy_get_entropy(entropy_dev, (uint8_t *)&seed, sizeof(seed)) == 0) {
			printf("Seeding BT controller RNG with 0x%08X\n", seed);
			srand_bl((int)seed);
		} else {
			srand_bl(0);
		}
	}
#else
	srand_bl(0);
#endif

	bt_controller_init();
#endif
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT
void arch_busy_wait(uint32_t usec_to_wait)
{
	platform_delay_us(usec_to_wait);
}
#endif
