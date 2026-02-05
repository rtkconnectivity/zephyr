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
#include <soc.h>

#include "system_init_ns.h"
#include "utils.h"
#include "sys_reset.h"
#include "osif_zephyr.h"
#include "clock_manager.h"
#include "vector_table.h"
#include "image_info.h"
#include "image_check.h"
#include "rom_uuid.h"
#include "trace.h"

extern char __extram_data_start[];
extern char __extram_data_end[];
extern char __extram_data_load_start[];
extern char __extram_bss_start[];
extern char __extram_bss_end[];

extern void _isr_wrapper(void);
extern void z_arm_svc(void);

#define S_RAM_VECTOR_ADDR               (0x14ec00)
#define STACK_ROM_ADDRESS   DT_REG_ADDR(DT_NODELABEL(bee_bt_controller))

typedef bool (*BOOL_PATCH_FUNC)();

static void rtl87x2g_extra_ram_init(void)
{
	arch_early_memcpy(__extram_data_start, __extram_data_load_start,
			(uintptr_t) __extram_data_end - (uintptr_t) __extram_data_start);
	arch_early_memcpy(__extram_bss_start, 0,
			(uintptr_t) __extram_bss_end - (uintptr_t) __extram_bss_start);
}


static void rtl87x2g_bt_controller_init(void)
{
	BOOL_PATCH_FUNC bt_controller_entry;
	T_ROM_HEADER_FORMAT *stack_header = (T_ROM_HEADER_FORMAT *)STACK_ROM_ADDRESS;

	uint8_t target_uuid[] = DEFINE_symboltable_uuid;

	if (memcmp(stack_header->uuid, target_uuid, UUID_SIZE) == 0) {
		bt_controller_entry = (BOOL_PATCH_FUNC)((uint32_t)stack_header->entry_ptr);
		printf("Load Realtek Bee BT Controller ROM>>>>\n");
		bt_controller_entry();
		printf("<<<<Successfully loaded Realtek Bee BT Controller ROM!\n");
	} else {
		printf("Failed to load Realtek Bee BT Controller ROM!\n");
	}
}

/*
 * The RTL87X2G initialization process updates vector entries in the RamVectorTable.
 * Therefore, we need to register these ISRs into Zephyr's interrupt system.
 */
static void rtl87x2g_isr_register(void)
{
	/*
	 * For interrupts that update the ISR during the RTL87X2G initialization,
	 * the following steps are necessary:
	 * 1. Register the ISR in Zephyr's sw_isr_table.
	 * 2. Update Zephyr's ISR wrapper back into the RamVectorTable.
	 * Note:
	 * Make sure skip the first 16 system exception vectors.
	 */
	uint32_t *RamVectorTable_INT = (uint32_t *)(SCB->VTOR + 16 * 4);

	for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
		if (RamVectorTable_INT[irq] != (uint32_t)_isr_wrapper) {
			if (NVIC_GetEnableIRQ(irq) == 1) {
				NVIC_DisableIRQ(irq);
				z_isr_install(irq, (void *)RamVectorTable_INT[irq], NULL);
				NVIC_EnableIRQ(irq);
			} else {
				z_isr_install(irq, (void *)RamVectorTable_INT[irq], NULL);
			}
			RamVectorTableUpdate(irq + 16, (IRQ_Fun)_isr_wrapper);
		}
	}

	/*
	 * The ISRs for WDT_IRQn, RXI300_IRQn, and RXI300_SEC_IRQn are registered
	 * before entering the Zephyr. Therefore, we need to specifically
	 * register these ISRs in Zephyr's sw_isr_table.
	 */
	IRQ_CONNECT(WDT_IRQn, 0, HardFault_Handler_Rom, NULL, 0);
	IRQ_CONNECT(RXI300_SEC_IRQn, 0, HardFault_Handler_Rom, NULL, 0);
	IRQ_CONNECT(RXI300_IRQn, 0, HardFault_Handler_Rom, NULL, 0);
}

void soc_early_init_hook(void)
{
	rtl87x2g_extra_ram_init();

	/* Workaround for RamVectorTableUpdate called within phy_init() to direct update vector table. */
	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
	SCB->VTOR = (uint32_t)S_RAM_VECTOR_ADDR;
	(void)memcpy((void *)S_RAM_VECTOR_ADDR, _vector_start, vector_size);

	/* Init osif module with Zephyr.*/
	os_zephyr_patch_init();

	/* Essential for clock, phy, thermal and pm.*/
	os_queue_func_init();

	/* Configure Memory Attritube through MPU. */
	mpu_setup();

	/* Set the active mode clk src. */
	/* !!bt mac rely on */
	set_active_mode_clk_src();

	/* RTK-PMU related initialization */
	si_flow_data_init();

	/* FT parameters apply */
	ft_paras_apply();

	pmu_apply_voltage_tune();

	pmu_power_on_sequence_restart();

	/* RXI300 init */
	hal_setup_hardware();

	/* DWT init & FPU init */
	hal_setup_cpu();

	/*
	 * Initialize secure OS function pointers based on TrustZone configuration:
	 *
	 * Case 1: TrustZone Enabled & Zephyr runs in Non-Secure World.
	 *   - Scenario: Secure World needs to call a function in Non-Secure World.
	 *   - Action: Must use cmse_nsfptr_create() to create a valid Non-Secure entry handle.
	 *   - Ref: https://developer.arm.com/documentation/100720/0200/CMSE-support
	 *
	 * Case 2: TrustZone Disabled OR (TrustZone Enabled & Zephyr runs in Secure World).
	 *   - Scenario: Caller and Callee are in the same domain (or no TrustZone exists).
	 *   - Action: No special handling required. Direct function pointer assignment.
	 */
	secure_os_func_ptr_init_rom();

	/* Create a mutex for HW aes, and register NS function pointers
	 * (hw_aes_take_sem and hw_aes_give_sem).
	 */
	hw_aes_mutex_init();

	/* Setup 32k clk src */
	set_up_32k_clk_src(); /* use osif mem api */
	set_lp_module_clk_info();

	/* RTK-PM Initialization */
	/* !!bt mac rely on this */
	os_register_pm_excluded_handle = os_register_pm_excluded_handle_imp;
	os_unregister_pm_excluded_handle = os_unregister_pm_excluded_handle_imp;
	platform_rtc_aon_init();
	power_manager_master_init();
	power_manager_slave_init();
	platform_pm_init();

	/* Dynamic Voltage Frequency Scaling initialization */
	dvfs_init();

#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
	/* Set certain interrupts to be generated in NS mode. */
	setup_non_secure_nvic();
#endif
}

void soc_late_init_hook(void)
{
	/* Init OSC32 SDM fw-k sw timer. */
	init_osc_sdm_timer(); /* use osif timer api */

	/* PHY initialization */
	phy_hw_control_init(false);
	/* Relay on RamVectorTableUpdate(TMETER_VECTORn, imp_patch_thermal_meter_handler)
	 * Vector Table must be on RAM if we want RamVectorTableUpdate works, otherwise will trigger hardfault. */
	phy_init(false); /* use osif timer api */

	/* Temperature compensation-related initialization */
	thermal_tracking_timer_init();

	rtl87x2g_bt_controller_init();

	rtl87x2g_isr_register();
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT
void arch_busy_wait(uint32_t usec_to_wait)
{
	platform_delay_us(usec_to_wait);
}
#endif

/* Overrides the weak ARM implementation */
void sys_arch_reboot(int type)
{
	/* Convert SYS_REBOOT_WARM (0) to RESET_ALL_EXCEPT_AON (1).
	 * Convert SYS_REBOOT_COLD (1) to RESET_ALL (0).
	 */
	int wdt_mode = (type == SYS_REBOOT_WARM) ? RESET_ALL_EXCEPT_AON : RESET_ALL;

	/* Call the watchdog system reset with the converted mode and reset reason. */
	WDG_SystemReset(wdt_mode, RESET_REASON_ZEPHYR);
}
