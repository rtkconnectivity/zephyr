/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/barrier.h>
#include <soc.h>

#include "rom_api_for_zephyr.h"
#include "os_sched.h"
#include "patch_header_check.h"
#include "vector_table.h"
#include "flash_nor_device.h"
#include "mem_config.h"
#include "utils.h"
#include "aon_reg.h"
#include "os_pm.h"
#include "trace.h"

extern void os_zephyr_patch_init(void);
extern void WDG_SystemReset(int wdt_mode, int reset_reason);

extern void z_arm_nmi(void);
extern void _isr_wrapper(void);
extern void z_arm_svc(void);

static int rtl87x2g_task_init(void)
{
	BOOL_PATCH_FUNC lowerstack_entry;
	T_ROM_HEADER_FORMAT *stack_header = (T_ROM_HEADER_FORMAT *)STACK_ROM_ADDRESS;

	if (memcmp(stack_header->uuid, nonsecure_rom_header.uuid, UUID_SIZE) == 0) {
		lowerstack_entry = (BOOL_PATCH_FUNC)((uint32_t)stack_header->entry_ptr);
		printk("Successfully loaded Realtek Lowerstack ROM!\n");
		lowerstack_entry();
	} else {
		printk("Failed to load Realtek Lowerstack ROM!\n");
	}

	AON_REG_WRITE_BITFIELD(AON_NS_REG0X_FW_GENERAL_NS, km4_pon_boot_done, 1);

	return 0;
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
			z_isr_install(irq, (void *)RamVectorTable_INT[irq], NULL);
			RamVectorTableUpdate(irq+16, (IRQ_Fun)_isr_wrapper);
		}
	}
/*
 * The ISRs for WDT_IRQn, RXI300_IRQn, and RXI300_SEC_IRQn are registered
 * before entering the Zephyr. Therefore, we need to specifically
 * register these ISRs in Zephyr's sw_isr_table.
 */
	z_isr_install(WDT_IRQn, (void *)HardFault_Handler_Rom, NULL);
	z_isr_install(RXI300_IRQn, (void *)HardFault_Handler_Rom, NULL);
	z_isr_install(RXI300_SEC_IRQn, (void *)HardFault_Handler_Rom, NULL);

/*
 * SVC_VECTORn and NMI_VECTORn are the only two exception vectors
 * that need specific updates back to zephyr's version.
 */
	RamVectorTableUpdate(SVC_VECTORn, (IRQ_Fun)z_arm_svc);
	RamVectorTableUpdate(NMI_VECTORn, (IRQ_Fun)z_arm_nmi);
}

static int rtl87x2g_platform_init(void)
{
/*
 * RTL87X2G reserves a RAM region for the vector table, referred to as the RamVectorTable.
 * Steps to initialize the vector table in RAM:
 * 1. Set the SCB->VTOR register to point to the start address of the RamVectorTable.
 * 2. Copy Zephyr's vector table to the RamVectorTable.
 */
	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
#if (CONFIG_TRUSTED_EXECUTION_NONSECURE == 1)
	/* tz enabled */
	SCB->VTOR = (uint32_t)NS_RAM_VECTOR_ADDR;
	(void)memcpy((void *)NS_RAM_VECTOR_ADDR, _vector_start, vector_size);
#else
	/* tz disabled */
	SCB->VTOR = (uint32_t)S_RAM_VECTOR_ADDR;
	(void)memcpy((void *)S_RAM_VECTOR_ADDR, _vector_start, vector_size);
#endif
	/* Init osif module with Zephyr.*/

	os_zephyr_patch_init();

	/* Init heap using Zephyr heap APIs. */
	os_init();

    /* Init essential APIs related to OS for RTK PM. */
	os_pm_init();

	/* TZ enabled: for “Non-secure function call”.
	 * Init non-secure function pointer that will be called by secure side using
	 * cmse_nsfptr_create().
	 * Example: RTK FLASH APIs in secure side would call os_lock() which is a non-secure function.
	 * Sample code: nonsecure_os_lock = (NS_UINT32_PATCH_FUNC)cmse_nsfptr_create(func);
	 * Link: https://developer.arm.com/documentation/100720/0200/CMSE-support

	 * TZ disabled: no special process, just a common function pointer assignment.
	 * Sample code: secure_os_lock = (UINT32_PATCH_FUNC)((uint32_t)func | 0x1);
	 */  
	secure_os_func_ptr_init();

	/* Function same as secure_os_func_ptr_init. But the non-secure function pointer
	 * is write_info_to_flash_before_reset that is used in WDG_SystemReset_Dump
	 * (secure function).
	 */
	secure_platform_func_ptr_init();

	/* Configure Memory Attritube through MPU.
	 * Refer to boot_cfg.common.mpu_region[8].
	 */
	mpu_setup();

	/* Set the active mode clk src. */
	set_active_mode_clk_src();

	/* Init RTK buffer log system. */
	log_module_trace_init(NULL);
	log_buffer_init();
	log_gdma_init();
	/* RTK-PMU related initialization */
	si_flow_data_init();

	/* FT parameters apply */
	ft_paras_apply();

	/* Setting RTK-PMU volatages */
	pmu_apply_voltage_tune();

	/* RTK PM: use km4_aon_boot_done to distinguish between normal boot
	 * and waking up from Power Down mode.
	 */
	bool aon_boot_done = AON_REG_READ_BITFIELD(AON_NS_REG0X_FW_GENERAL_NS, km4_aon_boot_done);

	if (!aon_boot_done) {
		pmu_power_on_sequence_restart();
	} else {
		si_flow_after_exit_low_power_mode();
		pmu_pm_exit();
	}
	AON_REG_WRITE_BITFIELD(AON_NS_REG0X_FW_GENERAL_NS, km4_aon_boot_done, 1);

	/* RXI300 init*/
	hal_setup_hardware();

	/* dwt init, mpu setup(again), init FPU */
	hal_setup_cpu();

	/* Create a mutex for HW aes, and register NS function pointers
	 * (hw_aes_take_sem and hw_aes_give_sem).
	 */
	hw_aes_mutex_init();

	/* Setup 32k clk src */
	set_up_32k_clk_src();/* use osif mem api */
	set_lp_module_clk_info();

	/* RTK-PM Initialization */
	platform_rtc_aon_init();
	power_manager_master_init();
	power_manager_slave_init();
	platform_pm_init();

	/* Init OSC32 SDM fw-k sw timer. */
	init_osc_sdm_timer();/* use osif timer api */

	/* Dynamic Voltage Frequency Scaling initialization */
	dvfs_init();

	/* PHY initialization */
	phy_hw_control_init(false);
	phy_init(false);/* use osif timer api */
	/* Temperature compensation-related initialization */
	thermal_tracking_timer_init();

	/* Flash init, can be moved to flash driver. */
	if (flash_nor_get_exist_nsc(FLASH_NOR_IDX_SPIC0)) {
		flash_nor_dump_main_info();
		flash_nor_cmd_list_init_nsc();
		flash_nor_init_bp_lv_nsc();
	}

#if (CONFIG_TRUSTED_EXECUTION_NONSECURE == 1)
	/* Set certain interrupts to be generated in NS mode.*/
	setup_non_secure_nvic();
#endif

	rtl87x2g_task_init();
	rtl87x2g_isr_register();

	return 0;
}

static int rtl87x2g_update_systick_config(void)
{
/* rtl87x2g's cortex-m systick timer is using external clock source
 * instead of cpu clock as referance.
 * The priority of systick interrupt is lowest for rtl87x2g SoCs.
 */
	NVIC_SetPriority(SysTick_IRQn, 0xff);
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;

	return 0;
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
	WDG_SystemReset(0, type);
}

SYS_INIT(rtl87x2g_platform_init, EARLY, 0);
SYS_INIT(rtl87x2g_update_systick_config, PRE_KERNEL_2, 1);
