/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <soc.h>
#include "trace.h"
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>
#include "rtl8752h_rom_defines.h"
#include "mem_config.h"
#include "mem_types.h"
#include "os_sched.h"
#include "os_sync.h"
#include "os_timer.h"
#include "os_pm.h"
#include "os_cfg.h"
#include "platform_cfg.h"
#include "rtl876x_aon_reg.h"
#include "rom_uuid.h"
/* T_ROM_HEADER_FORMAT */
#include "patch_header_check.h"
#include "patch.h"
#include "vector_table.h"
#include "pmu_manager.h"
#include "clock_manager.h"

#include "power_manager_interface.h"
#include "power_manager_master.h"
#include "power_manager_slave.h"
#include "power_manager_unit_platform.h"

#include "pingpong_buffer.h"
#include "system_rtl876x_int.h"
#include "log_uart_dma.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);
extern bool if_os_init_done;

enum {
	START_TO_RUN_C_CODE = 3,
	AFTER_CHECK_PAD_BOOT_FROM_FLASH_I = 4,
	AFTER_LOAD_PATCH = 5,
	AFTER_SET_SECURE_REG = 6,
};
extern void btaon_fast_update_8b(uint16_t offset, uint8_t mask, uint8_t data);
#define BOOT_STAGE_RECORD(x) btaon_fast_update(AON_FAST_REG_REG0X_FW_GENERAL, 1 << (x), 1 << (x))

extern void share_cache_ram(void);
extern bool check_hci_mode_flag(void);
extern void set_hci_mode_flag(bool enable);
extern void log_buffer_optimise_enable(void);
extern bool hw_aes_create_mutex(void);
extern void (*phy_hw_control_init)(bool dlps_flow);
extern void (*phy_init)(uint8_t dlps_flow);
extern void os_zephyr_patch_init(void);
extern void report_cache_info(void);

extern void z_arm_nmi(void);
extern void _isr_wrapper(void);
extern void z_arm_svc(void);

typedef void (*ISR_HANDLER)(const void *);

static void restore_isr_registered_before_zephyr(void)
{
	VECTORn_Type vector_n = System_VECTORn;
	IRQn_Type irqn;
	ISR_HANDLER *RamVectorTable = (ISR_HANDLER *)DATA_RAM_START_ADDR;
	ISR_HANDLER isr_handler;

	for (; vector_n <= UART2_VECTORn; ++vector_n) {
		isr_handler = RamVectorTable[(uint32_t)vector_n];
		if (RamVectorTable[(uint32_t)vector_n] != (ISR_HANDLER)ROM_Default_Handler) {
			irqn = vector_n - 16;
			if (irq_is_enabled(irqn)) {
				irq_disable(irqn);
				z_isr_install(irqn, isr_handler, NULL);
				irq_enable(irqn);
			} else {
				z_isr_install(irqn, isr_handler, NULL);
			}
			DBG_DIRECT("Restore ISR registered before zephyr: restore vector_n:%d "
				   "isr_addr:%x",
				   vector_n, isr_handler);
		}
	}

	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;

	memcpy((void *)DATA_RAM_START_ADDR, _vector_start, vector_size);
	SCB->VTOR = (uint32_t)DATA_RAM_START_ADDR;
}

static void restore_isr_registered_in_zephyr(void)
{
	VECTORn_Type vector_n = System_VECTORn;
	IRQn_Type irqn;
	ISR_HANDLER *RamVectorTable = (ISR_HANDLER *)DATA_RAM_START_ADDR;
	ISR_HANDLER isr_handler;

	for (; vector_n <= UART2_VECTORn; ++vector_n) {
		isr_handler = RamVectorTable[(uint32_t)vector_n];
		if (isr_handler != (ISR_HANDLER)_isr_wrapper) {
			irqn = vector_n - 16;
			if (irq_is_enabled(irqn)) {
				irq_disable(irqn);
				if (_sw_isr_table[irqn].isr != isr_handler) {
					z_isr_install(irqn, isr_handler, NULL);
				}
				RamVectorTableUpdate(vector_n, (IRQ_Fun)_isr_wrapper);
				irq_enable(irqn);
			} else {
				if (_sw_isr_table[irqn].isr != isr_handler) {
					z_isr_install(irqn, isr_handler, NULL);
				}
				RamVectorTableUpdate(vector_n, (IRQ_Fun)_isr_wrapper);
			}
			DBG_DIRECT("Restore ISR registered in SYS_INIT: vector_n:%d irqn:%d "
				   "isr_addr:%x",
				   vector_n, irqn, isr_handler);
		}
	}

	RamVectorTableUpdate(SVC_VECTORn, (IRQ_Fun)z_arm_svc);
	RamVectorTableUpdate(NMI_VECTORn, (IRQ_Fun)z_arm_nmi);
}

#ifdef CONFIG_BT
static int rtk_task_init(void)
{
	char c_rom_uuid[16] = DEFINE_symboltable_uuid;
	BOOL_PATCH_FUNC lowerstack_entry;
	T_ROM_HEADER_FORMAT *stack_header = (T_ROM_HEADER_FORMAT *)STACK_ROM_ADDRESS;

	if (memcmp(stack_header->uuid, c_rom_uuid, UUID_SIZE) == 0) {
		lowerstack_entry = (BOOL_PATCH_FUNC)((uint32_t)stack_header->entry_ptr);
		printk("Successfully loaded Realtek Lowerstack ROM!\n");
		lowerstack_entry();
	} else {
		printk("Failed to load Realtek Lowerstack ROM!\n");
	}
	return 0;
}
#endif

static int rtk_platform_init_stage_1(void)
{
	restore_isr_registered_before_zephyr();

	/* osif */
	os_zephyr_patch_init();
	os_init();

	if_os_init_done = true;

	BOOT_STAGE_RECORD(AFTER_LOAD_PATCH);

	if (os_cfg.wdgEnableInRom) {
		extern void enable_wdg_in_rom(void);
		enable_wdg_in_rom();
	}

	si_flow_data_init();

	ft_paras_apply();

	/* Config log module and level, init pointer trace_mask */
	log_module_trace_init(NULL);
	if (sys_init_cfg.logDisable == 0) {
		/* PingPong Buffer Init */
		PPB_Init(pMCU_PPB);
		/* Init Log UART channel */
		LOGUARTDriverInit();
		/* Init Log Uart DMA */
		LogUartDMAInit();
	}

	log_buffer_optimise_enable();
	/* enable cache */
	share_cache_ram();

	/* boot_error_code_print(); */
	set_active_mode_clk_src();

	pmu_apply_voltage_tune();

	BOOT_STAGE_RECORD(AFTER_SET_SECURE_REG);

	set_up_32k_clk_src();

	work_around_32k_power_glitch();

	AON_FAST_REG_REG0X_FW_GENERAL_TYPE aon_fast_boot = {
		.d16 = btaon_fast_read(AON_FAST_REG_REG0X_FW_GENERAL)};
	bool aon_boot_done = aon_fast_boot.aon_boot_done;
	if (!aon_boot_done) {
		pmu_power_on_sequence_restart();

		DBG_DIRECT("rtl8752h ROM version: %s %s", __DATE__, __TIME__);

		/* Pad_ClearAllWakeupINT(); */
	} else {
		/* power management exit */
		DBG_DIRECT("%s...", "si_flow_after_exit_low_power_mode");
		si_flow_after_exit_low_power_mode();

		pmu_pm_exit();
	}

	si_flow_after_power_on_sequence_restart();

	work_around_32k_power_glitch_after_restart();

	AON_FAST_REG_REG0X_FW_GENERAL_TYPE aon_fast_reg_0x0 = {
		.d16 = btaon_fast_read(AON_FAST_REG_REG0X_FW_GENERAL)};

	aon_fast_reg_0x0.aon_boot_done = 1;
	btaon_fast_write(AON_FAST_REG_REG0X_FW_GENERAL, aon_fast_reg_0x0.d16);

	/* ProgramStart */
	/* setlocale(LC_ALL, "C"); */
	hal_setup_hardware();

	hal_setup_cpu();

	if (check_hci_mode_flag()) {
		/* clear otp_upper.stack_en flag */
		sys_init_cfg.stack_en = 0;
		/* clear hci_mode flag */
		set_hci_mode_flag(false);
		BOOT_PRINT_WARN0("Switch to HCI Mode\n");
	}

	return 0;
}

int rtk_platform_init_stage_2(void)
{
	platform_rtc_aon_init();

	/* power management init */
	power_manager_master_init();
	power_manager_slave_init();
	platform_pm_init();

	init_osc_sdm_timer();

	phy_hw_control_init(false);
	phy_init(false);

#ifdef CONFIG_BT
	rtk_task_init();
#endif

	restore_isr_registered_in_zephyr();

	hw_aes_create_mutex();

	AON_FAST_REG_REG0X_FW_GENERAL_TYPE aon_fast_reg_0x0 = {
		.d16 = btaon_fast_read(AON_FAST_REG_REG0X_FW_GENERAL)};

	aon_fast_reg_0x0.d16 = btaon_fast_read(AON_FAST_REG_REG0X_FW_GENERAL);
	aon_fast_reg_0x0.pon_boot_done = 1;
	btaon_fast_write(AON_FAST_REG_REG0X_FW_GENERAL, aon_fast_reg_0x0.d16);

	return 0;
}

static int rtk_register_update(void)
{
	extern uint32_t SystemCpuClock;

	DBG_DIRECT("SystemCpuClock:%x", SystemCpuClock);
#ifdef CONFIG_SYSTICK_USE_EXTERNAL_CLOCK
#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC != 32000)
#error "CONFIG_SYSTICK_USE_EXTERNAL_CLOCK does not match CONFIG_SYS_CLOCK_TICKS_PER_SEC"
#endif
	/* Selects the SysTick timer clock source: external 32768 */
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
#else
#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC != 40000000)
#error "CPU Clock Rate does not match CONFIG_SYS_CLOCK_TICKS_PER_SEC"
#endif
#endif /* CONFIG_SYSTICK_USE_EXTERNAL_CLOCK */

	return 0;
}

SYS_INIT(rtk_platform_init_stage_1, EARLY, 0);
SYS_INIT(rtk_register_update, PRE_KERNEL_2, 1);
SYS_INIT(rtk_platform_init_stage_2, PRE_KERNEL_2, 2);
