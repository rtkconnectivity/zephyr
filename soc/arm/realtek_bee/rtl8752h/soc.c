#include <zephyr/kernel.h>
#include <string.h>
#include <soc.h>
#include "trace.h"
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/multi_heap.h>
#include "bee3plus_rom_defines.h"
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

extern bool if_os_init_done;

extern struct sys_multi_heap multi_heap;

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
extern uint8_t (*flash_nor_get_default_bp_lv)(void);
extern void flash_nor_dump_flash_info(void);
extern void os_zephyr_patch_init(void);
extern void report_cache_info(void);

/**
 * first stage vector(IRQn<=31),
 * not include such as FLASH_SEC_IRQn, WDT_IRQn
 */
static const IRQn_Type irq_restore_num[] = {
	BTMAC_IRQn,         BTMAC_WRAP_AROUND_IRQn, Timer4_5_IRQn,       ZIGBEE_IRQn,
	PF_RTC_IRQn,        Peripheral_IRQn,        GDMA0_Channel1_IRQn, GDMA0_Channel2_IRQn,
	GDMA0_Channel3_IRQn};

static uint32_t irq_restore_priority[sizeof(irq_restore_num) / sizeof(IRQn_Type)];

#if (CONFIG_PRINT_RTK_RAM_VECTOR_TABLE)
static void print_vtor_table(uint32_t *addr)
{
	VECTORn_Type vector_n = InitialSP_VECTORn;
	IRQn_Type irqn;
	uint32_t *RamVectorTable = addr;

	for (; vector_n <= WDT_VECTORn; ++vector_n) {
		if (vector_n <= UART2_VECTORn) {
			irqn = vector_n - 16;
			DBG_DIRECT("vector_n:%d irqn:%d isr_addr:%x", vector_n, irqn,
				   RamVectorTable[(uint32_t)vector_n]);
		} else {
			DBG_DIRECT("vector_n:%d irqn:multiplexed isr_addr:%x", vector_n,
				   RamVectorTable[(uint32_t)vector_n]);
		}
	}
}
#endif /* CONFIG_PRINT_RTK_RAM_VECTOR_TABLE */

static void rtk_load_irq_priority(void)
{
	uint32_t irqn;
	/* save selected irq priority from rom */
	for (int i = 0; i < sizeof(irq_restore_num) / sizeof(IRQn_Type); ++i) {
		irqn = irq_restore_num[i];
		/*RTK IRQn multiplexing */
		if (irqn >= Peripheral_First_IRQn) {
			irqn = Peripheral_IRQn;
		}
		irq_restore_priority[i] = NVIC_GetPriority(irqn);
	}
}
#ifdef CONFIG_PLATFORM_SPECIFIC_INIT
void z_arm_platform_init(void)
{
	DBG_DIRECT("%s...", __func__);

#if (CONFIG_PRINT_RTK_RAM_VECTOR_TABLE)
	print_vtor_table((uint32_t *)DATA_RAM_START_ADDR);
#endif

	rtk_load_irq_priority();
}
#endif

static void rtk_irq_restore_from_rom(void)
{
	uint32_t *RamVectorTable = (uint32_t *)DATA_RAM_START_ADDR;
	uint32_t vector_n;
	int irqn;
	/* set external irq from rtk rom project to isq_wrapper */
	for (int i = 0; i < sizeof(irq_restore_num) / sizeof(IRQn_Type); ++i) {
		irqn = irq_restore_num[i];
		/* RTK IRQn multiplexing */
		if (irqn >= Peripheral_First_IRQn) {
			irqn = Peripheral_IRQn;
		}
		vector_n = irqn + 16;
		/* rtk rom irq places vectors at RamVectorTable */
		if (RamVectorTable[vector_n] != (uint32_t)_isr_wrapper) {
			/* update zephyr irq dynamic */
			if (irq_is_enabled(irqn)) {
				irq_disable(irqn);
				irq_connect_dynamic(irqn, irq_restore_priority[i],
						    (void *)RamVectorTable[vector_n], NULL, 0);
				irq_enable(irqn);
			} else {
				irq_connect_dynamic(irqn, irq_restore_priority[i],
						    (void *)RamVectorTable[vector_n], NULL, 0);
			}
			DBG_DIRECT("restore vector_n:%d isr_addr:%x _isr_wrapper:%x", vector_n,
				   RamVectorTable[vector_n], _isr_wrapper);
#ifdef REALTEK_VTOR_RELOCATE
			/* update to rtk ram vector table */
			RamVectorTableUpdate(vector_n, (IRQ_Fun)_isr_wrapper);
#endif
			DBG_DIRECT("restore irqn:%d priority:%x", irqn, irq_restore_priority[i]);
		}
	}
}

static void rtk_irq_restore_and_relocate(void)
{
	/* only recover irq from rtk rom project, system exception not included */
	rtk_irq_restore_from_rom();
#ifdef REALTEK_VTOR_RELOCATE
	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
	/* copy *all* vector(exception + irq) to relocate */
	memcpy((void *)DATA_RAM_START_ADDR, _vector_start, vector_size);
	SCB->VTOR = (uint32_t)DATA_RAM_START_ADDR;
#endif
}

static int rtk_task_init(void)
{
	char c_rom_uuid[16] = DEFINE_symboltable_uuid;
	BOOL_PATCH_FUNC lowerstack_entry;
	T_ROM_HEADER_FORMAT *stack_header = (T_ROM_HEADER_FORMAT *)STACK_ROM_ADDRESS;

	if (memcmp(stack_header->uuid, c_rom_uuid, UUID_SIZE) == 0) {
		lowerstack_entry = (BOOL_PATCH_FUNC)((uint32_t)stack_header->entry_ptr);
		printk("Successfully loaded Realtek Lowerstack ROM!\n");
		lowerstack_entry();
		/* keep irq restore operation before lowstack_thread be scheduled */
		rtk_load_irq_priority();
		rtk_irq_restore_from_rom();
	} else {
		printk("Failed to load Realtek Lowerstack ROM!\n");
	}
	return 0;
}

static int rtk_platform_init(void)
{
	DBG_DIRECT("%s...", __func__);

	/* osif */
	os_zephyr_patch_init();
	os_init();

	/**
	 * SCB->VTOR points to zephyr's vector table which is placed in flash.
	 * However, vector table place in flash will trigger hardfault when flash erasing.
	 * So point SCB->VTOR to RAM vector table, and copy zephyr's vector table to RAM.
	 */
	rtk_irq_restore_and_relocate();

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

		DBG_DIRECT("Bee3Plus ROM version: %s %s", __DATE__, __TIME__);

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

	os_timer_init();
	if (check_hci_mode_flag()) {
		/* clear otp_upper.stack_en flag */
		sys_init_cfg.stack_en = 0;
		/* clear hci_mode flag */
		set_hci_mode_flag(false);
		BOOT_PRINT_WARN0("Switch to HCI Mode\n");
	}
	platform_rtc_aon_init();

	/* power management init */
	power_manager_master_init();
	power_manager_slave_init();
	platform_pm_init();
	os_pm_init();

	init_osc_sdm_timer();

	phy_hw_control_init(false);
	phy_init(false);

	rtk_task_init();

	if (flash_nor_get_exist(FLASH_NOR_IDX_SPIC0) != FLASH_NOR_EXIST_NONE) {
		if (flash_nor_load_query_info(FLASH_NOR_IDX_SPIC0) == FLASH_NOR_RET_SUCCESS) {
			/* apply SW Block Protect */
			if (boot_cfg.flash_setting.bp_enable) {
				/**
				 * set flash default block protect level depend on
				 * different flash id and different flash layout
				 */
				boot_cfg.flash_setting.bp_lv = flash_nor_get_default_bp_lv();
			} else {
				boot_cfg.flash_setting.bp_lv = 0;
			}

			if (flash_nor_set_tb_bit(FLASH_NOR_IDX_SPIC0, 1) == FLASH_NOR_RET_SUCCESS &&
			    flash_nor_set_bp_lv(FLASH_NOR_IDX_SPIC0,
						boot_cfg.flash_setting.bp_lv) ==
				    FLASH_NOR_RET_SUCCESS) {
				FLASH_PRINT_INFO1("Flash BP Lv = %d", boot_cfg.flash_setting.bp_lv);
			} else {
				FLASH_PRINT_INFO0("Flash BP fail!");
			}
		}
		flash_nor_dump_flash_info();
	}

	hw_aes_create_mutex();

	aon_fast_reg_0x0.d16 = btaon_fast_read(AON_FAST_REG_REG0X_FW_GENERAL);
	aon_fast_reg_0x0.pon_boot_done = 1;
	btaon_fast_write(AON_FAST_REG_REG0X_FW_GENERAL, aon_fast_reg_0x0.d16);

#if MEMORY_WATCH_EN
	/* dump heap and task stack usage. */
	extern void memory_watch_enable(void);
	memory_watch_enable();
#endif

	return 0;
}

static int rtk_register_update(void)
{
	extern uint32_t SystemCpuClock;

	DBG_DIRECT("SystemCpuClock:%x", SystemCpuClock);
#ifdef CONFIG_SYSTICK_USE_EXTERNAL_CLOCK
	/* Selects the SysTick timer clock source: external 32768 */
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
#endif
	return 0;
}

SYS_INIT(rtk_platform_init, EARLY, 0);
SYS_INIT(rtk_register_update, PRE_KERNEL_2, 1);
