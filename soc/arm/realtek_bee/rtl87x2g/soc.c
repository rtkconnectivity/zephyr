#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/barrier.h>
#include <soc.h>

#include "rom_api_for_zephyr.h"
#include "trace.h"
#include "os_sched.h"
#include "patch_header_check.h"
#include "vector_table.h"
#include "flash_nor_device.h"
#include "mem_config.h"
#include "utils.h"
#include "aon_reg.h"
#include "os_pm.h"

#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

extern void os_zephyr_patch_init(void);
extern void BTMAC_Handler(void);
extern void GDMA0_Channel9_Handler(void);
extern void PF_RTC_Handler(void);

extern void z_arm_nmi(void);
extern void _isr_wrapper(void);
extern void z_arm_svc(void);

extern const T_ROM_HEADER_FORMAT nonsecure_rom_header;

#define VECTOR_ADDRESS ((uintptr_t)_vector_start)

/*
 *  rtk_rom_irq_connect() calls IRQ_CONNECT to register ISRs (that are
 *  already registered in ROM) to zephyr's vector table/sw isr table. 
 */
void rtk_rom_irq_connect(void)
{
	IRQ_CONNECT(RXI300_IRQn, 0, HardFault_Handler_Rom, NULL, 0);
	IRQ_CONNECT(RXI300_SEC_IRQn, 0, HardFault_Handler_Rom, NULL, 0);
	IRQ_CONNECT(GDMA0_Channel9_IRQn, 6, GDMA0_Channel9_Handler, NULL, 0);
	IRQ_CONNECT(PF_RTC_IRQn, 0, PF_RTC_Handler, NULL, 0);
	IRQ_CONNECT(BTMAC_IRQn, 1, BTMAC_Handler, NULL, 0);
	IRQ_CONNECT(BTMAC_WRAP_AROUND_IRQn, 0, HardFault_Handler_Rom, NULL, 0);
	/* IRQ_CONNECT(Flash_SEC_IRQn, 5, Flash_SEC_Handler, NULL, 0); */
}

static int rtk_platform_init(void)
{
/*
 * Realtek has pre-planned RAM region for Vector Table(NS&S), so 
 * copy the Zephyr's vector table to here.
 */
	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
#if (CONFIG_TRUSTED_EXECUTION_NONSECURE==1)
	/* tz enabled */
	SCB->VTOR = (uint32_t)NS_RAM_VECTOR_ADDR;
	(void)memcpy((void *)NS_RAM_VECTOR_ADDR, _vector_start, vector_size);
#else
	/* tz disabled */
	SCB->VTOR = (uint32_t)S_RAM_VECTOR_ADDR;
	(void)memcpy((void *)S_RAM_VECTOR_ADDR, _vector_start, vector_size);
#endif

	/* connect rtk-rom-irq to zephyr's vector table, also will reset priority here!*/
	rtk_rom_irq_connect();

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
	RamVectorTableUpdate(SVC_VECTORn, (IRQ_Fun)z_arm_svc);

	/* Function same as secure_os_func_ptr_init. But the non-secure function pointer
	 * is write_info_to_flash_before_reset that is used in WDG_SystemReset_Dump(secure function). */
	secure_platform_func_ptr_init();

    /* Configure Memory Attritube through MPU.
	 * Refer to boot_cfg.common.mpu_region[8]. */
	mpu_setup();


	/* Set the active mode clk src. */
	set_active_mode_clk_src();

	/* Init RTK buffer log system. */
	log_module_trace_init(NULL);
	log_buffer_init();
	log_gdma_init();
	RamVectorTableUpdate(GDMA0_Channel9_VECTORn, (IRQ_Fun)_isr_wrapper);

	si_flow_data_init();

	/* FT parameters apply */
	ft_paras_apply();

	pmu_apply_voltage_tune();

	/* RTK PM: use km4_aon_boot_done to distinguish between normal boot and waking up from Power Down mode. */
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
	 * (hw_aes_take_sem and hw_aes_give_sem). */
	hw_aes_mutex_init();

	/* Setup 32k clk src */
	set_up_32k_clk_src();/* use osif mem api */
	set_lp_module_clk_info();

	/* RTK PM Initialization */
	platform_rtc_aon_init();
	power_manager_master_init();
	power_manager_slave_init();
	platform_pm_init();
	RamVectorTableUpdate(PF_RTC_VECTORn, (IRQ_Fun)_isr_wrapper);
	RamVectorTableUpdate(NMI_VECTORn, (IRQ_Fun)z_arm_nmi);

	/* Init OSC32 SDM fw-k sw timer. */
	init_osc_sdm_timer();/* use osif timer api */

	dvfs_init();

	phy_hw_control_init(false);
	phy_init(false);/* use osif timer api */

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

	return 0;
}

static int rtk_task_init(void)
{
	BOOL_PATCH_FUNC lowerstack_entry;
	T_ROM_HEADER_FORMAT *stack_header = (T_ROM_HEADER_FORMAT *)STACK_ROM_ADDRESS;

	if (memcmp(stack_header->uuid, nonsecure_rom_header.uuid, UUID_SIZE) == 0) {
		lowerstack_entry = (BOOL_PATCH_FUNC)((uint32_t)stack_header->entry_ptr);
		printk("Successfully loaded Realtek Lowerstack ROM!\n");
		lowerstack_entry();
		RamVectorTableUpdate(BTMAC_VECTORn, (IRQ_Fun)_isr_wrapper);
		RamVectorTableUpdate(BTMAC_WRAP_AROUND_VECTORn, (IRQ_Fun)_isr_wrapper);
	} else {
		printk("Failed to load Realtek Lowerstack ROM!\n");
	}

	/* RTK PM: use km4_pon_boot_done to distinguish between normal boot and
	 * waking up from DLPS mode. */
	AON_REG_WRITE_BITFIELD(AON_NS_REG0X_FW_GENERAL_NS, km4_pon_boot_done, 1);

	return 0;
}

static int rtk_register_update(void)
{
	NVIC_SetPriority(SysTick_IRQn, 0xff);
#ifdef CONFIG_SYSTICK_USE_EXTERNAL_CLOCK
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
#endif
/*
 * Unset SCB_CCR_DIV_0_TRP bit to avoid usagefault when dividing by zero.
 * If this bit is set, ll_iso_generate_cis_unframed_parameters_from_host_info()
 * in rom will trigger usage fault.
 * WARNING: In Freertos, this bit will not be set.
 */
	SCB->CCR &= ~SCB_CCR_DIV_0_TRP_Msk;
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
	extern void WDG_SystemReset(int wdt_mode, int reset_reason);
	WDG_SystemReset(0, type);
}

SYS_INIT(rtk_platform_init, EARLY, 0);
SYS_INIT(rtk_register_update, PRE_KERNEL_2, 1);
SYS_INIT(rtk_task_init, POST_KERNEL, 0);
