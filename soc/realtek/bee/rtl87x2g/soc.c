/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include "rom_api_for_zephyr.h"
#include "utils.h"
#include "os_sched.h"
extern void os_zephyr_patch_init(void);
extern void WDG_SystemReset(int wdt_mode, int reset_reason);

#include "trace.h"
static int rtl87x2g_platform_init(void)
{
	os_zephyr_patch_init();

	os_init();

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
	secure_platform_func_ptr_init();

    /* Configure Memory Attritube through MPU.
	 * Refer to boot_cfg.common.mpu_region[8]. */
	mpu_setup();

	/* Set the active mode clk src. */
	set_active_mode_clk_src();

	/* RXI300 init*/
	hal_setup_hardware();

	/* Setup 32k clk src */
	set_up_32k_clk_src();/* use osif mem api */
	set_lp_module_clk_info();

	init_osc_sdm_timer();/* use osif timer api */

	/* init DVFS()*/
	dvfs_init();

/* Will update TMETER ISR in Patch. Comment it temporarily.*/
	// phy_hw_control_init(false);
	// phy_init(false);/* use osif timer api */
	// thermal_tracking_timer_init();

	return 0;
}

static int rtl87x2g_update_systick_config(void)
{
	/* rtl87x2g's cortex-m systick timer is using external clock source instead of cpu clock as referance.
	 * The priority of systick interrupt is lowest for rtl87x2g SoCs. */
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
