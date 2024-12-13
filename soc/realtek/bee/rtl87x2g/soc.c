/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>

static int rtl87x2g_update_systick_config(void)
{
	/* rtl87x2g's cortex-m systick timer is using external clock source instead of cpu clock
	 * as referance. */
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;

	return 0;
}

SYS_INIT(rtl87x2g_update_systick_config, PRE_KERNEL_2, 1);
