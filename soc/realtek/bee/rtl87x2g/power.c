/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/pm.h>

#include <zephyr/kernel_structs.h>
#include <zephyr/init.h>
#include <string.h>
#include <stdint.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include <zephyr/tracing/tracing.h>

#include <cmsis_core.h>

/* for platform_pm_register_callback_func_with_priority */
#include <power_manager_unit_platform.h>
#include <pm.h>
#include <rtl_pinmux.h>
#include "os_pm.h"
#include <trace.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rtl87x2g_pm, LOG_LEVEL_INF);

struct k_work work_timeout_process;
struct k_work work_device_resume;

TYPE_SECTION_START_EXTERN(const struct device *, pm_device_slots);

/* Number of devices successfully suspended. */
static size_t num_susp_rtk;

extern void NMI_Handler(void);
extern void sys_clock_announce_process_timeout(void);
extern void pad_short_pulse_wake_up(int Status);
extern void sys_clock_restore_tick_and_cycle(void);
extern void os_pm_restore_tickcount(void);

volatile uint32_t CPU_StoreReg[6];
volatile uint8_t CPU_StoreReg_IPR[96];
volatile uint32_t Peripheral_StoreReg[2];
static void CPU_DLPS_Enter(void)
{
	/* NVIC store */
	uint32_t i;

	CPU_StoreReg[0] = NVIC->ISER[0];
	CPU_StoreReg[1] = NVIC->ISER[1];
	CPU_StoreReg[2] = NVIC->ISER[2];

	CPU_StoreReg[3] = NVIC->ISPR[0];
	CPU_StoreReg[4] = NVIC->ISPR[1];
	CPU_StoreReg[5] = NVIC->ISPR[2];

	/* Skip System_IRQn, WDG_IRQn, RXI300_IRQn, RXI300_SEC_IRQn,
	 * Zigbee_IRQn which are handled in rom.
	 */
	const uint8_t *IPR_pt = (const uint8_t *)NVIC->IPR;

	for (i = 5; i < 96; ++i) {
		CPU_StoreReg_IPR[i] = IPR_pt[i];
	}

	/* peripheral reg store */
	Peripheral_StoreReg[0] = SoC_VENDOR->u_008.REG_LOW_PRI_INT_MODE;
	Peripheral_StoreReg[1] = SoC_VENDOR->u_00C.REG_LOW_PRI_INT_EN;
}

void CPU_DLPS_Exit(void)
{
	/* peripheral reg restore */
	SoC_VENDOR->u_008.REG_LOW_PRI_INT_MODE = Peripheral_StoreReg[0];
	SoC_VENDOR->u_00C.REG_LOW_PRI_INT_EN = Peripheral_StoreReg[1];

	/* NVIC restore */
	uint32_t i;

	/* During enter and exit dlps, system will disable all interrupts. If any interrupt
	 * occurs during this period, this log will be printed.
	 * Every bit of pending register corresponds to an interrupt. Please refer to IRQn_Type
	 * from System_IRQn to PF_RTC_IRQn.
	 * For example:  "miss interrupt: pending register: 0x100, 0x0 , 0x0"
	 * It means that RTC interrupt occur during dlps store and restore flow.
	 * But because all interrupts are masked, these interrupts are pending.
	 */
	if ((CPU_StoreReg[0] & CPU_StoreReg[3]) || (CPU_StoreReg[1] & CPU_StoreReg[4]) ||
	    (CPU_StoreReg[2] & CPU_StoreReg[5])) {
		LOG_ERR("miss interrupt: pending register: 0x%x, 0x%x, 0x%x", CPU_StoreReg[3],
			CPU_StoreReg[4], CPU_StoreReg[5]);
	}

	/* Skip System_IRQn, WDG_IRQn, RXI300_IRQn, RXI300_SEC_IRQn,
	 * Zigbee_IRQn which are handled in rom.
	 */
	uint8_t *IPR_pt = (uint8_t *)NVIC->IPR;

	for (i = 5; i < 96; ++i) {
		IPR_pt[i] = CPU_StoreReg_IPR[i];
	}

	NVIC->ISER[0] = CPU_StoreReg[0];
	NVIC->ISER[1] = CPU_StoreReg[1];
	NVIC->ISER[2] = CPU_StoreReg[2];
}

static int pm_suspend_devices_rtk(void)
{
	pad_short_pulse_wake_up(1);
	Pad_ClearAllWakeupINT();
	System_WakeupDebounceClear(0);
	CPU_DLPS_Enter();

	const struct device *devs;
	size_t devc;

	devc = z_device_get_all_static(&devs);

	num_susp_rtk = 0;

	for (const struct device *dev = devs + devc - 1; dev >= devs; dev--) {
		int ret;

		/* Ignore uninitialized devices, busy devices, wake up sources, and
		 * devices with runtime PM enabled.
		 */
		if (!device_is_ready(dev) || pm_device_is_busy(dev) ||
		    pm_device_wakeup_is_enabled(dev) || pm_device_runtime_is_enabled(dev)) {
			continue;
		}

		ret = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
		/* ignore devices not supporting or already at the given state */
		if ((ret == -ENOSYS) || (ret == -ENOTSUP) || (ret == -EALREADY)) {
			continue;
		} else if (ret < 0) {
			LOG_ERR("Device %s did not enter %s state (%d)", dev->name,
				pm_device_state_str(PM_DEVICE_STATE_SUSPENDED), ret);
			return ret;
		}

		TYPE_SECTION_START(pm_device_slots)[num_susp_rtk] = dev;
		num_susp_rtk++;
	}
	return 0;
}

void pm_resume_devices_rtk(void)
{
	for (int i = (num_susp_rtk - 1); i >= 0; i--) {
		pm_device_action_run(TYPE_SECTION_START(pm_device_slots)[i],
				     PM_DEVICE_ACTION_RESUME);
	}

	CPU_DLPS_Exit();

	num_susp_rtk = 0;
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}

void pm_reusme_systick_and_process_timeout(void)
{
	/* Restore systick after driver resume to ensure no systick isr is triggered and exclude
	 * timer is timeout out and executed. Restore the sys clock of Zephyr. Note: exclude timer
	 * cb may rely on the driver resume.
	 */
	__disable_irq();
	os_pm_restore_tickcount();
	sys_clock_restore_tick_and_cycle();
	__enable_irq();

	/* Subtract the pended tick from the timeout list and manually trigger a timeout process. */
	sys_clock_announce_process_timeout();
}

/* Initialize power system */
static int rtl87x2g_power_init(void)
{
	int ret = 0;

	/* Init essential APIs related to OS for RTK PM. */
	os_pm_init();

	bt_power_mode_set(BTPOWER_DEEP_SLEEP);
	power_mode_set(POWER_DLPS_MODE);

	z_arm_nmi_set_handler(NMI_Handler);

	platform_pm_register_callback_func_with_priority((void *)pm_suspend_devices_rtk,
							 PLATFORM_PM_STORE, 1);
	platform_pm_register_callback_func_with_priority((void *)pm_resume_devices_rtk,
							 PLATFORM_PM_PEND, -1);
	platform_pm_register_callback_func_with_priority(
		(void *)pm_reusme_systick_and_process_timeout, PLATFORM_PM_PEND, INT8_MAX);

	return ret;
}

/* do it after lowerstack entry */
SYS_INIT(rtl87x2g_power_init, POST_KERNEL, 1);
