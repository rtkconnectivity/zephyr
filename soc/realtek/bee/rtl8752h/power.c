/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include <cmsis_core.h>
#include <dlps.h>
#include <trace.h>
#include <os_pm.h>
#include <rtl876x_pinmux.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);
#define REALTEK_POWER_LOG 0

/* ROM Extern Variables and Functions */
/* #include <power_manager_unit_platform.h> */
extern void (*platform_pm_register_callback_func_with_priority)(void *cb_func,
								PlatformPMStage pf_pm_stage,
								int8_t priority);

extern void NMI_Handler(void);
extern void sys_clock_announce_process_timeout(void);
extern void sys_clock_restore_tick_and_cycle(void);

#if REALTEK_POWER_LOG
#define POWER_LOG(...) DBG_DIRECT(__VA_ARGS__)
#else
#define POWER_LOG(...)                                                                             \
	do {                                                                                       \
	} while (0)
#endif

volatile uint32_t CPU_StoreReg[3]; /*  This array should be placed in RAM ON/Buffer ON.    */
volatile uint32_t CPU_StoreReg_IP[8];
volatile uint32_t PeriIntStoreReg;
/**
 * In RTK ROM code, SCB-VTOR is set to VTOR_RAM_ADDR
 * during dlps exiting, which is not same as _vector_start in zephyr.
 * So, it should be restored right now,
 * rather than restore it at a delayable work.
 */
static void CPU_DLPS_Enter(void)
{
	POWER_LOG("%s is called", __func__);

	/* store NVIC registers */
	CPU_StoreReg[0] = NVIC->ISER[0];
	CPU_StoreReg[1] = NVIC->ISPR[0];

	for (uint8_t i = 0; i < 8; ++i) {
		CPU_StoreReg_IP[i] = NVIC->IP[i];
	}

	/* store VTOR */
	CPU_StoreReg[2] = SCB->VTOR;
	POWER_LOG("store SCB->VTOR=%x", SCB->VTOR);

	/* store Vendor register */
	PeriIntStoreReg = PERIPHINT->EN;
}

static void CPU_DLPS_Exit(void)
{
	POWER_LOG("%s is called", __func__);

	/* restore NVIC registers */
	/* Don't restore NVIC pending register, but report warning */
	/* NVIC->ISPR[0] = CPU_StoreReg[1]; */
	if (CPU_StoreReg[0] & CPU_StoreReg[1]) {
		/* During enter and exit dlps, system will disable all interrupts.
		 * If any interrupt occurs during this period, this log will be printed.
		 * Every bit of pending register corresponds to an interrupt. Please refer
		 * to IRQn_Type from System_IRQn  * to UART2_IRQn.
		 * For example:  "miss interrupt: pending register: 0x42000"
		 * It means that RTC and ADC interrupt occur during dlps store and restore flow.
		 * But because all  * interrupts are masked, these interrupts are pending.
		 */
		POWER_LOG("miss interrupt: pending register: 0x%x", CPU_StoreReg[1]);
	}
	/* skip restore the priority of System_IRQn(#0) and BTMAC_IRQn(#2) */
	NVIC->IP[0] |= CPU_StoreReg_IP[0] & 0xFF00FF00;
	for (uint8_t i = 1; i < 8; ++i) {
		NVIC->IP[i] = CPU_StoreReg_IP[i];
	}

	/* restore VTOR */
	SCB->VTOR = CPU_StoreReg[2];
	POWER_LOG("restore SCB->VTOR=%x", SCB->VTOR);

	/* restore Vendor register */
	PERIPHINT->EN = PeriIntStoreReg;

	NVIC->ISER[0] = CPU_StoreReg[0];
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}

TYPE_SECTION_START_EXTERN(const struct device *, pm_device_slots);
void System_Handler(const void *param)
{
	irq_disable(System_IRQn);
	POWER_LOG("System_Handler");

	NVIC_ClearPendingIRQ(System_IRQn);
}

/* Number of devices successfully suspended. */
static size_t num_susp_rtk;

static int pm_suspend_devices_rtk(void)
{
	Pad_ClearAllWakeupINT();
	CPU_DLPS_Enter();

	/* Realtek PM Device flow */
	irq_disable(System_IRQn);
	/* common flow */
	const struct device *devs;
	size_t devc;

	devc = z_device_get_all_static(&devs);

	num_susp_rtk = 0;

	for (const struct device *dev = devs; dev < devs + devc; dev++) {
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
	/* Realtek PM Device flow */
	irq_enable(System_IRQn);
	/* common flow */
	for (int i = (num_susp_rtk - 1); i >= 0; i--) {
		pm_device_action_run(TYPE_SECTION_START(pm_device_slots)[i],
				     PM_DEVICE_ACTION_RESUME);
	}

	num_susp_rtk = 0;

	CPU_DLPS_Exit();
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

	/* Subtract the pended tick from the timeout list and manually trigger a timeout process.*/
	sys_clock_announce_process_timeout();
}

/* Initialize power system */
static int rtl8752h_power_init(void)
{
	int ret = 0;

	os_pm_init();

#ifdef CONFIG_PM_DEVICE
	irq_connect_dynamic(System_IRQn, 1, System_Handler, NULL, 0);
	irq_enable(System_IRQn);
#endif

	lps_mode_set(PLATFORM_DLPS_PFM);
	z_arm_nmi_set_handler(NMI_Handler);

	LOG_INF("set pm exit_stage_time from %d to %d",
		platform_pm_system.stage_time[PLATFORM_PM_EXIT], 13);

	platform_pm_system.stage_time[PLATFORM_PM_EXIT] = 13;

	platform_pm_register_callback_func_with_priority((void *)pm_suspend_devices_rtk,
							 PLATFORM_PM_STORE, 1);
	platform_pm_register_callback_func_with_priority((void *)pm_resume_devices_rtk,
							 PLATFORM_PM_PEND, -1);
	platform_pm_register_callback_func_with_priority(
		(void *)pm_reusme_systick_and_process_timeout, PLATFORM_PM_PEND, INT8_MAX);

	return ret;
}

/* do it after lowerstack entry */
SYS_INIT(rtl8752h_power_init, APPLICATION, 1);
