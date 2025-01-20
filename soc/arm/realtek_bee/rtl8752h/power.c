#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <cmsis_core.h>
#include <dlps.h>
/* #include <power_manager_unit_platform.h> */
extern void (*platform_pm_register_callback_func_with_priority)(void *, PlatformPMStage, int8_t);
#include <rtl876x_pinmux.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

uint32_t Pinmux_StoreReg[10]; /*  This array should be placed in RAM ON/Buffer ON.    */
extern void NMI_Handler(void);
extern void sys_clock_announce_process_timeout(void);
extern void z_sys_init_run_level(int level);

volatile uint32_t CPU_StoreReg[3]; /*  This array should be placed in RAM ON/Buffer ON.    */
volatile uint32_t CPU_StoreReg_IP[8];
volatile uint32_t PeriIntStoreReg;

#define REALTEK_POWER_DBG 1
#if REALTEK_POWER_DBG
#include "trace.h"
#define DBG_DIRECT_OPTIONAL(...) DBG_DIRECT(__VA_ARGS__)
static uint32_t wakeup_count;
static uint32_t last_wakeup_clk, last_sleep_clk;
#else
#define DBG_DIRECT_OPTIONAL(...)                                                                   \
	do {                                                                                       \
	} while (0)
#endif

/**
 * In RTK ROM code, SCB-VTOR is set to VTOR_RAM_ADDR
 * during dlps exiting, which is not same as _vector_start in zephyr.
 * So, it should be restored right now,
 * rather than restore it at a delayable work.
 */
static void CPU_DLPS_Enter(void)
{
	DBG_DIRECT("%s is called", __func__);
	/* NVIC store */
	uint32_t i;
	/* Interrupt Clear Enable Register */
	CPU_StoreReg[0] = NVIC->ISER[0];
	/* Interrupt Set Pending Register */
	CPU_StoreReg[1] = NVIC->ISPR[0];

	for (i = 0; i < 8; ++i) {
		CPU_StoreReg_IP[i] = NVIC->IP[i];
	}

	CPU_StoreReg[2] = SCB->VTOR;
	DBG_DIRECT("store SCB->VTOR=%x", SCB->VTOR);

	/* Save Vendor register */
	PeriIntStoreReg = PERIPHINT->EN;
}

static void CPU_DLPS_Exit(void)
{
	DBG_DIRECT("%s is called", __func__);
	/* NVIC restore */
	uint32_t i;

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
		DBG_DIRECT("miss interrupt: pending register: 0x%x", CPU_StoreReg[1]);
	}
	/* skip restore the priority of System_IRQn(#0) and BTMAC_IRQn(#2) */
	NVIC->IP[0] |= CPU_StoreReg_IP[0] & 0xFF00FF00;
	for (i = 1; i < 8; ++i) {
		NVIC->IP[i] = CPU_StoreReg_IP[i];
	}

	SCB->VTOR = CPU_StoreReg[2];
	DBG_DIRECT("restore SCB->VTOR=%x", SCB->VTOR);

	PERIPHINT->EN = PeriIntStoreReg;
	NVIC->ISER[0] = CPU_StoreReg[0];
}

static void Pinmux_DLPS_Enter(void)
{
	DBG_DIRECT("%s is called", __func__);
	uint8_t i = 0;

	for (i = 0; i < 10; i++) {
		Pinmux_StoreReg[i] = PINMUX->CFG[i];
	}
#ifndef CONFIG_PM_DEVICE
	irq_disable(System_IRQn);
	System_WakeUpPinEnable(P2_4, PAD_WAKEUP_POL_LOW, 0, 0);
#endif
}

static void Pinmux_DLPS_Exit(void)
{
	DBG_DIRECT("%s is called", __func__);
	uint8_t i;

	for (i = 0; i < 10; i++) {
		PINMUX->CFG[i] = Pinmux_StoreReg[i];
	}
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

/* porting from zephyr/subsys/pm/pm.c */
#ifdef CONFIG_PM_DEVICE
TYPE_SECTION_START_EXTERN(const struct device *, pm_device_slots);

#if !defined(CONFIG_PM_DEVICE_RUNTIME_EXCLUSIVE)
/* Number of devices successfully suspended. */
static size_t num_susp_rtk;

static int pm_suspend_devices_rtk(void)
{
	/* RTK dlps flow */
	NVIC_DisableIRQ(System_IRQn);
	/* zephyr common flow */
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
		    pm_device_state_is_locked(dev) || pm_device_wakeup_is_enabled(dev) ||
		    pm_device_runtime_is_enabled(dev)) {
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
	/* zephyr common flow */
	for (int i = (num_susp_rtk - 1); i >= 0; i--) {
		pm_device_action_run(TYPE_SECTION_START(pm_device_slots)[i],
				     PM_DEVICE_ACTION_RESUME);
	}

	num_susp_rtk = 0;
}
#endif /* !CONFIG_PM_DEVICE_RUNTIME_EXCLUSIVE */
#endif /* CONFIG_PM_DEVICE */

#define RTK_PM_WORKQ_STACK_SIZE 512
#define RTK_PM_WORKQ_PRIORITY   K_HIGHEST_THREAD_PRIO

K_THREAD_STACK_DEFINE(rtk_pm_workq_stack_area, RTK_PM_WORKQ_STACK_SIZE);

struct k_work_q rtk_pm_workq;
struct k_work work_timeout_process;
struct k_work work_device_resume;

void device_resume_handler(struct k_work *item)
{
#ifndef CONFIG_PM_DEVICE
	/* delay to workq (irq unlocked) */
	/* workaround for uart init(log) */
	z_sys_init_run_level(1);
	irq_enable(System_IRQn);
#endif

	platform_pm_get_statistics(&wakeup_count, &last_wakeup_clk, &last_sleep_clk);
	DBG_DIRECT("wakeup_count0x%x last_wakeup_clk0x%x last_sleep_clk0x%x wake reason 0x%x",
		   wakeup_count, last_wakeup_clk, last_sleep_clk, platform_pm_get_wakeup_reason());
}

void timeout_process_handler(struct k_work *item)
{
	sys_clock_announce_process_timeout();
}

/* do resume: call timeout_process_handler and device_resume_handler by submit k_work */
void submit_items_to_rtk_pm_workq(void)
{
	/*
	 * it can pend to workq after device PM okay
	 * k_work_submit_to_queue(&rtk_pm_workq, &work_device_resume);
	 */
	k_work_submit_to_queue(&rtk_pm_workq, &work_timeout_process);
}
#ifndef CONFIG_PM_DEVICE
/*ToDo: move to device PM? */
void System_Handler(void)
{
	NVIC_DisableIRQ(System_IRQn);
	DBG_DIRECT("System_Handler");

	if (System_WakeUpInterruptValue(P0_1) == SET) {
		DBG_DIRECT("P0_1");
		Pad_ClearWakeupINTPendingBit(P0_1);
		System_WakeUpPinDisable(P0_1);
		/* allowedSystemEnterDlps = false; */
	}
	if (System_WakeUpInterruptValue(P2_4) == SET) {
		DBG_DIRECT("P2_4");
		Pad_ClearWakeupINTPendingBit(P2_4);
		/* 避免重复触发 */
		System_WakeUpPinDisable(P2_4);
	}
	NVIC_ClearPendingIRQ(System_IRQn);
}
#endif

/* Initialize power system */
static int rtl87x2x_power_init(void)
{
	int ret = 0;
#ifndef CONFIG_PM_DEVICE
	irq_connect_dynamic(System_IRQn, 1, System_Handler, NULL, 0);
	irq_enable(System_IRQn);
#endif

	lps_mode_set(PLATFORM_DLPS_PFM);
	z_arm_nmi_set_handler(NMI_Handler);
	/* do devices & nvic resume in
	 * rtk_pm_workq thread via zephyr's workq
	 */
	k_work_queue_init(&rtk_pm_workq);
	k_work_queue_start(&rtk_pm_workq, rtk_pm_workq_stack_area,
			   K_THREAD_STACK_SIZEOF(rtk_pm_workq_stack_area), RTK_PM_WORKQ_PRIORITY,
			   NULL);
	k_work_init(&work_timeout_process, timeout_process_handler);
	k_work_init(&work_device_resume, device_resume_handler);

	/* register PM Store callback */
#ifdef CONFIG_PM_DEVICE
	platform_pm_register_callback_func_with_priority((void *)pm_suspend_devices_rtk,
							 PLATFORM_PM_STORE, 1);
#endif
	platform_pm_register_callback_func_with_priority((void *)CPU_DLPS_Enter, PLATFORM_PM_STORE,
							 1);
	platform_pm_register_callback_func_with_priority((void *)Pinmux_DLPS_Enter,
							 PLATFORM_PM_STORE, 1);
	/* register PM Restore callback */
	platform_pm_register_callback_func_with_priority((void *)CPU_DLPS_Exit, PLATFORM_PM_RESTORE,
							 1);
	platform_pm_register_callback_func_with_priority((void *)Pinmux_DLPS_Exit,
							 PLATFORM_PM_RESTORE, 1);
	platform_pm_register_callback_func_with_priority((void *)device_resume_handler,
							 PLATFORM_PM_RESTORE, 1);
#ifdef CONFIG_PM_DEVICE
	platform_pm_register_callback_func_with_priority((void *)pm_resume_devices_rtk,
							 PLATFORM_PM_RESTORE, 1);
#endif
	platform_pm_register_callback_func_with_priority((void *)submit_items_to_rtk_pm_workq,
							 PLATFORM_PM_RESTORE, 1);

	return ret;
}

/* do it after lowerstack entry */
SYS_INIT(rtl87x2x_power_init, POST_KERNEL, 1);
