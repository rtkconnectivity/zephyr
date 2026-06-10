/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/time_units.h>

#include <pck600.h>
#include <bitops.h>
#ifdef CONFIG_BT
#include <utils.h>
#endif

#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_DECLARE(soc);

#ifdef CONFIG_BT
/* Opcodes for lowerstack platform syscall interface */
#define SYSCALL_OPCODE_SET_MAC_POWER_MODE  40

/* BT MAC power modes */
#define BT_POWER_DEEP_SLEEP  0
#define BT_POWER_ACTIVE      1
#endif /* CONFIG_BT */

#ifdef CONFIG_PM_DEBUG_SNAPSHOT
#include <pck600_snapshot.h>

static const char *const pm_snapshot_ip_names[] = {
	"I2S",        "ADC",        "UART3",      "UART2",      "UART1",      "UART0",
	"TMETER",     "TIMER1_CH8", "TIMER1_CH7", "TIMER1_CH6", "TIMER1_CH5", "TIMER1_CH4",
	"TIMER1_CH3", "TIMER1_CH2", "TIMER1_CH1", "TIMER1_CH0", "TIMER0_CH1", "TIMER0_CH0",
	"SPI3W",      "SPI2",       "SPI1",       "SPI0",       "KEYSCAN",    "IR",
	"I2C1",       "I2C0",       "GPIO",       "DMA",        "TIMER2_CH0", "CAN",
	"USB",        "RFC",        "PRO24G",     "PCC",        "MODEM",      "CAL80M",
	"CAL32K",     "CAL10M",     "BTMAC",      "BT_VEN",     "AUDIO",      "AACK",
	"AES",        "SHA256",     "CPU",        "SPIC0",      "RNG",        "VDREG",
};

static uint32_t pm_wfi_count;

static void pm_snapshot_log_active_ips(void)
{
	int key = irq_lock();
	uint64_t mask = pck600_snapshot_get_device_mask();

	for (int i = 0; i < PCK600_SNAPSHOT_DEVICE_MAX; i++) {
		if (mask & ((uint64_t)1 << i)) {
			PCK600_SNAPSHOT_DEVICE_T dev = (PCK600_SNAPSHOT_DEVICE_T)i;
			PCK600_SNAPSHOT_STATUS_T status = pck600_snapshot_get_device_status(dev);

			LOG_INF("active IP: %s (bit %d) qactive=%d",
				pm_snapshot_ip_names[i], i, status);
		}
	}

	pck600_snapshot_clear_all_device_status();
	irq_unlock(key);
}

static void pm_snapshot_report_handler(struct k_work *work)
{
	bool lp_overflow = false;
	uint32_t lp_count = pck600_system_get_wakeup_count(&lp_overflow);

	LOG_INF("PM snapshot: wfi=%u  lp_enter=%u%s",
		pm_wfi_count, lp_count, lp_overflow ? " (overflow)" : "");

	pm_snapshot_log_active_ips();
}

static K_WORK_DEFINE(pm_snapshot_work, pm_snapshot_report_handler);

static void pm_snapshot_timer_handler(struct k_timer *timer)
{
	k_work_submit(&pm_snapshot_work);
}

static K_TIMER_DEFINE(pm_snapshot_timer, pm_snapshot_timer_handler, NULL);

#endif /* CONFIG_PM_DEBUG_SNAPSHOT */

static inline void pm_prepare_power_gating(void)
{
	SCB->CPACR &= ~(0xFU << 20);
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__WFI();
}

static inline void pm_power_on_sequence(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	SCB->CPACR |= ((3U << (10U * 2U)) | (3U << (11U * 2U)));
	SCnSCB->CPPWR &= ~(BIT20 | BIT22);
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	__disable_irq();
	irq_unlock(0);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		pm_prepare_power_gating();
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		return;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
#ifdef CONFIG_PM_DEBUG_SNAPSHOT
		pm_wfi_count++;
#endif
		pm_power_on_sequence();
		break;
	default:
		break;
	}
	/*
	 * Timing requirements (do not change the order of the following three steps):
	 * 1) Unlock the scheduler first, then enable interrupts.
	 *    Reason: The BT MAC module is extremely latency-sensitive. When handling
	 *    connection events, the lower stack task must be scheduled immediately
	 *    after the BT MAC ISR exits. If we do not unlock the scheduler before
	 *    enabling interrupts, the interrupt may fire, but thread switching cannot
	 *    happen immediately after the ISR, which can cause the connection event to fail.
	 */
	k_sched_unlock();
	/* 2) Re-enable global interrupts (clear ARM PRIMASK). */
	__enable_irq();
	/*
	 * 3) Lock the scheduler again to match the behavior in pm_system_suspend().
	 *    This keeps the critical section on the resume path consistent and helps
	 *    avoid unexpected preemption.
	 */
	k_sched_lock();
}

static int rtl87x2j_system_pm_init(void)
{
	pck600_system_set_dynamic_power_policy(POWER_POLICY_SYSTEM_ON_LOW_POWER);

#ifdef CONFIG_PM_DEBUG_SNAPSHOT
	k_timer_start(&pm_snapshot_timer, K_MSEC(3000), K_MSEC(3000));
#endif

	return 0;
}
SYS_INIT(rtl87x2j_system_pm_init, POST_KERNEL, 0);

#ifdef CONFIG_BT
static int rtl87x2j_bt_controller_pm_init(void)
{
	lowerstack_SystemCall_in_platform(SYSCALL_OPCODE_SET_MAC_POWER_MODE,
					  BT_POWER_DEEP_SLEEP, 0, 0);

	return 0;
}
SYS_INIT(rtl87x2j_bt_controller_pm_init, APPLICATION, 0);
#endif
