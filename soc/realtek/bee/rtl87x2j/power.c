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

#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_DECLARE(soc);

/**
 * @brief Implement the preparation flow for system-wide power gating
 */
#define portPM_PREPARE_FOR_POWER_GATING()                                                  \
	do {                                                                                   \
		SCB->CPACR &= ~((0xF) << 20);                                                      \
		CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;                                   \
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;                                                 \
		__WFI();                                                                           \
	} while (0)

#define portPM_POWER_ON_SEQUENCE()                                                         \
	do {                                                                                   \
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;                                    \
		SCB->CPACR |= ((3U << 10U * 2U) | (3U << 11U * 2U));                               \
		SCnSCB->CPPWR &= ~(BIT20 | BIT22);                                                 \
	} while (0)

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	/* Set PRIMASK */
	__disable_irq();
	/* Set BASEPRI to 0 */
	irq_unlock(0);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		portPM_PREPARE_FOR_POWER_GATING();
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		return;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	/* Set run mode config after wakeup */
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		portPM_POWER_ON_SEQUENCE();
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

/* Initialize power system */
static int rtl87x2j_power_init(void)
{
	int ret = 0;

	pck600_system_set_dynamic_power_policy(POWER_POLICY_SYSTEM_ON_LOW_POWER);

	return ret;
}

SYS_INIT(rtl87x2j_power_init, POST_KERNEL, 0);
