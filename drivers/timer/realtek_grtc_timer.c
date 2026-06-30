/*
 * Copyright(c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/timer/system_timer.h>

#include "soc.h"

#include "rtl_gtc.h"
#include "rtl_grtc.h"
#include "rtl_rcc.h"
#include "utils.h"

#define SYS_TIMER_GRTC_CHANNEL GRTC_COMP0
#define SYS_TIMER_GRTC_INT     GRTC_INT_COMP0
#define SYS_TIMER_IRQ          GRTC_CH0_IRQn

#define OVERFLOW_TIMER_GRTC_CHANNEL GRTC_COMP1
#define OVERFLOW_TIMER_GRTC_INT     GRTC_INT_COMP1
#define OVERFLOW_TIMER_IRQ          GRTC_CH1_IRQn

#define COUNTER_MAX   (0xffffffffUL)
#define TIMER_STOPPED (0xff000000UL)

#define CYC_PER_TICK (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#define MAX_TICKS  ((COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)

/* HW latch (GRTC_COMP_MIN_GAP_CYCLES = 2) + 1 cycle margin for CPU path */
#define MIN_DELAY (GRTC_COMP_MIN_GAP_CYCLES + 1)

/*
 * Minimum interval required between consecutive GRTC_SetCompValue() calls.
 *
 * Hardware requirement:
 * The Realtek GRTC compare register needs at least ~1.5 clock cycles
 * to latch the new value properly.
 *
 * At 32 kHz:
 *
 *   1 cycle = 31.25 us
 *   1.5 cycles ~= 47 us
 *
 * Therefore, wait at least 2 GTC cycles before programming the next compare
 * value. Without this delay, the interrupt may not trigger reliably when a new
 * timeout is programmed too soon after the previous compare update.
 */
#define GRTC_COMP_MIN_GAP_CYCLES 2
/* -------------------------------------------------------------------------- */
/* Local state                                                                */
/* -------------------------------------------------------------------------- */

/*
 * last_count stores the last announced kernel tick boundary in GTC HW cycles.
 */
static uint64_t last_count;

/*
 * Software high 32-bit overflow counter for extending GTC_GetCounter()
 * to 64-bit.
 */
static uint32_t gtc_overflow_cnt;

/*
 * Last GTC cycle count at which GRTC_SetCompValue() was called.
 * Used to satisfy the GRTC compare latch timing requirement.
 */
static uint64_t last_comp_set_count;

static inline uint64_t get_gtc_counter_unlocked(void)
{
	return ((uint64_t)gtc_overflow_cnt << 32) + (uint64_t)GTC_GetCounter();
}

static uint64_t get_gtc_counter(void)
{
	uint32_t key = irq_lock();
	uint64_t ret = get_gtc_counter_unlocked();

	irq_unlock(key);

	return ret;
}

static inline void grtc_wait_comp_gap(void)
{
	/*
	 * Hardware workaround:
	 *
	 * Ensure a minimum 1.5T interval between consecutive
	 * GRTC_SetCompValue() calls.
	 *
	 * The compare register requires about 1.5 GRTC clock cycles
	 * to latch the new value correctly.
	 */
	while ((get_gtc_counter_unlocked() - last_comp_set_count) < GRTC_COMP_MIN_GAP_CYCLES) {
		/* Wait until GRTC compare register can latch next value. */
	}
}

void gtc_overflow_isr(void)
{
	GRTC_ClearINTPendingBit(OVERFLOW_TIMER_GRTC_INT);

	uint32_t key = irq_lock();

	gtc_overflow_cnt++;

	irq_unlock(key);
}

void sys_timer_isr(void *arg)
{
	ARG_UNUSED(arg);

	GRTC_ClearINTPendingBit(SYS_TIMER_GRTC_INT);

	uint32_t key = irq_lock();

	uint64_t dticks = (get_gtc_counter_unlocked() - last_count) / CYC_PER_TICK;

	last_count += dticks * CYC_PER_TICK;

	irq_unlock(key);

	sys_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? dticks : 1);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL)

	ticks = ticks == K_TICKS_FOREVER ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	/*
	 * Only wait when the previous GRTC_SetCompValue() was too recent.
	 * This avoids paying a fixed delay on every timeout programming.
	 */
	grtc_wait_comp_gap();

	uint32_t key = irq_lock();

	/*
	 * Re-check inside the critical section because another context may
	 * have programmed the compare value just before irq_lock().
	 */
	grtc_wait_comp_gap();

	uint64_t now = get_gtc_counter_unlocked();
	uint32_t adj;
	uint32_t cyc = ticks * CYC_PER_TICK;

	/*
	 * Round up to the next tick boundary.
	 */
	adj = (uint32_t)(now - last_count) + (CYC_PER_TICK - 1);

	if (cyc <= MAX_CYCLES - adj) {
		cyc += adj;
	} else {
		cyc = MAX_CYCLES;
	}

	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;

	if ((int32_t)(cyc + last_count - now) < MIN_DELAY) {
		cyc += CYC_PER_TICK;
	}

	GRTC_SetCompValue(SYS_TIMER_GRTC_CHANNEL, cyc + last_count);

	last_comp_set_count = get_gtc_counter_unlocked();

	irq_unlock(key);

#endif /* CONFIG_TICKLESS_KERNEL */
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	uint32_t key = irq_lock();

	uint32_t ret = ((uint32_t)get_gtc_counter_unlocked() - (uint32_t)last_count) / CYC_PER_TICK;

	irq_unlock(key);

	return ret;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return (uint32_t)get_gtc_counter();
}

uint64_t sys_clock_cycle_get_64(void)
{
	return get_gtc_counter();
}

void sys_clock_disable(void)
{
	irq_disable(SYS_TIMER_IRQ);
}

int sys_clock_driver_init(void)
{
	RCC_ClockCmd(GRTC_CLOCK, ENABLE);

	/* Configure overflow compare channel. */
	IRQ_CONNECT(OVERFLOW_TIMER_IRQ, IRQ_PRIO_LOWEST, gtc_overflow_isr, 0, 0);
	irq_enable(OVERFLOW_TIMER_IRQ);

	GRTC_SetCompValue(OVERFLOW_TIMER_GRTC_CHANNEL, 0xFFFFFFFF);
	GRTC_CompReloadCmd(OVERFLOW_TIMER_GRTC_CHANNEL, DISABLE);
	GRTC_INTConfig(OVERFLOW_TIMER_GRTC_INT, ENABLE);

	/* Configure system timer compare channel. */
	IRQ_CONNECT(SYS_TIMER_IRQ, 0, sys_timer_isr, 0, 0);
	irq_enable(SYS_TIMER_IRQ);

	last_count = get_gtc_counter();

#if (CONFIG_TICKLESS_KERNEL == 1)
	GRTC_CompReloadCmd(SYS_TIMER_GRTC_CHANNEL, DISABLE);
	GRTC_SetCompValue(SYS_TIMER_GRTC_CHANNEL, 0xFFFFFFFF);
#else
	GRTC_SetCompReloadValue(SYS_TIMER_GRTC_CHANNEL, CYC_PER_TICK);
	GRTC_SetCompValue(SYS_TIMER_GRTC_CHANNEL, last_count + CYC_PER_TICK);
	GRTC_CompReloadCmd(SYS_TIMER_GRTC_CHANNEL, ENABLE);
#endif /* CONFIG_TICKLESS_KERNEL */

	GRTC_INTConfig(SYS_TIMER_GRTC_INT, ENABLE);

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
