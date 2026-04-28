/*
 * Copyright(c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/kernel.h>

#include "soc.h"

#include "rtl_gtc.h"
#include "rtl_grtc.h"
#include "rtl_rcc.h"
#include "utils.h"

#define COUNTER_MAX   (0xffffffffUL)
#define TIMER_STOPPED (0xff000000UL)

#define SYS_TIMER_GRTC_CHANNEL GRTC_COMP0
#define SYS_TIMER_GRTC_INT     GRTC_INT_COMP0
#define SYS_TIMER_IRQ          GRTC_CH0_IRQn

#define OVERFLOW_TIMER_GRTC_CHANNEL GRTC_COMP1
#define OVERFLOW_TIMER_GRTC_INT     GRTC_INT_COMP1
#define OVERFLOW_TIMER_IRQ          GRTC_CH1_IRQn

#define CYC_PER_TICK (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#define MAX_TICKS  ((COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)

/*
 * Minimum delay required between consecutive GRTC_SetCompValue calls.
 * This is a hardware requirement for the Realtek GRTC: the compare register
 * needs at least ~1.5 clock cycles (~47us at 32kHz) to properly latch the
 * new value. Without this delay, the interrupt may not trigger reliably
 * when setting a new timeout in quick succession.
 */
#define GRTC_COMP_SET_DELAY_US 47

#define MIN_DELAY MAX(3, (CYC_PER_TICK / 16))

/*
 * This local variable holds the amount of GTC HW cycles elapsed.
 *
 */
static uint64_t last_count;
static uint32_t gtc_overflow_cnt;

/* add overflowed cycle.*/
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

void gtc_overflow_isr(void)
{
	GRTC_ClearINTPendingBit(OVERFLOW_TIMER_GRTC_INT);

	uint32_t key = irq_lock();

	gtc_overflow_cnt++;
	irq_unlock(key);
}

void sys_timer_isr(void *arg)
{
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

	uint64_t now = get_gtc_counter_unlocked();
	uint32_t adj, cyc = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary. */
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

	/*
	 * Hardware workaround: ensure minimum 1.5T interval between consecutive
	 * GRTC_SetCompValue calls. The GRTC compare register requires ~1.5 clock
	 * cycles (~47us at 32kHz) to properly latch the new value. Without this
	 * delay, the interrupt may not trigger reliably when setting a new timeout
	 * in quick succession.
	 */
	platform_delay_us(GRTC_COMP_SET_DELAY_US);

	GRTC_SetCompValue(SYS_TIMER_GRTC_CHANNEL, cyc + last_count);

#endif
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
	IRQ_CONNECT(OVERFLOW_TIMER_IRQ, 6, gtc_overflow_isr, 0, 0);
	irq_enable(OVERFLOW_TIMER_IRQ);

	RCC_ClockCmd(GRTC_CLOCK, ENABLE);
	platform_delay_us(32);

	GRTC_SetCompValue(OVERFLOW_TIMER_GRTC_CHANNEL, 0xFFFFFFFF);
	GRTC_CompReloadCmd(OVERFLOW_TIMER_GRTC_CHANNEL, DISABLE);
	GRTC_INTConfig(OVERFLOW_TIMER_GRTC_INT, ENABLE);
	platform_delay_us(63);

	IRQ_CONNECT(SYS_TIMER_IRQ, 0, sys_timer_isr, 0, 0);
	irq_enable(SYS_TIMER_IRQ);

	last_count = get_gtc_counter();

	RCC_ClockCmd(GRTC_CLOCK, ENABLE);
	platform_delay_us(32);

#if (CONFIG_TICKLESS_KERNEL == 1)
	GRTC_CompReloadCmd(SYS_TIMER_GRTC_CHANNEL, DISABLE);
	GRTC_SetCompValue(SYS_TIMER_GRTC_CHANNEL, 0xFFFFFFFF);
#else
	GRTC_SetCompReloadValue(SYS_TIMER_GRTC_CHANNEL, CYC_PER_TICK);
	GRTC_SetCompValue(SYS_TIMER_GRTC_CHANNEL, last_count + CYC_PER_TICK);
	GRTC_CompReloadCmd(SYS_TIMER_GRTC_CHANNEL, ENABLE);
#endif

	GRTC_INTConfig(SYS_TIMER_GRTC_INT, ENABLE);
	platform_delay_us(63);

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
