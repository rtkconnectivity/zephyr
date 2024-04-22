/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/pm/pm.h>
#include <stdbool.h>
#include <zephyr/logging/log.h>
/* private kernel APIs */
#include <ksched.h>
#include <kswap.h>
#include <wait_q.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

void z_pm_save_idle_exit(void)
{
#ifdef CONFIG_PM
	/* Some CPU low power states require notification at the ISR
	 * to allow any operations that needs to be done before kernel
	 * switches task or processes nested interrupts.
	 * This can be simply ignored if not required.
	 */
	pm_system_resume();
#endif	/* CONFIG_PM */
#ifdef CONFIG_SYS_CLOCK_EXISTS
	sys_clock_idle_exit();
#endif
}

void idle(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	__ASSERT_NO_MSG(_current->base.prio >= 0);

	__enable_irq();//sync with realtek pm flow

	while (true) {
		/* SMP systems without a working IPI can't actual
		 * enter an idle state, because they can't be notified
		 * of scheduler changes (i.e. threads they should
		 * run).  They just spin instead, with a minimal
		 * relaxation loop to prevent hammering the scheduler
		 * lock and/or timer driver.  This is intended as a
		 * fallback configuration for new platform bringup.
		 */
		if (IS_ENABLED(CONFIG_SMP) && !IS_ENABLED(CONFIG_SCHED_IPI_SUPPORTED)) {
			for (volatile int i = 0; i < 100000; i++) {
				/* Empty loop */
			}
			z_swap_unlocked();
		}

		/* Note weird API: k_cpu_idle() is called with local
		 * CPU interrupts masked, and returns with them
		 * unmasked.  It does not take a spinlock or other
		 * higher level construct.
		 */
		(void) arch_irq_lock();

		extern void (*thermal_meter_read)(void);
		thermal_meter_read();

		extern void log_buffer_trigger_schedule_in_km4_idle_task(void);
		log_buffer_trigger_schedule_in_km4_idle_task();

#ifdef CONFIG_PM
        arch_irq_unlock(0);// unlock irq to make sure realtek pmu flow work normally.

		extern void (*power_manager_slave_inact_action_handler)(void);
		power_manager_slave_inact_action_handler();
#else
		k_cpu_idle();
#endif

#if !defined(CONFIG_PREEMPT_ENABLED)
# if !defined(CONFIG_USE_SWITCH) || defined(CONFIG_SPARC)
		/* A legacy mess: the idle thread is by definition
		 * preemptible as far as the modern scheduler is
		 * concerned, but older platforms use
		 * CONFIG_PREEMPT_ENABLED=n as an optimization hint
		 * that interrupt exit always returns to the
		 * interrupted context.  So in that setup we need to
		 * explicitly yield in the idle thread otherwise
		 * nothing else will run once it starts.
		 */
		if (_kernel.ready_q.cache != _current) {
			z_swap_unlocked();
		}
# endif
#endif
	}
}

void __weak arch_spin_relax(void)
{
	__ASSERT(!arch_irq_unlocked(arch_irq_lock()),
		 "this is meant to be called with IRQs disabled");

	arch_nop();
}
