/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BEE_PM_DEVICE_PM_TEST_H_
#define BEE_PM_DEVICE_PM_TEST_H_

#ifdef CONFIG_GPIO
#include <zephyr/drivers/gpio.h>

/* GPIO lines the tests drive or watch. The pin shell can move them to other
 * pads, so the tests read them at run time.
 */
extern struct gpio_dt_spec gpio_test_out;
extern struct gpio_dt_spec gpio_test_in;

#if defined(CONFIG_SENSOR) && defined(CONFIG_QDEC_BEE)
extern struct gpio_dt_spec phase_a;
extern struct gpio_dt_spec phase_b;
#endif
#endif /* CONFIG_GPIO */

#endif /* BEE_PM_DEVICE_PM_TEST_H_ */
