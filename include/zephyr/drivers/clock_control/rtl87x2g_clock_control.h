/*
 * Copyright(c) 2019, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RTL87X2G_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RTL87X2G_H_

#include <zephyr/device.h>

/**
 * @brief Obtain a reference to the RTL87X2G clock controller.
 *
 * There is a single clock controller in the RTL87X2G: cctl. The device can be
 * used without checking for it to be ready since it has no initialization
 * code subject to failures.
 */
#define RTL87X2G_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(cctl))

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RTL87X2G_H_ */
