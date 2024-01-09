/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_REALTEK_RTL87X2G_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_REALTEK_RTL87X2G_GPIO_H_

#define RTL87X2G_GPIO_INPUT_DEBOUNCE_MS_POS      8
#define RTL87X2G_GPIO_INPUT_DEBOUNCE_MS_MASK     (0xff << RTL87X2G_GPIO_INPUT_DEBOUNCE_MS_POS)

/**
 * @brief Enable GPIO pin debounce.
 *
 * The debounce flag is a Zephyr specific extension of the standard GPIO flags
 * specified by the Linux GPIO binding. Only applicable for Realtek rtl87x2g SoCs.
 */
#define RTL87X2G_GPIO_INPUT_DEBOUNCE_MS(ms)    ((0xff & ms) << RTL87X2G_GPIO_INPUT_DEBOUNCE_MS_POS)



#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_REALTEK_RTL87X2G_GPIO_H_ */
