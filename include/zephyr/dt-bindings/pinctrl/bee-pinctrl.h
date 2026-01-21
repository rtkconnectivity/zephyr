/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_

/** Position of the function field. */
#define BEE_FUN_POS   16U
/** Mask for the function field. */
#define BEE_FUN_MSK   0xFFFFU
/** Position of the pin field. */
#define BEE_PIN_POS   0U
/** Mask for the pin field. */
#define BEE_PIN_MSK   0x7FFU

#define BEE_PSEL(fun, pin)                                                       \
	(((((pin) & BEE_PIN_MSK) << BEE_PIN_POS) | (((BEE_##fun) & BEE_FUN_MSK) << BEE_FUN_POS)))

#define BEE_PSEL_DISCONNECTED(fun)                                                                 \
	(BEE_PIN_DISCONNECTED << BEE_PIN_POS | ((BEE_##fun & BEE_FUN_MSK) << BEE_FUN_POS))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_ */
