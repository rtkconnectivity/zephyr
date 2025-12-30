/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_REALTEK_BEE_RTL87X2G_PINCTRL_SOC_H_
#define ZEPHYR_SOC_REALTEK_BEE_RTL87X2G_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/rtl87x2g-pinctrl.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	/* bit[0:10]   pad number
	 * bit[11:12]  pad pull level
	 * bit[13]     pad output level
	 * bit[14]     pad direction
	 * bit[15]     pad pull strength
	 * bit[16:31]  pad pinmux function
	 * bit[32]     pad wakeup high level
	 * bit[33]     pad wakeup low level
	 * bit[34:36]  pad current level
	 */
	uint32_t pin: 11;
	uint32_t pull: 2;
	uint32_t drive: 1;
	uint32_t dir: 1;
	uint32_t pull_strength: 1;
	uint32_t fun: 16;
	uint32_t wakeup_high: 1;
	uint32_t wakeup_low: 1;
	uint32_t current_level: 2;
} pinctrl_soc_pin;

typedef pinctrl_soc_pin pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                         \
	{                                                                    \
		.pin = BEE_GET_PIN(DT_PROP_BY_IDX(node_id, prop, idx)),      \
		.pull = BEE_GET_PULL(DT_PROP_BY_IDX(node_id, prop, idx)),    \
		.drive = BEE_GET_DRIVE(DT_PROP_BY_IDX(node_id, prop, idx)),  \
		.dir = BEE_GET_DIR(DT_PROP_BY_IDX(node_id, prop, idx)),      \
		.pull_strength = DT_PROP_OR(node_id, bias_pull_strong, 0),         \
		.fun = BEE_GET_FUN(DT_PROP_BY_IDX(node_id, prop, idx)),      \
		.wakeup_high = DT_PROP_OR(node_id, wakeup_high, 0),                \
		.wakeup_low = DT_PROP_OR(node_id, wakeup_low, 0),                  \
		.current_level = DT_PROP_OR(node_id, current_level, 0),                  \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, psels,            \
				Z_PINCTRL_STATE_PIN_INIT)}

#define BEE_GET_FUN(pincfg) (((pincfg) >> BEE_FUN_POS) & BEE_FUN_MSK)
#define BEE_GET_DIR(pincfg) (((pincfg) >> BEE_DIR_POS) & BEE_DIR_MSK)
#define BEE_GET_DRIVE(pincfg) (((pincfg) >> BEE_DRIVE_POS) & BEE_DRIVE_MSK)
#define BEE_GET_PULL(pincfg) (((pincfg) >> BEE_PULL_POS) & BEE_PULL_MSK)
#define BEE_GET_PIN(pincfg) (((pincfg) >> BEE_PIN_POS) & BEE_PIN_MSK)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_REALTEK_BEE_RTL87X2G_PINCTRL_SOC_H_ */
