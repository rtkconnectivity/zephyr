/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
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

struct pinctrl_soc_pin {
	/* bit[0:10]   pad number
	 * bit[11]     pad pull disable
	 * bit[12]     pad pull dir
	 * bit[13]     pad output level
	 * bit[14]     pad direction
	 * bit[15]     pad pull strength
	 * bit[16:31]  pad pinmux function
	 * bit[32:33]  reserved
	 * bit[34:36]  pad current level
	 */
	uint32_t pin: 11;
	uint32_t pull_dis: 1;
	uint32_t pull_dir: 1;
	uint32_t drive: 1;
	uint32_t dir: 1;
	uint32_t pull_strength: 1;
	uint32_t fun: 16;
	uint32_t reserved_32: 1;
	uint32_t reserved_33: 1;
	uint32_t current_level: 2;
};

typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                         \
	{                                                                    \
		.pin = BEE_GET_PIN(DT_PROP_BY_IDX(node_id, prop, idx)),      \
		.fun = BEE_GET_FUN(DT_PROP_BY_IDX(node_id, prop, idx)),      \
		.pull_dis = DT_PROP_OR(node_id, bias_disable, 0),    \
		.pull_dir = DT_PROP_OR(node_id, bias_pull_up, 0),    \
		.drive = DT_PROP_OR(node_id, output_high, 0),  \
		.dir = DT_PROP_OR(node_id, output_enable, 0),      \
		.pull_strength = DT_PROP_OR(node_id, bias_pull_strong, 0),         \
		.current_level = DT_PROP_OR(node_id, current_level, 0),                  \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, psels,            \
				Z_PINCTRL_STATE_PIN_INIT)}

#define BEE_GET_FUN(pincfg) (((pincfg) >> BEE_FUN_POS) & BEE_FUN_MSK)
#define BEE_GET_PIN(pincfg) (((pincfg) >> BEE_PIN_POS) & BEE_PIN_MSK)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_REALTEK_BEE_RTL87X2G_PINCTRL_SOC_H_ */
