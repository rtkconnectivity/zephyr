/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Realtek Bee series Specific Pinctrl Data Structures
 *
 * This file defines the pinctrl state structure and initialization macros
 * used by the Zephyr pinctrl driver for the Realtek Bee series.
 */

#ifndef ZEPHYR_SOC_REALTEK_BEE_PINCTRL_SOC_H_
#define ZEPHYR_SOC_REALTEK_BEE_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <zephyr/dt-bindings/pinctrl/rtl87x2g-pinctrl.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <zephyr/dt-bindings/pinctrl/rtl8752h-pinctrl.h>
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#include <zephyr/dt-bindings/pinctrl/rtl87x2j-pinctrl.h>
#else
#error "Unsupported Realtek Bee SoC series"
#endif

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Realtek Bee series pin configuration structure.
 *
 * This structure holds the decoded configuration for a single pin,
 * using bitfields to optimize storage.
 */
struct pinctrl_soc_pin {
	/* Word 0 (32 bits) */
	uint32_t pin: 11;     /**< Pin number (bit[0:10]) */
	uint32_t pull_dis: 1; /**< Bias disable (bit[11]) */
	uint32_t pull_dir: 1; /**< Bias pull direction: 1 for Pull-up, 0 for Pull-down (bit[12]) */
	uint32_t drive: 1;    /**< Initial output level: 1 for High, 0 for Low (bit[13]) */
	uint32_t dir: 1;      /**< Output enable/Direction: 1 for Output, 0 for Input (bit[14]) */
	uint32_t pull_strength: 1; /**< Pull strength: 1 for Strong, 0 for Weak (bit[15]) */
	uint32_t fun: 16;          /**< Pinmux function index (bit[16:31]) */

	/* Word 1 (Partial) */
	uint32_t current_level: 2; /**< Drive current level (bit[32:33]) */
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	uint32_t sleep_hardware_state: 1; /**< Sleep hardware state (bit[34]) */
	uint32_t wakeup_high: 1;          /**< High level wakeup (bit[35]) */
	uint32_t wakeup_low: 1;           /**< Low level wakeup (bit[36]) */
#endif
};

/**
 * @brief Typedef for the pinctrl soc pin structure.
 */
typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize a pinctrl_soc_pin object from Devicetree.
 *
 * @param node_id The Devicetree node identifier.
 * @param prop The property name (usually 'pinctrl-N').
 * @param idx The index in the property array.
 */
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
#define PINCTRL_SLEEP_MODE_CONFIG(node_id)                                                         \
	.sleep_hardware_state = DT_PROP_OR(node_id, sleep_hardware_state, 0),                      \
	.wakeup_high = DT_PROP_OR(node_id, wakeup_high, 0),                                        \
	.wakeup_low = DT_PROP_OR(node_id, wakeup_low, 0),
#else
#define PINCTRL_SLEEP_MODE_CONFIG(node_id)
#endif

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{.pin = BEE_GET_PIN(DT_PROP_BY_IDX(node_id, prop, idx)),                                   \
	 .fun = BEE_GET_FUN(DT_PROP_BY_IDX(node_id, prop, idx)),                                   \
	 .pull_dis = DT_PROP_OR(node_id, bias_disable, 0),                                         \
	 .pull_dir = DT_PROP_OR(node_id, bias_pull_up, 0),                                         \
	 .drive = DT_PROP_OR(node_id, output_high, 0),                                             \
	 .dir = DT_PROP_OR(node_id, output_enable, 0),                                             \
	 .pull_strength = DT_PROP_OR(node_id, bias_pull_strong, 0),                                \
	 .current_level = DT_PROP_OR(node_id, current_level, 0),                                   \
	 PINCTRL_SLEEP_MODE_CONFIG(node_id)},

/**
 * @brief Utility macro to initialize a list of pinctrl_soc_pin objects.
 *
 * @param node_id The Devicetree node identifier.
 * @param prop The property name.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, psels,            \
				Z_PINCTRL_STATE_PIN_INIT)}

/**
 * @brief Extract the Function ID from the pinctrl specifier.
 */
#define BEE_GET_FUN(pincfg) (((pincfg) >> BEE_FUN_POS) & BEE_FUN_MSK)

/**
 * @brief Extract the Pin ID from the pinctrl specifier.
 */
#define BEE_GET_PIN(pincfg) (((pincfg) >> BEE_PIN_POS) & BEE_PIN_MSK)

/**
 * @name Realtek Bee Wakeup Configuration
 * @{
 */

/** @brief Wakeup type */
enum pinctrl_bee_wakeup_type {
	PINCTRL_BEE_WAKEUP_SYS = 0, /**< System wakeup */
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	PINCTRL_BEE_WAKEUP_PPU, /**< PPU wakeup */
#endif
};

/**
 * @brief Configure or disable wakeup on a pin.
 *
 * @param pin Pin number.
 * @param polarity Wakeup polarity: 0 for low level, 1 for high level.
 * @param type Wakeup type: PINCTRL_BEE_WAKEUP_SYS or PINCTRL_BEE_WAKEUP_PPU.
 * @param enable True to enable wakeup, false to disable.
 */
void pinctrl_bee_wakeup_config(uint8_t pin, uint8_t polarity, uint8_t type, bool enable);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_REALTEK_BEE_PINCTRL_SOC_H_ */
