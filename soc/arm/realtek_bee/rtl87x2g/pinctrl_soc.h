/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * rtl87x2g SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_ARM_REALTEK_RTL_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_REALTEK_RTL_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/rtl87x2g-pinctrl.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

typedef struct {
        uint32_t pin : 11;
        uint32_t pull : 2;
        uint32_t drive : 1;
        uint32_t dir : 1;
        uint32_t pull_strength : 1;
        uint32_t fun : 16;
        uint32_t wakeup_high : 1;
        uint32_t wakeup_low : 1;
} pinctrl_soc_pin;

typedef pinctrl_soc_pin pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                   \
	{								\
		.pin = RTL87X2G_GET_PIN(DT_PROP_BY_IDX(node_id, prop, idx)),		\
		.pull = RTL87X2G_GET_PULL(DT_PROP_BY_IDX(node_id, prop, idx)),		\
		.drive = RTL87X2G_GET_DRIVE(DT_PROP_BY_IDX(node_id, prop, idx)),		\
		.dir = RTL87X2G_GET_DIR(DT_PROP_BY_IDX(node_id, prop, idx)),		\
		.pull_strength = DT_PROP(node_id, bias_pull_strong),		\
		.fun = RTL87X2G_GET_FUN(DT_PROP_BY_IDX(node_id, prop, idx)),		\
		.wakeup_high = DT_PROP(node_id, wakeup_high),		\
		.wakeup_low = DT_PROP(node_id, wakeup_low),		\
	},

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                   \
    {DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),             \
                            DT_FOREACH_PROP_ELEM, psels,               \
                            Z_PINCTRL_STATE_PIN_INIT)}

/**
 * @brief Utility macro to obtain pin function.
 *
 * @param pincfg Pin configuration bit field.
 */
#define RTL87X2G_GET_FUN(pincfg) (((pincfg) >> RTL87X2G_FUN_POS) & RTL87X2G_FUN_MSK)

/**
 * @brief Utility macro to obtain pin drive mode.
 *
 * @param pincfg Pin configuration bit field.
 */
#define RTL87X2G_GET_DIR(pincfg) (((pincfg) >> RTL87X2G_DIR_POS) & RTL87X2G_DIR_MSK)

/**
 * @brief Utility macro to obtain pin drive mode.
 *
 * @param pincfg Pin configuration bit field.
 */
#define RTL87X2G_GET_DRIVE(pincfg) (((pincfg) >> RTL87X2G_DRIVE_POS) & RTL87X2G_DRIVE_MSK)

/**
 * @brief Utility macro to obtain pin pull configuration.
 *
 * @param pincfg Pin configuration bit field.
 */
#define RTL87X2G_GET_PULL(pincfg) (((pincfg) >> RTL87X2G_PULL_POS) & RTL87X2G_PULL_MSK)

/**
 * @brief Utility macro to obtain port and pin combination.
 *
 * @param pincfg Pin configuration bit field.
 */
#define RTL87X2G_GET_PIN(pincfg) (((pincfg) >> RTL87X2G_PIN_POS) & RTL87X2G_PIN_MSK)

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_REALTEK_RTL_COMMON_PINCTRL_SOC_H_ */
