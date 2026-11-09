/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Realtek Bee GPIO extension API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_BEE_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_BEE_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Pad value meaning that a GPIO pin is not carried by any pad. */
#define GPIO_BEE_PAD_NONE 0xFFU

/**
 * @brief Move a GPIO pin to another pad.
 *
 * Several pads can carry the same GPIO pin, and the pinctrl default state of
 * the port only picks one of them at boot. The pin must be disconnected
 * (GPIO_DISCONNECTED) while it is moved, so the old pad is already a software
 * mode pad and is left alone; a wakeup armed on it is disabled. Configure the
 * pin again afterwards.
 *
 * @param port GPIO port device.
 * @param pin Pin number within the port.
 * @param pad Pad number that carries the pin, or GPIO_BEE_PAD_NONE.
 * @param prev_pad If not NULL, set to the pad that carried the pin before, or
 *                 GPIO_BEE_PAD_NONE. Pass it back as @p pad to undo the move.
 *
 * @retval 0 On success.
 * @retval -EINVAL If the pad does not carry this pin of this port.
 * @retval -EBUSY If the pin is still connected.
 */
int gpio_bee_pin_remap(const struct device *port, gpio_pin_t pin, uint8_t pad, uint8_t *prev_pad);

/**
 * @brief Get the GPIO port and pin that a pad carries.
 *
 * @param pad Pad number.
 * @param port Set to the GPIO port device.
 * @param pin Set to the pin number within the port.
 *
 * @retval 0 On success.
 * @retval -EINVAL If the pad carries no GPIO pin.
 * @retval -ENODEV If the GPIO port of the pad is not enabled.
 */
int gpio_bee_pad_to_pin(uint8_t pad, const struct device **port, gpio_pin_t *pin);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_BEE_H_ */
