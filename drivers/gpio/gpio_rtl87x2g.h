/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_RTL87X2G_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_RTL87X2G_H_

/**
 * @file header for RTL87X2G GPIO
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/gpio.h>
#include <rtl_gpio.h>

/* GPIO buses definitions */

struct gpio_rtl87x2g_irq_info
{
    const struct device *irq_dev;
    uint8_t num_irq;
    struct gpio_irq_info
    {
        uint32_t irq;
        uint32_t priority;
    } gpio_irqs[];
};

/**
 * @brief configuration of GPIO device
 */
struct gpio_rtl87x2g_config
{
    struct gpio_driver_config common;
    uint16_t clkid;
    uint8_t port_num;
    GPIO_TypeDef *port_base;
    struct gpio_rtl87x2g_irq_info *irq_info;
};

#ifdef CONFIG_PM_DEVICE
struct pm_pad_node
{
    uint8_t pad_num;
    uint8_t gpio_num;
    struct pm_pad_node* next;
};
#endif

/**
 * @brief driver data
 */
struct gpio_rtl87x2g_data
{
    struct gpio_driver_data common;
    const struct device *dev;
    sys_slist_t cb;
    uint8_t pin_debounce_ms[32];
#ifdef CONFIG_PM_DEVICE
    GPIOStoreReg_Typedef store_buf;
    struct pm_pad_node* pm_pad_node_head;
    struct k_heap *heap;
#endif
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param dev GPIO port device pointer
 * @param pin IO pin
 * @param conf GPIO mode
 * @param func Pin function
 *
 * @return 0 on success, negative errno code on failure
 */
int gpio_rtl87x2g_configure(const struct device *dev, int pin, int conf, int func);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_RTL87X2G_H_ */
