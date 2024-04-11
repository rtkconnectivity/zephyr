/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_gpio

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <rtl_rcc.h>
#include <rtl_pinmux.h>
#include <rtl_gpio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <zephyr/dt-bindings/gpio/realtek-rtl87x2g-gpio.h>
#include "gpio_rtl87x2g.h"
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_rtl87x2g, CONFIG_GPIO_LOG_LEVEL);

static int gpio_rtl87x2g_gpio2pad(uint8_t port_num, uint32_t pin)
{
    /* There is no reuse situation for gpioa */
    if (port_num == 0)
    {
        if (pin < 16)
        {
            return pin;
        }
        else if (pin >= 21 && pin < 32)
        {
            return pin - 5;
        }
        else if (pin >= 16 && pin < 21)
        {
            return pin + 48;
        }
    }
    /* Handle reuse situation for gpiob */
    else if (port_num == 1)
    {
        if (pin < 19)
        {
            return pin + 27;
        }
        else
        {
#if CONFIG_SOC_RTL8777G
            return pin + 51;
#elif CONFIG_SOC_RTL8762GKU || CONFIG_SOC_RTL8762GTP
            if (pin < 21)
            {
                return pin + 27;
            }
            else
            {
                return pin + 51;
            }
#elif CONFIG_SOC_RTL8762GTU
            if (pin < 25)
            {
                return pin + 29;
            }
            else if (pin < 26)
            {
                return pin + 51;
            }
            else if (pin < 27)
            {
                return pin + 29;
            }
            else if (pin < 30)
            {
                return pin + 51;
            }
#elif CONFIG_SOC_RTL8772GWP
            if (pin >= 21)
            {
                return pin + 51;
            }
#elif CONFIG_SOC_RTL8772GWF
            if (pin >= 21 && pin < 24)
            {
                return pin + 51;
            }
            else if (pin >= 27 && pin < 32)
            {
                return pin + 29;
            }
#endif
        }
    }

    return -EIO;
}

static int gpio_rtl87x2g_pin_configure(const struct device *port, gpio_pin_t pin,
                                       gpio_flags_t flags)
{
    LOG_DBG("port=%s, pin=%d, flags=0x%x, line%d\n", port->name, pin, flags, __LINE__);

    const struct gpio_rtl87x2g_config *config = port->config;
    struct gpio_rtl87x2g_data *data = port->data;
    GPIO_TypeDef *port_base = config->port_base;
    uint8_t port_num = config->port_num;
    uint32_t gpio_bit = BIT(pin);
    int pad_pin = gpio_rtl87x2g_gpio2pad(port_num, pin);
    PADPullMode_TypeDef pull_config;
    GPIO_InitTypeDef gpio_init_struct;
    uint8_t debounce_ms = (flags & RTL87X2G_GPIO_INPUT_DEBOUNCE_MS_MASK) \
                          >> RTL87X2G_GPIO_INPUT_DEBOUNCE_MS_POS;

    __ASSERT(pad_pin >= 0, "gpio port or pin error");

    if (flags & GPIO_OPEN_SOURCE)
    {
        return -ENOTSUP;
    }

    if (flags == GPIO_DISCONNECTED)
    {
        Pinmux_Deinit(pad_pin);
        Pad_Config(pad_pin, PAD_SW_MODE, PAD_NOT_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
                   PAD_OUT_HIGH);

        return 0;
    }

    /* config pad pull status */

    if (flags & GPIO_PULL_UP)
    {
        pull_config = PAD_PULL_UP;
    }
    else if (flags & GPIO_PULL_DOWN)
    {
        pull_config = PAD_PULL_DOWN;
    }
    else
    {
        pull_config = PAD_PULL_NONE;
    }

    /* config gpio */

    GPIO_StructInit(&gpio_init_struct);

    if (debounce_ms)
    {
        gpio_init_struct.GPIO_DebounceClkSource = GPIO_DEBOUNCE_32K;
        gpio_init_struct.GPIO_DebounceClkDiv = GPIO_DEBOUNCE_DIVIDER_32;
        gpio_init_struct.GPIO_DebounceCntLimit = debounce_ms;
        gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
        data->pin_debounce_ms[pin] = debounce_ms;
    }
    else
    {
        gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
        data->pin_debounce_ms[pin] = 0;
    }

    gpio_init_struct.GPIO_Pin = gpio_bit;
    gpio_init_struct.GPIO_Mode = flags & GPIO_OUTPUT ? GPIO_Mode_OUT : GPIO_Mode_IN;
    gpio_init_struct.GPIO_OutPutMode = flags & GPIO_OPEN_DRAIN ? GPIO_OUTPUT_OPENDRAIN :
                                       GPIO_OUTPUT_PUSHPULL;
    gpio_init_struct.GPIO_ITCmd = flags & GPIO_INT_ENABLE ? ENABLE : DISABLE;
    gpio_init_struct.GPIO_ITTrigger = flags & GPIO_INT_LEVELS_LOGICAL ? GPIO_INT_Trigger_LEVEL :
                                      GPIO_INT_Trigger_EDGE;
    gpio_init_struct.GPIO_ITPolarity = flags & GPIO_INT_LOW_0 ? GPIO_INT_POLARITY_ACTIVE_LOW :
                                       GPIO_INT_POLARITY_ACTIVE_HIGH;
    Pad_Config(pad_pin, PAD_PINMUX_MODE, \
               PAD_IS_PWRON, \
               pull_config, \
               flags & GPIO_OUTPUT ? PAD_OUT_ENABLE : PAD_OUT_DISABLE, \
               flags & GPIO_OUTPUT_INIT_HIGH ? PAD_OUT_HIGH : PAD_OUT_LOW);
    Pinmux_Config(pad_pin, DWGPIO);

    switch (flags & (GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH | GPIO_OUTPUT_INIT_LOW))
    {
    case (GPIO_OUTPUT_HIGH):
        GPIO_WriteBit(port_base, gpio_bit, 1);
        break;
    case (GPIO_OUTPUT_LOW):
        GPIO_WriteBit(port_base, gpio_bit, 0);
        break;
    default:
        break;
    }

    /* to avoid trigger gpio interrupt */
    if (debounce_ms && (flags & GPIO_INT_ENABLE))
    {
        GPIO_INTConfig(port_base, gpio_bit, DISABLE);
        GPIO_Init(port_base, &gpio_init_struct);
        GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
        GPIO_INTConfig(port_base, gpio_bit, ENABLE);
        k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
        GPIO_ClearINTPendingBit(port_base, gpio_bit);
        GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);
    }
    else
    {
        GPIO_Init(port_base, &gpio_init_struct);
    }

    return 0;
}

static int gpio_rtl87x2g_port_get_raw(const struct device *port,
                                      gpio_port_value_t *value)
{
    const struct gpio_rtl87x2g_config *config = port->config;
    GPIO_TypeDef *port_base = config->port_base;

    *value = GPIO_ReadInputData(port_base);

    return 0;
}

static int gpio_rtl87x2g_port_set_masked_raw(const struct device *port,
                                             gpio_port_pins_t mask,
                                             gpio_port_value_t value)
{
    const struct gpio_rtl87x2g_config *config = port->config;
    GPIO_TypeDef *port_base = config->port_base;
    gpio_port_pins_t pins_value = GPIO_ReadInputData(port_base);

    pins_value = (pins_value & ~mask) | (mask & value);
    GPIO_Write(port_base, pins_value);

    return 0;
}

static int gpio_rtl87x2g_port_set_bits_raw(const struct device *port,
                                           gpio_port_pins_t pins)
{
    const struct gpio_rtl87x2g_config *config = port->config;
    GPIO_TypeDef *port_base = config->port_base;

    GPIO_SetBits(port_base, pins);

    return 0;
}

static int gpio_rtl87x2g_port_clear_bits_raw(const struct device *port,
                                             gpio_port_pins_t pins)
{
    const struct gpio_rtl87x2g_config *config = port->config;
    GPIO_TypeDef *port_base = config->port_base;

    GPIO_ResetBits(port_base, pins);

    return 0;
}

static int gpio_rtl87x2g_port_toggle_bits(const struct device *port,
                                          gpio_port_pins_t pins)
{
    const struct gpio_rtl87x2g_config *config = port->config;
    GPIO_TypeDef *port_base = config->port_base;
    gpio_port_pins_t pins_value = GPIO_ReadInputData(port_base);
    pins_value = (pins_value | pins) & ~(pins_value & pins);
    GPIO_Write(port_base, pins_value);
    LOG_DBG("port=%s, pin=0x%x, pins_value=0x%x, line%d\n", port->name, pins, pins_value,
            __LINE__);

    return 0;
}

static int gpio_rtl87x2g_pin_interrupt_configure(const struct device *port,
                                                 gpio_pin_t pin,
                                                 enum gpio_int_mode mode, enum gpio_int_trig trig)
{
    LOG_DBG("port=%s, pin=%d, mode=0x%x, trig=0x%x, line%d\n", port->name, pin, mode, trig,
            __LINE__);
    const struct gpio_rtl87x2g_config *config = port->config;
    struct gpio_rtl87x2g_data *data = port->data;
    GPIO_TypeDef *port_base = config->port_base;
    uint32_t gpio_bit = BIT(pin);
    GPIO_InitTypeDef gpio_init_struct;
#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
    if (mode == GPIO_INT_MODE_DISABLE_ONLY)
    {
        GPIO_MaskINTConfig(port_base, gpio_bit, ENALE);
        GPIO_INTConfig(port_base, gpio_bit, DISABLE);
        return 0;
    }
    else if (mode == GPIO_INT_MODE_ENABLE_ONLY)
    {
        GPIO_INTConfig(port_base, gpio_bit, ENABLE);
        GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);
        return 0;
    }
#endif /* CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT */

    GPIO_INTConfig(port_base, gpio_bit, DISABLE);

    GPIO_StructInit(&gpio_init_struct);

    gpio_init_struct.GPIO_Pin = gpio_bit;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
    if (data->pin_debounce_ms[pin])
    {
        gpio_init_struct.GPIO_DebounceClkSource = GPIO_DEBOUNCE_32K;
        gpio_init_struct.GPIO_DebounceClkDiv = GPIO_DEBOUNCE_DIVIDER_32;
        gpio_init_struct.GPIO_DebounceCntLimit = data->pin_debounce_ms[pin];
        gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    }
    else
    {
        gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
    }

    switch (trig)
    {
        case GPIO_INT_TRIG_LOW:
            gpio_init_struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
            break;
        case GPIO_INT_TRIG_HIGH:
            gpio_init_struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
            break;
        case GPIO_INT_TRIG_BOTH:
        default:
            return -ENOTSUP;
    }

    if (mode == GPIO_INT_DISABLE)
    {
        return 0;
    }
    else if (mode == GPIO_INT_MODE_LEVEL)
    {
        gpio_init_struct.GPIO_ITCmd = ENABLE;
        gpio_init_struct.GPIO_ITTrigger = GPIO_INT_Trigger_LEVEL;
    }
    else if (mode == GPIO_INT_MODE_EDGE)
    {
        gpio_init_struct.GPIO_ITCmd = ENABLE;
        gpio_init_struct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    }

    GPIO_Init(port_base, &gpio_init_struct);
    GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
    GPIO_INTConfig(port_base, gpio_bit, ENABLE);

    /* to avoid trigger gpio interrupt */
    if (data->pin_debounce_ms[pin])
    {
        k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
    }

    GPIO_ClearINTPendingBit(port_base, gpio_bit);
    GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);

    return 0;
}

static int gpio_rtl87x2g_manage_callback(const struct device *port,
                                         struct gpio_callback *cb,
                                         bool set)
{
    struct gpio_rtl87x2g_data *port_data = port->data;

    return gpio_manage_callback(&port_data->cb, cb, set);
}

static uint32_t gpio_rtl87x2g_get_pending_int(const struct device *dev)
{
    const struct gpio_rtl87x2g_config *config = dev->config;
    GPIO_TypeDef *port_base = config->port_base;

    return port_base->GPIO_INT_STS;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
int gpio_rtl87x2g_port_get_direction(const struct device *port, gpio_port_pins_t map,
                                     gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
    const struct gpio_rtl87x2g_config *config = port->config;
    GPIO_TypeDef *port_base = config->port_base;
    gpio_port_pins_t gpio_dir_status = port_base->GPIO_DDR;

    if (inputs != NULL)
    {
        *inputs = gpio_dir_status;
    }

    if (outputs != NULL)
    {
        *outputs = ~gpio_dir_status;
    }
    return 0;
}
#endif

#ifdef CONFIG_PM_DEVICE
static int gpio_rtl87x2g_pm_action(const struct device *port,
                                   enum pm_device_action action)
{
    const struct gpio_rtl87x2g_config *config = port->config;
    struct gpio_rtl87x2g_data *data = port->data;
    GPIO_TypeDef *port_base = config->port_base;
    int err;
    extern void GPIO_DLPSEnter(void *PeriReg, void *StoreBuf);
    extern void GPIO_DLPSExit(void *PeriReg, void *StoreBuf);

    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:

        GPIO_DLPSEnter(port_base, &data->store_buf);

        break;
    case PM_DEVICE_ACTION_RESUME:

        GPIO_DLPSExit(port_base, &data->store_buf);

        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct gpio_driver_api gpio_rtl87x2g_driver_api =
{
    .pin_configure = gpio_rtl87x2g_pin_configure,
    .port_get_raw = gpio_rtl87x2g_port_get_raw,
    .port_set_masked_raw = gpio_rtl87x2g_port_set_masked_raw,
    .port_set_bits_raw = gpio_rtl87x2g_port_set_bits_raw,
    .port_clear_bits_raw = gpio_rtl87x2g_port_clear_bits_raw,
    .port_toggle_bits = gpio_rtl87x2g_port_toggle_bits,
    .pin_interrupt_configure = gpio_rtl87x2g_pin_interrupt_configure,
    .manage_callback = gpio_rtl87x2g_manage_callback,
    .get_pending_int = gpio_rtl87x2g_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
    .port_get_direction = gpio_rtl87x2g_port_get_direction,
#endif
};

static void gpio_rtl87x2g_isr(void *arg)
{
    LOG_DBG("line%d\n", __LINE__);
    const struct device *dev = (struct device *)arg;
    const struct gpio_rtl87x2g_config *config = dev->config;
    struct gpio_rtl87x2g_data *data = dev->data;
    GPIO_TypeDef *port_base = config->port_base;
    sys_slist_t *list = &data->cb;
    const struct device *port = dev;
    uint32_t pins = port_base->GPIO_INT_STS;
    struct gpio_callback *cb = NULL, *tmp = NULL;
    SYS_SLIST_FOR_EACH_CONTAINER_SAFE(list, cb, tmp, node)
    {
        if (cb->pin_mask & pins)
        {
            __ASSERT(cb->handler, "No callback handler!");
            cb->handler(port, cb, cb->pin_mask & pins);
            GPIO_ClearINTPendingBit(port_base, cb->pin_mask & pins);
        }
    }
}

/**
 * @brief Initialize GPIO port
 *
 * Perform basic initialization of a GPIO port. The code will
 * enable the clock for corresponding peripheral.
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_rtl87x2g_init(const struct device *dev)
{
    struct gpio_rtl87x2g_data *data = dev->data;
    const struct gpio_rtl87x2g_config *config = dev->config;

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&config->clkid);

    for (uint8_t i = 0; i < config->irq_info->num_irq; ++i)
    {
        irq_connect_dynamic(config->irq_info->gpio_irqs[i].irq, \
                            config->irq_info->gpio_irqs[i].priority, \
                            (const void *)gpio_rtl87x2g_isr, dev,
                            0);
        irq_enable(config->irq_info->gpio_irqs[i].irq);
    }

    data->dev = dev;
    memset(data->pin_debounce_ms, 0, sizeof(data->pin_debounce_ms));
    return 0;
}

#define GPIO_RTL87X2G_SET_GPIO_IRQ_INFO(irq_idx, index)                         \
    {                                                                         \
        .irq  = DT_INST_IRQ_BY_IDX(index, irq_idx, irq),                      \
                .priority = DT_INST_IRQ_BY_IDX(index, irq_idx, priority),           \
    }

#define GPIO_RTL87X2G_SET_IRQ_INFO(index)                                       \
    static struct gpio_rtl87x2g_irq_info gpio_rtl87x2g_irq_info##index = {     \
        .gpio_irqs = {                                                    \
                                                                          LISTIFY(DT_NUM_IRQS(DT_DRV_INST(index)),                          \
                                                                                  GPIO_RTL87X2G_SET_GPIO_IRQ_INFO, (,), index)                     \
                     },                                                                       \
                     .num_irq = DT_NUM_IRQS(DT_DRV_INST(index))                             \
    };                                                                     \


#define GPIO_RTL87X2G_GET_IRQ_INFO(index)                 \
    .irq_info = &gpio_rtl87x2g_irq_info##index ,      \

#define GPIO_RTL87X2G_DEVICE_INIT(index)                                          \
    GPIO_RTL87X2G_SET_IRQ_INFO(index)                                            \
    static const struct gpio_rtl87x2g_config gpio_rtl87x2g_port##index##_cfg = {   \
        .common = {                                                              \
                                                                                 .port_pin_mask =                                                     \
                                                                                         GPIO_PORT_PIN_MASK_FROM_DT_INST(index),                                \
                  },                                                                        \
                  .port_num = DT_INST_PROP(index, port),                                     \
                              .port_base = (GPIO_TypeDef *)DT_INST_REG_ADDR(index),    \
                                           .clkid = DT_INST_CLOCKS_CELL(index, id),            \
                                                    GPIO_RTL87X2G_GET_IRQ_INFO(index)                                            \
    };                                                                                 \
    \
    static struct gpio_rtl87x2g_data gpio_rtl87x2g_port##index##_data;               \
    \
     PM_DEVICE_DT_INST_DEFINE(index, gpio_rtl87x2g_pm_action);    \
     DEVICE_DT_INST_DEFINE(index, gpio_rtl87x2g_init,                                  \
                          PM_DEVICE_DT_INST_GET(index),                                                                 \
                          &gpio_rtl87x2g_port##index##_data,                                    \
                          &gpio_rtl87x2g_port##index##_cfg,                                    \
                          PRE_KERNEL_1,                                                        \
                          CONFIG_GPIO_INIT_PRIORITY,                                            \
                          &gpio_rtl87x2g_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_RTL87X2G_DEVICE_INIT)

