/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_gpio

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <zephyr/dt-bindings/gpio/realtek-bee-gpio.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_rcc.h>
#include <rtl_pinmux.h>
#include <rtl_gpio.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_rcc.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_gpio.h>
#endif

#include "gpio_bee.h"
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>
#include "trace.h"

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define BEE_GPIO_WriteBit(port, bit, val)            GPIO_WriteBit(port, bit, val)
#define BEE_GPIO_ReadOutputDataBit(port, bit)        GPIO_ReadOutputDataBit(port, bit)
#define BEE_GPIO_INTConfig(port, bit, val)           GPIO_INTConfig(port, bit, val)
#define BEE_GPIO_Init(port, val)                     GPIO_Init(port, val)
#define BEE_GPIO_MaskINTConfig(port, bit, val)       GPIO_MaskINTConfig(port, bit, val)
#define BEE_GPIO_ClearINTPendingBit(port, bit)       GPIO_ClearINTPendingBit(port, bit)
#define BEE_GPIO_SetBits(port, bit)                  GPIO_SetBits(port, bit)
#define BEE_GPIO_ResetBits(port, bit)                GPIO_ResetBits(port, bit)
#define BEE_GPIO_ReadInputData(port)                 GPIO_ReadInputData(port)
#define BEE_GPIO_Write(port, val)                    GPIO_Write(port, val)
#define BEE_Pad_SetControlMode(pad, mode)            Pad_SetControlMode(pad, mode)
#define BEE_Pad_SetOutputLevel(pad, val)             Pad_SetOutputLevel(pad, val)
#define BEE_System_WakeUpPinEnable(pin, pol, deb_en) System_WakeUpPinEnable(pin, pol, deb_en)
#define BEE_GPIO_REG_INTSATUS                        GPIO_INT_STS
#define BEE_GPIO_REG_INT_EN                          GPIO_INT_EN

extern uint32_t GPIO_SwapDebPinBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define BEE_GPIO_WriteBit(port, bit, val)            GPIO_WriteBit(bit, val)
#define BEE_GPIO_ReadOutputDataBit(port, bit)        GPIO_ReadOutputDataBit(bit)
#define BEE_GPIO_INTConfig(port, bit, val)           GPIO_INTConfig(bit, val)
#define BEE_GPIO_Init(port, val)                     GPIO_Init(val)
#define BEE_GPIO_MaskINTConfig(port, bit, val)       GPIO_MaskINTConfig(bit, val)
#define BEE_GPIO_ClearINTPendingBit(port, bit)       GPIO_ClearINTPendingBit(bit)
#define BEE_GPIO_SetBits(port, bit)                  GPIO_SetBits(bit)
#define BEE_GPIO_ResetBits(port, bit)                GPIO_ResetBits(bit)
#define BEE_GPIO_ReadInputData(port)                 GPIO_ReadInputData()
#define BEE_GPIO_Write(port, val)                    GPIO_Write(val)
#define BEE_Pad_SetControlMode(pad, mode)            Pad_ControlSelectValue(pad, mode)
#define BEE_Pad_SetOutputLevel(pad, val)             Pad_OutputControlValue(pad, val)
#define BEE_System_WakeUpPinEnable(pin, pol, deb_en) System_WakeUpPinEnable(pin, pol, deb_en, 0)
#define BEE_GPIO_REG_INTSATUS                        INTSTATUS
#define BEE_GPIO_REG_INT_EN                          INTEN
#endif

LOG_MODULE_REGISTER(gpio_bee, CONFIG_GPIO_LOG_LEVEL);

#ifdef CONFIG_PM_DEVICE
	extern void GPIO_DLPSEnter(void *PeriReg, void *StoreBuf);
	extern void GPIO_DLPSExit(void *PeriReg, void *StoreBuf);
#endif

static int gpio_bee_gpio2pad(uint8_t port_num, uint32_t pin)
{
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	/* There is no reuse situation for gpioa */
	if (port_num == 0) {
		if (pin < 16) {
			return pin;
		} else if (pin >= 21 && pin < 32) {
			return pin - 5;
		} else if (pin >= 16 && pin < 21) {
			return pin + 48;
		}
	}
	/* Handle reuse situation for gpiob */
	else if (port_num == 1) {
		if (pin < 19) {
			return pin + 27;
		}
#if CONFIG_SOC_RTL8777G
		return pin + 51;
#elif CONFIG_SOC_RTL8762GKU || CONFIG_SOC_RTL8762GKH || CONFIG_SOC_RTL8762GTP
		if (pin < 21) {
			return pin + 27;
		} else {
			return pin + 51;
		}
#elif CONFIG_SOC_RTL8762GTU || CONFIG_SOC_RTL8762GTH
		if (pin == 20) {
			return 49;
		} else if (pin == 21) {
#if CONFIG_BEE_USE_P6_2_AS_GPIOB21
			return 50;
#else
			return 72;
#endif
		} else if (pin == 22) {
#if CONFIG_BEE_USE_P6_3_AS_GPIOB22
			return 51;
#else
			return 73;
#endif
		} else if (pin == 23) {
			return 52;
		} else if (pin == 24) {
#if CONFIG_BEE_USE_P6_5_AS_GPIOB24
			return 53;
#else
			return 75;
#endif
		} else if (pin == 25) {
			return 76;
		} else if (pin == 26) {
#if CONFIG_BEE_USE_P6_7_AS_GPIOB26
			return 55;
#else
			return 77;
#endif
		} else if (pin < 30) {
			return pin + 51;
		}

#elif CONFIG_SOC_RTL8772GWP
		if (pin >= 21) {
			return pin + 51;
		}
#elif CONFIG_SOC_RTL8772GWF
		if (pin >= 21 && pin < 24) {
			return pin + 51;
		} else if (pin >= 27 && pin < 32) {
			return pin + 29;
		}
#endif
	}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
	if (pin <= 9) {
		return pin;
	} else if (pin <= 12) {
		return pin + 26;
	} else if (pin == 13) {
		return 32;
	} else if (pin <= 28) {
		return pin;
	} else if (pin == 29) {
#if BEE_USE_P4_1_AS_GPIO29
		return 33;
#else
		return 29;
#endif
	} else if (pin == 30) {
#if BEE_USE_P4_2_AS_GPIO30
		return 34;
#else
		return 30;
#endif
	} else if (pin == 31) {
		return 35;
	}
#endif

	return -EIO;
}

static int gpio_bee_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	LOG_DBG("port=%s, pin=%d, flags=0x%x, line%d\n", port->name, pin, flags, __LINE__);

	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data = port->data;
	GPIO_TypeDef *port_base;
	uint8_t port_num = config->port_num;
	uint32_t gpio_bit = BIT(pin);
	int pad_pin = gpio_bee_gpio2pad(port_num, pin);
	uint32_t pull_config;
	GPIO_InitTypeDef gpio_init_struct;
	uint8_t debounce_ms =
		(flags & BEE_GPIO_INPUT_DEBOUNCE_MS_MASK) >> BEE_GPIO_INPUT_DEBOUNCE_MS_POS;
	int ret = 0;

	port_base = config->port_base;

	__ASSERT(pad_pin >= 0, "gpio port or pin error");

	if (flags & GPIO_OPEN_SOURCE) {
		ret = -ENOTSUP;
		return ret;
	}

#ifdef CONFIG_PM_DEVICE
	if (flags & BEE_GPIO_INPUT_PM_WAKEUP && flags & GPIO_INT_EDGE) {
		LOG_WRN("port=%s, pin=%d is configured as wakeup and edge trigger at the same time",
			port->name, pin);
		if (debounce_ms) {
			LOG_WRN("port=%s, pin=%d is configured as wakeup, debounce enable and edge "
				"trigger at the same time",
				port->name, pin);
		}
	}
#endif

	if (flags == GPIO_DISCONNECTED) {
		Pinmux_Deinit(pad_pin);
		Pad_Config(pad_pin, PAD_SW_MODE, PAD_NOT_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
			   PAD_OUT_HIGH);
	} else {
		/* configure pad pull status */
		if (flags & GPIO_PULL_UP) {
			pull_config = PAD_PULL_UP;
		} else if (flags & GPIO_PULL_DOWN) {
			pull_config = PAD_PULL_DOWN;
		} else {
			pull_config = PAD_PULL_NONE;
		}

		/* configure gpio */
		GPIO_StructInit(&gpio_init_struct);

		if (debounce_ms) {
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
			gpio_init_struct.GPIO_DebounceClkSource = GPIO_DEBOUNCE_32K;
			gpio_init_struct.GPIO_DebounceClkDiv = GPIO_DEBOUNCE_DIVIDER_32;
			gpio_init_struct.GPIO_DebounceCntLimit = debounce_ms;
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
			gpio_init_struct.GPIO_DebounceTime = debounce_ms;
#endif
			gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
			data->pin_debounce_ms[pin] = debounce_ms;
		} else {
			gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
			data->pin_debounce_ms[pin] = 0;
		}

		gpio_init_struct.GPIO_Pin = gpio_bit;
		gpio_init_struct.GPIO_Mode = flags & GPIO_OUTPUT ? GPIO_Mode_OUT : GPIO_Mode_IN;
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		gpio_init_struct.GPIO_OutPutMode =
			flags & GPIO_OPEN_DRAIN ? GPIO_OUTPUT_OPENDRAIN : GPIO_OUTPUT_PUSHPULL;
#endif
		gpio_init_struct.GPIO_ITCmd = flags & GPIO_INT_ENABLE ? ENABLE : DISABLE;
		gpio_init_struct.GPIO_ITTrigger = flags & GPIO_INT_LEVELS_LOGICAL
							  ? GPIO_INT_Trigger_LEVEL
							  : GPIO_INT_Trigger_EDGE;
		gpio_init_struct.GPIO_ITPolarity = flags & GPIO_INT_LOW_0
							   ? GPIO_INT_POLARITY_ACTIVE_LOW
							   : GPIO_INT_POLARITY_ACTIVE_HIGH;
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		Pad_Dedicated_Config(pad_pin, DISABLE);
#endif
		Pad_Config(pad_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, pull_config,
			   flags & GPIO_OUTPUT ? PAD_OUT_ENABLE : PAD_OUT_DISABLE,
			   flags & GPIO_OUTPUT_INIT_HIGH ? PAD_OUT_HIGH : PAD_OUT_LOW);
		Pinmux_Config(pad_pin, DWGPIO);

		switch (flags & (GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH | GPIO_OUTPUT_INIT_LOW)) {
		case (GPIO_OUTPUT_HIGH):
			BEE_GPIO_WriteBit(port_base, gpio_bit, 1);
			break;
		case (GPIO_OUTPUT_LOW):
			BEE_GPIO_WriteBit(port_base, gpio_bit, 0);
			break;
		default:
			break;
		}

		/* to avoid trigger gpio interrupt */
		if (debounce_ms && (flags & GPIO_INT_ENABLE)) {
			BEE_GPIO_INTConfig(port_base, gpio_bit, DISABLE);
			BEE_GPIO_Init(port_base, &gpio_init_struct);
			BEE_GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
			BEE_GPIO_INTConfig(port_base, gpio_bit, ENABLE);
			k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
			BEE_GPIO_ClearINTPendingBit(port_base, gpio_bit);
			BEE_GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);
		} else {
			BEE_GPIO_Init(port_base, &gpio_init_struct);
		}
	}

#ifdef CONFIG_PM_DEVICE
	sys_snode_t *prev;

	if (flags & GPIO_OUTPUT) {
		data->list.array[pin].mode = PM_PAD_OUTPUT;
	} else if (flags & GPIO_INPUT) {
		if (flags & BEE_GPIO_INPUT_PM_WAKEUP) {
			data->list.array[pin].mode = PM_PAD_WAKEUP;
		} else {
			data->list.array[pin].mode = PM_PAD_INPUT;
		}
	} else {
		if (sys_slist_find(&data->list.list, (sys_snode_t *)&data->list.array[pin],
				   &prev)) {
			sys_slist_remove(&data->list.list, prev,
					 (sys_snode_t *)&data->list.array[pin]);
		}

		return 0;
	}

	if (!sys_slist_find(&data->list.list, (sys_snode_t *)&data->list.array[pin], NULL)) {
		sys_slist_append(&data->list.list, (sys_snode_t *)&data->list.array[pin]);
	}

#endif

	return 0;
}

static int gpio_bee_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base;

	port_base = config->port_base;

	*value = BEE_GPIO_ReadInputData(port_base);

	return 0;
}

static int gpio_bee_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base;

	port_base = config->port_base;

	gpio_port_pins_t pins_value = BEE_GPIO_ReadInputData(port_base);

	pins_value = (pins_value & ~mask) | (mask & value);
	BEE_GPIO_Write(port_base, pins_value);

	return 0;
}

static int gpio_bee_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base;

	port_base = config->port_base;

	BEE_GPIO_SetBits(port_base, pins);

	return 0;
}

static int gpio_bee_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base;

	port_base = config->port_base;

	BEE_GPIO_ResetBits(port_base, pins);

	return 0;
}

static int gpio_bee_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base;

	port_base = config->port_base;

	uint32_t pins_value = BEE_GPIO_ReadInputData(port_base);

	pins_value = (pins_value | pins) & ~(pins_value & pins);
	BEE_GPIO_Write(port_base, pins_value);
	LOG_DBG("port=%s, pin=0x%x, pins_value=0x%x, line%d\n", port->name, pins, pins_value,
		__LINE__);

	return 0;
}

static int gpio_bee_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					    enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	LOG_DBG("port=%s, pin=%d, mode=0x%x, trig=0x%x, line%d\n", port->name, pin, mode, trig,
		__LINE__);
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data = port->data;
	GPIO_TypeDef *port_base;
	uint32_t gpio_bit = BIT(pin);
	GPIO_InitTypeDef gpio_init_struct;

	port_base = config->port_base;

#ifdef CONFIG_PM_DEVICE
	if (data->list.array[pin].mode == PM_PAD_WAKEUP && mode == GPIO_INT_MODE_EDGE) {
		LOG_WRN("port=%s, pin=%d is configured as wakeup and edge trigger at the same time",
			port->name, pin);
		if (data->pin_debounce_ms[pin]) {
			LOG_WRN("port=%s, pin=%d is configured as wakeup, debounce enable and edge "
				"trigger at the same time",
				port->name, pin);
		}
	}
#endif

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	if (mode == GPIO_INT_MODE_DISABLE_ONLY) {
		BEE_GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
		BEE_GPIO_INTConfig(port_base, gpio_bit, DISABLE);
		return 0;
	} else if (mode == GPIO_INT_MODE_ENABLE_ONLY) {
		BEE_GPIO_INTConfig(port_base, gpio_bit, ENABLE);
		BEE_GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);
		return 0;
	}
#endif /* CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT */

	BEE_GPIO_INTConfig(port_base, gpio_bit, DISABLE);

	GPIO_StructInit(&gpio_init_struct);

	gpio_init_struct.GPIO_Pin = gpio_bit;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
	if (data->pin_debounce_ms[pin]) {
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		gpio_init_struct.GPIO_DebounceClkSource = GPIO_DEBOUNCE_32K;
		gpio_init_struct.GPIO_DebounceClkDiv = GPIO_DEBOUNCE_DIVIDER_32;
		gpio_init_struct.GPIO_DebounceCntLimit = data->pin_debounce_ms[pin];
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
		gpio_init_struct.GPIO_DebounceTime = data->pin_debounce_ms[pin];
#endif
		gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
	} else {
		gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		return 0;
	} else if (mode == GPIO_INT_MODE_EDGE) {
		gpio_init_struct.GPIO_ITCmd = ENABLE;
		gpio_init_struct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
	} else if (mode == GPIO_INT_MODE_LEVEL) {
		gpio_init_struct.GPIO_ITCmd = ENABLE;
		gpio_init_struct.GPIO_ITTrigger = GPIO_INT_Trigger_LEVEL;
	}

	switch (trig) {
	case GPIO_INT_TRIG_LOW:
		gpio_init_struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
		break;
	case GPIO_INT_TRIG_HIGH:
		gpio_init_struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
		break;
	case GPIO_INT_TRIG_BOTH:
#if CONFIG_BEE_GPIO_SUPPORT_BOTH_EDGE
		gpio_init_struct.GPIO_ITTrigger = GPIO_INT_BOTH_EDGE;
		break;
#endif
	default:
		return -ENOTSUP;
	}

	BEE_GPIO_Init(port_base, &gpio_init_struct);
	BEE_GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
	BEE_GPIO_INTConfig(port_base, gpio_bit, ENABLE);

	/* to avoid trigger gpio interrupt */
	if (data->pin_debounce_ms[pin]) {
		k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
	}

	BEE_GPIO_ClearINTPendingBit(port_base, gpio_bit);
	BEE_GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);

	return 0;
}

static int gpio_bee_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
	struct gpio_bee_data *port_data = port->data;

	return gpio_manage_callback(&port_data->cb, cb, set);
}

static uint32_t gpio_bee_get_pending_int(const struct device *dev)
{
	const struct gpio_bee_config *config = dev->config;
	GPIO_TypeDef *port_base = config->port_base;

	return port_base->BEE_GPIO_REG_INTSATUS;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
int gpio_bee_port_get_direction(const struct device *port, gpio_port_pins_t map,
				gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base = config->port_base;
	gpio_port_pins_t gpio_dir_status = port_base->GPIO_DDR;

	if (inputs != NULL) {
		*inputs = gpio_dir_status;
	}

	if (outputs != NULL) {
		*outputs = ~gpio_dir_status;
	}

	return 0;
}
#endif

#ifdef CONFIG_PM_DEVICE
static void output_pad_pm_suspend(const struct device *port, struct pm_pad_node *pad_node)
{
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base = config->port_base;
#endif
	uint8_t pad_num, gpio_num;

	pad_num = pad_node->pad_num;
	gpio_num = pad_node->gpio_num;

	BEE_Pad_SetOutputLevel(pad_num, BEE_GPIO_ReadOutputDataBit(port_base, BIT(gpio_num)));
	BEE_Pad_SetControlMode(pad_num, PAD_SW_MODE);
}

static void input_pad_pm_suspend(const struct device *port, struct pm_pad_node *pad_node)
{
	uint8_t pad_num;

	pad_num = pad_node->pad_num;
	BEE_Pad_SetControlMode(pad_num, PAD_SW_MODE);
}

static void wakeup_pad_pm_suspend(const struct device *port, struct pm_pad_node *pad_node)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base = config->port_base;
	uint8_t pad_num, gpio_num;

	pad_num = pad_node->pad_num;
	gpio_num = pad_node->gpio_num;
	if (port_base->BEE_GPIO_REG_INT_EN & BIT(gpio_num)) {
#if CONFIG_BEE_GPIO_SUPPORT_BOTH_EDGE
		if (port_base->INTBOTHEDGE & BIT(gpio_num)) {
			port_base->DATAIN;
			bool high_trigger = !(port_base->DATAIN & BIT(gpio_num));

			Pad_ControlSelectValue(pad_num, PAD_SW_MODE);
			BEE_System_WakeUpPinEnable(
				pad_num, high_trigger ? PAD_WAKEUP_POL_HIGH : PAD_WAKEUP_POL_LOW,
				DISABLE);
			if (high_trigger) {
				port_base->INTPOLARITY |= BIT(gpio_num);

			} else {
				port_base->INTPOLARITY &= (~BIT(gpio_num));
			}
		} else {
#endif
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
			uint32_t GPIO_Pin_Swap = GPIO_SwapDebPinBit(port_base, BIT(gpio_num));
			bool high_trigger = port_base->GPIO_EXT_DEB_POL_CTL & GPIO_Pin_Swap;
			bool edge_trigger = port_base->GPIO_INT_LV & BIT(gpio_num);
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
			bool high_trigger = port_base->INTPOLARITY & BIT(gpio_num);
			bool edge_trigger = port_base->INTTYPE & BIT(gpio_num);
#endif

			if (edge_trigger) {
				pad_node->read_before_dlps =
					BEE_GPIO_ReadInputData(port_base) & BIT(gpio_num);
			}

			BEE_Pad_SetControlMode(pad_num, PAD_SW_MODE);
			BEE_System_WakeUpPinEnable(
				pad_num, high_trigger ? PAD_WAKEUP_POL_HIGH : PAD_WAKEUP_POL_LOW,
				DISABLE);
#if CONFIG_BEE_GPIO_SUPPORT_BOTH_EDGE
		}
#endif
	}
}

static void output_pad_pm_resume(const struct device *port, struct pm_pad_node *pad_node)
{
	uint8_t pad_num;

	pad_num = pad_node->pad_num;

	Pinmux_Config(pad_num, DWGPIO);
	BEE_Pad_SetControlMode(pad_num, PAD_PINMUX_MODE);
}

static void input_pad_pm_resume(const struct device *port, struct pm_pad_node *pad_node)
{
	uint8_t pad_num;

	pad_num = pad_node->pad_num;

	Pinmux_Config(pad_num, DWGPIO);
	BEE_Pad_SetControlMode(pad_num, PAD_PINMUX_MODE);
}

static void wakeup_pad_pm_resume(const struct device *port, struct pm_pad_node *pad_node)
{
	uint8_t pad_num;

	pad_num = pad_node->pad_num;

	System_WakeUpPinDisable(pad_num);
	Pinmux_Config(pad_num, DWGPIO);
	BEE_Pad_SetControlMode(pad_num, PAD_PINMUX_MODE);
}

static void wakeup_edge_trigger(const struct device *port, struct pm_pad_node *pad_node)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data = port->data;
	GPIO_TypeDef *port_base = config->port_base;
	uint8_t pad_num, gpio_num;

	pad_num = pad_node->pad_num;
	gpio_num = pad_node->gpio_num;

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	uint32_t GPIO_Pin_Swap = GPIO_SwapDebPinBit(port_base, BIT(gpio_num));
	bool high_trigger = port_base->GPIO_EXT_DEB_POL_CTL & GPIO_Pin_Swap;
	bool edge_trigger = port_base->GPIO_INT_LV & BIT(gpio_num);
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
	bool high_trigger = port_base->INTPOLARITY & BIT(gpio_num);
	bool edge_trigger = port_base->INTTYPE & BIT(gpio_num);
#endif
	bool trigger_cb = false;
	bool deb_enable = !!data->pin_debounce_ms[gpio_num];
	bool read_after_dlps = BEE_GPIO_ReadInputData(port_base) & BIT(gpio_num);

	if (!deb_enable) {
		if (edge_trigger) {
			BEE_GPIO_ClearINTPendingBit(port_base, BIT(gpio_num));
			if (high_trigger) {
				if (pad_node->read_before_dlps == 0 && read_after_dlps == 1) {
					trigger_cb = true;
				}
			} else {
				if (pad_node->read_before_dlps == 1 && read_after_dlps == 0) {
					trigger_cb = true;
				}
			}
		}

		if (trigger_cb) {
			gpio_fire_callbacks(&data->cb, port, BIT(gpio_num));
		}
	}
}

static int gpio_bee_pm_action(const struct device *port, enum pm_device_action action)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data = port->data;
	GPIO_TypeDef *port_base = config->port_base;
	struct pm_pad_node *pad_node;
	uint8_t pad_num, gpio_num;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		SYS_SLIST_FOR_EACH_CONTAINER(&data->list.list, pad_node, node) {
			pad_num = pad_node->pad_num;
			gpio_num = pad_node->gpio_num;
			switch (pad_node->mode) {
			case PM_PAD_OUTPUT:
				output_pad_pm_suspend(port, pad_node);
				break;
			case PM_PAD_INPUT:
				input_pad_pm_suspend(port, pad_node);
				break;
			case PM_PAD_WAKEUP:
				/* Enable pm wakeup function for gpios which ：
				 * 1. Configured BEE_GPIO_INPUT_PM_WAKEUP flag;
				 * 2. Enabled interrupt;
				 */
				wakeup_pad_pm_suspend(port, pad_node);
				break;
			default:
				break;
			}
		}

		GPIO_DLPSEnter(port_base, &data->store_buf);

		break;
	case PM_DEVICE_ACTION_RESUME:
		SYS_SLIST_FOR_EACH_CONTAINER(&data->list.list, pad_node, node) {
			pad_num = pad_node->pad_num;
			gpio_num = pad_node->gpio_num;
			switch (pad_node->mode) {
			case PM_PAD_OUTPUT:
				output_pad_pm_resume(port, pad_node);
				break;
			case PM_PAD_INPUT:
				input_pad_pm_resume(port, pad_node);
				break;
			case PM_PAD_WAKEUP:
				wakeup_pad_pm_resume(port, pad_node);
				break;
			default:
				break;
			}
		}

		GPIO_DLPSExit(port_base, &data->store_buf);

		SYS_SLIST_FOR_EACH_CONTAINER(&data->list.list, pad_node, node) {
			pad_num = pad_node->pad_num;
			gpio_num = pad_node->gpio_num;
			switch (pad_node->mode) {
			case PM_PAD_INPUT:
			case PM_PAD_WAKEUP:
				wakeup_edge_trigger(port, pad_node);
				break;
			default:
				break;
			}
		}

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct gpio_driver_api gpio_bee_driver_api = {
	.pin_configure = gpio_bee_pin_configure,
	.port_get_raw = gpio_bee_port_get_raw,
	.port_set_masked_raw = gpio_bee_port_set_masked_raw,
	.port_set_bits_raw = gpio_bee_port_set_bits_raw,
	.port_clear_bits_raw = gpio_bee_port_clear_bits_raw,
	.port_toggle_bits = gpio_bee_port_toggle_bits,
	.pin_interrupt_configure = gpio_bee_pin_interrupt_configure,
	.manage_callback = gpio_bee_manage_callback,
	.get_pending_int = gpio_bee_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_bee_port_get_direction,
#endif
};

static void gpio_bee_isr(void *arg)
{
	LOG_DBG("line%d\n", __LINE__);
	const struct device *dev = (struct device *)arg;
	const struct gpio_bee_config *config = dev->config;
	struct gpio_bee_data *data = dev->data;
	GPIO_TypeDef *port_base = config->port_base;
	const struct device *port = dev;
	uint32_t pins = port_base->BEE_GPIO_REG_INTSATUS;

	gpio_fire_callbacks(&data->cb, port, pins);

	for (uint32_t i = 0; i < 32; i++) {
		if (BIT(i) & pins) {
			BEE_GPIO_ClearINTPendingBit(port_base, BIT(i) & pins);
		}
	}
}

static int gpio_bee_init(const struct device *dev)
{
	struct gpio_bee_data *data = dev->data;
	const struct gpio_bee_config *config = dev->config;
	int ret = 0;

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);

	for (uint8_t i = 0; i < config->irq_info->num_irq; ++i) {
		irq_connect_dynamic(config->irq_info->gpio_irqs[i].irq,
				    config->irq_info->gpio_irqs[i].priority,
				    (const void *)gpio_bee_isr, dev, 0);
		irq_enable(config->irq_info->gpio_irqs[i].irq);
	}

	data->dev = dev;
	memset(data->pin_debounce_ms, 0, sizeof(data->pin_debounce_ms));

#ifdef CONFIG_PM_DEVICE
	sys_slist_init(&(data->list.list));
	for (uint8_t i = 0; i < 32; i++) {
		data->list.array[i].gpio_num = i;
		data->list.array[i].pad_num = gpio_bee_gpio2pad(config->port_num, i);
	}
#endif
	return ret;
}

#define GPIO_BEE_SET_GPIO_IRQ_INFO(irq_idx, index)                                                 \
	{                                                                                          \
		.irq = DT_INST_IRQ_BY_IDX(index, irq_idx, irq),                                    \
		.priority = DT_INST_IRQ_BY_IDX(index, irq_idx, priority),                          \
	}

#define GPIO_BEE_SET_IRQ_INFO(index)                                                               \
	static struct gpio_bee_irq_info gpio_bee_irq_info##index = {                               \
		.gpio_irqs = {LISTIFY(DT_NUM_IRQS(DT_DRV_INST(index)), GPIO_BEE_SET_GPIO_IRQ_INFO, \
				      (,), index)},                                               \
		.num_irq = DT_NUM_IRQS(DT_DRV_INST(index))};

#define GPIO_BEE_GET_IRQ_INFO(index) .irq_info = &gpio_bee_irq_info##index,

#ifdef CONFIG_PM_DEVICE
#define GPIO_BEE_ARRAY_DEFINE(index) struct pm_pad_node pm_pad_node_array##index[32];

#define GPIO_BEE_DATA_INIT(index) .list.array = pm_pad_node_array##index,

#else
#define GPIO_BEE_ARRAY_DEFINE(index)
#define GPIO_BEE_DATA_INIT(index)
#endif

#define GPIO_BEE_DEVICE_INIT(index)                                                                \
	GPIO_BEE_ARRAY_DEFINE(index)                                                               \
	GPIO_BEE_SET_IRQ_INFO(index)                                                               \
	static const struct gpio_bee_config gpio_bee_port##index##_cfg = {                         \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(index),           \
			},                                                                         \
		.port_num = DT_INST_PROP(index, port),                                             \
		.port_base = (GPIO_TypeDef *)DT_INST_REG_ADDR(index),                              \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		GPIO_BEE_GET_IRQ_INFO(index)};                                                     \
                                                                                                   \
	static struct gpio_bee_data gpio_bee_port##index##_data = {GPIO_BEE_DATA_INIT(index)};     \
	PM_DEVICE_DT_INST_DEFINE(index, gpio_bee_pm_action);                                       \
	DEVICE_DT_INST_DEFINE(index, gpio_bee_init, PM_DEVICE_DT_INST_GET(index),                  \
			      &gpio_bee_port##index##_data, &gpio_bee_port##index##_cfg,           \
			      PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_BEE_DEVICE_INIT)
