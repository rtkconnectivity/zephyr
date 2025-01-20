/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_gpio

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <rtl876x_rcc.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_gpio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control/rtl8752h_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <zephyr/dt-bindings/gpio/realtek-rtl8752h-gpio.h>
#include "gpio_rtl8752h.h"
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>

#include "trace.h"

#define DBG_DIRECT_SHOW 0

LOG_MODULE_REGISTER(gpio_rtl8752h, CONFIG_GPIO_LOG_LEVEL);

static int gpio_rtl8752h_gpio2pad(uint32_t pin)
{

	if (pin <= 9) {
		return pin;
	} else if (pin <= 12) {
		return pin + 26;
	} else if (pin == 13) {
		return 32;
	} else if (pin <= 28) {
		return pin;
	} else if (pin == 29) {
#if RTL8752H_USE_P4_1_AS_GPIO29
		return 33;
#else
		return 29;
#endif
	} else if (pin == 30) {
#if RTL8752H_USE_P4_2_AS_GPIO30
		return 34;
#else
		return 30;
#endif
	} else if (pin == 31) {
		return 35;
	}

	return -EIO;
}

#ifdef CONFIG_PM_DEVICE
static int gpio_rtl8752h_pm_pad_list_insert(struct pm_pad_node *head, struct pm_pad_node *array,
					    uint8_t pad_num, uint8_t gpio_num);
static void gpio_rtl8752h_pm_pad_list_remove(struct pm_pad_node *head, struct pm_pad_node *array,
					     uint8_t pad_num, uint8_t gpio_num);
#endif

static int gpio_rtl8752h_pin_configure(const struct device *port, gpio_pin_t pin,
				       gpio_flags_t flags)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("port=%s, pin=%d, flags=0x%x, line%d\n", port->name, pin, flags, __LINE__);
#endif
	struct gpio_rtl8752h_data *data = port->data;
	uint32_t gpio_bit = BIT(pin);
	int pad_pin = gpio_rtl8752h_gpio2pad(pin);
	uint32_t pull_config;
	GPIO_InitTypeDef gpio_init_struct;
	uint8_t debounce_ms = (flags & RTL8752H_GPIO_INPUT_DEBOUNCE_MS_MASK) >>
			      RTL8752H_GPIO_INPUT_DEBOUNCE_MS_POS;
	int ret = 0;

	__ASSERT(pad_pin >= 0, "gpio port or pin error");

	if (flags & GPIO_OPEN_SOURCE && flags & GPIO_OPEN_DRAIN) {
		ret = -ENOTSUP;
		return ret;
	}

	if (flags == GPIO_DISCONNECTED) {
		Pinmux_Deinit(pad_pin);
		Pad_Config(pad_pin, PAD_SW_MODE, PAD_NOT_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
			   PAD_OUT_HIGH);
	} else {
		/* config pad pull status */

		if (flags & GPIO_PULL_UP) {
			pull_config = PAD_PULL_UP;
		} else if (flags & GPIO_PULL_DOWN) {
			pull_config = PAD_PULL_DOWN;
		} else {
			pull_config = PAD_PULL_NONE;
		}

		/* config gpio */

		GPIO_StructInit(&gpio_init_struct);

		if (debounce_ms) {
			gpio_init_struct.GPIO_DebounceTime = debounce_ms;
			gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
			data->pin_debounce_ms[pin] = debounce_ms;
		} else {
			gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
			data->pin_debounce_ms[pin] = 0;
		}

		gpio_init_struct.GPIO_Pin = gpio_bit;
		gpio_init_struct.GPIO_Mode = flags & GPIO_OUTPUT ? GPIO_Mode_OUT : GPIO_Mode_IN;
		gpio_init_struct.GPIO_ITCmd = flags & GPIO_INT_ENABLE ? ENABLE : DISABLE;
		gpio_init_struct.GPIO_ITTrigger = flags & GPIO_INT_LEVELS_LOGICAL
							  ? GPIO_INT_Trigger_LEVEL
							  : GPIO_INT_Trigger_EDGE;
		gpio_init_struct.GPIO_ITPolarity = flags & GPIO_INT_LOW_0
							   ? GPIO_INT_POLARITY_ACTIVE_LOW
							   : GPIO_INT_POLARITY_ACTIVE_HIGH;
		Pad_Config(pad_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, pull_config,
			   flags & GPIO_OUTPUT ? PAD_OUT_ENABLE : PAD_OUT_DISABLE,
			   flags & GPIO_OUTPUT_INIT_HIGH ? PAD_OUT_HIGH : PAD_OUT_LOW);
		Pinmux_Config(pad_pin, DWGPIO);

		switch (flags & (GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH | GPIO_OUTPUT_INIT_LOW)) {
		case (GPIO_OUTPUT_HIGH):
			GPIO_WriteBit(gpio_bit, 1);
			break;
		case (GPIO_OUTPUT_LOW):
			GPIO_WriteBit(gpio_bit, 0);
			break;
		default:
			break;
		}

		/* to avoid trigger gpio interrupt */
		if (debounce_ms && (flags & GPIO_INT_ENABLE)) {
			GPIO_INTConfig(gpio_bit, DISABLE);
			GPIO_Init(&gpio_init_struct);
			GPIO_MaskINTConfig(gpio_bit, ENABLE);
			GPIO_INTConfig(gpio_bit, ENABLE);
			k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
			GPIO_ClearINTPendingBit(gpio_bit);
			GPIO_MaskINTConfig(gpio_bit, DISABLE);
		} else {
			GPIO_Init(&gpio_init_struct);
		}
	}

#ifdef CONFIG_PM_DEVICE
	if (flags & GPIO_OUTPUT) {
		gpio_rtl8752h_pm_pad_list_remove(data->list.wakeup_head, data->list.array, pad_pin,
						 pin);
		ret = gpio_rtl8752h_pm_pad_list_insert(data->list.output_head, data->list.array,
						       pad_pin, pin);
		if (ret) {
			LOG_ERR("Failed to insert gpio pm pad list");
			return ret;
		}
	} else if ((flags & GPIO_INPUT) && (flags & RTL8752H_GPIO_INPUT_PM_WAKEUP)) {
		gpio_rtl8752h_pm_pad_list_remove(data->list.output_head, data->list.array, pad_pin,
						 pin);
		ret = gpio_rtl8752h_pm_pad_list_insert(data->list.wakeup_head, data->list.array,
						       pad_pin, pin);
		if (ret) {
			LOG_ERR("Failed to insert gpio pm pad list");
			return ret;
		}
	}

#endif

	return 0;
}

static int gpio_rtl8752h_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	*value = GPIO_ReadInputData();

	return 0;
}

static int gpio_rtl8752h_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					     gpio_port_value_t value)
{
	gpio_port_pins_t pins_value = GPIO_ReadInputData();

	pins_value = (pins_value & ~mask) | (mask & value);
	GPIO_Write(pins_value);

	return 0;
}

static int gpio_rtl8752h_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	GPIO_SetBits(pins);

	return 0;
}

static int gpio_rtl8752h_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	GPIO_ResetBits(pins);

	return 0;
}

static int gpio_rtl8752h_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	gpio_port_pins_t pins_value = GPIO_ReadInputData();

	pins_value = (pins_value | pins) & ~(pins_value & pins);
	GPIO_Write(pins_value);
#if DBG_DIRECT_SHOW
	DBG_DIRECT("port=%s, pin=0x%x, pins_value=0x%x, line%d\n", port->name, pins, pins_value,
		   __LINE__);
#endif
	return 0;
}

static int gpio_rtl8752h_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
						 enum gpio_int_mode mode, enum gpio_int_trig trig)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("port=%s, pin=%d, mode=0x%x, trig=0x%x, line%d\n", port->name, pin, mode, trig,
		   __LINE__);
#endif
	struct gpio_rtl8752h_data *data = port->data;
	uint32_t gpio_bit = BIT(pin);
	GPIO_InitTypeDef gpio_init_struct;

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	if (mode == GPIO_INT_MODE_DISABLE_ONLY) {
		GPIO_MaskINTConfig(gpio_bit, ENABLE);
		GPIO_INTConfig(gpio_bit, DISABLE);
		return 0;
	} else if (mode == GPIO_INT_MODE_ENABLE_ONLY) {
		GPIO_INTConfig(gpio_bit, ENABLE);
		GPIO_MaskINTConfig(gpio_bit, DISABLE);
		return 0;
	}
#endif /* CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT */

	GPIO_INTConfig(gpio_bit, DISABLE);

	GPIO_StructInit(&gpio_init_struct);

	gpio_init_struct.GPIO_Pin = gpio_bit;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
	if (data->pin_debounce_ms[pin]) {
		gpio_init_struct.GPIO_DebounceTime = data->pin_debounce_ms[pin];
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
	default:
		return -ENOTSUP;
	}

	GPIO_Init(&gpio_init_struct);
	GPIO_MaskINTConfig(gpio_bit, ENABLE);
	GPIO_INTConfig(gpio_bit, ENABLE);

	/* to avoid trigger gpio interrupt */
	if (data->pin_debounce_ms[pin]) {
		k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
	}

	GPIO_ClearINTPendingBit(gpio_bit);
	GPIO_MaskINTConfig(gpio_bit, DISABLE);

	return 0;
}

static int gpio_rtl8752h_manage_callback(const struct device *port, struct gpio_callback *cb,
					 bool set)
{
	struct gpio_rtl8752h_data *port_data = port->data;

	return gpio_manage_callback(&port_data->cb, cb, set);
}

static uint32_t gpio_rtl8752h_get_pending_int(const struct device *dev)
{
	const struct gpio_rtl8752h_config *config = dev->config;
	GPIO_TypeDef *port_base = config->port_base;

	return port_base->INTSTATUS;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
int gpio_rtl8752h_port_get_direction(const struct device *port, gpio_port_pins_t map,
				     gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	const struct gpio_rtl8752h_config *config = port->config;
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
static int gpio_rtl8752h_pm_pad_list_init(struct pm_pad_node_list *list)
{
	list->output_head = &(list->array[32]);
	list->output_head->pad_num = 0xff;
	list->output_head->next_gpio_num = 0xff;

	list->wakeup_head = &(list->array[33]);
	list->wakeup_head->pad_num = 0xff;
	list->wakeup_head->next_gpio_num = 0xff;

	return 0;
}

static int gpio_rtl8752h_pm_pad_list_insert(struct pm_pad_node *head, struct pm_pad_node *array,
					    uint8_t pad_num, uint8_t gpio_num)
{
	struct pm_pad_node *new_node;
	struct pm_pad_node *cur_node = head;

	/* Search from head to tail */
	while (cur_node->next_gpio_num != 0xff) {
		if (cur_node->pad_num > pad_num &&
		    array[cur_node->next_gpio_num].pad_num < pad_num) {
			/* Insert the node */
			new_node = &(array[gpio_num]);
			new_node->pad_num = pad_num;
			new_node->next_gpio_num = cur_node->next_gpio_num;
			cur_node->next_gpio_num = gpio_num;
			return 0;
		} else if (cur_node->pad_num == pad_num) {
			return 0;
		}

		cur_node = &(array[cur_node->next_gpio_num]);
		continue;
	}

	if (cur_node->pad_num == pad_num) {
		return 0;
	}

	/* Insert the first node */
	new_node = &(array[gpio_num]);
	new_node->pad_num = pad_num;
	new_node->next_gpio_num = cur_node->next_gpio_num;
	cur_node->next_gpio_num = gpio_num;

	return 0;
}

static void gpio_rtl8752h_pm_pad_list_remove(struct pm_pad_node *head, struct pm_pad_node *array,
					     uint8_t pad_num, uint8_t gpio_num)
{
	struct pm_pad_node *cur_node = head;

	while (cur_node->next_gpio_num != 0xff) {
		if (array[cur_node->next_gpio_num].pad_num == pad_num) {
			if (array[cur_node->next_gpio_num].next_gpio_num != 0xff) {
				cur_node->next_gpio_num =
					array[cur_node->next_gpio_num].next_gpio_num;
			} else {
				cur_node->next_gpio_num = 0xff;
			}
			return;
		} else if (array[cur_node->next_gpio_num].pad_num < pad_num) {
			return;
		}

		cur_node = &(array[cur_node->next_gpio_num]);
		continue;
	}
}

static int gpio_rtl8752h_pm_action(const struct device *port, enum pm_device_action action)
{
	const struct gpio_rtl8752h_config *config = port->config;
	struct gpio_rtl8752h_data *data = port->data;
	GPIO_TypeDef *port_base = config->port_base;
	struct pm_pad_node *pm_pad_node_array = data->list.array;
	struct pm_pad_node *cur_output_pad_node = data->list.output_head;
	struct pm_pad_node *cur_wakeup_pad_node = data->list.wakeup_head;

	extern void GPIO_DLPSEnter(void *PeriReg, void *StoreBuf);
	extern void GPIO_DLPSExit(void *PeriReg, void *StoreBuf);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		while (cur_output_pad_node->next_gpio_num != 0xff) {
			Pad_OutputControlValue(
				pm_pad_node_array[cur_output_pad_node->next_gpio_num].pad_num,
				GPIO_ReadOutputDataBit(BIT(cur_output_pad_node->next_gpio_num)));
			Pad_ControlSelectValue(
				pm_pad_node_array[cur_output_pad_node->next_gpio_num].pad_num,
				PAD_SW_MODE);
			cur_output_pad_node =
				&(pm_pad_node_array[cur_output_pad_node->next_gpio_num]);
		}

		while (cur_wakeup_pad_node->next_gpio_num != 0xff) {
			/* Enable pm wakeup function for gpios which ：
			 * 1. Configured RTL8752H_GPIO_INPUT_PM_WAKEUP flag;
			 * 2. Enabled interrupt;
			 */
			if (port_base->INTEN & BIT(cur_wakeup_pad_node->next_gpio_num)) {
				bool high_trigger = port_base->INTPOLARITY &
						    BIT(cur_wakeup_pad_node->next_gpio_num);

				Pad_ControlSelectValue(
					pm_pad_node_array[cur_wakeup_pad_node->next_gpio_num]
						.pad_num,
					PAD_SW_MODE);
				System_WakeUpPinEnable(
					pm_pad_node_array[cur_wakeup_pad_node->next_gpio_num]
						.pad_num,
					high_trigger ? PAD_WAKEUP_POL_HIGH : PAD_WAKEUP_POL_LOW,
					DISABLE, 0);
			}
			cur_wakeup_pad_node =
				&(pm_pad_node_array[cur_wakeup_pad_node->next_gpio_num]);
		}

		GPIO_DLPSEnter(port_base, &data->store_buf);

		break;
	case PM_DEVICE_ACTION_RESUME:

		while (cur_output_pad_node->next_gpio_num != 0xff) {
			Pad_ControlSelectValue(
				pm_pad_node_array[cur_output_pad_node->next_gpio_num].pad_num,
				PAD_PINMUX_MODE);
			cur_output_pad_node =
				&(pm_pad_node_array[cur_output_pad_node->next_gpio_num]);
		}

		while (cur_wakeup_pad_node->next_gpio_num != 0xff) {
			Pad_ControlSelectValue(
				pm_pad_node_array[cur_wakeup_pad_node->next_gpio_num].pad_num,
				PAD_PINMUX_MODE);
			System_WakeUpPinDisable(
				pm_pad_node_array[cur_wakeup_pad_node->next_gpio_num].pad_num);
			cur_wakeup_pad_node =
				&(pm_pad_node_array[cur_wakeup_pad_node->next_gpio_num]);
		}

		GPIO_DLPSExit(port_base, &data->store_buf);

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct gpio_driver_api gpio_rtl8752h_driver_api = {
	.pin_configure = gpio_rtl8752h_pin_configure,
	.port_get_raw = gpio_rtl8752h_port_get_raw,
	.port_set_masked_raw = gpio_rtl8752h_port_set_masked_raw,
	.port_set_bits_raw = gpio_rtl8752h_port_set_bits_raw,
	.port_clear_bits_raw = gpio_rtl8752h_port_clear_bits_raw,
	.port_toggle_bits = gpio_rtl8752h_port_toggle_bits,
	.pin_interrupt_configure = gpio_rtl8752h_pin_interrupt_configure,
	.manage_callback = gpio_rtl8752h_manage_callback,
	.get_pending_int = gpio_rtl8752h_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_rtl8752h_port_get_direction,
#endif
};

static void gpio_rtl8752h_isr(void *arg)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("line%d\n", __LINE__);
#endif
	const struct device *dev = (struct device *)arg;
	const struct gpio_rtl8752h_config *config = dev->config;
	struct gpio_rtl8752h_data *data = dev->data;
	GPIO_TypeDef *port_base = config->port_base;
	const struct device *port = dev;
	uint32_t pins = port_base->INTSTATUS;

	gpio_fire_callbacks(&data->cb, port, pins);

	for (uint32_t i = 0; i < 32; i++) {
		if (BIT(i) & pins) {
			GPIO_ClearINTPendingBit(BIT(i) & pins);
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
static int gpio_rtl8752h_init(const struct device *dev)
{
	struct gpio_rtl8752h_data *data = dev->data;
	const struct gpio_rtl8752h_config *config = dev->config;
	int ret = 0;

	(void)clock_control_on(RTL8752H_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);
	for (uint8_t i = 0; i < config->irq_info->num_irq; ++i) {
		irq_disable(config->irq_info->gpio_irqs[i].irq);
		irq_connect_dynamic(config->irq_info->gpio_irqs[i].irq,
				    config->irq_info->gpio_irqs[i].priority,
				    (const void *)gpio_rtl8752h_isr, dev, 0);
		irq_enable(config->irq_info->gpio_irqs[i].irq);
	}

	data->dev = dev;
	memset(data->pin_debounce_ms, 0, sizeof(data->pin_debounce_ms));

#ifdef CONFIG_PM_DEVICE
	ret = gpio_rtl8752h_pm_pad_list_init(&(data->list));
	if (ret) {
		LOG_ERR("Failed to init gpio pm pad list");
		return ret;
	}

#endif
	return ret;
}

#define GPIO_RTL8752H_SET_GPIO_IRQ_INFO(irq_idx, index)                                            \
	{                                                                                          \
		.irq = DT_INST_IRQ_BY_IDX(index, irq_idx, irq),                                    \
		.priority = DT_INST_IRQ_BY_IDX(index, irq_idx, priority),                          \
	}

#define GPIO_RTL8752H_SET_IRQ_INFO(index)                                                          \
	static struct gpio_rtl8752h_irq_info gpio_rtl8752h_irq_info##index = {                     \
		.gpio_irqs = {LISTIFY(DT_NUM_IRQS(DT_DRV_INST(index)),                             \
				      GPIO_RTL8752H_SET_GPIO_IRQ_INFO, (, ), index)},              \
		.num_irq = DT_NUM_IRQS(DT_DRV_INST(index))};

#define GPIO_RTL8752H_GET_IRQ_INFO(index) .irq_info = &gpio_rtl8752h_irq_info##index,

#ifdef CONFIG_PM_DEVICE
#define GPIO_RTL8752H_ARRAY_DEFINE(index) struct pm_pad_node pm_pad_node_array##index[32 + 1 + 1];

#define GPIO_RTL8752H_DATA_INIT(index)                                                             \
	.list.array = pm_pad_node_array##index, .list.output_head = NULL, .list.wakeup_head = NULL,

#else
#define GPIO_RTL8752H_ARRAY_DEFINE(index)
#define GPIO_RTL8752H_DATA_INIT(index)
#endif

#define GPIO_RTL8752H_DEVICE_INIT(index)                                                           \
	GPIO_RTL8752H_ARRAY_DEFINE(index)                                                          \
	GPIO_RTL8752H_SET_IRQ_INFO(index)                                                          \
	static const struct gpio_rtl8752h_config gpio_rtl8752h_port##index##_cfg = {               \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(index),           \
			},                                                                         \
		.port_base = (GPIO_TypeDef *)DT_INST_REG_ADDR(index),                              \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		GPIO_RTL8752H_GET_IRQ_INFO(index)};                                                \
                                                                                                   \
	static struct gpio_rtl8752h_data gpio_rtl8752h_port##index##_data = {                      \
		GPIO_RTL8752H_DATA_INIT(index)};                                                   \
	PM_DEVICE_DT_INST_DEFINE(index, gpio_rtl8752h_pm_action);                                  \
	DEVICE_DT_INST_DEFINE(index, gpio_rtl8752h_init, PM_DEVICE_DT_INST_GET(index),             \
			      &gpio_rtl8752h_port##index##_data, &gpio_rtl8752h_port##index##_cfg, \
			      PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_rtl8752h_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_RTL8752H_DEVICE_INIT)
