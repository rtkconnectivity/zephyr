/*
 * Copyright(c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_counter_timer

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "bee_timer_common.h"

LOG_MODULE_REGISTER(counter_bee_timer, CONFIG_COUNTER_LOG_LEVEL);

#if defined(CONFIG_SOC_SERIES_RTL8752H)
#define GET_COUNTER_DEV_FROM_TIMER(timer_label)                                                    \
	DEVICE_DT_GET_OR_NULL(DT_CHILD(DT_NODELABEL(timer_label), counter))
#endif
struct counter_bee_top_data {
	counter_top_callback_t callback;
	void *user_data;
	uint32_t ticks;
};

struct counter_bee_alarm_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

#ifdef CONFIG_COUNTER_CAPTURE
struct counter_bee_capture {
	counter_capture_cb_t callback;
	void *user_data;
	counter_capture_flags_t flags;
	bool enabled;
};
#endif

struct counter_bee_data {
	struct counter_bee_top_data top;
	struct counter_bee_alarm_data alarm;
	uint32_t freq;
	const struct bee_timer_ops *ops;
#ifdef CONFIG_COUNTER_CAPTURE
	struct counter_bee_capture capture;
#endif
};

struct counter_bee_config {
	struct counter_config_info counter_info;
	uint32_t reg;
	uint16_t clkid;
	bool enhanced;
	uint8_t prescaler;
	uint8_t clock_div;
#ifdef CONFIG_COUNTER_CAPTURE
	/* NULL if the counter has no pinctrl-0 selecting the latch trigger pad. */
	const struct pinctrl_dev_config *pcfg;
	bool latch_debounce;
	uint8_t latch_debounce_div;
	uint16_t latch_debounce_count;
#endif
	void (*irq_config)(const struct device *dev);
#if defined(CONFIG_SOC_SERIES_RTL8752H)
	void (*shared_irq_config)(const struct device *dev);
#endif
	uint32_t (*get_irq_pending)(void);
};

static int counter_bee_timer_start(const struct device *dev)
{
	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	data->ops->start(cfg->reg);

	return 0;
}

static int counter_bee_timer_stop(const struct device *dev)
{
	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	data->ops->stop(cfg->reg);

	return 0;
}

static uint32_t counter_bee_timer_get_top_value(const struct device *dev)
{
	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	return data->ops->get_top(cfg->reg);
}

static int counter_bee_timer_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	*ticks = data->ops->get_count(cfg->reg);

	LOG_DBG("%s ticks=%x", dev->name, *ticks);

	return 0;
}

static int counter_bee_timer_set_top_value(const struct device *dev,
					   const struct counter_top_cfg *top_cfg)
{
	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	if (top_cfg == NULL) {
		LOG_ERR("There is no top_cfg");
		return -EINVAL;
	}

	if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		LOG_ERR("Unsupport setting top value without resetting counter");
		return -ENOTSUP;
	}

	if (data->alarm.callback) {
		LOG_ERR("Alarm is active");
		return -EBUSY;
	}

	if (top_cfg->ticks <= 1) {
		LOG_ERR("Top ticks too short");
		return -EBUSY;
	}

	data->ops->stop(cfg->reg);
	data->ops->int_disable(cfg->reg);
	data->ops->set_top(cfg->reg, top_cfg->ticks);
	data->ops->int_clear(cfg->reg);

	data->top.callback = top_cfg->callback;
	data->top.user_data = top_cfg->user_data;
	data->top.ticks = top_cfg->ticks;

	if (top_cfg->callback) {
		data->ops->int_enable(cfg->reg);
	}

	data->ops->start(cfg->reg);

	return 0;
}

static int counter_bee_timer_set_alarm(const struct device *dev, uint8_t chan,
				       const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(chan);

	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	if (alarm_cfg == NULL) {
		LOG_ERR("There is no alarm_cfg");
		return -EINVAL;
	}

	if (!alarm_cfg->callback) {
		LOG_ERR("There is no alarm callback");
		return -EINVAL;
	}

	if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		LOG_ERR("Unsupport absolute alarm");
		return -ENOTSUP;
	}

	if (data->alarm.callback) {
		LOG_ERR("Alarm is already active");
		return -EBUSY;
	}

	if (alarm_cfg->ticks > data->top.ticks) {
		LOG_ERR("Alarm ticks exceed top ticks");
		return -EINVAL;
	}

	if (alarm_cfg->ticks <= 1) {
		LOG_ERR("Alarm ticks too short");
		return -EINVAL;
	}

	data->ops->stop(cfg->reg);
	data->ops->set_top(cfg->reg, alarm_cfg->ticks);
	data->ops->int_disable(cfg->reg);
	data->ops->int_clear(cfg->reg);

	data->alarm.callback = alarm_cfg->callback;
	data->alarm.user_data = alarm_cfg->user_data;

	data->ops->int_enable(cfg->reg);
	data->ops->start(cfg->reg);

	return 0;
}

static int counter_bee_timer_cancel_alarm(const struct device *dev, uint8_t chan)
{
	ARG_UNUSED(chan);

	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	data->alarm.callback = NULL;
	data->alarm.user_data = NULL;

	data->ops->stop(cfg->reg);

	return 0;
}

static uint32_t counter_bee_timer_get_pending_int(const struct device *dev)
{
	const struct counter_bee_config *cfg = dev->config;

	return cfg->get_irq_pending();
}

static uint32_t counter_bee_timer_get_freq(const struct device *dev)
{
	struct counter_bee_data *data = dev->data;

	return data->freq;
}

#ifdef CONFIG_COUNTER_CAPTURE
/* Capture channel 0 is backed by the latch count triggered by a GPIO pad. */
static int counter_bee_capture_configure(const struct device *dev, uint8_t chan_id,
					 counter_capture_flags_t flags, counter_capture_cb_t cb,
					 void *user_data)
{
	ARG_UNUSED(chan_id);

	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;
	struct bee_latch_cfg latch_cfg;
	const struct pinctrl_state *state;
	uint32_t edge_flags = flags & COUNTER_CAPTURE_BOTH_EDGES;
	int ret;

	if (data->ops->latch_configure == NULL || cfg->pcfg == NULL) {
		LOG_ERR("Capture is not supported on this timer");
		return -ENOTSUP;
	}

	if (cb == NULL || edge_flags == 0U) {
		return -EINVAL;
	}

	if (data->capture.enabled) {
		LOG_ERR("Capture is already enabled");
		return -EBUSY;
	}

	if (edge_flags == COUNTER_CAPTURE_BOTH_EDGES) {
		latch_cfg.trigger = BEE_LATCH_TRIGGER_BOTH;
	} else if (edge_flags & COUNTER_CAPTURE_FALLING_EDGE) {
		latch_cfg.trigger = BEE_LATCH_TRIGGER_FALLING;
	} else {
		latch_cfg.trigger = BEE_LATCH_TRIGGER_RISING;
	}

	/* The trigger pad is the one pad of the default state, so it follows
	 * pinctrl_update_states().
	 */
	ret = pinctrl_lookup_state(cfg->pcfg, PINCTRL_STATE_DEFAULT, &state);
	if (ret < 0) {
		return ret;
	}

	if (state->pin_cnt != 1U) {
		LOG_ERR("Capture needs exactly one trigger pad");
		return -EINVAL;
	}

	ret = pinctrl_apply_state_direct(cfg->pcfg, state);
	if (ret < 0) {
		return ret;
	}

	latch_cfg.trigger_pad = state->pins[0].pin;
	latch_cfg.debounce_en = cfg->latch_debounce;
	latch_cfg.debounce_div = cfg->latch_debounce_div;
	latch_cfg.debounce_count = cfg->latch_debounce_count;

	ret = data->ops->latch_configure(cfg->reg, &latch_cfg);
	if (ret < 0) {
		LOG_ERR("Invalid latch configuration (%d)", ret);
		return ret;
	}

	data->capture.callback = cb;
	data->capture.user_data = user_data;
	data->capture.flags = flags;

	return 0;
}

static int counter_bee_capture_enable(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(chan_id);

	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	if (data->ops->latch_enable == NULL) {
		return -ENOTSUP;
	}

	if (data->capture.callback == NULL) {
		LOG_ERR("Capture is not configured");
		return -EINVAL;
	}

	/* Drop values latched before the capture was enabled. */
	data->ops->latch_clear(cfg->reg);
	data->capture.enabled = true;
	data->ops->latch_enable(cfg->reg);

	return 0;
}

static int counter_bee_capture_disable(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(chan_id);

	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;

	if (data->ops->latch_disable == NULL) {
		return -ENOTSUP;
	}

	data->ops->latch_disable(cfg->reg);
	data->capture.enabled = false;
	data->capture.callback = NULL;

	return 0;
}

static void counter_bee_capture_isr(const struct device *dev)
{
	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;
	struct counter_bee_capture *cap = &data->capture;

	while (cap->enabled && data->ops->latch_get_fifo_len(cfg->reg) > 0U) {
		uint32_t ticks = data->ops->latch_read_fifo(cfg->reg);
		counter_capture_cb_t cb = cap->callback;
		void *user_data = cap->user_data;
		counter_capture_flags_t flags = cap->flags;

		if (flags & COUNTER_CAPTURE_SINGLE_SHOT) {
			counter_bee_capture_disable(dev, 0);
			data->ops->latch_clear(cfg->reg);
			cb(dev, 0, flags, ticks, user_data);
			return;
		}

		cb(dev, 0, flags, ticks, user_data);
	}

	/* The FIFO is empty now, so the threshold interrupt can be cleared. */
	data->ops->latch_int_clear(cfg->reg);
}
#endif /* CONFIG_COUNTER_CAPTURE */

static int counter_bee_timer_init(const struct device *dev)
{
	const struct counter_bee_config *cfg = dev->config;
	struct counter_bee_data *data = dev->data;
	uint32_t pclk = 40000000;

	data->ops = bee_timer_get_ops(cfg->enhanced);
	if (!data->ops) {
		LOG_ERR("Bee timer HAL not enabled");
		return -ENOTSUP;
	}

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

#if defined(CONFIG_SOC_SERIES_RTL87X2G) || defined(CONFIG_SOC_SERIES_RTL87X2J)
	cfg->irq_config(dev);
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
	if (dev == GET_COUNTER_DEV_FROM_TIMER(timer4) ||
	    dev == GET_COUNTER_DEV_FROM_TIMER(timer5)) {
		cfg->shared_irq_config(dev);
	} else {
		cfg->irq_config(dev);
	}
#endif

	data->freq = pclk / cfg->prescaler;
	data->top.ticks = cfg->counter_info.max_top_value;

	data->ops->init(cfg->reg, cfg->clock_div, cfg->counter_info.max_top_value,
			BEE_TIMER_MODE_COUNTER, NULL);

	return 0;
}

static void irq_handler(const struct device *dev)
{
	struct counter_bee_data *data = dev->data;
	const struct counter_bee_config *cfg = dev->config;

#ifdef CONFIG_COUNTER_CAPTURE
	/* The latch interrupt shares the timer IRQ line. */
	if (data->capture.enabled && data->ops->latch_int_status(cfg->reg)) {
		counter_bee_capture_isr(dev);
	}

	if (!data->ops->int_status(cfg->reg)) {
		return;
	}
#endif

	data->ops->int_clear(cfg->reg);

	if (data->top.callback) {
		data->top.callback(dev, data->top.user_data);
	}

	if (data->alarm.callback) {
		uint32_t ticks;
		counter_alarm_callback_t cb = data->alarm.callback;

		counter_bee_timer_get_value(dev, &ticks);
		data->alarm.callback = NULL;
		cb(dev, 0, ticks, data->alarm.user_data);
	}
}

#if defined(CONFIG_SOC_SERIES_RTL8752H)
static void shared_irq_handler(void)
{
	const struct device *dev;
	struct counter_bee_data *data;
	const struct counter_bee_config *cfg;

	dev = GET_COUNTER_DEV_FROM_TIMER(timer4);
	if (dev && device_is_ready(dev)) {
		data = dev->data;
		cfg = dev->config;
		if (data->ops->int_status(cfg->reg)) {
			irq_handler(dev);
		}
	}

	dev = GET_COUNTER_DEV_FROM_TIMER(timer5);
	if (dev && device_is_ready(dev)) {
		data = dev->data;
		cfg = dev->config;
		if (data->ops->int_status(cfg->reg)) {
			irq_handler(dev);
		}
	}
}
#endif

static DEVICE_API(counter, counter_bee_timer_driver_api) = {
	.start = counter_bee_timer_start,
	.stop = counter_bee_timer_stop,
	.get_value = counter_bee_timer_get_value,
	.set_top_value = counter_bee_timer_set_top_value,
	.get_top_value = counter_bee_timer_get_top_value,
	.set_alarm = counter_bee_timer_set_alarm,
	.cancel_alarm = counter_bee_timer_cancel_alarm,
	.get_freq = counter_bee_timer_get_freq,
	.get_pending_int = counter_bee_timer_get_pending_int,
#ifdef CONFIG_COUNTER_CAPTURE
	.capture_configure = counter_bee_capture_configure,
	.enable_capture = counter_bee_capture_enable,
	.disable_capture = counter_bee_capture_disable,
#endif
};

#define PARENT_NODE(n) DT_INST_PARENT(n)

#if defined(CONFIG_SOC_SERIES_RTL87X2G) || defined(CONFIG_SOC_SERIES_RTL87X2J)
#define TIMER_IRQ_HANDLER(index)                                                                   \
	static void irq_config_##index(const struct device *dev)                                   \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(PARENT_NODE(index)), DT_IRQ(PARENT_NODE(index), priority),     \
			    irq_handler, DEVICE_DT_INST_GET(index), 0);                            \
		irq_enable(DT_IRQN(PARENT_NODE(index)));                                           \
	}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define TIMER_IRQ_HANDLER(index)                                                                   \
	static void irq_config_##index(const struct device *dev)                                   \
	{                                                                                          \
		irq_connect_dynamic(DT_IRQN(PARENT_NODE(index)),                                   \
				    DT_IRQ(PARENT_NODE(index), priority), (void *)irq_handler,     \
				    DEVICE_DT_INST_GET(index), 0);                                 \
		irq_enable(DT_IRQN(PARENT_NODE(index)));                                           \
	}                                                                                          \
	static void shared_irq_config##index(const struct device *dev)                             \
	{                                                                                          \
		irq_disable(DT_IRQN(PARENT_NODE(index)));                                          \
		irq_connect_dynamic(DT_IRQN(PARENT_NODE(index)),                                   \
				    DT_IRQ(PARENT_NODE(index), priority),                          \
				    (void *)shared_irq_handler, NULL, 0);                          \
		irq_enable(DT_IRQN(PARENT_NODE(index)));                                           \
	}
#endif

#define TIMER_IRQ_CONFIG_FUNC(index)                                                               \
	TIMER_IRQ_HANDLER(index);                                                                  \
	static uint32_t get_irq_pending_##index(void)                                              \
	{                                                                                          \
		return NVIC_GetPendingIRQ(DT_IRQN(PARENT_NODE(index)));                            \
	}

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define TIMER_DIV_CONFIG(index)                                                                    \
	.clock_div = CONCAT(CLOCK_DIV_, DT_PROP_OR(PARENT_NODE(index), prescaler, 1))
#define TIMER_IRQ_CONFIG(index) .irq_config = irq_config_##index
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define TIMER_DIV_CONFIG(index)                                                                    \
	.clock_div = CONCAT(TIM_CLOCK_DIVIDER_, DT_PROP_OR(PARENT_NODE(index), prescaler, 1))
#define TIMER_IRQ_CONFIG(index)                                                                    \
	.irq_config = irq_config_##index, .shared_irq_config = shared_irq_config##index
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#define TIMER_DIV_CONFIG(index)                                                                    \
	.clock_div = CONCAT(TIMER_CLOCK_DIV_, DT_PROP_OR(PARENT_NODE(index), prescaler, 1))
#define TIMER_IRQ_CONFIG(index) .irq_config = irq_config_##index
#endif

#ifdef CONFIG_COUNTER_CAPTURE
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
#define BEE_LATCH_DEBOUNCE_DIV_CONFIG(index)                                                       \
	.latch_debounce_div =                                                                      \
		CONCAT(TIMER_CLOCK_DIV_, DT_INST_PROP(index, latch_debounce_div))
#else
#define BEE_LATCH_DEBOUNCE_DIV_CONFIG(index) .latch_debounce_div = 0
#endif
#define BEE_CAPTURE_PINCTRL_DEFINE(index)                                                          \
	IF_ENABLED(DT_INST_PINCTRL_HAS_IDX(index, 0), (PINCTRL_DT_INST_DEFINE(index);))
#define BEE_CAPTURE_CONFIG(index)                                                                  \
	.pcfg = COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(index, 0),                                     \
			    (PINCTRL_DT_INST_DEV_CONFIG_GET(index)), (NULL)),                      \
	.latch_debounce = DT_INST_PROP(index, latch_debounce),                                     \
	BEE_LATCH_DEBOUNCE_DIV_CONFIG(index),                                                      \
	.latch_debounce_count = DT_INST_PROP(index, latch_debounce_count),
#else
#define BEE_CAPTURE_PINCTRL_DEFINE(index)
#define BEE_CAPTURE_CONFIG(index)
#endif

#define BEE_COUNTER_TIMER_INIT(index)                                                              \
	TIMER_IRQ_CONFIG_FUNC(index);                                                              \
	BEE_CAPTURE_PINCTRL_DEFINE(index)                                                          \
	static struct timer_data_##index {                                                         \
		struct counter_bee_data data;                                                      \
		struct counter_bee_alarm_data alarm;                                               \
	} counter_bee_data_##index = {0};                                                          \
	static const struct counter_bee_config counter_bee_config_##index = {                      \
		.counter_info =                                                                    \
			{                                                                          \
				.max_top_value = UINT32_MAX,                                       \
				.flags = 0,                                                        \
				.freq = 40000000 / DT_PROP_OR(PARENT_NODE(index), prescaler, 1),   \
				.channels = 1,                                                     \
			},                                                                         \
		.reg = DT_REG_ADDR(PARENT_NODE(index)),                                            \
		.clkid = DT_CLOCKS_CELL(PARENT_NODE(index), id),                                   \
		.enhanced = DT_NODE_HAS_COMPAT(PARENT_NODE(index), realtek_bee_enhanced_timer),    \
		.prescaler = DT_PROP_OR(PARENT_NODE(index), prescaler, 1),                         \
		TIMER_DIV_CONFIG(index),                                                           \
		BEE_CAPTURE_CONFIG(index)                                                          \
		TIMER_IRQ_CONFIG(index),                                                           \
		.get_irq_pending = get_irq_pending_##index,                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, counter_bee_timer_init, NULL, &counter_bee_data_##index,      \
			      &counter_bee_config_##index, PRE_KERNEL_1,                           \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_bee_timer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BEE_COUNTER_TIMER_INIT);
