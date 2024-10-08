/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_rtc

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl8752h_clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <rtl876x_rtc.h>
#include <rtl876x_rcc.h>

LOG_MODULE_REGISTER(counter_rtl8752h_rtc, CONFIG_COUNTER_LOG_LEVEL);

struct counter_rtl8752h_rtc_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct counter_rtl8752h_rtc_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
	uint32_t freq;
	struct counter_rtl8752h_rtc_ch_data alarm[];
};

struct counter_rtl8752h_rtc_config {
	struct counter_config_info counter_info;
	uint32_t reg;
	uint8_t prescaler;
	void (*irq_config)(const struct device *dev);
	void (*set_irq_pending)(void);
	uint32_t (*get_irq_pending)(void);
};

static const uint32_t rtc_cmp_int_table[] = {RTC_INT_COMP0, RTC_INT_COMP1, RTC_INT_COMP2,
					     RTC_INT_COMP3};

static bool rtc_late_int_table[] = {false, false, false, false};

static int counter_rtl8752h_rtc_start(const struct device *dev)
{
	LOG_DBG("counter start\n");
	RTC_Cmd(ENABLE);
	return 0;
}

static int counter_rtl8752h_rtc_stop(const struct device *dev)
{
	LOG_DBG("counter stop\n");
	RTC_Cmd(DISABLE);
	RTC_ResetCounter();
	RTC_ResetPrescalerCounter();
	return 0;
}

static int counter_rtl8752h_rtc_get_value(const struct device *dev, uint32_t *ticks)
{
	*ticks = RTC_GetCounter();

	LOG_DBG("ticks=%d\n", *ticks);
	return 0;
}

static uint32_t counter_rtl8752h_rtc_get_top_value(const struct device *dev)
{
	const struct counter_rtl8752h_rtc_config *cfg = dev->config;

	LOG_DBG("topvalue=%x\n", cfg->counter_info.max_top_value);
	return cfg->counter_info.max_top_value;
}

static int counter_rtl8752h_rtc_set_alarm(const struct device *dev, uint8_t chan,
					  const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_rtl8752h_rtc_data *data = dev->data;
	const struct counter_rtl8752h_rtc_config *cfg = dev->config;
	struct counter_rtl8752h_rtc_ch_data *chdata = &data->alarm[chan];
	uint32_t now, diff;

	if (alarm_cfg->ticks > counter_rtl8752h_rtc_get_top_value(dev)) {
		return -EINVAL;
	}

	if (chdata->callback) {
		return -EBUSY;
	}

	chdata->callback = alarm_cfg->callback;
	chdata->user_data = alarm_cfg->user_data;

	counter_rtl8752h_rtc_get_value(dev, &now);
	LOG_DBG("now=%d, ticks=%d\n", now, alarm_cfg->ticks);
	if (!(alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE)) {
		RTC_INTConfig(rtc_cmp_int_table[chan], ENABLE);
		RTC_SetCompValue(chan, alarm_cfg->ticks + now);
		return 0;
	}

	if (alarm_cfg->ticks > now) {
		RTC_INTConfig(rtc_cmp_int_table[chan], ENABLE);
		RTC_SetCompValue(chan, alarm_cfg->ticks);
		return 0;
	}

	if (alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) {
		diff = now - alarm_cfg->ticks;
		LOG_DBG("diff=%d, guard_period=%d\n", diff, data->guard_period);
		if (diff < data->guard_period) {
			rtc_late_int_table[chan] = true;
			cfg->set_irq_pending();
			RTC_INTConfig(rtc_cmp_int_table[chan], DISABLE);
			return -ETIME;
		}
	}
	chdata->callback = NULL;
	RTC_INTConfig(rtc_cmp_int_table[chan], DISABLE);
	return -ETIME;
}

static int counter_rtl8752h_rtc_cancel_alarm(const struct device *dev, uint8_t chan)
{
	struct counter_rtl8752h_rtc_data *data = dev->data;

	RTC_INTConfig(rtc_cmp_int_table[chan], DISABLE);
	data->alarm[chan].callback = NULL;

	LOG_DBG("chan=%d\n", chan);

	return 0;
}

static int counter_rtl8752h_rtc_set_top_value(const struct device *dev,
					      const struct counter_top_cfg *top_cfg)
{
	const struct counter_rtl8752h_rtc_config *cfg = dev->config;

	LOG_DBG("ticks=%x\n", top_cfg->ticks);
	if (top_cfg->ticks != cfg->counter_info.max_top_value) {
		LOG_ERR("Unspported set top value");
		return -ENOTSUP;
	}

	return 0;
}

static uint32_t counter_rtl8752h_rtc_get_pending_int(const struct device *dev)
{
	const struct counter_rtl8752h_rtc_config *cfg = dev->config;

	LOG_DBG("get pending\n");

	return cfg->get_irq_pending();
}

static uint32_t counter_rtl8752h_rtc_get_freq(const struct device *dev)
{
	struct counter_rtl8752h_rtc_data *data = dev->data;

	LOG_DBG("freq=%d\n", data->freq);

	return data->freq;
}

static uint32_t counter_rtl8752h_rtc_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_rtl8752h_rtc_data *data = dev->data;

	LOG_DBG("get guard period\n");

	return data->guard_period;
}

static int counter_rtl8752h_rtc_set_guard_period(const struct device *dev, uint32_t guard,
						 uint32_t flags)
{
	struct counter_rtl8752h_rtc_data *data = dev->data;

	LOG_DBG("guard=%d\n", guard);
	__ASSERT_NO_MSG(guard < counter_rtl8752h_rtc_get_top_value(dev));

	data->guard_period = guard;

	return 0;
}

static void alarm_irq_handle(const struct device *dev, uint32_t chan)
{
	struct counter_rtl8752h_rtc_data *data = dev->data;
	struct counter_rtl8752h_rtc_ch_data *alarm = &data->alarm[chan];
	counter_alarm_callback_t cb;
	uint32_t ticks;

	LOG_DBG("chan=%d\n", chan);
	counter_rtl8752h_rtc_get_value(dev, &ticks);
	cb = alarm->callback;
	alarm->callback = NULL;

	if (cb) {
		cb(dev, chan, ticks, alarm->user_data);
		rtc_late_int_table[chan] = false;
	}
	RTC_ClearCompINT(chan);
}

static void irq_handler(const struct device *dev)
{
	const struct counter_rtl8752h_rtc_config *cfg = dev->config;

	for (uint32_t i = 0; i < cfg->counter_info.channels; i++) {
		if (RTC_GetINTStatus(rtc_cmp_int_table[i]) || rtc_late_int_table[i]) {
			alarm_irq_handle(dev, i);
		}
	}
}

static int counter_rtl8752h_rtc_init(const struct device *dev)
{
	const struct counter_rtl8752h_rtc_config *cfg = dev->config;
	struct counter_rtl8752h_rtc_data *data = dev->data;

	/* use clock_control_get_rate if clock driver is available */
	uint32_t pclk = 32000;

	data->freq = pclk / cfg->prescaler;

	cfg->irq_config(dev);

	LOG_DBG("counter rtc init\n");

	__ASSERT(cfg->prescaler <= 0xfff, "rtc prescaler should be less than 0xfff");

	RTC_DeInit();
	RTC_SetPrescaler(cfg->prescaler - 1);
	RTC_ResetCounter();
	RTC_NvCmd(ENABLE);

	return 0;
}

static const struct counter_driver_api counter_rtl8752h_rtc_driver_api = {
	.start = counter_rtl8752h_rtc_start,
	.stop = counter_rtl8752h_rtc_stop,
	.get_value = counter_rtl8752h_rtc_get_value,
	.set_alarm = counter_rtl8752h_rtc_set_alarm,
	.cancel_alarm = counter_rtl8752h_rtc_cancel_alarm,
	.set_top_value = counter_rtl8752h_rtc_set_top_value,
	.get_pending_int = counter_rtl8752h_rtc_get_pending_int,
	.get_top_value = counter_rtl8752h_rtc_get_top_value,
	.get_guard_period = counter_rtl8752h_rtc_get_guard_period,
	.set_guard_period = counter_rtl8752h_rtc_set_guard_period,
	.get_freq = counter_rtl8752h_rtc_get_freq,
};

#define RTC_IRQ_CONFIG(index)                                                                      \
	static void irq_config_##index(const struct device *dev)                                   \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), irq_handler,        \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}                                                                                          \
	static void set_irq_pending_##index(void)                                                  \
	{                                                                                          \
		(NVIC_SetPendingIRQ(DT_INST_IRQN(index)));                                         \
	}                                                                                          \
	static uint32_t get_irq_pending_##index(void)                                              \
	{                                                                                          \
		return NVIC_GetPendingIRQ(DT_INST_IRQN(index));                                    \
	}

#define RTL8752H_RTC_INIT(index)                                                                   \
	RTC_IRQ_CONFIG(index);                                                                     \
	static struct rtc_data_##index {                                                           \
		struct counter_rtl8752h_rtc_data data;                                             \
		struct counter_rtl8752h_rtc_ch_data alarm[DT_INST_PROP(index, channels)];          \
	} counter_rtl8752h_rtc_data_##index = {0};                                                 \
	static const struct counter_rtl8752h_rtc_config counter_rtl8752h_rtc_config_##index = {    \
		.counter_info =                                                                    \
			{                                                                          \
				.max_top_value = UINT32_MAX,                                       \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.freq = 0,                                                         \
				.channels = DT_INST_PROP(index, channels),                         \
			},                                                                         \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.prescaler = DT_INST_PROP(index, prescaler),                                       \
		.irq_config = irq_config_##index,                                                  \
		.set_irq_pending = set_irq_pending_##index,                                        \
		.get_irq_pending = get_irq_pending_##index,                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, counter_rtl8752h_rtc_init, NULL,                              \
			      &counter_rtl8752h_rtc_data_##index,                                  \
			      &counter_rtl8752h_rtc_config_##index, PRE_KERNEL_1,                  \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_rtl8752h_rtc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTL8752H_RTC_INIT);
