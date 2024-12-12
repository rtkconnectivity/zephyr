/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_rtc

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl8752h_clock_control.h>
#include <zephyr/sys/util.h>

#include <rtl876x_rtc.h>
#include <rtl876x_rcc.h>
#include <rtl876x_nvic.h>
#include <vector_table.h>

#include <zephyr/logging/log.h>

#include <stdbool.h>

#include "time.h"
#include <zephyr/sys/timeutil.h>

LOG_MODULE_REGISTER(rtc_rtl8752h, CONFIG_RTC_LOG_LEVEL);

#ifdef CONFIG_RTC_ALARM
struct rtc_rtl8752h_ch_data {
	rtc_alarm_callback callback;
	void *user_data;
	bool enabled;
	bool pending;
	time_t alarm_time;
	uint16_t mask;
	struct rtc_time orogin_time;
	uint32_t cycles;
	struct k_mutex lock;
};
#endif

#ifdef CONFIG_RTC_UPDATE
struct rtc_rtl8752h_updata_data {
	rtc_update_callback callback;
	void *user_data;
	bool enabled;
	uint32_t cur_cnt;
};
#endif

struct rtc_rtl8752h_data {
	uint32_t freq;
	time_t last_update_time_sec;
	uint32_t last_update_rtc_cnt;
	struct rtc_time last_update_time;
	struct k_mutex lock;
#ifdef CONFIG_RTC_UPDATE
	struct rtc_rtl8752h_updata_data update;
#endif
#ifdef CONFIG_RTC_ALARM
	struct rtc_rtl8752h_ch_data alarm[];
#endif
};

struct rtc_rtl8752h_config {
	uint32_t reg;
	uint32_t src_freq;
	uint32_t prescaler;
	uint8_t channels;
	void (*irq_config)(const struct device *dev);
	void (*set_irq_pending)(void);
	uint32_t (*get_irq_pending)(void);
};

#ifdef CONFIG_RTC_ALARM
static const uint32_t rtc_cmp_int_table[] = {RTC_INT_COMP0, RTC_INT_COMP1, RTC_INT_COMP2,
					     RTC_INT_COMP3};
#ifdef CONFIG_PM_DEVICE
static const uint32_t rtc_cmp_wk_table[] = {RTC_WK_CMP0, RTC_WK_CMP1, RTC_WK_CMP2, RTC_WK_CMP3};
#endif
#endif

static time_t cnt2sec(const struct device *dev, uint32_t cnt)
{
	struct rtc_rtl8752h_data *data = dev->data;

	return cnt / data->freq;
}

#ifdef CONFIG_RTC_ALARM
static int rtc_rtl8752h_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				       const struct rtc_time *timeptr);

static uint32_t sec2cnt(const struct device *dev, time_t sec)
{
	struct rtc_rtl8752h_data *data = dev->data;

	return sec * data->freq;
}
#endif

static int rtc_rtl8752h_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	const struct rtc_rtl8752h_config *cfg = dev->config;
	struct rtc_rtl8752h_data *data = dev->data;

	int err = 0;

	err = k_mutex_lock(&data->lock, K_NO_WAIT);
	if (err) {
		return err;
	}

	RTC_INTConfig(RTC_INT_OVF, DISABLE);

#ifdef CONFIG_PM_DEVICE
	RTC_WKConfig(RTC_WK_OVF, DISABLE);
#endif

	data->last_update_time_sec = timeutil_timegm((struct tm *)timeptr);

	data->last_update_rtc_cnt = RTC_GetCounter();

	LOG_DBG("[set_time] RTC_GetCounter %d line%d\n", RTC_GetCounter(), __LINE__);

#ifdef CONFIG_RTC_ALARM

	/* update alarm time if set alarm before set time*/
	for (uint8_t i = 0; i < cfg->channels; i++) {
		if (data->alarm[i].enabled) {
			rtc_rtl8752h_alarm_set_time(dev, i, data->alarm[i].mask,
						    &(data->alarm[i].orogin_time));
		}
	}
#endif

	RTC_INTConfig(RTC_INT_OVF, ENABLE);

#ifdef CONFIG_PM_DEVICE
	RTC_WKConfig(RTC_WK_OVF, ENABLE);
#endif

	k_mutex_unlock(&data->lock);

	return err;
}

static int rtc_rtl8752h_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct rtc_rtl8752h_data *data = dev->data;
	uint32_t elapsed_cnt;
	time_t temp_time_sec;

	if (data->last_update_rtc_cnt != UINT32_MAX) {
		elapsed_cnt = RTC_GetCounter() - data->last_update_rtc_cnt;
	} else {
		elapsed_cnt = RTC_GetCounter();
	}

	temp_time_sec = cnt2sec(dev, elapsed_cnt);
	temp_time_sec += data->last_update_time_sec;
	LOG_DBG("RTC_GetCounter()%d data->last_update_rtc_cnt%d cnt2sec(dev, elapsed_cnt)%lld\n",
		RTC_GetCounter(), data->last_update_rtc_cnt, cnt2sec(dev, elapsed_cnt));
	LOG_DBG("elapsed_cnt%d temp_time_sec%lld data->last_update_time_sec%lld\n", elapsed_cnt,
		temp_time_sec, (data->last_update_time_sec));

	gmtime_r(&temp_time_sec, (struct tm *)timeptr);

	timeptr->tm_isdst = -1;
	timeptr->tm_nsec = 0;

	return 0;
}

#ifdef CONFIG_RTC_ALARM
static int rtc_rtl8752h_alarm_get_supported_fields(const struct device *dev, uint16_t id,
						   uint16_t *mask)
{
	const struct rtc_rtl8752h_config *cfg = dev->config;

	if (id > cfg->channels) {
		return -EINVAL;
	}

	*mask = (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE |
		 RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY |
		 RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR);

	return 0;
}

static int rtc_rtl8752h_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				       const struct rtc_time *timeptr)
{
	const struct rtc_rtl8752h_config *cfg = dev->config;
	struct rtc_rtl8752h_data *data = dev->data;
	time_t temp_time_sec;
	time_t cur_time_sec;
	time_t alarm_sec;
	struct rtc_time temp_time;
	struct rtc_time cur_time;
	uint32_t alarm_cnt;

	if (id > cfg->channels || (mask && (timeptr == 0))) {
		return -EINVAL;
	}

	uint32_t key = irq_lock();

	if (mask == 0 || timeptr == 0) {
		RTC_INTConfig(rtc_cmp_int_table[id], DISABLE);

#ifdef CONFIG_PM_DEVICE
		RTC_WKConfig(rtc_cmp_wk_table[id], DISABLE);
#endif

		/* clear pending if callback is called or if alarm is disabled */
		data->alarm[id].pending = false;
		data->alarm[id].enabled = false;
		return 0;
	}

	rtc_rtl8752h_get_time(dev, &cur_time);
	cur_time_sec = timeutil_timegm((struct tm *)(&cur_time));

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		temp_time.tm_sec = timeptr->tm_sec;
	} else {
		temp_time.tm_sec = 0;
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		temp_time.tm_min = timeptr->tm_min;
	} else {
		temp_time.tm_min = cur_time.tm_min;
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		temp_time.tm_hour = timeptr->tm_hour;
	} else {
		temp_time.tm_hour = cur_time.tm_hour;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		temp_time.tm_mday = timeptr->tm_mday;
	} else {
		temp_time.tm_mday = cur_time.tm_mday;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTH) {
		temp_time.tm_mon = timeptr->tm_mon;
	} else {
		temp_time.tm_mon = cur_time.tm_mon;
	}

	if (mask & RTC_ALARM_TIME_MASK_YEAR) {
		temp_time.tm_year = timeptr->tm_year;
	} else {
		temp_time.tm_year = cur_time.tm_year;
	}

	temp_time_sec = timeutil_timegm((struct tm *)(&temp_time));

	alarm_sec = temp_time_sec - cur_time_sec;

	alarm_cnt = sec2cnt(dev, alarm_sec);

	data->alarm[id].mask = mask;
	data->alarm[id].orogin_time = *timeptr;
	data->alarm[id].pending = false;
	data->alarm[id].enabled = true;

	RTC_SetCompValue(id, RTC_GetCounter() + alarm_cnt);

	LOG_DBG("[alarm_set_time]  cur_time sec%d min%d hour%d line%d\n", cur_time.tm_sec,
		cur_time.tm_min, cur_time.tm_hour, __LINE__);
	LOG_DBG("[alarm_set_time] temp_time sec%d min%d hour %d line%d\n", temp_time.tm_sec,
		temp_time.tm_min, temp_time.tm_hour, __LINE__);
	LOG_DBG("[alarm_set_time] RTC_GetCounter %d, alarm_sec %lld, alarm_cnt %d line%d\n",
		RTC_GetCounter(), alarm_sec, alarm_cnt, __LINE__);

	RTC_INTConfig(rtc_cmp_int_table[id], ENABLE);

#ifdef CONFIG_PM_DEVICE
	RTC_WKConfig(rtc_cmp_wk_table[id], ENABLE);
#endif

	irq_unlock(key);

	return 0;
}

static int rtc_rtl8752h_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				       struct rtc_time *timeptr)
{
	const struct rtc_rtl8752h_config *cfg = dev->config;
	struct rtc_rtl8752h_data *data = dev->data;

	if (id > cfg->channels || !timeptr) {
		return -EINVAL;
	}

	if (data->alarm[id].enabled) {
		*mask = data->alarm[id].mask;
		*timeptr = data->alarm[id].orogin_time;
	}

	return 0;
}

static int rtc_rtl8752h_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_rtl8752h_data *data = dev->data;

	return data->alarm[id].pending;
}

static int rtc_rtl8752h_alarm_set_callback(const struct device *dev, uint16_t id,
					   rtc_alarm_callback callback, void *user_data)
{
	const struct rtc_rtl8752h_config *cfg = dev->config;
	struct rtc_rtl8752h_data *data = dev->data;

	if (id > cfg->channels) {
		return -EINVAL;
	}

	uint32_t key = irq_lock();

	data->alarm[id].callback = callback;
	data->alarm[id].user_data = user_data;

	irq_unlock(key);

	return 0;
}
#endif

#ifdef CONFIG_RTC_UPDATE
static int rtc_rtl8752h_update_set_callback(const struct device *dev, rtc_update_callback callback,
					    void *user_data)
{
	struct rtc_rtl8752h_data *data = dev->data;

	data->update.callback = callback;
	data->update.user_data = user_data;

	if (callback == NULL) {
		data->update.enabled = false;
		RTC_INTConfig(RTC_INT_TICK, DISABLE);

#ifdef CONFIG_PM_DEVICE
		RTC_WKConfig(RTC_WK_TICK, DISABLE);
#endif

		return 0;
	}

	data->update.enabled = true;
	RTC_INTConfig(RTC_INT_TICK, ENABLE);

#ifdef CONFIG_PM_DEVICE
	RTC_WKConfig(RTC_WK_TICK, ENABLE);
#endif

	return 0;
}
#endif

struct rtc_driver_api rtc_rtl8752h_driver_api = {
	.set_time = rtc_rtl8752h_set_time,
	.get_time = rtc_rtl8752h_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_rtl8752h_alarm_get_supported_fields,
	.alarm_set_time = rtc_rtl8752h_alarm_set_time,
	.alarm_get_time = rtc_rtl8752h_alarm_get_time,
	.alarm_is_pending = rtc_rtl8752h_alarm_is_pending,
	.alarm_set_callback = rtc_rtl8752h_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */
#ifdef CONFIG_RTC_UPDATE
	.update_set_callback = rtc_rtl8752h_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */

	/* RTC_CALIBRATION not supported */
};

#ifdef CONFIG_RTC_ALARM
static void alarm_irq_handle(const struct device *dev, uint32_t chan)
{
	struct rtc_rtl8752h_data *data = dev->data;
	struct rtc_rtl8752h_ch_data *alarm = &data->alarm[chan];
	rtc_alarm_callback cb;
	struct rtc_time cur_time;
	time_t cur_time_sec;

	rtc_rtl8752h_get_time(dev, &cur_time);
	cur_time_sec = timeutil_timegm((struct tm *)(&cur_time));

	LOG_DBG("chan=%d\n", chan);

	cb = alarm->callback;

	if (cur_time_sec >= alarm->alarm_time) {
		rtc_rtl8752h_alarm_set_time(dev, chan, data->alarm[chan].mask,
					    &(data->alarm[chan].orogin_time));
		alarm->pending = true;
		if (cb) {
			cb(dev, chan, alarm->user_data);

			/* clear pending if callback is called or if alarm is disabled */
			alarm->pending = false;
		}
	}

	RTC_ClearCompINT(chan);
}
#endif

static void rtc_irq_handler(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	const struct rtc_rtl8752h_config *cfg = dev->config;
	struct rtc_rtl8752h_data *data = dev->data;

	if (RTC_GetINTStatus(RTC_INT_OVF) == SET) {
		/* updata tm */
		if (data->last_update_rtc_cnt != UINT32_MAX) {
			/* first time enter isr after set time */

			data->last_update_time_sec += cnt2sec(dev, ~data->last_update_rtc_cnt);

			data->last_update_rtc_cnt = UINT32_MAX;
		} else {
			data->last_update_time_sec += cnt2sec(dev, UINT32_MAX);
		}

		RTC_ClearINTPendingBit(RTC_INT_OVF);
	}

#ifdef CONFIG_PM_DEVICE
	if (RTC_GetWakeupStatus(RTC_WK_OVF)) {
		RTC_ClearWakeupStatusBit(RTC_WK_OVF);
	}
#endif

#ifdef CONFIG_RTC_ALARM
	for (uint32_t i = 0; i < cfg->channels; i++) {
		if (RTC_GetINTStatus(rtc_cmp_int_table[i])) {
			alarm_irq_handle(dev, i);
		}

#ifdef CONFIG_PM_DEVICE
		if (RTC_GetWakeupStatus(rtc_cmp_wk_table[i])) {
			RTC_ClearWakeupStatusBit(rtc_cmp_wk_table[i]);
		}
#endif
	}
#endif

#ifdef CONFIG_RTC_UPDATE
	if (RTC_GetINTStatus(RTC_INT_TICK) == SET) {
		LOG_DBG("RTC_INT_TICK data->update.cur_cnt %d data->freq %d\n",
			data->update.cur_cnt, data->freq);
		data->update.cur_cnt++;

		if (data->update.cur_cnt >= data->freq) {
			data->update.cur_cnt = 0;
			if (data->update.callback) {
				data->update.callback(dev, data->update.user_data);
			}
		}

		RTC_ClearINTPendingBit(RTC_INT_TICK);
	}

#ifdef CONFIG_PM_DEVICE
	if (RTC_GetWakeupStatus(RTC_WK_TICK)) {
		RTC_ClearWakeupStatusBit(RTC_WK_TICK);
	}
#endif
#endif
}

static int rtc_rtl8752h_init(const struct device *dev)
{
	const struct rtc_rtl8752h_config *cfg = dev->config;
	struct rtc_rtl8752h_data *data = dev->data;

	LOG_DBG("counter rtc init\n");
	__ASSERT(cfg->prescaler <= 0xfff, "rtc prescaler should be less than 0xfff");

	k_mutex_init(&data->lock);

	data->freq = cfg->src_freq / cfg->prescaler;

	RTC_DeInit();
	RTC_SetPrescaler(cfg->prescaler - 1);
	RTC_ResetPrescalerCounter();
	RTC_ResetCounter();
	RTC_INTConfig(RTC_INT_OVF, ENABLE);
	RTC_NvCmd(ENABLE);
#ifdef CONFIG_PM_DEVICE
	RTC_WKConfig(RTC_WK_OVF, ENABLE);
	RTC_SystemWakeupConfig(ENABLE);
#endif
	RTC_Cmd(ENABLE);

	cfg->irq_config(dev);

	return 0;
}

#define RTC_IRQ_CONFIG(index)                                                                      \
	static void irq_config_##index(const struct device *dev)                                   \
	{                                                                                          \
		RamVectorTableUpdate(RTC_IRQn, rtc_irq_handler);                                   \
		NVIC_InitTypeDef NVIC_InitStruct;                                                  \
                                                                                                   \
		NVIC_InitStruct.NVIC_IRQChannel = RTC_IRQn;                                        \
		NVIC_InitStruct.NVIC_IRQChannelPriority = 2;                                       \
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;                                       \
		NVIC_Init(&NVIC_InitStruct);                                                       \
	}                                                                                          \
	static void set_irq_pending_##index(void)                                                  \
	{                                                                                          \
		(NVIC_SetPendingIRQ(DT_INST_IRQN(index)));                                         \
	}                                                                                          \
	static uint32_t get_irq_pending_##index(void)                                              \
	{                                                                                          \
		return NVIC_GetPendingIRQ(DT_INST_IRQN(index));                                    \
	}

#ifdef CONFIG_RTC_ALARM
#define RTC_RTL8752H_ALARM_INIT(index) .alarm[DT_INST_PROP(index, channels)] = {0},
#else
#define RTC_RTL8752H_ALARM_INIT(index)
#endif

#ifdef CONFIG_RTC_UPDATE
#define RTC_RTL8752H_UPDATE_INIT                                                                   \
	.update = {                                                                                \
		.callback = NULL,                                                                  \
		.user_data = NULL,                                                                 \
		.enabled = false,                                                                  \
		.cur_cnt = 0,                                                                      \
	},
#else
#define RTC_RTL8752H_UPDATE_INIT
#endif

#define RTC_RTL8752H_INIT(index)                                                                   \
	RTC_IRQ_CONFIG(index);                                                                     \
	static struct rtc_rtl8752h_data rtc_rtl8752h_data_##index = {                              \
		RTC_RTL8752H_UPDATE_INIT RTC_RTL8752H_ALARM_INIT(index)};                          \
	static const struct rtc_rtl8752h_config rtc_rtl8752h_config_##index = {                    \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.src_freq = 32000,                                                                 \
		.prescaler = DT_INST_PROP(index, prescaler),                                       \
		.channels = DT_INST_PROP(index, channels),                                         \
		.irq_config = irq_config_##index,                                                  \
		.set_irq_pending = set_irq_pending_##index,                                        \
		.get_irq_pending = get_irq_pending_##index,                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, rtc_rtl8752h_init, NULL, &rtc_rtl8752h_data_##index,          \
			      &rtc_rtl8752h_config_##index, PRE_KERNEL_1,                          \
			      CONFIG_RTC_INIT_PRIORITY, &rtc_rtl8752h_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_RTL8752H_INIT);
