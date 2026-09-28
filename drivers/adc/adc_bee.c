/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_adc

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_adc.h>
#include <adc_lib.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_adc.h>
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#include <rtl_adc.h>
#else
#error "Unsupported Realtek Bee SoC series"
#endif

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_bee, CONFIG_ADC_LOG_LEVEL);

#define ADC_BEE_RESOLUTION     12U
#define ADC_12BIT_FULL_SCALE   4096U
#define ADC_12BIT_MAX_VALUE    (ADC_12BIT_FULL_SCALE - 1U)
#define ADC_BEE_VREF_BYPASS_MV 900
#define ADC_BEE_VREF_DIVIDE_MV 3300

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define BEE_ADC_DATA_AVE ADC_DataAvgEn
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define ADC_BEE_RTL8752H_UNSUPPORTED_CH 6U
#define BEE_ADC_DATA_AVE                ADC_DataAvgEn
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#define BEE_ADC_DATA_AVE ADC_DataAverageEn
#endif

#if defined(CONFIG_PM_DEVICE) && !defined(CONFIG_REALTEK_BEE_HAS_PCK600)
#define BEE_ADC_PM_STORE 1
#endif

#if defined(BEE_ADC_PM_STORE)
#include <zephyr/pm/device.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
typedef ADCStoreReg_TypeDef adc_bee_store_reg_t;
#define BEE_ADC_REG_DIG_CTRL   ADC_DIG_CTRL
#define BEE_ADC_REG_SCHED_CTRL ADC_SCHED_CTRL
#define BEE_ADC_REG_CTRL_INT   ADC_CTRL_INT
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
typedef ADCStoreReg_TypeDef adc_bee_store_reg_t;
#define BEE_ADC_REG_DIG_CTRL   CR
#define BEE_ADC_REG_SCHED_CTRL SCHCR
#define BEE_ADC_REG_CTRL_INT   INTCR
#endif

#define BEE_ADC_STORE_IDX_DIG_CTRL   0
#define BEE_ADC_STORE_IDX_SCHED_CTRL 1
#define BEE_ADC_STORE_IDX_CTRL_INT   2

extern void ADC_DLPSEnter(void *PeriReg, void *StoreBuf);
extern void ADC_DLPSExit(void *PeriReg, void *StoreBuf);
#endif /* BEE_ADC_PM_STORE */

struct adc_bee_config {
	mm_reg_t reg;
	uint16_t clkid;
	uint8_t channels;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

struct adc_bee_data {
	struct adc_context ctx;
	const struct device *dev;
	uint32_t bypass_channels;
	uint32_t configured_channels;
	uint32_t active_channels;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
#if defined(BEE_ADC_PM_STORE)
	adc_bee_store_reg_t store_buf;
#endif
};

#if defined(BEE_ADC_PM_STORE)
#define BEE_ADC_STORE_ALL(reg, buf)   ADC_DLPSEnter(reg, buf)
#define BEE_ADC_RESTORE_ALL(reg, buf) ADC_DLPSExit(reg, buf)
#define BEE_ADC_STORE_SCHED(reg, buf)                                                              \
	((buf)->adc_reg[BEE_ADC_STORE_IDX_SCHED_CTRL] = (reg)->BEE_ADC_REG_SCHED_CTRL)
#define BEE_ADC_STORE_CTRL(reg, buf)                                                               \
	do {                                                                                       \
		(buf)->adc_reg[BEE_ADC_STORE_IDX_CTRL_INT] = (reg)->BEE_ADC_REG_CTRL_INT;          \
		(buf)->adc_reg[BEE_ADC_STORE_IDX_DIG_CTRL] = (reg)->BEE_ADC_REG_DIG_CTRL;          \
	} while (0)
#else
#define BEE_ADC_STORE_ALL(reg, buf)
#define BEE_ADC_RESTORE_ALL(reg, buf)
#define BEE_ADC_STORE_SCHED(reg, buf)
#define BEE_ADC_STORE_CTRL(reg, buf)
#endif /* BEE_ADC_PM_STORE */

static int adc_bee_validate_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = POPCOUNT(sequence->channels) * sizeof(uint16_t);

	if (sequence->options != NULL) {
		needed *= (1U + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int adc_bee_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_bee_data *data = dev->data;
	const struct adc_bee_config *cfg = dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)cfg->reg;
	uint32_t valid_mask;
	int ret;

	if (sequence->channels == 0U) {
		LOG_ERR("No channel selected");
		return -EINVAL;
	}

	if (sequence->resolution != ADC_BEE_RESOLUTION) {
		LOG_ERR("Unsupported resolution %u, only %u-bit supported", sequence->resolution,
			ADC_BEE_RESOLUTION);
		return -ENOTSUP;
	}

	if (sequence->oversampling != 0U) {
		LOG_ERR("Oversampling is not supported");
		return -ENOTSUP;
	}

	if (sequence->calibrate) {
		LOG_ERR("Calibration during read is not supported");
		return -ENOTSUP;
	}

	valid_mask = BIT_MASK(cfg->channels);
	if ((sequence->channels & ~valid_mask) != 0U) {
		LOG_ERR("Invalid channel mask: 0x%08x", sequence->channels);
		return -EINVAL;
	}

#if defined(CONFIG_SOC_SERIES_RTL8752H)
	if ((sequence->channels & BIT(ADC_BEE_RTL8752H_UNSUPPORTED_CH)) != 0U) {
		LOG_ERR("ADC channel %u is not available on rtl8752h",
			ADC_BEE_RTL8752H_UNSUPPORTED_CH);
		return -ENOTSUP;
	}
#endif

	if ((sequence->channels & ~data->configured_channels) != 0U) {
		LOG_ERR("Sequence contains channels that were not configured");
		return -EINVAL;
	}

	ret = adc_bee_validate_buffer_size(sequence);
	if (ret < 0) {
		LOG_ERR("Buffer too small for requested sequence");
		return ret;
	}

	data->active_channels = sequence->channels;

	for (uint8_t i = 0U; i < cfg->channels; i++) {
		if ((sequence->channels & BIT(i)) == 0U) {
			continue;
		}

		if ((data->bypass_channels & BIT(i)) != 0U) {
			ADC_BypassCmd(i, ENABLE);
		} else {
			ADC_BypassCmd(i, DISABLE);
		}
	}

	ADC_BitMapConfig(adc, sequence->channels);
	BEE_ADC_STORE_SCHED(adc, &data->store_buf);

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_bee_read(const struct device *dev, const struct adc_sequence *seq)
{
	struct adc_bee_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_bee_start_read(dev, seq);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_bee_read_async(const struct device *dev, const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct adc_bee_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = adc_bee_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif

static int adc_bee_channel_setup(const struct device *dev, const struct adc_channel_cfg *chan_cfg)
{
	struct adc_bee_data *data = dev->data;
	const struct adc_bee_config *cfg = dev->config;

	if (chan_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported gain, only ADC_GAIN_1 is supported");
		return -ENOTSUP;
	}

	if (chan_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported reference, only ADC_REF_INTERNAL is supported");
		return -ENOTSUP;
	}

	if (chan_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported acquisition time");
		return -ENOTSUP;
	}

	if (chan_cfg->differential) {
		LOG_ERR("Differential mode is not supported");
		return -ENOTSUP;
	}

	if (chan_cfg->channel_id >= cfg->channels) {
		LOG_ERR("Invalid channel id %u", chan_cfg->channel_id);
		return -EINVAL;
	}

#if defined(CONFIG_SOC_SERIES_RTL8752H)
	if (chan_cfg->channel_id == ADC_BEE_RTL8752H_UNSUPPORTED_CH) {
		LOG_ERR("Channel %u is not available on rtl8752h", chan_cfg->channel_id);
		return -ENOTSUP;
	}
#endif

	data->configured_channels |= BIT(chan_cfg->channel_id);

	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_bee_data *data = CONTAINER_OF(ctx, struct adc_bee_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_bee_config *cfg = dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)cfg->reg;

	data->repeat_buffer = data->buffer;

	ADC_INTConfig(adc, ADC_INT_ONE_SHOT_DONE, ENABLE);
	ADC_Cmd(adc, ADC_ONE_SHOT_MODE, ENABLE);
	BEE_ADC_STORE_CTRL(adc, &data->store_buf);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_bee_data *data = CONTAINER_OF(ctx, struct adc_bee_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_bee_isr(const struct device *dev)
{
	struct adc_bee_data *data = dev->data;
	const struct adc_bee_config *cfg = dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)cfg->reg;
#if !defined(CONFIG_SOC_SERIES_RTL87X2J)
	ADC_ErrorStatus error_status = NO_ERROR;
#endif

	if (!ADC_GetINTStatus(adc, ADC_INT_ONE_SHOT_DONE)) {
		return;
	}

	for (uint8_t i = 0U; i < cfg->channels; i++) {
		uint16_t raw_data;
		int32_t voltage;
		uint32_t norm_raw;

		if ((data->active_channels & BIT(i)) == 0U) {
			continue;
		}

		raw_data = (uint16_t)ADC_ReadRawData(adc, i);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
		voltage = raw_data;
#endif

		if ((data->bypass_channels & BIT(i)) != 0U) {
#if !defined(CONFIG_SOC_SERIES_RTL87X2J)
			voltage = ADC_GetVoltage(BYPASS_SINGLE_MODE, raw_data, &error_status);
#endif
			norm_raw = ((uint32_t)MAX(voltage, 0) * ADC_12BIT_FULL_SCALE) /
				   ADC_BEE_VREF_BYPASS_MV;
		} else {
#if !defined(CONFIG_SOC_SERIES_RTL87X2J)
			voltage = ADC_GetVoltage(DIVIDE_SINGLE_MODE, raw_data, &error_status);
#endif
			norm_raw = ((uint32_t)MAX(voltage, 0) * ADC_12BIT_FULL_SCALE) /
				   ADC_BEE_VREF_DIVIDE_MV;
		}

#if !defined(CONFIG_SOC_SERIES_RTL87X2J)
		if (error_status != NO_ERROR) {
			LOG_ERR("Voltage conversion error on channel %u: %d", i, error_status);
		}
#endif

		if (norm_raw > ADC_12BIT_MAX_VALUE) {
			norm_raw = ADC_12BIT_MAX_VALUE;
		}

		*data->buffer++ = (uint16_t)norm_raw;
	}

	ADC_Cmd(adc, ADC_ONE_SHOT_MODE, DISABLE);
	ADC_INTConfig(adc, ADC_INT_ONE_SHOT_DONE, DISABLE);
	ADC_ClearINTPendingBit(adc, ADC_INT_ONE_SHOT_DONE);
	BEE_ADC_STORE_CTRL(adc, &data->store_buf);

	adc_context_on_sampling_done(&data->ctx, dev);
}

#if defined(BEE_ADC_PM_STORE)
static int adc_bee_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct adc_bee_config *cfg = dev->config;
	struct adc_bee_data *data = dev->data;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		(void)clock_control_off(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
		if ((err < 0) && (err != -ENOENT)) {
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (err < 0) {
			return err;
		}

		(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

		BEE_ADC_RESTORE_ALL((ADC_TypeDef *)cfg->reg, &data->store_buf);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* BEE_ADC_PM_STORE */

static DEVICE_API(adc, adc_bee_driver_api) = {
	.channel_setup = adc_bee_channel_setup,
	.read = adc_bee_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_bee_read_async,
#endif
};

static int adc_bee_init(const struct device *dev)
{
	struct adc_bee_data *data = dev->data;
	const struct adc_bee_config *cfg = dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)cfg->reg;
	ADC_InitTypeDef adc_init_struct;
	int ret;

	data->dev = dev;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);
	if (ret < 0) {
		return ret;
	}

#if !defined(CONFIG_SOC_SERIES_RTL87X2J)
	ADC_CalibrationInit();
#endif

	ADC_StructInit(&adc_init_struct);
	adc_init_struct.BEE_ADC_DATA_AVE = DISABLE;

#if defined(CONFIG_PM_DEVICE) && defined(CONFIG_REALTEK_BEE_HAS_PCK600)
	/* Disable power always on, or it will keep the  ADC Qactive on. */
	adc_init_struct.ADC_PowerAlwaysOnEn = DISABLE;
#else
	adc_init_struct.ADC_PowerAlwaysOnEn = ENABLE;
#endif

	for (uint8_t i = 0U; i < cfg->channels; i++) {
		adc_init_struct.ADC_SchIndex[i] = EXT_SINGLE_ENDED(i);
	}

#if defined(CONFIG_SOC_SERIES_RTL8752H)
	adc_init_struct.ADC_SchIndex[ADC_BEE_RTL8752H_UNSUPPORTED_CH] = 0;
#endif

	/* Reserve the last schedule channel for VBAT. */
	adc_init_struct.ADC_SchIndex[cfg->channels - 1U] = INTERNAL_VBAT_MODE;

	ADC_Init(adc, &adc_init_struct);

	BEE_ADC_STORE_ALL(adc, &data->store_buf);

#if defined(CONFIG_REALTEK_BEE_HAS_PCK600)
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
	if ((ret < 0) && (ret != -ENOENT)) {
		return ret;
	}
#endif

	cfg->irq_config_func(dev);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#if defined(BEE_ADC_PM_STORE)
#define BEE_ADC_PM_DEFINE(index) PM_DEVICE_DT_INST_DEFINE(index, adc_bee_pm_action);
#define BEE_ADC_PM_GET(index)    PM_DEVICE_DT_INST_GET(index)
#else
#define BEE_ADC_PM_DEFINE(index)
#define BEE_ADC_PM_GET(index) NULL
#endif

#define BEE_ADC_INIT(index)                                                                        \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	static void adc_bee_irq_config_func(const struct device *dev)                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), adc_bee_isr,        \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}                                                                                          \
	static struct adc_bee_data adc_bee_data_##index = {                                        \
		ADC_CONTEXT_INIT_TIMER(adc_bee_data_##index, ctx),                                 \
		ADC_CONTEXT_INIT_LOCK(adc_bee_data_##index, ctx),                                  \
		ADC_CONTEXT_INIT_SYNC(adc_bee_data_##index, ctx),                                  \
		.bypass_channels = DT_INST_PROP_OR(index, bypass_mode_map, 0),                     \
	};                                                                                         \
	static const struct adc_bee_config adc_bee_config_##index = {                              \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.channels = DT_INST_PROP(index, channels),                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.irq_config_func = adc_bee_irq_config_func,                                        \
	};                                                                                         \
	BEE_ADC_PM_DEFINE(index)                                                                   \
	DEVICE_DT_INST_DEFINE(index, adc_bee_init, BEE_ADC_PM_GET(index), &adc_bee_data_##index,   \
			      &adc_bee_config_##index, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,      \
			      &adc_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BEE_ADC_INIT)
