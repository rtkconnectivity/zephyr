/*
* Copyright(c) 2020, Realtek Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*/

#define DT_DRV_COMPAT realtek_rtl87x2g_adc

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>

#include <rtl_adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_rtl87x2g, CONFIG_ADC_LOG_LEVEL);

struct adc_rtl87x2g_config
{
    uint32_t reg;
    uint16_t clkid;
    struct reset_dt_spec reset;
    uint8_t channels;
    const struct pinctrl_dev_config *pcfg;
    uint8_t irq_num;
    void (*irq_config_func)(const struct device *dev);
};

struct adc_rtl87x2g_data
{
    struct adc_context ctx;
    const struct device *dev;
    uint8_t seq_map;
    uint16_t *buffer;
    uint16_t *repeat_buffer;
};

static int adc_rtl87x2g_start_read(const struct device *dev,
                                   const struct adc_sequence *sequence)
{
    struct adc_rtl87x2g_data *data = dev->data;
    const struct adc_rtl87x2g_config *cfg = dev->config;
    ADC_TypeDef *adc = (ADC_TypeDef *)cfg->reg;

    if (sequence->resolution != 12U)
    {
        LOG_ERR("resolution is not valid");
        return -ENOTSUP;
    }

    data->seq_map = sequence->channels;

    ADC_BitMapConfig(adc, BIT(cfg->channels) - 1, DISABLE);
    ADC_BitMapConfig(adc, sequence->channels, ENABLE);

    data->buffer = sequence->buffer;

    adc_context_start_read(&data->ctx, sequence);

    return adc_context_wait_for_completion(&data->ctx);
}

static int adc_rtl87x2g_read(const struct device *dev, const struct adc_sequence *seq)
{
    struct adc_rtl87x2g_data *data = dev->data;
    int error;

    adc_context_lock(&data->ctx, false, NULL);
    error = adc_rtl87x2g_start_read(dev, seq);
    adc_context_release(&data->ctx, error);

    return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_rtl87x2g_read_async(const struct device *dev,
                                   const struct adc_sequence *sequence,
                                   struct k_poll_signal *async)
{
    struct adc_rtl87x2g_data *data = dev->data;
    int error;

    adc_context_lock(&data->ctx, true, async);
    error = adc_rtl87x2g_start_read(dev, sequence);
    adc_context_release(&data->ctx, error);

    return error;

}
#endif /* CONFIG_ADC_ASYNC */

static int adc_rtl87x2g_channel_setup(const struct device *dev,
                                      const struct adc_channel_cfg *chan_cfg)
{
    const struct adc_rtl87x2g_config *cfg = dev->config;

    if (chan_cfg->gain != ADC_GAIN_1)
    {
        LOG_ERR("Gain is not valid");
        return -ENOTSUP;
    }

    if (chan_cfg->reference != ADC_REF_INTERNAL)
    {
        LOG_ERR("Reference is not valid");
        return -ENOTSUP;
    }

    if (chan_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT)
    {
        LOG_ERR("Unsupported acquisition_time '%d'", chan_cfg->acquisition_time);
        return -ENOTSUP;
    }

    if (chan_cfg->channel_id >= cfg->channels)
    {
        LOG_ERR("Invalid channel (%u)", chan_cfg->channel_id);
        return -EINVAL;
    }

    if (chan_cfg->differential)
    {
        LOG_ERR("Differential sampling not supported");
        return -ENOTSUP;
    }

    return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
    struct adc_rtl87x2g_data *data = CONTAINER_OF(ctx, struct adc_rtl87x2g_data, ctx);
    const struct device *dev = data->dev;
    const struct adc_rtl87x2g_config *cfg = dev->config;
    ADC_TypeDef *adc = (ADC_TypeDef *)cfg->reg;

    data->repeat_buffer = data->buffer;

    ADC_INTConfig(adc, ADC_INT_ONE_SHOT_DONE, ENABLE);
    ADC_Cmd(adc, ADC_ONE_SHOT_MODE, ENABLE);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
                                              bool repeat_sampling)
{
    struct adc_rtl87x2g_data *data = CONTAINER_OF(ctx, struct adc_rtl87x2g_data, ctx);

    if (repeat_sampling)
    {
        data->buffer = data->repeat_buffer;
    }
}

static void adc_rtl87x2g_isr(const struct device *dev)
{
    struct adc_rtl87x2g_data *data = dev->data;
    const struct adc_rtl87x2g_config *cfg = dev->config;
    ADC_TypeDef *adc = (ADC_TypeDef *)cfg->reg;

    if (ADC_GetINTStatus(adc, ADC_INT_ONE_SHOT_DONE))
    {
        for (uint32_t i = 0; i < cfg->channels; i++)
        {
            if (data->seq_map & BIT(i))
            {
                *data->buffer++ = (uint16_t)ADC_ReadRawData(adc, i);
            }
        }

        ADC_Cmd(adc, ADC_ONE_SHOT_MODE, DISABLE);
        ADC_INTConfig(adc, ADC_INT_ONE_SHOT_DONE, DISABLE);
        ADC_ClearINTPendingBit(adc, ADC_INT_ONE_SHOT_DONE);
        adc_context_on_sampling_done(&data->ctx, dev);
    }
}

static const struct adc_driver_api adc_rtl87x2g_driver_api =
{
    .channel_setup = adc_rtl87x2g_channel_setup,
    .read          = adc_rtl87x2g_read,
#ifdef CONFIG_ADC_ASYNC
    .read_async    = adc_rtl87x2g_read_async,
#endif /* CONFIG_ADC_ASYNC */
};

static int adc_rtl87x2g_init(const struct device *dev)
{
    struct adc_rtl87x2g_data *data = dev->data;
    const struct adc_rtl87x2g_config *cfg = dev->config;
    int ret;

    data->dev = dev;

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0)
    {
        return ret;
    }

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&cfg->clkid);

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0)
    {
        return ret;
    }

    ADC_InitTypeDef adc_init_struct;
    ADC_StructInit(&adc_init_struct);
    adc_init_struct.ADC_DataAvgEn        = DISABLE;
    adc_init_struct.ADC_PowerAlwaysOnEn  = ENABLE;
    adc_init_struct.ADC_SampleTime       = 255;
    for (uint32_t i = 0; i < cfg->channels; i++)
    {
        adc_init_struct.ADC_SchIndex[i] = EXT_SINGLE_ENDED(i);
    }
    ADC_Init(ADC, &adc_init_struct);

    cfg->irq_config_func(dev);

    adc_context_unlock_unconditionally(&data->ctx);

    return 0;
}

#define RTL87X2G_ADC_INIT(index)                                    \
    PINCTRL_DT_INST_DEFINE(index);                              \
    static void adc_rtl87x2g_irq_config_func(const struct device *dev)   \
    {                                   \
        IRQ_CONNECT(DT_INST_IRQN(index),                \
                    DT_INST_IRQ(index, priority),               \
                    adc_rtl87x2g_isr, DEVICE_DT_INST_GET(index),       \
                    0);                         \
        irq_enable(DT_INST_IRQN(index));                \
    }             \
    static struct adc_rtl87x2g_data adc_rtl87x2g_data_##index = {                   \
        ADC_CONTEXT_INIT_TIMER(adc_rtl87x2g_data_##index, ctx),                 \
        ADC_CONTEXT_INIT_LOCK(adc_rtl87x2g_data_##index, ctx),                  \
        ADC_CONTEXT_INIT_SYNC(adc_rtl87x2g_data_##index, ctx),                  \
    };                                          \
    const static struct adc_rtl87x2g_config adc_rtl87x2g_config_##index = {             \
        .reg = DT_INST_REG_ADDR(index),                         \
               .clkid = DT_INST_CLOCKS_CELL(index, id),                        \
                        /* .reset = RESET_DT_SPEC_INST_GET(index), */                       \
                        .channels = DT_INST_PROP(index, channels),                      \
                                    .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                  \
                                            .irq_num = DT_INST_IRQN(index),                         \
                                                       .irq_config_func = adc_rtl87x2g_irq_config_func,                   \
    };                                          \
    DEVICE_DT_INST_DEFINE(index,                                \
                          &adc_rtl87x2g_init, NULL,                     \
                          &adc_rtl87x2g_data_##index, &adc_rtl87x2g_config_##index,             \
                          POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,                \
                          &adc_rtl87x2g_driver_api);                        \

DT_INST_FOREACH_STATUS_OKAY(RTL87X2G_ADC_INIT)
