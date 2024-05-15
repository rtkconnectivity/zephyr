/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_pwm

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <rtl_tim.h>
#include <rtl_enh_tim.h>
#include <rtl_rcc.h>
#ifdef CONFIG_PM_DEVICE
#include <rtl_pinmux.h>
#endif
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_rtl87x2g, CONFIG_PWM_LOG_LEVEL);

/** PWM data. */
struct pwm_rtl87x2g_data
{
    /** Timer clock (Hz). */
    uint32_t tim_clk;
#ifdef CONFIG_PM_DEVICE
    union
    {
        TIMStoreReg_Typedef tim_store_buf;
        ENHTIMStoreReg_Typedef enhtim_store_buf;
    } store_buf;
    bool is_high_duty;
#endif
};

/** PWM configuration. */
struct pwm_rtl87x2g_config
{
    uint32_t reg;
    uint8_t channels;
    bool is_enhanced;
    uint16_t prescaler;
    uint16_t clkid;
    const struct pinctrl_dev_config *pcfg;
};

static int pwm_rtl87x2g_set_cycles(const struct device *dev, uint32_t channel,
                                   uint32_t period_cycles, uint32_t pulse_cycles,
                                   pwm_flags_t flags)
{
    LOG_DBG("channel=%d, period_cycles=%x, pulse_cycles=%x, flags=%x\n", \
            channel, period_cycles, pulse_cycles, flags);
    const struct pwm_rtl87x2g_config *config = dev->config;
#ifdef CONFIG_PM_DEVICE
    struct pwm_rtl87x2g_data *data = dev->data;
#endif
    void *timer_base = (void *)config->reg;

    if (channel > config->channels)
    {
        LOG_ERR("Invalid channel (%d)", channel);
        return -EINVAL;
    }

    if (config->is_enhanced)
    {
        if (flags & PWM_POLARITY_INVERTED)
        {
            if (period_cycles == 0U || pulse_cycles == 0U)
            {
                /* duty = 0 */
                ENHTIM_SetCCValue((ENHTIM_TypeDef *)timer_base, 0);
            }
            else if (period_cycles == pulse_cycles)
            {
                /* duty = 100 */
                ENHTIM_SetMaxCount((ENHTIM_TypeDef *)timer_base, UINT32_MAX - 1);
                ENHTIM_SetCCValue((ENHTIM_TypeDef *)timer_base, UINT32_MAX);
            }
            else
            {
                ENHTIM_SetMaxCount((ENHTIM_TypeDef *)timer_base, period_cycles);
                ENHTIM_SetCCValue((ENHTIM_TypeDef *)timer_base, period_cycles - pulse_cycles);
            }

        }

        else
        {
            if (period_cycles == 0U || pulse_cycles == 0U)
            {
                /* duty = 0 */
                ENHTIM_SetMaxCount((ENHTIM_TypeDef *)timer_base, UINT32_MAX - 1);
                ENHTIM_SetCCValue((ENHTIM_TypeDef *)timer_base, UINT32_MAX);
            }
            else if (period_cycles == pulse_cycles)
            {
                /* duty = 100 */
                ENHTIM_SetCCValue((ENHTIM_TypeDef *)timer_base, 0);
            }
            else
            {
                ENHTIM_SetMaxCount((ENHTIM_TypeDef *)timer_base, period_cycles);
                ENHTIM_SetCCValue((ENHTIM_TypeDef *)timer_base, pulse_cycles);
            }
        }

        ENHTIM_Cmd((ENHTIM_TypeDef *)timer_base, DISABLE);
        ENHTIM_Cmd((ENHTIM_TypeDef *)timer_base, ENABLE);
    }
    else
    {
        if (flags & PWM_POLARITY_INVERTED)
        {
            if (period_cycles == 0U || pulse_cycles == 0U)
            {
                /* duty = 0 */
                TIM_PWMChangeFreqAndDuty((TIM_TypeDef *)timer_base, \
                                         UINT32_MAX, 0);
            }
            else if (period_cycles == pulse_cycles)
            {
                /* duty = 100 */
                TIM_PWMChangeFreqAndDuty((TIM_TypeDef *)timer_base, \
                                         0, UINT32_MAX);
            }
            else
            {
                TIM_PWMChangeFreqAndDuty((TIM_TypeDef *)timer_base, \
                                         period_cycles - pulse_cycles, pulse_cycles);
            }
        }
        else
        {
            if (period_cycles == 0U || pulse_cycles == 0U)
            {
                /* duty = 0 */
                TIM_PWMChangeFreqAndDuty((TIM_TypeDef *)timer_base, \
                                         0, UINT32_MAX);
            }
            else if (period_cycles == pulse_cycles)
            {
                /* duty = 100 */
                TIM_PWMChangeFreqAndDuty((TIM_TypeDef *)timer_base, \
                                         UINT32_MAX, 0);
            }
            else
            {
                TIM_PWMChangeFreqAndDuty((TIM_TypeDef *)timer_base, \
                                         pulse_cycles, period_cycles - pulse_cycles);
            }
        }
        TIM_Cmd((TIM_TypeDef *)timer_base, DISABLE);
        TIM_Cmd((TIM_TypeDef *)timer_base, ENABLE);
    }

#ifdef CONFIG_PM_DEVICE
            data->is_high_duty = (pulse_cycles > (period_cycles >> 1));
            if ((flags & PWM_POLARITY_INVERTED))
            {
                data->is_high_duty = !data->is_high_duty;
            }
#endif

    return 0;
}

static int pwm_rtl87x2g_get_cycles_per_sec(const struct device *dev,
                                           uint32_t channel, uint64_t *cycles)
{
    struct pwm_rtl87x2g_data *data = dev->data;
    const struct pwm_rtl87x2g_config *config = dev->config;

    *cycles = (uint64_t)(data->tim_clk / config->prescaler);

    LOG_DBG("channel=%d, cycles=%d\n", channel, (uint32_t)*cycles);
    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int pwm_rtl87x2g_pm_action(const struct device *dev,
                                  enum pm_device_action action)
{
    const struct pwm_rtl87x2g_config *config = dev->config;
    struct pwm_rtl87x2g_data *data = dev->data;
    void *timer_base = (void *)config->reg;
    int err;

    extern void ENHTIM_DLPSEnter(void *PeriReg, void *StoreBuf);
    extern void ENHTIM_DLPSExit(void *PeriReg, void *StoreBuf);
    extern void TIM_DLPSEnter(void *PeriReg, void *StoreBuf);
    extern void TIM_DLPSExit(void *PeriReg, void *StoreBuf);

    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:

        if (config->is_enhanced)
        {
            ENHTIM_DLPSEnter(timer_base, &data->store_buf);
        }
        else
        {
            TIM_DLPSEnter(timer_base, &data->store_buf);
        }

        /* Move pins to sleep state */
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
        if (err == -ENOENT)
        {
            /* sleep status pinctrl is not configured, maintain the level at closest before sleep status */
            const struct pinctrl_state *state;
            err = pinctrl_lookup_state(config->pcfg, PINCTRL_STATE_DEFAULT, &state);
            if (err < 0)
            {
                return err;
            }
            Pad_Config(state->pins[0].pin, PAD_SW_MODE, PAD_IS_PWRON, 0, PAD_OUT_ENABLE, data->is_high_duty);
        }
        else if (err < 0)
        {
            return err;
        }

        break;
    case PM_DEVICE_ACTION_RESUME:
        /* Set pins to active state */
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (err < 0)
        {
            return err;
        }

        if (config->is_enhanced)
        {
            ENHTIM_DLPSExit(timer_base, &data->store_buf);
        }
        else
        {
            TIM_DLPSExit(timer_base, &data->store_buf);
        }

        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct pwm_driver_api pwm_rtl87x2g_driver_api =
{
    .set_cycles = pwm_rtl87x2g_set_cycles,
    .get_cycles_per_sec = pwm_rtl87x2g_get_cycles_per_sec,
};

static int pwm_rtl87x2g_init(const struct device *dev)
{
    const struct pwm_rtl87x2g_config *config = dev->config;
    struct pwm_rtl87x2g_data *data = dev->data;
    void *timer_base = (void *)config->reg;
    uint8_t clock_div;
    int ret;

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&config->clkid);

    data->tim_clk = 40000000;

#ifdef CONFIG_PM_DEVICE
    data->is_high_duty = false;
#endif

    /* apply pin configuration */
    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0)
    {
        return ret;
    }

    switch (config->prescaler)
    {
    case 1:
        clock_div = CLOCK_DIV_1;
        break;
    case 2:
        clock_div = CLOCK_DIV_2;
        break;
    case 4:
        clock_div = CLOCK_DIV_4;
        break;
    case 8:
        clock_div = CLOCK_DIV_8;
        break;
    case 16:
        clock_div = CLOCK_DIV_16;
        break;
    case 32:
        clock_div = CLOCK_DIV_32;
        break;
    case 40:
        clock_div = CLOCK_DIV_40;
        break;
    case 64:
        clock_div = CLOCK_DIV_64;
        break;
    default:
        clock_div = CLOCK_DIV_1;
        break;
    }
    if (config->is_enhanced)
    {
        ENHTIM_InitTypeDef enh_tim_init_struct;
        ENHTIM_StructInit(&enh_tim_init_struct);
        enh_tim_init_struct.ENHTIM_ClockSource = CK_40M_TIMER;
        enh_tim_init_struct.ENHTIM_ClockDiv_En = ENABLE;
        enh_tim_init_struct.ENHTIM_ClockDiv = clock_div;
        enh_tim_init_struct.ENHTIM_Mode = ENHTIM_MODE_PWM_MANUAL;
        enh_tim_init_struct.ENHTIM_PWMOutputEn = ENABLE;
        enh_tim_init_struct.ENHTIM_PWMStartPolarity = ENHTIM_PWM_START_WITH_LOW;
        enh_tim_init_struct.ENHTIM_MaxCount = UINT32_MAX;
        enh_tim_init_struct.ENHTIM_CCValue = UINT32_MAX;
        ENHTIM_Init((ENHTIM_TypeDef *)timer_base, &enh_tim_init_struct);
    }
    else
    {
        TIM_TimeBaseInitTypeDef timer_init_struct;
        TIM_StructInit(&timer_init_struct);
        timer_init_struct.TIM_ClockSrc = CK_40M_TIMER;
        timer_init_struct.TIM_SOURCE_DIV_En = ENABLE;
        timer_init_struct.TIM_SOURCE_DIV = clock_div;
        timer_init_struct.TIM_Mode = TIM_Mode_UserDefine;
        timer_init_struct.TIM_PWM_En = ENABLE;
        timer_init_struct.TIM_PWM_High_Count = 0;
        timer_init_struct.TIM_PWM_Low_Count = UINT32_MAX;
        TIM_TimeBaseInit((TIM_TypeDef *)timer_base, &timer_init_struct);
    }

    return 0;
}

#define PWM_RTL87X2G_INIT(index)                            \
    static struct pwm_rtl87x2g_data pwm_rtl87x2g_data_##index;                 \
    \
    PINCTRL_DT_INST_DEFINE(index);                         \
    \
    static const struct pwm_rtl87x2g_config pwm_rtl87x2g_config_##index = {        \
        .reg = DT_REG_ADDR(DT_INST_PARENT(index)),                 \
               .clkid = DT_CLOCKS_CELL(DT_INST_PARENT(index), id),         \
                        .prescaler = DT_PROP(DT_INST_PARENT(index), prescaler),        \
                                     .channels = DT_PROP(DT_INST_PARENT(index), channels),          \
                                                 .is_enhanced = DT_PROP(DT_INST_PARENT(index), is_enhanced),        \
                                                                .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),             \
    };                                     \
    \
     PM_DEVICE_DT_INST_DEFINE(index, pwm_rtl87x2g_pm_action);    \
     DEVICE_DT_INST_DEFINE(index, &pwm_rtl87x2g_init, PM_DEVICE_DT_INST_GET(index), &pwm_rtl87x2g_data_##index,     \
                          &pwm_rtl87x2g_config_##index, POST_KERNEL,           \
                          CONFIG_PWM_INIT_PRIORITY,                \
                          &pwm_rtl87x2g_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_RTL87X2G_INIT)
