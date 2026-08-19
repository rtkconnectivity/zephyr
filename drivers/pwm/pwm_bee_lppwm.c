/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_lppwm

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>

#include "rtl_lppwm.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_bee_lppwm, CONFIG_PWM_LOG_LEVEL);

/* The LPPWM is clocked from the always-on 32 kHz clock. */
#define LPPWM_SRC_CLOCK_HZ 32000U

/* The high and low counts are 16-bit register fields. */
#define LPPWM_COUNT_MAX 0xFFFFU

struct pwm_bee_lppwm_config {
	LPPWM_TypeDef *reg;
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
};

static int pwm_bee_lppwm_set_cycles(const struct device *dev, uint32_t channel,
				    uint32_t period_cycles, uint32_t pulse_cycles,
				    pwm_flags_t flags)
{
	const struct pwm_bee_lppwm_config *config = dev->config;
	LPPWM_InitTypeDef lppwm_init;

	LOG_DBG("channel=%u, period_cycles=%x, pulse_cycles=%x, flags=%x", channel, period_cycles,
		pulse_cycles, flags);

	/* The LPPWM only exposes a single channel (CH0). */
	if (channel != 0U) {
		return -EINVAL;
	}

	if (pulse_cycles > period_cycles) {
		return -EINVAL;
	}

	if (pulse_cycles > LPPWM_COUNT_MAX || (period_cycles - pulse_cycles) > LPPWM_COUNT_MAX) {
		return -EINVAL;
	}

	LPPWM_Reset(config->reg);

	LPPWM_StructInit(&lppwm_init);
	lppwm_init.LPPWM_Polarity = (flags & PWM_POLARITY_INVERTED) ? LPPWM_POLARITY_INVERT
							      : LPPWM_POLARITY_NORMAL;
	lppwm_init.LPPWM_PeriodHigh = pulse_cycles;
	lppwm_init.LPPWM_PeriodLow = period_cycles - pulse_cycles;

	LPPWM_Init(config->reg, &lppwm_init);
	LPPWM_Cmd(config->reg, ENABLE);

	return 0;
}

static int pwm_bee_lppwm_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					    uint64_t *cycles)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	*cycles = LPPWM_SRC_CLOCK_HZ;

	return 0;
}

static int pwm_bee_lppwm_init(const struct device *dev)
{
	const struct pwm_bee_lppwm_config *config = dev->config;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);

	return 0;
}

static DEVICE_API(pwm, pwm_bee_lppwm_driver_api) = {
	.set_cycles = pwm_bee_lppwm_set_cycles,
	.get_cycles_per_sec = pwm_bee_lppwm_get_cycles_per_sec,
};

#define PWM_BEE_LPPWM_INIT(index)                                                                  \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct pwm_bee_lppwm_config pwm_bee_lppwm_config_##index = {                  \
		.reg = (LPPWM_TypeDef *)DT_INST_REG_ADDR(index),                                   \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &pwm_bee_lppwm_init, NULL, NULL,                              \
			      &pwm_bee_lppwm_config_##index, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,\
			      &pwm_bee_lppwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_BEE_LPPWM_INIT)
