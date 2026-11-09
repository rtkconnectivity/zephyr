/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_pinmux.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_pinmux.h>
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#include <rtl_pinmux.h>
#include <rtl_wakeup.h>
#endif

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define BEE_PAD_SET_PULL(pin, stre)              Pad_SetPullStrength(pin, stre)
#define BEE_PAD_WAKEUP_ENABLE(pin, pol, en, deb) System_WakeUpPinEnable(pin, pol, en)
#define BEE_PAD_WAKEUP_DISABLE(pin)              System_WakeUpPinDisable(pin)
#define BEE_DRIVING_LEVEL0                       LEVEL0
#define BEE_DRIVING_LEVEL1                       LEVEL1
#define BEE_DRIVING_LEVEL2                       LEVEL2
#define BEE_DRIVING_LEVEL3                       LEVEL3
#define BEE_PAD_WAKEUP_HIGH                      PAD_WAKEUP_POL_HIGH
#define BEE_PAD_WAKEUP_LOW                       PAD_WAKEUP_POL_LOW
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define BEE_PAD_SET_PULL(pin, stre)              Pad_PullConfigValue(pin, stre)
#define BEE_PAD_WAKEUP_ENABLE(pin, pol, en, deb) System_WakeUpPinEnable(pin, pol, en, deb)
#define BEE_PAD_WAKEUP_DISABLE(pin)              System_WakeUpPinDisable(pin)
#define BEE_DRIVING_LEVEL0                       PAD_DRIVING_CURRENT_8_8mA
#define BEE_DRIVING_LEVEL1                       PAD_DRIVING_CURRENT_12_18mA
#define BEE_DRIVING_LEVEL2                       PAD_DRIVING_CURRENT_16_28mA
#define BEE_DRIVING_LEVEL3                       PAD_DRIVING_CURRENT_16_28mA
#define BEE_PAD_WAKEUP_HIGH                      PAD_WAKEUP_POL_HIGH
#define BEE_PAD_WAKEUP_LOW                       PAD_WAKEUP_POL_LOW
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#define BEE_PAD_SET_PULL(pin, stre)              Pad_SetPullStrength(pin, stre)
#define BEE_PAD_WAKEUP_ENABLE(pin, pol, en, deb) System_WakeUpPinEnable(pin, pol, en)
#define BEE_PAD_WAKEUP_DISABLE(pin)              System_WakeUpPinDisable(pin)
#define BEE_DRIVING_LEVEL0                       PAD_DRIVING_CURRENT_4mA
#define BEE_DRIVING_LEVEL1                       PAD_DRIVING_CURRENT_8mA
#define BEE_DRIVING_LEVEL2                       PAD_DRIVING_CURRENT_18mA
#define BEE_DRIVING_LEVEL3                       PAD_DRIVING_CURRENT_28mA
#define BEE_PAD_WAKEUP_HIGH                      SYSTEM_WAKEUP_POLARITY_HIGH
#define BEE_PAD_WAKEUP_LOW                       SYSTEM_WAKEUP_POLARITY_LOW
#endif

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint16_t cfg_fun = pin[0].fun;
	uint16_t cfg_pin = pin[0].pin;
	uint8_t cfg_dir = pin[0].dir ? PAD_OUT_ENABLE : PAD_OUT_DISABLE;
	uint8_t cfg_drv = pin[0].drive ? PAD_OUT_HIGH : PAD_OUT_LOW;
	uint8_t cfg_pull;
	uint8_t cfg_pull_strength = pin[0].pull_strength;
	uint8_t current_level = pin[0].current_level;
	if (pin[0].pull_dis) {
		cfg_pull = PAD_PULL_NONE;
	} else if (pin[0].pull_dir) {
		cfg_pull = PAD_PULL_UP;
	} else {
		cfg_pull = PAD_PULL_DOWN;
	}

	/* A sleep-hardware-state group only sets the pull the hardware applies
	 * on its own in DLPS. It follows the group that configures the same pad,
	 * since Pad_LPConfig() takes the active pull from the pad.
	 */
	if (pin[0].sleep_hardware_state) {
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
		Pad_LPConfig(cfg_pin, cfg_pull, ENABLE);
		Pad_ModeAutoSwitchCmd(cfg_pin, ENABLE);
#endif
		return;
	}

	/* set pull strength */
	BEE_PAD_SET_PULL(cfg_pin, cfg_pull_strength);

	/* set current level */
	switch (current_level) {
	case 0:
		Pad_SetDrivingCurrent(cfg_pin, BEE_DRIVING_LEVEL0);
		break;

	case 1:
		Pad_SetDrivingCurrent(cfg_pin, BEE_DRIVING_LEVEL1);
		break;

	case 2:
		Pad_SetDrivingCurrent(cfg_pin, BEE_DRIVING_LEVEL2);
		break;

	case 3:
		Pad_SetDrivingCurrent(cfg_pin, BEE_DRIVING_LEVEL3);
		break;

	default:
		break;
	}

	/* set pad and pinmux */
	if (cfg_fun == BEE_PWR_OFF) {
		/* Pinmux is per pad, so a stale function keeps the peripheral input routed */
		Pinmux_Deinit(cfg_pin);
		Pad_Config(cfg_pin, PAD_SW_MODE, PAD_NOT_PWRON, cfg_pull, cfg_dir, cfg_drv);
	} else if (cfg_fun == BEE_SW_MODE) {
		Pinmux_Deinit(cfg_pin);
		Pad_Config(cfg_pin, PAD_SW_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
	} else if (cfg_fun < BEE_PINMUX_MAX) {
		Pad_Config(cfg_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
		Pinmux_Config(cfg_pin, cfg_fun);
	} else if (cfg_fun > BEE_PWR_OFF) {
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		if (cfg_fun <= BEE_SDHC1_D7_P4_7) {
			Pad_Config(cfg_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir,
				   cfg_drv);
			Pad_Dedicated_Config(cfg_pin, ENABLE);
			Pinmux_HS_Config(SDHC_HS_MUX);
		} else {
			Pad_Config(cfg_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir,
				   cfg_drv);
			Pinmux_AON_Config(cfg_fun);
		}
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
		Pad_Config(cfg_pin, PAD_PON_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
		Pad_FunctionConfig(cfg_pin, cfg_fun - BEE_LPPWM_CH0);
#endif
	} else {
		__ASSERT(false, "Pinctrl configuration fail");
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}

int pinctrl_bee_release_state(const struct pinctrl_dev_config *config, uint8_t id)
{
	const struct pinctrl_state *state;
	int err;

	err = pinctrl_lookup_state(config, id, &state);
	if (err < 0) {
		return err;
	}

	for (uint8_t i = 0U; i < state->pin_cnt; i++) {
		pinctrl_soc_pin_t pin = state->pins[i];

		if (pin.sleep_hardware_state) {
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
			Pad_LPConfig(pin.pin, PAD_PULL_NONE, DISABLE);
#endif
			continue;
		}

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		if (pin.fun > BEE_SDHC1_D7_P4_7) {
			/* The AON mux only ORs the selection in, so clear the field here */
			AON_REG_WRITE(AON_REG3X_SYS, AON_REG_READ(AON_REG3X_SYS) &
							     ~((pin.fun & 0xFFU) << (pin.fun >> 8)));
		} else if (pin.fun > BEE_PWR_OFF) {
			Pad_Dedicated_Config(pin.pin, DISABLE);
		}
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
		/* An invalid pin number disables the PON function slot */
		if (pin.fun > BEE_PWR_OFF) {
			Pad_FunctionConfig(0xFF, pin.fun - BEE_LPPWM_CH0);
		}
#endif
		/* Pinmux is per pad, so a released pad must not keep feeding the peripheral */
		pin.fun = BEE_SW_MODE;
		pin.dir = 0U;
		pinctrl_configure_pin(&pin);
	}

	return 0;
}

int pinctrl_bee_release(const struct pinctrl_dev_config *config)
{
	int err;

	err = pinctrl_apply_state(config, PINCTRL_STATE_SLEEP);
	if (err != -ENOENT) {
		return err;
	}

	return pinctrl_bee_release_state(config, PINCTRL_STATE_DEFAULT);
}

void pinctrl_bee_wakeup_enable(uint8_t pin, uint8_t polarity, uint8_t type)
{
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	if (type == PINCTRL_BEE_WAKEUP_PPU) {
		if (polarity) {
			System_WakeUpPPUCmd(pin, BEE_PAD_WAKEUP_HIGH, ENABLE);
		} else {
			System_WakeUpPPUCmd(pin, BEE_PAD_WAKEUP_LOW, ENABLE);
		}
		return;
	}
#endif
	(void)type;

	if (polarity) {
		BEE_PAD_WAKEUP_ENABLE(pin, BEE_PAD_WAKEUP_HIGH, DISABLE, 0);
	} else {
		BEE_PAD_WAKEUP_ENABLE(pin, BEE_PAD_WAKEUP_LOW, DISABLE, 0);
	}
}

void pinctrl_bee_wakeup_config(uint8_t pin, uint8_t polarity, uint8_t type, bool enable)
{
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	if (type == PINCTRL_BEE_WAKEUP_PPU) {
		if (enable) {
			if (polarity) {
				System_WakeUpPPUCmd(pin, BEE_PAD_WAKEUP_HIGH, ENABLE);
			} else {
				System_WakeUpPPUCmd(pin, BEE_PAD_WAKEUP_LOW, ENABLE);
			}
		} else {
			System_WakeUpPPUCmd(pin, BEE_PAD_WAKEUP_LOW, DISABLE);
		}
		return;
	}
#endif
	(void)type;

	if (enable) {
		if (polarity) {
			BEE_PAD_WAKEUP_ENABLE(pin, BEE_PAD_WAKEUP_HIGH, DISABLE, 0);
		} else {
			BEE_PAD_WAKEUP_ENABLE(pin, BEE_PAD_WAKEUP_LOW, DISABLE, 0);
		}
	} else {
		BEE_PAD_WAKEUP_DISABLE(pin);
	}
}
