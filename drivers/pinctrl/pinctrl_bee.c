/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_pinmux.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_pinmux.h>
#endif

#include <trace.h>
#define DBG_DIRECT_SHOW 0

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define bee_pad_set_pull(pin, stre)       Pad_SetPullStrength(pin, stre)
#define bee_pad_wakeup(pin, pol, en, deb) System_WakeUpPinEnable(pin, pol, en)
#define BEE_DRIVING_LEVEL0 LEVEL0
#define BEE_DRIVING_LEVEL1 LEVEL1
#define BEE_DRIVING_LEVEL2 LEVEL2
#define BEE_DRIVING_LEVEL3 LEVEL3
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define bee_pad_set_pull(pin, stre)       Pad_PullConfigValue(pin, stre)
#define bee_pad_wakeup(pin, pol, en, deb) System_WakeUpPinEnable(pin, pol, en, deb)
#define BEE_DRIVING_LEVEL0 PAD_DRIVING_CURRENT_8_8mA
#define BEE_DRIVING_LEVEL1 PAD_DRIVING_CURRENT_12_18mA
#define BEE_DRIVING_LEVEL2 PAD_DRIVING_CURRENT_16_28mA
#define BEE_DRIVING_LEVEL3 PAD_DRIVING_CURRENT_16_28mA
#endif

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t cfg_fun = pin[0].fun;
	uint32_t cfg_pin = pin[0].pin;
	uint32_t cfg_dir = pin[0].dir;
	uint32_t cfg_drv = pin[0].drive;
	uint32_t cfg_pull = pin[0].pull;
	uint32_t cfg_pull_strength = pin[0].pull_strength;
	uint32_t cfg_wakeup_high = pin[0].wakeup_high;
	uint32_t cfg_wakeup_low = pin[0].wakeup_low;
	uint32_t current_level = pin[0].current_level;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] cfg_fun=%d, cfg_pin=%d, cfg_dir=%d,"
		   " cfg_drv=%d , cfg_pull=%d, cfg_pull_strength=%d, cfg_wakeup_high=%d, "
		   "cfg_wakeup_low=%d, current_level=%d",
		   __func__, cfg_fun, cfg_pin, cfg_dir, cfg_drv, cfg_pull, cfg_pull_strength,
		   cfg_wakeup_high, cfg_wakeup_low, current_level);
#endif

	bee_pad_set_pull(cfg_pin, cfg_pull_strength);
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

	if (cfg_fun == BEE_PWR_OFF) {
		Pad_Config(cfg_pin, PAD_SW_MODE, PAD_NOT_PWRON, cfg_pull, cfg_dir, cfg_drv);
	} else if (cfg_fun == BEE_SW_MODE) {
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
#endif
	}

	System_WakeUpPinDisable(cfg_pin);

	if (cfg_wakeup_high) {
		bee_pad_wakeup(cfg_pin, PAD_WAKEUP_POL_HIGH, DISABLE, 0);
	} else if (cfg_wakeup_low) {
		bee_pad_wakeup(cfg_pin, PAD_WAKEUP_POL_LOW, DISABLE, 0);
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] pin_cnt=%d", __func__, pin_cnt);
#endif
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}
