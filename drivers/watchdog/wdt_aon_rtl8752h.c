/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_aon_wdt

#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>

#include <rtl876x_aon_wdg.h>

LOG_MODULE_REGISTER(wdt_aon_rtl8752h, CONFIG_WDT_LOG_LEVEL);

#define AON_WDT_INITIAL_TIMEOUT DT_INST_PROP(0, initial_timeout_ms)

enum {
	RESET_WHOLE_CHIP_EXCEPT_AON_AND_RTC = 0,
	RESET_WHOLE_CHIP = 1
};

static uint32_t comp;
static uint8_t reset_level;

static int aon_wdt_rtl8752h_setup(const struct device *dev, uint8_t options)
{
	ARG_UNUSED(dev);

	/* 0, Stop count in low power mode; 1, Continue count in low power mode. */
	uint8_t cnt_ctl = 0;

	if (!(options & WDT_OPT_PAUSE_IN_SLEEP)) {
		cnt_ctl = 1;
	}

	if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG)) {
		/* Not support WDT_OPT_PAUSE_HALTED_BY_DBG */
		return -ENOTSUP;
	}

	/* Config wheather continue count in low power mode or not */
	AON_WDG_ConfigCntCtl(cnt_ctl);

	return 0;
}

static int aon_wdt_rtl8752h_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	AON_WDG_Disable();

	return 0;
}

static int aon_wdt_rtl8752h_install_timeout(const struct device *dev,
					    const struct wdt_timeout_cfg *config)
{
	ARG_UNUSED(dev);
	/* Callback is not supported by AON WDT */
	if (config->callback != NULL) {
		LOG_ERR("callback not supported by AON WDT");
		return -ENOTSUP;
	}

	if (config->window.min != 0U || config->window.max == 0U) {
		return -EINVAL;
	}

	if (config->flags != WDT_FLAG_RESET_SOC) {
		return -ENOTSUP;
	}

	reset_level = config->flags;
	AON_WDG_ConfigResetLevel(reset_level);

	comp = config->window.max;
	AON_WDG_ConfigComp(comp);

	AON_WDG_Enable();

	return 0;
}

static int aon_wdt_rtl8752h_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	AON_WDG_Restart();
	return 0;
}

static const struct wdt_driver_api aon_wdt_rtl8752h_api = {
	.setup = aon_wdt_rtl8752h_setup,
	.disable = aon_wdt_rtl8752h_disable,
	.install_timeout = aon_wdt_rtl8752h_install_timeout,
	.feed = aon_wdt_rtl8752h_feed,
};

static int aon_wdt_rtl8752h_init(const struct device *dev)
{
	int ret = 0;

#if !defined(CONFIG_WDT_DISABLE_AT_BOOT)
	/* const struct wdt_timeout_cfg config =
	 * {
	 *     .window.max = AON_WDT_INITIAL_TIMEOUT
	 * };

	 * ret = aon_wdt_rtl8752h_install_timeout(dev, &config);
	 */
#endif

	return ret;
}

DEVICE_DT_INST_DEFINE(0, aon_wdt_rtl8752h_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &aon_wdt_rtl8752h_api);
