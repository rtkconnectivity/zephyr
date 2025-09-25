/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_core_wdt

#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>

#include <rtl876x_wdg.h>

LOG_MODULE_REGISTER(wdt_core_rtl8752h, CONFIG_WDT_LOG_LEVEL);

#define CORE_WDT_INITIAL_TIMEOUT DT_INST_PROP(0, initial_timeout_ms)
#define CORE_WDT_DIV_FACTOR      DT_INST_PROP(0, div_factor)

#define CLK_FREQ      32768 /* 32.768k */
#define CNT_LIMIT_MAX 0xFFF

static uint32_t wdg_freq;
static uint32_t window_max_limit;

/*
 * cnt_limit: 2^(cnt_limit+1) - 1 ; max 11~15 = 0xFFF
 *          0: 0x001
 *          1: 0x003
 *          2: 0x007
 *          3: 0x00F
 *          4: 0x01F
 *          5: 0x03F
 *          6: 0x07F
 *          7: 0x0FF
 *          8: 0x1FF
 *          9: 0x3FF
 *          10: 0x7FF
 *          11~15: 0xFFF
 */
static uint8_t get_wdt_cnt_limit(uint32_t wdt_window)
{
	uint8_t result;
	uint32_t cnt = (uint32_t)(wdt_window * wdg_freq / 1000);
	uint32_t wdt_window_approximated = 0;

	/* Approximate */
	if (cnt == 0) {
		result = 0;
	} else if (cnt <= (1 << 1)) {
		result = 0;
	} else if (cnt <= (1 << 2)) {
		result = 1;
	} else if (cnt <= (1 << 3)) {
		result = 2;
	} else if (cnt <= (1 << 4)) {
		result = 3;
	} else if (cnt <= (1 << 5)) {
		result = 4;
	} else if (cnt <= (1 << 6)) {
		result = 5;
	} else if (cnt <= (1 << 7)) {
		result = 6;
	} else if (cnt <= (1 << 8)) {
		result = 7;
	} else if (cnt <= (1 << 9)) {
		result = 8;
	} else if (cnt <= (1 << 10)) {
		result = 9;
	} else if (cnt <= (1 << 11)) {
		result = 10;
	} else {
		/* CNT_LIMIT_MAX */
		result = 11;
	}

	if (result == 11) {
		wdt_window_approximated = (uint32_t)(CNT_LIMIT_MAX * 1000 / wdg_freq);
	} else {
		wdt_window_approximated = (uint32_t)((1 << result) * 1000 / wdg_freq);
	}

	LOG_INF("window.max=%d, cnt=%d, window.max(approximated)=%dms", wdt_window, result,
		wdt_window_approximated);
	return result;
}

static int core_wdt_rtl8752h_setup(const struct device *dev, uint8_t options)
{
	ARG_UNUSED(dev);

	/* Due the configuration will be lost in lowpower mode,
	 * the core wdt is not supported when CONFIG_PM=y
	 *
	 * if (!(options & WDT_OPT_PAUSE_IN_SLEEP)) {}
	 */

	if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG)) {
		/* Not support WDT_OPT_PAUSE_HALTED_BY_DBG */
		return -ENOTSUP;
	}

	return 0;
}

static int core_wdt_rtl8752h_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	WDG_Disable();

	return 0;
}

static int core_wdt_rtl8752h_install_timeout(const struct device *dev,
					     const struct wdt_timeout_cfg *config)
{
	ARG_UNUSED(dev);

	LOG_DBG("wdg_freq=%d, window_max_limit=%dms", wdg_freq, window_max_limit);

	/* Callback is not supported by CORE WDT */
	if (config->callback != NULL) {
		LOG_ERR("callback not supported by CORE WDT");
		return -ENOTSUP;
	}

	if (config->window.max == 0U) {
		WDG_SystemReset(RESET_ALL, SW_RESET_APP_START);
	}

	if (config->window.min != 0U) {
		return -EINVAL;
	}

	uint8_t cnt_limit = get_wdt_cnt_limit(config->window.max);

	uint8_t wdg_mode = 0;

	switch (config->flags) {
	case WDT_FLAG_RESET_CPU_CORE:
		wdg_mode = 2;
		break;
	case WDT_FLAG_RESET_SOC:
		wdg_mode = 3;
		break;
	case WDT_FLAG_RESET_NONE:
		wdg_mode = 0;
		break;
	default:
		LOG_ERR("Unsupported watchdog config flag");
		return -EINVAL;
	}
	WDG_Config(CORE_WDT_DIV_FACTOR, cnt_limit, wdg_mode);
	WDG_Enable();

	return 0;
}

static int core_wdt_rtl8752h_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	if (channel_id != 0) {
		LOG_ERR("Unsupported channel_id");
		return -ENOTSUP;
	}

	WDG_Restart();

	return 0;
}

static const struct wdt_driver_api core_wdt_rtl8752h_api = {
	.setup = core_wdt_rtl8752h_setup,
	.disable = core_wdt_rtl8752h_disable,
	.install_timeout = core_wdt_rtl8752h_install_timeout,
	.feed = core_wdt_rtl8752h_feed,
};

static int core_wdt_rtl8752h_init(const struct device *dev)
{
	int ret = 0;

	WDG_ClockEnable();
	wdg_freq = (uint32_t)(CLK_FREQ / (CORE_WDT_DIV_FACTOR + 1));
	window_max_limit = CNT_LIMIT_MAX * 1000 / wdg_freq;

#if !defined(CONFIG_WDT_DISABLE_AT_BOOT)
	const struct wdt_timeout_cfg config = {.window.max = CORE_WDT_INITIAL_TIMEOUT};

	ret = core_wdt_rtl8752h_install_timeout(dev, &config);
#endif

	return ret;
}

DEVICE_DT_INST_DEFINE(0, core_wdt_rtl8752h_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &core_wdt_rtl8752h_api);
