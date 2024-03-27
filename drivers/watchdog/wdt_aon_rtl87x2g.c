/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT realtek_rtl87x2g_aon_wdt

#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>

#include <rtl_aon_wdt.h>

LOG_MODULE_REGISTER(wdt_aon_rtl87x2g, CONFIG_WDT_LOG_LEVEL);

#define AON_WDT_INITIAL_TIMEOUT DT_INST_PROP(0, initial_timeout_ms)

static int aon_wdt_rtl87x2g_setup(const struct device *dev, uint8_t options)
{
    ARG_UNUSED(dev);

    //aon_wdg_enable();

    return 0;
}

static int aon_wdt_rtl87x2g_disable(const struct device *dev)
{
    ARG_UNUSED(dev);

    AON_WDT_Disable(AON_WDT);

    return 0;
}

static int aon_wdt_rtl87x2g_install_timeout(const struct device *dev,
                                            const struct wdt_timeout_cfg *config)
{
    /* Callback is not supported by AON WDT */
    if (config->callback != NULL)
    {
        LOG_ERR("callback not supported by AON WDT");
        return -ENOTSUP;
    }

    if (config->window.min != 0U || config->window.max == 0U)
    {
        return -EINVAL;
    }

    bool ret = AON_WDT_Start(AON_WDT, config->window.max, RESET_ALL);

    if (ret != true)
    {
        LOG_ERR("AON_WDT_Start() failed");
        return -EINVAL;
    }

    return 0;
}

static int aon_wdt_rtl87x2g_feed(const struct device *dev, int channel_id)
{
    ARG_UNUSED(channel_id);

    AON_WDT_Kick(AON_WDT);

    return 0;
}

static const struct wdt_driver_api aon_wdt_rtl87x2g_api =
{
    .setup = aon_wdt_rtl87x2g_setup,
    .disable = aon_wdt_rtl87x2g_disable,
    .install_timeout = aon_wdt_rtl87x2g_install_timeout,
    .feed = aon_wdt_rtl87x2g_feed,
};

static int aon_wdt_rtl87x2g_init(const struct device *dev)
{
    int ret = 0;

#if !defined(CONFIG_WDT_DISABLE_AT_BOOT)
    // const struct wdt_timeout_cfg config =
    // {
    //     .window.max = AON_WDT_INITIAL_TIMEOUT
    // };

    // ret = aon_wdt_rtl87x2g_install_timeout(dev, &config);
#endif

    return ret;
}

DEVICE_DT_INST_DEFINE(0, aon_wdt_rtl87x2g_init, NULL, NULL, NULL, POST_KERNEL,
                      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &aon_wdt_rtl87x2g_api);