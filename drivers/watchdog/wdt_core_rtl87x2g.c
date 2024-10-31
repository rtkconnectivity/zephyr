/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT realtek_rtl87x2g_core_wdt

#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>

#include <rtl_wdt.h>

extern void WDG_SystemReset(WDTMode_TypeDef wdt_mode,
                            int reset_reason);//can this function declare in rtl_wdt.h?

LOG_MODULE_REGISTER(wdt_core_rtl87x2g, CONFIG_WDT_LOG_LEVEL);

#define CORE_WDT_INITIAL_TIMEOUT DT_INST_PROP(0, initial_timeout_ms)

static int core_wdt_rtl87x2g_setup(const struct device *dev, uint8_t options)
{
    ARG_UNUSED(dev);

    return 0;
}

static int core_wdt_rtl87x2g_disable(const struct device *dev)
{
    ARG_UNUSED(dev);

    WDT_Disable();

    return 0;
}

static int core_wdt_rtl87x2g_install_timeout(const struct device *dev,
                                             const struct wdt_timeout_cfg *config)
{
    /* Callback is not supported by CORE WDT */
    if (config->callback != NULL)
    {
        LOG_ERR("callback not supported by CORE WDT");
        return -ENOTSUP;
    }

    if (config->window.max == 0U)
    {
        WDG_SystemReset(RESET_ALL, config->flags);
    }

    if (config->window.min != 0U)
    {
        return -EINVAL;
    }

    bool ret = WDT_Start(config->window.max, RESET_ALL);

    if (ret != true)
    {
        LOG_ERR("WDT_Start() failed");
        return -EINVAL;
    }

    return 0;
}

static int core_wdt_rtl87x2g_feed(const struct device *dev, int channel_id)
{
    ARG_UNUSED(channel_id);

    WDT_Kick();

    return 0;
}

static const struct wdt_driver_api core_wdt_rtl87x2g_api =
{
    .setup = core_wdt_rtl87x2g_setup,
    .disable = core_wdt_rtl87x2g_disable,
    .install_timeout = core_wdt_rtl87x2g_install_timeout,
    .feed = core_wdt_rtl87x2g_feed,
};

static int core_wdt_rtl87x2g_init(const struct device *dev)
{
    int ret = 0;

#if !defined(CONFIG_WDT_DISABLE_AT_BOOT)
    const struct wdt_timeout_cfg config =
    {
        .window.max = CORE_WDT_INITIAL_TIMEOUT
    };

    ret = core_wdt_rtl87x2g_install_timeout(dev, &config);
#endif

    return ret;
}

DEVICE_DT_INST_DEFINE(0, core_wdt_rtl87x2g_init, NULL, NULL, NULL, POST_KERNEL,
                      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &core_wdt_rtl87x2g_api);