/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>

#include <rtl_pinmux.h>

#include <trace.h>
#define DBG_DIRECT_SHOW 0

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
    uint32_t cfg_fun = pin[0].fun;
    uint32_t cfg_pin = pin[0].pin;
    uint32_t cfg_dir = pin[0].dir;
    uint32_t cfg_drv = pin[0].drive;
    uint32_t cfg_pull = pin[0].pull;
    uint32_t cfg_pull_strength = pin[0].pull_strength;

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[pinctrl_configure_pin] cfg_fun=%d, cfg_pin=%d,"
               "cfg_dir=%d, cfg_drv=%d , cfg_pull=%d, cfg_pull_strength=%d", \
               cfg_fun, cfg_pin, cfg_dir, cfg_drv, cfg_pull, cfg_pull_strength);
#endif

    Pad_SetPullStrength(cfg_pin, cfg_pull_strength);

    if (cfg_fun > RTL87X2G_SW_MODE)
    {
        Pad_Config(cfg_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
        Pinmux_AON_Config(cfg_fun);
    }
    else if (cfg_fun == RTL87X2G_SW_MODE)
    {
        Pad_Config(cfg_pin, PAD_SW_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
    }
    else
    {
        Pad_Config(cfg_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
        Pinmux_Config(cfg_pin, cfg_fun);
    }

}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
                           uintptr_t reg)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[pinctrl_configure_pins] pin_cnt=%d", pin_cnt);
#endif
    for (uint8_t i = 0U; i < pin_cnt; i++)
    {
        pinctrl_configure_pin(&pins[i]);
    }

    return 0;
}
