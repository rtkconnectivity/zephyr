/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_kscan

/**
 * @brief Driver for KSCAN port on RTL87X2G family processor.
 * @note  Please validate for newly added series.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include "rtl_keyscan.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(kscan_rtl87x2g, CONFIG_KSCAN_LOG_LEVEL);

struct kscan_rtl87x2g_config
{
    uint32_t reg;
    uint16_t clkid;
    const struct pinctrl_dev_config *pcfg;
    uint8_t row_size;
    uint8_t col_size;
    uint16_t deb_ms;
    uint16_t scan_ms;
    uint16_t rel_ms;
    void (*irq_config_func)();
};

typedef struct
{
    uint16_t column: 5;
    uint16_t row: 4;
} kscan_key_index;

struct kscan_rtl87x2g_data
{
    kscan_key_index keys[26];
    uint32_t key_map[CONFIG_RTL87X2G_KEYSCAN_MAX_ROW_SIZE];
    uint8_t press_num;
    kscan_callback_t callback;
    bool cb_en;
};

static int kscan_rtl87x2g_configure(const struct device *dev,
                                    kscan_callback_t callback)
{
    struct kscan_rtl87x2g_data *data = dev->data;

    if (!callback)
    {
        return -EINVAL;
    }

    data->callback = callback;
    LOG_DBG("dev %s: configure cb %p", dev->name, callback);

    return 0;
}

static int kscan_rtl87x2g_disable_callback(const struct device *dev)
{
    struct kscan_rtl87x2g_data *data = dev->data;

    LOG_DBG("dev %s: disable cb", dev->name);
    data->cb_en = false;
    return 0;
}

static int kscan_rtl87x2g_enable_callback(const struct device *dev)
{
    struct kscan_rtl87x2g_data *data = dev->data;

    LOG_DBG("dev %s: enable cb", dev->name);
    data->cb_en = true;

    return 0;
}


static void kscan_rtl87x2g_isr(const struct device *dev)
{
    const struct kscan_rtl87x2g_config *config = dev->config;
    struct kscan_rtl87x2g_data *data = dev->data;
    KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;
    kscan_key_index new_keys[26];
    uint32_t new_key_map[CONFIG_RTL87X2G_KEYSCAN_MAX_ROW_SIZE];
    uint8_t new_press_num = 0;
    kscan_callback_t callback;

    if (KeyScan_GetFlagState(keyscan, KEYSCAN_INT_FLAG_SCAN_END) == SET)
    {
        KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, ENABLE);

        if (KeyScan_GetFlagState(keyscan, KEYSCAN_FLAG_EMPTY) != SET)
        {
            new_press_num = (uint32_t)KeyScan_GetFifoDataNum(keyscan);
            KeyScan_Read(keyscan, (uint16_t *)&new_keys, new_press_num);
        }

        callback = data->callback;
        memset(new_key_map, 0, sizeof(new_key_map));

        for (uint8_t i = 0; i < new_press_num; i++)
        {
            uint8_t new_row = new_keys[i].row;
            uint8_t new_col = new_keys[i].column;
            /* update new_key_map, set bit if releated key pressed */

            new_key_map[new_row] |= BIT(new_col);

            /* do nothing if the pressed key has been detected pressed during last scan */
            if (data->key_map[new_row] & BIT(new_col)) { continue; }

            /* update key_map, set bit if the key is detected pressed first time */

            data->key_map[new_row] |= BIT(new_col);

            if (callback && data->cb_en)
            {
                callback(dev, new_row, new_col, true);
            }
        }

        for (uint8_t i = 0; i < data->press_num; i++)
        {
            uint8_t old_row = data->keys[i].row;
            uint8_t old_col = data->keys[i].column;

            /* do nothing if key detected pressed during last scan still pressed */

            if (new_key_map[old_row] & BIT(old_col)) { continue; }

            /* update key_map, clear bit if the key is detected released first time */

            data->key_map[old_row] &= ~BIT(old_col);
            if (callback && data->cb_en)
            {
                callback(dev, old_row, old_col, false);
            }
        }

        memcpy(data->keys, new_keys, sizeof(new_keys));

        data->press_num = new_press_num;

        KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_SCAN_END);
        KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, DISABLE);
    }
}


static const struct kscan_driver_api kscan_rtl87x2g_driver_api =
{
    .config = kscan_rtl87x2g_configure,
    .disable_callback = kscan_rtl87x2g_disable_callback,
    .enable_callback = kscan_rtl87x2g_enable_callback,
};

static int kscan_rtl87x2g_init(const struct device *dev)
{
    LOG_DBG("dev %s init\n", dev->name);
    const struct kscan_rtl87x2g_config *config = dev->config;
    struct kscan_rtl87x2g_data *data = dev->data;
    KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;
    int err;

    memset(&data->key_map, 0, sizeof(data->key_map));
    memset(&data->keys, 0, sizeof(data->keys));

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&config->clkid);

    err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (err < 0)
    {
        return err;
    }

    KEYSCAN_InitTypeDef kscan_init_struct;
    KeyScan_StructInit(&kscan_init_struct);

    kscan_init_struct.clockdiv = 49;
    kscan_init_struct.delayclk = 49;

    kscan_init_struct.rowSize = config->row_size;
    kscan_init_struct.colSize = config->col_size;
    kscan_init_struct.debounceEn = config->deb_ms ? ENABLE : DISABLE;
    kscan_init_struct.scantimerEn = config->scan_ms ? ENABLE : DISABLE;
    kscan_init_struct.detecttimerEn = config->rel_ms ? ENABLE : DISABLE;

    kscan_init_struct.debouncecnt = config->deb_ms * 2;
    kscan_init_struct.scanInterval = config->scan_ms * 2;
    kscan_init_struct.releasecnt = config->rel_ms * 2;

    kscan_init_struct.scanmode = KeyScan_Auto_Scan_Mode;
    kscan_init_struct.keylimit = 26;

    KeyScan_Init(keyscan, &kscan_init_struct);

    KeyScan_INTConfig(keyscan, KEYSCAN_INT_SCAN_END, ENABLE);

    KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_SCAN_END);
    KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, DISABLE);
    KeyScan_Cmd(keyscan, ENABLE);
    config->irq_config_func();

    return 0;
}

#define RTL87X2G_KSCAN_IRQ_HANDLER_DECL(index)               \
    static void kscan_rtl87x2g_irq_config_func_##index();
#define RTL87X2G_KSCAN_IRQ_HANDLER(index)                    \
    static void kscan_rtl87x2g_irq_config_func_##index() \
    {                                   \
        IRQ_CONNECT(DT_INST_IRQN(index),                \
                    DT_INST_IRQ(index, priority),               \
                    kscan_rtl87x2g_isr, DEVICE_DT_INST_GET(index),       \
                    0);                         \
        irq_enable(DT_INST_IRQN(index));                \
    }

#define RTL87X2G_KSCAN_IRQ_HANDLER_FUNC(index)               \
    .irq_config_func = kscan_rtl87x2g_irq_config_func_##index,

#define RTL87X2G_KSCAN_INIT(index)                                          \
    RTL87X2G_KSCAN_IRQ_HANDLER_DECL(index)                                   \
    \
    PINCTRL_DT_INST_DEFINE(index);                                            \
    \
    static const struct kscan_rtl87x2g_config kscan_rtl87x2g_cfg_##index = {      \
        .reg = DT_INST_REG_ADDR(index),                        \
               .clkid = DT_INST_CLOCKS_CELL(index, id),            \
                        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                     \
                                .row_size = DT_INST_PROP(index, row_size),                 \
                                            .col_size = DT_INST_PROP(index, col_size),                 \
                                                        .deb_ms = DT_INST_PROP_OR(index, debounce_time_ms, 0),                 \
                                                                  .scan_ms = DT_INST_PROP_OR(index, scan_time_ms, 0),                 \
                                                                             .rel_ms = DT_INST_PROP_OR(index, release_time_ms, 0),                 \
                                                                                       RTL87X2G_KSCAN_IRQ_HANDLER_FUNC(index)                                       \
    };                                                                               \
    \
    static struct kscan_rtl87x2g_data kscan_rtl87x2g_data_##index = {                \
        .callback = NULL,             \
    };        \
    DEVICE_DT_INST_DEFINE(index,                                              \
                          &kscan_rtl87x2g_init,                                  \
                          NULL,                           \
                          &kscan_rtl87x2g_data_##index, &kscan_rtl87x2g_cfg_##index,    \
                          POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,                    \
                          &kscan_rtl87x2g_driver_api);                                  \
    \
    RTL87X2G_KSCAN_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(RTL87X2G_KSCAN_INIT)