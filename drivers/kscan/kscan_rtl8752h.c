/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_kscan

/**
 * @brief Driver for KSCAN port on RTL8752H family processor.
 * @note  Please validate for newly added series.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/clock_control/rtl8752h_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#ifdef CONFIG_PM_DEVICE
#include "power_manager_unit_platform.h"
#include "dlps.h"
#endif
#include "rtl876x_keyscan.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_nvic.h"
#include "rtl876x_rcc.h"
#include "vector_table.h"

#include "trace.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(kscan_rtl8752h, CONFIG_KSCAN_LOG_LEVEL);

struct kscan_rtl8752h_config {
	uint32_t reg;
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
	uint8_t row_size;
	uint8_t col_size;
	uint16_t deb_us;
	uint16_t scan_us;
	uint16_t rel_us;
	uint8_t scan_debounce_cnt;
	void (*irq_config_func)();
};

typedef struct {
	uint16_t column: 5;
	uint16_t row: 4;
} kscan_key_index;

struct kscan_rtl8752h_data {
	kscan_key_index keys[26];
	uint32_t key_map[CONFIG_RTL8752H_KEYSCAN_MAX_ROW_SIZE];
	uint8_t press_num;
	kscan_callback_t callback;
	bool cb_en;
#ifdef CONFIG_PM_DEVICE
	KEYSCANStoreReg_Typedef store_buf;
#endif
};

#ifdef CONFIG_PM_DEVICE
static PMCheckResult kscan_pm_check_state = PM_CHECK_PASS;
#endif

static int kscan_rtl8752h_configure(const struct device *dev, kscan_callback_t callback)
{
	struct kscan_rtl8752h_data *data = dev->data;

	if (!callback) {
		return -EINVAL;
	}

	data->callback = callback;
	LOG_DBG("dev %s: configure cb %p", dev->name, callback);

	return 0;
}

static int kscan_rtl8752h_disable_callback(const struct device *dev)
{
	struct kscan_rtl8752h_data *data = dev->data;

	LOG_DBG("dev %s: disable cb", dev->name);
	data->cb_en = false;
	return 0;
}

static int kscan_rtl8752h_enable_callback(const struct device *dev)
{
	struct kscan_rtl8752h_data *data = dev->data;

	LOG_DBG("dev %s: enable cb", dev->name);
	data->cb_en = true;

	return 0;
}

static bool kscan_rtl8752h_ghost_key_filter(kscan_key_index *new_keys, uint8_t new_press_num)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	const struct kscan_rtl8752h_config *config = dev->config;
	KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;

	/* filter ghost key */
	if (new_press_num >= 4) {
		for (uint8_t i = 0; i < new_press_num - 2; i++) {
			/* two keys in the same column */
			if (new_keys[i].column == new_keys[i + 1].column) {
				for (uint8_t j = i + 2; j < new_press_num; j++) {
					/* another key in the same row,
					 * which is ghost key
					 */
					if (new_keys[i].row == new_keys[j].row) {
						LOG_ERR("ghost key "
							"detected!\n");
						KeyScan_ClearINTPendingBit(keyscan,
									   KEYSCAN_INT_SCAN_END);
						KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END,
								DISABLE);
						return true;
					}
				}
			}
		}
	}

	return false;
}

static void kscan_rtl8752h_isr(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	const struct kscan_rtl8752h_config *config = dev->config;
	struct kscan_rtl8752h_data *data = dev->data;
	KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;
	uint32_t scan_debounce_cnt = config->scan_debounce_cnt;
	kscan_callback_t callback = data->callback;
	static uint8_t press_cnt;
	static kscan_key_index last_keys[26];
	static bool all_release_flag = true;

	kscan_key_index new_keys[26];
	uint32_t new_key_map[CONFIG_RTL8752H_KEYSCAN_MAX_ROW_SIZE];
	uint8_t new_press_num = KeyScan_GetFifoDataNum(keyscan);

	if (KeyScan_GetFlagState(keyscan, KEYSCAN_INT_FLAG_SCAN_END) == SET) {
		KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, ENABLE);

		if (KeyScan_GetFlagState(keyscan, KEYSCAN_FLAG_EMPTY) != SET) {
			memset(new_keys, 0, sizeof(new_keys));
			KeyScan_Read(keyscan, (uint16_t *)&new_keys, new_press_num);
		}

		if (all_release_flag) {
			press_cnt = 0;

			memcpy(last_keys, new_keys, sizeof(new_keys));
			all_release_flag = false;
		}

		/* compare scan result between new_keys and last_keys */

		if (!memcmp(last_keys, new_keys, sizeof(new_keys))) {
			/* new_keys is same as last_keys */

			if (press_cnt >= scan_debounce_cnt) {
#if CONFIG_RTL8752H_KEYSCAN_GHOST_KEY_FILTER
				if (kscan_rtl8752h_ghost_key_filter(new_keys, new_press_num)) {
					return;
				}
#endif
				memset(new_key_map, 0, sizeof(new_key_map));

				/* update press keys */

				for (uint8_t i = 0; i < new_press_num; i++) {
					uint8_t new_row = new_keys[i].row;
					uint8_t new_col = new_keys[i].column;

					/* update new_key_map, set bit if related key pressed */

					new_key_map[new_row] |= BIT(new_col);

					/* do nothing if the pressed key has been detected pressed
					 * during last scan
					 */

					if (data->key_map[new_row] & BIT(new_col)) {
						continue;
					}

					/* update key_map, set bit if the key is detected pressed
					 * first time
					 */

					data->key_map[new_row] |= BIT(new_col);

					if (callback && data->cb_en) {
#ifdef CONFIG_PM_DEVICE
						kscan_pm_check_state = PM_CHECK_FAIL;
#endif
						callback(dev, new_row, new_col, true);
					}
				}

				/* update release keys */

				for (uint8_t i = 0; i < data->press_num; i++) {
					uint8_t old_row = data->keys[i].row;
					uint8_t old_col = data->keys[i].column;

					/* do nothing if key detected pressed during last scan still
					 * pressed
					 */

					if (new_key_map[old_row] & BIT(old_col)) {
						continue;
					}

					/* update key_map, clear bit if the key is detected released
					 * first time
					 */

					data->key_map[old_row] &= ~BIT(old_col);

					if (callback && data->cb_en) {
						callback(dev, old_row, old_col, false);
					}
				}

				memcpy(data->keys, new_keys, sizeof(new_keys));
				data->press_num = new_press_num;
				press_cnt = 0;
			}

			press_cnt++;

		} else {
			/* new_keys is different from last_keys */

			press_cnt = 0;
			memcpy(last_keys, new_keys, sizeof(new_keys));
		}

		KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_SCAN_END);
		KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, DISABLE);
	}

	if (KeyScan_GetFlagState(keyscan, KEYSCAN_INT_FLAG_ALL_RELEASE) == SET) {
#ifdef CONFIG_PM_DEVICE
		kscan_pm_check_state = PM_CHECK_PASS;
#endif
		press_cnt = 0;

		for (uint8_t i = 0; i < data->press_num; i++) {
			uint8_t old_row = data->keys[i].row;
			uint8_t old_col = data->keys[i].column;

			if (callback && data->cb_en) {
				callback(dev, old_row, old_col, false);
			}
		}

		data->press_num = 0;
		all_release_flag = true;

		memset(data->keys, 0, sizeof(data->keys));
		memset(data->key_map, 0, sizeof(data->key_map));

		KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_ALL_RELEASE);
	}
}

#ifdef CONFIG_PM_DEVICE
static int kscan_rtl8752h_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct kscan_rtl8752h_config *config = dev->config;
	struct kscan_rtl8752h_data *data = dev->data;
	KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;
	int err;

	extern void KEYSCAN_DLPSEnter(void *PeriReg, void *StoreBuf);
	extern void KEYSCAN_DLPSExit(void *PeriReg, void *StoreBuf);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:

		KEYSCAN_DLPSEnter(keyscan, &data->store_buf);

		/* Move pins to sleep state */
		err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
		if ((err < 0) && (err != -ENOENT)) {
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Set pins to active state */
		err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (err < 0) {
			return err;
		}

		KEYSCAN_DLPSExit(keyscan, &data->store_buf);

		/* check wakeup pin status */
		int ret;
		const struct pinctrl_state *state;

		ret = pinctrl_lookup_state(config->pcfg, PINCTRL_STATE_SLEEP, &state);
		if ((err < 0) && (err != -ENOENT)) {
			/* no kscan wakeup pin is configured */
			return ret;
		}
		/* there are kscan wakeup pins configured, check if they wakeup the system
		 */
		for (uint8_t i = 0U; i < state->pin_cnt; i++) {
			if (state->pins[i].wakeup_low || state->pins[i].wakeup_high) {
				if (System_WakeUpInterruptValue(state->pins[i].pin) == SET) {
					System_WakeUpPinDisable(state->pins[i].pin);
					Pad_ClearWakeupINTPendingBit(state->pins[i].pin);
					kscan_pm_check_state = PM_CHECK_FAIL;
				}
			}
		}

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static PMCheckResult kscan_pm_check(void)
{
	return kscan_pm_check_state;
}

static void kscan_register_dlps_cb(void)
{
	dlps_check_cb_reg(kscan_pm_check);
}

#endif /* CONFIG_PM_DEVICE */

static const struct kscan_driver_api kscan_rtl8752h_driver_api = {
	.config = kscan_rtl8752h_configure,
	.disable_callback = kscan_rtl8752h_disable_callback,
	.enable_callback = kscan_rtl8752h_enable_callback,
};

static int kscan_rtl8752h_init(const struct device *dev)
{
	LOG_DBG("dev %s init\n", dev->name);
	const struct kscan_rtl8752h_config *config = dev->config;
	struct kscan_rtl8752h_data *data = dev->data;
	KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;
	int err;

	memset(data->key_map, 0, sizeof(data->key_map));
	memset(data->keys, 0, sizeof(data->keys));

	(void)clock_control_on(RTL8752H_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	KEYSCAN_InitTypeDef kscan_init_struct;

	KeyScan_StructInit(&kscan_init_struct);

	kscan_init_struct.rowSize = config->row_size;
	kscan_init_struct.colSize = config->col_size;

	/* default scan clk is 2.5 MHz */
	kscan_init_struct.clockdiv = 1;

	/* default delay clk is 50 kHz */
	kscan_init_struct.delayclk = 49;

	kscan_init_struct.debouncecnt = (config->deb_us + 10) / 20;
	kscan_init_struct.scanInterval = (config->scan_us + 10) / 20;
	kscan_init_struct.releasecnt = (config->rel_us + 10) / 20;

	kscan_init_struct.debounceEn =
		kscan_init_struct.debouncecnt ? KeyScan_Debounce_Enable : KeyScan_Debounce_Disable;
	kscan_init_struct.scantimerEn = kscan_init_struct.scanInterval
						? KeyScan_ScanInterval_Enable
						: KeyScan_ScanInterval_Disable;
	kscan_init_struct.detecttimerEn = kscan_init_struct.releasecnt
						  ? KeyScan_Release_Detect_Enable
						  : KeyScan_Release_Detect_Disable;

	kscan_init_struct.scanmode = KeyScan_Auto_Scan_Mode;
	kscan_init_struct.keylimit = 26;
	kscan_init_struct.manual_sel = KeyScan_Manual_Sel_Key;

	KeyScan_Init(keyscan, &kscan_init_struct);

	/* set pre guard time */
	keyscan->CLKDIV = (keyscan->CLKDIV & ~(0x7 << 26)) | (6 << 26);

	KeyScan_INTConfig(keyscan, KEYSCAN_INT_SCAN_END, ENABLE);
	KeyScan_INTConfig(keyscan, KEYSCAN_INT_ALL_RELEASE, ENABLE);

	KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_SCAN_END);
	KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_ALL_RELEASE);

	KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, DISABLE);
	KeyScan_INTMask(keyscan, KEYSCAN_INT_ALL_RELEASE, DISABLE);
	KeyScan_Cmd(keyscan, ENABLE);
	config->irq_config_func();

#ifdef CONFIG_PM_DEVICE
	kscan_register_dlps_cb();
#endif
	return 0;
}

#define RTL8752H_KSCAN_IRQ_HANDLER_DECL(index)                                                     \
	static void kscan_rtl8752h_irq_config_func_##index(void);
#define RTL8752H_KSCAN_IRQ_HANDLER(index)                                                          \
	static void kscan_rtl8752h_irq_config_func_##index(void)                                   \
	{                                                                                          \
		RamVectorTableUpdate(KeyScan_IRQn, kscan_rtl8752h_isr);                            \
		NVIC_InitTypeDef NVIC_InitStruct;                                                  \
		NVIC_InitStruct.NVIC_IRQChannel = KeyScan_IRQn;                                    \
		NVIC_InitStruct.NVIC_IRQChannelPriority = 2;                                       \
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;                                       \
		NVIC_Init(&NVIC_InitStruct);                                                       \
	}

#define RTL8752H_KSCAN_IRQ_HANDLER_FUNC(index)                                                     \
	.irq_config_func = kscan_rtl8752h_irq_config_func_##index,

#define RTL8752H_KSCAN_INIT(index)                                                                 \
	RTL8752H_KSCAN_IRQ_HANDLER_DECL(index)                                                     \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct kscan_rtl8752h_config kscan_rtl8752h_cfg_##index = {                   \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.row_size = DT_INST_PROP(index, row_size),                                         \
		.col_size = DT_INST_PROP(index, col_size),                                         \
		.deb_us = DT_INST_PROP_OR(index, debounce_time_us, 0),                             \
		.scan_us = DT_INST_PROP_OR(index, scan_time_us, 0),                                \
		.rel_us = DT_INST_PROP_OR(index, release_time_us, 0),                              \
		.scan_debounce_cnt = DT_INST_PROP(index, scan_debounce_cnt),                       \
		RTL8752H_KSCAN_IRQ_HANDLER_FUNC(index)};                                           \
                                                                                                   \
	static struct kscan_rtl8752h_data kscan_rtl8752h_data_##index = {                          \
		.callback = NULL,                                                                  \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(index, kscan_rtl8752h_pm_action);                                 \
	DEVICE_DT_INST_DEFINE(index, &kscan_rtl8752h_init, PM_DEVICE_DT_INST_GET(index),           \
			      &kscan_rtl8752h_data_##index, &kscan_rtl8752h_cfg_##index,           \
			      APPLICATION, CONFIG_KSCAN_INIT_PRIORITY,                             \
			      &kscan_rtl8752h_driver_api);                                         \
                                                                                                   \
	RTL8752H_KSCAN_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(RTL8752H_KSCAN_INIT)
