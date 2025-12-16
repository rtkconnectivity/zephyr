/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_kscan

/**
 * @brief Driver for KSCAN port on BEE family processor.
 * @note  Please validate for newly added series.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/input/input.h>

#ifdef CONFIG_PM_DEVICE
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include "power_manager_unit_platform.h"
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include "dlps.h"

extern void (*platform_pm_register_callback_func_with_priority)(void *cb_func,
								PlatformPMStage pf_pm_stage,
								int8_t priority);

#endif

extern void KEYSCAN_DLPSEnter(void *PeriReg, void *StoreBuf);
extern void KEYSCAN_DLPSExit(void *PeriReg, void *StoreBuf, uint32_t scanmode, uint32_t manual_sel);

#endif

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include "rtl_keyscan.h"
#include "rtl_pinmux.h"
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include "rtl876x_keyscan.h"
#include "rtl876x_nvic.h"
#include "rtl876x_rcc.h"
#include "rtl876x_pinmux.h"
#include "vector_table.h"
#endif

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define BEE_Pad_SetControlMode(pad, mode)            Pad_SetControlMode(pad, mode)
#define BEE_Pad_SetPullMode(pad, pull)               Pad_SetPullMode(pad, pull)
#define BEE_System_WakeUpPinEnable(pin, pol, deb_en) System_WakeUpPinEnable(pin, pol, deb_en)
#define BEE_KSCAN_REG_CLKDIV                         KEYSCAN_CLK_DIV
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define BEE_Pad_SetControlMode(pad, mode)            Pad_ControlSelectValue(pad, mode)
#define BEE_Pad_SetPullMode(pad, pull)               Pad_PullUpOrDownValue(pad, pull)
#define BEE_System_WakeUpPinEnable(pin, pol, deb_en) System_WakeUpPinEnable(pin, pol, deb_en, 0)
#define BEE_KSCAN_REG_CLKDIV                         CLKDIV
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(kscan_bee, CONFIG_INPUT_LOG_LEVEL);

struct kscan_bee_config {
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

struct kscan_bee_data {
	/* To record all released status, if all key released, the next scan
	 * will start a software debounce
	 */
	bool all_release_flag;
	/* To record rows and cols of all pressed keys after software debounce */
	kscan_key_index keys[26];
	/* To record rows and cols of all pressed keys during last scan, then compared with all
	 * pressed keys during current scan
	 */
	kscan_key_index last_keys[26];
	/* To record rows and cols of all pressed keys after software debounce, bit[i] of
	 * new_key_map[j] setting to 1 means row[i] col[j] is pressed.
	 */
	uint32_t key_map[CONFIG_BEE_INPUT_KSCAN_MAX_ROW_SIZE];
	/* Total number of pressed pins before software debounce */
	uint8_t last_scanned_num;
	/* Total number of pressed pins after software debounce */
	uint8_t last_pressed_num;
	/* To count several scan for software debounce */
	uint8_t sw_deb_press_cnt;
	/* Row of pins to configure wakeup pins */
	uint16_t press_rows;
#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
	/* If timer started, do not configure auto scan during resume */
	bool timer_started;
#endif
#ifdef CONFIG_PM_DEVICE
	KEYSCANStoreReg_Typedef store_buf;
#endif
};

#ifdef CONFIG_PM_DEVICE
static PMCheckResult kscan_pm_check_state = PM_CHECK_PASS;
#endif

static int kscan_bee_init_driver(const struct device *dev, uint32_t scanmode, uint32_t manual_sel)
{
	LOG_DBG("dev %s init\n", dev->name);
	const struct kscan_bee_config *config = dev->config;
	KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;

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

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	kscan_init_struct.debounceEn = kscan_init_struct.debouncecnt ? ENABLE : DISABLE;
	kscan_init_struct.scantimerEn = kscan_init_struct.scanInterval ? ENABLE : DISABLE;
	kscan_init_struct.detecttimerEn = kscan_init_struct.releasecnt ? ENABLE : DISABLE;
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
	kscan_init_struct.debounceEn =
		kscan_init_struct.debouncecnt ? KeyScan_Debounce_Enable : KeyScan_Debounce_Disable;
	kscan_init_struct.scantimerEn = kscan_init_struct.scanInterval
						? KeyScan_ScanInterval_Enable
						: KeyScan_ScanInterval_Disable;
	kscan_init_struct.detecttimerEn = kscan_init_struct.releasecnt
						  ? KeyScan_Release_Detect_Enable
						  : KeyScan_Release_Detect_Disable;
#endif

	kscan_init_struct.manual_sel = manual_sel;
	kscan_init_struct.scanmode = scanmode;

	kscan_init_struct.keylimit = 26;

	KeyScan_Init(keyscan, &kscan_init_struct);

	/* set pre guard time */
	keyscan->BEE_KSCAN_REG_CLKDIV = (keyscan->BEE_KSCAN_REG_CLKDIV & ~(0x7 << 26)) | (6 << 26);

	KeyScan_INTConfig(keyscan, KEYSCAN_INT_SCAN_END, ENABLE);
	KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_SCAN_END);
	KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, DISABLE);

#if CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
	KeyScan_INTConfig(keyscan, KEYSCAN_INT_ALL_RELEASE, ENABLE);
	KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_ALL_RELEASE);
	KeyScan_INTMask(keyscan, KEYSCAN_INT_ALL_RELEASE, DISABLE);
#endif

	KeyScan_Cmd(keyscan, ENABLE);

#ifdef CONFIG_PM_DEVICE
	struct kscan_bee_data *data = dev->data;

	KEYSCAN_DLPSEnter(keyscan, &data->store_buf);
#endif

	return 0;
}

#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
static void manual_kscan_timer_cb(struct k_timer *timer);
static K_TIMER_DEFINE(manual_kscan_timer, manual_kscan_timer_cb, NULL);

static void manual_kscan_timer_cb(struct k_timer *timer)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(kscan));

#ifdef CONFIG_PM_DEVICE
	kscan_pm_check_state = PM_CHECK_FAIL;
#endif
	/* register trigger manual mode init */
	kscan_bee_init_driver(dev, KeyScan_Manual_Scan_Mode, KeyScan_Manual_Sel_Bit);
}
#endif

#if CONFIG_BEE_INPUT_KSCAN_GHOST_KEY_FILTER
static bool kscan_bee_ghost_key_filter(uint8_t new_press_num, kscan_key_index *new_keys)
{
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
						return true;
					}
				}
			}
		}
	}

	return false;
}
#endif

#if CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
static void kscan_bee_all_release_process(const struct device *dev)
{
	struct kscan_bee_data *data = dev->data;

#ifdef CONFIG_PM_DEVICE
	kscan_pm_check_state = PM_CHECK_PASS;
#endif
	data->sw_deb_press_cnt = 0;

	for (uint8_t i = 0; i < data->last_pressed_num; i++) {
		uint8_t old_row = data->keys[i].row;
		uint8_t old_col = data->keys[i].column;

		input_report_abs(dev, INPUT_ABS_X, old_col, false, K_FOREVER);
		input_report_abs(dev, INPUT_ABS_Y, old_row, false, K_FOREVER);
		input_report_key(dev, INPUT_BTN_TOUCH, false, true, K_FOREVER);
	}

	data->last_scanned_num = 0;
	data->last_pressed_num = 0;
	data->all_release_flag = true;

	memset(data->keys, 0, sizeof(data->keys));
	memset(data->key_map, 0, sizeof(data->key_map));
	memset(data->last_keys, 0, sizeof(data->last_keys));
}
#endif

static void kscan_bee_process(const struct device *dev, uint8_t new_press_num,
			      kscan_key_index *new_keys)
{
	const struct kscan_bee_config *config = dev->config;
	struct kscan_bee_data *data = dev->data;
	KEYSCAN_TypeDef *keyscan;

	keyscan = (KEYSCAN_TypeDef *)config->reg;

	/* Total debounce count for software debounce.
	 * If all key released, the next scan will start a software debounce,
	 * It is determined to be a valid key press only when the same key press is detected
	 * consecutively for scan_debounce_cnt times.
	 */
	uint32_t scan_debounce_cnt = config->scan_debounce_cnt;

	/* To record rows and cols of all pressed keys during current scan,
	 * bit[i] of new_key_map[j] setting to 1 means row[i] col[j] is pressed.
	 */
	uint32_t new_key_map[CONFIG_BEE_INPUT_KSCAN_MAX_ROW_SIZE];

#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
	KeyScan_Cmd(keyscan, DISABLE);
#endif

#ifdef CONFIG_PM_DEVICE
#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
	/* If use manual scan mode, enter pm during two scan */
	kscan_pm_check_state = PM_CHECK_PASS;
#else
	/* If use auto scan mode, enter pm only when all key released */
	kscan_pm_check_state = PM_CHECK_FAIL;
#endif
#endif

#if CONFIG_BEE_INPUT_KSCAN_GHOST_KEY_FILTER
	/* filter ghost keys */
	if (kscan_bee_ghost_key_filter(new_press_num, new_keys)) {
		goto start_timer;
	}
#endif

	/* software debounce */
	if (data->all_release_flag) {
		data->sw_deb_press_cnt = 0;

		data->all_release_flag = false;
	}

	if (memcmp(data->last_keys, new_keys, sizeof(data->last_keys)) == 0 &&
	    new_press_num == data->last_scanned_num) {
		/* new_keys is same as last_keys */
		if (data->sw_deb_press_cnt >= scan_debounce_cnt) {
			/* after sofeware debounce */
			memset(new_key_map, 0, sizeof(new_key_map));
		} else {
			/* debouncing */
			data->sw_deb_press_cnt++;
			goto start_timer;
		}
	} else {
		/* new_keys is different from last_keys, start a new software debounce */
		data->sw_deb_press_cnt = 0;
		memcpy(data->last_keys, new_keys, sizeof(data->last_keys));
		data->last_scanned_num = new_press_num;
#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
		data->press_rows = 0;
		for (uint8_t i = 0; i < new_press_num; i++) {
			data->press_rows |= BIT(new_keys[i].row);
		}
#endif
		goto start_timer;
	}

	/* after software debounce, process scan result */
	if (new_press_num == 0) {
		/* all key release */

		/* call all cbs for released keys */
		for (uint8_t i = 0; i < data->last_pressed_num; i++) {
			uint8_t old_row = data->keys[i].row;
			uint8_t old_col = data->keys[i].column;

			input_report_abs(dev, INPUT_ABS_X, old_col, false, K_FOREVER);
			input_report_abs(dev, INPUT_ABS_Y, old_row, false, K_FOREVER);
			input_report_key(dev, INPUT_BTN_TOUCH, false, true, K_FOREVER);
		}

		data->last_pressed_num = 0;
		data->press_rows = 0;
		data->all_release_flag = true;

		memset(data->keys, 0, sizeof(data->keys));
		memset(data->key_map, 0, sizeof(data->key_map));
		memset(data->last_keys, 0, sizeof(data->last_keys));

#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
		k_timer_stop(&manual_kscan_timer);
		data->timer_started = false;
		(void)clock_control_off(BEE_CLOCK_CONTROLLER,
					(clock_control_subsys_t)&config->clkid);
		(void)clock_control_on(BEE_CLOCK_CONTROLLER,
				       (clock_control_subsys_t)&config->clkid);
		kscan_bee_init_driver(dev, KeyScan_Manual_Scan_Mode, KeyScan_Manual_Sel_Key);
		return;
#endif
	} else {
		/* some key pressed */

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

			input_report_abs(dev, INPUT_ABS_X, new_col, false, K_FOREVER);
			input_report_abs(dev, INPUT_ABS_Y, new_row, false, K_FOREVER);
			input_report_key(dev, INPUT_BTN_TOUCH, true, true, K_FOREVER);
		}

		/* update release keys */
		for (uint8_t i = 0; i < data->last_pressed_num; i++) {
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

			input_report_abs(dev, INPUT_ABS_X, old_col, false, K_FOREVER);
			input_report_abs(dev, INPUT_ABS_Y, old_row, false, K_FOREVER);
			input_report_key(dev, INPUT_BTN_TOUCH, false, true, K_FOREVER);
		}

		data->last_pressed_num = new_press_num;
		memcpy(data->keys, new_keys, sizeof(data->keys));
	}

start_timer:
#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
	data->timer_started = true;
	k_timer_start(&manual_kscan_timer, K_USEC(config->scan_us), K_FOREVER);
#endif
}

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
static void kscan_bee_isr(const struct device *dev)
{
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
static void kscan_bee_isr(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
#endif

	const struct kscan_bee_config *config = dev->config;
	KEYSCAN_TypeDef *keyscan = (KEYSCAN_TypeDef *)config->reg;

	kscan_key_index new_keys[26];

	uint8_t new_press_num = KeyScan_GetFifoDataNum(keyscan);

	memset(new_keys, 0, sizeof(new_keys));

	if (KeyScan_GetFlagState(keyscan, KEYSCAN_INT_FLAG_SCAN_END) == SET) {
		KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, ENABLE);

		if (KeyScan_GetFlagState(keyscan, KEYSCAN_FLAG_EMPTY) != SET) {
			KeyScan_Read(keyscan, (uint16_t *)&new_keys, new_press_num);
		}

		KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_SCAN_END);
		KeyScan_INTMask(keyscan, KEYSCAN_INT_SCAN_END, DISABLE);

		kscan_bee_process(dev, new_press_num, new_keys);
	}

#if CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
	if (KeyScan_GetFlagState(keyscan, KEYSCAN_INT_FLAG_ALL_RELEASE) == SET) {

		kscan_bee_all_release_process(dev);
		KeyScan_ClearINTPendingBit(keyscan, KEYSCAN_INT_ALL_RELEASE);
	}
#endif
}

#ifdef CONFIG_PM_DEVICE
static PMCheckResult kscan_pm_check(void)
{
	return kscan_pm_check_state;
}

static void kscan_register_dlps_cb(void)
{
	platform_pm_register_callback_func_with_priority((void *)kscan_pm_check, PLATFORM_PM_CHECK,
							 1);
}

#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
static void pm_suspend_process_press(const struct device *dev)
{
	const struct kscan_bee_config *config = dev->config;
	struct kscan_bee_data *data = dev->data;
	const struct pinctrl_state *state;
	int ret;

	ret = pinctrl_lookup_state(config->pcfg, PINCTRL_STATE_SLEEP, &state);
	if ((ret < 0) && (ret != -ENOENT)) {
		/* no kscan wakeup pin is configured */
		return;
	}

	for (uint8_t i = 0; i < config->row_size + config->col_size; i++) {
		uint8_t wakeup_flag = state->pins[i].wakeup_high || state->pins[i].wakeup_low;
		uint8_t j = i >= config->col_size ? i - config->col_size : i;

		if (wakeup_flag) {
			if (data->press_rows & BIT(j)) {
				/* invert the wakeup level */
				if (state->pins[i].wakeup_high) {
					BEE_Pad_SetControlMode(state->pins[i].pin, PAD_SW_MODE);
#ifdef CONFIG_BEE_INPUT_KSCAN_RELEASE_WAKEUP
					BEE_System_WakeUpPinEnable(state->pins[i].pin,
								   PAD_WAKEUP_POL_LOW, DISABLE);
#else
					BEE_Pad_SetPullMode(state->pins[i].pin, PAD_PULL_UP);
#endif
				} else if (state->pins[i].wakeup_low) {
					BEE_Pad_SetControlMode(state->pins[i].pin, PAD_SW_MODE);
#ifdef CONFIG_BEE_INPUT_KSCAN_RELEASE_WAKEUP
					BEE_System_WakeUpPinEnable(state->pins[i].pin,
								   PAD_WAKEUP_POL_HIGH, DISABLE);
#else
					BEE_Pad_SetPullMode(state->pins[i].pin, PAD_PULL_DOWN);
#endif
				}
			} else {
				if (state->pins[i].wakeup_high) {
					BEE_Pad_SetControlMode(state->pins[i].pin, PAD_SW_MODE);
					BEE_System_WakeUpPinEnable(state->pins[i].pin,
								   PAD_WAKEUP_POL_HIGH, DISABLE);
				} else if (state->pins[i].wakeup_low) {
					BEE_Pad_SetControlMode(state->pins[i].pin, PAD_SW_MODE);
					BEE_System_WakeUpPinEnable(state->pins[i].pin,
								   PAD_WAKEUP_POL_LOW, DISABLE);
				}
			}
		} else {
			Pad_Config(state->pins[i].pin, PAD_SW_MODE, PAD_IS_PWRON,
				   state->pins[i].pull, state->pins[i].dir, state->pins[i].drive);
		}
	}
}
#endif

static int kscan_bee_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct kscan_bee_config *config = dev->config;
	struct kscan_bee_data *data;
	int ret;
	bool is_pad_wakeup = false;

	data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		const struct pinctrl_state *state;

		/* Move pins to sleep state */
#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
		if (!data->press_rows) {
			ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
			if ((ret < 0) && (ret != -ENOENT)) {
				return ret;
			}
		} else {
			pm_suspend_process_press(dev);
		}
#else
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
		if ((ret < 0) && (ret != -ENOENT)) {
			return ret;
		}
#endif
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* check wakeup pin status */
		ret = pinctrl_lookup_state(config->pcfg, PINCTRL_STATE_SLEEP, &state);
		if ((ret < 0) && (ret != -ENOENT)) {
			/* no kscan wakeup pin is configured */
			goto exit;
		}

		(void)clock_control_on(BEE_CLOCK_CONTROLLER,
				       (clock_control_subsys_t)&config->clkid);

		/* Set pins to active state */
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}

		/* there are kscan wakeup pins configured, check if they wakeup the system
		 */

		for (uint8_t i = 0U; i < state->pin_cnt; i++) {
			if (state->pins[i].wakeup_low || state->pins[i].wakeup_high) {
				System_WakeUpPinDisable(state->pins[i].pin);
				if (System_WakeUpInterruptValue(state->pins[i].pin) == SET) {
					is_pad_wakeup = true;
					kscan_pm_check_state = PM_CHECK_FAIL;
					Pad_ClearWakeupINTPendingBit(state->pins[i].pin);
				}
			}
		}

exit:
		/* Set pins to active state */
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}

#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
		if (is_pad_wakeup) {
			KEYSCAN_DLPSExit((void *)config->reg, &data->store_buf,
					 KeyScan_Manual_Scan_Mode, KeyScan_Manual_Sel_Bit);
		} else {
			if (data->timer_started == false) {
				KEYSCAN_DLPSExit((void *)config->reg, &data->store_buf,
						 KeyScan_Auto_Scan_Mode, KeyScan_Manual_Sel_Key);
			}
		}
#else
		KEYSCAN_DLPSExit((void *)config->reg, &data->store_buf, KeyScan_Auto_Scan_Mode,
				 KeyScan_Manual_Sel_Key);
#endif

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

#endif /* CONFIG_PM_DEVICE */

static int kscan_bee_init(const struct device *dev)
{
	LOG_DBG("dev %s init\n", dev->name);
	const struct kscan_bee_config *config = dev->config;
	struct kscan_bee_data *data = dev->data;

	memset(data->key_map, 0, sizeof(data->key_map));
	memset(data->keys, 0, sizeof(data->keys));

	pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);

#if !CONFIG_BEE_INPUT_KSCAN_AUTOSCAN_MODE
	kscan_bee_init_driver(dev, KeyScan_Manual_Scan_Mode, KeyScan_Manual_Sel_Key);
#else
	kscan_bee_init_driver(dev, KeyScan_Auto_Scan_Mode, KeyScan_Manual_Sel_Key);
#endif

	data->all_release_flag = true;

	config->irq_config_func();

#ifdef CONFIG_PM_DEVICE
	kscan_register_dlps_cb();
#endif
	return 0;
}

#define BEE_KSCAN_IRQ_HANDLER_DECL(index) static void kscan_bee_irq_config_func_##index(void);
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define BEE_KSCAN_IRQ_HANDLER(index)                                                               \
	static void kscan_bee_irq_config_func_##index(void)                                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), kscan_bee_isr,      \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define BEE_KSCAN_IRQ_HANDLER(index)                                                               \
	static void kscan_bee_irq_config_func_##index(void)                                        \
	{                                                                                          \
		RamVectorTableUpdate(Keyscan_VECTORn, kscan_bee_isr);                              \
		NVIC_InitTypeDef NVIC_InitStruct;                                                  \
		NVIC_InitStruct.NVIC_IRQChannel = KeyScan_IRQn;                                    \
		NVIC_InitStruct.NVIC_IRQChannelPriority = 2;                                       \
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;                                       \
		NVIC_Init(&NVIC_InitStruct);                                                       \
	}
#endif

#define BEE_KSCAN_IRQ_HANDLER_FUNC(index) .irq_config_func = kscan_bee_irq_config_func_##index,

#define BEE_KSCAN_INIT(index)                                                                      \
	BEE_KSCAN_IRQ_HANDLER_DECL(index)                                                          \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct kscan_bee_config kscan_bee_cfg_##index = {                             \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.row_size = DT_INST_PROP(index, row_size),                                         \
		.col_size = DT_INST_PROP(index, col_size),                                         \
		.deb_us = DT_INST_PROP_OR(index, debounce_time_us, 0),                             \
		.scan_us = DT_INST_PROP_OR(index, scan_time_us, 0),                                \
		.rel_us = DT_INST_PROP_OR(index, release_time_us, 0),                              \
		.scan_debounce_cnt = DT_INST_PROP(index, scan_debounce_cnt),                       \
		BEE_KSCAN_IRQ_HANDLER_FUNC(index)};                                                \
                                                                                                   \
	static struct kscan_bee_data kscan_bee_data_##index = {};                                  \
	PM_DEVICE_DT_INST_DEFINE(index, kscan_bee_pm_action);                                      \
	DEVICE_DT_INST_DEFINE(index, &kscan_bee_init, PM_DEVICE_DT_INST_GET(index),                \
			      &kscan_bee_data_##index, &kscan_bee_cfg_##index, POST_KERNEL,        \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);                                   \
                                                                                                   \
	BEE_KSCAN_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(BEE_KSCAN_INIT)
