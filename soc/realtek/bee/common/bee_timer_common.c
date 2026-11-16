/*
 * Copyright(c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bee_timer_common.h"
#include <soc.h>

#if defined(CONFIG_COUNTER_CAPTURE)
/* A latch FIFO threshold of 1 raises the interrupt on every latched edge. */
#define BEE_LATCH_FIFO_THRESHOLD 1U
#endif

#if defined(CONFIG_HAL_REALTEK_BEE_TIM)

/* Basic Timer Implementations */
#if defined(CONFIG_SOC_SERIES_RTL87X2G) || defined(CONFIG_SOC_SERIES_RTL8752H)
static void basic_tim_init(uint32_t reg, uint8_t prescaler_idx, uint32_t top_val,
			   enum bee_timer_mode mode, const struct bee_pwm_output_cfg *pwm_cfg)
{
	TIM_TimeBaseInitTypeDef timer_init_struct;

	ARG_UNUSED(pwm_cfg);

	TIM_StructInit(&timer_init_struct);

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	timer_init_struct.TIM_ClockSrc = CK_40M_TIMER;
	timer_init_struct.TIM_SOURCE_DIV_En = ENABLE;
#endif

	timer_init_struct.TIM_SOURCE_DIV = prescaler_idx;
	timer_init_struct.TIM_Mode = TIM_Mode_UserDefine;

	if (mode == BEE_TIMER_MODE_PWM) {
		timer_init_struct.TIM_PWM_En = ENABLE;
		timer_init_struct.TIM_PWM_High_Count = 0;
		timer_init_struct.TIM_PWM_Low_Count = UINT32_MAX;
	} else {
		timer_init_struct.TIM_Period = top_val;
	}

	TIM_TimeBaseInit((TIM_TypeDef *)reg, &timer_init_struct);
}

static void basic_tim_start(uint32_t reg)
{
	TIM_Cmd((TIM_TypeDef *)reg, ENABLE);
}

static void basic_tim_stop(uint32_t reg)
{
	TIM_Cmd((TIM_TypeDef *)reg, DISABLE);
}

static uint32_t basic_tim_get_count(uint32_t reg)
{
	return TIM_GetCurrentValue((TIM_TypeDef *)reg);
}

static uint32_t basic_tim_get_top(uint32_t reg)
{
	return TIM_GetCurrentLoadCnt(reg);
}

static void basic_tim_set_top(uint32_t reg, uint32_t top_val)
{
	TIM_ChangePeriod((TIM_TypeDef *)reg, top_val);
}

static void basic_tim_int_enable(uint32_t reg)
{
	TIM_INTConfig((TIM_TypeDef *)reg, ENABLE);
}

static void basic_tim_int_disable(uint32_t reg)
{
	TIM_INTConfig((TIM_TypeDef *)reg, DISABLE);
}

static void basic_tim_int_clear(uint32_t reg)
{
	TIM_ClearINT((TIM_TypeDef *)reg);
}

static bool basic_tim_int_status(uint32_t reg)
{
	return TIM_GetINTStatus((TIM_TypeDef *)reg) ? true : false;
}

static enum bee_pwm_output_mode basic_tim_set_pwm_duty(uint32_t reg, uint32_t period_cyc,
						       uint32_t pulse_cyc, bool inverted)
{
	uint32_t high, low;
	enum bee_pwm_output_mode output_mode = BEE_PWM_OUTPUT_MODE_TIMER;

	/* Basic Timer uses High Count vs Low Count logic */
	if (inverted) {
		if (period_cyc == 0U || pulse_cyc == 0U) {
			output_mode = BEE_PWM_OUTPUT_MODE_HIGH;
			high = UINT32_MAX;
			low = 0; /* duty = 0% */
		} else if (period_cyc == pulse_cyc) {
			output_mode = BEE_PWM_OUTPUT_MODE_LOW;
			high = 0;
			low = UINT32_MAX; /* duty = 100% */
		} else {
			output_mode = BEE_PWM_OUTPUT_MODE_TIMER;
			high = period_cyc - pulse_cyc;
			low = pulse_cyc;
		}
	} else {
		if (period_cyc == 0U || pulse_cyc == 0U) {
			output_mode = BEE_PWM_OUTPUT_MODE_LOW;
			high = 0;
			low = UINT32_MAX;
		} else if (period_cyc == pulse_cyc) {
			output_mode = BEE_PWM_OUTPUT_MODE_HIGH;
			high = UINT32_MAX;
			low = 0;
		} else {
			output_mode = BEE_PWM_OUTPUT_MODE_TIMER;
			high = pulse_cyc;
			low = period_cyc - pulse_cyc;
		}
	}

	TIM_PWMChangeFreqAndDuty((TIM_TypeDef *)reg, high, low);

	return output_mode;
}
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#define TIMER_MAX_CNT(reg) ((TIMER_TypeDef *)reg)->TIMER_MAX_CNT

static void basic_tim_init(uint32_t reg, uint8_t prescaler_idx, uint32_t top_val,
			   enum bee_timer_mode mode, const struct bee_pwm_output_cfg *pwm_cfg)
{
	TIMER_TimeBaseInitTypeDef timer_init_struct;

	TIMER_StructInit(&timer_init_struct);

	timer_init_struct.TIMER_ClockSrc = TIMER_CLOCK_SRC_40M;
	timer_init_struct.TIMER_ClockDiv = prescaler_idx;
	timer_init_struct.TIMER_Mode = TIMER_MODE_USERDEFINE;
	timer_init_struct.TIMER_OneShotEn = DISABLE;

	if (mode == BEE_TIMER_MODE_PWM) {
		timer_init_struct.PWM_En = ENABLE;
		timer_init_struct.TIMER_Period = UINT32_MAX;
		timer_init_struct.PWM_HighCount = 0;
		if (pwm_cfg != NULL) {
			timer_init_struct.PWM_Polarity =
				pwm_cfg->start_low ? PWM_POLARITY_LOW : PWM_POLARITY_HIGH;
			timer_init_struct.PWM_OutputMode =
				pwm_cfg->open_drain ? PWM_OPEN_DRAIN : PWM_PUSH_PULL;
		}
	} else {
		timer_init_struct.TIMER_Period = top_val;
	}

	TIMER_TimeBaseInit((TIMER_TypeDef *)reg, &timer_init_struct);
}

static void basic_tim_start(uint32_t reg)
{
	TIMER_Cmd((TIMER_TypeDef *)reg, ENABLE);
}

static void basic_tim_stop(uint32_t reg)
{
	TIMER_Cmd((TIMER_TypeDef *)reg, DISABLE);
}

static uint32_t basic_tim_get_count(uint32_t reg)
{
	return TIMER_GetCurrentValue((TIMER_TypeDef *)reg);
}

static uint32_t basic_tim_get_top(uint32_t reg)
{
	return TIMER_MAX_CNT(reg);
}

static void basic_tim_set_top(uint32_t reg, uint32_t top_val)
{
	TIMER_ChangePeriod((TIMER_TypeDef *)reg, top_val);
}

static void basic_tim_int_enable(uint32_t reg)
{
	TIMER_INTConfig((TIMER_TypeDef *)reg, TIMER_INT_TIMEOUT, ENABLE);
}

static void basic_tim_int_disable(uint32_t reg)
{
	TIMER_INTConfig((TIMER_TypeDef *)reg, TIMER_INT_TIMEOUT, DISABLE);
}

static void basic_tim_int_clear(uint32_t reg)
{
	TIMER_ClearINT((TIMER_TypeDef *)reg, TIMER_INT_TIMEOUT);
}

static bool basic_tim_int_status(uint32_t reg)
{
	return TIMER_GetINTStatus((TIMER_TypeDef *)reg, TIMER_INT_TIMEOUT) ? true : false;
}

#if defined(CONFIG_COUNTER_CAPTURE)
/* Not exposed in rtl_timer.h; declared the same way as rtl_timer.c does. */
extern TIMERCASE_TypeDef TIMER_GetTimerCase(TIMER_TypeDef *TIMERx);
extern void TIMER_SetLatchTriggerPad(TIMER_TypeDef *TIMERx, uint8_t Pin_num);
extern void TIMER_LatchDebCmd(TIMER_TypeDef *TIMERx, FunctionalState Newstate);
extern void TIMER_SetLatchDebClock(TIMER_TypeDef *TIMERx, uint16_t ClockSrc, uint16_t ClockDiv,
				   uint16_t CountLimit);

static int basic_tim_latch_configure(uint32_t reg, const struct bee_latch_cfg *cfg)
{
	TIMER_TypeDef *timer = (TIMER_TypeDef *)reg;
	TIMER_LAT_CNT_0_CFG_TypeDef lat_cfg;

	/* Only TIMER2_CH0 implements the GPIO latch count. */
	if (TIMER_GetTimerCase(timer) < TIMER_CASE4) {
		return -ENOTSUP;
	}

	/* TIMER_TimeBaseInit() only sets up the latch in free-run mode, so
	 * program the latch registers directly.
	 */
	TIMER_SetLatchTriggerPad(timer, cfg->trigger_pad);
	TIMER_LatchDebCmd(timer, DISABLE);
	if (cfg->debounce_en) {
		/* The clock source argument is ignored, the debounce clock is
		 * fixed to 32 kHz.
		 */
		TIMER_SetLatchDebClock(timer, TIMER_CLOCK_SRC_40M, cfg->debounce_div,
				       cfg->debounce_count);
		TIMER_LatchDebCmd(timer, ENABLE);
	}

	lat_cfg.d32 = timer->TIMER_LAT_CNT_0_CFG;
	lat_cfg.b.reg_timer_latch_cnt_0_en = DISABLE;
	lat_cfg.b.reg_timer_latch_cnt_0_trig_mode = cfg->trigger;
	lat_cfg.b.reg_timer_latch_cnt_0_fifo_th = BEE_LATCH_FIFO_THRESHOLD;
	timer->TIMER_LAT_CNT_0_CFG = lat_cfg.d32;

	return 0;
}

static void basic_tim_latch_cmd(TIMER_TypeDef *timer, FunctionalState state)
{
	TIMER_LAT_CNT_0_CFG_TypeDef lat_cfg = {.d32 = timer->TIMER_LAT_CNT_0_CFG};

	lat_cfg.b.reg_timer_latch_cnt_0_en = state;
	timer->TIMER_LAT_CNT_0_CFG = lat_cfg.d32;
}

static void basic_tim_latch_enable(uint32_t reg)
{
	TIMER_TypeDef *timer = (TIMER_TypeDef *)reg;

	basic_tim_latch_cmd(timer, ENABLE);
	TIMER_INTConfig(timer, TIMER_INT_LATCH_FIFO_THRESHOLD, ENABLE);
}

static void basic_tim_latch_disable(uint32_t reg)
{
	TIMER_TypeDef *timer = (TIMER_TypeDef *)reg;

	TIMER_INTConfig(timer, TIMER_INT_LATCH_FIFO_THRESHOLD, DISABLE);
	basic_tim_latch_cmd(timer, DISABLE);
}

static bool basic_tim_latch_int_status(uint32_t reg)
{
	return TIMER_GetINTStatus((TIMER_TypeDef *)reg, TIMER_INT_LATCH_FIFO_THRESHOLD) == SET;
}

static void basic_tim_latch_int_clear(uint32_t reg)
{
	TIMER_ClearINT((TIMER_TypeDef *)reg, TIMER_INT_LATCH_FIFO_THRESHOLD);
}

static uint8_t basic_tim_latch_get_fifo_len(uint32_t reg)
{
	return TIMER_GetLatchFIFOLength((TIMER_TypeDef *)reg);
}

static uint32_t basic_tim_latch_read_fifo(uint32_t reg)
{
	return ((TIMER_TypeDef *)reg)->TIMER_LAT_CNT_0;
}

static void basic_tim_latch_clear(uint32_t reg)
{
	TIMER_ClearFIFO((TIMER_TypeDef *)reg, TIMER_CLEAR_LATCH_FIFO);
	basic_tim_latch_int_clear(reg);
}
#endif /* CONFIG_COUNTER_CAPTURE */

static enum bee_pwm_output_mode basic_tim_set_pwm_duty(uint32_t reg, uint32_t period_cyc,
						       uint32_t pulse_cyc, bool inverted)
{
	uint32_t high, low;
	enum bee_pwm_output_mode output_mode = BEE_PWM_OUTPUT_MODE_TIMER;

	/* Basic Timer uses High Count vs Low Count logic */
	if (inverted) {
		if (period_cyc == 0U || pulse_cyc == 0U) {
			output_mode = BEE_PWM_OUTPUT_MODE_HIGH;
			high = UINT32_MAX;
			low = 0; /* duty = 0% */
		} else if (period_cyc == pulse_cyc) {
			output_mode = BEE_PWM_OUTPUT_MODE_LOW;
			high = 0;
			low = UINT32_MAX; /* duty = 100% */
		} else {
			output_mode = BEE_PWM_OUTPUT_MODE_TIMER;
			high = period_cyc - pulse_cyc;
			low = pulse_cyc;
		}
	} else {
		if (period_cyc == 0U || pulse_cyc == 0U) {
			output_mode = BEE_PWM_OUTPUT_MODE_LOW;
			high = 0;
			low = UINT32_MAX;
		} else if (period_cyc == pulse_cyc) {
			output_mode = BEE_PWM_OUTPUT_MODE_HIGH;
			high = UINT32_MAX;
			low = 0;
		} else {
			output_mode = BEE_PWM_OUTPUT_MODE_TIMER;
			high = pulse_cyc;
			low = period_cyc - pulse_cyc;
		}
	}

	TIMER_PWMChangeFreqAndDuty((TIMER_TypeDef *)reg, high + low, high);

	return output_mode;
}
#endif

static const struct bee_timer_ops bee_basic_ops = {
	.init = basic_tim_init,
	.start = basic_tim_start,
	.stop = basic_tim_stop,
	.get_count = basic_tim_get_count,
	.get_top = basic_tim_get_top,
	.set_top = basic_tim_set_top,
	.set_pwm_duty = basic_tim_set_pwm_duty,
	.int_enable = basic_tim_int_enable,
	.int_disable = basic_tim_int_disable,
	.int_clear = basic_tim_int_clear,
	.int_status = basic_tim_int_status,
#if defined(CONFIG_SOC_SERIES_RTL87X2J) && defined(CONFIG_COUNTER_CAPTURE)
	.latch_configure = basic_tim_latch_configure,
	.latch_enable = basic_tim_latch_enable,
	.latch_disable = basic_tim_latch_disable,
	.latch_int_status = basic_tim_latch_int_status,
	.latch_int_clear = basic_tim_latch_int_clear,
	.latch_get_fifo_len = basic_tim_latch_get_fifo_len,
	.latch_read_fifo = basic_tim_latch_read_fifo,
	.latch_clear = basic_tim_latch_clear,
#endif
};
#endif /* CONFIG_HAL_REALTEK_BEE_TIM */

#if defined(CONFIG_HAL_REALTEK_BEE_ENHTIM)

/* Enhanced Timer Implementations */

static void enh_tim_init(uint32_t reg, uint8_t prescaler_idx, uint32_t top_val,
			 enum bee_timer_mode mode, const struct bee_pwm_output_cfg *pwm_cfg)
{
	ENHTIM_InitTypeDef enh_tim_init_struct;

	ARG_UNUSED(pwm_cfg);

	ENHTIM_StructInit(&enh_tim_init_struct);

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	enh_tim_init_struct.ENHTIM_ClockSource = CK_40M_TIMER;
	enh_tim_init_struct.ENHTIM_ClockDiv_En = ENABLE;
	if (mode == BEE_TIMER_MODE_PWM) {
		enh_tim_init_struct.ENHTIM_PWMOutputEn = ENABLE;
	}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
	if (mode == BEE_TIMER_MODE_PWM) {
		enh_tim_init_struct.ENHTIM_PWMOutputEn = ENHTIM_PWM_ENABLE;
	}
#endif

	enh_tim_init_struct.ENHTIM_ClockDiv = prescaler_idx;
	enh_tim_init_struct.ENHTIM_Mode =
		ENHTIM_MODE_PWM_MANUAL; /* Used for both Count and PWM modes */
	enh_tim_init_struct.ENHTIM_MaxCount = top_val;

	if (mode == BEE_TIMER_MODE_PWM) {
		enh_tim_init_struct.ENHTIM_PWMStartPolarity = ENHTIM_PWM_START_WITH_LOW;
		enh_tim_init_struct.ENHTIM_CCValue = UINT32_MAX;
	}

	ENHTIM_Init((ENHTIM_TypeDef *)reg, &enh_tim_init_struct);
}

static void enh_tim_start(uint32_t reg)
{
	ENHTIM_Cmd((ENHTIM_TypeDef *)reg, ENABLE);
}

static void enh_tim_stop(uint32_t reg)
{
	ENHTIM_Cmd((ENHTIM_TypeDef *)reg, DISABLE);
}

static uint32_t enh_tim_get_count(uint32_t reg)
{
	return ENHTIM_GetCurrentCount((ENHTIM_TypeDef *)reg);
}

static uint32_t enh_tim_get_top(uint32_t reg)
{
	return ENHTIM_GetCurrentMAXCNT(reg);
}

static void enh_tim_set_top(uint32_t reg, uint32_t top_val)
{
	ENHTIM_SetMaxCount((ENHTIM_TypeDef *)reg, top_val);
}

static void enh_tim_int_enable(uint32_t reg)
{
	ENHTIM_INTConfig((ENHTIM_TypeDef *)reg, ENHTIM_INT_TIM, ENABLE);
}

static void enh_tim_int_disable(uint32_t reg)
{
	ENHTIM_INTConfig((ENHTIM_TypeDef *)reg, ENHTIM_INT_TIM, DISABLE);
}

static void enh_tim_int_clear(uint32_t reg)
{
	ENHTIM_ClearINTPendingBit((ENHTIM_TypeDef *)reg, ENHTIM_INT_TIM);
}

static bool enh_tim_int_status(uint32_t reg)
{
	return ENHTIM_GetINTStatus((ENHTIM_TypeDef *)reg, ENHTIM_INT_TIM) ? true : false;
}

#if defined(CONFIG_COUNTER_CAPTURE)
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
/* Latch count 0 is the GPIO triggered one. */
#define ENHTIM_GPIO_LATCH      LATCH_CNT_0
#define ENHTIM_GPIO_LATCH_INT  ENHTIM_INT_LATCH_CNT_FIFO_THD
#define ENHTIM_GPIO_LATCH_FIFO ENHTIM_FIFO_CLR_CNT0

/* Not exposed in rtl_enh_tim.h; declared the same way as rtl_enh_tim.c does. */
extern uint32_t ENHTIM_GetTimerID(ENHTIM_TypeDef *ENHTIMx);
extern void ENHTIM_ConfigLatchTrigger(uint32_t enhtim_id,
				      const ENHTIM_InitTypeDef *ENHTIM_InitStruct);

static int enh_tim_latch_configure(uint32_t reg, const struct bee_latch_cfg *cfg)
{
	ENHTIM_TypeDef *enhtim = (ENHTIM_TypeDef *)reg;
	ENHTIM_InitTypeDef init = {.ENHTIM_LatchTriggerPad = cfg->trigger_pad};
	ENHTIM_CONFIGURE_TypeDef conf;

	if (cfg->debounce_en) {
		return -ENOTSUP;
	}

	/* ENHTIM_Init() only sets up the latch trigger when the latch is enabled
	 * at init time, so set it up here.
	 */
	ENHTIM_ConfigLatchTrigger(ENHTIM_GetTimerID(enhtim), &init);

	conf.d32 = enhtim->ENHTIM_CONFIGURE;
	conf.b.Enhtimer_Latch_cnt_0_en = DISABLE;
	conf.b.Enhtimer_latch_cnt_0_trig_mode = cfg->trigger;
	conf.b.Enhtimer_latch_cnt_0_fifo_th = BEE_LATCH_FIFO_THRESHOLD;
	enhtim->ENHTIM_CONFIGURE = conf.d32;

	return 0;
}

static uint8_t enh_tim_latch_get_fifo_len(uint32_t reg)
{
	return ENHTIM_GetLatchCountFIFOLength((ENHTIM_TypeDef *)reg);
}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
/* Latch count 2 is the GPIO triggered one and the only one with a FIFO. */
#define ENHTIM_GPIO_LATCH      LATCH_CNT_2
#define ENHTIM_GPIO_LATCH_INT  ENHTIM_INT_LATCH_CNT2_FIFO_THD
#define ENHTIM_GPIO_LATCH_FIFO ENHTIM_FIFO_CLR_CNT2

/* ENHTIM_Init() is the only HAL API that sets up latch count 2, so follow its
 * setup here.
 */
static int enh_tim_latch_configure(uint32_t reg, const struct bee_latch_cfg *cfg)
{
	ENHTIM_TypeDef *enhtim = (ENHTIM_TypeDef *)reg;
	uint32_t enhtim_id = ((uint32_t)enhtim - (uint32_t)ENH_TIM0) / 0x24;
	uint16_t pad = cfg->trigger_pad;
	uint32_t temp = 0x3F;

	if (cfg->debounce_en) {
		return -ENOTSUP;
	}

	ENHTIM_LatchCountDisable(enhtim, ENHTIM_GPIO_LATCH);
	enhtim->CR &= ~((0x03 << (ENHTIM_GPIO_LATCH * 2 + 4)) | (0x1F << 23));
	enhtim->CR |= ((cfg->trigger & 0x03) << (ENHTIM_GPIO_LATCH * 2 + 4)) |
		      ((BEE_LATCH_FIFO_THRESHOLD & 0x1F) << 23);

	if ((pad >= P1_6) && (pad <= P3_6)) {
		temp = pad - 4;
	} else if ((pad >= P4_0) && (pad <= P5_2)) {
		temp = pad - 5;
	} else if (pad <= P1_1) {
		temp = pad;
	}

	/* TimerGPIOTriggerEn (bit 8) gates the pad trigger into latch count 2,
	 * nothing is latched without it.
	 */
	ENHTIM_LATCH_COUNT_CR &= ~0x1FF;
	ENHTIM_LATCH_COUNT_CR |= BIT(8) | ((enhtim_id & 0x01) << 6) | (temp & 0x3F);

	return 0;
}

static uint8_t enh_tim_latch_get_fifo_len(uint32_t reg)
{
	return ENHTIM_GetLatchCount2FIFOLength((ENHTIM_TypeDef *)reg);
}
#endif

static void enh_tim_latch_enable(uint32_t reg)
{
	ENHTIM_TypeDef *enhtim = (ENHTIM_TypeDef *)reg;

	ENHTIM_LatchCountEnable(enhtim, ENHTIM_GPIO_LATCH);
	ENHTIM_INTConfig(enhtim, ENHTIM_GPIO_LATCH_INT, ENABLE);
}

static void enh_tim_latch_disable(uint32_t reg)
{
	ENHTIM_TypeDef *enhtim = (ENHTIM_TypeDef *)reg;

	ENHTIM_INTConfig(enhtim, ENHTIM_GPIO_LATCH_INT, DISABLE);
	ENHTIM_LatchCountDisable(enhtim, ENHTIM_GPIO_LATCH);
}

static bool enh_tim_latch_int_status(uint32_t reg)
{
	return ENHTIM_GetINTStatus((ENHTIM_TypeDef *)reg, ENHTIM_GPIO_LATCH_INT) == SET;
}

static void enh_tim_latch_int_clear(uint32_t reg)
{
	ENHTIM_ClearINTPendingBit((ENHTIM_TypeDef *)reg, ENHTIM_GPIO_LATCH_INT);
}

static uint32_t enh_tim_latch_read_fifo(uint32_t reg)
{
	return ENHTIM_GetLatchCount((ENHTIM_TypeDef *)reg, ENHTIM_GPIO_LATCH);
}

static void enh_tim_latch_clear(uint32_t reg)
{
	ENHTIM_ClearFIFO((ENHTIM_TypeDef *)reg, ENHTIM_GPIO_LATCH_FIFO);
	enh_tim_latch_int_clear(reg);
}
#endif /* CONFIG_COUNTER_CAPTURE */

static enum bee_pwm_output_mode enh_tim_set_pwm_duty(uint32_t reg, uint32_t period_cyc,
						     uint32_t pulse_cyc, bool inverted)
{
	uint32_t cc_val, max_val;
	enum bee_pwm_output_mode output_mode = BEE_PWM_OUTPUT_MODE_TIMER;

	if (inverted) {
		if (period_cyc == 0U || pulse_cyc == 0U) {
			output_mode = BEE_PWM_OUTPUT_MODE_HIGH;
			max_val = UINT32_MAX - 1;
			cc_val = 0;
		} else if (period_cyc == pulse_cyc) {
			output_mode = BEE_PWM_OUTPUT_MODE_LOW;
			max_val = UINT32_MAX - 1;
			cc_val = UINT32_MAX;
		} else {
			output_mode = BEE_PWM_OUTPUT_MODE_TIMER;
			max_val = period_cyc;
			cc_val = period_cyc - pulse_cyc;
		}
	} else {
		if (period_cyc == 0U || pulse_cyc == 0U) {
			output_mode = BEE_PWM_OUTPUT_MODE_LOW;
			max_val = UINT32_MAX - 1;
			cc_val = UINT32_MAX;
		} else if (period_cyc == pulse_cyc) {
			output_mode = BEE_PWM_OUTPUT_MODE_HIGH;
			max_val = UINT32_MAX - 1;
			cc_val = 0;
		} else {
			output_mode = BEE_PWM_OUTPUT_MODE_TIMER;
			max_val = period_cyc;
			cc_val = pulse_cyc;
		}
	}

	ENHTIM_SetMaxCount((ENHTIM_TypeDef *)reg, max_val);
	ENHTIM_SetCCValue((ENHTIM_TypeDef *)reg, cc_val);

	return output_mode;
}

static const struct bee_timer_ops bee_enh_ops = {
	.init = enh_tim_init,
	.start = enh_tim_start,
	.stop = enh_tim_stop,
	.get_count = enh_tim_get_count,
	.get_top = enh_tim_get_top,
	.set_top = enh_tim_set_top,
	.set_pwm_duty = enh_tim_set_pwm_duty,
	.int_enable = enh_tim_int_enable,
	.int_disable = enh_tim_int_disable,
	.int_clear = enh_tim_int_clear,
	.int_status = enh_tim_int_status,
#if defined(CONFIG_COUNTER_CAPTURE)
	.latch_configure = enh_tim_latch_configure,
	.latch_enable = enh_tim_latch_enable,
	.latch_disable = enh_tim_latch_disable,
	.latch_int_status = enh_tim_latch_int_status,
	.latch_int_clear = enh_tim_latch_int_clear,
	.latch_get_fifo_len = enh_tim_latch_get_fifo_len,
	.latch_read_fifo = enh_tim_latch_read_fifo,
	.latch_clear = enh_tim_latch_clear,
#endif
};
#endif /* CONFIG_HAL_REALTEK_BEE_ENHTIM */

/* Public API Implementation */
const struct bee_timer_ops *bee_timer_get_ops(bool enhanced)
{
	if (enhanced) {
#if defined(CONFIG_HAL_REALTEK_BEE_ENHTIM)
		return &bee_enh_ops;
#else
		return NULL;
#endif
	} else {
#if defined(CONFIG_HAL_REALTEK_BEE_TIM)
		return &bee_basic_ops;
#else
		return NULL;
#endif
	}
}
