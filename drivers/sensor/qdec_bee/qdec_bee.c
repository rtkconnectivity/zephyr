/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_qdec

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/drivers/sensor/qdec_bee.h>
#include <soc.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <rtl876x_qdec.h>
#include <rtl876x_nvic.h>
#include "vector_table.h"

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include "trace.h"

LOG_MODULE_REGISTER(qdec_bee, CONFIG_SENSOR_LOG_LEVEL);
#define FULL_ANGLE       360
#define MAX_ACC_CNT_BITS 16

#ifdef CONFIG_PM_DEVICE
	extern void QDEC_DLPSEnter(void *PeriReg, void *StoreBuf);
	extern void QDEC_DLPSExit(void *PeriReg, void *StoreBuf);
#endif

struct qdec_bee_axis_data {
	int32_t acc;
	int16_t round;
	uint8_t counts_per_revolution;
	uint32_t debounce_time_ms;
	sensor_trigger_handler_t data_ready_handler;
	const struct sensor_trigger *data_ready_trigger;
#ifdef CONFIG_PM_DEVICE
	int32_t pm_acc;
#endif
};

struct qdec_bee_data {
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	struct qdec_bee_axis_data x;
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	struct qdec_bee_axis_data y;
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	struct qdec_bee_axis_data z;
#endif
#ifdef CONFIG_PM_DEVICE
	QDECStoreReg_Typedef store_buf;
#endif
};

struct qdec_bee_config {
	uint32_t reg;
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_connect)(void);
};

static int qdec_bee_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct qdec_bee_config *config = dev->config;
	QDEC_TypeDef *qdec = (QDEC_TypeDef *)config->reg;
	struct qdec_bee_data *data = dev->data;
	uint16_t acc_cnt;
	unsigned int key;

	if (1
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	    && ((enum sensor_attribute_qdec_bee)chan != SENSOR_ATTR_QDEC_X_ROTATION)
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	    && ((enum sensor_attribute_qdec_bee)chan != SENSOR_ATTR_QDEC_Y_ROTATION)
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	    && ((enum sensor_attribute_qdec_bee)chan != SENSOR_ATTR_QDEC_Z_ROTATION)
#endif
	    && (chan != SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}

	key = irq_lock();
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	if ((enum sensor_attribute_qdec_bee)chan == SENSOR_ATTR_QDEC_X_ROTATION ||
	    (chan == SENSOR_CHAN_ALL)) {
		acc_cnt = QDEC_GetAxisCount(qdec, QDEC_AXIS_X);
#ifdef CONFIG_PM_DEVICE
		data->x.acc = data->x.round * 65536 + acc_cnt + data->x.pm_acc;
#else
		data->x.acc = data->x.round * 65536 + acc_cnt;
#endif
	}
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	if ((enum sensor_attribute_qdec_bee)chan == SENSOR_ATTR_QDEC_Y_ROTATION ||
	    (chan == SENSOR_CHAN_ALL)) {
		acc_cnt = QDEC_GetAxisCount(qdec, QDEC_AXIS_Y);
#ifdef CONFIG_PM_DEVICE
		data->y.acc = data->y.round * 65536 + acc_cnt + data->y.pm_acc;
#else
		data->y.acc = data->y.round * 65536 + acc_cnt;
#endif
	}
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	if ((enum sensor_attribute_qdec_bee)chan == SENSOR_ATTR_QDEC_Z_ROTATION ||
	    (chan == SENSOR_CHAN_ALL)) {
		acc_cnt = QDEC_GetAxisCount(qdec, QDEC_AXIS_Z);
#ifdef CONFIG_PM_DEVICE
		data->z.acc = data->z.round * 65536 + acc_cnt + data->z.pm_acc;
#else
		data->z.acc = data->z.round * 65536 + acc_cnt;
#endif
	}
#endif
	irq_unlock(key);

	return 0;
}

static int qdec_bee_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct qdec_bee_data *data = dev->data;
	int32_t acc;

	switch ((enum sensor_attribute_qdec_bee)chan) {
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	case SENSOR_ATTR_QDEC_X_ROTATION:
		acc = (int32_t)data->x.acc;
		val->val1 = acc;
		val->val2 = 0;
		break;
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	case SENSOR_ATTR_QDEC_Y_ROTATION:
		acc = (int32_t)data->y.acc;
		val->val1 = acc;
		val->val2 = 0;
		break;
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	case SENSOR_ATTR_QDEC_Z_ROTATION:
		acc = (int32_t)data->z.acc;
		val->val1 = acc;
		val->val2 = 0;
		break;
#endif

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int qdec_bee_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				sensor_trigger_handler_t handler)
{
	const struct qdec_bee_config *config = dev->config;
	struct qdec_bee_data *data = dev->data;
	QDEC_TypeDef *qdec = (QDEC_TypeDef *)config->reg;
	unsigned int key;

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	if (1
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	    && ((enum sensor_attribute_qdec_bee)(trig->chan) != SENSOR_ATTR_QDEC_X_ROTATION)
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	    && ((enum sensor_attribute_qdec_bee)(trig->chan) != SENSOR_ATTR_QDEC_Y_ROTATION)
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	    && ((enum sensor_attribute_qdec_bee)(trig->chan) != SENSOR_ATTR_QDEC_Z_ROTATION)
#endif
	) {
		return -ENOTSUP;
	}

	if (handler) {
		key = irq_lock();
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
		if ((enum sensor_attribute_qdec_bee)(trig->chan) == SENSOR_ATTR_QDEC_X_ROTATION) {
			data->x.data_ready_handler = handler;
			data->x.data_ready_trigger = trig;
			QDEC_INTMask(qdec, QDEC_X_CT_INT_MASK, DISABLE);
			QDEC_INTMask(qdec, QDEC_X_ILLEGAL_INT_MASK, DISABLE);
			QDEC_INTConfig(qdec, QDEC_X_INT_NEW_DATA, ENABLE);
			QDEC_INTConfig(qdec, QDEC_X_INT_ILLEGAL, ENABLE);
		}
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
		if ((enum sensor_attribute_qdec_bee)(trig->chan) == SENSOR_ATTR_QDEC_Y_ROTATION) {
			data->y.data_ready_handler = handler;
			data->y.data_ready_trigger = trig;
			QDEC_INTMask(qdec, QDEC_Y_CT_INT_MASK, DISABLE);
			QDEC_INTMask(qdec, QDEC_Y_ILLEGAL_INT_MASK, DISABLE);
			QDEC_INTConfig(qdec, QDEC_Y_INT_NEW_DATA, ENABLE);
			QDEC_INTConfig(qdec, QDEC_Y_INT_ILLEGAL, ENABLE);
		}
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
		if ((enum sensor_attribute_qdec_bee)(trig->chan) == SENSOR_ATTR_QDEC_Z_ROTATION) {
			data->z.data_ready_handler = handler;
			data->z.data_ready_trigger = trig;
			QDEC_INTMask(qdec, QDEC_Z_CT_INT_MASK, DISABLE);
			QDEC_INTMask(qdec, QDEC_Z_ILLEGAL_INT_MASK, DISABLE);
			QDEC_INTConfig(qdec, QDEC_Z_INT_NEW_DATA, ENABLE);
			QDEC_INTConfig(qdec, QDEC_Z_INT_ILLEGAL, ENABLE);
		}
#endif
		irq_unlock(key);
	} else {
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
		if ((enum sensor_attribute_qdec_bee)(trig->chan) == SENSOR_ATTR_QDEC_X_ROTATION) {
			data->x.data_ready_handler = NULL;
			data->x.data_ready_trigger = trig;
			QDEC_INTMask(qdec, QDEC_X_CT_INT_MASK, ENABLE);
			QDEC_INTMask(qdec, QDEC_X_ILLEGAL_INT_MASK, ENABLE);
			QDEC_INTConfig(qdec, QDEC_X_INT_NEW_DATA, DISABLE);
			QDEC_INTConfig(qdec, QDEC_X_INT_ILLEGAL, DISABLE);
		}
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
		if ((enum sensor_attribute_qdec_bee)(trig->chan) == SENSOR_ATTR_QDEC_Y_ROTATION) {
			data->y.data_ready_handler = NULL;
			data->y.data_ready_trigger = trig;
			QDEC_INTMask(qdec, QDEC_Y_CT_INT_MASK, ENABLE);
			QDEC_INTMask(qdec, QDEC_Y_ILLEGAL_INT_MASK, ENABLE);
			QDEC_INTConfig(qdec, QDEC_Y_INT_NEW_DATA, DISABLE);
			QDEC_INTConfig(qdec, QDEC_Y_INT_ILLEGAL, DISABLE);
		}
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
		if ((enum sensor_attribute_qdec_bee)(trig->chan) == SENSOR_ATTR_QDEC_Z_ROTATION) {
			data->z.data_ready_handler = NULL;
			data->z.data_ready_trigger = trig;
			QDEC_INTMask(qdec, QDEC_Z_CT_INT_MASK, ENABLE);
			QDEC_INTMask(qdec, QDEC_Z_ILLEGAL_INT_MASK, ENABLE);
			QDEC_INTConfig(qdec, QDEC_Z_INT_NEW_DATA, DISABLE);
			QDEC_INTConfig(qdec, QDEC_Z_INT_ILLEGAL, DISABLE);
		}
#endif
	}

	return 0;
}

static void qdec_bee_isr(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct qdec_bee_data *data = dev->data;
	const struct qdec_bee_config *config = dev->config;
	QDEC_TypeDef *qdec = (QDEC_TypeDef *)config->reg;
	sensor_trigger_handler_t handler;
	const struct sensor_trigger *trig;

#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	if (QDEC_GetFlagState(qdec, QDEC_FLAG_ILLEGAL_STATUS_X)) {
		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_ILLEGAL_INT_X);
		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_ILLEGAL_CT_X);
		LOG_ERR("X axis qdec illegal status\n");
	} else if (QDEC_GetFlagState(qdec, QDEC_FLAG_NEW_CT_STATUS_X)) {
		if (QDEC_GetFlagState(qdec, QDEC_FLAG_OVERFLOW_X)) {
			data->x.round += 1;
			QDEC_ClearINTPendingBit(qdec, QDEC_CLR_OVERFLOW_X);
		}
		if (QDEC_GetFlagState(qdec, QDEC_FLAG_UNDERFLOW_X)) {
			data->x.round -= 1;
			QDEC_ClearINTPendingBit(qdec, QDEC_CLR_UNDERFLOW_X);
		}

		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_NEW_CT_X);
		handler = data->x.data_ready_handler;
		trig = data->x.data_ready_trigger;
		if (handler) {
			handler(dev, trig);
		}
	}
#endif

#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	if (QDEC_GetFlagState(qdec, QDEC_FLAG_ILLEGAL_STATUS_Y)) {
		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_ILLEGAL_INT_Y);
		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_ILLEGAL_CT_Y);
		LOG_ERR("Y axis qdec illegal status\n");
	} else if (QDEC_GetFlagState(qdec, QDEC_FLAG_NEW_CT_STATUS_Y)) {
		if (QDEC_GetFlagState(qdec, QDEC_FLAG_OVERFLOW_Y)) {
			data->y.round += 1;
			QDEC_ClearINTPendingBit(qdec, QDEC_CLR_OVERFLOW_Y);
		}
		if (QDEC_GetFlagState(qdec, QDEC_FLAG_UNDERFLOW_Y)) {
			data->y.round -= 1;
			QDEC_ClearINTPendingBit(qdec, QDEC_CLR_UNDERFLOW_Y);
		}

		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_NEW_CT_Y);
		handler = data->y.data_ready_handler;
		trig = data->y.data_ready_trigger;
		if (handler) {
			handler(dev, trig);
		}
	}
#endif

#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	if (QDEC_GetFlagState(qdec, QDEC_FLAG_ILLEGAL_STATUS_Z)) {
		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_ILLEGAL_INT_Z);
		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_ILLEGAL_CT_Z);
		LOG_ERR("Z axis qdec illegal status\n");
	} else if (QDEC_GetFlagState(qdec, QDEC_FLAG_NEW_CT_STATUS_Z)) {
		if (QDEC_GetFlagState(qdec, QDEC_FLAG_OVERFLOW_Z)) {
			data->z.round += 1;
			QDEC_ClearINTPendingBit(qdec, QDEC_CLR_OVERFLOW_Z);
		}
		if (QDEC_GetFlagState(qdec, QDEC_FLAG_UNDERFLOW_Z)) {
			data->z.round -= 1;
			QDEC_ClearINTPendingBit(qdec, QDEC_CLR_UNDERFLOW_Z);
		}

		QDEC_ClearINTPendingBit(qdec, QDEC_CLR_NEW_CT_Z);
		handler = data->z.data_ready_handler;
		trig = data->z.data_ready_trigger;
		if (handler) {
			handler(dev, trig);
		}
	}
#endif
}

#ifdef CONFIG_PM_DEVICE
static int qdec_bee_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct qdec_bee_config *config = dev->config;
	struct qdec_bee_data *data = dev->data;
	QDEC_TypeDef *qdec = (QDEC_TypeDef *)config->reg;
	uint16_t acc_cnt;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		QDEC_DLPSEnter(qdec, &data->store_buf);
#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
		acc_cnt = QDEC_GetAxisCount(qdec, QDEC_AXIS_X);
		data->x.pm_acc = data->x.round * 65536 + acc_cnt + data->x.pm_acc;
		data->x.round = 0;
#endif
#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
		acc_cnt = QDEC_GetAxisCount(qdec, QDEC_AXIS_Y);
		data->y.pm_acc = data->y.round * 65536 + acc_cnt + data->y.pm_acc;
		data->y.round = 0;
#endif
#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
		acc_cnt = QDEC_GetAxisCount(qdec, QDEC_AXIS_Z);
		data->z.pm_acc = data->z.round * 65536 + acc_cnt + data->z.pm_acc;
		data->z.round = 0;
#endif

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

		(void)clock_control_on(BEE_CLOCK_CONTROLLER,
				       (clock_control_subsys_t)&config->clkid);

		QDEC_DLPSExit(qdec, &data->store_buf);

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct sensor_driver_api qdec_bee_driver_api = {
	.sample_fetch = qdec_bee_sample_fetch,
	.channel_get = qdec_bee_channel_get,
	.trigger_set = qdec_bee_trigger_set,
};

static int qdec_bee_init(const struct device *dev)
{
	struct qdec_bee_data *data = dev->data;
	const struct qdec_bee_config *config = dev->config;
	QDEC_TypeDef *qdec = (QDEC_TypeDef *)config->reg;
	int ret = 0;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);

	if (ret < 0) {
		return ret;
	}

	QDEC_InitTypeDef qdec_init_struct;

	QDEC_StructInit(&qdec_init_struct);

#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	if (data->x.counts_per_revolution == 2) {
		qdec_init_struct.counterScaleX = CounterScale_2_Phase;
	} else if (data->x.counts_per_revolution == 4) {
		qdec_init_struct.counterScaleX = CounterScale_1_Phase;
	} else {
		LOG_ERR("Unspported counts_per_revolution: %d", data->x.counts_per_revolution);
		return -ENOTSUP;
	}

	qdec_init_struct.axisConfigX = ENABLE;
	qdec_init_struct.debounceTimeX = 32 * data->x.debounce_time_ms;
	qdec_init_struct.debounceEnableX = ENABLE;
	qdec_init_struct.initPhaseX = phaseMode0;
#endif

#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	if (data->y.counts_per_revolution == 2) {
		qdec_init_struct.counterScaleY = CounterScale_2_Phase;
	} else if (data->y.counts_per_revolution == 4) {
		qdec_init_struct.counterScaleY = CounterScale_1_Phase;
	} else {
		LOG_ERR("Unspported counts_per_revolution: %d", data->y.counts_per_revolution);
		return -ENOTSUP;
	}

	qdec_init_struct.axisConfigY = ENABLE;
	qdec_init_struct.debounceTimeY = 32 * data->y.debounce_time_ms;
	qdec_init_struct.debounceEnableY = ENABLE;
	qdec_init_struct.initPhaseY = phaseMode0;
#endif

#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	if (data->z.counts_per_revolution == 2) {
		qdec_init_struct.counterScaleZ = CounterScale_2_Phase;
	} else if (data->z.counts_per_revolution == 4) {
		qdec_init_struct.counterScaleZ = CounterScale_1_Phase;
	} else {
		LOG_ERR("Unspported counts_per_revolution: %d", data->z.counts_per_revolution);
		return -ENOTSUP;
	}

	qdec_init_struct.axisConfigZ = ENABLE;
	qdec_init_struct.debounceTimeZ = 32 * data->z.debounce_time_ms;
	qdec_init_struct.debounceEnableZ = ENABLE;
	qdec_init_struct.initPhaseZ = phaseMode0;
#endif

	qdec_init_struct.manualLoadInitPhase = ENABLE;
	QDEC_Init(qdec, &qdec_init_struct);

#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
	QDEC_INTMask(qdec, QDEC_X_CT_INT_MASK, DISABLE);
	QDEC_INTMask(qdec, QDEC_X_ILLEGAL_INT_MASK, DISABLE);
	QDEC_INTConfig(qdec, QDEC_X_INT_NEW_DATA, ENABLE);
	QDEC_INTConfig(qdec, QDEC_X_INT_ILLEGAL, ENABLE);
	QDEC_Cmd(qdec, QDEC_AXIS_X, ENABLE);
#endif

#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
	QDEC_INTMask(qdec, QDEC_Y_CT_INT_MASK, DISABLE);
	QDEC_INTMask(qdec, QDEC_Y_ILLEGAL_INT_MASK, DISABLE);
	QDEC_INTConfig(qdec, QDEC_Y_INT_NEW_DATA, ENABLE);
	QDEC_INTConfig(qdec, QDEC_Y_INT_ILLEGAL, ENABLE);
	QDEC_Cmd(qdec, QDEC_AXIS_Y, ENABLE);
#endif

#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
	QDEC_INTMask(qdec, QDEC_Z_CT_INT_MASK, DISABLE);
	QDEC_INTMask(qdec, QDEC_Z_ILLEGAL_INT_MASK, DISABLE);
	QDEC_INTConfig(qdec, QDEC_Z_INT_NEW_DATA, ENABLE);
	QDEC_INTConfig(qdec, QDEC_Z_INT_ILLEGAL, ENABLE);
	QDEC_Cmd(qdec, QDEC_AXIS_Z, ENABLE);
#endif

	config->irq_connect();

	return ret;
}

#define QDEC_BEE_AXIS_INIT(axis)                                                                   \
	.axis = {                                                                                  \
		.debounce_time_ms = DT_INST_PROP_OR(index, axis##_debounce_time_ms, 0),            \
		.counts_per_revolution = DT_INST_PROP_OR(index, axis##_counts_per_revolution, 4),  \
	}

#if CONFIG_BEE_QDEC_X_AXIS_ENABLE
#define QDEC_BEE_X_AXIS_INIT QDEC_BEE_AXIS_INIT(x),
#else
#define QDEC_BEE_X_AXIS_INIT
#endif

#if CONFIG_BEE_QDEC_Y_AXIS_ENABLE
#define QDEC_BEE_Y_AXIS_INIT QDEC_BEE_AXIS_INIT(y),
#else
#define QDEC_BEE_Y_AXIS_INIT
#endif

#if CONFIG_BEE_QDEC_Z_AXIS_ENABLE
#define QDEC_BEE_Z_AXIS_INIT QDEC_BEE_AXIS_INIT(z),
#else
#define QDEC_BEE_Z_AXIS_INIT
#endif

#define BEE_QDEC_IRQ_HANDLER(index)                                                                \
	static void qdec_bee_irq_connect_##index(void)                                             \
	{                                                                                          \
		RamVectorTableUpdate(Qdecode_VECTORn, qdec_bee_isr);                               \
		NVIC_InitTypeDef NVIC_InitStruct;                                                  \
		NVIC_InitStruct.NVIC_IRQChannel = Qdecode_IRQn;                                    \
		NVIC_InitStruct.NVIC_IRQChannelPriority = 2;                                       \
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;                                       \
		NVIC_Init(&NVIC_InitStruct);                                                       \
	}

#define QDEC_BEE_INIT(index)                                                                       \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	BEE_QDEC_IRQ_HANDLER(index);                                                               \
	static const struct qdec_bee_config qdec##index##_bee_config = {                           \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.irq_connect = qdec_bee_irq_connect_##index,                                       \
	};                                                                                         \
	static struct qdec_bee_data qdec##index##_bee_data = {                                     \
		QDEC_BEE_X_AXIS_INIT QDEC_BEE_Y_AXIS_INIT QDEC_BEE_Z_AXIS_INIT};                   \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(index, qdec_bee_pm_action);                                       \
	SENSOR_DEVICE_DT_INST_DEFINE(index, qdec_bee_init, PM_DEVICE_DT_INST_GET(index),           \
				     &qdec##index##_bee_data, &qdec##index##_bee_config,           \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                     \
				     &qdec_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_BEE_INIT);
