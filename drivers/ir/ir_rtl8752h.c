/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_ir

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/ir.h>
#include <zephyr/drivers/clock_control/rtl8752h_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <rtl876x_ir.h>

#include <zephyr/logging/log.h>

#include <trace.h>
#define DBG_DIRECT_SHOW 1
LOG_MODULE_REGISTER(ir_rtl8752h, CONFIG_IR_LOG_LEVEL);

#define PINCTRL_STATE_IR_TX (PINCTRL_STATE_PRIV_START + 1)
#define PINCTRL_STATE_IR_RX (PINCTRL_STATE_PRIV_START + 2)

struct ir_rtl8752h_config {
	IR_TypeDef *ir;
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

struct ir_rtl8752h_data {
	uint32_t src_clk;
	uint32_t frequency;
	uint8_t duty;
	uint8_t tx_len;
	uint32_t *rx_buf;
	ir_callback_t cb;
	void *cb_usr_data;
#ifdef CONFIG_PM_DEVICE
	IRStoreReg_Typedef store_buf;
#endif
};

static void ir_rtl8752h_reset(const struct device *dev)
{
	const struct ir_rtl8752h_config *config = dev->config;

	(void)clock_control_off(RTL8752H_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);
	(void)clock_control_on(RTL8752H_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);
}

static int ir_rtl8752h_config_tx_pin(const struct device *dev)
{
	const struct ir_rtl8752h_config *config = dev->config;
	int err;

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_IR_TX);
	if (err < 0) {
		return err;
	}
	
	return 0;
}

static int ir_rtl8752h_config_rx_pin(const struct device *dev)
{
	const struct ir_rtl8752h_config *config = dev->config;
	int err;

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_IR_RX);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int ir_rtl8752h_set_freq(const struct device *dev, uint32_t freq, uint8_t duty)
{
	struct ir_rtl8752h_data *data = dev->data;

	if (freq > data->src_clk || freq < 2442) {
		return -ENOTSUP;
	}

	data->frequency = freq;
	data->duty = duty;

	return 0;
}

static int ir_rtl8752h_tx_enable(const struct device *dev, ir_callback_t callback, void *user_data)
{
	const struct ir_rtl8752h_config *config = dev->config;
	struct ir_rtl8752h_data *data = dev->data;
	int err;

	if (callback == NULL) {
		return -EINVAL;
	}

	ir_rtl8752h_reset(dev);
	err = ir_rtl8752h_config_tx_pin(dev);
	if (err < 0) {
		return err;
	}

	data->cb = callback;
	data->cb_usr_data = user_data;

	IR_InitTypeDef IR_InitStruct;
	IR_StructInit(&IR_InitStruct);
	IR_InitStruct.IR_Freq = data->frequency;
	IR_InitStruct.IR_DutyCycle = data->duty;
	IR_InitStruct.IR_Mode = IR_MODE_TX;
	IR_InitStruct.IR_TxInverse = IR_TX_DATA_NORMAL;
	IR_Init(&IR_InitStruct);
	DBG_DIRECT("[%s] frequency%d duty%d line%d", __func__, data->frequency, data->duty, __LINE__);

	IR_Cmd(IR_MODE_TX, ENABLE);

	return 0;
}

static int ir_rtl8752h_tx(const struct device *dev, const uint32_t *buf, size_t len)
{
	const struct ir_rtl8752h_config *config = dev->config;
	struct ir_rtl8752h_data *data = dev->data;

	if (buf == NULL) {
		return -EINVAL;
	}

	if (len > IR_TX_FIFO_SIZE) {
		IR_SendBuf(buf, IR_TX_FIFO_SIZE, DISABLE);
		data->tx_len = IR_TX_FIFO_SIZE;
	} else {
		IR_SendBuf(buf, len, DISABLE);
		data->tx_len = len;
	}

	IR_MaskINTConfig(IR_INT_TF_EMPTY, DISABLE);
	IR_INTConfig(IR_INT_TF_EMPTY, ENABLE);
	// IR_MaskINTConfig(IR_INT_TX_FINISH, DISABLE);
	// IR_INTConfig(IR_INT_TX_FINISH, ENABLE);
	DBG_DIRECT("[%s] buf0x%x len%d line%d", __func__, buf, len, __LINE__);

	return 0;
}

// static void ir_rtl8752h_set_tx_cb(const struct device *dev, ir_tx_callback_t callback, void
// *user_data);
// {
// 	struct ir_rtl8752h_data *data = dev->data;

// }

static int ir_rtl8752h_rx_enable(const struct device *dev, ir_callback_t callback,
				 void *user_data, uint32_t rx_len, uint32_t *buf, uint32_t idle_cnt)
{
	const struct ir_rtl8752h_config *config = dev->config;
	struct ir_rtl8752h_data *data = dev->data;
	int err;

	if (callback == NULL || buf == NULL) {
		return -EINVAL;
	}

	ir_rtl8752h_reset(dev);
	err = ir_rtl8752h_config_rx_pin(dev);
	if (err < 0) {
		return err;
	}

	data->cb = callback;
	data->cb_usr_data = user_data;
	data->rx_buf = buf;

	IR_InitTypeDef IR_InitStruct;
	IR_StructInit(&IR_InitStruct);
	IR_InitStruct.IR_Freq = data->frequency;
	IR_InitStruct.IR_DutyCycle = data->duty;
	IR_InitStruct.IR_Mode = IR_MODE_RX;
	IR_InitStruct.IR_RxStartMode = IR_RX_AUTO_MODE;
	IR_InitStruct.IR_RxFIFOThrLevel = rx_len;
	IR_InitStruct.IR_RxFIFOFullCtrl = IR_RX_FIFO_FULL_DISCARD_NEWEST;
#if (IR_LEARN_TRIG_MODE == IR_LEARN_TRIG_RISING_EDGE)
	IR_InitStruct.IR_RxTriggerMode = IR_RX_RISING_EDGE;
#elif (IR_LEARN_TRIG_MODE == IR_LEARN_TRIG_FALL_EDGE)
	IR_InitStruct.IR_RxTriggerMode = IR_RX_FALL_EDGE;
#endif
	IR_InitStruct.IR_RxFilterTime = IR_RX_FILTER_TIME_200ns;
#if (IR_LEARN_TRIG_MODE == IR_LEARN_TRIG_RISING_EDGE)
	IR_InitStruct.IR_RxCntThrType = IR_RX_Count_Low_Level;
#elif (IR_LEARN_TRIG_MODE == IR_LEARN_TRIG_FALL_EDGE)
	IR_InitStruct.IR_RxCntThrType = IR_RX_Count_High_Level;
#endif
	IR_InitStruct.IR_RxCntThr = idle_cnt;

	IR_Init(&IR_InitStruct);

	IR_ClearRxFIFO();

	IR_INTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, ENABLE);
	IR_MaskINTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, DISABLE);

	IR_Cmd(IR_MODE_RX, ENABLE);

	return 0;
}

static int ir_rtl8752h_rx_disable(const struct device *dev, struct ir_event_rx *rx_data)
{
	const struct ir_rtl8752h_config *config = dev->config;
	struct ir_rtl8752h_data *data = dev->data;

	IR_INTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, DISABLE);
	IR_MaskINTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, ENABLE);

	rx_data->len = IR_GetRxDataLen();
	rx_data->buf = data->rx_buf;
	IR_ReceiveBuf(rx_data->buf, rx_data->len);

	IR_Cmd(IR_MODE_RX, DISABLE);

	return 0;
}

// static void ir_rtl8752h_set_rx_cb(const struct device *dev, ir_rx_callback_t callback, void
// *user_data);
// {
// 	struct ir_rtl8752h_data *data = dev->data;

// 	data->cb = callback;
// 	data->cb_usr_data = user_data;
// }

static void ir_rtl8752h_isr(const struct device *dev)
{
	const struct ir_rtl8752h_config *config = dev->config;
	struct ir_rtl8752h_data *data = dev->data;
	uint8_t tx_len = data->tx_len;
	struct ir_event evt;

	if (IR_GetINTStatus(IR_INT_TF_EMPTY)) {
		IR_MaskINTConfig(IR_INT_TF_EMPTY, ENABLE);
		IR_INTConfig(IR_INT_TF_EMPTY, DISABLE);
		IR_ClearINTPendingBit(IR_INT_TF_EMPTY_CLR);
		data->tx_len = 0;
		evt.type = IR_TX_COMPLETED;
		evt.data.tx.len = tx_len;
		if (data->cb) {
			data->cb(dev, &evt, data->cb_usr_data);
		}
	}

	if (IR_GetINTStatus(IR_INT_TX_FINISH)) {
		IR_MaskINTConfig(IR_INT_TX_FINISH, ENABLE);
		IR_INTConfig(IR_INT_TX_FINISH, DISABLE);
		IR_ClearINTPendingBit(IR_INT_TX_FINISH_CLR);
		data->tx_len = 0;
		evt.type = IR_TX_COMPLETED;
		evt.data.tx.len = tx_len;
		if (data->cb) {
			data->cb(dev, &evt, data->cb_usr_data);
		}
	}

	if (IR_GetINTStatus(IR_INT_RF_LEVEL)) {
		IR_MaskINTConfig(IR_INT_RF_LEVEL, ENABLE);
		IR_INTConfig(IR_INT_RF_LEVEL, DISABLE);
		IR_ClearINTPendingBit(IR_INT_RF_LEVEL_CLR);
		evt.type = IR_RX_RECEIVED;
		evt.data.rx.len = IR_GetRxDataLen();
		evt.data.rx.buf = data->rx_buf;
		IR_ReceiveBuf(evt.data.rx.buf, evt.data.rx.len);
		if (data->cb) {
			data->cb(dev, &evt, data->cb_usr_data);
		}
	}

	if (IR_GetINTStatus(IR_INT_RX_CNT_THR)) {
		IR_MaskINTConfig(IR_INT_RX_CNT_THR, ENABLE);
		IR_INTConfig(IR_INT_RX_CNT_THR, DISABLE);
		IR_ClearINTPendingBit(IR_INT_RX_CNT_THR_CLR);
		evt.type = IR_RX_STOPPED;
		evt.data.rx.len = IR_GetRxDataLen();
		evt.data.rx.buf = data->rx_buf;
		IR_ReceiveBuf(evt.data.rx.buf, evt.data.rx.len);
		if (data->cb) {
			data->cb(dev, &evt, data->cb_usr_data);
		}
	}

	DBG_DIRECT("[%s] line%d", __func__, __LINE__);
}

static const struct ir_driver_api ir_rtl8752h_driver_api = {
	.set_freq = ir_rtl8752h_set_freq,
	.tx_enable = ir_rtl8752h_tx_enable,
	.tx = ir_rtl8752h_tx,
	// .set_tx_cb = ir_rtl8752h_set_tx_cb,
	.rx_enable = ir_rtl8752h_rx_enable,
	.rx_disable = ir_rtl8752h_rx_disable,
	// .set_rx_cb = ir_rtl8752h_set_rx_cb,
};

static int ir_rtl8752h_init(const struct device *dev)
{
	DBG_DIRECT("[%s] line%d", __func__, __LINE__);
	const struct ir_rtl8752h_config *config = dev->config;
	struct ir_rtl8752h_data *data = dev->data;

	config->irq_config_func(dev);

	return 0;
}

#define RTL8752H_IR_IRQ_HANDLER(index)                                                             \
	static void ir_rtl8752h_irq_config_func_##index(const struct device *dev)                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), ir_rtl8752h_isr,    \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

#define RTL8752H_IR_IRQ_HANDLER_FUNC(index) .irq_config_func = ir_rtl8752h_irq_config_func_##index,

#define RTL8752H_IR_INIT(index)                                                                    \
	RTL8752H_IR_IRQ_HANDLER(index)                                                             \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct ir_rtl8752h_config ir_rtl8752h_cfg_##index = {                         \
		.ir = (IR_TypeDef *)DT_INST_REG_ADDR(index),                                       \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		RTL8752H_IR_IRQ_HANDLER_FUNC(index)};                                              \
                                                                                                   \
	static struct ir_rtl8752h_data ir_rtl8752h_data_##index = {                                \
		.src_clk = 40000000,                                                               \
	};  \
PM_DEVICE_DT_INST_DEFINE(index, ir_rtl8752h_pm_action);  \
DEVICE_DT_INST_DEFINE(index, &ir_rtl8752h_init, PM_DEVICE_DT_INST_GET(index),  \
		      &ir_rtl8752h_data_##index, &ir_rtl8752h_cfg_##index, POST_KERNEL,  \
		      CONFIG_IR_INIT_PRIORITY, &ir_rtl8752h_driver_api);  \

DT_INST_FOREACH_STATUS_OKAY(RTL8752H_IR_INIT)
