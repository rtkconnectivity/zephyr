/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_ir

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/ir.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_bee.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <rtl876x_ir.h>

#include <zephyr/logging/log.h>

#include <trace.h>
#define DBG_DIRECT_SHOW 0
LOG_MODULE_REGISTER(ir_bee, CONFIG_IR_LOG_LEVEL);

#define PINCTRL_STATE_IR_TX (PINCTRL_STATE_PRIV_START + 1)
#define PINCTRL_STATE_IR_RX (PINCTRL_STATE_PRIV_START + 2)

#define IR_HAS_TX_DMA DT_DMAS_HAS_NAME(DT_NODELABEL(ir), tx)
#define IR_HAS_RX_DMA DT_DMAS_HAS_NAME(DT_NODELABEL(ir), rx)

#if (IR_HAS_TX_DMA)
struct tx_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
	uint8_t src_addr_increment;
	uint8_t dst_addr_increment;
};
#endif

#if (IR_HAS_RX_DMA)
struct rx_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg[2];
	uint8_t src_addr_increment;
	uint8_t dst_addr_increment;
};
#endif

struct ir_bee_config {
	IR_TypeDef *ir;
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

struct ir_bee_data {
	uint32_t src_clk;
	uint32_t frequency;
	uint8_t duty;
	uint8_t tx_len;
	uint32_t *rx_buf[2];
	uint8_t rx_buf_index;
	uint32_t rx_len;
#if !(IR_HAS_RX_DMA)
	uint32_t cur_rx_len;
#endif
	uint32_t rx_idle_cnt;
	ir_callback_t cb;
	void *cb_usr_data;
	bool falling_trig;
	bool is_tx_mode;
#if IR_HAS_TX_DMA
	struct tx_stream dma_tx;
#endif
#if IR_HAS_RX_DMA
	struct rx_stream dma_rx;
#endif
#ifdef CONFIG_PM_DEVICE
	IRStoreReg_Typedef store_buf;
#endif
};

#if IR_HAS_TX_DMA
static void ir_bee_dma_tx_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
			     int status)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] line%d", __func__, __LINE__);
#endif
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(user_data);
	ARG_UNUSED(channel);
	ARG_UNUSED(status);
}
#endif

#if IR_HAS_RX_DMA
static void ir_bee_dma_rx_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
			     int status)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] line%d", __func__, __LINE__);
#endif
	struct device *dev = (struct device *)user_data;
	const struct ir_bee_config *cfg = dev->config;
	struct ir_bee_data *data = dev->data;
	struct ir_event evt;

	if (data->rx_buf_index == 0) {
		evt.data.rx.buf = data->rx_buf[0];
		data->rx_buf_index = 1;
	} else {
		evt.data.rx.buf = data->rx_buf[1];
		data->rx_buf_index = 0;
	}

	evt.type = IR_RX_RECEIVED;
	evt.data.rx.len = data->rx_len;

	if (data->cb) {
		data->cb(dev, &evt, data->cb_usr_data);
	}
}
#endif

static void ir_bee_reset(const struct device *dev)
{
	const struct ir_bee_config *config = dev->config;

	(void)clock_control_off(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);
	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);
}

static int ir_bee_config_tx_pin(const struct device *dev)
{
	const struct ir_bee_config *config = dev->config;
	int err;

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_IR_TX);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int ir_bee_config_rx_pin(const struct device *dev)
{
	const struct ir_bee_config *config = dev->config;
	int err;

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_IR_RX);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int ir_bee_set_freq(const struct device *dev, uint32_t freq, uint8_t duty)
{
	struct ir_bee_data *data = dev->data;

	if (freq > data->src_clk || freq < 2442) {
		return -ENOTSUP;
	}

	data->frequency = freq;
	data->duty = duty;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] frequency%d duty%d line%d", __func__, data->frequency, data->duty,
		   __LINE__);
#endif

	return 0;
}

static int ir_bee_tx_init(const struct device *dev)
{
	struct ir_bee_data *data = dev->data;
	int err;

	err = ir_bee_config_tx_pin(dev);
	if (err < 0) {
		return err;
	}

	ir_bee_reset(dev);

	IR_InitTypeDef IR_InitStruct;

	IR_StructInit(&IR_InitStruct);
	IR_InitStruct.IR_Freq = data->frequency;
	IR_InitStruct.IR_DutyCycle = data->duty;
	IR_InitStruct.IR_Mode = IR_MODE_TX;
	IR_InitStruct.IR_TxInverse = IR_TX_DATA_NORMAL;
#if IR_HAS_TX_DMA
	IR_InitStruct.IR_TxDmaEn = ENABLE;
	IR_InitStruct.IR_TxWaterLevel = IR_TX_FIFO_SIZE - data->dma_tx.dma_cfg.dest_burst_length;
#else
	IR_InitStruct.IR_TxDmaEn = DISABLE;
#endif
	IR_Init(&IR_InitStruct);

	IR_Cmd(IR_MODE_TX, DISABLE);

	return 0;
}

static int ir_bee_tx_enable(const struct device *dev, ir_callback_t callback, void *user_data)
{
	struct ir_bee_data *data = dev->data;
	int err;

	if (callback == NULL) {
		return -EINVAL;
	}

	err = ir_bee_tx_init(dev);
	if (err < 0) {
		return err;
	}

	data->cb = callback;
	data->cb_usr_data = user_data;
	data->is_tx_mode = true;

	return 0;
}

static int ir_bee_tx(const struct device *dev, const uint32_t *buf, size_t len)
{
	struct ir_bee_data *data = dev->data;
	int err;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] buf0x%x len%d line%d", __func__, buf, len, __LINE__);
#endif

	if (buf == NULL) {
		return -EINVAL;
	}

	err = ir_bee_tx_init(dev);
	if (err < 0) {
		return err;
	}

#if IR_HAS_TX_DMA
	data->tx_len = len;
	data->dma_tx.blk_cfg.source_address = (uint32_t)(buf);
	data->dma_tx.blk_cfg.block_size = (len + 1) * data->dma_tx.dma_cfg.source_data_size;

	if (dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &data->dma_tx.dma_cfg)) {
		LOG_ERR("dma tx config error!");
		return -EINVAL;
	}

	IR_Cmd(IR_MODE_TX, ENABLE);

	IR_MaskINTConfig(IR_INT_TX_FINISH, DISABLE);
	IR_INTConfig(IR_INT_TX_FINISH, ENABLE);

	if (dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel)) {
		LOG_ERR("dma tx start error!");
		return -EFAULT;
	}
#else
	IR_MaskINTConfig(IR_INT_TX_FINISH, DISABLE);
	IR_INTConfig(IR_INT_TX_FINISH, ENABLE);

	if (len > IR_TX_FIFO_SIZE) {
		IR_SendBuf(buf, IR_TX_FIFO_SIZE, ENABLE);
		data->tx_len = IR_TX_FIFO_SIZE;
	} else {
		IR_SendBuf(buf, len, ENABLE);
		data->tx_len = len;
	}

	IR_Cmd(IR_MODE_TX, ENABLE);

#endif

	return 0;
}

static int ir_bee_rx_init(const struct device *dev)
{
	struct ir_bee_data *data = dev->data;
	int err;

	err = ir_bee_config_rx_pin(dev);
	if (err < 0) {
		return err;
	}

	ir_bee_reset(dev);

	IR_InitTypeDef IR_InitStruct;

	IR_StructInit(&IR_InitStruct);
	IR_InitStruct.IR_Freq = data->frequency;
	IR_InitStruct.IR_DutyCycle = data->duty;
	IR_InitStruct.IR_Mode = IR_MODE_RX;
	IR_InitStruct.IR_RxStartMode = IR_RX_AUTO_MODE;
#if !IR_HAS_RX_DMA
	if (data->rx_len <= IR_RX_FIFO_SIZE) {
		IR_InitStruct.IR_RxFIFOThrLevel = data->rx_len - 1;
	} else {
		IR_InitStruct.IR_RxFIFOThrLevel = 15;
	}
#else
	IR_InitStruct.IR_RxFIFOThrLevel = 20;
#endif
	IR_InitStruct.IR_RxFIFOFullCtrl = IR_RX_FIFO_FULL_DISCARD_NEWEST;
	IR_InitStruct.IR_RxTriggerMode = data->falling_trig ? IR_RX_FALL_EDGE : IR_RX_RISING_EDGE;
	IR_InitStruct.IR_RxFilterTime = IR_RX_FILTER_TIME_200ns;
	IR_InitStruct.IR_RxCntThrType =
		data->falling_trig ? IR_RX_Count_High_Level : IR_RX_Count_Low_Level;
	IR_InitStruct.IR_RxCntThr = data->rx_idle_cnt;
#if IR_HAS_RX_DMA
	IR_InitStruct.IR_RxDmaEn = ENABLE;
	IR_InitStruct.IR_RxWaterLevel = data->dma_rx.dma_cfg.source_burst_length;
#else
	IR_InitStruct.IR_RxDmaEn = DISABLE;
#endif

	IR_Init(&IR_InitStruct);

	IR_ClearRxFIFO();
	IR_Cmd(IR_MODE_RX, ENABLE);

	return 0;
}

static int ir_bee_rx_enable(const struct device *dev, ir_callback_t callback, void *user_data,
			    uint32_t rx_len, uint32_t idle_cnt)
{
	struct ir_bee_data *data = dev->data;
	int err;

	if (callback == NULL) {
		return -EINVAL;
	}

	if (data->rx_len != rx_len) {
		if (data->rx_buf[0] != NULL) {
			k_free(data->rx_buf[0]);
#if IR_HAS_RX_DMA
			k_free(data->rx_buf[1]);
#endif
		}

		data->rx_buf[0] = k_malloc(rx_len * 4);
		if (data->rx_buf[0] == NULL) {
			LOG_ERR("[%s] buf malloc fail! line%d\n", __func__, __LINE__);
			return -EIO;
		}

#if IR_HAS_RX_DMA
		data->rx_buf[1] = k_malloc(rx_len * 4);
		if (data->rx_buf[1] == NULL) {
			k_free(data->rx_buf[0]);
			LOG_ERR("[%s] buf malloc fail! line%d\n", __func__, __LINE__);
			return -EIO;
		}
#endif
	}

	data->cb = callback;
	data->cb_usr_data = user_data;
	data->rx_len = rx_len;
#if !(IR_HAS_RX_DMA)
	data->cur_rx_len = 0;
#endif
	data->rx_idle_cnt = idle_cnt;
	data->is_tx_mode = false;

	err = ir_bee_rx_init(dev);
	if (err < 0) {
		return err;
	}

#if !IR_HAS_RX_DMA
	IR_INTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, ENABLE);
	IR_MaskINTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, DISABLE);
#else
	IR_INTConfig(IR_INT_RX_CNT_THR, ENABLE);
	IR_MaskINTConfig(IR_INT_RX_CNT_THR, DISABLE);
#endif

#if IR_HAS_RX_DMA
	data->rx_buf_index = 0;
	data->dma_rx.blk_cfg[0].block_size = rx_len * data->dma_rx.dma_cfg.source_data_size;
	data->dma_rx.blk_cfg[0].dest_address = (uint32_t)data->rx_buf[0];
	data->dma_rx.blk_cfg[1].block_size = rx_len * data->dma_rx.dma_cfg.source_data_size;
	data->dma_rx.blk_cfg[1].dest_address = (uint32_t)data->rx_buf[1];

	if (dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &data->dma_rx.dma_cfg)) {
		LOG_ERR("dma rx config error!");
		return -EINVAL;
	}

	if (dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel)) {
		LOG_ERR("dma rx start error!");
		return -EFAULT;
	}

#endif

	return 0;
}

static int ir_bee_rx_disable(const struct device *dev, struct ir_event_rx *rx_data)
{
	struct ir_bee_data *data = dev->data;

	IR_INTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, DISABLE);
	IR_MaskINTConfig(IR_INT_RF_LEVEL | IR_INT_RX_CNT_THR, ENABLE);

#if IR_HAS_RX_DMA
	struct dma_status stat;
	uint8_t remain_rx_len;

	dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat);
	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	if (rx_data) {
		if (data->rx_buf_index == 0) {
			rx_data->buf = data->rx_buf[0];
		} else {
			rx_data->buf = data->rx_buf[1];
		}

		rx_data->len = data->rx_len - stat.pending_length / 4;

		remain_rx_len = IR_GetRxDataLen();
		IR_ReceiveBuf(&rx_data->buf[rx_data->len], remain_rx_len);
		rx_data->len += remain_rx_len;

	} else {
		IR_ClearRxFIFO();
	}

	IR_Cmd(IR_MODE_RX, DISABLE);
#else
	if (rx_data) {
		rx_data->len = IR_GetRxDataLen();
		rx_data->buf = data->rx_buf[0];
		IR_ReceiveBuf(rx_data->buf, rx_data->len);
	} else {
		IR_ClearRxFIFO();
	}

	IR_Cmd(IR_MODE_RX, DISABLE);
#endif

	return 0;
}

static void ir_bee_isr(const struct device *dev)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] line%d", __func__, __LINE__);
#endif
	struct ir_bee_data *data = dev->data;
	uint8_t tx_len = data->tx_len;
	struct ir_event evt;
	uint8_t rx_len;

	memset(&evt, 0, sizeof(evt));

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

#if !IR_HAS_RX_DMA
	if (IR_GetINTStatus(IR_INT_RF_LEVEL)) {
		IR_ClearINTPendingBit(IR_INT_RF_LEVEL_CLR);

		rx_len = IR_GetRxDataLen();

		if (data->cur_rx_len <= data->rx_len - rx_len) {
			IR_ReceiveBuf(&data->rx_buf[0][data->cur_rx_len], rx_len);
			data->cur_rx_len += rx_len;
		} else {
			IR_ReceiveBuf(&data->rx_buf[0][data->cur_rx_len],
				      data->rx_len - data->cur_rx_len);
			data->cur_rx_len = data->rx_len;
		}

		if (data->cur_rx_len == data->rx_len) {
			evt.type = IR_RX_RECEIVED;
			evt.data.rx.len = data->rx_len;
			evt.data.rx.buf = data->rx_buf[0];
			if (data->cb) {
				data->cb(dev, &evt, data->cb_usr_data);
			}
			data->cur_rx_len = 0;
		}
	}
#endif

	if (IR_GetINTStatus(IR_INT_RX_CNT_THR)) {
		IR_ClearINTPendingBit(IR_INT_RX_CNT_THR_CLR);
#if IR_HAS_RX_DMA
		struct dma_status stat;
		uint8_t remain_rx_len;

		dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat);

		if (data->rx_buf_index == 0) {
			evt.data.rx.buf = data->rx_buf[0];
		} else {
			evt.data.rx.buf = data->rx_buf[1];
		}

		evt.type = IR_RX_STOPPED;
		evt.data.rx.len = data->rx_len - stat.pending_length / 4;
		remain_rx_len = IR_GetRxDataLen();
		IR_ReceiveBuf(&evt.data.rx.buf[evt.data.rx.len], remain_rx_len);
		evt.data.rx.len += remain_rx_len;

		if (data->cb) {
			data->cb(dev, &evt, data->cb_usr_data);
		}

#else

		rx_len = IR_GetRxDataLen();

		/* set rx 256 bytes, actual rx 257 bytes, rcv_evt 256bytes + stop_evt 1 bytes;
		 * set rx 256 bytes, actual rx 256 bytes, stop_evt 256 bytes;
		 * set rx 256 bytes, actual rx 255 bytes, stop_evt 255 bytes
		 */
		if (data->cur_rx_len <= data->rx_len - rx_len) {
			IR_ReceiveBuf(&data->rx_buf[0][data->cur_rx_len], rx_len);
			data->cur_rx_len += rx_len;
		} else {
			IR_ReceiveBuf(&data->rx_buf[0][data->cur_rx_len],
				      data->rx_len - data->cur_rx_len);
			data->cur_rx_len = data->rx_len;
			evt.type = IR_RX_RECEIVED;
			evt.data.rx.len = data->cur_rx_len;
			evt.data.rx.buf = data->rx_buf[0];
			if (data->cb) {
				data->cb(dev, &evt, data->cb_usr_data);
			}
			data->cur_rx_len = 0;

			rx_len = IR_GetRxDataLen();
			IR_ReceiveBuf(&data->rx_buf[0][data->cur_rx_len], rx_len);
			data->cur_rx_len += rx_len;
		}

		evt.type = IR_RX_STOPPED;
		evt.data.rx.len = data->cur_rx_len;
		evt.data.rx.buf = data->rx_buf[0];

		if (data->cb) {
			data->cb(dev, &evt, data->cb_usr_data);
		}

		data->cur_rx_len = 0;

#endif
	}
}

#ifdef CONFIG_PM_DEVICE
static int ir_bee_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct ir_bee_config *config = dev->config;
	struct ir_bee_data *data = dev->data;
	IR_TypeDef *ir = config->ir;
	int err;

	extern void IR_DLPSEnter(void *PeriReg, void *StoreBuf);
	extern void IR_DLPSExit(void *PeriReg, void *StoreBuf);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:

		IR_DLPSEnter(ir, &data->store_buf);

		/* Move pins to sleep state */
		err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
		if ((err < 0) && (err != -ENOENT)) {
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		if (data->is_tx_mode) {
			err = ir_bee_config_tx_pin(dev);

		} else {
			err = ir_bee_config_rx_pin(dev);
		}

		if (err < 0) {
			return err;
		}

		IR_DLPSExit(ir, &data->store_buf);

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct ir_driver_api ir_bee_driver_api = {
	.set_freq = ir_bee_set_freq,
	.tx_enable = ir_bee_tx_enable,
	.tx = ir_bee_tx,
	.rx_enable = ir_bee_rx_enable,
	.rx_disable = ir_bee_rx_disable,
};

static int ir_bee_init(const struct device *dev)
{
	const struct ir_bee_config *config = dev->config;
	struct ir_bee_data *data = dev->data;
	IR_TypeDef *ir = config->ir;

	config->irq_config_func(dev);

#if IR_HAS_TX_DMA
	atomic_set_bit(((struct dma_context *)data->dma_tx.dma_dev->data)->atomic,
		       data->dma_tx.dma_channel);

	memset(&data->dma_tx.blk_cfg, 0, sizeof(data->dma_tx.blk_cfg));

	data->dma_tx.blk_cfg.dest_address = (uint32_t)(&(ir->TX_FIFO));
	data->dma_tx.blk_cfg.source_address = 0; /* not ready */
	data->dma_tx.blk_cfg.source_addr_adj = data->dma_tx.src_addr_increment;
	data->dma_tx.blk_cfg.dest_addr_adj = data->dma_tx.dst_addr_increment;

	data->dma_tx.dma_cfg.block_count = 1;
	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	data->dma_tx.dma_cfg.user_data = (void *)dev;
#endif

#if IR_HAS_RX_DMA
	atomic_set_bit(((struct dma_context *)data->dma_rx.dma_dev->data)->atomic,
		       data->dma_rx.dma_channel);

	memset(&data->dma_rx.blk_cfg[0], 0, sizeof(data->dma_rx.blk_cfg[0]));
	memset(&data->dma_rx.blk_cfg[1], 0, sizeof(data->dma_rx.blk_cfg[1]));

	data->dma_rx.blk_cfg[0].dest_address = 0; /* not ready */
	data->dma_rx.blk_cfg[0].source_address = (uint32_t)(&(ir->RX_FIFO));
	data->dma_rx.blk_cfg[0].source_addr_adj = data->dma_rx.src_addr_increment;
	data->dma_rx.blk_cfg[0].dest_addr_adj = data->dma_rx.dst_addr_increment;
	data->dma_rx.blk_cfg[0].next_block = &(data->dma_rx.blk_cfg[1]);
	data->dma_rx.blk_cfg[1].dest_address = 0; /* not ready */
	data->dma_rx.blk_cfg[1].source_address = (uint32_t)(&(ir->RX_FIFO));
	data->dma_rx.blk_cfg[1].source_addr_adj = data->dma_rx.src_addr_increment;
	data->dma_rx.blk_cfg[1].dest_addr_adj = data->dma_rx.dst_addr_increment;
	data->dma_rx.blk_cfg[1].next_block = NULL;

	data->dma_rx.dma_cfg.block_count = 2;
	data->dma_rx.dma_cfg.cyclic = true;
	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg[0];
	data->dma_rx.dma_cfg.user_data = (void *)dev;
#endif

	return 0;
}

#define BEE_IR_IRQ_HANDLER(index)                                                                  \
	static void ir_bee_irq_config_func_##index(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), ir_bee_isr,         \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

#define BEE_IR_IRQ_HANDLER_FUNC(index) .irq_config_func = ir_bee_irq_config_func_##index,

#define IR_DMA_CHANNEL_INIT(index, dir)                                                            \
	.dma_dev = DEVICE_DT_GET(BEE_DMA_CTLR(index, dir)),                                        \
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                             \
	.dma_cfg =                                                                                 \
		{                                                                                  \
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),                   \
			.channel_direction =                                                       \
				BEE_DMA_CONFIG_DIRECTION(BEE_DMA_CHANNEL_CONFIG(index, dir)),      \
			.channel_priority =                                                        \
				BEE_DMA_CONFIG_PRIORITY(BEE_DMA_CHANNEL_CONFIG(index, dir)),       \
			.source_data_size = BEE_DMA_CONFIG_SOURCE_DATA_SIZE(                       \
				BEE_DMA_CHANNEL_CONFIG(index, dir)),                               \
			.dest_data_size = BEE_DMA_CONFIG_DESTINATION_DATA_SIZE(                    \
				BEE_DMA_CHANNEL_CONFIG(index, dir)),                               \
			.source_burst_length =                                                     \
				BEE_DMA_CONFIG_SOURCE_MSIZE(BEE_DMA_CHANNEL_CONFIG(index, dir)),   \
			.dest_burst_length = BEE_DMA_CONFIG_DESTINATION_MSIZE(                     \
				BEE_DMA_CHANNEL_CONFIG(index, dir)),                               \
			.complete_callback_en = true,                                              \
			.dma_callback = ir_bee_dma_##dir##_cb,                                     \
	},                                                                                         \
	.src_addr_increment = BEE_DMA_CONFIG_SOURCE_ADDR_INC(BEE_DMA_CHANNEL_CONFIG(index, dir)),  \
	.dst_addr_increment =                                                                      \
		BEE_DMA_CONFIG_DESTINATION_ADDR_INC(BEE_DMA_CHANNEL_CONFIG(index, dir)),

#define IR_DMA_CHANNEL(index, dir)                                                                 \
	.dma_##dir = {COND_CODE_1(DT_INST_DMAS_HAS_NAME(index, dir),                               \
				  (IR_DMA_CHANNEL_INIT(index, dir)), (NULL))},

#if (IR_HAS_TX_DMA && IR_HAS_RX_DMA)
#define IR_DMA_INIT(index) IR_DMA_CHANNEL(index, tx) IR_DMA_CHANNEL(index, rx)
#elif IR_HAS_TX_DMA
#define IR_DMA_INIT(index) IR_DMA_CHANNEL(index, tx)
#elif IR_HAS_RX_DMA
#define IR_DMA_INIT(index) IR_DMA_CHANNEL(index, rx)
#else
#define IR_DMA_INIT(index)
#endif

#define BEE_IR_INIT(index)                                                                         \
	BEE_IR_IRQ_HANDLER(index)                                                                  \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct ir_bee_config ir_bee_cfg_##index = {                                   \
		.ir = (IR_TypeDef *)DT_INST_REG_ADDR(index),                                       \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		BEE_IR_IRQ_HANDLER_FUNC(index)};                                                   \
                                                                                                   \
	static struct ir_bee_data ir_bee_data_##index = {                                          \
		.src_clk = 40000000,                                                               \
		.falling_trig = DT_INST_PROP_OR(index, rx_falling_edge_trig, 0),                   \
		IR_DMA_INIT(index)};                                                               \
	PM_DEVICE_DT_INST_DEFINE(index, ir_bee_pm_action);                                         \
	DEVICE_DT_INST_DEFINE(index, &ir_bee_init, PM_DEVICE_DT_INST_GET(index),                   \
			      &ir_bee_data_##index, &ir_bee_cfg_##index, POST_KERNEL,              \
			      CONFIG_IR_INIT_PRIORITY, &ir_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BEE_IR_INIT)
