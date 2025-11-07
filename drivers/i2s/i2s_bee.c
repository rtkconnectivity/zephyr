/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for I2S port on BEE family processor.
 * @note  Please validate for newly added series.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <soc.h>

#include <zephyr/drivers/dma/dma_bee.h>
#include <zephyr/drivers/dma.h>
#include <rtl876x_gdma.h>
#include <rtl876x_i2s.h>
#include <rtl876x_pinmux.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include "trace.h"

LOG_MODULE_REGISTER(i2s_bee, CONFIG_I2S_LOG_LEVEL);

#define DT_DRV_COMPAT realtek_bee_i2s

#define I2S_SRC_CLK 40000000

#if !defined(CONFIG_I2S_BEE_RX) && !defined(CONFIG_I2S_BEE_TX)
BUILD_ASSERT(false, "I2S RX or TX should be enabled!");
#endif

/*
 * I2S driver uses multi block feature of DMA, and relies on DMA driver
 * managing circular list of DMA blocks.
 *
 * This indicates the Tx/Rx stream.
 *
 * in_queue and out_queue are used as follows
 *   transmit stream:
 *   application provided buffer is queued to in_queue until loaded to DMA.
 *   when DMA channel is idle, buffer is retrieved from in_queue and loaded
 *   to DMA and queued to out_queue. when DMA completes, buffer is retrieved
 *   from out_queue and freed.
 *
 *   receive stream:
 *   driver allocates buffer from slab and loads DMA buffer is queued to
 *   in_queue when DMA completes, buffer is retrieved from in_queue
 *   and queued to out_queue when application reads, buffer is read
 *   (may optionally block) from out_queue and presented to application.
 */
struct stream {
	int32_t state;
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct i2s_config cfg;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg[2];
	uint8_t src_addr_increment;
	uint8_t dst_addr_increment;
	bool last_block;
	struct k_msgq in_queue;
	struct k_msgq out_queue;
	uint8_t *ping_pong_buf[2];
	uint8_t ping_pong_index;
};

struct i2s_bee_config {
	I2S_TypeDef *base;
	uint32_t clkid;
	const struct pinctrl_dev_config *pinctrl;
	void (*irq_config_func)(const struct device *dev);
};

/* Device run time data */
struct i2s_bee_data {
#if defined(CONFIG_I2S_BEE_TX)
	struct stream dma_tx;
	void *tx_in_msgs[CONFIG_I2S_BEE_TX_BLOCK_COUNT];
	void *tx_out_msgs[CONFIG_I2S_BEE_TX_BLOCK_COUNT];
#endif
#if defined(CONFIG_I2S_BEE_RX)
	struct stream dma_rx;
	void *rx_in_msgs[CONFIG_I2S_BEE_RX_BLOCK_COUNT];
	void *rx_out_msgs[CONFIG_I2S_BEE_RX_BLOCK_COUNT];
#endif
};

#if defined(CONFIG_I2S_BEE_TX)
static void i2s_bee_dma_tx_cb(const struct device *, void *, uint32_t, int);
static void i2s_tx_stream_disable(const struct device *, bool drop);
#endif
#if defined(CONFIG_I2S_BEE_RX)
static void i2s_bee_dma_rx_cb(const struct device *, void *, uint32_t, int);
static void i2s_rx_stream_disable(const struct device *, bool in_drop, bool out_drop);
#endif

static inline void i2s_purge_stream_buffers(struct stream *strm, struct k_mem_slab *mem_slab,
					    bool in_drop, bool out_drop)
{
	void *buffer;

	if (in_drop) {
		while (k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT) == 0) {
			k_mem_slab_free(mem_slab, buffer);
		}
	}

	if (out_drop) {
		while (k_msgq_get(&strm->out_queue, &buffer, K_NO_WAIT) == 0) {
			k_mem_slab_free(mem_slab, buffer);
		}
	}
}
static inline void i2s_set_satus(struct i2s_bee_data *dev_data, enum i2s_dir dir,
				 enum i2s_state state)
{
	if (dir == I2S_DIR_TX) {
#if defined(CONFIG_I2S_BEE_TX)
		dev_data->dma_tx.state = state;
#endif
	} else {
#if defined(CONFIG_I2S_BEE_RX)
		dev_data->dma_rx.state = state;
#endif
	}
}

#if defined(CONFIG_I2S_BEE_TX)
static void i2s_tx_stream_disable(const struct device *dev, bool drop)
{
	const struct i2s_bee_config *dev_cfg = dev->config;
	struct i2s_bee_data *dev_data = dev->data;
	I2S_TypeDef *base = (I2S_TypeDef *)dev_cfg->base;
	struct stream *strm = &dev_data->dma_tx;
	const struct device *dma_dev = strm->dma_dev;

	LOG_DBG("Stopping DMA channel %u for TX stream", strm->dma_channel);

	dma_stop(dma_dev, strm->dma_channel);
	I2S_Cmd(base, I2S_MODE_TX, DISABLE);

	/* purge buffers queued in the stream */
	if (drop) {
		i2s_purge_stream_buffers(strm, dev_data->dma_tx.cfg.mem_slab, true, true);
	}
}

/* This function is executed in the interrupt context */
static void i2s_bee_dma_tx_cb(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	const struct device *dev = (struct device *)arg;
	struct i2s_bee_data *dev_data = dev->data;
	struct stream *strm = &dev_data->dma_tx;
	void *buffer = NULL;
	int ret;

	LOG_DBG("tx cb");

	ret = k_msgq_get(&strm->out_queue, &buffer, K_NO_WAIT);
	if (ret == 0) {
		/* transmission complete. free the buffer */
		k_mem_slab_free(strm->cfg.mem_slab, buffer);
	} else {
		LOG_ERR("no buf in out_queue for channel %u", channel);
	}

	/* Received a STOP trigger, terminate TX immediately */
	if (strm->last_block) {
		strm->state = I2S_STATE_READY;
		LOG_DBG("TX STOPPED last_block set");
		goto disabled_exit_no_drop;
	}

	if (ret) {
		/* k_msgq_get() returned error, and was not last_block */
		strm->state = I2S_STATE_ERROR;
		goto disabled_exit_no_drop;
	}

	switch (strm->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		ret = k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT);
		if (ret) {
			if (strm->state == I2S_STATE_STOPPING) {
				/* TX queue has drained */
				strm->state = I2S_STATE_READY;
				goto disabled_exit_drop;
				LOG_DBG("TX stream has stopped");
			} else {
				strm->state = I2S_STATE_ERROR;
				goto disabled_exit_no_drop;
				LOG_DBG("TX stream no data");
			}
		}

		if (strm->ping_pong_index == 0) {
			memcpy(strm->ping_pong_buf[0], buffer, strm->cfg.block_size);
			strm->ping_pong_index = 1;
		} else {
			memcpy(strm->ping_pong_buf[1], buffer, strm->cfg.block_size);
			strm->ping_pong_index = 0;
		}

		ret = k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);
		goto enabled_exit;

	case I2S_STATE_ERROR:
	default:
		goto disabled_exit_drop;
	}

disabled_exit_no_drop:
	i2s_tx_stream_disable(dev, false);
	return;

disabled_exit_drop:
	i2s_tx_stream_disable(dev, true);
	return;

enabled_exit:
	return;
}

static int i2s_tx_stream_start(const struct device *dev)
{
	const struct i2s_bee_config *dev_cfg = dev->config;
	struct i2s_bee_data *dev_data = dev->data;
	I2S_TypeDef *base = (I2S_TypeDef *)dev_cfg->base;
	int ret = 0;
	void *buffer[2];
	struct stream *strm = &dev_data->dma_tx;
	const struct device *dma_dev = strm->dma_dev;
	struct dma_block_config *blk_cfg = strm->blk_cfg;

	if (blk_cfg[0].block_size != strm->cfg.block_size) {
		if (strm->ping_pong_buf[0] != NULL) {
			k_free(strm->ping_pong_buf[0]);
			k_free(strm->ping_pong_buf[1]);
		}

		strm->ping_pong_buf[0] = k_malloc(strm->cfg.block_size);
		if (strm->ping_pong_buf[0] == NULL) {
			return -EIO;
		}

		strm->ping_pong_buf[1] = k_malloc(strm->cfg.block_size);
		if (strm->ping_pong_buf[1] == NULL) {
			k_free(strm->ping_pong_buf[0]);
			return -EIO;
		}
	}

	/* retrieve buffer from input queue */
	ret = k_msgq_get(&strm->in_queue, &buffer[0], K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("No buffer in input queue to start");
		return -EIO;
	}

	memcpy(strm->ping_pong_buf[0], buffer[0], strm->cfg.block_size);

	/* Configure the DMA with the first TX block */
	blk_cfg[0].source_address = (uint32_t)strm->ping_pong_buf[0];
	blk_cfg[0].block_size = strm->cfg.block_size;
	blk_cfg[1].source_address = (uint32_t)strm->ping_pong_buf[1];
	blk_cfg[1].block_size = strm->cfg.block_size;

	LOG_DBG("tx stream start");
	ret = dma_config(dma_dev, strm->dma_channel, &strm->dma_cfg);
	if (ret) {
		LOG_ERR("Failed to config DMA Ch%d (%d)", strm->dma_channel, ret);
	}

	/* put buffer in output queue */
	ret = k_msgq_put(&strm->out_queue, &buffer[0], K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("failed to put buffer in output queue");
		return ret;
	}

	strm->ping_pong_index = 0;

	I2S_Cmd(base, I2S_MODE_TX, ENABLE);

	ret = dma_start(dma_dev, strm->dma_channel);
	if (ret < 0) {
		LOG_ERR("dma_start failed (%d)", ret);
		return ret;
	}

	return 0;
}

static int i2s_bee_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_bee_data *dev_data = dev->data;
	struct stream *strm = &dev_data->dma_tx;
	int ret;

	LOG_DBG("i2s_bee_write");
	if (strm->state != I2S_STATE_RUNNING && strm->state != I2S_STATE_READY) {
		LOG_ERR("invalid state (%d)", strm->state);
		return -EIO;
	}

	ret = k_msgq_put(&strm->in_queue, &mem_block, SYS_TIMEOUT_MS(strm->cfg.timeout));
	if (ret) {
		LOG_DBG("k_msgq_put returned code %d", ret);
		return ret;
	}

	return ret;
}
#endif

#if defined(CONFIG_I2S_BEE_RX)
static void i2s_rx_stream_disable(const struct device *dev, bool in_drop, bool out_drop)
{
	const struct i2s_bee_config *dev_cfg = dev->config;
	struct i2s_bee_data *dev_data = dev->data;
	I2S_TypeDef *base = (I2S_TypeDef *)dev_cfg->base;
	struct stream *strm = &dev_data->dma_rx;
	const struct device *dma_dev = strm->dma_dev;

	LOG_DBG("Stopping RX stream & DMA channel %u", strm->dma_channel);

	I2S_Cmd(base, I2S_MODE_RX, DISABLE);

	dma_stop(dma_dev, strm->dma_channel);

	/* purge buffers queued in the stream */
	if (in_drop || out_drop) {
		i2s_purge_stream_buffers(strm, dev_data->dma_rx.cfg.mem_slab, in_drop, out_drop);
	}
}

static int i2s_rx_stream_start(const struct device *dev)
{
	const struct i2s_bee_config *dev_cfg = dev->config;
	struct i2s_bee_data *dev_data = dev->data;
	I2S_TypeDef *base = (I2S_TypeDef *)dev_cfg->base;
	int ret = 0;
	void *buffer[2];
	struct stream *strm = &dev_data->dma_rx;
	const struct device *dma_dev = strm->dma_dev;
	uint8_t num_of_bufs;
	struct dma_block_config *blk_cfg = strm->blk_cfg;

	if (blk_cfg[0].block_size != strm->cfg.block_size) {
		if (strm->ping_pong_buf[0] != NULL) {
			k_free(strm->ping_pong_buf[0]);
			k_free(strm->ping_pong_buf[1]);
		}
		strm->ping_pong_buf[0] = k_malloc(strm->cfg.block_size);
		if (strm->ping_pong_buf[0] == NULL) {
			LOG_ERR("[%s] buf malloc fail! line%d\n", __func__, __LINE__);
			return -EIO;
		}
		strm->ping_pong_buf[1] = k_malloc(strm->cfg.block_size);
		if (strm->ping_pong_buf[1] == NULL) {
			k_free(strm->ping_pong_buf[0]);
			LOG_ERR("[%s] buf malloc fail! line%d\n", __func__, __LINE__);
			return -EIO;
		}
	}

	num_of_bufs = k_mem_slab_num_free_get(strm->cfg.mem_slab);

	/*
	 * Need at least 2 buffers on the RX memory slab
	 * for reliable DMA reception.
	 */
	if (num_of_bufs < 2) {
		return -EINVAL;
	}

	/* allocate 1st receive buffer from SLAB */
	ret = k_mem_slab_alloc(strm->cfg.mem_slab, &buffer[0], K_NO_WAIT);
	if (ret) {
		LOG_DBG("buffer alloc from mem_slab failed (%d)", ret);
		return ret;
	}

	/* Configure DMA block */
	blk_cfg[0].dest_address = (uint32_t)strm->ping_pong_buf[0];
	blk_cfg[0].block_size = strm->cfg.block_size;
	blk_cfg[1].dest_address = (uint32_t)strm->ping_pong_buf[1];
	blk_cfg[1].block_size = strm->cfg.block_size;

	ret = dma_config(dma_dev, strm->dma_channel, &strm->dma_cfg);
	if (ret) {
		LOG_ERR("Failed to config DMA Ch%d (%d)", strm->dma_channel, ret);
	}

	/* put buffer in input queue */
	ret = k_msgq_put(&strm->in_queue, &buffer[0], K_NO_WAIT);
	if (ret) {
		LOG_ERR("failed to put buffer in input queue, ret1 %d", ret);
		return ret;
	}

	LOG_DBG("Starting DMA Ch%u", strm->dma_channel);
	strm->ping_pong_index = 0;
	I2S_Cmd(base, I2S_MODE_RX, ENABLE);
	ret = dma_start(dma_dev, strm->dma_channel);

	if (ret) {
		LOG_ERR("Failed to start DMA Ch%d (%d)", strm->dma_channel, ret);
		return ret;
	}

	return 0;
}

static int i2s_bee_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_bee_data *dev_data = dev->data;
	struct stream *strm = &dev_data->dma_rx;
	void *buffer;
	int status, ret = 0;

	if (strm->state == I2S_STATE_NOT_READY) {
		LOG_ERR("invalid state %d", strm->state);
		return -EIO;
	}

	status = k_msgq_get(&strm->out_queue, &buffer, SYS_TIMEOUT_MS(strm->cfg.timeout));
	if (status != 0) {
		if (strm->state == I2S_STATE_ERROR) {
			ret = -EIO;
		} else {
			LOG_DBG("need retry");
			ret = -EAGAIN;
		}
		return ret;
	}

	*mem_block = buffer;
	*size = strm->cfg.block_size;
	return 0;
}

static void i2s_bee_dma_rx_cb(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	struct device *dev = (struct device *)arg;
	struct i2s_bee_data *dev_data = dev->data;
	struct stream *strm = &dev_data->dma_rx;
	void *buffer;
	int ret;

	LOG_DBG("RX cb");

	switch (strm->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		/* retrieve buffer from input queue */
		ret = k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT);
		__ASSERT_NO_MSG(ret == 0);

		if (strm->ping_pong_index == 0) {
			memcpy(buffer, strm->ping_pong_buf[0], strm->cfg.block_size);
			strm->ping_pong_index = 1;
		} else {
			memcpy(buffer, strm->ping_pong_buf[1], strm->cfg.block_size);
			strm->ping_pong_index = 0;
		}

		/* put buffer to output queue */
		ret = k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);

		if (ret != 0) {
			LOG_ERR("buffer %p -> out_queue %p err %d", buffer, &strm->out_queue, ret);
			i2s_rx_stream_disable(dev, false, false);
			strm->state = I2S_STATE_ERROR;
			return;
		}
		if (strm->state == I2S_STATE_RUNNING) {
			/* allocate new buffer for next audio frame */
			ret = k_mem_slab_alloc(strm->cfg.mem_slab, &buffer, K_NO_WAIT);
			if (ret != 0) {
				LOG_ERR("buffer alloc from slab %p err %d", strm->cfg.mem_slab,
					ret);
				i2s_rx_stream_disable(dev, false, false);
				strm->state = I2S_STATE_ERROR;
			} else {
				/* put buffer in input queue */
				ret = k_msgq_put(&strm->in_queue, &buffer, K_NO_WAIT);
				if (ret != 0) {
					LOG_ERR("%p -> in_queue %p err %d", buffer, &strm->in_queue,
						ret);
				}
			}
		} else {
			i2s_rx_stream_disable(dev, true, false);
			/* Received a STOP/DRAIN trigger */
			strm->state = I2S_STATE_READY;
		}
		break;
	case I2S_STATE_ERROR:
		i2s_rx_stream_disable(dev, true, true);
		break;
	}
}
#endif

static int i2s_bee_configure(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *i2s_cfg)
{
	const struct i2s_bee_config *dev_cfg = dev->config;
	struct i2s_bee_data *dev_data = dev->data;
	I2S_TypeDef *base = (I2S_TypeDef *)dev_cfg->base;

	I2S_InitTypeDef I2S_InitStruct;

	if (dir == I2S_DIR_TX) {
#if defined(CONFIG_I2S_BEE_TX)
		if ((dev_data->dma_tx.state != I2S_STATE_NOT_READY) &&
		    (dev_data->dma_tx.state != I2S_STATE_READY)) {
			LOG_ERR("invalid state tx(%u)", dev_data->dma_tx.state);
			return -EINVAL;
		}
		dev_data->dma_tx.state = I2S_STATE_NOT_READY;
#else
		return -EINVAL;
#endif
	} else if (dir == I2S_DIR_RX) {
#if defined(CONFIG_I2S_BEE_RX)
		if ((dev_data->dma_rx.state != I2S_STATE_NOT_READY) &&
		    (dev_data->dma_rx.state != I2S_STATE_READY)) {
			LOG_ERR("invalid state rx(%u)", dev_data->dma_rx.state);
			return -EINVAL;
		}
		dev_data->dma_rx.state = I2S_STATE_NOT_READY;
#else
		return -EINVAL;
#endif
	}

	if (i2s_cfg->frame_clk_freq == 8000) {
		I2S_InitStruct.I2S_BClockMi = 0x186A;
		I2S_InitStruct.I2S_BClockNi = 0x50;
	} else if (i2s_cfg->frame_clk_freq == 16000) {
		I2S_InitStruct.I2S_BClockMi = 0x186A;
		I2S_InitStruct.I2S_BClockNi = 0xA0;
	} else if (i2s_cfg->frame_clk_freq == 0) {
		(void)clock_control_off(BEE_CLOCK_CONTROLLER,
					(clock_control_subsys_t)&dev_cfg->clkid);

		i2s_set_satus(dev_data, dir, I2S_STATE_NOT_READY);
	} else {
		LOG_ERR("invalid i2s sample rate: %d", i2s_cfg->frame_clk_freq);
		i2s_set_satus(dev_data, dir, I2S_STATE_NOT_READY);
		return -EINVAL;
	}

	if (i2s_cfg->word_size == 8) {
		I2S_InitStruct.I2S_DataWidth = I2S_Width_8Bits;
	} else if (i2s_cfg->word_size == 16) {
		I2S_InitStruct.I2S_DataWidth = I2S_Width_16Bits;
	} else if (i2s_cfg->word_size == 24) {
		I2S_InitStruct.I2S_DataWidth = I2S_Width_24Bits;
	} else {
		LOG_ERR("invalid i2s word size: %d", i2s_cfg->word_size);
		i2s_set_satus(dev_data, dir, I2S_STATE_NOT_READY);
		return -EINVAL;
	}

	if (i2s_cfg->channels == 1) {
		I2S_InitStruct.I2S_ChannelType = I2S_Channel_Mono;
	} else if (i2s_cfg->channels == 2) {
		I2S_InitStruct.I2S_ChannelType = I2S_Channel_stereo;
	} else {
		LOG_ERR("invalid i2s channels: %d", i2s_cfg->channels);
		i2s_set_satus(dev_data, dir, I2S_STATE_NOT_READY);
		return -EINVAL;
	}

	if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_I2S) {
		I2S_InitStruct.I2S_DataFormat = I2S_Mode;
	} else if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_PCM_SHORT) {
		I2S_InitStruct.I2S_DataFormat = PCM_Mode_A;
	} else if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_PCM_LONG) {
		I2S_InitStruct.I2S_DataFormat = PCM_Mode_B;
	} else if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) ==
		   I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED) {
		I2S_InitStruct.I2S_DataFormat = Left_Justified_Mode;
	} else {
		LOG_ERR("invalid i2s format: 0x%x", i2s_cfg->format);
		i2s_set_satus(dev_data, dir, I2S_STATE_NOT_READY);
		return -EINVAL;
	}

	if ((i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) == I2S_FMT_DATA_ORDER_LSB) {
		I2S_InitStruct.I2S_TxBitSequence = I2S_TX_LSB_First;
		I2S_InitStruct.I2S_RxBitSequence = I2S_RX_LSB_First;
	} else {
		I2S_InitStruct.I2S_TxBitSequence = I2S_TX_MSB_First;
		I2S_InitStruct.I2S_RxBitSequence = I2S_RX_MSB_First;
	}

	if ((i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) == I2S_OPT_BIT_CLK_SLAVE) {
		I2S_InitStruct.I2S_DeviceMode = I2S_DeviceMode_Slave;
	} else {
		I2S_InitStruct.I2S_DeviceMode = I2S_DeviceMode_Master;
	}

	if ((i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) == I2S_OPT_BIT_CLK_GATED) {
		return -EINVAL;
	}

	if ((i2s_cfg->options & I2S_OPT_LOOPBACK) == I2S_OPT_LOOPBACK) {
		return -EINVAL;
	}

	I2S_InitStruct.I2S_ClockSource = I2S_CLK_40M;
	I2S_InitStruct.I2S_TxChSequence = I2S_TX_CH_L_R;
	I2S_InitStruct.I2S_RxChSequence = I2S_RX_CH_L_R;
	I2S_InitStruct.I2S_MCLKOutput = I2S_MCLK_128fs;
	I2S_InitStruct.I2S_DMACmd = I2S_DMA_ENABLE;
#if defined(CONFIG_I2S_BEE_TX)
	I2S_InitStruct.I2S_TxWaterlevel = 64 - dev_data->dma_tx.dma_cfg.dest_burst_length;
#endif
#if defined(CONFIG_I2S_BEE_RX)
	I2S_InitStruct.I2S_RxWaterlevel = dev_data->dma_rx.dma_cfg.source_burst_length;
#endif

	/* pinctrl */
	pinctrl_apply_state(dev_cfg->pinctrl, PINCTRL_STATE_DEFAULT);

	/* clock */
	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&dev_cfg->clkid);

	I2S_Init(base, &I2S_InitStruct);

	if (dir == I2S_DIR_TX) {
#if defined(CONFIG_I2S_BEE_TX)
		memcpy(&dev_data->dma_tx.cfg, i2s_cfg, sizeof(struct i2s_config));
#endif
	} else {
#if defined(CONFIG_I2S_BEE_RX)
		memcpy(&dev_data->dma_rx.cfg, i2s_cfg, sizeof(struct i2s_config));
#endif
	}

	i2s_set_satus(dev_data, dir, I2S_STATE_READY);

	return 0;
}

static const struct i2s_config *i2s_bee_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_bee_data *dev_data = dev->data;

#if defined(CONFIG_I2S_BEE_TX)
	if (dir == I2S_DIR_TX) {
		return &dev_data->dma_tx.cfg;
	}
#endif

#if defined(CONFIG_I2S_BEE_RX)
	if (dir == I2S_DIR_RX) {
		return &dev_data->dma_rx.cfg;
	}
#endif

	return NULL;
}

static int i2s_bee_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_bee_data *dev_data = dev->data;
	struct stream *strm;
	unsigned int key;
	int ret = 0;

	strm = NULL;

#if defined(CONFIG_I2S_BEE_TX)
	if (dir == I2S_DIR_TX) {
		strm = &dev_data->dma_tx;
	}
#endif

#if defined(CONFIG_I2S_BEE_RX)
	if (dir == I2S_DIR_RX) {
		strm = &dev_data->dma_rx;
	}
#endif

	if (strm == NULL) {
		return -ENOSYS;
	}

	key = irq_lock();
	switch (cmd) {
	case I2S_TRIGGER_START:
		if (strm->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %u", strm->state);
			ret = -EIO;
			break;
		}

		if (dir == I2S_DIR_TX) {
#if defined(CONFIG_I2S_BEE_TX)
			ret = i2s_tx_stream_start(dev);
#endif
		} else {
#if defined(CONFIG_I2S_BEE_RX)
			ret = i2s_rx_stream_start(dev);
#endif
		}

		if (ret < 0) {
			LOG_DBG("START trigger failed %d", ret);
			ret = -EIO;
			break;
		}

		strm->state = I2S_STATE_RUNNING;
		strm->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		if (strm->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}

		strm->state = I2S_STATE_STOPPING;
		strm->last_block = true;
		break;

	case I2S_TRIGGER_DRAIN:
		if (strm->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN/STOP trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}

		strm->state = I2S_STATE_STOPPING;
		break;

	case I2S_TRIGGER_DROP:
		if (strm->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}

		strm->state = I2S_STATE_READY;
		if (dir == I2S_DIR_TX) {
#if defined(CONFIG_I2S_BEE_TX)
			i2s_tx_stream_disable(dev, true);
#endif
		} else {
#if defined(CONFIG_I2S_BEE_RX)
			i2s_rx_stream_disable(dev, true, true);
#endif
		}
		break;

	case I2S_TRIGGER_PREPARE:
		if (strm->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state %d", strm->state);
			ret = -EIO;
			break;
		}
		strm->state = I2S_STATE_READY;
		if (dir == I2S_DIR_TX) {
#if defined(CONFIG_I2S_BEE_TX)
			i2s_tx_stream_disable(dev, true);
#endif
		} else {
#if defined(CONFIG_I2S_BEE_RX)
			i2s_rx_stream_disable(dev, true, true);
#endif
		}
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		ret = -EINVAL;
	}

	irq_unlock(key);

	return ret;
}

static void i2s_bee_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int i2s_bee_init(const struct device *dev)
{
	const struct i2s_bee_config *dev_cfg = dev->config;
	struct i2s_bee_data *dev_data = dev->data;
	I2S_TypeDef *base = (I2S_TypeDef *)dev_cfg->base;

#if defined(CONFIG_I2S_BEE_TX)
	if (!dev_data->dma_tx.dma_dev) {
		LOG_ERR("DMA device not found");
		return -ENODEV;
	}
#endif

#if defined(CONFIG_I2S_BEE_RX)
	if (!dev_data->dma_rx.dma_dev) {
		LOG_ERR("DMA device not found");
		return -ENODEV;
	}
#endif

	/* Initialize the buffer queues */
#if defined(CONFIG_I2S_BEE_TX)
	atomic_set_bit(((struct dma_context *)dev_data->dma_tx.dma_dev->data)->atomic,
		       dev_data->dma_tx.dma_channel);
	k_msgq_init(&dev_data->dma_tx.in_queue, (char *)dev_data->tx_in_msgs, sizeof(void *),
		    CONFIG_I2S_BEE_TX_BLOCK_COUNT);
	k_msgq_init(&dev_data->dma_tx.out_queue, (char *)dev_data->tx_out_msgs, sizeof(void *),
		    CONFIG_I2S_BEE_TX_BLOCK_COUNT);
	/* Configure dma tx config */
	memset(&(dev_data->dma_tx.blk_cfg[0]), 0, sizeof(dev_data->dma_tx.blk_cfg[0]));
	memset(&(dev_data->dma_tx.blk_cfg[1]), 0, sizeof(dev_data->dma_tx.blk_cfg[1]));
	dev_data->dma_tx.blk_cfg[0].dest_address = (uint32_t)(&(base->TX_DR));
	dev_data->dma_tx.blk_cfg[0].source_address = 0; /* not ready */
	dev_data->dma_tx.blk_cfg[0].source_addr_adj = dev_data->dma_tx.src_addr_increment;
	dev_data->dma_tx.blk_cfg[0].dest_addr_adj = dev_data->dma_tx.dst_addr_increment;
	dev_data->dma_tx.blk_cfg[0].next_block = &(dev_data->dma_tx.blk_cfg[1]);
	dev_data->dma_tx.blk_cfg[1].dest_address = (uint32_t)(&(base->TX_DR));
	dev_data->dma_tx.blk_cfg[1].source_address = 0; /* not ready */
	dev_data->dma_tx.blk_cfg[1].source_addr_adj = dev_data->dma_tx.src_addr_increment;
	dev_data->dma_tx.blk_cfg[1].dest_addr_adj = dev_data->dma_tx.dst_addr_increment;
	dev_data->dma_tx.blk_cfg[1].next_block = NULL;
	dev_data->dma_tx.dma_cfg.head_block = &dev_data->dma_tx.blk_cfg[0];
	dev_data->dma_tx.dma_cfg.user_data = (void *)dev;

	dev_data->dma_tx.state = I2S_STATE_NOT_READY;
#endif
#if defined(CONFIG_I2S_BEE_RX)
	atomic_set_bit(((struct dma_context *)dev_data->dma_rx.dma_dev->data)->atomic,
		       dev_data->dma_rx.dma_channel);
	k_msgq_init(&dev_data->dma_rx.in_queue, (char *)dev_data->rx_in_msgs, sizeof(void *),
		    CONFIG_I2S_BEE_RX_BLOCK_COUNT);
	k_msgq_init(&dev_data->dma_rx.out_queue, (char *)dev_data->rx_out_msgs, sizeof(void *),
		    CONFIG_I2S_BEE_RX_BLOCK_COUNT);
	/* Configure dma rx config */
	memset(&(dev_data->dma_rx.blk_cfg[0]), 0, sizeof(dev_data->dma_rx.blk_cfg[0]));
	memset(&(dev_data->dma_rx.blk_cfg[1]), 0, sizeof(dev_data->dma_rx.blk_cfg[1]));
	dev_data->dma_rx.blk_cfg[0].source_address = (uint32_t)(&(base->RX_DR));
	dev_data->dma_rx.blk_cfg[0].dest_address = 0; /* dest not ready */
	dev_data->dma_rx.blk_cfg[0].source_addr_adj = dev_data->dma_rx.src_addr_increment;
	dev_data->dma_rx.blk_cfg[0].dest_addr_adj = dev_data->dma_rx.dst_addr_increment;
	dev_data->dma_rx.blk_cfg[0].next_block = &(dev_data->dma_rx.blk_cfg[1]);
	dev_data->dma_rx.blk_cfg[1].source_address = (uint32_t)(&(base->RX_DR));
	dev_data->dma_rx.blk_cfg[1].dest_address = 0; /* dest not ready */
	dev_data->dma_rx.blk_cfg[1].source_addr_adj = dev_data->dma_rx.src_addr_increment;
	dev_data->dma_rx.blk_cfg[1].dest_addr_adj = dev_data->dma_rx.dst_addr_increment;
	dev_data->dma_rx.blk_cfg[1].next_block = NULL;
	dev_data->dma_rx.dma_cfg.head_block = &dev_data->dma_rx.blk_cfg[0];
	dev_data->dma_rx.dma_cfg.user_data = (void *)dev;

	dev_data->dma_rx.state = I2S_STATE_NOT_READY;
#endif

	/* register ISR */
	dev_cfg->irq_config_func(dev);

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

static const struct i2s_driver_api i2s_bee_driver_api = {
	.configure = i2s_bee_configure,
#if defined(CONFIG_I2S_BEE_RX)
	.read = i2s_bee_read,
#endif
#if defined(CONFIG_I2S_BEE_TX)
	.write = i2s_bee_write,
#endif
	.config_get = i2s_bee_config_get,
	.trigger = i2s_bee_trigger,
};

#define I2S_DMA_CHANNEL_INIT(index, dir)                                                           \
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
			.block_count = 2,                                                          \
			.complete_callback_en = true,                                              \
			.dma_callback = i2s_bee_dma_##dir##_cb,                                    \
			.cyclic = true,                                                            \
	},                                                                                         \
	.src_addr_increment = BEE_DMA_CONFIG_SOURCE_ADDR_INC(BEE_DMA_CHANNEL_CONFIG(index, dir)),  \
	.dst_addr_increment =                                                                      \
		BEE_DMA_CONFIG_DESTINATION_ADDR_INC(BEE_DMA_CHANNEL_CONFIG(index, dir)),

#define I2S_DMA_CHANNEL(index, dir)                                                                \
	.dma_##dir = {COND_CODE_1(DT_INST_DMAS_HAS_NAME(index, dir),                               \
				  (I2S_DMA_CHANNEL_INIT(index, dir)), (NULL))},

#if defined(CONFIG_I2S_BEE_TX) && defined(CONFIG_I2S_BEE_RX)
#define I2S_DMA_INIT(index) I2S_DMA_CHANNEL(index, tx) I2S_DMA_CHANNEL(index, rx)
#elif defined(CONFIG_I2S_BEE_TX)
#define I2S_DMA_INIT(index) I2S_DMA_CHANNEL(index, tx)
#elif defined(CONFIG_I2S_BEE_RX)
#define I2S_DMA_INIT(index) I2S_DMA_CHANNEL(index, rx)
#endif

#define BEE_I2S_INIT(index)                                                                        \
	static void i2s_bee_irq_config_func_##index(const struct device *dev);                     \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct i2s_bee_config i2s_bee_cfg_##index = {                                 \
		.base = (I2S_TypeDef *)DT_INST_REG_ADDR(index),                                    \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.irq_config_func = i2s_bee_irq_config_func_##index,                                \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                  \
	};                                                                                         \
                                                                                                   \
	static struct i2s_bee_data i2s_bee_data_##index = {I2S_DMA_INIT(index)};                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &i2s_bee_init, NULL, &i2s_bee_data_##index,                   \
			      &i2s_bee_cfg_##index, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,         \
			      &i2s_bee_driver_api);                                                \
                                                                                                   \
	static void i2s_bee_irq_config_func_##index(const struct device *dev)                      \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(index, 0, irq),                                     \
			    DT_INST_IRQ_BY_IDX(index, 0, priority), i2s_bee_isr,                   \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

DT_INST_FOREACH_STATUS_OKAY(BEE_I2S_INIT)
