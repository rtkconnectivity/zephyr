/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include "trace.h"
const struct device *dev_uart = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_console));
const struct device *dev_i2s = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2s));
const struct device *dev_codec = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(codec));

#define BLOCK_SIZE    256
#define NUM_RX_BLOCKS 2

K_MEM_SLAB_DEFINE(rx_mem_slab, BLOCK_SIZE, NUM_RX_BLOCKS, 32);
K_MEM_SLAB_DEFINE(tx_mem_slab, BLOCK_SIZE, NUM_RX_BLOCKS, 32);

#define TIMEOUT_SECONDS K_SECONDS(3)
static void i2s_timer_timeout(struct k_timer *timer);
K_TIMER_DEFINE(i2s_timer, i2s_timer_timeout, NULL);

uint8_t rx_buf[BLOCK_SIZE];
uint8_t tx_buf[BLOCK_SIZE];

bool stop;

static void i2s_timer_timeout(struct k_timer *timer)
{
	k_timer_stop(timer);
	stop = true;
}

static void i2s_codec_rx_sample(void)
{
	int ret;

	struct audio_codec_cfg codec_cfg;
	audio_property_t property = AUDIO_PROPERTY_OUTPUT_MUTE;
	audio_channel_t channel = AUDIO_CHANNEL_ALL;
	audio_property_value_t val = {
		.vol = 0,
		.mute = false,
	};

	memset(&codec_cfg, 0, sizeof(struct audio_codec_cfg));
	codec_cfg.mclk_freq = 2500000;
	codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	codec_cfg.dai_cfg.i2s.frame_clk_freq = 16000;
	codec_cfg.dai_cfg.i2s.word_size = 16;
	codec_cfg.dai_cfg.i2s.channels = 1;
	codec_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_DATA_ORDER_MSB;
	codec_cfg.dai_cfg.i2s.options = I2S_OPT_BIT_CLK_CONT | I2S_OPT_BIT_CLK_MASTER |
					I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_PINGPONG;

	codec_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE;
	codec_cfg.dai_cfg.i2s.mem_slab = &rx_mem_slab;
	codec_cfg.dai_cfg.i2s.timeout = SYS_FOREVER_MS;

	ret = audio_codec_configure(dev_codec, &codec_cfg);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	ret = audio_codec_set_property(dev_codec, property, channel, val);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	ret = audio_codec_apply_properties(dev_codec);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	/* init codec pin + init codec reg */
	audio_codec_start_output(dev_codec);

	/* enable i2s rcc + init i2s reg */
	codec_cfg.dai_cfg.i2s.frame_clk_freq = 16000;
	ret = i2s_configure(dev_i2s, I2S_DIR_RX, &codec_cfg.dai_cfg.i2s);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	/* enable i2s + config dma + enable dma */
	ret = i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	k_timer_start(&i2s_timer, TIMEOUT_SECONDS, K_NO_WAIT);
	while (!stop) {
		void *mem_block;
		size_t rx_size;
		const struct i2s_config *rx_cfg;

		ret = i2s_read(dev_i2s, &mem_block, &rx_size);
		if (ret) {
			printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
		}
		rx_cfg = i2s_config_get(dev_i2s, I2S_DIR_RX);
		memcpy(rx_buf, mem_block, rx_size);
		k_mem_slab_free(rx_cfg->mem_slab, mem_block);

		for (uint32_t i = 0; i < rx_size; i += 8) {
			uart_poll_out(dev_uart, rx_buf[i]);
			uart_poll_out(dev_uart, rx_buf[i + 1]);
			uart_poll_out(dev_uart, rx_buf[i + 2]);
			uart_poll_out(dev_uart, rx_buf[i + 3]);
			uart_poll_out(dev_uart, rx_buf[i + 4]);
			uart_poll_out(dev_uart, rx_buf[i + 5]);
			uart_poll_out(dev_uart, rx_buf[i + 6]);
			uart_poll_out(dev_uart, rx_buf[i + 7]);
		}
	}

	stop = false;

	/* disable i2s + disable dma */
	ret = i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_DROP);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	/* disable i2s rcc */
	codec_cfg.dai_cfg.i2s.frame_clk_freq = 0;
	ret = i2s_configure(dev_i2s, I2S_DIR_RX, &codec_cfg.dai_cfg.i2s);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	/* deinit codec */
	audio_codec_stop_output(dev_codec);
}

static void i2s_tx_sample(void)
{
	struct i2s_config i2s_cfg;
	int ret;

	memset(&i2s_cfg, 0, sizeof(struct i2s_config));
	i2s_cfg.frame_clk_freq = 16000;
	i2s_cfg.word_size = 16;
	i2s_cfg.channels = 1;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_DATA_ORDER_MSB;
	i2s_cfg.options = I2S_OPT_BIT_CLK_CONT | I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER |
			  I2S_OPT_PINGPONG;

	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.mem_slab = &tx_mem_slab;
	i2s_cfg.timeout = SYS_FOREVER_MS;

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	for (uint32_t i = 0; i < BLOCK_SIZE / 2; i++) {
		((uint16_t *)tx_buf)[i] = i;
	}

	ret = i2s_buf_write(dev_i2s, tx_buf, BLOCK_SIZE);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	k_timer_start(&i2s_timer, TIMEOUT_SECONDS, K_NO_WAIT);
	while (!stop) {
		ret = i2s_buf_write(dev_i2s, tx_buf, BLOCK_SIZE);
		if (ret) {
			printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
		}
	}

	stop = false;

	/* disable i2s + disable dma */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}

	/* disable i2s rcc */
	i2s_cfg.frame_clk_freq = 0;
	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (ret) {
		printf("[%s] ret%d line%d\n", __func__, ret, __LINE__);
	}
}

int main(void)
{
	i2s_codec_rx_sample();
	i2s_tx_sample();
	return 0;
}
