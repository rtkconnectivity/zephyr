/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>

#include <errno.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl8752h_clock_control.h>
#include <soc.h>

#include <rtl876x_rcc.h>
#include <rtl876x_codec.h>

#include <zephyr/logging/log.h>

#include "trace.h"

#define DT_DRV_COMPAT realtek_rtl8752h_codec

LOG_MODULE_REGISTER(codec_rtl8752h, CONFIG_AUDIO_CODEC_LOG_LEVEL);

struct codec_rtl8752h_config {
	const struct device *bus;
	const struct pinctrl_dev_config *pinctrl;
};

struct codec_rtl8752h_data {
	CODEC_InitTypeDef codec_initstruct;
	struct i2s_config i2s_cfg;
	size_t pcm_mem_size;
	struct k_mem_slab *pcm_mem_slab;
};

enum codec_channel_sequence_e {
	CODEC_SEQ_L_R,
	CODEC_SEQ_R_L,
	CODEC_SEQ_L_L,
	CODEC_SEQ_R_R,
};

enum codec_mic_bias_e {
	CODEC_MIC_BIAS_1_507,
	CODEC_MIC_BIAS_1_62,
	CODEC_MIC_BIAS_1_705,
	CODEC_MIC_BIAS_1_8,
	CODEC_MIC_BIAS_1_906,
	CODEC_MIC_BIAS_2_025,
	CODEC_MIC_BIAS_2_16,
	CODEC_MIC_BIAS_2_314,
};

enum codec_mic_bst_gain_e {
	CODEC_MIC_BST_GAIN_0DB,
	CODEC_MIC_BST_GAIN_20DB,
	CODEC_MIC_BST_GAIN_30DB,
	CODEC_MIC_BST_GAIN_40DB,
};

enum codec_mic_bst_mode_e {
	CODEC_MIC_BST_MODE_SINGLE,
	CODEC_MIC_BST_MODE_DIFF,
};

enum codec_mic_type_e {
	CODEC_MIC_TYPE_AMIC,
	CODEC_MIC_TYPE_DMIC,
};

enum codec_dmic_data_latch_e {
	CODEC_DMIC_DATA_LATCH_FALLING,
	CODEC_DMIC_DATA_LATCH_RISING,
};

enum codec_boost_gain_e {
	CODEC_BOOST_GAIN_0DB,
	CODEC_BOOST_GAIN_12DB,
	CODEC_BOOST_GAIN_24DB,
	CODEC_BOOST_GAIN_36DB,
};

static const uint32_t codec_channel_sequence_table[] = {
	CODEC_I2S_CH_L_R,
	CODEC_I2S_CH_R_L,
	CODEC_I2S_CH_L_L,
	CODEC_I2S_CH_R_R,
};

static const uint32_t codec_mic_bias_table[] = {
	MICBIAS_VOLTAGE_1_507, MICBIAS_VOLTAGE_1_62,  MICBIAS_VOLTAGE_1_705, MICBIAS_VOLTAGE_1_8,
	MICBIAS_VOLTAGE_1_906, MICBIAS_VOLTAGE_2_025, MICBIAS_VOLTAGE_2_16,  MICBIAS_VOLTAGE_2_314,
};

static const uint32_t codec_mic_bst_gain_table[] = {
	MICBST_Gain_0dB,
	MICBST_Gain_20dB,
	MICBST_Gain_30dB,
	MICBST_Gain_40dB,
};

static const uint32_t codec_mic_bst_mode_table[] = {
	MICBST_Mode_Single,
	MICBST_Mode_Differential,
};

static const uint32_t codec_mic_type_table[] = {
	CODEC_CH0_AMIC,
	CODEC_CH0_DMIC,
};

static const uint32_t codec_dmic_data_latch_table[] = {
	DMIC_Ch0_Falling_Latch,
	DMIC_Ch0_Rising_Latch,
};

static const uint32_t codec_boost_gain_table[] = {
	Ch0_Boost_Gain_0dB,
	Ch0_Boost_Gain_12dB,
	Ch0_Boost_Gain_24dB,
	Ch0_Boost_Gain_36dB,
};

static int codec_rtl8752h_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	const struct codec_rtl8752h_config *config = dev->config;
	struct codec_rtl8752h_data *const data = dev->data;
	struct i2s_config *i2s_cfg = &cfg->dai_cfg.i2s;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		return -EINVAL;
	}

	memcpy(&data->i2s_cfg, i2s_cfg, sizeof(struct i2s_config));

	if (cfg->mclk_freq == 321500) {
		data->codec_initstruct.CODEC_DmicClock = DMIC_Clock_312500Hz;
	} else if (cfg->mclk_freq == 625000) {
		data->codec_initstruct.CODEC_DmicClock = DMIC_Clock_625KHz;
	} else if (cfg->mclk_freq == 1250000) {
		data->codec_initstruct.CODEC_DmicClock = DMIC_Clock_1250KHz;
	} else if (cfg->mclk_freq == 2500000) {
		data->codec_initstruct.CODEC_DmicClock = DMIC_Clock_2500KHz;
	} else if (cfg->mclk_freq == 50000000) {
		data->codec_initstruct.CODEC_DmicClock = DMIC_Clock_5MHz;
	} else {
		return -EINVAL;
	}

	if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_I2S) {
		data->codec_initstruct.CODEC_I2SFormat = CODEC_I2S_DataFormat_I2S;
	} else if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_PCM_SHORT) {
		data->codec_initstruct.CODEC_I2SFormat = CODEC_I2S_DataFormat_PCM_A;
	} else if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) == I2S_FMT_DATA_FORMAT_PCM_LONG) {
		data->codec_initstruct.CODEC_I2SFormat = CODEC_I2S_DataFormat_PCM_B;
	} else if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) ==
		   I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED) {
		data->codec_initstruct.CODEC_I2SFormat = CODEC_I2S_DataFormat_LeftJustified;
	} else {
		return -EINVAL;
	}

	if (i2s_cfg->word_size == 8) {
		data->codec_initstruct.CODEC_I2SDataWidth = CODEC_I2S_DataWidth_8Bits;
	} else if (i2s_cfg->word_size == 16) {
		data->codec_initstruct.CODEC_I2SDataWidth = CODEC_I2S_DataWidth_16Bits;
	} else if (i2s_cfg->word_size == 24) {
		data->codec_initstruct.CODEC_I2SDataWidth = CODEC_I2S_DataWidth_24Bits;
	} else {
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 8000) {
		data->codec_initstruct.CODEC_SampleRate = SAMPLE_RATE_8KHz;
	}

	if (i2s_cfg->frame_clk_freq == 16000) {
		data->codec_initstruct.CODEC_SampleRate = SAMPLE_RATE_16KHz;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int codec_rtl8752h_set_property(const struct device *dev, audio_property_t property,
				       audio_channel_t channel, audio_property_value_t val)
{
	const struct codec_rtl8752h_config *config = dev->config;
	struct codec_rtl8752h_data *const data = dev->data;

	if (property != AUDIO_PROPERTY_OUTPUT_MUTE) {
		return -EINVAL;
	}

	if (channel != AUDIO_CHANNEL_ALL) {
		return -EINVAL;
	}

	data->codec_initstruct.CODEC_Ch0Mute = val.mute ? CODEC_CH0_MUTE : CODEC_CH0_UNMUTE;

	return 0;
}

static int codec_rtl8752h_apply_properties(const struct device *dev)
{
	const struct codec_rtl8752h_config *config = dev->config;
	struct codec_rtl8752h_data *const data = dev->data;

	return 0;
}

static void codec_rtl8752h_start_output(const struct device *dev)
{
	const struct codec_rtl8752h_config *config = dev->config;
	struct codec_rtl8752h_data *const data = dev->data;

	pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);

	CODEC_AnalogCircuitInit();

	RCC_PeriphClockCmd(APBPeriph_CODEC, APBPeriph_CODEC_CLOCK, ENABLE);

	CODEC_Init(CODEC, &data->codec_initstruct);
}

static void codec_rtl8752h_stop_output(const struct device *dev)
{
	CODEC_DeInit(CODEC);
}

static const struct audio_codec_api codec_rtl8752h_driver_api = {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2s)
	.configure = codec_rtl8752h_configure,
	.set_property = codec_rtl8752h_set_property,
	.apply_properties = codec_rtl8752h_apply_properties,
	.start_output = codec_rtl8752h_start_output,
	.stop_output = codec_rtl8752h_stop_output,
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2s) */
};

static int codec_rtl8752h_init(const struct device *dev)
{
	const struct codec_rtl8752h_config *config = dev->config;
	struct codec_rtl8752h_data *const data = dev->data;

	if (!device_is_ready(config->bus)) {
		return -ENODEV;
	}

	/* pinctrl */
	pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);

	return 0;
}

#define RTL8752H_CODEC_INIT(index)                                                                 \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct codec_rtl8752h_config codec_rtl8752h_cfg_##index = {                   \
		.bus = DEVICE_DT_GET(DT_INST_BUS(0)),                                              \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                  \
	};                                                                                         \
                                                                                                   \
	static struct codec_rtl8752h_data codec_rtl8752h_data_##index = {                          \
		.codec_initstruct =                                                                \
			{                                                                          \
				.CODEC_SampleRate = SAMPLE_RATE_16KHz,                             \
				.CODEC_I2SFormat = CODEC_I2S_DataFormat_I2S,                       \
				.CODEC_I2SDataWidth = CODEC_I2S_DataWidth_16Bits,                  \
				.CODEC_I2SChSequence =                                             \
					codec_channel_sequence_table[DT_INST_ENUM_IDX_OR(          \
						index, channel_sequence, CODEC_I2S_CH_L_L)],       \
				.CODEC_MicBIAS = codec_mic_bias_table[DT_INST_ENUM_IDX_OR(         \
					index, mic_bias, MICBIAS_VOLTAGE_1_8)],                    \
				.CODEC_MicBstMode = codec_mic_bst_mode_table[DT_INST_ENUM_IDX_OR(  \
					index, mic_bst_mode, MICBST_Mode_Single)],                 \
				.CODEC_MicBstGain = codec_mic_bst_gain_table[DT_INST_ENUM_IDX_OR(  \
					index, mic_bst_gain, MICBST_Gain_0dB)],                    \
				.CODEC_DmicClock = DMIC_Clock_2500KHz,                             \
				.CODEC_Ch0MicType = codec_mic_type_table[DT_INST_ENUM_IDX_OR(      \
					index, mic_type, CODEC_CH0_AMIC)],                         \
				.CODEC_Ch0BoostGain = codec_boost_gain_table[DT_INST_ENUM_IDX_OR(  \
					index, boost_gain, Ch0_Boost_Gain_0dB)],                   \
				.CODEC_Ch0Mute = CODEC_CH0_UNMUTE,                                 \
				.CODEC_Ch0ZeroDetTimeout = Ch0_ADC_Zero_DetTimeout_1024_16_Sample, \
				.CODEC_Ch0DmicDataLatch =                                          \
					codec_dmic_data_latch_table[DT_INST_ENUM_IDX_OR(           \
						index, dmic_data_latch, DMIC_Ch0_Rising_Latch)],   \
				.CODEC_Ch0AdGain = DT_INST_PROP_OR(index, ad_gain, 0x2f),          \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &codec_rtl8752h_init, NULL, &codec_rtl8752h_data_##index,     \
			      &codec_rtl8752h_cfg_##index, POST_KERNEL,                            \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY, &codec_rtl8752h_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTL8752H_CODEC_INIT)
