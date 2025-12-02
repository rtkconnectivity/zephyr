/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
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
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <soc.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_rcc.h>
#include <rtl_codec.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_rcc.h>
#include <rtl876x_codec.h>
#endif

#include <zephyr/logging/log.h>

#include "trace.h"

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define BEE_CODEC_CH_AMIC              CODEC_CH_AMIC
#define BEE_CODEC_CH_DMIC              CODEC_CH_DMIC
#define BEE_DMIC_Ch_Falling_Latch      DMIC_Ch_Falling_Latch
#define BEE_DMIC_Ch_Rising_Latch       DMIC_Ch_Rising_Latch
#define BEE_CODEC_I2SDataWidth         CODEC_I2SRxDataWidth
#define BEE_CODEC_I2S_DataWidth_8Bits  CODEC_I2S_Rx_DataWidth_8Bits
#define BEE_CODEC_I2S_DataWidth_16Bits CODEC_I2S_Rx_DataWidth_16Bits
#define BEE_CODEC_ADC_SampleRate       CODEC_SampleRate0
#define BEE_CODEC_DAC_SampleRate       CODEC_SampleRate1
#define BEE_CODEC_MUTE                 CODEC_MUTE
#define BEE_CODEC_UNMUTE               CODEC_UNMUTE
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define BEE_CODEC_CH_AMIC              CODEC_CH0_AMIC
#define BEE_CODEC_CH_DMIC              CODEC_CH0_DMIC
#define BEE_DMIC_Ch_Falling_Latch      DMIC_Ch0_Falling_Latch
#define BEE_DMIC_Ch_Rising_Latch       DMIC_Ch0_Rising_Latch
#define BEE_CODEC_I2SDataWidth         CODEC_I2SDataWidth
#define BEE_CODEC_I2S_DataWidth_8Bits  CODEC_I2S_DataWidth_8Bits
#define BEE_CODEC_I2S_DataWidth_16Bits CODEC_I2S_DataWidth_16Bits
#define BEE_CODEC_I2S_DataWidth_24Bits CODEC_I2S_DataWidth_24Bits
#define BEE_CODEC_ADC_SampleRate       CODEC_SampleRate
#define BEE_CODEC_MUTE                 CODEC_CH0_MUTE
#define BEE_CODEC_UNMUTE               CODEC_CH0_UNMUTE

#endif

#define DT_DRV_COMPAT realtek_bee_codec

LOG_MODULE_REGISTER(codec_bee, CONFIG_AUDIO_CODEC_LOG_LEVEL);

struct codec_bee_config {
	const struct device *bus;
	const struct pinctrl_dev_config *pinctrl;
};

struct codec_bee_data {
	CODEC_InitTypeDef codec_initstruct;
	struct i2s_config i2s_cfg;
	size_t pcm_mem_size;
	struct k_mem_slab *pcm_mem_slab;
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
	BEE_CODEC_CH_AMIC,
	BEE_CODEC_CH_DMIC,
};

static const uint32_t codec_dmic_data_latch_table[] = {
	BEE_DMIC_Ch_Falling_Latch,
	BEE_DMIC_Ch_Rising_Latch,
};

#if defined(CONFIG_SOC_SERIES_RTL8752H)
static const uint32_t codec_boost_gain_table[] = {
	Ch0_Boost_Gain_0dB,
	Ch0_Boost_Gain_12dB,
	Ch0_Boost_Gain_24dB,
	Ch0_Boost_Gain_36dB,
};
#endif

static int codec_bee_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	struct codec_bee_data *data = dev->data;
	struct i2s_config *i2s_cfg = &cfg->dai_cfg.i2s;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S || !i2s_cfg) {
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
		data->codec_initstruct.BEE_CODEC_I2SDataWidth = BEE_CODEC_I2S_DataWidth_8Bits;
	} else if (i2s_cfg->word_size == 16) {
		data->codec_initstruct.BEE_CODEC_I2SDataWidth = BEE_CODEC_I2S_DataWidth_16Bits;
#if defined(CONFIG_SOC_SERIES_RTL8752H)
	} else if (i2s_cfg->word_size == 24) {
		data->codec_initstruct.BEE_CODEC_I2SDataWidth = BEE_CODEC_I2S_DataWidth_24Bits;
#endif
	} else {
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 8000) {
		data->codec_initstruct.BEE_CODEC_ADC_SampleRate = SAMPLE_RATE_8KHz;
	}

	if (i2s_cfg->frame_clk_freq == 16000) {
		data->codec_initstruct.BEE_CODEC_ADC_SampleRate = SAMPLE_RATE_16KHz;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int codec_bee_set_property(const struct device *dev, audio_property_t property,
				  audio_channel_t channel, audio_property_value_t val)
{
	struct codec_bee_data *data = dev->data;

	if (channel != AUDIO_CHANNEL_ALL) {
		return -EINVAL;
	}

	if (property == AUDIO_PROPERTY_OUTPUT_MUTE) {
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		data->codec_initstruct.CODEC_Ch0Mute = val.mute ? BEE_CODEC_MUTE : BEE_CODEC_UNMUTE;
		if (data->codec_initstruct.CODEC_Ch1MicType == BEE_CODEC_CH_DMIC &&
		    data->i2s_cfg.channels == 2) {
			data->codec_initstruct.CODEC_Ch1Mute =
				val.mute ? BEE_CODEC_MUTE : BEE_CODEC_UNMUTE;
		} else if (data->i2s_cfg.channels == 0) {
			data->codec_initstruct.CODEC_DaMute =
				val.mute ? BEE_CODEC_MUTE : BEE_CODEC_UNMUTE;
		}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
		data->codec_initstruct.CODEC_Ch0Mute = val.mute ? BEE_CODEC_MUTE : BEE_CODEC_UNMUTE;
#endif
	} else if (property == AUDIO_PROPERTY_OUTPUT_VOLUME) {
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		data->codec_initstruct.CODEC_Ch0Mute = val.mute ? BEE_CODEC_MUTE : BEE_CODEC_UNMUTE;
		if (data->codec_initstruct.CODEC_Ch1MicType == BEE_CODEC_CH_DMIC &&
		    data->i2s_cfg.channels == 2) {
			data->codec_initstruct.CODEC_Ch1AdGain = val.vol;
		} else if (data->i2s_cfg.channels == 0) {
			data->codec_initstruct.CODEC_DaGain = val.vol;
		}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
		data->codec_initstruct.CODEC_Ch0AdGain = val.vol;
#endif
	} else {
		return -EINVAL;
	}

	return 0;
}

static int codec_bee_apply_properties(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static void codec_bee_start_output(const struct device *dev)
{
	const struct codec_bee_config *config = dev->config;
	struct codec_bee_data *data = dev->data;

	pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);

	CODEC_AnalogCircuitInit();

	RCC_PeriphClockCmd(APBPeriph_CODEC, APBPeriph_CODEC_CLOCK, ENABLE);

	CODEC_Init(CODEC, &data->codec_initstruct);
}

static void codec_bee_stop_output(const struct device *dev)
{
	CODEC_DeInit(CODEC);
}

static const struct audio_codec_api codec_bee_driver_api = {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2s)
	.configure = codec_bee_configure,
	.set_property = codec_bee_set_property,
	.apply_properties = codec_bee_apply_properties,
	.start_output = codec_bee_start_output,
	.stop_output = codec_bee_stop_output,
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2s) */
};

static int codec_bee_init(const struct device *dev)
{
	const struct codec_bee_config *config = dev->config;

	if (!device_is_ready(config->bus)) {
		return -ENODEV;
	}

	/* pinctrl */
	pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);

	return 0;
}

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define BEE_CODEC_STRUCT_INIT(index)                                                               \
	.codec_initstruct = {                                                                      \
		.CODEC_AdSampleRateSrc = SAMPLE_RATE_SRC0,                                         \
		.CODEC_DaSampleRateSrc = SAMPLE_RATE_SRC1,                                         \
		.BEE_CODEC_ADC_SampleRate = SAMPLE_RATE_16KHz,                                     \
		.BEE_CODEC_DAC_SampleRate = SAMPLE_RATE_16KHz,                                     \
		.CODEC_I2SFormat = CODEC_I2S_DataFormat_I2S,                                       \
		.BEE_CODEC_I2SDataWidth = BEE_CODEC_I2S_DataWidth_16Bits,                          \
		.CODEC_I2STxDataWidth = BEE_CODEC_I2S_DataWidth_16Bits,                            \
		.CODEC_I2SSameLrcEn = DISABLE,                                                     \
		.CODEC_I2SChSequence = codec_channel_sequence_table[DT_INST_ENUM_IDX_OR(           \
			index, channel_sequence, CODEC_I2S_CH_L_R)],                               \
		.CODEC_MicBIAS = codec_mic_bias_table[DT_INST_ENUM_IDX_OR(index, mic_bias,         \
									  MICBIAS_VOLTAGE_1_8)],   \
		.CODEC_MicBstMode = codec_mic_bst_mode_table[DT_INST_ENUM_IDX_OR(                  \
			index, mic_bst_mode, MICBST_Mode_Single)],                                 \
		.CODEC_MicBstGain = codec_mic_bst_gain_table[DT_INST_ENUM_IDX_OR(                  \
			index, mic_bst_gain, MICBST_Gain_20dB)],                                   \
		.CODEC_DmicClock = DMIC_Clock_2500KHz,                                             \
		.CODEC_Ch0MicType = codec_mic_type_table[DT_INST_ENUM_IDX_OR(index, mic_type,      \
									     BEE_CODEC_CH_AMIC)],  \
		.CODEC_Ch0Mute = BEE_CODEC_MUTE,                                                   \
		.CODEC_Ch0DmicDataLatch = codec_dmic_data_latch_table[DT_INST_ENUM_IDX_OR(         \
			index, dmic_data_latch, DMIC_Ch0_Rising_Latch)],                           \
		.CODEC_Ch0AdGain = DT_INST_PROP_OR(index, ad_gain, 0x2f),                          \
		.CODEC_Ch1MicType = codec_mic_type_table[DT_INST_ENUM_IDX_OR(index, mic_type,      \
									     BEE_CODEC_CH_AMIC)],  \
		.CODEC_Ch1Mute = BEE_CODEC_MUTE,                                                   \
		.CODEC_Ch1DmicDataLatch = codec_dmic_data_latch_table[DT_INST_ENUM_IDX_OR(         \
			index, dmic_data_latch, DMIC_Ch0_Rising_Latch)],                           \
		.CODEC_Ch1AdGain = DT_INST_PROP_OR(index, ad_gain, 0x2f),                          \
		.CODEC_DaMute = BEE_CODEC_MUTE,                                                    \
		.CODEC_DaGain = DT_INST_PROP_OR(index, da_gain, 0xAf),                             \
		.CODEC_DaC_Dither = DAC_DA_DITHER_DISABLE,                                         \
		.CODEC_I2SChannelLen = I2S_CHANNELLEN_32,                                          \
	},
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define BEE_CODEC_STRUCT_INIT(index)                                                               \
	.codec_initstruct = {                                                                      \
		.BEE_CODEC_ADC_SampleRate = SAMPLE_RATE_16KHz,                                     \
		.CODEC_I2SFormat = CODEC_I2S_DataFormat_I2S,                                       \
		.CODEC_I2SDataWidth = BEE_CODEC_I2S_DataWidth_16Bits,                              \
		.CODEC_I2SChSequence = codec_channel_sequence_table[DT_INST_ENUM_IDX_OR(           \
			index, channel_sequence, CODEC_I2S_CH_L_L)],                               \
		.CODEC_MicBIAS = codec_mic_bias_table[DT_INST_ENUM_IDX_OR(index, mic_bias,         \
									  MICBIAS_VOLTAGE_1_8)],   \
		.CODEC_MicBstMode = codec_mic_bst_mode_table[DT_INST_ENUM_IDX_OR(                  \
			index, mic_bst_mode, MICBST_Mode_Single)],                                 \
		.CODEC_MicBstGain = codec_mic_bst_gain_table[DT_INST_ENUM_IDX_OR(                  \
			index, mic_bst_gain, MICBST_Gain_0dB)],                                    \
		.CODEC_DmicClock = DMIC_Clock_2500KHz,                                             \
		.CODEC_Ch0MicType = codec_mic_type_table[DT_INST_ENUM_IDX_OR(index, mic_type,      \
									     BEE_CODEC_CH_AMIC)],  \
		.CODEC_Ch0BoostGain = codec_boost_gain_table[DT_INST_ENUM_IDX_OR(                  \
			index, boost_gain, Ch0_Boost_Gain_0dB)],                                   \
		.CODEC_Ch0Mute = BEE_CODEC_UNMUTE,                                                 \
		.CODEC_Ch0DmicDataLatch = codec_dmic_data_latch_table[DT_INST_ENUM_IDX_OR(         \
			index, dmic_data_latch, DMIC_Ch0_Rising_Latch)],                           \
		.CODEC_Ch0AdGain = DT_INST_PROP_OR(index, ad_gain, 0x2f),                          \
	},
#endif

#define BEE_CODEC_INIT(index)                                                                      \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct codec_bee_config codec_bee_cfg_##index = {                             \
		.bus = DEVICE_DT_GET(DT_INST_BUS(0)),                                              \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                  \
	};                                                                                         \
                                                                                                   \
	static struct codec_bee_data codec_bee_data_##index = {BEE_CODEC_STRUCT_INIT(index)};      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &codec_bee_init, NULL, &codec_bee_data_##index,               \
			      &codec_bee_cfg_##index, POST_KERNEL,                                 \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY, &codec_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BEE_CODEC_INIT)
