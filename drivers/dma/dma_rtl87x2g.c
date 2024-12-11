/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_dma

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <rtl_rcc.h>
#include <rtl_gdma.h>

#include <trace.h>

BUILD_ASSERT(CONFIG_HEAP_MEM_POOL_SIZE > 0);

#define DBG_DIRECT_SHOW 0
LOG_MODULE_REGISTER(dma_rtl87x2g, CONFIG_DMA_LOG_LEVEL);

struct dma_rtl87x2g_config {
	uint32_t reg;
	uint32_t channels;
	uint16_t clkid;
	void (*irq_configure)(void);
	uint32_t channel_base_table[];
};

struct dma_rtl87x2g_channel {
	dma_callback_t callback;
	void *user_data;
	bool busy;
	struct dma_config cfg;
	uint32_t total_size;
	GDMA_LLIDef *p_dma_lli;
};

struct dma_rtl87x2g_data {
	struct dma_rtl87x2g_channel *channels;
};

static int dma_rtl87x2g_ch2num(uint32_t reg, uint32_t ch)
{
	if ((GDMA_TypeDef *)reg == GDMA0) {
		return ch;
	} else {
		return -EIO;
	}
}

extern FlagStatus GDMA_GetSuspendChannelStatus(GDMA_ChannelTypeDef *GDMA_Channelx);

/*
 * API functions
 */

static int dma_rtl87x2g_configure(const struct device *dev, uint32_t channel,
				  struct dma_config *dma_cfg)
{
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num;
	GDMA_ChannelTypeDef *dma_channel;
	struct dma_block_config *cur_block;

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] channel=%d, channel_direction=%d, block_size=%d, "
		   "source_addr_adj=%d dest_addr_adj=%d, "
		   "source_data_size=%d, dest_data_size=%d, source_burst_length=%d, "
		   "dest_burst_length=%d, dma_cfg->dma_slot=%d, line%d", __func__,
		   channel, dma_cfg->channel_direction, dma_cfg->head_block->block_size,
		   dma_cfg->head_block->source_addr_adj, dma_cfg->head_block->dest_addr_adj,
		   dma_cfg->source_data_size, dma_cfg->dest_data_size, dma_cfg->source_burst_length,
		   dma_cfg->dest_burst_length, dma_cfg->dma_slot, __LINE__);
	DBG_DIRECT(
		"[dma_rtl87x2g_configure] channel=%d, source_address=%x, dest_address=%x, line%d",
		channel, dma_cfg->head_block->source_address, dma_cfg->head_block->dest_address,
		__LINE__);
#endif

	dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
	dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

	GDMA_InitTypeDef dma_init_struct;

	GDMA_Cmd(dma_channel_num, DISABLE);

	GDMA_StructInit(&dma_init_struct);

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
		return -EINVAL;
	}

	if (dma_cfg->source_chaining_en || dma_cfg->dest_chaining_en) {
		LOG_ERR("src/dest chaining not supported.");
		return -ENOTSUP;
	}

	if (data->channels[channel].p_dma_lli != NULL) {
		k_free(data->channels[channel].p_dma_lli);
	}

	data->channels[channel].p_dma_lli = k_malloc(sizeof(GDMA_LLIDef) * dma_cfg->block_count);

	if (data->channels[channel].p_dma_lli == NULL) {
		LOG_ERR("p_dma_lli malloc fail");
		return -EINVAL;
	}

	if (dma_cfg->head_block->source_gather_count != 0 ||
	    dma_cfg->head_block->dest_scatter_count != 0) {
		LOG_ERR("gather/scatter not supported.");
		return -ENOTSUP;
	}

	if (dma_cfg->channel_priority > 9) {
		LOG_ERR("channel_priority must be < 9 (%" PRIu32 ")", dma_cfg->channel_priority);
		return -EINVAL;
	}

	switch (dma_cfg->source_data_size) {
	case 1:
		dma_init_struct.GDMA_SourceDataSize = GDMA_DataSize_Byte;
		break;

	case 2:
		dma_init_struct.GDMA_SourceDataSize = GDMA_DataSize_HalfWord;
		break;

	case 4:
		dma_init_struct.GDMA_SourceDataSize = GDMA_DataSize_Word;
		break;

	default:
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")",
			dma_cfg->source_data_size);
		return -EINVAL;
	}

	switch (dma_cfg->dest_data_size) {
	case 1:
		dma_init_struct.GDMA_DestinationDataSize = GDMA_DataSize_Byte;
		break;

	case 2:
		dma_init_struct.GDMA_DestinationDataSize = GDMA_DataSize_HalfWord;
		break;

	case 4:
		dma_init_struct.GDMA_DestinationDataSize = GDMA_DataSize_Word;
		break;

	default:
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")",
			dma_cfg->source_data_size);
		return -EINVAL;
	}

	switch (dma_cfg->source_burst_length) {
	case 1:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_1;
		break;

	case 4:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_4;
		break;

	case 8:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_8;
		break;

	case 16:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_16;
		break;

	case 32:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_32;
		break;

	case 64:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_64;
		break;

	case 128:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_128;
		break;

	case 256:
		dma_init_struct.GDMA_SourceMsize = GDMA_Msize_256;
		break;

	default:
		LOG_ERR("source_burst_length must be 1, 4, 8, 16, 32, 64, 128, or 256 (%" PRIu32
			")",
			dma_cfg->source_burst_length);
		return -EINVAL;
	}

	switch (dma_cfg->dest_burst_length) {
	case 1:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_1;
		break;

	case 4:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_4;
		break;

	case 8:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_8;
		break;

	case 16:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_16;
		break;

	case 32:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_32;
		break;

	case 64:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_64;
		break;

	case 128:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_128;
		break;

	case 256:
		dma_init_struct.GDMA_DestinationMsize = GDMA_Msize_256;
		break;

	default:
		LOG_ERR("source_burst_length must be 1, 4, 8, 16, 32, 64, 128, or 256 (%" PRIu32
			")",
			dma_cfg->source_burst_length);
		return -EINVAL;
	}

	dma_init_struct.GDMA_ChannelNum = dma_channel_num;
	dma_init_struct.GDMA_DIR = dma_cfg->channel_direction;
	if (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY) {
		dma_init_struct.GDMA_BufferSize =
			dma_cfg->head_block->block_size / dma_cfg->dest_data_size;
	} else {
		dma_init_struct.GDMA_BufferSize =
			dma_cfg->head_block->block_size / dma_cfg->source_data_size;
	}

	if (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
		dma_init_struct.GDMA_DestHandshake = dma_cfg->dma_slot;
	} else if (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY) {
		dma_init_struct.GDMA_SourceHandshake = dma_cfg->dma_slot;
	}

	dma_init_struct.GDMA_ChannelPriority = dma_cfg->channel_priority;
	dma_init_struct.GDMA_Multi_Block_En = ENABLE;
	dma_init_struct.GDMA_Multi_Block_Mode = LLI_TRANSFER;
	dma_init_struct.GDMA_Multi_Block_Struct = (uint32_t)(data->channels[channel].p_dma_lli);

	GDMA_Init(dma_channel, &dma_init_struct);
	cur_block = dma_cfg->head_block;
	data->channels[channel].total_size = 0;
	for (uint8_t i = 0; i < dma_cfg->block_count; i++) {
		if (cur_block == NULL) {
			LOG_ERR("block%d dose not exsist", i);
			return -EINVAL;
		}

		data->channels[channel].p_dma_lli[i].SAR = (uint32_t)(cur_block->source_address);
		data->channels[channel].p_dma_lli[i].DAR = (uint32_t)(cur_block->dest_address);
		data->channels[channel].p_dma_lli[i].LLP =
			(i < dma_cfg->block_count - 1)
				? (uint32_t)(&(data->channels[channel].p_dma_lli[i + 1]))
				: (dma_cfg->cyclic ? (uint32_t)(data->channels[channel].p_dma_lli)
						   : 0);

		data->channels[channel].p_dma_lli[i].CTL_LOW =
			BIT(0) | (dma_init_struct.GDMA_DestinationDataSize << 1) |
			(dma_init_struct.GDMA_SourceDataSize << 4) |
			(cur_block->dest_addr_adj << 7) | (cur_block->source_addr_adj << 9) |
			(dma_init_struct.GDMA_DestinationMsize << 11) |
			(dma_init_struct.GDMA_SourceMsize << 14) |
			(dma_cfg->channel_direction << 20) |
			((i < dma_cfg->block_count - 1)
				 ? (dma_init_struct.GDMA_Multi_Block_Mode & LLP_SELECTED_BIT)
				 : 0);

		if (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY) {
			data->channels[channel].p_dma_lli[i].CTL_HIGH =
				cur_block->block_size / dma_cfg->dest_data_size;
		} else {
			data->channels[channel].p_dma_lli[i].CTL_HIGH =
				cur_block->block_size / dma_cfg->source_data_size;
		}

		data->channels[channel].total_size += cur_block->block_size;

#if DBG_DIRECT_SHOW
		DBG_DIRECT(
			"[dma_rtl87x2g_configure] channel=%d p_dma_lli[%d], channel_direction=%d, "
			"block_size=%d, source_addr_adj=%d dest_addr_adj=%d, "
			"source_data_size=%d, dest_data_size=%d, source_burst_length=%d, "
			"dest_burst_length=%d, dma_cfg->dma_slot=%d, line%d",

			channel, i, dma_cfg->channel_direction, cur_block->block_size,
			cur_block->source_addr_adj, cur_block->dest_addr_adj,
			dma_cfg->source_data_size, dma_cfg->dest_data_size,
			dma_cfg->source_burst_length, dma_cfg->dest_burst_length, dma_cfg->dma_slot,
			__LINE__);
		DBG_DIRECT("[%s] channel=%d p_dma_lli[%d], source_address=%x, "
			   "dest_address=%x, LLP=0x%x, line%d", __func__,
			   channel, i, cur_block->source_address, cur_block->dest_address,
			   data->channels[channel].p_dma_lli[i].LLP, __LINE__);
#endif

		cur_block = cur_block->next_block;
	}

	data->channels[channel].callback = dma_cfg->dma_callback;
	data->channels[channel].user_data = dma_cfg->user_data;
	data->channels[channel].cfg = *dma_cfg;

	return 0;
}

static int dma_rtl87x2g_reload(const struct device *dev, uint32_t channel, uint32_t src,
			       uint32_t dst, size_t size)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] channel=%d, line%d", __func__, channel, __LINE__);
#endif
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num;
	GDMA_ChannelTypeDef *dma_channel;

	dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
	dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

	if (channel >= cfg->channels) {
		LOG_ERR("reload channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels,
			channel);
		return -EINVAL;
	}

	if (data->channels[channel].busy) {
		return -EBUSY;
	}

	GDMA_Cmd(dma_channel_num, DISABLE);

	if (data->channels[channel].p_dma_lli == NULL) {
		LOG_ERR("configure dma before reload");
		return -EINVAL;
	}

	data->channels[channel].p_dma_lli[0].SAR = (uint32_t)src;
	data->channels[channel].p_dma_lli[0].DAR = (uint32_t)dst;
	data->channels[channel].p_dma_lli[0].LLP = 0;
	if (data->channels->cfg.channel_direction == PERIPHERAL_TO_MEMORY) {
		data->channels[channel].p_dma_lli[0].CTL_HIGH =
			(size / data->channels[channel].cfg.dest_data_size);
	} else {
		data->channels[channel].p_dma_lli[0].CTL_HIGH =
			(size / data->channels[channel].cfg.source_data_size);
	}

	data->channels[channel].total_size = size;

	return 0;
}

static int dma_rtl87x2g_start(const struct device *dev, uint32_t channel)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] channel=%d, line%d", __func__, channel, __LINE__);
#endif
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num;
	GDMA_ChannelTypeDef *dma_channel;

	dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
	dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

	if (channel >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels,
			channel);
		return -EINVAL;
	}

	if (data->channels[channel].cfg.error_callback_en) {
		GDMA_INTConfig(dma_channel_num, GDMA_INT_Error, ENABLE);
	}

	GDMA_INTConfig(dma_channel_num, GDMA_INT_Block | GDMA_INT_Transfer, ENABLE);

	data->channels[channel].busy = true;

	dma_channel->GDMA_CTLx_H = data->channels[channel].p_dma_lli[0].CTL_HIGH;

	GDMA_CFGx_L_TypeDef gdma_0x40 = {.d32 = dma_channel->GDMA_CFGx_L};
	GDMA_CTLx_L_TypeDef gdma_0x18 = {.d32 = dma_channel->GDMA_CTLx_L};

	dma_channel->GDMA_LLPx_L = (uint32_t)(data->channels[channel].p_dma_lli);
	gdma_0x18.b.llp_dst_en = 1;
	gdma_0x18.b.llp_src_en = 1;
	gdma_0x40.b.reload_src = 0;
	gdma_0x40.b.reload_dst = 0;
	dma_channel->GDMA_CTLx_L = gdma_0x18.d32;
	dma_channel->GDMA_CFGx_L = gdma_0x40.d32;

	GDMA_SetSourceAddress(dma_channel, 0);
	GDMA_SetDestinationAddress(dma_channel, 0);

	GDMA_Cmd(dma_channel_num, ENABLE);

	return 0;
}

static int dma_rtl87x2g_stop(const struct device *dev, uint32_t channel)
{
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] channel=%d, line%d", __func__, channel, __LINE__);
#endif
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num;
	GDMA_ChannelTypeDef *dma_channel;

	dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
	dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

	if (channel >= cfg->channels) {
		LOG_ERR("stop channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
		return -EINVAL;
	}

	GDMA_INTConfig(dma_channel_num, GDMA_INT_Transfer | GDMA_INT_Error | GDMA_INT_Block,
		       DISABLE);

	GDMA_Cmd(dma_channel_num, DISABLE);
	data->channels[channel].busy = false;

	return 0;
}

static int dma_rtl87x2g_suspend(const struct device *dev, uint32_t channel)
{
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num;
	GDMA_ChannelTypeDef *dma_channel;

	dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
	dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

	if (channel >= cfg->channels) {
		LOG_ERR("suspend channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels,
			channel);
		return -EINVAL;
	}

	if (!data->channels[channel].busy) {
		LOG_ERR("suspend channel not busy");
		return -EINVAL;
	}

	GDMA_SuspendCmd(dma_channel, ENABLE);

	return 0;
}

static int dma_rtl87x2g_resume(const struct device *dev, uint32_t channel)
{
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num;
	GDMA_ChannelTypeDef *dma_channel;

	dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
	dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

	if (channel >= cfg->channels) {
		LOG_ERR("resume channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels,
			channel);
		return -EINVAL;
	}

	if (!data->channels[channel].busy) {
		LOG_ERR("resume channel not busy");
		return -EINVAL;
	}

	if (!GDMA_GetSuspendChannelStatus(dma_channel)) {
		LOG_ERR("resume channel not suspend");
		return -EINVAL;
	}

	GDMA_SuspendCmd(dma_channel, DISABLE);

	return 0;
}

static int dma_rtl87x2g_get_status(const struct device *dev, uint32_t ch, struct dma_status *stat)
{
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, ch);
	GDMA_ChannelTypeDef *dma_channel =
		(GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];
	bool suspending = GDMA_GetSuspendChannelStatus(dma_channel);

	if (ch >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, ch);
		return -EINVAL;
	}

	stat->busy = data->channels[ch].busy;
	if (data->channels[ch].busy) {
		GDMA_SuspendCmd(dma_channel, ENABLE);
		stat->pending_length =
			data->channels[ch].total_size - GDMA_GetTransferLen(dma_channel);

		if (!suspending) {
			GDMA_SuspendCmd(dma_channel, DISABLE);
		}
	} else {
		stat->pending_length = 0;
	}

	stat->dir = data->channels[ch].cfg.channel_direction;

	return 0;
}

static bool dma_rtl87x2g_api_chan_filter(const struct device *dev, int ch, void *filter_param)
{
	uint32_t filter;

	if (!filter_param) {
		return BIT(ch);
	}

	filter = *((uint32_t *)filter_param);

	return (filter & BIT(ch));
}

static int dma_rtl87x2g_init(const struct device *dev)
{
	const struct dma_rtl87x2g_config *cfg = dev->config;
	int dma_channel_num;

	RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, ENABLE);

	(void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	for (uint32_t i = 0; i < cfg->channels; i++) {
		dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, i);
		if (dma_channel_num >= 0) {
			GDMA_INTConfig(dma_channel_num,
				       GDMA_INT_Transfer | GDMA_INT_Error | GDMA_INT_Block,
				       DISABLE);
			GDMA_Cmd(dma_channel_num, DISABLE);
		}
	}

	cfg->irq_configure();

	return 0;
}

static void dma_rtl87x2g_isr(const struct device *dev)
{
	const struct dma_rtl87x2g_config *cfg = dev->config;
	struct dma_rtl87x2g_data *data = dev->data;
	int dma_channel_num;
	uint32_t errflag, ftfflag, blockflag;
	int err = 0;
	GDMA_ChannelTypeDef *dma_channel;

	for (uint32_t i = 0; i < cfg->channels; i++) {
		dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, i);
		dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];
		errflag = ((GDMA_TypeDef *)cfg->reg)->GDMA_STATUSERR_L & BIT(dma_channel_num);
		ftfflag = ((GDMA_TypeDef *)cfg->reg)->GDMA_STATUSTFR_L & BIT(dma_channel_num);
		blockflag = ((GDMA_TypeDef *)cfg->reg)->GDMA_STATUSBLOCK_L & BIT(dma_channel_num);

		GDMA_ClearINTPendingBit(dma_channel_num,
					GDMA_INT_Transfer | GDMA_INT_Error | GDMA_INT_Block);

#if DBG_DIRECT_SHOW
		DBG_DIRECT("[%s] channel %d transferlen%d callback %x ftfflag%d "
			   "errflag%d blockflag%d complete_callback_en%d", __func__,
			   i, GDMA_GetTransferLen(dma_channel), data->channels[i].callback, ftfflag,
			   errflag, blockflag, data->channels[i].cfg.complete_callback_en);
#endif

		if (errflag == 0 && ftfflag == 0 && blockflag == 0) {
			continue;
		}

		if (errflag) {
			err = -EIO;
		}

		if (ftfflag) {
			data->channels[i].busy = false;
		}

		if (blockflag) {
			data->channels[i].total_size -= GDMA_GetTransferLen(dma_channel);
		}

		if (data->channels[i].callback) {
			if (ftfflag || errflag ||
			    (blockflag && data->channels[i].cfg.complete_callback_en)) {
				data->channels[i].callback(dev, data->channels[i].user_data, i,
							   err);
			}
		}
	}
}

static const struct dma_driver_api dma_rtl87x2g_driver_api = {
	.config = dma_rtl87x2g_configure,
	.reload = dma_rtl87x2g_reload,
	.start = dma_rtl87x2g_start,
	.stop = dma_rtl87x2g_stop,
	.suspend = dma_rtl87x2g_suspend,
	.resume = dma_rtl87x2g_resume,
	.get_status = dma_rtl87x2g_get_status,
	.get_attribute = NULL,
	.chan_filter = dma_rtl87x2g_api_chan_filter,
};

#define IRQ_CONFIGURE(n, index)                                                                    \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(index, n, irq), DT_INST_IRQ_BY_IDX(index, n, priority),     \
		    dma_rtl87x2g_isr, DEVICE_DT_INST_GET(index), 0);                               \
	irq_enable(DT_INST_IRQ_BY_IDX(index, n, irq));

#define CONFIGURE_ALL_IRQS(index, n) LISTIFY(n, IRQ_CONFIGURE, (), index)

#define DMA_CHANNER_BASE(n, index, dma_port) (GDMA##dma_port##_Channel##n##_BASE)

#define RTL87X2G_DMA_INIT(index)                                                                   \
	static void dma_rtl87x2g_##index##_irq_configure(void)                                     \
	{                                                                                          \
		CONFIGURE_ALL_IRQS(index, DT_NUM_IRQS(DT_DRV_INST(index)));                        \
	}                                                                                          \
	static const struct dma_rtl87x2g_config dma_rtl87x2g_##index##_config = {                  \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.channels = DT_INST_PROP(index, dma_channels),                                     \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.irq_configure = dma_rtl87x2g_##index##_irq_configure,                             \
		.channel_base_table = {LISTIFY(DT_INST_PROP(index, dma_channels),                  \
					       DMA_CHANNER_BASE, (,), index,                      \
					       DT_INST_PROP(index, dma_port))},                    \
	};                                                                                         \
                                                                                                   \
	static struct dma_rtl87x2g_channel                                                         \
		dma_rtl87x2g_##index##_channels[DT_INST_PROP(index, dma_channels)];                \
	ATOMIC_DEFINE(dma_rtl87x2g_atomic##index, DT_INST_PROP(index, dma_channels));              \
	static struct dma_rtl87x2g_data dma_rtl87x2g_##index##_data = {                            \
		.channels = dma_rtl87x2g_##index##_channels,                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &dma_rtl87x2g_init, NULL, &dma_rtl87x2g_##index##_data,       \
			      &dma_rtl87x2g_##index##_config, PRE_KERNEL_1,                        \
			      CONFIG_DMA_INIT_PRIORITY, &dma_rtl87x2g_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTL87X2G_DMA_INIT)
