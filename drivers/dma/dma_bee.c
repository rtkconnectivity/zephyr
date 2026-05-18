/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_dma

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/mem_blocks.h>
#include <zephyr/spinlock.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_rcc.h>
#include <rtl_gdma.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_rcc.h>
#include <rtl876x_gdma.h>
#include <flash_nor_device.h>
#include <fmc_platform.h>
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#include <rtl_rcc.h>
#include <rtl_dma.h>
#else
#error "Unsupported Realtek Bee SoC series"
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dma_bee, CONFIG_DMA_LOG_LEVEL);

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
/* DMA channel multi-block mode support */
#define DMA_HAS_MULTI_BLOCK_MODE(id)                                                               \
	((id) == 0 || (id) == 1 || (id) == 2 || (id) == 3 || (id) == 4 || (id) == 5 ||             \
	 (id) == 6 || (id) == 7 || (id) == 8)

/* Type definitions */
typedef GDMA_ChannelTypeDef bee_dma_channel_t;
typedef GDMA_InitTypeDef bee_dma_init_t;
typedef GDMA_TypeDef bee_dma_type_t;
typedef GDMA_LLIDef bee_dma_lli_t;
typedef GDMADataSize_TypeDef bee_dma_data_size_t;
typedef GDMAMSize_TypeDef bee_dma_msize_t;

/* Init structure members */
#define BEE_DMA_CHANNEL_NUM GDMA_ChannelNum
#define BEE_DMA_STRUCT_INIT GDMA_StructInit

/* DMA configuration */
#define BEE_DMA_DIRECTION           GDMA_DIR
#define BEE_DMA_BUFFER_SIZE         GDMA_BufferSize
#define BEE_DMA_SOURCE_HANDSHAKE    GDMA_SourceHandshake
#define BEE_DMA_DEST_HANDSHAKE      GDMA_DestHandshake
#define BEE_DMA_SOURCE_DATA_SIZE    GDMA_SourceDataSize
#define BEE_DMA_DEST_DATA_SIZE      GDMA_DestinationDataSize
#define BEE_DMA_SOURCE_INC          GDMA_SourceInc
#define BEE_DMA_DEST_INC            GDMA_DestinationInc
#define BEE_DMA_SOURCE_MSIZE        GDMA_SourceMsize
#define BEE_DMA_DEST_MSIZE          GDMA_DestinationMsize
#define BEE_DMA_SOURCE_ADDR         GDMA_SourceAddr
#define BEE_DMA_DEST_ADDR           GDMA_DestinationAddr
#define BEE_DMA_CHANNEL_PRIORITY    GDMA_ChannelPriority
#define BEE_DMA_MULTI_BLOCK_EN      GDMA_Multi_Block_En
#define BEE_DMA_MULTI_BLOCK_STRUCT  GDMA_Multi_Block_Struct
#define BEE_DMA_MULTI_BLOCK_MODE    GDMA_Multi_Block_Mode
#define BEE_DMA_MSIZE(n)            GDMA_Msize_##n
#define BEE_DMA_DATA_SIZE_BYTE      GDMA_DataSize_Byte
#define BEE_DMA_DATA_SIZE_HALF_WORD GDMA_DataSize_HalfWord
#define BEE_DMA_DATA_SIZE_WORD      GDMA_DataSize_Word
#define BEE_DMA_SRC_ADDR_ADJ        GDMA_SourceAddr_Inc
#define BEE_DMA_DEST_ADDR_ADJ       GDMA_DestAddr_Inc

/* Interrupt and status */
#define BEE_DMA_INT_ERROR    GDMA_INT_Error
#define BEE_DMA_INT_TRANSFER GDMA_INT_Transfer
#define BEE_DMA_INT_BLOCK    GDMA_INT_Block

/* Register access */
#define BEE_DMA_REG_STATUS_ERR   GDMA_STATUSERR_L
#define BEE_DMA_REG_STATUS_TFR   GDMA_STATUSTFR_L
#define BEE_DMA_REG_STATUS_BLOCK GDMA_STATUSBLOCK_L
#define BEE_GDMA_REG_CTLx_L      GDMA_CTLx_L
#define BEE_GDMA_REG_CFGx_L      GDMA_CFGx_L

/* External function declarations */
extern GDMA_TypeDef *GDMA_GetGDMAxByCh(uint8_t GDMA_ChannelNum);
extern uint8_t GDMA_GetGDMAChannelNum(uint8_t GDMA_ChannelNum);
extern FlagStatus GDMA_GetSuspendChannelStatus(bee_dma_channel_t *GDMA_Channelx);

/* Interrupt status macros */
#define DMA_GET_ERROR_INT_STATUS(channel_num)                                                      \
	(!!(GDMA_GetGDMAxByCh(channel_num)->GDMA_STATUSERR_L &                                     \
	    BIT(GDMA_GetGDMAChannelNum(channel_num))))

#define DMA_GET_BLOCK_INT_STATUS(channel_num)                                                      \
	(!!(GDMA_GetGDMAxByCh(channel_num)->GDMA_STATUSBLOCK_L &                                   \
	    BIT(GDMA_GetGDMAChannelNum(channel_num))))

/* DMA API macros */
#define BEE_DMA_INIT(channel, struct)                   GDMA_Init(channel, struct)
#define BEE_DMA_CMD(channel_num, cmd)                   GDMA_Cmd(channel_num, cmd)
#define BEE_DMA_SUSPEND_CMD(channel_num, cmd)           GDMA_SuspendCmd(channel_num, cmd)
#define BEE_DMA_GET_TRANSFER_LEN(channel)               GDMA_GetTransferLen(channel)
#define BEE_DMA_SET_BUFFER_SIZE(channel, size)          GDMA_SetBufferSize(channel, size)
#define BEE_DMA_SET_SOURCE_ADDRESS(channel, src)        GDMA_SetSourceAddress(channel, src)
#define BEE_DMA_SET_DESTINATION_ADDRESS(channel, src)   GDMA_SetDestinationAddress(channel, src)
#define BEE_DMA_INT_CONFIG(channel_num, int, cmd)       GDMA_INTConfig(channel_num, int, cmd)
#define BEE_DMA_CLEAR_INT_PENDING_BIT(channel_num, int) GDMA_ClearINTPendingBit(channel_num, int)
#define BEE_DMA_GET_SUSPEND_CHANNEL_STATUS(channel)     GDMA_GetSuspendChannelStatus(channel)
#define BEE_DMA_SET_LLP_ADDRESS(channel, addr)          GDMA_SetLLPAddress(channel, addr)
#define BEE_DMA_GET_TRANSFER_INT_STATUS(channel_num)    GDMA_GetTransferINTStatus(channel_num)
#define BEE_DMA_GET_SUSPEND_STATUS(channel) (((bee_dma_channel_t *)(channel))->GDMA_CFGx_L & BIT8)

/* Address conversion */
#define DMA_BEE_ADDR(addr) ((uint32_t)addr)
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
/* DMA channel multi-block mode support */
#define DMA_HAS_MULTI_BLOCK_MODE(id)             ((id) == 0 || (id) == 1)

/* Type definitions */
typedef GDMA_ChannelTypeDef bee_dma_channel_t;
typedef GDMA_InitTypeDef bee_dma_init_t;
typedef GDMA_TypeDef bee_dma_type_t;
typedef GDMA_LLIDef bee_dma_lli_t;
typedef uint32_t bee_dma_data_size_t;
typedef uint32_t bee_dma_msize_t;

/* Init structure members */
#define BEE_DMA_CHANNEL_NUM                      GDMA_ChannelNum
#define BEE_DMA_STRUCT_INIT                      GDMA_StructInit

/* DMA configuration */
#define BEE_DMA_DIRECTION                        GDMA_DIR
#define BEE_DMA_BUFFER_SIZE                      GDMA_BufferSize
#define BEE_DMA_SOURCE_HANDSHAKE                 GDMA_SourceHandshake
#define BEE_DMA_DEST_HANDSHAKE                   GDMA_DestHandshake
#define BEE_DMA_SOURCE_DATA_SIZE                 GDMA_SourceDataSize
#define BEE_DMA_DEST_DATA_SIZE                   GDMA_DestinationDataSize
#define BEE_DMA_SOURCE_INC                       GDMA_SourceInc
#define BEE_DMA_DEST_INC                         GDMA_DestinationInc
#define BEE_DMA_SOURCE_MSIZE                     GDMA_SourceMsize
#define BEE_DMA_DEST_MSIZE                       GDMA_DestinationMsize
#define BEE_DMA_SOURCE_ADDR                      GDMA_SourceAddr
#define BEE_DMA_DEST_ADDR                        GDMA_DestinationAddr
#define BEE_DMA_CHANNEL_PRIORITY                 GDMA_ChannelPriority
#define BEE_DMA_MULTI_BLOCK_EN                   GDMA_Multi_Block_En
#define BEE_DMA_MULTI_BLOCK_STRUCT               GDMA_Multi_Block_Struct
#define BEE_DMA_MULTI_BLOCK_MODE                 GDMA_Multi_Block_Mode
#define BEE_DMA_MSIZE(n)                         GDMA_Msize_##n
#define BEE_DMA_DATA_SIZE_BYTE                   GDMA_DataSize_Byte
#define BEE_DMA_DATA_SIZE_HALF_WORD              GDMA_DataSize_HalfWord
#define BEE_DMA_DATA_SIZE_WORD                   GDMA_DataSize_Word
#define BEE_DMA_SRC_ADDR_ADJ                     GDMA_SourceAddr_Inc
#define BEE_DMA_DEST_ADDR_ADJ                    GDMA_DestAddr_Inc

/* Interrupt and status */
#define BEE_DMA_INT_ERROR                        GDMA_INT_Error
#define BEE_DMA_INT_TRANSFER                     GDMA_INT_Transfer
#define BEE_DMA_INT_BLOCK                        GDMA_INT_Block

/* Register access */
#define BEE_DMA_REG_STATUS_ERR                   STATUS_ERR
#define BEE_DMA_REG_STATUS_TFR                   STATUS_TFR
#define BEE_DMA_REG_STATUS_BLOCK                 STATUS_BLOCK
#define BEE_GDMA_REG_CTLx_L                      CTL_LOW
#define BEE_GDMA_REG_CFGx_L                      CFG_LOW

/* External function declarations */
extern FlagStatus GDMA_GetSuspendChannelStatus(bee_dma_channel_t *GDMA_Channelx);

/* Interrupt status macros */
#define DMA_GET_ERROR_INT_STATUS(channel_num)    (!!(GDMA_BASE->STATUS_ERR & BIT(channel_num)))
#define DMA_GET_BLOCK_INT_STATUS(channel_num)    (!!(GDMA_BASE->STATUS_BLOCK & BIT(channel_num)))

/* DMA API macros */
#define BEE_DMA_INIT(channel, struct)            GDMA_Init(channel, struct)
#define BEE_DMA_CMD(channel_num, cmd)            GDMA_Cmd(channel_num, cmd)
#define BEE_DMA_SUSPEND_CMD(channel_num, cmd)    GDMA_SuspendCmd(channel_num, cmd)
#define BEE_DMA_GET_TRANSFER_LEN(channel)        GDMA_GetTransferLen(channel)
#define BEE_DMA_SET_BUFFER_SIZE(channel, size)   GDMA_SetBufferSize(channel, size)
#define BEE_DMA_SET_SOURCE_ADDRESS(channel, src) GDMA_SetSourceAddress(channel, src)
#define BEE_DMA_SET_DESTINATION_ADDRESS(channel, src)   GDMA_SetDestinationAddress(channel, src)
#define BEE_DMA_INT_CONFIG(channel_num, int, cmd)       GDMA_INTConfig(channel_num, int, cmd)
#define BEE_DMA_CLEAR_INT_PENDING_BIT(channel_num, int) GDMA_ClearINTPendingBit(channel_num, int)
#define BEE_DMA_GET_SUSPEND_CHANNEL_STATUS(channel)     GDMA_GetSuspendChannelStatus(channel)
#define BEE_DMA_SET_LLP_ADDRESS(channel, addr)          GDMA_SetLLPAddress(channel, addr)
#define BEE_DMA_GET_TRANSFER_INT_STATUS(channel_num)    GDMA_GetTransferINTStatus(channel_num)
#define BEE_DMA_GET_SUSPEND_STATUS(channel) (((bee_dma_channel_t *)(channel))->CFG_LOW & BIT8)

/* Address conversion */
#define DMA_BEE_ADDR(addr)                                                                         \
	(FMC_IS_SPIC0_CACHEABLE_ADDR(addr) ? (FMC_MAIN0_NON_CACHE_ADDR((uint32_t)(addr)))          \
					   : (uint32_t)(addr))
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
/* DMA channel multi-block mode support */
#define DMA_HAS_MULTI_BLOCK_MODE(id)                                                               \
	((id) == 0 || (id) == 1 || (id) == 2 || (id) == 3 || (id) == 4 || (id) == 5)

/* Type definitions */
typedef DMA_ChannelTypeDef bee_dma_channel_t;
typedef DMA_InitTypeDef bee_dma_init_t;
typedef DMA_TypeDef bee_dma_type_t;
typedef DMA_LLIDef bee_dma_lli_t;
typedef uint32_t bee_dma_data_size_t;
typedef uint32_t bee_dma_msize_t;

/* Init structure members */
#define BEE_DMA_CHANNEL_NUM         DMA_ChannelNum
#define BEE_DMA_STRUCT_INIT         DMA_StructInit

/* DMA configuration */
#define BEE_DMA_DIRECTION           DMA_Direction
#define BEE_DMA_BUFFER_SIZE         DMA_BufferSize
#define BEE_DMA_SOURCE_HANDSHAKE    DMA_SourceHandshake
#define BEE_DMA_DEST_HANDSHAKE      DMA_DestHandshake
#define BEE_DMA_SOURCE_DATA_SIZE    DMA_SourceDataSize
#define BEE_DMA_DEST_DATA_SIZE      DMA_DestinationDataSize
#define BEE_DMA_SOURCE_INC          DMA_SourceInc
#define BEE_DMA_DEST_INC            DMA_DestinationInc
#define BEE_DMA_SOURCE_MSIZE        DMA_SourceMsize
#define BEE_DMA_DEST_MSIZE          DMA_DestinationMsize
#define BEE_DMA_SOURCE_ADDR         DMA_SourceAddr
#define BEE_DMA_DEST_ADDR           DMA_DestinationAddr
#define BEE_DMA_CHANNEL_PRIORITY    DMA_ChannelPriority
#define BEE_DMA_MULTI_BLOCK_EN      DMA_MultiBlockEn
#define BEE_DMA_MULTI_BLOCK_STRUCT  DMA_MultiBlockStruct
#define BEE_DMA_MULTI_BLOCK_MODE    DMA_MultiBlockMode
#define BEE_DMA_MSIZE(n)            DMA_MSIZE_##n
#define BEE_DMA_DATA_SIZE_BYTE      DMA_DATA_SIZE_BYTE
#define BEE_DMA_DATA_SIZE_HALF_WORD DMA_DATA_SIZE_HALFWORD
#define BEE_DMA_DATA_SIZE_WORD      DMA_DATA_SIZE_WORD
#define BEE_DMA_SRC_ADDR_ADJ        DMA_SourceInc
#define BEE_DMA_DEST_ADDR_ADJ       DMA_DestInc

/* Interrupt and status */
#define BEE_DMA_INT_ERROR           DMA_INT_ERROR
#define BEE_DMA_INT_TRANSFER        DMA_INT_TRANSFER
#define BEE_DMA_INT_BLOCK           DMA_INT_BLOCK

/* Register access */
#define BEE_DMA_REG_STATUS_ERR      DMA_STATUSERR_L
#define BEE_DMA_REG_STATUS_TFR      DMA_STATUSTFR_L
#define BEE_DMA_REG_STATUS_BLOCK    DMA_MASKBLOCK_L
#define BEE_GDMA_REG_CTLx_L         DMA_CTLx_L
#define BEE_GDMA_REG_CFGx_L         DMA_CFGx_L

/* External function declarations */
extern DMA_TypeDef *DMA_GetDMAxByCh(uint8_t DMA_ChannelNum);
extern uint8_t DMA_GetDMAChNumByCh(uint8_t DMA_ChannelNum);
extern FlagStatus DMA_GetSuspendChannelStatus(bee_dma_channel_t *GDMA_Channelx);

/* Interrupt status macros */
#define DMA_GET_ERROR_INT_STATUS(channel_num)                                                      \
	(!!(DMA_GetDMAxByCh(channel_num)->DMA_STATUSERR_L & BIT(DMA_GetDMAChNumByCh(channel_num))))

#define DMA_GET_BLOCK_INT_STATUS(channel_num)                                                      \
	(!!(DMA_GetDMAxByCh(channel_num)->DMA_STATUSBLOCK_L &                                      \
	    BIT(DMA_GetDMAChNumByCh(channel_num))))

/* DMA API macros */
#define BEE_DMA_INIT(channel, struct)                   DMA_Init(channel, struct)
#define BEE_DMA_CMD(channel_num, cmd)                   DMA_Cmd(channel_num, cmd)
#define BEE_DMA_SUSPEND_CMD(channel_num, cmd)           DMA_SuspendCmd(channel_num, cmd)
#define BEE_DMA_GET_TRANSFER_LEN(channel)               DMA_GetTransferLen(channel)
#define BEE_DMA_SET_BUFFER_SIZE(channel, size)          DMA_SetBufferSize(channel, size)
#define BEE_DMA_SET_SOURCE_ADDRESS(channel, src)        DMA_SetSourceAddress(channel, src)
#define BEE_DMA_SET_DESTINATION_ADDRESS(channel, src)   DMA_SetDestinationAddress(channel, src)
#define BEE_DMA_INT_CONFIG(channel_num, int, cmd)       DMA_INTConfig(channel_num, int, cmd)
#define BEE_DMA_CLEAR_INT_PENDING_BIT(channel_num, int) DMA_ClearINTPendingBit(channel_num, int)
#define BEE_DMA_GET_SUSPEND_CHANNEL_STATUS(channel)     DMA_GetSuspendChannelStatus(channel)
#define BEE_DMA_SET_LLP_ADDRESS(channel, addr)          DMA_SetLLPAddress(channel, addr)
#define BEE_DMA_GET_TRANSFER_INT_STATUS(channel_num)    DMA_GetTransferINTStatus(channel_num)
#define BEE_DMA_GET_SUSPEND_STATUS(channel) (((bee_dma_channel_t *)(channel))->DMA_CFGx_L & BIT8)

/* Address conversion */
#define DMA_BEE_ADDR(addr)                  ((uint32_t)addr)
#endif

BUILD_ASSERT(CONFIG_DMA_BEE_LLI_POOL_COUNT > 0, "DMA LLI pool count must be > 0");

SYS_MEM_BLOCKS_DEFINE(dma_bee_lli_pool, BIT(LOG2CEIL(sizeof(bee_dma_lli_t))), 4,
		      CONFIG_DMA_BEE_LLI_POOL_COUNT);

static struct k_spinlock dma_bee_lli_lock;

struct dma_bee_config {
	uint32_t reg;
	uint32_t channels;
	uint16_t clkid;
	void (*irq_configure)(void);
	struct {
		volatile uint32_t channel_base;
		uint8_t channel_num;
	} channel_table[];
};

struct dma_bee_channel {
	dma_callback_t callback;
	void *user_data;
	bool busy;
	struct dma_config cfg;
	bool cyclic;
	uint32_t total_size;
	bee_dma_lli_t *dma_lli[CONFIG_DMA_BEE_MAX_BLOCKS_PER_CHANNEL];
	uint32_t allocated_lli_count;
};

struct dma_bee_isr_param {
	const struct device *dev;
	uint8_t channel_id;
};

struct dma_bee_data {
	/* this needs to be the first member */
	struct dma_context ctx;
	atomic_t channel_flags;
	struct dma_bee_channel *channels;
};

static void dma_bee_reset_block_transfer(bee_dma_channel_t *dma_channel)
{
	dma_channel->BEE_GDMA_REG_CTLx_L |= BIT27 | BIT28;
	dma_channel->BEE_GDMA_REG_CFGx_L &= ~(BIT30 | BIT31);
}

static int dma_bee_get_width(uint32_t size, void *width)
{
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	bee_dma_data_size_t *width_ptr = (bee_dma_data_size_t *)width;
#elif defined(CONFIG_SOC_SERIES_RTL8752H) || defined(CONFIG_SOC_SERIES_RTL87X2J)
	uint32_t *width_ptr = (uint32_t *)width;
#endif

	switch (size) {
	case 1:
		*width_ptr = BEE_DMA_DATA_SIZE_BYTE;
		break;
	case 2:
		*width_ptr = BEE_DMA_DATA_SIZE_HALF_WORD;
		break;
	case 4:
		*width_ptr = BEE_DMA_DATA_SIZE_WORD;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dma_bee_get_msize(uint32_t burst_len, void *msize)
{
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	bee_dma_msize_t *msize_ptr = (bee_dma_msize_t *)msize;
#elif defined(CONFIG_SOC_SERIES_RTL8752H) || defined(CONFIG_SOC_SERIES_RTL87X2J)
	uint32_t *msize_ptr = (uint32_t *)msize;
#endif

	switch (burst_len) {
	case 1:
		*msize_ptr = BEE_DMA_MSIZE(1);
		break;
	case 4:
		*msize_ptr = BEE_DMA_MSIZE(4);
		break;
	case 8:
		*msize_ptr = BEE_DMA_MSIZE(8);
		break;
	case 16:
		*msize_ptr = BEE_DMA_MSIZE(16);
		break;
	case 32:
		*msize_ptr = BEE_DMA_MSIZE(32);
		break;
	case 64:
		*msize_ptr = BEE_DMA_MSIZE(64);
		break;
	case 128:
		*msize_ptr = BEE_DMA_MSIZE(128);
		break;
	case 256:
		*msize_ptr = BEE_DMA_MSIZE(256);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dma_bee_check_dma_config(const struct dma_bee_config *cfg, uint32_t channel,
				    struct dma_config *dma_cfg, int dma_channel_num)
{
	if (channel >= cfg->channels) {
		LOG_ERR("Channel must be < %d (%d)", cfg->channels, channel);
		return -EINVAL;
	}

	if (!DMA_HAS_MULTI_BLOCK_MODE(dma_channel_num) && dma_cfg->block_count != 1) {
		LOG_ERR("DMA channel%d does not support chained block transfer", dma_channel_num);
		return -ENOTSUP;
	}

	if (dma_cfg->source_chaining_en || dma_cfg->dest_chaining_en) {
		LOG_ERR("Src/dest chaining not supported");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->source_gather_count != 0 ||
	    dma_cfg->head_block->dest_scatter_count != 0) {
		LOG_ERR("Gather/scatter not supported");
		return -ENOTSUP;
	}

	if (dma_cfg->channel_priority > 9) {
		LOG_ERR("Channel_priority must be < 9 (%d)", dma_cfg->channel_priority);
		return -EINVAL;
	}

	return 0;
}

static int configure_multi_block(struct dma_bee_data *data, uint32_t channel,
				 struct dma_config *dma_cfg, bee_dma_init_t *init_struct)
{
	struct dma_block_config *cur_block = dma_cfg->head_block;
	bee_dma_lli_t **lli_ptr_array;
	uint32_t next_lli;
	uint32_t llp_selected;
	int ret;
	k_spinlock_key_t key;

	if (dma_cfg->block_count > CONFIG_DMA_BEE_MAX_BLOCKS_PER_CHANNEL) {
		LOG_ERR("Block count %d exceeds driver limit %d", dma_cfg->block_count,
			CONFIG_DMA_BEE_MAX_BLOCKS_PER_CHANNEL);
		return -EINVAL;
	}

	key = k_spin_lock(&dma_bee_lli_lock);

	if (data->channels[channel].allocated_lli_count > 0) {
		ret = sys_mem_blocks_free(&dma_bee_lli_pool,
					  data->channels[channel].allocated_lli_count,
					  (void **)data->channels[channel].dma_lli);
		if (ret < 0) {
			LOG_ERR("Failed to free old LLIs");
		}
		data->channels[channel].allocated_lli_count = 0;
	}

	ret = sys_mem_blocks_alloc(&dma_bee_lli_pool, dma_cfg->block_count,
				   (void **)data->channels[channel].dma_lli);

	k_spin_unlock(&dma_bee_lli_lock, key);

	if (ret < 0) {
		LOG_ERR("Failed to allocate %d LLIs", dma_cfg->block_count);
		return -ENOMEM;
	}

	data->channels[channel].allocated_lli_count = dma_cfg->block_count;
	lli_ptr_array = data->channels[channel].dma_lli;

	init_struct->BEE_DMA_MULTI_BLOCK_EN = ENABLE;
	init_struct->BEE_DMA_MULTI_BLOCK_MODE = LLI_TRANSFER;
	init_struct->BEE_DMA_MULTI_BLOCK_STRUCT = (uint32_t)lli_ptr_array[0];

	data->channels[channel].total_size = 0;

	for (uint8_t i = 0; i < dma_cfg->block_count; i++) {
		if (cur_block == NULL) {
			LOG_ERR("Block%d does not exist", i);
			return -EINVAL;
		}

		lli_ptr_array[i]->SAR = DMA_BEE_ADDR(cur_block->source_address);
		lli_ptr_array[i]->DAR = DMA_BEE_ADDR(cur_block->dest_address);

		if (i < dma_cfg->block_count - 1) {
			next_lli = (uint32_t)(lli_ptr_array[i + 1]);
		} else if (dma_cfg->cyclic) {
			next_lli = (uint32_t)(lli_ptr_array[0]);
		} else {
			next_lli = 0;
		}

		lli_ptr_array[i]->LLP = next_lli;

		/* Enable LLP only when there is a valid next LLI to follow:
		 * either the next block in the list, or the first block again in cyclic mode.
		 */
		if (dma_cfg->cyclic || (i < dma_cfg->block_count - 1)) {
			llp_selected = (init_struct->BEE_DMA_MULTI_BLOCK_MODE & LLP_SELECTED_BIT);
		} else {
			llp_selected = 0;
		}

		lli_ptr_array[i]->CTL_LOW = BIT(0) | (init_struct->BEE_DMA_DEST_DATA_SIZE << 1) |
					    (init_struct->BEE_DMA_SOURCE_DATA_SIZE << 4) |
					    (cur_block->dest_addr_adj << 7) |
					    (cur_block->source_addr_adj << 9) |
					    (init_struct->BEE_DMA_DEST_MSIZE << 11) |
					    (init_struct->BEE_DMA_SOURCE_MSIZE << 14) |
					    (dma_cfg->channel_direction << 20) | llp_selected;

		lli_ptr_array[i]->CTL_HIGH = cur_block->block_size / dma_cfg->source_data_size;

		if (dma_cfg->cyclic) {
			data->channels[channel].total_size = cur_block->block_size;
		} else {
			data->channels[channel].total_size += cur_block->block_size;
		}

		cur_block = cur_block->next_block;
	}

	data->channels[channel].cyclic = dma_cfg->cyclic;

	return 0;
}

static int dma_bee_configure(const struct device *dev, uint32_t channel, struct dma_config *dma_cfg)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	int dma_channel_num;
	bee_dma_channel_t *dma_channel;
	bee_dma_init_t dma_init_struct;
	int ret;

	if (channel >= cfg->channels) {
		LOG_ERR("Channel must be < %d (%d)", cfg->channels, channel);
		return -EINVAL;
	}

	dma_channel_num = cfg->channel_table[channel].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[channel].channel_base;

	LOG_DBG("Channel=%d, channel_direction=%d, block_size=%d, "
		"source_addr_adj=%d dest_addr_adj=%d, "
		"source_data_size=%d, dest_data_size=%d, source_burst_length=%d, "
		"dest_burst_length=%d, dma_cfg->dma_slot=%d, "
		"source_address=%x, dest_address=%x, line%d",
		channel, dma_cfg->channel_direction, dma_cfg->head_block->block_size,
		dma_cfg->head_block->source_addr_adj, dma_cfg->head_block->dest_addr_adj,
		dma_cfg->source_data_size, dma_cfg->dest_data_size, dma_cfg->source_burst_length,
		dma_cfg->dest_burst_length, dma_cfg->dma_slot, dma_cfg->head_block->source_address,
		dma_cfg->head_block->dest_address, __LINE__);

	ret = dma_bee_check_dma_config(cfg, channel, dma_cfg, dma_channel_num);
	if (ret < 0) {
		return ret;
	}

	BEE_DMA_CMD(dma_channel_num, DISABLE);
	BEE_DMA_STRUCT_INIT(&dma_init_struct);

	if (dma_bee_get_width(dma_cfg->source_data_size,
			      &dma_init_struct.BEE_DMA_SOURCE_DATA_SIZE) < 0) {
		LOG_ERR("Invalid source_data_size %d", dma_cfg->source_data_size);
		return -EINVAL;
	}

	if (dma_bee_get_width(dma_cfg->dest_data_size, &dma_init_struct.BEE_DMA_DEST_DATA_SIZE) <
	    0) {
		LOG_ERR("Invalid dest_data_size %d", dma_cfg->dest_data_size);
		return -EINVAL;
	}

	if (dma_bee_get_msize(dma_cfg->source_burst_length, &dma_init_struct.BEE_DMA_SOURCE_MSIZE) <
	    0) {
		LOG_ERR("Invalid source_burst_length %d", dma_cfg->source_burst_length);
		return -EINVAL;
	}

	if (dma_bee_get_msize(dma_cfg->dest_burst_length, &dma_init_struct.BEE_DMA_DEST_MSIZE) <
	    0) {
		LOG_ERR("Invalid dest_burst_length %d", dma_cfg->dest_burst_length);
		return -EINVAL;
	}

	dma_init_struct.BEE_DMA_CHANNEL_NUM = dma_channel_num;
	dma_init_struct.BEE_DMA_DIRECTION = dma_cfg->channel_direction;
	dma_init_struct.BEE_DMA_BUFFER_SIZE =
		dma_cfg->head_block->block_size / dma_cfg->source_data_size;

	if (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
		dma_init_struct.BEE_DMA_DEST_HANDSHAKE = dma_cfg->dma_slot;
	} else if (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY) {
		dma_init_struct.BEE_DMA_SOURCE_HANDSHAKE = dma_cfg->dma_slot;
	}

	dma_init_struct.BEE_DMA_SOURCE_INC = dma_cfg->head_block->source_addr_adj;
	dma_init_struct.BEE_DMA_DEST_INC = dma_cfg->head_block->dest_addr_adj;
	dma_init_struct.BEE_DMA_SOURCE_ADDR = DMA_BEE_ADDR(dma_cfg->head_block->source_address);
	dma_init_struct.BEE_DMA_DEST_ADDR = DMA_BEE_ADDR(dma_cfg->head_block->dest_address);
	dma_init_struct.BEE_DMA_CHANNEL_PRIORITY = dma_cfg->channel_priority;

	if (DMA_HAS_MULTI_BLOCK_MODE(dma_channel_num)) {
		ret = configure_multi_block(data, channel, dma_cfg, &dma_init_struct);
		if (ret < 0) {
			return ret;
		}
	} else {
		dma_init_struct.BEE_DMA_MULTI_BLOCK_EN = DISABLE;
		data->channels[channel].total_size = dma_cfg->head_block->block_size;
	}

	BEE_DMA_INIT(dma_channel, &dma_init_struct);
	data->channels[channel].callback = dma_cfg->dma_callback;
	data->channels[channel].user_data = dma_cfg->user_data;
	data->channels[channel].cfg = *dma_cfg;

	return 0;
}

static int dma_bee_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			  size_t size)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	int dma_channel_num;
	bee_dma_channel_t *dma_channel;

	if (channel >= cfg->channels) {
		LOG_ERR("Reload channel must be < %d (%d)", cfg->channels, channel);
		return -EINVAL;
	}

	dma_channel_num = cfg->channel_table[channel].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[channel].channel_base;

	BEE_DMA_CMD(dma_channel_num, DISABLE);

	data->channels[channel].busy = false;

	if (!DMA_HAS_MULTI_BLOCK_MODE(dma_channel_num)) {
		BEE_DMA_SET_BUFFER_SIZE(dma_channel, size);
		BEE_DMA_SET_SOURCE_ADDRESS(dma_channel, DMA_BEE_ADDR(src));
		BEE_DMA_SET_DESTINATION_ADDRESS(dma_channel, DMA_BEE_ADDR(dst));

		data->channels[channel].total_size = size;
	} else {
		if (data->channels[channel].allocated_lli_count == 0) {
			LOG_ERR("Configure dma before reload");
			return -EINVAL;
		}

		data->channels[channel].dma_lli[0]->SAR = DMA_BEE_ADDR(src);
		data->channels[channel].dma_lli[0]->DAR = DMA_BEE_ADDR(dst);
		data->channels[channel].dma_lli[0]->LLP = 0;
		if (data->channels[channel].cfg.channel_direction == PERIPHERAL_TO_MEMORY) {
			data->channels[channel].dma_lli[0]->CTL_HIGH =
				(size / data->channels[channel].cfg.dest_data_size);
		} else {
			data->channels[channel].dma_lli[0]->CTL_HIGH =
				(size / data->channels[channel].cfg.source_data_size);
		}

		data->channels[channel].total_size = size;
	}

	return 0;
}

static int dma_bee_start(const struct device *dev, uint32_t channel)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	int dma_channel_num;
	bee_dma_channel_t *dma_channel;

	if (channel >= cfg->channels) {
		LOG_ERR("Start channel must be < %d (%d)", cfg->channels, channel);
		return -EINVAL;
	}

	dma_channel_num = cfg->channel_table[channel].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[channel].channel_base;

	if (!data->channels[channel].cfg.error_callback_dis) {
		BEE_DMA_INT_CONFIG(dma_channel_num, BEE_DMA_INT_ERROR, ENABLE);
	}

	if (!DMA_HAS_MULTI_BLOCK_MODE(dma_channel_num)) {
		BEE_DMA_INT_CONFIG(dma_channel_num, BEE_DMA_INT_TRANSFER, ENABLE);
		data->channels[channel].busy = true;
		BEE_DMA_CMD(dma_channel_num, ENABLE);
	} else {
		BEE_DMA_INT_CONFIG(dma_channel_num, BEE_DMA_INT_BLOCK | BEE_DMA_INT_TRANSFER,
				   ENABLE);

		data->channels[channel].busy = true;

		BEE_DMA_SET_BUFFER_SIZE(dma_channel, data->channels[channel].dma_lli[0]->CTL_HIGH);
		BEE_DMA_SET_LLP_ADDRESS(dma_channel,
					(uint32_t)(data->channels[channel].dma_lli[0]));
		dma_bee_reset_block_transfer(dma_channel);

		BEE_DMA_SET_SOURCE_ADDRESS(dma_channel, 0);
		BEE_DMA_SET_DESTINATION_ADDRESS(dma_channel, 0);

		BEE_DMA_CMD(dma_channel_num, ENABLE);
	}

	return 0;
}

static int dma_bee_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	int dma_channel_num;
	bee_dma_channel_t *dma_channel;

	if (channel >= cfg->channels) {
		LOG_ERR("Stop channel must be < %d (%d)", cfg->channels, channel);
		return -EINVAL;
	}

	dma_channel_num = cfg->channel_table[channel].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[channel].channel_base;

	if (!DMA_HAS_MULTI_BLOCK_MODE(dma_channel_num)) {
		BEE_DMA_INT_CONFIG(dma_channel_num, BEE_DMA_INT_TRANSFER | BEE_DMA_INT_ERROR,
				   DISABLE);
		BEE_DMA_CLEAR_INT_PENDING_BIT(dma_channel_num,
					      BEE_DMA_INT_TRANSFER | BEE_DMA_INT_ERROR);
	} else {
		BEE_DMA_INT_CONFIG(dma_channel_num,
				   BEE_DMA_INT_TRANSFER | BEE_DMA_INT_ERROR | BEE_DMA_INT_BLOCK,
				   DISABLE);
		BEE_DMA_CLEAR_INT_PENDING_BIT(dma_channel_num, BEE_DMA_INT_TRANSFER |
								       BEE_DMA_INT_ERROR |
								       BEE_DMA_INT_BLOCK);
	}

	BEE_DMA_CMD(dma_channel_num, DISABLE);
	data->channels[channel].busy = false;

	return 0;
}

static int dma_bee_suspend(const struct device *dev, uint32_t channel)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	int dma_channel_num;
	bee_dma_channel_t *dma_channel;

	if (channel >= cfg->channels) {
		LOG_ERR("Suspend channel must be < %d (%d)", cfg->channels, channel);
		return -EINVAL;
	}

	dma_channel_num = cfg->channel_table[channel].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[channel].channel_base;

	if (!data->channels[channel].busy) {
		LOG_ERR("Suspend channel not busy");
		return -EINVAL;
	}

	BEE_DMA_SUSPEND_CMD(dma_channel, ENABLE);

	return 0;
}

static int dma_bee_resume(const struct device *dev, uint32_t channel)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	int dma_channel_num;
	bee_dma_channel_t *dma_channel;

	if (channel >= cfg->channels) {
		LOG_ERR("Resume channel must be < %d (%d)", cfg->channels, channel);
		return -EINVAL;
	}

	dma_channel_num = cfg->channel_table[channel].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[channel].channel_base;

	if (!data->channels[channel].busy) {
		LOG_ERR("Resume channel not busy");
		return -EINVAL;
	}

	if (!BEE_DMA_GET_SUSPEND_CHANNEL_STATUS(dma_channel)) {
		LOG_ERR("Resume channel not suspend");
		return -EINVAL;
	}

	BEE_DMA_SUSPEND_CMD(dma_channel, DISABLE);

	return 0;
}

static int dma_bee_get_status(const struct device *dev, uint32_t ch, struct dma_status *stat)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	bool suspending;
	int dma_channel_num;
	bee_dma_channel_t *dma_channel;
	uint32_t timeout_cnt = 10000;

	if (ch >= cfg->channels) {
		LOG_ERR("Channel must be < %d (%d)", cfg->channels, ch);
		return -EINVAL;
	}

	dma_channel_num = cfg->channel_table[ch].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[ch].channel_base;
	suspending = BEE_DMA_GET_SUSPEND_STATUS(dma_channel);

	stat->busy = data->channels[ch].busy;
	if (data->channels[ch].busy) {
		BEE_DMA_SUSPEND_CMD(dma_channel, ENABLE);
		while (!BEE_DMA_GET_SUSPEND_CHANNEL_STATUS(dma_channel)) {
			if (--timeout_cnt == 0) {
				LOG_ERR("Get status timeout on channel %d", ch);
				break;
			}
		}

		stat->pending_length =
			data->channels[ch].total_size - BEE_DMA_GET_TRANSFER_LEN(dma_channel);

		if (!suspending) {
			BEE_DMA_SUSPEND_CMD(dma_channel, DISABLE);
		}
	} else {
		stat->pending_length = 0;
	}

	stat->dir = data->channels[ch].cfg.channel_direction;

	return 0;
}

static bool dma_bee_api_chan_filter(const struct device *dev, int ch, void *filter_param)
{
	uint32_t filter;

	ARG_UNUSED(dev);

	if (filter_param == NULL) {
		return true;
	}

	filter = *(uint32_t *)filter_param;

	return (filter & BIT(ch)) != 0U;
}

static int dma_bee_init(const struct device *dev)
{
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	int dma_channel_num;

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	for (uint32_t i = 0; i < cfg->channels; i++) {
		dma_channel_num = cfg->channel_table[i].channel_num;
		if (dma_channel_num >= 0) {
			BEE_DMA_INT_CONFIG(dma_channel_num,
					   BEE_DMA_INT_TRANSFER | BEE_DMA_INT_ERROR |
						   BEE_DMA_INT_BLOCK,
					   DISABLE);
			BEE_DMA_CMD(dma_channel_num, DISABLE);
		}
	}

	cfg->irq_configure();

	data->channel_flags = ATOMIC_INIT(0);
	data->ctx.atomic = &data->channel_flags;
	data->ctx.dma_channels = cfg->channels;
	data->ctx.magic = DMA_MAGIC;

	return 0;
}

static void dma_bee_isr(struct dma_bee_isr_param *param)
{
	const struct device *dev = param->dev;
	const struct dma_bee_config *cfg = dev->config;
	struct dma_bee_data *data = dev->data;
	uint8_t i = param->channel_id;
	int dma_channel_num;
	uint32_t errflag, ftfflag, blockflag;
	int status = DMA_STATUS_COMPLETE;
	bee_dma_channel_t *dma_channel;

	dma_channel_num = cfg->channel_table[i].channel_num;
	dma_channel = (bee_dma_channel_t *)cfg->channel_table[i].channel_base;
	errflag = DMA_GET_ERROR_INT_STATUS(dma_channel_num);
	ftfflag = BEE_DMA_GET_TRANSFER_INT_STATUS(dma_channel_num);
	blockflag = DMA_GET_BLOCK_INT_STATUS(dma_channel_num);

	LOG_DBG("Channel=%d transferlen=%d callback=0x%p ftfflag=%d errflag=%d blockflag=%d "
		"complete_callback_en=%d",
		i, BEE_DMA_GET_TRANSFER_LEN(dma_channel), data->channels[i].callback, ftfflag,
		errflag, blockflag, data->channels[i].cfg.complete_callback_en);

	if (!DMA_HAS_MULTI_BLOCK_MODE(dma_channel_num)) {
		if (errflag == 0 && ftfflag == 0) {
			return;
		}

		BEE_DMA_CLEAR_INT_PENDING_BIT(dma_channel_num, BEE_DMA_INT_TRANSFER);

		if (errflag) {
			BEE_DMA_CLEAR_INT_PENDING_BIT(dma_channel_num, BEE_DMA_INT_ERROR);
			status = -EIO;
		}

		data->channels[i].busy = false;

		if (data->channels[i].callback) {
			data->channels[i].callback(dev, data->channels[i].user_data, i, status);
		}
	} else {
		if (errflag == 0 && ftfflag == 0 && blockflag == 0) {
			return;
		}

		if (errflag) {
			BEE_DMA_CLEAR_INT_PENDING_BIT(dma_channel_num, BEE_DMA_INT_ERROR);
			status = -EIO;
		}

		if (ftfflag) {
			BEE_DMA_CLEAR_INT_PENDING_BIT(dma_channel_num, BEE_DMA_INT_TRANSFER);
			data->channels[i].busy = false;
		}

		if (blockflag) {
			BEE_DMA_CLEAR_INT_PENDING_BIT(dma_channel_num, BEE_DMA_INT_BLOCK);
			if (!data->channels[i].cyclic) {
				data->channels[i].total_size -=
					BEE_DMA_GET_TRANSFER_LEN(dma_channel);
			}

			if (data->channels[i].cfg.complete_callback_en && !errflag) {
				status = DMA_STATUS_BLOCK;
			}
		}

		if (data->channels[i].callback) {
			if (ftfflag || errflag ||
			    (blockflag && data->channels[i].cfg.complete_callback_en)) {
				data->channels[i].callback(dev, data->channels[i].user_data, i,
							   status);
			}
		}
	}
}

static DEVICE_API(dma, dma_bee_driver_api) = {
	.config = dma_bee_configure,
	.reload = dma_bee_reload,
	.start = dma_bee_start,
	.stop = dma_bee_stop,
	.suspend = dma_bee_suspend,
	.resume = dma_bee_resume,
	.get_status = dma_bee_get_status,
	.chan_filter = dma_bee_api_chan_filter,
};

#define IRQ_CONFIGURE(n, index)                                                                    \
	irq_disable(DT_INST_IRQ_BY_IDX(index, n, irq));                                            \
	irq_connect_dynamic(DT_INST_IRQ_BY_IDX(index, n, irq),                                     \
			    DT_INST_IRQ_BY_IDX(index, n, priority), (const void *)dma_bee_isr,     \
			    &dma_bee_##index##_isr_param[n], 0);                                   \
	irq_enable(DT_INST_IRQ_BY_IDX(index, n, irq));

#define CONFIGURE_ALL_IRQS(index, n) LISTIFY(n, IRQ_CONFIGURE, (), index)

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define DMA_CHANNEL_TABLE                                                                          \
	.channel_table = {                                                                         \
		{GDMA0_Channel0_BASE, GDMA_CH_NUM0}, {GDMA0_Channel1_BASE, GDMA_CH_NUM1},          \
		{GDMA0_Channel2_BASE, GDMA_CH_NUM2}, {GDMA0_Channel3_BASE, GDMA_CH_NUM3},          \
		{GDMA0_Channel4_BASE, GDMA_CH_NUM4}, {GDMA0_Channel5_BASE, GDMA_CH_NUM5},          \
		{GDMA0_Channel6_BASE, GDMA_CH_NUM6}, {GDMA0_Channel7_BASE, GDMA_CH_NUM7},          \
		{GDMA0_Channel8_BASE, GDMA_CH_NUM8},                                               \
	}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#define DMA_CHANNEL_TABLE                                                                          \
	.channel_table = {                                                                         \
		{GDMA_Channel0_BASE, GDMA_CH_NUM0},                                                \
		{GDMA_Channel1_BASE, GDMA_CH_NUM1},                                                \
		{GDMA_Channel2_BASE, GDMA_CH_NUM2},                                                \
	}
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#define DMA_CHANNEL_TABLE                                                                          \
	.channel_table = {                                                                         \
		{DMA_CH0_BASE, DMA_CH_NUM0}, {DMA_CH1_BASE, DMA_CH_NUM1},                          \
		{DMA_CH2_BASE, DMA_CH_NUM2}, {DMA_CH3_BASE, DMA_CH_NUM3},                          \
		{DMA_CH4_BASE, DMA_CH_NUM4}, {DMA_CH5_BASE, DMA_CH_NUM5},                          \
	}
#endif

#define ALL_ISR_PARAM_CONFIGURE(n, index)                                                          \
	{                                                                                          \
		.dev = DEVICE_DT_INST_GET(index),                                                  \
		.channel_id = n,                                                                   \
	},

#define CONFIGURE_ALL_ISR_PARAMS(index, n) LISTIFY(n, ALL_ISR_PARAM_CONFIGURE, (), index)

#define BEE_DMA_DRIVER_INIT(index)                                                                 \
	static struct dma_bee_isr_param dma_bee_##index##_isr_param[] = {                          \
		CONFIGURE_ALL_ISR_PARAMS(index, DT_NUM_IRQS(DT_DRV_INST(index)))};                 \
	static void dma_bee_##index##_irq_configure(void)                                          \
	{                                                                                          \
		CONFIGURE_ALL_IRQS(index, DT_NUM_IRQS(DT_DRV_INST(index)));                        \
	}                                                                                          \
	static const struct dma_bee_config dma_bee_##index##_config = {                            \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.channels = DT_INST_PROP(index, dma_channels),                                     \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.irq_configure = &dma_bee_##index##_irq_configure,                                 \
		DMA_CHANNEL_TABLE,                                                                 \
	};                                                                                         \
                                                                                                   \
	static struct dma_bee_channel                                                              \
		dma_bee_##index##_channels[DT_INST_PROP(index, dma_channels)];                     \
                                                                                                   \
	static struct dma_bee_data dma_bee_##index##_data = {                                      \
		.channels = dma_bee_##index##_channels,                                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &dma_bee_init, NULL, &dma_bee_##index##_data,                 \
			      &dma_bee_##index##_config, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,   \
			      &dma_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BEE_DMA_DRIVER_INIT)
