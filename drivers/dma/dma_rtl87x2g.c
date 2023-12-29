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

#include <rtl_rcc.h>
#include <rtl_gdma.h>

#include <trace.h>
#define DBG_DIRECT_SHOW 0
LOG_MODULE_REGISTER(dma_rtl87x2g, CONFIG_DMA_LOG_LEVEL);

struct dma_rtl87x2g_config
{
    uint32_t reg;
    uint32_t channels;
    uint16_t clkid;
    void (*irq_configure)(void);
    uint32_t channel_base_table[];
};

struct dma_rtl87x2g_channel
{
    dma_callback_t callback;
    void *user_data;
    uint32_t direction;
    bool busy;
    uint32_t block_size;
};

struct dma_rtl87x2g_data
{
    struct dma_rtl87x2g_channel *channels;
};

struct dma_rtl87x2g_srcdst_config
{
    uint32_t addr;
    uint32_t adj;
    uint32_t width;
};

static int dma_rtl87x2g_ch2num(uint32_t reg, uint32_t ch)
{
    if ((GDMA_TypeDef *)reg == GDMA0)
    {
        return ch;
    }
    else
    {
        return -EIO;
    }
}

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
    dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
    dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

    if (channel >= cfg->channels)
    {
        LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
                cfg->channels, channel);
        return -EINVAL;
    }

    if (dma_cfg->block_count != 1)
    {
        LOG_ERR("chained block transfer not supported.");
        return -ENOTSUP;
    }

    if (dma_cfg->channel_priority > 9)
    {
        LOG_ERR("channel_priority must be < 9 (%" PRIu32 ")",
                dma_cfg->channel_priority);
        return -EINVAL;
    }

    if (dma_cfg->source_data_size != GDMA_DataSize_Byte &&
        dma_cfg->source_data_size != GDMA_DataSize_HalfWord &&
        dma_cfg->source_data_size != GDMA_DataSize_Word)
    {
        LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")",
                dma_cfg->source_data_size);
        return -EINVAL;
    }

    if (dma_cfg->dest_data_size != GDMA_DataSize_Byte &&
        dma_cfg->dest_data_size != GDMA_DataSize_HalfWord &&
        dma_cfg->dest_data_size != GDMA_DataSize_Word)
    {
        LOG_ERR("dest_data_size must be 1, 2, or 4 (%" PRIu32 ")",
                dma_cfg->dest_data_size);
        return -EINVAL;
    }

    if (dma_cfg->channel_direction > PERIPHERAL_TO_MEMORY)
    {
        LOG_ERR("channel_direction must be MEMORY_TO_MEMORY, "
                "MEMORY_TO_PERIPHERAL or PERIPHERAL_TO_MEMORY (%" PRIu32
                ")",
                dma_cfg->channel_direction);
        return -ENOTSUP;
    }

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[dma_rtl87x2g_configure] channel=%d, channel_direction=%d, block_size=%d, source_addr_adj=%d dest_addr_adj=%d, \
source_data_size=%d, dest_data_size=%d, source_burst_length=%d, dest_burst_length=%d, dma_cfg->dma_slot=%d, line%d",
               \
               channel, dma_cfg->channel_direction, dma_cfg->head_block->block_size, \
               dma_cfg->head_block->source_addr_adj, dma_cfg->head_block->dest_addr_adj, \
               dma_cfg->source_data_size, dma_cfg->dest_data_size, \
               dma_cfg->source_burst_length, dma_cfg->dest_burst_length, dma_cfg->dma_slot, __LINE__);
    DBG_DIRECT("[dma_rtl87x2g_configure] channel=%d, source_address=%x, dest_address=%x, line%d", \
               channel, dma_cfg->head_block->source_address, dma_cfg->head_block->dest_address, __LINE__);
#endif
    GDMA_Cmd(dma_channel_num, DISABLE);

    GDMA_InitTypeDef dma_init_struct;
    GDMA_StructInit(&dma_init_struct);

    dma_init_struct.GDMA_ChannelNum = dma_channel_num;
    dma_init_struct.GDMA_DIR = dma_cfg->channel_direction;
    if (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY)
    {
        dma_init_struct.GDMA_BufferSize = dma_cfg->head_block->block_size >> dma_cfg->dest_data_size;
    }
    else
    {
        dma_init_struct.GDMA_BufferSize = dma_cfg->head_block->block_size >> dma_cfg->source_data_size;
    }
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[dma_rtl87x2g_configure] channel=%d, dma_init_struct.GDMA_BufferSize=%d, line%d", \
               channel, dma_init_struct.GDMA_BufferSize, __LINE__);
#endif
    dma_init_struct.GDMA_SourceInc = dma_cfg->head_block->source_addr_adj;
    dma_init_struct.GDMA_DestinationInc = dma_cfg->head_block->dest_addr_adj;
    dma_init_struct.GDMA_SourceDataSize = dma_cfg->source_data_size;
    dma_init_struct.GDMA_DestinationDataSize = dma_cfg->dest_data_size;
    dma_init_struct.GDMA_SourceMsize = dma_cfg->source_burst_length;
    dma_init_struct.GDMA_DestinationMsize = dma_cfg->dest_burst_length;
    dma_init_struct.GDMA_SourceAddr = dma_cfg->head_block->source_address;
    dma_init_struct.GDMA_DestinationAddr = dma_cfg->head_block->dest_address;
    dma_init_struct.GDMA_ChannelPriority = dma_cfg->channel_priority;
    if (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL)
    {
        dma_init_struct.GDMA_DestHandshake = dma_cfg->dma_slot;
    }
    else if (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY)
    {
        dma_init_struct.GDMA_SourceHandshake = dma_cfg->dma_slot;
    }

    GDMA_Init(dma_channel, &dma_init_struct);

    data->channels[channel].callback = dma_cfg->dma_callback;
    data->channels[channel].user_data = dma_cfg->user_data;
    data->channels[channel].direction = dma_cfg->channel_direction;
    data->channels[channel].block_size = dma_cfg->head_block->block_size;

    return 0;
}

static int dma_rtl87x2g_reload(const struct device *dev, uint32_t channel, uint32_t src,
                               uint32_t dst, size_t size)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[dma_rtl87x2g_reload] channel=%d, line%d", channel, __LINE__);
#endif
    const struct dma_rtl87x2g_config *cfg = dev->config;
    struct dma_rtl87x2g_data *data = dev->data;
    int dma_channel_num;
    GDMA_ChannelTypeDef *dma_channel;
    dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
    dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

    if (channel >= cfg->channels)
    {
        LOG_ERR("reload channel must be < %" PRIu32 " (%" PRIu32 ")",
                cfg->channels, channel);
        return -EINVAL;
    }

    if (data->channels[channel].busy)
    {
        return -EBUSY;
    }

    GDMA_Cmd(dma_channel_num, DISABLE);
    GDMA_SetBufferSize(dma_channel, size);
    GDMA_SetSourceAddress(dma_channel, src);
    GDMA_SetDestinationAddress(dma_channel, dst);

    data->channels[channel].block_size = size;

    return 0;
}

static int dma_rtl87x2g_start(const struct device *dev, uint32_t channel)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[dma_rtl87x2g_start] channel=%d, line%d", channel, __LINE__);
#endif
    const struct dma_rtl87x2g_config *cfg = dev->config;
    struct dma_rtl87x2g_data *data = dev->data;
    int dma_channel_num;
    GDMA_ChannelTypeDef *dma_channel;
    dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
    dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

    if (channel >= cfg->channels)
    {
        LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")",
                cfg->channels, channel);
        return -EINVAL;
    }

    GDMA_INTConfig(dma_channel_num, GDMA_INT_Transfer | GDMA_INT_Error, ENABLE);
    data->channels[channel].busy = true;
    GDMA_Cmd(dma_channel_num, ENABLE);

    return 0;
}

static int dma_rtl87x2g_stop(const struct device *dev, uint32_t channel)
{
    const struct dma_rtl87x2g_config *cfg = dev->config;
    struct dma_rtl87x2g_data *data = dev->data;
    int dma_channel_num;
    GDMA_ChannelTypeDef *dma_channel;
    dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, channel);
    dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];

    if (channel >= cfg->channels)
    {
        LOG_ERR("stop channel must be < %" PRIu32 " (%" PRIu32 ")",
                cfg->channels, channel);
        return -EINVAL;
    }

    GDMA_INTConfig(dma_channel_num, GDMA_INT_Transfer | GDMA_INT_Error, DISABLE);
    GDMA_ClearINTPendingBit(dma_channel_num, GDMA_INT_Transfer | GDMA_INT_Error);

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

    if (channel >= cfg->channels)
    {
        LOG_ERR("suspend channel must be < %" PRIu32 " (%" PRIu32 ")",
                cfg->channels, channel);
        return -EINVAL;
    }

    if (!data->channels[channel].busy)
    {
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

    if (channel >= cfg->channels)
    {
        LOG_ERR("resume channel must be < %" PRIu32 " (%" PRIu32 ")",
                cfg->channels, channel);
        return -EINVAL;
    }

    if (!data->channels[channel].busy)
    {
        LOG_ERR("resume channel not busy");
        return -EINVAL;
    }

    if (!GDMA_GetSuspendCmdStatus(dma_channel))
    {
        LOG_ERR("resume channel not suspend");
        return -EINVAL;
    }

    GDMA_SuspendCmd(dma_channel, DISABLE);

    return 0;
}

static int dma_rtl87x2g_get_status(const struct device *dev, uint32_t ch,
                                   struct dma_status *stat)
{
    const struct dma_rtl87x2g_config *cfg = dev->config;
    struct dma_rtl87x2g_data *data = dev->data;
    int dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, ch);
    GDMA_ChannelTypeDef *dma_channel = (GDMA_ChannelTypeDef *)cfg->channel_base_table[dma_channel_num];
    bool suspending = GDMA_GetSuspendChannelStatus(dma_channel);

    if (ch >= cfg->channels)
    {
        LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
                cfg->channels, ch);
        return -EINVAL;
    }

    stat->busy = data->channels[ch].busy;
    if (data->channels[ch].busy)
    {
        GDMA_SuspendCmd(dma_channel, ENABLE);
        stat->pending_length = data->channels[ch].block_size - \
                               GDMA_GetTransferLen(dma_channel);
        if (!suspending)
        {
            GDMA_SuspendCmd(dma_channel, DISABLE);
        }
    }
    else
    {
        stat->pending_length = 0;
    }

    stat->dir = data->channels[ch].direction;

    return 0;
}

static bool dma_rtl87x2g_api_chan_filter(const struct device *dev, int ch,
                                         void *filter_param)
{
    uint32_t filter;

    if (!filter_param)
    {
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

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&cfg->clkid);

    for (uint32_t i = 0; i < cfg->channels; i++)
    {
        dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, i);
        if (dma_channel_num >= 0)
        {
            GDMA_INTConfig(dma_channel_num, GDMA_INT_Transfer, DISABLE);
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
    uint32_t errflag, ftfflag;
    int err = 0;

    for (uint32_t i = 0; i < cfg->channels; i++)
    {
        dma_channel_num = dma_rtl87x2g_ch2num(cfg->reg, i);
        errflag = ((GDMA_TypeDef *)cfg->reg)->GDMA_StatuErr & BIT(dma_channel_num);
        ftfflag = ((GDMA_TypeDef *)cfg->reg)->GDMA_StatusTfr & BIT(dma_channel_num);

        if (errflag == 0 && ftfflag == 0)
        {
            continue;
        }

        if (errflag)
        {
            err = -EIO;
        }

        GDMA_ClearINTPendingBit(dma_channel_num, GDMA_INT_Transfer | GDMA_INT_Error);
        data->channels[i].busy = false;

        if (data->channels[i].callback)
        {
            data->channels[i].callback(
                dev, data->channels[i].user_data, i, err);
        }
    }
}

static const struct dma_driver_api dma_rtl87x2g_driver_api =
{
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

#define IRQ_CONFIGURE(n, index)                                                 \
    IRQ_CONNECT(DT_INST_IRQ_BY_IDX(index, n, irq),                          \
                DT_INST_IRQ_BY_IDX(index, n, priority), dma_rtl87x2g_isr,       \
                DEVICE_DT_INST_GET(index), 0);                              \
    irq_enable(DT_INST_IRQ_BY_IDX(index, n, irq));

#define CONFIGURE_ALL_IRQS(index, n) LISTIFY(n, IRQ_CONFIGURE, (), index)

#define DMA_CHANNER_BASE(n, index, dma_port)                                 \
    (GDMA##dma_port##_Channel##n##_BASE)

#define RTL87X2G_DMA_INIT(index)                                                    \
    static void dma_rtl87x2g_##index##_irq_configure(void)                       \
    {                                                                      \
        CONFIGURE_ALL_IRQS(index, DT_NUM_IRQS(DT_DRV_INST(index)));      \
    }                                                                      \
    static const struct dma_rtl87x2g_config dma_rtl87x2g_##index##_config = {        \
        .reg = DT_INST_REG_ADDR(index),                                 \
               .channels = DT_INST_PROP(index, dma_channels),                  \
                           .clkid = DT_INST_CLOCKS_CELL(index, id),                        \
                                    .irq_configure = dma_rtl87x2g_##index##_irq_configure,               \
                                                     .channel_base_table = {     \
                                                                                 LISTIFY(DT_INST_PROP(index, dma_channels),  \
                                                                                         DMA_CHANNER_BASE, (,), index, DT_INST_PROP(index, dma_port))        \
                                                                           },       \
    };                                                                     \
    \
    static struct dma_rtl87x2g_channel                                         \
    dma_rtl87x2g_##index##_channels[DT_INST_PROP(index, dma_channels)];   \
    ATOMIC_DEFINE(dma_rtl87x2g_atomic##index,                                   \
                  DT_INST_PROP(index, dma_channels));                       \
    static struct dma_rtl87x2g_data dma_rtl87x2g_##index##_data = {                  \
                .channels = dma_rtl87x2g_##index##_channels,                         \
    };                                                                     \
    \
    DEVICE_DT_INST_DEFINE(index, &dma_rtl87x2g_init, NULL,                      \
                          &dma_rtl87x2g_##index##_data,                          \
                          &dma_rtl87x2g_##index##_config, PRE_KERNEL_1,           \
                          CONFIG_DMA_INIT_PRIORITY, &dma_rtl87x2g_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTL87X2G_DMA_INIT)
