/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl87x2g_uart

/**
 * @brief Driver for UART port on RTL87X2G family processor.
 * @note  Please validate for newly added series.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control/rtl87x2g_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma/dma_rtl87x2g.h>
#include <zephyr/drivers/dma.h>
#endif

#include "uart_rtl87x2g.h"
#include <rtl_uart.h>

#include <zephyr/logging/log.h>

#include <trace.h>
#define DBG_DIRECT_SHOW 0
LOG_MODULE_REGISTER(uart_rtl87x2g, CONFIG_UART_LOG_LEVEL);

static const uint32_t RTL_UART_BAUDRATE_TABLE[][3] =
{
    {271, 10, 0x24A}, // 9600
    {150, 8,  0x3EF}, // 19200
    {20, 12,  0x252}, // 115200
    {11,  10, 0x3BB}, // 230400
    {11,  9,  0x084}, // 256000
    {7,   9,  0x3EF}, // 384000
    {6,   9,  0x0AA}, // 460800
    {3,   9,  0x0AA}, // 921600
    {4,   5,  0    }, // 1000000
    {2,   5,  0    }, // 2000000
    {1,   8,  0x292}, // 3000000
};

static uint32_t rtl_cfg2idx_baudrate(uint32_t baudrate)
{
    switch (baudrate)
    {
    case 9600:
        return 0;
    case 19200:
        return 1;
    case 115200:
        return 2;
    case 230400:
        return 3;
    case 256000:
        return 4;
    case 384000:
        return 5;
    case 460800:
        return 6;
    case 921600:
        return 7;
    case 1000000:
        return 8;
    case 2000000:
        return 9;
    case 3000000:
        return 10;

    default:
        return -ENOSYS;
    }
}

static UARTWordLen_TypeDef rtl_cfg2mac_data_bits(uint32_t data_bits)
{
    switch (data_bits)
    {
    case UART_CFG_DATA_BITS_7:
        return UART_WORD_LENGTH_7BIT;
    case UART_CFG_DATA_BITS_8:
        return UART_WORD_LENGTH_8BIT;
    default:
        return -ENOSYS;
    }
}

static int rtl_cfg2mac_stopbits(uint32_t stop_bits)
{
    switch (stop_bits)
    {
    case UART_CFG_STOP_BITS_1:
        return UART_STOP_BITS_1;
    case UART_CFG_STOP_BITS_2:
        return UART_STOP_BITS_2;
    default:
        return -ENOSYS;
    }
}

static UARTParity_TypeDef rtl_cfg2mac_parity(uint32_t parity)
{
    switch (parity)
    {
    case UART_CFG_PARITY_NONE:
        return UART_PARITY_NO_PARTY;
    case UART_CFG_PARITY_ODD:
        return UART_PARITY_ODD;
    case UART_CFG_PARITY_EVEN:
        return UART_PARITY_EVEN;
    default:
        return -ENOSYS;
    }
}

static int uart_rtl87x2g_configure(const struct device *dev,
                                   const struct uart_config *cfg)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;

    int baudrate_idx;
    int wordlen;
    int stopbits;
    int parity;

    baudrate_idx = rtl_cfg2idx_baudrate(cfg->baudrate);
    if (baudrate_idx < 0)
    {
        LOG_ERR("Unspported baudrate: %d", cfg->baudrate);
        return -ENOTSUP;
    }

    wordlen = rtl_cfg2mac_data_bits(cfg->data_bits);
    if (wordlen < 0)
    {
        LOG_ERR("Unspported data_bits: %d", cfg->data_bits);
        return -ENOTSUP;
    }

    stopbits = rtl_cfg2mac_stopbits(cfg->stop_bits);
    if (stopbits < 0)
    {
        LOG_ERR("Unspported stop_bits: %d", cfg->stop_bits);
        return -ENOTSUP;
    }

    parity = rtl_cfg2mac_parity(cfg->parity);
    if (parity < 0)
    {
        LOG_ERR("Unspported parity: %d", cfg->parity);
        return -ENOTSUP;
    }

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_configure] baudrate_idx=%d, wordlen=%d, stopbits=%d, parity=%d, hw_flow_ctrl=%d",
               \
               baudrate_idx, wordlen, stopbits, parity, config->hw_flow_ctrl);
#endif

    UART_InitTypeDef uart_init_struct;
    UART_StructInit(&uart_init_struct);
    uart_init_struct.UART_Div = RTL_UART_BAUDRATE_TABLE[baudrate_idx][0];
    uart_init_struct.UART_Ovsr = RTL_UART_BAUDRATE_TABLE[baudrate_idx][1];
    uart_init_struct.UART_OvsrAdj = RTL_UART_BAUDRATE_TABLE[baudrate_idx][2];
    uart_init_struct.UART_IdleTime = UART_RX_IDLE_1BYTE;
    uart_init_struct.UART_WordLen = wordlen;
    uart_init_struct.UART_StopBits = stopbits;
    uart_init_struct.UART_Parity = parity;
    uart_init_struct.UART_HardwareFlowControl = cfg->flow_ctrl;
    uart_init_struct.UART_RxThdLevel = 10;
    uart_init_struct.UART_TxThdLevel = UART_TX_FIFO_SIZE / 2;

#ifdef CONFIG_UART_ASYNC_API
    uart_init_struct.UART_DmaEn = ENABLE;
#endif

    UART_Init(uart, &uart_init_struct);

#if DBG_DIRECT_SHOW
    for (uint32_t i = 0; i < 0x5c / 4; i++)
    {
        // DBG_DIRECT("0x%x = 0x%x",((uint32_t *)uart+i),*((uint32_t *)uart+i));
    }
#endif
    data->uart_config = *cfg;
    return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_rtl87x2g_config_get(const struct device *dev,
                                    struct uart_config *cfg)
{
    struct uart_rtl87x2g_data *data = dev->data;

    *cfg = data->uart_config;

    return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_rtl87x2g_poll_in(const struct device *dev, unsigned char *c)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;

    if (!UART_GetFlagStatus(uart, UART_FLAG_RX_DATA_AVA))
    {
        return -1;
    }
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_poll_in] c=%c", *c);
#endif

    *c = (unsigned char)UART_ReceiveByte(uart);

    return 0;
}

static void uart_rtl87x2g_poll_out(const struct device *dev,
                                   unsigned char c)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;

    while (!(UART_GetTxFIFODataLen(uart) < UART_TX_FIFO_SIZE))
    {
    }

#if DBG_DIRECT_SHOW
    // DBG_DIRECT("[uart_rtl87x2g_poll_out] uart=0x%x, c=%c", (uint32_t)uart, c);
#endif

    UART_SendByte(uart, (uint8_t)c);
}

static int uart_rtl87x2g_err_check(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;

    uint32_t err = 0U;

    if (UART_GetFlagStatus(uart, UART_FLAG_RX_OVERRUN))
    {
        err |= UART_ERROR_OVERRUN;
    }

    if (UART_GetFlagStatus(uart, UART_FLAG_RX_PARITY_ERR))
    {
        err |= UART_ERROR_PARITY;
    }

    if (UART_GetFlagStatus(uart, UART_FLAG_RX_FRAME_ERR))
    {
        err |= UART_ERROR_FRAMING;
    }

    if (UART_GetFlagStatus(uart, UART_FLAG_RX_BREAK_ERR))
    {
        err |= UART_BREAK;
    }

    return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_rtl87x2g_fifo_fill(const struct device *dev,
                                   const uint8_t *tx_data,
                                   int size)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    uint8_t num_tx = 0U;
    unsigned int key;

    if (!UART_GetFlagStatus(uart, UART_FLAG_TX_EMPTY))
    {
        return num_tx;
    }

    /* Lock interrupts to prevent nested interrupts or thread switch */

    key = irq_lock();

    while ((size - num_tx > 0) &&
           UART_GetFlagStatus(uart, UART_FLAG_TX_EMPTY))
    {
        UART_SendByte(uart, (uint8_t)tx_data[num_tx++]);
    }

    irq_unlock(key);

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_fifo_fill] num_tx=%d", num_tx);
#endif
    return num_tx;
}

static int uart_rtl87x2g_fifo_read(const struct device *dev, uint8_t *rx_data,
                                   const int size)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    uint8_t num_rx = 0U;

    while ((size - num_rx > 0) &&
           UART_GetFlagStatus(uart, UART_FLAG_RX_DATA_AVA))
    {
        rx_data[num_rx++] = UART_ReceiveByte(uart);
    }

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_fifo_read] num_rx=%d", num_rx);
#endif
    return num_rx;
}

static void uart_rtl87x2g_irq_tx_enable(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    struct uart_rtl87x2g_data *data = dev->data;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_tx_enable]");
#endif

    data->tx_int_en = true;
    UART_INTConfig(uart, UART_INT_TX_FIFO_EMPTY, ENABLE);
}

static void uart_rtl87x2g_irq_tx_disable(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    struct uart_rtl87x2g_data *data = dev->data;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_tx_disable]");
#endif

    data->tx_int_en = false;
    UART_INTConfig(uart, UART_INT_TX_FIFO_EMPTY, DISABLE);
}

static int uart_rtl87x2g_irq_tx_ready(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_tx_ready]");
#endif
    return UART_GetFlagStatus(uart, UART_FLAG_TX_EMPTY) &&
           data->tx_int_en;
}

static int uart_rtl87x2g_irq_tx_complete(const struct device *dev)
{
    return uart_rtl87x2g_irq_tx_ready(dev);
}

static void uart_rtl87x2g_irq_rx_enable(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    struct uart_rtl87x2g_data *data = dev->data;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_rx_enable]");
#endif

    data->rx_int_en = true;
    UART_INTConfig(uart, UART_INT_RD_AVA, ENABLE);
}

static void uart_rtl87x2g_irq_rx_disable(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    struct uart_rtl87x2g_data *data = dev->data;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_rx_disable]");
#endif

    data->rx_int_en = false;
    UART_INTConfig(uart, UART_INT_RD_AVA, DISABLE);
}

static int uart_rtl87x2g_irq_rx_ready(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_rx_ready]");
#endif

    return UART_GetFlagStatus(uart, UART_FLAG_RX_DATA_AVA);
}

static void uart_rtl87x2g_irq_err_enable(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_err_enable]");
#endif

    UART_INTConfig(uart, UART_INT_RX_LINE_STS, ENABLE);
}

static void uart_rtl87x2g_irq_err_disable(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_err_disable]");
#endif

    UART_INTConfig(uart, UART_INT_RX_LINE_STS, DISABLE);
}

static int uart_rtl87x2g_irq_is_pending(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_is_pending]");
#endif

    return ((UART_GetFlagStatus(uart, UART_FLAG_TX_EMPTY) &&
             data->tx_int_en) ||
            (UART_GetFlagStatus(uart, UART_INT_RD_AVA) &&
             data->rx_int_en));
}

static int uart_rtl87x2g_irq_update(const struct device *dev)
{
    return 1;
}

static void uart_rtl87x2g_irq_callback_set(const struct device *dev,
                                           uart_irq_callback_user_data_t cb,
                                           void *cb_data)
{
    struct uart_rtl87x2g_data *data = dev->data;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_irq_callback_set]");
#endif

    data->user_cb = cb;
    data->user_data = cb_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_LINE_CTRL
int uart_rtl87x2g_line_ctrl_set(const struct device *dev, uint32_t ctrl, uint32_t val)
{
    LOG_ERR("Unsupport line_ctrl_set function");
    return -ENOTSUP;
}

int uart_rtl87x2g_line_ctrl_get(const struct device *dev, uint32_t ctrl, uint32_t *val)
{
    LOG_ERR("Unsupport line_ctrl_get function");
    return -ENOTSUP;
}
#endif

#ifdef CONFIG_UART_DRV_CMD
int uart_rtl87x2g_drv_cmd(const struct device *dev, uint32_t cmd, uint32_t p)
{
    LOG_ERR("Unsupport drv_cmd function");
    return -ENOTSUP;
}
#endif
#ifdef CONFIG_UART_ASYNC_API
static inline void async_user_callback(struct uart_rtl87x2g_data *data,
                                       struct uart_event *event)
{
    if (data->async_cb)
    {
        data->async_cb(data->dev, event, data->async_user_data);
    }
}

static inline void async_evt_rx_rdy(struct uart_rtl87x2g_data *data)
{
    LOG_DBG("rx_rdy: (%d %d)", data->dma_rx.offset, data->dma_rx.counter);
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[async_evt_rx_rdy] data->dma_rx.counter=%d, data->dma_rx.offset=%d",
               data->dma_rx.counter, data->dma_rx.offset);
#endif
    struct uart_event event =
    {
        .type = UART_RX_RDY,
        .data.rx.buf = data->dma_rx.buffer,
        .data.rx.len = data->dma_rx.counter - data->dma_rx.offset,
        .data.rx.offset = data->dma_rx.offset
    };

    /* update the current pos for new data */
    data->dma_rx.offset = data->dma_rx.counter;

    /* send event only for new data */
    if (event.data.rx.len > 0)
    {
        async_user_callback(data, &event);
    }
}

static inline void async_evt_rx_err(struct uart_rtl87x2g_data *data, int err_code)
{
    LOG_DBG("rx error: %d", err_code);

    struct uart_event event =
    {
        .type = UART_RX_STOPPED,
        .data.rx_stop.reason = err_code,
        .data.rx_stop.data.len = data->dma_rx.counter,
        .data.rx_stop.data.offset = 0,
        .data.rx_stop.data.buf = data->dma_rx.buffer
    };

    async_user_callback(data, &event);
}

static inline void async_evt_tx_done(struct uart_rtl87x2g_data *data)
{
    LOG_DBG("tx done: %d", data->dma_tx.counter);

    struct uart_event event =
    {
        .type = UART_TX_DONE,
        .data.tx.buf = data->dma_tx.buffer,
        .data.tx.len = data->dma_tx.counter
    };

    /* Reset tx buffer */
    data->dma_tx.buffer_length = 0;
    data->dma_tx.counter = 0;

    async_user_callback(data, &event);
}

static inline void async_evt_tx_abort(struct uart_rtl87x2g_data *data)
{
    LOG_DBG("tx abort: %d", data->dma_tx.counter);

    struct uart_event event =
    {
        .type = UART_TX_ABORTED,
        .data.tx.buf = data->dma_tx.buffer,
        .data.tx.len = data->dma_tx.counter
    };

    /* Reset tx buffer */
    data->dma_tx.buffer_length = 0;
    data->dma_tx.counter = 0;

    async_user_callback(data, &event);
}

static inline void async_evt_rx_buf_request(struct uart_rtl87x2g_data *data)
{
    struct uart_event evt =
    {
        .type = UART_RX_BUF_REQUEST,
    };

    async_user_callback(data, &evt);
}

static inline void async_evt_rx_buf_release(struct uart_rtl87x2g_data *data)
{
    struct uart_event evt =
    {
        .type = UART_RX_BUF_RELEASED,
        .data.rx_buf.buf = data->dma_rx.buffer,
    };

    async_user_callback(data, &evt);
}

static inline void async_timer_start(struct k_work_delayable *work,
                                     int32_t timeout)
{
    if ((timeout != SYS_FOREVER_US) && (timeout != 0))
    {
        /* start timer */
        LOG_DBG("async timer started for %d us", timeout);
        k_work_reschedule(work, K_USEC(timeout));
    }
}

static void uart_rtl87x2g_dma_rx_flush(const struct device *dev)
{
    struct dma_status stat;
    struct uart_rtl87x2g_data *data = dev->data;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_rx_flush]");
#endif

    if (dma_get_status(data->dma_rx.dma_dev,
                       data->dma_rx.dma_channel, &stat) == 0)
    {
        size_t rx_rcv_len = data->dma_rx.buffer_length -
                            stat.pending_length;
#if DBG_DIRECT_SHOW
        DBG_DIRECT("data->dma_rx.buffer_length=%d, stat.pending_length=%d, rx_rcv_len=%d, data->dma_rx.offset=%d, rx fifo=%d",
                   \
                   data->dma_rx.buffer_length, stat.pending_length, rx_rcv_len, data->dma_rx.offset,
                   UART_GetRxFIFOLen(((struct uart_rtl87x2g_config *)(dev->config))->uart));
#endif
        if (rx_rcv_len > data->dma_rx.offset)
        {
            data->dma_rx.counter = rx_rcv_len;

            async_evt_rx_rdy(data);
        }
    }
}

static inline void uart_rtl87x2g_dma_tx_enable(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_tx_enable]");
#endif
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    UART_MISCR_TypeDef uart_0x28 = {.d32 = uart->UART_MISCR};
    uart_0x28.b.txdma_en = ENABLE;
    uart_0x28.b.txdma_burstsize = 15;
    uart->UART_MISCR = uart_0x28.d32;
}

static inline void uart_rtl87x2g_dma_tx_disable(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_tx_disable]");
#endif
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    UART_MISCR_TypeDef uart_0x28 = {.d32 = uart->UART_MISCR};
    uart_0x28.b.txdma_en = DISABLE;
    uart->UART_MISCR = uart_0x28.d32;
}

static inline void uart_rtl87x2g_dma_rx_enable(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_rx_enable]");
#endif
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;
    UART_MISCR_TypeDef uart_0x28 = {.d32 = uart->UART_MISCR};
    uart_0x28.b.rxdma_en = ENABLE;
    uart_0x28.b.rxdma_burstsize = 1;
    uart->UART_MISCR = uart_0x28.d32;

    data->dma_rx.enabled = true;
}

static inline void uart_rtl87x2g_dma_rx_disable(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_rx_disable]");
#endif
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;
    UART_MISCR_TypeDef uart_0x28 = {.d32 = uart->UART_MISCR};
    uart_0x28.b.rxdma_en = DISABLE;
    uart_0x28.b.rxdma_burstsize = 1;
    uart->UART_MISCR = uart_0x28.d32;

    data->dma_rx.enabled = false;
}

void uart_rtl87x2g_dma_tx_cb(const struct device *dma_dev, void *user_data,
                             uint32_t channel, int status)
{
    const struct device *uart_dev = user_data;
    struct uart_rtl87x2g_data *data = uart_dev->data;
    struct dma_status stat;
    unsigned int key = irq_lock();

    /* Disable TX */
    uart_rtl87x2g_dma_tx_disable(uart_dev);

    (void)k_work_cancel_delayable(&data->dma_tx.timeout_work);

    if (!dma_get_status(data->dma_tx.dma_dev,
                        data->dma_tx.dma_channel, &stat))
    {
        data->dma_tx.counter = data->dma_tx.buffer_length -
                               stat.pending_length;
    }

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_tx_cb] channel=%d, status=%d, dma_tx.counter=%d", channel, status,
               data->dma_tx.counter);
#endif

    data->dma_tx.buffer_length = 0;

    irq_unlock(key);

    dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);

    /* Generate TX_DONE event when transmission is done */
    async_evt_tx_done(data);
}

static void uart_rtl87x2g_dma_replace_buffer(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_replace_buffer]");
#endif
    const struct device *uart_dev = dev;
    struct uart_rtl87x2g_data *data = uart_dev->data;

    /* Replace the buffer and reload the DMA */
    LOG_DBG("Replacing RX buffer: %d", data->rx_next_buffer_len);

    /* reload DMA */
    data->dma_rx.offset = 0;
    data->dma_rx.counter = 0;
    data->dma_rx.buffer = data->rx_next_buffer;
    data->dma_rx.buffer_length = data->rx_next_buffer_len;
    data->dma_rx.blk_cfg.block_size = data->dma_rx.buffer_length;
    data->dma_rx.blk_cfg.dest_address = (uint32_t)(data->dma_rx.buffer);
    data->rx_next_buffer = NULL;
    data->rx_next_buffer_len = 0;

    dma_reload(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
               data->dma_rx.blk_cfg.source_address,
               data->dma_rx.blk_cfg.dest_address,
               data->dma_rx.blk_cfg.block_size);

    dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

    /* Request next buffer */
    async_evt_rx_buf_request(data);
}

void uart_rtl87x2g_dma_rx_cb(const struct device *dma_dev, void *user_data,
                             uint32_t channel, int status)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_rx_cb] channel=%d, status=%d", channel, status);
#endif
    const struct device *uart_dev = user_data;
    struct uart_rtl87x2g_data *data = uart_dev->data;

    if (status < 0)
    {
        async_evt_rx_err(data, status);
        return;
    }

    (void)k_work_cancel_delayable(&data->dma_rx.timeout_work);

    /* true since this functions occurs when buffer if full */
    data->dma_rx.counter = data->dma_rx.buffer_length;

    async_evt_rx_rdy(data);

    if (data->rx_next_buffer != NULL)
    {
#if DBG_DIRECT_SHOW
        DBG_DIRECT("[uart_rtl87x2g_dma_rx_cb] channel=%d, status=%d, data->rx_next_buffer != NULL", channel,
                   status);
#endif
        async_evt_rx_buf_release(data);

        /* replace the buffer when the current
        * is full and not the same as the next
        * one.
        */
        uart_rtl87x2g_dma_replace_buffer(uart_dev);
    }
    else
    {
#if DBG_DIRECT_SHOW
        DBG_DIRECT("[uart_rtl87x2g_dma_rx_cb] channel=%d, status=%d, data->rx_next_buffer == NULL", channel,
                   status);
#endif

        k_work_reschedule(&data->dma_rx.timeout_work, K_TICKS(1));
    }
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_dma_rx_cb] exit");
#endif
}

static int uart_rtl87x2g_async_callback_set(const struct device *dev,
                                            uart_callback_t callback,
                                            void *user_data)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_callback_set]");
#endif
    struct uart_rtl87x2g_data *data = dev->data;

    data->async_cb = callback;
    data->async_user_data = user_data;

    return 0;
}

static int uart_rtl87x2g_async_tx(const struct device *dev,
                                  const uint8_t *tx_data, size_t buf_size, int32_t timeout)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_tx] bufsize=%d, timeout=%d", buf_size, timeout);
#endif
    struct uart_rtl87x2g_data *data = dev->data;
    int ret;

    if (data->dma_tx.dma_dev == NULL)
    {
        return -ENODEV;
    }

    if (data->dma_tx.buffer_length != 0)
    {
        return -EBUSY;
    }

    data->dma_tx.buffer = (uint8_t *)tx_data;
    data->dma_tx.buffer_length = buf_size;
    data->dma_tx.timeout = timeout;

    LOG_DBG("tx: l=%d", data->dma_tx.buffer_length);

    /* set source address */
    data->dma_tx.blk_cfg.source_address = (uint32_t)(data->dma_tx.buffer);
    data->dma_tx.blk_cfg.block_size = data->dma_tx.buffer_length;

    ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel,
                     &data->dma_tx.dma_cfg);

    if (ret != 0)
    {
        LOG_ERR("dma tx config error!");
        return -EINVAL;
    }

    /* Start TX timer */
    async_timer_start(&data->dma_tx.timeout_work, data->dma_tx.timeout);

    /* Enable TX DMA requests */
    uart_rtl87x2g_dma_tx_enable(dev);

    if (dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel))
    {
        LOG_ERR("UART err: TX DMA start failed!");
        return -EFAULT;
    }

    return 0;
}

static int uart_rtl87x2g_async_tx_abort(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_tx_abort]");
#endif
    struct uart_rtl87x2g_data *data = dev->data;
    size_t tx_buffer_length = data->dma_tx.buffer_length;
    struct dma_status stat;
    if (tx_buffer_length == 0)
    {
        return -EFAULT;
    }

    (void)k_work_cancel_delayable(&data->dma_tx.timeout_work);

    dma_suspend(data->dma_tx.dma_dev, data->dma_tx.dma_channel);

    if (!dma_get_status(data->dma_tx.dma_dev,
                        data->dma_tx.dma_channel, &stat))
    {
        data->dma_tx.counter = tx_buffer_length - stat.pending_length;
    }

    dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
    async_evt_tx_abort(data);

    return 0;
}

static int uart_rtl87x2g_async_rx_enable(const struct device *dev,
                                         uint8_t *rx_buf, size_t buf_size, int32_t timeout)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_rx_enable] buf_size=%d, timeout=%d", buf_size, timeout);
#endif
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;
    int ret;
    uint32_t cnt = UART_GetRxFIFOLen(uart);
    for (uint32_t i = 0; i < cnt; i++)
    {
        UART_ReceiveByte(uart);
    }

    if (data->dma_rx.dma_dev == NULL)
    {
        return -ENODEV;
    }

    if (data->dma_rx.enabled)
    {
        LOG_WRN("RX was already enabled");
        return -EBUSY;
    }

    data->dma_rx.offset = 0;
    data->dma_rx.buffer = rx_buf;
    data->dma_rx.buffer_length = buf_size;
    data->dma_rx.counter = 0;
    data->dma_rx.timeout = timeout;

    /* Disable RX interrupts to let DMA to handle it */
    UART_INTConfig(uart, UART_INT_RD_AVA, DISABLE);

    data->dma_rx.blk_cfg.block_size = buf_size;
    data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;

    ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
                     &data->dma_rx.dma_cfg);

    if (ret != 0)
    {
        LOG_ERR("UART ERR: RX DMA config failed!");
        return -EINVAL;
    }

    /* Enable RX DMA requests */
    uart_rtl87x2g_dma_rx_enable(dev);

    /* Enable UART_INT_RX_IDLE to define the end of a
    * RX DMA transaction.
    */
    UART_INTConfig(uart, UART_INT_RX_IDLE, DISABLE);
    UART_INTConfig(uart, UART_INT_RX_IDLE, ENABLE);

    if (dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel))
    {
        LOG_ERR("UART ERR: RX DMA start failed!");
        return -EFAULT;
    }

    /* Request next buffer */
    async_evt_rx_buf_request(data);

    LOG_DBG("async rx enabled");

    return ret;
}

static int uart_rtl87x2g_async_rx_buf_rsp(const struct device *dev, uint8_t *buf,
                                          size_t len)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_rx_buf_rsp] len=%d", len);
#endif
    struct uart_rtl87x2g_data *data = dev->data;

    LOG_DBG("replace buffer (%d)", len);
    data->rx_next_buffer = buf;
    data->rx_next_buffer_len = len;

    return 0;
}

static int uart_rtl87x2g_async_rx_disable(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_rx_disable]");
#endif
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;
    struct uart_event disabled_event =
    {
        .type = UART_RX_DISABLED
    };

    if (!data->dma_rx.enabled)
    {
        async_user_callback(data, &disabled_event);
        return -EFAULT;
    }

    UART_INTConfig(uart, UART_INT_RX_IDLE, DISABLE);

    uart_rtl87x2g_dma_rx_flush(dev);

    async_evt_rx_buf_release(data);

    dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

    uart_rtl87x2g_dma_rx_disable(dev);

    (void)k_work_cancel_delayable(&data->dma_rx.timeout_work);

    if (data->rx_next_buffer)
    {
        struct uart_event rx_next_buf_release_evt =
        {
            .type = UART_RX_BUF_RELEASED,
            .data.rx_buf.buf = data->rx_next_buffer,
        };
        async_user_callback(data, &rx_next_buf_release_evt);
    }

    data->rx_next_buffer = NULL;
    data->rx_next_buffer_len = 0;

    /* When async rx is disabled, enable interruptible instance of uart to function normally */

    LOG_DBG("rx: disabled");

    async_user_callback(data, &disabled_event);

    return 0;
}

static void uart_rtl87x2g_async_tx_timeout(struct k_work *work)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_tx_timeout]");
#endif
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct uart_dma_stream *tx_stream = CONTAINER_OF(dwork,
                                                     struct uart_dma_stream, timeout_work);
    struct uart_rtl87x2g_data *data = CONTAINER_OF(tx_stream,
                                                   struct uart_rtl87x2g_data, dma_tx);
    const struct device *dev = data->dev;

    uart_rtl87x2g_async_tx_abort(dev);

    LOG_DBG("tx: async timeout");
}

static void uart_rtl87x2g_async_rx_timeout(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct uart_dma_stream *rx_stream = CONTAINER_OF(dwork,
                                                     struct uart_dma_stream, timeout_work);
    struct uart_rtl87x2g_data *data = CONTAINER_OF(rx_stream,
                                                   struct uart_rtl87x2g_data, dma_rx);
    const struct device *dev = data->dev;

    LOG_DBG("rx timeout");

#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_rx_timeout] data->dma_rx.counter=%d, data->dma_rx.buffer_length=%d",
               \
               data->dma_rx.counter, data->dma_rx.buffer_length);
#endif
    if (data->dma_rx.counter == data->dma_rx.buffer_length)
    {
        uart_rtl87x2g_async_rx_disable(dev);
    }
    else
    {
        uart_rtl87x2g_dma_rx_flush(dev);
    }
}

static int uart_rtl87x2g_async_init(const struct device *dev)
{
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_async_init]");
#endif
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;

    data->dev = dev;

    if (data->dma_rx.dma_dev != NULL)
    {
        if (!device_is_ready(data->dma_rx.dma_dev))
        {
            return -ENODEV;
        }
    }

    if (data->dma_tx.dma_dev != NULL)
    {
        if (!device_is_ready(data->dma_tx.dma_dev))
        {
            return -ENODEV;
        }
    }

    /* Disable both TX and RX DMA requests */
    uart_rtl87x2g_dma_rx_disable(dev);
    uart_rtl87x2g_dma_tx_disable(dev);

    k_work_init_delayable(&data->dma_rx.timeout_work,
                          uart_rtl87x2g_async_rx_timeout);
    k_work_init_delayable(&data->dma_tx.timeout_work,
                          uart_rtl87x2g_async_tx_timeout);

    /* Configure dma rx config */
    memset(&data->dma_rx.blk_cfg, 0, sizeof(data->dma_rx.blk_cfg));

    data->dma_rx.blk_cfg.source_address = (uint32_t)(&(uart->UART_RBR_THR));


    data->dma_rx.blk_cfg.dest_address = 0; /* dest not ready */
    data->dma_rx.blk_cfg.source_addr_adj = data->dma_rx.src_addr_increment;
    data->dma_rx.blk_cfg.dest_addr_adj = data->dma_rx.dst_addr_increment;

    /* RX disable circular buffer */
    data->dma_rx.blk_cfg.source_reload_en  = 0;
    data->dma_rx.blk_cfg.dest_reload_en = 0;

    data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
    data->dma_rx.dma_cfg.user_data = (void *)dev;
    data->rx_next_buffer = NULL;
    data->rx_next_buffer_len = 0;

    /* Configure dma tx config */
    memset(&data->dma_tx.blk_cfg, 0, sizeof(data->dma_tx.blk_cfg));

    data->dma_tx.blk_cfg.dest_address = (uint32_t)(&(uart->UART_RBR_THR));

    data->dma_tx.blk_cfg.source_address = 0; /* not ready */

    data->dma_tx.blk_cfg.source_addr_adj = data->dma_tx.src_addr_increment;

    data->dma_tx.blk_cfg.dest_addr_adj = data->dma_tx.dst_addr_increment;

    data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
    data->dma_tx.dma_cfg.user_data = (void *)dev;

    return 0;
}

#endif /* CONFIG_UART_ASYNC_API */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || \
    defined(CONFIG_UART_ASYNC_API)

static void uart_rtl87x2g_isr(const struct device *dev)
{
    struct uart_rtl87x2g_data *data = dev->data;
    const struct uart_rtl87x2g_config *config = dev->config;
    UART_TypeDef *uart = config->uart;
    UART_GetIID(uart);
    if (data->user_cb)
    {
        data->user_cb(dev, data->user_data);
    }

#ifdef CONFIG_UART_ASYNC_API
    if (UART_GetFlagStatus(uart, UART_FLAG_RX_IDLE))
    {
        LOG_DBG("idle interrupt occurred");
        if (data->dma_rx.timeout == 0)
        {
#if DBG_DIRECT_SHOW
            DBG_DIRECT("[uart_rtl87x2g_isr] UART_FLAG_RX_IDLE timeout == 0");
#endif
            uart_rtl87x2g_dma_rx_flush(dev);
        }
        else
        {
#if DBG_DIRECT_SHOW
            dma_suspend(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
            DBG_DIRECT("[uart_rtl87x2g_isr] dma len=%d",
                       GDMA_GetTransferLen(GDMA_GetGDMAChannelx(data->dma_rx.dma_channel)));
            dma_resume(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
#endif

            /* Start the RX timer not null */
            UART_INTConfig(uart, UART_INT_RX_IDLE, DISABLE);
            UART_INTConfig(uart, UART_INT_RX_IDLE, ENABLE);
            async_timer_start(&data->dma_rx.timeout_work,
                              data->dma_rx.timeout);
        }
    }

    /* Clear errors */
    uart_rtl87x2g_err_check(dev);
#endif /* CONFIG_UART_ASYNC_API */
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_isr] exit");
#endif
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

#ifdef CONFIG_PM_DEVICE
static int uart_rtl87x2g_pm_action(const struct device *dev,
                                   enum pm_device_action action)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    UART_TypeDef *uart = config->uart;
    int err;
    extern void UART_DLPSEnter(void *PeriReg, void *StoreBuf);
    extern void UART_DLPSExit(void *PeriReg, void *StoreBuf);

    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:

        UART_DLPSEnter(uart, &data->store_buf);

        /* Move pins to sleep state */
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
        if ((err < 0) && (err != -ENOENT))
        {
            return err;
        }
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* Set pins to active state */
        err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (err < 0)
        {
            return err;
        }

        UART_DLPSExit(uart, &data->store_buf);

        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct uart_driver_api uart_rtl87x2g_driver_api =
{
    .poll_in = uart_rtl87x2g_poll_in,
    .poll_out = uart_rtl87x2g_poll_out,
    .err_check = uart_rtl87x2g_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = uart_rtl87x2g_configure,
    .config_get = uart_rtl87x2g_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill = uart_rtl87x2g_fifo_fill,
    .fifo_read = uart_rtl87x2g_fifo_read,
    .irq_tx_enable = uart_rtl87x2g_irq_tx_enable,
    .irq_tx_disable = uart_rtl87x2g_irq_tx_disable,
    .irq_tx_ready = uart_rtl87x2g_irq_tx_ready,
    .irq_tx_complete = uart_rtl87x2g_irq_tx_complete,
    .irq_rx_enable = uart_rtl87x2g_irq_rx_enable,
    .irq_rx_disable = uart_rtl87x2g_irq_rx_disable,
    .irq_rx_ready = uart_rtl87x2g_irq_rx_ready,
    .irq_err_enable = uart_rtl87x2g_irq_err_enable,
    .irq_err_disable = uart_rtl87x2g_irq_err_disable,
    .irq_is_pending = uart_rtl87x2g_irq_is_pending,
    .irq_update = uart_rtl87x2g_irq_update,
    .irq_callback_set = uart_rtl87x2g_irq_callback_set,
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
    .callback_set = uart_rtl87x2g_async_callback_set,
    .tx = uart_rtl87x2g_async_tx,
    .tx_abort = uart_rtl87x2g_async_tx_abort,
    .rx_enable = uart_rtl87x2g_async_rx_enable,
    .rx_buf_rsp = uart_rtl87x2g_async_rx_buf_rsp,
    .rx_disable = uart_rtl87x2g_async_rx_disable,
#endif  /* CONFIG_UART_ASYNC_API */

#ifdef CONFIG_UART_LINE_CTRL
    .line_ctrl_set = uart_rtl87x2g_line_ctrl_set,
    .line_ctrl_get = uart_rtl87x2g_line_ctrl_get,
#endif

#ifdef CONFIG_UART_DRV_CMD
    .drv_cmd = uart_rtl87x2g_drv_cmd,
#endif
};

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_rtl87x2g_init(const struct device *dev)
{
    const struct uart_rtl87x2g_config *config = dev->config;
    struct uart_rtl87x2g_data *data = dev->data;
    int err;
#if DBG_DIRECT_SHOW
    DBG_DIRECT("[uart_rtl87x2g_init]");
#endif
    data->dev = dev;

    (void)clock_control_on(RTL87X2G_CLOCK_CONTROLLER,
                           (clock_control_subsys_t)&config->clkid);

    /* Configure pinmux  */

    err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (err < 0)
    {
        return err;
    }

    /* Configure peripheral  */

    err = uart_rtl87x2g_configure(dev, &data->uart_config);
    if (err)
    {
        return err;
    }

    /* Enable nvic */
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || \
    defined(CONFIG_UART_ASYNC_API)
    config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

#ifdef CONFIG_UART_ASYNC_API
    return uart_rtl87x2g_async_init(dev);
#else
    return 0;
#endif
}

#ifdef CONFIG_UART_ASYNC_API

#define UART_DMA_CHANNEL_INIT(index, dir)            \
    .dma_dev = DEVICE_DT_GET(RTL87X2G_DMA_CTLR(index, dir)),         \
               .dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),          \
                              .dma_cfg = {                                  \
                                                                            .dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),            \
                                                                            .channel_direction = RTL87X2G_DMA_CONFIG_DIRECTION(         \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),           \
                                                                            .channel_priority = RTL87X2G_DMA_CONFIG_PRIORITY(        \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),       \
                                                                            .source_data_size = RTL87X2G_DMA_CONFIG_SOURCE_DATA_SIZE(          \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),        \
                                                                            .dest_data_size = RTL87X2G_DMA_CONFIG_DESTINATION_DATA_SIZE(         \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),         \
                                                                            .source_burst_length = RTL87X2G_DMA_CONFIG_SOURCE_MSIZE(        \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),         \
                                                                            .dest_burst_length = RTL87X2G_DMA_CONFIG_DESTINATION_MSIZE(        \
                                                                                    RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),         \
                                                                            .block_count = 1,              \
                                                                            .dma_callback = uart_rtl87x2g_dma_##dir##_cb,          \
                                         },                                  \
                                         .src_addr_increment = RTL87X2G_DMA_CONFIG_SOURCE_ADDR_INC(          \
                                                               RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),          \
                                                               .dst_addr_increment = RTL87X2G_DMA_CONFIG_DESTINATION_ADDR_INC(        \
                                                                       RTL87X2G_DMA_CHANNEL_CONFIG(index, dir)),         \

#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define RTL87X2G_UART_IRQ_HANDLER_DECL(index)               \
    static void uart_rtl87x2g_irq_config_func_##index(const struct device *dev);
#define RTL87X2G_UART_IRQ_HANDLER(index)                    \
    static void uart_rtl87x2g_irq_config_func_##index(const struct device *dev) \
    {                                   \
        IRQ_CONNECT(DT_INST_IRQN(index),                \
                    DT_INST_IRQ(index, priority),               \
                    uart_rtl87x2g_isr, DEVICE_DT_INST_GET(index),       \
                    0);                         \
        irq_enable(DT_INST_IRQN(index));                \
    }
#else
#define RTL87X2G_UART_IRQ_HANDLER_DECL(index) /* Not used */
#define RTL87X2G_UART_IRQ_HANDLER(index) /* Not used */
#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define RTL87X2G_UART_IRQ_HANDLER_FUNC(index)               \
    .irq_config_func = uart_rtl87x2g_irq_config_func_##index,
#else
#define RTL87X2G_UART_IRQ_HANDLER_FUNC(index) /* Not used */
#endif

#ifdef CONFIG_UART_ASYNC_API
#define UART_DMA_CHANNEL(index, dir)    \
    .dma_##dir = {                         \
                                           COND_CODE_1(DT_INST_DMAS_HAS_NAME(index, dir),               \
                                                       (UART_DMA_CHANNEL_INIT(index, dir)),           \
                                                       (NULL))                      \
                 },
#else
#define UART_DMA_CHANNEL(index, dir)
#endif

#define RTL87X2G_UART_INIT(index)                                          \
    RTL87X2G_UART_IRQ_HANDLER_DECL(index)                                   \
    \
    PINCTRL_DT_INST_DEFINE(index);                                            \
    \
    static const struct uart_rtl87x2g_config uart_rtl87x2g_cfg_##index = {      \
        .uart = (UART_TypeDef *)DT_INST_REG_ADDR(index),                        \
                .clkid = DT_INST_CLOCKS_CELL(index, id),            \
                         .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                            \
                                 .hw_flow_ctrl = DT_INST_PROP_OR(index, flow_ctrl, false),                 \
                                                 RTL87X2G_UART_IRQ_HANDLER_FUNC(index)                                       \
    };                                                                               \
    \
    static struct uart_rtl87x2g_data uart_rtl87x2g_data_##index = {                    \
        .uart_config = {                                                             \
                                                                                     .baudrate = DT_INST_PROP(index, current_speed),                           \
                                                                                     .parity = DT_INST_ENUM_IDX_OR(index, parity, UART_CFG_PARITY_NONE),       \
                                                                                     .stop_bits = DT_INST_ENUM_IDX_OR(index, stop_bits, UART_CFG_STOP_BITS_1),\
                                                                                     .data_bits = DT_INST_ENUM_IDX_OR(index, data_bits, UART_CFG_DATA_BITS_8),\
                                                                                     .flow_ctrl = DT_INST_PROP(index, hw_flow_control),                       \
                       },              \
                       UART_DMA_CHANNEL(index, rx)                     \
                       UART_DMA_CHANNEL(index, tx)                    \
    };                                                                       \
    \
     PM_DEVICE_DT_INST_DEFINE(index, uart_rtl87x2g_pm_action);    \
     DEVICE_DT_INST_DEFINE(index,                                              \
                          &uart_rtl87x2g_init,                                  \
                          PM_DEVICE_DT_INST_GET(index),                           \
                          &uart_rtl87x2g_data_##index, &uart_rtl87x2g_cfg_##index,    \
                          PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,                    \
                          &uart_rtl87x2g_driver_api);                                  \
    \
    RTL87X2G_UART_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(RTL87X2G_UART_INIT)



