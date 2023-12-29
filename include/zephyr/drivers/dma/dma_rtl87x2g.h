/*
* Copyright(c) 2020, Realtek Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_RTL87X2G_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_RTL87X2G_H_

#define RTL87X2G_DMA_CTLR(id, dir)                      \
    DT_INST_DMAS_CTLR_BY_NAME(id, dir)
#define RTL87X2G_DMA_CHANNEL_CONFIG(id, dir)                    \
    DT_INST_DMAS_CELL_BY_NAME(id, dir, config)

/* macros for channel-config */
/* direction defined on bits 0-1 */
/* 0 -> MEM_TO_MEM, 1 -> MEM_TO_PERIPH, 2 -> PERIPH_TO_MEM */
#define RTL87X2G_DMA_CONFIG_DIRECTION(config)       ((config >> 0) & 0x3)

/* source increment defined on bit 2-3 */
/* 0 -> increment, 1 -> decrement, 2 -> no change */
#define RTL87X2G_DMA_CONFIG_SOURCE_ADDR_INC(config) ((config >> 2) & 0x3)

/* destination increment defined on bit 4-5 */
/* 0 -> increment, 1 -> decrement, 2 -> no change */
#define RTL87X2G_DMA_CONFIG_DESTINATION_ADDR_INC(config)    ((config >> 4) & 0x3)

/* source data size defined on bits 6-7 */
/* 0 -> 1 byte, 1 -> 2 bytes, 2 -> 4 bytes */
#define RTL87X2G_DMA_CONFIG_SOURCE_DATA_SIZE(config)    \
    ((config >> 6) & 0x3)

/* destination data size defined on bits 8-9 */
/* 0 -> 1 byte, 1 -> 2 bytes, 2 -> 4 bytes */
#define RTL87X2G_DMA_CONFIG_DESTINATION_DATA_SIZE(config)   \
    ((config >> 8) & 0x3)

/* source msize defined on bits 10-12 */
/* 0 -> msize1, 1 -> msize4, 2 -> msize8, 3 -> msize16 */
/* 4 -> msize32, 5 -> msize64, 6 -> msize128, 7 -> msize256 */
#define RTL87X2G_DMA_CONFIG_SOURCE_MSIZE(config)    \
    ((config >> 10) & 0x7)

/* destination msize defined on bits 13-15 */
/* 0 -> msize1, 1 -> msize4, 2 -> msize8, 3 -> msize16 */
/* 4 -> msize32, 5 -> msize64, 6 -> msize128, 7 -> msize256 */
#define RTL87X2G_DMA_CONFIG_DESTINATION_MSIZE(config)   \
    ((config >> 13) & 0x7)

/* priority defined on bits 16-20 as 0-9 */
#define RTL87X2G_DMA_CONFIG_PRIORITY(config)        ((config >> 16) & 0x1f)

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_RTL87X2G_H_ */
