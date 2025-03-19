/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/ir.h>
#include <zephyr/kernel.h>

#define TX_LEN 100
uint32_t tx_buf[TX_LEN];
uint32_t tx_len = TX_LEN;
uint32_t tx_sent;
const struct device *const ir_dev = DEVICE_DT_GET(DT_NODELABEL(ir));

void ir_tx_cb(const struct device *dev, struct ir_event *evt, void *user_data)
{
	tx_sent += evt->data.tx.len;
	if (tx_sent < tx_len) {
		ir_tx(ir_dev, &(tx_buf[tx_sent]), tx_len - tx_sent);
	}
	printf("[%s] dev:%s evt:%d tx_len:%d\n", __func__, dev->name, evt->type, evt->data.tx.len);
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD);

	tx_buf[0] = 0x80000000 | 0x156; // 342 about 9ms
	tx_buf[1] = 0x00000000 | 0xAB;  // 171 about 4.5ms
	for (uint16_t i = 2; i < TX_LEN - 1;) {
		tx_buf[i + 1] = 0x80000000 | 0x15; // 21  about 560us
		tx_buf[i] = 0x00000000 | 0x15;     // 21  about 565us
		i += 2;
	}
	tx_buf[TX_LEN - 1] = 0x80000000 | 0x15;

	ir_set_freq(ir_dev, 38000, 3);
	ir_tx_enable(ir_dev, ir_tx_cb, NULL);
	ir_tx(ir_dev, tx_buf, 100);

	return 0;
}
