/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <zephyr/drivers/ir.h>
#include <zephyr/kernel.h>

/*
 * Is tx_done_sem (given from ir_tx_cb on IR_TX_COMPLETED) a *precise* end-of-
 * transmission signal -- i.e. does it fire only after the carrier has
 * physically clocked the whole frame out? This depends on the SoC's IR HAL and
 * must mirror the driver's BEE_IR_HAS_TX_FINISH:
 *
 *   - rtl8752h and rtl87x2j: the HAL provides the dedicated TX-finish interrupt
 *     (IR_SUPPORT_TX_FINISH_INTERRUPT == 1 / IR_INT_TX_FINISH), which fires
 *     exactly when transmission completes. So tx_done_sem is accurate and we
 *     can block on it.
 *
 *   - rtl87x2g: rtl_ir_def.h sets IR_SUPPORT_TX_FINISH_INTERRUPT to 0, so the
 *     HAL has no TX-finish interrupt. The driver falls back to the TX-FIFO-empty
 *     interrupt, which is asserted while the FIFO is still empty at TX start
 *     (before DMA fills it), so tx_done_sem fires almost immediately -- long
 *     before the frame is on air. It cannot gate a state change, so we must
 *     instead wait out the frame's computed airtime.
 */
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#define IR_TX_DONE_IS_PRECISE 0
#else
#define IR_TX_DONE_IS_PRECISE 1
#endif

/*
 * RX captures the raw 38 kHz carrier: every carrier period arrives as a short
 * mark (carrier ON, ~1/3 duty) followed by a short space (carrier OFF). Each
 * count is one 40 MHz tick (25 ns), so one 38 kHz period is ~1050 counts.
 *
 * The RX callback stream-demodulates (merges consecutive carrier periods into
 * one logical MARK, treats a gap > IR_GAP_THRESHOLD as a SPACE) into a single
 * group buffer, and signals main on IR_RX_STOPPED -- the idle gap between two
 * waveform groups. main then prints the group and re-transmits it (echo): the
 * demodulated symbols are converted back to TX carrier-period words and sent.
 * Keep the TX interval long enough that print + echo finish before the next
 * incoming frame.
 */
#define IR_COUNT_PER_US    40U       /* 40 MHz -> 40 counts per microsecond */
#define IR_CARRIER_CNT     1050U     /* counts in one 38 kHz carrier period */
#define IR_GAP_THRESHOLD   3000U     /* space > this (75 us) => logical gap */
#define IR_MARK_MAX_CNT    4000000U  /* mark > 100 ms is a bogus (startup) value */
#define GRP_MAX            256U      /* max logical symbols kept per group */

struct ir_sym {
	uint32_t cnt; /* duration in 40 MHz counts */
	bool mark;    /* true = carrier burst, false = gap */
};

static struct ir_sym grp[GRP_MAX];
static uint16_t grp_len;

/* Echo staging: demodulated group converted back to TX carrier-period words. */
static uint32_t echo_buf[GRP_MAX];
static uint16_t echo_len;
static uint32_t echo_periods; /* total carrier periods in echo_buf (for airtime) */

K_SEM_DEFINE(rx_group_sem, 0, 1); /* IR_RX_STOPPED -> main */
K_SEM_DEFINE(tx_done_sem, 0, 1);  /* TX chain finished -> main */

/* Streaming demod state, persists across callback invocations. */
static uint32_t s_mark_cnt; /* accumulated carrier-on envelope (counts) */
static uint32_t s_pend_sp;  /* pending short space inside a burst (counts) */
static bool s_in_mark;

#define TX_LEN 100
uint32_t tx_buf[TX_LEN];

/* Current TX frame being streamed out (chained through ir_tx_cb). */
static uint32_t *cur_tx_buf;
static uint32_t cur_tx_len;
static uint32_t tx_sent;

const struct device *const ir_dev = DEVICE_DT_GET(DT_NODELABEL(ir));

static void tx_frame(uint32_t *buf, uint32_t len)
{
	cur_tx_buf = buf;
	cur_tx_len = len;
	tx_sent = 0;
	k_sem_reset(&tx_done_sem); /* clear any stale give so the next wait is fresh */
	ir_tx(ir_dev, buf, len);
}

/*
 * Wait for the frame just handed to tx_frame() to finish before we reconfigure
 * the peripheral. On ICs with a real TX-finish interrupt (IR_TX_DONE_IS_PRECISE)
 * block on tx_done_sem and wake the instant TX completes; the timeout is only a
 * safety net. Elsewhere tx_done_sem is unreliable (see IR_TX_DONE_IS_PRECISE),
 * so wait out the frame's computed on-air time instead.
 */
static void tx_wait_frame_done(uint32_t airtime_ms)
{
#if IR_TX_DONE_IS_PRECISE
	k_sem_take(&tx_done_sem, K_MSEC(airtime_ms + 200U));
#else
	k_sleep(K_MSEC(airtime_ms));
#endif
}

static void ir_emit(bool mark, uint32_t cnt)
{
	if (mark && cnt > IR_MARK_MAX_CNT) {
		return; /* drop startup/glitch marks that are impossibly long */
	}
	if (grp_len < GRP_MAX) {
		grp[grp_len].cnt = cnt;
		grp[grp_len].mark = mark;
		grp_len++;
	}
}

void ir_tx_cb(const struct device *dev, struct ir_event *evt, void *user_data)
{
	tx_sent += evt->data.tx.len;
	if (tx_sent < cur_tx_len) {
		ir_tx(ir_dev, &(cur_tx_buf[tx_sent]), cur_tx_len - tx_sent);
	} else {
		k_sem_give(&tx_done_sem);
	}
	printf("[%s] dev:%s evt:%d tx_len:%d\n", __func__, dev->name, evt->type,
	       evt->data.tx.len);
}

void ir_rx_cb(const struct device *dev, struct ir_event *evt, void *user_data)
{
	for (uint32_t i = 0; i < evt->data.rx.len; i++) {
		uint32_t dur = evt->data.rx.buf[i] & 0x7fffffff;
		bool carrier = (evt->data.rx.buf[i] & 0x80000000) != 0;

		if (carrier) {
			s_mark_cnt += dur + s_pend_sp;
			s_pend_sp = 0;
			s_in_mark = true;
		} else if (dur <= IR_GAP_THRESHOLD) {
			s_pend_sp += dur; /* short gap inside a carrier burst */
		} else {
			if (s_in_mark) {
				ir_emit(true, s_mark_cnt);
				s_mark_cnt = 0;
				s_in_mark = false;
			}
			ir_emit(false, dur);
			s_pend_sp = 0;
		}
	}

	/* IR_RX_STOPPED = idle gap between two groups: hand off to main. */
	if (evt->type == IR_RX_STOPPED) {
		if (s_in_mark) {
			ir_emit(true, s_mark_cnt);
			s_mark_cnt = 0;
			s_in_mark = false;
		}
		s_pend_sp = 0;
		k_sem_give(&rx_group_sem);
	}
}

/* Print the demodulated group and convert it to TX carrier-period words. */
static void group_print_and_stage(void)
{
	echo_len = 0;
	echo_periods = 0;

	printf("==== waveform group: %u symbols ====\n", grp_len);
	for (uint16_t i = 0; i < grp_len; i++) {
		uint32_t us = grp[i].cnt / IR_COUNT_PER_US;
		uint32_t periods = (grp[i].cnt + IR_CARRIER_CNT / 2U) / IR_CARRIER_CNT;

		if (periods == 0U) {
			periods = 1U;
		} else if (periods > 0x3FFFFFFFU) {
			periods = 0x3FFFFFFFU;
		}

		if (grp[i].mark) {
			printf("  MARK  %6u us (%u cyc)\n", us, periods);
		} else {
			printf("  SPACE %6u us\n", us);
		}

		if (echo_len < GRP_MAX) {
			echo_buf[echo_len++] = (grp[i].mark ? 0x80000000U : 0U) | periods;
			echo_periods += periods;
		}
	}

	if (echo_len) {
		echo_buf[echo_len - 1] |= BIT(30); /* end-of-TX marker on the last word */
	}
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD);

	tx_buf[0] = 0x80000000 | 0x156; /* 342 about 9ms */
	tx_buf[1] = 0x00000000 | 0xAB;  /* 171 about 4.5ms */
	for (uint16_t i = 2; i < TX_LEN - 1;) {
		tx_buf[i] = 0x80000000 | 0x15;     /* 21  about 565us */
		tx_buf[i + 1] = 0x00000000 | 0x15; /* 21  about 560us */
		i += 2;
	}

	if (TX_LEN & BIT(0)) {
		tx_buf[TX_LEN - 1] = BIT(30) | 0x80000000 | 0x15;
	} else {
		tx_buf[TX_LEN - 1] = BIT(30) | 0x00000000 | 0x15;
	}

	ir_set_freq(ir_dev, 38000, 3);
	ir_tx_enable(ir_dev, ir_tx_cb, NULL);

	uint32_t i = 5;

	while (i--) {
		tx_frame(tx_buf, TX_LEN);
		k_sleep(K_MSEC(1000));
	}

	/* Enable RX; on each group, print it and echo it back out over TX. */
	ir_set_freq(ir_dev, 40000000, 3);
	ir_rx_enable(ir_dev, ir_rx_cb, NULL, 256, 800000);
	printf("IR RX enabled, waiting for frames from the TX board...\n");

	while (1) {
		k_sem_take(&rx_group_sem, K_FOREVER);

		/* Stop RX so grp[] is stable while we process and re-transmit. */
		struct ir_event_rx tail = {0};

		ir_rx_disable(ir_dev, &tail);
		group_print_and_stage();
		grp_len = 0;
		s_mark_cnt = 0;
		s_pend_sp = 0;
		s_in_mark = false;

		if (echo_len) {
			uint32_t airtime_ms = echo_periods / 38U + 30U; /* +30ms margin */

			printf("echo: re-transmitting %u words (~%u ms airtime)\n",
				echo_len, airtime_ms);
			ir_set_freq(ir_dev, 38000, 3);
			ir_tx_enable(ir_dev, ir_tx_cb, NULL);
			tx_frame(echo_buf, echo_len);
			tx_wait_frame_done(airtime_ms);
		}

		/* Back to receiving. */
		ir_set_freq(ir_dev, 40000000, 3);
		ir_rx_enable(ir_dev, ir_rx_cb, NULL, 256, 800000);
	}

	return 0;
}
