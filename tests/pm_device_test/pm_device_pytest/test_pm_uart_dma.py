import random
import time

import serial
import serial.tools.list_ports

from pm_device_pytest.pm_device_board_config import (
    get_dma_uart_baudrate,
    get_dma_uart_port,
)
from pm_device_pytest.pm_device_common import (
    assert_delta_close_to,
    assert_single_dlps_present,
)

DMA_PORT = get_dma_uart_port()
DMA_BAUDRATE = get_dma_uart_baudrate()
TEST_DATA = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijklmnopqrstuvwxyz"


class SerialDMADUT:
    def __init__(self, port, baudrate, timeout=0.1):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def write(self, data):
        self.ser.write(data)
        self.ser.flush()

    def read(self, size, timeout=5.0):
        self.ser.timeout = timeout
        data = self.ser.read(size)
        self.ser.timeout = 0.1
        return data

    def flush_input(self, timeout=0.5):
        end = time.time() + timeout
        while time.time() < end:
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
            else:
                break


def dma_dut():
    available = [p.device for p in serial.tools.list_ports.comports()]
    if DMA_PORT not in available:
        raise RuntimeError(f"[DMA DUT] {DMA_PORT} unavailable. Available: {available}")

    dma = SerialDMADUT(port=DMA_PORT, baudrate=DMA_BAUDRATE, timeout=0.1)
    try:
        yield dma
    finally:
        dma.close()


def _send_receive_dma(dut, data):
    dma = dut.dma
    dma.flush_input(timeout=0.5)
    dma.write(data)
    time.sleep(0.1)
    received = dma.read(len(data), timeout=5.0)
    return received, (received == data)


def test_pm_uart_dma(dut, dma_dut):
    dut.dma = dma_dut
    rand_delay_ms = random.randint(1000, 5000)

    dut.write(b"pm_test uartdma\r")
    logs_pre = dut.readlines_until(regex=r"send some data from dma uart", timeout=10.0)
    assert "send some data from dma uart" in "\n".join(logs_pre)

    received, ok = _send_receive_dma(dut, TEST_DATA)
    logs_rx = dut.readlines_until(regex=r"uart dma rx \d+ bytes", timeout=5.0)
    assert "uart dma rx" in "\n".join(logs_rx) and "bytes" in "\n".join(logs_rx)
    assert ok, f"Data mismatch before DLPS"

    logs_dlps = dut.readlines_until(regex=r"type on shell to wakeup", timeout=10.0)
    assert "before enter dlps" in "\n".join(logs_dlps)

    time.sleep(rand_delay_ms / 1000.0)
    dut.write(b"\r")

    logs_end = dut.readlines_until(regex=r"after exit dlps", timeout=10.0)
    assert "after exit dlps" in "\n".join(logs_end)

    logs_ready = dut.readlines_until(regex=r"send some data from dma uart", timeout=10.0)
    assert "send some data from dma uart" in "\n".join(logs_ready)

    received2, ok2 = _send_receive_dma(dut, TEST_DATA)
    logs_rx2 = dut.readlines_until(regex=r"uart dma rx \d+ bytes", timeout=5.0)
    assert "uart dma rx" in "\n".join(logs_rx2) and "bytes" in "\n".join(logs_rx2)
    assert ok2, f"Data mismatch after DLPS"

    logs_post = dut.readlines_until(regex=r".*~\$ *$", timeout=10.0)

    all_logs = logs_pre + logs_dlps + logs_end + logs_ready + logs_post
    session = assert_single_dlps_present(all_logs)
    assert_delta_close_to(session, target_ms=rand_delay_ms, tolerance_ms=300, reason="uartdma")