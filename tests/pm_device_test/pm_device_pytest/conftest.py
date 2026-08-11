import re
import time
from pathlib import Path
from typing import List, Optional, TextIO

import pytest
import serial
import serial.tools.list_ports

from pm_device_pytest.pm_device_board_config import (
    get_dma_uart_baudrate,
    get_dma_uart_port,
    get_shell_uart_baudrate,
    get_shell_uart_port,
)


def pytest_addoption(parser):
    """Add custom command line options."""
    parser.addoption(
        "--shell-port",
        action="store",
        default=None,
        help="Shell UART port (e.g., COM10)",
    )
    parser.addoption(
        "--shell-baudrate",
        action="store",
        type=int,
        default=None,
        help="Shell UART baudrate (e.g., 2000000)",
    )
    parser.addoption(
        "--dma-port",
        action="store",
        default=None,
        help="DMA UART port (e.g., COM21)",
    )
    parser.addoption(
        "--dma-baudrate",
        action="store",
        type=int,
        default=None,
        help="DMA UART baudrate (e.g., 2000000)",
    )


@pytest.fixture(scope="session", autouse=True)
def _parse_cmdline_options(request):
    """Parse command line options and store in module for board_config to use."""
    from pm_device_pytest import pm_device_board_config

    if request.config.getoption("--shell-port"):
        pm_device_board_config._cmdline_shell_port = request.config.getoption("--shell-port")
    if request.config.getoption("--shell-baudrate"):
        pm_device_board_config._cmdline_shell_baudrate = request.config.getoption("--shell-baudrate")
    if request.config.getoption("--dma-port"):
        pm_device_board_config._cmdline_dma_port = request.config.getoption("--dma-port")
    if request.config.getoption("--dma-baudrate"):
        pm_device_board_config._cmdline_dma_baudrate = request.config.getoption("--dma-baudrate")


class SerialShellDUT:
    ANSI_RE = re.compile(r"\x1B\[[0-9;]*[mK]")

    def __init__(
        self,
        port: str,
        baudrate: int = 2000000,
        timeout: float = 0.1,
        prompt_regex: str = r".*~\$ *$",
        log_file: Optional[str] = None,
        encoding: str = "utf-8",
        errors: str = "ignore",
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.prompt_regex = re.compile(prompt_regex)
        self.encoding = encoding
        self.errors = errors

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )

        self.all_logs: List[str] = []

        self._log_fp: Optional[TextIO] = None
        if log_file is not None:
            path = Path(log_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._log_fp = path.open("a", encoding="utf-8")
            self._log_fp.write("\n===== NEW SESSION =====\n")
            self._log_fp.flush()

    def _log_line(self, text: str):
        if not text:
            return

        if self._log_fp is not None:
            self._log_fp.write(text + "\n")
            self._log_fp.flush()

        self.all_logs.append(text)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self._log_fp is not None:
            self._log_fp.close()
            self._log_fp = None

    def write(self, data: bytes):
        self.ser.write(data)
        self.ser.flush()

    def readline(self) -> str:
        raw = self.ser.readline()

        if not raw:
            return ""

        try:
            text = raw.decode(self.encoding, errors=self.errors).rstrip("\r\n")
        except Exception:
            text = ""

        if text:
            clean = self.ANSI_RE.sub("", text)
            self._log_line(clean)
            return clean

        return ""

    def readlines_until(self, regex, timeout: float = 5.0) -> List[str]:
        if isinstance(regex, re.Pattern):
            pattern = regex
        else:
            pattern = re.compile(regex)

        end_time = time.time() + timeout
        lines: List[str] = []

        while time.time() < end_time:
            text = self.readline()
            if not text:
                continue

            lines.append(text)

            if pattern.search(text):
                break

        return lines

    def read_until_prompt(self, timeout: float = 5.0) -> List[str]:
        return self.readlines_until(self.prompt_regex, timeout=timeout)

    def flush_input(self, timeout: float = 0.5):
        end = time.time() + timeout
        while time.time() < end:
            text = self.readline()
            if not text:
                break

    def ensure_prompt(self, timeout: float = 5.0) -> List[str]:
        self.write(b"\r")
        logs = self.read_until_prompt(timeout=timeout)
        return logs

    def send_cmd(
        self,
        cmd: str,
        wait_prompt: bool = True,
        timeout: float = 10.0,
    ) -> List[str]:
        if not cmd.endswith("\n"):
            cmd_to_send = cmd + "\r\n"
        else:
            cmd_to_send = cmd

        self.write(cmd_to_send.encode(self.encoding))

        if not wait_prompt:
            return []

        logs = self.read_until_prompt(timeout=timeout)
        return logs

    def relay_on(self, channel: int):
        pass

    def relay_off(self, channel: int):
        pass


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


@pytest.fixture(scope="session")
def dut():
    """Shell UART DUT fixture."""
    target_port = get_shell_uart_port()
    target_baudrate = get_shell_uart_baudrate()

    print(f"\n[DUT] Connecting to {target_port} at {target_baudrate} baud...")

    available = [p.device for p in serial.tools.list_ports.comports()]

    if target_port not in available:
        raise RuntimeError(
            f"[DUT] {target_port} unavailable. Available serial ports: {available}"
        )

    log_dir = Path(__file__).parent / "logs"
    log_file = log_dir / "session_shell.log"

    d = SerialShellDUT(
        port=target_port,
        baudrate=target_baudrate,
        timeout=0.1,
        prompt_regex=r".*~\$ *$",
        log_file=str(log_file),
    )

    try:
        print(f"[DUT] Waiting for shell prompt...")
        logs = d.ensure_prompt(timeout=10.0)
        print(f"[DUT] Got {len(logs)} lines, first few:")
        for i, line in enumerate(logs[:5]):
            print(f"  [{i}] {line!r}")

        if not logs:
            print("[DUT] WARNING: No prompt received!")

        yield d

    finally:
        end = time.time() + 1.0
        while time.time() < end:
            text = d.readline()
            if not text:
                break

        d.close()


@pytest.fixture(scope="session")
def dma_dut():
    """DMA UART DUT fixture for uart_dma test."""
    dma_port = get_dma_uart_port()
    dma_baudrate = get_dma_uart_baudrate()

    available = [p.device for p in serial.tools.list_ports.comports()]
    if dma_port not in available:
        raise RuntimeError(f"[DMA DUT] {dma_port} unavailable. Available: {available}")

    dma = SerialDMADUT(port=dma_port, baudrate=dma_baudrate, timeout=0.1)
    try:
        yield dma
    finally:
        dma.close()