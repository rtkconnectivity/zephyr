"""Serial port configuration for PM device tests.

Configuration is read from serial_ports.ini file in the same directory.
Priority: command line > environment variables > config file > defaults.
"""

import configparser
import os
from pathlib import Path

# Command line options (set by conftest.py)
_cmdline_shell_port = None
_cmdline_shell_baudrate = None
_cmdline_dma_port = None
_cmdline_dma_baudrate = None


def _load_config():
    """Load configuration from serial_ports.ini."""
    config_file = Path(__file__).parent / "serial_ports.ini"
    config = configparser.ConfigParser()

    if config_file.exists():
        config.read(config_file)
    else:
        # Fallback to defaults if file doesn't exist
        config["shell_uart"] = {"port": "COM10", "baudrate": "2000000"}
        config["dma_uart"] = {"port": "COM11", "baudrate": "2000000"}

    return config


# Cache the config
_config = None


def _get_config():
    global _config
    if _config is None:
        _config = _load_config()
    return _config


def get_shell_uart_port() -> str:
    """Get shell UART port.

    Priority: command line > env var > config file > default (COM10)
    """
    global _cmdline_shell_port
    if _cmdline_shell_port:
        return _cmdline_shell_port

    if "SHELL_UART_PORT" in os.environ:
        return os.environ["SHELL_UART_PORT"]

    config = _get_config()
    return config.get("shell_uart", "port", fallback="COM10")


def get_shell_uart_baudrate() -> int:
    """Get shell UART baudrate.

    Priority: command line > env var > config file > default (2000000)
    """
    global _cmdline_shell_baudrate
    if _cmdline_shell_baudrate:
        return _cmdline_shell_baudrate

    if "SHELL_UART_BAUDRATE" in os.environ:
        return int(os.environ["SHELL_UART_BAUDRATE"])

    config = _get_config()
    return config.getint("shell_uart", "baudrate", fallback=2000000)


def get_dma_uart_port() -> str:
    """Get DMA UART port.

    Priority: command line > env var > config file > default (COM11)
    """
    global _cmdline_dma_port
    if _cmdline_dma_port:
        return _cmdline_dma_port

    if "DMA_UART_PORT" in os.environ:
        return os.environ["DMA_UART_PORT"]

    config = _get_config()
    return config.get("dma_uart", "port", fallback="COM11")


def get_dma_uart_baudrate() -> int:
    """Get DMA UART baudrate.

    Priority: command line > env var > config file > default (2000000)
    """
    global _cmdline_dma_baudrate
    if _cmdline_dma_baudrate:
        return _cmdline_dma_baudrate

    if "DMA_UART_BAUDRATE" in os.environ:
        return int(os.environ["DMA_UART_BAUDRATE"])

    config = _get_config()
    return config.getint("dma_uart", "baudrate", fallback=2000000)