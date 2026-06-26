.. _rtl8777g_dongle:

RTL8777G Dongle Evaluation Board
#################################

Overview
********

The RTL8777G Dongle Evaluation Board provides a hardware environment for user development, including:

- 1x Reset button
- 4x Buttons
- 3x LEDs
- 1x Micro USB-B port (connected to FT232RL USB-to-UART chip)
- 1x USB-A connector
- 25x Exposed pins (user-configurable)
- On-board ceramic antenna and IPEX socket (for optional external antenna)
- 5V to 3.3V LDO
- UART programming interface

The RTL8777G Dongle Evaluation Board features the RTL8777G SoC.
RTL8777G is an ultra-low power wireless microcontroller that supports Bluetooth 5.3/Zigbee/Thread/Matter protocols.

.. note::
    The RTL8777G is part of the RTL87x2G series chip family (despite the naming inconsistency).

Hardware
********

SoC Series
==========

The RTL8777G/RTL87x2G series comprises various chip types, each supporting different hardware features.

Below are the common hardware features:

- Realtek KM4 core compatible with Arm Cortex-M55, running at 40MHz (Maximum 125MHz)
- M-profile Vector Extension (MVE) for vector computation
- 32KB I-Cache, 16KB D-Cache, and 384KB SRAM
- Hardware Keyscan / Quad Decode
- Flash On-The-Fly Decryption
- Embedded IR TX/RX
- ISO7816 Interface
- SPIC/SPI_m/SPI_s/SDIO/SD (eMMC)
- Low Power Comparator
- 8-Channel AUXADC
- CAN Bus
- I2S/DAC/AMIC/DMIC/PDM
- USB 2.0 High-Speed Interface

Connections and IOs
===================

The UART2 console uses P3_0 (TX) and P3_1 (RX), connected through the on-board FT232RL
USB-to-UART converter accessible via the Micro USB-B port.

LEDs:

- LED D1 Blue: GPIOA_21 (P2_0), active high
- LED D2 Red:  GPIOA_18 (P8_2, XO32K), active high
- LED D3 Green: GPIOA_17 (P8_1, XI32K), active high

Buttons (active low, internal pull-up):

- SW1: GPIOB_22 (P9_1)
- SW2: GPIOB_21 (P9_0)
- SW3: GPIOB_8  (P4_3)
- SW4: GPIOB_7  (P4_2)

System Clock
============

The RTL8777G SoC is configured to use the internal 32KHz clock as a source for the
system clock, with the CPU running at 40MHz.

Serial Port
===========

The RTL8777G SoC has 6 UARTs. By default, UART2 is configured for the console and
log output at 115200 baud.

Programming and Debugging
*************************

Flashing
========

Before flashing, ensure the SoC is in download mode:

1. Use a jumper to connect the LOG pin (P0_3) with GND.
2. Press the RST button once to reboot the SoC and enter download mode.
3. Flash using mpcli or JLink.
4. Remove the jumper and press RST to enter normal mode.

To build and flash the :zephyr:code-sample:`hello_world` sample:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rtl8777g_dongle
   :goals: build flash

References
**********

- `RTL87x2G Introduction <https://www.realmcu.com/en/Home/Product/RTL8762G-RTL877xG-Series>`_
