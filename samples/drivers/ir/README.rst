.. zephyr:code-sample:: bee-ir
   :name: Realtek Bee IR TX/RX
   :relevant-api: ir_interface

   Transmit and receive infrared (IR) frames on Realtek Bee SoCs.

Overview
********

This sample demonstrates how to use the Realtek Bee IR driver to:

#. Transmit a standard NEC infrared frame using a 38 kHz carrier.
#. Switch the peripheral to receive mode and capture incoming frames.

The IR controller consumes/produces an array of 32-bit words. Bit 31 selects
the carrier state (``1`` = carrier on / *mark*, ``0`` = carrier off / *space*)
and the remaining bits encode the duration expressed in carrier periods.

Both transmit and receive paths use DMA when a ``dmas`` property is present on
the ``ir`` node; otherwise the driver falls back to interrupt-driven FIFO
transfers.

Requirements
************

A board based on one of the supported Realtek Bee SoCs:

* :ref:`rtl8752h_evb`
* :ref:`rtl87x2g_evb_a`
* :ref:`rtl87x2j_evb`

An external IR LED must be wired to the ``ir-tx`` pin and an IR receiver/photo
diode to the ``ir-rx`` pin as configured in the board overlay.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/ir
   :board: rtl87x2j_evb_rtl8762jth
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console

   IR sample on ir@40015800
   IR TX completed (67 words sent)
   IR receiver enabled, waiting for frames...
   IR RX received 256 words
