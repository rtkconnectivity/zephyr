.. _i2s_bee:

I2S BEE Driver Sample
######################

Overview
********

This sample demonstrates the usage of the I2S and Audio Codec drivers for
Realtek BEE family SoCs (rtl87x2j, rtl8752h, rtl87x2g).

It includes three usage scenarios:

- **i2s_codec_rx**: I2S codec receive (capture) using both the audio codec
  and I2S driver APIs.
- **i2s_codec_tx**: I2S codec transmit (playback) using both the audio codec
  and I2S driver APIs.
- **i2s_tx**: I2S-only transmit (without the audio codec).

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/i2s_bee
   :board: rtl87x2j_evb_rtl8762jth
   :goals: build
   :compact:

Sample Output
=============

.. code-block:: console

    I2S BEE sample started
    rx_size 256
