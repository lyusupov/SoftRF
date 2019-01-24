## UAT Bridge

This folder contains source code of **UAT Bridge** firmware for Texas Instruments CC1310 RF SoC. <sup>1</sup><br>
The code is essential part of [SoftRF project](https://github.com/lyusupov/SoftRF).<br>

<sup>1</sup> - _mass production of CC1312R IC (direct successor of CC1310) is scheduled on Q1 of 2019_.<br>
 _Announced specs of the CC1312R are promising to make **1090ES** <sub>(worldwide ADS-B standard)</sub> reception possible as well_.<br>

## Purpose

Primary purpose of the **UAT bridge** is to receive ADS-B traffic information on UAT (978 MHz) aviation frequency,<br>
then to re-broadcast the data using one of supported RF ISM band protocols.<br>

For the list of the ISM protocols, please, read [SoftRF specs](https://github.com/lyusupov/SoftRF#compatibility-1).<br>

Traffic information is re-broadcasted at very low power setting, so reception coverage is limited to close vicinity of
the aircraft, equipped with the **UAT Bridge**. Other aircrafts nearby, such as gliders on the grid, have a chance to receive the data as well.<br>  

The "**bridge**" is one-way only. No re-broadcasting from ISM into UAT is allowed. Thus, it does **not** transmit anything on 978 MHz.<br>

For the benefit of most of pilots, receiving device is not necessarily has to be one of [SoftRF](https://github.com/lyusupov/SoftRF) units.<br>
Other make and models, such as SkyTraxx, OGN Tracker, PilotAware or FLARM should suffice.<br> 

## Hardware

DIY **UAT bridge** hardware consists of:
1. [SoftRF-UAT](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced) main board, and
2. [SoftRF-LoRa](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module) daughterboard.

## Alternative use

The firmware auto-senses presence of connected [SoftRF-LoRa](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module) daughterboard.<br>
When the daughterboard is not attached - remaining [SoftRF-UAT](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced) board fallbacks into **UAT Receiver** operation mode automatically.<br>

**UAT Receiver** makes reception of UAT signals, then transmits serial data packets over UART @ 2 Mbps.<br>
The data rate and format of the packets was choosen to be compatible with [Stratux UATRadio, Low Power v3](https://www.amazon.com/dp/B07JNSHCLQ/).

## Build instructions

To build firmware binary from source code, please, use [these instructions](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#cc13xx).

## Validation

Reception capability of the **UAT Receiver** mode was validated both with:

1. **SoftRF WebUI** on ESP32:

<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/uat-webui-1.jpg)

<br>

2. and genuine [**Stratux**](http://stratux.me/) software on a Raspberry Pi:

<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/UATbridge_Stratux.JPG)

## Known limitations

1. **brige** mode is not well tested right now. **receiver** mode is known to be Ok ;
2. reception of UAT signals is currently limited to "downlink" (air-to-air) frames only. So, realtime weather and other "bells and whistles" of the UAT protocol are not available ;
3. due to limited maximum speed of ESP8266's UART, this platform is not able to use **UAT receiver** right now. ESP32 and Raspberry Pi are Ok ;
4. RF settings of CC1310 IC are sub-optimal yet. This causes 20-30% packets loss ratio ;
5. Stock E70 module has built-in passive HF filter to meet FCC compliance. Because of that, sensitivity on 978 MHz is reduced on 10-15 dB. To achieve full performance, you need to open the shield and solder a bypass.  
