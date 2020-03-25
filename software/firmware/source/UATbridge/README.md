## UAT Bridge

This folder contains source code of **UAT Bridge** firmware for Texas Instruments CC1310 RF SoC. <sup>1</sup><br>
The code is essential part of [SoftRF project](https://github.com/lyusupov/SoftRF).<br>

<sup>1</sup> - _mass production of CC1312R IC (direct successor of CC1310) is scheduled by TI on Q1 of 2019_.<br>
&nbsp;&nbsp;&nbsp; _Announced specs of the CC1312R are promising to make **1090ES** <sub>(worldwide ADS-B standard)</sub> reception possible as well_.<br>

## Purpose

Primary purpose of the **UAT bridge** is to receive ADS-B traffic information on UAT (978 MHz) aviation frequency,<br>
then to re-broadcast the data using one of supported RF ISM band protocols.<br>

For the list of the ISM protocols, please, read [SoftRF specs](https://github.com/lyusupov/SoftRF#compatibility-1).<br>

Traffic information is re-broadcasted at very low power setting, so reception coverage is limited to a close vicinity of
the aircraft, equipped with the **UAT Bridge**. Other aircrafts nearby, such as gliders on the grid, have a chance to receive the information as well.<br>  

The "**bridge**" is one-way only. No re-broadcasting from ISM band into aviation band (UAT) is allowed. Thus, it does **not** transmit anything on 978 MHz.<br>

For the benefit of most of pilots, **receiving device is not necessarily has to be one of [SoftRF](https://github.com/lyusupov/SoftRF) units**.<br>
Other make and models, such as SkyTraxx, OGN Tracker, PilotAware or FLARM should suffice.<br> 

## Hardware

DIY **UAT bridge** hardware consists of:
1. [SoftRF-UAT](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced) main board, and
2. [SoftRF-LoRa](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module) daughterboard.

<br>
<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/SoftRF-UAT-2.jpg" height="199" width="400"><br>
<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/SoftRF-UAT-3.jpg" height="234" width="400">


## Alternative use

The firmware auto-senses presence of connected [SoftRF-LoRa](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module) daughterboard.<br>
When the daughterboard is not attached - remaining [SoftRF-UAT](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced) board fallbacks into **UAT Receiver** operation mode automatically.<br>

**UAT Receiver** makes reception of UAT signals, then transmits serial data frames over UART @ 2 Mbps.<br>
This data rate and format of the frames were choosen to be compatible with [Stratux UATRadio, Low Power v3](https://www.amazon.com/dp/B07JNSHCLQ/).

## Build instructions

To build firmware binary from source code, please, follow [these instructions](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#cc13xx).

## Validation

Reception capability of the **UAT Receiver** mode was validated both with:

1. **SoftRF WebUI** on ESP32:

<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/uat-webui-1.jpg)

<br>

2. and genuine [**Stratux**](http://stratux.me/) software on a Raspberry Pi:

<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/UATbridge_Stratux.JPG)

Reception capability of real life UAT traffic in **Normal** mode had been validated on **Sun-n-Fun** (2019) Fly-In & Expo in Lakeland, FL:

![](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/uat-normal-8.jpg)

<p><img src="https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/uat-normal-7.jpg" height="710" width="400">&nbsp;<img src="https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/uat-normal-6.jpg" height="710" width="400"></p>

## Known limitations

1. reception of UAT signals is currently limited to "downlink" (air-to-air) packets only. So, realtime weather and other "bells and whistles" of the UAT protocol are not available ;
2. due to limited maximum speed of ESP8266's UART, this platform is not able to use **UAT receiver** right now. ESP32 and Raspberry Pi are Ok ;
3. RF settings of CC1310 IC are not optimal yet. This causes average 10-20% packets loss ratio at this time ;
4. "Stock" EByte E70 module has built-in passive HF filter to meet FCC compliance. Because of that, sensitivity @ 978 MHz is reduced by 10-15 dB. To achieve full performance, you need to open the shield and solder a bypass.
