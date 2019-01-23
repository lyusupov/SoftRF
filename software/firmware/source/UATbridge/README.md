## UAT Bridge

This folder contains source code of **UAT Bridge** firmware for Texas Instruments CC1310 RF SoC. <sup>1</sup><br>
The code is essential part of [SoftRF project](https://github.com/lyusupov/SoftRF).<br>

<sup>1</sup> - _mass production of CC1312R IC (direct successor of CC1310) is scheduled on Q1 of 2019_.<br>
 _Announced specs of the CC1312R are promising to make 1090ES <sub>(worldwide ADS-B standard)</sub> reception possible as well_.<br>

## Purpose

Primary purpose of the **UAT bridge** is to receive ADS-B traffic information on UAT (978 MHz) aviation frequency,<br>
then to re-broadcast the data using one of supported RF ISM band protocols.<br>

For the list of the ISM protocols, please, read SoftRF specs.<br>

Traffic information is rebroadcasted at very low power setting, so reception coverage is limited to a close vicinity of
the aircraft,<br>
equipped with the **UAT Bridge**. Other aircrafts nearby, such as gliders on the grid, have a chance to receive the data as well.<br>  

The "**bridge**" is one-way only. No re-broadcasting from ISM into UAT is allowed. Thus, it does **not** transmit anything on 978 MHz.<br>

For the benefit of most of pilots, receiving device is not necessarily has to be one of [SoftRF](https://github.com/lyusupov/SoftRF) units.<br>
Other make and models, such as SkyTraxx, OGN Tracker, PilotAware or FLARM should suffice.<br> 

## Hardware

DIY **UAT bridge** hardware consists of:
1) [SoftRF-UAT](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced) main board, and
2) [SoftRF-LoRa](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module) daughterboard.

## Alternative use

The firmware auto-senses presence of connected [SoftRF-LoRa](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module) daughterboard.<br>
When the daughterboard is not attached - remaining [SoftRF-UAT](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced) board does fallback into **UAT Receiver** operation mode.<br>

**UAT Receiver** does reception of UAT signals then transmits serial data packets over UART on 2 Mbps.<br>
The data rate and format of the packets is choosen to be compatible with [Stratux UATRadio, Low Power v3](https://www.amazon.com/dp/B07JNSHCLQ/).

## Build instructions

The firmware is to be built on a Linux i686 host.<br>
[Energia](http://energia.nu/download/) IDE has to be pre-installed first.<br>

1) create all necessary symbolic links by doing **make links**:

```
$ make links
Creating symlink Platform_CC13XX.cpp  -->  ../SoftRF/Platform_CC13XX.cpp
Creating symlink SoCHelper.cpp  -->  ../SoftRF/SoCHelper.cpp
Creating symlink RFHelper.cpp  -->  ../SoftRF/RFHelper.cpp
Creating symlink GNSSHelper.cpp  -->  ../SoftRF/GNSSHelper.cpp

< ... skipped ... >
```

2) build the firmware down to ELF binary code:

```
$ make build
energia --verify --verbose-build UATbridge.ino
Picked up JAVA_TOOL_OPTIONS:
Loading configuration...
Initializing packages...
Preparing boards...
Verifying...

< ... skipped ... >
```

3) if you want to use [**cc2538-bsl.py**](https://github.com/JelmerT/cc2538-bsl) tool to put the firmware into flash memory of CC1310 IC via **boot loader**, run **make ihex**:

```
$ make ihex

< ... skipped ... >
``` 

## Validation

Reception capability of the **UAT Receiver** mode was validated both with:

1) **SoftRF WebUI**:

<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/uat-webui-1.jpg)

<br>

2) and genuine [**Stratux**](http://stratux.me/) software on Raspberry Pi:

<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/UATbridge_Stratux.JPG)

