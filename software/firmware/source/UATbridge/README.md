## UAT Bridge

This folder contains source code of **UAT Bridge** firmware for Texas Instruments **CC1310** and **CC1352R** RF SoCs.<br>
The code is essential part of [SoftRF project](https://github.com/lyusupov/SoftRF).<br>

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

The firmware is to be built on a Linux x86_64 host.<br>
[Energia](http://energia.nu/download/) IDE has to be pre-installed first.<br>

1. open ``<Home>`` directory
2. create **Energia** sub-directory
3. transfer full content of **UATbridge**, **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [UATbridge](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/UATbridge) **-->** ``<Home>``/Energia/UATbridge <br>
    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**-->** ``<Home>``/Energia/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) &nbsp;&nbsp;&nbsp;&nbsp;**-->** ``<Home>``/Energia/libraries <br>

4. change directory on ``<Home>``/Energia/UATbridge
5. create all necessary symbolic links by doing **make links**:

```
$ make links
Creating symlink Platform_CC13XX.cpp  -->  ../SoftRF/Platform_CC13XX.cpp
Creating symlink SoCHelper.cpp  -->  ../SoftRF/SoCHelper.cpp
Creating symlink RFHelper.cpp  -->  ../SoftRF/RFHelper.cpp
Creating symlink GNSSHelper.cpp  -->  ../SoftRF/GNSSHelper.cpp

< ... skipped ... >
```

6. start **Energia** application
7. open **UATbridge** sketch from _File_ -> _Open_ menu
8. [install support](http://energia.nu/guide/boards/) for TI CC13XX familiy with _Tools_ -> _Board_ ->  _Boards_ _Manager..._
9. Select _Tools_ -> _Board_ ->  _LaunchPad_ _w/_ _CC1310_ _EMT_ _(48MHz)_
10. Select _Tools_ -> _Port_ ->  ``<your XDS110 port device name>``
11. Select _Tools_ -> _Programmer_ ->  _dslite_
12. try to build and upload using _Sketch_ -> _Upload_

12a. or, to build the firmware using shell prompt, do:

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

13a. inspect **/tmp** directory and sub-folders, take **UATbridge.ino.elf** file and use it with TI's **Uniflash** or **Flash Programmer 2**.<br>


14a. (Optional) if you want to use [**cc2538-bsl.py**](https://github.com/JelmerT/cc2538-bsl) tool to put the firmware into flash memory of CC1310 IC via **boot loader**, run **make ihex**:

```
$ make ihex

< ... skipped ... >
```

<br>


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
