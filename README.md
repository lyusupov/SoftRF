# SoftRF &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![Join the chat at https://gitter.im/lyusupov/SoftRF](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/lyusupov/SoftRF?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) [![Build Status](https://travis-ci.org/lyusupov/SoftRF.png?branch=master)](https://travis-ci.org/lyusupov/SoftRF "Build Status") 
Multifunctional DIY IoT-based general aviation proximity awareness system.

Features:
* 2-way raw data bridge between 868/915 MHz radio band and Wi-Fi ;
* standalone, battery powered, compatible proximity awareness instrument that fits typical 2.25 inches hole ;
* lightweight version to carry onboard of an UAV.

# Compatibility <sup>1</sup>
Type|Protocol|FLARM|OGN tracker|PilotAware|Skytraxx|SoftRF
---|---|---|---|---|---|---
Radio|FLARM AIR V6|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|<p align="center">OGNTP</p>||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>2</sup></p>
&nbsp;|<p align="center">P3I</p>|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>2</sup></p>
&nbsp;|<p align="center">UAT&nbsp;978</p>|||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>3</sup></p>
&nbsp;|<p align="center">FANET+</p>||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>2</sup></p>
Data|FLARM NMEA|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|Garmin GDL90|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|<p align="center">MAVLINK</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|<p align="center">Dump1090 &nbsp;<sup>4</sup></p>|||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>

<sup>1</sup> - it is necessary for a reader to distinguish the difference between statement "**compatible**" and statement "**fully compatible**".<br>
&nbsp;&nbsp;&nbsp;&nbsp; SoftRF implements only a reasonable minimum of the protocols specs. No "bells and whistles" so far.<br>
<sup>2</sup> - valid for [Prime Edition Mark II](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) ; valid for **Standalone** and **UAV** **Editions** with optional DIY [SoftRF LoRa RF module](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module)<br>
<sup>3</sup> - [**Reception**](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/UATbridge) of traffic 'downlink' frames only. Valid for **Standalone Edition** with optional DIY [SoftRF UAT module](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced)<br>
<sup>4</sup> - also known as "raw ADS-B"<br>

# Models
Model|Platform|First appearance|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Status&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|Comment
---|:---:|:---:|:---:|---
1&nbsp;[Prime](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition)<br>2&nbsp;[Standalone](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)<br>3&nbsp;[UAV](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)|[Espressif<br>ESP8266](https://en.wikipedia.org/wiki/ESP8266)|Q4 2015|![](https://placehold.it/140x70/00A000/000000?text=Good)|[Prime](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition) model is no longer supported - use [Prime MkII](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) instead.<br>ESP8266 platform will be phased out through year 2019 in favour of ESP32.
1&nbsp;[Prime Mark II](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII)<br>2&nbsp;[Standalone](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition) + [adapter](https://github.com/lyusupov/ESP32-NODEMCU-ADAPTER)<br>3&nbsp;[UAV](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)<br>4&nbsp;[SkyView EZ](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ)|[Espressif<br>ESP32](https://en.wikipedia.org/wiki/ESP32)|Q1 2018|![](https://placehold.it/140x70/00A000/000000?text=Good)|Today's best platform
1&nbsp;[Raspberry Edition](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)<br>2&nbsp;SkyView Pi|[Broadcom<br>BCM283X<br>(Raspberry Pi)](https://en.wikipedia.org/wiki/Raspberry_Pi)|Q4 2018|![](https://placehold.it/140x40/c5f015/000000?text=May+need)<br>![](https://placehold.it/140x40/c5f015/000000?text=improvements)|Good for use together with RTL-SDR dongles to achieve additional 1090ES (and 978UAT) ADS-B air traffic reception.
[UAT module](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced)|[Texas Instruments<br>CC13XX](http://www.ti.com/product/cc1310)|Q1 2019|![](https://placehold.it/140x40/c5f015/000000?text=May+need)<br>![](https://placehold.it/140x40/c5f015/000000?text=improvements)<!-- ![](https://placehold.it/140x70/FFFF00/000000?text=In+progress)-->|Resources of this platform are not sufficient to fit full SoftRF firmware.<br>Unique RF radio specs are useful for [UAT reception](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/UATbridge).

# Documentation

* SoftRF overview
    * [Part 1](https://github.com/lyusupov/SoftRF/blob/master/documents/SoftRF-release-1.pdf) (PDF, December 2015)
    * [Part 2](https://github.com/lyusupov/SoftRF/raw/master/documents/SoftRF-release-2.pdf) (PDF, March 2017)
* [Prime Edition MkII](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII.-Quick-start)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/v5)
* [Standalone Edition](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)
    * [Specifications](https://github.com/lyusupov/SoftRF/wiki/Standalone-Specifications)
    * [Shield](https://github.com/lyusupov/SoftRF/wiki/Standalone-Shield)
    * [Modules](https://github.com/lyusupov/SoftRF/wiki/Standalone-Modules)
    * [Assembly](https://github.com/lyusupov/SoftRF/tree/master/hardware)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/v4)
    * [ESP32 adapter](https://github.com/lyusupov/ESP32-NODEMCU-ADAPTER)
* [UAV Edition](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)
    * [Bill of materials](https://github.com/lyusupov/SoftRF/wiki/UAV-BOM)
    * [Wiring and pin-out](https://github.com/lyusupov/ESP32-NODEMCU-ADAPTER)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/UAV)
* [Raspberry Edition](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
* [SkyView EZ](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
    * [Settings](https://github.com/lyusupov/SoftRF/wiki/SkyView-settings) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

* Software
    * Firmware
        * [Release information](https://github.com/lyusupov/SoftRF/releases) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/updated-icon.gif)
        * [Binary](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries)
        * [Settings](https://github.com/lyusupov/SoftRF/wiki/Settings) <!-- ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg) -->
        * [Update](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-(Web-method)) (Web method) <!-- ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg) -->
        * [Source](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source)
* Extras
    * [Long range (LoRa) RF module](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module)
    * Protocols
        * [UAT](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/UATbridge) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)    
        * [OGNTP](https://github.com/lyusupov/SoftRF/wiki/OGNTP-compatibility)
        * [P3I Open (PilotAware)](https://github.com/lyusupov/SoftRF/wiki/PilotAware-compatibility)
        * [FANET (Skytraxx)](https://github.com/lyusupov/SoftRF/wiki/FANET-compatibility)
    * [Garmin GDL90 datalink](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility)
        * [SkyDemon](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility#skydemon)
        * [Avare](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility#avare)
        * [ForeFlight](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility#foreflight)
    * [Integration with airborne (Stratux, PilotAware,...) and groundstation (FlightRadar24, FlightAware,...) ADS-B receivers](https://github.com/lyusupov/SoftRF/wiki/Integration-with-RTL%E2%80%90SDR-based-ADS%E2%80%90B-receivers)
    * [Combined use of multiple SoftRF units](https://github.com/lyusupov/SoftRF/tree/master/software/app/XCSoar7_preview12_plus)
* Archive
    * [Prime Edition](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition)
        * [Bill of materials](https://github.com/lyusupov/SoftRF/wiki/Prime-BOM)
        * [Wiring](https://github.com/lyusupov/SoftRF/wiki/Prime-Wiring)
        * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/v1)
    * Applications
        * [Emulator](https://github.com/lyusupov/SoftRF/tree/master/software/app/Emulator)

# Highlights

* [**Prime Edition Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/SoftRF-Case-v5-Exterior.jpg)

* [**Standalone Edition**](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)

![](https://github.com/lyusupov/SoftRF/blob/master/case/v4/SoftRF-Case-v4-Exterior.jpg)

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/first-five-units.jpg)

* [**SkyView EZ**](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/skyview-16-ePaper-vs-OLED.jpg)

* [**Software driven emulation**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/softrf-emulation.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/LK8000-emu.jpg)

![](https://github.com/lyusupov/SoftRF/blob/master/case/v1/SoftRF-Case-v1-Exterior.jpg)

* [**UAV appliance**](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/uav-rx-hookup.jpg)
