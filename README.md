# SoftRF &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![Join the chat at https://gitter.im/lyusupov/SoftRF](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/lyusupov/SoftRF?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) [![Build Status](https://github.com/lyusupov/SoftRF/actions/workflows/main.yml/badge.svg)](https://github.com/lyusupov/SoftRF/actions/workflows/main.yml "Build Status") 
DIY, multi-functional, compatible, sub-1 GHz ISM band radio based proximity awareness system for general aviation.

## Features

* **Wearable**
    * [**Badge Edition**](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
<!--    * [**Bracelet Edition**](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg) -->
* **Plug-and-Play**
    * [**Dongle Edition**](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
* **Portable**
    * [**Prime Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
* **Classic** 2.25 inch instrument
    * [**Standalone Edition**](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)
* **Certified** (CE/FCC)
    * [**Uni Edition**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)
    * [**Mini Edition**](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)
    * [**Lego Edition**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
    * [**Balkan Edition**](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* **Wireless** 2.7 inch E-Ink **display**
    * [**SkyView EZ**](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
* can **receive ADS-B** air traffic
    * 978 UAT
        * [**Uni Edition**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)
    * 1090 ES
        * [**ES Edition**](https://github.com/lyusupov/SoftRF/wiki/ES-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
    * 1090 ES and/or 978 UAT
        * [**Raspberry Edition**](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)
* can operate **multiple protocols** at the same time
    * [**Octave Concept**](https://github.com/lyusupov/SoftRF/wiki/Octave-Concept)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* **Weatherproof**
    * [**Lego Edition**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg) - IP67
* **UAV** add-on
    * [**UAV Edition**](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)
* Raspberry Pi **friendly**
    * [**Raspberry Edition**](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)
    * [**SkyView Pi**](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pi)
* for **education** purpose
    * [**Academy Edition**](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition)
* **made in** EU
    * [**Balkan Edition**](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

<br>

<!--
* 2-way raw data bridge between 868/915 MHz radio band and Wi-Fi ;
* standalone, battery powered, compatible proximity awareness instrument that fits typical 2.25 inches hole ;
* lightweight version to carry onboard of an UAV.
-->

<!--
[<img src="https://user-images.githubusercontent.com/5849637/128593239-cf30ef4c-1761-4dc1-874a-2be9eaacab77.jpg">](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition)
-->
[<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Badge-banner.jpg">](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition)

# Compatibility <sup>1</sup>
Type|Protocol|FLARM|OGN tracker|PilotAware|Skytraxx|SoftRF
---|---|---|---|---|---|---
Radio|FLARM AIR V6|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_yellow_32.png)<sup>2</sup></p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|<p align="center">OGNTP</p>||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>3</sup></p>
&nbsp;|<p align="center">P3I</p>|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>3</sup></p>
&nbsp;|<p align="center">978&nbsp;UAT<br>ADS-B</p>|||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>4</sup></p>
&nbsp;|<p align="center">1090&nbsp;ES<br>ADS-B</p>|||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>5</sup></p>
&nbsp;|<p align="center">FANET+</p>||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)<sup>3</sup></p>
Data|FLARM NMEA|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|Garmin GDL90|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|<p align="center">MAVLINK</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>|||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>
&nbsp;|<p align="center">Dump1090 &nbsp;<sup>6</sup></p>|||||<p align="center">![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/check-mark_32.png)</p>

<sup>1</sup> - it is necessary for a reader to distinguish the difference between statement "**compatible**" and statement "**fully compatible**".<br>
&nbsp;&nbsp;&nbsp;&nbsp; SoftRF implements only a reasonable minimum of the protocols specs. No "bells and whistles" so far.<br>
<sup>2</sup> - FANET+ can not receive FLARM. However it is able to transmit it.<br>
<sup>3</sup> - valid for [**Prime Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) , [**Dongle**](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition) , [**Mini**](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition) , [**Badge**](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition) , [**Bracelet**](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition) , [**Academy**](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition) , [**Octave**](https://github.com/lyusupov/SoftRF/wiki/Octave-Concept) , [**Lego**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition) and [**Balkan**](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition) **Editions**; valid for [**Standalone**](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition) , [**UAV**](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition) and [**Uni**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition) **Editions** with optional DIY [SoftRF LoRa RF module](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module)<br>
<sup>4</sup> - [**Reception**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition#ads-b-out-remark) of traffic 'downlink' frames only. Valid for [**Uni Edition**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition) alone and for [**Standalone Edition**](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition) with optional DIY [SoftRF UAT module](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced)<br>
<sup>5</sup> - Reception of traffic 'downlink' frames only. Valid for [**ES Edition**](https://github.com/lyusupov/SoftRF/wiki/ES-Edition)<br>
<sup>6</sup> - also known as "raw ADS-B"<br>

# Models
## By Processing Unit
Model(s)|Platform|First appearance|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Status&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|Notes
---|:---:|:---:|:---:|---
1&nbsp;[Prime](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition)<br>2&nbsp;[Standalone](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)<br>3&nbsp;[UAV](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)<br>4&nbsp;[**WebTop&nbsp;Serial**](https://github.com/lyusupov/SoftRF/wiki/WebTop-Serial-adapter)|[Espressif<br>ESP8266](https://en.wikipedia.org/wiki/ESP8266)|Q4 2015|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)<!-- ![](https://via.placeholder.com/140x70/00A000/000000?text=Good) -->|[Prime](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition) model is no longer supported - use [Prime MkII](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) instead.<br><!-- ESP8266 platform will be phased out through year 2020 in favour of ESP32.-->
1&nbsp;[**Prime Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII)<br>2&nbsp;[**Standalone**](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition) + [adapter](https://github.com/lyusupov/ESP32-NODEMCU-ADAPTER)<br>3&nbsp;[UAV](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)<br>4&nbsp;[**SkyView EZ**](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ)<br>5&nbsp;[Flight Recorder](https://github.com/lyusupov/SoftRF/wiki/Flight-Recorder)<br>6&nbsp;[**WebTop USB**](https://github.com/lyusupov/SoftRF/wiki/WebTop-USB)![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[Espressif<br>ESP32](https://en.wikipedia.org/wiki/ESP32)<br>[ESP32-S2](https://en.wikipedia.org/wiki/ESP32#ESP32-S2)<br>[ESP32-S3](https://en.wikipedia.org/wiki/ESP32#ESP32-S3)|Q1 2018|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|Today's best platform
1&nbsp;[**Raspberry Edition**](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)<br>2&nbsp;[**SkyView Pi**](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pi)|[Broadcom<br>BCM283X<br>(Raspberry Pi)](https://en.wikipedia.org/wiki/Raspberry_Pi)|Q4 2018|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|Good for use together with RTL-SDR dongles to achieve additional 1090ES (and 978UAT) ADS-B air traffic reception.
1&nbsp;[UAT module](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced)<br>2&nbsp;[**Uni**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)|[Texas Instruments<br>CC1310](http://www.ti.com/product/CC1310) and [CC13x2R](http://www.ti.com/product/CC1352R)|Q1 2019|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)<!-- ![](https://via.placeholder.com/140x70/FFFF00/000000?text=In+progress) -->|1&nbsp;Unique RF radio specs are useful for [UAT978 **ADS-B** reception](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/UATbridge) [ [1](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/uat-normal-7.jpg) , [2](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/uat-normal-8.jpg) , [3](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/uat-normal-6.jpg) ] ; <br>2&nbsp;holds [**FCC/CE** mark](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition#certificates)
1&nbsp;[Retro](https://github.com/lyusupov/SoftRF/wiki/Retro-Edition)<br>2&nbsp;[**Dongle**](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition)<!-- <br>3&nbsp;[**Bracelet**](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg) --><br>3&nbsp;[**Balkan**](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[STMicroelectronics](https://en.wikipedia.org/wiki/STMicroelectronics)<br>[STM32**L073**](https://www.st.com/en/microcontrollers-microprocessors/stm32l073rz.html) and<br>[STM32**WLE5**](https://www.st.com/en/microcontrollers-microprocessors/stm32wle5cc.html)|Q3 2019|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)<!-- ![](https://via.placeholder.com/140x70/FFFF00/000000?text=In+progress)-->|<!-- [AcSiP **S76G**](http://www.acsip.com.tw/index.php?action=products-detail&fid1=19&fid2=&fid3=&id=41&lang=3) "3-in-1" system-in-package is doing good [ [1](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/watch-1.jpg) , [2](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/t-motion-4.jpg) , [3](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/t-motion-5.jpg) ] .<br>As well as STM32F103C8 "**Blue Pill**" (same MCU that [STM32 OGN tracker](http://wiki.glidernet.org/stm32-ogn-tracker) uses) [ [4](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/stm32_breadboard.jpg) , [5](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/stm32_ognweb_1.JPG) ] . -->
[**Mini**](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)|[Cypress<br>PSoC 4100**S**](https://en.wikipedia.org/wiki/Cypress_PSoC)|Q3 2020|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)<!-- ![](https://via.placeholder.com/140x40/c5f015/000000?text=May+need)<br>![](https://via.placeholder.com/140x40/c5f015/000000?text=improvements) -->|1&nbsp;good add-on candidate for modded Kobo e-Readers ;<br>2&nbsp;holds [**FCC/CE** mark](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition#certificates)
[**Badge**](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition)|[Nordic Semiconductor<br>nRF52840](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52840)|Q4 2020|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[**ES**](https://github.com/lyusupov/SoftRF/wiki/ES-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[NXP Semiconductors<br>LPC4320](https://en.wikipedia.org/wiki/NXP_LPC#LPC4000_series)|Q2 2021|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_may_need_imp.png)|Unique RF radio specs are useful for 1090ES **ADS-B** reception [ [1](https://user-images.githubusercontent.com/5849637/140610617-7c74582e-b0d7-4610-8ac2-d51f55c9086d.png) , [2](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/ES-1.jpg) ]
1&nbsp;[**Academy**](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition)<br>2&nbsp;[USB2BT](https://github.com/lyusupov/SoftRF/wiki/USB-to-Bluetooth-adapter)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[Microchip&nbsp;(Atmel)<br>SAM&nbsp;D21](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/32-bit-mcus/sam-32-bit-mcus/sam-d) <!-- and [ATmega2560](https://www.microchip.com/en-us/product/ATmega2560) -->|Q4 2021|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png) <!-- ![](https://via.placeholder.com/140x40/c5f015/000000?text=May+need)<br>![](https://via.placeholder.com/140x40/c5f015/000000?text=improvements) -->|good for teaching students of air traffic proximity awareness
[**Octave**](https://github.com/lyusupov/SoftRF/wiki/Octave-Concept) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[ASR Microelectronics<br>ASR6601](https://asriot.readthedocs.io/en/latest/ASR6601/index.html)|Q1 2022|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)| **the best 'price per radio protocol' ratio** on the market
[**Lego**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)|[Raspberry Pi<br>Foundation<br>**RP2040**](https://en.wikipedia.org/wiki/RP2040)|Q1 2022|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_may_need_imp.png)|&nbsp;holds **FCC/CE** mark

## By sub-1 GHz radio
Radio|Model(s)|First appearance|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Status&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|Notes
---|:---:|:---:|:---:|---
[Nordic Semiconductor<br>nRF905](https://infocenter.nordicsemi.com/pdf/nRF905_PS_v1.5.pdf)|1&nbsp;[Prime](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition)<br>2&nbsp;[Standalone](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)<br>3&nbsp;[UAV](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)|Q4 2015|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[Semtech<br>SX1276](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1276)|1&nbsp;[LoRa module](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module)<br>2&nbsp;[**Prime Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII)<br>3&nbsp;[**Raspberry Edition**](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)<br>4&nbsp;[**Dongle**](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition)<br>5&nbsp;[**Academy**](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition)<!-- <br>5&nbsp;[**Bracelet**](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition) -->|Q4 2017|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[Texas Instruments<br>CC1310](http://www.ti.com/product/CC1310)|[UAT module](https://github.com/lyusupov/UAT-test-signal#variant-2-advanced)|Q1 2019|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[Semtech<br>SX1231](https://www.semtech.com/products/wireless-rf/fsk-transceivers/sx1231)|[Retro](https://github.com/lyusupov/SoftRF/wiki/Retro-Edition)|Q3 2019|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_limited.png)|operates through OGN driver with OGNTP protocol only
[Semtech<br>SX1262](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262)|1&nbsp;[**Prime Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII)<br>2&nbsp;[**Mini**](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)<br>3&nbsp;[**Badge**](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition)<br>4&nbsp;[**Lego**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)<br>5&nbsp;[**Balkan**](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)|Q1 2020|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[Texas Instruments<br>CC13x2R](http://www.ti.com/product/CC1352R)|[**Uni Edition**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)|Q2 2020|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[Maxim Integrated](https://en.wikipedia.org/wiki/Maxim_Integrated)<br>[MAX2837](https://www.maximintegrated.com/en/products/comms/wireless-rf/MAX2837.html)&nbsp;and<br>[MAX5864](https://www.maximintegrated.com/en/products/analog/data-converters/analog-front-end-ics/MAX5864.html)|[**ES Edition**](https://github.com/lyusupov/SoftRF/wiki/ES-Edition)|Q2 2021|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_may_need_imp.png)|**S**oftware<br>**D**efined<br>**R**adio

## By GNSS chip
GNSS|Model(s)|First appearance|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Status&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|Notes
---|:---:|:---:|:---:|---
Generic<br>NMEA|[Standalone](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)|Q4 2016|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[U-blox](https://en.wikipedia.org/wiki/U-blox)<br>6/7/8|1&nbsp;[**Prime Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII)<br>2&nbsp;[Retro](https://github.com/lyusupov/SoftRF/wiki/Retro-Edition)<br>3&nbsp;[**Uni**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)<br>4&nbsp;[**Lego**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)<br>5&nbsp;[**Balkan**](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)|Q2 2018|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[Hangzhou ZhongKe](http://www.icofchina.com/)<br>AT6558|[1&nbsp;**Prime Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII)<br>2&nbsp;[**Mini**](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)<br>3&nbsp;[**Badge**](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition)|Q2 2019|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[Sony<br>CXD5603GF](https://www.sony-semicon.co.jp/e/products/lsi/gps/product.html)|1&nbsp;[**Dongle**](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition)<!-- <br>2&nbsp;[**Bracelet**](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition) -->|Q4 2019|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[MediaTek](https://en.wikipedia.org/wiki/MediaTek)<br>MT3339|1&nbsp;[**Raspberry**](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)<br>2&nbsp;[**Uni**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)<br>3&nbsp;[**Academy**](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition)<br>4&nbsp;[**Lego**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)|Q3 2020|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;
[GOKE](http://www.goke.com/en/)<br>GK9501|[**Mini**](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)|Q3 2020|![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/icon_good.png)|&nbsp;

# Documentation

* SoftRF overview
    * [Part 1](https://github.com/lyusupov/SoftRF/blob/master/documents/SoftRF-release-1.pdf) (PDF, December 2015)
    * [Part 2](https://github.com/lyusupov/SoftRF/raw/master/documents/SoftRF-release-2.pdf) (PDF, March 2017)
* [Badge Edition](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition.-Quick-start)
    * [Screen pages](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition.-Screen-pages)
    * [Bluetooth LE](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition.-Bluetooth-LE)
    * [Aircrafts database](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition.-Aircrafts-database)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/Badge)
    * [Display care](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition.-Display-care)
    * [Expansion module](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition.-Expansion-module) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
<!-- * [Bracelet Edition](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg) -->
* [Prime Edition MkII](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII.-Quick-start)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/v5)
    * Firmware update
        * [USB](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries#esp32)
        * [webUI](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28Web-method%29#esp32)
        * [programming device](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28hardware-method%29)
* [Standalone Edition](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)
    * [Specifications](https://github.com/lyusupov/SoftRF/wiki/Standalone-Specifications)
    * [Shield](https://github.com/lyusupov/SoftRF/wiki/Standalone-Shield)
    * [Modules](https://github.com/lyusupov/SoftRF/wiki/Standalone-Modules)
    * [Assembly](https://github.com/lyusupov/SoftRF/tree/master/hardware)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/v4)
    * [ESP32 adapter](https://github.com/lyusupov/ESP32-NODEMCU-ADAPTER)
* [ES Edition](https://github.com/lyusupov/SoftRF/wiki/ES-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* [Octave Concept](https://github.com/lyusupov/SoftRF/wiki/Octave-Concept)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* [Uni Edition](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)
    * [Firmware installation](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition.-Firmware-maintenance-procedures)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/Uni)
* [Mini Edition](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)
    * [Firmware installation](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries#cubecell)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition.-Quick-start)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/Mini)
* [Dongle Edition](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
    * [Firmware installation](https://github.com/lyusupov/SoftRF/wiki/AcSiP-S7xG-flashing-instructions)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition.-Quick-start)
    * [Settings](https://github.com/lyusupov/SoftRF/wiki/Dongle-settings)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/Dongle)
* [Lego Edition](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* [Balkan Edition](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* [Academy Edition](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition)
* [UAV Edition](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)
    * [Bill of materials](https://github.com/lyusupov/SoftRF/wiki/UAV-BOM)
    * [Wiring and pin-out](https://github.com/lyusupov/ESP32-NODEMCU-ADAPTER)
    * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/UAV)
* [Raspberry Edition](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)

* SkyView
    * [EZ](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)
        * [Quick start](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start)
        * [Settings](https://github.com/lyusupov/SoftRF/wiki/SkyView-settings)
        * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/SkyView)
        * [Dual boot](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Dual-boot)
    * [Pi](https://github.com/lyusupov/SoftRF/wiki/SkyView-Pi)

* [WebTop USB](https://github.com/lyusupov/SoftRF/wiki/WebTop-USB)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)
* [WebTop Serial](https://github.com/lyusupov/SoftRF/wiki/WebTop-Serial-adapter)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/WebTop-Serial.-Quick-Start)
* [Flight Recorder](https://github.com/lyusupov/SoftRF/wiki/Flight-Recorder)
    * [Quick start](https://github.com/lyusupov/SoftRF/wiki/Flight-Recorder.-Quick-Start)

* Software
    * Firmware
        * [Release information](https://github.com/lyusupov/SoftRF/releases) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/updated-icon.gif)
        * [Binary](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries)
        * [Settings](https://github.com/lyusupov/SoftRF/wiki/Settings)
        * [Update](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-(Web-method)) (Web method)
        * [Source](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source)
* Extras
    * [Long range (LoRa) RF module](https://github.com/lyusupov/SoftRF/wiki/SoftRF-LoRa-module)
        * [Frequency deviation](https://github.com/lyusupov/SoftRF/wiki/Frequency-deviation)
    * Protocols
        * [UAT](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/UATbridge)    
        * [OGNTP](https://github.com/lyusupov/SoftRF/wiki/OGNTP-compatibility)
        * [P3I Open (PilotAware)](https://github.com/lyusupov/SoftRF/wiki/PilotAware-compatibility)
        * [FANET (Skytraxx)](https://github.com/lyusupov/SoftRF/wiki/FANET-compatibility)
    * [Garmin GDL90 datalink](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility)
        * [SkyDemon](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility#skydemon)
        * [Avare](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility#avare)
        * [ForeFlight](https://github.com/lyusupov/SoftRF/wiki/Garmin-GDL90-compatibility#foreflight)
    * [Integration with airborne (Stratux, PilotAware,...) and groundstation (FlightRadar24, FlightAware,...) ADS-B receivers](https://github.com/lyusupov/SoftRF/wiki/Integration-with-RTL%E2%80%90SDR-based-ADS%E2%80%90B-receivers)
    * [Combined use of multiple SoftRF units](https://github.com/lyusupov/SoftRF/wiki/Combined-use-of-multiple-SoftRF-units) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/updated-icon.gif)
* Archive
    * [Prime Edition](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition)
        * [Bill of materials](https://github.com/lyusupov/SoftRF/wiki/Prime-BOM)
        * [Wiring](https://github.com/lyusupov/SoftRF/wiki/Prime-Wiring)
        * [Enclosure](https://github.com/lyusupov/SoftRF/tree/master/case/v1)
    * [Retro Edition](https://github.com/lyusupov/SoftRF/wiki/Retro-Edition)
    * Applications
        * [Emulator](https://github.com/lyusupov/SoftRF/tree/master/software/app/Emulator)

# Highlights

* [**Badge Edition**](https://github.com/lyusupov/SoftRF/wiki/Badge-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Badge-31.jpg)

* [**Prime Edition Mark II**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition-MkII) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/SoftRF-Case-v5-Exterior.jpg)

* [**ES Edition**](https://github.com/lyusupov/SoftRF/wiki/ES-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/ES-1.jpg)
<!-- * [**Bracelet Edition**](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Bracelet-2.jpg) -->

* [**SoftRF Tool**](https://github.com/lyusupov/SoftRF/wiki/SoftRF-Configuration-Tool)

<p><img src="https://github.com/lyusupov/SoftRF/blob/master/documents/images/SoftRF_Tool-7.jpg" width="800" height="592"></p>

* [**Standalone Edition**](https://github.com/lyusupov/SoftRF/wiki/Standalone-Edition)

![](https://github.com/lyusupov/SoftRF/blob/master/case/v4/SoftRF-Case-v4-Exterior.jpg)

![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/first-five-units.jpg)

* [**Uni Edition**](https://github.com/lyusupov/SoftRF/wiki/Uni-Edition)

<p><img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/bom/LPSTK-CC1352R.png" height="310" width="426"><img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/Uni-3.jpg" height="310" width="240"></p>

* [**Dongle Edition**](https://github.com/lyusupov/SoftRF/wiki/Dongle-Edition) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/t-motion-22.jpg)

* [**Mini Edition**](https://github.com/lyusupov/SoftRF/wiki/Mini-Edition)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Mini-7.jpg)

* [**SkyView EZ**](https://github.com/lyusupov/SoftRF/wiki/SkyView-EZ) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/hot_icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/skyview-16-ePaper-vs-OLED.jpg)

* [**Lego Edition**](https://github.com/lyusupov/SoftRF/wiki/Lego-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

![](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/Lego-11.png)

* [**Balkan Edition**](https://github.com/lyusupov/SoftRF/wiki/Balkan-Edition)&nbsp;![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

![](https://raw.githubusercontent.com/lyusupov/SoftRF/master/documents/images/Balkan-1.jpg)

* [**Academy Edition**](https://github.com/lyusupov/SoftRF/wiki/Academy-Edition)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/bom/Lora_GPS_Shield_2.jpg)

* [**Octave Concept**](https://github.com/lyusupov/SoftRF/wiki/Octave-Concept) ![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/new-icon.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Octave-1.jpg)

* [**UAV appliance**](https://github.com/lyusupov/SoftRF/wiki/UAV-Edition)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/uav-rx-hookup.jpg)

* [**Raspberry Edition**](https://github.com/lyusupov/SoftRF/wiki/Raspberry-Edition)

![](https://www.dragino.com/media/k2/galleries/106/Lora%20GPS%20Hat_60.png)

* [**Flight Recorder**](https://github.com/lyusupov/SoftRF/wiki/Flight-Recorder)

<p><img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/iLogger-boot.jpg" width="186" height="260">&nbsp;<img src="https://github.com/lyusupov/SoftRF/raw/master/documents/images/iLogger-SeeYou.JPG" width="494" height="260"></p>

<!--
* [**Software driven emulation**](https://github.com/lyusupov/SoftRF/wiki/Prime-Edition)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/softrf-emulation.jpg)

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/LK8000-emu.jpg)

![](https://github.com/lyusupov/SoftRF/blob/master/case/v1/SoftRF-Case-v1-Exterior.jpg)
-->

