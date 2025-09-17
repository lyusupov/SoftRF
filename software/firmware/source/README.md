# SoftRF firmware build instructions

* [NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#nodemcu)<br>
* [ESP32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#esp32)<br>
* [Raspberry Pi](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#raspberry-pi)<br>
* [CC13X0](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#cc13x0)<br>
* [STM32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#stm32)<br>
* [ASR650x](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#asr650x)<br>
* [nRF52840](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#nrf52840)<br>
* [SAMD21](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#samd21)<br>
* [RP2040](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#rp2040)<br>
* [RA4M1](https://github.com/lyusupov/SoftRF/edit/master/software/firmware/source#ra4m1)<br>
* [RP2350](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#rp2350)<br>
* [Luckfox Lyra](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#luckfox-lyra)<br>

<br>

## NodeMCU

1. Follow [these official instructions](https://github.com/esp8266/Arduino#installing-with-boards-manager)
  to install Arduino IDE and latest Arduino ESP8266 Core
2. Become familiar with IDE and **NodeMCU** by building and uploading of a basic **Blink** sketch:<br>

    _File_ -> _Examples_ -> _ESP8266_ -> _Blink_ <br>

    then<br>

    _Sketch_ -> _Upload_

3. When you are done with the lesson, close your **Arduino** application
4. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. Select _Tools_ -> _Board_ -> _NodeMCU_ _1.0_
10. in _Tools_ -> _lwIP Variant_  -> _v2 Higher Bandwidth_
11. _Sketch_ -> _Upload_

<br>

## ESP32

1. Follow [these official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/source/installing.rst)
  to install Arduino IDE and a **certain version** of Arduino ESP32 Core<!--[**stable** Arduino ESP32 Core](https://github.com/espressif/arduino-esp32/releases/tag/1.0.5) (1.0.5)--><br>
    For **ESP32**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**1.0.5**](https://github.com/espressif/arduino-esp32/releases/tag/1.0.5)<br>
    For **ESP32-S3**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**2.0.9**](https://github.com/espressif/arduino-esp32/releases/tag/2.0.9)<br>
    For **ESP32-C3**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**2.0.9**](https://github.com/espressif/arduino-esp32/releases/tag/2.0.9)<br>
    For **ESP32-C6**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**3.0.7**](https://github.com/espressif/arduino-esp32/releases/tag/3.0.7)<br>
    For **ESP32-P4**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**3.3.1**](https://github.com/espressif/arduino-esp32/releases/tag/3.3.1)<br>
    For **ESP32-C5**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**3.3.1**](https://github.com/espressif/arduino-esp32/releases/tag/3.3.1)<br>

2. Become familiar with IDE and **DoIt ESP32 DevKit** by building and uploading of a basic **Blink** sketch:<br>
```
int ledPin = 2; // use pin 14 for TTGO T-Beam rev.05 or higher   

void setup()
{
    pinMode(ledPin, OUTPUT);
    Serial.begin(115200);
}

void loop()
{
    Serial.println("Hello, world!");
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
}
```

3. When you are done with the lesson, close your **Arduino** application
4. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. For **ESP32**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _ESP32_ _Dev_ _Module_<br>
    For **ESP32-S3**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _ESP32-S3_ _Dev_ _Module_<br>
    For **ESP32-C3**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _ESP32-C3_ _Dev_ _Module_<br>
    For **ESP32-C5**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _ESP32-C5_ _Dev_ _Module_<br>
    For **ESP32-C6**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _ESP32-C6_ _Dev_ _Module_<br>
    For **ESP32-P4**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _ESP32-P4_ _Dev_ _Module_<br>
10. Select _Tools_ -> _Flash_ _Mode_ ->  _DIO_
11. For **ESP32**:<br>
    For **ESP32-S3**:<br>
    For **ESP32-C3**:<br>
    For **ESP32-C6**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Flash_ _Size_ ->  _4MB_<br>
    For **ESP32-C5**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Flash_ _Size_ ->  _8MB_<br>
    For **ESP32-P4**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Flash_ _Size_ ->  _16MB_<br>
12. For **ESP32**:<br>
    For **ESP32-S3**:<br>
    For **ESP32-C3**:<br>
    For **ESP32-P4**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Partition_ _Scheme_ ->  _Minimal_ _SPIFFS_<br>
    For **ESP32-C5**:<br>
    For **ESP32-C6**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Partition_ _Scheme_ ->  _No_ _FS_ _4MB_ _(2MB_ _APP_ _x2)_<br>
13. For **ESP32**:<br>
    For **ESP32-S3**:<br>
    For **ESP32-C3**:<br>
    For **ESP32-C5**:<br>
    For **ESP32-C6**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Flash_ _Frequency_ ->  _80MHz_<br>
    For **ESP32-P4**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Flash_ _Frequency_ ->  _40MHz_<br>
14. For **ESP32**:<br>
    For **ESP32-S3**:<br>
    For **ESP32-C3**:<br>
    For **ESP32-C5**:<br>
    For **ESP32-C6**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _CPU_ _Frequency_ ->  _80MHz_<br>
    For **ESP32-P4**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _CPU_ _Frequency_ ->  _40MHz_<br>
15. Select _Tools_ -> _PSRAM_ ->  _Enabled_
16. _Sketch_ -> _Upload_

<br>

## Raspberry Pi

Although CLI application's source code for Raspberry Edition is located [at the same place](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source)<br>
and is shared with other SoftRF platforms - build instructions for the code are different.

This build has to be done on a Raspberry Pi host.<br>
A known good Raspbian OS version that fits for this purpose is "Stretch".<br>
Make sure that basic development packages (such as: binutils, g++, GNU make & etc) are installed.<br>

1. transfer full content of **SoftRF** and **libraries** GitHub folders into a temporary build directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<your path>``/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<your path>``/libraries <br>

2. change directory on ``<your path>``/SoftRF and execute **make** as follows:<br>

```
pi@raspberrypi: $ make pi
(cd ../libraries/bcm2835/src/../ ; ./configure ; make)
checking for a BSD-compatible install... /usr/bin/install -c
checking whether build environment is sane... yes
checking for a thread-safe mkdir -p... /bin/mkdir -p
checking for gawk... no

< ... skipped ... >
```

As a result of the build, two program binaries will become created:
- **SoftRF** - the program code designed to work with Raspberry Pi's primary SPI bus (SPI 0);
- **SoftRF-aux** - the same program but to run over auxiliary SPI bus (SPI 1).

<br>

## CC13X0

[Energia](http://energia.nu/download/) IDE has to be pre-installed first.<br>

1. open ``<Home>`` directory
2. create **Energia** sub-directory
3. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Energia/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Energia/libraries <br>

4. start **Energia** application
5. open **SoftRF** sketch from _File_ -> _Open_ menu
6. [install support](http://energia.nu/guide/boards/) for TI CC13XX familiy with _Tools_ -> _Board_ ->  _Boards_ _Manager..._
7. Select _Tools_ -> _Board_ ->  _LaunchPad_ _w/_ _CC1310_ _EMT_ _(48MHz)_
8. Select _Tools_ -> _Port_ ->  ``<your XDS110 port device name>``
9. Select _Tools_ -> _Programmer_ ->  _dslite_
10. try to build and upload using _Sketch_ -> _Upload_

10a. or, to build the firmware using shell prompt, do:

```
$ make cc
energia --verify --verbose-build SoftRF.ino
Picked up JAVA_TOOL_OPTIONS:
Loading configuration...
Initializing packages...
Preparing boards...
Verifying...

< ... skipped ... >
```

11a. inspect **/tmp** directory and sub-folders, take **SoftRF.ino.elf** file and use it with TI's **Uniflash** or **Flash Programmer 2**.<br>


12a. (Optional) if you want to use [**cc2538-bsl.py**](https://github.com/JelmerT/cc2538-bsl) tool to put the firmware into flash memory of CC1310 IC via **boot loader**, run **make ihex**:

```
$ make ihex

< ... skipped ... >
``` 

<br>

## STM32

You will need to have an ST-LINK/V2 USB adapter connected in order to put the firmware into your hardware's flash memory.<br>

1. Follow [these official instructions](https://github.com/stm32duino/wiki/wiki/Getting-Started)
  to install Arduino IDE and **certain version** of Arduino STM32 Core -<!-- [latest **stable** Arduino STM32 Core](https://github.com/stm32duino/Arduino_Core_STM32/releases/tag/1.9.0) (1.9.0) --><br>
    For **S76G**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**1.9.0**](https://github.com/stm32duino/Arduino_Core_STM32/releases/tag/1.9.0)<br>
    For **STM32WLE5**:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[**2.4.0**](https://github.com/stm32duino/Arduino_Core_STM32/releases/tag/2.4.0)<br>
    For **STM32F103C8** "Blue Pill":&nbsp;&nbsp;&nbsp;[**2.4.0**](https://github.com/stm32duino/Arduino_Core_STM32/releases/tag/2.4.0)<br>
2. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
3. create **Arduino** sub-directory
4. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

5. start **Arduino** application
6. open **SoftRF** sketch from _File_ -> _Open_ menu
7. For **S76G**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _Nucleo_64_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ _part_ _number_ ->  _Nucleo_ _L073RZ_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _U(S)ART_ _support_ ->  _Enabled_ _(no_ _generic_ _'Serial')_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _USB_ _support_ _(if available)_ ->  _CDC_ _(generic_ _'Serial'_ _supersede_ _U(S)ART)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _USB_ _speed_ _(if available)_ ->  _Low/Full_ _Speed_<br>
    For **STM32WLE5**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _Generic_ _STM32WL_ _series_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ _part_ _number_ ->  _Generic_ _WLE5CCUx_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _U(S)ART_ _support_ ->  _Enabled_ _(generic_ _'Serial')_<br>
    For **STM32F103C8** "Blue Pill":<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _Generic_ _STM32F1_ _series_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ _part_ _number_ ->  _BluePill_ _F103CB_ _(or_ _C8_ _with_ _128k)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _U(S)ART_ _support_ ->  _Enabled_ _(generic_ _'Serial')_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _USB_ _support_ _(if available)_ ->  _CDC_ _(no_ _generic_ _'Serial')_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _USB_ _speed_ _(if available)_ ->  _Low/Full_ _Speed_<br>
8. Select _Tools_ -> _Optimize_ ->  _Smallest_ _(-Os_ _default)_<br>
9. Select _Tools_ -> _C_ _Runtime_ _library_ ->  _Newlib_ _Nano_ _(default)_
10. For **S76G "Dongle"**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Upload_ _method_ ->  _STM32CubeProgrammer_ _(DFU)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Port_ ->  ``<your Dongle's DFU device name>``<br>
    For **STM32WLE5**:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Upload_ _method_ ->  _STM32CubeProgrammer_ _(SWD)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Port_ ->  ``<your ST-LINK/V2 port device name>``<br>
    For **S76G "SkyWatch"** or **STM32F103C8** "Blue Pill":<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Upload_ _method_ ->  _STM32CubeProgrammer_ _(SWD)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Port_ ->  ``<your ST-LINK/V2 port device name>``<br>
11. try to build and upload using _Sketch_ -> _Upload_

<br>

## ASR650x

1. Follow [these official instructions](https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/quick_start.html)
  to install Arduino IDE and [latest **stable** Heltec CubeCell (ASR650X) Arduino Support](https://github.com/HelTecAutomation/ASR650x-Arduino/releases/tag/V1.4.0) (1.4.0)
2. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
3. create **Arduino** sub-directory
4. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

5. start **Arduino** application
6. open **SoftRF** sketch from _File_ -> _Open_ menu
7. Select _Tools_ -> _Board_ ->  _CubeCell-GPS_ _(_ _HTC-AB02S_ _)_
8. Select _Tools_ -> _LORAWAN_AT_SUPPORT_ ->  _OFF_
9. Select _Tools_ -> _LORAWAN_RGB_ ->  _DEACTIVE_
10. Select _Tools_ -> _LORAWAN_CLASS_ ->  _CLASS_A_
11. Select _Tools_ -> _LORAWAN_ADR_ ->  _OFF_
12. Select _Tools_ -> _LORAWAN_NETMODE_ ->  _ABP_
13. Select _Tools_ -> _LORAWAN_Net_Reservation_ ->  _OFF_
14. Select _Tools_ -> _LORAWAN_UPLINKMODE_ ->  _UNCONFIRMED_
15. Select _Tools_ -> _LoRaWan_ _Debug_ _Level_ ->  _None_
16. Select _Tools_ -> _LORAWAN_REGION_ ->  _REGION_EU433_
17. Select _Tools_ -> _Port_ ->  ``<your CubeCell port device name>``
18. try to build and upload using _Sketch_ -> _Upload_

<br>

## nRF52840

1. Follow [these official instructions](https://github.com/adafruit/Adafruit_nRF52_Arduino#recommended-adafruit-nrf52-bsp-via-the-arduino-board-manager)
  to install Arduino IDE and [latest **stable** Arduino Core for Adafruit Bluefruit nRF52 Boards](https://github.com/adafruit/Adafruit_nRF52_Arduino/releases/tag/1.3.0) (1.3.0)
2. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
3. create **Arduino** sub-directory
4. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

5. start **Arduino** application
6. open **SoftRF** sketch from _File_ -> _Open_ menu
7. Select _Tools_ -> _Board_ ->  _Nordic_ _nRF52840DK_ _(PCA10056)_
8. Select _Tools_ -> _Bootloader_ ->  _0.3.2_  _SoftDevice_  _s140_  _6.1.1_
9. Select _Tools_ -> _Debug_ ->   _Level_ _0_  _(Release)_
10. Select _Tools_ -> _Port_ ->  ``<your nRF52840 port device name>``
11. try to build and upload using _Sketch_ -> _Upload_

<br>

## SAMD21

1. Follow [these official instructions](https://www.arduino.cc/en/Guide/ArduinoM0)
  to install Arduino IDE and latest Atmel SAMD Core
2. Become familiar with IDE and **Arduino M0** or **Zero** by building and uploading of a basic **Blink** sketch:<br>

    _File_ -> _Examples_ -> _01.Basics_ -> _Blink_ <br>

    then<br>

    _Sketch_ -> _Upload_

3. When you are done with the lesson, close your **Arduino** application
4. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. _Sketch_ -> _Upload_

<br>

## RP2040

1. Follow [these official instructions](https://github.com/earlephilhower/arduino-pico#installing-via-arduino-boards-manager)
  to install Arduino IDE and [latest **stable** Raspberry Pi Pico Arduino core, for all RP2040 boards](https://github.com/earlephilhower/arduino-pico/releases/tag/3.8.0) (3.8.0)
2. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
3. create **Arduino** sub-directory
4. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

5. start **Arduino** application
6. open **SoftRF** sketch from _File_ -> _Open_ menu
7. For RAK11310 WisBlock Core:<br>
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ -> _Generic_ _RP2040_<br>
   For Raspberry Pi Pico with Waveshare modules:<br>
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ -> _Raspberry_ _Pi_ _Pico_<br>
8. Select _Tools_ -> _Flash_ _Size_ -> _2MB_ _(Sketch:_ _1MB,_ _FS:_ _1MB)_
9. Select _Tools_ -> _USB_ _Stack_ -> _Adafruit_ _TinyUSB_
10. try to build and upload using _Sketch_ -> _Upload_

<br>

## RA4M1

1. Follow [these official instructions](https://docs.arduino.cc/tutorials/uno-r4-wifi/r4-wifi-getting-started/)
  to install Arduino IDE and latest Renesas Core
2. Become familiar with IDE and **Arduino Uno R4 WiFi** by building and uploading of a basic **Blink** sketch:<br>

    _File_ -> _Examples_ -> _01.Basics_ -> _Blink_ <br>

    then<br>

    _Sketch_ -> _Upload_

3. When you are done with the lesson, close your **Arduino** application
4. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. _Sketch_ -> _Upload_

<br>

## RP2350

1. Follow [these official instructions](https://github.com/earlephilhower/arduino-pico#installing-via-arduino-boards-manager)
  to install Arduino IDE and [latest **stable** Raspberry Pi Pico Arduino core, for all RP2XXX boards](https://github.com/earlephilhower/arduino-pico/releases/tag/4.5.1) (4.5.1)
2. open ``<My Documents>`` (Windows) , ``<Home>`` (Linux) or ``<Documents>`` (MacOS) directory
3. create **Arduino** sub-directory
4. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

5. start **Arduino** application
6. open **SoftRF** sketch from _File_ -> _Open_ menu
7. For Raspberry Pi Pico 2W:<br>
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ -> _Raspberry_ _Pi_ _Pico_ _2W_<br>
8. Select _Tools_ -> _Flash_ _Size_ -> _4MB_ _(Sketch:_ _1MB,_ _FS:_ _3MB)_
9. Select _Tools_ -> _USB_ _Stack_ -> _Adafruit_ _TinyUSB_
10. Select _Tools_ -> _IP/Bluetooth_ _Stack_ -> _IPv4_ _+_ _Bluetooth_
11. try to build and upload using _Sketch_ -> _Upload_

<br>

## Luckfox Lyra

Although CLI application's source code for Lyra Edition is located [at the same place](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source)<br>
and is shared with other SoftRF platforms - build instructions for the code are different.

This build has to be done on a Luckfox Lyra host.<br>
Make sure that basic development packages (such as: binutils, g++, GNU make & etc) are installed.<br>

1. transfer full content of **SoftRF** and **libraries** GitHub folders into a temporary build directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<your path>``/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<your path>``/libraries <br>

2. change directory on ``<your path>``/SoftRF and execute **make** as follows:<br>

```
pi@raspberrypi: $ make lyra
(cd ../libraries/bcm2835/src/../ ; ./configure ; make)
checking for a BSD-compatible install... /usr/bin/install -c
checking whether build environment is sane... yes
checking for a thread-safe mkdir -p... /bin/mkdir -p
checking for gawk... no

< ... skipped ... >
```

As a result of the build, one program binary will become created:
- **SoftRF** - the program code designed to work with Luckfox Lyra's primary SPI bus;

<br>
