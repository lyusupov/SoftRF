# SoftRF firmware build instructions

[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#nodemcu)<br>
[ESP32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#esp32)<br>
[Raspberry Pi](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#raspberry-pi)<br>
[CC13XX](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#cc13xx)<br>
[STM32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#stm32)<br>

<br>

## NodeMCU

1. Follow [these official instructions](https://github.com/esp8266/Arduino#installing-with-boards-manager)
  to install Arduino IDE and latest Arduino ESP8266 Core
2. Become familiar with IDE and **NodeMCU** by building and uploading of a basic **Blink** sketch:<br>

    _File_ -> _Examples_ -> _ESP8266_ -> _Blink_ <br>

    then<br>

    _Sketch_ -> _Upload_

3. When you are done with the lesson, close your **Arduino** application
4. open ``<My Documents>`` (Windows) or ``<Home>`` (Linux) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. Select _Tools_ -> _Board_ -> _NodeMCU_ _1.0_
10. in _Tools_ -> _lwIP Variant_  -> _v2 Higher Bandwidth_
11. _Sketch_ -> _Upload_

## ESP32

1. Follow [these official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
  to install Arduino IDE and [latest **stable** Arduino ESP32 Core](https://github.com/espressif/arduino-esp32/releases/tag/1.0.4) (1.0.4)
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
4. open ``<My Documents>`` (Windows) or ``<Home>`` (Linux) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. Select _Tools_ -> _Board_ ->  _ESP32_ _Dev_ _Module_
10. Select _Tools_ -> _Flash_ _Mode_ ->  _DIO_
11. Select _Tools_ -> _Flash_ _Size_ ->  _4MB_
12. Select _Tools_ -> _Partition_ _Scheme_ ->  _Minimal_ _SPIFFS_
13. Select _Tools_ -> _Flash_ _Frequency_ ->  _80MHz_
14. Select _Tools_ -> _CPU_ _Frequency_ ->  _80MHz_
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

## CC13XX

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
$ make build
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

You will also need to get and connect ST-LINK/V2 USB adapter in order to put the firmware into your hardware's flash memory.<br>

1. Follow [these official instructions](https://github.com/stm32duino/wiki/wiki/Getting-Started)
  to install Arduino IDE and [latest **stable** Arduino STM32 Core](https://github.com/stm32duino/Arduino_Core_STM32/releases/tag/1.9.0) (1.9.0)
2. open ``<My Documents>`` (Windows) or ``<Home>`` (Linux) directory
3. create **Arduino** sub-directory
4. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) &nbsp;&nbsp;**-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

5. start **Arduino** application
6. open **SoftRF** sketch from _File_ -> _Open_ menu
7. For S76G:<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _Nucleo_64_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Optimize_ ->  _Fast_ _(-O1)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ _part_ _number_ ->  _Nucleo_ _L073RZ_<br>
    For STM32F103C8 "Blue Pill":<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ ->  _Generic_ _STM32F1_ _series_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Optimize_ ->  _Smallest_ _(-Os_ _default)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Board_ _part_ _number_ ->  _BluePill_ _F103CB_ _(or_ _C8_ _with_ _128k)_<br>
8. Select _Tools_ -> _C_ _Runtime_ _library_ ->  _Newlib_ _Nano_ _(default)_
9. Select _Tools_ -> _USB_ _speed_ _(if available)_ ->  _Low/Full_ _Speed_
10. For S76G "Dongle":<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _USB_ _support_ _(if available)_ ->  _CDC_ _(generic_ _'Serial'_ _supersede_ _U(S)ART)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _U(S)ART_ _support_ ->  _Enabled_ _(no_ _generic_ _'Serial')_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Upload_ _method_ ->  _STM32CubeProgrammer_ _(DFU)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Port_ ->  ``<your Dongle's DFU device name>``<br>
    For S76G "SkyWatch" or STM32F103C8 "Blue Pill":<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _USB_ _support_ _(if available)_ ->  _CDC_ _(no_ _generic_ _'Serial')_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _U(S)ART_ _support_ ->  _Enabled_ _(generic_ _'Serial')_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Upload_ _method_ ->  _STM32CubeProgrammer_ _(SWD)_<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Select _Tools_ -> _Port_ ->  ``<your ST-LINK/V2 port device name>``<br>
11. try to build and upload using _Sketch_ -> _Upload_
