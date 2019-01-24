# SoftRF firmware build instructions

[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#nodemcu)<br>
[ESP32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#esp32)<br>
[Raspberry Pi](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#raspberry-pi)<br>
[CC13XX](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#cc13xx)<br>

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

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) **-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. Select _Tools_ -> _Board_ -> _NodeMCU_ _1.0_
10. in _Tools_ -> _lwIP Variant_  -> _v2 Higher Bandwidth_
11. _Sketch_ -> _Upload_

## ESP32

1. Follow [these official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
  to install Arduino IDE and [latest **stable** Arduino ESP32 Core](https://github.com/espressif/arduino-esp32/releases/tag/1.0.1) (1.0.1)
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

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) **-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

<!-- 7. take **libbt.a** binary from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32/misc)<br>
    and overwrite existing one in ``<My Documents>``/Arduino/hardware/espressif/esp32/tools/sdk/lib -->
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

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) **-->** ``<your path>``/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<your path>``/libraries <br>

2. change directory on ``<your path>``/SoftRF and execute **make** as follows:<br>

```
pi@raspberrypi: $ make -f Makefile.RPi
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

The firmware is to be built on a Linux i686 host.<br>
[Energia](http://energia.nu/download/) IDE has to be pre-installed first.<br>

1. open ``<Home>`` directory
2. create **Energia** sub-directory
3. transfer full content of **UATbridge**, **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [UATbridge](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/UATbridge) **-->** ``<Home>``/Energia/UATbridge <br>
    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) **-->** ``<Home>``/Energia/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<Home>``/Energia/libraries <br>

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

6. build the firmware down to ELF binary code:

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

&nbsp;&nbsp;&nbsp;&nbsp; or do regular build procedure using Energia's UI.<br>

7. inspect **/tmp** directory and sub-folders, take **UATbridge.ino.elf** file and use it with TI's **Uniflash** or **Flash Programmer 2**.<br>
   Or try to use built-in **Energia**'s uploading capability. 

8. (Optional) if you want to use [**cc2538-bsl.py**](https://github.com/JelmerT/cc2538-bsl) tool to put the firmware into flash memory of CC1310 IC via **boot loader**, run **make ihex**:

```
$ make ihex

< ... skipped ... >
``` 

