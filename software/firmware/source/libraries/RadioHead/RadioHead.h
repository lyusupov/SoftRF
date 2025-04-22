// RadioHead.h
// Author: Mike McCauley DO NOT CONTACT THE AUTHOR DIRECTLY
// Copyright (C) 2014 Mike McCauley
// $Id: RadioHead.h,v 1.89 2020/08/05 04:32:19 mikem Exp mikem $

/*! \mainpage RadioHead Packet Radio library for embedded microprocessors

This is the RadioHead Packet Radio library for embedded microprocessors.
It provides a complete object-oriented library for sending and receiving packetized messages
via a variety of common data radios and other transports on a range of embedded microprocessors.

\par Download

The version of the package that this documentation refers to can be downloaded 
from https://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.143.zip

You can always find the latest version of this documentation at
https://www.airspayce.com/mikem/arduino/RadioHead

You can also find online help and discussion at 
https://groups.google.com/group/radiohead-arduino
Please use that group for all questions and discussions on this topic. 
Do not contact the author directly, unless it is to discuss commercial licensing.
Before asking a question or reporting a bug, please read 
- https://en.wikipedia.org/wiki/Wikipedia:Reference_desk/How_to_ask_a_software_question
- https://www.catb.org/esr/faqs/smart-questions.html
- https://www.chiark.greenend.org.uk/~shgtatham/bugs.html

Caution: Developing this type of software and using data radios
successfully is challenging and requires a substantial knowledge
base in software and radio and data transmission technologies and
theory.  It may not be an appropriate project for beginners. If
you are a beginner, you will need to spend some time gaining
knowledge in these areas first.

\par Overview

RadioHead consists of 2 main sets of classes: Drivers and Managers.

- Drivers provide low level access to a range of different packet radios and other packetized message transports.
- Managers provide high level message sending and receiving facilities for a range of different requirements.

Every RadioHead program will have an instance of a Driver to
provide access to the data radio or transport, and usually an instance of a
Manager that uses that driver to send and receive messages for the
application. The programmer is required to instantiate a Driver
and a Manager, and to initialise the Manager. Thereafter the
facilities of the Manager can be used to send and receive
messages.

It is also possible to use a Driver on its own, without a Manager, although this only allows unaddressed, 
unreliable transport via the Driver's facilities.

In some specialised use cases, it is possible to instantiate more than one Driver and more than one Manager.

A range of different common embedded microprocessor platforms are supported, allowing your project to run
on your choice of processor.

Example programs are included to show the main modes of use.

\par Drivers

The following Drivers are provided:

- RH_RF22
Works with Hope-RF
RF22B and RF23B based transceivers, and compatible chips and modules, 
including the RFM22B transceiver module such as 
hthis bare module: https://www.sparkfun.com/products/10153
and this shield: https://www.sparkfun.com/products/11018 
and this board: https://www.anarduino.com/miniwireless
and RF23BP modules such as: https://www.anarduino.com/details.jsp?pid=130
Supports GFSK, FSK and OOK. Access to other chip 
features such as on-chip temperature measurement, analog-digital 
converter, transmitter power control etc is also provided.

- RH_RF24
Works with Silicon Labs Si4460/4461/4463/4464 family of transceivers chip, and the equivalent
HopeRF RF24/26/27 family of chips and the HopeRF RFM24W/26W/27W modules.
Supports GFSK, FSK and OOK. Access to other chip 
features such as on-chip temperature measurement, analog-digital 
converter, transmitter power control etc is also provided.

- RH_RF69 
Works with Hope-RF
RF69B based radio modules, such as the RFM69 module, (as used on the excellent Moteino and Moteino-USB 
boards from LowPowerLab https://lowpowerlab.com/moteino/ )
and compatible chips and modules such as RFM69W, RFM69HW, RFM69CW, RFM69HCW (Semtech SX1231, SX1231H).
Also works with Anarduino MiniWireless -CW and -HW boards https://www.anarduino.com/miniwireless/ including
the marvellous high powered MinWireless-HW (with 20dBm output for excellent range).
Supports GFSK, FSK.

- RH_NRF24
Works with Nordic nRF24 based 2.4GHz radio modules, such as nRF24L01 and others.
Also works with Hope-RF RFM73 
and compatible devices (such as BK2423). nRF24L01 and RFM73 can interoperate
with each other.

- RH_NRF905
Works with Nordic nRF905 based 433/868/915 MHz radio modules.

- RH_NRF51
Works with Nordic nRF51 compatible 2.4 GHz SoC/devices such as the nRF51822.
Also works with Sparkfun nRF52832 breakout board, with Arduino 1.8.9 and
Sparkfun nRF52 boards manager 0.2.3. 

- RH_RF95
Works with Semtech SX1276/77/78/79, Modtronix inAir4 and inAir9,
and HopeRF RFM95/96/97/98 and other similar LoRa capable radios.
Supports Long Range (LoRa) with spread spectrum frequency hopping, large payloads etc.
FSK/GFSK/OOK modes are not (yet) supported.
Also works with the same chips on Linux using LoRa-file-ops Linux driver ioctls to
transmit and receive RadioHead compatible messages. 
Requires a modified version of LoRa-file-ops driver to be installed,
and a compatible radio to be connected appropriately:
https://github.com/starnight/LoRa/tree/file-ops
See the RH_LoRaFileOps class documentation for more information.

- RH_MRF89
Works with Microchip MRF89XA and compatible transceivers.
and modules such as MRF89XAM9A and MRF89XAM8A.

- RH_CC110
Works with Texas Instruments CC110L transceivers and compatible modules such as
Anaren AIR BoosterPack 430BOOST-CC110L

- RH_E32
Works with EBYTE E32-TTL-1W serial radio transceivers (and possibly other transceivers in the same family)

- RH_ASK
Works with a range of inexpensive ASK (amplitude shift keying) RF transceivers such as RX-B1 
(also known as ST-RX04-ASK) receiver; TX-C1 transmitter and DR3100 transceiver; FS1000A/XY-MK-5V transceiver;
HopeRF RFM83C / RFM85. Supports ASK (OOK).

- RH_ABZ
Works with EcoNode SmartTrap, Tlera Grasshopper and family. Almost any board equipped with a muRata cmwx1zzabz module
should work. Tested with EcoNode SmartTrap, Arduino 1.8.9, GrumpyOldPizza Arduino Core for STM32L0.
When building for EcoNode SmartTrap in Arduino IDE, select board type Grasshopper-L082CZ.
This chip and GrumpyOldPizza Arduino Core for STM32L0 are now supported by PlatformIO: 
https://docs.platformio.org/en/latest/platforms/ststm32.html#arduino-stm32l0-configuration-system

- RH_SX126x
Works with Semtech SX1261/2/8 LoRa capable transceivers.
Can be confgigured to automatically controls any digital output pins required to control external RF switches or amplifiers.
Tested with NiceRF LoRa1268-915 and Teensy 3.1.
https://www.nicerf.com/lora-module/915mhz-lora-module-lora1262.html

- RH_STM32WLx
Works with STM32WLE5xx and STM32WLE4xx family processors that include a SX1261/2 multi power amplifier transceiver.
Automatically manages the dedicated SPI interface and interrupt, and the internal SPI slave select and reset pins to the radio.
Examples provided. Requires the stm32duino package using these instructions:
https://community.st.com/t5/stm32-mcus/stm32-arduino-stm32duino-tutorial/ta-p/49649

- RH_Serial
Works with RS232, RS422, RS485, RS488 and other point-to-point and multidropped serial connections, 
or with TTL serial UARTs such as those on Arduino and many other processors,
or with data radios with a 
serial port interface. RH_Serial provides packetization and error detection over any hardware or 
virtual serial connection. Also builds and runs on Linux and OSX.

- RH_TCP
For use with simulated sketches compiled and running on Linux.
Works with tools/etherSimulator.pl to pass messages between simulated sketches, allowing
testing of Manager classes on Linux and without need for real radios or other transport hardware.

- RHEncryptedDriver
Adds encryption and decryption to any RadioHead transport driver, using any encrpytion cipher
supported by ArduinoLibs Cryptographic Library https://rweather.github.io/arduinolibs/crypto.html
which also supports the newly adopted ASCON lightweight cryptography standard for IoT, as announced by 
National Institute of Standards and Technology (NIST).

Drivers can be used on their own to provide unaddressed, unreliable datagrams. 
All drivers have the same identical API.
Or you can use any Driver with any of the Managers described below.

We welcome contributions of well tested and well documented code to support other transports.

If your radio or transciever is not on the list above, there is a good chance it
won't work without modifying RadioHead to suit it.  If you wish for
support for another radio or transciever, and you send at least 2 of them to
AirSpayce Pty Ltd, we will consider adding support for it.

\par Managers

The drivers above all provide for unaddressed, unreliable (ie unacknowleged), variable
length messages, but if you need more than that, the following
Managers are provided:

- RHDatagram
  Addressed, unreliable variable length messages, with optional broadcast facilities.

- RHReliableDatagram
  Addressed, reliable, retransmitted, acknowledged variable length messages.

- RHRouter
  Multi-hop delivery of RHReliableDatagrams from source node to destination node via 0 or more
  intermediate nodes, with manual, pre-programmed routing.

- RHMesh
  Multi-hop delivery of RHReliableDatagrams with automatic route discovery and rediscovery.

Any Manager may be used with any Driver.

\par Platforms

A range of processors and platforms are supported:

- Arduino and the Arduino IDE (version 1.0 to 1.8.1 and 2.1.1 and later)
Including Diecimila, Uno, Mega, Leonardo, Yun, Due, Zero, Minima and possibly others from https://arduino.cc/, 
  Also similar boards such as 
 - Moteino https://lowpowerlab.com/moteino/ 
 - Anarduino Mini https://www.anarduino.com/mini/ 
 - RedBearLab Blend V1.0 https://redbearlab.com/blend/ (with Arduino 1.0.5 and RedBearLab Blend Add-On version 20140701) 
 -  MoteinoMEGA https://lowpowerlab.com/shop/moteinomega 
    (with Arduino 1.0.5 and the MoteinoMEGA Arduino Core 
    https://github.com/LowPowerLab/Moteino/tree/master/MEGA/Core)
 - ESP8266 on Arduino IDE and Boards Manager per https://github.com/esp8266/Arduino 
   Tested using Arduino 1.6.8 with esp8266 by ESP8266 Community version 2.1.0
   Also Arduino 1.8.1 with esp8266 by SparkFun Electronics 2.5.2
   Examples serial_reliable_datagram_* and ask_* are shown to work. 
   CAUTION: The GHz radio included in the ESP8266 is
   not yet supported.
   CAUTION: tests here show that when powered by an FTDI USB-Serial converter, 
   the ESP8266 can draw so much power when transmitting on its GHz WiFi that VCC will sag
   causing random crashes. We strongly recommend a large cap, say 1000uF 10V on VCC if you are also using the WiFi.
 - Various Talk2 Whisper boards eg https://wisen.com.au/store/products/whisper-node-lora.
   Use Arduino Board Manager to install the Talk2 code support. 
 - etc.

- STM32 F4 Discover board, using Arduino 1.8.2 or later and
   Roger Clarkes Arduino_STM from https://github.com/rogerclarkmelbourne/Arduino_STM32
   Caution: with this library and board, sending text to Serial causes the board to hang in mysterious ways. 
   Serial2 emits to PA2. The default SPI pins are SCK: PB3, MOSI PB5, MISO PB4. 
   We tested with PB0 as slave select and PB1 as interrupt pin for various radios. RH_ASK and RH_Serial also work.
   Also works with stm32duino 1.8.0 from https://github.com/stm32duino/Arduino_Core_STM32, wich can be 
   installed on Arduino with BoardManager. Select board: STM32 Discovery F407.

- ChipKIT Core with Arduino IDE on any ChipKIT Core supported Digilent processor (tested on Uno32)
  https://chipkit.net/wiki/index.php?title=ChipKIT_core

- Maple and Flymaple boards with libmaple and the Maple-IDE development environment
  https://leaflabs.com/devices/maple/ and https://www.open-drone.org/flymaple

- Teensy including Teensy 3.1 and earlier built using Arduino IDE 1.0.5 to 1.6.4 and later with 
  teensyduino addon 1.18 to 1.23 and later.
  https://www.pjrc.com/teensy

- Particle Photon https://store.particle.io/collections/photon and ARM3 based CPU with built-in 
  Wi-Fi transceiver and extensive IoT software suport. RadioHead does not support the built-in transceiver
  but can be used to control other SPI based radios, Serial ports etc.
  See below for details on how to build RadioHead for Photon

- ATTiny built using Arduino IDE 1.8 and the ATTiny core from 
  https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
  using the instructions at
  https://medium.com/jungletronics/attiny85-easy-flashing-through-arduino-b5f896c48189
  (Caution: these are very small processors and not all RadioHead features may be available, depending on memory requirements)
  (Caution: we have not had good success building RH_ASK sketches for ATTiny 85  with SpenceKonde ATTinyCore)

- ATtiny Mega (tinyAVR 1-series) chips supported by Spencer Konde's megaTinyCore 
  (https://github.com/SpenceKonde/megaTinyCore) 
  (on Arduino 1.8.9 or later) such as AtTiny 3216, ATtiny 1616 etc. These chips can be easily programmed through their
  UPDI pin, using an ordinary Arduino board programmed as a jtag2updi programmer as described in 
  https://github.com/SpenceKonde/megaTinyCore/blob/master/MakeUPDIProgrammer.md. 
  Make sure you set the programmer type to jtag2updi in the Arduino Tools->Programmer menu.
  See https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/ImportantInfo.md for links to pinouts 
  and pin numbering information for all the suported chips.

- nRF51 compatible Arm chips such as nRF51822 with Arduino 1.6.4 and later using the procedures
  in https://redbearlab.com/getting-started-nrf51822/

- nRF52 compatible Arm chips such as as Adafruit BLE Feather board
  https://www.adafruit.com/product/3406

- Adafruit Feather. These are excellent boards that are available with a variety of radios. We tested with the 
  Feather 32u4 with RFM69HCW radio, with Arduino IDE 1.6.8 and the Adafruit AVR Boards board manager version 1.6.10.
  https://www.adafruit.com/products/3076

- Adafruit Feather M0 boards with Arduino 1.8.1 and later, using the Arduino and Adafruit SAMD board support.
  https://learn.adafruit.com/adafruit-feather-m0-basic-proto/using-with-arduino-ide

- ESP32 built using Arduino IDE 1.8.9 or later using the ESP32 toolchain installed per
  https://github.com/espressif/arduino-esp32 or from the Arduino IDE Board Manager. Tested up to version 3.0.2.
  The internal 2.4GHz radio is not yet supported. Tested with RFM22 using SPI interface. 
  Uses the VSPI SPI bus by default.
  You can enable use of the alternative HSPI bus for SPI by defining RH_ESP32_USE_HSPI in RadioHead.h.

- Raspberry Pi
  Uses BCM2835 library for GPIO https://www.airspayce.com/mikem/bcm2835/
  Currently works only with RH_NRF24 driver or other drivers that do not require interrupt support.
  Contributed by Mike Poublon.

- Linux and OSX
  Using the RHutil/HardwareSerial class, the RH_Serial driver and any manager will
  build and run on Linux and OSX. These can be used to build programs that talk securely and reliably to
  Arduino and other processors or to other Linux or OSX hosts on a reliable, error detected (and possibly encrypted) datagram
  protocol over various types of serial line.

- Mongoose OS, courtesy Paul Austen. Mongoose OSis an Internet of Things Firmware Development Framework 
  available under Apache License Version 2.0. It supports low power, connected microcontrollers such as: 
  ESP32, ESP8266, TI CC3200, TI CC3220, STM32. 
  https://mongoose-os.com/ 

- muRata cmwx1zzabz module, which includes an STM32L0 processor,
  a SX1276 LoRa radio and an antenna switch.

- Raspberry Pi Pico, on Arduino, using either the Raspberry Pi Pico/RP2040 core by Earle F. Philhower, version 1.93,
  installed per https://arduino-pico.readthedocs.io/_/downloads/en/latest/pdf/
  or the alternative Arduino MBED OS RP2040 core version 2.4.1. At this stage support is partial: RH_ASK works
  but radio drivers that use interrupts do not (yet) work in either core.

- Heltec Cube Cell family, eg HTCC-AB01:
  https://heltec.org/project/htcc-ab01-v2/
  https://resource.heltec.cn/download/CubeCell/HTCC-AB01/HTCC-AB01_SchematicDiagram.pdf
  These modules have an ASR6501 or ASR6502 ARM Cortex M0+ processor https://www.microchip.ua/wireless/ASR6501.pdf
  which includes a built-in LoRa capable SX1262 radio, and which is supported by the RadioHead RH_SX126x driver.
  Demonstrated interoperation with RH_RF95 driver.
  Build with the CubeCell board support 1.5.0 via the Arduino Board Manager per 
  https://docs.heltec.org/en/node/asr650x/asr650x_general_docs/quick_start/cubecell-use-arduino-board-manager.html
  Tested with 1.5.0 in Arduino IDE.
  RH_ASK driver works with this board, but since the timer interrupt
  takes the timeout in milliseconds, not microseconds, the fastest bit rate
  we can support is 1000 / 8 = 125 bits per second.
  
Other platforms are partially supported, such as Generic AVR 8 bit processors, MSP430. 
We welcome contributions that will expand the range of supported platforms. 

If your processor is not on the list above, there is a good chance it
wont work without modifying RadioHead to suit it.  If you wish for
support for another processor, and you send 2 of them to
AirSpayce Pty Ltd, we will consider adding support for it.

RadioHead is available (through the efforts of others) 
for PlatformIO. PlatformIO is a cross-platform code builder and the missing library manager.
https://platformio.org/#!/lib/show/124/RadioHead

\par History

RadioHead was created in April 2014, substantially based on code from some of our other earlier Radio libraries:

- RHMesh, RHRouter, RHReliableDatagram and RHDatagram are derived from the RF22 library version 1.39.
- RH_RF22 is derived from the RF22 library version 1.39.
- RH_RF69 is derived from the RF69 library version 1.2.
- RH_ASK is based on the VirtualWire library version 1.26, after significant conversion to C++.
- RH_Serial was new.
- RH_NRF24 is based on the NRF24 library version 1.12, with some significant changes.

During this combination and redevelopment, we have tried to retain all the processor dependencies and support from
the libraries that were contributed by other people. However not all platforms can be tested by us, so if you
find that support from some platform has not been successfully migrated, please feel free to fix it and send us a 
patch.

Users of RHMesh, RHRouter, RHReliableDatagram and RHDatagram in the previous RF22 library will find that their
existing code will run mostly without modification. See the RH_RF22 documentation for more details.

\par Installation

For Arduino IDE, install in the usual way: unzip the distribution zip file to the libraries
sub-folder of your sketchbook. 
The example sketches will be visible in in your Arduino, mpide, maple-ide or whatever.
https://arduino.cc/en/Guide/Libraries

\par Building for Particle Photon

The Photon is not supported by the Arduino IDE, so it takes a little effort to set up a build environment.
Heres what we did to enable building of RadioHead example sketches on Linux, 
but there are other ways to skin this cat.
Basic reference for getting started is: https://particle-firmware.readthedocs.org/en/develop/build/
- Download the ARM gcc cross compiler binaries and unpack it in a suitable place:
\code
cd /tmp
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2015-q4-major/+download/gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2
tar xvf gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2
\endcode
- If dfu-util and friends not installed on your platform, download dfu-util and friends to somewhere in your path
\code
cd ~/bin
wget https://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/linux-i386/dfu-util
wget https://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/linux-i386/dfu-suffix
wget https://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/linux-i386/dfu-prefix
\endcode
- Download the Particle firmware (contains headers and libraries require to compile Photon sketches) 
  to a suitable place:
\code
cd /tmp
wget https://github.com/spark/firmware/archive/develop.zip
unzip develop.zip
\endcode
- Make a working area containing the RadioHead library source code and your RadioHead sketch. You must
  rename the sketch from .pde or .ino to application.cpp
\code
cd /tmp
mkdir RadioHead
cd RadioHead
cp /usr/local/projects/arduino/libraries/RadioHead/ *.h .
cp /usr/local/projects/arduino/libraries/RadioHead/ *.cpp .
cp /usr/local/projects/arduino/libraries/RadioHead/examples/cc110/cc110_client/cc110_client.pde application.cpp
\endcode
- Edit application.cpp and comment out any \#include <SPI.h> so it looks like:
\code
  // #include <SPI.h>
\endcode
- Connect your Photon by USB. Put it in DFU mode as descibed in Photon documentation. Light should be flashing yellow
- Compile the RadioHead sketch and install it as the user program (this does not update the rest of the
  Photon firmware, just the user part:
\code
cd /tmp/firmware-develop/main
PATH=$PATH:/tmp/gcc-arm-none-eabi-5_2-2015q4/bin make APPDIR=/tmp/RadioHead all PLATFORM=photon program-dfu
\endcode
- You should see RadioHead compile without errors and download the finished sketch into the Photon.

\par Compatible Hardware Suppliers

We have had good experiences with the following suppliers of RadioHead compatible hardware:

- LittleBird https://littlebirdelectronics.com.au in Australia for all manner of Arduinos and radios.
- LowPowerLab https://lowpowerlab.com/moteino in USA for the excellent Moteino and Moteino-USB 
  boards which include Hope-RF RF69B radios on-board.
- Anarduino and HopeRF USA (https://www.hoperfusa.com and https://www.anarduino.com) who have a wide range
  of HopeRF radios and Arduino integrated modules.
- SparkFun https://www.sparkfun.com/ in USA who design and sell a wide range of Arduinos and radio modules.
- Wisen https://wisen.com.au who design and sell a wide range of integrated radio/processor modules including the
  excellent Talk2 range.

\par Coding Style

RadioHead is designed so it can run on small processors with very
limited resources and strict timing contraints.  As a result, we
tend only to use the simplest and least demanding (in terms of memory and CPU) C++
facilities. In particular we avoid as much as possible dynamic
memory allocation, and the use of complex objects like C++
strings, IO and buffers. We are happy with this, but we are aware
that some people may think we are leaving useful tools on the
table. You should not use this code as an example of how to do
generalised C++ programming on well resourced processors.

\par Code Contributions

We welcome, and will consider for merging into the mainline, contributions of fixes, patches, improvements etc. 
that meet the following criteria:

- Are generally useful to more than a few people.
- Are thoroughly tested.
- Dont break anything else.
- Are backwards compatible.
- Are properly and completely documented.
- Conform to the coding style of the rest of the library.
- Clearly transfer the ownership of the intellectual property to Mike McCauley.
- Are posted on the Google group as a patch in unified Diff format, 
  made against the latest version of the library, downloaded from airspayce.com, as described above.

There is currently no known fully up-to-date git repository, and at
present we have no intentions of making one ourselves.

\par Donations

This library is offered under a free GPL license for those who want to use it that way. 
We try hard to keep it up to date, fix bugs
and to provide free support. If this library has helped you save time or money, please consider donating at
https://www.airspayce.com or here:

\htmlonly <form action="https://www.paypal.com/cgi-bin/webscr" method="post"><input type="hidden" name="cmd" value="_donations" /> <input type="hidden" name="business" value="mikem@airspayce.com" /> <input type="hidden" name="lc" value="AU" /> <input type="hidden" name="item_name" value="Airspayce" /> <input type="hidden" name="item_number" value="RadioHead" /> <input type="hidden" name="currency_code" value="USD" /> <input type="hidden" name="bn" value="PP-DonationsBF:btn_donateCC_LG.gif:NonHosted" /> <input type="image" alt="PayPal — The safer, easier way to pay online." name="submit" src="https://www.paypalobjects.com/en_AU/i/btn/btn_donateCC_LG.gif" /> <img alt="" src="https://www.paypalobjects.com/en_AU/i/scr/pixel.gif" width="1" height="1" border="0" /></form> \endhtmlonly

\subpage packingdata "Passing Sensor Data Between RadioHead nodes"

\par Trademarks

RadioHead is a trademark of AirSpayce Pty Ltd. The RadioHead mark was first used on April 12 2014 for
international trade, and is used only in relation to data communications hardware and software and related services.
It is not to be confused with any other similar marks covering other goods and services.

\par Copyright

This software is Copyright (C) 2011-2022 Mike McCauley. Use is subject to license
conditions. The main licensing options available are GPL V3 or Commercial:

\par Open Source Licensing GPL V3

This is the appropriate option if you want to share the source code of your
application with everyone you distribute it to, and you also want to give them
the right to share who uses it. If you wish to use this software under Open
Source Licensing, you must contribute all your source code to the open source
community in accordance with the GPL Version 3 when your application is
distributed. See https://www.gnu.org/licenses/gpl-3.0.html

\par Commercial Licensing

This is the appropriate option if you are creating proprietary applications
and you are not prepared to distribute and share the source code of your
application. To purchase a commercial license, contact info@airspayce.com

\par Revision History
\version 1.1 2014-04-14<br>
             Initial public release
\version 1.2 2014-04-23<br>
             Fixed various typos. <br>
             Added links to compatible Anarduino products.<br>
             Added RHNRFSPIDriver, RH_NRF24 classes to support Nordic NRF24 based radios.
\version 1.3 2014-04-28<br>
             Various documentation fixups.<br>
             RHDatagram::setThisAddress() did not set the local copy of thisAddress. Reported by Steve Childress.<br>
             Fixed a problem on Teensy with RF22 and RF69, where the interrupt pin needs to be set for input, <br>
             else pin interrupt doesn't work properly. Reported by Steve Childress and patched by 
             Adrien van den Bossche. Thanks.<br>
             Fixed a problem that prevented RF22 honouring setPromiscuous(true). Reported by Steve Childress.<br>
             Updated documentation to clarify some issues to do with maximum message lengths 
             reported by Steve Childress.<br>
             Added support for yield() on systems that support it (currently Arduino 1.5.5 and later)
             so that spin-loops can suport multitasking. Suggested by Steve Childress.<br>
             Added RH_RF22::setGpioReversed() so the reversal it can be configured at run-time after
             radio initialisation. It must now be called _after_ init(). Suggested by Steve Childress.<br>
\version 1.4 2014-04-29<br>
             Fixed further problems with Teensy compatibility for RH_RF22. Tested on Teensy 3.1.
             The example/rf22_* examples now run out of the box with the wiring connections as documented for Teensy
             in RH_RF22.<br>
             Added YIELDs to spin-loops in RHRouter, RHMesh and RHReliableDatagram, RH_NRF24.<br>
             Tested RH_Serial examples with Teensy 3.1: they now run out of the box.<br>
             Tested RH_ASK examples with Teensy 3.1: they now run out of the box.<br>
             Reduced default SPI speed for NRF24 from 8MHz to 1MHz on Teensy, to improve reliability when
             poor wiring is in use.<br>
             on some devices such as Teensy.<br>
             Tested RH_NRF24 examples with Teensy 3.1: they now run out of the box.<br>
\version 1.5 2014-04-29<br>
             Added support for Nordic Semiconductor nRF905 transceiver with RH_NRF905 driver. Also
             added examples for nRF905 and tested on Teensy 3.1
\version 1.6 2014-04-30<br>
             NRF905 examples were missing
\version 1.7 2014-05-03<br>
             Added support for Arduino Due. Tested with RH_NRF905, RH_Serial, RH_ASK.
             IMPORTANT CHANGE to interrupt pins on Arduino with RH_RF22 and RH_RF69 constructors:
             previously, you had to specify the interrupt _number_ not the interrupt _pin_. Arduinos and Uno32
             are now consistent with all other platforms: you must specify the interrupt pin number. Default
             changed to pin 2 (a common choice with RF22 shields).
             Removed examples/maple/maple_rf22_reliable_datagram_client and 
             examples/maple/maple_rf22_reliable_datagram_client since the rf22 examples now work out
             of the box with Flymaple.
             Removed examples/uno32/uno32_rf22_reliable_datagram_client and 
             examples/uno32/uno32_rf22_reliable_datagram_client since the rf22 examples now work out
             of the box with ChipKit Uno32.
\version 1.8 2014-05-08 <br>
             Added support for YIELD in Teensy 2 and 3, suggested by Steve Childress.<br>
             Documentation updates. Clarify use of headers and Flags<br>
             Fixed misalignment in RH_RF69 between ModemConfigChoice definitions and the implemented choices
             which meant you didnt get the choice you thought and GFSK_Rb55555Fd50 hung the transmitter.<br>
             Preliminary work on Linux simulator.
\version 1.9 2014-05-14 <br>
             Added support for using Timer 2 instead of Timer 1 on Arduino in RH_ASK when
             RH_ASK_ARDUINO_USE_TIMER2 is defined. With the kind assistance of
             Luc Small. Thanks!<br>
             Updated comments in RHReliableDatagram concerning servers, retries, timeouts and delays.
             Fixed an error in RHReliableDatagram where recvfrom return value was not checked.
             Reported by Steve Childress.<br>
             Added Linux simulator support so simple RadioHead sketches can be compiled and run on Linux.<br>
             Added RH_TCP driver to permit message passing between simulated sketches on Linux.<br>
             Added example simulator sketches.<br>
             Added tools/etherSimulator.pl, a simulator of the 'Luminiferous Ether' that passes
             messages between simulated sketches and can simulate random message loss etc.<br>
             Fixed a number of typos and improved some documentation.<br>
\version 1.10 2014-05-15 <br>
             Added support for RFM73 modules to RH_NRF24. These 2 radios are very similar, and can interoperate
             with each other. Added new RH_NRF24::TransmitPower enums for the RFM73, which has a different 
             range of available powers<br>
             reduced the default SPI bus speed for RH_NRF24 to 1MHz, since so many modules and CPU have problems
             with 8MHz.<br>
\version 1.11 2014-05-18<br>
             Testing RH_RF22 with RFM23BP and 3.3V Teensy 3.1 and 5V Arduinos. 
             Updated documentation with respect to GPIO and antenna
             control pins for RFM23. Updated documentation with respect to transmitter power control for RFM23<br>
             Fixed a problem with RH_RF22 driver, where GPIO TX and RX pins were not configured during
             initialisation, causing poor transmit power and sensitivity on those RF22/RF23 devices where GPIO controls
             the antenna selection pins.
\version 1.12 2014-05-20<br>
             Testing with RF69HW and the RH_RF69 driver. Works well with the Anarduino MiniWireless -CW and -HW 
             boards https://www.anarduino.com/miniwireless/ including
             the marvellous high powered MinWireless-HW (with 20dBm output for excellent range).<br>
             Clarified documentation of RH_RF69::setTxPower values for different models of RF69.<br>
             Added RHReliableDatagram::resetRetransmissions().<br>
             Retransmission count precision increased to uin32_t.<br>
             Added data about actual power measurements from RFM22 module.<br>
\version 1.13 2014-05-23<br>
             setHeaderFlags(flags) changed to setHeaderFlags(set, clear), enabling any flags to be
             individually set and cleared by either RadioHead or application code. Requested by Steve Childress.<br>
             Fixed power output setting for boost power on RF69HW for 18, 19 and 20dBm.<br>
             Added data about actual power measurements from RFM69W and RFM69HW modules.<br>
\version 1.14 2014-05-26<br>
             RH_RF69::init() now always sets the PA boost back to the default settings, else can get invalid
             PA power modes after uploading new sketches without a power cycle. Reported by Bryan.<br>
             Added new macros RH_VERSION_MAJOR RH_VERSION_MINOR, with automatic maintenance in Makefile.<br>
             Improvements to RH_TCP: constructor now honours the server argument in the form "servername:port".<br>
             Added YIELD to RHReliableDatagram::recvfromAckTimeout. Requested by Steve Childress.<br>
             Fixed a problem with RH_RF22 reliable datagram acknowledgements that was introduced in version 1.13.
             Reported by Steve Childress.<br>
\version 1.15 2014-05-27<br>
             Fixed a problem with the RadioHead .zip link.
\version 1.16 2014-05-30 <br>
             Fixed RH_RF22 so that lastRssi() returns the signal strength in dBm. Suggested by Steve Childress.<br>
             Added support for getLastPreambleTime() to RH_RF69. Requested by Steve Childress.<br>
             RH_NRF24::init() now checks if there is a device connected and responding, else init() will fail.
             Suggested by Steve Brown.<br>
             RHSoftwareSPI now initialises default values for SPI pins MOSI = 12, MISO = 11 and SCK = 13.<br>
             Fixed some problems that prevented RH_NRF24 working with mixed software and hardware SPI 
             on different devices: a race condition
             due to slow SPI transfers and fast acknowledgement.<br>
\version 1.17 2014-06-02 <br>
             Fixed a debug typo in RHReliableDatagram that was introduced in 1.16.<br>
             RH_NRF24 now sets default power, data rate and channel in init(), in case another
             app has previously set different values without powerdown.<br>
             Caution: there are still problems with RH_NRF24 and Software SPI. Do not use.<br>
\version 1.18 2014-06-02<br>
             Improvements to performance of RH_NRF24 statusRead, allowing RH_NRF24 and Software SPI
             to operate on slow devices like Arduino Uno.<br>
\version 1.19 2014-06-19<br>
             Added examples ask_transmitter.pde and ask_receiver.pde.<br>
             Fixed an error in the RH_RF22 doc for connection of Teensy to RF22.<br>
             Improved documentation of start symbol bit patterns in RH_ASK.cpp
\version 1.20 2014-06-24<br>
             Fixed a problem with compiling on platforms such as ATtiny where SS is not defined.<br>
             Added YIELD to RHMesh::recvfromAckTimeout().<br>
\version 1.21 2014-06-24<br>
             Fixed an issue in RH_Serial where characters might be lost with back-to-back frames.
             Suggested by Steve Childress.<br>
             Brought previous RHutil/crc16.h code into mainline RHCRC.cpp to prevent name collisions
             with other similarly named code in other libraries. Suggested by Steve Childress.<br>
k             Fix SPI bus speed errors on 8MHz Arduinos.
\version 1.22 2014-07-01<br>
             Update RH_ASK documentation for common wiring connections.<br>
             Testing RH_ASK with HopeRF RFM83C/RFM85 courtesy Anarduino https://www.anarduino.com/<br>
             Testing RH_NRF24 with Itead Studio IBoard Pro https://imall.iteadstudio.com/iboard-pro.html
             using both hardware SPI on the ITDB02 Parallel LCD Module Interface pins and software SPI
             on the nRF24L01+ Module Interface pins. Documented wiring required.<br>
             Added support for AVR 1284 and 1284p, contributed by Peter Scargill.
             Added support for Semtech SX1276/77/78 and HopeRF RFM95/96/97/98 and other similar LoRa capable radios
             in LoRa mode only. Tested with the excellent MiniWirelessLoRa from 
             Anarduino https://www.anarduino.com/miniwireless<br>
\version 1.23 2014-07-03<br>
             Changed the default modulation for RH_RF69 to GFSK_Rb250Fd250, since the previous default
             was not very reliable.<br>
             Documented RH_RF95 range tests.<br>
             Improvements to RH_RF22 RSSI readings so that lastRssi correctly returns the last message in dBm.<br>
\version 1.24 2014-07-18
             Added support for building RadioHead for STM32F4 Discovery boards, using the native STM Firmware libraries,
             in order to support Codec2WalkieTalkie (https://www.airspayce.com/mikem/Codec2WalkieTalkie)
             and other projects. See STM32ArduinoCompat.<br>
             Default modulation for RH_RF95 was incorrectly set to a very slow Bw125Cr48Sf4096
\version 1.25 2014-07-25
             The available() function will longer terminate any current transmission, and force receive mode. 
             Now, if there is no unprocessed incoming message and an outgoing message is currently being transmitted, 
             available() will return false.<br>
             RHRouter::sendtoWait(uint8_t*, uint8_t, uint8_t, uint8_t) renamed to sendtoFromSourceWait due to conflicts
             with new sendtoWait() with optional flags.<br>
             RHMEsh and RHRouter already supported end-to-end application layer flags, but RHMesh::sendtoWait() 
             and RHRouter::sendToWait have now been extended to expose a way to send optional application layer flags.
\version 1.26 2014-08-12
             Fixed a Teensy 2.0 compile problem due yield() not available on Teensy < 3.0. <br>
             Adjusted the algorithm of RH_RF69::temperatureRead() to more closely reflect reality.<br>
             Added functions to RHGenericDriver to get driver packet statistics: rxBad(), rxGood(), txGood().<br>
             Added RH_RF69::printRegisters().<br>
             RH_RF95::printRegisters() was incorrectly printing the register index instead of the address.
             Reported by Phang Moh Lim.<br>
             RH_RF95, added definitions for some more registers that are usable in LoRa mode.<br>
             RH_RF95::setTxPower now uses RH_RF95_PA_DAC_ENABLE to achieve 21, 22 and 23dBm.<br>
             RH_RF95, updated power output measurements.<br>
             Testing RH_RF69 on Teensy 3.1 with RF69 on PJRC breakout board. OK.<br>
             Improvements so RadioHead will build under Arduino where SPI is not supported, such as 
             ATtiny.<br>
             Improvements so RadioHead will build for ATTiny using Arduino IDE and tinycore arduino-tiny-0100-0018.zip.<br>
             Testing RH_ASK on ATTiny85. Reduced RAM footprint. 
             Added helpful documentation. Caution: RAM memory is *very* tight on this platform.<br>
             RH_RF22 and RH_RF69, added setIdleMode() function to allow the idle mode radio operating state
             to be controlled for lower idle power consumption at the expense of slower transitions to TX and RX.<br>
\version 1.27 2014-08-13
             All RH_RF69 modulation schemes now have data whitening enabled by default.<br>
             Tested and added a number of OOK modulation schemes to RH_RF69 Modem config table.<br>
             Minor improvements to a number of the faster RH_RF69 modulation schemes, but some slower ones
             are still not working correctly.<br>
\version 1.28 2014-08-20
             Added new RH_RF24 driver to support Si446x, RF24/26/26, RFM24/26/27 family of transceivers.
             Tested with the excellent
             Anarduino Mini and RFM24W and RFM26W with the generous assistance of the good people at 
             Anarduino https://www.anarduino.com.
\version 1.29 2014-08-21
             Fixed a compile error in RH_RF24 introduced at the last minute in hte previous release.<br>
             Improvements to RH_RF69 modulation schemes: now include the AFCBW in teh ModemConfig.<br>
             ModemConfig RH_RF69::FSK_Rb2Fd5 and RH_RF69::GFSK_Rb2Fd5 are now working.<br> 
\version 1.30 2014-08-25
             Fixed some compile problems with ATtiny84 on Arduino 1.5.5 reported by Glen Cook.<br>
\version 1.31 2014-08-27
             Changed RH_RF69 FSK and GFSK modulations from Rb2_4Fd2_4 to Rb2_4Fd4_8 and FSK_Rb4_8Fd4_8 to FSK_Rb4_8Fd9_6
             since the previous ones were unreliable (they had modulation indexes of 1).<br>
\version 1.32 2014-08-28
             Testing with RedBearLab Blend board https://redbearlab.com/blend/. OK.<br>
             Changed more RH_RF69 FSK and GFSK slowish modulations to have modulation index of 2 instead of 1. 
             This required chnaging the symbolic names.<br>
\version 1.33 2014-09-01
             Added support for sleep mode in RHGeneric driver, with new mode 
             RHModeSleep and new virtual function sleep().<br>
             Added support for sleep to RH_RF69, RH_RF22, RH_NRF24, RH_RF24, RH_RF95 drivers.<br>
\version 1.34 2014-09-19
             Fixed compile errors in example rf22_router_test.<br>
             Fixed a problem with RH_NRF24::setNetworkAddress, also improvements to RH_NRF24 register printing.
             Patched by Yveaux.<br>
             Improvements to RH_NRF24 initialisation for version 2.0 silicon.<br>
             Fixed problem with ambigiguous print call in RH_RFM69 when compiling for Codec2.<br>
             Fixed a problem with RH_NRF24 on RFM73 where the LNA gain was not set properly, reducing the sensitivity
             of the receiver.
\version 1.35 2014-09-19
             Fixed a problem with interrupt setup on RH_RF95 with Teensy3.1. Reported by AD.<br>
\version 1.36 2014-09-22
             Improvements to interrupt pin assignments for __AVR_ATmega1284__ and__AVR_ATmega1284P__, provided by
             Peter Scargill.<br>
             Work around a bug in Arduino 1.0.6 where digitalPinToInterrupt is defined but NOT_AN_INTERRUPT is not.<br>
 \version 1.37 2014-10-19
             Updated doc for connecting RH_NRF24 to Arduino Mega.<br>
             Changes to RHGenericDriver::setHeaderFlags(), so that the default for the clear argument
             is now RH_FLAGS_APPLICATION_SPECIFIC, which is less surprising to users.
             Testing with the excellent MoteinoMEGA from LowPowerLab 
             https://lowpowerlab.com/shop/moteinomega with on-board RFM69W.
 \version 1.38 2014-12-29
             Fixed compile warning on some platforms where RH_RF24::send and RH_RF24::writeTxFifo 
             did not return a value.<br>
             Fixed some more compiler warnings in RH_RF24 on some platforms.<br>
             Refactored printRegisters for some radios. Printing to Serial
             is now controlled by the definition of RH_HAVE_SERIAL.<br>
             Added partial support for ARM M4 w/CMSIS with STM's Hardware Abstraction lib for 
             Steve Childress.<br>
 \version 1.39 2014-12-30
             Fix some compiler warnings under IAR.<br>
             RH_HAVE_SERIAL and Serial.print calls removed for ATTiny platforms.<br>
 \version 1.40 2015-03-09
             Added notice about availability on PlatformIO, thanks to Ivan Kravets.<br>
             Fixed a problem with RH_NRF24 where short packet lengths would occasionally not be trasmitted
             due to a race condition with RH_NRF24_TX_DS. Reported by Mark Fox.<br>
 \version 1.41 2015-03-29
             RH_RF22, RH_RF24, RH_RF69 and RH_RF95 improved to allow driver.init() to be called multiple
             times without reallocating a new interrupt, allowing the driver to be reinitialised
             after sleeping or powering down.
 \version 1.42 2015-05-17
             Added support for RH_NRF24 driver on Raspberry Pi, using BCM2835
             library for GPIO pin IO. Contributed by Mike Poublon.<br>
             Tested RH_NRF24 module with NRF24L01+PA+LNA SMA Antenna Wireless Transceiver modules
             similar to: https://www.elecfreaks.com/wiki/index.php?title=2.4G_Wireless_nRF24L01p_with_PA_and_LNA
             works with no software changes. Measured max power output 18dBm.<br>
 \version 1.43 2015-08-02
             Added RH_NRF51 driver to support Nordic nRF51 family processor with 2.4GHz radio such 
             as nRF51822, to be built on Arduino 1.6.4 and later. Tested with RedBearLabs nRF51822 board
             and BLE Nano kit<br>
 \version 1.44 2015-08-08
             Fixed errors with compiling on some platforms without serial, such as ATTiny. 
             Reported by Friedrich Müller.<br>
 \version 1.45 2015-08-13
             Added support for using RH_Serial on Linux and OSX (new class RHutil/HardwareSerial
             encapsulates serial ports on those platforms). Example examples/serial upgraded
             to build and run on Linux and OSX using the tools/simBuild builder.
             RHMesh, RHRouter and RHReliableDatagram updated so they can use RH_Serial without
             polling loops on Linux and OSX for CPU efficiency.<br>
 \version 1.46 2015-08-14
             Amplified some doc concerning Linux and OSX RH_Serial. Added support for 230400
             baud rate in HardwareSerial.<br>
             Added sample sketches nrf51_audio_tx and nrf51_audio_rx which show how to
             build an audio TX/RX pair with RedBear nRF51822 boards and a SparkFun MCP4725 DAC board.
             Uses the built-in ADC of the nRF51822 to sample audio at 5kHz and transmit packets
             to the receiver which plays them via the DAC.<br>
\version 1.47 2015-09-18
             Removed top level Makefile from distribution: its only used by the developer and 
             its presence confuses some people.<br>
             Fixed a problem with RHReliableDatagram with some versions of Raspberry Pi random() that causes 
             problems: random(min, max) sometimes exceeds its max limit.
\version 1.48 2015-09-30
             Added support for Arduino Zero. Tested on Arduino Zero Pro.
\version 1.49 2015-10-01
             Fixed problems that prevented interrupts working correctly on Arduino Zero and Due.
             Builds and runs with 1.6.5 (with 'Arduino SAMD Boards' for Zero version 1.6.1) from arduino.cc.
             Arduino version 1.7.7 from arduino.org is not currently supported.
\version 1.50 2015-10-25
             Verified correct building and operation with Arduino 1.7.7 from arduino.org.
             Caution: You must burn the bootloader from 1.7.7 to the Arduino Zero before it will 
             work with Arduino 1.7.7 from arduino.org. Conversely, you must burn the bootloader from 1.6.5 
             to the Arduino Zero before it will 
             work with Arduino 1.6.5 from arduino.cc. Sigh.
             Fixed a problem with RH_NRF905 that prevented the power and frequency ranges being set
             properly. Reported by Alan Webber.
\version 1.51 2015-12-11
             Changes to RH_RF6::setTxPower() to be compatible with SX1276/77/78/79 modules that
             use RFO transmitter pins instead of PA_BOOST, such as the excellent
             Modtronix inAir4 https://modtronix.com/inair4.html 
             and inAir9 modules https://modtronix.com/inair9.html. With the kind assistance of 
             David from Modtronix.
\version 1.52 2015-12-17
             Added RH_MRF89 module to suport Microchip MRF89XA and compatible transceivers.
             and modules.<br>
\version 1.53 2016-01-02
             Added RH_CC110 module to support Texas Instruments CC110L and compatible transceivers and modules.<br>
\version 1.54 2016-01-29
             Added support for ESP8266 processor on Arduino IDE. Examples serial_reliable_datagram_* are shown to work. 
             CAUTION: SPI not supported yet. Timers used by RH_ASK are not tested. 
             The GHz radio included in the ESP8266 is not yet supported. 
\version 1.55 2016-02-12
             Added macros for htons() and friends to RadioHead.h.
             Added example sketch serial_gateway.pde. Acts as a transparent gateway between RH_RF22 and RH_Serial, 
             and with minor mods acts as a universal gateway between any 2 RadioHead driver networks.
             Initial work on supporting STM32 F2 on Particle Photon: new platform type defined.
             Fixed many warnings exposed by test building for Photon.
             Particle Photon tested support for RH_Serial, RH_ASK, SPI, RH_CC110 etc.
             Added notes on how to build RadioHead sketches for Photon.
\version 1.56 2016-02-18 
             Implemented timers for RH_ASK on ESP8266, added some doc on IO pin selection.
\version 1.57 2016-02-23
             Fixed an issue reported by S3B, where RH_RF22 would sometimes not clear the rxbufvalid flag.
\version 1.58 2-16-04-04
             Tested RH_RF69 with Arduino Due. OK. Updated doc.<br>
             Added support for all ChipKIT Core supported boards 
             https://chipkit.net/wiki/index.php?title=ChipKIT_core
             Tested on ChipKIT Uno32.<br>
             Digilent Uno32 under the old MPIDE is no longer formally 
             supported but may continue to work for some time.<br>
\version 1.59 2016-04-12
             Testing with the excellent Rocket Scream Mini Ultra Pro with the RFM95W and RFM69HCW modules from
             https://www.rocketscream.com/blog/product/mini-ultra-pro-with-radio/  (915MHz versions). Updated
             documentation with hints to suit. Caution: requires Arduino 1.6.8 and Arduino SAMD Boards 1.6.5.
             See also https://www.rocketscream.com/blog/2016/03/10/radio-range-test-with-rfm69hcw/
             for the vendors tests and range with the RFM69HCW version. They also have an RF95 version equipped with
             TCXO temperature controllled oscillator for extra frequency stability and support of very slow and
             long range protocols.
             These boards are highly recommended. They also include battery charging support.
\version 1.60 2016-06-25
             Tested with the excellent talk2 Whisper Node boards 
            (https://talk2.wisen.com.au/ and https://bitbucket.org/talk2/), 
             an Arduino Nano compatible board, which include an on-board RF69 radio, external antenna, 
             run on 2xAA batteries and support low power operations. RF69 examples work without modification.
             Added support for ESP8266 SPI, provided by David Skinner.
\version 1.61 2016-07-07
             Patch to RH_ASK.cpp for ESP8266, to prevent crashes in interrupt handlers. Patch from Alexander Mamchits.
\version 1.62 2016-08-17
             Fixed a problem in RH_ASK where _rxInverted was not properly initialised. Reported by "gno.sun.sop".
             Added support for  waitCAD() and isChannelActive() and setCADTimeout() to RHGeneric.
             Implementation of RH_RF95::isChannelActive() allows the RF95 module to support
             Channel Activity Detection (CAD). Based on code contributed by Bent Guldbjerg Christensen.
             Implmentations of isChannelActive() plus documentation for other radio modules wil be welcomed.
\version 1.63 2016-10-20
             Testing with Adafruit Feather 32u4 with RFM69HCW. Updated documentation to reflect.<br>
\version 1.64 2016-12-10
             RHReliableDatagram now initialises _seenids. Fix from Ben Lim.<br>
             In RH_NRF51, added get_temperature().<br>
             In RH_NRF51, added support for AES packet encryption, which required a slight change 
             to the on-air message format.<br>
\version 1.65 2017-01-11
             Fixed a race condition with RH_NRF51 that prevented ACKs being reliably received.<br>
             Removed code in RH_NRF51 that enabled the DC-DC converter. This seems not to be a necessary condition
             for the radio to work and is now left to the application if that is required.<br>
             Proven interoperation between nRF51822 and nRF52832.<br>
             Modification and testing of RH_NRF51 so it works with nRF52 family processors,
             such Sparkfun nRF52832 breakout board, with Arduino 1.6.13 and
             Sparkfun nRF52 boards manager 0.2.3 using the procedures outlined in
             https://learn.sparkfun.com/tutorials/nrf52832-breakout-board-hookup-guide<br>
             Caution, the Sparkfun development system for Arduino is still immature. We had to 
             rebuild the nrfutil program since the supplied one was not suitable for 
             the Linux host we were developing on. See https://forum.sparkfun.com/viewtopic.php?f=32&t=45071
             Also, after downloading a sketch in the nRF52832, the program does not start executing cleanly: 
             you have to reset the processor again by pressing the reset button. 
             This appears to be a problem with nrfutil, rather than a bug in RadioHead.
\version 1.66 2017-01-15
             Fixed some errors in (unused) register definitions in RH_RF95.h.<br>
             Fixed a problem that caused compilation errors in RH_NRF51 if the appropriate board 
             support was not installed.
\version 1.67 2017-01-24
             Added RH_RF95::frequencyError() to return the estimated centre frequency offset in Hz 
             of the last received message
\version 1.68 2017-01-25
             Fixed arithmetic error in RH_RF95::frequencyError() for some platforms.
\version 1.69 2017-02-02
             Added RH_RF95::lastSNR() and improved lastRssi() calculations per the manual.
\version 1.70 2017-02-03
             Added link to Binpress commercial license purchasing.
\version 1.71 2017-02-07
             Improved support for STM32. Patch from Bent Guldbjerg Christensen.
\version 1.72 2017-03-02
             In RH_RF24, fixed a problem where some important properties were not set by the ModemConfig.
             Added properties 2007, 2008, 2009. Also properties 200a was not being set in the chip.
             Reported by Shannon Bailey and Alan Adamson.
             Fixed corresponding convert.pl and added it to the distribution.
\version 1.73 2017-03-04
             Significant changes to RH_RF24 and its API. It is no longer possible to change the modulation scheme
             programatically: it proved impossible to cater for all the possible crystal frequencies,
             base frequency and modulation schemes. Instead you can use one of a small set of supplied radio
             configuration header files, or generate your own with Silicon Labs WDS application. Changing
             modulation scheme required editing RH_RF24.cpp to specify the appropriate header and recompiling.
             convert.pl is now redundant and removed from the distribution.
\version 1.74 2017-03-08
             Changed RHReliableDatagram so it would not ACK messages heard addressed to other nodes
             in promiscuous mode.<br>
             Added RH_RF24::deviceType() to return the integer value of the connected device.<br>
             Added documentation about how to connect RFM69 to an ESP8266. Tested OK.<br>
             RH_RF24 was not correctly changing state in sleep() and setModeIdle().<br>
             Added example rf24_lowpower_client.pde showing how to put an arduino and radio into a low power
             mode between transmissions to save battery power.<br>
             Improvements to RH_RF69::setTxPower so it now takes an optional ishighpowermodule
             flag to indicate if the connected module is a high power RFM69HW, and so set the power level
             correctly. Based on code contributed by bob.
\version 1.75 2017-06-22
             Fixed broken compiler issues with RH_RF95::frequencyError() reported by Steve Rogerson.<br>
             Testing with the very excellent Rocket Scream boards equipped with RF95 TCXO modules. The
             temperature controlled oscillator stabilises the chip enough to be able to use even the slowest
             protocol Bw125Cr48Sf4096. Caution, the TCXO model radios are not low power when in sleep (consuming
             about ~600 uA, reported by Phang Moh Lim).<br>
             Added support for EBYTE E32-TTL-1W and family serial radio transceivers. These RF95 LoRa based radios
             can deliver reliable messages at up to 7km measured.
\version 1.76 2017-06-23
             Fixed a problem with RH_RF95 hanging on transmit under some mysterious circumstances.
             Reported by several people at  https://forum.pjrc.com/threads/41878-Probable-race-condition-in-Radiohead-library?p=146601#post146601 <br>
             Increased the size of rssi variables to 16 bits to permit RSSI less than -128 as reported by RF95.
\version 1.77 2017-06-25
             Fixed a compilation error with lastRssi().<br>
\version 1.78 2017-07-19
             Fixed a number of unused variable warnings from g++.<br>
             Added new module RHEncryptedDriver and examples, contributed by Philippe Rochat, which
             adds encryption and decryption to any RadioHead transport driver, using any encryption cipher
             supported by ArduinoLibs Cryptographic Library https://rweather.github.io/arduinolibs/crypto.html
             Includes several examples.<br>
\version 1.79 2017-07-25
             Added documentation about 'Passing Sensor Data Between RadioHead nodes'.<br>
             Changes to RH_CC110 driver to calculate RSSI in dBm, based on a patch from Jurie Pieterse.<br>
             Added missing passthroughmethoids to RHEncryptedDriver, allowing it to be used with RHDatagram,
             RHReliableDatagram etc. Tested with RH_Serial. Added examples
\version 1.80 2017-10-04
             Testing with the very fine Talk2 Whisper Node LoRa boards https://wisen.com.au/store/products/whisper-node-lora
             an Arduino compatible board, which include an on-board RFM95/96 LoRa Radio (Semtech SX1276), external antenna, 
             run on 2xAAA batteries and support low power operations. RF95 examples work without modification.
             Use Arduino Board Manager to install the Talk2 code support. Upload the code with an FTDI adapter set to 5V.<br>
             Added support for SPI transactions in development environments that support it with SPI_HAS_TRANSACTION.
             Tested on ESP32 with RFM-22 and Teensy 3.1 with RF69
             Added support for ESP32, tested with RFM-22 connected by SPI.<br>
\version 1.81 2017-11-15
             RH_CC110, moved setPaTable() from protected to public.<br>
             RH_RF95 modem config Bw125Cr48Sf4096 altered to enable slow daat rate in register 26
             as suggested by Dieter Kneffel.
             Added support for nRF52 compatible Arm chips such as as Adafruit BLE Feather board
             https://www.adafruit.com/product/3406, with a patch from Mike Bell.<br>
             Fixed a problem where rev 1.80 broke Adafruit M0 LoRa support by declaring 
             bitOrder variable always as a unsigned char. Reported by Guilherme Jardim.<br>
             In RH_RF95, all modes now have AGC enabled, as suggested by Dieter Kneffel.<br>
\version 1.82 2018-01-07
             Added guard code to RH_NRF24::waitPacketSent() so that if the transmit never completes for some
             reason, the code will eventually return with FALSE.
             Added the low-datarate-optimization bit to config for RH_RF95::Bw125Cr48Sf4096.
             Fix from Jurie Pieterse to ensure RH_CC110::sleep always enters sleep mode.
             Update ESP32 support to include ASK timers. RH_ASK module is now working on ESP32.
\version 1.83 2018-02-12
             Testing adafruit M0 Feather with E32. Updated RH_E32 documentation to show suggested connections
             and contructor initialisation.<br>
             Fixed a problem with RHEncryptedDriver that could cause a crash on some platforms when used
             with RHReliableDatagram. Reported by Joachim Baumann.<br>
	     Improvments to doxygen doc layout in RadioHead.h
\version 1.84 2018-05-07
             Compiles with Roger Clarkes Arduino_STM32 https://github.com/rogerclarkmelbourne/Arduino_STM32,
	     to support STM32F103C etc, and STM32 F4 Discovery etc.<br>
	     Tested STM32 F4 Discovery board with RH_RF22, RH_ASK and RH_Serial.

\version 1.85 2018-07-09
             RHGenericDriver methods changed to virtual, to allow overriding by RHEncrypredDriver: 
	     lastRssi(), mode(), setMode(). Reported by Eyal Gal.<br>
	     Fixed a problem with compiling RH_E32 on some older IDEs, contributed by Philippe Rochat.<br>
	     Improvements to RH_RF95 to improve detection of bad packets, contributed by PiNi.<br>
	     Fixed an error in RHEncryptedDriver that caused incorrect message lengths for messages multiples of 16 bytes 
	     when STRICT_CONTENT_LEN is defined.<br>
	     Fixed a bug in RHMesh which causes the creation of a route to the address which is the byte 
	     behind the end of the route array. Reported by Pascal Gillès de Pélichy.<br>
\version 1.86 2018-08-28
             Update commercial licensing, remove binpress.
\version 1.87 2018-10-06
             RH_RF22 now resets all registers to default state before initialisation commences. Suggested by Wothke.<br>
	     Added RH_ENABLE_EXPLICIT_RETRY_DEDUP which improves the handling of duplicate detection especially
	     in the case where a transmitter periodically wakes up and start transmitting from the first sequence number.
	     Patch courtesy Justin Newitter. Thanks.
\version 1.88 2018-11-13
             Updated to support ATTiny using instructions in
             https://medium.com/jungletronics/attiny85-easy-flashing-through-arduino-b5f896c48189
	     Updated examples ask_transmitter and ask_receiver to compile cleanly on ATTiny. 
	     Tested using ATTiny85 and Arduino 1.8.1. <br>
\version 1.89 2018-11-15
             Testing with ATTiny core from https://github.com/SpenceKonde/ATTinyCore and RH_ASK, 
	     using example ask_transmitter. This resulted in 'Low Memory, instability may occur', 
             and the resulting sketch would transmit only one packet. Suggest ATTiny users do not use this core, but use 
	     the one from https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json 
	     as described in https://medium.com/jungletronics/attiny85-easy-flashing-through-arduino-b5f896c48189 <br>
	     Added support for RH_RF95::setSpreadingFactor(), RH_RF95::setSignalBandwidth(), RH_RF95::setLowDatarate() and
	     RH_RF95::setPayloadCRC(). Patch from Brian Norman. Thanks.<br>

\version 1.90 2019-05-21
             Fixed a block size error in RhEncryptedDriver for the case when
             using STRICT_CONTENT_LEN and sending messages of exactly _blockcipher.blockSize() bytes in length.
	     Reported and patched by Philippe Rochat.
             Patch from Samuel Archibald to prevent compile errors with RH_AAK.cpp fo ATSAMD51.
	     Fixed a probem in RH_RF69::setSyncWords that prevented setSyncWords(NULL, 0) correctly
	     disabling sync detection and generation. Reported by Federico Maggi.
	     RHHardwareSPI::usingInterrupt() was a noop. Fixed to call SPI.usingInterrupt(interrupt);.

\version 1.91 2019-06-01
             Fixed a problem with new RHHardwareSPI::usingInterrupt() that prevented compilation on ESP8266
	     which does not have that call.

\version 1.92 2019-07-14
             Retested serial_reliable_datagram_client.pde and serial_reliable_datagram_server.pde built on Linux
	     as described in their headers, and with USB-RS485 adapters. No changes, working correctly.
	     Testing of nRF5232 with Sparkfun nRF52 board support 0.2.3 shows that there appears to be a problem with
	     interrupt handlers on this board, and none of the interrupt based radio drivers can be expected to work 
	     with this chip.
	     Ensured all interrupt routines are flagged with ICACHE_RAM_ATTR when compiled for ESP8266, to prevent crashes.

\version 1.94 2019-09-02
             Fixed a bug in RHSoftwareSPI where RHGenericSPI::setBitOrder() has no effect for
	     on RHSoftwareSPI. Reported by Peter.<br>
	     Added support in RHRouter for a node to optionally be leaf node, and not participate as a router in the
	     network. See RHRouter::setNodeTypePatch from Alex Evans.<br>
	     Fixed a problem with ESP32 causing compile errors over missing SPI.usingInterrupt().<br>

\version 1.95 2019-10-14
             Fixed some typos in RH_RF05.h macro definitions reported by Clayton Smith.<br>
	     Patch from Michael Cain from RH_ASK on ESP32, untested by me.<br>
	     Added support for RPi Zero and Zero W for the RF95, contributed by Brody Mahoney. 
	     Not tested by me.<br>

\version 1.96 2019-10-14
             Added examples for RPi Zero and Zero W to examples/raspi/rf95, contributed by Brody Mahoney
	     not tested by me. <br>

\version 1.97 2019-11-02
             Added support for Mongoose OS, contributed by Paul Austen.

\version 1.98 2020-01-06
             Rationalised use of RH_PLATFORM_ATTINY to be consistent with other platforms.<br>
	     Added support for RH_PLATFORM_ATTINY_MEGA, for use with Spencer Konde's megaTinyCore 
	     https://github.com/SpenceKonde/megaTinyCore on Atmel megaAVR ATtiny 1-series chips. 
	     Tested with AtTiny 3217, 3216 and 1614, using 
	     RH_Serial, RH_ASK, and RH_RF22 drivers.<br>

\version 1.99 2020-03-07
              Release under GPL V3

\version 1.100 2020-03-12
             Fixed a problem that prevented compilation of RH_NRF51 
	     on Arduino for Sparkfun nRF52832 Breakout board.<br>

\version 1.101 2020-04-10
	     Tested nRF52832 with RFM69W module and RH_RF69, using Software SPI and hardware interrutps OK.<br>
	     Fixed warnings about 'deleting object of polymorphic class' if driver is dynamically allocated.<br>
	     Fixed problems in RH_ASK and HardwareSPI to work with STM32F4 Discovery with latest 
	     version of stm32duino https://github.com/stm32duino/Arduino_Core_STM32.
	     Testing with stm32duino 1.8.0 downloaded with Board Manager per 
	     https://github.com/stm32duino/Arduino_Core_STM32 .
	     Now builds and run RH_ASK examples with STM32F4 Discovery board. 
	     Build without error for STM32 F1 and F4 but Does not compile for Generic STM32F3.

\version 1.102 2020-05-15
             Updated RH_RF95::setPayloadCRC to affect CRC generation on outgoing packets as well 
	     as CRC detection and checking on incoming packets.
	     Added new modem config for RH_RF95. RH_RF95::Bw125Cr45Sf2048
	     Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range. Tested
	     against RPI with LoRa-file-ops driver https://github.com/starnight/LoRa/tree/file-ops 
	     and send.c test program.
	     Fixed a problem with (re-)definition of SS on ESP32, reported and fixed by Justin Newitter.

\version 1.103 2020-05-30
             Fixed some errors in RH_RF95::setTxPower which cased the power levels to be set incorrectly.
	     Checked operation and improved documentation. Valid settings are:
	     2 to 20 (useRFO false) and 0 to 15 (useRFO true). 18, 19 and 20 (useRFO false) turn on the PA_DAC.
	     Fixed RF95 examples to reflect correct use.
	     Added RH_ABZ driver, which supports the muRata CMWX1ZZABZ (TypeABZ) module
	     which includes an STM32L0 processor, a SX1276 LoRa radio and an antenna switch. 
	     Requires the Grumpy Old Pizza Arduino Core installed per https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
	     Examples provided.

\version 1.104 2020-06-08
             Fixed a problem with new RH_ABZ module that prevents compilation with standard 0.0.10 version of STM32L0 <br>
	     Arduino Core installed with Board Manager: STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL 
	     is only available in later versions.<br>

\version 1.105 2020-06-03
             Added support for RH_ABZ on STM32L072xx on Grumpy Old Pizza Arduino Core<br>

\version 1.106 2020-06-16
             Patch from	Livio Tenze for RH_RF22 to fix a problem with interrupts on on ESP8266.<br>
	     Added examples/rf22/rf22_cw, with example showing how to emit a carrier wave (CW).<br>
	     Reverted delay in RHSPIDriver::init() back to 100ms for all platforms except ABZ, where
	     100ms interferes with the USB serial port with at least some versions of the core.<br>
	     Updated and clarified documentation about TCXO use in RH_ABZ and examples.<br>
	     Fixed documentation SS->NSEL pin for RH_RF22 with AtMega. Seems that SS on that platform
	     is now defined as pin 10, not 53. Dont know when that changed.<br>

\version 1.107 2020-06-26
             Improvements to RHEncryptedDriver, so that you will get an explanatory error message if you 
	     include RHEncryptedDriver.h without enabling the class with RH_ENABLE_ENCRYPTION_MODULE in RadioHead.h.<br>
	     Fixed newly introduced errors when compiling for ATTiny 25/45/85.<br>

\version 1.108 2020-07-05
             Fixed a problem with RH_RF22 on ESP8266 introcuced in version 1.106 which prevented
	     messages being received. Some other cleanups in RH_RF22<br>

\version 1.109 2020-07-23
             Fixed a problem that prevented RHEncryptedDriver compiling when RH_ENCRYPTED_DRIVER was enabled.<br>
	     Added optiona hops argument to RHRouter and RHMesh recvfromAck() and receivedFomAckTimeout()
	     to allow the hop count to be retreived if desired.
\version 1.110 2020-07-28
	     Fixed a problem where _spi.beginTransaction and _spi.endTransaction were missing from RHSPIDriver::spiRead.
\version 1.111 2020-07-30
             Fixed some compilation problems on some platforms with RH_PLATFORM_ATTINY_MEGA and Kondes MegaTinyCore 1.1.2.<br>
	     Fixed some other errors with  Kondes MegaTinyCore 2.0.5: F_CPU_CORRECTED no longer exists, 
	     bitOrder and PinStatus no longer needed.<br>
	     Improved detection of RH_PLATFORM_ATTINY_MEGA by looking for defined(MEGATINYCORE), defined
	     in later versions of MegaTinyCore.

\version 1.112 2020-08-05
             Fixed some compiler warnings in STM32 Discovery and other processors.<br>
	     Added support for ST's Arduino Core STM32, https://github.com/stm32duino/Arduino_Core_STM32
	     to RH_ASK, per https://github.com/r-map/rmap/commit/edf9931b21cb7df5b4f67835307255e0fcb301bb.<br>
	     Added documentation about requirements for code contributions.<br>
	     Added library.properties file that some IDEs need.<br>
	     Added support for multithreaded RH_RF95 support on Raspberry Pi, courtesy Tilman Glötzner.<br>
	     Includes example programs, including for the Dragino Lora/GPS Hat which
	     send GPS coordinates.<br>
	     
\version 1.113 2020-10-15
             Changes to RH_ASK on SpenceKonde's megaTinyCore so it will use TCB0 on platforms 
	     where TCB1 is not available.<br>
	     Minor imprvements to documentation.<br>
	     Move to local git for source code control.<br>
	     Added support for RH_ABZ::deinit() for Matt Way.
	     Fixed compilation error in RH_ASK caused by changes to interrupt API in Arduino_Core_STM32 1.9

\version 1.114 2021-03-24
             Fixed compile warnings, errors on some platforms, in RH_ASK.cpp for STM32 core 1.9.0
	     to do with odd behaviour of callback_function_t.<br>
	     Fixed some problems with SPI transactions in RH_RF69 and RH_RF24 that affected 
	     operations with STM32F030F4. Reported and patched by Adam De Muri.<br>
	     Added tools/createGPX.pl, contributed by Tilman Glötzner: A perl script to process the dump files that can be 
	     produced with the  Dragino LoRa-GPS-hat and rf95_client2 + rf95_server2. 
	     The programs exchange sequence numbers, timestamps and gps coordinates 
	     via Lora which can be used for range measurements or visualization. The 
	     perl script is meant for post processing and  converts the log files 
	     into Gpx-files which in turn can be imported to google maps.<br>
	     Added support for MRF89XAM8A, which has support for different frequency bands to RH_MRF89. Patch courtesy 
	     Michael Kefeder.

\version 1.115 2021-03-24
             Fix compile error in new MRF89XAM8A support.

\version 1.116 2021-03-30
             Patch from Vlasta Hajek: 1.  Made RH_ASK usable again for ATTINY after breaking changes.
	     2. Added the possibility to use Timer 1 on ATtiny8x. Tested on ATtiny 85 
	     and ATtiny84 for more than a year.
	     3. Added patch for ESP32 from jPerotto. This finally makes receiving via 
	     RH_ASK on ESP32 without panics. Tested for more than 6 months.<br>

\version 1.117 2021-05-26
             Added documention to RH_MRF89 about how to achieve 1μA power consumption in sleep mode.
	     Courtesy Fernando Faria.<br>
	     Improved documentation for the meaning of 'len' in all recv*, as suggested by Sean. E. Sean.<br>
	     waitAvailable and waitAvaiableTimeout now take an optional polldelay argument that 
	     can be useful for introducing on multitasking systems where a driver requires polling.<br>
	     Fixed compile errors with forward declarations in RH_E32.h and RH_Serial.h on Ardiono IDE on Windows 
	     reported by Skywodd.

\version 1.118 2021-07-16
             Added a 1 us delay in RHSPIDriver::spiWrite as recommended in 
	     https://forum.pjrc.com/attachment.php?attachmentid=10948&d=1499109224 
	     for corect operation with some fast processors. <br>
	     Tested Ebyte E22-900T22S radios with serial interface and the examples from 
	     examples/serial/serial_reliable_datagram_* on Linux. 
	     These radios implement a transparent serial connection
	     using LORA. They can also be configured as independent relays.
	     You can alter the default configuration of the radios (including frequency, 
	     transmission rates, node addresses etc) by using the 
	     Ebyte RF Settings for E22 tool from https://www.cdebyte.net/download-tools and after
	     setting the device to configuraiotn mode (M1 high: temporarily remove the M1 link). 
	     Caution, to use this
	     tool on Windows, you also must install the CH340 USB-Serial port driver, see
	     https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all#windows-710   
	     These modules can also be configured programatically, but this is not supported by RadioHead. <br>
	     Fixed compile errors to do with RH_DRAM_ATTR on non-ESP-32 platforms. <br>
	     Fixed problems reported with crashes in RH_ASK on STM32 boards with core >= 1.9.0 due to 
	     incorrect declaration of interrupt function.  <br>

\version 1.119 2021-08-02
             Changes to RadioHead.h RHHardwareSPI.cpp and RHSPIDriver.cpp for ESP32 support
	     provided by Juliano Perotto.

\version 1.120 2021-11-13
             Added intial support for Raspberry Pi Pico, using the Raspberry Pi Pico/RP2040 core by Earle F. Philhower, version 1.93,
	     installed per https://arduino-pico.readthedocs.io/_/downloads/en/latest/pdf/
	     RH_ASK works with default pins (11 and 12). But it seems that interrupt callbacks (needed by most SPI based
	     radio drivers in RadioHead) are not working (yet) in this core, at least when compiled on our Linux platform. 
	     It appears
	     there is some bug in std::map that causes the interrupt handling to not work correctly 
	     (which I have been able to reproduce in a simple sketch that only uses std::map). This is very strange.
	     Interrupts _are_ supported and work in the alternative Arduino MBED OS RP2040 core version 2.4.1, 
	     but there are other mysterious crashes RadioHead when a radio driver reads SPI inside an interrupt. 
	     But RH_ASK also works with this core.
	     So the result (so far) is partial support of RadioHead using either core (so far). 
	     We will revisit this when either core is updated<br>
	     On ESP32, added support for using the non-default HSPI bus by defining RH_ESP32_USE_HSPI.<br>
	     Improved the reading of RSSI for the last packet in RH_CC110: seems that it was being read too soon. Now
	     read it from the end received packet with RH_CC110_APPEND_STATUS.<br>
             Added RH_LoRaFileOps, driver on RPi+Linux and using LoRa-file-ops Linux driver ioctls to
	     transmit and receive RadioHead compatible messages via SX1276/77/78/79
	     and compatible radios. Requires a modified version of LoRa-file-ops driver to be installed,
	     and a compatible radio to be connected appropriately:
	     https://github.com/starnight/LoRa/tree/file-ops <br>

\version 1.121 2022-02-02
             Restored RH_RF95 code to clear the IRQ flags (twice).<br>

\version 1.122 2023-05-20
             Added RHRouter::getNextValidRoutingTableEntry() contributed by w...<br>
	     Various compatibility imnprovements for STM32L0 etc from Calin Radoni.<br>
	     Added ability to overriding the value of RH_ENABLE_EXPLICIT_RETRY_DEDUP
	     via a compiler arg. Example from PlatformIO:
	     build_flags =
	      -D"RH_ENABLE_EXPLICIT_RETRY_DEDUP=1"
	      Contributed by Justin Newitter.<br>
	     Fixed an error in the header lengths in RH_TCP, which could result in the last octet
	     of the payload being lost. checkForEvents() now returns false instead of exit() in case of IO failure<br>

\version 1.123 2023-08-07
             Added support for Arduino Uno R4 Minima.

\version 1.124 2023-08-09
             Extended support for Arduino Uno R4 to Arduino Uno R4 WIFI

\version 1.125 2023-08-14
             Improved support for Arduino Uno R4: ATOMIC_BLOCK_START and ATOMIC_BLOCK_END are no 
	     longer defined since they hang on R4, and are not necessary since they have SPI_HAS_TRANSACTION

\version 1.126 2023-11-24
             ESP8266 support now uses IRAM_ATTR instead of ICACHE_RAM_ATTR. This should work fine with core 2.7.4 and later.<br>
	     Changes to definitions of ATOMIC_BLOCK_START and ATOMIC_BLOCK_END for RH_PLATFORM_ESP32: Previous
	     use of XTOS_DISABLE_ALL_INTERRUPTS not suported for ESP32C3. Now uses ATOMIC_ENTER_CRITICAL()
	     and ATOMIC_EXIT_CRITICAL() which appear to be supported for all platforms in Arduino esp32 package.<br>

\version 1.127 2024-01-07
             Added support for SX126x family radios, and also for the STM32WLE5xx and STM32WLE4xx families of ARM procedssors that
	     have a SX126x radio built in, and as used in the LoRa-E5-HF module (which is used in WiO-E5 mini development board),
	     the LoRa-E5-LF module and the NUCLEO_WL55JC1 development board. Tested with WiO-E5 mini development board, but no standalone
	     SPI versions of this radio.

\version 1.128 2024-01-12
             Added missing sx126x examples to distribution.

\version 1.129 2024-03-21
             Rename all .pde sketches to .ino
	     Significant changes to RH_SX126x to support other modules such as the NiceRF LoRa1262 family.
	     https://www.nicerf.com/lora-module/915mhz-lora-module-lora1262.html
	     Sample client and server sketches demonstrating use, compatible with Teensy and other 3.3V compatible Arduinos.

\version 1.130 2024-04-12
             Adjustments to RHHardwareSPI fopr compatibility with RAKwireless RAK4360/RAK4361, contributed by gfja.
	     Fixed a preprocessor typo in unused code in RHHardwareSPI.cpp, reported by brodymahoney.

\version 1.131 2024-07-02
             Fixed problem with failed compilation in RH_ASK.cpp, when used with ESP 32 board library version 3.o or greated because
	     the Espressif library timer API changed.<br>
	     
\version 1.132 2024-07-08
             Fixed problem with SX126x where a packet recieved with a CRC error would leave the chip in standby mode
	     but RH_SX126x thought it was still in RX mode. Reported by kalev.<br>
	     
\version 1.133 2024-07-08
             Revisit SX126x RX CRC error handling with the assistance of kalev.
	     
\version 1.134 2024-07-10
             Improvements to SX126x contributed by kalev:<br>
	     1. Save IRQ flags for debugging outside interrupt handler.
	     every time the handler is invoked save IRQ flags and rise _iflag.<br>
	     2. runtime set on/off IRQ mask. This gives possibility to
	     investigate erroneous packets. <br>
	     3. introduce raw mode send and receive for debugging.<br>
	     4. add getFrequencyError() If receiver and sender
	     main clock frequencies are off ( usually when pairing sx126x and 
	     sx127x ) it's difficult to adjust other party's ( usually sx127x ) frequency.<br>
	     
\version 1.135 2024-07-12
             Fixed a problem in RHSPIDriver: if the Slave Select pin was set to 0xff, then it would still drive the
	     SS pin. This broke use of EEPROM on STM32wl. Reported and fixed by Craig Zych.

\version 1.136 2024-07-25
	     Fixed a typo in the comments for the MOSI pin number for the alternative ESP32 HSPI interface.<br>
	     Examples were not included in idstribution since names changed to .ino

\version 1.137 2025-01-03
             Fixed a problem where Timer definitions for RP2350 are differnet to RP2340
	     Reported and patched by Conor O'Neill.
	     
\version 1.138 2025-01-05
             Added support for Heltec CubeCell modules, including the builtin SX126x radio.
	     Updated sx1262_client.ino and sx1262_server.ino to demonstrate usage.

\version 1.139 2025-01-07
             Updated CubeCell support for Heltec CubeCell board 1.5.0 installed in Arduino IDE
	     per https://docs.heltec.org/en/node/asr650x/asr650x_general_docs/quick_start/cubecell-use-arduino-board-manager.html

\version 1.140 2025-01-07
             Fixed a compile problem on some platforms like PlatformIO to do with nullptr.

\version 1.141 2025-01-08
             Fixed another nullptr compile problem on Arduino 2.3.4

\version 1.142 2025-01-11
             Improvements to RH_SX126x to support sleep mode better, This requires us to waituntilNotBusy() _after_ NSS has been
	     asserted, not before.

\version 1.143 2025-01-21
             Extended Cube Cell Board support to all Cube Cell board variants currently supported by Cube Cell 1.5.0.


\author  Mike McCauley. DO NOT CONTACT THE AUTHOR DIRECTLY. USE THE GOOGLE GROUP GIVEN ABOVE
*/

/*! \page packingdata 
\par Passing Sensor Data Between RadioHead nodes

People often ask about how to send data (such as numbers, sensor
readings etc) from one RadioHead node to another. Although this issue
is not specific to RadioHead, and more properly lies in the area of
programming for networks, we will try to give some guidance here.

One reason for the uncertainty and confusion in this area, especially
amongst beginners, is that there is no *best* way to do it. The best
solution for your project may depend on the range of processors and
data that you have to deal with. Also, it gets more difficult if you
need to send several numbers in one packet, and/or deal with floating
point numbers and/or different types of processors.

The principal cause of difficulty is that different microprocessors of
the kind that run RadioHead may have different ways of representing
binary data such as integers. Some processors are little-endian and
some are big-endian in the way they represent multi-byte integers
(https://en.wikipedia.org/wiki/Endianness). And different processors
and maths libraries may represent floating point numbers in radically
different ways:
(https://en.wikipedia.org/wiki/Floating-point_arithmetic)

All the RadioHead examples show how to send and receive simple NUL
terminated ASCII strings, and if thats all you want, refer to the
examples folder in your RadioHead distribution.  But your needs may be
more complicated than that.

The essence of all engineering is compromise so it will be up to you to
decide whats best for your particular needs. The main choices are:
- Raw Binary
- Network Order Binary
- ASCII

\par Raw Binary

With this technique you just pack the raw binary numbers into the packet:

\code
// Sending a single 16 bit unsigned integer
// in the transmitter:
...
uint16_t data = getsomevalue();
if (!driver.send((uint8_t*)&data, sizeof(data)))
{
    ...
\endcode

\code
// and in the receiver:
...
uint16_t data;
uint8_t datalen = sizeof(data);
if (   driver.recv((uint8_t*)&data, &datalen)
    && datalen == sizeof(data))
{
    // Have the data, so do something with it
    uint16_t xyz = data;
    ...
\endcode

If you need to send more than one number at a time, its best to pack
them into a structure

\code
// Sending several 16 bit unsigned integers in a structure
// in a common header for your project:
typedef struct
{
    uint16_t   dataitem1;
    uint16_t   dataitem2;
} MyDataStruct;
...
\endcode

\code
// In the transmitter
...
MyDataStruct data;
data.dataitem1 = getsomevalue();
data.dataitem2 = getsomeothervalue();
if (!driver.send((uint8_t*)&data, sizeof(data)))
{
    ...
\endcode

\code
// in the receiver
MyDataStruct data;
uint8_t datalen = sizeof(data);
if (   driver.recv((uint8_t*)&data, &datalen)
    && datalen == sizeof(data))
{
    // Have the data, so do something with it
    uint16_t pqr = data.dataitem1;
    uint16_t xyz = data.dataitem2;
    ....
\endcode


The disadvantage with this simple technique becomes apparent if your
transmitter and receiver have different endianness: the integers you
receive will not be the same as the ones you sent (actually they are,
but with the internal bytes swapped around, so they probably wont make
sense to you). Endianness is not a problem if *every* data item you
send is a just single byte (uint8_t or int8_t or char), or if the
transmitter and receiver have the same endianness.

So you should only adopt this technique if:
- You only send data items of a single byte each, or
- You are absolutely sure (now and forever into the future) that you
will only ever use the same processor endianness in the transmitter and receiver.

\par Network Order Binary

One solution to the issue of endianness in your processors is to
always convert your data from the processor's native byte order to
'network byte order' before transmission and then convert it back to
the receiver's native byte order on reception. You do this with the
htons (host to network short) macro and friends. These functions may
be a no-op on big-endian processors.

With this technique you convert every multi-byte number to and from
network byte order (note that in most Arduino processors an integer is
in fact a short, and is the same as int16_t. We prefer to use types
that explicitly specify their size so we can be sure of applying the
right conversions):

\code
// Sending a single 16 bit unsigned integer
// in the transmitter:
...
uint16_t data = htons(getsomevalue());
if (!driver.send((uint8_t*)&data, sizeof(data)))
{
    ...
\endcode
\code
// and in the receiver:
...
uint16_t data;
uint8_t datalen = sizeof(data);
if (   driver.recv((uint8_t*)&data, &datalen)
    && datalen == sizeof(data))
{
    // Have the data, so do something with it
    uint16_t xyz = ntohs(data);
    ...
\endcode

If you need to send more than one number at a time, its best to pack
them into a structure

\code
// Sending several 16 bit unsigned integers in a structure
// in a common header for your project:
typedef struct
{
    uint16_t   dataitem1;
    uint16_t   dataitem2;
} MyDataStruct;
...
\endcode
\code
// In the transmitter
...
MyDataStruct data;
data.dataitem1 = htons(getsomevalue());
data.dataitem2 = htons(getsomeothervalue());
if (!driver.send((uint8_t*)&data, sizeof(data)))
{
    ...
\endcode
\code
// in the receiver
MyDataStruct data;
uint8_t datalen = sizeof(data);
if (   driver.recv((uint8_t*)&data, &datalen)
    && datalen == sizeof(data))
{
    // Have the data, so do something with it
    uint16_t pqr = ntohs(data.dataitem1);
    uint16_t xyz = ntohs(data.dataitem2);
    ....
\endcode

This technique is quite general for integers but may not work if you
want to send floating point number between transmitters and receivers
that have different floating point number representations.


\par ASCII

In this technique, you transmit the printable ASCII equivalent of
each floating point and then convert it back to a float in the receiver:

\code
// In the transmitter
...
float data = getsomevalue();
uint8_t buf[15]; // Bigger than the biggest possible ASCII
snprintf(buf, sizeof(buf), "%f", data);
if (!driver.send(buf, strlen(buf) + 1)) // Include the trailing NUL
{
    ...
\endcode
\code

// In the receiver
...
float data;
uint8_t buf[15]; // Bigger than the biggest possible ASCII
uint8_t buflen = sizeof(buf);
if (driver.recv(buf, &buflen))
{
    // Have the data, so do something with it
    float data = atof(buf); // String to float
    ...
\endcode

\par Conclusion:

- This is just a basic introduction to the issues. You may need to
extend your study into related C/C++ programming techniques.

- You can extend these ideas to signed 16 bit (int16_t) and 32 bit
(uint32_t, int32_t) numbers.

- Things can be simple or complicated depending on the needs of your
project.

- We are not going to write your code for you: its up to you to take
these examples and explanations and extend them to suit your needs.

*/



#ifndef RadioHead_h
#define RadioHead_h

// Official version numbers are maintained automatically by Makefile:
#define RH_VERSION_MAJOR 1
#define RH_VERSION_MINOR 143

// Symbolic names for currently supported platform types
#define RH_PLATFORM_ARDUINO          1
#define RH_PLATFORM_MSP430           2
#define RH_PLATFORM_STM32            3
#define RH_PLATFORM_GENERIC_AVR8     4
#define RH_PLATFORM_UNO32            5
#define RH_PLATFORM_UNIX             6
#define RH_PLATFORM_STM32STD         7
#define RH_PLATFORM_STM32F4_HAL      8 
#define RH_PLATFORM_RASPI            9
#define RH_PLATFORM_NRF51            10
#define RH_PLATFORM_ESP8266          11
#define RH_PLATFORM_STM32F2          12
#define RH_PLATFORM_CHIPKIT_CORE     13
#define RH_PLATFORM_ESP32            14						   
#define RH_PLATFORM_NRF52            15
#define RH_PLATFORM_MONGOOSE_OS      16
#define RH_PLATFORM_ATTINY           17
// Spencer Kondes megaTinyCore:						   
#define RH_PLATFORM_ATTINY_MEGA      18
#define RH_PLATFORM_STM32L0          19
#define RH_PLATFORM_RASPI_PICO       20
						   
////////////////////////////////////////////////////
// Select platform automatically, if possible
#ifndef RH_PLATFORM
 #if (defined(MPIDE) && MPIDE>=150 && defined(ARDUINO))
  // Using ChipKIT Core on Arduino IDE
  #define RH_PLATFORM RH_PLATFORM_CHIPKIT_CORE
 #elif defined(MPIDE)
  // Uno32 under old MPIDE, which has been discontinued:
  #define RH_PLATFORM RH_PLATFORM_UNO32
 #elif defined(NRF51) || defined(NRF52)
  #define RH_PLATFORM RH_PLATFORM_NRF51
 #elif defined(NRF52)
  #define RH_PLATFORM RH_PLATFORM_NRF52
 #elif defined(ESP8266)
  #define RH_PLATFORM RH_PLATFORM_ESP8266
 #elif defined(ESP32)
  #define RH_PLATFORM RH_PLATFORM_ESP32
 #elif defined(STM32L0) || defined(ARDUINO_ARCH_STM32L0)
  #define RH_PLATFORM RH_PLATFORM_STM32L0
 #elif defined(MGOS)
  #define RH_PLATFORM RH_PLATFORM_MONGOOSE_OS
#elif defined(MEGATINYCORE) || defined(ARDUINO_attinyxy2) || defined(ARDUINO_attinyxy4) || defined(ARDUINO_attinyxy6) || defined(ARDUINO_attinyxy7)
  #define RH_PLATFORM RH_PLATFORM_ATTINY_MEGA
 #elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtinyX4__) || defined(__AVR_ATtinyX5__) || defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny4313__) || defined(__AVR_ATtinyX313__) || defined(ARDUINO_attiny)
  #define RH_PLATFORM RH_PLATFORM_ATTINY
 #elif defined(ARDUINO)
  #define RH_PLATFORM RH_PLATFORM_ARDUINO
 #elif defined(__MSP430G2452__) || defined(__MSP430G2553__)
  #define RH_PLATFORM RH_PLATFORM_MSP430
 #elif defined(MCU_STM32F103RE)
  #define RH_PLATFORM RH_PLATFORM_STM32
 #elif defined(STM32F2XX)
  #define RH_PLATFORM RH_PLATFORM_STM32F2
 #elif defined(USE_STDPERIPH_DRIVER)
  #define RH_PLATFORM RH_PLATFORM_STM32STD
 #elif defined(RASPBERRY_PI)
  #define RH_PLATFORM RH_PLATFORM_RASPI
 #elif defined(__unix__) // Linux
  #define RH_PLATFORM RH_PLATFORM_UNIX
 #elif defined(__APPLE__) // OSX
  #define RH_PLATFORM RH_PLATFORM_UNIX
						   
 #else
  #error Platform not defined! 	
 #endif
#endif

////////////////////////////////////////////////////
// Platform specific headers:
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
 #if (ARDUINO >= 100)
  #include <Arduino.h>
 #else
  #include <wiring.h>
 #endif
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL
 
 #if defined(ARDUINO_ARCH_STM32F4)
  // output to Serial causes hangs on STM32 F4 Discovery board
  // There seems to be no way to output text to the USB connection
  #undef Serial						   
  #define Serial Serial2
  
 #elif defined(ARDUINO_ARCH_RP2040)
  #if defined(PICO_RP2350)
    // Raspi Pico 2350
    #define RH_ASK_PICO_ALARM_IRQ TIMER0_IRQ_1
  #else
    // Raspi Pico 2040
    #define RH_ASK_PICO_ALARM_IRQ TIMER_IRQ_1
  #endif
  #define RH_ASK_PICO_ALARM_NUM 1
  
 #elif defined(ARDUINO_LORA_E5_MINI)
  // WiO-E5 mini, or boards conating Seeed LoRa-E5-LF or LoRa-E5-HF, processor is STM32WLE5JC
  #include <SubGhz.h>
 
#elif defined(CubeCell_Board) || defined(CubeCell_Board_V2) || defined(CubeCell_Capsule) || defined(CubeCell_Module) || defined(CubeCell_Module_V2) || defined(CubeCell_BoardPlus) || defined(CubeCell_GPS) || defined(CubeCell_ModulePlus) || defined(CubeCell_HalfAA) || defined(CubeCell_BoardP)
  #define RH_CUBE_CELL_BOARD
  #define RH_MISSING_SPIUSINGINTERRUPT
  #include <board-config.h> // For Radio pin definitions
  
 #endif

#elif (RH_PLATFORM == RH_PLATFORM_ATTINY)
  #include <Arduino.h>
//  #warning Arduino TinyCore does not support hardware SPI. Use software SPI instead.
#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
 #include <SPI.h>
  #define RH_HAVE_HARDWARE_SPI
  #define RH_HAVE_SERIAL
  // On most AT_TINY_MEGA, Timer A is used for millis/micros, and B 0 or 1 for Tone by default.
  // But not all devices support TCB1, so we use TCB0 on some variants.
  // This is the same strategy for timer selection that Tone uses:
  #if defined(MILLIS_USE_TIMERB0) && defined(TCB1)
   #define RH_ATTINY_MEGA_ASK_TIMER TCB1
   #define RH_ATTINY_MEGA_ASK_TIMER_VECTOR TCB1_INT_vect
  #else
   #define RH_ATTINY_MEGA_ASK_TIMER TCB0
   #define RH_ATTINY_MEGA_ASK_TIMER_VECTOR TCB0_INT_vect
  #endif
						   
#elif (RH_PLATFORM == RH_PLATFORM_ESP8266) // ESP8266 processor on Arduino IDE
 #include <Arduino.h>
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL
 #define RH_MISSING_SPIUSINGINTERRUPT

#elif (RH_PLATFORM == RH_PLATFORM_ESP32)   // ESP32 processor on Arduino IDE
 #include <Arduino.h>
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL
 #define RH_MISSING_SPIUSINGINTERRUPT
 // ESP32 has 2 user SPI buses: VSPI and HSPI. They are essentially identical, but use different pins.
 // The usual, default bus VSPI (available as SPI object in Arduino) uses pins:
 // SCLK:      18
 // MISO:      19
 // MOSI:      23
 // SS:	       5
 // The other HSPI bus uses pins
 // SCLK:      14
 // MISO:      12
 // MOSI:      13
 // SS:	       15
 // By default RadioHead uses VSPI, but you can make it use HSPI by defining this:
 //#define RH_ESP32_USE_HSPI
						   
#elif (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS) // Mongoose OS platform
  #include <mgos.h>
  #include <mgos_adc.h>
  #include <mgos_pwm.h>
  #include <MGOSCompat/HardwareSerial.h>
  #include <MGOSCompat/HardwareSPI.h>
  #include <MGOSCompat/MGOS.h>
  #include <math.h> // We use the floor() math function.
  #define RH_HAVE_HARDWARE_SPI
   //If a Radio is connected via a serial port then this defines the serial
   //port the radio is connected to.
  #if defined(RH_SERIAL_PORT)
   #if RH_SERIAL_PORT == 0
    #define Serial Serial0
   #elif RH_SERIAL_PORT == 1
    #define Serial Serial1
   #elif RH_SERIAL_PORT == 2
    #define Serial Serial2
   #endif
  #else
   #warning "RH_SERIAL_PORT not defined. Therefore serial port 0 selected"
   #define Serial Serial0
  #endif
  #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_MSP430) // LaunchPad specific
 #include "legacymsp430.h"
 #include "Energia.h"
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_UNO32 || RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
 #include <WProgram.h>
 #include <string.h>
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define memcpy_P memcpy
 #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_STM32) // Maple, Flymaple etc
 #include <STM32ArduinoCompat/wirish.h>	
 #include <stdint.h>
 #include <string.h>
 #include <STM32ArduinoCompat/HardwareSPI.h>
 #define RH_HAVE_HARDWARE_SPI
 // Defines which timer to use on Maple
 #define MAPLE_TIMER 1
 #define PROGMEM
 #define memcpy_P memcpy
 #define Serial SerialUSB
 #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_STM32L0)
 #include <Arduino.h>
 #include <SPI.h>
 // Can define this in platformio.ini						   
 #ifndef RH_EXCLUDE_STM32L0_INCLUDES
  // https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
  #include <stm32l0_gpio.h>
  #include <stm32l0_exti.h>
  #include <stm32l0_rtc.h>
 #endif
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL 

#elif (RH_PLATFORM == RH_PLATFORM_STM32F2) // Particle Photon with firmware-develop
 #include <stm32f2xx.h>
 #include <application.h>
 #include <math.h> // floor
 #define RH_HAVE_SERIAL
 #define RH_HAVE_HARDWARE_SPI

#elif (RH_PLATFORM == RH_PLATFORM_STM32STD) // STM32 with STM32F4xx_StdPeriph_Driver 
 #include <stm32f4xx.h>
 #include <wirish.h>	
 #include <stdint.h>
 #include <string.h>
 #include <math.h>
 #include <HardwareSPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define Serial SerialUSB
 #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8) 
 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include <string.h>
 #include <stdbool.h>
 #define RH_HAVE_HARDWARE_SPI
 #include <SPI.h>

// For Steve Childress port to ARM M4 w/CMSIS with STM's Hardware Abstraction lib. 
// See ArduinoWorkarounds.h (not supplied)
#elif (RH_PLATFORM == RH_PLATFORM_STM32F4_HAL) 
 #include <ArduinoWorkarounds.h>
 #include <stm32f4xx.h> // Also using ST's CubeMX to generate I/O and CPU setup source code for IAR/EWARM, not GCC ARM.
 #include <stdint.h>
 #include <string.h>
 #include <math.h>
 #define RH_HAVE_HARDWARE_SPI // using HAL (Hardware Abstraction Libraries from ST along with CMSIS, not arduino libs or pins concept.

#elif (RH_PLATFORM == RH_PLATFORM_RASPI)
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL
 #define PROGMEM
// You can enable MUTEX to protect critical sections for multithreading						   
// #define RH_USE_MUTEX
 #if (__has_include (<pigpio.h>))
  #include <RHutil_pigpio/RasPi.h>
 #else
  #include <RHutil/RasPi.h>
 #endif
 #include <string.h>
 //Define SS for CS0 or pin 24
 #define SS 8

#elif (RH_PLATFORM == RH_PLATFORM_NRF51)
 #define RH_HAVE_SERIAL
 #define PROGMEM
  #include <Arduino.h>

#elif (RH_PLATFORM == RH_PLATFORM_NRF52)
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL
 #define RH_HAVE_SPI_ATTACH_INTERRUPT
 #define PROGMEM
  #include <Arduino.h>

#elif (RH_PLATFORM == RH_PLATFORM_UNIX) 
 // Simulate the sketch on Linux and OSX
 #include <RHutil/simulator.h>
 #define RH_HAVE_SERIAL
#include <netinet/in.h> // For htons and friends

#else
 #error Platform unknown!
#endif

////////////////////////////////////////////////////
// This is an attempt to make a portable atomic block
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
 #if defined(RH_CUBE_CELL_BOARD)
  // No atomic header file available
 #elif defined(__arm__)
  #include <RHutil/atomic.h>
 #else
  #include <util/atomic.h>
 #endif

 #if defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI) || defined(RH_CUBE_CELL_BOARD)
  // Standard arduino ATOMIC block crashes on MBED version of Pico as at 2021-08-12						   // and ius not available/required for UNO R4
  #define ATOMIC_BLOCK_START {
  #define ATOMIC_BLOCK_END }						   
 #else
  // Most Arduinos
  #define ATOMIC_BLOCK_START     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  #define ATOMIC_BLOCK_END }
 #endif
#elif (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
 // UsingChipKIT Core on Arduino IDE
 #define ATOMIC_BLOCK_START unsigned int __status = disableInterrupts(); {
 #define ATOMIC_BLOCK_END } restoreInterrupts(__status);
#elif (RH_PLATFORM == RH_PLATFORM_UNO32)
 // Under old MPIDE, which has been discontinued:
 #include <peripheral/int.h>
 #define ATOMIC_BLOCK_START unsigned int __status = INTDisableInterrupts(); {
 #define ATOMIC_BLOCK_END } INTRestoreInterrupts(__status);
#elif (RH_PLATFORM == RH_PLATFORM_STM32L0) 
 #define ATOMIC_BLOCK_START uint32_t primask = __get_PRIMASK(); __disable_irq(); {
 #define ATOMIC_BLOCK_END } __set_PRIMASK(primask);
#elif (RH_PLATFORM == RH_PLATFORM_STM32F2) // Particle Photon with firmware-develop
 #define ATOMIC_BLOCK_START { int __prev = HAL_disable_irq();
 #define ATOMIC_BLOCK_END  HAL_enable_irq(__prev); }
#elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
// See hardware/esp8266/2.0.0/cores/esp8266/Arduino.h
 #define ATOMIC_BLOCK_START { uint32_t __savedPS = xt_rsil(15);
 #define ATOMIC_BLOCK_END xt_wsr_ps(__savedPS);}
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
// jPerotto see hardware/esp32/1.0.4/tools/sdk/include/esp32/xtensa/xruntime.h
// No, not supported on ESP32C3
// #define ATOMIC_BLOCK_START uint32_t volatile register ilevel = XTOS_DISABLE_ALL_INTERRUPTS;
// #define ATOMIC_BLOCK_END XTOS_RESTORE_INTLEVEL(ilevel);
// This compiles for all ESP32 but is it correct?
// #define ATOMIC_BLOCK_START int RH_ATOMIC_state = portSET_INTERRUPT_MASK_FROM_ISR();
// #define ATOMIC_BLOCK_END portCLEAR_INTERRUPT_MASK_FROM_ISR(RH_ATOMIC_state);
// These appear to be defined for all ESP32 type:
  #include "freertos/atomic.h"
 #define ATOMIC_BLOCK_START ATOMIC_ENTER_CRITICAL()
 #define ATOMIC_BLOCK_END ATOMIC_EXIT_CRITICAL()
#else 
 // TO BE DONE:
 #define ATOMIC_BLOCK_START
 #define ATOMIC_BLOCK_END
#endif

////////////////////////////////////////////////////
// Try to be compatible with systems that support yield() and multitasking
// instead of spin-loops
// Recent Arduino IDE or Teensy 3 has yield()
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(RH_CUBE_CELL_BOARD) 
 #define YIELD
#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO && ARDUINO >= 155) || (defined(TEENSYDUINO) && defined(__MK20DX128__))
 #define YIELD yield();
#elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
// ESP8266 also has it
 #define YIELD yield();
#elif (RH_PLATFORM == RH_PLATFORM_STM32L0)
 #define YIELD yield();
#elif (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS)
 //ESP32 and ESP8266 use freertos so we include calls
 //that we would normall exit a function and return to
 //the rtos in mgosYield() (E.G flush TX uart buffer
 extern "C" {
   void mgosYield(void);
 }
 #define YIELD mgosYield()
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
 // ESP32 also has it
 #define YIELD yield();
#else
 #define YIELD
#endif

////////////////////////////////////////////////////
// digitalPinToInterrupt is not available prior to Arduino 1.5.6 and 1.0.6
// See https://arduino.cc/en/Reference/attachInterrupt
#ifndef NOT_AN_INTERRUPT
 #define NOT_AN_INTERRUPT -1
#endif
#ifndef digitalPinToInterrupt
 #if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && !defined(__arm__)

  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   // Arduino Mega, Mega ADK, Mega Pro
   // 2->0, 3->1, 21->2, 20->3, 19->4, 18->5
   #define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : NOT_AN_INTERRUPT)))

  #elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
   // Arduino 1284 and 1284P - See Manicbug and Optiboot
   // 10->0, 11->1, 2->2
   #define digitalPinToInterrupt(p) ((p) == 10 ? 0 : ((p) == 11 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))

  #elif defined(__AVR_ATmega32U4__)
   // Leonardo, Yun, Micro, Pro Micro, Flora, Esplora
   // 3->0, 2->1, 0->2, 1->3, 7->4
   #define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))

  #else
   // All other arduino except Due:
   // Serial Arduino, Extreme, NG, BT, Uno, Diecimila, Duemilanove, Nano, Menta, Pro, Mini 04, Fio, LilyPad, Ethernet etc
   // 2->0, 3->1
   #define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))

  #endif
  
 #elif (RH_PLATFORM == RH_PLATFORM_UNO32) || (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
  // Hmmm, this is correct for Uno32, but what about other boards on ChipKIT Core?
  #define digitalPinToInterrupt(p) ((p) == 38 ? 0 : ((p) == 2 ? 1 : ((p) == 7 ? 2 : ((p) == 8 ? 3 : ((p) == 735 ? 4 : NOT_AN_INTERRUPT)))))

 #elif (RH_PLATFORM == RH_PLATFORM_ESP32)
  #define digitalPinToInterrupt(p) (((p) < 40) ? (p) : -1)

 #elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
  #define digitalPinToInterrupt(p) (((p) < EXTERNAL_NUM_INTERRUPTS)? (p) : NOT_AN_INTERRUPT)


 #else
  // Everything else (including Due and Teensy) interrupt number the same as the interrupt pin number
  #define digitalPinToInterrupt(p) (p)
 #endif
#endif

// On some platforms, attachInterrupt() takes a pin number, not an interrupt number
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined (__arm__) && (defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_SAM_DUE)) || defined(ARDUINO_ARCH_STM32L0) 
 #define RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
#endif

// Slave select pin, some platforms such as ATTiny do not define it.
// ESP32 pins_arduino.h uses static const uint8_t SS = <UINT>; instead
// of a #define to declare the SS constant.
#if (RH_PLATFORM != RH_PLATFORM_ESP32)
  #ifndef SS
    #define SS 10
  #endif
#endif

// Some platforms require special attributes for interrupt routines						   
#if (RH_PLATFORM == RH_PLATFORM_ESP8266)
    // interrupt handler and related code must be in RAM on ESP8266,
    // according to issue #46.
    // 2023-09-19: ESP8266 is now compatible with ESP 32 and uses IRAM_ATTR
    #define RH_INTERRUPT_ATTR IRAM_ATTR
						   
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
    #define RH_INTERRUPT_ATTR IRAM_ATTR
#else
    #define RH_INTERRUPT_ATTR
#endif

// These defs cause trouble on some versions of Arduino
#undef abs
#undef round
#undef double

// Sigh: there is no widespread adoption of htons and friends in the base code, only in some WiFi headers etc
// that have a lot of excess baggage
#if RH_PLATFORM != RH_PLATFORM_UNIX && !defined(htons)
// #ifndef htons
// These predefined macros available on modern GCC compilers
 #if   __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  // Atmel processors
  #define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
  #define ntohs(x) htons(x)
  #define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
  #define ntohl(x) htonl(x)

 #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  // Others
  #define htons(x) (x)
  #define ntohs(x) (x)
  #define htonl(x) (x)
  #define ntohl(x) (x)

 #else
  #error "RadioHead.h: Dont know how to define htons and friends for this processor" 
 #endif
#endif

// Some platforms need a mutex for multihreaded case
#ifdef RH_USE_MUTEX
 #include <pthread.h>
 #define RH_DECLARE_MUTEX(X) pthread_mutex_t X;						   
 #define RH_MUTEX_INIT(X) pthread_mutex_init(&X, NULL)
 #define RH_MUTEX_LOCK(X) pthread_mutex_lock(&X)
 #define RH_MUTEX_UNLOCK(X) pthread_mutex_unlock(&X)						   
#else
 #define RH_DECLARE_MUTEX(X)
 #define RH_MUTEX_INIT(X)
 #define RH_MUTEX_LOCK(X)
 #define RH_MUTEX_UNLOCK(X)
#endif

// This is the address that indicates a broadcast
#define RH_BROADCAST_ADDRESS 0xff

// Specifies an invalid IO pin selection
#define RH_INVALID_PIN       0xff

// Here we have some system wide macros you can define to alter the baviour of RadioHead
// in various ways. The Ardiono IDE has no way to configure such things at compile time so
// on that pltform you are forced to edit these macros here.
// On platformio you can add them to platformio.ini like, say:
// -D RH_ACK_DELAY=10`

// Uncomment this to add a delay before acknowledgement in RHReliableDatagram.
// In some networks with mixed processor speeds, may need this delay to prevent a
// fast processor sending an ack before the receiver is ready for it.
// The time is in milliseconds
// #define  RH_ACK_DELAY 10

// RH_Uncomment this to control which timer used by RH_ASK in some platforms like
// STM32:
// #define  RH_HW_TIMER TIM21`

// Uncomment this is to enable Encryption (see RHEncryptedDriver):
// But ensure you have installed the Crypto directory from arduinolibs first:
// https://rweather.github.io/arduinolibs/index.html
//#define RH_ENABLE_ENCRYPTION_MODULE

// Some platforms like RocketScream need this to see debug Serial output from within RH
// and if it goes to Serial, get a hang after a few minutes.
//#define Serial SerialUSB

#endif
