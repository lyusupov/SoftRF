## WiFiWebServer

[![arduino-library-badge](https://www.ardu-badge.com/badge/WiFiWebServer.svg?)](https://www.ardu-badge.com/WiFiWebServer)
[![GitHub release](https://img.shields.io/github/release/khoih-prog/WiFiWebServer.svg)](https://github.com/khoih-prog/WiFiWebServer/releases)
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/khoih-prog/WiFiWebServer/blob/master/LICENSE)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](#Contributing)
[![GitHub issues](https://img.shields.io/github/issues/khoih-prog/WiFiWebServer.svg)](http://github.com/khoih-prog/WiFiWebServer/issues)

<a href="https://www.buymeacoffee.com/khoihprog6" title="Donate to my libraries using BuyMeACoffee"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Donate to my libraries using BuyMeACoffee" style="height: 50px !important;width: 181px !important;" ></a>
<a href="https://www.buymeacoffee.com/khoihprog6" title="Donate to my libraries using BuyMeACoffee"><img src="https://img.shields.io/badge/buy%20me%20a%20coffee-donate-orange.svg?logo=buy-me-a-coffee&logoColor=FFDD00" style="height: 20px !important;width: 200px !important;" ></a>
<a href="https://profile-counter.glitch.me/khoih-prog/count.svg" title="Total khoih-prog Visitor count"><img src="https://profile-counter.glitch.me/khoih-prog/count.svg" style="height: 30px;width: 200px;"></a>
<a href="https://profile-counter.glitch.me/khoih-prog-WiFiWebServer/count.svg" title="WiFiWebServer Visitor count"><img src="https://profile-counter.glitch.me/khoih-prog-WiFiWebServer/count.svg" style="height: 30px;width: 200px;"></a>

---
---

## Table of Contents

* [Why do we need this WiFiWebServer library](#why-do-we-need-this-wifiwebserver-library)
  * [Features](#features)
  * [Currently Supported Boards](#currently-supported-boards)
  * [Currently supported WiFi shields/modules](#currently-supported-wifi-shieldsmodules)
* [Changelog](changelog.md) 
* [Prerequisites](#prerequisites)
* [Installation](#installation)
  * [Use Arduino Library Manager](#use-arduino-library-manager)
  * [Manual Install](#manual-install)
  * [VS Code & PlatformIO](#vs-code--platformio)
* [Packages' Patches](#packages-patches)
  * [1. For Adafruit nRF52840 and nRF52832 boards](#1-for-adafruit-nRF52840-and-nRF52832-boards)
  * [2. For Teensy boards](#2-for-teensy-boards)
  * [3. For Arduino SAM DUE boards](#3-for-arduino-sam-due-boards)
  * [4. For Arduino SAMD boards](#4-for-arduino-samd-boards)
      * [For core version v1.8.10+](#for-core-version-v1810)
      * [For core version v1.8.9-](#for-core-version-v189-)
  * [5. For Adafruit SAMD boards](#5-for-adafruit-samd-boards)
  * [6. For Seeeduino SAMD boards](#6-for-seeeduino-samd-boards)
  * [7. For STM32 boards](#7-for-stm32-boards) 
    * [7.1. For STM32 boards to use LAN8720](#71-for-stm32-boards-to-use-lan8720)
    * [7.2. For STM32 boards to use Serial1](#72-for-stm32-boards-to-use-serial1)
  * [8. For RP2040-based boards using Earle Philhower arduino-pico core](#8-for-rp2040-based-boards-using-earle-philhower-arduino-pico-core)
    * [8.1. To use BOARD_NAME](#81-to-use-board_name)
    * [8.2. To avoid compile error relating to microsecondsToClockCycles](#82-to-avoid-compile-error-relating-to-microsecondstoclockcycles)
  * [9. For Portenta_H7 boards using Arduino IDE in Linux](#9-for-portenta_h7-boards-using-arduino-ide-in-linux)
  * [10. For RTL8720DN boards using AmebaD core](#10-for-rtl8720dn-boards-using-amebad-core)
  * [11. For SAMD21 and SAMD51 boards using ArduinoCore-fab-sam core](#11-For-SAMD21-and-SAMD51-boards-using-ArduinoCore-fab-sam-core)
  * [12. For Seeeduino RP2040 boards](#12-For-Seeeduino-RP2040-boards)
  * [13. For Seeeduino nRF52840 boards](#13-For-Seeeduino-nRF52840-boards)
* [How to configure to use different WiFi Libraries](#how-to-configure-to-use-different-wifi-libraries) 
  * [1. Modify pin-to-pin connection in WiFiNINA_Generic library](#1-modify-pin-to-pin-connection-in-wifinina_generic-library)
  * [2. How to select which built-in WiFi or shield to use](#2-how-to-select-which-built-in-wifi-or-shield-to-use) 
  * [3. Important](#3-important) 
* [Usage](#usage) 
  * [Class Constructor](#class-constructor) 
  * [Basic Operations](#basic-operations) 
  * [Advanced Options](#advanced-options) 
  * [Other Function Calls](#other-function-calls) 
* [Examples](#examples)
  * [Original Examples](#original-examples) 
    * [ 1. AdvancedWebServer](examples/AdvancedWebServer)
    * [ 2. AP_SimpleWebServer](examples/AP_SimpleWebServer)
    * [ 3. HelloServer](examples/HelloServer)
    * [ 4. HelloServer2](examples/HelloServer2)
    * [ 5. HttpBasicAuth](examples/HttpBasicAuth)
    * [ 6. **MQTTClient_Auth**](examples/MQTTClient_Auth)
    * [ 7. **MQTTClient_Basic**](examples/MQTTClient_Basic)
    * [ 8. **MQTT_ThingStream**](examples/MQTT_ThingStream)
    * [ 9. PostServer](examples/PostServer)
    * [10. ScanNetworks](examples/ScanNetworks)
    * [11. SimpleAuthentication](examples/SimpleAuthentication)
    * [12. UdpNTPClient](examples/UdpNTPClient)
    * [13. UdpSendReceive](examples/UdpSendReceive)
    * [14. WebClient](examples/WebClient) 
    * [15. WebClientRepeating](examples/WebClientRepeating)
    * [16. WebServer](examples/WebServer)
    * [17. WiFiUdpNtpClient](examples/WiFiUdpNtpClient)
    * [18. multiFileProject](examples/multiFileProject) **New**
  * [HTTP and WebSocket Client New Examples](#http-and-websocket-client-new-examples) 
    * [ 1. BasicAuthGet](examples/HTTPClient/BasicAuthGet)
    * [ 2. CustomHeader](examples/HTTPClient/CustomHeader)
    * [ 3. DweetGet](examples/HTTPClient/DweetGet)
    * [ 4. DweetPost](examples/HTTPClient/DweetPost)
    * [ 5. HueBlink](examples/HTTPClient/HueBlink)
    * [ 6. node_test_server](examples/HTTPClient/node_test_server)
    * [ 7. PostWithHeaders](examples/HTTPClient/PostWithHeaders)
    * [ 8. SimpleDelete](examples/HTTPClient/SimpleDelete)
    * [ 9. SimpleGet](examples/HTTPClient/SimpleGet)
    * [10. SimpleHTTPExample](examples/HTTPClient/SimpleHTTPExample)
    * [11. SimplePost](examples/HTTPClient/SimplePost)
    * [12. SimplePut](examples/HTTPClient/SimplePut)
    * [13. SimpleWebSocket](examples/HTTPClient/SimpleWebSocket)
  * [WiFiMulti Examples](#WiFiMulti-examples) **New**
    * [ 1. AdvancedWebServer_WiFiMulti](examples/WiFiMulti/AdvancedWebServer_WiFiMulti)
    * [ 2. MQTTClient_Auth_WiFiMulti](examples/WiFiMulti/MQTTClient_Auth_WiFiMulti)
    * [ 3. MQTTClient_Basic_WiFiMulti](examples/WiFiMulti/MQTTClient_Basic_WiFiMulti)
    * [ 4. MQTT_ThingStream_WiFiMulti](examples/WiFiMulti/MQTT_ThingStream_WiFiMulti)
    * [ 5. WiFiUdpNtpClient_WiFiMulti](examples/WiFiMulti/WiFiUdpNtpClient_WiFiMulti)
    * [ 6. WebClient_WiFiMulti](examples/WiFiMulti/WebClient_WiFiMulti)
    * [ 7. WebClientRepeating_WiFiMulti](examples/WiFiMulti/WebClientRepeating_WiFiMulti)
    * [ 8. WebServer_WiFiMulti](examples/WiFiMulti/WebServer_WiFiMulti) 
* [Example AdvancedWebServer](#example-advancedwebserver)
  * [1. File AdvancedWebServer.ino](#1-file-advancedwebserverino)
  * [2. File defines.h](#2-file-definesh)
* [Debug Terminal Output Samples](#debug-terminal-output-samples)
  * [1. AdvancedWebServer on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library](#1-advancedwebserver-on-arduino-samd_nano_33_iot-using-wifinina_generic-library)
  * [2. SimpleWebSocket on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library](#2-simplewebsocket-on-arduino-samd_nano_33_iot-using-wifinina_generic-library)
  * [3. SimpleHTTPExample on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library](#3-simplehttpexample-on-arduino-samd_nano_33_iot-using-wifinina_generic-library)
  * [4. DweetPost on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library](#4-dweetpost-on-arduino-samd_nano_33_iot-using-wifinina_generic-library)
  * [5. DweetGet on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library](#5-dweetget-on-arduino-samd_nano_33_iot-using-wifinina_generic-library)
  * [6. MQTTClient_Auth on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library](#6-mqttclient_auth-on-arduino-samd_nano_33_iot-using-wifinina_generic-library)
  * [7. MQTT_ThingStream on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library](#7-mqtt_thingstream-on-arduino-samd_nano_33_iot-using-wifinina_generic-library)
  * [8. WebClientRepeating on RASPBERRY_PI_PICO with Custom WiFi using Custom WiFi Library](#8-webclientrepeating-on-raspberry_pi_pico-with-custom-wifi-using-custom-wifi-library)
  * [9. AdvancedWebServer on Arduino Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library](#9-advancedwebserver-on-arduino-nano-rp2040-connect-with-wifinina-using-wifinina_generic-library)
  * [10. SimpleHTTPExample on ESP32_DEV](#10-simplehttpexample-on-esp32_dev)
  * [11. AdvancedWebServer on PORTENTA_H7_M7 with Portenta_H7 WiFi](#11-advancedwebserver-on-portenta_h7_m7-with-portenta_h7-wifi)
  * [12. MQTTClient_Auth on ESP32_DEV](#12-mqttclient_auth-on-portenta_h7_m7-with-portenta_h7-wifi)
  * [13. WebClientRepeating on ESP32_DEV](#13-webclientrepeating-on-portenta_h7_m7-with-portenta_h7-wifi)
  * [14. AdvancedWebServer on ESP32C3_DEV with ESP WiFi](#14-AdvancedWebServer-on-ESP32C3_DEV-with-ESP-WiFi)
  * [15. AdvancedWebServer on ESP32S3_DEV with ESP WiFi](#15-AdvancedWebServer-on-ESP32S3_DEV-with-ESP-WiFi)
  * [16. AdvancedWebServer_WiFiMulti on Nano RP2040 Connect with WiFiNINA](#16-AdvancedWebServer_WiFiMulti-on-Nano-RP2040-Connect-with-WiFiNINA)
  * [17. MQTTClient_Auth_WiFiMulti on Nano RP2040 Connect with WiFiNINA](#17-MQTTClient_Auth_WiFiMulti-on-Nano-RP2040-Connect-with-WiFiNINA)
  * [18. WiFiUdpNTPClient_WiFiMulti on Nano RP2040 Connect with WiFiNINA](#18-WiFiUdpNTPClient_WiFiMulti-on-Nano-RP2040-Connect-with-WiFiNINA)
  * [19. AdvancedWebServer_WiFiMulti on RASPBERRY_PI_PICO_W](#19-AdvancedWebServer_WiFiMulti-on-RASPBERRY_PI_PICO_W)
* [Debug](#debug)
* [Troubleshooting](#troubleshooting)
* [Issues](#issues)
* [TO DO](#to-do)
* [DONE](#done)
* [Contributions and Thanks](#contributions-and-thanks)
* [Contributing](#contributing)
* [License](#license)
* [Copyright](#copyright)

---
---

### Why do we need this [WiFiWebServer library](https://github.com/khoih-prog/WiFiWebServer)

#### Features

This [WiFiWebServer library](https://github.com/khoih-prog/WiFiWebServer) is a simple yet complete WebServer library for **AVR, Portenta_H7, Teensy, SAM DUE, Arduino SAMD21, Adafruit SAMD21/SAMD51, Adafruit nRF52, ESP32/ESP8266, STM32F/L/H/G/WB/MP1, etc. boards, using WiFi modules/shields (WiFiNINA, WiFi101, U-Blox W101, W102, ESP8266/ESP32-AT, etc.)**. 

The functions are similar and compatible to those of [`ESP32 WebServer`](https://github.com/espressif/arduino-esp32/tree/master/libraries/WebServer) and [`ESP8266WebServer`](https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer) libraries to make life much easier to port sketches from ESP8266/ESP32.

This [**WiFiWebServer library**](https://github.com/khoih-prog/WiFiWebServer), from v1.1.0, also provides high-level **HTTP and WebSocket Client** with the functions are similar and compatible to those of [**ArduinoHttpClient Library**](https://github.com/arduino-libraries/ArduinoHttpClient)

The library provides supports to:

1. WiFi Client, STA and AP mode
2. TCP Server and Client
3. UDP Server and Client
4. HTTP Server and Client
5. HTTP GET and POST requests, provides argument parsing, handles one client at a time.
6. **High-level HTTP (GET, POST, PUT, PATCH, DELETE) and WebSocket Client**. From v1.1.0.

It is based on and modified from:

1. [Ivan Grokhotkov's ESP8266WebServer](https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer)
2. [Ivan Grokhotkov's ESP32 WebServer](https://github.com/espressif/arduino-esp32/tree/master/libraries/WebServer)
3. [ArduinoHttpClient Library](https://github.com/arduino-libraries/ArduinoHttpClient)

The WiFiWebServer class found in `WiFiWebServer.h` header, is a simple web server that knows how to handle HTTP requests such as GET and POST and can only support one client at a time.

The newly-created WiFiMulti-related examples in [WiFiMulti](https://github.com/khoih-prog/WiFiWebServer/tree/master/examples/WiFiMulti) demonstrate how to use the new [WiFiMulti_Generic](https://github.com/khoih-prog/WiFiMulti_Generic) library to connect to the best of **multi-WiFi APs**, with **auto-checking / auto-reconnecting** features when WiFi connection is lost.

---

#### Currently Supported Boards

This [**WiFiWebServer library**](https://github.com/khoih-prog/WiFiWebServer) currently supports these following boards:

 1. SAM DUE

 2. SAMD21

  - Arduino: ZERO, MKR, NANO_33_IOT, etc.
  - Adafruit SAMD21 (M0) : ItsyBitsy M0, Feather M0, Feather M0 Express, Metro M0 Express, Circuit Playground Express, Trinket M0, PIRkey, HalloWing M0, Crickit M0, etc.
  - Seeeduino:  LoRaWAN, Zero, Femto M0, XIAO M0, Wio GPS Board, etc.
  
 3. SAMD51

  - Adafruit SAMD51 (M4) : Metro M4, Grand Central M4, ItsyBitsy M4, Feather M44 Express, Trellis M4, Metro M4 AirLift lite, MONSTER M4SK Express, Hallowing EM4 xpress, etc.
  - Seeeduino: Wio Terminal, Grove UI Wireless
  
 4. Teensy (4.1, 4.0, 3.6, 3.5, 3,2, 3.1, 3.0, LC)
 5. All STM32F/L/H/G/WB/MP1 with more than 32KB flash memory.
 6. AVR Mega1280, 2560, ADK, 32U4, 16U4, etc. using Arduino, Adafruit or Sparkfun core. To use patch for `ArduinoSTL` library.

 7. RP2040-based boards, such as **Nano RP2040 Connect**, or **RASPBERRY_PI_PICO_W with CYW43439 WiFi**, using [**Arduino-mbed RP2040** core](https://github.com/arduino/ArduinoCore-mbed) or [**Earle Philhower's arduino-pico** core](https://github.com/earlephilhower/arduino-pico)
 
 8. RP2040-based boards, such as **RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040**, using [**Arduino-mbed RP2040** core](https://github.com/arduino/ArduinoCore-mbed) or [**Earle Philhower's arduino-pico** core](https://github.com/earlephilhower/arduino-pico).

 9. **ESP32**
 
  - ESP32 boards, such as `ESP32_DEV`, etc.
  - ESP32S2-based boards, such as `ESP32S2_DEV`, `ESP32_S2 Saola`, etc.
  - ESP32C3-based boards, such as `ESP32C3_DEV`, etc. **New**
  - ESP32_S3 (ESP32S3_DEV, ESP32_S3_BOX, UM TINYS3, UM PROS3, UM FEATHERS3, etc.) **New**
  
10. **ESP8266**

11. **Portenta_H7**

12. **nRF52 boards**, such as **AdaFruit Feather nRF52832, nRF52840 Express, BlueFruit Sense, Itsy-Bitsy nRF52840 Express, Metro nRF52840 Express, NINA_B302_ublox, NINA_B112_ublox, etc.**

13. Arduino `megaAVR` boards using Arduino core

  - UNO WiFi Rev2   : `WiFi101`      **New**
  - Nano Every      : `WiFiEspAT`    **New**
  
14. CO2 Ampel

  - SAMD      : `WiFi101`      **New**

15. STM32 using `STM32duino Maple` core. **New**

16. Sparkfun SAMD

- SAMD21, SAMD51 : `WiFiEspAT`  **New**

17. Industruino SAMD

  - D21G : WiFiEspAT  **New**

18. Tlera Corp STM32WB boards

  - Firefly-WB55RG, Nucleo-WB55RG, etc. : `WiFiEspAT`  **New**

19. Maixduino boards

  - Sipeed Maixduino, etc. : `WiFiEspAT`  **New**

20. `RTL8720DN` using Realtek `AmebaD` core  **New**

21. Arduino, Sparkfun, Adafruit, etc. AVR boards (Mega, 32U4, etc.). To use patch for `ArduinoSTL` library **New**

  - Arduino Uno / Mega / Duemilanove / Diecimila / LilyPad / Mini / Fio / Nano, etc.
  - **Arduino ATMega 16U4, 32U4** such as AVR Leonardo, Leonardo ETH, YUN, Esplora, LILYPAD_USB, AVR_ROBOT_CONTROL, AVR_ROBOT_MOTOR, AVR_INDUSTRIAL101, etc.
  - **Adafruit ATMega 32U4** such as AVR_FLORA8, AVR_FEATHER32U4, AVR_CIRCUITPLAY, AVR_ITSYBITSY32U4_5V, AVR_ITSYBITSY32U4_3V, AVR_BLUEFRUITMICRO, AVR_ADAFRUIT32U4, etc.
  - **Adafruit ATMega 328(P)** such as AVR_METRO, AVR_FEATHER328P, AVR_PROTRINKET5, AVR_PROTRINKET3, AVR_PROTRINKET5FTDI, AVR_PROTRINKET3FTDI, etc.
  - **Generic or Sparkfun AVR ATmega_32U4** such as **AVR_MAKEYMAKEY, AVR_PROMICRO, etc.**
  - **Generic or Sparkfun AVR ATmega_328(P)** such as **ARDUINO_REDBOT, ARDUINO_AVR_DIGITAL_SANDBOX, etc.**
  - **Generic or Sparkfun AVR ATmega128RFA1** such as **ATMEGA128RFA1_DEV_BOARD, etc.**

---

#### Currently supported WiFi shields/modules

1. WiFiNINA using [`WiFiNINA_Generic library`](https://github.com/khoih-prog/WiFiNINA_Generic)
2. WiFi101 using [`WiFi101_Generic library`](https://github.com/khoih-prog/WiFi101_Generic) **New**
3. u-blox W101, W102 using [`WiFiNINA_Generic library`](https://github.com/khoih-prog/WiFiNINA_Generic)
4. ESP8266-AT command using [`WiFiEspAT library`](https://github.com/jandrassy/WiFiEspAT)
5. ESP8266/ESP32-AT command using [`ESP_AT_Lib library`](https://github.com/khoih-prog/ESP_AT_Lib)
6. Built-in WiFi of ESP32, ESP8266
7. Built-in WiFi of Portenta_H7
8. Built-in CYW43439 WiFi of RASPBERRY_PI_PICO_W **New**

----
---

## Prerequisites

 1. [`Arduino IDE 1.8.19+` for Arduino](https://github.com/arduino/Arduino). [![GitHub release](https://img.shields.io/github/release/arduino/Arduino.svg)](https://github.com/arduino/Arduino/releases/latest)
 2. [`ESP32 Core 2.0.5+`](https://github.com/espressif/arduino-esp32) for ESP32-based boards. [![Latest release](https://img.shields.io/github/release/espressif/arduino-esp32.svg)](https://github.com/espressif/arduino-esp32/releases/latest/)
 3. [`ESP8266 Core 3.0.2+`](https://github.com/esp8266/Arduino) for ESP8266-based boards. [![Latest release](https://img.shields.io/github/release/esp8266/Arduino.svg)](https://github.com/esp8266/Arduino/releases/latest/).
 4. [`Arduino AVR core 1.8.6+`](https://github.com/arduino/ArduinoCore-avr) for Arduino (Use Arduino Board Manager) for AVR boards. [![GitHub release](https://img.shields.io/github/release/arduino/ArduinoCore-avr.svg)](https://github.com/arduino/ArduinoCore-avr/releases/latest)
 5. [`Teensy core v1.57+`](https://www.pjrc.com/teensy/td_download.html) for Teensy (4.1, 4.0, 3.6, 3.5, 3,2, 3.1, 3.0) boards.
 6. [`Arduino SAM DUE core v1.6.12+`](https://github.com/arduino/ArduinoCore-sam) for SAM DUE ARM Cortex-M3 boards.
 7. [`Arduino SAMD core 1.8.13+`](https://github.com/arduino/ArduinoCore-samd) for SAMD ARM Cortex-M0+ boards. [![GitHub release](https://img.shields.io/github/release/arduino/ArduinoCore-samd.svg)](https://github.com/arduino/ArduinoCore-samd/releases/latest)
 8. [`Adafruit SAMD core 1.7.11+`](https://github.com/adafruit/ArduinoCore-samd) for SAMD ARM Cortex-M0+ and M4 boards (Nano 33 IoT, etc.). [![GitHub release](https://img.shields.io/github/release/adafruit/ArduinoCore-samd.svg)](https://github.com/adafruit/ArduinoCore-samd/releases/latest)
 9. [`Seeeduino SAMD core 1.8.3+`](https://github.com/Seeed-Studio/ArduinoCore-samd) for SAMD21/SAMD51 boards (XIAO M0, Wio Terminal, etc.). [![Latest release](https://img.shields.io/github/release/Seeed-Studio/ArduinoCore-samd.svg)](https://github.com/Seeed-Studio/ArduinoCore-samd/releases/latest/)
10. [`Adafruit nRF52 v1.3.0+`](https://github.com/adafruit/Adafruit_nRF52_Arduino) for nRF52 boards such as Adafruit NRF52840_FEATHER, NRF52832_FEATHER, NRF52840_FEATHER_SENSE, NRF52840_ITSYBITSY, NRF52840_CIRCUITPLAY, NRF52840_CLUE, NRF52840_METRO, NRF52840_PCA10056, PARTICLE_XENON, **NINA_B302_ublox**, etc. [![GitHub release](https://img.shields.io/github/release/adafruit/Adafruit_nRF52_Arduino.svg)](https://github.com/adafruit/Adafruit_nRF52_Arduino/releases/latest)
11. [`Arduino Core for STM32 v2.3.0+`](https://github.com/stm32duino/Arduino_Core_STM32) for STM32F/L/H/G/WB/MP1 boards. [![GitHub release](https://img.shields.io/github/release/stm32duino/Arduino_Core_STM32.svg)](https://github.com/stm32duino/Arduino_Core_STM32/releases/latest)
12. [`Earle Philhower's arduino-pico core v2.6.3+`](https://github.com/earlephilhower/arduino-pico) for RP2040-based boards such as **RASPBERRY_PI_PICO, RASPBERRY_PI_PICO_W, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040**, etc. [![GitHub release](https://img.shields.io/github/release/earlephilhower/arduino-pico.svg)](https://github.com/earlephilhower/arduino-pico/releases/latest)
13. [`ArduinoCore-mbed mbed_rp2040, mbed_nano, mbed_portenta core 3.4.1+`](https://github.com/arduino/ArduinoCore-mbed) for Arduino (Use Arduino Board Manager) **Portenta_H7, RP2040-based boards, such as Nano_RP2040_Connect, RASPBERRY_PI_PICO**. [![GitHub release](https://img.shields.io/github/release/arduino/ArduinoCore-mbed.svg)](https://github.com/arduino/ArduinoCore-mbed/releases/latest)
14. [`Arduino megaAVR core 1.8.7+`](https://github.com/arduino/ArduinoCore-megaavr/releases) for Arduino megaAVR boards such as **UNO_WIFI_REV2, NANO_EVERY**

15. [`Functional-Vlpp library v1.0.2+`](https://github.com/khoih-prog/functional-vlpp) to use server's lambda function. To install, check [![arduino-library-badge](https://www.ardu-badge.com/badge/Functional-Vlpp.svg?)](https://www.ardu-badge.com/Functional-Vlpp)
16. [`WiFiNINA_Generic library v1.8.15-1+`](https://github.com/khoih-prog/WiFiNINA_Generic) if using WiFiNINA. To install, check [![arduino-library-badge](https://www.ardu-badge.com/badge/WiFiNINA_Generic.svg?)](https://www.ardu-badge.com/WiFiNINA_Generic).
17. [`ESP_AT_Lib library v1.4.1+`](https://github.com/khoih-prog/ESP_AT_Lib) if using ESP8288/ESP32-AT shields. To install, check [![arduino-library-badge](https://www.ardu-badge.com/badge/ESP_AT_Lib.svg?)](https://www.ardu-badge.com/ESP_AT_Lib).
18. [`WiFi101_Generic library v1.0.0+`](https://github.com/khoih-prog/WiFi101_Generic) to use SAMD MKR1000, etc. boards with WiFi101 for sending larger data. To install, check [![arduino-library-badge](https://www.ardu-badge.com/badge/WiFi101_Generic.svg?)](https://www.ardu-badge.com/WiFi101_Generic).
19. [`WiFiEspAT library v1.4.1+`](https://github.com/jandrassy/WiFiEspAT) if using ESP8288/ESP32-AT shields. [![GitHub release](https://img.shields.io/github/release/jandrassy/WiFiEspAT.svg)](https://github.com/jandrassy/WiFiEspAT/releases/latest)
20. [`WiFiMulti_Generic library v1.2.2+`](https://github.com/khoih-prog/WiFiMulti_Generic) to use WiFiMulti function. To install, check [![arduino-library-badge](https://www.ardu-badge.com/badge/WiFiMulti_Generic.svg?)](https://www.ardu-badge.com/WiFiMulti_Generic). **New**


---

## Installation

### Use Arduino Library Manager
The best and easiest way is to use `Arduino Library Manager`. Search for [**WiFiWebServer**](https://github.com/khoih-prog/WiFiWebServer), then select / install the latest version.
You can also use this link [![arduino-library-badge](https://www.ardu-badge.com/badge/WiFiWebServer.svg?)](https://www.ardu-badge.com/WiFiWebServer) for more detailed instructions.

### Manual Install

1. Navigate to [**WiFiWebServer**](https://github.com/khoih-prog/WiFiWebServer) page.
2. Download the latest release `WiFiWebServer-master.zip`.
3. Extract the zip file to `WiFiWebServer-master` directory 
4. Copy the whole `WiFiWebServer-master` folder to Arduino libraries' directory such as `~/Arduino/libraries/`.

### VS Code & PlatformIO:

1. Install [VS Code](https://code.visualstudio.com/)
2. Install [PlatformIO](https://platformio.org/platformio-ide)
3. Install [**WiFiWebServer** library](https://registry.platformio.org/libraries/khoih-prog/WiFiWebServer) by using [Library Manager](https://registry.platformio.org/libraries/khoih-prog/WiFiWebServer/installation). Search for **WiFiWebServer** in [Platform.io Author's Libraries](https://platformio.org/lib/search?query=author:%22Khoi%20Hoang%22)
4. Use included [platformio.ini](platformio/platformio.ini) file from examples to ensure that all dependent libraries will installed automatically. Please visit documentation for the other options and examples at [Project Configuration File](https://docs.platformio.org/page/projectconf.html)

---
---

### Packages' Patches

#### 1. For Adafruit nRF52840 and nRF52832 boards

**To be able to compile, run and automatically detect and display BOARD_NAME on nRF52840/nRF52832 boards**, you have to copy the whole [nRF52 Packages_Patches](Packages_Patches/adafruit/hardware/nrf52/1.3.0) directory into Adafruit nRF52 directory (~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0). 

Supposing the Adafruit nRF52 version is 1.3.0. These files must be copied into the directory:
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/platform.txt`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/boards.txt`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/cores/nRF5/Udp.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/cores/nRF5/Print.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/cores/nRF5/Print.cpp`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/variants/NINA_B302_ublox/variant.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/variants/NINA_B302_ublox/variant.cpp`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/variants/NINA_B112_ublox/variant.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/variants/NINA_B112_ublox/variant.cpp`
- **`~/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/cores/nRF5/Udp.h`**

Whenever a new version is installed, remember to copy these files into the new version directory. For example, new version is x.yy.z
These files must be copied into the directory:

- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/platform.txt`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/boards.txt`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/cores/nRF5/Udp.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/cores/nRF5/Print.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/cores/nRF5/Print.cpp`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/variants/NINA_B302_ublox/variant.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/variants/NINA_B302_ublox/variant.cpp`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/variants/NINA_B112_ublox/variant.h`
- `~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/variants/NINA_B112_ublox/variant.cpp`
- **`~/.arduino15/packages/adafruit/hardware/nrf52/x.yy.z/cores/nRF5/Udp.h`**

#### 2. For Teensy boards
 
 **To be able to compile and run on Teensy boards**, you have to copy the files in [**Packages_Patches for Teensy directory**](Packages_Patches/hardware/teensy/avr) into Teensy hardware directory (./arduino-1.8.19/hardware/teensy/avr/boards.txt). 

Supposing the Arduino version is 1.8.19. These files must be copied into the directory:

- `./arduino-1.8.19/hardware/teensy/avr/boards.txt`
- `./arduino-1.8.19/hardware/teensy/avr/cores/teensy/Stream.h`
- `./arduino-1.8.19/hardware/teensy/avr/cores/teensy3/Stream.h`
- `./arduino-1.8.19/hardware/teensy/avr/cores/teensy4/Stream.h`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
These files must be copied into the directory:

- `./arduino-x.yy.zz/hardware/teensy/avr/boards.txt`
- `./arduino-x.yy.zz/hardware/teensy/avr/cores/teensy/Stream.h`
- `./arduino-x.yy.zz/hardware/teensy/avr/cores/teensy3/Stream.h`
- `./arduino-x.yy.zz/hardware/teensy/avr/cores/teensy4/Stream.h`

#### 3. For Arduino SAM DUE boards
 
 **To be able to compile and run on SAM DUE boards**, you have to copy the whole [SAM DUE](Packages_Patches/arduino/hardware/sam/1.6.12) directory into Arduino sam directory (~/.arduino15/packages/arduino/hardware/sam/1.6.12). 

Supposing the Arduino SAM core version is 1.6.12. This file must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/sam/1.6.12/platform.txt`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
This file must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/sam/x.yy.zz/platform.txt`

#### 4. For Arduino SAMD boards
 
 ***To be able to compile, run and automatically detect and display BOARD_NAME on Arduino SAMD (Nano-33-IoT, etc) boards***, you have to copy the whole [Arduino SAMD Packages_Patches](Packages_Patches/arduino/hardware/samd/1.8.13) directory into Arduino SAMD directory (~/.arduino15/packages/arduino/hardware/samd/1.8.13).
 
#### For core version v1.8.10+

Supposing the Arduino SAMD version is 1.8.13. Now only one file must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/samd/1.8.13/platform.txt`

Whenever a new version is installed, remember to copy this files into the new version directory. For example, new version is x.yy.zz

This file must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/samd/x.yy.zz/platform.txt`
 
#### For core version v1.8.9-

Supposing the Arduino SAMD version is 1.8.9. These files must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/samd/1.8.9/platform.txt`
- ***`~/.arduino15/packages/arduino/hardware/samd/1.8.9/cores/arduino/Arduino.h`***

Whenever a new version is installed, remember to copy these files into the new version directory. For example, new version is x.yy.z

These files must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/samd/x.yy.z/platform.txt`
- ***`~/.arduino15/packages/arduino/hardware/samd/x.yy.z/cores/arduino/Arduino.h`***
 
 This is mandatory to fix the ***notorious Arduino SAMD compiler error***. See [Improve Arduino compatibility with the STL (min and max macro)](https://github.com/arduino/ArduinoCore-samd/pull/399)
 
```
 ...\arm-none-eabi\include\c++\7.2.1\bits\stl_algobase.h:243:56: error: macro "min" passed 3 arguments, but takes just 2
     min(const _Tp& __a, const _Tp& __b, _Compare __comp)
```

Whenever the above-mentioned compiler error issue is fixed with the new Arduino SAMD release, you don't need to copy the `Arduino.h` file anymore.

#### 5. For Adafruit SAMD boards
 
 ***To be able to compile, run and automatically detect and display BOARD_NAME on Adafruit SAMD (Itsy-Bitsy M4, etc) boards***, you have to copy the whole [Adafruit SAMD Packages_Patches](Packages_Patches/adafruit/hardware/samd/1.7.11) directory into Adafruit samd directory (~/.arduino15/packages/adafruit/hardware/samd/1.7.11). 

Supposing the Adafruit SAMD core version is 1.7.11. These files must be copied into the directory:

- `~/.arduino15/packages/adafruit/hardware/samd/1.7.11/platform.txt`
- `~/.arduino15/packages/adafruit/hardware/samd/1.7.11/cores/arduino/Print.h`
- `~/.arduino15/packages/adafruit/hardware/samd/1.7.11/cores/arduino/Print.cpp`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
These files must be copied into the directory:

- `~/.arduino15/packages/adafruit/hardware/samd/x.yy.zz/platform.txt`
- `~/.arduino15/packages/adafruit/hardware/samd/x.yy.zz/cores/arduino/Print.h`
- `~/.arduino15/packages/adafruit/hardware/samd/x.yy.zz/cores/arduino/Print.cpp`

#### 6. For Seeeduino SAMD boards
 
 ***To be able to compile, run and automatically detect and display BOARD_NAME on Seeeduino SAMD (XIAO M0, Wio Terminal, etc) boards***, you have to copy the whole [Seeeduino SAMD Packages_Patches](Packages_Patches/Seeeduino/hardware/samd/1.8.3) directory into Seeeduino samd directory (~/.arduino15/packages/Seeeduino/hardware/samd/1.8.3). 

Supposing the Seeeduino SAMD core version is 1.8.3. These files must be copied into the directory:

- `~/.arduino15/packages/Seeeduino/hardware/samd/1.8.3/platform.txt`
- `~/.arduino15/packages/Seeeduino/hardware/samd/1.8.3/cores/arduino/Arduino.h`
- `~/.arduino15/packages/Seeeduino/hardware/samd/1.8.3/cores/arduino/Print.h`
- `~/.arduino15/packages/Seeeduino/hardware/samd/1.8.3/cores/arduino/Print.cpp`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
These files must be copied into the directory:

- `~/.arduino15/packages/Seeeduino/hardware/samd/x.yy.zz/platform.txt`
- `~/.arduino15/packages/Seeeduino/hardware/samd/x.yy.zz/cores/arduino/Arduino.h`
- `~/.arduino15/packages/Seeeduino/hardware/samd/x.yy.zz/cores/arduino/Print.h`
- `~/.arduino15/packages/Seeeduino/hardware/samd/x.yy.zz/cores/arduino/Print.cpp`

#### 7. For STM32 boards

#### 7.1 For STM32 boards to use LAN8720

For `Generic STM32F4 series` boards, such as `STM32F407VE`, using `LAN8720`, please use STM32 core `v2.2.0` as breaking core `v2.3.0` creates the compile error.

---

To use LAN8720 on some STM32 boards 

- **Nucleo-144 (F429ZI, NUCLEO_F746NG, NUCLEO_F746ZG, NUCLEO_F756ZG)**
- **Discovery (DISCO_F746NG)**
- **STM32F4 boards (BLACK_F407VE, BLACK_F407VG, BLACK_F407ZE, BLACK_F407ZG, BLACK_F407VE_Mini, DIYMORE_F407VGT, FK407M1)**

you have to copy the files [stm32f4xx_hal_conf_default.h](Packages_Patches/STM32/hardware/stm32/2.2.0/system/STM32F4xx) and [stm32f7xx_hal_conf_default.h](Packages_Patches/STM32/hardware/stm32/2.2.0/system/STM32F7xx) into STM32 stm32 directory (~/.arduino15/packages/STM32/hardware/stm32/2.2.0/system) to overwrite the old files.

Supposing the STM32 stm32 core version is 2.2.0. These files must be copied into the directory:

- `~/.arduino15/packages/STM32/hardware/stm32/2.2.0/system/STM32F4xx/stm32f4xx_hal_conf_default.h` for STM32F4.
- `~/.arduino15/packages/STM32/hardware/stm32/2.2.0/system/STM32F7xx/stm32f7xx_hal_conf_default.h` for Nucleo-144 STM32F7.

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz,
these files must be copied into the corresponding directory:

- `~/.arduino15/packages/STM32/hardware/stm32/x.yy.zz/system/STM32F4xx/stm32f4xx_hal_conf_default.h`
- `~/.arduino15/packages/STM32/hardware/stm32/x.yy.zz/system/STM32F7xx/stm32f7xx_hal_conf_default.h`


#### 7.2 For STM32 boards to use Serial1

**To use Serial1 on some STM32 boards without Serial1 definition (Nucleo-144 NUCLEO_F767ZI, Nucleo-64 NUCLEO_L053R8, etc.) boards**, you have to copy the files [STM32 variant.h](Packages_Patches/STM32/hardware/stm32/2.3.0) into STM32 stm32 directory (~/.arduino15/packages/STM32/hardware/stm32/2.3.0). You have to modify the files corresponding to your boards, this is just an illustration how to do.

Supposing the STM32 stm32 core version is 2.3.0. These files must be copied into the directory:

- `~/.arduino15/packages/STM32/hardware/stm32/2.3.0/variants/STM32F7xx/F765Z(G-I)T_F767Z(G-I)T_F777ZIT/NUCLEO_F767ZI/variant.h` for Nucleo-144 NUCLEO_F767ZI.
- `~/.arduino15/packages/STM32/hardware/stm32/2.3.0/variants/STM32L0xx/L052R(6-8)T_L053R(6-8)T_L063R8T/NUCLEO_L053R8/variant.h` for Nucleo-64 NUCLEO_L053R8.

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz,
these files must be copied into the corresponding directory:

- `~/.arduino15/packages/STM32/hardware/stm32/x.yy.zz/variants/STM32F7xx/F765Z(G-I)T_F767Z(G-I)T_F777ZIT/NUCLEO_F767ZI/variant.h`
- `~/.arduino15/packages/STM32/hardware/stm32/x.yy.zz/variants/STM32L0xx/L052R(6-8)T_L053R(6-8)T_L063R8T/NUCLEO_L053R8/variant.h`

#### 8. For RP2040-based boards using [Earle Philhower arduino-pico core](https://github.com/earlephilhower/arduino-pico)

#### 8.1 To use BOARD_NAME

 **To be able to automatically detect and display BOARD_NAME on RP2040-based boards (RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040, GENERIC_RP2040, etc) boards**, you have to copy the file [RP2040 platform.txt](Packages_Patches/rp2040/hardware/rp2040/1.4.0) into rp2040 directory (~/.arduino15/packages/rp2040/hardware/rp2040/1.4.0). 

Supposing the rp2040 core version is 1.4.0. This file must be copied into the directory:

- `~/.arduino15/packages/rp2040/hardware/rp2040/1.4.0/platform.txt`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
This file must be copied into the directory:

- `~/.arduino15/packages/rp2040/hardware/rp2040/x.yy.zz/platform.txt`

With core after v1.5.0, this step is not necessary anymore thanks to the PR [Add -DBOARD_NAME="{build.board}" #136](https://github.com/earlephilhower/arduino-pico/pull/136).

#### 8.2 To avoid compile error relating to microsecondsToClockCycles

Some libraries, such as [Adafruit DHT-sensor-library](https://github.com/adafruit/DHT-sensor-library), require the definition of microsecondsToClockCycles(). **To be able to compile and run on RP2040-based boards**, you have to copy the files in [**RP2040 Arduino.h**](Packages_Patches/rp2040/hardware/rp2040/1.4.0/cores/rp2040/Arduino.h) into rp2040 directory (~/.arduino15/packages/rp2040/hardware/rp2040/1.4.0).

Supposing the rp2040 core version is 1.4.0. This file must be copied to replace:

- `~/.arduino15/packages/rp2040/hardware/rp2040/1.4.0/cores/rp2040/Arduino.h`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
This file must be copied to replace:

- `~/.arduino15/packages/rp2040/hardware/rp2040/x.yy.zz/cores/rp2040/Arduino.h`

With core after v1.5.0, this step is not necessary anymore thanks to the PR [Add defs for compatibility #142](https://github.com/earlephilhower/arduino-pico/pull/142).


#### 9. For Portenta_H7 boards using Arduino IDE in Linux

  **To be able to upload firmware to Portenta_H7 using Arduino IDE in Linux (Ubuntu, etc.)**, you have to copy the file [portenta_post_install.sh](Packages_Patches/arduino/hardware/mbed_portenta/3.4.1/portenta_post_install.sh) into mbed_portenta directory (~/.arduino15/packages/arduino/hardware/mbed_portenta/3.4.1/portenta_post_install.sh). 
  
  Then run the following command using `sudo`
  
```
$ cd ~/.arduino15/packages/arduino/hardware/mbed_portenta/3.4.1
$ chmod 755 portenta_post_install.sh
$ sudo ./portenta_post_install.sh
```

This will create the file `/etc/udev/rules.d/49-portenta_h7.rules` as follows:

```
# Portenta H7 bootloader mode UDEV rules

SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="035b", GROUP="plugdev", MODE="0666"
```

Supposing the ArduinoCore-mbed core version is 3.4.1. Now only one file must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/mbed_portenta/3.4.1/portenta_post_install.sh`

Whenever a new version is installed, remember to copy this files into the new version directory. For example, new version is x.yy.zz

This file must be copied into the directory:

- `~/.arduino15/packages/arduino/hardware/mbed_portenta/x.yy.zz/portenta_post_install.sh`


#### 10. For RTL8720DN boards using AmebaD core
 
 To avoid compile error relating to PROGMEM, you have to copy the file [Realtek AmebaD core pgmspace.h](Packages_Patches/realtek/hardware/AmebaD/3.1.4/cores/ambd/avr/pgmspace.h) into Realtek AmebaD directory (~/.arduino15/packages/realtek/hardware/AmebaD/3.1.4/cores/ambd/avr/pgmspace.h). 

Supposing the Realtek AmebaD core version is 3.1.4. This file must be copied into the directory:

- `~/.arduino15/packages/realtek/hardware/AmebaD/3.1.4/cores/ambd/avr/pgmspace.h`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
This file must be copied into the directory:

- `~/.arduino15/packages/realtek/hardware/AmebaD/x.yy.zz/cores/ambd/avr/pgmspace.h`


#### 11. For SAMD21 and SAMD51 boards using ArduinoCore-fab-sam core
 
 To avoid compile error relating to SAMD21/SAMD51, you have to copy the file [ArduinoCore-fab-sam core pgmspace.h](Packages_Patches/Fab_SAM_Arduino/hardware/samd/1.9.0/boards.txt) into `ArduinoCore-fab-sam` samd directory (~/.arduino15/packages/Fab_SAM_Arduino/hardware/samd/1.9.0/boards.txt). 

Supposing the `ArduinoCore-fab-sam` samd core version is 1.9.0. This file must be copied into the directory:

- `~/.arduino15/packages/Fab_SAM_Arduino/hardware/samd/1.9.0/boards.txt`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
This file must be copied into the directory:

- `~/.arduino15/packages/Fab_SAM_Arduino/hardware/samd/x.yy.zz/boards.txt`


#### 12. For Seeeduino RP2040 boards
 
 ***To be able to compile, run and automatically detect and display BOARD_NAME on Seeeduino RP2040 (XIAO RP2040, Wio RP2040 Mini) boards***, you have to copy the whole [Seeeduino RP2040 Packages_Patches](Packages_Patches/Seeeduino/hardware/rp2040/2.7.2) directory into Seeeduino samd directory (~/.arduino15/packages/Seeeduino/hardware/rp2040/2.7.2). 

Supposing the Seeeduino RP2040 core version is 2.7.2. These files must be copied into the directory:

- `~/.arduino15/packages/Seeeduino/hardware/rp2040/2.7.2/boards.txt`
- `~/.arduino15/packages/Seeeduino/hardware/rp2040/2.7.2/variants/Seeed_XIAO_RP2040/pins_arduino.h`

Whenever a new version is installed, remember to copy this file into the new version directory. For example, new version is x.yy.zz
These files must be copied into the directory:

- `~/.arduino15/packages/Seeeduino/hardware/samd/x.yy.zz/boards.txt`
- `~/.arduino15/packages/Seeeduino/hardware/samd/x.yy.zz/variants/Seeed_XIAO_RP2040/pins_arduino.h`


---

#### 13. For Seeeduino nRF52840 boards

**To be able to compile and run on Xiao nRF52840 boards**, you have to copy the whole [nRF52 1.0.0](Packages_Patches/Seeeduino/hardware/nrf52/1.0.0) directory into Seeeduino nRF52 directory (~/.arduino15/packages/Seeeduino/hardware/nrf52/1.0.0). 

Supposing the Seeeduino nRF52 version is 1.0.0. These files must be copied into the directory:

- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/1.0.0/platform.txt`**
- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/1.0.0/cores/nRF5/Print.h`**
- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/1.0.0/cores/nRF5/Print.cpp`**
- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/1.0.0/cores/nRF5/Udp.h`**

Whenever a new version is installed, remember to copy these files into the new version directory. For example, new version is x.yy.z
These files must be copied into the directory:

- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/x.yy.z/platform.txt`**
- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/x.yy.z/cores/nRF5/Print.h`**
- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/x.yy.z/cores/nRF5/Print.cpp`**
- **`~/.arduino15/packages/Seeeduino/hardware/nrf52/x.yy.z/cores/nRF5/Udp.h`**


---
---

### How to configure to use different WiFi Libraries

#### 1. Modify pin-to-pin connection in WiFiNINA_Generic library

Please change the pin-to-pin connection in `~/Arduino/libraries/src/WiFiNINA_Pinout_Generic.h` to match actual connection if using WiFiNINA with [`WiFiNINA_Generic library`](https://github.com/khoih-prog/WiFiNINA_Generic).

For example

```cpp
#elif ( defined(NRF52840_FEATHER) || defined(NRF52832_FEATHER) || defined(NRF52_SERIES) || defined(ARDUINO_NRF52_ADAFRUIT) || \
        defined(NRF52840_FEATHER_SENSE) || defined(NRF52840_ITSYBITSY) || defined(NRF52840_CIRCUITPLAY) || defined(NRF52840_CLUE) || \
        defined(NRF52840_METRO) || defined(NRF52840_PCA10056) || defined(PARTICLE_XENON) || defined(NINA_B302_ublox) || defined(NINA_B112_ublox) )

  #warning You have to modify pin usage according to actual connection for NRF528XX
  // To define pin out for WiFiNINA here

  //#define PINS_COUNT           (60u)
  //NINA
  #define NINA_GPIO0  (26u)                             //26
  #define NINA_RESETN (27u)
  #define NINA_ACK    (28u)

  #define SPIWIFI_SS       24   //PIN_SPI1_SS            //24
  #define SPIWIFI_ACK      28   //NINA_ACK               //28 
  #define SPIWIFI_RESET    27   //NINA_RESETN            //27
```

#### 2. How to select which built-in WiFi or shield to use

- To use W102-based WiFiNINA, define in the sketch:

```cpp
#define USE_WIFI_NINA         true
```

- To use built-in WiFi101 or shield:

```cpp
#define USE_WIFI_NINA         false
#define USE_WIFI101           true
```

- To use MKR1000 with built-in WiFi101:

```cpp
// Don't care false or true
#define USE_WIFI_NINA         false
```

- For boards other than MKR1000, to use another WiFi library with the standard **WiFi.h**, such as [`WiFiEspAT library`](https://github.com/jandrassy/WiFiEspAT) library

```cpp
#define USE_WIFI_NINA         false
```

- To use another WiFi library without the standard **WiFi.h**

For example, WiFi_XYZ library uses **WiFi_XYZ.h**

```cpp
#define USE_WIFI_NINA         false
#define USE_WIFI_CUSTOM       true

...
//Must be placed before #include <WiFiWebServer.h>
#include <WiFi_XYZ.h>
#include <WiFiWebServer.h>
```

#### 3. Important

- The **WiFiEsp, WiFi_Link libraries are not supported**. Don't use unless you know how to modify those libraries.
- Requests to support for any custom WiFi library will be ignored. **Use at your own risk**.

---
---

### Usage

#### Class Constructor

```cpp
  WiFiWebServer server(80);
```

Creates the WiFiWebServer class object.

*Parameters:* 
 
host port number: ``int port`` (default is the standard HTTP port 80)

---

#### Basic Operations

**Starting the server**

```cpp
  void begin();
```

**Handling incoming client requests**

```cpp
  void handleClient();
```

**Disabling the server**

```cpp
  void close();
  void stop();
```

Both methods function the same

**Client request handlers**

```cpp
  void on();
  void addHandler();
  void onNotFound();
  void onFileUpload();	
```

Example:

```cpp
  server.on("/", handlerFunction);
  server.onNotFound(handlerFunction);   // called when handler is not assigned
  server.onFileUpload(handlerFunction); // handle file uploads
```

**Sending responses to the client**

```cpp
  void send();
  void send_P();
```

`Parameters:`

`code` - HTTP response code, can be `200` or `404`, etc.

`content_type` - HTTP content type, like `"text/plain"` or `"image/png"`, etc.

`content` - actual content body

---

#### Advanced Options

**Getting information about request arguments**

```cpp
  const String & arg();
  const String & argName();
  int   args();
  bool  hasArg();
```

`Function usage:`

`arg` - get request argument value, use `arg("plain")` to get POST body
	
`argName` - get request argument name
	
`args` - get arguments count
	
`hasArg` - check if argument exist

**Getting information about request headers**

```cpp
  const String & header();
  const String & headerName();
  const String & hostHeader();
  int   headers();
  bool  hasHeader();
``` 

`Function usage:`

`header` - get request header value

`headerName` - get request header name

`hostHeader` - get request host header if available, else empty string
  
`headers` - get header count
	
`hasHeader` - check if header exist

**Authentication**

```cpp
  bool authenticate();
  void requestAuthentication();
```

`Function usage:`

`authenticate` - server authentication, returns true if client is authenticated else false

`requestAuthentication` - sends authentication failure response to the client

`Example Usage:`

```cpp

  if(!server.authenticate(username, password))
  {
    server.requestAuthentication();
  }
```

---

#### Other Function Calls

```cpp
  const String& uri(); // get the current uri
  HTTPMethod    method(); // get the current method 
  WiFiClient    client(); // get the current client
  HTTPUpload&   upload(); // get the current upload
  
  void setContentLength(); // set content length
  void sendHeader(); // send HTTP header
  void sendContent(); // send content
  void sendContent_P(); 
  void collectHeaders(); // set the request headers to collect
  void serveStatic();
  
  size_t streamFile();
```

---
---

### Examples:

#### Original Examples

 1. [AdvancedWebServer](examples/AdvancedWebServer)
 2. [AP_SimpleWebServer](examples/AP_SimpleWebServer)
 3. [HelloServer](examples/HelloServer)
 4. [HelloServer2](examples/HelloServer2)
 5. [HttpBasicAuth](examples/HttpBasicAuth)
 6. [MQTTClient_Auth](examples/MQTTClient_Auth)
 7. [MQTTClient_Basic](examples/MQTTClient_Basic)
 8. [MQTT_ThingStream](examples/MQTT_ThingStream)
 9. [PostServer](examples/PostServer)
10. [ScanNetworks](examples/ScanNetworks)
11. [SimpleAuthentication](examples/SimpleAuthentication)
12. [UdpNTPClient](examples/UdpNTPClient)
13. [UdpSendReceive](examples/UdpSendReceive)
14. [WebClient](examples/WebClient)
15. [WebClientRepeating](examples/WebClientRepeating)
16. [WebServer](examples/WebServer)
17. [WiFiUdpNtpClient](examples/WiFiUdpNtpClient)
18. [multiFileProject](examples/multiFileProject) **New**

#### HTTP and WebSocket Client New Examples

 1. [BasicAuthGet](examples/HTTPClient/BasicAuthGet)
 2. [CustomHeader](examples/HTTPClient/CustomHeader)
 3. [DweetGet](examples/HTTPClient/DweetGet)
 4. [DweetPost](examples/HTTPClient/DweetPost)
 5. [HueBlink](examples/HTTPClient/HueBlink)
 6. [node_test_server](examples/HTTPClient/node_test_server)
 7. [PostWithHeaders](examples/HTTPClient/PostWithHeaders)
 8. [SimpleDelete](examples/HTTPClient/SimpleDelete)
 9. [SimpleGet](examples/HTTPClient/SimpleGet)
10. [SimpleHTTPExample](examples/HTTPClient/SimpleHTTPExample)
11. [SimplePost](examples/HTTPClient/SimplePost)
12. [SimplePut](examples/HTTPClient/SimplePut)
13. [SimpleWebSocket](examples/HTTPClient/SimpleWebSocket)

#### WiFiMulti Examples

 1. [AdvancedWebServer_WiFiMulti](examples/WiFiMulti/AdvancedWebServer_WiFiMulti)
 2. [MQTTClient_Auth_WiFiMulti](examples/WiFiMulti/MQTTClient_Auth_WiFiMulti)
 3. [MQTTClient_Basic_WiFiMulti](examples/WiFiMulti/MQTTClient_Basic_WiFiMulti)
 4. [MQTT_ThingStream_WiFiMulti](examples/WiFiMulti/MQTT_ThingStream_WiFiMulti)
 5. [WiFiUdpNtpClient_WiFiMulti](examples/WiFiMulti/WiFiUdpNtpClient_WiFiMulti)
 6. [WebClient_WiFiMulti](examples/WiFiMulti/WebClient_WiFiMulti)
 7. [WebClientRepeating_WiFiMulti](examples/WiFiMulti/WebClientRepeating_WiFiMulti)
 8. [WebServer_WiFiMulti](examples/WiFiMulti/WebServer_WiFiMulti)

---

### Example [AdvancedWebServer](examples/AdvancedWebServer)

#### 1. File [AdvancedWebServer.ino](examples/AdvancedWebServer/AdvancedWebServer.ino)


https://github.com/khoih-prog/WiFiWebServer/blob/9094e545cd4da8007fd321212a27a36edd2d3da2/examples/AdvancedWebServer/AdvancedWebServer.ino#L40-L327

#### 2. File [defines.h](examples/AdvancedWebServer/defines.h)


https://github.com/khoih-prog/WiFiWebServer/blob/9094e545cd4da8007fd321212a27a36edd2d3da2/examples/AdvancedWebServer/defines.h#L12-L409

---
---

### Debug Terminal Output Samples

#### 1. AdvancedWebServer on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library

The following are debug terminal output and screen shot when running example [**AdvancedWebServer**](examples/AdvancedWebServer) on **SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library**

<p align="center">
    <img src="https://github.com/khoih-prog/WiFiWebServer/raw/master/pics/AdvancedWebServer.png">
</p>

```cpp
Starting AdvancedServer on SAMD_NANO_33_IOT
WiFiWebServer v1.10.1
[NN] ===============================
[NN] 
Used/default SPI pinout: 
[NN] MOSI:  11
[NN] MISO:  12
[NN] SCK:  13
[NN] SS:  10
[NN] ===============================
[NN] 
Used/default NINA pinout: 
[NN] NINA_GPIO0:  26
[NN] NINA_RESETN/SPIWIFI_RESET:  27
[NN] NINA_ACK:  28
[NN] SS:  10
[NN] ===============================
[NN] 
Actual final pinout to used: 
[NN] SPIWIFI_SS:  24
[NN] SLAVESELECT/SPIWIFI_SS:  24
[NN] SLAVEREADY/SPIWIFI_ACK/NINA_ACK:  28
[NN] SLAVERESET/SPIWIFI_RESET/NINA_RESETN:  27
[NN] ===============================
Connecting to WPA SSID: HueNet1
HTTP server started @ 192.168.2.118
H[WIFI] String Len = 0, extend to 2048
WiFiWebServer::handleClient: New Client
method:  GET
url:  /
search:
headerName: Host
headerValue: 192.168.2.118
headerName: Connection
headerValue: keep-alive
headerName: Cache-Control
headerValue: max-age=0
headerName: DNT
headerValue: 1
headerName: Upgrade-Insecure-Requests
headerValue: 1
headerName: User-Agent
headerValue: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/84.0.4147.89 Safari/537.36
headerName: Accept
headerValue: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9
headerName: Accept-Encoding
headerValue: gzip, deflate
headerName: Accept-Language
headerValue: en-GB,en-US;q=0.9,en;q=0.8
headerName: Cookie
headerValue: NINASESSIONID=0
args:
args count:  0
args:
args count:  0
Request: /
Arguments: 
Final list of key/value pairs:
WiFiWebServer::_handleRequest handle
WiFiWebServer::send1: len =  330
content =  <html><head><meta http-equiv='refresh' content='5'/><title>WiFiNINA SAMD_NANO_33_IOT</title><style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style></head><body><h1>Hello from WiFiNINA</h1><h2>on SAMD_NANO_33_IOT</h2><p>Uptime: 00:00:20</p><img src="/test.svg" /></body></html>
WiFiWebServer::_prepareHeader sendHeader Conn close
WiFiWebServer::send1: write header =  HTTP/1.1 200 OK
Content-Type: text/html
Content-Length: 330
Connection: close


WiFiWebServer::sendContent: Client.write content:  <html><head><meta http-equiv='refresh' content='5'/><title>WiFiNINA SAMD_NANO_33_IOT</title><style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style></head><body><h1>Hello from WiFiNINA</h1><h2>on SAMD_NANO_33_IOT</h2><p>Uptime: 00:00:20</p><img src="/test.svg" /></body></html>
WiFiWebServer::_handleRequest OK
WiFiWebServer::handleClient: Client disconnected
WiFiWebServer::handleClient: Don't keepCurrentClient
WiFiWebServer::handleClient: Client disconnected
WiFiWebServer::handleClient: New Client
method:  GET
url:  /test.svg
search:
headerName: Host
headerValue: 192.168.2.118
headerName: Connection
headerValue: keep-alive
headerName: User-Agent
headerValue: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/84.0.4147.89 Safari/537.36
headerName: DNT
headerValue: 1
headerName: Accept
headerValue: image/webp,image/apng,image/*,*/*;q=0.8
headerName: Referer
headerValue: http://192.168.2.118/
headerName: Accept-Encoding
headerValue: gzip, deflate
headerName: Accept-Language
headerValue: en-GB,en-US;q=0.9,en;q=0.8
headerName: Cookie
headerValue: NINASESSIONID=0
args:
args count:  0
args:
args count:  0
Request: /test.svg
Arguments: 
Final list of key/value pairs:
WiFiWebServer::_handleRequest handle
WiFiWebServer::send1: len =  1946
content =  <svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="310" height="150">
<rect width="310" height="150" fill="rgb(250, 230, 210)" stroke-width="1" stroke="rgb(0, 0, 0)" />
<g stroke="black">
<line x1="10" y1="77" x2="20" y2="67" stroke-width="1" />
<line x1="20" y1="67" x2="30" y2="98" stroke-width="1" />
<line x1="30" y1="98" x2="40" y2="111" stroke-width="1" />
<line x1="40" y1="111" x2="50" y2="90" stroke-width="1" />
<line x1="50" y1="90" x2="60" y2="22" stroke-width="1" />
<line x1="60" y1="22" x2="70" y2="98" stroke-width="1" />
<line x1="70" y1="98" x2="80" y2="64" stroke-width="1" />
<line x1="80" y1="64" x2="90" y2="104" stroke-width="1" />
<line x1="90" y1="104" x2="100" y2="31" stroke-width="1" />
<line x1="100" y1="31" x2="110" y2="59" stroke-width="1" />
<line x1="110" y1="59" x2="120" y2="139" stroke-width="1" />
<line x1="120" y1="139" x2="130" y2="117" stroke-width="1" />
<line x1="130" y1="117" x2="140" y2="75" stroke-width="1" />
<line x1="140" y1="75" x2="150" y2="72" stroke-width="1" />
<line x1="150" y1="72" x2="160" y2="137" stroke-width="1" />
<line x1="160" y1="137" x2="170" y2="20" stroke-width="1" />
<line x1="170" y1="20" x2="180" y2="94" stroke-width="1" />
<line x1="180" y1="94" x2="190" y2="81" stroke-width="1" />
<line x1="190" y1="81" x2="200" y2="38" stroke-width="1" />
<line x1="200" y1="38" x2="210" y2="33" stroke-width="1" />
<line x1="210" y1="33" x2="220" y2="53" stroke-width="1" />
<line x1="220" y1="53" x2="230" y2="88" stroke-width="1" />
<line x1="230" y1="88" x2="240" y2="32" stroke-width="1" />
<line x1="240" y1="32" x2="250" y2="110" stroke-width="1" />
<line x1="250" y1="110" x2="260" y2="87" stroke-width="1" />
<line x1="260" y1="87" x2="270" y2="11" stroke-width="1" />
<line x1="270" y1="11" x2="280" y2="98" stroke-width="1" />
<line x1="280" y1="98" x2="290" y2="76" stroke-width="1" />
<line x1="290" y1="76" x2="300" y2="121" stroke-width="1" />
</g>
</svg>

WiFiWebServer::_prepareHeader sendHeader Conn close
WiFiWebServer::send1: write header =  HTTP/1.1 200 OK
Content-Type: image/svg+xml
Content-Length: 1946
Connection: close
```

---

#### 2. SimpleWebSocket on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library

The terminal output of **SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library** running [SimpleWebSocket example](examples/HTTPClient/SimpleWebSocket) to demonstrate newly-added WebSocket Client feature.

```cpp
Starting SimpleWebSocket on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
WiFiWebServer v1.10.1
Connecting to SSID: HueNet1
You're connected to the network, IP = 192.168.2.98
SSID: HueNet1, Signal strength (RSSI):-24 dBm
starting WebSocket client
Sending Hello 0
Sending Hello 1
Received a message:
0 => Hello from SimpleWebSocket on SAMD_NANO_33_IOT, millis = 6642
Sending Hello 2
Received a message:
1 => Hello from SimpleWebSocket on SAMD_NANO_33_IOT, millis = 11648
Sending Hello 3
Received a message:
2 => Hello from SimpleWebSocket on SAMD_NANO_33_IOT, millis = 16655
Sending Hello 4
Received a message:
3 => Hello from SimpleWebSocket on SAMD_NANO_33_IOT, millis = 21661
Sending Hello 5
Received a message:
4 => Hello from SimpleWebSocket on SAMD_NANO_33_IOT, millis = 26668
Sending Hello 6
Received a message:
5 => Hello from SimpleWebSocket on SAMD_NANO_33_IOT, millis = 31675
```

---

#### 3. SimpleHTTPExample on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library

The terminal output of **SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library** running [SimpleHTTPExample example](examples/HTTPClient/SimpleHTTPExample) to demonstrate newly-added HTTP Client feature.

```cpp
Starting SimpleHTTPExample on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
WiFiWebServer v1.10.1
Connecting to SSID: HueNet1
You're connected to the network, IP = 192.168.2.98
SSID: HueNet1, Signal strength (RSSI):-21 dBm
startedRequest ok
Got status code: 200
Content length is: 2263
Body returned follows:

           `:;;;,`                      .:;;:.           
        .;;;;;;;;;;;`                :;;;;;;;;;;:     TM 
      `;;;;;;;;;;;;;;;`            :;;;;;;;;;;;;;;;      
     :;;;;;;;;;;;;;;;;;;         `;;;;;;;;;;;;;;;;;;     
    ;;;;;;;;;;;;;;;;;;;;;       .;;;;;;;;;;;;;;;;;;;;    
   ;;;;;;;;:`   `;;;;;;;;;     ,;;;;;;;;.`   .;;;;;;;;   
  .;;;;;;,         :;;;;;;;   .;;;;;;;          ;;;;;;;  
  ;;;;;;             ;;;;;;;  ;;;;;;,            ;;;;;;. 
 ,;;;;;               ;;;;;;.;;;;;;`              ;;;;;; 
 ;;;;;.                ;;;;;;;;;;;`      ```       ;;;;;`
 ;;;;;                  ;;;;;;;;;,       ;;;       .;;;;;
`;;;;:                  `;;;;;;;;        ;;;        ;;;;;
,;;;;`    `,,,,,,,,      ;;;;;;;      .,,;;;,,,     ;;;;;
:;;;;`    .;;;;;;;;       ;;;;;,      :;;;;;;;;     ;;;;;
:;;;;`    .;;;;;;;;      `;;;;;;      :;;;;;;;;     ;;;;;
.;;;;.                   ;;;;;;;.        ;;;        ;;;;;
 ;;;;;                  ;;;;;;;;;        ;;;        ;;;;;
 ;;;;;                 .;;;;;;;;;;       ;;;       ;;;;;,
 ;;;;;;               `;;;;;;;;;;;;                ;;;;; 
 `;;;;;,             .;;;;;; ;;;;;;;              ;;;;;; 
  ;;;;;;:           :;;;;;;.  ;;;;;;;            ;;;;;;  
   ;;;;;;;`       .;;;;;;;,    ;;;;;;;;        ;;;;;;;:  
    ;;;;;;;;;:,:;;;;;;;;;:      ;;;;;;;;;;:,;;;;;;;;;;   
    `;;;;;;;;;;;;;;;;;;;.        ;;;;;;;;;;;;;;;;;;;;    
      ;;;;;;;;;;;;;;;;;           :;;;;;;;;;;;;;;;;:     
       ,;;;;;;;;;;;;;,              ;;;;;;;;;;;;;;       
         .;;;;;;;;;`                  ,;;;;;;;;:         
                                                         
                                                         
                                                         
                                                         
    ;;;   ;;;;;`  ;;;;:  .;;  ;; ,;;;;;, ;;. `;,  ;;;;   
    ;;;   ;;:;;;  ;;;;;; .;;  ;; ,;;;;;: ;;; `;, ;;;:;;  
   ,;:;   ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;,`;, ;;  ;;  
   ;; ;:  ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;;`;, ;;  ;;. 
   ;: ;;  ;;;;;:  ;;  ;; .;;  ;;   ,;,   ;;`;;;, ;;  ;;` 
  ,;;;;;  ;;`;;   ;;  ;; .;;  ;;   ,;,   ;; ;;;, ;;  ;;  
  ;;  ,;, ;; .;;  ;;;;;:  ;;;;;: ,;;;;;: ;;  ;;, ;;;;;;  
  ;;   ;; ;;  ;;` ;;;;.   `;;;:  ,;;;;;, ;;  ;;,  ;;;;   
  
```

---

#### 4. DweetPost on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library

The terminal output of **SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library** running [DweetPost example](examples/HTTPClient/DweetPost) to demonstrate newly-added HTTP Client feature.

```cpp
Starting DweetPost on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
WiFiWebServer v1.10.1
Connecting to SSID: HueNet1
You're connected to the network, IP = 192.168.2.98
SSID: HueNet1, Signal strength (RSSI):-22 dBm
making POST request
Status code: 200
Response: {"this":"succeeded","by":"dweeting","the":"dweet","with":{"thing":"Hello-from-SAMD_NANO_33_IOT","created":"2020-11-17T19:55:37.378Z","content":{"sensorValue":581},"transaction":"f968ee5f-35b5-4984-ac3d-34d93fdaddbe"}}
Wait ten seconds

making POST request
Status code: 200
Response: {"this":"succeeded","by":"dweeting","the":"dweet","with":{"thing":"Hello-from-SAMD_NANO_33_IOT","created":"2020-11-17T19:55:48.925Z","content":{"sensorValue":570},"transaction":"68ee52e0-22a3-4af2-96cd-aaa53587b314"}}
Wait ten seconds
```

---

#### 5. DweetGet on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library

The terminal output of **SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library** running [DweetGet example](examples/HTTPClient/DweetGet) to demonstrate newly-added HTTP Client feature.

```cpp
Starting DweetGet on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
WiFiWebServer v1.10.1
Connecting to SSID: HueNet1
You're connected to the network, IP = 192.168.2.98
SSID: HueNet1, Signal strength (RSSI):-25 dBm
Making GET request
Status code: 200
Response: {"this":"succeeded","by":"getting","the":"dweets","with":[{"thing":"Hello-from-SAMD_NANO_33_IOT","created":"2020-11-17T20:06:18.905Z","content":{"sensorValue":567}}]}
"sensorValue":567
Value string: 567
Actual value: 567
Wait ten seconds
Making GET request
Status code: 200
Response: {"this":"succeeded","by":"getting","the":"dweets","with":[{"thing":"Hello-from-SAMD_NANO_33_IOT","created":"2020-11-17T20:06:18.905Z","content":{"sensorValue":567}}]}
"sensorValue":567
Value string: 567
Actual value: 567
Wait ten seconds
```

---

#### 6. MQTTClient_Auth on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library

The terminal output of **SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library** running [MQTTClient_Auth example](examples/MQTTClient_Auth) to demonstrate newly-added MQTT Client feature.


```cpp
Starting MQTTClient_Auth on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
WiFiWebServer v1.10.1
Please upgrade the firmware
Connecting to WPA SSID: HueNet1
Connected! IP address: 192.168.2.98
Attempting MQTT connection to broker.emqx.io...connected
Message Send : MQTT_Pub => Hello from MQTTClient_Auth on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
Message arrived [MQTT_Pub] Hello from MQTTClient_Auth on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
Message Send : MQTT_Pub => Hello from MQTTClient_Auth on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
Message arrived [MQTT_Pub] Hello from MQTTClient_Auth on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
```

---

#### 7. MQTT_ThingStream on Arduino SAMD_NANO_33_IOT using WiFiNINA_Generic Library

The terminal output of **SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library** running [MQTT_ThingStream example](examples/MQTT_ThingStream) to demonstrate newly-added MQTT Client feature.


```cpp
Start MQTT_ThingStream on SAMD_NANO_33_IOT
Starting MQTTClient_Auth on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
WiFiWebServer v1.10.1
Please upgrade the firmware
Connecting to WPA SSID: HueNet1
Connected! IP address: 192.168.2.98
***************************************
STM32_Pub
***************************************
Attempting MQTT connection to broker.emqx.io
...connected
Published connection message successfully!
Subscribed to: STM32_Sub
MQTT Message Send : STM32_Pub => Hello from MQTT_ThingStream on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
MQTT Message receive [STM32_Pub] Hello from MQTT_ThingStream on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
MQTT Message Send : STM32_Pub => Hello from MQTT_ThingStream on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
MQTT Message receive [STM32_Pub] Hello from MQTT_ThingStream on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
MQTT Message Send : STM32_Pub => Hello from MQTT_ThingStream on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library
MQTT Message receive [STM32_Pub] Hello from MQTT_ThingStream on SAMD_NANO_33_IOT with WiFiNINA using WiFiNINA_Generic Library

```

---

#### 8. WebClientRepeating on RASPBERRY_PI_PICO with Custom WiFi using Custom WiFi Library

The terminal output of **RASPBERRY_PI_PICO with Custom WiFi (ESP8266-AT) using Custom WiFi (WiFiEspAT) Library** running [WebClientRepeating example](examples/WebClientRepeating) to demonstrate new RP2040-based board using [**Earle Philhower's arduino-pico** core](https://github.com/earlephilhower/arduino-pico)

```cpp
Starting WebClientRepeating on RASPBERRY_PI_PICO with Custom WiFi using Custom WiFi Library
WiFiWebServer v1.10.1
WiFi shield init done
Connecting to SSID: HueNet1
You're connected to the network, IP = 192.168.2.76
SSID: HueNet1, Signal strength (RSSI):-29 dBm
Connecting...
HTTP/1.1 200 OK
Server: nginx/1.4.2
Date: Wed, 26 May 2021 03:56:08 GMT
Content-Type: text/plain
Content-Length: 2263
Last-Modified: Wed, 02 Oct 2013 13:46:47 GMT
Connection: close
Vary: Accept-Encoding
ETag: "524c23c7-8d7"
Accept-Ranges: bytes


           `:;;;,`                      .:;;:.           
        .;;;;;;;;;;;`                :;;;;;;;;;;:     TM 
      `;;;;;;;;;;;;;;;`            :;;;;;;;;;;;;;;;      
     :;;;;;;;;;;;;;;;;;;         `;;;;;;;;;;;;;;;;;;     
    ;;;;;;;;;;;;;;;;;;;;;       .;;;;;;;;;;;;;;;;;;;;    
   ;;;;;;;;:`   `;;;;;;;;;     ,;;;;;;;;.`   .;;;;;;;;   
  .;;;;;;,         :;;;;;;;   .;;;;;;;          ;;;;;;;  
  ;;;;;;             ;;;;;;;  ;;;;;;,            ;;;;;;. 
 ,;;;;;               ;;;;;;.;;;;;;`              ;;;;;; 
 ;;;;;.                ;;;;;;;;;;;`      ```       ;;;;;`
 ;;;;;                  ;;;;;;;;;,       ;;;       .;;;;;
`;;;;:                  `;;;;;;;;        ;;;        ;;;;;
,;;;;`    `,,,,,,,,      ;;;;;;;      .,,;;;,,,     ;;;;;
:;;;;`    .;;;;;;;;       ;;;;;,      :;;;;;;;;     ;;;;;
:;;;;`    .;;;;;;;;      `;;;;;;      :;;;;;;;;     ;;;;;
.;;;;.                   ;;;;;;;.        ;;;        ;;;;;
 ;;;;;                  ;;;;;;;;;        ;;;        ;;;;;
 ;;;;;                 .;;;;;;;;;;       ;;;       ;;;;;,
 ;;;;;;               `;;;;;;;;;;;;                ;;;;; 
 `;;;;;,             .;;;;;; ;;;;;;;              ;;;;;; 
  ;;;;;;:           :;;;;;;.  ;;;;;;;            ;;;;;;  
   ;;;;;;;`       .;;;;;;;,    ;;;;;;;;        ;;;;;;;:  
    ;;;;;;;;;:,:;;;;;;;;;:      ;;;;;;;;;;:,;;;;;;;;;;   
    `;;;;;;;;;;;;;;;;;;;.        ;;;;;;;;;;;;;;;;;;;;    
      ;;;;;;;;;;;;;;;;;           :;;;;;;;;;;;;;;;;:     
       ,;;;;;;;;;;;;;,              ;;;;;;;;;;;;;;       
         .;;;;;;;;;`                  ,;;;;;;;;:         
                                                         
                                                         
                                                         
                                                         
    ;;;   ;;;;;`  ;;;;:  .;;  ;; ,;;;;;, ;;. `;,  ;;;;   
    ;;;   ;;:;;;  ;;;;;; .;;  ;; ,;;;;;: ;;; `;, ;;;:;;  
   ,;:;   ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;,`;, ;;  ;;  
   ;; ;:  ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;;`;, ;;  ;;. 
   ;: ;;  ;;;;;:  ;;  ;; .;;  ;;   ,;,   ;;`;;;, ;;  ;;` 
  ,;;;;;  ;;`;;   ;;  ;; .;;  ;;   ,;,   ;; ;;;, ;;  ;;  
  ;;  ,;, ;; .;;  ;;;;;:  ;;;;;: ,;;;;;: ;;  ;;, ;;;;;;  
  ;;   ;; ;;  ;;` ;;;;.   `;;;:  ,;;;;;, ;;  ;;,  ;;;;   
```

---

#### 9. AdvancedWebServer on Arduino Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library

The following are debug terminal output and screen shot when running example [**AdvancedWebServer**](examples/AdvancedWebServer) on **Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library**

<p align="center">
    <img src="https://github.com/khoih-prog/WiFiWebServer/raw/master/pics/AdvancedWebServer_NanoRP2040Connect.png">
</p>


```cpp
Starting AdvancedServer on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
WiFiWebServer v1.10.1
Connecting to WPA SSID: HueNet1
HTTP server started @ 192.168.2.130
H[WIFI] handleClient: New Client
[WIFI] method:  GET
[WIFI] url:  /
[WIFI] search:
[WIFI] headerName: Host
[WIFI] headerValue: 192.168.2.130
[WIFI] headerName: Connection
[WIFI] headerValue: keep-alive
[WIFI] headerName: Cache-Control
[WIFI] headerValue: max-age=0
[WIFI] headerName: Upgrade-Insecure-Requests
[WIFI] headerValue: 1
[WIFI] headerName: DNT
[WIFI] headerValue: 1
[WIFI] headerName: User-Agent
[WIFI] headerValue: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.77 Safari/537.36
[WIFI] headerName: Accept
[WIFI] headerValue: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9
[WIFI] headerName: Referer
[WIFI] headerValue: http://192.168.2.130/
[WIFI] headerName: Accept-Encoding
[WIFI] headerValue: gzip, deflate
[WIFI] headerName: Accept-Language
[WIFI] headerValue: en-GB,en-US;q=0.9,en;q=0.8,vi;q=0.7
[WIFI] args:
[WIFI] args count:  0
[WIFI] args:
[WIFI] args count:  0
[WIFI] Request: /
[WIFI] Arguments: 
[WIFI] Final list of key/value pairs:
[WIFI] _handleRequest handle
[WIFI] send1: len =  392
[WIFI] content =  <html><head><meta http-equiv='refresh' content='5'/><title>Nano RP2040 Connect</title><style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style></head><body><h1>Hello from Nano RP2040 Connect</h1><h3>running WiFiWebServer</h3><h3>on WiFiNINA using WiFiNINA_Generic Library</h3><p>Uptime: 0 d 00:00:12</p><img src="/test.svg" /></body></html>
[WIFI] _prepareHeader sendHeader Conn close
[WIFI] send1: write header =  HTTP/1.1 200 OK
Content-Type: text/html
Content-Length: 392
Connection: close
[WIFI] sendContent: Client.write content:  <html><head><meta http-equiv='refresh' content='5'/><title>Nano RP2040 Connect</title><style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style></head><body><h1>Hello from Nano RP2040 Connect</h1><h3>running WiFiWebServer</h3><h3>on WiFiNINA using WiFiNINA_Generic Library</h3><p>Uptime: 0 d 00:00:12</p><img src="/test.svg" /></body></html>
[WIFI] _handleRequest OK
[WIFI] handleClient: Client disconnected
[WIFI] handleClient: Don't keepCurrentClient
[WIFI] handleClient: Client disconnected
[WIFI] handleClient: New Client
[WIFI] method:  GET
[WIFI] url:  /test.svg
[WIFI] search:  
[WIFI] headerName: Host
[WIFI] headerValue: 192.168.2.130
[WIFI] headerName: Connection
[WIFI] headerValue: keep-alive
[WIFI] headerName: User-Agent
[WIFI] headerValue: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.77 Safari/537.36
[WIFI] headerName: DNT
[WIFI] headerValue: 1
[WIFI] headerName: Accept
[WIFI] headerValue: image/avif,image/webp,image/apng,image/svg+xml,image/*,*/*;q=0.8
[WIFI] headerName: Referer
[WIFI] headerValue: http://192.168.2.130/
[WIFI] headerName: Accept-Encoding
[WIFI] headerValue: gzip, deflate
[WIFI] headerName: Accept-Language
[WIFI] headerValue: en-GB,en-US;q=0.9,en;q=0.8,vi;q=0.7
[WIFI] args:  
[WIFI] args count:  0
[WIFI] args:  
[WIFI] args count:  0
[WIFI] Request: /test.svg
[WIFI] Arguments: 
[WIFI] Final list of key/value pairs:
[WIFI] _handleRequest handle
[WIFI] send1: len =  1954
[WIFI] content =  <svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="310" height="150">
<rect width="310" height="150" fill="rgb(250, 230, 210)" stroke-width="3" stroke="rgb(0, 0, 0)" />
<g stroke="blue">
<line x1="10" y1="77" x2="20" y2="92" stroke-width="2" />
<line x1="20" y1="92" x2="30" y2="123" stroke-width="2" />
<line x1="30" y1="123" x2="40" y2="136" stroke-width="2" />
<line x1="40" y1="136" x2="50" y2="115" stroke-width="2" />
<line x1="50" y1="115" x2="60" y2="47" stroke-width="2" />
<line x1="60" y1="47" x2="70" y2="123" stroke-width="2" />
<line x1="70" y1="123" x2="80" y2="89" stroke-width="2" />
<line x1="80" y1="89" x2="90" y2="129" stroke-width="2" />
<line x1="90" y1="129" x2="100" y2="56" stroke-width="2" />
<line x1="100" y1="56" x2="110" y2="83" stroke-width="2" />
<line x1="110" y1="83" x2="120" y2="33" stroke-width="2" />
<line x1="120" y1="33" x2="130" y2="11" stroke-width="2" />
<line x1="130" y1="11" x2="140" y2="99" stroke-width="2" />
<line x1="140" y1="99" x2="150" y2="96" stroke-width="2" />
<line x1="150" y1="96" x2="160" y2="31" stroke-width="2" />
<line x1="160" y1="31" x2="170" y2="44" stroke-width="2" />
<line x1="170" y1="44" x2="180" y2="118" stroke-width="2" />
<line x1="180" y1="118" x2="190" y2="105" stroke-width="2" />
<line x1="190" y1="105" x2="200" y2="62" stroke-width="2" />
<line x1="200" y1="62" x2="210" y2="57" stroke-width="2" />
<line x1="210" y1="57" x2="220" y2="77" stroke-width="2" />
<line x1="220" y1="77" x2="230" y2="111" stroke-width="2" />
<line x1="230" y1="111" x2="240" y2="55" stroke-width="2" />
<line x1="240" y1="55" x2="250" y2="133" stroke-width="2" />
<line x1="250" y1="133" x2="260" y2="110" stroke-width="2" />
<line x1="260" y1="110" x2="270" y2="34" stroke-width="2" />
<line x1="270" y1="34" x2="280" y2="121" stroke-width="2" />
<line x1="280" y1="121" x2="290" y2="99" stroke-width="2" />
<line x1="290" y1="99" x2="300" y2="14" stroke-width="2" />
</g>
</svg>
[WIFI] _prepareHeader sendHeader Conn close
[WIFI] send1: write header =  HTTP/1.1 200 OK
Content-Type: image/svg+xml
Content-Length: 1954
Connection: close

```

---

#### 10. SimpleHTTPExample on ESP32_DEV

The terminal output of **ESP32_DEV** running [SimpleHTTPExample example](examples/HTTPClient/SimpleHTTPExample) to demonstrate newly-added HTTP Client feature.

```cpp
Starting SimpleHTTPExample on ESP32_DEV with ESP WiFi using WiFi Library
WiFiWebServer v1.10.1
Connecting to SSID: HueNet1
You're connected to the network, IP = 192.168.2.80
SSID: HueNet1, Signal strength (RSSI):-23 dBm
startedRequest ok
Got status code: 200
Content length is: 2263
Body returned follows:

           `:;;;,`                      .:;;:.           
        .;;;;;;;;;;;`                :;;;;;;;;;;:     TM 
      `;;;;;;;;;;;;;;;`            :;;;;;;;;;;;;;;;      
     :;;;;;;;;;;;;;;;;;;         `;;;;;;;;;;;;;;;;;;     
    ;;;;;;;;;;;;;;;;;;;;;       .;;;;;;;;;;;;;;;;;;;;    
   ;;;;;;;;:`   `;;;;;;;;;     ,;;;;;;;;.`   .;;;;;;;;   
  .;;;;;;,         :;;;;;;;   .;;;;;;;          ;;;;;;;  
  ;;;;;;             ;;;;;;;  ;;;;;;,            ;;;;;;. 
 ,;;;;;               ;;;;;;.;;;;;;`              ;;;;;; 
 ;;;;;.                ;;;;;;;;;;;`      ```       ;;;;;`
 ;;;;;                  ;;;;;;;;;,       ;;;       .;;;;;
`;;;;:                  `;;;;;;;;        ;;;        ;;;;;
,;;;;`    `,,,,,,,,      ;;;;;;;      .,,;;;,,,     ;;;;;
:;;;;`    .;;;;;;;;       ;;;;;,      :;;;;;;;;     ;;;;;
:;;;;`    .;;;;;;;;      `;;;;;;      :;;;;;;;;     ;;;;;
.;;;;.                   ;;;;;;;.        ;;;        ;;;;;
 ;;;;;                  ;;;;;;;;;        ;;;        ;;;;;
 ;;;;;                 .;;;;;;;;;;       ;;;       ;;;;;,
 ;;;;;;               `;;;;;;;;;;;;                ;;;;; 
 `;;;;;,             .;;;;;; ;;;;;;;              ;;;;;; 
  ;;;;;;:           :;;;;;;.  ;;;;;;;            ;;;;;;  
   ;;;;;;;`       .;;;;;;;,    ;;;;;;;;        ;;;;;;;:  
    ;;;;;;;;;:,:;;;;;;;;;:      ;;;;;;;;;;:,;;;;;;;;;;   
    `;;;;;;;;;;;;;;;;;;;.        ;;;;;;;;;;;;;;;;;;;;    
      ;;;;;;;;;;;;;;;;;           :;;;;;;;;;;;;;;;;:     
       ,;;;;;;;;;;;;;,              ;;;;;;;;;;;;;;       
         .;;;;;;;;;`                  ,;;;;;;;;:         
                                                         
                                                         
                                                         
                                                         
    ;;;   ;;;;;`  ;;;;:  .;;  ;; ,;;;;;, ;;. `;,  ;;;;   
    ;;;   ;;:;;;  ;;;;;; .;;  ;; ,;;;;;: ;;; `;, ;;;:;;  
   ,;:;   ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;,`;, ;;  ;;  
   ;; ;:  ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;;`;, ;;  ;;. 
   ;: ;;  ;;;;;:  ;;  ;; .;;  ;;   ,;,   ;;`;;;, ;;  ;;` 
  ,;;;;;  ;;`;;   ;;  ;; .;;  ;;   ,;,   ;; ;;;, ;;  ;;  
  ;;  ,;, ;; .;;  ;;;;;:  ;;;;;: ,;;;;;: ;;  ;;, ;;;;;;  
  ;;   ;; ;;  ;;` ;;;;.   `;;;:  ,;;;;;, ;;  ;;,  ;;;;   
Got status code: 200
Content length is: 2263  
```

---


#### 11. AdvancedWebServer on PORTENTA_H7_M7 with Portenta_H7 WiFi

The following are debug terminal output and screen shot when running example [**AdvancedWebServer**](examples/AdvancedWebServer) on **PORTENTA_H7_M7 with Portenta_H7 WiFi**

<p align="center">
    <img src="https://github.com/khoih-prog/WiFiWebServer/raw/master/pics/AdvancedWebServer_Portenta_H7.png">
</p>


```cpp
Starting AdvancedServer on PORTENTA_H7_M7 with Portenta_H7 WiFi
WiFiWebServer v1.10.1
Connecting to WPA SSID: HueNet1
HTTP server started @ 192.168.2.138
H[WIFI] String Len = 0, extend to 2048
HHHHHHHHH HHHHHHH
```

---

#### 12. MQTTClient_Auth on PORTENTA_H7_M7 with Portenta_H7 WiFi

The terminal output of **PORTENTA_H7_M7 with Portenta_H7 WiFi** running [MQTTClient_Auth example](examples/MQTTClient_Auth) to demonstrate MQTT Client feature.


```cpp
Starting MQTTClient_Auth on PORTENTA_H7_M7 with Portenta_H7 WiFi
WiFiWebServer v1.10.1
Connecting to SSID: HueNet1
Connected! IP address: 192.168.2.130
Attempting MQTT connection to broker.emqx.io...connected
Message Send : MQTT_Pub => Hello from MQTTClient_Auth on PORTENTA_H7_M7 with Portenta_H7 WiFi
Message arrived [MQTT_Pub] Hello from MQTTClient_Auth on PORTENTA_H7_M7 with Portenta_H7 WiFi
Message Send : MQTT_Pub => Hello from MQTTClient_Auth on PORTENTA_H7_M7 with Portenta_H7 WiFi
Message arrived [MQTT_Pub] Hello from MQTTClient_Auth on PORTENTA_H7_M7 with Portenta_H7 WiFi
```

---


#### 13. WebClientRepeating on PORTENTA_H7_M7 with Portenta_H7 WiFi

The terminal output of **PORTENTA_H7_M7 with Portenta_H7 WiFi** running [WebClientRepeating example](examples/WebClientRepeating).


```cpp
Starting WebClientRepeating on PORTENTA_H7_M7 with Portenta_H7 WiFi
WiFiWebServer v1.10.1
Connecting to SSID: HueNet1
You're connected to the network, IP = 192.168.2.130
SSID: HueNet1, Signal strength (RSSI):-33 dBm
Connecting...
HTTP/1.1 200 OK
Server: nginx/1.4.2
Date: Wed, 08 Sep 2021 01:29:41 GMT
Content-Type: text/plain
Content-Length: 2263
Last-Modified: Wed, 02 Oct 2013 13:46:47 GMT
Connection: close
Vary: Accept-Encoding
ETag: "524c23c7-8d7"
Accept-Ranges: bytes


           `:;;;,`                      .:;;:.           
        .;;;;;;;;;;;`                :;;;;;;;;;;:     TM 
      `;;;;;;;;;;;;;;;`            :;;;;;;;;;;;;;;;      
     :;;;;;;;;;;;;;;;;;;         `;;;;;;;;;;;;;;;;;;     
    ;;;;;;;;;;;;;;;;;;;;;       .;;;;;;;;;;;;;;;;;;;;    
   ;;;;;;;;:`   `;;;;;;;;;     ,;;;;;;;;.`   .;;;;;;;;   
  .;;;;;;,         :;;;;;;;   .;;;;;;;          ;;;;;;;  
  ;;;;;;             ;;;;;;;  ;;;;;;,            ;;;;;;. 
 ,;;;;;               ;;;;;;.;;;;;;`              ;;;;;; 
 ;;;;;.                ;;;;;;;;;;;`      ```       ;;;;;`
 ;;;;;                  ;;;;;;;;;,       ;;;       .;;;;;
`;;;;:                  `;;;;;;;;        ;;;        ;;;;;
,;;;;`    `,,,,,,,,      ;;;;;;;      .,,;;;,,,     ;;;;;
:;;;;`    .;;;;;;;;       ;;;;;,      :;;;;;;;;     ;;;;;
:;;;;`    .;;;;;;;;      `;;;;;;      :;;;;;;;;     ;;;;;
.;;;;.                   ;;;;;;;.        ;;;        ;;;;;
 ;;;;;                  ;;;;;;;;;        ;;;        ;;;;;
 ;;;;;                 .;;;;;;;;;;       ;;;       ;;;;;,
 ;;;;;;               `;;;;;;;;;;;;                ;;;;; 
 `;;;;;,             .;;;;;; ;;;;;;;              ;;;;;; 
  ;;;;;;:           :;;;;;;.  ;;;;;;;            ;;;;;;  
   ;;;;;;;`       .;;;;;;;,    ;;;;;;;;        ;;;;;;;:  
    ;;;;;;;;;:,:;;;;;;;;;:      ;;;;;;;;;;:,;;;;;;;;;;   
    `;;;;;;;;;;;;;;;;;;;.        ;;;;;;;;;;;;;;;;;;;;    
      ;;;;;;;;;;;;;;;;;           :;;;;;;;;;;;;;;;;:     
       ,;;;;;;;;;;;;;,              ;;;;;;;;;;;;;;       
         .;;;;;;;;;`                  ,;;;;;;;;:         
                                                         
                                                         
                                                         
                                                         
    ;;;   ;;;;;`  ;;;;:  .;;  ;; ,;;;;;, ;;. `;,  ;;;;   
    ;;;   ;;:;;;  ;;;;;; .;;  ;; ,;;;;;: ;;; `;, ;;;:;;  
   ,;:;   ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;,`;, ;;  ;;  
   ;; ;:  ;;  ;;  ;;  ;; .;;  ;;   ,;,   ;;;;`;, ;;  ;;. 
   ;: ;;  ;;;;;:  ;;  ;; .;;  ;;   ,;,   ;;`;;;, ;;  ;;` 
  ,;;;;;  ;;`;;   ;;  ;; .;;  ;;   ,;,   ;; ;;;, ;;  ;;  
  ;;  ,;, ;; .;;  ;;;;;:  ;;;;;: ,;;;;;: ;;  ;;, ;;;;;;  
  ;;   ;; ;;  ;;` ;;;;.   `;;;:  ,;;;;;, ;;  ;;,  ;;;;   
```

---


#### 14. AdvancedWebServer on ESP32C3_DEV with ESP WiFi

The following are debug terminal output and screen shot when running example [**AdvancedWebServer**](examples/AdvancedWebServer) on **PORTENTA_H7_M7 with Portenta_H7 WiFi**

<p align="center">
    <img src="https://github.com/khoih-prog/WiFiWebServer/raw/master/pics/AdvancedWebServer_ESP32_C3.png">
</p>


```cpp
Starting AdvancedWebServer on ESP32C3_DEV with ESP WiFi using WiFi Library
WiFiWebServer v1.10.1
Connecting to WPA SSID: HueNet1
HTTP server started @ 192.168.2.86
HH
```

---

#### 15. AdvancedWebServer on ESP32S3_DEV with ESP WiFi

The following are debug terminal output and screen shot when running example [**AdvancedWebServer**](examples/AdvancedWebServer) on **PORTENTA_H7_M7 with Portenta_H7 WiFi**

<p align="center">
    <img src="https://github.com/khoih-prog/WiFiWebServer/raw/master/pics/AdvancedWebServer_ESP32_S3.png">
</p>


```cpp
Starting AdvancedWebServer on ESP32S3_DEV with ESP WiFi using WiFi Library
WiFiWebServer v1.10.1
Connecting to WPA SSID: HueNet1
HTTP server started @ 192.168.2.86
HH
```

---

#### 16. AdvancedWebServer_WiFiMulti on Nano RP2040 Connect with WiFiNINA

The following are debug terminal output and screen shot when running example [**AdvancedWebServer_WiFiMulti**](examples/WiFiMulti/AdvancedWebServer_WiFiMulti) on **Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library**

<p align="center">
    <img src="https://github.com/khoih-prog/WiFiWebServer/raw/master/pics/AdvancedWebServer_WiFiMulti_NanoRP2040Connect.png">
</p>


```cpp
Starting AdvancedWebServer_WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
WiFiMulti_Generic v1.2.2
WiFiWebServer v1.10.1
Connecting WiFi...
WiFi connected, IP address: 192.168.2.113
You're connected to the network, IP = 192.168.2.113
SSID: HueNet1, Signal strength (RSSI):-20 dBm
HTTP server started @ 192.168.2.113
H
```

---

#### 17. MQTTClient_Auth_WiFiMulti on Nano RP2040 Connect with WiFiNINA

The following are debug terminal output and screen shot when running example [**MQTTClient_Auth_WiFiMulti**](examples/WiFiMulti/MQTTClient_Auth_WiFiMulti) on **Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library**

```cpp
Starting MQTTClient_Auth_WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
WiFiMulti_Generic v1.2.2
WiFiWebServer v1.10.1
Connecting WiFi...
WiFi connected, IP address: 192.168.2.113
You're connected to the network, IP = 192.168.2.113
SSID: HueNet1, Signal strength (RSSI):-18 dBm
Attempting MQTT connection to broker.emqx.io...connected
Published connection message successfully!
Subscribed to: Nano RP2040 ConnectSub
H
MQTT Message Send : Nano RP2040 ConnectPub => Hello from MQTTClient_Auth__WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
Message arrived [Nano RP2040 ConnectPub] Hello from MQTTClient_Auth__WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
HH
MQTT Message Send : Nano RP2040 ConnectPub => Hello from MQTTClient_Auth__WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
H
Message arrived [Nano RP2040 ConnectPub] Hello from MQTTClient_Auth__WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
HH
MQTT Message Send : Nano RP2040 ConnectPub => Hello from MQTTClient_Auth__WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
H
Message arrived [Nano RP2040 ConnectPub] Hello from MQTTClient_Auth__WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
```

---

#### 18. WiFiUdpNTPClient_WiFiMulti on Nano RP2040 Connect with WiFiNINA

The following are debug terminal output and screen shot when running example [**WiFiUdpNTPClient_WiFiMulti**](examples/WiFiMulti/WiFiUdpNTPClient_WiFiMulti) on **Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library**

```cpp
Starting WiFiUdpNTPClient_WiFiMulti on Nano RP2040 Connect with WiFiNINA using WiFiNINA_Generic Library
WiFiMulti_Generic v1.2.2
WiFiWebServer v1.10.1
Connecting WiFi...
WiFi connected, IP address: 192.168.2.113
You're connected to the network, IP = 192.168.2.113
SSID: HueNet1, Signal strength (RSSI):-17 dBm
Starting connection to server...
Listening on port 2390
HHH
Packet received
Seconds since Jan 1 1900 = 3869665072
Unix time = 1660676272
The UTC time is 18:57:52
HH
Packet received
Seconds since Jan 1 1900 = 3869665132
Unix time = 1660676332
The UTC time is 18:58:52
HH
```

---

#### 19. AdvancedWebServer_WiFiMulti on RASPBERRY_PI_PICO_W

The following are debug terminal output and screen shot when running example [**AdvancedWebServer_WiFiMulti**](examples/WiFiMulti/AdvancedWebServer_WiFiMulti) on **RASPBERRY_PI_PICO_W with CYW43439 WiFi**

<p align="center">
    <img src="https://github.com/khoih-prog/WiFiWebServer/raw/master/pics/AdvancedWebServer_WiFiMulti_RP2040W.png">
</p>


```cpp
Starting AdvancedWebServer_WiFiMulti on RASPBERRY_PI_PICO_W with RP2040W CYW43439 WiFi
WiFiMulti_Generic v1.2.2
WiFiWebServer v1.10.1
Connecting WiFi...

WiFi connected, IP address: 192.168.2.180
You're connected to the network, IP = 192.168.2.180
SSID: HueNet1, Signal strength (RSSI):0 dBm
HTTP server started @ 192.168.2.180
HH
```

---
---

### Debug

Debug is enabled by default on Serial. Debug Level from 0 to 4. To disable, change the _WIFI_LOGLEVEL_ and _WIFININA_LOGLEVEL_ to 0

```cpp
// Use this to output debug msgs to Serial
#define DEBUG_WIFI_WEBSERVER_PORT   Serial

// Debug Level from 0 to 4
#define _WIFI_LOGLEVEL_             1
#define _WIFININA_LOGLEVEL_         1
```

---

## Troubleshooting

If you get compilation errors, more often than not, you may need to install a newer version of the board's core, applying Libraries' Patches, Packages' Patches or this library latest version.

---
---

### Issues

Submit issues to: [WiFiWebServer issues](https://github.com/khoih-prog/WiFiWebServer/issues)

---

### TO DO

1. Bug Searching and Killing
2. Add SSL/TLS Client and Server support
3. Support more types of boards using WiFiNINA and other WiFi shields.
4. Add support to megaAVR boards using [MegaCoreX core](https://github.com/MCUdude/MegaCoreX)


### DONE

 1. Add support to Arduino SAMD21, Adafruit SAMD21/SAMD51, Seeeduino SAMD21/SAMD51.
 2. Add support to nRF52.
 3. Add support to SAM DUE.
 4. Add support to all STM32F/L/H/G/WB/MP1.
 5. Add support to WiFiNINA using [**WiFiNINA_Generic library**](https://github.com/khoih-prog/WiFiNINA_Generic). 
 6. Add support to [**ESP_AT_Lib library**](https://github.com/khoih-prog/ESP_AT_Lib).
 7. Add support to [`WiFi101 library`](https://www.arduino.cc/en/Reference/WiFi101).
 8. Add support to [`WiFiEspAT library`](https://github.com/jandrassy/WiFiEspAT).
 9. Add support to PROGMEM-related commands, such as sendContent_P() and send_P()
10. Add **High-level HTTP (GET, POST, PUT, PATCH, DELETE) and WebSocket Client**
11. Add support to **Arduino Nano RP2040 Connect** using [**Arduino mbed OS for Nano boards**](https://github.com/arduino/ArduinoCore-mbed).
12. Add support to RP2040-based boards, such as **RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040**, using [**Earle Philhower's arduino-pico** core](https://github.com/earlephilhower/arduino-pico).
13. Add support to RP2040-based boards, such as **RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040**, using [**Arduino-mbed mbed_rp2040** core](https://github.com/arduino/ArduinoCore-mbed).
14. Add support to **Portenta_H7 boards**, using [**Arduino-mbed mbed_portenta** core](https://github.com/arduino/ArduinoCore-mbed).
15. Reduce usage of Arduino String with std::string
16. Optimize library code and examples by using **reference-passing instead of value-passing**.
17. Add support to new **ESP32-S3** and **ESP32_C3**
18. Add support to megaAVR boards (UNO_WIFI_REV2, NANO_EVERY) using [Arduino megaAVR core](https://github.com/arduino/ArduinoCore-megaavr)
19. Rewrite library and add example [multiFileProject](examples/multiFileProject) to demo for multiple-file project to fix `multiple-definitions` linker error
20. Add [WiFiMulti_Generic](https://github.com/khoih-prog/WiFiMulti_Generic) library support
21. Add many WiFiMulti-related examples in [WiFiMulti](https://github.com/khoih-prog/WiFiWebServer/tree/master/examples/WiFiMulti)
22. Add support to RASPBERRY_PI_PICO_W using CYW4343 WiFi with [**Earle Philhower's arduino-pico core** v2.4.0+](https://github.com/earlephilhower/arduino-pico)
23. Better workaround for RP2040W `WiFi.status()` bug using `ping()` to local gateway
24. Add new features, such as `CORS`
25. Use `allman astyle` and add `utils`
26. Using new [`WiFi101_Generic library`](https://github.com/khoih-prog/WiFi101_Generic) for sending larger data

---
---

## Contributions and Thanks

1. Based on and modified from [**Ivan Grokhotkov's ESP8266WebServer**](https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer)
2. Thanks to good work of [Miguel Alexandre Wisintainer](https://github.com/tcpipchip) for initiating, inspriring, working with, developing, debugging and testing. Without that, support to nRF52, especially **U-Blox B302 running as nRF52840 and U-Blox B112 running as nRF52832**, has never been started and finished. See [u-blox nina b](https://github.com/khoih-prog/WiFiNINA_Generic/issues/1)
3. [Adrian McEwen](https://github.com/amcewen) for [HttpClient Library](https://github.com/amcewen/HttpClient) on which the [ArduinoHttpClient Library](https://github.com/arduino-libraries/ArduinoHttpClient) and this [EthernetWebServer library](https://github.com/khoih-prog/EthernetWebServer) are relied.
4. [RQnet](https://github.com/RQnet) to report issue [Decoding Error. two times called urlDecode in Parsing-impl.h. #17](https://github.com/khoih-prog/WiFiWebServer/issues/17) leading to version v1.6.3 to fix the decoding error bug when using special `&` in data fields.

<table>
  <tr>
    <td align="center"><a href="https://github.com/igrr"><img src="https://github.com/igrr.png" width="100px;" alt="igrr"/><br /><sub><b>⭐️⭐️ Ivan Grokhotkov</b></sub></a><br /></td>
    <td align="center"><a href="https://github.com/amcewen"><img src="https://github.com/amcewen.png" width="100px;" alt="amcewen"/><br /><sub><b>⭐️ Adrian McEwen</b></sub></a><br /></td>
    <td align="center"><a href="https://github.com/tcpipchip"><img src="https://github.com/tcpipchip.png" width="100px;" alt="tcpipchip"/><br /><sub><b>⭐️ Miguel Wisintainer</b></sub></a><br /></td>
    <td align="center"><a href="https://github.com/RQnet"><img src="https://github.com/RQnet.png" width="100px;" alt="RQnet"/><br /><sub><b>RQnet</b></sub></a><br /></td>
  </tr> 
</table>

---

## Contributing

If you want to contribute to this project:
- Report bugs and errors
- Ask for enhancements
- Create issues and pull requests
- Tell other people about this library

---

### License

- The library is licensed under [MIT](https://github.com/khoih-prog/WiFiWebServer/blob/master/LICENSE)

---

## Copyright

Copyright (c) 2020- Khoi Hoang


