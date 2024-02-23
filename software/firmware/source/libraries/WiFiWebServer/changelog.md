## WiFiWebServer Changelog

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

* [Changelog](#changelog)
  * [Releases v1.10.1](#releases-v1101)
  * [Releases v1.10.0](#releases-v1100)
  * [Releases v1.9.5](#releases-v195)
  * [Releases v1.9.4](#releases-v194)
  * [Releases v1.9.3](#releases-v193)
  * [Releases v1.9.2](#releases-v192)
  * [Releases v1.9.1](#releases-v191)
  * [Releases v1.9.0](#releases-v190)
  * [Releases v1.8.0](#releases-v180)
  * [Releases v1.7.0](#releases-v170)
  * [Releases v1.6.3](#releases-v163)
  * [Releases v1.6.2](#releases-v162)
  * [Releases v1.6.1](#releases-v161)
  * [Releases v1.6.0](#releases-v160)
  * [Releases v1.5.4](#releases-v154)
  * [Releases v1.5.3](#releases-v153)
  * [Releases v1.5.2](#releases-v152)
  * [Releases v1.5.1](#releases-v151)
  * [Major Releases v1.5.0](#major-releases-v150)
  * [Releases v1.4.2](#releases-v142)
  * [Releases v1.4.1](#releases-v141)
  * [Major Releases v1.4.0](#major-releases-v140)
  * [Releases v1.3.1](#releases-v131)
  * [Releases v1.3.0](#releases-v130)
  * [Major Releases v1.2.0](#major-releases-v120)
  * [Releases v1.1.1](#releases-v111)
  * [Major Releases v1.1.0](#major-releases-v110)
  * [Releases v1.0.7](#releases-v107)
  * [Releases v1.0.6](#releases-v106)
  * [Releases v1.0.5](#releases-v105)
  * [Releases v1.0.4](#releases-v104)
  * [Releases v1.0.3](#releases-v103)
  * [Releases v1.0.2](#releases-v102)
  * [Releases v1.0.1](#releases-v101)
  * [Initial Releases v1.0.0](#initial-releases-v100)

---
---

## Changelog

### Releases v1.10.1

1. Using new [`WiFi101_Generic library`](https://github.com/khoih-prog/WiFi101_Generic) for sending larger data
2. Update `Packages' Patches`

### Releases v1.10.0

1. Add new features, such as `CORS`, etc.
2. Update code and examples
3. Use `allman astyle` and add `utils`
4. Update `Packages' Patches`

### Releases v1.9.5

1. Restore support to Teensy, etc. 
2. Fix bug in examples

### Releases v1.9.4

1. Restore support to ESP32 and ESP8266. Check [Problem using ESP8266 nodeMCU 1.0 #20](https://github.com/khoih-prog/WiFiWebServer/issues/20)

### Releases v1.9.3

1. Better workaround for RP2040W `WiFi.status()` bug using `ping()` to local gateway
2. Update WiFiMulti-related examples

### Releases v1.9.2

1. Workaround for RP2040W WiFi.status() bug
2. Update WiFiMulti-related examples

### Releases v1.9.1

1. Add WiFiMulti support to RASPBERRY_PI_PICO_W using CYW43439 WiFi

### Releases v1.9.0

1. Add support to RASPBERRY_PI_PICO_W using CYW43439 WiFi
2. Update `Packages' Patches`

### Releases v1.8.0

1. Add [WiFiMulti_Generic](https://github.com/khoih-prog/WiFiMulti_Generic) library support
2. Add many WiFiMulti-related examples in [WiFiMulti](https://github.com/khoih-prog/WiFiWebServer/tree/master/examples/WiFiMulti)
3. Update `Packages' Patches`

### Releases v1.7.0

1. Fix issue with Portenta_H7 core v2.7.2+. Check [[Portenta_H7] WiFi WebServer extremely slow from core v2.7.2 - v3.0.1 #441](https://github.com/arduino/ArduinoCore-mbed/issues/441)
2. Rewrite to avoid `multiple-definitions` linker error for multiple-file project
3. Add example [multiFileProject](examples/multiFileProject) to demo how to avoid `multiple-definitions` linker error for multiple-file project
4. Update `Packages' Patches`

### Releases v1.6.3

1. Fix decoding error bug when using special `&` in data fields. Check [Decoding Error. two times called urlDecode in Parsing-impl.h. #17](https://github.com/khoih-prog/WiFiWebServer/issues/17)
2. Update `Packages' Patches`


### Releases v1.6.2

1. Add support to megaAVR boards (UNO_WIFI_REV2, NANO_EVERY) using [Arduino megaAVR core](https://github.com/arduino/ArduinoCore-megaavr)
2. Update `Packages' Patches`

### Releases v1.6.1

1. Fix issue in v1.6.0

### Releases v1.6.0

1. Add support to new ESP32-S3 and ESP32_C3
2. Update `Packages' Patches`

### Releases v1.5.4

1. Fix libb64 `fallthrough` compile warning
2. Fix bug not supporting ESP32/ESP8266 boards.
3. Fix bug for WiFi other than WiFiNINA

### Releases v1.5.3

1. Fix authenticate issue caused by libb64

### Releases v1.5.2

1. Fix wrong http status header bug. Check [fix for wrong http status header #42](https://github.com/khoih-prog/EthernetWebServer/pull/42)

### Releases v1.5.1

1. Fix bug related to String in library and examples

### Major Releases v1.5.0

1. Reduce usage of Arduino String with std::string
2. Optimize library code and examples by using **reference-passing instead of value-passing**.
3. Update `Packages' Patches`
4. Add more ESP32/ESP8266 supporting code

### Releases v1.4.2

1. Update `platform.ini` and `library.json` to use original `khoih-prog` instead of `khoih.prog` after PIO fix
2. Update `Packages' Patches`

### Releases v1.4.1

1. Change option for PIO `lib_compat_mode` from default `soft` to `strict` to minimize compile error in crosss-platform
2. Update `Packages' Patches` for many boards

### Major Releases v1.4.0

1. Add support to **Portenta_H7** using [**Arduino mbed_portenta core**](https://github.com/arduino/ArduinoCore-mbed).
2. Update `Packages' Patches` for **Portenta_H7**

### Releases v1.3.1

1. Add support to ESP32/ESP8266 to use in some rare use-cases
2. Update `Packages' Patches`
3. Split `changelog.md` from `README.md`

### Releases v1.3.0

1. Add support to Adafruit nRF52 core v0.22.0+
2. Add support to Raytac MDBT50Q_RX Dongle
3. Update `Packages' Patches`

### Major Releases v1.2.0

1. Add support to **Arduino Nano RP2040 Connect** using [**Arduino mbed OS for Nano boards**](https://github.com/arduino/ArduinoCore-mbed).
2. Add support to RP2040-based boards, such as **RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040**, using [**Earle Philhower's arduino-pico** v1.5.1+ core](https://github.com/earlephilhower/arduino-pico).
3. Add support to RP2040-based boards, such as **RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040**, using [**Arduino-mbed RP2040** v2.1.0+ core](https://github.com/arduino/ArduinoCore-mbed).
4. Add to examples the support to ESP32-AT/ESP8266-AT WiFi, using [`WiFiEspAT library`](https://github.com/jandrassy/WiFiEspAT)
5. Fix bugs
6. Update `Packages' Patches`

### Releases v1.1.1

1. Clean-up all compiler warnings possible.
2. Add MQTT examples
3. Add Version String 

### Major Releases v1.1.0

1. Add high-level **HTTP and WebSockets Client** by merging [ArduinoHttpClient Library](https://github.com/arduino-libraries/ArduinoHttpClient)
2. Add many more examples for HTTP and WebSockets Client.

### Releases v1.0.7

1. Add support to **PROGMEM-related commands, such as sendContent_P() and send_P()**
2. Update Platform.ini to support **PlatformIO 5.x owner-based dependency declaration.**
3. Clean up code.
4. Update examples.

#### Releases v1.0.6

1. Add support to all **STM32F/L/H/G/WB/MP1** boards.
2. Add support to **Seeeduino SAMD21/SAMD51** boards.
3. Restructure examples. Clean-up code.

#### Releases v1.0.5

1. Fix bug not closing client and releasing socket exposed in NINA Firmware v1.4.0.
2. Enhance examples.

#### Releases v1.0.4

1. Add support to boards using **WiFi101 built-in or shield**. For example MKR1000, Teensy, Mega, etc..
2. Support any future custom WiFi library that meets the no-compiling-error requirements.

#### Releases v1.0.3

1. Add support to **nRF52** boards, such as **AdaFruit Feather nRF52832, nRF52840 Express, BlueFruit Sense, Itsy-Bitsy nRF52840 Express, Metro nRF52840 Express, NINA_B302_ublox, etc.**

#### Releases v1.0.2

1. Add support to **SAM51 (Itsy-Bitsy M4, Metro M4, Grand Central M4, Feather M4 Express, etc.) and SAM DUE**.

#### Releases v1.0.1

1. Use new [`WiFiNINA_Generic library`](https://github.com/khoih-prog/WiFiNINA_Generic) to provide support to many more boards running WiFiNINA.

The original WiFiNINA library only supports **Nano-33 IoT**, Arduino MKR WiFi 1010, Arduino MKR VIDOR 4000 and Arduino UNO WiFi Rev.2.

#### Initial Releases v1.0.0

This is simple yet complete WebServer library for `AVR Mega, Teensy, SAMD21, STM32, etc.` boards running WiFi modules/shields (WiFiNINA U-Blox W101, W102, etc.). **The functions are similar and compatible to ESP8266/ESP32 WebServer libraries** to make life much easier to port sketches from ESP8266/ESP32.


