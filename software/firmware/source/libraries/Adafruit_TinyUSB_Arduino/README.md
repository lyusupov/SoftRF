# Adafruit TinyUSB Library for Arduino

[![Build Status](https://github.com/adafruit/Adafruit_TinyUSB_Arduino/workflows/Build/badge.svg)](https://github.com/adafruit/Adafruit_TinyUSB_Arduino/actions) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)

This library works with platform that includes [TinyUSB stack](https://github.com/hathach/tinyusb), typically you will see **Adafruit_TinyUSB_Core** inside core folder. Supported platform are:

- [Adafruit_nRF52_Arduino](https://github.com/adafruit/Adafruit_nRF52_Arduino)
- [Adafruit ArduinoCore-samd](https://github.com/adafruit/ArduinoCore-samd) **TinyUSB** must be selected in menu `Tools->USB Stack`

In addition to CDC that provided by the platform core, this library provide platform-independent for

- Human Interface Device (HID): Generic (In & Out), Keyboard, Mouse, Gamepad etc ...
- Mass Storage Class (MSC): with multiple LUNs
- Musical Instrument Digital Interface (MIDI)
- WebUSB with vendor specific class

More document to write ... 
