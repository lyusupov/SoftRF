# Adafruit SPA06_003 [![Build Status](https://github.com/adafruit/Adafruit_SPA06_003/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_SPA06_003/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_SPA06_003/html/index.html)

<a href="https://www.adafruit.com/product/6420"><img src="assets/board.jpg" width="500px"><br/>
<i>Adafruit SPA06_003 Breakout</i></a>

This is the Adafruit SPA06_003 Digital Pressure Sensor library for Arduino.

Tested and works great with the Adafruit SPA06_003 Breakout Board.

## About the SPA06_003

The SPA06_003 is a miniaturized Digital Barometric Air Pressure Sensor with high accuracy and low current consumption. Key features:

* Pressure range: 300 ... 1100hPa (+9000m ... -500m relating to sea level)
* Temperature Range: -40...+85°C  
* Supply voltage: 1.7 ... 3.6V (VDD), 1.08... 3.6V (VDDIO)
* Small footprint: 2.0mm x 2.5mm; Super-flat: 0.95mm
* Relative accuracy: typ.±0.03hPa, equiv. to ±0.25 m
* Absolute accuracy: typ. ±0.3hPa (300 ... 1100hPa)
* Temperature accuracy: typ. ± 1°C
* Measurement time: 3.6ms for low precision mode
* Average current consumption: 1.7 µA for pressure measurement, 1.5 µA for temperature measurement at 1Hz sampling rate
* I2C and SPI interface
* FIFO: Stores latest 32 pressure or temperature measurements
* Embedded 24-bit ADC

## Installation

To install, use the Arduino Library Manager and search for "Adafruit SPA06_003" and install the library.

## Dependencies

This library depends on the [Adafruit BusIO library](https://github.com/adafruit/Adafruit_BusIO)

## Contributing

Contributions are welcome! Please read our [Code of Conduct](https://github.com/adafruit/Adafruit_SPA06_003/blob/main/CODE_OF_CONDUCT.md)
before contributing to help this project stay welcoming.

## Documentation and doxygen

Documentation is produced by doxygen. Contributions should include documentation for any new features.

## Formatting and clang-format

This library uses [`clang-format`](https://releases.llvm.org/download.html) to standardize the formatting of `.cpp` and `.h` files. 
Contributions should be formatted using `clang-format`:

The `-i` flag will make the changes to the file.
```bash
clang-format -i *.cpp *.h
```
If you prefer to make the changes yourself, running `clang-format` without the `-i` flag will print out a formatted version of the file. You can save this to a file and diff it against the original to see the changes.

Note that the formatting output by `clang-format` is what the automated formatting checker will expect. Any irrelevant changes (such as un-necessary whitespace changes) will result in a failed build until they are addressed. Using the `-i` flag is highly recommended.

## About this Driver

Written by Limor 'ladyada' Fried with assistance from Claude Code for Adafruit Industries. MIT license, check license.txt for more information All text above must be included in any redistribution

To install, use the Arduino Library Manager and search for "Adafruit SPA06_003" and install the library.