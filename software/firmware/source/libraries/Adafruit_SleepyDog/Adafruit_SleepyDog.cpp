/*!
 * @file Adafruit_SleepyDog.cpp
 *
 * @mainpage Adafruit SleepyDog Library
 *
 * @section intro_sec Introduction
 *
 * Arduino library to use the watchdog timer for system reset and low power
 * sleep.
 *
 * Currently supports the following hardware:
 *
 * Arduino Uno or other ATmega328P-based boards.
 * Arduino Mega or other ATmega2560- or 1280-based boards.
 * Arduino Zero, Adafruit Feather M0 (ATSAMD21).
 * Arduino Leonardo or other 32u4-based boards (e.g. Adafruit Feather) WITH
 * CAVEAT: USB Serial connection is clobbered on sleep; if sketch does not
 * require Serial comms, this is not a concern. The example sketches all print
 * to Serial and appear frozen, but the logic does otherwise continue to run.
 * You can restore the USB serial connection after waking up using
 * `USBDevice.attach();` and then reconnect to USB serial from the host machine.
 * Partial support for Teensy 3.X and LC (watchdog, no sleep).
 *
 * Adafruit Trinket and other boards using ATtiny MCUs are NOT supported.
 */

#include "Adafruit_SleepyDog.h"

/*!
 * @brief Global instance of the main class for sketches to use.
 */
WatchdogType Watchdog;
