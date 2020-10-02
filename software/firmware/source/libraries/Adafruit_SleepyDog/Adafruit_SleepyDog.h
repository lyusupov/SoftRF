/*!
 * @file Adafruit_SleepyDog.h
 */

#ifndef ADAFRUIT_SLEEPYDOG_H
#define ADAFRUIT_SLEEPYDOG_H

// Platform-specific code goes below.  Each #ifdef should check for the presence
// of their platform and pull in the appropriate watchdog implementation type,
// then typedef it to WatchdogType so the .cpp file can create a global
// instance.
#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)
#include "utility/WatchdogAVR.h"
typedef WatchdogAVR WatchdogType;
#elif defined(ARDUINO_ARCH_SAMD)
// Arduino Zero / ATSAMD series CPU watchdog support.
#include "utility/WatchdogSAMD.h"
typedef WatchdogSAMD WatchdogType;
#elif defined(__MK20DX128__) || defined(__MK20DX256__) ||                      \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)
// Teensy 3.x watchdog support.
#include "utility/WatchdogKinetisK.h"
typedef WatchdogKinetisKseries WatchdogType;
#elif defined(__MKL26Z64__)
// Teensy LC watchdog support.
#include "utility/WatchdogKinetisL.h"
typedef WatchdogKinetisLseries WatchdogType;
#elif defined(NRF52_SERIES)
#include "utility/WatchdogNRF.h"
typedef WatchdogNRF WatchdogType;
#else
#error Unsupported platform for the Adafruit Watchdog library!
#endif

extern WatchdogType Watchdog;

#endif
