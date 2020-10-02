#ifndef WATCHDOGSAMD_H
#define WATCHDOGSAMD_H

#include <Arduino.h>

class WatchdogSAMD {
public:
  WatchdogSAMD() : _initialized(false) {}

  // Enable the watchdog timer to reset the machine after a period of time
  // without any calls to reset().  The passed in period (in milliseconds)
  // is just a suggestion and a lower value might be picked if the hardware
  // does not support the exact desired value.
  // User code should NOT set the 'isForSleep' argument either way --
  // it's used internally by the library, but your sketch should leave this
  // out when calling enable(), just let the default have its way.
  //
  // The actual period (in milliseconds) before a watchdog timer reset is
  // returned.
  int enable(int maxPeriodMS = 0, bool isForSleep = false);

  // Reset or 'kick' the watchdog timer to prevent a reset of the device.
  void reset();

  // Find out the cause of the last reset - see datasheet for bitmask
  uint8_t resetCause();

  // Completely disable the watchdog timer.
  void disable();

  // Enter the lowest power sleep mode (using the watchdog timer) for the
  // desired period of time.  The passed in period (in milliseconds) is
  // just a suggestion and a lower value might be picked if the hardware
  // does not support the exact desired value
  //
  // The actual period (in milliseconds) that the hardware was asleep will be
  // returned.
  int sleep(int maxPeriodMS = 0);

private:
  void _initialize_wdt();

  bool _initialized;
};

#endif
