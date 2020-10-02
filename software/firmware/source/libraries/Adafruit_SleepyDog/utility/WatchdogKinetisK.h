#ifndef WATCHDOGKINETISK_H
#define WATCHDOGKINETISK_H

class WatchdogKinetisKseries {
public:
  WatchdogKinetisKseries() : setting(0) {}

  // Enable the watchdog timer to reset the machine after a period of time
  // without any calls to reset().  The passed in period (in milliseconds) is
  // just a suggestion and a lower value might be picked if the hardware does
  // not support the exact desired value.
  //
  // The actual period (in milliseconds) before a watchdog timer reset is
  // returned.
  int enable(int maxPeriodMS = 0);

  // Reset or 'kick' the watchdog timer to prevent a reset of the device.
  void reset();

  // Completely disable the watchdog timer.
  void disable();

  // Enter the lowest power sleep mode (using the watchdog timer) for the
  // desired period of time.  The passed in period (in milliseconds) is
  // just a suggestion and a lower value might be picked if the hardware does
  // not support the exact desired value
  //
  // The actual period (in milliseconds) that the hardware was asleep will be
  // returned.
  //
  // NOTE: This is currently not implemented on the SAMD21!
  int sleep(int maxPeriodMS = 0);

private:
  int setting;
};

#endif
