#ifndef WATCHDOGAVR_H
#define WATCHDOGAVR_H

class WatchdogAVR {
public:
  WatchdogAVR() : _wdto(-1) {}

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
  int sleep(int maxPeriodMS = 0);

private:
  // Pick the closest (but not higher) watchdog timer value from the provided
  // maximum period.  Sets wdto to the chosen period value suitable for
  // passing to wdt_enable(), and actualMS to the chosen period value in
  // milliseconds.  A max value of 0 will pick the longest value possible.
  void _setPeriod(int maxMS, int &wdto, int &actualMS);

  // Keep the last selected watchdog timer period so that the watchdog can be
  // re-enabled at that rate after sleep.  A value of -1 means no watchdog
  // timer was enabled.
  int _wdto;
};

#endif