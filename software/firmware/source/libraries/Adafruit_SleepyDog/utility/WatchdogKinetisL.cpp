// Be careful to use a platform-specific conditional include to only make the
// code visible for the appropriate platform.  Arduino will try to compile and
// link all .cpp files regardless of platform.
#if defined(__MKL26Z64__)

#include "WatchdogKinetisL.h"
#include <kinetis.h>

// Normally the watchdog is disabled at startup.  This removes the startup
// code.  The watchdog will be active with 1024 ms timeout.  Hopefully the
// user will configure the watchdog and begin resetting it before it causes
// a reboot.  There is no way to start up without the watchdog and then
// enable it later...
extern "C" void startup_early_hook(void) {}

// Enable the watchdog timer to reset the machine after a period of time
// without any calls to reset().  The passed in period (in milliseconds) is
// just a suggestion and a lower value might be picked if the hardware does
// not support the exact desired value.
//
// The actual period (in milliseconds) before a watchdog timer reset is
// returned.
int WatchdogKinetisLseries::enable(int maxPeriodMS) {
  // The watchdog can only be programmed once.  Then it's forever
  // locked to this setting (until the chip reboots).
  if (maxPeriodMS <= 0 || maxPeriodMS > 256) {
    SIM_COPC = 12;
  } else if (maxPeriodMS > 32) {
    SIM_COPC = 8;
  } else {
    SIM_COPC = 4;
  }
  // Read the actual setting.
  int val = SIM_COPC & 12;
  if (val == 12)
    return 1024;
  if (val == 8)
    return 256;
  return 32;
}

// Reset or 'kick' the watchdog timer to prevent a reset of the device.
void WatchdogKinetisLseries::reset() {
  __disable_irq();
  SIM_SRVCOP = 0x55;
  SIM_SRVCOP = 0xAA;
  __enable_irq();
}

// Completely disable the watchdog timer.
void WatchdogKinetisLseries::disable() {
  // no can do....
  // The watchdog timer in this chip is write-once.
  // The chip boots up with the watchdog at 1024 ms.
  // You only get to configure it once.  Then it
  // remains locked to that setting, until a reboot.
}

// Enter the lowest power sleep mode for the desired period of time.  The
// passed in period (in milliseconds) is just a suggestion and a lower value
// might be picked if the hardware does not support the exact desired value
//
// The actual period (in milliseconds) that the hardware was asleep will be
// returned.
int WatchdogKinetisLseries::sleep(int maxPeriodMS) {
  if (maxPeriodMS <= 0)
    return 0;
  // TODO....
  return 0;
}

#endif
