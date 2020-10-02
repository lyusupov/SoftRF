// Be careful to use a platform-specific conditional include to only make the
// code visible for the appropriate platform.  Arduino will try to compile and
// link all .cpp files regardless of platform.
#if defined(__MK20DX128__) || defined(__MK20DX256__) ||                        \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)

#include "WatchdogKinetisK.h"
#include <kinetis.h>

static void one_bus_cycle(void) __attribute__((always_inline));
static void watchdog_config(int cfg, int val);

// Enable the watchdog timer to reset the machine after a period of time
// without any calls to reset().  The passed in period (in milliseconds) is
// just a suggestion and a lower value might be picked if the hardware does
// not support the exact desired value.
//
// The actual period (in milliseconds) before a watchdog timer reset is
// returned.
int WatchdogKinetisKseries::enable(int maxPeriodMS) {
  if (maxPeriodMS < 4) {
    maxPeriodMS = 8000; // default is 8 seconds
  }
  if (setting != maxPeriodMS) {
    watchdog_config(WDOG_STCTRLH_WDOGEN, maxPeriodMS);
    setting = maxPeriodMS;
  }
  return maxPeriodMS;
}

// Reset or 'kick' the watchdog timer to prevent a reset of the device.
void WatchdogKinetisKseries::reset() {
  __disable_irq();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  __enable_irq();
}

// Completely disable the watchdog timer.
void WatchdogKinetisKseries::disable() {
  if (setting > 0) {
    watchdog_config(0, 4);
    setting = 0;
  }
}

// Enter the lowest power sleep mode for the desired period of time.  The
// passed in period (in milliseconds) is just a suggestion and a lower value
// might be picked if the hardware does not support the exact desired value
//
// The actual period (in milliseconds) that the hardware was asleep will be
// returned.
int WatchdogKinetisKseries::sleep(int maxPeriodMS) {
  if (maxPeriodMS <= 0)
    return 0;
  // TODO....
  return 0;
}

static void watchdog_config(int cfg, int val) {
  __disable_irq();
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  one_bus_cycle();
  WDOG_STCTRLH = cfg | WDOG_STCTRLH_ALLOWUPDATE;
  WDOG_TOVALH = val >> 16;
  WDOG_TOVALL = val;
  WDOG_PRESC = 0;
  __enable_irq();
  for (int i = 0; i < 256; i++) {
    one_bus_cycle();
  }
}

static void one_bus_cycle(void) {
  __asm__ volatile("nop");
#if (F_CPU / F_BUS) > 1
  __asm__ volatile("nop");
#endif
#if (F_CPU / F_BUS) > 2
  __asm__ volatile("nop");
#endif
#if (F_CPU / F_BUS) > 3
  __asm__ volatile("nop");
#endif
#if (F_CPU / F_BUS) > 4
  __asm__ volatile("nop");
#endif
#if (F_CPU / F_BUS) > 5
  __asm__ volatile("nop");
#endif
#if (F_CPU / F_BUS) > 6
  __asm__ volatile("nop");
#endif
#if (F_CPU / F_BUS) > 7
  __asm__ volatile("nop");
#endif
}

#endif
