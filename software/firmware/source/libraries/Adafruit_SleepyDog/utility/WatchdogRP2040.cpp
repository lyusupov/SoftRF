#if defined(ARDUINO_ARCH_RP2040)

#include "WatchdogRP2040.h"

/**********************************************************************************************/
/*!
    @brief  Initializes the RP2040's hardware watchdog timer.
    @param    maxPeriodMS
              Timeout period of WDT in milliseconds
    @return The actual period (in milliseconds) before a watchdog timer
            reset is returned, 0 otherwise.
*/
/**********************************************************************************************/
int WatchdogRP2040::enable(int maxPeriodMS) {
  if (maxPeriodMS < 0)
    return 0;

  // Enables the RP2040's hardware WDT with maxPeriodMS delay
  // (wdt should be updated every maxPeriodMS ms) and
  // enables pausing the WDT on debugging when stepping thru
  watchdog_enable(maxPeriodMS, 1);

  _wdto = maxPeriodMS;
  return maxPeriodMS;
}

/**************************************************************************/
/*!
    @brief  Reload the watchdog counter with the amount of time set in
            enable().
*/
/**************************************************************************/
void WatchdogRP2040::reset() { watchdog_update(); }

/**************************************************************************/
/*!
    @brief  Once enabled, the RP2040's Watchdog Timer can NOT be disabled.
*/
/**************************************************************************/
void WatchdogRP2040::disable() {}

/**************************************************************************/
/*!
    @brief  Configures the RP2040 to enter a lower power (WFE) sleep
            for a period of time.
    @param    maxPeriodMS
              Time to sleep the RP2040, in millis.
    @return The actual period (in milliseconds) that the hardware was
            asleep will be returned. Otherwise, 0 will be returned if the
            hardware could not enter the low-power mode.
*/
/**************************************************************************/
int WatchdogRP2040::sleep(int maxPeriodMS) {
  if (maxPeriodMS < 0)
    return 0;

  // perform a lower power (WFE) sleep (pico-core calls sleep_ms(sleepTime))
  sleep_ms(maxPeriodMS);

  return maxPeriodMS;
}

#endif // ARDUINO_ARCH_RP2040
