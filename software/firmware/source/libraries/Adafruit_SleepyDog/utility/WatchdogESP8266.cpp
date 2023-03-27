#if defined(ARDUINO_ARCH_ESP8266)

#include "WatchdogESP8266.h"

/**********************************************************************************************/
/*!
    @brief  Initializes the ESP8266's software WDT
    @param    maxPeriodMS
              Timeout period of WDT in milliseconds
    @return The actual period (in milliseconds) before a watchdog timer
            reset is returned, 0 otherwise.
    NOTE: Configuring the software WDT timeout maxPeriodMS value is NOT
   IMPLEMENTED in the ESP8266 BSP [1]. Further, an investigation into the
   default software WDT time yielded a fixed timeout period of 3.2 seconds [2].
    [1] https://github.com/esp8266/Arduino/blob/master/cores/esp8266/Esp.h#L91
    [2]https://sigmdel.ca/michel/program/esp8266/arduino/watchdogs_en.html#ESP8266_WDT_TIMEOUT
*/
/**********************************************************************************************/
int WatchdogESP8266::enable(int maxPeriodMS) {
  if (maxPeriodMS < 0)
    return 0;

  // Enable the WDT
  ESP.wdtEnable(0);

  _wdto = maxPeriodMS;
  return maxPeriodMS;
}

/**************************************************************************/
/*!
    @brief  Feeds the Watchdog timer.
    NOTE: Calling yield() or delay() also feeds the hardware and software
    watchdog timers.
*/
/**************************************************************************/
void WatchdogESP8266::reset() { ESP.wdtFeed(); }

/**************************************************************************/
/*!
    @brief  Disables the Watchdog Timer.
        NOTE: Please don't stop the software WDT too long
        (less than 6 seconds), otherwise it will trigger a hardware
        watchdog reset!
*/
/**************************************************************************/
void WatchdogESP8266::disable() { ESP.wdtDisable(); }

/**************************************************************************/
/*!
    @brief  Configures the ESP8266 to enter a low-power sleep mode for a
            desired amount of time.
    @param    maxPeriodMS
              Time to sleep the ESP8266, in millis.
    @return The actual period (in milliseconds) that the hardware was
            asleep will be returned. Otherwise, 0 will be returned if the
            hardware could not enter the low-power mode.
*/
/**************************************************************************/
int WatchdogESP8266::sleep(int maxPeriodMS) {
  if (maxPeriodMS < 0)
    return 0;
  // Convert from MS to microseconds
  uint64_t sleepTime = maxPeriodMS * 1000;

  // Assert that we can not sleep longer than the max. time calculated by ESP
  if (sleepTime > ESP.deepSleepMax())
    return 0;

  // Enters deep sleep with mode WAKE_RF_DEFAULT
  ESP.deepSleep(sleepTime);

  return maxPeriodMS;
}

#endif // ARDUINO_ARCH_ESP8266