/*!
 * @file WatchdogESP32.h
 *
 * Support for ESP32 TWDT and low-power sleep modes.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * MIT License, all text here must be included in any redistribution.
 *
 */
#ifndef WATCHDOGESP32_H_
#define WATCHDOGESP32_H_
#include "esp_sleep.h"
#include "esp_task_wdt.h"

/**************************************************************************/
/*!
    @brief  Class that contains functions for interacting with the ESP32's
            WDT and low-power sleep functions.
*/
/**************************************************************************/
class WatchdogESP32 {
public:
  WatchdogESP32() : _wdto(-1){};
  int enable(int maxPeriodMS = 0);
  void reset();
  void disable();
  int sleep(int maxPeriodMS = 0);

private:
  int _wdto;
};

#endif // WATCHDOGESP32_H