/*!
 * @file WatchdogRP2040.h
 *
 * Support for RP2040 Hardware Watchdog Timer API
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
#ifndef WATCHDOGRP2040_H_
#define WATCHDOGRP2040_H_

#include <hardware/watchdog.h>
#include <pico/time.h>

/**************************************************************************/
/*!
    @brief  Class that contains functions for interacting with the
            RP2040's hardware watchdog timer
*/
/**************************************************************************/
class WatchdogRP2040 {
public:
  WatchdogRP2040() : _wdto(-1){};
  int enable(int maxPeriodMS = 0);
  void disable()
      __attribute__((error("RP2040 WDT cannot be disabled once enabled")));
  void reset();
  int sleep(int maxPeriodMS = 0);

private:
  int _wdto;
};

#endif // WatchdogRP2040_H