# STM32RTC
A RTC library for STM32.

## Requirement
* [Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32) version >= 1.3.0

# API

This library is based on the Arduino RTCZero library.
The library allows to take control of the internal RTC of the STM32 boards.

Singleton design pattern is used to ensure that only one STM32RTC instance is instantiated:
```
/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();
```

The following functions are not supported:

* **`void standbyMode()`**: use the STM32 Low Power library instead.
* **`uint8_t getAlarmMonth()`**: month not supported by STM32 RTC architecture.
* **`uint8_t getAlarmYear()`**: year not supported by STM32 RTC architecture.
* **`void setAlarmMonth(uint8_t month)`**: month not supported by STM32 RTC architecture.
* **`void setAlarmYear(uint8_t year)`**: year not supported by STM32 RTC architecture.
* **`void setAlarmDate(uint8_t day, uint8_t month, uint8_t year)`**: month and year not supported by STM32 RTC architecture.

The following functions have been added to support specific STM32 RTC features:

_RTC hours mode (12 or 24)_
* **`void begin(Hour_Format format)`**

_RTC clock source_
* **`Source_Clock getClockSource(void)`** : get current clock source.
* **`void setClockSource(Source_Clock source)`** : this function must be called before `begin()`.

_RTC Asynchronous and Synchronous prescaler_
* **`void getPrediv(int8_t *predivA, int16_t *predivS)`** : get user (a)synchronous prescaler values if set else computed ones for the current clock source.
* **`void setPrediv(int8_t predivA, int16_t predivS)`** : set user (a)synchronous prescaler values.  This function must be called before `begin()`. Use -1 to reset value and use computed ones.

_SubSeconds management_
* **`uint32_t getSubSeconds(void)`**
* **`void setSubSeconds(uint32_t subSeconds)`**

_Hour format (AM or PM)_
* **`uint8_t getHours(AM_PM *period)`**
* **`void setHours(uint8_t hours, AM_PM period)`**
* **`void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds, uint32_t subSeconds, AM_PM period)`**
* **`void setAlarmHours(uint8_t hours, AM_PM period)`**
* **`uint8_t getAlarmHours(AM_PM *period)`**
* **`void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds, AM_PM period)`**

_Week day configuration_
* **`uint8_t getWeekDay(void)`**
* **`void setWeekDay(uint8_t weekDay)`**
* **`void setDate(uint8_t weekDay, uint8_t day, uint8_t month, uint8_t year)`**

_Time and date configuration (added for convenience)_
* **`void getTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds, uint32_t *subSeconds, AM_PM *period = NULL)`**
* **`void getDate(uint8_t *weekDay, uint8_t *day, uint8_t *month, uint8_t *year)`**

### Since STM32 Core version > 1.5.0
_Reset time management_
By default, if a time is set it will not be reset afer a reboot.

Using `begin(true)` or `begin(true, HOUR_24)` will reset the RTC registers.

To know if a time has already been set use:
* **`bool isTimeSet(void)`**
```
  if (!rtc.isTimeSet()) {
    // Set the time
    rtc.setHours(hours);
    rtc.setMinutes(minutes);
    rtc.setSeconds(seconds);
  }
```

Refer to the Arduino RTC documentation for the other functions  
http://arduino.cc/en/Reference/RTC

## Source

Source files available at:  
https://github.com/stm32duino/STM32RTC
