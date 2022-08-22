# STM32RTC
A RTC library for STM32.

## Requirement
* [Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32) version >= 1.3.0

# API

This library is based on the Arduino RTCZero library.
The library allows to take control of the internal RTC of the STM32 boards.

Singleton design pattern is used to ensure that only one STM32RTC instance is instantiated:
```C++
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
* **`void getPrediv(int8_t *predivA, int16_t *predivS)`** : get (a)synchronous prescaler values if set else computed ones for the current clock source.
* **`void setPrediv(int8_t predivA, int16_t predivS)`** : set (a)synchronous prescaler values.  This function must be called before `begin()`. Use -1 to reset value and use computed ones. Those values have to match the following conditions: **_1Hz = RTC CLK source / ((predivA + 1) * (predivS + 1))_**

_SubSeconds management_
* **`uint32_t getSubSeconds(void)`**
* **`void setSubSeconds(uint32_t subSeconds)`**

_Hour format (AM or PM)_
* **`uint8_t getHours(AM_PM *period = nullptr)`**
* **`void setHours(uint8_t hours, AM_PM period = AM)`**
* **`void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds, uint32_t subSeconds = 1000, AM_PM period = AM)`**
* **`void setAlarmHours(uint8_t hours, AM_PM period = AM)`**
* **`uint8_t getAlarmHours(AM_PM *period)`**
* **`void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds, AM_PM period)`**

_Week day configuration_
* **`uint8_t getWeekDay(void)`**
* **`void setWeekDay(uint8_t weekDay)`**
* **`void setDate(uint8_t weekDay, uint8_t day, uint8_t month, uint8_t year)`**

_Time and date configuration (added for convenience)_
* **`void getTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds, uint32_t *subSeconds, AM_PM *period = nullptr)`**
* **`void getDate(uint8_t *weekDay, uint8_t *day, uint8_t *month, uint8_t *year)`**

### Since STM32RTC version higher than 1.0.3

_SubSeconds alarm management_

  Important note:
  - STM32F1 and STM32L1xx (Ultra Low Power Medium (ULPM) density) series do not support subsecond.
  - Subsecond “resolution” depends on synchronous prescaler value. Bigger than this value is, better resolution will get for subsecond.

  * **`void setAlarmSubSeconds(uint32_t subSeconds)`**

  * **Updated API:**
    * **`void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds, uint32_t subSeconds = 0, AM_PM period = AM)`**
    * **`uint32_t getEpoch(uint32_t *subSeconds = nullptr)`**
    * **`void setEpoch(uint32_t ts, uint32_t subSeconds = 0)`**
    * **`void setAlarmEpoch(uint32_t ts, Alarm_Match match = MATCH_DHHMMSS, uint32_t subSeconds = 0)`** 

_Library version management_

  STM32 RTC library version is based on Semantic Versioning 2.0.0 (https://semver.org/)

  This will ease some dependencies:

    * `STM32_RTC_VERSION_MAJOR` -> major version
    * `STM32_RTC_VERSION_MINOR` -> minor version
    * `STM32_RTC_VERSION_PATCH` -> patch version
    * `STM32_RTC_VERSION_EXTRA` -> Extra label
     with:
      - 0: official release
      - [1-9]: release candidate
      - F[0-9]: development

    * `STM32_RTC_VERSION` --> Full version number

  `STM32_RTC_VERSION` can de used to handle some API change:
```C++
#if defined(STM32_RTC_VERSION) && (STM32_RTC_VERSION  >= 0x01010000)
  rtc.setAlarmTime(alarmHours, alarmMinutes, alarmSeconds, 123);
#else
  rtc.setAlarmTime(alarmHours, alarmMinutes, alarmSeconds);
#endif
```

### Since STM32RTC version higher than 1.1.1

_One-Second interrupt_

  STM32 RTC includes a one-second interrupt for generating a periodic interrupt signal.
  - This feature is native on the stm32F1xx and mapped on the existing WakeUp interrupt on other stm32 mcus. 
  - It is not available on some stm32F0 devices.

  * **new API:**
    * **`void attachSecondsInterrupt(voidFuncPtr callback)`**
    * **`void detachSecondsInterrupt(void)`**


_Date retention for stm32F1xx_

  STM32 RTC includes date save/retrieve mechanism for the stm32F1xx mcu, that do not have a date counter.

  The content is stored in BackUp memory which is kept during Reset and powered by Vbat when the Vdd is off.


### Since STM32 Core version > 1.5.0
_Reset time management_

By default, if a time is set it will not be reset after a reboot.

Using `begin(true)` or `begin(true, HOUR_24)` will reset the RTC registers.

To know if a time has already been set use:
* **`bool isTimeSet(void)`**
```C++
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
