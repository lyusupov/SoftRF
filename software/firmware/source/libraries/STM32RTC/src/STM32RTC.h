/**
  ******************************************************************************
  * @file    STM32RTC.h
  * @author  Frederic Pillon
  * @brief   Provides a RTC interface for Arduino
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef __STM32_RTC_H
#define __STM32_RTC_H

#include "Arduino.h"
#if defined(STM32_CORE_VERSION) && (STM32_CORE_VERSION  > 0x01090000)
  #include "rtc.h"
#else
bool RTC_IsConfigured(void);
bool RTC_IsAlarmSet(void);
#endif
// Check if RTC HAL enable in variants/board_name/stm32yzxx_hal_conf.h
#ifndef HAL_RTC_MODULE_ENABLED
  #error "RTC configuration is missing. Check flag HAL_RTC_MODULE_ENABLED in variants/board_name/stm32yzxx_hal_conf.h"
#endif

/**
 * @brief STM32 RTC library version number
 */
#define STM32_RTC_VERSION_MAJOR    (0x01U) /*!< [31:24] major version */
#define STM32_RTC_VERSION_MINOR    (0x03U) /*!< [23:16] minor version */
#define STM32_RTC_VERSION_PATCH    (0x01U) /*!< [15:8]  patch version */
/*
 * Extra label for development:
 * 0: official release
 * [1-9]: release candidate
 * F[0-9]: development
 */
#define STM32_RTC_VERSION_EXTRA    (0x00U) /*!< [7:0]  extra version */
#define STM32_RTC_VERSION          ((STM32_RTC_VERSION_MAJOR << 24U)\
                                        |(STM32_RTC_VERSION_MINOR << 16U)\
                                        |(STM32_RTC_VERSION_PATCH << 8U )\
                                        |(STM32_RTC_VERSION_EXTRA))

typedef void(*voidFuncPtr)(void *);

#define IS_CLOCK_SOURCE(SRC) (((SRC) == STM32RTC::LSI_CLOCK) || ((SRC) == STM32RTC::LSE_CLOCK) ||\
                              ((SRC) == STM32RTC::HSE_CLOCK))
#define IS_HOUR_FORMAT(FMT)  (((FMT) == STM32RTC::HOUR_12) || ((FMT) == STM32RTC::HOUR_24))

class STM32RTC {
  public:

    enum Hour_Format : uint8_t {
      HOUR_12 = HOUR_FORMAT_12,
      HOUR_24 = HOUR_FORMAT_24
    };

    enum AM_PM : uint8_t {
      AM = HOUR_AM,
      PM = HOUR_PM
    };

    enum Alarm_Match : uint8_t {
      MATCH_OFF          = OFF_MSK,                          // Never
      MATCH_SS           = SS_MSK,                           // Every Minute
      MATCH_MMSS         = SS_MSK | MM_MSK,                  // Every Hour
      MATCH_HHMMSS       = SS_MSK | MM_MSK | HH_MSK,         // Every Day
      MATCH_DHHMMSS      = SS_MSK | MM_MSK | HH_MSK | D_MSK, // Every Month
      /* NOTE: STM32 RTC can't assign a month or a year to an alarm. Those enum
      are kept for compatibility but are ignored inside enableAlarm(). */
      MATCH_MMDDHHMMSS   = SS_MSK | MM_MSK | HH_MSK | D_MSK | M_MSK,
      MATCH_YYMMDDHHMMSS = SS_MSK | MM_MSK | HH_MSK | D_MSK | M_MSK | Y_MSK
    };

    enum Source_Clock : uint8_t {
      LSI_CLOCK = ::LSI_CLOCK,
      LSE_CLOCK = ::LSE_CLOCK,
      HSE_CLOCK = ::HSE_CLOCK
    };

    static STM32RTC &getInstance()
    {
      static STM32RTC instance; // Guaranteed to be destroyed.
      // Instantiated on first use.
      return instance;
    }

    STM32RTC(STM32RTC const &)        = delete;
    void operator=(STM32RTC const &)  = delete;

    void begin(bool resetTime, Hour_Format format = HOUR_24);
    void begin(Hour_Format format = HOUR_24);

    void end(void);

    Source_Clock getClockSource(void);
    void setClockSource(Source_Clock source);

    void enableAlarm(Alarm_Match match);
    void disableAlarm(void);

    void attachInterrupt(voidFuncPtr callback, void *data = nullptr);
    void detachInterrupt(void);

#ifdef ONESECOND_IRQn
    // Other mcu than stm32F1 will use the WakeUp feature to interrupt each second.
    void attachSecondsInterrupt(voidFuncPtr callback);
    void detachSecondsInterrupt(void);

#endif /* ONESECOND_IRQn */
    // Kept for compatibility: use STM32LowPower library.
    void standbyMode();

    /* Get Functions */

    uint32_t getSubSeconds(void);
    uint8_t getSeconds(void);
    uint8_t getMinutes(void);
    uint8_t getHours(AM_PM *period = nullptr);
    void getTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds, uint32_t *subSeconds, AM_PM *period = nullptr);

    uint8_t getWeekDay(void);
    uint8_t getDay(void);
    uint8_t getMonth(void);
    uint8_t getYear(void);
    void getDate(uint8_t *weekDay, uint8_t *day, uint8_t *month, uint8_t *year);

    uint32_t getAlarmSubSeconds(void);
    uint8_t getAlarmSeconds(void);
    uint8_t getAlarmMinutes(void);
    uint8_t getAlarmHours(AM_PM *period = nullptr);

    uint8_t getAlarmDay(void);

    // Kept for compatibility with Arduino RTCZero library.
    uint8_t getAlarmMonth();
    uint8_t getAlarmYear();

    /* Set Functions */

    void setSubSeconds(uint32_t subSeconds);
    void setSeconds(uint8_t seconds);
    void setMinutes(uint8_t minutes);
    void setHours(uint8_t hours, AM_PM period = AM);
    void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds, uint32_t subSeconds = 1000, AM_PM period = AM);

    void setWeekDay(uint8_t weekDay);
    void setDay(uint8_t day);
    void setMonth(uint8_t month);
    void setYear(uint8_t year);
    void setDate(uint8_t day, uint8_t month, uint8_t year);
    void setDate(uint8_t weekDay, uint8_t day, uint8_t month, uint8_t year);

    void setAlarmSubSeconds(uint32_t subSeconds);
    void setAlarmSeconds(uint8_t seconds);
    void setAlarmMinutes(uint8_t minutes);
    void setAlarmHours(uint8_t hours, AM_PM period = AM);
    void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds, uint32_t subSeconds = 0, AM_PM period = AM);

    void setAlarmDay(uint8_t day);

    // Kept for compatibility with Arduino RTCZero library.
    void setAlarmMonth(uint8_t month);
    void setAlarmYear(uint8_t year);
    void setAlarmDate(uint8_t day, uint8_t month, uint8_t year);

    /* Epoch Functions */

    uint32_t getEpoch(uint32_t *subSeconds = nullptr);
    uint32_t getY2kEpoch(void);
    void setEpoch(uint32_t ts, uint32_t subSeconds = 0);
    void setY2kEpoch(uint32_t ts);
    void setAlarmEpoch(uint32_t ts, Alarm_Match match = MATCH_DHHMMSS, uint32_t subSeconds = 0);

#if defined(STM32F1xx)
    void getPrediv(uint32_t *predivA, int16_t *dummy = nullptr);
    void setPrediv(uint32_t predivA, int16_t dummy = 0);
#else
    void getPrediv(int8_t *predivA, int16_t *predivS);
    void setPrediv(int8_t predivA, int16_t predivS);
#endif /* STM32F1xx */
    bool isConfigured(void)
    {
      return RTC_IsConfigured();
    }
    bool isAlarmEnabled(void)
    {
      return RTC_IsAlarmSet();
    }
    bool isTimeSet(void)
    {
#if defined(STM32_CORE_VERSION) && (STM32_CORE_VERSION  > 0x01050000)
      return _timeSet;
#else
      return false;
#endif
    }

    friend class STM32LowPower;

  private:
    STM32RTC(void): _clockSource(LSI_CLOCK) {}

    static bool _timeSet;

    Hour_Format _format;
    AM_PM       _hoursPeriod;
    uint8_t     _hours;
    uint8_t     _minutes;
    uint8_t     _seconds;
    uint32_t    _subSeconds;
    uint8_t     _year;
    uint8_t     _month;
    uint8_t     _day;
    uint8_t     _wday;

    uint8_t     _alarmDay;
    uint8_t     _alarmHours;
    uint8_t     _alarmMinutes;
    uint8_t     _alarmSeconds;
    uint32_t    _alarmSubSeconds;
    AM_PM       _alarmPeriod;
    Alarm_Match _alarmMatch;

    Source_Clock _clockSource;

    void configForLowPower(Source_Clock source);

    void syncTime(void);
    void syncDate(void);
    void syncAlarmTime(void);

};

#endif // __STM32_RTC_H
