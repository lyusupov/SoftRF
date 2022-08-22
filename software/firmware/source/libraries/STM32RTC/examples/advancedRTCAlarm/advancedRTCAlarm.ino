/*
  advancedRTCAlarm

  This sketch is an extension of simpleRTCAlarm.
  It uses the optional 'data' alarm callback parameters to
  reload alarm with 'atime' offset indefinitely.

  Creation 25 May 2018
  by Frederic Pillon for STMicroelectronics
  Modified 03 Jul 2020
  by Frederic Pillon for STMicroelectronics

  This example code is in the public domain.

  https://github.com/stm32duino/STM32RTC
*/

#include <STM32RTC.h>

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

/* Declare it volatile since it's incremented inside an interrupt */
volatile int alarmMatch_counter = 0;

/* Change this value to set alarm match offset in millisecond */
/* Note that STM32F1xx does not manage subsecond only second */
static uint32_t atime = 678;

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
const byte day = 25;
const byte month = 9;
const byte year = 15;

void setup()
{
  Serial.begin(9600);

  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  //rtc.setClockSource(STM32RTC::LSE_CLOCK);

  rtc.begin(); // initialize RTC 24H format

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);

  rtc.attachInterrupt(alarmMatch, &atime);
  rtc.setAlarmDay(day);
  rtc.setAlarmTime(16, 0, 10, 567);
  rtc.enableAlarm(rtc.MATCH_DHHMMSS);
}

void loop()
{

}

void alarmMatch(void *data)
{
  uint32_t epoc;
  uint32_t epoc_ms;
  uint32_t sec = 0;
  uint32_t _millis = 1000;

  if (data != NULL) {
    _millis = *(uint32_t*)data;
    // Minimum is 1 second
    if (sec == 0) {
      sec = 1;
    }
  }

  sec = _millis / 1000;
#ifdef STM32F1xx
  // Minimum is 1 second
  if (sec == 0) {
    sec = 1;
  }
  epoc = rtc.getEpoch(&epoc_ms);
#else
  _millis = _millis % 1000;
  epoc = rtc.getEpoch(&epoc_ms);

  //Update epoch_ms - might need to add a second to epoch
  epoc_ms += _millis;
  if (epoc_ms >= 1000) {
    sec ++;
    epoc_ms -= 1000;
  }
#endif
  Serial.printf("Alarm Match %i\n", ++alarmMatch_counter);
  rtc.setAlarmEpoch(epoc + sec, STM32RTC::MATCH_SS, epoc_ms);
}
