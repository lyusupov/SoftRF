/*
  advancedRTCAlarm

  This sketch is an extension of simpleRTCAlarm.
  It uses the optional 'data' alarm callback parameters to
  reload alarm with 'atime' offset indefinitely.


  Creation 25 May 2018
  by Frederic Pillon for STMicroelectronics

  This example code is in the public domain.

  https://github.com/stm32duino/STM32RTC

*/

#include <STM32RTC.h>

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

/* Declare it volatile since it's incremented inside an interrupt */
volatile int alarmMatch_counter = 0;

/* Change this value to set alarm match offset */
static uint32_t atime = 5;

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
  rtc.setAlarmTime(16, 0, 10);
  rtc.enableAlarm(rtc.MATCH_DHHMMSS);
}

void loop()
{

}

void alarmMatch(void *data)
{
  uint32_t sec = 1;
  if(data != NULL) {
    sec = *(uint32_t*)data;
    // Minimum is 1 second
    if (sec == 0){
      sec = 1;
    }
  }
  alarmMatch_counter++;
  Serial.print("Alarm Match ");
  Serial.println(alarmMatch_counter);
  rtc.setAlarmEpoch( rtc.getEpoch() + sec);
}
