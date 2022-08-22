/*
  RTC_Seconds

  This sketch allows to test STM32RTC Seconds IRQ.

  Creation 25 nov 2021
  by FRASTM for STMicroelectronics

  This example code is in the public domain.

  Note that this sketch is valid for STM32F1xx or stm32 MCU with WakeUp interrupt
       (WUTE flag present in the RTC CR register)

  https://github.com/stm32duino/STM32RTC

*/

#include <STM32RTC.h>

#ifndef ONESECOND_IRQn
#error "RTC has no feature for One-Second interrupt"
#endif

/* use led to display seconds */
#if defined(LED_BUILTIN)
#define pin  LED_BUILTIN
#endif

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

/* Change these values to set the current initial time

   format: date: "Dec 31 2017" and time: "23:59:56"
   by default use built date and time
*/
/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 5;
const byte hours = 11;

/* Change these values to set the current initial date */
/* Monday 25 Nov. 2021 */
const byte weekDay = 4;
const byte day = 25;
const byte month = 11;
const byte year = 21;

bool toggling = false; // changed each second by the CallBack function

static STM32RTC::Hour_Format hourFormat = STM32RTC::HOUR_24;
static STM32RTC::AM_PM period = STM32RTC::AM;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

#if defined(LED_BUILTIN)
  // configure pin in output mode
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
#endif /* LED_BUILTIN */

  Serial.print("RTC Init ");

  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source. Use LSE for better accuracy if available
  // rtc.setClockSource(STM32RTC::LSE_CLOCK);

  // initialize RTC 24H format
  rtc.begin(hourFormat);

  // Set the time
  rtc.setHours(hours, period);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setWeekDay(weekDay);
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  Serial.println("with seconds Alarm");
  rtc.attachSecondsInterrupt(rtc_SecondsCB);
}

void loop()
{
  uint8_t sec = 0;
  uint8_t mn = 0;
  uint8_t hrs = 0;
  uint32_t subs = 0;
  rtc.getTime(&hrs, &mn, &sec, &subs);

  if (toggling) {
   Serial.printf("%02d:%02d:%02d\n", hrs, mn, sec);
  } else {
   Serial.printf("%02d %02d %02d\n", hrs, mn, sec);
  }
#if defined(LED_BUILTIN)
  digitalWrite(pin, toggling);
#endif /* LED_BUILTIN */
  delay(1000);
}

/* callback function on each second interrupt */
void rtc_SecondsCB(void *data)
{
  UNUSED(data);
  toggling = !toggling;
}
