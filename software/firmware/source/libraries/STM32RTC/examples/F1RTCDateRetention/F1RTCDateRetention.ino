/*
  F1RTCDateRetention

  This sketch shows how to keep the RTC date with backup registers on a stm32F1 MCU.
  It is reserved for stm32F1 as other devices actually embed the feature.
  If a VBat is present on the target board, then the content is kept when
  the VDD power supply is off, else the content is reset

  Creation 03 dec 2021
  by FRASTM for STMicroelectronics

  This example code is in the public domain.

  https://github.com/stm32duino/STM32RTC
*/

#include <STM32RTC.h>

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

#if !defined(STM32F1xx)
#error "Not applicable (only stm32F1xx save date in backup memory)"
#endif /* !STM32F1xx */

#define INITIAL_SEC 53
#define INITIAL_MIN 59
#define INITIAL_HOUR 23
#define INITIAL_WDAY 2
#define INITIAL_DAY 7
#define INITIAL_MONTH 12
#define INITIAL_YEAR 21

uint32_t subSec;
byte seconds;
byte minutes;
byte hours;
byte am_pm;

/* these values are read in the backUp register */
byte weekDay;
byte day;
byte month;
byte year;

void setup()
{
  Serial.begin(115200);

  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  //rtc.setClockSource(STM32RTC::LSE_CLOCK);

  // true: reset the backUp register (restart from the initial date/time above on each reset)
  // false: keep the latest date from the BackUp register on each reset
  bool reset_bkup = false;
  rtc.begin(reset_bkup); // initialize RTC 24H format
  // get the date if stored in BackUp reg
  rtc.getDate(&weekDay, &day, &month, &year);
  if (reset_bkup) {
    Serial.printf("reset the date to %02d/%02d/%02d\n", day, month, year);
  } else {
    Serial.printf("date from the BackUp %02d/%02d/%02d\n", day, month, year);
  }

  // Check if date is valid = read in the backUp reg.
  // Note that backup reg only keep the date /time
  // and a power off reset the content if not saved by Vbat
  // HAL_RTC init date is set to 1st of January 2000
if ((day == 1) && (month == RTC_MONTH_JANUARY) && (year == 0)) {
    // Set the time
    rtc.setHours(INITIAL_HOUR);
    rtc.setMinutes(INITIAL_MIN);
    rtc.setSeconds(INITIAL_SEC);

    // Set the date
    rtc.setWeekDay(INITIAL_WDAY);
    rtc.setDay(INITIAL_DAY);
    rtc.setMonth(INITIAL_MONTH);
    rtc.setYear(INITIAL_YEAR);

    // Here you can also use
    //rtc.setTime(hours, minutes, seconds);
    //rtc.setDate(weekDay, day, month, year);
  }
}

void loop()
{
  // Print date...
  Serial.printf("%02d/%02d/%02d \t", rtc.getDay(), rtc.getMonth(), rtc.getYear());

  uint8_t sec = 0;
  uint8_t mn = 0;
  uint8_t hrs = 0;
  uint32_t subs = 0;
  rtc.getTime(&hrs, &mn, &sec, &subs);

  // ...and time
  Serial.printf("%02d:%02d:%02d\n", hrs, mn, sec);

  delay(1000);
}
