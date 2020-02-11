/*
  AdvancedTimedWakeup

  This sketch demonstrates the usage of Internal Interrupts to wakeup a chip in deep sleep mode.

  In this sketch:
  - RTC date and time are configured.
  - Alarm is set to wake up the processor each 'atime' and called a custom alarm callback
  which increment a value and reload alarm with 'atime' offset.

  This example code is in the public domain.
*/

#include "STM32LowPower.h"
#include <STM32RTC.h>

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

// Time in second between blink
static uint32_t atime = 1;

// Declare it volatile since it's incremented inside an interrupt
volatile int alarmMatch_counter = 0;

// Variables for RTC configurations
static byte seconds = 0;
static byte minutes = 0;
static byte hours = 0;

static byte weekDay = 1;
static byte day = 1;
static byte month = 1;
static byte year = 18;

void setup() {
  rtc.begin();
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(weekDay, day, month, year);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  while(!Serial) {}

  // Configure low power
  LowPower.begin();
  LowPower.enableWakeupFrom(&rtc, alarmMatch, &atime);

  // Configure first alarm in 2 second then it will be done in the rtc callback
  rtc.setAlarmEpoch( rtc.getEpoch() + 2 );
}

void loop() {
  Serial.print("Alarm Match: ");
  Serial.print(alarmMatch_counter);
  Serial.println(" times.");
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  LowPower.deepSleep();
  digitalWrite(LED_BUILTIN, LOW);
  LowPower.deepSleep();
}

void alarmMatch(void* data)
{
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
  uint32_t sec = 1;
  if(data != NULL) {
    sec = *(uint32_t*)data;
    // Minimum is 1 second
    if (sec == 0){
      sec = 1;
    }
  }
  alarmMatch_counter++;
  rtc.setAlarmEpoch( rtc.getEpoch() + sec);
}

