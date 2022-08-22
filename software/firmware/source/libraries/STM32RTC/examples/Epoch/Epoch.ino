/*
  Epoch

  This sketch shows how to manage the RTC using Epoch time

  Creation 12 Dec 2017
  by Wi6Labs
  Modified 03 Jul 2020
  by Frederic Pillon for STMicroelectronics

  This example code is in the public domain.

  https://github.com/stm32duino/STM32RTC
*/

#include <STM32RTC.h>
#include <time.h>

/* Get the rtc object */
STM32RTC& rtc = STM32RTC::getInstance();

void setup() {
  Serial.begin(9600);

  // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
  // By default the LSI is selected as source.
  //rtc.setClockSource(STM32RTC::LSE_CLOCK);

  rtc.begin(); // initialize RTC 24H format

  rtc.setEpoch(1451606400); // Jan 1, 2016
}

void loop() {
  uint32_t ss = rtc.getSubSeconds();
  uint32_t epoch = rtc.getEpoch();
  time_t rawtime = epoch;
  struct tm ts;
  char buf[80];

  Serial.print("Unix time = ");
  Serial.println(epoch);

  Serial.print("Seconds since Jan 1 2000 = ");
  Serial.println(rtc.getY2kEpoch());

  // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
  ts = *localtime(&rawtime);
  strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S", &ts);
  Serial.print(buf);
  Serial.print(".");
  print2digits(ss);
  Serial.println();

  delay(678);
}

void print2digits(uint32_t number) {
  if (number < 100) {
    Serial.print("0");
  }
  if (number < 10) {
    Serial.print("0");
  }
  Serial.print(number);
}