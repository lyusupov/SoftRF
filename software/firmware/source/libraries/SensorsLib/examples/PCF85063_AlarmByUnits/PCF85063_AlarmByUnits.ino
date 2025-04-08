/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      PCF85063_AlarmByUnits.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-12-11
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <time.h>
#include "SensorPCF85063.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  4
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  5
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  14
#endif


SensorPCF85063 rtc;


uint32_t intervalue = 0;
uint8_t nextHour = 22;
uint8_t nextMonth = 1;
uint8_t nextDay = 1;
uint8_t nextMinute = 59;
uint8_t nextSecond = 20;


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    if (!rtc.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find PCF85063 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    pinMode(SENSOR_IRQ, INPUT_PULLUP);

    rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);

    nextSecond = 55;

    // First test alarm seconds
    rtc.setAlarmBySecond(30);

    rtc.enableAlarm();

}

void printDateTime()
{
    if (millis() - intervalue > 1000) {
        /**
        /// Format output time*
        Option:
            DT_FMT_HM,          // Format Style : Hour:Minute
            DT_FMT_HMS,         // Format Style : Hour:Minute:Second
            DT_FMT_YMD,         // Format Style : Year-Month-Day
            DT_FMT_MDY,         // Format Style : Month-Day-Year
            DT_FMT_DMY,         // Format Style : Day-Month-Year
            DT_FMT_YMD_HMS,     // Format Style : Year-Month-Day/Hour:Minute:Second
            Default : DT_FMT_YMD_HMS_WEEK // Format Style : Year-Month-Day/Hour:Minute:Second - Weekday
        */
        Serial.println(rtc.strftime());

        intervalue = millis();
    }
}

// Test seconds timing
void  testAlarmSeconds()
{
    while (1) {
        if (digitalRead(SENSOR_IRQ) == LOW) {
            Serial.println("testAlarmSeconds Interrupt .... ");
            if (rtc.isAlarmActive()) {
                Serial.println("Alarm active");
                rtc.resetAlarm();
                rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                rtc.setAlarmByMinutes(0);
                return;
            }
        }
        printDateTime();
    }
}


// Test minute timing
void  testAlarmMinute()
{
    while (1) {
        if (digitalRead(SENSOR_IRQ) == LOW) {
            Serial.println("testAlarmMinute Interrupt .... ");
            if (rtc.isAlarmActive()) {
                Serial.println("Alarm active");
                rtc.resetAlarm();
                rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                nextHour++;
                if (nextHour >= 24) {
                    nextHour = 23;
                    nextDay = 25;
                    rtc.setAlarmByHours(0);
                    Serial.println("setAlarmByHours");
                    return;
                }
            }
        }
        printDateTime();
    }
}

// Test hour timing
void  testAlarmHour()
{
    while (1) {
        if (digitalRead(SENSOR_IRQ) == LOW) {
            Serial.println("testAlarmHour Interrupt .... ");
            if (rtc.isAlarmActive()) {
                Serial.println("Alarm active");
                rtc.resetAlarm();
                rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                nextDay++;
                if (nextDay >= 30) {
                    nextMonth = 1;
                    nextHour = 23;
                    nextMinute = 59;
                    nextSecond = 55;
                    nextDay = rtc.getDaysInMonth(nextMonth, 2022);
                    rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                    rtc.setAlarmByDays(1);
                    Serial.println("setAlarmByDays");
                    return;
                }
            }
        }
        printDateTime();
    }
}

// Test day timing
void  testAlarmDay()
{
    while (1) {
        if (digitalRead(SENSOR_IRQ) == LOW) {
            Serial.println("testAlarmDay Interrupt .... ");
            if (rtc.isAlarmActive()) {
                Serial.println("Alarm active");
                rtc.resetAlarm();
                nextDay = rtc.getDaysInMonth(nextMonth, 2022);
                rtc.setDateTime(2022, nextMonth, nextDay, nextHour, nextMinute, nextSecond);
                nextMonth++;
                if (nextMonth >= 12) {
                    return;
                }
            }
        }
        printDateTime();
    }
}




void loop()
{
    testAlarmSeconds();
    testAlarmMinute();
    testAlarmHour();
    testAlarmDay();

    Serial.println("Test done ...");
    while (1) {
        delay(100);
    }
}



