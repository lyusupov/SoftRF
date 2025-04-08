/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      PCF85063_SimpleTime.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-07
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <SensorPCF85063.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  7
#endif

SensorPCF85063 rtc;
uint32_t interval = 0;
uint32_t loopCount = 0;

void printInt(int val)
{
    if (val < 10) {
        Serial.print("0");
    }
    Serial.print(val);
}


void setup()
{

    Serial.begin(115200);
    // Wait for the serial port to be ready
    while (!Serial);

    // Try to initialize the RTC module using I2C with specified SDA and SCL pins
    if (!rtc.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find PCF85063 - check your wiring!");
        // Enter an infinite loop to halt the program
        while (1) {
            delay(1000);
        }
    }

    uint16_t year = 2023;
    uint8_t month = 9;
    uint8_t day = 7;
    uint8_t hour = 11;
    uint8_t minute = 24;
    uint8_t second = 30;

    // Set the defined date and time on the RTC
    rtc.setDateTime(year, month, day, hour, minute, second);

    if (!rtc.isClockIntegrityGuaranteed()) {
        Serial.println("[ERROR]:Clock integrity is not guaranteed; oscillator has stopped or has been interrupted");
    }
}

void loop()
{
    // Check if one second has passed since the last update
    if (millis() > interval) {

        // Update the interval to the current time
        interval = millis() + 1000;

        // Retrieve the current date and time from the RTC
        RTC_DateTime datetime = rtc.getDateTime();

        Serial.print("[RTC ]:");
        Serial.print(" Year :");  printInt(datetime.getYear());
        Serial.print(" Month:");  printInt(datetime.getMonth());
        Serial.print(" Day :");   printInt(datetime.getDay());
        Serial.print(" Hour:");   printInt(datetime.getHour());
        Serial.print(" Minute:"); printInt(datetime.getMinute());
        Serial.print(" Sec :");   printInt(datetime.getSecond());
        Serial.println();

        // Convert the RTC date and time to Unix time
        struct tm info = datetime.toUnixTime();
        Serial.print("[UNIX]:");
        Serial.print(" Year :");  printInt(info.tm_year + 1900); // tm_year starts counting from 1900
        Serial.print(" Month:");  printInt(info.tm_mon + 1);     // tm_mon range is 0 - 11, 0 means January
        Serial.print(" Day :");   printInt(info.tm_mday);
        Serial.print(" Hour:");   printInt(info.tm_hour);
        Serial.print(" Minute:"); printInt(info.tm_min);
        Serial.print(" Sec :");   printInt(info.tm_sec);
        Serial.println();

        // Set a new Unix time at the 10th loop iteration
        if (loopCount == 10) {
            Serial.print("Set Unix Time:");
            Serial.println();
            Serial.println();
            struct tm utc_tm;
            utc_tm.tm_year = 2025 - 1900; // tm_year starts counting from 1900
            utc_tm.tm_mon = 0;            // tm_mon range is 0 - 11, 0 means January
            utc_tm.tm_mday = 23;
            utc_tm.tm_hour = 7;
            utc_tm.tm_min = 1;
            utc_tm.tm_sec = 28;
            rtc.setDateTime(utc_tm);
        }

        // Set a UTC time with a time zone offset of 8 hours at the 20th loop iteration
        if (loopCount == 20) {
            Serial.print("Set UTC time to time zone offset 8 hours:");
            Serial.println();
            Serial.println();
            struct tm utc_tm;
            utc_tm.tm_year = 2025 - 1900; // tm_year starts counting from 1900
            utc_tm.tm_mon = 0;            // tm_mon range is 0 - 11, 0 means January
            utc_tm.tm_mday = 23;
            utc_tm.tm_hour = 7;
            utc_tm.tm_min = 1;
            utc_tm.tm_sec = 28;
            rtc.convertUtcToTimezone(utc_tm, 8 * 3600);
            rtc.setDateTime(utc_tm);
        }

        if (loopCount > 30) {

            char buf[64];

            struct tm timeinfo;
            // Get the time C library structure
            rtc.getDateTime(&timeinfo);

            // Format the output using the strftime function
            // For more formats, please refer to :
            // https://man7.org/linux/man-pages/man3/strftime.3.html

            size_t written = strftime(buf, 64, "%A, %B %d %Y %H:%M:%S", &timeinfo);

            if (written != 0) {
                Serial.println(buf);
            }

            written = strftime(buf, 64, "%b %d %Y %H:%M:%S", &timeinfo);
            if (written != 0) {
                Serial.println(buf);
            }

            written = strftime(buf, 64, "%A, %d. %B %Y %I:%M%p", &timeinfo);
            if (written != 0) {
                Serial.println(buf);
            }
        }

        ++loopCount;
    }
}
