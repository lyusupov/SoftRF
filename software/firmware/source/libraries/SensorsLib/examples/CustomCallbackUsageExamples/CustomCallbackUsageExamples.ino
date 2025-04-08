/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      CustomCallbackUsageExamples.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-20
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorPCF8563.hpp"
#include "SensorBMA423.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  21
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  22
#endif

SensorPCF8563 rtc;
SensorBMA423 accel;

uint32_t intervalue;
char buf[64];

bool i2c_wr_function(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, bool writeReg, bool isWrite)
{
    if (isWrite) {
        Wire.beginTransmission(addr);
        if (writeReg) {
            Wire.write(reg);
        }
        if (buf && len > 0) {
            Wire.write(buf, len);
        }
        return (Wire.endTransmission() == 0);

    } else {
        if (writeReg) {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.endTransmission();
        }
        Wire.requestFrom(addr, static_cast<uint8_t>(len));
        for (size_t i = 0; i < len; ++i) {
            if (Wire.available()) {
                buf[i] = Wire.read();
            } else {
                return false;
            }
        }
        return true;
    }
}


uint32_t hal_callback(SensorCommCustomHal::Operation op, void *param1, void *param2)
{
    switch (op) {
    case SensorCommCustomHal::OP_PINMODE: {
        uint8_t pin = reinterpret_cast<uintptr_t>(param1);
        uint8_t mode = reinterpret_cast<uintptr_t>(param2);
        pinMode(pin, mode);
    }
    break;
    case SensorCommCustomHal::OP_DIGITALWRITE: {
        uint8_t pin = reinterpret_cast<uintptr_t>(param1);
        uint8_t level = reinterpret_cast<uintptr_t>(param2);
        digitalWrite(pin, level);
    }
    break;
    case SensorCommCustomHal::OP_DIGITALREAD: {
        uint8_t pin = reinterpret_cast<uintptr_t>(param1);
        return digitalRead(pin);
    }
    break;
    case SensorCommCustomHal::OP_MILLIS:
        return millis();
        break;
    case SensorCommCustomHal::OP_DELAY: {
        if (param1) {
            uint32_t ms = reinterpret_cast<uintptr_t>(param1);
            delay(ms);
        }
    }
    break;
    case SensorCommCustomHal::OP_DELAYMICROSECONDS: {
        uint32_t us = reinterpret_cast<uintptr_t>(param1);
        delayMicroseconds(us);
    }
    break;
    default:
        break;
    }
    return 0;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_STM32)) && !defined(ARDUINO_ARCH_MBED)
    Wire.setSCL(SENSOR_SCL);
    Wire.setSDA(SENSOR_SDA);
    Wire.begin();
#elif defined(ARDUINO_ARCH_NRF52)
    Wire.setPins(SENSOR_SDA, SENSOR_SCL);
    Wire.begin();
#elif defined(ARDUINO_ARCH_ESP32)
    Wire.begin(SENSOR_SDA, SENSOR_SCL);
#else
    Wire.begin();
#endif

    // Using SensorLib with callback functions
    // Other sensor classes also support the same methods and can be applied to different platforms
    if (!rtc.begin(i2c_wr_function)) {
        Serial.println("Failed to find PCF8563 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    // The simplest way to set up
    rtc.setDateTime(2024, 1, 17, 4, 21, 30);

    // Unix tm structure sets the time
    struct tm timeinfo;
    timeinfo.tm_yday = 2025 - 1900; //Counting starts from 1900, so subtract 1900 here
    timeinfo.tm_mon = 1 - 1;        //Months start at 0, so you need to subtract 1.
    timeinfo.tm_mday = 17;
    timeinfo.tm_hour = 4;
    timeinfo.tm_min = 30;
    timeinfo.tm_sec = 30;
    rtc.setDateTime(timeinfo);

    if (!accel.begin(i2c_wr_function, hal_callback)) {
        Serial.println("Failed to find BMA423 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    //Default 4G ,200HZ
    accel.configAccelerometer();

    accel.enableAccelerometer();
}


void loop()
{
    if (millis() - intervalue > 1000) {

        intervalue = millis();

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

        Serial.print("Temperature:");
        Serial.print(accel.getTemperature(SensorBMA423::TEMP_DEG));
        Serial.print("*C ");
        Serial.print(accel.getTemperature(SensorBMA423::TEMP_FAHRENHEIT));
        Serial.print("*F");
        Serial.print("Dir:");
        Serial.print(accel.direction());
        Serial.println();
    }
}



