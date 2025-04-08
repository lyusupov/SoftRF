/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      XL9555_AdjustBacklight.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-11-22
 * @note      Use XL9555 to drive AW9364 led driver, use esp32 to test, the highest I2C communication rate must reach 1MHz, otherwise it will not work
 */
#include <Arduino.h>
#include "ExtensionIOXL9555.hpp"
#include "AW9364LedDriver.hpp"

#ifdef ENABLE_TFT
#include "TFT_eSPI.h"
TFT_eSPI tft;
#endif

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  -1
#endif

// Drive LED  pin, corresponding to XL9555 GPIO2
#define BACKLIGHT_PIN   2

ExtensionIOXL9555 io;
AW9364LedDriver ledDriver;
uint8_t level = 0;

void setup()
{
    Serial.begin(115200);

    // Set the interrupt input to input pull-up
    if (SENSOR_IRQ > 0) {
        pinMode(SENSOR_IRQ, INPUT_PULLUP);
    }

#ifdef ENABLE_TFT
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_GREEN);
#endif

    /*
    *
    *    If the device address is not known, the 0xFF parameter can be passed in.
    *
    *    XL9555_UNKOWN_ADDRESS  = 0xFF
    *
    *    If the device address is known, the device address is given
    *
    *    XL9555_SLAVE_ADDRESS0  = 0x20
    *    XL9555_SLAVE_ADDRESS1  = 0x21
    *    XL9555_SLAVE_ADDRESS2  = 0x22
    *    XL9555_SLAVE_ADDRESS3  = 0x23
    *    XL9555_SLAVE_ADDRESS4  = 0x24
    *    XL9555_SLAVE_ADDRESS5  = 0x25
    *    XL9555_SLAVE_ADDRESS6  = 0x26
    *    XL9555_SLAVE_ADDRESS7  = 0x27
    */
    const uint8_t chip_address = XL9555_UNKOWN_ADDRESS;

    if (!io.begin(Wire, chip_address, SENSOR_SDA, SENSOR_SCL)) {
        while (1) {
            Serial.println("Failed to find XL9555 - check your wiring!");
            delay(1000);
        }
    }

    ledDriver.begin(&io, BACKLIGHT_PIN);

}

void loop()
{
    // 16 dimming levels
    Serial.print("level:");
    Serial.println(level);
    ledDriver.setBrightness(level);
    level++;
    level %= MAX_BRIGHTNESS_STEPS;
    delay(1000);
}



