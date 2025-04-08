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
 * @file      AW9364_LedDriver.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-11-22
 */
#include <Arduino.h>
#include "AW9364LedDriver.hpp"

#ifdef ENABLE_TFT
#include "TFT_eSPI.h"
TFT_eSPI tft;
#endif

// Drive LED pin
#define BACKLIGHT_PIN   2

AW9364LedDriver ledDriver;

uint8_t level = 0;

void setup()
{
    Serial.begin(115200);

#ifdef ENABLE_TFT
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_GREEN);
#endif

    // Initialize LED driver
    ledDriver.begin(BACKLIGHT_PIN);
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



