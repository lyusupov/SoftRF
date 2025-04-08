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
 * @file      TouchDrv_FT6232_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-02
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvFT6X36.hpp"

#ifndef TOUCH_SDA
#define TOUCH_SDA  23
#endif

#ifndef TOUCH_SCL
#define TOUCH_SCL  32
#endif

#ifndef TOUCH_IRQ
#define TOUCH_IRQ  38
#endif

TouchDrvFT6X36 touch;


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(TOUCH_IRQ, INPUT);

    if (!touch.begin(Wire, FT6X36_SLAVE_ADDRESS, TOUCH_SDA, TOUCH_SCL)) {
        Serial.println("Failed to find FT6X36 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
    touch.interruptTrigger();

    Serial.println("Init FT6X36 Sensor success!");
}


void loop()
{
    int16_t x[2], y[2];
    if (digitalRead(TOUCH_IRQ) == LOW) {
        uint8_t touched = touch.getPoint(x, y, 2);
        for (int i = 0; i < touched; ++i) {
            Serial.print("X[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(x[i]);
            Serial.print(" ");
            Serial.print(" Y[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(y[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
    delay(50);
}



