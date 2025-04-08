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
 * @file      TouchDrv_CSTxxx_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvCSTXXX.hpp"

#ifndef TOUCH_SDA
#define TOUCH_SDA  8
#endif

#ifndef TOUCH_SCL
#define TOUCH_SCL  10
#endif

#ifndef TOUCH_IRQ
#define TOUCH_IRQ  5
#endif

#ifndef TOUCH_RST
#define TOUCH_RST  -1
#endif

TouchDrvCSTXXX touch;
int16_t x[5], y[5];
bool  isPressed = false;

void scanDevices(void)
{
    byte error, address;
    int nDevices = 0;
    Serial.println("Scanning for I2C devices ...");
    for (address = 0x01; address < 0x7f; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            Serial.println(address, HEX);
            nDevices++;
        } else if (error != 2) {
            Serial.print("Error ");
            Serial.print(error);
            Serial.print(" at address 0x");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

#if TOUCH_RST != -1
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(30);
    digitalWrite(TOUCH_RST, HIGH);
    delay(50);
    // delay(1000);
#endif

    // Search for known CSTxxx device addresses
    uint8_t address = 0xFF;

#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_STM32)) && !defined(ARDUINO_ARCH_MBED)
    Wire.setSCL(TOUCH_SCL);
    Wire.setSDA(TOUCH_SDA);
    Wire.begin();
#elif defined(ARDUINO_ARCH_NRF52)
    Wire.setPins(TOUCH_SDA, TOUCH_SCL);
    Wire.begin();
#elif defined(ARDUINO_ARCH_ESP32)
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
#else
    Wire.begin();
#endif

    // Scan I2C devices
    scanDevices();

    Wire.beginTransmission(CST816_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        address = CST816_SLAVE_ADDRESS;
    }
    Wire.beginTransmission(CST226SE_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        address = CST226SE_SLAVE_ADDRESS;
    }
    Wire.beginTransmission(CST328_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        address = CST328_SLAVE_ADDRESS;
    }
    while (address == 0xFF) {
        Serial.println("Could't find touch chip!"); delay(1000);
    }

    touch.setPins(TOUCH_RST, TOUCH_IRQ);

    /*
    * Support type.
    * TouchDrv_UNKOWN       : Judging by identification ID
    * TouchDrv_CST8XX       : CST816X,CST328,CST716,CST820
    * TouchDrv_CST226       : CST226X
    * TouchDrv_CST92XX      : CST9217,CST9220
    */
    // Can choose fixed touch model or automatic identification by ID
    // touch.setTouchDrvModel(TouchDrv_CST226);


    // Support CST81X CST226 CST9217 CST9220 ....
    bool result = touch.begin(Wire, address, TOUCH_SDA, TOUCH_SCL);
    if (result == false) {
        while (1) {
            Serial.println("Failed to initialize CST series touch, please check the connection...");
            delay(1000);
        }
    }

    Serial.print("Model :"); Serial.println(touch.getModelName());

    // T-Display-S3 CST328 touch panel, touch button coordinates are is 85 , 360
    // touch.setCenterButtonCoordinate(85, 360);

    // T-Display-AMOLED 1.91 Inch CST816T touch panel, touch button coordinates is 600, 120.
    // touch.setCenterButtonCoordinate(600, 120);  // Only suitable for AMOLED 1.91 inch

    // T-Display-Bar Inch CST820 touch panel, touch button coordinates is 30,400
    // touch.setCenterButtonCoordinate(30, 400);  // Only suitable for T-Display-Bar

    // Depending on the touch panel, not all touch panels have touch buttons.
    touch.setHomeButtonCallback([](void *user_data) {
        Serial.println("Home key pressed!");
    }, NULL);


    // Unable to obtain coordinates after turning on sleep
    // CST816T sleep current = 1.1 uA
    // CST226SE sleep current = 60 uA
    // touch.sleep();

    // Set touch max xy
    // touch.setMaxCoordinates(536, 240);

    // Set swap xy
    // touch.setSwapXY(true);

    // Set mirror xy
    // touch.setMirrorXY(true, true);

    //Register touch plane interrupt pin
    attachInterrupt(TOUCH_IRQ, []() {
        isPressed = true;
    }, FALLING);

}

void loop()
{
    if (isPressed) {
        isPressed = false;
        uint8_t touched = touch.getPoint(x, y, touch.getSupportTouchPoint());
        if (touched) {
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
    }

    delay(5);
}



