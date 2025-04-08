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
 * @file      CustomCallbackTouchDrvInterface.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-21
 * @note      CustomCallbackTouchDrvInterface use LilyGo T-RGB,T-RGB has three types of screens, each of which uses different touch driver chips.
 *            The example demonstrates using the touch interface class and one sketch is suitable for three types of touch chips.
 *            The example demonstrates using custom callbacks to read touch or sensors. This method is also applicable to other platforms.
 *            The prerequisite compilation platform must support C++11.
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvFT6X36.hpp"
#include "TouchDrvCSTXXX.hpp"
#include "TouchDrvGT911.hpp"
#include "ExtensionIOXL9555.hpp"
#include "SensorWireHelper.h"

#ifndef TOUCH_SDA
#define TOUCH_SDA  8
#endif

#ifndef TOUCH_SCL
#define TOUCH_SCL  48
#endif

#ifndef TOUCH_IRQ
#define TOUCH_IRQ  1
#endif

#ifndef TOUCH_RST
#define TOUCH_RST  1
#endif

// Use the TouchDrvInterface base class for automatic discovery and use of multi-touch devices
TouchDrvInterface *touchDrv;
// T-RGB uses XL9555 as the reset control of the touch screen
ExtensionIOXL9555 extension;

int16_t x[5], y[5];

ExtensionIOXL9555::ExtensionGPIO tp_reset = ExtensionIOXL9555::IO1;

/**
 * @brief  i2c_wr_function
 * @note   I2C communication using custom callbacks
 * @param  addr: 7-Bit Device Address
 * @param  reg: Register address, only needs to be written when writeReg is true
 * @param  *buf: When isWrite is true, it represents the written buf, otherwise it is the read data buffer
 * @param  len: When isWrite is true, it indicates the length of data written , otherwise it is the data read length
 * @param  writeReg: writeReg is the register address that needs to be written, otherwise the register address is not written
 * @param  isWrite: Data write and read direction, true means write, false means read
 * @retval True means the device is executed successfully, false means it fails
 */
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
    // Set GPIO mode
    case SensorCommCustomHal::OP_PINMODE: {
        uint8_t pin = reinterpret_cast<uintptr_t>(param1);
        uint8_t mode = reinterpret_cast<uintptr_t>(param2);
        pinMode(pin, mode);
    }
    break;
    // Set GPIO level
    case SensorCommCustomHal::OP_DIGITALWRITE: {
        uint8_t pin = reinterpret_cast<uintptr_t>(param1);
        uint8_t level = reinterpret_cast<uintptr_t>(param2);
        digitalWrite(pin, level);
    }
    break;
    // Read GPIO level
    case SensorCommCustomHal::OP_DIGITALREAD: {
        uint8_t pin = reinterpret_cast<uintptr_t>(param1);
        return digitalRead(pin);
    }
    break;
    // Get the current running milliseconds
    case SensorCommCustomHal::OP_MILLIS:
        return millis();

    // Delay in milliseconds
    case SensorCommCustomHal::OP_DELAY: {
        if (param1) {
            uint32_t ms = reinterpret_cast<uintptr_t>(param1);
            delay(ms);
        }
    }
    break;
    // Delay in microseconds
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

void TouchDrvDigitalWrite(uint8_t gpio, uint8_t level)
{
    if (gpio & 0x80) {
        extension.digitalWrite(gpio & 0x7F, level);
    } else {
        digitalWrite(gpio, level);
    }
}

uint8_t TouchDrvDigitalRead(uint8_t gpio)
{
    if (gpio & 0x80) {
        return extension.digitalRead(gpio & 0x7F);
    } else {
        return digitalRead(gpio);
    }
}

void TouchDrvPinMode(uint8_t gpio, uint8_t mode)
{
    if (gpio & 0x80) {
        extension.pinMode(gpio & 0x7F, mode);
    } else {
        pinMode(gpio, mode);
    }
}

bool setupTouchDrv()
{
    // Add the highest bit to indicate that the GPIO extension is used, not the ESP's GPIO
    const uint8_t touch_reset_pin = tp_reset | 0x80;
    const uint8_t touch_irq_pin = TOUCH_IRQ;
    bool result = false;

    touchDrv = new TouchDrvCSTXXX();
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(i2c_wr_function, hal_callback, CST816_SLAVE_ADDRESS);
    if (result) {
        const char *model = touchDrv->getModelName();
        Serial.print("Successfully initialized "); Serial.print(model);
        Serial.print(",using "); Serial.print(model); Serial.println(" Driver!");
        return true;
    }
    delete touchDrv;

    touchDrv = new TouchDrvGT911();
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(i2c_wr_function, hal_callback, GT911_SLAVE_ADDRESS_L);
    if (result) {
        const char *model = touchDrv->getModelName();
        Serial.print("Successfully initialized "); Serial.print(model);
        Serial.print(",using "); Serial.print(model); Serial.println(" Driver!");
        return true;
    }
    delete touchDrv;

    touchDrv = new TouchDrvFT6X36();
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(i2c_wr_function, hal_callback, FT3267_SLAVE_ADDRESS);
    if (result) {
        TouchDrvFT6X36 *tmp = static_cast<TouchDrvFT6X36 *>(touchDrv);
        tmp->interruptTrigger();
        const char *model = touchDrv->getModelName();
        Serial.print("Successfully initialized "); Serial.print(model);
        Serial.print(",using "); Serial.print(model); Serial.println(" Driver!");
        return true;
    }
    delete touchDrv;

    Serial.println("Unable to find touch device.");

    touchDrv = NULL;

    return false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Start!");

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

    SensorWireHelper::dumpDevices(Wire);

    // Use unspecified address to test the address discovery function.
    // T-RGB XL9555 uses 0X20 device address
    if (!extension.begin(i2c_wr_function, XL9555_UNKOWN_ADDRESS)) {
        Serial.println("Failed to find XL9555 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    if (!setupTouchDrv()) {
        while (1) {
            delay(3000);
        }
    }

}

void loop()
{
    if (touchDrv->isPressed()) {
        uint8_t touched = touchDrv->getPoint(x, y, touchDrv->getSupportTouchPoint());
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




