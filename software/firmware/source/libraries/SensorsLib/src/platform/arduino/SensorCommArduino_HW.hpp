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
 * @file      SensorCommArduino_HW.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#include "../SensorCommBase.hpp"

#ifdef ARDUINO

class HalArduino : public SensorHal
{
public:
    void pinMode(uint8_t pin, uint8_t mode)
    {
        if (modeCallback) {
            modeCallback(pin, mode);
        } else {
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_ZEPHYR)
            ::pinMode(pin, static_cast<PinMode>(mode));
#else
            ::pinMode(pin, mode);
#endif
        }
    }

    void digitalWrite(uint8_t pin, uint8_t level)
    {
        if (writeCallback) {
            writeCallback(pin, level);
        } else {
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_ZEPHYR)
            ::digitalWrite(pin, static_cast<PinStatus>(level));
#else
            ::digitalWrite(pin, level);
#endif
        }
    }

    uint8_t digitalRead(uint8_t pin)
    {
        if (readCallback) {
            return readCallback(pin);
        }
        return ::digitalRead(pin);
    }

    uint32_t millis()
    {
        return ::millis();
    }

    void delay(uint32_t ms)
    {
        ::delay(ms);;
    }

    void delayMicroseconds(uint32_t us)
    {
        ::delayMicroseconds(us);
    }
};

#endif  //*ARDUINO
