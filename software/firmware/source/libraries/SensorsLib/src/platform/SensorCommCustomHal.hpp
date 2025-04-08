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
 * @file      SensorCommCustomHal.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#include "SensorCommBase.hpp"

class SensorCommCustomHal : public SensorHal
{
public:
    enum Operation {
        OP_PINMODE,
        OP_DIGITALWRITE,
        OP_DIGITALREAD,
        OP_MILLIS,
        OP_DELAY,
        OP_DELAYMICROSECONDS,
    };

    using CustomHalCallback = uint32_t (*)(Operation op, void *param1, void *param2);

    SensorCommCustomHal(CustomHalCallback callback) : halCallback(callback) {}

    void pinMode(uint8_t pin, uint8_t mode) override
    {
        if (modeCallback) {
            modeCallback(pin, mode);
        } else if (halCallback) {
            halCallback(OP_PINMODE, reinterpret_cast<void *>(pin), reinterpret_cast<void *>(mode));
        }
    }

    void digitalWrite(uint8_t pin, uint8_t level) override
    {
        if (writeCallback) {
            writeCallback(pin, level);
        } else if (halCallback) {
            halCallback(OP_DIGITALWRITE, reinterpret_cast<void *>(pin), reinterpret_cast<void *>(level));
        }
    }

    uint8_t digitalRead(uint8_t pin) override
    {
        if (readCallback) {
            return readCallback(pin);
        } else if (halCallback) {
            return halCallback(OP_DIGITALREAD, reinterpret_cast<void *>(pin), nullptr);
        }
        return 0;
    }

    uint32_t millis()override
    {
        if (halCallback) {
            return halCallback(OP_MILLIS, nullptr, nullptr);
        }
        return 0;
    }

    void delay(uint32_t ms) override
    {
        if (halCallback) {
            halCallback(OP_DELAY, reinterpret_cast<void *>(ms), nullptr);
        }
    }

    void delayMicroseconds(uint32_t us)override
    {
        if (halCallback) {
            halCallback(OP_DELAYMICROSECONDS, reinterpret_cast<void *>(us), nullptr);
        }
    }
private:
    CustomHalCallback halCallback;
};