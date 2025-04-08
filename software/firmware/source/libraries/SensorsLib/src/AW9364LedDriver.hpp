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
 * @file      AW9364LedDriver.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-11-22
 *
 */
#pragma once

#include "SensorLib.h"
#include "VirtualGpio.hpp"

constexpr uint8_t MAX_BRIGHTNESS_STEPS = 16;

class AW9364LedDriver
{
public:

    void begin(uint8_t pin)
    {
        _driver_pin = pin;
        _virtualGpio = NULL;
        this->_pinMode(_driver_pin, OUTPUT);
        this->_digitalWrite(_driver_pin, LOW);
    }

    void begin(VirtualGpio *handler, uint8_t pin)
    {
        _virtualGpio = handler;
        _driver_pin = pin;
        this->_pinMode(_driver_pin, OUTPUT);
        this->_digitalWrite(_driver_pin, LOW);
    }

    void setBrightness(uint8_t value)
    {
        if (_brightness == value) {
            return;
        }
        if (value > 16) {
            value = 16;
        }
        if (value == 0) {
            this->_digitalWrite(_driver_pin, 0);
            _brightness = 0;
            return;
        }
        if (_brightness == 0) {
            this->_digitalWrite(_driver_pin, 1);
            _brightness = MAX_BRIGHTNESS_STEPS;
        }
        int from = MAX_BRIGHTNESS_STEPS - _brightness;
        int to = MAX_BRIGHTNESS_STEPS - value;
        int num = (MAX_BRIGHTNESS_STEPS + to - from) % MAX_BRIGHTNESS_STEPS;

        uint32_t frequency = 0;
        if (_virtualGpio) {
            // If use I / O port Expander for driving, need to increase the I2C communication rate. 
            // The test uses XINLUDA XL9555. The manual advertises the maximum communication rate as 400K. 
            // In fact, it can be driven normally at 1000K.
            // https://item.szlcsc.com/datasheet/XL9555/639796.html
            frequency = _virtualGpio->getClock();
            _virtualGpio->setClock(1000000UL);
        }

        for (int i = 0; i < num; i++) {
            this->_digitalWrite(_driver_pin, 0);
            this->_digitalWrite(_driver_pin, 1);
        }

        if (_virtualGpio) {
            // Restore the original communication speed to avoid disrupting 
            // the communication of other devices that do not support high-speed rates
            _virtualGpio->setClock(frequency);
        }
        _brightness = value;
    }


    uint8_t getBrightness() const
    {
        return _brightness;
    }

protected:

    void _pinMode(uint8_t pin, uint8_t mode)
    {
        if (_virtualGpio) {
            _virtualGpio->pinMode(pin, mode);
        } else {
            pinMode(pin, mode);
        }
    }

    void _digitalWrite(uint8_t pin, uint8_t value)
    {
        if (_virtualGpio) {
            _virtualGpio->digitalWrite(pin, value);
        } else {
            digitalWrite(pin, value);
        }
    }

private:
    VirtualGpio *_virtualGpio = NULL;
    uint8_t _driver_pin;
    uint8_t _brightness = 0;
};