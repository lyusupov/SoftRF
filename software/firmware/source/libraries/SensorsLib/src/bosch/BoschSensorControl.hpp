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
 * @file      BoschSensorControl.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-22
 *
 */
#pragma once

#include <stdint.h>
#include <stdio.h>
#ifdef ARDUINO
#include <Stream.h>
#endif

class SensorBHI260APControl
{
public:
    bool wakeUpFIFOEnabled;
    bool nonWakeUpFIFOEnabled;
    bool statusFIFOEnabled;
    bool debuggingEnabled;
    bool faultEnabled;
    bool interruptIsActiveLow;
    bool interruptIsPulseTriggered;
    bool interruptPinDriveIsOpenDrain;

    SensorBHI260APControl() : wakeUpFIFOEnabled(true), nonWakeUpFIFOEnabled(true), statusFIFOEnabled(true),
        debuggingEnabled(true), faultEnabled(true), interruptIsActiveLow(false),
        interruptIsPulseTriggered(false), interruptPinDriveIsOpenDrain(false) {}

#if defined(ARDUINO) && !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    void print(Stream &stream)
    {
        stream.printf("Host interrupt control\n");
        stream.printf("-- Wake up FIFO :%s\n", (wakeUpFIFOEnabled ? "enabled" : "disabled"));
        stream.printf("-- Non wake up FIFO :%s\n", (nonWakeUpFIFOEnabled ? "enabled" : "disabled"));
        stream.printf("-- Status FIFO :%s\n", (statusFIFOEnabled ? "enabled" : "disabled"));
        stream.printf("-- Debugging :%s\n", (debuggingEnabled ? "enabled" : "disabled"));
        stream.printf("-- Fault :%s\n", (faultEnabled ? "enabled" : "disabled"));
        stream.printf("-- Interrupt is :%s\n", (interruptIsActiveLow ? "active low" : "active high"));
        stream.printf("-- Interrupt is :%s triggered.\n", (interruptIsPulseTriggered ? "pulse" : "level"));
        stream.printf("-- Interrupt pin drive is :%s\n", (interruptPinDriveIsOpenDrain ? "open drain" : "push-pull"));
    }
#else
    void print()
    {
        printf("Host interrupt control\n");
        printf("-- Wake up FIFO :%s\n", (wakeUpFIFOEnabled ? "enabled" : "disabled"));
        printf("-- Non wake up FIFO :%s\n", (nonWakeUpFIFOEnabled ? "enabled" : "disabled"));
        printf("-- Status FIFO :%s\n", (statusFIFOEnabled ? "enabled" : "disabled"));
        printf("-- Debugging :%s\n", (debuggingEnabled ? "enabled" : "disabled"));
        printf("-- Fault :%s\n", (faultEnabled ? "enabled" : "disabled"));
        printf("-- Interrupt is :%s\n", (interruptIsActiveLow ? "active low" : "active high"));
        printf("-- Interrupt is :%s triggered.\n", (interruptIsPulseTriggered ? "pulse" : "level"));
        printf("-- Interrupt pin drive is :%s\n", (interruptPinDriveIsOpenDrain ? "open drain" : "push-pull"));
    }
#endif
};

