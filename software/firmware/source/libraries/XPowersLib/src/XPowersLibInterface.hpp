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
 * @file      XPowersLibInterface.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-08-28
 *
 */
#pragma once

#include <stdint.h>
#include "XPowersParams.hpp"


class XPowersLibInterface
{

public:


    //TODO:Unified interface functions and parameters

    XPowersLibInterface() {};

    virtual ~XPowersLibInterface() {}

    virtual bool enablePowerOutput(uint8_t channel) = 0;

    virtual bool disablePowerOutput(uint8_t channel) = 0;

    virtual uint64_t getIrqStatus();

    virtual void clearIrqStatus();

    virtual bool enableIRQ(uint64_t opt);

    virtual bool disableIRQ(uint64_t opt);

    virtual uint8_t getChipID();

    virtual uint16_t getBattVoltage();

    virtual int getBatteryPercent(void);

    /*
    virtual bool enableVbusVoltageMeasure(void);
    virtual bool disableVbusVoltageMeasure(void);
    virtual bool enableSystemVoltageMeasure(void);
    virtual bool disableSystemVoltageMeasure(void);
    virtual bool enableTemperatureMeasure(void);
    virtual bool disableTemperatureMeasure(void);
    virtual bool enableBattVoltageMeasure(void);
    virtual bool disableBattVoltageMeasure(void);
    virtual bool enableBattDetection(void);
    virtual bool disableBattDetection(void);
    */


};

