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
 * @file      XPowersLibInterface.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-08-28
 *
 */
#if defined(ARDUINO)
#include <Arduino.h>
#endif

#include "XPowersLibInterface.hpp"


bool XPowersLibInterface::isChannelAvailable(uint8_t channel)
{
    if (__chipModel == XPOWERS_AXP192) {
        switch (channel) {
        case XPOWERS_DCDC1:
        case XPOWERS_DCDC2:
        case XPOWERS_DCDC3:
        case XPOWERS_LDO2:
        case XPOWERS_LDO3:
        case XPOWERS_LDOIO:
            return true;
        default:
            return false;
        }
    } else if (__chipModel == XPOWERS_AXP202) {

        switch (channel) {
        case XPOWERS_DCDC2:
        case XPOWERS_DCDC3:
        case XPOWERS_LDO2:
        case XPOWERS_LDO3:
        case XPOWERS_LDO4:
        case XPOWERS_LDO5:
            return true;
        default:
            return false;
        }

    } else if (__chipModel == XPOWERS_AXP2101) {
        switch (channel) {
        case XPOWERS_DCDC1:
        case XPOWERS_DCDC2:
        case XPOWERS_DCDC3:
        case XPOWERS_DCDC4:
        case XPOWERS_DCDC5:
        case XPOWERS_ALDO1:
        case XPOWERS_ALDO2:
        case XPOWERS_ALDO3:
        case XPOWERS_ALDO4:
        case XPOWERS_BLDO1:
        case XPOWERS_BLDO2:
        case XPOWERS_VBACKUP:
        case XPOWERS_CPULDO:
            return true;
        default:
            // DLDO is not available, will also return false
            return false;
        }
    }
    return false;
}

void XPowersLibInterface::setProtectedChannel(uint8_t channel)
{
    __protectedMask |= _BV(channel);
}

void XPowersLibInterface::setUnprotectChannel(uint8_t channel)
{
    __protectedMask &= (~_BV(channel));
}

bool XPowersLibInterface::getProtectedChannel(uint8_t channel)
{
    return __protectedMask & _BV(channel);
}


uint16_t XPowersLibInterface::getVbusVoltage()
{
    return 0;
}
