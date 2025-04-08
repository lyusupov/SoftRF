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
 * @file      PCF8563Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-09
 *
 */

#pragma once

#include <stdint.h>


class PCF8563Constants
{
protected:
    // Unique I2C device address
    static constexpr uint8_t PCF8563_SLAVE_ADDRESS = 0x51;

    // Register addresses
    static constexpr uint8_t STAT1_REG = 0x00;
    static constexpr uint8_t STAT2_REG = 0x01;
    static constexpr uint8_t SEC_REG = 0x02;
    static constexpr uint8_t MIN_REG = 0x03;
    static constexpr uint8_t HR_REG = 0x04;
    static constexpr uint8_t DAY_REG = 0x05;
    static constexpr uint8_t WEEKDAY_REG = 0x06;
    static constexpr uint8_t MONTH_REG = 0x07;
    static constexpr uint8_t YEAR_REG = 0x08;
    static constexpr uint8_t ALRM_MIN_REG = 0x09;
    static constexpr uint8_t SQW_REG = 0x0D;
    static constexpr uint8_t TIMER1_REG = 0x0E;
    static constexpr uint8_t TIMER2_REG = 0x0F;

    // Mask values
    static constexpr uint8_t VOL_LOW_MASK = 0x80;
    static constexpr uint8_t MINUTES_MASK = 0x7F;
    static constexpr uint8_t HOUR_MASK = 0x3F;
    static constexpr uint8_t WEEKDAY_MASK = 0x07;
    static constexpr uint8_t CENTURY_MASK = 0x80;
    static constexpr uint8_t DAY_MASK = 0x3F;
    static constexpr uint8_t MONTH_MASK = 0x1F;
    static constexpr uint8_t TIMER_CTL_MASK = 0x03;

    // Alarm and Timer flags
    static constexpr uint8_t ALARM_AF = 0x08;
    static constexpr uint8_t TIMER_TF = 0x04;
    static constexpr uint8_t ALARM_AIE = 0x02;
    static constexpr uint8_t TIMER_TIE = 0x01;
    static constexpr uint8_t TIMER_TE = 0x80;
    static constexpr uint8_t TIMER_TD10 = 0x03;

    // Other constants
    static constexpr uint8_t NO_ALARM = 0xFF;
    static constexpr uint8_t ALARM_ENABLE = 0x80;
    static constexpr uint8_t CLK_ENABLE = 0x80;
};