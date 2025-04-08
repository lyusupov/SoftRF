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
 * @file      PCF85063Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-07
 *
 */

#pragma once

#include <stdint.h>

class PCF85063Constants
{
protected:
    // Unique I2C device address
    static constexpr uint8_t PCF85063_SLAVE_ADDRESS = 0x51;

    // Register addresses
    static constexpr uint8_t PCF85063_CTRL1_REG = 0x00;
    static constexpr uint8_t PCF85063_CTRL2_REG = 0x01;
    static constexpr uint8_t PCF85063_OFFSET_REG = 0x02;
    static constexpr uint8_t PCF85063_RAM_REG = 0x03;
    static constexpr uint8_t PCF85063_SEC_REG = 0x04;
    static constexpr uint8_t PCF85063_MIN_REG = 0x05;
    static constexpr uint8_t PCF85063_HR_REG = 0x06;
    static constexpr uint8_t PCF85063_DAY_REG = 0x07;
    static constexpr uint8_t PCF85063_WEEKDAY_REG = 0x08;
    static constexpr uint8_t PCF85063_MONTH_REG = 0x09;
    static constexpr uint8_t PCF85063_YEAR_REG = 0x0A;
    static constexpr uint8_t PCF85063_ALRM_SEC_REG = 0x0B;
    static constexpr uint8_t PCF85063_ALRM_MIN_REG = 0x0C;
    static constexpr uint8_t PCF8563_ALRM_HR_REG = 0x0D;
    static constexpr uint8_t PCF85063_ALRM_DAY_REG = 0x0E;
    static constexpr uint8_t PCF85063_ALRM_WEEK_REG = 0x0F;
    static constexpr uint8_t PCF85063_TIMER_VAL_REG = 0x10;
    static constexpr uint8_t PCF85063_TIMER_MD_REG = 0x11;

    // Mask values
    static constexpr uint8_t PCF85063_CTRL1_TEST_EN_MASK = (1 << 7u);
    static constexpr uint8_t PCF85063_CTRL1_CLOCK_EN_MASK = (1 << 5u);
    static constexpr uint8_t PCF85063_CTRL1_SOFTRST_EN_MASK = (1 << 4u);
    static constexpr uint8_t PCF85063_CTRL1_CIE_EN_MASK = (1 << 2u);
    static constexpr uint8_t PCF85063_CTRL1_HOUR_FORMAT_12H_MASK = (1 << 1u);

    // Other constants
    static constexpr uint8_t PCF85063_NO_ALARM = 0xFF;
    static constexpr uint8_t PCF85063_ALARM_ENABLE = 0x80;
};