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
 * @file      CM32181Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-14
 *
 */

#pragma once

#include <stdint.h>

// CM32181 slave address can be 0x10 or 0x48, determined by pin ADDR configuration
#define CM32181_ADDR_PRIMARY            (0x10)
#define CM32181_ADDR_SECONDARY          (0x48)
#define CM32181_SLAVE_ADDRESS           (0x10)  //Compatible with previous version definitions

class CM32181Constants
{
protected:
    // CM32181 registers
    static constexpr uint8_t REG_ALS_CONF = 0x00;
    static constexpr uint8_t REG_ALS_THDH = 0x01;
    static constexpr uint8_t REG_ALS_THDL = 0x02;
    static constexpr uint8_t REG_ALS_PSM = 0x03;
    static constexpr uint8_t REG_ALS_DATA = 0x04;
    static constexpr uint8_t REG_ALS_STATUS = 0x06;
    static constexpr uint8_t REG_ID = 0x07;
    static constexpr uint8_t CM32181_CHIP_ID = 0x81;
};












