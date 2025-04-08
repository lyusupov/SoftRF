/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      LTR533Constants.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-09
 */
#pragma once

#include <stdint.h>


class LTR553Constants
{
protected:
    // Unique I2C device address
    static constexpr uint8_t LTR553_SLAVE_ADDRESS = 0x23;   

    // Registers
    static constexpr uint8_t REG_ALS_CONTR = 0x80;
    static constexpr uint8_t REG_PS_CONTR = 0x81;
    static constexpr uint8_t REG_PS_LED = 0x82;
    static constexpr uint8_t REG_PS_N_PULSES = 0x83;
    static constexpr uint8_t REG_PS_MEAS_RATE = 0x84;
    static constexpr uint8_t REG_ALS_MEAS_RATE = 0x85;
    static constexpr uint8_t REG_PART_ID = 0x86;
    static constexpr uint8_t REG_MANUFAC_ID = 0x87;
    static constexpr uint8_t REG_ALS_DATA_CH1_0 = 0x88;
    static constexpr uint8_t REG_ALS_DATA_CH1_1 = 0x89;
    static constexpr uint8_t REG_ALS_DATA_CH0_0 = 0x8A;
    static constexpr uint8_t REG_ALS_DATA_CH0_1 = 0x8B;
    static constexpr uint8_t REG_ALS_PS_STATUS = 0x8C;
    static constexpr uint8_t REG_PS_DATA_0 = 0x8D;
    static constexpr uint8_t REG_PS_DATA_1 = 0x8E;
    static constexpr uint8_t REG_INTERRUPT = 0x8F;
    static constexpr uint8_t REG_PS_THRES_UP_0 = 0x90;
    static constexpr uint8_t REG_PS_THRES_UP_1 = 0x91;
    static constexpr uint8_t REG_PS_THRES_LOW_0 = 0x92;
    static constexpr uint8_t REG_PS_THRES_LOW_1 = 0x93;
    static constexpr uint8_t REG_PS_OFFSET_1 = 0x94;
    static constexpr uint8_t REG_PS_OFFSET_0 = 0x95;
    static constexpr uint8_t REG_ALS_THRES_UP_0 = 0x97;
    static constexpr uint8_t REG_ALS_THRES_UP_1 = 0x98;
    static constexpr uint8_t REG_ALS_THRES_LOW_0 = 0x99;
    static constexpr uint8_t REG_ALS_THRES_LOW_1 = 0x9A;
    static constexpr uint8_t REG_INTERRUPT_PERSIST = 0x9E;

    // Default Manufacturer ID
    static constexpr uint8_t LTR553_DEFAULT_MAN_ID = 0x05;
};