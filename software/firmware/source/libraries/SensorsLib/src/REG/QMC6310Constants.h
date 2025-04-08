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
 * @file      QMC6310Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */

#pragma once

#include <stdint.h>



class QMC6310Constants
{
protected:


    // Register addresses
    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_LSB_DX = 0x01;
    static constexpr uint8_t REG_MSB_DX = 0x02;
    static constexpr uint8_t REG_LSB_DY = 0x03;
    static constexpr uint8_t REG_MSB_DY = 0x04;
    static constexpr uint8_t REG_LSB_DZ = 0x05;
    static constexpr uint8_t REG_MSB_DZ = 0x06;
    static constexpr uint8_t REG_STAT = 0x09;
    static constexpr uint8_t REG_CMD1 = 0x0A;
    static constexpr uint8_t REG_CMD2 = 0x0B;
    static constexpr uint8_t REG_SIGN = 0x29;
    static constexpr uint8_t QMC6310_CHIP_ID = 0x80;
};



