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
 * @file      XL9555Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-17
 *
 */
#pragma once

#include <stdint.h>

// A0, A1, A2 connect to GND
#define XL9555_UNKOWN_ADDRESS        (0xFF)
#define XL9555_SLAVE_ADDRESS0        (0x20)
#define XL9555_SLAVE_ADDRESS1        (0x21)
#define XL9555_SLAVE_ADDRESS2        (0x22)
#define XL9555_SLAVE_ADDRESS3        (0x23)
#define XL9555_SLAVE_ADDRESS4        (0x24)
#define XL9555_SLAVE_ADDRESS5        (0x25)
#define XL9555_SLAVE_ADDRESS6        (0x26)
#define XL9555_SLAVE_ADDRESS7        (0x27)


class XL95xxConstants
{
protected:
    static constexpr uint8_t XL9555_CTRL_INP0   = (0x00);  // Input Port 0  /R
    static constexpr uint8_t XL9555_CTRL_INP1   = (0x01);  // Input Port 1  /R
    static constexpr uint8_t XL9555_CTRL_OUTP0  = (0x02);  // Output Port 0 /RW
    static constexpr uint8_t XL9555_CTRL_OUTP1  = (0x03);  // Output Port 1 /RW
    static constexpr uint8_t XL9555_CTRL_PIP0   = (0x04);  // Polarity Inversion Port 0 /RW
    static constexpr uint8_t XL9555_CTRL_PIP1   = (0x05);  // Polarity Inversion Port 1 /RW
    static constexpr uint8_t XL9555_CTRL_CFG0   = (0x06);  // Configuration Port 0 /RW
    static constexpr uint8_t XL9555_CTRL_CFG1   = (0x07);  // Configuration Port 1 /RW
    static constexpr uint8_t XL9555_MAX_PIN     = (15);
};
