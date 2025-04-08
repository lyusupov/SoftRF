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
 * @file      SensorCommEnhanced.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#include "SensorCommBase.hpp"
#include <vector>

class EnhancedSensorCommBase : public SensorCommBase
{
private:
    template<typename T>
    std::vector<uint8_t> splitRegister(T reg)
    {
        std::vector<uint8_t> result;
        const size_t size = sizeof(T);
        for (size_t i = 0; i < size; ++i) {
            result.push_back(static_cast<uint8_t>(reg >> (8 * (size - 1 - i))));
        }
        return result;
    }

    template<typename T>
    int readRegisterImpl(T reg, uint8_t *buf, size_t len)
    {
        std::vector<uint8_t> splitReg = splitRegister(reg);
        return readRegister(splitReg[0], buf, len);
    }


    template<typename T>
    int writeRegisterImpl(T reg, uint8_t val)
    {
        std::vector<uint8_t> splitReg = splitRegister(reg);
        return writeRegister(splitReg[0], val);
    }


    template<typename T>
    int writeRegisterImpl(T reg, uint8_t *buf, size_t len)
    {
        std::vector<uint8_t> splitReg = splitRegister(reg);
        return writeRegister(splitReg[0], buf, len);
    }


    template<typename T>
    int writeRegisterImpl(T reg, uint8_t norVal, uint8_t orVal)
    {
        std::vector<uint8_t> splitReg = splitRegister(reg);
        return writeRegister(splitReg[0], norVal, orVal);
    }


public:
    template<typename T>
    int readRegister(T reg)
    {
        std::vector<uint8_t> splitReg = splitRegister(reg);
        return readRegister(splitReg[0]);
    }


    template<typename T>
    int readRegister(T reg, uint8_t *buf, size_t len)
    {
        return readRegisterImpl(reg, buf, len);
    }


    template<typename T>
    int writeRegister(T reg, uint8_t val)
    {
        return writeRegisterImpl(reg, val);
    }


    template<typename T>
    int writeRegister(T reg, uint8_t *buf, size_t len)
    {
        return writeRegisterImpl(reg, buf, len);
    }


    template<typename T>
    int writeRegister(T reg, uint8_t norVal, uint8_t orVal)
    {
        return writeRegisterImpl(reg, norVal, orVal);
    }
};

