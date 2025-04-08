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
 * @file      SensorCommCustom.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#include "SensorCommBase.hpp"

class SensorCommCustom : public SensorCommBase
{
public:
    using CustomCallback = bool(*)(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, bool writeReg, bool isWrite);

    SensorCommCustom(CustomCallback callback, uint8_t addr) : customCallback(callback), addr(addr) {}

    bool init() override
    {
        return true;
    }

    void deinit() override {}

    int writeRegister(const uint8_t reg, uint8_t val) override
    {
        return writeRegister(reg, &val, 1);
    }

    int writeRegister(const uint8_t reg, uint8_t norVal, uint8_t orVal) override
    {
        int val = readRegister(reg);
        if (val < 0) {
            return -1;
        }
        val &= norVal;
        val |= orVal;
        return writeRegister(reg, reinterpret_cast<uint8_t *>(&val), 1);
    }

    int writeRegister(const uint8_t reg, uint8_t *buf, size_t len) override
    {
        if (customCallback(addr, reg, buf, len, true, true)) {
            return 0;
        } else {
            return -1;
        }
    }

    int writeBuffer(uint8_t *buffer, size_t len)
    {
        if (customCallback(addr, 0x00, buffer, len, false, true)) {
            return 0;
        } else {
            return -1;
        }
    }

    int readRegister(const uint8_t reg) override
    {
        uint8_t value = 0x00;
        if (readRegister(reg, &value, 1) < 0) {
            return -1;
        }
        return value;
    }

    int readRegister(const uint8_t reg, uint8_t *buf, size_t len) override
    {
        if (customCallback(addr, reg, buf, len, true, false)) {
            return 0;
        } else {
            return -1;
        }
    }

    int writeThenRead(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len) override
    {
        writeBuffer((uint8_t *)write_buffer, write_len);
        if (customCallback(addr, 0x00, read_buffer, read_len, false, false)) {
            return 0;
        } else {
            return -1;
        }
    }

    bool setRegisterBit(const uint8_t reg, uint8_t bit) override
    {
        uint8_t value = readRegister(reg);
        value |= (1 << bit);
        return writeRegister(reg, reinterpret_cast<uint8_t *>(&value), 1) == 0;
    }

    bool clrRegisterBit(const uint8_t reg, uint8_t bit) override
    {
        uint8_t value = readRegister(reg);
        value &= ~(1 << bit);
        return writeRegister(reg, reinterpret_cast<uint8_t *>(&value), 1) == 0;
    }

    bool getRegisterBit(const uint8_t reg, uint8_t bit) override
    {
        uint8_t value = readRegister(reg);
        return (value & (1 << bit)) != 0;
    }

    void setParams(const CommParamsBase &params) override
    {
#if defined(__cpp_rtti)
        const auto pdat = dynamic_cast<const I2CParam *>(&params);
#else
        const auto pdat = static_cast<const I2CParam *>(&params);
#endif
        uint8_t type = pdat->getType();
        switch (type) {
        case I2CParam::I2C_SET_ADDR:
            addr = pdat->getParams();
            break;
        case I2CParam::I2C_SET_FLAG:
            // TODO:
            // sendStopFlag = pdat->getParams();
            break;
        default:
            break;
        }
    }
private:
    CustomCallback customCallback;
    uint8_t addr;
};
