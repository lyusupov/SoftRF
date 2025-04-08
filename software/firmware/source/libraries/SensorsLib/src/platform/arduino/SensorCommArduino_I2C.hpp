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
 * @file      SensorCommArduino_I2C.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */

#pragma once

#include "../SensorCommBase.hpp"

#ifdef ARDUINO

class SensorCommI2C : public SensorCommBase
{
public:
    SensorCommI2C(TwoWire &wire, uint8_t addr, int sda, int scl, SensorHal *hal = nullptr) : hal(hal), wire(wire), addr(addr), sda(sda), scl(scl), sendStopFlag(true) {}

    bool init() override
    {
        setPins();
        wire.begin();
        return true;
    }

    void deinit() override
    {
        // Do not destroy wire bus
        // wire.end();
    }

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
        // log_d("ADDR:0x%02X REG:0x%02X, LEN:%u", addr, reg, len);
        wire.beginTransmission(addr);
        wire.write(reg);
        if (buf && len > 0) {
            wire.write(buf, len);
        }
        if (wire.endTransmission() == 0) {
            return 0;
        } else {
            return -1;
        }
    }

    int writeBuffer(uint8_t *buffer, size_t len)
    {
        if (!buffer || len == 0)return -1;
        wire.beginTransmission(addr);
        wire.write(buffer, len);
        if (wire.endTransmission() == 0) {
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
        // log_d("ADDR:0x%02X REG:0x%02X, LEN:%u", addr, reg, len);
        wire.beginTransmission(addr);
        wire.write(reg);
        wire.endTransmission(sendStopFlag);
        wire.requestFrom(addr, static_cast<uint8_t>(len));
        return wire.readBytes(buf, len) == len ? 0 : -1;
    }

    int writeThenRead(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len) override
    {
        wire.beginTransmission(addr);
        wire.write(write_buffer, write_len);
        if (wire.endTransmission(sendStopFlag) != 0) {
            return -1;
        }
        wire.requestFrom(addr, read_len);
        return wire.readBytes(read_buffer, read_len) == read_len ? 0 : -1;
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
            sendStopFlag = pdat->getParams();
            break;
        default:
            break;
        }
    }

private:

    void setPins()
    {
        if (sda != -1 && scl != -1) {
#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_ESP32)
            wire.setPins(sda, scl);
#elif (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_STM32)) && !defined(ARDUINO_ARCH_MBED)
            wire.end();
#if !defined(ARDUINO_ARCH_MBED)
            wire.setSDA(sda);
            wire.setSCL(scl);
#endif  /*ARDUINO_ARCH_MBED*/

#elif defined(ARDUINO_ARCH_MBED)
            //No thing.
#else
#warning "Wire custom GPIO mapping function is not implemented"
#endif
        }
    }
    SensorHal *hal;
    TwoWire &wire;
    uint8_t addr;
    int sda;
    int scl;
    bool sendStopFlag;
};

#endif  //*ARDUINO
