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
 * @file      SensorCommBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once
#include "SensorLib.h"
#include <memory>

typedef struct {
    uint8_t byte0 : 8;
    uint8_t byte1 : 8;
    uint8_t byte2 : 8;
    uint8_t byte3 : 8;
} ByteStruct;

typedef union {
    uint32_t value;
    ByteStruct bytes;
    uint8_t byte_array[4];
} ByteUnion;

#if __cplusplus == 201103L
namespace std
{
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}
#endif


template <typename T, size_t N>
constexpr size_t arraySize(const T (&arr)[N])
{
    return N;
}


class CommParamsBase
{
public:
    virtual ~CommParamsBase() = default;
};

class I2CParam : public CommParamsBase
{
public:
    enum I2CParamType {
        I2C_SET_ADDR,
        I2C_SET_FLAG,
        I2C_SET_CLOCK,
        I2C_SET_WRITE_DELAY_US,
        I2C_SET_READ_DELAY_US,
    };

    I2CParam(I2CParamType type, uint32_t params) : type(type), params(params) {}

    uint32_t getParams() const
    {
        return params;
    }
    uint8_t getType() const
    {
        return type;
    }

protected:
    enum I2CParamType type;
    uint32_t params;
};

#if !defined(ARDUINO)

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define SPI_LSB 0
#define SPI_MSB 1

class SPISettings
{
public:
    SPISettings() : clock(1000000), bitOrder(SPI_MSB), dataMode(SPI_MODE0) {}
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) :
        clock(clock), bitOrder(bitOrder), dataMode(dataMode) {}
    uint32_t clock;
    uint8_t  bitOrder;
    uint8_t  dataMode;
};
#endif


class SPIParam : public CommParamsBase
{
private:
    SPISettings setting;
public:
    SPIParam(SPISettings setting) : setting(setting) {}
    SPISettings getSetting() const
    {
        return setting;
    }
};

class SensorCommBase
{
public:
    virtual bool init() = 0;
    virtual void deinit() = 0;

    virtual int readRegister(const uint8_t reg) = 0;
    virtual int readRegister(const uint8_t reg, uint8_t *buf, size_t len) = 0;

    virtual int writeRegister(const uint8_t reg, uint8_t val) = 0;
    virtual int writeRegister(const uint8_t reg, uint8_t *buf, size_t len) = 0;
    virtual int writeRegister(const uint8_t reg, uint8_t norVal, uint8_t orVal) = 0;
    virtual int writeBuffer(uint8_t *buffer, size_t len) = 0;

    virtual int writeThenRead(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len) = 0;

    virtual bool setRegisterBit(const uint8_t reg, uint8_t bit) = 0;
    virtual bool clrRegisterBit(const uint8_t reg, uint8_t bit) = 0;
    virtual bool getRegisterBit(const uint8_t reg, uint8_t bit) = 0;

    virtual void setParams(const CommParamsBase &params) = 0;
    virtual ~SensorCommBase() = default;
};

class SensorHalCustom
{
public:
    using CustomWrite = void(*)(uint8_t pin, uint8_t level);
    using CustomRead  = uint8_t(*)(uint8_t pin);
    using CustomMode  = void(*)(uint8_t pin, uint8_t mode);

    SensorHalCustom() : writeCallback(nullptr), readCallback(nullptr), modeCallback(nullptr) {}

    void setCustomWrite(CustomWrite callback)
    {
        writeCallback = callback;
    }

    void setCustomRead(CustomRead callback)
    {
        readCallback = callback;
    }

    void setCustomMode(CustomMode callback)
    {
        modeCallback = callback;
    }

protected:
    CustomWrite writeCallback;
    CustomRead readCallback;
    CustomMode modeCallback;
};

class SensorHal : public SensorHalCustom
{
public:
    virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
    virtual void digitalWrite(uint8_t pin, uint8_t level) = 0;
    virtual uint8_t digitalRead(uint8_t pin) = 0;
    virtual uint32_t millis() = 0;
    virtual void delay(uint32_t ms) = 0;
    virtual void delayMicroseconds(uint32_t us) = 0;
};


