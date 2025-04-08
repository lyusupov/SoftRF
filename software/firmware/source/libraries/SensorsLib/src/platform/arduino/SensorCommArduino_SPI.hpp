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
 * @file      SensorCommArduino_SPI.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */

#pragma once

#include "../SensorCommBase.hpp"

#ifdef ARDUINO


#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_STM32))  && !defined(ARDUINO_ARCH_MBED)
#define  __writeBytes(txBuf,size) spi.transfer(txBuf,nullptr,size)
#elif defined(ARDUINO_ARCH_NRF52)
#define  __writeBytes(txBuf,size) spi.transfer(txBuf,nullptr,size)
#elif defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_ZEPHYR)
#define  __writeBytes(txBuf,size) spi.transfer((void*)txBuf,size)
#else
#define  __writeBytes(txBuf,size) spi.writeBytes(txBuf,size)
#endif


class SensorCommSPI : public SensorCommBase
{
public:
    SensorCommSPI(SPIClass &spi, uint8_t csPin, SensorHal *hal) : spi(spi), csPin(csPin), hal(hal) {}
    SensorCommSPI(SPIClass &spi, uint8_t csPin, int mosi, int miso, int sck, SensorHal *hal) : spi(spi),
        csPin(csPin), hal(hal), mosi(mosi), miso(miso), sck(sck) {}

    bool init() override
    {
        log_d("SensorCommSPI");
        if (!hal) {
            log_e("hal pointer is null");
            return false;
        }
        if (mosi != -1 && miso != -1 && sck != -1) {
#if   defined(ARDUINO_ARCH_NRF52)
            spi.setPins(miso, sck, mosi);
            spi.begin();
#elif defined(ARDUINO_ARCH_ESP32)
            spi.begin(sck, miso, mosi);
#elif defined(ARDUINO_ARCH_RP2040)
#if !defined(ARDUINO_ARCH_MBED)
            spi.setSCK(sck);
            spi.setRX(miso);
            spi.setTX(mosi);
#endif  /*ARDUINO_ARCH_MBED*/
            spi.begin();
#elif defined(ARDUINO_ARCH_STM32)
            spi.setMISO(miso);
            spi.setMOSI(mosi);
            spi.setSCLK(sck);
            spi.begin();
#else
#warning "SPI custom GPIO mapping function is not implemented"
#endif
        }
        hal->pinMode(csPin, OUTPUT);
        hal->digitalWrite(csPin, HIGH);
        return true;
    }

    void deinit() override
    {
        // Do not destroy spi bus
        // spi.end();
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
        hal->digitalWrite(csPin, LOW);
        spi.beginTransaction(setting);
        spi.transfer(reg);
        for (size_t i = 0; i < len; ++i) {
            spi.transfer(buf[i]);
        }
        spi.endTransaction();
        hal->digitalWrite(csPin, HIGH);
        return 0;
    }

    int readRegister(const uint8_t reg) override
    {
        uint8_t value = 0x00;
        if (readRegister(reg, &value, 1) < 0) {
            return -1;
        }
        return value;
    }

    int writeBuffer(uint8_t *buffer, size_t len)
    {
        hal->digitalWrite(csPin, LOW);
        spi.beginTransaction(setting);
        if (buffer && len > 0) {
            // spi.writeBytes(buffer, len);
            __writeBytes(buffer, len);
        }
        spi.endTransaction();
        hal->digitalWrite(csPin, HIGH);
        return 0;
    }

    int readRegister(const uint8_t reg, uint8_t *buf, size_t len) override
    {
        hal->digitalWrite(csPin, LOW);
        spi.beginTransaction(setting);
        spi.transfer(reg | 0x80);
#if defined(ARDUINO_ARCH_ESP32)
        spi.transferBytes(NULL, buf, len);
#else
        for (size_t i = 0; i < len; i++) {
            buf[i] = spi.transfer(0x00);
        }
#endif
        spi.endTransaction();
        hal->digitalWrite(csPin, HIGH);
        return 0;
    }

    int writeThenRead(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len) override
    {
        hal->digitalWrite(csPin, LOW);
        spi.beginTransaction(setting);

        // spi.writeBytes(write_buffer, write_len);
        __writeBytes(write_buffer, write_len);

        spi.endTransaction();

        spi.beginTransaction(setting);
        for (size_t i = 0; i < read_len; ++i) {
            read_buffer[i] = spi.transfer(0x00);
        }
        spi.endTransaction();
        hal->digitalWrite(csPin, HIGH);
        return 0;
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
        const auto pdat = dynamic_cast<const SPIParam *>(&params);
#else
        const auto pdat = static_cast<const SPIParam *>(&params);
#endif
        setting = pdat->getSetting();
    }

private:
    SPIClass &spi;
    uint8_t csPin;
    SensorHal *hal;
    SPISettings setting;
    int mosi, miso, sck;
};

#undef __writeBytes

#endif  //*ARDUINO








