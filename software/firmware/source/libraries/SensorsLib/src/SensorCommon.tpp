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
 * @file      SensorCommon.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */


#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#endif


#define DEV_WIRE_NONE       (0)
#define DEV_WIRE_ERR        (-1)
#define DEV_WIRE_TIMEOUT    (-2)

#ifdef _BV
#undef _BV
#endif
#define _BV(b)                          (1UL << (uint32_t)(b))

// #define LOG_PORT Serial
#ifdef LOG_PORT
#define LOG(fmt, ...) LOG_PORT.printf("[%s] " fmt , __func__, ##__VA_ARGS__)
#define LOG_BIN(x)    LOG_PORT.println(x,BIN);
#else
#define LOG(fmt, ...)
#define LOG_BIN(x)
#endif


#define SENSORLIB_ATTR_NOT_IMPLEMENTED    __attribute__((error("Not implemented")))


template <class chipType>
class SensorCommon
{
    typedef int (*iic_fptr_t)(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);

public:
    ~SensorCommon()
    {
        if (__spiSetting) {
            delete __spiSetting;
        }
    }

    void setSpiSetting(uint32_t freq, uint8_t dataOrder = SPI_MSBFIRST, uint8_t dataMode = SPI_MODE0)
    {
        __freq = freq;
        __dataOrder = dataOrder;
        __dataMode = dataMode;
    }

#if defined(ARDUINO)
    bool begin(TwoWire &w, uint8_t addr, int sda, int scl)
    {
        LOG("Using Wire interface.\n");
        if (__has_init)return thisChip().initImpl();
        __wire = &w;
        __wire->begin(sda, scl);
        __addr = addr;
        __spi = NULL;
        thisReadRegCallback = NULL;
        thisWriteRegCallback = NULL;
        __has_init = thisChip().initImpl();
        return __has_init;
    }

    bool begin(int cs, int mosi = -1, int miso = -1, int sck = -1, SPIClass &spi = SPI)
    {
        LOG("Using SPI interface.\n");
        if (__has_init)return thisChip().initImpl();
        __spi = &spi;
        __cs  = cs;
        pinMode(__cs, OUTPUT);
        digitalWrite(__cs, HIGH);
        __spiSetting = new SPISettings(__freq, __dataOrder, __dataMode);
        if (!__spiSetting) {
            return false;
        }
        if (mosi != -1 && miso != -1 && sck == -1) {
            __spi->begin(sck, miso, mosi);
        } else {
            __spi->begin();
        }
        __wire = NULL;
        __readMask = thisChip().getReadMaskImpl();
        __has_init = thisChip().initImpl();
        return __has_init;
    }

#endif


    bool begin(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)
    {
        LOG("Using Custom interface.\n");
        if (__has_init)return thisChip().initImpl();
        thisReadRegCallback = readRegCallback;
        thisWriteRegCallback = writeRegCallback;
        __addr = addr;
        __spi = NULL;
        __has_init = thisChip().initImpl();
        return __has_init;
    }

protected:
    int readRegister(uint8_t reg)
    {
        uint8_t val = 0;
        if (thisReadRegCallback) {
            if (thisReadRegCallback(__addr, reg, &val, 1) != 0) {
                return DEV_WIRE_NONE;
            }
            return val;
        }
#if defined(ARDUINO)
        if (__wire) {
            __wire->beginTransmission(__addr);
            __wire->write(reg);
            if (__wire->endTransmission() != 0) {
                LOG("I2C Transfer Error!\n");
                return DEV_WIRE_ERR;
            }
            __wire->requestFrom(__addr, 1U);
            return __wire->read();
        }
        if (__spi) {
            uint8_t  data = 0x00;
            __spi->beginTransaction(*__spiSetting);
            digitalWrite(__cs, LOW);
            __spi->transfer(__readMask != -1 ? (reg  | __readMask) : reg);
            data = __spi->transfer(0x00);
            digitalWrite(__cs, HIGH);
            __spi->endTransaction();
            return data;
        }
#endif
        return DEV_WIRE_ERR;
    }

    int writeRegister(uint8_t reg, uint8_t norVal, uint8_t orVal)
    {
        int val = readRegister(reg);
        if (val == DEV_WIRE_ERR) {
            return DEV_WIRE_ERR;
        }
        val &= norVal;
        val |= orVal;
        return writeRegister(reg, val);
    }

    int writeRegister(uint8_t reg, uint8_t val)
    {
        if (thisWriteRegCallback) {
            return thisWriteRegCallback(__addr, reg, &val, 1);
        }
#if defined(ARDUINO)
        if (__wire) {
            __wire->beginTransmission(__addr);
            __wire->write(reg);
            __wire->write(val);
            return (__wire->endTransmission() == 0) ? DEV_WIRE_NONE : DEV_WIRE_ERR;
        }
        if (__spi) {
            __spi->beginTransaction(*__spiSetting);
            digitalWrite(__cs, LOW);
            __spi->transfer(reg);
            __spi->transfer(val);
            digitalWrite(__cs, HIGH);
            __spi->endTransaction();
            return DEV_WIRE_NONE;
        }
#endif
        return DEV_WIRE_ERR;
    }

    int readRegister(uint8_t reg, uint8_t *buf, uint8_t lenght)
    {
        if (thisReadRegCallback) {
            return thisReadRegCallback(__addr, reg, buf, lenght);
        }
#if defined(ARDUINO)
        if (__wire) {
            __wire->beginTransmission(__addr);
            __wire->write(reg);
            if (__wire->endTransmission() != 0) {
                return DEV_WIRE_ERR;
            }
            __wire->requestFrom(__addr, lenght);
            return __wire->readBytes(buf, lenght) == lenght ? DEV_WIRE_NONE : DEV_WIRE_ERR;
        }
        if (__spi) {
            __spi->beginTransaction(*__spiSetting);
            digitalWrite(__cs, LOW);
            __spi->transfer(__readMask != -1 ? (reg  | __readMask) : reg);
            for (size_t i = 0; i < lenght; i++) {
                buf[i] = __spi->transfer(0x00);
            }
            digitalWrite(__cs, HIGH);
            __spi->endTransaction();
            return DEV_WIRE_NONE;
        }
#endif
        return DEV_WIRE_ERR;
    }

    int writeRegister(uint8_t reg, uint8_t *buf, uint8_t lenght)
    {
        if (thisWriteRegCallback) {
            return thisWriteRegCallback(__addr, reg, buf, lenght);
        }
#if defined(ARDUINO)
        if (__wire) {
            __wire->beginTransmission(__addr);
            __wire->write(reg);
            __wire->write(buf, lenght);
            return (__wire->endTransmission() == 0) ? 0 : DEV_WIRE_ERR;
        }
        if (__spi) {
            __spi->beginTransaction(*__spiSetting);
            digitalWrite(__cs, LOW);
            __spi->transfer(reg);
            __spi->transfer(buf, lenght);
            digitalWrite(__cs, HIGH);
            __spi->endTransaction();
            return DEV_WIRE_NONE;
        }
#endif
        return DEV_WIRE_ERR;
    }


    bool inline clrRegisterBit(uint8_t registers, uint8_t bit)
    {
        int val = readRegister(registers);
        if (val == DEV_WIRE_ERR) {
            return false;
        }
        return  writeRegister(registers, (val & (~_BV(bit)))) == 0;
    }

    bool inline setRegisterBit(uint8_t registers, uint8_t bit)
    {
        int val = readRegister(registers);
        if (val == DEV_WIRE_ERR) {
            return false;
        }
        return  writeRegister(registers, (val | (_BV(bit)))) == 0;
    }

    bool inline getRegisterBit(uint8_t registers, uint8_t bit)
    {
        int val = readRegister(registers);
        if (val == DEV_WIRE_ERR) {
            return false;
        }
        return val & _BV(bit);
    }

    uint16_t inline readRegisterH8L4(uint8_t highReg, uint8_t lowReg)
    {
        int h8 = readRegister(highReg);
        int l4 = readRegister(lowReg);
        if (h8 == DEV_WIRE_ERR || l4 == DEV_WIRE_ERR)return 0;
        return (h8 << 4) | (l4 & 0x0F);
    }

    uint16_t inline readRegisterH8L5(uint8_t highReg, uint8_t lowReg)
    {
        int h8 = readRegister(highReg);
        int l5 = readRegister(lowReg);
        if (h8 == DEV_WIRE_ERR || l5 == DEV_WIRE_ERR)return 0;
        return (h8 << 5) | (l5 & 0x1F);
    }

    uint16_t inline readRegisterH6L8(uint8_t highReg, uint8_t lowReg)
    {
        int h6 = readRegister(highReg);
        int l8 = readRegister(lowReg);
        if (h6 == DEV_WIRE_ERR || l8 == DEV_WIRE_ERR)return 0;
        return ((h6 & 0x3F) << 8) | l8;
    }

    uint16_t inline readRegisterH5L8(uint8_t highReg, uint8_t lowReg)
    {
        int h5 = readRegister(highReg);
        int l8 = readRegister(lowReg);
        if (h5 == DEV_WIRE_ERR || l8 == DEV_WIRE_ERR)return 0;
        return ((h5 & 0x1F) << 8) | l8;
    }

    /*
     * CRTP Helper
     */
protected:

    bool begin()
    {
#if defined(ARDUINO)
        if (__has_init) return thisChip().initImpl();
        __has_init = true;

        if (__wire) {
            log_i("SDA:%d SCL:%d", __sda, __scl);
            __wire->begin(__sda, __scl);
        }
        if (__spi) {
            // int cs, int mosi = -1, int miso = -1, int sck = -1, SPIClass &spi = SPI
            begin(__cs, __mosi, __miso, __sck, *__spi);
        }

#endif  /*ARDUINO*/
        return thisChip().initImpl();
    }

    void end()
    {
#if defined(ARDUINO)
        if (__wire) {
#if defined(ESP_IDF_VERSION)
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,4,0)
            __wire->end();
#endif  /*ESP_IDF_VERSION*/
#endif  /*ESP_IDF_VERSION*/
        }
#endif /*ARDUINO*/
    }


    inline const chipType &thisChip() const
    {
        return static_cast<const chipType &>(*this);
    }

    inline chipType &thisChip()
    {
        return static_cast<chipType &>(*this);
    }

protected:
    bool        __has_init              = false;
#if defined(ARDUINO)
    TwoWire     *__wire                 = NULL;
    SPIClass    *__spi                  = NULL;
    SPISettings *__spiSetting           = NULL;
#endif
    uint32_t    __freq                  = 1000000;
    uint8_t     __dataOrder             = SPI_MSBFIRST;
    uint8_t     __dataMode              = SPI_MODE0;
    int         __readMask              = -1;
    int         __sda                   = -1;
    int         __scl                   = -1;
    int         __cs                    = -1;
    int         __miso                  = -1;
    int         __mosi                  = -1;
    int         __sck                   = -1;
    uint8_t     __addr                  = 0xFF;
    iic_fptr_t  thisReadRegCallback     = NULL;
    iic_fptr_t  thisWriteRegCallback    = NULL;
};
