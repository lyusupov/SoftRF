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
 * @file      SensorCM32181.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-14
 *
 */
#pragma once

#include "REG/CM32181Constants.h"
#include "SensorPlatform.hpp"

class SensorCM32181 : public CM32181Constants
{
public:
    enum Sampling {
        SAMPLING_X1,    //ALS Sensitivity x 1
        SAMPLING_X2,    //ALS Sensitivity x 2
        SAMPLING_X1_8,  //ALS Sensitivity x (1/8)
        SAMPLING_X1_4,  //ALS Sensitivity x (1/4)
    };

    enum IntegrationTime {
        INTEGRATION_TIME_25MS = 0x0C,
        INTEGRATION_TIME_50MS = 0x08,
        INTEGRATION_TIME_100MS = 0x00, //0000 = 100ms
        INTEGRATION_TIME_200MS, //0001 = 200ms
        INTEGRATION_TIME_400MS, //0010 = 400ms
        INTEGRATION_TIME_800MS, //0011 = 800ms
    };

    enum PowerSaveMode {
        PowerSaveMode1 = 0x00,
        PowerSaveMode2,
        PowerSaveMode3,
        PowerSaveMode4
    };

    enum InterruptEvent {
        ALS_EVENT_LOW_TRIGGER = 0x00,
        ALS_EVENT_HIGH_TRIGGER,
        ALS_EVENT_NULL,
    };

    SensorCM32181() : comm(nullptr) {}

    ~SensorCM32181()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = CM32181_ADDR_PRIMARY, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(wire, addr, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = CM32181_ADDR_PRIMARY, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(port_num, addr, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = CM32181_ADDR_PRIMARY)
    {
        comm = std::make_unique<SensorCommI2C>(handle, addr);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback, uint8_t addr = CM32181_ADDR_PRIMARY)
    {
        comm = std::make_unique<SensorCommCustom>(callback, addr);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }

    void setSampling(Sampling tempSampling  = SAMPLING_X1,
                     IntegrationTime int_time = INTEGRATION_TIME_200MS
                    )
    {
        uint16_t data;
        comm->readRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
        data &= ~(0x03 << 11);
        data |= tempSampling << 11;
        data &= ~(0xF << 6);
        data |= int_time << 6;
        comm->readRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    void setIntThreshold(uint16_t low_threshold, uint16_t high_threshold)
    {

        uint8_t buffer[2] = {0};
        buffer[1] = highByte(high_threshold);
        buffer[0] = lowByte(high_threshold);;
        comm->writeRegister(REG_ALS_THDH, buffer, 2);
        buffer[1] = highByte(low_threshold);
        buffer[0] = lowByte(low_threshold);
        comm->writeRegister(REG_ALS_THDL, buffer, 2);
    }


    void powerSave(PowerSaveMode mode, bool enable)
    {
        uint16_t data = 0x00;
        comm->readRegister(REG_ALS_PSM, (uint8_t *)&data, 2);
        data |= mode << 1;
        enable ? data |= 0x01 : data |= 0x00;
        comm->writeRegister(REG_ALS_PSM, (uint8_t *)&data, 2);
    }

    // Read REG_ALS_STATUS register to clear interrupt
    InterruptEvent getIrqStatus()
    {
        uint16_t data;
        comm->readRegister(REG_ALS_STATUS, (uint8_t *)&data, 2);
        return bitRead(data, 15) ? ALS_EVENT_LOW_TRIGGER :  bitRead(data, 14) ? ALS_EVENT_HIGH_TRIGGER : ALS_EVENT_NULL;
    }

    void enableINT()
    {
        uint16_t data;
        comm->readRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitWrite(data, 1, 1);
        comm->writeRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    void disableINT()
    {
        uint16_t data;
        comm->readRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitWrite(data, 1, 0);
        comm->writeRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    void powerOn()
    {
        uint16_t data;
        comm->readRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitClear(data, 0);
        comm->writeRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    void powerDown()
    {
        uint16_t data;
        comm->readRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitSet(data, 0);
        comm->writeRegister(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    uint16_t getRaw()
    {
        uint8_t buffer[2] = {0};
        comm->readRegister(REG_ALS_DATA, buffer, 2);
        return (uint16_t) buffer[0] | (uint16_t)(buffer[1] << 8);
    }

    float getLux()
    {
        return getRaw() * calibration_factor;
    }

    int getChipID()
    {
        uint8_t buffer[2] = {0};
        comm->readRegister(REG_ID, buffer, 2);
        return  lowByte(buffer[0]);
    }

private:

    bool initImpl()
    {
        I2CParam params(I2CParam::I2C_SET_FLAG, false);
        comm->setParams(params);
        int chipID = getChipID();
        log_i("chipID:%d\n", chipID);
        if (chipID < 0 ) {
            return false;
        }
        if (chipID != CM32181_CHIP_ID) {
            return false;
        }
        return true;
    }

    std::unique_ptr<SensorCommBase> comm;
    // The default calibration value, learned from the manual,
    // is now unable to obtain the calibration value from the specified register
    const float calibration_factor = 0.286;
};



