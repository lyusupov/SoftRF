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
 * @file      SensorCommEspIDF_I2C.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#include "../SensorCommBase.hpp"

#if !defined(ARDUINO)  && defined(ESP_PLATFORM)

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_idf_version.h"
#include "esp_err.h"
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#define USEING_I2C_LEGACY                       1
#endif  //ESP_IDF_VERSION

#define SENSORLIB_I2C_MASTER_TIMEOUT_MS         1000
#define SENSORLIB_I2C_MASTER_SPEED              400000

class SensorCommI2C : public SensorCommBase
{
public:

#if defined(USEING_I2C_LEGACY)
    SensorCommI2C(i2c_port_t i2c_num, uint8_t addr, int sda, int scl, SensorHal *ptr = nullptr) :
        addr(addr), sda(sda), scl(scl), _i2cNum(i2c_num), sendStopFlag(true)
    {}
#else
    SensorCommI2C(i2c_master_bus_handle_t handle, uint8_t addr, SensorHal *ptr = nullptr) :
        addr(addr), _busHandle(handle), _i2cDevice(nullptr), sendStopFlag(true) {}
#endif

    bool init() override
    {
#if defined(USEING_I2C_LEGACY)
        return init_legacy();
#else
        return init_ll_hal();
#endif
    }

    void deinit() override
    {
#ifndef USEING_I2C_LEGACY
        if (_i2cDevice) {
            // Initialization failed, delete device
            log_i("i2c_master_bus_rm_device");
            i2c_master_bus_rm_device(_i2cDevice);
        }
#endif
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
        size_t totalLength = sizeof(uint8_t) + len;
        uint8_t *write_buffer = (uint8_t *)malloc(totalLength);
        if (!write_buffer) {
            return -1;
        }
        write_buffer[0] = reg;
        if (buf && len > 0) {
            memcpy(write_buffer + 1, buf, len);
        }
        int ret = writeBuffer(write_buffer, totalLength);
        free(write_buffer);
        return ret;
    }

    int writeBuffer(uint8_t *buffer, size_t len)
    {

#if defined(USEING_I2C_LEGACY)
        if (ESP_OK == i2c_master_write_to_device(_i2cNum, addr, buffer, len,
                SENSORLIB_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)) {
            return 0;
        }
#else //ESP_IDF_VERSION
        if (ESP_OK == i2c_master_transmit(_i2cDevice, buffer, len, -1)) {
            return 0;
        }

#endif //ESP_IDF_VERSION
        return -1;
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
#if defined(USEING_I2C_LEGACY)
        if (ESP_OK == i2c_master_write_read_device(_i2cNum, addr, (uint8_t *)&reg, 1, buf, len,
                SENSORLIB_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)) {
            return 0;
        }
#else //ESP_IDF_VERSION
        if (ESP_OK == i2c_master_transmit_receive(_i2cDevice, (const uint8_t *)&reg, 1, buf, len, -1)) {
            return 0;
        }
#endif //ESP_IDF_VERSION
        return -1;
    }

    int writeThenRead(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len) override
    {
#if defined(USEING_I2C_LEGACY)
        if (ESP_OK == i2c_master_write_read_device(
                    _i2cNum,
                    addr,
                    write_buffer,
                    write_len,
                    read_buffer,
                    read_len,
                    SENSORLIB_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)) {
            return 0;
        }
#else //ESP_IDF_VERSION
        if (ESP_OK == i2c_master_transmit_receive(
                    _i2cDevice,
                    write_buffer,
                    write_len,
                    read_buffer,
                    read_len,
                    -1)) {
            return 0;
        }
#endif //ESP_IDF_VERSION
        return -1;
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

#if !defined(USEING_I2C_LEGACY)
        deinit();
        init_ll_hal();
#endif
    }

private:



#if defined(USEING_I2C_LEGACY)

    bool init_legacy()
    {
        if (sda != -1 || scl != -1) {
            return true;
        }
        i2c_config_t i2c_conf;
        memset(&i2c_conf, 0, sizeof(i2c_conf));
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = sda;
        i2c_conf.scl_io_num = scl;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.master.clk_speed = SENSORLIB_I2C_MASTER_SPEED;

        /**
         * @brief Without checking whether the initialization is successful,
         * I2C may be initialized externally,
         * so just make sure there is an initialization here.
         */
        // static const IDF_TX_BUF_DISABLE =  0;  /*!< I2C master doesn't need buffer */
        // static const IDF_RX_BUF_DISABLE =  0;  /*!< I2C master doesn't need buffer */

        i2c_param_config(_i2cNum, &i2c_conf);
        i2c_driver_install(_i2cNum, i2c_conf.mode, 0, 0, 0);
        return true;
    }

#else //USEING_I2C_LEGACY

    // * Using the new API of esp-idf 5.x, need to pass the I2C BUS handle,
    // * which is useful when the bus shares multiple devices.
    bool init_ll_hal()
    {
        log_i("Using ESP-IDF Driver interface.");

        i2c_device_config_t devConf;
        if (_busHandle == NULL) return false;
        devConf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        devConf.device_address = addr;
        devConf.scl_speed_hz = SENSORLIB_I2C_MASTER_SPEED;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,3,0))
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,4,0))
        // New fields since esp-idf-v5.3-beta1
        devConf.scl_wait_us = 0;
#endif
        devConf.flags.disable_ack_check = 0;
#endif

        if (ESP_OK != i2c_master_bus_add_device(_busHandle, &devConf, &_i2cDevice)) {
            log_i("i2c_master_bus_add_device failed !");
            return false;
        }
        log_i("Added Device Address : 0x%X  New Dev Address: %p Speed :%lu ", addr, _i2cDevice, devConf.scl_speed_hz);
        return true;
    }
#endif //ESP 5.X

    uint8_t addr;
#if defined(USEING_I2C_LEGACY)
    int sda;
    int scl;
    i2c_port_t  _i2cNum;
#else
    i2c_master_bus_handle_t  _busHandle;
    i2c_master_dev_handle_t  _i2cDevice;
#endif
    bool sendStopFlag;
};

#endif  //*ESP_PLATFORM
