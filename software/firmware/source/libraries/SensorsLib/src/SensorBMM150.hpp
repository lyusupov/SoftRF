/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      SensorBMM150.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-09
 * @note      Most source code references come from the https://github.com/boschsensortec/BMM150-Sensor-API
 *            Simplification for Arduino
 */
#pragma once

#include "bosch/BMM150/bmm150.h"
#include "SensorPlatform.hpp"

#if defined(ARDUINO)

/*! @name I2C ADDRESS       */
#define BMM150_DEFAULT_I2C_ADDRESS                UINT8_C(0x10)
#define BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH       UINT8_C(0x11)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW       UINT8_C(0x12)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH      UINT8_C(0x13)


class SensorBMM150
{
public:
    enum  PowerMode {
        POWERMODE_NORMAL,
        POWERMODE_FORCED,
        POWERMODE_SLEEP,
        POWERMODE_SUSPEND,
    };

    enum InterruptLevel {
        INTERRUPT_HIGH_ACTIVE,
        INTERRUPT_LOW_ACTIVE,
    };

    SensorBMM150(): comm(nullptr),
        hal(nullptr),
        staticComm(nullptr),
        dev(nullptr),
        _rst(-1),
        _error_code(0)
    {
    }

    ~SensorBMM150()
    {
    }

    void setPins(int rst)
    {
        _rst = rst;
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr, int sda, int scl)
    {
        if (!beginCommonStatic<SensorCommI2C, HalArduino>(comm, staticComm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl(BMM150_I2C_INTF);
    }

    bool begin(SPIClass &spi, uint8_t csPin, int mosi, int miso, int sck)
    {
        if (!beginCommonStatic<SensorCommSPI, HalArduino>(comm,
                staticComm, hal,
                spi, csPin, mosi, miso, sck)) {
            return false;
        }
        return initImpl(BMM150_I2C_INTF);
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
    {
        if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl(BHY2_I2C_INTERFACE);
    }
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, handle, addr)) {
            return false;
        }
        return initImpl(BHY2_I2C_INTERFACE);
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(CommInterface interface,
               SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(interface,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl(static_cast<bmm150_intf>(interface));
    }

    void reset()
    {
        if (_rst != -1) {
            hal->digitalWrite(_rst, HIGH);
            hal->delay(5);
            hal->digitalWrite(_rst, LOW);
            hal->delay(10);
            hal->digitalWrite(_rst, HIGH);
            hal->delay(5);
        }
    }

    void sleep()
    {
        setMode(POWERMODE_SLEEP);
    }

    bool setMode(PowerMode mode)
    {
        settings.pwr_mode = mode;
        return  bmm150_set_op_mode(&settings, dev.get()) == BMM150_OK;
    }

    bool setThreshold(uint8_t high_th, uint8_t low_th)
    {
        settings.int_settings.high_threshold = high_th;
        settings.int_settings.low_threshold = low_th;
        return bmm150_set_sensor_settings(BMM150_SEL_HIGH_THRESHOLD_SETTING, &settings, dev.get()) == BMM150_OK;
    }

    bool setInterruptLevel(InterruptLevel level)
    {
        settings.int_settings.high_int_en = level;
        return bmm150_set_sensor_settings(BMM150_SEL_HIGH_THRESHOLD_INT, &settings, dev.get()) == BMM150_OK;
    }

    bool enableINT()
    {
        settings.int_settings.int_pin_en = BMM150_INT_ENABLE;
        return bmm150_set_sensor_settings(BMM150_SEL_INT_PIN_EN, &settings, dev.get()) == BMM150_OK;
    }

    bool disableINT()
    {
        settings.int_settings.int_pin_en = BMM150_INT_DISABLE;
        return bmm150_set_sensor_settings(BMM150_SEL_INT_PIN_EN, &settings, dev.get()) == BMM150_OK;
    }

    bool enabledDataReady()
    {
        settings.int_settings.drdy_pin_en = BMM150_INT_ENABLE;
        return bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev.get()) == BMM150_OK;
    }

    bool disabledDataReady()
    {
        settings.int_settings.drdy_pin_en = BMM150_INT_DISABLE;
        return bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev.get()) == BMM150_OK;
    }

    uint8_t getChipID()
    {
        return dev->chip_id;
    }

    uint8_t getIrqStatus()
    {
        bmm150_get_interrupt_status(dev.get());
        return dev->int_status;
    }

    bool isDataReady()
    {
        return dev->int_status & BMM150_INT_ASSERTED_DRDY;
    }

    bool isLowThreshold()
    {
        return dev->int_status & BMM150_INT_ASSERTED_LOW_THRES;
    }

    bool isHighThreshold()
    {
        return dev->int_status & BMM150_INT_ASSERTED_HIGH_THRES;
    }

    struct bmm150_mag_data getMag()
    {
        struct bmm150_mag_data data = {0, 0, 0};
        bmm150_read_mag_data(&data, dev.get());
        return data;
    }

    bool getMag(int16_t &x, int16_t &y, int16_t &z)
    {
        struct bmm150_mag_data data;
        if (bmm150_read_mag_data(&data, dev.get()) != BMM150_OK) {
            return false;
        }
        x = data.x;
        y = data.y;
        z = data.z;
        return true;
    }

private:


    bool initImpl(bmm150_intf interface)
    {
        memset(&settings, 0, sizeof(settings));

        if (_rst != -1) {
            hal->pinMode(_rst, OUTPUT);
        }

        reset();

        dev = std::make_unique<struct bmm150_dev>();
        if (!dev) {
            log_e(" Device handler malloc failed!");
            return false;
        }

        dev->intf = interface;
        dev->read = SensorCommStatic::sensor_static_read_data;
        dev->write = SensorCommStatic::sensor_static_write_data;
        dev->intf_ptr = staticComm.get();
        dev->delay_us = SensorCommStatic::sensor_static_delay_us;

        _error_code = bmm150_init(dev.get());
        if (_error_code != BMM150_OK) {
            return false;
        }
        _error_code = bmm150_soft_reset(dev.get());
        if (_error_code != BMM150_OK) {
            return false;
        }
        bmm150_get_sensor_settings(&settings, dev.get());
        return _error_code == BMM150_OK;
    }

    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    std::unique_ptr<SensorCommStatic> staticComm;
    std::unique_ptr<struct bmm150_dev> dev;
    int                     _rst;
    int8_t                  _error_code;
    struct bmm150_settings  settings;

};


#endif /*defined(ARDUINO)*/




