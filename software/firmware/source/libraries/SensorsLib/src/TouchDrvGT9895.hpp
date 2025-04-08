/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      TouchDrvGT9895.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */
#pragma once

#include "REG/GT9895Constants.h"
#include "TouchDrvInterface.hpp"

class TouchDrvGT9895 :  public TouchDrvInterface, public GT9895Constants
{
public:
    TouchDrvGT9895();
    ~TouchDrvGT9895();

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t address = GT9895_SLAVE_ADDRESS_L, int sda = -1, int scl = -1);
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = GT9895_SLAVE_ADDRESS_L, int sda = -1, int scl = -1);
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = GT9895_SLAVE_ADDRESS_L);
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = GT9895_SLAVE_ADDRESS_L);

    void deinit();
    void reset();
    void sleep();
    void wakeup();
    void idle();

    uint8_t getSupportTouchPoint();
    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t size = 1);
    bool isPressed();

    uint32_t getChipID();
    bool getResolution(int16_t *x, int16_t *y);

    const char *getModelName();

    void setGpioCallback(CustomMode mode_cb,
                         CustomWrite write_cb,
                         CustomRead read_cb);

private:

    int is_risk_data(const uint8_t *data, int size);
    int checksum_cmp(const uint8_t *data, int size, int mode);

    int readVersion(ChipFirmwareVersion *version);
    int convertChipInfo(ChipInfo *info, const uint8_t *data);
    void printChipInfo(ChipInfo *ic_info);
    int readChipInfo(ChipInfo *ic_info);

    void clearStatus();
    int getTouchData( uint8_t *pre_buf, uint32_t pre_buf_len);

    bool initImpl();

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;

    ChipTsEvent          _ts_event;
    ChipFirmwareVersion  _version;
    ChipInfo             _ic_info;
};


