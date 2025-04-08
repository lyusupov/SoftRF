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
 * @file      TouchDrvCST226.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#pragma once

#include "TouchDrvInterface.hpp"

#define CST226SE_SLAVE_ADDRESS      (0x5A)

class TouchDrvCST226 : public TouchDrvInterface
{
public:

    TouchDrvCST226();

    ~TouchDrvCST226();

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = CST226SE_SLAVE_ADDRESS, int sda = -1, int scl = -1);
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = CST226SE_SLAVE_ADDRESS, int sda = -1, int scl = -1);
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = CST226SE_SLAVE_ADDRESS);
#endif
#endif


    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = CST226SE_SLAVE_ADDRESS);

    void reset();

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point);

    bool isPressed();

    const char *getModelName();

    void sleep();

    void wakeup();

    void idle();

    uint8_t getSupportTouchPoint();

    bool getResolution(int16_t *x, int16_t *y);

    void setHomeButtonCallback(HomeButtonCallback cb, void *user_data);

    void setGpioCallback(CustomMode mode_cb,
                         CustomWrite write_cb,
                         CustomRead read_cb);

private:
    bool initImpl();
protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    TouchData report;
    static constexpr uint8_t  CST226SE_CHIPTYPE   =  (0xA8);
};










