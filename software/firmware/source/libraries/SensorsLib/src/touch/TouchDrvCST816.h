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
 * @file      TouchDrvCST816.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#pragma once

#include "TouchDrvInterface.hpp"

#define CST816_SLAVE_ADDRESS    0x15

class TouchDrvCST816 : public TouchDrvInterface
{
public:
    TouchDrvCST816();

    ~TouchDrvCST816();

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t address = CST816_SLAVE_ADDRESS, int sda = -1, int scl = -1);
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = CST816_SLAVE_ADDRESS, int sda = -1, int scl = -1);
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = CST816_SLAVE_ADDRESS);
#endif
#endif

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = CST816_SLAVE_ADDRESS);

    void reset();

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point);

    bool isPressed();

    const char *getModelName();

    void sleep();

    void wakeup();

    void idle();

    uint8_t getSupportTouchPoint();

    bool getResolution(int16_t *x, int16_t *y);

    void setCenterButtonCoordinate(int16_t x, int16_t y);

    void setHomeButtonCallback(HomeButtonCallback cb, void *user_data);

    void disableAutoSleep();

    void enableAutoSleep();

    void setGpioCallback(CustomMode mode_cb,
                         CustomWrite write_cb,
                         CustomRead read_cb);

private:
    bool initImpl();

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;

    int16_t _center_btn_x;
    int16_t _center_btn_y;

    static constexpr uint8_t  CST8xx_REG_STATUS         = (0x00);
    static constexpr uint8_t  CST8xx_REG_XPOS_HIGH      = (0x03);
    static constexpr uint8_t  CST8xx_REG_XPOS_LOW       = (0x04);
    static constexpr uint8_t  CST8xx_REG_YPOS_HIGH      = (0x05);
    static constexpr uint8_t  CST8xx_REG_YPOS_LOW       = (0x06);
    static constexpr uint8_t  CST8xx_REG_DIS_AUTOSLEEP  = (0xFE);
    static constexpr uint8_t  CST8xx_REG_CHIP_ID        = (0xA7);
    static constexpr uint8_t  CST8xx_REG_FW_VERSION     = (0xA9);
    static constexpr uint8_t  CST8xx_REG_SLEEP          = (0xE5);
    static constexpr uint8_t  CST816S_CHIP_ID           = (0xB4);
    static constexpr uint8_t  CST816T_CHIP_ID           = (0xB5);
    static constexpr uint8_t  CST716_CHIP_ID            = (0x20);
    static constexpr uint8_t  CST820_CHIP_ID            = (0xB7);
    static constexpr uint8_t  CST816D_CHIP_ID           = (0xB6);

};







