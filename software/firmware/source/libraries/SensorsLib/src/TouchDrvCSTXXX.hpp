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
 * @file      TouchDrvCSTXXX.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 * @date      last 2025-01-20
 *
 */
#pragma once

#include "touch/TouchDrvCST226.h"
#include "touch/TouchDrvCST816.h"
#include "touch/TouchDrvCST92xx.h"

#define CSTXXX_SLAVE_ADDRESS        (0x15)
#define CST328_SLAVE_ADDRESS        (0x1A)

enum TouchDrvType {
    TouchDrv_CST226,
    TouchDrv_CST8XX,
    TouchDrv_CST92XX,
    TouchDrv_UNKOWN,
};

class TouchDrvCSTXXX : public TouchDrvInterface
{
public:
    TouchDrvCSTXXX();

    ~TouchDrvCSTXXX();

    // Set the touch screen model. If not set, the default is to read
    // the touch screen version information to determine which screen it is.
    void setTouchDrvModel(TouchDrvType model);

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr, int sda, int scl)override;
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)override;
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)override;
#endif  // ESP_PLATFORM
#endif  // ARDUINO

    // Set the callback I2C read and write method and initialize the touch screen.
    // If successful, return true, otherwise false

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr) override;

    // Set other ways to control touch RST or INT
    void setGpioCallback(SensorHalCustom::CustomMode mode_cb,
                         SensorHalCustom::CustomWrite write_cb,
                         SensorHalCustom::CustomRead read_cb) override;

    // Reset Touch
    void reset()override;

    // Get the XY coordinates of the touch screen touch
    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1)override;

    // Get whether the touch screen is pressed
    bool isPressed()override;

    // Get the touch screen model
    const char *getModelName()override;

    // Set the touch screen to sleep mode
    void sleep()override;

    // Wake up the touch screen, depends on the RST Pin, if it is not connected, it will be invalid
    void wakeup()override;

    // Idle mode, this is not implemented, most of the CSTXXX series have automatic sleep function
    void idle()override;

    // Get the maximum number of touch points supported by the touch screen
    uint8_t getSupportTouchPoint()override;

    // Get touch screen resolution, not implemented
    bool getResolution(int16_t *x, int16_t *y)override;

    // Set the screen touch button coordinates
    void setCenterButtonCoordinate(uint16_t x, uint16_t y);

    // Set screen touch button callback function
    void setHomeButtonCallback(TouchDrvInterface::HomeButtonCallback callback, void *user_data = NULL);

    // Disable automatic sleep, only for CST328
    void disableAutoSleep();

    // Enable automatic sleep, only for CST328
    void enableAutoSleep();

    // Swap XY coordinates
    void setSwapXY(bool swap);

    // Mirror XY Coordinates
    void setMirrorXY(bool mirrorX, bool mirrorY);

    // Set the maximum X, Y coordinates of the screen
    void setMaxCoordinates(uint16_t x, uint16_t y);

private:

    using DriverCreator = std::unique_ptr<TouchDrvInterface> (*)();

    static constexpr uint8_t driverCreatorMaxNum = 3;
    static DriverCreator driverCreators[driverCreatorMaxNum];

    std::unique_ptr<TouchDrvInterface> createDriver(TouchDrvType type)
    {
        if (/*type >= TouchDrv_UNKOWN &&*/
            type < sizeof(driverCreators) / sizeof(driverCreators[0])) {
            return driverCreators[type]();
        }
        return nullptr;
    }

    void setupDriver();

    SensorHalCustom::CustomWrite        _writePtr;
    SensorHalCustom::CustomRead         _readPtr;
    SensorHalCustom::CustomMode         _modePtr;
    TouchDrvType                        _touchType;
    std::unique_ptr<TouchDrvInterface>  _drv;
};
