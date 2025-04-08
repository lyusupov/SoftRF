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
 * @file      TouchDrvCSTXXX.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 * @date      last 2025-01-20
 *
 */
#include "TouchDrvCSTXXX.hpp"

#define CSTXXX_SLAVE_ADDRESS        (0x15)
#define CST328_SLAVE_ADDRESS        (0x1A)

TouchDrvCSTXXX::DriverCreator TouchDrvCSTXXX::driverCreators[TouchDrvCSTXXX::driverCreatorMaxNum] = {
    []() -> std::unique_ptr<TouchDrvInterface> { return std::make_unique<TouchDrvCST226>(); },
    []() -> std::unique_ptr<TouchDrvInterface> { return std::make_unique<TouchDrvCST816>(); },
    []() -> std::unique_ptr<TouchDrvInterface> { return std::make_unique<TouchDrvCST92xx>(); }
};

TouchDrvCSTXXX::TouchDrvCSTXXX():
    _writePtr(nullptr),
    _readPtr(nullptr),
    _modePtr(nullptr),
    _touchType(TouchDrv_UNKOWN),
    _drv(nullptr) {}

TouchDrvCSTXXX::~TouchDrvCSTXXX() {}

void TouchDrvCSTXXX::setTouchDrvModel(TouchDrvType model)
{
    _touchType = model;
}

void TouchDrvCSTXXX::setupDriver()
{
    if (_drv) {
        _drv->setPins(_rst, _irq);
        _drv->setCustomMode(_modePtr);
        _drv->setCustomWrite(_writePtr);
        _drv->setCustomRead(_readPtr);
    }
}

#if defined(ARDUINO)
bool TouchDrvCSTXXX::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    bool success = false;
    for (int i = (_touchType == TouchDrv_UNKOWN) ? 0 : _touchType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<TouchDrvType>(i));
        setupDriver();
        if (_drv && _drv->begin(wire, addr, sda, scl)) {
            _touchType = static_cast<TouchDrvType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _touchType = TouchDrv_UNKOWN;
    }
    return success;
}
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
bool TouchDrvCSTXXX::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    bool success = false;
    for (int i = (_touchType == TouchDrv_UNKOWN) ? 0 : _touchType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<TouchDrvType>(i));
        setupDriver();
        if (_drv && _drv->begin(port_num, addr, sda, scl)) {
            _touchType = static_cast<TouchDrvType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _touchType = TouchDrv_UNKOWN;
    }
    return success;
}
#else
bool TouchDrvCSTXXX::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    bool success = false;
    for (int i = (_touchType == TouchDrv_UNKOWN) ? 0 : _touchType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<TouchDrvType>(i));
        setupDriver();
        if (_drv && _drv->begin(handle, addr)) {
            _touchType = static_cast<TouchDrvType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _touchType = TouchDrv_UNKOWN;
    }
    return success;
}
#endif  // ESP_PLATFORM
#endif  // ARDUINO

void TouchDrvCSTXXX::setGpioCallback(SensorHalCustom::CustomMode mode_cb,
                                     SensorHalCustom::CustomWrite write_cb,
                                     SensorHalCustom::CustomRead read_cb)
{
    _writePtr = write_cb;
    _readPtr = read_cb;
    _modePtr = mode_cb;
}

bool TouchDrvCSTXXX::begin(SensorCommCustom::CustomCallback callback,
                           SensorCommCustomHal::CustomHalCallback hal_callback,
                           uint8_t addr)
{
    bool success = false;
    for (int i = (_touchType == TouchDrv_UNKOWN) ? 0 : _touchType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<TouchDrvType>(i));
        setupDriver();
        if (_drv && _drv->begin(callback, hal_callback, addr)) {
            _touchType = static_cast<TouchDrvType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _touchType = TouchDrv_UNKOWN;
    }
    return success;
}


void TouchDrvCSTXXX::reset()
{
    if (!_drv)return;
    _drv->reset();
}

uint8_t TouchDrvCSTXXX::getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point)
{
    if (!_drv)return 0;
    return _drv->getPoint(x_array, y_array, get_point);
}

bool TouchDrvCSTXXX::isPressed()
{
    if (!_drv)return false;
    return _drv->isPressed();
}

const char *TouchDrvCSTXXX::getModelName()
{
    if (!_drv)return "NULL";
    return _drv->getModelName();
}

void TouchDrvCSTXXX::sleep()
{
    if (!_drv)return;
    _drv->sleep();
}

void TouchDrvCSTXXX::wakeup()
{
    if (!_drv)return;
    _drv->reset();
}

void TouchDrvCSTXXX::idle()
{
    if (!_drv)return;
    _drv->idle();
}

uint8_t TouchDrvCSTXXX::getSupportTouchPoint()
{
    if (!_drv)return 0;
    return _drv->getSupportTouchPoint();
}

bool TouchDrvCSTXXX::getResolution(int16_t *x, int16_t *y)
{
    if (!_drv)return false;
    return _drv->getResolution(x, y);
}

void TouchDrvCSTXXX::setCenterButtonCoordinate(uint16_t x, uint16_t y)
{
    if (!_drv)return ;
    const char *model = _drv->getModelName();
    if (strncmp(model, "CST8", 3) == 0) {
        TouchDrvCST816 *pT = static_cast<TouchDrvCST816 *>(_drv.get());
        pT->setCenterButtonCoordinate(x, y);
    }
}

void TouchDrvCSTXXX::setHomeButtonCallback(TouchDrvInterface::HomeButtonCallback callback, void *user_data)
{
    if (!_drv)return ;
    const char *model = _drv->getModelName();
    if (strncmp(model, "CST8", 3) == 0) {
        TouchDrvCST816 *pT = static_cast<TouchDrvCST816 *>(_drv.get());
        pT->setHomeButtonCallback(callback, user_data);

    } else if (strncmp(model, "CST2", 3) == 0) {
        TouchDrvCST226 *pT = static_cast<TouchDrvCST226 *>(_drv.get());
        pT->setHomeButtonCallback(callback, user_data);
    }
}

void TouchDrvCSTXXX::disableAutoSleep()
{
    if (!_drv)return ;
    const char *model = _drv->getModelName();
    if (strncmp(model, "CST8", 3) == 0) {
        TouchDrvCST816 *pT = static_cast<TouchDrvCST816 *>(_drv.get());
        pT->disableAutoSleep();
    }
}

void TouchDrvCSTXXX::enableAutoSleep()
{
    if (!_drv)return ;
    const char *model = _drv->getModelName();
    if (strncmp(model, "CST8", 3) == 0) {
        TouchDrvCST816 *pT = static_cast<TouchDrvCST816 *>(_drv.get());
        pT->enableAutoSleep();
    }
}

void TouchDrvCSTXXX::setSwapXY(bool swap)
{
    if (!_drv)return ;
    _drv->setSwapXY(swap);
}

void TouchDrvCSTXXX::setMirrorXY(bool mirrorX, bool mirrorY)
{
    if (!_drv)return ;
    _drv->setMirrorXY(mirrorX, mirrorY);
}

void TouchDrvCSTXXX::setMaxCoordinates(uint16_t x, uint16_t y)
{
    if (!_drv)return ;
    _drv->setMaxCoordinates(x, y);
}

