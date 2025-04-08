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
 * @file      SensorRtcHelper.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-24
 *
 */
#include "SensorRtcHelper.hpp"

SensorRtcHelper::DriverCreator SensorRtcHelper::driverCreators[SensorRtcHelper::driverCreatorMaxNum] = {
    []() -> std::unique_ptr<SensorRTC> { return std::make_unique<SensorPCF85063>(); },
    []() -> std::unique_ptr<SensorRTC> { return std::make_unique<SensorPCF8563>(); },
};

SensorRtcHelper::SensorRtcHelper() : _drvType(RtcDrv_UNKOWN), _drv(nullptr)
{

}

SensorRtcHelper::~SensorRtcHelper()
{
}


void SensorRtcHelper::setRtcDrvModel(SensorRTCType model)
{
    _drvType = model;
}

#if defined(ARDUINO)
bool SensorRtcHelper::begin(TwoWire &wire,  int sda, int scl)
{
    bool success = false;
    for (int i = (_drvType == RtcDrv_UNKOWN) ? 0 : _drvType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<SensorRTCType>(i));

        if (_drv && _drv->begin(wire, sda, scl)) {
            _drvType = static_cast<SensorRTCType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _drvType = RtcDrv_UNKOWN;
    }
    return success;
}

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool SensorRtcHelper::begin(i2c_port_t port_num,  int sda, int scl)
{
    bool success = false;
    for (int i = (_drvType == RtcDrv_UNKOWN) ? 0 : _drvType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<SensorRTCType>(i));

        if (_drv && _drv->begin(port_num, sda, scl)) {
            _drvType = static_cast<SensorRTCType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _drvType = RtcDrv_UNKOWN;
    }
    return success;
}

#else

bool SensorRtcHelper::begin(i2c_master_bus_handle_t handle)
{
    bool success = false;
    for (int i = (_drvType == RtcDrv_UNKOWN) ? 0 : _drvType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<SensorRTCType>(i));

        if (_drv && _drv->begin(handle)) {
            _drvType = static_cast<SensorRTCType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _drvType = RtcDrv_UNKOWN;
    }
    return success;
}

#endif  // ESP_PLATFORM
#endif  // ARDUINO

bool SensorRtcHelper::begin(SensorCommCustom::CustomCallback callback)
{
    bool success = false;
    for (int i = (_drvType == RtcDrv_UNKOWN) ? 0 : _drvType; i < driverCreatorMaxNum; ++i) {
        _drv = createDriver(static_cast<SensorRTCType>(i));
        if (_drv && _drv->begin(callback)) {
            _drvType = static_cast<SensorRTCType>(i);
            success = true;
            break;
        }
    }
    if (!success) {
        _drvType = RtcDrv_UNKOWN;
    }
    return success;
}

void SensorRtcHelper::setDateTime(RTC_DateTime datetime)
{
    _drv->setDateTime(datetime);
}

RTC_DateTime SensorRtcHelper::getDateTime()
{
    return _drv->getDateTime();
}

const char *SensorRtcHelper::getChipName()
{
    return _drv->getChipName();
}

std::unique_ptr<SensorRTC> SensorRtcHelper::createDriver(SensorRTCType type)
{
    if (type < sizeof(driverCreators) / sizeof(driverCreators[0])) {
        return driverCreators[type]();
    }
    return nullptr;
}
