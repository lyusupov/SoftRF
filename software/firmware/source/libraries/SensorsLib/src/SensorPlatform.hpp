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
 * @file      SensorPlatform.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#if defined(ARDUINO)
#include "platform/arduino/SensorCommArduino_HW.hpp"
#include "platform/arduino/SensorCommArduino_I2C.hpp"
#include "platform/arduino/SensorCommArduino_SPI.hpp"
#elif !defined(ARDUINO)  && defined(ESP_PLATFORM)
#include "platform/espidf/SensorCommEspIDF_HW.hpp"
#include "platform/espidf/SensorCommEspIDF_I2C.hpp"
#include "platform/espidf/SensorCommEspIDF_SPI.hpp"
#endif
#include "platform/SensorCommCustom.hpp"
#include "platform/SensorCommCustomHal.hpp"
#include "platform/SensorCommDebug.hpp"
#include "platform/SensorCommStatic.hpp"

enum CommInterface {
    COMM_CUSTOM = 0,
    COMM_SPI = 1,
    COMM_I2C = 2
};

template<typename CommType, typename HalType, typename... Args>
bool beginCommonStatic(
    std::unique_ptr<SensorCommBase> &comm,
    std::unique_ptr<SensorCommStatic> &staticComm,
    std::unique_ptr<SensorHal> &hal,
    Args &&... args)
{
    hal = std::make_unique<HalType>();
    if (!hal) {
        return false;
    }
    comm = std::make_unique<CommType>(std::forward<Args>(args)..., hal.get());
    if (!comm) {
        hal.reset();
        return false;
    }
    if (!comm->init()) {
        log_e("Bus init failed!");
        return false;
    }
    staticComm = std::make_unique<SensorCommStatic>(comm.get(), hal.get());
    if (!staticComm) {
        return false;
    }
    return true;
}

template<typename CommType, typename HalType, typename... Args>
bool beginCommon(
    std::unique_ptr<SensorCommBase> &comm,
    std::unique_ptr<SensorHal> &hal,
    Args &&... args)
{
    hal = std::make_unique<HalType>();
    if (!hal) {
        return false;
    }
    comm = std::make_unique<CommType>(std::forward<Args>(args)..., hal.get());
    if (!comm) {
        hal.reset();
        return false;
    }
    comm->init();
    return true;
}

template <typename CommType, typename HalType>
bool beginCommCustomCallback(CommInterface interface,
                             typename CommType::CustomCallback callback,
                             typename HalType::CustomHalCallback hal_callback,
                             uint8_t addr,
                             std::unique_ptr<SensorCommBase> &comm,
                             std::unique_ptr<SensorHal> &hal)
{
    hal = std::make_unique<HalType>(hal_callback);
    if (!hal) {
        return false;
    }
    comm = std::make_unique<CommType>(callback, addr);
    if (!comm) {
        return false;
    }
    comm->init();
    return true;
}
