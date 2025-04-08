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
 * @file      TouchDrvCST226.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#include "TouchDrvCST226.h"

TouchDrvCST226::TouchDrvCST226() : comm(nullptr), hal(nullptr)
{

}

TouchDrvCST226::~TouchDrvCST226()
{
    if (comm) {
        comm->deinit();
    }
}

#if defined(ARDUINO)
bool TouchDrvCST226::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
        return false;
    }
    return initImpl();
}
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool TouchDrvCST226::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
        return false;
    }
    return true;
}
#else
bool TouchDrvCST226::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
        return false;
    }
    return true;
}
#endif  //USEING_I2C_LEGACY
#endif //ARDUINO

bool TouchDrvCST226::begin(SensorCommCustom::CustomCallback callback,
                           SensorCommCustomHal::CustomHalCallback hal_callback,
                           uint8_t addr)
{
    if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
            callback, hal_callback, addr, comm, hal)) {
        return false;
    }
    return initImpl();
}

void TouchDrvCST226::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, LOW);
        hal->delay(100);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(100);
    } else {
        comm->writeRegister(0xD1, 0x0E);
        hal->delay(20);
    }
}

uint8_t TouchDrvCST226::getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point)
{
    static constexpr uint8_t  statusReg   =  (0x00);
    static constexpr uint8_t  bufferSize =   (28);

    uint8_t buffer[bufferSize];
    uint8_t index = 0;

    if (comm->readRegister(statusReg, buffer, bufferSize) == -1) {
        return 0;
    }
    if (buffer[0] == 0x83 && buffer[1] == 0x17 && buffer[5] == 0x80) {
        if (_HButtonCallback) {
            _HButtonCallback(_userData);
        }
        return 0;
    }

    if (buffer[6] != 0xAB)return 0;
    if (buffer[0] == 0xAB)return 0;
    if (buffer[5] == 0x80)return 0;

    uint8_t numPoints = buffer[5] & 0x7F;

    if (numPoints > 5  || !numPoints) {
        comm->writeRegister(0x00, 0xAB);
        return 0;
    }

    for (int i = 0; i < numPoints; i++) {
        report.id[i] = buffer[index] >> 4;
        report.status[i] = buffer[index] & 0x0F;
        report.x[i] = (uint16_t)((buffer[index + 1] << 4) | ((buffer[index + 3] >> 4) & 0x0F));
        report.y[i] = (uint16_t)((buffer[index + 2] << 4) | (buffer[index + 3] & 0x0F));
        report.pressure[i] = buffer[index + 4];
        index = (i == 0) ?  (index + 7) :  (index + 5);
    }

    updateXY(numPoints, report.x, report.y);

    if (numPoints) {
        for (int i = 0; i < get_point; i++) {
            x_array[i] =  report.x[i];
            y_array[i] =  report.y[i];
        }
    }

    return numPoints;
}

bool TouchDrvCST226::isPressed()
{
    static uint32_t lastPulse = 0;
    if (_irq != -1) {
        int val = hal->digitalRead(_irq) == LOW;
        if (val) {
            //Filter low levels with intervals greater than 1000ms
            val = (hal->millis() - lastPulse > 1000) ?  false : true;
            lastPulse = hal->millis();
            return val;
        }
        return false;
    }
    return getPoint(NULL, NULL, 1);
}


const char *TouchDrvCST226::getModelName()
{
    return "CST226SE";
}

void TouchDrvCST226::sleep()
{
    comm->writeRegister(0xD1, 0x05);
#ifdef ARDUINO_ARCH_ESP32
    if (_irq != -1) {
        hal->pinMode(_irq, OPEN_DRAIN);
    }
    if (_rst != -1) {
        hal->pinMode(_rst, OPEN_DRAIN);
    }
#endif
}

void TouchDrvCST226::wakeup()
{
    reset();
}

void TouchDrvCST226::idle()
{

}

uint8_t TouchDrvCST226::getSupportTouchPoint()
{
    return 5;
}

bool TouchDrvCST226::getResolution(int16_t *x, int16_t *y)
{
    *x = _resX;
    *y = _resY;
    return true;
}

void TouchDrvCST226::setHomeButtonCallback(HomeButtonCallback cb, void *user_data)
{
    _HButtonCallback = cb;
    _userData = user_data;
}

void TouchDrvCST226::setGpioCallback(CustomMode mode_cb,
                                     CustomWrite write_cb,
                                     CustomRead read_cb)
{
    SensorHalCustom::setCustomMode(mode_cb);
    SensorHalCustom::setCustomWrite(write_cb);
    SensorHalCustom::setCustomRead(read_cb);
}

bool TouchDrvCST226::initImpl()
{

    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
    }

    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    reset();

    uint8_t buffer[8];
    // Enter Command mode
    comm->writeRegister(0xD1, 0x01);
    hal->delay(10);
    uint8_t write_buffer[2] = {0xD1, 0xFC};
    comm->writeThenRead(write_buffer, 2, buffer, 4);
    uint32_t checkcode = 0;
    checkcode = buffer[3];
    checkcode <<= 8;
    checkcode |= buffer[2];
    checkcode <<= 8;
    checkcode |= buffer[1];
    checkcode <<= 8;
    checkcode |= buffer[0];

    log_i("Chip checkcode:0x%lx.", checkcode);

    write_buffer[0] = {0xD1};
    write_buffer[1] = {0xF8};
    comm->writeThenRead(write_buffer, 2, buffer, 4);
    _resX = ( buffer[1] << 8) | buffer[0];
    _resY = ( buffer[3] << 8) | buffer[2];
    log_i("Chip resolution X:%u Y:%u", _resX, _resY);

    write_buffer[0] = {0xD2};
    write_buffer[1] = {0x04};
    comm->writeThenRead(write_buffer, 2, buffer, 4);
    uint32_t chipType = buffer[3];
    chipType <<= 8;
    chipType |= buffer[2];


    uint32_t ProjectID = buffer[1];
    ProjectID <<= 8;
    ProjectID |= buffer[0];
    log_i("Chip type :0x%lx, ProjectID:0X%lx",
          chipType, ProjectID);



    write_buffer[0] = {0xD2};
    write_buffer[1] = {0x08};
    comm->writeThenRead(write_buffer, 2, buffer, 8);

    uint32_t fwVersion = buffer[3];
    fwVersion <<= 8;
    fwVersion |= buffer[2];
    fwVersion <<= 8;
    fwVersion |= buffer[1];
    fwVersion <<= 8;
    fwVersion |= buffer[0];

    uint32_t checksum = buffer[7];
    checksum <<= 8;
    checksum |= buffer[6];
    checksum <<= 8;
    checksum |= buffer[5];
    checksum <<= 8;
    checksum |= buffer[4];

    log_i("Chip ic version:0x%lx, checksum:0x%lx",
          fwVersion, checksum);

    if (fwVersion == 0xA5A5A5A5) {
        log_e("Chip ic don't have firmware.");
        return false;
    }
    if ((checkcode & 0xffff0000) != 0xCACA0000) {
        log_e("Firmware info read error.");
        return false;
    }

    if (chipType != CST226SE_CHIPTYPE) {
        log_e("Chip ID does not match, should be 0x%2X", CST226SE_CHIPTYPE);
        return false;
    }

    _chipID = chipType;

    // Exit Command mode
    comm->writeRegister(0xD1, 0x09);

    return true;
}
















