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
 * @file      TouchDrvCST816.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#include "TouchDrvCST816.h"

TouchDrvCST816::TouchDrvCST816() : comm(nullptr), hal(nullptr),
    _center_btn_x(0),
    _center_btn_y(0)
{
}

TouchDrvCST816::~TouchDrvCST816()
{
    if (comm) {
        comm->deinit();
    }
}

#if defined(ARDUINO)
bool TouchDrvCST816::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
        return false;
    }
    return initImpl();
}
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool TouchDrvCST816::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
        return false;
    }
    return true;
}
#else
bool TouchDrvCST816::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
        return false;
    }
    return true;
}
#endif  //USEING_I2C_LEGACY
#endif //ARDUINO

bool TouchDrvCST816::begin(SensorCommCustom::CustomCallback callback,
                           SensorCommCustomHal::CustomHalCallback hal_callback,
                           uint8_t addr)
{
    if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
            callback, hal_callback, addr, comm, hal)) {
        return false;
    }
    return initImpl();
}

void TouchDrvCST816::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, LOW);
        hal->delay(30);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(50);
    }
}

uint8_t TouchDrvCST816::getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point)
{
    uint8_t buffer[13];
    if (comm->readRegister(CST8xx_REG_STATUS, buffer, 13) == -1) {
        return 0;
    }

    if (!buffer[2] || !x_array || !y_array || !get_point) {
        return 0;
    }

    // Some CST816T will return all 0xFF after turning off automatic sleep.
    if (buffer[2] == 0xFF) {
        return 0;
    }

    uint8_t numPoints = buffer[2] & 0x0F;

    // CST816 only supports single touch
    if (numPoints > 1) {
        return 0;
    }

    int16_t tmp_x, tmp_y;

    tmp_x = ((buffer[CST8xx_REG_XPOS_HIGH] & 0x0F) << 8 | buffer[CST8xx_REG_XPOS_LOW]);
    tmp_y = ((buffer[CST8xx_REG_YPOS_HIGH] & 0x0F) << 8 | buffer[CST8xx_REG_YPOS_LOW]);

    // Depends on touch screen firmware
    if (tmp_x == _center_btn_x && tmp_y == _center_btn_y && _HButtonCallback) {
        _HButtonCallback(_userData);
        return 0;
    }

    x_array[0] = tmp_x;
    y_array[0] = tmp_y;

    updateXY(numPoints, x_array, y_array);

    return numPoints;
}

bool TouchDrvCST816::isPressed()
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


const char *TouchDrvCST816::getModelName()
{
    switch (_chipID) {
    case CST816S_CHIP_ID:
        return "CST816S";
    case CST816T_CHIP_ID:
        return "CST816T";
    case CST716_CHIP_ID:
        return "CST716";
    case CST820_CHIP_ID:
        return "CST820";
    case CST816D_CHIP_ID:
        return "CST816D";
    default:
        break;
    }
    return "UNKNOW";
}

void TouchDrvCST816::sleep()
{
    comm->writeRegister(CST8xx_REG_SLEEP, 0x03);
#ifdef ARDUINO_ARCH_ESP32
    if (_irq != -1) {
        hal->pinMode(_irq, OPEN_DRAIN);
    }
    if (_rst != -1) {
        hal->pinMode(_rst, OPEN_DRAIN);
    }
#endif
}

void TouchDrvCST816::wakeup()
{
    reset();
}

void TouchDrvCST816::idle()
{

}

uint8_t TouchDrvCST816::getSupportTouchPoint()
{
    return 1;
}

bool TouchDrvCST816::getResolution(int16_t *x, int16_t *y)
{
    return false;
}

void TouchDrvCST816::setHomeButtonCallback(HomeButtonCallback cb, void *user_data)
{
    _HButtonCallback = cb;
    _userData = user_data;
}

void TouchDrvCST816::setCenterButtonCoordinate(int16_t x, int16_t y)
{
    _center_btn_x = x;
    _center_btn_y = y;
}


void TouchDrvCST816::disableAutoSleep()
{
    switch (_chipID) {
    case CST816S_CHIP_ID:
    case CST816T_CHIP_ID:
    case CST820_CHIP_ID:
    case CST816D_CHIP_ID:
        reset();
        hal->delay(50);
        comm->writeRegister(CST8xx_REG_DIS_AUTOSLEEP, 0x01);
        break;
    case CST716_CHIP_ID:
    default:
        break;
    }
}

void TouchDrvCST816::enableAutoSleep()
{
    switch (_chipID) {
    case CST816S_CHIP_ID:
    case CST816T_CHIP_ID:
    case CST820_CHIP_ID:
    case CST816D_CHIP_ID:
        reset();
        hal->delay(50);
        comm->writeRegister(CST8xx_REG_DIS_AUTOSLEEP, (uint8_t)0x00);
        break;
    case CST716_CHIP_ID:
    default:
        break;
    }
}

void TouchDrvCST816::setGpioCallback(CustomMode mode_cb,
                                     CustomWrite write_cb,
                                     CustomRead read_cb)
{
    SensorHalCustom::setCustomMode(mode_cb);
    SensorHalCustom::setCustomWrite(write_cb);
    SensorHalCustom::setCustomRead(read_cb);
}

bool TouchDrvCST816::initImpl()
{

    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
    }

    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    reset();

    int chip_id =   comm->readRegister(CST8xx_REG_CHIP_ID);
    log_i("Chip ID:0x%x", chip_id);

    int version =   comm->readRegister(CST8xx_REG_FW_VERSION);
    log_i("Version :0x%x", version);

    // CST716  : 0x20
    // CST816S : 0xB4
    // CST816T : 0xB5
    // CST816D : 0xB6
    // CST226SE : A7 = 0X20
    if (chip_id != CST816S_CHIP_ID &&
            chip_id != CST816T_CHIP_ID  &&
            chip_id != CST820_CHIP_ID &&
            chip_id != CST816D_CHIP_ID &&
            (chip_id != CST716_CHIP_ID || version == 0)) {
        log_e("Chip ID does not match, should be CST816S:0X%02X , CST816T:0X%02X , CST816D:0X%02X , CST820:0X%02X , CST716:0X%02X",
              CST816S_CHIP_ID, CST816T_CHIP_ID, CST816D_CHIP_ID, CST820_CHIP_ID, CST716_CHIP_ID);
        return false;
    }

    _chipID = chip_id;

    log_i("Touch type:%s", getModelName());

    return true;
}




