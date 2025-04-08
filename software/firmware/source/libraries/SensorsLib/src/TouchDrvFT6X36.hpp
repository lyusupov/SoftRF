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
 * @file      TouchDrvFT6X36.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-01
 *
 */
#pragma once

#include "REG/FT6X36Constants.h"
#include "TouchDrvInterface.hpp"

class TouchDrvFT6X36 :  public TouchDrvInterface
{
public:
    enum GesTrue {
        NO_GESTURE,
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_RIGHT,
        ZOOM_IN,
        ZOOM_OUT,
    } ;

    enum EventFlag {
        EVENT_PUT_DOWN,
        EVENT_PUT_UP,
        EVENT_CONTACT,
        EVENT_NONE,
    } ;

    enum PowerMode {
        PMODE_ACTIVE = 0,         // ~4mA
        PMODE_MONITOR = 1,        // ~3mA
        PMODE_DEEP_SLEEP = 3,     // ~100uA  The reset pin must be pulled down to wake up
    } ;

    EventFlag event;

    TouchDrvFT6X36() : comm(nullptr), hal(nullptr) {}

    ~TouchDrvFT6X36()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = FT6X36_SLAVE_ADDRESS, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl();
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = FT6X36_SLAVE_ADDRESS, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = FT6X36_SLAVE_ADDRESS)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
            return false;
        }
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl();
    }

    uint8_t getDeviceMode(void)
    {
        return comm->readRegister(FT6X36_REG_MODE) & 0x03;
    }

    // Obtaining gestures depends on whether the built-in firmware of the chip has this function
    uint8_t getGesture()
    {
        int val = comm->readRegister(FT6X36_REG_GEST);
        switch (val) {
        case 0x10:
            return MOVE_UP;
        case 0x14:
            return MOVE_RIGHT;
        case 0x18:
            return MOVE_DOWN;
        case 0x1C:
            return MOVE_LEFT;
        case 0x48:
            return ZOOM_IN;
        case 0x49:
            return ZOOM_OUT;
        default:
            break;
        }
        return NO_GESTURE;
    }

    void setThreshold(uint8_t value)
    {
        comm->writeRegister(FT6X36_REG_THRESHOLD, value);
    }

    uint8_t getThreshold(void)
    {
        return comm->readRegister(FT6X36_REG_THRESHOLD);
    }

    uint8_t getMonitorTime(void)
    {
        return comm->readRegister(FT6X36_REG_MONITOR_TIME);
    }

    void setMonitorTime(uint8_t sec)
    {
        comm->writeRegister(FT6X36_REG_MONITOR_TIME, sec);
    }

    // Calibration useless actually,
    // any value  set will not be valid,
    // depending on the internal firmware of the chip.
    /*
    void enableAutoCalibration(void)
    {
        comm->writeRegister(FT6X36_REG_AUTO_CLB_MODE, 0x00);
    }

    void disableAutoCalibration(void)
    {
        comm->writeRegister(FT6X36_REG_AUTO_CLB_MODE, 0xFF);
    }
    */

    uint16_t getLibraryVersion()
    {
        uint8_t buffer[2];
        comm->readRegister(FT6X36_REG_LIB_VERSION_H, buffer, 2);
        return (buffer[0] << 8) | buffer[1];
    }

    // The interrupt is triggered only if a touch is detected during the scan cycle
    void interruptPolling(void)
    {
        //datasheet this bit is 0,Actually, it's wrong
        comm->writeRegister(FT6X36_REG_INT_STATUS, 1);
    }

    // Triggers an interrupt whenever a touch is detected
    void interruptTrigger(void)
    {
        //datasheet this bit is 1,Actually, it's wrong
        comm->writeRegister(FT6X36_REG_INT_STATUS, (uint8_t)0);
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t size = 1)
    {
        uint8_t buffer[16];

        if (!x_array || !y_array || !size)
            return 0;

        if (comm->readRegister(FT6X36_REG_MODE, buffer, 16) == -1) {
            return 0;
        }

        // uint8_t mode = buffer[0];
        //REG 0x01
        // uint8_t gesture = buffer[1];
        //REG 0x02
        uint8_t numPoints = buffer[2] & 0x0F;
        if (numPoints == 0 || numPoints == 0x0F) {
            return 0;
        }

        //REG 0x03 ~ 0x04
        // uint8_t eventFlag = (buffer[3] & 0xC0) >> 6;
        uint16_t posX = ((buffer[3] & 0x0F) << 8) | buffer[4];
        //REG 0x05 ~ 0x06
        uint16_t posY = ((buffer[5] & 0x0F) << 8) | buffer[6] ;


        x_array[0] = posX;
        y_array[0] = posY;

        if (numPoints == 2) {
            //REG 0x09 ~ 0x0A
            posX = ((buffer[9] & 0x0F) << 8) | buffer[10];
            //REG 0x0B ~ 0x0C
            posY = ((buffer[11] & 0x0F) << 8) | buffer[12] ;

            if (size == 2) {
                x_array[1] = posX;
                y_array[1] = posY;
            }
        }
        updateXY(numPoints, x_array, y_array);

        return numPoints;
    }

    bool isPressed()
    {
        if (_irq != -1) {
            return hal->digitalRead(_irq) == LOW;
        }
        return comm->readRegister(FT6X36_REG_STATUS) & 0x0F;
    }

    void setPowerMode(PowerMode mode)
    {
        comm->writeRegister(FT6X36_REG_POWER_MODE, mode);
    }

    void sleep()
    {
        comm->writeRegister(FT6X36_REG_POWER_MODE, PMODE_DEEP_SLEEP);
    }

    void wakeup()
    {
        reset();
    }

    void idle()
    {

    }

    uint8_t getSupportTouchPoint()
    {
        return 1;
    }

    uint32_t getChipID(void)
    {
        return comm->readRegister(FT6X36_REG_CHIP_ID);
    }

    uint8_t getVendorID(void)
    {
        return comm->readRegister(FT6X36_REG_VENDOR1_ID);
    }

    uint8_t getErrorCode(void)
    {
        return comm->readRegister(FT6X36_REG_ERROR_STATUS);
    }

    const char *getModelName()
    {
        switch (_chipID) {
        case FT6206_CHIP_ID: return "FT6206";
        case FT6236_CHIP_ID: return "FT6236";
        case FT6236U_CHIP_ID: return "FT6236U";
        case FT3267_CHIP_ID: return "FT3267";
        default: return "UNKNOWN";
        }
    }


    bool getResolution(int16_t *x, int16_t *y)
    {
        return false;
    }

    void reset()
    {
        if (_rst != -1) {
            hal->pinMode(_rst, OUTPUT);
            hal->digitalWrite(_rst, HIGH);
            hal->delay(10);
            hal->digitalWrite(_rst, LOW);
            hal->delay(30);
            hal->digitalWrite(_rst, HIGH);
            // For the variant of GPIO extended RST,
            // communication and hal->delay are carried out simultaneously, and 160ms is measured in T-RGB esp-idf new api
            hal->delay(160);
        }
    }


    void  setGpioCallback(CustomMode mode_cb,
                          CustomWrite write_cb,
                          CustomRead read_cb)
    {
        SensorHalCustom::setCustomMode(mode_cb);
        SensorHalCustom::setCustomWrite(write_cb);
        SensorHalCustom::setCustomRead(read_cb);
    }

private:
    bool initImpl()
    {
        if (_irq != -1) {
            hal->pinMode(_irq, INPUT);
        }

        reset();

        uint8_t vendId = comm->readRegister(FT6X36_REG_VENDOR1_ID);


        if (vendId != FT6X36_VEND_ID) {
            log_e("Vendor id is 0x%X not match!", vendId);
            return false;
        }

        _chipID = comm->readRegister(FT6X36_REG_CHIP_ID);

        if ((_chipID != FT6206_CHIP_ID) &&
                (_chipID != FT6236_CHIP_ID) &&
                (_chipID != FT6236U_CHIP_ID)  &&
                (_chipID != FT3267_CHIP_ID)
           ) {
            log_e("Vendor id is not match!");
            log_e("ChipID:0x%lx should be 0x06 or 0x36 or 0x64", _chipID);
            return false;
        }

        log_i("Vend ID: 0x%X", vendId);
        log_i("Chip ID: 0x%lx", _chipID);
        log_i("Firm Version: 0x%X", comm->readRegister(FT6X36_REG_FIRM_VERS));
        log_i("Point Rate Hz: %u", comm->readRegister(FT6X36_REG_PERIOD_ACTIVE));
        log_i("Thresh : %u", comm->readRegister(FT6X36_REG_THRESHOLD));

        // change threshold to be higher/lower
        comm->writeRegister(FT6X36_REG_THRESHOLD, 60);

        log_i("Chip library version : 0x%x", getLibraryVersion());

        // This register describes period of monitor status, it should not less than 30.
        log_i("Chip period of monitor status : 0x%x", comm->readRegister(FT6X36_REG_PERIOD_MONITOR));

        // This register describes the period of active status, it should not less than 12


        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
};



