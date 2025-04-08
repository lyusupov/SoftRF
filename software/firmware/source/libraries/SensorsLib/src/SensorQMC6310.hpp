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
 * @file      SensorQMC6310.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */
#pragma once

#include "REG/QMC6310Constants.h"
#include "SensorPlatform.hpp"

static constexpr uint8_t QMC6310U_SLAVE_ADDRESS = 0x1C;
static constexpr uint8_t QMC6310N_SLAVE_ADDRESS = 0x3C;

class Polar
{
public:
    Polar(): polar(0), Gauss(0), uT(0) {}
    Polar(float polar, float Gauss, float uT): polar(polar), Gauss(Gauss), uT(uT) {}
    float polar;
    float Gauss;
    float uT;
};


class SensorQMC6310 :  public QMC6310Constants
{
public:
    enum SensorMode {
        MODE_SUSPEND,
        MODE_NORMAL,
        MODE_SINGLE,
        MODE_CONTINUOUS,
    };

    // Unit:Gauss
    enum MagRange {
        RANGE_30G,
        RANGE_12G,
        RANGE_8G,
        RANGE_2G,
    };

    enum OutputRate {
        DATARATE_10HZ,
        DATARATE_50HZ,
        DATARATE_100HZ,
        DATARATE_200HZ,
    };

    enum CtrlReg {
        SET_RESET_ON,
        SET_ONLY_ON,
        SET_RESET_OFF,
    };

    enum OverSampleRatio {
        OSR_8,
        OSR_4,
        OSR_2,
        OSR_1,
    };

    enum DownSampleRatio {
        DSR_1,
        DSR_2,
        DSR_4,
        DSR_8,
    };

    SensorQMC6310() : comm(nullptr), hal(nullptr) {}

    ~SensorQMC6310()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = QMC6310U_SLAVE_ADDRESS, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = QMC6310U_SLAVE_ADDRESS, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = QMC6310U_SLAVE_ADDRESS)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#endif
#endif

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = QMC6310U_SLAVE_ADDRESS)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl();
    }

    void reset()
    {
        comm->writeRegister(REG_CMD2, (uint8_t)0x80);
        hal->delay(10);
        comm->writeRegister(REG_CMD2, (uint8_t)0x00);
    }

    uint8_t getChipID()
    {
        return comm->readRegister(REG_CHIP_ID);
    }


    int getStatus()
    {
        return comm->readRegister(REG_STAT);
    }

    bool isDataReady()
    {
        if (comm->readRegister(REG_STAT) & 0x01) {
            return true;
        }
        return false;
    }

    bool isDataOverflow()
    {
        if (comm->readRegister(REG_STAT) & 0x02) {
            return true;
        }
        return false;
    }

    void setSelfTest(bool en)
    {
        en ? comm->setRegisterBit(REG_CMD2, 1)
        : comm->clrRegisterBit(REG_CMD2, 1);
    }

    int setMode(SensorMode m)
    {
        return comm->writeRegister(REG_CMD1, 0xFC, m);
    }

    int setCtrlRegister(CtrlReg c)
    {
        return comm->writeRegister(REG_CMD2, 0xFC, c);
    }

    int setDataOutputRate(OutputRate odr)
    {
        return comm->writeRegister(REG_CMD1, 0xF3, (odr << 2));
    }

    int setOverSampleRate(OverSampleRatio osr)
    {
        return comm->writeRegister(REG_CMD1, 0xCF, (osr << 4));
    }

    int setDownSampleRate(DownSampleRatio dsr)
    {
        return comm->writeRegister(REG_CMD1, 0x3F, (dsr << 6));
    }

    // Define the sign for X Y and Z axis
    int setSign(uint8_t x, uint8_t y, uint8_t z)
    {
        int sign = x + y * 2 + z * 4;
        return comm->writeRegister(REG_SIGN, sign);
    }

    int configMagnetometer(SensorMode mode, MagRange range, OutputRate odr,
                           OverSampleRatio osr, DownSampleRatio dsr)
    {
        if (setMagRange(range) < 0) {
            return -1;;
        }
        if (comm->writeRegister(REG_CMD1, 0xFC, mode) < 0) {
            return -1;;
        }
        if (comm->writeRegister(REG_CMD1, 0xF3, (odr << 2)) < 0) {
            return -1;;
        }
        if (comm->writeRegister(REG_CMD1, 0xCF, (osr << 4)) < 0) {
            return -1;;
        }
        if (comm->writeRegister(REG_CMD1, 0x3F, (dsr << 6)) < 0) {
            return -1;;
        }
        return 0;
    }

    int setMagRange(MagRange range)
    {
        switch (range) {
        case RANGE_30G:
            _sensitivity = 0.1;
            break;
        case RANGE_12G:
            _sensitivity = 0.04;
            break;
        case RANGE_8G:
            _sensitivity = 0.026;
            break;
        case RANGE_2G:
            _sensitivity = 0.0066;
            break;
        default:
            break;
        }
        return comm->writeRegister(REG_CMD2, 0xF3, (range << 2));
    }

    void setOffset(int x, int y, int z)
    {
        _x_offset = x; _y_offset = y; _z_offset = z;
    }

    int readData()
    {
        uint8_t buffer[6];
        int16_t x, y, z;
        if (comm->readRegister(REG_LSB_DX, buffer,
                               6) != -1) {
            x = (int16_t)(buffer[1] << 8) | (buffer[0]);
            y = (int16_t)(buffer[3] << 8) | (buffer[2]);
            z = (int16_t)(buffer[5] << 8) | (buffer[4]);

            if (x == 32767) {
                x = -((65535 - x) + 1);
            }
            x = (x - _x_offset);
            if (y == 32767) {
                y = -((65535 - y) + 1);
            }
            y = (y - _y_offset);
            if (z == 32767) {
                z = -((65535 - z) + 1);
            }
            z = (z - _z_offset);

            _raw[0] = x;
            _raw[1] = y;
            _raw[2] = z;

            _mag[0] = (float)x * _sensitivity;
            _mag[1] = (float)y * _sensitivity;
            _mag[2] = (float)z * _sensitivity;
            return 0;
        }
        return -1;;
    }

    void setDeclination(float dec)
    {
        _declination = dec;
    }

    bool readPolar(Polar &p)
    {
        if (isDataReady()) {
            readData();
            float x = getX();
            float y = getY();
            float z = getZ();
            float angle = (atan2(x, -y) / PI) * 180.0 + _declination;
            angle = _convertAngleToPositive(angle);
            float magnitude = sqrt(x * x + y * y + z * z);
            p = Polar(angle, magnitude * 100, magnitude);
            return true;
        }
        return false;
    }

    int16_t getRawX()
    {
        return _raw[0];
    }

    int16_t getRawY()
    {
        return _raw[1];
    }

    int16_t getRawZ()
    {
        return _raw[2];
    }

    float getX()
    {
        return _mag[0];
    }

    float getY()
    {
        return _mag[1];
    }

    float getZ()
    {
        return _mag[2];
    }

    void getMag(float &x, float &y, float &z)
    {
        x = _mag[0];
        y = _mag[1];
        z = _mag[2];
    }

    void dumpCtrlRegister()
    {
        uint8_t buffer[2];
        comm->readRegister(REG_CMD1, buffer, 2);
        for (int i = 0; i < 2; ++i) {
            log_d("CMD%d: 0x%02x", i + 1, buffer[i]);
        }
    }

private:

    float _convertAngleToPositive(float angle)
    {
        if (angle >= 360.0) {
            angle = angle - 360.0;
        }
        if (angle < 0) {
            angle = angle + 360.0;
        }
        return angle;
    }


    bool initImpl()
    {
        reset();
        return getChipID() == QMC6310_CHIP_ID;
    }


protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    int16_t _raw[3];
    float _mag[3];
    float _declination;
    float _sensitivity;
    int16_t _x_offset = 0, _y_offset = 0, _z_offset = 0;
};

