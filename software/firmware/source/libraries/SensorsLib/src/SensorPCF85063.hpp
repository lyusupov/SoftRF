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
 * @file      SensorPCF85063.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-07
 *
 */
#pragma once
#include "REG/PCF85063Constants.h"
#include "SensorRTC.h"
#include "SensorPlatform.hpp"
class SensorPCF85063 : public SensorRTC, public PCF85063Constants
{
public:
    using SensorRTC::setDateTime;
    using SensorRTC::getDateTime;

    enum ClockHz {
        CLK_32768HZ = 0,
        CLK_16384HZ,
        CLK_8192HZ,
        CLK_4096HZ,
        CLK_2048HZ,
        CLK_1024HZ,
        CLK_1HZ,
        CLK_LOW,
    };

    SensorPCF85063() : comm(nullptr) {}

    ~SensorPCF85063()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(wire, PCF85063_SLAVE_ADDRESS, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(port_num, PCF85063_SLAVE_ADDRESS, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle)
    {
        comm = std::make_unique<SensorCommI2C>(handle, PCF85063_SLAVE_ADDRESS);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback)
    {
        comm = std::make_unique<SensorCommCustom>(callback, PCF85063_SLAVE_ADDRESS);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }

    void setDateTime(RTC_DateTime datetime)
    {
        uint8_t buffer[7];
        buffer[0] = DEC2BCD(datetime.getSecond()) & 0x7F;
        buffer[1] = DEC2BCD(datetime.getMinute());
        buffer[2] = DEC2BCD(datetime.getHour());
        buffer[3] = DEC2BCD(datetime.getDay());
        buffer[4] = getDayOfWeek(datetime.getDay(), datetime.getMonth(), datetime.getYear());
        buffer[5] = DEC2BCD(datetime.getMonth());
        buffer[6] = DEC2BCD(datetime.getYear() % 100);

        comm->writeRegister(PCF85063_SEC_REG, buffer, 7);
    }


    RTC_DateTime getDateTime()
    {
        RTC_DateTime datetime;
        uint8_t buffer[7];
        uint8_t hour = 0;
        comm->readRegister(PCF85063_SEC_REG, buffer, 7);
        uint8_t second = BCD2DEC(buffer[0] & 0x7F);
        uint8_t minute = BCD2DEC(buffer[1] & 0x7F);
        if (is24Hour) {
            hour   = BCD2DEC(buffer[2] & 0x3F);    // 24-hour mode
        } else {
            // datetime.AMPM = (buffer[2] & 0x20) == 0x20 ? 'A' : 'P';
            hour   = BCD2DEC(buffer[2] & 0x1F);    // 12-hour mode
        }
        uint8_t day    = BCD2DEC(buffer[3] & 0x3F);
        uint8_t week   = BCD2DEC(buffer[4] & 0x07);
        uint8_t month  = BCD2DEC(buffer[5] & 0x1F);
        uint16_t year   = BCD2DEC(buffer[6]) + 2000;
        return RTC_DateTime(year, month, day, hour, minute, second, week);
    }

    bool isClockIntegrityGuaranteed()
    {
        return comm->getRegisterBit(PCF85063_SEC_REG, 7) == 0;
    }

    /*
    Default use 24H mode
    bool is24HourMode()
    {
        return is24Hour;
    }

    bool is12HourMode()
    {
        return !is24Hour;
    }

    void set24Hour()
    {
        is24Hour = true;
        comm->clrRegisterBit(PCF85063_CTRL1_REG, 1);
    }

    void set12Hour()
    {
        is24Hour = false;
        comm->setRegisterBit(PCF85063_CTRL1_REG, 1);
    }
    */

    void stop()
    {
        comm->setRegisterBit(PCF85063_CTRL1_REG, 5);
    }

    void start()
    {
        comm->clrRegisterBit(PCF85063_CTRL1_REG, 5);
    }

    bool isRunning()
    {
        return !comm->getRegisterBit(PCF85063_CTRL1_REG, 5);
    }

    void enableAlarm()
    {
        comm->setRegisterBit(PCF85063_CTRL2_REG, 7);
    }

    void disableAlarm()
    {
        comm->clrRegisterBit(PCF85063_CTRL2_REG, 7);
    }

    void resetAlarm()
    {
        comm->clrRegisterBit(PCF85063_CTRL2_REG, 6);
    }

    bool isAlarmActive()
    {
        return comm->getRegisterBit(PCF85063_CTRL2_REG, 6);
    }

    RTC_Alarm getAlarm()
    {
        uint8_t buffer[5];
        comm->readRegister(PCF85063_ALRM_MIN_REG, buffer, 5);
        buffer[0] = BCD2DEC(buffer[0] & 0x80);  //second
        buffer[1] = BCD2DEC(buffer[1] & 0x40);  //minute
        buffer[2] = BCD2DEC(buffer[2] & 0x40);  //hour
        buffer[3] = BCD2DEC(buffer[3] & 0x08);  //day
        buffer[4] = BCD2DEC(buffer[4] & 0x08);  //weekday
        return RTC_Alarm(buffer[2], buffer[1], buffer[0], buffer[3], buffer[4]);
    }

    void setAlarm(RTC_Alarm alarm)
    {
        setAlarm(alarm.getHour(), alarm.getMinute(), alarm.getSecond(),
                 alarm.getDay(), alarm.getWeek());
    }

    void setAlarm(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t week)
    {
        uint8_t buffer[5] = {0};

        RTC_DateTime datetime =  getDateTime();

        uint8_t daysInMonth =  getDaysInMonth(datetime.getMonth(), datetime.getYear());

        if (second != PCF85063_NO_ALARM) {
            if (second > 59) {
                second = 59;
            }
            buffer[0] = DEC2BCD(second);
            buffer[0] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[0] = PCF85063_ALARM_ENABLE;
        }

        if (minute != PCF85063_NO_ALARM) {
            if (minute > 59) {
                minute = 59;
            }
            buffer[1] = DEC2BCD(minute);
            buffer[1] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[1] = PCF85063_ALARM_ENABLE;
        }
        if (hour != PCF85063_NO_ALARM) {
            if (is24Hour) {
                if (hour > 23) {
                    hour = 23;
                }
                buffer[2] = DEC2BCD(hour);
                buffer[2] &= ~PCF85063_ALARM_ENABLE;
            } else {
                /*
                if (hour > 12) {
                    hour = 12;
                }
                buffer[2] = DEC2BCD(hour);
                buffer[2] |= isAM ? 0 : _BV(5);
                buffer[2] &= ~PCF85063_ALARM_ENABLE;
                */
            }
        } else {
            buffer[2] = PCF85063_ALARM_ENABLE;
        }
        if (day != PCF85063_NO_ALARM) {
            buffer[3] = DEC2BCD(((day) < (1) ? (1) : ((day) > (daysInMonth) ? (daysInMonth) : (day))));
            buffer[3] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[3] = PCF85063_ALARM_ENABLE;
        }
        if (week != PCF85063_NO_ALARM) {
            if (week > 6) {
                week = 6;
            }
            buffer[4] = DEC2BCD(week);
            buffer[4] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[4] = PCF85063_ALARM_ENABLE;
        }
        comm->writeRegister(PCF85063_ALRM_SEC_REG, buffer, 4);
    }

    void setAlarmByHours(uint8_t hour)
    {
        setAlarm(hour,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM);
    }

    void setAlarmBySecond(uint8_t second)
    {
        setAlarm(PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 second,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM);
    }

    void setAlarmByMinutes(uint8_t minute)
    {
        setAlarm(PCF85063_NO_ALARM,
                 minute,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM);
    }

    void setAlarmByDays(uint8_t day)
    {
        setAlarm(PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 day,
                 PCF85063_NO_ALARM);
    }

    void setAlarmByWeekDay(uint8_t week)
    {
        setAlarm(PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 week);
    }

    void setClockOutput(ClockHz hz)
    {
        int val = comm->readRegister(PCF85063_CTRL2_REG);
        if (val == -1)return;
        val &= 0xF8;
        val |= hz;
        comm->writeRegister(PCF85063_CTRL2_REG, val);
    }

    const char *getChipName()
    {
        return "PCF85063";
    }

private:

    bool initImpl()
    {
        //Check device is online
        int val =  comm->readRegister(PCF85063_RAM_REG);
        if (val < 0) {
            log_e("Device is offline!");
            return false;
        }
        // Read the contents of a RAM register
        uint8_t tmp = comm->readRegister(PCF85063_RAM_REG);

        bool rlst = false;
        // By judging whether the highest bit of the RAM register can be changed,
        // it can be judged whether it belongs to PCF85063
        comm->writeRegister(PCF85063_RAM_REG, val | _BV(7));
        val =  comm->readRegister(PCF85063_RAM_REG);
        if (val & 0x80) {
            comm->writeRegister(PCF85063_RAM_REG, val & ~_BV(7));
            val =  comm->readRegister(PCF85063_RAM_REG);
            if ((val & 0x80) == 0) {
                rlst = true;
            }
        }

        if (!rlst) {
            log_e("Failed to write to RAM memory register. Maybe this chip is pcf8563.");
            return false;
        }

        // Restore the contents of the RAM registers
        comm->writeRegister(PCF85063_RAM_REG, tmp);

        //Default use 24-hour mode
        is24Hour = !comm->getRegisterBit(PCF85063_CTRL1_REG, 1);
        if (!is24Hour) {
            // Set 24H Mode
            comm->clrRegisterBit(PCF85063_CTRL1_REG, 1);
            is24Hour = true;
        }

        //Turn on RTC
        start();

        return isRunning();
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
    bool is24Hour;
};



