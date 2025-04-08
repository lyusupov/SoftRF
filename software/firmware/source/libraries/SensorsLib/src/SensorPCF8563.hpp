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
 * @file      SensorPCF8563.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-09
 *
 */
#pragma once

#include "REG/PCF8563Constants.h"
#include "SensorRTC.h"
#include "SensorPlatform.hpp"

class SensorPCF8563 : public SensorRTC, public PCF8563Constants
{
public:
    using SensorRTC::setDateTime;
    using SensorRTC::getDateTime;

    enum ClockHz {
        CLK_32768HZ,
        CLK_1024HZ,
        CLK_32HZ,
        CLK_1HZ,
        CLK_DISABLE,
    };

    SensorPCF8563() : comm(nullptr) {}

    ~SensorPCF8563()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(wire, PCF8563_SLAVE_ADDRESS, sda, scl);
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
        comm = std::make_unique<SensorCommI2C>(port_num, PCF8563_SLAVE_ADDRESS, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle)
    {
        comm = std::make_unique<SensorCommI2C>(handle, PCF8563_SLAVE_ADDRESS);
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
        comm = std::make_unique<SensorCommCustom>(callback, PCF8563_SLAVE_ADDRESS);
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

        if ((2000 % datetime.getYear()) == 2000) {
            buffer[5] &= 0x7F;
        } else {
            buffer[5] |= 0x80;
        }
        comm->writeRegister(SEC_REG, buffer, 7);
    }

    RTC_DateTime getDateTime()
    {
        uint8_t buffer[7];
        comm->readRegister(SEC_REG, buffer, 7);
        uint8_t second = BCD2DEC(buffer[0] & 0x7F);
        uint8_t minute = BCD2DEC(buffer[1] & 0x7F);
        uint8_t hour   = BCD2DEC(buffer[2] & 0x3F);
        uint8_t day    = BCD2DEC(buffer[3] & 0x3F);
        uint8_t week   = BCD2DEC(buffer[4] & 0x07);
        uint8_t month  = BCD2DEC(buffer[5] & 0x1F);
        uint16_t year   = BCD2DEC(buffer[6]);
        //century :  0 = 1900 , 1 = 2000
        year += (buffer[5] & CENTURY_MASK) ?  1900 : 2000;
        return RTC_DateTime(year, month, day, hour, minute, second, week);
    }


    bool isClockIntegrityGuaranteed()
    {
        return comm->getRegisterBit(SEC_REG, 7) == 0;
    }


    RTC_Alarm getAlarm()
    {
        uint8_t buffer[4];
        comm->readRegister(ALRM_MIN_REG, buffer, 4);
        buffer[0] = BCD2DEC(buffer[0] & 0x80); //minute
        buffer[1] = BCD2DEC(buffer[1] & 0x40); //hour
        buffer[2] = BCD2DEC(buffer[2] & 0x40); //day
        buffer[3] = BCD2DEC(buffer[3] & 0x08); //weekday
        // RTC_Alarm(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t week)
        return RTC_Alarm(buffer[1], buffer[0], 0, buffer[2], buffer[3]);
    }

    void enableAlarm()
    {
        comm->setRegisterBit(STAT2_REG, 1);
    }

    void disableAlarm()
    {
        comm->clrRegisterBit(STAT2_REG, 1);
    }

    void resetAlarm()
    {
        comm->clrRegisterBit(STAT2_REG, 3);
    }

    bool isAlarmActive()
    {
        return comm->getRegisterBit(STAT2_REG, 3);
    }

    void setAlarm(RTC_Alarm alarm)
    {
        setAlarm( alarm.getHour(), alarm.getMinute(), alarm.getDay(), alarm.getWeek());
    }

    void setAlarm(uint8_t hour, uint8_t minute, uint8_t day, uint8_t week)
    {
        uint8_t buffer[4] = {0};

        RTC_DateTime datetime =  getDateTime();

        uint8_t daysInMonth =  getDaysInMonth(datetime.getMonth(), datetime.getYear());

        if (minute != NO_ALARM) {
            if (minute > 59) {
                minute = 59;
            }
            buffer[0] = DEC2BCD(minute);
            buffer[0] &= ~ALARM_ENABLE;
        } else {
            buffer[0] = ALARM_ENABLE;
        }

        if (hour != NO_ALARM) {
            if (hour > 23) {
                hour = 23;
            }
            buffer[1] = DEC2BCD(hour);
            buffer[1] &= ~ALARM_ENABLE;
        } else {
            buffer[1] = ALARM_ENABLE;
        }
        if (day != NO_ALARM) {
            buffer[2] = DEC2BCD(((day) < (1) ? (1) : ((day) > (daysInMonth) ? (daysInMonth) : (day))));
            buffer[2] &= ~ALARM_ENABLE;
        } else {
            buffer[2] = ALARM_ENABLE;
        }
        if (week != NO_ALARM) {
            if (week > 6) {
                week = 6;
            }
            buffer[3] = DEC2BCD(week);
            buffer[3] &= ~ALARM_ENABLE;
        } else {
            buffer[3] = ALARM_ENABLE;
        }
        comm->writeRegister(ALRM_MIN_REG, buffer, 4);
    }

    void setAlarmByMinutes(uint8_t minute)
    {
        setAlarm(NO_ALARM, minute, NO_ALARM, NO_ALARM);
    }
    void setAlarmByDays(uint8_t day)
    {
        setAlarm(NO_ALARM, NO_ALARM, day, NO_ALARM);
    }
    void setAlarmByHours(uint8_t hour)
    {
        setAlarm(hour, NO_ALARM, NO_ALARM, NO_ALARM);
    }
    void setAlarmByWeekDay(uint8_t week)
    {
        setAlarm(NO_ALARM, NO_ALARM, NO_ALARM, week);
    }

    bool isCountdownTimerEnable()
    {
        uint8_t buffer[2];
        buffer[0] = comm->readRegister(STAT2_REG);
        buffer[1] = comm->readRegister(TIMER1_REG);
        if (buffer[0] & TIMER_TIE) {
            return buffer[1] & TIMER_TE ? true : false;
        }
        return false;
    }

    bool isCountdownTimerActive()
    {
        return comm->getRegisterBit(STAT2_REG, 2);
    }

    void enableCountdownTimer()
    {
        comm->setRegisterBit(STAT2_REG, 0);
    }

    void disableCountdownTimer()
    {
        comm->clrRegisterBit(STAT2_REG, 0);
    }

    void setCountdownTimer(uint8_t val, uint8_t freq)
    {
        uint8_t buffer[3];
        buffer[1] = comm->readRegister(TIMER1_REG);
        buffer[1] |= (freq &  TIMER_TD10);
        buffer[2] = val;
        comm->writeRegister(TIMER1_REG, buffer[1]);
        comm->writeRegister(TIMER2_REG, buffer[2]);
    }

    void clearCountdownTimer()
    {
        uint8_t val;
        val = comm->readRegister(STAT2_REG);
        val &= ~(TIMER_TF | TIMER_TIE);
        val |= ALARM_AF;
        comm->writeRegister(STAT2_REG, val);
        comm->writeRegister(TIMER1_REG, (uint8_t)0x00);
    }

    void setClockOutput(ClockHz freq)
    {
        if (freq == CLK_DISABLE) {
            comm->clrRegisterBit(SQW_REG, 7);
        } else {
            comm->writeRegister(SQW_REG,  freq | CLK_ENABLE);
        }
    }

    const char *getChipName()
    {
        return "PCF8563";
    }

private:

    bool initImpl()
    {
        //Check device is online
        int ret = comm->readRegister(SEC_REG);
        if (ret < 0) {
            return false;
        }
        if (BCD2DEC(ret & 0x7F) > 59) {
            return false;
        }
        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
};



