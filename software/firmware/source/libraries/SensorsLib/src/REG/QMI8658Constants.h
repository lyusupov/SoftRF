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
 * @file      QMI8658Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */
#pragma once

#include <stdint.h>

// @brief  device address
#define QMI8658_L_SLAVE_ADDRESS                 (0x6B)
#define QMI8658_H_SLAVE_ADDRESS                 (0x6A)

class QMI8658Constants
{
protected:

    // @brief  registers default value
    static constexpr uint8_t QMI8658_REG_WHOAMI_DEFAULT = 0x05;
    static constexpr uint8_t QMI8658_REG_STATUS_DEFAULT = 0x03;
    static constexpr uint8_t QMI8658_REG_RESET_DEFAULT = 0xB0;


    //* General Purpose Registers
    static constexpr uint8_t QMI8658_REG_WHOAMI = 0x00;
    static constexpr uint8_t QMI8658_REG_REVISION = 0x01;


    //* Setup and Control Registers
    static constexpr uint8_t QMI8658_REG_CTRL1 = 0x02;
    static constexpr uint8_t QMI8658_REG_CTRL2 = 0x03;
    static constexpr uint8_t QMI8658_REG_CTRL3 = 0x04;
    // Reserved
    static constexpr uint8_t QMI8658_REG_CTRL5 = 0x06;
    // Reserved
    static constexpr uint8_t QMI8658_REG_CTRL7 = 0x08;
    static constexpr uint8_t QMI8658_REG_CTRL8 = 0x09;
    static constexpr uint8_t QMI8658_REG_CTRL9 = 0x0A;

    //* Host Controlled Calibration Registers (See CTRL9, Usage is Optional)
    static constexpr uint8_t QMI8658_REG_CAL1_L = 0x0B;
    static constexpr uint8_t QMI8658_REG_CAL1_H = 0x0C;
    static constexpr uint8_t QMI8658_REG_CAL2_L = 0x0D;
    static constexpr uint8_t QMI8658_REG_CAL2_H = 0x0E;
    static constexpr uint8_t QMI8658_REG_CAL3_L = 0x0F;
    static constexpr uint8_t QMI8658_REG_CAL3_H = 0x10;
    static constexpr uint8_t QMI8658_REG_CAL4_L = 0x11;
    static constexpr uint8_t QMI8658_REG_CAL4_H = 0x12;

    //* FIFO Registers
    static constexpr uint8_t QMI8658_REG_FIFO_WTM_TH = 0x13;
    static constexpr uint8_t QMI8658_REG_FIFO_CTRL = 0x14;
    static constexpr uint8_t QMI8658_REG_FIFO_COUNT = 0x15;
    static constexpr uint8_t QMI8658_REG_FIFO_STATUS = 0x16;
    static constexpr uint8_t QMI8658_REG_FIFO_DATA = 0x17;

    //* Status Registers
    static constexpr uint8_t QMI8658_REG_STATUS_INT = 0x2D;
    static constexpr uint8_t QMI8658_REG_STATUS0 = 0x2E;
    static constexpr uint8_t QMI8658_REG_STATUS1 = 0x2F;

    //* Timestamp Register
    static constexpr uint8_t QMI8658_REG_TIMESTAMP_L = 0x30;
    static constexpr uint8_t QMI8658_REG_TIMESTAMP_M = 0x31;
    static constexpr uint8_t QMI8658_REG_TIMESTAMP_H = 0x32;

    //* Data Output Registers (16 bits 2’s Complement Except COD Sensor Data)
    static constexpr uint8_t QMI8658_REG_TEMPERATURE_L = 0x33;
    static constexpr uint8_t QMI8658_REG_TEMPERATURE_H = 0x34;
    static constexpr uint8_t QMI8658_REG_AX_L = 0x35;
    static constexpr uint8_t QMI8658_REG_AX_H = 0x36;
    static constexpr uint8_t QMI8658_REG_AY_L = 0x37;
    static constexpr uint8_t QMI8658_REG_AY_H = 0x38;
    static constexpr uint8_t QMI8658_REG_AZ_L = 0x39;
    static constexpr uint8_t QMI8658_REG_AZ_H = 0x3A;
    static constexpr uint8_t QMI8658_REG_GX_L = 0x3B;
    static constexpr uint8_t QMI8658_REG_GX_H = 0x3C;
    static constexpr uint8_t QMI8658_REG_GY_L = 0x3D;
    static constexpr uint8_t QMI8658_REG_GY_H = 0x3E;
    static constexpr uint8_t QMI8658_REG_GZ_L = 0x3F;
    static constexpr uint8_t QMI8658_REG_GZ_H = 0x40;

    //* COD Indication and General Purpose Registers

    // Calibration-On-Demand status register
    static constexpr uint8_t QMI8658_REG_COD_STATUS = 0x46;
    static constexpr uint8_t QMI8658_REG_DQW_L = 0x49;
    static constexpr uint8_t QMI8658_REG_DQW_H = 0x4A;
    static constexpr uint8_t QMI8658_REG_DQX_L = 0x4B;
    static constexpr uint8_t QMI8658_REG_DQX_H = 0x4C;

    static constexpr uint8_t QMI8658_REG_DQY_L = 0x4D;
    static constexpr uint8_t QMI8658_REG_DQY_H = 0x4E;
    static constexpr uint8_t QMI8658_REG_DQZ_L = 0x4F;
    static constexpr uint8_t QMI8658_REG_DQZ_H = 0x50;

    static constexpr uint8_t QMI8658_REG_DVX_L = 0x51;
    static constexpr uint8_t QMI8658_REG_DVX_H = 0x52;
    static constexpr uint8_t QMI8658_REG_DVY_L = 0x53;
    static constexpr uint8_t QMI8658_REG_DVY_H = 0x54;
    static constexpr uint8_t QMI8658_REG_DVZ_L = 0x55;
    static constexpr uint8_t QMI8658_REG_DVZ_H = 0x56;

    //* Activity Detection Output Registers
    static constexpr uint8_t QMI8658_REG_TAP_STATUS = 0x59;
    static constexpr uint8_t QMI8658_REG_STEP_CNT_LOW = 0x5A;
    static constexpr uint8_t QMI8658_REG_STEP_CNT_MID = 0x5B;
    static constexpr uint8_t QMI8658_REG_STEP_CNT_HIGH = 0x5C;
    static constexpr uint8_t QMI8658_REG_RESET = 0x60;

    //* Reset Register
    static constexpr uint8_t QMI8658_REG_RST_RESULT = 0x4D;
    static constexpr uint8_t QMI8658_REG_RST_RESULT_VAL = 0x80;

    static constexpr uint8_t STATUS0_ACCEL_AVAIL = 0x01;
    static constexpr uint8_t STATUS0_GYRO_AVAIL = 0x02;
    static constexpr uint8_t QMI8658_ACCEL_LPF_MASK = 0xF9;
    static constexpr uint8_t QMI8658_GYRO_LPF_MASK = 0x9F;

    static constexpr uint8_t QMI8658_ACCEL_EN_MASK = 0x01;
    static constexpr uint8_t QMI8658_GYRO_EN_MASK = 0x02;
    static constexpr uint8_t QMI8658_ACCEL_GYRO_EN_MASK = 0x03;


    static constexpr uint8_t QMI8658_FIFO_MAP_INT1 = 0x04;    // ctrl1
};