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
 * @file      MPU6886Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */

#pragma once


#define MPU6886_SLAVE_ADDRESS               (0x68)

#define MPU6886_REG_XG_OFFS_TC_H            ((uint8_t)0x04)
#define MPU6886_REG_XG_OFFS_TC_L            ((uint8_t)0x05)
#define MPU6886_REG_YG_OFFS_TC_H            ((uint8_t)0x07)
#define MPU6886_REG_YG_OFFS_TC_L            ((uint8_t)0x08)
#define MPU6886_REG_ZG_OFFS_TC_H            ((uint8_t)0x0A)
#define MPU6886_REG_ZG_OFFS_TC_L            ((uint8_t)0x0B)
#define MPU6886_REG_SELF_TEST_X_ACCEL       ((uint8_t)0x0D)
#define MPU6886_REG_SELF_TEST_Y_ACCEL       ((uint8_t)0x0E)
#define MPU6886_REG_SELF_TEST_Z_ACCEL       ((uint8_t)0x0F)
#define MPU6886_REG_XG_OFFS_USRH            ((uint8_t)0x13)
#define MPU6886_REG_XG_OFFS_USRL            ((uint8_t)0x14)
#define MPU6886_REG_YG_OFFS_USRH            ((uint8_t)0x15)
#define MPU6886_REG_YG_OFFS_USRL            ((uint8_t)0x16)
#define MPU6886_REG_ZG_OFFS_USRH            ((uint8_t)0x17)
#define MPU6886_REG_ZG_OFFS_USRL            ((uint8_t)0x18)
#define MPU6886_REG_SMPLRT_DIV              ((uint8_t)0x19)
#define MPU6886_REG_CONFIG                  ((uint8_t)0x1A)
#define MPU6886_REG_GYRO_CONFIG             ((uint8_t)0x1B)
#define MPU6886_REG_ACCEL_CONFIG            ((uint8_t)0x1C)
#define MPU6886_REG_ACCEL_CONFIG_2          ((uint8_t)0x1D)
#define MPU6886_REG_LP_MODE_CFG             ((uint8_t)0x1E)
#define MPU6886_REG_ACCEL_WOM_X_THR         ((uint8_t)0x20)
#define MPU6886_REG_ACCEL_WOM_Y_THR         ((uint8_t)0x21)
#define MPU6886_REG_ACCEL_WOM_Z_THR         ((uint8_t)0x22)
#define MPU6886_REG_FIFO_EN                 ((uint8_t)0x23)
#define MPU6886_REG_FSYNC_INT               ((uint8_t)0x36)
#define MPU6886_REG_INT_PIN_CFG             ((uint8_t)0x37)
#define MPU6886_REG_INT_ENABLE              ((uint8_t)0x38)
#define MPU6886_REG_FIFO_WM_INT_STATUS      ((uint8_t)0x39)
#define MPU6886_REG_INT_STATUS              ((uint8_t)0x3A)
#define MPU6886_REG_ACCEL_XOUT_H            ((uint8_t)0x3B)
#define MPU6886_REG_ACCEL_XOUT_L            ((uint8_t)0x3C)
#define MPU6886_REG_ACCEL_YOUT_H            ((uint8_t)0x3D)
#define MPU6886_REG_ACCEL_YOUT_L            ((uint8_t)0x3E)
#define MPU6886_REG_ACCEL_ZOUT_H            ((uint8_t)0x3F)
#define MPU6886_REG_ACCEL_ZOUT_L            ((uint8_t)0x40)
#define MPU6886_REG_TEMP_OUT_H              ((uint8_t)0x41)
#define MPU6886_REG_TEMP_OUT_L              ((uint8_t)0x42)
#define MPU6886_REG_GYRO_XOUT_H             ((uint8_t)0x43)
#define MPU6886_REG_GYRO_XOUT_L             ((uint8_t)0x44)
#define MPU6886_REG_GYRO_YOUT_H             ((uint8_t)0x45)
#define MPU6886_REG_GYRO_YOUT_L             ((uint8_t)0x46)
#define MPU6886_REG_GYRO_ZOUT_H             ((uint8_t)0x47)
#define MPU6886_REG_GYRO_ZOUT_L             ((uint8_t)0x48)
#define MPU6886_REG_SELF_TEST_X_GYRO        ((uint8_t)0x50)
#define MPU6886_REG_SELF_TEST_Y_GYRO        ((uint8_t)0x51)
#define MPU6886_REG_SELF_TEST_Z_GYRO        ((uint8_t)0x52)
#define MPU6886_REG_E_ID0                   ((uint8_t)0x53)
#define MPU6886_REG_E_ID1                   ((uint8_t)0x54)
#define MPU6886_REG_E_ID2                   ((uint8_t)0x55)
#define MPU6886_REG_E_ID3                   ((uint8_t)0x56)
#define MPU6886_REG_E_ID4                   ((uint8_t)0x57)
#define MPU6886_REG_E_ID5                   ((uint8_t)0x58)
#define MPU6886_REG_E_ID6                   ((uint8_t)0x59)
#define MPU6886_REG_FIFO_WM_TH1             ((uint8_t)0x60)
#define MPU6886_REG_FIFO_WM_TH2             ((uint8_t)0x61)
#define MPU6886_REG_SIGNAL_PATH_RESET       ((uint8_t)0x68)
#define MPU6886_REG_ACCEL_INTEL_CTRL        ((uint8_t)0x69)
#define MPU6886_REG_USER_CTRL               ((uint8_t)0x6A)
#define MPU6886_REG_PWR_MGMT_1              ((uint8_t)0x6B)
#define MPU6886_REG_PWR_MGMT_2              ((uint8_t)0x6C)
#define MPU6886_REG_I2C_IF                  ((uint8_t)0x70)
#define MPU6886_REG_FIFO_COUNTH             ((uint8_t)0x72)
#define MPU6886_REG_FIFO_COUNTL             ((uint8_t)0x73)
#define MPU6886_REG_FIFO_R_W                ((uint8_t)0x74)
#define MPU6886_REG_WHO_AM_I                ((uint8_t)0x75)
#define MPU6886_REG_XA_OFFSET_H             ((uint8_t)0x77)
#define MPU6886_REG_XA_OFFSET_L             ((uint8_t)0x78)
#define MPU6886_REG_YA_OFFSET_H             ((uint8_t)0x7A)
#define MPU6886_REG_YA_OFFSET_L             ((uint8_t)0x7B)
#define MPU6886_REG_ZA_OFFSET_H             ((uint8_t)0x7D)
#define MPU6886_REG_ZA_OFFSET_L             ((uint8_t)0x7E)


#define MPU6886_WHO_AM_I_RES                ((uint8_t)0x19)





