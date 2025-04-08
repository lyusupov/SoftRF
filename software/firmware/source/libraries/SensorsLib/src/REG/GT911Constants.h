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
 * @file      GT911Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */

#pragma once

#include <stdint.h>

#define GT911_SLAVE_ADDRESS_H               (0x14)
#define GT911_SLAVE_ADDRESS_L               (0x5D)
#define GT911_SLAVE_ADDRESS_UNKOWN          (0xFF)

class GT911Constants
{
protected:
    // Real-time command (Write only)
    static constexpr uint16_t  GT911_COMMAND                 = (0x8040);
    static constexpr uint16_t  GT911_ESD_CHECK               = (0x8041);
    static constexpr uint16_t  GT911_COMMAND_CHECK           = (0x8046);

    // Configuration information (R/W)
    static constexpr uint16_t  GT911_CONFIG_START            = (0x8047);
    static constexpr uint16_t  GT911_CONFIG_VERSION          = (0x8047);
    static constexpr uint16_t  GT911_X_OUTPUT_MAX_LOW        = (0x8048);
    static constexpr uint16_t  GT911_X_OUTPUT_MAX_HIGH       = (0x8049);
    static constexpr uint16_t  GT911_Y_OUTPUT_MAX_LOW        = (0x804A);
    static constexpr uint16_t  GT911_Y_OUTPUT_MAX_HIGH       = (0x804B);
    static constexpr uint16_t  GT911_TOUCH_NUMBER            = (0x804C);
    static constexpr uint16_t  GT911_MODULE_SWITCH_1         = (0x804D);
    static constexpr uint16_t  GT911_MODULE_SWITCH_2         = (0x804E);
    static constexpr uint16_t  GT911_SHAKE_COUNT             = (0x804F);
    static constexpr uint16_t  GT911_FILTER                  = (0x8050);
    static constexpr uint16_t  GT911_LARGE_TOUCH             = (0x8051);
    static constexpr uint16_t  GT911_NOISE_REDUCTION         = (0x8052);
    static constexpr uint16_t  GT911_SCREEN_TOUCH_LEVEL      = (0x8053);
    static constexpr uint16_t  GT911_SCREEN_RELEASE_LEVEL    = (0x8054);
    static constexpr uint16_t  GT911_LOW_POWER_CONTROL       = (0x8055);
    static constexpr uint16_t  GT911_REFRESH_RATE            = (0x8056);
    static constexpr uint16_t  GT911_X_THRESHOLD             = (0x8057);
    static constexpr uint16_t  GT911_Y_THRESHOLD             = (0x8058);
    static constexpr uint16_t  GT911_X_SPEED_LIMIT           = (0x8059); // Reserve
    static constexpr uint16_t  GT911_Y_SPEED_LIMIT           = (0x805A); // Reserve
    static constexpr uint16_t  GT911_SPACE_TOP_BOTTOM        = (0x805B);
    static constexpr uint16_t  GT911_SPACE_LEFT_RIGHT        = (0x805C);
    static constexpr uint16_t  GT911_MINI_FILTER             = (0x805D);
    static constexpr uint16_t  GT911_STRETCH_R0              = (0x805E);
    static constexpr uint16_t  GT911_STRETCH_R1              = (0x805F);
    static constexpr uint16_t  GT911_STRETCH_R2              = (0x8060);
    static constexpr uint16_t  GT911_STRETCH_RM              = (0x8061);
    static constexpr uint16_t  GT911_DRV_GROUPA_NUM          = (0x8062);
    static constexpr uint16_t  GT911_DRV_GROUPB_NUM          = (0x8063);
    static constexpr uint16_t  GT911_SENSOR_NUM              = (0x8064);
    static constexpr uint16_t  GT911_FREQ_A_FACTOR           = (0x8065);
    static constexpr uint16_t  GT911_FREQ_B_FACTOR           = (0x8066);
    static constexpr uint16_t  GT911_PANEL_BIT_FREQ_L        = (0x8067);
    static constexpr uint16_t  GT911_PANEL_BIT_FREQ_H        = (0x8068);
    static constexpr uint16_t  GT911_PANEL_SENSOR_TIME_L     = (0x8069); // Reserve
    static constexpr uint16_t  GT911_PANEL_SENSOR_TIME_H     = (0x806A);
    static constexpr uint16_t  GT911_PANEL_TX_GAIN           = (0x806B);
    static constexpr uint16_t  GT911_PANEL_RX_GAIN           = (0x806C);
    static constexpr uint16_t  GT911_PANEL_DUMP_SHIFT        = (0x806D);
    static constexpr uint16_t  GT911_DRV_FRAME_CONTROL       = (0x806E);
    static constexpr uint16_t  GT911_CHARGING_LEVEL_UP       = (0x806F);
    static constexpr uint16_t  GT911_MODULE_SWITCH3          = (0x8070);
    static constexpr uint16_t  GT911_GESTURE_DIS             = (0X8071);
    static constexpr uint16_t  GT911_GESTURE_LONG_PRESS_TIME = (0x8072);
    static constexpr uint16_t  GT911_X_Y_SLOPE_ADJUST        = (0X8073);
    static constexpr uint16_t  GT911_GESTURE_CONTROL         = (0X8074);
    static constexpr uint16_t  GT911_GESTURE_SWITCH1         = (0X8075);
    static constexpr uint16_t  GT911_GESTURE_SWITCH2         = (0X8076);
    static constexpr uint16_t  GT911_GESTURE_REFRESH_RATE    = (0x8077);
    static constexpr uint16_t  GT911_GESTURE_TOUCH_LEVEL     = (0x8078);
    static constexpr uint16_t  GT911_NEWGREENWAKEUPLEVEL     = (0x8079);
    static constexpr uint16_t  GT911_FREQ_HOPPING_START      = (0x807A);
    static constexpr uint16_t  GT911_FREQ_HOPPING_END        = (0X807B);
    static constexpr uint16_t  GT911_NOISE_DETECT_TIMES      = (0x807C);
    static constexpr uint16_t  GT911_HOPPING_FLAG            = (0X807D);
    static constexpr uint16_t  GT911_HOPPING_THRESHOLD       = (0X807E);
    static constexpr uint16_t  GT911_NOISE_THRESHOLD         = (0X807F); // Reserve
    static constexpr uint16_t  GT911_NOISE_MIN_THRESHOLD     = (0X8080);
    static constexpr uint16_t  GT911_HOPPING_SENSOR_GROUP    = (0X8082);
    static constexpr uint16_t  GT911_HOPPING_SEG1_NORMALIZE  = (0X8083);
    static constexpr uint16_t  GT911_HOPPING_SEG1_FACTOR     = (0X8084);
    static constexpr uint16_t  GT911_MAIN_CLOCK_ADJUST       = (0X8085);
    static constexpr uint16_t  GT911_HOPPING_SEG2_NORMALIZE  = (0X8086);
    static constexpr uint16_t  GT911_HOPPING_SEG2_FACTOR     = (0X8087);
    static constexpr uint16_t  GT911_HOPPING_SEG3_NORMALIZE  = (0X8089);
    static constexpr uint16_t  GT911_HOPPING_SEG3_FACTOR     = (0X808A);
    static constexpr uint16_t  GT911_HOPPING_SEG4_NORMALIZE  = (0X808C);
    static constexpr uint16_t  GT911_HOPPING_SEG4_FACTOR     = (0X808D);
    static constexpr uint16_t  GT911_HOPPING_SEG5_NORMALIZE  = (0X808F);
    static constexpr uint16_t  GT911_HOPPING_SEG5_FACTOR     = (0X8090);
    static constexpr uint16_t  GT911_HOPPING_SEG6_NORMALIZE  = (0X8092);
    static constexpr uint16_t  GT911_KEY_1                   = (0X8093);
    static constexpr uint16_t  GT911_KEY_2                   = (0X8094);
    static constexpr uint16_t  GT911_KEY_3                   = (0X8095);
    static constexpr uint16_t  GT911_KEY_4                   = (0X8096);
    static constexpr uint16_t  GT911_KEY_AREA                = (0X8097);
    static constexpr uint16_t  GT911_KEY_TOUCH_LEVEL         = (0X8098);
    static constexpr uint16_t  GT911_KEY_LEAVE_LEVEL         = (0X8099);
    static constexpr uint16_t  GT911_KEY_SENS_1_2            = (0X809A);
    static constexpr uint16_t  GT911_KEY_SENS_3_4            = (0X809B);
    static constexpr uint16_t  GT911_KEY_RESTRAIN            = (0X809C);
    static constexpr uint16_t  GT911_KEY_RESTRAIN_TIME       = (0X809D);
    static constexpr uint16_t  GT911_GESTURE_LARGE_TOUCH     = (0X809E);
    static constexpr uint16_t  GT911_HOTKNOT_NOISE_MAP       = (0X80A1);
    static constexpr uint16_t  GT911_LINK_THRESHOLD          = (0X80A2);
    static constexpr uint16_t  GT911_PXY_THRESHOLD           = (0X80A3);
    static constexpr uint16_t  GT911_GHOT_DUMP_SHIFT         = (0X80A4);
    static constexpr uint16_t  GT911_GHOT_RX_GAIN            = (0X80A5);
    static constexpr uint16_t  GT911_FREQ_GAIN0              = (0X80A6);
    static constexpr uint16_t  GT911_FREQ_GAIN1              = (0X80A7);
    static constexpr uint16_t  GT911_FREQ_GAIN2              = (0X80A8);
    static constexpr uint16_t  GT911_FREQ_GAIN3              = (0X80A9);
    static constexpr uint16_t  GT911_COMBINE_DIS             = (0X80B3);
    static constexpr uint16_t  GT911_SPLIT_SET               = (0X80B4);
    static constexpr uint16_t  GT911_SENSOR_CH0              = (0X80B7);
    static constexpr uint16_t  GT911_DRIVER_CH0              = (0X80D5);
    static constexpr uint16_t  GT911_CONFIG_CHKSUM           = (0X80FF);
    static constexpr uint16_t  GT911_CONFIG_FRESH            = (0X8100);
    static constexpr uint16_t  GT911_CONFIG_SIZE             = (0xFF - 0x46);
    // Coordinate information
    static constexpr uint16_t  GT911_PRODUCT_ID              = (0x8140);
    static constexpr uint16_t  GT911_FIRMWARE_VERSION        = (0x8144);
    static constexpr uint16_t  GT911_X_RESOLUTION            = (0x8146);
    static constexpr uint16_t  GT911_Y_RESOLUTION            = (0x8148);

    static constexpr uint16_t  GT911_VENDOR_ID               = (0X8140);
    static constexpr uint16_t  GT911_INFORMATION             = (0X8140);
    static constexpr uint16_t  GT911_POINT_INFO              = (0X814E);
    static constexpr uint16_t  GT911_POINT_1                 = (0X814F);
    static constexpr uint16_t  GT911_POINT_2                 = (0X8157);
    static constexpr uint16_t  GT911_POINT_3                 = (0X815F);
    static constexpr uint16_t  GT911_POINT_4                 = (0X8167);
    static constexpr uint16_t  GT911_POINT_5                 = (0X816F);


    static constexpr uint16_t  GT911_DEV_ID                   =  (911);
    static constexpr uint8_t   GT911_BASE_REF_RATE             =  (5);
    static constexpr uint8_t   GT911_REG_LENGTH                =  (186);

};




