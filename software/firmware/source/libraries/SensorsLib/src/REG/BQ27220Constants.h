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
 * @file      BQ27220Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-16
 *
 */

#pragma once

#include <stdint.h>


class BQ27220Constants
{
protected:

    //* Sub command
    static constexpr uint16_t BQ27220_SUB_CMD_CTRL_STATUS                = (0x0000);
    static constexpr uint16_t BQ27220_SUB_CMD_DEVICE_NUMBER              = (0x0001);
    static constexpr uint16_t BQ27220_SUB_CMD_FW_VERSION                 = (0x0002);
    static constexpr uint16_t BQ27220_SUB_CMD_HW_VERSION                 = (0x0003);
    static constexpr uint16_t BQ27220_SUB_CMD_BOARD_OFFSET               = (0x0009);
    static constexpr uint16_t BQ27220_SUB_CMD_CC_OFFSET                  = (0x000A);
    static constexpr uint16_t BQ27220_SUB_CMD_CC_OFFSET_SAVE             = (0x000B);
    static constexpr uint16_t BQ27220_SUB_CMD_OCV_CMD                    = (0x000C);
    static constexpr uint16_t BQ27220_SUB_CMD_BAT_INSERT                 = (0x000D);
    static constexpr uint16_t BQ27220_SUB_CMD_BAT_REMOVE                 = (0x000E);
    static constexpr uint16_t BQ27220_SUB_CMD_SET_SNOOZE                 = (0x0013);
    static constexpr uint16_t BQ27220_SUB_CMD_CLEAR_SNOOZE               = (0x0014);
    static constexpr uint16_t BQ27220_SUB_CMD_SET_PROFILE_1              = (0x0015);
    static constexpr uint16_t BQ27220_SUB_CMD_SET_PROFILE_2              = (0x0016);
    static constexpr uint16_t BQ27220_SUB_CMD_SET_PROFILE_3              = (0x0017);
    static constexpr uint16_t BQ27220_SUB_CMD_SET_PROFILE_4              = (0x0018);
    static constexpr uint16_t BQ27220_SUB_CMD_SET_PROFILE_5              = (0x0019);
    static constexpr uint16_t BQ27220_SUB_CMD_SET_PROFILE_6              = (0x001A);
    static constexpr uint16_t BQ27220_SUB_CMD_CAL_TOGGLE                 = (0x002D);
    static constexpr uint16_t BQ27220_SUB_CMD_SEALED                     = (0x0030);
    static constexpr uint16_t BQ27220_SUB_CMD_RESET                      = (0x0041);
    static constexpr uint16_t BQ27220_SUB_CMD_EXIT_CAL                   = (0x0080);
    static constexpr uint16_t BQ27220_SUB_CMD_ENTER_CAL                  = (0x0081);
    static constexpr uint16_t BQ27220_SUB_CMD_ENTER_CFG_UPDATE           = (0x0090);
    static constexpr uint16_t BQ27220_SUB_CMD_EXIT_CFG_UPDATE_REINIT     = (0x0091);
    static constexpr uint16_t BQ27220_SUB_CMD_EXIT_CFG_UPDATE            = (0x0092);
    static constexpr uint16_t BQ27220_SUB_CMD_RETURN_TO_ROM              = (0x0F00);

    //* Standard commands
    static constexpr uint8_t BQ27220_REG_STA_AT_RATE                     = (0x02);
    static constexpr uint8_t BQ27220_REG_STA_AT_RATE_TIME_TO_EMPTY       = (0x04);
    static constexpr uint8_t BQ27220_REG_STA_NTC_TEMP                    = (0x06);
    static constexpr uint8_t BQ27220_REG_STA_BAT_VOLTAGE                 = (0x08);
    static constexpr uint8_t BQ27220_REG_STA_BAT_STATUS                  = (0x0A);
    static constexpr uint8_t BQ27220_REG_STA_CURRENT                     = (0x0C);
    static constexpr uint8_t BQ27220_REG_STA_REMAINING_CAPACITY          = (0x10);
    static constexpr uint8_t BQ27220_REG_STA_FULL_CHARGE_CAPACITY        = (0x12);
    static constexpr uint8_t BQ27220_REG_STA_TIME_TO_EMPTY               = (0x16);
    static constexpr uint8_t BQ27220_REG_STA_TIME_TO_FULL                = (0x18);
    static constexpr uint8_t BQ27220_REG_STA_STANDBY_CURRENT             = (0x1A);
    static constexpr uint8_t BQ27220_REG_STA_STANDBY_TIME_TO_EMPTY       = (0x1C);
    static constexpr uint8_t BQ27220_REG_STA_MAX_LOAD_CURRENT            = (0x1E);
    static constexpr uint8_t BQ27220_REG_STA_MAX_LOAD_TO_EMPTY           = (0x20);
    static constexpr uint8_t BQ27220_REG_STA_COULOMB_COUNT               = (0x22);
    static constexpr uint8_t BQ27220_REG_STA_AVG_POWER                   = (0x24);
    static constexpr uint8_t BQ27220_REG_STA_INTER_TEMP                  = (0x28);
    static constexpr uint8_t BQ27220_REG_STA_CYCLE_COUNT                 = (0x2A);
    static constexpr uint8_t BQ27220_REG_STA_STATE_OF_CHARGE             = (0x2C);
    static constexpr uint8_t BQ27220_REG_STA_STATE_OF_HEALTH             = (0x2E);
    static constexpr uint8_t BQ27220_REG_STA_CHARGING_VOLTAGE            = (0x30);
    static constexpr uint8_t BQ27220_REG_STA_CHARGING_CURRENT            = (0x32);
    static constexpr uint8_t BQ27220_REG_STA_BTP_DISC_SET                = (0x34);
    static constexpr uint8_t BQ27220_REG_STA_BTP_CHARGE_SET              = (0x36);
    static constexpr uint8_t BQ27220_REG_STA_OPERATION_STATUS            = (0x3A);
    static constexpr uint8_t BQ27220_REG_STA_DESIGN_CAPACITY             = (0x3C);
    static constexpr uint8_t BQ27220_REG_STA_DESIGN_CAPACITY_MSB         = (0x3E);
    static constexpr uint8_t BQ27220_REG_STA_DESIGN_CAPACITY_LSB         = (0x3F);
    static constexpr uint8_t BQ27220_REG_MAC_BUFFER_START                = (0x40);
    static constexpr uint8_t BQ27220_REG_MAC_BUFFER_END                  = (0x5F);
    static constexpr uint8_t BQ27220_REG_MAC_DATA_SUM                    = (0x60);
    static constexpr uint8_t BQ27220_REG_MAC_DATA_LEN                    = (0x61);
    static constexpr uint8_t BQ27220_REG_ANALOG_COUNT                    = (0x79);
    static constexpr uint8_t BQ27220_REG_RAW_CURRENT                     = (0x7A);
    static constexpr uint8_t BQ27220_REG_RAW_VOLTAGE                     = (0x7C);
    static constexpr uint8_t BQ27220_REG_ROM_START                       = (0x3E);

    static constexpr uint16_t  BQ27220_CHIP_ID                           = (0x0220);

    static constexpr uint16_t BQ27220_ROM_FULL_CHARGE_CAPACITY           = (0x929D);
    static constexpr uint16_t BQ27220_ROM_DESIGN_CAPACITY                = (0x929F);
    static constexpr uint16_t BQ27220_ROM_OPERATION_CONFIG_A             = (0x9206);
    static constexpr uint16_t BQ27220_ROM_OPERATION_CONFIG_B             = (0x9208);
};
