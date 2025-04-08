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
 * @file      BMA423Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-17
 *
 */
#pragma once

#include <stdint.h>

#define	BMA423_I2C_ADDR_PRIMARY	            (0x18)
#define	BMA423_I2C_ADDR_SECONDARY	        (0x19)

class BMA423Constants
{
protected:
    static constexpr uint8_t BAM423_SENSOR_RESOLUTION   = (12)  ;     //*
    static constexpr uint8_t RESET_REG                  = (0x7E);     //*
    static constexpr uint8_t CHIP_ID_ADDR               = (0x00);     /**\name CHIP ID ADDRESS*/
    static constexpr uint8_t CHIP_ID                    = (0x13);     /**\name Chip ID of BMA423 sensor */
    static constexpr uint8_t POWER_CONF_ADDR            = (0x7C);     /**\name POWER_CTRL REGISTER*/
    static constexpr uint8_t POWER_CTRL_ADDR            = (0x7D);     /**\name POWER_CTRL REGISTER*/
    static constexpr uint8_t ADVANCE_POWER_SAVE_MSK     = (0x01);     /**\name ADVANCE POWER SAVE POSITION AND MASK*/
    static constexpr uint8_t INT_MAP_1_ADDR             = (0X56);     /**\name MAP INTERRUPT 1 and 2 REGISTERS*/
    static constexpr uint8_t INT_MAP_2_ADDR             = (0X57);     /**\name MAP INTERRUPT 1 and 2 REGISTERS*/
    static constexpr uint8_t INT_MAP_DATA_ADDR          = (0x58);     /**\name MAP INTERRUPT 1 and 2 REGISTERS*/
    static constexpr uint8_t INIT_CTRL_ADDR             = (0x59);     /**\name MAP INTERRUPT 1 and 2 REGISTERS*/
    static constexpr uint8_t RESERVED_REG_5B_ADDR       = (0x5B);     /**\name FEATURE CONFIG RELATED */
    static constexpr uint8_t RESERVED_REG_5C_ADDR       = (0x5C);     /**\name FEATURE CONFIG RELATED */
    static constexpr uint8_t FEATURE_CONFIG_ADDR        = (0x5E);     /**\name FEATURE CONFIG RELATED */
    static constexpr uint8_t INTERNAL_ERROR             = (0x5F);     /**\name FEATURE CONFIG RELATED */
    static constexpr uint8_t STEP_CNT_OUT_0_ADDR        = (0x1E);     /**\name GPIO REGISTERS*/
    static constexpr uint8_t HIGH_G_OUT_ADDR            = (0x1F);     /**\name GPIO REGISTERS*/
    static constexpr uint8_t ACTIVITY_OUT_ADDR          = (0x27);     /**\name GPIO REGISTERS*/
    static constexpr uint8_t ORIENTATION_OUT_ADDR       = (0x28);     /**\name GPIO REGISTERS*/
    static constexpr uint8_t INTERNAL_STAT              = (0x2A);     /**\name GPIO REGISTERS*/
    static constexpr uint8_t SENSORTIME_0_ADDR          = (0X18);     /**\name SENSOR TIME REGISTERS*/
    static constexpr uint8_t INT_STAT_0_ADDR            = (0X1C);     /**\name INTERRUPT/FEATURE STATUS REGISTERS*/
    static constexpr uint8_t INT_STAT_1_ADDR            = (0X1D);     /**\name INTERRUPT/FEATURE STATUS REGISTERS*/
    static constexpr uint8_t DATA_0_ADDR                = (0X0A);     /**\name AUX/ACCEL DATA BASE ADDRESS REGISTERS*/
    static constexpr uint8_t DATA_8_ADDR                = (0X12);     /**\name AUX/ACCEL DATA BASE ADDRESS REGISTERS*/
    static constexpr uint8_t ACCEL_CONFIG_ADDR          = (0X40);     /**\name AUX/ACCEL DATA BASE ADDRESS REGISTERS*/
    static constexpr uint8_t TEMPERATURE_ADDR           = (0X22);     /**\name AUX/ACCEL DATA BASE ADDRESS REGISTERS*/
    static constexpr uint8_t INT1_IO_CTRL_ADDR          = (0X53);     /**\name INTERRUPT ENABLE REGISTERS*/
    static constexpr uint8_t INT2_IO_CTRL_ADDR          = (0X54);     /**\name INTERRUPT ENABLE REGISTERS*/
    static constexpr uint8_t INTR_LATCH_ADDR            = (0X55);     /**\name LATCH DURATION REGISTERS*/


    /**\name BUS READ AND WRITE LENGTH FOR MAG & ACCEL*/
    static constexpr uint8_t MAG_TRIM_DATA_SIZE         = (16);
    static constexpr uint8_t MAG_XYZ_DATA_LENGTH        = (6);
    static constexpr uint8_t MAG_XYZR_DATA_LENGTH       = (8);
    static constexpr uint8_t ACCEL_DATA_LENGTH          = (6);
    static constexpr uint8_t FIFO_DATA_LENGTH           = (2);
    static constexpr uint8_t TEMP_DATA_SIZE             = (1);



    /**\name Feature offset address */
    static constexpr uint8_t ANY_NO_MOTION_OFFSET     = (0x00);
    static constexpr uint8_t STEP_CNTR_OFFSET         = (0x36);
    static constexpr uint8_t STEP_CNTR_PARAM_OFFSET   = (0x04);
    static constexpr uint8_t WAKEUP_OFFSET            = (0x38);
    static constexpr uint8_t TILT_OFFSET              = (0x3A);
    static constexpr uint8_t CONFIG_ID_OFFSET         = (0x3C);
    static constexpr uint8_t AXES_REMAP_OFFSET        = (0x3E);

    /**\name Sensor feature size */
    static constexpr uint8_t FEATURE_SIZE             = (64);
    static constexpr uint8_t ANYMOTION_EN_LEN         = (2);
    static constexpr uint8_t RD_WR_MIN_LEN            = (2);

    /**************************************************************/
    /**\name    Remap Axes */
    /**************************************************************/
    static constexpr uint8_t X_AXIS_MASK              = (0x03);
    static constexpr uint8_t X_AXIS_SIGN_MASK         = (0x04);
    static constexpr uint8_t Y_AXIS_MASK              = (0x18);
    static constexpr uint8_t Y_AXIS_SIGN_MASK         = (0x20);
    static constexpr uint8_t Z_AXIS_MASK              = (0xC0);
    static constexpr uint8_t Z_AXIS_SIGN_MASK         = (0x01);



    /**\name ACCELEROMETER ENABLE POSITION AND MASK*/
    static constexpr uint8_t ACCEL_ENABLE_MSK           = (0x04);
    static constexpr uint8_t ASIC_INITIALIZED           = (0x01);

    /**************************************************************/
    /**\name    Step Counter & Detector */
    /**************************************************************/
    /**\name Step counter enable macros */
    static constexpr uint8_t STEP_CNTR_EN_POS         = (4);
    static constexpr uint8_t STEP_CNTR_EN_MSK         = (0x10);
    static constexpr uint8_t ACTIVITY_EN_MSK          = (0x20);

    /**\name Step counter watermark macros */
    static constexpr uint16_t STEP_CNTR_WM_MSK         = (0x03FF);

    /**\name Step counter reset macros */
    static constexpr uint8_t STEP_CNTR_RST_POS        = (2);
    static constexpr uint8_t STEP_CNTR_RST_MSK        = (0x04);

    /**\name Step detector enable macros */
    static constexpr uint8_t STEP_DETECTOR_EN_POS     = (3);
    static constexpr uint8_t STEP_DETECTOR_EN_MSK     = (0x08);

    /**\name Tilt enable macros */
    static constexpr uint8_t TILT_EN_MSK              = (0x01);

    /**\name Step count output length*/
    static constexpr uint8_t STEP_CNTR_DATA_SIZE      = (4);

    /**\name Wakeup enable macros */
    static constexpr uint8_t WAKEUP_EN_MSK            = (0x01);

    /**\name Wake up sensitivity macros */
    static constexpr uint8_t WAKEUP_SENS_POS          = (1);
    static constexpr uint8_t WAKEUP_SENS_MSK          = (0x0E);

    /**\name Tap selection macro */
    static constexpr uint8_t TAP_SEL_POS              = (4);
    static constexpr uint8_t TAP_SEL_MSK              = (0x10);

    /**************************************************************/
    /**\name    Any Motion */
    /**************************************************************/
    /**\name Any motion threshold macros */
    static constexpr uint8_t ANY_NO_MOTION_THRES_POS      = (0);
    static constexpr uint16_t ANY_NO_MOTION_THRES_MSK      = (0x07FF);

    /**\name Any motion selection macros */
    static constexpr uint8_t ANY_NO_MOTION_SEL_POS        = (3);
    static constexpr uint8_t ANY_NO_MOTION_SEL_MSK        = (0x08);

    /**\name Any motion enable macros */
    static constexpr uint8_t ANY_NO_MOTION_AXIS_EN_POS    = (5);
    static constexpr uint8_t ANY_NO_MOTION_AXIS_EN_MSK    = (0xE0);

    /**\name Any motion duration macros */
    static constexpr uint16_t ANY_NO_MOTION_DUR_MSK        = (0x1FFF);

    /**\name INTERRUPT MAPS    */
    static constexpr uint8_t INTR1_MAP       = (0);
    static constexpr uint8_t INTR2_MAP       = (1);



    /**\name    CONSTANTS */
    static constexpr uint8_t FIFO_CONFIG_LENGTH             = (2);
    static constexpr uint8_t ACCEL_CONFIG_LENGTH            = (2);
    static constexpr uint8_t FIFO_WM_LENGTH                 = (2);
    static constexpr uint16_t CONFIG_STREAM_SIZE             = (6144);
    static constexpr uint8_t NON_LATCH_MODE                 = (0);
    static constexpr uint8_t LATCH_MODE                     = (1);
    static constexpr uint8_t PUSH_PULL                      = (0);
    static constexpr uint8_t ACTIVE_HIGH                    = (1);
    static constexpr uint8_t ACTIVE_LOW                     = (0);
    static constexpr uint8_t EDGE_TRIGGER                   = (1);
    static constexpr uint8_t LEVEL_TRIGGER                  = (0);
    static constexpr uint8_t OUTPUT_ENABLE                  = (1);
    static constexpr uint8_t OUTPUT_DISABLE                 = (0);
    static constexpr uint8_t INPUT_ENABLE                   = (1);
    static constexpr uint8_t INPUT_DISABLE                  = (0);


    /**\name    OUTPUT TYPE ENABLE POSITION AND MASK*/
    static constexpr uint8_t INT_EDGE_CTRL_MASK            = (0x01);
    static constexpr uint8_t INT_EDGE_CTRL_POS             = (0x00);
    static constexpr uint8_t INT_LEVEL_MASK                = (0x02);
    static constexpr uint8_t INT_LEVEL_POS                 = (0x01);
    static constexpr uint8_t INT_OPEN_DRAIN_MASK           = (0x04);
    static constexpr uint8_t INT_OPEN_DRAIN_POS            = (0x02);
    static constexpr uint8_t INT_OUTPUT_EN_MASK            = (0x08);
    static constexpr uint8_t INT_OUTPUT_EN_POS             = (0x03);
    static constexpr uint8_t INT_INPUT_EN_MASK             = (0x10);
    static constexpr uint8_t INT_INPUT_EN_POS              = (0x04);


    /**\name Interrupt status macros */
    static constexpr uint8_t STEP_CNTR_INT                = (0x02);
    static constexpr uint8_t ACTIVITY_INT                 = (0x04);
    static constexpr uint8_t TILT_INT                     = (0x08);
    static constexpr uint8_t WAKEUP_INT                   = (0x20);
    static constexpr uint8_t ANY_NO_MOTION_INT            = (0x40);
    static constexpr uint8_t ERROR_INT                    = (0x80);

};


