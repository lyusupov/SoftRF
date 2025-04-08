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
 * @file      SensorBMA423.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-03-31
 * @note      Most source code references come from the https://github.com/boschsensortec/BMA423-Sensor-API
 *            Simplification for Arduino
 */
#pragma once

#include "REG/BMA423Constants.h"
#include "SensorPlatform.hpp"

class SensorBMA423 : public BMA423Constants
{
public:
    enum AccelRange {
        RANGE_2G,
        RANGE_4G,
        RANGE_8G,
        RANGE_16G
    };

    /*! Output data rate in Hz */
    enum AccelODR {
        ODR_0_78HZ = 1,
        ODR_1_56HZ,
        ODR_3_12HZ,
        ODR_6_25HZ,
        ODR_12_5HZ,
        ODR_25HZ,
        ODR_50HZ,
        ODR_100HZ,
        ODR_200HZ,
        ODR_400HZ,
        ODR_800HZ,
        ODR_1600HZ,
    };

    /*! Bandwidth parameter, determines filter configuration */
    enum BandWidth {
        BW_OSR4_AVG1,
        BW_OSR2_AVG2,
        BW_NORMAL_AVG4,
        BW_CIC_AVG8,
        BW_RES_AVG16,
        BW_RES_AVG32,
        BW_RES_AVG64,
        BW_RES_AVG128,
    };

    enum PerformanceMode {
        PERF_CIC_AVG_MODE,
        PERF_CONTINUOUS_MODE,
    };

    enum TemperatureUnit {
        TEMP_DEG,
        TEMP_FAHRENHEIT,
        TEMP_KELVIN,
    };

    // Calculate direction facing the front of the chip
    enum SensorDir {
        DIRECTION_BOTTOM_LEFT,
        DIRECTION_TOP_RIGHT,
        DIRECTION_TOP_LEFT,
        DIRECTION_BOTTOM_RIGHT,
        DIRECTION_BOTTOM,
        DIRECTION_TOP
    };

    // Chip orientation and orientation
    enum SensorRemap {
        // Top right corner
        REMAP_TOP_LAYER_RIGHT_CORNER,
        // Front bottom left corner
        REMAP_TOP_LAYER_BOTTOM_LEFT_CORNER,
        // Top left corner
        REMAP_TOP_LAYER_LEFT_CORNER,
        // Top bottom right corner
        REMAP_TOP_LAYER_BOTTOM_RIGHT_CORNER,
        // Bottom top right corner
        REMAP_BOTTOM_LAYER_TOP_RIGHT_CORNER,
        // Bottom bottom left corner
        REMAP_BOTTOM_LAYER_BOTTOM_LEFT_CORNER,
        // Bottom bottom right corner
        REMAP_BOTTOM_LAYER_BOTTOM_RIGHT_CORNER,
        // Bottom top left corner
        REMAP_BOTTOM_LAYER_TOP_LEFT_CORNER,
    };

    enum Feature {
        /**\name Feature enable macros for the sensor */
        FEATURE_STEP_CNTR   = 0x01,
        /**\name Below macros are mutually exclusive */
        FEATURE_ANY_MOTION  = 0x02,
        FEATURE_NO_MOTION   = 0x04,
        FEATURE_ACTIVITY    = 0x08,
        FEATURE_TILT        = 0x10,
        FEATURE_WAKEUP      = 0x20,
    };

    /**\name Interrupt status macros */
    enum FeatureInterrupt {
        INT_STEP_CNTR     = 0x02,
        INT_ACTIVITY      = 0x04,
        INT_TILT          = 0x05,
        INT_WAKEUP        = 0x20,
        INT_ANY_NO_MOTION = 0x40,
    };

    SensorBMA423();

    ~SensorBMA423();

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = BMA423_I2C_ADDR_SECONDARY, int sda = -1, int scl = -1);

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = BMA423_I2C_ADDR_SECONDARY, int sda = -1, int scl = -1);
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = BMA423_I2C_ADDR_SECONDARY);
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = BMA423_I2C_ADDR_SECONDARY);

    void reset();

    bool enablePowerSave();

    bool disablePowerSave();

    void disableInterruptCtrl();

    void enableInterruptCtrl();

    bool enableAccelerometer();

    bool disableAccelerometer();

    bool configAccelerometer(AccelRange range = RANGE_4G, AccelODR odr = ODR_200HZ,
                             BandWidth bw = BW_NORMAL_AVG4,
                             PerformanceMode perfMode = PERF_CONTINUOUS_MODE);

    bool getAccelRaw(int16_t *rawBuffer);

    bool getAccelerometer(int16_t &x, int16_t &y, int16_t &z);

    float getTemperature(TemperatureUnit unit);

    uint8_t direction();

    bool setRemapAxes(SensorRemap remap);

    bool setStepCounterWatermark(uint16_t watermark);

    bool disablePedometer();

    bool enablePedometer(bool enable = true);

    uint32_t getPedometerCounter();

    void resetPedometer();

    bool enableFeature(uint8_t feature, uint8_t enable);

    uint16_t readIrqStatus();

    uint16_t getIrqStatus();

    bool configInterrupt(
        /*! Trigger condition of interrupt pin */
        uint8_t edge_ctrl = LEVEL_TRIGGER,
        /*! Level of interrupt pin */
        uint8_t level = ACTIVE_HIGH,
        /*! Behaviour of interrupt pin to open drain */
        uint8_t od = PUSH_PULL,
        /*! Output enable for interrupt pin */
        uint8_t output_en = OUTPUT_ENABLE,
        /*! Input enable for interrupt pin */
        uint8_t input_en = INPUT_DISABLE,
        /*! Variable used to select the interrupt pin1 or pin2 for interrupt configuration. */
        uint8_t int_line = INTR1_MAP
    );

    bool configFeatureInterrupt(uint16_t  feature_interrupt_mask, bool enable);

    bool enablePedometerIRQ();

    bool enableTiltIRQ();

    bool enableWakeupIRQ();

    bool enableAnyNoMotionIRQ();

    bool enableActivityIRQ();

    bool disablePedometerIRQ();

    bool disableTiltIRQ();

    bool disableWakeupIRQ();

    bool disableAnyNoMotionIRQ();

    bool disableActivityIRQ();

    bool isActivity();

    bool isTilt();

    bool isDoubleTap();

    bool isAnyNoMotion();

    bool isPedometer();

private:

    bool interruptMap(uint8_t int_line, uint16_t int_map, uint8_t enable);

    /*!
     *  @brief This API sets the interrupt mode in the sensor.
     */
    bool setInterruptMode(uint8_t mode);

    bool feature_disable(uint8_t feature, uint8_t len, uint8_t *buffer);

    int feature_enable(uint8_t feature, uint8_t len, uint8_t *buffer);

    void update_len(uint8_t *len, uint8_t feature, uint8_t enable);

    bool configure();

    bool initImpl();

    uint16_t    int_status;
    uint8_t     int_line;
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
};
