/**
 *
 * @license MIT License
 *
 * Copyright (c) 2026 lewis he
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
 * @file      SensorQMC6309.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-20
 *
 */
#pragma once

#include "platform/comm/I2CDeviceWithHal.hpp"
#include "sensor/MagnetometerBase.hpp"

static constexpr uint8_t QMC6309_SLAVE_ADDRESS = (0x7C);
static constexpr uint8_t QMC6309H_SLAVE_ADDRESS = (0x0C);

class SensorQMC6309 : public MagnetometerBase, public I2CDeviceWithHal
{
public:

    enum class MagSetResetMode {
        SET_AND_RESET_ON,
        SET_ONLY_ON,
        SET_AND_RESET_OFF
    };

    enum class VoltageMode {
        KEEP,
        VDD_MODE1,
        VDD_MODE2
    };

    /**
     * @brief Default constructor. Initializes communication and hardware abstraction pointers to nullptr.
     *
     * Creates a SensorQMC6309 object with no active communication or hardware
     * abstraction instances.
     */
    SensorQMC6309() = default;

    /**
     * @brief Destructor. Deinitializes the communication object if it exists.
     *
     * Ensures proper cleanup by deinitializing the communication interface
     * when the sensor object is destroyed.
     */
    ~SensorQMC6309() = default;

    /**
     * @brief  Reads the magnetic field data from the sensor.
     * @note   This function retrieves the latest magnetic field measurements
     *         from the sensor and stores them in the provided data structure.
     * @param  data: A reference to a MagnetometerData structure to store the
     *               retrieved magnetic field data.
     * @retval True if the data was read successfully, false otherwise.
     */
    bool readData(MagnetometerData &data) override
    {
        uint8_t buffer[6] = {0};

        int status = readReg(REG_0x09_STAT);
        if (status < 0) {
            log_e("Failed to read status register");
            return false;
        }

        // OVL (Overflow)
        if (isBitSet(status, 1)) {
            data.overflow = true;
            log_w("Data overflow detected");
        } else {
            data.overflow = false;
        }

        // DRDY (Data Ready)
        if (!isBitSet(status, 0)) {
            // log_e("Data not ready");
            return false;
        }

        if (readRegBuff(REG_0x01_LSB_DX, buffer, sizeof(buffer)) < 0) {
            log_e("Failed to read magnetic field data");
            return false;
        }
        data.raw.x = (int16_t)(buffer[1] << 8) | (buffer[0]);  // Combine X LSB and MSB
        data.raw.y = (int16_t)(buffer[3] << 8) | (buffer[2]);  // Combine Y LSB and MSB
        data.raw.z = (int16_t)(buffer[5] << 8) | (buffer[4]);  // Combine Z LSB and MSB

        data.raw.x -= _x_offset;
        data.raw.y -= _y_offset;
        data.raw.z -= _z_offset;

        // Convert raw values to Gauss using sensitivity (depends on selected magnetic range)
        data.magnetic_field.x = (float)(data.raw.x) * _sensitivity;
        data.magnetic_field.y = (float)(data.raw.y) * _sensitivity;
        data.magnetic_field.z = (float)(data.raw.z) * _sensitivity;

        // Calculate heading
        data.heading = MagnetometerUtils::calculateHeading(data, _declination_rad);

        // Convert heading to degrees
        data.heading_degrees = data.heading * (180.0 / M_PI);

        return true;
    }

    /**
     * @brief  Checks if new data is available from the sensor.
     * @note   This function reads the status register to determine if new
     *         magnetic field data is ready to be processed.
     * @retval True if new data is available, false otherwise.
     */
    bool isDataReady() override
    {
        return getRegBit(REG_0x09_STAT, 0);
    }

    /**
    * @brief  Checks if data overflow has occurred.
    * @note   This function reads the status register to determine if
    *         the magnetic field data has overflowed.
    * @retval True if data overflow has occurred, false otherwise.
    */
    bool isDataOverflow()
    {
        return getRegBit(REG_0x09_STAT, 1);
    }

    /**
     * @brief  Checks if the NVM is ready for access.
     * @note   This function reads the status register to determine if
     *         the NVM is ready.
     * @retval True if the NVM is ready, false otherwise.
     */
    bool isNVMReady()
    {
        return getRegBit(REG_0x09_STAT, 3);
    }

    /**
     * @brief  Checks if the NVM load is complete.
     * @note   This function reads the status register to determine if
     *         the NVM load is complete.
     * @retval True if the NVM load is complete, false otherwise.
     */
    bool isNVMLoadDone()
    {
        return getRegBit(REG_0x09_STAT, 4);
    }

    /**
    * @brief  Resets the sensor to its default state.
    * @note   This function sends a reset command to the sensor and waits
    *         for a short period to allow the reset to take effect.
    * @retval True if the reset command was successful, false otherwise.
    */
    bool reset() override
    {
        if (writeReg(REG_0x0B_CMD2, 0x80) < 0) {
            log_e("Failed to set soft reset");
            return false;
        }
        if (writeReg(REG_0x0B_CMD2, 0x00) < 0) {
            log_e("Failed to clear soft reset");
            return false;
        }

        hal->delay(5);

        uint8_t retry = 0;
        while (retry++ < 5) {
            int status = readReg(REG_0x09_STAT);
            if (status >= 0 && isBitSet(status, 4) && isBitSet(status, 3)) {
                return true;
            }
            hal->delay(1);
        }

        log_e("NVM not ready after soft reset");
        return false;
    }

    /**
     * @brief  Checks if the sensor is functioning correctly.
     * @note   This function performs a self-test by comparing the
     *         magnetic field readings before and after enabling self-test mode.
     * @retval True if the self-test passed, false otherwise.
     */
    bool selfTest() override
    {
        int16_t x_result = 0, y_result = 0, z_result = 0;
        return selfTest(x_result, y_result, z_result);
    }

    /**
    * @brief  Performs a self-test on the sensor.
    * @param  &x_result: Self-test X-axis value from register 0x13
    * @param  &y_result: Self-test Y-axis value from register 0x14
    * @param  &z_result: Self-test Z-axis value from register 0x15
    * @retval True if the self-test passed, false otherwise.
    */
    bool selfTest(int16_t &x_result, int16_t &y_result, int16_t &z_result)
    {
        if (!setOperationMode(OperationMode::SUSPEND)) {
            log_e("Failed to set SUSPEND mode for selfTest");
            return false;
        }

        if (!setOperationMode(OperationMode::CONTINUOUS_MEASUREMENT)) {
            log_e("Failed to set CONTINUOUS_MEASUREMENT for selfTest");
            return false;
        }

        hal->delay(20);

        if (writeReg(REG_0x0E_SELFTEST_CTRL, MASK_SOFT_RST) < 0) {
            log_e("Failed to trigger self-test");
            return false;
        }

        uint8_t retry = 0;
        while (!getRegBit(REG_0x09_STAT, 2) && retry++ < 50) {
            hal->delay(5);
        }
        if (retry > 50) {
            log_e("Self-test not ready (ST_RDY bit is 0)");
            return false;
        }

        int x_test = readReg(REG_0x13_SELFTEST_X);
        int y_test = readReg(REG_0x14_SELFTEST_Y);
        int z_test = readReg(REG_0x15_SELFTEST_Z);
        if (x_test < 0 || y_test < 0 || z_test < 0) {
            log_e("Failed to read self-test result");
            return false;
        }

        x_result = (int8_t)x_test;
        y_result = (int8_t)y_test;
        z_result = (int8_t)z_test;

        setOperationMode(OperationMode::SUSPEND);

        int16_t x_abs = x_result < 0 ? -x_result : x_result;
        int16_t y_abs = y_result < 0 ? -y_result : y_result;
        int16_t z_abs = z_result < 0 ? -z_result : z_result;

        bool x_ok = (x_abs >= 1 && x_abs <= 50);
        bool y_ok = (y_abs >= 1 && y_abs <= 50);
        bool z_ok = (z_abs >= 1 && z_abs <= 50);

        if (!x_ok || !y_ok || !z_ok) {
            log_w("Self-test data out of range: X=%d, Y=%d, Z=%d (expected abs in 1..50)",
                  x_result, y_result, z_result);
        }

        return x_ok && y_ok && z_ok;
    }

    /**
    * @brief  Sets the full-scale range of the magnetometer.
    * @note   This function configures the magnetometer to operate within a
    *         specific range, affecting the sensitivity and maximum measurable
    *         magnetic field strength.
    * @param  range: FS_8G, FS_16G, FS_32G
    * @retval True if the range was set successfully, false otherwise.
    */
    bool setFullScaleRange(MagFullScaleRange range) override
    {
        float full_scale = 0.0f;
        float sensitivity = 0.0f;
        uint8_t range_value = 0;
        switch (range) {
        case MagFullScaleRange::FS_32G:
            sensitivity = 0.001f;      // 1000 LSB/Gauss
            range_value = 0x00 << 2;
            full_scale = 32.0f;
            break;
        case MagFullScaleRange::FS_16G:
            sensitivity = 0.0005f;     // 2000 LSB/Gauss
            range_value = 0x01 << 2;
            full_scale = 16.0f;
            break;
        case MagFullScaleRange::FS_8G:
            sensitivity = 0.00025f;    // 4000 LSB/Gauss
            range_value = 0x02 << 2;
            full_scale = 8.0f;
            break;
        default:
            log_e("Invalid magnetometer range.");
            return false;
        }

        if (updateBits(REG_0x0B_CMD2, 0x0C, range_value) < 0) {
            log_e("Failed to set full scale range.");
            return false;
        }
        _sensitivity = sensitivity;
        _config.range = full_scale;
        return true;
    }

    /**
     * @brief  Sets the output data rate for the magnetometer.
     * @note   This function should be called to configure the sensor's output data rate.
     * @param  data_rate_hz: The desired output data rate in Hz.  Allowed values are 1.0, 10.0, 50.0, 100.0 and 200.0HZ.
     * @retval True if the output data rate was set successfully, false otherwise.
     */
    bool setOutputDataRate(float data_rate_hz) override
    {
        int rangeInt = static_cast<int>(data_rate_hz * 100 + 0.5);
        uint8_t regValue = 0;
        switch (rangeInt) {
        case 100:           // 1.0
            regValue = 0x00 << 4;
            break;
        case 1000:         // 10.0
            regValue = 0x01 << 4;
            break;
        case 5000:         // 50.0
            regValue = 0x02 << 4;
            break;
        case 10000:         // 100.0
            regValue = 0x03 << 4;
            break;
        case 20000:         // 200.0
            regValue = 0x04 << 4;
            break;
        default:
            log_e("Invalid output data rate");
            return false;
        }
        if (updateBits(REG_0x0A_CMD1, 0x70, regValue) < 0) {
            log_e("Failed to set bandwidth");
            return false;
        }
        _config.sample_rate = data_rate_hz;
        return true;
    }

    /**
    * @brief  Sets the operation mode of the magnetometer.
    * @note   This function configures the magnetometer to operate in a specific
    *         mode, affecting its power consumption and measurement behavior.
    * @param  mode: SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
    * @retval True if the mode was set successfully, false otherwise.
    */
    bool setOperationMode(OperationMode mode) override
    {
        uint8_t mode_val = 0x00;
        switch (mode) {
        case OperationMode::SUSPEND:
            mode_val = 0x00;
            break;
        case OperationMode::NORMAL:
            mode_val = 0x01;
            break;
        case OperationMode::SINGLE_MEASUREMENT:
            mode_val = 0x02;
            break;
        case OperationMode::CONTINUOUS_MEASUREMENT:
            mode_val = 0x03;
            break;
        default:
            log_e("Invalid operation mode");
            return false;
        }
        if (updateBits(REG_0x0A_CMD1, 0x03, mode_val) < 0) {
            log_e("Failed to set operation mode");
            return false;
        }
        _config.mode = mode;
        return true;
    }

    /**
    * @brief  Sets the oversampling rate of the magnetometer.
    * @note   This function configures the magnetometer's oversampling rate,
    *         affecting the trade-off between noise and measurement speed.
    * @param  osr: OSR_8, OSR_4, OSR_2, OSR_1
    * @retval True if the oversampling rate was set successfully, false otherwise.
    */
    bool setOversamplingRate(MagOverSampleRatio osr) override
    {
        uint8_t osr_val = 0x00;
        switch (osr) {
        case MagOverSampleRatio::OSR_8:
            osr_val = 0x00 << 3;
            _oversampling_rate = 8;
            break;
        case MagOverSampleRatio::OSR_4:
            osr_val = 0x01 << 3;
            _oversampling_rate = 4;
            break;
        case MagOverSampleRatio::OSR_2:
            osr_val = 0x02 << 3;
            _oversampling_rate = 2;
            break;
        case MagOverSampleRatio::OSR_1:
            osr_val = 0x03 << 3;
            _oversampling_rate = 1;
            break;
        default:
            log_e("Invalid oversampling rate");
            return false;
        }
        return updateBits(REG_0x0A_CMD1, 0x18, osr_val) == 0;
    }

    /**
    * @brief  Sets the downsampling rate of the magnetometer.
    * @retval True if the downsampling rate was set successfully, false otherwise.
    */
    bool setDownsamplingRate(MagDownSampleRatio dsr) override
    {
        log_e("QMC6309 does not support downsampling rate setting");
        return false;
    }

    /**
     * @brief  Sets the low-pass filter of the magnetometer.
     * @note   This function configures the magnetometer's low-pass filter,
     *         affecting the trade-off between noise and measurement speed.
     * @param  lpf: LPF_1, LPF_2, LPF_4, LPF_8, LPF_16
     * @retval True if the low-pass filter was set successfully, false otherwise.
     */
    bool setLowPassFilter(MagLowPassFilter lpf)
    {
        uint8_t lpf_val = 0x00;
        switch (lpf) {
        case MagLowPassFilter::LPF_1:
            lpf_val = 0x00 << 5;
            break;
        case MagLowPassFilter::LPF_2:
            lpf_val = 0x01 << 5;
            break;
        case MagLowPassFilter::LPF_4:
            lpf_val = 0x02 << 5;
            break;
        case MagLowPassFilter::LPF_8:
            lpf_val = 0x03 << 5;
            break;
        case MagLowPassFilter::LPF_16:
            lpf_val = 0x04 << 5;
            break;
        default:
            log_e("Invalid low-pass filter");
            return false;
        }
        return updateBits(REG_0x0A_CMD1, 0xE0, lpf_val) == 0;
    }

    /**
       * @brief  Gets the chip ID of the magnetometer.
       * @note   This function retrieves the chip ID from the sensor.
       * @retval The chip ID.
       */
    uint8_t getChipID() const
    {
        return _info.uid;
    }

    /**
     * @brief  Configures the magnetometer with multiple parameters.
     * @note   QMC6309 supports oversampling rate (OSR) settings.
     * @param  mode: SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
     * @param  range: FS_8G, FS_16G, FS_32G
     * @param  data_rate_hz: Allowed values are 1.0, 10.0, 50.0, 100.0 and 200.0HZ.
     * @param  osr: OSR_8, OSR_4, OSR_2, OSR_1
     * @param  dsr: QMC6309 does not support downsampling rate settings; this parameter is ignored.
     * @retval True if the configuration was successful, false otherwise.
     */
    bool configMagnetometer(OperationMode mode, MagFullScaleRange range, float data_rate_hz,
                            MagOverSampleRatio osr, MagDownSampleRatio dsr = MagDownSampleRatio::DSR_1) override
    {
        if (!setOperationMode(mode)) {
            return false;
        }
        if (!setFullScaleRange(range)) {
            return false;
        }
        if (!setOutputDataRate(data_rate_hz)) {
            return false;
        }
        if (!setOversamplingRate(osr)) {
            return false;
        }
        return true;
    }

    /**
    * @brief  Sets the set/reset mode of the magnetometer.
    * @note   According to datasheet section 9.2.4, there are 3 modes:
    *         - SET_AND_RESET_ON: Both set and reset enabled (default)
    *         - SET_ONLY_ON: Only set enabled
    *         - SET_AND_RESET_OFF: Both disabled
    *         Note: Value 0b10 is reserved.
    * @param  mode: The desired set/reset mode.
    * @retval True if the mode was set successfully, false otherwise.
    */
    bool setSetResetMode(MagSetResetMode mode)
    {
        uint8_t sr_val = 0x00;
        switch (mode) {
        case MagSetResetMode::SET_AND_RESET_ON:
            sr_val = 0x00;
            break;
        case MagSetResetMode::SET_ONLY_ON:
            sr_val = 0x01;
            break;
        case MagSetResetMode::SET_AND_RESET_OFF:
            sr_val = 0x03;
            break;
        default:
            log_e("Invalid set/reset mode");
            return false;
        }
        return updateBits(REG_0x0B_CMD2, 0x03, sr_val) == 0;
    }

    /**
     * @brief  Set the voltage mode for QMC6309H variant.
     * @note   This function configures the voltage mode for the QMC6309H variant. QMC6309 invalid this setting.
     * @param  mode: The desired voltage mode. see VoltageMode
     * @retval None
     */
    void setVoltageMode(VoltageMode mode)
    {
        _qmc6309h_vdd_mode = mode;
    }

private:

    static constexpr uint8_t QMC6309_CHIP_ID = 0x90;
    static constexpr uint8_t QMC6309H_CHIP_ID = 0xA5;
    static constexpr uint8_t REG_0x00_CHIP_ID = 0x00;
    static constexpr uint8_t REG_0x01_LSB_DX = 0x01;
    static constexpr uint8_t REG_0x02_MSB_DX = 0x02;
    static constexpr uint8_t REG_0x03_LSB_DY = 0x03;
    static constexpr uint8_t REG_0x04_MSB_DY = 0x04;
    static constexpr uint8_t REG_0x05_LSB_DZ = 0x05;
    static constexpr uint8_t REG_0x06_MSB_DZ = 0x06;
    static constexpr uint8_t REG_0x09_STAT = 0x09;
    static constexpr uint8_t REG_0x0A_CMD1 = 0x0A;
    static constexpr uint8_t REG_0x0B_CMD2 = 0x0B;
    static constexpr uint8_t REG_0x40_IO_CONFIG = 0x40;
    static constexpr uint8_t REG_0x28_OTP_CTRL = 0x28;
    static constexpr uint8_t REG_0x29_SIGN = 0x29;

    static constexpr uint8_t REG_0x0E_SELFTEST_CTRL = 0x0E;  // Self-test control register
    static constexpr uint8_t REG_0x13_SELFTEST_X = 0x13;     // Self-test X-axis data register
    static constexpr uint8_t REG_0x14_SELFTEST_Y = 0x14;     // Self-test Y-axis data register
    static constexpr uint8_t REG_0x15_SELFTEST_Z = 0x15;     // Self-test Z-axis data register

    static constexpr uint8_t MASK_DRDY = 0x01;
    static constexpr uint8_t MASK_OVFL = 0x02;
    static constexpr uint8_t MASK_ST_RDY = 0x04;
    static constexpr uint8_t MASK_NVM_RDY = 0x08;
    static constexpr uint8_t MASK_NVM_LOAD_DONE = 0x10;
    static constexpr uint8_t MASK_SOFT_RST = 0x80;
    static constexpr uint8_t MASK_ODR = 0x70;
    static constexpr uint8_t MASK_RNG = 0x0C;
    static constexpr uint8_t MASK_SET_RESET_MODE = 0x03;

    bool _is_qmc6309h = false;
    VoltageMode _qmc6309h_vdd_mode = VoltageMode::KEEP;

    bool applyQMC6309HVoltageMode()
    {
        if (!_is_qmc6309h || _qmc6309h_vdd_mode == VoltageMode::KEEP) {
            return true;
        }

        int reg = readReg(REG_0x40_IO_CONFIG);
        if (reg < 0) {
            log_e("Failed to read QMC6309H IO config");
            return false;
        }

        uint8_t value = (uint8_t)reg;
        switch (_qmc6309h_vdd_mode) {
        case VoltageMode::VDD_MODE1:
            value = (value & 0x3F) | 0x80;
            break;
        case VoltageMode::VDD_MODE2:
            value = (value & 0x3F) | 0x40;
            break;
        case VoltageMode::KEEP:
            return true;
        }

        if (writeReg(REG_0x40_IO_CONFIG, value) < 0) {
            log_e("Failed to write QMC6309H IO config");
            return false;
        }
        hal->delay(1);
        return true;
    }

    bool reloadOTP()
    {
        for (uint8_t retry = 0; retry < 20; ++retry) {
            if (writeReg(REG_0x28_OTP_CTRL, 0x02) < 0) {
                hal->delay(2);
                continue;
            }

            hal->delay(2);

            for (uint8_t count = 0; count < 100; ++count) {
                int status = readReg(REG_0x09_STAT);
                if (status >= 0 && isBitSet(status, 4)) {
                    return true;
                }
                hal->delay(1);
            }
        }

        log_e("Failed to reload OTP");
        return false;
    }

    bool initImpl(uint8_t param) override
    {
        (void)param;

        if (!reset()) {
            return false;
        }

        if (!reloadOTP()) {
            return false;
        }

        _info.uid = readReg(REG_0x00_CHIP_ID);
        _info.manufacturer = "QSTMagnetic";
        _info.type = SensorType::MAGNETOMETER;
        _info.i2c_address = _addr;
        _info.version = 1;  // Set a default version

        if (_info.uid == QMC6309_CHIP_ID) {
            _is_qmc6309h = false;
            _info.model = "QMC6309";
        } else if (_info.uid == QMC6309H_CHIP_ID) {
            _is_qmc6309h = true;
            _info.model = "QMC6309H";
        } else {
            log_e("Unsupported chip ID");
            return false;
        }

        if (!applyQMC6309HVoltageMode()) {
            return false;
        }

        // Follow vendor init defaults: HPFM + 32G + set/reset on + OSR1=8 + LPF4.
        if (!setSetResetMode(MagSetResetMode::SET_AND_RESET_ON)) {
            return false;
        }
        if (!setFullScaleRange(MagFullScaleRange::FS_32G)) {
            return false;
        }
        if (!setOutputDataRate(1.0f)) {
            return false;
        }
        if (!setOversamplingRate(MagOverSampleRatio::OSR_8)) {
            return false;
        }
        if (!setLowPassFilter(MagLowPassFilter::LPF_4)) {
            return false;
        }
        if (!setOperationMode(OperationMode::CONTINUOUS_MEASUREMENT)) {
            return false;
        }

        _config.mode = OperationMode::CONTINUOUS_MEASUREMENT;
        _config.range = 32.0f;
        _config.sample_rate = 1.0f;
        _config.latency = 0;
        _config.type = SensorType::MAGNETOMETER;

        return true;
    }
};
