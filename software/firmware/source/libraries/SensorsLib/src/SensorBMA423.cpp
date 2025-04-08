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
 * @file      SensorBMA423.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-03-31
 * @note      Most source code references come from the https://github.com/boschsensortec/BMA423-Sensor-API
 *            Simplification for Arduino
 */
#include "SensorBMA423.hpp"
#include "REG/BMA423Config.h"

SensorBMA423::SensorBMA423() : comm(nullptr) {}

SensorBMA423::~SensorBMA423()
{
    if (comm) {
        comm->deinit();
    }
}

#if defined(ARDUINO)
bool SensorBMA423::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
        return false;
    }
    return initImpl();
}

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool SensorBMA423::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
        return false;
    }
    return initImpl();
}
#else   /*USEING_I2C_LEGACY*/
bool SensorBMA423::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
        return false;
    }
    return initImpl();
}
#endif  //ESP_PLATFORM
#endif  //ARDUINO


bool SensorBMA423::begin(SensorCommCustom::CustomCallback callback,
                         SensorCommCustomHal::CustomHalCallback hal_callback,
                         uint8_t addr)
{
    if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
            callback, hal_callback, addr, comm, hal)) {
        return false;
    }
    return initImpl();
}

void SensorBMA423::reset()
{
    comm->writeRegister(RESET_REG, 0xB6);
    hal->delay(20);
}

bool SensorBMA423::enablePowerSave()
{
    uint8_t val;
    val = comm->readRegister(POWER_CONF_ADDR);
    val |= ADVANCE_POWER_SAVE_MSK;
    comm->writeRegister(POWER_CONF_ADDR, val);
    return true;
}

bool SensorBMA423::disablePowerSave()
{
    uint8_t val;
    val = comm->readRegister(POWER_CONF_ADDR);
    val &= ~(ADVANCE_POWER_SAVE_MSK);
    comm->writeRegister(POWER_CONF_ADDR, val);
    return true;
}

void SensorBMA423::disableInterruptCtrl()
{
    comm->writeRegister(INIT_CTRL_ADDR, (uint8_t)0x00);
}

void SensorBMA423::enableInterruptCtrl()
{
    comm->writeRegister(INIT_CTRL_ADDR, (uint8_t)0x01);
}

bool SensorBMA423::enableAccelerometer()
{
    uint8_t val;
    val = comm->readRegister(POWER_CTRL_ADDR);
    val |= ACCEL_ENABLE_MSK;
    comm->writeRegister(POWER_CTRL_ADDR, val);
    return true;
}

bool SensorBMA423::disableAccelerometer()
{
    uint8_t val;
    val = comm->readRegister( POWER_CTRL_ADDR);
    val &= (~ACCEL_ENABLE_MSK);
    comm->writeRegister(POWER_CTRL_ADDR, val);
    return true;
}

bool SensorBMA423::configAccelerometer(AccelRange range, AccelODR odr,
                                       BandWidth bw,
                                       PerformanceMode perfMode )

{
    uint8_t buffer[2] = {0, 0};
    if (perfMode == PERF_CONTINUOUS_MODE) {
        if (bw > BW_NORMAL_AVG4) {
            return false;
        }
    } else if (perfMode == PERF_CIC_AVG_MODE) {
        if (bw > BW_RES_AVG128) {
            return false;
        }
    } else {
        return false;
    }
    if ((odr < ODR_0_78HZ) || (odr > ODR_1600HZ)) {
        return false;
    }

    buffer[0] = odr & 0x0F;
    buffer[0] |= (uint8_t)(bw << 4);
    buffer[0] |= (uint8_t)(perfMode << 7);
    buffer[1] = range & 0x03;

    /* Burst write is not possible in
    suspend mode hence individual write is
    used with hal->delay of 1 ms */
    comm->writeRegister(ACCEL_CONFIG_ADDR, buffer[0]);
    hal->delay(2);
    comm->writeRegister(ACCEL_CONFIG_ADDR + 1, buffer[1]);
    return true;
}


bool SensorBMA423::getAccelRaw(int16_t *rawBuffer)
{
    uint8_t buffer[6] = {0};
    if (comm->readRegister(DATA_8_ADDR, buffer, 6) != -1) {
        /* Accel data x axis */
        rawBuffer[0] = (int16_t)(buffer[1] << 8) | (buffer[0]);
        /* Accel data y axis */
        rawBuffer[1] = (int16_t)(buffer[3] << 8) | (buffer[2]);
        /* Accel data z axis */
        rawBuffer[2] = (int16_t)(buffer[5] << 8) | (buffer[4]);
    } else {
        return false;
    }
    return true;
}

bool SensorBMA423::getAccelerometer(int16_t &x, int16_t &y, int16_t &z)
{
    int16_t raw[3];
    if (getAccelRaw(raw)) {
        x = raw[0] / 16;
        y = raw[1] / 16;
        z = raw[2] / 16;
        return true;
    }
    return false;
}

float SensorBMA423::getTemperature(TemperatureUnit unit)
{
    int32_t raw = comm->readRegister(TEMPERATURE_ADDR);
    /* '0' value read from the register corresponds to 23 degree C */
    raw = (raw * 1000) + (23 * 1000);
    switch (unit) {
    case TEMP_FAHRENHEIT:
        /* Temperature in degree Fahrenheit */
        /* 1800 = 1.8 * 1000 */
        raw = ((raw / 1000) * 1800) + (32 * 1000);
        break;
    case TEMP_KELVIN:
        /* Temperature in degree Kelvin */
        /* 273150 = 273.15 * 1000 */
        raw = raw + 273150;
        break;
    default:
        break;
    }
    float res = (float)raw / (float)1000.0;
    /* 0x80 - raw read from the register and 23 is the ambient raw added.
     * If the raw read from register is 0x80, it means no valid
     * information is available */
    if (((raw - 23) / 1000) == 0x80) {
        return 0;
    }
    return res;
}


uint8_t SensorBMA423::direction()
{
    int16_t x = 0, y = 0, z = 0;
    getAccelerometer(x, y, z);
    uint16_t absX = abs(x);
    uint16_t absY = abs(y);
    uint16_t absZ = abs(z);
    if ((absZ > absX) && (absZ > absY)) {
        if (z > 0) {
            return  DIRECTION_TOP;
        } else {
            return DIRECTION_BOTTOM;
        }
    } else if ((absY > absX) && (absY > absZ)) {
        if (y > 0) {
            return DIRECTION_TOP_RIGHT;
        } else {
            return  DIRECTION_BOTTOM_LEFT;
        }
    } else {
        if (x < 0) {
            return  DIRECTION_BOTTOM_RIGHT;
        } else {
            return DIRECTION_TOP_LEFT;
        }
    }
    return 0;
}

bool SensorBMA423::setRemapAxes(SensorRemap remap)
{
    //Top
    // No.1 REG: 0x3e -> 0x88   REG: 0x3f -> 0x0
    // No.2 REG: 0x3e -> 0xac   REG: 0x3f -> 0x0
    // No.3 REG: 0x3e -> 0x85   REG: 0x3f -> 0x0
    // No.4 REG: 0x3e -> 0xa1   REG: 0x3f -> 0x0

    // Bottom
    // No.5 REG: 0x3e -> 0x81   REG: 0x3f -> 0x1
    // No.6 REG: 0x3e -> 0xa5   REG: 0x3f -> 0x1
    // No.7 REG: 0x3e -> 0x8c   REG: 0x3f -> 0x1
    // No.8 REG: 0x3e -> 0xa8   REG: 0x3f -> 0x1

    uint8_t configReg0[] = {0x88, 0xAC, 0x85, 0xA1, 0x81, 0xA5, 0x8C, 0xA8};
    if (remap > sizeof(configReg0) / sizeof(configReg0[0])) {
        return false;
    }
    uint8_t buffer[FEATURE_SIZE] = {0};
    uint8_t index = AXES_REMAP_OFFSET;
    if (comm->readRegister(FEATURE_CONFIG_ADDR,  buffer, FEATURE_SIZE) == -1) {
        return false;
    }
    buffer[index] = configReg0[remap];
    buffer[index + 1] = remap >= 4 ? 0x00 : 0x01;
    return comm->writeRegister(FEATURE_CONFIG_ADDR,  buffer, FEATURE_SIZE) != -1;
}

bool SensorBMA423::setStepCounterWatermark(uint16_t watermark)
{
    uint8_t buffer[FEATURE_SIZE] = {0};
    uint8_t index = STEP_CNTR_OFFSET;
    uint16_t wm_lsb = 0;
    uint16_t wm_msb = 0;
    int rslt;
    uint16_t data = 0;

    rslt = comm->readRegister(FEATURE_CONFIG_ADDR, buffer, FEATURE_SIZE);
    if (rslt != -1) {
        wm_lsb = buffer[index];
        wm_msb = buffer[index + 1] << 8;
        data = wm_lsb | wm_msb;
        /* Sets only watermark bits in the complete 16 bits of data */
        data = ((data & ~(0x03FFU)) | (watermark & 0x03FFU));
        /* Splits 16 bits of data to individual 8 bits data */
        buffer[index] = (uint8_t)(data & 0x00FFU);
        buffer[index + 1] = (uint8_t)((data & 0xFF00U) >> 8);
        /* Writes stepcounter watermark settings in the sensor */
        rslt = comm->writeRegister(FEATURE_CONFIG_ADDR, buffer, FEATURE_SIZE);
    }
    return rslt != -1;
}

bool SensorBMA423::disablePedometer()
{
    return enablePedometer(false);
}

bool SensorBMA423::enablePedometer(bool enable)
{
    int rslt;
    uint8_t buffer[FEATURE_SIZE] = {0};
    /* Step detector enable bit pos. is 1 byte ahead of the base address */
    uint8_t index = STEP_CNTR_OFFSET + 1;
    rslt = comm->readRegister(FEATURE_CONFIG_ADDR, buffer, FEATURE_SIZE);
    if (rslt != -1) {
        buffer[index] = ((buffer[index] & ~0x08) | ((enable << 3) & 0x08));
        rslt = comm->writeRegister(FEATURE_CONFIG_ADDR, buffer, FEATURE_SIZE);
    }
    return rslt != -1;
}

uint32_t SensorBMA423::getPedometerCounter()
{
    uint8_t buffer[4] = {0};
    /* Reads the step counter output data from the gpio register */
    int rslt = comm->readRegister(STEP_CNT_OUT_0_ADDR, buffer, 4);
    if (rslt != -1) {
        return ((uint32_t)buffer[0]) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
    }
    return 0;
}

void SensorBMA423::resetPedometer()
{
    uint8_t buffer[FEATURE_SIZE] = {0};
    /* Reset bit is 1 byte ahead of base address */
    uint8_t index = STEP_CNTR_OFFSET + 1;
    int rslt = comm->readRegister(FEATURE_CONFIG_ADDR, buffer, FEATURE_SIZE);
    if (rslt != -1) {
        buffer[index] = ((buffer[index] & ~0x04U) | ((1 << 2U) & 0x04U));
        comm->writeRegister(FEATURE_CONFIG_ADDR, buffer, FEATURE_SIZE);
    }
}

bool SensorBMA423::enableFeature(uint8_t feature, uint8_t enable)
{
    uint8_t buffer[FEATURE_SIZE] = {0};
    int rslt;
    uint8_t len;
    /* Update the length for read and write */
    update_len(&len, feature, enable);
    rslt = comm->readRegister(FEATURE_CONFIG_ADDR, buffer, len);
    if (rslt != -1) {
        if (enable) {
            /* Enables the feature */
            rslt = feature_enable(feature, len, buffer);
        } else {
            /* Disables the feature */
            rslt = feature_disable(feature, len, buffer);
        }
    }
    return rslt != -1;
}

uint16_t SensorBMA423::readIrqStatus()
{
    uint8_t data[2] = {0};
    if (comm->readRegister(INT_STAT_0_ADDR, data, 2) != -1) {
        int_status = data[0] | (data[1] << 8);
        return int_status;
    }
    return 0;
}

uint16_t SensorBMA423::getIrqStatus()
{
    return int_status;
}

bool SensorBMA423::configInterrupt(
    /*! Trigger condition of interrupt pin */
    uint8_t edge_ctrl,
    /*! Level of interrupt pin */
    uint8_t level,
    /*! Behaviour of interrupt pin to open drain */
    uint8_t od,
    /*! Output enable for interrupt pin */
    uint8_t output_en,
    /*! Input enable for interrupt pin */
    uint8_t input_en,
    /*! Variable used to select the interrupt pin1 or pin2 for interrupt configuration. */
    uint8_t int_line
)
{
    uint8_t interrupt_address_array[2] = {INT1_IO_CTRL_ADDR, INT2_IO_CTRL_ADDR};
    uint8_t data = 0;
    if (int_line > 1) {
        return false;
    }
    data = ((uint8_t)((edge_ctrl & INT_EDGE_CTRL_MASK) |
                      ((level << 1) & INT_LEVEL_MASK) |
                      ((od << 2) & INT_OPEN_DRAIN_MASK) |
                      ((output_en << 3) & INT_OUTPUT_EN_MASK) |
                      ((input_en << 4) & INT_INPUT_EN_MASK)));

    this->int_line = int_line;

    return  comm->writeRegister(interrupt_address_array[int_line],  &data, 1) != -1;
}

bool SensorBMA423::configFeatureInterrupt(uint16_t  feature_interrupt_mask, bool enable)
{
    return  interruptMap(int_line,  feature_interrupt_mask, enable);
}


bool SensorBMA423::enablePedometerIRQ()
{
    return  (interruptMap(int_line,  INT_STEP_CNTR, true));
}

bool SensorBMA423::enableTiltIRQ()
{
    return  (interruptMap(int_line, INT_TILT, true));
}

bool SensorBMA423::enableWakeupIRQ()
{
    return  (interruptMap(int_line, INT_WAKEUP, true));
}

bool SensorBMA423::enableAnyNoMotionIRQ()
{
    return  (interruptMap(int_line, INT_ANY_NO_MOTION, true));
}

bool SensorBMA423::enableActivityIRQ()
{
    return  (interruptMap(int_line, INT_ACTIVITY, true));
}

bool SensorBMA423::disablePedometerIRQ()
{
    return  (interruptMap(int_line,  INT_STEP_CNTR, false));
}

bool SensorBMA423::disableTiltIRQ()
{
    return  (interruptMap(int_line, INT_TILT, false));
}

bool SensorBMA423::disableWakeupIRQ()
{
    return  (interruptMap(int_line, INT_WAKEUP, false));
}

bool SensorBMA423::disableAnyNoMotionIRQ()
{
    return  (interruptMap(int_line, INT_ANY_NO_MOTION, false));
}

bool SensorBMA423::disableActivityIRQ()
{
    return  (interruptMap(int_line, INT_ACTIVITY, false));
}


bool SensorBMA423::isActivity()
{
    return (int_status & ACTIVITY_INT);
}

bool SensorBMA423::isTilt()
{
    return (int_status & TILT_INT);
}

bool SensorBMA423::isDoubleTap()
{
    return (int_status & WAKEUP_INT);
}

bool SensorBMA423::isAnyNoMotion()
{
    return (int_status & ACTIVITY_INT);
}

bool SensorBMA423::isPedometer()
{
    return (int_status & STEP_CNTR_INT);
}

bool SensorBMA423::interruptMap(uint8_t int_line, uint16_t int_map, uint8_t enable)
{
    int rslt;
    uint8_t data[3] = {0, 0, 0};
    uint8_t index[2] = {INT_MAP_1_ADDR, INT_MAP_2_ADDR};
    rslt = comm->readRegister(INT_MAP_1_ADDR, data, 3);
    if (enable) {
        /* Feature interrupt mapping */
        data[int_line] |= (uint8_t)(int_map & (0x00FF));
        /* Hardware interrupt mapping */
        if (int_line == INTR2_MAP)
            data[2] |= (uint8_t)((int_map & (0xFF00)) >> 4);
        else
            data[2] |= (uint8_t)((int_map & (0xFF00)) >> 8);

        rslt = comm->writeRegister(index[int_line], &data[int_line], 1);
        rslt = comm->writeRegister(INT_MAP_DATA_ADDR, &data[2], 1);

    } else {
        /* Feature interrupt un-mapping */
        data[int_line] &= (~(uint8_t)(int_map & (0x00FF)));
        /* Hardware interrupt un-mapping */
        if (int_line == INTR2_MAP)
            data[2] &= (~(uint8_t)((int_map & (0xFF00)) >> 4));
        else
            data[2] &= (~(uint8_t)((int_map & (0xFF00)) >> 8));

        rslt = comm->writeRegister(index[int_line], &data[int_line], 1);
        rslt = comm->writeRegister(INT_MAP_DATA_ADDR, &data[2], 1);

    }
    return rslt != -1;
}

/*!
 *  @brief This API sets the interrupt mode in the sensor.
 */
bool SensorBMA423::setInterruptMode(uint8_t mode)
{
    if (mode == NON_LATCH_MODE || mode == LATCH_MODE)
        return comm->writeRegister(INTR_LATCH_ADDR, &mode, 1) != -1;
    return false;
}


bool SensorBMA423::feature_disable(uint8_t feature, uint8_t len, uint8_t *buffer)
{
    uint8_t index = 0;

    /* Disable step counter */
    if ((feature & FEATURE_STEP_CNTR) > 0) {
        /* Step counter enable bit pos. is 1 byte ahead of the
        base address */
        index = STEP_CNTR_OFFSET + 1;
        buffer[index] = buffer[index] & (~STEP_CNTR_EN_MSK);
    }

    /* Disable activity */
    if ((feature & FEATURE_ACTIVITY) > 0) {
        /* Activity enable bit pos. is 1 byte ahead of the
        base address */
        index = STEP_CNTR_OFFSET + 1;
        buffer[index] = buffer[index] & (~ACTIVITY_EN_MSK);
    }
    /* Disable tilt */
    if ((feature & FEATURE_TILT) > 0) {
        /* Tilt enable bit pos. is the base address(0x3A) of tilt */
        index = TILT_OFFSET;
        buffer[index] = buffer[index] & (~TILT_EN_MSK);
    }

    /* Disable wakeup */
    if ((feature & FEATURE_WAKEUP) > 0) {
        /* Tilt enable bit pos. is the base address(0x38) of wakeup */
        index = WAKEUP_OFFSET;
        buffer[index] = buffer[index] & (~WAKEUP_EN_MSK);
    }

    /* Disable anymotion/nomotion */
    if ((feature & FEATURE_ANY_MOTION) > 0 || (feature & FEATURE_NO_MOTION) > 0) {
        /* Any/Nomotion enable bit pos. is 1 bytes ahead of the
        any/nomotion base address(0x00) */
        index = 1;

        if ((feature & FEATURE_ANY_MOTION) > 0) {
            /* Disable anymotion */
            buffer[index] = buffer[index] | ANY_NO_MOTION_SEL_MSK;
        } else {
            /* Disable nomotion */
            buffer[index] = buffer[index] & (~ANY_NO_MOTION_SEL_MSK);
        }
        /* Any/Nomotion axis enable bit pos. is 3 byte ahead of the
        any/nomotion base address(0x00) */
        index = 3;
        buffer[index] = buffer[index] & (~ANY_NO_MOTION_AXIS_EN_MSK);
    }
    /* Write the configured settings in the sensor */
    return comm->writeRegister(FEATURE_CONFIG_ADDR, buffer, len) != -1;
}

int SensorBMA423::feature_enable(uint8_t feature, uint8_t len, uint8_t *buffer)
{
    uint8_t index = 0;

    /* Enable step counter */
    if ((feature & FEATURE_STEP_CNTR) > 0) {
        /* Step counter enable bit pos. is 1 byte ahead of the
        base address */
        index = STEP_CNTR_OFFSET + 1;
        buffer[index] = buffer[index] | STEP_CNTR_EN_MSK;
    }

    /* Enable activity */
    if ((feature & FEATURE_ACTIVITY) > 0) {
        /* Activity enable bit pos. is 1 byte ahead of the
        base address */
        index = STEP_CNTR_OFFSET + 1;
        buffer[index] = buffer[index] | ACTIVITY_EN_MSK;
    }
    /* Enable tilt */
    if ((feature & FEATURE_TILT) > 0) {
        /* Tilt enable bit pos. is the base address(0x3A) of tilt */
        index = TILT_OFFSET;
        buffer[index] = buffer[index] | TILT_EN_MSK;
    }

    /* Enable wakeup */
    if ((feature & FEATURE_WAKEUP) > 0) {
        /* Wakeup enable bit pos. is the base address(0x38) of wakeup */
        index = WAKEUP_OFFSET;
        buffer[index] = buffer[index] | WAKEUP_EN_MSK;
    }

    /* Enable anymotion/nomotion */
    if ((feature & FEATURE_ANY_MOTION) > 0 || (feature & FEATURE_NO_MOTION) > 0) {
        /* Any/Nomotion enable bit pos. is 1 bytes ahead of the
        any/nomotion base address(0x00) */
        index = 1;

        if ((feature & FEATURE_ANY_MOTION) > 0) {
            /* Enable anymotion */
            buffer[index] = buffer[index] & (~ANY_NO_MOTION_SEL_MSK);
        } else {
            /* Enable nomotion */
            buffer[index] = buffer[index] | ANY_NO_MOTION_SEL_MSK;
        }
    }

    /* Write the feature enable settings in the sensor */
    return comm->writeRegister(FEATURE_CONFIG_ADDR, buffer, len) != -1;
}

void SensorBMA423::update_len(uint8_t *len, uint8_t feature, uint8_t enable)
{
    uint8_t length = FEATURE_SIZE;

    if ((feature == FEATURE_ANY_MOTION) || (feature == FEATURE_NO_MOTION)) {
        /* Change the feature length to 2 for reading and writing of 2 bytes for
        any/no-motion enable */
        length = ANYMOTION_EN_LEN;

        /* Read and write 4 byte to disable the any/no motion completely along with
        all axis */
        if (!enable) {
            /*Change the feature length to 4 for reading and writing
            of 4 bytes for any/no-motion enable */
            length = length + 2;
        }
    }
    *len = length;
}

bool SensorBMA423::configure()
{
    uint8_t val;
    val = comm->readRegister(INTERNAL_STAT);
    if (val == ASIC_INITIALIZED) {
        log_d("No need configure!");
        readIrqStatus();    //clear irq status
        return true;
    }

    disablePowerSave();
    hal->delay(1);
    disableInterruptCtrl();


    const uint8_t *stream_data = bma423_config_file;
    const uint8_t maxReadWriteLength = 32;

    for (size_t index = 0; index < CONFIG_STREAM_SIZE; index += maxReadWriteLength) {
        uint8_t msb = (uint8_t)((index / 2) >> 4);
        uint8_t lsb = ((index / 2) & 0x0F);
        comm->writeRegister(RESERVED_REG_5B_ADDR,  lsb);
        comm->writeRegister(RESERVED_REG_5C_ADDR,  msb);
        comm->writeRegister(FEATURE_CONFIG_ADDR,  (uint8_t *)(stream_data + index), maxReadWriteLength);
    }

    enableInterruptCtrl();

    hal->delay(150);

    val = comm->readRegister(INTERNAL_STAT);
    if (val == ASIC_INITIALIZED) {
        log_d("BMA configure SUCCESS!");
    } else {
        log_d("BMA configure FAILED!");
    }
    return val == ASIC_INITIALIZED;
}

bool SensorBMA423::initImpl()
{
    uint8_t id = 0x00;
    int retry = 5;
    while (retry--) {
        id = comm->readRegister(CHIP_ID_ADDR);
        if (id != CHIP_ID) {
            reset();
        }
    }
    if (id == CHIP_ID) {
        return configure();
    }
    log_d("ChipID:0x%x should be 0x%x", id, CHIP_ID);
    return false;
}

