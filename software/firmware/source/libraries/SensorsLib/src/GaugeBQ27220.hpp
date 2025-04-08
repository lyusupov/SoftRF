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
 * @file      GaugeBQ27220.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-13
 */
#pragma once

#include "REG/BQ27220Constants.h"
#include "SensorPlatform.hpp"

// BQ27220 Unique device address
static constexpr uint8_t BQ27220_SLAVE_ADDRESS  = (0x55);

// *INDENT-OFF*
class FuelGaugeOperationStatus {
private:
    bool isConfigUpdateMode;
    bool isBtpThresholdExceeded;
    bool isCapacityAccumulationThrottled;
    bool isInitializationComplete;
    bool isDischargeCycleCompliant;
    bool isBatteryVoltageBelowEdv2;
    uint8_t securityAccessLevel;
    bool isCalibrationModeEnabled;
public:
    FuelGaugeOperationStatus(uint16_t value) {
        // parse CFGUPDATE bit
        isConfigUpdateMode = (value & 0x0400) != 0;
        // parse BTPINT bit
        isBtpThresholdExceeded = (value & 0x80) != 0;
        // parse SMTH bit
        isCapacityAccumulationThrottled = (value & 0x40) != 0;
        // parse INITCOMP bit
        isInitializationComplete = (value & 0x20) != 0;
        // parse VDQ bit
        isDischargeCycleCompliant = (value & 0x10) != 0;
        // parse EDV2 bit
        isBatteryVoltageBelowEdv2 = (value & 0x08) != 0;
        // parse SEC bit
        securityAccessLevel = (value >> 1) & 0x03;
        // parse CALMD bit
        isCalibrationModeEnabled = (value & 0x01) != 0;
    }

    // Gets whether the fuel gauge is in configuration update mode
    bool getIsConfigUpdateMode() const {
        return isConfigUpdateMode;
    }

    // Get whether the BTP threshold has been exceeded
    bool getIsBtpThresholdExceeded() const {
        return isBtpThresholdExceeded;
    }

    // Gets whether the remaining capacity accumulation is throttled
    bool getIsCapacityAccumulationThrottled() const {
        return isCapacityAccumulationThrottled;
    }

    // Get whether the fuel gauge initialization is completed
    bool getIsInitializationComplete() const {
        return isInitializationComplete;
    }

    // Get whether the current discharge cycle complies with FCC update requirements
    bool getIsDischargeCycleCompliant() const {
        return isDischargeCycleCompliant;
    }

    // Get whether the battery voltage is lower than the EDV2 threshold
    bool getIsBatteryVoltageBelowEdv2() const {
        return isBatteryVoltageBelowEdv2;
    }

    // Get the current security access level
    uint8_t getSecurityAccessLevel() const {
        return securityAccessLevel;
    }

    // Gets whether calibration mode is enabled
    bool getIsCalibrationModeEnabled() const {
        return isCalibrationModeEnabled;
    }
};

class BatteryStatus {
protected:
    struct {
        //* Full discharge detected. This flag is set and cleared based on the SOC
        //* Flag Config B option selected.
        bool fullDischargeDetected;
        //* OCV measurement update is complete. True when set
        bool ocvMeasurementUpdateComplete;
        //* Status bit indicating that an OCV read failed due to current.
        //* This bit can only be set if a battery is present after receiving an OCV_CMD(). True when set
        bool ocvReadFailedDueToCurrent;
        //* The device operates in SLEEP mode when set.
        //* This bit will be temporarily cleared during AD measurements in SLEEP mode.
        bool inSleepMode;
        //* Over-temperature is detected during charging. If Operation Config B [INT_OT] bit = 1,
        //* the SOC_INT pin toggles once when the [OTC] bit is set.
        bool overTemperatureDuringCharging;
        //* Over-temperature detected during discharge condition. True when set. If Operation Config B [INT_OT] bit = 1,
        //* the SOC_INT pin toggles once when the [OTD] bit is set.
        bool overTemperatureDuringDischarge;
        //* Full charge detected. This flag is set and cleared based on the SOC Flag Config A and SOC Flag Config B options selected.
        bool fullChargeDetected;
        //* Charge Inhibit: If set, indicates that charging should not begin because the Temperature() is outside the range
        //* [Charge Inhibit Temp Low, Charge Inhibit Temp High]. True when set
        bool chargeInhibited;
        //* Termination of charging alarm. This flag is set and cleared based on the selected SOC Flag Config A option.
        bool chargingTerminationAlarm;
        //* A good OCV measurement was made. True when set
        bool goodOcvMeasurement;
        //* Detects inserted battery. True when set.
        bool batteryInserted;
        //* Battery presence detected. True when set.
        bool batteryPresent;
        //* Termination discharge alarm. This flag is set and cleared according to the selected SOC Flag Config A option.
        bool dischargeTerminationAlarm;
        //* System shutdown bit indicating that the system should be shut down. True when set. If set, the SOC_INT pin toggles once.
        bool systemShutdownRequired;
        //* When set, the device is in DISCHARGE mode; when cleared, the device is in CHARGING or RELAXATION mode.
        bool inDischargeMode;
    } status;
public:
    /**
     * @brief Constructor for BatteryStatus class.
     * @details Initializes the battery status fields based on the given 16-bit bitmaps.
     * @param bitmaps A 16-bit unsigned integer representing the battery status bitmaps.
     */
    BatteryStatus(uint16_t bitmaps){
        uint8_t hsb = (bitmaps >> 8) & 0xFF;
        uint8_t lsb = bitmaps & 0xFF;
        status.fullDischargeDetected = (hsb & _BV(7)) != 0;
        status.ocvMeasurementUpdateComplete = (hsb & _BV(6)) != 0;
        status.ocvReadFailedDueToCurrent = (hsb & _BV(5)) != 0;
        status.inSleepMode = (hsb & _BV(4)) != 0;
        status.overTemperatureDuringCharging = (hsb & _BV(3)) != 0;
        status.overTemperatureDuringDischarge = (hsb & _BV(2)) != 0;
        status.fullChargeDetected = (hsb & _BV(1)) != 0;
        status.chargeInhibited = (hsb & _BV(0)) != 0;

        // status.RSVD = (lsb & (1 << 7))!= 0;
        status.chargingTerminationAlarm = (lsb & _BV(6)) != 0;
        status.goodOcvMeasurement = (lsb & _BV(5)) != 0;
        status.batteryInserted = (lsb & _BV(4)) != 0;
        status.batteryPresent = (lsb & _BV(3)) != 0;
        status.dischargeTerminationAlarm = (lsb & _BV(2)) != 0;
        status.systemShutdownRequired = (lsb & _BV(1)) != 0;
        status.inDischargeMode = (lsb & _BV(0)) != 0;
    };

       /**
     * @brief Check if full discharge is detected.
     * @details This flag is set and cleared based on the SOC Flag Config B option selected.
     * @return True if full discharge is detected, false otherwise.
     */
    bool isFullDischargeDetected() const { 
        return status.fullDischargeDetected; 
    }

    /**
     * @brief Check if the OCV measurement update is complete.
     * @return True if the OCV measurement update is complete, false otherwise.
     */
    bool isOcvMeasurementUpdateComplete() const {
         return status.ocvMeasurementUpdateComplete; 
    }

    /**
     * @brief Check if the OCV read failed due to current.
     * @details This bit can only be set if a battery is present after receiving an OCV_CMD().
     * @return True if the OCV read failed due to current, false otherwise.
     */
    bool isOcvReadFailedDueToCurrent() const {
         return status.ocvReadFailedDueToCurrent; 
    }

    /**
     * @brief Check if the device is in sleep mode.
     * @details This bit will be temporarily cleared during AD measurements in SLEEP mode.
     * @return True if the device is in sleep mode, false otherwise.
     */
    bool isInSleepMode() const {
         return status.inSleepMode; 
    }

    /**
     * @brief Check if over-temperature is detected during charging.
     * @details If Operation Config B [INT_OT] bit = 1, the SOC_INT pin toggles once when the [OTC] bit is set.
     * @return True if over-temperature is detected during charging, false otherwise.
     */
    bool isOverTemperatureDuringCharging() const {
         return status.overTemperatureDuringCharging; 
    }

    /**
     * @brief Check if over-temperature is detected during discharge.
     * @details If Operation Config B [INT_OT] bit = 1, the SOC_INT pin toggles once when the [OTD] bit is set.
     * @return True if over-temperature is detected during discharge, false otherwise.
     */
    bool isOverTemperatureDuringDischarge() const {
         return status.overTemperatureDuringDischarge; 
    }

    /**
     * @brief Check if full charge is detected.
     * @details This flag is set and cleared based on the SOC Flag Config A and SOC Flag Config B options selected.
     * @return True if full charge is detected, false otherwise.
     */
    bool isFullChargeDetected() const {
         return status.fullChargeDetected; 
    }

    /**
     * @brief Check if charging is inhibited.
     * @details If set, indicates that charging should not begin because the Temperature() is outside the range [Charge Inhibit Temp Low, Charge Inhibit Temp High].
     * @return True if charging is inhibited, false otherwise.
     */
    bool isChargeInhibited() const {
         return status.chargeInhibited; 
    }

    /**
     * @brief Check if the charging termination alarm is triggered.
     * @details This flag is set and cleared based on the selected SOC Flag Config A option.
     * @return True if the charging termination alarm is triggered, false otherwise.
     */
    bool isChargingTerminationAlarm() const {
         return status.chargingTerminationAlarm; 
    }

    /**
     * @brief Check if a good OCV measurement was made.
     * @return True if a good OCV measurement was made, false otherwise.
     */
    bool isGoodOcvMeasurement() const {
         return status.goodOcvMeasurement; 
    }

    /**
     * @brief Check if a battery is inserted.
     * @return True if a battery is inserted, false otherwise.
     */
    bool isBatteryInserted() const {
         return status.batteryInserted; 
    }

    /**
     * @brief Check if a battery is present.
     * @return True if a battery is present, false otherwise.
     */
    bool isBatteryPresent() const {
         return status.batteryPresent; 
    }

    /**
     * @brief Check if the discharge termination alarm is triggered.
     * @details This flag is set and cleared according to the selected SOC Flag Config A option.
     * @return True if the discharge termination alarm is triggered, false otherwise.
     */
    bool isDischargeTerminationAlarm() const {
         return status.dischargeTerminationAlarm; 
    }

    /**
     * @brief Check if system shutdown is required.
     * @details If set, the SOC_INT pin toggles once.
     * @return True if system shutdown is required, false otherwise.
     */
    bool isSystemShutdownRequired() const {
         return status.systemShutdownRequired; 
    }

    /**
     * @brief Check if the device is in discharge mode.
     * @details When set, the device is in DISCHARGE mode; when cleared, the device is in CHARGING or RELAXATION mode.
     * @return True if the device is in discharge mode, false otherwise.
     */
    bool isInDischargeMode() const {
         return status.inDischargeMode; 
    }
};

class OperationConfig {
private:
    //* OperationConfig *//
    uint16_t rawA;
    // High - byte bits
    bool temps;       // When set to 1, select an external thermistor for Temperature().
    bool reserved1;   // Reserved. Do not use.
    bool batg_pol;    // BATT_GD pin polarity. Low level is 0, high level is 1.
    bool batg_en;     // Enable BATT_GD function.
    bool reserved2;   // Reserved.
    bool sleep;       // If the working conditions allow, the fuel gauge can enter the SLEEP state. Set to true when set.
    bool slpwakechg;  // When Current > Sleep Current but not enough to trigger an event, accumulate the estimated charge from the sleep - wake state. Set to true when enabled.
    bool wrtemp;      // Write temperature. The temperature should be written by the host and used for fuel gauge monitoring. When not using an external thermistor and not using the internal temperature sensor, set it to false.

    // Low - byte bits
    bool bienable;    // When enabled, the fuel gauge can detect battery insertion. If disabled, the fuel gauge will rely on the host to use the BAT_INSERT or BAT_REMOVE command to set and clear the BatteryStatus(BATTPRES) bit. Set to true when set.
    bool reserved3;   // Reserved. Do not use.
    bool bl_pup_en;   // Battery insertion pin pull - up enable.
    int pfc_cfg;      // Pin Function Code (PFC) mode selection: PFC 0, 1, 2, or 3, selected by 00, 01, 10, or 11 respectively.
    bool wake_en;     // Wake - up enable.
    int wk_th1;       // Wake - up threshold 1.
    int wk_th0;       // Wake - up threshold 0.

    //* OperationConfigB *//
    uint16_t rawB;
    // High - byte bits
    // Reserved.
    // Reserved.
    // Reserved.
    // Reserved.
    bool defaultSeal; // Sealed during POR. 0 = No seal after POR (default setting), 1 = Seal after POR.
    bool nonRemovable; // Non - removable.
    // Reserved.
    // Reserved.

    // Low - byte bits
    bool intBrem;    // When the battery is removed and [BIEnable] = 1, GPIOUT pulses for 1ms. Enable when set.
    bool intBatL;    // Enable GPIOUT pin toggling when TDA is set.
    bool intState;   // Enable SOC_INT function to pulse the GPIOUT pin when the current direction changes.
    bool intOcv;     // Enable SOC_INT function to generate pulses based on the OCV command.
    // Reserved.
    bool intOt;      // Enable SOC_INT function to generate pulses based on over - temperature conditions and combined with BatteryStatus()[OTC or OTD].
    bool intPol;     // GPIOUT pin polarity control. Low level is 0, high level is 1.
    bool intFocv;    // If this bit is set, GPIOUT will pulse during the first measurement.

public:
    /**
     * @brief Constructor for OperationConfig class.
     * @details Parses the 16 - bit register value to initialize the member variables representing different configuration bits.
     * @param registerValue A 16 - bit unsigned integer representing the Operation Config A register value.
     */
    OperationConfig(uint32_t registerValue) {

        //* OperationConfig A *//
        rawA = (registerValue >> 16);
        // Parse high - byte bits
        temps = (rawA >> 7) & 1;
        reserved1 = (rawA >> 6) & 1;
        batg_pol = (rawA >> 5) & 1;
        batg_en = (rawA >> 4) & 1;
        reserved2 = (rawA >> 3) & 1;
        sleep = (rawA >> 2) & 1;
        slpwakechg = (rawA >> 1) & 1;
        wrtemp = rawA & 1;

        // Parse low - byte bits
        bienable = (rawA >> 15) & 1;
        reserved3 = (rawA >> 14) & 1;
        bl_pup_en = (rawA >> 13) & 1;
        pfc_cfg = (rawA >> 11) & 3;
        wake_en = (rawA >> 10) & 1;
        wk_th1 = (rawA >> 9) & 1;
        wk_th0 = (rawA >> 8) & 1;

        //* OperationConfig B *//
        rawB = (registerValue & 0xFFFF);

        // Parse high - byte bits
        defaultSeal = (rawB >> 3) & 1;
        nonRemovable = (rawB >> 2) & 1;
  
        // Parse low - byte bits
        intBrem = (rawB >> 15) & 1;
        intBatL = (rawB >> 14) & 1;
        intState = (rawB >> 13) & 1;
        intOcv = (rawB >> 12) & 1;
        intOt = (rawB >> 10) & 1;
        intPol = (rawB >> 9) & 1;
        intFocv = (rawB >> 8) & 1;
    }

    /**
     * @brief Get the value in configuration register A.
     * @return Two unsigned byte values.
     */
    uint16_t getConfigA() const {
        return rawA;
    }

    /**
     * @brief Get the value in configuration register B.
     * @return Two unsigned byte values.
     */
    uint16_t getConfigB() const {
        return rawB;
    }

    /**
     * @brief Check if the external thermistor is selected for temperature measurement.
     * @return True if the external thermistor is selected, false otherwise.
     */
    bool isTempsSet() const {
        return temps;
    }

    /**
     * @brief Check the BATT_GD pin polarity.
     * @return True if the BATT_GD pin is high - level, false if it is low - level.
     */
    bool isBatgPolHigh() const {
        return batg_pol;
    }

    /**
     * @brief Check if the BATT_GD function is enabled.
     * @return True if the BATT_GD function is enabled, false otherwise.
     */
    bool isBatgEnEnabled() const {
        return batg_en;
    }

    /**
     * @brief Check if the fuel gauge can enter the SLEEP state.
     * @return True if the fuel gauge can enter the SLEEP state, false otherwise.
     */
    bool canEnterSleep() const {
        return sleep;
    }

    /**
     * @brief Check if the slpwakechg function is enabled.
     * @return True if the slpwakechg function is enabled, false otherwise.
     */
    bool isSlpwakechgEnabled() const {
        return slpwakechg;
    }

    /**
     * @brief Check if the write temperature function is enabled.
     * @return True if the write temperature function is enabled, false otherwise.
     */
    bool isWrtempEnabled() const {
        return wrtemp;
    }

    /**
     * @brief Check if battery insertion detection is enabled.
     * @return True if battery insertion detection is enabled, false otherwise.
     */
    bool isBienableEnabled() const {
        return bienable;
    }

    /**
     * @brief Check if the battery insertion pin pull - up is enabled.
     * @return True if the battery insertion pin pull - up is enabled, false otherwise.
     */
    bool isBlPupEnEnabled() const {
        return bl_pup_en;
    }

    /**
     * @brief Get the Pin Function Code (PFC) mode.
     * @return The PFC mode (0, 1, 2, or 3).
     */
    int getPfcCfg() const {
        return pfc_cfg;
    }

    /**
     * @brief Check if the wake - up function is enabled.
     * @return True if the wake - up function is enabled, false otherwise.
     */
    bool isWakeEnEnabled() const {
        return wake_en;
    }

    /**
     * @brief Get the Wake - up threshold 1 value.
     * @return The value of Wake - up threshold 1.
     */
    int getWkTh1() const {
        return wk_th1;
    }

    /**
     * @brief Get the Wake - up threshold 0 value.
     * @return The value of Wake - up threshold 0.
     */
    int getWkTh0() const {
        return wk_th0;
    }

    //* OperationConfig B *//

    /**
     * @brief Check if the default seal option is enabled (sealed after POR).
     * @return True if enabled, false otherwise.
     */
    bool isDefaultSealEnabled() const {
        return defaultSeal;
    }

    /**
     * @brief Check if the non - removable option is set.
     * @return True if set, false otherwise.
     */
    bool isNonRemovableSet() const {
        return nonRemovable;
    }

    /**
     * @brief Check if the INT_BREM function is enabled.
     * @return True if enabled, false otherwise.
     */
    bool isIntBremEnabled() const {
        return intBrem;
    }

    /**
     * @brief Check if the INT_BATL function is enabled.
     * @return True if enabled, false otherwise.
     */
    bool isIntBatLEnabled() const {
        return intBatL;
    }

    /**
     * @brief Check if the INT_STATE function is enabled.
     * @return True if enabled, false otherwise.
     */
    bool isIntStateEnabled() const {
        return intState;
    }

    /**
     * @brief Check if the INT_OCV function is enabled.
     * @return True if enabled, false otherwise.
     */
    bool isIntOcvEnabled() const {
        return intOcv;
    }

    /**
     * @brief Check if the INT_OT function is enabled.
     * @return True if enabled, false otherwise.
     */
    bool isIntOtEnabled() const {
        return intOt;
    }

    /**
     * @brief Check if the INT_POL function is enabled (high - level polarity).
     * @return True if enabled, false otherwise.
     */
    bool isIntPolHigh() const {
        return intPol;
    }

    /**
     * @brief Check if the INT_FOCV function is enabled.
     * @return True if enabled, false otherwise.
     */
    bool isIntFocvEnabled() const {
        return intFocv;
    }
};
// *INDENT-ON*

class GaugeBQ27220 : public BQ27220Constants
{
private:
    typedef struct {
        uint16_t AtRate;                // mA        -   REG02 - 03
        uint16_t AtRateTimeToEmpty;     // Minute    -   REG04 - 05
        uint16_t Temperature;           // 0.1*K     -   REG06 - 07
        uint16_t Voltage;               // mV        -   REG08 - 09
        uint16_t BatteryStatus;         // bitmaps   -   REG0A - 0B
        int16_t  Current;               // mA        -   REG0C - 0D

        uint16_t RESERVE1;              // Rse       -   REG0E - 0F

        uint16_t RemainingCapacity;     // mAh       -   REG10 - 11
        uint16_t FullChargeCapacity;    // mAh       -   REG12 - 13

        uint16_t RESERVE2;              // Resv      -   REG14 - 15

        uint16_t TimeToEmpty;           // Minute    -   REG16 - 17
        uint16_t TimeToFull;            // Minute    -   REG18 - 19
        int16_t  StandbyCurrent;        // mA        -   REG1A - 1B
        uint16_t StandbyTimeToEmpty;    // Minute    -   REG1C - 1D
        int16_t  MaxLoadCurrent;        // mA        -   REG1E - 1F
        uint16_t MaxLoadTimeToEmpty;    // Minute    -   REG20 - 21
        uint16_t RawCoulombCount;       // Count     -   REG22 - 23
        int16_t  AveragePower;          // mW        -   REG24 - 25

        uint16_t RESERVE3;              // Resv      -   REG26 - 27

        uint16_t InternalTemperature;   // 0.1*K     -   REG28 - 29
        uint16_t CycleCount;            // Count     -   REG2A - 2B
        uint16_t StateOfCharge;         // %         -   REG2C - 2D
        uint16_t StateOfHealth;         // %         -   REG2E - 2F
        uint16_t ChargingVoltage;       // mV        -   REG30 - 31  - 65535 request max voltage
        uint16_t ChargingCurrent;       // mA        -   REG32 - 33  - 65535 request max current
        uint16_t BTPDischargeSet;       // Control   -   REG34 - 35
        uint16_t BTPChargeSet;          // Control   -   REG36 - 37

        uint16_t OperationStatus;       // Status    -   REG3A - 3B
        uint16_t DesignCapacity;        // mAh       -   REG3C - 3D
    } BatteryData;

public:

    enum Access {
        FULL_ACCESS = 1,
        UNBLOCK_ACCESS,
        SEALED_ACCESS,
    };

    GaugeBQ27220() : comm(nullptr), hal(nullptr), accessKey(0xFFFFFFFF) {}

    ~GaugeBQ27220()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, BQ27220_SLAVE_ADDRESS, sda, scl)) {
            return false;
        }
        return initImpl();
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, BQ27220_SLAVE_ADDRESS, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, BQ27220_SLAVE_ADDRESS)) {
            return false;
        }
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, BQ27220_SLAVE_ADDRESS, comm, hal)) {
            return false;
        }
        return initImpl();
    }


    /**
     * @brief Refresh the battery data by reading from the registers.
     * @details This function attempts to read battery data from the specified registers of the fuel gauge.
     *          If any of the read operations fail, the function returns false; otherwise, it returns true.
     * @return true if all read operations are successful and the data is refreshed, false otherwise.
     */
    bool refresh()
    {
        uint8_t buffer[REGISTER_COUNT];
        if (comm->readRegister(START_REGISTER, buffer, REGISTER_COUNT) < 0) {
            return false;
        }
        uint8_t *ptr = (uint8_t *)&data;
        for (int i = 0; i < REGISTER_COUNT; i += 2) {
            uint16_t value = (buffer[i + 1] << 8) | buffer[i];
            *(uint16_t *)ptr = value;
            ptr += 2;
        }

        if (comm->readRegister(0x3A, buffer, 4) < 0) {
            return false;
        }
        data.OperationStatus =  (buffer[1] << 8) | buffer[0];
        data.DesignCapacity =  (buffer[3] << 8) | buffer[2];
        return true;
    }

    // *INDENT-OFF*
    /**
     * @brief Get the AtRate value.
     * @details The AtRate represents the current rate at which the battery is being discharged or charged.
     *          The value is in milliamperes (mA).
     * @return The AtRate value in mA.
     */
    uint16_t getAtRate() const { return data.AtRate; }

    /**
     * @brief Get the AtRateTimeToEmpty value.
     * @details The AtRateTimeToEmpty indicates the estimated time remaining until the battery is empty
     *          at the current discharge rate. The unit is minutes.
     * @return The AtRateTimeToEmpty value in minutes.
     */
    uint16_t getAtRateTimeToEmpty() const { return data.AtRateTimeToEmpty; }

    /**
     * @brief Get the temperature value.
     * @details This function returns the temperature measured by the fuel gauge in Celsius.
     *          The raw data is stored in tenths of Kelvin and is converted to Celsius in this function.
     * @return The temperature value in Celsius.
     */
    float    getTemperature() const { return (data.Temperature * 0.1f) - 273.15f; }

    /**
     * @brief Get the battery voltage value.
     * @details This value represents the measured battery pack voltage in millivolts (mV),
     *          ranging from 0 to 6000mV.
     * @return The battery voltage value in mV.
     */
    uint16_t getVoltage() const { return data.Voltage; }

    /**
     * @brief Get the battery status.
     * @details The Read Word function returns the contents of the Fuel Gauge Status Register,
     *          describing the current battery status.
     * @return The battery status.
     */
    BatteryStatus getBatteryStatus() const { return BatteryStatus(data.BatteryStatus); }

    /**
     * @brief Get the instantaneous current value.
     * @details The instantaneous current in the sense resistor. It is updated once per second.
     *          A negative value indicates a discharge current. The unit is milliamperes (mA).
     * @return The instantaneous current value in mA.
     */
    int16_t  getCurrent() const { return data.Current; }

    /**
     * @brief Get the remaining battery capacity.
     * @details This function returns the remaining capacity of the battery in milliampere - hours (mAh).
     * @return The remaining battery capacity in mAh.
     */
    uint16_t getRemainingCapacity() const { return data.RemainingCapacity; }

    /**
     * @brief Get the full charge capacity of the battery.
     * @details This function returns the full charge capacity of the battery in milliampere - hours (mAh).
     * @return The full charge capacity of the battery in mAh.
     */
    uint16_t getFullChargeCapacity() const { return data.FullChargeCapacity; }

    /**
     * @brief Get the estimated time until the battery is empty.
     * @details This function returns the estimated time remaining until the battery is empty
     *          under normal operating conditions. The unit is minutes.
     * @return The estimated time to empty in minutes.
     */
    uint16_t getTimeToEmpty() const { return data.TimeToEmpty; }

    /**
     * @brief Get the estimated time until the battery is fully charged.
     * @details This function returns the estimated time remaining until the battery is fully charged.
     *          The unit is minutes.
     * @return The estimated time to full in minutes.
     */
    uint16_t getTimeToFull() const { return data.TimeToFull; }

    /**
     * @brief Get the standby current value.
     * @details The standby current measured by the sense resistor. The unit is milliamperes (mA).
     * @return The standby current value in mA.
     */
    int16_t  getStandbyCurrent() const { return data.StandbyCurrent; }

    /**
     * @brief Get the estimated time until the battery is empty at standby discharge rate.
     * @details This function returns the predicted remaining battery life in minutes at the standby discharge rate.
     * @return The estimated time to empty at standby discharge rate in minutes.
     */
    uint16_t getStandbyTimeToEmpty() const { return data.StandbyTimeToEmpty; }

    /**
     * @brief Get the maximum load current value.
     * @details This function returns the signed integer value in milliamperes (mA) at the maximum load.
     * @return The maximum load current value in mA.
     */
    int16_t  getMaxLoadCurrent() const { return data.MaxLoadCurrent; }

    /**
     * @brief Get the estimated time until the battery is empty at maximum load current discharge rate.
     * @details This function returns the predicted remaining battery life in minutes at the maximum load current discharge rate.
     * @return The estimated time to empty at maximum load current discharge rate in minutes.
     */
    uint16_t getMaxLoadTimeToEmpty() const { return data.MaxLoadTimeToEmpty; }

    /**
     * @brief Get the raw coulomb count value.
     * @details The amount of coulombs transferred from a battery during charge/discharge.
     * @return The raw coulomb count value.
     */
    uint16_t getRawCoulombCount() const { return data.RawCoulombCount; }

    /**
     * @brief Get the average power value.
     * @details The average power during battery charging and discharging.
     *          Values are negative during discharging and positive during charging.
     *          A value of 0 means the battery is not discharging. The value is reported in milliwatts (mW).
     * @return The average power value in mW.
     */
    int16_t  getAveragePower() const { return data.AveragePower; }

    /**
     * @brief Get the internal temperature value.
     * @details This read - only function returns the internal temperature sensor value measured by the fuel gauge in Celsius.
     *          The raw data is stored in tenths of Kelvin and is converted to Celsius in this function.
     * @return The internal temperature value in Celsius.
     */
    float    getInternalTemperature() const { return (data.InternalTemperature * 0.1f) - 273.15f; }

    /**
     * @brief Get the cycle count value.
     * @details The number of cycles the active battery has experienced, ranging from 0 to 65535.
     *          A cycle occurs when the accumulated discharge ≥ the cycle threshold.
     * @return The cycle count value.
     */
    uint16_t getCycleCount() const { return data.CycleCount; }

    /**
     * @brief Get the state of charge value.
     * @details This read - only function returns an unsigned integer value representing the predicted
     *          remaining battery capacity as a percentage of the full charge capacity, in the range 0 to 100%.
     * @return The state of charge value in percentage.
     */
    uint16_t getStateOfCharge() const { return data.StateOfCharge; }

    /**
     * @brief Get the state of health value.
     * @details This read - only function returns an unsigned integer value representing the percentage of
     *          the full charge capacity to the design capacity, ranging from 0 to 100%.
     * @return The state of health value in percentage.
     */
    uint16_t getStateOfHealth() const { return data.StateOfHealth; }

    /**
     * @brief Get the requested charging voltage value.
     * @details This read - only function returns the unsigned integer value of the desired battery
     *          charging voltage. A value of 65,535 indicates that the battery is requesting the
     *          maximum voltage from the battery charger.
     * @return The requested charging voltage value in mV.
     */
    uint16_t getRequestChargingVoltage() const { return data.ChargingVoltage; }

    /**
     * @brief Get the requested charging current value.
     * @details This read - only function returns an unsigned integer value for the desired battery
     *          charging current. A value of 65,535 indicates that the battery is requesting the maximum
     *          current from the battery charger.
     * @return The requested charging current value in mA.
     */
    uint16_t getRequestChargingCurrent() const { return data.ChargingCurrent; }

    /**
     * @brief Get the BTP discharge set value.
     * @details This read/write word command updates the BTP setup threshold that triggers the BTP
     *          interrupt in the discharge direction and sets the OperationStatus()[BTPINT] bit.
     * @return The BTP discharge set value.
     */
    uint16_t getBTPDischargeSet() const { return data.BTPDischargeSet; }

    /**
     * @brief Get the BTP charge set value.
     * @details This read/write word command updates the BTP setup threshold that triggers the BTP
     *          interrupt in the charge direction and sets the OperationStatus()[BTPINT] bit.
     * @return The BTP charge set value.
     */
    uint16_t getBTPChargeSet() const { return data.BTPChargeSet; }

    /**
     * @brief Get the operation status.
     * @details The Read Word function returns the contents of the internal status register.
     *          See datasheet page 27.
     * @return The operation status.
     */
    FuelGaugeOperationStatus getOperationStatus() const { return FuelGaugeOperationStatus(data.OperationStatus); }

    /**
     * @brief Get the design capacity value.
     * @details This read - only function returns the value stored in the design capacity in milliampere - hours (mAh).
     *          This value is used as the theoretical or nominal capacity of a new battery pack
     *          and is used to calculate the state of health.
     * @return The design capacity value in mAh.
     */
    uint16_t getDesignCapacity() const { return data.DesignCapacity; }

    // *INDENT-ON*

    /**
    * @brief Set the new design capacity and full charge capacity of the battery.
    *
    * This function is responsible for updating the design capacity and full charge capacity of the battery.
    * It first checks the device's access settings, enters the configuration update mode, writes the new capacity values
    * and checksums, and finally exits the configuration update mode. If the device was previously in a sealed state,
    * it will be restored to the sealed mode after the operation is completed.
    * For new devices, use the default key for access. If it is an encrypted device, use setAccessKey(uint32_t key) to set the key.
    * @param newDesignCapacity The new design capacity to be set, of type uint16_t.
    * @param newFullChargeCapacity The new full charge capacity to be set, of type uint16_t.
    * @return bool Returns true if the setting is successful, false otherwise.
    */
    bool setNewCapacity(uint16_t newDesignCapacity, uint16_t newFullChargeCapacity)
    {
        return performConfigUpdate<bool>([this, newDesignCapacity, newFullChargeCapacity]() {
            // Set the design capacity
            if (!setCapacity(newDesignCapacity, lowByte(BQ27220_ROM_DESIGN_CAPACITY),
                             highByte(BQ27220_ROM_DESIGN_CAPACITY))) {
                log_e("Failed to set design capacity!");
                return false;
            }
            hal->delay(10);

            // Set full charge capacity
            if (!setCapacity(newFullChargeCapacity, lowByte(BQ27220_ROM_FULL_CHARGE_CAPACITY),
                             highByte(BQ27220_ROM_FULL_CHARGE_CAPACITY))) {
                log_e("Failed to set full charge capacity!");
                return false;
            }
            hal->delay(10);

            return true;
        });
    }

    /**
     * @brief Retrieve the operation configuration of the device.
     *
     * This function is responsible for obtaining the current operation configuration of the device. 
     * The operation configuration typically includes various settings and parameters that govern 
     * the device's behavior, such as operating modes, thresholds, and enable/disable flags.
     * 
     * @return An `OperationConfig` object that contains the current operation configuration of the device.
     */
    OperationConfig getOperationConfig()
    {
        return performConfigUpdate<OperationConfig>([this]() {
            comm->writeRegister(BQ27220_REG_ROM_START, lowByte(BQ27220_ROM_OPERATION_CONFIG_A));
            hal->delay(10);
            comm->writeRegister(BQ27220_REG_ROM_START + 1, highByte(BQ27220_ROM_OPERATION_CONFIG_A));
            hal->delay(10);
            uint8_t buffer[4];
            comm->readRegister(0x40, buffer, 4);
            uint16_t bitmapsA = (buffer[0] << 8) | buffer[1];
            uint16_t bitmapsB = (buffer[2] << 8) | buffer[3];
            OperationConfig value((bitmapsA << 16) | bitmapsB);
            return value;
        });
    }

    /**
     * @brief Retrieve the unique identifier of the chip.
     * @return A 16 - bit unsigned integer representing the chip's unique identifier. 
     */
    uint16_t getChipID()
    {
        uint8_t buffer[2] = {0};
        if (sendSubCommand(BQ27220_SUB_CMD_DEVICE_NUMBER, true) < 0) {
            return 0;
        }
        if (this->getMACData(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        return static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
    }

    /**
     * @brief Get the hardware version of the device.
     * @details This function attempts to retrieve the hardware version by writing a specific value
     *          to the hardware version sub - command register and then reading the MAC data.
     *          If any step fails during the process, it returns -1 to indicate an error.
     *          Otherwise, it combines the two bytes read from the MAC data into a 16 - bit unsigned integer
     *          and returns it as the hardware version.
     * @return The hardware version as a 16 - bit unsigned integer if successful, -1 otherwise.
     */
    int getHardwareVersion()
    {
        uint8_t buffer[2];
        constexpr uint8_t value = 0x00;
        comm->writeRegister(BQ27220_SUB_CMD_HW_VERSION, value);
        if (this->getMACData(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        return static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
    }


    /**
     * @brief Get the firmware version of the device.
     * @details This function tries to obtain the firmware version by sending a specific sub - command
     *          to request the firmware version. If the sub - command sending fails or the subsequent
     *          reading of the MAC data fails, it returns -1. Otherwise, it combines the four bytes
     *          read from the MAC data into a 32 - bit unsigned integer and returns it as the firmware version.
     * @return The firmware version as a 32 - bit unsigned integer if successful, -1 otherwise.
     */
    int getFirmwareVersion()
    {
        uint8_t buffer[4];
        if (sendSubCommand(BQ27220_SUB_CMD_FW_VERSION, true) < 0) {
            return -1;
        }
        if (this->getMACData(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        return  (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
    }

    /**
     * @brief Reset the device.
     * @details This function sends a reset sub - command to the device to perform a reset operation.
     */
    void reset()
    {
        sendSubCommand(BQ27220_SUB_CMD_RESET);
    }

    /**
     * @brief Set the access key for the device.
     * @details If this function is not called, the default access key is 0xFFFFFFFF.
     *          This function allows the user to set a custom access key.
     * @param key The 32 - bit unsigned integer representing the access key to be set.
     */
    void setAccessKey(uint32_t key)
    {
        accessKey = key;
    }

private:
    FuelGaugeOperationStatus getOperationStatusNow()
    {
        return FuelGaugeOperationStatus(getHalfWord(BQ27220_REG_STA_OPERATION_STATUS));
    }

    // This read-write block will return the result data for the currently active subcommand.
    int getMACData(uint8_t *buffer, uint8_t request_len)
    {
        const uint8_t max_size = BQ27220_REG_MAC_BUFFER_END - BQ27220_REG_MAC_BUFFER_START + 1;
        if (request_len > max_size) {
            return -1;
        }
        if (!buffer) {
            return -1;
        }
        uint8_t reg = BQ27220_REG_MAC_BUFFER_START;
        return comm->writeThenRead(&reg, 1, buffer, request_len);
    }

    // This read and write function returns the checksum of the current subcommand and data block.
    // Writing to this register provides the checksum required to execute a subcommand that requires data.
    uint16_t getMACDataSum()
    {
        uint8_t sum = 0x00;
        if (comm->readRegister(BQ27220_REG_MAC_DATA_SUM, &sum, 1) < 0) {
            return 0;
        }
        return sum;
    }

    // This read and write function returns the number of MACData()
    // bytes contained in MACDataSum() as part of the response.
    uint16_t getMACDataLen()
    {
        uint8_t length = 0x00;
        if (comm->readRegister(BQ27220_REG_MAC_DATA_LEN, &length, 1) < 0) {
            return 0;
        }
        return length;
    }

    // Subcommands
    int sendSubCommand(uint16_t subCmd, bool waitConfirm = false)
    {
        uint8_t buffer[3];
        buffer[0] = 0x00;
        buffer[1] = lowByte(subCmd);
        buffer[2] = highByte(subCmd);
        if (comm->writeBuffer(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        if (!waitConfirm) {
            hal->delay(10);
            return 0;
        }
        constexpr uint8_t statusReg = 0x00;
        int waitCount = 20;
        hal->delay(10);
        while (waitCount--) {
            comm->writeThenRead(&statusReg, 1, buffer, 2);
            uint16_t *value = reinterpret_cast<uint16_t *>(buffer);
            if (*value == 0xFFA5) {
                return 0;
            }
            hal->delay(100);
        }
        log_e("Subcommand failed!");
        return -1;
    }

    // Unlock Safe Mode
    int unsealDevice()
    {
        uint8_t cmd1[] = {0x00, 0x14, 0x04};
        if (comm->writeBuffer(cmd1, arraySize(cmd1)) < 0) {
            return -1;
        }
        hal->delay(10);
        uint8_t cmd2[] = {0x00, 0x72, 0x36};
        if (comm->writeBuffer(cmd2, arraySize(cmd2)) < 0) {
            return -1;
        }
        hal->delay(10);
        return 0;
    }

    // Full access key, 0xFFFFFFFF if not set
    int unsealFullAccess()
    {
        uint8_t buffer[3];
        buffer[0] = 0x00;
        buffer[1] = lowByte((accessKey >> 24));
        buffer[2] = lowByte((accessKey >> 16));
        if (comm->writeBuffer(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        hal->delay(10);
        buffer[1] = lowByte((accessKey >> 8));
        buffer[2] = lowByte((accessKey));
        if (comm->writeBuffer(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        hal->delay(10);
        return 0;
    }

    int exitSealMode()
    {
        return  sendSubCommand(BQ27220_SUB_CMD_SEALED);
    }

    template<typename ResultType, typename Func>
    ResultType performConfigUpdate(Func specificOperation)
    {
        bool isSealed = false;

        // Check access settings
        FuelGaugeOperationStatus status = getOperationStatusNow();
        uint8_t currAccesslevel = status.getSecurityAccessLevel();
        if (currAccesslevel == SEALED_ACCESS) {
            isSealed = true;
            unsealDevice();
        }
        if (currAccesslevel != FULL_ACCESS) {
            unsealFullAccess();
        }

        // Send ENTER_CFG_UPDATE command (0x0090)
        sendSubCommand(BQ27220_SUB_CMD_ENTER_CFG_UPDATE);

        // Confirm CFUPDATE mode by polling the OperationStatus() register until Bit 2 is set.
        bool isConfigUpdate = false;
        uint32_t timeout = hal->millis() + 1500UL;
        while (timeout > hal->millis()) {
            status = getOperationStatusNow();
            if (status.getIsConfigUpdateMode()) {
                isConfigUpdate = true;
                break;
            }
            hal->delay(100);
        }
        if (!isConfigUpdate) {
            log_e("The update mode has timed out. It may also be that the access key for full permissions is invalid!");
            if (std::is_same<ResultType, bool>::value) {
                return false;
            } else if (std::is_same<ResultType, OperationConfig>::value) {
                return { 0x00 };
            }
        }

        ResultType result = specificOperation();

        // Exit CFUPDATE mode by sending the EXIT_CFG_UPDATE_REINIT (0x0091) or EXIT_CFG_UPDATE (0x0092) command
        sendSubCommand(BQ27220_SUB_CMD_EXIT_CFG_UPDATE_REINIT);
        hal->delay(10);

        // Confirm that CFUPDATE mode has been exited by polling the OperationStatus() register until bit 2 is cleared
        timeout = hal->millis() + 3000UL;
        while (timeout > hal->millis()) {
            status = getOperationStatusNow();
            if (!status.getIsConfigUpdateMode()) {
                timeout = hal->millis() + 1000;
                break;
            }
            hal->delay(100);
        }
        if (hal->millis() > timeout) {
            log_e("Timed out waiting to exit update mode.");
            if (std::is_same<ResultType, bool>::value) {
                return false;
            } else if (std::is_same<ResultType, OperationConfig>::value) {
                return { 0x00 };
            }
        }

        // If the device was previously in SEALED state, return to SEALED mode by sending the Control(0x0030) subcommand
        if (isSealed) {
            log_d("Restore Safe Mode!");
            exitSealMode();
        }

        return result;
    }


    bool setCapacity(uint16_t newCapacity, uint8_t msbAccessValue, uint8_t lsbAccessValue)
    {
        constexpr uint8_t fixedDataLength = 0x06;

        // Write to access the MSB of Capacity
        comm->writeRegister(BQ27220_REG_ROM_START, msbAccessValue);
        hal->delay(10);

        // Write to access the LSB of Capacity
        comm->writeRegister(BQ27220_REG_ROM_START + 1, lsbAccessValue);
        hal->delay(10);

        // Write two Capacity bytes starting from 0x40
        uint8_t newCapacityMsb = highByte(newCapacity);
        uint8_t newCapacityLsb = lowByte(newCapacity);
        uint8_t capacityRaw[] = {newCapacityMsb, newCapacityLsb};
        comm->writeRegister(BQ27220_REG_MAC_BUFFER_START, capacityRaw, 2);

        // Calculate new checksum
        uint8_t newChksum = 0xFF - ((msbAccessValue + lsbAccessValue + newCapacityMsb + newCapacityLsb) & 0xFF);

        // Write new checksum (0x60)
        comm->writeRegister(BQ27220_REG_MAC_DATA_SUM, newChksum);

        // Write the block length
        comm->writeRegister(BQ27220_REG_MAC_DATA_LEN, fixedDataLength);

        return true;
    }

    int getHalfWord(uint8_t reg)
    {
        uint8_t buffer[2] = {0};
        if (comm->writeThenRead(&reg, 1, buffer, arraySize(buffer)) < 0) {
            log_e("Read register %02X failed!", reg);
            return UINT16_MAX;
        }
        return (buffer[1] << 8) | buffer[0];
    }

    bool initImpl()
    {
        int chipID = getChipID();
        if (chipID != BQ27220_CHIP_ID) {
            log_e("Chip id not match : %02X\n", chipID);
            return false;
        }

        int sw = getFirmwareVersion();
        if (sw < 0) {
            log_e("Software version error!");
        } else {
            log_d("Software version 0x%04X", sw);
        }
        hal->delay(100);
        int hw = getHardwareVersion();
        if (hw < 0) {
            log_e("Hardware version error!");
        } else {
            log_d("Hardware version 0x%04X", hw);
        }

        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    uint32_t accessKey;
    BatteryData data;
    const uint8_t START_REGISTER  = 0x02;
    const uint8_t REGISTER_COUNT  = 54;
};

