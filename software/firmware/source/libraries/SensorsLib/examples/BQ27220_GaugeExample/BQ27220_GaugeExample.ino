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
 * @file      BQ27220_GaugeExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-13
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <GaugeBQ27220.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  2
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  3
#endif

GaugeBQ27220 gauge;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    if (!gauge.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to BQ27220 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("Init BQ27220 Sensor success!");



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

    // uint32_t key = 0x12345678;
    // gauge.setAccessKey(key)

    uint16_t newDesignCapacity = 3500;
    uint16_t newFullChargeCapacity = 3500;
    gauge.setNewCapacity(newDesignCapacity, newFullChargeCapacity);

    OperationConfig config = gauge.getOperationConfig();

    //* OperationConfig A *//
    Serial.print("OperationConfigA Values:0x");
    Serial.println(config.getConfigA(), HEX);

    Serial.print("External Thermistor Selected: ");
    Serial.println(config.isTempsSet() ? "YES" : "NO");

    Serial.print("BATT_GD Pin Polarity High: ");
    Serial.println(config.isBatgPolHigh() ? "YES" : "NO");

    Serial.print("BATT_GD Function Enabled: ");
    Serial.println(config.isBatgEnEnabled() ? "YES" : "NO");

    Serial.print("Can Enter SLEEP State: ");
    Serial.println(config.canEnterSleep() ? "YES" : "NO");

    Serial.print("slpwakechg Function Enabled: ");
    Serial.println(config.isSlpwakechgEnabled() ? "YES" : "NO");

    Serial.print("Write Temperature Function Enabled: ");
    Serial.println(config.isWrtempEnabled() ? "YES" : "NO");

    Serial.print("Battery Insertion Detection Enabled: ");
    Serial.println(config.isBienableEnabled() ? "YES" : "NO");

    Serial.print("Battery Insertion Pin Pull - Up Enabled: ");
    Serial.println(config.isBlPupEnEnabled() ? "YES" : "NO");

    Serial.print("Pin Function Code (PFC) Mode: ");
    Serial.println(config.getPfcCfg());

    Serial.print("Wake - Up Function Enabled: ");
    Serial.println(config.isWakeEnEnabled() ? "YES" : "NO");

    Serial.print("Wake - Up Threshold 1: ");
    Serial.println(config.getWkTh1());

    Serial.print("Wake - Up Threshold 0: ");
    Serial.println(config.getWkTh0());

    //* OperationConfig B *//
    Serial.print("\nOperationConfigB Values:0x");
    Serial.println(config.getConfigB(), HEX);

    Serial.print("Default Seal Option Enabled: ");
    Serial.println(config.isDefaultSealEnabled() ? "YES" : "NO");

    Serial.print("Non - Removable Option Set: ");
    Serial.println(config.isNonRemovableSet() ? "YES" : "NO");

    Serial.print("INT_BREM Function Enabled: ");
    Serial.println(config.isIntBremEnabled() ? "YES" : "NO");

    Serial.print("INT_BATL Function Enabled: ");
    Serial.println(config.isIntBatLEnabled() ? "YES" : "NO");

    Serial.print("INT_STATE Function Enabled: ");
    Serial.println(config.isIntStateEnabled() ? "YES" : "NO");

    Serial.print("INT_OCV Function Enabled: ");
    Serial.println(config.isIntOcvEnabled() ? "YES" : "NO");

    Serial.print("INT_OT Function Enabled: ");
    Serial.println(config.isIntOtEnabled() ? "YES" : "NO");

    Serial.print("INT_POL Function Enabled (High - Level Polarity): ");
    Serial.println(config.isIntPolHigh() ? "YES" : "NO");

    Serial.print("INT_FOCV Function Enabled: ");
    Serial.println(config.isIntFocvEnabled() ? "YES" : "NO");

    delay(10000);
}


void loop()
{
    uint32_t startMeasTime = millis();

    if (gauge.refresh()) {

        uint32_t endMesTime = millis();

        Serial.print("Polling time: "); Serial.print(endMesTime - startMeasTime); Serial.println(" ms");

        Serial.println("\nStandard query:");
        Serial.print("\t- AtRate:"); Serial.print(gauge.getAtRate()); Serial.println(" mA");
        Serial.print("\t- AtRateTimeToEmpty:"); Serial.print(gauge.getAtRateTimeToEmpty()); Serial.println(" minutes");
        Serial.print("\t- Temperature:"); Serial.print(gauge.getTemperature() ); Serial.println(" ℃");
        Serial.print("\t- BatteryVoltage:"); Serial.print(gauge.getVoltage()); Serial.println(" mV");
        Serial.print("\t- InstantaneousCurrent:"); Serial.print(gauge.getCurrent()); Serial.println(" mAh");
        Serial.print("\t- RemainingCapacity:"); Serial.print(gauge.getRemainingCapacity()); Serial.println(" mAh");
        Serial.print("\t- FullChargeCapacity:"); Serial.print(gauge.getFullChargeCapacity()); Serial.println(" mAh");
        Serial.print("\t- DesignCapacity:"); Serial.print(gauge.getDesignCapacity()); Serial.println(" mAh");
        Serial.print("\t- TimeToEmpty:"); Serial.print(gauge.getTimeToEmpty()); Serial.println(" minutes");
        Serial.print("\t- TimeToFull:"); Serial.print(gauge.getTimeToFull()); Serial.println(" minutes");
        Serial.print("\t- StandbyCurrent:"); Serial.print(gauge.getStandbyCurrent()); Serial.println(" mA");
        Serial.print("\t- StandbyTimeToEmpty:"); Serial.print(gauge.getStandbyTimeToEmpty()); Serial.println(" minutes");
        Serial.print("\t- MaxLoadCurrent:"); Serial.print(gauge.getMaxLoadCurrent()); Serial.println(" mA");
        Serial.print("\t- MaxLoadTimeToEmpty:"); Serial.print(gauge.getMaxLoadTimeToEmpty()); Serial.println(" minute");
        Serial.print("\t- RawCoulombCount:"); Serial.print(gauge.getRawCoulombCount()); Serial.println(" mAh");
        Serial.print("\t- AveragePower:"); Serial.print(gauge.getAveragePower()); Serial.println(" mW");
        Serial.print("\t- InternalTemperature:"); Serial.print(gauge.getInternalTemperature()); Serial.println(" ℃");
        Serial.print("\t- CycleCount:"); Serial.println(gauge.getCycleCount());
        Serial.print("\t- StateOfCharge:"); Serial.print(gauge.getStateOfCharge()); Serial.println(" %");
        Serial.print("\t- StateOfHealth:"); Serial.print(gauge.getStateOfHealth()); Serial.println(" %");
        Serial.print("\t- RequestChargingVoltage:"); Serial.print(gauge.getRequestChargingVoltage()); Serial.println(" mV");
        Serial.print("\t- RequestChargingCurrent:"); Serial.print(gauge.getRequestChargingCurrent()); Serial.println(" mA");
        Serial.print("\t- BTPDischargeSet:"); Serial.print(gauge.getBTPDischargeSet()); Serial.println(" mAh");
        Serial.print("\t- BTPChargeSet:"); Serial.print(gauge.getBTPChargeSet()); Serial.println(" mAh");
        FuelGaugeOperationStatus status = gauge.getOperationStatus();
        BatteryStatus batteryStatus = gauge.getBatteryStatus();

        Serial.println("\nOperation Status:");
        Serial.print("\t- getIsConfigUpdateMode:"); Serial.println(status.getIsConfigUpdateMode() ? "YES" : "NO");
        Serial.print("\t- getIsBtpThresholdExceeded:"); Serial.println(status.getIsBtpThresholdExceeded() ? "YES" : "NO");
        Serial.print("\t- getIsCapacityAccumulationThrottled:"); Serial.println(status.getIsCapacityAccumulationThrottled() ? "YES" : "NO");
        Serial.print("\t- getIsInitializationComplete:"); Serial.println(status.getIsInitializationComplete() ? "YES" : "NO");
        Serial.print("\t- getIsDischargeCycleCompliant:"); Serial.println(status.getIsDischargeCycleCompliant() ? "YES" : "NO");
        Serial.print("\t- getIsBatteryVoltageBelowEdv2:"); Serial.println(status.getIsBatteryVoltageBelowEdv2() ? "YES" : "NO");
        Serial.print("\t- getSecurityAccessLevel:"); Serial.println(status.getSecurityAccessLevel());
        Serial.print("\t- getIsCalibrationModeEnabled:"); Serial.println(status.getIsCalibrationModeEnabled() ? "YES" : "NO");

        Serial.println("\nBattery Status:");
        if (batteryStatus.isFullDischargeDetected()) {
            Serial.println("\t- Full discharge detected.");
        }
        if (batteryStatus.isOcvMeasurementUpdateComplete()) {
            Serial.println("\t- OCV measurement update is complete.");
        }
        if (batteryStatus.isOcvReadFailedDueToCurrent()) {
            Serial.println("\t- Status bit indicating that an OCV read failed due to current.");
            Serial.println("\tThis bit can only be set if a battery is present after receiving an OCV_CMD().");
        }
        if (batteryStatus.isInSleepMode()) {
            Serial.println("\t- The device operates in SLEEP mode");
        }
        if (batteryStatus.isOverTemperatureDuringCharging()) {
            Serial.println("\t- Over-temperature is detected during charging.");
        }
        if (batteryStatus.isOverTemperatureDuringDischarge()) {
            Serial.println("\t- Over-temperature detected during discharge condition.");
        }
        if (batteryStatus.isFullChargeDetected()) {
            Serial.println("\t- Full charge detected.");
        }
        if (batteryStatus.isChargeInhibited()) {
            Serial.println("\t- Charge Inhibit: If set, indicates that charging should not begin because the Temperature() is outside the range");
            Serial.println("\t[Charge Inhibit Temp Low, Charge Inhibit Temp High]. ");
        }
        if (batteryStatus.isChargingTerminationAlarm()) {
            Serial.println("\t- Termination of charging alarm. This flag is set and cleared based on the selected SOC Flag Config A option.");
        }
        if (batteryStatus.isGoodOcvMeasurement()) {
            Serial.println("\t- A good OCV measurement was made.");
        }
        if (batteryStatus.isBatteryInserted()) {
            Serial.println("\t- Detects inserted battery.");
        }
        if (batteryStatus.isBatteryPresent()) {
            Serial.println("\t- Battery presence detected.");
        }
        if (batteryStatus.isDischargeTerminationAlarm()) {
            Serial.println("\t- Termination discharge alarm. This flag is set and cleared according to the selected SOC Flag Config A option.");
        }
        if (batteryStatus.isSystemShutdownRequired()) {
            Serial.println("\t- System shutdown bit indicating that the system should be shut down. True when set. If set, the SOC_INT pin toggles once.");
        }
        if (batteryStatus.isInDischargeMode()) {
            Serial.println("\t- When set, the device is in DISCHARGE mode; when cleared, the device is in CHARGING or RELAXATION mode.");
        }
        Serial.println("===============================================");

    }
    delay(3000);
}



