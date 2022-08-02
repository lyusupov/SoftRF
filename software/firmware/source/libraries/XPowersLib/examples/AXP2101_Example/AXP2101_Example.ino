/*
MIT License

Copyright (c) 2022 lewis he

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
! WARN:
Please do not run the example without knowing the external load voltage of the PMU,
it may burn your external load, please check the voltage setting before running the example,
if there is any loss, please bear it by yourself
*/
#error "Running this example is known to not damage the device! Please go and uncomment this!"

// Defined using AXP2102
#define XPOWERS_CHIP_AXP2102

#include <Wire.h>
#include <Arduino.h>
#include "XPowersLib.h"


bool  pmu_flag = 0;
XPowersPMU PMU;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t pmu_irq_pin = 35;

void setFlag(void)
{
    pmu_flag = true;
}


void setup()
{
    Serial.begin(115200);

    bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, i2c_sda, i2c_scl);

    if (result == false) {
        Serial.println("PMU is not online..."); while (1)delay(50);
    }

    Serial.printf("getID:0x%x\n", PMU.getChipID());

    // Set the minimum system operating voltage inside the PMU,
    // below this value will shut down the PMU
    PMU.setMinSystemVoltage(XPOWERS_VSYS_VOL_4V5);

    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    PMU.setVbusVoltageLimit(XPOWERS_VBUS_VOL_LIM_4V36);

    // Set the maximum current of the PMU VBUS input,
    // higher than this value will turn off the PMU
    PMU.setVbusCurrentLimit(XPOWERS_VBUS_CUR_LIM_1500MA);

    // DC1 IMAX=2A
    // 1500~3400mV,100mV/step,20steps
    PMU.setDC1Voltage(3300);
    Serial.printf("DC1  :%s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "ENABLE" : "DISABLE", PMU.getDC1Voltage());

    // DC2 IMAX=2A
    // 500~1200mV  10mV/step,71steps
    // 1220~1540mV 20mV/step,17steps
    PMU.setDC2Voltage(1000);
    Serial.printf("DC2  :%s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "ENABLE" : "DISABLE", PMU.getDC2Voltage());

    // DC3 IMAX = 2A
    // 500~1200mV,10mV/step,71steps
    // 1220~1540mV,20mV/step,17steps
    // 1600~3400mV,100mV/step,19steps
    PMU.setDC3Voltage(3300);
    Serial.printf("DC3  :%s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", PMU.getDC3Voltage());

    // DCDC4 IMAX=1.5A
    // 500~1200mV,10mV/step,71steps
    // 1220~1840mV,20mV/step,32steps
    PMU.setDC4Voltage(1000);
    Serial.printf("DC4  :%s   Voltage:%u mV \n",  PMU.isEnableDC4()  ? "ENABLE" : "DISABLE", PMU.getDC4Voltage());

    // DC5 IMAX=2A
    // 1200mV
    // 1400~3700mV,100mV/step,24steps
    PMU.setDC5Voltage(3300);
    Serial.printf("DC5  :%s   Voltage:%u mV \n",  PMU.isEnableDC5()  ? "ENABLE" : "DISABLE", PMU.getDC5Voltage());

    //ALDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO1Voltage(3300);

    //ALDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO2Voltage(3300);

    //ALDO3 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO3Voltage(3300);

    //ALDO4 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO4Voltage(3300);

    //BLDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setBLDO1Voltage(3300);

    //BLDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setBLDO2Voltage(3300);

    //CPUSLDO IMAX=30mA
    //500~1400mV,50mV/step,19steps
    PMU.setCPUSLDOVoltage(1000);

    //DLDO1 IMAX=300mA
    //500~3400mV, 100mV/step,29steps
    PMU.setDLDO1Voltage(3300);

    //DLDO2 IMAX=300mA
    //500~1400mV, 50mV/step,2steps
    PMU.setDLDO2Voltage(3300);


    // PMU.enableDC1();
    PMU.enableDC2();
    PMU.enableDC3();
    PMU.enableDC4();
    PMU.enableDC5();
    PMU.enableALDO1();
    PMU.enableALDO2();
    PMU.enableALDO3();
    PMU.enableALDO4();
    PMU.enableBLDO1();
    PMU.enableBLDO2();
    PMU.enableCPUSLDO();
    PMU.enableDLDO1();
    PMU.enableDLDO2();


    Serial.println("DCDC=======================================================================");
    Serial.printf("DC1  :%s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "ENABLE" : "DISABLE", PMU.getDC1Voltage());
    Serial.printf("DC2  :%s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "ENABLE" : "DISABLE", PMU.getDC2Voltage());
    Serial.printf("DC3  :%s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", PMU.getDC3Voltage());
    Serial.printf("DC4  :%s   Voltage:%u mV \n",  PMU.isEnableDC4()  ? "ENABLE" : "DISABLE", PMU.getDC4Voltage());
    Serial.printf("DC5  :%s   Voltage:%u mV \n",  PMU.isEnableDC5()  ? "ENABLE" : "DISABLE", PMU.getDC5Voltage());
    Serial.println("ALDO=======================================================================");
    Serial.printf("ALDO1:%s   Voltage:%u mV\n",  PMU.isEnableALDO1()  ? "ENABLE" : "DISABLE", PMU.getALDO1Voltage());
    Serial.printf("ALDO2:%s   Voltage:%u mV\n",  PMU.isEnableALDO2()  ? "ENABLE" : "DISABLE", PMU.getALDO2Voltage());
    Serial.printf("ALDO3:%s   Voltage:%u mV\n",  PMU.isEnableALDO3()  ? "ENABLE" : "DISABLE", PMU.getALDO3Voltage());
    Serial.printf("ALDO4:%s   Voltage:%u mV\n",  PMU.isEnableALDO4()  ? "ENABLE" : "DISABLE", PMU.getALDO4Voltage());
    Serial.println("BLDO=======================================================================");
    Serial.printf("BLDO1:%s   Voltage:%u mV\n",  PMU.isEnableBLDO1()  ? "ENABLE" : "DISABLE", PMU.getBLDO1Voltage());
    Serial.printf("BLDO2:%s   Voltage:%u mV\n",  PMU.isEnableBLDO2()  ? "ENABLE" : "DISABLE", PMU.getBLDO2Voltage());
    Serial.println("CPUSLDO====================================================================");
    Serial.printf("CPUSLDO:%s Voltage:%u mV\n",  PMU.isEnableCPUSLDO() ? "ENABLE" : "DISABLE", PMU.getCPUSLDOVoltage());
    Serial.println("DLDO=======================================================================");
    Serial.printf("DLDO1:%s   Voltage:%u mV\n",  PMU.isEnableDLDO1()  ? "ENABLE" : "DISABLE", PMU.getDLDO1Voltage());
    Serial.printf("DLDO2:%s   Voltage:%u mV\n",  PMU.isEnableDLDO2()  ? "ENABLE" : "DISABLE", PMU.getDLDO2Voltage());
    Serial.println("===========================================================================");

    // Set the time of pressing the button to turn off
    PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = PMU.getPowerKeyPressOffTime();
    Serial.print("PowerKeyPressOffTime:");
    switch (opt) {
    case XPOWERS_POWEROFF_4S: Serial.println("4 Second");
        break;
    case XPOWERS_POWEROFF_6S: Serial.println("6 Second");
        break;
    case XPOWERS_POWEROFF_8S: Serial.println("8 Second");
        break;
    case XPOWERS_POWEROFF_10S: Serial.println("10 Second");
        break;
    default:
        break;
    }
    // Set the button power-on press time
    PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
    opt = PMU.getPowerKeyPressOnTime();
    Serial.print("PowerKeyPressOnTime:");
    switch (opt) {
    case XPOWERS_POWERON_128MS: Serial.println("128 Ms");
        break;
    case XPOWERS_POWERON_512MS: Serial.println("512 Ms");
        break;
    case XPOWERS_POWERON_1S: Serial.println("1 Second");
        break;
    case XPOWERS_POWERON_2S: Serial.println("2 Second");
        break;
    default:
        break;
    }

    Serial.println("===========================================================================");

    bool en;

    // DCDC 120%(130%) high voltage turn off PMIC function
    en = PMU.getDCHighVoltagePowerDowmEn();
    Serial.print("getDCHighVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");
    // DCDC1 85% low voltage turn off PMIC function
    en = PMU.getDC1LowVoltagePowerDowmEn();
    Serial.print("getDC1LowVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");
    // DCDC2 85% low voltage turn off PMIC function
    en = PMU.getDC2LowVoltagePowerDowmEn();
    Serial.print("getDC2LowVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");
    // DCDC3 85% low voltage turn off PMIC function
    en = PMU.getDC3LowVoltagePowerDowmEn();
    Serial.print("getDC3LowVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");
    // DCDC4 85% low voltage turn off PMIC function
    en = PMU.getDC4LowVoltagePowerDowmEn();
    Serial.print("getDC4LowVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");
    // DCDC5 85% low voltage turn off PMIC function
    en = PMU.getDC5LowVoltagePowerDowmEn();
    Serial.print("getDC5LowVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");

    // PMU.setDCHighVoltagePowerDowm(true);
    // PMU.setDC1LowVoltagePowerDowm(true);
    // PMU.setDC2LowVoltagePowerDowm(true);
    // PMU.setDC3LowVoltagePowerDowm(true);
    // PMU.setDC4LowVoltagePowerDowm(true);
    // PMU.setDC5LowVoltagePowerDowm(true);

    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU.disableTSPinMeasure();

    // PMU.enableTemperatureMeasure();

    // Enable internal ADC detection
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();


    // Manual control CHGLED
    // PMU.setChargerLedFunction(XPOWER_CHGLED_MANUAL);
    // PMU.setChargingLedFreq(XPOWERS_CHG_LED_FRE_4HZ);


    // The default setting is TypeA mode, and the CHGLED is automatically controlled by the PMU.
    PMU.setChargerLedFunction(XPOWER_CHGLED_TYPEA);


    PMU.enableChargingLed();
    // PMU.disableChargingLed();


    pinMode(pmu_irq_pin, INPUT);
    attachInterrupt(pmu_irq_pin, setFlag, FALLING);


    // Disable all interrupts
    PMU.disableIRQ(XPOWERS_ALL_IRQ);
    // Clear all interrupt flags
    PMU.clearIrqStatus();
    // Enable the required interrupt function
    PMU.enableIRQ(
        XPOWERS_BAT_INSERT_IRQ    | XPOWERS_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_VBUS_INSERT_IRQ   | XPOWERS_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_PKEY_SHORT_IRQ    | XPOWERS_PKEY_LONG_IRQ       |   //POWER KEY
        XPOWERS_BAT_CHG_DONE_IRQ  | XPOWERS_BAT_CHG_START_IRQ       //CHARGE
        // XPOWERS_PKEY_NEGATIVE_IRQ | XPOWERS_PKEY_POSITIVE_IRQ   |   //POWER KEY
    );

    // Set the precharge charging current
    PMU.setPrechargeCurr(XPOWERS_PRECHARGE_50MA);
    // Set constant current charge current limit
    PMU.setChargerConstantCurr(XPOWERS_ICC_CHG_200MA);
    // Set stop charging termination current
    PMU.setChargerTerminationCurr(XPOWERS_CHG_ITERM_25MA);

    // Set charge cut-off voltage
    PMU.setChargerVoltageLimit(XPOWERS_CHG_VOL_4V1);

    // Set the watchdog trigger event type
    PMU.setWatchdogConfig(XPOWERS_WDT_IRQ_TO_PIN);
    // Set watchdog timeout
    PMU.setWatchdogTimeout(XPOWERS_WDT_TIMEOUT_4S);
    // Enable watchdog to trigger interrupt event
    PMU.enableWatchdog();

    // PMU.disableWatchdog();

}

void printPMU()
{
    Serial.print("isCharging:"); Serial.println(PMU.isCharging() ? "YES" : "NO");
    Serial.print("isDischarge:"); Serial.println(PMU.isDischarge() ? "YES" : "NO");
    Serial.print("isStandby:"); Serial.println(PMU.isStandby() ? "YES" : "NO");
    Serial.print("isVbusIn:"); Serial.println(PMU.isVbusIn() ? "YES" : "NO");
    Serial.print("isVbusGood:"); Serial.println(PMU.isVbusGood() ? "YES" : "NO");
    Serial.print("getChargerStatus:");
    uint8_t charge_status = PMU.getChargerStatus();
    if (charge_status == XPOWERS_CHG_TRI_STATE) {
        Serial.println("tri_charge");
    } else if (charge_status == XPOWERS_CHG_PRE_STATE) {
        Serial.println("pre_charge");
    } else if (charge_status == XPOWERS_CHG_CC_STATE) {
        Serial.println("constant charge");
    } else if (charge_status == XPOWERS_CHG_CV_STATE) {
        Serial.println("constant voltage");
    } else if (charge_status == XPOWERS_CHG_DONE_STATE) {
        Serial.println("charge done");
    } else if (charge_status == XPOWERS_CHG_STOP_STATE) {
        Serial.println("not chargin");
    }

    Serial.print("getBattVoltage:"); Serial.print(PMU.getBattVoltage()); Serial.println("mV");
    Serial.print("getVbusVoltage:"); Serial.print(PMU.getVbusVoltage()); Serial.println("mV");
    Serial.print("getSystemVoltage:"); Serial.print(PMU.getSystemVoltage()); Serial.println("mV");

    // The battery percentage may be inaccurate at first use, the PMU will automatically
    // learn the battery curve and will automatically calibrate the battery percentage
    // after a charge and discharge cycle
    if (PMU.isBatteryConnect()) {
        Serial.print("getBatteryPercent:"); Serial.print(PMU.getBatteryPercent()); Serial.println("%");
    }

    Serial.println();
}



void enterPmuSleep(void)
{
    // Set the wake-up source to PWRKEY
    PMU.wakeupControl(XPOWERS_WAKEUP_IRQ_PIN_TO_LOW, true);

    // Set sleep flag
    PMU.enableSleep();

    PMU.disableDC2();
    PMU.disableDC3();
    PMU.disableDC4();
    PMU.disableDC5();

    PMU.disableALDO1();
    PMU.disableALDO2();
    PMU.disableALDO3();
    PMU.disableALDO4();

    PMU.disableBLDO1();
    PMU.disableBLDO2();

    PMU.disableCPUSLDO();
    PMU.disableDLDO1();
    PMU.disableDLDO2();

    // Finally, turn off the power of the control chip
    PMU.disableDC1();
}

void loop()
{

    if (pmu_flag) {

        pmu_flag = false;

        // Get PMU Interrupt Status Register
        uint32_t status = PMU.getIrqStatus();
        Serial.print("STATUS => HEX:");
        Serial.print(status, HEX);
        Serial.print(" BIN:");
        Serial.println(status, BIN);

        if (PMU.isDropWarningLevel2Irq()) {
            Serial.println("isDropWarningLevel2");
        }
        if (PMU.isDropWarningLevel1Irq()) {
            Serial.println("isDropWarningLevel1");
        }
        if (PMU.isGaugeWdtTimeoutIrq()) {
            Serial.println("isWdtTimeout");
        }
        if (PMU.isBatChargerOverTemperatureIrq()) {
            Serial.println("isBatChargeOverTemperature");
        }
        if (PMU.isBatWorkOverTemperatureIrq()) {
            Serial.println("isBatWorkOverTemperature");
        }
        if (PMU.isBatWorkUnderTemperatureIrq()) {
            Serial.println("isBatWorkUnderTemperature");
        }
        if (PMU.isVbusInsertIrq()) {
            Serial.println("isVbusInsert");
        }
        if (PMU.isVbusRemoveIrq()) {
            Serial.println("isVbusRemove");
        }
        if (PMU.isBatInsertIrq()) {
            Serial.println("isBatInsert");
        }
        if (PMU.isBatRemoveIrq()) {
            Serial.println("isBatRemove");
        }

        if (PMU.isPekeyShortPressIrq()) {
            Serial.println("isPekeyShortPress");
            // enterPmuSleep();

            Serial.print("Read pmu data buffer .");
            uint8_t data[4] = {0};
            PMU.readDataBuffer(data, XPOWERS_DATA_BUFFER_SIZE);
            for (int i = 0; i < 4; ++i) {
                Serial.print(data[i]);
                Serial.print(",");
            }
            Serial.println();
        }

        if (PMU.isPekeyLongPressIrq()) {
            Serial.println("isPekeyLongPress");
            Serial.println("write pmu data buffer .");
            uint8_t data[4] = {1, 2, 3, 4};
            PMU.writeDataBuffer(data, XPOWERS_DATA_BUFFER_SIZE);
        }

        if (PMU.isPekeyNegativeIrq()) {
            Serial.println("isPekeyNegative");
        }
        if (PMU.isPekeyPositiveIrq()) {
            Serial.println("isPekeyPositive");
        }
        if (PMU.isWdtExpireIrq()) {
            Serial.println("isWdtExpire");
            printPMU();
        }
        if (PMU.isLdoOverCurrentIrq()) {
            Serial.println("isLdoOverCurrentIrq");
        }
        if (PMU.isBatfetOverCurrentIrq()) {
            Serial.println("isBatfetOverCurrentIrq");
        }
        if (PMU.isBatChagerDoneIrq()) {
            Serial.println("isBatChagerDone");
        }
        if (PMU.isBatChagerStartIrq()) {
            Serial.println("isBatChagerStart");
        }
        if (PMU.isBatDieOverTemperatureIrq()) {
            Serial.println("isBatDieOverTemperature");
        }
        if (PMU.isChagerOverTimeoutIrq()) {
            Serial.println("isChagerOverTimeout");
        }
        if (PMU.isBatOverVoltageIrq()) {
            Serial.println("isBatOverVoltage");
        }

        // Clear PMU Interrupt Status Register
        PMU.clearIrqStatus();

    }
    delay(10);
}

