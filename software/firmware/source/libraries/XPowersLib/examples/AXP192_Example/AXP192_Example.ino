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

// Defined using AXP192
#define XPOWERS_CHIP_AXP192

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

    bool result = PMU.begin(Wire, AXP192_SLAVE_ADDRESS, i2c_sda, i2c_scl);

    if (result == false) {
        Serial.println("PMU is not online..."); while (1)delay(50);
    }


    Serial.printf("getID:0x%x\n", PMU.getChipID());

    // Set the minimum system operating voltage inside the PMU,
    // below this value will shut down the PMU
    // Range: 2600~3300mV
    PMU.setSysPowerDownVoltage(2700);

    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    PMU.setVbusVoltageLimit(XPOWERS_AXP192_VBUS_VOL_LIM_4V5);

    // Turn off USB input current limit
    PMU.setVbusCurrentLimit(XPOWERS_AXP192_VBUS_CUR_LIM_OFF);

    // DC1 700~3500mV, IMAX=1.2A
    PMU.setDC1Voltage(3300);
    Serial.printf("DC1  :%s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "+" : "-", PMU.getDC1Voltage());

    // DC2 700~2750mV, IMAX=1.6A;
    PMU.setDC2Voltage(700);
    Serial.printf("DC2  :%s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "+" : "-", PMU.getDC2Voltage());

    // DC3 700~3500mV,IMAX=0.7A;
    PMU.setDC3Voltage(3300);
    Serial.printf("DC3  :%s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "+" : "-", PMU.getDC3Voltage());


    //LDO2 1800~3300V, 100mV/step, IMAX=200mA
    PMU.setLDO2Voltage(1800);

    //LDO3 1800~3300V, 100mV/step, IMAX=200mA
    PMU.setLDO3Voltage(1800);

    //LDOio 1800~3300V, 100mV/step, IMAX=50mA
    PMU.setLDOioVoltage(3300);


    // Enable power output channel
    // PMU.enableDC1();
    PMU.enableDC2();
    PMU.enableDC3();
    PMU.enableLDO2();
    PMU.enableLDO3();
    PMU.enableLDOio();

    Serial.println("DCDC=======================================================================");
    Serial.printf("DC1  :%s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "ENABLE" : "DISABLE", PMU.getDC1Voltage());
    Serial.printf("DC2  :%s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "ENABLE" : "DISABLE", PMU.getDC2Voltage());
    Serial.printf("DC3  :%s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", PMU.getDC3Voltage());
    Serial.println("LDO=======================================================================");
    Serial.printf("LDO2: %s   Voltage:%u mV\n",  PMU.isEnableLDO2()  ? "ENABLE" : "DISABLE", PMU.getLDO2Voltage());
    Serial.printf("LDO3: %s   Voltage:%u mV\n",  PMU.isEnableLDO3()  ? "ENABLE" : "DISABLE", PMU.getLDO3Voltage());
    Serial.printf("LDOio: %s   Voltage:%u mV\n",  PMU.isEnableLDOio()  ? "ENABLE" : "DISABLE", PMU.getLDOioVoltage());
    Serial.println("==========================================================================");

    // Set the time of pressing the button to turn off
    PMU.setPowerKeyPressOffTime(XPOWERS_AXP192_POWEROFF_4S);
    uint8_t opt = PMU.getPowerKeyPressOffTime();
    Serial.print("PowerKeyPressOffTime:");
    switch (opt) {
    case XPOWERS_AXP192_POWEROFF_4S: Serial.println("4 Second");
        break;
    case XPOWERS_AXP192_POWEROFF_65: Serial.println("6 Second");
        break;
    case XPOWERS_AXP192_POWEROFF_8S: Serial.println("8 Second");
        break;
    case XPOWERS_AXP192_POWEROFF_10S: Serial.println("10 Second");
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

    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU.disableTSPinMeasure();

    // PMU.enableTemperatureMeasure();
    // PMU.disableTemperatureMeasure();

    // Enable internal ADC detection
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();

    /*
      The default setting is CHGLED is automatically controlled by the PMU.
    - XPOWERS_CHG_LED_OFF,
    - XPOWERS_CHG_LED_BLINK_1HZ,
    - XPOWERS_CHG_LED_BLINK_4HZ,
    - XPOWERS_CHG_LED_ON,
    - XPOWERS_CHG_LED_CTRL_CHG,
    * */
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);


    pinMode(pmu_irq_pin, INPUT);
    attachInterrupt(pmu_irq_pin, setFlag, FALLING);

    // Disable all interrupts
    PMU.disableIRQ(XPOWERS_AXP192_ALL_IRQ);
    // Clear all interrupt flags
    PMU.clearIrqStatus();
    // Enable the required interrupt function
    PMU.enableIRQ(
        XPOWERS_AXP192_BAT_INSERT_IRQ    | XPOWERS_AXP192_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_AXP192_VBUS_INSERT_IRQ   | XPOWERS_AXP192_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP192_PKEY_SHORT_IRQ    | XPOWERS_AXP192_PKEY_LONG_IRQ       |   //POWER KEY
        XPOWERS_AXP192_BAT_CHG_DONE_IRQ  | XPOWERS_AXP192_BAT_CHG_START_IRQ   |    //CHARGE
        // XPOWERS_AXP192_PKEY_NEGATIVE_IRQ | XPOWERS_AXP192_PKEY_POSITIVE_IRQ   |   //POWER KEY
        XPOWERS_AXP192_TIMER_TIMEOUT_IRQ               //Timer
    );

    // Set constant current charge current limit
    PMU.setChargerConstantCurr(XPOWERS_AXP192_CHG_CUR_280MA);
    // Set stop charging termination current
    PMU.setChargerTerminationCurr(XPOWERS_AXP192_CHG_ITERM_LESS_10_PERCENT);

    // Set charge cut-off voltage
    PMU.setChargeTargetVoltage(XPOWERS_AXP192_CHG_VOL_4V2);

    // Cache writes and reads, as long as the PMU remains powered, the data will always be stored inside the PMU
    Serial.println("Write pmu data buffer .");
    uint8_t data[XPOWERS_AXP192_DATA_BUFFER_SIZE] = {1, 2, 3, 4, 5, 6};
    PMU.writeDataBuffer(data, XPOWERS_AXP192_DATA_BUFFER_SIZE);
    memset(data, 0, XPOWERS_AXP192_DATA_BUFFER_SIZE);

    Serial.print("Read pmu data buffer :");
    PMU.readDataBuffer(data, XPOWERS_AXP192_DATA_BUFFER_SIZE);
    for (int i = 0; i < XPOWERS_AXP192_DATA_BUFFER_SIZE; ++i) {
        Serial.print(data[i]);
        Serial.print(",");
    }
    Serial.println();

    // Set the timing after one minute, the isWdtExpireIrq will be triggered in the loop interrupt function
    PMU.setTimerout(1);
}

void printPMU()
{
    Serial.print("isCharging:"); Serial.println(PMU.isCharging() ? "YES" : "NO");
    Serial.print("isDischarge:"); Serial.println(PMU.isDischarge() ? "YES" : "NO");
    Serial.print("isVbusIn:"); Serial.println(PMU.isVbusIn() ? "YES" : "NO");
    Serial.print("getBattVoltage:"); Serial.print(PMU.getBattVoltage()); Serial.println("mV");
    Serial.print("getVbusVoltage:"); Serial.print(PMU.getVbusVoltage()); Serial.println("mV");
    Serial.print("getSystemVoltage:"); Serial.print(PMU.getSystemVoltage()); Serial.println("mV");
    Serial.print("getTemperature:"); Serial.print(PMU.getTemperature()); Serial.println("*C");

    if (PMU.isBatteryConnect()) {
        Serial.print("getBatteryPercent:"); Serial.print(PMU.getBatteryPercent()); Serial.println("%");
    }

    Serial.println();
}



void enterPmuSleep(void)
{
    // Set sleep flag
    PMU.enableSleep();

    PMU.disableDC2();
    PMU.disableDC3();

    PMU.disableLDO2();
    PMU.disableLDO3();

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

        if (PMU.isAcinOverVoltageIrq()) {
            Serial.println("isAcinOverVoltageIrq");
        }
        if (PMU.isAcinInserIrq()) {
            Serial.println("isAcinInserIrq");
        }
        if (PMU.isAcinRemoveIrq()) {
            Serial.println("isAcinRemoveIrq");
        }
        if (PMU.isVbusOverVoltageIrq()) {
            Serial.println("isVbusOverVoltageIrq");
        }
        if (PMU.isVbusInsertIrq()) {
            Serial.println("isVbusInsertIrq");
        }
        if (PMU.isVbusRemoveIrq()) {
            Serial.println("isVbusRemoveIrq");
        }
        if (PMU.isVbusLowVholdIrq()) {
            Serial.println("isVbusLowVholdIrq");
        }
        if (PMU.isBatInsertIrq()) {
            Serial.println("isBatInsertIrq");
        }
        if (PMU.isBatRemoveIrq()) {
            Serial.println("isBatRemoveIrq");
        }
        if (PMU.isBattEnterActivateIrq()) {
            Serial.println("isBattEnterActivateIrq");
        }
        if (PMU.isBattExitActivateIrq()) {
            Serial.println("isBattExitActivateIrq");
        }
        if (PMU.isBatChagerStartIrq()) {
            Serial.println("isBatChagerStartIrq");
        }
        if (PMU.isBatChagerDoneIrq()) {
            Serial.println("isBatChagerDoneIrq");
        }
        if (PMU.isBattTempHighIrq()) {
            Serial.println("isBattTempHighIrq");
        }
        if (PMU.isBattTempLowIrq()) {
            Serial.println("isBattTempLowIrq");
        }
        if (PMU.isChipOverTemperatureIrq()) {
            Serial.println("isChipOverTemperatureIrq");
        }
        if (PMU.isChargingCurrentLessIrq()) {
            Serial.println("isChargingCurrentLessIrq");
        }
        if (PMU.isDC1VoltageLessIrq()) {
            Serial.println("isDC1VoltageLessIrq");
        }
        if (PMU.isDC2VoltageLessIrq()) {
            Serial.println("isDC2VoltageLessIrq");
        }
        if (PMU.isDC3VoltageLessIrq()) {
            Serial.println("isDC3VoltageLessIrq");
        }
        if (PMU.isPekeyShortPressIrq()) {
            Serial.println("isPekeyShortPress");

            // enterPmuSleep();

            //CHG LED mode test
            uint8_t m =  PMU.getChargingLedMode();
            Serial.print("getChargingLedMode:");
            Serial.println(m++);
            m %= XPOWERS_CHG_LED_CTRL_CHG;
            Serial.printf("setChargingLedMode:%u", m);
            PMU.setChargingLedMode(m);

        }
        if (PMU.isPekeyLongPressIrq()) {
            Serial.println("isPekeyLongPress");

        }
        if (PMU.isNOEPowerOnIrq()) {
            Serial.println("isNOEPowerOnIrq");
        }
        if (PMU.isNOEPowerDownIrq()) {
            Serial.println("isNOEPowerDownIrq");
        }
        if (PMU.isVbusEffectiveIrq()) {
            Serial.println("isVbusEffectiveIrq");
        }
        if (PMU.isVbusInvalidIrq()) {
            Serial.println("isVbusInvalidIrq");
        }
        if (PMU.isVbusSessionIrq()) {
            Serial.println("isVbusSessionIrq");
        }
        if (PMU.isVbusSessionEndIrq()) {
            Serial.println("isVbusSessionEndIrq");
        }
        if (PMU.isLowVoltageLevel2Irq()) {
            Serial.println("isLowVoltageLevel2Irq");
        }
        if (PMU.isWdtExpireIrq()) {
            Serial.println("isWdtExpire");

            printPMU();
            // Clear the timer state and continue to the next timer
            PMU.clearTimerFlag();
        }
        if (PMU.isGpio2EdgeTriggerIrq()) {
            Serial.println("isGpio2EdgeTriggerIrq");
        }
        if (PMU.isGpio1EdgeTriggerIrq()) {
            Serial.println("isGpio1EdgeTriggerIrq");
        }
        if (PMU.isGpio0EdgeTriggerIrq()) {
            Serial.println("isGpio0EdgeTriggerIrq");
        }
        // Clear PMU Interrupt Status Register
        PMU.clearIrqStatus();

    }
    delay(10);
}

