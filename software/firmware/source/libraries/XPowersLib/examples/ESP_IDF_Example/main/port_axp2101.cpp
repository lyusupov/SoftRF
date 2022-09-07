#include <stdio.h>
#include <cstring>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#ifdef CONFIG_XPOWERS_CHIP_AXP2102

#define XPOWERS_CHIP_AXP2102
#include "XPowersLib.h"
static const char *TAG = "AXP2101";

static XPowersPMU PMU;

extern int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);
extern int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);


esp_err_t pmu_init()
{
    if (PMU.begin(AXP2101_SLAVE_ADDRESS, pmu_register_read, pmu_register_write_byte)) {
        ESP_LOGI(TAG, "Init PMU SUCCESS!");
    } else {
        ESP_LOGE(TAG, "Init PMU FAILED!");
        return ESP_FAIL;
    }

    //Turn off not use power channel
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


    //ESP32s3 Core VDD
    PMU.setDC3Voltage(3300);
    PMU.enableDC3();

    //Extern 3.3V VDD
    PMU.setDC1Voltage(3300);
    PMU.enableDC1();

    // CAM DVDD  1500~1800
    PMU.setALDO1Voltage(1800);
    // PMU.setALDO1Voltage(1500);
    PMU.enableALDO1();

    // CAM DVDD 2500~2800
    PMU.setALDO2Voltage(2800);
    PMU.enableALDO2();

    // CAM AVDD 2800~3000
    PMU.setALDO4Voltage(3000);
    PMU.enableALDO4();

    // PIR VDD 3300
    PMU.setALDO3Voltage(3300);
    PMU.enableALDO3();

    // OLED VDD 3300
    PMU.setBLDO1Voltage(3300);
    PMU.enableBLDO1();

    // MIC VDD 33000
    PMU.setBLDO2Voltage(3300);
    PMU.enableBLDO2();

    Serial.println("DCDC=======================================================================");
    Serial.printf("DC1  : %s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "+" : "-", PMU.getDC1Voltage());
    Serial.printf("DC2  : %s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "+" : "-", PMU.getDC2Voltage());
    Serial.printf("DC3  : %s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "+" : "-", PMU.getDC3Voltage());
    Serial.printf("DC4  : %s   Voltage:%u mV \n",  PMU.isEnableDC4()  ? "+" : "-", PMU.getDC4Voltage());
    Serial.printf("DC5  : %s   Voltage:%u mV \n",  PMU.isEnableDC5()  ? "+" : "-", PMU.getDC5Voltage());
    Serial.println("ALDO=======================================================================");
    Serial.printf("ALDO1: %s   Voltage:%u mV\n",  PMU.isEnableALDO1()  ? "+" : "-", PMU.getALDO1Voltage());
    Serial.printf("ALDO2: %s   Voltage:%u mV\n",  PMU.isEnableALDO2()  ? "+" : "-", PMU.getALDO2Voltage());
    Serial.printf("ALDO3: %s   Voltage:%u mV\n",  PMU.isEnableALDO3()  ? "+" : "-", PMU.getALDO3Voltage());
    Serial.printf("ALDO4: %s   Voltage:%u mV\n",  PMU.isEnableALDO4()  ? "+" : "-", PMU.getALDO4Voltage());
    Serial.println("BLDO=======================================================================");
    Serial.printf("BLDO1: %s   Voltage:%u mV\n",  PMU.isEnableBLDO1()  ? "+" : "-", PMU.getBLDO1Voltage());
    Serial.printf("BLDO2: %s   Voltage:%u mV\n",  PMU.isEnableBLDO2()  ? "+" : "-", PMU.getBLDO2Voltage());
    Serial.println("CPUSLDO====================================================================");
    Serial.printf("CPUSLDO: %s Voltage:%u mV\n",  PMU.isEnableCPUSLDO() ? "+" : "-", PMU.getCPUSLDOVoltage());
    Serial.println("DLDO=======================================================================");
    Serial.printf("DLDO1: %s   Voltage:%u mV\n",  PMU.isEnableDLDO1()  ? "+" : "-", PMU.getDLDO1Voltage());
    Serial.printf("DLDO2: %s   Voltage:%u mV\n",  PMU.isEnableDLDO2()  ? "+" : "-", PMU.getDLDO2Voltage());
    Serial.println("===========================================================================");

    PMU.clearIrqStatus();

    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();
    PMU.enableTemperatureMeasure();

    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU.disableTSPinMeasure();

    // Disable all interrupts
    PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Clear all interrupt flags
    PMU.clearIrqStatus();
    // Enable the required interrupt function
    PMU.enableIRQ(
        XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
        // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
    );

    // Set the precharge charging current
    PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    // Set constant current charge current limit
    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    // Set stop charging termination current
    PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

    // Set charge cut-off voltage
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

    // Set the watchdog trigger event type
    // PMU.setWatchdogConfig(XPOWERS_AXP2101_WDT_IRQ_TO_PIN);
    // Set watchdog timeout
    // PMU.setWatchdogTimeout(XPOWERS_AXP2101_WDT_TIMEOUT_4S);
    // Enable watchdog to trigger interrupt event
    // PMU.enableWatchdog();
    return ESP_OK;
}


void pmu_isr_handler()
{
// Get PMU Interrupt Status Register
    PMU.getIrqStatus();

    if (PMU.isDropWarningLevel2Irq()) {
        ESP_LOGI(TAG, "isDropWarningLevel2");
    }
    if (PMU.isDropWarningLevel1Irq()) {
        ESP_LOGI(TAG, "isDropWarningLevel1");
    }
    if (PMU.isGaugeWdtTimeoutIrq()) {
        ESP_LOGI(TAG, "isWdtTimeout");
    }
    if (PMU.isBatChargerOverTemperatureIrq()) {
        ESP_LOGI(TAG, "isBatChargeOverTemperature");
    }
    if (PMU.isBatWorkOverTemperatureIrq()) {
        ESP_LOGI(TAG, "isBatWorkOverTemperature");
    }
    if (PMU.isBatWorkUnderTemperatureIrq()) {
        ESP_LOGI(TAG, "isBatWorkUnderTemperature");
    }
    if (PMU.isVbusInsertIrq()) {
        ESP_LOGI(TAG, "isVbusInsert");
    }
    if (PMU.isVbusRemoveIrq()) {
        ESP_LOGI(TAG, "isVbusRemove");
    }
    if (PMU.isBatInsertIrq()) {
        ESP_LOGI(TAG, "isBatInsert");
    }
    if (PMU.isBatRemoveIrq()) {
        ESP_LOGI(TAG, "isBatRemove");
    }
    if (PMU.isPekeyShortPressIrq()) {
        ESP_LOGI(TAG, "isPekeyShortPress");
    }
    if (PMU.isPekeyLongPressIrq()) {
        ESP_LOGI(TAG, "isPekeyLongPress");
    }
    if (PMU.isPekeyNegativeIrq()) {
        ESP_LOGI(TAG, "isPekeyNegative");
    }
    if (PMU.isPekeyPositiveIrq()) {
        ESP_LOGI(TAG, "isPekeyPositive");
    }
    if (PMU.isWdtExpireIrq()) {
        ESP_LOGI(TAG, "isWdtExpire");
    }
    if (PMU.isLdoOverCurrentIrq()) {
        ESP_LOGI(TAG, "isLdoOverCurrentIrq");
    }
    if (PMU.isBatfetOverCurrentIrq()) {
        ESP_LOGI(TAG, "isBatfetOverCurrentIrq");
    }
    if (PMU.isBatChagerDoneIrq()) {
        ESP_LOGI(TAG, "isBatChagerDone");
    }
    if (PMU.isBatChagerStartIrq()) {
        ESP_LOGI(TAG, "isBatChagerStart");
    }
    if (PMU.isBatDieOverTemperatureIrq()) {
        ESP_LOGI(TAG, "isBatDieOverTemperature");
    }
    if (PMU.isChagerOverTimeoutIrq()) {
        ESP_LOGI(TAG, "isChagerOverTimeout");
    }
    if (PMU.isBatOverVoltageIrq()) {
        ESP_LOGI(TAG, "isBatOverVoltage");
    }
    // Clear PMU Interrupt Status Register
    PMU.clearIrqStatus();
}

#endif  /*CONFIG_XPOWERS_AXP2101_CHIP_AXP2102*/

