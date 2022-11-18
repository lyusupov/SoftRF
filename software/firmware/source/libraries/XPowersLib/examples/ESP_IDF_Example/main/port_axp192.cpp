#include <stdio.h>
#include <cstring>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#ifdef CONFIG_XPOWERS_CHIP_AXP192

#define XPOWERS_CHIP_AXP192
#include "XPowersLib.h"
static const char *TAG = "AXP192";

XPowersPMU PMU;

extern int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);
extern int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);


esp_err_t pmu_init()
{
    if (PMU.begin(AXP192_SLAVE_ADDRESS, pmu_register_read, pmu_register_write_byte)) {
        ESP_LOGI(TAG, "Init PMU SUCCESS!");
    } else {
        ESP_LOGE(TAG, "Init PMU FAILED!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "getID:0x%x", PMU.getChipID());

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
    ESP_LOGI(TAG, "DC1  :%s   Voltage:%u mV ",  PMU.isEnableDC1()  ? "ENABLE" : "DISABLE", PMU.getDC1Voltage());

    // DC2 700~2750mV, IMAX=1.6A;
    PMU.setDC2Voltage(700);
    ESP_LOGI(TAG, "DC2  :%s   Voltage:%u mV ",  PMU.isEnableDC2()  ? "ENABLE" : "DISABLE", PMU.getDC2Voltage());

    // DC3 700~3500mV,IMAX=0.7A;
    PMU.setDC3Voltage(3300);
    ESP_LOGI(TAG, "DC3  :%s   Voltage:%u mV ",  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", PMU.getDC3Voltage());


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

    ESP_LOGI(TAG, "DCDC=======================================================================\n");
    ESP_LOGI(TAG, "DC1  :%s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "ENABLE" : "DISABLE", PMU.getDC1Voltage());
    ESP_LOGI(TAG, "DC2  :%s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "ENABLE" : "DISABLE", PMU.getDC2Voltage());
    ESP_LOGI(TAG, "DC3  :%s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", PMU.getDC3Voltage());
    ESP_LOGI(TAG, "LDO=======================================================================\n");
    ESP_LOGI(TAG, "LDO2: %s   Voltage:%u mV\n",  PMU.isEnableLDO2()  ? "ENABLE" : "DISABLE", PMU.getLDO2Voltage());
    ESP_LOGI(TAG, "LDO3: %s   Voltage:%u mV\n",  PMU.isEnableLDO3()  ? "ENABLE" : "DISABLE", PMU.getLDO3Voltage());
    ESP_LOGI(TAG, "LDOio: %s   Voltage:%u mV\n",  PMU.isEnableLDOio()  ? "ENABLE" : "DISABLE", PMU.getLDOioVoltage());
    ESP_LOGI(TAG, "==========================================================================\n");

    // Set the time of pressing the button to turn off
    PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = PMU.getPowerKeyPressOffTime();
    ESP_LOGI(TAG, "PowerKeyPressOffTime:");
    switch (opt) {
    case XPOWERS_POWEROFF_4S: ESP_LOGI(TAG, "4 Second");
        break;
    case XPOWERS_POWEROFF_6S: ESP_LOGI(TAG, "6 Second");
        break;
    case XPOWERS_POWEROFF_8S: ESP_LOGI(TAG, "8 Second");
        break;
    case XPOWERS_POWEROFF_10S: ESP_LOGI(TAG, "10 Second");
        break;
    default:
        break;
    }
    // Set the button power-on press time
    PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
    opt = PMU.getPowerKeyPressOnTime();
    ESP_LOGI(TAG, "PowerKeyPressOnTime:");
    switch (opt) {
    case XPOWERS_POWERON_128MS: ESP_LOGI(TAG, "128 Ms");
        break;
    case XPOWERS_POWERON_512MS: ESP_LOGI(TAG, "512 Ms");
        break;
    case XPOWERS_POWERON_1S: ESP_LOGI(TAG, "1 Second");
        break;
    case XPOWERS_POWERON_2S: ESP_LOGI(TAG, "2 Second");
        break;
    default:
        break;
    }

    ESP_LOGI(TAG, "===========================================================================");

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
    ESP_LOGI(TAG, "Write pmu data buffer .");
    uint8_t data[XPOWERS_AXP192_DATA_BUFFER_SIZE] = {1, 2, 3, 4, 5, 6};
    PMU.writeDataBuffer(data, XPOWERS_AXP192_DATA_BUFFER_SIZE);
    memset(data, 0, XPOWERS_AXP192_DATA_BUFFER_SIZE);

    ESP_LOGI(TAG, "Read pmu data buffer :");
    PMU.readDataBuffer(data, XPOWERS_AXP192_DATA_BUFFER_SIZE);
    ESP_LOG_BUFFER_HEX(TAG, data, XPOWERS_AXP192_DATA_BUFFER_SIZE);

    // Set the timing after one minute, the isWdtExpireIrq will be triggered in the loop interrupt function
    PMU.setTimerout(1);

    return ESP_OK;
}


void pmu_isr_handler()
{
    // Get PMU Interrupt Status Register
    PMU.getIrqStatus();

    if (PMU.isAcinOverVoltageIrq()) {
        ESP_LOGI(TAG, "isAcinOverVoltageIrq");
    }
    if (PMU.isAcinInserIrq()) {
        ESP_LOGI(TAG, "isAcinInserIrq");
    }
    if (PMU.isAcinRemoveIrq()) {
        ESP_LOGI(TAG, "isAcinRemoveIrq");
    }
    if (PMU.isVbusOverVoltageIrq()) {
        ESP_LOGI(TAG, "isVbusOverVoltageIrq");
    }
    if (PMU.isVbusInsertIrq()) {
        ESP_LOGI(TAG, "isVbusInsertIrq");
    }
    if (PMU.isVbusRemoveIrq()) {
        ESP_LOGI(TAG, "isVbusRemoveIrq");
    }
    if (PMU.isVbusLowVholdIrq()) {
        ESP_LOGI(TAG, "isVbusLowVholdIrq");
    }
    if (PMU.isBatInsertIrq()) {
        ESP_LOGI(TAG, "isBatInsertIrq");
    }
    if (PMU.isBatRemoveIrq()) {
        ESP_LOGI(TAG, "isBatRemoveIrq");
    }
    if (PMU.isBattEnterActivateIrq()) {
        ESP_LOGI(TAG, "isBattEnterActivateIrq");
    }
    if (PMU.isBattExitActivateIrq()) {
        ESP_LOGI(TAG, "isBattExitActivateIrq");
    }
    if (PMU.isBatChagerStartIrq()) {
        ESP_LOGI(TAG, "isBatChagerStartIrq");
    }
    if (PMU.isBatChagerDoneIrq()) {
        ESP_LOGI(TAG, "isBatChagerDoneIrq");
    }
    if (PMU.isBattTempHighIrq()) {
        ESP_LOGI(TAG, "isBattTempHighIrq");
    }
    if (PMU.isBattTempLowIrq()) {
        ESP_LOGI(TAG, "isBattTempLowIrq");
    }
    if (PMU.isChipOverTemperatureIrq()) {
        ESP_LOGI(TAG, "isChipOverTemperatureIrq");
    }
    if (PMU.isChargingCurrentLessIrq()) {
        ESP_LOGI(TAG, "isChargingCurrentLessIrq");
    }
    if (PMU.isDC1VoltageLessIrq()) {
        ESP_LOGI(TAG, "isDC1VoltageLessIrq");
    }
    if (PMU.isDC2VoltageLessIrq()) {
        ESP_LOGI(TAG, "isDC2VoltageLessIrq");
    }
    if (PMU.isDC3VoltageLessIrq()) {
        ESP_LOGI(TAG, "isDC3VoltageLessIrq");
    }
    if (PMU.isPekeyShortPressIrq()) {
        ESP_LOGI(TAG, "isPekeyShortPress");
        // enterPmuSleep();
    }
    if (PMU.isPekeyLongPressIrq()) {
        ESP_LOGI(TAG, "isPekeyLongPress");

    }
    if (PMU.isNOEPowerOnIrq()) {
        ESP_LOGI(TAG, "isNOEPowerOnIrq");
    }
    if (PMU.isNOEPowerDownIrq()) {
        ESP_LOGI(TAG, "isNOEPowerDownIrq");
    }
    if (PMU.isVbusEffectiveIrq()) {
        ESP_LOGI(TAG, "isVbusEffectiveIrq");
    }
    if (PMU.isVbusInvalidIrq()) {
        ESP_LOGI(TAG, "isVbusInvalidIrq");
    }
    if (PMU.isVbusSessionIrq()) {
        ESP_LOGI(TAG, "isVbusSessionIrq");
    }
    if (PMU.isVbusSessionEndIrq()) {
        ESP_LOGI(TAG, "isVbusSessionEndIrq");
    }
    if (PMU.isLowVoltageLevel2Irq()) {
        ESP_LOGI(TAG, "isLowVoltageLevel2Irq");
    }
    if (PMU.isWdtExpireIrq()) {
        ESP_LOGI(TAG, "isWdtExpire");
        // Clear the timer state and continue to the next timer
        PMU.clearTimerFlag();
    }
    if (PMU.isGpio2EdgeTriggerIrq()) {
        ESP_LOGI(TAG, "isGpio2EdgeTriggerIrq");
    }
    if (PMU.isGpio1EdgeTriggerIrq()) {
        ESP_LOGI(TAG, "isGpio1EdgeTriggerIrq");
    }
    if (PMU.isGpio0EdgeTriggerIrq()) {
        ESP_LOGI(TAG, "isGpio0EdgeTriggerIrq");
    }
    // Clear PMU Interrupt Status Register
    PMU.clearIrqStatus();
}
#endif /*CONFIG_XPOWERS_AXP192_CHIP_AXP192*/


