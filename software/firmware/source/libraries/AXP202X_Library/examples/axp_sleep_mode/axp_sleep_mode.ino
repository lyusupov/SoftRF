#include <Wire.h>
#include <axp20x.h>

AXP20X_Class axp;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t irq_pin  = 35;

bool pmu_irq = false;


void sleepPMU()
{
    int ret;
    // PEK or GPIO edge wake-up function enable setting in Sleep mode
    do {
        // In order to ensure that it is set correctly,
        // the loop waits for it to return the correct return value
        Serial.println("Set PMU in sleep mode");
        ret = axp.setSleep();
        delay(500);
    } while (ret != AXP_PASS) ;

    // Turn off all power channels, only use PEK or AXP GPIO to wake up

    // After setting AXP202/AXP192 to sleep,
    // it will start to record the status of the power channel that was turned off after setting,
    // it will restore the previously set state after PEK button or GPIO wake up


    // Turn off all AXP192 power channels
    ret = axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    Serial.printf("Set Power AXP192_LDO2:%s\n", ret == AXP_PASS ? "OK" : "FAIL");

    ret = axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    Serial.printf("Set Power AXP192_LDO3:%s\n", ret == AXP_PASS ? "OK" : "FAIL");

    ret = axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
    Serial.printf("Set Power AXP192_DCDC1:%s\n", ret == AXP_PASS ? "OK" : "FAIL");

    ret = axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    Serial.printf("Set Power AXP192_DCDC2:%s\n", ret == AXP_PASS ? "OK" : "FAIL");

    ret = axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
    Serial.printf("Set Power AXP192_EXTEN:%s\n", ret == AXP_PASS ? "OK" : "FAIL");

    Serial.flush();
    delay(1000);

    // Tbeam v1.0 uses DC3 as the MCU power channel, turning it off as the last
    ret = axp.setPowerOutPut(AXP192_DCDC3, AXP202_OFF);
    Serial.printf("Set Power AXP192_DCDC3:%s\n", ret == AXP_PASS ? "OK" : "FAIL");

    // Turn off all AXP202 power channels
    // axp.setPowerOutPut(AXP202_LDO2, AXP202_OFF);
    // axp.setPowerOutPut(AXP202_LDO3, AXP202_OFF);
    // axp.setPowerOutPut(AXP202_LDO4, AXP202_OFF);
    // axp.setPowerOutPut(AXP202_DCDC2, AXP202_OFF);
    // axp.setPowerOutPut(AXP202_DCDC3, AXP202_OFF);
    // axp.setPowerOutPut(AXP202_EXTEN, AXP202_OFF);


    // If you set the power supply to sleep mode and you turn off the power supply of the MCU,
    // you will not be able to use the wake-up mode provided by the MCU.
    // If you do not turn off the power of the MCU, you can continue to use it
}

void setup()
{
    Serial.begin(115200);

    delay(3000);

    Wire.begin(i2c_sda, i2c_scl);


    // Test with AXP192 ,TBeam v1.0 board
    int ret = axp.begin(Wire, AXP192_SLAVE_ADDRESS);
    if (ret == AXP_FAIL) {
        Serial.println("AXP Power begin failed");
        while (1);
    }

    // Register the PMU interrupt pin, it will be triggered on the falling edge
    pinMode(irq_pin, INPUT);
    attachInterrupt(irq_pin, [] {
        pmu_irq = true;
    }, FALLING);

    // Before using IRQ, remember to clear the IRQ status register
    axp.clearIRQ();

    // Turn on the key to press the interrupt function
    axp.enableIRQ(AXP202_PEK_SHORTPRESS_IRQ, true);


    Serial.println("Wait for the PEK button to be pressed");
}

void loop()
{
    if ( pmu_irq) {
        pmu_irq = false;
        axp.readIRQ();
        // When the PEK button is pressed briefly, the PMU is set to sleep
        if (axp.isPEKShortPressIRQ()) {
            // Clear all interrupt status before going to sleep
            axp.clearIRQ();
            // Set PMU to sleep
            sleepPMU();
        }
    }
}

