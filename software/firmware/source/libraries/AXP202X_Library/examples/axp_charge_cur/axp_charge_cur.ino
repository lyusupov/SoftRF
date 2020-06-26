#include <Wire.h>
#include <axp20x.h>

AXP20X_Class axp;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;

void setup()
{
    Serial.begin(115200);

    delay(3000);

    Wire.begin(i2c_sda, i2c_scl);

    int ret = axp.begin(Wire);
    if (ret == AXP_FAIL) {
        Serial.println("AXP Power begin failed");
        while (1);
    }
    int cur = axp.getChargeControlCur();
    Serial.printf("Current charge control current = %d mA \n", cur);


    //axp202 allows maximum charging current of 1800mA, minimum 300mA
    axp.setChargeControlCur(500);
    Serial.printf("Set charge control current 500 mA \n");

    //When the chip is axp192 / 173, the allowed values are 0 ~ 15, 
    //corresponding to the axp1xx_charge_current_t enumeration
    // axp.setChargeControlCur(AXP1XX_CHARGE_CUR_550MA);
    // Serial.printf("Set charge control current 550 mA \n");

    cur = axp.getChargeControlCur();
    Serial.printf("Current charge control current = %d mA \n", cur);


}

void loop()
{
}

