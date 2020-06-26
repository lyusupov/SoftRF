#include <Wire.h>
#include <axp20x.h>

AXP20X_Class axp;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;

void setup()
{
    Serial.begin(115200);

    Wire.begin(i2c_sda, i2c_scl);

    // Use AXP202 by default
    // int ret = axp.begin(Wire);

    // Uncomment will use AXP192
    int ret = axp.begin(Wire, AXP192_SLAVE_ADDRESS);

    if (ret == AXP_FAIL) {
        Serial.println("AXP Power begin failed");
        while (1);
    }

}

void loop()
{
    float temp = axp.getTemp();
    Serial.print(temp);
    Serial.println("*C");

    // In T-Beam, T-Watch, and other products using AXP192 and AXP202 in TTGO, TS is not connected.
    // float ts = axp.getTSTemp();
    // Serial.print(ts);
    // Serial.println("*C");

    delay(1000);
}

