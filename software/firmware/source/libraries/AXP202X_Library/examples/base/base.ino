/*
MIT License

Copyright (c) 2019 lewis he

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
#include <Wire.h>
#include <axp20x.h>


AXP20X_Class axp;
bool  axpIrq = 0;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t axp_irq_pin = 35;


uint8_t readBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t ret = 0;
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    uint8_t cnt = Wire.requestFrom(addr, (uint8_t)len, (uint8_t)1);
    if (!cnt) {
        ret =  0xFF;
    }
    uint16_t index = 0;
    while (Wire.available()) {
        if (index > len)return 0xFF;
        data[index++] = Wire.read();
    }
    return ret;
}

uint8_t writeBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t ret = 0;
    Wire.beginTransmission(addr);
    Wire.write(reg);
    for (uint16_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    ret =  Wire.endTransmission();
    return ret ? ret : 0xFF;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(i2c_sda, i2c_scl);

    //! Use the Wire port
    // int ret = axp.begin(Wire);

    //! Use custom I2C to return processing
    int ret = axp.begin(readBytes, writeBytes);

    if (ret == AXP_FAIL) {
        Serial.println("AXP Power begin failed");
        while (1);
    }

    //! enable all irq channel
    axp.enableIRQ(0xFFFFFFFF, true);

    //! attachInterrupt to gpio 35
    pinMode(axp_irq_pin, INPUT_PULLUP);
    attachInterrupt(axp_irq_pin, [] {
        axpIrq = 1;
    }, FALLING);
    axp.clearIRQ();

    axp.setPowerOutPut(AXP202_DCDC3, AXP202_ON);
    axp.setPowerOutPut(AXP202_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP202_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP202_LDO4, AXP202_ON);
    axp.setPowerOutPut(AXP202_DCDC2, AXP202_ON);
    axp.setLDO4Voltage(AXP202_LDO4_1800MV);
    axp.setLDO3Voltage(3500);
    axp.setPowerOutPut(AXP202_LDO3, AXP202_ON);

    Serial.print("DC2:");
    Serial.print(axp.isDCDC2Enable() ? String(axp.getDCDC2Voltage()) + " mv" : "DISABLE");
    Serial.print("  ");

    Serial.print("DC3:");
    Serial.print(axp.isDCDC3Enable() ? String(axp.getDCDC3Voltage()) + " mv" : "DISABLE");
    Serial.print("  ");

    Serial.print("LDO2:");
    Serial.print(axp.isLDO2Enable() ? String(axp.getLDO2Voltage()) + " mv" : "DISABLE");
    Serial.print("  ");

    Serial.print("LDO3:");
    Serial.print(axp.isLDO3Enable() ? String(axp.getLDO3Voltage()) + " mv" : "DISABLE");
    Serial.print("  ");

    Serial.print("LDO4:");
    Serial.print(axp.isLDO4Enable() ? "ENABLE" : "DISABLE");
    Serial.print("  ");

    Serial.print("Exten:");
    Serial.print(axp.isExtenEnable() ? "ENABLE" : "DISABLE");
    Serial.print("\r\n");

}


void loop()
{
    if (axpIrq) {
        axpIrq = 0;
        axp.readIRQ();
        if (axp.isPEKShortPressIRQ()) {
            Serial.printf("AXP202 PEK key Click\n");
        }
        axp.clearIRQ();
    }
}

