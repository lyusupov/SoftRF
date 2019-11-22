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
uint8_t second = 0;
uint32_t timer = 0;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t axp_irq_pin = 35;

void setup()
{
    Serial.begin(115200);
    Wire.begin(i2c_sda, i2c_scl);

    int ret = axp.begin(Wire);

    if (ret == AXP_FAIL) {
        Serial.println("AXP Power begin failed");
        while (1);
    }

    //! enable timer irq  and pek key press irq channel
    axp.enableIRQ(AXP202_TIMER_TIMEOUT_IRQ | AXP202_PEK_SHORTPRESS_IRQ, true);

    //! attachInterrupt to gpio 35
    pinMode(axp_irq_pin, INPUT);
    attachInterrupt(axp_irq_pin, [] {
        axpIrq = 1;
    }, FALLING);
    axp.clearIRQ();

    axp.setTimer(1);

}


void loop()
{
    if (millis() - timer > 1000) {
        second++;
        Serial.printf("%u second\n", second);
        timer = millis();
    }
    if (axpIrq) {
        axpIrq = 0;
        axp.readIRQ();
        if (axp.isPEKShortPressIRQ()) {
            Serial.printf("AXP202 PEK key Click\n");
        }
        if (axp.isTimerTimeoutIRQ()) {
            Serial.printf("AXP202 timer timeout\n");
            axp.clearTimerStatus();
            while (1) {
                delay(100);
            }
        }
        axp.clearIRQ();
    }
}

