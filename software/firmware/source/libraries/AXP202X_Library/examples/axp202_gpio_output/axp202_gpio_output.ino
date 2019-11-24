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

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;

void setup()
{
    Serial.begin(115200);
    Wire.begin(i2c_sda, i2c_scl);

    int ret = axp.begin(Wire);

    if (ret == AXP_FAIL) {
        Serial.println("AXP Power begin failed");
        while (1);
    }
    
    axp.setGPIOMode(AXP_GPIO_0, AXP_IO_OUTPUT_HIGH_MODE);
    axp.setGPIOMode(AXP_GPIO_1, AXP_IO_OUTPUT_HIGH_MODE);

    //! GPIO2, GPIO3 is only allowed to be configured to output low
    axp.gpioWrite(AXP_GPIO_2, LOW);
    axp.gpioWrite(AXP_GPIO_3, LOW);

}

void loop()
{
    //! GPIO0, GPIO1 allows output high and low
    axp.gpioWrite(AXP_GPIO_0, HIGH);
    axp.gpioWrite(AXP_GPIO_1, HIGH);
    delay(1000);
    axp.gpioWrite(AXP_GPIO_0, LOW);
    axp.gpioWrite(AXP_GPIO_1, LOW);
    delay(1000);
}

