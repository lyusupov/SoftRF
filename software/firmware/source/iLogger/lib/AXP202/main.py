'''
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

axp20x.py - MicroPython library for X-Power AXP202 chip.
Created by Lewis he on June 24, 2019.
github:https://github.com/lewisxhe/AXP202X_Libraries
'''
import AXP202.axp202
import time


a = axp202.PMU()
a.setChgLEDMode(axp202.AXP20X_LED_BLINK_1HZ)
a.enablePower(axp202.AXP202_LDO2)
a.setLDO2Voltage(1800)
a.enableADC(axp202.AXP202_ADC1, axp202.AXP202_VBUS_VOL_ADC1)
a.enableADC(axp202.AXP202_ADC1, axp202.AXP202_VBUS_CUR_ADC1)
a.enableADC(axp202.AXP202_ADC1, axp202.AXP202_BATT_VOL_ADC1)
a.enableADC(axp202.AXP202_ADC1, axp202.AXP202_BATT_CUR_ADC1)


while True:
    voltage = a.getVbusVoltage()
    current = a.getVbusCurrent()
    battCurrent = a.getBattChargeCurrent()
    perce = a.getBattPercentage()
    print("isChargeing: V: %f C:%f  BC:%f  perce:%d" % (voltage, current, battCurrent,perce))
    time.sleep(1)
