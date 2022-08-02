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

// Defined using AXP2102
#define XPOWERS_CHIP_AXP2102

#include <Wire.h>
#include <Arduino.h>
#include "XPowersLib.h"

XPowersPMU PMU;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
uint16_t targetVol;
uint16_t vol = 0;


void setup()
{
    Serial.begin(115200);

    bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, i2c_sda, i2c_scl);

    if (result == false) {
        Serial.println("PMU is not online..."); while (1)delay(50);
    }

    Serial.println("AXP2101 Power Output Test.");



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


    // DC1 IMAX=2A
    // 1500~3400mV,100mV/step,20steps
    vol = 1500;
    for (int i = 0; i < 20; ++i) {
        PMU.setDC1Voltage(vol);
        vol += 100;
        Serial.printf("DC1  :%s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "ENABLE" : "DISABLE", PMU.getDC1Voltage());
    }


    // DC2 IMAX=2A
    // 500~1200mV  10mV/step,71steps
    vol = 500;
    for (int i = 0; i < 71; ++i) {
        PMU.setDC2Voltage(vol);
        delay(1);
        targetVol = PMU.getDC2Voltage();
        Serial.printf("[%d]DC2  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC2()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 10;
    }

    // DC2 IMAX=2A
    // 1220~1540mV 20mV/step,17steps
    vol = 1220;
    for (int i = 0; i < 17; ++i) {
        PMU.setDC2Voltage(vol);
        delay(1);
        targetVol = PMU.getDC2Voltage();
        Serial.printf("[%u]DC2  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC2()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 20;
    }

    // DC3 IMAX = 2A
    // 500~1200mV,10mV/step,71steps
    vol = 500;
    for (int i = 0; i < 71; ++i) {
        PMU.setDC3Voltage(vol);
        delay(1);
        targetVol = PMU.getDC3Voltage();
        Serial.printf("[%u]DC3  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 10;
    }

    // DC3 IMAX = 2A
    // 1220~1540mV,20mV/step,17steps
    vol = 1220;
    for (int i = 0; i < 17; ++i) {
        PMU.setDC3Voltage(vol);
        delay(1);
        targetVol = PMU.getDC3Voltage();
        Serial.printf("[%u]DC3  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 20;
    }


    // DC3 IMAX = 2A
    // 1600~3400mV,100mV/step,19steps
    vol = 1600;
    for (int i = 0; i < 19; ++i) {
        PMU.setDC3Voltage(vol);
        delay(1);
        targetVol = PMU.getDC3Voltage();
        Serial.printf("[%u]DC3  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC3()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }


    // DCDC4 IMAX=1.5A
    // 500~1200mV,10mV/step,71steps
    vol = 500;
    for (int i = 0; i < 71; ++i) {
        PMU.setDC4Voltage(vol);
        delay(1);
        targetVol = PMU.getDC4Voltage();
        Serial.printf("[%u]DC4  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC4()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 10;
    }

    // DCDC4 IMAX=1.5A
    // 1220~1840mV,20mV/step,32steps
    vol = 1220;
    for (int i = 0; i < 32; ++i) {
        PMU.setDC4Voltage(vol);
        delay(1);
        targetVol = PMU.getDC4Voltage();
        Serial.printf("[%u]DC4  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC4()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 20;
    }

    // DC5 IMAX=2A
    // 1200mV
    PMU.setDC5Voltage(1200);
    targetVol = PMU.getDC5Voltage();
    Serial.printf("[0]DC5  :%s   Voltage:%u mV \n",   PMU.isEnableDC5()  ? "ENABLE" : "DISABLE", targetVol );


    // DC5 IMAX=2A
    // 1400~3700mV,100mV/step,24steps
    vol = 1400;
    for (int i = 0; i < 24; ++i) {
        PMU.setDC5Voltage(vol);
        delay(1);
        targetVol = PMU.getDC5Voltage();
        Serial.printf("[%u]DC5  :%s   Voltage:%u mV \n", i,  PMU.isEnableDC5()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }



    //ALDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    vol = 500;
    for (int i = 0; i < 31; ++i) {
        PMU.setALDO1Voltage(vol);
        delay(1);
        targetVol = PMU.getALDO1Voltage();
        Serial.printf("[%u]ALDO1  :%s   Voltage:%u mV \n", i,  PMU.isEnableALDO1()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }

    //ALDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    vol = 500;
    for (int i = 0; i < 31; ++i) {
        PMU.setALDO2Voltage(vol);
        delay(1);
        targetVol = PMU.getALDO2Voltage();
        Serial.printf("[%u]ALDO2  :%s   Voltage:%u mV \n", i,  PMU.isEnableALDO2()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }

    //ALDO3 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    vol = 500;
    for (int i = 0; i < 31; ++i) {
        PMU.setALDO3Voltage(vol);
        delay(1);
        targetVol = PMU.getALDO3Voltage();
        Serial.printf("[%u]ALDO3  :%s   Voltage:%u mV \n", i,  PMU.isEnableALDO3()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }

    //ALDO4 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    vol = 500;
    for (int i = 0; i < 31; ++i) {
        PMU.setALDO4Voltage(vol);
        delay(1);
        targetVol = PMU.getALDO4Voltage();
        Serial.printf("[%u]ALDO4  :%s   Voltage:%u mV \n", i,  PMU.isEnableALDO4()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }

    //BLDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    vol = 500;
    for (int i = 0; i < 31; ++i) {
        PMU.setBLDO1Voltage(vol);
        delay(1);
        targetVol = PMU.getBLDO1Voltage();
        Serial.printf("[%u]BLDO1  :%s   Voltage:%u mV \n", i,  PMU.isEnableBLDO1()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }

    //BLDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    vol = 500;
    for (int i = 0; i < 31; ++i) {
        PMU.setBLDO2Voltage(vol);
        delay(1);
        targetVol = PMU.getBLDO2Voltage();
        Serial.printf("[%u]BLDO2  :%s   Voltage:%u mV \n", i,  PMU.isEnableBLDO2()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }


    //CPUSLDO IMAX=30mA
    //500~1400mV,50mV/step,19steps
    vol = 500;
    for (int i = 0; i < 19; ++i) {
        PMU.setCPUSLDOVoltage(vol);
        delay(1);
        targetVol = PMU.getCPUSLDOVoltage();
        Serial.printf("[%u]CPUSLDO  :%s   Voltage:%u mV \n", i,  PMU.isEnableCPUSLDO()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 50;
    }

    //DLDO1 IMAX=300mA
    //500~3400mV, 100mV/step,29steps
    vol = 500;
    for (int i = 0; i < 29; ++i) {
        PMU.setDLDO1Voltage(vol);
        delay(1);
        targetVol = PMU.getDLDO1Voltage();
        Serial.printf("[%u]DLDO1  :%s   Voltage:%u mV \n", i,  PMU.isEnableDLDO1()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }

    //DLDO2 IMAX=300mA
    //500~1400mV, 50mV/step,2steps
    vol = 500;
    for (int i = 0; i < 29; ++i) {
        PMU.setDLDO2Voltage(vol);
        delay(1);
        targetVol = PMU.getDLDO2Voltage();
        Serial.printf("[%u]DLDO2  :%s   Voltage:%u mV \n", i,  PMU.isEnableDLDO2()  ? "ENABLE" : "DISABLE", targetVol );
        if (targetVol != vol)Serial.println(">>> FAILED!");
        vol += 100;
    }


    /*
    ! WARN:
    Please do not run the example without knowing the external load voltage of the PMU,
    it may burn your external load, please check the voltage setting before running the example,
    if there is any loss, please bear it by yourself
    */

    // PMU.enableDC1();
    // PMU.enableDC2();
    // PMU.enableDC3();
    // PMU.enableDC4();
    // PMU.enableDC5();

    // PMU.enableALDO1();
    // PMU.enableALDO2();
    // PMU.enableALDO3();
    // PMU.enableALDO4();


    // PMU.enableBLDO1();
    // PMU.enableBLDO2();

    // PMU.enableCPUSLDO();

    // PMU.enableDLDO1();
    // PMU.enableDLDO2();


}

void loop()
{
    delay(10);
}

