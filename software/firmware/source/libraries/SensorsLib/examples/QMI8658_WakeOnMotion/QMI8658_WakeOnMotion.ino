/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      QMI8658_WakeOnMotion.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-11-05
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SensorQMI8658.hpp"
#ifdef ARDUINO_T_BEAM_S3_SUPREME
#include <XPowersAXP2101.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
#endif


// #define USE_I2C              //Using the I2C interface

#ifdef USE_I2C
#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#else   /* SPI interface */

#ifndef SPI_MOSI
#define SPI_MOSI   (35)
#endif

#ifndef SPI_SCK
#define SPI_SCK    (36)
#endif

#ifndef SPI_MISO
#define SPI_MISO   (37)
#endif

#ifndef IMU_CS
#define IMU_CS      34      // IMU CS PIN
#endif

#endif  /* USE_I2C*/


#ifndef IMU_IRQ
#define IMU_IRQ     33      // IMU INT PIN
#endif

#ifndef OLED_SDA
#define OLED_SDA    22      // Display Wire SDA Pin
#endif

#ifndef OLED_SCL
#define OLED_SCL    21      // Display Wire SCL Pin
#endif


SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;


bool interruptFlag = false;

void setFlag(void)
{
    interruptFlag = true;

}

void beginPower()
{
    // T_BEAM_S3_SUPREME The PMU voltage needs to be turned on to use the sensor
#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    XPowersAXP2101 power;
    power.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
    power.disableALDO1();
    power.disableALDO2();
    delay(250);
    power.setALDO1Voltage(3300); power.enableALDO1();
    power.setALDO2Voltage(3300); power.enableALDO2();
#endif
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    beginPower();

    bool ret = false;
#ifdef USE_I2C
    ret = qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL);
#else
#if defined(SPI_MOSI) && defined(SPI_SCK) && defined(SPI_MISO)
    ret = qmi.begin(SPI, IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK);
#else
    ret = qmi.begin(SPI, IMU_CS);
#endif
#endif

    if (!ret) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    /* Get chip id*/
    Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);

    // enabling wake on motion low power mode with a threshold of 120 mg and
    // an accelerometer data rate of 128 Hz.
    qmi.configWakeOnMotion();

    /*
    * When the QMI8658 is configured as Wom, the interrupt level is arbitrary,
    * not absolute high or low, and it is in the jump transition state
    */
    pinMode(IMU_IRQ, INPUT_PULLUP);

    attachInterrupt(IMU_IRQ, setFlag, CHANGE);

    // Print register configuration information
    qmi.dumpCtrlRegister();
}


void loop()
{

    if (interruptFlag) {
        interruptFlag = false;
        uint8_t status =  qmi.getStatusRegister();
        Serial.print("STATUS:"); Serial.print(status);
        Serial.print(" BIN:"); Serial.println(status, BIN);

        if (status & SensorQMI8658::EVENT_SIGNIFICANT_MOTION) {
            Serial.println("EVENT_SIGNIFICANT_MOTION");
        } else  if (status & SensorQMI8658::EVENT_NO_MOTION) {
            Serial.println("EVENT_NO_MOTION");
        } else  if (status & SensorQMI8658::EVENT_ANY_MOTION) {
            Serial.println("EVENT_ANY_MOTION");
        } else  if (status & SensorQMI8658::EVENT_PEDOMETER_MOTION) {
            Serial.println("EVENT_PEDOMETER_MOTION");
        } else  if (status & SensorQMI8658::EVENT_WOM_MOTION) {
            Serial.println("EVENT_WOM_MOTION");
        } else  if (status & SensorQMI8658::EVENT_TAP_MOTION) {
            Serial.println("EVENT_TAP_MOTION");
        }
    }
}



